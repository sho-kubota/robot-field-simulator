(() => {
  "use strict";
  const FIELD_W = 2362, FIELD_H = 1143;
  const ROBOT_W = 231, ROBOT_L = 243, HALF_W = ROBOT_W / 2;
  const TREAD = 191, AXLE_FROM_REAR = 100;
  const FRONT_LEN = ROBOT_L - AXLE_FROM_REAR;
  const ANGLE_SNAP_DEG = 8;
  const NODE_MERGE_MM = 2;
  const WALL_ALIGN_DEG = 15;
  const WALL_BTN_ALIGN_DEG = 45;
  const DEFAULT_POSE = { x: 186.4, y: 956.7, theta: 0 };
  const WALL_EPS_MM = .5;
  const LS_KEY = "wroSimState.v1";

  const $ = (id) => document.getElementById(id);
  const svg = $("fieldSvg");
  const robot = $("robot");
  const courseLayer = $("courseLayer");
  const traceLayer = $("traceLayer");
  const oldLayer = $("oldLayer");
  const measureLayer = $("measureLayer");
  const snapLayer = $("snapLayer");
  const statusEl = $("status");
  const readout = $("readout");
  const filePanel = $("filePanel");
  const snapLineEl = $("snapLine");
  const snapAngleEl = $("snapAngle");
  const pivotToggleEl = $("pivotToggle");
  const showCourseEl = $("showCourse");
  const undoBtn = $("undoBtn");
  const redoBtn = $("redoBtn");
  const copyBtn = $("copyBtn");
  const csvBtn = $("csvBtn");
  const gridToggleEl = $("gridToggle");
  const gridLayer = $("gridLayer");
  const bgEl = $("bgImage");
  const bgToggleEl = $("bgToggle");
  const bgToggleWrap = $("bgToggleWrap");

  // 初期位置設定用要素
  const initXInput = $("initX");
  const initYInput = $("initY");
  const initAngleInput = $("initAngle");
  const setInitBtn = $("setInitBtn");
  const resetInitBtn = $("resetInitBtn");

  let lines = [];
  let nodes = [];
  let initPos = { ...DEFAULT_POSE }; // 設定可能な初期位置
  let pose = { ...initPos };
  let beforePose = { ...pose };
  let history = [{ ...pose, moveCm: 0, signedCm: 0, rot: 0 }];
  let pointer = null;
  let undoStack = [], redoStack = [];
  let csvText = "";
  let saveFailedNoted = false;
  let prevRun = null;
  let runStart = null;

  // ---- 初期位置の管理機能 ----
  function loadInitPos() {
    const saved = localStorage.getItem("wro_sim_init_pos");
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        initPos = {
          x: Number.isFinite(parsed.x) ? parsed.x : DEFAULT_POSE.x,
          y: Number.isFinite(parsed.y) ? parsed.y : DEFAULT_POSE.y,
          theta: Number.isFinite(parsed.theta) ? parsed.theta : (Number.isFinite(parsed.angle) ? parsed.angle : DEFAULT_POSE.theta)
        };
      } catch (e) {
        console.error("初期位置の読み込みエラー", e);
      }
    }
    if (initXInput) initXInput.value = initPos.x;
    if (initYInput) initYInput.value = initPos.y;
    if (initAngleInput) initAngleInput.value = initPos.theta;
  }

  function saveInitPos() {
    localStorage.setItem("wro_sim_init_pos", JSON.stringify(initPos));
  }

  [initXInput, initYInput, initAngleInput].forEach((inputEl) => {
    if (!inputEl) return;
    inputEl.addEventListener("change", () => {
      initPos.x = parseFloat(initXInput.value) || 0;
      initPos.y = parseFloat(initYInput.value) || 0;
      initPos.theta = parseFloat(initAngleInput.value) || 0;
      saveInitPos();
    });
  });

  if (setInitBtn) {
    setInitBtn.addEventListener("click", () => {
      initPos.x = Number(pose.x.toFixed(1));
      initPos.y = Number(pose.y.toFixed(1));
      initPos.theta = Number(pose.theta.toFixed(0));

      if (initXInput) initXInput.value = initPos.x;
      if (initYInput) initYInput.value = initPos.y;
      if (initAngleInput) initAngleInput.value = initPos.theta;

      saveInitPos();
      if (statusEl) statusEl.textContent = `初期位置を (${initPos.x}, ${initPos.y}, ${initPos.theta}°) に登録しました`;
    });
  }

  if (resetInitBtn) {
    resetInitBtn.addEventListener("click", () => resetHome());
  }

  // ---- 表示ビュー管理 ----
  let view = { x: 0, y: 0, w: FIELD_W, h: FIELD_H };
  function applyView() {
    svg.setAttribute("viewBox", `${fmt(view.x)} ${fmt(view.y)} ${fmt(view.w)} ${fmt(view.h)}`);
  }
  function zoomAt(px, py, factor) {
    const w = clamp(view.w * factor, FIELD_W / 40, FIELD_W * 2);
    const k = w / view.w;
    view.x = px - (px - view.x) * k;
    view.y = py - (py - view.y) * k;
    view.w = w;
    view.h = w * FIELD_H / FIELD_W;
    applyView();
  }
  function fitView() {
    view = { x: 0, y: 0, w: FIELD_W, h: FIELD_H };
    applyView();
    statusEl.textContent = "表示をフィールド全体に戻しました。";
  }

  function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }
  function rad(d) { return d * Math.PI / 180; }
  function deg(r) { return r * 180 / Math.PI; }
  function norm(a) { return ((a % 360) + 360) % 360; }
  function deltaAngle(a, b) { return ((((b - a) + 540) % 360) - 180); }
  function unit(theta) { return { x: Math.cos(rad(theta)), y: Math.sin(rad(theta)) }; }
  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
  function fmt(v, d = 1) { return Number(v).toFixed(d); }
  function signedCmText(v) { return `${v > 0 ? "+" : ""}${fmt(v)}cm`; }

  function nearestWallDist(p) {
    return [
      ["左", p.x - extentXB(p.theta)],
      ["右", FIELD_W - p.x - extentXF(p.theta)],
      ["上", p.y - extentYU(p.theta)],
      ["下", FIELD_H - p.y - extentYD(p.theta)],
    ].reduce((a, b) => b[1] < a[1] ? b : a);
  }

  function robotCorners(theta) {
    const r = rad(theta), c = Math.cos(r), s = Math.sin(r);
    return [
      [-AXLE_FROM_REAR, -HALF_W], [FRONT_LEN, -HALF_W],
      [-AXLE_FROM_REAR, HALF_W], [FRONT_LEN, HALF_W],
    ].map(([x, y]) => ({ x: x * c - y * s, y: x * s + y * c }));
  }

  function extentXF(theta) { let m = 0; for (const p of robotCorners(theta)) m = Math.max(m, p.x); return m; }
  function extentXB(theta) { let m = 0; for (const p of robotCorners(theta)) m = Math.max(m, -p.x); return m; }
  function extentYU(theta) { let m = 0; for (const p of robotCorners(theta)) m = Math.max(m, -p.y); return m; }
  function extentYD(theta) { let m = 0; for (const p of robotCorners(theta)) m = Math.max(m, p.y); return m; }

  function clampToWalls(p) {
    const xf = extentXF(p.theta), xb = extentXB(p.theta), yu = extentYU(p.theta), yd = extentYD(p.theta);
    return { ...p, x: clamp(p.x, xb, FIELD_W - xf), y: clamp(p.y, yu, FIELD_H - yd) };
  }

  const WHEEL_LOCAL_X = 0;
  const WHEEL_LOCAL_Y = TREAD / 2;

  // 1. rotatedPose の修正（第3引数 isBackward を追加）
  function rotatedPose(anchor, delta, isBackward = false) {
    const th = norm(anchor.theta + delta);
    if (!pivotToggleEl.checked || Math.abs(delta) < 1e-9) return { x: anchor.x, y: anchor.y, theta: th };
    const r = rad(anchor.theta), c = Math.cos(r), s = Math.sin(r);
    const px = WHEEL_LOCAL_X;

    // 後進時は回転軸となる車輪（py）を反転する
    const py = (delta > 0 ? WHEEL_LOCAL_Y : -WHEEL_LOCAL_Y) * (isBackward ? -1 : 1);
    const pxw = anchor.x + (px * c - py * s);
    const pyw = anchor.y + (px * s + py * c);
    const dr = rad(delta), dc = Math.cos(dr), ds = Math.sin(dr);
    const vx = anchor.x - pxw, vy = anchor.y - pyw;
    return { x: pxw + vx * dc - vy * ds, y: pyw + vx * ds + vy * dc, theta: th };
  }

  // 3. executePivotTurn 関数の修正
  function executePivotTurn(angleDeg, isBackward = false) {
    const nextPose = rotatedPose(pose, angleDeg, isBackward);

    commit(nextPose, `${isBackward ? '後進' : ''}信地旋回(${angleDeg}°)`, {
      moveLocked: false,
      snap: false,
      turn: "one",
      back: isBackward
    });
  }

  function wallContacts(p) {
    const xf = extentXF(p.theta), xb = extentXB(p.theta), yn = extentYU(p.theta), yp = extentYD(p.theta), c = [];
    if (p.x <= xb + WALL_EPS_MM) c.push("左壁");
    if (p.x >= FIELD_W - xf - WALL_EPS_MM) c.push("右壁");
    if (p.y <= yn + WALL_EPS_MM) c.push("上壁");
    if (p.y >= FIELD_H - yp - WALL_EPS_MM) c.push("下壁");
    return c;
  }

  function applyWallDon(oldPose, cand, alignDeg = WALL_ALIGN_DEG) {
    cand = { ...cand, theta: norm(cand.theta) };
    const oldWalls = wallContacts(oldPose);
    const walls = wallContacts(cand);
    const fresh = walls.filter(w => !oldWalls.includes(w));
    if (!fresh.length) return cand;
    const vert = fresh.includes("左壁") || fresh.includes("右壁");
    const horiz = fresh.includes("上壁") || fresh.includes("下壁");
    const cands = [...(vert ? [0, 180] : []), ...(horiz ? [90, 270] : [])];
    let best = null;
    for (const c of cands) {
      const d = Math.abs(deltaAngle(cand.theta, c));
      if (d <= alignDeg && (!best || d < best.d)) best = { c, d };
    }
    if (!best) return { ...cand, wallDon: { walls: fresh, corrected: null } };
    const corrected = deltaAngle(cand.theta, best.c);
    let q = { ...cand, theta: norm(best.c) };
    const xf = extentXF(q.theta), xb = extentXB(q.theta), yp = extentYD(q.theta), yn = extentYU(q.theta);
    if (walls.includes("左壁")) q.x = xb;
    if (walls.includes("右壁")) q.x = FIELD_W - xf;
    if (walls.includes("上壁")) q.y = yn;
    if (walls.includes("下壁")) q.y = FIELD_H - yp;
    return { ...q, wallDon: { walls: fresh, corrected } };
  }

  function maxDriveT(p) {
    const u = unit(p.theta), xf = extentXF(p.theta), xb = extentXB(p.theta), yu = extentYU(p.theta), yd = extentYD(p.theta);
    let lo = -Infinity, hi = Infinity, loWall = "?", hiWall = "?";
    if (Math.abs(u.x) > 1e-9) {
      const a = (xb - p.x) / u.x, b = (FIELD_W - xf - p.x) / u.x;
      if (Math.max(a, b) < hi) { hi = Math.max(a, b); hiWall = u.x > 0 ? "右壁" : "左壁"; }
      if (Math.min(a, b) > lo) { lo = Math.min(a, b); loWall = u.x > 0 ? "左壁" : "右壁"; }
    }
    if (Math.abs(u.y) > 1e-9) {
      const a = (yu - p.y) / u.y, b = (FIELD_H - yd - p.y) / u.y;
      if (Math.max(a, b) < hi) { hi = Math.max(a, b); hiWall = u.y > 0 ? "下壁" : "上壁"; }
      if (Math.min(a, b) > lo) { lo = Math.min(a, b); loWall = u.y > 0 ? "上壁" : "下壁"; }
    }
    return { lo, hi, loWall, hiWall };
  }

  function squareUpInPlace() {
    const walls = wallContacts(pose);
    if (!walls.length) return false;
    const vert = walls.includes("左壁") || walls.includes("右壁");
    const horiz = walls.includes("上壁") || walls.includes("下壁");
    const cands = [...(vert ? [0, 180] : []), ...(horiz ? [90, 270] : [])];
    let best = null;
    for (const c of cands) {
      const d = Math.abs(deltaAngle(pose.theta, c));
      if (d <= WALL_BTN_ALIGN_DEG && (!best || d < best.d)) best = { c, d };
    }
    if (!best) return false;
    const th = norm(best.c);
    let np = { ...pose, theta: th };
    const xf = extentXF(th), xb = extentXB(th), yu = extentYU(th), yd = extentYD(th);
    if (walls.includes("左壁")) np.x = xb;
    if (walls.includes("右壁")) np.x = FIELD_W - xf;
    if (walls.includes("上壁")) np.y = yu;
    if (walls.includes("下壁")) np.y = FIELD_H - yd;
    commit(np, "壁ドン(その場で直角補正)", { moveLocked: false, snap: false });
    return true;
  }

  function driveToWall(sign) {
    const u = unit(pose.theta);
    const t = maxDriveT(pose);
    const d = sign > 0 ? t.hi : t.lo;
    const targetWall = sign > 0 ? t.hiWall : t.loWall;
    if (wallContacts(pose).includes(targetWall)) {
      if (squareUpInPlace()) return;
      statusEl.textContent = `すでに${targetWall}に触れていますが、向きが直角から45°より大きくずれています。回転ハンドルかQ・Eで向きを整えてからもう一度押してください。`;
      return;
    }
    if (!Number.isFinite(d) || Math.abs(d) < 1) {
      statusEl.textContent = "壁まで移動できません。向きを変えて試してください。";
      return;
    }
    commit({ x: pose.x + u.x * d, y: pose.y + u.y * d, theta: pose.theta },
      sign > 0 ? "壁ドン(前)" : "壁ドン(後)",
      { moveLocked: true, snap: false, alignDeg: WALL_BTN_ALIGN_DEG });
  }

  function wallDonDrive(sign) { driveToWall(sign); }

  function el(tag, attrs = {}) {
    const n = document.createElementNS("http://www.w3.org/2000/svg", tag);
    for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v);
    return n;
  }

  function svgPoint(evt) {
    const p = svg.createSVGPoint();
    p.x = evt.clientX; p.y = evt.clientY;
    const m = svg.getScreenCTM();
    if (!m) return { x: p.x, y: p.y };
    return p.matrixTransform(m.inverse());
  }

  function snapTheta(theta) {
    const n = norm(theta);
    if (!snapAngleEl.checked) return norm(Math.round(n * 10) / 10);
    let best = n, bestD = Infinity;
    for (const c of [0, 90, 180, 270, 360]) {
      const d = Math.abs(deltaAngle(n, c));
      if (d < bestD) { bestD = d; best = c % 360; }
    }
    return bestD <= ANGLE_SNAP_DEG ? best : (Math.round(n * 10) / 10) % 360;
  }

  let csvTruncated = false;
  function parseCsv(text) {
    let rows = text.trim().split(/\r?\n/).map(row => row.split(/[\t,]/).map(v => Number(String(v).trim()))).filter(v => v.length >= 4 && v.slice(0, 4).every(Number.isFinite));
    csvTruncated = rows.length > 2000;
    if (csvTruncated) rows = rows.slice(0, 2000);
    return rows.map((v, i) => ({ id: i + 1, x1: v[0], y1: v[1], x2: v[2], y2: v[3] }));
  }
  function truncNote() { return csvTruncated ? " ※行数上限のため先頭2000本のみ" : ""; }

  function nearestPoint(px, py, line) {
    const ax = line.x1, ay = line.y1, bx = line.x2, by = line.y2;
    const dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy || 1;
    const t = clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0, 1);
    const x = ax + t * dx, y = ay + t * dy;
    return { x, y, d: Math.hypot(px - x, py - y), line, t };
  }

  function segmentIntersection(a, b) {
    const x1 = a.x1, y1 = a.y1, x2 = a.x2, y2 = a.y2, x3 = b.x1, y3 = b.y1, x4 = b.x2, y4 = b.y2;
    const den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (Math.abs(den) < 1e-9) return null;
    const px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den;
    const py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (y3 - y4)) / den;
    const within = (p, a, b) => p >= Math.min(a, b) - .5 && p <= Math.max(a, b) + .5;
    if (within(px, x1, x2) && within(py, y1, y2) && within(px, x3, x4) && within(py, y3, y4)) return { x: px, y: py };
    return null;
  }

  function rebuildNodes() {
    const pts = [];
    for (const l of lines) { pts.push({ x: l.x1, y: l.y1, type: "end" }, { x: l.x2, y: l.y2, type: "end" }); }
    for (let i = 0; i < lines.length; i++) for (let j = i + 1; j < lines.length; j++) {
      const p = segmentIntersection(lines[i], lines[j]);
      if (p) pts.push({ x: p.x, y: p.y, type: "junction" });
    }
    const merged = [];
    for (const p of pts) {
      let m = merged.find(q => Math.hypot(q.x - p.x, q.y - p.y) <= NODE_MERGE_MM);
      if (m) {
        m.x = (m.x * m.count + p.x) / (m.count + 1);
        m.y = (m.y * m.count + p.y) / (m.count + 1);
        m.count++;
        if (p.type === "junction") m.type = "junction";
      } else {
        merged.push({ x: p.x, y: p.y, type: p.type, count: 1 });
      }
    }
    nodes = merged;
  }

  function renderCourse() {
    courseLayer.innerHTML = "";
    if (!showCourseEl.checked) return;
    for (const l of lines) {
      courseLayer.appendChild(el("line", { class: "courseLine", x1: l.x1, y1: l.y1, x2: l.x2, y2: l.y2, stroke: "rgba(0,0,0,.70)", "stroke-width": 24, "stroke-linecap": "round" }));
      courseLayer.appendChild(el("line", { class: "courseLine", x1: l.x1, y1: l.y1, x2: l.x2, y2: l.y2, stroke: "rgba(19,185,85,.95)", "stroke-width": 6, "stroke-linecap": "round" }));
    }
    for (const n of nodes.filter(p => p.type === "junction")) {
      courseLayer.appendChild(el("circle", { cx: n.x, cy: n.y, r: 18, fill: "rgba(19,185,85,.26)", stroke: "rgba(19,185,85,.95)", "stroke-width": 4, "pointer-events": "none" }));
    }
  }

  function constrainToDriveAxis(target, anchor) {
    const u = unit(anchor.theta);
    let t = (target.x - anchor.x) * u.x + (target.y - anchor.y) * u.y;
    let lo = -Infinity, hi = Infinity;
    function bounds(coord, dir, min, max) {
      if (Math.abs(dir) < 1e-9) { if (coord < min || coord > max) { lo = 0; hi = 0; } return; }
      let a = (min - coord) / dir, b = (max - coord) / dir;
      if (a > b) [a, b] = [b, a];
      lo = Math.max(lo, a); hi = Math.min(hi, b);
    }
    const xf = extentXF(anchor.theta), xb = extentXB(anchor.theta), yn = extentYU(anchor.theta), yp = extentYD(anchor.theta);
    bounds(anchor.x, u.x, xb, FIELD_W - xf);
    bounds(anchor.y, u.y, yn, FIELD_H - yp);
    if (lo <= hi) t = clamp(t, lo, hi);
    return { x: anchor.x + t * u.x, y: anchor.y + t * u.y, theta: anchor.theta, driveT: t };
  }

  function closestDrivePointToSegment(anchor, desired, line) {
    const u = unit(anchor.theta);
    const vx = line.x2 - line.x1, vy = line.y2 - line.y1;
    const wx = anchor.x - line.x1, wy = anchor.y - line.y1;
    const b = u.x * vx + u.y * vy, c = vx * vx + vy * vy, d = u.x * wx + u.y * wy, e = vx * wx + vy * wy;
    const D = c - b * b;
    let s, t;
    if (c < 1e-9) {
      t = 0; s = (line.x1 - anchor.x) * u.x + (line.y1 - anchor.y) * u.y;
    } else if (Math.abs(D) < 1e-7) {
      const want = (desired.x - anchor.x) * u.x + (desired.y - anchor.y) * u.y;
      const s1 = (line.x1 - anchor.x) * u.x + (line.y1 - anchor.y) * u.y;
      const s2 = (line.x2 - anchor.x) * u.x + (line.y2 - anchor.y) * u.y;
      s = clamp(want, Math.min(s1, s2), Math.max(s1, s2));
      const q = { x: anchor.x + s * u.x, y: anchor.y + s * u.y };
      const n = nearestPoint(q.x, q.y, line);
      return { x: q.x, y: q.y, d: n.d, line, type: "line", desiredD: dist(q, desired) };
    } else {
      s = (b * e - c * d) / D;
      t = (e - b * d) / D;
      if (t < 0) { t = 0; s = (line.x1 - anchor.x) * u.x + (line.y1 - anchor.y) * u.y; }
      else if (t > 1) { t = 1; s = (line.x2 - anchor.x) * u.x + (line.y2 - anchor.y) * u.y; }
    }
    const q = { x: anchor.x + s * u.x, y: anchor.y + s * u.y };
    const r = { x: line.x1 + t * vx, y: line.y1 + t * vy };
    return { x: q.x, y: q.y, d: dist(q, r), line, type: "line", desiredD: dist(q, desired) };
  }

  function nodeDriveCandidate(anchor, desired, node) {
    const u = unit(anchor.theta);
    const s = (node.x - anchor.x) * u.x + (node.y - anchor.y) * u.y;
    const q0 = { x: anchor.x + s * u.x, y: anchor.y + s * u.y };
    const q = constrainToDriveAxis(q0, anchor);
    return { x: q.x, y: q.y, d: Math.hypot(q.x - node.x, q.y - node.y), node, type: node.type, desiredD: dist(q, desired) };
  }

  function snapOnDriveAxis(target, anchor) {
    let pos = constrainToDriveAxis(target, anchor);
    let best = null;
    if (snapLineEl.checked && lines.length) {
      const range = 45;
      for (const l of lines) {
        const c = closestDrivePointToSegment(anchor, pos, l);
        if (c.d <= range && c.desiredD <= Math.max(100, range * 2.4)) {
          const score = c.desiredD + c.d * 3;
          if (!best || score < best.score) best = { ...c, score };
        }
      }
      for (const n of nodes) {
        const c = nodeDriveCandidate(anchor, pos, n);
        const nodeBonus = n.type === "junction" ? 45 : 12;
        if (c.d <= range * 1.15 && c.desiredD <= Math.max(120, range * 2.8)) {
          const score = c.desiredD + c.d * 2 - nodeBonus;
          if (!best || score < best.score) best = { ...c, score };
        }
      }
    }
    if (best) pos = { x: best.x, y: best.y, theta: anchor.theta, driveT: (best.x - anchor.x) * unit(anchor.theta).x + (best.y - anchor.y) * unit(anchor.theta).y };
    return { pose: pos, snap: best };
  }

  function snapFree(p) {
    let pos = clampToWalls({ ...p, theta: snapTheta(p.theta) });
    let best = null;
    if (snapLineEl.checked && lines.length) {
      const range = 45;
      for (const l of lines) { const c = nearestPoint(pos.x, pos.y, l); if (c.d <= range && (!best || c.d < best.score)) best = { ...c, type: "line", score: c.d }; }
      for (const n of nodes) { const d = Math.hypot(pos.x - n.x, pos.y - n.y); const bonus = n.type === "junction" ? 18 : 0; if (d <= range * 1.15 && (!best || d - bonus < best.score)) best = { x: n.x, y: n.y, d, node: n, type: n.type, score: d - bonus }; }
    }
    if (best) { pos = clampToWalls({ ...pos, x: best.x, y: best.y }); }
    return { pose: pos, snap: best };
  }

  function signedForwardCm(from, to) { const u = unit(from.theta); return ((to.x - from.x) * u.x + (to.y - from.y) * u.y) / 10; }

  function wheelTravelCm(dDeg, mode) {
    const dr = rad(dDeg);
    if (mode === "信地") return dr > 0 ? { l: dr * TREAD / 10, r: 0 } : { l: 0, r: -dr * TREAD / 10 };
    return { l: dr * TREAD / 2 / 10, r: -dr * TREAD / 2 / 10 };
  }

  function odometry() {
    let l = 0, r = 0;
    for (const h of history) {
      if (h.turn === "信地") { const t = wheelTravelCm(h.rot ?? 0, "信地"); l += t.l; r += t.r; continue; }
      const fwd = h.signedCm ?? 0;
      const t = wheelTravelCm(h.rot ?? 0, null);
      l += fwd + t.l; r += fwd + t.r;
    }
    return { l, r };
  }

  function robotGhost(p, opacity = .35) {
    return `<g transform="translate(${fmt(p.x)} ${fmt(p.y)}) rotate(${fmt(p.theta)})" opacity="${opacity}">
      <rect x="-100" y="-115.5" width="243" height="231" rx="14" fill="rgba(0,0,0,.08)" stroke="#111" stroke-width="6" stroke-dasharray="18 12"></rect>
      <rect x="75.9" y="-111.5" width="63.1" height="223" rx="10" fill="rgba(255,153,0,.30)" stroke="#ff9900" stroke-width="3"></rect>
      <line x1="0" y1="0" x2="150" y2="0" stroke="#111" stroke-width="10" marker-end="url(#blueArrow)"></line>
    </g>`;
  }

  function renderTrace() {
    traceLayer.innerHTML = "";
    const labels = !pointer;
    if (history.length > 1) {
      traceLayer.appendChild(el("polyline", { points: history.map(h => `${fmt(h.x)},${fmt(h.y)}`).join(" "), fill: "none", stroke: "#006ee6", "stroke-width": 7, "stroke-linecap": "round", "stroke-linejoin": "round", "marker-end": "url(#blueArrow)", opacity: .9, "pointer-events": "none" }));

      for (let i = 1; i < history.length; i++) {
        const a = history[i - 1], b = history[i];
        const d = Math.hypot(b.x - a.x, b.y - a.y);

        if (labels && d >= .5) {
          const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
          const nx = -(b.y - a.y) / d * 32, ny = (b.x - a.x) / d * 32;
          const moveText = el("text", { x: mx + nx, y: my + ny, "text-anchor": "middle", class: "traceLabel" });
          moveText.textContent = signedCmText(b.signedCm ?? b.moveCm ?? 0);
          traceLayer.appendChild(moveText);
        }

        const rot = (typeof b.rot === "number") ? b.rot : deltaAngle(a.theta, b.theta);
        if (labels && Math.abs(rot) >= .05) {
          const radius = 76;
          const path = arcPath(b.x, b.y, radius, a.theta, b.theta);
          if (path) {
            traceLayer.appendChild(el("path", {
              d: path, fill: "none", stroke: "#e23a48", "stroke-width": 8, "stroke-linecap": "round", "marker-end": "url(#arrow)", "pointer-events": "none"
            }));
          }

          const heading = unit(b.theta);
          traceLayer.appendChild(el("line", {
            x1: b.x, y1: b.y, x2: b.x + heading.x * 108, y2: b.y + heading.y * 108,
            stroke: "#e23a48", "stroke-width": 8, "stroke-linecap": "round", "pointer-events": "none"
          }));

          const mid = a.theta + rot / 2;
          const labelRadius = 132;
          const tx = b.x + Math.cos(rad(mid)) * labelRadius;
          const ty = b.y + Math.sin(rad(mid)) * labelRadius;
          const angleText = el("text", { x: tx, y: ty, "text-anchor": "middle", class: "traceAngleLabel" });
          angleText.textContent = `${rot > 0 ? "+" : ""}${fmt(rot, 0)}°${b.turn === "信地" ? "(信地旋回)" : ""} → θ${fmt(b.theta, 0)}°`;
          traceLayer.appendChild(angleText);
        }
      }
    }

    history.forEach((h, i) => {
      traceLayer.appendChild(el("circle", { cx: h.x, cy: h.y, r: i === history.length - 1 ? 14 : 9, fill: i === history.length - 1 ? "#ff9900" : "#006ee6", stroke: "#fff", "stroke-width": 4 }));
      if (h.wallDon) {
        const t = el("text", { x: h.x + (h.x < FIELD_W / 2 ? 260 : -260), y: h.y + (h.y < FIELD_H / 2 ? 260 : -260), "text-anchor": "middle", class: "wallDonLabel" });
        t.textContent = h.wallDon.corrected === null ? "壁ドン(角度大)" : "壁ドン";
        traceLayer.appendChild(t);
      }
    });
  }

  function arcPath(cx, cy, r, a0, a1) {
    const da = deltaAngle(a0, a1); if (Math.abs(da) < .01) return "";
    const s = norm(a0), e = s + da;
    const x0 = cx + Math.cos(rad(s)) * r, y0 = cy + Math.sin(rad(s)) * r;
    const x1 = cx + Math.cos(rad(e)) * r, y1 = cy + Math.sin(rad(e)) * r;
    return `M ${fmt(x0)} ${fmt(y0)} A ${r} ${r} 0 0 ${da >= 0 ? 1 : 0} ${fmt(x1)} ${fmt(y1)}`;
  }

  function renderMeasure(snap = null) {
    oldLayer.innerHTML = robotGhost(beforePose);
    if (prevRun && prevRun.length > 1) {
      oldLayer.appendChild(el("polyline", { points: prevRun.map(p => `${fmt(p.x)},${fmt(p.y)}`).join(" "), fill: "none", stroke: "#5b6570", "stroke-width": 6, "stroke-dasharray": "20 14", opacity: .55, "pointer-events": "none" }));
    }
    measureLayer.innerHTML = "";
    const move = Math.hypot(pose.x - beforePose.x, pose.y - beforePose.y) / 10;
    const signed = signedForwardCm(beforePose, pose);
    const rot = deltaAngle(beforePose.theta, pose.theta);
    if (move > .01) {
      measureLayer.appendChild(el("line", { x1: beforePose.x, y1: beforePose.y, x2: pose.x, y2: pose.y, stroke: "#ff9900", "stroke-width": 9, "stroke-dasharray": "22 14", "stroke-linecap": "round", "marker-end": "url(#arrow)", "pointer-events": "none" }));
      const mx = (beforePose.x + pose.x) / 2, my = (beforePose.y + pose.y) / 2;
      const t = el("text", { x: mx, y: my - 36, "text-anchor": "middle", class: "bigLabel" });
      t.textContent = signedCmText(signed);
      measureLayer.appendChild(t);
    }
    if (Math.abs(rot) > .01) {
      const d = arcPath(pose.x, pose.y, 190, beforePose.theta, pose.theta);
      if (d) measureLayer.appendChild(el("path", { d, fill: "none", stroke: "#006ee6", "stroke-width": 8, "marker-end": "url(#blueArrow)", "pointer-events": "none" }));
      const a = beforePose.theta + rot / 2;
      const t = el("text", { x: pose.x + Math.cos(rad(a)) * 245, y: pose.y + Math.sin(rad(a)) * 245, "text-anchor": "middle", class: "bigLabel" });
      t.textContent = `${rot > 0 ? "+" : ""}${fmt(rot, 0)}°`;
      measureLayer.appendChild(t);
    }
    snapLayer.innerHTML = "";
    if (snap) {
      const x = snap.node?.x ?? snap.x, y = snap.node?.y ?? snap.y;
      snapLayer.appendChild(el("circle", { cx: x, cy: y, r: snap.type === "junction" ? 30 : 24, fill: "none", stroke: snap.type === "junction" ? "#e23a48" : "#13b955", "stroke-width": 7, "stroke-dasharray": "10 7" }));
      const flip = x > FIELD_W * 0.72;
      const label = el("text", { x: x + (flip ? -36 : 36), y: y - 24, "text-anchor": flip ? "end" : "start", class: "smallLabel" });
      label.textContent = snap.type === "junction" ? "交差点吸着" : "黒線吸着";
      snapLayer.appendChild(label);
    }
    const totalCm = history.reduce((s, h) => s + Math.abs(h.signedCm ?? h.moveCm ?? 0), 0);
    const odo = odometry();
    const [wallName, wallDist] = nearestWallDist(pose);
    const touching = wallContacts(pose);
    const elapsedSec = runStart ? (Date.now() - runStart) / 1000 : 0;
    readout.textContent = `x ${fmt(pose.x)} / y ${fmt(pose.y)} / θ ${fmt(pose.theta)}° / ${signedCmText(signed)} / 累計 ${fmt(totalCm)}cm / 輪L ${signedCmText(odo.l)}・輪R ${signedCmText(odo.r)} / 壁まで${wallName}${fmt(Math.max(0, wallDist), 0)}mm${touching.length ? ` / 触:${touching.join("・")}` : ""} / 経過 ${fmt(elapsedSec)}秒`;
  }

  function renderGrid() {
    if (!gridLayer) return;
    gridLayer.innerHTML = "";
    if (!gridToggleEl.checked) return;
    for (let x = 100; x < FIELD_W; x += 100) {
      gridLayer.appendChild(el("line", { x1: x, y1: 0, x2: x, y2: FIELD_H, stroke: x % 500 ? "rgba(16,20,24,.08)" : "rgba(0,110,230,.22)", "stroke-width": x % 500 ? 2 : 4 }));
    }
    for (let y = 100; y < FIELD_H; y += 100) {
      gridLayer.appendChild(el("line", { x1: 0, y1: y, x2: FIELD_W, y2: y, stroke: y % 500 ? "rgba(16,20,24,.08)" : "rgba(0,110,230,.22)", "stroke-width": y % 500 ? 2 : 4 }));
    }
  }

  function renderHistoryPanel() {
    const list = $("historyList");
    if (!list) return;
    const rows = [`<div class="hHead">開始 (${fmt(history[0].x)}, ${fmt(history[0].y)}) θ${fmt(history[0].theta)}°</div>`];
    let accCm = 0, accRot = 0;
    for (let i = 1; i < history.length; i++) {
      const h = history[i];
      accCm += Math.abs(h.signedCm ?? h.moveCm ?? 0);
      accRot += (h.rot ?? 0);
      const kind = h.signedCm < -0.01 ? "後退" : h.signedCm > 0.01 ? "前進" : "その場";
      const distTxt = (kind === "その場") ? "" : ` ${h.signedCm > 0 ? "+" : ""}${fmt(h.signedCm)}cm`;
      const rotTxt = `回転 ${h.rot > 0 ? "+" : ""}${fmt(h.rot, 0)}°`;
      const marks = (h.wallDon ? "/壁ドン" + (h.wallDon.corrected === null ? "(角度大)" : "") : "") + (h.turn ? "/信地旋回" : "");
      rows.push(`<div class="hRow${i === history.length - 1 ? " now" : ""}" role="listitem"${i === history.length - 1 ? ' aria-current="true"' : ""}>${i}: ${kind}${distTxt} ${rotTxt} → θ${fmt(h.theta, 0)}°(計 ${fmt(accCm)}cm・${fmt(accRot, 0)}°)${marks}</div>`);
    }
    list.innerHTML = rows.join("");
    list.scrollTop = list.scrollHeight;
  }

  function renderAll(snap = null) {
    robot.setAttribute("transform", `translate(${fmt(pose.x)} ${fmt(pose.y)}) rotate(${fmt(pose.theta)})`);
    renderTrace();
    renderMeasure(snap);
    renderHistoryPanel();
    updateHistoryButtons();
  }

  function stateJson() { return JSON.stringify({ history, pose, beforePose }); }
  function restoreState(json) {
    const s = JSON.parse(json);
    history = s.history; pose = s.pose; beforePose = s.beforePose;
  }
  function undo() {
    if (!undoStack.length) return;
    redoStack.push(stateJson());
    restoreState(undoStack.pop());
    statusEl.textContent = "元に戻しました。";
    saveLocal(); renderAll();
  }
  function redo() {
    if (!redoStack.length) return;
    undoStack.push(stateJson());
    restoreState(redoStack.pop());
    statusEl.textContent = "やり直しました。";
    saveLocal(); renderAll();
  }
  function updateHistoryButtons() {
    if (!undoBtn || !redoBtn) return;
    undoBtn.disabled = !undoStack.length;
    redoBtn.disabled = !redoStack.length;
  }

  function commit(next, reason = "変更", opts = { moveLocked: true, snap: true }) {
    const old = { ...pose };
    let prepared;
    if (opts.moveLocked) prepared = opts.snap ? snapOnDriveAxis(next, old) : { pose: constrainToDriveAxis(next, old), snap: null };
    else if (opts.snap) prepared = snapFree(next);
    else prepared = { pose: { ...next }, snap: null };
    let p = clampToWalls({ x: prepared.pose.x, y: prepared.pose.y, theta: norm(prepared.pose.theta) });
    let wallDon = null;
    if (opts.moveLocked) {
      const r = applyWallDon(old, p, opts.alignDeg ?? WALL_ALIGN_DEG);
      wallDon = r.wallDon || null;
      p = { x: r.x, y: r.y, theta: r.theta };
    }
    const moveCm = Math.hypot(p.x - old.x, p.y - old.y) / 10;
    const signedCm = signedForwardCm(old, p);
    const rot = deltaAngle(old.theta, p.theta);
    const pivotTurn = Math.abs(rot) >= .05 && opts.turn === "one";
    if (moveCm < .01 && Math.abs(rot) < .01) { statusEl.setAttribute("aria-live", "polite"); renderMeasure(prepared.snap); return; }
    undoStack.push(stateJson());
    if (undoStack.length > 200) undoStack.shift();
    redoStack.length = 0;
    if (runStart === null) runStart = Date.now();
    beforePose = old; pose = p;
    history.push({ ...pose, moveCm, signedCm, rot, wallDon, turn: pivotTurn ? "信地" : null });
    if (history.length > 200) {
      const dropped = history.slice(1, history.length - 199);
      const acc = dropped.reduce((a, h) => ({ moveCm: a.moveCm + (h.moveCm || 0), signedCm: a.signedCm + (h.signedCm || 0) }), { moveCm: 0, signedCm: 0 });
      const kept = history[history.length - 199];
      kept.moveCm = (kept.moveCm || 0) + acc.moveCm;
      kept.signedCm = (kept.signedCm || 0) + acc.signedCm;
      history = [history[0], ...history.slice(-199)];
    }
    const kind = signedCm < -0.01 ? "後退" : signedCm > 0.01 ? "前進" : "その場";
    const snapMsg = prepared.snap ? (prepared.snap.type === "junction" ? " / T字・十字に吸着" : " / 黒線に吸着") : "";
    const donMsg = !wallDon ? "" :
      wallDon.corrected === null ? ` / 壁ドン(${wallDon.walls.join("・")}、角度大)` :
        Math.abs(wallDon.corrected) < .05 ? ` / 壁ドン(${wallDon.walls.join("・")}、直角どおり)` :
          ` / 壁ドン(${wallDon.walls.join("・")}、θ${wallDon.corrected > 0 ? "+" : ""}${fmt(wallDon.corrected, 0)}°補正)`;
    const turnMsg = pivotTurn ? " / 信地旋回" : "";
    statusEl.setAttribute("aria-live", "polite");
    statusEl.textContent = `${reason}: ${kind} ${signedCmText(signedCm)}、回転 ${rot > 0 ? "+" : ""}${fmt(rot, 0)}°${snapMsg}${donMsg}${turnMsg}`;
    renderAll(prepared.snap);
    saveLocal();
  }

  function onDown(evt) {
    if (evt.isPrimary === false || pointer) return;
    const p = svgPoint(evt); svg.focus();
    statusEl.setAttribute("aria-live", "off");
    if (evt.button === 1 || (!evt.button && evt.shiftKey)) {
      pointer = { type: "pan", id: evt.pointerId, cx: evt.clientX, cy: evt.clientY };
      try { svg.setPointerCapture(evt.pointerId); } catch (_) { }
      evt.preventDefault();
      return;
    }
    statusEl.setAttribute("aria-live", "polite");
    if (evt.button !== 0) return;
    if (evt.altKey) {
      pointer = { type: "measure", id: evt.pointerId, a: p };
      try { svg.setPointerCapture(evt.pointerId); } catch (_) { }
      evt.preventDefault();
      return;
    }
    const action = evt.target.dataset.action || evt.target.closest?.("[data-action]")?.dataset?.action;
    if (action === "rotate") {
      pointer = { type: "rotate", id: evt.pointerId, anchor: { ...pose } };
      beforePose = { ...pose }; try { svg.setPointerCapture(evt.pointerId); } catch (_) { } robot.classList.add("dragging"); evt.preventDefault(); return;
    }
    if (action === "drag" || evt.target.closest?.("#robot")) {
      pointer = { type: "drive", id: evt.pointerId, anchor: { ...pose }, dx: pose.x - p.x, dy: pose.y - p.y };
      beforePose = { ...pose }; try { svg.setPointerCapture(evt.pointerId); } catch (_) { } robot.classList.add("dragging"); evt.preventDefault(); return;
    }
    commit({ x: p.x, y: p.y, theta: pose.theta }, "クリック前後移動", { moveLocked: true, snap: true });
  }

  function onMove(evt) {
    if (!pointer || evt.pointerId !== pointer.id) return;
    const p = svgPoint(evt); evt.preventDefault();
    if (pointer.type === "pan") {
      const q = svgPoint({ clientX: pointer.cx, clientY: pointer.cy });
      view.x -= p.x - q.x;
      view.y -= p.y - q.y;
      pointer.cx = evt.clientX; pointer.cy = evt.clientY;
      applyView();
      return;
    }
    if (pointer.type === "measure") {
      measureLayer.innerHTML = "";
      measureLayer.appendChild(el("line", { x1: pointer.a.x, y1: pointer.a.y, x2: p.x, y2: p.y, stroke: "#5b6570", "stroke-width": 6, "stroke-dasharray": "16 10", "marker-end": "url(#blueArrow)", "pointer-events": "none" }));
      const t = el("text", { x: (pointer.a.x + p.x) / 2, y: (pointer.a.y + p.y) / 2 - 24, "text-anchor": "middle", class: "bigLabel" });
      t.textContent = `${fmt(Math.hypot(p.x - pointer.a.x, p.y - pointer.a.y))}mm`;
      measureLayer.appendChild(t);
      return;
    }

    // 正しい onMove 内の処理
    if (pointer.type === "rotate") {
      const target = snapTheta(deg(Math.atan2(p.y - pointer.anchor.y, p.x - pointer.anchor.x)));
      const delta = deltaAngle(pointer.anchor.theta, target);

      const isBackward = evt.shiftKey;
      pointer.isBackward = isBackward;

      pose = clampToWalls(rotatedPose(pointer.anchor, delta, isBackward));
      statusEl.textContent = `回転中: θ ${fmt(pose.theta)}°${pivotToggleEl.checked ? (isBackward ? "(後進信地)" : "(信地旋回)") : ""}`;
      renderAll();
    } else {
      const target = { x: p.x + pointer.dx, y: p.y + pointer.dy, theta: pointer.anchor.theta };
      const prepared = snapOnDriveAxis(target, pointer.anchor);
      pose = { x: prepared.pose.x, y: prepared.pose.y, theta: pointer.anchor.theta };
      const signed = signedForwardCm(pointer.anchor, pose);
      statusEl.textContent = prepared.snap ? `前後移動中: ${signedCmText(signed)} / 吸着中` : `前後移動中: ${signedCmText(signed)}`;
      renderAll(prepared.snap);
    }
  }

  function onUp(evt) {
    if (!pointer || evt.pointerId !== pointer.id) return;
    statusEl.setAttribute("aria-live", "polite");
    evt.preventDefault(); robot.classList.remove("dragging");
    try { svg.releasePointerCapture(pointer.id); } catch (_) { }
    if (pointer.type === "pan" || pointer.type === "measure") { pointer = null; return; }
    const next = { ...pose };
    pose = { ...pointer.anchor };
    // onUp 関数内の commit 呼び出し部分を以下のように調整
    const isRotate = pointer.type === "rotate";
    const isBackward = !!pointer.isBackward;

    commit(
      next,
      isRotate ? `${isBackward ? '後進' : ''}向き変更` : "前後ドラッグ",
      isRotate ? {
        moveLocked: false,
        snap: false,
        turn: pivotToggleEl.checked ? "one" : undefined,
        back: isBackward
      } : {
        moveLocked: true,
        snap: true
      }
    );

    // ★ここに pointer = null; を追加してドラッグ状態を解除します
    pointer = null;
  }

  function onCancel(evt) {
    if (!pointer || evt.pointerId !== pointer.id) return;
    statusEl.setAttribute("aria-live", "polite");
    try { svg.releasePointerCapture(pointer.id); } catch (_) { }
    if (pointer.type !== "pan" && pointer.type !== "measure") {
      pose = { ...pointer.anchor };
      statusEl.textContent = "操作を中断しました。位置は移動前の状態です。";
      renderAll();
    }
    robot.classList.remove("dragging");
    pointer = null;
  }

  function saveLocal() {
    try {
      localStorage.setItem(LS_KEY, JSON.stringify({ history, pose, beforePose, csv: csvText, pivot: pivotToggleEl.checked }));
      saveFailedNoted = false;
    } catch (_) {
      if (!saveFailedNoted) {
        saveFailedNoted = true;
        statusEl.textContent = "作業状態の自動保存ができません(プライベートモード等)。このまま操作は続けられます。";
      }
    }
  }

  function readSavedState() {
    try { return JSON.parse(localStorage.getItem(LS_KEY) || "null"); } catch (_) { return null; }
  }

  function tryRestore(clearStacks = true) {
    const s = readSavedState();
    const okPose = p => !!p && [p.x, p.y, p.theta].every(Number.isFinite);
    if (!s || !Array.isArray(s.history) || !s.history.length) return false;
    if (!okPose(s.pose)) return false;
    if (!s.history.every(h => h && okPose(h))) return false;
    const num = (v, d = 0) => Number.isFinite(v) ? v : d;
    const okRot = v => { const n = num(v); return Math.abs(n) <= 360 ? n : 0; };
    const okWallDon = w => !!w && Array.isArray(w.walls) && w.walls.length > 0 && w.walls.every(x => ["左壁", "右壁", "上壁", "下壁"].includes(x)) && (w.corrected === null || Number.isFinite(w.corrected));
    let restored = (s.history.length > 200 ? [s.history[0], ...s.history.slice(-199)] : s.history).map(h => ({
      ...h,
      theta: norm(num(h.theta)),
      moveCm: num(h.moveCm), signedCm: num(h.signedCm), rot: okRot(h.rot),
      wallDon: okWallDon(h.wallDon) ? { walls: h.wallDon.walls, corrected: h.wallDon.corrected } : null,
      turn: h.turn === "信地" ? "信地" : null,
    }));
    if (restored.length > 200) {
      const dropped = restored.slice(1, restored.length - 199);
      const acc = dropped.reduce((a, h) => ({ moveCm: a.moveCm + (h.moveCm || 0), signedCm: a.signedCm + (h.signedCm || 0) }), { moveCm: 0, signedCm: 0 });
      const kept = restored[restored.length - 199];
      kept.moveCm = (kept.moveCm || 0) + acc.moveCm;
      kept.signedCm = (kept.signedCm || 0) + acc.signedCm;
      restored = [restored[0], ...restored.slice(-199)];
    }
    history = restored;
    pose = clampToWalls({ ...s.pose, theta: norm(num(s.pose.theta)) });
    beforePose = okPose(s.beforePose) ? clampToWalls({ ...s.beforePose, theta: norm(num(s.beforePose.theta)) }) : { ...pose };
    if (pivotToggleEl && (s.pivot === true || s.pivot === false)) pivotToggleEl.checked = s.pivot;
    if (clearStacks) { undoStack.length = 0; redoStack.length = 0; }
    return true;
  }

  async function loadCsvAuto() {
    const firstRun = !readSavedState();
    try {
      const res = await fetch("lines.csv", { cache: "no-store" });
      if (!res.ok) throw new Error(`${res.status}`);
      csvText = await res.text();
      lines = parseCsv(csvText);
      rebuildNodes(); renderCourse();
      const restored = tryRestore();
      if (!restored) {
        const init = snapFree(pose); pose = init.pose; beforePose = { ...pose }; history = [{ ...pose, moveCm: 0, signedCm: 0, rot: 0 }];
      }
      const zeroNote = lines.length ? "" : " ※黒線が0本です。CSVの中身を確認してください。";
      statusEl.textContent = `lines.csv を読み込みました${restored ? "(前回の軌跡を復帰)" : ""}: 黒線 ${lines.length}本 / 交差点 ${nodes.filter(n => n.type === "junction").length}個${truncNote()}${zeroNote}`;
      filePanel.open = firstRun;
      const histPanelEl = $("historyPanel");
      if (histPanelEl && firstRun) histPanelEl.open = true;
      saveLocal(); renderAll();
    } catch (e) {
      const s = readSavedState();
      if (s && s.csv) {
        csvText = s.csv; lines = parseCsv(csvText);
        rebuildNodes(); renderCourse();
        statusEl.textContent = `lines.csv を自動読込できず、前回保存分で再開しました: 黒線 ${lines.length}本${truncNote()}`;
        tryRestore(); saveLocal(); renderAll();
      } else {
        const restored = tryRestore();
        statusEl.textContent = restored
          ? "lines.csv を自動読込できませんが、前回の軌跡で再開しました。『ヘルプと手動読込』からCSVを選べます。"
          : "lines.csv を自動読込できません。左上の『ヘルプと手動読込』からCSVを選択してください。";
        filePanel.open = true;
        renderCourse(); renderAll();
      }
    }
  }

  function loadCsvText(text, source = "lines.csv", keepTrace = false) {
    undoStack.push(stateJson());
    if (undoStack.length > 200) undoStack.shift();
    redoStack.length = 0;
    csvText = text;
    lines = parseCsv(text); rebuildNodes(); renderCourse();
    if (!(keepTrace && tryRestore(false))) {
      prevRun = null; runStart = null;
      const init = snapFree(pose); pose = init.pose; beforePose = { ...pose }; history = [{ ...pose, moveCm: 0, signedCm: 0, rot: 0 }];
    }
    statusEl.textContent = `${source} 読込: 黒線 ${lines.length}本 / 交差点 ${nodes.filter(n => n.type === "junction").length}個${truncNote()}。(Ctrl+Zで戻せます)`;
    saveLocal(); renderAll();
  }

  function buildCommands() {
    const rows = [["手順", "操作", "距離cm", "回転°", "絶対角度°", "x mm", "y mm", "備考"]];
    history.forEach((h, i) => {
      if (i === 0) { rows.push([0, "開始", "", 0, fmt(h.theta, 0), fmt(h.x), fmt(h.y), ""]); return; }
      const kind = h.signedCm < -0.01 ? "後退" : h.signedCm > 0.01 ? "前進" : "その場";
      const parts = [];
      if (h.wallDon) parts.push(`壁ドン(${h.wallDon.walls.join("・")}${h.wallDon.corrected === null ? "，角度大" : Math.abs(h.wallDon.corrected) < .05 ? "" : `，${fmt(h.wallDon.corrected, 0)}°補正`})`);
      if (h.turn) parts.push("信地旋回");
      const note = parts.join("/");
      rows.push([i, kind, kind === "その場" ? "" : fmt(h.signedCm ?? 0), fmt(h.rot, 0), fmt(h.theta, 0), fmt(h.x), fmt(h.y), note]);
    });
    let fwdCm = 0, backCm = 0, rotSum = 0, donCount = 0, pivotCount = 0;
    history.forEach((h, i) => {
      if (!i) return;
      const c = h.signedCm ?? h.moveCm ?? 0;
      if (c > 0.01) fwdCm += c; else if (c < -0.01) backCm -= c;
      rotSum += Math.abs(h.rot ?? 0);
      if (h.wallDon) donCount++;
      if (h.turn === "信地") pivotCount++;
    });
    const odo = odometry();
    rows.push(["計", `前進${fmt(fwdCm)}cm/後退${fmt(backCm)}cm`, "", "", "", "", "", `回転計${fmt(rotSum, 0)}°/壁ドン${donCount}回/信地${pivotCount}回/左輪${fmt(odo.l)}cm・右輪${fmt(odo.r)}cm`]);
    return rows.map(r => r.join(",")).join("\r\n");
  }

  svg.addEventListener("pointerdown", onDown);
  svg.addEventListener("pointermove", onMove);
  svg.addEventListener("pointerup", onUp);
  svg.addEventListener("pointercancel", onCancel);
  svg.addEventListener("wheel", evt => {
    evt.preventDefault();
    if (evt.deltaY === 0) return;
    if (!svg.getScreenCTM()) return;
    const p = svgPoint(evt);
    zoomAt(p.x, p.y, Math.exp(evt.deltaY * 0.002));
  }, { passive: false });
  svg.addEventListener("contextmenu", evt => { if (evt.shiftKey || pointer) evt.preventDefault(); });

  document.addEventListener("keydown", evt => {
    if (evt.ctrlKey || evt.metaKey || evt.altKey) return;
    const tag = (evt.target && evt.target.tagName) || "";
    if (tag === "TEXTAREA" || tag === "SELECT") return;
    if (tag === "INPUT" && evt.target.type !== "checkbox") return;
    const move = evt.shiftKey ? 50 : 10, turn = evt.shiftKey ? 15 : 5;
    let next = { ...pose }, opts = { moveLocked: true, snap: true }, reason = "キー操作";
    if (evt.key === "ArrowUp") { const u = unit(pose.theta); next.x += u.x * move; next.y += u.y * move; reason = "前進"; }
    else if (evt.key === "ArrowDown") { const u = unit(pose.theta); next.x -= u.x * move; next.y -= u.y * move; reason = "後退"; }
    else if (evt.key === "ArrowLeft") { Object.assign(next, rotatedPose(pose, -turn)); opts = { moveLocked: false, snap: false, turn: pivotToggleEl.checked ? "one" : undefined }; reason = "左回転"; }
    else if (evt.key === "ArrowRight") { Object.assign(next, rotatedPose(pose, +turn)); opts = { moveLocked: false, snap: false, turn: pivotToggleEl.checked ? "one" : undefined }; reason = "右回転"; }
    else if (evt.key.toLowerCase() === "q" && !evt.ctrlKey && !evt.metaKey) { Object.assign(next, rotatedPose(pose, -90)); opts = { moveLocked: false, snap: false, turn: pivotToggleEl.checked ? "one" : undefined }; reason = "90°左回転"; }
    else if (evt.key.toLowerCase() === "e" && !evt.ctrlKey && !evt.metaKey) { Object.assign(next, rotatedPose(pose, +90)); opts = { moveLocked: false, snap: false, turn: pivotToggleEl.checked ? "one" : undefined }; reason = "90°右回転"; }
    else if (evt.key.toLowerCase() === "w" && !evt.ctrlKey && !evt.metaKey) { evt.preventDefault(); wallDonDrive(1); return; }
    else if (evt.key.toLowerCase() === "s" && !evt.ctrlKey && !evt.metaKey) { evt.preventDefault(); wallDonDrive(-1); return; }
    else if (evt.key.toLowerCase() === "r" && !evt.ctrlKey && !evt.metaKey) { evt.preventDefault(); resetHome(); return; }
    else if (evt.key.toLowerCase() === "f" && !evt.ctrlKey && !evt.metaKey) { evt.preventDefault(); fitView(); return; }
    else if (evt.key.toLowerCase() === "t" && !evt.ctrlKey && !evt.metaKey) { evt.preventDefault(); setPivotMode(!pivotToggleEl.checked); return; }
    else return;
    evt.preventDefault(); commit(next, reason, opts);
  });

  // ---- 初期位置へ戻す ----
  function resetHome() {
    const target = clampToWalls({ x: initPos.x, y: initPos.y, theta: initPos.theta });
    if (history.length === 1 && dist(pose, target) < .01 && Math.abs(deltaAngle(pose.theta, target.theta)) < .01) {
      statusEl.textContent = "すでに初期位置です。";
      return;
    }
    undoStack.push(stateJson());
    if (undoStack.length > 200) undoStack.shift();
    redoStack.length = 0;
    if (history.length > 1) prevRun = history.map(h => ({ x: h.x, y: h.y }));
    runStart = null;
    pose = target;
    beforePose = { ...pose };
    history = [{ ...pose, moveCm: 0, signedCm: 0, rot: 0 }];
    statusEl.textContent = `初期位置 (${fmt(pose.x)}, ${fmt(pose.y)}) θ${fmt(pose.theta)}° に戻しました。(Ctrl+Zで戻せます)`;
    saveLocal(); renderAll();
  }

  $("clearBtn").addEventListener("click", () => {
    if (history.length === 1) { statusEl.textContent = "軌跡は空です。"; return; }
    undoStack.push(stateJson());
    if (undoStack.length > 200) undoStack.shift();
    redoStack.length = 0;
    if (history.length > 1) prevRun = history.map(h => ({ x: h.x, y: h.y }));
    runStart = null;
    beforePose = { ...pose }; history = [{ ...pose, moveCm: 0, signedCm: 0, rot: 0 }];
    statusEl.textContent = "軌跡をクリアしました。(Ctrl+Zで元に戻せます)";
    saveLocal(); renderAll();
  });
  $("reloadBtn").addEventListener("click", () => { loadCsvAuto(); });
  $("wallDonFwdBtn").addEventListener("click", () => wallDonDrive(1));
  $("wallDonBackBtn").addEventListener("click", () => wallDonDrive(-1));
  $("resetBtn").addEventListener("click", resetHome);
  $("fitBtn").addEventListener("click", fitView);
  showCourseEl.addEventListener("change", renderCourse);
  gridToggleEl.addEventListener("change", renderGrid);
  snapAngleEl.addEventListener("change", () => { statusEl.textContent = snapAngleEl.checked ? `角度吸着ON: 0/90/180/270°の±${ANGLE_SNAP_DEG}°だけ吸着します。` : "角度吸着OFF"; });

  function setPivotMode(on) {
    pivotToggleEl.checked = on;
    statusEl.textContent = on ? "信地旋回ON: 片輪を固定して弧を描くように回ります(回転中に位置が変わります)。T で切替。" : "信地旋回OFF(超信地): その場で回転します。T で切替。";
    saveLocal();
  }
  pivotToggleEl.addEventListener("change", () => setPivotMode(pivotToggleEl.checked));
  snapLineEl.addEventListener("change", () => { statusEl.textContent = snapLineEl.checked ? "黒線吸着ON: 移動先が黒線・交差点に近いと吸着します。" : "黒線吸着OFF"; });
  $("csvInput").addEventListener("change", evt => { const f = evt.target.files?.[0]; evt.target.blur(); if (f) { f.text().then(t => loadCsvText(t, f.name, false)); } });
  undoBtn.addEventListener("click", undo);
  redoBtn.addEventListener("click", redo);
  copyBtn.addEventListener("click", async () => {
    const t = buildCommands().replace(/,/g, "\t");
    let ok = false;
    try { await navigator.clipboard.writeText(t); ok = true; }
    catch (_) {
      try {
        const ta = document.createElement("textarea");
        ta.value = t; document.body.appendChild(ta); ta.select();
        ok = document.execCommand("copy");
        ta.remove();
      } catch (_2) { }
    }
    statusEl.textContent = ok ? "コマンド列をクリップボードにコピーしました(タブ区切り)。" : "クリップボードにコピーできませんでした。CSV出力を使ってください。";
  });
  csvBtn.addEventListener("click", () => {
    const blob = new Blob(["\ufeff" + buildCommands()], { type: "text/csv" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "commands.csv";
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.ObjectURL.revokeObjectURL(a.href), 1000);
    statusEl.textContent = "コマンド列を commands.csv に保存しました。";
  });

  const PDFJS_URL = "https://cdn.jsdelivr.net/npm/pdfjs-dist@3.11.174/build/pdf.min.js";
  const PDFJS_WORKER_URL = "https://cdn.jsdelivr.net/npm/pdfjs-dist@3.11.174/build/pdf.worker.min.js";
  let pdfLibPromise = null;
  function loadPdfJs() {
    if (window.pdfjsLib) return Promise.resolve(window.pdfjsLib);
    if (!pdfLibPromise) {
      pdfLibPromise = new Promise((resolve, reject) => {
        const s = document.createElement("script");
        const timer = setTimeout(() => reject(new Error("タイムアウト")), 10000);
        s.src = PDFJS_URL;
        s.onload = () => { clearTimeout(timer); window.pdfjsLib ? resolve(window.pdfjsLib) : reject(new Error("読み込み失敗")); };
        s.onerror = () => { clearTimeout(timer); reject(new Error("ネットワークエラー")); };
        document.head.appendChild(s);
      }).catch(e => { pdfLibPromise = null; throw e; });
    }
    return pdfLibPromise;
  }
  let bgUrl = "";
  function setBg(url, { hideWrapOnError = false } = {}) {
    if (bgUrl && /^blob:/.test(bgUrl) && bgUrl !== url) {
      try { URL.revokeObjectURL(bgUrl); } catch (_) { }
    }
    bgUrl = url || "";
    if (bgUrl) {
      bgEl.setAttribute("href", bgUrl);
      bgEl.setAttributeNS("http://www.w3.org/1999/xlink", "href", bgUrl);
      bgEl.style.display = bgToggleEl.checked ? "" : "none";
      bgToggleWrap.classList.remove("hidden");
    } else {
      bgEl.removeAttribute("href");
      bgEl.removeAttributeNS("http://www.w3.org/1999/xlink", "href");
      bgEl.style.display = "none";
      if (hideWrapOnError) bgToggleWrap.classList.add("hidden");
    }
  }
  async function renderPdfBackground(buf, name = "field.pdf", quiet = false) {
    try {
      const lib = await loadPdfJs();
      lib.GlobalWorkerOptions.workerSrc = PDFJS_WORKER_URL;
      const doc = await lib.getDocument({ data: buf }).promise;
      const page = await doc.getPage(1);
      const vp1 = page.getViewport({ scale: 1 });
      const vp = page.getViewport({ scale: 1800 / vp1.width });
      const c = document.createElement("canvas");
      c.width = Math.ceil(vp.width); c.height = Math.ceil(vp.height);
      await page.render({ canvasContext: c.getContext("2d"), viewport: vp }).promise;
      setBg(c.toDataURL("image/jpeg", .85));
      if (!quiet) statusEl.textContent = `${name} を背景に表示しました。(黒線はCSVから描画)`;
      return true;
    } catch (e) {
      if (!bgUrl) {
        setBg("", { hideWrapOnError: true });
        if (!quiet) statusEl.textContent = `${name} を背景にできません(${e && e.message ? e.message : "エラー"})。オフライン時はCSVの黒線のみで動作します。`;
      } else {
        if (!quiet) statusEl.textContent = `${name} を背景にできませんでした(${e && e.message ? e.message : "エラー"})。以前の背景を維持します。`;
      }
      return false;
    }
  }

  function loadJpgAuto() {
    return new Promise(resolve => {
      const probe = new Image();
      probe.onload = () => { setBg(probe.src); resolve(true); };
      probe.onerror = () => resolve(false);
      probe.src = "field_bg.jpg";
    });
  }

  async function loadPdfAuto() {
    if (location.protocol === "file:") return false;
    try {
      const res = await fetch("field.pdf", { cache: "no-store" });
      if (!res.ok) throw new Error(`${res.status}`);
      return await renderPdfBackground(await res.arrayBuffer(), "field.pdf", true);
    } catch (_) { return false; }
  }

  async function loadBackgroundAuto() {
    if (await loadJpgAuto()) return;
    await loadPdfAuto();
  }

  // ★ここに追加: config.jsonを非同期で読み込んで初期位置を更新する関数
  async function loadConfigAuto() {
    try {
      const res = await fetch("config.json", { cache: "no-store" });
      if (!res.ok) return;
      const config = await res.json();
      if (config && config.robot_init) {
        // config.jsonの座標で初期位置(initPos)を上書き
        if (Number.isFinite(config.robot_init.x)) initPos.x = config.robot_init.x;
        if (Number.isFinite(config.robot_init.y)) initPos.y = config.robot_init.y;
        if (Number.isFinite(config.robot_init.angle)) initPos.theta = config.robot_init.angle;

        // 入力フォームの表示値も更新
        if (initXInput) initXInput.value = initPos.x;
        if (initYInput) initYInput.value = initPos.y;
        if (initAngleInput) initAngleInput.value = initPos.theta;

        // 前回の保存状態（LocalStorage）がない初回起動時は、ロボットを新初期位置へ配置
        if (!readSavedState()) {
          resetHome();
        }
      }
    } catch (e) {
      console.warn("config.json の読み込みをスキップしました:", e);
    }
  }

  bgToggleEl.addEventListener("change", () => {
    if (bgEl.style.display === "none" && bgUrl && bgToggleEl.checked) {
      bgEl.style.display = "";
    } else {
      bgEl.style.display = bgToggleEl.checked ? "" : "none";
    }
    statusEl.textContent = bgToggleEl.checked ? "背景画像を表示しました。" : "背景画像を隠しました。";
  });

  $("bgInput").addEventListener("change", async evt => {
    const f = evt.target.files?.[0];
    evt.target.blur();
    if (!f) return;
    if (/\.pdf$/i.test(f.name) || f.type === "application/pdf") {
      await renderPdfBackground(await f.arrayBuffer(), f.name);
    } else {
      setBg(URL.createObjectURL(f));
      statusEl.textContent = `${f.name} を背景に表示しました。`;
    }
  });

  window.addEventListener("dragover", evt => { evt.preventDefault(); });
  window.addEventListener("drop", async evt => {
    evt.preventDefault();
    const f = evt.dataTransfer?.files?.[0];
    if (!f) return;
    if (/\.csv$/i.test(f.name)) {
      loadCsvText(await f.text(), f.name, false);
    } else if (/\.pdf$/i.test(f.name) || f.type === "application/pdf") {
      await renderPdfBackground(await f.arrayBuffer(), f.name);
    } else if (/^image\//.test(f.type) || /\.(jpe?g|png)$/i.test(f.name)) {
      setBg(URL.createObjectURL(f));
      statusEl.textContent = `${f.name} を背景に表示しました。`;
    }
  });

  document.addEventListener("keydown", evt => {
    if (!(evt.ctrlKey || evt.metaKey)) return;
    const tag = (evt.target && evt.target.tagName) || "";
    if (tag === "TEXTAREA" || tag === "SELECT") return;
    if (tag === "INPUT" && evt.target.type !== "checkbox") return;
    const k = evt.key.toLowerCase();
    if (k === "z" && !evt.shiftKey) { evt.preventDefault(); undo(); }
    else if ((k === "z" && evt.shiftKey) || k === "y") { evt.preventDefault(); redo(); }
  });

  (async () => {
    loadInitPos();
    await loadConfigAuto(); // ★ config.json を読み込む
    applyView();
    renderCourse(); renderGrid(); renderAll(); loadCsvAuto(); loadBackgroundAuto();
  })();

  window.simDebug = { get pose() { return pose; }, get history() { return history; }, executePivotTurn, commit, undo, redo, wallContacts, clampToWalls, applyWallDon, wallDonDrive, driveToWall, resetHome, parseCsv, snapFree, buildCommands, zoomAt, fitView, rotatedPose, extentXF, extentXB, extentYU, extentYD, setPivotMode, renderHistoryPanel, odometry, wheelTravelCm, get pivotOn() { return !!(pivotToggleEl && pivotToggleEl.checked); }, get view() { return view; } };
})();

// パネルのドラッグ移動処理
(() => {
  const panel = document.querySelector('.filePanel');
  if (!panel) return;

  const handle = panel.querySelector('summary');
  if (!handle) return;

  let isDragging = false;
  let hasMoved = false;
  let startX = 0, startY = 0;
  let initialLeft = 0, initialTop = 0;

  handle.addEventListener('pointerdown', (e) => {
    if (e.button !== 0) return;
    isDragging = true;
    hasMoved = false;
    startX = e.clientX;
    startY = e.clientY;

    const rect = panel.getBoundingClientRect();
    initialLeft = rect.left;
    initialTop = rect.top;

    panel.style.right = 'auto';
    panel.style.bottom = 'auto';

    handle.setPointerCapture(e.pointerId);
  });

  handle.addEventListener('pointermove', (e) => {
    if (!isDragging) return;
    const dx = e.clientX - startX;
    const dy = e.clientY - startY;

    if (Math.hypot(dx, dy) > 5) {
      hasMoved = true;
    }

    panel.style.left = `${initialLeft + dx}px`;
    panel.style.top = `${initialTop + dy}px`;
  });

  const stopDrag = (e) => {
    if (!isDragging) return;
    isDragging = false;
    try {
      handle.releasePointerCapture(e.pointerId);
    } catch (err) { }
  };

  handle.addEventListener('pointerup', stopDrag);
  handle.addEventListener('pointercancel', stopDrag);

  handle.addEventListener('click', (e) => {
    if (hasMoved) {
      e.preventDefault();
      hasMoved = false;
    }
  });
})();
