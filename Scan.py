import sys
import cv2
import pymupdf  # PyMuPDF
import numpy as np
import csv  # CSV出力用に標準ライブラリをインポート


def analyze_robot_field(pdf_path):
    # 1. PDFを読み込んで画像化
    doc = pymupdf.open(pdf_path)
    try:
        return _analyze_pdf(doc)
    finally:
        doc.close()


def _analyze_pdf(doc):
    page = doc[0]

    # 回転ページは get_pixmap が追従して縦長画像になり、アスペクトクロップが破綻する。
    # 静かな誤出力を避けるため回転付きページは解析しない。
    if page.rotation % 360 != 0:
        print(
            f"エラー: PDFページが回転しています({page.rotation}°)。"
            "正立のPDFを用意してください(回転したまま解析すると誤った線を抽出します)。"
        )
        return []

    # 解像度: 1mm = 5ピクセル (127 DPI)
    px_per_mm = 5.0
    dpi = px_per_mm * 25.4
    zoom = dpi / 72.0
    matrix = pymupdf.Matrix(zoom, zoom)
    # alpha=False でアルファチャンネルを省略し、純粋なRGB画像として取得
    pix = page.get_pixmap(matrix=matrix, alpha=False)

    # numpy配列に変換
    img = np.frombuffer(pix.samples, dtype=np.uint8).reshape(
        pix.height, pix.width, pix.n
    )
    if pix.n == 4:
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
    elif pix.n == 1:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    # 2. グレースケール化と厳格な二値化
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    BLACK_THRESHOLD = 50
    _, binary = cv2.threshold(gray, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    # 黒線内部の白ノイズ除去
    noise_kernel = np.ones((5, 5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, noise_kernel)

    # 3. フィールド範囲の推定 (アスペクト比から余白を計算)
    img_h, img_w = binary.shape
    field_w_mm = 2362.0
    field_h_mm = 1143.0
    field_aspect = field_w_mm / field_h_mm
    img_aspect = img_w / img_h

    if img_aspect > field_aspect:
        field_h_px = img_h
        field_w_px = int(img_h * field_aspect)
        offset_x = (img_w - field_w_px) // 2
        offset_y = 0
    else:
        field_w_px = img_w
        field_h_px = int(img_w / field_aspect)
        offset_x = 0
        offset_y = (img_h - field_h_px) // 2

    if offset_x > 0 or offset_y > 0:
        print(
            f"注意: PDFページに余白があります(余白 x:{offset_x}px y:{offset_y}px)。"
            "field.pdf が縮小コピー等でないか確認してください。"
        )

    # フィールド部分を切り抜き
    field_binary = binary[
        offset_y: offset_y + field_h_px, offset_x: offset_x + field_w_px
    ]
    # デバッグ用に元画像のフィールド部分をカラーで保持
    field_img = img[
        offset_y: offset_y + field_h_px, offset_x: offset_x + field_w_px
    ].copy()

    # シミュレーター用の背景画像(検出線を描画する前の状態)を保存。
    # そのままでは巨大(5px/mm)なので 2px/mm 相当に縮小して出力する。
    bg_scale = 2.0 / px_per_mm
    bg_img = cv2.resize(
        field_img,
        (int(field_img.shape[1] * bg_scale), int(field_img.shape[0] * bg_scale)),
        interpolation=cv2.INTER_AREA,
    )
    if not cv2.imwrite("field_bg.jpg", cv2.cvtColor(bg_img, cv2.COLOR_RGB2BGR),
                       [cv2.IMWRITE_JPEG_QUALITY, 85]):
        print("警告: field_bg.jpg の保存に失敗しました。")
    else:
        print("field_bg.jpg を作成しました。(HTMLシミュレーターの背景用)")

    # 4. パラメータ設定
    line_thickness_mm = 20.0
    min_line_length_mm = 30.0  # ノイズ除去用の最小の線の長さ

    # 5. 形態学的処理で縦線と横線を抽出
    kernel_h_size = max(1, int(line_thickness_mm / 2 * px_per_mm))
    kernel_w_size = int(min_line_length_mm * px_per_mm)

    # 水平線の抽出
    kernel_h = cv2.getStructuringElement(
        cv2.MORPH_RECT, (kernel_w_size, kernel_h_size)
    )
    horizontal_lines = cv2.morphologyEx(field_binary, cv2.MORPH_OPEN, kernel_h)

    # 垂直線の抽出
    kernel_v = cv2.getStructuringElement(
        cv2.MORPH_RECT, (kernel_h_size, kernel_w_size)
    )
    vertical_lines = cv2.morphologyEx(field_binary, cv2.MORPH_OPEN, kernel_v)

    detected_lines = []

    # 線の太さ判定は定数なのでループの外で一度だけ計算する
    min_thickness = int(15 * px_per_mm)
    max_thickness = int(25 * px_per_mm)

    # 白チェック用のグレー画像(フィールド部分)。毎回切り出し直さず一度だけ作る
    gray_field = gray[
        offset_y: offset_y + field_h_px, offset_x: offset_x + field_w_px
    ]

    def check_white_touch(y_start, y_end, x_start, x_end, is_horizontal):
        WHITE_THRESHOLD = 200

        if is_horizontal:
            top_y = max(0, y_start - 3)
            bottom_y = min(field_binary.shape[0] - 1, y_end + 3)
            top_region = gray_field[top_y:y_start, x_start:x_end]
            bottom_region = gray_field[y_end:bottom_y, x_start:x_end]

            return np.any(top_region >= WHITE_THRESHOLD) or np.any(
                bottom_region >= WHITE_THRESHOLD
            )
        else:
            left_x = max(0, x_start - 3)
            right_x = min(field_binary.shape[1] - 1, x_end + 3)
            left_region = gray_field[y_start:y_end, left_x:x_start]
            right_region = gray_field[y_start:y_end, x_end:right_x]

            return np.any(left_region >= WHITE_THRESHOLD) or np.any(
                right_region >= WHITE_THRESHOLD
            )

    def process_lines(line_img, is_horizontal):
        num_labels, _, stats, _ = cv2.connectedComponentsWithStats(line_img)
        for i in range(1, num_labels):
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]

            if is_horizontal:
                if not (min_thickness <= h <= max_thickness and w > h * 2):
                    continue

                # 端点の延長補正
                limit_px = int(15 * px_per_mm)
                new_x_start = x
                count = 0
                while new_x_start > 0 and count < limit_px:
                    if np.any(field_binary[y: y + h, new_x_start - 1] == 255):
                        new_x_start -= 1
                        count += 1
                    else:
                        break

                new_x_end = x + w
                count = 0
                while (
                    new_x_end < field_binary.shape[1] - 1 and count < limit_px
                ):
                    if np.any(field_binary[y: y + h, new_x_end] == 255):
                        new_x_end += 1
                        count += 1
                    else:
                        break

                y_center = y + h // 2

                if not check_white_touch(y, y + h, new_x_start, new_x_end, True):
                    continue

                # 全ての端を1cm (10mm) カットする
                cut_px = int(10 * px_per_mm)
                new_x_start += cut_px
                new_x_end -= cut_px

                if new_x_start >= new_x_end:
                    continue

                mm_x_start = (new_x_start + offset_x) / px_per_mm
                mm_x_end = (new_x_end + offset_x) / px_per_mm
                mm_y = (y_center + offset_y) / px_per_mm

                detected_lines.append(
                    {
                        "type": "horizontal",
                        "start": (mm_x_start, mm_y),
                        "end": (mm_x_end, mm_y),
                    }
                )

                # デバッグ描画 (BGR: 黄緑色)
                cv2.line(
                    field_img,
                    (new_x_start, y_center),
                    (new_x_end, y_center),
                    (102, 255, 102),
                    3,
                )
            else:
                if not (min_thickness <= w <= max_thickness and h > w * 2):
                    continue

                # 端点の延長補正
                limit_px = int(15 * px_per_mm)
                new_y_start = y
                count = 0
                while new_y_start > 0 and count < limit_px:
                    if np.any(field_binary[new_y_start - 1, x: x + w] == 255):
                        new_y_start -= 1
                        count += 1
                    else:
                        break

                new_y_end = y + h
                count = 0
                while (
                    new_y_end < field_binary.shape[0] - 1 and count < limit_px
                ):
                    if np.any(field_binary[new_y_end, x: x + w] == 255):
                        new_y_end += 1
                        count += 1
                    else:
                        break

                x_center = x + w // 2

                if not check_white_touch(
                    new_y_start, new_y_end, x, x + w, False
                ):
                    continue

                # 全ての端を1cm (10mm) カットする
                cut_px = int(10 * px_per_mm)
                new_y_start += cut_px
                new_y_end -= cut_px

                if new_y_start >= new_y_end:
                    continue

                mm_x = (x_center + offset_x) / px_per_mm
                mm_y_start = (new_y_start + offset_y) / px_per_mm
                mm_y_end = (new_y_end + offset_y) / px_per_mm

                detected_lines.append(
                    {
                        "type": "vertical",
                        "start": (mm_x, mm_y_start),
                        "end": (mm_x, mm_y_end),
                    }
                )

                # デバッグ描画 (BGR: 黄緑色)
                cv2.line(
                    field_img,
                    (x_center, new_y_start),
                    (x_center, new_y_end),
                    (102, 255, 102),
                    3,
                )

    process_lines(horizontal_lines, True)
    process_lines(vertical_lines, False)

    # 6. スタートエリアの検出（カラー画像から青色を抽出）
    start_areas = detect_start_area(
        field_img, offset_x, offset_y, px_per_mm
    )

    for i, area in enumerate(start_areas):
        print(
            f"スタートエリア {i+1}: 座標({area['x']:.1f}mm, {area['y']:.1f}mm), "
            f"サイズ({area['w']:.1f}mm × {area['h']:.1f}mm)"
        )

    # debug.jpg を作成
    debug_img_bgr = cv2.cvtColor(field_img, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite("debug.jpg", debug_img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 90]):
        print("警告: debug.jpg の保存に失敗しました。")
    else:
        print("debug.jpg を作成しました。")

    return detected_lines


def detect_start_area(field_img, offset_x, offset_y, px_per_mm):
    """
    内部輪郭のみを抽出し、25cm×25cmのスタートエリアを検出する関数
    """
    # 1. RGBからHSV色空間へ変換
    hsv = cv2.cvtColor(field_img, cv2.COLOR_RGB2HSV)

    # 2. 青色の範囲を指定 (112, 191, 143)
    lower_blue = np.array([102, 120, 80])
    upper_blue = np.array([122, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 3. 枠線の細い途切れを補正
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 4. 階層構造を取得するために RETR_TREE を指定
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    start_areas = []

    # 輪郭が存在しない場合
    if hierarchy is None:
        return start_areas

    # 250mm × 250mm（誤差 ±50mm）の範囲設定
    target_w_mm = 250.0
    target_h_mm = 250.0
    tolerance_mm = 50.0

    min_w_px = (target_w_mm - tolerance_mm) * px_per_mm
    max_w_px = (target_w_mm + tolerance_mm) * px_per_mm
    min_h_px = (target_h_mm - tolerance_mm) * px_per_mm
    max_h_px = (target_h_mm + tolerance_mm) * px_per_mm

    # 階層情報の配列を取得 [Next, Previous, First_Child, Parent]
    hierarchy_data = hierarchy[0]

    for i, cnt in enumerate(contours):
        # 親 (Parent) が存在しない輪郭 (==-1) は最外郭なのでスキップし、内部輪郭のみ抽出
        if hierarchy_data[i][3] == -1:
            continue

        # 輪郭の近似
        epsilon = 0.03 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        x, y, w, h = cv2.boundingRect(cnt)

        # 頂点数および外形サイズのチェック
        if 4 <= len(approx) <= 6:
            if min_w_px <= w <= max_w_px and min_h_px <= h <= max_h_px:
                mm_x = (x + offset_x) / px_per_mm
                mm_y = (y + offset_y) / px_per_mm
                mm_w = w / px_per_mm
                mm_h = h / px_per_mm

                start_areas.append(
                    {
                        "x": mm_x,
                        "y": mm_y,
                        "w": mm_w,
                        "h": mm_h,
                    }
                )

                # デバッグ画像に赤色の太枠を描画 (BGR: 0, 0, 255)
                cv2.rectangle(field_img, (x, y), (x + w, y + h), (0, 0, 255), 4)

    return start_areas


def save_to_csv(lines, csv_path):
    """検出した線の情報をCSVファイルに保存する"""
    with open(csv_path, mode="w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        # データの書き込み
        for i, line in enumerate(lines):
            writer.writerow([
                f"{line['start'][0]:.1f}",
                f"{line['start'][1]:.1f}",
                f"{line['end'][0]:.1f}",
                f"{line['end'][1]:.1f}",
            ])
    print(f"{csv_path} を作成しました。")


if __name__ == "__main__":
    import os
    import sys

    pdf_file = "field.pdf"
    csv_file = "lines.csv"
    if not os.path.exists(pdf_file):
        print(f"エラー: {pdf_file} が見つかりません。リポジトリ直下で実行してください。")
        sys.exit(1)
    try:
        lines = analyze_robot_field(pdf_file)
        print(f"検出された黒線の数: {len(lines)}")
        if not lines:
            print("黒線が1本も検出できませんでした。既存の lines.csv を壊さないよう更新を中止します(入力PDFや明るさを確認してください)。")
            sys.exit(2)

        # コンソールに表示
        for i, line in enumerate(lines):
            print(f"線 {i+1}:")
            print(f"  向き: {'横線' if line['type'] == 'horizontal' else '縦線'}")
            print(f"  始点: ({line['start'][0]:.1f} mm, {line['start'][1]:.1f} mm)")
            print(f"  終点: ({line['end'][0]:.1f} mm, {line['end'][1]:.1f} mm)")

        # CSVに出力
        save_to_csv(lines, csv_file)

    except Exception as e:
        print(f"エラーが発生しました: {e}")
        sys.exit(1)  # CI等で失敗に気づけるよう終了コードを返す
