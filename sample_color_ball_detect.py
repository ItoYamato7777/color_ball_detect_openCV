import cv2
import numpy as np

# 検出したいボールの色の範囲をHSVで指定します。
# 色の範囲は環境によって調整が必要になります。
# 各色につき、( lower_hsv, upper_hsv ) のタプルで指定します。
# 赤色はHSV空間で0付近と180付近に範囲が分かれるため、タプルのリストで指定します。
color_ranges = {
    'red': [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [180, 255, 255])],
    'blue': [([100, 150, 0], [140, 255, 255])],   # 例: 青色の範囲 (調整してください)
    'green': [([40, 70, 0], [80, 255, 255])]     # 例: 緑色の範囲 (調整してください)
}

# 検出したボールを描画する際の色を指定します (BGR形式)。
# color_rangesで指定したキー名と合わせます。
color_bgr = {
    'red': (0, 0, 255),      # 赤色ボールは赤で描画
    'blue': (255, 0, 0),     # 青色ボールは青で描画
    'green': (0, 255, 0)     # 緑色ボールは緑で描画
}


# カメラからの入力を開始 (0はデフォルトのカメラ)
# 動画ファイルから読み込む場合は、 'video.mp4' のようにファイルパスを指定
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("エラー: カメラを開けません")
    exit()

print("ボール検出を開始します。終了するには 'q' キーを押してください。")

while True:
    # フレームを1枚キャプチャ
    ret, frame = cap.read()
    if not ret:
        print("エラー: フレームをキャプチャできません")
        break

    # フレームをHSV色空間に変換
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 各色について検出処理を行います
    for color_name, ranges in color_ranges.items():
        # 指定した色の範囲でマスクを作成
        # 赤色のように複数の範囲がある場合はcv2.bitwise_orで結合します
        mask = None
        for (lower, upper) in ranges:
            if mask is None:
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            else:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lower), np.array(upper)))


        # マスクを収縮・膨張させてノイズを除去
        # カーネルサイズやiterationsは検出精度に応じて調整してください
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # マスクから輪郭を検出
        # OpenCV 3.xと4.xで戻り値の形式が異なる場合があるため、検出された輪郭だけを取得
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 検出されたボールの数を色ごとにカウントするためのカウンター
        ball_count = 0

        # 輪郭が検出された場合
        if len(contours) > 0:
            # 検出されたすべての輪郭を処理します
            for i, c in enumerate(contours):
                # 輪郭の面積が小さすぎる場合はノイズとして無視
                if cv2.contourArea(c) < 100: # 面積の閾値は適宜調整
                    continue

                # 輪郭を囲む最小の円を取得
                ((x, y), radius) = cv2.minEnclosingCircle(c)

                # 輪郭のモーメントを計算
                M = cv2.moments(c)

                # モーメントから中心座標を計算 (分母がゼロになる可能性を避ける)
                center = None
                if M["m00"] != 0:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # ある程度の大きさの円（ボールらしいサイズ）のみを処理
                if center is not None and radius > 8: # 半径の閾値は適宜調整
                    ball_count += 1
                    # ボール名を「色_連番」の形式で生成 (例: red_1, blue_2)
                    ball_name = f"{color_name}_{ball_count}"

                    # このボールを描画する色を取得 (見つからなければ白)
                    draw_color = color_bgr.get(color_name, (255, 255, 255))

                    # 元のフレームに円と中心点を描画
                    cv2.circle(frame, (int(x), int(y)), int(int(radius)), draw_color, 2)
                    cv2.circle(frame, center, 5, draw_color, -1)

                    # ボール名と中心座標を表示
                    text = f"{ball_name} X:{center[0]}, Y:{center[1]}"
                    # cv2.putText()の引数: (画像, 表示する文字列, 表示開始座標, フォント, 文字サイズ, 色, 太さ)
                    cv2.putText(frame, text, (center[0] - 50, center[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)

                    # ターミナルに出力
                    print(f"検出: {ball_name}, 中心座標: X={center[0]}, Y={center[1]}")


    # 結果を表示
    cv2.imshow('Ball Detection Result', frame) # ウィンドウタイトルを変更
    # 各色のマスクを確認したい場合は、下のコメントアウトを解除してください
    cv2.imshow(f'{color_name} Mask', mask)

    # 'q'キーが押されたらループを終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 後処理: カメラを解放し、全てのウィンドウを閉じる
cap.release()
cv2.destroyAllWindows()