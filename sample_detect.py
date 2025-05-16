import cv2
import numpy as np

# ボールの色の範囲をHSVで指定 (例: 赤色)
# この値は検出したいボールの色に合わせて調整してください
lower_color = np.array([0, 100, 100])
upper_color = np.array([10, 255, 255])

# カメラからの入力を開始 (0はデフォルトのカメラ)
# 動画ファイルから読み込む場合は、 'video.mp4' のようにファイルパスを指定
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("エラー: カメラを開けません")
    exit()

while True:
    # フレームを1枚キャプチャ
    ret, frame = cap.read()
    if not ret:
        print("エラー: フレームをキャプチャできません")
        break

    # フレームをHSV色空間に変換
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 指定した色の範囲でマスクを作成
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # マスクを収縮・膨張させてノイズを除去
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # マスクから輪郭を検出
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    center = None

    # 輪郭が検出された場合
    if len(contours) > 0:
        # 最大の輪郭を取得
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)

        # モーメントから中心座標を計算 (ゼロ除算を避ける)
        if M["m00"] != 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # ある程度の大きさの円のみを処理
            if radius > 10:
                # 元のフレームに円と中心点を描画
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # 中心座標を表示
                text = f"X: {center[0]}, Y: {center[1]}"
                cv2.putText(frame, text, (center[0] - 50, center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                print(f"ボールの中心座標: X={center[0]}, Y={center[1]}")


    # 結果を表示
    cv2.imshow('Frame', frame)
    # cv2.imshow('Mask', mask) # マスクも表示したい場合

    # 'q'キーが押されたらループを終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 後処理
cap.release()
cv2.destroyAllWindows()