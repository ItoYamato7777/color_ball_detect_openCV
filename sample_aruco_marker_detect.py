import cv2
import numpy as np

# --- 設定 ---
# 使用するArUcoマーカーの辞書を指定します。
aruco_dict_type = cv2.aruco.DICT_4X4_50

# OpenCV 4.7以降の推奨される書き方
aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# マーカーの実際のサイズ（一辺の長さ、単位はメートルやセンチメートルなど任意だが、後述のtvecの単位に影響）
# 例: 5センチメートル = 0.05メートル
marker_length = 0.05

# カメラの内部パラメータ (カメラキャリブレーションで取得)
# *** 注意: ここはダミーの値です。必ずご自身のカメラでキャリブレーションして取得した値に置き換えてください。 ***
# カメラ行列 K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
camera_matrix = np.array([[1000.0, 0, 640.0],
                          [0, 1000.0, 480.0],
                          [0, 0, 1.0]]) # 例: 1280x960ピクセルの画像サイズを想定した適当な値

# 歪み係数 distCoeffs = [k1, k2, p1, p2, k3, ...]
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # 例: 歪みがほとんどない、または補正済みの想定 (ゼロにするのは一般的ではない)
# *** 実際のキャリブレーション結果を使用してください。 ***

# 座標軸を描画する際の軸の長さ (メートルなど、marker_lengthと同じ単位)
axis_length = 0.1 # 例: 10センチメートル

# --- カメラからの入力 ---
cap = cv2.VideoCapture(0) # 0はデフォルトのカメラ

if not cap.isOpened():
    print("エラー: カメラを開けません")
    exit()

print("ArUcoマーカー検出と姿勢推定を開始します。終了するには 'q' キーを押してください。")

# --- メインループ ---
while True:
    # フレームを1枚キャプチャ
    ret, frame = cap.read()
    if not ret:
        print("エラー: フレームをキャプチャできません")
        break

    # グレースケールに変換
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ArUcoマーカーを検出
    corners, ids, rejected = detector.detectMarkers(gray)

    # 検出されたマーカーがある場合
    if ids is not None:
        # 検出されたマーカーの姿勢を推定
        # rvecs: 各マーカーの回転ベクトル
        # tvecs: 各マーカーの並進ベクトル
        # objPoints: 各マーカーのモデル座標（通常は不要）
        rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        # 検出されたマーカーとID、そして座標軸をフレーム上に描画
        for i, marker_id in enumerate(ids):
            # マーカーのIDを取得
            marker_id = marker_id[0]

            # 検出されたマーカーとIDを描画
            cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[marker_id]]))

            # 姿勢推定の結果（rvecs[i]とtvecs[i]）を使って座標軸を描画
            # drawFrameAxesの引数: (画像, カメラ行列, 歪み係数, 回転ベクトル, 並進ベクトル, 軸の長さ)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], axis_length)

            # マーカーの中心座標を計算 (表示用)
            corners_of_marker = corners[i][0]
            center_x = int(np.mean(corners_of_marker[:, 0]))
            center_y = int(np.mean(corners_of_marker[:, 1]))
            center = (center_x, center_y)

            # 中心点に点を描画
            cv2.circle(frame, center, 5, (0, 0, 255), -1) # 赤い点を描画

            # マーカーのIDと姿勢推定結果（並進ベクトル）をフレーム上に表示
            # 並進ベクトル tvec は [X, Y, Z] の形式で、カメラを原点としたマーカーの位置（単位はmarker_lengthと同じ）
            text = f"ID: {marker_id}"
            text_pos = f"Pos: X={tvecs[i][0][0]:.2f}, Y={tvecs[i][0][1]:.2f}, Z={tvecs[i][0][2]:.2f}"
            cv2.putText(frame, text, (center_x - 50, center_y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) # 緑色でテキスト描画
            cv2.putText(frame, text_pos, (center_x - 50, center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2) # 黄色で位置表示

            # ターミナルにマーカーのIDと姿勢推定結果を出力
            print(f"検出: ArUco ID={marker_id}, 回転ベクトル(rvec): {rvecs[i].flatten()}, 並進ベクトル(tvec): {tvecs[i].flatten()}")

    # 結果を表示
    cv2.imshow('ArUco Marker Detection with Pose Estimation', frame)

    # 'q'キーが押されたらループを終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 後処理: カメラを解放し、全てのウィンドウを閉じる
cap.release()
cv2.destroyAllWindows()