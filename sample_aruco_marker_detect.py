import cv2
import numpy as np

# 使用するArUcoマーカーの辞書を指定します。
# 例: DICT_6X6_250 (6x6のパターンでIDが0-249のマーカー)
# 使用するマーカーに合わせて変更してください。
# 主な辞書タイプ:
# DICT_4X4_50, DICT_4X4_100, DICT_4X4_250, DICT_4X4_1000
# DICT_5X5_50, DICT_5X5_100, DICT_5X5_250, DICT_5X5_1000
# DICT_6X6_50, DICT_6X6_100, DICT_6X6_250, DICT_6X6_1000
# DICT_7X7_50, DICT_7X7_100, DICT_7X7_250, DICT_7X7_1000
# DICT_ARUCO_ORIGINAL
aruco_dict_type = cv2.aruco.DICT_4X4_50
# aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type) # 過去のバージョン
# aruco_params = cv2.aruco.DetectorParameters_create() # 過去のバージョン

# OpenCV 4.7以降の推奨される書き方
aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


# カメラからの入力を開始 (0はデフォルトのカメラ)
# 動画ファイルから読み込む場合は、 'video.mp4' のようにファイルパスを指定
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("エラー: カメラを開けません")
    exit()

print("ArUcoマーカー検出を開始します。終了するには 'q' キーを押してください。")

while True:
    # フレームを1枚キャプチャ
    ret, frame = cap.read()
    if not ret:
        print("エラー: フレームをキャプチャできません")
        break

    # グレースケールに変換 (検出処理のため、必須ではありませんが多くの場合推奨されます)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ArUcoマーカーを検出
    # 戻り値: corners (検出されたマーカーの角), ids (マーカーのID), rejected (検出されなかった候補)
    # corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params) # 過去のバージョン
    corners, ids, rejected = detector.detectMarkers(gray) # OpenCV 4.7以降

    # 検出されたマーカーがある場合
    if ids is not None:
        # 検出されたマーカーとIDをフレーム上に描画
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # 各検出されたマーカーの情報を処理
        for i, corner in enumerate(corners):
            # マーカーのIDを取得
            marker_id = ids[i][0]

            # マーカーの4つの角の座標を取得
            # corner[0] は numpy 配列で [[x1, y1], [x2, y2], [x3, y3], [x4, y4]] の形式
            corners_of_marker = corner[0]

            # マーカーの中心座標を計算 (4つの角の平均)
            center_x = int(np.mean(corners_of_marker[:, 0]))
            center_y = int(np.mean(corners_of_marker[:, 1]))
            center = (center_x, center_y)

            # 中心点に点を描画
            cv2.circle(frame, center, 5, (0, 0, 255), -1) # 赤い点を描画

            # マーカーのIDと中心座標をフレーム上に表示
            text = f"ID: {marker_id}, Center: ({center_x}, {center_y})"
            # cv2.putText()の引数: (画像, 表示する文字列, 表示開始座標, フォント, 文字サイズ, 色, 太さ)
            cv2.putText(frame, text, (center_x - 50, center_y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) # 緑色でテキスト描画

            # ターミナルにマーカーのIDと中心座標を出力
            print(f"検出: ArUco ID={marker_id}, 中心座標: X={center_x}, Y={center_y}")


    # 結果を表示
    cv2.imshow('ArUco Marker Detection', frame)

    # 'q'キーが押されたらループを終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 後処理: カメラを解放し、全てのウィンドウを閉じる
cap.release()
cv2.destroyAllWindows()