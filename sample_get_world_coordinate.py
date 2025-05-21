import numpy as np
import cv2
import os
import time

# --- 設定 ---
# 前回のキャリブレーション結果ファイル
calibration_file = 'camera_calibration.npz'

# チェスボードの内側のコーナーの数 (キャリブレーション時と同じ設定)
nx = 7
ny = 6

# チェスボードの正方形の一辺の実際のサイズ (キャリブレーション時と同じ単位)
# このサイズが世界座標系での単位となります (例: mm)
square_size = 25 # 例: 25 mm

# 描画する座標軸の長さ (SQUARE_SIZE を基準にすると良い)
axis_length = square_size * 3

# --- キャリブレーション結果の読み込み ---
if not os.path.exists(calibration_file):
    print(f"エラー: キャリブレーションファイル '{calibration_file}' が見つかりません。")
    print("前回のキャリブレーションを実行してファイルを作成してください。")
    exit()

mtx = np.array([[667.78589722327888,0.00000000000000,303.04460463605631],
                          [0.00000000000000,666.67910084463063,189.24609815186295],
                          [0.00000000000000,0.00000000000000,1.00000000000000]], dtype=np.float64)

dist = np.array([-0.09306442948031,0.43190686558612,0.00774182962887,-0.00843446225062,-0.46640650833319]) # 歪みがないと仮定

print("カメラ内部パラメータと歪み係数を読み込みました。")

# --- 世界座標系を定義するための3次元点の準備 ---
objp = np.zeros((ny * nx, 3), np.float32)
objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2) * square_size

# --- 3D軸の定義 (世界座標系) ---
# X軸(赤), Y軸(緑), Z軸(青) を描画
origin_3d  = np.float32([[0, 0, 0],                         # 原点
                          [axis_length, 0, 0],             # X軸の先端
                          [0, axis_length, 0],             # Y軸の先端
                          [0, 0, -axis_length]]).reshape(-1, 3) # Z軸の先端 (チェス盤から手前向き)

# 軸描画関数 (チュートリアル参考)
def draw_axes(img, imgpts):
    """
    画像に3D軸を描画する。
    img: 描画対象の画像
    origin_2d_pt: 軸の始点 (2D画像座標)
    axis_end_pts_2d: 各軸の終点 (2D画像座標のリスト)
    colors: 各軸の色 (BGRのタプルのリスト)
    """
    origin = tuple(map(int, imgpts[0].ravel()))
    x_axis_end = tuple(map(int, imgpts[1].ravel()))
    y_axis_end = tuple(map(int, imgpts[2].ravel()))
    z_axis_end = tuple(map(int, imgpts[3].ravel()))

    # X軸 (青)
    img = cv2.line(img, origin, x_axis_end, (255, 0, 0), 3) # BGR: 青
    # Y軸 (緑)
    img = cv2.line(img, origin, y_axis_end, (0, 255, 0), 3) # BGR: 緑
    # Z軸 (赤)
    img = cv2.line(img, origin, z_axis_end, (0, 0, 255), 3) # BGR: 赤

# --- カメラの初期化 ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("エラー: カメラを開けませんでした。カメラIDを確認してください。")
    exit()

print("\nカメラ映像を表示中...'1'キーで世界座標系を設定 (チェスボードを表示してください)。'q'キーで終了。")
print(f"設定するチェスボード: 内側コーナー {nx}x{ny}, 正方形サイズ {square_size} mm")

# --- 外部パラメータ格納変数 ---
rvec_w2c = None # 世界座標系 -> カメラ座標系への回転ベクトル
tvec_w2c = None # 世界座標系 -> カメラ座標系への並進ベクトル
world_frame_established = False # 世界座標系が設定されたかどうかのフラグ

# solvePnPに使用するコーナー検出精度向上のための基準
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

while True:
    ret, frame = cap.read()
    if not ret:
        print("エラー: フレームを読み込めませんでした。")
        time.sleep(0.1)
        continue

    display_frame = frame.copy()

    if world_frame_established and mtx is not None and dist is not None and \
       rvec_w2c is not None and tvec_w2c is not None:
        try:
            # 世界座標系の原点 (0,0,0) を画像平面に投影
            origin_2d, _ = cv2.projectPoints(origin_3d, rvec_w2c, tvec_w2c, mtx, dist)

            # 軸を描画
            draw_axes(display_frame, origin_2d)

            cv2.putText(display_frame, "World Frame Set", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

        except Exception as e:
            print(f"警告: 軸描画エラー: {e}")

    cv2.imshow('Camera Feed - Press 1 to Set World Frame, q to Quit', display_frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    if key == ord('1'):
        print("\n'1'キーが押されました。チェスボードを検出します...")
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret_corners, corners = cv2.findChessboardCorners(gray_frame, (nx, ny), None)

        if ret_corners == True:
            print("コーナーが見つかりました。solvePnPを実行します...")
            corners_subpix = cv2.cornerSubPix(gray_frame, corners, (11, 11), (-1, -1), criteria)

            try:
                ret_pnp, rvec_temp, tvec_temp = cv2.solvePnP(objp, corners_subpix, mtx, dist)

                if ret_pnp:
                    print("solvePnP成功。世界座標系が設定されました。")
                    rvec_w2c = rvec_temp
                    tvec_w2c = tvec_temp
                    world_frame_established = True

                    # 確認用ウィンドウ表示
                    img_confirm = frame.copy()
                    cv2.drawChessboardCorners(img_confirm, (nx, ny), corners_subpix, ret_corners)

                    # 確認用ウィンドウにも軸を描画
                    origin_2d_confirm, _ = cv2.projectPoints(origin_3d, rvec_w2c, tvec_w2c, mtx, dist)
                    draw_axes(img_confirm, origin_2d_confirm)

                    cv2.imshow('World Frame Set Confirmation', img_confirm)
                    print("設定された世界座標軸の確認ウィンドウを表示しました。このウィンドウを閉じると、メインのカメラ映像に戻ります。")
                    cv2.waitKey(0)
                    cv2.destroyWindow('World Frame Set Confirmation')
                else:
                    print("solvePnPが失敗しました。検出されたコーナーが不適切かもしれません。")
                    world_frame_established = False
            except Exception as e:
                print(f"エラー: solvePnP 実行中にエラーが発生しました: {e}")
                world_frame_established = False
        else:
            print("チェスボードコーナーが見つかりませんでした。")
            print("カメラにチェスボード全体が鮮明に写っているか確認してください。")
            world_frame_established = False # コーナーが見つからなければリセット

cap.release()
cv2.destroyAllWindows()
print("プログラムを終了しました。")