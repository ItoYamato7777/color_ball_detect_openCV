import numpy as np
import cv2
import os

# --- 設定 ---
# 前回のキャリブレーション結果ファイル
calibration_file = 'camera_calibration.npz'

# 世界座標系を定義するために撮影したチェスボード画像ファイル
# この画像でのチェスボードの位置・向きが、世界座標系の基準となります
world_origin_image = 'world_origin_01.jpg' # 実際のファイル名に合わせてください

# チェスボードの内側のコーナーの数 (前回のキャリブレーションと同じ設定)
nx = 7
ny = 6

# チェスボードの正方形の一辺の実際のサイズ (前回のキャリブレーションと同じ単位)
square_size = 25 # 例: 25 mm

# 世界座標系を定義するための外部パラメータを保存するファイル名
world_transform_file = 'world_transform.npz'

# --- キャリブレーション結果の読み込み ---
if not os.path.exists(calibration_file):
    print(f"エラー: キャリブレーションファイル '{calibration_file}' が見つかりません。")
    print("前回のキャリブレーションを実行してファイルを作成してください。")
    exit()

with np.load(calibration_file) as X:
    mtx, dist = [X[i] for i in ('mtx', 'dist')]
    # img_size なども必要に応じて読み込みます

print("カメラ内部パラメータと歪み係数を読み込みました。")
print("カメラ行列 (mtx):")
print(mtx)
print("\n歪み係数 (dist):")
print(dist)

# --- 世界座標系を定義するための3次元点の準備 ---
# この3次元座標リストが、定義する世界座標系におけるチェスボードコーナーの位置となります
# 例: チェスボードの左上を原点 (0,0,0) とし、X軸、Y軸がチェスボードに沿う場合
objp = np.zeros((ny*nx, 3), np.float32)
objp[:, :2] = np.mgrid[0:ny, 0:nx].T.reshape(-1, 2) * square_size

# --- 世界座標系定義用画像の読み込みと処理 ---
img = cv2.imread(world_origin_image)
if img is None:
     print(f"エラー: 世界座標系定義用画像 '{world_origin_image}' の読み込みに失敗しました。")
     exit()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_size = (img.shape[1], img.shape[0]) # 画像サイズを取得

# チェスボードコーナーの検出
ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

if ret == True:
    print(f"'{world_origin_image}': コーナーが見つかりました。")

    # 見つかったコーナーの画像座標の精度向上
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners_subpix = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    # (オプション) 見つかったコーナーを描画して確認
    cv2.drawChessboardCorners(img, (nx, ny), corners_subpix, ret)
    cv2.imshow('World Origin Chessboard', img)
    print("検出されたコーナーを表示しました。キーを押すと続行します。")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # --- solvePnPで外部パラメータを取得 ---
    # objp: 世界座標系 (として定義したチェスボード座標系) での3D点
    # corners_subpix: 画像座標系での2D点
    # mtx: カメラ行列 (内部パラメータ)
    # dist: 歪み係数
    # 結果として、rvec (回転ベクトル) と tvec (並進ベクトル) が得られます。
    # これらは「世界座標系からカメラ座標系への変換」(r_w2c, t_w2c) を表します。
    ret, rvec_w2c, tvec_w2c = cv2.solvePnP(objp, corners_subpix, mtx, dist)

    print("\nsolvePnP 結果 (世界座標系 -> カメラ座標系):")
    print("回転ベクトル (rvec_w2c):")
    print(rvec_w2c)
    print("\n並進ベクトル (tvec_w2c):")
    print(tvec_w2c)

    # --- カメラ座標系から世界座標系への変換パラメータを計算 ---
    # rvec_w2c を回転行列 R_w2c に変換
    R_w2c, _ = cv2.Rodrigues(rvec_w2c)

    # 逆変換を計算 (カメラ座標系 -> 世界座標系)
    R_c2w = R_w2c.T # 回転行列の逆は転置
    t_c2w = -R_c2w @ tvec_w2c # t_c2w = -R_c2w * t_w2c

    print("\n計算された逆変換パラメータ (カメラ座標系 -> 世界座標系):")
    print("回転行列 (R_c2w):")
    print(R_c2w)
    print("\n並進ベクトル (t_c2w):")
    print(t_c2w)

    # --- 結果の保存 ---
    print(f"\n世界座標系への変換パラメータを '{world_transform_file}' に保存します...")
    np.savez(world_transform_file, R_c2w=R_c2w, t_c2w=t_c2w, rvec_w2c=rvec_w2c, tvec_w2c=tvec_w2c)

    print("完了しました。")

else:
    print(f"エラー: '{world_origin_image}' 内でチェスボードコーナーが見つかりませんでした。")
    print("画像を確認し、チェスボード全体が鮮明に写っているか確認してください。")