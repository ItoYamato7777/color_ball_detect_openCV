import numpy as np
import cv2
import glob
import os

# --- 設定 ---
# チェスボードの内側のコーナーの数 (横方向, 縦方向)
# 例: 8x6のチェスボードなら (7, 5)
nx = 7
ny = 5

# チェスボードの正方形の一辺の実際のサイズ (任意の単位, 例: mm)
# この単位が世界座標系の単位になります
square_size = 20 # 例: 20 mm

# キャリブレーション画像が保存されているフォルダ
images_folder = 'calibration_images'

# キャリブレーション結果を保存するファイル名
calibration_file = 'camera_calibration.npz'

# --- 3次元のキャリブレーションポイントの準備 ---
# チェスボード座標系におけるコーナーの3次元座標
# (0,0,0), (s,0,0), (2s,0,0), ..., (nx*s, (ny-1)*s, 0) となるように生成
# グリッドの点は Z=0 の平面上にあると仮定します
objp = np.zeros((ny*nx, 3), np.float32)
# np.mgrid[0:ny, 0:nx] は ny x nx のグリッド座標行列を生成します
# reshape(-1, 2) で (ny*nx, 2) の形状に変換します
# * square_size で実際のサイズにスケーリングします
objp[:, :2] = np.mgrid[0:ny, 0:nx].T.reshape(-1, 2) * square_size

# --- キャリブレーションポイントを格納するリスト ---
# 世界座標系における3D点
objpoints = [] # 3d point in real world space
# 画像座標系における2D点
imgpoints = [] # 2d points in image plane

# --- キャリブレーション画像の読み込みと処理 ---
# 指定フォルダ内のJPG画像をすべて取得
images = glob.glob(os.path.join(images_folder, '*.jpg'))

if not images:
    print(f"エラー: '{images_folder}' フォルダに画像が見つかりませんでした。")
    print("キャリブレーション画像をフォルダに入れて実行してください。")
    exit()

print(f"{len(images)} 枚の画像を処理します...")

# 画像サイズを保持するため、最初の画像を読み込む
img = cv2.imread(images[0])
if img is None:
     print(f"エラー: 画像 '{images[0]}' の読み込みに失敗しました。")
     exit()
img_size = (img.shape[1], img.shape[0]) # (width, height)

found_count = 0 # コーナーが見つかった画像の数

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"警告: 画像 '{fname}' の読み込みに失敗しました。スキップします。")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # チェスボードコーナーの検出
    # ret: 見つかったかどうかの真偽値
    # corners: 見つかったコーナーの画像座標リスト
    ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

    # コーナーが見つかった場合
    if ret == True:
        found_count += 1
        print(f"'{fname}': コーナーが見つかりました ({found_count} / {len(images)})")

        # objpoints に世界座標系の3D点を追加
        objpoints.append(objp)

        # 見つかったコーナーの画像座標の精度向上 (サブピクセル推定)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # corners_subpix に精度向上後の2D画像座標リストが格納される
        corners_subpix = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # imgpoints に精度向上後の2D点を追加
        imgpoints.append(corners_subpix)

        # (オプション) 見つかったコーナーを描画して確認
        cv2.drawChessboardCorners(img, (nx, ny), corners_subpix, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500) # 500ミリ秒表示

    else:
        print(f"'{fname}': コーナーが見つかりませんでした。")

# cv2.destroyAllWindows() # オプション表示ウィンドウを閉じる場合

if found_count < 10: # 適切な枚数は必要に応じて調整
    print("\nエラー: キャリブレーションに必要な数のコーナーが検出できませんでした。")
    print("異なる角度や距離からより多くの画像を撮影するか、チェスボードを鮮明に写してください。")
    exit()

# --- カメラキャリブレーションの実行 ---
# objpoints: 世界座標系の3D点リスト
# imgpoints: 画像座標系の2D点リスト
# img_size: 画像サイズ
# mtx: カメラ行列 (出力)
# dist: 歪み係数 (出力)
# rvecs: 各画像における回転ベクトル (出力)
# tvecs: 各画像における並進ベクトル (出力)
print("\nキャリブレーションを実行中です...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

# --- 結果の表示 ---
print("\n--- キャリブレーション結果 ---")
print(f"RMS 再投影誤差: {ret}") # 値が小さいほど精度が高い (理想は1.0未満など)
print("\nカメラ行列 (mtx):")
print(mtx)
print("\n歪み係数 (dist):")
print(dist)
# rvecs, tvecsは各画像に対する外部パラメータですが、キャリブレーションの主目的は mtx, distの取得です

# --- 結果の保存 ---
print(f"\nキャリブレーション結果を '{calibration_file}' に保存します...")
np.savez(calibration_file, mtx=mtx, dist=dist, rms=ret, img_size=img_size)

print("完了しました。")

# --- (参考) キャリブレーション結果を使った画像の歪み補正 ---
# 保存したファイルからパラメータを読み込む場合:
# with np.load(calibration_file) as X:
#     mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rms','img_size')]

# サンプル画像を読み込み
# test_img_path = images[0] # 最初の画像を例として使う
# img = cv2.imread(test_img_path)
# if img is not None:
#     # 歪み補正を実行
#     undistorted_img = cv2.undistort(img, mtx, dist)

#     # 元画像と補正後画像を並べて表示
#     combined_img = np.hstack((img, undistorted_img))
#     cv2.imshow("Original vs Undistorted", combined_img)
#     print("\n歪み補正された画像を表示しました。キーを押すと終了します。")
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
# else:
#     print(f"\n参考画像の読み込みに失敗しました: {test_img_path}")