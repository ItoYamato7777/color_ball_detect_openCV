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

# 世界座標系の軸の描画長さ (solvePnPで使った単位に合わせる)
# axis_length を square_size より十分大きくする
axis_length = 100 # 例: 100 mm

# --- キャリブレーション結果の読み込み ---
if not os.path.exists(calibration_file):
    print(f"エラー: キャリブレーションファイル '{calibration_file}' が見つかりません。")
    print("前回のキャリブレーションを実行してファイルを作成してください。")
    exit()

try:
    with np.load(calibration_file) as X:
        mtx = X['mtx']
        dist = X['dist']
        # img_size = tuple(X['img_size']) # 画像サイズも読み込んでおくと良い
except Exception as e:
    print(f"エラー: キャリブレーションファイル '{calibration_file}' の読み込み中にエラーが発生しました: {e}")
    exit()


print("カメラ内部パラメータと歪み係数を読み込みました。")


# --- 世界座標系を定義するための3次元点の準備 ---
objp = np.zeros((ny*nx, 3), np.float32)
objp[:, :2] = np.mgrid[0:ny, 0:nx].T.reshape(-1, 2) * square_size


# --- カメラの初期化 ---
cap = cv2.VideoCapture(0) # 0は通常PCのデフォルトカメラ。必要に応じて変更してください。
# カメラによってはキャプチャ前に解像度を設定すると安定することがあります
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


if not cap.isOpened():
    print("エラー: カメラを開けませんでした。カメラIDを確認してください。")
    exit()

print("\nカメラ映像を表示中...'1'キーで世界座標系を設定 (チェスボードを表示してください)。'q'キーで終了。")
print(f"設定するチェスボード: 内側コーナー {nx}x{ny}, 正方形サイズ {square_size} (単位:solvePnPの単位)")


# --- 外部パラメータ格納変数 ---
rvec_w2c = None # 世界座標系 -> カメラ座標系への回転ベクトル
tvec_w2c = None # 世界座標系 -> カメラ座標系への並進ベクトル
world_frame_established = False # 世界座標系が設定されたかどうかのフラグ

# solvePnPに使用するコーナー検出精度向上のための基準
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


while True:
    # フレームの読み込み
    ret, frame = cap.read()

    if not ret:
        print("エラー: フレームを読み込めませんでした。")
        # カメラが切断された可能性などを考慮し、少し待ってリトライするか検討
        time.sleep(0.1)
        continue # 次のフレームへスキップ

    # 表示用にフレームをコピー
    display_frame = frame.copy()

    # 世界座標系が設定済みで、カメラ行列が有効なら軸を描画
    if world_frame_established and mtx is not None and dist is not None:
        try:
            # drawFrameAxesは、指定されたrvec, tvecで定義される座標系の軸を、
            # カメラmtx, distを通してどのように見えるか描画します。
            # ここではrvec_w2c, tvec_w2c (世界座標系->カメラ座標系) を使うことで、
            # 世界座標系の原点 (チェスボードの左上内側コーナー) に軸を描画できます。
            cv2.drawFrameAxes(display_frame, mtx, dist, rvec_w2c, tvec_w2c, axis_length, thickness=3)

            # 軸が描画されていることを示すテキスト
            cv2.putText(display_frame, "World Frame Set", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        except Exception as e:
             # 描画中にエラーが発生した場合（稀だがパラメータ異常など）
             print(f"警告: drawFrameAxes 描画エラー: {e}")
             # エラーが出てもループは止めない

    # 映像の表示
    cv2.imshow('Camera Feed - Press 1 to Set World Frame, q to Quit', display_frame)

    # キー入力の待機
    key = cv2.waitKey(1) & 0xFF # 1ms待機

    # 'q' キーで終了
    if key == ord('q'):
        break

    # '1' キーで世界座標系を設定 (チェスボードを検出)
    if key == ord('1'):
        print("\n'1'キーが押されました。チェスボードを検出します...")
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # チェスボードコーナーの検出
        # flags=cv2.CALIB_CB_FAST_CHECK を追加すると高速化される場合がありますが、
        # 精度が若干落ちる可能性もあります。まずはNoneで。
        ret_corners, corners = cv2.findChessboardCorners(gray_frame, (nx, ny), None)

        if ret_corners == True:
            print("コーナーが見つかりました。solvePnPを実行します...")

            # 見つかったコーナーの画像座標の精度向上
            corners_subpix = cv2.cornerSubPix(gray_frame, corners, (11, 11), (-1, -1), criteria)

            # solvePnPで外部パラメータ (世界座標系 -> カメラ座標系) を計算
            # objp: 定義した世界座標系での3D点 (チェスボードコーナー)
            # corners_subpix: その画像上での2D点
            # mtx, dist: カメラの内部パラメータ
            # 結果: rvec_w2c, tvec_w2c (世界座標系から見たカメラの位置・向き)
            try:
                ret_pnp, rvec_w2c_temp, tvec_w2c_temp = cv2.solvePnP(objp, corners_subpix, mtx, dist)

                if ret_pnp:
                    print("solvePnP成功。世界座標系が設定されました。")
                    # 成功した場合のみ、外部パラメータを更新
                    rvec_w2c = rvec_w2c_temp
                    tvec_w2c = tvec_w2c_temp
                    world_frame_established = True
                    # 外部パラメータはループ外の変数に格納されたままなので、以降のフレームで使用されます。

                    # (オプション) 設定された瞬間の画像に検出されたコーナーと軸を描画して表示
                    # 検出確認
                    img_confirm = frame.copy()
                    cv2.drawChessboardCorners(img_confirm, (nx, ny), corners_subpix, ret_corners)
                    # 軸描画確認 (計算した外部パラメータを使用)
                    #TODO 外部パラメータではなくチェスコーナーを使用する
                    cv2.drawFrameAxes(img_confirm, mtx, dist, rvec_w2c, tvec_w2c, axis_length, thickness=3)
                    cv2.imshow('World Frame Set Confirmation', img_confirm)
                    print("設定された世界座標軸の確認ウィンドウを表示しました。このウィンドウを閉じると、メインのカメラ映像に戻ります。")
                    cv2.waitKey(0) # 確認表示の間、キー入力を待つ
                    cv2.destroyWindow('World Frame Set Confirmation') # 確認ウィンドウを閉じる

                else:
                    print("solvePnPが失敗しました。検出されたコーナーが不適切かもしれません。")
            except Exception as e:
                 print(f"エラー: solvePnP 実行中にエラーが発生しました: {e}")


        else:
            print("チェスボードコーナーが見つかりませんでした。")
            print("カメラにチェスボード全体が鮮明に写っているか確認してください。")


# --- 終了処理 ---
cap.release()
cv2.destroyAllWindows()
print("プログラムを終了しました。")