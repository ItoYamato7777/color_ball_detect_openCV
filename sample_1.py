import cv2
import numpy as np
import glob

# Load previously saved data
with np.load('camera_calibration.npz') as X:
    mtx = X['mtx']
    dist = X['dist']

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5) # X軸 (赤)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5) # Y軸 (緑)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5) # Z軸 (青)
    return img

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Z軸をチェスボードから離れる方向に設定（カメラ方向とは逆）
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

# 表示ウィンドウの最大幅と高さを定義 (例: 画面サイズの80%など)
# 環境に合わせて調整してください。
# 例えば、フルHDディスプレイなら width_limit=1920*0.8, height_limit=1080*0.8
MAX_DISPLAY_WIDTH = 1280
MAX_DISPLAY_HEIGHT = 720 # 一般的なノートPCの解像度などを想定

for fname in glob.glob('*.jpg'):
    img = cv2.imread(fname)
    if img is None: # 画像が読み込めなかった場合の処理を追加
        print(f"Error: Could not read image {fname}")
        continue

    # 表示用に画像をリサイズする処理を追加
    display_img = img.copy() # オリジナルのimgは変更しないでおく
    h, w = display_img.shape[:2]

    if w > MAX_DISPLAY_WIDTH or h > MAX_DISPLAY_HEIGHT:
        # 縦横比を維持してリサイズ
        scale = min(MAX_DISPLAY_WIDTH / w, MAX_DISPLAY_HEIGHT / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        display_img = cv2.resize(display_img, (new_w, new_h), interpolation=cv2.INTER_AREA)

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # グレースケール変換はオリジナル画像に対して行う
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Find the rotation and translation vectors.
        ret_pnp, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
        
        ret_pnp = True

        # solvePnPRansacが成功した場合のみ描画
        if ret_pnp:
            # project 3D points to image plane
            # ここで投影された軸は、リサイズ前のオリジナル画像座標系に基づいている
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

            # draw関数に渡す前に、imgptsの座標をリサイズ後の画像に合わせる必要あり
            # これを適切に行うには、元のimgptsの座標をscaleでスケーリングする必要がある
            # ただし、drawFrameAxesの方が簡単
            # drawFrameAxesを使用する場合は、imgptsの変換は不要
            # img = draw(img,corners2,imgpts) # 軸の描画はオリジナル画像に対して行われる

            # ★推奨: cv2.drawFrameAxes を使用する方が簡単で正確
            # imgに対して直接 drawFrameAxes を呼び出す
            display_img = cv2.drawFrameAxes(img, mtx, dist, rvecs, tvecs, 3, 5) # 長さ3、太さ5 (例)

            # 表示用に画像をリサイズする処理を再度追加 (drawFrameAxes適用後)
            h, w = display_img.shape[:2]
            if w > MAX_DISPLAY_WIDTH or h > MAX_DISPLAY_HEIGHT:
                scale = min(MAX_DISPLAY_WIDTH / w, MAX_DISPLAY_HEIGHT / h)
                new_w = int(w * scale)
                new_h = int(h * scale)
                display_img = cv2.resize(display_img, (new_w, new_h), interpolation=cv2.INTER_AREA)


            cv2.imshow('img',display_img) # リサイズした画像を表示
            k = cv2.waitKey(0) & 0xff
            if k == ord('s'):
                # 保存はオリジナルサイズで行いたい場合は、imgを保存
                # リサイズした画像を保存したい場合は、display_imgを保存
                cv2.imwrite(fname[:6]+'.png', img) # オリジナルサイズで保存
        else:
            print(f"solvePnPRansac failed for {fname}")
            cv2.imshow('img', display_img) # 失敗してもリサイズした画像を表示
            cv2.waitKey(0)
    else:
        print(f"Chessboard not found in {fname}")
        cv2.imshow('img', display_img) # チェスボードが見つからなくてもリサイズした画像を表示
        cv2.waitKey(0)

cv2.destroyAllWindows()