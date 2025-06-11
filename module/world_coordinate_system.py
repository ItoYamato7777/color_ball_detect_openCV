# module/world_coordinate_system.py

import numpy as np
import cv2

class WorldCoordinateSystem:
    """
    チェスボードを用いて世界座標系を定義し、その座標軸を描画するクラス。
    """
    def __init__(self, mtx, dist, nx, ny, square_size, axis_length, criteria):
        """
        世界座標系設定に必要なパラメータを初期化します。

        Args:
            mtx (numpy.ndarray): カメラ内部パラメータ行列。
            dist (numpy.ndarray): 歪み係数。
            nx (int): チェスボードの内側コーナー数 (X方向)。
            ny (int): チェスボードの内側コーナー数 (Y方向)。
            square_size (float): チェスボードの正方形の一辺のサイズ (世界座標系の単位)。
            axis_length (float): 描画する座標軸の長さ。
            criteria (tuple): cv2.cornerSubPix のための criteria。
        """
        self.mtx = mtx
        self.dist = dist
        self.nx = nx #
        self.ny = ny #
        self.square_size = square_size #
        self.axis_length = axis_length #
        self.criteria = criteria #

        # 世界座標系を定義するための3次元点の準備 (objp)
        self.objp = np.zeros((self.ny * self.nx, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.nx, 0:self.ny].T.reshape(-1, 2) * self.square_size

        # 3D軸の定義 (世界座標系)
        # X軸(元コードコメント:青 -> 実際は (255,0,0) なのでBGRで青), Y軸(緑), Z軸(赤)
        # Z軸はチェス盤から手前向きに修正 (-axis_length)
        self.origin_3d = np.float32([[0, 0, 0],
                                     [self.axis_length, 0, 0],
                                     [0, self.axis_length, 0],
                                     [0, 0, -self.axis_length]]).reshape(-1, 3)

        self.rvec_w2c = None  # 世界座標系 -> カメラ座標系への回転ベクトル
        self.tvec_w2c = None  # 世界座標系 -> カメラ座標系への並進ベクトル
        self.world_frame_established = False

        print("WorldCoordinateSystem: 初期化完了。")

    def _draw_axes_on_image(self, img, imgpts):
        """
        画像に3D軸を描画するプライベートヘルパーメソッド。
        Args:
            img (numpy.ndarray): 描画対象の画像。
            imgpts (numpy.ndarray): 投影された軸の座標点 (原点、X終点、Y終点、Z終点)。
        """
        origin = tuple(map(int, imgpts[0].ravel()))
        x_axis_end = tuple(map(int, imgpts[1].ravel()))
        y_axis_end = tuple(map(int, imgpts[2].ravel()))
        z_axis_end = tuple(map(int, imgpts[3].ravel()))

        # <--- BGRカラーモデルに基づき、X軸を赤、Z軸を青に変更
        cv2.line(img, origin, x_axis_end, (0, 255, 0), 3)  # X軸 (緑)
        cv2.line(img, origin, y_axis_end, (0, 0, 255), 3)  # Y軸 (赤)
        cv2.line(img, origin, z_axis_end, (255, 0, 0), 3)  # Z軸 (青)
        return img

    def establish_world_frame(self, frame):
        """
        指定されたフレームからチェスボードを検出し、世界座標系を設定します。
        成功した場合、回転ベクトルと並進ベクトルを更新し、確認ウィンドウを表示します。

        Args:
            frame (numpy.ndarray): カメラフレーム。

        Returns:
            bool: 世界座標系の設定に成功した場合は True、失敗した場合は False。
        """
        print("\n世界座標系設定処理を開始します。チェスボードを検出中...")
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret_corners, corners = cv2.findChessboardCorners(gray_frame, (self.nx, self.ny), None)

        if ret_corners:
            print("チェスボードコーナーが見つかりました。solvePnPを実行します...")
            corners_subpix = cv2.cornerSubPix(gray_frame, corners, (11, 11), (-1, -1), self.criteria)

            try:
                ret_pnp, rvec_temp, tvec_temp = cv2.solvePnP(self.objp, corners_subpix, self.mtx, self.dist)

                if ret_pnp:
                    self.rvec_w2c = rvec_temp
                    self.tvec_w2c = tvec_temp
                    self.world_frame_established = True
                    print("solvePnP成功。世界座標系が設定されました。")

                    img_confirm = frame.copy()
                    cv2.drawChessboardCorners(img_confirm, (self.nx, self.ny), corners_subpix, ret_corners)

                    origin_2d_confirm, _ = cv2.projectPoints(self.origin_3d, self.rvec_w2c, self.tvec_w2c, self.mtx, self.dist)
                    self._draw_axes_on_image(img_confirm, origin_2d_confirm)

                    cv2.imshow('World Frame Set Confirmation', img_confirm)
                    print("設定された世界座標軸の確認ウィンドウを表示しました。このウィンドウを閉じる (何かキーを押す) と、メインのカメラ映像に戻ります。")
                    cv2.waitKey(0)
                    cv2.destroyWindow('World Frame Set Confirmation')
                    return True
                else:
                    print("solvePnPが失敗しました。")
                    self.world_frame_established = False
                    return False
            except Exception as e:
                print(f"エラー: solvePnP 実行中にエラーが発生しました: {e}")
                self.world_frame_established = False
                return False
        else:
            print("チェスボードコーナーが見つかりませんでした。")
            print(f"カメラに {self.nx}x{self.ny} のチェスボード全体が鮮明に写っているか確認してください。")
            self.world_frame_established = False
            return False

    def draw_world_axes(self, frame):
        """
        世界座標系が設定されていれば、指定されたフレームに座標軸を描画します。

        Args:
            frame (numpy.ndarray): 描画対象のカメラフレーム。

        Returns:
            numpy.ndarray: 座標軸が描画されたフレーム。
        """
        if self.world_frame_established and self.rvec_w2c is not None and self.tvec_w2c is not None:
            try:
                projected_pts, _ = cv2.projectPoints(self.origin_3d, self.rvec_w2c, self.tvec_w2c, self.mtx, self.dist)
                frame = self._draw_axes_on_image(frame, projected_pts)
                cv2.putText(frame, "World Frame Set", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            except Exception as e:
                print(f"警告: 世界座標軸の描画エラー: {e}")
        return frame