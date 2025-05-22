import numpy as np
import cv2

class BallWorldTranslator:
    """
    検出されたボールの2D画像座標を3D世界座標に変換し、
    その情報を描画する機能を提供するクラス。
    """
    def __init__(self, mtx, dist, ball_radius_world):
        """
        BallWorldTranslatorを初期化します。

        Args:
            mtx (numpy.ndarray): カメラ内部パラメータ行列 (3x3)。
            dist (numpy.ndarray): カメラ歪み係数 (1xN or Nx1)。
            ball_radius_world (float): ボールの物理的な半径 (世界座標系の単位、例: mm)。
                                      PDFに基づき、ボール中心のZ座標は -ball_radius_world となる。
        """
        self.mtx = mtx
        self.dist = dist
        self.ball_radius_world = ball_radius_world # ball_h に相当
        self.ball_z_world = -ball_radius_world   # 世界座標系におけるボール中心のZ座標

        if self.mtx is None or self.dist is None:
            raise ValueError("カメラ内部パラメータ (mtx) または歪み係数 (dist) が提供されていません。")
        print(f"BallWorldTranslator: 初期化完了。ボール半径: {self.ball_radius_world}, ボールZ座標(世界): {self.ball_z_world}")

    def calculate_world_coords(self, ball_center_uv, rvec_w2c, tvec_w2c):
        """
        単一のボールの2D画像中心座標から3D世界座標 (X, Y) を計算します。
        Z座標は初期化時に設定された self.ball_z_world を使用します。

        Args:
            ball_center_uv (tuple): ボールの2D画像中心座標 (u, v)。
            rvec_w2c (numpy.ndarray): 世界座標系からカメラ座標系への回転ベクトル (3x1)。
            tvec_w2c (numpy.ndarray): 世界座標系からカメラ座標系への並進ベクトル (3x1)。

        Returns:
            numpy.ndarray or None: 計算されたボールの3D世界座標 (X, Y, Z_world)。
                                   計算に失敗した場合は None。
        """
        if rvec_w2c is None or tvec_w2c is None:
            # print("BallWorldTranslator: 世界座標系の外部パラメータ (rvec, tvec) が未設定のため計算できません。")
            return None

        u, v = ball_center_uv

        # 1. 画像座標の歪み補正 (undistortPointsは正規化座標を返すためP=mtxでピクセル座標に戻す)
        # 入力は (1, N, 2) の形状である必要があるので、(1, 1, 2) に reshape
        uv_raw_pixel = np.array([[[float(u), float(v)]]], dtype=np.float32)
        uv_undistorted_pixel = cv2.undistortPoints(uv_raw_pixel, self.mtx, self.dist, P=self.mtx)
        
        if uv_undistorted_pixel is None or len(uv_undistorted_pixel) == 0:
            print(f"BallWorldTranslator: 点 ({u},{v}) の歪み補正に失敗しました。")
            return None
            
        u_prime = uv_undistorted_pixel[0, 0, 0]
        v_prime = uv_undistorted_pixel[0, 0, 1]

        # 2. 回転行列の計算
        R_w2c, _ = cv2.Rodrigues(rvec_w2c) # (3x3)

        # 3. 変換用行列 M_proj = mtx @ H_target の構築
        # H_target = [r1 | r2 | tvec_w2c - r3 * ball_radius_world]
        # ここで Z_world = -ball_radius_world を使うので、
        # H_target = [r1 | r2 | tvec_w2c + r3 * Z_world_ball_center]
        #  = [ R_w2c[:,0], R_w2c[:,1], tvec_w2c.flatten() + R_w2c[:,2] * self.ball_z_world ]
        
        r1 = R_w2c[:, 0]
        r2 = R_w2c[:, 1]
        r3 = R_w2c[:, 2]
        
        # tvec_w2c は (3x1) なので .flatten() で (3,) にする
        # Z_world は self.ball_z_world を使用
        col3_H = tvec_w2c.flatten() + r3 * self.ball_z_world 
        
        H_target = np.vstack((r1, r2, col3_H)).T # (3x3)
        
        M_proj = self.mtx @ H_target # (3x3)

        # 4. 連立一次方程式の作成と解法
        # s * u_prime = m11*X + m12*Y + m13
        # s * v_prime = m21*X + m22*Y + m23
        # s         = m31*X + m32*Y + m33
        # これを X, Y について解く
        # (m11 - u_prime*m31)X + (m12 - u_prime*m32)Y = u_prime*m33 - m13
        # (m21 - v_prime*m31)X + (m22 - v_prime*m32)Y = v_prime*m33 - m23
        
        m = M_proj # エイリアス
        
        # 係数行列 A_eq と 右辺ベクトル B_eq (A_eq * [X;Y] = B_eq)
        A_eq = np.array([
            [m[0,0] - u_prime * m[2,0], m[0,1] - u_prime * m[2,1]],
            [m[1,0] - v_prime * m[2,0], m[1,1] - v_prime * m[2,1]]
        ])
        
        B_eq = np.array([
            u_prime * m[2,2] - m[0,2],
            v_prime * m[2,2] - m[1,2]
        ])
        
        try:
            XY_world = np.linalg.solve(A_eq, B_eq)
            X_world = XY_world[0]
            Y_world = XY_world[1]
            # print(f"BallWorldTranslator: ({u},{v}) -> World (X,Y,Z): ({X_world:.2f}, {Y_world:.2f}, {self.ball_z_world:.2f})")
            return np.array([X_world, Y_world, self.ball_z_world])
        except np.linalg.LinAlgError:
            # print(f"BallWorldTranslator: 点 ({u_prime},{v_prime}) の世界座標計算で線形代数エラー（特異行列など）。")
            return None

    def draw_world_coordinates_on_frame(self, frame, detected_balls_info_with_world):
        """
        検出されたボールの2D画像座標の近くに、計算された3D世界座標を描画します。

        Args:
            frame (numpy.ndarray): 描画対象のカメラフレーム。
            detected_balls_info_with_world (list): 
                各ボールの情報（'name', 'center_uv', 'world_xyz' を含む辞書）のリスト。
                'world_xyz' が None の場合は描画しません。

        Returns:
            numpy.ndarray: 3D世界座標が描画されたフレーム。
        """
        for ball_info in detected_balls_info_with_world:
            center_uv = ball_info.get('center_uv')
            world_xyz = ball_info.get('world_xyz')
            ball_name = ball_info.get('name', '')

            if center_uv is not None and world_xyz is not None:
                text_x = center_uv[0]
                text_y = center_uv[1] + 20 # ボールの少し下に表示

                world_coord_text = f"W:({world_xyz[0]:.0f},{world_xyz[1]:.0f},{world_xyz[2]:.0f})mm"
                
                cv2.putText(frame, world_coord_text, (text_x, text_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1, cv2.LINE_AA)
        return frame