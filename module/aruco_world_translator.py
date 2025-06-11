import numpy as np
import cv2

class ArucoWorldTranslator:
    """
    検出されたArUcoマーカーのカメラ座標系における姿勢を、
    世界座標系における位置と向きに変換する機能を提供するクラス。
    """
    def __init__(self):
        """
        ArucoWorldTranslatorを初期化します。
        """
        print("ArucoWorldTranslator: 初期化完了。")

    def calculate_marker_world_pose(self, rvec_m2c, tvec_m2c, rvec_w2c, tvec_w2c):
        """
        マーカーのカメラ座標系での姿勢 (rvec_m2c, tvec_m2c) と
        世界座標系のカメラ座標系での姿勢 (rvec_w2c, tvec_w2c) を用いて、
        マーカーの世界座標系における位置と向き（回転行列）を計算します。

        Args:
            rvec_m2c (numpy.ndarray): マーカーからカメラ座標系への回転ベクトル (3x1)。
            tvec_m2c (numpy.ndarray): マーカーからカメラ座標系への並進ベクトル (3x1)。
            rvec_w2c (numpy.ndarray): 世界座標系からカメラ座標系への回転ベクトル (3x1)。
            tvec_w2c (numpy.ndarray): 世界座標系からカメラ座標系への並進ベクトル (3x1)。

        Returns:
            tuple (numpy.ndarray, numpy.ndarray) or (None, None):
                - P_marker_world (numpy.ndarray): マーカー原点の世界座標 (X, Y, Z) (3x1)。
                - R_w2m (numpy.ndarray): 世界座標系からマーカー座標系への回転行列 (3x3)。
                計算に失敗した場合は (None, None)。
        """
        if rvec_m2c is None or tvec_m2c is None or rvec_w2c is None or tvec_w2c is None:
            return None, None

        try:
            # 回転ベクトルを回転行列に変換
            R_m2c, _ = cv2.Rodrigues(rvec_m2c)  # マーカー座標系 -> カメラ座標系
            R_w2c, _ = cv2.Rodrigues(rvec_w2c)  # 世界座標系 -> カメラ座標系

            # R_w2c の逆行列 R_c2w (カメラ座標系 -> 世界座標系)
            R_c2w = R_w2c.T # 回転行列の逆行列は転置行列

            # マーカー原点の世界座標 P_marker_world = R_c2w @ (t_m2c - t_w2c)
            # tvecs は (3,1) 配列なので、(3,) に変形してから計算する
            P_marker_world = R_c2w @ (tvec_m2c.reshape(3) - tvec_w2c.reshape(3))
            P_marker_world = P_marker_world.reshape(3, 1) # (3x1) に戻す

            # 世界座標系から見たマーカーの向き R_w2m = R_c2w @ R_m2c
            R_w2m = R_c2w @ R_m2c
            
            return P_marker_world, R_w2m
        except Exception as e:
            print(f"ArucoWorldTranslator: 世界座標変換中にエラーが発生: {e}")
            return None, None

    def draw_world_pose_on_frame(self, frame, detected_markers_info_with_world):
        """
        検出・変換されたマーカーの世界座標位置と向きの情報を描画します。

        Args:
            frame (numpy.ndarray): 描画対象のカメラフレーム。
            detected_markers_info_with_world (list):
                各マーカーの情報（'id', 'center_uv', 'world_position', 'world_orientation_matrix' を含む辞書）のリスト。
                'world_position' や 'world_orientation_matrix' が None の場合は一部または全部を描画しません。

        Returns:
            numpy.ndarray: 情報が描画されたフレーム。
        """
        for marker_info in detected_markers_info_with_world:
            center_uv = marker_info.get('center_uv')
            marker_id = marker_info.get('id')
            world_pos = marker_info.get('world_position') # (3x1) array or None
            world_rot_mat = marker_info.get('world_orientation_matrix') # (3x3) array or None

            if center_uv is None or marker_id is None:
                continue

            text_x = center_uv[0]
            text_y = center_uv[1] + 20  # マーカーの少し下に表示

            if world_pos is not None:
                pos_text = f"ID:{marker_id} WPos:({world_pos[0,0]:.0f},{world_pos[1,0]:.0f},{world_pos[2,0]:.0f})mm"
                cv2.putText(frame, pos_text, (text_x - 70, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1, cv2.LINE_AA) # Magenta
                text_y += 15 # 次の行へ

            # if world_rot_mat is not None:
                # 簡単のため、マーカーのZ軸が世界座標系のどの方向を向いているかを表示
                # マーカー座標系のZ軸は (0,0,1)^T
                # これを世界座標系で表現すると R_w2m @ (0,0,1)^T = R_w2m の3列目
                # marker_z_axis_in_world = world_rot_mat[:, 2]
                # orientation_text = f"WZax:({marker_z_axis_in_world[0]:.2f},{marker_z_axis_in_world[1]:.2f},{marker_z_axis_in_world[2]:.2f})"
                # cv2.putText(frame, orientation_text, (text_x - 70, text_y),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 100, 0), 1, cv2.LINE_AA) # Blue-ish

        return frame