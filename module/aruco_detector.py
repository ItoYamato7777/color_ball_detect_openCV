import numpy as np
import cv2

class ArucoDetector:
    """
    ArUcoマーカーを検出し、そのIDと姿勢（カメラ座標系における位置と向き）を推定・描画するクラス。
    検出したマーカーの情報をリストとして返す機能を追加。
    """
    def __init__(self, camera_matrix, dist_coeffs, aruco_dict_type, marker_length, axis_length):
        """
        ArUcoマーカー検出器を初期化します。

        Args:
            camera_matrix (numpy.ndarray): カメラ内部パラメータ行列。
            dist_coeffs (numpy.ndarray): 歪み係数。
            aruco_dict_type (int): 使用するArUco辞書のタイプ (例: cv2.aruco.DICT_4X4_50)。
            marker_length (float): マーカーの一辺の実際の長さ (例: メートル)。
            axis_length (float): マーカーに描画する座標軸の長さ (marker_length と同じ単位)。
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_length = marker_length
        self.axis_length_on_marker = axis_length 

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type) 
        self.aruco_params = cv2.aruco.DetectorParameters() 
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params) 
        print("ArucoDetector: 初期化完了。")

    def detect_and_draw_markers(self, frame):
        """
        指定されたフレームからArUcoマーカーを検出し、ID、枠、姿勢推定に基づく座標軸、
        および情報を描画します。検出されたマーカーの情報も返します。

        Args:
            frame (numpy.ndarray): 処理対象のカメラフレーム。

        Returns:
            tuple: (numpy.ndarray, list)
                - マーカー情報が描画されたフレーム。
                - 検出された各マーカーの情報（ID, rvec_m2c, tvec_m2c, center_uv）を含む辞書のリスト。
                  例: [{'id': marker_id, 'rvec_m2c': rvec, 'tvec_m2c': tvec, 'center_uv': (x,y)}, ...]
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        corners, ids, rejected = self.detector.detectMarkers(gray) 
        
        detected_markers_info = []

        if ids is not None:
            # 検出されたマーカーの姿勢を推定
            # rvecs: 各マーカーの回転ベクトル, tvecs: 各マーカーの並進ベクトル
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers( #
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i, marker_id_array in enumerate(ids):
                marker_id = marker_id_array[0] 
                rvec_m2c = rvecs[i] # マーカーからカメラへの回転ベクトル
                tvec_m2c = tvecs[i] # マーカーからカメラへの並進ベクトル

                # マーカーの枠とIDを描画
                cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[marker_id]])) 

                # 姿勢推定の結果を使って座標軸を描画
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, 
                                  rvec_m2c, tvec_m2c, self.axis_length_on_marker)

                corners_of_marker = corners[i][0] 
                center_x = int(np.mean(corners_of_marker[:, 0])) 
                center_y = int(np.mean(corners_of_marker[:, 1])) 
                center_uv = (center_x, center_y) 

                cv2.circle(frame, center_uv, 5, (0, 0, 255), -1)  # マーカー中心に赤点

                text_id_cam = f"ID:{marker_id} (Cam)" 
                text_pos_cam = f"PosC:({tvec_m2c[0][0]:.2f},{tvec_m2c[0][1]:.2f},{tvec_m2c[0][2]:.2f})" 

                # cv2.putText(frame, text_id_cam, (center_x - 60, center_y - 30), 
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) 
                # cv2.putText(frame, text_pos_cam, (center_x - 60, center_y - 10), 
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1) 

                # 検出情報をリストに追加
                detected_markers_info.append({
                    'id': marker_id,
                    'rvec_m2c': rvec_m2c,
                    'tvec_m2c': tvec_m2c,
                    'center_uv': center_uv
                })
        
        return frame, detected_markers_info