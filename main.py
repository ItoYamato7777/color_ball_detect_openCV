import numpy as np
import cv2
import time

from module.camera_manager import CameraManager
from module.world_coordinate_system import WorldCoordinateSystem
from module.color_ball_detector import ColorBallDetector
from module.aruco_detector import ArucoDetector #
from module.ball_world_translator import BallWorldTranslator

from module.aruco_world_translator import ArucoWorldTranslator

# --- 定数定義 (各スクリプトから集約) ---

# カメラ内部パラメータと歪み係数 (キャリブレーション済みとされる値)
MTX_CALIB = np.array([[826.13725388229750,0.00000000000000,283.41487737448006],
                      [0.00000000000000,827.27756538026267,216.17748304394581],
                      [0.00000000000000,0.00000000000000,1.00000000000000]], dtype=np.float64) #

DIST_CALIB = np.array([-0.25266220406939,3.21743761465987,-0.00442676354045,-0.01670514589555,-9.40231611478410]) #

# チェスボード設定
CHESSBOARD_NX = 7  # チェスボードの内側のコーナーの数 (X方向)
CHESSBOARD_NY = 6  # チェスボードの内側のコーナーの数 (Y方向)
CHESSBOARD_SQUARE_SIZE = 25  # チェスボードの正方形の一辺の実際のサイズ (mm)
WORLD_AXIS_LENGTH = CHESSBOARD_SQUARE_SIZE # 描画する座標軸の長さ

# solvePnPに使用するコーナー検出精度向上のための基準
PNP_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #

ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50 # 使用するArUcoマーカーの辞書
ARUCO_MARKER_LENGTH = 0.08  # マーカーの実際のサイズ (メートル単位)
ARUCO_AXIS_LENGTH = 0.05
# 注意: CHESSBOARD_SQUARE_SIZE は mm 単位、ARUCO_MARKER_LENGTH はメートル単位です。
# 世界座標系は mm で統一されていると仮定し、Arucoマーカーの単位もmmに変換するか、
# WorldCoordinateSystem の square_size をメートルにする必要があります。
# ここでは、ArucoDetector と ArucoWorldTranslator に渡すマーカー長は mm 単位であると仮定し、
# 定義を修正します。
ARUCO_MARKER_LENGTH_MM = ARUCO_MARKER_LENGTH * 1000 # mm単位に変換 (80mm)
ARUCO_AXIS_LENGTH_ON_MARKER_MM = ARUCO_AXIS_LENGTH * 1000 # mm単位に変換 (50mm)


COLOR_RANGES_HSV = {
    'red': [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [180, 255, 255])],
    'blue': [([100, 150, 0], [140, 255, 255])],
    'green': [([30, 64, 64], [90, 255, 255])]
} #
COLOR_BGR_DRAW = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0)
} #
MORPH_KERNEL = np.ones((5, 5), np.uint8) #
MIN_CONTOUR_AREA_BALL = 100 #
MIN_BALL_RADIUS = 15 #

BALL_RADIUS_WORLD_MM = 55.0 #


class VisionSystem:
    """
    カメラ管理、世界座標系設定、ArUcoマーカー検出、カラーボール検出、
    およびボールの世界座標変換の機能を統合し、全体の処理フローを管理するクラス。
    """
    def __init__(self, camera_id=0, frame_width=640, frame_height=480):
        """
        VisionSystemを初期化します。各機能モジュールをセットアップします。

        Args:
            camera_id (int): 使用するカメラのID。
            frame_width (int): カメラフレームの希望する幅。
            frame_height (int): カメラフレームの希望する高さ。
        """
        print("VisionSystem: 初期化を開始します...")
        try:
            self.camera_manager = CameraManager(camera_id, frame_width, frame_height) #
        except IOError as e:
            print(f"エラー: VisionSystemの初期化に失敗しました。{e}") #
            raise

        self.mtx_calib = MTX_CALIB #
        self.dist_calib = DIST_CALIB #

        self.world_coordinate_system = WorldCoordinateSystem(
            mtx=self.mtx_calib,
            dist=self.dist_calib,
            nx=CHESSBOARD_NX,
            ny=CHESSBOARD_NY,
            square_size=CHESSBOARD_SQUARE_SIZE, # mm単位
            axis_length=WORLD_AXIS_LENGTH,     # mm単位
            criteria=PNP_CRITERIA
        ) #

        # ArucoDetectorには mm 単位のマーカー長を渡す
        self.aruco_detector = ArucoDetector(
            camera_matrix=self.mtx_calib,
            dist_coeffs=self.dist_calib,
            aruco_dict_type=ARUCO_DICT_TYPE,
            marker_length=ARUCO_MARKER_LENGTH_MM, # mm単位に変更
            axis_length=ARUCO_AXIS_LENGTH_ON_MARKER_MM # mm単位に変更
        ) #

        self.color_ball_detector = ColorBallDetector(
            color_ranges_hsv=COLOR_RANGES_HSV,
            color_bgr_draw=COLOR_BGR_DRAW,
            morph_kernel=MORPH_KERNEL,
            min_contour_area=MIN_CONTOUR_AREA_BALL,
            min_radius=MIN_BALL_RADIUS
        ) #
        
        self.ball_world_translator = BallWorldTranslator(
            mtx=self.mtx_calib,
            dist=self.dist_calib,
            ball_radius_world=BALL_RADIUS_WORLD_MM 
        ) #
        
        # ArucoWorldTranslator のインスタンスを生成
        self.aruco_world_translator = ArucoWorldTranslator()
        
        print("VisionSystem: 全てのモジュールの初期化が完了しました。") #

    def _handle_key_input(self, key, current_frame_for_world_setup): #
        if key == ord('q'): #
            print("'q'キーが押されました。プログラムを終了します。") #
            return True

        if key == ord('1'): #
            print("\n'1'キーが押されました。世界座標系の設定を試みます...") #
            self.world_coordinate_system.establish_world_frame(current_frame_for_world_setup) #

        return False #

    def _process_frame(self, frame):
        """
        単一フレームに対して全ての画像処理を実行します。

        Args:
            frame (numpy.ndarray): 加工対象のフレーム。

        Returns:
            numpy.ndarray: 全ての描画処理が適用されたフレーム。
        """
        processed_frame = frame.copy() # 毎フレームコピーして処理

        # 1. カラーボールを検出・描画し、2D画像上のボール情報を取得
        processed_frame, detected_balls_info_2d = self.color_ball_detector.detect_and_draw_balls(processed_frame) #

        # 2. ArUcoマーカーを検出・描画し、カメラ座標系でのマーカー情報を取得
        processed_frame, detected_markers_info_cam = self.aruco_detector.detect_and_draw_markers(processed_frame) # の呼び出し方を修正

        # 3. 世界座標系の軸を描画
        processed_frame = self.world_coordinate_system.draw_world_axes(processed_frame) #

        # 4. ボールの世界座標を計算・描画
        detected_balls_info_with_world = [] #
        if self.world_coordinate_system.world_frame_established: #
            rvec_w2c = self.world_coordinate_system.rvec_w2c #
            tvec_w2c = self.world_coordinate_system.tvec_w2c #

            if rvec_w2c is not None and tvec_w2c is not None: #
                for ball_2d_info in detected_balls_info_2d: #
                    center_uv = ball_2d_info.get('center_uv') #
                    if center_uv: #
                        world_xyz = self.ball_world_translator.calculate_world_coords( #
                            ball_center_uv=center_uv,
                            rvec_w2c=rvec_w2c,
                            tvec_w2c=tvec_w2c
                        )
                        current_ball_info_with_world = ball_2d_info.copy() #
                        current_ball_info_with_world['world_xyz'] = world_xyz #
                        detected_balls_info_with_world.append(current_ball_info_with_world) #

                        if world_xyz is not None: #
                            print(f"Ball: {ball_2d_info.get('name')}, World Coords (X,Y,Z): ({world_xyz[0]:.1f}, {world_xyz[1]:.1f}, {world_xyz[2]:.1f}) mm") #
                        # else: #
                            # print(f"Ball: {ball_2d_info.get('name')}, World Coords: Calculation failed or not attempted.") #
            else: #
                for ball_2d_info in detected_balls_info_2d: #
                    current_ball_info_with_world = ball_2d_info.copy() #
                    current_ball_info_with_world['world_xyz'] = None #
                    detected_balls_info_with_world.append(current_ball_info_with_world) #
        else: #
            for ball_2d_info in detected_balls_info_2d: #
                current_ball_info_with_world = ball_2d_info.copy() #
                current_ball_info_with_world['world_xyz'] = None #
                detected_balls_info_with_world.append(current_ball_info_with_world) #

        processed_frame = self.ball_world_translator.draw_world_coordinates_on_frame( #
            processed_frame,
            detected_balls_info_with_world
        )
        
        # 5. ArUcoマーカーの世界座標を計算・描画
        detected_markers_info_with_world = []
        if self.world_coordinate_system.world_frame_established:
            rvec_w2c = self.world_coordinate_system.rvec_w2c
            tvec_w2c = self.world_coordinate_system.tvec_w2c

            if rvec_w2c is not None and tvec_w2c is not None:
                for marker_cam_info in detected_markers_info_cam:
                    rvec_m2c = marker_cam_info.get('rvec_m2c')
                    tvec_m2c = marker_cam_info.get('tvec_m2c')
                    
                    if rvec_m2c is not None and tvec_m2c is not None:
                        world_pos, world_rot_mat = self.aruco_world_translator.calculate_marker_world_pose(
                            rvec_m2c=rvec_m2c,
                            tvec_m2c=tvec_m2c,
                            rvec_w2c=rvec_w2c,
                            tvec_w2c=tvec_w2c
                        )
                        current_marker_info_with_world = marker_cam_info.copy()
                        current_marker_info_with_world['world_position'] = world_pos
                        current_marker_info_with_world['world_orientation_matrix'] = world_rot_mat
                        detected_markers_info_with_world.append(current_marker_info_with_world)

                        if world_pos is not None and world_rot_mat is not None:
                            print(f"Marker ID: {marker_cam_info.get('id')}, World Pos (X,Y,Z): ({world_pos[0,0]:.1f}, {world_pos[1,0]:.1f}, {world_pos[2,0]:.1f}) mm")
                            # print(f"Marker ID: {marker_cam_info.get('id')}, World RotMat Z-axis: ({world_rot_mat[0,2]:.2f}, {world_rot_mat[1,2]:.2f}, {world_rot_mat[2,2]:.2f})")
                    else:
                        current_marker_info_with_world = marker_cam_info.copy()
                        current_marker_info_with_world['world_position'] = None
                        current_marker_info_with_world['world_orientation_matrix'] = None
                        detected_markers_info_with_world.append(current_marker_info_with_world)
            else:
                for marker_cam_info in detected_markers_info_cam:
                    current_marker_info_with_world = marker_cam_info.copy()
                    current_marker_info_with_world['world_position'] = None
                    current_marker_info_with_world['world_orientation_matrix'] = None
                    detected_markers_info_with_world.append(current_marker_info_with_world)
        else:
            for marker_cam_info in detected_markers_info_cam:
                current_marker_info_with_world = marker_cam_info.copy()
                current_marker_info_with_world['world_position'] = None
                current_marker_info_with_world['world_orientation_matrix'] = None
                detected_markers_info_with_world.append(current_marker_info_with_world)

        processed_frame = self.aruco_world_translator.draw_world_pose_on_frame(
            processed_frame,
            detected_markers_info_with_world
        )

        return processed_frame #

    def run(self): #
        if not self.camera_manager.is_opened(): #
            print("エラー: カメラが正常に開かれていないため、実行を中止します。") #
            return

        print("\nカメラ映像を表示中...'1'キーで世界座標系を設定。'q'キーで終了。") #
        print(f"世界座標系設定用チェスボード: 内側コーナー {CHESSBOARD_NX}x{CHESSBOARD_NY}, 正方形サイズ {CHESSBOARD_SQUARE_SIZE}mm") #
        print(f"ボール半径 (World): {BALL_RADIUS_WORLD_MM}mm (Z座標はこれのマイナス値になります)") #
        print(f"ArUcoマーカー長 (World): {ARUCO_MARKER_LENGTH_MM}mm")


        main_window_name = 'Vision System Output - Press 1 to Set World, q to Quit' #
        cv2.namedWindow(main_window_name, cv2.WINDOW_AUTOSIZE) #

        while True: #
            ret, frame = self.camera_manager.read_frame() #
            if not ret: #
                print("エラー: フレームの読み込みに失敗しました。1秒待機します。") #
                time.sleep(1) #
                continue

            display_frame = self._process_frame(frame) #

            cv2.imshow(main_window_name, display_frame) #

            key = cv2.waitKey(1) & 0xFF #
            if self._handle_key_input(key, frame): #
                break

        self.cleanup() #

    def cleanup(self): #
        print("VisionSystem: クリーンアップ処理を開始します。") #
        self.camera_manager.release() #
        cv2.destroyAllWindows() #
        print("VisionSystem: カメラを解放し、全てのウィンドウを閉じました。プログラムを終了しました。") #


if __name__ == "__main__": #
    try:
        vision_system = VisionSystem(camera_id=1, frame_width=640, frame_height=480) # から frame_width, frame_height を変更
        vision_system.run() #
    except IOError as e:
        print(f"プログラムの実行中にカメラ関連のエラーが発生しました: {e}") #
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}") #