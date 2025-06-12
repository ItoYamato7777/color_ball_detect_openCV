# main.py

import numpy as np
import cv2
import time
import threading # <--- 追加: マルチスレッド機能のインポート
import copy      # <--- 追加: 安全なデータコピーのため

from module.camera_manager import CameraManager
from module.world_coordinate_system import WorldCoordinateSystem
from module.color_ball_detector import ColorBallDetector
from module.aruco_detector import ArucoDetector 
from module.ball_world_translator import BallWorldTranslator
from module.aruco_world_translator import ArucoWorldTranslator
from module.robot_controller import RobotController
from module.action_planner import ActionPlanner

# --- 定数定義 (変更なし) ---
MTX_CALIB = np.array([[826.13725388229750,0.00000000000000,283.41487737448006],
                      [0.00000000000000,827.27756538026267,216.17748304394581],
                      [0.00000000000000,0.00000000000000,1.00000000000000]], dtype=np.float64) 
DIST_CALIB = np.array([-0.25266220406939,3.21743761465987,-0.00442676354045,-0.01670514589555,-9.40231611478410])
CHESSBOARD_NX = 7
CHESSBOARD_NY = 6
CHESSBOARD_SQUARE_SIZE = 25
WORLD_AXIS_LENGTH = CHESSBOARD_SQUARE_SIZE
PNP_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
ARUCO_MARKER_LENGTH_MM = 80.0
ARUCO_AXIS_LENGTH_ON_MARKER_MM = 50.0
COLOR_RANGES_HSV = {
    'red': [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [180, 255, 255])],
    'blue': [([100, 150, 0], [140, 255, 255])],
    'green': [([30, 64, 64], [90, 255, 255])]
}
COLOR_BGR_DRAW = {'red': (0, 0, 255), 'blue': (255, 0, 0), 'green': (0, 255, 0)}
MORPH_KERNEL = np.ones((5, 5), np.uint8)
MIN_CONTOUR_AREA_BALL = 100
MIN_BALL_RADIUS = 15
BALL_RADIUS_WORLD_MM = 55.0


# --- 追加: ロボット制御を専門に行う新しいスレッドクラス ---
class RobotControlThread(threading.Thread):
    """
    ActionPlannerの実行をバックグラウンドで処理するためのスレッド。
    これにより、メインの画像処理ループがブロックされるのを防ぐ。
    """
    def __init__(self, action_planner, shared_state, lock):
        super().__init__()
        self.action_planner = action_planner
        self.shared_state = shared_state
        self.lock = lock
        self._stop_event = threading.Event()

    def stop(self):
        """スレッドを安全に停止させるためのイベントをセットする。"""
        self._stop_event.set()

    def run(self):
        """スレッドのメイン処理。停止イベントがセットされるまでループする。"""
        print("[RobotControlThread] Started.")
        while not self._stop_event.is_set():
            # 共有メモリから最新の情報を安全に取得
            with self.lock:
                robot_pose = copy.deepcopy(self.shared_state.get('robot_pose'))
                balls_info = copy.deepcopy(self.shared_state.get('balls_info'))
            
            # 取得した情報でプランナーの状態を更新
            self.action_planner.update_world_state(robot_pose, balls_info)
            # 行動計画を実行（この呼び出しは同期的で、ロボットの動作完了までここで待機する）
            self.action_planner.plan_and_execute()
            
            # CPUを過剰に消費しないように、短いスリープを入れる
            time.sleep(0.1) 
        print("[RobotControlThread] Stopped.")


class VisionSystem:
    """
    カメラ管理、画像処理、そしてロボット制御スレッドの起動と管理を行うメインクラス。
    """
    def __init__(self, camera_id=1, frame_width=640, frame_height=480):
        print("VisionSystem: 初期化を開始します...")
        try:
            self.camera_manager = CameraManager(camera_id, frame_width, frame_height)
        except IOError as e:
            print(f"エラー: VisionSystemの初期化に失敗しました。{e}")
            raise

        self.mtx_calib = MTX_CALIB
        self.dist_calib = DIST_CALIB

        # (画像処理モジュールの初期化は変更なし)
        self.world_coordinate_system = WorldCoordinateSystem(
            mtx=self.mtx_calib, dist=self.dist_calib, nx=CHESSBOARD_NX, ny=CHESSBOARD_NY,
            square_size=CHESSBOARD_SQUARE_SIZE, axis_length=WORLD_AXIS_LENGTH, criteria=PNP_CRITERIA
        )
        self.aruco_detector = ArucoDetector(
            camera_matrix=self.mtx_calib, dist_coeffs=self.dist_calib,
            aruco_dict_type=ARUCO_DICT_TYPE, marker_length=ARUCO_MARKER_LENGTH_MM,
            axis_length=ARUCO_AXIS_LENGTH_ON_MARKER_MM
        )
        self.color_ball_detector = ColorBallDetector(
            color_ranges_hsv=COLOR_RANGES_HSV, color_bgr_draw=COLOR_BGR_DRAW,
            morph_kernel=MORPH_KERNEL, min_contour_area=MIN_CONTOUR_AREA_BALL, min_radius=MIN_BALL_RADIUS
        )
        self.ball_world_translator = BallWorldTranslator(
            mtx=self.mtx_calib, dist=self.dist_calib, ball_radius_world=BALL_RADIUS_WORLD_MM
        )
        self.aruco_world_translator = ArucoWorldTranslator()
        
        # --- 修正: スレッド関連の初期化処理を追加 ---
        # 1. スレッド間で共有するデータのための変数を準備
        self.lock = threading.Lock()
        self.shared_state = {
            'robot_pose': None,
            'balls_info': []
        }
        
        # 2. ロボット制御と行動計画モジュールを初期化
        self.robot_controller = RobotController()
        self.action_planner = ActionPlanner(self.robot_controller)
        
        # 3. ロボット制御スレッドを初期化
        self.robot_control_thread = RobotControlThread(
            self.action_planner, self.shared_state, self.lock
        )
        
        print("VisionSystem: 全てのモジュールの初期化が完了しました。")

    def _handle_key_input(self, key, current_frame_for_world_setup):
        # (このメソッドは変更なし)
        if key == ord('q'):
            print("'q'キーが押されました。プログラムを終了します。")
            return True
        if key == ord('1'):
            print("\n'1'キーが押されました。世界座標系の設定を試みます...")
            self.world_coordinate_system.establish_world_frame(current_frame_for_world_setup)
        return False

    def _process_frame(self, frame):
        # (このメソッドは変更なし)
        processed_frame = frame.copy()
        processed_frame, detected_balls_info_2d = self.color_ball_detector.detect_and_draw_balls(processed_frame)
        processed_frame, detected_markers_info_cam = self.aruco_detector.detect_and_draw_markers(processed_frame)
        processed_frame = self.world_coordinate_system.draw_world_axes(processed_frame)

        detected_balls_info_with_world = []
        if self.world_coordinate_system.world_frame_established:
            rvec_w2c, tvec_w2c = self.world_coordinate_system.rvec_w2c, self.world_coordinate_system.tvec_w2c
            for ball_2d_info in detected_balls_info_2d:
                current_ball_info = ball_2d_info.copy()
                current_ball_info['world_xyz'] = self.ball_world_translator.calculate_world_coords(
                    ball_2d_info.get('center_uv'), rvec_w2c, tvec_w2c)
                detected_balls_info_with_world.append(current_ball_info)
        processed_frame = self.ball_world_translator.draw_world_coordinates_on_frame(
            processed_frame, detected_balls_info_with_world)
        
        detected_markers_info_with_world = []
        if self.world_coordinate_system.world_frame_established:
            rvec_w2c, tvec_w2c = self.world_coordinate_system.rvec_w2c, self.world_coordinate_system.tvec_w2c
            for marker_cam_info in detected_markers_info_cam:
                current_marker_info = marker_cam_info.copy()
                world_pos, world_rot_mat = self.aruco_world_translator.calculate_marker_world_pose(
                    marker_cam_info.get('rvec_m2c'), marker_cam_info.get('tvec_m2c'), rvec_w2c, tvec_w2c)
                current_marker_info['world_position'] = world_pos
                current_marker_info['world_orientation_matrix'] = world_rot_mat
                detected_markers_info_with_world.append(current_marker_info)
        processed_frame = self.aruco_world_translator.draw_world_pose_on_frame(
            processed_frame, detected_markers_info_with_world)
        
        return processed_frame, detected_markers_info_with_world, detected_balls_info_with_world

    def run(self):
        # --- 修正: メインループをマルチスレッド対応に変更 ---
        if not self.camera_manager.is_opened():
            print("エラー: カメラが正常に開かれていないため、実行を中止します。")
            return

        # 制御スレッドを開始
        self.robot_control_thread.start()

        print("\nカメラ映像を表示中...'1'キーで世界座標系を設定。'q'キーで終了。")
        main_window_name = 'Vision System Output - Press 1 to Set World, q to Quit'
        cv2.namedWindow(main_window_name, cv2.WINDOW_AUTOSIZE)

        while True:
            # メインスレッドはカメラからの画像取得と処理に専念
            ret, frame = self.camera_manager.read_frame()
            if not ret:
                print("エラー: フレームの読み込みに失敗しました。1秒待機します。")
                time.sleep(1)
                continue
            
            # 画像処理を実行
            display_frame, markers_info, balls_info = self._process_frame(frame)
            
            # 処理結果を「共有メモ」にロックをかけて書き込む
            with self.lock:
                self.shared_state['robot_pose'] = markers_info[0] if markers_info else None
                self.shared_state['balls_info'] = balls_info

            # 画面を表示
            cv2.imshow(main_window_name, display_frame)

            # キー入力を処理
            key = cv2.waitKey(1) & 0xFF
            if self._handle_key_input(key, frame):
                break # 'q'が押されたらループを抜ける

        # ループが終了したらクリーンアップ処理
        self.cleanup()

    def cleanup(self):
        # --- 修正: スレッドの安全な停止処理を追加 ---
        print("VisionSystem: クリーンアップ処理を開始します。")

        # 1. 制御スレッドに停止信号を送る
        print("[Cleanup] Stopping robot control thread...")
        self.robot_control_thread.stop()
        # 2. 制御スレッドが完全に終了するのを待つ
        self.robot_control_thread.join()
        print("[Cleanup] Robot control thread stopped.")

        # 3. ロボットコントローラーの接続を閉じる
        self.robot_controller.close()
        
        # 4. カメラを解放し、ウィンドウを閉じる
        self.camera_manager.release()
        cv2.destroyAllWindows()
        print("VisionSystem: プログラムを正常に終了しました。")


if __name__ == "__main__":
    try:
        vision_system = VisionSystem(camera_id=1, frame_width=640, frame_height=480)
        vision_system.run()
    except IOError as e:
        print(f"カメラ関連のエラー: {e}")
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")