# module/action_planner.py

import math
from enum import Enum, auto
import time

from .robot_controller import RobotController


class ActionPlanner:
    """
    VisionSystemから受け取った情報に基づき、ロボットの行動計画を立て、実行するクラス。
    ステートマシンとして実装し、ロボットの現在の状態に応じて行動を決定する。
    """

    # ロボットの行動状態を定義する列挙型
    class State(Enum):
        SEARCHING_BALL = auto()         # 0. ボールを探している
        MOVING_TO_BALL_Y = auto()       # 1. ボールのY座標（左右）に移動中
        MOVING_TO_BALL_X = auto()       # 2. ボールのX座標（前後）に移動中
        PICKING_UP_BALL = auto()        # 3. ボールを拾い上げ中
        MOVING_TO_GOAL_Y = auto()       # 4. ゴールのY座標（左右）に移動中
        MOVING_TO_GOAL_X = auto()       # 5. ゴールのX座標（前後）に移動中
        DROPPING_BALL = auto()          # 6. ボールをゴールに投下中
        WAITING = auto()                # 7. 待機中（対象が見つからないなど）

    def __init__(self, robot_controller: RobotController):
        """
        ActionPlannerを初期化します。
        """
        # --- 制御用パラメータ・定数 (ここで動作を微調整します) ---

        # ゴール（カゴ）の座標 (mm単位)
        self.GOAL_POSITION = {'x': 240., 'y': 0.0}
        
        # ボールを拾う際の最終停止位置のオフセット (mm単位)
        self.PICKUP_OFFSET_X = 165.0   # 前後方向のオフセット
        self.PICKUP_OFFSET_Y = 0.0    # 左右方向のオフセット
        
        # 座標が合ったとみなす許容誤差 (mm単位)
        self.POSITION_TOLERANCE = 20.0
        
        # Proportional-Control Gain: 残り距離のうち、一度にどれだけの割合を進むかを決める係数 (0.0-1.0)
        self.MOVE_PROPORTIONAL_GAIN = 0.2
        
        # 一度に保持できるボールの最大数
        self.MAX_BALL_CAPACITY = 3
        
        # ターゲットをロックするX軸方向の距離 (mm単位)
        self.TARGET_LOCK_DISTANCE_X = 200.0

        self.TARGET_Y_LIMIT = 350.0  # <--- ターゲットにするボールのY座標の最大絶対値 (mm)

        # --- 内部状態 ---
        self.robot_controller = robot_controller
        self.state = self.State.SEARCHING_BALL
        self.target_ball = None
        self.balls_collected_count = 0
        self.robot_pose = None
        self.balls_info = []

        self.search_X_OFFSET = 20 #ロボットより前方にあるボールを選ぶ際のしきい値
        
        # ターゲットロック機能のための変数
        self.target_locked = False
        self.locked_target_position = None
        self.locked_target_radius_px = 0

        print("ActionPlanner: Initialized for synchronous control.")
        print(f"  - Target Y-axis limit: +/- {self.TARGET_Y_LIMIT}mm")


    def update_world_state(self, robot_pose: dict, balls_info: list):
        """
        VisionSystemから最新の世界情報を更新します。
        このメソッドはmain.pyのループから毎フレーム呼び出されます。
        """
        self.robot_pose = robot_pose
        self.balls_info = balls_info

    def _find_nearest_ball(self):
        """
        ロボットの前方、かつY座標の範囲内にある最も近いボールを見つける。
        """
        if not self.robot_pose or not self.balls_info:
            return None
            
        robot_pos_vec = self.robot_pose['world_position']
        robot_x = robot_pos_vec[0, 0]
        robot_y = robot_pos_vec[1, 0]

        # <--- フィルタリング条件を追加 ---
        candidate_balls = []
        for ball in self.balls_info:
            if ball.get('world_xyz') is not None:
                ball_pos = ball['world_xyz']
                # 条件1: ロボットより前方にあるか
                is_forward = ball_pos[0] > robot_x + self.search_X_OFFSET 
                # 条件2: Y座標が指定範囲内にあるか
                is_within_y_limit = abs(ball_pos[1]) <= self.TARGET_Y_LIMIT
                
                if is_forward and is_within_y_limit:
                    candidate_balls.append(ball)
        
        if not candidate_balls:
            return None

        # 候補の中から最短距離のボールを見つける
        nearest_ball = min(
            candidate_balls, 
            key=lambda b: math.sqrt((robot_x - b['world_xyz'][0])**2 + (robot_y - b['world_xyz'][1])**2)
        )
        
        return nearest_ball

    # <--- 描画のために現在のターゲット情報を返す新しいメソッド ---
    def get_target_info(self):
        """
        main.pyの描画ループに現在のターゲット情報を渡す。
        """
        if self.target_locked and self.locked_target_position is not None:
            # ターゲットがロックされている場合
            return {
                'locked': True,
                'position_3d': self.locked_target_position,
                'radius_px': self.locked_target_radius_px
            }
        elif self.target_ball is not None:
            # ターゲットはあるが、まだロックされていない場合
            return {
                'locked': False,
                'name': self.target_ball.get('name')
            }
        # ターゲットが何もない場合
        return None

    def plan_and_execute(self):
        """
        現在の状態に基づいて行動計画を立て、実行します。
        同期通信のため、このメソッド内のrobot_controllerの各命令は、
        ロボットの動作が完了するまで次の処理に進みません。
        """
        if self.robot_pose is None:
            return

        current_robot_pos_vec = self.robot_pose['world_position']

        # --- ステートマシン ---
        if self.state == self.State.SEARCHING_BALL:
            # 探索開始時にターゲットロックを解除
            self.target_locked = False
            self.locked_target_position = None
            
            # 新しいターゲットボールを探す
            self.target_ball = self._find_nearest_ball()
            
            if self.target_ball:
                # ターゲットが見つかったら、Y軸（左右）の調整状態に移行
                self.state = self.State.MOVING_TO_BALL_Y
            else:
                # 見つからなければ待機状態へ
                self.state = self.State.WAITING
        
        elif self.state == self.State.MOVING_TO_BALL_Y or self.state == self.State.MOVING_TO_BALL_X:
            # --- ボールへの移動ロジック ---

            # ステップ1: 目標座標の決定
            if self.target_locked:
                # ターゲットがロックされている場合、最後に保存した座標を使い続ける
                target_pos = self.locked_target_position
            else:
                # ロックされていなければ、常に最新のボール位置を再探索する
                new_target = self._find_nearest_ball()
                if new_target is None: # 途中で見失ったら待機状態へ
                    self.state = self.State.WAITING
                    return
                self.target_ball = new_target
                target_pos = self.target_ball['world_xyz']

                # ステップ2: ターゲットロックの判定
                robot_x = current_robot_pos_vec[0, 0]
                if abs(robot_x - target_pos[0]) < self.TARGET_LOCK_DISTANCE_X:
                    self.target_locked = True
                    self.locked_target_position = target_pos
                    # <--- ロック時に描画用の半径も保存 ---
                    self.locked_target_radius_px = self.target_ball.get('radius_px', 20) # デフォルト値20
                    print(f">>> TARGET LOCKED on {self.target_ball['name']} <<<")

            # ステップ3: 現在の状態に応じた移動の実行
            if self.state == self.State.MOVING_TO_BALL_Y:
                # Y軸（左右）の調整
                robot_y = current_robot_pos_vec[1, 0]
                target_y = target_pos[1] + self.PICKUP_OFFSET_Y
                delta_y = target_y - robot_y
                
                if abs(delta_y) > self.POSITION_TOLERANCE:
                    distance_to_move_mm = abs(delta_y) * self.MOVE_PROPORTIONAL_GAIN
                    direction = "right" if delta_y > 0 else "left"
                    
                    robot_x = current_robot_pos_vec[0, 0]
                    delta_x_for_display = (target_pos[0] - self.PICKUP_OFFSET_X) - robot_x
                    print(f"  - Target:({target_pos[0]:.1f}, {target_pos[1]:.1f}), Delta:(X:{delta_x_for_display:.1f}, Y:{delta_y:.1f})")

                    self.robot_controller.move(direction, distance_to_move_mm)
                else: # Y座標が合ったのでX座標の調整へ
                    self.state = self.State.MOVING_TO_BALL_X
            
            elif self.state == self.State.MOVING_TO_BALL_X:
                # X軸（前後）の調整
                robot_x = current_robot_pos_vec[0, 0]
                target_x = target_pos[0] - self.PICKUP_OFFSET_X
                delta_x = target_x - robot_x

                if abs(delta_x) > self.POSITION_TOLERANCE:
                    distance_to_move_mm = abs(delta_x) * self.MOVE_PROPORTIONAL_GAIN
                    direction = "up" if delta_x > 0 else "down"

                    robot_y = current_robot_pos_vec[1, 0]
                    delta_y_for_display = (target_pos[1] + self.PICKUP_OFFSET_Y) - robot_y
                    print(f"  - Target:({target_pos[0]:.1f}, {target_pos[1]:.1f}), Delta:(X:{delta_x:.1f}, Y:{delta_y_for_display:.1f})")

                    self.robot_controller.move(direction, distance_to_move_mm)
                else: # X座標も合ったのでピックアップへ
                    self.state = self.State.PICKING_UP_BALL

        elif self.state == self.State.MOVING_TO_GOAL_Y:
            robot_y = current_robot_pos_vec[1, 0]
            delta_y = self.GOAL_POSITION['y'] - robot_y
            if abs(delta_y) > self.POSITION_TOLERANCE:
                self.robot_controller.move("right" if delta_y > 0 else "left", abs(delta_y) * self.MOVE_PROPORTIONAL_GAIN)
            else:
                self.state = self.State.MOVING_TO_GOAL_X
        
        elif self.state == self.State.MOVING_TO_GOAL_X:
            robot_x = current_robot_pos_vec[0, 0]
            delta_x = self.GOAL_POSITION['x'] - robot_x
            if abs(delta_x) > self.POSITION_TOLERANCE:
                self.robot_controller.move("up" if delta_x > 0 else "down", abs(delta_x) * self.MOVE_PROPORTIONAL_GAIN)
            else:
                self.state = self.State.DROPPING_BALL

        elif self.state == self.State.PICKING_UP_BALL:
            print("State: PICKING_UP_BALL")
            # 1. ボールを拾う動作を実行
            self.robot_controller.pick_ball()
            
            # ロボットのx座標が300以上の場合、pick動作の後に1秒間後退するコマンドを追加
            print("  - Action: Moving back for 1 second after pickup.")
            if(robot_x > 300):
                self.robot_controller.move("down", 20.0) 
            
            # 2. ボール収集カウンタを増やす
            self.balls_collected_count += 1
            
            # 3. 拾ったボールをリストから削除
            if self.target_ball:
                self.balls_info = [b for b in self.balls_info if b['name'] != self.target_ball['name']]
            
            # 4. 次の状態に遷移
            if self.balls_collected_count >= self.MAX_BALL_CAPACITY:
                self.state = self.State.MOVING_TO_GOAL_Y
            else:
                self.state = self.State.SEARCHING_BALL

        elif self.state == self.State.DROPPING_BALL:
            print("State: DROPPING_BALL")
            self.robot_controller.drop_ball()
            self.balls_collected_count = 0
            self.state = self.State.SEARCHING_BALL

        elif self.state == self.State.WAITING:
            # 待機中は、再びボールが見つかるのを待つため、探索状態に戻る
            print("State: WAITING... searching again.")
            self.state = self.State.SEARCHING_BALL
            time.sleep(1) # 少し待ってから再探索