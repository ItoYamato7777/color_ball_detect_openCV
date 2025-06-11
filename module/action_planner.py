# module/action_planner.py

import math
from enum import Enum, auto
import time

# robot_controllerは別途実装されているという前提
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
        # --- 制御用パラメータ・定数 ---
        self.GOAL_POSITION = {'x': 20.0, 'y': 0.0}   # ゴールの座標(cm単位) ※要調整
        self.PICKUP_OFFSET_X = 85.0                 # <--- ボールを拾うために手前で止まるオフセット(X軸方向)
        self.PICKUP_OFFSET_Y = 0.0
        self.POSITION_TOLERANCE = 20.0               # 座標合わせの許容誤差(cm)
        # Proportional-Control Gain: 残り距離のうち、一度にどれだけの割合を進むかを決める係数 (0.0-1.0)
        self.MOVE_PROPORTIONAL_GAIN = 0.6
        # move_coefficient 移動距離の比例係数
        self.move_coefficient = 0.25
        self.MAX_BALL_CAPACITY = 3  # ロボットが保持できるボールの最大数
        # Dynamic Interval: 移動距離に応じて次のコマンドまでの待機時間を動的に変更するための係数
        self.MIN_COMMAND_INTERVAL = 0.25  # 待機時間（秒）の最小値
        self.MAX_COMMAND_INTERVAL = 1.5   # 待機時間（秒）の最大値
        self.INTERVAL_TIME_GAIN = 0.05    # 移動距離(cm)を待機時間(秒)に変換する係数 (例: 10cm移動 -> 0.5s待機)
        
        self.TARGET_LOCK_DISTANCE_X = 100.0  # <--- ターゲットをロックするX軸距離(mm)

        # --- 内部状態 ---
        self.robot_controller = robot_controller
        self.state = self.State.SEARCHING_BALL
        self.target_ball = None
        self.balls_collected_count = 0
        self.robot_pose = None
        self.balls_info = []
        self.last_command_time = 0
        self.dynamic_interval = self.MAX_COMMAND_INTERVAL
        
        self.target_locked = False           # <--- ターゲットロック状態フラグ
        self.locked_target_position = None   # <--- ロックしたターゲットの座標

        print("ActionPlanner: Advanced targeting enabled.")
        print(f"  - Target Lock Distance: {self.TARGET_LOCK_DISTANCE_X}mm")


    def update_world_state(self, robot_pose: dict, balls_info: list):
        self.robot_pose = robot_pose
        self.balls_info = balls_info

    def _find_nearest_ball(self):
        """
        ロボットの前方にある最も近いボールを見つけてターゲットに設定する。
        """
        if not self.robot_pose or not self.balls_info:
            return None
            
        robot_pos_vec = self.robot_pose['world_position']
        robot_x = robot_pos_vec[0, 0]
        robot_y = robot_pos_vec[1, 0]

        # <--- ロボットより前方にあるボールのみを候補とする ---
        forward_balls = []
        for ball in self.balls_info:
            if ball.get('world_xyz') is not None:
                ball_x = ball['world_xyz'][0]
                if ball_x > robot_x:
                    forward_balls.append(ball)
        
        if not forward_balls:
            return None

        nearest_ball = None
        min_dist = float('inf')
        for ball in forward_balls:
            ball_pos = ball['world_xyz']
            dist = math.sqrt((robot_x - ball_pos[0])**2 + (robot_y - ball_pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_ball = ball
        
        return nearest_ball

    def plan_and_execute(self):
        """
        現在の状態に基づいて行動計画を立て、実行します。
        """
        if self.robot_pose is None:
            return

        current_robot_pos_vec = self.robot_pose['world_position']
        current_time = time.time()

        if current_time - self.last_command_time < self.dynamic_interval:
            return

        # --- ステートマシン ---
        if self.state == self.State.SEARCHING_BALL:
            self.target_locked = False           # <--- 探索開始時にロックを解除
            self.locked_target_position = None   # <--- 座標をリセット
            self.target_ball = self._find_nearest_ball()
            if self.target_ball:
                self.state = self.State.MOVING_TO_BALL_Y
                self.dynamic_interval = 0
            else:
                self.state = self.State.WAITING
        
        elif self.state == self.State.MOVING_TO_BALL_Y or self.state == self.State.MOVING_TO_BALL_X:
            # <--- 修正: ボールへの移動ロジックを統合・更新 ---
            
            # ステップ1: ターゲットの決定（ロックされていなければ再探索）
            if self.target_locked:
                target_pos = self.locked_target_position
            else:
                new_target = self._find_nearest_ball()
                if new_target is None: # 前方にボールがなくなった場合
                    self.state = self.State.WAITING
                    return
                self.target_ball = new_target
                target_pos = self.target_ball['world_xyz']

                # ステップ2: ターゲットロックの判定
                robot_x = current_robot_pos_vec[0, 0]
                if abs(robot_x - target_pos[0]) < self.TARGET_LOCK_DISTANCE_X:
                    self.target_locked = True
                    self.locked_target_position = target_pos
                    print(f">>> TARGET LOCKED on {self.target_ball['name']} at ({target_pos[0]:.1f}, {target_pos[1]:.1f}) <<<")

            # ステップ3: 現在の状態で移動を実行
            if self.state == self.State.MOVING_TO_BALL_Y:
                target_y = target_pos[1] + self.PICKUP_OFFSET_Y
                robot_y = current_robot_pos_vec[1, 0]
                delta_y = target_y - robot_y
                if abs(delta_y) > self.POSITION_TOLERANCE:
                    distance_to_move_mm = abs(delta_y) * self.MOVE_PROPORTIONAL_GAIN
                    direction = "right" if delta_y > 0 else "left"
                    self.robot_controller.move(direction, distance_to_move_mm)
                    
                    distance_cm = distance_to_move_mm / 10.0
                    calculated_interval = distance_cm * self.INTERVAL_TIME_GAIN
                    self.dynamic_interval = max(self.MIN_COMMAND_INTERVAL, min(self.MAX_COMMAND_INTERVAL, calculated_interval))
                    self.last_command_time = current_time
                else: # Y座標が合ったのでX座標の調整へ
                    self.state = self.State.MOVING_TO_BALL_X
                    self.dynamic_interval = 0 
            
            elif self.state == self.State.MOVING_TO_BALL_X:
                target_x = target_pos[0] - self.PICKUP_OFFSET_X
                robot_x = current_robot_pos_vec[0, 0]
                delta_x = target_x - robot_x
                if abs(delta_x) > self.POSITION_TOLERANCE:
                    distance_to_move_mm = abs(delta_x) * self.MOVE_PROPORTIONAL_GAIN
                    direction = "up" if delta_x > 0 else "down"
                    self.robot_controller.move(direction, distance_to_move_mm)

                    distance_cm = distance_to_move_mm / 10.0
                    calculated_interval = distance_cm * self.INTERVAL_TIME_GAIN
                    self.dynamic_interval = max(self.MIN_COMMAND_INTERVAL, min(self.MAX_COMMAND_INTERVAL, calculated_interval))
                    self.last_command_time = current_time
                else: # X座標も合ったのでピックアップへ
                    self.state = self.State.PICKING_UP_BALL
                    self.dynamic_interval = 0

        # (MOVING_TO_GOAL と PICKING_UP/DROPPING は変更なしのため省略)
        elif self.state == self.State.MOVING_TO_GOAL_Y:
            target_y = self.GOAL_POSITION['y']
            robot_y = current_robot_pos_vec[1, 0]
            delta_y = target_y - robot_y
            if abs(delta_y) > self.POSITION_TOLERANCE:
                distance_to_move_mm = abs(delta_y) * self.MOVE_PROPORTIONAL_GAIN
                direction = "right" if delta_y > 0 else "left"
                self.robot_controller.move(direction, distance_to_move_mm)

                distance_cm = distance_to_move_mm / 10.0
                calculated_interval = distance_cm * self.INTERVAL_TIME_GAIN
                self.dynamic_interval = max(self.MIN_COMMAND_INTERVAL, min(self.MAX_COMMAND_INTERVAL, calculated_interval))
                self.last_command_time = current_time
            else:
                self.state = self.State.MOVING_TO_GOAL_X
                self.dynamic_interval = 0
        
        elif self.state == self.State.MOVING_TO_GOAL_X:
            target_x = self.GOAL_POSITION['x']
            robot_x = current_robot_pos_vec[0, 0]
            delta_x = target_x - robot_x
            if abs(delta_x) > self.POSITION_TOLERANCE:
                distance_to_move_mm = abs(delta_x) * self.MOVE_PROPORTIONAL_GAIN
                direction = "up" if delta_x > 0 else "down"
                self.robot_controller.move(direction, distance_to_move_mm)
                
                distance_cm = distance_to_move_mm / 10.0
                calculated_interval = distance_cm * self.INTERVAL_TIME_GAIN
                self.dynamic_interval = max(self.MIN_COMMAND_INTERVAL, min(self.MAX_COMMAND_INTERVAL, calculated_interval))
                self.last_command_time = current_time
            else:
                self.state = self.State.DROPPING_BALL
                self.dynamic_interval = 0

        elif self.state == self.State.PICKING_UP_BALL:
            self.robot_controller.pick_up_ball()
            self.last_command_time = current_time
            self.dynamic_interval = self.MAX_COMMAND_INTERVAL
            
            self.balls_collected_count += 1
            if self.target_ball: # target_ballがNoneでないことを確認
                self.balls_info = [b for b in self.balls_info if b['name'] != self.target_ball['name']]
            self.target_ball = None

            if self.balls_collected_count >= self.MAX_BALL_CAPACITY:
                self.state = self.State.MOVING_TO_GOAL_Y
            else:
                self.state = self.State.SEARCHING_BALL

        elif self.state == self.State.DROPPING_BALL:
            self.robot_controller.drop_ball()
            self.last_command_time = current_time
            self.dynamic_interval = self.MAX_COMMAND_INTERVAL
            
            self.balls_collected_count = 0
            self.state = self.State.SEARCHING_BALL

        elif self.state == self.State.WAITING:
            self.dynamic_interval = self.MAX_COMMAND_INTERVAL
            pass