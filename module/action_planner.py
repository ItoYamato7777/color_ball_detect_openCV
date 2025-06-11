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
        self.PICKUP_OFFSET_X = 25.0                 # <--- ボールを拾うために手前で止まるオフセット(X軸方向)
        self.POSITION_TOLERANCE = 20.0               # 座標合わせの許容誤差(cm)
        # Proportional-Control Gain: 残り距離のうち、一度にどれだけの割合を進むかを決める係数 (0.0-1.0)
        self.MOVE_PROPORTIONAL_GAIN = 0.6
        # move_coefficient 移動距離の比例係数
        self.move_coefficient = 0.1
        self.MAX_BALL_CAPACITY = 3  # ロボットが保持できるボールの最大数
        # Dynamic Interval: 移動距離に応じて次のコマンドまでの待機時間を動的に変更するための係数
        self.MIN_COMMAND_INTERVAL = 0.25  # 待機時間（秒）の最小値
        self.MAX_COMMAND_INTERVAL = 1.5   # 待機時間（秒）の最大値
        self.INTERVAL_TIME_GAIN = 0.05    # 移動距離(cm)を待機時間(秒)に変換する係数 (例: 10cm移動 -> 0.5s待機)

        # --- 内部状態 ---
        self.robot_controller = robot_controller
        self.state = self.State.SEARCHING_BALL
        self.target_ball = None
        self.balls_collected_count = 0
        self.robot_pose = None
        self.balls_info = []
        self.last_command_time = 0
        self.dynamic_interval = self.MAX_COMMAND_INTERVAL  # <--- 次のコマンドまでの動的な待機時間

        print("ActionPlanner: Proportional control enabled.")

    def update_world_state(self, robot_pose: dict, balls_info: list):
        self.robot_pose = robot_pose
        self.balls_info = balls_info

    def _find_nearest_ball(self):
        """最も近いボールを見つけてターゲットに設定する。"""
        if not self.robot_pose or not self.balls_info:
            return None
        robot_pos_vec = self.robot_pose['world_position']
        robot_x = robot_pos_vec[0, 0]
        robot_y = robot_pos_vec[1, 0]
        nearest_ball = None
        min_dist = float('inf')
        for ball in self.balls_info:
            if ball.get('world_xyz') is None:
                continue
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

        # 前回のコマンド送信から、動的に計算された待機時間(dynamic_interval)が経過するまで何もしない
        if current_time - self.last_command_time < self.dynamic_interval:
            return

        # --- ステートマシン ---
        if self.state == self.State.SEARCHING_BALL:
            self.target_ball = self._find_nearest_ball()
            if self.target_ball:
                self.state = self.State.MOVING_TO_BALL_Y
                # 新しいシーケンスを開始するため、待機時間をリセット
                self.dynamic_interval = 0
            else:
                self.state = self.State.WAITING
        
        elif self.state in [self.State.MOVING_TO_BALL_Y, self.State.MOVING_TO_GOAL_Y]:
            is_moving_to_goal = self.state == self.State.MOVING_TO_GOAL_Y
            target_y = self.GOAL_POSITION['y'] if is_moving_to_goal else self.target_ball['world_xyz'][1]
            
            robot_y = current_robot_pos_vec[1, 0]
            delta_y = target_y - robot_y
            
            if abs(delta_y) > self.POSITION_TOLERANCE:
                distance_to_move_mm = abs(delta_y) * self.MOVE_PROPORTIONAL_GAIN
                direction = "right" if delta_y > 0 else "left"
                self.robot_controller.move(direction, distance_to_move_mm * self.move_coefficient)
                
                distance_cm = distance_to_move_mm / 10.0
                calculated_interval = distance_cm * self.INTERVAL_TIME_GAIN
                self.dynamic_interval = max(self.MIN_COMMAND_INTERVAL, min(self.MAX_COMMAND_INTERVAL, calculated_interval))
                self.last_command_time = current_time
            else:
                self.state = self.State.MOVING_TO_GOAL_X if is_moving_to_goal else self.State.MOVING_TO_BALL_X
                self.dynamic_interval = 0 # すぐに次の軸の調整に移る

        elif self.state in [self.State.MOVING_TO_BALL_X, self.State.MOVING_TO_GOAL_X]:
            is_moving_to_goal = self.state == self.State.MOVING_TO_GOAL_X
            target_x_base = self.GOAL_POSITION['x'] if is_moving_to_goal else self.target_ball['world_xyz'][0]
            
            # ボールに向かう時のみオフセットを適用
            offset = self.PICKUP_OFFSET_X if not is_moving_to_goal else 0
            target_x = target_x_base - offset
            
            robot_x = current_robot_pos_vec[0, 0]
            delta_x = target_x - robot_x

            if abs(delta_x) > self.POSITION_TOLERANCE:
                distance_to_move_mm = abs(delta_x) * self.MOVE_PROPORTIONAL_GAIN
                direction = "up" if delta_x > 0 else "down"
                self.robot_controller.move(direction, distance_to_move_mm * self.move_coefficient)
                
                # <--- 次の待機時間を動的に計算 ---
                distance_cm = distance_to_move_mm / 10.0
                calculated_interval = distance_cm * self.INTERVAL_TIME_GAIN
                self.dynamic_interval = max(self.MIN_COMMAND_INTERVAL, min(self.MAX_COMMAND_INTERVAL, calculated_interval))
                self.last_command_time = current_time
            else:
                if is_moving_to_goal:
                    self.state = self.State.DROPPING_BALL
                else:
                    self.state = self.State.PICKING_UP_BALL
                self.dynamic_interval = 0 # すぐに次のアクションに移る

        elif self.state == self.State.PICKING_UP_BALL:
            self.robot_controller.pick_up_ball()
            self.last_command_time = current_time
            self.dynamic_interval = self.MAX_COMMAND_INTERVAL # ピックアップ動作中は最大時間待機
            
            self.balls_collected_count += 1
            self.balls_info = [b for b in self.balls_info if b['name'] != self.target_ball['name']]
            self.target_ball = None

            if self.balls_collected_count >= self.MAX_BALL_CAPACITY:
                self.state = self.State.MOVING_TO_GOAL_Y
            else:
                self.state = self.State.SEARCHING_BALL

        elif self.state == self.State.DROPPING_BALL:
            self.robot_controller.drop_ball()
            self.last_command_time = current_time
            self.dynamic_interval = self.MAX_COMMAND_INTERVAL # ドロップ動作中は最大時間待機
            
            self.balls_collected_count = 0
            self.state = self.State.SEARCHING_BALL

        elif self.state == self.State.WAITING:
            self.dynamic_interval = self.MAX_COMMAND_INTERVAL # 待機中は長く待つ
            pass