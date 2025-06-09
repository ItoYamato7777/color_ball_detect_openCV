# module/action_planner.py

import math
from enum import Enum, auto

from .robot_controller import RobotController


class ActionPlanner:
    """
    VisionSystemから受け取った情報に基づき、ロボットの行動計画を立て、実行するクラス。
    ステートマシンとして実装し、ロボットの現在の状態に応じて行動を決定する。
    """

    # ロボットの行動状態を定義する列挙型
    class State(Enum):
        SEARCHING_BALL = auto()         # 0. ボールを探している
        MOVING_TO_BALL_Y = auto()       # 1. ボールのY座標に移動中
        MOVING_TO_BALL_X = auto()       # 2. ボールのX座標（オフセット考慮）に移動中
        PICKING_UP_BALL = auto()        # 3. ボールを拾い上げ中
        MOVING_TO_GOAL_Y = auto()       # 4. ゴールのY座標に移動中
        MOVING_TO_GOAL_X = auto()       # 5. ゴールのX座標に移動中
        DROPPING_BALL = auto()          # 6. ボールをゴールに投下中
        WAITING = auto()                # 7. 待機中（対象が見つからないなど）

    def __init__(self, robot_controller: RobotController):
        """
        ActionPlannerを初期化します。

        Args:
            robot_controller (RobotController): ロボットの動作を制御するコントローラー。
        """
        # --- 制御用パラメータ・定数 ---
        self.GOAL_POSITION = {'x': 0.0, 'y': 180.0}  # ゴールの座標(cm単位) ※要調整
        self.PICKUP_OFFSET_Y = 10.0                 # ボールを拾うために手前で止まるオフセット(cm)
        self.POSITION_TOLERANCE = 2.0               # 座標合わせの許容誤差(cm)
        self.MAX_BALL_CAPACITY = 3                  # 一度に持てるボールの最大数

        # --- 内部状態 ---
        self.robot_controller = robot_controller
        self.state = self.State.SEARCHING_BALL
        self.target_ball = None
        self.balls_collected_count = 0
        self.robot_pose = None
        self.balls_info = []

        print("ActionPlanner: 初期化完了。")
        print(f"  - Goal Position: {self.GOAL_POSITION}")
        print(f"  - Max Ball Capacity: {self.MAX_BALL_CAPACITY}")

    def update_world_state(self, robot_pose: dict, balls_info: list):
        """
        VisionSystemから最新の世界情報を更新します。

        Args:
            robot_pose (dict): ロボットの姿勢情報 {'world_position': (x,y,z)}
            balls_info (list): 検出されたボール情報のリスト
        """
        self.robot_pose = robot_pose
        self.balls_info = balls_info

    def _find_nearest_ball(self):
        """最も近いボールを見つけてターゲットに設定する。"""
        if not self.robot_pose or not self.balls_info:
            return None

        robot_pos = self.robot_pose['world_position']
        nearest_ball = None
        min_dist = float('inf')

        for ball in self.balls_info:
            if ball.get('world_xyz') is None:
                continue
            
            ball_pos = ball['world_xyz']
            dist = math.sqrt((robot_pos[0] - ball_pos[0])**2 + (robot_pos[1] - ball_pos[1])**2)
            
            if dist < min_dist:
                min_dist = dist
                nearest_ball = ball
        
        return nearest_ball

    def plan_and_execute(self):
        """
        現在の状態に基づいて行動計画を立て、実行します。
        このメソッドが毎フレーム呼び出されることを想定しています。
        """
        if self.robot_pose is None:
            # print("Planner: Robot not found. Waiting...")
            return

        current_robot_pos = self.robot_pose['world_position']

        # --- ステートマシン ---
        if self.state == self.State.SEARCHING_BALL:
            print("State: SEARCHING_BALL")
            self.target_ball = self._find_nearest_ball()
            if self.target_ball:
                print(f"  -> Found target ball: {self.target_ball['name']} at {self.target_ball['world_xyz']}")
                self.state = self.State.MOVING_TO_BALL_Y
            else:
                print("  -> No balls found. Waiting.")
                self.state = self.State.WAITING

        elif self.state == self.State.MOVING_TO_BALL_Y:
            print("State: MOVING_TO_BALL_Y")
            target_y = self.target_ball['world_xyz'][1] - self.PICKUP_OFFSET_Y # オフセット分だけ手前を目標に
            delta_y = target_y - current_robot_pos[1]
            
            if abs(delta_y) > self.POSITION_TOLERANCE:
                direction = "up" if delta_y > 0 else "down"
                self.robot_controller.move(direction, abs(delta_y))
            else:
                print("  -> Y coordinate aligned.")
                self.state = self.State.MOVING_TO_BALL_X

        elif self.state == self.State.MOVING_TO_BALL_X:
            print("State: MOVING_TO_BALL_X")
            target_x = self.target_ball['world_xyz'][0]
            delta_x = target_x - current_robot_pos[0]

            if abs(delta_x) > self.POSITION_TOLERANCE:
                direction = "right" if delta_x > 0 else "left"
                self.robot_controller.move(direction, abs(delta_x))
            else:
                print("  -> X coordinate aligned.")
                self.state = self.State.PICKING_UP_BALL

        elif self.state == self.State.PICKING_UP_BALL:
            print("State: PICKING_UP_BALL")
            self.robot_controller.pick_up_ball()
            self.balls_collected_count += 1
            print(f"  -> Ball collected. Total: {self.balls_collected_count}/{self.MAX_BALL_CAPACITY}")

            # ターゲットにしたボールをリストから削除（再ターゲットしないように）
            self.balls_info.remove(self.target_ball)
            self.target_ball = None

            if self.balls_collected_count >= self.MAX_BALL_CAPACITY:
                self.state = self.State.MOVING_TO_GOAL_Y
            else:
                self.state = self.State.SEARCHING_BALL

        elif self.state == self.State.MOVING_TO_GOAL_Y:
            print("State: MOVING_TO_GOAL_Y")
            delta_y = self.GOAL_POSITION['y'] - current_robot_pos[1]
            if abs(delta_y) > self.POSITION_TOLERANCE:
                direction = "up" if delta_y > 0 else "down"
                self.robot_controller.move(direction, abs(delta_y))
            else:
                print("  -> Goal Y coordinate aligned.")
                self.state = self.State.MOVING_TO_GOAL_X

        elif self.state == self.State.MOVING_TO_GOAL_X:
            print("State: MOVING_TO_GOAL_X")
            delta_x = self.GOAL_POSITION['x'] - current_robot_pos[0]
            if abs(delta_x) > self.POSITION_TOLERANCE:
                direction = "right" if delta_x > 0 else "left"
                self.robot_controller.move(direction, abs(delta_x))
            else:
                print("  -> Goal X coordinate aligned.")
                self.state = self.State.DROPPING_BALL
        
        elif self.state == self.State.DROPPING_BALL:
            print("State: DROPPING_BALL")
            self.robot_controller.drop_ball()
            self.balls_collected_count = 0
            print("  -> All balls dropped. Restarting sequence.")
            self.state = self.State.SEARCHING_BALL

        elif self.state == self.State.WAITING:
            # 何もしない
            pass