# module/robot_controller.py

import time

class RobotController:
    """
    ロボットの物理的な動作を制御するためのダミークラス。
    ActionPlannerからの指示を受け取り、実行すべきアクションをコンソールに表示します。
    
    実際のロボット制御APIに置き換えることを想定しています。
    """
    def __init__(self):
        """
        RobotControllerを初期化します。
        """
        print("[RobotController] Initialized. (Dummy Implementation)")

    def move(self, direction: str, distance_cm: float):
        """
        ロボットを指定された方向に指定された距離だけ移動させます。
        このダミー実装では、アクションをコンソールに出力するだけです。

        Args:
            direction (str): 移動方向 ("up", "down", "right", "left")
            distance_cm (float): 移動距離 (cm)
        """
        # TODO: ここに実際のロボットを動かすコードを実装します。
        #       (例: シリアル通信でモータードライバに指示を送るなど)
        print(f"[RobotController] ACTION: Move {direction} for {distance_cm:.2f} cm")
        
        # 動作に時間がかかることをシミュレートしたい場合は、以下のコメントを解除
        # time.sleep(0.5) 

    def pick_up_ball(self):
        """
        目の前のボールを拾い上げる動作を実行します。
        このダミー実装では、アクションをコンソールに出力するだけです。
        """
        # TODO: ここにアームなどを動かしてボールを拾うコードを実装します。
        print("[RobotController] ACTION: Picking up ball")
        
        # 動作に時間がかかることをシミュレートしたい場合は、以下のコメントを解除
        # time.sleep(1.0)

    def drop_ball(self):
        """
        保持しているボールをゴール（カゴ）に落とす動作を実行します。
        このダミー実装では、アクションをコンソールに出力するだけです。
        """
        # TODO: ここにカゴを傾けるなどしてボールを落とすコードを実装します。
        print("[RobotController] ACTION: Dropping all balls")
        
        # 動作に時間がかかることをシミュレートしたい場合は、以下のコメントを解除
        # time.sleep(1.0)