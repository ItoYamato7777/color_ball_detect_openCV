# module/robot_controller.py

import socket
import math
import time

UDP_IP = "192.168.."  #GR-ROSEのIPアドレス
UDP_PORT = 12345 #ポート番号 GR-ROSE側と揃える

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("サーボ番号（1〜3）と角度（度）を指定してください（例：1 30）")
print("終了するには Ctrl+C または 空入力 + Enter")

#コマンドライン入力待ち機能
while True:
    try:
        cmd = input("入力 > ").strip()
        if not cmd:
            continue

        parts = cmd.split()
        if len(parts) != 2:
            print("形式が正しくありません．例：2 -45")
            continue

        servo_id = int(parts[0])
        angle_deg = float(parts[1])

        if not 1 <= servo_id <= 3:
            print("サーボ番号は1〜3で指定してください")
            continue

        # 度→ラジアン変換（GR-ROSE側がラジアンで受信）
        angle_rad = math.radians(angle_deg)

        # メッセージ形式： "1,0.5236" （ID=1, 角度0.5236[rad]）
        message = f"{servo_id},{angle_rad:.4f}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT)) #送信
        print(f"送信しました: サーボ{servo_id} → {angle_deg:.1f}度（{angle_rad:.4f} rad）")

    except ValueError:
        print("数値の形式が正しくありません")
    except KeyboardInterrupt:
        print("\n終了します")
        break





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