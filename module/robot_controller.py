# module/robot_controller.py

import socket

class RobotController:
    """
    ロボットの物理的な動作を制御するためのクラス。
    ActionPlannerからの指示をUDP通信でマイコンに送信します。
    """
    def __init__(self, ip_address="192.168.3.109", port=12345):
        """
        RobotControllerを初期化し、UDPソケットを準備します。
        
        Args:
            ip_address (str): ロボットのマイコンのIPアドレス。
            port (int): UDP通信で使用するポート番号。
        """
        self.udp_ip = ip_address
        self.udp_port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"[RobotController] Initialized. Sending to {self.udp_ip}:{self.udp_port}")

    def _send_command(self, command: str, value: float):
        """
        コマンドと値をフォーマットしてマイコンに送信するプライベートメソッド。
        
        Args:
            command (str): マイコンに送るコマンド文字列。
            value (float): コマンドに付随する数値。
        """
        message = f"{command},{value:.2f}"
        self.sock.sendto(message.encode('utf-8'), (self.udp_ip, self.udp_port))
        print(f"[RobotController] Sent: \"{message}\"")

    def move(self, direction: str, distance_mm: float):
        """
        ロボットを指定された方向に指定された距離だけ移動させます。
        距離(mm)を距離(cm)に変換し、その数値をマイコンに送信します。
        マイコン側で、受信した数値 * 100ミリ秒 の時間、モーターを駆動します。

        Args:
            direction (str): 移動方向 ("up", "down", "right", "left")
            distance_mm (float): 移動距離 (mm)
        """
        # ActionPlannerはmm単位で計算するため、cm単位に変換
        distance_cm = distance_mm / 10.0
        # distance_cmの値をそのまま送信する
        self._send_command(direction, distance_cm)

    def pick_up_ball(self):
        """
        ボールを拾い上げる動作をマイコンに指示します。
        """
        # 動作コマンドには数値を必要としないため、0を送信
        self._send_command("pick_up", 0)

    def drop_ball(self):
        """
        ボールを落とす動作をマイコンに指示します。
        """
        # 動作コマンドには数値を必要としないため、0を送信
        self._send_command("drop", 0)