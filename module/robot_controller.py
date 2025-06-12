# module/robot_controller.py

import socket

class RobotController:
    """
    ロボットの物理的な動作を制御するためのクラス。
    TCP通信を使い、マイコンにコマンドを送信し、動作完了報告を待つ。
    """
    def __init__(self, ip_address="192.168.3.73", port=12345, timeout=15.0):
        """
        RobotControllerを初期化し、マイコンとのTCP接続を確立します。
        """
        self.tcp_ip = ip_address
        self.tcp_port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout) # タイムアウトを設定

        try:
            print(f"[RobotController] Connecting to {self.tcp_ip}:{self.tcp_port}...")
            self.sock.connect((self.tcp_ip, self.tcp_port))
            print("[RobotController] Connected successfully.")
        except socket.error as e:
            print(f"[RobotController] Connection failed: {e}")
            raise

    def execute_and_wait(self, command: str, value: float):
        """
        コマンドを送信し、マイコンからの完了報告('done')を待つ。
        """
        try:
            message = f"{command},{value:.2f}\n" # 改行コードを追加
            print(f"[RobotController] Sending command: \"{message.strip()}\"")
            self.sock.sendall(message.encode('utf-8'))

            # マイコンからの応答を待つ (最大1024バイト)
            response = self.sock.recv(1024).decode('utf-8').strip()
            if response == "done":
                print(f"[RobotController] Received 'done' for command: {command}")
            else:
                print(f"[RobotController] WARN: Received unexpected response: {response}")

        except socket.timeout:
            print("[RobotController] ERROR: Connection timed out. Robot not responding.")
            raise
        except socket.error as e:
            print(f"[RobotController] ERROR: Connection error: {e}")
            raise

    def move(self, direction: str, distance_mm: float):
        distance_cm = distance_mm / 10.0
        self.execute_and_wait(direction, distance_cm)

    def pick_up_ball(self):
        self.execute_and_wait("pick_up", 0)

    def drop_ball(self):
        self.execute_and_wait("drop", 0)

    def close(self):
        """
        ソケット接続を閉じる
        """
        print("[RobotController] Closing connection.")
        self.sock.close()