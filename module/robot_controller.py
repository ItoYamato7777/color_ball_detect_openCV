# module/robot_controller.py

import socket

class RobotController:
    """
    ロボットの物理的な動作を制御するためのクラス。
    TCP接続を維持し、同期的にコマンドを実行する。
    """
    def __init__(self, ip_address="192.168.3.109", port=12345, timeout=15.0):
        """
        RobotControllerを初期化し、ロボットへの接続を試みます。
        """
        self.tcp_ip = ip_address
        self.tcp_port = port
        self.timeout = timeout
        self.sock = None
        self.connected = False
        print(f"[RobotController] Initializing for persistent connection. Target: {self.tcp_ip}:{self.tcp_port}")
        self._connect()

    def _connect(self):
        """TCP接続を確立または再確立するプライベートメソッド。"""
        # 既存のソケットがあれば安全に閉じる
        if self.sock:
            self.sock.close()
        
        try:
            print("[RobotController] Attempting to connect to the robot...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.tcp_ip, self.tcp_port))
            self.connected = True
            print("[RobotController] Connection established successfully.")
        except (socket.error, socket.timeout) as e:
            print(f"[RobotController] ERROR: Failed to connect - {e}")
            self.connected = False
            self.sock = None

    def execute_and_wait(self, command: str, value: float):
        """
        確立済みの接続を使ってコマンドを送信し、完了報告を待機します。
        """
        # 接続が切れていたら、再接続を試みる
        if not self.connected:
            print("[RobotController] Not connected. Attempting to reconnect...")
            self._connect()
            # それでも接続できなければ、例外を送出して処理を中断
            if not self.connected:
                raise socket.error("Failed to reconnect to the robot. Please check the robot.")

        try:
            # 1. コマンド送信
            message = f"{command},{value:.2f}\n"
            print(f"[RobotController] Sending command: \"{message.strip()}\"")
            self.sock.sendall(message.encode('utf-8'))

            # 2. 完了報告待機
            response = self.sock.recv(1024).decode('utf-8').strip()
            if response == "done":
                print(f"[RobotController] Received 'done' for command: {command}")
            else:
                # 予期せぬ応答（空を含む）の場合は警告を表示
                print(f"[RobotController] WARN: Received unexpected response: '{response}'")

        except (socket.timeout, socket.error) as e:
            # 送受信中にエラーが起きたら、接続が切れたと判断
            print(f"[RobotController] ERROR: Communication failed - {e}. Connection lost.")
            self.connected = False
            if self.sock:
                self.sock.close()
            self.sock = None
            # エラーを呼び出し元に伝播させ、再試行を促す
            raise

    def move(self, direction: str, distance_mm: float):
        distance_cm = distance_mm / 10.0
        self.execute_and_wait(direction, distance_cm)

    def pick_ball(self):
        self.execute_and_wait("pick", 0)

    def drop_ball(self):
        self.execute_and_wait("drop", 0)

    def close(self):
        """
        プログラム終了時にソケット接続を安全に閉じます。
        """
        if self.connected and self.sock:
            print("[RobotController] Closing connection.")
            self.sock.close()
            self.sock = None
            self.connected = False