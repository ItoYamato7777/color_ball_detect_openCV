# module/robot_controller.py

import socket

class RobotController:
    """
    ロボットの物理的な動作を制御するためのクラス。
    1アクションごとにTCP接続を確立し、同期的にコマンドを実行する。
    """
    def __init__(self, ip_address="192.168.3.109", port=12345, timeout=15.0):
        """
        RobotControllerを初期化します。
        """
        self.tcp_ip = ip_address
        self.tcp_port = port
        self.timeout = timeout
        print(f"[RobotController] Initialized. Target: {self.tcp_ip}:{self.tcp_port}")

    def execute_and_wait(self, command: str, value: float):
        """
        1. 接続 -> 2. コマンド送信 -> 3. 完了報告待機 -> 4. 切断 の一連の処理を行う。
        """
        # <--- 毎回新しいソケットを作成 ---
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)

        try:
            # 1. 接続
            sock.connect((self.tcp_ip, self.tcp_port))
            
            # 2. コマンド送信
            message = f"{command},{value:.2f}\n"
            print(f"[RobotController] Sending command: \"{message.strip()}\"")
            sock.sendall(message.encode('utf-8'))

            # 3. 完了報告待機
            response = sock.recv(1024).decode('utf-8').strip()
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
        finally:
            # 4. 切断
            sock.close()

    def move(self, direction: str, distance_mm: float):
        distance_cm = distance_mm / 10.0
        self.execute_and_wait(direction, distance_cm)

    def pick_ball(self):
        self.execute_and_wait("pick", 0.01)

    def drop_ball(self):
        self.execute_and_wait("drop", 0.01)

    def close(self):
        # 接続を維持しないモデルになったため、このメソッドは不要になるが、
        # main.py側で呼び出されている可能性を考慮し、何もしないメソッドとして残す。
        pass