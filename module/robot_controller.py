# module/robot_controller.py (UDP + 再試行版)

import socket

class RobotController:
    """
    ロボットの物理的な動作を制御するためのクラス。
    UDP通信を使用し、応答がない場合はコマンドを再試行する。
    """
    def __init__(self, ip_address="192.168.3.109", port=12345, timeout=10.0, retries=2):
        """
        RobotControllerを初期化します。
        
        Args:
            ip_address (str): ロボットのIPアドレス。
            port (int): 使用するUDPポート。
            timeout (float): 応答を待つ秒数。
            retries (int): タイムアウトした場合の再試行回数。
        """
        self.robot_address = (ip_address, port)
        self.timeout = timeout
        self.max_retries = retries
        
        # UDPソケットを作成
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 受信タイムアウトを設定
        self.sock.settimeout(self.timeout)
        
        print(f"[RobotController] Initialized for UDP communication. Target: {self.robot_address}")

    def send_command_with_retry(self, command: str, value: float):
        """
        コマンドを送信し、'done'が返ってくるまで指定回数再試行します。
        """
        message = f"{command},{value:.2f}\n"
        encoded_message = message.encode('utf-8')
        
        for attempt in range(self.max_retries):
            try:
                # 1. コマンド送信
                print(f"[RobotController] Sending command (Attempt {attempt + 1}/{self.max_retries}): \"{message.strip()}\"")
                self.sock.sendto(encoded_message, self.robot_address)

                # 2. 応答待機
                response, addr = self.sock.recvfrom(1024)
                decoded_response = response.decode('utf-8').strip()

                if decoded_response == "done":
                    print(f"[RobotController] Received 'done' for command: {command}")
                    return True # 成功したので関数を抜ける
                else:
                    print(f"[RobotController] WARN: Received unexpected response: '{decoded_response}'")

            except socket.timeout:
                # タイムアウト例外が発生した場合
                print(f"[RobotController] WARN: Timeout waiting for response from robot.")
                if attempt == self.max_retries - 1:
                    print(f"[RobotController] ERROR: Command failed after {self.max_retries} attempts.")
                    raise # 最終試行でも失敗したら例外を発生させる
        
        return False # ここには到達しないはずだが、念のため

    def move(self, direction: str, distance_mm: float):
        distance_cm = distance_mm / 10.0
        self.send_command_with_retry(direction, distance_cm)

    def pick_ball(self):
        self.send_command_with_retry("pick", 0)

    def drop_ball(self):
        self.send_command_with_retry("drop", 0)

    def close(self):
        """
        プログラム終了時にソケットを閉じます。
        """
        if self.sock:
            print("[RobotController] Closing socket.")
            self.sock.close()