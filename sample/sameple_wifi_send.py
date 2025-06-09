import socket
import math
import time

UDP_IP = "192.168.3.73"  #GR-ROSEのIPアドレス
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