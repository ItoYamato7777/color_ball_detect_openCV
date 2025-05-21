import numpy as np
import cv2

class CameraManager:
    """
    カメラの初期化、フレームのキャプチャ、解放を管理するクラス。
    """
    def __init__(self, camera_id=0, width=640, height=480):
        """
        カメラを初期化し、指定された解像度を設定します。

        Args:
            camera_id (int): 使用するカメラのID。
            width (int): カメラフレームの幅。
            height (int): カメラフレームの高さ。
        """
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"エラー: カメラID {self.camera_id} を開けませんでした。")
            raise IOError(f"Cannot open camera {self.camera_id}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"カメラ {self.camera_id} を解像度 {self.width}x{self.height} で開きました。")

    def read_frame(self):
        """
        カメラから1フレームを読み込みます。

        Returns:
            tuple: (bool, numpy.ndarray) フレーム取得の成否とフレーム画像。
                   失敗した場合は (False, None) を返します。
        """
        ret, frame = self.cap.read()
        if not ret:
            print("エラー: カメラからフレームを読み込めませんでした。")
            return False, None
        return ret, frame

    def release(self):
        """
        カメラリソースを解放します。
        """
        if self.cap.isOpened():
            self.cap.release()
            print(f"カメラ {self.camera_id} を解放しました。")

    def is_opened(self):
        """
        カメラが正常に開いているかを確認します。

        Returns:
            bool: カメラが開いていれば True、そうでなければ False。
        """
        return self.cap.isOpened()

    def get_frame_properties(self):
        """
        現在のフレームの幅と高さを返します。

        Returns:
            tuple: (int, int) フレームの幅と高さ。
        """
        return self.width, self.height