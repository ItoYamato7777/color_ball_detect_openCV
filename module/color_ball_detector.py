import numpy as np
import cv2

class ColorBallDetector:
    """
    指定された色のボールを検出し、円と情報（名前、座標）を描画するクラス。
    """
    def __init__(self, color_ranges_hsv, color_bgr_draw, morph_kernel, min_contour_area, min_radius):
        """
        カラーボール検出器を初期化します。

        Args:
            color_ranges_hsv (dict): 色名とHSV範囲のリストの辞書。
                                     例: {'red': [([h1,s1,v1],[h2,s2,v2]), ...]}
            color_bgr_draw (dict): 色名と描画色のBGRタプルの辞書。
                                   例: {'red': (0,0,255)}
            morph_kernel (numpy.ndarray): 形態学的処理に使用するカーネル。
            min_contour_area (int): 検出対象とする最小の輪郭面積。
            min_radius (int): 検出対象とする最小のボール半径。
        """
        self.color_ranges = color_ranges_hsv #
        self.color_bgr = color_bgr_draw #
        self.kernel = morph_kernel #
        self.min_contour_area = min_contour_area #
        self.min_radius = min_radius #
        print("ColorBallDetector: 初期化完了。")

    def detect_and_draw_balls(self, frame):
        """
        指定されたフレームから各色のボールを検出し、円、名前、座標を描画します。
        また、各色のマスク画像も表示します。

        Args:
            frame (numpy.ndarray): 処理対象のカメラフレーム。

        Returns:
            numpy.ndarray: ボール情報が描画されたフレーム。
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #

        for color_name, ranges in self.color_ranges.items(): #
            mask = None
            for (lower, upper) in ranges: #
                current_mask = cv2.inRange(hsv, np.array(lower), np.array(upper)) #
                if mask is None:
                    mask = current_mask
                else:
                    mask = cv2.bitwise_or(mask, current_mask) #

            # マスクを収縮・膨張させてノイズを除去
            mask = cv2.erode(mask, self.kernel, iterations=2) #
            mask = cv2.dilate(mask, self.kernel, iterations=2) #

            # マスクから輪郭を検出
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #

            # デバッグ用にマスク画像を表示
            cv2.imshow(f'{color_name} Mask', mask)

            ball_count = 0 #
            if len(contours) > 0:
                for i, c in enumerate(contours): #
                    if cv2.contourArea(c) < self.min_contour_area: #
                        continue

                    ((x, y), radius) = cv2.minEnclosingCircle(c) #
                    M = cv2.moments(c) #
                    center = None
                    if M["m00"] != 0: #
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) #

                    if center is not None and radius > self.min_radius: #
                        ball_count += 1 #
                        ball_name = f"{color_name}_{ball_count}" #
                        draw_color = self.color_bgr.get(color_name, (255, 255, 255)) #

                        cv2.circle(frame, (int(x), int(y)), int(radius), draw_color, 2) #
                        cv2.circle(frame, center, 5, draw_color, -1) #

                        text = f"{ball_name} X:{center[0]}, Y:{center[1]}" #
                        cv2.putText(frame, text, (center[0] - 50, center[1] - 20), #
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)

                        # ターミナルに出力
                        # print(f"検出: {ball_name}, 中心座標: X={center[0]}, Y={center[1]}")
        return frame