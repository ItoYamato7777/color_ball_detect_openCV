import numpy as np
import cv2
import math

class ColorBallDetector:
    """
    指定された色のボールを検出し、円と情報（名前、座標）を描画するクラス。
    検出したボールの情報をリストとして返す機能を追加。
    """
    def __init__(self, color_ranges_hsv, color_bgr_draw, morph_kernel, min_contour_area, min_radius, min_circularity=0.85):
        """
        カラーボール検出器を初期化します。

        Args:
            color_ranges_hsv (dict): 色名とHSV範囲のリストの辞書。
            color_bgr_draw (dict): 色名と描画色のBGRタプルの辞書。
            morph_kernel (numpy.ndarray): 形態学的処理に使用するカーネル。
            min_contour_area (int): 検出対象とする最小の輪郭面積。
            min_radius (int): 検出対象とする最小のボール半径。
            min_circularity (float): 検出対象とする最小の円形度。1.0に近いほど真円。
        """
        self.color_ranges = color_ranges_hsv
        self.color_bgr = color_bgr_draw
        self.kernel = morph_kernel
        self.min_contour_area = min_contour_area
        self.min_radius = min_radius
        self.min_circularity = min_circularity
        print(f"ColorBallDetector: 初期化完了。最小円形度: {self.min_circularity}")

    def detect_and_draw_balls(self, frame):
        """
        指定されたフレームから各色のボールを検出し、円、名前、座標を描画します。
        また、各色のマスク画像も表示します。

        Returns:
            tuple: (numpy.ndarray, list)
                - 描画処理が適用されたフレーム。
                - 検出された各ボールの情報 (中心座標、半径、色名) を含む辞書のリスト。
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected_balls_info = [] # 検出されたボール情報を格納するリスト

        for color_name, ranges in self.color_ranges.items():
            mask = None
            for (lower, upper) in ranges:
                current_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                if mask is None:
                    mask = current_mask
                else:
                    mask = cv2.bitwise_or(mask, current_mask)

            mask = cv2.erode(mask, self.kernel, iterations=2)
            mask = cv2.dilate(mask, self.kernel, iterations=2)

            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.imshow(f'{color_name} Mask', mask)

            ball_count = 0
            if len(contours) > 0:
                for i, c in enumerate(contours):
                    # 1. 面積が小さすぎる輪郭はノイズとして除外 (既存の処理)
                    area = cv2.contourArea(c)
                    if area < self.min_contour_area:
                        continue

                    # 2. 円形度を計算して、円らしくないものを除外
                    perimeter = cv2.arcLength(c, True)
                    # ゼロ除算を避ける
                    if perimeter == 0:
                        continue
                    
                    circularity = (4 * math.pi * area) / (perimeter * perimeter)
                    if circularity < self.min_circularity:
                        continue # 円形度がしきい値未満なら、この輪郭は無視

                    ((x_img, y_img), radius_px) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center_uv = None
                    if M["m00"] != 0:
                        center_uv = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                    if center_uv is not None and radius_px > self.min_radius:
                        ball_count += 1
                        ball_name = f"{color_name}_{ball_count}"
                        draw_color = self.color_bgr.get(color_name, (255, 255, 255))

                        cv2.circle(frame, (int(x_img), int(y_img)), int(radius_px), draw_color, 2)
                        cv2.circle(frame, center_uv, 5, draw_color, -1)
                        
                        detected_balls_info.append({
                            'name': ball_name,
                            'center_uv': center_uv,
                            'radius_px': radius_px
                        })
        
        return frame, detected_balls_info