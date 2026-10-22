#!/usr/bin/env python3
"""Program Pengolahan Kamera (HSV Vision) terintegrasi pada package robot_description.

Mendeteksi:
1. Pasangan bola merah & hijau (gerbang 01-10) beserta Bounding Box (BBox) dan titik tengah celah.
2. Kotak hijau di lantai (imaging target) beserta BBox dan titik tengah.
3. Kotak biru di lantai (imaging target) beserta BBox dan titik tengah.

Menerbitkan:
- /vision/debug_image (Image BGR8 dengan overlay BBox, crosshair tengah, garis pemandu)
- /vision/mask_red (Image mono8)
- /vision/mask_green (Image mono8)
- /vision/mask_blue (Image mono8)
- /vision/gate_info (String JSON berisi info deteksi gerbang)
- /vision/target_info (String JSON berisi info deteksi kotak hijau & biru)
"""

import json
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraVisionNode(Node):
    def __init__(self):
        super().__init__('camera_vision_node')

        # HSV Threshold parameters
        self.declare_parameter('hsv_red_low_1', [0, 90, 60])
        self.declare_parameter('hsv_red_high_1', [12, 255, 255])
        self.declare_parameter('hsv_red_low_2', [168, 90, 60])
        self.declare_parameter('hsv_red_high_2', [179, 255, 255])
        self.declare_parameter('hsv_green_low', [35, 65, 45])
        self.declare_parameter('hsv_green_high', [95, 255, 255])
        self.declare_parameter('hsv_blue_low', [90, 65, 45])
        self.declare_parameter('hsv_blue_high', [135, 255, 255])

        self.declare_parameter('ball_min_area', 25.0)
        self.declare_parameter('box_min_area', 70.0)

        self.red_lo1 = np.array(self.get_parameter('hsv_red_low_1').value, dtype=np.uint8)
        self.red_hi1 = np.array(self.get_parameter('hsv_red_high_1').value, dtype=np.uint8)
        self.red_lo2 = np.array(self.get_parameter('hsv_red_low_2').value, dtype=np.uint8)
        self.red_hi2 = np.array(self.get_parameter('hsv_red_high_2').value, dtype=np.uint8)
        self.green_lo = np.array(self.get_parameter('hsv_green_low').value, dtype=np.uint8)
        self.green_hi = np.array(self.get_parameter('hsv_green_high').value, dtype=np.uint8)
        self.blue_lo = np.array(self.get_parameter('hsv_blue_low').value, dtype=np.uint8)
        self.blue_hi = np.array(self.get_parameter('hsv_blue_high').value, dtype=np.uint8)

        self.ball_min_area = float(self.get_parameter('ball_min_area').value)
        self.box_min_area = float(self.get_parameter('box_min_area').value)

        # State
        self.current_mission_status = 'INITIALIZING'
        self.frame_width = 640
        self.frame_height = 480

        # Subscriptions
        self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            String, '/mission/status', self._mission_status_callback, 10
        )

        # Publishers
        self.debug_image_pub = self.create_publisher(Image, '/vision/debug_image', 10)
        self.mask_red_pub = self.create_publisher(Image, '/vision/mask_red', 10)
        self.mask_green_pub = self.create_publisher(Image, '/vision/mask_green', 10)
        self.mask_blue_pub = self.create_publisher(Image, '/vision/mask_blue', 10)
        self.gate_info_pub = self.create_publisher(String, '/vision/gate_info', 10)
        self.target_info_pub = self.create_publisher(String, '/vision/target_info', 10)
        self.dock_info_pub = self.create_publisher(String, '/vision/dock_info', 10)

        self.get_logger().info('Camera Vision Node (robot_description) is ready and listening to /camera/image_raw')

    def _mission_status_callback(self, msg: String):
        self.current_mission_status = msg.data

    @staticmethod
    def _decode_image(msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()
        h, w = int(msg.height), int(msg.width)
        channels = 1 if enc in ('mono8', '8uc1') else 4 if ('rgba' in enc or 'bgra' in enc) else 3
        step = int(msg.step) if msg.step else w * channels
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, step)
        pixels = rows[:, :w * channels]

        if channels == 1:
            return cv2.cvtColor(pixels.reshape(h, w), cv2.COLOR_GRAY2BGR)
        img = pixels.reshape(h, w, channels)
        if enc in ('rgb8', 'r8g8b8'):
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc in ('rgba8', 'r8g8b8a8'):
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if enc in ('bgra8', 'b8g8r8a8'):
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img.copy()

    @staticmethod
    def _to_image_msg(cv_image: np.ndarray, encoding: str, header) -> Image:
        msg = Image()
        msg.header = header
        msg.height, msg.width = cv_image.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = msg.width if encoding == 'mono8' else msg.width * 3
        msg.data = np.ascontiguousarray(cv_image).tobytes()
        return msg

    def _find_spheres(self, mask: np.ndarray):
        """Mencari kontur bola buoy (merah/hijau)."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        spheres = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.ball_min_area:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter <= 0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            x, y, w, h = cv2.boundingRect(c)
            aspect = w / max(float(h), 1.0)
            if circularity > 0.30 and 0.5 <= aspect <= 2.0:
                cx = x + w / 2.0
                cy = y + h / 2.0
                spheres.append({
                    'center': (cx, cy),
                    'bbox': (int(x), int(y), int(w), int(h)),
                    'area': float(area),
                    'circularity': float(circularity)
                })
        return spheres

    def _find_boxes(self, mask: np.ndarray):
        """Mencari kontur dinding target foto (imaging target) berwarna hijau atau biru."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.box_min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            rect = area / max(float(w * h), 1.0)
            aspect = w / max(float(h), 1.0)
            # Dinding target berdiri (rasio lebar:tinggi 0.3 s.d 4.5, keterisian rect > 0.3)
            if 0.3 <= aspect <= 4.5 and rect > 0.30:
                cx = x + w / 2.0
                cy = y + h / 2.0
                boxes.append({
                    'center': (cx, cy),
                    'bbox': (int(x), int(y), int(w), int(h)),
                    'area': float(area)
                })
        return max(boxes, key=lambda b: b['area'], default=None)

    def _find_dock_buoys(self, mask_blue: np.ndarray):
        """Mendeteksi bola biru dermaga (3 buoy sejajar)."""
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        buoys = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 15.0 or area > 6000.0:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter <= 0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            x, y, w, h = cv2.boundingRect(c)
            aspect = w / max(float(h), 1.0)
            if circularity > 0.35 and 0.5 <= aspect <= 1.8:
                buoys.append({
                    'center': (x + w / 2.0, y + h / 2.0),
                    'bbox': (int(x), int(y), int(w), int(h)),
                    'area': float(area)
                })

        count = len(buoys)
        # 3 buoy sejajar di dermaga
        found = (count >= 3) or (count == 2 and any(b['area'] > 25 for b in buoys))
        return {
            'found': found,
            'count': count,
            'buoys': buoys
        }

    def _select_gate_pair(self, red_spheres, green_spheres):
        """Memilih pasangan bola merah & hijau terdekat yang membentuk gerbang sah di depan robot.

        Aturan Gerbang Sah:
        1. Bola Hijau harus berada di sebelah KIRI dan Bola Merah di sebelah KANAN (dx_pixel = r_x - g_x > 0).
        2. Kedua bola harus berada pada jarak yang sebanding (rasio area tidak boleh jomplang akibat beda gerbang).
        3. Menolak pasangan palsu silang (misal Hijau Gerbang 2 berpasangan dengan Merah Gerbang 3).
        """
        if not red_spheres or not green_spheres:
            return None

        best_pair = None
        best_score = float('inf')

        mid_x = self.frame_width / 2.0
        mid_y = self.frame_height / 2.0

        for r in red_spheres:
            for g in green_spheres:
                # 1. Aturan Gerbang Utama: Hijau di Kiri (g['center'][0] lebih kecil), Merah di Kanan (r['center'][0] lebih besar)
                # Jarak horizontal di kamera: r_x harus lebih besar dari g_x
                dx_cam = r['center'][0] - g['center'][0]
                if dx_cam <= 10.0:
                    # Bukan gerbang yang dihadapi (merah di kiri hijau di kanan adalah arah sebaliknya / silang)
                    continue

                dy_cam = g['center'][1] - r['center'][1]
                sep = math.hypot(dx_cam, dy_cam)

                # 2. Batas pemisahan piksel
                if sep < 15.0 or sep > self.frame_width * 0.95:
                    continue

                # 3. KONSISTENSI UKURAN (ANTI-CROSS-GATE):
                # Bola pada gerbang yang sama memiliki radius fisik identik (16 cm) dan jarak ke robot serupa.
                # Jika satu bola sangat besar dan bola pasangannya sangat kecil (rasio area > 3.0),
                # berarti itu BUKAN pasangan gerbang yang sama (misal Hijau Gerbang 2 dengan Merah Gerbang 3)!
                area_r = max(1.0, r['area'])
                area_g = max(1.0, g['area'])
                area_ratio = max(area_r / area_g, area_g / area_r)
                if area_ratio > 3.0:
                    continue

                # 4. KONSISTENSI TINGGI LANTAI (GROUND PLANE):
                # Kedua bola gerbang berdiri di lantai datar. Perbedaan vertikal piksel tidak boleh melebihi pemisahan horizontal
                if abs(dy_cam) > sep * 0.75:
                    continue

                center_x = (r['center'][0] + g['center'][0]) / 2.0
                center_y = (r['center'][1] + g['center'][1]) / 2.0
                total_area = area_r + area_g

                # Prioritas utama: Gerbang paling dekat (area terbesar dan letak vertikal terendah di layar)
                # Ditambah penalti deviasi rasio area
                y_weight = max(0.3, center_y / mid_y)
                score = (abs(center_x - mid_x) + 30.0 + area_ratio * 15.0) / (total_area * y_weight + 1.0)

                if score < best_score:
                    best_score = score
                    error = (center_x - mid_x) / mid_x
                    best_pair = {
                        'center': (center_x, center_y),
                        'error': float(error),
                        'separation': float(sep),
                        'red': r,
                        'green': g
                    }
        return best_pair

    def _image_callback(self, msg: Image):
        try:
            frame = self._decode_image(msg)
        except Exception as e:
            self.get_logger().warn(f'Image decode exception: {e}')
            return

        self.frame_height, self.frame_width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), np.uint8)

        # 1. Mask Warna
        mask_red_1 = cv2.inRange(hsv, self.red_lo1, self.red_hi1)
        mask_red_2 = cv2.inRange(hsv, self.red_lo2, self.red_hi2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        mask_green = cv2.inRange(hsv, self.green_lo, self.green_hi)
        mask_blue = cv2.inRange(hsv, self.blue_lo, self.blue_hi)

        # Operasi morfologi pembersihan noise
        mask_red = cv2.morphologyEx(cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)

        # 2. Deteksi Bounding Box Bola & Gerbang
        red_spheres = self._find_spheres(mask_red)
        green_spheres = self._find_spheres(mask_green)
        gate_pair = self._select_gate_pair(red_spheres, green_spheres)

        # 3. Deteksi Kotak Hijau & Biru (Imaging Targets) & 3 Buoy Dermaga
        green_box = self._find_boxes(mask_green)
        blue_box = self._find_boxes(mask_blue)
        dock_data = self._find_dock_buoys(mask_blue)

        # 4. Gambar Debug Overlay
        debug_frame = frame.copy()

        # Garis tengah acuan robot (sumbu simetri kamera)
        mid_x = self.frame_width // 2
        mid_y = self.frame_height // 2
        cv2.line(debug_frame, (mid_x, 0), (mid_x, self.frame_height), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(debug_frame, (0, mid_y), (self.frame_width, mid_y), (100, 100, 100), 1, cv2.LINE_AA)

        # Gambarkan semua BBox bola terdeteksi
        for r in red_spheres:
            rx, ry, rw, rh = r['bbox']
            cv2.rectangle(debug_frame, (rx, ry), (rx + rw, ry + rh), (0, 0, 255), 2)
            cv2.putText(debug_frame, 'RED BUOY', (rx, max(15, ry - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        for g in green_spheres:
            gx, gy, gw, gh = g['bbox']
            cv2.rectangle(debug_frame, (gx, gy), (gx + gw, gy + gh), (0, 255, 0), 2)
            cv2.putText(debug_frame, 'GREEN BUOY', (gx, max(15, gy - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # Visualisasi Titik Tengah Gerbang Celah Merah-Hijau
        gate_info_data = {'found': False}
        if gate_pair is not None:
            gcx, gcy = int(gate_pair['center'][0]), int(gate_pair['center'][1])
            rcx, rcy = int(gate_pair['red']['center'][0]), int(gate_pair['red']['center'][1])
            green_cx, green_cy = int(gate_pair['green']['center'][0]), int(gate_pair['green']['center'][1])

            # Garis penghubung gerbang merah - hijau
            cv2.line(debug_frame, (rcx, rcy), (green_cx, green_cy), (0, 255, 255), 2, cv2.LINE_AA)
            # Crosshair titik tengah celah
            cv2.drawMarker(debug_frame, (gcx, gcy), (0, 255, 255), cv2.MARKER_CROSS, 22, 2)
            # Garis deviasi ke garis tengah layar
            cv2.line(debug_frame, (mid_x, gcy), (gcx, gcy), (0, 165, 255), 2)

            err_text = f"GATE CENTER err={gate_pair['error']:+.2f}"
            cv2.putText(debug_frame, err_text, (max(10, gcx - 60), max(20, gcy - 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)

            gate_info_data = {
                'found': True,
                'center_x': gate_pair['center'][0],
                'center_y': gate_pair['center'][1],
                'error': gate_pair['error'],
                'separation': gate_pair['separation'],
                'red_bbox': list(gate_pair['red']['bbox']),
                'green_bbox': list(gate_pair['green']['bbox'])
            }

        # Visualisasi Kotak Target Hijau & Biru
        target_info_data = {'green': {'found': False}, 'blue': {'found': False}}

        if green_box is not None:
            bx, by, bw, bh = green_box['bbox']
            bcx, bcy = int(green_box['center'][0]), int(green_box['center'][1])
            err_g = (bcx - mid_x) / float(mid_x)
            cv2.rectangle(debug_frame, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
            cv2.drawMarker(debug_frame, (bcx, bcy), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
            cv2.putText(debug_frame, f"GREEN BOX err={err_g:+.2f}", (bx, max(15, by - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
            target_info_data['green'] = {
                'found': True,
                'center_x': float(bcx),
                'center_y': float(bcy),
                'error': float(err_g),
                'bbox': [bx, by, bw, bh],
                'area': float(green_box['area'])
            }

        if blue_box is not None:
            bx, by, bw, bh = blue_box['bbox']
            bcx, bcy = int(blue_box['center'][0]), int(blue_box['center'][1])
            err_b = (bcx - mid_x) / float(mid_x)
            cv2.rectangle(debug_frame, (bx, by), (bx + bw, by + bh), (255, 140, 0), 2)
            cv2.drawMarker(debug_frame, (bcx, bcy), (255, 140, 0), cv2.MARKER_CROSS, 16, 2)
            cv2.putText(debug_frame, f"BLUE BOX err={err_b:+.2f}", (bx, max(15, by - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 140, 0), 2)
            target_info_data['blue'] = {
                'found': True,
                'center_x': float(bcx),
                'center_y': float(bcy),
                'error': float(err_b),
                'bbox': [bx, by, bw, bh],
                'area': float(blue_box['area'])
            }

        # Visualisasi Bola Biru Dermaga
        if dock_data['found']:
            for b in dock_data['buoys']:
                bx, by, bw, bh = b['bbox']
                cv2.rectangle(debug_frame, (bx, by), (bx + bw, by + bh), (255, 200, 0), 2)
                cv2.putText(debug_frame, 'DOCK BUOY', (bx, max(15, by - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 0), 1)

        # 4. Integrasi All-in-One: Panel PiP HSV Mask (Red, Green, Blue) di Pojok Kanan Atas
        # Seluruh visualisasi (Camera POV + BBox/Gap + Mask Merah/Hijau/Biru) digabung dalam 1 frame
        tw, th = 98, 68
        pad = 6
        total_w = 3 * tw + 2 * pad + 12
        panel_x1 = self.frame_width - total_w - 8
        panel_y1 = 6
        panel_x2 = self.frame_width - 8
        panel_y2 = panel_y1 + th + 18

        # Background transparan untuk panel mask
        overlay = debug_frame.copy()
        cv2.rectangle(overlay, (panel_x1, panel_y1), (panel_x2, panel_y2), (15, 15, 15), -1)
        cv2.addWeighted(overlay, 0.70, debug_frame, 0.30, 0, debug_frame)
        cv2.rectangle(debug_frame, (panel_x1, panel_y1), (panel_x2, panel_y2), (70, 70, 70), 1)

        masks_info = [
            ('RED MASK', mask_red, (0, 0, 255), (60, 60, 255)),
            ('GREEN MASK', mask_green, (0, 255, 0), (60, 255, 60)),
            ('BLUE MASK', mask_blue, (255, 180, 0), (255, 200, 60))
        ]

        for i, (title, mask, color_bgr, text_color) in enumerate(masks_info):
            bx = panel_x1 + 6 + i * (tw + pad)
            by = panel_y1 + 14

            sm = cv2.resize(mask, (tw, th), interpolation=cv2.INTER_NEAREST)
            thumb = np.full((th, tw, 3), 15, dtype=np.uint8)
            thumb[sm > 0] = color_bgr

            debug_frame[by:by + th, bx:bx + tw] = thumb
            cv2.rectangle(debug_frame, (bx, by), (bx + tw, by + th), color_bgr, 1)
            cv2.putText(debug_frame, title, (bx + 3, by - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.30, text_color, 1, cv2.LINE_AA)

        # HUD Title & Camera POV Indicator di Bagian Kiri Atas
        cv2.putText(debug_frame, "CAMERA POV [LIVE RAW + ANNOTATIONS]", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        if gate_pair is not None:
            cv2.putText(debug_frame, f"GATE: DETECTED (err={gate_pair['error']:+.2f}, sep={gate_pair['separation']:.0f}px)", (10, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(debug_frame, "GATE: VISUAL SCAN ACTIVE", (10, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

        # Status Bar di Bagian Bawah Layar
        status_banner = f"MISSION: {self.current_mission_status}"
        cv2.rectangle(debug_frame, (0, self.frame_height - 24), (self.frame_width, self.frame_height), (20, 20, 20), -1)
        cv2.putText(debug_frame, status_banner, (10, self.frame_height - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # 5. Menerbitkan Topik Vision
        self.debug_image_pub.publish(self._to_image_msg(debug_frame, 'bgr8', msg.header))
        self.mask_red_pub.publish(self._to_image_msg(mask_red, 'mono8', msg.header))
        self.mask_green_pub.publish(self._to_image_msg(mask_green, 'mono8', msg.header))
        self.mask_blue_pub.publish(self._to_image_msg(mask_blue, 'mono8', msg.header))

        self.gate_info_pub.publish(String(data=json.dumps(gate_info_data)))
        self.target_info_pub.publish(String(data=json.dumps(target_info_data)))
        self.dock_info_pub.publish(String(data=json.dumps(dock_data)))


def main(args=None):
    rclpy.init(args=args)
    node = CameraVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
