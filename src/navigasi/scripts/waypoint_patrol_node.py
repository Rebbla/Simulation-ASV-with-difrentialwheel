#!/usr/bin/env python3
"""Program Navigasi Waypoint Patrol (package navigasi).

Mengontrol robot diff-drive:
1. Mengikuti urutan 10 gerbang (S-curve: buoy 03 -> 02 -> 01 -> 07..04 -> 08..10).
2. Setelah gerbang 1 (buoy 03) lolos, robot memutar haluan ke KANAN menuju gerbang 2.
3. Setelah gerbang 2 (buoy 02) lolos, deteksi plane-crossing langsung memicu BELOK KIRI ke gerbang 3.
4. Visual servoing kamera memusatkan celah merah-hijau saat melintasi gerbang.
5. Memusatkan kamera pada kotak biru dan hijau, berhenti dan menyimpan foto PNG.
6. Kembali ke dermaga melalui ramp dan mencatat penyelesaian misi di CSV log.
"""

import csv
import json
import math
import os
import time
import traceback
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _clamp(val, low, high):
    return max(low, min(high, val))


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _unit_vector(dx, dy):
    length = math.hypot(dx, dy)
    return (1.0, 0.0) if length < 1e-8 else (dx / length, dy / length)


def _quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class WaypointPatrolNode(Node):
    def __init__(self):
        super().__init__('waypoint_patrol_node')

        # ── Parameter Konfigurasi ─────────────────────────────────────
        self.declare_parameter('autonomous', True)
        self.declare_parameter('initial_world_pose', [-2.83, -3.41, 1.5708])
        self.declare_parameter('gate_red_world', [0.0, 0.0])
        self.declare_parameter('gate_green_world', [0.0, 0.0])
        self.declare_parameter('blue_target_world', [2.59, -2.29])
        self.declare_parameter('green_target_world', [1.89, -3.15])
        self.declare_parameter('dock_world_pose', [-2.83, -3.41, 1.5708])
        self.declare_parameter('dock_ramp_approach_world', [-2.83, -2.85])

        self.declare_parameter('gate_approach_distance', 0.55)
        self.declare_parameter('gate_exit_distance', 0.25)
        self.declare_parameter('gate_turn_distance', 0.45)
        self.declare_parameter('target_standoff_distance', 0.50)
        self.declare_parameter('capture_stable_frames', 6)

        self.declare_parameter('max_linear_speed', 0.28)
        self.declare_parameter('cross_speed', 0.16)
        self.declare_parameter('approach_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.85)
        self.declare_parameter('waypoint_tolerance', 0.16)
        self.declare_parameter('gate_center_tolerance', 0.15)
        self.declare_parameter('target_center_tolerance', 0.15)

        self.declare_parameter('capture_directory', '~/Documents/UTSRE502/captures')

        # Baca Nilai Parameter
        self.autonomous = bool(self.get_parameter('autonomous').value)
        self.start_world = [float(v) for v in self.get_parameter('initial_world_pose').value]
        self.reds = self._parse_pairs(self.get_parameter('gate_red_world').value)
        self.greens = self._parse_pairs(self.get_parameter('gate_green_world').value)

        self.blue_box = [float(v) for v in self.get_parameter('blue_target_world').value]
        self.green_box = [float(v) for v in self.get_parameter('green_target_world').value]
        self.dock = [float(v) for v in self.get_parameter('dock_world_pose').value]
        self.ramp = [float(v) for v in self.get_parameter('dock_ramp_approach_world').value]

        self.gate_approach_dist = float(self.get_parameter('gate_approach_distance').value)
        self.gate_exit_dist = float(self.get_parameter('gate_exit_distance').value)
        self.gate_turn_dist = float(self.get_parameter('gate_turn_distance').value)
        self.standoff_dist = float(self.get_parameter('target_standoff_distance').value)
        self.capture_frames_req = int(self.get_parameter('capture_stable_frames').value)

        self.max_speed = float(self.get_parameter('max_linear_speed').value)
        self.cross_speed = float(self.get_parameter('cross_speed').value)
        self.approach_speed = float(self.get_parameter('approach_speed').value)
        self.max_turn = float(self.get_parameter('max_angular_speed').value)
        self.waypoint_tol = float(self.get_parameter('waypoint_tolerance').value)
        self.gate_center_tol = float(self.get_parameter('gate_center_tolerance').value)
        self.target_center_tol = float(self.get_parameter('target_center_tolerance').value)

        cap_dir = os.path.expanduser(str(self.get_parameter('capture_directory').value))
        self.capture_dir = Path(cap_dir)
        self.green_dir = self.capture_dir / 'green'
        self.blue_dir = self.capture_dir / 'blue'
        self.green_dir.mkdir(parents=True, exist_ok=True)
        self.blue_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.capture_dir / 'mission_log.csv'
        self.gate_report_path = self.capture_dir / 'gate_passed_report.txt'
        self.gate_start_time = time.monotonic()

        # ── State Variabel ───────────────────────────────────────────
        self.pose = None           # (x, y, yaw) dalam world frame
        self.odom_origin = None    # Nilai odom saat start
        self.ready = False
        self.gates = []
        self.path_world = []
        self.gate_index = 0
        self.sub_wp_index = 0
        self.phase = 'WAIT_ODOM' if self.autonomous else 'MANUAL_DEBUG'

        # Kamera & Vision data
        self.latest_raw_frame = None
        self.latest_gate_info = {'found': False}
        self.latest_target_info = {'green': {'found': False}, 'blue': {'found': False}}
        self.latest_dock_info = {'found': False, 'count': 0}
        self.last_gate_detection_time = 0.0
        self.target_stable_count = 0
        self.capture_phase_start_time = 0.0
        self.last_status_text = ''

        # ── Komunikasi ROS 2 ─────────────────────────────────────────
        self.create_subscription(Odometry, '/odom', self._odom_callback, 20)
        self.create_subscription(Image, '/camera/image_raw', self._raw_image_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/vision/gate_info', self._gate_info_callback, 10)
        self.create_subscription(String, '/vision/target_info', self._target_info_callback, 10)
        self.create_subscription(String, '/vision/dock_info', self._dock_info_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(PathMsg, '/mission/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission/markers', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # Timer loop kontrol 20 Hz
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('Waypoint Patrol Node aktif dan siap.')

    @staticmethod
    def _parse_pairs(values):
        flt = [float(v) for v in values]
        if len(flt) % 2 != 0:
            raise ValueError('Koordinat pairs harus genap (x, y)')
        return list(zip(flt[0::2], flt[1::2]))

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose
        x_o = p.position.x
        y_o = p.position.y
        yaw_o = _quaternion_to_yaw(p.orientation)

        if self.odom_origin is None:
            self.odom_origin = (x_o, y_o, yaw_o)
            self._generate_route()
            self.ready = True
            if self.autonomous:
                self.phase = 'DOCK_EXIT'
                self.get_logger().info('Odometry diterima. Memulai misi di fase DOCK_EXIT...')

        # Konversi pose odom ke world frame sesuai posisi spawn Gazebo
        x0, y0, yaw0 = self.odom_origin
        sx, sy, syaw = self.start_world

        dx_o = x_o - x0
        dy_o = y_o - y0
        dyaw_o = yaw_o - yaw0

        x_rel = math.cos(-yaw0) * dx_o - math.sin(-yaw0) * dy_o
        y_rel = math.sin(-yaw0) * dx_o + math.cos(-yaw0) * dy_o

        w_x = sx + math.cos(syaw) * x_rel - math.sin(syaw) * y_rel
        w_y = sy + math.sin(syaw) * x_rel + math.cos(syaw) * y_rel
        w_yaw = _wrap_angle(syaw + dyaw_o)

        self.pose = (w_x, w_y, w_yaw)

    def _raw_image_callback(self, msg: Image):
        try:
            h, w = int(msg.height), int(msg.width)
            enc = msg.encoding.lower()
            channels = 1 if enc in ('mono8', '8uc1') else 4 if ('rgba' in enc or 'bgra' in enc) else 3
            step = int(msg.step) if msg.step else w * channels
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, step)
            pixels = rows[:, :w * channels]
            if enc in ('rgb8', 'r8g8b8'):
                self.latest_raw_frame = cv2.cvtColor(pixels.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
            elif enc in ('bgr8', '8uc3'):
                self.latest_raw_frame = pixels.reshape(h, w, 3).copy()
        except Exception:
            pass

    def _gate_info_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_gate_info = data
            if data.get('found', False):
                self.last_gate_detection_time = time.monotonic()
        except Exception:
            pass

    def _target_info_callback(self, msg: String):
        try:
            self.latest_target_info = json.loads(msg.data)
        except Exception:
            pass

    def _dock_info_callback(self, msg: String):
        try:
            self.latest_dock_info = json.loads(msg.data)
        except Exception:
            pass

    def _world_to_odom(self, point, target_yaw=None):
        ox, oy, oyaw = self.odom_origin
        sx, sy, syaw = self.start_world
        d_angle = oyaw - syaw
        dx, dy = point[0] - sx, point[1] - sy

        out_x = ox + math.cos(d_angle) * dx - math.sin(d_angle) * dy
        out_y = oy + math.sin(d_angle) * dx + math.cos(d_angle) * dy
        out_yaw = None if target_yaw is None else oyaw + _wrap_angle(target_yaw - syaw)
        return out_x, out_y, out_yaw

    def _generate_route(self):
        """Membangun lintasan waypoint 10 gerbang (S-Curve & sirkuit) beserta turn waypoint."""
        start_pt = tuple(self.start_world[:2])
        # Titik keluar dermaga lurus
        dock_exit_pt = (
            self.start_world[0] + 0.85 * math.cos(self.start_world[2]),
            self.start_world[1] + 0.85 * math.sin(self.start_world[2])
        )
        prev_point = dock_exit_pt
        self.gates = []
        self.path_world = [start_pt, dock_exit_pt]

        for i, (red, green) in enumerate(zip(self.reds, self.greens)):
            center = ((red[0] + green[0]) / 2.0, (red[1] + green[1]) / 2.0)
            axis = _unit_vector(green[0] - red[0], green[1] - red[1])
            normal = (-axis[1], axis[0])  # Vektor tegak lurus sumbu gerbang (arah maju)
            incoming = (center[0] - prev_point[0], center[1] - prev_point[1])
            if normal[0] * incoming[0] + normal[1] * incoming[1] < 0:
                normal = (-normal[0], -normal[1])

            # Ekstra jarak keluar pada Gerbang 7 (exit_d = 0.35m) agar bodi belakang robot
            # benar-benar bebas dari Buoy Merah 04 sebelum robot memulai putaran menuju Gerbang 8
            exit_d = 0.35 if (i + 1) == 7 else self.gate_exit_dist
            approach = (center[0] - normal[0] * self.gate_approach_dist,
                        center[1] - normal[1] * self.gate_approach_dist)
            exit_pt = (center[0] + normal[0] * exit_d,
                       center[1] + normal[1] * exit_d)

            self.gates.append({
                'index': i + 1,
                'red': red,
                'green': green,
                'center': center,
                'normal': normal,
                'approach': approach,
                'exit': exit_pt,
                'turn': exit_pt,
                'approach_waypoints': [approach]
            })
            prev_point = center

        # Sisipkan Waypoint Lengkung Masuk Koridor (Anti-Tabrak Sudut Dalam)
        for i in range(len(self.gates)):
            curr = self.gates[i]
            if i == 0:
                curr['approach_waypoints'] = [curr['approach']]
            else:
                prev = self.gates[i - 1]
                p_exit = prev['exit']
                n_prev = prev['normal']
                c_next = curr['center']
                n_next = curr['normal']
                dot_normals = n_prev[0] * n_next[0] + n_prev[1] * n_next[1]

                # Jika orientasi gerbang berbelok tajam (> 50 derajat, misal Gerbang 3->4 atau 7->8)
                if dot_normals < 0.65:
                    if abs(n_prev[1]) > abs(n_prev[0]):
                        p_corner = (p_exit[0], c_next[1])
                    else:
                        p_corner = (c_next[0], p_exit[1])

                    r = 0.60
                    p_start = (p_corner[0] - r * n_prev[0], p_corner[1] - r * n_prev[1])
                    p_mid = (p_corner[0] - r * 0.293 * n_prev[0] + r * 0.293 * n_next[0],
                             p_corner[1] - r * 0.293 * n_prev[1] + r * 0.293 * n_next[1])

                    curr['approach_waypoints'] = [p_start, p_mid, curr['approach']]
                else:
                    # Cek pergeseran lateral (lateral offset) antar gerbang berurutan
                    dx = c_next[0] - p_exit[0]
                    dy = c_next[1] - p_exit[1]
                    lateral_offset = abs(-n_next[1] * dx + n_next[0] * dy)
                    if lateral_offset > 0.18:
                        # Sisipkan titik transisi lateral agar robot sejajar dengan garis tengah gerbang berikutnya sebelum masuk
                        # dan tidak memotong sudut buoy bagian dalam (misal Buoy Merah 09 setelah Gerbang 08)
                        p_shift = (c_next[0] - n_next[0] * (self.gate_approach_dist + 0.22),
                                   c_next[1] - n_next[1] * (self.gate_approach_dist + 0.22))
                        curr['approach_waypoints'] = [p_shift, curr['approach']]
                    else:
                        curr['approach_waypoints'] = [curr['approach']]

        # Hitung Turn Waypoint untuk setiap gerbang & susun jalur /mission/path lengkap
        for i in range(len(self.gates)):
            curr_exit = self.gates[i]['exit']
            if i + 1 < len(self.gates):
                next_target = self.gates[i + 1]['approach_waypoints'][0]
                dist_to_next = math.hypot(next_target[0] - curr_exit[0], next_target[1] - curr_exit[1])
                # Batasi jarak turn agar tidak melampaui approach waypoint gerbang berikutnya
                turn_dist = min(self.gate_turn_dist, max(0.04, dist_to_next * 0.40))
            else:
                sgn_exit = 1.0 if self.blue_box[0] > 0 else -1.0
                next_target = (sgn_exit * 3.15, -1.60)
                turn_dist = self.gate_turn_dist

            turn_dir = _unit_vector(next_target[0] - curr_exit[0], next_target[1] - curr_exit[1])
            turn_pt = (curr_exit[0] + turn_dir[0] * turn_dist,
                       curr_exit[1] + turn_dir[1] * turn_dist)
            self.gates[i]['turn'] = turn_pt

            self.path_world += self.gates[i]['approach_waypoints'] + [curr_exit, turn_pt]

        # Waypoint Target Kotak Biru & Hijau (Dilewati di jalur air aman), Koridor, dan Dock
        sgn = 1.0 if self.blue_box[0] > 0 else -1.0
        # 1. Jalur air di sebelah luar Kotak Biru (Kotak Biru di x = sgn*2.40, y = -2.10)
        self.blue_approach = (sgn * 3.15, -1.60)
        self.blue_photo = (sgn * 3.00, -2.10)
        self.blue_pass = (sgn * 3.10, -2.60)

        # 2. Tikungan sudut tenggara melengkung mulus
        self.corner_turn = (sgn * 2.80, -3.00)

        # 3. Jalur air di sebelah utara Kotak Hijau (Kotak Hijau di x = sgn*1.50, y = -3.60)
        self.green_approach = (sgn * 2.10, -2.90)
        self.green_photo = (sgn * 1.60, -2.90)
        self.green_pass = (sgn * 0.80, -2.90)

        # 4. Koridor lurus bawah menyusuri dinding selatan menuju dermaga
        self.dock_corridor = (sgn * -1.50, -2.90)
        self.dock_mid = (sgn * -2.40, -2.90)

        self.path_world += [
            self.blue_approach, self.blue_photo, self.blue_pass,
            self.corner_turn,
            self.green_approach, self.green_photo, self.green_pass,
            self.dock_corridor, self.dock_mid,
            tuple(self.ramp), tuple(self.dock[:2])
        ]

    def _calc_standoff(self, from_pt, target_pt):
        d = _unit_vector(target_pt[0] - from_pt[0], target_pt[1] - from_pt[1])
        return (target_pt[0] - d[0] * self.standoff_dist,
                target_pt[1] - d[1] * self.standoff_dist)

    def _distance_to_target(self, target_world):
        return math.hypot(target_world[0] - self.pose[0], target_world[1] - self.pose[1])

    def _navigate_to_waypoint(self, target_world, speed, cmd: Twist):
        dx = target_world[0] - self.pose[0]
        dy = target_world[1] - self.pose[1]
        dist = math.hypot(dx, dy)

        if dist <= self.waypoint_tol:
            return True

        target_heading = math.atan2(dy, dx)
        heading_err = _wrap_angle(target_heading - self.pose[2])

        # Kendali putar proporsional responsif (gain 1.8)
        cmd.angular.z = _clamp(1.8 * heading_err, -self.max_turn, self.max_turn)

        # Kurva kecepatan kontinu dengan prioritas belok (turn-in-place priority):
        # Saat sudut deviasi haluan masih lebar, kecepatan linier dikurangi secara halus
        # agar robot memutar haluan terlebih dahulu dan tidak meluncur/memotong sudut (anti corner-cutting)
        alignment = max(0.0, math.cos(heading_err))
        cmd.linear.x = min(speed, max(0.04, 0.5 * dist)) * (alignment ** 2.5)
        return False

    def _save_target_photo(self, color_name: str) -> str:
        if self.latest_raw_frame is None:
            return ''
        folder = self.green_dir if color_name == 'green' else self.blue_dir
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = f"{color_name}_box_{timestamp}.png"
        full_path = folder / filename
        cv2.imwrite(str(full_path), self.latest_raw_frame)
        self.get_logger().info(f"📸 Foto {color_name.upper()} tersimpan: {full_path}")
        return str(full_path)

    def _log_mission_event(self, event: str, target: str, note: str = ''):
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        is_new = not self.csv_path.exists()
        with self.csv_path.open('a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if is_new:
                writer.writerow(['timestamp', 'event', 'target', 'note'])
            writer.writerow([datetime.now().isoformat(timespec='milliseconds'), event, target, note])

    def _log_gate_success(self, gate):
        duration = time.monotonic() - getattr(self, 'gate_start_time', time.monotonic())
        idx = gate['index']
        total = len(self.gates)
        now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        x = self.pose[0] if self.pose else 0.0
        y = self.pose[1] if self.pose else 0.0
        yaw = math.degrees(self.pose[2]) if self.pose else 0.0

        # 1. Catat ke file teks laporan pencapaian gerbang resmi
        try:
            self.capture_dir.mkdir(parents=True, exist_ok=True)
            is_new = not self.gate_report_path.exists()
            with self.gate_report_path.open('a', encoding='utf-8') as f:
                if is_new:
                    f.write("=" * 88 + "\n")
                    f.write("LAPORAN RESMI KEBERHASILAN PATROLI GERBANG (AUTONOMOUS PATROL MISSION)\n")
                    f.write("=" * 88 + "\n")
                    f.write(f"{'No':<4} | {'Gerbang':<12} | {'Waktu Selesai':<23} | {'Durasi':<10} | {'Koordinat Exit':<18} | {'Status':<10}\n")
                    f.write("-" * 88 + "\n")
                f.write(f"{idx:02d}   | Gerbang {idx:02d}    | {now_str:<23} | {duration:6.2f} s  | ({x:5.2f}, {y:5.2f})    | BERHASIL\n")
        except Exception as e:
            self.get_logger().warn(f"Gagal menulis gate_report_path: {e}")

        # 2. Cetak banner informatif di console log
        self.get_logger().info(
            f"\n╔════════════════════════════════════════════════════════════════════╗\n"
            f"║ 🎯 GERBANG {idx:02d} BERHASIL DILALUI SEMPURNA!                             ║\n"
            f"║ ⏱️  Waktu Selesai : {now_str:<32} ║\n"
            f"║ ⏳ Durasi Gerbang : {duration:5.2f} detik                                  ║\n"
            f"║ 📍 Posisi Robot   : x={x:5.2f}m, y={y:5.2f}m, yaw={yaw:5.1f}°                  ║\n"
            f"║ 📊 Progres Misi   : {idx:02d} dari {total:02d} Gerbang Selesai Dilalui               ║\n"
            f"╚════════════════════════════════════════════════════════════════════╝"
        )

        # 3. Catat di mission log CSV
        self._log_mission_event('GATE_PASSED', f"gate_{idx:02d}", f"passed at exit ({x:.2f}, {y:.2f}) in {duration:.2f}s")

    def _control_loop(self):
        try:
            self._control_step()
        except Exception as e:
            self.get_logger().error(f"Pengecualian tak terduga di _control_loop ({getattr(self, 'phase', 'UNKNOWN')}): {e}\n{traceback.format_exc()}")
            cmd_stop = Twist()
            self.cmd_pub.publish(cmd_stop)

    def _control_step(self):
        if not self.ready or self.pose is None:
            self._update_status('WAITING_FOR_ODOM')
            return

        self._publish_visualizations()

        if not self.autonomous:
            self._update_status('MANUAL_MODE')
            return

        cmd = Twist()

        # ── 0. Meluncur Lurus Keluar Dermaga (DOCK_EXIT) ───────────────
        if self.phase == 'DOCK_EXIT':
            # Kunci haluan tegak lurus ke Utara (+Y, syaw = 1.5708)
            heading_err = _wrap_angle(self.start_world[2] - self.pose[2])
            cmd.angular.z = _clamp(1.5 * heading_err, -0.25, 0.25)
            cmd.linear.x = 0.16  # Kecepatan stabil menuruni tanjakan ramp

            # Hitung jarak maju dari dermaga sepanjang sumbu hadap awal
            forward_dist = (self.pose[0] - self.start_world[0]) * math.cos(self.start_world[2]) + \
                           (self.pose[1] - self.start_world[1]) * math.sin(self.start_world[2])

            # Ramp berakhir di ~0.58m, ambang aman di lantai datar adalah 0.85m
            if forward_dist >= 0.85:
                self.get_logger().info('✅ Keluar dermaga selesai. Memulai navigasi S-curve ke Gerbang 01...')
                self.gate_index = 0
                self._set_phase('GATE_APPROACH')

        # ── 1. Pendekatan Gerbang (Memadukan Lengkung Koridor & Visual Kamera) ───
        elif self.phase == 'GATE_APPROACH':
            gate = self.gates[self.gate_index]
            app_wps = gate.get('approach_waypoints', [gate['approach']])

            # Jika gerbang memiliki titik lengkung transisi koridor (misal tikungan Gerbang 4 atau 8)
            if self.sub_wp_index < len(app_wps) - 1:
                target_wp = app_wps[self.sub_wp_index]
                if self._navigate_to_waypoint(target_wp, self.max_speed, cmd):
                    self.sub_wp_index += 1
                    self.get_logger().info(f"Lengkung Masuk Gerbang {gate['index']:02d}: WP {self.sub_wp_index}/{len(app_wps)} tercapai.")
            else:
                # Titik approach final (sudah lurus di garis tengah koridor di depan mulut gerbang)
                final_app = app_wps[-1]
                dist_to_center = self._distance_to_target(gate['center'])
                dist_to_app = self._distance_to_target(final_app)

                is_vision_fresh = (
                    self.latest_gate_info.get('found', False) and
                    (time.monotonic() - self.last_gate_detection_time < 0.45)
                )

                # Hitung sudut ke titik approach dan center gerbang
                dx_app = final_app[0] - self.pose[0]
                dy_app = final_app[1] - self.pose[1]
                heading_to_app = math.atan2(dy_app, dx_app)
                heading_err_app = _wrap_angle(heading_to_app - self.pose[2])

                dx_c = gate['center'][0] - self.pose[0]
                dy_c = gate['center'][1] - self.pose[1]
                nav_steer = _wrap_angle(math.atan2(dy_c, dx_c) - self.pose[2])

                # Robot hanya menggunakan visual servoing jika:
                # 1. Jarak ke center berada dalam rentang pandang efektif gerbang (0.28m s.d. 0.85m)
                #    Jika < 0.28m, buoy gerbang sudah berada di luar FOV kamera (azimuth > 40 deg),
                #    sehingga deteksi apapun adalah gerbang lain di belakangnya (false lock-on).
                # 2. Dan haluan robot sudah menghadap ke arah sasaran (|heading_err_app| < 0.5 rad)
                # 3. Dan arah koreksi visual tidak kontradiktif dengan arah center gerbang (|visual_steer - nav_steer| < 0.45 rad)
                use_vision = False
                if is_vision_fresh and (0.28 <= dist_to_center <= 0.85) and abs(heading_err_app) < 0.50:
                    error_x = self.latest_gate_info['error']
                    visual_steer = -1.5 * error_x
                    if abs(visual_steer - nav_steer) < 0.45:
                        use_vision = True

                if use_vision:
                    cmd.angular.z = _clamp(0.60 * visual_steer + 0.40 * nav_steer, -self.max_turn, self.max_turn)
                    cmd.linear.x = self.approach_speed if abs(error_x) > 0.20 else self.cross_speed
                else:
                    self._navigate_to_waypoint(final_app, self.max_speed, cmd)

                # Transisi ke CROSS_GATE jika sudah mencapai approach point atau dekat mulut gerbang
                if dist_to_app <= self.waypoint_tol or dist_to_center <= 0.25:
                    self.get_logger().info(f"Mendekati Gerbang {gate['index']:02d}. Mengunci visual celah...")
                    self._set_phase('CROSS_GATE')

        # ── 2. Menembus Gerbang dengan Visual Servoing Celah ───────────
        elif self.phase == 'CROSS_GATE':
            gate = self.gates[self.gate_index]
            dist_to_exit = self._distance_to_target(gate['exit'])

            # Hitung kemajuan sepanjang normal gerbang (Plane-crossing detection dalam world frame)
            dx_c = self.pose[0] - gate['center'][0]
            dy_c = self.pose[1] - gate['center'][1]
            dist_to_center = math.hypot(dx_c, dy_c)
            nx = gate['normal'][0]
            ny = gate['normal'][1]
            forward_progress = dx_c * nx + dy_c * ny

            is_vision_fresh = (
                self.latest_gate_info.get('found', False) and
                (time.monotonic() - self.last_gate_detection_time < 0.45)
            )

            # Arahkan robot ke exit point
            dx_ex = gate['exit'][0] - self.pose[0]
            dy_ex = gate['exit'][1] - self.pose[1]
            heading_to_exit = math.atan2(dy_ex, dx_ex)
            steer = _wrap_angle(heading_to_exit - self.pose[2])

            # Gunakan visual servoing HANYA jika sebelum melewati garis buoys (forward_progress < -0.05)
            # dan buoy masih dalam FOV kamera (dist_to_center > 0.25).
            # Begitu robot berada di antara atau melewati buoys (forward_progress >= -0.05),
            # buoy gerbang ini sudah out of FOV sehingga kamera pasti melihat gerbang lain (misal Gerbang 8 saat di Gerbang 7).
            use_vision = False
            if is_vision_fresh and forward_progress < -0.05 and dist_to_center > 0.25:
                error_x = self.latest_gate_info['error']
                visual_steer = -1.5 * error_x
                if abs(visual_steer - steer) < 0.45:
                    use_vision = True

            if use_vision:
                cmd.angular.z = _clamp(0.60 * visual_steer + 0.40 * steer, -self.max_turn, self.max_turn)
                cmd.linear.x = self.cross_speed if abs(error_x) < 0.25 else self.cross_speed * 0.7
            else:
                cmd.angular.z = _clamp(1.5 * steer, -self.max_turn, self.max_turn)
                cmd.linear.x = self.cross_speed

            # DETEKSI LOLOS GERBANG ANTI-STUCK:
            # Robot baru dinyatakan lolos jika bodi fisik sudah sepenuhnya melewati celah buoys (forward_progress >= 0.75 * exit_dist)
            if forward_progress >= 0.75 * self.gate_exit_dist or dist_to_exit <= self.waypoint_tol:
                self._log_gate_success(gate)
                self._set_phase('GATE_TURN')

        # ── 3. Solusi Robot S-Curve: Belok Aktif Menuju Gerbang Berikutnya ───
        elif self.phase == 'GATE_TURN':
            gate = self.gates[self.gate_index]
            if self.gate_index + 1 < len(self.gates):
                next_target = self.gates[self.gate_index + 1]['approach_waypoints'][0]
            else:
                next_target = tuple(self.blue_box)

            # Hitung sudut hadap langsung ke gerbang berikutnya (world frame)
            dx_next = next_target[0] - self.pose[0]
            dy_next = next_target[1] - self.pose[1]
            target_heading = math.atan2(dy_next, dx_next)
            heading_err = _wrap_angle(target_heading - self.pose[2])

            # Putar haluan responsif
            cmd.angular.z = _clamp(1.8 * heading_err, -self.max_turn, self.max_turn)

            dist_to_turn = self._distance_to_target(gate['turn'])

            # Kecepatan maju kontinu dan mulus, tidak langsung di-nolkan secara kasar
            alignment = max(0.0, math.cos(heading_err))
            cmd.linear.x = min(self.max_speed * 0.5, 0.4 * dist_to_turn) * (alignment ** 2.0)

            # Selesai fase belok jika sudah dekat turn point DAN haluan sudah menghadap target berikutnya
            if dist_to_turn <= self.waypoint_tol or (dist_to_turn <= self.waypoint_tol * 1.5 and abs(heading_err) < 0.25):
                self.gate_index += 1
                self.sub_wp_index = 0
                if self.gate_index < len(self.gates):
                    self._set_phase('GATE_APPROACH')
                else:
                    self.get_logger().info('🏁 Semua 10 Gerbang Selesai! Menuju & Melewati Kotak Biru...')
                    self._set_phase('BLUE_APPROACH')

        # ── 4. Menuju Kotak Biru, Foto dari Samping, & Melaju MELEWATI Kotak ─────────
        elif self.phase == 'BLUE_APPROACH':
            if self._navigate_to_waypoint(self.blue_approach, self.max_speed, cmd):
                self._set_phase('CAPTURE_BLUE')

        elif self.phase == 'CAPTURE_BLUE':
            elapsed = time.monotonic() - self.capture_phase_start_time
            blue_det = self.latest_target_info.get('blue', {})

            if blue_det.get('found', False):
                err = blue_det['error']
                cmd.angular.z = _clamp(-1.2 * err, -0.35, 0.35)
                cmd.linear.x = 0.08  # Melaju pelan di samping kotak
                if abs(err) < self.target_center_tol:
                    self.target_stable_count += 1
                else:
                    self.target_stable_count = 0
            else:
                self._navigate_to_waypoint(self.blue_photo, self.approach_speed, cmd)

            # Syarat foto: Target stabil beberapa frame ATAU timeout 2.0s
            if self.target_stable_count >= 4 or elapsed >= 2.0:
                photo_path = self._save_target_photo('blue')
                self._log_mission_event('TARGET_CAPTURED', 'blue_box', photo_path)
                self.get_logger().info('📸 Kotak Biru difoto & disimpan! Melaju di jalur air MELEWATI Kotak Biru...')
                self._set_phase('BLUE_PASS')

        elif self.phase == 'BLUE_PASS':
            # Melaju di jalur air terbuka MELEWATI Kotak Biru hingga tuntas
            if self._navigate_to_waypoint(self.blue_pass, self.max_speed, cmd):
                self.get_logger().info('✅ Berhasil MELEWATI Kotak Biru! Melengkung di sudut menuju Kotak Hijau...')
                self._set_phase('CORNER_TURN')

        elif self.phase == 'CORNER_TURN':
            # Mengitari sudut tenggara secara mulus
            if self._navigate_to_waypoint(self.corner_turn, self.max_speed, cmd):
                self._set_phase('GREEN_APPROACH')

        # ── 5. Menuju Kotak Hijau, Foto dari Samping, & Melaju MELEWATI Kotak ────────
        elif self.phase == 'GREEN_APPROACH':
            if self._navigate_to_waypoint(self.green_approach, self.max_speed, cmd):
                self._set_phase('CAPTURE_GREEN')

        elif self.phase == 'CAPTURE_GREEN':
            elapsed = time.monotonic() - self.capture_phase_start_time
            green_det = self.latest_target_info.get('green', {})

            if green_det.get('found', False):
                err = green_det['error']
                cmd.angular.z = _clamp(-1.2 * err, -0.35, 0.35)
                cmd.linear.x = 0.08  # Melaju pelan di samping kotak
                if abs(err) < self.target_center_tol:
                    self.target_stable_count += 1
                else:
                    self.target_stable_count = 0
            else:
                self._navigate_to_waypoint(self.green_photo, self.approach_speed, cmd)

            if self.target_stable_count >= 4 or elapsed >= 2.0:
                photo_path = self._save_target_photo('green')
                self._log_mission_event('TARGET_CAPTURED', 'green_box', photo_path)
                self.get_logger().info('📸 Kotak Hijau difoto & disimpan! Melaju di jalur air MELEWATI Kotak Hijau...')
                self._set_phase('GREEN_PASS')

        elif self.phase == 'GREEN_PASS':
            # Melaju di jalur air terbuka MELEWATI Kotak Hijau hingga tuntas
            if self._navigate_to_waypoint(self.green_pass, self.max_speed, cmd):
                self.get_logger().info('✅ Berhasil MELEWATI Kotak Hijau! Melaju di koridor bawah menuju Dermaga...')
                self._set_phase('DOCK_CORRIDOR')

        # ── 6. Koridor Lurus Bawah Menuju & BERHENTI di 3 Buoy Biru Dermaga ─────
        elif self.phase == 'DOCK_CORRIDOR':
            if self._navigate_to_waypoint(self.dock_corridor, self.max_speed, cmd):
                self._set_phase('DOCK_APPROACH')

        elif self.phase == 'DOCK_APPROACH':
            if self._navigate_to_waypoint(self.dock_mid, self.max_speed, cmd):
                self._set_phase('DOCK_RAMP')

        elif self.phase == 'DOCK_RAMP':
            if self._navigate_to_waypoint(self.ramp, self.max_speed, cmd):
                self._set_phase('DOCK')

        elif self.phase == 'DOCK':
            # Melaju tepat ke dermaga menghadap 3 Buoy Biru
            if self._navigate_to_waypoint(tuple(self.dock[:2]), self.approach_speed, cmd):
                self._set_phase('DOCK_ALIGN')

        elif self.phase == 'DOCK_ALIGN':
            yaw_err = _wrap_angle(self.dock[2] - self.pose[2])
            cmd.angular.z = _clamp(1.4 * yaw_err, -0.4, 0.4)
            cmd.linear.x = 0.0

            elapsed = time.monotonic() - self.capture_phase_start_time
            dock_verified = self.latest_dock_info.get('found', False)
            dock_count = self.latest_dock_info.get('count', 0)

            # Robot BERHENTI TOTAL di 3 Buoy Biru Dermaga:
            if abs(yaw_err) < 0.12 or elapsed >= 3.5:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self._set_phase('MISSION_COMPLETE')
                self._log_mission_event(
                    'MISSION_COMPLETE', 'dock',
                    f'Successfully passed all gates, blue & green boxes, and stopped at 3 dock blue buoys (count={dock_count}).'
                )
                self.get_logger().info(f'🏆 MISI SELESAI! Robot berhasil melewati seluruh 10 Gerbang, Kotak Biru & Hijau, dan BERHENTI di 3 Buoy Biru Dermaga (count={dock_count})!')

        elif self.phase == 'MISSION_COMPLETE':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self._update_status(self.phase)

    def _set_phase(self, new_phase):
        self.phase = new_phase
        self.capture_phase_start_time = time.monotonic()
        self.target_stable_count = 0
        if new_phase == 'GATE_APPROACH':
            self.sub_wp_index = 0
            self.gate_start_time = time.monotonic()
        self.get_logger().info(f"Navigasi Phase berganti -> {new_phase}")

    def _update_status(self, text):
        progress = f"{text} | Gate {min(self.gate_index + 1, len(self.gates))}/{len(self.gates)}"
        if progress != self.last_status_text:
            self.status_pub.publish(String(data=progress))
            self.last_status_text = progress

    def _publish_visualizations(self):
        # 1. Path (Diterbitkan dalam frame world - selaras 100% dengan Gazebo)
        path_msg = PathMsg()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for pt in self.path_world:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = pt[0]
            ps.pose.position.y = pt[1]
            ps.pose.position.z = 0.03
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        # 2. Markers (Diterbitkan dalam frame world - koordinat nyata Gazebo)
        marker_arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        m_id = 0

        # Scale proporsional sesuai radius buoy & ukuran objek baru
        buoy_scale = 0.16 if abs(self.start_world[0]) < 5.0 else 0.35
        box_w = 0.60 if abs(self.start_world[0]) < 5.0 else 1.2
        box_h = 0.45 if abs(self.start_world[0]) < 5.0 else 0.6
        dock_w = 0.80 if abs(self.start_world[0]) < 5.0 else 1.6
        dock_l = 0.70 if abs(self.start_world[0]) < 5.0 else 2.4

        def create_marker(ns, pt, color, scale, shape=Marker.SPHERE):
            nonlocal m_id
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = stamp
            m.ns = ns
            m.id = m_id
            m_id += 1
            m.type = shape
            m.action = Marker.ADD
            m.pose.position.x = pt[0]
            m.pose.position.y = pt[1]
            m.pose.position.z = 0.08 if shape == Marker.SPHERE else (0.225 if "box" in ns else 0.01)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = color
            return m

        for i, g in enumerate(self.gates):
            marker_arr.markers.append(create_marker(f"gate_{i+1}_red", g['red'], (1.0, 0.05, 0.05, 0.95), (buoy_scale, buoy_scale, buoy_scale)))
            marker_arr.markers.append(create_marker(f"gate_{i+1}_green", g['green'], (0.05, 1.0, 0.05, 0.95), (buoy_scale, buoy_scale, buoy_scale)))

        marker_arr.markers.append(create_marker("blue_box", self.blue_box, (0.05, 0.2, 0.9, 0.9), (box_w, box_h, 0.45), Marker.CUBE))
        marker_arr.markers.append(create_marker("green_box", self.green_box, (0.1, 0.9, 0.1, 0.9), (box_w, box_h, 0.45), Marker.CUBE))
        marker_arr.markers.append(create_marker("dock", tuple(self.dock[:2]), (0.9, 0.9, 0.9, 0.8), (dock_w, dock_l, 0.02), Marker.CUBE))

        self.marker_pub.publish(marker_arr)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
