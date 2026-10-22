# Simulation-ASV-with-difrentialwheel
## 🤖 Autonomous Patrol Robot — ROS 2 Jazzy & Gazebo Harmonic

Dokumentasi komprehensif sistem robot patrol otonom (*differential drive surface vehicle*) menggunakan model SolidWorks asli pada environment **ROS 2 Jazzy Jalisco**, **Gazebo Harmonic (GZ Sim)**, dan visualisasi terintegrasi pada **RViz2**.

Sistem ini dirancang untuk menyelesaikan sirkuit kompetisi patroli otonom sesuai diagram resmi: meluncur dari dermaga, menembus 10 pasang gerbang buoy (merah-hijau), memotret dan melintasi Kotak Biru dan Kotak Hijau di jalur air terbuka yang aman (tanpa melindas/menginjak kotak), hingga kembali dan **berhenti total tepat di depan 3 Buoy Biru Dermaga**.

---

## 📌 Daftar Isi
1. [Arsitektur & Algoritma Navigasi](#-1-arsitektur--algoritma-navigasi)
2. [Sistem Pengolahan Citra & Image Capture](#-2-sistem-pengolahan-citra--image-capture)
3. [Struktur Folder & Penjelasan Berkas](#-3-struktur-folder--penjelasan-berkas)
4. [Tampilan Antarmuka RViz2](#-4-tampilan-antarmuka-rviz2)
5. [Panduan Menjalankan Simulasi](#-5-panduan-menjalankan-simulasi)
6. [Daftar Topik ROS 2 & Protokol Pesan](#-6-daftar-topik-ros-2--protokol-pesan)

---

## 🧭 1. Arsitektur & Algoritma Navigasi

Sistem navigasi dikendalikan oleh node otonom [`waypoint_patrol_node.py`](file:///home/pilja/Documents/UTSRE502/src/navigasi/scripts/waypoint_patrol_node.py) yang menerapkan metode **Hybrid Waypoint Guidance + Closed-Loop Visual Servoing**:

```
 ┌─────────────────────────┐      /odom (50 Hz)       ┌────────────────────────┐
 │   Gazebo Harmonic       │─────────────────────────►│                        │
 │   Physics Engine        │                          │  waypoint_patrol_node  │
 └─────────────────────────┘                          │  (Finite State Machine │
              ▲                                       │   + Visual Servoing    │
              │ /cmd_vel (Twist)                      │   + Path Tracking)     │
              └───────────────────────────────────────│                        │
                                                      └────────────────────────┘
                                                                   ▲
 ┌─────────────────────────┐   /vision/gate_info (JSON)            │
 │   camera_vision_node    │───────────────────────────────────────┘
 │   (HSV Color Filter     │   /vision/target_info (JSON)
 │    & Bounding Box)      │   /vision/dock_info (JSON)
 └─────────────────────────┘
```

### A. Algoritma Utama Navigasi:
1. **Hierarchical Finite State Machine (FSM):**
   Misi diatur melalui mesin status berurutan:
   - `DOCK_EXIT` $\to$ Meluncur lurus menuruni ramp dermaga ke arah Utara ($\text{yaw} = +90^\circ$).
   - `GATE_APPROACH` $\to$ Menuju titik pendekatan (*approach waypoint*) gerbang aktif.
   - `CROSS_GATE` $\to$ Menembus celah gerbang memadukan target waypoint dan visual servoing kamera.
   - `GATE_TURN` $\to$ Melakukan rotasi haluan aktif (*turn-in-place priority*) menuju gerbang berikutnya.
   - `BLUE_APPROACH` $\to$ Melaju ke jalur air di sisi luar Kotak Biru ($x = \text{sgn} \times 3.20\text{ m}$).
   - `CAPTURE_BLUE` $\to$ Membidik kamera ke Kotak Biru dari samping, menstabilkan, dan menyimpan foto PNG.
   - `BLUE_PASS` $\to$ Melaju di jalur air terbuka melewati Kotak Biru hingga $( \text{sgn} \times 3.10, -2.60 )$.
   - `CORNER_TURN` $\to$ Melengkung mulus mengitari sudut tenggara arena $( \text{sgn} \times 2.80, -3.00 )$.
   - `GREEN_APPROACH` $\to$ Melaju ke jalur air di sebelah utara Kotak Hijau ($y = -2.90\text{ m}$).
   - `CAPTURE_GREEN` $\to$ Membidik kamera ke Kotak Hijau dari samping, menstabilkan, dan menyimpan foto PNG.
   - `GREEN_PASS` $\to$ Melaju di jalur air terbuka melewati Kotak Hijau hingga $( \text{sgn} \times 0.80, -2.90 )$.
   - `DOCK_CORRIDOR` $\to$ Melaju menyusuri garis lurus bawah arena hingga $( \text{sgn} \times -1.50, -2.90 )$.
   - `DOCK_APPROACH` $\to$ Mengarah ke bibir dermaga di $( \text{sgn} \times -2.40, -2.90 )$.
   - `DOCK_RAMP` $\to$ Menaiki ramp tanjakan dermaga.
   - `DOCK` $\to$ Masuk ke platform dermaga tepat menghadap 3 Buoy Biru Dermaga.
   - `DOCK_ALIGN` $\to$ Menyelaraskan arah hadap robot dan **BERHENTI TOTAL** (`cmd_vel = 0`).
   - `MISSION_COMPLETE` $\to$ Seluruh misi selesai dan mencatat laporan resmi.

2. **Plane-Crossing Detection (Deteksi Lolos Gerbang Anti-Stuck):**
   Menggunakan proyeksi *dot-product* perpindahan robot terhadap vektor normal arah gerbang:
   $$\text{forward\_progress} = (x_{\text{robot}} - x_{\text{center}}) \cdot n_x + (y_{\text{robot}} - y_{\text{center}}) \cdot n_y$$
   Robot dinyatakan berhasil melewati gerbang ketika $\text{forward\_progress} \ge 0.75 \times d_{\text{exit}}$, mencegah robot macet jika melintasi gerbang sedikit di luar batas toleransi radius lingkaran.

3. **Continuous Velocity Profiling with Turn-in-Place Priority:**
   Kecepatan linier diatur berbanding lurus dengan keselarasan haluan ($\Delta \theta$):
   $$v_x = \min(v_{\text{max}}, 0.5 \cdot d) \times \max(0.0, \cos(\Delta \theta))^{2.5}$$
   $$\omega_z = \text{clamp}(1.8 \cdot \Delta \theta, -\omega_{\text{max}}, \omega_{\text{max}})$$
   Ketika robot menghadapi tikungan tajam, kecepatan linier otomatis melambat mendekati nol agar robot memutar haluan di tempat (*turn-in-place*), mengeliminasi pemotongan sudut (*anti corner-cutting*).

4. **Anti-Collision Lateral Shift & Extended Exit:**
   - **Gerbang 7 $\to$ 8:** Titik keluar Gerbang 7 diperpanjang menjadi $0.35\text{ m}$ ke depan ($x = 0.88\text{ m}$). Robot melaju lurus keluar sebelum memutar ke kanan, menjamin roda belakang bebas dari Buoy Merah 04.
   - **Gerbang 8 $\to$ 9:** Sistem mendeteksi pergeseran lateral $> 0.18\text{ m}$ dan menyisipkan titik transisi lateral ($p_{\text{shift}}$) sehingga robot bergeser ke timur terlebih dahulu sebelum menuju selatan, memberikan ruang bebas $50\text{ cm}$ dari Buoy Merah 09.

---

## 📷 2. Sistem Pengolahan Citra & Image Capture

Pengolahan visual ditangani oleh [`camera_vision_node.py`](file:///home/pilja/Documents/UTSRE502/src/robot_description/scripts/camera_vision_node.py) menggunakan pustaka **OpenCV (cv2)** dan **NumPy**:

### A. Pipeline Pengolahan Citra:
1. **Konversi Ruang Warna HSV:**
   Gambar BGR dari topik `/camera/image_raw` dikonversi ke ruang warna HSV (*Hue, Saturation, Value*) yang tahan terhadap variasi intensitas pencahayaan:
   - **Merah (Buoy Merah):** Menggabungkan 2 rentang hue melintasi batas $0^\circ/180^\circ$:
     - Rendah: $H \in [0, 10], S \in [100, 255], V \in [70, 255]$
     - Tinggi: $H \in [170, 180], S \in [100, 255], V \in [70, 255]$
   - **Hijau (Buoy Hijau & Kotak Hijau):** $H \in [35, 85], S \in [80, 255], V \in [50, 255]$
   - **Biru (Kotak Biru & 3 Buoy Biru Dermaga):** $H \in [95, 130], S \in [80, 255], V \in [50, 255]$

2. **Pembersihan Morfologi Biner:**
   Menerapkan operasi *Morphological Opening* ($3\times3$ kernel) untuk membuang bintik *noise*, dilanjutkan *Morphological Closing* untuk menutup lubang kecil pada objek.

3. **Ekstraksi Kontur & Bounding Box (BBox):**
   - Mendeteksi kontur dengan `cv2.findContours(cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)`.
   - Menghitung koordinat tengah kontur menggunakan momen geometri:
     $$c_x = \frac{M_{10}}{M_{00}}, \quad c_y = \frac{M_{01}}{M_{00}}$$
   - Menggambar BBox persegi panjang berwarna, crosshair titik tengah, dan garis panduan kemudi.

4. **Validasi Pasangan Gerbang Sah (*Gate Pairing Heuristics*):**
   - Aturan arah: Bola Hijau harus berada di kiri dan Bola Merah di kanan kamera ($x_{\text{red}} > x_{\text{green}}$).
   - Rasio ukuran: $\max(\text{area}_r/\text{area}_g, \text{area}_g/\text{area}_r) \le 3.0$ (menolak pasangan bola silang antar gerbang berbeda).
   - Menghitung deviasi error celah terhadap garis tengah kamera:
     $$\text{error}_x = \frac{c_x - (\text{width} / 2)}{\text{width} / 2}$$

### B. Mekanisme Capture & Logging:
1. **Pemicu Otomatis (*Auto-Trigger Capture*):**
   Saat berada di fase `CAPTURE_BLUE` atau `CAPTURE_GREEN`, sistem mengunci target di layar kamera. Setelah target stabil selama minimal 4 frame berturut-turut (atau batas waktu 2.0 detik), robot langsung menyimpan citra kamera raw (tanpa overlay) dengan kompresi kualitas tinggi.
2. **Format & Lokasi Penyimpanan:**
   - Folder Target Biru: [`captures/blue/target_blue_YYYYMMDD_HHMMSS_fff.png`](file:///home/pilja/Documents/UTSRE502/captures/blue/)
   - Folder Target Hijau: [`captures/green/target_green_YYYYMMDD_HHMMSS_fff.png`](file:///home/pilja/Documents/UTSRE502/captures/green/)
3. **Log Pelaporan Misi Resmi:**
   - **Laporan Teks:** [`captures/gate_passed_report.txt`](file:///home/pilja/Documents/UTSRE502/captures/gate_passed_report.txt) mencatat nomor gerbang, waktu selesai, durasi lintasan, dan koordinat exit.
   - **Laporan CSV:** [`captures/mission_log.csv`](file:///home/pilja/Documents/UTSRE502/captures/mission_log.csv) mencatat seluruh event misi (`GATE_PASSED`, `TARGET_CAPTURED`, `MISSION_COMPLETE`).

---

## 📁 3. Struktur Folder & Penjelasan Berkas

```text
UTSRE502/
├── README.md                                    # Dokumentasi utama proyek & panduan pengoperasian
├── kill_sim.sh                                  # Skrip pembersih proses Gazebo, ROS 2, dan RViz2
├── captures/                                    # Direktori penyimpanan hasil foto & laporan misi
│   ├── blue/                                    # Arsip citra foto Kotak Biru (*.png)
│   ├── green/                                   # Arsip citra foto Kotak Hijau (*.png)
│   ├── gate_passed_report.txt                   # Laporan resmi kelulusan gerbang tabular
│   └── mission_log.csv                          # Catatan log peristiwa misi berbasis timestamp
│
├── src/
│   ├── robot_description/                       # [PACKAGE 1] Model Fisik Robot & Pemrosesan Kamera
│   │   ├── CMakeLists.txt                       # Build file CMake
│   │   ├── package.xml                          # Definisi dependensi package ROS 2
│   │   ├── meshes/                              # Berkas 3D mesh SolidWorks asli
│   │   │   ├── base.STL                         # 3D mesh sasis bodi robot
│   │   │   ├── l_wheel.STL                      # 3D mesh roda kiri
│   │   │   ├── r_wheel.STL                      # 3D mesh roda kanan
│   │   │   └── imu.STL                          # 3D mesh dudukan sensor IMU
│   │   ├── urdf/
│   │   │   ├── robot_patrol.urdf.xacro          # Model URDF robot: differential drive, caster ball,
│   │   │   │                                    # sensor kamera RGB 90° FOV, dan IMU
│   │   │   ├── rb.SLDASM.urdf                   # Model referensi rakitan SolidWorks
│   │   │   └── rb.SLDASM.csv                    # Data joint dan koordinat inersia
│   │   └── scripts/
│   │       └── camera_vision_node.py            # Node pengolahan citra HSV, BBox buoy & kotak target,
│   │                                            # visual servoing error, dan PiP display overlay
│   │
│   ├── navigasi/                                # [PACKAGE 2] Kendali Navigasi Otonom
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── scripts/
│   │       └── waypoint_patrol_node.py          # State machine patroli 10 gerbang, anti-tabrak,
│   │                                            # pemotretan target, navigasi koridor, dan docking
│   │
│   └── bringup/                                 # [PACKAGE 3] Konfigurasi, Peluncur, & Lingkungan Simulasi
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   └── simulation.launch.py             # Launch file master: menjalankan Gazebo Harmonic,
│       │                                        # ROS-GZ Bridge, Robot State Publisher, Vision,
│       │                                        # Waypoint Patrol, dan RViz2
│       ├── config/
│       │   ├── waypoints_right_8x8.yaml         # Parameter koordinat buoy, target, & dock Arena Kanan
│       │   ├── waypoints_left_8x8.yaml          # Parameter koordinat buoy, target, & dock Arena Kiri
│       │   ├── waypoints_right.yaml             # Konfigurasi arena kanan skala 30x30 m
│       │   └── waypoints_left.yaml              # Konfigurasi arena kiri skala 30x30 m
│       ├── worlds/
│       │   ├── patrol_right_ground_8x8.sdf      # Model dunia Gazebo Arena Kanan 8x8 m (SDF resmi)
│       │   ├── patrol_left_ground_8x8.sdf       # Model dunia Gazebo Arena Kiri 8x8 m (SDF resmi)
│       │   ├── patrol_right_ground.sdf          # Model dunia Gazebo arena kanan 30x30 m
│       │   └── patrol_left_ground.sdf           # Model dunia Gazebo arena kiri 30x30 m
│       └── rviz/
│           └── simulation_debug.rviz            # Konfigurasi tampilan RViz2 (3D World + Vision Debug)
```

---

## 🖥️ 4. Tampilan Antarmuka RViz2

Konfigurasi RViz2 ([`simulation_debug.rviz`](file:///home/pilja/Documents/UTSRE502/src/bringup/rviz/simulation_debug.rviz)) menyatukan seluruh visualisasi 3D sirkuit dan display kamera dalam satu antarmuka yang bersih dan lega:

```
┌───────────────────────────────────────────────────────────┬───────────────────────────────┐
│                    RViz2 3D Viewport                      │         Vision Debug          │
│                                                           │     (/vision/debug_image)     │
│  - SolidWorks RobotModel : Model 3D robot bergerak live  │                               │
│  - Odometry Trail        : Garis jejak odometri (/odom)   │  [CAMERA POV - Sudut Pandang  │
│  - Waypoint Path (Cyan)  : Rute lengkap sirkuit resmi     │   Kamera Robot + Bounding     │
│  - Gates & Target Boxes  : Model bola gerbang & kotak 3D  │   Box Deteksi Gerbang/Target] │
│  - Arena Grid            : Garis grid lantai 8x8 m        │                               │
│  - TF Frames             : Orientasi koordinat sumbu      │  ┌────────┬────────┬────────┐ │
│                                                           │  │ RED    │ GREEN  │ BLUE   │ │
│                                                           │  │ MASK   │ MASK   │ MASK   │ │
│                                                           │  └────────┴────────┴────────┘ │
└───────────────────────────────────────────────────────────┴───────────────────────────────┘
```

1. **Jendela 3D Utama:**
   - Menampilkan pergerakan robot nyata, jejak odometri, garis rute waypoint (cyan), gerbang buoy 3D, dan target kotak.
2. **Jendela Tunggal `Vision Debug` (`/vision/debug_image`):**
   - Merangkum Camera POV asli dengan anotasi deteksi.
   - Dilengkapi panel *Picture-in-Picture* (PiP) mini di pojok kanan atas untuk **Mask Merah**, **Mask Hijau**, dan **Mask Biru**.
   - Menghilangkan 5 jendela gambar terpisah yang sebelumnya memadati layar RViz.

---

## 🚀 5. Panduan Menjalankan Simulasi

### A. Kompilasi Workspace
Pastikan environment ROS 2 Jazzy telah disumber, lalu kompilasi workspace:
```bash
cd ~/Documents/UTSRE502
colcon build --symlink-install
source install/setup.bash
```

### B. Menjalankan Arena Kanan (`arena:=right` - Standar Default)
```bash
./kill_sim.sh && source install/setup.bash && ros2 launch bringup simulation.launch.py arena:=right
```

### C. Menjalankan Arena Kiri (`arena:=left`)
```bash
./kill_sim.sh && source install/setup.bash && ros2 launch bringup simulation.launch.py arena:=left
```

### D. Menghentikan Simulasi Bersih
Gunakan skrip pembunuh proses otomatis:
```bash
./kill_sim.sh
```

---

## 📡 6. Daftar Topik ROS 2 & Protokol Pesan

| Nama Topik | Tipe Pesan | Deskripsi |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Citra kamera raw dari sensor simulasi Gazebo (30 Hz) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Perintah kecepatan linier dan sudut kemudi robot |
| `/odom` | `nav_msgs/msg/Odometry` | Data odometri pergerakan robot dari Gazebo (50 Hz) |
| `/vision/debug_image` | `sensor_msgs/msg/Image` | Gambar BGR8 dengan anotasi deteksi dan PiP mask 3 warna |
| `/vision/gate_info` | `std_msgs/msg/String` (JSON) | Koordinat pixel, error horizontal, dan status deteksi gerbang |
| `/vision/target_info` | `std_msgs/msg/String` (JSON) | Status deteksi dan bounding box Kotak Hijau & Kotak Biru |
| `/vision/dock_info` | `std_msgs/msg/String` (JSON) | Deteksi jumlah dan posisi 3 Buoy Biru Dermaga |
| `/mission/path` | `nav_msgs/msg/Path` | Garis lintasan rute waypoint sirkuit resmi (warna cyan) |
| `/mission/markers` | `visualization_msgs/msg/MarkerArray` | Model 3D gerbang buoy, kotak target, dan dermaga di RViz2 |
| `/mission/status` | `std_msgs/msg/String` | Status fase aktif dari FSM misi patroli |
