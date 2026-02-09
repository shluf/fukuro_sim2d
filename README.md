# fukuro_sim2d

Simulator 2D untuk robot omniwheel berbasis ROS2 dan Pygame. Package ini menyediakan lingkungan simulasi untuk pengembangan dan testing strategi robot sepak bola.

## Deskripsi

fukuro_sim2d adalah simulator robotik 2D yang dirancang untuk mensimulasikan pergerakan robot omniwheel dalam pertandingan sepak bola robot. Simulator ini menggunakan Pygame untuk visualisasi dan terintegrasi penuh dengan ROS2 untuk komunikasi antar komponen.

### Fitur Utama

- Simulasi fisika robot omniwheel 3 roda dengan kinematika yang realistis
- Model fisika bola dengan friction, collision, dan dribbling
- Dua mode kompetisi: Regional dan Nasional
- Visualisasi real-time dengan Pygame
- Drag-and-drop untuk mengatur posisi bola dan obstacle
- Checkbox untuk enable/disable obstacle secara interaktif
- Integrasi lengkap dengan ROS2 topics dan services

## Mode Kompetisi

### Regional (8m x 6m)
- Lapangan setengah dengan gawang di sisi bawah
- 2 robot aktif (robot2 dan robot3)
- 9 obstacle statis yang dapat diatur posisinya
- Sistem koordinat: origin (0,0) di pojok kiri atas, X ke kanan, Y ke bawah

### Nasional (12m x 8m)
- Lapangan penuh dengan gawang di kiri dan kanan
- 3 robot tim sendiri (r1=keeper, r2=striker, r3=striker)
- 3 robot lawan
- Sistem koordinat: origin (0,0) di pojok kiri bawah, X ke kanan, Y ke atas

## Instalasi

### Prasyarat

- ROS2 (Humble atau lebih baru)
- Python 3.8+
- Pygame
- NumPy

### Build Package

```bash
cd ~/ros2_fukuro_strategy
colcon build --packages-select fukuro_sim2d
source install/setup.bash
```

### Install Dependencies

```bash
pip install pygame numpy
```

## Penggunaan

### Menjalankan Simulator

```bash
ros2 run fukuro_sim2d simulation
```

### Kontrol Interaktif

#### Mouse
- **Klik kiri + drag** pada bola: memindahkan posisi bola
- **Klik kiri + drag** pada obstacle: memindahkan obstacle
- **Klik kanan** pada obstacle: toggle enable/disable

#### Keyboard
- **M**: Toggle mode (Regional/Nasional)
- **R**: Reset simulasi
- **SPACE**: Toggle pause
- **ESC**: Keluar dari simulator

### ROS2 Topics

#### Published Topics

- `/world_state` (WorldState): State dunia simulasi termasuk posisi robot, bola, dan obstacle
- `/strategy_state` (StrategyState): State strategi untuk koordinasi robot

#### Subscribed Topics

Mode Regional (robot2 dan robot3):
- `/robot2/cmd_vel` (Twist): Perintah kecepatan untuk robot2
- `/robot3/cmd_vel` (Twist): Perintah kecepatan untuk robot3

Mode Nasional (robot1, robot2, robot3):
- `/robot1/cmd_vel` (Twist): Perintah kecepatan untuk keeper
- `/robot2/cmd_vel` (Twist): Perintah kecepatan untuk striker 1
- `/robot3/cmd_vel` (Twist): Perintah kecepatan untuk striker 2

Topik lainnya:
- `/strategy/change` (String): Perintah perubahan strategi (format JSON)

### ROS2 Services

- `/robot{N}/dribbler` (DribblerControl): Kontrol dribbler on/off
- `/robot{N}/kick` (KickService): Perintah kick dengan power
- `/robot{N}/set_ready` (SetReady): Set ready state robot

## Struktur Package

```
fukuro_sim2d/
├── fukuro_sim2d/
│   ├── __init__.py
│   ├── simulation_node.py          # Node ROS2 utama
│   ├── objects/
│   │   ├── __init__.py
│   │   └── field.py                # Konfigurasi lapangan
│   ├── physics/
│   │   ├── __init__.py
│   │   ├── ball_model.py           # Model fisika bola
│   │   ├── omni3_kinematics.py     # Kinematika omniwheel
│   │   └── robot_model.py          # Model fisika robot
│   └── render/
│       ├── __init__.py
│       ├── frame_converter.py      # Konversi koordinat
│       └── renderer.py             # Rendering Pygame
├── resource/
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Parameter Fisika

### Robot
- Radius: 0.1 m
- Massa: 5.0 kg
- Max velocity linear: 2.0 m/s
- Max velocity angular: 3.0 rad/s
- Wheel positions: 120 derajat spacing

### Bola
- Radius: 0.075 m
- Massa: 0.45 kg
- Friction coefficient: 0.15
- Restitution (pantul): 0.7

### Obstacle
- Ukuran default: 0.4 m x 0.4 m
- Dapat diaktifkan/nonaktifkan secara interaktif

## Contoh Penggunaan

### Menggerakkan Robot

```bash
# Gerakkan robot2 maju
ros2 topic pub /robot2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Mengaktifkan Dribbler

```bash
ros2 service call /robot2/dribbler fukuro_interface/srv/DribblerControl "{enable: true}"
```

### Melakukan Kick

```bash
ros2 service call /robot2/kick fukuro_interface/srv/KickService "{power: 100}"
```

### Mengganti Strategi

```bash
ros2 topic pub --once /strategy/change std_msgs/msg/String "data: '{\"mode\": \"kickoff_kanan\"}'"
```

## Koordinat World Frame

### Regional
- Origin (0, 0) berada di pojok kiri atas (samping gawang)
- Sumbu X mengarah ke kanan (0 - 8 m)
- Sumbu Y mengarah ke bawah (0 - 6 m)
- Gawang berada di Y = 6.0 m

### Nasional
- Origin (0, 0) berada di pojok kiri bawah
- Sumbu X mengarah ke kanan (0 - 12 m)
- Sumbu Y mengarah ke atas (0 - 8 m)
- Gawang kiri di X = 0, gawang kanan di X = 12 m

## Troubleshooting

### Pygame tidak terinstall
```bash
pip install pygame
```

### Topic tidak muncul
Pastikan ROS2 environment sudah di-source:
```bash
source ~/ros2_fukuro_strategy/install/setup.bash
```

### Simulator lag atau lambat
- Tutup aplikasi lain yang berat
- Kurangi jumlah obstacle aktif
- Pastikan driver grafis terinstall dengan benar

## Lisensi

MIT License

## Maintainer

- shluf (luthfisalis09@gmail.com)

## Dependencies

- rclpy
- geometry_msgs
- std_msgs
- fukuro_interface
- pygame
- numpy
