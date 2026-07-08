# fukuro_sim2d

Simulator 2D robot omniwheel berbasis ROS 2 + Pygame untuk menguji strategi Fukuro. Simulator dapat berjalan dalam dua mode komunikasi:

1. **Standalone** — hanya ROS 2 topic/service internal, tanpa Base Station.
2. **Protobuf/Base Station** — membuka 3 TCP port dan bertindak seperti 3 robot fisik untuk `fukuro_base_station`.

## Fitur

- Simulasi fisika robot omniwheel 3 roda dengan limit kecepatan dan akselerasi.
- Simulasi bola dengan friction, collision, kick, dan dribbler/grip.
- Mode lapangan `regional` dan `nasional`.
- Dynamic Role Reassignment sederhana di simulator (`R1_STRIKER`, `R2_SUPPORTER`, `R3_DEFENDER`, dll.).
- Publikasi `WorldState` per robot:
  - `/r1/fukuro/world_model/state`
  - `/r2/fukuro/world_model/state`
  - `/r3/fukuro/world_model/state`
- Bridge TCP/Protobuf opsional untuk Base Station:
  - `r1 → 8081`
  - `r2 → 8082`
  - `r3 → 8083`

## Build

```bash
cd ~/ros2_fukuro
colcon build --packages-select fukuro_sim2d fukuro_sim2d_planner fukuro_behavior_tree fukuro_pathplanning
source install/setup.bash
```

Dependency Python:

```bash
pip install pygame numpy protobuf
```

## Mode 1 — Standalone

Gunakan mode ini jika ingin menjalankan simulator + behavior tree tanpa `fukuro_base_station` dan tanpa RefBox.

### Jalankan semua node via script

```bash
cd ~/ros2_fukuro
source install/setup.bash
./assets/scripts/run_base_station_bridge.sh sim \
  --sim-mode nasional \
  --sim-comms standalone
```

Mode ini menjalankan:

- `fukuro_pathplanning/navigator_node`
- `fukuro_behavior_tree/strategy_node`
- `fukuro_sim2d_planner/global_planner_node`
- `fukuro_sim2d_planner/local_controller_node`
- `fukuro_sim2d/simulation`

TCP protobuf ke Base Station **tidak dibuka**.

### Mulai behavior tree manual

Karena tidak ada RefBox/Base Station, ubah mode BT lewat ROS service:

```bash
ros2 service call /fukuro/strategy/change fukuro_interface/srv/StrategyChange "{new_strategy: 'playing'}"
```

Stop:

```bash
ros2 service call /fukuro/strategy/change fukuro_interface/srv/StrategyChange "{new_strategy: 'stop'}"
```

Homing:

```bash
ros2 service call /fukuro/strategy/homing std_srvs/srv/SetBool "{data: true}"
```

### Jalankan simulator saja standalone

```bash
ros2 run fukuro_sim2d simulation --ros-args \
  -p sim_mode:=nasional \
  -p sim_comms_mode:=standalone \
  -p base_station_bridge.enabled:=false
```

## Mode 2 — Protobuf / Base Station / RefBox

Gunakan mode ini jika ingin `fukuro_base_station` menerima RefBox command lalu mengirim `Protobuf_From_BS` ke simulator seperti ke robot fisik.

### Jalankan ROS simulator stack

```bash
cd ~/ros2_fukuro
source install/setup.bash
./assets/scripts/run_base_station_bridge.sh sim \
  --sim-mode nasional \
  --sim-comms protobuf
```

Simulator akan membuka:

| Robot | Port | Arah ke Base Station |
|---|---:|---|
| `r1` | `8081` | `Protobuf_From_ROS` keluar, `Protobuf_From_BS` masuk |
| `r2` | `8082` | `Protobuf_From_ROS` keluar, `Protobuf_From_BS` masuk |
| `r3` | `8083` | `Protobuf_From_ROS` keluar, `Protobuf_From_BS` masuk |

Cek port:

```bash
ss -ltnp | grep -E '8081|8082|8083'
```

### Hubungkan Base Station

Di `fukuro_base_station`, tambahkan/koneksikan robot ke host simulator:

Jika Base Station satu PC dengan simulator:

```text
r1: 127.0.0.1:8081
r2: 127.0.0.1:8082
r3: 127.0.0.1:8083
```

Jika beda PC, gunakan IP PC simulator, contoh:

```text
r1: 192.168.1.12:8081
r2: 192.168.1.12:8082
r3: 192.168.1.12:8083
```

Cek koneksi established:

```bash
ss -tnp state established | grep -E '8081|8082|8083'
```

### Flow command RefBox

1. RefBox mengirim JSON command ke Base Station (`28097`).
2. Base Station menerjemahkan command ke `Protobuf_From_BS`.
3. Simulator menerima packet di port `8081/8082/8083`.
4. Simulator publish `/rX/fukuro/comms/base_station` dan update `WorldState`.
5. Behavior tree membaca `WorldState.dynamic_role`, `active_control`, `restart_type`, dll.

Contoh cek packet Base Station yang diterima simulator:

```bash
ros2 topic echo /r2/fukuro/comms/base_station --once
```

Untuk `START`, nilai penting:

```yaml
active_control: 2
```

Untuk kickoff restart, nilai penting:

```yaml
restart_type: 1
restart_for_us: true/false
```

## Parameter penting

| Parameter | Nilai | Default | Keterangan |
|---|---|---|---|
| `sim_mode` | `regional`, `nasional` | `regional` | Mode lapangan |
| `sim_comms_mode` | `standalone`, `protobuf` | `protobuf` | Mode komunikasi simulator |
| `base_station_bridge.enabled` | `true`, `false` | `true` | Backward-compatible switch TCP bridge |
| `base_station_bridge.port_r1` | integer | `8081` | Port robot r1 |
| `base_station_bridge.port_r2` | integer | `8082` | Port robot r2 |
| `base_station_bridge.port_r3` | integer | `8083` | Port robot r3 |

## Topic utama

Published:

```text
/r1/fukuro/world_model/state
/r2/fukuro/world_model/state
/r3/fukuro/world_model/state
/r1/fukuro/comms/base_station   # hanya menerima data setelah Base Station kirim packet
/r2/fukuro/comms/base_station
/r3/fukuro/comms/base_station
```

Subscribed:

```text
/r1/cmd_vel
/r2/cmd_vel
/r3/cmd_vel
/r1/fukuro/strategy/goal
/r2/fukuro/strategy/goal
/r3/fukuro/strategy/goal
```

Services per robot:

```text
/r1/fukuro/controller/dribbler
/r2/fukuro/controller/dribbler
/r3/fukuro/controller/dribbler
/r1/fukuro/controller/kick
/r2/fukuro/controller/kick
/r3/fukuro/controller/kick
/r1/fukuro/strategy/set_ready
/r2/fukuro/strategy/set_ready
/r3/fukuro/strategy/set_ready
```

## Troubleshooting

### Base Station command masuk tapi simulator tidak berubah

Pastikan Base Station terkoneksi ke `8081/8082/8083`:

```bash
ss -tnp state established | grep -E '8081|8082|8083'
```

Jika tidak ada koneksi, command hanya berhenti di UI Base Station.

### `/r2/fukuro/comms/base_station` kosong

Topic ini hanya berisi **data yang dikirim Base Station ke simulator**. Jika `friendly_robots`, `enemy_robots`, atau `active_obstacles` kosong, berarti Base Station belum mengisi field tersebut di `Protobuf_From_BS` atau belum menerima telemetry cukup dari robot lain.

### Behavior tree tetap idle di standalone

Panggil:

```bash
ros2 service call /fukuro/strategy/change fukuro_interface/srv/StrategyChange "{new_strategy: 'playing'}"
```

### Pygame tidak muncul

Pastikan environment desktop tersedia. Untuk smoke test headless:

```bash
SDL_VIDEODRIVER=dummy ros2 run fukuro_sim2d simulation --ros-args -p sim_mode:=nasional
```
