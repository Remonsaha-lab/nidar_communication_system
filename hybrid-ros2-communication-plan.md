# Hybrid ROS 2 Communication Plan: Drone (RPi 4) to Linux Ground Station
## Approach 3: Hybrid Architecture – Physically Decoupled Control & Prioritized Data Planes

---

## 1. System Overview & Architectural Decision Record

This document defines the end-to-end communication architecture between an **onboard drone companion computer (Raspberry Pi 4)** and a **Linux-based Ground Control Station (GCS) laptop**, implementing **Approach 3: Hybrid Architecture (Best of Both Approaches)**.

### Architectural Core: Physically Decoupled Planes
The system physically isolates the **flight-safety control plane** from the **high-bandwidth autonomy and vision data plane**:
1. **Control & Failsafe Plane (433 MHz MAVLink - Always Alive)**:
   - Direct hardware connection: **Pixhawk TELEM1** ↔ **YRRC 433 MHz Air Radio** ))) ((( **YRRC 433 MHz Ground Radio** ↔ **Linux Laptop USB** (`/dev/ttyUSB0`) running **QGroundControl**.
   - Completely independent of the Raspberry Pi 4 operating system, Wi-Fi link, or ROS 2 middleware.
   - Even if the RPi 4 crashes, overheats, or loses power, manual control, live telemetry, and emergency failsafes (RTL, Disarm, Abort) remain 100% operational.
2. **Autonomy, Vision & Data Plane (RTL8812AU Wi-Fi - Prioritized QoS)**:
   - Connects **Raspberry Pi 4** (USB 3.0 RTL8812AU) ↔ **Linux Laptop** via 5 GHz / 2.4 GHz Wi-Fi.
   - Carries ROS 2 Humble DDS topics (`rmw_cyclonedds_cpp`), low-latency H.264 video (GStreamer UDP 5600), occupancy grid maps, and AI survivor detection events.
3. **Companion Computer Bridge (Pixhawk ↔ RPi 4)**:
   - Pixhawk TELEM2 port connects directly to RPi 4 hardware UART (GPIO Pins 8/10/6 -> `/dev/ttyAMA0` @ 921600 baud) or via USB CDC-ACM (`/dev/ttyACM0`).
   - Runs **Micro-XRCE-DDS Agent** (PX4) or **MAVROS 2** (ArduPilot) onboard the RPi 4.
   - **No `mavlink-routerd` is required**, eliminating unnecessary multiplexing overhead and avoiding single-point-of-failure vulnerabilities.

### Core Hardware Components
- **Onboard Companion Computer**: Raspberry Pi 4 Model B (Broadcom BCM2711, Quad-core Cortex-A72 @ 1.5/1.8 GHz, 4GB/8GB RAM, `arm64` architecture, running Ubuntu 22.04 LTS Server).
- **Flight Controller Unit (FCU)**: Pixhawk (PX4 Autopilot v1.14+ or ArduPilot Copter 4.4+).
- **Control & Telemetry Link**: YRRC 433 MHz SiK-compatible telemetry radio pair (Air radio wired to Pixhawk `TELEM1`, Ground radio connected to GCS laptop via USB).
- **High-Bandwidth Data & Video Link**: RTL8812AU high-power dual-band (2.4 GHz / 5 GHz) USB 3.0 Wi-Fi adapter.
- **Camera**: Raspberry Pi Camera Module v2/v3 (CSI ribbon) or standard UVC USB webcam.
- **Robotics Middleware**: ROS 2 Humble Hawksbill (`rmw_cyclonedds_cpp` with unicast peer discovery).
- **Ground Control Station (GCS)**: Linux Laptop (x86_64 architecture, running Ubuntu 22.04 LTS Desktop) running QGroundControl, RViz2, and Custom Mission Dashboard.

---

## 2. Approach 3: Hybrid System Architecture

```text
                               433 MHz Independent MAVLink Control Link
                ┌────────────────────────────────────────────────────────────────────────┐
                │                                                                        │
  Flight Controller (FCU)                                                          Ground Station
[Pixhawk Autopilot]                                                               ((( Ground 433 Radio ── USB ── Linux Laptop
  [TELEM1 Port] ── UART (57600) ── YRRC 433 Air Radio )))                                                 │    (QGroundControl)
        │                                                                                                 │    [MAVLink Only]
        │ UART (TELEM2: 921600 baud / Pins 8,10,6)                                                        │
        ▼                                                                                                 │
  Raspberry Pi 4 (Drone Companion)                                                                        │
   • ROS 2 Humble Autonomy Stack                                                                          │
   • Micro-XRCE-DDS Agent / MAVROS 2                                                                      │
   • Hardware H.264 Encoder (GStreamer)                                                                   │
   • Adaptive Publisher (Link-Quality Aware)                                                              │
   • Mission State Machine (Normal / Degraded / Autonomous)                                               │
   [USB 3.0] ── RTL8812AU Wi-Fi ──────────── 5 GHz / 2.4 GHz Wi-Fi Data Link ─────────────────────────────┴── RTL8812AU / Wi-Fi
                                              • Priority-Tagged ROS 2 Topics (P0–P5)                           (Linux Laptop)
                                              • Map Deltas (RLE Compressed)                                    • ROS 2 Humble
                                              • Survivor Detection Events (JSON/Msg)                           • RViz2 / Dashboard
                                              • Low-Latency H.264 Video (UDP 5600)                             • YOLOv8 / Video Recv
```

### 2.1 Architectural Pillars of Approach 3

| Feature | Approach 1 (Single Wi-Fi) | Approach 2 (Decoupled Planes) | Approach 3 (Hybrid - Selected) |
| :--- | :--- | :--- | :--- |
| **Safety Link** | Vulnerable (Over Wi-Fi) | Independent (433 MHz to FCU) | **Independent (433 MHz directly to Pixhawk FCU)** |
| **High-Bandwidth Link** | Single Wi-Fi | 2.4 GHz Wi-Fi | **Dual-Band 2.4 GHz / 5 GHz Wi-Fi (RTL8812AU)** |
| **Single Point of Failure** | RPi 4 / Wi-Fi crash kills drone | None for flight safety | **None for flight safety (C2 link fully isolated)** |
| **Data Plane Protocols** | Plain ROS 2 topics | Raw Zenoh bridge | **Priority-Tagged Topics (P0–P5) + Map Deltas + Adaptive QoS** |
| **Link Degradation Behavior** | Total link freeze / timeout | Manual pilot takeover | **Graceful degradation: local caching, adaptive throttling, auto-resync** |

### 2.2 Key Design Principles (Approach 3)
1. **Safety First**: The Command & Control (C2) link is physically isolated from the data link. No companion software failure can compromise autopilot flight safety.
2. **Graceful Degradation**: When Wi-Fi packet loss climbs, the companion computer automatically throttles debug topics and video bitrate, while critical survivor events and map deltas retain bandwidth. If Wi-Fi disconnects entirely, the drone continues autonomous execution locally while caching video to disk.
3. **Bandwidth Awareness**: Send what matters most, when needed. Initial maps are sent as full keyframes; subsequent updates travel as lightweight run-length encoded (RLE) deltas.
4. **RF Reality**: 433 MHz penetrates obstacles, vegetation, and concrete far better than 2.4/5 GHz Wi-Fi, guaranteeing non-line-of-sight (NLOS) emergency failsafe connectivity.
5. **Test for Failure**: System tests explicitly include unseating the Wi-Fi dongle and killing RPi 4 processes to verify instantaneous autopilot failsafe and GCS 433 MHz command continuity.
6. **Log Everything**: All flight telemetry, detection bounding boxes, and system performance metrics are recorded locally to NVMe/microSD for post-mission reconstruction.

---

## 3. Hardware Interfacing & Role Allocation

### 3.1 Onboard Vehicle: Raspberry Pi 4 & Pixhawk FCU

#### Hardware Setup & Wiring Connections
```text
           +-------------------------------------------------------------+
           |                    4S / 6S LiPo Battery                     |
           +------------------------------┬------------------------------+
                                          │
                                          ▼
                           +------------------------------+
                           |    Power Distribution Board   |
                           +-------┬--------------┬-------+
                                   │              │
                   Main Battery V  │              │ Main Battery V
                                   ▼              ▼
                 +--------------------+        +--------------------+
                 |  FCU Power Module  |        |  5V 4A Step-Down   |
                 |  (5V 2A Clean)     |        |  UBEC              |
                 +---------┬----------+        +---------┬----------+
                           │                             │ 5V & GND
                           ▼                             ▼
                 +--------------------+        +--------------------+
                 |  Flight Controller |        |   Raspberry Pi 4   |
                 |      (Pixhawk)     |        |  (Pin 2/4 5V, 6 G) |
                 +---------┬----------+        +---------┬----------+
                           │                             │
          [TELEM1 Port]    │                             │ [GPIO Header]
          Pin 1: 5V ───────┤                             │ Pin 8:  GPIO14 (TX) ──┐
          Pin 2: TXD ──────┼──┐                          │ Pin 10: GPIO15 (RX) ──┼──┐
          Pin 3: RXD ──────┼┐ │                          │ Pin 6:  GND ──────────┼─┐│
          Pin 6: GND ──────┼┼─┼─┐                        +---------┬----------+  │ ││
                           ││ │ │                                  │ [USB 3.0]   │ ││
                           ▼▼ ▼ ▼                                  ▼             │ ││
                 +--------------------+                  +--------------------+  │ ││
                 | YRRC 433 Air Radio |                  | RTL8812AU Wi-Fi    |  │ ││
                 | (SiK Transceiver)  |                  | High-Gain Adapter  |  │ ││
                 +--------------------+                  +--------------------+  │ ││
                                                                                 │ ││
          [TELEM2 Port]                                                          │ ││
          Pin 2: TXD ────────────────────────────────────────────────────────────┼─┘│
          Pin 3: RXD ────────────────────────────────────────────────────────────┼──┘
          Pin 6: GND ────────────────────────────────────────────────────────────┴───
          *WARNING: LEAVE PIN 1 (5V) OF TELEM2 DISCONNECTED!
```

- **Pixhawk TELEM1 ↔ YRRC 433 MHz Air Radio**:
  - Connect using the standard 6-pin JST-GH to 6-pin DF13/JST cable:
    - Pin 1 (VCC): 5V power from Pixhawk
    - Pin 2 (TX): Connected to Radio RX
    - Pin 3 (RX): Connected to Radio TX
    - Pin 6 (GND): Ground
  - Pixhawk supplies clean 5V power to the air radio (~100–250 mA).
- **Pixhawk TELEM2 ↔ Raspberry Pi 4 UART**:
  - **RPi Pin 8 (GPIO 14 / TXD)** → Pixhawk TELEM2 Pin 3 (RXD)
  - **RPi Pin 10 (GPIO 15 / RXD)** → Pixhawk TELEM2 Pin 2 (TXD)
  - **RPi Pin 6 (GND)** → Pixhawk TELEM2 Pin 6 (Common Ground)
  - **CRITICAL**: Do **NOT** connect TELEM2 Pin 1 (5V) to the RPi 4. Both devices have independent power sources; connecting 5V rails will damage the FCU or companion board.
  - *USB Alternative*: You can alternatively connect Pixhawk's USB-C/Micro-USB port directly to an RPi 4 USB 2.0 port (`/dev/ttyACM0`). Both methods provide reliable high-speed serial.
- **RTL8812AU Wi-Fi Adapter**:
  - Connect to an RPi 4 **USB 3.0 port** (blue) for maximum throughput.
- **Camera Module**:
  - Connect via dedicated CSI ribbon cable (Raspberry Pi Camera v2/v3 / HQ Camera) or use an available USB port for a UVC webcam.
- **Power Delivery**:
  - The RPi 4 and RTL8812AU draw up to 3.0A peak during transmission. Power the RPi 4 exclusively via a dedicated **5V 4A UBEC**.
- **Antenna Separation**:
  - Keep the YRRC 433 MHz dipole/monopole antenna and the RTL8812AU Wi-Fi antennas separated by at least **15–20 cm** on the drone arms to prevent RF harmonic desensitization.

#### Raspberry Pi 4 OS & Hardware UART Configuration
1. Flash **Ubuntu 22.04 LTS 64-bit Server (arm64)**.
2. Enable hardware PL011 UART and disable the Linux serial kernel console:
   Edit `/boot/firmware/cmdline.txt`:
   ```bash
   # Remove any reference to 'console=serial0,115200' or 'console=ttyAMA0,115200'
   # Keep 'console=tty1' so HDMI output remains functional
   ```
   Edit `/boot/firmware/config.txt`:
   ```ini
   enable_uart=1
   dtoverlay=disable-bt
   ```
   *Disabling Bluetooth routes the dedicated hardware PL011 UART directly to `/dev/ttyAMA0` (also aliased as `/dev/serial0`).*

3. Install the RTL8812AU Linux kernel driver on the RPi 4:
   ```bash
   sudo apt update
   sudo apt install -y build-essential dkms git bc linux-headers-$(uname -r)
   git clone -b v5.6.4.2 https://github.com/aircrack-ng/rtl8812au.git
   cd rtl8812au
   sudo make dkms_install
   sudo modprobe 8812au
   ```

#### RPi 4 Compute Budget & Node Responsibilities
The quad-core Cortex-A72 CPU budget is intentionally partitioned to prevent overheating and frame drops:

| Node / Process | Function | CPU Target |
| :--- | :--- | :--- |
| `fcu_bridge` | Micro-XRCE-DDS Agent (PX4) or MAVROS 2 (ArduPilot) via `/dev/ttyAMA0` @ 921600 baud | 5–8% |
| `rpicam-vid` / GStreamer | Hardware H.264 video compression & RTP packetization | 15–20% |
| `telemetry_aggregator` | Ingests vehicle state, battery, GPS; formats ROS 2 messages | < 5% |
| `adaptive_publisher` | Monitored link quality; throttles topic rates and map resolution dynamically | < 3% |
| `mission_state_machine` | Coordinates states: `NORMAL` ↔ `DEGRADED` ↔ `AUTONOMOUS` | < 2% |
| `local_planner_avoidance` | 2D lidar / sonar obstacle avoidance & trajectory generation | 10–15% |
| **Total RPi 4 Load** | **Leaves ~45–50% headroom for thermal stability and peak loads** | **~50–55%** |

---

### 3.2 Ground Control Station: Linux Laptop

#### Hardware Setup
- Architecture: x86_64, running **Ubuntu 22.04 LTS Desktop**.
- **YRRC 433 MHz Ground Radio**: Connected via USB (`/dev/ttyUSB0`).
- **Network Interface**: RTL8812AU high-gain dual-band adapter or internal Wi-Fi card.

#### Ground Station Software Stack
The Linux laptop handles all heavy visual, AI, and spatial mapping workloads:

| Application / Node | Function | Interface / Link |
| :--- | :--- | :--- |
| **QGroundControl (QGC)** | Primary flight instrument: telemetry, battery, attitude, emergency RTL, disarm | 433 MHz Serial (`/dev/ttyUSB0` @ 57600 baud) |
| **GStreamer Receiver** | Low-latency H.264 video decoder & virtual camera pipeline | Wi-Fi UDP Port 5600 |
| **YOLOv8 Detection Node** | Real-time survivor & object detection executed on Laptop GPU (CUDA) | Video feed from GStreamer |
| **SLAM & Map Builder** | Heavy 2D/3D SLAM (RTAB-Map / LIO-SAM), generates occupancy grid | ROS 2 DDS over Wi-Fi |
| **Custom Mission Dashboard** | Integrates map viewer, survivor alerts, mission status, video overlay | ROS 2 DDS / Web GUI |
| **Rosbag2 Logger** | High-speed logging of all mission topics to NVMe storage | ROS 2 DDS |

---

## 4. Network Configuration & ROS 2 DDS Setup

### 4.1 Field Wi-Fi Network Architecture
Deploy a dedicated field network using a static IP configuration to avoid DHCP handshake delays:
- **Subnet**: `192.168.50.0/24`
- **RPi 4 (Drone)**: Static IP `192.168.50.10`
- **Linux Laptop (GCS)**: Static IP `192.168.50.20`

#### Configuration Option A: Raspberry Pi 4 as 5 GHz Access Point
On the RPi 4:
```bash
sudo nmcli con add type wifi ifname wlan0 con-name DroneHotspot autoconnect yes ssid NIDAR_DRONE_NET mode ap
sudo nmcli con modify DroneHotspot 802-11-wireless.band a
sudo nmcli con modify DroneHotspot 802-11-wireless.channel 36
sudo nmcli con modify DroneHotspot 802-11-wireless-security.key-mgmt wpa-psk
sudo nmcli con modify DroneHotspot 802-11-wireless-security.psk "NidarSecureFlight2026"
sudo nmcli con modify DroneHotspot ipv4.method manual ipv4.addresses 192.168.50.10/24
sudo nmcli con up DroneHotspot
```

#### Configuration Option B: Dedicated Field Travel Router (Recommended for Long Range)
Using an external high-power dual-band router (e.g. GL.iNet GL-AXT1800) positioned at the ground station:
- Router Gateway: `192.168.50.1`
- RPi 4: `192.168.50.10`
- Ground Laptop: `192.168.50.20`

---

### 4.2 Eclipse Cyclone DDS Unicast Configuration
Wi-Fi networks frequently drop multicast discovery packets. Configure Cyclone DDS with an explicit unicast peer profile on both machines.

Create `/etc/ros/cyclonedds.xml` on **both** RPi 4 and Ground Laptop:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Discovery>
            <Peers>
                <Peer address="192.168.50.10"/>
                <Peer address="192.168.50.20"/>
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
    </Domain>
</CycloneDDS>
```

Add environment exports to `~/.bashrc` on **both** machines:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Low-Latency Video Pipeline (RPi 4 → Linux Laptop)

Video is transmitted out-of-band over UDP port `5600` via RTP/H.264 to protect ROS 2 DDS communication from video buffer saturation.

### 5.1 Sender Pipeline (Raspberry Pi 4)
Using hardware-accelerated H.264 encoding with low intra-frame intervals (keyframe every 15 frames = 0.5s) to guarantee fast visual recovery if packets are dropped:
```bash
rpicam-vid -t 0 --inline --width 1280 --height 720 --framerate 30 --bitrate 2500000 --intra 15 -o - | \
gst-launch-1.0 fdsrc ! \
  h264parse ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=192.168.50.20 port=5600 sync=false
```
*For standard USB UVC webcams*:
```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw, width=1280, height=720, framerate=30/1 ! \
  videoconvert ! \
  x264enc tune=zerolatency bitrate=2500 speed-preset=ultrafast key-int-max=15 ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=192.168.50.20 port=5600 sync=false
```

### 5.2 Receiver Pipeline (Ground Linux Laptop)
Ultra-low-latency decoding directly into X11/Wayland display:
```bash
gst-launch-1.0 -v udpsrc port=5600 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96" ! \
  rtph264depay ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink sync=false
```
*To route directly into Python / OpenCV / YOLOv8*:
```python
import cv2
pipeline = (
    "udpsrc port=5600 caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96\" ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false"
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        # Run YOLOv8 inference on Laptop GPU
        cv2.imshow("NIDAR Live Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()
```

---

## 6. Flight Controller & 433 MHz MAVLink Integration

### 6.1 Hardware Wiring & Interface Diagram

```text
+-----------------------+               +-----------------------+
|  Flight Controller    |               |    Raspberry Pi 4     |
|      (Pixhawk)        |               |  (Companion Computer) |
|                       |               |                       |
|   [TELEM1 Port]       |               |    [GPIO Header]      |
|     TX/RX/GND/5V      |               |    Pins 8, 10, 6      |
+-----------┬-----------+               +-----------┬-----------+
            │                                       │
            │ Serial UART (57600 baud)              │ Serial UART (921600 baud)
            ▼                                       ▼
+-----------------------+               +-----------------------+
|  YRRC 433 Air Radio   |               | PL011 UART Hardware   |
|  (SiK Transceiver)    |               | (/dev/ttyAMA0)        |
+-----------┬-----------+               +-----------┬-----------+
            )                                       │
      433 MHz RF Link                               │ USB 3.0
            )                                       ▼
            │                           +-----------------------+
            │                           | RTL8812AU Wi-Fi       |
            ▼                           +-----------┬-----------+
+-----------------------+                           )
| YRRC 433 Ground Radio |                     5 GHz Wi-Fi Link
|   (USB connected)     |                           )
+-----------┬-----------+                           ▼
            │ USB (/dev/ttyUSB0)        +-----------------------+
            ▼                           | Ground Linux Laptop   |
+-----------------------+               | • ROS 2 Humble Nodes  |
|   QGroundControl      |               | • RViz2 / Dashboard   |
|   (Control Plane)     |               | • Video Decoder / AI  |
+-----------------------+               +-----------------------+
```

### 6.2 Autopilot Firmware Configuration (Pixhawk)
Connect Pixhawk to your laptop via USB and configure parameters in QGroundControl:

#### If Using PX4 Autopilot:
- **TELEM1 Port (433 MHz Radio)**:
  - `MAV_1_CONFIG` = `101` (Assigns MAVLink instance 1 to TELEM1)
  - `SER_TEL1_BAUD` = `57600` (Standard SiK radio baud rate)
  - `MAV_1_MODE` = `0` (Normal telemetry stream)
  - `MAV_1_RATE` = `1200` B/s (Optimized for low-bandwidth 433 MHz radio)
- **TELEM2 Port (RPi 4 Companion Link)**:
  - `UXRCE_DDS_CFG` = `102` (Assigns Micro-XRCE-DDS client to TELEM2)
  - `SER_TEL2_BAUD` = `921600` (High-speed baud rate)
  - *Alternatively, if using MAVLink for companion*: `MAV_2_CONFIG` = `102`, `SER_TEL2_BAUD` = `921600`.
- **Failsafe Configuration**:
  - `COM_RC_IN_MODE` = `1` (Allows MAVLink ground station control without physical RC)
  - `NAV_RCL_ACT` = `2` (Return to Launch on RC loss, if applicable)
  - `COM_OBL_ACT` = `2` (Return to Launch if companion computer heartbeat stops)

#### If Using ArduPilot (Copter):
- **TELEM1 Port (433 MHz Radio)**:
  - `SERIAL1_PROTOCOL` = `2` (MAVLink 2)
  - `SERIAL1_BAUD` = `57` (57600 baud)
- **TELEM2 Port (RPi 4 Companion Link)**:
  - `SERIAL2_PROTOCOL` = `2` (MAVLink 2)
  - `SERIAL2_BAUD` = `921` (921600 baud)
- **Failsafes**:
  - `FS_GCS_ENABLE` = `1` (RTL if 433 MHz GCS heartbeat lost for > 5s)
  - `FS_OPTIONS` = Bitmask to enable RTL upon companion computer loss

---

### 6.3 Companion Computer to FCU Bridge (RPi 4)

#### Option A: Micro-XRCE-DDS Agent (PX4 Autopilot)
Build and run on RPi 4:
```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j4
sudo make install
sudo ldconfig /usr/local/lib/
```
Run the agent over hardware UART:
```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```
*(Directly exposes native ROS 2 topics: `/fmu/in/...` and `/fmu/out/...` on the RPi 4).*

#### Option B: MAVROS 2 (ArduPilot or PX4)
```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-msgs
ros2 launch mavros mavros.launch.py fcu_url:=serial:///dev/ttyAMA0:921600
```

---

## 7. Approach 3: End-to-End Data Flow & Application Layer Protocols

### 7.1 Priority-Tagged Message Hierarchy (P0–P5)
All data in the system is categorized and mapped to the appropriate physical link:

```text
+-----------------------------------------------------------------------------------------+
|                                    PRIORITY MAPPING                                     |
+----+-------------+--------------------------------+-------------------+-----------------+
| Tag| Category    | Data Types                     | Primary Link      | Transport QoS   |
+----+-------------+--------------------------------+-------------------+-----------------+
| P0 | Emergency   | FCU Heartbeat, E-Stop, Abort   | 433 MHz & Wi-Fi   | Reliable        |
| P1 | Critical    | Attitude, Battery, GPS, Mode   | 433 MHz (MAVLink) | Best Effort     |
| P2 | Important   | Survivor Events, Map Deltas    | Wi-Fi (ROS 2)     | Reliable        |
| P3 | Normal      | Video Stream, Pose, Local Plan | Wi-Fi (UDP/ROS 2) | Best Effort     |
| P4 | Low         | Diagnostics, CPU/Temp Status   | Wi-Fi (ROS 2)     | Best Effort     |
| P5 | Debug       | Raw Point Clouds, Logs         | Wi-Fi (ROS 2)     | Best Effort     |
+----+-------------+--------------------------------+-------------------+-----------------+
```

### 7.2 Application Layer Protocols

#### 1. Survivor Event Schema
When the onboard or ground detection system spots a survivor, it creates a structured event published reliably over `/survivor/detection`:
```json
{
  "event_id": "SURV_042",
  "timestamp": 1727533800.123,
  "confidence": 0.94,
  "class_name": "human",
  "grid_x": 45.2,
  "grid_y": 18.7,
  "gps_lat": 28.613942,
  "gps_lon": 77.209018,
  "thermal_signature_c": 36.8,
  "bounding_box": [320, 240, 110, 205]
}
```

#### 2. Map Compression (Full + Delta RLE)
To prevent massive occupancy grid maps from saturating the Wi-Fi link:
- **Initial Sync**: RPi 4 transmits full baseline occupancy grid (`nav_msgs/msg/OccupancyGrid`) upon connection.
- **Continuous Updates**: Only changed cells (deltas) are published using run-length encoding (RLE). Bandwidth drops from ~2 MB/s to < 25 KB/s.

#### 3. Mission State Machine
```text
                  ┌────────────────────────────────────────┐
                  │                 NORMAL                 │
                  │   Full Wi-Fi Link + 433 MHz Link Alive │
                  └───────────────────┬────────────────────┘
                                      │
                         Wi-Fi Lost / Packet Loss > 30%
                                      │
                                      ▼
                  ┌────────────────────────────────────────┐
                  │                DEGRADED                │
                  │ • 433 MHz Control Plane 100% Active    │
                  │ • Autonomy Continues Locally on RPi 4  │
                  │ • Video Cached Locally to SD/SSD       │
                  │ • Map Deltas Buffered in Memory        │
                  └───────────────────┬────────────────────┘
                                      │
                     Companion Loss / Mission Timeout
                                      │
                                      ▼
                  ┌────────────────────────────────────────┐
                  │               AUTONOMOUS               │
                  │ • Pixhawk Executes Auto-RTL / Land     │
                  │ • Operator Can Override via 433 MHz QGC│
                  └────────────────────────────────────────┘
```

#### 4. Adaptive Publisher (Link-Quality Aware)
The onboard RPi 4 continuously monitors ping latency and packet drop rates to the ground station. It adapts publishing rates dynamically:
- **Good Signal (RTT < 20 ms, Drop < 2%)**: Video at 30 fps (2.5 Mbps), Maps at 2 Hz, Debug logs enabled.
- **Fair Signal (RTT 20–100 ms, Drop 2–15%)**: Video throttled to 15 fps (1.0 Mbps), Maps at 0.5 Hz, Debug logs disabled.
- **Poor / Broken Signal (Drop > 25%)**: Video streaming paused (cached to disk), Map deltas queued, only P2 Survivor Events attempted.

---

## 8. Link-Loss & Failsafe Logic

```text
                            ┌───────────────────────────┐
                            │   Continuous Health Check │
                            └─────────────┬─────────────┘
                                          │
                  ┌───────────────────────┴───────────────────────┐
                  ▼                                               ▼
         [Wi-Fi Link Lost]                               [RPi 4 Power / OS Hang]
    (ROS 2 / Video Unavailable)                         (Companion Link Halted)
                  │                                               │
                  ▼                                               ▼
   • 433 MHz link to Pixhawk UNAFFECTED!           • 433 MHz link to Pixhawk UNAFFECTED!
   • Operator maintains full QGC telemetry.        • Operator maintains full QGC telemetry.
   • RPi 4 enters DEGRADED state:                  • Pixhawk detects companion heartbeat
     - Caches video locally to SD card.              timeout on TELEM2.
     - Continues local obstacle avoidance.         • Pixhawk initiates failsafe:
     - Buffers survivor events in memory.            - Auto-RTL (Return to Launch) or
   • Operator can command RTL or land via              Stationary Loiter.
     independent 433 MHz radio at any time.        • Operator can manually disarm or pilot.
```

### Safety Analysis: True Decoupling vs Dual USB
In Approach 3, **the 433 MHz radio never routes through the companion computer**. This eliminates the single point of failure present in USB-multiplexed architectures:
- An RPi 4 kernel panic, CPU freeze, or USB bus reset has **zero impact** on the pilot's 433 MHz telemetry and failsafe controls.
- The drone can always be recovered or safely landed via QGroundControl.

---

## 9. Step-by-Step Validation & Integration Sequence

### Stage 1: Bench Test RPi 4 ↔ Laptop ROS 2 Discovery
*Objective*: Verify cross-architecture (`arm64` ↔ `x86_64`) DDS discovery without any flight hardware connected.

1. Connect both RPi 4 and Ground Laptop to the Wi-Fi network. Verify ping:
   ```bash
   ping -c 3 192.168.50.10
   ping -c 3 192.168.50.20
   ```
2. Verify firewall allows ROS 2 DDS UDP ports:
   ```bash
   sudo ufw allow 11811:11840/udp
   ```
3. Test topic exchange:
   - On **RPi 4** (`arm64`): `ros2 run demo_nodes_cpp talker`
   - On **Ground Laptop** (`x86_64`): `ros2 run demo_nodes_py listener`
   - Confirm `/chatter` messages arrive cleanly.

---

### Stage 2: Independent 433 MHz MAVLink Link Verification
*Objective*: Verify independent ground station communication with the FCU.

1. Connect YRRC 433 MHz Air Radio to Pixhawk `TELEM1`.
2. Connect YRRC 433 MHz Ground Radio to Laptop USB (`/dev/ttyUSB0`).
3. Launch **QGroundControl** on the Linux laptop.
4. Verify link establishes at 57600 baud. Confirm real-time gyroscope, attitude, and battery telemetry in QGC.

---

### Stage 3: RPi 4 ↔ Pixhawk Serial Communication Test
*Objective*: Confirm RPi 4 communicates with Pixhawk over TELEM2 UART.

1. Verify hardware UART permissions on RPi 4:
   ```bash
   sudo usermod -a -G dialout $USER
   ls -l /dev/ttyAMA0
   ```
2. Launch Micro-XRCE-DDS Agent on RPi 4:
   ```bash
   MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
   ```
3. Verify FCU topics populate on RPi 4:
   ```bash
   ros2 topic list
   ```

---

### Stage 4: Video Streaming & Latency Test
*Objective*: Stream 720p H.264 video from RPi 4 to Ground Laptop and measure end-to-end latency.

1. Start GStreamer receiver on Ground Laptop:
   ```bash
   gst-launch-1.0 -v udpsrc port=5600 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96" ! \
     rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
   ```
2. Start GStreamer sender on RPi 4.
3. Verify video displays on Ground Laptop with < 120 ms latency.

---

### Stage 5: Full Combined Integration Test (Propellers Removed)
*Objective*: Verify all systems operate concurrently without radio interference.

> [!WARNING]
> **SAFETY FIRST**: REMOVE ALL PROPELLERS FROM THE DRONE BEFORE CONDUCTING THIS TEST.

1. Power the drone via LiPo battery.
2. Monitor RPi 4 throttling status and temperature:
   ```bash
   vcgencmd get_throttled && vcgencmd measure_temp
   ```
   *(Throttled must return `0x0`; temperature must remain < 70°C).*
3. Run concurrent operations:
   - 433 MHz Telemetry active in QGroundControl.
   - Video stream active on port 5600.
   - ROS 2 topics active (`ros2 topic hz /vehicle/telemetry`).
4. **Simulate Wi-Fi Disconnection**: Unplug Wi-Fi adapter on laptop.
   - Confirm QGC telemetry remains continuous over 433 MHz.
   - Confirm RPi 4 switches to `DEGRADED` mode and caches video locally.
5. Reconnect Wi-Fi and verify automatic ROS 2 recovery within 5 seconds.

---

## 10. Execution Checklist

- [ ] Flash Ubuntu 22.04 LTS 64-bit Server on RPi 4.
- [ ] Disable Linux serial console on RPi 4 (`/boot/firmware/cmdline.txt` and `config.txt`) for `/dev/ttyAMA0`.
- [ ] Wire Pixhawk TELEM1 to YRRC 433 MHz Air Radio (5V, TX, RX, GND).
- [ ] Wire Pixhawk TELEM2 to RPi 4 GPIO (Pins 8, 10, 6); leave Pin 1 (5V) DISCONNECTED.
- [ ] Power RPi 4 via dedicated 5V 4A UBEC; power Pixhawk via FCU Power Module.
- [ ] Install RTL8812AU Wi-Fi driver on RPi 4 and Ground Laptop.
- [ ] Configure Cyclone DDS unicast peer XML (`/etc/ros/cyclonedds.xml`) on both machines.
- [ ] Build & deploy Micro-XRCE-DDS Agent or MAVROS 2 on RPi 4.
- [ ] Connect Ground YRRC 433 MHz radio to Laptop USB and verify QGroundControl telemetry at 57600 baud.
- [ ] Deploy GStreamer H.264 video pipeline and verify < 120 ms latency.
- [ ] Configure Pixhawk companion loss failsafe (`COM_OBL_ACT` or `FS_GCS_ENABLE`).
- [ ] Execute full ground test with propellers removed.
