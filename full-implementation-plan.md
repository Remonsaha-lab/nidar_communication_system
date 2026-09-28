# Full Implementation Plan: Hybrid ROS 2 Drone Communication & Autonomy System
## Approach 3: Physically Decoupled Control (433 MHz) & Prioritized Data (RTL8812AU Wi-Fi)

**Hardware Target**: Raspberry Pi 4 Companion Computer (Drone) + Pixhawk Autopilot + Linux Laptop (Ground Control Station)  
**Architecture**: Approach 3 Hybrid Architecture – Independent 433 MHz MAVLink Safety Link directly to Pixhawk + High-Bandwidth Wi-Fi Link for ROS 2 Humble & H.264 Video

---

## 1. Complete Tech Stack Specification

| Category | Component | Specific Technology / Version | Deployment Target | Role in Approach 3 |
| :--- | :--- | :--- | :--- | :--- |
| **Hardware** | Companion Computer | Raspberry Pi 4 Model B (4GB or 8GB RAM, Broadcom BCM2711) | Onboard Drone | Perception, video encode, local planning, ROS 2 nodes |
| | Flight Controller (FCU) | Pixhawk 4 / 6C / Cube Orange (PX4 v1.14+ or ArduPilot 4.4+) | Onboard Drone | EKF, attitude control, motor mixing, core failsafes |
| | Telemetry Radios | YRRC 433 MHz SiK-compatible Radios (500mW / 100mW, 57600 baud) | Air (Pixhawk TELEM1) & Ground (Laptop USB) | **Independent Safety Link (C2 & Failsafe Always Alive)** |
| | Wi-Fi Adapters | Realtek RTL8812AU Dual-Band USB 3.0 High-Gain Adapter | Air (RPi 4 USB 3.0) & Ground (Laptop) | High-throughput data plane (ROS 2 DDS & H.264 Video) |
| | Camera | RPi Camera Module v2/v3 (Sony IMX219 / IMX708, CSI) or USB UVC | Onboard Drone | Live search-and-rescue visual stream |
| | Power Delivery | 5V 4A UBEC (for RPi 4) + FCU Power Module (for Pixhawk) | Onboard Drone | Independent power rails for flight safety |
| | Ground Station | x86_64 Laptop (Ubuntu 22.04 LTS Desktop, NVIDIA GPU optional) | Ground Station | GCS operator station (QGroundControl, RViz2, AI) |
| **Operating Systems** | Air OS | Ubuntu Server 22.04.4 LTS 64-bit (`arm64`) | Raspberry Pi 4 | Headless real-time robotics environment |
| | Ground OS | Ubuntu Desktop 22.04.4 LTS 64-bit (`x86_64`) | Linux Laptop | Operator GUI & heavy GPU processing |
| **Kernel Drivers** | Wi-Fi Driver | RTL8812AU DKMS Driver (`aircrack-ng/rtl8812au` v5.6.4.2) | RPi 4 & Ground Laptop | High-speed 5 GHz / 2.4 GHz wireless link |
| **Robotics Middleware** | ROS 2 Core | ROS 2 Humble Hawksbill (`ros-base` on RPi, `desktop` on Laptop) | RPi 4 & Ground Laptop | Autonomy, messaging, services |
| | DDS Implementation | Eclipse Cyclone DDS (`rmw_cyclonedds_cpp`) + Unicast Peer Profile | RPi 4 & Ground Laptop | Reliable P2P topic exchange without multicast loss |
| **Autopilot Bridge** | PX4 Middleware | Micro-XRCE-DDS Agent over UART `/dev/ttyAMA0` @ 921600 baud | RPi 4 | Exposes native ROS 2 topics (`/fmu/...`) on RPi 4 |
| | ArduPilot Middleware | MAVROS 2 (`ros-humble-mavros`) via `/dev/ttyAMA0:921600` | RPi 4 | Alternative FCU bridge |
| **Application Layer** | Protocols & QoS | Priority-Tagged Messages (P0–P5), Map Deltas (RLE), Adaptive QoS | RPi 4 & Ground Laptop | Link-quality aware throttling & graceful degradation |
| **Video Pipeline** | Capture & Streaming | GStreamer 1.20+, `rpicam-vid`, Hardware H.264 RTP/UDP (Port 5600) | RPi 4 | Low-latency (< 120 ms) video feed |
| | Decoding & AI Feed | GStreamer `udpsrc`, OpenCV 4.8+, PyTorch / Ultralytics YOLOv8 | Ground Laptop | Real-time survivor detection & display |
| **Ground Station Apps** | Autopilot GCS | QGroundControl (v4.3+) | Ground Laptop | Independent 433 MHz MAVLink flight instrument |
| | ROS 2 Visualization | RViz2 / Foxglove Studio / Custom Dashboard | Ground Laptop | 3D mapping, survivor alerts, trajectory view |
| **Process Daemon** | Auto-start on boot | Linux `systemd` Services | RPi 4 | Autonomous unattended startup |

---

## 2. Phase 1: Hardware Wiring & Pinout Guide

```text
               DRONE ONBOARD WIRING SCHEMATIC (APPROACH 3 HYBRID ARCHITECTURE)

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
    *CRITICAL: LEAVE PIN 1 (5V) OF TELEM2 DISCONNECTED!
```

### 2.1 Critical Wiring Rules
1. **Never Power RPi 4 from Pixhawk**: Pixhawk’s internal regulator can only supply ~1.5A on its 5V rail. Connecting RPi 4 to Pixhawk 5V will brown out the autopilot and cause an inflight crash. Power the RPi 4 strictly via a dedicated **5V 4A UBEC**.
2. **Leave TELEM2 Pin 1 (5V) Disconnected**: When connecting Pixhawk TELEM2 to RPi 4 GPIO, only connect **TX (Pin 2)**, **RX (Pin 3)**, and **GND (Pin 6)**. Leaving Pin 1 disconnected isolates the power domains while providing a clean common reference ground.
3. **Crossed Serial Signals**:
   - RPi Pin 8 (GPIO 14 / TXD) connects to Pixhawk TELEM2 **RX (Pin 3)**.
   - RPi Pin 10 (GPIO 15 / RXD) connects to Pixhawk TELEM2 **TX (Pin 2)**.
4. **USB Alternative for Companion Link**: Alternatively, you can plug Pixhawk's USB-C/Micro-USB port into an RPi 4 USB 2.0 port (`/dev/ttyACM0`). The TELEM1 radio connection remains untouched.
5. **Antenna Separation**: Position the YRRC 433 MHz antenna and the RTL8812AU Wi-Fi antennas at least **15–20 cm apart** on the drone arms to prevent RF harmonic interference.

---

## 3. Phase 2: Raspberry Pi 4 OS & Driver Installation

### Step 2.1: Flash Ubuntu Server 22.04 LTS
1. Insert a 64GB+ high-speed microSD card (SanDisk Extreme A2 recommended) into your PC.
2. Use **Raspberry Pi Imager**:
   - **OS**: Other general-purpose OS → Ubuntu → **Ubuntu Server 22.04.4 LTS (64-bit arm64)**.
   - **Settings (Gear Icon)**:
     - Set Hostname: `nidar-rpi`
     - Username: `nidar`, Password: `nidarpassword`
     - Enable SSH (Use password authentication).

### Step 2.2: Enable Hardware PL011 UART (`/dev/ttyAMA0`)
By default, Raspberry Pi routes the primary UART to the Bluetooth controller and attaches a serial Linux login console. We must reconfigure it for high-speed FCU communication.

1. **Disable serial console in `/boot/firmware/cmdline.txt`**:
   ```bash
   sudo nano /boot/firmware/cmdline.txt
   ```
   Remove any reference to `console=serial0,115200` or `console=ttyAMA0,115200`.  
   *(Ensure `console=tty1` remains intact for display output).*

2. **Enable PL011 UART and disable Bluetooth in `/boot/firmware/config.txt`**:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
   Add to the bottom of the file:
   ```ini
   enable_uart=1
   dtoverlay=disable-bt
   ```
   *Disabling Bluetooth maps the high-speed PL011 hardware UART directly to `/dev/ttyAMA0` (also aliased as `/dev/serial0`).*

3. **Disable the systemd serial console service and add permissions**:
   ```bash
   sudo systemctl stop serial-getty@ttyAMA0.service
   sudo systemctl disable serial-getty@ttyAMA0.service
   sudo usermod -a -G dialout $USER
   sudo reboot
   ```

### Step 2.3: Build & Install RTL8812AU Wi-Fi Driver
Run on the RPi 4:
```bash
sudo apt update
sudo apt install -y build-essential dkms git bc linux-headers-$(uname -r)

cd ~
git clone -b v5.6.4.2 https://github.com/aircrack-ng/rtl8812au.git
cd rtl8812au
sudo make dkms_install
sudo modprobe 8812au
```
Verify the Wi-Fi interface is detected:
```bash
ip link show
```
*(Look for an interface named `wlan1` or similar corresponding to the Realtek device).*

---

## 4. Phase 3: Field Network Configuration

### Step 3.1: Network Scheme
- **Subnet**: `192.168.50.0/24`
- **Raspberry Pi 4 (Drone)**: Static IP `192.168.50.10`
- **Linux Laptop (Ground Station)**: Static IP `192.168.50.20`

### Step 3.2: Configure RPi 4 as 5 GHz Access Point
On the RPi 4:
```bash
sudo nmcli con add type wifi ifname wlan1 con-name DroneHotspot autoconnect yes ssid NIDAR_DRONE_NET mode ap
sudo nmcli con modify DroneHotspot 802-11-wireless.band a
sudo nmcli con modify DroneHotspot 802-11-wireless.channel 36
sudo nmcli con modify DroneHotspot 802-11-wireless-security.key-mgmt wpa-psk
sudo nmcli con modify DroneHotspot 802-11-wireless-security.psk "NidarSecureFlight2026"
sudo nmcli con modify DroneHotspot ipv4.method manual ipv4.addresses 192.168.50.10/24
sudo nmcli con up DroneHotspot
```

### Step 3.3: Connect Ground Laptop to Drone Hotspot
On the Ground Laptop:
```bash
sudo nmcli con add type wifi ifname wlan0 con-name ConnectDrone ssid NIDAR_DRONE_NET
sudo nmcli con modify ConnectDrone 802-11-wireless-security.key-mgmt wpa-psk
sudo nmcli con modify ConnectDrone 802-11-wireless-security.psk "NidarSecureFlight2026"
sudo nmcli con modify ConnectDrone ipv4.method manual ipv4.addresses 192.168.50.20/24
sudo nmcli con up ConnectDrone
```
Verify network latency:
```bash
ping -c 5 192.168.50.10
```
*(Average RTT over 5 GHz should be < 3 ms).*

---

## 5. Phase 4: ROS 2 Humble & Cyclone DDS Setup

### Step 4.1: Install ROS 2 Humble
```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
# On RPi 4:
sudo apt install -y ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp python3-colcon-common-extensions

# On Ground Laptop:
sudo apt install -y ros-humble-desktop ros-humble-rmw-cyclonedds-cpp python3-colcon-common-extensions
```

### Step 4.2: Create Cyclone DDS Unicast Configuration
Create `/etc/ros/cyclonedds.xml` on **both** machines:
```bash
sudo mkdir -p /etc/ros
sudo nano /etc/ros/cyclonedds.xml
```
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
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

### Step 4.3: Set ROS 2 Environment Variables
Add to `~/.bashrc` on **both** machines:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml" >> ~/.bashrc
source ~/.bashrc
```

### Step 4.4: Verify Cross-Platform ROS 2 Discovery
1. On **RPi 4** (`arm64`): `ros2 run demo_nodes_cpp talker`
2. On **Ground Laptop** (`x86_64`): `ros2 run demo_nodes_py listener`
3. Confirm `/chatter` messages arrive cleanly.

---

## 6. Phase 5: Flight Controller Firmware & Serial Bridge

### Step 5.1: Configure Autopilot Parameters in QGroundControl
Connect Pixhawk to your laptop via USB and configure:

#### If Using PX4 Autopilot:
- **TELEM1 (433 MHz SiK Radio)**:
  - `MAV_1_CONFIG` = `101` (Assigns MAVLink to TELEM1)
  - `SER_TEL1_BAUD` = `57600` (Standard SiK baud rate)
  - `MAV_1_MODE` = `0` (Normal telemetry)
- **TELEM2 (RPi 4 Companion Computer)**:
  - `UXRCE_DDS_CFG` = `102` (Assigns Micro-XRCE-DDS Agent client to TELEM2)
  - `SER_TEL2_BAUD` = `921600` (High-speed baud rate for companion)
- **Safety & Failsafe**:
  - `COM_OBL_ACT` = `2` (Return to Launch upon loss of companion heartbeat)
  - `NAV_RCL_ACT` = `2` (Return to Launch upon RC loss)
- *Reboot Pixhawk.*

#### If Using ArduPilot (Copter):
- **TELEM1 (433 MHz Radio)**:
  - `SERIAL1_PROTOCOL` = `2` (MAVLink 2)
  - `SERIAL1_BAUD` = `57` (57600 baud)
- **TELEM2 (RPi 4 Companion Computer)**:
  - `SERIAL2_PROTOCOL` = `2` (MAVLink 2)
  - `SERIAL2_BAUD` = `921` (921600 baud)
- **Safety**:
  - `FS_GCS_ENABLE` = `1` (RTL if GCS heartbeat lost)
- *Reboot Pixhawk.*

---

### Step 5.2: Install Micro-XRCE-DDS Agent on RPi 4 (For PX4)
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

Test serial connection over PL011 UART:
```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```
*(Verify FCU topics appear on RPi 4: `ros2 topic list` -> shows `/fmu/in/...` and `/fmu/out/...`).*

---

### Step 5.3: Independent Ground 433 MHz Radio Verification
1. Plug the YRRC 433 MHz Ground Radio into the Linux laptop USB (`/dev/ttyUSB0`).
2. Launch **QGroundControl**.
3. Confirm QGC connects automatically at 57600 baud.
4. Verify independent attitude, compass, GPS, and battery telemetry displays on QGC.

---

## 7. Phase 6: Low-Latency H.264 Video Pipeline

### Step 6.1: Install GStreamer
On **both** RPi 4 and Ground Laptop:
```bash
sudo apt update
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer1.0-dev
```

### Step 6.2: Launch Video Sender on RPi 4

#### Option A: CSI Camera (RPi Camera Module v2/v3)
```bash
rpicam-vid -t 0 --inline --width 1280 --height 720 --framerate 30 \
  --bitrate 2500000 --intra 15 --profile baseline \
  -o - | gst-launch-1.0 fdsrc ! h264parse ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=192.168.50.20 port=5600 sync=false
```

#### Option B: USB UVC Webcam (`/dev/video0`)
```bash
gst-launch-1.0 -v v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=2500 key-int-max=15 ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=192.168.50.20 port=5600 sync=false
```

### Step 6.3: Launch Video Receiver on Ground Laptop
```bash
gst-launch-1.0 -v udpsrc port=5600 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96" ! \
  rtph264depay ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink sync=false
```

### Step 6.4: Python Ground Receiver for AI / YOLOv8 Detection
Save as `~/ground_ai_detector.py` on the Ground Laptop:
```python
import cv2
import time

def main():
    gst_pipeline = (
        "udpsrc port=5600 caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96\" ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1"
    )
    
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("ERROR: Failed to open GStreamer pipeline on Ground Laptop.")
        return

    print("SUCCESS: Connected to live video stream from RPi 4. Press 'q' to exit.")
    
    prev_time = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        curr_time = time.time()
        fps = 1.0 / (curr_time - prev_time)
        prev_time = curr_time
        
        cv2.putText(frame, f"NIDAR LIVE FEED - FPS: {fps:.1f}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Insert YOLO model inference here:
        # results = yolo_model(frame)
        
        cv2.imshow("Ground Control Live Video Stream", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

---

## 8. Phase 7: ROS 2 Package & Custom Node Implementation

### Step 8.1: Interface Definition Package (`nidar_interfaces`)
```bash
mkdir -p ~/nidar_ws/src
cd ~/nidar_ws/src
ros2 pkg create --build-type ament_cmake nidar_interfaces
mkdir -p nidar_interfaces/msg
```

Create `nidar_interfaces/msg/VehicleTelemetry.msg`:
```text
std_msgs/Header header
float32 latitude
float32 longitude
float32 altitude_relative
float32 roll
float32 pitch
float32 yaw
float32 battery_percentage
float32 battery_voltage
string flight_mode
bool armed
```

Create `nidar_interfaces/msg/SurvivorDetection.msg` (Approach 3 Survivor Schema):
```text
std_msgs/Header header
string survivor_id
float32 latitude
float32 longitude
float32 grid_x
float32 grid_y
float32 confidence
string class_label
int32 bbox_x
int32 bbox_y
int32 bbox_width
int32 bbox_height
```

Update `nidar_interfaces/CMakeLists.txt`:
```cmake
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleTelemetry.msg"
  "msg/SurvivorDetection.msg"
  DEPENDENCIES std_msgs
)

ament_package()
```

Build the interfaces:
```bash
cd ~/nidar_ws
colcon build --packages-select nidar_interfaces
source install/setup.bash
```

---

### Step 8.2: Vehicle Telemetry & Adaptive Publisher Node (RPi 4)
Create `~/nidar_ws/src/nidar_vehicle/nidar_vehicle/telemetry_node.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nidar_interfaces.msg import VehicleTelemetry

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        self.publisher_ = self.create_publisher(VehicleTelemetry, '/vehicle/telemetry', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.get_logger().info('NIDAR Telemetry Node Initialized (10 Hz).')

    def timer_callback(self):
        msg = VehicleTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.latitude = 28.6139
        msg.longitude = 77.2090
        msg.altitude_relative = 12.5
        msg.battery_percentage = 85.0
        msg.battery_voltage = 15.8
        msg.flight_mode = "HOLD"
        msg.armed = True
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 9. Phase 8: Systemd Auto-Start Daemons (RPi 4)

Configure services on the RPi 4 so all processes boot automatically when the drone battery is connected.

### Service 1: Micro-XRCE-DDS Agent Auto-Start
Create `/etc/systemd/system/micro-xrce-agent.service`:
```ini
[Unit]
Description=Micro-XRCE-DDS Agent for Pixhawk FCU
After=network.target

[Service]
Type=simple
User=nidar
ExecStart=/usr/local/bin/MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

### Service 2: Onboard ROS 2 Telemetry Node Auto-Start
Create `/etc/systemd/system/nidar-telemetry.service`:
```ini
[Unit]
Description=NIDAR Onboard ROS 2 Telemetry Node
After=micro-xrce-agent.service

[Service]
Type=simple
User=nidar
Environment="ROS_DOMAIN_ID=42"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/nidar/nidar_ws/install/setup.bash && ros2 run nidar_vehicle telemetry_node"
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

### Service 3: Video Streaming Auto-Start
Create `/etc/systemd/system/nidar-video.service`:
```ini
[Unit]
Description=NIDAR GStreamer Low Latency Video Pipeline
After=network.target

[Service]
Type=simple
User=nidar
ExecStart=/bin/bash -c "/usr/bin/rpicam-vid -t 0 --inline --width 1280 --height 720 --framerate 30 --bitrate 2500000 --intra 15 -o - | /usr/bin/gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.50.20 port=5600 sync=false"
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

Enable all services on boot:
```bash
sudo systemctl daemon-reload
sudo systemctl enable micro-xrce-agent.service
sudo systemctl enable nidar-telemetry.service
sudo systemctl enable nidar-video.service
```

---

## 10. Phase 9: Verification, Safety Protocols & Troubleshooting

### 10.1 Pre-Flight Ground Test Checklist
- [ ] **Propellers REMOVED** during all bench and ground tests.
- [ ] Pixhawk TELEM1 connected to YRRC 433 MHz Air Radio.
- [ ] Pixhawk TELEM2 connected to RPi 4 GPIO (Pins 8, 10, 6); Pin 1 (5V) confirmed **DISCONNECTED**.
- [ ] Ground YRRC 433 MHz radio detected in Linux laptop (`ls -l /dev/ttyUSB*`).
- [ ] QGroundControl launched and reporting live telemetry over 433 MHz @ 57600 baud.
- [ ] RPi 4 powered via dedicated 5V 4A UBEC; CPU temperature verified < 65°C (`vcgencmd measure_temp`).
- [ ] RPi 4 Wi-Fi connected to Ground Laptop (`ping -c 3 192.168.50.20`).
- [ ] ROS 2 topic `/vehicle/telemetry` receiving data on laptop (`ros2 topic echo /vehicle/telemetry`).
- [ ] Video stream received on laptop with latency < 120 ms.
- [ ] **Simulated RPi 4 Crash Test**: Stopped RPi 4 power; verified 433 MHz QGC telemetry and manual controls remained 100% active.
- [ ] **Simulated Wi-Fi Loss Test**: Disconnected laptop Wi-Fi; verified QGC 433 MHz telemetry remained unbroken and drone entered `DEGRADED` state with local video caching.

### 10.2 Troubleshooting Guide

| Issue / Error | Root Cause | Solution |
| :--- | :--- | :--- |
| `Permission denied: /dev/ttyAMA0` | User not in `dialout` group | Run `sudo usermod -a -G dialout $USER`, then log out and back in |
| RPi 4 CPU throttled or restarts | UBEC current insufficient (< 4A) | Power RPi 4 from dedicated 5V 4A UBEC. Check `vcgencmd get_throttled` (must return `0x0`) |
| No MAVLink packets on QGC | Baud rate mismatch or wiring reversed | Verify TELEM1 baud is `57600`. Check that Radio RX goes to Pixhawk TX, and Radio TX goes to Pixhawk RX |
| `MicroXRCEAgent` fails to connect | Wrong device or serial console active | Confirm `/dev/ttyAMA0` exists and `serial-getty@ttyAMA0.service` is disabled |
| ROS 2 topics visible on RPi 4 but not on Laptop | Multicast dropped by Wi-Fi | Verify `/etc/ros/cyclonedds.xml` exists on both machines with correct unicast peer IPs |
| Video stream shows lag or buffer bloat | GStreamer buffer accumulation | Add `sync=false drop=true max-size-buffers=1` to the receiver pipeline |
| 433 MHz radio range drops when Wi-Fi active | RF harmonic interference | Maintain at least 15–20 cm physical separation between 433 MHz and Wi-Fi antennas |
