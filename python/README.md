# 🐍 Python Tools & Scripts

Python scripts สำหรับการวิเคราะห์ ควบคุม และทดสอบ BLEGS Quadruped Robot

---

## 📂 โครงสร้างโฟลเดอร์

```
python/
├── control/          # Robot control & gait implementation
├── kinematics/       # IK calculations & workspace analysis
├── analysis/         # Data plotting & performance analysis
├── navigation/       # Path planning & motion estimation
├── simulation/       # Python-based simulations
├── vision/           # Computer vision & AR tag detection
├── sensors/          # IMU & sensor testing
└── models/           # URDF robot models
```

---

## 🎮 Control Scripts

### การควบคุมหุ่นยนต์ Quadruped

| ไฟล์ | คำอธิบาย | สถานะ |
|------|----------|-------|
| **test_quadruped_control.py** | ✨ **Main control script** - Quadruped robot control with Binary Protocol v1.2 | ✅ Active |
| **relative_position_control.py** | Navigation system with path planning and time estimation | ✅ Active |
| **Quadruped_Gait_Control.py** | Quadruped gait control (older version) | 📦 Legacy |
| **Quadruped_Gait_Control_No_EF.py** | Gait control without end-effector link | 📦 Legacy |
| **Gait_Control_Binary_Protocol.py** | Binary protocol gait control | 📦 Legacy |
| **Gait_Control_Real_Motors.py** | Real motor gait testing | 📦 Legacy |
| **Export_Gait_Setpoints.py** | Export gait trajectories to CSV | 🔧 Utility |

### 🌟 test_quadruped_control.py - Main Control Interface

**Features:**
- ✅ Auto-discovery of motors via COM port scanning
- ✅ Motor ID detection using PING command
- ✅ Inverse kinematics for all 4 legs (5-bar linkage)
- ✅ Trot gait implementation
- ✅ Real-time visualization (matplotlib)
- ✅ Binary Protocol v1.2 communication
- ✅ Emergency stop & error handling

**Quick Start:**
```powershell
python test_quadruped_control.py
```

**Controls:**
- `SPACE` - Start/pause gait
- `ESC` - Emergency stop
- `Q` - Quit

**Configuration:**
```python
# Robot dimensions (in test_quadruped_control.py)
BODY_LENGTH = 200.0   # mm
BODY_WIDTH = 170.0    # mm
MOTOR_SPACING = 85.0  # mm

# Gait parameters
UPDATE_RATE = 50      # Hz
GAIT_TYPE = 'trot'    # 'trot', 'walk', 'stand'
```

---

## 🦾 Kinematics Scripts

### Inverse Kinematics & Workspace Analysis

| ไฟล์ | คำอธิบาย |
|------|----------|
| **IK-Five-Bar-Leg-Analytical.py** | Analytical IK solution for 5-bar linkage |
| **IK-Five-Bar-Leg-Numerical.py** | Numerical IK using optimization |
| **IK-Five-Bar-Leg-Animation.py** | Animated IK visualization |
| **Five-Bar-Workspace.py** | Workspace reachability analysis |
| **Quadruped_IK_Test.py** | Full quadruped IK testing |
| **Quadruped_IK_Test_No_EF.py** | Quadruped IK without end-effector |
| **Single_Leg_IK_Test.py** | Single leg IK validation |

**Example:**
```powershell
python IK-Five-Bar-Leg-Analytical.py
```

---

## � Analysis Scripts

### การวิเคราะห์ข้อมูลและประสิทธิภาพ

| ไฟล์ | คำอธิบาย |
|------|----------|
| **Plot_Motor_Log.py** | Plot motor position/current logs |
| **Plot_Foot_Path.py** | Visualize foot trajectories |
| **Quadruped_Gait_Phase_Plot.py** | Gait phase diagram visualization |
| **Dynamic-Torque-Analysis.py** | Torque requirements analysis |
| **Actuation-Weight-Scatter.py** | Actuator weight vs performance |
| **DOF-Weight-Scatter.py** | DOF vs weight scatter plots |
| **TW-Weight-Scatter.py** | Torque-to-weight ratio analysis |
| **TW-Bar-Chart.py** | Torque-to-weight bar charts |
| **Sensor-Usage-Matrix.py** | Sensor usage matrix visualization |

---

## 🎯 Navigation Scripts

### Path Planning & Motion Estimation

| Module | คำอธิบาย |
|--------|----------|
| **simple_planner.py** | Simple navigation path planner |
| **time_estimator.py** | Time-based motion estimation |

**Usage:**
```python
from navigation.simple_planner import SimpleNavigationPlanner
from navigation.time_estimator import TimeBasedEstimator

planner = SimpleNavigationPlanner()
path = planner.plan(start, goal)
```

---

## 👁️ Vision Scripts

### Computer Vision & AR Tag Detection

| ไฟล์/โฟลเดอร์ | คำอธิบาย |
|-------------|----------|
| **AR-Tag-Detection.py** | AR tag detection |
| **AR-Tag-Tracker.py** | Real-time AR tag tracking |
| **AR-Tag-Optimize.py** | AR tag detection optimization |
| **ar_tag/** | AR tag utilities |
| **Color-Blob-Detection.py** | Color blob detection |
| **Hought-Circle.py** | Hough circle detection |
| **Yolo-Test.py** | YOLO object detection testing |
| **Open-Cam.py** | Camera testing utility |
| **camera_calibration/** | Camera calibration tools |

---

## 📡 Sensor Scripts

| ไฟล์ | คำอธิบาย |
|------|----------|
| **GY25_Serial_Test.py** | GY25 IMU serial communication test |

---

## 🧪 Simulation Scripts

### Python-based Simulations

| โฟลเดอร์ | คำอธิบาย |
|---------|----------|
| **gait_control/** | Gait simulation scripts |

---

## 🔧 การติดตั้ง

### ข้อกำหนดเบื้องต้น

- Python 3.7 หรือใหม่กว่า

### ติดตั้ง Dependencies

```powershell
# Core dependencies
pip install numpy matplotlib pyserial

# For vision scripts
pip install opencv-python opencv-contrib-python

# For analysis
pip install pandas scipy
```

---

## � Quick Start Guide

### 1. ควบคุมหุ่นยนต์ด้วย test_quadruped_control.py

```powershell
# 1. เชื่อมต่อ USB cables ทั้ง 8 มอเตอร์
# 2. รัน script
python python/control/test_quadruped_control.py

# Script จะทำ auto-discovery และแสดง:
# - Motor discovery progress
# - COM port assignments
# - Motor ID mapping
# - Real-time leg positions
```

### 2. ทดสอบ Kinematics

```powershell
# Analytical IK solution
python python/kinematics/IK-Five-Bar-Leg-Analytical.py

# Animated workspace visualization
python python/kinematics/IK-Five-Bar-Leg-Animation.py
```

### 3. วิเคราะห์ข้อมูล

```powershell
# Plot motor logs
python python/analysis/Plot_Motor_Log.py <log_file.csv>

# Visualize foot trajectories
python python/analysis/Plot_Foot_Path.py
```

#### Example Output
```
============================================================
High-Speed Binary Protocol - Test Client
============================================================
Connected to COM3 @ 921600 baud

--- Test 1: Ping ---
[TX] Ping
     Packet: fe ee 03 00 fc ff
[RX] Packet Type: 0x81
     Position: -45.23°
     Current: 0 mA
     Flags: 0x00

--- Test 2: Direct Position Command ---
[TX] Direct Position: -45.0°
     Packet: fe ee 01 05 00 b8 e8 ff ff 3a 7c
[RX] Status Feedback:
     Position: -45.20°
     Moving: True
     At Goal: False
```

#### Functions

**Sending Commands:**
```python
send_direct_position(port, target_degrees)
send_scurve_position(port, target_degrees, duration_ms)
send_ping(port)
```

**Receiving Feedback:**
```python
result = receive_packet(port, timeout=0.1)
if result:
    pkt_type, payload = result
    status = parse_status_feedback(payload)
```

**CRC Calculation:**
```python
crc = calculate_crc16(data_bytes)
```

---

## 🚶 2. test_gait_csv.py

### คำอธิบาย
Script สำหรับเล่น gait trajectory จากไฟล์ CSV ในอัตราที่กำหนด รองรับการวนซ้ำและ real-time monitoring

### คุณสมบัติ
- ✅ อ่านตำแหน่งจากไฟล์ CSV
- ✅ ควบคุม playback rate (Hz)
- ✅ รองรับ 2 โหมด: Direct และ S-Curve
- ✅ Real-time feedback monitoring
- ✅ Progress bar แสดงความคืบหน้า
- ✅ Loop mode สำหรับเล่นซ้ำ
- ✅ Verbose mode สำหรับ debugging

---

## 📖 วิธีใช้งาน test_gait_csv.py

### รูปแบบคำสั่งพื้นฐาน

```powershell
python test_gait_csv.py <PORT> <CSV_FILE> [OPTIONS]
```

### Arguments

| Argument | Required | คำอธิบาย |
|----------|----------|----------|
| `PORT` | ✅ | Serial port (เช่น COM9, /dev/ttyUSB0) |
| `CSV_FILE` | ✅ | ไฟล์ CSV ที่มีข้อมูลตำแหน่ง |

### Options

| Option | Default | คำอธิบาย |
|--------|---------|----------|
| `--column` | `motor_A_shaft_deg` | ชื่อคอลัมน์ที่มีข้อมูลตำแหน่ง |
| `--rate` | `200` | อัตราการส่งคำสั่ง (Hz) |
| `--mode` | `direct` | โหมดควบคุม: `direct` หรือ `scurve` |
| `--baudrate` | `921600` | Serial baudrate |
| `--loop` | - | เล่นซ้ำต่อเนื่อง |
| `--verbose` | - | แสดงข้อมูล debug ละเอียด |
| `--no-start` | - | ไม่ส่งคำสั่ง start (มอเตอร์ทำงานอยู่แล้ว) |

---

## 📝 ตัวอย่างการใช้งาน

### 1. เล่น Gait ที่ 200 Hz (แนะนำ)

```powershell
python test_gait_csv.py COM9 gait_setpoints_300steps_200hz.csv --rate 200
```

**ผลลัพธ์:**
```
Loading trajectory from: gait_setpoints_300steps_200hz.csv
Loaded 300 position setpoints from column 'motor_A_shaft_deg'
Range: -1134.61° to -765.69°

Connecting to COM9 at 921600 baud...
Connected!
[TX] Start command sent (ASCII 'S')

============================================================
Starting Trajectory Playback
============================================================
Total Points:  300
Rate:          200 Hz (5.00 ms/point)
Control Mode:  DIRECT
Duration:      1.50 seconds
Loop:          Disabled
============================================================

[████████████████████████░░░░░░░░░░░░░░░░]  60.0% | Point 180/300 | 
Target: -1050.67° | Actual: -1045.23° | Error:  5.44°
```

---

### 2. เล่นช้าๆ เพื่อความแม่นยำ (30 Hz)

```powershell
python test_gait_csv.py COM9 gait_setpoints_300steps_200hz.csv --rate 30
```

**ข้อดี:** Tracking error ต่ำกว่า (2-10°), มอเตอร์ตามตำแหน่งได้ดีกว่า

---

### 3. ใช้ S-Curve สำหรับการเคลื่อนไหวที่นุ่มนวล

```powershell
python test_gait_csv.py COM9 gait.csv --rate 100 --mode scurve
```

---

### 4. Loop ต่อเนื่องสำหรับทดสอบความทนทาน

```powershell
python test_gait_csv.py COM9 gait.csv --rate 100 --loop
```

กด `Ctrl+C` เพื่อหยุด

---

### 5. Debug Mode เพื่อดู Packet ทุกตัว

```powershell
python test_gait_csv.py COM9 gait.csv --rate 50 --verbose
```

**ผลลัพธ์:**
```
[TX] Position: -1003.79° | Packet: fe ee 01 05 00 e6 77 fe ff 8c e7
  [RX] Actual: -1010.23° | Error:  6.44°
[TX] Position: -1007.85° | Packet: fe ee 01 05 00 4f 76 fe ff fc bb
  [RX] Actual: -1013.56° | Error:  5.71°
```

---

### 6. ใช้กับคอลัมน์อื่นใน CSV

```powershell
python test_gait_csv.py COM9 trajectory.csv --column motor_B_angle --rate 150
```

---

## 📊 รูปแบบไฟล์ CSV

### ตัวอย่าง CSV

```csv
time_ms,motor_A_shaft_deg,motor_B_shaft_deg
0,-765.69,-850.23
5,-767.52,-852.41
10,-769.46,-854.67
15,-771.13,-856.88
...
```

### ข้อกำหนด
- ✅ ต้องมี header row
- ✅ ชื่อคอลัมน์ต้องตรงกับที่ระบุใน `--column`
- ✅ ค่าต้องเป็นตัวเลข (degrees)
- ✅ รองรับค่าลบได้

---

## 🎛️ การตั้งค่าที่แนะนำ

### สำหรับ Gait Control (การเดิน)

| Scenario | Rate | Mode | Command |
|----------|------|------|---------|
| **ทดสอบเบื้องต้น** | 30 Hz | direct | `--rate 30` |
| **การเดินปกติ** | 100 Hz | direct | `--rate 100` |
| **การเดินเร็ว** | 200 Hz | direct | `--rate 200` |
| **การเดินนุ่มนวล** | 50 Hz | scurve | `--rate 50 --mode scurve` |

---

## ⚠️ ข้อควรระวัง

### 1. Rate เร็วเกินไป
- **อาการ**: Tracking error สูง (>15°), มอเตอร์ร้อน
- **แก้ไข**: ลด rate ลง (30-100 Hz)

### 2. Serial Port ไม่ตรง
```
Serial port error: could not open port 'COM44'
```
**แก้ไข**: ตรวจสอบ COM port
```powershell
[System.IO.Ports.SerialPort]::getportnames()
```

### 3. CSV File หาไม่เจอ
```
Error: CSV file not found
```
**แก้ไข**: ใช้ absolute path หรือคัดลอกมาที่ tools folder

### 4. Column ไม่มี
```
Column 'motor_X' not found in CSV
```
**แก้ไข**: ระบุชื่อคอลัมน์ที่ถูกต้องด้วย `--column`

---

## 📈 การวิเคราะห์ผลลัพธ์

### ความหมายของ Error

| Error Range | ความหมาย | แนะนำ |
|-------------|----------|-------|
| 0-5° | **ดีเยี่ยม** | เหมาะกับ precision control |
| 5-10° | **ดี** | เหมาะกับ gait ปกติ |
| 10-15° | **พอใช้** | ควรลด rate หรือ tune PID |
| >15° | **แย่** | rate เร็วเกินไป ควรลดลง |

---

## 🔍 Troubleshooting

### ปัญหา: มอเตอร์ไม่เคลื่อนที่
- ตรวจสอบว่ามอเตอร์ start แล้ว
- ใช้ `--verbose` เพื่อดู feedback

### ปัญหา: Actual rate ต่ำกว่า target
- Python serial overhead ~20-30%
- ตั้ง rate สูงกว่าที่ต้องการเล็กน้อย
- ปิด verbose mode

### ปัญหา: Feedback หาย
- ลด rate ลง
- ตรวจสอบ serial buffer
- ใช้ direct mode แทน scurve

---

## 💻 ตัวอย่าง Batch Script

### gait_test.bat
```batch
@echo off
echo Starting Gait Test...
python test_gait_csv.py COM9 gait_setpoints_300steps_200hz.csv --rate 100
pause
```

### continuous_test.bat
```batch
@echo off
echo Continuous Gait Test - Press Ctrl+C to stop
python test_gait_csv.py COM9 gait.csv --rate 100 --loop
pause
```

---

## 🚀 Future Tools

### Planned Utilities

1. **motor_tuner.py**
   - GUI สำหรับ tuning PID parameters
   - Real-time position/current plotting
   - Step response analysis

2. **trajectory_generator.py**
   - สร้าง trajectory files
   - Import/export motion sequences
   - Multi-motor coordination

3. **logger.py**
   - Data logging และ telemetry
   - CSV/JSON export
   - Performance analysis

4. **firmware_flasher.py**
   - Automated firmware upload
   - Version management
   - Configuration backup/restore

---

## 📊 Test Scenarios

### Scenario 1: Position Accuracy Test
```python
positions = [-90, -45, 0, 45, 90]
for target in positions:
    send_direct_position(port, target)
    time.sleep(0.5)
    result = receive_packet(port)
    # Compare target vs actual
```

### Scenario 2: Speed Test
```python
start_time = time.time()
for i in range(100):
    send_ping(port)
    receive_packet(port)
elapsed = time.time() - start_time
print(f"Average: {elapsed/100*1000:.2f} ms per cycle")
```

### Scenario 3: S-Curve Smoothness
```python
# Smooth motion test
send_scurve_position(port, 90, 2000)   # 2 seconds
time.sleep(0.01)
for i in range(200):  # Sample 200 times
    send_ping(port)
    result = receive_packet(port)
    # Log position over time
    time.sleep(0.01)
```

---

## 🐛 Debugging Tools

### Serial Monitor (ASCII Mode)
```python
# Disable binary mode on MCU
send_command(port, b'B\n')

# Now use regular serial monitor
# Commands: M0, M1, #45.0, etc.
```

### Packet Analyzer
```python
# View raw packet bytes
packet = b'\xfe\xee\x01\x05\x00\xb8\xe8\xff\xff\x3a\x7c'
print(f"Header: {packet[0]:02x} {packet[1]:02x}")
print(f"Type: {packet[2]:02x}")
print(f"Length: {packet[3]}")
print(f"Payload: {packet[4:4+packet[3]].hex(' ')}")
print(f"CRC: {packet[-2]:02x} {packet[-1]:02x}")
```

---

## 📝 Development Notes

### Adding New Commands

1. **Update protocol.h** (MCU side)
   ```cpp
   enum PacketType : uint8_t {
       PKT_CMD_NEW_COMMAND = 0x04,  // Add here
   };
   ```

2. **Add Python function** (PC side)
   ```python
   def send_new_command(port, param1, param2):
       # Build packet
       payload = struct.pack('<ii', param1, param2)
       # Calculate CRC and send
   ```

3. **Test**
   ```python
   send_new_command(port, 100, 200)
   result = receive_packet(port)
   ```

---

## 🔧 Troubleshooting


---

## 📚 เอกสารเพิ่มเติม

- **Binary Protocol**: ดูที่ `docs/technical/PROTOCOL.md`
- **User Guide**: ดูที่ `docs/getting-started/USER_GUIDE.md`
- **Hardware Setup**: ดูที่ `docs/guides/HARDWARE_SETUP.md`

---

## 🆘 ช่วยเหลือและสนับสนุน

หากพบปัญหาหรือต้องการความช่วยเหลือ:

1. ตรวจสอบเอกสารใน `docs/`
2. ทดสอบด้วย verbose mode
3. บันทึก error message และ log files

---

---

## ⚙️ การตั้งค่าหุ่นยนต์

### Expected Motor IDs

```python
EXPECTED_MOTOR_IDS = {
    'FL': {'A': 1, 'B': 2},  # Front Left
    'FR': {'A': 3, 'B': 4},  # Front Right
    'RL': {'A': 5, 'B': 6},  # Rear Left
    'RR': {'A': 7, 'B': 8}   # Rear Right
}
```

### Robot Dimensions

- **Body Length**: 200 mm
- **Body Width**: 170 mm
- **Motor Spacing**: 85 mm
- **Link Lengths**: L_AC = L_BD = 105 mm, L_CE = L_DE = 145 mm
- **Gear Ratio**: 8:1

---

## 🐛 Troubleshooting

### Serial Communication Issues

**CRC Errors:**
- ตรวจสอบ baud rate (ต้องเป็น 921600)
- ลด USB latency timer (1ms)
- ใช้สาย USB ที่มีคุณภาพ

**Motor Discovery ล้มเหลว:**
- ตรวจสอบว่า firmware อัพโหลดแล้ว
- ตรวจสอบ COM ports ใน Device Manager
- ลองปิด/เปิด USB ใหม่

**Motors ไม่เคลื่อนที่:**
- ตรวจสอบแหล่งจ่ายไฟ (12-24V)
- ตรวจสอบ encoder connections
- ดู error flags ใน status feedback

### Import Errors

```powershell
# ถ้ามีปัญหา navigation imports
cd python/control
python test_quadruped_control.py
```

---

## 📚 เอกสารเพิ่มเติม

### BLEGS Documentation
- **Protocol**: [docs/technical/PROTOCOL.md](../docs/technical/PROTOCOL.md)
- **Hardware Setup**: [docs/guides/HARDWARE_SETUP.md](../docs/guides/HARDWARE_SETUP.md)
- **Gait Analysis**: [docs/technical/GAIT_CONTROL_ANALYSIS.md](../docs/technical/GAIT_CONTROL_ANALYSIS.md)

### External Resources
- [Python Serial Documentation](https://pyserial.readthedocs.io/)
- [Five-Bar Linkage Kinematics](https://en.wikipedia.org/wiki/Five-bar_linkage)
- [Quadruped Gait Patterns](https://www.mdpi.com/2218-6581/8/2/30)

---

## 🎓 Development Guidelines

### Adding New Control Scripts

1. **Use Binary Protocol v1.2** (see PROTOCOL.md)
2. **Import from test_quadruped_control.py** for constants
3. **Handle serial errors gracefully**
4. **Add documentation in docstrings**

### Code Structure

```python
# Standard imports
import numpy as np
import serial
import time

# Import from test_quadruped_control
import test_quadruped_control as tqc
from test_quadruped_control import (
    BAUD_RATE, ControlMode, EXPECTED_MOTOR_IDS
)

# Your code here
```

---

## 📊 Future Development

### Planned Features

- [ ] Web-based control interface
- [ ] ROS2 integration
- [ ] Machine learning gait optimization
- [ ] Advanced vision-based navigation
- [ ] Multi-robot coordination
- [ ] Virtual reality teleoperation

---

**เวอร์ชัน**: 2.0  
**อัพเดทล่าสุด**: 8 กุมภาพันธ์ 2026  
**ผู้พัฒนา**: M-TRCH
