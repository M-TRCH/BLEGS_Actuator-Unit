# 🚀 Quick Start Guide

คู่มือเริ่มต้นใช้งาน BLEGS Actuator Unit อย่างรวดเร็ว

---

## 📋 สิ่งที่ต้องเตรียม

### Hardware
- ✅ BLEGS Actuator Unit (STM32G431CBU6)
- ✅ BLDC Motor with AS5047P Encoder
- ✅ 24V DC Power Supply
- ✅ USB-to-Serial Adapter (รองรับ 921,600 baud)

### Software
- ✅ PlatformIO (หรือ Arduino IDE with STM32 support)
- ✅ Python 3.6+ with pyserial
- ✅ Serial Terminal (PuTTY, Arduino Serial Monitor, etc.)

---

## 🔌 การเชื่อมต่อ Hardware

### 1. Power Supply
```
24V DC → BLEGS Actuator Unit
GND    → Common Ground
```

### 2. Serial Communication
```
USB-Serial → BLEGS UART
TX         → RX
RX         → TX
GND        → GND
```

### 3. Motor Connection
- 3-phase motor wires → Motor driver output
- Encoder SPI → STM32 SPI pins
- Verify encoder power (3.3V or 5V)

---

## 💻 การติดตั้ง Software

### 1. ติดตั้ง PlatformIO

**Windows:**
```bash
# Using Python pip
pip install platformio
```

**หรือใช้ VS Code Extension:**
1. เปิด VS Code
2. ติดตั้ง extension "PlatformIO IDE"
3. Reload VS Code

### 2. ติดตั้ง Python Dependencies

```bash
pip install pyserial
```

---

## 🔨 Build และ Upload Firmware

### ใช้ PlatformIO CLI

```bash
# Clone project (ถ้ายังไม่มี)
git clone https://github.com/M-TRCH/BLEGS_Actuator-Unit.git
cd BLEGS_Actuator-Unit

# Build firmware
pio run

# Upload to board
pio run --target upload

# Monitor serial output
pio device monitor --baud 921600
```

### ใช้ VS Code + PlatformIO

1. เปิดโปรเจคใน VS Code
2. กด **PlatformIO: Build** (Ctrl+Alt+B)
3. กด **PlatformIO: Upload** (Ctrl+Alt+U)
4. กด **PlatformIO: Serial Monitor** (Ctrl+Alt+S)

---

## 🎮 การทดสอบครั้งแรก

### 1. ตรวจสอบการเชื่อมต่อ Serial

เปิด Serial Monitor ที่ **921,600 baud** คุณจะเห็น:

```
[INIT] BLEGS Actuator Unit
[INIT] Encoder initialized
[INIT] Motor alignment...
[READY] System ready - Press START button
```

### 2. เริ่มการทำงานของมอเตอร์

**วิธีที่ 1: กดปุ่ม START บนบอร์ด**
- LED จะเปลี่ยนเป็นสีน้ำเงิน
- มอเตอร์พร้อมรับคำสั่ง

**วิธีที่ 2: ส่งคำสั่งผ่าน Serial**
```
S
```
Response: `Motor started`

### 3. ทดสอบการควบคุมตำแหน่ง (ASCII Mode)

```
M1              → เปลี่ยนเป็น S-Curve mode
#0              → ย้ายไปตำแหน่ง 0°
#90             → ย้ายไปตำแหน่ง 90°
#-45.5          → ย้ายไปตำแหน่ง -45.5°
```

คุณจะเห็น feedback แบบนี้:
```
Returns:    12.5    12.3
Returns:    45.8    45.6
Returns:    89.9    90.0
```

### 4. ทดสอบ Binary Protocol

```bash
# Run Python test script
cd tools
python test_protocol.py
```

Output ที่คาดหวัง:
```
============================================================
High-Speed Binary Protocol - Test Client
============================================================
Connected to COM44 @ 921600 baud

--- Sending Start Command ---
[TX] Start Command (ASCII 'S')
Motor should now be running...

--- Test 1: Ping ---
[RX] Position: -1.27°  Current: 0 mA  Flags: 0x00

--- Test 2: Direct Position Command ---
[TX] Direct Position: -45.0°
[RX] Position: -1.25°  Moving: True
```

---

## 🎯 การใช้งานพื้นฐาน

### ASCII Protocol (สำหรับ Debug)

| คำสั่ง | คำอธิบาย | ตัวอย่าง |
|--------|----------|----------|
| `S` | เริ่มการทำงานของมอเตอร์ | `S` |
| `M0` | เปลี่ยนเป็น Direct Position mode | `M0` |
| `M1` | เปลี่ยนเป็น S-Curve mode | `M1` |
| `#<pos>` | ตั้งค่าตำแหน่งเป้าหมาย | `#90.5` |
| `B` | สลับ Binary Protocol ON/OFF | `B` |

### Binary Protocol (สำหรับ High-Speed Control)

```python
import serial
from test_protocol import *

# Connect
port = serial.Serial('COM44', 921600, timeout=1)

# Start motor
port.write(b'S')
time.sleep(2)

# Send position command
send_direct_position(port, 45.0)
result = receive_packet(port)

# Send S-Curve command
send_scurve_position(port, -90.0, 1000)  # Move to -90° in 1000ms
result = receive_packet(port)
```

---

## 📊 สถานะ LED

| สี | สถานะ | ความหมาย |
|----|-------|----------|
| 🟡 เหลือง | INIT | กำลังเริ่มต้นระบบ |
| 🟢 เขียว | READY | พร้อมทำงาน |
| 🔵 น้ำเงิน | RUNNING | กำลังทำงาน |
| 🔴 แดง | ERROR | เกิดข้อผิดพลาด |
| 🟠 ส้ม | WARNING | คำเตือน |

---

## ⚠️ การแก้ปัญหาเบื้องต้น

### ปัญหา: ไม่มี Serial Output

**แก้ไข:**
1. ตรวจสอบ COM port ให้ถูกต้อง
2. ตรวจสอบ Baud rate = 921,600
3. ตรวจสอบการเชื่อมต่อ TX/RX (สลับกัน)
4. Reset บอร์ด

### ปัญหา: มอเตอร์ไม่เคลื่อนที่

**แก้ไข:**
1. ตรวจสอบว่าได้กด START button หรือส่ง `S` แล้ว
2. ตรวจสอบ LED status (ต้องเป็นสีน้ำเงิน)
3. ตรวจสอบ 24V power supply
4. ตรวจสอบ motor connections

### ปัญหา: LED แสดงสีแดง

**แก้ไข:**
1. มุมปัจจุบันอยู่นอกช่วง calibration (±20°)
2. หมุนมอเตอร์ด้วยมือให้อยู่ในตำแหน่งที่ถูกต้อง
3. Reset บอร์ดและลองใหม่

### ปัญหา: Python Script Error

**แก้ไข:**
1. ตรวจสอบว่าติดตั้ง `pyserial` แล้ว
   ```bash
   pip install pyserial
   ```
2. แก้ไข COM port ใน `test_protocol.py`
3. ปิด Serial Monitor ก่อนรัน Python script

---

## 📚 เอกสารเพิ่มเติม

### สำหรับผู้เริ่มต้น
- [Hardware Configuration](../guides/HARDWARE_SETUP.md) - การตั้งค่า hardware
- [Basic Commands](USER_GUIDE.md) - คำสั่งพื้นฐาน
- [Troubleshooting](../guides/TROUBLESHOOTING.md) - การแก้ปัญหา

### สำหรับนักพัฒนา
- [Binary Protocol Guide](../technical/PROTOCOL.md) - Protocol specification
- [Motor Control Theory](../technical/MOTOR_CONTROL_THEORY.md) - ทฤษฎีการควบคุมมอเตอร์
- [API Reference](../api-reference/README.md) - C++ และ Python API

---

## ✅ Checklist การเริ่มต้นใช้งาน

- [ ] ติดตั้ง PlatformIO และ Python
- [ ] เชื่อมต่อ hardware ครบถ้วน
- [ ] Build และ upload firmware สำเร็จ
- [ ] เชื่อมต่อ Serial @ 921,600 baud
- [ ] เห็น initialization messages
- [ ] กด START หรือส่ง `S` แล้ว
- [ ] ทดสอบคำสั่ง ASCII (`M1`, `#90`)
- [ ] ทดสอบ Python script สำเร็จ
- [ ] LED แสดงสถานะถูกต้อง
- [ ] มอเตอร์เคลื่อนที่ได้ปกติ

---

**🎉 ยินดีด้วย! คุณพร้อมใช้งาน BLEGS Actuator Unit แล้ว**

**Next Steps:**
1. อ่าน [User Guide](USER_GUIDE.md) เพื่อเรียนรู้คำสั่งทั้งหมด
2. ศึกษา [Binary Protocol](../technical/PROTOCOL.md) สำหรับ high-speed control
3. ปรับแต่ง [PID Parameters](../guides/PID_TUNING.md) ตามความต้องการ
4. Explore [Examples](../../tools/README.md) และ test scripts

---

**Last Updated:** December 20, 2025  
**Version:** 1.0
