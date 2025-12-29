# 📚 BLEGS Actuator Unit - Documentation

ยินดีต้อนรับสู่ศูนย์รวมเอกสารสำหรับ BLEGS Actuator Unit

---

## 🎯 เริ่มต้นที่นี่

### สำหรับผู้ใช้ใหม่
1. **[Quick Start Guide](getting-started/QUICK_START.md)** - เริ่มต้นใช้งานใน 10 นาที
2. **[User Guide](getting-started/USER_GUIDE.md)** - คู่มือการใช้งานพื้นฐาน
3. **[Hardware Setup](guides/HARDWARE_SETUP.md)** - การติดตั้ง hardware

### สำหรับนักพัฒนา
1. **[Binary Protocol](technical/PROTOCOL.md)** - Protocol specification
2. **[API Reference](api-reference/README.md)** - C++ และ Python API
3. **[Commutation Tests](technical/COMMUTATION_TEST_RESULTS.md)** - ผลการทดสอบมอเตอร์

### สำหรับการแก้ปัญหา
1. **[Troubleshooting Guide](guides/TROUBLESHOOTING.md)** - วิธีแก้ปัญหาทั่วไป

---

## 📂 โครงสร้างเอกสาร

```
docs/
│
├── 📁 getting-started/          # เอกสารสำหรับเริ่มต้น
│   ├── QUICK_START.md           # เริ่มต้นใช้งาน (10 นาที)
│   └── USER_GUIDE.md            # คู่มือผู้ใช้งานพื้นฐาน
│
├── 📁 technical/                # เอกสารทางเทคนิค
│   ├── PROTOCOL.md              # Binary Protocol Specification
│   └── COMMUTATION_TEST_RESULTS.md  # ผลการทดสอบ
│
├── 📁 api-reference/            # API Documentation
│   └── README.md                # API Overview
│
└── 📁 guides/                   # คู่มือเฉพาะเรื่อง
    ├── HARDWARE_SETUP.md        # การติดตั้ง hardware
    └── TROUBLESHOOTING.md       # แก้ไขปัญหา
```

---

## 📖 คู่มือตามหัวข้อ

### 🚀 Getting Started

| เอกสาร | คำอธิบาย | เวลา | ระดับ |
|--------|----------|------|-------|
| [Quick Start](getting-started/QUICK_START.md) | เริ่มต้นใช้งานอย่างรวดเร็ว | 10 นาที | 🟢 Beginner |
| [User Guide](getting-started/USER_GUIDE.md) | คู่มือการใช้งานทั้งหมด | 30 นาที | 🟢 Beginner |

---

### 🔧 Hardware & Setup

| เอกสาร | คำอธิบาย | ระดับ |
|--------|----------|-------|
| [Hardware Setup](guides/HARDWARE_SETUP.md) | Pinout, wiring, power | 🟡 Intermediate |
| [Troubleshooting](guides/TROUBLESHOOTING.md) | แก้ไขปัญหาต่างๆ | 🟢 Beginner |

---

### 📡 Communication Protocol

| เอกสาร | คำอธิบาย | ระดับ |
|--------|----------|-------|
| [Binary Protocol](technical/PROTOCOL.md) | Protocol specification ฉบับสมบูรณ์ | 🟡 Intermediate |
| [Protocol Upgrade Summary](PROTOCOL_UPGRADE_SUMMARY.md) | สรุปการอัพเกรด | 🟢 Beginner |

**เนื้อหาครอบคลุม:**
- Packet structure และ format
- Command และ feedback definitions
- CRC-16 integrity checking
- Python และ C++ API examples
- Timing analysis

---

### ⚙️ Motor Control

| เอกสาร | คำอธิบาย | ระดับ |
|--------|----------|-------|
| [Motor Control Guide](MOTOR_CONTROL_GUIDE.md) | ทฤษฎีและการใช้งาน | 🔴 Advanced |
| [Commutation Test Results](technical/COMMUTATION_TEST_RESULTS.md) | ผลการทดสอบ | 🟡 Intermediate |

**เนื้อหาครอบคลุม:**
- Space Vector PWM (SVPWM)
- Field-Oriented Control (FOC)
- Park/Clarke transformations
- PID position control
- S-Curve motion planning

---

## 🎓 Learning Paths

### Path 1: ผู้ใช้งานทั่วไป (2-3 ชั่วโมง)
```
1. Quick Start Guide (10 min)
   ↓
2. User Guide (30 min)
   ↓
3. Hardware Setup (1 hr)
   ↓
4. Troubleshooting (เมื่อมีปัญหา)
```

### Path 2: นักพัฒนา Software (5-6 ชั่วโมง)
```
1. Quick Start Guide (10 min)
   ↓
2. Binary Protocol (2 hr)
   ↓
3. API Reference (1 hr)
   ↓
4. ทดลองเขียน control software (2+ hr)
```

### Path 3: Motor Control Engineer (10+ ชั่วโมง)
```
1. Quick Start Guide (10 min)
   ↓
2. Motor Control Guide (4 hr)
   ↓
3. Commutation Test Results (1 hr)
   ↓
4. ทดลองปรับแต่ง parameters (2+ hr)
```

---

## 🔍 Quick Reference

### ASCII Commands
```
S           # Start motor
M0          # Direct Position mode
M1          # S-Curve mode
#<value>    # Set target position
B           # Toggle Binary Protocol
```

### LED Status
```
🟡 Yellow   → Initializing
🟢 Green    → Ready
🔵 Blue     → Running
🔴 Red      → Error
```

---

## 📊 Technical Specifications

- **MCU:** STM32G431CBU6 (170MHz, 32KB RAM, 128KB Flash)
- **Encoder:** AS5047P (14-bit, ±0.022° accuracy)
- **Communication:** UART @ 921,600 baud
- **Control Loop:** 5 kHz position control
- **Protocol Latency:** ~264 μs round-trip

---

## 🛠️ Common Tasks

### ทดสอบการสื่อสาร
```bash
# ASCII mode
pio device monitor --baud 921600

# Binary mode
cd tools
python test_protocol.py
```

### Build Firmware
```bash
pio run
pio run --target upload
```

---

## 🔗 External Resources

- [PlatformIO Docs](https://docs.platformio.org/)
- [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [AS5047P Datasheet](https://ams.com/as5047p)

---

**🎉 เริ่มต้นด้วย [Quick Start Guide](getting-started/QUICK_START.md)**

---

**Maintained by:** M-TRCH  
**Last Updated:** December 20, 2025  
**Documentation Version:** 1.1

**Maintained by:** M-TRCH  
**Last Review:** December 3, 2025
