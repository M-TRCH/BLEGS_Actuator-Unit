# 📑 BLEGS Actuator Unit - Documentation Index

## 🗂️ โครงสร้างโปรเจค

```
BLEGS_Actuator-Unit/
│
├── 📁 docs/                      # เอกสารทั้งหมด
│   ├── README.md                 # คู่มือนำทางเอกสาร
│   ├── BINARY_PROTOCOL_GUIDE.md  # Protocol specification
│   ├── PROTOCOL_UPGRADE_SUMMARY.md # การอัพเกรดระบบ
│   └── MOTOR_CONTROL_GUIDE.md    # การควบคุมมอเตอร์
│
├── 📁 tools/                     # เครื่องมือสำหรับทดสอบ
│   ├── README.md                 # คู่มือการใช้งาน tools
│   └── test_protocol.py          # Python test script
│
├── 📁 include/                   # Header files
│   ├── protocol.h                # Protocol structures
│   ├── motor_control.h           # Motor control API
│   ├── encoder.h                 # Encoder interface
│   ├── svpwm.h                   # SVPWM implementation
│   └── ...
│
├── 📁 src/                       # Source files
│   ├── main.cpp                  # Main application
│   ├── protocol.cpp              # Protocol implementation
│   ├── motor_control.cpp         # Motor control logic
│   └── ...
│
├── 📁 lib/                       # Third-party libraries
│   ├── AS5047P/                  # Encoder library
│   └── scurve_profile/           # S-curve motion planning
│
└── README.md                     # โปรเจคหลัก
```

---

## 🎯 การนำทางด่วน

### สำหรับผู้เริ่มต้น
1. **[README.md](../README.md)** - เริ่มต้นที่นี่!
2. **[Protocol Upgrade Summary](docs/PROTOCOL_UPGRADE_SUMMARY.md)** - ภาพรวมของระบบ
3. **[Test Protocol](tools/test_protocol.py)** - ทดสอบการสื่อสาร

### สำหรับนักพัฒนา
1. **[Binary Protocol Guide](docs/BINARY_PROTOCOL_GUIDE.md)** - Protocol specification
2. **[Motor Control Guide](docs/MOTOR_CONTROL_GUIDE.md)** - Control algorithms
3. **[Tools README](tools/README.md)** - Development utilities

### สำหรับ Hardware Integration
1. **[platformio.ini](../platformio.ini)** - Build configuration
2. **[include/config.h](../include/config.h)** - Hardware configuration
3. **[include/motor_conf.h](../include/motor_conf.h)** - Motor parameters

---

## 📚 เอกสารหลัก

### 1. Protocol Documentation

| เอกสาร | คำอธิบาย | ประเภท |
|--------|---------|--------|
| [Binary Protocol Guide](docs/BINARY_PROTOCOL_GUIDE.md) | คู่มือ protocol ฉบับสมบูรณ์ | 📘 Guide |
| [Protocol Upgrade Summary](docs/PROTOCOL_UPGRADE_SUMMARY.md) | สรุปการอัพเกรด | 📋 Summary |

**เนื้อหาครอบคลุม:**
- Packet structures และ format
- Command และ feedback definitions
- CRC-16 integrity checking
- API reference (Python + C++)
- Performance analysis
- Troubleshooting guide

### 2. Motor Control Documentation

| เอกสาร | คำอธิบาย | ประเภท |
|--------|---------|--------|
| [Motor Control Guide](docs/MOTOR_CONTROL_GUIDE.md) | คู่มือการควบคุมมอเตอร์ | 📗 Technical |

**เนื้อหาครอบคลุม:**
- SVPWM (Space Vector PWM)
- Field-Oriented Control (FOC)
- PID position control
- S-Curve motion planning
- Encoder calibration

### 3. Tools & Utilities

| ไฟล์ | คำอธิบาย | ภาษา |
|------|---------|------|
| [test_protocol.py](tools/test_protocol.py) | Test client | 🐍 Python |
| [Tools README](tools/README.md) | คู่มือการใช้งาน tools | 📄 Markdown |

---

## 🔍 Quick Reference

### ASCII Commands (Legacy Mode)
```
M0          - Switch to Direct Position Control
M1          - Switch to S-Curve Profile Control
#<value>    - Set target position (degrees)
B           - Toggle Binary Protocol ON/OFF
```

### Binary Protocol Commands
```python
# Python API
send_direct_position(port, 45.0)
send_scurve_position(port, 45.0, 1000)
send_ping(port)
```

### Build Commands
```bash
# Build firmware
pio run

# Upload to board
pio run --target upload

# Clean build
pio run --target clean
```

---

## 📊 Technical Specifications

### Hardware
- **MCU**: STM32G431CBU6 (170MHz, 32KB RAM, 128KB Flash)
- **Encoder**: AS5047P (14-bit absolute magnetic)
- **Communication**: UART @ 921,600 baud
- **Motor Driver**: Three-phase MOSFET inverter

### Performance
- **Control Loop**: 5 kHz position control
- **PWM Frequency**: 10-20 kHz (configurable)
- **Protocol Latency**: ~264 μs round-trip
- **Position Accuracy**: < 0.1° with encoder

### Memory Usage
```
RAM:   15.4% (5040 / 32768 bytes)
Flash: 40.6% (53248 / 131072 bytes)
```

---

## 🎓 Learning Path

### Level 1: Basic Usage
1. Read [README.md](../README.md)
2. Build and upload firmware
3. Test with ASCII commands via Serial Monitor
4. Run [test_protocol.py](tools/test_protocol.py)

### Level 2: Protocol Development
1. Study [Binary Protocol Guide](docs/BINARY_PROTOCOL_GUIDE.md)
2. Understand packet structures
3. Implement custom commands
4. Modify Python test script

### Level 3: Motor Control Tuning
1. Read [Motor Control Guide](docs/MOTOR_CONTROL_GUIDE.md)
2. Understand SVPWM and FOC
3. Tune PID parameters
4. Optimize S-Curve profiles

### Level 4: Advanced Integration
1. Study source code in `src/` and `include/`
2. Add new control modes
3. Implement multi-motor coordination
4. Integrate with higher-level control systems

---

## 🔧 Common Tasks

### ทดสอบการสื่อสาร
```bash
cd tools
python test_protocol.py
```

### แก้ไข PID Parameters
1. Edit `include/motor_conf.h`
2. Rebuild: `pio run`
3. Upload: `pio run --target upload`
4. Test with position commands

### เพิ่ม Command ใหม่
1. Add packet type in `include/protocol.h`
2. Implement handler in `src/protocol.cpp`
3. Add case in `src/main.cpp`
4. Update Python client in `tools/test_protocol.py`

### Debug Communication Issues
1. Toggle to ASCII mode: Send `B`
2. Check baud rate: 921600
3. Verify CRC implementation
4. Use packet analyzer in tools

---

## 📝 Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-03 | Initial binary protocol implementation |
| 0.9.0 | [Original] | ASCII protocol only |

---

## 🤝 Contributing

### การเพิ่มเอกสาร
1. สร้างไฟล์ใหม่ใน `docs/`
2. อัพเดต `docs/README.md`
3. เพิ่มลิงก์ในไฟล์นี้

### การเพิ่ม Tools
1. สร้าง script ใน `tools/`
2. อัพเดต `tools/README.md`
3. เพิ่มตัวอย่างการใช้งาน

---

## 📞 Support & Contact

- **Issues**: [GitHub Issues](https://github.com/M-TRCH/BLEGS_Actuator-Unit/issues)
- **Documentation**: ดูที่ `docs/` folder
- **Examples**: ดูที่ `tools/` folder

---

**Maintained by:** M-TRCH  
**Project Status:** ✅ Active Development  
**Last Updated:** December 3, 2025
