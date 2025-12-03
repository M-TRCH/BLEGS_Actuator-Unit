# 📚 Documentation

เอกสารประกอบสำหรับโปรเจค BLEGS Actuator Unit

## 📖 Available Documents

### 1. [Binary Protocol Guide](BINARY_PROTOCOL_GUIDE.md)
คู่มือการใช้งาน High-Speed Binary Communication Protocol

**เนื้อหา:**
- Protocol overview และ packet structures
- Command และ feedback packet definitions
- CRC-16 integrity checking
- Python และ C++ API reference
- Timing analysis และ performance metrics
- Troubleshooting guide

**เหมาะสำหรับ:**
- การพัฒนา PC-side control software
- การเข้าใจ protocol specification
- Debug communication issues

---

### 2. [Protocol Upgrade Summary](PROTOCOL_UPGRADE_SUMMARY.md)
สรุปการอัพเกรด protocol จาก ASCII เป็น hybrid mode

**เนื้อหา:**
- ไฟล์ที่เพิ่มเข้ามา
- Features และ improvements
- Performance comparison (ASCII vs Binary)
- Build information และ memory usage
- Usage examples
- Future extensions

**เหมาะสำหรับ:**
- ทำความเข้าใจการเปลี่ยนแปลงที่เกิดขึ้น
- Migration guide
- Quick reference

---

### 3. [Motor Control Guide](MOTOR_CONTROL_GUIDE.md)
คู่มือเกี่ยวกับการควบคุมมอเตอร์ BLDC และ SVPWM

**เนื้อหา:**
- SVPWM (Space Vector Pulse Width Modulation)
- Field-Oriented Control (FOC)
- Park/Clarke transformations
- PID position control
- S-Curve motion planning
- Encoder calibration

**เหมาะสำหรับ:**
- เข้าใจหลักการทำงานของ motor control
- Tuning PID parameters
- Advanced control algorithms

---

## 🎯 Quick Navigation

### สำหรับการเริ่มต้น
1. อ่าน [README หลัก](../README.md) สำหรับ project overview
2. ดู [Protocol Upgrade Summary](PROTOCOL_UPGRADE_SUMMARY.md) สำหรับ quick start
3. ทดสอบด้วย [Python script](../tools/test_protocol.py)

### สำหรับการพัฒนา Protocol
1. [Binary Protocol Guide](BINARY_PROTOCOL_GUIDE.md) - Full specification
2. [Python API Reference](BINARY_PROTOCOL_GUIDE.md#-api-reference)
3. [C++ API Reference](BINARY_PROTOCOL_GUIDE.md#-api-reference)

### สำหรับการ Tuning มอเตอร์
1. [Motor Control Guide](MOTOR_CONTROL_GUIDE.md)
2. Configuration files: `include/motor_conf.h`
3. PID parameters: `include/pid_controller.h`

---

## 📝 Document Version

| Document | Version | Last Updated |
|----------|---------|--------------|
| Binary Protocol Guide | 1.0 | 2025-12-03 |
| Protocol Upgrade Summary | 1.0 | 2025-12-03 |
| Motor Control Guide | 1.0 | [Original] |

---

## 🔄 Updates & Contributions

เอกสารเหล่านี้จะได้รับการอัพเดตเมื่อมีการเพิ่มฟีเจอร์ใหม่หรือพบปัญหา

หากพบข้อผิดพลาดหรือต้องการเพิ่มเติม:
1. สร้าง issue ในโปรเจค
2. แก้ไขและ submit pull request
3. ติดต่อผู้พัฒนา

---

**Maintained by:** M-TRCH  
**Last Review:** December 3, 2025
