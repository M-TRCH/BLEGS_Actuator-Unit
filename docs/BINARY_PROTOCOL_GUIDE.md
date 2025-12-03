# Binary Protocol Implementation Guide

## ภาพรวม (Overview)

โปรเจคนี้รองรับ **Hybrid Protocol Mode** ที่สามารถทำงานได้ทั้ง:
1. **ASCII Protocol** (Legacy) - สำหรับ debugging และทดสอบด้วยมือ
2. **Binary Protocol** (New) - สำหรับ high-speed control loop (1 kHz)

---

## 🔧 การติดตั้ง (Installation)

### ไฟล์ที่เพิ่มเข้ามา

```
include/
  └── protocol.h        # Protocol structures และ function declarations
src/
  └── protocol.cpp      # Protocol implementation (CRC, parsing, etc.)
  └── main.cpp          # Updated with hybrid protocol support
test_protocol.py        # Python test client
```

### Dependencies

- **MCU Side**: ไม่มี dependency เพิ่มเติม (ใช้ Arduino core เท่านั้น)
- **PC Side**: Python 3.6+ กับ `pyserial`

```bash
pip install pyserial
```

---

## 📡 การใช้งาน (Usage)

### 1. ASCII Protocol (Legacy Mode)

ยังคงใช้งานได้เหมือนเดิม:

```
M0              # เปลี่ยนเป็น Direct Position Control
M1              # เปลี่ยนเป็น S-Curve Control
#-45.5          # ตั้งค่าตำแหน่งเป้าหมาย -45.5 degrees
B               # Toggle Binary Protocol ON/OFF
```

### 2. Binary Protocol (High-Speed Mode)

#### Packet Structure

```
┌──────────┬──────────┬─────────────┬─────────────┬─────────┬─────────┐
│ Header 1 │ Header 2 │ Packet Type │ Payload Len │ Payload │  CRC16  │
│  (0xFE)  │  (0xEE)  │   (1 byte)  │   (1 byte)  │  (var)  │ (2 byte)│
└──────────┴──────────┴─────────────┴─────────────┴─────────┴─────────┘
```

#### Packet Types

| ID | Direction | Description |
|----|-----------|-------------|
| `0x01` | PC → Motor | CMD_SET_GOAL (ตั้งค่าตำแหน่งเป้าหมาย) |
| `0x03` | PC → Motor | CMD_PING (ตรวจสอบสถานะ) |
| `0x81` | Motor → PC | FB_STATUS (รายงานสถานะปัจจุบัน) |
| `0x83` | Motor → PC | FB_ERROR (รายงานข้อผิดพลาด) |

---

## 📦 Command Packets

### CMD_SET_GOAL (0x01)

#### Mode 0: Direct Position

```python
# Python example
send_direct_position(port, -45.0)  # Move to -45.0 degrees
```

**Payload Structure:**
```
┌──────────────┬────────────────────────────┐
│ Control Mode │     Target Position        │
│   (0x00)     │  (int32, degrees * 100)    │
│   1 byte     │         4 bytes            │
└──────────────┴────────────────────────────┘
```

**Example Packet:**
```
FE EE 01 05 00 B8 E8 FF FF [CRC_L] [CRC_H]
         │  │  └──────────┘
         │  │  Target: -45.00° = -4500 (0xFFFFE8B8)
         │  └─ Control Mode: 0x00 (Direct)
         └──── Payload Length: 5 bytes
```

#### Mode 1: S-Curve Profile

```python
# Python example
send_scurve_position(port, 45.0, 1000)  # Move to 45° in 1000ms
```

**Payload Structure:**
```
┌──────────────┬────────────────────────────┬──────────────┐
│ Control Mode │     Target Position        │  Duration    │
│   (0x01)     │  (int32, degrees * 100)    │ (uint16, ms) │
│   1 byte     │         4 bytes            │   2 bytes    │
└──────────────┴────────────────────────────┴──────────────┘
```

**Example Packet:**
```
FE EE 01 07 01 94 11 00 00 E8 03 [CRC_L] [CRC_H]
         │  │  └──────────┘ └───┘
         │  │  Target: 45.00° Duration: 1000ms
         │  └─ Control Mode: 0x01 (S-Curve)
         └──── Payload Length: 7 bytes
```

### CMD_PING (0x03)

ใช้ตรวจสอบสถานะและความเร็วของการตอบสนอง

**Payload:** ไม่มี (length = 0)

```
FE EE 03 00 [CRC_L] [CRC_H]
```

---

## 📨 Feedback Packets

### FB_STATUS (0x81)

มอเตอร์จะส่งกลับทันทีหลังได้รับคำสั่ง

**Payload Structure:**
```
┌─────────────────┬─────────────────┬──────────────┐
│ Actual Position │ Actual Current  │ Status Flags │
│ (int32, deg*100)│  (int16, mA)    │  (uint8)     │
│    4 bytes      │    2 bytes      │   1 byte     │
└─────────────────┴─────────────────┴──────────────┘
```

**Status Flags (Bitfield):**
- Bit 0: `STATUS_MOVING` - กำลังเคลื่อนที่
- Bit 1: `STATUS_ERROR` - เกิดข้อผิดพลาด
- Bit 2: `STATUS_AT_GOAL` - ถึงตำแหน่งเป้าหมายแล้ว
- Bit 3: `STATUS_OVERHEAT` - อุณหภูมิสูงเกินไป
- Bit 4: `STATUS_OVERCURRENT` - กระแสสูงเกินไป
- Bit 5: `STATUS_ENCODER_ERROR` - Encoder มีปัญหา

**Example Response:**
```
FE EE 81 07 94 11 00 00 00 00 04 [CRC_L] [CRC_H]
         │  └──────────┘ └───┘ └─
         │  Position: 45.00°    Status: 0x04 (AT_GOAL)
         │  Current: 0 mA
         └─ Payload Length: 7 bytes
```

### FB_ERROR (0x83)

รายงานข้อผิดพลาดเมื่อเกิดปัญหา

**Payload Structure:**
```
┌────────────┬──────────────────┬─────────────┐
│ Error Code │ Last Packet Type │ Debug Info  │
│  (uint8)   │     (uint8)      │  (uint16)   │
│  1 byte    │     1 byte       │  2 bytes    │
└────────────┴──────────────────┴─────────────┘
```

**Error Codes:**
- `0x01` - CRC Failed
- `0x02` - Invalid Packet
- `0x03` - Timeout
- `0x04` - Unknown Command
- `0x05` - Invalid Payload

---

## 🧪 การทดสอบ (Testing)

### 1. ทดสอบด้วย Python Script

```bash
# แก้ไข PORT ใน test_protocol.py
python test_protocol.py
```

**Output ตัวอย่าง:**
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

### 2. ทดสอบด้วย Serial Monitor (ASCII Mode)

1. เปิด Serial Monitor @ 921,600 baud
2. ส่งคำสั่ง:
   ```
   B          # Disable binary mode
   M1         # S-Curve mode
   #45.0      # Move to 45 degrees
   ```

---

## ⚡ Performance Analysis

### Timing (@ 921,600 baud, 11 μs/byte)

| Packet Type | Size | TX Time | RX Time | Round-Trip |
|-------------|------|---------|---------|------------|
| Direct Position | 10 bytes | 110 μs | 132 μs | **242 μs** |
| S-Curve Profile | 12 bytes | 132 μs | 132 μs | **264 μs** |
| Ping | 6 bytes | 66 μs | 132 μs | **198 μs** |

**สรุป:** ใช้เวลาเพียง ~26% ของ 1ms control loop → เหลือเวลา 740 μs สำหรับ computation

### Bandwidth Utilization

สำหรับหุ่นยนต์ 8 มอเตอร์ (parallel UART):
- แต่ละมอเตอร์: 264 μs
- **Total latency: ~264 μs** (เพราะส่งพร้อมกัน)
- Overhead: 26.4% ของ 1ms loop

---

## 🔍 Troubleshooting

### ปัญหา: CRC Failed

**สาเหตุ:**
- Noise on UART line
- Baud rate mismatch
- USB latency timer ไม่เหมาะสม

**แก้ไข:**
1. ตรวจสอบ baud rate ให้ตรงกัน (921,600)
2. ลด Latency Timer ของ USB-TTL driver เป็น 1ms
3. ใช้สาย USB ที่ดี หรือเพิ่ม pull-up resistor

### ปัญหา: No Response

**ตรวจสอบ:**
```
B          # Toggle binary mode
```
หากได้ response "Binary mode: ENABLED" แสดงว่า MCU ทำงาน แต่ binary packet อาจมีปัญหา

### ปัญหา: Python Test Timeout

```python
# เพิ่ม timeout
result = receive_packet(port, timeout=0.5)
```

---

## 🚀 Advanced Usage

### การส่งคำสั่งต่อเนื่อง (Stream Control)

```python
# Example: Smooth trajectory
import numpy as np

positions = np.linspace(-90, 90, 100)
for pos in positions:
    send_direct_position(port, pos)
    time.sleep(0.001)  # 1kHz
    receive_packet(port, timeout=0.002)
```

### การอ่านสถานะแบบ Real-time

```python
# Poll status every 10ms
while True:
    send_ping(port)
    result = receive_packet(port)
    if result:
        pkt_type, payload = result
        status = parse_status_feedback(payload)
        print(f"Pos: {status['position_deg']:6.2f}°", end='\r')
    time.sleep(0.01)
```

---

## 📝 Migration Guide (ASCII → Binary)

### เดิม (ASCII):
```python
port.write(b'#45.5\n')
```

### ใหม่ (Binary):
```python
send_scurve_position(port, 45.5, 1000)
result = receive_packet(port)
```

### ประโยชน์:
- ✅ **เร็วกว่า 10x** (no parsing overhead)
- ✅ **มี CRC** ตรวจสอบความถูกต้อง
- ✅ **Auto feedback** ทันทีหลังส่งคำสั่ง
- ✅ **Structured data** ง่ายต่อการขยาย

---

## 📚 API Reference

### C++ (MCU Side)

```cpp
#include "protocol.h"

// Initialize protocol
protocolInit();

// Check for binary packet
if (isBinaryPacketAvailable(SystemSerial)) {
    BinaryPacket pkt;
    if (receivePacket(SystemSerial, &pkt)) {
        // Process packet
    }
}

// Send feedback
sendStatusFeedback(SystemSerial, position*100, current, flags);

// Send error
sendErrorFeedback(SystemSerial, ERR_CRC_FAILED);
```

### Python (PC Side)

```python
from test_protocol import *

# Send commands
send_direct_position(port, -45.0)
send_scurve_position(port, 45.0, 1000)
send_ping(port)

# Receive response
result = receive_packet(port)
if result:
    pkt_type, payload = result
    status = parse_status_feedback(payload)
```

---

## 🔮 Future Extensions

### Planned Features:
1. **CMD_SET_CONFIG (0x02)** - Runtime PID tuning
2. **Torque/Velocity Control Modes**
3. **Multi-motor synchronized commands**
4. **Data logging packets**

### How to Add New Packet Type:

1. เพิ่ม enum ใน `protocol.h`
2. เพิ่ม payload struct
3. เพิ่ม case ใน `main.cpp`
4. อัพเดต Python client

---

**สร้างโดย:** M-TRCH  
**วันที่:** 2025-12-03  
**เวอร์ชัน:** 1.0
