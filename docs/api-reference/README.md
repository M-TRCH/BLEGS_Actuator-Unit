# 💻 API Reference

คู่มือ API สำหรับ BLEGS Actuator Unit

---

## 📋 Overview

BLEGS Actuator Unit มี API สองส่วน:

1. **C++ API** (MCU-side) - สำหรับ firmware development
2. **Python API** (PC-side) - สำหรับ control software

---

## 🔵 C++ API (MCU-side)

### Protocol Functions

#### `protocolInit()`

เริ่มต้น protocol system

```cpp
void protocolInit();
```

**Usage:**
```cpp
void setup() {
    protocolInit();
}
```

---

#### `isBinaryPacketAvailable()`

ตรวจสอบว่ามี binary packet รอรับหรือไม่

```cpp
bool isBinaryPacketAvailable(HardwareSerial* serial);
```

**Parameters:**
- `serial`: Pointer to HardwareSerial (เช่น `SystemSerial`)

**Returns:**
- `true` - มี packet พร้อมรับ
- `false` - ไม่มี packet

**Usage:**
```cpp
void loop() {
    if (isBinaryPacketAvailable(SystemSerial)) {
        // Process packet
    }
}
```

---

#### `receivePacket()`

รับ binary packet จาก serial

```cpp
bool receivePacket(HardwareSerial* serial, BinaryPacket* pkt);
```

**Parameters:**
- `serial`: Pointer to HardwareSerial
- `pkt`: Pointer to BinaryPacket structure

**Returns:**
- `true` - รับสำเร็จ
- `false` - รับไม่สำเร็จ (CRC error, timeout, etc.)

**Usage:**
```cpp
BinaryPacket pkt;
if (receivePacket(SystemSerial, &pkt)) {
    // pkt.type contains packet type
    // pkt.payload contains data
}
```

---

#### `sendStatusFeedback()`

ส่ง status feedback กลับไปยัง PC

```cpp
void sendStatusFeedback(HardwareSerial* serial, int32_t position, 
                       int16_t current, uint8_t flags);
```

**Parameters:**
- `serial`: Pointer to HardwareSerial
- `position`: ตำแหน่งปัจจุบัน (degrees × 100)
- `current`: กระแสปัจจุบัน (mA)
- `flags`: Status flags (bitfield)

**Status Flags:**
```cpp
#define STATUS_MOVING         0x01  // กำลังเคลื่อนที่
#define STATUS_ERROR          0x02  // มี error
#define STATUS_AT_GOAL        0x04  // ถึงเป้าหมายแล้ว
#define STATUS_OVERHEAT       0x08  // ร้อนเกินไป
#define STATUS_OVERCURRENT    0x10  // กระแสสูงเกินไป
#define STATUS_ENCODER_ERROR  0x20  // Encoder ผิดพลาด
```

**Usage:**
```cpp
float current_pos = 45.5;  // degrees
int16_t current_mA = 1500; // 1.5A
uint8_t flags = STATUS_MOVING;

sendStatusFeedback(SystemSerial, 
                  (int32_t)(current_pos * 100),
                  current_mA,
                  flags);
```

---

#### `sendErrorFeedback()`

ส่ง error feedback

```cpp
void sendErrorFeedback(HardwareSerial* serial, uint8_t errorCode, 
                      uint8_t lastPktType, uint16_t debugInfo);
```

**Parameters:**
- `serial`: Pointer to HardwareSerial
- `errorCode`: Error code
- `lastPktType`: Packet type ที่ทำให้เกิด error
- `debugInfo`: ข้อมูล debug เพิ่มเติม

**Error Codes:**
```cpp
#define ERR_CRC_FAILED      0x01
#define ERR_INVALID_PACKET  0x02
#define ERR_TIMEOUT         0x03
#define ERR_UNKNOWN_CMD     0x04
#define ERR_INVALID_PAYLOAD 0x05
```

**Usage:**
```cpp
if (crc_check_failed) {
    sendErrorFeedback(SystemSerial, ERR_CRC_FAILED);
}
```

---

### Motor Control Functions

#### `motor_setMode()`

ตั้งค่าโหมดการควบคุม

```cpp
void motor_setMode(uint8_t mode);
```

**Parameters:**
- `mode`: 
  - `0` = Direct Position Control
  - `1` = S-Curve Position Control

**Usage:**
```cpp
motor_setMode(1);  // S-Curve mode
```

---

#### `motor_setGoalPosition()`

ตั้งค่าตำแหน่งเป้าหมาย

```cpp
void motor_setGoalPosition(float position_deg);
```

**Parameters:**
- `position_deg`: ตำแหน่งในหน่วยองศา

**Usage:**
```cpp
motor_setGoalPosition(90.0);   // ไป 90°
motor_setGoalPosition(-45.5);  // ไป -45.5°
```

---

#### `motor_getCurrentPosition()`

อ่านค่าตำแหน่งปัจจุบัน

```cpp
float motor_getCurrentPosition();
```

**Returns:**
- ตำแหน่งปัจจุบันในหน่วยองศา

**Usage:**
```cpp
float pos = motor_getCurrentPosition();
Serial.println(pos);  // 45.23
```

---

### Encoder Functions

#### `encoder.init()`

เริ่มต้น encoder

```cpp
bool AS5047P_init(SPIClass& spi, uint8_t csPin);
```

**Parameters:**
- `spi`: SPI instance
- `csPin`: Chip select pin

**Returns:**
- `true` - สำเร็จ
- `false` - ล้มเหลว

---

#### `encoder.getAngle()`

อ่านมุมจาก encoder

```cpp
float AS5047P_getAngle();
```

**Returns:**
- มุมในหน่วยองศา (0-360)

---

## 🐍 Python API (PC-side)

### Communication Functions

#### `send_direct_position()`

ส่งคำสั่งตำแหน่งแบบ Direct

```python
def send_direct_position(port, position_deg):
    """
    ส่งคำสั่ง Direct Position
    
    Args:
        port: serial.Serial object
        position_deg: ตำแหน่งในหน่วยองศา (float)
    
    Returns:
        None
    """
```

**Usage:**
```python
import serial
from test_protocol import send_direct_position

port = serial.Serial('COM44', 921600, timeout=1)
send_direct_position(port, 45.0)    # ไป 45°
send_direct_position(port, -90.5)   # ไป -90.5°
```

---

#### `send_scurve_position()`

ส่งคำสั่งตำแหน่งแบบ S-Curve

```python
def send_scurve_position(port, position_deg, duration_ms):
    """
    ส่งคำสั่ง S-Curve Position
    
    Args:
        port: serial.Serial object
        position_deg: ตำแหน่งในหน่วยองศา (float)
        duration_ms: ระยะเวลาในการเคลื่อนที่ (int, milliseconds)
    
    Returns:
        None
    """
```

**Usage:**
```python
send_scurve_position(port, 90.0, 1000)   # ไป 90° ใน 1000ms
send_scurve_position(port, -45.0, 500)   # ไป -45° ใน 500ms
```

---

#### `send_ping()`

ส่ง ping command เพื่อตรวจสอบสถานะ

```python
def send_ping(port):
    """
    ส่ง PING command
    
    Args:
        port: serial.Serial object
    
    Returns:
        None
    """
```

**Usage:**
```python
send_ping(port)
result = receive_packet(port)
```

---

#### `receive_packet()`

รับ packet จาก motor

```python
def receive_packet(port, timeout=0.1):
    """
    รับ binary packet
    
    Args:
        port: serial.Serial object
        timeout: รอรับนานสุด (seconds)
    
    Returns:
        tuple: (packet_type, payload) หรือ None ถ้าไม่สำเร็จ
    """
```

**Usage:**
```python
result = receive_packet(port, timeout=0.5)
if result:
    pkt_type, payload = result
    if pkt_type == 0x81:  # STATUS
        status = parse_status_feedback(payload)
        print(f"Position: {status['position_deg']:.2f}°")
```

---

#### `parse_status_feedback()`

แปลง payload เป็น status dictionary

```python
def parse_status_feedback(payload):
    """
    แปลง STATUS feedback payload
    
    Args:
        payload: bytes
    
    Returns:
        dict: {
            'position_deg': float,
            'current_mA': int,
            'flags': int,
            'moving': bool,
            'at_goal': bool,
            'error': bool
        }
    """
```

**Usage:**
```python
status = parse_status_feedback(payload)
print(f"Position: {status['position_deg']:.2f}°")
print(f"Current: {status['current_mA']} mA")
print(f"Moving: {status['moving']}")
print(f"At Goal: {status['at_goal']}")
```

---

### Helper Functions

#### `calculate_crc16()`

คำนวณ CRC-16 checksum

```python
def calculate_crc16(data):
    """
    คำนวณ CRC-16/MODBUS
    
    Args:
        data: bytes or list of int
    
    Returns:
        int: CRC-16 value (0x0000 - 0xFFFF)
    """
```

**Usage:**
```python
data = [0xFE, 0xEE, 0x01, 0x05, 0x00, 0xB8, 0xE8, 0xFF, 0xFF]
crc = calculate_crc16(data)
print(f"CRC: 0x{crc:04X}")
```

---

## 📚 Complete Examples

### Example 1: C++ Packet Handling

```cpp
#include "protocol.h"

void setup() {
    SystemSerial.begin(921600);
    protocolInit();
}

void loop() {
    if (isBinaryPacketAvailable(SystemSerial)) {
        BinaryPacket pkt;
        if (receivePacket(SystemSerial, &pkt)) {
            switch (pkt.type) {
                case CMD_SET_GOAL:
                    handleSetGoal(&pkt);
                    break;
                    
                case CMD_PING:
                    sendStatusFeedback(SystemSerial, 
                                     current_pos * 100,
                                     current_mA,
                                     status_flags);
                    break;
            }
        }
    }
}

void handleSetGoal(BinaryPacket* pkt) {
    uint8_t mode = pkt->payload[0];
    int32_t target = *(int32_t*)&pkt->payload[1];
    float target_deg = target / 100.0f;
    
    motor_setGoalPosition(target_deg);
    
    // Send feedback
    sendStatusFeedback(SystemSerial,
                      target,
                      current_mA,
                      STATUS_MOVING);
}
```

---

### Example 2: Python Control Loop

```python
import serial
import time
from test_protocol import *

# Connect
port = serial.Serial('COM44', 921600, timeout=1)

# Start motor
port.write(b'S')
time.sleep(2)

# Control loop @ 10Hz
positions = [0, 45, 90, 45, 0, -45, -90, -45, 0]

for pos in positions:
    # Send command
    send_scurve_position(port, pos, 1000)
    
    # Receive feedback
    result = receive_packet(port)
    if result:
        pkt_type, payload = result
        status = parse_status_feedback(payload)
        print(f"Target: {pos:6.1f}°  "
              f"Actual: {status['position_deg']:6.2f}°  "
              f"Moving: {status['moving']}")
    
    time.sleep(1.0)

port.close()
```

---

### Example 3: Real-time Monitoring

```python
import serial
import time
from test_protocol import *

port = serial.Serial('COM44', 921600, timeout=1)
port.write(b'S')
time.sleep(2)

# Poll status @ 50Hz
try:
    while True:
        send_ping(port)
        result = receive_packet(port)
        if result:
            _, payload = result
            status = parse_status_feedback(payload)
            print(f"\rPos: {status['position_deg']:7.2f}°  "
                  f"I: {status['current_mA']:5d}mA  "
                  f"Flags: 0x{status['flags']:02X}", 
                  end='')
        time.sleep(0.02)  # 50Hz
except KeyboardInterrupt:
    print("\nStopped")
    port.close()
```

---

## 🔗 Related Documents

- [Binary Protocol Guide](../technical/PROTOCOL.md) - Protocol specification
- [Quick Start Guide](../getting-started/QUICK_START.md) - การเริ่มต้นใช้งาน
- [User Guide](../getting-started/USER_GUIDE.md) - คำสั่งพื้นฐาน

---

**Last Updated:** December 20, 2025  
**Version:** 1.0
