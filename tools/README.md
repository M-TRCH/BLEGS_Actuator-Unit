# 🛠️ Tools & Utilities

เครื่องมือสำหรับทดสอบและพัฒนาโปรเจค BLEGS Actuator Unit

---

## 📁 Available Tools

### 1. test_protocol.py
Python script สำหรับทดสอบ Binary Communication Protocol

#### Features
- ✅ ส่งคำสั่ง Direct Position
- ✅ ส่งคำสั่ง S-Curve Profile
- ✅ Ping/health check
- ✅ รับและแสดง status feedback
- ✅ CRC-16 calculation และ verification
- ✅ Packet parsing และ validation

#### Requirements
```bash
pip install pyserial
```

#### Usage

**Basic Test:**
```bash
python test_protocol.py
```

**Custom Port:**
```python
# Edit in test_protocol.py
PORT = 'COM3'  # Change to your port
BAUDRATE = 921600
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

### Port Not Found
```bash
# List available ports
python -c "import serial.tools.list_ports; print(list(serial.tools.list_ports.comports()))"
```

### CRC Errors
- ตรวจสอบ baud rate (ต้องเป็น 921600)
- ลด USB latency timer (1ms)
- ใช้สาย USB ที่มีคุณภาพ

### Timeout
- เพิ่ม timeout: `receive_packet(port, timeout=0.5)`
- ตรวจสอบว่า MCU ทำงานปกติ
- Toggle binary mode ด้วย `B` command

---

## 📚 Additional Resources

- [Binary Protocol Guide](../docs/BINARY_PROTOCOL_GUIDE.md)
- [Python Serial Documentation](https://pyserial.readthedocs.io/)
- [Struct Format Strings](https://docs.python.org/3/library/struct.html)

---

**Maintained by:** M-TRCH  
**Last Updated:** December 3, 2025
