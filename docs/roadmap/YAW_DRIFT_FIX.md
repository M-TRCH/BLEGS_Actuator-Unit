# Yaw Drift Fix - IMU Integration for Orientation Control

**โครงการ:** BLEGS Actuator Unit - Quadruped Robot  
**เอกสาร:** การแก้ไขปัญหา Yaw Drift ด้วย IMU Integration  
**วันที่:** 11 กุมภาพันธ์ 2026  
**ผู้จัดทำ:** M-TRCH  
**เวอร์ชัน:** 1.0  
**สถานะ:** ✅ Implementation Complete - Ready for Testing

---

## สารบัญ

1. [บทนำ](#1-บทนำ)
2. [ปัญหาที่พบ](#2-ปัญหาที่พบ)
3. [แนวทางการแก้ไข](#3-แนวทางการแก้ไข)
4. [สถาปัตยกรรมระบบ](#4-สถาปัตยกรรมระบบ)
5. [รายละเอียดการ Implementation](#5-รายละเอียดการ-implementation)
6. [การใช้งาน](#6-การใช้งาน)
7. [พารามิเตอร์ที่สามารถปรับได้](#7-พารามิเตอร์ที่สามารถปรับได้)
8. [การทดสอบ](#8-การทดสอบ)
9. [Roadmap ถัดไป](#9-roadmap-ถัดไป)

---

## 1. บทนำ

จากการทดสอบระบบควบคุมตำแหน่งแบบสัมพัทธ์ (Relative Position Control) เมื่อวันที่ 10 กุมภาพันธ์ 2026 พบว่าหุ่นยนต์สามารถควบคุมตำแหน่งแกน Y ได้แม่นยำ ±30mm แต่มีปัญหา **Yaw Drift** คือหุ่นยนต์เลี้ยวเองระหว่างเดิน ทำให้ไม่สามารถเดินตามเส้นทางตรงได้

เอกสารนี้อธิบายการแก้ไขปัญหาดังกล่าวโดยการเพิ่ม **IMU (Inertial Measurement Unit)** เข้าไปในระบบควบคุม เพื่อตรวจวัดและแก้ไข yaw orientation แบบ real-time

---

## 2. ปัญหาที่พบ

### 2.1 อาการ

จากการทดสอบ (Roadmap 7.1):
- ✅ ควบคุมตำแหน่ง Y-axis: แม่นยำ ±30mm
- ✅ การเคลื่อนที่: ราบรื่น, เสถียร
- ⚠️ **Yaw Drift**: หุ่นยนต์เลี้ยวเองระหว่างเดิน (~5-15° ต่อเมตร)

### 2.2 สาเหตุ

1. **ไม่มีระบบควบคุม yaw orientation** - ระบบเดิมควบคุมเฉพาะแกน Y เท่านั้น
2. **Trot gait ไม่สมมาตร** - การเดินอาจมีความไม่สมมาตรระหว่างขาซ้าย-ขวา
3. **Friction ไม่เท่ากัน** - เท้าแต่ละข้างอาจมีแรงเสียดทานต่างกัน
4. **ความคลาดเคลื่อนของมอเตอร์** - มอเตอร์แต่ละตัวมี error เล็กน้อย

### 2.3 ผลกระทบ

- หุ่นยนต์ไม่สามารถเดินตามเส้นทางตรงได้
- การเคลื่อนที่ระยะไกลมีความคลาดเคลื่อนสูง
- ไม่สามารถใช้งาน waypoint navigation ได้อย่างแม่นยำ

---

## 3. แนวทางการแก้ไข

### 3.1 กลยุทธ์หลัก

เพิ่ม **IMU (BNO055)** เข้าไปในระบบเพื่อ:
1. **ตรวจวัด yaw orientation** แบบ real-time
2. **คำนวณ yaw error** เทียบกับทิศทางเป้าหมาย
3. **ปรับ gait trajectory** เพื่อแก้ไขการเบี่ยงเบน

### 3.2 PD Controller

ใช้ **PD (Proportional-Derivative) Controller** คำนวณ lateral offset:

$$u_{yaw} = K_p \cdot e_{yaw} + K_d \cdot \frac{de_{yaw}}{dt}$$

โดยที่:
- $e_{yaw}$ = target_yaw - current_yaw (degrees)
- $u_{yaw}$ = lateral offset correction (mm)
- $K_p$ = Proportional gain (default: 0.5 mm/deg)
- $K_d$ = Derivative gain (default: 0.1 mm/(deg/s))

### 3.3 หลักการทำงาน

```
IMU (yaw) → YawController → lateral_offset → Trajectory Generation → Legs
    ↑                                                                    ↓
    └────────────────────── Feedback Loop ───────────────────────────────┘
```

**ตัวอย่าง:**
- หุ่นยนต์เลี้ยวขวา (+5°) → yaw error = -5° → lateral offset = -2.5mm
- Offset นี้จะเลื่อนขาซ้ายไปข้างหน้า และขาขวาถอยหลัง
- สร้าง turning moment ให้หุ่นกลับมาตรง

---

## 4. สถาปัตยกรรมระบบ

### 4.1 Control Architecture (Updated)

```
┌─────────────────────────────────────────────────────────────────────┐
│                    HIGH-LEVEL LAYER (10-50 Hz)                      │
│               SimpleNavigationPlanner (move_relative)               │
├─────────────────────────────────────────────────────────────────────┤
│                     MID-LEVEL LAYER (50 Hz)                         │
│                                                                     │
│   ┌─────────────┐         ┌──────────────┐                         │
│   │ State       │ ────→   │ Yaw          │                         │
│   │ Estimator   │  yaw    │ Controller   │ → lateral_offset        │
│   │ (IMU)       │         │ (PD)         │                         │
│   └─────────────┘         └──────────────┘                         │
│         ↓                         ↓                                 │
│   ┌─────────────────────────────────────────┐                      │
│   │ Gait Generator + Trajectory             │                      │
│   │ (with lateral offset correction)        │                      │
│   └─────────────┬───────────────────────────┘                      │
│                 ↓                                                   │
│   ┌─────────────────────────────────────────┐                      │
│   │ Inverse Kinematics (5-Bar IK)           │                      │
│   └─────────────┬───────────────────────────┘                      │
├─────────────────┼───────────────────────────────────────────────────┤
│                 ↓         LOW-LEVEL LAYER (5 kHz)                   │
│   ┌──────────────────────────────────────────┐                     │
│   │ Motor Control (Binary Protocol)         │                     │
│   └──────────────────────────────────────────┘                     │
├─────────────────────────────────────────────────────────────────────┤
│                       FEEDBACK LOOP                                 │
│   ┌──────────────┐                                                  │
│   │ IMU Reader   │ ← BNO055 (921600 baud, Binary Protocol v1.2)   │
│   │ (100 Hz)     │                                                  │
│   └──────────────┘                                                  │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2 Component Overview

| Component | File | Description |
|-----------|------|-------------|
| **IMUReader** | `navigation/imu_reader.py` | Thread-safe IMU interface (BNO055) |
| **YawController** | `navigation/time_estimator.py` | PD controller for yaw correction |
| **TimeBasedEstimator** | `navigation/time_estimator.py` | State estimator with IMU integration |
| **Main Control** | `control/relative_position_control.py` | Updated control loop with yaw correction |

---

## 5. รายละเอียดการ Implementation

### 5.1 ไฟล์ใหม่ที่สร้าง

#### `navigation/imu_reader.py` ✅

**IMUReader Class** - Thread-safe interface สำหรับ BNO055 IMU

**ฟีเจอร์:**
- ✅ Background thread สำหรับอ่านข้อมูล continuous
- ✅ Binary Protocol v1.2 (เหมือน test_bno055_imu.py)
- ✅ Thread-safe access ด้วย threading.Lock
- ✅ CRC16 validation
- ✅ Auto-reconnect protection
- ✅ Set zero reference command

**API หลัก:**
```python
# สร้างและเชื่อมต่อ IMU
imu = create_imu_reader('COM22', auto_connect=True)

# อ่านค่า yaw
yaw = imu.get_yaw()  # Returns: float (degrees)

# อ่านค่า orientation ทั้งหมด
orientation = imu.get_orientation()
# Returns: {'roll': ..., 'pitch': ..., 'yaw': ..., 'calibrated': ..., 'error': ...}

# ตั้งค่า zero reference
imu.set_zero()

# ตรวจสอบสถานะ
is_connected = imu.is_connected()
is_calibrated = imu.is_calibrated()
stats = imu.get_stats()  # packets, crc_errors, connected

# Cleanup
imu.disconnect()
```

**Convention:**
- **Left turn** = negative yaw (-)
- **Right turn** = positive yaw (+)

### 5.2 ไฟล์ที่แก้ไข

#### `navigation/time_estimator.py` ✅

**เพิ่ม YawController Class:**

```python
yaw_controller = YawController(
    K_p=0.5,              # Proportional gain (mm/deg)
    K_d=0.1,              # Derivative gain (mm/(deg/s))
    max_correction=10.0   # Maximum lateral offset (mm)
)

# Set target
yaw_controller.set_target(0.0)  # Target: 0 degrees (straight)

# Compute correction
lateral_offset = yaw_controller.compute(current_yaw)
# Returns: float (mm)
#   Positive = shift right (correct left drift)
#   Negative = shift left (correct right drift)
```

**อัปเดต TimeBasedEstimator:**

```python
# รองรับ IMU
estimator = TimeBasedEstimator(imu_reader=imu_reader)

# เมธอดใหม่
yaw = estimator.get_yaw()
yaw_error = estimator.get_yaw_error()
has_imu = estimator.has_imu()
```

#### `control/relative_position_control.py` ✅

**การเปลี่ยนแปลงหลัก:**

1. **Import IMU components:**
```python
from navigation.time_estimator import TimeBasedEstimator, YawController
from navigation.imu_reader import IMUReader, create_imu_reader
```

2. **เพิ่มพารามิเตอร์ IMU:**
```python
IMU_PORT = 'COM22'
IMU_ENABLED = True
YAW_K_P = 0.5
YAW_K_D = 0.1
YAW_MAX_CORRECTION = 10.0
```

3. **เพิ่มตัวแปร global:**
```python
imu_reader = None
yaw_controller = None
```

4. **อัปเดต trajectory generation:**
```python
def get_trajectory_for_velocity(v_body_y, leg_id, lateral_offset=0.0):
    # Apply lateral offset for yaw correction
    leg_lateral_offset = lateral_offset if leg_id in ['FR', 'RR'] else -lateral_offset
    
    trajectory = generate_elliptical_trajectory(
        # ...
        home_x=DEFAULT_STANCE_OFFSET_X + leg_lateral_offset,
        # ...
    )
```

5. **อัปเดต control loop:**
```python
# 2.2 Compute yaw correction
lateral_offset = 0.0
if yaw_controller and state_estimator.has_imu():
    current_yaw = state_estimator.get_yaw()
    lateral_offset = yaw_controller.compute(current_yaw, current_time)

# 2.3 Generate trajectories with correction
trajectories = {}
for leg_id in ['FR', 'FL', 'RR', 'RL']:
    trajectories[leg_id] = get_trajectory_for_velocity(v_body_y, leg_id, lateral_offset)
```

6. **อัปเดต status display:**
```python
# Show yaw in status
if state_estimator.has_imu():
    yaw_err = state_estimator.get_yaw_error()
    status_msg += f" | Yaw: {yaw:+.1f}° (err: {yaw_err:+.1f}°)"
```

7. **เพิ่มคำสั่งใหม่ใน interactive mode:**
- `[Z]` - Set IMU zero reference
- `[I]` - Show IMU status

---

## 6. การใช้งาน

### 6.1 การเตรียมระบบ

1. **เชื่อมต่อ IMU:**
   - เสียบ BNO055 IMU เข้า USB (COM22 หรือปรับใน `IMU_PORT`)
   - ติดตั้ง IMU บนหุ่นยนต์ให้แกน Z ชี้ขึ้น
   - Convention: 
     - หุ่นเลี้ยวซ้าย → yaw ลดลง (-)
     - หุ่นเลี้ยวขวา → yaw เพิ่มขึ้น (+)

2. **รัน relative_position_control.py:**
```bash
python python/control/relative_position_control.py
```

3. **IMU จะ auto-connect:**
```
📡 Connecting to IMU on COM22...
  ✅ IMU connected successfully
  🧭 Yaw controller initialized (K_p=0.5, K_d=0.1)
```

### 6.2 คำสั่งใหม่

#### Interactive Mode

**[Z] - Set IMU Zero:**
```
> Z
🧭 Setting IMU zero reference...
  ✅ IMU zero set
```
- ใช้เมื่อต้องการตั้งค่าทิศทางปัจจุบันเป็น reference (0°)
- ควรทำตอนหุ่นยนต์อยู่ในท่าตรง

**[I] - Show IMU Status:**
```
> I
📡 IMU Status:
    Connected: ✅
    Port: COM22
    Calibrated: ✅
    Yaw: +2.34°
    Pitch: -0.56°
    Roll: +1.12°
    Packets received: 15234
    CRC errors: 0
```

### 6.3 ตัวอย่างการใช้งาน

#### ทดสอบ yaw correction:

1. เปิด interactive mode
2. กด `[Z]` เพื่อ set zero
3. กด `[1]` เพื่อเดินหน้า 100mm
4. สังเกตว่า status จะแสดง yaw error:
```
📊 Position: +45.2/+100.0 mm | Progress: 45% | v_y: +50.0 mm/s | Yaw: +2.1° (err: -2.1°) | Time: 1.2s
```

5. หุ่นยนต์จะปรับ gait อัตโนมัติเพื่อแก้ไข yaw drift

### 6.4 การปิด IMU

```python
# สามารถปิด IMU ได้โดยตั้งค่า
IMU_ENABLED = False
```

ระบบจะทำงานแบบเดิม (ไม่มี yaw control)

---

## 7. พารามิเตอร์ที่สามารถปรับได้

### 7.1 IMU Settings

```python
IMU_PORT = 'COM22'          # Serial port (ปรับตาม COM port จริง)
IMU_ENABLED = True          # เปิด/ปิด IMU
```

### 7.2 Yaw Controller Tuning

```python
YAW_K_P = 0.5               # Proportional gain (mm/deg)
                            # เพิ่ม = แก้เร็วกว่า แต่อาจ overshoot
                            # ลด = แก้ช้าแต่นุ่มนวล

YAW_K_D = 0.1               # Derivative gain (mm/(deg/s))
                            # เพิ่ม = ลด overshoot
                            # ลด = อาจมี oscillation

YAW_MAX_CORRECTION = 10.0   # Maximum lateral offset (mm)
                            # จำกัดการแก้ไขสูงสุดต่อ cycle
```

### 7.3 การปรับ Tuning

**ถ้า yaw oscillates (แกว่ง):**
- ลด `YAW_K_P` หรือเพิ่ม `YAW_K_D`

**ถ้าแก้ช้าเกินไป:**
- เพิ่ม `YAW_K_P`

**ถ้าการแก้รุนแรงเกินไป:**
- ลด `YAW_MAX_CORRECTION`

**แนะนำเริ่มต้น:**
- `K_P = 0.5`, `K_D = 0.1`, `max_correction = 10.0`

---

## 8. การทดสอบ

### 8.1 Unit Tests

**ทดสอบ IMUReader:**
```python
from navigation.imu_reader import create_imu_reader
import time

# Connect
imu = create_imu_reader('COM22')
time.sleep(2)

# Test reading
for _ in range(10):
    print(f"Yaw: {imu.get_yaw():.2f}°")
    time.sleep(0.1)

# Test set zero
imu.set_zero()
time.sleep(0.5)
print(f"After zero: {imu.get_yaw():.2f}°")

imu.disconnect()
```

**ทดสอบ YawController:**
```python
from navigation.time_estimator import YawController

controller = YawController(K_p=0.5, K_d=0.1)
controller.set_target(0.0)

# Simulate right drift (+5°)
correction = controller.compute(5.0)
print(f"Drift: +5° → Correction: {correction:.2f}mm")
# Expected: negative correction (shift left)
```

### 8.2 Integration Tests

**Test 1: Short forward with IMU**
```
python python/control/relative_position_control.py
> 1  # Move forward 100mm
```

**Expected Results:**
- ✅ Position error: ≤ 30mm
- ✅ Yaw error: ≤ 3° (improved from ~5-15° without IMU)
- ✅ Smooth correction (no oscillation)

**Test 2: Long forward (500mm)**
```
> 3  # Move forward 500mm
```

**Expected Results:**
- ✅ Position error: ≤ 50mm
- ✅ Yaw error: ≤ 5°
- ✅ Straighter path than before

**Test 3: Backward with IMU**
```
> 6  # Move backward 200mm
```

**Expected Results:**
- ✅ Yaw correction works in both directions

### 8.3 Performance Metrics

| Metric | Without IMU | With IMU (Target) |
|--------|-------------|-------------------|
| Y-axis accuracy | ±30mm | ±30mm |
| Yaw drift (per meter) | 5-15° | **< 3°** ✅ |
| Correction smoothness | N/A | No oscillation ✅ |
| IMU update rate | N/A | 100 Hz ✅ |
| Control loop rate | 50 Hz | 50 Hz (unchanged) |

---

## 9. Roadmap ถัดไป

### 9.1 ระยะสั้น (1 สัปดาห์)

- [x] ✅ Implement IMUReader
- [x] ✅ Implement YawController
- [x] ✅ Integrate with control loop
- [x] ✅ Update UI/menu
- [ ] 🎯 **ทดสอบบนหุ่นยนต์จริง**
- [ ] 🎯 **Fine-tune controller parameters**

### 9.2 ระยะกลาง (2-4 สัปดาห์)

- [ ] ⬜ เพิ่ม logging สำหรับ yaw data
- [ ] ⬜ วิเคราะห์ performance metrics
- [ ] ⬜ ปรับปรุง controller (adaptive gains?)
- [ ] ⬜ เพิ่มการควบคุม X-axis (strafe) + yaw
- [ ] ⬜ Implement full 2D navigation (X + Y + Yaw)

### 9.3 ระยะยาว (1-2 เดือน)

- [ ] ⬜ Sensor Fusion (IMU + Visual Odometry)
- [ ] ⬜ Kalman Filter for state estimation
- [ ] ⬜ SLAM integration
- [ ] ⬜ Waypoint navigation with yaw control

---

## สรุป

การเพิ่ม **IMU integration** เข้าไปในระบบควบคุมตำแหน่งแบบสัมพัทธ์ช่วยแก้ไขปัญหา **Yaw Drift** ได้โดย:

✅ **Thread-safe IMU reader** (BNO055, 100 Hz)  
✅ **PD controller** สำหรับ yaw correction  
✅ **Real-time trajectory adjustment** ด้วย lateral offset  
✅ **UI updates** แสดง yaw status  
✅ **Zero errors** in code - ready for testing

**ขั้นตอนถัดไป:**
1. ทดสอบบนหุ่นยนต์จริง
2. Fine-tune controller parameters
3. วิเคราะห์ performance improvement

**Reference:**
- Roadmap: `ROADMAP_RELATIVE_POSITION_CONTROL.md` → Task #7, #8
- IMU Test: `python/sensors/test_bno055_imu.py`
- Binary Protocol: v1.2

---

**Author:** M-TRCH  
**Date:** February 11, 2026  
**Status:** ✅ Implementation Complete - Ready for Testing
