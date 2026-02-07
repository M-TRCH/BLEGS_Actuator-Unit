# การประเมินความเป็นไปได้: การควบคุมตำแหน่งแบบสัมพัทธ์ (Relative Position Control)

**โครงการ:** BLEGS Actuator Unit - Quadruped Robot  
**เอกสาร:** การประเมินความเป็นไปได้ในการสร้างระบบควบคุมตาม Hierarchical Architecture  
**วันที่:** 4 กุมภาพันธ์ 2026  
**ผู้จัดทำ:** M-TRCH  
**เวอร์ชัน:** 1.0

---

## 1. บทสรุปผู้บริหาร

การทดลองระบบควบคุมตำแหน่งแบบสัมพัทธ์ (Relative Position Control) บนแกน Y (เดินหน้า-ถอยหลัง) มี **ความเป็นไปได้สูงมาก (90/100)** เนื่องจากโครงสร้างพื้นฐานส่วนใหญ่มีอยู่แล้วในระบบปัจจุบัน

---

## 2. ข้อกำหนดการทดสอบ

### 2.1 ขอบเขตการทดสอบ

| รายการ | ข้อกำหนด |
|--------|----------|
| **แกนการเคลื่อนที่** | แกน Y เท่านั้น (เดินหน้า-ถอยหลัง) |
| **รูปแบบการควบคุม** | Relative Position - เคลื่อนที่จากตำแหน่งปัจจุบัน |
| **การหมุน** | ไม่มี (ไม่เลี้ยว) |
| **State Estimator** | แบบง่าย - ใช้การนับเวลา (Time-based Estimation) |
| **วัตถุประสงค์** | ตรวจสอบความถูกต้องของสถาปัตยกรรม |

### 2.2 ตัวอย่างการใช้งาน

```
คำสั่ง: move_relative(y=+500mm)  
ความหมาย: เดินหน้า 500mm จากตำแหน่งปัจจุบัน

คำสั่ง: move_relative(y=-300mm)  
ความหมาย: ถอยหลัง 300mm จากตำแหน่งปัจจุบัน
```

---

## 3. การเปรียบเทียบกับสถาปัตยกรรมอ้างอิง

### 3.1 สถาปัตยกรรม Hierarchical Control (จากเอกสาร ROBOT_ARCHITECTURE.tex)

```
┌─────────────────────────────────────────────────────────────────┐
│                    HIGH-LEVEL LAYER (10-50 Hz)                  │
│                      Navigation Planner                          │
│     P_target → Position Error → Velocity Command (v_world)      │
├─────────────────────────────────────────────────────────────────┤
│                     MID-LEVEL LAYER (50 Hz)                     │
│                                                                  │
│  ┌────────────────┐    ┌──────────────────────────────────────┐ │
│  │ Gait Generator │ ←  │ v_body (World to Body Transform)     │ │
│  │  - Stance FSM  │    └──────────────────────────────────────┘ │
│  │  - Swing FSM   │                                              │
│  └───────┬────────┘                                              │
│          ↓                                                       │
│  ┌────────────────┐                                              │
│  │ Inverse Kinem. │  Foot Position → Joint Angles (θ1, θ2)      │
│  │  (5-Bar IK)    │                                              │
│  └───────┬────────┘                                              │
├──────────┼──────────────────────────────────────────────────────┤
│          ↓              LOW-LEVEL LAYER (5 kHz)                 │
│  ┌────────────────┐                                              │
│  │ Motor Control  │  PID Position Control → PWM Voltage         │
│  │   (FOC/SVPWM)  │                                              │
│  └───────┬────────┘                                              │
├──────────┼──────────────────────────────────────────────────────┤
│          ↓              FEEDBACK LOOP                            │
│  ┌────────────────┐                                              │
│  │ State Estimator│  Dead Reckoning → P_current                 │
│  │ (Time-based)   │                                              │
│  └────────────────┘                                              │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 สถานะปัจจุบันของแต่ละ Module

| Module | สถานะ | ความสมบูรณ์ | ไฟล์ที่เกี่ยวข้อง |
|--------|-------|------------|------------------|
| **Motor Controller** | ✅ สมบูรณ์ | 100% | `motor_control.cpp`, `pid_controller.h` |
| **5-Bar IK** | ✅ สมบูรณ์ | 100% | `test_quadruped_control.py` (line 928-965) |
| **Gait Generator** | ✅ ใช้งานได้ | 90% | `test_quadruped_control.py` (line 1002-1080) |
| **Navigation Planner** | ⚠️ ต้องสร้างใหม่ | 0% | - |
| **State Estimator** | ⚠️ ต้องสร้างใหม่ | 0% | - |

---

## 4. การวิเคราะห์ส่วนที่ต้องพัฒนา

### 4.1 Navigation Planner (ต้องสร้างใหม่ - แบบง่าย)

**เวอร์ชันเต็ม (จากเอกสาร):**
```python
# Complex World Frame Navigation
e_p = P_target - P_current
v_world = min(v_max, K_p * |e_p|) * (e_p / |e_p|)
v_body = R_z(ψ)^T @ v_world
```

**เวอร์ชันที่ต้องการ (แบบง่าย - แกน Y เท่านั้น):**
```python
# Simple Y-axis Navigation (No Rotation)
class SimpleNavigationPlanner:
    def __init__(self, v_max=50.0, K_p=1.0):
        self.target_y = 0.0          # Target relative distance (mm)
        self.current_y = 0.0         # Estimated current position (mm)
        self.v_max = v_max           # Max velocity (mm/s)
        self.K_p = K_p               # Proportional gain
    
    def set_relative_target(self, delta_y):
        """Set relative movement target"""
        self.target_y = delta_y
        self.current_y = 0.0         # Reset current position
    
    def compute_velocity(self):
        """Compute body velocity command"""
        e_y = self.target_y - self.current_y
        
        # Simple proportional control with saturation
        v_y = min(self.v_max, self.K_p * abs(e_y)) * np.sign(e_y)
        
        return v_y  # mm/s (positive = forward)
    
    def is_target_reached(self, tolerance=10.0):
        """Check if target is reached"""
        return abs(self.target_y - self.current_y) < tolerance
```

**ความซับซ้อน:** ⭐ (ต่ำมาก)  
**เวลาที่ต้องใช้:** 2-4 ชั่วโมง

### 4.2 State Estimator (ต้องสร้างใหม่ - แบบง่าย)

**เวอร์ชันเต็ม (จากเอกสาร):**
```python
# Sensor Fusion with IMU + Odometry
P_k+1 = P_k + (R_k(ψ) @ v_body_k) * Δt
# Requires: IMU data, Kalman Filter, Odometry from leg kinematics
```

**เวอร์ชันที่ต้องการ (แบบง่าย - Time-based):**
```python
# Simple Time-based Position Estimation
class TimeBasedEstimator:
    def __init__(self):
        self.position_y = 0.0        # Estimated Y position (mm)
        self.last_update_time = 0.0
    
    def update(self, v_body_y, current_time):
        """Update position estimate using velocity and time"""
        if self.last_update_time > 0:
            dt = current_time - self.last_update_time
            self.position_y += v_body_y * dt
        
        self.last_update_time = current_time
    
    def reset(self):
        """Reset position to zero (new movement command)"""
        self.position_y = 0.0
        self.last_update_time = 0.0
    
    def get_position(self):
        """Get estimated Y position"""
        return self.position_y
```

**ความซับซ้อน:** ⭐ (ต่ำมาก)  
**เวลาที่ต้องใช้:** 1-2 ชั่วโมง

### 4.3 การปรับปรุง Gait Generator

**สิ่งที่มีอยู่แล้ว:**
- Elliptical trajectory generation ✅
- Trot gait pattern ✅
- Stance/Swing phase FSM ✅
- Forward/Backward support (parameter `reverse`) ✅

**สิ่งที่ต้องเพิ่ม:**
```python
# การเชื่อมต่อ Velocity Command กับ Gait Generator
def update_gait_from_velocity(v_body_y):
    """
    แปลง body velocity เป็น gait parameters
    
    Args:
        v_body_y: Body frame Y velocity (mm/s)
                  positive = forward, negative = backward
    """
    # คำนวณ step length จาก velocity
    # step_length ∝ v_body * gait_cycle_time
    gait_cycle_time = TRAJECTORY_STEPS / UPDATE_RATE  # seconds
    step_length = v_body_y * gait_cycle_time
    
    # จำกัด step length
    step_length = np.clip(step_length, -GAIT_STEP_FORWARD, GAIT_STEP_FORWARD)
    
    # กำหนดทิศทาง
    reverse = (v_body_y < 0)
    
    return step_length, reverse
```

**ความซับซ้อน:** ⭐⭐ (ต่ำ)  
**เวลาที่ต้องใช้:** 2-3 ชั่วโมง

---

## 5. แผนการดำเนินงาน

### 5.1 ขั้นตอนการพัฒนา

```
Phase 1: สร้าง Core Components (4-6 ชั่วโมง)
├── Task 1.1: สร้าง SimpleNavigationPlanner class
├── Task 1.2: สร้าง TimeBasedEstimator class
└── Task 1.3: สร้างฟังก์ชัน update_gait_from_velocity()

Phase 2: Integration (2-3 ชั่วโมง)
├── Task 2.1: เชื่อมต่อ Navigation → Gait Generator
├── Task 2.2: เชื่อมต่อ State Estimator → Navigation Planner
└── Task 2.3: สร้าง Control Loop หลัก

Phase 3: Testing (2-4 ชั่วโมง)
├── Task 3.1: ทดสอบ move_relative(+100mm) - เดินหน้าสั้น
├── Task 3.2: ทดสอบ move_relative(-100mm) - ถอยหลังสั้น
├── Task 3.3: ทดสอบ move_relative(+500mm) - เดินหน้าระยะไกล
└── Task 3.4: ปรับ tuning parameters (K_p, v_max)
```

### 5.2 ไทม์ไลน์โดยประมาณ

| ขั้นตอน | เวลาโดยประมาณ | ความยาก |
|---------|--------------|---------|
| Phase 1 | 4-6 ชั่วโมง | ⭐⭐ ต่ำ |
| Phase 2 | 2-3 ชั่วโมง | ⭐⭐ ต่ำ |
| Phase 3 | 2-4 ชั่วโมง | ⭐⭐⭐ ปานกลาง |
| **รวม** | **8-13 ชั่วโมง** | - |

---

## 6. โครงสร้างโค้ดที่แนะนำ

### 6.1 ไฟล์ใหม่ที่ต้องสร้าง

```
tools/
├── test_quadruped_control.py        # (มีอยู่แล้ว)
├── relative_position_control.py     # (ใหม่) - ระบบควบคุมตำแหน่งสัมพัทธ์
└── navigation/                      # (ใหม่) - โฟลเดอร์สำหรับ navigation modules
    ├── __init__.py
    ├── simple_planner.py            # Navigation Planner แบบง่าย
    └── time_estimator.py            # State Estimator แบบ time-based
```

### 6.2 โครงสร้าง Main Control Loop

```python
# relative_position_control.py - โครงสร้างหลัก

from navigation.simple_planner import SimpleNavigationPlanner
from navigation.time_estimator import TimeBasedEstimator

# Initialize components
nav_planner = SimpleNavigationPlanner(v_max=50.0, K_p=1.0)
state_estimator = TimeBasedEstimator()

def move_relative_y(target_distance_mm, timeout_s=30.0):
    """
    เคลื่อนที่แบบ relative บนแกน Y
    
    Args:
        target_distance_mm: ระยะทางที่ต้องการเคลื่อนที่ (mm)
                           positive = เดินหน้า
                           negative = ถอยหลัง
        timeout_s: เวลา timeout สูงสุด (วินาที)
    
    Returns:
        bool: True ถ้าถึงเป้าหมาย, False ถ้า timeout
    """
    # 1. ตั้งค่าเป้าหมาย
    nav_planner.set_relative_target(target_distance_mm)
    state_estimator.reset()
    
    start_time = time.time()
    
    # 2. Control Loop
    while not nav_planner.is_target_reached():
        current_time = time.time()
        
        # Check timeout
        if current_time - start_time > timeout_s:
            print("⚠️ Timeout reached!")
            return False
        
        # 2.1 คำนวณ velocity command
        v_body_y = nav_planner.compute_velocity()
        
        # 2.2 แปลงเป็น gait parameters
        step_length, reverse = update_gait_from_velocity(v_body_y)
        
        # 2.3 อัพเดท gait และส่งคำสั่งไปยัง motors
        update_all_legs_gait(step_length, reverse)
        
        # 2.4 อัพเดท state estimator
        state_estimator.update(v_body_y, current_time)
        nav_planner.current_y = state_estimator.get_position()
        
        # 2.5 รอ next cycle
        time.sleep(1.0 / UPDATE_RATE)
    
    # 3. หยุดเดิน
    stop_all_motors()
    
    print(f"✅ Target reached! Distance: {state_estimator.get_position():.1f} mm")
    return True
```

---

## 7. ข้อพิจารณาและข้อจำกัด

### 7.1 ข้อจำกัดของ Time-based Estimation

| ข้อจำกัด | ผลกระทบ | แนวทางแก้ไข |
|----------|---------|------------|
| **Drift Error** | ตำแหน่งประมาณการจะ drift ตามเวลา | จำกัดระยะทางต่อครั้ง (<1m) |
| **ไม่มี Feedback** | ไม่รู้ว่ามอเตอร์เลื่อนไถลหรือไม่ | ใช้สำหรับทดสอบเท่านั้น |
| **ความแม่นยำต่ำ** | อาจคลาดเคลื่อน 10-20% | ยอมรับได้สำหรับการทดสอบ |

### 7.2 ข้อจำกัดของการเคลื่อนที่แกน Y เดียว

- ❌ ไม่สามารถเลี้ยวได้
- ❌ ไม่สามารถเดินข้าง (strafe) ได้
- ❌ ไม่มีการแก้ไขทิศทาง (yaw correction)
- ✅ เหมาะสำหรับการทดสอบ architecture เบื้องต้น

### 7.3 พารามิเตอร์ที่ต้อง Tune

```python
# Navigation Planner
v_max = 50.0      # mm/s - ความเร็วสูงสุด (ลดลงถ้าไม่เสถียร)
K_p = 1.0         # Proportional gain (ปรับตาม response)
tolerance = 10.0  # mm - ความละเอียดการหยุด

# Gait Parameters (มีอยู่แล้ว)
GAIT_LIFT_HEIGHT = 15.0    # mm
GAIT_STEP_FORWARD = 50.0   # mm
UPDATE_RATE = 50           # Hz
```

---

## 8. สรุปและข้อเสนอแนะ

### 8.1 สรุปความเป็นไปได้

| เกณฑ์ | คะแนน | หมายเหตุ |
|-------|-------|----------|
| **ความพร้อมของ Infrastructure** | 9/10 | Low-level control พร้อมแล้ว |
| **ความซับซ้อนที่ต้องเพิ่ม** | 8/10 | ต้องเพิ่มแค่ 2 class ง่ายๆ |
| **ความเสี่ยงทางเทคนิค** | 9/10 | ใช้หลักการที่พิสูจน์แล้ว |
| **เวลาที่ต้องใช้** | 8/10 | 8-13 ชั่วโมง |
| **รวม** | **90/100** | **ความเป็นไปได้สูงมาก** |

### 8.2 ข้อเสนอแนะ

1. **เริ่มจากระยะสั้น**: ทดสอบ `move_relative(+100mm)` ก่อน แล้วค่อยเพิ่มระยะทาง
2. **ใช้ความเร็วต่ำ**: เริ่มจาก `v_max = 30 mm/s` แล้วค่อยเพิ่ม
3. **บันทึก Log**: เก็บข้อมูล estimated position vs actual (วัดด้วยตา) เพื่อปรับ tuning
4. **พร้อมสำหรับขั้นต่อไป**: หลังทดสอบสำเร็จ สามารถเพิ่ม:
   - การเคลื่อนที่แกน X (เดินข้าง)
   - การหมุน (yaw control)
   - State Estimator แบบ sensor fusion (ถ้ามี IMU)

### 8.3 ขั้นตอนถัดไป

เมื่อได้รับการอนุมัติ ฉันจะดำเนินการ:
1. สร้างไฟล์ `relative_position_control.py`
2. สร้าง navigation module classes
3. ทดสอบและปรับ tuning

---

## ภาคผนวก A: สูตรทางคณิตศาสตร์

### A.1 Position Error (แบบง่าย - แกน Y เดียว)

$$e_y = y_{target} - y_{current}$$

### A.2 Velocity Command (Proportional Control with Saturation)

$$v_y = \min(v_{max}, K_p |e_y|) \cdot \text{sign}(e_y)$$

### A.3 Position Update (Dead Reckoning)

$$y_{k+1} = y_k + v_{y,k} \cdot \Delta t$$

### A.4 Target Reached Condition

$$|e_y| < \epsilon_{tolerance}$$

---

## ภาคผนวก B: Dependency ที่ต้องใช้

```python
# Python packages (มีอยู่แล้วใน project)
import numpy as np
import time
import threading
import serial

# Internal modules (มีอยู่แล้ว)
from test_quadruped_control import (
    BinaryMotorController,
    calculate_ik_no_ef,
    generate_elliptical_trajectory,
    # ... other imports
)
```

---

*เอกสารนี้จัดทำเพื่อประเมินความเป็นไปได้ในการสร้างระบบควบคุมตำแหน่งแบบสัมพัทธ์ตามสถาปัตยกรรม Hierarchical Control*
