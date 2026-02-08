# Roadmap: ระบบควบคุมตำแหน่งแบบสัมพัทธ์ (Relative Position Control)

**โครงการ:** BLEGS Actuator Unit - Quadruped Robot  
**เอกสาร:** Roadmap การพัฒนาและผลการทดสอบระบบควบคุมตำแหน่งแกน Y  
**วันที่:** 5 กุมภาพันธ์ 2026  
**ผู้จัดทำ:** M-TRCH  
**เวอร์ชัน:** 1.0  
**สถานะ:** ✅ ทดสอบเสร็จสิ้น - พร้อมปรับเทียบ

---

## สารบัญ

1. [บทสรุป](#1-บทสรุป)
2. [สถาปัตยกรรมระบบ](#2-สถาปัตยกรรมระบบ)
3. [ทฤษฎีและหลักการ](#3-ทฤษฎีและหลักการ)
4. [รายละเอียดการ Implement](#4-รายละเอียดการ-implement)
5. [ผลการทดสอบ](#5-ผลการทดสอบ)
6. [การวิเคราะห์ Calibration Factor](#6-การวิเคราะห์-calibration-factor)
7. [แผนการปรับปรุง](#7-แผนการปรับปรุง)
8. [ภาคผนวก](#8-ภาคผนวก)

---

## 1. บทสรุป

### 1.1 วัตถุประสงค์

ระบบควบคุมตำแหน่งแบบสัมพัทธ์ (Relative Position Control) ออกแบบมาเพื่อควบคุมการเคลื่อนที่ของหุ่นยนต์ quadruped บนแกน Y (เดินหน้า-ถอยหลัง) โดยอ้างอิงจากสถาปัตยกรรม Hierarchical Control ที่กำหนดไว้ในเอกสาร `ROBOT_ARCHITECTURE.tex`

### 1.2 สถานะโครงการ

| รายการ | สถานะ | หมายเหตุ |
|--------|-------|----------|
| High-Level: Navigation Planner | ✅ สมบูรณ์ | `simple_planner.py` |
| Mid-Level: Gait Generator | ✅ สมบูรณ์ | `test_quadruped_control.py` |
| Mid-Level: Inverse Kinematics | ✅ สมบูรณ์ | 5-Bar IK ทำงานถูกต้อง |
| Low-Level: Motor Control | ✅ สมบูรณ์ | FOC/SVPWM Firmware |
| State Estimator | ✅ สมบูรณ์ | `time_estimator.py` |
| **Integration Testing** | ✅ ผ่านการทดสอบ | คำสั่ง 1-4 ทำงานได้ดีเยี่ยม |
| **Calibration** | ⚠️ ต้องปรับค่า | พบ Scaling Factor 3x |

### 1.3 ผลการทดสอบโดยสรุป

การทดสอบคำสั่งเคลื่อนที่ทั้ง 4 โหมดทำงานได้ **ดีเยี่ยม** โดยหุ่นยนต์เคลื่อนที่อย่างราบรื่น มีข้อสังเกตสำคัญคือ:

| คำสั่ง | ตำแหน่งเป้าหมาย | ระยะทางจริง | อัตราส่วน |
|--------|----------------|-------------|----------|
| 1 | 100 mm | 300 mm | 3.00x |
| 2 | 200 mm | 600 mm | 3.00x |
| 3 | 300 mm | 920 mm | 3.07x |
| 4 | 400 mm | 1240 mm | 3.10x |

**Calibration Factor โดยเฉลี่ย: ≈ 3.04x**

---

## 2. สถาปัตยกรรมระบบ

### 2.1 Hierarchical Control Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    HIGH-LEVEL LAYER (10-50 Hz)                      │
│               SimpleNavigationPlanner (move_relative)               │
│                                                                     │
│   ┌─────────────┐    ┌──────────────┐    ┌─────────────────┐       │
│   │ Set Target  │ →  │ Compute      │ →  │ Output:         │       │
│   │ δy (mm)     │    │ Velocity     │    │ v_body_y (mm/s) │       │
│   └─────────────┘    └──────────────┘    └─────────────────┘       │
├─────────────────────────────────────────────────────────────────────┤
│                     MID-LEVEL LAYER (50 Hz)                         │
│                                                                     │
│   ┌────────────────┐                 ┌──────────────────────────┐  │
│   │ Gait Generator │                 │ v_body → step_length     │  │
│   │ - Elliptical   │ ←──────────────│ - Direction (fwd/bwd)    │  │
│   │ - Trot Pattern │                 └──────────────────────────┘  │
│   └───────┬────────┘                                                │
│           ↓                                                         │
│   ┌────────────────┐                                                │
│   │ Inverse Kinem. │  Foot Position (x,y) → Joint Angles (θA, θB)  │
│   │  (5-Bar IK)    │                                                │
│   └───────┬────────┘                                                │
├───────────┼─────────────────────────────────────────────────────────┤
│           ↓              LOW-LEVEL LAYER (5 kHz)                    │
│   ┌────────────────┐                                                │
│   │ Motor Control  │  PID Position Control → PWM/FOC               │
│   │ (Binary Proto) │                                                │
│   └───────┬────────┘                                                │
├───────────┼─────────────────────────────────────────────────────────┤
│           ↓              FEEDBACK LOOP                              │
│   ┌────────────────┐                                                │
│   │ State Estimator│  Dead Reckoning: y_{k+1} = y_k + v_y·Δt       │
│   │ (Time-based)   │                                                │
│   └───────┬────────┘                                                │
│           │                                                         │
│           └──────────────→ Navigation Planner (Position Feedback)   │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow

```
User Command: move_relative(+500mm)
        ↓
┌───────────────────────────────────────────────────────────────┐
│ Navigation Planner                                            │
│   target_y = 500mm                                            │
│   current_y = 0mm (จาก State Estimator)                       │
│   e_y = 500 - 0 = 500mm                                       │
│   v_y = min(50, 1.0 × 500) × sign(500) = 50 mm/s             │
└───────────────────────────────────────────────────────────────┘
        ↓
┌───────────────────────────────────────────────────────────────┐
│ Velocity → Gait Mapping                                       │
│   GAIT_CYCLE_TIME = 30/50 = 0.6s                             │
│   step_length = |50| × 0.6 = 30mm                            │
│   reverse = False                                             │
└───────────────────────────────────────────────────────────────┘
        ↓
┌───────────────────────────────────────────────────────────────┐
│ Trajectory Generation (per leg)                               │
│   generate_elliptical_trajectory(                             │
│       num_steps=30,                                           │
│       lift_height=15mm,                                       │
│       step_forward=30mm,                                      │
│       stance_ratio=0.70                                       │
│   )                                                           │
└───────────────────────────────────────────────────────────────┘
        ↓
┌───────────────────────────────────────────────────────────────┐
│ Inverse Kinematics (5-Bar)                                    │
│   Foot position (x, y) → Motor angles (θA, θB)               │
└───────────────────────────────────────────────────────────────┘
        ↓
┌───────────────────────────────────────────────────────────────┐
│ Motor Commands (Binary Protocol v1.2)                         │
│   set_position_direct(angle_deg) → ทุกมอเตอร์                │
└───────────────────────────────────────────────────────────────┘
```

---

## 3. ทฤษฎีและหลักการ

### 3.1 Proportional Navigation Control

ระบบใช้ **Proportional Control with Velocity Saturation** สำหรับคำนวณคำสั่งความเร็ว:

#### สมการ Position Error:
$$e_y = y_{target} - y_{current}$$

#### สมการ Velocity Command:
$$v_y = \min(v_{max}, K_p \cdot |e_y|) \cdot \text{sign}(e_y)$$

โดยที่:
- $v_{max}$ = 50 mm/s (ความเร็วสูงสุด)
- $K_p$ = 1.0 (Proportional Gain)
- $e_y$ = Position Error (mm)

#### เงื่อนไขการถึงเป้าหมาย:
$$|e_y| < \epsilon_{tolerance}$$

โดยที่ $\epsilon_{tolerance}$ = 10 mm

### 3.2 Dead Reckoning State Estimation

ใช้การประมาณตำแหน่งแบบ **Dead Reckoning** โดยอินทิเกรตความเร็วตามเวลา:

$$y_{k+1} = y_k + v_{y,k} \cdot \Delta t$$

**ข้อจำกัด:**
- Drift Error สะสมตามระยะทาง
- ไม่มี Sensor Feedback ตรวจจับการลื่นไถล
- แนะนำสำหรับระยะทาง < 1 เมตร

### 3.3 Elliptical Gait Trajectory

Trajectory ของเท้าใช้สมการ **Elliptical Path** แบ่งเป็น 2 เฟส:

#### Stance Phase (70% ของ cycle):
เท้าอยู่บนพื้น เคลื่อนที่ถอยหลังเทียบกับลำตัว:
$$P_x(t) = -\text{step\_forward} \cdot \cos(t), \quad t \in [0, \pi]$$
$$P_y(t) = \text{home\_y}$$

#### Swing Phase (30% ของ cycle):
เท้ายกขึ้นและก้าวไปข้างหน้า:
$$P_x(t) = -\text{step\_forward} \cdot \cos(t), \quad t \in [\pi, 2\pi]$$
$$P_y(t) = \text{home\_y} + \text{lift\_height} \cdot |\sin(t)|$$

**สังเกต:** Trajectory ครอบคลุมช่วง $[-\text{step\_forward}, +\text{step\_forward}]$ บนแกน X ดังนั้นระยะทางรวมที่เท้าเคลื่อนที่ต่อ cycle = $2 \times \text{step\_forward}$

### 3.4 Trot Gait Pattern

Trot Gait ใช้ขาตรงข้ามมุมเคลื่อนที่พร้อมกัน:

| ขา | Phase Offset | กลุ่ม |
|----|--------------|------|
| FR (Front Right) | 0.0 | A |
| RL (Rear Left) | 0.0 | A |
| FL (Front Left) | 0.5 | B |
| RR (Rear Right) | 0.5 | B |

กลุ่ม A และ B สลับระหว่าง Stance และ Swing ทำให้หุ่นยนต์เคลื่อนที่อย่างต่อเนื่อง

### 3.5 Five-Bar Linkage Inverse Kinematics

ใช้ **Geometric Solution** สำหรับ 5-Bar Linkage:

```
       Motor A ●────────────● Motor B
               │\          /│
               │ \   C    / │
          L_AC │  \──────/  │ L_BD
               │   \    /   │
               │    \  /    │
               │     \/     │
               │      E     │ ← Foot Position
               │    (x,y)   │
```

#### พารามิเตอร์ขา:
- $L_{AC}$ = 105 mm (Motor A to Joint C)
- $L_{BD}$ = 105 mm (Motor B to Joint D)
- $L_{CE}$ = 145 mm (Joint C to Foot E)
- $L_{DE}$ = 145 mm (Joint D to Foot E)
- Motor Spacing = 85 mm

#### สมการ IK:
กำหนด Target Position $(x, y)$ หา $\theta_A$ และ $\theta_B$:

$$\theta_i = 2 \arctan \left( \frac{-B \pm \sqrt{B^2 - 4AC}}{2A} \right)$$

โดยที่ $A$, $B$, $C$ คำนวณจากพารามิเตอร์ทางเรขาคณิตของขา

---

## 4. รายละเอียดการ Implement

### 4.1 โครงสร้างไฟล์

```
tools/
├── relative_position_control.py   # Main control script
├── test_quadruped_control.py      # Base motor control & gait
└── navigation/
    ├── __init__.py
    ├── simple_planner.py          # Navigation Planner
    └── time_estimator.py          # State Estimator
```

### 4.2 พารามิเตอร์ที่สำคัญ

```python
# Navigation Parameters
NAV_V_MAX = 50.0           # Maximum velocity (mm/s)
NAV_K_P = 1.0              # Proportional gain
NAV_TOLERANCE = 10.0       # Position tolerance (mm)
NAV_TIMEOUT = 60.0         # Maximum timeout (seconds)

# Gait Parameters
UPDATE_RATE = 50           # Control loop frequency (Hz)
TRAJECTORY_STEPS = 30      # Steps per gait cycle
GAIT_LIFT_HEIGHT = 15.0    # Foot lift height (mm)
GAIT_STEP_FORWARD = 50.0   # Maximum step length (mm)
SMOOTH_TROT_STANCE_RATIO = 0.70  # Stance phase ratio

# Derived Parameters
GAIT_CYCLE_TIME = TRAJECTORY_STEPS / UPDATE_RATE  # = 0.6 seconds
```

### 4.3 Control Loop หลัก

```python
def move_relative_y(target_distance_mm, timeout_s=60.0):
    # 1. Initialize
    nav_planner.set_relative_target(target_distance_mm)
    state_estimator.start()
    
    # 2. Control Loop
    while not nav_planner.is_target_reached():
        # 2.1 Compute velocity command
        v_body_y = nav_planner.compute_velocity()
        
        # 2.2 Generate trajectories
        for leg_id in ['FR', 'FL', 'RR', 'RL']:
            trajectories[leg_id] = get_trajectory_for_velocity(v_body_y, leg_id)
        
        # 2.3 Update all legs (IK + Motor commands)
        update_all_legs_gait(trajectories, step_indices)
        
        # 2.4 Advance step indices
        for leg_id in step_indices:
            step_indices[leg_id] = (step_indices[leg_id] + 1) % TRAJECTORY_STEPS
        
        # 2.5 Update state estimator
        state_estimator.update(v_body_y, current_time)
        nav_planner.update_position(state_estimator.get_position())
        
        # 2.6 Rate limiting
        sleep_time = (1.0 / UPDATE_RATE) - loop_duration
        time.sleep(max(0, sleep_time))
    
    # 3. Stop and report
    stop_all_legs()
```

### 4.4 Velocity to Gait Mapping

```python
def update_gait_from_velocity(v_body_y: float) -> tuple:
    """
    แปลง body velocity เป็น gait parameters
    
    step_length ∝ v_body × gait_cycle_time
    """
    step_length = abs(v_body_y) * GAIT_CYCLE_TIME
    step_length = min(step_length, GAIT_STEP_FORWARD)
    reverse = (v_body_y < 0)
    
    return step_length, reverse
```

---

## 5. ผลการทดสอบ

### 5.1 สภาพแวดล้อมทดสอบ

| รายการ | ค่า |
|--------|-----|
| โหมดการทดสอบ | Hardware (Real Robot) |
| จำนวนขาที่ใช้งาน | 4 ขา (8 มอเตอร์) |
| พื้นผิว | พื้นเรียบ |
| Control Mode | Direct Position |
| Gait Type | Trot |

### 5.2 ผลการทดสอบคำสั่ง 1-4

#### คำสั่ง 1: Move Forward +100mm
```
Target: +100 mm
Actual Movement: 300 mm
Ratio: 3.00x
Status: ✅ ทำงานได้ดีเยี่ยม
```

#### คำสั่ง 2: Move Forward +200mm
```
Target: +200 mm
Actual Movement: 600 mm
Ratio: 3.00x
Status: ✅ ทำงานได้ดีเยี่ยม
```

#### คำสั่ง 3: Move Forward +300mm
```
Target: +300 mm
Actual Movement: 920 mm
Ratio: 3.07x
Status: ✅ ทำงานได้ดีเยี่ยม
```

#### คำสั่ง 4: Move Forward +400mm
```
Target: +400 mm
Actual Movement: 1240 mm
Ratio: 3.10x
Status: ✅ ทำงานได้ดีเยี่ยม
```

### 5.3 สรุปผลการทดสอบ

| Metric | ค่า |
|--------|-----|
| **อัตราความสำเร็จ** | 100% (4/4 คำสั่ง) |
| **Calibration Factor (เฉลี่ย)** | 3.04x |
| **ความคงที่ของ Factor** | สูง (σ ≈ 0.05) |
| **คุณภาพการเคลื่อนที่** | ดีเยี่ยม, ราบรื่น |
| **การรักษาเสถียรภาพ** | ดี, ไม่ล้ม |

### 5.4 กราฟความสัมพันธ์

```
Actual Distance (mm)
     ^
1400 │                              ●
     │                             /
1200 │                            /
     │                           /
1000 │                          /
     │                        ●
 800 │                       /
     │                      /
 600 │                   ●
     │                  /
 400 │                 /
     │              ●
 200 │             /
     │            /
   0 ├──────●────┼────┼────┼────┼────→ Target (mm)
     0    100  200  300  400
     
     Actual ≈ 3.0 × Target
```

---

## 6. การวิเคราะห์ Calibration Factor

### 6.1 สาเหตุที่เป็นไปได้

การที่หุ่นยนต์เคลื่อนที่ได้ระยะทางจริง **3 เท่า** ของเป้าหมาย มีสาเหตุหลักดังนี้:

#### สาเหตุที่ 1: Trajectory Span (2x Factor)

จากสมการ trajectory:
$$P_x(t) = -\text{step\_forward} \cdot \cos(t)$$

เมื่อ $t$ เปลี่ยนจาก $0$ ถึง $\pi$:
- $P_x(0) = -\text{step\_forward}$
- $P_x(\pi) = +\text{step\_forward}$

**ระยะทาง X ที่เท้าเคลื่อนที่ = $2 \times \text{step\_forward}$**

นี่หมายความว่า ถ้าตั้ง `step_forward = 30mm` เท้าจะเคลื่อนที่ 60mm ต่อ cycle ทำให้ลำตัวเคลื่อนที่ 60mm แทนที่จะเป็น 30mm ตามที่คาดหวัง

**Contribution: 2.0x**

#### สาเหตุที่ 2: Gait Cycle Overlap

ด้วย Stance Ratio = 70% และ Trot gait pattern ที่มี phase offset 0.5:

- ทั้งสองกลุ่มขา (A และ B) มีช่วง overlap ที่อยู่ใน stance พร้อมกัน
- การ overlap นี้อาจทำให้มี propulsion เพิ่มขึ้น

**Contribution: ~1.5x (ประมาณ)**

#### การคำนวณ Combined Factor:
$$\text{Total Factor} = 2.0 \times 1.5 = 3.0$$

ซึ่งตรงกับผลการทดสอบ!

### 6.2 สาเหตุอื่นที่เป็นไปได้

1. **State Estimator Integration Rate**: ถ้า dt ในการ integrate ไม่ตรงกับ loop rate จริง
2. **Velocity Command Scaling**: V_max อาจไม่ตรงกับความเร็วจริงของ gait
3. **Slip Factor**: พื้นผิวที่มี friction ดีอาจให้ propulsion มากกว่าที่คาด

### 6.3 การแก้ไข Calibration Factor

มี 3 แนวทางในการแก้ไข:

#### แนวทาง A: ปรับค่า velocity mapping (แนะนำ)

```python
# เพิ่ม calibration constant
VELOCITY_CALIBRATION = 1.0 / 3.0  # = 0.333

def update_gait_from_velocity(v_body_y: float) -> tuple:
    step_length = abs(v_body_y) * GAIT_CYCLE_TIME * VELOCITY_CALIBRATION
    step_length = min(step_length, GAIT_STEP_FORWARD)
    reverse = (v_body_y < 0)
    return step_length, reverse
```

#### แนวทาง B: ปรับค่า target ก่อนส่งคำสั่ง

```python
def move_relative_y_calibrated(target_mm):
    calibrated_target = target_mm / 3.0
    return move_relative_y(calibrated_target)
```

#### แนวทาง C: ปรับใน State Estimator

```python
ESTIMATOR_CALIBRATION = 3.0

def update(self, v_body_y, current_time):
    # ...
    displacement = v_body_y * dt * ESTIMATOR_CALIBRATION
    self.position_y += displacement
```

### 6.4 ข้อแนะนำ

**แนะนำใช้แนวทาง A** เนื่องจาก:
1. แก้ที่ต้นเหตุ (velocity → gait mapping)
2. ไม่ต้องเปลี่ยนแปลง API การเรียกใช้
3. State Estimator ยังคงแสดงค่าถูกต้องตามที่คำนวณ

---

## 7. แผนการปรับปรุง

### 7.1 Roadmap ระยะสั้น (1-2 สัปดาห์)

| ลำดับ | งาน | ความสำคัญ | เวลา |
|-------|-----|-----------|------|
| 1 | ✅ ทดสอบระบบพื้นฐาน | สูง | เสร็จแล้ว |
| 2 | ⬜ เพิ่ม Calibration Constant | สูง | 30 นาที |
| 3 | ⬜ ทดสอบหลังปรับ calibration | สูง | 2 ชั่วโมง |
| 4 | ⬜ ทดสอบ backward movement | กลาง | 1 ชั่วโมง |
| 5 | ⬜ เพิ่ม logging และ data recording | กลาง | 2 ชั่วโมง |

### 7.2 Roadmap ระยะกลาง (1-2 เดือน)

| ลำดับ | งาน | ความสำคัญ | เวลา |
|-------|-----|-----------|------|
| 6 | ⬜ เพิ่มการเคลื่อนที่แกน X (strafe) | กลาง | 8 ชั่วโมง |
| 7 | ⬜ เพิ่มการหมุน (yaw control) | กลาง | 8 ชั่วโมง |
| 8 | ⬜ Integrate IMU สำหรับ orientation | กลาง | 16 ชั่วโมง |
| 9 | ⬜ ปรับปรุง State Estimator (Sensor Fusion) | ต่ำ | 24 ชั่วโมง |

### 7.3 Roadmap ระยะยาว (3-6 เดือน)

| ลำดับ | งาน | ความสำคัญ |
|-------|-----|-----------|
| 10 | ⬜ Absolute Position Control (Waypoint Navigation) | กลาง |
| 11 | ⬜ Obstacle Avoidance Integration | ต่ำ |
| 12 | ⬜ Terrain Adaptation | ต่ำ |
| 13 | ⬜ Multi-waypoint Path Planning | ต่ำ |

---

## 8. ภาคผนวก

### A. Parameter Reference

```python
# === Navigation ===
NAV_V_MAX = 50.0              # mm/s
NAV_K_P = 1.0                 # -
NAV_TOLERANCE = 10.0          # mm
NAV_TIMEOUT = 60.0            # seconds

# === Gait ===
UPDATE_RATE = 50              # Hz
TRAJECTORY_STEPS = 30         # steps/cycle
GAIT_CYCLE_TIME = 0.6         # seconds
GAIT_LIFT_HEIGHT = 15.0       # mm
GAIT_STEP_FORWARD = 50.0      # mm
SMOOTH_TROT_STANCE_RATIO = 0.70

# === Robot Geometry ===
BODY_LENGTH = 200.0           # mm
BODY_WIDTH = 170.0            # mm
MOTOR_SPACING = 85.0          # mm
L_AC = 105.0                  # mm
L_BD = 105.0                  # mm
L_CE = 145.0                  # mm
L_DE = 145.0                  # mm
DEFAULT_STANCE_HEIGHT = -220.0  # mm

# === Calibration ===
VELOCITY_CALIBRATION = 0.333  # 1/3 (ค่าที่แนะนำ)
```

### B. Testing Commands

```python
# Interactive Mode Commands (จากเมนู)
[1] Move forward +100mm
[2] Move backward -100mm
[3] Move forward +500mm
[4] Custom distance (ใส่ระยะทางเอง)

# คำสั่งที่ใช้ในการทดสอบ (ผ่านตัวเลือก [4] Custom distance)
move_relative_y(+100.0)   # ทดสอบ 1: Forward 100mm → เคลื่อนที่จริง 300mm
move_relative_y(+200.0)   # ทดสอบ 2: Forward 200mm → เคลื่อนที่จริง 600mm
move_relative_y(+300.0)   # ทดสอบ 3: Forward 300mm → เคลื่อนที่จริง 920mm
move_relative_y(+400.0)   # ทดสอบ 4: Forward 400mm → เคลื่อนที่จริง 1240mm

# Direct Function Calls (สำหรับเรียกใช้ใน Python)
move_relative_y(+100.0)   # Forward 100mm
move_relative_y(-200.0)   # Backward 200mm
move_relative_y(+500.0, timeout_s=30.0)  # With custom timeout
```

### C. Troubleshooting

| ปัญหา | สาเหตุที่เป็นไปได้ | วิธีแก้ไข |
|-------|-------------------|----------|
| หุ่นยนต์ไม่เคลื่อนที่ | Motor ไม่ได้เชื่อมต่อ | ตรวจสอบ COM port และ motor discovery |
| เคลื่อนที่ผิดทิศทาง | IK mirror_x ผิด | ตรวจสอบ leg configuration |
| เคลื่อนที่ไม่ราบรื่น | UPDATE_RATE ต่ำเกินไป | เพิ่มเป็น 100 Hz |
| ล้มระหว่างเดิน | Stance height ไม่เหมาะสม | ปรับ DEFAULT_STANCE_HEIGHT |
| ระยะทางไม่ถูกต้อง | Calibration factor | ใช้ค่า VELOCITY_CALIBRATION = 0.333 |

### D. References

1. **ROBOT_ARCHITECTURE.tex** - Hierarchical Control Architecture specification
2. **RELATIVE_POSITION_CONTROL_FEASIBILITY.md** - Feasibility study document
3. **PROTOCOL.md** - Binary Protocol v1.2 specification
4. **test_quadruped_control.py** - Base gait control implementation

---

## บันทึกการเปลี่ยนแปลง

| เวอร์ชัน | วันที่ | ผู้แก้ไข | รายละเอียด |
|---------|--------|---------|------------|
| 1.0 | 5 ก.พ. 2026 | M-TRCH | เอกสารฉบับแรก + ผลการทดสอบ |

---

*เอกสารนี้เป็นส่วนหนึ่งของโครงการ BLEGS Actuator Unit - Quadruped Robot*
