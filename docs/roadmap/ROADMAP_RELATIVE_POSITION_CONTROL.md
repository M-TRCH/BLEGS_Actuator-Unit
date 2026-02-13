# Roadmap: ระบบควบคุมตำแหน่งแบบสัมพัทธ์ (Relative Position Control)

**โครงการ:** BLEGS Actuator Unit - Quadruped Robot  
**เอกสาร:** Roadmap การพัฒนาและผลการทดสอบระบบควบคุมตำแหน่งแกน Y  
**วันที่:** 5 กุมภาพันธ์ 2026 (อัปเดต: 13 กุมภาพันธ์ 2026)  
**ผู้จัดทำ:** M-TRCH  
**เวอร์ชัน:** 1.6  
**สถานะ:** ✅ Roadmap 7.1-7.2 ดำเนินการแล้ว | 🎯 Y-Accuracy ±30mm | ✅ Yaw Control ใช้งานได้ | ⚠️ Balance Control ต้องปรับปรุง

---

## สารบัญ

1. [บทสรุป](#1-บทสรุป)
2. [สถาปัตยกรรมระบบ](#2-สถาปัตยกรรมระบบ)
3. [ทฤษฎีและหลักการ](#3-ทฤษฎีและหลักการ)
4. [รายละเอียดการ Implement](#4-รายละเอียดการ-implement)
5. [ผลการทดสอบ](#5-ผลการทดสอบ)
6. [การวิเคราะห์ Calibration Factor](#6-การวิเคราะห์-calibration-factor)
7. [แผนการปรับปรุง](#7-แผนการปรับปรุง)
   - 7.1 [Roadmap ระยะสั้น](#71-roadmap-ระยะสั้น-1-2-สัปดาห์)
   - 7.2 [Roadmap ระยะกลาง](#72-roadmap-ระยะกลาง-1-2-เดือน)
   - 7.3 [Roadmap ระยะยาว](#73-roadmap-ระยะยาว-3-6-เดือน)
   - 7.4 [Roadmap ขั้นสูง](#74-roadmap-ขั้นสูง-6-12-เดือน)
   - 7.5 [Performance Improvement Targets](#75-performance-improvement-targets)
   - 7.6 [Testing & Validation Roadmap](#76-testing--validation-roadmap)
   - 7.7 [Risk Management](#77-risk-management)
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
| **Calibration** | ✅ เสร็จสิ้น | Applied factor 3.04 (10 ก.พ. 2026) |
| **Logging System** | ✅ เสร็จสิ้น | CSV logging with toggle [L] |
| **Idle March** | ✅ เสร็จสิ้น | 2x lift height for visibility |
| **Backward Testing** | ✅ เสร็จสิ้น | Test command [6] added |
| **Roadmap 7.1** | ✅ เสร็จสมบูรณ์ | Y-axis ±30mm accuracy |
| **IMU Integration** | ✅ เสร็จสิ้น | GY25 serial reader (11 ก.พ. 2026) |
| **Yaw Control** | ✅ ใช้งานได้ | Differential stepping, x=±15mm, yaw=±1.5° @ 500mm |
| **Balance Controller** | 🔄 ต้องปรับปรุง | ดีในท่ายืน, overshoot ในท่าเดิน (12 ก.พ.) |
| **Bezier Trajectory** | ✅ เสร็จสิ้น | นุ่มนวลกว่า elliptical (13 ก.พ. 2026) |
| **Smooth Transitions** | 🔄 ต้องปรับปรุง | กระตุกในขา swing phase (13 ก.พ.) |
| **Parameter Tuning** | ✅ เสร็จสิ้น | lift_height=15mm, stance_ratio=0.75 (13 ก.พ.) |

### 1.3 ผลการทดสอบโดยสรุป

#### Phase 1: การทดสอบก่อน Calibration (5 ก.พ. 2026)

การทดสอบคำสั่งเคลื่อนที่ทั้ง 4 โหมดทำงานได้ **ดีเยี่ยม** โดยหุ่นยนต์เคลื่อนที่อย่างราบรื่น มีข้อสังเกตสำคัญคือ:

| คำสั่ง | ตำแหน่งเป้าหมาย | ระยะทางจริง | อัตราส่วน |
|--------|----------------|-------------|----------|
| 1 | 100 mm | 300 mm | 3.00x |
| 2 | 200 mm | 600 mm | 3.00x |
| 3 | 300 mm | 920 mm | 3.07x |
| 4 | 400 mm | 1240 mm | 3.10x |

**Calibration Factor โดยเฉลี่ย: ≈ 3.04x**

#### Phase 2: การทดสอบหลัง Calibration (10 ก.พ. 2026)

หลังจากนำ `VELOCITY_CALIBRATION = 3.04` ไปใช้งาน:

| ผลการทดสอบ | ค่า |
|------------|-----|
| **ทิศทาง** | ✅ เดินหน้า-ถอยหลัง ทำงานถูกต้อง |
| **ความแม่นยำ Y-axis** | ✅ ±30mm (ยอมรับได้) |
| **การเคลื่อนที่** | ✅ ราบรื่น, เสถียร |
| **ปัญหาที่พบ** | ⚠️ **Yaw Drift**: หุ่นยนต์เลี้ยวเองระหว่างเดิน |

#### Phase 3: IMU Integration & Yaw Control (11 ก.พ. 2026)

| ฟีเจอร์ | สถานะ | รายละเอียด |
|---------|--------|------------|
| **IMU Integration** | ✅ สำเร็จ | GY25 serial reader @ COM22 |
| **Yaw Controller** | ✅ ทำงานได้ | Differential stepping (PD control) |
| **ผลการทดสอบ** | ✅ ยอมรับได้ | @ y=500mm: x=+15mm, yaw=1.5° |
| **พารามิเตอร์** | 🔧 | Kp=0.8, Kd=0.01, max_corr=15mm |

**วิเคราะห์ผล:**
- Lateral drift (X-axis): ±15mm ใน 500mm (3% error) - ยอมรับได้สำหรับ dead reckoning
- Yaw drift: ±1.5° ใน 500mm - ยอมรับได้สำหรับการเดินระยะสั้น-กลาง
- Differential stepping ทำงานตามที่ออกแบบ
- แนวทาง: parameter tuning ต่อเนื่องเพื่อเพิ่มความแม่นยำ

#### Phase 4: Balance Control Development (12 ก.พ. 2026)

| ฟีเจอร์ | สถานะ | รายละเอียด |
|---------|--------|------------|
| **Balance Controller** | ✅ สร้างเสร็จ | PD control สำหรับ roll & pitch |
| **ท่ายืน** | ✅ ดีเยี่ยม | ตอบสนองเร็ว, รักษาระดับได้ดี |
| **ท่าเดิน** | ⚠️ ต้องปรับปรุง | Overshoot ในระยะยืด-หดของขา |
| **Toggle Control** | ✅ | กด [C] เพื่อเปิด/ปิด |

**วิเคราะห์ปัญหา:**
- **ท่ายืน (Standing)**: ทำงานได้ดีเยี่ยม - ตอบสนองเร็ว รักษาสมดุลได้ดี
- **ท่าเดิน (Walking)**: พบปัญหา overshoot คล้ายการตอบสนองมากเกินไป
  - สาเหตุที่เป็นไปได้: gains สูงเกินไป, update rate ไม่เหมาะสม
  - ผลกระทบ: การยืด-หดของขาไม่ราบรื่น, อาจส่งผลต่อเสถียรภาพ
  - แนวทางแก้ไข: ลด gains, ปรับ filter, หรือใช้ balance เฉพาะ stance phase

**พารามิเตอร์ปัจจุบัน:**
- ROLL_K_P = 0.8, ROLL_K_D = 0.03
- PITCH_K_P = 0.8, PITCH_K_D = 0.03  
- MAX_HEIGHT_OFFSET = 20.0 mm

**สถานะ:** ใช้งานได้ในท่ายืน, ยังไม่แนะนำให้ใช้ในท่าเดิน ต้องปรับแต่งเพิ่มเติม

#### Phase 5: Bezier Trajectory Implementation (13 ก.พ. 2026)

**งานที่ดำเนินการ:**
- ✅ เปลี่ยนจาก Elliptical เป็น Bezier Curve trajectory
- ✅ ปรับ control point multiplier = 1.25 (แก้ไขปัญหาความสูงไม่ถึง 100%)
- ✅ ทดสอบเปรียบเทียบกับ elliptical trajectory

**ผลการทดสอบ:**
- ✅ Bezier curve สร้างการเคลื่อนที่ที่นุ่มนวลกว่า elliptical เล็กน้อย
- ✅ การเปลี่ยนทิศทางเดินหน้า-ถอยหลังทำงานถูกต้อง (ใช้ `reverse=not reverse`)
- ✅ ความแม่นยำยังคงเดิม (±30mm @ 500mm)

**สถานะ:** ✅ เสร็จสมบูรณ์ - ใช้งานได้ดีกว่า elliptical

#### Phase 6: Parameter Tuning & Optimization (13 ก.พ. 2026)

**งานที่ดำเนินการ:**
- ✅ ปรับ `GAIT_LIFT_HEIGHT = 15.0` mm (ลดจาก 30.0)
- ✅ ปรับ `SMOOTH_TROT_STANCE_RATIO = 0.75` (เพิ่มจาก 0.5)
- ✅ ทดสอบความเสถียรและการยกเท้า

**ผลการทดสอบ:**
- ✅ **Stance Ratio 0.75**: หุ่นยนต์เดินเสถียรขึ้นมาก (ดีกว่า 0.5 อย่างเห็นได้ชัด)
- ⚠️ **Lift Height 15mm**: ยกพ้นพื้นได้ในพื้นเรียบ แต่มีโอกาสลากพื้นหากสมดุลเปลี่ยน
  - **ข้อเสนอแนะ**: อาจต้องเพิ่มเป็น 20-25mm สำหรับ safety margin
  - **หมายเหตุ**: idle march ใช้ `2x lift_height` = 30mm (พอดี)

**สถานะ:** ✅ เสร็จสมบูรณ์ - stance ratio ดีแล้ว, lift height อาจต้องปรับเพิ่ม

#### Phase 7: Smooth Transition Development (13 ก.พ. 2026)

**งานที่ดำเนินการ:**
- ✅ เพิ่ม Command [7] - Smooth walk +600mm with march transitions
- ✅ Implement `transition_to_march` parameter ใน `move_relative_y()`
- ✅ Implement `resume_from` parameter ใน `start_idle_march()`
- ✅ ทดสอบ seamless transition (march → walk → march)

**ผลการทดสอบ:**

**Transition 1: Idle March → Walking**
- ✅ **Stance Phase Legs**: ราบเรียบมาก (FR, RL เริ่มด้วย stance)
- ⚠️ **Swing Phase Legs**: กระตุกบ้าง (FL, RR เริ่มด้วย swing)
- **สาเหตุ**: เท้าที่อยู่กลางอากาศต้องเปลี่ยน trajectory ทันที

**Transition 2: Walking → Idle March**
- ✅ **Stance Phase Legs**: ราบเรียบมาก (เท้าอยู่บนพื้น)
- ⚠️ **Swing Phase Legs**: กระแทกพื้นอย่างเห็นได้ชัด (เท้าตกลงมาเร็ว)
- **สาเหตุ**: trajectory เปลี่ยนจาก forward movement → vertical lift ทันที

**สถานะ:** 🔄 ใช้งานได้แต่ต้องปรับปรุง - ต้องแก้ไข swing phase transition

**การแก้ไข Yaw Drift (11 ก.พ. 2026):**
- ✅ เพิ่ม IMU integration (GY25 serial reader)
- ✅ ใช้ Differential Stepping สำหรับ yaw correction
- ✅ ผลการทดสอบ: x=+15mm, yaw=1.5° @ y=500mm (ค่าที่ยอมรับได้)
- พารามิเตอร์: YAW_K_P = 0.8, YAW_K_D = 0.01, YAW_MAX_CORRECTION = 15.0 mm
- 🔧 Parameter tuning ต่อเนื่อง (13 ก.พ. 2026) เพื่อเพิ่มการชดเชย

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

### 7.1 Roadmap ระยะสั้น (1-2 สัปดาห์) - ✅ เสร็จสมบูรณ์ (วันที่ 13 ก.พ. 2026)

| ลำดับ | งาน | ความสำคัญ | เวลา | สถานะ |
|-------|-----|-----------|------|--------|
| 1 | ✅ ทดสอบระบบพื้นฐาน | สูง | เสร็จแล้ว | เสร็จ 5 ก.พ. 2026 |
| 2 | ✅ เพิ่ม Calibration Constant | สูง | 30 นาที | เสร็จ 10 ก.พ. 2026 |
| 3 | ✅ ทดสอบหลังปรับ calibration | สูง | 2 ชั่วโมง | เสร็จ 10 ก.พ. 2026 |
| 4 | ✅ ทดสอบ backward movement | กลาง | 1 ชั่วโมง | เสร็จ 10 ก.พ. 2026 |
| 5 | ✅ เพิ่ม logging และ data recording | กลาง | 2 ชั่วโมง | เสร็จ 10 ก.พ. 2026 |

**สรุป Roadmap 7.1: ✅ เสร็จสมบูรณ์ 100% (5-10 ก.พ. 2026)**

#### รายละเอียดงานที่เสร็จแล้ว

**[2] เพิ่ม Calibration Constant** ✅
- **Implementation**: แนวทาง C (ปรับใน State Estimator)
- **ค่า Calibration**: `VELOCITY_CALIBRATION = 3.04`
- **การทำงาน**: คูณ v_body_y ด้วย calibration factor ใน `state_estimator.update()`
- **ผลที่ได้**: ระยะทางแม่นยำ ±30mm
- **ไฟล์ที่แก้ไข**: `relative_position_control.py` บรรทัด 87-89, 565-568

**[3] ทดสอบหลังปรับ calibration** ✅
- **ผลการทดสอบ**:
  - ✅ เดินหน้า: แม่นยำ error ±30mm
  - ✅ ถอยหลัง: แม่นยำ error ±30mm
  - ⚠️ ปัญหา: หุ่นยนต์เลี้ยวเองระหว่างเดิน (Yaw Drift)
- **สาเหตุ**: ไม่มีระบบควบคุม yaw orientation
- **แนวทางแก้ไข**: ต้องเพิ่ม yaw control (Roadmap 7.2 Task #7)
- **วันที่ทดสอบ**: 10 ก.พ. 2026

**[4] ทดสอบ backward movement** ✅
- **เพิ่มคำสั่งใหม่**: `test_backward_200()` สำหรับทดสอบถอยหลัง 200mm
- **Menu command**: กด `[6]` เพื่อทดสอบ backward -200mm
- **วัตถุประสงค์**: ยืนยันว่า calibration ทำงานในทั้ง 2 ทิศทาง
- **ไฟล์ที่แก้ไข**: `relative_position_control.py` บรรทัด 639-647, 707, 755-756, 805-806

**[5] เพิ่ม logging และ data recording** ✅
- **ฟีเจอร์**:
  - Auto-generate CSV log files with timestamp
  - Log: time, target_y, current_y, error, v_y, step_length, step_indices
  - Toggle logging: กด `[L]` ในเมนู
  - Log rate: ทุก 10 control cycles (configurable)
  - Log path: `logs/movement_<distance>mm_<timestamp>.csv`
- **Configuration**:
  - `ENABLE_LOGGING = False` (default off)
  - `LOG_FILE_PATH = "logs/"`
  - `LOG_RATE = 10` cycles
- **Functions**:
  - `init_logging()`: สร้างไฟล์ log
  - `log_control_step()`: บันทึกข้อมูลแต่ละ step
  - `close_logging()`: ปิดไฟล์ log
- **ไฟล์ที่แก้ไข**: `relative_position_control.py` บรรทัด 89-95, 119-121, 304-390, 523-526, 581-583, 601-604, 612-613, 710, 788-795, 828-835

### 7.2 Roadmap ระยะกลาง (1-2 เดือน) - 🔄 ดำเนินการบางส่วน

| ลำดับ | งาน | ความสำคัญ | เวลา | สถานะ |
|-------|-----|-----------|------|--------|
| 6 | ⬜ แก้ไข Swing Phase Transition | **🔴 สูงมาก** | 4 ชั่วโมง | **ต้องทำต่อ (13 ก.พ.)** |
| 7 | ⬜ เพิ่มการเคลื่อนที่แกน X (strafe) | กลาง | 8 ชั่วโมง | ยังไม่เริ่ม |
| 8 | ✅ เพิ่มการควบคุม yaw orientation | **สูง** | 8 ชั่วโมง | **เสร็จ 11 ก.พ. 2026** |
| 9 | ✅ Integrate IMU สำหรับ orientation | **สูง** | 16 ชั่วโมง | **เสร็จ 11 ก.พ. 2026** |
| 10 | 🔄 ปรับปรุง Balance Control | **สูง** | 24 ชั่วโมง | **กำลังปรับแต่ง** |
| 11 | ⬜ ปรับปรุง State Estimator (Sensor Fusion) | กลาง | 24 ชั่วโมง | ยังไม่เริ่ม |

**สรุปความคืบหน้า Roadmap 7.2:**

**[6] ⚠️ แก้ไข Swing Phase Transition (ต้องทำต่อ - Priority สูงมาก)**
- **ปัญหาที่พบ** (13 ก.พ. 2026):
  1. March → Walk: ขา swing phase กระตุกเมื่อเริ่มเดิน
  2. Walk → March: ขา swing phase กระแทกพื้นเมื่อหยุดเดิน
- **สาเหตุ**:
  - Trajectory เปลี่ยนทันทีโดยไม่พิจารณาว่าเท้าอยู่ในอากาศหรือไม่
  - March trajectory (step_forward=0) vs Walk trajectory (step_forward=30mm)
  - Vertical velocity ไม่ได้ถูก smooth transition
- **แนวทางแก้ไข**:
  1. **Phase Detection**: ตรวจสอบ gait phase ก่อน transition
     - ถ้าเท้าอยู่ใน swing phase → รอให้ลงพื้นก่อน transition
     - ใช้ modulo operation: `if (step_index % TRAJECTORY_STEPS) < stance_steps`
  2. **Trajectory Interpolation**: ใช้ linear interpolation ระหว่าง old/new trajectory
     - Blend factor: `alpha = min(1.0, elapsed_time / blend_duration)`
     - `P_blended = (1 - alpha) * P_old + alpha * P_new`
     - Blend duration ≈ 0.2-0.3 seconds (10-15 steps @ 50Hz)
  3. **Velocity Continuity**: คำนวณ velocity ของเท้าและทำให้ต่อเนื่อง
     - `v_foot = (P[k+1] - P[k]) / dt`
     - Ensure `|v_foot_after - v_foot_before| < threshold`
  4. **Synchronization**: รอให้ทุกขาเข้าสู่ stance phase พร้อมกันก่อน transition
     - Implementation: `wait_for_all_stance_phase()` function
- **Estimated Time**: 4 ชั่วโมง
- **Priority**: 🔴 สูงมาก (ทำให้ command [7] ใช้งานได้อย่างสมบูรณ์)
- **Success Criteria**: 
  - Transition ไม่กระตุก/กระแทกพื้นทุกขา
  - Smooth acceleration/deceleration ในทุก transition


**[7] ✅ เพิ่มการควบคุม yaw orientation (เสร็จ 11 ก.พ. 2026)**
- **ฟีเจอร์ที่เพิ่ม**:
  - YawController class ด้วย PD control
  - Differential stepping: ปรับระยะก้าวซ้าย-ขวาต่างกัน
  - IMU yaw feedback integration
- **พารามิเตอร์**:
  - YAW_K_P = 0.8 mm/deg
  - YAW_K_D = 0.01 mm/(deg/s)  
  - YAW_MAX_CORRECTION = 15.0 mm
- **ผลการทดสอบ**: @ y=500mm → x=+15mm, yaw=1.5° ✅ ยอมรับได้
- **สถานะ**: ใช้งานได้ดี, parameter tuning ต่อเนื่อง (13 ก.พ.)

**[8] ✅ Integrate IMU สำหรับ orientation (เสร็จ 11 ก.พ. 2026)**
- **Hardware**: GY25 IMU @ COM22 (serial communication)
- **Features**:
  - IMUReader class สำหรับอ่านข้อมูล roll, pitch, yaw
  - Real-time orientation feedback @ 50 Hz
  - Error handling และ timeout protection
- **Integration**: เชื่อมต่อกับ YawController และ BalanceController
- **สถานะ**: ทำงานเสถียร

**[9] 🔄 ปรับปรุง Balance Control (กำลังดำเนินการ 12-13 ก.พ. 2026)**
- **สร้างเสร็จแล้ว** (12 ก.พ.):
  - BalanceController class (PD control สำหรับ roll & pitch)
  - Per-leg height offset computation
  - Toggle control [C] key
- **ผลการทดสอบ**:
  - ✅ **ท่ายืน**: ดีเยี่ยม - ตอบสนองเร็ว รักษาระดับได้ดี
  - ⚠️ **ท่าเดิน**: overshoot ในระยะยืด-หดของขา
- **ปัญหา**:
  - Gains อาจสูงเกินไปสำหรับ dynamic movement
  - Update rate หรือ filtering ไม่เหมาะสม
  - อาจต้องใช้ balance เฉพาะ stance phase
- **แนวทางแก้ไข**:
  - ปรับลด gains (Kp, Kd)
  - เพิ่ม low-pass filter
  - Disable balance ใน swing phase
  - Tune update rate และ phase detection
- **สถานะ**: ใช้งานได้ในท่ายืน, ต้องปรับแต่งสำหรับท่าเดิน

**[10] ✅ Bezier Trajectory Implementation (เสร็จ 13 ก.พ. 2026)**
- ✅ Implement Bezier curve trajectory generator
- ✅ แก้ไข control point height (multiplier 1.25)
- ✅ ทดสอบเปรียบเทียบกับ elliptical
- **ผลลัพธ์**: นุ่มนวลกว่า elliptical, ทำงานถูกต้องทั้งเดินหน้า-ถอยหลัง

**[11] ✅ Parameter Tuning (เสร็จ 13 ก.พ. 2026)**
- ✅ ปรับ GAIT_LIFT_HEIGHT = 15.0 mm (ลดลงจาก 30.0)
- ✅ ปรับ SMOOTH_TROT_STANCE_RATIO = 0.75 (เพิ่มจาก 0.5)
- **ผลลัพธ์**: 
  - Stance ratio 0.75 → หุ่นยนต์เดินเสถียรขึ้นมาก ✅
  - Lift height 15mm → พอใช้ได้แต่แนะนำเพิ่มเป็น 20-25mm ⚠️

**[12] 🔄 Smooth Transition Development (ต้องปรับปรุง 13 ก.พ. 2026)**
- ✅ สร้าง Command [7] - smooth walk with march transitions
- ✅ Implement transition_to_march และ resume_from parameters
- ⚠️ **ปัญหาที่พบ**:
  - Stance phase legs: transition ราบเรียบ ✅
  - Swing phase legs: กระตุก/กระแทกพื้น ⚠️
- **สาเหตุ**: trajectory เปลี่ยนทันทีขณะเท้าอยู่กลางอากาศ
- **สถานะ**: ต้องแก้ไข swing phase transition logic

### 7.3 Roadmap ระยะยาว (3-6 เดือน)

| ลำดับ | งาน | ความสำคัญ | เวลา (ประมาณ) |
|-------|-----|-----------|---------------|
| 10 | ⬜ Absolute Position Control (Waypoint Navigation) | กลาง | 32 ชั่วโมง |
| 11 | ⬜ Sensor Fusion (IMU + Encoder Integration) | สูง | 40 ชั่วโมง |
| 12 | ⬜ Obstacle Avoidance Integration | กลาง | 48 ชั่วโมง |
| 13 | ⬜ Terrain Adaptation (Rough Surface) | กลาง | 56 ชั่วโมง |
| 14 | ⬜ Multi-waypoint Path Planning | ต่ำ | 24 ชั่วโมง |
| 15 | ⬜ Velocity Profile Optimization | กลาง | 16 ชั่วโมง |

#### รายละเอียดงานระยะยาว

**[10] Absolute Position Control**
- **วัตถุประสงค์**: ควบคุมตำแหน่งสัมบูรณ์ในพื้นที่ทำงาน (global coordinate)
- **ฟีเจอร์หลัก**:
  - World coordinate frame tracking
  - Set absolute target position (x, y, θ)
  - Coordinate transformation (body → world frame)
  - Position initialization/reset
- **Dependencies**: IMU integration, improved state estimation
- **Success Criteria**: ควบคุมตำแหน่งได้แม่นยำ ±20mm ในระยะ 5 เมตร

**[11] Sensor Fusion (IMU + Encoder Integration)**
- **วัตถุประสงค์**: ปรับปรุง state estimation ด้วย sensor fusion
- **ฟีเจอร์หลัก**:
  - Extended Kalman Filter (EKF) implementation
  - IMU (GY-25) orientation feedback
  - Motor encoder position feedback
  - Drift correction algorithm
  - Complementary filter for acceleration
- **Dependencies**: GY25 serial integration, encoder calibration
- **Success Criteria**: ลด drift error เหลือ < 5% ใน trajectory 10 เมตร

**[12] Obstacle Avoidance Integration**
- **วัตถุประสงค์**: หลีกเลี่ยงสิ่งกีดขวางระหว่างการเคลื่อนที่
- **ฟีเจอร์หลัก**:
  - Vision-based obstacle detection (AR tag / color blob)
  - Dynamic path replanning
  - Safe distance maintenance
  - Emergency stop protocol
- **Dependencies**: Vision system integration, path planning
- **Success Criteria**: หลีกเลี่ยงวัตถุขนาด > 50mm ที่ระยะ > 200mm

**[13] Terrain Adaptation**
- **วัตถุประสงค์**: ปรับ gait ให้เหมาะกับพื้นผิวที่ไม่เรียบ
- **ฟีเจอร์หลัก**:
  - Surface roughness detection
  - Adaptive stance height
  - Dynamic gait parameter adjustment
  - Slip detection and recovery
- **Dependencies**: IMU, force estimation
- **Success Criteria**: เดินบนพื้นขรุขระได้โดยไม่สะดุด

**[14] Multi-waypoint Path Planning**
- **วัตถุประสงค์**: วางแผนเส้นทางผ่านจุดหลายจุด
- **ฟีเจอร์หลัก**:
  - Waypoint queue management
  - Smooth trajectory between waypoints
  - Dynamic waypoint insertion/removal
  - Path optimization (shortest path)
- **Dependencies**: Absolute position control
- **Success Criteria**: ผ่าน waypoint ≥ 5 จุดติดต่อกันได้สำเร็จ

**[15] Velocity Profile Optimization**
- **วัตถุประสงค์**: ปรับปรุง velocity profile สำหรับการเคลื่อนที่ที่ราบรื่น
- **ฟีเจอร์หลัก**:
  - Trapezoidal velocity profile
  - Smooth acceleration/deceleration
  - Jerk minimization
  - Energy-efficient velocity planning
- **Dependencies**: Dynamic model, torque estimation
- **Success Criteria**: ลด peak acceleration ≥ 30%, ประหยัดพลังงาน ≥ 20%

### 7.4 Roadmap ขั้นสูง (6-12 เดือน)

| ลำดับ | งาน | ความสำคัญ | เวลา (ประมาณ) |
|-------|-----|-----------|---------------|
| 16 | ⬜ Model Predictive Control (MPC) | ต่ำ | 80 ชั่วโมง |
| 17 | ⬜ Learning-based Gait Optimization | ต่ำ | 120 ชั่วโมง |
| 18 | ⬜ Stair Climbing Capability | กลาง | 64 ชั่วโมง |
| 19 | ⬜ Dynamic Stability Controller | กลาง | 72 ชั่วโมง |
| 20 | ⬜ Multi-robot Coordination | ต่ำ | 96 ชั่วโมง |
| 21 | ⬜ Autonomous Exploration Mode | ต่ำ | 100 ชั่วโมง |

#### รายละเอียดงานขั้นสูง

**[16] Model Predictive Control (MPC)**
- **วัตถุประสงค์**: ใช้ MPC สำหรับ trajectory optimization
- **ฟีเจอร์หลัก**:
  - Real-time trajectory optimization
  - Constraint handling (joint limits, stability)
  - Predictive collision avoidance
  - Optimal control sequence generation
- **Research Required**: MPC formulation, solver selection (OSQP/qpOASES)

**[17] Learning-based Gait Optimization**
- **วัตถุประสงค์**: ใช้ machine learning ปรับปรุง gait parameters
- **ฟีเจอร์หลัก**:
  - Reinforcement learning for gait tuning
  - Adaptive to different terrains
  - Energy consumption optimization
  - Transfer learning from simulation
- **Research Required**: RL framework (PPO/SAC), sim-to-real transfer

**[18] Stair Climbing Capability**
- **วัตถุประสงค์**: เพิ่มความสามารถในการขึ้น-ลงบันได
- **ฟีเจอร์หลัก**:
  - Step edge detection
  - Height-adaptive gait
  - Dynamic balance control
  - Fall recovery protocol
- **Constraints**: Step height ≤ 80mm, step depth ≥ 150mm

**[19] Dynamic Stability Controller**
- **วัตถุประสงค์**: รักษาเสถียรภาพแบบไดนามิก
- **ฟีเจอร์หลัก**:
  - Zero Moment Point (ZMP) controller
  - Center of Mass (CoM) trajectory planning
  - Push recovery
  - Disturbance rejection
- **Research Required**: ZMP calculation, stability margin estimation

**[20] Multi-robot Coordination**
- **วัตถุประสงค์**: ประสานงานหลายหุ่นยนต์
- **ฟีเจอร์หลัก**:
  - Inter-robot communication protocol
  - Formation control
  - Collision avoidance between robots
  - Synchronized task execution
- **Dependencies**: Wireless communication, distributed control

**[21] Autonomous Exploration Mode**
- **วัตถุประสงค์**: สำรวจพื้นที่โดยอัตโนมัติ
- **ฟีเจอร์หลัก**:
  - SLAM (Simultaneous Localization and Mapping)
  - Frontier-based exploration
  - Loop closure detection
  - Map optimization
- **Dependencies**: Vision system, absolute positioning

### 7.5 Performance Improvement Targets

#### ปัจจุบัน (13 ก.พ. 2026 - หลัง Calibration + IMU + Balance)
| Metric | ค่าปัจจุบัน | เป้าหมายระยะกลาง | เป้าหมายระยะยาว |
|--------|-------------|-------------------|------------------|
| Position Accuracy (Y) | ±30mm @ 500mm ✅ | ±20mm @ 1m | ±10mm @ 1m |
| Lateral Drift (X) | ±15mm @ 500mm ✅ | ±10mm @ 1m | ±5mm @ 1m |
| Yaw Accuracy | ±1.5° @ 500mm ✅ | ±5° @ 1m | ±2° @ 5m |
| Yaw Control | Differential stepping ✅ | Active correction | Closed-loop control |
| Balance Control | ดีในท่ายืน ⚠️ | ใช้ได้ในท่าเดิน | Robust all-terrain |
| Max Velocity | 70 mm/s ✅ | 100 mm/s | 200 mm/s |
| Repeatability | ±50mm | ±20mm | ±10mm |
| Battery Life | 15 min | 30 min | 60 min |
| Terrain Coverage | Flat only | Low roughness | Stairs, obstacles |
| Control Latency | ~20ms | ~10ms | ~5ms |
| Success Rate | 100% (flat) ✅ | 95% (rough) | 90% (stairs) |

### 7.6 Testing & Validation Roadmap

| Phase | Testing Focus | Timeline |
|-------|---------------|----------|
| **Phase 1** | Calibration Validation | Week 1-2 |
| | - Forward/backward accuracy | |
| | - Repeatability testing (20 trials) | |
| | - Different velocities | |
| **Phase 2** | Extended Range Testing | Week 3-4 |
| | - Long distance (5m, 10m) | |
| | - Drift measurement | |
| | - Battery consumption | |
| **Phase 3** | Multi-axis Integration | Month 2 |
| | - X-axis (strafe) validation | |
| | - Yaw rotation testing | |
| | - Combined motions | |
| **Phase 4** | Sensor Fusion Validation | Month 3-4 |
| | - IMU integration accuracy | |
| | - Encoder feedback validation | |
| | - Comparison: dead reckoning vs sensor fusion | |
| **Phase 5** | Terrain Testing | Month 5-6 |
| | - Rough surface | |
| | - Inclined surface (±10°) | |
| | - Outdoor testing | |
| **Phase 6** | Advanced Capability | Month 7-12 |
| | - Obstacle avoidance | |
| | - Autonomous navigation | |
| | - Stress testing | |

### 7.7 Risk Management

| Risk | Probability | Impact | Mitigation Strategy |
|------|-------------|--------|---------------------|
| Calibration drift over time | Medium | Medium | Periodic recalibration, auto-calibration routine |
| Slip on smooth surfaces | High | Medium | Surface detection, adaptive friction control |
| Battery depletion during mission | Medium | High | Battery monitoring, auto-return-to-base |
| Motor overheating | Low | High | Thermal monitoring, duty cycle limits |
| Communication loss | Low | High | Failsafe mode, emergency stop |
| IMU drift | High | Medium | Sensor fusion, periodic zero-velocity updates |
| Collision damage | Medium | High | Obstacle detection, protective bumpers |
| Software bugs | Medium | Medium | Extensive testing, graceful error handling |

---

## 8. ภาคผนวก

### A. Parameter Reference

```python
# === Navigation ===
NAV_V_MAX = 70.0              # mm/s (updated Feb 10, 2026)
NAV_K_P = 1.0                 # -
NAV_TOLERANCE = 10.0          # mm
NAV_TIMEOUT = 60.0            # seconds

# === Calibration (Feb 10, 2026) ===
VELOCITY_CALIBRATION = 3.04   # Applied in state_estimator - Compensates for trajectory span (2x) × gait overlap (1.5x)
# Result: Y-axis accuracy ±30mm (tested and verified)

# === IMU Integration (Feb 11, 2026) ===
IMU_PORT = 'COM22'            # Serial port for GY25 IMU
IMU_ENABLED = True            # Enable/disable IMU integration
YAW_K_P = 0.8                 # Yaw controller proportional gain (mm/deg)
YAW_K_D = 0.01                # Yaw controller derivative gain (mm/(deg/s))
YAW_MAX_CORRECTION = 15.0     # Maximum differential step (mm)

# === Bezier Trajectory (Feb 13, 2026) ===
# Replaced elliptical trajectory with Bezier curves
# Control point multiplier = 1.25 (to achieve 100% actual lift height)
# Result: นุ่มนวลกว่า elliptical trajectory

# === Smooth Transitions (Feb 13, 2026) ===
# Command [7]: Smooth walk +600mm with march transitions
# Issues found:
# - Stance phase legs: smooth ✅
# - Swing phase legs: jerky/ground impact ⚠️
# TODO: Implement phase detection and trajectory blending

# === Balance Control (Feb 12, 2026) ===
BALANCE_ENABLED = False       # Enable/disable balance control (toggle with [C])
ROLL_K_P = 0.8                # Roll proportional gain (mm/deg)
ROLL_K_D = 0.03               # Roll derivative gain (mm/(deg/s))
PITCH_K_P = 0.8               # Pitch proportional gain (mm/deg)
PITCH_K_D = 0.03              # Pitch derivative gain (mm/(deg/s))
MAX_HEIGHT_OFFSET = 20.0      # Maximum leg height offset from balance (mm)
INVERT_ROLL = False           # Invert roll correction direction
INVERT_PITCH = True           # Invert pitch correction direction

# === Logging ===
ENABLE_LOGGING = False        # Enable data logging (toggle with [L])
LOG_FILE_PATH = "logs/"       # Directory for log files
LOG_RATE = 10                 # Log every N control cycles

# === Gait (Updated Feb 13, 2026) ===
UPDATE_RATE = 50              # Hz
TRAJECTORY_STEPS = 30         # steps/cycle
GAIT_CYCLE_TIME = 0.6         # seconds
GAIT_LIFT_HEIGHT = 15.0       # mm (ลดจาก 30.0 - tested Feb 13, 2026)
                               # NOTE: พอใช้ในพื้นเรียบ แต่แนะนำ 20-25mm สำหรับ safety margin
                               # Idle march ใช้ 2x = 30mm
GAIT_STEP_FORWARD = 30.0      # mm (ลดจาก 50.0 - matched with actual usage Feb 13, 2026)
SMOOTH_TROT_STANCE_RATIO = 0.75  # - (เพิ่มจาก 0.5 → เสถียรมากขึ้น - tested Feb 13, 2026)

# === Robot Geometry ===
BODY_LENGTH = 200.0           # mm
BODY_WIDTH = 170.0            # mm
MOTOR_SPACING = 85.0          # mm
L_AC = 105.0                  # mm
L_BD = 105.0                  # mm
L_CE = 145.0                  # mm
L_DE = 145.0                  # mm
DEFAULT_STANCE_HEIGHT = -220.0  # mm
```

### B. Testing Commands

```python
# Interactive Mode Commands (จากเมนู)
[1] Move forward +100mm
[2] Move backward -100mm
[3] Move forward +500mm
[4] Custom distance (ใส่ระยะทางเอง)
[5] Run test sequence
[6] Test backward -200mm (Roadmap 7.1)

# Idle March Commands
[M] Start marching in place (step height: 30mm)
[S] Stop marching (return to stand)

# Other Commands
[H] Move to home/stand position
[P] Print current status
[D] Toggle debug output
[L] Toggle logging (saves to logs/movement_*.csv)
[C] Toggle balance control (roll & pitch stabilization)
[Q] Quit (motors return to -90° init position)

# คำสั่งที่ใช้ในการทดสอบ (ผ่านตัวเลือก [4] Custom distance)
# หมายเหตุ: ค่าเหล่านี้ใช้ก่อนมี calibration - จะให้ 3x overshoot
move_relative_y(+100.0)   # ทดสอบ 1: Forward 100mm → จริง 300mm (ก่อน calib)
move_relative_y(+200.0)   # ทดสอบ 2: Forward 200mm → จริง 600mm (ก่อน calib)
move_relative_y(+300.0)   # ทดสอบ 3: Forward 300mm → จริง 920mm (ก่อน calib)
move_relative_y(+400.0)   # ทดสอบ 4: Forward 400mm → จริง 1240mm (ก่อน calib)

# หลัง calibration (Feb 10, 2026) - คาดว่าจะแม่นยำ ±10%
move_relative_y(+100.0)   # Forward 100mm → คาดว่าจริง ~100mm
move_relative_y(-200.0)   # Backward 200mm → คาดว่าจริง ~200mm
move_relative_y(+500.0, timeout_s=30.0)  # With custom timeout

# Direct Function Calls (สำหรับเรียกใช้ใน Python)
move_relative_y(+100.0)   # Forward 100mm
move_relative_y(-200.0)   # Backward 200mm
move_relative_y(+500.0, timeout_s=30.0)  # With custom timeout
```

### C. Log File Format

Logging สามารถเปิด/ปิดได้ด้วยคำสั่ง `[L]` ไฟล์ log จะถูกบันทึกที่:

```
logs/movement_<distance>mm_<timestamp>.csv
```

**Format:**
```csv
time,target_y,current_y,error,v_y,step_length,FR_idx,FL_idx,RR_idx,RL_idx
0.000,100.0,0.0,100.0,50.00,4.93,0,15,15,0
0.200,100.0,10.0,90.0,50.00,4.93,10,25,25,10
0.400,100.0,20.0,80.0,50.00,4.93,20,5,5,20
...
```

**คอลัมน์:**
- `time`: เวลาที่ผ่านไป (วินาที)
- `target_y`: ตำแหน่งเป้าหมาย (mm)
- `current_y`: ตำแหน่งปัจจุบัน (mm) - จาก state estimator
- `error`: ค่าความผิดพลาด (mm)
- `v_y`: คำสั่งความเร็ว (mm/s)
- `step_length`: ความยาว step ที่คำนวณได้ (mm)
- `FR_idx, FL_idx, RR_idx, RL_idx`: Step index ของแต่ละขา (0-29)

### D. Troubleshooting

| ปัญหา | สาเหตุที่เป็นไปได้ | วิธีแก้ไข |
|-------|-------------------|----------|
| หุ่นยนต์ไม่เคลื่อนที่ | Motor ไม่ได้เชื่อมต่อ | ตรวจสอบ COM port และ motor discovery |
| เคลื่อนที่ผิดทิศทาง | IK mirror_x ผิด | ตรวจสอบ leg configuration |
| เคลื่อนที่ไม่ราบรื่น | UPDATE_RATE ต่ำเกินไป | เพิ่มเป็น 100 Hz |
| ล้มระหว่างเดิน | Stance height ไม่เหมาะสม | ปรับ DEFAULT_STANCE_HEIGHT |
| ระยะทางไม่ถูกต้อง (Y) | Calibration factor | ใช้ค่า VELOCITY_CALIBRATION = 3.04 |
| Yaw drift (ก่อน 11 ก.พ.) | ไม่มี yaw control | ✅ แก้ไขแล้ว - ใช้ differential stepping |
| **Balance overshoot ในท่าเดิน** | **Gains สูงเกินไป** | **ลด ROLL/PITCH Kp,Kd หรือ disable ใน swing phase** |
| IMU ไม่ทำงาน | Serial port ผิด | ตรวจสอบ IMU_PORT (ปัจจุบัน COM22) |
| Log file ไม่ถูกสร้าง | ENABLE_LOGGING = False | กด [L] เพื่อเปิด logging |
| Idle march ไม่ชัดเจน | Lift height ต่ำเกินไป | ปรับ march_lift_height (ค่าปัจจุบัน 2x) |

### E. References

1. **ROBOT_ARCHITECTURE.tex** - Hierarchical Control Architecture specification
2. **RELATIVE_POSITION_CONTROL_FEASIBILITY.md** - Feasibility study document
3. **PROTOCOL.md** - Binary Protocol v1.2 specification
4. **test_quadruped_control.py** - Base gait control implementation

---

## บันทึกการเปลี่ยนแปลง

| เวอร์ชัน | วันที่ | ผู้แก้ไข | รายละเอียด |
|---------|--------|---------|------------|
| 1.0 | 5 ก.พ. 2026 | M-TRCH | เอกสารฉบับแรก + ผลการทดสอบ |
| 1.1 | 10 ก.พ. 2026 | M-TRCH | ขยายแผนพัฒนา: เพิ่มรายละเอียดระยะยาว, roadmap ขั้นสูง, performance targets, testing roadmap, risk management |
| 1.2 | 10 ก.พ. 2026 | M-TRCH | ดำเนินการตาม Roadmap 7.1: เพิ่ม calibration constant (3.04), logging system, backward test, idle march (2x lift height), init position return |
| 1.3 | 10 ก.พ. 2026 | M-TRCH | ✅ Roadmap 7.1 เสร็จสมบูรณ์: ทดสอบแล้ว accuracy ±30mm, พบ Yaw Drift issue, เพิ่มความสำคัญของ yaw control (Task #7) |
| 1.4 | 11 ก.พ. 2026 | M-TRCH | ✅ IMU Integration & Yaw Control: เพิ่ม GY25 IMU reader, YawController (differential stepping), ผลทดสอบ x=±15mm, yaw=±1.5° @ 500mm |
| 1.5 | 12 ก.พ. 2026 | M-TRCH | ✅ Balance Control Development: เพิ่ม BalanceController (PD control), ทำงานดีในท่ายืน, พบ overshoot ในท่าเดิน ต้องปรับปรุง |
| 1.6 | 13 ก.พ. 2026 | M-TRCH | 🔧 Parameter Tuning: ปรับ yaw controller (เพิ่มชดเชย), ปรับระยะก้าวขา (สั้นลง), อัปเดต roadmap ครอบคลุมงาน 11-13 ก.พ., สรุปสถานะและผลการทดสอบทั้งหมด |

---

*เอกสารนี้เป็นส่วนหนึ่งของโครงการ BLEGS Actuator Unit - Quadruped Robot*
