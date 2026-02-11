# วิเคราะห์การทำงานของระบบชดเชยมุม Yaw

**วันที่:** 11 กุมภาพันธ์ 2026  
**สถานะ:** ⚠️ พบปัญหาการ implement ที่ไม่ถูกต้อง

---

## 1. ระบบพิกัดของขาหุ่นยนต์ (Leg Frame)

จากการวิเคราะห์ code ใน `generate_elliptical_trajectory()`:

```python
px = direction * (-step_forward * np.cos(t))
py = home_y + lift_height * abs(np.sin(t))
temp_trajectory.append((px + home_x, py))
```

**ระบบพิกัด:**
- **X-axis**: ทิศทาง FORWARD/BACKWARD (ไปข้างหน้า/ข้างหลัง) ❌ ไม่ใช่ซ้าย-ขวา!
- **Y-axis**: ทิศทาง VERTICAL (ขึ้น-ลง)
- **Origin**: จุดยึดขาที่ตัวหุ่นยนต์

**พารามิเตอร์:**
- `step_forward`: ระยะก้าวไปข้างหน้า/ข้างหลัง (mm)
- `home_x`: ตำแหน่ง offset ในแกน X (forward/backward)
- `home_y`: ตำแหน่ง offset ในแกน Y (ความสูง)

---

## 2. ปัญหาที่พบในการ Implement ปัจจุบัน

### 2.1 การใช้งานที่ผิด: `lateral_offset` → `home_x`

**Code ปัจจุบัน** (line 193-220 ใน relative_position_control.py):

```python
def get_trajectory_for_velocity(v_body_y, leg_id, lateral_offset=0.0):
    # ...
    mirror_x = (leg_id in ['FR', 'RR'])
    
    # ❌ ชื่อตัวแปร "lateral_offset" แต่ใช้กับ home_x (forward/backward!)
    leg_lateral_offset = lateral_offset if mirror_x else -lateral_offset
    
    trajectory = generate_elliptical_trajectory(
        # ...
        home_x=DEFAULT_STANCE_OFFSET_X + leg_lateral_offset,  # ❌ ผิด!
        # ...
    )
```

### 2.2 ปัญหาตามมา

**ปัญหาที่ 1: ผิดแกน (Wrong Axis)**
- ตัวแปรชื่อ `lateral_offset` แต่นำไปใช้กับ `home_x` ซึ่งเป็นแกน **forward/backward**
- `lateral_offset` ควรหมายถึง การเลื่อนด้านข้าง (left-right) แต่จริงๆ มันเลื่อนไปข้างหน้า-ข้างหลัง!

**ปัญหาที่ 2: กลไกการเลี้ยวไม่ชัดเจน**
- การเปลี่ยน `home_x` จะเลื่อน trajectory ทั้งหมดไปข้างหน้าหรือข้างหลัง
- สำหรับ trot gait นี่อาจสร้างโมเมนต์หมุนได้ แต่เป็นวิธีที่ผิดและไม่ชัดเจน

**ปัญหาที่ 3: ความสับสนในการตั้งชื่อ**
- ชื่อตัวแปร `lateral_offset` ทำให้เข้าใจผิดว่าเป็นการเลื่อนด้านข้าง
- จริงๆ แล้วมันคือ `differential_forward_offset` มากกว่า

---

## 3. วิธีการที่ถูกต้องในการแก้ไข Yaw Drift

### 3.1 หลักการ: Differential Drive

เพื่อให้หุ่นยนต์เลี้ยว เราต้องสร้าง **ความแตกต่างของการก้าวไปข้างหน้า** ระหว่างขาซ้ายและขวา:

**เลี้ยวขวา (Turn Right):**
- ขาซ้าย (FL, RL): ก้าวไปข้างหน้า**มากกว่า** → `step_forward_left = base + offset`
- ขาขวา (FR, RR): ก้าวไปข้างหน้า**น้อยกว่า** → `step_forward_right = base - offset`
- ผลลัพธ์: หุ่นยนต์หมุนไปทางขวา

**เลี้ยวซ้าย (Turn Left):**
- ขาซ้าย (FL, RL): ก้าวไปข้างหน้า**น้อยกว่า** → `step_forward_left = base - offset`
- ขาขวา (FR, RR): ก้าวไปข้างหน้า**มากกว่า** → `step_forward_right = base + offset`
- ผลลัพธ์: หุ่นยนต์หมุนไปทางซ้าย

### 3.2 การแปลงจาก YawController Output

**YawController Convention** (จาก time_estimator.py):
```python
# error = target_yaw - current_yaw
# ถ้าหุ่นยนต์เอียงซ้าย (-yaw): error > 0 → correction > 0 (ต้องเลี้ยวขวาเพื่อแก้)
# ถ้าหุ่นยนต์เอียงขวา (+yaw): error < 0 → correction < 0 (ต้องเลี้ยวซ้ายเพื่อแก้)
```

**การแปลงเป็น step_forward_offset:**
- `correction > 0`: ต้องเลี้ยวขวา → ขาซ้ายก้าวมากกว่า, ขาขวาก้าวน้อยกว่า
- `correction < 0`: ต้องเลี้ยวซ้าย → ขาขวาก้าวมากกว่า, ขาซ้ายก้าวน้อยกว่า

---

## 4. วิธีแก้ไขที่แนะนำ

### Option 1: แก้ไขชื่อและใช้ `home_x` (Quick Fix)

**ข้อดี:** แก้ไขน้อย, อาจใช้งานได้ (ถ้าเดิมทีมันใช้งานได้)
**ข้อเสีย:** ชื่อตัวแปรสับสน, กลไกไม่ชัดเจน

```python
def get_trajectory_for_velocity(v_body_y, leg_id, yaw_correction=0.0):
    """
    Args:
        yaw_correction: Forward/backward offset for yaw correction (mm)
                       Positive = this leg steps MORE forward (helps turn right overall)
    """
    # Determine side
    is_left_side = (leg_id in ['FL', 'RL'])
    
    # Apply correction (left legs get positive offset to turn right)
    forward_offset = yaw_correction if is_left_side else -yaw_correction
    
    trajectory = generate_elliptical_trajectory(
        # ...
        home_x=DEFAULT_STANCE_OFFSET_X + forward_offset,
        # ...
    )
```

### Option 2: ใช้ `step_forward` differential (Recommended)

**ข้อดี:** ชัดเจน, ถูกต้องตามหลักการ
**ข้อเสีย:** ต้องแก้ code มากกว่า

```python
def get_trajectory_for_velocity(v_body_y, leg_id, yaw_correction=0.0):
    """
    Args:
        yaw_correction: Differential forward stepping for yaw control (mm)
                       Positive = turn right (left legs step more)
    """
    step_length, reverse = update_gait_from_velocity(v_body_y)
    
    # Determine side
    is_left_side = (leg_id in ['FL', 'RL'])
    
    # Apply differential stepping
    # Left legs: +correction to turn right
    # Right legs: -correction to turn right
    step_differential = yaw_correction if is_left_side else -yaw_correction
    adjusted_step = step_length + step_differential
    
    # Clamp to valid range
    adjusted_step = max(0, min(adjusted_step, GAIT_STEP_FORWARD * 1.5))
    
    mirror_x = (leg_id in ['FR', 'RR'])
    
    trajectory = generate_elliptical_trajectory(
        num_steps=TRAJECTORY_STEPS,
        lift_height=GAIT_LIFT_HEIGHT,
        step_forward=adjusted_step,  # ✅ ใช้ step_forward ที่แตกต่างกัน
        mirror_x=mirror_x,
        stance_ratio=SMOOTH_TROT_STANCE_RATIO,
        home_x=DEFAULT_STANCE_OFFSET_X,  # ✅ home_x คงที่
        home_y=DEFAULT_STANCE_HEIGHT,
        reverse=reverse
    )
    
    return trajectory
```

### Option 3: เพิ่มพารามิเตอร์ใหม่ `lateral_stance_offset` (Most Correct)

สร้างระบบพิกัดที่ชัดเจน:
- `home_x`: forward/backward offset
- `home_y`: vertical offset  
- เพิ่ม `lateral_offset` ใหม่สำหรับควบคุมความกว้างของ stance

```python
def generate_elliptical_trajectory(..., home_x=0.0, home_y=None, home_lateral=0.0):
    """
    home_x: Forward/backward offset
    home_y: Vertical height
    home_lateral: Lateral offset from leg root (ใช้กับระบบ body frame)
    """
    # ... implementation with proper coordinate transformation
```

---

## 5. การทดสอบที่แนะนำ

### Test 1: Static Stance Width Test
```python
# ทดสอบว่า home_x มีผลกับอะไร
# ตั้ง home_x ต่างกันระหว่างขาซ้าย-ขวา
# สังเกตว่าขาไปข้างหน้า-หลังหรือซ้าย-ขวา
```

### Test 2: Yaw Correction Direction Test
```python
# Set yaw_correction = +10mm (ควรเลี้ยวขวา)
# สังเกตทิศทางการเลี้ยว
# ถ้าเลี้ย��ซ้าย = เครื่องหมายผิด
```

### Test 3: Coordinate Frame Validation
```python
# วาดกราฟ trajectory ของแต่ละขา
# ตรวจสอบว่า X-axis คือ forward/backward จริงหรือไม่
```

---

## 6. สรุปและคำแนะนำ

### ปัญหาหลัก
1. ✅ **YawController ทำงานถูกต้อง** - PD controller และ convention ถูกต้อง
2. ❌ **การนำไปใช้ผิด** - `lateral_offset` ถูกใช้กับ `home_x` (forward/backward axis)
3. ⚠️ **ชื่อตัวแปรสับสน** - `lateral_offset` ไม่ใช่ lateral จริงๆ
4. ⚠️ **กลไกไม่ชัดเจน** - ไม่แน่ใจว่าการเปลี่ยน `home_x` จะสร้างการเลี้ยวจริงหรือไม่

### ขั้นตอนถัดไป
1. **ทดสอบระบบปัจจุบันก่อน**: อาจจะใช้งานได้ (โดยบังเอิญ) ถ้ากลไกการเปลี่ยน home_x สร้างโมเมนต์หมุน
2. **ถ้าไม่ได้ผล**: แก้ไขตาม Option 2 (step_forward differential)
3. **ถ้าใช้งานได้**: แก้ชื่อตัวแปรและเพิ่มคำอธิบายให้ชัดเจน (Option 1)

### การแก้ไขที่แนะนำ
**→ Option 2 (step_forward differential)** เพราะ:
- ชัดเจนที่สุด
- ถูกต้องตามหลักการ differential drive
- ง่ายต่อการ debug และปรับแต่ง
- ชื่อตัวแปรสื่อความหมายถูกต้อง

---

**ผู้วิเคราะห์:** GitHub Copilot  
**สถานะ:** รอการทดสอบและแก้ไข
