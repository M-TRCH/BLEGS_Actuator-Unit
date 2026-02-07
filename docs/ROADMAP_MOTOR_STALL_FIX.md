# 🔧 Roadmap: แก้ไขปัญหามอเตอร์ค้าง (Motor Stall Analysis)

> **วันที่วิเคราะห์:** 20 มกราคม 2026  
> **ปัญหาหลัก:** มอเตอร์ค้างเมื่อ error สะสมมาก (position control) หรือเคลื่อนที่ด้วยความเร็วสูง

---

## 📋 สารบัญ

1. [สรุปปัญหาที่พบ](#-สรุปปัญหาที่พบ)
2. [ปัญหาที่ 1: Integral Windup](#-ปัญหาที่-1-integral-windup-ไมสมบูรณ)
3. [ปัญหาที่ 2: ไม่มี Velocity Feedforward](#-ปัญหาที่-2-ไมมี-velocity-feedforward)
4. [ปัญหาที่ 3: Derivative Kick](#-ปัญหาที่-3-derivative-kick)
5. [ปัญหาที่ 4: Multi-turn Wrap Detection](#-ปัญหาที่-4-multi-turn-wrap-detection-ลมเหลวที่ความเร็วสูง)
6. [ปัญหาที่ 5: ไม่มี Current Limiting](#-ปัญหาที่-5-ไมมี-current-limiting)
7. [ลำดับการแก้ไขที่แนะนำ](#-ลำดับการแก้ไขที่แนะนำ)
8. [Checklist การทดสอบ](#-checklist-การทดสอบ)

---

## 🎯 สรุปปัญหาที่พบ

| # | ปัญหา | ไฟล์ที่เกี่ยวข้อง | ความรุนแรง | ความยากในการแก้ |
|---|-------|------------------|------------|-----------------|
| 1 | Integral Windup ไม่สมบูรณ์ | `include/pid_controller.h` | 🔴 สูง | ⭐⭐ ปานกลาง |
| 2 | ไม่มี Velocity Feedforward | `src/main.cpp`, `lib/scurve_profile/` | 🔴 สูง | ⭐⭐⭐ สูง |
| 3 | Derivative Kick | `include/pid_controller.h` | 🟡 กลาง | ⭐ ง่าย |
| 4 | Multi-turn Wrap Detection | `src/encoder.cpp` | 🟡 กลาง | ⭐⭐ ปานกลาง |
| 5 | ไม่มี Current Limiting | `src/main.cpp`, `src/motor_control.cpp` | 🟡 กลาง | ⭐⭐⭐ สูง |

---

## 🔴 ปัญหาที่ 1: Integral Windup ไม่สมบูรณ์

### 📍 ตำแหน่งโค้ด
**ไฟล์:** `include/pid_controller.h` บรรทัด 36-48

### 🔍 การวิเคราะห์

**โค้ดปัจจุบัน:**
```cpp
if (fabs(error) <= tolerance) 
{
    error = 0;
    integral = 0;
} 
else 
{
    integral += error * dt;  // ← ปัญหาอยู่ตรงนี้
}

// Limit integral value
if (integral_limit > 0) 
{
    if (integral > integral_limit) integral = integral_limit;
    else if (integral < -integral_limit) integral = -integral_limit;
}
```

**ปัญหา:**
1. **Integral สะสมแม้ output ถึง saturation** - เมื่อ `vq_cmd` ถึง `output_limit` (±10V) แต่ integral ยังคงสะสมต่อ
2. **Windup สูงสุด = 2000.0** - ค่านี้สูงมาก เมื่อ Ki = 0.02 → contribution สูงสุด = 40V (เกิน output_limit มาก!)
3. **เมื่อ error ลดลง integral ที่สะสมจะทำให้ overshoot** → oscillation → มอเตอร์ค้าง

**สถานการณ์ที่ทำให้เกิดปัญหา:**
- มอเตอร์ติดขัด (mechanical obstruction)
- Load สูงเกินกำลัง
- Setpoint เปลี่ยนแปลงมากกะทันหัน
- การเคลื่อนที่ระยะไกลด้วย S-curve

### ✅ แนวทางแก้ไข

**วิธีที่ 1: Back-calculation Anti-windup (แนะนำ)**
```cpp
float compute(float measured, float dt) 
{
    float error = setpoint - measured;

    // Calculate proportional and derivative terms first
    float P_term = Kp * error;
    float derivative = (error - previous_error) / dt;
    float D_term = Kd * derivative;
    previous_error = error;

    // Calculate unsaturated output
    float output_unsaturated = P_term + Ki * integral + D_term;
    
    // Saturate output
    float output;
    if (output_limit > 0) {
        output = constrain(output_unsaturated, -output_limit, output_limit);
    } else {
        output = output_unsaturated;
    }

    // Back-calculation: only integrate if not saturated
    // OR if error is helping to desaturate
    bool is_saturated = (output != output_unsaturated);
    bool error_helps_desaturate = (error * output_unsaturated < 0);
    
    if (!is_saturated || error_helps_desaturate) {
        integral += error * dt;
        if (integral_limit > 0) {
            integral = constrain(integral, -integral_limit, integral_limit);
        }
    }

    return output;
}
```

**วิธีที่ 2: ลด integral_limit ให้สอดคล้องกับ output_limit**
```cpp
// ปรับค่าใน motor_control.cpp
// integral_limit ควร = output_limit / Ki = 10.0 / 0.02 = 500.0
PIDController position_pid(0.04f, 0.02f, 0.0f, 500.0f, 10.0f, 0.5f);
```

**วิธีที่ 3: Conditional Integration (ง่ายที่สุด)**
```cpp
// เพิ่ม integral เฉพาะเมื่อ output ไม่ถึง limit
if (fabs(output) < output_limit * 0.95f) {
    integral += error * dt;
}
```

### 🧪 การทดสอบ

1. **ทดสอบ step response ขนาดใหญ่**
   - สั่ง setpoint จาก 0° ไป 360° (หลัง gear = 2880°)
   - ดู overshoot และ settling time
   
2. **ทดสอบ mechanical obstruction**
   - ล็อคมอเตอร์ไว้ 2-3 วินาที
   - ปล่อยและดูว่า overshoot มากแค่ไหน

3. **ดู integral value ผ่าน debug**
   ```cpp
   SystemSerial->print("Integral: ");
   SystemSerial->println(position_pid.integral);
   ```

### ⏱️ เวลาที่คาดว่าใช้
- แก้ไขโค้ด: 30 นาที
- ทดสอบ: 1-2 ชั่วโมง

---

## 🔴 ปัญหาที่ 2: ไม่มี Velocity Feedforward

### 📍 ตำแหน่งโค้ด
**ไฟล์:** `src/main.cpp` บรรทัด 413-420, `lib/scurve_profile/scurve_profile.cpp`

### 🔍 การวิเคราะห์

**โค้ดปัจจุบัน:**
```cpp
if (control_mode == POSITION_CONTROL_WITH_SCURVE) 
{
    float elapsed_time = (float)(current_time - start_scurve_time) / 1000000.0f;
    position_pid.setpoint = scurve.getPosition(elapsed_time);
    positionControl(readRotorAbsoluteAngle(WITH_ABS_OFFSET), &vq_cmd);
}
```

**ปัญหา:**
1. **PID ต้อง "ไล่ตาม" setpoint ที่เคลื่อนที่** - ทำให้เกิด tracking error ตลอดเวลา
2. **ที่ความเร็วสูง tracking error จะมากขึ้น** - Integral สะสม → windup
3. **ไม่มีการ "บอกล่วงหน้า"** ว่ามอเตอร์ควรหมุนเร็วแค่ไหน

**หลักการ Feedforward:**
```
vq_cmd = PID_output + Kv * velocity_reference + Ka * acceleration_reference
                      ↑ feedforward term       ↑ optional
```

### ✅ แนวทางแก้ไข

**ขั้นตอนที่ 1: เพิ่มฟังก์ชัน getVelocity() ใน ScurveProfile**

**ไฟล์:** `lib/scurve_profile/scurve_profile.h`
```cpp
class ScurveProfile 
{
public:
    // ... existing members ...
    
    float getVelocity(float t);  // เพิ่มฟังก์ชันใหม่
};
```

**ไฟล์:** `lib/scurve_profile/scurve_profile.cpp`
```cpp
float ScurveProfile::getVelocity(float t) 
{
    float dir = (q1 > q0) ? 1.0f : -1.0f;

    if (t < 0) return 0.0f;
    if (t > totalTime) return 0.0f;

    if (t < Ta) {
        // Acceleration phase: v = a * t
        return dir * amax * t;
    } else if (t < (Ta + Tv)) {
        // Constant velocity phase
        return dir * vmax;
    } else {
        // Deceleration phase: v = vmax - a * (t - Ta - Tv)
        float td = t - (Ta + Tv);
        return dir * (vmax - amax * td);
    }
}
```

**ขั้นตอนที่ 2: ใช้ Feedforward ใน main.cpp**

```cpp
// กำหนดค่า feedforward gain (ต้อง tune)
#define KV_FEEDFORWARD 0.01f  // deg/s -> voltage

// ในส่วน position control
if (control_mode == POSITION_CONTROL_WITH_SCURVE) 
{
    float elapsed_time = (float)(current_time - start_scurve_time) / 1000000.0f;
    position_pid.setpoint = scurve.getPosition(elapsed_time);
    
    // Calculate feedforward
    float velocity_ref = scurve.getVelocity(elapsed_time);
    float feedforward = velocity_ref * KV_FEEDFORWARD;
    
    // PID + Feedforward
    float pid_output;
    positionControl(readRotorAbsoluteAngle(WITH_ABS_OFFSET), &pid_output);
    vq_cmd = pid_output + feedforward;
    
    // Clamp total output
    vq_cmd = constrain(vq_cmd, -10.0f, 10.0f);
}
```

### 🔢 การคำนวณ KV_FEEDFORWARD

```
ความสัมพันธ์คร่าวๆ:
- Speed constant = 12.3 RPM/V
- แปลงเป็น deg/s/V = 12.3 * 360 / 60 = 73.8 deg/s/V
- ดังนั้น V/deg/s = 1/73.8 = 0.0135

เริ่มต้นที่ KV_FEEDFORWARD = 0.01f แล้ว tune
```

### 🧪 การทดสอบ

1. **ทดสอบ tracking error**
   - Plot: setpoint vs actual position ระหว่าง S-curve motion
   - ก่อนแก้: จะเห็น lag ตลอด
   - หลังแก้: lag ควรน้อยลงมาก

2. **ทดสอบความเร็วต่างๆ**
   - vmax = 1000, 2000, 4000 deg/s
   - ดูว่า tracking error สัมพันธ์กับความเร็วอย่างไร

### ⏱️ เวลาที่คาดว่าใช้
- แก้ไขโค้ด: 1 ชั่วโมง
- Tuning Kv: 1-2 ชั่วโมง

---

## 🟡 ปัญหาที่ 3: Derivative Kick

### 📍 ตำแหน่งโค้ด
**ไฟล์:** `include/pid_controller.h` บรรทัด 51

### 🔍 การวิเคราะห์

**โค้ดปัจจุบัน:**
```cpp
float derivative = (error - previous_error) / dt;
```

**ปัญหา:**
1. **Derivative kick:** เมื่อ setpoint เปลี่ยนกะทันหัน → error กระโดด → derivative spike
2. **Noise amplification:** Encoder noise ถูกขยายโดย derivative term
3. **ปัจจุบัน Kd = 0** ดังนั้นปัญหานี้ยังไม่เกิดขึ้น แต่ถ้าเพิ่ม Kd จะมีผล

### ✅ แนวทางแก้ไข

**วิธีที่ 1: Derivative on Measurement (แนะนำ)**
```cpp
// เก็บ previous_measured แทน previous_error
float previous_measured = 0.0f;

float compute(float measured, float dt) 
{
    float error = setpoint - measured;
    
    // Derivative on measurement (negative sign!)
    float derivative = -(measured - previous_measured) / dt;
    previous_measured = measured;
    
    // ... rest of PID
}
```

**วิธีที่ 2: Low-pass filter บน derivative**
```cpp
float alpha = 0.1f;  // filter coefficient
float derivative_raw = (error - previous_error) / dt;
derivative_filtered = alpha * derivative_raw + (1 - alpha) * derivative_filtered;
```

### 🧪 การทดสอบ

1. **ทดสอบ step change**
   - สั่ง setpoint แบบกะทันหัน (ไม่ใช้ S-curve)
   - ดู vq_cmd ไม่ควรมี spike

2. **เปิด Kd และทดสอบ**
   ```cpp
   // ลองเปิด Kd
   PIDController position_pid(0.04f, 0.02f, 0.001f, ...);
   ```

### ⏱️ เวลาที่คาดว่าใช้
- แก้ไขโค้ด: 15 นาที
- ทดสอบ: 30 นาที

---

## 🟡 ปัญหาที่ 4: Multi-turn Wrap Detection ล้มเหลวที่ความเร็วสูง

### 📍 ตำแหน่งโค้ด
**ไฟล์:** `src/encoder.cpp` บรรทัด 50-58

### 🔍 การวิเคราะห์

**โค้ดปัจจุบัน:**
```cpp
void updateMultiTurnTracking()
{
    float raw_angle_deg = raw_rotor_angle * RAW_TO_DEGREE;
    float delta = raw_angle_deg - last_raw_angle_deg;
    
    if (delta > 180.0f)         rotor_turns--;
    else if (delta < -180.0f)   rotor_turns++;
    
    last_raw_angle_deg = raw_angle_deg;
}
```

**ปัญหา:**

**การคำนวณความเร็วสูงสุดที่ถูกต้อง:**
```
- Sample rate = 10 kHz (SVPWM frequency)
- Sample period = 100 µs
- Maximum angle change = 180° per sample
- Maximum shaft speed = 180° / 100µs = 1,800,000 °/s = 5,000 RPM

- กับ Gear ratio 8:1:
  - Maximum output speed = 5000 / 8 = 625 RPM ที่ output shaft
  
- Nominal speed = 120 RPM (จาก motor_conf.h)
- ดังนั้น margin = 625 / 120 = 5.2x ← อาจไม่พอในกรณี overspeed!
```

**สถานการณ์ที่อาจเกิดปัญหา:**
- มอเตอร์ถูกหมุนด้วยมือเร็วๆ ขณะเครื่องทำงาน
- Free-fall ของ leg mechanism
- การ oscillation รุนแรงจาก unstable PID

### ✅ แนวทางแก้ไข

**วิธีที่ 1: ตรวจสอบ delta ที่ผิดปกติ**
```cpp
void updateMultiTurnTracking()
{
    float raw_angle_deg = raw_rotor_angle * RAW_TO_DEGREE;
    float delta = raw_angle_deg - last_raw_angle_deg;
    
    // Normal wrap detection (150-210 degrees)
    if (delta > 180.0f && delta < 210.0f) {
        rotor_turns--;
    } else if (delta < -180.0f && delta > -210.0f) {
        rotor_turns++;
    } else if (fabs(delta) >= 210.0f) {
        // Abnormal jump - possible encoder error or extremely high speed
        // Option 1: Skip this sample
        // Option 2: Set error flag
        // Option 3: Estimate based on previous velocity
        
        // สำหรับตอนนี้ใช้ skip
        last_raw_angle_deg = raw_angle_deg;
        return;  // Don't update turns
    }
    
    last_raw_angle_deg = raw_angle_deg;
}
```

**วิธีที่ 2: ใช้ velocity estimation**
```cpp
// เก็บ velocity estimate
static float estimated_velocity = 0.0f;  // deg/s

void updateMultiTurnTracking()
{
    float raw_angle_deg = raw_rotor_angle * RAW_TO_DEGREE;
    float delta = raw_angle_deg - last_raw_angle_deg;
    
    // Unwrap delta
    if (delta > 180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    
    // Calculate expected delta from velocity
    float expected_delta = estimated_velocity * 0.0001f;  // 100µs period
    
    // If actual delta is way off from expected, it might be encoder error
    if (fabs(delta - expected_delta) > 90.0f) {
        // Use expected delta instead
        delta = expected_delta;
    }
    
    // Update velocity estimate (low-pass filter)
    float measured_velocity = delta / 0.0001f;
    estimated_velocity = 0.9f * estimated_velocity + 0.1f * measured_velocity;
    
    // Update turns based on unwrapped delta
    // ... (implementation details)
    
    last_raw_angle_deg = raw_angle_deg;
}
```

### 🧪 การทดสอบ

1. **ทดสอบความเร็วสูง**
   - สั่ง vq_cmd = ±10V โดยตรง
   - ดู absolute angle ว่าถูกต้องหลังหมุนหลายรอบหรือไม่

2. **ทดสอบหมุนด้วยมือ**
   - หมุนมอเตอร์ด้วยมือเร็วๆ
   - ดูว่า position jump ผิดปกติหรือไม่

### ⏱️ เวลาที่คาดว่าใช้
- แก้ไขโค้ด: 30 นาที
- ทดสอบ: 1 ชั่วโมง

---

## 🟡 ปัญหาที่ 5: ไม่มี Current Limiting

### 📍 ตำแหน่งโค้ด
**ไฟล์:** `src/main.cpp`, `include/motor_conf.h`

### 🔍 การวิเคราะห์

**ข้อมูลจาก motor_conf.h:**
```cpp
#define NOMINAL_CURRENT             10.5f           // Nominal current in Amperes
#define STALL_CURRENT               25.0f           // Stall current in Amperes
#define PHASE_TO_PHASE_RESISTANCE   0.214f          // Ohms
```

**ปัญหา:**
1. **vq_cmd สูงสุด = 10V** → กระแสโดยประมาณ = 10V / 0.214Ω = 46.7A (เกิน stall current!)
2. **ไม่มีการตรวจสอบ** ว่ากระแสเกินหรือไม่
3. **Driver อาจ shutdown** โดยไม่มี feedback กลับมา

### ✅ แนวทางแก้ไข

**วิธีที่ 1: Limit vq_cmd ตามกระแส (Simple)**
```cpp
// คำนวณ voltage limit จาก current limit
// V_max = I_max * R = 10.5A * 0.214Ω = 2.25V (สำหรับ nominal)
// V_max = I_max * R = 25.0A * 0.214Ω = 5.35V (สำหรับ stall)

#define VQ_CMD_LIMIT 5.0f  // Conservative limit

// ใน main.cpp
vq_cmd = constrain(vq_cmd, -VQ_CMD_LIMIT, VQ_CMD_LIMIT);
```

**วิธีที่ 2: Current Feedback Loop (ซับซ้อนกว่า)**
```cpp
// อ่านกระแสจาก current sensor (ถ้ามี)
currentUpdate();
float current_mA = currentEstimateDC();

// ลด vq_cmd ถ้ากระแสเกิน
#define CURRENT_LIMIT_MA 10500.0f  // 10.5A

if (fabs(current_mA) > CURRENT_LIMIT_MA) {
    // Scale down vq_cmd
    float scale = CURRENT_LIMIT_MA / fabs(current_mA);
    vq_cmd *= scale;
}
```

**วิธีที่ 3: ปรับ output_limit ใน PID**
```cpp
// ลด output_limit ให้ปลอดภัย
// เดิม: output_limit = 10.0
// ใหม่: output_limit = 5.0 (หรือคำนวณจาก I_nominal * R)

PIDController position_pid(0.04f, 0.02f, 0.0f, 2000.0f, 5.0f, 0.5f);
```

### 🧪 การทดสอบ

1. **วัดกระแสจริง**
   - ใช้ clamp meter วัดกระแสที่สาย power
   - สั่ง vq_cmd = 5V และดูกระแส

2. **ทดสอบ stall condition**
   - ล็อคมอเตอร์ไว้
   - ดูกระแสและอุณหภูมิ

### ⏱️ เวลาที่คาดว่าใช้
- แก้ไขโค้ด: 20 นาที
- ทดสอบ: 1 ชั่วโมง (รวมวัดกระแส)

---

## 📊 ลำดับการแก้ไขที่แนะนำ

### Phase 1: Quick Wins (ทำก่อน - ผลกระทบสูง ความเสี่ยงต่ำ)

| ลำดับ | ปัญหา | เหตุผล |
|-------|--------|--------|
| 1.1 | ลด output_limit จาก 10.0 → 5.0 | ลดความเสี่ยง overcurrent ทันที |
| 1.2 | ลด integral_limit จาก 2000 → 500 | ลด windup severity ทันที |

```cpp
// แก้ไขใน motor_control.cpp บรรทัด 17
PIDController position_pid(0.04f, 0.02f, 0.0f, 500.0f, 5.0f, 0.5f);
//                                              ↑        ↑
//                                        ลดจาก 2000  ลดจาก 10
```

### Phase 2: Core Fixes (แก้ไขหลัก)

| ลำดับ | ปัญหา | เหตุผล |
|-------|--------|--------|
| 2.1 | Back-calculation anti-windup | แก้สาเหตุหลักของ windup |
| 2.2 | Derivative on measurement | เตรียมพร้อมสำหรับการเพิ่ม Kd |

### Phase 3: Performance Improvements (เพิ่มประสิทธิภาพ)

| ลำดับ | ปัญหา | เหตุผล |
|-------|--------|--------|
| 3.1 | Velocity feedforward | ปรับปรุง tracking ที่ความเร็วสูง |
| 3.2 | Multi-turn protection | ป้องกัน position error ถาวร |

### Phase 4: Advanced Protection

| ลำดับ | ปัญหา | เหตุผล |
|-------|--------|--------|
| 4.1 | Current feedback limiting | Protection loop สมบูรณ์ |

---

## ✅ Checklist การทดสอบ

### ก่อนเริ่มแก้ไข (Baseline)
- [ ] บันทึก step response ปัจจุบัน (setpoint 0° → 360°)
- [ ] บันทึก settling time
- [ ] บันทึก overshoot %
- [ ] บันทึก tracking error ระหว่าง S-curve

### หลังแก้ไขแต่ละ Phase

**Phase 1:**
- [ ] มอเตอร์ยังทำงานปกติ
- [ ] กระแสไม่เกิน nominal (10.5A)
- [ ] ไม่มี driver shutdown

**Phase 2:**
- [ ] Step response ดีขึ้น (overshoot ลดลง)
- [ ] ไม่มี oscillation เมื่อถึง setpoint
- [ ] ทดสอบ mechanical obstruction - ไม่มี violent overshoot

**Phase 3:**
- [ ] Tracking error ลดลงที่ความเร็วสูง
- [ ] Multi-turn position ถูกต้องหลังหมุนหลายรอบ

**Phase 4:**
- [ ] กระแส maintain อยู่ในขอบเขตที่กำหนด
- [ ] Stall current ไม่เกิน limit

---

## 📝 Notes

### Debug Commands
```cpp
// เพิ่มใน debug section ของ main.cpp
SystemSerial->print(position_pid.setpoint);
SystemSerial->print("\t");
SystemSerial->print(readRotorAbsoluteAngle(WITH_ABS_OFFSET));
SystemSerial->print("\t");
SystemSerial->print(position_pid.integral);
SystemSerial->print("\t");
SystemSerial->println(vq_cmd);
```

### Emergency Recovery
ถ้ามอเตอร์ค้างหรือสั่น:
1. กด Emergency Stop หรือ Power off
2. ลด vq_cmd limit ลง
3. ลด Ki ลง
4. เพิ่ม tolerance (deadband)

---

*เอกสารนี้สร้างขึ้นจากการ code review เมื่อ 20 มกราคม 2026*
