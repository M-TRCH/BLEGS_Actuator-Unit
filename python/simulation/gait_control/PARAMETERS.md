# Simulation Parameters — `sim_smooth_walk_600.py`

สรุปพารามิเตอร์ทั้งหมดที่ใช้ในสคริปต์จำลอง Command [7]: Smooth Walk +600mm

---

## 1. Robot Parameters (Five-bar Linkage, No EF Link)

พารามิเตอร์ระบุขนาดกลไก five-bar linkage — **ใช้อ้างอิงเท่านั้น** เนื่องจาก PyBullet คำนวณ IK จาก URDF โดยตรง

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `L_AC` | 105.0 | mm | Motor A → Joint C |
| `L_BD` | 105.0 | mm | Motor B → Joint D |
| `L_CE` | 145.0 | mm | Joint C → Foot E (ไม่มี EF link) |
| `L_DE` | 145.0 | mm | Joint D → Foot E (ไม่มี EF link) |
| `MOTOR_SPACING` | 85.0 | mm | ระยะห่างระหว่าง Motor A กับ Motor B |

---

## 2. Gait Parameters

### 2.1 Stance

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `STANCE_HEIGHT_MM` | −220.0 | mm | ตำแหน่งเท้าในแกน Y ของ leg frame (ลบ = ต่ำกว่ามอเตอร์) |
| `STANCE_OFFSET_X_MM` | 0.0 | mm | ออฟเซ็ตเท้าในแกน X |

### 2.2 Trajectory

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `GAIT_LIFT_HEIGHT_MM` | 15.0 | mm | ความสูงยกขาขณะเดิน (swing phase) |
| `MARCH_LIFT_HEIGHT_MM` | 30.0 | mm | ความสูงยกขาขณะเดินย่ำ (2× เดินปกติ) |
| `GAIT_STEP_FORWARD_MM` | 30.0 | mm | ระยะก้าวสูงสุดครึ่งหนึ่ง (half-excursion) |
| `TRAJECTORY_STEPS` | 30 | steps | จำนวนจุดต่อรอบ gait cycle |
| `STANCE_RATIO` | 0.75 | — | สัดส่วนเฟส stance (75% อยู่บนพื้น, 25% swing) |

### 2.3 Timing (Derived)

| Parameter | Value | Unit | Derivation |
|-----------|-------|------|------------|
| `UPDATE_RATE` | 50 | Hz | อัตราอัปเดต gait control loop |
| `GAIT_CYCLE_TIME` | 0.6 | s | `TRAJECTORY_STEPS / UPDATE_RATE` |
| `GAIT_DT` | 0.02 | s | `1 / UPDATE_RATE` |

### 2.4 Bézier Control Points

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lift_ratio` | 0.4 | Bézier CP1 horizontal position (swing เริ่มยก) |
| `land_ratio` | 0.6 | Bézier CP2 horizontal position (swing เริ่มลง) |
| CP height multiplier | 1.25× | ชดเชยที่ Bézier ไม่ผ่าน control point |

---

## 3. Navigation Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `NAV_V_MAX_MM_S` | 70.0 | mm/s | ความเร็วเดินสูงสุด |
| `NAV_KP` | 1.0 | — | P-gain ของ navigation controller |
| `NAV_TOL_MM` | 10.0 | mm | ระยะ tolerance ที่ถือว่าถึงเป้าหมาย |
| `NAV_TIMEOUT_S` | 60.0 | s | timeout เฟสเดิน |

---

## 4. Simulation Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `SIM_DT` | 1/240 ≈ 0.00417 | s | Physics timestep ของ PyBullet |
| `START_HEIGHT_M` | 0.30 | m | ความสูงเริ่มต้นของหุ่นยนต์ (Z) |
| `WALK_TARGET_MM` | 600.0 | mm | ระยะทางเป้าหมาย Command [7] |

### Phase Durations

| Phase | Parameter | Duration | Description |
|-------|-----------|----------|-------------|
| 0 — Warm-up | `WARMUP_DUR` | 2.0 s | ยืนนิ่งรอเซ็ตตัวภายใต้แรงโน้มถ่วง |
| 1 — Idle March | `MARCH_DUR` | 2.0 s | เดินย่ำอยู่กับที่ |
| 2 — Walking | — | distance-based | เดินหน้า 600mm (จบเมื่อถึงเป้าหรือ timeout) |
| 3 — Post March | `POST_MARCH_DUR` | 2.0 s | เดินย่ำอยู่กับที่หลังเดิน |
| 4 — Stand | `STAND_SETTLE_DUR` | 1.0 s | กลับท่ายืน |

---

## 5. Body Geometry (URDF)

ตำแหน่งข้อต่อ hip จาก URDF (หน่วย: เมตร)

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `HIP_X_FRONT` | +0.19875 | m | ตำแหน่ง X ของ FR/FL hip |
| `HIP_X_REAR` | −0.16 | m | ตำแหน่ง X ของ RR/RL hip |
| `HIP_Y` | 0.1535 | m | ออฟเซ็ตด้านข้าง (±) |
| `THIGH_OFFSET_Y` | 0.0235 | m | ออฟเซ็ตด้านข้างของ thigh joint จาก hip |

### Derived

| Parameter | Value | Unit | Derivation |
|-----------|-------|------|------------|
| `BASE_X_AVG` | ≈ 0.1794 | m | `(HIP_X_FRONT + |HIP_X_REAR|) / 2` — สมมาตรหน้า-หลัง |
| `_sz` | −0.220 | m | `STANCE_HEIGHT_MM / 1000` |
| `_fy_r` | −0.177 | m | `-(HIP_Y + THIGH_OFFSET_Y)` — ขาขวา |
| `_fy_l` | +0.177 | m | `+(HIP_Y + THIGH_OFFSET_Y)` — ขาซ้าย |

### Home Foot Positions (Body Frame, metres)

| Leg | X | Y | Z |
|-----|---|---|---|
| FR | +0.1794 | −0.177 | −0.220 |
| FL | +0.1794 | +0.177 | −0.220 |
| RR | −0.1794 | −0.177 | −0.220 |
| RL | −0.1794 | +0.177 | −0.220 |

---

## 6. Balance PD Gains

ควบคุมทรงตัวโดยแก้ไขตำแหน่งเป้าหมายของเท้าตาม pitch/roll ของ body

| Parameter | Value | Axis | Type |
|-----------|-------|------|------|
| `BAL_KP_PITCH` | 0.006 | Pitch → X correction | Proportional |
| `BAL_KD_PITCH` | 0.012 | Pitch → X correction | Derivative |
| `BAL_KP_ROLL` | 0.006 | Roll → Y correction | Proportional |
| `BAL_KD_ROLL` | 0.012 | Roll → Y correction | Derivative |

---

## 7. Joint Control Gains

PyBullet `setJointMotorControlArray` POSITION_CONTROL gains — ใช้คนละชุดระหว่างยืนกับเดิน

### Walking (Phase 1, 2, 3)

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `POS_GAIN_WALK` | 0.3 | — | Position gain (นุ่มนวลขึ้น) |
| `VEL_GAIN_WALK` | 0.5 | — | Velocity gain |
| `FORCE_WALK` | 9.0 | N·m | แรงบิดสูงสุด |

### Standing (Phase 0, 4)

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `POS_GAIN_STAND` | 0.5 | — | Position gain (แข็งกว่าเดิน) |
| `VEL_GAIN_STAND` | 0.7 | — | Velocity gain |
| `FORCE_STAND` | 10.0 | N·m | แรงบิดสูงสุด |

### IK Damping

| Parameter | Value | Description |
|-----------|-------|-------------|
| `JOINT_DAMPING` | 0.5 | Joint damping สำหรับ `calculateInverseKinematics` |

---

## 8. Trot Phasing

ขาทแยงมุมเคลื่อนพร้อมกัน (diagonal gait)

| Leg | Phase Offset | Pair |
|-----|-------------|------|
| FR | 0.0 | Pair A |
| RL | 0.0 | Pair A |
| FL | 0.5 | Pair B |
| RR | 0.5 | Pair B |

Initial step index = `offset × TRAJECTORY_STEPS` → FR=0, FL=15, RR=15, RL=0

---

## 9. Display & Recording

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `STATUS_PRINT_INTERVAL` | 1.0 | s | ความถี่พิมพ์สถานะขณะเดิน |
| `CAMERA_UPDATE_INTERVAL` | 0.3 | s | ความถี่อัปเดตกล้องตาม body |
| `VIDEO_RECORD` | False | — | เปิด/ปิดบันทึกวิดีโอ MP4 |
| `VIDEO_DIR` | `"videos"` | — | โฟลเดอร์เก็บวิดีโอ (relative to script) |
| `VIDEO_FPS` | 30 | fps | เฟรมเรตของวิดีโอ |
| `VIDEO_WIDTH` | 1280 | px | ความกว้างเฟรม |
| `VIDEO_HEIGHT` | 720 | px | ความสูงเฟรม |

---

## 10. Parameter Source Mapping

| Category | Source File |
|----------|------------|
| Linkage dimensions | `test_quadruped_control.py` |
| Gait & stance | `relative_position_control.py` |
| Bézier trajectory | `bezier_gait.py` |
| Trot phasing | `test_quadruped_control.py` → `get_gait_phase_offset()` |
| Balance PD gains | `gait_control_trot.py` |
| Joint control gains | `gait_control_trot.py` |
| Body geometry | `my_robot.urdf` |
| Navigation | `relative_position_control.py` |
