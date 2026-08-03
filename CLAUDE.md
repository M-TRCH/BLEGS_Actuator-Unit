# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

BLEGS quadruped robot — two tiers in one repo:

- **Firmware** (`src/`, `include/`, `lib/`, `platformio.ini`): one STM32G431CB board = one "Actuator Unit" driving one BLDC motor with an AS5047P encoder. The robot has 8 units (legs FL/FR/RL/RR × motors A/B). Voltage-mode sinusoidal commutation (inverse-Park + SVPWM, no current loop); position PID (currently P-only, Kp=0.08) nominally 5 kHz.
- **Host** (`python/`): Windows-first Python that talks to each unit over its own USB COM port and runs gait generation, kinematics, vision ground truth, and paper analysis. No ROS, no package structure — control scripts are standalone monoliths by design.
- `docs/` is bilingual Thai/English; roadmaps are living status docs in Thai; `docs/theory/` is Thai LaTeX (XeLaTeX + TH Sarabun New).

## Commands

Firmware — PlatformIO Core 6.1.19 is installed but **not on PATH**; use the full path (or the PlatformIO extension terminal):

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run              # build (single env: genericSTM32G431CB)
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t upload    # flash via ST-Link
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor   # serial monitor @ 921600
```

Python — **no requirements.txt/pyproject exists**; deps are prose in `python/README.md`: core `numpy matplotlib pyserial`; vision `opencv-python opencv-contrib-python`; analysis `pandas scipy`; ML compensation also needs `scikit-learn joblib`; simulation needs `pybullet`. The Python 3.12 on PATH has only numpy+matplotlib (verified 2026-07); the intended interpreter is a conda env (`.vscode/settings.json`) and `conda` is not on PATH in plain shells. `python -m py_compile <file>` works everywhere and is the cheapest check.

```powershell
python python/paper_testing/test_3/walk_test.py       # CURRENT hardware test (menu: [7] walk, [8]/[9] turns, [R] keyboard RC, [M] ML comp)
python python/control/relative_position_control.py    # its parent: menu-driven navigation control
python python/control/test_quadruped_control.py       # oldest demo; doubles as the shared library others import
```

**There are no automated tests.** Files named `*_test*.py` / `test_*.py` are interactive hardware drivers, not pytest (`test/` at root is PlatformIO's empty C++ test dir). To verify control changes without hardware: run the script with no motors attached — discovery finds nothing and it auto-enters `SIMULATION_MODE` (motor writes become no-ops, the full 50 Hz loop still runs); or stub `serial`/`msvcrt` in `sys.modules` and call functions directly.

## Architecture

**Wire protocol** — three hand-maintained copies: `include/protocol.h`+`src/protocol.cpp` (firmware), `python/control/test_quadruped_control.py`, and inlined again in `walk_test.py`. `docs/technical/PROTOCOL.md` is stale on load-bearing points (wrong example bytes, omits gear ratio, documents removed ASCII mode) — trust code over the doc.

- Framing `FE EE | type | len | payload≤32 | CRC16-LE`; CRC-16/MODBUS (poly 0xA001, init 0xFFFF) over type+len+payload only.
- One COM port per motor, 921600 8N1, no on-wire addressing; motor_id (1–8, stored in EEPROM) appears only in feedback. Discovery = open every COM port and PING. Leg map: FL={A:1,B:2}, FR={3,4}, RL={5,6}, RR={7,8}.
- Positions on the wire are **motor-shaft centi-degrees** (int32) = joint degrees × GEAR_RATIO(8) × 100.
- Live firmware handles only SET_GOAL and PING. All SET_GOAL modes (incl. S-curve) are executed as instant direct-position setpoint changes — `set_position_scurve()` durations are silently ignored by firmware. PING doubles as the motor start command; the reply is FB_STATUS (0x81), never FB_PONG.
- The BNO055 IMU sidecar protocol (COM22, `python/sensors/test_bno055_imu.py`, inlined IMUReader in walk_test.py) reuses the FE EE header but a **different CRC** (CCITT 0x1021, big-endian fields/CRC). Two CRC helpers coexist in walk_test.py on purpose — do not unify them.

**Host control flow** (same shape in walk_test.py and relative_position_control.py): 50 Hz loop → target velocity `v_body_y` (P-controller on remaining distance, or RC keys) → step length = |v|·0.6 s, capped → per-leg cubic-Bezier trajectory (TRAJECTORY_STEPS=30/cycle, trot phase offsets FR/RL=0.0, FL/RR=0.5; turning = differential step length left vs right, **positive yaw_correction = right turn**) → analytical five-bar IK `calculate_ik_no_ef` (L_AC=L_BD=105 mm, L_CE=L_DE=145 mm, motor spacing 85 mm, foot = joint E, home stance (0, −220) mm) → per-motor serial write. Odometry is dead reckoning only: commanded v × VELOCITY_CALIBRATION(3.04); changing gait geometry invalidates that constant. March-in-place = same Bezier generator with step_forward=0 and 2× lift height; walk/march hand `step_indices` to each other for phase continuity.

**Which file is canonical**

- `python/paper_testing/test_3/walk_test.py` — the current field test (newest work). Deliberately standalone: inlines protocol+IK+gait+navigation (~70% copied from the two below) with diverged tunables (lift 40 vs 15, step 35 vs 30).
- `python/control/relative_position_control.py` — walk_test's parent; still has IMU_ENABLED=True (stale — see below).
- `python/control/test_quadruped_control.py` — despite the name, the **shared library**: relative_position_control.py and standing_balance_control.py `import test_quadruped_control as tqc` and mutate its module globals. Access shared state as `tqc.leg_motors` / `tqc.motor_registry` (module attribute), never `from`-import those names — `register_leg_motors()` rebinds them.
- Legacy, unimported, kept in-tree: `Quadruped_Gait_Control*.py`, `Gait_Control_*.py`. The "Main"/"Legacy" table in `python/README.md` predates walk_test.py entirely; its lower two-thirds documents `test_gait_csv.py`, which no longer exists.
- Kinematics reference: `python/kinematics/Quadruped_IK_Test_No_EF.py`. The `_No_EF` variant (foot = joint E) matches current hardware; unsuffixed EF files model the older leg with a 40 mm foot link.
- PyBullet sim (`python/simulation/gait_control/`) validates the gait layer only — its URDF uses 2-DOF serial legs, not the five-bar linkage, and its gait constants have drifted from the hardware scripts.

**ML compensation pipeline** (paper experiment 2): `test_2/single_leg_xy_control.py` grid sweep + ArUco capture → `test_2/output/data/grid_log.csv` → `train_compensation_model.py` → models in `test_2/output/models/` → consumed cross-folder by walk_test.py's `[M]` toggle (default `model_poly4`; `.json` poly models are dependency-free, `.pkl` needs sklearn+joblib). Sign convention everywhere: command = target − predicted_error.

## Conventions that will bite you

- **Copy-paste is the architecture.** `BinaryMotorController` exists in 5+ files, `calculate_crc16` in 6, the five-bar IK in ~8. This is deliberate (walk_test.py's docstring says so). A fix to shared logic must be replicated per file — grep the function name across `python/` before calling a fix complete.
- **Same-named constants hold different values per script** (verified): GAIT_LIFT_HEIGHT 40/15/30, GAIT_STEP_FORWARD 35/30/50/60, DEFAULT_STANCE_HEIGHT −220/−200, SMOOTH_TROT_STANCE_RATIO 0.75/0.70/0.65. Never assume a tuning value transfers between files; edit the script you are actually running.
- Keyboard UX is `msvcrt` (Windows-only): SPACE pause, E e-stop, Q quit, non-blocking polls inside the 50 Hz loops. On non-Windows the in-motion keys silently disappear (menus fall back to `input()`); the RC mode refuses to run.
- RC mode ([R]) reads its **drive** keys (W/S/A/D plus Q/E pivot) as real key state via `GetAsyncKeyState` (`_rc_read_drive_keys`), not from the msvcrt event stream — Windows auto-repeats only the newest key, so chorded holds like W+A are invisible to an event-based deadman. Only gate on console focus when `GetConsoleWindow()` is *visible*: under ConPTY (Windows Terminal, VS Code) it returns a hidden window, and gating on it kills every drive key. msvcrt still handles the one-shot keys, and a watchdog falls back to the event deadman if the state path never sees a held key.
- **RC key map differs from the other modes**: Q/E pivot in place, so emergency stop is **[X]** and exit is **[ESC]** (elsewhere E is e-stop and Q quits). Pivot uses a latch (`pivot_active`) so a turn left over from an arc cannot spin the robot as the drive keys are released.
- RC turning is a **ratio** of step length (`RC_TURN_K`, tuned live with [I]/[K]) rather than a fixed mm bias, so the radius is roughly speed-independent: `R ≈ (BODY_WIDTH/2)·(S_out+S_in)/(S_out−S_in)`. k > 1.0 makes the inner legs step backwards for a tighter arc — `generate_bezier_trajectory` accepts a negative `step_forward` natively because its start/end x are symmetric about `home_x`. RC also runs with **zero tilt offset** (`RC_APPLY_STATIC_TRIM=False`): `STATIC_ROLL_TRIM_MM` belongs to modes 7/8/9 and rides on the [M] ML flag there.
- Configuration = editing top-of-file UPPERCASE constants (comments often Thai). COM ports are hard-coded (IMU=COM22); paper/vision scripts hard-code author-machine media paths (`D:\THESIS\...`). Only `apriltag_runway_tilt_check.py` has argparse.
- Logs: walk_test.py → `python/paper_testing/test_3/output/log/movement_±Nmm_<ts>.csv`; relative_position_control.py → repo-root `logs/`; 4 rows (one per leg) per logged tick, decimated 1:10 by LOG_RATE. Mode R logs as `movement_+0mm_*` with phase `RC`.
- The `apriltag_*` scripts in test_3 actually use **ArUco** DICT_6X6_250 (cv2.aruco), not AprilTag. Foot/end-effector marker is always ID 4. The physical runway has a misprinted duplicate ID-6 tag standing in for ID 7 — `--duplicate-id6-mode` (default right-as-7) exists solely for that.

## Hardware & safety facts (verified against source)

- **EMERGENCY STOP IS CURRENTLY NON-FUNCTIONAL END-TO-END.** `src/main.cpp` has the `PKT_CMD_EMERGENCY_STOP` cases commented out (the packet hits `default:` → ERR_UNKNOWN_COMMAND and the motor keeps holding at full torque); `include/protocol.h` specifies magic bytes DE AD BE EF + double confirmation; every Python `emergency_stop_all()` sends a single empty payload. Three-way contradiction. Pressing [E] only aborts the host loop — treat killing motor power as the only real e-stop.
- No overcurrent/thermal protection in the firmware power path (`SVPWM controller - NO SAFETY CHECKS`); current sense is telemetry-only; the only guard is the ±12 V PID output clamp on a 24 V bus.
- Every boot physically sweeps the motor (open-loop commutation-offset search) before the start gate; after start the unit servos to −90° joint (−720 motor-shaft deg). Motors must physically start near −90°, and every script parks them there on exit.
- **No IMU is installed on the real robot** (confirmed 2026-07). `IMU_ENABLED=False` in walk_test.py is permanent policy — yaw heading-hold and balance paths are inert but kept guarded; do not re-enable or build IMU-dependent features for hardware runs. relative_position_control.py still says True (stale).
- Provisioning a new unit: set `active=true` on exactly one `saveMotorDataToEEPROM(...)` line in `src/main.cpp` setup(), flash once (it saves then deliberately halts in `while(1)`), revert to false, reflash. At runtime the EEPROM (magic 0xBEEF1234), not that table, is authoritative for motor_id/offsets.
- Known unfixed firmware bugs: (1) a single non-0xFE byte at the RX buffer head wedges packet reception until power-cycle — nothing drains non-header bytes since the ASCII else-branch was commented out; (2) `findRotorOffset` in `src/motor_control.cpp` line 113 uses the `CCW` macro (always true) where the `ccw` parameter was intended, so the CW pass computes its threshold against the wrong constant.
- Pin ground truth is `include/system.h` (serial USART1 PA9/PA10, encoder CS PB12, current sense PA2/PA3, PWM PB0/PB1/PB13, start button PA1, NeoPixel PC13). The wiring-diagram section of `docs/guides/HARDWARE_SETUP.md` contradicts both the firmware and the doc's own pinout table (it would put the LED on a motor PWM pin and USB-serial on ADC pins) — never wire from that diagram.
- USB-serial: adapters must support 921600 (FTDI recommended); set Windows USB latency timer to 1 ms to avoid CRC errors.
