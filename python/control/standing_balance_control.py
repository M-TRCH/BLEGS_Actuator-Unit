"""
Standing Body Balance Control - Roll & Pitch Stabilization
Author: M-TRCH
Date: February 12, 2026
Version: 1.1 - Corrected leg offset mapping from physical testing

This script implements standing body balance control using IMU feedback
to keep the robot body level by adjusting individual leg heights.

IMPORTANT UPDATE (v1.1):
    Physical testing revealed INVERTED response on PITCH axis:
    - Roll: Works correctly with INVERT_ROLL = False
    - Pitch: Tilt FRONT → Robot raised REAR legs (inverted)
    
    Solution: INVERT_ROLL = False, INVERT_PITCH = True (set as default)
    This inverts only the pitch error sign in the PD controller.
    
    If corrections go wrong direction, toggle [R] for roll or [X] for pitch.

Architecture:
┌─────────────────────────────────────────────────────────────────┐
│                     IMU SENSOR (BNO055)                         │
│                 Roll & Pitch feedback (100 Hz)                  │
├─────────────────────────────────────────────────────────────────┤
│                   BALANCE CONTROLLER (50 Hz)                    │
│         PD Control on Roll & Pitch → Leg Height Offsets         │
│                                                                 │
│   dh_roll  = K_roll  * roll_error  + D_roll  * d(roll)/dt      │
│   dh_pitch = K_pitch * pitch_error + D_pitch * d(pitch)/dt     │
│                                                                 │
│   Leg Height Mapping:                                             │
│     FL: stance_height + dh_pitch - dh_roll                      │
│     FR: stance_height + dh_pitch + dh_roll                      │
│     RL: stance_height - dh_pitch - dh_roll                      │
│     RR: stance_height - dh_pitch + dh_roll                      │
│                                                                 │
│   NOTE: INVERT_ROLL=True, INVERT_PITCH=True by default         │
│   (Physical testing showed inverted response)                   │
├─────────────────────────────────────────────────────────────────┤
│                    INVERSE KINEMATICS (5-Bar)                   │
│              Foot Position → Motor Angles                       │
├─────────────────────────────────────────────────────────────────┤
│                     LOW-LEVEL LAYER (5 kHz)                     │
│              Motor Control (FOC/SVPWM) - Firmware               │
└─────────────────────────────────────────────────────────────────┘

Sign Convention:
    Roll:  Positive = right side down  → Raise right legs, Lower left legs
    Pitch: Positive = front down       → Raise front legs, Lower rear legs
    Height: Negative = lower (further from body)

Troubleshooting Inverted Corrections:
    DEFAULT: INVERT_ROLL = True, INVERT_PITCH = True
    (Based on physical testing showing inverted response)
    
    If balance corrections go the WRONG direction:
    1. Start balance control [B]
    2. Manually tilt robot and observe response
    3. Toggle inversion flags [R] for roll or [X] for pitch
    4. Test: RIGHT tilt should raise RIGHT legs (FR, RR)
    5. Test: FRONT tilt should raise FRONT legs (FL, FR)
    
    If robot behaves correctly from the start:
        Set INVERT_ROLL = False and INVERT_PITCH = False in code

Usage:
    python standing_balance_control.py

    Commands:
        [B] Start/Stop balance control
        [Z] Set IMU zero reference
        [+/-] Adjust PD gains
        [T] Tilt test (simulated disturbance)
"""

import numpy as np
import time
import threading
import sys
import os

# Add parent directory (python/) to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)  # Go up one level to python/
sys.path.insert(0, parent_dir)

# Import navigation components
from navigation.imu_reader import IMUReader, create_imu_reader

# Import the module to access global variables by reference
import test_quadruped_control as tqc

# Import constants and functions
from test_quadruped_control import (
    # Protocol constants
    BAUD_RATE, FAST_TIMEOUT,
    ControlMode, CONTROL_MODE,

    # Robot configuration
    BODY_LENGTH, BODY_WIDTH, MOTOR_SPACING,
    L_AC, L_BD, L_CE, L_DE, GEAR_RATIO,
    EXPECTED_MOTOR_IDS, DEFAULT_STANCE_HEIGHT, DEFAULT_STANCE_OFFSET_X,

    # Control parameters
    UPDATE_RATE,

    # Classes and functions
    BinaryMotorController,
    discover_motors, register_leg_motors, start_all_motors,
    get_motor_positions, calculate_ik_no_ef,
    smooth_move_to_home_position,
    emergency_stop_all,

    # Thread locks
    control_lock, viz_lock
)

# Windows keyboard input
if sys.platform == 'win32':
    import msvcrt

# ============================================================================
# BALANCE CONTROL PARAMETERS
# ============================================================================

# IMU connection
IMU_PORT = 'COM22'          # Serial port for IMU
IMU_BAUD = 921600           # IMU baud rate

# PD gains for roll stabilization
ROLL_K_P = 0.8              # Proportional gain (mm/deg)
ROLL_K_D = 0.05             # Derivative gain (mm/(deg/s))

# PD gains for pitch stabilization
PITCH_K_P = 0.8             # Proportional gain (mm/deg)
PITCH_K_D = 0.05            # Derivative gain (mm/(deg/s))

# Sign inversion flags (adjust based on IMU mounting & testing)
# DEFAULT: Only PITCH inverted based on physical testing results
# 
# Physical testing showed:
#   - Roll: Works correctly with INVERT_ROLL = False
#   - Pitch: Tilt front → robot raised REAR legs (inverted) → needs INVERT_PITCH = True
# 
# Solution: Invert pitch error sign to compensate
INVERT_ROLL = False         # DEFAULT: Normal (change to True if left-right is backwards)
INVERT_PITCH = True         # DEFAULT: Inverted (change to False if front-rear is backwards)

# Safety limits
MAX_HEIGHT_OFFSET = 30.0    # Maximum leg height offset (mm)
MAX_ROLL_ANGLE = 25.0       # Max roll before emergency stop (deg)
MAX_PITCH_ANGLE = 25.0      # Max pitch before emergency stop (deg)
MIN_LEG_HEIGHT = -260.0     # Lowest allowed foot position (mm)
MAX_LEG_HEIGHT = -160.0     # Highest allowed foot position (mm)

# Control loop
BALANCE_RATE = 50           # Balance control loop rate (Hz)
STATUS_PRINT_INTERVAL = 0.5 # Status print interval (seconds)

# Logging
ENABLE_LOGGING = False
LOG_DIR = "logs/"
LOG_RATE = 5                # Log every N control cycles

# Simulation & Debug
SIMULATION_MODE = False
DEBUG_BALANCE = False

# ============================================================================
# BALANCE CONTROLLER CLASS
# ============================================================================

class BalanceController:
    """
    PD Controller for body roll and pitch stabilization.

    Computes per-leg height offsets to keep the body level based on
    IMU roll and pitch feedback.

    Leg layout (top view):
        FL ──── FR        Front (+pitch = nose down)
         │      │
         │ Body │
         │      │
        RL ──── RR        Rear

    Control logic (negative feedback):
        error = current - target
        dh = Kp × error + Kd × d(error)/dt

    Height correction mapping:
        Roll (+)  = right side down → dh_roll  > 0 → raise right, lower left
        Pitch (+) = front side down → dh_pitch > 0 → raise front, lower rear

        Theoretical mapping:
        FL: +dh_pitch - dh_roll   (front-left)
        FR: +dh_pitch + dh_roll   (front-right)
        RL: -dh_pitch - dh_roll   (rear-left)
        RR: -dh_pitch + dh_roll   (rear-right)

    Example verification:
        If roll = +5° (right down), dh_roll = +4mm:
            FL: 0 - 4 = -4mm → height = -220 + (-4) = -224mm (lower left) ✓
            FR: 0 + 4 = +4mm → height = -220 + 4 = -216mm (raise right) ✓

    Physical testing correction:
        Testing revealed INVERTED response on PITCH axis only:
        - Roll: Works correctly (INVERT_ROLL = False)
        - Pitch: Tilt FRONT → Robot raised REAR legs (inverted)
        
        Solution: INVERT_ROLL = False, INVERT_PITCH = True (default)
        This inverts only the pitch error sign.

    Note: In our coordinate system height is negative (lower = more negative),
    so adding a positive offset raises the leg (less negative).
    """

    def __init__(self,
                 roll_Kp: float = ROLL_K_P, roll_Kd: float = ROLL_K_D,
                 pitch_Kp: float = PITCH_K_P, pitch_Kd: float = PITCH_K_D,
                 max_offset: float = MAX_HEIGHT_OFFSET):
        # PD gains
        self.roll_Kp = roll_Kp
        self.roll_Kd = roll_Kd
        self.pitch_Kp = pitch_Kp
        self.pitch_Kd = pitch_Kd
        self.max_offset = max_offset

        # Target orientation (level = 0°)
        self.target_roll = 0.0
        self.target_pitch = 0.0

        # State for derivative term
        self._prev_roll_error = 0.0
        self._prev_pitch_error = 0.0
        self._prev_time = None

        # Output (for logging / display)
        self.dh_roll = 0.0
        self.dh_pitch = 0.0

    def reset(self):
        """Reset controller state."""
        self._prev_roll_error = 0.0
        self._prev_pitch_error = 0.0
        self._prev_time = None
        self.dh_roll = 0.0
        self.dh_pitch = 0.0

    def set_target(self, roll_deg: float = 0.0, pitch_deg: float = 0.0):
        """Set target roll/pitch (normally 0 for level)."""
        self.target_roll = roll_deg
        self.target_pitch = pitch_deg

    def compute(self, current_roll: float, current_pitch: float,
                current_time: float = None) -> dict:
        """
        Compute per-leg height offsets from IMU roll/pitch.

        Args:
            current_roll: Current roll angle (deg), + = right down
            current_pitch: Current pitch angle (deg), + = front down
            current_time: Timestamp (seconds)

        Returns:
            dict: {leg_id: height_offset_mm} for each leg
        """
        if current_time is None:
            current_time = time.time()

        # Errors (negative feedback: error = current - target)
        # If roll = +5° (right down), error = +5 → dh_roll = (+) → raise right legs
        # If pitch = +5° (front down), error = +5 → dh_pitch = (+) → raise front legs
        roll_error = current_roll - self.target_roll
        pitch_error = current_pitch - self.target_pitch
        
        # Apply sign inversion (DEFAULT: True for both based on physical testing)
        # Physical testing showed inverted response, so error signs are inverted
        # to compensate. Toggle [R]/[X] if corrections go wrong direction.
        if INVERT_ROLL:
            roll_error = -roll_error
        if INVERT_PITCH:
            pitch_error = -pitch_error

        # Derivative
        dt = 0.0
        d_roll = 0.0
        d_pitch = 0.0
        if self._prev_time is not None:
            dt = current_time - self._prev_time
            if dt > 0:
                d_roll = (roll_error - self._prev_roll_error) / dt
                d_pitch = (pitch_error - self._prev_pitch_error) / dt

        # PD output
        self.dh_roll = self.roll_Kp * roll_error + self.roll_Kd * d_roll
        self.dh_pitch = self.pitch_Kp * pitch_error + self.pitch_Kd * d_pitch

        # Clamp
        self.dh_roll = np.clip(self.dh_roll, -self.max_offset, self.max_offset)
        self.dh_pitch = np.clip(self.dh_pitch, -self.max_offset, self.max_offset)

        # Store for next iteration
        self._prev_roll_error = roll_error
        self._prev_pitch_error = pitch_error
        self._prev_time = current_time

        # Compute per-leg offsets (original theoretical mapping)
        offsets = {
            'FL': +self.dh_pitch - self.dh_roll,
            'FR': +self.dh_pitch + self.dh_roll,
            'RL': -self.dh_pitch - self.dh_roll,
            'RR': -self.dh_pitch + self.dh_roll,
        }

        return offsets


# ============================================================================
# GLOBAL STATE
# ============================================================================

imu_reader = None
balance_controller = None

balance_running = False
balance_thread = None

# Logging
log_file = None
log_counter = 0

# ============================================================================
# LEG CONTROL HELPERS
# ============================================================================

def send_leg_angles(leg_id: str, theta_A: float, theta_B: float) -> bool:
    """Send motor angles (radians) to a leg."""
    angle_A_deg = np.rad2deg(theta_A)
    angle_B_deg = np.rad2deg(theta_B)

    if SIMULATION_MODE or leg_id not in tqc.leg_motors:
        if DEBUG_BALANCE:
            print(f"    [SIM] {leg_id}: A={angle_A_deg:+.1f}°, B={angle_B_deg:+.1f}°")
        return True

    motors = tqc.leg_motors[leg_id]
    success_A = motors['A'].set_position_direct(angle_A_deg)
    success_B = motors['B'].set_position_direct(angle_B_deg)

    if DEBUG_BALANCE:
        print(f"    [HW] {leg_id}: A={angle_A_deg:+.1f}°, B={angle_B_deg:+.1f}° -> {success_A and success_B}")

    return success_A and success_B


def update_leg_position(leg_id: str, x: float, y: float) -> bool:
    """Move a leg to target foot position (x, y) via IK."""
    P_A, P_B = get_motor_positions(leg_id)
    angles = calculate_ik_no_ef(np.array([x, y]), P_A, P_B)

    if np.isnan(angles).any():
        print(f"  ⚠️  IK failed for {leg_id} at ({x:.1f}, {y:.1f})")
        return False

    theta_A, theta_B = angles

    with viz_lock:
        tqc.leg_states[leg_id]['target_angles'] = [theta_A, theta_B]
        tqc.leg_states[leg_id]['target_pos'] = [x, y]

    return send_leg_angles(leg_id, theta_A, theta_B)


def move_to_stand_position() -> bool:
    """Move all legs to default standing position."""
    print("\n🦿 Moving to standing position...")
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        success = update_leg_position(leg_id, DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT)
        if not success:
            print(f"  ❌ Failed to move {leg_id}")
            return False
    print("  ✅ Standing position reached")
    return True

# ============================================================================
# LOGGING
# ============================================================================

def init_balance_log() -> bool:
    """Open a CSV log file for balance data."""
    global log_file, log_counter

    if not ENABLE_LOGGING:
        return False

    try:
        from datetime import datetime
        os.makedirs(LOG_DIR, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{LOG_DIR}balance_{timestamp}.csv"
        log_file = open(filename, 'w')
        log_file.write("time,roll,pitch,dh_roll,dh_pitch,"
                       "h_FL,h_FR,h_RL,h_RR\n")
        log_counter = 0
        print(f"  📝 Logging to: {filename}")
        return True
    except Exception as e:
        print(f"  ⚠️  Log init failed: {e}")
        return False


def log_balance_step(elapsed: float, roll: float, pitch: float,
                     dh_roll: float, dh_pitch: float,
                     heights: dict) -> None:
    """Write one line to the balance log."""
    global log_counter
    if not ENABLE_LOGGING or log_file is None:
        return

    log_counter += 1
    if log_counter % LOG_RATE != 0:
        return

    line = (f"{elapsed:.3f},{roll:+.2f},{pitch:+.2f},"
            f"{dh_roll:+.2f},{dh_pitch:+.2f},"
            f"{heights.get('FL', 0):.1f},{heights.get('FR', 0):.1f},"
            f"{heights.get('RL', 0):.1f},{heights.get('RR', 0):.1f}\n")
    log_file.write(line)
    log_file.flush()


def close_balance_log():
    """Close log file."""
    global log_file
    if log_file is not None:
        log_file.close()
        log_file = None
        print("  ✅ Balance log closed")

# ============================================================================
# BALANCE CONTROL LOOP
# ============================================================================

def balance_control_loop():
    """
    Main balance control loop (runs in its own thread).

    Reads IMU → computes PD corrections → sends leg commands at BALANCE_RATE Hz.
    """
    global balance_running

    print("  ⚖️  Balance control loop started")

    start_time = time.time()
    last_status_time = start_time
    cycle_count = 0

    init_balance_log()

    try:
        while balance_running:
            loop_start = time.time()
            elapsed = loop_start - start_time
            cycle_count += 1

            # ------ Read IMU ------
            if imu_reader and imu_reader.is_receiving_data():
                orientation = imu_reader.get_orientation()
                roll = orientation['roll']
                pitch = orientation['pitch']
            elif SIMULATION_MODE:
                # Simulated small oscillation for testing without IMU
                roll = 2.0 * np.sin(2 * np.pi * 0.3 * elapsed)
                pitch = 1.5 * np.sin(2 * np.pi * 0.2 * elapsed + 0.5)
            else:
                # No data - skip
                time.sleep(1.0 / BALANCE_RATE)
                continue

            # ------ Safety check ------
            if abs(roll) > MAX_ROLL_ANGLE or abs(pitch) > MAX_PITCH_ANGLE:
                print(f"\n🚨 SAFETY LIMIT! Roll={roll:+.1f}° Pitch={pitch:+.1f}°")
                print("    Emergency stop - balance deactivated")
                balance_running = False
                move_to_stand_position()
                break

            # ------ Compute PD correction ------
            offsets = balance_controller.compute(roll, pitch, loop_start)

            # ------ Apply to legs ------
            heights = {}
            for leg_id in ['FR', 'FL', 'RR', 'RL']:
                target_y = DEFAULT_STANCE_HEIGHT + offsets[leg_id]

                # Clamp to safe range
                target_y = np.clip(target_y, MIN_LEG_HEIGHT, MAX_LEG_HEIGHT)
                heights[leg_id] = target_y

                update_leg_position(leg_id, DEFAULT_STANCE_OFFSET_X, target_y)

            # ------ Logging ------
            log_balance_step(elapsed, roll, pitch,
                             balance_controller.dh_roll,
                             balance_controller.dh_pitch,
                             heights)

            # ------ Status print ------
            if loop_start - last_status_time >= STATUS_PRINT_INTERVAL:
                avg_rate = cycle_count / elapsed if elapsed > 0 else 0
                status_line = (f"  ⚖️  Roll:{roll:+5.1f}° Pitch:{pitch:+5.1f}° "
                               f"| dh_R:{balance_controller.dh_roll:+5.1f}mm "
                               f"dh_P:{balance_controller.dh_pitch:+5.1f}mm")
                
                # Add leg heights in debug mode
                if DEBUG_BALANCE:
                    status_line += (f" | FL:{heights['FL']:+.0f} FR:{heights['FR']:+.0f} "
                                   f"RL:{heights['RL']:+.0f} RR:{heights['RR']:+.0f}")
                
                status_line += f" | {avg_rate:.0f} Hz | {elapsed:.1f}s"
                print(status_line)
                last_status_time = loop_start

            # ------ Rate limit ------
            loop_duration = time.time() - loop_start
            sleep_time = (1.0 / BALANCE_RATE) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"\n  ⚠️  Balance loop error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        close_balance_log()
        print("  🛑 Balance control loop stopped")


def start_balance() -> bool:
    """Start the balance control loop in a background thread."""
    global balance_running, balance_thread, balance_controller

    if balance_running:
        print("  ⚠️  Balance already running!")
        return False

    # Verify IMU or simulation
    if not SIMULATION_MODE:
        if not (imu_reader and imu_reader.is_receiving_data()):
            print("  ❌ IMU not available - cannot start balance")
            print("  💡 Use simulation mode or connect IMU first")
            return False

    # Create / reset controller
    if balance_controller is None:
        balance_controller = BalanceController()
    balance_controller.reset()

    print("\n⚖️  Starting balance control...")
    print(f"    Roll  PD: Kp={balance_controller.roll_Kp:.2f}  Kd={balance_controller.roll_Kd:.3f}")
    print(f"    Pitch PD: Kp={balance_controller.pitch_Kp:.2f}  Kd={balance_controller.pitch_Kd:.3f}")
    print(f"    Max offset: ±{balance_controller.max_offset:.0f} mm")
    print(f"    Rate: {BALANCE_RATE} Hz")

    balance_running = True
    balance_thread = threading.Thread(target=balance_control_loop, daemon=True)
    balance_thread.start()

    print("  ✅ Balance control started")
    return True


def stop_balance() -> bool:
    """Stop the balance control loop and return to standing."""
    global balance_running, balance_thread

    if not balance_running:
        print("  ⚠️  Balance not running!")
        return False

    print("\n🛑 Stopping balance control...")
    balance_running = False

    if balance_thread and balance_thread.is_alive():
        balance_thread.join(timeout=2.0)

    time.sleep(0.2)
    move_to_stand_position()

    print("  ✅ Balance stopped - standing position restored")
    return True

# ============================================================================
# TILT DISTURBANCE TEST
# ============================================================================

def run_tilt_test(axis: str = 'roll', amplitude_deg: float = 5.0,
                  duration_s: float = 10.0):
    """
    Simulate a periodic tilt disturbance while balance control is active.
    
    This overrides the IMU target to inject a sinusoidal reference
    and observes how the controller tracks it.

    Args:
        axis: 'roll' or 'pitch'
        amplitude_deg: Tilt amplitude (degrees)
        duration_s: Duration of test (seconds)
    """
    if not balance_running:
        print("  ❌ Balance must be running before tilt test!")
        return

    print(f"\n🧪 Tilt Test: {axis.upper()} ±{amplitude_deg}° for {duration_s:.0f}s")
    print("    Press [Q] to abort test")

    start = time.time()
    freq = 0.25  # Hz (4 second period)

    try:
        while time.time() - start < duration_s:
            t = time.time() - start
            offset = amplitude_deg * np.sin(2 * np.pi * freq * t)

            if axis == 'roll':
                balance_controller.set_target(roll_deg=offset, pitch_deg=0.0)
            else:
                balance_controller.set_target(roll_deg=0.0, pitch_deg=offset)

            # Check for abort
            if sys.platform == 'win32' and msvcrt.kbhit():
                key = msvcrt.getch()
                if key.lower() == b'q':
                    print("    ⏹️  Tilt test aborted")
                    break

            time.sleep(0.05)

    finally:
        # Reset target to level
        balance_controller.set_target(0.0, 0.0)
        print("  ✅ Tilt test finished - target reset to level")

# ============================================================================
# INTERACTIVE MENU
# ============================================================================

def print_menu():
    """Print interactive control menu."""
    mode_str = "SIMULATION" if SIMULATION_MODE else "HARDWARE"
    debug_str = "ON" if DEBUG_BALANCE else "OFF"
    bal_str = "⚖️  ACTIVE" if balance_running else "🛑 STOPPED"

    imu_status = "❌ DISABLED"
    if imu_reader and imu_reader.is_connected():
        if imu_reader.is_receiving_data():
            o = imu_reader.get_orientation()
            imu_status = f"✅ R:{o['roll']:+.1f}° P:{o['pitch']:+.1f}° Y:{o['yaw']:+.1f}°"
        else:
            imu_status = "⚠️ NO DATA"
    elif SIMULATION_MODE:
        imu_status = "🔄 SIMULATED"

    gains = ""
    if balance_controller:
        gains = (f"Roll Kp={balance_controller.roll_Kp:.2f} Kd={balance_controller.roll_Kd:.3f} | "
                 f"Pitch Kp={balance_controller.pitch_Kp:.2f} Kd={balance_controller.pitch_Kd:.3f}")
    
    invert_status = f"Roll {'⟲' if INVERT_ROLL else '→'} Pitch {'⟲' if INVERT_PITCH else '→'}"

    print("\n" + "=" * 70)
    print(f"  STANDING BALANCE CONTROL - [{mode_str}] - Debug: {debug_str}")
    print(f"  Balance: {bal_str} | Direction: {invert_status}")
    print(f"  IMU: {imu_status}")
    if gains:
        print(f"  Gains: {gains}")
    print("=" * 70)
    print("  Balance Commands:")
    print("    [B] Start / Stop balance control")
    print("    [H] Move to home/stand position (balance off)")
    print("  Tilt Test:")
    print("    [T] Roll tilt test (sinusoidal disturbance)")
    print("    [Y] Pitch tilt test (sinusoidal disturbance)")
    print("  Gain Tuning:")
    print("    [1/2] Roll  Kp  -/+ 0.1")
    print("    [3/4] Roll  Kd  -/+ 0.01")
    print("    [5/6] Pitch Kp  -/+ 0.1")
    print("    [7/8] Pitch Kd  -/+ 0.01")
    print("    [0]   Reset gains to defaults")
    print("  Direction Tuning:")
    print("    [R] Toggle roll inversion (if correction goes wrong way)")
    print("    [X] Toggle pitch inversion (if correction goes wrong way)")
    print(f"        Current: Roll {'⟲INVERTED' if INVERT_ROLL else '→NORMAL'}, Pitch {'⟲INVERTED' if INVERT_PITCH else '→NORMAL'}")
    print(f"        Default: BOTH INVERTED (change in code if robot works correctly)")
    print("  IMU Commands:")
    print("    [Z] Set IMU zero reference")
    print("    [I] Show IMU status")
    print("  Other:")
    print("    [D] Toggle debug output")
    print("    [L] Toggle logging")
    print("    [P] Print current status")
    print("    [Q] Quit")
    print("=" * 70)


def _adjust_gain(name: str, current: float, delta: float, minimum: float = 0.0) -> float:
    """Adjust a gain value and print result."""
    new_val = max(minimum, current + delta)
    print(f"  🔧 {name}: {current:.3f} → {new_val:.3f}")
    return new_val


def interactive_mode():
    """Run the interactive control menu."""
    global balance_controller, DEBUG_BALANCE, ENABLE_LOGGING, INVERT_ROLL, INVERT_PITCH

    balance_controller = BalanceController()

    print("\n🎮 Interactive balance control mode")

    while True:
        print_menu()

        if sys.platform == 'win32':
            print("  Enter command: ", end='', flush=True)
            key = msvcrt.getch()
            print(key.decode('utf-8', errors='ignore'))

            if key.lower() == b'b':
                if balance_running:
                    stop_balance()
                else:
                    start_balance()

            elif key.lower() == b'h':
                if balance_running:
                    stop_balance()
                move_to_stand_position()

            elif key.lower() == b't':
                run_tilt_test(axis='roll')

            elif key.lower() == b'y':
                run_tilt_test(axis='pitch')

            elif key == b'1':
                balance_controller.roll_Kp = _adjust_gain("Roll Kp", balance_controller.roll_Kp, -0.1)
            elif key == b'2':
                balance_controller.roll_Kp = _adjust_gain("Roll Kp", balance_controller.roll_Kp, +0.1)
            elif key == b'3':
                balance_controller.roll_Kd = _adjust_gain("Roll Kd", balance_controller.roll_Kd, -0.01)
            elif key == b'4':
                balance_controller.roll_Kd = _adjust_gain("Roll Kd", balance_controller.roll_Kd, +0.01)
            elif key == b'5':
                balance_controller.pitch_Kp = _adjust_gain("Pitch Kp", balance_controller.pitch_Kp, -0.1)
            elif key == b'6':
                balance_controller.pitch_Kp = _adjust_gain("Pitch Kp", balance_controller.pitch_Kp, +0.1)
            elif key == b'7':
                balance_controller.pitch_Kd = _adjust_gain("Pitch Kd", balance_controller.pitch_Kd, -0.01)
            elif key == b'8':
                balance_controller.pitch_Kd = _adjust_gain("Pitch Kd", balance_controller.pitch_Kd, +0.01)
            elif key == b'0':
                balance_controller.roll_Kp = ROLL_K_P
                balance_controller.roll_Kd = ROLL_K_D
                balance_controller.pitch_Kp = PITCH_K_P
                balance_controller.pitch_Kd = PITCH_K_D
                print("  🔧 Gains reset to defaults")

            elif key.lower() == b'r':
                INVERT_ROLL = not INVERT_ROLL
                print(f"  🔄 Roll inversion: {'ON' if INVERT_ROLL else 'OFF'}")
                if INVERT_ROLL:
                    print("     WARNING: Enabling roll inversion - verify carefully!")
                    print("     Tilt right → robot will raise LEFT legs (inverted)")
                else:
                    print("     Tilt right → robot will raise RIGHT legs (DEFAULT behavior)")
                
            elif key.lower() == b'x':
                INVERT_PITCH = not INVERT_PITCH
                print(f"  🔄 Pitch inversion: {'ON' if INVERT_PITCH else 'OFF'}")
                if INVERT_PITCH:
                    print("     Tilt front down → robot will raise front legs (DEFAULT behavior)")
                else:
                    print("     WARNING: Disabling default inversion - verify carefully!")
                    print("     Tilt front down → robot should still raise front legs")

            elif key.lower() == b'z':
                if imu_reader and imu_reader.is_connected():
                    imu_reader.set_zero()
                    print("  ✅ IMU zero set")
                else:
                    print("  ⚠️  IMU not available")

            elif key.lower() == b'i':
                print("\n📡 IMU Status:")
                if imu_reader and imu_reader.is_connected():
                    o = imu_reader.get_orientation()
                    s = imu_reader.get_stats()
                    print(f"    Roll:  {o['roll']:+.2f}°")
                    print(f"    Pitch: {o['pitch']:+.2f}°")
                    print(f"    Yaw:   {o['yaw']:+.2f}°")
                    print(f"    Cal:   {'✅' if o['calibrated'] else '⚠️'}")
                    print(f"    Pkts:  {s['packets']}")
                    print(f"    CRC err: {s['crc_errors']}")
                elif SIMULATION_MODE:
                    print("    Simulated IMU (no hardware)")
                else:
                    print("    ❌ Not connected")

            elif key.lower() == b'd':
                DEBUG_BALANCE = not DEBUG_BALANCE
                print(f"\n🔧 Debug: {'ON' if DEBUG_BALANCE else 'OFF'}")

            elif key.lower() == b'l':
                ENABLE_LOGGING = not ENABLE_LOGGING
                print(f"\n📝 Logging: {'ON' if ENABLE_LOGGING else 'OFF'}")

            elif key.lower() == b'p':
                print("\n📊 Status:")
                print(f"    Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")
                print(f"    Balance: {'RUNNING' if balance_running else 'STOPPED'}")
                print(f"    Direction: Roll {'INVERTED' if INVERT_ROLL else 'NORMAL'}, Pitch {'INVERTED' if INVERT_PITCH else 'NORMAL'}")
                print(f"    Legs: {list(tqc.leg_motors.keys())}")
                print(f"    Stance: x={DEFAULT_STANCE_OFFSET_X}, y={DEFAULT_STANCE_HEIGHT}")
                if balance_controller:
                    print(f"    dh_roll:  {balance_controller.dh_roll:+.2f} mm")
                    print(f"    dh_pitch: {balance_controller.dh_pitch:+.2f} mm")

            elif key.lower() == b'q':
                print("\n👋 Goodbye!")
                break

            else:
                print("  ❓ Unknown command")

        else:
            # Non-Windows fallback
            cmd = input("  Enter command: ").strip().lower()
            if cmd == 'b':
                if balance_running:
                    stop_balance()
                else:
                    start_balance()
            elif cmd == 'h':
                if balance_running:
                    stop_balance()
                move_to_stand_position()
            elif cmd == 't':
                run_tilt_test(axis='roll')
            elif cmd == 'y':
                run_tilt_test(axis='pitch')
            elif cmd == 'r':
                INVERT_ROLL = not INVERT_ROLL
                print(f"  🔄 Roll inversion: {'ON' if INVERT_ROLL else 'OFF'}")
            elif cmd == 'x':
                INVERT_PITCH = not INVERT_PITCH
                print(f"  🔄 Pitch inversion: {'ON' if INVERT_PITCH else 'OFF'}")
            elif cmd == 'z':
                if imu_reader and imu_reader.is_connected():
                    imu_reader.set_zero()
                else:
                    print("  ⚠️  IMU not available")
            elif cmd == 'q':
                print("\n👋 Goodbye!")
                break
            else:
                print("  ❓ Unknown command (use b/h/t/y/r/x/z/q)")

# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """Main entry point."""
    global SIMULATION_MODE, imu_reader, balance_controller

    print("=" * 70)
    print("  BLEGS Standing Balance Control - Roll & Pitch Stabilization")
    print("=" * 70)
    print("  Version: 1.1 (Corrected offset mapping)")
    print("  Author:  M-TRCH")
    print("  Date:    February 12, 2026")
    print("=" * 70)
    print("  Balance Parameters:")
    print(f"    Roll  PD: Kp={ROLL_K_P}, Kd={ROLL_K_D}")
    print(f"    Pitch PD: Kp={PITCH_K_P}, Kd={PITCH_K_D}")
    print(f"    Direction: Roll {'INVERTED' if INVERT_ROLL else 'NORMAL'}, Pitch {'INVERTED' if INVERT_PITCH else 'NORMAL'}")
    print(f"    (Default: Roll NORMAL, Pitch INVERTED - change in code if needed)")
    print(f"    Max offset: ±{MAX_HEIGHT_OFFSET} mm")
    print(f"    Safety limits: Roll ±{MAX_ROLL_ANGLE}°, Pitch ±{MAX_PITCH_ANGLE}°")
    print(f"    Leg height range: [{MIN_LEG_HEIGHT}, {MAX_LEG_HEIGHT}] mm")
    print(f"  Stance:")
    print(f"    Height: {DEFAULT_STANCE_HEIGHT} mm")
    print(f"    Offset X: {DEFAULT_STANCE_OFFSET_X} mm")
    print(f"    Body: {BODY_LENGTH}×{BODY_WIDTH} mm")
    print(f"  Control rate: {BALANCE_RATE} Hz")
    print("=" * 70)

    # --- Step 1: IMU Connection ---
    print(f"\n📡 Connecting to IMU on {IMU_PORT}...")
    imu_reader = create_imu_reader(IMU_PORT, auto_connect=True)

    if imu_reader and imu_reader.is_connected():
        print("  ✅ IMU port opened")

        # Wait for data
        print("  ⏳ Waiting for IMU data...")
        data_ok = False
        for _ in range(20):
            if imu_reader.is_receiving_data():
                print("  ✅ IMU data streaming")
                data_ok = True
                break
            time.sleep(0.1)

        if not data_ok:
            print("  ⚠️  No IMU data - check wiring")
            imu_reader.disconnect()
            imu_reader = None
        else:
            # Wait for calibration
            print("  ⏳ Waiting for calibration...")
            for _ in range(30):
                if imu_reader.is_calibrated():
                    print("  ✅ IMU calibrated")
                    break
                time.sleep(0.1)
            else:
                print("  ⚠️  Not fully calibrated (continuing)")

            imu_reader.set_zero()
            time.sleep(0.2)
            o = imu_reader.get_orientation()
            print(f"  🧭 Zero set — Roll:{o['roll']:+.1f}° Pitch:{o['pitch']:+.1f}° Yaw:{o['yaw']:+.1f}°")
    else:
        print("  ❌ IMU not available")
        imu_reader = None

    # --- Step 2: Motor Discovery ---
    print("\n🔍 Discovering motors...")
    discovered_motors = discover_motors()

    if not discovered_motors:
        print("  ❌ No motors found!")
        print("  ⚠️  Running in SIMULATION MODE")
        SIMULATION_MODE = True

    # --- Step 3: Register Motors ---
    if discovered_motors:
        all_registered = register_leg_motors(discovered_motors)
        if not all_registered:
            print("  ⚠️  Not all motors registered")
            resp = input("  Continue anyway? (y/n): ")
            if resp.lower() != 'y':
                return
        print(f"  📋 Registered legs: {list(tqc.leg_motors.keys())}")

    # --- Step 4: Start Motors & Home ---
    if discovered_motors and tqc.leg_motors:
        print("\n🚀 Starting motors...")
        start_all_motors()

        for motor in tqc.motor_registry.values():
            motor.set_timeout(FAST_TIMEOUT)

        # Compute home angles
        home_angles = {}
        for leg_id in tqc.leg_motors:
            P_A, P_B = get_motor_positions(leg_id)
            angles = calculate_ik_no_ef(
                np.array([DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT]),
                P_A, P_B
            )
            home_angles[leg_id] = np.rad2deg(angles)

        print("\n🏠 Moving to home position...")
        smooth_move_to_home_position(tqc.leg_motors, home_angles, duration_s=3.0)
    else:
        SIMULATION_MODE = True
        print("  ⚠️  No motors - SIMULATION MODE")

    print(f"\n🎮 Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")
    
    # Direction check reminder
    print("\n" + "=" * 70)
    print("  ⚠️  DIRECTION CHECK REMINDER:")
    print("  DEFAULT: INVERT_ROLL=False, INVERT_PITCH=True")
    print("  (Based on physical testing: Roll normal, Pitch inverted)")
    print("")
    print("  Verify corrections work as expected:")
    print("    1. Start balance [B]")
    print("    2. Tilt robot RIGHT → Should RAISE right legs (FR, RR)")
    print("    3. Tilt robot FRONT DOWN → Should RAISE front legs (FL, FR)")
    print("    4. If opposite, toggle [R] or [X] to correct direction")
    print("=" * 70)

    # --- Step 5: Interactive loop ---
    try:
        interactive_mode()
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted")
    finally:
        # Stop balance if running
        if balance_running:
            stop_balance()

        # Motors to init position
        if tqc.motor_registry:
            print("\n🏠 Moving motors to init position (-90°)...")
            for motor_id, motor in tqc.motor_registry.items():
                try:
                    motor.set_position_direct(-90.0)
                except Exception as e:
                    print(f"  ⚠️  Motor {motor_id}: {e}")
            time.sleep(1.0)

        # Disconnect motors
        print("\n🔌 Disconnecting motors...")
        for motor in tqc.motor_registry.values():
            motor.disconnect()
        print("  ✅ Done")

        # Disconnect IMU
        if imu_reader and imu_reader.is_connected():
            print("📡 Disconnecting IMU...")
            imu_reader.disconnect()
            print("  ✅ Done")


if __name__ == "__main__":
    main()
