"""
Relative Position Control for Quadruped Robot - Y-axis Movement with Balance Control
Author: M-TRCH
Date: February 5, 2026
Updated: February 12, 2026 - Added Roll & Pitch Balance Control

This script implements relative position control for Y-axis movement (forward/backward)
with integrated body balance stabilization based on the hierarchical control architecture.

Architecture:
┌─────────────────────────────────────────────────────────────────┐
│                    HIGH-LEVEL LAYER (10-50 Hz)                  │
│               SimpleNavigationPlanner (move_relative)            │
├─────────────────────────────────────────────────────────────────┤
│                     MID-LEVEL LAYER (50 Hz)                     │
│  Gait Generator + Balance Controller + IK (5-Bar) + Yaw Control │
│                                                                 │
│  - Bezier curve trajectory generation (forward/backward)       │
│  - Roll & Pitch stabilization (IMU-based PD control)           │
│  - Differential stepping for yaw correction                     │
│  - Per-leg height offset computation                            │
├─────────────────────────────────────────────────────────────────┤
│                     LOW-LEVEL LAYER (5 kHz)                     │
│              Motor Control (FOC/SVPWM) - Firmware               │
├─────────────────────────────────────────────────────────────────┤
│                       FEEDBACK LOOP                              │
│              IMU (Roll, Pitch, Yaw) + Dead Reckoning            │
└─────────────────────────────────────────────────────────────────┘

Features:
    ✅ Relative position control (forward/backward movement)
    ✅ Yaw correction using differential stepping
    ✅ Roll & Pitch balance control (toggle with [C] key)
    ✅ Idle marching (stepping in place) with balance
    ✅ Smooth transition from marching to walking
    ✅ Velocity calibration for accurate positioning

Balance Control:
    - PD controller for roll and pitch stabilization
    - Per-leg height offsets to keep body level
    - Works during both walking and idle marching
    - Toggle on/off with [C] key in interactive mode
    - Default: DISABLED (enable in interactive menu)

Usage:
    python relative_position_control.py
    
    Commands:
        move_relative(+500)  - Move forward 500mm
        move_relative(-300)  - Move backward 300mm
        [C]                  - Toggle balance control
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
from navigation.simple_planner import SimpleNavigationPlanner
from navigation.time_estimator import TimeBasedEstimator, YawController
from navigation.imu_reader import IMUReader, create_imu_reader

# Import Bezier gait trajectory generator
from bezier_gait import generate_bezier_trajectory

# Import the module to access global variables by reference
import test_quadruped_control as tqc

# Import functions and classes (configurations are defined locally for standalone operation)
from test_quadruped_control import (
    # Protocol constants
    BAUD_RATE, FAST_TIMEOUT,
    ControlMode, CONTROL_MODE,
    
    # Robot configuration (import for reference, but override locally)
    EXPECTED_MOTOR_IDS,
    
    # Classes and functions
    BinaryMotorController,
    discover_motors, register_leg_motors, start_all_motors,
    get_motor_positions, calculate_ik_no_ef,
    get_gait_phase_offset,
    smooth_move_to_home_position,
    emergency_stop_all,
    
    # Thread locks
    control_lock, viz_lock
)

# NOTE: leg_motors, motor_registry, leg_states are accessed via tqc.* 
# because they are updated by register_leg_motors() in the other module

# Windows keyboard input
if sys.platform == 'win32':
    import msvcrt

# ============================================================================
# RELATIVE POSITION CONTROL PARAMETERS
# ============================================================================

# Robot physical configuration (STANDALONE - all parameters defined here)
# Five-bar linkage dimensions
L_AC = 105.0                # Length from motor A to point C (mm)
L_BD = 105.0                # Length from motor B to point D (mm)  
L_CE = 145.0                # Length from C to end-effector (mm)
L_DE = 145.0                # Length from D to end-effector (mm)
GEAR_RATIO = 8.0            # Motor gear ratio (8:1)

# Body dimensions
BODY_LENGTH = 200.0         # Body length (mm)
BODY_WIDTH = 170.0          # Body width (mm)
MOTOR_SPACING = 85.0        # Distance between motors A and B (mm)

# Motor configuration
MOTOR_INIT_ANGLE = -90.0    # Initial motor angle (degrees)

# Stance configuration
DEFAULT_STANCE_HEIGHT = -220.0    # Default standing height (mm, negative = below motor plane)
DEFAULT_STANCE_OFFSET_X = 0.0     # Default X offset for standing position (mm)

# Gait parameters (can be tuned for different walking characteristics)
GAIT_LIFT_HEIGHT = 15.0           # Default foot lift height during walking (mm)
GAIT_STEP_FORWARD = 30.0          # Maximum step length (mm)
UPDATE_RATE = 50                  # Control loop update rate (Hz)
TRAJECTORY_STEPS = 30             # Number of steps per gait cycle
SMOOTH_TROT_STANCE_RATIO = 0.75   # Stance phase ratio (0.5 = 50% stance, 50% swing)

# Navigation parameters (tunable)
NAV_V_MAX = 70.0           # Maximum velocity (mm/s)
NAV_K_P = 1.0              # Proportional gain
NAV_TOLERANCE = 10.0       # Position tolerance (mm)
NAV_TIMEOUT = 60.0         # Maximum time for movement (seconds)

# Calibration parameters (from testing - Feb 10, 2026)
# NOTE: Calibration applied in state_estimator, NOT in gait generation
# This maintains normal walking speed while correcting position estimation
VELOCITY_CALIBRATION = 3.04  # Actual movement is 3x of estimated (trajectory span × overlap)

# IMU parameters (added Feb 11, 2026)
IMU_PORT = 'COM22'              # Serial port for IMU
IMU_ENABLED = True              # Enable/disable IMU integration
YAW_K_P = 0.8                   # Yaw controller proportional gain (mm/deg) - reduced to prevent overcorrection
YAW_K_D = 0.01                  # Yaw controller derivative gain (mm/(deg/s)) - reduced for smoother response
YAW_MAX_CORRECTION = 15.0        # Maximum differential step (mm) - reduced to prevent aggressive turning

# Balance control parameters (added Feb 12, 2026)
BALANCE_ENABLED = False         # Enable/disable balance control (toggle with [C] key)
ROLL_K_P = 0.8                  # Roll proportional gain (mm/deg) - reduced for smoother response
ROLL_K_D = 0.03                 # Roll derivative gain (mm/(deg/s)) - reduced for smoother response
PITCH_K_P = 0.8                 # Pitch proportional gain (mm/deg) - reduced for smoother response
PITCH_K_D = 0.03                # Pitch derivative gain (mm/(deg/s)) - reduced for smoother response
MAX_HEIGHT_OFFSET = 20.0        # Maximum leg height offset from balance (mm)
INVERT_ROLL = False             # Invert roll correction direction if needed
INVERT_PITCH = True             # Invert pitch correction direction (default based on testing)

# Gait to velocity mapping
GAIT_CYCLE_TIME = TRAJECTORY_STEPS / UPDATE_RATE  # seconds per gait cycle

# Logging parameters
ENABLE_LOGGING = False      # Enable data logging to file
LOG_FILE_PATH = "logs/"     # Directory for log files
LOG_RATE = 10               # Log every N control cycles

# Simulation mode (auto-detected if no motors found)
SIMULATION_MODE = False
DEBUG_GAIT = False  # Set to True to see gait debug output

# ============================================================================
# BALANCE CONTROLLER CLASS
# ============================================================================

class BalanceController:
    """
    PD Controller for body roll and pitch stabilization during walking.
    
    Computes per-leg height offsets to keep the body level based on
    IMU roll and pitch feedback.
    
    Leg layout (top view):
        FL ──── FR        Front (+pitch = nose down)
         │      │
         │ Body │
         │      │
        RL ──── RR        Rear
    
    Height correction mapping:
        FL: stance_height + dh_pitch - dh_roll   (front-left)
        FR: stance_height + dh_pitch + dh_roll   (front-right)
        RL: stance_height - dh_pitch - dh_roll   (rear-left)
        RR: stance_height - dh_pitch + dh_roll   (rear-right)
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
        
        # Errors (negative feedback)
        roll_error = current_roll - self.target_roll
        pitch_error = current_pitch - self.target_pitch
        
        # Apply sign inversion if needed
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
        
        # Compute per-leg offsets
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

# Control components
nav_planner = None
state_estimator = None
imu_reader = None
yaw_controller = None
balance_controller = None

# Control state
control_running = False
control_paused = True

# Idle marching state
idle_marching = False
march_thread = None
march_step_indices = {'FR': 0, 'FL': 0, 'RR': 0, 'RL': 0}

# Trajectory caches (pre-computed for each step length)
trajectory_cache = {}

# Logging state
log_file = None
log_counter = 0
log_data = []  # Buffer for log data

# ============================================================================
# GAIT VELOCITY MAPPING
# ============================================================================

def update_gait_from_velocity(v_body_y: float) -> tuple:
    """
    Convert body velocity to gait parameters.
    
    This function maps the velocity command from the navigation planner
    to the gait generator parameters (step length and direction).
    
    NOTE: Calibration is NOT applied here to maintain proper walking speed.
    Calibration is applied in state_estimator instead.
    
    Args:
        v_body_y: Body frame Y velocity (mm/s)
                  Positive = forward, Negative = backward
    
    Returns:
        tuple: (step_length, reverse)
            step_length: Step length in mm
            reverse: True if walking backward
    """
    # Calculate step length from velocity (NO calibration here)
    # step_length ∝ v_body * gait_cycle_time
    step_length = abs(v_body_y) * GAIT_CYCLE_TIME
    
    # Limit step length to maximum
    step_length = min(step_length, GAIT_STEP_FORWARD)
    
    # Determine direction
    reverse = (v_body_y < 0)
    
    return step_length, reverse


def get_trajectory_for_velocity(v_body_y: float, leg_id: str, yaw_correction: float = 0.0) -> list:
    """
    Get or generate trajectory for given velocity with optional yaw correction.
    
    Uses Differential Step Forward method: left and right legs take different
    step lengths to create a turning moment for yaw correction.
    
    Args:
        v_body_y: Body frame Y velocity (mm/s)
        leg_id: Leg identifier ('FR', 'FL', 'RR', 'RL')
        yaw_correction: Differential forward stepping for yaw control (mm)
                       Positive = turn right (left legs step MORE forward)
                       Negative = turn left (right legs step MORE forward)
    
    Returns:
        List of (x, y) positions for the trajectory
    """
    step_length, reverse = update_gait_from_velocity(v_body_y)
    
    # Determine side (left or right)
    is_left_side = (leg_id in ['FL', 'RL'])
    
    # Apply differential stepping for yaw correction
    # Left legs: +correction to turn right, -correction to turn left
    # Right legs: -correction to turn right, +correction to turn left
    step_differential = yaw_correction if is_left_side else -yaw_correction
    adjusted_step = step_length + step_differential
    
    # Clamp to valid range (prevent negative or excessive steps)
    adjusted_step = max(5.0, min(adjusted_step, GAIT_STEP_FORWARD * 1.5))
    
    # Determine if this leg should mirror X (for right side legs)
    mirror_x = (leg_id in ['FR', 'RR'])
    
    # Generate trajectory with adjusted step length using Bezier curves
    # NOTE: Invert reverse parameter - Bezier convention is opposite to original elliptical
    trajectory = generate_bezier_trajectory(
        num_steps=TRAJECTORY_STEPS,
        lift_height=GAIT_LIFT_HEIGHT,
        step_forward=adjusted_step,  # ✅ Use differential step forward
        mirror_x=mirror_x,
        stance_ratio=SMOOTH_TROT_STANCE_RATIO,
        home_x=DEFAULT_STANCE_OFFSET_X,  # ✅ Keep home_x constant
        home_y=DEFAULT_STANCE_HEIGHT,
        reverse=not reverse  # ✅ INVERTED: Bezier uses opposite convention
    )
    
    return trajectory

# ============================================================================
# LEG CONTROL FUNCTIONS
# ============================================================================

def send_leg_angles(leg_id: str, theta_A: float, theta_B: float) -> bool:
    """
    Send motor angles to a leg.
    
    Args:
        leg_id: Leg identifier
        theta_A: Motor A angle in radians
        theta_B: Motor B angle in radians
    
    Returns:
        True if successful
    """
    # Convert radians to degrees
    angle_A_deg = np.rad2deg(theta_A)
    angle_B_deg = np.rad2deg(theta_B)
    
    # Simulation mode - just update state, don't send to real motors
    if SIMULATION_MODE or leg_id not in tqc.leg_motors:
        if DEBUG_GAIT:
            print(f"    [SIM] {leg_id}: A={angle_A_deg:+.1f}°, B={angle_B_deg:+.1f}°")
        return True
    
    # Hardware mode - send to actual motors
    motors = tqc.leg_motors[leg_id]
    
    # Send to motors
    success_A = motors['A'].set_position_direct(angle_A_deg)
    success_B = motors['B'].set_position_direct(angle_B_deg)
    
    if DEBUG_GAIT:
        print(f"    [HW] {leg_id}: A={angle_A_deg:+.1f}°, B={angle_B_deg:+.1f}° -> {success_A and success_B}")
    
    return success_A and success_B


def update_leg_position(leg_id: str, foot_position: tuple, balance_offset: float = 0.0) -> bool:
    """
    Update leg to target foot position using IK with optional balance correction.
    
    Args:
        leg_id: Leg identifier
        foot_position: Target (x, y) position in leg frame
        balance_offset: Height offset from balance controller (mm)
    
    Returns:
        True if successful
    """
    x, y = foot_position
    
    # Apply balance correction to Y position (height)
    # Note: height is negative, so adding positive offset raises the leg
    y_corrected = y + balance_offset
    
    P_A, P_B = get_motor_positions(leg_id)
    
    # Calculate IK
    angles = calculate_ik_no_ef(np.array([x, y_corrected]), P_A, P_B)
    
    if np.isnan(angles).any():
        print(f"  ⚠️  IK failed for {leg_id} at ({x:.1f}, {y_corrected:.1f})")
        return False
    
    theta_A, theta_B = angles
    
    # Update leg state
    with viz_lock:
        tqc.leg_states[leg_id]['target_angles'] = [theta_A, theta_B]
        tqc.leg_states[leg_id]['target_pos'] = [x, y_corrected]
    
    # Send to motors
    return send_leg_angles(leg_id, theta_A, theta_B)


def update_all_legs_gait(trajectories: dict, step_indices: dict, balance_offsets: dict = None) -> bool:
    """
    Update all legs based on their trajectories and current step indices.
    
    Args:
        trajectories: Dict of {leg_id: trajectory_list}
        step_indices: Dict of {leg_id: current_step_index}
        balance_offsets: Optional dict of {leg_id: height_offset_mm} from balance controller
    
    Returns:
        True if all successful
    """
    all_success = True
    
    if balance_offsets is None:
        balance_offsets = {'FR': 0.0, 'FL': 0.0, 'RR': 0.0, 'RL': 0.0}
    
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        if leg_id in trajectories and leg_id in step_indices:
            traj = trajectories[leg_id]
            idx = step_indices[leg_id] % len(traj)
            foot_pos = traj[idx]
            
            # Get balance offset for this leg
            offset = balance_offsets.get(leg_id, 0.0)
            
            success = update_leg_position(leg_id, foot_pos, balance_offset=offset)
            if not success:
                all_success = False
    
    return all_success


def stop_all_legs() -> None:
    """Stop all legs at current position."""
    print("  ⏹️  Stopping all legs...")
    # Motors will hold position due to PID control
    # Just need to stop sending new commands


def move_to_stand_position() -> bool:
    """
    Move all legs to standing position.
    
    Returns:
        True if successful
    """
    print("\n🦿 Moving to standing position...")
    
    stand_pos = (DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT)
    
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        success = update_leg_position(leg_id, stand_pos)
        if not success:
            print(f"  ❌ Failed to move {leg_id}")
            return False
    
    print("  ✅ Standing position reached")
    return True

# ============================================================================
# LOGGING FUNCTIONS
# ============================================================================

def init_logging(target_distance: float) -> bool:
    """
    Initialize logging system.
    
    Args:
        target_distance: Target distance for this movement
    
    Returns:
        True if logging initialized successfully
    """
    global log_file, log_data, log_counter
    
    if not ENABLE_LOGGING:
        return False
    
    try:
        import os
        from datetime import datetime
        
        # Create log directory if not exists
        os.makedirs(LOG_FILE_PATH, exist_ok=True)
        
        # Generate log filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{LOG_FILE_PATH}movement_{target_distance:+.0f}mm_{timestamp}.csv"
        
        # Open log file
        log_file = open(filename, 'w')
        
        # Write header
        log_file.write("time,target_y,current_y,error,v_y,step_length,")
        log_file.write("FR_idx,FL_idx,RR_idx,RL_idx\n")
        
        # Reset counter and buffer
        log_counter = 0
        log_data = []
        
        print(f"  📝 Logging to: {filename}")
        return True
        
    except Exception as e:
        print(f"  ⚠️  Logging init failed: {e}")
        return False


def log_control_step(elapsed_time: float, status: dict, v_body_y: float, 
                     step_length: float, step_indices: dict) -> None:
    """
    Log one control step.
    
    Args:
        elapsed_time: Elapsed time since start
        status: Navigation status dict
        v_body_y: Current velocity command
        step_length: Current step length
        step_indices: Current step indices for all legs
    """
    global log_file, log_counter, log_data
    
    if not ENABLE_LOGGING or log_file is None:
        return
    
    log_counter += 1
    
    # Log every LOG_RATE cycles
    if log_counter % LOG_RATE == 0:
        line = f"{elapsed_time:.3f},"
        line += f"{status['target_y']:.1f},{status['current_y']:.1f},"
        line += f"{status['error']:.1f},{v_body_y:+.2f},{step_length:.2f},"
        line += f"{step_indices['FR']},{step_indices['FL']},"
        line += f"{step_indices['RR']},{step_indices['RL']}\n"
        
        log_file.write(line)
        log_file.flush()  # Ensure data is written


def close_logging() -> None:
    """Close logging file."""
    global log_file
    
    if log_file is not None:
        log_file.close()
        log_file = None
        print("  ✅ Log file closed")

# ============================================================================
# IDLE MARCHING (STEPPING IN PLACE)
# ============================================================================

def march_in_place_loop():
    """
    Internal loop for marching in place.
    Runs in separate thread.
    """
    global idle_marching, march_step_indices
    
    print("  🚶 Idle march loop started")
    
    # Generate trajectories for stepping in place (step_forward = 0)
    # Use higher lift for more visible marching
    march_lift_height = GAIT_LIFT_HEIGHT * 2.0  # Double the lift height for idle march
    march_trajectories = {}
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        mirror_x = (leg_id in ['FR', 'RR'])
        march_trajectories[leg_id] = generate_bezier_trajectory(
            num_steps=TRAJECTORY_STEPS,
            lift_height=march_lift_height,
            step_forward=0.0,  # No forward movement, just lift up and down
            mirror_x=mirror_x,
            stance_ratio=SMOOTH_TROT_STANCE_RATIO,
            home_x=DEFAULT_STANCE_OFFSET_X,
            home_y=DEFAULT_STANCE_HEIGHT,
            reverse=False  # Doesn't matter for step_forward=0, but kept consistent
        )
    
    try:
        while idle_marching:
            loop_start = time.time()
            
            # Compute balance corrections if enabled
            balance_offsets = None
            if BALANCE_ENABLED and balance_controller is not None:
                if imu_reader and imu_reader.is_receiving_data():
                    orientation = imu_reader.get_orientation()
                    roll = orientation['roll']
                    pitch = orientation['pitch']
                    balance_offsets = balance_controller.compute(roll, pitch, loop_start)
            
            # Update all legs with balance corrections
            update_all_legs_gait(march_trajectories, march_step_indices, balance_offsets)
            
            # Advance step indices
            for leg_id in march_step_indices:
                march_step_indices[leg_id] = (march_step_indices[leg_id] + 1) % TRAJECTORY_STEPS
            
            # Rate limiting
            loop_duration = time.time() - loop_start
            sleep_time = (1.0 / UPDATE_RATE) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except Exception as e:
        print(f"  ⚠️  March loop error: {e}")
    finally:
        print("  🛑 Idle march loop stopped")


def start_idle_march(resume_from: dict = None) -> bool:
    """
    Start marching in place (stepping without moving forward).
    
    Args:
        resume_from: Optional dict of {leg_id: step_index} to resume from specific positions
                    (for smooth transition from walking). If None, will start from phase offsets.
    
    Returns:
        True if started successfully
    """
    global idle_marching, march_thread, march_step_indices
    
    if idle_marching:
        print("  ⚠️  Already marching!")
        return False
    
    print("\n🚶 Starting idle march (stepping in place)...")
    
    # Initialize or resume step indices
    if resume_from is not None:
        # Resume from provided positions (smooth transition)
        march_step_indices = resume_from.copy()
        print("  ✨ Resuming from current gait phase (smooth transition)")
    else:
        # Initialize with trot phase offsets (fresh start)
        for leg_id in march_step_indices:
            phase_offset = get_gait_phase_offset(leg_id, 'trot')
            march_step_indices[leg_id] = int(phase_offset * TRAJECTORY_STEPS)
    
    # Start marching thread
    idle_marching = True
    march_thread = threading.Thread(target=march_in_place_loop, daemon=True)
    march_thread.start()
    
    print("  ✅ Idle march started")
    print("  💡 Robot is now stepping in place")
    print("  💡 Use movement commands to transition to walking")
    return True


def stop_idle_march() -> bool:
    """
    Stop marching in place and return to standing position.
    
    Returns:
        True if stopped successfully
    """
    global idle_marching, march_thread
    
    if not idle_marching:
        print("  ⚠️  Not marching!")
        return False
    
    print("\n🛑 Stopping idle march...")
    
    # Stop the march loop
    idle_marching = False
    
    # Wait for thread to finish
    if march_thread and march_thread.is_alive():
        march_thread.join(timeout=2.0)
    
    # Move to standing position
    time.sleep(0.2)  # Brief pause
    move_to_stand_position()
    
    print("  ✅ Idle march stopped")
    return True

# ============================================================================
# MAIN CONTROL FUNCTION
# ============================================================================

def move_relative_y(target_distance_mm: float, timeout_s: float = NAV_TIMEOUT, 
                    transition_to_march: bool = False) -> bool:
    """
    Move relative distance on Y-axis (forward/backward).
    
    This is the main high-level control function that implements
    the hierarchical control architecture.
    
    If the robot is currently marching in place, it will smoothly
    transition to walking without stopping.
    
    Args:
        target_distance_mm: Distance to move (mm)
                           Positive = forward
                           Negative = backward
        timeout_s: Maximum time for movement (seconds)
        transition_to_march: If True, will transition to idle march after reaching target
                            instead of stopping (for smooth continuous movement)
    
    Returns:
        True if target reached, False if timeout or error
    """
    global nav_planner, state_estimator, idle_marching, march_thread, march_step_indices
    
    print("\n" + "="*70)
    print("  📍 RELATIVE POSITION CONTROL - Y-AXIS")
    print("="*70)
    print(f"  Target: {target_distance_mm:+.1f} mm")
    print(f"  Direction: {'Forward' if target_distance_mm > 0 else 'Backward'}")
    print(f"  v_max: {NAV_V_MAX} mm/s")
    print(f"  Timeout: {timeout_s:.1f} s")
    print(f"  Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")
    print(f"  Motors available: {len(tqc.leg_motors)} legs ({list(tqc.leg_motors.keys())})")
    
    # Check if transitioning from idle march
    transitioning_from_march = idle_marching
    if transitioning_from_march:
        print("  🔄 Transitioning from idle march to walking...")
        # Stop idle march thread but keep the current step indices
        idle_marching = False
        if march_thread and march_thread.is_alive():
            march_thread.join(timeout=1.0)
    
    print("="*70)
    
    # Initialize components if needed
    if nav_planner is None:
        nav_planner = SimpleNavigationPlanner(
            v_max=NAV_V_MAX,
            K_p=NAV_K_P,
            tolerance=NAV_TOLERANCE
        )
    
    # Initialize state estimator with IMU (if available from main())
    if state_estimator is None:
        state_estimator = TimeBasedEstimator(imu_reader=imu_reader)
    
    # Initialize balance controller if enabled
    global balance_controller
    if BALANCE_ENABLED and balance_controller is None:
        balance_controller = BalanceController()
        print(f"  ⚖️  Balance controller initialized (Roll Kp={ROLL_K_P}, Pitch Kp={PITCH_K_P})")
    
    # 1. Set target
    nav_planner.set_relative_target(target_distance_mm)
    state_estimator.start()
    
    # Set yaw target if controller available
    if yaw_controller is not None and state_estimator.has_imu():
        yaw_controller.set_target(state_estimator.get_yaw())
        print(f"  🎯 Yaw target set: {state_estimator.get_yaw():.1f}°")
    
    # Reset balance controller if enabled
    if BALANCE_ENABLED and balance_controller is not None:
        balance_controller.reset()
        balance_controller.set_target(0.0, 0.0)
        print(f"  ⚖️  Balance target set: Level (0° roll, 0° pitch)")
    
    # Initialize logging
    init_logging(target_distance_mm)
    
    start_time = time.time()
    last_status_time = start_time
    
    # Initialize step indices for each leg
    if transitioning_from_march:
        # Continue from current march positions for smooth transition
        step_indices = march_step_indices.copy()
        print("  ✨ Smooth transition - continuing from current gait phase")
    else:
        # Start from initial positions
        step_indices = {'FR': 0, 'FL': 0, 'RR': 0, 'RL': 0}
        
        # Apply phase offsets for trot gait
        for leg_id in step_indices:
            phase_offset = get_gait_phase_offset(leg_id, 'trot')
            step_indices[leg_id] = int(phase_offset * TRAJECTORY_STEPS)
    
    print("\n🚶 Starting movement...")
    print("  Press [SPACE] to pause/resume")
    print("  Press [E] for emergency stop")
    print("  Press [Q] to abort")
    
    global control_paused
    control_paused = False
    
    try:
        # 2. Main Control Loop
        while not nav_planner.is_target_reached():
            current_time = time.time()
            loop_start = current_time
            
            # Check timeout
            elapsed = current_time - start_time
            if elapsed > timeout_s:
                print(f"\n⚠️  Timeout reached! ({timeout_s:.1f}s)")
                return False
            
            # Check for keyboard input (non-blocking)
            if sys.platform == 'win32' and msvcrt.kbhit():
                key = msvcrt.getch()
                if key == b' ':
                    control_paused = not control_paused
                    if control_paused:
                        print("\n⏸️  PAUSED - Press [SPACE] to resume")
                    else:
                        print("\n▶️  RESUMED")
                elif key.lower() == b'e':
                    print("\n⚠️  EMERGENCY STOP!")
                    emergency_stop_all()
                    return False
                elif key.lower() == b'q':
                    print("\n⏹️  ABORTED by user")
                    return False
            
            # Skip update if paused
            if control_paused:
                time.sleep(0.05)
                continue
            
            # 2.1 Compute velocity command (Navigation Planner)
            v_body_y = nav_planner.compute_velocity()
            
            # Get step length for logging
            step_length, _ = update_gait_from_velocity(v_body_y)
            
            # 2.2 Compute yaw correction (if IMU available)
            yaw_correction = 0.0
            if yaw_controller is not None and state_estimator.has_imu():
                current_yaw = state_estimator.get_yaw()
                yaw_correction = yaw_controller.compute(current_yaw, current_time)
            
            # 2.3 Generate trajectories for current velocity with yaw correction
            trajectories = {}
            for leg_id in ['FR', 'FL', 'RR', 'RL']:
                trajectories[leg_id] = get_trajectory_for_velocity(v_body_y, leg_id, yaw_correction)
            
            # 2.3.5 Compute balance corrections (if enabled and IMU available)
            balance_offsets = None
            if BALANCE_ENABLED and balance_controller is not None:
                if imu_reader and imu_reader.is_receiving_data():
                    orientation = imu_reader.get_orientation()
                    roll = orientation['roll']
                    pitch = orientation['pitch']
                    balance_offsets = balance_controller.compute(roll, pitch, current_time)
                else:
                    # No IMU data, use zero offsets
                    balance_offsets = {'FR': 0.0, 'FL': 0.0, 'RR': 0.0, 'RL': 0.0}
            
            # 2.4 Update all legs with balance corrections
            update_all_legs_gait(trajectories, step_indices, balance_offsets)
            
            # 2.5 Advance step indices
            for leg_id in step_indices:
                step_indices[leg_id] = (step_indices[leg_id] + 1) % TRAJECTORY_STEPS
            
            # 2.6 Update state estimator with calibration
            # Apply calibration here: actual movement is 3x of what trajectory suggests
            state_estimator.update(v_body_y * VELOCITY_CALIBRATION, current_time)
            nav_planner.update_position(state_estimator.get_position())
            
            # 2.7 Log control data
            status = nav_planner.get_status()
            log_control_step(elapsed, status, v_body_y, step_length, step_indices)
            
            # 2.8 Print status periodically
            if current_time - last_status_time >= 0.5:
                est_status = state_estimator.get_status()
                status_msg = (f"  📊 Position: {status['current_y']:+.1f}/{status['target_y']:+.1f} mm "
                             f"| Progress: {status['progress']:.0f}% "
                             f"| v_y: {v_body_y:+.1f} mm/s")
                
                # Add yaw status if IMU available
                if state_estimator.has_imu():
                    yaw_err = state_estimator.get_yaw_error()
                    status_msg += f" | Yaw: {est_status['yaw']:+.1f}° (err: {yaw_err:+.1f}°)"
                
                # Add balance status if enabled
                if BALANCE_ENABLED and balance_controller is not None:
                    if imu_reader and imu_reader.is_receiving_data():
                        o = imu_reader.get_orientation()
                        status_msg += f" | R:{o['roll']:+.1f}° P:{o['pitch']:+.1f}°"
                
                status_msg += f" | Time: {elapsed:.1f}s"
                print(status_msg)
                last_status_time = current_time
            
            # 2.8 Wait for next cycle
            loop_duration = time.time() - loop_start
            sleep_time = (1.0 / UPDATE_RATE) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # 3. Target reached - stop or transition to march
        if transition_to_march:
            # Smooth transition: copy current step indices to march_step_indices
            march_step_indices = step_indices.copy()
            print("\n  🔄 Ready for smooth transition to idle march...")
        else:
            stop_all_legs()
        
        close_logging()
        
        final_pos = state_estimator.get_position()
        final_time = state_estimator.get_elapsed_time()
        
        print("\n" + "="*70)
        print("  ✅ TARGET REACHED!")
        print("="*70)
        print(f"  Final position: {final_pos:+.1f} mm")
        print(f"  Target: {target_distance_mm:+.1f} mm")
        print(f"  Error: {abs(target_distance_mm - final_pos):.1f} mm")
        print(f"  Time: {final_time:.2f} s")
        print(f"  Avg velocity: {state_estimator.get_average_velocity():.1f} mm/s")
        
        # Show yaw drift if IMU available
        if state_estimator.has_imu():
            final_yaw_error = state_estimator.get_yaw_error()
            print(f"  Yaw error: {final_yaw_error:+.1f}° (drift from straight line)")
        
        print("="*70)
        
        return True
        
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
        close_logging()
        return False
    
    finally:
        # Ensure motors stop
        stop_all_legs()
        close_logging()

# ============================================================================
# TEST FUNCTIONS
# ============================================================================

def test_short_forward():
    """Test: Move forward 100mm"""
    print("\n" + "="*70)
    print("  TEST 1: Short Forward Movement (+100mm)")
    print("="*70)
    return move_relative_y(+100.0, timeout_s=30.0)


def test_short_backward():
    """Test: Move backward 100mm"""
    print("\n" + "="*70)
    print("  TEST 2: Short Backward Movement (-100mm)")
    print("="*70)
    return move_relative_y(-100.0, timeout_s=30.0)


def test_backward_200():
    """Test: Move backward 200mm"""
    print("\n" + "="*70)
    print("  TEST 4: Medium Backward Movement (-200mm)")
    print("="*70)
    return move_relative_y(-200.0, timeout_s=30.0)


def test_long_forward():
    """Test: Move forward 500mm"""
    print("\n" + "="*70)
    print("  TEST 3: Long Forward Movement (+500mm)")
    print("="*70)
    return move_relative_y(+500.0, timeout_s=60.0)


def test_smooth_walk_600():
    """Test: Smooth walk with marching transition - 600mm"""
    print("\n" + "="*70)
    print("  TEST 7: Smooth Walk with Idle March (+600mm)")
    print("="*70)
    print("  Sequence:")
    print("    1. Idle march for 2 seconds")
    print("    2. Smooth transition to walking forward 600mm")
    print("    3. Smooth transition to idle march for 2 seconds")
    print("    4. Return to standing position")
    print("="*70)
    
    try:
        # Step 1: Start idle marching
        print("\n  Step 1/4: Starting idle march...")
        if not start_idle_march():
            print("  ❌ Failed to start idle march")
            return False
        
        # March for 2 seconds
        print("  🚶 Marching in place for 2 seconds...")
        time.sleep(2.0)
        
        # Step 2: Walk forward 600mm with smooth transition TO march at end
        print("\n  Step 2/4: Transitioning to walking forward 600mm...")
        success = move_relative_y(+600.0, timeout_s=60.0, transition_to_march=True)
        
        if not success:
            print("  ❌ Walking failed")
            # Try to stop cleanly
            if idle_marching:
                stop_idle_march()
            return False
        
        # Step 3: Transition directly to idle march (no pause, smooth transition)
        print("\n  Step 3/4: Transitioning to idle march...")
        # Use global march_step_indices that was set by move_relative_y()
        if not start_idle_march(resume_from=march_step_indices):
            print("  ❌ Failed to start idle march")
            return False
        
        # March for 2 seconds
        print("  🚶 Marching in place for 2 seconds...")
        time.sleep(2.0)
        
        # Step 4: Return to standing position
        print("\n  Step 4/4: Returning to standing position...")
        if not stop_idle_march():
            print("  ❌ Failed to stop idle march")
            return False
        
        print("\n" + "="*70)
        print("  ✅ SMOOTH WALK TEST COMPLETED!")
        print("="*70)
        print("  Total sequence completed successfully:")
        print("    ✓ Pre-walk idle march (2s)")
        print("    ✓ Forward walk (600mm)")
        print("    ✓ Post-walk idle march (2s)")
        print("    ✓ Return to standing")
        print("="*70)
        
        return True
        
    except Exception as e:
        print(f"\n  ❌ Test failed with error: {e}")
        # Cleanup
        if idle_marching:
            stop_idle_march()
        return False


def run_test_sequence():
    """Run full test sequence with user confirmation between tests."""
    
    tests = [
        ("Short Forward (+100mm)", test_short_forward),
        ("Short Backward (-100mm)", test_short_backward),
        ("Long Forward (+500mm)", test_long_forward),
    ]
    
    results = []
    
    for i, (name, test_func) in enumerate(tests):
        print(f"\n{'='*70}")
        print(f"  Ready to run: {name}")
        print(f"  Press [ENTER] to start, [S] to skip, [Q] to quit")
        print(f"{'='*70}")
        
        while True:
            if sys.platform == 'win32':
                key = msvcrt.getch()
                if key == b'\r':  # Enter
                    break
                elif key.lower() == b's':
                    print("  ⏭️  Skipped")
                    results.append((name, "SKIPPED"))
                    break
                elif key.lower() == b'q':
                    print("  ⏹️  Test sequence aborted")
                    return results
            else:
                input()
                break
        
        if results and results[-1][0] == name:  # Was skipped
            continue
        
        # Run test
        success = test_func()
        results.append((name, "PASSED" if success else "FAILED"))
        
        # Wait before next test
        print("\n  Waiting 3 seconds before next test...")
        time.sleep(3.0)
    
    # Print summary
    print("\n" + "="*70)
    print("  TEST RESULTS SUMMARY")
    print("="*70)
    for name, result in results:
        icon = "✅" if result == "PASSED" else ("⏭️" if result == "SKIPPED" else "❌")
        print(f"  {icon} {name}: {result}")
    print("="*70)
    
    return results

# ============================================================================
# INTERACTIVE MENU
# ============================================================================

def print_menu():
    """Print interactive menu."""
    mode_str = "SIMULATION" if SIMULATION_MODE else "HARDWARE"
    debug_str = "ON" if DEBUG_GAIT else "OFF"
    march_status = "🚶 MARCHING" if idle_marching else "🧍 STANDING"
    
    # Get IMU status
    imu_status = "❌ DISABLED"
    balance_status = "🛑 OFF"
    if IMU_ENABLED:
        if imu_reader and imu_reader.is_connected():
            if imu_reader.is_receiving_data():
                cal_status = "✅" if imu_reader.is_calibrated() else "⚠️"
                current_yaw = imu_reader.get_yaw()
                imu_status = f"✅ ACTIVE {cal_status} (Yaw: {current_yaw:+.1f}°)"
                
                # Show balance status if IMU available
                if BALANCE_ENABLED:
                    o = imu_reader.get_orientation()
                    balance_status = f"⚖️  ON (R:{o['roll']:+.1f}° P:{o['pitch']:+.1f}°)"
                else:
                    balance_status = "🛑 OFF"
            else:
                imu_status = "⚠️ CONNECTED BUT NO DATA"
        else:
            imu_status = "⚠️ NOT CONNECTED"
    
    print("\n" + "="*70)
    print(f"  RELATIVE POSITION CONTROL - [{mode_str}] - Debug: {debug_str}")
    print(f"  Status: {march_status}")
    print(f"  IMU: {imu_status}")
    print(f"  Balance: {balance_status}")
    print("="*70)
    print("  Movement Commands:")
    print("    [1] Move forward +100mm")
    print("    [2] Move backward -100mm")
    print("    [3] Move forward +500mm")
    print("    [4] Custom distance (enter value)")
    print("    [5] Run test sequence")
    print("    [6] Test backward -200mm (Roadmap 7.1)")
    print("    [7] Smooth walk +600mm with march transitions")
    print("  Idle March Commands:")
    print("    [M] Start marching in place (step without moving)")
    print("    [S] Stop marching (return to stand)")
    print("  Balance Commands:")
    print("    [C] Toggle balance control (stabilize roll & pitch)")
    print("  IMU Commands:")
    print("    [Z] Set IMU zero (reset yaw reference)")
    print("    [I] Show IMU status")
    print("  Other Commands:")
    print("    [H] Move to home/stand position")
    print("    [P] Print current status")
    print("    [D] Toggle debug output")
    print("    [L] Toggle logging")
    print("    [Q] Quit")
    print("="*70)
    if idle_marching:
        print("  💡 TIP: Movement commands will smoothly transition from marching")
        print("="*70)


def interactive_mode():
    """Run interactive control mode."""
    global nav_planner, state_estimator, DEBUG_GAIT, ENABLE_LOGGING
    
    # Initialize navigation planner
    nav_planner = SimpleNavigationPlanner(
        v_max=NAV_V_MAX,
        K_p=NAV_K_P,
        tolerance=NAV_TOLERANCE
    )
    
    # Initialize state estimator with IMU (already connected in main())
    state_estimator = TimeBasedEstimator(imu_reader=imu_reader)
    
    print("\n🎮 Interactive mode started")
    if imu_reader and imu_reader.is_connected():
        if imu_reader.is_receiving_data():
            print(f"  📡 IMU: Connected & Streaming (Yaw: {imu_reader.get_yaw():+.1f}°)")
        else:
            print("  📡 IMU: Connected (waiting for data...)")
    else:
        print("  📡 IMU: Not available")
    
    while True:
        print_menu()
        
        if sys.platform == 'win32':
            print("  Enter command: ", end='', flush=True)
            key = msvcrt.getch()
            print(key.decode('utf-8', errors='ignore'))
            
            if key == b'1':
                move_relative_y(+100.0)
            elif key == b'2':
                move_relative_y(-100.0)
            elif key == b'3':
                move_relative_y(+500.0)
            elif key == b'4':
                print("  Enter distance (mm): ", end='', flush=True)
                distance_str = input()
                try:
                    distance = float(distance_str)
                    move_relative_y(distance)
                except ValueError:
                    print("  ❌ Invalid input!")
            elif key == b'5':
                run_test_sequence()
            elif key == b'6':
                test_backward_200()
            elif key == b'7':
                test_smooth_walk_600()
            elif key.lower() == b'm':
                start_idle_march()
            elif key.lower() == b's':
                stop_idle_march()
            elif key.lower() == b'c':
                # Toggle balance control
                global BALANCE_ENABLED, balance_controller
                BALANCE_ENABLED = not BALANCE_ENABLED
                
                if BALANCE_ENABLED:
                    # Initialize balance controller if not already done
                    if balance_controller is None:
                        balance_controller = BalanceController()
                    balance_controller.reset()
                    print(f"\n⚖️  Balance control ENABLED")
                    print(f"    Roll  PD: Kp={ROLL_K_P:.2f}, Kd={ROLL_K_D:.3f}")
                    print(f"    Pitch PD: Kp={PITCH_K_P:.2f}, Kd={PITCH_K_D:.3f}")
                    print(f"    Max offset: ±{MAX_HEIGHT_OFFSET:.0f} mm")
                    
                    if not (imu_reader and imu_reader.is_receiving_data()):
                        print("    ⚠️  WARNING: IMU not available - balance will not work!")
                else:
                    print(f"\n🛑 Balance control DISABLED")
            elif key.lower() == b'z':
                if imu_reader and imu_reader.is_connected():
                    print("\n🧭 Setting IMU zero reference...")
                    imu_reader.set_zero()
                    print("  ✅ IMU zero set")
                else:
                    print("\n⚠️  IMU not available")
            elif key.lower() == b'i':
                print("\n📡 IMU Status:")
                if IMU_ENABLED:
                    if imu_reader and imu_reader.is_connected():
                        orientation = imu_reader.get_orientation()
                        stats = imu_reader.get_stats()
                        print(f"    Connected: ✅")
                        print(f"    Port: {IMU_PORT}")
                        print(f"    Calibrated: {'✅' if orientation['calibrated'] else '⚠️'}")
                        print(f"    Yaw: {orientation['yaw']:+.2f}°")
                        print(f"    Pitch: {orientation['pitch']:+.2f}°")
                        print(f"    Roll: {orientation['roll']:+.2f}°")
                        print(f"    Packets received: {stats['packets']}")
                        print(f"    CRC errors: {stats['crc_errors']}")
                    else:
                        print(f"    Status: ❌ Not connected")
                        print(f"    Port: {IMU_PORT}")
                else:
                    print(f"    Status: ❌ Disabled (IMU_ENABLED=False)")
            elif key.lower() == b'h':
                if idle_marching:
                    stop_idle_march()
                else:
                    move_to_stand_position()
            elif key.lower() == b'p':
                print("\n📊 Current Status:")
                print(f"    Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")
                print(f"    Debug: {'ON' if DEBUG_GAIT else 'OFF'}")
                print(f"    Idle March: {'ACTIVE' if idle_marching else 'STOPPED'}")
                if idle_marching:
                    print(f"    March Step Indices: {march_step_indices}")
                print(f"    Balance Control: {'ENABLED' if BALANCE_ENABLED else 'DISABLED'}")
                if BALANCE_ENABLED and balance_controller:
                    print(f"    Balance Gains: Roll Kp={balance_controller.roll_Kp:.2f}, Pitch Kp={balance_controller.pitch_Kp:.2f}")
                print(f"    Registered legs: {list(tqc.leg_motors.keys())}")
                print(f"    Nav Planner: {nav_planner}")
                print(f"    Estimator: {state_estimator}")
            elif key.lower() == b'd':
                DEBUG_GAIT = not DEBUG_GAIT
                print(f"\n🔧 Debug mode: {'ON' if DEBUG_GAIT else 'OFF'}")
            elif key.lower() == b'l':
                ENABLE_LOGGING = not ENABLE_LOGGING
                print(f"\n📝 Logging: {'ON' if ENABLE_LOGGING else 'OFF'}")
                if ENABLE_LOGGING:
                    print(f"   Log path: {LOG_FILE_PATH}")
                    print(f"   Log rate: every {LOG_RATE} cycles")
            elif key.lower() == b'q':
                print("\n👋 Goodbye!")
                break
            else:
                print("  ❓ Unknown command")
        else:
            cmd = input("  Enter command: ").strip()
            if cmd == '1':
                move_relative_y(+100.0)
            elif cmd == '2':
                move_relative_y(-100.0)
            elif cmd == '3':
                move_relative_y(+500.0)
            elif cmd == '4':
                distance_str = input("  Enter distance (mm): ")
                try:
                    distance = float(distance_str)
                    move_relative_y(distance)
                except ValueError:
                    print("  ❌ Invalid input!")
            elif cmd == '5':
                run_test_sequence()
            elif cmd == '7':
                test_smooth_walk_600()
            elif cmd == '6':
                test_backward_200()
            elif cmd.lower() == 'm':
                start_idle_march()
            elif cmd.lower() == 's':
                stop_idle_march()
            elif cmd.lower() == 'z':
                if imu_reader and imu_reader.is_connected():
                    print("\n🧭 Setting IMU zero reference...")
                    imu_reader.set_zero()
                    print("  ✅ IMU zero set")
                else:
                    print("\n⚠️  IMU not available")
            elif cmd.lower() == 'i':
                print("\n📡 IMU Status:")
                if IMU_ENABLED:
                    if imu_reader and imu_reader.is_connected():
                        orientation = imu_reader.get_orientation()
                        stats = imu_reader.get_stats()
                        print(f"    Connected: ✅")
                        print(f"    Port: {IMU_PORT}")
                        print(f"    Calibrated: {'✅' if orientation['calibrated'] else '⚠️'}")
                        print(f"    Yaw: {orientation['yaw']:+.2f}°")
                        print(f"    Pitch: {orientation['pitch']:+.2f}°")
                        print(f"    Roll: {orientation['roll']:+.2f}°")
                        print(f"    Packets received: {stats['packets']}")
                        print(f"    CRC errors: {stats['crc_errors']}")
                    else:
                        print(f"    Status: ❌ Not connected")
                        print(f"    Port: {IMU_PORT}")
                else:
                    print(f"    Status: ❌ Disabled (IMU_ENABLED=False)")
            elif cmd.lower() == 'h':
                if idle_marching:
                    stop_idle_march()
                else:
                    move_to_stand_position()
            elif cmd.lower() == 'p':
                print("\n📊 Current Status:")
                print(f"    Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")
                print(f"    Debug: {'ON' if DEBUG_GAIT else 'OFF'}")
                print(f"    Idle March: {'ACTIVE' if idle_marching else 'STOPPED'}")
                if idle_marching:
                    print(f"    March Step Indices: {march_step_indices}")
                print(f"    Registered legs: {list(tqc.leg_motors.keys())}")
                print(f"    Nav Planner: {nav_planner}")
                print(f"    Estimator: {state_estimator}")
            elif cmd.lower() == 'd':
                DEBUG_GAIT = not DEBUG_GAIT
                print(f"\n🔧 Debug mode: {'ON' if DEBUG_GAIT else 'OFF'}")
            elif cmd.lower() == 'l':
                ENABLE_LOGGING = not ENABLE_LOGGING
                print(f"\n📝 Logging: {'ON' if ENABLE_LOGGING else 'OFF'}")
                if ENABLE_LOGGING:
                    print(f"   Log path: {LOG_FILE_PATH}")
                    print(f"   Log rate: every {LOG_RATE} cycles")
            elif cmd.lower() == 'q':
                print("\n👋 Goodbye!")
                break
            else:
                print("  ❓ Unknown command")

# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """Main entry point."""
    global SIMULATION_MODE
    
    print("="*70)
    print("  BLEGS Relative Position Control - Y-Axis Movement")
    print("="*70)
    print("  Version: 1.0")
    print("  Based on: Hierarchical Control Architecture")
    print("  Author: M-TRCH")
    print("  Date: February 5, 2026")
    print("="*70)
    print(f"  Navigation Parameters:")
    print(f"    v_max: {NAV_V_MAX} mm/s")
    print(f"    K_p: {NAV_K_P}")
    print(f"    Tolerance: {NAV_TOLERANCE} mm")
    print(f"  Gait Parameters:")
    print(f"    Update rate: {UPDATE_RATE} Hz")
    print(f"    Cycle time: {GAIT_CYCLE_TIME:.3f} s")
    print(f"    Step length: {GAIT_STEP_FORWARD} mm")
    print(f"    Lift height: {GAIT_LIFT_HEIGHT} mm")
    print("="*70)
    
    # --- Step 1: IMU Connection (before motors) ---
    global imu_reader, yaw_controller
    if IMU_ENABLED:
        print(f"\n📡 Connecting to IMU on {IMU_PORT}...")
        imu_reader = create_imu_reader(IMU_PORT, auto_connect=True)
        if imu_reader and imu_reader.is_connected():
            print("  ✅ IMU port opened successfully")
            
            # Wait for first data packet (up to 2 seconds)
            print("  ⏳ Waiting for IMU data...")
            data_received = False
            for i in range(20):
                if imu_reader.is_receiving_data():
                    print("  ✅ IMU data streaming")
                    data_received = True
                    break
                time.sleep(0.1)
            
            if not data_received:
                print("  ⚠️  No data received from IMU - check wiring and firmware")
                print("  ⚠️  Continuing without yaw control")
                imu_reader.disconnect()
                imu_reader = None
            else:
                # Wait for calibration
                print("  ⏳ Waiting for IMU calibration...")
                for i in range(30):
                    if imu_reader.is_calibrated():
                        print(f"  ✅ IMU calibrated")
                        break
                    time.sleep(0.1)
                else:
                    print("  ⚠️  IMU not fully calibrated (continuing anyway)")
                
                # Set current orientation as zero reference
                imu_reader.set_zero()
                time.sleep(0.2)
                
                # Initialize yaw controller
                yaw_controller = YawController(
                    K_p=YAW_K_P,
                    K_d=YAW_K_D,
                    max_correction=YAW_MAX_CORRECTION
                )
                print(f"  🧭 Yaw controller initialized (K_p={YAW_K_P}, K_d={YAW_K_D})")
        else:
            print("  ❌ Failed to open IMU port - check COM port and permissions")
            print("  ⚠️  Continuing without yaw control")
            imu_reader = None
    else:
        print("\n📡 IMU disabled (IMU_ENABLED=False)")
    
    # --- Step 2: Motor Discovery ---
    print("\n🔍 Discovering motors...")
    discovered_motors = discover_motors()
    
    if not discovered_motors:
        print("\n❌ No motors found!")
        print("  Please check connections and try again.")
        print("\n⚠️  Running in SIMULATION MODE (no motors)")
        SIMULATION_MODE = True
    
    # --- Step 3: Register Motors ---
    if discovered_motors:
        all_registered = register_leg_motors(discovered_motors)
        
        if not all_registered:
            print("\n⚠️  Not all motors registered!")
            response = input("  Continue anyway? (y/n): ")
            if response.lower() != 'y':
                return
        
        # Verify registration
        print(f"\n📋 Registered legs: {list(tqc.leg_motors.keys())}")
        print(f"   Total motors: {len(tqc.motor_registry)}")
    
    # --- Step 4: Start Motors ---
    if discovered_motors and tqc.leg_motors:
        print("\n🚀 Starting motors...")
        start_all_motors()
        
        # Switch to fast timeout
        for motor in tqc.motor_registry.values():
            motor.set_timeout(FAST_TIMEOUT)
        
        # Move to home position
        home_angles = {}
        for leg_id in tqc.leg_motors:
            P_A, P_B = get_motor_positions(leg_id)
            angles = calculate_ik_no_ef(
                np.array([DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT]),
                P_A, P_B
            )
            home_angles[leg_id] = np.rad2deg(angles)
        
        print(f"\n🏠 Moving to home position...")
        print(f"   Home angles: {home_angles}")
        smooth_move_to_home_position(tqc.leg_motors, home_angles, duration_s=3.0)
    else:
        SIMULATION_MODE = True
        print("\n⚠️  No motors available - running in SIMULATION MODE")
    
    print(f"\n🎮 Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")
    
    # --- Step 5: Run Interactive Mode ---
    try:
        interactive_mode()
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user")
    finally:
        # Cleanup
        global idle_marching
        if idle_marching:
            print("\n🛑 Stopping idle march...")
            idle_marching = False
            time.sleep(0.5)
        
        # Move motors to init position (-90 deg) before disconnecting
        if tqc.motor_registry:
            print("\n🏠 Moving motors to init position (-90°)...")
            init_angle = MOTOR_INIT_ANGLE
            
            for motor_id, motor in tqc.motor_registry.items():
                try:
                    motor.set_position_direct(init_angle)
                    print(f"  ✓ Motor {motor_id}: {init_angle}°")
                except Exception as e:
                    print(f"  ⚠️  Motor {motor_id} failed: {e}")
            
            # Wait for motors to reach position
            time.sleep(1.0)
            print("  ✅ Motors at init position")
        
        print("\n🔌 Disconnecting motors...")
        for motor in tqc.motor_registry.values():
            motor.disconnect()
        print("  ✅ Done")
    
    # Cleanup IMU if connected
    if imu_reader and imu_reader.is_connected():
        print("\n📡 Disconnecting IMU...")
        imu_reader.disconnect()
        print("  ✅ Done")


if __name__ == "__main__":
    main()
