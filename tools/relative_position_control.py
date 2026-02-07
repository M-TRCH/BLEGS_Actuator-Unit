"""
Relative Position Control for Quadruped Robot - Y-axis Movement
Author: M-TRCH
Date: February 5, 2026

This script implements relative position control for Y-axis movement (forward/backward)
based on the hierarchical control architecture defined in ROBOT_ARCHITECTURE.tex.

Architecture:
┌─────────────────────────────────────────────────────────────────┐
│                    HIGH-LEVEL LAYER (10-50 Hz)                  │
│               SimpleNavigationPlanner (move_relative)            │
├─────────────────────────────────────────────────────────────────┤
│                     MID-LEVEL LAYER (50 Hz)                     │
│         Gait Generator + Inverse Kinematics (5-Bar IK)          │
├─────────────────────────────────────────────────────────────────┤
│                     LOW-LEVEL LAYER (5 kHz)                     │
│              Motor Control (FOC/SVPWM) - Firmware               │
├─────────────────────────────────────────────────────────────────┤
│                       FEEDBACK LOOP                              │
│              TimeBasedEstimator (Dead Reckoning)                │
└─────────────────────────────────────────────────────────────────┘

Usage:
    python relative_position_control.py
    
    Commands:
        move_relative(+500)  - Move forward 500mm
        move_relative(-300)  - Move backward 300mm
"""

import numpy as np
import time
import threading
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import navigation components
from navigation.simple_planner import SimpleNavigationPlanner
from navigation.time_estimator import TimeBasedEstimator

# Import the module to access global variables by reference
import test_quadruped_control as tqc

# Import constants and functions (these don't change)
from test_quadruped_control import (
    # Protocol constants
    BAUD_RATE, FAST_TIMEOUT,
    ControlMode, CONTROL_MODE,
    
    # Robot configuration
    BODY_LENGTH, BODY_WIDTH, MOTOR_SPACING,
    L_AC, L_BD, L_CE, L_DE, GEAR_RATIO,
    EXPECTED_MOTOR_IDS, DEFAULT_STANCE_HEIGHT, DEFAULT_STANCE_OFFSET_X,
    GAIT_LIFT_HEIGHT, GAIT_STEP_FORWARD,
    
    # Control parameters
    UPDATE_RATE, TRAJECTORY_STEPS, SMOOTH_TROT_STANCE_RATIO,
    
    # Classes and functions
    BinaryMotorController,
    discover_motors, register_leg_motors, start_all_motors,
    get_motor_positions, calculate_ik_no_ef,
    generate_elliptical_trajectory, get_gait_phase_offset,
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

# Navigation parameters (tunable)
NAV_V_MAX = 50.0           # Maximum velocity (mm/s) - Start slow for safety
NAV_K_P = 1.0              # Proportional gain
NAV_TOLERANCE = 10.0       # Position tolerance (mm)
NAV_TIMEOUT = 60.0         # Maximum time for movement (seconds)

# Gait to velocity mapping
GAIT_CYCLE_TIME = TRAJECTORY_STEPS / UPDATE_RATE  # seconds per gait cycle

# Simulation mode (auto-detected if no motors found)
SIMULATION_MODE = False
DEBUG_GAIT = False  # Set to True to see gait debug output

# ============================================================================
# GLOBAL STATE
# ============================================================================

# Control components
nav_planner = None
state_estimator = None

# Control state
control_running = False
control_paused = True

# Trajectory caches (pre-computed for each step length)
trajectory_cache = {}

# ============================================================================
# GAIT VELOCITY MAPPING
# ============================================================================

def update_gait_from_velocity(v_body_y: float) -> tuple:
    """
    Convert body velocity to gait parameters.
    
    This function maps the velocity command from the navigation planner
    to the gait generator parameters (step length and direction).
    
    Args:
        v_body_y: Body frame Y velocity (mm/s)
                  Positive = forward, Negative = backward
    
    Returns:
        tuple: (step_length, reverse)
            step_length: Step length in mm
            reverse: True if walking backward
    """
    # Calculate step length from velocity
    # step_length ∝ v_body * gait_cycle_time
    step_length = abs(v_body_y) * GAIT_CYCLE_TIME
    
    # Limit step length to maximum
    step_length = min(step_length, GAIT_STEP_FORWARD)
    
    # Determine direction
    reverse = (v_body_y < 0)
    
    return step_length, reverse


def get_trajectory_for_velocity(v_body_y: float, leg_id: str) -> list:
    """
    Get or generate trajectory for given velocity.
    
    Args:
        v_body_y: Body frame Y velocity (mm/s)
        leg_id: Leg identifier ('FR', 'FL', 'RR', 'RL')
    
    Returns:
        List of (x, y) positions for the trajectory
    """
    step_length, reverse = update_gait_from_velocity(v_body_y)
    
    # Determine if this leg should mirror X
    mirror_x = (leg_id in ['FR', 'RR'])
    
    # Generate trajectory
    trajectory = generate_elliptical_trajectory(
        num_steps=TRAJECTORY_STEPS,
        lift_height=GAIT_LIFT_HEIGHT,
        step_forward=step_length,
        mirror_x=mirror_x,
        stance_ratio=SMOOTH_TROT_STANCE_RATIO,
        home_x=DEFAULT_STANCE_OFFSET_X,
        home_y=DEFAULT_STANCE_HEIGHT,
        reverse=reverse
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


def update_leg_position(leg_id: str, foot_position: tuple) -> bool:
    """
    Update leg to target foot position using IK.
    
    Args:
        leg_id: Leg identifier
        foot_position: Target (x, y) position in leg frame
    
    Returns:
        True if successful
    """
    x, y = foot_position
    P_A, P_B = get_motor_positions(leg_id)
    
    # Calculate IK
    angles = calculate_ik_no_ef(np.array([x, y]), P_A, P_B)
    
    if np.isnan(angles).any():
        print(f"  ⚠️  IK failed for {leg_id} at ({x:.1f}, {y:.1f})")
        return False
    
    theta_A, theta_B = angles
    
    # Update leg state
    with viz_lock:
        tqc.leg_states[leg_id]['target_angles'] = [theta_A, theta_B]
        tqc.leg_states[leg_id]['target_pos'] = [x, y]
    
    # Send to motors
    return send_leg_angles(leg_id, theta_A, theta_B)


def update_all_legs_gait(trajectories: dict, step_indices: dict) -> bool:
    """
    Update all legs based on their trajectories and current step indices.
    
    Args:
        trajectories: Dict of {leg_id: trajectory_list}
        step_indices: Dict of {leg_id: current_step_index}
    
    Returns:
        True if all successful
    """
    all_success = True
    
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        if leg_id in trajectories and leg_id in step_indices:
            traj = trajectories[leg_id]
            idx = step_indices[leg_id] % len(traj)
            foot_pos = traj[idx]
            
            success = update_leg_position(leg_id, foot_pos)
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
# MAIN CONTROL FUNCTION
# ============================================================================

def move_relative_y(target_distance_mm: float, timeout_s: float = NAV_TIMEOUT) -> bool:
    """
    Move relative distance on Y-axis (forward/backward).
    
    This is the main high-level control function that implements
    the hierarchical control architecture.
    
    Args:
        target_distance_mm: Distance to move (mm)
                           Positive = forward
                           Negative = backward
        timeout_s: Maximum time for movement (seconds)
    
    Returns:
        True if target reached, False if timeout or error
    """
    global nav_planner, state_estimator
    
    print("\n" + "="*70)
    print("  📍 RELATIVE POSITION CONTROL - Y-AXIS")
    print("="*70)
    print(f"  Target: {target_distance_mm:+.1f} mm")
    print(f"  Direction: {'Forward' if target_distance_mm > 0 else 'Backward'}")
    print(f"  v_max: {NAV_V_MAX} mm/s")
    print(f"  Timeout: {timeout_s:.1f} s")
    print(f"  Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")
    print(f"  Motors available: {len(tqc.leg_motors)} legs ({list(tqc.leg_motors.keys())})")
    print("="*70)
    
    # Initialize components if needed
    if nav_planner is None:
        nav_planner = SimpleNavigationPlanner(
            v_max=NAV_V_MAX,
            K_p=NAV_K_P,
            tolerance=NAV_TOLERANCE
        )
    
    if state_estimator is None:
        state_estimator = TimeBasedEstimator()
    
    # 1. Set target
    nav_planner.set_relative_target(target_distance_mm)
    state_estimator.start()
    
    start_time = time.time()
    last_status_time = start_time
    
    # Initialize step indices for each leg
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
            
            # 2.2 Generate trajectories for current velocity
            trajectories = {}
            for leg_id in ['FR', 'FL', 'RR', 'RL']:
                trajectories[leg_id] = get_trajectory_for_velocity(v_body_y, leg_id)
            
            # 2.3 Update all legs
            update_all_legs_gait(trajectories, step_indices)
            
            # 2.4 Advance step indices
            for leg_id in step_indices:
                step_indices[leg_id] = (step_indices[leg_id] + 1) % TRAJECTORY_STEPS
            
            # 2.5 Update state estimator
            state_estimator.update(v_body_y, current_time)
            nav_planner.update_position(state_estimator.get_position())
            
            # 2.6 Print status periodically
            if current_time - last_status_time >= 0.5:
                status = nav_planner.get_status()
                est_status = state_estimator.get_status()
                print(f"  📊 Position: {status['current_y']:+.1f}/{status['target_y']:+.1f} mm "
                      f"| Progress: {status['progress']:.0f}% "
                      f"| v_y: {v_body_y:+.1f} mm/s "
                      f"| Time: {elapsed:.1f}s")
                last_status_time = current_time
            
            # 2.7 Wait for next cycle
            loop_duration = time.time() - loop_start
            sleep_time = (1.0 / UPDATE_RATE) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # 3. Target reached - stop and report
        stop_all_legs()
        
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
        print("="*70)
        
        return True
        
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
        return False
    
    finally:
        # Ensure motors stop
        stop_all_legs()

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


def test_long_forward():
    """Test: Move forward 500mm"""
    print("\n" + "="*70)
    print("  TEST 3: Long Forward Movement (+500mm)")
    print("="*70)
    return move_relative_y(+500.0, timeout_s=60.0)


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
    
    print("\n" + "="*70)
    print(f"  RELATIVE POSITION CONTROL - [{mode_str}] - Debug: {debug_str}")
    print("="*70)
    print("  Movement Commands:")
    print("    [1] Move forward +100mm")
    print("    [2] Move backward -100mm")
    print("    [3] Move forward +500mm")
    print("    [4] Custom distance (enter value)")
    print("    [5] Run test sequence")
    print("  Other Commands:")
    print("    [H] Move to home/stand position")
    print("    [P] Print current status")
    print("    [D] Toggle debug output")
    print("    [Q] Quit")
    print("="*70)


def interactive_mode():
    """Run interactive control mode."""
    global nav_planner, state_estimator, DEBUG_GAIT
    
    # Initialize components
    nav_planner = SimpleNavigationPlanner(
        v_max=NAV_V_MAX,
        K_p=NAV_K_P,
        tolerance=NAV_TOLERANCE
    )
    state_estimator = TimeBasedEstimator()
    
    print("\n🎮 Interactive mode started")
    
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
            elif key.lower() == b'h':
                move_to_stand_position()
            elif key.lower() == b'p':
                print("\n📊 Current Status:")
                print(f"    Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")
                print(f"    Debug: {'ON' if DEBUG_GAIT else 'OFF'}")
                print(f"    Registered legs: {list(tqc.leg_motors.keys())}")
                print(f"    Nav Planner: {nav_planner}")
                print(f"    Estimator: {state_estimator}")
            elif key.lower() == b'd':
                DEBUG_GAIT = not DEBUG_GAIT
                print(f"\n🔧 Debug mode: {'ON' if DEBUG_GAIT else 'OFF'}")
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
            elif cmd.lower() == 'h':
                move_to_stand_position()
            elif cmd.lower() == 'p':
                print("\n📊 Current Status:")
                print(f"    Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")
                print(f"    Debug: {'ON' if DEBUG_GAIT else 'OFF'}")
                print(f"    Registered legs: {list(tqc.leg_motors.keys())}")
                print(f"    Nav Planner: {nav_planner}")
                print(f"    Estimator: {state_estimator}")
            elif cmd.lower() == 'd':
                DEBUG_GAIT = not DEBUG_GAIT
                print(f"\n🔧 Debug mode: {'ON' if DEBUG_GAIT else 'OFF'}")
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
    
    # --- Step 1: Motor Discovery ---
    print("\n🔍 Discovering motors...")
    discovered_motors = discover_motors()
    
    if not discovered_motors:
        print("\n❌ No motors found!")
        print("  Please check connections and try again.")
        print("\n⚠️  Running in SIMULATION MODE (no motors)")
        SIMULATION_MODE = True
    
    # --- Step 2: Register Motors ---
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
    
    # --- Step 3: Start Motors ---
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
    
    # --- Step 4: Run Interactive Mode ---
    try:
        interactive_mode()
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user")
    finally:
        # Cleanup
        print("\n🔌 Disconnecting motors...")
        for motor in tqc.motor_registry.values():
            motor.disconnect()
        print("  ✅ Done")


if __name__ == "__main__":
    main()
