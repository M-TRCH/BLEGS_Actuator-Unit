#!/usr/bin/env python3
"""
Gait Control Test Script - Standalone Version
Generates elliptical trajectory internally (no CSV required)
Uses correct Binary Protocol v1.2 with proper feedback parsing

Author: M-TRCH
Date: 2026-01-20
Version: 2.1

Based on Quadruped_Gait_Control_No_EF.py protocol implementation
"""

# ============================================================================
# ▼▼▼ RUN CONFIGURATION - EDIT THESE VALUES ▼▼▼
# ============================================================================

# Serial Port Configuration
CONFIG_PORT = "COM3"              # Serial port (e.g., "COM3", "/dev/ttyUSB0")
CONFIG_BAUDRATE = 921600          # Baudrate (default: 921600)

# Motor Selection
CONFIG_MOTOR = "A"                # Motor to control: "A" or "B"

# Playback Configuration
CONFIG_RATE_HZ = 50.0             # Playback rate in Hz (matches Quadruped: UPDATE_RATE=50)
CONFIG_MODE = "direct"            # Control mode: "direct" or "scurve"
CONFIG_LOOP = True                # True = loop continuously, False = run once
CONFIG_VERBOSE = False            # True = detailed output, False = progress bar

# Gait Parameters (matches Quadruped_Gait_Control_No_EF.py)
CONFIG_STEP_LENGTH = 50.0         # Step length in mm (GAIT_STEP_FORWARD)
CONFIG_LIFT_HEIGHT = 15.0         # Foot lift height in mm (GAIT_LIFT_HEIGHT)
CONFIG_STANCE_HEIGHT = -200.0     # Stance height in mm (DEFAULT_STANCE_HEIGHT)
CONFIG_NUM_POINTS = 20            # Number of trajectory points (TRAJECTORY_STEPS)
CONFIG_REVERSE = False            # True = backward gait, False = forward gait
CONFIG_STANCE_RATIO = 0.65        # Stance phase ratio (SMOOTH_TROT_STANCE_RATIO)

# Initialization
CONFIG_INIT_WAIT = 3.0            # Wait time for motor to reach start (seconds)
CONFIG_SKIP_START = False         # True = skip PING (motor already running)

# ============================================================================
# ▲▲▲ END OF CONFIGURATION ▲▲▲
# ============================================================================

import struct
import serial
import serial.tools.list_ports
import time
import math
import argparse
from typing import List, Tuple, Optional
from enum import IntEnum
from dataclasses import dataclass

# ============================================================================
# PROTOCOL CONSTANTS (v1.2) - Matches protocol.h
# ============================================================================

HEADER_1 = 0xFE
HEADER_2 = 0xEE

class PacketType(IntEnum):
    """Packet type enumeration - matches protocol.h"""
    PKT_CMD_SET_GOAL = 0x01
    PKT_CMD_SET_CONFIG = 0x02
    PKT_CMD_PING = 0x03
    PKT_CMD_EMERGENCY_STOP = 0x04
    PKT_FB_STATUS = 0x81
    PKT_FB_CONFIG = 0x82
    PKT_FB_ERROR = 0x83
    PKT_FB_PONG = 0x84

class ControlMode(IntEnum):
    """Control mode enumeration"""
    MODE_DIRECT_POSITION = 0x00
    MODE_SCURVE_PROFILE = 0x01

class StatusFlags(IntEnum):
    """Status flags bitmask"""
    STATUS_MOVING = (1 << 0)
    STATUS_ERROR = (1 << 1)
    STATUS_AT_GOAL = (1 << 2)
    STATUS_OVERHEAT = (1 << 3)
    STATUS_OVERCURRENT = (1 << 4)
    STATUS_ENCODER_ERROR = (1 << 5)
    STATUS_EMERGENCY_STOPPED = (1 << 6)

class ErrorCode(IntEnum):
    """Error code enumeration"""
    ERR_CRC_FAILED = 0x01
    ERR_INVALID_PACKET = 0x02
    ERR_TIMEOUT = 0x03
    ERR_UNKNOWN_COMMAND = 0x04
    ERR_INVALID_PAYLOAD = 0x05

# ============================================================================
# ROBOT CONFIGURATION (No EF Link)
# ============================================================================

# --- Five-Bar Linkage Parameters (mm) ---
MOTOR_SPACING = 85.0  # Distance between Motor A and Motor B (horizontal)

# Link lengths
L_AC = 105.0  # Link 1 length (Motor A to joint C)
L_BD = 105.0  # Link 2 length (Motor B to joint D)
L_CE = 145.0  # Link 3 length (joint C to joint E)
L_DE = 145.0  # Link 4 length (joint D to joint E)

# Motor Configuration
GEAR_RATIO = 8.0  # Motor shaft to output shaft gear ratio
MOTOR_INIT_ANGLE = -90.0  # Initial motor angle (degrees) in robot frame

# Default Standing Pose
DEFAULT_STANCE_HEIGHT = -200.0  # mm (negative = down)
DEFAULT_STANCE_OFFSET_X = 0.0   # mm

# Gait Parameters
GAIT_LIFT_HEIGHT = 15.0    # mm
GAIT_STEP_FORWARD = 50.0   # mm

# Motor positions (Left leg configuration)
P_A = [-MOTOR_SPACING/2, 0.0]
P_B = [MOTOR_SPACING/2, 0.0]

# ============================================================================
# CRC-16-IBM CALCULATION
# ============================================================================

def calculate_crc16(data: bytes) -> int:
    """
    Calculate CRC-16-IBM for data buffer
    Polynomial: 0xA001 (reversed 0x8005)
    Initial value: 0xFFFF
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

# ============================================================================
# PACKET BUILDING FUNCTIONS
# ============================================================================

def build_packet(pkt_type: int, payload: bytes = b'') -> bytes:
    """Build a complete binary packet with CRC"""
    header = bytes([HEADER_1, HEADER_2])
    type_byte = bytes([pkt_type])
    payload_len = bytes([len(payload)])
    
    # CRC is calculated over: packet_type + payload_len + payload
    crc_data = type_byte + payload_len + payload
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    
    return header + type_byte + payload_len + payload + crc_bytes


def build_ping_packet() -> bytes:
    """Build PING packet"""
    return build_packet(PacketType.PKT_CMD_PING)


def build_direct_position_packet(target_degrees: float) -> bytes:
    """
    Build direct position command packet
    
    Payload format:
    - control_mode: 1 byte (0x00 = direct)
    - target_pos: 4 bytes (int32, degrees * 100)
    """
    mode = bytes([ControlMode.MODE_DIRECT_POSITION])
    target_pos = struct.pack('<i', int(target_degrees * 100))
    payload = mode + target_pos
    return build_packet(PacketType.PKT_CMD_SET_GOAL, payload)


def build_scurve_position_packet(target_degrees: float, duration_ms: int) -> bytes:
    """
    Build S-Curve position command packet
    
    Payload format:
    - control_mode: 1 byte (0x01 = scurve)
    - target_pos: 4 bytes (int32, degrees * 100)
    - duration_ms: 2 bytes (uint16)
    """
    mode = bytes([ControlMode.MODE_SCURVE_PROFILE])
    target_pos = struct.pack('<i', int(target_degrees * 100))
    duration = struct.pack('<H', duration_ms)
    payload = mode + target_pos + duration
    return build_packet(PacketType.PKT_CMD_SET_GOAL, payload)


def build_emergency_stop_packet() -> bytes:
    """Build emergency stop packet"""
    return build_packet(PacketType.PKT_CMD_EMERGENCY_STOP)

# ============================================================================
# FEEDBACK PARSING
# ============================================================================

@dataclass
class MotorFeedback:
    """Motor feedback data structure"""
    motor_id: int
    position_deg: float
    current_ma: int
    flags: int
    is_moving: bool
    at_goal: bool
    has_error: bool
    emergency_stopped: bool


def parse_status_feedback(payload: bytes) -> Optional[MotorFeedback]:
    """
    Parse FB_STATUS payload
    
    PayloadStatus structure (from protocol.h):
    - motor_id: 1 byte (uint8)
    - actual_pos: 4 bytes (int32, degrees * 100)
    - actual_current: 2 bytes (int16, mA)
    - status_flags: 1 byte (uint8)
    
    Total: 8 bytes
    """
    if len(payload) < 8:
        return None
    
    motor_id = payload[0]
    actual_pos_raw = struct.unpack('<i', payload[1:5])[0]
    actual_current = struct.unpack('<h', payload[5:7])[0]
    status_flags = payload[7]
    
    return MotorFeedback(
        motor_id=motor_id,
        position_deg=actual_pos_raw / 100.0,
        current_ma=actual_current,
        flags=status_flags,
        is_moving=bool(status_flags & StatusFlags.STATUS_MOVING),
        at_goal=bool(status_flags & StatusFlags.STATUS_AT_GOAL),
        has_error=bool(status_flags & StatusFlags.STATUS_ERROR),
        emergency_stopped=bool(status_flags & StatusFlags.STATUS_EMERGENCY_STOPPED)
    )


def receive_feedback(port: serial.Serial, timeout: float = 0.008) -> Optional[MotorFeedback]:
    """
    Receive and parse feedback packet from motor (non-blocking)
    
    Returns MotorFeedback object or None if no valid packet received
    """
    # Check if there's any data available first
    if port.in_waiting < 12:  # Minimum packet size: 2 header + 2 meta + 8 payload
        return None
    
    original_timeout = port.timeout
    port.timeout = timeout
    
    try:
        # Read header
        header = port.read(2)
        if len(header) != 2:
            return None
        
        if header[0] != HEADER_1 or header[1] != HEADER_2:
            # Clear buffer if invalid header
            port.reset_input_buffer()
            return None
        
        # Read packet type and length
        meta = port.read(2)
        if len(meta) != 2:
            return None
        
        pkt_type = meta[0]
        payload_len = meta[1]
        
        # Validate payload length
        if payload_len > 32:
            port.reset_input_buffer()
            return None
        
        # Read payload
        payload = port.read(payload_len)
        if len(payload) != payload_len:
            return None
        
        # Read CRC
        crc_bytes = port.read(2)
        if len(crc_bytes) != 2:
            return None
        
        # Verify CRC
        received_crc = struct.unpack('<H', crc_bytes)[0]
        calculated_crc = calculate_crc16(bytes([pkt_type, payload_len]) + payload)
        
        if received_crc != calculated_crc:
            return None
        
        # Parse based on packet type
        if pkt_type == PacketType.PKT_FB_STATUS:
            return parse_status_feedback(payload)
        
        return None
    finally:
        port.timeout = original_timeout

# ============================================================================
# KINEMATICS FUNCTIONS (No EF Link)
# ============================================================================

def solve_circle_intersection(center1, radius1, center2, radius2, choose_lower=True):
    """Find intersection of two circles, return lower point by default"""
    import numpy as np
    
    c1 = np.array(center1)
    c2 = np.array(center2)
    
    V_12 = c2 - c1
    d = np.linalg.norm(V_12)
    
    # Check if circles intersect
    if d > (radius1 + radius2) or d < abs(radius1 - radius2) or d == 0:
        return None
    
    a = (radius1**2 - radius2**2 + d**2) / (2 * d)
    h_squared = radius1**2 - a**2
    
    if h_squared < 0:
        return None
    
    h = math.sqrt(h_squared)
    v_d = V_12 / d
    v_perp = np.array([-v_d[1], v_d[0]])
    
    P_intersection_1 = c1 + a * v_d + h * v_perp
    P_intersection_2 = c1 + a * v_d - h * v_perp
    
    if choose_lower:
        return P_intersection_2 if P_intersection_2[1] < P_intersection_1[1] else P_intersection_1
    else:
        return P_intersection_1 if P_intersection_1[1] > P_intersection_2[1] else P_intersection_2


def calculate_ik(target_x: float, target_y: float, prev_theta_A: float = None, prev_theta_B: float = None) -> Tuple[float, float]:
    """
    Calculate Inverse Kinematics for five-bar linkage (No EF link)
    With continuity check to avoid solution flipping
    
    Args:
        target_x: Target foot X position in mm (leg frame)
        target_y: Target foot Y position in mm (leg frame, negative = down)
        prev_theta_A: Previous theta_A for continuity check
        prev_theta_B: Previous theta_B for continuity check
    
    Returns:
        (theta_A, theta_B) in degrees, or (nan, nan) if no solution
    """
    import numpy as np
    
    P_E_target = np.array([target_x, target_y])
    P_A_pos = np.array(P_A)
    P_B_pos = np.array(P_B)
    
    # Try both configurations for joint C (lower and upper)
    P_C_lower = solve_circle_intersection(P_A_pos, L_AC, P_E_target, L_CE, choose_lower=True)
    P_C_upper = solve_circle_intersection(P_A_pos, L_AC, P_E_target, L_CE, choose_lower=False)
    
    # Try both configurations for joint D (lower and upper)
    P_D_lower = solve_circle_intersection(P_B_pos, L_BD, P_E_target, L_DE, choose_lower=True)
    P_D_upper = solve_circle_intersection(P_B_pos, L_BD, P_E_target, L_DE, choose_lower=False)
    
    # Calculate all possible theta_A values
    theta_A_options = []
    if P_C_lower is not None:
        V_AC = P_C_lower - P_A_pos
        theta_A_options.append(math.degrees(math.atan2(V_AC[1], V_AC[0])))
    if P_C_upper is not None:
        V_AC = P_C_upper - P_A_pos
        theta_A_options.append(math.degrees(math.atan2(V_AC[1], V_AC[0])))
    
    # Calculate all possible theta_B values
    theta_B_options = []
    if P_D_lower is not None:
        V_BD = P_D_lower - P_B_pos
        theta_B_options.append(math.degrees(math.atan2(V_BD[1], V_BD[0])))
    if P_D_upper is not None:
        V_BD = P_D_upper - P_B_pos
        theta_B_options.append(math.degrees(math.atan2(V_BD[1], V_BD[0])))
    
    if not theta_A_options or not theta_B_options:
        return (float('nan'), float('nan'))
    
    # Choose the solution closest to previous angles (for continuity)
    if prev_theta_A is not None:
        # Find theta_A closest to previous
        theta_A = min(theta_A_options, key=lambda x: abs(x - prev_theta_A))
    else:
        # Default: choose the more negative angle (typical robot configuration)
        theta_A = min(theta_A_options)
    
    if prev_theta_B is not None:
        # Find theta_B closest to previous
        theta_B = min(theta_B_options, key=lambda x: abs(x - prev_theta_B))
    else:
        # Default: choose the more negative angle
        theta_B = max(theta_B_options)  # For motor B, choose less negative (closer to -90)
    
    return (theta_A, theta_B)

# ============================================================================
# TRAJECTORY GENERATION
# ============================================================================

def generate_elliptical_trajectory(
    step_forward: float = GAIT_STEP_FORWARD,
    lift_height: float = GAIT_LIFT_HEIGHT,
    num_steps: int = 30,
    stance_ratio: float = 0.65,
    home_x: float = DEFAULT_STANCE_OFFSET_X,
    home_y: float = DEFAULT_STANCE_HEIGHT,
    reverse: bool = False
) -> List[Tuple[float, float]]:
    """
    Generate elliptical foot trajectory for trot gait
    
    The trajectory is designed to loop seamlessly - the last point transitions
    smoothly back to the first point.
    
    Args:
        step_forward: Forward step length in mm
        lift_height: Foot lift height in mm
        num_steps: Number of trajectory points
        stance_ratio: Ratio of stance phase (0.0-1.0)
        home_x: Home position X offset in mm
        home_y: Home position Y (height) in mm
        reverse: True for backward motion
    
    Returns:
        List of (x, y) positions in mm
    """
    trajectory = []
    
    swing_steps = int(num_steps * (1.0 - stance_ratio))
    stance_steps = num_steps - swing_steps
    
    direction = -1 if reverse else 1
    
    for i in range(num_steps):
        if i < swing_steps:
            # Swing phase - elliptical arc (0 to π)
            # Use (i / swing_steps) to go from 0 to just before π
            # The swing ends at the point before stance begins
            t = math.pi * i / swing_steps
        else:
            # Stance phase - linear on ground (π to 2π)
            # Map stance_steps points from π to 2π (exclusive of 2π for seamless loop)
            # stance_progress goes from 0/stance_steps to (stance_steps-1)/stance_steps
            stance_idx = i - swing_steps
            t = math.pi + math.pi * (stance_idx + 1) / stance_steps
        
        # X position (forward/backward)
        px = direction * (-step_forward * math.cos(t))
        
        # Y position (up/down)
        if i < swing_steps:
            # Swing - elliptical lift
            py = home_y + lift_height * math.sin(t)
        else:
            # Stance - on ground
            py = home_y
        
        trajectory.append((px + home_x, py))
    
    return trajectory


def trajectory_to_motor_angles(trajectory: List[Tuple[float, float]], motor_select: str = 'A') -> List[float]:
    """
    Convert foot trajectory to motor shaft angles with continuity check
    
    Args:
        trajectory: List of (x, y) foot positions
        motor_select: 'A' or 'B' to select which motor angle
    
    Returns:
        List of motor shaft angles in degrees
    """
    motor_angles = []
    prev_theta_A = None
    prev_theta_B = None
    
    for (x, y) in trajectory:
        theta_A, theta_B = calculate_ik(x, y, prev_theta_A, prev_theta_B)
        
        if math.isnan(theta_A) or math.isnan(theta_B):
            print(f"Warning: No IK solution for ({x:.1f}, {y:.1f})")
            # Use previous angle or init angle
            if motor_angles:
                motor_angles.append(motor_angles[-1])
            else:
                motor_angles.append(MOTOR_INIT_ANGLE * GEAR_RATIO)
            continue
        
        # Update previous angles for continuity tracking
        prev_theta_A = theta_A
        prev_theta_B = theta_B
        
        # Select motor angle and convert to shaft angle (multiply by gear ratio)
        if motor_select.upper() == 'A':
            motor_angles.append(theta_A * GEAR_RATIO)
        else:
            motor_angles.append(theta_B * GEAR_RATIO)
    
    # Verify continuity - warn if any large jumps remain
    max_jump = 0
    max_jump_idx = 0
    for i in range(1, len(motor_angles)):
        jump = abs(motor_angles[i] - motor_angles[i-1])
        if jump > max_jump:
            max_jump = jump
            max_jump_idx = i
    
    if max_jump > 100:  # More than 100 degrees jump is suspicious
        print(f"Warning: Large angle jump detected at step {max_jump_idx}: {max_jump:.1f} degrees")
    
    # Check wrap-around continuity (last -> first for looping)
    wrap_jump = abs(motor_angles[0] - motor_angles[-1])
    if wrap_jump > 50:  # More than 50 degrees at wrap-around
        print(f"Warning: Wrap-around discontinuity: {wrap_jump:.1f}° (last={motor_angles[-1]:.1f}° -> first={motor_angles[0]:.1f}°)")
    
    return motor_angles

# ============================================================================
# MOTOR COMMUNICATION
# ============================================================================

def send_position(port: serial.Serial, target_degrees: float, mode: str = 'direct', 
                  duration_ms: int = 20, verbose: bool = False) -> bool:
    """Send position command to motor"""
    if mode == 'direct':
        packet = build_direct_position_packet(target_degrees)
    else:
        packet = build_scurve_position_packet(target_degrees, duration_ms)
    
    port.write(packet)
    
    if verbose:
        print(f"[TX] {mode.upper()}: {target_degrees:7.2f}° | Packet: {packet.hex(' ')}")
    
    return True


def send_start_command(port: serial.Serial, max_retries: int = 5) -> Optional[float]:
    """
    Send start command using PING packet
    
    Args:
        port: Serial port object
        max_retries: Maximum number of PING attempts
    
    Returns:
        Current motor position in degrees, or None if no response
    """
    print("[TX] Sending PING to start motor...")
    
    for attempt in range(max_retries):
        packet = build_ping_packet()
        port.write(packet)
        time.sleep(0.5)  # Wait for response
        
        # Try to receive response with longer timeout
        feedback = receive_feedback(port, timeout=0.2)
        if feedback:
            print(f"[RX] Motor {feedback.motor_id} responded: {feedback.position_deg:.2f}° (attempt {attempt + 1})")
            return feedback.position_deg
        
        if attempt < max_retries - 1:
            print(f"[RX] No response (attempt {attempt + 1}/{max_retries}), retrying...")
            port.reset_input_buffer()
    
    print(f"[RX] No response after {max_retries} attempts (motor may still be starting)")
    return None


def find_nearest_trajectory_index(motor_angles: List[float], current_pos: float) -> int:
    """
    Find the index in trajectory that is closest to current position
    
    Args:
        motor_angles: List of motor shaft angles
        current_pos: Current motor position in degrees
    
    Returns:
        Index of nearest position in trajectory
    """
    min_distance = float('inf')
    nearest_idx = 0
    
    for idx, angle in enumerate(motor_angles):
        distance = abs(angle - current_pos)
        if distance < min_distance:
            min_distance = distance
            nearest_idx = idx
    
    return nearest_idx


def reorder_trajectory_from_index(motor_angles: List[float], start_idx: int) -> List[float]:
    """
    Reorder trajectory to start from specified index
    
    Args:
        motor_angles: Original trajectory list
        start_idx: Index to start from
    
    Returns:
        Reordered trajectory list
    """
    return motor_angles[start_idx:] + motor_angles[:start_idx]


def move_to_start_position(port: serial.Serial, target_pos: float, wait_time: float = 3.0, 
                           mode: str = 'scurve', verbose: bool = False):
    """
    Move motor to start position and wait
    
    Args:
        port: Serial port object
        target_pos: Target position in degrees
        wait_time: Time to wait after sending command (seconds)
        mode: Control mode ('direct' or 'scurve')
        verbose: Print debug info
    """
    print(f"\n[INIT] Moving to start position: {target_pos:.2f}°")
    
    # Send position command (use S-curve for smooth motion)
    if mode == 'scurve':
        packet = build_scurve_position_packet(target_pos, int(wait_time * 500))  # Half wait time as duration
    else:
        packet = build_direct_position_packet(target_pos)
    
    port.write(packet)
    
    if verbose:
        print(f"[TX] Moving to start: {target_pos:.2f}° | Packet: {packet.hex(' ')}")
    
    # Wait for motor to reach position
    print(f"[INIT] Waiting {wait_time:.1f}s for motor to reach start position...")
    
    # Monitor progress during wait
    start_time = time.time()
    last_feedback = None
    
    while (time.time() - start_time) < wait_time:
        # Check for feedback
        feedback = receive_feedback(port, timeout=0.1)
        if feedback:
            last_feedback = feedback
            error = abs(feedback.position_deg - target_pos)
            elapsed = time.time() - start_time
            remaining = wait_time - elapsed
            
            print(f"\r[INIT] Position: {feedback.position_deg:7.2f}° | "
                  f"Target: {target_pos:7.2f}° | "
                  f"Error: {error:5.2f}° | "
                  f"Time remaining: {remaining:.1f}s", end='', flush=True)
        
        time.sleep(0.1)
    
    print()  # New line after progress
    
    if last_feedback:
        final_error = abs(last_feedback.position_deg - target_pos)
        print(f"[INIT] Ready! Final position: {last_feedback.position_deg:.2f}° (error: {final_error:.2f}°)")
    else:
        print(f"[INIT] Ready! (no feedback received)")

# ============================================================================
# TRAJECTORY PLAYBACK
# ============================================================================

def playback_trajectory(
    port: serial.Serial,
    motor_angles: List[float],
    rate_hz: float = 50.0,
    mode: str = 'direct',
    verbose: bool = False,
    loop: bool = False
):
    """
    Playback motor trajectory at specified rate
    
    Args:
        port: Serial port object
        motor_angles: List of motor shaft angles in degrees
        rate_hz: Playback rate in Hz
        mode: 'direct' or 'scurve'
        verbose: Print debug info
        loop: Loop trajectory continuously
    """
    period_s = 1.0 / rate_hz
    total_points = len(motor_angles)
    
    print(f"\n{'='*60}")
    print(f"Starting Trajectory Playback")
    print(f"{'='*60}")
    print(f"Total Points:  {total_points}")
    print(f"Rate:          {rate_hz} Hz ({period_s*1000:.2f} ms/point)")
    print(f"Control Mode:  {mode.upper()}")
    print(f"Duration:      {total_points * period_s:.2f} seconds per cycle")
    print(f"Loop:          {'Enabled' if loop else 'Disabled'}")
    print(f"Angle Range:   {min(motor_angles):.2f}° to {max(motor_angles):.2f}°")
    print(f"{'='*60}\n")
    
    # Clear any pending data before starting
    port.reset_input_buffer()
    port.reset_output_buffer()
    
    try:
        iteration = 0
        while True:
            iteration += 1
            print(f"[Iteration {iteration}] Starting trajectory...")
            
            start_time = time.time()
            feedback_count = 0
            error_sum = 0.0
            last_feedback = None
            
            for idx, target_pos in enumerate(motor_angles):
                point_start_time = time.time()
                
                # Send position command
                duration_ms = int(period_s * 1000)
                send_position(port, target_pos, mode=mode, duration_ms=duration_ms, verbose=verbose)
                
                # Small delay for motor to respond
                time.sleep(0.003)
                
                # Try to receive feedback (non-blocking check)
                feedback = receive_feedback(port, timeout=0.002)
                
                if feedback:
                    last_feedback = feedback
                    feedback_count += 1
                    error = abs(feedback.position_deg - target_pos)
                    error_sum += error
                    
                    if verbose:
                        flags_str = []
                        if feedback.is_moving:
                            flags_str.append("MOVING")
                        if feedback.at_goal:
                            flags_str.append("AT_GOAL")
                        if feedback.has_error:
                            flags_str.append("ERROR")
                        if feedback.emergency_stopped:
                            flags_str.append("E-STOP")
                        
                        print(f"  [RX] Motor {feedback.motor_id}: {feedback.position_deg:7.2f}° | "
                              f"Error: {error:5.2f}° | I={feedback.current_ma}mA | "
                              f"Flags: {','.join(flags_str) if flags_str else 'OK'}")
                    else:
                        # Progress bar mode
                        progress = (idx + 1) / total_points * 100
                        bar_len = 40
                        filled = int(bar_len * (idx + 1) / total_points)
                        bar = '█' * filled + '░' * (bar_len - filled)
                        
                        print(f'\r[{bar}] {progress:5.1f}% | '
                              f'Point {idx+1}/{total_points} | '
                              f'Target: {target_pos:7.2f}° | '
                              f'Actual: {feedback.position_deg:7.2f}° | '
                              f'Error: {error:5.2f}°', 
                              end='', flush=True)
                else:
                    if not verbose:
                        progress = (idx + 1) / total_points * 100
                        bar_len = 40
                        filled = int(bar_len * (idx + 1) / total_points)
                        bar = '█' * filled + '░' * (bar_len - filled)
                        
                        # Show last known position if available
                        if last_feedback:
                            print(f'\r[{bar}] {progress:5.1f}% | '
                                  f'Point {idx+1}/{total_points} | '
                                  f'Target: {target_pos:7.2f}° | '
                                  f'Last: {last_feedback.position_deg:7.2f}°', 
                                  end='', flush=True)
                        else:
                            print(f'\r[{bar}] {progress:5.1f}% | '
                                  f'Point {idx+1}/{total_points} | '
                                  f'Target: {target_pos:7.2f}° | '
                                  f'[No feedback]', 
                                  end='', flush=True)
                
                # Timing control
                elapsed = time.time() - point_start_time
                sleep_time = period_s - elapsed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif verbose:
                    print(f"  Warning: Running {-sleep_time*1000:.2f}ms behind schedule")
            
            # End of iteration summary
            total_time = time.time() - start_time
            avg_error = error_sum / feedback_count if feedback_count > 0 else 0
            
            print(f"\n\n[Iteration {iteration}] Completed in {total_time:.2f}s "
                  f"(avg rate: {total_points/total_time:.1f} Hz)")
            print(f"  Feedback received: {feedback_count}/{total_points} ({100*feedback_count/total_points:.1f}%)")
            if feedback_count > 0:
                print(f"  Average error: {avg_error:.2f}°")
            
            if not loop:
                break
            
            # Seamless loop - no delay, continue immediately
            # The trajectory should be designed to wrap smoothly
            
    except KeyboardInterrupt:
        print("\n\nPlayback stopped by user")

# ============================================================================
# MAIN FUNCTION
# ============================================================================

def list_com_ports():
    """List available COM ports"""
    ports = serial.tools.list_ports.comports()
    if ports:
        print("Available COM ports:")
        for port in ports:
            print(f"  {port.device}: {port.description}")
    else:
        print("No COM ports found")
    return [p.device for p in ports]


def main():
    parser = argparse.ArgumentParser(
        description='Gait Control Test - Standalone Version (No CSV Required)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run trot gait at 50 Hz with direct position control
  python test_gait_standalone.py COM3 --rate 50
  
  # Run with S-Curve smoothing
  python test_gait_standalone.py COM3 --mode scurve
  
  # Run backward gait, loop continuously
  python test_gait_standalone.py COM3 --reverse --loop
  
  # Custom gait parameters
  python test_gait_standalone.py COM3 --step 60 --lift 20 --height -180
  
  # List available COM ports
  python test_gait_standalone.py --list-ports
        """
    )
    
    # All arguments now default to CONFIG values from top of file
    parser.add_argument('port', nargs='?', default=CONFIG_PORT,
                        help=f'Serial port (default: {CONFIG_PORT})')
    parser.add_argument('--list-ports', action='store_true', help='List available COM ports')
    parser.add_argument('--motor', choices=['A', 'B'], default=CONFIG_MOTOR,
                        help=f'Motor to control: A or B (default: {CONFIG_MOTOR})')
    parser.add_argument('--rate', type=float, default=CONFIG_RATE_HZ,
                        help=f'Playback rate in Hz (default: {CONFIG_RATE_HZ})')
    parser.add_argument('--mode', choices=['direct', 'scurve'], default=CONFIG_MODE,
                        help=f'Control mode (default: {CONFIG_MODE})')
    parser.add_argument('--baudrate', type=int, default=CONFIG_BAUDRATE,
                        help=f'Serial baudrate (default: {CONFIG_BAUDRATE})')
    # Boolean flags - use BooleanOptionalAction so CONFIG defaults work properly
    # Usage: --loop (enable) or --no-loop (disable), default from CONFIG
    parser.add_argument('--loop', action=argparse.BooleanOptionalAction, default=CONFIG_LOOP,
                        help=f'Loop trajectory continuously (default: {CONFIG_LOOP})')
    parser.add_argument('--verbose', action=argparse.BooleanOptionalAction, default=CONFIG_VERBOSE,
                        help=f'Print detailed debug information (default: {CONFIG_VERBOSE})')
    parser.add_argument('--no-start', action=argparse.BooleanOptionalAction, default=CONFIG_SKIP_START,
                        help=f'Skip sending start command (default: {CONFIG_SKIP_START})')
    parser.add_argument('--reverse', action=argparse.BooleanOptionalAction, default=CONFIG_REVERSE,
                        help=f'Reverse (backward) gait (default: {CONFIG_REVERSE})')
    
    # Gait parameters - defaults from CONFIG
    parser.add_argument('--step', type=float, default=CONFIG_STEP_LENGTH,
                        help=f'Step length in mm (default: {CONFIG_STEP_LENGTH})')
    parser.add_argument('--lift', type=float, default=CONFIG_LIFT_HEIGHT,
                        help=f'Lift height in mm (default: {CONFIG_LIFT_HEIGHT})')
    parser.add_argument('--height', type=float, default=CONFIG_STANCE_HEIGHT,
                        help=f'Stance height in mm (default: {CONFIG_STANCE_HEIGHT})')
    parser.add_argument('--points', type=int, default=CONFIG_NUM_POINTS,
                        help=f'Number of trajectory points (default: {CONFIG_NUM_POINTS})')
    parser.add_argument('--stance-ratio', type=float, default=CONFIG_STANCE_RATIO,
                        help=f'Stance phase ratio 0.0-1.0 (default: {CONFIG_STANCE_RATIO})')
    parser.add_argument('--init-wait', type=float, default=CONFIG_INIT_WAIT,
                        help=f'Wait time for motor to reach start position (default: {CONFIG_INIT_WAIT}s)')
    
    args = parser.parse_args()
    
    # List ports mode
    if args.list_ports:
        list_com_ports()
        return 0
    
    # Check port argument
    if not args.port or args.port == "":
        print("Error: Serial port required.")
        print(f"  - Edit CONFIG_PORT at top of this file, or")
        print(f"  - Use: python {__file__} COM3")
        print(f"  - Use --list-ports to see available ports.")
        return 1
    
    # Print current configuration
    print(f"\n{'='*60}")
    print(f"Configuration (edit at top of file to change defaults)")
    print(f"{'='*60}")
    print(f"  Port:       {args.port} @ {args.baudrate} baud")
    print(f"  Motor:      {args.motor}")
    print(f"  Rate:       {args.rate} Hz")
    print(f"  Mode:       {args.mode}")
    print(f"  Loop:       {args.loop}")
    print(f"  Verbose:    {args.verbose}")
    
    # Generate trajectory
    print(f"\n{'='*60}")
    print(f"Generating Elliptical Trajectory (Trot Gait)")
    print(f"{'='*60}")
    print(f"Step length:   {args.step:.1f} mm")
    print(f"Lift height:   {args.lift:.1f} mm")
    print(f"Stance height: {args.height:.1f} mm")
    print(f"Points:        {args.points}")
    print(f"Stance ratio:  {args.stance_ratio:.0%}")
    print(f"Direction:     {'Backward' if args.reverse else 'Forward'}")
    print(f"Motor:         {args.motor}")
    
    # Generate foot trajectory
    foot_trajectory = generate_elliptical_trajectory(
        step_forward=args.step,
        lift_height=args.lift,
        num_steps=args.points,
        stance_ratio=args.stance_ratio,
        home_y=args.height,
        reverse=args.reverse
    )
    
    print(f"\nFoot trajectory:")
    print(f"  X range: {min(p[0] for p in foot_trajectory):.1f} to {max(p[0] for p in foot_trajectory):.1f} mm")
    print(f"  Y range: {min(p[1] for p in foot_trajectory):.1f} to {max(p[1] for p in foot_trajectory):.1f} mm")
    
    # Convert to motor angles
    motor_angles = trajectory_to_motor_angles(foot_trajectory, motor_select=args.motor)
    
    print(f"\nMotor {args.motor} shaft angles:")
    print(f"  Range: {min(motor_angles):.2f}° to {max(motor_angles):.2f}°")
    print(f"  Wrap-around jump: {abs(motor_angles[0] - motor_angles[-1]):.2f}° (last->first)")
    
    # Open serial port
    try:
        print(f"\nConnecting to {args.port} at {args.baudrate} baud...")
        port = serial.Serial(args.port, args.baudrate, timeout=0.1)
        time.sleep(0.5)
        print("Connected!")
        
        # Clear any pending data
        port.reset_input_buffer()
        port.reset_output_buffer()
        
        # Send start command and get current position
        current_pos = None
        if not args.no_start:
            current_pos = send_start_command(port)
        
        # Find nearest trajectory point and reorder
        if current_pos is not None:
            nearest_idx = find_nearest_trajectory_index(motor_angles, current_pos)
            nearest_angle = motor_angles[nearest_idx]
            print(f"\n[INIT] Current position: {current_pos:.2f}°")
            print(f"[INIT] Nearest trajectory point: index {nearest_idx}, angle {nearest_angle:.2f}°")
            
            # Reorder trajectory to start from nearest point
            motor_angles = reorder_trajectory_from_index(motor_angles, nearest_idx)
            print(f"[INIT] Trajectory reordered to start from index {nearest_idx}")
        else:
            print(f"\n[INIT] No current position feedback, using original trajectory order")
        
        # Move to start position and wait
        start_position = motor_angles[0]
        move_to_start_position(port, start_position, wait_time=args.init_wait, 
                               mode='scurve', verbose=args.verbose)
        
        # Start playback
        playback_trajectory(
            port,
            motor_angles,
            rate_hz=args.rate,
            mode=args.mode,
            verbose=args.verbose,
            loop=args.loop
        )
        
        print("\nDone!")
        port.close()
        
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
