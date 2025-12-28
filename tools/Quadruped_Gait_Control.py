"""
Quadruped Gait Control - Binary Protocol v1.2
Author: M-TRCH
Date: December 26, 2025

This script controls a quadruped robot (4 legs, 8 motors) using Binary Protocol v1.2.
Features:
- Automatic motor discovery and registration via COM port scanning
- Motor ID detection using PING command
- IK calculation for all 4 legs
- Trot gait implementation
- Real-time visualization

Protocol: Binary Protocol v1.2 (see PROTOCOL.md)
"""

import numpy as np
import serial
import serial.tools.list_ports
import time
import threading
import struct
import traceback
import sys
import os
from enum import IntEnum
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Windows keyboard input
if sys.platform == 'win32':
    import msvcrt

# ============================================================================
# PROTOCOL CONSTANTS (v1.2)
# ============================================================================

HEADER_1 = 0xFE
HEADER_2 = 0xEE

class PacketType(IntEnum):
    """Packet type enumeration"""
    PKT_CMD_SET_GOAL = 0x01
    PKT_CMD_PING = 0x03
    PKT_CMD_EMERGENCY_STOP = 0x04
    PKT_FB_STATUS = 0x81
    PKT_FB_ERROR = 0x83

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
# COMMUNICATION PARAMETERS
# ============================================================================

BAUD_RATE = 921600
SERIAL_TIMEOUT = 0.05  # 50ms timeout for discovery
FAST_TIMEOUT = 0.002   # 2ms timeout for high-speed operation
PING_RETRIES = 3       # Number of PING retries during discovery

# ============================================================================
# ROBOT CONFIGURATION
# ============================================================================

# --- Robot Body Dimensions (mm) ---
BODY_LENGTH = 200.0   # Distance between front and rear hip axes
BODY_WIDTH = 170.0    # Distance between left and right hip axes

# --- Five-Bar Linkage Parameters (mm) ---
MOTOR_SPACING = 85.0  # Distance between Motor A and Motor B (horizontal)

# Link lengths (same for all 4 legs)
L_AC = 105.0  # Link 1 length (Motor A to joint C)
L_BD = 105.0  # Link 2 length (Motor B to joint D)
L_CE = 145.0  # Link 3 length (joint C to joint E)
L_DE = 145.0  # Link 4 length (joint D to joint E)
L_EF = 40.0   # Offset length (joint E to foot F)

# Offset ratios
OFFSET_RATIO_E = 37.0 / 29.0
OFFSET_RATIO_D = 8.0 / 29.0

# --- Motor Configuration ---
GEAR_RATIO = 8.0  # Motor shaft to output shaft gear ratio
MOTOR_INIT_ANGLE = -90.0  # Initial motor angle (degrees)

# --- Expected Motor IDs for Each Leg ---
# Motor A (inner) and Motor B (outer) indices
EXPECTED_MOTOR_IDS = {
    'FL': {'A': 1, 'B': 2},  # Front Left
    'FR': {'A': 3, 'B': 4},  # Front Right
    'RL': {'A': 5, 'B': 6},  # Rear Left
    'RR': {'A': 7, 'B': 8}   # Rear Right
}

# --- Leg Motor Positions in Leg Frame (mm) ---
P_A_LEFT = np.array([-MOTOR_SPACING/2, 0.0])
P_B_LEFT = np.array([MOTOR_SPACING/2, 0.0])
P_A_RIGHT = np.array([MOTOR_SPACING/2, 0.0])
P_B_RIGHT = np.array([-MOTOR_SPACING/2, 0.0])

# --- Default Standing Pose ---
DEFAULT_STANCE_HEIGHT = -200.0  # mm (negative = down)
DEFAULT_STANCE_OFFSET_X = 0.0   # mm

# --- Motion Parameters ---
GAIT_LIFT_HEIGHT = 15.0    # mm (ยกขาต่ำลง)
GAIT_STEP_FORWARD = 30.0   # mm (ก้าวสั้นลง)

# ============================================================================
# CONTROL PARAMETERS
# ============================================================================

CONTROL_MODE = ControlMode.MODE_DIRECT_POSITION  # Control mode for gait control
UPDATE_RATE = 50  # Hz (20ms per update)
TRAJECTORY_STEPS = 30  # Number of steps in one gait cycle (เดินเร็วขึ้น)
GAIT_TYPE = 'trot'  # 'trot', 'walk', 'stand'

# --- Single Motor Mode ---
SINGLE_MOTOR_MODE = False  # Set to True to enable single motor testing
SINGLE_MOTOR_OSCILLATION = 30.0  # Oscillation amplitude in degrees
SINGLE_MOTOR_PERIOD = 0.6  # Oscillation period in seconds (600ms)
 
# --- Visualization Parameters ---
ENABLE_VISUALIZATION = False
PLOT_UPDATE_RATE = 10  # Hz 

# ============================================================================
# GLOBAL VARIABLES
# ============================================================================

# Thread-safe locks
viz_lock = threading.Lock()
error_lock = threading.Lock()
control_lock = threading.Lock()

# Visualization state
plot_running = True

# Control state
gait_running = False
gait_paused = True

# Leg states
# Note: Motors start at MOTOR_INIT_ANGLE (-90 deg), IK home position is different!
# actual_angles initialized to motor init angle (radians)
MOTOR_INIT_ANGLE_RAD = np.deg2rad(MOTOR_INIT_ANGLE)
leg_states = {
    'FR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'red'},
    'FL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'blue'},
    'RR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'orange'},
    'RL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'green'}
}

# Motor registry (populated during discovery)
motor_registry = {}  # {motor_id: motor_controller}
leg_motors = {}      # {leg_id: {'A': motor_controller, 'B': motor_controller}}

# Error tracking
error_stats = {}

# Single motor mode state
single_motor_controller = None
single_motor_start_time = None

# ============================================================================
# PROTOCOL FUNCTIONS
# ============================================================================

def calculate_crc16(data: bytes) -> int:
    """Calculate CRC-16-IBM for data buffer"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def build_packet(pkt_type: int, payload: bytes = b'') -> bytes:
    """Build a complete binary packet with CRC"""
    header = bytes([HEADER_1, HEADER_2])
    type_byte = bytes([pkt_type])
    payload_len = bytes([len(payload)])
    
    # Calculate CRC over: pkt_type + payload_len + payload
    crc_data = type_byte + payload_len + payload
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    
    return header + type_byte + payload_len + payload + crc_bytes

# ============================================================================
# MOTOR CONTROLLER CLASS
# ============================================================================

class BinaryMotorController:
    """High-speed motor controller using binary protocol v1.2"""
    
    def __init__(self, port, motor_id=None):
        self.port = port
        self.motor_id = motor_id
        self.serial = None
        self.is_connected = False
        self.current_setpoint = 0.0
        self.current_position = 0.0
        self.current_current = 0
        self.current_flags = 0
        self.lock = threading.Lock()
        self.stats_tx_count = 0
        self.stats_rx_count = 0
        self.stats_errors = 0
    
    def connect(self, timeout=SERIAL_TIMEOUT):
        """Connect to motor via serial port"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=BAUD_RATE,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(0.1)
            
            # Flush buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            self.is_connected = True
            return True
            
        except Exception as e:
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from motor"""
        if self.serial and self.is_connected:
            try:
                self.serial.close()
            except:
                pass
            self.is_connected = False
    
    def set_timeout(self, timeout):
        """Update serial timeout"""
        if self.serial and self.is_connected:
            self.serial.timeout = timeout
    
    def send_ping(self) -> dict:
        """
        Send PING command and wait for response
        Returns motor info dict or None if no response
        """
        if not self.is_connected:
            return None
        
        try:
            # Flush buffers
            self.serial.reset_input_buffer()
            
            # Build and send PING packet
            packet = build_packet(PacketType.PKT_CMD_PING)
            self.serial.write(packet)
            self.stats_tx_count += 1
            
            # Read response
            response = self._read_packet()
            if response:
                pkt_type, payload = response
                if pkt_type == PacketType.PKT_FB_STATUS and len(payload) >= 8:
                    motor_id = payload[0]
                    position_raw = struct.unpack('<i', payload[1:5])[0]
                    current_raw = struct.unpack('<h', payload[5:7])[0]
                    status_flags = payload[7]
                    
                    self.motor_id = motor_id
                    self.current_position = (position_raw / 100.0) / GEAR_RATIO
                    self.current_current = current_raw
                    self.current_flags = status_flags
                    
                    return {
                        'motor_id': motor_id,
                        'position': self.current_position,
                        'current': current_raw,
                        'flags': status_flags
                    }
            return None
            
        except Exception as e:
            return None
    
    def start_motor(self) -> bool:
        """
        Start motor using PING command (Binary Protocol initialization)
        Returns True if motor responded successfully
        """
        for retry in range(PING_RETRIES):
            result = self.send_ping()
            if result:
                print(f"  ✅ Motor ID {result['motor_id']} started on {self.port}")
                time.sleep(0.1)  # Small delay after start
                return True
            time.sleep(0.1)
        return False
    
    def set_position_direct(self, angle_deg: float) -> bool:
        """
        Set target position using direct mode (fastest)
        
        Args:
            angle_deg: Target angle in robot coordinate (degrees)
            
        Returns:
            True if command sent successfully
        """
        if not self.is_connected:
            return False
        
        try:
            motor_angle = angle_deg * GEAR_RATIO
            
            with self.lock:
                # Build payload: mode + target_pos (int32, degrees*100)
                mode = bytes([ControlMode.MODE_DIRECT_POSITION])
                target_pos = struct.pack('<i', int(motor_angle * 100))
                payload = mode + target_pos
                
                # Build and send packet
                packet = build_packet(PacketType.PKT_CMD_SET_GOAL, payload)
                self.serial.write(packet)
                self.serial.flush()  # Ensure data is sent immediately
                self.stats_tx_count += 1
                self.current_setpoint = angle_deg
                
                # Try to read feedback response (non-blocking)
                # Wait a short time for response
                time.sleep(0.001)  # 1ms wait for response
                if self.serial.in_waiting >= 12:  # Minimum packet size
                    result = self._read_packet()
                    if result:
                        pkt_type, payload_data = result
                        if pkt_type == PacketType.PKT_FB_STATUS and len(payload_data) >= 8:
                            position_raw = struct.unpack('<i', payload_data[1:5])[0]
                            current_raw = struct.unpack('<h', payload_data[5:7])[0]
                            status_flags = payload_data[7]
                            self.current_position = (position_raw / 100.0) / GEAR_RATIO
                            self.current_current = current_raw
                            self.current_flags = status_flags
            
            return True
            
        except Exception as e:
            self.stats_errors += 1
            return False
    
    def set_position_scurve(self, angle_deg: float, duration_ms: int) -> bool:
        """
        Set target position using S-Curve profile
        
        Args:
            angle_deg: Target angle in robot coordinate (degrees)
            duration_ms: Movement duration in milliseconds
            
        Returns:
            True if command sent successfully
        """
        if not self.is_connected:
            return False
        
        try:
            motor_angle = angle_deg * GEAR_RATIO
            
            with self.lock:
                mode = bytes([ControlMode.MODE_SCURVE_PROFILE])
                target_pos = struct.pack('<i', int(motor_angle * 100))
                duration = struct.pack('<H', duration_ms)
                payload = mode + target_pos + duration
                
                packet = build_packet(PacketType.PKT_CMD_SET_GOAL, payload)
                self.serial.write(packet)
                self.serial.flush()  # Ensure data is sent immediately
                self.stats_tx_count += 1
                self.current_setpoint = angle_deg
                
                # Try to read feedback response (non-blocking)
                time.sleep(0.001)  # 1ms wait for response
                if self.serial.in_waiting >= 12:
                    result = self._read_packet()
                    if result:
                        pkt_type, payload_data = result
                        if pkt_type == PacketType.PKT_FB_STATUS and len(payload_data) >= 8:
                            position_raw = struct.unpack('<i', payload_data[1:5])[0]
                            current_raw = struct.unpack('<h', payload_data[5:7])[0]
                            status_flags = payload_data[7]
                            self.current_position = (position_raw / 100.0) / GEAR_RATIO
                            self.current_current = current_raw
                            self.current_flags = status_flags
            
            return True
            
        except Exception as e:
            self.stats_errors += 1
            return False
    
    def send_emergency_stop(self) -> bool:
        """Send emergency stop command"""
        if not self.is_connected:
            return False
        
        try:
            packet = build_packet(PacketType.PKT_CMD_EMERGENCY_STOP)
            self.serial.write(packet)
            self.stats_tx_count += 1
            print(f"  ⚠️  Emergency stop sent to Motor ID {self.motor_id}")
            return True
        except:
            return False
    
    def _read_packet(self) -> tuple:
        """
        Read a complete packet from serial
        Returns (pkt_type, payload) or None
        """
        try:
            # Read header
            header = self.serial.read(2)
            if len(header) != 2 or header[0] != HEADER_1 or header[1] != HEADER_2:
                return None
            
            # Read packet type and length
            meta = self.serial.read(2)
            if len(meta) != 2:
                return None
            
            pkt_type = meta[0]
            payload_len = meta[1]
            
            # Read payload
            payload = self.serial.read(payload_len)
            if len(payload) != payload_len:
                return None
            
            # Read CRC
            crc_bytes = self.serial.read(2)
            if len(crc_bytes) != 2:
                return None
            
            received_crc = struct.unpack('<H', crc_bytes)[0]
            
            # Verify CRC
            crc_data = bytes([pkt_type, payload_len]) + payload
            calculated_crc = calculate_crc16(crc_data)
            
            if received_crc != calculated_crc:
                self.stats_errors += 1
                return None
            
            self.stats_rx_count += 1
            return (pkt_type, payload)
            
        except Exception as e:
            return None
    
    def read_feedback(self) -> dict:
        """
        Read motor feedback from binary protocol
        Returns dictionary with feedback data or None
        """
        if not self.is_connected:
            return None
        
        try:
            if self.serial.in_waiting < 2:
                return None
            
            result = self._read_packet()
            if result:
                pkt_type, payload = result
                if pkt_type == PacketType.PKT_FB_STATUS and len(payload) >= 8:
                    motor_id = payload[0]
                    position_raw = struct.unpack('<i', payload[1:5])[0]
                    current_raw = struct.unpack('<h', payload[5:7])[0]
                    status_flags = payload[7]
                    
                    with self.lock:
                        self.current_position = (position_raw / 100.0) / GEAR_RATIO
                        self.current_current = current_raw
                        self.current_flags = status_flags
                    
                    return {
                        'motor_id': motor_id,
                        'position': self.current_position,
                        'current': current_raw,
                        'flags': status_flags,
                        'is_moving': bool(status_flags & StatusFlags.STATUS_MOVING),
                        'at_goal': bool(status_flags & StatusFlags.STATUS_AT_GOAL),
                        'error': bool(status_flags & StatusFlags.STATUS_ERROR),
                        'emergency_stopped': bool(status_flags & StatusFlags.STATUS_EMERGENCY_STOPPED)
                    }
            return None
            
        except Exception as e:
            return None
    
    def get_current_position(self):
        """Get current position (thread-safe)"""
        with self.lock:
            return self.current_position
    
    def get_stats(self):
        """Get communication statistics"""
        return {
            'motor_id': self.motor_id,
            'tx': self.stats_tx_count,
            'rx': self.stats_rx_count,
            'errors': self.stats_errors,
            'success_rate': (self.stats_rx_count / max(1, self.stats_tx_count)) * 100
        }

# ============================================================================
# MOTOR DISCOVERY AND REGISTRATION
# ============================================================================

def scan_com_ports() -> list:
    """Scan for available COM ports"""
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append({
            'device': port.device,
            'description': port.description,
            'hwid': port.hwid
        })
    return ports

def discover_motors() -> dict:
    """
    Discover all motors by scanning COM ports and sending PING
    Returns dictionary of {motor_id: motor_controller}
    """
    print("\n" + "="*70)
    print("  🔍 MOTOR DISCOVERY")
    print("="*70)
    
    # Get available COM ports
    ports = scan_com_ports()
    
    if not ports:
        print("  ❌ No COM ports found!")
        return {}
    
    print(f"\n  Found {len(ports)} COM port(s):")
    for port in ports:
        print(f"    • {port['device']}: {port['description']}")
    
    # Dictionary to store discovered motors
    discovered = {}
    
    print(f"\n  📡 Scanning for motors (timeout={SERIAL_TIMEOUT*1000:.0f}ms, retries={PING_RETRIES})...")
    
    for port_info in ports:
        port = port_info['device']
        print(f"\n    Checking {port}...")
        
        # Create temporary controller for discovery
        controller = BinaryMotorController(port)
        
        if not controller.connect(timeout=SERIAL_TIMEOUT):
            print(f"      ❌ Failed to open {port}")
            continue
        
        # Try PING multiple times
        motor_info = None
        for attempt in range(PING_RETRIES):
            motor_info = controller.send_ping()
            if motor_info:
                break
            time.sleep(0.05)
        
        if motor_info:
            motor_id = motor_info['motor_id']
            print(f"      ✅ Motor ID {motor_id} found!")
            print(f"         Position: {motor_info['position']:.2f}°")
            print(f"         Current:  {motor_info['current']} mA")
            print(f"         Flags:    0x{motor_info['flags']:02X}")
            
            # Store in discovered dictionary
            discovered[motor_id] = controller
        else:
            print(f"      ⚠️  No response (no motor on this port)")
            controller.disconnect()
    
    print(f"\n  📊 Discovery complete: {len(discovered)} motor(s) found")
    return discovered

def register_leg_motors(discovered_motors: dict) -> bool:
    """
    Register discovered motors to leg assignments
    Returns True if all required motors are found
    """
    global leg_motors, motor_registry
    
    print("\n" + "="*70)
    print("  🔗 MOTOR REGISTRATION")
    print("="*70)
    
    motor_registry = discovered_motors
    leg_motors = {}
    
    # Check each leg's motor assignments
    all_assigned = True
    missing_motors = []
    
    for leg_id, motor_ids in EXPECTED_MOTOR_IDS.items():
        motor_a_id = motor_ids['A']
        motor_b_id = motor_ids['B']
        
        motor_a = discovered_motors.get(motor_a_id)
        motor_b = discovered_motors.get(motor_b_id)
        
        if motor_a and motor_b:
            leg_motors[leg_id] = {
                'A': motor_a,
                'B': motor_b,
                'motor_a_id': motor_a_id,
                'motor_b_id': motor_b_id
            }
            print(f"    ✅ {leg_id}: Motor A (ID {motor_a_id}) + Motor B (ID {motor_b_id})")
        else:
            all_assigned = False
            if not motor_a:
                missing_motors.append(f"{leg_id} Motor A (ID {motor_a_id})")
            if not motor_b:
                missing_motors.append(f"{leg_id} Motor B (ID {motor_b_id})")
            print(f"    ❌ {leg_id}: INCOMPLETE - Missing motor(s)")
    
    if missing_motors:
        print(f"\n  ⚠️  Missing motors:")
        for m in missing_motors:
            print(f"      • {m}")
    
    return all_assigned

def start_all_motors() -> bool:
    """Start all registered motors using Binary Protocol PING"""
    print("\n" + "="*70)
    print("  🚀 STARTING MOTORS")
    print("="*70)
    
    success_count = 0
    total_count = len(motor_registry)
    
    for motor_id, controller in motor_registry.items():
        # Switch to fast timeout after discovery
        controller.set_timeout(FAST_TIMEOUT)
        
        # Send PING to start motor
        result = controller.send_ping()
        if result:
            success_count += 1
        else:
            print(f"  ❌ Failed to start Motor ID {motor_id}")
    
    print(f"\n  📊 Started {success_count}/{total_count} motors")
    
    # Wait for motors to initialize
    if success_count > 0:
        print("  ⏳ Waiting for motor initialization...")
        time.sleep(2.0)
    
    return success_count == total_count

# ============================================================================
# KINEMATICS FUNCTIONS
# ============================================================================

def get_motor_positions(leg_id):
    """Get motor positions based on leg side"""
    if leg_id in ['FL', 'RL']:
        return P_A_LEFT.copy(), P_B_LEFT.copy()
    else:  # FR, RR
        return P_A_RIGHT.copy(), P_B_RIGHT.copy()

def solve_circle_intersection(center1, radius1, center2, radius2, choose_lower=True):
    """Find intersection of two circles"""
    V_12 = center2 - center1
    d = np.linalg.norm(V_12)
    
    if d > (radius1 + radius2) or d < abs(radius1 - radius2) or d == 0:
        return np.array([np.nan, np.nan])
    
    a = (radius1**2 - radius2**2 + d**2) / (2 * d)
    h_squared = radius1**2 - a**2
    
    if h_squared < 0:
        return np.array([np.nan, np.nan])
    
    h = np.sqrt(h_squared)
    v_d = V_12 / d
    v_perp = np.array([-v_d[1], v_d[0]])
    
    P_intersection_1 = center1 + a * v_d + h * v_perp
    P_intersection_2 = center1 + a * v_d - h * v_perp
    
    if choose_lower:
        return P_intersection_2 if P_intersection_2[1] < P_intersection_1[1] else P_intersection_1
    else:
        return P_intersection_1 if P_intersection_1[1] > P_intersection_2[1] else P_intersection_2

def calculate_ik_analytical(P_F_target, P_A, P_B, elbow_C_down=True, elbow_D_down=True):
    """
    Calculate Inverse Kinematics using analytical method
    
    Args:
        P_F_target: Target foot position [x, y] in leg frame
        P_A: Motor A position [x, y]
        P_B: Motor B position [x, y]
        elbow_C_down: True to choose lower elbow position for joint C
        elbow_D_down: True to choose lower elbow position for joint D
        
    Returns:
        numpy array [theta_A, theta_B] in radians, or [nan, nan] if no solution
    """
    R_FD = L_DE * OFFSET_RATIO_E
    R_DB = L_BD
    
    # Find joint D position (switched: elbow_D_down=True → choose upper)
    P_D = solve_circle_intersection(P_F_target, R_FD, P_B, R_DB, not elbow_D_down)
    
    if np.isnan(P_D).any():
        return np.array([np.nan, np.nan])
    
    # Calculate joint E position
    P_E = (29.0 * P_F_target + 8.0 * P_D) / 37.0
    
    # Find joint C position
    P_C = solve_circle_intersection(P_A, L_AC, P_E, L_CE, elbow_C_down)
    
    if np.isnan(P_C).any():
        return np.array([np.nan, np.nan])
    
    # Calculate motor angles
    V_AC = P_C - P_A
    V_BD = P_D - P_B
    
    theta_A = np.arctan2(V_AC[1], V_AC[0])
    theta_B = np.arctan2(V_BD[1], V_BD[0])
    
    return np.array([theta_A, theta_B])

def calculate_fk_positions(theta_A, theta_B, P_A, P_B):
    """Calculate forward kinematics positions for visualization"""
    P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
    P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])
    
    V_CD = P_D - P_C
    d = np.linalg.norm(V_CD)
    
    if d > 0 and d <= (L_CE + L_DE) and d >= abs(L_CE - L_DE):
        a = (L_CE**2 - L_DE**2 + d**2) / (2 * d)
        h_squared = L_CE**2 - a**2
        
        if h_squared >= 0:
            h = np.sqrt(h_squared)
            v_d = V_CD / d
            v_perp = np.array([-v_d[1], v_d[0]])
            
            P_E1 = P_C + a * v_d + h * v_perp
            P_E2 = P_C + a * v_d - h * v_perp
            
            P_F1 = (37.0 * P_E1 - 8.0 * P_D) / 29.0
            P_F2 = (37.0 * P_E2 - 8.0 * P_D) / 29.0
            
            # Choose configuration with lower foot position
            if P_F1[1] < P_F2[1]:
                return P_C, P_D, P_E1, P_F1
            else:
                return P_C, P_D, P_E2, P_F2
    
    return None, None, None, None

# ============================================================================
# TRAJECTORY GENERATION
# ============================================================================

def generate_elliptical_trajectory(num_steps=60, lift_height=30.0, step_forward=60.0, mirror_x=False):
    """
    Generate elliptical walking trajectory for one leg
    
    Args:
        num_steps: Number of steps in trajectory
        lift_height: Maximum height lift (mm)
        step_forward: Step length (mm)
        mirror_x: If True, reverse X direction (for right side legs)
        
    Returns:
        List of (x, y) positions in leg frame
    """
    trajectory = []
    home_y = DEFAULT_STANCE_HEIGHT
    
    a = step_forward
    b = lift_height
    
    for i in range(num_steps):
        t = 2 * np.pi * i / num_steps
        px = a * np.cos(t)
        if mirror_x:
            px = -px
        py = home_y + b * np.sin(t)
        trajectory.append((px, py))
    
    return trajectory

def get_gait_phase_offset(leg_id, gait_type='trot'):
    """
    Get phase offset for each leg based on gait type
    
    Args:
        leg_id: Leg identifier ('FR', 'FL', 'RR', 'RL')
        gait_type: Type of gait ('trot', 'walk', 'stand')
        
    Returns:
        Phase offset in range [0, 1)
    """
    if gait_type == 'trot':
        # Trot: diagonal legs move together
        # FR+RL in phase, FL+RR opposite phase
        offsets = {'FR': 0.0, 'FL': 0.5, 'RR': 0.5, 'RL': 0.0}
        return offsets[leg_id]
    
    elif gait_type == 'walk':
        # Walk: legs move in sequence FR -> RR -> FL -> RL
        offsets = {'FR': 0.0, 'RR': 0.25, 'FL': 0.5, 'RL': 0.75}
        return offsets[leg_id]
    
    elif gait_type == 'stand':
        return 0.0
    
    return 0.0

# ============================================================================
# CONTROL FUNCTIONS
# ============================================================================

def toggle_gait_control():
    """Toggle gait control on/off"""
    global gait_running, gait_paused
    with control_lock:
        if gait_paused:
            gait_running = True
            gait_paused = False
            print("\n▶️  Gait control STARTED!")
        else:
            gait_running = False
            gait_paused = True
            print("\n⏸️  Gait control PAUSED!")

def reset_error_stats():
    """Reset all error statistics"""
    global error_stats
    with error_lock:
        error_stats.clear()
    print("\n🔄 Error statistics reset!")

def emergency_stop_all():
    """Send emergency stop to all motors"""
    print("\n⚠️  EMERGENCY STOP - ALL MOTORS!")
    for motor_id, controller in motor_registry.items():
        controller.send_emergency_stop()

# ============================================================================
# SINGLE MOTOR CONTROL
# ============================================================================

def check_keyboard_input():
    """
    Check for keyboard input (non-blocking)
    Returns the key pressed or None
    """
    if sys.platform == 'win32':
        if msvcrt.kbhit():
            key = msvcrt.getch()
            # Handle special keys
            if key == b'\xe0' or key == b'\x00':
                msvcrt.getch()  # Consume the second byte
                return None
            return key.decode('utf-8', errors='ignore').lower()
    return None


def run_single_motor_mode(controller):
    """
    Run single motor oscillation test
    
    Args:
        controller: BinaryMotorController instance
    """
    global gait_running, gait_paused, plot_running
    
    print("\n" + "="*70)
    print("  🔧 SINGLE MOTOR TEST MODE")
    print("="*70)
    print(f"  Motor ID: {controller.motor_id}")
    print(f"  Oscillation: ±{SINGLE_MOTOR_OSCILLATION}°")
    print(f"  Period: {SINGLE_MOTOR_PERIOD}s")
    print(f"  Control Mode: {'S-Curve' if CONTROL_MODE == ControlMode.MODE_SCURVE_PROFILE else 'Direct'}")
    print("="*70)
    
    # Start motor
    print("\n🚀 Starting motor...")
    if not controller.start_motor():
        print("❌ Failed to start motor!")
        return
    
    # Switch to fast timeout
    controller.set_timeout(FAST_TIMEOUT)
    
    # Get initial position
    initial_pos = controller.current_position
    print(f"  Initial position: {initial_pos:.2f}°")
    
    # Use initial position as center (relative movement)
    center_pos = initial_pos
    print(f"\n🏠 Using current position as center ({center_pos:.2f}°)")
    print(f"  Oscillation range: {center_pos - SINGLE_MOTOR_OSCILLATION:.1f}° to {center_pos + SINGLE_MOTOR_OSCILLATION:.1f}°")
    
    # Small delay for stabilization
    time.sleep(0.5)
    
    print("\n⏸️  Single motor control ready (PAUSED)")
    print("  Press [SPACE] to start/pause oscillation")
    print("  Press [E] for emergency stop")
    print("  Press [Q] to quit")
    print("="*70)
    
    gait_paused = True
    start_time = time.time()
    last_status_time = 0
    running = True
    
    try:
        while running:
            # Check keyboard input
            key = check_keyboard_input()
            if key:
                if key == ' ':
                    # Before resuming, get current motor position via PING
                    with control_lock:
                        was_paused = gait_paused
                    
                    if was_paused:
                        # About to resume - query current position
                        ping_result = controller.send_ping()
                        if ping_result:
                            center_pos = ping_result['position']
                            print(f"\n▶️  Resuming from position: {center_pos:.2f}°")
                            print(f"  New oscillation range: {center_pos - SINGLE_MOTOR_OSCILLATION:.1f}° to {center_pos + SINGLE_MOTOR_OSCILLATION:.1f}°")
                        else:
                            print("\n⚠️  Failed to get current position, using last known")
                        start_time = time.time()  # Reset start time for smooth oscillation
                    
                    toggle_gait_control()
                elif key == 'e':
                    controller.send_emergency_stop()
                    with control_lock:
                        gait_paused = True
                    print("\n⚠️  Emergency stop activated!")
                elif key == 'q':
                    print("\n⏹️  Quit requested...")
                    running = False
                    continue
            
            # Check if paused
            with control_lock:
                is_paused = gait_paused
            
            if is_paused:
                time.sleep(0.01)  # Short sleep when paused
                continue
            
            loop_start = time.perf_counter()
            
            # Calculate oscillation angle
            elapsed = time.time() - start_time
            phase = (elapsed % SINGLE_MOTOR_PERIOD) / SINGLE_MOTOR_PERIOD
            target_angle = center_pos + SINGLE_MOTOR_OSCILLATION * np.sin(2 * np.pi * phase)
            
            # Send command
            if CONTROL_MODE == ControlMode.MODE_SCURVE_PROFILE:
                duration_ms = int(1000 / UPDATE_RATE)
                controller.set_position_scurve(target_angle, duration_ms)
            else:
                controller.set_position_direct(target_angle)
            
            # Read feedback
            feedback = controller.read_feedback()
            
            # Print status periodically (every second)
            current_time = time.time()
            if current_time - last_status_time >= 1.0:
                last_status_time = current_time
                cycle = int(elapsed / SINGLE_MOTOR_PERIOD)
                current_pos = controller.current_position
                error = target_angle - current_pos
                print(f"  🔄 Cycle {cycle}: Target={target_angle:+6.1f}° | "
                      f"Actual={current_pos:+6.1f}° | Error={error:+5.1f}°")
            
            # Timing control
            elapsed_loop = time.perf_counter() - loop_start
            sleep_time = max(0, (1.0 / UPDATE_RATE) - elapsed_loop)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n⏹️  Single motor test stopped by user")
    
    finally:
        # Return to init position
        print("\n🏠 Returning to init position (-90°)...")
        controller.set_position_scurve(MOTOR_INIT_ANGLE, 2000)
        time.sleep(2.5)
        
        # Print statistics
        stats = controller.get_stats()
        print(f"\n📊 Communication Statistics:")
        print(f"    Motor {stats['motor_id']}: TX={stats['tx']}, RX={stats['rx']}, "
              f"Errors={stats['errors']}, Success={stats['success_rate']:.1f}%")
        
        # Disconnect
        print("\n🔌 Disconnecting motor...")
        controller.disconnect()
        
        print("\n✅ Single motor test completed")
        print("="*70)


def run_single_motor_with_visualization(controller):
    """
    Run single motor test with simple visualization
    
    Args:
        controller: BinaryMotorController instance
    """
    global gait_running, gait_paused
    
    print("\n" + "="*70)
    print("  🔧 SINGLE MOTOR TEST MODE (with Visualization)")
    print("="*70)
    print(f"  Motor ID: {controller.motor_id}")
    print(f"  Oscillation: ±{SINGLE_MOTOR_OSCILLATION}°")
    print(f"  Period: {SINGLE_MOTOR_PERIOD}s")
    print("="*70)
    
    # Start motor
    print("\n🚀 Starting motor...")
    if not controller.start_motor():
        print("❌ Failed to start motor!")
        return
    
    controller.set_timeout(FAST_TIMEOUT)
    initial_pos = controller.current_position
    print(f"  Initial position: {initial_pos:.2f}°")
    
    # Use initial position as center (relative movement)
    center_pos = initial_pos
    print(f"\n🏠 Using current position as center ({center_pos:.2f}°)")
    print(f"  Oscillation range: {center_pos - SINGLE_MOTOR_OSCILLATION:.1f}° to {center_pos + SINGLE_MOTOR_OSCILLATION:.1f}°")
    
    # Small delay for stabilization
    time.sleep(0.5)
    
    # Create visualization
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle(f'Single Motor Test - Motor ID {controller.motor_id}', 
                 fontsize=14, weight='bold')
    
    # Position plot - use center_pos as reference
    ax1.set_xlim(0, 10)
    ax1.set_ylim(center_pos - SINGLE_MOTOR_OSCILLATION * 1.5, 
                 center_pos + SINGLE_MOTOR_OSCILLATION * 1.5)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (°)')
    ax1.set_title('Motor Position')
    ax1.grid(True, alpha=0.3)
    
    target_line, = ax1.plot([], [], 'r-', label='Target', linewidth=2)
    actual_line, = ax1.plot([], [], 'b-', label='Actual', linewidth=2)
    ax1.legend(loc='upper right')
    
    # Error plot
    ax2.set_xlim(0, 10)
    ax2.set_ylim(-10, 10)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (°)')
    ax2.set_title('Position Error')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    error_line, = ax2.plot([], [], 'g-', linewidth=2)
    
    # Data buffers
    time_data = []
    target_data = []
    actual_data = []
    error_data = []
    
    info_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes,
                         fontsize=10, verticalalignment='top', family='monospace',
                         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    fig.text(0.5, 0.02, 'Controls: [SPACE] Start/Stop  |  [E] Emergency Stop', 
             ha='center', fontsize=10, family='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    gait_paused = True
    start_time = None
    
    def on_key_press(event):
        nonlocal start_time, center_pos
        if event.key == ' ':
            # Before resuming, get current motor position via PING
            if gait_paused:
                # About to resume - query current position
                ping_result = controller.send_ping()
                if ping_result:
                    center_pos = ping_result['position']
                    print(f"\n▶️  Resuming from position: {center_pos:.2f}°")
                    print(f"  New oscillation range: {center_pos - SINGLE_MOTOR_OSCILLATION:.1f}° to {center_pos + SINGLE_MOTOR_OSCILLATION:.1f}°")
                    # Update Y-axis limits
                    ax1.set_ylim(center_pos - SINGLE_MOTOR_OSCILLATION * 1.5, 
                                 center_pos + SINGLE_MOTOR_OSCILLATION * 1.5)
                else:
                    print("\n⚠️  Failed to get current position, using last known")
            
            toggle_gait_control()
            if not gait_paused:
                start_time = time.time()
                time_data.clear()
                target_data.clear()
                actual_data.clear()
                error_data.clear()
        elif event.key == 'e' or event.key == 'E':
            controller.send_emergency_stop()
    
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    def update_plot(frame):
        nonlocal start_time
        
        with control_lock:
            is_paused = gait_paused
        
        if is_paused or start_time is None:
            info_text.set_text('Status: PAUSED\nPress SPACE to start')
            return target_line, actual_line, error_line, info_text
        
        elapsed = time.time() - start_time
        phase = (elapsed % SINGLE_MOTOR_PERIOD) / SINGLE_MOTOR_PERIOD
        target_angle = center_pos + SINGLE_MOTOR_OSCILLATION * np.sin(2 * np.pi * phase)
        
        # Send command
        if CONTROL_MODE == ControlMode.MODE_SCURVE_PROFILE:
            controller.set_position_scurve(target_angle, int(1000 / UPDATE_RATE))
        else:
            controller.set_position_direct(target_angle)
        
        # Read feedback
        controller.read_feedback()
        current_pos = controller.current_position
        error = target_angle - current_pos
        
        # Update data
        time_data.append(elapsed)
        target_data.append(target_angle)
        actual_data.append(current_pos)
        error_data.append(error)
        
        # Limit data points
        max_points = int(10 * UPDATE_RATE)
        if len(time_data) > max_points:
            time_data.pop(0)
            target_data.pop(0)
            actual_data.pop(0)
            error_data.pop(0)
        
        # Update plot limits
        if time_data:
            ax1.set_xlim(max(0, elapsed - 10), elapsed + 0.5)
            ax2.set_xlim(max(0, elapsed - 10), elapsed + 0.5)
        
        target_line.set_data(time_data, target_data)
        actual_line.set_data(time_data, actual_data)
        error_line.set_data(time_data, error_data)
        
        stats = controller.get_stats()
        info_text.set_text(
            f'Status: RUNNING\n'
            f'Target: {target_angle:+6.1f}°\n'
            f'Actual: {current_pos:+6.1f}°\n'
            f'Error:  {error:+5.1f}°\n'
            f'TX: {stats["tx"]}  RX: {stats["rx"]}  Err: {stats["errors"]}'
        )
        
        return target_line, actual_line, error_line, info_text
    
    print("\n⏸️  Single motor control ready (PAUSED)")
    print("  Press [SPACE] in window to start")
    print("  Press [E] for emergency stop")
    print("  Close window to exit")
    print("="*70)
    
    anim = FuncAnimation(fig, update_plot, interval=int(1000/UPDATE_RATE),
                        blit=True, cache_frame_data=False)
    
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # Return to init position
        print("\n🏠 Returning to init position (-90°)...")
        controller.set_position_scurve(MOTOR_INIT_ANGLE, 2000)
        time.sleep(2.5)
        
        # Print statistics
        stats = controller.get_stats()
        print(f"\n📊 Communication Statistics:")
        print(f"    Motor {stats['motor_id']}: TX={stats['tx']}, RX={stats['rx']}, "
              f"Errors={stats['errors']}, Success={stats['success_rate']:.1f}%")
        
        # Disconnect
        print("\n🔌 Disconnecting motor...")
        controller.disconnect()
        
        print("\n✅ Single motor test completed")
        print("="*70)

# ============================================================================
# VISUALIZATION
# ============================================================================

def create_leg_subplot(ax, leg_id, leg_name):
    """Create subplot for one leg"""
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-150, 150)
    ax.set_ylim(-300, 50)
    ax.set_xlabel('Y (mm)', fontsize=9)
    ax.set_ylabel('X (mm)', fontsize=9)
    ax.set_title(f'{leg_id} - {leg_name}', fontsize=10, weight='bold')
    
    # Get motor positions
    P_A, P_B = get_motor_positions(leg_id)
    
    # Get motor IDs if registered
    motor_a_id = EXPECTED_MOTOR_IDS[leg_id]['A']
    motor_b_id = EXPECTED_MOTOR_IDS[leg_id]['B']
    
    # Draw motor positions
    ax.plot(P_A[0], P_A[1], 'o', color='darkblue', markersize=10, zorder=5)
    ax.plot(P_B[0], P_B[1], 'o', color='darkred', markersize=10, zorder=5)
    
    # Add motor index labels
    ax.text(P_A[0], P_A[1]+25, str(motor_a_id), fontsize=9, weight='bold', 
            color='white', ha='center', va='center', zorder=10,
            bbox=dict(boxstyle='circle', facecolor='darkblue', edgecolor='white', linewidth=1))
    ax.text(P_B[0], P_B[1]+25, str(motor_b_id), fontsize=9, weight='bold',
            color='white', ha='center', va='center', zorder=10,
            bbox=dict(boxstyle='circle', facecolor='darkred', edgecolor='white', linewidth=1))
    
    # Links
    link1 = ax.plot([], [], '-', color='darkblue', linewidth=3, zorder=4)[0]
    link2 = ax.plot([], [], '-', color='darkred', linewidth=3, zorder=4)[0]
    link3 = ax.plot([], [], '--', color='orange', linewidth=2, zorder=3)[0]
    link4 = ax.plot([], [], '--', color='cyan', linewidth=2, zorder=3)[0]
    link5 = ax.plot([], [], '-', color='green', linewidth=2.5, zorder=4)[0]
    
    # Joints
    joint_c = ax.plot([], [], 'ro', markersize=7, zorder=5)[0]
    joint_d = ax.plot([], [], 'bo', markersize=7, zorder=5)[0]
    joint_e = ax.plot([], [], 's', color='purple', markersize=6, zorder=5)[0]
    foot = ax.plot([], [], '*', color='green', markersize=15, zorder=6)[0]
    target = ax.plot([], [], 'x', color='red', markersize=10, markeredgewidth=2, zorder=6)[0]
    
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                        fontsize=7, verticalalignment='top', family='monospace',
                        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    return {
        'links': [link1, link2, link3, link4, link5],
        'joints': [joint_c, joint_d, joint_e],
        'foot': foot,
        'target': target,
        'info_text': info_text,
        'P_A': P_A,
        'P_B': P_B
    }

def update_leg_plot(leg_id, plot_elements):
    """Update one leg's plot"""
    with viz_lock:
        theta_A, theta_B = leg_states[leg_id]['target_angles']
        target_x, target_y = leg_states[leg_id]['target_pos']
        actual_A, actual_B = leg_states[leg_id]['actual_angles']
    
    P_A = plot_elements['P_A']
    P_B = plot_elements['P_B']
    
    # Calculate FK for target
    P_C, P_D, P_E, P_F = calculate_fk_positions(theta_A, theta_B, P_A, P_B)
    
    if P_C is not None:
        plot_elements['links'][0].set_data([P_A[0], P_C[0]], [P_A[1], P_C[1]])
        plot_elements['links'][1].set_data([P_B[0], P_D[0]], [P_B[1], P_D[1]])
        plot_elements['links'][2].set_data([P_C[0], P_E[0]], [P_C[1], P_E[1]])
        plot_elements['links'][3].set_data([P_D[0], P_E[0]], [P_D[1], P_E[1]])
        plot_elements['links'][4].set_data([P_E[0], P_F[0]], [P_E[1], P_F[1]])
        
        plot_elements['joints'][0].set_data([P_C[0]], [P_C[1]])
        plot_elements['joints'][1].set_data([P_D[0]], [P_D[1]])
        plot_elements['joints'][2].set_data([P_E[0]], [P_E[1]])
        plot_elements['foot'].set_data([P_F[0]], [P_F[1]])
        plot_elements['target'].set_data([target_x], [target_y])
        
        # Update info text
        motor_a_id = EXPECTED_MOTOR_IDS[leg_id]['A']
        motor_b_id = EXPECTED_MOTOR_IDS[leg_id]['B']
        
        error_A = np.rad2deg(theta_A) - np.rad2deg(actual_A)
        error_B = np.rad2deg(theta_B) - np.rad2deg(actual_B)
        
        plot_elements['info_text'].set_text(
            f'Motor {motor_a_id}: {np.rad2deg(theta_A):+6.1f}°\n'
            f'Motor {motor_b_id}: {np.rad2deg(theta_B):+6.1f}°\n'
            f'ΔA: {error_A:+5.1f}° ΔB: {error_B:+5.1f}°\n'
            f'Pos: ({P_F[0]:.0f},{P_F[1]:.0f})'
        )

def visualization_thread():
    """Thread function for real-time visualization"""
    global plot_running
    
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f'Quadruped Gait Control - Binary Protocol v1.2 - {GAIT_TYPE.upper()} Gait', 
                 fontsize=14, weight='bold')
    
    # Create 2x2 grid for 4 legs
    ax_FL = plt.subplot(2, 2, 1)
    ax_FR = plt.subplot(2, 2, 2)
    ax_RL = plt.subplot(2, 2, 3)
    ax_RR = plt.subplot(2, 2, 4)
    
    plot_elements = {
        'FR': create_leg_subplot(ax_FR, 'FR', 'Front Right'),
        'FL': create_leg_subplot(ax_FL, 'FL', 'Front Left'),
        'RR': create_leg_subplot(ax_RR, 'RR', 'Rear Right'),
        'RL': create_leg_subplot(ax_RL, 'RL', 'Rear Left')
    }
    
    # Add control instructions
    fig.text(0.5, 0.02, 'Controls: [SPACE] Start/Stop  |  [R] Reset Stats  |  [E] Emergency Stop', 
             ha='center', fontsize=10, family='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    def update_plot(frame):
        if not plot_running:
            return []
        
        artists = []
        for leg_id in ['FR', 'FL', 'RR', 'RL']:
            update_leg_plot(leg_id, plot_elements[leg_id])
            artists.extend(plot_elements[leg_id]['links'])
            artists.extend(plot_elements[leg_id]['joints'])
            artists.append(plot_elements[leg_id]['foot'])
            artists.append(plot_elements[leg_id]['target'])
            artists.append(plot_elements[leg_id]['info_text'])
        
        return artists
    
    def on_key_press(event):
        if event.key == ' ':
            toggle_gait_control()
        elif event.key == 'r' or event.key == 'R':
            reset_error_stats()
        elif event.key == 'e' or event.key == 'E':
            emergency_stop_all()
    
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    anim = FuncAnimation(fig, update_plot, interval=int(1000/PLOT_UPDATE_RATE), 
                        blit=True, cache_frame_data=False)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.show()
    
    plot_running = False

def start_visualization():
    """Start visualization in separate thread"""
    viz_thread = threading.Thread(target=visualization_thread, daemon=True)
    viz_thread.start()
    time.sleep(1.0)
    return viz_thread

# ============================================================================
# MAIN CONTROL LOOP
# ============================================================================

def main():
    global plot_running, gait_running, gait_paused, leg_states
    
    print("="*70)
    print("  BLEGS Quadruped Gait Control - Binary Protocol v1.2")
    print("="*70)
    
    if SINGLE_MOTOR_MODE:
        print("  ⚙️  MODE: SINGLE MOTOR TEST")
    else:
        print(f"  Number of Legs: 4 (FR, FL, RR, RL)")
        print(f"  Total Motors: 8")
    
    print(f"  Protocol: Binary v1.2")
    print(f"  Baud Rate: {BAUD_RATE}")
    
    if not SINGLE_MOTOR_MODE:
        print(f"  Gait Type: {GAIT_TYPE.upper()}")
    
    print(f"  Control Mode: {'S-Curve' if CONTROL_MODE == ControlMode.MODE_SCURVE_PROFILE else 'Direct'}")
    print(f"  Update Rate: {UPDATE_RATE} Hz")
    print(f"  Gear Ratio: {GEAR_RATIO}:1")
    
    if not SINGLE_MOTOR_MODE:
        print(f"  Motor Init Angle: {MOTOR_INIT_ANGLE}° (robot)")
        print(f"  Home Position: ({DEFAULT_STANCE_OFFSET_X}, {DEFAULT_STANCE_HEIGHT}) mm")
    else:
        print(f"  Oscillation: ±{SINGLE_MOTOR_OSCILLATION}°")
        print(f"  Period: {SINGLE_MOTOR_PERIOD}s")
    
    print(f"  Visualization: {'Enabled' if ENABLE_VISUALIZATION else 'Disabled'}")
    print("="*70)
    
    # --- Step 1: Motor Discovery ---
    discovered_motors = discover_motors()
    
    if not discovered_motors:
        print("\n❌ No motors discovered. Please check connections.")
        print("="*70)
        return
    
    # --- Single Motor Mode ---
    if SINGLE_MOTOR_MODE:
        # Use first discovered motor
        motor_id = list(discovered_motors.keys())[0]
        controller = discovered_motors[motor_id]
        
        print(f"\n✅ Using Motor ID {motor_id} for single motor test")
        
        if ENABLE_VISUALIZATION:
            run_single_motor_with_visualization(controller)
        else:
            run_single_motor_mode(controller)
        
        return
    
    # --- Step 2: Motor Registration ---
    all_motors_found = register_leg_motors(discovered_motors)
    
    if not all_motors_found:
        print("\n⚠️  Not all motors were found.")
        user_input = input("  Continue with available motors? (y/n): ").strip().lower()
        if user_input != 'y':
            print("\n  Aborting...")
            for controller in discovered_motors.values():
                controller.disconnect()
            return
    
    if not leg_motors:
        print("\n❌ No complete leg pairs found. Cannot continue.")
        for controller in discovered_motors.values():
            controller.disconnect()
        return
    
    # --- Step 3: Start All Motors ---
    start_all_motors()
    
    # --- Step 4: Start Visualization ---
    viz_thread = None
    if ENABLE_VISUALIZATION:
        print("\n📊 Starting real-time visualization...")
        viz_thread = start_visualization()
        print("  Visualization started!")
    
    # --- Step 5: Generate Trajectories ---
    print(f"\n🚶 Generating walking trajectories...")
    trajectories = {}
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        mirror_x = leg_id in ['FR', 'RR']
        trajectories[leg_id] = generate_elliptical_trajectory(
            num_steps=TRAJECTORY_STEPS,
            lift_height=GAIT_LIFT_HEIGHT,
            step_forward=GAIT_STEP_FORWARD,
            mirror_x=mirror_x
        )
    print(f"  Generated {TRAJECTORY_STEPS} waypoints per leg")
    
    # --- Step 6: Initialize Home Position ---
    print(f"\n🏠 Moving to home position...")
    print(f"  Note: Motors start at {MOTOR_INIT_ANGLE}° (robot angle)")
    prev_solutions = {}
    home_angles_all = {}
    
    for leg_id in leg_motors.keys():
        home_pos = np.array([DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT])
        P_A, P_B = get_motor_positions(leg_id)
        home_angles = calculate_ik_analytical(home_pos, P_A, P_B, elbow_C_down=True, elbow_D_down=False)
        
        if not np.isnan(home_angles).any():
            prev_solutions[leg_id] = home_angles
            home_angles_all[leg_id] = home_angles
            
            # Calculate movement from init angle
            init_angle_A = MOTOR_INIT_ANGLE
            init_angle_B = MOTOR_INIT_ANGLE
            target_angle_A = np.rad2deg(home_angles[0])
            target_angle_B = np.rad2deg(home_angles[1])
            
            delta_A = target_angle_A - init_angle_A
            delta_B = target_angle_B - init_angle_B
            
            with viz_lock:
                leg_states[leg_id]['target_angles'] = home_angles.tolist()
                leg_states[leg_id]['target_pos'] = home_pos.tolist()
            
            # Send S-Curve command for smooth home movement (3 seconds for safety)
            motor_a = leg_motors[leg_id]['A']
            motor_b = leg_motors[leg_id]['B']
            motor_a.set_position_scurve(target_angle_A, 3000)
            motor_b.set_position_scurve(target_angle_B, 3000)
            
            print(f"    {leg_id}: θA={target_angle_A:+.1f}° (Δ{delta_A:+.1f}°), θB={target_angle_B:+.1f}° (Δ{delta_B:+.1f}°)")
        else:
            print(f"    {leg_id}: IK FAILED for home position!")
    
    print(f"\n  ⏳ Moving to home position (3 seconds)...")
    time.sleep(3.5)  # Wait for home position (3s movement + margin)
    
    # --- Step 7: Main Gait Loop ---
    print(f"\n⏸️  Gait control ready (PAUSED)")
    if ENABLE_VISUALIZATION:
        print("  Press [SPACE] in visualization window to start")
    else:
        print("  Press [SPACE] to start/pause")
    print("  Press [E] for emergency stop")
    if not ENABLE_VISUALIZATION:
        print("  Press [Q] to quit")
    else:
        print("  Press Ctrl+C to exit")
    print("="*70)
    
    cycle_count = 0
    frame = 0
    last_status_time = 0
    running = True
    
    try:
        while running and plot_running:
            # Check keyboard input (for non-visualization mode)
            if not ENABLE_VISUALIZATION:
                key = check_keyboard_input()
                if key:
                    if key == ' ':
                        toggle_gait_control()
                    elif key == 'e':
                        emergency_stop_all()
                        with control_lock:
                            gait_paused = True
                        print("\n⚠️  Emergency stop activated!")
                    elif key == 'q':
                        print("\n⏹️  Quit requested...")
                        running = False
                        continue
            
            # Check if paused
            with control_lock:
                is_paused = gait_paused
            
            if is_paused:
                time.sleep(0.01)  # Short sleep when paused
                continue
            
            loop_start = time.perf_counter()
            
            # Update each leg
            for leg_id in leg_motors.keys():
                # Calculate current phase
                phase_offset = get_gait_phase_offset(leg_id, GAIT_TYPE)
                current_phase = (frame + int(phase_offset * TRAJECTORY_STEPS)) % TRAJECTORY_STEPS
                
                # Get target position
                if leg_id in trajectories:
                    px, py = trajectories[leg_id][current_phase]
                else:
                    px, py = DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT
                
                # Calculate IK with configuration search
                configs = [
                    (True, False),
                    (True, True),
                    (False, False),
                    (False, True)
                ]
                
                best_solution = None
                best_distance = float('inf')
                P_A, P_B = get_motor_positions(leg_id)
                
                for elbow_C, elbow_D in configs:
                    solution = calculate_ik_analytical(
                        np.array([px, py]),
                        P_A, P_B,
                        elbow_C_down=elbow_C,
                        elbow_D_down=elbow_D
                    )
                    
                    if not np.isnan(solution).any():
                        if leg_id not in prev_solutions or prev_solutions[leg_id] is None:
                            if elbow_C and not elbow_D:
                                best_solution = solution
                                break
                        else:
                            angle_diff = np.abs(solution - prev_solutions[leg_id])
                            angle_diff = np.minimum(angle_diff, 2*np.pi - angle_diff)
                            distance = np.sum(angle_diff)
                            
                            if distance < best_distance:
                                best_distance = distance
                                best_solution = solution
                
                if best_solution is not None and not np.isnan(best_solution).any():
                    prev_solutions[leg_id] = best_solution
                    theta_A, theta_B = best_solution
                    
                    # Update visualization state
                    with viz_lock:
                        leg_states[leg_id]['target_angles'] = best_solution.tolist()
                        leg_states[leg_id]['target_pos'] = [px, py]
                        leg_states[leg_id]['phase'] = current_phase
                    
                    # Send commands to motors
                    motor_a = leg_motors[leg_id]['A']
                    motor_b = leg_motors[leg_id]['B']
                    
                    motor_a.set_position_direct(np.rad2deg(theta_A))
                    motor_b.set_position_direct(np.rad2deg(theta_B))
                    
                    # Read feedback
                    feedback_a = motor_a.read_feedback()
                    feedback_b = motor_b.read_feedback()
                    
                    # Update actual angles
                    if feedback_a:
                        with viz_lock:
                            leg_states[leg_id]['actual_angles'][0] = np.deg2rad(feedback_a['position'])
                    
                    if feedback_b:
                        with viz_lock:
                            leg_states[leg_id]['actual_angles'][1] = np.deg2rad(feedback_b['position'])
            
            # Update frame counter
            frame = (frame + 1) % TRAJECTORY_STEPS
            
            # Print status every cycle (or periodically in non-viz mode)
            if frame == 0:
                cycle_count += 1
                if cycle_count % 5 == 0:
                    print(f"  🔄 Gait Cycle #{cycle_count}")
            
            # Print periodic status in terminal mode
            if not ENABLE_VISUALIZATION:
                current_time = time.time()
                if current_time - last_status_time >= 2.0:  # Every 2 seconds
                    last_status_time = current_time
                    print(f"  🔄 Gait running - Cycle #{cycle_count}, Frame {frame}/{TRAJECTORY_STEPS}")
            
            # Timing control
            elapsed = time.perf_counter() - loop_start
            sleep_time = max(0, (1.0 / UPDATE_RATE) - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n⏹️  Gait control stopped by user")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        traceback.print_exc()
    
    finally:
        # Stop visualization
        plot_running = False
        
        # Return to init position (-90°)
        print("\n🏠 Returning to init position (-90°)...")
        try:
            for leg_id in leg_motors.keys():
                motor_a = leg_motors[leg_id]['A']
                motor_b = leg_motors[leg_id]['B']
                motor_a.set_position_scurve(MOTOR_INIT_ANGLE, 3000)
                motor_b.set_position_scurve(MOTOR_INIT_ANGLE, 3000)
            time.sleep(3.5)
        except:
            pass
        
        # Print statistics
        print("\n📊 Communication Statistics:")
        for motor_id, controller in motor_registry.items():
            stats = controller.get_stats()
            print(f"    Motor {motor_id}: TX={stats['tx']}, RX={stats['rx']}, "
                  f"Errors={stats['errors']}, Success={stats['success_rate']:.1f}%")
        
        # Disconnect all motors
        print("\n🔌 Disconnecting motors...")
        for controller in motor_registry.values():
            controller.disconnect()
        
        print("\n✅ Quadruped gait control terminated successfully")
        print("="*70)
        
        if viz_thread and ENABLE_VISUALIZATION:
            print("\n📊 Closing visualization...")
            time.sleep(1.0)

if __name__ == "__main__":
    main()
