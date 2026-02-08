"""
Quadruped Gait Control - Binary Protocol v1.2 (No EF Link Version)
Author: M-TRCH
Date: January 7, 2026

This script controls a quadruped robot (4 legs, 8 motors) WITHOUT EF link.
Joint E becomes the foot tip (end-effector).

Features:
- Automatic motor discovery and registration via COM port scanning
- Motor ID detection using PING command
- IK calculation WITHOUT EF link (E is the foot)
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
import csv
from datetime import datetime

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
# ROBOT CONFIGURATION (No EF Link)
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

# NOTE: No L_EF link - Joint E is the foot!

# --- Motor Configuration ---
GEAR_RATIO = 8.0  # Motor shaft to output shaft gear ratio
MOTOR_INIT_ANGLE = -90.0  # Initial motor angle (degrees)

# --- Expected Motor IDs for Each Leg ---
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
GAIT_LIFT_HEIGHT = 15.0    # mm
GAIT_STEP_FORWARD = 50.0   # mm

# Smooth Trot Parameters
SMOOTH_TROT_LIFT_HEIGHT = 15.0  # mm

# ============================================================================
# CONTROL PARAMETERS
# ============================================================================

CONTROL_MODE = ControlMode.MODE_DIRECT_POSITION
UPDATE_RATE = 50  # Hz (20ms per update)

TRAJECTORY_STEPS = 20  # Number of steps in one gait cycle

# ============================================================================
# GAIT CONFIGURATIONS
# ============================================================================
# Phase offset: when swing phase starts (0-1)
# Duty factor: fraction of cycle in stance phase (higher = more stable)
# Leg mapping: FL=Front Left, FR=Front Right, RL=Rear Left, RR=Rear Right

GAIT_CONFIGS = {
    'crawl': {
        'name': 'CRAWL',
        'description': 'One leg at a time, very stable',
        'emoji': '🐌',
        'phases': {'FL': 0.0, 'RR': 0.25, 'FR': 0.5, 'RL': 0.75},
        'duty_factor': 0.75,  # 75% stance, 25% swing
        'num_steps': 40,
        'lift_height': 20.0,
        'step_length': 30.0,
        'speed_desc': '25mm/s'
    },
    'trot': {
        'name': 'TROT',
        'description': 'Diagonal pairs, fast & efficient',
        'emoji': '⚡',
        'phases': {'FL': 0.5, 'RR': 0.5, 'FR': 0.0, 'RL': 0.0},
        'duty_factor': 0.5,  # 50% stance, 50% swing
        'num_steps': 20,
        'lift_height': 15.0,
        'step_length': 50.0,
        'speed_desc': '100mm/s'
    },
    'trot_opt': {
        'name': 'TROT OPT',
        'description': 'Optimized trot, high lift, fast swing',
        'emoji': '🚀',
        'phases': {'FL': 0.5, 'RR': 0.5, 'FR': 0.0, 'RL': 0.0},
        'duty_factor': 0.35,  # 35% stance, 65% swing
        'num_steps': 30,      # More steps for smooth high-lift
        'lift_height': 25.0,  # Higher lift
        'step_length': 55.0,  # Slightly longer stride
        'speed_desc': '130mm/s',
        'smooth_factor': 0.6  # Smooth high-speed motion
    },
    'pace': {
        'name': 'PACE',
        'description': 'Same-side legs together',
        'emoji': '🦒',
        'phases': {'FL': 0.0, 'RL': 0.0, 'FR': 0.5, 'RR': 0.5},
        'duty_factor': 0.5,
        'num_steps': 25,
        'lift_height': 15.0,
        'step_length': 45.0,
        'speed_desc': '80mm/s'
    },
    'bound': {
        'name': 'BOUND',
        'description': 'Front legs push, hind legs land (backward)',
        'emoji': '🐇',
        'phases': {'FL': 0.0, 'FR': 0.0, 'RL': 0.5, 'RR': 0.5},  # Front first = BACKWARD
        'duty_factor': 0.50,  # 50% stance
        'num_steps': 25,      # Cycle time: 500ms @ 50Hz = 25 steps (FASTER!)
        'lift_height': 30.0,  # Step height: moderate
        'step_length': 50.0,  # Step length: full stride
        'speed_desc': '100mm/s BACKWARD',
        'smooth_factor': 0.8  # Good smoothing for fast motion
    },
    'pronk': {
        'name': 'PRONK',
        'description': 'All legs together (hopping)',
        'emoji': '🦘',
        'phases': {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0},
        'duty_factor': 0.45,  # 45% stance - smoother landing
        'num_steps': 50,      # Many steps for smooth hop
        'lift_height': 20.0,  # Moderate lift
        'step_length': 30.0,
        'speed_desc': '60mm/s',
        'smooth_factor': 0.9  # Maximum smoothing
    },
    'gallop': {
        'name': 'GALLOP',
        'description': 'Asymmetric running gait',
        'emoji': '🐎',
        'phases': {'FL': 0.0, 'FR': 0.1, 'RL': 0.5, 'RR': 0.6},
        'duty_factor': 0.45,  # 45% stance - smoother
        'num_steps': 40,      # More steps for fluidity
        'lift_height': 18.0,  # Lower lift
        'step_length': 55.0,
        'speed_desc': '120mm/s',
        'smooth_factor': 0.7  # Good smoothing
    },
    'stand': {
        'name': 'STAND',
        'description': 'Static pose testing',
        'emoji': '🧍',
        'phases': {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0},
        'duty_factor': 1.0,  # Always in stance
        'num_steps': 1,
        'lift_height': 0.0,
        'step_length': 0.0,
        'speed_desc': '0mm/s'
    }
}

# Gait Types
DEFAULT_GAIT_TYPE = 'trot'
current_gait_type = DEFAULT_GAIT_TYPE

# Smooth Trot Parameters
SMOOTH_TROT_STEPS = 30
SMOOTH_TROT_STANCE_RATIO = 0.65

# --- Single Motor Mode ---
SINGLE_MOTOR_MODE = False
SINGLE_MOTOR_OSCILLATION = 30.0
SINGLE_MOTOR_PERIOD = 0.6
 
# --- Visualization Parameters ---
ENABLE_VISUALIZATION = False
PLOT_UPDATE_RATE = 10  # Hz

# --- Simulation Mode ---
SIMULATION_MODE = False  # Set to True to run without real motors

# --- Data Logging Parameters ---
ENABLE_DATA_LOGGING = True  # Set to True to enable motor feedback logging
LOG_DIRECTORY = "logs"  # Directory to store log files

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
MOTOR_INIT_ANGLE_RAD = np.deg2rad(MOTOR_INIT_ANGLE)
leg_states = {
    'FR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'red'},
    'FL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'blue'},
    'RR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'orange'},
    'RL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'green'}
}

# Motor registry
motor_registry = {}
leg_motors = {}

# Error tracking
error_stats = {}

# Single motor mode state
single_motor_controller = None
single_motor_start_time = None

# Data logging
log_file = None
log_writer = None
log_lock = threading.Lock()
log_start_time = 0.0
log_record_count = 0

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
        """Send PING command and wait for response"""
        if not self.is_connected:
            return None
        
        try:
            self.serial.reset_input_buffer()
            
            packet = build_packet(PacketType.PKT_CMD_PING)
            self.serial.write(packet)
            self.stats_tx_count += 1
            
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
    
    def _read_packet(self):
        """Read and parse incoming packet"""
        try:
            header = self.serial.read(2)
            if len(header) != 2 or header[0] != HEADER_1 or header[1] != HEADER_2:
                return None
            
            pkt_type = self.serial.read(1)[0]
            payload_len = self.serial.read(1)[0]
            
            payload = self.serial.read(payload_len) if payload_len > 0 else b''
            crc_bytes = self.serial.read(2)
            
            if len(crc_bytes) != 2:
                return None
            
            received_crc = struct.unpack('<H', crc_bytes)[0]
            calculated_crc = calculate_crc16(bytes([pkt_type, payload_len]) + payload)
            
            if received_crc != calculated_crc:
                return None
            
            self.stats_rx_count += 1
            return pkt_type, payload
            
        except Exception as e:
            return None
    
    def start_motor(self) -> bool:
        """Start motor using PING command"""
        for retry in range(PING_RETRIES):
            result = self.send_ping()
            if result:
                print(f"  ✅ Motor ID {result['motor_id']} started on {self.port}")
                time.sleep(0.1)
                return True
            time.sleep(0.1)
        return False
    
    def set_position_direct(self, angle_deg: float) -> bool:
        """Set target position using direct mode"""
        if not self.is_connected:
            return False
        
        try:
            motor_angle = angle_deg * GEAR_RATIO
            
            with self.lock:
                mode = bytes([ControlMode.MODE_DIRECT_POSITION])
                target_pos = struct.pack('<i', int(motor_angle * 100))
                payload = mode + target_pos
                
                packet = build_packet(PacketType.PKT_CMD_SET_GOAL, payload)
                self.serial.write(packet)
                self.serial.flush()
                self.stats_tx_count += 1
                self.current_setpoint = angle_deg
            
            return True
            
        except Exception as e:
            with error_lock:
                motor_id = self.motor_id if self.motor_id else "Unknown"
                error_stats[motor_id] = error_stats.get(motor_id, 0) + 1
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
                self.serial.flush()
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
            self.serial.flush()
            print(f"  ⚠️  Emergency stop sent to Motor ID {self.motor_id}")
            return True
        except:
            return False
    
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
                        self.motor_id = motor_id
                        self.current_position = (position_raw / 100.0) / GEAR_RATIO
                        self.current_current = current_raw
                        self.current_flags = status_flags
                    
                    feedback_data = {
                        'motor_id': motor_id,
                        'position': self.current_position,
                        'current': current_raw,
                        'flags': status_flags,
                        'is_moving': bool(status_flags & StatusFlags.STATUS_MOVING),
                        'at_goal': bool(status_flags & StatusFlags.STATUS_AT_GOAL),
                        'error': bool(status_flags & StatusFlags.STATUS_ERROR),
                        'emergency_stopped': bool(status_flags & StatusFlags.STATUS_EMERGENCY_STOPPED)
                    }
                    
                    return feedback_data
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

def discover_motors() -> dict:
    """Scan COM ports and discover motors"""
    print("\n" + "="*70)
    print("  🔍 MOTOR DISCOVERY")
    print("="*70)
    
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append({'device': port.device, 'description': port.description})
    
    if not ports:
        print("  ❌ No COM ports found!")
        print("\n  💡 Would you like to run in SIMULATION mode? (no real motors)")
        response = input("  Run simulation? [y/N]: ").strip().lower()
        if response == 'y':
            global SIMULATION_MODE, ENABLE_VISUALIZATION
            SIMULATION_MODE = True
            ENABLE_VISUALIZATION = True
            print("  ✅ Simulation mode enabled")
        return {}
    
    print(f"\n  Found {len(ports)} COM port(s):")
    for port in ports:
        print(f"    • {port['device']}: {port['description']}")
    
    discovered = {}
    
    print(f"\n  📡 Scanning for motors (timeout={SERIAL_TIMEOUT*1000:.0f}ms, retries={PING_RETRIES})...")
    
    for port_info in ports:
        port = port_info['device']
        print(f"\n    Checking {port}...")
        
        controller = BinaryMotorController(port)
        
        if not controller.connect(timeout=SERIAL_TIMEOUT):
            print(f"      ❌ Failed to open {port}")
            continue
        
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
            
            discovered[motor_id] = controller
        else:
            print(f"      ⚠️  No response (no motor on this port)")
            controller.disconnect()
    
    print(f"\n  📊 Discovery complete: {len(discovered)} motor(s) found")
    return discovered

def register_leg_motors(discovered_motors: dict) -> bool:
    """Register discovered motors to leg assignments"""
    global leg_motors, motor_registry
    
    print("\n" + "="*70)
    print("  🔗 MOTOR REGISTRATION")
    print("="*70)
    
    motor_registry = discovered_motors
    leg_motors = {}
    
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
        controller.set_timeout(FAST_TIMEOUT)
        
        result = controller.send_ping()
        if result:
            success_count += 1
        else:
            print(f"  ❌ Failed to start Motor ID {motor_id}")
    
    print(f"\n  📊 Started {success_count}/{total_count} motors")
    
    if success_count > 0:
        print("  ⏳ Waiting for motor initialization...")
        time.sleep(2.0)
    
    return success_count == total_count

# ============================================================================
# KINEMATICS FUNCTIONS (No EF Link)
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

def calculate_ik_no_ef(P_E_target, P_A, P_B, elbow_C_down=True, elbow_D_down=True):
    """
    Calculate Inverse Kinematics WITHOUT EF link (E is the foot)
    
    Args:
        P_E_target: Target foot position [x, y] in leg frame
        P_A: Motor A position [x, y]
        P_B: Motor B position [x, y]
        elbow_C_down: True to choose lower elbow position for joint C
        elbow_D_down: True to choose lower elbow position for joint D
        
    Returns:
        numpy array [theta_A, theta_B] in radians, or [nan, nan] if no solution
    """
    # Find joint C position
    P_C = solve_circle_intersection(P_A, L_AC, P_E_target, L_CE, elbow_C_down)
    
    if np.isnan(P_C).any():
        return np.array([np.nan, np.nan])
    
    # Find joint D position
    P_D = solve_circle_intersection(P_B, L_BD, P_E_target, L_DE, elbow_D_down)
    
    if np.isnan(P_D).any():
        return np.array([np.nan, np.nan])
    
    # Calculate motor angles
    V_AC = P_C - P_A
    V_BD = P_D - P_B
    
    theta_A = np.arctan2(V_AC[1], V_AC[0])
    theta_B = np.arctan2(V_BD[1], V_BD[0])
    
    return np.array([theta_A, theta_B])

def calculate_fk_no_ef(theta_A, theta_B, P_A, P_B):
    """
    Calculate forward kinematics positions WITHOUT EF link
    
    Returns:
        P_C, P_D, P_E (foot position)
    """
    # Calculate P_C and P_D from motor angles
    P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
    P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])
    
    # Calculate P_E using circle intersection between C and D
    V_CD = P_D - P_C
    d = np.linalg.norm(V_CD)
    
    if d > 0 and d <= (L_CE + L_DE) and d >= abs(L_CE - L_DE):
        a = (L_CE**2 - L_DE**2 + d**2) / (2 * d)
        h_squared = L_CE**2 - a**2
        
        if h_squared >= 0:
            h = np.sqrt(h_squared)
            v_d = V_CD / d
            v_perp = np.array([-v_d[1], v_d[0]])
            
            # Try both configurations and choose the one with lower y (foot down)
            P_E1 = P_C + a * v_d + h * v_perp
            P_E2 = P_C + a * v_d - h * v_perp
            
            # Choose configuration with lower foot position
            if P_E1[1] < P_E2[1]:
                P_E = P_E1
            else:
                P_E = P_E2
            
            return P_C, P_D, P_E
    
    return None, None, None

# ============================================================================
# TRAJECTORY GENERATION
# ============================================================================

def smooth_step(t, smooth_factor=0.5):
    """
    Apply smoothstep easing function for gentle acceleration/deceleration
    t: input value 0-1
    smooth_factor: 0 = linear, 1 = maximum smoothing (quintic)
    """
    if smooth_factor <= 0:
        return t
    elif smooth_factor >= 1:
        # Quintic smoothstep (very smooth)
        return t * t * t * (t * (t * 6 - 15) + 10)
    else:
        # Blend between linear and cubic smoothstep
        cubic = t * t * (3 - 2 * t)
        return t * (1 - smooth_factor) + cubic * smooth_factor


def generate_elliptical_trajectory(step_forward, lift_height, num_steps, stance_ratio=0.5, 
                                   home_x=0.0, home_y=DEFAULT_STANCE_HEIGHT, 
                                   reverse=False, mirror_x=False, smooth_factor=0.5):
    """
    Generate smooth elliptical foot trajectory with easing
    
    Args:
        step_forward: Forward step length (mm)
        lift_height: Maximum foot lift height (mm)
        num_steps: Number of trajectory waypoints
        stance_ratio: Fraction of cycle in stance phase (duty factor)
        home_x, home_y: Home position
        reverse: True for backward motion
        mirror_x: True for right-side legs
        smooth_factor: 0-1, higher = smoother transitions
    """
    trajectory = []
    
    swing_steps = max(1, int(num_steps * (1.0 - stance_ratio)))
    stance_steps = max(1, num_steps - swing_steps)
    
    direction = -1 if reverse else 1
    
    for i in range(num_steps):
        if i < swing_steps:
            # Swing phase - apply smoothstep for gentle takeoff/landing
            raw_progress = i / swing_steps
            phase_progress = smooth_step(raw_progress, smooth_factor)
            t = np.pi + np.pi * phase_progress  # π to 2π
            
            # Smooth height profile using raised cosine (gentler than sin)
            height_progress = smooth_step(raw_progress, smooth_factor)
            # Bell curve for height: peaks in middle, gentle at ends
            height_factor = np.sin(np.pi * height_progress)
            # Apply additional smoothing to height
            height_factor = height_factor ** (1 + smooth_factor * 0.5)
            
            py = home_y + lift_height * height_factor
        else:
            # Stance phase - smooth ground contact
            stance_index = i - swing_steps
            raw_progress = stance_index / stance_steps
            phase_progress = smooth_step(raw_progress, smooth_factor * 0.3)  # Less smoothing on ground
            t = np.pi * phase_progress  # 0 to π
            
            py = home_y  # Flat on ground
        
        # FIXED: Removed negative sign for correct forward motion
        px = direction * (step_forward * np.cos(t))
        if mirror_x:
            px = -px
        
        trajectory.append((px + home_x, py))
    
    return trajectory

def get_gait_phase_offset(leg_id):
    """Get phase offset for each leg based on current gait type"""
    global current_gait_type
    
    if current_gait_type in GAIT_CONFIGS:
        return GAIT_CONFIGS[current_gait_type]['phases'].get(leg_id, 0.0)
    
    # Fallback to trot if unknown gait type
    return GAIT_CONFIGS['trot']['phases'].get(leg_id, 0.0)


def get_gait_duty_factor():
    """Get duty factor for current gait type"""
    global current_gait_type
    
    if current_gait_type in GAIT_CONFIGS:
        return GAIT_CONFIGS[current_gait_type]['duty_factor']
    return 0.5  # Default 50% duty factor


def get_gait_parameters():
    """Get all gait parameters for current gait type"""
    global current_gait_type
    
    if current_gait_type in GAIT_CONFIGS:
        config = GAIT_CONFIGS[current_gait_type]
        return {
            'num_steps': config['num_steps'],
            'lift_height': config['lift_height'],
            'step_length': config['step_length'],
            'duty_factor': config['duty_factor'],
            'smooth_factor': config.get('smooth_factor', 0.5)  # Default 0.5
        }
    # Fallback defaults
    return {
        'num_steps': TRAJECTORY_STEPS,
        'lift_height': GAIT_LIFT_HEIGHT,
        'step_length': GAIT_STEP_FORWARD,
        'duty_factor': 0.5,
        'smooth_factor': 0.5
    }

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

def emergency_stop_all():
    """Send emergency stop to all motors"""
    print("\n⚠️  EMERGENCY STOP - ALL MOTORS!")
    for motor_id, controller in motor_registry.items():
        controller.send_emergency_stop()

def initialize_data_logging():
    """Initialize CSV log file for motor feedback data"""
    global log_file, log_writer
    
    if not ENABLE_DATA_LOGGING:
        return
    
    if not os.path.exists(LOG_DIRECTORY):
        os.makedirs(LOG_DIRECTORY)
        print(f"  📁 Created log directory: {LOG_DIRECTORY}")
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(LOG_DIRECTORY, f"motor_feedback_no_ef_{timestamp}.csv")
    
    try:
        log_file = open(log_filename, 'w', newline='', buffering=1)  # Line buffering
        log_writer = csv.writer(log_file)
        
        # Write header
        log_writer.writerow([
            'timestamp',
            'elapsed_ms',
            'motor_id',
            'setpoint_deg',
            'position_deg',
            'error_deg',
            'current_mA',
            'flags_hex',
            'is_moving',
            'at_goal',
            'error_flag',
            'emergency_stopped'
        ])
        log_file.flush()  # Ensure header is written immediately
        
        print(f"  📝 Data logging initialized: {log_filename}")
        print(f"  ℹ️  Logging enabled = {ENABLE_DATA_LOGGING}")
        print(f"  ℹ️  Log writer ready = {log_writer is not None}")
        return True
        
    except Exception as e:
        print(f"  ❌ Failed to initialize data logging: {e}")
        return False

def log_motor_feedback(feedback_data, setpoint, motor_id=None):
    """Log motor feedback data to CSV file
    
    Args:
        feedback_data: Feedback dict or None if feedback failed
        setpoint: Commanded position in degrees
        motor_id: Motor ID (required if feedback_data is None)
    """
    global log_writer, log_file, log_record_count
    
    if not ENABLE_DATA_LOGGING or log_writer is None:
        return
    
    try:
        with log_lock:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            elapsed_ms = int((time.time() - log_start_time) * 1000)
            
            if feedback_data is not None:
                # Normal case: feedback received
                error_deg = setpoint - feedback_data['position']
                
                log_writer.writerow([
                    timestamp,
                    elapsed_ms,
                    feedback_data['motor_id'],
                    f"{setpoint:.2f}",
                    f"{feedback_data['position']:.2f}",
                    f"{error_deg:.2f}",
                    feedback_data['current'],
                    f"0x{feedback_data['flags']:02X}",
                    1 if feedback_data.get('is_moving', False) else 0,
                    1 if feedback_data.get('at_goal', False) else 0,
                    1 if feedback_data.get('error', False) else 0,
                    1 if feedback_data.get('emergency_stopped', False) else 0
                ])
                log_record_count += 1
            else:
                # Feedback missing: log setpoint with null feedback values
                if motor_id is None:
                    return  # Can't log without motor ID
                
                log_writer.writerow([
                    timestamp,
                    elapsed_ms,
                    motor_id,
                    f"{setpoint:.2f}",
                    "NaN",  # position unavailable
                    "NaN",  # error unavailable
                    "0",    # current unavailable
                    "0xFF", # flags invalid
                    0, 0, 1, 0  # Mark as error condition
                ])
                log_record_count += 1
            
            # Flush every 50 records to ensure data is written
            if hasattr(log_file, 'flush') and (log_record_count % 50) == 0:
                log_file.flush()
            
    except Exception as e:
        print(f"⚠️ Logging error: {e}")  # Debug: print errors

def close_data_logging():
    """Close log file and print summary"""
    global log_file, log_writer, log_record_count
    
    if not ENABLE_DATA_LOGGING or log_file is None:
        return
    
    try:
        with log_lock:
            if log_file:
                log_file.flush()  # Final flush
                log_file.close()
                print(f"  📝 Data log file closed ({log_record_count} records written)")
                log_file = None
                log_writer = None
    except Exception as e:
        print(f"  ⚠️ Error closing log: {e}")

def select_gait_mode():
    """Allow user to select gait mode at startup"""
    global current_gait_type
    
    print("\n" + "="*70)
    print("  🚶 SELECT GAIT MODE")
    print("="*70)
    print("  Available gait modes:")
    print("  ┌────────────────────────────────────────────────────────────────┐")
    print("  │  [1] TROT     - Diagonal pairs, fast & efficient   ⚡ 100mm/s │")
    print("  │  [2] TROT OPT - High lift, 65% swing optimized     🚀 130mm/s │")
    print("  │  [3] CRAWL    - One leg at a time, very stable     🐌  25mm/s │")
    print("  │  [4] PACE     - Same-side legs together            🦒  80mm/s │")
    print("  │  [5] BOUND    - Front/hind legs together           🐇 100mm/s │")
    print("  │  [6] PRONK    - All legs together (hopping)        🦘  60mm/s │")
    print("  │  [7] GALLOP   - Asymmetric running gait            🐎 120mm/s │")
    print("  │  [8] STAND    - Static pose testing                🧍   0mm/s │")
    print("  └────────────────────────────────────────────────────────────────┘")
    print("="*70)
    
    gait_map = {
        '1': 'trot',
        '2': 'trot_opt',
        '3': 'crawl',
        '4': 'pace',
        '5': 'bound',
        '6': 'pronk',
        '7': 'gallop',
        '8': 'stand'
    }
    
    while True:
        choice = input("  Select mode [1-8] (default: 1): ").strip()
        
        if choice == '':
            choice = '1'
        
        if choice in gait_map:
            current_gait_type = gait_map[choice]
            config = GAIT_CONFIGS[current_gait_type]
            print(f"  ✅ Selected: {config['name']} {config['emoji']} - {config['description']}")
            break
        else:
            print("  ❌ Invalid choice. Please select 1-8.")
    
    return current_gait_type

def change_gait_mode(new_mode):
    """Change gait mode during runtime"""
    global current_gait_type
    
    if new_mode in GAIT_CONFIGS:
        old_mode = current_gait_type
        current_gait_type = new_mode
        
        old_config = GAIT_CONFIGS.get(old_mode, {'name': old_mode.upper(), 'emoji': ''})
        new_config = GAIT_CONFIGS[new_mode]
        
        old_name = f"{old_config['name']} {old_config['emoji']}"
        new_name = f"{new_config['name']} {new_config['emoji']}"
        
        print(f"\n🔄 Gait mode changed: {old_name} → {new_name}")
        print(f"   Phase offsets: FL={new_config['phases']['FL']:.2f}, FR={new_config['phases']['FR']:.2f}, "
              f"RL={new_config['phases']['RL']:.2f}, RR={new_config['phases']['RR']:.2f}")
        print(f"   Duty factor: {new_config['duty_factor']:.0%}")
        return True
    return False

def reset_error_stats():
    """Reset all error statistics"""
    global error_stats
    with error_lock:
        error_stats.clear()
    print("\n🔄 Error statistics reset!")

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
    ax.set_title(f'{leg_id} - {leg_name} (No EF)', fontsize=10, weight='bold')
    
    P_A, P_B = get_motor_positions(leg_id)
    
    # Draw motor positions
    ax.plot(P_A[0], P_A[1], 'o', color='darkblue', markersize=10, label='Motor A', zorder=5)
    ax.plot(P_B[0], P_B[1], 'o', color='darkred', markersize=10, label='Motor B', zorder=5)
    
    color = leg_states[leg_id]['color']
    
    # Links
    link1 = ax.plot([], [], '-', color='darkblue', linewidth=3, label='AC', zorder=4)[0]
    link2 = ax.plot([], [], '-', color='darkred', linewidth=3, label='BD', zorder=4)[0]
    link3 = ax.plot([], [], '--', color='orange', linewidth=2, label='CE', zorder=3)[0]
    link4 = ax.plot([], [], '--', color='cyan', linewidth=2, label='DE', zorder=3)[0]
    
    # Joints
    joint_c = ax.plot([], [], 'ro', markersize=7, zorder=5)[0]
    joint_d = ax.plot([], [], 'bo', markersize=7, zorder=5)[0]
    joint_e_foot = ax.plot([], [], '*', color='green', markersize=15, label='Foot (E)', zorder=6)[0]
    
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                        fontsize=7, verticalalignment='top', family='monospace',
                        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    ax.legend(loc='lower right', fontsize=7, framealpha=0.9)
    
    return {
        'links': [link1, link2, link3, link4],
        'joints': [joint_c, joint_d, joint_e_foot],
        'info_text': info_text
    }

def update_leg_plot(leg_id, plot_elements):
    """Update one leg's plot"""
    with viz_lock:
        theta_A, theta_B = leg_states[leg_id]['target_angles']
        target_x, target_y = leg_states[leg_id]['target_pos']
    
    P_A, P_B = get_motor_positions(leg_id)
    
    # Calculate FK
    P_C, P_D, P_E = calculate_fk_no_ef(theta_A, theta_B, P_A, P_B)
    
    if P_C is not None:
        # Update links
        plot_elements['links'][0].set_data([P_A[0], P_C[0]], [P_A[1], P_C[1]])
        plot_elements['links'][1].set_data([P_B[0], P_D[0]], [P_B[1], P_D[1]])
        plot_elements['links'][2].set_data([P_C[0], P_E[0]], [P_C[1], P_E[1]])
        plot_elements['links'][3].set_data([P_D[0], P_E[0]], [P_D[1], P_E[1]])
        
        # Update joints
        plot_elements['joints'][0].set_data([P_C[0]], [P_C[1]])
        plot_elements['joints'][1].set_data([P_D[0]], [P_D[1]])
        plot_elements['joints'][2].set_data([P_E[0]], [P_E[1]])
        
        # Update info text
        mode_text = "SIMULATION" if SIMULATION_MODE else "REAL"
        plot_elements['info_text'].set_text(
            f'Mode: {mode_text}\n'
            f'Motor A: {np.rad2deg(theta_A):+6.1f}°\n'
            f'Motor B: {np.rad2deg(theta_B):+6.1f}°\n'
            f'Foot (E): ({P_E[0]:.0f},{P_E[1]:.0f})'
        )

def visualization_thread():
    """Thread function for real-time visualization"""
    global plot_running
    
    fig = plt.figure(figsize=(14, 10))
    mode_text = "SIMULATION MODE" if SIMULATION_MODE else "REAL MODE"
    fig.suptitle(f'Quadruped Gait Control - {current_gait_type.upper()} ({mode_text})', 
                 fontsize=14, weight='bold')
    
    # Create 2x2 grid for 4 legs
    ax_FR = plt.subplot(2, 2, 1)
    ax_FL = plt.subplot(2, 2, 2)
    ax_RR = plt.subplot(2, 2, 3)
    ax_RL = plt.subplot(2, 2, 4)
    
    # Create plot elements for each leg
    plot_elements = {
        'FR': create_leg_subplot(ax_FR, 'FR', 'Front Right'),
        'FL': create_leg_subplot(ax_FL, 'FL', 'Front Left'),
        'RR': create_leg_subplot(ax_RR, 'RR', 'Rear Right'),
        'RL': create_leg_subplot(ax_RL, 'RL', 'Rear Left')
    }
    
    # Add control instructions
    fig.text(0.5, 0.02, 'Controls: [SPACE] Start/Stop | [1/T] Trot | [2/O] TrotOpt | [3/C] Crawl | [4/P] Pace | [5/B] Bound | [6/K] Pronk | [7/G] Gallop | [8/S] Stand | [R] Reset | [E] E-Stop', 
             ha='center', fontsize=7, family='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    def update_plot(frame):
        if not plot_running:
            return []
        
        artists = []
        for leg_id in ['FR', 'FL', 'RR', 'RL']:
            update_leg_plot(leg_id, plot_elements[leg_id])
            artists.extend(plot_elements[leg_id]['links'])
            artists.extend(plot_elements[leg_id]['joints'])
            artists.append(plot_elements[leg_id]['info_text'])
        
        return artists
    
    def on_key_press(event):
        if event.key == ' ':
            toggle_gait_control()
        elif event.key == 'r' or event.key == 'R':
            reset_error_stats()
        elif event.key == 'e' or event.key == 'E':
            emergency_stop_all()
        elif event.key == '1' or event.key == 't' or event.key == 'T':
            change_gait_mode('trot')
        elif event.key == '2' or event.key == 'o' or event.key == 'O':  # O for Optimized
            change_gait_mode('trot_opt')
        elif event.key == '3' or event.key == 'c' or event.key == 'C':
            change_gait_mode('crawl')
        elif event.key == '4' or event.key == 'p' or event.key == 'P':
            change_gait_mode('pace')
        elif event.key == '5' or event.key == 'b' or event.key == 'B':
            change_gait_mode('bound')
        elif event.key == '6' or event.key == 'k' or event.key == 'K':  # K for pronK/hop
            change_gait_mode('pronk')
        elif event.key == '7' or event.key == 'g' or event.key == 'G':
            change_gait_mode('gallop')
        elif event.key == '8' or event.key == 's' or event.key == 'S':
            change_gait_mode('stand')
    
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    anim = FuncAnimation(fig, update_plot, interval=int(1000/PLOT_UPDATE_RATE), 
                        blit=True, cache_frame_data=False)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.98])
    plt.show()
    
    plot_running = False

# ============================================================================
# GAIT CONTROL LOOP
# ============================================================================

def gait_control_loop(trajectories, prev_solutions):
    """Main gait control loop"""
    global gait_running, gait_paused, leg_states, log_start_time, current_gait_type
    
    frame = 0
    cycle_count = 0
    last_gait_type = current_gait_type  # Track gait mode changes
    
    while gait_running:
        # Regenerate trajectories if gait mode changed
        if current_gait_type != last_gait_type:
            gait_params = get_gait_parameters()
            gait_config = GAIT_CONFIGS.get(current_gait_type, GAIT_CONFIGS['trot'])
            
            print(f"\\n🔄 Regenerating trajectories for {gait_config['name']} {gait_config['emoji']} mode...")
            print(f"   Steps={gait_params['num_steps']}, Lift={gait_params['lift_height']}mm, "
                  f"Stride={gait_params['step_length']}mm, Duty={gait_params['duty_factor']:.0%}, Smooth={gait_params['smooth_factor']:.1f}")
            
            for leg_id in ['FR', 'FL', 'RR', 'RL']:
                mirror_x = leg_id in ['FR', 'RR']
                trajectories[leg_id] = generate_elliptical_trajectory(
                    step_forward=gait_params['step_length'],
                    lift_height=gait_params['lift_height'],
                    num_steps=gait_params['num_steps'],
                    stance_ratio=gait_params['duty_factor'],
                    reverse=False,
                    mirror_x=mirror_x,
                    smooth_factor=gait_params['smooth_factor']
                )
            last_gait_type = current_gait_type
            frame = 0  # Reset frame counter
        
        if gait_paused:
            time.sleep(0.05)
            continue
        
        loop_start = time.perf_counter()
        
        # Get trajectory length based on current gait
        gait_params = get_gait_parameters()
        traj_len = gait_params['num_steps']
        
        # Update each leg
        for leg_id in ['FR', 'FL', 'RR', 'RL']:
            # In simulation mode, skip motor check
            if not SIMULATION_MODE and leg_id not in leg_motors:
                continue
            
            # Calculate current phase
            phase_offset = get_gait_phase_offset(leg_id)
            current_phase = (frame + int(phase_offset * traj_len)) % traj_len
            
            # Get target position from trajectory
            px, py = trajectories[leg_id][current_phase]
            
            # Calculate IK - try all configurations
            configs = [
                (True, True),   # C=Down, D=Down
                (True, False),  # C=Down, D=Up
                (False, True),  # C=Up, D=Down
                (False, False)  # C=Up, D=Up
            ]
            
            best_solution = None
            best_distance = float('inf')
            P_A, P_B = get_motor_positions(leg_id)
            
            for elbow_C, elbow_D in configs:
                solution = calculate_ik_no_ef(
                    np.array([px, py]),
                    P_A, P_B,
                    elbow_C_down=elbow_C,
                    elbow_D_down=elbow_D
                )
                
                if not np.isnan(solution).any():
                    if prev_solutions[leg_id] is None:
                        if elbow_C and elbow_D:
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
                    leg_states[leg_id]['target_angles'] = [theta_A, theta_B]
                    leg_states[leg_id]['target_pos'] = [px, py]
                    leg_states[leg_id]['phase'] = current_phase
                
                # Send to motors (only in real mode)
                if not SIMULATION_MODE:
                    motor_a = leg_motors[leg_id]['A']
                    motor_b = leg_motors[leg_id]['B']
                    
                    setpoint_a_deg = np.rad2deg(theta_A)
                    setpoint_b_deg = np.rad2deg(theta_B)
                    
                    motor_a.set_position_direct(setpoint_a_deg)
                    motor_b.set_position_direct(setpoint_b_deg)
                    
                    # Wait for feedback to arrive (motors respond asynchronously)
                    time.sleep(0.002)  # 2ms wait for both feedback packets
                    
                    # Read feedback from both motors
                    feedback_a = motor_a.read_feedback() if hasattr(motor_a, 'read_feedback') else None
                    feedback_b = motor_b.read_feedback() if hasattr(motor_b, 'read_feedback') else None
                    
                    # Log all setpoints with feedback (or None if missing)
                    if ENABLE_DATA_LOGGING:
                        log_motor_feedback(feedback_a, setpoint_a_deg, motor_a.motor_id)
                        log_motor_feedback(feedback_b, setpoint_b_deg, motor_b.motor_id)
                    
                    # Update actual angles
                    if feedback_a:
                        with viz_lock:
                            leg_states[leg_id]['actual_angles'][0] = np.deg2rad(feedback_a['position'])
                    
                    if feedback_b:
                        with viz_lock:
                            leg_states[leg_id]['actual_angles'][1] = np.deg2rad(feedback_b['position'])
        
        # Update frame counter
        frame = (frame + 1) % traj_len
        
        if frame == 0:
            cycle_count += 1
            if cycle_count % 5 == 0:
                print(f"  🔄 Gait Cycle #{cycle_count}")
        
        # Timing control
        elapsed = time.perf_counter() - loop_start
        sleep_time = max(0, (1.0 / UPDATE_RATE) - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    print("\n⏹️  Gait control loop stopped")

# ============================================================================
# KEYBOARD INPUT
# ============================================================================

def keyboard_input_thread():
    """Thread for handling keyboard input"""
    global gait_running, gait_paused
    
    print("\n" + "="*70)
    print("  ⌨️  KEYBOARD CONTROLS")
    print("="*70)
    if SIMULATION_MODE:
        print("  Use visualization window:")
        print("  [SPACE]  - Start/Pause gait (press in plot window)")
        print("  [1/T] Trot, [2/M] Smooth, [3/B] Back, [4/W] Walk, [5/C] Crawl, [6/S] Stand")
        print("  [Q]      - Quit (type here and press Enter)")
    else:
        print("  [SPACE]  - Start/Pause gait")
        print("  [1/T] Trot, [2/M] Smooth, [3/B] Back, [4/W] Walk, [5/C] Crawl, [6/S] Stand")
        print("  [E]      - Emergency stop all motors")
        print("  [R]      - Reset error statistics")
        print("  [Q]      - Quit program")
    print("="*70)
    
    if SIMULATION_MODE:
        # In simulation mode, wait for 'q' input
        while gait_running:
            try:
                cmd = input()
                if cmd.lower() == 'q':
                    print("\n👋 Quitting...")
                    gait_running = False
                    break
            except:
                time.sleep(0.1)
    else:
        # Real mode with keyboard monitoring
        while gait_running:
            key = check_keyboard_input()
            if key:
                if key == ' ':
                    toggle_gait_control()
                elif key == 'e':
                    emergency_stop_all()
                    with control_lock:
                        gait_paused = True
                    print("\n⚠️  Emergency stop activated!")
                elif key == 'r':
                    reset_error_stats()
                elif key == 'q':
                    print("\n👋 Quitting...")
                    gait_running = False
                    break
                elif key == '1' or key == 't':
                    change_gait_mode('trot')
                elif key == '2' or key == 'm':
                    change_gait_mode('smooth_trot')
                elif key == '3' or key == 'b':
                    change_gait_mode('backward_trot')
                elif key == '4' or key == 'w':
                    change_gait_mode('walk')
                elif key == '5' or key == 'c':
                    change_gait_mode('crawl')
                elif key == '6' or key == 's':
                    change_gait_mode('stand')
            
            time.sleep(0.05)

# ============================================================================
# MAIN FUNCTION
# ============================================================================

def main():
    global gait_running, gait_paused, motor_registry, leg_motors, plot_running, log_start_time
    
    print("="*70)
    print("  BLEGS Quadruped Gait Control - No EF Link Version")
    print("="*70)
    print(f"  Number of Legs: 4 (FR, FL, RR, RL)")
    print(f"  Total Motors: 8")
    print(f"  Protocol: Binary Protocol v1.2")
    print(f"  Baud Rate: {BAUD_RATE}")
    print(f"  Link Configuration: AC={L_AC}mm, BD={L_BD}mm, CE={L_CE}mm, DE={L_DE}mm")
    print(f"  NOTE: Joint E is the FOOT (no EF link)")
    
    # Let user select gait mode
    select_gait_mode()
    print(f"  Gait Type: {current_gait_type.upper()}")
    
    print(f"  Control Mode: {'S-Curve' if CONTROL_MODE == ControlMode.MODE_SCURVE_PROFILE else 'Direct'}")
    print(f"  Update Rate: {UPDATE_RATE} Hz")
    print(f"  Gear Ratio: {GEAR_RATIO}:1")
    print(f"  Motor Init Angle: {MOTOR_INIT_ANGLE}° (robot)")
    print(f"  Home Position: ({DEFAULT_STANCE_OFFSET_X}, {DEFAULT_STANCE_HEIGHT}) mm")
    print(f"  Visualization: {'Enabled' if ENABLE_VISUALIZATION else 'Disabled'}")
    print("="*70)
    
    # --- Step 1: Motor Discovery ---
    discovered = discover_motors()
    
    if len(discovered) == 0 and not SIMULATION_MODE:
        print("\n❌ No motors discovered and simulation not enabled. Exiting.")
        return
    
    if not SIMULATION_MODE:
        # --- Step 2: Motor Registration ---
        all_assigned = register_leg_motors(discovered)
        
        if not all_assigned:
            print("\n⚠️  Not all motors were found.")
            user_input = input("  Continue with available motors? (y/n): ").strip().lower()
            if user_input != 'y':
                print("\n  Aborting...")
                for controller in discovered.values():
                    controller.disconnect()
                return
        
        if not leg_motors:
            print("\n❌ No complete leg pairs found. Cannot continue.")
            for controller in discovered.values():
                controller.disconnect()
            return
        
        # --- Step 3: Start All Motors ---
        start_all_motors()
    else:
        print("\n" + "="*70)
        print("  🎮 SIMULATION MODE ACTIVATED")
        print("="*70)
        print("  Running without real motors")
        print("  Visualization will show leg movements")
        print("="*70)
    
    # --- Step 3.5: Initialize Data Logging ---
    if ENABLE_DATA_LOGGING:
        print("\n📝 Initializing data logging...")
        log_start_time = time.time()
        initialize_data_logging()
    
    # --- Step 4: Start Visualization ---
    viz_thread = None
    if ENABLE_VISUALIZATION:
        print("\n📊 Starting real-time visualization...")
        viz_thread = threading.Thread(target=visualization_thread, daemon=True)
        viz_thread.start()
        time.sleep(1.0)
        print("  Visualization started!")
    
    # --- Step 5: Generate Trajectories ---
    print(f"\n🚶 Generating walking trajectories...")
    trajectories = {}
    
    # Get gait parameters from configuration
    gait_params = get_gait_parameters()
    gait_config = GAIT_CONFIGS.get(current_gait_type, GAIT_CONFIGS['trot'])
    
    num_steps = gait_params['num_steps']
    lift_height = gait_params['lift_height']
    step_length = gait_params['step_length']
    duty_factor = gait_params['duty_factor']
    smooth_factor = gait_params['smooth_factor']
    
    print(f"  Gait: {gait_config['name']} {gait_config['emoji']}")
    print(f"  Parameters: steps={num_steps}, lift={lift_height}mm, stride={step_length}mm, duty={duty_factor:.0%}, smooth={smooth_factor:.1f}")
    
    # Generate trajectories for all legs
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        mirror_x = leg_id in ['FR', 'RR']
        trajectories[leg_id] = generate_elliptical_trajectory(
            step_forward=step_length,
            lift_height=lift_height,
            num_steps=num_steps,
            stance_ratio=duty_factor,
            reverse=False,
            mirror_x=mirror_x,
            smooth_factor=smooth_factor
        )
    
    print(f"  Generated {num_steps} waypoints per leg")
    
    # --- Step 6: Initialize Home Position ---
    print(f"\n🏠 Moving to home position...")
    print(f"  Note: Motors start at {MOTOR_INIT_ANGLE}° (robot angle)")
    prev_solutions = {}
    
    for leg_id in (leg_motors.keys() if not SIMULATION_MODE else ['FR', 'FL', 'RR', 'RL']):
        home_pos = np.array([DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT])
        P_A, P_B = get_motor_positions(leg_id)
        home_angles = calculate_ik_no_ef(home_pos, P_A, P_B, elbow_C_down=True, elbow_D_down=True)
        
        if not np.isnan(home_angles).any():
            prev_solutions[leg_id] = home_angles
            
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
            if not SIMULATION_MODE:
                motor_a = leg_motors[leg_id]['A']
                motor_b = leg_motors[leg_id]['B']
                motor_a.set_position_scurve(target_angle_A, 3000)
                motor_b.set_position_scurve(target_angle_B, 3000)
            
            print(f"    {leg_id}: θA={target_angle_A:+.1f}° (Δ{delta_A:+.1f}°), θB={target_angle_B:+.1f}° (Δ{delta_B:+.1f}°)")
        else:
            print(f"    {leg_id}: IK FAILED for home position!")
            prev_solutions[leg_id] = None
    
    if not SIMULATION_MODE:
        print(f"\n  ⏳ Moving to home position (3 seconds)...")
        time.sleep(3.5)  # Wait for home position (3s movement + margin)
    
    # --- Step 7: Main Gait Loop ---
    print(f"\n⏸️  Gait control ready (PAUSED) - Mode: {current_gait_type.upper()}")
    if ENABLE_VISUALIZATION:
        print("  Press [SPACE] in visualization window to start")
        print("  Press [1/T] Trot, [2/M] Smooth, [3/B] Back, [4/W] Walk, [5/C] Crawl, [6/S] Stand")
    else:
        print("  Press [SPACE] to start/pause")
        print("  Press [1/T] Trot, [2/M] Smooth, [3/B] Back, [4/W] Walk, [5/C] Crawl, [6/S] Stand")
    print("  Press [E] for emergency stop")
    if not ENABLE_VISUALIZATION:
        print("  Press [Q] to quit")
    else:
        print("  Press Ctrl+C to exit")
    print("="*70)
    
    # Start gait control
    gait_running = True
    
    gait_thread = threading.Thread(target=gait_control_loop, args=(trajectories, prev_solutions), daemon=True)
    gait_thread.start()
    
    # Start keyboard input handler
    try:
        keyboard_input_thread()
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user")
    
    # Cleanup
    gait_running = False
    plot_running = False
    time.sleep(0.5)
    
    print("\n" + "="*70)
    print("  🛑 SHUTTING DOWN")
    print("="*70)
    
    # Return to init position (-90°)
    if not SIMULATION_MODE:
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
    
    # Close data logging
    if ENABLE_DATA_LOGGING:
        close_data_logging()
    
    # Print statistics
    if not SIMULATION_MODE:
        print("\n📊 Communication Statistics:")
        for motor_id, controller in motor_registry.items():
            stats = controller.get_stats()
            print(f"    Motor {motor_id}: TX={stats['tx']}, RX={stats['rx']}, "
                  f"Errors={stats['errors']}, Success={stats['success_rate']:.1f}%")
    
    # Disconnect motors (only in real mode)
    if not SIMULATION_MODE:
        print("\n🔌 Disconnecting motors...")
        for controller in motor_registry.values():
            controller.disconnect()
    
    if ENABLE_VISUALIZATION:
        print("\n📊 Closing visualization...")
        plt.close('all')
        time.sleep(1.0)
    
    print("\n✅ Quadruped gait control terminated successfully")
    print("="*70)

if __name__ == "__main__":
    main()
