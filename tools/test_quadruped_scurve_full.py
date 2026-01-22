"""
Quadruped Gait Control - Full S-Curve Mode
Author: M-TRCH
Date: January 22, 2026

This script controls a quadruped robot (4 legs, 8 motors) using Full S-Curve mode.
All motor commands use MODE_SCURVE_FULL with configurable vmax, amax, jmax parameters.

Features:
- Full S-Curve trajectory control with explicit parameters
- Tunable velocity, acceleration, and jerk limits
- Smooth transitions for all movements
- Automatic motor discovery and registration

Protocol: Binary Protocol v1.2 with MODE_SCURVE_FULL (0x02)
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
    MODE_SCURVE_FULL = 0x02  # Full S-Curve with vmax, amax, jmax

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
SERIAL_TIMEOUT = 0.2   # 200ms timeout for discovery
FAST_TIMEOUT = 0.01    # 10ms timeout for high-speed operation
PING_RETRIES = 5       # Number of PING retries during discovery

# Debug mode
DEBUG_SERIAL = False

# ============================================================================
# ROBOT CONFIGURATION
# ============================================================================

# --- Robot Body Dimensions (mm) ---
BODY_LENGTH = 200.0
BODY_WIDTH = 170.0

# --- Five-Bar Linkage Parameters (mm) ---
MOTOR_SPACING = 85.0

# Link lengths (same for all 4 legs)
L_AC = 105.0
L_BD = 105.0
L_CE = 145.0
L_DE = 145.0
L_EF = 40.0

# Offset ratios
OFFSET_RATIO_E = 37.0 / 29.0
OFFSET_RATIO_D = 8.0 / 29.0

# --- Motor Configuration ---
GEAR_RATIO = 8.0
MOTOR_INIT_ANGLE = -90.0

# --- Expected Motor IDs for Each Leg ---
EXPECTED_MOTOR_IDS = {
    'FL': {'A': 1, 'B': 2},
    'FR': {'A': 3, 'B': 4},
    'RL': {'A': 5, 'B': 6},
    'RR': {'A': 7, 'B': 8}
}

# --- Leg Motor Positions in Leg Frame (mm) ---
P_A_LEFT = np.array([-MOTOR_SPACING/2, 0.0])
P_B_LEFT = np.array([MOTOR_SPACING/2, 0.0])
P_A_RIGHT = np.array([MOTOR_SPACING/2, 0.0])
P_B_RIGHT = np.array([-MOTOR_SPACING/2, 0.0])

# --- Default Standing Pose ---
DEFAULT_STANCE_HEIGHT = -200.0  # mm
DEFAULT_STANCE_OFFSET_X = 0.0   # mm

# --- Motion Parameters ---
GAIT_LIFT_HEIGHT = 15.0    # mm
GAIT_STEP_FORWARD = 50.0   # mm

# Smooth Trot Parameters
SMOOTH_TROT_STANCE_RATIO = 0.65

# ============================================================================
# S-CURVE PARAMETERS (Full Control Mode)
# ============================================================================

# --- S-Curve Profile Parameters ---
# These values control the motion profile for all movements
# Values are in ROBOT coordinates (before gear ratio scaling)
# Actual motor values = these × GEAR_RATIO
# START SLOW for testing - increase gradually!

# Default parameters for gait control (SLOW START for testing)
# Tested working values from firmware: v_max=4000, a_max=180000 (motor shaft)
SCURVE_GAIT_V_MAX = 50        # Max velocity (degrees/s) - VERY SLOW for testing
SCURVE_GAIT_A_MAX = 500       # Max acceleration (×10 in protocol) = 5,000 deg/s² robot
SCURVE_GAIT_J_MAX = 500       # Max jerk (×100 in protocol) - not used in trapezoidal

# Parameters for slow/smooth movements (init, home transitions)
SCURVE_SLOW_V_MAX = 30        # Max velocity (degrees/s) - VERY SLOW
SCURVE_SLOW_A_MAX = 300       # Max acceleration (×10) = 3,000 deg/s² robot
SCURVE_SLOW_J_MAX = 300       # Max jerk (×100) - not used

# Parameters for single motor oscillation test
SCURVE_TEST_V_MAX = 50        # Max velocity (degrees/s) - SLOW for testing
SCURVE_TEST_A_MAX = 500       # Max acceleration (×10)
SCURVE_TEST_J_MAX = 500       # Max jerk (×100)

# ============================================================================
# CONTROL PARAMETERS
# ============================================================================

CONTROL_MODE = ControlMode.MODE_SCURVE_FULL  # ใช้ Full S-Curve เท่านั้น!
UPDATE_RATE = 50  # Hz (20ms per update)
TRAJECTORY_STEPS = 20
GAIT_TYPE = 'trot'

# --- Single Motor Mode ---
SINGLE_MOTOR_MODE = False
SINGLE_MOTOR_OSCILLATION = 30.0  # degrees
SINGLE_MOTOR_PERIOD = 0.6  # seconds

# --- Visualization Parameters ---
ENABLE_VISUALIZATION = False
PLOT_UPDATE_RATE = 10  # Hz 

# ============================================================================
# GLOBAL VARIABLES
# ============================================================================

viz_lock = threading.Lock()
error_lock = threading.Lock()
control_lock = threading.Lock()

plot_running = True
gait_running = False
gait_paused = True

MOTOR_INIT_ANGLE_RAD = np.deg2rad(MOTOR_INIT_ANGLE)
leg_states = {
    'FR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'red'},
    'FL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'blue'},
    'RR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'orange'},
    'RL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'green'}
}

motor_registry = {}
leg_motors = {}
error_stats = {}
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
    
    crc_data = type_byte + payload_len + payload
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    
    return header + type_byte + payload_len + payload + crc_bytes

def open_serial_with_timeout(port, baudrate, timeout, open_timeout=3.0):
    """Open serial port with a timeout to prevent blocking."""
    result = {'serial': None, 'error': None}
    
    def try_open():
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                dsrdtr=False,
                rtscts=False
            )
            ser.dtr = False
            ser.rts = False
            result['serial'] = ser
        except Exception as e:
            result['error'] = str(e)
    
    thread = threading.Thread(target=try_open, daemon=True)
    thread.start()
    thread.join(timeout=open_timeout)
    
    if thread.is_alive():
        result['error'] = f"Timeout opening port (>{open_timeout}s)"
        return None, result['error']
    
    return result['serial'], result['error']

# ============================================================================
# MOTOR CONTROLLER CLASS (Full S-Curve Mode)
# ============================================================================

class FullSCurveMotorController:
    """Motor controller using Full S-Curve mode exclusively"""
    
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
        
        # Default S-Curve parameters
        self.default_v_max = SCURVE_GAIT_V_MAX
        self.default_a_max = SCURVE_GAIT_A_MAX
        self.default_j_max = SCURVE_GAIT_J_MAX
    
    def set_scurve_params(self, v_max: int, a_max: int, j_max: int):
        """Set default S-Curve parameters for this controller"""
        self.default_v_max = v_max
        self.default_a_max = a_max
        self.default_j_max = j_max
    
    def connect(self, timeout=SERIAL_TIMEOUT, open_timeout=3.0):
        """Connect to motor via serial port with timeout protection"""
        try:
            print(f"      📂 Opening serial port (timeout={open_timeout}s)...")
            
            self.serial, error = open_serial_with_timeout(
                port=self.port,
                baudrate=BAUD_RATE,
                timeout=timeout,
                open_timeout=open_timeout
            )
            
            if self.serial is None:
                print(f"      ❌ Failed to open: {error}")
                self.is_connected = False
                return False
            
            print(f"      ✓ Port opened (DTR/RTS disabled)")
            
            print(f"      ⏳ Waiting for MCU ready (500ms)...")
            time.sleep(0.5)
            
            print(f"      ✓ Flushing buffers...")
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.1)
            self.serial.reset_input_buffer()
            
            self.is_connected = True
            print(f"      ✅ Connection established!")
            return True
            
        except serial.SerialException as e:
            print(f"      ❌ Serial error: {e}")
            self.is_connected = False
            return False
        except Exception as e:
            print(f"      ❌ Connect exception: {e}")
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
            self.serial.reset_output_buffer()
            time.sleep(0.02)
            
            packet = build_packet(PacketType.PKT_CMD_PING)
            bytes_written = self.serial.write(packet)
            self.serial.flush()
            self.stats_tx_count += 1
            
            if DEBUG_SERIAL:
                print(f"        [TX] {bytes_written} bytes: {packet.hex()}")
            
            start_wait = time.time()
            max_wait = max(0.3, self.serial.timeout if self.serial.timeout else 0.3)
            
            while (time.time() - start_wait) < max_wait:
                if self.serial.in_waiting >= 2:
                    break
                time.sleep(0.005)
            
            bytes_waiting = self.serial.in_waiting
            if bytes_waiting < 2:
                if DEBUG_SERIAL:
                    print(f"        [RX] No response")
                return None
            
            if DEBUG_SERIAL:
                print(f"        [RX] {bytes_waiting} bytes waiting")
            
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
            print(f"      [DEBUG] send_ping exception: {e}")
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
    
    def set_position(self, angle_deg: float, v_max: int = None, 
                     a_max: int = None, j_max: int = None) -> bool:
        """
        Set target position using Full S-Curve profile
        
        Args:
            angle_deg: Target angle in robot coordinate (degrees)
            v_max: Max velocity (degrees/s), None = use default
            a_max: Max acceleration (degrees/s² / 10), None = use default
            j_max: Max jerk (degrees/s³ / 100), None = use default
            
        Returns:
            True if command sent successfully
        """
        if not self.is_connected:
            return False
        
        # Use defaults if not specified
        if v_max is None:
            v_max = self.default_v_max
        if a_max is None:
            a_max = self.default_a_max
        if j_max is None:
            j_max = self.default_j_max
        
        try:
            motor_angle = angle_deg * GEAR_RATIO
            
            # Scale velocity and acceleration for motor shaft (× GEAR_RATIO)
            # Motor spins faster than output by gear ratio
            motor_v_max = int(v_max * GEAR_RATIO)
            motor_a_max = int(a_max * GEAR_RATIO)
            motor_j_max = int(j_max * GEAR_RATIO)  # Not used but send anyway
            
            # Clamp to uint16 max (65535)
            motor_v_max = min(motor_v_max, 65535)
            motor_a_max = min(motor_a_max, 65535)
            motor_j_max = min(motor_j_max, 65535)
            
            with self.lock:
                mode = bytes([ControlMode.MODE_SCURVE_FULL])
                target_pos = struct.pack('<i', int(motor_angle * 100))
                v_max_bytes = struct.pack('<H', motor_v_max)
                a_max_bytes = struct.pack('<H', motor_a_max)
                j_max_bytes = struct.pack('<H', motor_j_max)
                payload = mode + target_pos + v_max_bytes + a_max_bytes + j_max_bytes
                
                packet = build_packet(PacketType.PKT_CMD_SET_GOAL, payload)
                self.serial.write(packet)
                self.serial.flush()
                self.stats_tx_count += 1
                self.current_setpoint = angle_deg
                
                # Try to read feedback response
                time.sleep(0.001)
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
    
    def set_position_slow(self, angle_deg: float) -> bool:
        """Set position with slow/smooth S-Curve parameters"""
        return self.set_position(angle_deg, 
                                  SCURVE_SLOW_V_MAX, 
                                  SCURVE_SLOW_A_MAX, 
                                  SCURVE_SLOW_J_MAX)
    
    def set_position_fast(self, angle_deg: float) -> bool:
        """Set position with fast gait S-Curve parameters"""
        return self.set_position(angle_deg, 
                                  SCURVE_GAIT_V_MAX, 
                                  SCURVE_GAIT_A_MAX, 
                                  SCURVE_GAIT_J_MAX)
    
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
        """Read a complete packet from serial"""
        try:
            if self.serial.in_waiting < 2:
                return None
            
            header = self.serial.read(2)
            if len(header) != 2:
                return None
            
            if header[0] != HEADER_1 or header[1] != HEADER_2:
                if self.serial.in_waiting > 0:
                    self.serial.reset_input_buffer()
                return None
            
            meta = self.serial.read(2)
            if len(meta) != 2:
                return None
            
            pkt_type = meta[0]
            payload_len = meta[1]
            
            if payload_len > 128:
                self.serial.reset_input_buffer()
                return None
            
            payload = self.serial.read(payload_len)
            if len(payload) != payload_len:
                return None
            
            crc_bytes = self.serial.read(2)
            if len(crc_bytes) != 2:
                return None
            
            received_crc = struct.unpack('<H', crc_bytes)[0]
            crc_data = bytes([pkt_type, payload_len]) + payload
            calculated_crc = calculate_crc16(crc_data)
            
            if received_crc != calculated_crc:
                self.stats_errors += 1
                return None
            
            self.stats_rx_count += 1
            return (pkt_type, payload)
            
        except Exception as e:
            print(f"      [DEBUG] _read_packet exception: {e}")
            return None
    
    def read_feedback(self) -> dict:
        """Read motor feedback from binary protocol"""
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
    """Discover all motors by scanning COM ports and sending PING"""
    print("\n" + "="*70)
    print("  🔍 MOTOR DISCOVERY (Full S-Curve Mode)")
    print("="*70)
    
    ports = scan_com_ports()
    
    if not ports:
        print("  ❌ No COM ports found!")
        return {}
    
    print(f"\n  Found {len(ports)} COM port(s):")
    for port in ports:
        print(f"    • {port['device']}: {port['description']}")
    
    discovered = {}
    
    print(f"\n  📡 Scanning for motors (timeout={SERIAL_TIMEOUT*1000:.0f}ms, retries={PING_RETRIES})...")
    
    for port_info in ports:
        port = port_info['device']
        print(f"\n    Checking {port}...")
        
        controller = FullSCurveMotorController(port)
        
        if not controller.connect(timeout=SERIAL_TIMEOUT):
            print(f"      ❌ Failed to open {port}")
            continue
        
        print(f"      🔗 Connected, sending PING...")
        
        motor_info = None
        for attempt in range(PING_RETRIES):
            print(f"      📡 PING attempt {attempt + 1}/{PING_RETRIES}...")
            motor_info = controller.send_ping()
            if motor_info:
                break
            time.sleep(0.1)
        
        if motor_info:
            motor_id = motor_info['motor_id']
            print(f"      ✅ Motor ID {motor_id} found!")
            print(f"         Position: {motor_info['position']:.2f}°")
            print(f"         Current:  {motor_info['current']} mA")
            print(f"         Flags:    0x{motor_info['flags']:02X}")
            
            discovered[motor_id] = controller
            time.sleep(0.2)
        else:
            print(f"      ⚠️  No response (no motor on this port)")
            controller.disconnect()
        
        time.sleep(0.1)
    
    if discovered:
        print(f"\n  ⏳ Stabilizing after discovery...")
        time.sleep(0.5)
    
    print(f"\n  📊 Discovery complete: {len(discovered)} motor(s) found")
    return discovered

def register_leg_motors(discovered_motors: dict) -> bool:
    """Register discovered motors to leg assignments"""
    global leg_motors, motor_registry
    
    print("\n" + "="*70)
    print("  🔗 MOTOR REGISTRATION (Full S-Curve Mode)")
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
            time.sleep(0.1)
        else:
            all_assigned = False
            if not motor_a:
                missing_motors.append(f"{leg_id} Motor A (ID {motor_a_id})")
            if not motor_b:
                missing_motors.append(f"{leg_id} Motor B (ID {motor_b_id})")
            print(f"    ❌ {leg_id}: INCOMPLETE - Missing motor(s)")
    
    if leg_motors:
        print(f"\n  ⏳ Stabilizing after registration...")
        time.sleep(0.5)
    
    if missing_motors:
        print(f"\n  ⚠️  Missing motors:")
        for m in missing_motors:
            print(f"      • {m}")
    
    return all_assigned

def start_all_motors() -> bool:
    """Start all registered motors"""
    print("\n" + "="*70)
    print("  🚀 STARTING MOTORS (Full S-Curve Mode)")
    print("="*70)
    
    success_count = 0
    total_count = len(motor_registry)
    
    for motor_id, controller in motor_registry.items():
        print(f"  🔧 Starting Motor ID {motor_id}...")
        
        controller.set_timeout(FAST_TIMEOUT)
        time.sleep(0.1)
        
        result = controller.send_ping()
        if result:
            success_count += 1
            print(f"    ✅ Motor ID {motor_id} started successfully")
        else:
            print(f"    ❌ Failed to start Motor ID {motor_id}")
        
        time.sleep(0.2)
    
    print(f"\n  📊 Started {success_count}/{total_count} motors")
    
    if success_count > 0:
        print("  ⏳ Waiting for motor initialization (3 seconds)...")
        time.sleep(3.0)
    
    return success_count == total_count

# ============================================================================
# KINEMATICS FUNCTIONS
# ============================================================================

def get_motor_positions(leg_id):
    """Get motor positions based on leg side"""
    if leg_id in ['FL', 'RL']:
        return P_A_LEFT.copy(), P_B_LEFT.copy()
    else:
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
    """Calculate Inverse Kinematics using analytical method"""
    R_FD = L_DE * OFFSET_RATIO_E
    R_DB = L_BD
    
    P_D = solve_circle_intersection(P_F_target, R_FD, P_B, R_DB, not elbow_D_down)
    
    if np.isnan(P_D).any():
        return np.array([np.nan, np.nan])
    
    P_E = (29.0 * P_F_target + 8.0 * P_D) / 37.0
    P_C = solve_circle_intersection(P_A, L_AC, P_E, L_CE, elbow_C_down)
    
    if np.isnan(P_C).any():
        return np.array([np.nan, np.nan])
    
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
            
            if P_F1[1] < P_F2[1]:
                return P_C, P_D, P_E1, P_F1
            else:
                return P_C, P_D, P_E2, P_F2
    
    return None, None, None, None

# ============================================================================
# TRAJECTORY GENERATION
# ============================================================================

def generate_elliptical_trajectory(num_steps=60, lift_height=30.0, step_forward=60.0, mirror_x=False,
                                   stance_ratio=0.5, home_x=0.0, home_y=None, reverse=False):
    """Generate elliptical walking trajectory for one leg"""
    if home_y is None:
        home_y = DEFAULT_STANCE_HEIGHT
    
    trajectory = []
    
    swing_steps = int(num_steps * (1.0 - stance_ratio))
    stance_steps = num_steps - swing_steps
    
    if swing_steps < 1:
        swing_steps = 1
        stance_steps = num_steps - 1
    if stance_steps < 1:
        stance_steps = 1
        swing_steps = num_steps - 1
    
    direction = -1 if reverse else 1
    home_stance_index = stance_steps // 2
    temp_trajectory = []
    
    for i in range(num_steps):
        if i < swing_steps:
            phase_progress = i / swing_steps
            t = np.pi + np.pi * phase_progress
            py = home_y + lift_height * abs(np.sin(t))
        else:
            stance_index = i - swing_steps
            phase_progress = stance_index / stance_steps
            t = np.pi * phase_progress
            py = home_y
        
        px = direction * (-step_forward * np.cos(t))
        if mirror_x:
            px = -px
        
        temp_trajectory.append((px + home_x, py))
    
    home_index = swing_steps + home_stance_index
    trajectory = temp_trajectory[home_index:] + temp_trajectory[:home_index]
    
    return trajectory

def get_gait_phase_offset(leg_id, gait_type='trot'):
    """Get phase offset for each leg based on gait type"""
    if gait_type == 'trot':
        offsets = {'FR': 0.0, 'FL': 0.5, 'RR': 0.5, 'RL': 0.0}
        return offsets[leg_id]
    elif gait_type == 'walk':
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
            print("\n▶️  Gait control STARTED (Full S-Curve Mode)!")
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

def smooth_move_to_home_position(leg_motors_dict, home_angles_dict, duration_s=3.0):
    """Move all motors to home position using slow S-Curve profile"""
    print(f"\n🏠 Moving to home position using Full S-Curve (SLOW)...")
    print(f"    Duration: ~{duration_s:.1f}s")
    print(f"    V_max: {SCURVE_SLOW_V_MAX} deg/s")
    print(f"    A_max: {SCURVE_SLOW_A_MAX * 10} deg/s²")
    print(f"    J_max: {SCURVE_SLOW_J_MAX * 100} deg/s³")
    
    for leg_id, motors in leg_motors_dict.items():
        if leg_id not in home_angles_dict:
            continue
        
        target_A, target_B = home_angles_dict[leg_id]
        
        # Use slow S-Curve parameters for smooth movement
        motors['A'].set_position_slow(target_A)
        motors['B'].set_position_slow(target_B)
        
        with viz_lock:
            leg_states[leg_id]['target_angles'] = [np.deg2rad(target_A), np.deg2rad(target_B)]
        
        print(f"    {leg_id}: θA={target_A:+.1f}°, θB={target_B:+.1f}°")
    
    print(f"\n    ⏳ Waiting for motors to reach home position...")
    time.sleep(duration_s + 0.5)
    
    print(f"    ✅ Home position reached!")

def check_keyboard_input():
    """Check for keyboard input (non-blocking)"""
    if sys.platform == 'win32':
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b'\xe0' or key == b'\x00':
                msvcrt.getch()
                return None
            return key.decode('utf-8', errors='ignore').lower()
    return None

# ============================================================================
# SINGLE MOTOR CONTROL (Full S-Curve Mode)
# ============================================================================

def run_single_motor_mode(controller):
    """Run single motor oscillation test with Full S-Curve"""
    global gait_running, gait_paused, plot_running
    
    print("\n" + "="*70)
    print("  🔧 SINGLE MOTOR TEST MODE (Full S-Curve)")
    print("="*70)
    print(f"  Motor ID: {controller.motor_id}")
    print(f"  Oscillation: ±{SINGLE_MOTOR_OSCILLATION}°")
    print(f"  Period: {SINGLE_MOTOR_PERIOD}s")
    print(f"  S-Curve Parameters:")
    print(f"    V_max: {SCURVE_TEST_V_MAX} deg/s")
    print(f"    A_max: {SCURVE_TEST_A_MAX * 10} deg/s²")
    print(f"    J_max: {SCURVE_TEST_J_MAX * 100} deg/s³")
    print("="*70)
    
    print("\n🚀 Starting motor...")
    if not controller.start_motor():
        print("❌ Failed to start motor!")
        return
    
    controller.set_timeout(FAST_TIMEOUT)
    
    initial_pos = controller.current_position
    print(f"  Initial position: {initial_pos:.2f}°")
    
    center_pos = initial_pos
    print(f"\n🏠 Using current position as center ({center_pos:.2f}°)")
    print(f"  Oscillation range: {center_pos - SINGLE_MOTOR_OSCILLATION:.1f}° to {center_pos + SINGLE_MOTOR_OSCILLATION:.1f}°")
    
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
            key = check_keyboard_input()
            if key:
                if key == ' ':
                    toggle_gait_control()
                    if not gait_paused:
                        start_time = time.time()
                elif key == 'e':
                    controller.send_emergency_stop()
                    gait_paused = True
                elif key == 'q':
                    running = False
                    continue
            
            with control_lock:
                is_paused = gait_paused
            
            if is_paused:
                time.sleep(0.01)
                continue
            
            loop_start = time.perf_counter()
            
            elapsed = time.time() - start_time
            phase = (elapsed % SINGLE_MOTOR_PERIOD) / SINGLE_MOTOR_PERIOD
            target_angle = center_pos + SINGLE_MOTOR_OSCILLATION * np.sin(2 * np.pi * phase)
            
            # Use Full S-Curve with test parameters
            controller.set_position(target_angle, 
                                     SCURVE_TEST_V_MAX, 
                                     SCURVE_TEST_A_MAX, 
                                     SCURVE_TEST_J_MAX)
            
            feedback = controller.read_feedback()
            
            current_time = time.time()
            if current_time - last_status_time >= 1.0:
                last_status_time = current_time
                current_pos = controller.current_position
                error = target_angle - current_pos
                print(f"\r  Target: {target_angle:+6.1f}°  Actual: {current_pos:+6.1f}°  Error: {error:+5.1f}°", end='', flush=True)
            
            elapsed_loop = time.perf_counter() - loop_start
            sleep_time = max(0, (1.0 / UPDATE_RATE) - elapsed_loop)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n⏹️  Single motor test stopped by user")
    
    finally:
        print("\n\n🏠 Returning to init position (-90°)...")
        controller.set_position_slow(MOTOR_INIT_ANGLE)
        time.sleep(3.0)
        
        stats = controller.get_stats()
        print(f"\n📊 Communication Statistics:")
        print(f"    Motor {stats['motor_id']}: TX={stats['tx']}, RX={stats['rx']}, "
              f"Errors={stats['errors']}, Success={stats['success_rate']:.1f}%")
        
        print("\n🔌 Disconnecting motor...")
        controller.disconnect()
        
        print("\n✅ Single motor test completed")
        print("="*70)

# ============================================================================
# MAIN CONTROL LOOP
# ============================================================================

def main():
    global plot_running, gait_running, gait_paused, leg_states
    
    print("="*70)
    print("  BLEGS Quadruped Gait Control - FULL S-CURVE MODE")
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
    
    print(f"  Control Mode: FULL S-CURVE (MODE_SCURVE_FULL)")
    print(f"  Update Rate: {UPDATE_RATE} Hz")
    print(f"  Gear Ratio: {GEAR_RATIO}:1")
    
    print(f"\n  📊 S-Curve Parameters (Gait):")
    print(f"      V_max: {SCURVE_GAIT_V_MAX} deg/s")
    print(f"      A_max: {SCURVE_GAIT_A_MAX * 10} deg/s²")
    print(f"      J_max: {SCURVE_GAIT_J_MAX * 100} deg/s³")
    
    if not SINGLE_MOTOR_MODE:
        print(f"\n  Motor Init Angle: {MOTOR_INIT_ANGLE}° (robot)")
        print(f"  Home Position: ({DEFAULT_STANCE_OFFSET_X}, {DEFAULT_STANCE_HEIGHT}) mm")
    else:
        print(f"\n  Oscillation: ±{SINGLE_MOTOR_OSCILLATION}°")
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
        motor_id = list(discovered_motors.keys())[0]
        controller = discovered_motors[motor_id]
        
        print(f"\n✅ Using Motor ID {motor_id} for single motor test")
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
    
    # --- Step 4: Generate Trajectories ---
    print(f"\n🚶 Generating walking trajectories...")
    trajectories = {}
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        mirror_x = leg_id in ['FR', 'RR']
        trajectories[leg_id] = generate_elliptical_trajectory(
            num_steps=TRAJECTORY_STEPS,
            lift_height=GAIT_LIFT_HEIGHT,
            step_forward=GAIT_STEP_FORWARD,
            mirror_x=mirror_x,
            stance_ratio=SMOOTH_TROT_STANCE_RATIO,
            home_x=DEFAULT_STANCE_OFFSET_X,
            home_y=DEFAULT_STANCE_HEIGHT
        )
    
    first_waypoint = trajectories['FL'][0]
    print(f"  Generated {TRAJECTORY_STEPS} waypoints per leg")
    print(f"  Stance ratio: {SMOOTH_TROT_STANCE_RATIO:.0%}")
    print(f"  First waypoint: ({first_waypoint[0]:.1f}, {first_waypoint[1]:.1f}) mm")
    print(f"  Home position:  ({DEFAULT_STANCE_OFFSET_X:.1f}, {DEFAULT_STANCE_HEIGHT:.1f}) mm")
    
    # --- Step 5: Initialize Home Position ---
    print(f"\n🏠 Calculating home position...")
    print(f"  Note: Motors start at {MOTOR_INIT_ANGLE}° (robot angle)")
    prev_solutions = {}
    home_angles_all = {}
    
    for leg_id in leg_motors.keys():
        home_pos = np.array([DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT])
        P_A, P_B = get_motor_positions(leg_id)
        home_angles = calculate_ik_analytical(home_pos, P_A, P_B, elbow_C_down=True, elbow_D_down=False)
        
        if not np.isnan(home_angles).any():
            prev_solutions[leg_id] = home_angles
            
            target_angle_A = np.rad2deg(home_angles[0])
            target_angle_B = np.rad2deg(home_angles[1])
            home_angles_all[leg_id] = [target_angle_A, target_angle_B]
            
            init_angle_A = MOTOR_INIT_ANGLE
            init_angle_B = MOTOR_INIT_ANGLE
            delta_A = target_angle_A - init_angle_A
            delta_B = target_angle_B - init_angle_B
            
            with viz_lock:
                leg_states[leg_id]['target_angles'] = home_angles.tolist()
                leg_states[leg_id]['target_pos'] = home_pos.tolist()
            
            print(f"    {leg_id}: θA={target_angle_A:+.1f}° (Δ{delta_A:+.1f}°), θB={target_angle_B:+.1f}° (Δ{delta_B:+.1f}°)")
        else:
            print(f"    {leg_id}: IK FAILED for home position!")
    
    # Move to home position with slow S-Curve
    smooth_move_to_home_position(leg_motors, home_angles_all, duration_s=3.0)
    
    # --- Step 6: Main Gait Loop ---
    print(f"\n⏸️  Gait control ready (PAUSED)")
    print("  Press [SPACE] to start/pause")
    print("  Press [E] for emergency stop")
    print("  Press [Q] to quit")
    print("="*70)
    
    cycle_count = 0
    frame = 0
    last_status_time = 0
    running = True
    
    try:
        while running and plot_running:
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
            
            with control_lock:
                is_paused = gait_paused
            
            if is_paused:
                time.sleep(0.01)
                continue
            
            loop_start = time.perf_counter()
            
            # Update each leg
            for leg_id in leg_motors.keys():
                phase_offset = get_gait_phase_offset(leg_id, GAIT_TYPE)
                current_phase = (frame + int(phase_offset * TRAJECTORY_STEPS)) % TRAJECTORY_STEPS
                
                if leg_id in trajectories:
                    px, py = trajectories[leg_id][current_phase]
                else:
                    px, py = DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT
                
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
                    
                    with viz_lock:
                        leg_states[leg_id]['target_angles'] = best_solution.tolist()
                        leg_states[leg_id]['target_pos'] = [px, py]
                        leg_states[leg_id]['phase'] = current_phase
                    
                    motor_a = leg_motors[leg_id]['A']
                    motor_b = leg_motors[leg_id]['B']
                    
                    # Use Full S-Curve with gait parameters
                    motor_a.set_position(np.rad2deg(theta_A), 
                                          SCURVE_GAIT_V_MAX, 
                                          SCURVE_GAIT_A_MAX, 
                                          SCURVE_GAIT_J_MAX)
                    motor_b.set_position(np.rad2deg(theta_B), 
                                          SCURVE_GAIT_V_MAX, 
                                          SCURVE_GAIT_A_MAX, 
                                          SCURVE_GAIT_J_MAX)
                    
                    feedback_a = motor_a.read_feedback()
                    feedback_b = motor_b.read_feedback()
                    
                    if feedback_a:
                        with viz_lock:
                            leg_states[leg_id]['actual_angles'][0] = np.deg2rad(feedback_a['position'])
                    
                    if feedback_b:
                        with viz_lock:
                            leg_states[leg_id]['actual_angles'][1] = np.deg2rad(feedback_b['position'])
            
            frame = (frame + 1) % TRAJECTORY_STEPS
            
            if frame == 0:
                cycle_count += 1
                if cycle_count % 5 == 0:
                    print(f"  🔄 Gait Cycle #{cycle_count}")
            
            current_time = time.time()
            if current_time - last_status_time >= 2.0:
                last_status_time = current_time
                print(f"  🔄 Gait running - Cycle #{cycle_count}, Frame {frame}/{TRAJECTORY_STEPS}")
            
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
        plot_running = False
        
        # Return to init position with slow S-Curve
        print("\n🏠 Returning to init position (-90°)...")
        try:
            for leg_id in leg_motors.keys():
                motor_a = leg_motors[leg_id]['A']
                motor_b = leg_motors[leg_id]['B']
                motor_a.set_position_slow(MOTOR_INIT_ANGLE)
                motor_b.set_position_slow(MOTOR_INIT_ANGLE)
            time.sleep(3.5)
        except:
            pass
        
        print("\n📊 Communication Statistics:")
        for motor_id, controller in motor_registry.items():
            stats = controller.get_stats()
            print(f"    Motor {motor_id}: TX={stats['tx']}, RX={stats['rx']}, "
                  f"Errors={stats['errors']}, Success={stats['success_rate']:.1f}%")
        
        print("\n🔌 Disconnecting motors...")
        for controller in motor_registry.values():
            controller.disconnect()
        
        print("\n✅ Quadruped gait control terminated successfully")
        print("="*70)

if __name__ == "__main__":
    main()
