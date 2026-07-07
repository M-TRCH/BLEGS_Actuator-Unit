"""
Standalone Walk Test - Modes 7, 8, 9 Only
Author: M-TRCH
Date: June 5, 2026

Standalone version of relative_position_control.py trimmed to:
    [7] Smooth walk +600mm with march transitions
    [8] Turn LEFT while walking +300mm (march -> turn -> march)
    [9] Turn RIGHT while walking +300mm (march -> turn -> march)

All external module dependencies are inlined.  No imports from the
navigation/, control/, or lib/ packages are required.

Log columns (4 rows per control cycle, one per leg):
    time, phase, leg,
    theta1_setpoint_deg, theta1_actual_deg,
    theta2_setpoint_deg, theta2_actual_deg,
    current_A_mA, current_B_mA
"""

import numpy as np
import time
import threading
import sys
import os
import json
import serial
import serial.tools.list_ports
import struct
from enum import IntEnum
from itertools import combinations_with_replacement
from typing import List, Tuple, Optional, Dict

if sys.platform == 'win32':
    import msvcrt

# ============================================================================
# ROBOT / GAIT CONFIGURATION
# ============================================================================

L_AC = 105.0
L_BD = 105.0
L_CE = 145.0
L_DE = 145.0
GEAR_RATIO = 8.0

BODY_LENGTH = 200.0
BODY_WIDTH = 170.0
MOTOR_SPACING = 85.0
MOTOR_INIT_ANGLE = -90.0

DEFAULT_STANCE_HEIGHT = -220.0
DEFAULT_STANCE_OFFSET_X = 0.0

GAIT_LIFT_HEIGHT = 40.0     # old: 15.0
GAIT_STEP_FORWARD = 35.0    # old: 25.0
UPDATE_RATE = 50
TRAJECTORY_STEPS = 30
SMOOTH_TROT_STANCE_RATIO = 0.75

NAV_V_MAX = 70.0
NAV_K_P = 1.0
NAV_TOLERANCE = 10.0
NAV_TIMEOUT = 60.0

VELOCITY_CALIBRATION = 3.04

IMU_PORT = 'COM22'
IMU_ENABLED = True
YAW_K_P = 0.8
YAW_K_D = 0.01
YAW_MAX_CORRECTION = 15.0

TURN_V_MAX = 40.0
TURN_BIAS = 7.5

BALANCE_ENABLED = False
ROLL_K_P = 0.8
ROLL_K_D = 0.03
PITCH_K_P = 0.8
PITCH_K_D = 0.03
MAX_HEIGHT_OFFSET = 20.0
INVERT_ROLL = False
INVERT_PITCH = True
STATIC_ROLL_TRIM_MM = -10.0

GAIT_CYCLE_TIME = TRAJECTORY_STEPS / UPDATE_RATE

# ------------------------- Test distances (editable) -----------------------
# Change these values to adjust test distances for modes 7, 8, 9.
MODE7_DISTANCE_MM = 2400.0
MODE_TURN_DISTANCE_MM = 300.0

ENABLE_LOGGING = True
# Store logs under the test_3 output folder
LOG_FILE_PATH = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output', 'log'))
LOG_RATE = 10

SIMULATION_MODE = False

DEBUG_GAIT = False

ML_COMPENSATION_ENABLED = False
COMPENSATION_MODEL_DIR = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'test_2', 'output', 'models'))
COMPENSATION_MODEL_NAME = 'model_poly4'
ML_COMPENSATION_GAIN = 0.25
ML_COMPENSATION_ALPHA = 0.05
ML_COMPENSATION_MAX_DELTA_MM = 0.4
ML_COMPENSATION_MAX_ABS_MM = 5.0

# ============================================================================
# PROTOCOL CONSTANTS
# ============================================================================

HEADER_1 = 0xFE
HEADER_2 = 0xEE

BAUD_RATE = 921600
SERIAL_TIMEOUT = 0.2
FAST_TIMEOUT = 0.01
PING_RETRIES = 5
DEBUG_SERIAL = False

PROTOCOL_HEADER1 = 0xFE
PROTOCOL_HEADER2 = 0xEE
FB_IMU_DATA = 0x85
FB_IMU_CALIBRATION = 0x87
CMD_SET_ZERO = 0x06
IMU_STATUS_CALIBRATED = 0x01
IMU_STATUS_ERROR = 0x80


class PacketType(IntEnum):
    PKT_CMD_SET_GOAL = 0x01
    PKT_CMD_PING = 0x03
    PKT_CMD_EMERGENCY_STOP = 0x04
    PKT_FB_STATUS = 0x81
    PKT_FB_ERROR = 0x83


class ControlMode(IntEnum):
    MODE_DIRECT_POSITION = 0x00
    MODE_SCURVE_PROFILE = 0x01
    MODE_SCURVE_FULL = 0x02


class StatusFlags(IntEnum):
    STATUS_MOVING = (1 << 0)
    STATUS_ERROR = (1 << 1)
    STATUS_AT_GOAL = (1 << 2)
    STATUS_OVERHEAT = (1 << 3)
    STATUS_OVERCURRENT = (1 << 4)
    STATUS_ENCODER_ERROR = (1 << 5)
    STATUS_EMERGENCY_STOPPED = (1 << 6)


CONTROL_MODE = ControlMode.MODE_DIRECT_POSITION

EXPECTED_MOTOR_IDS = {
    'FL': {'A': 1, 'B': 2},
    'FR': {'A': 3, 'B': 4},
    'RL': {'A': 5, 'B': 6},
    'RR': {'A': 7, 'B': 8},
}

P_A_LEFT = np.array([-MOTOR_SPACING / 2, 0.0])
P_B_LEFT = np.array([MOTOR_SPACING / 2, 0.0])
P_A_RIGHT = np.array([MOTOR_SPACING / 2, 0.0])
P_B_RIGHT = np.array([-MOTOR_SPACING / 2, 0.0])

MOTOR_INIT_ANGLE_RAD = np.deg2rad(MOTOR_INIT_ANGLE)

# ============================================================================
# GLOBAL MUTABLE STATE  (mirrors tqc.* in the original)
# ============================================================================

viz_lock = threading.Lock()
control_lock = threading.Lock()

leg_states = {
    'FR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD],
           'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD],
           'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'red'},
    'FL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD],
           'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD],
           'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'blue'},
    'RR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD],
           'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD],
           'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'orange'},
    'RL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD],
           'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD],
           'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'green'},
}

motor_registry: dict = {}
leg_motors: dict = {}

# ============================================================================
# PROTOCOL HELPERS
# ============================================================================

def calculate_crc16(data: bytes) -> int:
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
    header = bytes([HEADER_1, HEADER_2])
    type_byte = bytes([pkt_type])
    payload_len = bytes([len(payload)])
    crc_data = type_byte + payload_len + payload
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    return header + type_byte + payload_len + payload + crc_bytes


def open_serial_with_timeout(port, baudrate, timeout, open_timeout=3.0):
    result = {'serial': None, 'error': None}

    def try_open():
        try:
            ser = serial.Serial(
                port=port, baudrate=baudrate, timeout=timeout,
                write_timeout=timeout,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, dsrdtr=False, rtscts=False)
            ser.dtr = False
            ser.rts = False
            result['serial'] = ser
        except Exception as e:
            result['error'] = str(e)

    thread = threading.Thread(target=try_open, daemon=True)
    thread.start()
    thread.join(timeout=open_timeout)
    if thread.is_alive():
        return None, f"Timeout opening port (>{open_timeout}s)"
    return result['serial'], result['error']

# ============================================================================
# MOTOR CONTROLLER
# ============================================================================

class BinaryMotorController:
    def __init__(self, port, motor_id=None):
        self.port = port
        self.motor_id = motor_id
        self.serial = None
        self.is_connected = False
        self.current_setpoint = 0.0
        self.current_position = 0.0
        self.current_current = 0   # latest current reading (mA)
        self.current_flags = 0
        self.lock = threading.Lock()
        self.stats_tx_count = 0
        self.stats_rx_count = 0
        self.stats_errors = 0

    def connect(self, timeout=SERIAL_TIMEOUT, open_timeout=3.0):
        try:
            self.serial, error = open_serial_with_timeout(
                self.port, BAUD_RATE, timeout, open_timeout)
            if self.serial is None:
                self.is_connected = False
                return False
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.1)
            self.serial.reset_input_buffer()
            self.is_connected = True
            return True
        except Exception:
            self.is_connected = False
            return False

    def disconnect(self):
        if self.serial and self.is_connected:
            try:
                self.serial.close()
            except Exception:
                pass
            self.is_connected = False

    def set_timeout(self, timeout):
        if self.serial and self.is_connected:
            self.serial.timeout = timeout

    def send_ping(self) -> dict:
        if not self.is_connected:
            return None
        try:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.02)
            packet = build_packet(PacketType.PKT_CMD_PING)
            self.serial.write(packet)
            self.serial.flush()
            self.stats_tx_count += 1
            start_wait = time.time()
            max_wait = max(0.3, self.serial.timeout or 0.3)
            while (time.time() - start_wait) < max_wait:
                if self.serial.in_waiting >= 2:
                    break
                time.sleep(0.005)
            if self.serial.in_waiting < 2:
                return None
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
                    return {'motor_id': motor_id, 'position': self.current_position,
                            'current': current_raw, 'flags': status_flags}
            return None
        except Exception:
            return None

    def start_motor(self) -> bool:
        for _ in range(PING_RETRIES):
            result = self.send_ping()
            if result:
                time.sleep(0.1)
                return True
            time.sleep(0.1)
        return False

    def set_position_direct(self, angle_deg: float) -> bool:
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
        except Exception:
            self.stats_errors += 1
            return False

    def set_position_scurve(self, angle_deg: float, duration_ms: int) -> bool:
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
        except Exception:
            self.stats_errors += 1
            return False

    def send_emergency_stop(self) -> bool:
        if not self.is_connected:
            return False
        try:
            packet = build_packet(PacketType.PKT_CMD_EMERGENCY_STOP)
            self.serial.write(packet)
            self.stats_tx_count += 1
            return True
        except Exception:
            return False

    def _read_packet(self) -> tuple:
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
            if received_crc != calculate_crc16(crc_data):
                self.stats_errors += 1
                return None
            self.stats_rx_count += 1
            return (pkt_type, payload)
        except Exception:
            return None

# ============================================================================
# MOTOR DISCOVERY / REGISTRATION
# ============================================================================

def discover_motors() -> dict:
    print("\n" + "=" * 70)
    print("  MOTOR DISCOVERY")
    print("=" * 70)
    ports = [p for p in serial.tools.list_ports.comports()]
    if not ports:
        print("  No COM ports found!")
        return {}
    print(f"\n  Found {len(ports)} COM port(s):")
    for p in ports:
        print(f"    • {p.device}: {p.description}")
    discovered = {}
    for port_info in ports:
        port = port_info.device
        controller = BinaryMotorController(port)
        if not controller.connect(timeout=SERIAL_TIMEOUT):
            continue
        motor_info = None
        for attempt in range(PING_RETRIES):
            motor_info = controller.send_ping()
            if motor_info:
                break
            time.sleep(0.1)
        if motor_info:
            motor_id = motor_info['motor_id']
            print(f"      Motor ID {motor_id} on {port}")
            discovered[motor_id] = controller
            time.sleep(0.2)
        else:
            controller.disconnect()
        time.sleep(0.1)
    if discovered:
        time.sleep(0.5)
    print(f"\n  Discovery complete: {len(discovered)} motor(s) found")
    return discovered


def register_leg_motors(discovered_motors: dict) -> bool:
    global leg_motors, motor_registry
    motor_registry = discovered_motors
    leg_motors = {}
    all_assigned = True
    for leg_id, motor_ids in EXPECTED_MOTOR_IDS.items():
        motor_a = discovered_motors.get(motor_ids['A'])
        motor_b = discovered_motors.get(motor_ids['B'])
        if motor_a and motor_b:
            leg_motors[leg_id] = {'A': motor_a, 'B': motor_b,
                                  'motor_a_id': motor_ids['A'],
                                  'motor_b_id': motor_ids['B']}
            print(f"    {leg_id}: Motor A (ID {motor_ids['A']}) + Motor B (ID {motor_ids['B']})")
            time.sleep(0.1)
        else:
            all_assigned = False
            print(f"    {leg_id}: INCOMPLETE")
    if leg_motors:
        time.sleep(0.5)
    return all_assigned


def start_all_motors() -> bool:
    global motor_registry
    success = 0
    for motor_id, controller in motor_registry.items():
        controller.set_timeout(FAST_TIMEOUT)
        time.sleep(0.1)
        result = controller.send_ping()
        if result:
            success += 1
        time.sleep(0.2)
    print(f"\n  Started {success}/{len(motor_registry)} motors")
    if success > 0:
        print("  Waiting for motor initialization (3 seconds)...")
        time.sleep(3.0)
    return success == len(motor_registry)


def emergency_stop_all():
    for controller in motor_registry.values():
        controller.send_emergency_stop()

# ============================================================================
# KINEMATICS
# ============================================================================

def get_motor_positions(leg_id):
    if leg_id in ('FL', 'RL'):
        return P_A_LEFT.copy(), P_B_LEFT.copy()
    return P_A_RIGHT.copy(), P_B_RIGHT.copy()


def _solve_circle_intersection(center1, radius1, center2, radius2, choose_lower=True):
    V_12 = center2 - center1
    d = np.linalg.norm(V_12)
    if d > (radius1 + radius2) or d < abs(radius1 - radius2) or d == 0:
        return np.array([np.nan, np.nan])
    a = (radius1 ** 2 - radius2 ** 2 + d ** 2) / (2 * d)
    h_sq = radius1 ** 2 - a ** 2
    if h_sq < 0:
        return np.array([np.nan, np.nan])
    h = np.sqrt(h_sq)
    v_d = V_12 / d
    v_perp = np.array([-v_d[1], v_d[0]])
    p1 = center1 + a * v_d + h * v_perp
    p2 = center1 + a * v_d - h * v_perp
    if choose_lower:
        return p2 if p2[1] < p1[1] else p1
    return p1 if p1[1] > p2[1] else p2


def calculate_ik_no_ef(P_E_target, P_A, P_B,
                       elbow_C_down=True, elbow_D_down=True):
    P_C = _solve_circle_intersection(P_A, L_AC, P_E_target, L_CE, elbow_C_down)
    if np.isnan(P_C).any():
        return np.array([np.nan, np.nan])
    P_D = _solve_circle_intersection(P_B, L_BD, P_E_target, L_DE, elbow_D_down)
    if np.isnan(P_D).any():
        return np.array([np.nan, np.nan])
    V_AC = P_C - P_A
    V_BD = P_D - P_B
    return np.array([np.arctan2(V_AC[1], V_AC[0]),
                     np.arctan2(V_BD[1], V_BD[0])])


def get_gait_phase_offset(leg_id, gait_type='trot'):
    if gait_type == 'trot':
        return {'FR': 0.0, 'FL': 0.5, 'RR': 0.5, 'RL': 0.0}[leg_id]
    return 0.0


def smooth_move_to_home_position(leg_motors_dict, home_angles_dict,
                                 duration_s=3.0, num_steps=50):
    duration_ms = int(duration_s * 1000)
    for leg_id, motors in leg_motors_dict.items():
        if leg_id not in home_angles_dict:
            continue
        target_A, target_B = home_angles_dict[leg_id]
        motors['A'].set_position_scurve(target_A, duration_ms)
        motors['B'].set_position_scurve(target_B, duration_ms)
        with viz_lock:
            leg_states[leg_id]['target_angles'] = [
                np.deg2rad(target_A), np.deg2rad(target_B)]
    print(f"    Waiting for motors to reach home position ({duration_s:.1f}s)...")
    time.sleep(duration_s + 0.5)
    print("    Home position reached!")

# ============================================================================
# BEZIER TRAJECTORY GENERATOR
# ============================================================================

def _cubic_bezier(t: float, P0, P1, P2, P3):
    t = np.clip(t, 0.0, 1.0)
    B0 = (1 - t) ** 3
    B1 = 3 * (1 - t) ** 2 * t
    B2 = 3 * (1 - t) * t ** 2
    B3 = t ** 3
    return B0 * P0 + B1 * P1 + B2 * P2 + B3 * P3


def _bezier_swing(num_steps, start_x, end_x, home_y, lift_height,
                  lift_ratio=0.4, land_ratio=0.6):
    P0 = np.array([start_x, home_y])
    P1 = np.array([start_x + (end_x - start_x) * lift_ratio * 0.5,
                   home_y + lift_height * 1.25])
    P2 = np.array([start_x + (end_x - start_x) * (1.0 - (1.0 - land_ratio) * 0.5),
                   home_y + lift_height * 1.25])
    P3 = np.array([end_x, home_y])
    traj = []
    for i in range(num_steps):
        t = i / (num_steps - 1) if num_steps > 1 else 0.0
        pt = _cubic_bezier(t, P0, P1, P2, P3)
        traj.append((pt[0], pt[1]))
    return traj


def _bezier_stance(num_steps, start_x, end_x, home_y):
    traj = []
    for i in range(num_steps):
        t = i / (num_steps - 1) if num_steps > 1 else 0.0
        traj.append((start_x + (end_x - start_x) * t, home_y))
    return traj


def generate_bezier_trajectory(num_steps=30, lift_height=15.0,
                               step_forward=30.0, mirror_x=False,
                               stance_ratio=0.7, home_x=0.0,
                               home_y=-220.0, reverse=False,
                               lift_ratio=0.4, land_ratio=0.6):
    stance_steps = max(1, int(num_steps * stance_ratio))
    swing_steps = max(1, num_steps - stance_steps)
    step_sign = -1.0 if reverse else 1.0
    sx = home_x + step_forward * step_sign
    ex = home_x - step_forward * step_sign
    ssw_x = ex
    esw_x = sx
    if mirror_x:
        sx, ex, ssw_x, esw_x = (-sx + 2 * home_x, -ex + 2 * home_x,
                                 -ssw_x + 2 * home_x, -esw_x + 2 * home_x)
    stance = _bezier_stance(stance_steps, sx, ex, home_y)
    swing = _bezier_swing(swing_steps, ssw_x, esw_x, home_y,
                          lift_height, lift_ratio, land_ratio)
    return stance + swing

# ============================================================================
# IMU READER
# ============================================================================

class IMUReader:
    def __init__(self, port: str, baud_rate: int = 921600):
        self.port = port
        self.baud_rate = baud_rate
        self._serial = None
        self._lock = threading.Lock()
        self._yaw = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._calibrated = False
        self._error = False
        self._last_update_time = None
        self._packet_count = 0
        self._crc_errors = 0
        self._connected = False
        self._running = False
        self._read_thread = None
        self._first_packet_received = False

    def connect(self) -> bool:
        try:
            available = [p.device for p in serial.tools.list_ports.comports()]
            if self.port not in available:
                print(f"  IMU port {self.port} not found")
                self._connected = False
                return False
            self._serial = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            time.sleep(0.5)
            self._serial.reset_input_buffer()
            self._connected = True
            self._running = True
            self._read_thread = threading.Thread(
                target=self._read_loop, daemon=True)
            self._read_thread.start()
            return True
        except Exception as e:
            print(f"  IMU connect error: {e}")
            self._connected = False
            return False

    def disconnect(self):
        self._running = False
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._connected = False

    def set_zero(self) -> bool:
        if not self._connected or not self._serial:
            return False
        try:
            buf = bytearray([PROTOCOL_HEADER1, PROTOCOL_HEADER2, CMD_SET_ZERO, 0])
            crc = self._crc16(buf[2:4])
            buf.append((crc >> 8) & 0xFF)
            buf.append(crc & 0xFF)
            self._serial.write(buf)
            self._serial.flush()
            return True
        except Exception:
            return False

    def get_yaw(self) -> float:
        with self._lock:
            return self._yaw

    def get_orientation(self) -> Dict[str, float]:
        with self._lock:
            return {'roll': self._roll, 'pitch': self._pitch, 'yaw': self._yaw,
                    'calibrated': self._calibrated, 'error': self._error}

    def is_calibrated(self) -> bool:
        with self._lock:
            return self._calibrated

    def is_connected(self) -> bool:
        return self._connected and self._serial is not None and self._serial.is_open

    def is_receiving_data(self) -> bool:
        with self._lock:
            if self._last_update_time is None:
                return False
            return (time.time() - self._last_update_time) < 0.5

    def get_stats(self) -> Dict:
        with self._lock:
            return {'packets': self._packet_count, 'crc_errors': self._crc_errors,
                    'connected': self.is_connected(),
                    'receiving_data': self.is_receiving_data()}

    def _read_loop(self):
        while self._running:
            try:
                result = self._receive_packet(timeout=0.02)
                if result:
                    pkt_type, payload = result
                    if pkt_type == FB_IMU_DATA:
                        self._parse_imu_data(payload)
            except Exception:
                pass
            time.sleep(0.001)

    def _receive_packet(self, timeout=0.02):
        if not self._serial or not self._serial.is_open:
            return None
        start = time.time()
        while time.time() - start < timeout:
            if self._serial.in_waiting > 0:
                b1 = self._serial.read(1)
                if not b1:
                    continue
                if b1[0] == PROTOCOL_HEADER1:
                    b2 = self._serial.read(1)
                    if b2 and b2[0] == PROTOCOL_HEADER2:
                        pb = self._serial.read(1)
                        lb = self._serial.read(1)
                        if not pb or not lb:
                            continue
                        pkt_type = pb[0]
                        plen = lb[0]
                        payload = self._serial.read(plen)
                        if len(payload) != plen:
                            continue
                        crc_bytes = self._serial.read(2)
                        if len(crc_bytes) != 2:
                            continue
                        recv_crc = (crc_bytes[0] << 8) | crc_bytes[1]
                        calc_crc = self._crc16(
                            bytes([pkt_type, plen]) + payload)
                        if recv_crc != calc_crc:
                            with self._lock:
                                self._crc_errors += 1
                            continue
                        return (pkt_type, payload)
        return None

    def _parse_imu_data(self, payload: bytes):
        if len(payload) != 10:
            return
        try:
            _, roll, pitch, yaw, _, status = struct.unpack('>BhhhHB', payload)
            with self._lock:
                self._roll = roll / 100.0
                self._pitch = pitch / 100.0
                self._yaw = yaw / 100.0
                self._calibrated = bool(status & IMU_STATUS_CALIBRATED)
                self._error = bool(status & IMU_STATUS_ERROR)
                self._last_update_time = time.time()
                self._packet_count += 1
                if not self._first_packet_received:
                    self._first_packet_received = True
                    print(f"  First IMU packet (Yaw: {self._yaw:+.1f}°)")
        except Exception:
            pass

    @staticmethod
    def _crc16(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc


def create_imu_reader(port: str = 'COM22',
                      auto_connect: bool = True) -> Optional[IMUReader]:
    reader = IMUReader(port)
    if auto_connect:
        if not reader.connect():
            return None
    return reader

# ============================================================================
# NAVIGATION: SIMPLE PLANNER
# ============================================================================

class SimpleNavigationPlanner:
    def __init__(self, v_max=50.0, K_p=1.0, tolerance=10.0):
        self.target_y = 0.0
        self.current_y = 0.0
        self.v_max = v_max
        self.K_p = K_p
        self.tolerance = tolerance
        self._is_active = False

    def set_relative_target(self, delta_y: float):
        self.target_y = delta_y
        self.current_y = 0.0
        self._is_active = True
        print(f"  Navigation target set: {delta_y:+.1f} mm")

    def update_position(self, position_y: float):
        self.current_y = position_y

    def compute_velocity(self) -> float:
        if not self._is_active:
            return 0.0
        e_y = self.target_y - self.current_y
        v_mag = min(self.v_max, self.K_p * abs(e_y))
        return float(v_mag * np.sign(e_y)) if e_y != 0 else 0.0

    def get_error(self) -> float:
        return self.target_y - self.current_y

    def get_progress(self) -> float:
        if self.target_y == 0:
            return 100.0
        return max(0.0, min(100.0, (self.current_y / self.target_y) * 100.0))

    def is_target_reached(self) -> bool:
        if not self._is_active:
            return True
        return abs(self.target_y - self.current_y) < self.tolerance

    def get_status(self) -> dict:
        return {'active': self._is_active, 'target_y': self.target_y,
                'current_y': self.current_y, 'error': self.get_error(),
                'progress': self.get_progress(),
                'target_reached': self.is_target_reached()}

    def __repr__(self):
        return (f"SimpleNavigationPlanner(v_max={self.v_max}, K_p={self.K_p}, "
                f"tolerance={self.tolerance}, active={self._is_active})")

# ============================================================================
# NAVIGATION: YAW CONTROLLER
# ============================================================================

class YawController:
    def __init__(self, K_p=0.5, K_d=0.1, max_correction=10.0):
        self.K_p = K_p
        self.K_d = K_d
        self.max_correction = max_correction
        self.target_yaw = 0.0
        self._last_error = 0.0
        self._last_update_time = None

    def set_target(self, yaw_deg: float):
        self.target_yaw = yaw_deg

    def reset(self):
        self.target_yaw = 0.0
        self._last_error = 0.0
        self._last_update_time = None

    def compute(self, current_yaw: float, current_time: float = None) -> float:
        if current_time is None:
            current_time = time.time()
        error = self.target_yaw - current_yaw
        p_term = self.K_p * error
        d_term = 0.0
        if self._last_update_time is not None:
            dt = current_time - self._last_update_time
            if dt > 0:
                d_term = self.K_d * (error - self._last_error) / dt
        correction = p_term + d_term
        correction = max(-self.max_correction,
                         min(self.max_correction, correction))
        self._last_error = error
        self._last_update_time = current_time
        return correction

# ============================================================================
# NAVIGATION: TIME-BASED ESTIMATOR
# ============================================================================

class TimeBasedEstimator:
    def __init__(self, imu_reader=None):
        self.position_y = 0.0
        self.velocity_y = 0.0
        self.total_distance = 0.0
        self._last_update_time = None
        self._start_time = None
        self._update_count = 0
        self._imu_reader = imu_reader
        self.yaw = 0.0
        self.target_yaw = 0.0
        self._imu_available = (imu_reader is not None)

    def start(self):
        self.reset()
        self._start_time = time.time()
        self._last_update_time = self._start_time
        if self._imu_available and self._imu_reader.is_connected():
            self.target_yaw = self._imu_reader.get_yaw()
            print(f"  State estimator started (IMU yaw: {self.target_yaw:.1f}°)")
        else:
            print("  State estimator started (no IMU)")

    def update(self, v_body_y: float, current_time: float = None):
        if current_time is None:
            current_time = time.time()
        if self._last_update_time is not None:
            dt = current_time - self._last_update_time
            disp = v_body_y * dt
            self.position_y += disp
            self.total_distance += abs(disp)
        if self._imu_available and self._imu_reader.is_connected():
            self.yaw = self._imu_reader.get_yaw()
        self.velocity_y = v_body_y
        self._last_update_time = current_time
        self._update_count += 1

    def reset(self):
        self.position_y = 0.0
        self.velocity_y = 0.0
        self.total_distance = 0.0
        self._last_update_time = None
        self._start_time = None
        self._update_count = 0
        if self._imu_available and self._imu_reader and self._imu_reader.is_connected():
            self.target_yaw = self._imu_reader.get_yaw()
            self.yaw = self.target_yaw

    def get_position(self) -> float:
        return self.position_y

    def get_elapsed_time(self) -> float:
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time

    def get_average_velocity(self) -> float:
        elapsed = self.get_elapsed_time()
        return self.position_y / elapsed if elapsed > 0 else 0.0

    def get_yaw(self) -> float:
        return self.yaw

    def get_yaw_error(self) -> float:
        return self.target_yaw - self.yaw

    def has_imu(self) -> bool:
        return (self._imu_available and
                self._imu_reader is not None and
                self._imu_reader.is_connected())

    def get_status(self) -> dict:
        s = {'position_y': self.position_y, 'velocity_y': self.velocity_y,
             'elapsed_time': self.get_elapsed_time(),
             'average_velocity': self.get_average_velocity()}
        if self._imu_available:
            s['yaw'] = self.yaw
            s['target_yaw'] = self.target_yaw
            s['yaw_error'] = self.get_yaw_error()
        return s

    def __repr__(self):
        return (f"TimeBasedEstimator(position_y={self.position_y:.2f}mm, "
                f"velocity_y={self.velocity_y:.2f}mm/s)")

# ============================================================================
# BALANCE CONTROLLER
# ============================================================================

class BalanceController:
    def __init__(self,
                 roll_Kp=ROLL_K_P, roll_Kd=ROLL_K_D,
                 pitch_Kp=PITCH_K_P, pitch_Kd=PITCH_K_D,
                 max_offset=MAX_HEIGHT_OFFSET):
        self.roll_Kp = roll_Kp
        self.roll_Kd = roll_Kd
        self.pitch_Kp = pitch_Kp
        self.pitch_Kd = pitch_Kd
        self.max_offset = max_offset
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self._prev_roll_error = 0.0
        self._prev_pitch_error = 0.0
        self._prev_time = None
        self.dh_roll = 0.0
        self.dh_pitch = 0.0

    def reset(self):
        self._prev_roll_error = 0.0
        self._prev_pitch_error = 0.0
        self._prev_time = None
        self.dh_roll = 0.0
        self.dh_pitch = 0.0

    def set_target(self, roll_deg=0.0, pitch_deg=0.0):
        self.target_roll = roll_deg
        self.target_pitch = pitch_deg

    def compute(self, current_roll: float, current_pitch: float,
                current_time: float = None) -> dict:
        if current_time is None:
            current_time = time.time()
        roll_error = current_roll - self.target_roll
        pitch_error = current_pitch - self.target_pitch
        if INVERT_ROLL:
            roll_error = -roll_error
        if INVERT_PITCH:
            pitch_error = -pitch_error
        d_roll = d_pitch = 0.0
        if self._prev_time is not None:
            dt = current_time - self._prev_time
            if dt > 0:
                d_roll = (roll_error - self._prev_roll_error) / dt
                d_pitch = (pitch_error - self._prev_pitch_error) / dt
        self.dh_roll = np.clip(self.roll_Kp * roll_error + self.roll_Kd * d_roll,
                               -self.max_offset, self.max_offset)
        self.dh_pitch = np.clip(self.pitch_Kp * pitch_error + self.pitch_Kd * d_pitch,
                                -self.max_offset, self.max_offset)
        self._prev_roll_error = roll_error
        self._prev_pitch_error = pitch_error
        self._prev_time = current_time
        return {
            'FL': +self.dh_pitch - self.dh_roll,
            'FR': +self.dh_pitch + self.dh_roll,
            'RL': -self.dh_pitch - self.dh_roll,
            'RR': -self.dh_pitch + self.dh_roll,
        }

# ============================================================================
# GLOBAL CONTROL STATE
# ============================================================================

nav_planner: Optional[SimpleNavigationPlanner] = None
state_estimator: Optional[TimeBasedEstimator] = None
imu_reader: Optional[IMUReader] = None
yaw_controller: Optional[YawController] = None
balance_controller: Optional[BalanceController] = None

control_paused = True
idle_marching = False
march_thread: Optional[threading.Thread] = None
march_step_indices = {'FR': 0, 'FL': 0, 'RR': 0, 'RL': 0}

ml_compensation_state = {
    'FR': {'err_x': 0.0, 'err_y': 0.0, 'initialized': False},
    'FL': {'err_x': 0.0, 'err_y': 0.0, 'initialized': False},
    'RR': {'err_x': 0.0, 'err_y': 0.0, 'initialized': False},
    'RL': {'err_x': 0.0, 'err_y': 0.0, 'initialized': False},
}

log_file = None
log_counter = 0
_log_start_time = 0.0
_compensation_model_cache: Optional[dict] = None
_compensation_model_cache_name: Optional[str] = None

# ============================================================================
# GAIT HELPERS
# ============================================================================

def update_gait_from_velocity(v_body_y: float) -> tuple:
    step_length = min(abs(v_body_y) * GAIT_CYCLE_TIME, GAIT_STEP_FORWARD)
    return step_length, (v_body_y < 0)


def get_trajectory_for_velocity(v_body_y: float, leg_id: str,
                                yaw_correction: float = 0.0) -> list:
    step_length, reverse = update_gait_from_velocity(v_body_y)
    is_left = leg_id in ('FL', 'RL')
    step_diff = yaw_correction if is_left else -yaw_correction
    adjusted_step = max(5.0, min(step_length + step_diff, GAIT_STEP_FORWARD * 1.5))
    mirror_x = leg_id in ('FR', 'RR')
    return generate_bezier_trajectory(
        num_steps=TRAJECTORY_STEPS,
        lift_height=GAIT_LIFT_HEIGHT,
        step_forward=adjusted_step,
        mirror_x=mirror_x,
        stance_ratio=SMOOTH_TROT_STANCE_RATIO,
        home_x=DEFAULT_STANCE_OFFSET_X,
        home_y=DEFAULT_STANCE_HEIGHT,
        reverse=not reverse,
    )

# ============================================================================
# ML COMPENSATION
# ============================================================================

def _build_poly_features(
    theta_a_deg: float, theta_b_deg: float, degree: int,
    curr_a: float = 0.0, curr_b: float = 0.0,
    n_vars: int = 2,
) -> np.ndarray:
    vars_ = [theta_a_deg, theta_b_deg, curr_a, curr_b][:n_vars]
    feats = []
    for d in range(1, degree + 1):
        for combo in combinations_with_replacement(range(n_vars), d):
            feat = 1.0
            for idx in combo:
                feat *= vars_[idx]
            feats.append(feat)
    return np.array(feats)


def _list_compensation_models() -> list[str]:
    names: set[str] = set()
    if os.path.isdir(COMPENSATION_MODEL_DIR):
        for filename in os.listdir(COMPENSATION_MODEL_DIR):
            if filename.endswith(('.pkl', '.json')):
                names.add(os.path.splitext(filename)[0])
    return sorted(names)


def load_compensation_model(name: str = COMPENSATION_MODEL_NAME) -> dict | None:
    if os.path.isabs(name) or (os.sep in name) or ('/' in name):
        base = os.path.splitext(name)[0]
    else:
        base = os.path.join(COMPENSATION_MODEL_DIR, os.path.splitext(name)[0])

    pkl_path = base + '.pkl'
    json_path = base + '.json'

    if os.path.isfile(pkl_path):
        try:
            import joblib
            data = joblib.load(pkl_path)
            data['type'] = 'pkl'
            data['model_name'] = data.get('model_name', os.path.basename(pkl_path))
            return data
        except Exception as exc:
            print(f"  ML model load failed ({pkl_path}): {exc}")

    if os.path.isfile(json_path):
        try:
            with open(json_path, 'r', encoding='utf-8') as file_obj:
                data = json.load(file_obj)
            data['type'] = 'json'
            data['model_name'] = data.get('model_name', os.path.basename(json_path))
            return data
        except Exception as exc:
            print(f"  ML model load failed ({json_path}): {exc}")

    print(f"  ML model not found: {os.path.basename(base)}")
    return None


def get_compensation_model(verbose: bool = False) -> dict | None:
    global _compensation_model_cache, _compensation_model_cache_name
    if _compensation_model_cache is not None and \
            _compensation_model_cache_name == COMPENSATION_MODEL_NAME:
        return _compensation_model_cache

    model = load_compensation_model(COMPENSATION_MODEL_NAME)
    if model is None:
        _compensation_model_cache = None
        _compensation_model_cache_name = None
        return None

    _compensation_model_cache = model
    _compensation_model_cache_name = COMPENSATION_MODEL_NAME
    if verbose:
        print(f"  ML compensation ready: {model.get('model_name', COMPENSATION_MODEL_NAME)}")
    return model


def predict_kinematic_error(
    theta_a_deg: float, theta_b_deg: float, model: dict,
    curr_a: float | None = None, curr_b: float | None = None,
) -> tuple[float, float]:
    if curr_a is None or curr_b is None:
        median = model.get('input_stats', {}).get('median', [])
        if curr_a is None:
            curr_a = float(median[2]) if len(median) > 2 else 0.0
        if curr_b is None:
            curr_b = float(median[3]) if len(median) > 3 else 0.0

    if model.get('type') == 'pkl':
        n_feat = getattr(model['model_x'], 'n_features_in_', 2)
        features = [[theta_a_deg, theta_b_deg, curr_a, curr_b]] if n_feat >= 4 \
            else [[theta_a_deg, theta_b_deg]]
        err_x = float(model['model_x'].predict(features)[0])
        err_y = float(model['model_y'].predict(features)[0])
        return err_x, err_y

    degree = model.get('polynomial_degree', 2)
    feature_names = model.get('feature_names', [])
    n_vars = 4 if any('currA' in name or 'currB' in name for name in feature_names) else 2
    phi = _build_poly_features(theta_a_deg, theta_b_deg, degree, curr_a, curr_b, n_vars)
    model_x = model['model_x']
    model_y = model['model_y']
    err_x = model_x['intercept'] + float(np.dot(model_x['coef'], phi))
    err_y = model_y['intercept'] + float(np.dot(model_y['coef'], phi))
    return err_x, err_y


def get_ml_status() -> str:
    if not ML_COMPENSATION_ENABLED:
        return 'OFF'
    model = get_compensation_model(verbose=False)
    if model is None:
        return f'ON (model missing: {COMPENSATION_MODEL_NAME})'
    return f"ON ({model.get('model_name', COMPENSATION_MODEL_NAME)})"


def reset_ml_compensation_state() -> None:
    for leg_state in ml_compensation_state.values():
        leg_state['err_x'] = 0.0
        leg_state['err_y'] = 0.0
        leg_state['initialized'] = False


def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def _filter_ml_compensation(leg_id: str, err_x: float,
                            err_y: float) -> tuple[float, float]:
    state = ml_compensation_state.setdefault(
        leg_id, {'err_x': 0.0, 'err_y': 0.0, 'initialized': False})

    raw_x = _clamp(err_x, -ML_COMPENSATION_MAX_ABS_MM, ML_COMPENSATION_MAX_ABS_MM)
    raw_y = _clamp(err_y, -ML_COMPENSATION_MAX_ABS_MM, ML_COMPENSATION_MAX_ABS_MM)

    if not state['initialized']:
        state['err_x'] = raw_x
        state['err_y'] = raw_y
        state['initialized'] = True
        return raw_x, raw_y

    alpha = _clamp(ML_COMPENSATION_ALPHA, 0.0, 1.0)
    filt_x = alpha * raw_x + (1.0 - alpha) * state['err_x']
    filt_y = alpha * raw_y + (1.0 - alpha) * state['err_y']

    delta_x = _clamp(filt_x - state['err_x'],
                     -ML_COMPENSATION_MAX_DELTA_MM,
                     ML_COMPENSATION_MAX_DELTA_MM)
    delta_y = _clamp(filt_y - state['err_y'],
                     -ML_COMPENSATION_MAX_DELTA_MM,
                     ML_COMPENSATION_MAX_DELTA_MM)

    state['err_x'] = _clamp(state['err_x'] + delta_x,
                            -ML_COMPENSATION_MAX_ABS_MM,
                            ML_COMPENSATION_MAX_ABS_MM)
    state['err_y'] = _clamp(state['err_y'] + delta_y,
                            -ML_COMPENSATION_MAX_ABS_MM,
                            ML_COMPENSATION_MAX_ABS_MM)
    return state['err_x'], state['err_y']


def toggle_ml_compensation() -> bool:
    global ML_COMPENSATION_ENABLED
    if not ML_COMPENSATION_ENABLED:
        model = get_compensation_model(verbose=False)
        if model is None:
            print("  Cannot enable ML compensation: model unavailable")
            names = _list_compensation_models()
            if names:
                print(f"  Available models: {', '.join(names)}")
            else:
                print(f"  No models found in: {COMPENSATION_MODEL_DIR}")
            return False
        reset_ml_compensation_state()
        ML_COMPENSATION_ENABLED = True
        print(f"  ML compensation ENABLED [{model.get('model_name', COMPENSATION_MODEL_NAME)}]")
        return True

    ML_COMPENSATION_ENABLED = False
    reset_ml_compensation_state()
    print("  ML compensation DISABLED")
    return True


def apply_ml_compensation(leg_id: str, x: float, y: float) -> tuple[float, float, bool]:
    if not ML_COMPENSATION_ENABLED:
        return x, y, False

    model = get_compensation_model(verbose=False)
    if model is None:
        return x, y, False

    p_a, p_b = get_motor_positions(leg_id)
    nominal_angles = calculate_ik_no_ef(np.array([x, y]), p_a, p_b)
    if np.isnan(nominal_angles).any():
        return x, y, False

    current_a = None
    current_b = None
    if not SIMULATION_MODE and leg_id in leg_motors:
        motors = leg_motors[leg_id]
        current_a = motors['A'].current_current
        current_b = motors['B'].current_current

    err_x, err_y = predict_kinematic_error(
        np.rad2deg(nominal_angles[0]), np.rad2deg(nominal_angles[1]),
        model, current_a, current_b)
    gain = _clamp(ML_COMPENSATION_GAIN, 0.0, 1.0)
    err_x *= gain
    err_y *= gain
    err_x, err_y = _filter_ml_compensation(leg_id, err_x, err_y)
    return x - err_x, y - err_y, True

# ============================================================================
# LEG CONTROL
# ============================================================================

def send_leg_angles(leg_id: str, theta_A: float, theta_B: float) -> bool:
    angle_A_deg = np.rad2deg(theta_A)
    angle_B_deg = np.rad2deg(theta_B)
    if SIMULATION_MODE or leg_id not in leg_motors:
        return True
    motors = leg_motors[leg_id]
    return motors['A'].set_position_direct(angle_A_deg) and \
           motors['B'].set_position_direct(angle_B_deg)


def update_leg_position(leg_id: str, foot_position: tuple,
                        balance_offset: float = 0.0) -> bool:
    x, y = foot_position
    y_corrected = y + balance_offset
    P_A, P_B = get_motor_positions(leg_id)
    cmd_x, cmd_y, compensated = apply_ml_compensation(leg_id, x, y_corrected)
    angles = calculate_ik_no_ef(np.array([cmd_x, cmd_y]), P_A, P_B)
    if np.isnan(angles).any() and compensated:
        angles = calculate_ik_no_ef(np.array([x, y_corrected]), P_A, P_B)
        cmd_x, cmd_y = x, y_corrected
    if np.isnan(angles).any():
        return False
    theta_A, theta_B = angles
    with viz_lock:
        leg_states[leg_id]['target_angles'] = [theta_A, theta_B]
        leg_states[leg_id]['target_pos'] = [cmd_x, cmd_y]
    return send_leg_angles(leg_id, theta_A, theta_B)


def update_all_legs_gait(trajectories: dict, step_indices: dict,
                         balance_offsets: dict = None) -> bool:
    if balance_offsets is None:
        balance_offsets = {k: 0.0 for k in ('FR', 'FL', 'RR', 'RL')}
    ok = True
    for leg_id in ('FR', 'FL', 'RR', 'RL'):
        if leg_id in trajectories and leg_id in step_indices:
            traj = trajectories[leg_id]
            idx = step_indices[leg_id] % len(traj)
            ok = update_leg_position(leg_id, traj[idx],
                                     balance_offsets.get(leg_id, 0.0)) and ok
    return ok


def stop_all_legs():
    pass  # Motors hold position via PID


def get_static_roll_offsets() -> dict:
    if not ML_COMPENSATION_ENABLED:
        return {leg_id: 0.0 for leg_id in ('FR', 'FL', 'RR', 'RL')}
    trim = STATIC_ROLL_TRIM_MM
    return {
        'FL': +trim,
        'RL': +trim,
        'FR': -trim,
        'RR': -trim,
    }


def combine_balance_offsets(dynamic_offsets: dict = None) -> dict:
    static_offsets = get_static_roll_offsets()
    if dynamic_offsets is None:
        return static_offsets
    return {
        leg_id: static_offsets[leg_id] + dynamic_offsets.get(leg_id, 0.0)
        for leg_id in ('FR', 'FL', 'RR', 'RL')
    }


def move_to_stand_position() -> bool:
    stand_pos = (DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT)
    static_offsets = get_static_roll_offsets()
    for leg_id in ('FR', 'FL', 'RR', 'RL'):
        if not update_leg_position(leg_id, stand_pos, static_offsets[leg_id]):
            return False
    return True

# ============================================================================
# LOGGING
# ============================================================================

def init_logging(target_distance: float) -> bool:
    global log_file, log_counter, _log_start_time
    if not ENABLE_LOGGING:
        return False
    if log_file is not None:
        return True
    try:
        from datetime import datetime
        os.makedirs(LOG_FILE_PATH, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(
            LOG_FILE_PATH,
            f"movement_{target_distance:+.0f}mm_{timestamp}.csv")
        log_file = open(filename, 'w')
        cols = [
            'time', 'phase', 'leg',
            'theta1_setpoint_deg', 'theta1_actual_deg',
            'theta2_setpoint_deg', 'theta2_actual_deg',
            'current_A_mA', 'current_B_mA',
        ]
        log_file.write(','.join(cols) + '\n')
        log_counter = 0
        _log_start_time = time.time()
        print(f"  Logging to: {filename}")
        return True
    except Exception as e:
        print(f"  Logging init failed: {e}")
        return False


def log_control_step(phase: str) -> None:
    global log_file, log_counter, _log_start_time
    if not ENABLE_LOGGING or log_file is None:
        return
    log_counter += 1
    if log_counter % LOG_RATE != 0:
        return
    elapsed = time.time() - _log_start_time
    for leg_id in ('FR', 'FL', 'RR', 'RL'):
        # Setpoint angles
        if leg_id in leg_states:
            angles = leg_states[leg_id].get(
                'target_angles', [float('nan'), float('nan')])
            sp1 = np.rad2deg(angles[0])
            sp2 = np.rad2deg(angles[1])
        else:
            sp1 = sp2 = float('nan')
        # Actual position from encoder feedback
        if not SIMULATION_MODE and leg_id in leg_motors:
            motors = leg_motors[leg_id]
            act1 = motors['A'].current_position
            act2 = motors['B'].current_position
            cur_A = motors['A'].current_current   # mA
            cur_B = motors['B'].current_current   # mA
        else:
            act1 = act2 = float('nan')
            cur_A = cur_B = float('nan')
        log_file.write(
            f"{elapsed:.4f},{phase},{leg_id},"
            f"{sp1:.3f},{act1:.3f},{sp2:.3f},{act2:.3f},"
            f"{cur_A},{cur_B}\n")
    log_file.flush()


def close_logging() -> None:
    global log_file
    if log_file is not None:
        log_file.close()
        log_file = None
        print("  Log file closed")

# ============================================================================
# IDLE MARCHING
# ============================================================================

def _march_loop():
    global idle_marching, march_step_indices
    march_lift = GAIT_LIFT_HEIGHT * 2.0
    march_traj = {}
    for leg_id in ('FR', 'FL', 'RR', 'RL'):
        mirror_x = leg_id in ('FR', 'RR')
        march_traj[leg_id] = generate_bezier_trajectory(
            num_steps=TRAJECTORY_STEPS, lift_height=march_lift,
            step_forward=0.0, mirror_x=mirror_x,
            stance_ratio=SMOOTH_TROT_STANCE_RATIO,
            home_x=DEFAULT_STANCE_OFFSET_X, home_y=DEFAULT_STANCE_HEIGHT,
            reverse=False)
    try:
        while idle_marching:
            t0 = time.time()
            bal = get_static_roll_offsets()
            if BALANCE_ENABLED and balance_controller is not None:
                if imu_reader and imu_reader.is_receiving_data():
                    o = imu_reader.get_orientation()
                    bal = combine_balance_offsets(
                        balance_controller.compute(o['roll'], o['pitch'], t0))
            update_all_legs_gait(march_traj, march_step_indices, bal)
            for leg_id in march_step_indices:
                march_step_indices[leg_id] = (
                    march_step_indices[leg_id] + 1) % TRAJECTORY_STEPS
            log_control_step("MARCH")
            sleep_t = (1.0 / UPDATE_RATE) - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)
    except Exception as e:
        print(f"  March loop error: {e}")


def start_idle_march(resume_from: dict = None) -> bool:
    global idle_marching, march_thread, march_step_indices
    if idle_marching:
        return False
    if resume_from is not None:
        march_step_indices = resume_from.copy()
    else:
        for leg_id in march_step_indices:
            phase_offset = get_gait_phase_offset(leg_id, 'trot')
            march_step_indices[leg_id] = int(phase_offset * TRAJECTORY_STEPS)
    idle_marching = True
    march_thread = threading.Thread(target=_march_loop, daemon=True)
    march_thread.start()
    print("  Idle march started")
    return True


def stop_idle_march() -> bool:
    global idle_marching, march_thread
    if not idle_marching:
        return False
    idle_marching = False
    if march_thread and march_thread.is_alive():
        march_thread.join(timeout=2.0)
    time.sleep(0.2)
    move_to_stand_position()
    print("  Idle march stopped")
    return True

# ============================================================================
# MOVE RELATIVE Y
# ============================================================================

def move_relative_y(target_distance_mm: float, timeout_s: float = NAV_TIMEOUT,
                    transition_to_march: bool = False) -> bool:
    global nav_planner, state_estimator, idle_marching, march_thread
    global march_step_indices, balance_controller, control_paused

    print("\n" + "=" * 70)
    print(f"  RELATIVE POSITION CONTROL  Target: {target_distance_mm:+.1f} mm")
    print("=" * 70)

    transitioning = idle_marching
    if transitioning:
        idle_marching = False
        if march_thread and march_thread.is_alive():
            march_thread.join(timeout=1.0)

    if nav_planner is None:
        nav_planner = SimpleNavigationPlanner(
            v_max=NAV_V_MAX, K_p=NAV_K_P, tolerance=NAV_TOLERANCE)
    if state_estimator is None:
        state_estimator = TimeBasedEstimator(imu_reader=imu_reader)
    if BALANCE_ENABLED and balance_controller is None:
        balance_controller = BalanceController()

    nav_planner.set_relative_target(target_distance_mm)
    state_estimator.start()

    if yaw_controller is not None and state_estimator.has_imu():
        yaw_controller.set_target(state_estimator.get_yaw())
    if BALANCE_ENABLED and balance_controller is not None:
        balance_controller.reset()
        balance_controller.set_target(0.0, 0.0)

    log_was_open = (log_file is not None)
    init_logging(target_distance_mm)

    start_time = time.time()
    last_status_time = start_time

    if transitioning:
        step_indices = march_step_indices.copy()
    else:
        step_indices = {}
        for leg_id in ('FR', 'FL', 'RR', 'RL'):
            phase_offset = get_gait_phase_offset(leg_id, 'trot')
            step_indices[leg_id] = int(phase_offset * TRAJECTORY_STEPS)

    control_paused = False

    try:
        while not nav_planner.is_target_reached():
            t_now = time.time()
            elapsed = t_now - start_time
            if elapsed > timeout_s:
                print(f"\n  Timeout reached! ({timeout_s:.1f}s)")
                return False

            if sys.platform == 'win32' and msvcrt.kbhit():
                key = msvcrt.getch()
                if key == b' ':
                    control_paused = not control_paused
                    print("\n  PAUSED" if control_paused else "\n  RESUMED")
                elif key.lower() == b'e':
                    print("\n  EMERGENCY STOP!")
                    emergency_stop_all()
                    return False
                elif key.lower() == b'q':
                    print("\n  ABORTED by user")
                    return False

            if control_paused:
                time.sleep(0.05)
                continue

            v_body_y = nav_planner.compute_velocity()

            yaw_correction = 0.0
            if yaw_controller is not None and state_estimator.has_imu():
                yaw_correction = yaw_controller.compute(
                    state_estimator.get_yaw(), t_now)

            trajectories = {leg: get_trajectory_for_velocity(
                v_body_y, leg, yaw_correction)
                for leg in ('FR', 'FL', 'RR', 'RL')}

            bal = None
            if BALANCE_ENABLED and balance_controller is not None:
                if imu_reader and imu_reader.is_receiving_data():
                    o = imu_reader.get_orientation()
                    bal = combine_balance_offsets(balance_controller.compute(
                        o['roll'], o['pitch'], t_now))
                else:
                    bal = combine_balance_offsets()
            else:
                bal = combine_balance_offsets()

            update_all_legs_gait(trajectories, step_indices, bal)

            for leg_id in step_indices:
                step_indices[leg_id] = (
                    step_indices[leg_id] + 1) % TRAJECTORY_STEPS

            state_estimator.update(v_body_y * VELOCITY_CALIBRATION, t_now)
            nav_planner.update_position(state_estimator.get_position())
            status = nav_planner.get_status()
            log_control_step("WALK")

            if t_now - last_status_time >= 0.5:
                print(f"  Pos: {status['current_y']:+.1f}/{status['target_y']:+.1f} mm"
                      f" | {status['progress']:.0f}% | v={v_body_y:+.1f} mm/s"
                      f" | t={elapsed:.1f}s")
                last_status_time = t_now

            loop_dur = time.time() - t_now
            sleep_t = (1.0 / UPDATE_RATE) - loop_dur
            if sleep_t > 0:
                time.sleep(sleep_t)

        if transition_to_march:
            march_step_indices.update(step_indices)
        else:
            stop_all_legs()

        print(f"\n  TARGET REACHED! Final={state_estimator.get_position():+.1f} mm"
              f"  Time={state_estimator.get_elapsed_time():.2f}s")
        return True

    except KeyboardInterrupt:
        print("\n  Interrupted by user")
        return False
    finally:
        stop_all_legs()
        if not log_was_open:
            close_logging()

# ============================================================================
# MOVE WITH TURN
# ============================================================================

def move_relative_y_with_turn(target_distance_mm: float, turn_bias: float,
                               timeout_s: float = NAV_TIMEOUT,
                               transition_to_march: bool = False,
                               v_max: float = TURN_V_MAX) -> bool:
    global nav_planner, state_estimator, idle_marching, march_thread
    global march_step_indices, balance_controller, control_paused

    direction_str = "RIGHT" if turn_bias > 0 else "LEFT"
    print("\n" + "=" * 70)
    print(f"  TURNING WALK  Target: {target_distance_mm:+.1f} mm  Turn: {direction_str}"
          f"  Bias: {turn_bias:+.1f} mm")
    print("=" * 70)

    transitioning = idle_marching
    if transitioning:
        idle_marching = False
        if march_thread and march_thread.is_alive():
            march_thread.join(timeout=1.0)

    nav_planner = SimpleNavigationPlanner(
        v_max=v_max, K_p=NAV_K_P, tolerance=NAV_TOLERANCE)
    if state_estimator is None:
        state_estimator = TimeBasedEstimator(imu_reader=imu_reader)
    if BALANCE_ENABLED and balance_controller is None:
        balance_controller = BalanceController()

    nav_planner.set_relative_target(target_distance_mm)
    state_estimator.start()

    if yaw_controller is not None and state_estimator.has_imu():
        yaw_controller.set_target(state_estimator.get_yaw())
    if BALANCE_ENABLED and balance_controller is not None:
        balance_controller.reset()
        balance_controller.set_target(0.0, 0.0)

    log_was_open = (log_file is not None)
    init_logging(target_distance_mm)

    start_time = time.time()
    last_status_time = start_time

    if transitioning:
        step_indices = march_step_indices.copy()
    else:
        step_indices = {}
        for leg_id in ('FR', 'FL', 'RR', 'RL'):
            step_indices[leg_id] = int(
                get_gait_phase_offset(leg_id, 'trot') * TRAJECTORY_STEPS)

    control_paused = False

    try:
        while not nav_planner.is_target_reached():
            t_now = time.time()
            elapsed = t_now - start_time
            if elapsed > timeout_s:
                print(f"\n  Timeout reached! ({timeout_s:.1f}s)")
                return False

            if sys.platform == 'win32' and msvcrt.kbhit():
                key = msvcrt.getch()
                if key == b' ':
                    control_paused = not control_paused
                    print("\n  PAUSED" if control_paused else "\n  RESUMED")
                elif key.lower() == b'e':
                    print("\n  EMERGENCY STOP!")
                    emergency_stop_all()
                    return False
                elif key.lower() == b'q':
                    print("\n  ABORTED by user")
                    return False

            if control_paused:
                time.sleep(0.05)
                continue

            v_body_y = nav_planner.compute_velocity()

            yaw_correction = turn_bias
            if yaw_controller is not None and state_estimator.has_imu():
                yaw_correction += yaw_controller.compute(
                    state_estimator.get_yaw(), t_now)

            trajectories = {leg: get_trajectory_for_velocity(
                v_body_y, leg, yaw_correction)
                for leg in ('FR', 'FL', 'RR', 'RL')}

            bal = None
            if BALANCE_ENABLED and balance_controller is not None:
                if imu_reader and imu_reader.is_receiving_data():
                    o = imu_reader.get_orientation()
                    bal = combine_balance_offsets(balance_controller.compute(
                        o['roll'], o['pitch'], t_now))
                else:
                    bal = combine_balance_offsets()
            else:
                bal = combine_balance_offsets()

            update_all_legs_gait(trajectories, step_indices, bal)

            for leg_id in step_indices:
                step_indices[leg_id] = (
                    step_indices[leg_id] + 1) % TRAJECTORY_STEPS

            state_estimator.update(v_body_y * VELOCITY_CALIBRATION, t_now)
            nav_planner.update_position(state_estimator.get_position())
            status = nav_planner.get_status()
            log_control_step("TURN")

            if t_now - last_status_time >= 0.5:
                print(f"  Pos: {status['current_y']:+.1f}/{status['target_y']:+.1f} mm"
                      f" | {status['progress']:.0f}% | {direction_str}"
                      f" | t={elapsed:.1f}s")
                last_status_time = t_now

            loop_dur = time.time() - t_now
            sleep_t = (1.0 / UPDATE_RATE) - loop_dur
            if sleep_t > 0:
                time.sleep(sleep_t)

        if transition_to_march:
            march_step_indices.update(step_indices)
        else:
            stop_all_legs()

        print(f"\n  TARGET REACHED ({direction_str})! "
              f"Final={state_estimator.get_position():+.1f} mm")
        return True

    except KeyboardInterrupt:
        print("\n  Interrupted by user")
        return False
    finally:
        nav_planner = SimpleNavigationPlanner(
            v_max=NAV_V_MAX, K_p=NAV_K_P, tolerance=NAV_TOLERANCE)
        stop_all_legs()
        if not log_was_open:
            close_logging()

# ============================================================================
# TEST FUNCTIONS  (modes 7, 8, 9)
# ============================================================================

def test_smooth_walk_600() -> bool:
    """Mode 7: Smooth walk +600mm with march transitions."""
    print("\n" + "=" * 70)
    print(f"  MODE 7: Smooth Walk {MODE7_DISTANCE_MM:+.0f}mm")
    print("=" * 70)
    init_logging(MODE7_DISTANCE_MM)
    try:
        print("\n  Step 1/4: Starting idle march...")
        if not start_idle_march():
            print("  Failed to start idle march")
            return False
        print("  Marching in place for 2 seconds...")
        time.sleep(2.0)

        print(f"\n  Step 2/4: Walking forward {MODE7_DISTANCE_MM:+.0f}mm...")
        success = move_relative_y(+MODE7_DISTANCE_MM, timeout_s=60.0,
                      transition_to_march=True)
        if not success:
            if idle_marching:
                stop_idle_march()
            return False

        print("\n  Step 3/4: Transitioning to idle march...")
        if not start_idle_march(resume_from=march_step_indices):
            print("  Failed to start idle march")
            return False
        print("  Marching in place for 2 seconds...")
        time.sleep(2.0)

        print("\n  Step 4/4: Returning to standing position...")
        if not stop_idle_march():
            print("  Failed to stop idle march")
            return False

        print("\n  MODE 7 COMPLETED!")
        return True
    except Exception as e:
        print(f"\n  Test failed: {e}")
        if idle_marching:
            stop_idle_march()
        return False
    finally:
        close_logging()


def _test_turn(direction: str, turn_bias: float) -> bool:
    """Shared helper for turn tests (modes 8 & 9)."""
    print("\n" + "=" * 70)
    print(f"  TURN {direction}: march -> walk {MODE_TURN_DISTANCE_MM:.0f}mm -> march -> stand")
    print(f"  Turn bias: {turn_bias:+.1f} mm")
    print("=" * 70)
    init_logging(MODE_TURN_DISTANCE_MM)
    try:
        print("\n  Step 1/4: Starting idle march...")
        if not start_idle_march():
            print("  Failed to start idle march")
            return False
        print("  Marching in place for 2 seconds...")
        time.sleep(2.0)

        print(f"\n  Step 2/4: Turning walk {direction} {MODE_TURN_DISTANCE_MM:+.0f}mm...")
        success = move_relative_y_with_turn(
            +MODE_TURN_DISTANCE_MM, turn_bias, timeout_s=60.0, transition_to_march=True)
        if not success:
            if idle_marching:
                stop_idle_march()
            return False

        print("\n  Step 3/4: Transitioning to idle march...")
        if not start_idle_march(resume_from=march_step_indices):
            print("  Failed to start idle march")
            return False
        print("  Marching in place for 2 seconds...")
        time.sleep(2.0)

        print("\n  Step 4/4: Returning to standing position...")
        if not stop_idle_march():
            print("  Failed to stop idle march")
            return False

        print(f"\n  TURN {direction} COMPLETED!")
        return True
    except Exception as e:
        print(f"\n  Test failed: {e}")
        if idle_marching:
            stop_idle_march()
        return False
    finally:
        close_logging()


def test_turn_left_300() -> bool:
    """Mode 8: Walk forward 300mm turning LEFT."""
    return _test_turn("LEFT", turn_bias=-TURN_BIAS)


def test_turn_right_300() -> bool:
    """Mode 9: Walk forward 300mm turning RIGHT."""
    return _test_turn("RIGHT", turn_bias=+TURN_BIAS)

# ============================================================================
# MENU
# ============================================================================

def print_menu():
    imu_status = "DISABLED"
    if IMU_ENABLED and imu_reader:
        if imu_reader.is_receiving_data():
            imu_status = f"ACTIVE (Yaw: {imu_reader.get_yaw():+.1f}°)"
        elif imu_reader.is_connected():
            imu_status = "CONNECTED (no data)"
        else:
            imu_status = "NOT CONNECTED"
    march_status = "MARCHING" if idle_marching else "STANDING"
    print("\n" + "=" * 70)
    print(f"  WALK TEST STANDALONE  [{('SIM' if SIMULATION_MODE else 'HW')}]  {march_status}")
    print(f"  IMU: {imu_status}")
    print(f"  ML : {get_ml_status()}")
    print("=" * 70)
    print(f"  [7] Smooth walk {MODE7_DISTANCE_MM:+.0f}mm (march -> walk -> march -> stand)")
    print(f"  [8] Turn LEFT {MODE_TURN_DISTANCE_MM:+.0f}mm   (march -> turn -> march -> stand)")
    print(f"  [9] Turn RIGHT {MODE_TURN_DISTANCE_MM:+.0f}mm  (march -> turn -> march -> stand)")
    print("  [M] Toggle ML compensation")
    print("  [Q] Quit")
    print("=" * 70)


def interactive_mode():
    global nav_planner, state_estimator
    nav_planner = SimpleNavigationPlanner(
        v_max=NAV_V_MAX, K_p=NAV_K_P, tolerance=NAV_TOLERANCE)
    state_estimator = TimeBasedEstimator(imu_reader=imu_reader)

    while True:
        print_menu()
        if sys.platform == 'win32':
            print("  Enter command: ", end='', flush=True)
            key = msvcrt.getch()
            print(key.decode('utf-8', errors='ignore'))
            if key == b'7':
                test_smooth_walk_600()
            elif key == b'8':
                test_turn_left_300()
            elif key == b'9':
                test_turn_right_300()
            elif key.lower() == b'm':
                toggle_ml_compensation()
            elif key.lower() == b'q':
                print("\n  Goodbye!")
                break
            else:
                print("  Unknown command")
        else:
            cmd = input("  Enter command: ").strip()
            if cmd == '7':
                test_smooth_walk_600()
            elif cmd == '8':
                test_turn_left_300()
            elif cmd == '9':
                test_turn_right_300()
            elif cmd.lower() == 'm':
                toggle_ml_compensation()
            elif cmd.lower() == 'q':
                print("\n  Goodbye!")
                break
            else:
                print("  Unknown command")

# ============================================================================
# MAIN
# ============================================================================

def main():
    global SIMULATION_MODE, imu_reader, yaw_controller

    print("=" * 70)
    print("  BLEGS Walk Test - Standalone (Modes 7, 8, 9)")
    print("=" * 70)
    print(f"  ML compensation: {get_ml_status()}")

    # --- IMU ---
    if IMU_ENABLED:
        print(f"\n  Connecting to IMU on {IMU_PORT}...")
        imu_reader = create_imu_reader(IMU_PORT, auto_connect=True)
        if imu_reader and imu_reader.is_connected():
            print("  IMU port opened")
            data_ok = False
            for _ in range(20):
                if imu_reader.is_receiving_data():
                    data_ok = True
                    break
                time.sleep(0.1)
            if not data_ok:
                print("  No data from IMU - continuing without yaw control")
                imu_reader.disconnect()
                imu_reader = None
            else:
                for _ in range(30):
                    if imu_reader.is_calibrated():
                        break
                    time.sleep(0.1)
                imu_reader.set_zero()
                time.sleep(0.2)
                yaw_controller = YawController(
                    K_p=YAW_K_P, K_d=YAW_K_D,
                    max_correction=YAW_MAX_CORRECTION)
                print(f"  Yaw controller initialized")
        else:
            print("  Failed to open IMU port")
            imu_reader = None
    else:
        print("\n  IMU disabled")

    # --- Motors ---
    print("\n  Discovering motors...")
    discovered = discover_motors()
    if not discovered:
        print("\n  No motors found - running in SIMULATION MODE")
        SIMULATION_MODE = True

    if discovered:
        all_reg = register_leg_motors(discovered)
        if not all_reg:
            print("\n  Not all motors registered!")
            resp = input("  Continue anyway? (y/n): ")
            if resp.lower() != 'y':
                return

        if leg_motors:
            print("\n  Starting motors...")
            start_all_motors()
            for motor in motor_registry.values():
                motor.set_timeout(FAST_TIMEOUT)

            home_angles = {}
            for lid in leg_motors:
                P_A, P_B = get_motor_positions(lid)
                angles = calculate_ik_no_ef(
                    np.array([DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT]),
                    P_A, P_B)
                home_angles[lid] = np.rad2deg(angles)

            print("\n  Moving to home position...")
            smooth_move_to_home_position(leg_motors, home_angles, duration_s=3.0)
    else:
        SIMULATION_MODE = True

    print(f"\n  Mode: {'SIMULATION' if SIMULATION_MODE else 'HARDWARE'}")

    # --- Run ---
    try:
        interactive_mode()
    except KeyboardInterrupt:
        print("\n  Interrupted by user")
    finally:
        global idle_marching
        if idle_marching:
            idle_marching = False
            time.sleep(0.5)
        if motor_registry:
            print("\n  Moving motors to init position (-90 deg)...")
            for motor in motor_registry.values():
                try:
                    motor.set_position_direct(MOTOR_INIT_ANGLE)
                except Exception:
                    pass
            time.sleep(1.0)
        for motor in motor_registry.values():
            motor.disconnect()
        if imu_reader and imu_reader.is_connected():
            imu_reader.disconnect()
        print("  Done")


if __name__ == "__main__":
    main()
