"""
Single Leg XY Position Control - Five-Bar Linkage
Author: M-TRCH
Date: May 2026

ทดสอบขาเดียว (2 มอเตอร์, 2 COM port) โดยรอรับคำสั่งตำแหน่ง (x, y)
แล้วคำนวณ IK และส่งคำสั่งไปยังมอเตอร์

Architecture:
┌─────────────────────────────────────────────────────────┐
│  User Input: (x, y) mm in leg frame                     │
├─────────────────────────────────────────────────────────┤
│  IK Solver  ─  Five-Bar linkage (no EF link)            │
│    Motor A angle (theta_A)  /  Motor B angle (theta_B)  │
├─────────────────────────────────────────────────────────┤
│  Motor A ── COM_PORT_A      Motor B ── COM_PORT_B       │
│  (Binary Protocol v1.2, 921600 baud)                    │
└─────────────────────────────────────────────────────────┘

Leg Frame:
    Motor A ────────── Motor B
       (-42.5, 0)    (+42.5, 0)    ← for LEFT-side leg
       (+42.5, 0)    (-42.5, 0)    ← for RIGHT-side leg

       x = 0  (center)
       y = 0  (motor plane)
       y < 0  (below motor plane, foot in stance)

Workspace (approx):
    x  : -60 … +60  mm
    y  : -260 … -120 mm  (stance height range)

Default home position: (0, -220) mm

Usage:
    python single_leg_xy_control.py

    Commands (interactive):
        0,-220        → move foot to x=0, y=-220 mm
        20,-180       → move foot to x=20, y=-180 mm
        home          → return to default stance position
        status        → show motor feedback
        scan          → re-scan COM ports
        s             → send with S-Curve (smooth) profile (toggle)
        q             → quit
"""

import numpy as np
import time
import sys
import os
import struct
import threading
import serial
import serial.tools.list_ports

# Windows keyboard input
if sys.platform == 'win32':
    import msvcrt

# Optional: OpenCV for live capture mode (ArUco-based position measurement)
try:
    import cv2
    import cv2.aruco as _cv2_aruco
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

# ============================================================================
# USER CONFIGURATION  ← แก้ไขส่วนนี้ก่อนใช้งาน
# ============================================================================

# COM port สำหรับ Motor A (ซ้าย/inner) และ Motor B (ขวา/outer)
# กำหนดค่า None เพื่อให้ auto-scan หา port อัตโนมัติ
COM_PORT_A = None          # e.g. 'COM3'  — Motor A (inner)
COM_PORT_B = None          # e.g. 'COM4'  — Motor B (outer)

# ขาที่ต้องการทดสอบ: 'LEFT' หรือ 'RIGHT'
# LEFT  → P_A = (-42.5, 0), P_B = (+42.5, 0)
# RIGHT → P_A = (+42.5, 0), P_B = (-42.5, 0)
LEG_SIDE = 'LEFT'

# Motor ID ที่คาดหวัง (ใช้สำหรับ verification เท่านั้น, ไม่ block การทำงาน)
EXPECTED_MOTOR_A_ID = None   # e.g. 1  — ใส่ None เพื่อไม่ตรวจสอบ
EXPECTED_MOTOR_B_ID = None   # e.g. 2

# Control profile
USE_SCURVE = True           # True = S-Curve smooth, False = Direct position
SCURVE_DURATION_MS = 500    # ระยะเวลาการเคลื่อนที่แบบ S-Curve (ms)

# Grid sweep (calibration LUT)
GRID_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output', 'data', 'workspace_grid.csv')
GRID_DWELL_S = 0.5          # เวลาหยุดที่แต่ละจุด (วินาที) ในโหมด auto
GRID_AUTO = True            # True = auto advance, False = กด Enter เพื่อไปจุดถัดไป
GRID_LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output', 'data', 'grid_log.csv')

# Capture mode — ArUco-based live position measurement
CAPTURE_DIR       = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'capture_image')
CAPTURE_TIMEOUT_S = 20.0    # (ไม่ใช้แล้ว — เก็บไว้สำหรับ backward-compat)
CAMERA_INDEX      = 1       # index กล้อง: 0 = built-in, 1 = HDMI capture card
CAMERA_WIDTH      = 3840    # ความกว้างภาพที่ต้องการ (px); 0 = default
CAMERA_HEIGHT     = 2160    # ความสูงภาพ
CALIB_NPZ         = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'calibration_olympus25mm.npz')
ARUCO_E_ID        = 4       # ArUco marker ID ของ end-effector E
ARUCO_A_ID        = 0       # ArUco marker ID ของ Motor A
ARUCO_B_ID        = 1       # ArUco marker ID ของ Motor B
CAPTURE_SETTLE_S  = 0.3     # รอให้ภาพนิ่งก่อนจับภาพ (วินาที) — เพิ่มเติมจาก dwell
CAPTURE_N_FRAMES  = 10      # จำนวน frame ที่ใช้เฉลี่ยตำแหน่ง ArUco
CAPTURE_FLUSH_N   = 4       # frame ที่ flush ทิ้งก่อนจับ (ล้าง buffer กล้อง)
CAPTURE_WIN_W     = 1280    # ความกว้างหน้าต่าง preview (px)
SHOW_CAPTURE_WIN  = True    # True = แสดงหน้าต่าง annotated ระหว่างทำงาน

# Circle path trajectory
CIRCLE_CENTER_X  =  0.0    # จุดกึ่งกลาง x (mm)
CIRCLE_CENTER_Y  = -200.0  # จุดกึ่งกลาง y (mm)
CIRCLE_RADIUS    =  25.0   # รัศมี (mm) — แนะนำ 20-30 mm
CIRCLE_POINTS    =  36     # จำนวนจุดต่อรอบ (36 = ทุก 10°, 72 = ทุก 5°)
CIRCLE_REVS      =  1      # จำนวนรอบ
CIRCLE_DWELL_S   =  0.0    # เวลาหยุดต่อจุด (วินาที); 0 = ใช้แค่ S-Curve duration
CIRCLE_FREQ_HZ   =  0.6    # ความเร็ว (รอบ/วินาที); 0 = ใช้ SCURVE_DURATION_MS แทน
CIRCLE_COMPENSATION     = False          # True = เปิด feed-forward kinematic error compensation
COMPENSATION_MODEL_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output', 'models')
COMPENSATION_MODEL_NAME = 'model_poly4'  # ชื่อโมเดลเริ่มต้น (ไม่ต้องใส่ .pkl/.json)

# ============================================================================
# PROTOCOL / COMMUNICATION CONSTANTS
# ============================================================================

HEADER_1   = 0xFE
HEADER_2   = 0xEE
BAUD_RATE  = 921600
TIMEOUT    = 0.2    # seconds — ใช้ระหว่าง discovery
FAST_TIMEOUT = 0.05 # seconds — ใช้ระหว่าง high-speed operation
PING_RETRIES = 5

# ============================================================================
# FIVE-BAR LINKAGE PARAMETERS
# ============================================================================

MOTOR_SPACING  = 85.0   # ระยะห่างระหว่าง Motor A และ Motor B (mm)
L_AC  = 105.0           # ความยาว link จาก Motor A ถึง joint C (mm)
L_BD  = 105.0           # ความยาว link จาก Motor B ถึง joint D (mm)
L_CE  = 145.0           # ความยาว link จาก joint C ถึง end-effector E (mm)
L_DE  = 145.0           # ความยาว link จาก joint D ถึง end-effector E (mm)
GEAR_RATIO = 8.0        # อัตราทด gear (8:1)

# ตำแหน่งมอเตอร์ในระบบพิกัดขา
if LEG_SIDE == 'LEFT':
    P_A = np.array([-MOTOR_SPACING / 2, 0.0])   # Motor A (inner)
    P_B = np.array([ MOTOR_SPACING / 2, 0.0])   # Motor B (outer)
else:  # RIGHT
    P_A = np.array([ MOTOR_SPACING / 2, 0.0])   # Motor A (inner)
    P_B = np.array([-MOTOR_SPACING / 2, 0.0])   # Motor B (outer)

# ตำแหน่ง home (ท่ายืน)
HOME_X = 0.0
HOME_Y = -220.0
MOTOR_INIT_ANGLE = -90.0   # degrees

# ============================================================================
# PACKET TYPE ENUM (Binary Protocol v1.2)
# ============================================================================

PKT_CMD_SET_GOAL       = 0x01
PKT_CMD_PING           = 0x03
PKT_CMD_EMERGENCY_STOP = 0x04
PKT_FB_STATUS          = 0x81
PKT_FB_ERROR           = 0x83

MODE_DIRECT_POSITION   = 0x00
MODE_SCURVE_PROFILE    = 0x01

# ============================================================================
# PROTOCOL HELPERS
# ============================================================================

def calculate_crc16(data: bytes) -> int:
    """CRC-16-IBM checksum"""
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
    """สร้าง binary packet พร้อม CRC"""
    header      = bytes([HEADER_1, HEADER_2])
    type_byte   = bytes([pkt_type])
    len_byte    = bytes([len(payload)])
    crc_data    = type_byte + len_byte + payload
    crc         = calculate_crc16(crc_data)
    crc_bytes   = struct.pack('<H', crc)
    return header + type_byte + len_byte + payload + crc_bytes


# ============================================================================
# MOTOR CONTROLLER CLASS
# ============================================================================

class SingleMotorController:
    """
    Controller สำหรับมอเตอร์ 1 ตัว ผ่าน Binary Protocol v1.2
    (ดัดแปลงจาก BinaryMotorController ใน test_quadruped_control.py)
    """

    def __init__(self, port: str, label: str = 'Motor'):
        self.port   = port
        self.label  = label        # 'A' หรือ 'B' สำหรับ display
        self.serial = None
        self.motor_id = None
        self.is_connected = False
        self.current_position = 0.0   # degrees (output shaft)
        self.current_current  = 0
        self.current_flags    = 0
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """เปิด serial port พร้อม timeout protection"""
        result = {'ser': None, 'err': None}

        def _open():
            try:
                ser = serial.Serial(
                    port=self.port, baudrate=BAUD_RATE,
                    timeout=TIMEOUT, write_timeout=TIMEOUT,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    dsrdtr=False, rtscts=False
                )
                ser.dtr = False
                ser.rts = False
                result['ser'] = ser
            except Exception as e:
                result['err'] = str(e)

        t = threading.Thread(target=_open, daemon=True)
        t.start()
        t.join(timeout=5.0)

        if t.is_alive() or result['ser'] is None:
            print(f"    [{self.label}] ❌  Cannot open {self.port}: {result['err']}")
            return False

        self.serial = result['ser']
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        time.sleep(0.1)
        self.serial.reset_input_buffer()
        self.is_connected = True
        print(f"    [{self.label}] ✅  Opened {self.port}")
        return True

    def disconnect(self):
        """ปิด serial port"""
        if self.serial and self.is_connected:
            try:
                self.serial.close()
            except Exception:
                pass
            self.is_connected = False

    def set_timeout(self, t: float):
        if self.serial and self.is_connected:
            self.serial.timeout = t
            self.serial.write_timeout = t

    # ------------------------------------------------------------------
    # Low-level packet I/O
    # ------------------------------------------------------------------

    def _read_packet(self):
        """อ่าน 1 packet จาก serial → (pkt_type, payload) หรือ None"""
        try:
            if self.serial.in_waiting < 2:
                return None
            header = self.serial.read(2)
            if len(header) != 2 or header[0] != HEADER_1 or header[1] != HEADER_2:
                self.serial.reset_input_buffer()
                return None
            meta = self.serial.read(2)
            if len(meta) != 2:
                return None
            pkt_type, payload_len = meta[0], meta[1]
            if payload_len > 128:
                self.serial.reset_input_buffer()
                return None
            payload   = self.serial.read(payload_len)
            crc_bytes = self.serial.read(2)
            if len(payload) != payload_len or len(crc_bytes) != 2:
                return None
            received_crc   = struct.unpack('<H', crc_bytes)[0]
            calculated_crc = calculate_crc16(bytes([pkt_type, payload_len]) + payload)
            if received_crc != calculated_crc:
                return None
            return (pkt_type, payload)
        except Exception:
            return None

    def _parse_status(self, payload):
        """แยก status payload → อัปเดต current_position / current / flags"""
        if len(payload) >= 8:
            self.motor_id        = payload[0]
            pos_raw              = struct.unpack('<i', payload[1:5])[0]
            self.current_position = (pos_raw / 100.0) / GEAR_RATIO   # output-shaft degrees
            self.current_current  = struct.unpack('<h', payload[5:7])[0]
            self.current_flags    = payload[7]

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def ping(self) -> dict | None:
        """ส่ง PING และรอ response → dict หรือ None"""
        if not self.is_connected:
            return None
        try:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.02)
            self.serial.write(build_packet(PKT_CMD_PING))
            self.serial.flush()

            # รอ response
            deadline = time.time() + max(0.3, TIMEOUT)
            while time.time() < deadline:
                if self.serial.in_waiting >= 2:
                    break
                time.sleep(0.005)

            result = self._read_packet()
            if result:
                pkt_type, payload = result
                if pkt_type == PKT_FB_STATUS and len(payload) >= 8:
                    self._parse_status(payload)
                    return {
                        'motor_id': self.motor_id,
                        'position': self.current_position,
                        'current':  self.current_current,
                        'flags':    self.current_flags,
                    }
            return None
        except Exception:
            return None

    def set_position_direct(self, angle_deg: float) -> bool:
        """ส่งคำสั่งตำแหน่งแบบ Direct (เร็วที่สุด, ไม่ smooth)"""
        if not self.is_connected:
            return False
        try:
            motor_angle = angle_deg * GEAR_RATIO
            with self._lock:
                payload = bytes([MODE_DIRECT_POSITION]) + struct.pack('<i', int(motor_angle * 100))
                self.serial.write(build_packet(PKT_CMD_SET_GOAL, payload))
                self.serial.flush()
                time.sleep(0.001)
                if self.serial.in_waiting >= 12:
                    r = self._read_packet()
                    if r:
                        pkt_type, pl = r
                        if pkt_type == PKT_FB_STATUS:
                            self._parse_status(pl)
            return True
        except Exception:
            return False

    def set_position_scurve(self, angle_deg: float, duration_ms: int = SCURVE_DURATION_MS) -> bool:
        """ส่งคำสั่งตำแหน่งแบบ S-Curve (smooth)"""
        if not self.is_connected:
            return False
        try:
            motor_angle = angle_deg * GEAR_RATIO
            with self._lock:
                payload = (bytes([MODE_SCURVE_PROFILE])
                           + struct.pack('<i', int(motor_angle * 100))
                           + struct.pack('<H', duration_ms))
                self.serial.write(build_packet(PKT_CMD_SET_GOAL, payload))
                self.serial.flush()
                time.sleep(0.001)
                if self.serial.in_waiting >= 12:
                    r = self._read_packet()
                    if r:
                        pkt_type, pl = r
                        if pkt_type == PKT_FB_STATUS:
                            self._parse_status(pl)
            return True
        except Exception:
            return False

    def send_emergency_stop(self) -> bool:
        """ส่ง Emergency Stop"""
        if not self.is_connected:
            return False
        try:
            self.serial.write(build_packet(PKT_CMD_EMERGENCY_STOP))
            self.serial.flush()
            print(f"    [{self.label}] ⚠️  Emergency stop sent")
            return True
        except Exception:
            return False

    def read_feedback(self) -> dict | None:
        """อ่าน feedback ที่ค้างอยู่ใน buffer"""
        if not self.is_connected:
            return None
        r = self._read_packet()
        if r:
            pkt_type, payload = r
            if pkt_type == PKT_FB_STATUS:
                self._parse_status(payload)
                return {
                    'motor_id': self.motor_id,
                    'position': self.current_position,
                    'current':  self.current_current,
                    'flags':    self.current_flags,
                    'moving':   bool(self.current_flags & 0x01),
                    'at_goal':  bool(self.current_flags & 0x04),
                }
        return None


# ============================================================================
# INVERSE KINEMATICS
# ============================================================================

def _circle_intersect_both(c1, r1, c2, r2):
    """หาจุดตัดทั้งสองของวงกลม 2 วง → (p1, p2) หรือ None ถ้าไม่มีจุดตัด"""
    d = np.linalg.norm(c2 - c1)
    if d > (r1 + r2) or d < abs(r1 - r2) or d == 0:
        return None
    a  = (r1**2 - r2**2 + d**2) / (2 * d)
    h2 = r1**2 - a**2
    if h2 < 0:
        return None
    h  = np.sqrt(h2)
    vd = (c2 - c1) / d
    vp = np.array([-vd[1], vd[0]])
    return c1 + a*vd + h*vp, c1 + a*vd - h*vp


def _circle_intersect_clamped(c1, r1, c2, r2):
    """เหมือน _circle_intersect_both แต่ clamp d ให้อยู่ใน valid range เสมอ
    เมื่อจุดเป้าหมายเกิน singularity boundary จะคืนค่า tangent point
    (แขนยืดสุด / งอสุดในทิศทางของเป้าหมาย) แทนที่จะ return None
    ใช้ใน forced IK สำหรับ grid sweep นอก workspace"""
    d = np.linalg.norm(c2 - c1)
    if d == 0:
        return None
    d_safe = np.clip(d, abs(r1 - r2) + 1e-9, r1 + r2 - 1e-9)
    a  = (r1**2 - r2**2 + d_safe**2) / (2 * d_safe)
    h2 = max(0.0, r1**2 - a**2)
    h  = np.sqrt(h2)
    vd = (c2 - c1) / d
    vp = np.array([-vd[1], vd[0]])
    return c1 + a*vd + h*vp, c1 + a*vd - h*vp


def calculate_fk(theta_A_deg: float, theta_B_deg: float) -> np.ndarray | None:
    """
    คำนวณ Forward Kinematics: มุมมอเตอร์ → ตำแหน่งปลายขา E (lower branch)

    Args:
        theta_A_deg: มุม output-shaft ของ Motor A (degrees)
        theta_B_deg: มุม output-shaft ของ Motor B (degrees)

    Returns:
        [x, y] (mm) ตำแหน่งปลายขา (lower E) หรือ None ถ้าไม่มี solution
    """
    C = P_A + L_AC * np.array([np.cos(np.deg2rad(theta_A_deg)), np.sin(np.deg2rad(theta_A_deg))])
    D = P_B + L_BD * np.array([np.cos(np.deg2rad(theta_B_deg)), np.sin(np.deg2rad(theta_B_deg))])
    pts_E = _circle_intersect_both(C, L_CE, D, L_DE)
    if pts_E is None:
        return None
    # เลือก E ที่อยู่ต่ำกว่า (foot อยู่ใต้มอเตอร์)
    return pts_E[1] if pts_E[1][1] < pts_E[0][1] else pts_E[0]


def _home_ik_reference_rad() -> np.ndarray | None:
    """
    คำนวณมุมมอเตอร์ที่ home position (radians) โดยเลือก lower-elbow solution
    ใช้เป็น reference สำหรับการเลือก IK solution ครั้งแรก
    """
    P_E = np.array([HOME_X, HOME_Y], dtype=float)
    pts_C = _circle_intersect_both(P_A, L_AC, P_E, L_CE)
    pts_D = _circle_intersect_both(P_B, L_BD, P_E, L_DE)
    if pts_C is None or pts_D is None:
        return None
    C = pts_C[1] if pts_C[1][1] < pts_C[0][1] else pts_C[0]
    D = pts_D[1] if pts_D[1][1] < pts_D[0][1] else pts_D[0]
    tA = np.arctan2((C - P_A)[1], (C - P_A)[0])
    tB = np.arctan2((D - P_B)[1], (D - P_B)[0])
    return np.array([tA, tB])


# IK continuity state — เก็บในหน่วย radians (เหมือน test_quadruped_control.py)
# reset เป็น None เมื่อเรียก go_init() เพื่อให้ครั้งถัดไปอ้างอิง home
_ik_prev_angles_rad: np.ndarray | None = None


def calculate_ik(target_xy: np.ndarray, force: bool = False) -> tuple[float, float] | None:
    """
    คำนวณ Inverse Kinematics สำหรับ Five-Bar linkage (ไม่มี EF link)

    ใช้อัลกอริทึมเดียวกับ test_quadruped_control.py:
    - ประเมิน 4 elbow configurations ทุกครั้ง
    - เลือก solution ที่มี angular distance ต่ำสุด (radians, wrap-around)
    - ครั้งแรก (หรือหลัง reset): เทียบกับ home reference angles
    - ครั้งถัดไป: เทียบกับมุมก่อนหน้า (continuity)

    หมายเหตุ: FK validation ไม่สามารถ discriminate IK solutions ได้
    เนื่องจาก target E อยู่บน arm circles ของทุก candidate เสมอ
    วิธีที่เชื่อถือได้คือ angle continuity เท่านั้น

    Args:
        target_xy: [x, y] ตำแหน่งปลายขา (mm) ในระบบพิกัดขา
        force    : True = ใช้ clamped IK (สั่งงานนอก workspace ได้ โดย clamp
                   ไปที่ singularity boundary ในทิศทางของเป้าหมาย)

    Returns:
        (theta_A_deg, theta_B_deg) มุมมอเตอร์ (output-shaft, degrees)
        หรือ None ถ้าไม่มี solution
    """
    global _ik_prev_angles_rad

    P_E = np.array(target_xy, dtype=float)
    _intersect = _circle_intersect_clamped if force else _circle_intersect_both

    pts_C = _intersect(P_A, L_AC, P_E, L_CE)
    if pts_C is None:
        return None

    pts_D = _intersect(P_B, L_BD, P_E, L_DE)
    if pts_D is None:
        return None

    # กำหนด reference (radians)
    if _ik_prev_angles_rad is None:
        ref = _home_ik_reference_rad()
        if ref is None:
            ref = np.deg2rad(np.array([MOTOR_INIT_ANGLE, MOTOR_INIT_ANGLE]))
    else:
        ref = _ik_prev_angles_rad

    # ประเมิน 4 configurations (ci, di) = index เข้า pts_C, pts_D
    # เรียงลำดับ: (0,0), (0,1), (1,0), (1,1)
    # ตรงกับ test_quadruped: (True,False),(True,True),(False,False),(False,True)
    best_solution = None
    best_distance = float('inf')

    for ci in range(2):
        for di in range(2):
            C = pts_C[ci]
            D = pts_D[di]
            tA = np.arctan2((C - P_A)[1], (C - P_A)[0])   # radians
            tB = np.arctan2((D - P_B)[1], (D - P_B)[0])   # radians
            solution = np.array([tA, tB])

            # Angular distance ใน radians พร้อม wrap-around
            # (เหมือน test_quadruped_control.py บรรทัด 1944-1946)
            diff = np.abs(solution - ref)
            diff = np.minimum(diff, 2 * np.pi - diff)
            distance = np.sum(diff)

            if distance < best_distance:
                best_distance = distance
                best_solution = solution

    if best_solution is None:
        return None

    _ik_prev_angles_rad = best_solution
    return float(np.rad2deg(best_solution[0])), float(np.rad2deg(best_solution[1]))



# ============================================================================
# COM PORT SCANNING / AUTO-DETECT
# ============================================================================

def list_com_ports() -> list[str]:
    """คืน list ของ COM port ที่มีอยู่"""
    return [p.device for p in serial.tools.list_ports.comports()]


def auto_detect_ports() -> tuple[str | None, str | None]:
    """
    Auto-detect 2 COM ports สำหรับ Motor A และ Motor B
    ถ้ามี 2 port → ส่งคืนตามลำดับที่พบ
    ถ้ามี 1 port → คืน (port, None)
    ถ้าไม่มีเลย → คืน (None, None)
    """
    ports = list_com_ports()
    if len(ports) >= 2:
        return ports[0], ports[1]
    elif len(ports) == 1:
        return ports[0], None
    return None, None


def select_ports_interactively() -> tuple[str, str]:
    """
    แสดง COM port ที่มี แล้วให้ user เลือก port สำหรับ Motor A และ B
    """
    ports = list_com_ports()
    if not ports:
        print("  ❌ ไม่พบ COM port")
        sys.exit(1)

    print("\n  COM ports ที่พบ:")
    for i, p in enumerate(ports):
        print(f"    [{i}] {p}")

    def pick(label: str, default_idx: int) -> str:
        while True:
            try:
                inp = input(f"  เลือก port สำหรับ Motor {label} [{ports[default_idx]}]: ").strip()
                if inp == '':
                    return ports[default_idx]
                idx = int(inp)
                if 0 <= idx < len(ports):
                    return ports[idx]
                # ถ้าพิมพ์ชื่อ port โดยตรง (เช่น COM3)
                if inp.upper() in [p.upper() for p in ports]:
                    return inp.upper()
                print("  ⚠️  Invalid selection")
            except ValueError:
                if inp.upper() in [p.upper() for p in ports]:
                    return inp.upper()
                print("  ⚠️  Invalid input")

    port_a = pick('A (inner)', 0)
    port_b = pick('B (outer)', min(1, len(ports) - 1))
    return port_a, port_b


# ============================================================================
# SINGLE LEG CONTROLLER
# ============================================================================

class SingleLegController:
    """
    Controller สำหรับขาเดียว (Motor A + Motor B)
    รับคำสั่งตำแหน่ง (x, y) และส่งไปยัง motor ผ่าน IK
    """

    def __init__(self, port_a: str, port_b: str):
        self.motor_a = SingleMotorController(port_a, label='A')
        self.motor_b = SingleMotorController(port_b, label='B')
        self.use_scurve  = USE_SCURVE
        self.last_target = (HOME_X, HOME_Y)
        self.connected   = False

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """เชื่อมต่อ motor ทั้งคู่"""
        print("\n  Connecting motors...")
        ok_a = self.motor_a.connect()
        ok_b = self.motor_b.connect()
        if not ok_a or not ok_b:
            print("  ❌ ไม่สามารถเชื่อมต่อ motor ได้")
            return False
        self.connected = True
        return True

    def disconnect(self):
        """ตัดการเชื่อมต่อ"""
        self.motor_a.disconnect()
        self.motor_b.disconnect()
        self.connected = False

    def ping_both(self) -> bool:
        """PING motor ทั้งคู่เพื่อ initialize และตรวจสอบ"""
        print("\n  Pinging motors...")
        ok = True
        for motor, exp_id, label in [
            (self.motor_a, EXPECTED_MOTOR_A_ID, 'A'),
            (self.motor_b, EXPECTED_MOTOR_B_ID, 'B'),
        ]:
            success = False
            for attempt in range(PING_RETRIES):
                result = motor.ping()
                if result:
                    found_id = result['motor_id']
                    pos      = result['position']
                    print(f"    [Motor {label}] ✅  ID={found_id}  pos={pos:+.1f}°  "
                          f"current={result['current']} mA  flags=0x{result['flags']:02X}")
                    if exp_id is not None and found_id != exp_id:
                        print(f"    [Motor {label}] ⚠️  คาดหวัง ID={exp_id} แต่พบ ID={found_id}")
                    motor.set_timeout(FAST_TIMEOUT)
                    success = True
                    break
                time.sleep(0.1)
            if not success:
                print(f"    [Motor {label}] ❌  ไม่ตอบสนอง")
                ok = False
        return ok

    def go_home(self) -> bool:
        """เคลื่อนไปยัง home position (0, -220)"""
        return self.move_to(HOME_X, HOME_Y)

    def go_init(self):
        """ส่ง motor กลับตำแหน่ง init (-90°) ก่อน disconnect"""
        global _ik_prev_angles_rad
        print("\n  Moving to init angle (-90°)...")
        if self.use_scurve:
            self.motor_a.set_position_scurve(MOTOR_INIT_ANGLE, duration_ms=1000)
            self.motor_b.set_position_scurve(MOTOR_INIT_ANGLE, duration_ms=1000)
        else:
            self.motor_a.set_position_direct(MOTOR_INIT_ANGLE)
            self.motor_b.set_position_direct(MOTOR_INIT_ANGLE)
        _ik_prev_angles_rad = None   # reset IK state เพื่อให้ครั้งถัดไปอ้างอิง home
        time.sleep(1.2)

    # ------------------------------------------------------------------
    # Core move
    # ------------------------------------------------------------------

    def move_to(self, x: float, y: float, duration_ms: int | None = None,
                force: bool = False) -> bool:
        """
        เคลื่อนปลายขาไปยัง (x, y) mm

        Args:
            duration_ms: S-Curve duration (ms) สำหรับการเคลื่อนที่นี้โดยเฉพาะ;
                         None = ใช้ค่า SCURVE_DURATION_MS จาก config
            force      : True = ใช้ clamped IK (สั่งงานนอก workspace ได้)

        Returns:
            True ถ้าส่งคำสั่งสำเร็จ, False ถ้า IK ล้มเหลว
        """
        if not self.connected:
            print("  ❌ ยังไม่ได้เชื่อมต่อ")
            return False

        result = calculate_ik([x, y], force=force)
        if result is None:
            print(f"  ❌ IK ล้มเหลว: ({x:.1f}, {y:.1f}) อยู่นอก workspace")
            return False

        theta_A_deg, theta_B_deg = result

        fk_pos = calculate_fk(theta_A_deg, theta_B_deg)
        fk_err = np.linalg.norm(fk_pos - np.array([x, y])) if fk_pos is not None else float('nan')

        print(f"  → target ({x:+.1f}, {y:+.1f}) mm  "
              f"│  θA={theta_A_deg:+.2f}°  θB={theta_B_deg:+.2f}°  "
              f"│  FK err={fk_err:.3f} mm", end='  ')

        _dur = duration_ms if duration_ms is not None else SCURVE_DURATION_MS
        if self.use_scurve:
            ok_a = self.motor_a.set_position_scurve(theta_A_deg, duration_ms=_dur)
            ok_b = self.motor_b.set_position_scurve(theta_B_deg, duration_ms=_dur)
        else:
            ok_a = self.motor_a.set_position_direct(theta_A_deg)
            ok_b = self.motor_b.set_position_direct(theta_B_deg)

        if ok_a and ok_b:
            self.last_target = (x, y)
            print("✅")
            return True
        else:
            print("❌ (ส่งคำสั่งล้มเหลว)")
            return False

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def print_status(self):
        """แสดงสถานะ motor ปัจจุบัน"""
        # Request fresh feedback via PING
        fb_a = self.motor_a.ping()
        fb_b = self.motor_b.ping()
        # Restore fast timeout
        self.motor_a.set_timeout(FAST_TIMEOUT)
        self.motor_b.set_timeout(FAST_TIMEOUT)

        print("\n  ─── Motor Status ───────────────────────────────")
        for label, fb in [('A', fb_a), ('B', fb_b)]:
            if fb:
                flags = fb['flags']
                moving   = '🔄' if (flags & 0x01) else '  '
                at_goal  = '🎯' if (flags & 0x04) else '  '
                error    = '⚠️ ' if (flags & 0x02) else '  '
                print(f"    Motor {label}: pos={fb['position']:+7.2f}°  "
                      f"current={fb['current']:5d} mA  "
                      f"flags=0x{flags:02X}  {moving}{at_goal}{error}")
            else:
                print(f"    Motor {label}: ❌ ไม่ได้รับ feedback")

        x, y = self.last_target
        print(f"  Last target: ({x:+.1f}, {y:+.1f}) mm")
        profile = 'S-Curve' if self.use_scurve else 'Direct'
        print(f"  Profile    : {profile}")
        print(f"  Leg side   : {LEG_SIDE}")
        print("  ────────────────────────────────────────────────")

    def toggle_profile(self):
        """สลับระหว่าง S-Curve และ Direct profile"""
        self.use_scurve = not self.use_scurve
        mode = 'S-Curve (smooth)' if self.use_scurve else 'Direct (fast)'
        print(f"  Profile เปลี่ยนเป็น: {mode}")

    def emergency_stop(self):
        """ส่ง Emergency Stop ไปยัง motor ทั้งคู่"""
        self.motor_a.send_emergency_stop()
        self.motor_b.send_emergency_stop()


# ============================================================================
# WORKSPACE PREVIEW
# ============================================================================

def print_workspace_info():
    """แสดงข้อมูล workspace และ leg frame"""
    print("\n  ─── Leg Frame ──────────────────────────────────")
    if LEG_SIDE == 'LEFT':
        print(f"    Motor A (inner) : ({P_A[0]:+.1f}, {P_A[1]:+.1f}) mm")
        print(f"    Motor B (outer) : ({P_B[0]:+.1f}, {P_B[1]:+.1f}) mm")
    else:
        print(f"    Motor A (inner) : ({P_A[0]:+.1f}, {P_A[1]:+.1f}) mm")
        print(f"    Motor B (outer) : ({P_B[0]:+.1f}, {P_B[1]:+.1f}) mm")
    print(f"    Link lengths    : L_AC={L_AC} L_BD={L_BD} L_CE={L_CE} L_DE={L_DE} mm")
    print(f"    Gear ratio      : {GEAR_RATIO}:1")
    print(f"    Home position   : ({HOME_X:+.1f}, {HOME_Y:+.1f}) mm")
    print("  ─── Safe Calibration Workspace ─────────────────")
    _d_out = L_AC + L_CE          # outer singularity radius (mm) = 250
    _d_in  = abs(L_AC - L_CE)     # inner singularity radius (mm) = 40
    _margin = 15.0                # safety margin (mm)
    _d_lim  = _d_out - _margin    # 235 mm
    import math as _m
    # x limit at y = -200 mm  (from outer singularity of Motor A/B)
    _x_at200 = int(_m.sqrt(max(0, _d_lim**2 - 200**2)) - MOTOR_SPACING/2)
    # deepest safe y at x = 0
    _y_min0  = -int(_m.sqrt(_d_lim**2 - (MOTOR_SPACING/2)**2))
    # deepest safe y at x = 75 mm
    _y_min75 = -int(_m.sqrt(max(0, _d_lim**2 - (75 + MOTOR_SPACING/2)**2)))
    print(f"    Outer singularity : d = L_AC+L_CE = {int(_d_out)} mm  (ห้ามเกิน)")
    print(f"    Inner singularity : d = |L_AC-L_CE| = {int(_d_in)} mm  (ห้ามต่ำกว่า)")
    print(f"    Safety margin     : {int(_margin)} mm  → d_limit = {int(_d_lim)} mm")
    print(f"    x : ±{_x_at200} mm  (ที่ y = -200 mm)  |  ±75 mm  (ที่ y > -200 mm)")
    print(f"    y : -80 … {_y_min0} mm  (x=0)  |  -80 … {_y_min75} mm  (x=±75 mm)")
    print("  ────────────────────────────────────────────────")


# ============================================================================
# INTERACTIVE COMMAND LOOP
# ============================================================================

HELP_TEXT = """
  ─── Commands ────────────────────────────────────────────────────────
  x,y          ส่งขาไปยังตำแหน่ง (x, y) mm  เช่น  0,-220  หรือ  20,-180
  home         กลับ home position (0, -220) mm
  init         ส่ง motor กลับ init angle (-90°)
  status       แสดงสถานะ motor feedback
  info         แสดงข้อมูล workspace / leg frame
  s            สลับ profile: S-Curve ↔ Direct
  scan         แสดง COM ports ที่มีอยู่
  grid         รันการทดสอบไล่ตำแหน่งตาม workspace_grid.csv (ทุกจุดตามลำดับ)
  capture      ไล่ตำแหน่งตาม workspace_grid.csv พร้อมวัด actual XY ด้วยกล้อง+ArUco
  circle       เคลื่อนที่ตาม path วงกลม (ใช้ค่า default จาก config)
  circle R     เช่น  circle 25         → วงกลม R=25 mm ที่ center default
  circle R cx cy     เช่น  circle 25 0 -200  → กำหนด center ด้วย
  circle R cx cy N   เช่น  circle 25 0 -200 3 → N รอบ
  (เพิ่ม comp/nocomp เพื่อเปิด/ปิด compensation; เพิ่ม <ชื่อโมเดล> เพื่อเลือกโมเดล)
  เช่น  circle comp  |  circle 25 0 -200 comp model_mlp  |  circle nocomp
  models       แสดงรายการโมเดล compensation ที่มีใน output/models/
  e / estop    Emergency Stop
  h / help     แสดง help นี้
  q / quit     ออกจากโปรแกรม
  ─────────────────────────────────────────────────────────────────────
  ตัวอย่างตำแหน่ง  [ปลอดภัย: d_A,d_B ∈ (40, 250) mm — margin ≥ 15 mm]:
    0,-155    → กลาง workspace    (d_A=d_B≈161 mm)
    50,-155   → ขวา, y ตื้น      (d_A≈181 mm, d_B≈155 mm)
    -50,-180  → ซ้าย, y กลาง    (d_A≈180 mm, d_B≈202 mm)
    0,-200    → ล่างกลาง         (d_A=d_B≈200 mm)
  หลีกเลี่ยง: |x| > 80 mm ที่ y < -200 mm  (ใกล้ outer singularity)
"""


def parse_xy(text: str) -> tuple[float, float] | None:
    """แปลง string 'x,y' เป็น (float, float) หรือ None"""
    try:
        parts = text.replace(' ', '').split(',')
        if len(parts) == 2:
            return float(parts[0]), float(parts[1])
    except ValueError:
        pass
    return None


# ============================================================================
# CAPTURE MODE — ArUco-based live position measurement
# ============================================================================


def _load_calib(path: str):
    """โหลด camera_matrix, dist_coeffs จาก .npz — คืน (None, None) ถ้าไม่พบ"""
    try:
        data = np.load(path)
        return data['camera_matrix'], data['dist_coeffs']
    except Exception:
        return None, None


def _open_camera(index: int, width: int, height: int):
    """
    เปิดกล้องด้วย cv2.CAP_DSHOW (Windows) และตั้งค่า resolution
    คืน cv2.VideoCapture หรือ None ถ้าเปิดไม่ได้
    """
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return None
    if width > 0 and height > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"  📷 Camera index={index}  {actual_w}×{actual_h}")
    return cap


def _build_leg_transform(a_px: np.ndarray, b_px: np.ndarray):
    """
    สร้าง transform pixel → leg-frame จากตำแหน่ง Motor A และ B ใน pixel

    Returns: (origin_px, v_x, v_y_down, px_per_mm)
        origin_px  : กึ่งกลาง A–B (pixel)
        v_x        : unit vector ทิศ +x (A→B)
        v_y_down   : unit vector ทิศลงในภาพ (ตรงกับทิศ y− ใน leg frame)
        px_per_mm  : scale
    """
    origin = (a_px + b_px) / 2.0
    v_AB   = b_px - a_px
    dist   = np.linalg.norm(v_AB)
    if dist < 1e-6:
        return None
    vx    = v_AB / dist
    vy_dn = np.array([-vx[1], vx[0]])   # 90° CCW ใน image space → ลง
    ppmm  = dist / MOTOR_SPACING
    return origin, vx, vy_dn, ppmm


def _px_to_legframe(pt_px: np.ndarray, transform) -> tuple[float, float]:
    """pixel → (x_mm, y_mm) ในระบบพิกัดขา"""
    origin, vx, vy_dn, ppmm = transform
    d = pt_px - origin
    return float(np.dot(d, vx) / ppmm), float(-np.dot(d, vy_dn) / ppmm)


def _grab_aruco_sample(
    cap,
    detector,
    camera_matrix,
    dist_coeffs,
    n_frames: int   = CAPTURE_N_FRAMES,
    flush_n:  int   = CAPTURE_FLUSH_N,
) -> tuple[dict, np.ndarray | None]:
    """
    Flush กล้อง flush_n frame แล้วจับ n_frames frame ใหม่
    ตรวจ ArUco ทุก frame และหาค่า median ตำแหน่งของแต่ละ marker

    Returns:
        markers   : {marker_id: center_px (float64 ndarray)}
                    ว่างถ้าไม่พบ marker ใดเลย
        last_frame: frame สุดท้ายที่ undistort แล้ว (BGR) หรือ None
    """
    # Flush old frames from buffer
    for _ in range(flush_n):
        cap.read()

    accumulator: dict[int, list] = {}
    last_frame = None

    for _ in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            continue
        if camera_matrix is not None:
            frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        last_frame = frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                center = corners[i][0].mean(axis=0)
                accumulator.setdefault(int(mid), []).append(center)

    markers = {mid: np.median(pts, axis=0)
               for mid, pts in accumulator.items()}
    return markers, last_frame


def _show_capture_frame(
    frame:          np.ndarray,
    markers:        dict,
    transform,
    pid:            int,
    target_xy:      tuple[float, float],
    video_xy:       tuple[float, float],
    point_num:      int,
    total_points:   int,
    motor_feedback: dict | None = None,
) -> None:
    """
    วาด annotation บน frame แล้วแสดงใน cv2 window (non-blocking)

    Annotations:
      • เส้น Motor A–B (แกนอ้างอิง)
      • วงกลมที่ Motor A, B, E พร้อม label
      • cross-hair ที่ E
      • กรอบ info ด้านซ้ายบน: ID, progress, target, actual, error
      • scale bar
    """
    vis = frame.copy()
    h, w = vis.shape[:2]

    # ─── Marker colors ───────────────────────────────────────────────
    _COLORS = {
        ARUCO_A_ID: (  0, 220,   0),   # green — Motor A
        ARUCO_B_ID: ( 50, 180, 255),   # sky-blue — Motor B
        ARUCO_E_ID: (  0,  80, 255),   # orange-red — end-effector E
    }
    _LABELS = {ARUCO_A_ID: 'A', ARUCO_B_ID: 'B', ARUCO_E_ID: 'E'}

    # ─── Motor axis line A–B ─────────────────────────────────────────
    if ARUCO_A_ID in markers and ARUCO_B_ID in markers:
        pa = tuple(markers[ARUCO_A_ID].astype(int))
        pb = tuple(markers[ARUCO_B_ID].astype(int))
        cv2.line(vis, pa, pb, (0, 220, 0), 2)
        # Origin (midpoint)
        if transform is not None:
            orig = tuple(transform[0].astype(int))
            cv2.circle(vis, orig, 6, (0, 255, 255), -1)
            cv2.putText(vis, 'O', (orig[0] + 10, orig[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # ─── Detected markers ────────────────────────────────────────────
    for mid, pos_px in markers.items():
        cx, cy = int(pos_px[0]), int(pos_px[1])
        color  = _COLORS.get(mid, (180, 180, 180))
        label  = _LABELS.get(mid, f'ID{mid}')
        cv2.circle(vis, (cx, cy), 14, color, 2)
        cv2.putText(vis, label, (cx + 18, cy + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    # ─── Cross-hair at E ─────────────────────────────────────────────
    if ARUCO_E_ID in markers:
        ex, ey = int(markers[ARUCO_E_ID][0]), int(markers[ARUCO_E_ID][1])
        cv2.drawMarker(vis, (ex, ey), _COLORS[ARUCO_E_ID],
                       cv2.MARKER_CROSS, 40, 2)
        cv2.circle(vis, (ex, ey), 20, _COLORS[ARUCO_E_ID], 2)

    # ─── Scale bar (50 mm) ───────────────────────────────────────────
    if transform is not None:
        ppmm = transform[3]
        bar_px = int(ppmm * 50)
        bar_y  = h - 40
        bar_x0 = 40
        bar_x1 = bar_x0 + bar_px
        cv2.line(vis, (bar_x0, bar_y), (bar_x1, bar_y), (255, 255, 255), 3)
        cv2.line(vis, (bar_x0, bar_y - 8), (bar_x0, bar_y + 8), (255, 255, 255), 2)
        cv2.line(vis, (bar_x1, bar_y - 8), (bar_x1, bar_y + 8), (255, 255, 255), 2)
        cv2.putText(vis, '50 mm', (bar_x0, bar_y - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # ─── Info overlay ────────────────────────────────────────────────
    tx, ty   = target_xy
    vx_mm, vy_mm = video_xy
    err_mm   = (np.sqrt((vx_mm - tx)**2 + (vy_mm - ty)**2)
                if not (np.isnan(vx_mm) or np.isnan(vy_mm)) else float('nan'))

    if motor_feedback:
        mot_str = (f"tA={motor_feedback.get('tA_act', float('nan')):+.1f}  "
                   f"tB={motor_feedback.get('tB_act', float('nan')):+.1f} deg")
    else:
        mot_str = ''

    markers_ids = sorted(markers.keys())
    info_lines = [
        (f"Point {pid}  [{point_num}/{total_points}]",  (255, 255, 100)),
        (f"Target : ({tx:+.1f}, {ty:+.1f}) mm",        (220, 220, 220)),
        (f"Actual : ({vx_mm:+.1f}, {vy_mm:+.1f}) mm"
         if not np.isnan(vx_mm)
         else "Actual : N/A (E not detected)",           (100, 200, 255)),
        (f"Error  : {err_mm:.2f} mm"
         if not np.isnan(err_mm)
         else "Error  : N/A",
         (100, 255, 150) if (not np.isnan(err_mm) and err_mm < 5.0) else (80, 80, 255)),
        (f"Motors : {mot_str}",                         (200, 200, 200)),
        (f"Markers: {markers_ids}",                     (180, 220, 180)),
    ]

    # Semi-transparent background box
    box_h   = len(info_lines) * 42 + 20
    box_w   = 560
    overlay = vis.copy()
    cv2.rectangle(overlay, (10, 10), (box_w, box_h), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, 0.55, vis, 0.45, 0, vis)

    y_off = 44
    for line_text, color in info_lines:
        cv2.putText(vis, line_text, (20, y_off),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.80, (0, 0, 0), 3)
        cv2.putText(vis, line_text, (20, y_off),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.80, color, 2)
        y_off += 42

    # ─── Resize and show ─────────────────────────────────────────────
    win_w  = CAPTURE_WIN_W
    win_h  = max(1, int(h * win_w / w))
    small  = cv2.resize(vis, (win_w, win_h))
    WIN    = 'Capture Mode — ArUco Position Tracking  [q=stop]'
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, win_w, win_h)
    cv2.imshow(WIN, small)
    key = cv2.waitKey(1) & 0xFF
    return key  # caller can check for 'q' (113)


def _motor_dwell_and_feedback(leg: 'SingleLegController', dwell_s: float) -> dict:
    """
    รอ dwell_s วินาที แล้วอ่าน motor feedback
    คืน dict {tA_cmd, tB_cmd, tA_act, tB_act}
    """
    time.sleep(dwell_s)
    if _ik_prev_angles_rad is not None:
        tA_cmd = float(np.rad2deg(_ik_prev_angles_rad[0]))
        tB_cmd = float(np.rad2deg(_ik_prev_angles_rad[1]))
    else:
        tA_cmd = tB_cmd = float('nan')
    fb_a = leg.motor_a.ping()
    fb_b = leg.motor_b.ping()
    leg.motor_a.set_timeout(FAST_TIMEOUT)
    leg.motor_b.set_timeout(FAST_TIMEOUT)
    tA_act = fb_a['position'] if fb_a else float('nan')
    tB_act = fb_b['position'] if fb_b else float('nan')
    return {'tA_cmd': tA_cmd, 'tB_cmd': tB_cmd,
            'tA_act': tA_act, 'tB_act': tB_act}


def run_capture_point(
    leg: 'SingleLegController',
    target_id: int,
    cap=None,
    csv_path: str = GRID_FILE,
) -> bool:
    """
    เคลื่อนไปยัง point_id = target_id แล้วถ่ายภาพจากกล้อง
    ตรวจจับ ArUco markers คำนวณ actual position (x,y) ใน leg frame
    บันทึกลง grid_log.csv และแสดงภาพ annotated

    Args:
        cap: cv2.VideoCapture ที่เปิดอยู่แล้ว (ถ้า None จะเปิด/ปิดเอง)

    Returns:
        True ถ้าสำเร็จ (เคลื่อนที่ได้และ log แล้ว)
    """
    if not _CV2_OK:
        print("  ❌ capture mode ต้องการ opencv-python  (pip install opencv-python opencv-contrib-python)")
        return False

    import csv as _csv

    # ─── โหลดตำแหน่งจาก CSV ──────────────────────────────────────────
    try:
        with open(csv_path, newline='', encoding='utf-8') as _f:
            _rows = list(_csv.DictReader(_f))
            _row = next(
                (r for idx, r in enumerate(_rows)
                 if (int(r['point_id']) if 'point_id' in r else idx + 1) == target_id),
                None
            )
    except FileNotFoundError:
        print(f"  ❌ ไม่พบไฟล์: {csv_path}")
        return False
    except Exception as _e:
        print(f"  ❌ โหลด CSV ล้มเหลว: {_e}")
        return False

    if _row is None:
        print(f"  ⚠️  ไม่พบ point_id={target_id} ใน {csv_path}")
        return False

    tx, ty = float(_row['target_x_mm']), float(_row['target_y_mm'])

    # ─── เปิดกล้องถ้ายังไม่มี ────────────────────────────────────────
    _own_cap = False
    if cap is None:
        camera_matrix, dist_coeffs = _load_calib(CALIB_NPZ)
        cap = _open_camera(CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT)
        if cap is None:
            print(f"  ❌ ไม่สามารถเปิดกล้อง index={CAMERA_INDEX}")
            return False
        _own_cap = True
    camera_matrix, dist_coeffs = _load_calib(CALIB_NPZ)
    aruco_dict   = _cv2_aruco.getPredefinedDictionary(_cv2_aruco.DICT_6X6_250)
    aruco_params = _cv2_aruco.DetectorParameters()
    detector     = _cv2_aruco.ArucoDetector(aruco_dict, aruco_params)

    try:
        # ─── เคลื่อนที่ไปยังตำแหน่ง ─────────────────────────────────
        print(f"\n  Capture ID={target_id}: ({tx:+.1f}, {ty:+.1f}) mm")
        ok = leg.move_to(tx, ty, force=True)

        # รอ dwell + อ่าน motor feedback
        dwell_total = max(GRID_DWELL_S, SCURVE_DURATION_MS / 1000.0 + 0.2)
        fb = _motor_dwell_and_feedback(leg, dwell_total)

        print(f"    Motor cmd  θA={fb['tA_cmd']:+.2f}°  θB={fb['tB_cmd']:+.2f}°")
        print(f"    Motor act  θA={fb['tA_act']:+.2f}°  θB={fb['tB_act']:+.2f}°")

        # ─── รอให้ภาพนิ่ง แล้วจับ ArUco ─────────────────────────────
        time.sleep(CAPTURE_SETTLE_S)
        markers, frame = _grab_aruco_sample(
            cap, detector, camera_matrix, dist_coeffs,
            n_frames=CAPTURE_N_FRAMES, flush_n=CAPTURE_FLUSH_N,
        )

        # ─── คำนวณ actual position ───────────────────────────────────
        transform = None
        video_x = video_y = float('nan')

        if ARUCO_A_ID in markers and ARUCO_B_ID in markers:
            transform = _build_leg_transform(markers[ARUCO_A_ID], markers[ARUCO_B_ID])

        if transform is not None and ARUCO_E_ID in markers:
            video_x, video_y = _px_to_legframe(markers[ARUCO_E_ID], transform)

        err_mm = (np.sqrt((video_x - tx)**2 + (video_y - ty)**2)
                  if not (np.isnan(video_x) or np.isnan(video_y)) else float('nan'))

        print(f"    ArUco markers detected: {sorted(markers.keys())}")
        if not np.isnan(video_x):
            print(f"    Actual pos  x={video_x:+.2f}  y={video_y:+.2f} mm  |  err={err_mm:.2f} mm")
        else:
            print(f"    Actual pos  N/A (E marker not detected)")

        # ─── แสดงภาพ annotated ───────────────────────────────────────
        if SHOW_CAPTURE_WIN and frame is not None:
            _show_capture_frame(
                frame, markers, transform,
                pid=target_id, target_xy=(tx, ty),
                video_xy=(video_x, video_y),
                point_num=1, total_points=1,
                motor_feedback=fb,
            )

        # ─── บันทึก grid_log ─────────────────────────────────────────
        _write_grid_log(
            target_id, tx, ty,
            fb['tA_cmd'], fb['tB_cmd'],
            fb['tA_act'], fb['tB_act'],
            video_x=video_x, video_y=video_y,
        )
        return ok

    finally:
        if _own_cap:
            cap.release()
            cv2.destroyAllWindows()


def _write_grid_log(
    point_id: int,
    target_x: float, target_y: float,
    tA_cmd: float, tB_cmd: float,
    tA_act: float, tB_act: float,
    video_x: float | None = None,
    video_y: float | None = None,
    log_path: str = GRID_LOG_FILE,
) -> None:
    """
    บันทึกผล grid/capture ลง CSV (append mode)
    สร้าง header อัตโนมัติถ้ายังไม่มีไฟล์

    Columns:
        point_id, target_x_mm, target_y_mm,
        cmd_thetaA_deg, cmd_thetaB_deg,
        act_thetaA_deg, act_thetaB_deg,
        video_act_x_mm, video_act_y_mm,
        video_err_x_mm, video_err_y_mm, video_err_dist_mm
    """
    import csv as _csv
    _FIELDNAMES = [
        'point_id', 'target_x_mm', 'target_y_mm',
        'cmd_thetaA_deg', 'cmd_thetaB_deg',
        'act_thetaA_deg', 'act_thetaB_deg',
        'video_act_x_mm', 'video_act_y_mm',
        'video_err_x_mm', 'video_err_y_mm', 'video_err_dist_mm',
    ]

    _vx = video_x if video_x is not None else float('nan')
    _vy = video_y if video_y is not None else float('nan')
    if not (np.isnan(_vx) or np.isnan(_vy)):
        _ex   = _vx - target_x
        _ey   = _vy - target_y
        _edst = float(np.sqrt(_ex**2 + _ey**2))
    else:
        _ex = _ey = _edst = float('nan')

    def _fmt(v): return '' if np.isnan(v) else f'{v:.4f}'

    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    write_header = not os.path.isfile(log_path)
    try:
        with open(log_path, 'a', newline='', encoding='utf-8') as _f:
            _w = _csv.DictWriter(_f, fieldnames=_FIELDNAMES)
            if write_header:
                _w.writeheader()
            _w.writerow({
                'point_id':          point_id,
                'target_x_mm':       f'{target_x:.4f}',
                'target_y_mm':       f'{target_y:.4f}',
                'cmd_thetaA_deg':    f'{tA_cmd:.4f}',
                'cmd_thetaB_deg':    f'{tB_cmd:.4f}',
                'act_thetaA_deg':    f'{tA_act:.4f}',
                'act_thetaB_deg':    f'{tB_act:.4f}',
                'video_act_x_mm':    _fmt(_vx),
                'video_act_y_mm':    _fmt(_vy),
                'video_err_x_mm':    _fmt(_ex),
                'video_err_y_mm':    _fmt(_ey),
                'video_err_dist_mm': _fmt(_edst),
            })
        _vid_str = (f"  vid ({_vx:+.2f},{_vy:+.2f}) mm  err={_edst:.2f} mm"
                    if not np.isnan(_vx) else "")
        print(f"  📄 log ID={point_id}  "
              f"cmd θA={tA_cmd:+.2f}° θB={tB_cmd:+.2f}°  "
              f"act θA={tA_act:+.2f}° θB={tB_act:+.2f}°"
              f"{_vid_str}")
    except Exception as _e:
        print(f"  ⚠️  เขียน log ล้มเหลว: {_e}")


def run_capture_sweep(
    leg: 'SingleLegController',
    csv_path: str  = GRID_FILE,
    dwell_s: float = GRID_DWELL_S,
    start_id: int  = 1,
) -> None:
    """
    ไล่ตำแหน่งตาม calibration grid CSV เหมือน run_grid_sweep
    แต่เพิ่มการวัด actual position ด้วยกล้องและ ArUco markers
    บันทึก video_act_x/y + error ลง grid_log.csv ขณะทำงาน
    แสดงภาพ annotated พร้อม info ใน window (ไม่บันทึกรูป)

    ArUco IDs:
        0 = Motor A  (reference — fixed)
        1 = Motor B  (reference — fixed)
        4 = End-effector E  (target ที่วัด)
    """
    if not _CV2_OK:
        print("  ❌ capture mode ต้องการ opencv-python")
        print("     pip install opencv-python opencv-contrib-python")
        return

    import csv as _csv

    # ─── โหลด CSV ────────────────────────────────────────────────────
    try:
        with open(csv_path, newline='', encoding='utf-8') as f:
            reader = _csv.DictReader(f)
            points = [
                (int(row['point_id']) if 'point_id' in row else idx + 1,
                 float(row['target_x_mm']), float(row['target_y_mm']))
                for idx, row in enumerate(reader)
            ]
    except FileNotFoundError:
        print(f"  ❌ ไม่พบไฟล์: {csv_path}")
        return
    except Exception as e:
        print(f"  ❌ โหลด CSV ล้มเหลว: {e}")
        return

    if not points:
        print("  ❌ ไม่มีข้อมูลใน CSV")
        return

    points = sorted([p for p in points if p[0] >= start_id], key=lambda p: p[0])
    total  = len(points)
    if total == 0:
        print(f"  ❌ ไม่มีจุดที่ point_id >= {start_id}")
        return

    # ─── เปิดกล้องและ ArUco detector ────────────────────────────────
    camera_matrix, dist_coeffs = _load_calib(CALIB_NPZ)
    cap = _open_camera(CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT)
    if cap is None:
        print(f"  ❌ ไม่สามารถเปิดกล้อง index={CAMERA_INDEX}")
        print("     ลองเปลี่ยน CAMERA_INDEX ใน config section")
        return

    aruco_dict   = _cv2_aruco.getPredefinedDictionary(_cv2_aruco.DICT_6X6_250)
    aruco_params = _cv2_aruco.DetectorParameters()
    detector     = _cv2_aruco.ArucoDetector(aruco_dict, aruco_params)

    # ─── แสดงสรุปก่อนเริ่ม ──────────────────────────────────────────
    print(f"\n  ─── Capture Sweep ──────────────────────────────")
    print(f"    File     : {csv_path}")
    print(f"    Points   : {total} จุด (เริ่ม ID={points[0][0]})")
    print(f"    Dwell    : {dwell_s:.1f} s/จุด")
    print(f"    Camera   : index={CAMERA_INDEX}  settle={CAPTURE_SETTLE_S}s  "
          f"frames={CAPTURE_N_FRAMES}")
    print(f"    Calib    : {'✅ โหลดแล้ว' if camera_matrix is not None else '⚠️ ไม่พบ (ไม่ undistort)'}")
    print(f"    ArUco    : E=ID{ARUCO_E_ID}  A=ID{ARUCO_A_ID}  B=ID{ARUCO_B_ID}")
    print(f"    Log      : {GRID_LOG_FILE}")
    print(f"  ────────────────────────────────────────────────")
    print("  กด Ctrl+C เพื่อหยุดกลางคัน\n")

    # ─── cached transform จาก marker A,B ────────────────────────────
    _transform_cache = None

    n_ok_move  = 0
    n_ok_video = 0
    errors_mm  = []

    WIN_NAME = 'Capture Mode — ArUco Position Tracking  [q=stop]'

    try:
        for idx, (pid, tx, ty) in enumerate(points, start=1):
            print(f"\n  [{idx:3d}/{total}] ID={pid:3d}  target=({tx:+6.1f}, {ty:+7.1f}) mm",
                  end='  ', flush=True)

            # ── เคลื่อนที่ ──────────────────────────────────────────
            ok_move = leg.move_to(tx, ty, force=True)
            if ok_move:
                n_ok_move += 1

            # ── รอ dwell + อ่าน feedback ─────────────────────────
            dwell_total = max(dwell_s, SCURVE_DURATION_MS / 1000.0 + 0.2)
            fb = _motor_dwell_and_feedback(leg, dwell_total)

            # ── รอภาพนิ่ง ────────────────────────────────────────
            time.sleep(CAPTURE_SETTLE_S)

            # ── จับ ArUco ─────────────────────────────────────────
            markers, frame = _grab_aruco_sample(
                cap, detector, camera_matrix, dist_coeffs,
                n_frames=CAPTURE_N_FRAMES, flush_n=CAPTURE_FLUSH_N,
            )

            # ── คำนวณ transform จาก A,B ──────────────────────────
            if ARUCO_A_ID in markers and ARUCO_B_ID in markers:
                t = _build_leg_transform(markers[ARUCO_A_ID], markers[ARUCO_B_ID])
                if t is not None:
                    _transform_cache = t

            # ── คำนวณ actual position ─────────────────────────────
            video_x = video_y = float('nan')
            if _transform_cache is not None and ARUCO_E_ID in markers:
                video_x, video_y = _px_to_legframe(
                    markers[ARUCO_E_ID], _transform_cache
                )
                n_ok_video += 1
                err_mm = float(np.sqrt((video_x - tx)**2 + (video_y - ty)**2))
                errors_mm.append(err_mm)
                print(f"E=({video_x:+.1f},{video_y:+.1f}) err={err_mm:.2f}mm", flush=True)
            else:
                detected_str = str(sorted(markers.keys()))
                print(f"E not detected  markers={detected_str}", flush=True)

            # ── แสดงภาพ annotated ─────────────────────────────────
            if SHOW_CAPTURE_WIN and frame is not None:
                key = _show_capture_frame(
                    frame, markers, _transform_cache,
                    pid=pid, target_xy=(tx, ty),
                    video_xy=(video_x, video_y),
                    point_num=idx, total_points=total,
                    motor_feedback=fb,
                )
                if key == ord('q'):
                    print("\n  ⏹️  ยกเลิกโดยกด q ในหน้าต่างภาพ")
                    break

            # ── บันทึก log ────────────────────────────────────────
            _write_grid_log(
                pid, tx, ty,
                fb['tA_cmd'], fb['tB_cmd'],
                fb['tA_act'], fb['tB_act'],
                video_x=video_x, video_y=video_y,
            )

            # ── ตรวจ keyboard interrupt ───────────────────────────
            if sys.platform == 'win32' and msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'q', b'Q'):
                    print("\n  ⏹️  ยกเลิกโดยกด q")
                    break

    except KeyboardInterrupt:
        print("\n  ⏹️  ยกเลิกโดย Ctrl+C")
    finally:
        cap.release()
        if SHOW_CAPTURE_WIN:
            cv2.destroyWindow(WIN_NAME)

    # ─── สรุปผล ──────────────────────────────────────────────────────
    print(f"\n  ─── สรุปผล Capture Sweep ──────────────────────")
    print(f"    จุดทั้งหมด    : {total}")
    print(f"    เคลื่อนที่ได้  : {n_ok_move}")
    print(f"    วัดได้ (ArUco): {n_ok_video}")
    if errors_mm:
        print(f"    Error mean    : {np.mean(errors_mm):.2f} mm")
        print(f"    Error max     : {np.max(errors_mm):.2f} mm")
        print(f"    Error min     : {np.min(errors_mm):.2f} mm")
    print(f"    Log บันทึกที่  : {GRID_LOG_FILE}")
    print(f"  ────────────────────────────────────────────────")

    print("\n  กลับ home position...")
    leg.go_home()





def run_grid_sweep(
    leg: 'SingleLegController',
    csv_path: str = GRID_FILE,
    dwell_s: float = GRID_DWELL_S,
    auto: bool = GRID_AUTO,
    start_id: int = 1,
) -> None:
    """
    ไล่ตำแหน่งตาม calibration grid CSV ตามลำดับ point_id

    Args:
        leg      : SingleLegController ที่เชื่อมต่อแล้ว
        csv_path : path ของ CSV ไฟล์  (columns: point_id, target_x_mm, target_y_mm)
        dwell_s  : เวลาหยุดที่แต่ละจุด (วินาที)  ใช้เมื่อ auto=True
        auto     : True = เดินหน้าอัตโนมัติ, False = รอกด Enter
        start_id : point_id แรกที่ต้องการเริ่ม (ข้ามจุดก่อนหน้า)
    """
    import csv as _csv

    # ─── โหลด CSV ────────────────────────────────────────────────────
    try:
        with open(csv_path, newline='', encoding='utf-8') as f:
            reader = _csv.DictReader(f)
            points = [
                (int(row['point_id']) if 'point_id' in row else idx + 1,
                 float(row['target_x_mm']), float(row['target_y_mm']))
                for idx, row in enumerate(reader)
            ]
    except FileNotFoundError:
        print(f"  ❌ ไม่พบไฟล์: {csv_path}")
        return
    except Exception as e:
        print(f"  ❌ โหลด CSV ล้มเหลว: {e}")
        return

    if not points:
        print("  ❌ ไม่มีข้อมูลใน CSV")
        return

    # กรองจุดที่ point_id >= start_id แล้วเรียงลำดับ
    points = sorted([p for p in points if p[0] >= start_id], key=lambda p: p[0])
    total  = len(points)

    if total == 0:
        print(f"  ❌ ไม่มีจุดที่ point_id >= {start_id}")
        return

    mode_str = f'auto  (dwell={dwell_s:.1f} s)' if auto else 'manual (Enter to advance)'
    print(f"\n  ─── Grid Sweep ─────────────────────────────────")
    print(f"    File    : {csv_path}")
    print(f"    Points  : {total} จุด (เริ่มที่ ID={points[0][0]})")
    print(f"    Mode    : {mode_str}")
    print(f"    Profile : {'S-Curve' if leg.use_scurve else 'Direct'}")
    print(f"  ────────────────────────────────────────────────")
    if not auto:
        print("  กด Enter เพื่อไปจุดถัดไป  |  พิมพ์ q + Enter เพื่อหยุด")
    else:
        print("  กด Ctrl+C เพื่อหยุดกลางคัน")
    print()

    failed = []
    skipped = []

    try:
        for idx, (pid, tx, ty) in enumerate(points, start=1):
            print(f"  [{idx:3d}/{total}] ID={pid:3d}  target=({tx:+6.1f}, {ty:+7.1f}) mm", end='  ')

            ok = leg.move_to(tx, ty, force=True)
            if not ok:
                failed.append(pid)

            # บันทึกมุมที่สั่ง จาก IK state ล่าสุด
            if _ik_prev_angles_rad is not None:
                tA_cmd = float(np.rad2deg(_ik_prev_angles_rad[0]))
                tB_cmd = float(np.rad2deg(_ik_prev_angles_rad[1]))
            else:
                tA_cmd = tB_cmd = float('nan')

            _should_break = False
            if auto:
                if ok:
                    time.sleep(dwell_s)   # รอ settle เฉพาะเมื่อ IK สำเร็จ
            else:
                if ok:
                    try:
                        ans = input()
                        if ans.strip().lower() == 'q':
                            print("  ⏹️  ยกเลิกโดย user")
                            _should_break = True
                    except (EOFError, KeyboardInterrupt):
                        print()
                        _should_break = True

            # อ่าน actual angles หลัง dwell แล้วเขียน log (ทุกจุด รวมถึง IK ล้มเหลว)
            if ok:
                fb_a = leg.motor_a.ping()
                fb_b = leg.motor_b.ping()
                leg.motor_a.set_timeout(FAST_TIMEOUT)
                leg.motor_b.set_timeout(FAST_TIMEOUT)
                actual_a = fb_a['position'] if fb_a else float('nan')
                actual_b = fb_b['position'] if fb_b else float('nan')
            else:
                actual_a = actual_b = float('nan')
            _write_grid_log(pid, tx, ty, tA_cmd, tB_cmd, actual_a, actual_b)

            if _should_break:
                break

    except KeyboardInterrupt:
        print("\n  ⏹️  ยกเลิกโดย Ctrl+C")

    # ─── สรุปผล ──────────────────────────────────────────────────────
    print()
    print(f"  ─── สรุปผล Grid Sweep ──────────────────────────")
    print(f"    จำนวนจุดทั้งหมด : {total}")
    print(f"    สำเร็จ           : {total - len(failed) - len(skipped)}")
    if failed:
        print(f"    IK ล้มเหลว      : {len(failed)} จุด  → ID {failed}")
    print(f"  ────────────────────────────────────────────────")

    # กลับ home หลังจบ sweep
    print("\n  กลับ home position...")
    leg.go_home()


# ============================================================================
# CIRCLE PATH MODE
# ============================================================================

def _build_poly_features(tA: float, tB: float, degree: int) -> np.ndarray:
    """
    สร้าง polynomial features แบบเดียวกับ sklearn PolynomialFeatures(include_bias=False)
    สำหรับ 2 ตัวแปร [tA, tB]

    degree=2 → [tA, tB, tA², tA·tB, tB²]
    degree=3 → [tA, tB, tA², tA·tB, tB², tA³, tA²tB, tA·tB², tB³]
    degree=4 → ... (14 features)
    """
    feats = []
    for d in range(1, degree + 1):
        for i in range(d + 1):           # i = power of tB, d-i = power of tA
            feats.append(tA ** (d - i) * tB ** i)
    return np.array(feats)


def _list_models() -> list[str]:
    """คืนรายชื่อโมเดลใน COMPENSATION_MODEL_DIR (ไม่มี extension, ไม่ซ้ำ, เรียงลำดับ)"""
    names: set[str] = set()
    if os.path.isdir(COMPENSATION_MODEL_DIR):
        for f in os.listdir(COMPENSATION_MODEL_DIR):
            if f.endswith(('.pkl', '.json')):
                names.add(os.path.splitext(f)[0])
    return sorted(names)


def print_models() -> None:
    """แสดงรายการโมเดลที่มีใน COMPENSATION_MODEL_DIR พร้อม format และ default"""
    names = _list_models()
    if not names:
        print(f"  ⚠️  ไม่พบโมเดลใน output/models/  (ยังไม่ได้รัน train_compensation_model.py)")
        return
    print(f"\n  ─── โมเดลที่มีใน output/models/ ───────────────────────")
    for name in names:
        has_pkl  = os.path.isfile(os.path.join(COMPENSATION_MODEL_DIR, name + '.pkl'))
        has_json = os.path.isfile(os.path.join(COMPENSATION_MODEL_DIR, name + '.json'))
        fmts = []
        if has_pkl:  fmts.append('pkl')
        if has_json: fmts.append('json')
        default = '  ← default' if name == COMPENSATION_MODEL_NAME else ''
        print(f"    {name:<30}  [{', '.join(fmts)}]{default}")
    print(f"  ────────────────────────────────────────────────────")
    print(f"  ใช้งาน:  circle comp <ชื่อโมเดล>  เช่น  circle comp model_mlp\n")


def load_compensation_model(name: str = COMPENSATION_MODEL_NAME) -> dict | None:
    """
    โหลด compensation model จาก output/models/
    รองรับ .pkl (sklearn Pipeline) และ .json (polynomial coefficients)

    Args:
        name: ชื่อโมเดล (ไม่ต้องใส่ extension) หรือ full path
              เช่น 'model_poly4', 'model_mlp', 'model_svr_rbf'

    Returns:
        dict พร้อม key 'type' = 'pkl' หรือ 'json'
        - pkl: {'type': 'pkl', 'model_name': str, 'model_x': Pipeline,
                'model_y': Pipeline, 'metrics': dict}
        - json: {'type': 'json', 'model_name': str, 'polynomial_degree': int,
                 'feature_names': list, 'model_x': dict, 'model_y': dict, ...}
        หรือ None ถ้าโหลดไม่สำเร็จ
    """
    import json as _json

    # Resolve path: ถ้าไม่ใช่ absolute path ให้หาในโฟลเดอร์ models
    if os.path.isabs(name) or (os.sep in name) or ('/' in name):
        base = os.path.splitext(name)[0]  # strip extension ถ้ามี
    else:
        stem = os.path.splitext(name)[0]  # เผื่อ user ใส่ .pkl/.json มาด้วย
        base = os.path.join(COMPENSATION_MODEL_DIR, stem)

    pkl_path  = base + '.pkl'
    json_path = base + '.json'

    # ─── ลอง .pkl ก่อน ──────────────────────────────────────────────
    if os.path.isfile(pkl_path):
        try:
            import joblib
            data: dict = joblib.load(pkl_path)
            data['type'] = 'pkl'
            model_name = data.get('model_name', os.path.basename(pkl_path))
            metrics    = data.get('metrics', {})
            print(f"  ✅ โหลดโมเดล: {model_name}  [pkl / sklearn Pipeline]")
            if metrics:
                print(f"     RMSE X={metrics.get('rmse_x', float('nan')):.4f} mm  "
                      f"R²X={metrics.get('r2_x', float('nan')):.4f}  "
                      f"RMSE Y={metrics.get('rmse_y', float('nan')):.4f} mm  "
                      f"R²Y={metrics.get('r2_y', float('nan')):.4f}")
            return data
        except Exception as e:
            print(f"  ⚠️  โหลด pkl ล้มเหลว: {e}")

    # ─── ลอง .json ───────────────────────────────────────────────────
    if os.path.isfile(json_path):
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = _json.load(f)
            data['type'] = 'json'
            model_name = data.get('model_name', os.path.basename(json_path))
            deg        = data.get('polynomial_degree', '?')
            features   = data.get('feature_names', [])
            print(f"  ✅ โหลดโมเดล: {model_name}  [json / poly deg={deg}]")
            print(f"     Features: {features}")
            mx = data.get('model_x', {})
            my = data.get('model_y', {})
            print(f"     model_x: intercept={mx.get('intercept', float('nan')):+.4f}  "
                  f"coef (first 5)={mx.get('coef', [])[:5]}")
            print(f"     model_y: intercept={my.get('intercept', float('nan')):+.4f}  "
                  f"coef (first 5)={my.get('coef', [])[:5]}")
            return data
        except Exception as e:
            print(f"  ⚠️  โหลด json ล้มเหลว: {e}")

    # ─── ไม่พบ ────────────────────────────────────────────────────────
    print(f"  ⚠️  ไม่พบโมเดล '{os.path.basename(base)}'  ใน {COMPENSATION_MODEL_DIR}")
    print(f"     พิมพ์ 'models' เพื่อดูรายการโมเดลที่มี")
    return None


def predict_kinematic_error(
    thetaA_deg: float, thetaB_deg: float, model: dict
) -> tuple[float, float]:
    """
    ทำนาย kinematic error จากมุมมอเตอร์
    รองรับทั้ง sklearn Pipeline (.pkl) และ polynomial coefficients (.json)

    Args:
        thetaA_deg: มุม Motor A (degrees)
        thetaB_deg: มุม Motor B (degrees)
        model: dict ที่โหลดจาก load_compensation_model()

    Returns:
        (err_x, err_y) — ค่า error ที่คาดการณ์ (mm) สำหรับชดเชย
    """
    if model.get('type') == 'pkl':
        # sklearn Pipeline: รองรับ Poly, SVR, Random Forest, MLP ฯลฯ
        X = [[thetaA_deg, thetaB_deg]]
        err_x = float(model['model_x'].predict(X)[0])
        err_y = float(model['model_y'].predict(X)[0])
        return err_x, err_y
    else:
        # JSON polynomial model
        degree = model.get('polynomial_degree', 2)
        phi    = _build_poly_features(thetaA_deg, thetaB_deg, degree)
        mx     = model['model_x']
        my     = model['model_y']
        err_x  = mx['intercept'] + float(np.dot(mx['coef'], phi))
        err_y  = my['intercept'] + float(np.dot(my['coef'], phi))
        return err_x, err_y


def run_circle_path(
    leg: 'SingleLegController',
    cx:          float = CIRCLE_CENTER_X,
    cy:          float = CIRCLE_CENTER_Y,
    radius:      float = CIRCLE_RADIUS,
    n_points:    int   = CIRCLE_POINTS,
    revolutions: int   = CIRCLE_REVS,
    dwell_s:     float = CIRCLE_DWELL_S,
    freq_hz:     float = CIRCLE_FREQ_HZ,
    compensate:  bool  = CIRCLE_COMPENSATION,
    model_name:  str | None = None,   # None = ใช้ COMPENSATION_MODEL_NAME
) -> None:
    """เคลื่อนที่ตาม path วงกลม

    X(θ) = cx + R·cos(θ)
    Y(θ) = cy + R·sin(θ)
    θ ไล่จาก 0 → 2π × revolutions (ทวนเข็มนาฬิกาในระบบพิกัดมาตรฐาน)

    freq_hz > 0  : ควบคุมความเร็ว (รอบ/วินาที) — S-Curve duration คำนวณอัตโนมัติ
    freq_hz = 0  : ใช้ SCURVE_DURATION_MS + dwell_s แทน
    compensate   : True = เปิด feed-forward kinematic error compensation
    model_name   : ชื่อโมเดล เช่น 'model_poly4', 'model_mlp'  (None = ใช้ default)
    """
    total_pts = n_points * revolutions
    step_rad  = 2 * np.pi / n_points

    # คำนวณ timing
    if freq_hz > 0:
        step_s  = 1.0 / (freq_hz * n_points)        # วินาทีต่อจุด
        step_ms = max(20, int(step_s * 1000))        # S-Curve duration (ms), ขั้นต่ำ 20 ms
    else:
        step_s  = max(dwell_s, SCURVE_DURATION_MS / 1000.0 if leg.use_scurve else 0.05)
        step_ms = SCURVE_DURATION_MS

    print(f"\n  ─── Circle Path ───────────────────────────────")
    print(f"    Center     : ({cx:+.1f}, {cy:+.1f}) mm")
    print(f"    Radius     : {radius:.1f} mm")
    print(f"    Points     : {n_points} ต่อรอบ  ×{revolutions} รอบ  = {total_pts} จุด")
    if freq_hz > 0:
        print(f"    Frequency  : {freq_hz:.2f} Hz  ({step_s*1000:.0f} ms/จุด, S-Curve {step_ms} ms)")
    else:
        print(f"    Wait/point : {step_s*1000:.0f} ms")
    print(f"    Profile    : {'S-Curve' if leg.use_scurve else 'Direct'}")
    _eff_model = model_name or COMPENSATION_MODEL_NAME
    print(f"    Compensate : {'✅ เปิด' if compensate else '❌ ปิด'}"
          + (f"  [{_eff_model}]" if compensate else ''))
    print(f"  ────────────────────────────────────────────────")
    print(f"  กด Ctrl+C เพื่อหยุด\n")

    # โหลด compensation model ถ้าเปิดใช้งาน
    comp_model: dict | None = None
    if compensate:
        comp_model = load_compensation_model(_eff_model)
        if comp_model is None:
            print("  ⚠️  โหลดโมเดลล้มเหลว — วิ่งโดยไม่มี compensation\n")

    # ตรวจสอบทุกจุดก่อนเริ่ม
    reachable = 0
    for i in range(total_pts):
        theta = step_rad * i
        x = cx + radius * np.cos(theta)
        y = cy + radius * np.sin(theta)
        if calculate_ik(np.array([x, y])) is not None:
            reachable += 1
    if reachable < total_pts:
        print(f"  ⚠️  {total_pts - reachable}/{total_pts} จุด อยู่นอก workspace — ดำเนินการต่อ? (y/n)")
        if input("  > ").strip().lower() not in ('y', 'yes'):
            print("  ยกเลิก")
            return

    done = 0
    failed = 0
    comp_applied = 0
    try:
        for i in range(total_pts):
            theta   = step_rad * i
            ideal_x = cx + radius * np.cos(theta)
            ideal_y = cy + radius * np.sin(theta)

            # ─── Feed-forward Kinematic Error Compensation ──────────
            if compensate and comp_model is not None:
                # Step 2: IK รอบที่ 1 — หามุมอุดมคติ
                ik_ideal = calculate_ik([ideal_x, ideal_y])
                if ik_ideal is not None:
                    ideal_tA, ideal_tB = ik_ideal
                    # Step 3: ทำนาย error จากมุมอุดมคติ
                    err_x, err_y = predict_kinematic_error(
                        ideal_tA, ideal_tB, comp_model)
                    # Step 4: พิกัดที่ชดเชยแล้ว
                    target_x = ideal_x - err_x
                    target_y = ideal_y - err_y
                    comp_applied += 1
                else:
                    target_x, target_y = ideal_x, ideal_y
            else:
                target_x, target_y = ideal_x, ideal_y
            # ────────────────────────────────────────────────────────

            sys.stdout.write(
                f"\r  [{i+1:4d}/{total_pts}]  θ={np.degrees(theta):+7.2f}°  "
                f"ideal=({ideal_x:+7.2f},{ideal_y:+7.2f})  "
                f"cmd=({target_x:+7.2f},{target_y:+7.2f}) mm  "
            )
            sys.stdout.flush()

            # Step 5: IK รอบที่ 2 (ผ่าน leg.move_to) + ส่งคำสั่งมอเตอร์
            ok = leg.move_to(target_x, target_y,
                             duration_ms=step_ms if leg.use_scurve else None)
            if ok is False:
                failed += 1
                sys.stdout.write("[IK fail]")
                sys.stdout.flush()
            else:
                done += 1
                time.sleep(step_s)

    except KeyboardInterrupt:
        print("\n\n  ⛔ หยุดโดย Ctrl+C")

    print(f"\n\n  ─── สรุป Circle Path ──────────────────────────")
    print(f"    สำเร็จ    : {done}/{total_pts}")
    if compensate and comp_model is not None:
        print(f"    ชดเชยแล้ว : {comp_applied}/{total_pts} จุด")
    if failed:
        print(f"    IK ล้มเหลว: {failed} จุด")
    print(f"  ────────────────────────────────────────────────")


def run_interactive(leg: SingleLegController):
    """Main interactive command loop"""
    print(HELP_TEXT)

    while True:
        try:
            cmd = input("  cmd> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue

        # ─── ออก ─────────────────────────────────────────────────────
        if cmd in ('q', 'quit', 'exit'):
            print("  Exiting...")
            break

        # ─── Help ────────────────────────────────────────────────────
        elif cmd in ('h', 'help'):
            print(HELP_TEXT)

        # ─── Home ────────────────────────────────────────────────────
        elif cmd == 'home':
            leg.go_home()

        # ─── Init angle ──────────────────────────────────────────────
        elif cmd == 'init':
            leg.go_init()

        # ─── Status ──────────────────────────────────────────────────
        elif cmd in ('status', 'st'):
            leg.print_status()

        # ─── Info ────────────────────────────────────────────────────
        elif cmd in ('info', 'ws'):
            print_workspace_info()

        # ─── Toggle profile ──────────────────────────────────────────
        elif cmd == 's':
            leg.toggle_profile()

        # ─── Scan COM ports ──────────────────────────────────────────
        elif cmd == 'scan':
            ports = list_com_ports()
            print(f"\n  COM ports ({len(ports)}):")
            for p in ports:
                print(f"    • {p}")

        # ─── Capture mode (ArUco-based position measurement) ───────────
        elif cmd == 'capture':
            run_capture_sweep(leg)

        # ─── Circle path ──────────────────────────────────────────────
        elif cmd == 'circle' or cmd.startswith('circle '):
            parts = cmd.split()
            # แยก flag comp / nocomp / model_<name> ก่อน parse ตัวเลข
            _model_name = None
            if 'comp' in parts:
                _compensate = True
                parts = [p for p in parts if p != 'comp']
            elif 'nocomp' in parts:
                _compensate = False
                parts = [p for p in parts if p != 'nocomp']
            else:
                _compensate = CIRCLE_COMPENSATION
            # ดึง token ที่เป็นชื่อโมเดล (ขึ้นต้นด้วย model_)
            num_parts = []
            for _p in parts:
                if _p.startswith('model_'):
                    _model_name = _p
                else:
                    num_parts.append(_p)
            parts = num_parts
            try:
                if len(parts) == 1:
                    run_circle_path(leg, compensate=_compensate,
                                    model_name=_model_name)
                elif len(parts) == 2:
                    run_circle_path(leg, radius=float(parts[1]),
                                    compensate=_compensate,
                                    model_name=_model_name)
                elif len(parts) == 4:
                    run_circle_path(leg, radius=float(parts[1]),
                                    cx=float(parts[2]), cy=float(parts[3]),
                                    compensate=_compensate,
                                    model_name=_model_name)
                elif len(parts) == 5:
                    run_circle_path(leg, radius=float(parts[1]),
                                    cx=float(parts[2]), cy=float(parts[3]),
                                    revolutions=int(parts[4]),
                                    compensate=_compensate,
                                    model_name=_model_name)
                else:
                    print("  ⚠️  รูปแบบ: circle [R [cx cy [N]]] [comp|nocomp] [model_<ชื่อ>]")
            except ValueError:
                print("  ⚠️  ค่าพารามิเตอร์ไม่ถูกต้อง  เช่น  circle 25 0 -200 comp model_mlp")

        # ─── Models list ─────────────────────────────────────────────
        elif cmd == 'models':
            print_models()

        # ─── Grid sweep ───────────────────────────────────────────────
        elif cmd == 'grid':
            run_grid_sweep(leg)

        # ─── Emergency Stop ──────────────────────────────────────────
        elif cmd in ('e', 'estop', 'stop'):
            leg.emergency_stop()

        # ─── XY position command ─────────────────────────────────────
        else:
            parsed = parse_xy(cmd)
            if parsed is not None:
                x, y = parsed
                leg.move_to(x, y)
            else:
                print(f"  ⚠️  ไม่รู้จักคำสั่ง '{cmd}'  (พิมพ์ help เพื่อดูคำสั่ง)")


# ============================================================================
# MAIN
# ============================================================================

def main():
    print("=" * 65)
    print("  BLEGS Single Leg XY Position Control")
    print("=" * 65)
    print(f"  Leg side    : {LEG_SIDE}")
    print(f"  Home pos    : ({HOME_X:+.1f}, {HOME_Y:+.1f}) mm")
    print(f"  Profile     : {'S-Curve' if USE_SCURVE else 'Direct'}")
    print(f"  S-Curve dur : {SCURVE_DURATION_MS} ms")
    print("=" * 65)

    # ─── 1. Determine COM ports ──────────────────────────────────────
    global COM_PORT_A, COM_PORT_B

    if COM_PORT_A is None or COM_PORT_B is None:
        ports = list_com_ports()
        print(f"\n  COM ports ที่พบ ({len(ports)}):")
        for p in ports:
            print(f"    • {p}")

        if len(ports) == 0:
            print("\n  ❌ ไม่พบ COM port – กรุณาเชื่อมต่ออุปกรณ์แล้วลองใหม่")
            sys.exit(1)
        elif len(ports) == 1:
            print("\n  ⚠️  พบแค่ 1 port – ต้องการ 2 port สำหรับ Motor A และ B")
            cont = input("  ต้องการใช้ port เดียวกันทั้งคู่? (y/N): ").strip().lower()
            if cont == 'y':
                COM_PORT_A = COM_PORT_B = ports[0]
            else:
                print("  กรุณาเชื่อมต่อ motor ทั้งสอง แล้วรันใหม่")
                sys.exit(1)
        elif COM_PORT_A is None and COM_PORT_B is None:
            # Auto-assign if exactly 2 ports, otherwise ask user
            if len(ports) == 2:
                COM_PORT_A, COM_PORT_B = ports[0], ports[1]
                print(f"\n  Auto-assigned:")
                print(f"    Motor A → {COM_PORT_A}")
                print(f"    Motor B → {COM_PORT_B}")
                confirm = input("  ยืนยัน? (Y/n): ").strip().lower()
                if confirm == 'n':
                    COM_PORT_A, COM_PORT_B = select_ports_interactively()
            else:
                COM_PORT_A, COM_PORT_B = select_ports_interactively()
        else:
            # One of them is None – fill in the missing one
            COM_PORT_A, COM_PORT_B = select_ports_interactively()

    print(f"\n  Motor A → {COM_PORT_A}")
    print(f"  Motor B → {COM_PORT_B}")

    # ─── 2. Connect ──────────────────────────────────────────────────
    leg = SingleLegController(COM_PORT_A, COM_PORT_B)

    if not leg.connect():
        print("\n  ❌ Connection failed")
        sys.exit(1)

    # ─── 3. PING / initialize ────────────────────────────────────────
    print("\n  Initializing motors (PING)...")
    time.sleep(0.5)
    ok = leg.ping_both()
    if not ok:
        print("\n  ⚠️  บางมอเตอร์ไม่ตอบสนอง – กดต่อเพื่อดำเนินการต่อหรือ Ctrl+C เพื่อออก")
        try:
            input("  กด Enter เพื่อดำเนินการต่อ: ")
        except KeyboardInterrupt:
            leg.disconnect()
            sys.exit(0)

    print("\n  ⏳ รอ 3 วินาทีให้ motor พร้อม...")
    time.sleep(3.0)

    # ─── 4. Move to home ─────────────────────────────────────────────
    print_workspace_info()
    print("\n  Moving to home position...")
    leg.go_home()
    time.sleep(0.8)

    # ─── 5. Interactive loop ─────────────────────────────────────────
    print("\n  ✅ พร้อมรับคำสั่ง (พิมพ์ 'help' เพื่อดูคำสั่งทั้งหมด)")
    try:
        run_interactive(leg)
    except KeyboardInterrupt:
        print("\n  Interrupted")
    finally:
        # ─── 6. Cleanup ──────────────────────────────────────────────
        print("\n  Cleaning up...")
        if leg.connected:
            leg.go_init()
            time.sleep(1.2)
            leg.disconnect()
        print("  ✅ Done")


if __name__ == '__main__':
    main()
