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
GRID_FILE = r'C:\Users\mteer\OneDrive\Desktop\calibration_grid.csv'
GRID_DWELL_S = 1.5          # เวลาหยุดที่แต่ละจุด (วินาที) ในโหมด auto
GRID_AUTO = True            # True = auto advance, False = กด Enter เพื่อไปจุดถัดไป

# Capture mode
CAPTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'capture_image')
CAPTURE_TIMEOUT_S = 20.0    # วินาที รอไฟล์รูปใหม่ก่อน timeout

# Circle path trajectory
CIRCLE_CENTER_X  =  0.0    # จุดกึ่งกลาง x (mm)
CIRCLE_CENTER_Y  = -200.0  # จุดกึ่งกลาง y (mm)
CIRCLE_RADIUS    =  25.0   # รัศมี (mm) — แนะนำ 20-30 mm
CIRCLE_POINTS    =  36     # จำนวนจุดต่อรอบ (36 = ทุก 10°, 72 = ทุก 5°)
CIRCLE_REVS      =  1      # จำนวนรอบ
CIRCLE_DWELL_S   =  0.0    # เวลาหยุดต่อจุด (วินาที); 0 = ใช้แค่ S-Curve duration
CIRCLE_FREQ_HZ   =  0.6    # ความเร็ว (รอบ/วินาที); 0 = ใช้ SCURVE_DURATION_MS แทน
CIRCLE_COMPENSATION = False  # True = เปิด feed-forward kinematic error compensation
COMPENSATION_MODEL_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'compensation_model.json'
)

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


def calculate_ik(target_xy: np.ndarray) -> tuple[float, float] | None:
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

    Returns:
        (theta_A_deg, theta_B_deg) มุมมอเตอร์ (output-shaft, degrees)
        หรือ None ถ้าไม่มี solution
    """
    global _ik_prev_angles_rad

    P_E = np.array(target_xy, dtype=float)

    pts_C = _circle_intersect_both(P_A, L_AC, P_E, L_CE)
    if pts_C is None:
        return None

    pts_D = _circle_intersect_both(P_B, L_BD, P_E, L_DE)
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

    def move_to(self, x: float, y: float, duration_ms: int | None = None) -> bool:
        """
        เคลื่อนปลายขาไปยัง (x, y) mm

        Args:
            duration_ms: S-Curve duration (ms) สำหรับการเคลื่อนที่นี้โดยเฉพาะ;
                         None = ใช้ค่า SCURVE_DURATION_MS จาก config

        Returns:
            True ถ้าส่งคำสั่งสำเร็จ, False ถ้า IK ล้มเหลว
        """
        if not self.connected:
            print("  ❌ ยังไม่ได้เชื่อมต่อ")
            return False

        result = calculate_ik([x, y])
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
    print("  ─── Approximate Workspace ──────────────────────")
    print("    x : -60 … +60  mm  (horizontal)")
    print("    y : -260 … -120 mm (vertical, negative = below motors)")
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
  grid         รันการทดสอบไล่ตำแหน่งตาม calibration_grid.csv (ทุกจุดตามลำดับ)
  grid N       เคลื่อนไปยังตำแหน่ง point_id = N จุดเดียว  เช่น  grid 10
  capture N    เหมือน grid N แต่รอรับรูปใน capture_image แล้วเปลี่ยนชื่อพร้อมข้อมูลตำแหน่ง
  circle       เคลื่อนที่ตาม path วงกลม (ใช้ค่า default จาก config)
  circle R     เช่น  circle 25         → วงกลม R=25 mm ที่ center default
  circle R cx cy     เช่น  circle 25 0 -200  → กำหนด center ด้วย
  circle R cx cy N   เช่น  circle 25 0 -200 3 → N รอบ
  (เพิ่ม comp หรือ nocomp ท้ายคำสั่ง เพื่อเปิด/ปิด kinematic error compensation)
  เช่น  circle comp  |  circle 25 0 -200 1 comp  |  circle nocomp
  e / estop    Emergency Stop
  h / help     แสดง help นี้
  q / quit     ออกจากโปรแกรม
  ─────────────────────────────────────────────────────────────────────
  ตัวอย่างตำแหน่ง:
    0,-220    → home (ยืนตรง)
    0,-180    → ยกขาขึ้น 40 mm
    30,-220   → เหยียบไปทางขวา 30 mm
    -30,-200  → เหยียบซ้าย ยกขึ้นเล็กน้อย
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
# CAPTURE MODE
# ============================================================================

_IMAGE_EXTS = {
    '.jpg', '.JPG', '.png', '.bmp', '.tiff', '.tif', '.webp',
    '.heic', '.heif', '.raw', '.dng', '.cr2', '.cr3', '.nef',
    '.arw', '.orf', '.rw2', '.pef', '.srw',
}


def _snapshot_files(folder: str) -> set:
    """คืน set ของชื่อไฟล์ทั้งหมดใน folder (ไม่กรอง extension)"""
    try:
        return {f for f in os.listdir(folder)
                if os.path.isfile(os.path.join(folder, f))}
    except FileNotFoundError:
        return set()


def _is_image(filename: str) -> bool:
    """ตรวจว่าชื่อไฟล์มี extension เป็นรูปภาพ (case-insensitive)"""
    return os.path.splitext(filename)[1].lower() in _IMAGE_EXTS


def _wait_for_new_image(folder: str, known: set, timeout_s: float) -> str | None:
    """
    รอไฟล์รูปใหม่ใน folder → คืน full path หรือ None ถ้า timeout
    - known: snapshot ของไฟล์ทั้งหมด (จาก _snapshot_files) ก่อนเริ่มรอ
    - กรอง image extension ที่ระดับนี้ เพื่อไม่พลาดนามสกุลตัวพิมพ์ใหญ่
    """
    deadline = time.time() + timeout_s
    last_print = 0.0
    while time.time() < deadline:
        current = _snapshot_files(folder)
        new_images = {f for f in current - known if _is_image(f)}
        if new_images:
            return os.path.join(folder, sorted(new_images)[0])
        # แสดง countdown ทุก 5 วินาที
        now = time.time()
        if now - last_print >= 5.0:
            remaining = deadline - now
            print(f"     รอ... {remaining:.0f} s  (ไฟล์ใน folder: {len(current)})", flush=True)
            last_print = now
        time.sleep(0.3)
    return None


def _angle_fmt(v: float) -> str:
    """
    ฟอร์แมตมุม (degrees) สำหรับชื่อไฟล์ — ใช้อักขระที่ Windows รองรับ
    ตัวอย่าง:  +15.0 → 'p15p0'   -63.5 → 'n63p5'   nan → 'nan'
    """
    if np.isnan(v):
        return 'nan'
    sign = 'p' if v >= 0 else 'n'
    return f"{sign}{abs(v):.1f}".replace('.', 'p')


def run_capture_point(
    leg: 'SingleLegController',
    target_id: int,
    csv_path: str = GRID_FILE,
    capture_dir: str = CAPTURE_DIR,
    capture_timeout_s: float = CAPTURE_TIMEOUT_S,
) -> bool:
    """
    เคลื่อนไปยัง point_id = target_id, รับ motor feedback
    แล้วรอไฟล์รูปใหม่ใน capture_dir และเปลี่ยนชื่อไฟล์ดังนี้:

        p{id:03d}_x{x}_y{y}_cA{tA_cmd}_cB{tB_cmd}_aA{tA_act}_aB{tB_act}{ext}

    โดย  x/y/angle ใช้ _angle_fmt(): +15.0→p15p0, -63.5→n63p5
    ตัวอย่าง:
        p010_xp15p0_yn180p0_cAn63p5_cBn85p2_aAn63p1_aBn85p0.jpg

    Returns:
        True ถ้าสำเร็จทั้งหมด
    """
    import csv as _csv

    # ─── โหลดตำแหน่งจาก CSV ──────────────────────────────────────────
    try:
        with open(csv_path, newline='', encoding='utf-8') as _f:
            _row = next(
                (r for r in _csv.DictReader(_f) if int(r['point_id']) == target_id),
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

    # ─── เตรียม capture_dir ──────────────────────────────────────────
    os.makedirs(capture_dir, exist_ok=True)
    known_images = _snapshot_files(capture_dir)

    # ─── เคลื่อนที่ไปยังตำแหน่ง ─────────────────────────────────────
    print(f"\n  Capture ID={target_id}: ({tx:+.1f}, {ty:+.1f}) mm")
    ok = leg.move_to(tx, ty)
    if not ok:
        return False

    # บันทึกมุมที่สั่ง (output-shaft, degrees) จาก IK state ล่าสุด
    if _ik_prev_angles_rad is not None:
        tA_cmd = float(np.rad2deg(_ik_prev_angles_rad[0]))
        tB_cmd = float(np.rad2deg(_ik_prev_angles_rad[1]))
    else:
        tA_cmd = tB_cmd = float('nan')

    # ─── รอให้มอเตอร์เสร็จ → รับ feedback จริง ──────────────────────
    time.sleep(max(GRID_DWELL_S, SCURVE_DURATION_MS / 1000.0 + 0.2))

    fb_a = leg.motor_a.ping()
    fb_b = leg.motor_b.ping()
    leg.motor_a.set_timeout(FAST_TIMEOUT)
    leg.motor_b.set_timeout(FAST_TIMEOUT)

    tA_act = fb_a['position'] if fb_a else float('nan')
    tB_act = fb_b['position'] if fb_b else float('nan')

    print("\n  ─── Motor Status ───────────────────────────────")
    for label, fb in [('A', fb_a), ('B', fb_b)]:
        if fb:
            flags = fb['flags']
            moving  = '🔄' if (flags & 0x01) else '  '
            at_goal = '🎯' if (flags & 0x04) else '  '
            error   = '⚠️ ' if (flags & 0x02) else '  '
            print(f"    Motor {label}: pos={fb['position']:+7.2f}°  "
                  f"current={fb['current']:5d} mA  "
                  f"flags=0x{flags:02X}  {moving}{at_goal}{error}")
        else:
            print(f"    Motor {label}: ❌ ไม่ได้รับ feedback")
    print(f"    Cmd   A: {tA_cmd:+.2f}°  B: {tB_cmd:+.2f}°")
    print(f"    Actual A: {tA_act:+.2f}°  B: {tB_act:+.2f}°")
    print("  ────────────────────────────────────────────────")

    # ─── รอไฟล์รูปใหม่ ───────────────────────────────────────────────
    print(f"\n  ⏳ รอไฟล์รูปใหม่ใน {capture_dir}")
    print(f"     (timeout={capture_timeout_s:.0f} s — ถ่ายรูปหรือ copy ไฟล์เข้าโฟลเดอร์)")
    new_path = _wait_for_new_image(capture_dir, known_images, capture_timeout_s)

    if new_path is None:
        print("  ⚠️  ไม่พบไฟล์รูปใหม่ภายใน timeout")
        return False

    # ─── เปลี่ยนชื่อไฟล์ ─────────────────────────────────────────────
    ext = os.path.splitext(new_path)[1].lower()
    new_name = (
        f"p{target_id:03d}"
        f"_x{_angle_fmt(tx)}_y{_angle_fmt(ty)}"
        f"_cA{_angle_fmt(tA_cmd)}_cB{_angle_fmt(tB_cmd)}"
        f"_aA{_angle_fmt(tA_act)}_aB{_angle_fmt(tB_act)}"
        f"{ext}"
    )
    new_full = os.path.join(capture_dir, new_name)

    # retry loop: รอให้แอปกล้อง/Windows ปล่อย file handle ก่อน rename
    _RENAME_RETRIES = 10
    _RENAME_DELAY   = 0.5   # seconds
    for _attempt in range(_RENAME_RETRIES):
        try:
            os.rename(new_path, new_full)
            print(f"  ✅ {os.path.basename(new_path)}")
            print(f"     → {new_name}")
            return True
        except PermissionError:
            if _attempt < _RENAME_RETRIES - 1:
                print(f"     ไฟล์ถูกล็อก รอ {_RENAME_DELAY:.1f}s... ({_attempt + 1}/{_RENAME_RETRIES})",
                      flush=True)
                time.sleep(_RENAME_DELAY)
            else:
                print(f"  ❌ เปลี่ยนชื่อไฟล์ล้มเหลว: ไฟล์ยังถูกล็อกอยู่หลังจากลอง {_RENAME_RETRIES} ครั้ง")
                return False
        except Exception as _e:
            print(f"  ❌ เปลี่ยนชื่อไฟล์ล้มเหลว: {_e}")
            return False

    return False


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
                (int(row['point_id']), float(row['target_x_mm']), float(row['target_y_mm']))
                for row in reader
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

            ok = leg.move_to(tx, ty)
            if not ok:
                failed.append(pid)

            if auto:
                time.sleep(dwell_s)
            else:
                try:
                    ans = input()
                    if ans.strip().lower() == 'q':
                        print("  ⏹️  ยกเลิกโดย user")
                        break
                except (EOFError, KeyboardInterrupt):
                    print()
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

def load_compensation_model(path: str = COMPENSATION_MODEL_FILE) -> dict | None:
    """โหลด polynomial regression model จาก JSON สำหรับชดเชย kinematic error"""
    import json
    try:
        with open(path, 'r', encoding='utf-8') as f:
            model = json.load(f)
        print(f"  ✅ โหลด compensation model: {os.path.basename(path)}")
        m = model.get('metrics', {})
        if m:
            print(f"     RMSE X: {m.get('rmse_x_before',0):.3f} → {m.get('rmse_x_after',0):.3f} mm  "
                  f"RMSE Y: {m.get('rmse_y_before',0):.3f} → {m.get('rmse_y_after',0):.3f} mm")
        return model
    except FileNotFoundError:
        print(f"  ⚠️  ไม่พบ compensation model: {path}")
        return None
    except Exception as e:
        print(f"  ⚠️  โหลด compensation model ล้มเหลว: {e}")
        return None


def predict_kinematic_error(
    thetaA_deg: float, thetaB_deg: float, model: dict
) -> tuple[float, float]:
    """
    ทำนาย kinematic error จากมุมมอเตอร์ด้วย Second-Order Polynomial Regression

    Features: [tA, tB, tA², tA·tB, tB²]  (ลำดับตาม compensation_model.json)

    Returns:
        (err_x, err_y) — ค่า error ที่คาดการณ์ (mm)
    """
    tA  = thetaA_deg
    tB  = thetaB_deg
    phi = np.array([tA, tB, tA * tA, tA * tB, tB * tB])

    mx    = model['model_x']
    err_x = mx['intercept'] + float(np.dot(mx['coef'], phi))

    my    = model['model_y']
    err_y = my['intercept'] + float(np.dot(my['coef'], phi))

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
) -> None:
    """เคลื่อนที่ตาม path วงกลม

    X(θ) = cx + R·cos(θ)
    Y(θ) = cy + R·sin(θ)
    θ ไล่จาก 0 → 2π × revolutions (ทวนเข็มนาฬิกาในระบบพิกัดมาตรฐาน)

    freq_hz > 0  : ควบคุมความเร็ว (รอบ/วินาที) — S-Curve duration คำนวณอัตโนมัติ
    freq_hz = 0  : ใช้ SCURVE_DURATION_MS + dwell_s แทน
    compensate   : True = เปิด feed-forward kinematic error compensation
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
    print(f"    Compensate : {'✅ เปิด' if compensate else '❌ ปิด'}")
    print(f"  ────────────────────────────────────────────────")
    print(f"  กด Ctrl+C เพื่อหยุด\n")

    # โหลด compensation model ถ้าเปิดใช้งาน
    comp_model: dict | None = None
    if compensate:
        comp_model = load_compensation_model()
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

        # ─── Capture mode (grid N + image rename) ───────────────────
        elif cmd.startswith('capture '):
            parts = cmd.split()
            if len(parts) > 1 and parts[1].isdigit():
                run_capture_point(leg, int(parts[1]))
            else:
                print("  ⚠️  ใช้งาน: capture N  (N = point_id)")

        # ─── Circle path ──────────────────────────────────────────────
        elif cmd == 'circle' or cmd.startswith('circle '):
            parts = cmd.split()
            # แยก flag comp / nocomp ก่อน parse ตัวเลข
            if 'comp' in parts:
                _compensate = True
                parts = [p for p in parts if p != 'comp']
            elif 'nocomp' in parts:
                _compensate = False
                parts = [p for p in parts if p != 'nocomp']
            else:
                _compensate = CIRCLE_COMPENSATION
            try:
                if len(parts) == 1:
                    run_circle_path(leg, compensate=_compensate)
                elif len(parts) == 2:
                    run_circle_path(leg, radius=float(parts[1]),
                                    compensate=_compensate)
                elif len(parts) == 4:
                    run_circle_path(leg, radius=float(parts[1]),
                                    cx=float(parts[2]), cy=float(parts[3]),
                                    compensate=_compensate)
                elif len(parts) == 5:
                    run_circle_path(leg, radius=float(parts[1]),
                                    cx=float(parts[2]), cy=float(parts[3]),
                                    revolutions=int(parts[4]),
                                    compensate=_compensate)
                else:
                    print("  ⚠️  รูปแบบ: circle [R [cx cy [N]]] [comp|nocomp]")
            except ValueError:
                print("  ⚠️  ค่าพารามิเตอร์ไม่ถูกต้อง  เช่น  circle 25 0 -200 comp")

        # ─── Grid sweep / single point ────────────────────────────────
        elif cmd == 'grid' or cmd.startswith('grid '):
            parts = cmd.split()
            if len(parts) > 1 and parts[1].isdigit():
                # grid N → เคลื่อนไปจุดเดียว
                target_id = int(parts[1])
                import csv as _csv
                try:
                    with open(GRID_FILE, newline='', encoding='utf-8') as _f:
                        _row = next(
                            (r for r in _csv.DictReader(_f) if int(r['point_id']) == target_id),
                            None
                        )
                    if _row is None:
                        print(f"  ⚠️  ไม่พบ point_id={target_id} ใน {GRID_FILE}")
                    else:
                        tx, ty = float(_row['target_x_mm']), float(_row['target_y_mm'])
                        print(f"  Grid ID={target_id}: ({tx:+.1f}, {ty:+.1f}) mm")
                        leg.move_to(tx, ty)
                        # รอให้มอเตอร์เคลื่อนที่เสร็จแล้วอ่าน feedback
                        time.sleep(max(GRID_DWELL_S, SCURVE_DURATION_MS / 1000.0 + 0.2))
                        leg.print_status()
                except FileNotFoundError:
                    print(f"  ❌ ไม่พบไฟล์: {GRID_FILE}")
                except Exception as _e:
                    print(f"  ❌ โหลด CSV ล้มเหลว: {_e}")
            else:
                # grid → sweep ทุกจุด
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
