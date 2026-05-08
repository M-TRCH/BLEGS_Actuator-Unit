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

def _circle_intersect(c1, r1, c2, r2, choose_lower=True) -> np.ndarray:
    """หาจุดตัดของวงกลม 2 วง → [x, y] หรือ [nan, nan]"""
    d = np.linalg.norm(c2 - c1)
    if d > (r1 + r2) or d < abs(r1 - r2) or d == 0:
        return np.array([np.nan, np.nan])
    a  = (r1**2 - r2**2 + d**2) / (2 * d)
    h2 = r1**2 - a**2
    if h2 < 0:
        return np.array([np.nan, np.nan])
    h   = np.sqrt(h2)
    vd  = (c2 - c1) / d
    vp  = np.array([-vd[1], vd[0]])
    p1  = c1 + a * vd + h * vp
    p2  = c1 + a * vd - h * vp
    if choose_lower:
        return p2 if p2[1] < p1[1] else p1
    else:
        return p1 if p1[1] > p2[1] else p2


def calculate_ik(target_xy: np.ndarray) -> tuple[float, float] | None:
    """
    คำนวณ Inverse Kinematics สำหรับ Five-Bar linkage (ไม่มี EF link)

    Args:
        target_xy: [x, y] ตำแหน่งปลายขา (mm) ในระบบพิกัดขา

    Returns:
        (theta_A_deg, theta_B_deg) มุมมอเตอร์ (output-shaft, degrees)
        หรือ None ถ้าไม่มี solution
    """
    P_E = np.array(target_xy, dtype=float)

    P_C = _circle_intersect(P_A, L_AC, P_E, L_CE, choose_lower=True)
    if np.isnan(P_C).any():
        return None

    P_D = _circle_intersect(P_B, L_BD, P_E, L_DE, choose_lower=True)
    if np.isnan(P_D).any():
        return None

    theta_A_rad = np.arctan2((P_C - P_A)[1], (P_C - P_A)[0])
    theta_B_rad = np.arctan2((P_D - P_B)[1], (P_D - P_B)[0])

    return np.rad2deg(theta_A_rad), np.rad2deg(theta_B_rad)


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
        print("\n  Moving to init angle (-90°)...")
        if self.use_scurve:
            self.motor_a.set_position_scurve(MOTOR_INIT_ANGLE, duration_ms=1000)
            self.motor_b.set_position_scurve(MOTOR_INIT_ANGLE, duration_ms=1000)
        else:
            self.motor_a.set_position_direct(MOTOR_INIT_ANGLE)
            self.motor_b.set_position_direct(MOTOR_INIT_ANGLE)
        time.sleep(1.2)

    # ------------------------------------------------------------------
    # Core move
    # ------------------------------------------------------------------

    def move_to(self, x: float, y: float) -> bool:
        """
        เคลื่อนปลายขาไปยัง (x, y) mm

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

        print(f"  → target ({x:+.1f}, {y:+.1f}) mm  "
              f"│  θA={theta_A_deg:+.2f}°  θB={theta_B_deg:+.2f}°", end='  ')

        if self.use_scurve:
            ok_a = self.motor_a.set_position_scurve(theta_A_deg)
            ok_b = self.motor_b.set_position_scurve(theta_B_deg)
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
