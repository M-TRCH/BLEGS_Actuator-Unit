"""
Gait Control - Binary Protocol High-Speed Motor Control
Author: M-TRCH
Date: December 4, 2025

This script controls real BLDC motors using Binary Protocol for high-speed communication.
Upgraded from: Gait_Control_Real_Motors.py
Protocol: Binary Protocol v1.1 (see BINARY_PROTOCOL_GUIDE.md)
"""

import numpy as np
import serial
import time
import threading
import struct
from enum import IntEnum
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Protocol Constants ---
HEADER_1 = 0xFE
HEADER_2 = 0xEE

class PacketType(IntEnum):
    """Packet type enumeration"""
    PKT_CMD_SET_GOAL = 0x01
    PKT_CMD_PING = 0x03
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

# --- 1. Motor Communication Parameters ---
MOTOR1_PORT = 'COM4'  # Left motor (Motor A)
MOTOR2_PORT = 'COM9'   # Right motor (Motor B)
BAUD_RATE = 921600
SERIAL_TIMEOUT = 0.002  # 2ms timeout for high-speed operation

# Motor initial position
MOTOR_INIT_ANGLE = -90.0  # degrees

# Gear ratio (from motor shaft to output shaft)
GEAR_RATIO = 8.0

# --- 2. Robot Kinematic Parameters (mm) ---
P_A = np.array([-42.5, 0.0])  # Motor A position (left)
P_B = np.array([42.5, 0.0])   # Motor B position (right)

L_AC = 105.0  # Link 1 length
L_BD = 105.0  # Link 2 length
L_CE = 145.0  # Link 3 length
L_DE = 145.0  # Link 4 length
L_EF = 40.0   # Offset length

# Offset ratios
OFFSET_RATIO_E = 37.0 / 29.0
OFFSET_RATIO_D = 8.0 / 29.0

# --- 3. Motion Control Parameters ---
CONTROL_MODE = ControlMode.MODE_DIRECT_POSITION  # Direct mode for fastest response
UPDATE_RATE = 100  # Hz (10ms per update)
TRAJECTORY_STEPS = 60  # Number of steps in one gait cycle (60 steps × 10ms = 600ms/cycle - FASTER!)

# --- 3.5 Visualization Parameters ---
ENABLE_VISUALIZATION = True
PLOT_UPDATE_RATE = 10  # Hz

# Global variables for visualization
current_angles = [0.0, 0.0]
actual_angles = [0.0, 0.0]
current_target = [0.0, -200.0]
viz_lock = threading.Lock()
plot_running = True

# Global variables for error tracking
error_history_A = []
error_history_B = []
max_error_A = 0.0
max_error_B = 0.0
error_lock = threading.Lock()

# Global variables for control
gait_running = False
gait_paused = False
control_lock = threading.Lock()

def reset_error_stats():
    """Reset all error statistics"""
    global error_history_A, error_history_B, max_error_A, max_error_B
    with error_lock:
        error_history_A.clear()
        error_history_B.clear()
        max_error_A = 0.0
        max_error_B = 0.0
    print("\n🔄 Error statistics reset!")

def toggle_gait_control():
    """Toggle gait control on/off"""
    global gait_running, gait_paused
    with control_lock:
        if not gait_running:
            gait_running = True
            gait_paused = False
            print("\n▶️  Gait control STARTED!")
        else:
            gait_running = False
            gait_paused = True
            print("\n⏸️  Gait control PAUSED!")

# --- 4. Protocol Functions ---
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

def build_packet(pkt_type: int, payload: bytes) -> bytes:
    """Build a complete binary packet with CRC"""
    header = bytes([HEADER_1, HEADER_2])
    type_byte = bytes([pkt_type])
    payload_len = bytes([len(payload)])
    
    # Calculate CRC over: pkt_type + payload_len + payload
    crc_data = type_byte + payload_len + payload
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    
    return header + type_byte + payload_len + payload + crc_bytes

# --- 5. Binary Motor Controller Class ---
class BinaryMotorController:
    """High-speed motor controller using binary protocol"""
    
    def __init__(self, port, name, initial_angle):
        self.port = port
        self.name = name
        self.initial_angle = initial_angle
        self.serial = None
        self.is_connected = False
        self.current_setpoint = 0.0
        self.current_position = 0.0
        self.current_flags = 0
        self.lock = threading.Lock()
        self.stats_tx_count = 0
        self.stats_rx_count = 0
        self.stats_errors = 0
        
    def connect(self):
        """Connect to motor via serial port"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=BAUD_RATE,
                timeout=SERIAL_TIMEOUT,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(0.5)
            
            # Flush buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Send start command (ASCII)
            self.serial.write(b'S')
            time.sleep(0.1)
            
            self.is_connected = True
            print(f"✅ {self.name} connected on {self.port}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect {self.name} on {self.port}: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from motor"""
        if self.serial and self.is_connected:
            self.serial.close()
            self.is_connected = False
            print(f"🔌 {self.name} disconnected")
    
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
            # Convert robot angle to motor shaft angle
            motor_angle = angle_deg * GEAR_RATIO
            
            with self.lock:
                # Build payload: mode + target_pos (int32, degrees*100)
                mode = bytes([ControlMode.MODE_DIRECT_POSITION])
                target_pos = struct.pack('<i', int(motor_angle * 100))
                payload = mode + target_pos
                
                # Build and send packet
                packet = build_packet(PacketType.PKT_CMD_SET_GOAL, payload)
                self.serial.write(packet)
                self.stats_tx_count += 1
            
            return True
            
        except Exception as e:
            print(f"❌ {self.name} set_position error: {e}")
            self.is_connected = False
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
                # Build payload: mode + target_pos + duration
                mode = bytes([ControlMode.MODE_SCURVE_PROFILE])
                target_pos = struct.pack('<i', int(motor_angle * 100))
                duration = struct.pack('<H', duration_ms)
                payload = mode + target_pos + duration
                
                # Build and send packet
                packet = build_packet(PacketType.PKT_CMD_SET_GOAL, payload)
                self.serial.write(packet)
                self.stats_tx_count += 1
            
            return True
            
        except Exception as e:
            print(f"❌ {self.name} set_position_scurve error: {e}")
            self.is_connected = False
            return False
    
    def read_feedback(self) -> dict:
        """
        Read motor feedback from binary protocol
        
        Returns:
            Dictionary with feedback data or None
        """
        if not self.is_connected:
            return None
        
        try:
            # Read header
            if self.serial.in_waiting < 2:
                return None
            
            header = self.serial.read(2)
            if len(header) != 2 or header[0] != HEADER_1 or header[1] != HEADER_2:
                # Invalid header, flush buffer
                self.serial.reset_input_buffer()
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
                # Don't print error - too noisy, just count it
                return None
            
            self.stats_rx_count += 1
            
            # Parse status feedback
            if pkt_type == PacketType.PKT_FB_STATUS and len(payload) >= 7:
                actual_pos_raw = struct.unpack('<i', payload[0:4])[0]
                actual_current = struct.unpack('<h', payload[4:6])[0]
                status_flags = payload[6]
                
                with self.lock:
                    # Convert motor shaft angle back to robot angle
                    self.current_position = (actual_pos_raw / 100.0) / GEAR_RATIO
                    self.current_flags = status_flags
                
                return {
                    'position': self.current_position,
                    'current': actual_current,
                    'flags': status_flags,
                    'is_moving': bool(status_flags & StatusFlags.STATUS_MOVING),
                    'at_goal': bool(status_flags & StatusFlags.STATUS_AT_GOAL),
                    'error': bool(status_flags & StatusFlags.STATUS_ERROR)
                }
            
            elif pkt_type == PacketType.PKT_FB_ERROR:
                error_code = payload[0] if len(payload) > 0 else 0
                # Don't print error - too noisy, just count it
                self.stats_errors += 1
            
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
            'tx': self.stats_tx_count,
            'rx': self.stats_rx_count,
            'errors': self.stats_errors,
            'success_rate': (self.stats_rx_count / max(1, self.stats_tx_count)) * 100
        }

# --- 6. Kinematics Functions ---
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

def calculate_ik_analytical(P_F_target, elbow_C_down=True, elbow_D_down=True):
    """Calculate Inverse Kinematics using analytical method"""
    (x_f, y_f) = P_F_target
    
    R_FD = 145.0 * 37.0 / 29.0
    R_DB = L_BD
    
    P_D = solve_circle_intersection(P_F_target, R_FD, P_B, R_DB, elbow_D_down)
    
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

def generate_walking_trajectory(num_steps=60, lift_height=30, step_forward=60):
    """Generate elliptical walking trajectory"""
    trajectory = []
    home_y = -200
    
    a = step_forward
    b = lift_height
    
    for i in range(num_steps):
        t = 2 * np.pi * i / num_steps
        px = a * np.cos(t)
        py = home_y + b * np.sin(t)
        trajectory.append((px, py))
    
    return trajectory

# --- 7. Visualization Functions ---
def calculate_fk_positions(theta_A, theta_B):
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
            P_E = P_C + a * v_d - h * v_perp
            P_F = (OFFSET_RATIO_E * P_E) - (OFFSET_RATIO_D * P_D)
            
            return P_C, P_D, P_E, P_F
    
    return None, None, None, None

def visualization_thread():
    """Thread function for real-time visualization"""
    global plot_running, current_angles, actual_angles, current_target
    
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-250, 250)
    ax.set_ylim(-350, 100)
    ax.set_xlabel('X Position (mm)', fontsize=11, weight='bold')
    ax.set_ylabel('Y Position (mm)', fontsize=11, weight='bold')
    ax.set_title('Binary Protocol - Real-Time Visualization', fontsize=13, weight='bold')
    
    ax.plot(P_A[0], P_A[1], 'ro', markersize=12, label='Motor A', zorder=5)
    ax.plot(P_B[0], P_B[1], 'bo', markersize=12, label='Motor B', zorder=5)
    
    # Setpoint links (solid, bright)
    link1, = ax.plot([], [], 'r-', linewidth=5, label='Target', zorder=4)
    link2, = ax.plot([], [], 'b-', linewidth=5, zorder=4)
    link3, = ax.plot([], [], 'orange', linestyle='--', linewidth=4, zorder=3)
    link4, = ax.plot([], [], 'cyan', linestyle='--', linewidth=4, zorder=3)
    link5, = ax.plot([], [], 'g-', linewidth=3.5, zorder=4)
    
    # Actual links (dashed, faded)
    link1_actual, = ax.plot([], [], 'r--', linewidth=3, alpha=0.5, label='Actual', zorder=3)
    link2_actual, = ax.plot([], [], 'b--', linewidth=3, alpha=0.5, zorder=3)
    link3_actual, = ax.plot([], [], 'orange', linestyle=':', linewidth=2, alpha=0.4, zorder=2)
    link4_actual, = ax.plot([], [], 'cyan', linestyle=':', linewidth=2, alpha=0.4, zorder=2)
    link5_actual, = ax.plot([], [], 'g--', linewidth=2, alpha=0.5, zorder=3)
    
    # Joints
    joint_c, = ax.plot([], [], 'ro', markersize=10, markeredgecolor='black', markeredgewidth=2, zorder=5)
    joint_d, = ax.plot([], [], 'bo', markersize=10, markeredgecolor='black', markeredgewidth=2, zorder=5)
    joint_e, = ax.plot([], [], 's', color='purple', markersize=8, markeredgecolor='black', markeredgewidth=2, zorder=5)
    foot, = ax.plot([], [], '*', color='green', markersize=20, markeredgecolor='black', markeredgewidth=1.5, zorder=6)
    target, = ax.plot([], [], 'x', color='red', markersize=15, markeredgewidth=3, label='Target Pos', zorder=6)
    
    joint_c_actual, = ax.plot([], [], 'ro', markersize=6, alpha=0.5, zorder=4)
    joint_d_actual, = ax.plot([], [], 'bo', markersize=6, alpha=0.5, zorder=4)
    joint_e_actual, = ax.plot([], [], 's', color='purple', markersize=5, alpha=0.5, zorder=4)
    foot_actual, = ax.plot([], [], '*', color='green', markersize=12, alpha=0.5, zorder=5)
    
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                        fontsize=9, verticalalignment='top', family='monospace',
                        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    # Add instruction text for keyboard controls
    control_text = ax.text(0.02, 0.02, 'Controls: [SPACE] Start/Stop  |  [R] Reset Error Stats', 
                        transform=ax.transAxes,
                        fontsize=8, verticalalignment='bottom', family='monospace',
                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    ax.legend(loc='lower right', fontsize=8, framealpha=0.9)
    
    def update_plot(frame):
        global plot_running, current_angles, actual_angles, current_target
        
        if not plot_running:
            return (link1, link2, link3, link4, link5,
                    link1_actual, link2_actual, link3_actual, link4_actual, link5_actual,
                    joint_c, joint_d, joint_e, foot, target,
                    joint_c_actual, joint_d_actual, joint_e_actual, foot_actual, info_text)
        
        with viz_lock:
            theta_A, theta_B = current_angles
            theta_A_actual, theta_B_actual = actual_angles
            target_x, target_y = current_target
        
        P_C, P_D, P_E, P_F = calculate_fk_positions(theta_A, theta_B)
        P_C_actual, P_D_actual, P_E_actual, P_F_actual = calculate_fk_positions(theta_A_actual, theta_B_actual)
        
        if P_C is not None and P_D is not None and P_E is not None and P_F is not None:
            link1.set_data([P_A[0], P_C[0]], [P_A[1], P_C[1]])
            link2.set_data([P_B[0], P_D[0]], [P_B[1], P_D[1]])
            link3.set_data([P_C[0], P_E[0]], [P_C[1], P_E[1]])
            link4.set_data([P_D[0], P_E[0]], [P_D[1], P_E[1]])
            link5.set_data([P_E[0], P_F[0]], [P_E[1], P_F[1]])
            
            joint_c.set_data([P_C[0]], [P_C[1]])
            joint_d.set_data([P_D[0]], [P_D[1]])
            joint_e.set_data([P_E[0]], [P_E[1]])
            foot.set_data([P_F[0]], [P_F[1]])
            target.set_data([target_x], [target_y])
        
        if P_C_actual is not None and P_D_actual is not None and P_E_actual is not None and P_F_actual is not None:
            link1_actual.set_data([P_A[0], P_C_actual[0]], [P_A[1], P_C_actual[1]])
            link2_actual.set_data([P_B[0], P_D_actual[0]], [P_B[1], P_D_actual[1]])
            link3_actual.set_data([P_C_actual[0], P_E_actual[0]], [P_C_actual[1], P_E_actual[1]])
            link4_actual.set_data([P_D_actual[0], P_E_actual[0]], [P_D_actual[1], P_E_actual[1]])
            link5_actual.set_data([P_E_actual[0], P_F_actual[0]], [P_E_actual[1], P_F_actual[1]])
            
            joint_c_actual.set_data([P_C_actual[0]], [P_C_actual[1]])
            joint_d_actual.set_data([P_D_actual[0]], [P_D_actual[1]])
            joint_e_actual.set_data([P_E_actual[0]], [P_E_actual[1]])
            foot_actual.set_data([P_F_actual[0]], [P_F_actual[1]])
            
            # Calculate error statistics
            with error_lock:
                avg_err_A = np.mean(error_history_A) if len(error_history_A) > 0 else 0.0
                avg_err_B = np.mean(error_history_B) if len(error_history_B) > 0 else 0.0
                max_err_A = max_error_A
                max_err_B = max_error_B
            
            info_text.set_text(
                f'═══ Binary Protocol Mode ═══\n'
                f'θA Target:  {np.rad2deg(theta_A):+7.1f}°\n'
                f'θA Actual:  {np.rad2deg(theta_A_actual):+7.1f}°\n'
                f'θB Target:  {np.rad2deg(theta_B):+7.1f}°\n'
                f'θB Actual:  {np.rad2deg(theta_B_actual):+7.1f}°\n\n'
                f'═══ Foot Position ═══\n'
                f'Target: ({P_F[0]:+6.1f}, {P_F[1]:+6.1f})\n'
                f'Actual: ({P_F_actual[0]:+6.1f}, {P_F_actual[1]:+6.1f})\n\n'
                f'═══ Current Error ═══\n'
                f'ΔθA: {np.rad2deg(theta_A - theta_A_actual):+6.1f}°\n'
                f'ΔθB: {np.rad2deg(theta_B - theta_B_actual):+6.1f}°\n'
                f'ΔX:  {P_F[0] - P_F_actual[0]:+6.1f} mm\n'
                f'ΔY:  {P_F[1] - P_F_actual[1]:+6.1f} mm\n\n'
                f'═══ Error Statistics ═══\n'
                f'Max θA:  {max_err_A:6.2f}°\n'
                f'Avg θA:  {avg_err_A:6.2f}°\n'
                f'Max θB:  {max_err_B:6.2f}°\n'
                f'Avg θB:  {avg_err_B:6.2f}°'
            )
        
        return (link1, link2, link3, link4, link5,
                link1_actual, link2_actual, link3_actual, link4_actual, link5_actual,
                joint_c, joint_d, joint_e, foot, target,
                joint_c_actual, joint_d_actual, joint_e_actual, foot_actual, info_text)
    
    anim = FuncAnimation(fig, update_plot, interval=int(1000/PLOT_UPDATE_RATE), blit=True, cache_frame_data=False)
    
    # Add keyboard event handler
    def on_key_press(event):
        if event.key == 'r' or event.key == 'R':
            reset_error_stats()
        elif event.key == ' ':  # Space bar
            toggle_gait_control()
    
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    plt.tight_layout()
    plt.show()
    
    plot_running = False

def start_visualization():
    """Start visualization in separate thread"""
    viz_thread = threading.Thread(target=visualization_thread, daemon=True)
    viz_thread.start()
    time.sleep(1.0)
    return viz_thread

# --- 8. Main Control Loop ---
def main():
    global plot_running, current_angles, actual_angles, current_target
    global error_history_A, error_history_B, max_error_A, max_error_B
    global gait_running, gait_paused
    
    print("="*70)
    print("  BLEGS Gait Control - Binary Protocol High-Speed System")
    print("="*70)
    print(f"  Motor 1 (Left):  {MOTOR1_PORT}")
    print(f"  Motor 2 (Right): {MOTOR2_PORT}")
    print(f"  Baud Rate: {BAUD_RATE}")
    print(f"  Protocol: Binary v1.1")
    print(f"  Control Mode: {'S-Curve' if CONTROL_MODE == ControlMode.MODE_SCURVE_PROFILE else 'Direct'}")
    print(f"  Update Rate: {UPDATE_RATE} Hz")
    print(f"  Gear Ratio: {GEAR_RATIO}:1")
    print(f"  Visualization: {'Enabled' if ENABLE_VISUALIZATION else 'Disabled'}")
    print("="*70)
    
    # Start visualization thread
    viz_thread = None
    if ENABLE_VISUALIZATION:
        print("\n📊 Starting real-time visualization...")
        viz_thread = start_visualization()
        print("  Visualization started!")
    
    # Create motor controllers
    motor_A = BinaryMotorController(MOTOR1_PORT, "Motor A (Left)", MOTOR_INIT_ANGLE)
    motor_B = BinaryMotorController(MOTOR2_PORT, "Motor B (Right)", MOTOR_INIT_ANGLE)
    
    # Connect to motors
    print("\n🔌 Connecting to motors...")
    if not motor_A.connect():
        print("❌ Failed to connect Motor A. Exiting...")
        return
    
    if not motor_B.connect():
        print("❌ Failed to connect Motor B. Exiting...")
        motor_A.disconnect()
        return
    
    print("  Motors initialized and started!")
    time.sleep(2.0)  # Wait for motors to be ready
    
    try:
        # Generate trajectory
        print(f"\n🚶 Generating walking trajectory...")
        trajectory = generate_walking_trajectory(
            num_steps=TRAJECTORY_STEPS,
            lift_height=30,
            step_forward=60
        )
        print(f"  Generated {len(trajectory)} waypoints")
        print(f"  Update rate: {UPDATE_RATE} Hz")
        
        # Move to home position (using S-Curve for smooth start)
        print(f"\n🏠 Moving to home position (0, -200)...")
        home_pos = np.array([0.0, -200.0])
        home_angles = calculate_ik_analytical(home_pos, elbow_C_down=True, elbow_D_down=True)
        
        if not np.isnan(home_angles).any():
            motor_A.set_position_scurve(np.rad2deg(home_angles[0]), 2000)
            motor_B.set_position_scurve(np.rad2deg(home_angles[1]), 2000)
            print(f"  Target angles: θA={np.rad2deg(home_angles[0]):.2f}°, θB={np.rad2deg(home_angles[1]):.2f}°")
            time.sleep(2.5)
        else:
            print("❌ Failed to calculate home position IK")
            return
        
        # Start gait cycle
        print(f"\n⏸️  Gait control ready (PAUSED)")
        print("  Press [SPACE] in visualization window to start")
        print("  Press Ctrl+C to stop")
        print("="*70)
        
        prev_solution = home_angles
        cycle_count = 0
        gait_paused = True  # Start in paused state
        
        while True:
            # Check if gait control is paused
            with control_lock:
                is_paused = gait_paused
            
            if is_paused:
                time.sleep(0.05)  # Small sleep when paused
                continue
            
            cycle_count += 1
            print(f"\n🔄 Gait Cycle #{cycle_count}")
            
            for frame, (px, py) in enumerate(trajectory):
                # Check pause state before each step
                with control_lock:
                    if gait_paused:
                        break
                # Find best IK solution
                configs = [
                    (True, True),
                    (True, False),
                    (False, True),
                    (False, False)
                ]
                
                best_solution = None
                best_distance = float('inf')
                
                for elbow_C, elbow_D in configs:
                    solution = calculate_ik_analytical(
                        np.array([px, py]), 
                        elbow_C_down=elbow_C, 
                        elbow_D_down=elbow_D
                    )
                    
                    if not np.isnan(solution).any():
                        angle_diff = np.abs(solution - prev_solution)
                        angle_diff = np.minimum(angle_diff, 2*np.pi - angle_diff)
                        distance = np.sum(angle_diff)
                        
                        if distance < best_distance:
                            best_distance = distance
                            best_solution = solution
                
                if best_solution is not None:
                    theta_A, theta_B = best_solution
                    prev_solution = best_solution
                    
                    # Update visualization
                    with viz_lock:
                        current_angles = [theta_A, theta_B]
                        current_target = [px, py]
                    
                    # Send commands (Direct mode for fastest response)
                    loop_start_time = time.perf_counter()
                    
                    motor_A.set_position_direct(np.rad2deg(theta_A))
                    motor_B.set_position_direct(np.rad2deg(theta_B))
                    
                    # Read feedback
                    feedback_A = motor_A.read_feedback()
                    feedback_B = motor_B.read_feedback()
                    
                    cmd_time = (time.perf_counter() - loop_start_time) * 1000  # ms
                    
                    # Update actual angles (update each motor independently)
                    if feedback_A:
                        with viz_lock:
                            actual_angles[0] = np.deg2rad(feedback_A['position'])
                        
                        # Calculate and track errors for Motor A (output shaft angles)
                        error_A = abs(np.rad2deg(theta_A - np.deg2rad(feedback_A['position'])))
                        with error_lock:
                            error_history_A.append(error_A)
                            max_error_A = max(max_error_A, error_A)
                    
                    if feedback_B:
                        with viz_lock:
                            actual_angles[1] = np.deg2rad(feedback_B['position'])
                        
                        # Calculate and track errors for Motor B (output shaft angles)
                        error_B = abs(np.rad2deg(theta_B - np.deg2rad(feedback_B['position'])))
                        with error_lock:
                            error_history_B.append(error_B)
                            max_error_B = max(max_error_B, error_B)
                    
                    # Check connection
                    if not motor_A.is_connected or not motor_B.is_connected:
                        print("\n\n🚨 Motor disconnected!")
                        raise RuntimeError("Motor disconnection")
                    
                    # Display status (compact)
                    if (frame + 1) % 10 == 0:  # Print every 10 steps
                        print(f"  [{frame+1:3d}/{len(trajectory)}] "
                              f"Cmd: {cmd_time:.2f}ms "
                              f"θA:{np.rad2deg(theta_A):+6.1f}° "
                              f"θB:{np.rad2deg(theta_B):+6.1f}°", end='')
                        if feedback_A and feedback_B:
                            print(f" | Actual: θA:{feedback_A['position']:+6.1f}° "
                                  f"θB:{feedback_B['position']:+6.1f}°")
                        else:
                            print()
                    
                    # Precise timing control - compensate for execution time
                    elapsed = time.perf_counter() - loop_start_time
                    sleep_time = max(0, (1.0 / UPDATE_RATE) - elapsed)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                else:
                    print(f"  ⚠️  IK failed for ({px:.1f}, {py:.1f})")
    
    except KeyboardInterrupt:
        print("\n\n⏹️  Gait control stopped by user")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Stop visualization
        plot_running = False
        
        # Return to home (S-Curve)
        print("\n🏠 Returning to home position...")
        try:
            motor_A.set_position_scurve(np.rad2deg(home_angles[0]), 2000)
            motor_B.set_position_scurve(np.rad2deg(home_angles[1]), 2000)
            time.sleep(2.5)
        except:
            pass
        
        # Print statistics
        print("\n📊 Communication Statistics:")
        stats_A = motor_A.get_stats()
        stats_B = motor_B.get_stats()
        print(f"  Motor A: TX={stats_A['tx']}, RX={stats_A['rx']}, "
              f"Errors={stats_A['errors']}, Success={stats_A['success_rate']:.1f}%")
        print(f"  Motor B: TX={stats_B['tx']}, RX={stats_B['rx']}, "
              f"Errors={stats_B['errors']}, Success={stats_B['success_rate']:.1f}%")
        
        # Print error statistics
        print("\n📈 Output Shaft Error Statistics:")
        with error_lock:
            if len(error_history_A) > 0:
                avg_err_A = np.mean(error_history_A)
                avg_err_B = np.mean(error_history_B)
                print(f"  Motor A: Max={max_error_A:.2f}°, Avg={avg_err_A:.2f}°, Samples={len(error_history_A)}")
                print(f"  Motor B: Max={max_error_B:.2f}°, Avg={avg_err_B:.2f}°, Samples={len(error_history_B)}")
            else:
                print("  No error data collected")
        
        # Disconnect motors
        print("\n🔌 Disconnecting motors...")
        motor_A.disconnect()
        motor_B.disconnect()
        
        print("\n✅ Gait control terminated successfully")
        print("="*70)
        
        if viz_thread and ENABLE_VISUALIZATION:
            print("\n📊 Closing visualization...")
            time.sleep(1.0)

if __name__ == "__main__":
    main()
