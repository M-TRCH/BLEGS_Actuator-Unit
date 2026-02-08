"""
Gait Control - Real Motor Control via Serial Port
Author: M-TRCH
Date: November 22, 2025

This script controls real BLDC motors for the 5-bar parallel linkage robot.
Based on: IK-Five-Bar-Leg-Animation.py
Motors: MOTOR1=COM44, MOTOR2=COM9
Initial angle: -90 degrees for both motors
"""

import numpy as np
import serial
import time
import threading
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
import matplotlib.patches as mpatches

# --- 1. Motor Communication Parameters ---
MOTOR1_PORT = 'COM9'  # Left motor (Motor A)
MOTOR2_PORT = 'COM44'   # Right motor (Motor B)
BAUD_RATE = 921600
SERIAL_TIMEOUT = 0.1

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
CONTROL_MODE = 0  # 1 = S-Curve mode (smooth), 0 = Direct mode (fast)
UPDATE_RATE = 45  # Hz (22ms per update) - 2.25x faster gait (1.5x from previous 30Hz)
TRAJECTORY_STEPS = 60  # Number of steps in one gait cycle

# --- 3.5 Visualization Parameters ---
ENABLE_VISUALIZATION = True  # Enable real-time visualization
PLOT_UPDATE_RATE = 10  # Hz (100ms per plot update)

# Global variables for visualization
current_angles = [0.0, 0.0]  # [theta_A, theta_B] in radians (setpoint)
actual_angles = [0.0, 0.0]   # [theta_A, theta_B] in radians (actual from feedback)
current_target = [0.0, -200.0]  # [x, y] target position
viz_lock = threading.Lock()
plot_running = True

# --- 4. Serial Communication Class ---
class MotorController:
    """Class for controlling a single motor via serial port"""
    
    def __init__(self, port, name, initial_angle):
        self.port = port
        self.name = name
        self.initial_angle = initial_angle
        self.serial = None
        self.is_connected = False
        self.current_setpoint = 0.0
        self.current_position = 0.0
        self.lock = threading.Lock()
        
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
            time.sleep(0.5)  # Wait for connection to stabilize
            
            # Flush buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
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
    
    def set_mode(self, mode):
        """Set control mode (0=Direct, 1=S-Curve)"""
        if not self.is_connected:
            return False
        
        try:
            command = f"M{mode}\n"
            self.serial.write(command.encode())
            time.sleep(0.05)
            
            # Read response
            if self.serial.in_waiting > 0:
                response = self.serial.readline().decode().strip()
                print(f"  {self.name} mode: {response}")
            
            return True
        except Exception as e:
            print(f"❌ {self.name} set_mode error: {e}")
            return False
    
    def switch_mode(self, mode):
        """Switch control mode (0=Direct, 1=S-Curve) without printing response"""
        if not self.is_connected:
            return False
        
        try:
            command = f"M{mode}\n"
            self.serial.write(command.encode())
            time.sleep(0.05)
            
            # Clear response buffer
            if self.serial.in_waiting > 0:
                self.serial.readline()
            
            return True
        except Exception as e:
            return False
    
    def set_position(self, angle_deg):
        """
        Set target position in degrees
        Input: angle_deg in robot coordinate (relative to kinematic model)
        Output: Sends command to motor with gear ratio compensation
        
        Motor command calculation:
        - Desired robot angle must be multiplied by gear ratio (8:1)
        - Example: To move to -90°, send command: -90° × 8 = -720°
        """
        if not self.is_connected:
            return False
        
        try:
            # Convert robot angle to motor shaft angle
            # Motor angle = Robot angle × Gear ratio
            motor_angle = angle_deg * GEAR_RATIO
            
            with self.lock:
                # Flush input buffer before sending new command to prevent buffer overflow
                self.serial.reset_input_buffer()
                
                command = f"#{motor_angle:.2f}\n"
                self.serial.write(command.encode())
                
                # Small delay to allow command to be sent
                time.sleep(0.001)
            
            return True
        except Exception as e:
            print(f"❌ {self.name} set_position error: {e}")
            print(f"⚠️  {self.name} may have disconnected!")
            self.is_connected = False
            return False
    
    def read_feedback(self):
        """Read motor feedback (Returns: setpoint current_position)"""
        if not self.is_connected:
            return None
        
        try:
            # Read multiple lines if available to clear buffer
            max_attempts = 5
            latest_feedback = None
            
            for _ in range(max_attempts):
                if self.serial.in_waiting > 0:
                    try:
                        line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                        
                        # Parse feedback: "Returns:    <setpoint>    <current_position>"
                        if line.startswith("Returns:"):
                            parts = line.split()
                            if len(parts) >= 3:
                                try:
                                    setpoint = float(parts[1])
                                    position = float(parts[2])
                                    
                                    with self.lock:
                                        # Convert motor shaft angle back to robot angle
                                        # Robot angle = Motor angle / Gear ratio
                                        self.current_setpoint = setpoint / GEAR_RATIO
                                        self.current_position = position / GEAR_RATIO
                                    
                                    latest_feedback = (self.current_setpoint, self.current_position)
                                except ValueError:
                                    # Skip corrupted data
                                    continue
                    except UnicodeDecodeError:
                        # Skip corrupted data
                        continue
                else:
                    break
            
            return latest_feedback
        except Exception as e:
            # Only print error if it's not a parsing issue
            if "could not convert" not in str(e):
                print(f"❌ {self.name} read_feedback error: {e}")
                print(f"⚠️  {self.name} may have disconnected!")
                self.is_connected = False
            return None
    
    def get_current_position(self):
        """Get current position (thread-safe)"""
        with self.lock:
            return self.current_position

# --- 5. Kinematics Functions ---
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
    
    # Calculate P_D from circle-circle intersection
    R_FD = 145.0 * 37.0 / 29.0  # ≈ 185.0 mm
    R_DB = L_BD  # = 105.0 mm
    
    # Invert elbow_D_down to match hardware orientation
    P_D = solve_circle_intersection(P_F_target, R_FD, P_B, R_DB, elbow_D_down)
    
    if np.isnan(P_D).any():
        return np.array([np.nan, np.nan])
    
    # Calculate P_E
    P_E = (29.0 * P_F_target + 8.0 * P_D) / 37.0
    
    # Calculate P_C
    P_C = solve_circle_intersection(P_A, L_AC, P_E, L_CE, elbow_C_down)
    
    if np.isnan(P_C).any():
        return np.array([np.nan, np.nan])
    
    # Calculate angles
    V_AC = P_C - P_A
    V_BD = P_D - P_B
    
    theta_A = np.arctan2(V_AC[1], V_AC[0])
    theta_B = np.arctan2(V_BD[1], V_BD[0])
    
    return np.array([theta_A, theta_B])

def generate_walking_trajectory(num_steps=60, lift_height=30, step_forward=60):
    """Generate elliptical walking trajectory"""
    trajectory = []
    home_y = -200  # Home position Y coordinate
    
    a = step_forward  # Semi-major axis (horizontal)
    b = lift_height   # Semi-minor axis (vertical)
    
    for i in range(num_steps):
        t = 2 * np.pi * i / num_steps
        px = a * np.cos(t)
        py = home_y + b * np.sin(t)
        trajectory.append((px, py))
    
    return trajectory

# --- 6. Visualization Functions ---
def calculate_fk_positions(theta_A, theta_B):
    """Calculate forward kinematics positions for visualization"""
    # Calculate joint positions
    P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
    P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])
    
    # Calculate P_E (intersection point)
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
            
            # Calculate foot position
            P_F = (OFFSET_RATIO_E * P_E) - (OFFSET_RATIO_D * P_D)
            
            return P_C, P_D, P_E, P_F
    
    return None, None, None, None

def visualization_thread():
    """Thread function for real-time visualization"""
    global plot_running, current_angles, current_target
    
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-250, 250)
    ax.set_ylim(-350, 100)
    ax.set_xlabel('X Position (mm)', fontsize=11, weight='bold')
    ax.set_ylabel('Y Position (mm)', fontsize=11, weight='bold')
    ax.set_title('Real-Time 5-Bar Linkage Visualization', fontsize=13, weight='bold')
    
    # Plot motor positions
    ax.plot(P_A[0], P_A[1], 'ro', markersize=12, label='Motor A (Left)', zorder=5)
    ax.plot(P_B[0], P_B[1], 'bo', markersize=12, label='Motor B (Right)', zorder=5)
    
    # Initialize link lines for SETPOINT (solid, bright colors)
    link1, = ax.plot([], [], 'r-', linewidth=5, label='L₁ (AC) - Target', zorder=4)
    link2, = ax.plot([], [], 'b-', linewidth=5, label='L₂ (BD) - Target', zorder=4)
    link3, = ax.plot([], [], 'orange', linestyle='--', linewidth=4, label='L₃ (CE)', zorder=3)
    link4, = ax.plot([], [], 'cyan', linestyle='--', linewidth=4, label='L₄ (DE)', zorder=3)
    link5, = ax.plot([], [], 'g-', linewidth=3.5, label='L₅ (EF)', zorder=4)
    
    # Initialize link lines for ACTUAL (dashed, faded colors)
    link1_actual, = ax.plot([], [], 'r--', linewidth=3, alpha=0.5, label='L₁ (AC) - Actual', zorder=3)
    link2_actual, = ax.plot([], [], 'b--', linewidth=3, alpha=0.5, label='L₂ (BD) - Actual', zorder=3)
    link3_actual, = ax.plot([], [], 'orange', linestyle=':', linewidth=2, alpha=0.4, zorder=2)
    link4_actual, = ax.plot([], [], 'cyan', linestyle=':', linewidth=2, alpha=0.4, zorder=2)
    link5_actual, = ax.plot([], [], 'g--', linewidth=2, alpha=0.5, zorder=3)
    
    # Initialize joint markers for SETPOINT
    joint_c, = ax.plot([], [], 'ro', markersize=10, markeredgecolor='black', markeredgewidth=2, zorder=5)
    joint_d, = ax.plot([], [], 'bo', markersize=10, markeredgecolor='black', markeredgewidth=2, zorder=5)
    joint_e, = ax.plot([], [], 's', color='purple', markersize=8, markeredgecolor='black', markeredgewidth=2, zorder=5)
    foot, = ax.plot([], [], '*', color='green', markersize=20, markeredgecolor='black', markeredgewidth=1.5, zorder=6)
    target, = ax.plot([], [], 'x', color='red', markersize=15, markeredgewidth=3, label='Target', zorder=6)
    
    # Initialize joint markers for ACTUAL (smaller, faded)
    joint_c_actual, = ax.plot([], [], 'ro', markersize=6, alpha=0.5, zorder=4)
    joint_d_actual, = ax.plot([], [], 'bo', markersize=6, alpha=0.5, zorder=4)
    joint_e_actual, = ax.plot([], [], 's', color='purple', markersize=5, alpha=0.5, zorder=4)
    foot_actual, = ax.plot([], [], '*', color='green', markersize=12, alpha=0.5, zorder=5)
    
    # Text display
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                        fontsize=9, verticalalignment='top', family='monospace',
                        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    ax.legend(loc='lower right', fontsize=8, framealpha=0.9)
    
    def update_plot(frame):
        """Update plot with current mechanism state"""
        global plot_running, current_angles, actual_angles, current_target
        
        if not plot_running:
            return (link1, link2, link3, link4, link5, 
                    link1_actual, link2_actual, link3_actual, link4_actual, link5_actual,
                    joint_c, joint_d, joint_e, foot, target,
                    joint_c_actual, joint_d_actual, joint_e_actual, foot_actual, info_text)
        
        with viz_lock:
            theta_A, theta_B = current_angles  # Setpoint angles
            theta_A_actual, theta_B_actual = actual_angles  # Actual angles
            target_x, target_y = current_target
        
        # Calculate SETPOINT positions
        P_C, P_D, P_E, P_F = calculate_fk_positions(theta_A, theta_B)
        
        # Calculate ACTUAL positions
        P_C_actual, P_D_actual, P_E_actual, P_F_actual = calculate_fk_positions(theta_A_actual, theta_B_actual)
        
        if P_C is not None and P_D is not None and P_E is not None and P_F is not None:
            # Update SETPOINT links
            link1.set_data([P_A[0], P_C[0]], [P_A[1], P_C[1]])
            link2.set_data([P_B[0], P_D[0]], [P_B[1], P_D[1]])
            link3.set_data([P_C[0], P_E[0]], [P_C[1], P_E[1]])
            link4.set_data([P_D[0], P_E[0]], [P_D[1], P_E[1]])
            link5.set_data([P_E[0], P_F[0]], [P_E[1], P_F[1]])
            
            # Update SETPOINT joints
            joint_c.set_data([P_C[0]], [P_C[1]])
            joint_d.set_data([P_D[0]], [P_D[1]])
            joint_e.set_data([P_E[0]], [P_E[1]])
            foot.set_data([P_F[0]], [P_F[1]])
            target.set_data([target_x], [target_y])
        
        if P_C_actual is not None and P_D_actual is not None and P_E_actual is not None and P_F_actual is not None:
            # Update ACTUAL links
            link1_actual.set_data([P_A[0], P_C_actual[0]], [P_A[1], P_C_actual[1]])
            link2_actual.set_data([P_B[0], P_D_actual[0]], [P_B[1], P_D_actual[1]])
            link3_actual.set_data([P_C_actual[0], P_E_actual[0]], [P_C_actual[1], P_E_actual[1]])
            link4_actual.set_data([P_D_actual[0], P_E_actual[0]], [P_D_actual[1], P_E_actual[1]])
            link5_actual.set_data([P_E_actual[0], P_F_actual[0]], [P_E_actual[1], P_F_actual[1]])
            
            # Update ACTUAL joints
            joint_c_actual.set_data([P_C_actual[0]], [P_C_actual[1]])
            joint_d_actual.set_data([P_D_actual[0]], [P_D_actual[1]])
            joint_e_actual.set_data([P_E_actual[0]], [P_E_actual[1]])
            foot_actual.set_data([P_F_actual[0]], [P_F_actual[1]])
            
            # Update info text
            info_text.set_text(
                f'═══ Real-Time Status ═══\n'
                f'θA Target:  {np.rad2deg(theta_A):+7.1f}°\n'
                f'θA Actual:  {np.rad2deg(theta_A_actual):+7.1f}°\n'
                f'θB Target:  {np.rad2deg(theta_B):+7.1f}°\n'
                f'θB Actual:  {np.rad2deg(theta_B_actual):+7.1f}°\n\n'
                f'═══ Foot Position ═══\n'
                f'Target: ({P_F[0]:+6.1f}, {P_F[1]:+6.1f})\n'
                f'Actual: ({P_F_actual[0]:+6.1f}, {P_F_actual[1]:+6.1f})\n\n'
                f'═══ Position Error ═══\n'
                f'ΔθA: {np.rad2deg(theta_A - theta_A_actual):+6.1f}°\n'
                f'ΔθB: {np.rad2deg(theta_B - theta_B_actual):+6.1f}°\n'
                f'ΔX:  {P_F[0] - P_F_actual[0]:+6.1f} mm\n'
                f'ΔY:  {P_F[1] - P_F_actual[1]:+6.1f} mm'
            )
        
        return (link1, link2, link3, link4, link5,
                link1_actual, link2_actual, link3_actual, link4_actual, link5_actual,
                joint_c, joint_d, joint_e, foot, target,
                joint_c_actual, joint_d_actual, joint_e_actual, foot_actual, info_text)
    
    # Create animation
    anim = FuncAnimation(fig, update_plot, interval=int(1000/PLOT_UPDATE_RATE), blit=True, cache_frame_data=False)
    
    plt.tight_layout()
    plt.show()
    
    # Set flag when window is closed
    plot_running = False

def start_visualization():
    """Start visualization in separate thread"""
    viz_thread = threading.Thread(target=visualization_thread, daemon=True)
    viz_thread.start()
    time.sleep(1.0)  # Wait for plot to initialize
    return viz_thread

# --- 6. Main Control Loop ---
def main():
    global plot_running, current_angles, actual_angles, current_target
    
    print("="*70)
    print("  BLEGS Gait Control - Real Motor Control System")
    print("="*70)
    print(f"  Motor 1 (Left):  {MOTOR1_PORT}")
    print(f"  Motor 2 (Right): {MOTOR2_PORT}")
    print(f"  Baud Rate: {BAUD_RATE}")
    print(f"  Control Mode: {'S-Curve (Smooth)' if CONTROL_MODE == 1 else 'Direct (Fast)'}")
    print(f"  Initial Angle: {MOTOR_INIT_ANGLE}°")
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
    motor_A = MotorController(MOTOR1_PORT, "Motor A (Left)", MOTOR_INIT_ANGLE)
    motor_B = MotorController(MOTOR2_PORT, "Motor B (Right)", MOTOR_INIT_ANGLE)
    
    # Connect to motors
    print("\n🔌 Connecting to motors...")
    if not motor_A.connect():
        print("❌ Failed to connect Motor A. Exiting...")
        return
    
    if not motor_B.connect():
        print("❌ Failed to connect Motor B. Exiting...")
        motor_A.disconnect()
        return
    
    try:
        # Set control mode
        print(f"\n⚙️  Setting control mode to {CONTROL_MODE}...")
        motor_A.set_mode(CONTROL_MODE)
        motor_B.set_mode(CONTROL_MODE)
        time.sleep(0.5)
        
        # Generate trajectory
        print(f"\n🚶 Generating walking trajectory...")
        trajectory = generate_walking_trajectory(
            num_steps=TRAJECTORY_STEPS,
            lift_height=30,  # mm
            step_forward=60  # mm
        )
        print(f"  Generated {len(trajectory)} waypoints")
        print(f"  Path: Elliptical (60mm × 30mm)")
        print(f"  Update rate: {UPDATE_RATE} Hz")
        
        # Move to home position first
        print(f"\n🏠 Moving to home position (0, -200)...")
        print("  Switching to S-Curve mode for smooth movement...")
        motor_A.switch_mode(1)  # S-Curve mode
        motor_B.switch_mode(1)
        time.sleep(0.2)
        
        home_pos = np.array([0.0, -200.0])
        home_angles = calculate_ik_analytical(home_pos, elbow_C_down=True, elbow_D_down=True)
        
        if not np.isnan(home_angles).any():
            motor_A.set_position(np.rad2deg(home_angles[0]))
            motor_B.set_position(np.rad2deg(home_angles[1]))
            print(f"  Target angles: θA={np.rad2deg(home_angles[0]):.2f}°, θB={np.rad2deg(home_angles[1]):.2f}°")
            time.sleep(3.0)  # Wait for motors to reach home position
        else:
            print("❌ Failed to calculate home position IK")
            return
        
        # Switch to control mode for gait
        print(f"\n⚙️  Switching to {'S-Curve' if CONTROL_MODE == 1 else 'Direct'} mode for gait control...")
        motor_A.switch_mode(CONTROL_MODE)
        motor_B.switch_mode(CONTROL_MODE)
        time.sleep(0.2)
        
        # Start gait cycle
        print(f"\n▶️  Starting gait control...")
        print("  Press Ctrl+C to stop")
        print("="*70)
        
        prev_solution = home_angles
        cycle_count = 0
        
        while True:
            cycle_count += 1
            print(f"\n🔄 Gait Cycle #{cycle_count}")
            
            for frame, (px, py) in enumerate(trajectory):
                # Find best IK solution (closest to previous angles)
                configs = [
                    (True, True),   # Down-Down
                    (True, False),  # Down-Up
                    (False, True),  # Up-Down
                    (False, False)  # Up-Up
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
                    
                    # Send commands to motors
                    success_A = motor_A.set_position(np.rad2deg(theta_A))
                    success_B = motor_B.set_position(np.rad2deg(theta_B))
                    
                    # Check for motor disconnection
                    if not motor_A.is_connected or not motor_B.is_connected:
                        print("\n\n🚨 EMERGENCY STOP: Motor disconnected!")
                        if not motor_A.is_connected:
                            print(f"   ❌ {motor_A.name} lost connection")
                        if not motor_B.is_connected:
                            print(f"   ❌ {motor_B.name} lost connection")
                        raise RuntimeError("Motor disconnection detected - stopping all operations")
                    
                    # Read feedback
                    feedback_A = motor_A.read_feedback()
                    feedback_B = motor_B.read_feedback()
                    
                    # Update actual angles for visualization
                    if feedback_A and feedback_B:
                        with viz_lock:
                            actual_angles = [np.deg2rad(feedback_A[1]), np.deg2rad(feedback_B[1])]
                    
                    # Display status
                    print(f"  [{frame+1:2d}/{len(trajectory)}] "
                          f"Pos:({px:+6.1f}, {py:+6.1f}) "
                          f"θA:{np.rad2deg(theta_A):+6.1f}° "
                          f"θB:{np.rad2deg(theta_B):+6.1f}°", end='')
                    
                    if feedback_A and feedback_B:
                        print(f" | Actual: θA:{feedback_A[1]:+6.1f}° θB:{feedback_B[1]:+6.1f}°")
                    else:
                        print()
                    
                    # Wait for next update
                    time.sleep(1.0 / UPDATE_RATE)
                else:
                    print(f"  ⚠️  [{frame+1:2d}/{len(trajectory)}] Failed to find IK solution for ({px:.1f}, {py:.1f})")
    
    except KeyboardInterrupt:
        print("\n\n⏹️  Gait control stopped by user")
    
    except Exception as e:
        print(f"\n❌ Error during execution: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Stop visualization
        plot_running = False
        
        # Return to home position
        print("\n🏠 Returning to home position...")
        print("  Switching to S-Curve mode for smooth return...")
        try:
            motor_A.switch_mode(1)  # S-Curve mode
            motor_B.switch_mode(1)
            time.sleep(0.2)
            
            motor_A.set_position(np.rad2deg(home_angles[0]))
            motor_B.set_position(np.rad2deg(home_angles[1]))
            time.sleep(2.0)
        except:
            pass
        
        # Disconnect motors
        print("\n🔌 Disconnecting motors...")
        motor_A.disconnect()
        motor_B.disconnect()
        
        print("\n✅ Gait control terminated successfully")
        print("="*70)
        
        # Wait for visualization thread to close
        if viz_thread and ENABLE_VISUALIZATION:
            print("\n📊 Closing visualization...")
            time.sleep(1.0)

if __name__ == "__main__":
    main()
