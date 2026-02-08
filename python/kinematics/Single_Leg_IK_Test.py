"""
Single Leg IK Test with Link Length Display
Author: M-TRCH
Date: January 4, 2026

Tests inverse kinematics for a single leg with real-time visualization.
Displays link lengths on the graphics.
This is a simulation/test version - does NOT send commands to real motors.

Standalone file - all dependencies included.
"""

import numpy as np
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================================================
# CONFIGURATION
# ============================================================================

# --- Five-Bar Linkage Parameters (mm) ---
MOTOR_SPACING = 85.0  # Distance between Motor A and Motor B (horizontal)

# Link lengths
L_AC = 105.0  # Link 1 length (Motor A to joint C)
L_BD = 105.0  # Link 2 length (Motor B to joint D)
L_CE = 145.0  # Link 3 length (joint C to joint E)
L_DE = 145.0  # Link 4 length (joint D to joint E)
L_EF = 40.0   # Offset length (joint E to foot F)

# Offset ratios (for calculating joint E position from D and F)
OFFSET_RATIO_E = 37.0 / 29.0
OFFSET_RATIO_D = 8.0 / 29.0

# --- Leg Motor Positions in Leg Frame (mm) ---
# X-axis points down, Y-axis points outward from body
P_A = np.array([-MOTOR_SPACING/2, 0.0])  # Motor A position (left/inner)
P_B = np.array([MOTOR_SPACING/2, 0.0])   # Motor B position (right/outer)

# --- Default Standing Pose ---
DEFAULT_STANCE_HEIGHT = -200.0  # Z-coordinate of foot in leg frame (mm, negative = down)
DEFAULT_STANCE_OFFSET_X = 0.0   # X-coordinate of foot in leg frame (mm)

# --- Motion Parameters ---
GAIT_LIFT_HEIGHT = 30.0    # mm
GAIT_STEP_FORWARD = 60.0   # mm

# --- Test Parameters ---
ENABLE_VISUALIZATION = True
PLOT_UPDATE_RATE = 10  # Hz
UPDATE_RATE = 50  # Hz (simulation update rate)
TRAJECTORY_STEPS = 100  # Number of steps in one gait cycle

# ============================================================================
# GLOBAL VARIABLES
# ============================================================================

# Global variables for visualization (thread-safe)
viz_lock = threading.Lock()
plot_running = True
test_running = False
test_paused = True

# Leg state
leg_state = {
    'target_angles': [0.0, 0.0],
    'target_pos': [0.0, -200.0],
    'phase': 0
}

# ============================================================================
# KINEMATICS FUNCTIONS
# ============================================================================

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
    """
    Calculate Inverse Kinematics using analytical method
    
    Args:
        P_F_target: Target foot position [x, y] in leg frame
        elbow_C_down: True to choose lower elbow position for joint C
        elbow_D_down: True to choose lower elbow position for joint D
        
    Returns:
        numpy array [theta_A, theta_B] in radians, or [nan, nan] if no solution
    """
    (x_f, y_f) = P_F_target
    
    R_FD = L_DE * OFFSET_RATIO_E
    R_DB = L_BD
    
    # Find joint D position
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

def calculate_fk_positions(theta_A, theta_B):
    """Calculate forward kinematics positions for visualization"""
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
            
            P_E1 = P_C + a * v_d + h * v_perp
            P_E2 = P_C + a * v_d - h * v_perp
            
            P_F1 = (37.0 * P_E1 - 8.0 * P_D) / 29.0
            P_F2 = (37.0 * P_E2 - 8.0 * P_D) / 29.0
            
            # Choose configuration with lower foot position
            if P_F1[1] < P_F2[1]:
                P_E = P_E1
                P_F = P_F1
            else:
                P_E = P_E2
                P_F = P_F2
            
            return P_C, P_D, P_E, P_F
    
    return None, None, None, None

# ============================================================================
# TRAJECTORY GENERATION
# ============================================================================

def generate_elliptical_trajectory(num_steps=60, lift_height=30, step_forward=60):
    """
    Generate elliptical walking trajectory
    
    Args:
        num_steps: Number of steps in trajectory
        lift_height: Maximum height lift (mm)
        step_forward: Step length (mm)
        
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
        py = home_y + b * np.sin(t)
        trajectory.append((px, py))
    
    return trajectory

# ============================================================================
# CONTROL LOOP
# ============================================================================

def control_loop():
    """Main control loop for single leg IK test"""
    global test_running, test_paused, leg_state
    
    print("\n🤖 Single Leg IK Test - Control Loop Started")
    print(f"  Update Rate: {UPDATE_RATE} Hz")
    print(f"  Trajectory Steps: {TRAJECTORY_STEPS}")
    print("="*70)
    
    # Generate trajectory
    trajectory = generate_elliptical_trajectory(
        num_steps=TRAJECTORY_STEPS,
        lift_height=GAIT_LIFT_HEIGHT,
        step_forward=GAIT_STEP_FORWARD
    )
    
    # Initialize home position
    prev_solution = None
    home_pos = np.array([DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT])
    home_angles = calculate_ik_analytical(home_pos, elbow_C_down=True, elbow_D_down=False)
    
    if not np.isnan(home_angles).any():
        with viz_lock:
            leg_state['target_angles'] = home_angles.tolist()
            leg_state['target_pos'] = home_pos.tolist()
            leg_state['phase'] = 0
        prev_solution = home_angles
    
    print(f"\n⏸️  Test ready (PAUSED)")
    print("  Press [SPACE] in visualization window to start")
    
    cycle_count = 0
    frame = 0
    
    while test_running:
        # Check if paused
        if test_paused:
            time.sleep(0.05)
            continue
        
        loop_start = time.perf_counter()
        
        # Get target position from trajectory
        px, py = trajectory[frame]
        
        # Calculate IK - try all configurations
        configs = [
            (True, False),   # C=Down, D=Down (actual)
            (True, True),    # C=Down, D=Up (actual)
            (False, False),  # C=Up, D=Down (actual)
            (False, True)    # C=Up, D=Up (actual)
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
                if prev_solution is None:
                    if elbow_C and not elbow_D:  # C=Down, D=Down (actual)
                        best_solution = solution
                        break
                else:
                    # Calculate angular distance
                    angle_diff = np.abs(solution - prev_solution)
                    angle_diff = np.minimum(angle_diff, 2*np.pi - angle_diff)
                    distance = np.sum(angle_diff)
                    
                    if distance < best_distance:
                        best_distance = distance
                        best_solution = solution
        
        if best_solution is not None and not np.isnan(best_solution).any():
            prev_solution = best_solution
            
            # Update target angles
            with viz_lock:
                leg_state['target_angles'] = best_solution.tolist()
                leg_state['target_pos'] = [px, py]
                leg_state['phase'] = frame
        
        # Update frame counter
        frame = (frame + 1) % TRAJECTORY_STEPS
        
        # Print status every cycle
        if frame == 0:
            cycle_count += 1
            print(f"  Cycle #{cycle_count} completed")
        
        # Timing control
        elapsed = time.perf_counter() - loop_start
        sleep_time = max(0, (1.0 / UPDATE_RATE) - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    print("\n⏹️  Control loop stopped")

def toggle_test_control():
    """Toggle test control on/off"""
    global test_paused
    test_paused = not test_paused
    if test_paused:
        print("\n⏸️  Test PAUSED")
    else:
        print("\n▶️  Test STARTED")

# ============================================================================
# VISUALIZATION
# ============================================================================

def visualization_thread():
    """Thread function for real-time visualization"""
    global plot_running
    
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.suptitle('Single Leg IK Test with Link Lengths', fontsize=14, weight='bold')
    
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-150, 150)
    ax.set_ylim(-300, 50)
    ax.set_xlabel('Y (mm)', fontsize=11)
    ax.set_ylabel('X (mm)', fontsize=11)
    
    # Draw motor positions
    ax.plot(P_A[0], P_A[1], 'o', color='darkblue', markersize=12, label='Motor A', zorder=5)
    ax.plot(P_B[0], P_B[1], 'o', color='darkred', markersize=12, label='Motor B', zorder=5)
    
    # Add motor labels
    ax.text(P_A[0], P_A[1]+25, 'A', fontsize=11, weight='bold',
            color='white', ha='center', va='center', zorder=10,
            bbox=dict(boxstyle='circle', facecolor='darkblue', edgecolor='white', linewidth=1.5))
    ax.text(P_B[0], P_B[1]+25, 'B', fontsize=11, weight='bold',
            color='white', ha='center', va='center', zorder=10,
            bbox=dict(boxstyle='circle', facecolor='darkred', edgecolor='white', linewidth=1.5))
    
    # Create link lines
    link_AC = ax.plot([], [], '-', color='darkblue', linewidth=4, label='AC', zorder=4)[0]
    link_BD = ax.plot([], [], '-', color='darkred', linewidth=4, label='BD', zorder=4)[0]
    link_CE = ax.plot([], [], '--', color='orange', linewidth=3, label='CE', zorder=3)[0]
    link_DE = ax.plot([], [], '--', color='cyan', linewidth=3, label='DE', zorder=3)[0]
    link_EF = ax.plot([], [], '-', color='green', linewidth=3.5, label='EF', zorder=4)[0]
    
    # Create joints
    joint_c = ax.plot([], [], 'o', color='red', markersize=8, zorder=5)[0]
    joint_d = ax.plot([], [], 'o', color='blue', markersize=8, zorder=5)[0]
    joint_e = ax.plot([], [], 's', color='purple', markersize=7, zorder=5)[0]
    joint_f = ax.plot([], [], 'D', color='green', markersize=10, zorder=5)[0]
    
    # Create text annotations for link lengths
    text_AC = ax.text(0, 0, '', fontsize=10, weight='bold', color='darkblue', zorder=15)
    text_BD = ax.text(0, 0, '', fontsize=10, weight='bold', color='darkred', zorder=15)
    text_CE = ax.text(0, 0, '', fontsize=10, weight='bold', color='orange', zorder=15)
    text_DE = ax.text(0, 0, '', fontsize=10, weight='bold', color='cyan', zorder=15)
    text_EF = ax.text(0, 0, '', fontsize=10, weight='bold', color='green', zorder=15)
    
    # Info text
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                        fontsize=9, verticalalignment='top', family='monospace',
                        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.95))
    
    ax.legend(loc='lower right', fontsize=9, framealpha=0.9)
    
    def update_plot(frame):
        if not plot_running:
            return []
        
        with viz_lock:
            theta_A, theta_B = leg_state['target_angles']
            target_x, target_y = leg_state['target_pos']
        
        # Calculate FK
        P_C, P_D, P_E, P_F = calculate_fk_positions(theta_A, theta_B)
        
        if P_C is not None:
            # Update links
            link_AC.set_data([P_A[0], P_C[0]], [P_A[1], P_C[1]])
            link_BD.set_data([P_B[0], P_D[0]], [P_B[1], P_D[1]])
            link_CE.set_data([P_C[0], P_E[0]], [P_C[1], P_E[1]])
            link_DE.set_data([P_D[0], P_E[0]], [P_D[1], P_E[1]])
            link_EF.set_data([P_E[0], P_F[0]], [P_E[1], P_F[1]])
            
            # Update joints
            joint_c.set_data([P_C[0]], [P_C[1]])
            joint_d.set_data([P_D[0]], [P_D[1]])
            joint_e.set_data([P_E[0]], [P_E[1]])
            joint_f.set_data([P_F[0]], [P_F[1]])
            
            # Update link length annotations (with offset from midpoint)
            mid_AC = (P_A + P_C) / 2
            mid_BD = (P_B + P_D) / 2
            mid_CE = (P_C + P_E) / 2
            mid_DE = (P_D + P_E) / 2
            mid_EF = (P_E + P_F) / 2
            
            # Calculate perpendicular offset for text positioning (10mm offset)
            offset = 10
            
            # AC - offset to the left (perpendicular)
            vec_AC = P_C - P_A
            perp_AC = np.array([-vec_AC[1], vec_AC[0]]) / np.linalg.norm(vec_AC) * offset
            text_AC.set_position((mid_AC[0] + perp_AC[0], mid_AC[1] + perp_AC[1]))
            text_AC.set_text(f'{L_AC:.0f}')
            
            # BD - offset perpendicular
            vec_BD = P_D - P_B
            perp_BD = np.array([-vec_BD[1], vec_BD[0]]) / np.linalg.norm(vec_BD) * offset
            text_BD.set_position((mid_BD[0] + perp_BD[0], mid_BD[1] + perp_BD[1]))
            text_BD.set_text(f'{L_BD:.0f}')
            
            # CE - offset perpendicular
            vec_CE = P_E - P_C
            perp_CE = np.array([-vec_CE[1], vec_CE[0]]) / np.linalg.norm(vec_CE) * offset
            text_CE.set_position((mid_CE[0] + perp_CE[0], mid_CE[1] + perp_CE[1]))
            text_CE.set_text(f'{L_CE:.0f}')
            
            # DE - offset perpendicular
            vec_DE = P_E - P_D
            perp_DE = np.array([-vec_DE[1], vec_DE[0]]) / np.linalg.norm(vec_DE) * offset
            text_DE.set_position((mid_DE[0] + perp_DE[0], mid_DE[1] + perp_DE[1]))
            text_DE.set_text(f'{L_DE:.0f}')
            
            # EF - offset perpendicular
            vec_EF = P_F - P_E
            perp_EF = np.array([-vec_EF[1], vec_EF[0]]) / np.linalg.norm(vec_EF) * offset
            text_EF.set_position((mid_EF[0] + perp_EF[0], mid_EF[1] + perp_EF[1]))
            text_EF.set_text(f'{L_EF:.0f}')
            
            # Update info text
            info_text.set_text(
                f'Motor A: {np.rad2deg(theta_A):+7.2f}°\n'
                f'Motor B: {np.rad2deg(theta_B):+7.2f}°\n'
                f'Target: ({target_x:.1f}, {target_y:.1f})\n'
                f'Actual: ({P_F[0]:.1f}, {P_F[1]:.1f})'
            )
        
        return [link_AC, link_BD, link_CE, link_DE, link_EF,
                joint_c, joint_d, joint_e, joint_f,
                text_AC, text_BD, text_CE, text_DE, text_EF, info_text]
    
    def on_key_press(event):
        if event.key == ' ':
            toggle_test_control()
    
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    # Add control instructions
    fig.text(0.5, 0.02, 'Controls: [SPACE] Start/Stop',
             ha='center', fontsize=11, family='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.9))
    
    anim = FuncAnimation(fig, update_plot, interval=int(1000/PLOT_UPDATE_RATE),
                        blit=True, cache_frame_data=False)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.98])
    plt.show()
    
    plot_running = False

def start_visualization():
    """Start visualization in main thread"""
    visualization_thread()

# ============================================================================
# MAIN FUNCTION
# ============================================================================

def main():
    global test_running, test_paused, plot_running
    
    print("="*70)
    print("  BLEGS Single Leg IK Test with Link Length Display")
    print("="*70)
    print(f"  Link Lengths:")
    print(f"    AC: {L_AC:.0f} mm")
    print(f"    BD: {L_BD:.0f} mm")
    print(f"    CE: {L_CE:.0f} mm")
    print(f"    DE: {L_DE:.0f} mm")
    print(f"    EF: {L_EF:.0f} mm")
    print(f"  Motor Spacing: {MOTOR_SPACING:.0f} mm")
    print(f"  Update Rate: {UPDATE_RATE} Hz")
    print(f"  Trajectory Steps: {TRAJECTORY_STEPS}")
    print("="*70)
    
    # Start control loop
    print("\n🚀 Starting control loop...")
    test_running = True
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()
    
    # Start visualization (blocks until window closed)
    if ENABLE_VISUALIZATION:
        print("\n📊 Starting visualization...")
        try:
            start_visualization()
        except KeyboardInterrupt:
            print("\n\n⏹️  Test stopped by user")
    else:
        try:
            while test_running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n\n⏹️  Test stopped by user")
    
    # Cleanup
    test_running = False
    plot_running = False
    
    print("\n✅ Test terminated successfully")
    print("="*70)

if __name__ == "__main__":
    main()
