"""
Quadruped IK Test - No EF Link Version
Author: M-TRCH
Date: January 7, 2026

Tests inverse kinematics for all 4 legs (FR, FL, RR, RL) WITHOUT link EF.
Joint E becomes the foot tip (end-effector).

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

# --- Robot Body Dimensions (mm) ---
BODY_LENGTH = 200.0   # Distance between front and rear hip axes
BODY_WIDTH = 170.0    # Distance between left and right hip axes (2 × 85mm)

# --- Five-Bar Linkage Parameters (mm) ---
MOTOR_SPACING = 85.0  # Distance between Motor A and Motor B (horizontal)

# Link lengths (same for all 4 legs)
L_AC = 105.0  # Link 1 length (Motor A to joint C)
L_BD = 105.0  # Link 2 length (Motor B to joint D)
L_CE = 145.0  # Link 3 length (joint C to joint E)
L_DE = 145.0  # Link 4 length (joint D to joint E)

# NOTE: No L_EF link - Joint E is the foot tip!

# --- Motor Configuration ---
GEAR_RATIO = 8.0  # Motor shaft to output shaft gear ratio
MOTOR_INIT_ANGLE = -90.0  # Initial motor angle (degrees)

# --- Motor Indices for Each Leg ---
MOTOR_INDICES = {
    'FL': {'A': 1, 'B': 2},  # Front Left
    'FR': {'A': 3, 'B': 4},  # Front Right
    'RL': {'A': 5, 'B': 6},  # Rear Left
    'RR': {'A': 7, 'B': 8}   # Rear Right
}

# --- Leg Motor Positions in Leg Frame (mm) ---
P_A_LEFT = np.array([-MOTOR_SPACING/2, 0.0])  # Motor A position for left legs
P_B_LEFT = np.array([MOTOR_SPACING/2, 0.0])   # Motor B position for left legs
P_A_RIGHT = np.array([MOTOR_SPACING/2, 0.0])  # Motor A position for right legs (mirrored)
P_B_RIGHT = np.array([-MOTOR_SPACING/2, 0.0]) # Motor B position for right legs (mirrored)

def get_motor_positions(leg_id):
    """Get motor positions based on leg side"""
    if leg_id in ['FL', 'RL']:
        return P_A_LEFT, P_B_LEFT
    else:  # FR, RR
        return P_A_RIGHT, P_B_RIGHT

# --- Default Standing Pose ---
DEFAULT_STANCE_HEIGHT = -200.0  # Z-coordinate of foot (E) in leg frame (mm, negative = down)
DEFAULT_STANCE_OFFSET_X = 0.0   # X-coordinate of foot (E) in leg frame (mm)

# --- Motion Parameters ---
GAIT_LIFT_HEIGHT = 30.0    # mm
GAIT_STEP_FORWARD = 60.0   # mm

# --- Test Parameters ---
ENABLE_VISUALIZATION = True
PLOT_UPDATE_RATE = 10  # Hz
UPDATE_RATE = 50  # Hz (simulation update rate)
TRAJECTORY_STEPS = 100  # Number of steps in one gait cycle

# --- Gait Pattern ---
GAIT_TYPE = 'trot'  # 'trot', 'walk', 'stand'

# ============================================================================
# GLOBAL VARIABLES
# ============================================================================

# Global variables for visualization (thread-safe)
viz_lock = threading.Lock()
plot_running = True
test_running = False
test_paused = True

# Leg states
leg_states = {
    'FR': {'target_angles': [0.0, 0.0], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'red', 'motor_A': 3, 'motor_B': 4},
    'FL': {'target_angles': [0.0, 0.0], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'blue', 'motor_A': 1, 'motor_B': 2},
    'RR': {'target_angles': [0.0, 0.0], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'orange', 'motor_A': 7, 'motor_B': 8},
    'RL': {'target_angles': [0.0, 0.0], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'green', 'motor_A': 5, 'motor_B': 6}
}

# ============================================================================
# KINEMATICS FUNCTIONS (No EF Link)
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

def calculate_ik_no_ef(P_E_target, P_A, P_B, elbow_C_down=True, elbow_D_down=True):
    """
    Calculate Inverse Kinematics WITHOUT EF link (E is the foot)
    
    Args:
        P_E_target: Target foot position [x, y] in leg frame (this is joint E)
        P_A: Motor A position [x, y]
        P_B: Motor B position [x, y]
        elbow_C_down: True to choose lower elbow position for joint C
        elbow_D_down: True to choose lower elbow position for joint D
        
    Returns:
        numpy array [theta_A, theta_B] in radians, or [nan, nan] if no solution
    """
    # Find joint C position: intersection of circles centered at A (radius L_AC) and E (radius L_CE)
    P_C = solve_circle_intersection(P_A, L_AC, P_E_target, L_CE, elbow_C_down)
    
    if np.isnan(P_C).any():
        return np.array([np.nan, np.nan])
    
    # Find joint D position: intersection of circles centered at B (radius L_BD) and E (radius L_DE)
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

def generate_elliptical_trajectory(num_steps=60, lift_height=30, step_forward=60, mirror_x=False):
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
            px = -px  # Reverse X direction for right side legs
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
        offsets = {'FR': 0.0, 'FL': 0.5, 'RR': 0.5, 'RL': 0.0}
        return offsets[leg_id]
    elif gait_type == 'walk':
        offsets = {'FR': 0.0, 'RR': 0.25, 'FL': 0.5, 'RL': 0.75}
        return offsets[leg_id]
    elif gait_type == 'stand':
        return 0.0
    else:
        return 0.0

# ============================================================================
# CONTROL LOOP
# ============================================================================

def control_loop():
    """Main control loop for quadruped IK test"""
    global test_running, test_paused, leg_states
    
    print("\n🤖 Quadruped IK Test (No EF Link) - Control Loop Started")
    print(f"  Gait Type: {GAIT_TYPE.upper()}")
    print(f"  Update Rate: {UPDATE_RATE} Hz")
    print(f"  Trajectory Steps: {TRAJECTORY_STEPS}")
    print(f"  NOTE: Joint E is the foot (no EF link)")
    print("="*70)
    
    # Generate trajectory for each leg
    trajectories = {}
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        mirror_x = leg_id in ['FR', 'RR']
        trajectories[leg_id] = generate_elliptical_trajectory(
            num_steps=TRAJECTORY_STEPS,
            lift_height=GAIT_LIFT_HEIGHT,
            step_forward=GAIT_STEP_FORWARD,
            mirror_x=mirror_x
        )
    
    # Initialize home position
    prev_solutions = {}
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        home_pos = np.array([DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT])
        P_A, P_B = get_motor_positions(leg_id)
        home_angles = calculate_ik_no_ef(home_pos, P_A, P_B, elbow_C_down=True, elbow_D_down=True)
        
        if not np.isnan(home_angles).any():
            with viz_lock:
                leg_states[leg_id]['target_angles'] = home_angles.tolist()
                leg_states[leg_id]['target_pos'] = home_pos.tolist()
                leg_states[leg_id]['phase'] = get_gait_phase_offset(leg_id, GAIT_TYPE)
            prev_solutions[leg_id] = home_angles
    
    print(f"\n⏸️  Test ready (PAUSED)")
    print("  Press [SPACE] in visualization window to start")
    
    cycle_count = 0
    frame = 0
    
    while test_running:
        if test_paused:
            time.sleep(0.05)
            continue
        
        loop_start = time.perf_counter()
        
        # Update each leg
        for leg_id in ['FR', 'FL', 'RR', 'RL']:
            phase_offset = get_gait_phase_offset(leg_id, GAIT_TYPE)
            current_phase = (frame + int(phase_offset * TRAJECTORY_STEPS)) % TRAJECTORY_STEPS
            
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
                        if elbow_C and elbow_D:  # C=Down, D=Down
                            best_solution = solution
                            break
                    else:
                        # Calculate angular distance
                        angle_diff = np.abs(solution - prev_solutions[leg_id])
                        angle_diff = np.minimum(angle_diff, 2*np.pi - angle_diff)
                        distance = np.sum(angle_diff)
                        
                        if distance < best_distance:
                            best_distance = distance
                            best_solution = solution
            
            if best_solution is not None and not np.isnan(best_solution).any():
                prev_solutions[leg_id] = best_solution
                
                with viz_lock:
                    leg_states[leg_id]['target_angles'] = best_solution.tolist()
                    leg_states[leg_id]['target_pos'] = [px, py]
                    leg_states[leg_id]['phase'] = current_phase
        
        # Update frame counter
        frame = (frame + 1) % TRAJECTORY_STEPS
        
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

def create_leg_subplot(ax, leg_id, leg_name):
    """Create subplot for one leg"""
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-150, 150)
    ax.set_ylim(-300, 50)
    ax.set_xlabel('Y (mm)', fontsize=9)
    ax.set_ylabel('X (mm)', fontsize=9)
    ax.set_title(f'{leg_id} - {leg_name} (No EF)', fontsize=10, weight='bold')
    
    motor_A_idx = leg_states[leg_id]['motor_A']
    motor_B_idx = leg_states[leg_id]['motor_B']
    P_A, P_B = get_motor_positions(leg_id)
    
    # Draw motor positions
    ax.plot(P_A[0], P_A[1], 'o', color='darkblue', markersize=10, label=f'Motor {motor_A_idx}', zorder=5)
    ax.plot(P_B[0], P_B[1], 'o', color='darkred', markersize=10, label=f'Motor {motor_B_idx}', zorder=5)
    
    # Add motor labels
    ax.text(P_A[0], P_A[1]+25, str(motor_A_idx), fontsize=9, weight='bold', 
            color='white', ha='center', va='center', zorder=10,
            bbox=dict(boxstyle='circle', facecolor='darkblue', edgecolor='white', linewidth=1))
    ax.text(P_B[0], P_B[1]+25, str(motor_B_idx), fontsize=9, weight='bold',
            color='white', ha='center', va='center', zorder=10,
            bbox=dict(boxstyle='circle', facecolor='darkred', edgecolor='white', linewidth=1))
    
    color = leg_states[leg_id]['color']
    
    # Links (only 4 links now: AC, BD, CE, DE)
    link1 = ax.plot([], [], '-', color='darkblue', linewidth=3, label='AC', zorder=4)[0]
    link2 = ax.plot([], [], '-', color='darkred', linewidth=3, label='BD', zorder=4)[0]
    link3 = ax.plot([], [], '--', color='orange', linewidth=2, label='CE', zorder=3)[0]
    link4 = ax.plot([], [], '--', color='cyan', linewidth=2, label='DE', zorder=3)[0]
    
    # Joints (C, D, E)
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
        plot_elements['joints'][2].set_data([P_E[0]], [P_E[1]])  # Foot at E
        
        # Update info text
        motor_A_idx = leg_states[leg_id]['motor_A']
        motor_B_idx = leg_states[leg_id]['motor_B']
        plot_elements['info_text'].set_text(
            f'Motor {motor_A_idx}: {np.rad2deg(theta_A):+6.1f}°\n'
            f'Motor {motor_B_idx}: {np.rad2deg(theta_B):+6.1f}°\n'
            f'Foot (E): ({P_E[0]:.0f},{P_E[1]:.0f})'
        )

def visualization_thread():
    """Thread function for real-time visualization"""
    global plot_running
    
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f'Quadruped IK Test - {GAIT_TYPE.upper()} Gait (No EF Link)', fontsize=14, weight='bold')
    
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
    fig.text(0.5, 0.02, 'Controls: [SPACE] Start/Stop | Joint E = Foot (No EF Link)', 
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
            artists.append(plot_elements[leg_id]['info_text'])
        
        return artists
    
    def on_key_press(event):
        if event.key == ' ':
            toggle_test_control()
    
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
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
    print("  BLEGS Quadruped IK Test - No EF Link Version")
    print("="*70)
    print(f"  Number of Legs: 4 (FR, FL, RR, RL)")
    print(f"  Gait Type: {GAIT_TYPE.upper()}")
    print(f"  Body Size: {BODY_LENGTH}×{BODY_WIDTH} mm")
    print(f"  Motor Spacing: {MOTOR_SPACING} mm")
    print(f"  Link Lengths: AC={L_AC}, BD={L_BD}, CE={L_CE}, DE={L_DE}")
    print(f"  NOTE: Joint E is the FOOT (no EF link)")
    print(f"  Update Rate: {UPDATE_RATE} Hz")
    print(f"  Trajectory Steps: {TRAJECTORY_STEPS}")
    print(f"  Visualization: {'Enabled' if ENABLE_VISUALIZATION else 'Disabled'}")
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
