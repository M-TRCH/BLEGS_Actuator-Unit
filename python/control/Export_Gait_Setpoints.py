"""
Export Gait Angle Setpoints for MCU Testing
Author: M-TRCH
Date: December 4, 2025

This script generates and exports motor angle setpoints from the gait trajectory
for MCU-side testing and debugging.
"""

import numpy as np
import csv
import json

# --- Robot Kinematic Parameters (mm) ---
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

# --- Gait Parameters (Match Gait_Control_Binary_Protocol.py) ---
UPDATE_RATE = 200  # Hz (5ms per update)
TRAJECTORY_STEPS = 300  # Number of steps in one gait cycle
LIFT_HEIGHT = 30  # mm
STEP_FORWARD = 60  # mm
HOME_Y = -200  # mm

# Gear ratio
GEAR_RATIO = 8.0

# --- Kinematics Functions ---
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

def generate_walking_trajectory(num_steps=300, lift_height=30, step_forward=60):
    """Generate elliptical walking trajectory"""
    trajectory = []
    home_y = HOME_Y
    
    a = step_forward
    b = lift_height
    
    for i in range(num_steps):
        t = 2 * np.pi * i / num_steps
        px = a * np.cos(t)
        py = home_y + b * np.sin(t)
        trajectory.append((px, py))
    
    return trajectory

# --- Main Export Function ---
def export_gait_setpoints():
    """Generate and export gait angle setpoints"""
    
    print("="*70)
    print("  Gait Angle Setpoints Export for MCU Testing")
    print("="*70)
    print(f"  Update Rate: {UPDATE_RATE} Hz ({1000.0/UPDATE_RATE:.1f} ms per step)")
    print(f"  Trajectory Steps: {TRAJECTORY_STEPS}")
    print(f"  Gait Cycle Duration: {TRAJECTORY_STEPS/UPDATE_RATE*1000:.0f} ms")
    print(f"  Ellipse: {STEP_FORWARD}mm × {LIFT_HEIGHT}mm")
    print(f"  Gear Ratio: {GEAR_RATIO}:1")
    print("="*70)
    
    # Generate trajectory
    print(f"\n📐 Generating trajectory...")
    trajectory = generate_walking_trajectory(
        num_steps=TRAJECTORY_STEPS,
        lift_height=LIFT_HEIGHT,
        step_forward=STEP_FORWARD
    )
    print(f"  ✅ Generated {len(trajectory)} waypoints")
    
    # Calculate IK for all points
    print(f"\n🔧 Calculating inverse kinematics...")
    
    # Start from home position
    home_pos = np.array([0.0, HOME_Y])
    home_angles = calculate_ik_analytical(home_pos, elbow_C_down=True, elbow_D_down=True)
    
    if np.isnan(home_angles).any():
        print("❌ Failed to calculate home position IK")
        return
    
    print(f"  Home Position: ({home_pos[0]}, {home_pos[1]}) mm")
    print(f"  Home Angles: θA={np.rad2deg(home_angles[0]):.2f}°, θB={np.rad2deg(home_angles[1]):.2f}°")
    
    # Data storage
    data = []
    prev_solution = home_angles
    
    # IK configuration options
    configs = [
        (True, True),
        (True, False),
        (False, True),
        (False, False)
    ]
    
    for step_num, (px, py) in enumerate(trajectory):
        # Find best IK solution (closest to previous)
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
            
            # Convert to degrees (robot coordinate)
            theta_A_deg = np.rad2deg(theta_A)
            theta_B_deg = np.rad2deg(theta_B)
            
            # Convert to motor shaft angle (with gear ratio)
            motor_A_deg = theta_A_deg * GEAR_RATIO
            motor_B_deg = theta_B_deg * GEAR_RATIO
            
            # Time stamp
            time_ms = step_num * (1000.0 / UPDATE_RATE)
            
            data.append({
                'step': step_num,
                'time_ms': time_ms,
                'target_x_mm': px,
                'target_y_mm': py,
                'theta_A_deg': theta_A_deg,
                'theta_B_deg': theta_B_deg,
                'motor_A_shaft_deg': motor_A_deg,
                'motor_B_shaft_deg': motor_B_deg,
                'motor_A_shaft_deg_x100': int(motor_A_deg * 100),  # MCU format (degrees*100)
                'motor_B_shaft_deg_x100': int(motor_B_deg * 100)   # MCU format (degrees*100)
            })
        else:
            print(f"  ⚠️  IK failed for step {step_num}: ({px:.1f}, {py:.1f})")
    
    print(f"  ✅ Calculated IK for {len(data)}/{TRAJECTORY_STEPS} steps")
    
    # --- Export to CSV ---
    csv_filename = "gait_setpoints_300steps_200hz.csv"
    print(f"\n💾 Exporting to CSV: {csv_filename}")
    
    with open(csv_filename, 'w', newline='') as csvfile:
        fieldnames = [
            'step', 'time_ms', 'target_x_mm', 'target_y_mm',
            'theta_A_deg', 'theta_B_deg',
            'motor_A_shaft_deg', 'motor_B_shaft_deg',
            'motor_A_shaft_deg_x100', 'motor_B_shaft_deg_x100'
        ]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for row in data:
            writer.writerow(row)
    
    print(f"  ✅ CSV exported successfully")
    
    # --- Export to JSON ---
    json_filename = "gait_setpoints_300steps_200hz.json"
    print(f"\n💾 Exporting to JSON: {json_filename}")
    
    json_data = {
        'metadata': {
            'update_rate_hz': UPDATE_RATE,
            'trajectory_steps': TRAJECTORY_STEPS,
            'cycle_duration_ms': TRAJECTORY_STEPS / UPDATE_RATE * 1000,
            'step_duration_ms': 1000.0 / UPDATE_RATE,
            'lift_height_mm': LIFT_HEIGHT,
            'step_forward_mm': STEP_FORWARD,
            'home_y_mm': HOME_Y,
            'gear_ratio': GEAR_RATIO,
            'motor_shaft_unit': 'degrees * 100 (int32)',
            'description': 'Gait angle setpoints for MCU testing - 300 steps @ 200 Hz'
        },
        'setpoints': data
    }
    
    with open(json_filename, 'w') as jsonfile:
        json.dump(json_data, jsonfile, indent=2)
    
    print(f"  ✅ JSON exported successfully")
    
    # --- Export MCU-friendly format (compact) ---
    mcu_filename = "gait_setpoints_mcu.txt"
    print(f"\n💾 Exporting MCU format: {mcu_filename}")
    
    with open(mcu_filename, 'w') as mcufile:
        mcufile.write(f"# Gait Setpoints for MCU Testing\n")
        mcufile.write(f"# Update Rate: {UPDATE_RATE} Hz ({1000.0/UPDATE_RATE:.1f} ms per step)\n")
        mcufile.write(f"# Total Steps: {TRAJECTORY_STEPS}\n")
        mcufile.write(f"# Cycle Duration: {TRAJECTORY_STEPS/UPDATE_RATE*1000:.0f} ms\n")
        mcufile.write(f"# Format: step, time_ms, motor_A_deg*100, motor_B_deg*100\n")
        mcufile.write(f"#\n")
        
        for row in data:
            mcufile.write(f"{row['step']:3d}, {row['time_ms']:7.1f}, "
                         f"{row['motor_A_shaft_deg_x100']:6d}, "
                         f"{row['motor_B_shaft_deg_x100']:6d}\n")
    
    print(f"  ✅ MCU format exported successfully")
    
    # --- Statistics ---
    print(f"\n📊 Statistics:")
    theta_A_values = [d['theta_A_deg'] for d in data]
    theta_B_values = [d['theta_B_deg'] for d in data]
    motor_A_values = [d['motor_A_shaft_deg'] for d in data]
    motor_B_values = [d['motor_B_shaft_deg'] for d in data]
    
    print(f"\n  Robot Coordinate (Output Shaft):")
    print(f"    Motor A: {min(theta_A_values):+7.2f}° to {max(theta_A_values):+7.2f}° (Range: {max(theta_A_values)-min(theta_A_values):.2f}°)")
    print(f"    Motor B: {min(theta_B_values):+7.2f}° to {max(theta_B_values):+7.2f}° (Range: {max(theta_B_values)-min(theta_B_values):.2f}°)")
    
    print(f"\n  Motor Shaft (with {GEAR_RATIO}:1 ratio):")
    print(f"    Motor A: {min(motor_A_values):+8.2f}° to {max(motor_A_values):+8.2f}° (Range: {max(motor_A_values)-min(motor_A_values):.2f}°)")
    print(f"    Motor B: {min(motor_B_values):+8.2f}° to {max(motor_B_values):+8.2f}° (Range: {max(motor_B_values)-min(motor_B_values):.2f}°)")
    
    print(f"\n  MCU Format (degrees × 100):")
    mcu_A_values = [d['motor_A_shaft_deg_x100'] for d in data]
    mcu_B_values = [d['motor_B_shaft_deg_x100'] for d in data]
    print(f"    Motor A: {min(mcu_A_values):+7d} to {max(mcu_A_values):+7d}")
    print(f"    Motor B: {min(mcu_B_values):+7d} to {max(mcu_B_values):+7d}")
    
    print(f"\n✅ Export completed successfully!")
    print("="*70)
    print(f"\n📁 Generated Files:")
    print(f"  1. {csv_filename} - Full data with all columns")
    print(f"  2. {json_filename} - Structured data with metadata")
    print(f"  3. {mcu_filename} - Compact format for MCU testing")
    print("="*70)

if __name__ == "__main__":
    export_gait_setpoints()
