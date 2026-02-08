"""
Plot Foot Tip Path - Quadruped Gait Trajectory Visualization
Author: M-TRCH
Date: January 7, 2026

This script visualizes the foot tip trajectories for different gait modes
(trot, smooth trot, backward trot, walk, crawl) used in the quadruped robot.

Features:
- Plots elliptical swing-stance trajectories for all 4 legs
- Multiple gait mode visualization
- 2D and 3D trajectory plotting
- Phase offset visualization
- Trajectory comparison
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from mpl_toolkits.mplot3d import Axes3D

# ============================================================================
# GAIT PARAMETERS (from Quadruped_Gait_Control.py)
# ============================================================================

# Default Standing Pose
DEFAULT_STANCE_HEIGHT = -200.0  # mm
DEFAULT_STANCE_OFFSET_X = 0.0   # mm

# Motion Parameters for Different Gaits
GAIT_PARAMS = {
    'trot': {
        'step_forward': 45.0,      # mm
        'lift_height': 15.0,       # mm
        'steps': 20,               # Total steps per cycle
        'stance_ratio': 0.5,       # 50% stance, 50% swing
        'description': 'TROT - Fast & Efficient (90mm/s)',
        'color': 'red'
    },
    'smooth_trot': {
        'step_forward': 50.0,      # mm
        'lift_height': 15.0,       # mm
        'steps': 30,               # Total steps per cycle
        'stance_ratio': 0.65,      # 65% stance, 35% swing
        'description': 'SMOOTH TROT - Balanced & Stable (80mm/s)',
        'color': 'blue'
    },
    'backward_trot': {
        'step_forward': 50.0,      # mm (same as smooth trot)
        'lift_height': 15.0,       # mm
        'steps': 30,               # Total steps per cycle
        'stance_ratio': 0.65,      # 65% stance, 35% swing
        'description': 'BACKWARD TROT - Reverse Motion (80mm/s)',
        'color': 'purple'
    },
    'walk': {
        'step_forward': 30.0,      # mm
        'lift_height': 15.0,       # mm
        'steps': 30,               # Total steps per cycle
        'stance_ratio': 0.75,      # 75% stance, 25% swing
        'description': 'WALK - Slow & Stable (50mm/s)',
        'color': 'green'
    },
    'crawl': {
        'step_forward': 20.0,      # mm
        'lift_height': 10.0,       # mm
        'steps': 40,               # Total steps per cycle
        'stance_ratio': 0.75,      # 75% stance, 25% swing
        'description': 'CRAWL - Very Slow & Stable (25mm/s)',
        'color': 'orange'
    }
}

# Leg Phase Offsets for Different Gaits
GAIT_PHASE_OFFSETS = {
    'trot': {
        'FR': 0.0,   'FL': 0.5,   'RR': 0.5,   'RL': 0.0
    },
    'smooth_trot': {
        'FR': 0.0,   'FL': 0.5,   'RR': 0.5,   'RL': 0.0
    },
    'backward_trot': {
        'FR': 0.0,   'FL': 0.5,   'RR': 0.5,   'RL': 0.0
    },
    'walk': {
        'FR': 0.0,   'RR': 0.25,  'FL': 0.5,   'RL': 0.75
    },
    'crawl': {
        'FR': 0.0,   'RR': 0.25,  'FL': 0.5,   'RL': 0.75
    }
}

# Leg colors for visualization
LEG_COLORS = {
    'FR': 'red',
    'FL': 'blue',
    'RR': 'orange',
    'RL': 'green'
}

# ============================================================================
# TRAJECTORY GENERATION
# ============================================================================

def generate_elliptical_trajectory(step_forward, lift_height, num_steps, stance_ratio=0.5, 
                                   home_x=0.0, home_y=DEFAULT_STANCE_HEIGHT, 
                                   reverse=False, mirror_x=False):
    """
    Generate elliptical foot trajectory (same as in Quadruped_Gait_Control.py)
    
    Args:
        step_forward: Forward step distance (mm)
        lift_height: Lift height during swing (mm)
        num_steps: Total number of steps in one cycle
        stance_ratio: Ratio of stance phase (0 to 1)
        home_x: Home X position offset (mm)
        home_y: Home Y position (height) (mm)
        reverse: True for backward motion
        mirror_x: True to mirror X axis (for right side legs)
        
    Returns:
        List of (x, y) tuples representing foot positions
    """
    trajectory = []
    
    # Calculate swing and stance steps
    swing_steps = int(num_steps * (1.0 - stance_ratio))
    stance_steps = num_steps - swing_steps
    
    # Direction multiplier: +1 for forward, -1 for backward
    direction = -1 if reverse else 1
    
    for i in range(num_steps):
        if i < swing_steps:
            # Swing phase FIRST: π to 2π (top half of ellipse - in air)
            phase_progress = i / swing_steps
            t = np.pi + np.pi * phase_progress  # π to 2π
        else:
            # Stance phase SECOND: 0 to π (bottom half of ellipse - on ground)
            stance_index = i - swing_steps
            phase_progress = stance_index / stance_steps
            t = np.pi * phase_progress  # 0 to π
        
        # Elliptical motion
        px = direction * (-step_forward * np.cos(t))
        if mirror_x:
            px = -px
        
        # Y: Only lift during swing phase
        if i < swing_steps:  # Swing phase
            py = home_y + lift_height * abs(np.sin(t))
        else:  # Stance phase
            py = home_y  # Stay on ground
        
        trajectory.append((px + home_x, py))
    
    return trajectory

def generate_leg_trajectories(gait_type='trot', phase_offset=0.0):
    """
    Generate trajectories for all 4 legs
    
    Args:
        gait_type: Type of gait ('trot', 'smooth_trot', 'backward_trot', 'walk', 'crawl')
        phase_offset: Phase offset for the leg (0.0 to 1.0)
        
    Returns:
        Dictionary with leg trajectories
    """
    params = GAIT_PARAMS[gait_type]
    phase_offsets = GAIT_PHASE_OFFSETS[gait_type]
    
    reverse = (gait_type == 'backward_trot')
    
    leg_trajectories = {}
    
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        mirror = (leg_id in ['FR', 'RR'])  # Mirror X for right legs
        
        # Generate base trajectory
        trajectory = generate_elliptical_trajectory(
            step_forward=params['step_forward'],
            lift_height=params['lift_height'],
            num_steps=params['steps'],
            stance_ratio=params['stance_ratio'],
            reverse=reverse,
            mirror_x=mirror
        )
        
        # Apply phase offset
        offset_steps = int(phase_offsets[leg_id] * params['steps'])
        trajectory = trajectory[offset_steps:] + trajectory[:offset_steps]
        
        leg_trajectories[leg_id] = trajectory
    
    return leg_trajectories

# ============================================================================
# PLOTTING FUNCTIONS
# ============================================================================

def plot_single_gait(gait_type='trot', save_fig=False):
    """
    Plot foot trajectories for a single gait type (all 4 legs)
    """
    params = GAIT_PARAMS[gait_type]
    leg_trajectories = generate_leg_trajectories(gait_type)
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    for leg_id, trajectory in leg_trajectories.items():
        x_data = [p[0] for p in trajectory]
        y_data = [p[1] for p in trajectory]
        
        # Plot trajectory
        ax.plot(x_data, y_data, '-', color=LEG_COLORS[leg_id], 
                linewidth=2, label=f'{leg_id} Leg', alpha=0.7)
        
        # Mark start point
        ax.plot(x_data[0], y_data[0], 'o', color=LEG_COLORS[leg_id], 
                markersize=10, markeredgecolor='black', markeredgewidth=1.5)
        
        # Mark end point
        ax.plot(x_data[-1], y_data[-1], 's', color=LEG_COLORS[leg_id], 
                markersize=8, markeredgecolor='black', markeredgewidth=1.5)
    
    ax.axhline(y=DEFAULT_STANCE_HEIGHT, color='gray', linestyle='--', 
               linewidth=1, label='Ground Level', alpha=0.5)
    ax.axvline(x=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    
    ax.set_xlabel('X Position (mm)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (mm)', fontsize=12, fontweight='bold')
    ax.set_title(f'Foot Tip Trajectories - {params["description"]}', 
                 fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    
    if save_fig:
        filename = f'foot_path_{gait_type}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"✅ Saved: {filename}")
    
    plt.show()

def plot_all_gaits_comparison(save_fig=False):
    """
    Plot comparison of all gait types (single leg FR for clarity)
    """
    fig, ax = plt.subplots(figsize=(14, 8))
    
    for gait_type, params in GAIT_PARAMS.items():
        leg_trajectories = generate_leg_trajectories(gait_type)
        trajectory = leg_trajectories['FR']  # Use FR leg for comparison
        
        x_data = [p[0] for p in trajectory]
        y_data = [p[1] for p in trajectory]
        
        # Plot trajectory
        ax.plot(x_data, y_data, '-', color=params['color'], 
                linewidth=2.5, label=params['description'], alpha=0.7)
        
        # Mark start point
        ax.plot(x_data[0], y_data[0], 'o', color=params['color'], 
                markersize=10, markeredgecolor='black', markeredgewidth=1.5)
    
    ax.axhline(y=DEFAULT_STANCE_HEIGHT, color='gray', linestyle='--', 
               linewidth=1.5, label='Ground Level', alpha=0.5)
    ax.axvline(x=0, color='gray', linestyle='--', linewidth=1.5, alpha=0.5)
    
    ax.set_xlabel('X Position (mm)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (mm)', fontsize=12, fontweight='bold')
    ax.set_title('Gait Comparison - Front Right (FR) Leg Trajectories', 
                 fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Set axis limits
    ax.set_xlim(-60, 60)
    ax.set_ylim(-210, -160)
    ax.set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    
    if save_fig:
        filename = 'foot_path_comparison.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"✅ Saved: {filename}")
    
    plt.show()

def plot_3d_trajectory(gait_type='trot', save_fig=False):
    """
    Plot 3D trajectory with time as third dimension
    """
    params = GAIT_PARAMS[gait_type]
    leg_trajectories = generate_leg_trajectories(gait_type)
    
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    for leg_id, trajectory in leg_trajectories.items():
        x_data = [p[0] for p in trajectory]
        y_data = [p[1] for p in trajectory]
        t_data = list(range(len(trajectory)))
        
        # Plot trajectory
        ax.plot(x_data, t_data, y_data, '-', color=LEG_COLORS[leg_id], 
                linewidth=2, label=f'{leg_id} Leg', alpha=0.7)
        
        # Mark start point
        ax.scatter(x_data[0], t_data[0], y_data[0], color=LEG_COLORS[leg_id], 
                   s=100, edgecolors='black', linewidths=1.5, marker='o')
    
    ax.set_xlabel('X Position (mm)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Time Step', fontsize=11, fontweight='bold')
    ax.set_zlabel('Y Position (mm)', fontsize=11, fontweight='bold')
    ax.set_title(f'3D Foot Trajectories - {params["description"]}', 
                 fontsize=13, fontweight='bold')
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_fig:
        filename = f'foot_path_3d_{gait_type}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"✅ Saved: {filename}")
    
    plt.show()

def plot_phase_diagram(gait_type='trot', save_fig=False):
    """
    Plot phase diagram showing when each leg is in swing vs stance
    """
    params = GAIT_PARAMS[gait_type]
    phase_offsets = GAIT_PHASE_OFFSETS[gait_type]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # Top plot: trajectories
    leg_trajectories = generate_leg_trajectories(gait_type)
    
    for leg_id, trajectory in leg_trajectories.items():
        x_data = [p[0] for p in trajectory]
        y_data = [p[1] for p in trajectory]
        
        ax1.plot(x_data, y_data, '-', color=LEG_COLORS[leg_id], 
                linewidth=2, label=f'{leg_id} Leg', alpha=0.7)
        ax1.plot(x_data[0], y_data[0], 'o', color=LEG_COLORS[leg_id], 
                markersize=10, markeredgecolor='black', markeredgewidth=1.5)
    
    ax1.axhline(y=DEFAULT_STANCE_HEIGHT, color='gray', linestyle='--', 
               linewidth=1, alpha=0.5)
    ax1.set_xlabel('X Position (mm)', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Y Position (mm)', fontsize=11, fontweight='bold')
    ax1.set_title(f'Foot Trajectories - {params["description"]}', 
                 fontsize=12, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal', adjustable='box')
    
    # Bottom plot: phase diagram
    swing_steps = int(params['steps'] * (1.0 - params['stance_ratio']))
    
    leg_order = ['FR', 'FL', 'RR', 'RL']
    for i, leg_id in enumerate(leg_order):
        offset = phase_offsets[leg_id]
        offset_steps = int(offset * params['steps'])
        
        # Swing phase bar
        swing_start = offset_steps
        swing_end = (offset_steps + swing_steps) % params['steps']
        
        if swing_end > swing_start:
            ax2.barh(i, swing_steps, left=swing_start, height=0.6, 
                    color=LEG_COLORS[leg_id], alpha=0.7, 
                    edgecolor='black', linewidth=1.5, label='Swing' if i == 0 else '')
        else:
            # Wrap around
            ax2.barh(i, params['steps'] - swing_start, left=swing_start, height=0.6, 
                    color=LEG_COLORS[leg_id], alpha=0.7, 
                    edgecolor='black', linewidth=1.5)
            ax2.barh(i, swing_end, left=0, height=0.6, 
                    color=LEG_COLORS[leg_id], alpha=0.7, 
                    edgecolor='black', linewidth=1.5)
        
        # Stance phase bar (lighter color)
        stance_steps = params['steps'] - swing_steps
        stance_start = (swing_start + swing_steps) % params['steps']
        
        if stance_start + stance_steps <= params['steps']:
            ax2.barh(i, stance_steps, left=stance_start, height=0.6, 
                    color=LEG_COLORS[leg_id], alpha=0.3, 
                    edgecolor='black', linewidth=1.5, linestyle='--',
                    label='Stance' if i == 0 else '')
        else:
            # Wrap around
            ax2.barh(i, params['steps'] - stance_start, left=stance_start, height=0.6, 
                    color=LEG_COLORS[leg_id], alpha=0.3, 
                    edgecolor='black', linewidth=1.5, linestyle='--')
            ax2.barh(i, (stance_start + stance_steps) - params['steps'], left=0, height=0.6, 
                    color=LEG_COLORS[leg_id], alpha=0.3, 
                    edgecolor='black', linewidth=1.5, linestyle='--')
    
    ax2.set_yticks(range(len(leg_order)))
    ax2.set_yticklabels([f'{leg} Leg' for leg in leg_order])
    ax2.set_xlabel('Time Step', fontsize=11, fontweight='bold')
    ax2.set_title('Phase Diagram (Swing vs Stance)', fontsize=12, fontweight='bold')
    ax2.set_xlim(0, params['steps'])
    ax2.grid(True, axis='x', alpha=0.3)
    ax2.legend(loc='upper right', fontsize=9)
    
    plt.tight_layout()
    
    if save_fig:
        filename = f'foot_path_phase_{gait_type}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"✅ Saved: {filename}")
    
    plt.show()

# ============================================================================
# MAIN MENU
# ============================================================================

def main():
    """Main menu for foot path plotting"""
    print("\n" + "="*70)
    print("  🦿 FOOT TIP PATH VISUALIZATION")
    print("="*70)
    print("  Select visualization type:")
    print("    [1] Single Gait - All 4 Legs (TROT)")
    print("    [2] Single Gait - All 4 Legs (SMOOTH TROT)")
    print("    [3] Single Gait - All 4 Legs (BACKWARD TROT)")
    print("    [4] Single Gait - All 4 Legs (WALK)")
    print("    [5] Single Gait - All 4 Legs (CRAWL)")
    print("    [6] Gait Comparison - FR Leg Only")
    print("    [7] 3D Trajectory - All 4 Legs (TROT)")
    print("    [8] Phase Diagram - TROT")
    print("    [9] Phase Diagram - SMOOTH TROT")
    print("    [0] Generate All Plots")
    print("="*70)
    
    choice = input("  Enter choice [0-9]: ").strip()
    
    save = input("  Save figures? [y/N]: ").strip().lower() == 'y'
    
    if choice == '1':
        plot_single_gait('trot', save_fig=save)
    elif choice == '2':
        plot_single_gait('smooth_trot', save_fig=save)
    elif choice == '3':
        plot_single_gait('backward_trot', save_fig=save)
    elif choice == '4':
        plot_single_gait('walk', save_fig=save)
    elif choice == '5':
        plot_single_gait('crawl', save_fig=save)
    elif choice == '6':
        plot_all_gaits_comparison(save_fig=save)
    elif choice == '7':
        plot_3d_trajectory('trot', save_fig=save)
    elif choice == '8':
        plot_phase_diagram('trot', save_fig=save)
    elif choice == '9':
        plot_phase_diagram('smooth_trot', save_fig=save)
    elif choice == '0':
        print("\n  🎨 Generating all plots...")
        for gait in ['trot', 'smooth_trot', 'backward_trot', 'walk', 'crawl']:
            plot_single_gait(gait, save_fig=save)
            plot_phase_diagram(gait, save_fig=save)
        plot_all_gaits_comparison(save_fig=save)
        plot_3d_trajectory('trot', save_fig=save)
        print("\n  ✅ All plots generated!")
    else:
        print("  ❌ Invalid choice!")

if __name__ == "__main__":
    main()
