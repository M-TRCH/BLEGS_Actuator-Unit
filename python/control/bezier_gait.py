"""
Bezier Curve-based Gait Trajectory Generator
Author: M-TRCH
Date: February 13, 2026

This module provides smooth gait trajectory generation using Cubic Bezier curves
for quadruped robot locomotion. Replaces elliptical trajectory with smoother,
more natural motion profiles.

Features:
    - Cubic Bezier curves for swing phase (smooth lift and landing)
    - Linear interpolation for stance phase (stable ground contact)
    - Configurable control points for motion customization
    - Compatible with existing gait controller interface
"""

import numpy as np
from typing import List, Tuple


def cubic_bezier(t: float, P0: np.ndarray, P1: np.ndarray, 
                 P2: np.ndarray, P3: np.ndarray) -> np.ndarray:
    """
    Compute point on cubic Bezier curve at parameter t.
    
    Cubic Bezier formula:
        B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
    
    Args:
        t: Parameter in [0, 1]
        P0: Start point (x, y)
        P1: First control point
        P2: Second control point
        P3: End point
    
    Returns:
        Point (x, y) on the curve
    """
    t = np.clip(t, 0.0, 1.0)
    
    # Bezier basis functions
    B0 = (1 - t) ** 3
    B1 = 3 * (1 - t) ** 2 * t
    B2 = 3 * (1 - t) * t ** 2
    B3 = t ** 3
    
    # Compute point
    point = B0 * P0 + B1 * P1 + B2 * P2 + B3 * P3
    
    return point


def generate_bezier_swing_trajectory(num_steps: int,
                                     start_x: float, end_x: float,
                                     home_y: float, lift_height: float,
                                     lift_ratio: float = 0.4,
                                     land_ratio: float = 0.6) -> List[Tuple[float, float]]:
    """
    Generate smooth swing phase trajectory using cubic Bezier curve.
    
    The swing phase lifts the foot smoothly, moves it forward, and lands it gently.
    Control points are optimized for natural-looking leg motion:
    
    - P0: Start position (back, on ground)
    - P1: Lift control point (slightly forward, lifted)
    - P2: Landing control point (forward, lifted)
    - P3: End position (front, on ground)
    
    Args:
        num_steps: Number of trajectory points
        start_x: Starting X position (mm)
        end_x: Ending X position (mm)
        home_y: Ground level Y position (mm, negative)
        lift_height: Maximum foot lift height (mm)
        lift_ratio: Fraction of swing for lifting (default 0.4)
        land_ratio: Fraction of swing for landing (default 0.6)
    
    Returns:
        List of (x, y) positions
    """
    trajectory = []
    
    # Define control points for smooth swing
    P0 = np.array([start_x, home_y])  # Start on ground
    
    # P1: Control point for lift phase - positioned for smooth upward acceleration
    # Slightly forward from start, at elevated height
    # NOTE: Using 1.25x because Bezier curve doesn't pass through control points,
    #       it's pulled toward them. This gives actual peak ≈ 100% of lift_height
    P1 = np.array([
        start_x + (end_x - start_x) * lift_ratio * 0.5,
        home_y + lift_height * 1.25
    ])
    
    # P2: Control point for landing phase - positioned for smooth downward deceleration
    # Slightly before end, at elevated height
    P2 = np.array([
        start_x + (end_x - start_x) * (1.0 - (1.0 - land_ratio) * 0.5),
        home_y + lift_height * 1.25
    ])
    
    P3 = np.array([end_x, home_y])  # End on ground
    
    # Generate trajectory points
    for i in range(num_steps):
        t = i / (num_steps - 1) if num_steps > 1 else 0.0
        point = cubic_bezier(t, P0, P1, P2, P3)
        trajectory.append((point[0], point[1]))
    
    return trajectory


def generate_bezier_stance_trajectory(num_steps: int,
                                      start_x: float, end_x: float,
                                      home_y: float) -> List[Tuple[float, float]]:
    """
    Generate linear stance phase trajectory (foot on ground).
    
    During stance, the foot remains on the ground and moves linearly
    backward relative to the body (pushing the body forward).
    
    Args:
        num_steps: Number of trajectory points
        start_x: Starting X position (mm)
        end_x: Ending X position (mm)
        home_y: Ground level Y position (mm, negative)
    
    Returns:
        List of (x, y) positions
    """
    trajectory = []
    
    for i in range(num_steps):
        if num_steps > 1:
            t = i / (num_steps - 1)
        else:
            t = 0.0
        
        # Linear interpolation
        x = start_x + (end_x - start_x) * t
        y = home_y  # Stay on ground
        
        trajectory.append((x, y))
    
    return trajectory


def generate_bezier_trajectory(num_steps: int = 30,
                               lift_height: float = 15.0,
                               step_forward: float = 30.0,
                               mirror_x: bool = False,
                               stance_ratio: float = 0.7,
                               home_x: float = 0.0,
                               home_y: float = -220.0,
                               reverse: bool = False,
                               lift_ratio: float = 0.4,
                               land_ratio: float = 0.6) -> List[Tuple[float, float]]:
    """
    Generate complete gait cycle trajectory using Bezier curves.
    
    The gait cycle consists of:
    - Stance phase (stance_ratio): Linear motion, foot on ground
    - Swing phase (1 - stance_ratio): Bezier curve, foot lifted
    
    Coordinate system:
    - X: Forward/backward (positive = foot forward relative to body)
    - Y: Up/down (negative = below motor plane)
    
    Args:
        num_steps: Total number of steps per gait cycle
        lift_height: Maximum foot lift during swing (mm)
        step_forward: Step length - foot excursion from center (mm)
        mirror_x: If True, mirror trajectory horizontally (for right-side legs)
        stance_ratio: Fraction of cycle in stance phase (0.7 = 70% stance)
        home_x: Center X position (mm)
        home_y: Ground level Y position (mm, negative)
        reverse: If True, generate backward walking trajectory
        lift_ratio: Bezier control - fraction for lift phase (0.4 = 40%)
        land_ratio: Bezier control - fraction for landing phase (0.6 = 60%)
    
    Returns:
        List of (x, y) positions for complete gait cycle
    """
    # Calculate number of steps for each phase
    stance_steps = max(1, int(num_steps * stance_ratio))
    swing_steps = max(1, num_steps - stance_steps)
    
    # Define trajectory limits
    # During stance: foot moves from +step_forward to -step_forward (backward relative to body)
    # During swing: foot moves from -step_forward to +step_forward (forward in air)
    step_sign = -1.0 if reverse else 1.0
    
    stance_start_x = home_x + step_forward * step_sign
    stance_end_x = home_x - step_forward * step_sign
    
    swing_start_x = stance_end_x
    swing_end_x = stance_start_x
    
    # Apply X-axis mirroring for right-side legs
    if mirror_x:
        stance_start_x = -stance_start_x + 2 * home_x
        stance_end_x = -stance_end_x + 2 * home_x
        swing_start_x = -swing_start_x + 2 * home_x
        swing_end_x = -swing_end_x + 2 * home_x
    
    # Generate stance phase (linear, on ground)
    stance_traj = generate_bezier_stance_trajectory(
        stance_steps,
        stance_start_x,
        stance_end_x,
        home_y
    )
    
    # Generate swing phase (Bezier curve, lifted)
    swing_traj = generate_bezier_swing_trajectory(
        swing_steps,
        swing_start_x,
        swing_end_x,
        home_y,
        lift_height,
        lift_ratio,
        land_ratio
    )
    
    # Combine: stance first, then swing
    trajectory = stance_traj + swing_traj
    
    return trajectory


def visualize_trajectory(trajectory: List[Tuple[float, float]], 
                        title: str = "Bezier Gait Trajectory") -> None:
    """
    Visualize trajectory using matplotlib (for debugging/tuning).
    
    Args:
        trajectory: List of (x, y) positions
        title: Plot title
    """
    try:
        import matplotlib.pyplot as plt
        
        if not trajectory:
            print("Empty trajectory - nothing to plot")
            return
        
        x_vals = [p[0] for p in trajectory]
        y_vals = [p[1] for p in trajectory]
        
        plt.figure(figsize=(10, 6))
        plt.plot(x_vals, y_vals, 'b-', linewidth=2, label='Trajectory')
        plt.plot(x_vals[0], y_vals[0], 'go', markersize=10, label='Start')
        plt.plot(x_vals[-1], y_vals[-1], 'ro', markersize=10, label='End')
        plt.grid(True, alpha=0.3)
        plt.xlabel('X Position (mm)')
        plt.ylabel('Y Position (mm)')
        plt.title(title)
        plt.legend()
        plt.axis('equal')
        plt.tight_layout()
        plt.show()
        
    except ImportError:
        print("Matplotlib not available - skipping visualization")


# ============================================================================
# TESTING & VALIDATION
# ============================================================================

if __name__ == "__main__":
    """Test and visualize Bezier trajectories."""
    
    print("="*70)
    print("  Bezier Gait Trajectory Generator - Test Suite")
    print("="*70)
    
    # Test 1: Forward walking trajectory
    print("\n[Test 1] Forward Walking Trajectory")
    print("-" * 50)
    traj_forward = generate_bezier_trajectory(
        num_steps=30,
        lift_height=15.0,
        step_forward=30.0,
        mirror_x=False,
        stance_ratio=0.7,
        home_x=0.0,
        home_y=-220.0,
        reverse=False
    )
    print(f"  Generated {len(traj_forward)} points")
    print(f"  Start: ({traj_forward[0][0]:.1f}, {traj_forward[0][1]:.1f})")
    print(f"  End: ({traj_forward[-1][0]:.1f}, {traj_forward[-1][1]:.1f})")
    
    # Check stance phase (should be on ground)
    stance_points = int(30 * 0.7)
    ground_level = -220.0
    stance_on_ground = all(abs(p[1] - ground_level) < 0.1 for p in traj_forward[:stance_points])
    print(f"  Stance on ground: {'✅ Pass' if stance_on_ground else '❌ Fail'}")
    
    # Check swing phase (should lift)
    max_height = max(p[1] for p in traj_forward[stance_points:])
    expected_max = ground_level + 15.0
    height_ok = abs(max_height - expected_max) < 2.0
    print(f"  Max swing height: {max_height:.1f} mm (expected ~{expected_max:.1f}) {'✅' if height_ok else '❌'}")
    
    # Test 2: Backward walking trajectory
    print("\n[Test 2] Backward Walking Trajectory")
    print("-" * 50)
    traj_backward = generate_bezier_trajectory(
        num_steps=30,
        lift_height=15.0,
        step_forward=30.0,
        reverse=True
    )
    print(f"  Generated {len(traj_backward)} points")
    print(f"  Direction: Backward (reverse=True)")
    
    # Test 3: Mirrored trajectory (right leg)
    print("\n[Test 3] Mirrored Trajectory (Right Leg)")
    print("-" * 50)
    traj_left = generate_bezier_trajectory(num_steps=30, mirror_x=False)
    traj_right = generate_bezier_trajectory(num_steps=30, mirror_x=True)
    
    # Check if properly mirrored
    x_sum = traj_left[0][0] + traj_right[0][0]
    mirrored_ok = abs(x_sum) < 0.1  # Should sum to 0 if mirrored around home_x=0
    print(f"  Left start X: {traj_left[0][0]:.1f} mm")
    print(f"  Right start X: {traj_right[0][0]:.1f} mm")
    print(f"  Mirroring: {'✅ Pass' if mirrored_ok else '❌ Fail'}")
    
    # Test 4: Idle march (step_forward = 0)
    print("\n[Test 4] Idle March (Stepping in Place)")
    print("-" * 50)
    traj_march = generate_bezier_trajectory(
        num_steps=30,
        lift_height=30.0,  # Higher for visibility
        step_forward=0.0,  # No forward movement
    )
    print(f"  Generated {len(traj_march)} points")
    
    # Check that X doesn't change much
    x_range = max(p[0] for p in traj_march) - min(p[0] for p in traj_march)
    march_ok = x_range < 1.0  # Should be minimal X movement
    print(f"  X range: {x_range:.2f} mm {'✅ Pass' if march_ok else '❌ Fail'}")
    print(f"  Max lift: {max(p[1] for p in traj_march):.1f} mm")
    
    # Test 5: Smoothness check
    print("\n[Test 5] Trajectory Smoothness")
    print("-" * 50)
    
    # Calculate velocity changes (acceleration proxy)
    def check_smoothness(traj, name):
        velocities = []
        for i in range(len(traj) - 1):
            dx = traj[i+1][0] - traj[i][0]
            dy = traj[i+1][1] - traj[i][1]
            v = np.sqrt(dx**2 + dy**2)
            velocities.append(v)
        
        if velocities:
            avg_v = np.mean(velocities)
            std_v = np.std(velocities)
            smoothness = std_v / avg_v if avg_v > 0 else 0
            
            print(f"  {name}:")
            print(f"    Mean velocity: {avg_v:.2f} mm/step")
            print(f"    Std deviation: {std_v:.2f}")
            print(f"    Smoothness ratio: {smoothness:.3f} (lower = smoother)")
            
            return smoothness < 0.5  # Arbitrary threshold
        return False
    
    smooth_ok = check_smoothness(traj_forward, "Forward trajectory")
    
    # Summary
    print("\n" + "="*70)
    print("  TEST SUMMARY")
    print("="*70)
    all_tests = [
        ("Stance on ground", stance_on_ground),
        ("Swing height correct", height_ok),
        ("Mirroring correct", mirrored_ok),
        ("Idle march correct", march_ok),
        ("Trajectory smooth", smooth_ok)
    ]
    
    for name, result in all_tests:
        icon = "✅" if result else "❌"
        print(f"  {icon} {name}")
    
    passed = sum(1 for _, r in all_tests if r)
    print(f"\n  Results: {passed}/{len(all_tests)} tests passed")
    print("="*70)
    
    # Optional: Visualize if matplotlib available
    print("\n[Visualization] Attempting to plot trajectories...")
    try:
        import matplotlib.pyplot as plt
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Bezier Gait Trajectory Comparison', fontsize=16)
        
        # Plot 1: Forward
        ax = axes[0, 0]
        x = [p[0] for p in traj_forward]
        y = [p[1] for p in traj_forward]
        ax.plot(x, y, 'b-', linewidth=2)
        ax.plot(x[0], y[0], 'go', markersize=10)
        ax.plot(x[-1], y[-1], 'ro', markersize=10)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_title('Forward Walking')
        ax.axis('equal')
        
        # Plot 2: Backward
        ax = axes[0, 1]
        x = [p[0] for p in traj_backward]
        y = [p[1] for p in traj_backward]
        ax.plot(x, y, 'r-', linewidth=2)
        ax.plot(x[0], y[0], 'go', markersize=10)
        ax.plot(x[-1], y[-1], 'ro', markersize=10)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_title('Backward Walking')
        ax.axis('equal')
        
        # Plot 3: Left vs Right (mirrored)
        ax = axes[1, 0]
        x_l = [p[0] for p in traj_left]
        y_l = [p[1] for p in traj_left]
        x_r = [p[0] for p in traj_right]
        y_r = [p[1] for p in traj_right]
        ax.plot(x_l, y_l, 'b-', linewidth=2, label='Left leg')
        ax.plot(x_r, y_r, 'r-', linewidth=2, label='Right leg (mirrored)')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_title('Left vs Right Leg')
        ax.legend()
        ax.axis('equal')
        
        # Plot 4: Idle march
        ax = axes[1, 1]
        x = [p[0] for p in traj_march]
        y = [p[1] for p in traj_march]
        ax.plot(x, y, 'g-', linewidth=2)
        ax.plot(x[0], y[0], 'go', markersize=10)
        ax.plot(x[-1], y[-1], 'ro', markersize=10)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_title('Idle March (In Place)')
        ax.axis('equal')
        
        plt.tight_layout()
        plt.show()
        
        print("  ✅ Visualization displayed")
        
    except ImportError:
        print("  ⚠️  Matplotlib not available - skipping visualization")
    except Exception as e:
        print(f"  ⚠️  Visualization error: {e}")
    
    print("\n✅ All tests completed!")
