"""
Simulation of Command [7]: Smooth Walk +600mm with March Transitions
====================================================================
PyBullet simulation replicating the behavior of
relative_position_control.py's test_smooth_walk_600() command.

The robot leg mechanism uses NO EF link — Joint E is directly the foot.
Each leg is a simplified 2-DOF serial chain (thigh + shank) in the URDF,
driven by Bezier gait trajectories and PyBullet's IK.

Sequence:
    Phase 0 - WARMUP:      Stand still to settle under gravity (2s)
    Phase 1 - IDLE_MARCH:  March in place, trot phasing (2s)
    Phase 2 - WALKING:     Walk forward 600mm (smooth transition from march)
    Phase 3 - POST_MARCH:  March in place (2s, smooth transition from walking)
    Phase 4 - STAND:       Return to standing position (1s)

Reference architecture (from relative_position_control.py):
    HIGH-LEVEL:  NavigationPlanner  →  velocity command
    MID-LEVEL:   Bezier trajectory  →  foot positions  →  IK  →  joint angles
    LOW-LEVEL:   PyBullet motor control (position PID)
    BALANCE:     PD controller on pitch & roll (body orientation feedback)

Author:  Generated for BLEGS Actuator Unit project
Date:    February 2026
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import os
from enum import IntEnum


# ============================================================================
# SIMULATION PHASES
# ============================================================================

class Phase(IntEnum):
    WARMUP     = 0
    IDLE_MARCH = 1
    WALKING    = 2
    POST_MARCH = 3
    STAND      = 4
    DONE       = 5

LEG_IDS = ['FR', 'FL', 'RR', 'RL']


# ============================================================================
# ROBOT PARAMETERS  (Five-bar linkage, NO EF link)
# ============================================================================

# Linkage dimensions (mm) — for reference / documentation only
# (PyBullet uses the URDF geometry directly for IK)
L_AC = 105.0   # Motor A → joint C
L_BD = 105.0   # Motor B → joint D
L_CE = 145.0   # Joint C → foot E  (no EF extension)
L_DE = 145.0   # Joint D → foot E  (no EF extension)
MOTOR_SPACING = 85.0


# ============================================================================
# GAIT PARAMETERS  (from relative_position_control.py)
# ============================================================================

# Stance
STANCE_HEIGHT_MM   = -220.0   # Foot Y in leg frame (mm, negative = below motors)
STANCE_OFFSET_X_MM =    0.0   # Foot X in leg frame (mm)

# Trajectory
GAIT_LIFT_HEIGHT_MM  =  15.0  # Walking swing lift (mm)
MARCH_LIFT_HEIGHT_MM =  30.0  # March-in-place lift (2× walking)
GAIT_STEP_FORWARD_MM =  30.0  # Maximum step half-excursion (mm)
TRAJECTORY_STEPS     =  30    # Points per gait cycle
STANCE_RATIO         =   0.75 # Fraction of cycle in stance

# Timing
UPDATE_RATE      = 50          # Gait control rate (Hz)
GAIT_CYCLE_TIME  = TRAJECTORY_STEPS / UPDATE_RATE   # 0.6 s
GAIT_DT          = 1.0 / UPDATE_RATE                # 20 ms

# Navigation
NAV_V_MAX_MM_S   = 70.0       # Max walking velocity (mm/s)
NAV_KP           =  1.0       # Proportional gain
NAV_TOL_MM       = 10.0       # Position tolerance (mm)
NAV_TIMEOUT_S    = 60.0       # Walking phase timeout (s)


# ============================================================================
# SIMULATION PARAMETERS
# ============================================================================

SIM_DT           = 1.0 / 240.0   # PyBullet physics timestep
START_HEIGHT_M   = 0.30           # Robot initial Z (m)
WALK_TARGET_MM   = 600.0          # Command [7] target distance

# Phase durations (s)
WARMUP_DUR       = 2.0
MARCH_DUR        = 2.0
POST_MARCH_DUR   = 2.0
STAND_SETTLE_DUR = 1.0


# ============================================================================
# BODY GEOMETRY  (from URDF, in metres)
# ============================================================================

HIP_X_FRONT     =  0.19875       # FR/FL hip X
HIP_X_REAR      = -0.16          # RR/RL hip X
HIP_Y           =  0.1535        # Hip lateral offset (±)
THIGH_OFFSET_Y  =  0.0235        # Thigh joint lateral offset from hip

# Symmetrized X so front/rear step equally
BASE_X_AVG = (HIP_X_FRONT + abs(HIP_X_REAR)) / 2.0   # ≈ 0.1794 m

# Home foot positions in body frame  [X_fwd, Y_side, Z_down]
_sz = STANCE_HEIGHT_MM / 1000.0                        # −0.220 m
_fy_r = -(HIP_Y + THIGH_OFFSET_Y)                     # right-side Y
_fy_l =  (HIP_Y + THIGH_OFFSET_Y)                     # left-side  Y

HOME_FOOT_POS = {
    'FR': [ BASE_X_AVG, _fy_r, _sz],
    'FL': [ BASE_X_AVG, _fy_l, _sz],
    'RR': [-BASE_X_AVG, _fy_r, _sz],
    'RL': [-BASE_X_AVG, _fy_l, _sz],
}


# ============================================================================
# BALANCE PD GAINS  (same as gait_control_trot.py)
# ============================================================================

BAL_KP_PITCH = 0.006
BAL_KD_PITCH = 0.012
BAL_KP_ROLL  = 0.006
BAL_KD_ROLL  = 0.012


# ============================================================================
# JOINT CONTROL GAINS
# ============================================================================

JOINT_DAMPING   = 0.5

POS_GAIN_WALK   = 0.3
VEL_GAIN_WALK   = 0.5
FORCE_WALK      = 9.0    # N·m

POS_GAIN_STAND  = 0.5
VEL_GAIN_STAND  = 0.7
FORCE_STAND     = 10.0   # N·m


# ============================================================================
# DISPLAY
# ============================================================================

STATUS_PRINT_INTERVAL = 1.0   # seconds between status lines during walking
CAMERA_UPDATE_INTERVAL = 0.3  # seconds between camera re-centre


# ============================================================================
# BEZIER TRAJECTORY GENERATION
# (Inlined from bezier_gait.py — no external dependency needed)
# ============================================================================

def _cubic_bezier(t, P0, P1, P2, P3):
    """Cubic Bézier at parameter *t* ∈ [0, 1]."""
    t  = np.clip(t, 0.0, 1.0)
    u  = 1.0 - t
    return u*u*u * P0 + 3*u*u*t * P1 + 3*u*t*t * P2 + t*t*t * P3


def _swing_traj(n, x0, x1, hy, lift, lr=0.4, la=0.6):
    """Swing phase: Bézier curve from (x0, hy) → (x1, hy) with peak lift."""
    P0 = np.array([x0, hy])
    # Control points — 1.25× compensates for Bézier not passing through CPs
    P1 = np.array([x0 + (x1 - x0) * lr * 0.5,
                    hy + lift * 1.25])
    P2 = np.array([x0 + (x1 - x0) * (1.0 - (1.0 - la) * 0.5),
                    hy + lift * 1.25])
    P3 = np.array([x1, hy])
    pts = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0.0
        pt = _cubic_bezier(t, P0, P1, P2, P3)
        pts.append((float(pt[0]), float(pt[1])))
    return pts


def _stance_traj(n, x0, x1, hy):
    """Stance phase: linear interpolation on the ground."""
    pts = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0.0
        pts.append((x0 + (x1 - x0) * t, hy))
    return pts


def generate_gait_trajectory(step_fwd_mm, lift_mm, reverse=False):
    """
    Generate one complete gait cycle (stance → swing) in the **leg frame**.

    Args:
        step_fwd_mm : Half of total foot excursion (mm).
        lift_mm     : Peak foot lift during swing (mm).
        reverse     : If True, walk backward.

    Returns:
        List of (x_mm, y_mm) with length == TRAJECTORY_STEPS.
    """
    n_stance = max(1, int(TRAJECTORY_STEPS * STANCE_RATIO))
    n_swing  = max(1, TRAJECTORY_STEPS - n_stance)

    sgn    = -1.0 if reverse else 1.0
    x_fwd  = STANCE_OFFSET_X_MM + step_fwd_mm * sgn   # front of stride
    x_back = STANCE_OFFSET_X_MM - step_fwd_mm * sgn   # back of stride

    stance = _stance_traj(n_stance, x_fwd, x_back, STANCE_HEIGHT_MM)
    swing  = _swing_traj(n_swing, x_back, x_fwd, STANCE_HEIGHT_MM, lift_mm)
    return stance + swing


# ============================================================================
# TROT PHASING
# ============================================================================

# Diagonal pairs in phase: FR+RL vs FL+RR
TROT_OFFSETS = {'FR': 0.0, 'FL': 0.5, 'RR': 0.5, 'RL': 0.0}


def init_step_indices():
    """Create step-index dict with trot phase offsets."""
    return {leg: int(TROT_OFFSETS[leg] * TRAJECTORY_STEPS) for leg in LEG_IDS}


# ============================================================================
# NAVIGATION PLANNER  (simplified — uses PyBullet ground truth)
# ============================================================================

class NavigationPlanner:
    """P-controller on distance using the simulated body X position."""

    def __init__(self):
        self.origin_x = 0.0
        self.target_mm = 0.0
        self.traveled_mm = 0.0

    def set_target(self, x_now_m, target_mm):
        self.origin_x = x_now_m
        self.target_mm = target_mm
        self.traveled_mm = 0.0

    def update(self, x_now_m):
        self.traveled_mm = (x_now_m - self.origin_x) * 1000.0

    def velocity(self):
        remaining = self.target_mm - self.traveled_mm
        return float(np.clip(NAV_KP * remaining, -NAV_V_MAX_MM_S, NAV_V_MAX_MM_S))

    def is_done(self):
        return abs(self.target_mm - self.traveled_mm) < NAV_TOL_MM


def velocity_to_step(v_mm_s):
    """Convert velocity (mm/s) → (step_mm, is_reverse)."""
    step = min(abs(v_mm_s) * GAIT_CYCLE_TIME, GAIT_STEP_FORWARD_MM)
    return step, (v_mm_s < 0)


# ============================================================================
# PYBULLET SETUP
# ============================================================================

def setup_pybullet():
    """
    Connect PyBullet, load ground plane + robot URDF.

    Returns:
        robot_id, n_joints, foot_link_ids, moveable_joint_ids, ik_index_map
    """
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(SIM_DT)
    p.loadURDF("plane.urdf")

    # --- Camera initial view ---
    p.resetDebugVisualizerCamera(
        cameraDistance=1.0, cameraYaw=50, cameraPitch=-25,
        cameraTargetPosition=[0, 0, 0.15])

    # --- Load robot URDF ---
    script_dir   = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
    urdf_path    = os.path.join(project_root, "python", "models", "urdf",
                                "quadruped", "my_robot.urdf")
    robot = p.loadURDF(urdf_path,
                       [0, 0, START_HEIGHT_M],
                       p.getQuaternionFromEuler([0, 0, 0]))

    # --- Build name → ID dictionaries ---
    # (handle ASCII-160 non-breaking spaces that some URDF editors insert)
    joint_map, link_map = {}, {}
    n_joints = p.getNumJoints(robot)
    for i in range(n_joints):
        info  = p.getJointInfo(robot, i)
        jname = info[1].decode('utf-8').replace('\xa0', ' ').strip()
        lname = info[12].decode('utf-8').replace('\xa0', ' ').strip()
        joint_map[jname] = i
        if lname:
            link_map[lname] = i

    print(f"  Loaded URDF: {n_joints} joints  "
          f"({sum(1 for i in range(n_joints) if p.getJointInfo(robot,i)[2]==p.JOINT_REVOLUTE)} revolute)")

    # --- Revolute (controllable) joints for IK mapping ---
    ctrl_ids  = [i for i in range(n_joints)
                 if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]
    ik_index  = {jid: idx for idx, jid in enumerate(ctrl_ids)}

    # --- Per-leg lookup tables ---
    foot_ids = {}
    move_ids = {}
    for leg in LEG_IDS:
        foot_ids[leg] = link_map[f'{leg}_foot_link']
        move_ids[leg] = [joint_map[f'{leg}_thigh_joint'],
                         joint_map[f'{leg}_shank_joint']]
        print(f"    {leg}  foot_link={foot_ids[leg]}  "
              f"thigh={move_ids[leg][0]}  shank={move_ids[leg][1]}")

    return robot, n_joints, foot_ids, move_ids, ik_index


# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================

def main():
    robot, n_joints, foot_ids, move_ids, ik_index = setup_pybullet()
    nav = NavigationPlanner()

    # ---- Mutable state ----
    phase          = Phase.WARMUP
    phase_timer    = 0.0
    gait_timer     = 0.0
    sim_time       = 0.0
    status_timer   = 0.0
    camera_timer   = 0.0

    step_indices   = init_step_indices()
    foot_targets   = {leg: list(HOME_FOOT_POS[leg]) for leg in LEG_IDS}

    prev_pitch_err = 0.0
    prev_roll_err  = 0.0

    # ---- Banner ----
    print("\n" + "=" * 70)
    print("  SIM: Smooth Walk +600 mm  (March → Walk → March → Stand)")
    print("  Leg mechanism: No EF link  (L_CE = L_DE = 145 mm, foot at E)")
    print("=" * 70)
    print(f"  Gait cycle : {TRAJECTORY_STEPS} steps @ {UPDATE_RATE} Hz  "
          f"= {GAIT_CYCLE_TIME:.2f} s   (stance {STANCE_RATIO*100:.0f}%)")
    print(f"  Stance ht  : {STANCE_HEIGHT_MM} mm   |  Max step : "
          f"±{GAIT_STEP_FORWARD_MM} mm")
    print(f"  Nav v_max  : {NAV_V_MAX_MM_S} mm/s   |  Tolerance: "
          f"{NAV_TOL_MM} mm")
    print("=" * 70)
    for i, (ph, dur) in enumerate([
        ("Warm-up",    WARMUP_DUR),
        ("Idle march", MARCH_DUR),
        (f"Walk +{WALK_TARGET_MM:.0f} mm", None),
        ("Post march", POST_MARCH_DUR),
        ("Stand",      STAND_SETTLE_DUR),
    ]):
        dur_s = f"{dur:.0f} s" if dur else "distance-based"
        print(f"    Phase {i}: {ph:20s}  ({dur_s})")
    print("=" * 70 + "\n")

    # ================================================================
    #  LOOP
    # ================================================================
    try:
        while phase != Phase.DONE:
            sim_time     += SIM_DT
            phase_timer  += SIM_DT
            gait_timer   += SIM_DT
            status_timer += SIM_DT
            camera_timer += SIM_DT

            base_pos, base_orn = p.getBasePositionAndOrientation(robot)
            roll, pitch, yaw   = p.getEulerFromQuaternion(base_orn)

            # ========================================================
            # 1.  PHASE TRANSITIONS
            # ========================================================
            if phase == Phase.WARMUP and phase_timer >= WARMUP_DUR:
                phase, phase_timer = Phase.IDLE_MARCH, 0.0
                print(f"  [{sim_time:6.1f}s] >>> Phase 1: IDLE MARCH")

            elif phase == Phase.IDLE_MARCH and phase_timer >= MARCH_DUR:
                phase, phase_timer = Phase.WALKING, 0.0
                nav.set_target(base_pos[0], WALK_TARGET_MM)
                print(f"  [{sim_time:6.1f}s] >>> Phase 2: WALKING "
                      f"(+{WALK_TARGET_MM:.0f} mm)")

            elif phase == Phase.WALKING:
                nav.update(base_pos[0])
                if nav.is_done():
                    print(f"  [{sim_time:6.1f}s] Target reached  "
                          f"(traveled {nav.traveled_mm:+.1f} mm)")
                    phase, phase_timer = Phase.POST_MARCH, 0.0
                    print(f"  [{sim_time:6.1f}s] >>> Phase 3: POST MARCH")
                elif phase_timer >= NAV_TIMEOUT_S:
                    print(f"  [{sim_time:6.1f}s] TIMEOUT  "
                          f"(traveled {nav.traveled_mm:+.1f} mm)")
                    phase, phase_timer = Phase.POST_MARCH, 0.0
                    print(f"  [{sim_time:6.1f}s] >>> Phase 3: POST MARCH")

            elif phase == Phase.POST_MARCH and phase_timer >= POST_MARCH_DUR:
                phase, phase_timer = Phase.STAND, 0.0
                print(f"  [{sim_time:6.1f}s] >>> Phase 4: STANDING")

            elif phase == Phase.STAND and phase_timer >= STAND_SETTLE_DUR:
                phase = Phase.DONE
                total_x_mm = base_pos[0] * 1000.0
                print(f"\n  [{sim_time:6.1f}s] === SIMULATION COMPLETE ===")
                print(f"  Final body X : {total_x_mm:+.1f} mm")
                print(f"  Total time   : {sim_time:.1f} s")

            # ========================================================
            # 2.  GAIT TARGET UPDATE  (at UPDATE_RATE Hz)
            # ========================================================
            is_gaiting = phase in (Phase.IDLE_MARCH, Phase.WALKING,
                                   Phase.POST_MARCH)

            if is_gaiting and gait_timer >= GAIT_DT:
                gait_timer -= GAIT_DT

                # — Determine step length & lift —
                if phase == Phase.WALKING:
                    v_body    = nav.velocity()
                    step_mm, reverse = velocity_to_step(v_body)
                    lift_mm   = GAIT_LIFT_HEIGHT_MM
                else:                                       # marching
                    step_mm   = 0.0
                    reverse   = False
                    lift_mm   = MARCH_LIFT_HEIGHT_MM

                # — Single trajectory, sampled per-leg at its phase offset —
                traj = generate_gait_trajectory(step_mm, lift_mm, reverse)
                for leg in LEG_IDS:
                    idx         = step_indices[leg] % len(traj)
                    x_mm, y_mm  = traj[idx]
                    home        = HOME_FOOT_POS[leg]
                    foot_targets[leg] = [
                        home[0] + x_mm / 1000.0,    # body X + gait offset
                        home[1],                     # body Y (constant)
                        y_mm / 1000.0,               # body Z from trajectory
                    ]

                # — Advance step indices —
                for leg in LEG_IDS:
                    step_indices[leg] = (step_indices[leg] + 1) % TRAJECTORY_STEPS

            elif not is_gaiting:
                # Standing phases → hold home position
                for leg in LEG_IDS:
                    foot_targets[leg] = list(HOME_FOOT_POS[leg])

            # ========================================================
            # 3.  BALANCE CORRECTION  (PD on pitch & roll)
            # ========================================================
            pitch_err = -pitch
            roll_err  = -roll

            d_pitch = (pitch_err - prev_pitch_err) / SIM_DT
            d_roll  = (roll_err  - prev_roll_err)  / SIM_DT

            pitch_corr = BAL_KP_PITCH * pitch_err + BAL_KD_PITCH * d_pitch
            roll_corr  = BAL_KP_ROLL  * roll_err  + BAL_KD_ROLL  * d_roll

            prev_pitch_err = pitch_err
            prev_roll_err  = roll_err

            # ========================================================
            # 4.  INVERSE KINEMATICS  &  MOTOR CONTROL
            # ========================================================
            is_standing = phase in (Phase.WARMUP, Phase.STAND)
            pg   = POS_GAIN_STAND if is_standing else POS_GAIN_WALK
            vg   = VEL_GAIN_STAND if is_standing else VEL_GAIN_WALK
            frc  = FORCE_STAND    if is_standing else FORCE_WALK

            for leg in LEG_IDS:
                # Apply balance correction to target
                tgt = list(foot_targets[leg])
                tgt[0] += pitch_corr       # pitch  → X
                tgt[1] += roll_corr        # roll   → Y

                # Body-relative → world coordinates
                world_pos, _ = p.multiplyTransforms(
                    base_pos, base_orn, tgt, [0, 0, 0, 1])

                # IK for all controllable joints, extract this leg's
                all_angles = p.calculateInverseKinematics(
                    robot, foot_ids[leg], world_pos,
                    jointDamping=[JOINT_DAMPING] * n_joints,
                    maxNumIterations=50)

                angles = [all_angles[ik_index[j]] for j in move_ids[leg]]

                p.setJointMotorControlArray(
                    robot, move_ids[leg], p.POSITION_CONTROL,
                    targetPositions=angles,
                    forces=[frc, frc],
                    positionGains=[pg, pg],
                    velocityGains=[vg, vg])

            # ========================================================
            # 5.  STATUS PRINT  (during walking, every ~1 s)
            # ========================================================
            if phase == Phase.WALKING and status_timer >= STATUS_PRINT_INTERVAL:
                status_timer = 0.0
                v = nav.velocity()
                sl, _ = velocity_to_step(v)
                print(f"  [{sim_time:6.1f}s] Walk: "
                      f"{nav.traveled_mm:+7.1f} / {WALK_TARGET_MM:.0f} mm  "
                      f"v={v:+5.1f} mm/s  step={sl:4.1f} mm  "
                      f"pitch={np.degrees(pitch):+5.1f}°  "
                      f"roll={np.degrees(roll):+5.1f}°")

            # ========================================================
            # 6.  CAMERA FOLLOW
            # ========================================================
            if camera_timer >= CAMERA_UPDATE_INTERVAL:
                camera_timer = 0.0
                p.resetDebugVisualizerCamera(
                    cameraDistance=1.0, cameraYaw=50, cameraPitch=-25,
                    cameraTargetPosition=[base_pos[0], base_pos[1], 0.15])

            # ========================================================
            # 7.  STEP PHYSICS
            # ========================================================
            p.stepSimulation()
            time.sleep(SIM_DT)

    except KeyboardInterrupt:
        print(f"\n  [{sim_time:.1f}s] Interrupted by user")

    finally:
        print("\n  Disconnecting PyBullet...")
        if p.isConnected():
            p.disconnect()
        print("  Done.")


if __name__ == "__main__":
    main()
