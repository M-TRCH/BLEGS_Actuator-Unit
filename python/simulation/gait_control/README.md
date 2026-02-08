# Phase 3: Gait Control Simulation

## Overview
Phase 3 implements dynamic gait control for the quadruped robot using PyBullet physics simulation. This phase demonstrates a trot gait pattern with balance control.

## Files
- `gait_control_trot.py` - Main simulation script implementing 2-DOF compliant trot gait
- `../../../models/urdf/quadruped/my_robot.urdf` - URDF model of the quadruped robot

## Features

### Gait Pattern
- **Trot Gait**: Diagonal leg pairs move together
  - Pair 1: Front-Right (FR) + Rear-Left (RL)
  - Pair 2: Front-Left (FL) + Rear-Right (RR)
  
### Control System
1. **Inverse Kinematics (IK)**: 
   - 2-DOF per leg (thigh and shank joints)
   - Hip joints are fixed for stability
   
2. **Balance Control**:
   - Pitch correction (front-back balance)
   - Roll correction (left-right balance)
   - PD control with configurable gains

3. **Gait Parameters**:
   - Step Length: 50mm
   - Lift Height: 50mm
   - Step Time: 0.6 seconds
   - Warm-up Time: 2.0 seconds

### Robot Specifications
- **Base**: 490mm × 260mm × 92.5mm
- **Hip Position**:
  - Front (FR/FL): x=198.75mm, y=±153.5mm
  - Rear (RR/RL): x=-160mm, y=±153.5mm
- **Leg Segments**:
  - Thigh: 105mm
  - Shank: 145mm
  - Total leg length: 250mm
- **Standing Height**: 200mm

## Requirements
```
pybullet>=3.2.7
```

## Usage

### Running the Simulation
```powershell
cd scripts/simulation/gait_control
python gait_control_trot.py
```

### Controls
- Press `Ctrl+C` to stop the simulation
- The PyBullet GUI window will open showing the robot in 3D

### Sequence
1. **Warm-up Phase** (0-2 seconds): Robot maintains home position
2. **Walking Phase** (2+ seconds): Robot executes trot gait with balance control

## Tuning Parameters

### Gait Parameters
```python
STEP_LENGTH = 0.05   # Step length in meters
LIFT_HEIGHT = 0.05   # Leg lift height in meters
STEP_TIME = 0.6      # Time for one step cycle in seconds
```

### Balance Control Gains
```python
# Pitch (front-back)
BALANCE_KP_PITCH = 0.006  # Proportional gain
BALANCE_KD_PITCH = 0.012  # Derivative gain

# Roll (left-right)
BALANCE_KP_ROLL = 0.006   # Proportional gain
BALANCE_KD_ROLL = 0.012   # Derivative gain
```

### Motor Control
```python
# Walking mode
forces = [9, 9]                  # Nm
positionGains = [0.3, 0.3]      # Position tracking stiffness
velocityGains = [0.5, 0.5]      # Velocity damping

# Warm-up mode
forces = [10, 10]               # Nm
positionGains = [0.5, 0.5]      # Higher stiffness for stability
velocityGains = [0.7, 0.7]      # Higher damping for stability
```

### IK Parameters
```python
JOINT_DAMPING = 0.5  # Higher values = softer compliance
maxNumIterations=50  # IK solver iterations
```

## Troubleshooting

### KeyError Issues
If you encounter `KeyError` for joint or link names:
1. Check URDF file for special characters (especially non-breaking spaces, ASCII 160)
2. The script automatically handles `.replace(u'\xa0', u' ').strip()` on all joint/link names

### Robot Falls or Unstable
1. Increase `BALANCE_KP_*` and `BALANCE_KD_*` gains
2. Reduce `STEP_LENGTH` or `LIFT_HEIGHT`
3. Increase `STEP_TIME` for slower, more stable gait
4. Increase `positionGains` and `velocityGains`

### Robot Doesn't Move
1. Check if URDF path is correct
2. Verify all joints are properly defined
3. Ensure PyBullet is properly installed

## Next Steps
- Implement different gait patterns (walk, bound, gallop)
- Add terrain adaptation
- Implement obstacle avoidance
- Add sensor feedback (IMU, force sensors)
- Export gait trajectories for real hardware deployment

## Related Files
- Phase 1 Kinematics: `../../kinematics/`
- Phase 2 Dynamics: `../../analysis/Dynamic-Torque-Analysis.py`
- URDF Models: `../../../models/urdf/quadruped/`
