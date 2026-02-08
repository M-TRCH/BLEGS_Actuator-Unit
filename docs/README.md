# 📚 BLEGS Actuator Unit - Documentation

Documentation for BLEGS BLDC Motor Control System

---

## 📂 Structure

```
docs/
├── guides/          # Hardware setup & troubleshooting
├── technical/       # Protocol & system architecture
├── results/         # Test results & measurements
├── roadmap/         # Development plans & analysis
└── theory/          # Control theory & algorithms (WIP)
```

---

## 🔍 Quick Reference

### Hardware & Setup
- [Hardware Setup](guides/HARDWARE_SETUP.md) - Pinout, wiring, connections
- [Troubleshooting](guides/TROUBLESHOOTING.md) - Common issues & solutions

### Technical Specifications
- [Binary Protocol](technical/PROTOCOL.md) - Communication protocol specification
- [Gait Control Analysis](technical/GAIT_CONTROL_ANALYSIS.md) - Quadruped control analysis

### Test Results
- [Commutation Tests](results/COMMUTATION_TEST_RESULTS.md) - Motor performance data

### Development Roadmap
- [Motor Stall Fix](roadmap/ROADMAP_MOTOR_STALL_FIX.md) - PID improvements & anti-windup
- [Relative Position Control](roadmap/ROADMAP_RELATIVE_POSITION_CONTROL.md) - Navigation system testing

---

## ⚙️ System Overview

**Hardware:**
- STM32G431CBU6 (170MHz ARM Cortex-M4)
- AS5047P Encoder (14-bit absolute)
- BLDC Motor with FOC/SVPWM
- UART @ 921,600 baud

**Control:**
- Field-Oriented Control (FOC)
- PID Position Control with Anti-windup
- S-Curve Motion Planning
- Multi-turn Tracking

**Communication:**
- Binary Protocol (high-speed)
- CRC-16 integrity checking
- Real-time feedback

---

## ⚡ Quick Commands

```bash
# Build & Upload
pio run --target upload

# Monitor Serial
pio device monitor --baud 921600
```

### ASCII Commands
```
S           # Start motor
M0          # Direct Position mode
M1          # S-Curve mode
#<value>    # Set target position
```

---

## 📖 Theory Documentation

Complete control theory and mathematical modeling:
- **Phase 1:** Kinematics - Five-bar mechanism forward kinematics
- **Phase 2:** Dynamics - Lagrangian dynamics and equations of motion
- **Phase 3:** Simulation - MATLAB/Simulink system modeling
- **Phase 4:** Control - PID and advanced control strategies
- **Phase 5:** Quadruped - Full robot integration

See [theory/README.md](theory/README.md) for LaTeX documentation.

---

**Last Updated:** February 8, 2026  
**Version:** 3.1
