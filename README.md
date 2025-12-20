# BLEGS – BLDC-based Leg Execution & Guidance System

## 📋 Project Description

The BLEGS project focuses on developing a compact and efficient actuation system for robotic leg movement using Brushless DC (BLDC) motors. The system is designed to deliver precise, responsive, and energy-efficient control of leg joints, enabling dynamic motion such as walking, balancing, or directional adjustment.

## ✨ Key Features

### Motor Control
- **SVPWM-based Control** - Sinusoidal Space Vector PWM for smooth operation with minimal torque ripple
- **Field-Oriented Control (FOC)** - Park/Clarke transformations for precise torque control
- **PID Position Control** - Closed-loop position tracking with encoder feedback
- **S-Curve Motion Planning** - Smooth acceleration profiles for natural movement

### Communication
- **Hybrid Protocol** - Supports both ASCII (debug) and Binary (high-speed) communication
- **CRC-16 Integrity** - Error detection for reliable data transmission
- **921,600 baud** - High-speed serial communication
- **Auto Feedback** - Real-time status reporting (position, current, flags)

### Hardware
- **STM32G431CBU6** - 170MHz ARM Cortex-M4 with FPU and DSP
- **AS5047P Encoder** - 14-bit magnetic absolute position sensor
- **Three-Phase Inverter** - MOSFET-based motor driver with current sensing
- **Compact Design** - Optimized for embedded robotics applications

## 🚀 Quick Start

### Build & Upload

```bash
# Using PlatformIO
pio run --target upload

# Or using the full path
C:\Users\mteer\.platformio\penv\Scripts\platformio.exe run --target upload
```

### Serial Communication

#### ASCII Mode (Default for debugging)
```
M1          # Switch to S-Curve mode
#45.0       # Move to 45 degrees
B           # Toggle binary protocol
```

#### Binary Mode (For high-speed control)
```python
# See tools/test_protocol.py
python tools/test_protocol.py
```

## 📂 Project Structure

```
BLEGS_Actuator-Unit/
├── include/           # Header files
│   ├── config.h       # Configuration parameters
│   ├── motor_control.h
│   ├── protocol.h     # Binary protocol definitions
│   ├── encoder.h
│   ├── svpwm.h
│   └── ...
├── src/              # Source files
│   ├── main.cpp      # Main application
│   ├── protocol.cpp  # Protocol implementation
│   ├── motor_control.cpp
│   ├── svpwm.cpp
│   └── ...
├── lib/              # Libraries
│   ├── AS5047P/      # Encoder library
│   └── scurve_profile/
├── docs/             # 📚 Documentation (organized)
│   ├── getting-started/  # Quick start & user guide
│   ├── technical/        # Protocol & test results
│   ├── guides/           # Hardware & troubleshooting
│   └── api-reference/    # C++ & Python API
├── tools/            # Utilities & test scripts
│   └── test_protocol.py
└── platformio.ini    # Build configuration
```

## 📖 Documentation

**⭐ Start here:** [docs/README.md](docs/README.md) - Complete documentation index

### 📂 Documentation Structure

```
docs/
├── README.md                         # ⭐ Main documentation hub
├── getting-started/
│   ├── QUICK_START.md                # 10-minute setup guide
│   └── USER_GUIDE.md                 # Complete user manual
├── technical/
│   ├── PROTOCOL.md                   # Binary Protocol specification
│   └── COMMUTATION_TEST_RESULTS.md   # Motor test results
├── api-reference/
│   └── README.md                     # C++ & Python API
└── guides/
    ├── HARDWARE_SETUP.md             # Hardware wiring & pinout
    └── TROUBLESHOOTING.md            # Problem solving guide
```

### 📚 Documentation by Category

#### 🚀 Getting Started
- [Quick Start Guide](docs/getting-started/QUICK_START.md) - Get started in 10 minutes
- [User Guide](docs/getting-started/USER_GUIDE.md) - Complete usage manual

#### 🔧 Hardware & Setup
- [Hardware Setup](docs/guides/HARDWARE_SETUP.md) - Pinout, wiring, connections
- [Troubleshooting](docs/guides/TROUBLESHOOTING.md) - Fix common issues

#### 📡 Protocol & Technical
- [Binary Protocol](docs/technical/PROTOCOL.md) - Protocol specification
- [ComTechnical Specifications

### Performance Metrics
- **Control Loop**: 5 kHz position control
- **PWM Frequency**: 10 kHz SVPWM (configurable)
- **Communication**: 264 μs round-trip (binary protocol)
- **Position Accuracy**: < 0.1° with encoder feedback

### Hardware Specs
- **MCU**: STM32G431CBU6 (170MHz ARM Cortex-M4, 32KB RAM, 128KB Flash)
- **Encoder**: AS5047P (14-bit magnetic absolute position)
- **Serial**: 921,600 baud, 8-N-1
- **Gear Ratio**: 8:1
- **Motor**: BLDC with 14 pole pairs

#### ASCII Commands
```
S           # Start motor
M0          # Direct position control mode
M1          # S-Curve motion planning mode
#<value>    # Set target position (degrees)
B           # Toggle binary protocol ON/OFF
```

#### Build Commands
```bash
pio run                    # Build firmware
pio run --target upload    # Build and upload to board
```

## 🛠️ Development

### Requirements
- PlatformIO (or Arduino IDE with STM32 support)
- Python 3.6+ with pyserial (for testing)
- USB-to-Serial adapter (921,600 baud capable)

### Testing
```bash
# Install dependencies
pip install pyserial

# Run protocol test
python tools/test_protocol.py
```

## 🎯 Project Goals

1. Build a stable BLDC motor control module for joint actuation
2. Implement real-time control algorithms (SVPWM, FOC, PID)
3. Achieve smooth, accurate leg movement with minimal latency
4. Ensure modularity for future integration with full robotic platforms

## 📊 Performance Metrics

- **Control Loop**: 5 kHz position control
- **PWM Frequency**: 10 kHz (configurable)
- **Communication**: 264 μs round-trip (binary protocol)
- **Position Accuracy**: < 0.1° with encoder feedback

## 🔮 Future Enhancements

- Torque/Velocity control modes
- Multi-motor synchronization
- Runtime PID parameter tuning
- Advanced trajectory planning (quintic polynomials)
- CAN bus support for distributed systems

## 📝 License

[Add your license information here]

## 👥 Contributors

- M-TRCH

## 📞 Support

For issues, questions, or contributions, please refer to the documentation in the `docs/` folder or contact the project maintainers.

---

**Build Status**: ✅ Tested on STM32G431CBU6  
**Last Updated**: December 3, 2025
