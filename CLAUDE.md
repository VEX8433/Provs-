# Generation 1 - VEX Robotics V5 Firmware

## Project Overview

This is a **VEX Robotics EDR V5 embedded firmware project** for a 2026 PushBack robot competition. The project controls a 6-motor drivetrain with intake mechanisms and pneumatic systems.

**Target Hardware**: VEX EDR V5 Brain (ARM Cortex-A9)
**Language**: C++20 (gnu++20)
**Build System**: GNU Make with PROS toolchain
**Toolchain**: arm-none-eabi-g++ (GCC 13.3.1)

---

## Directory Structure

```
Generation-1/
├── src/                          # Source code
│   ├── main.cpp                  # Entry point and competition modes
│   └── SubSystems/               # Robot subsystem implementations
│       ├── Chassis.cpp/hpp       # 6-motor drivetrain control
│       ├── Intake.cpp/hpp        # Intake mechanism + pneumatics
│       └── Calculations/
│           └── PID.cpp/hpp       # PID controller implementation
│
├── include/                      # Header files
│   ├── main.h                    # Main header with global declarations
│   ├── api.h                     # PROS API aggregation
│   ├── pros/                     # PROS kernel headers
│   ├── lemlib/                   # LemLib robotics library headers
│   └── liblvgl/                  # LVGL GUI library headers
│
├── firmware/                     # Pre-compiled libraries
│   ├── LemLib.a                  # LemLib compiled library
│   ├── libpros.a                 # PROS kernel library
│   ├── liblvgl.a                 # LVGL graphics library
│   └── *.ld                      # Linker scripts
│
├── bin/                          # Build output (hot/cold packages)
├── static/                       # Static assets
├── Makefile                      # Build configuration
├── common.mk                     # Shared build rules
└── project.pros                  # PROS project configuration
```

---

## Libraries & Dependencies

### PROS Kernel (v4.1.0)
Core VEX robotics API providing:
- Motor control (PWM velocity/voltage modes)
- Sensor interfaces (IMU, distance, rotation, vision)
- Controller input handling
- Pneumatic solenoid control (ADI ports)
- RTOS and threading

**Key Headers**:
- `pros/motors.hpp` - Motor control
- `pros/imu.hpp` - Inertial Measurement Unit
- `pros/rotation.h` - Rotation tracking sensors
- `pros/distance.h` - Ultrasonic distance sensors
- `pros/adi.hpp` - Analog/Digital I/O (pneumatics)
- `pros/controller.hpp` - Game controller

### LemLib (v0.5.4)
Advanced robotics library for:
- Chassis control with odometry
- PID controllers for motion
- Drive curves (exponential response mapping)
- Exit conditions for autonomous movement
- Tracking wheel integration

### liblvgl (v8.3.8)
Lightweight graphics library for V5 brain display (currently unused in code).

### fmt Library
Modern C++ string formatting (bundled with PROS).

---

## Architecture

### Subsystem Pattern
The project uses **object-oriented subsystem architecture**:

```
Chassis class
├── Manages 6 drive motors (3 per side)
├── Controls IMU and tracking sensors
├── inlineTelOp() - Joystick differential drive
└── brake() - Emergency stop

Intake class
├── Manages 2 intake motors
├── Controls 3 pneumatic solenoids
├── Uses distance sensor for feedback
└── telOP() - Button-based control states

PID class
├── Proportional-Integral-Derivative control loop
├── Configurable gains (kP, kI, kD)
└── Integral windup protection
```

### Competition Lifecycle
```cpp
initialize()              // Robot boot
disabled()                // Between matches
competition_initialize()  // Pre-match setup
autonomous()              // 15-sec autonomous period
opcontrol()               // Driver control period (1:45)
```

---

## Hardware Configuration

### Drive Motors (6 total)
| Port | Motor       | Gear Ratio | Reversed |
|------|-------------|------------|----------|
| 2    | LEFT_FRONT  | Blue (600) | No       |
| 3    | LEFT_MIDDLE | Blue (600) | Yes      |
| 4    | LEFT_BACK   | Blue (600) | Yes      |
| 6    | RIGHT_FRONT | Blue (600) | No       |
| 7    | RIGHT_MIDDLE| Blue (600) | No       |
| 9    | RIGHT_BACK  | Blue (600) | Yes      |

### Intake Motors (2 total)
| Port | Motor | Gear Ratio | Reversed |
|------|-------|------------|----------|
| 1    | left  | Blue (600) | No       |
| 10   | right | Blue (600) | Yes      |

### Sensors
| Port | Sensor   | Purpose           |
|------|----------|-------------------|
| 0    | IMU      | Orientation       |
| 0    | Distance | Collision detect  |
| 0    | Rotation | Odometry tracking |

### Pneumatics (ADI Ports)
| Port | Name        | Function              |
|------|-------------|-----------------------|
| A    | bot         | Bottom intake piston  |
| B    | top         | Top intake piston     |
| C    | doinker     | Doinker mechanism     |
| D    | tongue      | Tongue mechanism      |
| E    | backDoinker | Rear doinker          |
| F    | doublePark  | Double park mechanism |

---

## Build System

### Key Commands
```bash
# Build the project
make

# Clean build artifacts
make clean

# Upload to robot (requires PROS CLI)
pros upload
```

### Build Configuration
- **Architecture**: ARM Cortex-A9
- **C++ Standard**: gnu++20
- **Optimization**: -Os (size optimized)
- **Hot/Cold Linking**: Enabled for faster uploads

---

## Control Scheme (TeleOp)

### Joystick
- **Left Y**: Forward/backward drive
- **Right X**: Left/right turn

### Buttons
| Button | Action                    |
|--------|---------------------------|
| R1     | Intake (motors + pistons) |
| L1     | Score top                 |
| R2     | Score mid                 |
| X      | Outtake (reverse)         |
| L2     | Toggle doinker            |
| Y      | Toggle tongue             |
| B      | Toggle back doinker       |
| A      | Toggle double park        |

---

## Code Conventions

### File Organization
- Headers use `#pragma once` for include guards
- Subsystem classes separate interface (.hpp) from implementation (.cpp)
- Hardware ports defined as constants in main.cpp

### Motion Control
- Exponential response curves for smooth joystick input
- Formula: `exp(-(t/10)) + exp((abs(input)-127)/10) * (1 - exp(-(t/10))) * input * scale`

### Naming
- PascalCase for class names
- camelCase for methods and variables
- UPPER_SNAKE_CASE for motor/hardware constants

---

## Known Issues

1. **Intake Constructor Bug** (`src/SubSystems/Intake.cpp:3`):
   ```cpp
   // Bug: Both motors initialized to RIGHT parameter
   Intake::Intake(...) : left(RIGHT), right(RIGHT), ...
   // Should be: left(LEFT), right(RIGHT)
   ```

2. **Empty Autonomous**: No autonomous routines implemented yet

3. **No Test Framework**: Manual testing only

---

## Development Notes

### Adding New Subsystems
1. Create `NewSystem.hpp` and `NewSystem.cpp` in `src/SubSystems/`
2. Declare hardware in `src/main.cpp`
3. Instantiate subsystem object
4. Call methods from `opcontrol()` or `autonomous()`

### Implementing Autonomous
Use LemLib's chassis control with odometry:
```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);  // Set starting position
    chassis.moveToPose(24, 24, 90, 4000);  // Move to point
    // ... more movements
}
```

### PID Tuning
Adjust gains in PID constructor:
```cpp
PID controller(kP, kI, kD, integralThreshold);
// Start with P only, add D, then I if needed
```

---

## Resources

- [PROS Documentation](https://pros.cs.purdue.edu/v5/)
- [LemLib Documentation](https://lemlib.readthedocs.io/)
- [VEX V5 API Reference](https://pros.cs.purdue.edu/v5/api/cpp/index.html)
