# RobStride Motor Controller TUI

An interactive terminal user interface (TUI) for controlling RobStride motors with real-time feedback, multi-actuator support, and comprehensive diagnostics.

## Features

- **Multi-Actuator Support**
  - Automatic bus scanning to discover available actuators
  - Auto-connect to first available actuator on startup
  - Hot-swap between actuators using arrow keys
  - Real-time display of all available actuators on the bus

- **Real-time Motor Status Display**
  - Current and target position (in radians and degrees)
  - Position error tracking with color-coded feedback
  - Velocity and torque monitoring
  - Temperature monitoring with color-coded warnings
  - Control configuration display (kp, kd, max_torque)
  - 50ms update rate

- **Comprehensive Fault Monitoring**
  - Uncalibrated motor detection
  - Hall encoding faults
  - Magnetic encoding faults
  - Over-temperature warnings
  - Overcurrent detection
  - Undervoltage detection

- **Configurable Position Increments**
  - 0.001 rad (fine control)
  - 0.01 rad (default)
  - 0.1 rad (coarse control)

- **Advanced Control Modes**
  - Normal mode: Standard position control
  - Lock mode: High stiffness for resisting external forces
  - Motor re-zeroing capability

## Usage

Run the application:

```bash
cargo run --release
```

Or run the compiled binary directly:

```bash
./target/release/run-rob
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| `↑` or `W` | Increase target position by current increment |
| `↓` or `S` | Decrease target position by current increment |
| `←` | Switch to previous actuator in the list |
| `→` | Switch to next actuator in the list |
| `Tab` | Cycle through available increments (0.001, 0.01, 0.1) |
| `0` | Set zero point (calibrate current position as zero) |
| `Z` | Return to zero position |
| `L` | Toggle Lock Mode (high stiffness) |
| `Q` or `Esc` | Quit application (safely disables motor) |

## Display Sections

### Connection & Diagnostics
Real-time connection status and system information:
- **Connection Status**: CONNECTED/DISCONNECTED with color coding
- **Motor Status**: ENABLED/DISABLED
- **Control Mode**: NORMAL/LOCKED
- **Commands Sent**: Total number of commands transmitted
- **Feedback Received**: Total number of feedback messages
- **Last Feedback**: Time since last feedback (color-coded by freshness)
- **Active Actuator**: Currently controlled actuator ID
- **Available Actuators**: List of all actuators found on the bus (active one in brackets)

### Motor Status
Comprehensive real-time motor state:
- **Target Position**: The commanded position (rad and degrees)
- **Current Position**: Actual motor position from feedback (rad and degrees)
- **Position Error**: Difference between target and current (rad and degrees)
- **Velocity**: Current motor velocity (rad/s)
- **Torque**: Current motor torque (Nm)
- **Temperature**: Motor temperature with color-coded warnings (°C)
- **Control Config**: Active control parameters (kp, kd, max_torque)
- **Faults**: Comprehensive fault status display
  - Uncalibrated motor
  - Hall encoding errors
  - Magnetic encoding errors
  - Over-temperature
  - Overcurrent
  - Undervoltage

### Available Increments
Lists all increment options with the active one highlighted in green with an arrow (→).

### Controls
Quick reference for all keyboard commands.

## Logging

Since the TUI uses the terminal display, logs are written to files instead of stdout:
- Logs are stored in `./logs/motor-control.log`
- Daily rotation is enabled
- Use `RUST_LOG` environment variable to control log level:

```bash
RUST_LOG=debug cargo run --release
```

## Motor Configuration

The application automatically scans for available actuators on startup and connects to the first one found. Default configuration:

- **CAN Interface**: can0
- **Actuator Scanning**: IDs 1, 2, 3, 127
- **Normal Mode Control Parameters**:
  - kp: 90.0
  - kd: 5.0
  - max_torque: 100.0 Nm
  - max_velocity: 50.0 rad/s
  - max_current: 90.0 A
- **Lock Mode Control Parameters**:
  - kp: 80.0 (higher stiffness)
  - kd: 2.0 (higher damping)
  - max_torque: 100.0 Nm
  - max_velocity: 50.0 rad/s
  - max_current: 90.0 A

Edit `src/main.rs` to modify these parameters or add more actuator IDs to scan.

## Safety

The application includes proper cleanup:
- Motor is commanded to zero torque on exit
- Motor is disabled before shutdown
- Terminal is properly restored even on errors
- Ctrl+C handling for graceful shutdown

## Requirements

- Rust 1.70 or later
- SocketCAN interface (can0) configured and active
- RobStride motor(s) connected and powered

## CAN Interface Setup

Install required tools:
```bash
sudo apt install can-utils net-tools
```

Setup CAN interface (or use the provided `setup_can.sh` script):
```bash
sudo pkill slcand || true
sudo ip link set can0 down 2>/dev/null || true
sudo slcand -o -s8 -t hw /dev/ttyACM0 can0
sudo ip link set can0 up
```

Verify the interface is up:
```bash
ip -details link show can0
```

Monitor CAN traffic (optional):
```bash
candump -tz -x can0
```

## Tips

- **Multiple Motors**: The app automatically discovers all motors on the bus. Use ← and → to switch between them.
- **Lock Mode**: Use Lock Mode (press `L`) when you need the motor to resist external forces with high stiffness.
- **Calibration**: Press `0` to set the motor's current physical position as the new zero point. This is useful for calibrating the motor's home position.
- **Return to Zero**: Press `Z` to command the motor to move back to the zero position.
- **Diagnostics**: Check the fault indicators if a motor behaves unexpectedly. Temperature and torque readings help identify mechanical issues.
- **Logging**: Check `./logs/motor-control.log.*` for detailed diagnostic information if something goes wrong.
