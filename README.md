# RobStride Motor Controller TUI

An interactive terminal user interface (TUI) for controlling RobStride motors with real-time feedback and configurable position increments.

## Features

- **Real-time Motor Status Display**
  - Current and target position (in radians and degrees)
  - Position error tracking
  - Velocity and torque monitoring
  - 50ms update rate

- **Configurable Position Increments**
  - 0.001 rad (fine control)
  - 0.01 rad (default)
  - 0.1 rad (medium control)
  - 1.0 rad (coarse control)

- **Intuitive Keyboard Controls**
  - Arrow keys or W/S for position adjustment
  - Tab to cycle through increments
  - Quick zero position command

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
| `Tab` | Cycle through available increments (0.001, 0.01, 0.1, 1.0) |
| `0` | Reset target position to zero |
| `Q` or `Esc` | Quit application (safely disables motor) |

## Display Sections

### Motor Status
Shows real-time feedback from the motor:
- **Target Position**: The commanded position
- **Current Position**: Actual motor position from feedback
- **Position Error**: Difference between target and current
- **Velocity**: Current motor velocity
- **Torque**: Current motor torque
- **Current Increment**: Active step size for position adjustments

### Available Increments
Lists all increment options with the active one highlighted in green with an arrow (→).

### Controls
Quick reference for keyboard commands.

## Logging

Since the TUI uses the terminal display, logs are written to files instead of stdout:
- Logs are stored in `./logs/motor-control.log`
- Daily rotation is enabled
- Use `RUST_LOG` environment variable to control log level:

```bash
RUST_LOG=debug cargo run --release
```

## Motor Configuration

The application is configured for:
- **Motor ID**: 1
- **CAN Interface**: can0
- **Control Parameters**:
  - kp: 24.0
  - kd: 0.6
  - max_torque: 40.0 Nm
  - max_velocity: 50.0 rad/s
  - max_current: 10.0 A

Edit `src/main.rs` to modify these parameters.

## Safety

The application includes proper cleanup:
- Motor is commanded to zero torque on exit
- Motor is disabled before shutdown
- Terminal is properly restored even on errors
- Ctrl+C handling for graceful shutdown

## Requirements

- Rust 1.70 or later
- SocketCAN interface (can0) configured and active
- RobStride motor connected and powered

sudo apt install can-utils net-tools

sudo pkill slcand || true
sudo ip link set can0 down 2>/dev/null || true
sudo slcand -o -s8 -t hw /dev/ttyACM0 can0
sudo ip link set can0 up
ip -details link show can0
candump -tz -x can0
