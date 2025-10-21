# RobStride Motor Control API

HTTP/JSON API for controlling RobStride motors. Runs independently from the TUI or alongside it.

## Running the API

```bash
cargo run --release --bin run-rob-api
```

The API server will start on `http://0.0.0.0:8080`

## API Endpoints

### Status & Information

#### `GET /api/status`
Get current motor state including position, velocity, torque, temperature, and faults.

**Response:**
```json
{
  "actuator_id": 1,
  "motor_state": {
    "target_position": 0.0,
    "current_position": 0.0,
    "current_velocity": 0.0,
    "current_torque": 0.0,
    "current_temperature": 25.0,
    "fault_uncalibrated": false,
    "fault_hall_encoding": false,
    "fault_magnetic_encoding": false,
    "fault_over_temperature": false,
    "fault_overcurrent": false,
    "fault_undervoltage": false,
    "increment": 0.01,
    "increment_index": 1,
    "feedback_received": true,
    "command_count": 123,
    "feedback_count": 123,
    "motor_enabled": true,
    "lock_mode": false,
    "available_actuators": [1, 2, 3],
    "current_kp": 150.0,
    "current_kd": 8.0,
    "current_max_torque": 150.0
  }
}
```

#### `GET /api/actuators`
List all available actuators on the CAN bus.

**Response:**
```json
[1, 2, 3]
```

### Motor Control

#### `POST /api/actuator/:id`
Switch to a different actuator.

**Example:**
```bash
curl -X POST http://localhost:8080/api/actuator/2
```

#### `POST /api/position`
Set absolute target position.

**Request Body:**
```json
{
  "position": 1.5708
}
```

**Example:**
```bash
curl -X POST http://localhost:8080/api/position \
  -H "Content-Type: application/json" \
  -d '{"position": 1.5708}'
```

#### `POST /api/position/increment`
Increment position by specified amount.

**Request Body:**
```json
{
  "amount": 0.1
}
```

**Example:**
```bash
curl -X POST http://localhost:8080/api/position/increment \
  -H "Content-Type: application/json" \
  -d '{"amount": 0.1}'
```

#### `POST /api/position/decrement`
Decrement position by specified amount.

**Request Body:**
```json
{
  "amount": 0.1
}
```

**Example:**
```bash
curl -X POST http://localhost:8080/api/position/decrement \
  -H "Content-Type: application/json" \
  -d '{"amount": 0.1}'
```

### Calibration & Modes

#### `POST /api/zero`
Set the motor's current physical position as the new zero point (calibration).

**Example:**
```bash
curl -X POST http://localhost:8080/api/zero
```

#### `POST /api/target-zero`
Command the motor to return to position zero.

**Example:**
```bash
curl -X POST http://localhost:8080/api/target-zero
```

#### `POST /api/lock`
Toggle lock mode (high stiffness).

**Example:**
```bash
curl -X POST http://localhost:8080/api/lock
```

### Motor Enable/Disable

#### `POST /api/enable`
Enable the motor.

**Example:**
```bash
curl -X POST http://localhost:8080/api/enable
```

#### `POST /api/disable`
Disable the motor.

**Example:**
```bash
curl -X POST http://localhost:8080/api/disable
```

## Control Parameters

The API uses the same control presets as the TUI:

- **Normal Mode**: kp=150.0, kd=8.0, max_torque=150.0 Nm
- **Lock Mode**: kp=200.0, kd=10.0, max_torque=200.0 Nm

## Example Usage

### Python Example

```python
import requests

API_URL = "http://localhost:8080"

# Get status
response = requests.get(f"{API_URL}/api/status")
status = response.json()
print(f"Current position: {status['motor_state']['current_position']}")

# Set position to 90 degrees (π/2 radians)
requests.post(f"{API_URL}/api/position", json={"position": 1.5708})

# Increment by 0.1 radians
requests.post(f"{API_URL}/api/position/increment", json={"amount": 0.1})

# Enable lock mode
requests.post(f"{API_URL}/api/lock")
```

### JavaScript Example

```javascript
const API_URL = "http://localhost:8080";

// Get status
const status = await fetch(`${API_URL}/api/status`).then(r => r.json());
console.log(`Current position: ${status.motor_state.current_position}`);

// Set position
await fetch(`${API_URL}/api/position`, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ position: 1.5708 })
});

// Toggle lock mode
await fetch(`${API_URL}/api/lock`, { method: 'POST' });
```

### Bash Example

```bash
# Get status
curl http://localhost:8080/api/status | jq

# Set position to 1 radian
curl -X POST http://localhost:8080/api/position \
  -H "Content-Type: application/json" \
  -d '{"position": 1.0}'

# Increment position
curl -X POST http://localhost:8080/api/position/increment \
  -H "Content-Type: application/json" \
  -d '{"amount": 0.5}'

# Zero the motor
curl -X POST http://localhost:8080/api/zero
```

## CORS

The API has CORS enabled by default, allowing requests from any origin. This makes it easy to use from web applications.

## Logging

Logs are written to stdout. Use `RUST_LOG` environment variable to control verbosity:

```bash
RUST_LOG=debug cargo run --release --bin run-rob-api
```

## Running Both TUI and API

You can run both the TUI and API simultaneously:

```bash
# Terminal 1: Run the API
cargo run --release --bin run-rob-api

# Terminal 2: Run the TUI
cargo run --release --bin run-rob-tui
```

Note: Both will control the same motor(s), so be careful about conflicting commands.
