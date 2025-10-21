use eyre::Result;
use robstride::{
    ActuatorConfiguration, ActuatorType, ControlConfig, Supervisor,
};
use serde::{Deserialize, Serialize};
use std::time::Instant;
use tracing::info;

/// Shared motor state that can be used by both TUI and API
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorState {
    pub target_position: f32,
    pub current_position: f32,
    pub current_velocity: f32,
    pub current_torque: f32,
    pub current_temperature: f32,
    pub fault_uncalibrated: bool,
    pub fault_hall_encoding: bool,
    pub fault_magnetic_encoding: bool,
    pub fault_over_temperature: bool,
    pub fault_overcurrent: bool,
    pub fault_undervoltage: bool,
    pub increment: f32,
    pub increment_index: usize,
    pub feedback_received: bool,
    #[serde(skip)]
    pub last_feedback_time: Option<Instant>,
    pub command_count: u64,
    pub feedback_count: u64,
    pub motor_enabled: bool,
    pub lock_mode: bool,
    pub available_actuators: Vec<u8>,
    #[serde(skip)]
    pub last_scan_time: Option<Instant>,
    pub current_kp: f32,
    pub current_kd: f32,
    pub current_max_torque: f32,
}

impl MotorState {
    pub fn new() -> Self {
        Self {
            target_position: 0.0,
            current_position: 0.0,
            current_velocity: 0.0,
            current_torque: 0.0,
            current_temperature: 0.0,
            fault_uncalibrated: false,
            fault_hall_encoding: false,
            fault_magnetic_encoding: false,
            fault_over_temperature: false,
            fault_overcurrent: false,
            fault_undervoltage: false,
            increment: 0.01,
            increment_index: 1,
            feedback_received: false,
            last_feedback_time: None,
            command_count: 0,
            feedback_count: 0,
            motor_enabled: false,
            lock_mode: false,
            available_actuators: Vec::new(),
            last_scan_time: None,
            current_kp: 0.0,
            current_kd: 0.0,
            current_max_torque: 0.0,
        }
    }

    pub fn get_increments() -> [f32; 3] {
        [0.001, 0.01, 0.1]
    }

    pub fn cycle_increment(&mut self) {
        let increments = Self::get_increments();
        self.increment_index = (self.increment_index + 1) % increments.len();
        self.increment = increments[self.increment_index];
    }

    pub fn increase_position(&mut self) {
        self.target_position += self.increment;
    }

    pub fn decrease_position(&mut self) {
        self.target_position -= self.increment;
    }

    pub fn set_target_zero(&mut self) {
        self.target_position = 0.0;
    }

    pub fn toggle_lock_mode(&mut self) {
        self.lock_mode = !self.lock_mode;
    }
}

impl Default for MotorState {
    fn default() -> Self {
        Self::new()
    }
}

/// Control configuration presets
pub struct ControlPresets;

impl ControlPresets {
    /// High stiffness configuration for normal operation
    pub fn normal_mode() -> ControlConfig {
        ControlConfig {
            kp: 150.0,
            kd: 8.0,
            max_torque: Some(150.0),
            max_velocity: Some(80.0),
            max_current: Some(100.0),
        }
    }

    /// Maximum stiffness configuration for lock mode
    pub fn lock_mode() -> ControlConfig {
        ControlConfig {
            kp: 200.0,
            kd: 10.0,
            max_torque: Some(200.0),
            max_velocity: Some(100.0),
            max_current: Some(100.0),
        }
    }
}

/// Scan for available actuators on the CAN bus
pub async fn scan_actuators(supervisor: &mut Supervisor) -> Result<Vec<u8>> {
    let actuator_configs = vec![
        (1, ActuatorConfiguration {
            actuator_type: ActuatorType::RobStride00,
            ..Default::default()
        }),
        (2, ActuatorConfiguration {
            actuator_type: ActuatorType::RobStride00,
            ..Default::default()
        }),
        (3, ActuatorConfiguration {
            actuator_type: ActuatorType::RobStride00,
            ..Default::default()
        }),
        (127, ActuatorConfiguration {
            actuator_type: ActuatorType::RobStride00,
            ..Default::default()
        }),
    ];

    let actuators = supervisor.scan_bus(0xFE, "socketcan", &actuator_configs).await?;
    info!("Found {} actuators: {:?}", actuators.len(), actuators);
    Ok(actuators)
}

/// Update motor state from supervisor feedback
pub async fn update_motor_state(
    supervisor: &mut Supervisor,
    motor_state: &mut MotorState,
    actuator_id: u8,
) -> Result<()> {
    if let Ok(Some((feedback, _))) = supervisor.get_feedback(actuator_id).await {
        motor_state.current_position = feedback.angle;
        motor_state.current_velocity = feedback.velocity;
        motor_state.current_torque = feedback.torque;
        motor_state.current_temperature = feedback.temperature;
        motor_state.fault_uncalibrated = feedback.fault_uncalibrated;
        motor_state.fault_hall_encoding = feedback.fault_hall_encoding;
        motor_state.fault_magnetic_encoding = feedback.fault_magnetic_encoding;
        motor_state.fault_over_temperature = feedback.fault_over_temperature;
        motor_state.fault_overcurrent = feedback.fault_overcurrent;
        motor_state.fault_undervoltage = feedback.fault_undervoltage;
        motor_state.feedback_received = true;
        motor_state.last_feedback_time = Some(Instant::now());
        motor_state.feedback_count += 1;
    }
    Ok(())
}

/// Send position command to motor
pub async fn send_position_command(
    supervisor: &mut Supervisor,
    motor_state: &mut MotorState,
    actuator_id: u8,
) -> Result<()> {
    supervisor
        .command(actuator_id, motor_state.target_position, 0.0, 0.0)
        .await?;
    motor_state.command_count += 1;
    Ok(())
}

/// Apply control configuration based on lock mode
pub async fn apply_control_config(
    supervisor: &mut Supervisor,
    motor_state: &mut MotorState,
    actuator_id: u8,
) -> Result<()> {
    let cfg = if motor_state.lock_mode {
        ControlPresets::lock_mode()
    } else {
        ControlPresets::normal_mode()
    };

    supervisor.configure(actuator_id, cfg.clone()).await?;
    motor_state.current_kp = cfg.kp;
    motor_state.current_kd = cfg.kd;
    motor_state.current_max_torque = cfg.max_torque.unwrap_or(0.0);

    info!(
        "Applied {} mode: kp={}, kd={}, max_torque={}",
        if motor_state.lock_mode { "LOCK" } else { "NORMAL" },
        cfg.kp,
        cfg.kd,
        cfg.max_torque.unwrap_or(0.0)
    );

    Ok(())
}
