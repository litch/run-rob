use anyhow::{Context, Result};
use robstride::{
    ActuatorConfiguration, ActuatorType, SocketCanTransport, Supervisor, TransportType,
    robstride00::RobStride00,
};
use run_rob::{
    ControlPresets, MotorState, scan_actuators, send_position_command,
    update_motor_state,
};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::RwLock;
use tracing::{error, info};
use tracing_subscriber::EnvFilter;
use zenoh::prelude::r#async::*;
use zenoh::publication::Publisher;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GimbalCoreCommand {
    Slew(SlewCommand),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SlewCommand {
    pub yaw: f32,
    pub pitch: f32,
    pub timestamp_ms: u64,
}

impl SlewCommand {
    pub fn new(yaw: f32, pitch: f32) -> Self {
        Self {
            yaw,
            pitch,
            timestamp_ms: std::time::SystemTime::now()
                .duration_since(std::time::SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalState {
    pub yaw_position: f32,
    pub pitch_position: f32,
    pub yaw_velocity: f32,
    pub pitch_velocity: f32,
    pub yaw_torque: f32,
    pub pitch_torque: f32,
    pub yaw_temperature: f32,
    pub pitch_temperature: f32,
    pub timestamp_ms: u64,
}

pub struct ZenohBridge {
    state_publisher: Publisher<'static>,
    motor_states: Arc<RwLock<(MotorState, MotorState)>>, // (yaw, pitch)
}

impl ZenohBridge {
    pub async fn new(
        session: Arc<Session>,
        namespace: String,
        motor_states: Arc<RwLock<(MotorState, MotorState)>>,
    ) -> Result<Self> {
        let clean_namespace = namespace.trim_start_matches('/').to_string();

        let state_topic = format!("{}/gimbal_core/state", clean_namespace);
        let command_topic = format!("{}/gimbal_core/command", clean_namespace);

        info!("Creating Zenoh pub/sub");
        info!("  State: {}", state_topic);
        info!("  Command: {}", command_topic);

        let state_publisher = session
            .declare_publisher(state_topic)
            .res()
            .await
            .map_err(|e| anyhow::anyhow!("Failed to declare publisher: {:?}", e))?;
        let command_subscriber = session
            .declare_subscriber(command_topic)
            .res()
            .await
            .map_err(|e| anyhow::anyhow!("Failed to declare subscriber: {:?}", e))?;

        // Spawn command handler
        let motor_states_clone = motor_states.clone();
        tokio::spawn(async move {
            loop {
                match command_subscriber.recv_async().await {
                    Ok(sample) => {
                        if let Err(e) = Self::handle_command(sample, motor_states_clone.clone()).await
                        {
                            error!("Failed to handle command: {e}");
                        }
                    }
                    Err(e) => {
                        error!("Failed to receive command: {e}");
                    }
                }
            }
        });

        Ok(Self {
            state_publisher,
            motor_states,
        })
    }

    async fn handle_command(
        sample: Sample,
        motor_states: Arc<RwLock<(MotorState, MotorState)>>,
    ) -> Result<()> {
        let payload = sample.value.payload.contiguous();
        let command: GimbalCoreCommand = serde_json::from_slice(&payload)
            .context("Failed to deserialize command")?;

        match command {
            GimbalCoreCommand::Slew(slew) => {
                info!("Received slew command: yaw={:.3}, pitch={:.3}", slew.yaw, slew.pitch);
                let mut states = motor_states.write().await;
                states.0.set_position(slew.yaw);   // yaw motor
                states.1.set_position(slew.pitch); // pitch motor
            }
        }

        Ok(())
    }

    pub async fn publish_state(&self) -> Result<()> {
        let states = self.motor_states.read().await;
        let state = GimbalState {
            yaw_position: states.0.current_position,
            pitch_position: states.1.current_position,
            yaw_velocity: states.0.current_velocity,
            pitch_velocity: states.1.current_velocity,
            yaw_torque: states.0.current_torque,
            pitch_torque: states.1.current_torque,
            yaw_temperature: states.0.current_temperature,
            pitch_temperature: states.1.current_temperature,
            timestamp_ms: std::time::SystemTime::now()
                .duration_since(std::time::SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
        };

        let json = serde_json::to_string(&state)?;
        self.state_publisher
            .put(json)
            .res()
            .await
            .map_err(|e| anyhow::anyhow!("Failed to publish state: {:?}", e))?;

        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::from_default_env()
                .add_directive("polling=off".parse().unwrap())
                .add_directive("async_io=off".parse().unwrap())
                .add_directive("run_rob=info".parse().unwrap()),
        )
        .init();

    info!("Starting Gimbal Core Bus");

    // Setup motor supervisor
    let mut supervisor = Supervisor::new(Duration::from_millis(1000))
        .map_err(|e| anyhow::anyhow!("Failed to create supervisor: {}", e))?;
    let socketcan = SocketCanTransport::new("can0".to_string())
        .await
        .map_err(|e| anyhow::anyhow!("Failed to create SocketCAN transport: {}", e))?;
    supervisor
        .add_transport("socketcan".to_string(), TransportType::SocketCAN(socketcan))
        .await
        .map_err(|e| anyhow::anyhow!("Failed to add transport: {}", e))?;

    let mut supervisor_runner = supervisor.clone_controller();
    let supervisor_handle = tokio::spawn(async move {
        info!("Starting supervisor task");
        if let Err(e) = supervisor_runner.run(Duration::from_millis(10)).await {
            error!("Supervisor task failed: {}", e);
        }
    });

    // Scan for available actuators
    info!("Scanning for available actuators...");
    let available_actuators = scan_actuators(&mut supervisor)
        .await
        .map_err(|e| anyhow::anyhow!("Failed to scan actuators: {}", e))?;

    if available_actuators.len() < 2 {
        error!("Need at least 2 actuators for gimbal control, found {}", available_actuators.len());
        return Err(anyhow::anyhow!("Insufficient actuators"));
    }

    let yaw_motor_id = available_actuators[1];
    let pitch_motor_id = available_actuators[0];
    info!("Using actuator {} for YAW, {} for PITCH", yaw_motor_id, pitch_motor_id);

    // Initialize motors
    for &motor_id in &[yaw_motor_id, pitch_motor_id] {
        supervisor
            .add_actuator(
                Box::new(RobStride00::new(
                    motor_id,
                    0xFE,
                    supervisor
                        .get_transport_tx("socketcan")
                        .await
                        .map_err(|e| anyhow::anyhow!("Failed to get transport: {}", e))?,
                )),
                ActuatorConfiguration {
                    actuator_type: ActuatorType::RobStride00,
                    ..Default::default()
                },
            )
            .await;

        let cfg = ControlPresets::normal_mode();
        supervisor
            .configure(motor_id, cfg.clone())
            .await
            .map_err(|e| anyhow::anyhow!("Failed to configure motor {}: {}", motor_id, e))?;
        supervisor
            .enable(motor_id)
            .await
            .map_err(|e| anyhow::anyhow!("Failed to enable motor {}: {}", motor_id, e))?;
        supervisor
            .zero(motor_id)
            .await
            .map_err(|e| anyhow::anyhow!("Failed to zero motor {}: {}", motor_id, e))?;
        info!("Initialized motor {}", motor_id);
    }

    tokio::time::sleep(Duration::from_millis(500)).await;

    // Initialize motor states
    let mut yaw_state = MotorState::new();
    let mut pitch_state = MotorState::new();

    // Get initial feedback
    for (motor_id, state) in [(yaw_motor_id, &mut yaw_state), (pitch_motor_id, &mut pitch_state)] {
        if let Ok(Some((feedback, _))) = supervisor
            .get_feedback(motor_id)
            .await
            .map_err(|e| anyhow::anyhow!("Failed to get feedback: {}", e))
        {
            state.current_position = feedback.angle;
            state.target_position = feedback.angle;
            state.feedback_received = true;
            state.motor_enabled = true;
            info!("Initial feedback from motor {}: angle={:.4}", motor_id, feedback.angle);
        }
    }

    let motor_states = Arc::new(RwLock::new((yaw_state, pitch_state)));

    // Setup Zenoh
    info!("Connecting to Zenoh...");
    let zenoh_config = zenoh::config::Config::default();
    let session = Arc::new(
        zenoh::open(zenoh_config)
            .res()
            .await
            .map_err(|e| anyhow::anyhow!("Failed to connect to Zenoh: {:?}", e))?,
    );
    info!("Connected to Zenoh");

    // Create Zenoh bridge
    let namespace = std::env::var("ZENOH_NAMESPACE").unwrap_or_else(|_| "gimbal".to_string());
    let bridge = ZenohBridge::new(session.clone(), namespace, motor_states.clone()).await?;
    info!("Zenoh bridge created");

    // Spawn motor control loop (200Hz)
    let motor_states_clone = motor_states.clone();
    let mut supervisor_clone = supervisor.clone_controller();
    let motor_control_handle = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(5)); // 200Hz
        loop {
            interval.tick().await;

            let mut states = motor_states_clone.write().await;
            
            // Update yaw motor
            if let Err(e) = update_motor_state(&mut supervisor_clone, &mut states.0, yaw_motor_id).await {
                error!("Failed to update yaw motor state: {}", e);
            }
            if let Err(e) = send_position_command(&mut supervisor_clone, &mut states.0, yaw_motor_id).await {
                error!("Failed to send yaw motor command: {}", e);
            }

            // Update pitch motor
            if let Err(e) = update_motor_state(&mut supervisor_clone, &mut states.1, pitch_motor_id).await {
                error!("Failed to update pitch motor state: {}", e);
            }
            if let Err(e) = send_position_command(&mut supervisor_clone, &mut states.1, pitch_motor_id).await {
                error!("Failed to send pitch motor command: {}", e);
            }
        }
    });

    // Spawn state publishing loop (200Hz)
    let state_publish_handle = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(5)); // 200Hz
        let mut count = 0u64;
        loop {
            interval.tick().await;
            
            if let Err(e) = bridge.publish_state().await {
                error!("Failed to publish state: {}", e);
            }

            count += 1;
            if count % 200 == 0 {
                info!("Published {} state updates", count);
            }
        }
    });

    info!("Gimbal Core Bus is running");
    info!("  - Receiving slew commands on: gimbal/gimbal_core/command");
    info!("  - Publishing state at 200Hz on: gimbal/gimbal_core/state");

    // Keep running
    tokio::signal::ctrl_c().await?;
    info!("Shutting down gracefully...");

    // Stop background tasks first
    motor_control_handle.abort();
    state_publish_handle.abort();
    
    // Give tasks a moment to stop
    tokio::time::sleep(Duration::from_millis(50)).await;

    // IMPORTANT: Disable motors first (this stops the control loop)
    info!("Disabling motors...");
    match supervisor.disable(yaw_motor_id, false).await {
        Ok(_) => info!("Yaw motor disabled successfully"),
        Err(e) => error!("Failed to disable yaw motor: {}", e),
    }
    match supervisor.disable(pitch_motor_id, false).await {
        Ok(_) => info!("Pitch motor disabled successfully"),
        Err(e) => error!("Failed to disable pitch motor: {}", e),
    }

    // Wait for disable commands to be processed
    tokio::time::sleep(Duration::from_millis(200)).await;

    // Stop supervisor task last
    supervisor_handle.abort();
    
    // Give supervisor time to clean up
    tokio::time::sleep(Duration::from_millis(100)).await;

    info!("Shutdown complete - motors disabled");
    Ok(())
}
