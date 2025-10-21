use run_rob::{
    MotorState, ControlPresets, scan_actuators, update_motor_state,
    send_position_command, apply_control_config, record_episode,
};
use robstride::{
    robstride00::RobStride00, ActuatorConfiguration, ActuatorType,
    SocketCanTransport, Supervisor, TransportType,
};
use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::IntoResponse,
    routing::{get, post},
    Json, Router,
};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::RwLock;
use tower_http::cors::CorsLayer;
use tracing::{error, info};
use tracing_subscriber::EnvFilter;

/// Shared application state
struct AppState {
    supervisor: Arc<RwLock<Supervisor>>,
    motor_state: Arc<RwLock<MotorState>>,
    current_actuator: Arc<RwLock<u8>>,
}

/// Request body for setting position
#[derive(Debug, Deserialize)]
struct SetPositionRequest {
    position: f32,
}

/// Request body for incrementing position
#[derive(Debug, Deserialize)]
struct IncrementRequest {
    amount: f32,
}

/// Request body for recording episode
#[derive(Debug, Deserialize)]
struct RecordRequest {
    duration_secs: f32,
}

/// Response for status endpoint
#[derive(Debug, Serialize)]
struct StatusResponse {
    actuator_id: u8,
    motor_state: MotorState,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::from_default_env()
                .add_directive("polling=off".parse().unwrap())
                .add_directive("async_io=off".parse().unwrap()),
        )
        .init();

    info!("Starting RobStride Motor Control API");

    // Setup motor supervisor
    let mut supervisor = Supervisor::new(Duration::from_millis(1000))?;
    let socketcan = SocketCanTransport::new("can0".to_string()).await?;
    supervisor
        .add_transport("socketcan".to_string(), TransportType::SocketCAN(socketcan))
        .await?;

    let mut supervisor_runner = supervisor.clone_controller();
    tokio::spawn(async move {
        info!("Starting supervisor task");
        if let Err(e) = supervisor_runner.run(Duration::from_millis(10)).await {
            error!("Supervisor task failed: {}", e);
        }
    });

    // Scan for available actuators
    info!("Scanning for available actuators...");
    let available_actuators = scan_actuators(&mut supervisor).await?;

    // Use first available actuator, or default to 1
    let actuator_id = *available_actuators.first().unwrap_or(&1);
    info!("Auto-connecting to actuator {}", actuator_id);

    supervisor
        .add_actuator(
            Box::new(RobStride00::new(
                actuator_id,
                0xFE,
                supervisor.get_transport_tx("socketcan").await?,
            )),
            ActuatorConfiguration {
                actuator_type: ActuatorType::RobStride00,
                ..Default::default()
            },
        )
        .await;

    // Configure and enable motor
    let cfg = ControlPresets::normal_mode();
    supervisor.configure(actuator_id, cfg.clone()).await?;
    supervisor.enable(actuator_id).await?;
    supervisor.zero(actuator_id).await?;

    tokio::time::sleep(Duration::from_millis(500)).await;

    // Initialize motor state
    let mut motor_state = MotorState::new();
    motor_state.motor_enabled = true;
    motor_state.available_actuators = available_actuators;
    motor_state.current_kp = cfg.kp;
    motor_state.current_kd = cfg.kd;
    motor_state.current_max_torque = cfg.max_torque.unwrap_or(0.0);

    // Create shared state
    let state = Arc::new(AppState {
        supervisor: Arc::new(RwLock::new(supervisor)),
        motor_state: Arc::new(RwLock::new(motor_state)),
        current_actuator: Arc::new(RwLock::new(actuator_id)),
    });

    // Spawn background task to update motor state
    let state_clone = state.clone();
    tokio::spawn(async move {
        loop {
            let actuator_id = *state_clone.current_actuator.read().await;
            let mut supervisor = state_clone.supervisor.write().await;
            let mut motor_state = state_clone.motor_state.write().await;

            // Update motor feedback
            if let Err(e) = update_motor_state(&mut supervisor, &mut motor_state, actuator_id).await
            {
                error!("Failed to update motor state: {}", e);
            }

            // Send position command
            if let Err(e) = send_position_command(&mut supervisor, &mut motor_state, actuator_id).await
            {
                error!("Failed to send position command: {}", e);
            }

            drop(supervisor);
            drop(motor_state);

            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    });

    // Build API router
    let app = Router::new()
        .route("/api/status", get(get_status))
        .route("/api/actuators", get(get_actuators))
        .route("/api/actuator/:id", post(switch_actuator))
        .route("/api/position", post(set_position))
        .route("/api/position/increment", post(increment_position))
        .route("/api/position/decrement", post(decrement_position))
        .route("/api/zero", post(zero_motor))
        .route("/api/target-zero", post(target_zero))
        .route("/api/lock", post(toggle_lock))
        .route("/api/enable", post(enable_motor))
        .route("/api/disable", post(disable_motor))
        .route("/api/record", post(record_episode_handler))
        .layer(CorsLayer::permissive())
        .with_state(state);

    // Start server
    let addr = "0.0.0.0:8080";
    info!("API server listening on {}", addr);
    let listener = tokio::net::TcpListener::bind(addr).await?;
    axum::serve(listener, app).await?;

    Ok(())
}

/// GET /api/status - Get current motor state
async fn get_status(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let motor_state = state.motor_state.read().await;
    let actuator_id = *state.current_actuator.read().await;

    Json(StatusResponse {
        actuator_id,
        motor_state: motor_state.clone(),
    })
}

/// GET /api/actuators - List available actuators
async fn get_actuators(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let motor_state = state.motor_state.read().await;
    Json(motor_state.available_actuators.clone())
}

/// POST /api/actuator/:id - Switch to actuator
async fn switch_actuator(
    State(state): State<Arc<AppState>>,
    Path(id): Path<u8>,
) -> impl IntoResponse {
    let mut current_actuator = state.current_actuator.write().await;
    let motor_state = state.motor_state.read().await;

    if !motor_state.available_actuators.contains(&id) {
        return (StatusCode::BAD_REQUEST, "Actuator not available").into_response();
    }

    let mut supervisor = state.supervisor.write().await;

    // Disable current actuator
    let _ = supervisor.disable(*current_actuator, false).await;

    // Switch to new actuator
    *current_actuator = id;

    // Add and enable new actuator
    if let Ok(tx) = supervisor.get_transport_tx("socketcan").await {
        supervisor
            .add_actuator(
                Box::new(RobStride00::new(id, 0xFE, tx)),
                ActuatorConfiguration {
                    actuator_type: ActuatorType::RobStride00,
                    ..Default::default()
                },
            )
            .await;

        let cfg = ControlPresets::normal_mode();
        let _ = supervisor.configure(id, cfg).await;
        let _ = supervisor.enable(id).await;

        info!("Switched to actuator {}", id);
        (StatusCode::OK, format!("Switched to actuator {}", id)).into_response()
    } else {
        (StatusCode::INTERNAL_SERVER_ERROR, "Failed to switch actuator").into_response()
    }
}

/// POST /api/position - Set target position
async fn set_position(
    State(state): State<Arc<AppState>>,
    Json(req): Json<SetPositionRequest>,
) -> impl IntoResponse {
    let mut motor_state = state.motor_state.write().await;
    motor_state.target_position = req.position;
    info!("Set target position to {}", req.position);
    (StatusCode::OK, format!("Position set to {}", req.position))
}

/// POST /api/position/increment - Increment position
async fn increment_position(
    State(state): State<Arc<AppState>>,
    Json(req): Json<IncrementRequest>,
) -> impl IntoResponse {
    let mut motor_state = state.motor_state.write().await;
    motor_state.target_position += req.amount;
    info!("Incremented position by {}", req.amount);
    (StatusCode::OK, format!("Position incremented by {}", req.amount))
}

/// POST /api/position/decrement - Decrement position
async fn decrement_position(
    State(state): State<Arc<AppState>>,
    Json(req): Json<IncrementRequest>,
) -> impl IntoResponse {
    let mut motor_state = state.motor_state.write().await;
    motor_state.target_position -= req.amount;
    info!("Decremented position by {}", req.amount);
    (StatusCode::OK, format!("Position decremented by {}", req.amount))
}

/// POST /api/zero - Zero the motor (set current position as zero)
async fn zero_motor(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let actuator_id = *state.current_actuator.read().await;
    let mut supervisor = state.supervisor.write().await;
    let mut motor_state = state.motor_state.write().await;

    if let Err(e) = supervisor.zero(actuator_id).await {
        error!("Failed to zero motor: {}", e);
        return (StatusCode::INTERNAL_SERVER_ERROR, format!("Failed to zero motor: {}", e));
    }

    motor_state.target_position = 0.0;
    info!("Zeroed actuator {}", actuator_id);
    (StatusCode::OK, format!("Zeroed actuator {}", actuator_id))
}

/// POST /api/target-zero - Set target position to zero
async fn target_zero(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let mut motor_state = state.motor_state.write().await;
    motor_state.set_target_zero();
    info!("Set target position to zero");
    (StatusCode::OK, "Target position set to zero")
}

/// POST /api/lock - Toggle lock mode
async fn toggle_lock(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let actuator_id = *state.current_actuator.read().await;
    let mut supervisor = state.supervisor.write().await;
    let mut motor_state = state.motor_state.write().await;

    motor_state.toggle_lock_mode();

    if let Err(e) = apply_control_config(&mut supervisor, &mut motor_state, actuator_id).await {
        error!("Failed to apply control config: {}", e);
        return (
            StatusCode::INTERNAL_SERVER_ERROR,
            format!("Failed to toggle lock mode: {}", e),
        );
    }

    let mode = if motor_state.lock_mode { "LOCKED" } else { "NORMAL" };
    info!("Lock mode: {}", mode);
    (StatusCode::OK, format!("Lock mode: {}", mode))
}

/// POST /api/enable - Enable motor
async fn enable_motor(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let actuator_id = *state.current_actuator.read().await;
    let mut supervisor = state.supervisor.write().await;
    let mut motor_state = state.motor_state.write().await;

    if let Err(e) = supervisor.enable(actuator_id).await {
        error!("Failed to enable motor: {}", e);
        return (StatusCode::INTERNAL_SERVER_ERROR, format!("Failed to enable motor: {}", e));
    }

    motor_state.motor_enabled = true;
    info!("Enabled actuator {}", actuator_id);
    (StatusCode::OK, format!("Enabled actuator {}", actuator_id))
}

/// POST /api/disable - Disable motor
async fn disable_motor(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let actuator_id = *state.current_actuator.read().await;
    let mut supervisor = state.supervisor.write().await;
    let mut motor_state = state.motor_state.write().await;

    if let Err(e) = supervisor.disable(actuator_id, false).await {
        error!("Failed to disable motor: {}", e);
        return (StatusCode::INTERNAL_SERVER_ERROR, format!("Failed to disable motor: {}", e));
    }

    motor_state.motor_enabled = false;
    info!("Disabled actuator {}", actuator_id);
    (StatusCode::OK, format!("Disabled actuator {}", actuator_id))
}

/// POST /api/record - Record motor data for specified duration
async fn record_episode_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<RecordRequest>,
) -> impl IntoResponse {
    let actuator_id = *state.current_actuator.read().await;
    let mut supervisor = state.supervisor.write().await;
    let mut motor_state = state.motor_state.write().await;

    info!("Starting recording for {} seconds", req.duration_secs);

    match record_episode(&mut supervisor, &mut motor_state, actuator_id, req.duration_secs).await {
        Ok(episode) => {
            info!("Recording complete: {} samples", episode.data_points.len());
            (StatusCode::OK, Json(episode)).into_response()
        }
        Err(e) => {
            error!("Failed to record episode: {}", e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to record episode: {}", e),
            )
                .into_response()
        }
    }
}
