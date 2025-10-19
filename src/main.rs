use robstride::{
    robstride00::{RobStride00}, ActuatorConfiguration, ActuatorType, ControlConfig, SocketCanTransport, Supervisor, TransportType
};
use std::time::Duration;
use tracing::{error, info};
use tracing_subscriber::EnvFilter;
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout, Alignment},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph, List, ListItem},
    Terminal,
};
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use std::io;




struct MotorState {
    target_position: f32,
    current_position: f32,
    current_velocity: f32,
    current_torque: f32,
    current_temperature: f32,
    fault_uncalibrated: bool,
    fault_hall_encoding: bool,
    fault_magnetic_encoding: bool,
    fault_over_temperature: bool,
    fault_overcurrent: bool,
    fault_undervoltage: bool,
    increment: f32,
    increment_index: usize,
    feedback_received: bool,
    last_feedback_time: Option<std::time::Instant>,
    command_count: u64,
    feedback_count: u64,
    motor_enabled: bool,
    lock_mode: bool,
    available_actuators: Vec<u8>,
    last_scan_time: Option<std::time::Instant>,
    current_kp: f32,
    current_kd: f32,
    current_max_torque: f32,
}

impl MotorState {
    fn new() -> Self {
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

    fn get_increments() -> [f32; 3] {
        [0.001, 0.01, 0.1]
    }

    fn cycle_increment(&mut self) {
        let increments = Self::get_increments();
        self.increment_index = (self.increment_index + 1) % increments.len();
        self.increment = increments[self.increment_index];
    }

    fn increase_position(&mut self) {
        self.target_position += self.increment;
    }

    fn decrease_position(&mut self) {
        self.target_position -= self.increment;
    }

    fn set_target_zero(&mut self) {
        self.target_position = 0.0;
    }

    fn toggle_lock_mode(&mut self) {
        self.lock_mode = !self.lock_mode;
    }
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // Initialize logging to file instead of stdout (since we're using TUI)
    let file_appender = tracing_appender::rolling::daily("./logs", "motor-control.log");
    let (non_blocking, _guard) = tracing_appender::non_blocking(file_appender);
    
    tracing_subscriber::fmt()
        .with_writer(non_blocking)
        .with_env_filter(EnvFilter::from_default_env()
            .add_directive("polling=off".parse().unwrap())
            .add_directive("async_io=off".parse().unwrap()),
        )
        .init();

    // Setup motor
    let mut supervisor = Supervisor::new(Duration::from_millis(1000))?;
    let socketcan = SocketCanTransport::new("can0".to_string()).await?;
    supervisor
        .add_transport("socketcan".to_string(), TransportType::SocketCAN(socketcan))
        .await?;

    let mut supervisor_runner = supervisor.clone_controller();
    let supervisor_handle = tokio::spawn(async move {
        info!("starting supervisor task");
        if let Err(e) = supervisor_runner.run(Duration::from_millis(10)).await {
            error!("Supervisor task failed: {}", e);
        }
    });

    // Scan for available actuators first
    info!("Scanning for available actuators...");
    let actuator_configs = vec![
        (1, ActuatorConfiguration { actuator_type: ActuatorType::RobStride00, ..Default::default() }),
        (2, ActuatorConfiguration { actuator_type: ActuatorType::RobStride00, ..Default::default() }),
        (3, ActuatorConfiguration { actuator_type: ActuatorType::RobStride00, ..Default::default() }),
        (127, ActuatorConfiguration { actuator_type: ActuatorType::RobStride00, ..Default::default() }),
    ];
    
    let available_actuators = supervisor.scan_bus(0xFE, "socketcan", &actuator_configs).await?;
    info!("Found {} actuators: {:?}", available_actuators.len(), available_actuators);
    
    // Use first available actuator, or default to 1
    let actuator_id = *available_actuators.first().unwrap_or(&1);
    info!("Auto-connecting to actuator {}", actuator_id);
    
    supervisor.add_actuator(
        Box::new(RobStride00::new(
            actuator_id,
            0xFE,
            supervisor.get_transport_tx("socketcan").await?,
        )),
        ActuatorConfiguration {
            actuator_type: ActuatorType::RobStride00,
            ..Default::default()
        },
    ).await;

    info!("Enabling the actuator {}", actuator_id);

    // Start with high stiffness configuration
    let cfg = ControlConfig {
        kp: 150.0,
        kd: 8.0,
        max_torque: Some(150.0),
        max_velocity: Some(80.0),
        max_current: Some(100.0), 
    };
    supervisor.configure(actuator_id, cfg.clone()).await?;
    supervisor.enable(actuator_id).await?;
    supervisor.zero(actuator_id).await?;

    tokio::time::sleep(Duration::from_millis(500)).await;

    // Initialize motor state
    let mut motor_state = MotorState::new();
    motor_state.motor_enabled = true;
    
    // Get initial feedback
    if let Ok(Some((feedback, _))) = supervisor.get_feedback(actuator_id).await {
        motor_state.current_position = feedback.angle;
        motor_state.target_position = feedback.angle;
        motor_state.feedback_received = true;
        motor_state.last_feedback_time = Some(std::time::Instant::now());
        motor_state.feedback_count = 1;
        info!("Initial feedback received: angle={:.4}, velocity={:.4}, torque={:.4}", 
              feedback.angle, feedback.velocity, feedback.torque);
    } else {
        info!("WARNING: No initial feedback received from motor!");
    }

    // Setup terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let result = run_tui(&mut terminal, &mut motor_state, &mut supervisor, actuator_id).await;

    // Restore terminal
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;

    // Cleanup
    supervisor.command(actuator_id, 0.0, 0.0, 0.0).await?;
    supervisor_handle.abort();
    supervisor.disable(actuator_id, false).await?;

    result
}

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    motor_state: &mut MotorState,
    supervisor: &mut Supervisor,
    mut actuator_id: u8,
) -> eyre::Result<()> {
    let mut last_update = std::time::Instant::now();
    let mut current_lock_mode = motor_state.lock_mode;
    let mut current_actuator_index = 0usize;
    
    loop {
        // Scan for available actuators every 5 seconds
        if motor_state.last_scan_time.is_none() || 
           motor_state.last_scan_time.unwrap().elapsed() >= Duration::from_secs(5) {
            // Scan bus for actuators
            let actuator_configs = vec![
                (1, ActuatorConfiguration { actuator_type: ActuatorType::RobStride00, ..Default::default() }),
                (2, ActuatorConfiguration { actuator_type: ActuatorType::RobStride00, ..Default::default() }),
                (3, ActuatorConfiguration { actuator_type: ActuatorType::RobStride00, ..Default::default() }),
                (127, ActuatorConfiguration { actuator_type: ActuatorType::RobStride00, ..Default::default() }),
            ];
            
            if let Ok(found_actuators) = supervisor.scan_bus(0xFE, "socketcan", &actuator_configs).await {
                motor_state.available_actuators = found_actuators;
                motor_state.last_scan_time = Some(std::time::Instant::now());
                info!("Scanned bus, found {} actuators: {:?}", motor_state.available_actuators.len(), motor_state.available_actuators);
            }
        }
        
        // Check if lock mode changed and update motor configuration
        if current_lock_mode != motor_state.lock_mode {
            let cfg = if motor_state.lock_mode {
                // Lock mode: MAXIMUM stiffness - lock it down hard
                ControlConfig {
                    kp: 200.0,  // Maximum position gain
                    kd: 10.0,   // Maximum damping
                    max_torque: Some(200.0),  // Maximum torque
                    max_velocity: Some(100.0),
                    max_current: Some(100.0),  // Maximum current
                }
            } else {
                // Normal mode: High stiffness (much stiffer than before)
                ControlConfig {
                    kp: 150.0,  // Very high position gain
                    kd: 8.0,    // High damping
                    max_torque: Some(150.0),
                    max_velocity: Some(80.0),
                    max_current: Some(100.0),
                }
            };
            
            supervisor.configure(actuator_id, cfg.clone()).await?;
            motor_state.current_kp = cfg.kp;
            motor_state.current_kd = cfg.kd;
            motor_state.current_max_torque = cfg.max_torque.unwrap_or(0.0);
            current_lock_mode = motor_state.lock_mode;
            info!("Lock mode {}: kp={}, kd={}, max_torque={}", 
                  if motor_state.lock_mode { "ENABLED" } else { "DISABLED" },
                  cfg.kp, cfg.kd, cfg.max_torque.unwrap_or(0.0));
        }
        
        // Update motor feedback at regular intervals
        if last_update.elapsed() >= Duration::from_millis(50) {
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
                motor_state.last_feedback_time = Some(std::time::Instant::now());
                motor_state.feedback_count += 1;
                
                // Log all feedback fields to help debug
                if motor_state.feedback_count % 20 == 0 {
                    info!("Feedback #{}: angle={:.4}, vel={:.4}, torque={:.4}, temp={:.1}, faults=[uncal:{}, hall:{}, mag:{}, temp:{}, curr:{}, volt:{}]", 
                          motor_state.feedback_count, feedback.angle, feedback.velocity, 
                          feedback.torque, feedback.temperature,
                          feedback.fault_uncalibrated, feedback.fault_hall_encoding,
                          feedback.fault_magnetic_encoding, feedback.fault_over_temperature,
                          feedback.fault_overcurrent, feedback.fault_undervoltage);
                }
            } else {
                motor_state.feedback_received = false;
            }
            
            // Send position command
            if let Err(e) = supervisor.command(actuator_id, motor_state.target_position, 0.0, 0.0).await {
                error!("Failed to send command: {}", e);
            } else {
                motor_state.command_count += 1;
            }
            last_update = std::time::Instant::now();
        }

        // Draw UI
        terminal.draw(|f| {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .margin(2)
                .constraints([
                    Constraint::Length(3),
                    Constraint::Length(8),
                    Constraint::Length(20),
                    Constraint::Length(8),
                    Constraint::Min(5),
                ])
                .split(f.area());

            // Title
            let title = Paragraph::new("RobStride Motor Controller")
                .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD))
                .alignment(Alignment::Center)
                .block(Block::default().borders(Borders::ALL));
            f.render_widget(title, chunks[0]);

            // Connection Status
            let connection_status = if motor_state.feedback_received {
                ("CONNECTED", Color::Green)
            } else {
                ("DISCONNECTED", Color::Red)
            };
            
            let motor_status = if motor_state.motor_enabled {
                ("ENABLED", Color::Green)
            } else {
                ("DISABLED", Color::Yellow)
            };

            let feedback_age = motor_state.last_feedback_time
                .map(|t| t.elapsed().as_millis())
                .unwrap_or(9999);
            
            let lock_status = if motor_state.lock_mode {
                ("LOCKED", Color::Red)
            } else {
                ("NORMAL", Color::Green)
            };

            // Format available actuators list
            let actuators_str = if motor_state.available_actuators.is_empty() {
                "Scanning...".to_string()
            } else {
                motor_state.available_actuators.iter()
                    .map(|id| {
                        if *id == actuator_id {
                            format!("[{}]", id)  // Highlight current actuator
                        } else {
                            format!("{}", id)
                        }
                    })
                    .collect::<Vec<_>>()
                    .join(", ")
            };

            let connection_text = vec![
                Line::from(vec![
                    Span::styled("Connection: ", Style::default().fg(Color::Yellow)),
                    Span::styled(connection_status.0, Style::default().fg(connection_status.1).add_modifier(Modifier::BOLD)),
                    Span::raw("  |  "),
                    Span::styled("Motor: ", Style::default().fg(Color::Yellow)),
                    Span::styled(motor_status.0, Style::default().fg(motor_status.1).add_modifier(Modifier::BOLD)),
                ]),
                Line::from(vec![
                    Span::styled("Lock Mode: ", Style::default().fg(Color::Yellow)),
                    Span::styled(lock_status.0, Style::default().fg(lock_status.1).add_modifier(Modifier::BOLD)),
                ]),
                Line::from(vec![
                    Span::styled("Commands Sent: ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{}", motor_state.command_count), Style::default().fg(Color::Cyan)),
                    Span::raw("  |  "),
                    Span::styled("Feedback Received: ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{}", motor_state.feedback_count), Style::default().fg(Color::Cyan)),
                ]),
                Line::from(vec![
                    Span::styled("Last Feedback: ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{}ms ago", feedback_age), 
                        Style::default().fg(if feedback_age < 100 { Color::Green } else if feedback_age < 500 { Color::Yellow } else { Color::Red })),
                ]),
                Line::from(vec![
                    Span::styled("Active Actuator: ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{}", actuator_id), Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                ]),
                Line::from(vec![
                    Span::styled("Available Actuators: ", Style::default().fg(Color::Yellow)),
                    Span::styled(actuators_str, Style::default().fg(Color::Green)),
                ]),
            ];
            
            let connection = Paragraph::new(connection_text)
                .block(Block::default().borders(Borders::ALL).title("Connection & Diagnostics"));
            f.render_widget(connection, chunks[1]);

            // Motor Status
            let temp_color = if motor_state.current_temperature > 60.0 {
                Color::Red
            } else if motor_state.current_temperature > 50.0 {
                Color::Yellow
            } else {
                Color::Green
            };
            
            let has_faults = motor_state.fault_uncalibrated || motor_state.fault_hall_encoding || 
                            motor_state.fault_magnetic_encoding || motor_state.fault_over_temperature ||
                            motor_state.fault_overcurrent || motor_state.fault_undervoltage;
            
            let status_text = vec![
                Line::from(vec![
                    Span::styled("Target pPosition:   ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{:8.4} rad ({:7.2}°)", motor_state.target_position, motor_state.target_position.to_degrees()), Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                ]),
                Line::from(vec![
                    Span::styled("Current Position:  ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{:8.4} rad ({:7.2}°)", motor_state.current_position, motor_state.current_position.to_degrees()), Style::default().fg(Color::White)),
                ]),
                Line::from(vec![
                    Span::styled("Position Error:    ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{:8.4} rad ({:7.2}°)", motor_state.target_position - motor_state.current_position, (motor_state.target_position - motor_state.current_position).to_degrees()), Style::default().fg(Color::Red)),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled("Velocity:          ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{:8.4} rad/s", motor_state.current_velocity), Style::default().fg(Color::White)),
                ]),
                Line::from(vec![
                    Span::styled("Torque:            ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{:8.4} Nm", motor_state.current_torque), Style::default().fg(Color::White)),
                ]),
                Line::from(vec![
                    Span::styled("Temperature:       ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{:5.1} °C", motor_state.current_temperature), Style::default().fg(temp_color)),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled("Control Config:    ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("kp={:.1}  kd={:.1}  max_torque={:.1} Nm", 
                        motor_state.current_kp, motor_state.current_kd, motor_state.current_max_torque), 
                        Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled("Faults: ", Style::default().fg(Color::Yellow)),
                    Span::styled(if has_faults { "ACTIVE" } else { "None" }, 
                        Style::default().fg(if has_faults { Color::Red } else { Color::Green }).add_modifier(Modifier::BOLD)),
                ]),
                Line::from(vec![
                    Span::styled("  Uncalibrated:    ", Style::default().fg(Color::Gray)),
                    Span::styled(if motor_state.fault_uncalibrated { "YES" } else { "no" }, 
                        Style::default().fg(if motor_state.fault_uncalibrated { Color::Red } else { Color::DarkGray })),
                    Span::raw("  "),
                    Span::styled("Hall Enc:    ", Style::default().fg(Color::Gray)),
                    Span::styled(if motor_state.fault_hall_encoding { "YES" } else { "no" }, 
                        Style::default().fg(if motor_state.fault_hall_encoding { Color::Red } else { Color::DarkGray })),
                ]),
                Line::from(vec![
                    Span::styled("  Magnetic Enc:    ", Style::default().fg(Color::Gray)),
                    Span::styled(if motor_state.fault_magnetic_encoding { "YES" } else { "no" }, 
                        Style::default().fg(if motor_state.fault_magnetic_encoding { Color::Red } else { Color::DarkGray })),
                    Span::raw("  "),
                    Span::styled("Over Temp:   ", Style::default().fg(Color::Gray)),
                    Span::styled(if motor_state.fault_over_temperature { "YES" } else { "no" }, 
                        Style::default().fg(if motor_state.fault_over_temperature { Color::Red } else { Color::DarkGray })),
                ]),
                Line::from(vec![
                    Span::styled("  Overcurrent:     ", Style::default().fg(Color::Gray)),
                    Span::styled(if motor_state.fault_overcurrent { "YES" } else { "no" }, 
                        Style::default().fg(if motor_state.fault_overcurrent { Color::Red } else { Color::DarkGray })),
                    Span::raw("  "),
                    Span::styled("Undervolt:   ", Style::default().fg(Color::Gray)),
                    Span::styled(if motor_state.fault_undervoltage { "YES" } else { "no" }, 
                        Style::default().fg(if motor_state.fault_undervoltage { Color::Red } else { Color::DarkGray })),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled("Current Increment: ", Style::default().fg(Color::Yellow)),
                    Span::styled(format!("{:.4} rad", motor_state.increment), Style::default().fg(Color::Magenta).add_modifier(Modifier::BOLD)),
                ]),
            ];
            
            let status = Paragraph::new(status_text)
                .block(Block::default().borders(Borders::ALL).title("Motor Status"));
            f.render_widget(status, chunks[2]);

            // Available Increments
            let increments = MotorState::get_increments();
            let increment_items: Vec<ListItem> = increments
                .iter()
                .enumerate()
                .map(|(i, inc)| {
                    let style = if i == motor_state.increment_index {
                        Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)
                    } else {
                        Style::default().fg(Color::Gray)
                    };
                    let marker = if i == motor_state.increment_index { "→ " } else { "  " };
                    ListItem::new(format!("{}{:.4} rad", marker, inc)).style(style)
                })
                .collect();
            
            let increments_list = List::new(increment_items)
                .block(Block::default().borders(Borders::ALL).title("Available Increments (Tab to cycle)"));
            f.render_widget(increments_list, chunks[3]);

            // Controls
            let controls_text = vec![
                Line::from(vec![
                    Span::styled("↑/W", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::raw("  Increase position  "),
                    Span::styled("↓/S", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::raw("  Decrease position"),
                ]),
                Line::from(vec![
                    Span::styled("Tab", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::raw("  Cycle increment    "),
                    Span::styled("0", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::raw("    Set zero point"),
                ]),
                Line::from(vec![
                    Span::styled("Z", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::raw("    Return to zero     "),
                    Span::styled("L", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::raw("    Toggle Lock"),
                ]),
                Line::from(vec![
                    Span::styled("←/→", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::raw("  Switch actuator"),
                ]),
                Line::from(vec![
                    Span::styled("Q/Esc", Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)),
                    Span::raw("  Quit"),
                ]),
            ];
            
            let controls = Paragraph::new(controls_text)
                .block(Block::default().borders(Borders::ALL).title("Controls"));
            f.render_widget(controls, chunks[4]);
        })?;

        // Handle input (non-blocking)
        if event::poll(Duration::from_millis(10))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => {
                            break;
                        }
                        KeyCode::Up | KeyCode::Char('w') | KeyCode::Char('W') => {
                            motor_state.increase_position();
                        }
                        KeyCode::Down | KeyCode::Char('s') | KeyCode::Char('S') => {
                            motor_state.decrease_position();
                        }
                        KeyCode::Tab => {
                            motor_state.cycle_increment();
                        }
                        KeyCode::Char('0') => {
                            // Set current position as zero (calibration)
                            let _ = supervisor.zero(actuator_id).await;
                            motor_state.target_position = 0.0;
                            info!("Set zero point for actuator {}", actuator_id);
                        }
                        KeyCode::Char('z') | KeyCode::Char('Z') => {
                            // Command motor to return to zero position
                            motor_state.set_target_zero();
                        }
                        KeyCode::Char('l') | KeyCode::Char('L') => {
                            motor_state.toggle_lock_mode();
                        }
                        KeyCode::Left => {
                            // Switch to previous actuator
                            if !motor_state.available_actuators.is_empty() {
                                if current_actuator_index == 0 {
                                    current_actuator_index = motor_state.available_actuators.len() - 1;
                                } else {
                                    current_actuator_index -= 1;
                                }
                                let new_actuator_id = motor_state.available_actuators[current_actuator_index];
                                if new_actuator_id != actuator_id {
                                    // Disable current actuator
                                    let _ = supervisor.disable(actuator_id, false).await;
                                    
                                    // Switch to new actuator
                                    actuator_id = new_actuator_id;
                                    
                                    // Add and enable new actuator if not already added
                                    supervisor.add_actuator(
                                        Box::new(RobStride00::new(
                                            actuator_id,
                                            0xFE,
                                            supervisor.get_transport_tx("socketcan").await.unwrap(),
                                        )),
                                        ActuatorConfiguration {
                                            actuator_type: ActuatorType::RobStride00,
                                            ..Default::default()
                                        },
                                    ).await;
                                    
                                    let cfg = ControlConfig {
                                        kp: 24.0,
                                        kd: 0.6,
                                        max_torque: Some(100.0),
                                        max_velocity: Some(50.0),
                                        max_current: Some(10.0),
                                    };
                                    let _ = supervisor.configure(actuator_id, cfg).await;
                                    let _ = supervisor.enable(actuator_id).await;
                                    
                                    // Reset motor state for new actuator
                                    motor_state.target_position = 0.0;
                                    motor_state.feedback_received = false;
                                    motor_state.command_count = 0;
                                    motor_state.feedback_count = 0;
                                    
                                    info!("Switched to actuator {}", actuator_id);
                                }
                            }
                        }
                        KeyCode::Right => {
                            // Switch to next actuator
                            if !motor_state.available_actuators.is_empty() {
                                current_actuator_index = (current_actuator_index + 1) % motor_state.available_actuators.len();
                                let new_actuator_id = motor_state.available_actuators[current_actuator_index];
                                if new_actuator_id != actuator_id {
                                    // Disable current actuator
                                    let _ = supervisor.disable(actuator_id, false).await;
                                    
                                    // Switch to new actuator
                                    actuator_id = new_actuator_id;
                                    
                                    // Add and enable new actuator if not already added
                                    supervisor.add_actuator(
                                        Box::new(RobStride00::new(
                                            actuator_id,
                                            0xFE,
                                            supervisor.get_transport_tx("socketcan").await.unwrap(),
                                        )),
                                        ActuatorConfiguration {
                                            actuator_type: ActuatorType::RobStride00,
                                            ..Default::default()
                                        },
                                    ).await;
                                    
                                    let cfg = ControlConfig {
                                        kp: 24.0,
                                        kd: 0.6,
                                        max_torque: Some(100.0),
                                        max_velocity: Some(50.0),
                                        max_current: Some(10.0),
                                    };
                                    let _ = supervisor.configure(actuator_id, cfg).await;
                                    let _ = supervisor.enable(actuator_id).await;
                                    
                                    // Reset motor state for new actuator
                                    motor_state.target_position = 0.0;
                                    motor_state.feedback_received = false;
                                    motor_state.command_count = 0;
                                    motor_state.feedback_count = 0;
                                    
                                    info!("Switched to actuator {}", actuator_id);
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
        }

        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    Ok(())
}
