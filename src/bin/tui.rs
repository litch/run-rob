use run_rob::{
    MotorState, ControlPresets, scan_actuators, update_motor_state, 
    send_position_command, apply_control_config,
};
use robstride::{
    robstride00::RobStride00, ActuatorConfiguration, ActuatorType, 
    SocketCanTransport, Supervisor, TransportType
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
    let available_actuators = scan_actuators(&mut supervisor).await?;
    
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
    let cfg = ControlPresets::normal_mode();
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
    actuator_id: u8,
) -> eyre::Result<()> {
    let mut last_update = std::time::Instant::now();
    let mut current_lock_mode = motor_state.lock_mode;
    
    // Track motor states for both motors
    let mut motor_states: Vec<MotorState> = vec![MotorState::new(), MotorState::new()];
    let mut motors: Vec<u8> = Vec::new();
    
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
                motor_state.available_actuators = found_actuators.clone();
                motor_state.last_scan_time = Some(std::time::Instant::now());
                info!("Scanned bus, found {} actuators: {:?}", motor_state.available_actuators.len(), motor_state.available_actuators);
                
                // Initialize motors array if we have actuators
                if motors.is_empty() && found_actuators.len() >= 2 {
                    motors = found_actuators;
                    
                    // Initialize both motors
                    for (idx, &motor_id) in motors.iter().enumerate().take(2) {
                        supervisor.add_actuator(
                            Box::new(RobStride00::new(
                                motor_id,
                                0xFE,
                                supervisor.get_transport_tx("socketcan").await?,
                            )),
                            ActuatorConfiguration {
                                actuator_type: ActuatorType::RobStride00,
                                ..Default::default()
                            },
                        ).await;
                        
                        let cfg = ControlPresets::normal_mode();
                        supervisor.configure(motor_id, cfg.clone()).await?;
                        supervisor.enable(motor_id).await?;
                        supervisor.zero(motor_id).await?;
                        
                        motor_states[idx].motor_enabled = true;
                        motor_states[idx].current_kp = cfg.kp;
                        motor_states[idx].current_kd = cfg.kd;
                        motor_states[idx].current_max_torque = cfg.max_torque.unwrap_or(0.0);
                        
                        info!("Initialized motor {} (motors[{}])", motor_id, idx);
                    }
                }
            }
        }
        
        // Check if lock mode changed and update motor configuration
        if current_lock_mode != motor_state.lock_mode {
            apply_control_config(supervisor, motor_state, actuator_id).await?;
            current_lock_mode = motor_state.lock_mode;
        }
        
        // Update motor feedback at regular intervals
        if last_update.elapsed() >= Duration::from_millis(50) && motors.len() >= 2 {
            // Update both motors
            for (idx, &motor_id) in motors.iter().enumerate().take(2) {
                update_motor_state(supervisor, &mut motor_states[idx], motor_id).await?;
                
                // Log feedback periodically
                if motor_states[idx].feedback_count % 20 == 0 {
                    info!("Motor {} (motors[{}]) Feedback #{}: angle={:.4}, vel={:.4}, torque={:.4}", 
                          motor_id, idx, motor_states[idx].feedback_count, 
                          motor_states[idx].current_position, 
                          motor_states[idx].current_velocity, 
                          motor_states[idx].current_torque);
                }
                
                // Send position command
                send_position_command(supervisor, &mut motor_states[idx], motor_id).await?;
            }
            
            // Update display state with tilt motor (motors[0]) info
            if motors.len() >= 1 {
                motor_state.current_position = motor_states[0].current_position;
                motor_state.current_velocity = motor_states[0].current_velocity;
                motor_state.current_torque = motor_states[0].current_torque;
                motor_state.current_temperature = motor_states[0].current_temperature;
                motor_state.feedback_received = motor_states[0].feedback_received;
                motor_state.feedback_count = motor_states[0].feedback_count;
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
                    Constraint::Min(15),
                    Constraint::Length(8),
                    Constraint::Length(7),
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

            // Motor Status - Split into two columns
            let motor_columns = Layout::default()
                .direction(Direction::Horizontal)
                .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
                .split(chunks[2]);
            
            // Helper function to create motor status text
            let create_motor_status = |state: &MotorState, motor_id: u8, _motor_name: &str| {
                let temp_color = if state.current_temperature > 60.0 {
                    Color::Red
                } else if state.current_temperature > 50.0 {
                    Color::Yellow
                } else {
                    Color::Green
                };
                
                let has_faults = state.fault_uncalibrated || state.fault_hall_encoding || 
                                state.fault_magnetic_encoding || state.fault_over_temperature ||
                                state.fault_overcurrent || state.fault_undervoltage;
                
                vec![
                    Line::from(vec![
                        Span::styled(format!("ID: {}", motor_id), Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    ]),
                    Line::from(vec![
                        Span::styled("Target:  ", Style::default().fg(Color::Yellow)),
                        Span::styled(format!("{:6.3} rad", state.target_position), Style::default().fg(Color::Green)),
                    ]),
                    Line::from(vec![
                        Span::styled("Current: ", Style::default().fg(Color::Yellow)),
                        Span::styled(format!("{:6.3} rad", state.current_position), Style::default().fg(Color::White)),
                    ]),
                    Line::from(vec![
                        Span::styled("Error:   ", Style::default().fg(Color::Yellow)),
                        Span::styled(format!("{:6.3} rad", state.target_position - state.current_position), Style::default().fg(Color::Red)),
                    ]),
                    Line::from(""),
                    Line::from(vec![
                        Span::styled("Vel:  ", Style::default().fg(Color::Yellow)),
                        Span::styled(format!("{:6.3} rad/s", state.current_velocity), Style::default().fg(Color::White)),
                    ]),
                    Line::from(vec![
                        Span::styled("Torq: ", Style::default().fg(Color::Yellow)),
                        Span::styled(format!("{:6.3} Nm", state.current_torque), Style::default().fg(Color::White)),
                    ]),
                    Line::from(vec![
                        Span::styled("Temp: ", Style::default().fg(Color::Yellow)),
                        Span::styled(format!("{:4.1} °C", state.current_temperature), Style::default().fg(temp_color)),
                    ]),
                    Line::from(""),
                    Line::from(vec![
                        Span::styled("Faults: ", Style::default().fg(Color::Yellow)),
                        Span::styled(if has_faults { "YES" } else { "None" }, 
                            Style::default().fg(if has_faults { Color::Red } else { Color::Green })),
                    ]),
                    Line::from(vec![
                        Span::styled("Feedback: ", Style::default().fg(Color::Yellow)),
                        Span::styled(format!("{}", state.feedback_count), Style::default().fg(Color::Cyan)),
                    ]),
                ]
            };
            
            // Render tilt motor (motors[0])
            if motors.len() >= 1 {
                let tilt_status = create_motor_status(&motor_states[0], motors[0], "TILT");
                let tilt_widget = Paragraph::new(tilt_status)
                    .block(Block::default().borders(Borders::ALL).title("TILT Motor (↑/↓)"));
                f.render_widget(tilt_widget, motor_columns[0]);
            }
            
            // Render pan motor (motors[1])
            if motors.len() >= 2 {
                let pan_status = create_motor_status(&motor_states[1], motors[1], "PAN");
                let pan_widget = Paragraph::new(pan_status)
                    .block(Block::default().borders(Borders::ALL).title("PAN Motor (←/→)"));
                f.render_widget(pan_widget, motor_columns[1]);
            }

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
                    Span::styled("↑/↓", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::raw("  Control TILT motor (motors[0])  "),
                ]),
                Line::from(vec![
                    Span::styled("←/→", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::raw("  Control PAN motor (motors[1])"),
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
                        KeyCode::Up => {
                            // Control tilt motor (motors[0])
                            if motors.len() >= 1 {
                                motor_states[0].increase_position();
                                info!("Tilt motor (motors[0], ID {}): target={:.4}", motors[0], motor_states[0].target_position);
                            }
                        }
                        KeyCode::Down => {
                            // Control tilt motor (motors[0])
                            if motors.len() >= 1 {
                                motor_states[0].decrease_position();
                                info!("Tilt motor (motors[0], ID {}): target={:.4}", motors[0], motor_states[0].target_position);
                            }
                        }
                        KeyCode::Left => {
                            // Control pan motor (motors[1])
                            if motors.len() >= 2 {
                                motor_states[1].decrease_position();
                                info!("Pan motor (motors[1], ID {}): target={:.4}", motors[1], motor_states[1].target_position);
                            }
                        }
                        KeyCode::Right => {
                            // Control pan motor (motors[1])
                            if motors.len() >= 2 {
                                motor_states[1].increase_position();
                                info!("Pan motor (motors[1], ID {}): target={:.4}", motors[1], motor_states[1].target_position);
                            }
                        }
                        KeyCode::Tab => {
                            // Cycle increment for both motors
                            motor_states[0].cycle_increment();
                            motor_states[1].cycle_increment();
                            motor_state.increment = motor_states[0].increment;
                            motor_state.increment_index = motor_states[0].increment_index;
                        }
                        KeyCode::Char('0') => {
                            // Set current position as zero for both motors
                            if motors.len() >= 2 {
                                let _ = supervisor.zero(motors[0]).await;
                                let _ = supervisor.zero(motors[1]).await;
                                motor_states[0].target_position = 0.0;
                                motor_states[1].target_position = 0.0;
                                info!("Set zero point for both motors");
                            }
                        }
                        KeyCode::Char('z') | KeyCode::Char('Z') => {
                            // Command both motors to return to zero position
                            motor_states[0].set_target_zero();
                            motor_states[1].set_target_zero();
                            info!("Returning both motors to zero");
                        }
                        KeyCode::Char('l') | KeyCode::Char('L') => {
                            motor_state.toggle_lock_mode();
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
