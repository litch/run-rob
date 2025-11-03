use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use ratatui::{
    Terminal,
    backend::CrosstermBackend,
    layout::{Alignment, Constraint, Direction, Layout},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph},
};
use serde::{Deserialize, Serialize};
use std::io;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::RwLock;
use tracing::{error, info};
use tracing_subscriber::EnvFilter;
use zenoh::pubsub::Publisher;

// TUI-specific state with increment logic
#[derive(Debug, Clone)]
struct TuiState {
    yaw_target: f32,
    pitch_target: f32,
    increment: f32,
    increment_index: usize,
    corner_cycle_index: usize,
    corner_cycle_origin: (f32, f32), // Store the origin when cycle starts
}

impl TuiState {
    fn new() -> Self {
        Self {
            yaw_target: 0.0,
            pitch_target: 0.0,
            increment: 0.1,
            increment_index: 2,
            corner_cycle_index: 0,
            corner_cycle_origin: (0.0, 0.0),
        }
    }

    fn get_increments() -> Vec<f32> {
        vec![0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1.0]
    }

    fn cycle_increment(&mut self) {
        let increments = Self::get_increments();
        self.increment_index = (self.increment_index + 1) % increments.len();
        self.increment = increments[self.increment_index];
    }

    fn get_corner_offsets() -> [(f32, f32); 4] {
        // (yaw_offset, pitch_offset) in radians
        // 20 degrees = ~0.349 rad, 10 degrees = ~0.175 rad
        let yaw_offset = 20.0_f32.to_radians();
        let pitch_offset = 10.0_f32.to_radians();
        [
            (yaw_offset, pitch_offset),    // +20 yaw, +10 pitch
            (-yaw_offset, pitch_offset),   // -20 yaw, +10 pitch
            (yaw_offset, -pitch_offset),   // +20 yaw, -10 pitch
            (-yaw_offset, -pitch_offset),  // -20 yaw, -10 pitch
        ]
    }

    fn next_corner(&mut self) -> (f32, f32) {
        let corners = Self::get_corner_offsets();
        let (yaw_offset, pitch_offset) = corners[self.corner_cycle_index];
        self.corner_cycle_index = (self.corner_cycle_index + 1) % corners.len();
        
        let new_yaw = self.corner_cycle_origin.0 + yaw_offset;
        let new_pitch = self.corner_cycle_origin.1 + pitch_offset;
        (new_yaw, new_pitch)
    }

    fn start_corner_cycle(&mut self) {
        // Store current position as origin and reset cycle
        self.corner_cycle_origin = (self.yaw_target, self.pitch_target);
        self.corner_cycle_index = 0;
    }
}

// Messages from bus
#[derive(Debug, Clone, Serialize, Deserialize)]
struct GimbalState {
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

// Commands to bus
#[derive(Debug, Clone, Serialize, Deserialize)]
enum GimbalCoreCommand {
    Slew(SlewCommand),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SlewCommand {
    pub yaw: f32,
    pub pitch: f32,
    pub timestamp_ms: u64,
}

impl SlewCommand {
    fn new(yaw: f32, pitch: f32) -> Self {
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

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // Initialize logging to file
    let file_appender = tracing_appender::rolling::daily("./logs", "tui.log");
    let (non_blocking, _guard) = tracing_appender::non_blocking(file_appender);

    tracing_subscriber::fmt()
        .with_writer(non_blocking)
        .with_env_filter(
            EnvFilter::from_default_env()
                .add_directive("polling=off".parse().unwrap())
                .add_directive("async_io=off".parse().unwrap()),
        )
        .init();

    info!("Starting Gimbal TUI");

    // Connect to Zenoh
    let zenoh_config = zenoh::Config::default();
    let session = Arc::new(
        zenoh::open(zenoh_config)
            .await
            .map_err(|e| eyre::eyre!("Failed to connect to Zenoh: {:?}", e))?,
    );
    info!("Connected to Zenoh");

    let namespace = std::env::var("ZENOH_NAMESPACE").unwrap_or_else(|_| "gimbal".to_string());
    let state_topic = format!("{}/gimbal_core/state", namespace);
    let command_topic = format!("{}/gimbal_core/command", namespace);

    info!("Subscribing to state: {}", state_topic);
    info!("Publishing commands to: {}", command_topic);

    // Subscribe to state updates
    let subscriber = session
        .declare_subscriber(state_topic)
        .await
        .map_err(|e| eyre::eyre!("Failed to subscribe: {:?}", e))?;
    let command_publisher = session
        .declare_publisher(command_topic)
        .await
        .map_err(|e| eyre::eyre!("Failed to create publisher: {:?}", e))?;

    // Shared state
    let gimbal_state = Arc::new(RwLock::new(None::<GimbalState>));
    let last_update = Arc::new(RwLock::new(Instant::now()));

    // Spawn state receiver
    let gimbal_state_clone = gimbal_state.clone();
    let last_update_clone = last_update.clone();
    tokio::spawn(async move {
        loop {
            match subscriber.recv_async().await {
                Ok(sample) => {
                    let payload = sample.payload().to_bytes();
                    match serde_json::from_slice::<GimbalState>(&payload) {
                        Ok(state) => {
                            *gimbal_state_clone.write().await = Some(state);
                            *last_update_clone.write().await = Instant::now();
                        }
                        Err(e) => {
                            error!("Failed to deserialize state: {}", e);
                        }
                    }
                }
                Err(e) => {
                    error!("Failed to receive state: {}", e);
                }
            }
        }
    });

    // Setup terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let result = run_tui(
        &mut terminal,
        gimbal_state,
        last_update,
        command_publisher,
    )
    .await;

    // Restore terminal
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;

    result
}

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    gimbal_state: Arc<RwLock<Option<GimbalState>>>,
    last_update: Arc<RwLock<Instant>>,
    command_publisher: Publisher<'static>,
) -> eyre::Result<()> {
    let mut tui_state = TuiState::new();

    loop {
        // Get current state
        let state = gimbal_state.read().await.clone();
        let update_age = last_update.read().await.elapsed();

        // Draw UI
        terminal.draw(|f| {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .margin(2)
                .constraints([
                    Constraint::Length(3),
                    Constraint::Length(6),
                    Constraint::Min(15),
                    Constraint::Length(8),
                    Constraint::Length(7),
                ])
                .split(f.area());

            // Title
            let title = Paragraph::new("Gimbal Controller (Zenoh Client)")
                .style(
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                )
                .alignment(Alignment::Center)
                .block(Block::default().borders(Borders::ALL));
            f.render_widget(title, chunks[0]);

            // Connection Status
            let connection_status = if state.is_some() && update_age < Duration::from_secs(1) {
                ("CONNECTED", Color::Green)
            } else {
                ("DISCONNECTED", Color::Red)
            };

            let connection_text = vec![
                Line::from(vec![
                    Span::styled("Bus Connection: ", Style::default().fg(Color::Yellow)),
                    Span::styled(
                        connection_status.0,
                        Style::default()
                            .fg(connection_status.1)
                            .add_modifier(Modifier::BOLD),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("Last Update: ", Style::default().fg(Color::Yellow)),
                    Span::styled(
                        format!("{}ms ago", update_age.as_millis()),
                        Style::default().fg(if update_age.as_millis() < 100 {
                            Color::Green
                        } else if update_age.as_millis() < 500 {
                            Color::Yellow
                        } else {
                            Color::Red
                        }),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("Target: ", Style::default().fg(Color::Yellow)),
                    Span::styled(
                        format!("Yaw: {:.3}, Pitch: {:.3}", tui_state.yaw_target, tui_state.pitch_target),
                        Style::default().fg(Color::Cyan),
                    ),
                ]),
            ];

            let connection = Paragraph::new(connection_text).block(
                Block::default()
                    .borders(Borders::ALL)
                    .title("Status"),
            );
            f.render_widget(connection, chunks[1]);

            // Motor Status - Split into two columns
            let motor_columns = Layout::default()
                .direction(Direction::Horizontal)
                .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
                .split(chunks[2]);

            // Yaw Motor
            let yaw_text = if let Some(ref s) = state {
                let temp_color = if s.yaw_temperature > 60.0 {
                    Color::Red
                } else if s.yaw_temperature > 50.0 {
                    Color::Yellow
                } else {
                    Color::Green
                };

                vec![
                    Line::from(vec![
                        Span::styled("Position: ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} rad", s.yaw_position),
                            Style::default().fg(Color::White),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("Target:   ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} rad", tui_state.yaw_target),
                            Style::default().fg(Color::Green),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("Error:    ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} rad", tui_state.yaw_target - s.yaw_position),
                            Style::default().fg(Color::Red),
                        ),
                    ]),
                    Line::from(""),
                    Line::from(vec![
                        Span::styled("Velocity: ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} rad/s", s.yaw_velocity),
                            Style::default().fg(Color::White),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("Torque:   ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} Nm", s.yaw_torque),
                            Style::default().fg(Color::White),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("Temp:     ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:4.1} °C", s.yaw_temperature),
                            Style::default().fg(temp_color),
                        ),
                    ]),
                ]
            } else {
                vec![Line::from("Waiting for data...")]
            };

            let yaw_widget = Paragraph::new(yaw_text).block(
                Block::default()
                    .borders(Borders::ALL)
                    .title("YAW Motor (←/→)"),
            );
            f.render_widget(yaw_widget, motor_columns[0]);

            // Pitch Motor
            let pitch_text = if let Some(ref s) = state {
                let temp_color = if s.pitch_temperature > 60.0 {
                    Color::Red
                } else if s.pitch_temperature > 50.0 {
                    Color::Yellow
                } else {
                    Color::Green
                };

                vec![
                    Line::from(vec![
                        Span::styled("Position: ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} rad", s.pitch_position),
                            Style::default().fg(Color::White),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("Target:   ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} rad", tui_state.pitch_target),
                            Style::default().fg(Color::Green),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("Error:    ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} rad", tui_state.pitch_target - s.pitch_position),
                            Style::default().fg(Color::Red),
                        ),
                    ]),
                    Line::from(""),
                    Line::from(vec![
                        Span::styled("Velocity: ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} rad/s", s.pitch_velocity),
                            Style::default().fg(Color::White),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("Torque:   ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:6.3} Nm", s.pitch_torque),
                            Style::default().fg(Color::White),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("Temp:     ", Style::default().fg(Color::Yellow)),
                        Span::styled(
                            format!("{:4.1} °C", s.pitch_temperature),
                            Style::default().fg(temp_color),
                        ),
                    ]),
                ]
            } else {
                vec![Line::from("Waiting for data...")]
            };

            let pitch_widget = Paragraph::new(pitch_text).block(
                Block::default()
                    .borders(Borders::ALL)
                    .title("PITCH Motor (↑/↓)"),
            );
            f.render_widget(pitch_widget, motor_columns[1]);

            // Available Increments
            let increments = TuiState::get_increments();
            let increment_items: Vec<ListItem> = increments
                .iter()
                .enumerate()
                .map(|(i, inc)| {
                    let style = if i == tui_state.increment_index {
                        Style::default()
                            .fg(Color::Green)
                            .add_modifier(Modifier::BOLD)
                    } else {
                        Style::default().fg(Color::Gray)
                    };
                    let marker = if i == tui_state.increment_index {
                        "→ "
                    } else {
                        "  "
                    };
                    ListItem::new(format!("{}{:.4} rad", marker, inc)).style(style)
                })
                .collect();

            let increments_list = List::new(increment_items).block(
                Block::default()
                    .borders(Borders::ALL)
                    .title("Available Increments (Tab to cycle)"),
            );
            f.render_widget(increments_list, chunks[3]);

            // Controls
            let controls_text = vec![
                Line::from(vec![
                    Span::styled(
                        "↑/↓",
                        Style::default()
                            .fg(Color::Cyan)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::raw("  Control PITCH motor  "),
                    Span::styled(
                        "←/→",
                        Style::default()
                            .fg(Color::Cyan)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::raw("  Control YAW motor"),
                ]),
                Line::from(vec![
                    Span::styled(
                        "Tab",
                        Style::default()
                            .fg(Color::Cyan)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::raw("  Cycle increment    "),
                    Span::styled(
                        "Z",
                        Style::default()
                            .fg(Color::Cyan)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::raw("    Return to zero"),
                ]),
                Line::from(vec![
                    Span::styled(
                        "X",
                        Style::default()
                            .fg(Color::Cyan)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::raw("    Cycle corners (±20°/±10° pattern)"),
                ]),
                Line::from(vec![
                    Span::styled(
                        "Q/Esc",
                        Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
                    ),
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
                            tui_state.pitch_target += tui_state.increment;
                            send_command(&command_publisher, tui_state.yaw_target, tui_state.pitch_target).await?;
                            info!("Pitch target: {:.4}", tui_state.pitch_target);
                        }
                        KeyCode::Down => {
                            tui_state.pitch_target -= tui_state.increment;
                            send_command(&command_publisher, tui_state.yaw_target, tui_state.pitch_target).await?;
                            info!("Pitch target: {:.4}", tui_state.pitch_target);
                        }
                        KeyCode::Left => {
                            tui_state.yaw_target -= tui_state.increment;
                            send_command(&command_publisher, tui_state.yaw_target, tui_state.pitch_target).await?;
                            info!("Yaw target: {:.4}", tui_state.yaw_target);
                        }
                        KeyCode::Right => {
                            tui_state.yaw_target += tui_state.increment;
                            send_command(&command_publisher, tui_state.yaw_target, tui_state.pitch_target).await?;
                            info!("Yaw target: {:.4}", tui_state.yaw_target);
                        }
                        KeyCode::Tab => {
                            tui_state.cycle_increment();
                        }
                        KeyCode::Char('z') | KeyCode::Char('Z') => {
                            tui_state.yaw_target = 0.0;
                            tui_state.pitch_target = 0.0;
                            send_command(&command_publisher, 0.0, 0.0).await?;
                            info!("Returning to zero");
                        }
                        KeyCode::Char('x') | KeyCode::Char('X') => {
                            // First press starts the cycle from current position
                            if tui_state.corner_cycle_index == 0 {
                                tui_state.start_corner_cycle();
                                info!("Starting corner cycle from yaw={:.3}, pitch={:.3}", 
                                      tui_state.corner_cycle_origin.0, tui_state.corner_cycle_origin.1);
                            }
                            
                            let (new_yaw, new_pitch) = tui_state.next_corner();
                            tui_state.yaw_target = new_yaw;
                            tui_state.pitch_target = new_pitch;
                            send_command(&command_publisher, new_yaw, new_pitch).await?;
                            info!("Corner cycle {}/4: yaw={:.3}, pitch={:.3}", 
                                  tui_state.corner_cycle_index, new_yaw, new_pitch);
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

async fn send_command(publisher: &Publisher<'static>, yaw: f32, pitch: f32) -> eyre::Result<()> {
    let cmd = GimbalCoreCommand::Slew(SlewCommand::new(yaw, pitch));
    let json = serde_json::to_string(&cmd)?;
    publisher
        .put(json)
        .await
        .map_err(|e| eyre::eyre!("Failed to send command: {:?}", e))?;
    Ok(())
}
