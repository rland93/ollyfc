use crossterm::{
    event::{self, KeyCode, KeyEvent, KeyModifiers},
    execute,
    terminal::{self, EnterAlternateScreen, LeaveAlternateScreen},
};
use std::io;
use tui::{
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout},
    style::{Color, Style},
    text::{Span, Spans},
    widgets::{Block, Borders, List, ListItem, Paragraph},
    Terminal,
};

mod usb;

enum AppState {
    Searching,
    Interacting(usb::FcUsbDevice), // Assuming FcUsbDevice is your device type
}

fn main() -> Result<(), io::Error> {
    // Initialize terminal and state
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;
    let mut app_state = AppState::Searching;

    loop {
        terminal
            .draw(|f| {
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .margin(1)
                    .constraints(
                        [
                            Constraint::Percentage(90), // Main area
                            Constraint::Percentage(10), // Control bar
                        ]
                        .as_ref(),
                    )
                    .split(f.size());

                match &app_state {
                    AppState::Searching => {
                        // Render Searching UI
                        let text = Paragraph::new("Searching for device...")
                            .block(Block::default().borders(Borders::ALL).title("Status"));
                        f.render_widget(text, chunks[0]);

                        // Search for device
                        if let Some(device) = usb::find_fc() {
                            app_state = AppState::Interacting(device);
                        }
                    }
                    AppState::Interacting(device) => {
                        // TODO: usb communications
                    }
                }
                let exit_text = Span::styled("Exit (CTRL+C)", Style::default().fg(Color::Red));
                let control_bar = Paragraph::new(Spans::from(exit_text))
                    .block(Block::default().borders(Borders::ALL));
                f.render_widget(control_bar, chunks[1]);
            })
            .unwrap();

        // Handle inputs
        if event::poll(std::time::Duration::from_millis(100))? {
            if let event::Event::Key(KeyEvent {
                code, modifiers, ..
            }) = event::read()?
            {
                match (code, modifiers) {
                    // Exit on CTRL+C or Esc
                    (KeyCode::Char('c'), KeyModifiers::CONTROL)
                    | (KeyCode::Esc, KeyModifiers::NONE) => {
                        break;
                    }
                    _ => {}
                }
            }
        }

        // Include a small sleep to reduce CPU usage
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    // Restore terminal
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;

    Ok(())
}
