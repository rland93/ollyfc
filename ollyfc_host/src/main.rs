use crossterm::{
    event::{self, KeyCode, KeyEvent, KeyModifiers},
    execute,
    terminal::{self, EnterAlternateScreen, LeaveAlternateScreen},
};
use std::io;
use tui::{
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Style},
    text::{Span, Spans},
    widgets::{Block, Borders, List, ListItem, Paragraph},
    Frame, Terminal,
};
use usb::FcUsbDevice;

mod usb;

enum AppState {
    Searching,
    Interacting(usb::FcUsbDevice),
}

fn main() -> Result<(), io::Error> {
    // initialize
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;
    let mut app_state = AppState::Searching;

    execute!(std::io::stdout(), terminal::Clear(terminal::ClearType::All))?;
    execute!(std::io::stdout(), EnterAlternateScreen)?;

    // primary loop
    loop {
        terminal
            .draw(|f: &mut Frame<'_, CrosstermBackend<io::Stdout>>| {
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .margin(1)
                    .constraints(
                        [
                            Constraint::Min(3),    // Main area
                            Constraint::Length(3), // Control bar
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
                        let block = Block::default().borders(Borders::ALL).title("Status");
                        f.render_widget(block, chunks[0]);
                    }
                }
                let exit_text = Span::styled("Exit (CTRL+C)", Style::default().fg(Color::Red));
                let control_bar = Paragraph::new(Spans::from(exit_text))
                    .block(Block::default().borders(Borders::ALL));
                f.render_widget(control_bar, chunks[1]);
            })
            .unwrap();

        // keyboard inputs
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

        // sleep a bit
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    // restore terminal after closed
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;

    Ok(())
}
