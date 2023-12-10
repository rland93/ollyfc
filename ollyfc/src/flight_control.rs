use defmt::Format;

#[derive(Debug, Clone, Copy, Format)]
pub struct ControlPolicy {
    pub elevator: u16,
    pub aileron: u16,
    pub rudder: u16,
    pub throttle: u16,
}

impl ControlPolicy {
    pub fn default() -> Self {
        Self {
            elevator: 0,
            aileron: 0,
            rudder: 0,
            throttle: 0,
        }
    }
}
