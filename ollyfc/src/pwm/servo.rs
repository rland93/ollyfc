/// Servo control structure for precise angle control.
pub struct Servo {
    min_pulse_width: u16,
    max_pulse_width: u16,
}

pub struct ServoPwmOut {
    pub on: u16,
    pub off: u16,
}

impl Servo {
    /// Creates a new `Servo` instance.
    ///
    /// # Parameters
    /// - `min_pulse_width`: Minimum pulse width count for 0 degrees.
    /// - `max_pulse_width`: Maximum pulse width count for 180 degrees.
    /// - `zero_offset`: Offset in counts to align the zero position.
    pub fn new(min_pulse_width: u16, max_pulse_width: u16) -> Self {
        Servo {
            min_pulse_width,
            max_pulse_width,
        }
    }

    /// Converts an angle to on and off pulse width counts.
    ///
    /// # Parameters
    /// - `angle`: Desired angle from 0 to 180 degrees.
    ///
    /// # Returns
    /// - Tuple of on and off pulse width counts.
    ///
    pub fn angle_to_counts(&self, angle: f32) -> ServoPwmOut {
        let angle = angle % 180.0;
        let scaled_angle = angle / 180.0;
        let pulse_range = self.max_pulse_width - self.min_pulse_width;
        let pulse_width = pulse_range as f32 * scaled_angle;
        let pulse_width = (pulse_width + self.min_pulse_width as f32) as u16;
        let on_count = 0;
        let off_count = on_count + pulse_width;

        ServoPwmOut {
            on: on_count,
            off: off_count,
        }
    }
}
