/// Servo control structure for precise angle control.
pub struct Servo {
    min_pulse_width: u16,
    max_pulse_width: u16,
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
    pub fn angle_to_counts(&self, angle: f32) -> (u16, u16) {
        if angle < 0.0 || angle > 180.0 {
            panic!("Angle must be between 0 and 180 degrees");
        }

        let scaled_angle = angle / 180.0;
        let pulse_range = self.max_pulse_width - self.min_pulse_width;
        let pulse_width = pulse_range as f32 * scaled_angle;
        let pulse_width = (pulse_width + self.min_pulse_width as f32) as u16;
        let on = 0;
        let off = on + pulse_width;

        (on, off)
    }
}
