use csv::WriterBuilder;
use ollyfc_common::FlightLogData;
use std::error::Error;

pub fn export_to_csv(flight_logs: &[FlightLogData]) -> Result<String, Box<dyn Error>> {
    let mut wtr = WriterBuilder::new().from_writer(vec![]);

    wtr.write_record(&[
        "timestamp",
        "accel_x",
        "accel_y",
        "accel_z",
        "pitch",
        "yaw",
        "roll",
        "throttle",
        "aileron",
        "elevator",
        "rudder",
        "arm",
        "enable",
        "record",
        "control_elevator",
        "control_aileron",
        "control_rudder",
        "control_throttle",
    ])?;

    // Write each record
    for log in flight_logs {
        wtr.write_record(&[
            log.timestamp.to_string(),
            log.sensor_input.accel_x.to_string(),
            log.sensor_input.accel_y.to_string(),
            log.sensor_input.accel_z.to_string(),
            log.sensor_input.pitch.to_string(),
            log.sensor_input.yaw.to_string(),
            log.sensor_input.roll.to_string(),
            log.sbus_input.throttle.to_string(),
            log.sbus_input.aileron.to_string(),
            log.sbus_input.elevator.to_string(),
            log.sbus_input.rudder.to_string(),
            log.sbus_input.arm.to_string(),
            log.sbus_input.enable.to_string(),
            log.sbus_input.record.to_string(),
            log.control_policy.ctl_elevator.to_string(),
            log.control_policy.ctl_aileron.to_string(),
            log.control_policy.ctl_rudder.to_string(),
            log.control_policy.ctl_throttle.to_string(),
        ])?;
    }

    wtr.flush()?;
    Ok(String::from_utf8(wtr.into_inner()?)?)
}
