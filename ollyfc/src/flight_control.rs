use defmt::info;
use rtic::Mutex;
use rtic_monotonics::{
    systick::{ExtU32, Systick},
    Monotonic,
};

use ollyfc_common::{ControlPolicy, FlightLogData, SBusInput, SensorInput};

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
enum SwitchMode {
    Low,
    Neutral,
    High,
}

fn switch_mode(switch_input: u16) -> SwitchMode {
    if switch_input < 500 {
        SwitchMode::High
    } else if switch_input > 1500 {
        SwitchMode::Low
    } else {
        SwitchMode::Neutral
    }
}

fn elevator_ctl(ele: u16, en: SwitchMode) -> u16 {
    if en == SwitchMode::High {
        ele + 200
    } else {
        ele
    }
}

fn aileron_ctl(ail: u16) -> u16 {
    ail
}

fn rudder_ctl(rud: u16) -> u16 {
    rud
}

fn throttle_ctl(thr: u16) -> u16 {
    thr
}

fn scale_servo(elevator: u16, duty_max: u16) -> u16 {
    let min_pulse_width = 900; // 900 μs
    let max_pulse_width = 2100; // 2100 μs
    let pulse_width_range = max_pulse_width - min_pulse_width;
    let servo_pulse_width =
        min_pulse_width + (elevator as f32 / 2048.0 * pulse_width_range as f32) as u16;
    let pwm_duty = (servo_pulse_width as f32 / 20000.0 * duty_max as f32) as u16;
    return pwm_duty;
}

pub async fn flight_loop(
    mut cx: crate::app::primary_flight_loop_task::Context<'_>,
    mut log_ch_s: rtic_sync::channel::Sender<'static, FlightLogData, { crate::LOGDATA_CHAN_SIZE }>,
) {
    loop {
        let now = rtic_monotonics::systick::Systick::now();
        // Flight Controls
        let controls = cx
            .shared
            .flight_controls
            .lock(|fc: &mut crate::sbus::FlightControls| fc.clone());
        let arm_mode = switch_mode(controls.arm);
        let enable_mode = switch_mode(controls.enable);
        let record_mode = switch_mode(controls.record);

        info!(
            "rec: {}, arm: {}, en: {}",
            record_mode, arm_mode, enable_mode
        );
        // Sensor
        let gyro = cx.shared.sensor.lock(|g: &mut SensorInput| g.clone());
        #[cfg(feature = "no-sensors")]
        let gyro = SensorInput::default();

        // control policies
        let ele = elevator_ctl(controls.elevator, arm_mode);
        let ail = aileron_ctl(controls.aileron);
        let rud = rudder_ctl(controls.rudder);
        let thr = throttle_ctl(controls.throttle);

        // Send data to logger
        let log_data = FlightLogData {
            timestamp: Systick::now().ticks(),
            sbus_input: SBusInput {
                throttle: controls.throttle,
                aileron: controls.aileron,
                elevator: controls.elevator,
                rudder: controls.rudder,
                arm: arm_mode as u16,
                enable: enable_mode as u16,
                record: record_mode as u16,
            },
            sensor_input: SensorInput {
                pitch: gyro.pitch,
                yaw: gyro.yaw,
                roll: gyro.roll,
                accel_x: gyro.accel_x,
                accel_y: gyro.accel_y,
                accel_z: gyro.accel_z,
            },
            control_policy: ControlPolicy {
                ctl_elevator: ele,
                ctl_aileron: ail,
                ctl_rudder: rud,
                ctl_throttle: thr,
            },
        };
        if record_mode == SwitchMode::High {
            info!("Sending log data");
        }
        log_ch_s
            .send(log_data)
            .await
            .expect("Error sending log data");

        let duty = scale_servo(ele, cx.local.elevator_channel.get_max_duty());
        cx.local.elevator_channel.set_duty(duty);

        rtic_monotonics::systick::Systick::delay_until(now + 20u32.millis()).await;
    }
}
