#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
/// In this example, we configure the BMI088 IMU to store readings in the FIFO.
/// We poll the FIFO to read out new readings in bulk.
///
use defmt_rtt as _;
use embedded_hal_bus as ebus;
use panic_probe as _;
use stm32f4xx_hal::{gpio, pac, prelude::*, spi};

type GyroDev = ebus::spi::AtomicDevice<
    'static,
    spi::Spi<pac::SPI3>,
    gpio::Pin<'B', 5, gpio::Output>,
    ebus::spi::NoDelay,
>;
type AccelDev = ebus::spi::AtomicDevice<
    'static,
    spi::Spi<pac::SPI3>,
    gpio::Pin<'D', 2, gpio::Output>,
    ebus::spi::NoDelay,
>;
type Spi3Bus = ebus::util::AtomicCell<spi::Spi<pac::SPI3>>;
static mut SPI3BUS: Option<Spi3Bus> = None;

use rtic_monotonics::{systick_monotonic, Monotonic};
systick_monotonic!(Mono, 1000);

use bmi088::Bmi088;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [EXTI1])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        dev_accel: Bmi088<bmi088::interface::SpiInterface<AccelDev>>,
        dev_gyro: Bmi088<bmi088::interface::SpiInterface<GyroDev>>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let rcc = dp.RCC.constrain();
        let hse = 12.MHz();
        let sysclk = 64.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        Mono::start(cx.core.SYST, sysclk.to_Hz());

        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let gpiod = dp.GPIOD.split();

        defmt::debug!("spi setup");
        let dev_spi: spi::Spi3 = spi::Spi3::new(
            dp.SPI3,
            (
                gpioc.pc10.into_alternate(),
                gpioc.pc11.into_alternate(),
                gpioc.pc12.into_alternate(),
            ),
            spi::Mode {
                polarity: spi::Polarity::IdleLow,
                phase: spi::Phase::CaptureOnFirstTransition,
            },
            8.MHz(),
            &clocks,
        );
        let mut acc_cs = gpiod.pd2.into_push_pull_output();
        let mut gyr_cs = gpiob.pb5.into_push_pull_output();
        acc_cs.set_high();
        gyr_cs.set_high();

        let bus = unsafe {
            SPI3BUS = Some(ebus::util::AtomicCell::new(dev_spi));
            SPI3BUS.as_ref().unwrap()
        };

        let mut dev_accel =
            Bmi088::new_with_spi(ebus::spi::AtomicDevice::new_no_delay(&bus, acc_cs).unwrap());
        let mut dev_gyro =
            Bmi088::new_with_spi(ebus::spi::AtomicDevice::new_no_delay(&bus, gyr_cs).unwrap());

        defmt::info!(
            "accel: {:x}, gyro: {:x}",
            dev_accel.acc_chip_id_read().unwrap(),
            dev_gyro.gyro_chip_id_read().unwrap()
        );

        // configure
        let config = bmi088::AccConfiguration::builder()
            .acc_power_conf(bmi088::AccPowerConf::Active)
            .acc_power_ctrl(bmi088::AccPowerCtrl::On)
            .acc_bandwidth((bmi088::AccBandwidth::X4, bmi088::AccDataRate::Hz400))
            .acc_range(bmi088::AccRange::G3)
            // stream mode - discards old data from the queue
            .acc_fifo_mode(bmi088::AccFifoMode::Stream)
            // enable fifo
            .acc_fifo_conf1(bmi088::AccFifoConfig1 {
                acc_en: true,
                int1_input_en: false,
                int2_input_en: false,
            })
            // no downsampling
            .acc_fifo_downs(0)
            .build();

        dev_accel.configure_accelerometer(config).unwrap();

        let mode = dev_accel.acc_conf_read().unwrap();
        defmt::debug!("mode CONF: {:?}", defmt::Debug2Format(&mode));
        let fifoconfig1 = dev_accel.acc_fifo_config1_read().unwrap();
        defmt::debug!("config1 CONF: {:?}", defmt::Debug2Format(&fifoconfig1));
        let downs = dev_accel.acc_fifo_downs_read().unwrap();
        defmt::debug!("downs CONF: {:?}", defmt::Debug2Format(&downs));
        let fifomode = dev_accel.acc_fifo_mode_read().unwrap();
        defmt::debug!("config0 CONF: {:?}", defmt::Debug2Format(&fifomode));
        let wtm = dev_accel.acc_fifo_wtm_read().unwrap();
        defmt::debug!("wtm CONF: {:?}", defmt::Debug2Format(&wtm));
        let int1 = dev_accel.acc_int1_io_ctrl_read().unwrap();
        defmt::debug!("int1 CONF: {:?}", defmt::Debug2Format(&int1));
        let int2 = dev_accel.acc_int2_io_ctrl_read().unwrap();
        defmt::debug!("int2 CONF: {:?}", defmt::Debug2Format(&int2));
        let int_map = dev_accel.acc_int1_int2_map_data_read().unwrap();
        defmt::debug!("int_map CONF: {:?}", defmt::Debug2Format(&int_map));
        let pwr_conf = dev_accel.acc_pwr_conf_read().unwrap();
        defmt::debug!("pwr_conf CONF: {:?}", defmt::Debug2Format(&pwr_conf));
        let pwr_ctrl = dev_accel.acc_pwr_ctrl_read().unwrap();
        defmt::debug!("pwr_ctrl CONF: {:?}", defmt::Debug2Format(&pwr_ctrl));
        let range = dev_accel.acc_range_read().unwrap();
        defmt::debug!("range CONF: {:?}", defmt::Debug2Format(&range));

        defmt::info!("setup done");

        sensor_process::spawn().ok();

        (
            Shared {
                dev_accel,
                dev_gyro,
            },
            Local {},
        )
    }

    // polling task to read out the FIFO.
    #[task(priority=1, shared=[dev_accel, dev_gyro])]
    async fn sensor_process(mut cx: sensor_process::Context) {
        loop {
            let now = Mono::now();

            // burst read into the queue
            let mut buf = [0u8; 1024];
            cx.shared.dev_accel.lock(|a| {
                a.acc_fifo_data(&mut buf).unwrap();
            });

            let mut header: bmi088::AccFifoFrameHeader;
            let mut cursor = 0;
            let mut frames_read: usize = 0;
            while cursor + 7 < buf.len() && frames_read < 40 {
                header = bmi088::AccFifoFrameHeader::from(buf[cursor]);

                match header {
                    bmi088::AccFifoFrameHeader::Acceleration(_tags) => {
                        frames_read += 1;
                        let d = bmi088::Sensor3DData::from_le_slice(
                            buf[cursor + 1..cursor + 7].try_into().unwrap(),
                        );
                        defmt::debug!("[{},{},{}]", d.x, d.y, d.z);
                    }
                    bmi088::AccFifoFrameHeader::End => {
                        break;
                    }
                    _ => {}
                }

                let n = header.bytes_to_read();
                cursor += n + 1;
            }
            defmt::warn!("frames read: {}", frames_read);

            // Adjust based on the number of frames read out and the sample
            // rate. At 400Hz, we have 2.5ms per sample, therefore we should
            // see 10 frames read out every time this loop runs.
            Mono::delay_until(now + 25.millis()).await;
        }
    }

    #[idle(shared=[dev_accel])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            let len = cx
                .shared
                .dev_accel
                .lock(|a| a.acc_fifo_length_read().unwrap());
            defmt::info!("len: {}", len);
            Mono::delay_ms(&mut Mono, 10);
            cortex_m::asm::nop();
        }
    }
}
