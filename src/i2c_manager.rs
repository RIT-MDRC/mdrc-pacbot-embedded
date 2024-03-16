use crate::vl6180x::VL6180X;
use crate::Irqs;
use defmt::info;
use embassy_rp::gpio::{AnyPin, Level, Output};
use embassy_rp::i2c::{SclPin, SdaPin};
use embassy_rp::peripherals::I2C0;
use embassy_rp::{i2c, Peripheral};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::Timer;

pub const NUM_DISTANCE_SENSORS: usize = 8;

#[embassy_executor::task]
pub async fn run_i2c(
    peri: impl Peripheral<P = I2C0> + 'static,
    scl: impl Peripheral<P = impl SclPin<I2C0>> + 'static,
    sda: impl Peripheral<P = impl SdaPin<I2C0>> + 'static,
    x_shut: [AnyPin; NUM_DISTANCE_SENSORS],
    sender: Sender<'static, ThreadModeRawMutex, [u8; NUM_DISTANCE_SENSORS], 64>,
) {
    info!("i2c task started");

    let mut i2c = i2c::I2c::new_async(peri, scl, sda, Irqs, i2c::Config::default());

    const SENSOR_NONE: Option<VL6180X> = None;
    let mut sensors: [Option<VL6180X>; NUM_DISTANCE_SENSORS] = [SENSOR_NONE; NUM_DISTANCE_SENSORS];
    let mut x_shut = x_shut.map(|p| Output::new(p, Level::Low));

    let addresses: [u8; NUM_DISTANCE_SENSORS] = [0x29, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37];
    let mut old_range = [0; NUM_DISTANCE_SENSORS];

    loop {
        // fetch new values
        let mut changed = false;
        for i in 1..NUM_DISTANCE_SENSORS {
            let mut sensor_error = false;
            if let Some(sensor) = &mut sensors[i] {
                if let Ok(range) = sensor.range_from_history(&mut i2c).await {
                    // info!("range {} {}", range, sensor.continuous_mode_enabled(&mut i2c).await);
                    if range != old_range[i] {
                        old_range[i] = range;
                        changed = true;
                    }
                    if range == 0 {
                        info!(
                            "Range status error {:?}",
                            sensor.range_status(&mut i2c).await
                        );
                    }
                } else {
                    sensor_error = true;
                }
            } else {
                // the sensor isn't connected - try to connect to it
                info!("Trying to connect to sensor {}", i);
                x_shut[i].set_high();
                info!("Sensor {} set high", i);
                Timer::after_millis(300).await;
                if let Err(e) = VL6180X::new(&mut i2c, 0x29).await {
                    info!("{:?}", e);
                }
                // info!("{:?}", VL6180X::new(&mut i2c, 0x29).await);
                if let Ok(mut sensor) = VL6180X::new(&mut i2c, 0x29).await {
                    // sensor.set_address(&mut i2c, addresses[i]).await;
                    if let Ok(_) = sensor.set_address(&mut i2c, addresses[i]).await {
                        Timer::after_millis(300).await;
                        if let Ok(sensor) = VL6180X::new(&mut i2c, addresses[i]).await {
                            Timer::after_millis(300).await;
                            if let Ok(_) = sensor.start_range_continuous(&mut i2c, 1).await {
                                sensors[i] = Some(sensor)
                            } else {
                                info!("start cont err");
                                sensor_error = true;
                            }
                        } else {
                            info!("remake device err");
                            sensor_error = true;
                        }
                    } else {
                        info!("set addr err");
                        if let Err(e) = sensor.set_address(&mut i2c, addresses[i]).await {
                            info!("{:?}", e);
                        }
                        sensor_error = true;
                    }
                } else {
                    info!("make sensor err");
                    // something went wrong - in case this sensor wakes up, force it to shut down for now
                    sensor_error = true;
                }
            }
            if sensor_error {
                x_shut[i].set_low();
                old_range[i] = 0;
                sensors[i] = None;
                changed = true;
            }
        }
        if changed {
            sender.send(old_range).await;
        }
        Timer::after_millis(1).await;
    }
}
