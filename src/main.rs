//! Template

#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

mod config;
mod i2c_manager;
mod motors;
mod vl6180x;
mod wifi;

use cyw43::Control;

use crate::config::PacbotConfig;
use crate::i2c_manager::{run_i2c, NUM_DISTANCE_SENSORS};
use crate::motors::{run_motors, NUM_MOTORS};
use crate::wifi::{wifi_setup, PacbotCommand};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::Pin;
use embassy_rp::peripherals::{I2C0, PIO0};
use embassy_rp::pio;
use embassy_rp::{bind_interrupts, i2c};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

static SENSORS_CHANNEL: Channel<ThreadModeRawMutex, [u8; NUM_DISTANCE_SENSORS], 64> =
    Channel::new();

static ENCODERS_CHANNEL: Channel<
    ThreadModeRawMutex,
    ([i64; NUM_MOTORS], [f32; NUM_MOTORS], [f32; NUM_MOTORS]),
    64,
> = Channel::new();

/// Motor velocities and PID parameters and PID limits
static REQUESTED_VELOCITY_CHANNEL: Channel<ThreadModeRawMutex, PacbotCommand, 64> = Channel::new();

async fn blink<'a>(control: &mut Control<'a>, count: usize, duration: Duration) {
    for _ in 0..count {
        control.gpio_set(0, true).await;
        Timer::after(duration).await;
        control.gpio_set(0, false).await;
        Timer::after(duration).await;
    }
    Timer::after(Duration::from_secs(1)).await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let p = embassy_rp::init(Default::default());

    spawner
        .spawn(run_motors(
            PacbotConfig::default(),
            (p.PIN_18, p.PIN_19, p.PIN_20, p.PIN_21, p.PIN_26, p.PIN_27),
            (p.PWM_CH1, p.PWM_CH2, p.PWM_CH5),
            [
                (p.PIN_13.degrade(), p.PIN_12.degrade()),
                (p.PIN_14.degrade(), p.PIN_15.degrade()),
                (p.PIN_11.degrade(), p.PIN_10.degrade()),
            ],
            ENCODERS_CHANNEL.sender(),
        ))
        .expect("Failed in motor controller");

    spawner
        .spawn(run_i2c(
            p.I2C0,
            p.PIN_17,
            p.PIN_16,
            [
                p.PIN_2.degrade(),
                p.PIN_3.degrade(),
                p.PIN_4.degrade(),
                p.PIN_5.degrade(),
                p.PIN_6.degrade(),
                p.PIN_7.degrade(),
                p.PIN_8.degrade(),
                p.PIN_9.degrade(),
            ],
            SENSORS_CHANNEL.sender(),
        ))
        .expect("Failed in i2c task");

    spawner
        .spawn(wifi_setup(
            spawner.clone(),
            p.PIN_23,
            p.PIN_25,
            p.PIO0,
            p.PIN_24,
            p.PIN_29,
            p.DMA_CH0,
            REQUESTED_VELOCITY_CHANNEL.sender(),
        ))
        .expect("Failed in wifi setup task");
}
