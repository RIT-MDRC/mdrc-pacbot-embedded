use crate::config::{WIFI_NETWORK, WIFI_PASSWORD};
use crate::i2c_manager::NUM_DISTANCE_SENSORS;
use crate::motors::NUM_MOTORS;
use crate::{blink, Irqs, ENCODERS_CHANNEL, SENSORS_CHANNEL};
use core::future::poll_fn;
use core::task::Poll;
use cyw43_pio::PioSpi;
use defmt::{error, info, unwrap, Format};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_net::udp::{PacketMetadata, RecvError, UdpSocket};
use embassy_net::{Config, IpEndpoint, Ipv4Address, Ipv4Cidr, Stack, StackResources};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::Pio;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Timer};
use futures::TryFutureExt;
use heapless::Vec;
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

/// Messages from the client
#[derive(Copy, Clone, Deserialize)]
pub struct PacbotCommand {
    pub motors: [MotorRequest; 3],
    pub pid: [f32; 3],
    pub pid_limits: [f32; 3],
}

/// The way the client wants the motor to be controlled
#[derive(Copy, Clone, Debug, Format, PartialOrd, PartialEq, Deserialize)]
pub enum MotorRequest {
    /// Use PID to move the motor to this velocity
    Velocity(f32),
    /// Set PWM to these values directly
    Pwm(u16, u16),
}

/// Messages from the robot to the client
#[derive(Copy, Clone, Serialize)]
pub struct PacbotSensors {
    pub distance_sensors: [u8; NUM_DISTANCE_SENSORS],
    pub encoders: [i64; NUM_MOTORS],
    pub encoder_velocities: [f32; NUM_MOTORS],
    pub pid_output: [f32; NUM_MOTORS],
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
pub async fn wifi_setup(
    spawner: Spawner,
    pwr: PIN_23,
    cs: PIN_25,
    pio: PIO0,
    dio: PIN_24,
    clk: PIN_29,
    dma: DMA_CH0,
    motor_sender: Sender<'static, ThreadModeRawMutex, PacbotCommand, 64>,
) {
    info!("Wifi task started");

    let pwr = Output::new(pwr, Level::Low);
    let cs = Output::new(cs, Level::High);
    let mut pio = Pio::new(pio, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, dio, clk, dma);

    // let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    // let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    info!("Wifi startup complete");

    let config = Config::dhcpv4(Default::default());
    // let config = Config::ipv4_static(embassy_net::StaticConfigV4 {
    //     address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 1, 212), 24),
    //     dns_servers: Vec::new(),
    //     gateway: None,
    // });

    // Generate random seed
    let seed = 0xab9a_dd1a_3b2b_715a; // chosen by fair dice roll

    // Init network stack
    static STACK: StaticCell<Stack<cyw43::NetDriver<'static>>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        net_device,
        config,
        RESOURCES.init(StackResources::<2>::new()),
        seed,
    ));

    unwrap!(spawner.spawn(net_task(stack)));

    info!("Network stack initialized");

    blink(&mut control, 1, Duration::from_millis(300)).await;

    loop {
        // control.join_open("RIT-WiFi").await;
        match control.join_open("testnetwork").await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
                blink(&mut control, 3, Duration::from_millis(100)).await;
            }
        }
        // match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
        //     Ok(_) => break,
        //     Err(err) => {
        //         info!("join failed with status={}", err.status);
        //         // blink(&mut control, 3, Duration::from_millis(100)).await;
        //     }
        // }
    }

    info!("Joined network");

    blink(&mut control, 1, Duration::from_millis(300)).await;

    // Wait for DHCP, not necessary when using static IP
    info!("Waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    info!("DHCP is now up!");

    info!("ip = {}", stack.config_v4().unwrap().address);

    blink(&mut control, 1, Duration::from_millis(300)).await;

    let mut rx_meta = [PacketMetadata::EMPTY; 64];
    let mut rx_buffer = [0; 8092];
    let mut tx_meta = [PacketMetadata::EMPTY; 64];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    let mut socket = UdpSocket::new(
        &stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );
    socket.bind(20002).unwrap();

    info!("Listening for UDP on port 20002...");
    let mut msg_bytes = [0; 100];

    let mut msg = PacbotSensors {
        distance_sensors: [0; NUM_DISTANCE_SENSORS],
        encoders: [0; NUM_MOTORS],
        encoder_velocities: [0.0; NUM_MOTORS],
        pid_output: [0.0; NUM_MOTORS],
    };
    let config = bincode::config::standard();
    let motor_stop_command = PacbotCommand {
        motors: [
            MotorRequest::Velocity(0.0),
            MotorRequest::Velocity(0.0),
            MotorRequest::Velocity(0.0),
        ],
        pid: [5.0, 0.0, 0.0],
        pid_limits: [1000.0, 1000.0, 1000.0],
    };
    loop {
        match recv_from_with_timeout(&socket, &mut buf, Duration::from_secs(1)).await {
            Some(Ok((_n, ep))) => {
                if let Ok((command, _)) =
                    bincode::serde::decode_from_slice::<PacbotCommand, _>(&buf, config)
                {
                    let _ = motor_sender.try_send(command);
                }
                while let Ok(new_sensors) = SENSORS_CHANNEL.try_receive() {
                    msg.distance_sensors = new_sensors;
                }
                while let Ok((encoders, velocities, pid_output)) = ENCODERS_CHANNEL.try_receive() {
                    msg.encoders = encoders;
                    msg.encoder_velocities = velocities;
                    msg.pid_output = pid_output;
                    // info!("{:?}", msg.encoder_velocities);
                }
                let n = bincode::serde::encode_into_slice(&msg, &mut msg_bytes, config).unwrap();
                socket.send_to(&msg_bytes[..n], ep).await.unwrap();
            }
            Some(Err(e)) => {
                let _ = motor_sender.try_send(motor_stop_command.clone());
                error!("Error when receiving! Setting motors to 0");
            }
            None => {
                let _ = motor_sender.try_send(motor_stop_command.clone());
                error!("Recv timeout! Setting motors to 0");
            }
        }
    }
}

async fn recv_from_with_timeout<'a>(
    socket: &UdpSocket<'a>,
    buf: &mut [u8],
    timeout: Duration,
) -> Option<Result<(usize, IpEndpoint), RecvError>> {
    let timer = Timer::after(timeout);

    let future = select(poll_fn(move |cx| socket.poll_recv_from(buf, cx)), timer).await;

    // At this point, if the future resolved because of the timer, it means a timeout occurred.
    // If it resolved because of poll_recv_from, then data was received before the timeout.
    match future {
        Either::First(x) => Some(x),
        _ => None,
    }
}
