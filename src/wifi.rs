use crate::config::{PacbotConfig, DEFAULT_WIFI_NETWORK, WIFI_NETWORK, WIFI_PASSWORD};
use crate::i2c_manager::NUM_DISTANCE_SENSORS;
use crate::motors::NUM_MOTORS;
use crate::{blink, Irqs, ENCODERS_CHANNEL, SENSORS_CHANNEL};
use core::future::poll_fn;
use cyw43_pio::PioSpi;
use defmt::{debug, error, info, unwrap, Format};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_net::udp::{PacketMetadata, RecvError, UdpSocket};
use embassy_net::{Config, IpEndpoint, Stack, StackResources};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::Pio;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Timer};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

/// Messages from the client
#[derive(Copy, Clone, Deserialize)]
pub struct PacbotCommand {
    /// Velocities or PWM values for the motors or pins
    pub motors: [MotorRequest; 3],
    /// Values of P, I, and D
    pub pid: [f32; 3],
    /// PID limits for P, I, and D terms
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
struct PacbotSensors {
    /// 8 bit readings from distance sensors
    pub distance_sensors: [u8; NUM_DISTANCE_SENSORS],
    /// Encoder positions, can be quite large
    pub encoders: [i64; NUM_MOTORS],
    /// Encoder velocities, passed to PID controllers
    pub encoder_velocities: [f32; NUM_MOTORS],
    /// Current PID output
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

/// Starts up Wi-Fi functionality, and listens for/responds to UDP packets
#[allow(clippy::too_many_arguments)]
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
    config: PacbotConfig,
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

    // Determine whether static or dynamic IP should be used
    let network_config = if let Some((ip, gw)) = config.static_ip {
        Config::ipv4_static(embassy_net::StaticConfigV4 {
            address: embassy_net::Ipv4Cidr::new(
                embassy_net::Ipv4Address::new(ip.0, ip.1, ip.2, ip.3),
                24,
            ),
            dns_servers: heapless::Vec::new(),
            gateway: Some(embassy_net::Ipv4Address::new(gw.0, gw.1, gw.2, gw.3)),
        })
    } else {
        Config::dhcpv4(Default::default())
    };

    // Generate random seed
    let seed = 0xab9a_dd1a_3b2b_715a; // chosen by fair dice roll

    // Init network stack
    static STACK: StaticCell<Stack<cyw43::NetDriver<'static>>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        net_device,
        network_config,
        RESOURCES.init(StackResources::<2>::new()),
        seed,
    ));
    unwrap!(spawner.spawn(net_task(stack)));
    info!("Network stack initialized");

    blink(&mut control, 1, Duration::from_millis(300)).await;

    // Attempt to join network
    let network = WIFI_NETWORK.unwrap_or(DEFAULT_WIFI_NETWORK);
    loop {
        if let Some(password) = WIFI_PASSWORD {
            match control.join_wpa2(network, password).await {
                Ok(_) => break,
                Err(err) => {
                    info!("join failed with status={}", err.status);
                    blink(&mut control, 3, Duration::from_millis(100)).await;
                }
            }
        } else {
            match control.join_open(network).await {
                Ok(_) => break,
                Err(err) => {
                    info!("join failed with status={}", err.status);
                    blink(&mut control, 3, Duration::from_millis(100)).await;
                }
            }
        }
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

    // buffers for UDP
    let mut rx_meta = [PacketMetadata::EMPTY; 64];
    let mut rx_buffer = [0; 8092];
    let mut tx_meta = [PacketMetadata::EMPTY; 64];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    // start UDP
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );
    socket.bind(20002).unwrap();
    info!("Listening for UDP on port 20002...");

    let bincode_config = bincode::config::standard();

    // udp will be sending out sensor readings
    let mut outgoing_msg_buf = [0; 100];
    let mut sensors_msg = PacbotSensors {
        distance_sensors: [0; NUM_DISTANCE_SENSORS],
        encoders: [0; NUM_MOTORS],
        encoder_velocities: [0.0; NUM_MOTORS],
        pid_output: [0.0; NUM_MOTORS],
    };

    // and receiving motor requests
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
        // receive new data
        match recv_from_with_timeout(&socket, &mut buf, Duration::from_secs(1)).await {
            Some(Ok((_n, ep))) => {
                // parse the command
                if let Ok((command, _)) =
                    bincode::serde::decode_from_slice::<PacbotCommand, _>(&buf, bincode_config)
                {
                    // if it was parsed successfully, send the result to the motors
                    let _ = motor_sender.try_send(command);
                }
                // is new sensor data available?
                while let Ok(new_sensors) = SENSORS_CHANNEL.try_receive() {
                    sensors_msg.distance_sensors = new_sensors;
                }
                // is new encoder data available?
                while let Ok((encoders, velocities, pid_output)) = ENCODERS_CHANNEL.try_receive() {
                    sensors_msg.encoders = encoders;
                    sensors_msg.encoder_velocities = velocities;
                    sensors_msg.pid_output = pid_output;
                }
                // serialize the sensor message
                let n = bincode::serde::encode_into_slice(
                    sensors_msg,
                    &mut outgoing_msg_buf,
                    bincode_config,
                )
                .unwrap();
                // send the sensor message
                socket.send_to(&outgoing_msg_buf[..n], ep).await.unwrap();
            }
            Some(Err(_)) => {
                // there was an error with the socket
                let _ = motor_sender.try_send(motor_stop_command);
                error!("Error when receiving! Setting motors to 0");
            }
            None => {
                // the socket didn't receive data in time
                // send stop to the motors just in case
                let _ = motor_sender.try_send(motor_stop_command);
                debug!("Recv timeout! Setting motors to 0");
            }
        }
    }
}

/// Try to receive data into the buffer.
///
/// If no data is received after the given duration, returns None
async fn recv_from_with_timeout<'a>(
    socket: &UdpSocket<'a>,
    buf: &mut [u8],
    timeout: Duration,
) -> Option<Result<(usize, IpEndpoint), RecvError>> {
    let future = select(
        poll_fn(move |cx| socket.poll_recv_from(buf, cx)),
        Timer::after(timeout),
    )
    .await;

    match future {
        Either::First(x) => Some(x),
        _ => None,
    }
}
