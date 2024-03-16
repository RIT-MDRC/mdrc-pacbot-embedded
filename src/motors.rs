use crate::config::PacbotConfig;
use crate::wifi::MotorRequest;
use crate::REQUESTED_VELOCITY_CHANNEL;
use defmt::info;
use embassy_rp::gpio::{AnyPin, Input, Pull};
use embassy_rp::peripherals::{
    PIN_18, PIN_19, PIN_20, PIN_21, PIN_26, PIN_27, PWM_CH1, PWM_CH2, PWM_CH5,
};
use embassy_rp::pwm::{Config, Pwm};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Instant, Timer};
use pid::Pid;
use rotary_encoder_hal::{Direction, Rotary};

pub const NUM_MOTORS: usize = 3;

/// Holds information needed for encoder tracking
#[derive(Copy, Clone)]
struct EncoderData {
    /// The current pins the encoder is using
    pins: (usize, usize),
    /// Signed position of the encoder; increases with clockwise movement
    ticks: i64,
    /// The time of the most recent tick, used for velocity calculation
    last_tick: Instant,
    /// The time of the tick before the most recent tick, used for velocity calculation
    last_last_tick: Instant,
    /// Signed estimated velocity of the motor; positive is clockwise
    velocity: f32,
}

/// Holds all the configurable motors/encoders
struct MotorIO<'a> {
    config: PacbotConfig,
    motors: (Pwm<'a, PWM_CH1>, Pwm<'a, PWM_CH2>, Pwm<'a, PWM_CH5>),
    motor_configs: [Config; NUM_MOTORS],
    encoders: [Option<Rotary<Input<'a>, Input<'a>>>; NUM_MOTORS],
    encoder_data: [EncoderData; NUM_MOTORS],
}

#[allow(dead_code)]
impl<'a> MotorIO<'a> {
    /// Applies changes to motor_configs to the pins
    fn set_pwm_config(&mut self, id: usize) {
        match id {
            0 => self.motors.0.set_config(&self.motor_configs[id]),
            1 => self.motors.1.set_config(&self.motor_configs[id]),
            2 => self.motors.2.set_config(&self.motor_configs[id]),
            _ => panic!("Invalid motor id sent to MotorIO::set_config"),
        }
    }

    /// Sets the "top" value for all PWMs
    pub fn set_pwm_top(&mut self, top: u16) {
        for i in 0..NUM_MOTORS {
            self.motor_configs[i].top = top;
        }
        self.motors.0.set_config(&self.motor_configs[0]);
        self.motors.1.set_config(&self.motor_configs[1]);
        self.motors.2.set_config(&self.motor_configs[2]);
    }

    /// Get the current output on the given pin
    pub fn get_pin_pwm(&self, id: usize) -> u16 {
        if id % 2 == 0 {
            self.motor_configs[id / NUM_MOTORS].compare_a
        } else {
            self.motor_configs[id / NUM_MOTORS].compare_b
        }
    }

    /// Set the output value for a given pin
    pub fn set_pin_pwm(&mut self, id: usize, compare: u16) {
        if id % 2 == 0 {
            self.motor_configs[id / NUM_MOTORS].compare_a = compare;
        } else {
            self.motor_configs[id / NUM_MOTORS].compare_b = compare;
        }
        self.set_pwm_config(id / NUM_MOTORS)
    }

    /// Get the current PWM outputs for the given motor
    pub fn get_motor_speeds(&self, id: usize) -> (u16, u16) {
        let (a, b) = self.config.motors()[id];
        (self.get_pin_pwm(a), self.get_pin_pwm(b))
    }

    /// Set the PWM outputs for the given motor
    pub fn set_motor_speeds(&mut self, id: usize, compare_a: u16, compare_b: u16) {
        let (a, b) = self.config.motors()[id];
        self.set_pin_pwm(a, compare_a);
        self.set_pin_pwm(b, compare_b);
    }

    /// Destroy the encoders and set them up again with values from the configuration
    pub fn set_up_encoders(&mut self) {
        const ARRAY_REPEAT_VALUE: Option<Input> = None;
        let mut pins = [ARRAY_REPEAT_VALUE; NUM_MOTORS * 2];

        for id in 0..NUM_MOTORS {
            let (a, b) = self.encoders[id].take().unwrap().into_inner();
            pins[self.encoder_data[id].pins.0] = Some(a);
            pins[self.encoder_data[id].pins.1] = Some(b);
        }

        for id in 0..NUM_MOTORS {
            let (a, b) = self.config.encoders()[id];
            self.encoders[id] = Some(Rotary::new(
                pins[a].take().unwrap(),
                pins[b].take().unwrap(),
            ));
            self.encoder_data[id] = EncoderData {
                pins: (a, b),
                ticks: 0,
                last_tick: Instant::now(),
                last_last_tick: Instant::now(),
                velocity: 0.0,
            }
        }
    }
}

#[embassy_executor::task]
pub async fn run_motors(
    config: PacbotConfig,
    motor_pins: (PIN_18, PIN_19, PIN_20, PIN_21, PIN_26, PIN_27),
    pwm: (PWM_CH1, PWM_CH2, PWM_CH5),
    encoder_pins: [(AnyPin, AnyPin); NUM_MOTORS],
    encoder_send: Sender<
        'static,
        ThreadModeRawMutex,
        ([i64; NUM_MOTORS], [f32; NUM_MOTORS], [f32; NUM_MOTORS]),
        64,
    >,
) {
    info!("Motors task started");

    let motors = (
        Pwm::new_output_ab(pwm.0, motor_pins.0, motor_pins.1, Config::default()),
        Pwm::new_output_ab(pwm.1, motor_pins.2, motor_pins.3, Config::default()),
        Pwm::new_output_ab(pwm.2, motor_pins.4, motor_pins.5, Config::default()),
    );

    let encoders = encoder_pins.map(|(a, b)| {
        Some(Rotary::new(
            Input::new(a, Pull::Down),
            Input::new(b, Pull::Down),
        ))
    });

    let mut motor_io = MotorIO {
        config,
        motors,
        motor_configs: [Config::default(), Config::default(), Config::default()],
        encoders,
        encoder_data: [EncoderData {
            pins: (0, 1),
            ticks: 0,
            last_tick: Instant::now(),
            last_last_tick: Instant::now(),
            velocity: 0.0,
        }; 3],
    };
    motor_io.encoder_data[1].pins = (2, 3);
    motor_io.encoder_data[2].pins = (4, 5);
    motor_io.set_up_encoders();

    // pid is tracking velocity in ticks/s, max 11 * 150 ticks /s
    let pwm_top: u16 = 0x8000;
    let mut pids = [Pid::new(0.0, pwm_top as f32); NUM_MOTORS];

    for i in 0..NUM_MOTORS {
        motor_io.set_pwm_top(pwm_top);
        pids[i].p(3000.0, 1000.0);
    }

    let mut motor_requests = [MotorRequest::Pwm(pwm_top, pwm_top); NUM_MOTORS];
    let mut pid_output = [0.0; NUM_MOTORS];
    loop {
        while let Ok(command) = REQUESTED_VELOCITY_CHANNEL.try_receive() {
            if motor_requests != command.motors {
                info!("New requested motor velocities: {:?}", command.motors);
                motor_requests = command.motors;
            }
            for i in 0..NUM_MOTORS {
                if let MotorRequest::Velocity(v) = command.motors[i] {
                    // velocity in grid units/s, setpoint in encoder ticks/sec
                    // 1 gu = 88.9mm, so multiply by 88.9 to get mm/s
                    // wheel diameter is 38.1mm
                    // so 1 tick = 38.1 * pi / 150
                    // so 1 tick = 0.7979mm
                    // so 1mm = 1.253 ticks
                    // so multiply by 1.253 to get ticks/s
                    pids[i].setpoint(v * 88.9 * 1.253);
                    pids[i].p(command.pid[0], command.pid_limits[0]);
                    pids[i].i(command.pid[1], command.pid_limits[1]);
                    pids[i].d(command.pid[2], command.pid_limits[2]);
                }
            }
        }
        for motor_id in 0..NUM_MOTORS {
            // update encoder
            let mut dir = Direction::None;
            if let Some(ref mut e) = motor_io.encoders[motor_id] {
                dir = e.update().unwrap()
            }
            match dir {
                Direction::Clockwise => {
                    motor_io.encoder_data[motor_id].ticks += 1;
                    motor_io.encoder_data[motor_id].last_last_tick =
                        motor_io.encoder_data[motor_id].last_tick;
                    motor_io.encoder_data[motor_id].last_tick = Instant::now();
                }
                Direction::CounterClockwise => {
                    motor_io.encoder_data[motor_id].ticks -= 1;
                    motor_io.encoder_data[motor_id].last_last_tick =
                        motor_io.encoder_data[motor_id].last_tick;
                    motor_io.encoder_data[motor_id].last_tick = Instant::now();
                }
                Direction::None => {}
            }
            // If it didn't change, use the direction from before
            if dir == Direction::None {
                dir = if motor_io.encoder_data[motor_id].velocity < 0.0 {
                    Direction::CounterClockwise
                } else {
                    Direction::Clockwise
                };
            }
            // update estimated velocity
            let dt = (motor_io.encoder_data[motor_id].last_tick
                - motor_io.encoder_data[motor_id].last_last_tick)
                .as_micros() as f32;
            // if it has been too long since the last encoder tick, use the elapsed time to estimate velocity
            let time_since_last = motor_io.encoder_data[motor_id]
                .last_tick
                .elapsed()
                .as_micros() as f32;
            motor_io.encoder_data[motor_id].velocity = if dt < time_since_last {
                // what would the velocity be if it ticked now?
                1_000_000.0 / time_since_last
            } else {
                // use the time between the previous two ticks
                1_000_000.0 / dt
            };
            if dir == Direction::CounterClockwise {
                motor_io.encoder_data[motor_id].velocity *= -1.0;
            }

            // update motor output
            match motor_requests[motor_id] {
                MotorRequest::Velocity(_) => {
                    // update PID
                    let output = pids[motor_id]
                        .next_control_output(motor_io.encoder_data[motor_id].velocity)
                        .output;
                    pid_output[motor_id] = output;
                    // set pwm
                    if output > 0.0 {
                        motor_io.set_motor_speeds(motor_id, output as u16, 0);
                    } else {
                        motor_io.set_motor_speeds(motor_id, 0, -output as u16);
                    }
                }
                MotorRequest::Pwm(a, b) => {
                    // set pwm directly
                    motor_io.set_motor_speeds(motor_id, a, b);
                }
            }
        }
        let _ = encoder_send.try_send((
            motor_io.encoder_data.map(|x| x.ticks),
            motor_io.encoder_data.map(|x| x.velocity),
            pid_output,
        ));
        Timer::after_micros(100).await
    }
}
