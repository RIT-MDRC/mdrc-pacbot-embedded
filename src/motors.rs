use crate::config::PacbotConfig;
use crate::wifi::MotorRequest;
use crate::REQUESTED_VELOCITY_CHANNEL;
use defmt::info;
use embassy_rp::gpio::{AnyPin, Input, Pull};
use embassy_rp::peripherals::{
    PIN_18, PIN_19, PIN_20, PIN_21, PIN_26, PIN_27, PWM_CH1, PWM_CH2, PWM_CH5,
};
use embassy_rp::pwm;
use embassy_rp::pwm::Pwm;
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
    motor_configs: [pwm::Config; NUM_MOTORS],
    encoders: [Option<Rotary<Input<'a>, Input<'a>>>; NUM_MOTORS],
    encoder_data: [EncoderData; NUM_MOTORS],
}

#[allow(dead_code)]
impl<'a> MotorIO<'a> {
    /// Create a new MotorIO
    fn new(
        config: PacbotConfig,
        motors: (Pwm<'a, PWM_CH1>, Pwm<'a, PWM_CH2>, Pwm<'a, PWM_CH5>),
        motor_config: pwm::Config,
        encoders: [Rotary<Input<'a>, Input<'a>>; NUM_MOTORS],
    ) -> Self {
        let mut s = Self {
            config,
            motors,
            motor_configs: [motor_config.clone(), motor_config.clone(), motor_config],
            encoders: encoders.map(Some),
            encoder_data: [0, 1, 2].map(|x| EncoderData {
                pins: (x * 2, x * 2 + 1),
                ticks: 0,
                last_tick: Instant::now(),
                last_last_tick: Instant::now(),
                velocity: 0.0,
            }),
        };
        s.set_up_encoders();
        s
    }

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
            self.motor_configs[id / 2].compare_a
        } else {
            self.motor_configs[id / 2].compare_b
        }
    }

    /// Set the output value for a given pin
    pub fn set_pin_pwm(&mut self, id: usize, compare: u16) {
        if id % 2 == 0 {
            self.motor_configs[id / 2].compare_a = compare;
        } else {
            self.motor_configs[id / 2].compare_b = compare;
        }
        self.set_pwm_config(id / 2)
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

#[allow(clippy::type_complexity)]
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

    let pwm_top: u16 = 0x8000;
    let mut pwm_config = pwm::Config::default();
    pwm_config.top = pwm_top;

    let motors = (
        Pwm::new_output_ab(pwm.0, motor_pins.0, motor_pins.1, pwm_config.clone()),
        Pwm::new_output_ab(pwm.1, motor_pins.2, motor_pins.3, pwm_config.clone()),
        Pwm::new_output_ab(pwm.2, motor_pins.4, motor_pins.5, pwm_config.clone()),
    );

    let encoders = encoder_pins
        .map(|(a, b)| Rotary::new(Input::new(a, Pull::Down), Input::new(b, Pull::Down)));

    let mut motor_io = MotorIO::new(config, motors, pwm_config, encoders);

    // pid is tracking velocity in ticks/s, max 11 * 150 ticks /s
    let mut default_pid = Pid::new(0.0, pwm_top as f32);
    default_pid.p(5.0, 1000.0);
    let mut pids = [default_pid; NUM_MOTORS];

    let mut motor_requests = [MotorRequest::Pwm(pwm_top, pwm_top); NUM_MOTORS];
    let mut pid_output = [0.0; NUM_MOTORS];
    loop {
        while let Ok(command) = REQUESTED_VELOCITY_CHANNEL.try_receive() {
            if motor_requests != command.motors {
                info!("New requested motor velocities: {:?}", command.motors);
                motor_requests = command.motors;
            }
            for (i, motor_request) in command.motors.iter().enumerate() {
                if let MotorRequest::Velocity(v) = motor_request {
                    // velocity in grid units/s, setpoint in encoder ticks/sec
                    // 1 gu = 88.9mm, so multiply by 88.9 to get mm/s
                    // wheel diameter is 38.1mm
                    // so 1 tick = 38.1 * pi / 150
                    // so 1 tick = 0.7979mm
                    // so 1mm = 1.253 ticks
                    // so multiply by 1.253 to get ticks/s
                    pids[i].setpoint(*v * 88.9 * 1.253);
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
            let encoder_data = &mut motor_io.encoder_data[motor_id];
            match dir {
                Direction::Clockwise => {
                    encoder_data.ticks += 1;
                    encoder_data.last_last_tick = encoder_data.last_tick;
                    encoder_data.last_tick = Instant::now();
                }
                Direction::CounterClockwise => {
                    encoder_data.ticks -= 1;
                    encoder_data.last_last_tick = encoder_data.last_tick;
                    encoder_data.last_tick = Instant::now();
                }
                Direction::None => {}
            }
            // If it didn't change, use the direction from before
            if dir == Direction::None {
                dir = if encoder_data.velocity < 0.0 {
                    Direction::CounterClockwise
                } else {
                    Direction::Clockwise
                };
            }
            // update estimated velocity
            let dt = if let Some(dt) = encoder_data
                .last_tick
                .checked_duration_since(encoder_data.last_last_tick)
            {
                dt.as_micros() as f32
            } else {
                // sometimes the instants are out of order
                10.0
            };
            // if it has been too long since the last encoder tick, use the elapsed time to estimate velocity
            let time_since_last = encoder_data.last_tick.elapsed().as_micros() as f32;
            encoder_data.velocity = if time_since_last > dt {
                // what would the velocity be if it ticked now?
                1_000_000.0 / time_since_last
            } else {
                // use the time between the previous two ticks
                1_000_000.0 / dt
            };
            if dir == Direction::CounterClockwise {
                encoder_data.velocity *= -1.0;
            }

            // update motor output
            match motor_requests[motor_id] {
                MotorRequest::Velocity(v) => {
                    // not great...
                    if v == 0.0 {
                        motor_io.set_motor_speeds(motor_id, pwm_top, pwm_top);
                    } else {
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
