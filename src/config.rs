use crate::i2c_manager::NUM_DISTANCE_SENSORS;
use crate::motors::NUM_MOTORS;
use serde::{Deserialize, Serialize};

/// Known Wi-Fi networks; will attempt to join each, in order
pub const WIFI_NETWORK: &str = env!("WIFI_NETWORK");
pub const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

/// Holds the order of the motor/encoder pins
#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub struct PacbotConfig {
    distance_sensors: [usize; NUM_DISTANCE_SENSORS],
    encoders: [(usize, usize); NUM_MOTORS],
    motors: [(usize, usize); NUM_MOTORS],

    static_ip: Option<(u8, u8, u8, u8)>,
    udp_port: usize,
}

impl Default for PacbotConfig {
    fn default() -> Self {
        Self {
            distance_sensors: [0, 1, 2, 3, 4, 5, 6, 7],
            encoders: [(0, 1), (2, 3), (4, 5)],
            motors: [(0, 1), (2, 3), (4, 5)],
            static_ip: Some((192, 168, 1, 212)),
            udp_port: 20002,
        }
    }
}

#[allow(dead_code)]
impl PacbotConfig {
    /// The distance sensor X SHUT pins are stored in an array of length NUM_DISTANCE_SENSORS
    ///
    /// These should be the indices of the x shut pin in the array, such that the first element
    /// points to the pin controlling the sensor at 0 degrees, then progressing counterclockwise
    pub fn distance_sensors(&self) -> &[usize; NUM_DISTANCE_SENSORS] {
        &self.distance_sensors
    }

    /// The distance sensor X SHUT pins are stored in an array of length NUM_DISTANCE_SENSORS
    ///
    /// These should be the indices of the x shut pin in the array, such that the first element
    /// points to the pin controlling the sensor at 0 degrees, then progressing counterclockwise.
    /// This function will swap the current pin with the one at the given index
    pub fn set_distance_sensor(&mut self, id: usize, pin: usize) {
        // first, find where the pin is currently
        let mut current = 0;
        for i in 0..NUM_DISTANCE_SENSORS {
            if self.distance_sensors[i] == pin {
                current = i;
                break;
            }
        }
        // then, swap the two
        self.distance_sensors.swap(id, current);
    }

    /// The encoder pins are stored in an array of length NUM_MOTORS * 2
    ///
    /// These should be filled in with the indices of pins (a, b) for each encoder so that positive
    /// motor outputs yield clockwise motion
    pub fn encoders(&self) -> &[(usize, usize); NUM_MOTORS] {
        &self.encoders
    }

    /// The encoder pins are stored in an array of length NUM_MOTORS * 2
    ///
    /// These should be filled in with the indices of pins (a, b) for each encoder so that positive
    /// motor outputs yield clockwise motion. This function will swap the current pin with the one
    /// at the given index
    pub fn set_encoder_a(&mut self, id: usize, a: usize) {
        for i in 0..NUM_MOTORS {
            if self.encoders[i].0 == a {
                self.encoders[i].0 = self.encoders[id].0;
                self.encoders[id].0 = a;
                break;
            }
            if self.encoders[i].1 == a {
                self.encoders[i].1 = self.encoders[id].0;
                self.encoders[id].0 = a;
                break;
            }
        }
    }

    /// The encoder pins are stored in an array of length NUM_MOTORS * 2
    ///
    /// These should be filled in with the indices of pins (a, b) for each encoder so that positive
    /// motor outputs yield clockwise motion. This function will swap the current pin with the one
    /// at the given index
    pub fn set_encoder_b(&mut self, id: usize, b: usize) {
        for i in 0..NUM_MOTORS {
            if self.encoders[i].0 == b {
                self.encoders[i].0 = self.encoders[id].1;
                self.encoders[id].1 = b;
                break;
            }
            if self.encoders[i].1 == b {
                self.encoders[i].1 = self.encoders[id].1;
                self.encoders[id].1 = b;
                break;
            }
        }
    }

    /// The motor pins are stored in an array of PWM pairs
    ///
    /// These should be filled in with the `x` for each motor where the first element is the motor
    /// at angle 0 degrees, progressing counterclockwise, where:
    /// - `x // 2` is the PWM pair
    /// - `x % 2` is the pin within the pair
    /// - Positive movement is clockwise
    pub fn motors(&self) -> &[(usize, usize); NUM_MOTORS] {
        &self.motors
    }

    /// The motor pins are stored in an array of PWM pairs
    ///
    /// These should be filled in with the `x` for each motor where the first element is the motor
    /// at angle 0 degrees, progressing counterclockwise, where:
    /// - `x // 2` is the PWM pair
    /// - `x % 2` is the pin within the pair
    /// - Positive movement is clockwise
    /// This function will swap the current pin with the one at the given index
    pub fn set_motor_a(&mut self, id: usize, a: usize) {
        for i in 0..NUM_MOTORS {
            if self.motors[i].0 == a {
                self.motors[i].0 = self.motors[id].0;
                self.motors[id].0 = a;
                break;
            }
            if self.motors[i].1 == a {
                self.motors[i].1 = self.motors[id].0;
                self.motors[id].0 = a;
                break;
            }
        }
    }

    /// The motor pins are stored in an array of PWM pairs
    ///
    /// These should be filled in with the `x` for each motor where the first element is the motor
    /// at angle 0 degrees, progressing counterclockwise, where:
    /// - `x // 2` is the PWM pair
    /// - `x % 2` is the pin within the pair
    /// - Positive movement is clockwise
    /// This function will swap the current pin with the one at the given index
    pub fn set_motor_b(&mut self, id: usize, b: usize) {
        for i in 0..NUM_MOTORS {
            if self.motors[i].0 == b {
                self.motors[i].0 = self.motors[id].1;
                self.motors[id].1 = b;
                break;
            }
            if self.motors[i].1 == b {
                self.motors[i].1 = self.motors[id].1;
                self.motors[id].1 = b;
                break;
            }
        }
    }

    /// If not None, the pico will try to request this static IP
    pub fn static_ip_mut(&mut self) -> &mut Option<(u8, u8, u8, u8)> {
        &mut self.static_ip
    }

    /// The port where the pico will listen for UDP messages
    pub fn udp_port_mut(&mut self) -> &mut usize {
        &mut self.udp_port
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_set_distance_sensor() {
        let mut config = PacbotConfig::default();
        config.set_distance_sensor(0, 1);
        assert_eq!(config.distance_sensors(), &[1, 0, 2, 3, 4, 5, 6, 7]);
        config.set_distance_sensor(2, 7);
        assert_eq!(config.distance_sensors(), &[1, 0, 7, 3, 4, 5, 6, 2]);
        config.set_distance_sensor(1, 7);
        assert_eq!(config.distance_sensors(), &[1, 7, 0, 3, 4, 5, 6, 2]);
    }

    #[test]
    fn test_set_encoder_a() {
        let mut config = PacbotConfig::default();
        config.set_encoder_a(0, 3);
        assert_eq!(config.encoders(), &[(3, 1), (2, 0), (4, 5)]);
    }

    #[test]
    fn test_set_encoder_b() {
        let mut config = PacbotConfig::default();
        config.set_encoder_b(0, 2);
        assert_eq!(config.encoders(), &[(1, 2), (3, 0), (4, 5)]);
    }

    #[test]
    fn test_set_motor_a() {
        let mut config = PacbotConfig::default();
        config.set_motor_a(0, 3);
        assert_eq!(config.motors(), &[(3, 1), (2, 0), (4, 5)]);
    }

    #[test]
    fn test_set_motor_b() {
        let mut config = PacbotConfig::default();
        config.set_motor_b(0, 2);
        assert_eq!(config.motors(), &[(1, 2), (3, 0), (4, 5)]);
    }
}
