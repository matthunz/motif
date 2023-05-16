use std::marker::PhantomData;

use embedded_hal::{
    adc::{Channel, OneShot},
    PwmPin,
};
use num::complex::Complex32;

pub mod induction;
pub use induction::InductionMotorVhzControl;

pub mod pwm;
use num_traits::{FromPrimitive, ToPrimitive};
pub use pwm::Pwm;

mod rate_limiter;
pub use rate_limiter::RateLimiter;

fn complex_to_abc(u: Complex32) -> [f32; 3] {
    [
        u.re,
        0.5 * (-u.re + 3f32.sqrt() * u.im),
        0.5 * (-u.re - 3f32.sqrt() * u.im),
    ]
}

fn abc_to_complex(u: [f32; 3]) -> f32 {
    (2. / 3.) * u[0] - (u[1] + u[2]) / 3. + 1. * (u[1] - u[2]) / 3f32.sqrt()
}

pub struct AnalogSensor<T, P, A, W> {
    adc: T,
    pin: P,
    from_min: f32,
    from_max: f32,
    to_min: f32,
    to_max: f32,
    _marker: PhantomData<(A, W)>,
}

impl<T, P, A, W> Sensor for AnalogSensor<T, P, A, W>
where
    T: OneShot<A, W, P>,
    P: Channel<A>,
    W: ToPrimitive,
{
    fn read(&mut self) -> f32 {
        let v = self.adc.read(&mut self.pin).ok().unwrap().to_f32().unwrap();

        // Calculate the ratio of the input value relative to the input range
        let ratio = (v - self.from_min) / (self.from_max - self.from_min);

        // Map the ratio to the output range
        self.to_min + (ratio * (self.to_max - self.to_min))
    }
}

pub trait Sensor {
    fn read(&mut self) -> f32;
}

pub struct Motor<T, U, V> {
    phase_current_sensors: [T; 3],
    dc_bus_sensor: U,
    pwm: [V; 3],
}

impl<T, U, V> Motor<T, U, V> {
    pub fn phase_currents(&mut self) -> [f32; 3]
    where
        T: Sensor,
    {
        [
            self.phase_current_sensors[0].read(),
            self.phase_current_sensors[1].read(),
            self.phase_current_sensors[2].read(),
        ]
    }

    pub fn dc_bus_voltage(&mut self) -> f32
    where
        U: Sensor,
    {
        self.dc_bus_sensor.read()
    }

    pub fn output(&mut self, ratios: [f32; 3])
    where
        V: PwmPin,
        V::Duty: FromPrimitive + ToPrimitive,
    {
        for (pwm, ratio) in self.pwm.iter_mut().zip(ratios) {
            let duty = pwm.get_max_duty().to_f32().unwrap() * ratio;

            pwm.set_duty(V::Duty::from_f32(duty.round()).unwrap())
        }
    }
}
