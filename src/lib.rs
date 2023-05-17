#![no_std]

use embedded_hal::{Pwm, PwmPin};
use num_complex::Complex32;
use num_traits::{Float, FromPrimitive, ToPrimitive};

pub mod control;
pub use control::Control;

pub mod model;
pub use model::Model;

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

pub struct Motor<M, C, D> {
    pub model: M,
    pub control: C,
    pub drive: D,
    pub w_m_ref: f32,
    pub is_armed: bool,
}

impl<M, C, D> Motor<M, C, D> {
    pub fn control(&mut self, t: f32)
    where
        M: Model,
        C: Control<M>,
        D: Drive,
    {
        if !self.is_armed {
            todo!()
        }

        let duty_cycle_ratios = self.control.control(&mut self.model, self.w_m_ref, t);
        self.drive.drive(duty_cycle_ratios);
    }
}

pub trait Drive {
    fn drive(&mut self, duty_cycle_ratios: [f32; 3]);
}

pub struct MotorDrive<T, C> {
    pub pwm: T,
    pub channels: [C; 3],
}

impl<T, C> MotorDrive<T, C> {
    pub fn new(pwm: T, channels: [C; 3]) -> Self {
        Self { pwm, channels }
    }
}

impl<T, C> Drive for MotorDrive<T, C>
where
    T: Pwm<Channel = C>,
    T::Duty: FromPrimitive + ToPrimitive,
    C: Clone,
{
    fn drive(&mut self, duty_cycle_ratios: [f32; 3]) {
        for (channel, duty_cycle_ratio) in self.channels.iter().cloned().zip(duty_cycle_ratios) {
            let duty = self.pwm.get_max_duty().to_f32().unwrap() * duty_cycle_ratio;
            self.pwm
                .set_duty(channel, T::Duty::from_f32(duty.round()).unwrap())
        }
    }
}
