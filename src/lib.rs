#![no_std]

use embedded_hal::PwmPin;
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

pub struct Motor<M, C, V> {
    pub model: M,
    pub control: C,
    pub pwm: [V; 3],
    pub w_m_ref: f32,
    pub is_armed: bool,
}

impl<M, C, V> Motor<M, C, V> {
    pub fn control(&mut self, t: f32)
    where
        M: Model,
        C: Control<M>,
        V: PwmPin,
        V::Duty: FromPrimitive + ToPrimitive,
    {
        if !self.is_armed {
            todo!()
        }

        let duty_cycle_ratios = self.control.control(&mut self.model, self.w_m_ref, t);
        for (pwm, duty_cycle_ratio) in self.pwm.iter_mut().zip(duty_cycle_ratios) {
            let duty = pwm.get_max_duty().to_f32().unwrap() * duty_cycle_ratio;
            pwm.set_duty(V::Duty::from_f32(duty.round()).unwrap())
        }
    }
}
