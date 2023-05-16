#![no_std]

use embedded_hal::PwmPin;
use num::complex::Complex32;
use num_traits::{FromPrimitive, ToPrimitive};

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
}

impl<M, C, V> Motor<M, C, V> {
    pub fn control(&mut self, t: f32)
    where
        M: Model,
        C: Control<M>,
        V: PwmPin,
        V::Duty: FromPrimitive + ToPrimitive,
    {
        let (_t_s, ratios) = self.control.control(&mut self.model, self.w_m_ref, t);
        for (pwm, ratio) in self.pwm.iter_mut().zip(ratios) {
            let duty = pwm.get_max_duty().to_f32().unwrap() * ratio;

            pwm.set_duty(V::Duty::from_f32(duty.round()).unwrap())
        }
    }
}
