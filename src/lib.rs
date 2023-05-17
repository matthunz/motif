#![no_std]

use num_complex::Complex32;
use num_traits::Float;

pub mod control;
pub use control::Control;

pub mod drive;
pub use drive::Drive;

pub mod model;
pub use model::Model;

pub struct Motor<M, C, D> {
    pub model: M,
    pub control: C,
    pub drive: D,
    pub w_m_ref: f32,
}

impl<M, C, D> Motor<M, C, D> {
    /// Control the motor at the current time `t`.
    pub fn control(&mut self, t: f32)
    where
        M: Model,
        C: Control<M>,
        D: Drive,
    {
        let duty_cycle_ratios = self.control.control(&mut self.model, self.w_m_ref, t);
        self.drive.drive(duty_cycle_ratios);
    }
}

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
