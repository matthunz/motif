use crate::Model;

mod induction;
pub use induction::InductionMotorVhzControl;

pub mod pwm;
pub use pwm::Pwm;

mod rate_limiter;
pub use rate_limiter::RateLimiter;

pub trait Control<M: Model> {
    /// Calculate the 3-phase PWM duty cycle ratios to control the motor.
    fn control(&mut self, drive: &mut M, w_m_ref: f32, t_s: f32) -> [f32; 3];
}
