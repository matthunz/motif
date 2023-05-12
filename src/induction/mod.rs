use std::f32::consts::PI;

mod vhz;
pub use vhz::InductionMotorVhzControl;

pub trait Speed {
    /// Speed reference (in electrical rad/s) at the current time `t`.
    fn speed(&self, t: f32) -> f32;
}

impl<F> Speed for F
where
    F: Fn(f32) -> f32,
{
    fn speed(&self, t: f32) -> f32 {
        self(t)
    }
}

pub struct DefaultSpeed;

impl Speed for DefaultSpeed {
    fn speed(&self, t: f32) -> f32 {
        if t > 0.2 {
            2. * PI * 50.
        } else {
            0.
        }
    }
}
