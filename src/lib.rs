use num::{complex::Complex32, Zero};
use std::f32::consts::PI;

pub mod pwm;
pub use pwm::Pwm;

mod rate_limiter;
pub use rate_limiter::RateLimiter;

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

/// V/Hz control for induction motor drives.
pub struct InductionMotorVhzControl<T = DefaultSpeed> {
    // Speed reference (in electrical rad/s).
    pub w_m_ref: T,

    /// Frequency reference rate limiter.
    pub rate_limiter: RateLimiter,

    /// PWM duty cycle control.
    pub pwm: Pwm,
    pub i_s_ref: Complex32,
    pub psi_s_ref: Complex32,
    pub l_sgm: f32,
    pub r_r: f32,
    pub k_w: Complex32,
    pub w_r_ref: Complex32,
    pub theta_s: f32,
    pub l_m: f32,
    pub k_u: f32,
    pub r_s: f32,
    pub alpha_i: f32,
    pub alpha_f: f32,

    /// Sampling period (s)
    pub t_s: f32,

    /// Current time (s)
    pub t: f32,
}

impl Default for InductionMotorVhzControl<DefaultSpeed> {
    fn default() -> Self {
        let t_s = 250e-6;
        let r_r = 2.1;
        let l_m = 0.224;
        let l_sgm = 0.21;
        let w_rb = r_r * (l_m + l_sgm) / (l_sgm * l_m);

        Self {
            w_m_ref: DefaultSpeed,
            i_s_ref: Complex32::zero(),
            psi_s_ref: Complex32::new(1.04, 1.), // 1 p.u
            l_sgm,
            r_r,
            k_w: Complex32::new(4., 1.),
            w_r_ref: Complex32::zero(),
            rate_limiter: RateLimiter::new(2. * PI * 120.),
            theta_s: 0.,
            pwm: Pwm::default(),
            l_m,
            k_u: 1.,
            r_s: 3.7,
            alpha_i: 0.1 * w_rb,
            alpha_f: 0.1 * w_rb,
            t_s,
            t: 0.,
        }
    }
}

impl<T> InductionMotorVhzControl<T>
where
    T: Speed,
{
    /// Calculate the stator voltage reference.
    pub fn voltage_reference(&self, w_s: Complex32, i_s: f32) -> Complex32 {
        // Nominal magnetizing current
        let i_sd_nom = self.psi_s_ref / (self.l_m + self.l_sgm);

        // Operating-point current for RI compensation
        let i_s_ref0 = i_sd_nom + Complex32::new(1., self.i_s_ref.im);

        // Term -R_s omitted to avoid problems due to the voltage saturation
        // k = -R_s + k_u*L_sgm*(alpha + 1j*w_m0)
        let k = self.k_u * self.l_sgm * (self.r_r / self.l_m + 1. * w_s);

        self.r_s * i_s_ref0 + 1. * w_s * self.psi_s_ref + k * (self.i_s_ref - i_s)
    }
}
