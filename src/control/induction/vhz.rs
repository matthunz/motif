use crate::{
    abc_to_complex,
    control::{Pwm, RateLimiter},
    Control, Model,
};
use core::f32::consts::PI;
use num_complex::{Complex32, ComplexFloat};
use num_traits::{Float, Zero};

pub struct Builder {
    r_r: f32,
    l_m: f32,
    l_sgm: f32,
}

impl Default for Builder {
    fn default() -> Self {
        Self {
            r_r: 2.1,
            l_m: 0.224,
            l_sgm: 0.21,
        }
    }
}

impl Builder {
    pub fn r_r(mut self, r_r: f32) -> Self {
        self.r_r = r_r;
        self
    }

    pub fn l_m(mut self, l_m: f32) -> Self {
        self.l_m = l_m;
        self
    }
    pub fn l_sgm(mut self, l_sgm: f32) -> Self {
        self.l_sgm = l_sgm;
        self
    }

    pub fn build(self) -> InductionMotorVhzControl {
        let w_rb = self.r_r * (self.l_m + self.l_sgm) / (self.l_sgm * self.l_m);

        InductionMotorVhzControl {
            i_s_ref: Complex32::zero(),
            psi_s_ref: Complex32::new(1.04, 1.), // 1 p.u
            w_r_ref: Complex32::zero(),
            l_sgm: self.l_sgm,
            r_r: self.r_r,
            k_w: Complex32::new(4., 1.),
            rate_limiter: RateLimiter::new(2. * PI * 120.),
            theta_s: 0.,
            pwm: Pwm::default(),
            l_m: self.l_m,
            k_u: 1.,
            r_s: 3.7,
            alpha_i: 0.1 * w_rb,
            alpha_f: 0.1 * w_rb,
        }
    }
}

/// V/Hz control for induction motor drives.
pub struct InductionMotorVhzControl {
    /// Frequency reference rate limiter.
    pub rate_limiter: RateLimiter,

    /// PWM duty cycle control.
    pub pwm: Pwm,

    /// Stator inductance
    pub l_sgm: f32,

    pub k_w: Complex32,

    pub k_u: f32,

    /// Magnetizing inductance
    pub l_m: f32,

    /// Stator resistance
    pub r_s: f32,

    /// Rotor resistance
    pub r_r: f32,

    /// Reference rotor speed
    w_r_ref: Complex32,

    /// Reference stator current
    i_s_ref: Complex32,

    /// Reference stator flux
    psi_s_ref: Complex32,

    /// Angle of stator flux
    theta_s: f32,

    /// Proportional gain for the stator current controller.
    alpha_i: f32,

    /// Proportional gain for the stator flux controller.
    alpha_f: f32,
}

impl Default for InductionMotorVhzControl {
    fn default() -> Self {
        Self::builder().build()
    }
}

impl InductionMotorVhzControl {
    pub fn builder() -> Builder {
        Builder::default()
    }

    /// Calculate the stator the dynamic stator frequency (used in the coordinate transformations).
    pub fn stator_freq(&self, w_s_ref: Complex32, i_s: Complex32) -> [Complex32; 2] {
        // Operating-point quantities
        let psi_r_ref = self.psi_s_ref - self.l_sgm * self.i_s_ref;
        let psi_r_ref_sqr = Float::powi(psi_r_ref.abs(), 2);

        // Compute the dynamic stator frequency
        if psi_r_ref_sqr > 0. {
            // Slip estimate based on the measured current
            let w_r = self.r_r * (i_s * psi_r_ref.conj()).im / psi_r_ref_sqr;
            // Dynamic frequency
            let w_s = w_s_ref + self.k_w * (self.w_r_ref - w_r);

            [w_s, w_r.into()]
        } else {
            [Complex32::zero(); 2]
        }
    }

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

impl<M: Model> Control<M> for InductionMotorVhzControl {
    fn control(&mut self, drive: &mut M, w_m_ref: f32, t_s: f32) -> [f32; 3] {
        let i_s_abc = drive.phase_currents();
        let u_dc = drive.dc_bus_voltage();

        // Rate limit the frequency reference
        let w_m_ref = self.rate_limiter.rate_limit(t_s, w_m_ref);

        // Space vector transformation
        let i_s = Float::exp(-1. * self.theta_s) * abc_to_complex(i_s_abc);

        // Slip compensation
        let w_s_ref = w_m_ref + self.w_r_ref;

        // Dynamic stator frequency and slip frequency
        let [w_s, w_r] = self.stator_freq(w_s_ref, i_s.into());

        // Voltage reference
        let u_s_ref = self.voltage_reference(w_s, i_s);

        // Compute the duty ratios
        let d_abc_ref = self
            .pwm
            .duty_ratios(t_s, u_s_ref, u_dc, self.theta_s, w_s.re);

        // Update the states
        self.i_s_ref += t_s * self.alpha_i * (i_s - self.i_s_ref);
        self.w_r_ref += t_s * self.alpha_f * (w_r - self.w_r_ref);

        // Next line: limit into [-pi, pi)
        self.theta_s += (t_s * w_s).re;
        self.theta_s = (self.theta_s + PI) % (2. * PI) - PI;

        d_abc_ref
    }
}
