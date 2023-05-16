use crate::{abc_to_complex, Pwm, RateLimiter};
use num::{complex::Complex32, Zero};
use std::f32::consts::PI;

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
            l_sgm: self.l_sgm,
            r_r: self.r_r,
            k_w: Complex32::new(4., 1.),
            w_r_ref: Complex32::zero(),
            rate_limiter: RateLimiter::new(2. * PI * 120.),
            theta_s: 0.,
            pwm: Pwm::default(),
            l_m: self.l_m,
            k_u: 1.,
            r_s: 3.7,
            alpha_i: 0.1 * w_rb,
            alpha_f: 0.1 * w_rb,
            t: 0.,
        }
    }
}

/// V/Hz control for induction motor drives.
pub struct InductionMotorVhzControl {
    /// Frequency reference rate limiter.
    pub rate_limiter: RateLimiter,

    /// PWM duty cycle control.
    pub pwm: Pwm,

    /// Reference stator current
    pub i_s_ref: Complex32,

    /// Reference stator flux
    pub psi_s_ref: Complex32,

    /// Stator inductance
    pub l_sgm: f32,

    pub k_w: Complex32,

    pub k_u: f32,

    /// Reference rotor speed
    pub w_r_ref: Complex32,

    /// Angle of stator flux
    pub theta_s: f32,

    /// Magnetizing inductance
    pub l_m: f32,

    /// Stator resistance
    pub r_s: f32,

    /// Rotor resistance
    pub r_r: f32,

    /// Proportional gain for the stator current controller.
    pub alpha_i: f32,

    /// Proportional gain for the stator flux controller.
    pub alpha_f: f32,

    // Last recorded time
    pub t: f32,
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

    /// Calculate the 3-phase PWM duty cycle ratios to control the motor.
    /// Arguments:
    /// `i_i_abc`: Phase currents of the motor.
    /// `u_dc`: DC-bus voltage.
    /// `w_m_ref`: Speed reference (in electrical rad/s).
    /// `t`: Current time (in seconds).
    pub fn control(
        &mut self,
        i_s_abc: [f32; 3],
        u_dc: f32,
        w_m_ref: f32,
        t: f32,
    ) -> (f32, [f32; 3]) {
        // Calculate the sampling frequency
        let t_s = t - self.t;
        self.t = t;

        // Rate limit the frequency reference
        let w_m_ref = self.rate_limiter.rate_limit(t_s, w_m_ref);

        // Space vector transformation
        let i_s = (-1. * self.theta_s).exp() * abc_to_complex(i_s_abc);

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

        (t_s, d_abc_ref)
    }

    /// Calculate the stator the dynamic stator frequency (used in the coordinate transformations).
    pub fn stator_freq(&self, i_s: Complex32, w_s_ref: Complex32) -> [Complex32; 2] {
        // Operating-point quantities
        let psi_r_ref = self.psi_s_ref - self.l_sgm * self.i_s_ref;
        let psi_r_ref_sqr = psi_r_ref.powi(2);

        // Compute the dynamic stator frequency
        if psi_r_ref_sqr.re > 0. {
            // Slip estimate based on the measured current
            let w_r = self.r_r * (i_s * psi_r_ref.conj()).im / psi_r_ref_sqr;
            // Dynamic frequency
            let w_s = w_s_ref + self.k_w * (self.w_r_ref - w_r);

            [w_s, w_r]
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
