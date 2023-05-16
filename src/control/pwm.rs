use crate::{abc_to_complex, complex_to_abc};
use core::f32::consts::PI;
use num_complex::{Complex32, ComplexFloat};
use num_traits::{Float, Zero};

/// Duty ratio references and realized voltage for three-phase PWM.
/// This contains the computation of the duty ratio references and the realized voltage.
/// The digital delay effects are taken into account in the realized voltage.
pub struct Pwm {
    is_six_step: bool,
    realized_voltage: Complex32,
    u_ref_lim_old: Complex32,
}

impl Default for Pwm {
    fn default() -> Self {
        Self::new(false)
    }
}

impl Pwm {
    pub fn new(is_six_step: bool) -> Self {
        Self {
            is_six_step,
            realized_voltage: Zero::zero(),
            u_ref_lim_old: Zero::zero(),
        }
    }

    /// Calculate the duty ratios and update the state.
    /// Arguments:
    /// `u_ref` : Voltage reference in synchronous coordinates.
    /// `u_dc` : DC-bus voltage.
    /// theta : Angle of synchronous coordinates.
    /// w : Angular speed of synchronous coordinates.
    pub fn duty_ratios(
        &mut self,
        t_s: f32,
        u_ref: Complex32,
        u_dc: f32,
        theta: f32,
        w: f32,
    ) -> [f32; 3] {
        let (d_abc_ref, u_ref_lim) = self.output(t_s, u_ref, u_dc, theta, w);
        self.update(u_ref_lim);

        d_abc_ref
    }

    /// Calculate the duty ratio limited voltage reference.
    pub fn output(
        &self,
        t_s: f32,
        u_ref: Complex32,
        u_dc: f32,
        theta: f32,
        w: f32,
    ) -> ([f32; 3], Complex32) {
        //  Advance the angle due to the computational delay (T_s) and the ZOH (PWM) delay (0.5*T_s)
        let theta_comp = theta + 1.5 * t_s * w;

        // Voltage reference in stator coordinates
        let mut u_s_ref = Float::exp(theta_comp) * u_ref;

        // Modify angle in the overmodulation region
        if self.is_six_step {
            u_s_ref = six_step_overmodulation(u_s_ref, u_dc);
        }

        // Duty ratios
        let d_abc_ref = duty_ratios(u_s_ref, u_dc);

        // Realizable voltage
        let u_s_ref_lim = abc_to_complex(d_abc_ref) * u_dc;
        let u_ref_lim = Float::exp(-1. * theta_comp) * u_s_ref_lim;

        (d_abc_ref, u_ref_lim.into())
    }

    /// Update the voltage estimate for the next sampling instant.
    pub fn update(&mut self, u_ref_lim: Complex32) {
        self.realized_voltage = 0.5 * (self.u_ref_lim_old + u_ref_lim);
        self.u_ref_lim_old = u_ref_lim;
    }
}

pub fn six_step_overmodulation(u_s_ref: Complex32, u_dc: f32) -> Complex32 {
    // Limited magnitude
    let r = u_s_ref.abs().min(2. / 3. * u_dc);

    if Float::sqrt(3.) * r > u_dc {
        // Angle and sector of the reference vector
        let theta = u_s_ref.arg();
        let sector = (3. * theta / PI).floor();

        // Angle reduced to the first sector (at which sector == 0)
        let mut theta0 = theta - sector * PI / 3.;

        // Intersection angle, see Eq. (9)
        let alpha_g = PI / 6. - Float::acos(u_dc / (Float::sqrt(3.) * r));

        // Modify the angle according to Eq. (4)
        if alpha_g <= theta0 && theta0 <= PI / 6. {
            theta0 = alpha_g;
        } else if PI / 6. <= theta0 && theta0 <= PI / 3. - alpha_g {
            theta0 = PI / 3. - alpha_g;
        }

        // Modified reference voltage
        (r * Float::exp(theta0 + sector * PI / 3.)).into()
    } else {
        u_s_ref
    }
}

/// Calculate the duty ratios for three-phase PWM.
/// This computes the duty ratios using a symmetrical suboscillation method.
/// This method is identical to the standard space-vector PWM.
///
/// Arguments
/// `u_s_ref`: Voltage reference in stator coordinates.
/// `u_dc` : DC-bus voltage.
pub fn duty_ratios(u_s_ref: Complex32, u_dc: f32) -> [f32; 3] {
    // Phase voltages without the zero-sequence voltage
    let mut u_abc = complex_to_abc(u_s_ref);

    // Symmetrization by adding the zero-sequence voltage
    let u_0 = 0.5 * (u_abc[0].max(u_abc[1]).max(u_abc[2]) + u_abc[0].min(u_abc[1]).min(u_abc[2]));
    u_abc = [u_abc[0] - u_0, u_abc[1] - u_0, u_abc[2] - u_0];

    // Preventing overmodulation by means of a minimum phase error method
    let m = (2. / u_dc) * u_abc[0].max(u_abc[1]).max(u_abc[2]);
    if m > 1. {
        u_abc = [u_abc[0] / m, u_abc[1] / m, u_abc[2] / m];
    }

    // Duty ratios
    [
        u_abc[0] / u_dc + 0.5,
        u_abc[1] / u_dc + 0.5,
        u_abc[2] / u_dc + 0.5,
    ]
}
