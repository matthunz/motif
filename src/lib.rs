use num::complex::Complex32;

pub mod induction;
pub use induction::InductionMotorVhzControl;

pub mod pwm;
pub use pwm::Pwm;

mod rate_limiter;
pub use rate_limiter::RateLimiter;

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
