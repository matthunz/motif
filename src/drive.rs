use embedded_hal::Pwm;
use num_traits::{Float, FromPrimitive, ToPrimitive};

pub trait Drive {
    fn drive(&mut self, duty_cycle_ratios: [f32; 3]);
}

pub struct MotorDrive<T, C> {
    pub pwm: T,
    pub channels: [C; 3],
}

impl<T, C> MotorDrive<T, C> {
    pub fn new(pwm: T, channels: [C; 3]) -> Self {
        Self { pwm, channels }
    }
}

impl<T, C> Drive for MotorDrive<T, C>
where
    T: Pwm<Channel = C>,
    T::Duty: FromPrimitive + ToPrimitive,
    C: Clone,
{
    fn drive(&mut self, duty_cycle_ratios: [f32; 3]) {
        for (channel, duty_cycle_ratio) in self.channels.iter().cloned().zip(duty_cycle_ratios) {
            let duty = self.pwm.get_max_duty().to_f32().unwrap() * duty_cycle_ratio;
            self.pwm
                .set_duty(channel, T::Duty::from_f32(duty.round()).unwrap())
        }
    }
}
