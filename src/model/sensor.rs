use core::marker::PhantomData;
use embedded_hal::adc::{Channel, OneShot};
use num_traits::ToPrimitive;

pub trait Sensor {
    fn read(&mut self) -> f32;
}

pub struct AnalogSensor<T, P, A, W> {
    adc: T,
    pin: P,
    from_min: f32,
    from_max: f32,
    to_min: f32,
    to_max: f32,
    _marker: PhantomData<(A, W)>,
}

impl<T, P, A, W> Sensor for AnalogSensor<T, P, A, W>
where
    T: OneShot<A, W, P>,
    P: Channel<A>,
    W: ToPrimitive,
{
    fn read(&mut self) -> f32 {
        let v = self.adc.read(&mut self.pin).ok().unwrap().to_f32().unwrap();

        // Calculate the ratio of the input value relative to the input range
        let ratio = (v - self.from_min) / (self.from_max - self.from_min);

        // Map the ratio to the output range
        self.to_min + (ratio * (self.to_max - self.to_min))
    }
}
