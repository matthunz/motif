use core::marker::PhantomData;
use embedded_hal::adc::{Channel, OneShot};
use num_traits::ToPrimitive;

pub struct AnalogSensor<P> {
    pub pin: P,
    pub from_min: f32,
    pub from_max: f32,
    pub to_min: f32,
    pub to_max: f32,
}

impl<P> AnalogSensor<P> {
    pub fn read<T, A, W>(&mut self, adc: &mut T) -> f32
    where
        T: OneShot<A, W, P>,
        P: Channel<A>,
        W: ToPrimitive,
    {
        let v = adc.read(&mut self.pin).ok().unwrap().to_f32().unwrap();

        // Calculate the ratio of the input value relative to the input range
        let ratio = (v - self.from_min) / (self.from_max - self.from_min);

        // Map the ratio to the output range
        self.to_min + (ratio * (self.to_max - self.to_min))
    }
}
