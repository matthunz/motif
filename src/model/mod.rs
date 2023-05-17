use core::marker::PhantomData;
use embedded_hal::adc::{Channel, OneShot};
use num_traits::ToPrimitive;

mod sensor;
pub use sensor::Sensor;

pub trait Model {
    fn phase_currents(&mut self) -> [f32; 3];

    fn dc_bus_voltage(&mut self) -> f32;
}

pub trait SensoredModel {
    fn speed(&self) -> f32;
}

pub struct MotorModel<T, X, Y, Z, U, A, W> {
    adc: T,
    phase_current_sensors: (Sensor<X>, Sensor<Y>, Sensor<Z>),
    dc_bus_sensor: Sensor<U>,
    _marker: PhantomData<(A, W)>,
}

impl<T, X, Y, Z, U, A, W> MotorModel<T, X, Y, Z, U, A, W> {
    pub fn new(
        adc: T,
        phase_current_sensors: (Sensor<X>, Sensor<Y>, Sensor<Z>),
        dc_bus_sensor: Sensor<U>,
    ) -> Self {
        Self {
            adc,
            phase_current_sensors,
            dc_bus_sensor,
            _marker: PhantomData,
        }
    }
}

impl<T, X, Y, Z, U, A, W> Model for MotorModel<T, X, Y, Z, U, A, W>
where
    T: OneShot<A, W, X> + OneShot<A, W, Y> + OneShot<A, W, Z> + OneShot<A, W, U>,
    X: Channel<A>,
    Y: Channel<A>,
    Z: Channel<A>,
    U: Channel<A>,
    W: ToPrimitive,
{
    fn phase_currents(&mut self) -> [f32; 3] {
        let (x, y, z) = &mut self.phase_current_sensors;
        [
            x.read(&mut self.adc),
            y.read(&mut self.adc),
            z.read(&mut self.adc),
        ]
    }

    fn dc_bus_voltage(&mut self) -> f32 {
        self.dc_bus_sensor.read(&mut self.adc)
    }
}
