mod sensor;
pub use sensor::{AnalogSensor, Sensor};

pub trait Model {
    fn phase_currents(&mut self) -> [f32; 3];

    fn dc_bus_voltage(&mut self) -> f32;
}

pub trait SensoredModel {
    fn speed(&self) -> f32;
}

pub struct MotorModel<T, U> {
    phase_current_sensors: [T; 3],
    dc_bus_sensor: U,
}

impl<T, U> Model for MotorModel<T, U>
where
    T: Sensor,
    U: Sensor,
{
    fn phase_currents(&mut self) -> [f32; 3] {
        [
            self.phase_current_sensors[0].read(),
            self.phase_current_sensors[1].read(),
            self.phase_current_sensors[2].read(),
        ]
    }

    fn dc_bus_voltage(&mut self) -> f32 {
        self.dc_bus_sensor.read()
    }
}
