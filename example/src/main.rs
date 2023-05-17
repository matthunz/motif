#![no_main]
#![no_std]

use cortex_m_rt::entry;
use motif::control::InductionMotorVhzControl;
use motif::model::{AnalogSensor, Model, MotorModel};
use motif::{Drive, Motor, MotorDrive};
use panic_halt as _;
use stm32f1xx_hal::adc::Adc;
use stm32f1xx_hal::device::Peripherals;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::timer::{Channel, Tim2NoRemap};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    let mut flash = peripherals.FLASH.constrain();
    let rcc = peripherals.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = peripherals.AFIO.constrain();
    let mut gpioa = peripherals.GPIOA.split();

    let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);

    let mut pwm = peripherals.TIM2.pwm_hz::<Tim2NoRemap, _, _>(
        (c1, c2, c3),
        &mut afio.mapr,
        1.kHz(),
        &clocks,
    );
    let channels = [Channel::C1, Channel::C2, Channel::C3];
    for channel in channels {
        pwm.enable(channel);
    }
    let drive = MotorDrive::new(pwm, channels);

    let adc1 = Adc::adc1(peripherals.ADC1, clocks);

    // Setup GPIOB
    let mut gpiob = peripherals.GPIOB.split();

    // Configure pb0, pb1 as an analog input
    let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
    let ch1 = gpiob.pb1.into_analog(&mut gpiob.crl);
    let ch2 = gpioa.pa5.into_analog(&mut gpioa.crl);
    let ch3 = gpioa.pa6.into_analog(&mut gpioa.crl);
    let a = AnalogSensor {
        pin: ch0,
        from_min: 0.,
        from_max: 1.,
        to_min: 0.,
        to_max: 24.,
    };
    let b = AnalogSensor {
        pin: ch1,
        from_min: 0.,
        from_max: 1.,
        to_min: 0.,
        to_max: 24.,
    };
    let c = AnalogSensor {
        pin: ch2,
        from_min: 0.,
        from_max: 1.,
        to_min: 0.,
        to_max: 24.,
    };
    let d = AnalogSensor {
        pin: ch3,
        from_min: 0.,
        from_max: 1.,
        to_min: 0.,
        to_max: 12.,
    };
    let model = MotorModel::<_, _, _, _, _, _, u16>::new(adc1, (a, b, c), d);

    let control = InductionMotorVhzControl::default();

    let mut motor = Motor {
        model,
        control,
        drive,
        w_m_ref: 0.,
        is_armed: false,
    };

    loop {
        motor.control(0.);
    }
}
