#![no_main]
#![no_std]

use cortex_m_rt::entry;
use motif::{MotorDrive, Drive};
use panic_halt as _;
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
    let mut drive = MotorDrive::new(pwm, channels);
    drive.drive([1.; 3]);

    loop {}
}
