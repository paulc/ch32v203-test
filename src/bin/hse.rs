#![no_std]
#![no_main]

use ch32_hal::gpio::{Level, Output};

use embassy_executor::Spawner;
use embassy_time::Timer;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE;
    let p = ch32_hal::init(config);

    // LED
    let mut led = Output::new(p.PA15, Level::Low, Default::default());
    for _ in 0..10 {
        led.toggle();
        Timer::after_millis(100).await;
    }
    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
