#![no_std]
#![no_main]

use ch32_hal::gpio::{Level, Output};

use embassy_executor::Spawner;
use embassy_time::Timer;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let config = ch32_hal::Config::default();
    let p = ch32_hal::init(config);

    // LED
    let mut led = Output::new(p.PB2, Level::Low, Default::default());
    loop {
        led.toggle();
        Timer::after_millis(100).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
