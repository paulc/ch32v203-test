#![no_std]
#![no_main]

use ch32_hal::debug::SDIPrint;
use ch32_hal::gpio::{Level, Output};
use ch32_hal::rcc;
use ch32_hal::try_println;

use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    SDIPrint::enable();
    let config = ch32_hal::Config {
        rcc: rcc::Config::SYSCLK_FREQ_96MHZ_HSI,
        ..Default::default()
    };
    let p = ch32_hal::init(config);
    try_println!("INIT");

    Timer::after_millis(500).await;

    // LED
    let mut led = Output::new(p.PA15, Level::Low, Default::default());

    for _ in 0..10 {
        led.toggle();
        Timer::after_millis(50).await;
    }

    loop {
        try_println!("LOOP");
        led.set_high();
        Timer::after_millis(50).await;
        led.set_low();
        Timer::after_millis(2000).await;
    }
}
