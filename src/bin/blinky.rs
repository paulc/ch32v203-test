#![no_std]
#![no_main]

use ch32_hal::debug::SDIPrint;
use ch32_hal::gpio::{Level, Output};
use ch32_hal::try_println;

use v203::lowpower;

use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;

const RCC_CFG: ch32_hal::rcc::Config = ch32_hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSI;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    SDIPrint::enable();
    let config = ch32_hal::Config {
        rcc: RCC_CFG,
        ..Default::default()
    };
    let p = ch32_hal::init(config);
    try_println!("INIT");

    Timer::after_millis(500).await;

    lowpower::gpio_all_pulldown(false);

    let from_standby = lowpower::woke_from_standby();
    try_println!("FROM_STANDBY: {}", from_standby);

    // LED
    let mut led = Output::new(p.PA15, Level::Low, Default::default());

    if !from_standby {
        for _ in 0..10 {
            led.toggle();
            Timer::after_millis(50).await;
        }
    }

    lowpower::rtc_init().unwrap();
    try_println!("LOW_POWER > RTC_INIT");

    loop {
        led.set_high();
        lowpower::stop_for_ms(500, RCC_CFG).unwrap();
        led.set_low();
        lowpower::standby_for_ms(5000, false, false);
    }
}
