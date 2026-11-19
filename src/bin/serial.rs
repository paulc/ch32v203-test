#![no_std]
#![no_main]

use ch32_hal::bind_interrupts;
use ch32_hal::gpio::{Level, Output};
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_time::Timer;

use v203::serial::{serial_read, serial_write};
use v203::serial_println;

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI;
    let p = ch32_hal::init(config);

    let uart_tx = p.PA9;
    let uart_rx = p.PA10;

    let uart_config = usart::Config::default(); // 115200,N,8,1
    let (uart_tx, uart_rx) = match usart::Uart::new(
        p.USART1,
        uart_rx,
        uart_tx,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        uart_config,
    ) {
        Ok(uart) => uart.split(),
        Err(_) => {
            panic!("UART_TX");
        }
    };

    // Spawn serial reader/writer tasks
    spawner.spawn(serial_read(uart_rx).unwrap());
    spawner.spawn(serial_write(uart_tx).unwrap());

    // LED
    let mut led = Output::new(p.PA15, Level::Low, Default::default());
    let mut count = 0_u32;
    loop {
        led.toggle();
        serial_println!(">> Count: {}", count);
        Timer::after_millis(2000).await;
        count += 1;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
