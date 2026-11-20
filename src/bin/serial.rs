#![no_std]
#![no_main]

use ch32_hal::bind_interrupts;
use ch32_hal::gpio::{Level, Output};
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_sync::channel::Channel;
use embassy_time::Timer;

use portable_atomic::Ordering;

use static_cell::StaticCell;

use v203::console_println;
use v203::serial::{serial1_read, serial1_write, RxChannelType, RxReceiverType, PORT1};

static RX_CHANNEL: StaticCell<RxChannelType> = StaticCell::new();

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

    // Create channel for serial handler
    let rx_chan = RX_CHANNEL.init(Channel::new());
    let rx_sender = rx_chan.sender();
    let rx_receiver = rx_chan.receiver();

    // Spawn serial reader/writer/handler tasks
    spawner.spawn(serial1_read(uart_rx, rx_sender).unwrap());
    spawner.spawn(serial1_write(uart_tx).unwrap());
    spawner.spawn(serial_handler(rx_receiver).unwrap());

    // LED
    let mut led = Output::new(p.PA15, Level::Low, Default::default());
    let mut count = 0_u32;
    loop {
        led.toggle();
        console_println!(">> Count: {}", count);
        Timer::after_millis(5000).await;
        count += 1;
    }
}

#[embassy_executor::task]
pub async fn serial_handler(chan: RxReceiverType) {
    loop {
        let line = chan.receive().await;
        if !line.is_empty() {
            let mut it = line.split_ascii_whitespace();
            match it.next() {
                Some("hello") => {
                    console_println!("-- Hello >>{}<<", it.next().unwrap_or("There"))
                }
                Some("echo") => {
                    if it.next().unwrap_or("on") == "on" {
                        console_println!("-- ECHO ON");
                        PORT1.echo.store(true, Ordering::Relaxed);
                    } else {
                        console_println!("-- ECHO OFF");
                        PORT1.echo.store(false, Ordering::Relaxed);
                    }
                }
                Some(rx) => console_println!("-- RX: {}", rx),
                None => {}
            }
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
