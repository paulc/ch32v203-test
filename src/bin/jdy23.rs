#![no_std]
#![no_main]

use ch32_hal::bind_interrupts;
use ch32_hal::gpio::{Level, Output};
use ch32_hal::mode::Async;
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration, Timer};

use portable_atomic::Ordering;

use static_cell::StaticCell;

use v203::console_println;
use v203::serial::{serial1_read, serial1_write, RxChannelType, RxReceiverType, PORT1};

static RX_CHANNEL: StaticCell<RxChannelType> = StaticCell::new();

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
    USART2 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART2>;
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
            panic!("UART");
        }
    };

    // Create channel for serial handler
    let rx_chan = RX_CHANNEL.init(Channel::new());
    let rx_sender = rx_chan.sender();
    let rx_receiver = rx_chan.receiver();

    // Spawn serial reader/writer/handler tasks
    spawner.spawn(serial1_read(uart_rx, rx_sender).unwrap());
    spawner.spawn(serial1_write(uart_tx).unwrap());

    let jdy23_tx = p.PA2;
    let jdy23_rx = p.PA3;
    let jdy23_config = usart::Config::default(); // Default to 115200 - use check_baud to force module

    let (mut jdy23_tx, mut jdy23_rx) = match usart::Uart::new(
        p.USART2,
        jdy23_rx,
        jdy23_tx,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH6,
        jdy23_config,
    ) {
        Ok(uart) => uart.split(),
        Err(_) => {
            panic!("JDY23");
        }
    };

    match check_baud(&mut jdy23_tx, &mut jdy23_rx).await {
        Ok(_) => console_println!(">>> BAUDRATE 115200"),
        Err(e) => console_println!("!!! ERROR SETTING BAUDRATE: {:#?}", e),
    }

    spawner.spawn(serial_handler(rx_receiver, jdy23_tx, jdy23_rx).unwrap());

    // LED
    let mut led = Output::new(p.PA15, Level::Low, Default::default());
    loop {
        led.set_high();
        Timer::after_millis(200).await;
        led.set_low();
        Timer::after_millis(5000).await;
    }
}

#[derive(ufmt::derive::uDebug)]
enum Jdy23Error {
    Timeout,
    UartError,
    ConfigError,
    InvalidSpeed,
    InvalidData,
}

async fn check_baud(
    tx: &mut usart::UartTx<'static, ch32_hal::peripherals::USART2, Async>,
    rx: &mut usart::UartRx<'static, ch32_hal::peripherals::USART2, Async>,
) -> Result<(), Jdy23Error> {
    match send_cmd(tx, rx, b"AT+VER").await {
        Ok(_) => Ok(()),
        Err(Jdy23Error::Timeout) => {
            // Wrong baudrate - switch to 9600 and reset
            let mut jdy23_config = usart::Config::default();
            jdy23_config.baudrate = 9600;
            tx.set_config(&jdy23_config)
                .map_err(|_| Jdy23Error::ConfigError)?;
            rx.set_config(&jdy23_config)
                .map_err(|_| Jdy23Error::ConfigError)?;

            // Set BAUD to 115200
            let r = send_cmd(tx, rx, b"AT+BAUD0").await?;

            // Reset to 115200
            jdy23_config.baudrate = 115200;
            tx.set_config(&jdy23_config)
                .map_err(|_| Jdy23Error::ConfigError)?;
            rx.set_config(&jdy23_config)
                .map_err(|_| Jdy23Error::ConfigError)?;

            // Check OK
            if r.trim_ascii() == "+OK" {
                Timer::after_millis(1000).await; // Wait for module to reconfigure
                Ok(())
            } else {
                Err(Jdy23Error::InvalidSpeed)
            }
        }
        Err(e) => Err(e),
    }
}
async fn send_cmd(
    tx: &mut usart::UartTx<'static, ch32_hal::peripherals::USART2, Async>,
    rx: &mut usart::UartRx<'static, ch32_hal::peripherals::USART2, Async>,
    cmd: &[u8],
) -> Result<heapless::String<32>, Jdy23Error> {
    let mut buf = [0_u8; 32];
    tx.write(cmd).await.map_err(|_| Jdy23Error::UartError)?;
    tx.write(b"\r\n").await.map_err(|_| Jdy23Error::UartError)?;
    let n = match with_timeout(Duration::from_secs(1), rx.read_until_idle(&mut buf)).await {
        Ok(Ok(n)) => Ok(n),
        Ok(Err(_)) => Err(Jdy23Error::UartError),
        Err(_) => Err(Jdy23Error::Timeout),
    }?;
    let v = heapless::Vec::<u8, 32>::from_slice(&buf[..n]).map_err(|_| Jdy23Error::InvalidData)?;
    let s = heapless::String::<32>::from_utf8(v).map_err(|_| Jdy23Error::InvalidData)?;
    Ok(s)
}

#[embassy_executor::task]
pub async fn serial_handler(
    chan: RxReceiverType,
    mut tx: usart::UartTx<'static, ch32_hal::peripherals::USART2, Async>,
    mut rx: usart::UartRx<'static, ch32_hal::peripherals::USART2, Async>,
) {
    console_println!("--- JDY23 ---");

    match send_cmd(&mut tx, &mut rx, b"AT+VER").await {
        Ok(r) => console_println!(">>> {}", r.trim_ascii()),
        Err(e) => console_println!("!!! ERROR: {:#?}", e),
    }

    loop {
        let line = chan.receive().await;
        if !line.is_empty() {
            let mut it = line.split_ascii_whitespace();
            match it.next() {
                Some("send") => {
                    match send_cmd(&mut tx, &mut rx, it.next().unwrap_or("AT+STAT").as_bytes())
                        .await
                    {
                        Ok(r) => console_println!(">>> {}", r.trim_ascii()),
                        Err(e) => console_println!("!!! ERROR: {:#?}", e),
                    }
                }
                Some("reset-baud") => match check_baud(&mut tx, &mut rx).await {
                    Ok(_) => console_println!(">>> OK"),
                    Err(e) => console_println!("!!! ERROR: {:#?}", e),
                },
                Some("echo") => {
                    if it.next().unwrap_or("on") == "on" {
                        console_println!("-- ECHO ON");
                        PORT1.echo.store(true, Ordering::Relaxed);
                    } else {
                        console_println!("-- ECHO OFF");
                        PORT1.echo.store(false, Ordering::Relaxed);
                    }
                }
                Some(rx) => console_println!("-- ERROR: {}", rx),
                None => {}
            }
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
