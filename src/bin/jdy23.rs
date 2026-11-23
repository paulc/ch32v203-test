#![no_std]
#![no_main]

//
// This example connects to a JDY23 BLE/Serial module over USART2
// with a simple CLI on the serial console (USART1). When the JDY23
// is connected the CLI implements a transparent TX/RX mode.
//
// The JDT23 always starts at 9600 baud when powered on and after reset
// (AT+RST) but will retain the set baud rate while powered on so we
// dont know what starting state it is in - `check_baud` forces to 115200
//
// Chainging the BLE params (AT+NAME/AT+ADVIN/AT+MAC etc) only takes
// effect after reset (AT+RST)
//
// (Note MacOS caches the device name so you need to change MAC as well)
//
use ch32_hal::bind_interrupts;
use ch32_hal::exti::ExtiInput;
use ch32_hal::gpio::{Level, Output, Pull};
use ch32_hal::usart;
use ch32_hal::wdg::IndependentWatchdog;

use embassy_executor::Spawner;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};

use static_cell::StaticCell;

use v203::console_println;
use v203::power::{reset_cause, ResetCause};
use v203::serial::{serial1_read, serial1_write, RxChannelType};

use crate::jdy23::{serial_handler, status_handler};

static RX_CHANNEL: StaticCell<RxChannelType> = StaticCell::new();
bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
    USART2 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART2>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let reset = reset_cause(); // first thing, before init

    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI;
    let p = ch32_hal::init(config);

    let mut wdg = IndependentWatchdog::new(p.IWDG, 5_000_000); // 5s
    wdg.unleash();

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

    console_println!(">>> INIT: {}", reset.cause.as_str());

    if reset.cause == ResetCause::PowerOn {
        // Allow JDY-23 module to initialise
        Timer::after_millis(1000).await;
    }
    // Create JDY23 uart
    let jdy23_tx = p.PA2;
    let jdy23_rx = p.PA3;
    let jdy23_config = usart::Config::default(); // Default to 115200 - use check_baud to force module

    let (jdy23_tx, jdy23_rx) = match usart::Uart::new(
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

    console_println!(r"      _ ______   __   ____  _____ ");
    console_println!(r"     | |  _ \ \ / /  |___ \|___ / ");
    console_println!(r"  _  | | | | \ V /____ __) | |_ \ ");
    console_println!(r" | |_| | |_| || |_____/ __/ ___) |");
    console_println!(r"  \___/|____/ |_|    |_____|____/ ");
    console_println!(r"                                  ");

    // ALlow serial port to flush
    Timer::after_millis(100).await;

    spawner.spawn(serial_handler(rx_receiver, jdy23_tx, jdy23_rx).unwrap());

    let status = ExtiInput::new(p.PA0, p.EXTI0, Pull::None);
    spawner.spawn(status_handler(status).unwrap());

    // LED
    let mut led = Output::new(p.PA15, Level::Low, Default::default());
    let mut ticker = Ticker::every(Duration::from_millis(2000));
    loop {
        wdg.pet();
        led.set_high();
        Timer::after_millis(200).await;
        led.set_low();
        ticker.next().await;
    }
}

mod jdy23 {
    use ch32_hal::exti::ExtiInput;
    use ch32_hal::mode::Async;
    use ch32_hal::signature::unique_id;
    use ch32_hal::usart;

    use embassy_futures::select::{select, select3, Either, Either3};
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;
    use embassy_time::{with_timeout, Duration, Timer};

    use portable_atomic::{AtomicBool, Ordering};

    use v203::console_println;
    use v203::serial::{RxReceiverType, TxSink, PORT1};

    static CONNECTED_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    static CONNECTED: AtomicBool = AtomicBool::new(false);

    #[derive(ufmt::derive::uDebug)]
    pub enum Jdy23Error {
        Timeout,
        UartError,
        ConfigError,
        InvalidSpeed,
        InvalidData,
    }

    #[embassy_executor::task]
    pub async fn status_handler(mut status: ExtiInput<'static>) {
        loop {
            status.wait_for_high().await;
            Timer::after_millis(100).await; // Debounce
            if status.is_high() {
                CONNECTED.store(true, Ordering::Relaxed);
                CONNECTED_SIGNAL.signal(());
                console_println!(">>> CONNECTED");
                status.wait_for_low().await;
                CONNECTED.store(false, Ordering::Relaxed);
                console_println!(">>> DISCONNECTED");
            }
        }
    }

    #[panic_handler]
    fn panic(_info: &core::panic::PanicInfo) -> ! {
        loop {}
    }

    #[embassy_executor::task]
    pub async fn serial_handler(
        chan: RxReceiverType,
        mut tx: usart::UartTx<'static, ch32_hal::peripherals::USART2, Async>,
        mut rx: usart::UartRx<'static, ch32_hal::peripherals::USART2, Async>,
    ) {
        if let Err(e) = jdy23_config(&mut tx, &mut rx).await {
            console_println!("!!! ERROR: {:#?}", e);
            panic!("JDY23_CONFIG");
        }
        console_println!("----- COMMAND MODE ------");
        loop {
            match select(chan.receive(), CONNECTED_SIGNAL.wait()).await {
                Either::First(line) => {
                    // Command mode
                    if !line.is_empty() {
                        let mut it = line.split_ascii_whitespace();
                        match it.next() {
                            Some("send") => {
                                match send_cmd(
                                    &mut tx,
                                    &mut rx,
                                    it.next().unwrap_or("AT+STAT").as_bytes(),
                                )
                                .await
                                {
                                    Ok(r) => console_println!(">>> {}", r.trim_ascii()),
                                    Err(e) => console_println!("!!! ERROR: {:#?}", e),
                                }
                            }
                            Some("status") => {
                                console_println!(
                                    ">>> CONNECTED: {}",
                                    CONNECTED.load(Ordering::Relaxed)
                                )
                            }
                            Some("reset") => match jdy23_config(&mut tx, &mut rx).await {
                                Ok(_) => console_println!(">>> CONFIG OK"),
                                Err(e) => console_println!(">>> CONFIG ERROR: {:#?}", e),
                            },
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
                Either::Second(_) => {
                    // Transparent Mode
                    console_println!("----- TRANSPARENT MODE ------");
                    let mut buf = [0_u8; 64];
                    while CONNECTED.load(Ordering::Relaxed) {
                        match select3(
                            rx.read_until_idle(&mut buf),
                            chan.receive(),
                            Timer::after_millis(500),
                        )
                        .await
                        {
                            Either3::First(Ok(n)) => {
                                if &buf[..n] == b"+DISCONNECT\r\n" {
                                    break;
                                }
                                let _ = TxSink(&PORT1.tx).push(&buf[..n]);
                            }
                            Either3::First(Err(_)) => {}
                            Either3::Second(line) => {
                                let _ = tx.write(line.as_bytes()).await;
                                let _ = tx.write(b"\r\n").await;
                            }
                            Either3::Third(_) => {
                                // Timeout to force CONNECTED check
                            }
                        }
                    }
                    console_println!("----- COMMAND MODE ------");
                }
            }
        }
    }

    async fn jdy23_config(
        tx: &mut usart::UartTx<'static, ch32_hal::peripherals::USART2, Async>,
        rx: &mut usart::UartRx<'static, ch32_hal::peripherals::USART2, Async>,
    ) -> Result<(), Jdy23Error> {
        async {
            console_println!("----- JDY23 CONFIG ------");

            // Generate device name and mac from UID
            let mut device_name = heapless::String::<16>::new();
            let _ = ufmt::uwrite!(&mut device_name, "JDY-23-");
            for b in unique_id().iter().take(4) {
                let _ = ufmt::uwrite!(&mut device_name, "{:02X}", *b);
            }

            let mut device_mac = heapless::String::<12>::new();
            for b in unique_id().iter().take(6) {
                let _ = ufmt::uwrite!(&mut device_mac, "{:02X}", *b);
            }

            let mut name = heapless::String::<24>::new();
            let mut set_name = heapless::String::<24>::new();
            let _ = ufmt::uwrite!(&mut name, "+NAME:{}", device_name);
            let _ = ufmt::uwrite!(&mut set_name, "AT+NAME{}", device_name);

            let mut mac = heapless::String::<24>::new();
            let mut set_mac = heapless::String::<24>::new();
            // JDY23 AT+MAC prepends 00 to MAC and uses = vs : (?)
            let _ = ufmt::uwrite!(&mut mac, "+MAC=00{}", device_mac);
            let _ = ufmt::uwrite!(&mut set_mac, "AT+MAC{}", device_mac);

            // Force JDY23 baudrate to 115200
            check_baud(tx, rx).await?;

            // Make sure we are disconnected
            let _ = send_cmd(tx, rx, b"AT+DISC").await?;

            // Get Version
            let r = send_cmd(tx, rx, b"AT+VER").await?;
            console_println!("... AT+VER : {}", r.trim_ascii());

            // Check config
            let mut update = false;
            for (cmd, val, set) in [
                ("AT+NAME", name.as_str(), set_name.as_str()),
                ("AT+MAC", mac.as_str(), set_mac.as_str()),
                ("AT+ADVIN", "+ADVIN:4", "AT+ADVIN4"),
                ("AT+STARTEN", "+STARTEN:0", "AT+STARTEN0"),
                ("AT+HOSTEN", "+HOSTEN:0", "AT+HOSTEN0"),
            ] {
                let r = send_cmd(tx, rx, cmd.as_bytes()).await?;
                console_println!("... {} : {}", cmd, r.trim_ascii());
                if r.trim_ascii() != val {
                    // console_println!(">{}< !=  >{}<", r.trim_ascii(), val);
                    let r = send_cmd(tx, rx, set.as_bytes()).await?;
                    update = true;
                    console_println!("+++ {} : {}", set, r.trim_ascii());
                }
            }
            // If we have updated any value reset device
            if update {
                let _ = send_cmd(tx, rx, b"AT+RST").await?;
                // Wait for module to reconfigure
                Timer::after_millis(1000).await;
                // Reset baudrate after AT+RST
                check_baud(tx, rx).await?;
            }
            Ok::<(), Jdy23Error>(())
        }
        .await
    }

    async fn check_baud(
        tx: &mut usart::UartTx<'static, ch32_hal::peripherals::USART2, Async>,
        rx: &mut usart::UartRx<'static, ch32_hal::peripherals::USART2, Async>,
    ) -> Result<(), Jdy23Error> {
        match send_cmd(tx, rx, b"AT+VER").await {
            Ok(_) => Ok(()),
            Err(Jdy23Error::Timeout) => {
                // Wrong baudrate - set port to 9600
                let mut jdy23_config = usart::Config::default();
                jdy23_config.baudrate = 9600;
                tx.set_config(&jdy23_config)
                    .map_err(|_| Jdy23Error::ConfigError)?;
                rx.set_config(&jdy23_config)
                    .map_err(|_| Jdy23Error::ConfigError)?;

                // Set JDY23 to 115200 (BAUD0)
                let r = send_cmd(tx, rx, b"AT+BAUD0").await?;

                // Reset port to 115200
                jdy23_config.baudrate = 115200;
                tx.set_config(&jdy23_config)
                    .map_err(|_| Jdy23Error::ConfigError)?;
                rx.set_config(&jdy23_config)
                    .map_err(|_| Jdy23Error::ConfigError)?;

                // Check OK
                if r.trim_ascii() == "+OK" {
                    Timer::after_millis(1000).await; // Wait for module to reconfigure
                    console_println!("... SET BAUD: 115200");
                    Ok(())
                } else {
                    Err(Jdy23Error::InvalidSpeed)
                }
            }
            Err(e) => {
                console_println!(">>> ERR: {:#?}", e);
                Err(e)
            }
        }
    }

    async fn send_cmd(
        tx: &mut usart::UartTx<'static, ch32_hal::peripherals::USART2, Async>,
        rx: &mut usart::UartRx<'static, ch32_hal::peripherals::USART2, Async>,
        cmd: &[u8],
    ) -> Result<heapless::String<32>, Jdy23Error> {
        let mut buf = [0_u8; 32];
        let transaction = async {
            tx.write(cmd).await.map_err(|_| Jdy23Error::UartError)?;
            tx.write(b"\r\n").await.map_err(|_| Jdy23Error::UartError)?;
            let n = rx
                .read_until_idle(&mut buf)
                .await
                .map_err(|_| Jdy23Error::UartError)?;
            Ok::<usize, Jdy23Error>(n)
        };
        let n = match with_timeout(Duration::from_millis(1000), transaction).await {
            Ok(Ok(n)) => Ok(n),
            Ok(Err(_)) => Err(Jdy23Error::UartError),
            Err(_) => Err(Jdy23Error::Timeout),
        }?;
        let v =
            heapless::Vec::<u8, 32>::from_slice(&buf[..n]).map_err(|_| Jdy23Error::InvalidData)?;
        let s = heapless::String::<32>::from_utf8(v).map_err(|_| Jdy23Error::InvalidData)?;
        Ok(s)
    }
}
