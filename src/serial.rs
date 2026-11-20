use ch32_hal::mode::Async;
use ch32_hal::usart;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::pipe::Pipe;

use portable_atomic::{AtomicBool, Ordering};

const PIPE_LEN: usize = 256;
const LINE_BUF_LEN: usize = 64;
const RX_CHANNEL_LEN: usize = 2;

pub type RxChannelType =
    Channel<CriticalSectionRawMutex, heapless::String<LINE_BUF_LEN>, RX_CHANNEL_LEN>;
pub type RxSenderType =
    Sender<'static, CriticalSectionRawMutex, heapless::String<LINE_BUF_LEN>, RX_CHANNEL_LEN>;
pub type RxReceiverType =
    Receiver<'static, CriticalSectionRawMutex, heapless::String<LINE_BUF_LEN>, RX_CHANNEL_LEN>;
pub type RxLine = heapless::String<LINE_BUF_LEN>;
pub type TxPipe = Pipe<CriticalSectionRawMutex, PIPE_LEN>;

pub struct Port {
    pub tx: TxPipe,
    pub echo: AtomicBool,
}

impl Port {
    pub const fn new() -> Self {
        Self {
            tx: Pipe::new(),
            echo: AtomicBool::new(true),
        }
    }
}

pub static PORT1: Port = Port::new();
#[cfg(feature = "usart2")]
pub static PORT2: Port = Port::new();

pub struct TxSink(pub &'static TxPipe);

impl TxSink {
    pub fn push(&self, mut data: &[u8]) -> Result<(), ()> {
        while !data.is_empty() {
            match self.0.try_write(data) {
                Ok(n) => data = &data[n..],
                Err(_) => return Err(()), // pipe full
            }
        }
        Ok(())
    }
}

impl ufmt::uWrite for TxSink {
    type Error = ();
    fn write_str(&mut self, s: &str) -> Result<(), ()> {
        let mut rest = s.as_bytes();
        // Translate '\n' to '\r\n'
        while let Some(i) = rest.iter().position(|&b| b == b'\n') {
            let (head, tail) = rest.split_at(i);
            self.push(head)?;
            if head.last() != Some(&b'\r') {
                self.push(b"\r")?;
            }
            self.push(b"\n")?;
            rest = &tail[1..];
        }
        self.push(rest)
    }
}

/// Print to default console
#[macro_export]
macro_rules! console_println {
    ($($arg:tt)*) => {{ let _ = ::ufmt::uwriteln!(&mut $crate::serial::TxSink(&$crate::serial::PORT1.tx), $($arg)*); }};
}

/// Print to specific port
#[macro_export]
macro_rules! serial_print {
    ($pipe:expr, $($arg:tt)*) => {{ let _ = ::ufmt::uwrite!(&mut $crate::serial::TxSink($pipe), $($arg)*); }};
}

/// Print to specific port
#[macro_export]
macro_rules! serial_println {
    ($pipe:expr, $($arg:tt)*) => {{ let _ = ::ufmt::uwriteln!(&mut $crate::serial::TxSink($pipe), $($arg)*); }};
}

pub async fn read_loop<T: usart::Instance>(
    mut rx: usart::UartRx<'static, T, Async>,
    chan: RxSenderType,
    port: &'static Port,
) {
    let mut buf = [0u8; 16];
    let mut line_buf: RxLine = heapless::String::new();
    loop {
        match rx.read_until_idle(&mut buf).await {
            Ok(n) => {
                if n > 0 && buf[..n].is_ascii() {
                    if port.echo.load(Ordering::Relaxed) {
                        let _ = port.tx.try_write(&buf[..n]);
                        if buf[n - 1] == b'\r' {
                            let _ = port.tx.try_write(b"\n");
                        }
                    }
                    let s = unsafe { str::from_utf8_unchecked(&buf[..n]) };
                    // Check if buffer full
                    if line_buf.push_str(s).is_err() {
                        chan.send(line_buf.clone()).await; // Send line (possibly incomplete)
                        line_buf.clear();
                        let _ = line_buf.push_str(s); // Always safe
                    }
                    // Check for line
                    while let Some(i) = line_buf.find(['\n', '\r']) {
                        let line = line_buf[..i].trim_ascii();
                        if !line.is_empty() {
                            chan.send(RxLine::try_from(line).unwrap()).await;
                        }
                        line_buf.drain(..=i);
                    }
                }
            }
            Err(e) => {
                let msg = match e {
                    usart::Error::Overrun => "overrun",
                    usart::Error::Framing => "framing",
                    usart::Error::Noise => "noise",
                    _ => "other",
                };
                serial_println!(&port.tx, "-- ERR {}", msg);
            }
        }
    }
}

pub async fn write_loop<T: usart::Instance>(
    mut tx: usart::UartTx<'static, T, Async>,
    port: &'static Port,
) {
    let mut b = [0u8; 64];
    loop {
        let n = port.tx.read(&mut b).await;
        let _ = tx.write(&b[..n]).await;
        let _ = tx.flush().await;
    }
}

#[embassy_executor::task]
pub async fn serial1_read(
    rx: usart::UartRx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
    chan: RxSenderType,
) {
    read_loop(rx, chan, &PORT1).await
}

#[embassy_executor::task]
pub async fn serial1_write(
    tx: usart::UartTx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
) {
    write_loop(tx, &PORT1).await
}

#[cfg(feature = "usart2")]
#[embassy_executor::task]
pub async fn serial2_read(
    rx: usart::UartRx<'static, ch32_hal::peripherals::USART2, ch32_hal::mode::Async>,
    chan: RxSenderType,
) {
    read_loop(rx, chan, &PORT2).await
}

#[cfg(feature = "usart2")]
#[embassy_executor::task]
pub async fn serial2_write(
    tx: usart::UartTx<'static, ch32_hal::peripherals::USART2, ch32_hal::mode::Async>,
) {
    write_loop(tx, &PORT2).await
}
