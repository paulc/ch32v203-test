use ch32_hal::usart;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;

pub static TX_PIPE: Pipe<CriticalSectionRawMutex, 256> = Pipe::new();

pub struct TxSink;

pub fn push(mut data: &[u8]) -> Result<(), ()> {
    while !data.is_empty() {
        match TX_PIPE.try_write(data) {
            Ok(n) => data = &data[n..],
            Err(_) => return Err(()), // pipe full
        }
    }
    Ok(())
}

impl ufmt::uWrite for TxSink {
    type Error = ();
    fn write_str(&mut self, s: &str) -> Result<(), ()> {
        let mut rest = s.as_bytes();
        // Translate '\n' to '\r\n'
        while let Some(i) = rest.iter().position(|&b| b == b'\n') {
            let (head, tail) = rest.split_at(i);
            push(head)?;
            if head.last() != Some(&b'\r') {
                push(b"\r")?;
            }
            push(b"\n")?;
            rest = &tail[1..];
        }
        push(rest)
    }
}

#[macro_export]
macro_rules! serial_println {
    ($($arg:tt)*) => {{ let _ = ::ufmt::uwriteln!(&mut $crate::serial::TxSink, $($arg)*); }};
}

async fn handle_cmd(line: &str) {
    let line = line.trim_ascii();
    if !line.is_empty() {
        let mut it = line.split_ascii_whitespace();
        match it.next() {
            Some("hello") => serial_println!("-- Hello >>{}<<", it.next().unwrap_or("There")),
            Some(cmd) => serial_println!("-- CMD: {}", cmd),
            None => {}
        }
    }
}

#[embassy_executor::task]
pub async fn serial_read(
    mut rx: usart::UartRx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
) {
    let mut buf = [0u8; 16];
    let mut line_buf = heapless::String::<64>::new();
    loop {
        match rx.read_until_idle(&mut buf).await {
            Ok(n) => {
                if n > 0 && buf[..n].is_ascii() {
                    let s = unsafe { str::from_utf8_unchecked(&buf[..n]) };
                    // Check if buffer full
                    if line_buf.push_str(s).is_err() {
                        handle_cmd(&line_buf).await;
                        line_buf.clear();
                    }
                    // Check for line
                    while let Some(i) = line_buf.find(['\n', '\r']) {
                        handle_cmd(&line_buf[..i].trim_ascii()).await;
                        line_buf.drain(..=i);
                    }
                }
            }
            Err(e) => match e {
                usart::Error::Overrun => {
                    serial_println!("-- ERR overrun");
                }
                usart::Error::Framing => {
                    serial_println!("-- ERR framing");
                }
                usart::Error::Noise => {
                    serial_println!("-- ERR noise");
                }
                _ => {
                    serial_println!("-- ERR other");
                }
            },
        }
    }
}

#[embassy_executor::task]
pub async fn serial_write(
    mut tx: usart::UartTx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
) {
    let mut b = [0u8; 64];
    loop {
        let n = TX_PIPE.read(&mut b).await;
        let _ = tx.write(&b[..n]).await;
        let _ = tx.flush().await;
    }
}
