#![no_std]
#![no_main]

use ch32_hal::bind_interrupts;
use ch32_hal::gpio::{Level, Output};
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};

use static_cell::StaticCell;

use v203::console_println;
use v203::serial::{serial1_read, serial1_write, RxChannelType, RxReceiverType};

use rand::rngs::SmallRng;
use rand::{RngExt, SeedableRng};

static RX_CHANNEL: StaticCell<RxChannelType> = StaticCell::new();

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE;
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

    // RNG
    let mut adc = ch32_hal::adc::Adc::new(p.ADC1, Default::default());
    let seed = rng_seed(&mut adc, &mut ch32_hal::adc::Temperature).unwrap();
    let mut rng = SmallRng::seed_from_u64(seed);

    // LED
    let mut led = Output::new(p.PA15, Level::Low, Default::default());

    let mut _count = 0_u32;
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        let rand: u32 = rng.random();

        console_println!("{:08x}", rand);
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn serial_handler(chan: RxReceiverType) {
    loop {
        let line = chan.receive().await;
        if !line.is_empty() {
            console_println!(">> {}", line.trim_ascii());
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

use ch32_hal::adc::{Adc, AdcChannel, Pga, SampleTime};
use ch32_hal::peripherals::ADC1;

pub fn rng_seed(adc: &mut Adc<'static, ADC1>, chan: &mut impl AdcChannel<ADC1>) -> Option<u64> {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325; // FNV-1a 64-bit offset basis
    let mut prev = u16::MAX;
    let mut changes = 0u32;
    // 1024 samples requires 0.06 bits of entropy per sample for 64-bits
    for _ in 0..1024 {
        let s = adc.convert(chan, SampleTime::CYCLES7_5, Pga::X1);
        changes += (s != prev) as u32;
        prev = s;
        h = (h ^ s as u64).wrapping_mul(0x0000_0100_0000_01b3); // FNV-1a 64-bit prime
    }
    // Check we have at-least 128 changes
    (changes >= 128).then_some(h)
}
