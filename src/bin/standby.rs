#![no_std]
#![no_main]

use ch32_hal::bind_interrupts;
use ch32_hal::gpio::{Input, Level, Output, Pull};
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_time::Timer;

use v203::power;

use ufmt::uwrite;

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

const RCC_CFG: ch32_hal::rcc::Config = ch32_hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSI;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let reset = power::reset_cause(); // first thing, before init/await
    let woke_from_standby = reset.cause == power::ResetCause::StandbyWake;

    let config = ch32_hal::Config {
        rcc: RCC_CFG,
        ..Default::default()
    };
    let p = ch32_hal::init(config);

    let mut uart_tx = p.PA9;
    let uart_rx = p.PA10;
    let led_pin = p.PA15;

    let mut buf = heapless::String::<256>::new();

    power::gpio_all_pulldown(false);
    let _ = Input::new(uart_tx.reborrow(), Pull::Up);

    Timer::after_millis(500).await;

    let mut led = Output::new(led_pin, Level::Low, Default::default());

    let uart_config = usart::Config::default(); // 115200,N,8,1
    let (mut uart_tx, _uart_rx) = match usart::Uart::new(
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

    if !woke_from_standby {
        for _ in 0..10 {
            led.toggle();
            Timer::after_millis(50).await;
        }
    }

    let rtc_init = power::rtc_init().is_ok();

    if !woke_from_standby {
        // Clear BKP regs
        power::bkp_write_buf([0_u8; 20]);
    }

    let counter = power::bkp_read(1);
    let _ = uwrite!(
        buf,
        "=== WAKE: cause={} rst={} csr={} rtc_init={} counter={}\r\n",
        reset.cause.as_str(),
        reset.rst_flags,
        reset.pwr_csr,
        rtc_init,
        counter
    );
    let _ = uart_tx.write(buf.as_bytes()).await;
    let _ = uart_tx.flush().await;
    buf.clear();

    let mut bkp = power::bkp_read_buf();
    let _ = uwrite!(buf, "=== BKP: {:?}\r\n", bkp);
    let _ = uart_tx.write(buf.as_bytes()).await;
    let _ = uart_tx.flush().await;
    buf.clear();

    let c2 = counter.wrapping_add(1).wrapping_pow(2) as u32;
    bkp[16..20].copy_from_slice(&c2.to_le_bytes());
    power::bkp_write_buf(bkp);

    power::bkp_write(1, counter.wrapping_add(1));

    loop {
        led.set_high();
        power::stop_for_ms(500, RCC_CFG).unwrap();
        led.set_low();

        // Standby
        power::standby_for_ms(2000, false, false);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
