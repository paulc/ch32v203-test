#![no_std]
#![no_main]
#![allow(unused)]

use ch32_hal::gpio::{Level, Output};
use qingke::riscv;

// Volatile MMIO helpers used by every example
#[inline(always)]
fn rd(a: u32) -> u32 {
    unsafe { core::ptr::read_volatile(a as *const u32) }
}
#[inline(always)]
fn wr(a: u32, v: u32) {
    unsafe { core::ptr::write_volatile(a as *mut u32, v) }
}
#[inline(always)]
fn set(a: u32, m: u32) {
    wr(a, rd(a) | m)
}
#[inline(always)]
fn clr(a: u32, m: u32) {
    wr(a, rd(a) & !m)
}
#[inline(always)]
fn rmw(a: u32, mask: u32, v: u32) {
    wr(a, (rd(a) & !mask) | v)
}
// Registers named R16_* in the RM are 16 bits wide
#[inline(always)]
fn rd16(a: u32) -> u16 {
    unsafe { core::ptr::read_volatile(a as *const u16) }
}
#[inline(always)]
fn wr16(a: u32, v: u16) {
    unsafe { core::ptr::write_volatile(a as *mut u16, v) }
}

// RCC clock-enable registers (RM 3.4)
const RCC_AHBPCENR: u32 = 0x4002_1014; // DMA1EN b0, CRCEN b6, OTG_FSEN b12
const RCC_APB2PCENR: u32 = 0x4002_1018; // AFIOEN b0, IOPAEN..IOPDEN b2..b5, ADC1EN b9, TIM1EN b11, SPI1EN b12, USART1EN b14
const RCC_APB1PCENR: u32 = 0x4002_101C; // TIM2..4EN b0..b2, WWDGEN b11, SPI2EN b14, USART2..4EN b17..b19,
                                        // I2C1EN b21, I2C2EN b22, USBDEN b23, CAN1EN b25, BKPEN b27, PWREN b28
                                        // GPIO ports (RM 10.3)
const GPIOA: u32 = 0x4001_0800;
const GPIOB: u32 = 0x4001_0C00;
const BSHR: u32 = 0x10; // set (b0..15) / reset (b16..31)

/// Write the 4-bit CNF[1:0]:MODE[1:0] field of one pin (see the GPIO sheet)
fn pin_cfg(port: u32, pin: u32, nibble: u32) {
    let reg = port + if pin < 8 { 0x00 } else { 0x04 }; // CFGLR / CFGHR
    let sh = (pin % 8) * 4;
    rmw(reg, 0xF << sh, nibble << sh);
}

const STK: u32 = 0xE000_F000;
const STK_CTLR: u32 = STK + 0x00;
const STK_CNTL: u32 = STK + 0x08;
const STK_CNTH: u32 = STK + 0x0C;

fn now_ticks() -> u64 {
    // tear-free 64-bit read
    loop {
        let hi = rd(STK + 0x0C);
        let lo = rd(STK + 0x08);
        if rd(STK + 0x0C) == hi {
            return ((hi as u64) << 32) | lo as u64;
        }
    }
}

fn delay_ticks(ticks: u64) {
    let start = now_ticks();
    loop {
        if (now_ticks() - start) > ticks {
            break;
        }
    }
}

const RCC_CTLR: u32 = 0x4002_1000;
const RCC_CFGR0: u32 = 0x4002_1004;

#[qingke_rt::entry]
fn main() -> ! {
    // Set clock to HSE 144MHz
    set(RCC_CTLR, 1 << 16); // HSEON
    while rd(RCC_CTLR) & (1 << 17) == 0 {} // HSERDY
    wr(RCC_CFGR0, 0x00BDA502); // 144MHz HSE
    set(RCC_CTLR, 1 << 24); // PLLON
    while rd(RCC_CTLR) & (1 << 25) == 0 {} // PLLRDY
    rmw(RCC_CFGR0, 0b11, 0b10); // SW = PLL
    while (rd(RCC_CFGR0) >> 2) & 0b11 != 0b10 {} // SWS confirms PLL

    // Initialise SYSTICK
    wr(STK_CTLR, 0); // stop while initialising
    wr(STK_CNTL, 0);
    wr(STK_CNTH, 0);
    wr(STK_CTLR, (1 << 2) | (1 << 0));

    set(RCC_APB2PCENR, 1 << 2);
    pin_cfg(GPIOA, 15, 0x3); // PA15: Push-pull @ 50 MHz

    loop {
        wr(GPIOA + 0x10, 0x0000_8000); // BSHR: set PA15 · reset none
        delay_ticks(72000000);
        wr(GPIOA + 0x10, 0x8000_0000); // BSHR: set none · reset PA15
        delay_ticks(72000000);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
