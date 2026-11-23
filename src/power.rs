//! Low-power modes for CH32V203 (tested on CH32V203K8T6), woken by the RTC alarm.
//!
//! ch32-hal has no PWR/RTC/BKP driver for V20x, so this uses the PAC directly
//! plus raw pointers for PWR and BKP (neither is in the V203 PAC).
//!
//! - `stop_for_ms`:    STOP, ~77 uA measured on a minimal board. RAM and
//!                     registers kept; execution continues after the call.
//! - `standby_for_ms`: STANDBY, ~1-2 uA (datasheet 1.1 uA with LSI). Wake-up
//!                     is a reset: execution restarts at `main`. Keep state in
//!                     the backup registers (`bkp_read`/`bkp_write`).
//!
//! Usage:
//!     const RCC_CFG: ch32_hal::rcc::Config = ch32_hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSI;
//!     let p = ch32_hal::init(ch32_hal::Config { rcc: RCC_CFG, ..Default::default() });
//!     power::gpio_all_pulldown(false);       // before creating pin drivers
//!     let reset = power::reset_cause();      // first thing in main
//!     let from_standby = reset.cause == power::ResetCause::StandbyWake;
//!     power::rtc_init().unwrap();
//!     ...
//!     match power::stop_for_ms(5_000, RCC_CFG).unwrap() {
//!         power::Wake::Alarm => { /* timed out */ }
//!         power::Wake::Other => { /* EXTI pin etc. */ }
//!     }
//!     power::standby_for_ms(5_000, false, true); // never returns; PA0 can wake
//!
//! The SDI debug link drops whenever the chip is in STOP/STANDBY: use
//! `try_println!` (plain `println!` blocks forever once wlink loses the link).

use ch32_hal::pac;
use ch32_hal::pac::rcc::vals::Rtcsel;

// ---------------------------------------------------------------- registers

// PWR (not in the V203 PAC). RM §3.4.
const PWR_CTLR: *mut u32 = 0x4000_7000 as *mut u32;
const PWR_CSR: *mut u32 = 0x4000_7004 as *mut u32;
const PWR_CTLR_LPDS: u32 = 1 << 0; // regulator low-power in STOP
const PWR_CTLR_PDDS: u32 = 1 << 1; // 0 = STOP, 1 = STANDBY
const PWR_CTLR_CWUF: u32 = 1 << 2; // clear WUF
const PWR_CTLR_CSBF: u32 = 1 << 3; // clear SBF
const PWR_CTLR_DBP: u32 = 1 << 8; // backup-domain write enable
const PWR_CTLR_RAM_STBY: u32 = 1 << 16; // keep RAM powered in STANDBY (SDK PWR_EnterSTANDBYMode_RAM)
const PWR_CSR_WUF: u32 = 1 << 0; // wake-up event (RTC alarm / WKUP pin) seen
const PWR_CSR_SBF: u32 = 1 << 1; // woke from STANDBY (not reliably set on V203)
const PWR_CSR_EWUP: u32 = 1 << 8; // WKUP pin (PA0, rising edge) wakes STANDBY

// BKP data registers (not in the V203 PAC): 10 x 16 bit, DATAR1 at +0x04.
const BKP_BASE: usize = 0x4000_6C00;
pub const BKP_REGS: usize = 10;

const RTC_ALARM_EXTI_LINE: usize = 17;
const RTC_ALARM_IRQ: u32 = 57; // pac::Interrupt::RTC_ALARM
const RTC_IRQ: u32 = 19; // pac::Interrupt::RTC (set by ALRIE, never enabled)

/// LSI is nominally 40 kHz but only accurate to a few percent.
/// For an accurate period, fit a 32.768 kHz crystal, use Rtcsel::LSE / LSEON
/// with RTC_SRC_HZ = 32_768 and RTC_TICK_HZ = 1024.
const RTC_SRC_HZ: u32 = 40_000;
const RTC_TICK_HZ: u32 = 1_000; // 1 ms tick -> PRL = 39

// ---------------------------------------------------------------- setup

/// Put every GPIO into pull-down input mode (CNF=10, MODE=00, ODR=0).
///
/// After reset all pins are floating inputs, including the pads not bonded
/// out on LQFP32 (PB2/BOOT1, PB9-PB15, PC13-PC15); floating inputs drew
/// ~400 uA extra in STOP. WCH's examples call GPIO_ALL_IPD() for this, and the
/// datasheet low-power figures are measured this way.
///
/// Call once right after `ch32_hal::init()` and before creating pin drivers:
/// each HAL driver (Output::new, Uart::new, ...) reconfigures its own pins.
///
/// `disable_swd = true` also frees PA13/PA14 (AFIO SW_CFG = 0b100) so they get
/// pulled down too. The WCH-Link can then only attach while NRST is held,
/// so leave it false while developing.
pub fn gpio_all_pulldown(disable_swd: bool) {
    use pac::gpio::regs::{Cfghr, Cfglr, Outdr};

    pac::RCC.apb2pcenr().modify(|w| {
        w.set_afioen(true);
        w.set_iopaen(true);
        w.set_iopben(true);
        w.set_iopcen(true);
        w.set_iopden(true);
    });
    for gpio in [pac::GPIOA, pac::GPIOB, pac::GPIOC, pac::GPIOD] {
        gpio.outdr().write_value(Outdr(0)); // ODR = 0 selects pull-down
        gpio.cfglr().write_value(Cfglr(0x8888_8888));
        gpio.cfghr().write_value(Cfghr(0x8888_8888));
    }
    if disable_swd {
        pac::AFIO.pcfr1().modify(|w| w.set_sw_cfg(0b100));
    }
}

/// One-time setup: start LSI, clock the RTC from it, route the alarm to EXTI17.
/// Also enables backup-domain writes (needed by `bkp_write`).
pub fn rtc_init() -> Result<(), &'static str> {
    let rcc = pac::RCC;

    rcc.apb1pcenr().modify(|w| {
        w.set_pwren(true);
        w.set_bkpen(true);
    });
    clear_low_power_config();
    unsafe { PWR_CTLR.write_volatile(PWR_CTLR.read_volatile() | PWR_CTLR_DBP) };

    rcc.rstsckr().modify(|w| w.set_lsion(true));
    wait("lsirdy", || rcc.rstsckr().read().lsirdy())?;

    let bd = rcc.bdctlr().read();
    if !(bd.rtcen() && bd.rtcsel() == Rtcsel::LSI) {
        // RTCSEL is write-once: reset the backup domain if another source is set.
        if bd.rtcsel() != Rtcsel::NO_CLK {
            rcc.bdctlr().modify(|w| w.set_bdrst(true));
            rcc.bdctlr().modify(|w| w.set_bdrst(false));
        }
        // Two separate writes, each confirmed by readback. Setting RTCSEL and
        // RTCEN in one write leaves RTCEN clear on the V203.
        rcc.bdctlr().modify(|w| w.set_rtcsel(Rtcsel::LSI));
        wait("rtcsel", || rcc.bdctlr().read().rtcsel() == Rtcsel::LSI)?;
        rcc.bdctlr().modify(|w| w.set_rtcen(true));
        wait("rtcen", || rcc.bdctlr().read().rtcen())?;
    }
    rtc_sync()?;
    // Always (re)write the prescaler: it is write-only and lives in the
    // backup domain, so a stale value would otherwise survive resets.
    rtc_configure(|| {
        let prl = RTC_SRC_HZ / RTC_TICK_HZ - 1;
        pac::RTC.pscrh().write(|w| w.set_prlh((prl >> 16) as u8));
        pac::RTC.pscrl().write(|w| w.set_prll(prl as u16));
    })?;

    // Alarm -> EXTI line 17, rising edge, interrupt mode.
    pac::RTC.ctlrh().modify(|w| w.set_alrie(true));
    pac::EXTI
        .rtenr()
        .modify(|w| w.set_tr(RTC_ALARM_EXTI_LINE, true));
    pac::EXTI
        .intenr()
        .modify(|w| w.set_mr(RTC_ALARM_EXTI_LINE, true));

    sctlr_fix();
    Ok(())
}

// ---------------------------------------------------------------- STOP

/// What ended a `stop_for_ms`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wake {
    /// The RTC alarm: the full period elapsed.
    Alarm,
    /// Any other STOP exit: an EXTI line whose interrupt is enabled
    /// (e.g. an `ExtiInput` a task is awaiting), PVD, USB wake-up, ...
    Other,
}

/// Enter STOP for up to `ms` milliseconds (2 ..= u32::MAX, 1 ms ticks), then
/// restore clocks with `rcc_cfg` (pass the same config given to
/// `ch32_hal::init`). Returns early with `Wake::Other` if something else ends
/// STOP.
///
/// An EXTI pin only wakes the core if its interrupt is enabled in the PFIC
/// when STOP is entered: with ch32-hal's `ExtiInput` that means the task
/// awaiting the pin must already be parked at its `.await`. Its ISR runs as
/// soon as this function leaves its critical section, so the task is ready
/// when the caller next yields.
///
/// WFI (as WFE) can also fall through without entering STOP (a stale event,
/// or a pending timer interrupt). A real STOP exit always leaves SYSCLK on
/// HSI, so a fall-through is recognised by the clock source being unchanged
/// and the loop simply tries again; the pending ISR runs in between.
///
/// After `Wake::Other` the alarm stays armed and may fire later; that only
/// sets flags (its IRQ stays disabled), and the next call clears them.
///
/// While stopped all HCLK/PCLK clocks halt, so the embassy-time driver
/// freezes: `Instant::now()` is behind wall-clock time by the time spent in STOP.
pub fn stop_for_ms(ms: u32, rcc_cfg: ch32_hal::rcc::Config) -> Result<Wake, &'static str> {
    set_alarm_in_ms(ms)?;
    sctlr_fix();

    // Clock source in use now; after a real STOP it reads back as HSI.
    let sws_before = pac::RCC.cfgr0().read().sws();
    let stopped = || pac::RCC.cfgr0().read().sws() != sws_before;

    let wake = loop {
        let wake = critical_section::with(|_| {
            if alarm_fired() {
                return Some(Wake::Alarm);
            }
            enter_stop();
            if alarm_fired() {
                Some(Wake::Alarm)
            } else if stopped() {
                Some(Wake::Other)
            } else {
                None // fell through without stopping: retry
            }
        });
        if let Some(w) = wake {
            break w;
        }
    };

    // A real STOP leaves SYSCLK on HSI with PLL/HSE off: restore the tree.
    if stopped() {
        unsafe { ch32_hal::rcc::init(rcc_cfg) };
    }

    clear_alarm_flags();
    rtc_sync()?; // RTC shadow registers are stale after STOP (RM §6.3.2)
    Ok(wake)
}

/// One WFI with SLEEPDEEP set, regulator in low-power mode. Must be called
/// with interrupts masked.
fn enter_stop() {
    let pfic = pac::PFIC;
    unsafe {
        let v = PWR_CTLR.read_volatile() & !(PWR_CTLR_PDDS | PWR_CTLR_LPDS);
        PWR_CTLR.write_volatile(v | PWR_CTLR_LPDS);
    }
    // With interrupts masked, a pending-but-disabled IRQ does not wake WFI on
    // the V203, so enable RTC_ALARM in the PFIC for the duration of the WFI
    // only. It is disabled and unpended again before the critical section
    // ends, so its (absent) ISR never runs.
    pfic.ienr2()
        .write_value(pac::pfic::regs::Ienr2(1 << (RTC_ALARM_IRQ - 32)));
    pfic.sctlr().modify(|w| {
        w.set_sleepdeep(true);
        w.set_setevent(false);
    });
    unsafe { core::arch::asm!("wfi") };
    pfic.sctlr().modify(|w| {
        w.set_sleepdeep(false);
        w.set_setevent(false);
    });
    pfic.irer2()
        .write_value(pac::pfic::regs::Irer2(1 << (RTC_ALARM_IRQ - 32)));
    pfic.iprr2()
        .write_value(pac::pfic::regs::Iprr2(1 << (RTC_ALARM_IRQ - 32)));
    unsafe { PWR_CTLR.write_volatile(PWR_CTLR.read_volatile() & !PWR_CTLR_LPDS) };
}

// ---------------------------------------------------------------- STANDBY

/// Enter STANDBY for up to `ms` milliseconds (2 ..= u32::MAX). Never returns:
/// the wake-up is a reset and execution restarts at `main`, where
/// `reset_cause()` reports `StandbyWake` and, after `rtc_init()`,
/// `last_wake_was_alarm()` tells the RTC alarm from the WKUP pin.
///
/// `wake_pin`: a rising edge on PA0 (WKUP) also wakes the chip. While enabled
/// PA0 is forced to input pull-down. If PA0 is already high on entry the
/// chip wakes immediately, so only enter STANDBY with the pin low.
///
/// All RAM contents and peripheral state are lost (GPIOs go high-Z) unless
/// `retain_ram` is set, which keeps the RAM powered (SDK
/// PWR_EnterSTANDBYMode_RAM) at some extra current. Retained data still gets
/// overwritten by the startup code's .data/.bss init unless it lives in a
/// no-init linker section, so prefer the backup registers for small state.
pub fn standby_for_ms(ms: u32, retain_ram: bool, wake_pin: bool) -> ! {
    // Errors here mean the RTC is not running; a reset is the only sane exit.
    if set_alarm_in_ms(ms).is_err() {
        system_reset();
    }
    sctlr_fix();

    unsafe {
        let csr = PWR_CSR.read_volatile() & !PWR_CSR_EWUP;
        PWR_CSR.write_volatile(if wake_pin { csr | PWR_CSR_EWUP } else { csr });

        let mut v = PWR_CTLR.read_volatile() & !(PWR_CTLR_LPDS | PWR_CTLR_RAM_STBY);
        v |= PWR_CTLR_PDDS | PWR_CTLR_CWUF; // STANDBY, and clear a stale WUF
        if retain_ram {
            v |= PWR_CTLR_RAM_STBY;
        }
        PWR_CTLR.write_volatile(v);
    }

    let pfic = pac::PFIC;
    pfic.sctlr().modify(|w| {
        w.set_sleepdeep(true);
        w.set_setevent(false);
    });
    // The first WFI(-as-WFE) usually just consumes a stale event; interrupts
    // stay enabled so any pending ISR runs before the next attempt.
    for _ in 0..1_000 {
        unsafe { core::arch::asm!("wfi") };
        pfic.sctlr().modify(|w| w.set_setevent(false));
    }
    // Could not enter STANDBY (should not happen): reset rather than return.
    system_reset();
}

/// Raw PWR_CSR (WUF bit 0, SBF bit 1, PVDO bit 2, EWUP bit 8), for debugging.
/// Read it before `reset_cause()`, which clears the flags (or use its `pwr_csr`).
pub fn pwr_csr() -> u32 {
    pac::RCC.apb1pcenr().modify(|w| w.set_pwren(true));
    unsafe { PWR_CSR.read_volatile() }
}

/// Why the chip (re)started. From `reset_cause()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetCause {
    /// Power-on / brown-out reset.
    PowerOn,
    /// NRST pin (button or debugger), including NRST pressed during STANDBY.
    Nrst,
    /// Woken from STANDBY by the RTC alarm or the WKUP pin
    /// (`last_wake_was_alarm()` after `rtc_init()` tells which).
    StandbyWake,
    /// Software reset (PFIC RESETSYS, e.g. `standby_for_ms`'s fallback).
    Software,
    /// Independent or window watchdog.
    Watchdog,
    /// Low-power management reset (illegal STANDBY/STOP entry per option bytes).
    LowPower,
    /// No recognised flag set.
    Unknown,
}

impl ResetCause {
    /// Short name, for `ufmt`/`defmt`-free logging.
    pub fn as_str(&self) -> &'static str {
        match self {
            ResetCause::PowerOn => "power-on",
            ResetCause::Nrst => "nrst",
            ResetCause::StandbyWake => "standby-wake",
            ResetCause::Software => "software",
            ResetCause::Watchdog => "watchdog",
            ResetCause::LowPower => "low-power",
            ResetCause::Unknown => "unknown",
        }
    }
}

/// `reset_cause()` result, with the raw flags for logging.
#[derive(Debug, Clone, Copy)]
pub struct ResetInfo {
    pub cause: ResetCause,
    /// RCC_RSTSCKR bits 31:24: LPWR(7) WWDG(6) IWDG(5) SFT(4) POR(3) PIN(2).
    pub rst_flags: u8,
    /// PWR_CSR: WUF(0) SBF(1) PVDO(2) EWUP(8).
    pub pwr_csr: u32,
}

/// Read and clear the reset and wake-up flags, and undo any leftover
/// STOP/STANDBY configuration. Call exactly once, as the first thing in
/// `main` (before `ch32_hal::init` and before any `.await`).
///
/// Observed on a CH32V203K8T6 (rst_flags / pwr_csr):
/// - RTC alarm STANDBY wake: POR (0x08) / WUF
/// - NRST while running:     PIN (0x04) / 0
/// - NRST during STANDBY:    PIN|POR (0x0C) / 0
///
/// SBF is never set on the V203, so a STANDBY wake is recognised by WUF with
/// no PIN reset. PIN is checked first, which also covers the case of NRST
/// pressed after an RTC alarm left WUF set while running.
///
/// The reset flags accumulate until cleared, so they are cleared here (RMVF)
/// to make the next boot's flags meaningful.
pub fn reset_cause() -> ResetInfo {
    let rcc = pac::RCC;
    rcc.apb1pcenr().modify(|w| w.set_pwren(true));

    let r = rcc.rstsckr().read();
    let rst_flags = (r.0 >> 24) as u8;
    rcc.rstsckr().modify(|w| w.set_rmvf(true));

    let pwr_csr = unsafe { PWR_CSR.read_volatile() };
    unsafe {
        // WKUP pin back to a normal GPIO, clear WUF/SBF for the next cycle.
        PWR_CSR.write_volatile(pwr_csr & !PWR_CSR_EWUP);
        PWR_CTLR.write_volatile(PWR_CTLR.read_volatile() | PWR_CTLR_CSBF | PWR_CTLR_CWUF);
    }
    clear_low_power_config();

    let woke = pwr_csr & (PWR_CSR_WUF | PWR_CSR_SBF) != 0;
    let cause = if r.iwdgrstf() || r.wwdgrstf() {
        ResetCause::Watchdog
    } else if r.lpwrrstf() {
        ResetCause::LowPower
    } else if r.pinrstf() {
        ResetCause::Nrst
    } else if r.sftrstf() {
        ResetCause::Software
    } else if woke {
        ResetCause::StandbyWake
    } else if r.porrstf() {
        ResetCause::PowerOn
    } else {
        ResetCause::Unknown
    };

    ResetInfo {
        cause,
        rst_flags,
        pwr_csr,
    }
}

/// True if this boot is a wake-up from STANDBY. Shorthand for
/// `reset_cause().cause == ResetCause::StandbyWake`; it clears the same
/// flags, so call only one of the two, once, first thing in `main`.
pub fn woke_from_standby() -> bool {
    reset_cause().cause == ResetCause::StandbyWake
}

/// Undo any STOP/STANDBY setup left over from before a reset.
///
/// PWR_CTLR is only cleared by a power reset or a STANDBY wake-up, not by an
/// NRST/system reset, so after NRST interrupts STANDBY, PDDS can still be set.
/// If SLEEPDEEP also survives, the executor's first idle WFI (e.g. the first
/// `Timer` await) drops the chip straight back into STANDBY, which looks like
/// "NRST doesn't wake it". Called from `reset_cause()` and `rtc_init()`.
pub fn clear_low_power_config() {
    pac::RCC.apb1pcenr().modify(|w| w.set_pwren(true));
    unsafe {
        let v = PWR_CTLR.read_volatile();
        PWR_CTLR.write_volatile(v & !(PWR_CTLR_PDDS | PWR_CTLR_LPDS | PWR_CTLR_RAM_STBY));
    }
    pac::PFIC.sctlr().modify(|w| {
        w.set_sleepdeep(false);
        w.set_setevent(false);
    });
}

/// After a STANDBY wake-up: true if the RTC alarm caused it, false if it was
/// the WKUP pin. Call after `rtc_init()` (the RTC flags live in the backup
/// domain and survive the reset; `standby_for_ms` clears them on entry).
pub fn last_wake_was_alarm() -> bool {
    pac::RTC.ctlrl().read().alrf()
}

/// Read backup data register `n` (1..=10). Survives STANDBY and NRST while
/// VDD/VBAT stays up.
pub fn bkp_read(n: usize) -> u16 {
    assert!((1..=BKP_REGS).contains(&n));
    pac::RCC.apb1pcenr().modify(|w| w.set_bkpen(true));
    unsafe { ((BKP_BASE + 4 * n) as *const u32).read_volatile() as u16 }
}

/// Write backup data register `n` (1..=10). Needs `rtc_init()` first (DBP).
pub fn bkp_write(n: usize, value: u16) {
    assert!((1..=BKP_REGS).contains(&n));
    unsafe { ((BKP_BASE + 4 * n) as *mut u32).write_volatile(value as u32) };
}

/// Read backup registers as [u8; 20]
pub fn bkp_read_buf() -> [u8; 20] {
    let mut buf = [0_u8; 20];
    for (i, s) in buf.chunks_exact_mut(2).enumerate() {
        s.copy_from_slice(&bkp_read(i + 1).to_le_bytes())
    }
    buf
}

/// Write backup registers from [u8; 20]
pub fn bkp_write_buf(buf: [u8; 20]) {
    let mut r = [0_u8; 2];
    for (i, s) in buf.chunks_exact(2).enumerate() {
        r.copy_from_slice(s);
        bkp_write(i + 1, u16::from_le_bytes(r));
    }
}

fn system_reset() -> ! {
    // PFIC_CFGR: KEYCODE = 0xBEEF, RESETSYS = 1.
    pac::PFIC
        .cfgr()
        .write_value(pac::pfic::regs::Cfgr((0xBEEF << 16) | (1 << 7)));
    loop {}
}

// ---------------------------------------------------------------- RTC helpers

/// Current RTC counter, read consistently across the two 16-bit halves.
pub fn rtc_counter() -> u32 {
    let rtc = pac::RTC;
    loop {
        let h1 = rtc.cnth().read().cnth();
        let l = rtc.cntl().read().cntl();
        let h2 = rtc.cnth().read().cnth();
        if h1 == h2 {
            return (h1 as u32) << 16 | l as u32;
        }
    }
}

fn set_alarm_in_ms(ms: u32) -> Result<(), &'static str> {
    let ticks = (ms as u64 * RTC_TICK_HZ as u64 / 1000).max(2) as u32;
    let rtc = pac::RTC;
    rtc_sync()?;
    let alarm = rtc_counter().wrapping_add(ticks);
    rtc_configure(|| {
        rtc.alrmh().write(|w| w.set_alrh((alarm >> 16) as u16));
        rtc.alrml().write(|w| w.set_alrl(alarm as u16));
    })?;
    // If the alarm fires after this point the flag is simply seen later.
    clear_alarm_flags();
    Ok(())
}

fn alarm_fired() -> bool {
    pac::EXTI.intfr().read().if_(RTC_ALARM_EXTI_LINE)
}

fn clear_alarm_flags() {
    // ALRF is rc_w0; the other flags read back as 1 and writing 1 leaves them alone.
    pac::RTC.ctlrl().modify(|w| w.set_alrf(false));
    pac::EXTI
        .intfr()
        .write(|w| w.set_if_(RTC_ALARM_EXTI_LINE, true));
    pac::PFIC
        .iprr2()
        .write_value(pac::pfic::regs::Iprr2(1 << (RTC_ALARM_IRQ - 32)));
    pac::PFIC
        .iprr1()
        .write_value(pac::pfic::regs::Iprr1(1 << RTC_IRQ));
}

/// Keep WFI-as-WFE semantics (the embassy riscv32 executor needs WFITOWFE +
/// SEVONPEND to wake from its masked WFI) but drop the latched event.
/// SETEVENT (bit 5) reads back as 1, so a plain read-modify-write would latch
/// a fresh event and make the next WFI return immediately: always write it 0.
fn sctlr_fix() {
    pac::PFIC.sctlr().modify(|w| {
        w.set_wfitowfe(true);
        w.set_sevonpend(true);
        w.set_setevent(false);
    });
}

/// Spin until `ok()` is true; give up after ~2M polls so a dead RTCCLK
/// reports an error instead of hanging.
fn wait(what: &'static str, mut ok: impl FnMut() -> bool) -> Result<(), &'static str> {
    for _ in 0..2_000_000u32 {
        if ok() {
            return Ok(());
        }
    }
    Err(what)
}

/// Wait for the APB1 shadow registers to resync with the RTC core.
fn rtc_sync() -> Result<(), &'static str> {
    pac::RTC.ctlrl().modify(|w| w.set_rsf(false));
    wait("rsf", || pac::RTC.ctlrl().read().rsf())
}

/// RM §6.2.3 configuration sequence: wait RTOFF, set CNF, write, clear CNF, wait RTOFF.
fn rtc_configure(f: impl FnOnce()) -> Result<(), &'static str> {
    let rtc = pac::RTC;
    wait("rtoff", || rtc.ctlrl().read().rtoff())?;
    rtc.ctlrl().modify(|w| w.set_cnf(true));
    f();
    rtc.ctlrl().modify(|w| w.set_cnf(false));
    wait("rtoff", || rtc.ctlrl().read().rtoff())
}
