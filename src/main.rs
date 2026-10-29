//! RP2040 dual-core demo for the DFRobot Beetle RP2040, built on Embassy.
//!
//! Both Cortex-M0+ cores run their own Embassy executor:
//!   * core 0: USB CDC serial logger + the "coordinator" task
//!   * core 1: a "worker" task waiting for jobs from core 0
//!
//! Every round we count the primes below `LIMIT` twice:
//!   1. on core 0 alone,
//!   2. split between core 0 and core 1 running at the same time.
//! The timings printed to the serial console show the ~2x speed-up and the
//! interleaved log lines show both cores working simultaneously.
//!
//! Open the console with e.g. `screen /dev/tty.usbmodem* 115200`.

#![no_std]
#![no_main]

use embassy_executor::{Executor, Spawner};
use embassy_futures::yield_now;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use log::info;
use panic_halt as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

/// Count primes in [0, LIMIT).
const LIMIT: u32 = 100_000;
/// Numbers are processed in blocks; block `b` belongs to core `b % 2`
/// (or to core 0 alone in the single-core run). Interleaving keeps the
/// work balanced because bigger numbers are more expensive to test.
const BLOCK: u32 = 2_000;
const BLOCKS: u32 = LIMIT.div_ceil(BLOCK);

static mut CORE1_STACK: Stack<8192> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

/// core 0 -> core 1: "start counting your half" (payload = round number)
static JOBS: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();
/// core 1 -> core 0: number of primes core 1 found
static RESULTS: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();

/// Which core is executing this code: the SIO CPUID register (0xD000_0000)
/// reads 0 on core 0 and 1 on core 1.
fn core_id() -> u32 {
    unsafe { core::ptr::read_volatile(0xD000_0000 as *const u32) }
}

fn is_prime(n: u32) -> bool {
    if n < 2 {
        return false;
    }
    if n % 2 == 0 {
        return n == 2;
    }
    let mut d = 3;
    while d * d <= n {
        if n % d == 0 {
            return false;
        }
        d += 2;
    }
    true
}

fn count_block(block: u32) -> u32 {
    let start = block * BLOCK;
    let end = (start + BLOCK).min(LIMIT);
    (start..end).filter(|&n| is_prime(n)).count() as u32
}

/// Count primes in the blocks `first, first + step, ...`.
/// Yields between blocks so other tasks on the same core (USB!) keep running.
async fn count_blocks(first: u32, step: u32, tag: &str) -> u32 {
    let mut total = 0;
    let mut block = first;
    while block < BLOCKS {
        total += count_block(block);
        if block % 10 == first % 10 {
            info!("  [{} on core {}] block {:2}/{} done", tag, core_id(), block + 1, BLOCKS);
        }
        block += step;
        yield_now().await;
    }
    total
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    // Start core 1 with its own executor.
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| spawner.spawn(core1_worker().unwrap()))
        },
    );

    // Core 0 executor: USB logger + coordinator.
    let driver = Driver::new(p.USB, Irqs);
    let led = Output::new(p.PIN_13, Level::Low); // Beetle RP2040 on-board LED = GP13
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(usb_logger(driver).unwrap());
        spawner.spawn(core0_coordinator(spawner, led).unwrap());
    })
}

#[embassy_executor::task]
async fn usb_logger(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(2048, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn core1_worker() {
    // Logged before USB is up, so this line is usually lost; the
    // "core 1 got job" lines below prove core 1 is alive.
    info!("[core {}] worker ready", core_id());
    loop {
        let round = JOBS.receive().await;
        info!("[core {}] got job for round {}", core_id(), round);
        let primes = count_blocks(1, 2, "dual").await;
        RESULTS.send(primes).await;
    }
}

#[embassy_executor::task]
async fn core0_coordinator(_spawner: Spawner, mut led: Output<'static>) {
    // Give the host time to enumerate the USB device and you time to open a terminal.
    for i in (1..=5).rev() {
        info!("starting in {}...", i);
        Timer::after(Duration::from_secs(1)).await;
    }

    let mut round = 1;
    loop {
        info!("");
        info!("===== round {} : primes below {} =====", round, LIMIT);

        // --- 1) single core -------------------------------------------------
        info!("single-core run (core {} does everything)", core_id());
        let t0 = Instant::now();
        let single = count_blocks(0, 1, "single").await;
        let single_ms = t0.elapsed().as_millis();
        info!("single-core: {} primes in {} ms", single, single_ms);

        // --- 2) both cores in parallel --------------------------------------
        info!("dual-core run (even blocks -> core 0, odd blocks -> core 1)");
        led.set_high();
        let t0 = Instant::now();
        JOBS.send(round).await; // wake core 1
        let mine = count_blocks(0, 2, "dual").await;
        let theirs = RESULTS.receive().await; // wait for core 1
        let dual_ms = t0.elapsed().as_millis();
        led.set_low();

        info!(
            "dual-core:   {} primes in {} ms  (core0: {}, core1: {})",
            mine + theirs,
            dual_ms,
            mine,
            theirs
        );
        if mine + theirs == single {
            info!("results match");
        } else {
            info!("MISMATCH! {} vs {}", single, mine + theirs);
        }
        let speedup_x100 = single_ms * 100 / dual_ms.max(1);
        info!("speed-up: {}.{:02}x", speedup_x100 / 100, speedup_x100 % 100);

        round += 1;
        Timer::after(Duration::from_secs(3)).await;
    }
}
