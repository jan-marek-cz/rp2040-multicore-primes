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
//!
//! ---------------------------------------------------------------------------
//! BIG PICTURE (read this first)
//! ---------------------------------------------------------------------------
//! The RP2040 chip has TWO identical CPU cores. After reset only core 0 runs;
//! core 1 sits asleep until core 0 explicitly starts it (`spawn_core1`).
//!
//! Embassy is an *async* framework for microcontrollers. Instead of threads it
//! uses "tasks" (async functions) that are run by an "executor". An executor is
//! a tiny scheduler: it runs a task until the task hits an `.await` that can't
//! complete yet, then switches to another task that is ready. When nothing is
//! ready, the core goes to sleep until an interrupt wakes it.
//!
//! Important: one executor = one core. Tasks on the same executor never run at
//! the same time; they take turns (cooperative multitasking). To get REAL
//! parallelism we give each core its own executor.
//!
//! Data flow in this program:
//!
//!     core 0 (coordinator)                     core 1 (worker)
//!     --------------------                     ---------------
//!     JOBS.send(round)  ───────────────────▶   JOBS.receive()
//!     count even blocks                        count odd blocks
//!     RESULTS.receive() ◀───────────────────   RESULTS.send(primes)
//!
//! Meanwhile a second task on core 0 (`usb_logger`) ships every `info!` line
//! over USB so you can read it on your computer.
//! ---------------------------------------------------------------------------

// `no_std`: we don't have an operating system, so the Rust standard library
// (files, threads, heap by default, println!...) is unavailable. We only get
// `core` (basic types, iterators, Option/Result, etc.).
#![no_std]
// `no_main`: there is no normal `fn main()` called by an OS. Instead the
// `#[cortex_m_rt::entry]` attribute below marks the function the chip jumps to
// after reset.
#![no_main]

// --- Imports ----------------------------------------------------------------
use embassy_executor::{Executor, Spawner}; // the async scheduler + handle to start tasks
use embassy_futures::yield_now; // "pause me briefly so other tasks can run"
use embassy_rp::bind_interrupts; // macro to connect hardware interrupts to Embassy drivers
use embassy_rp::gpio::{Level, Output}; // digital pins (we use one for the LED)
use embassy_rp::multicore::{Stack, spawn_core1}; // starting the second core
use embassy_rp::peripherals::USB; // the USB hardware block
use embassy_rp::usb::{Driver, InterruptHandler}; // low-level USB driver
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex; // a lock that is safe across BOTH cores
use embassy_sync::channel::Channel; // a queue for sending values between tasks/cores
use embassy_time::{Duration, Instant, Timer}; // timekeeping: delays and stopwatches
use log::info; // standard `log` crate macro; output goes to whatever logger is installed (USB here)
use panic_halt as _; // on panic: just stop (loop forever). `as _` = "link it in, I won't name it"
use static_cell::StaticCell; // lets us create `'static` values at runtime, safely, exactly once

// The USB peripheral raises the USBCTRL_IRQ interrupt whenever something
// happens on the bus. This macro generates a struct `Irqs` that tells Embassy:
// "when USBCTRL_IRQ fires, call the USB driver's InterruptHandler".
// We pass `Irqs` to `Driver::new` below as proof that this wiring exists.
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// --- Workload configuration -------------------------------------------------

/// Count primes in [0, LIMIT).
const LIMIT: u32 = 100_000;
/// Numbers are processed in blocks; block `b` belongs to core `b % 2`
/// (or to core 0 alone in the single-core run). Interleaving keeps the
/// work balanced because bigger numbers are more expensive to test.
///
/// (If we instead gave core 0 the first half and core 1 the second half,
/// core 1 would get all the big, slow-to-test numbers and finish last.)
const BLOCK: u32 = 2_000;
/// Number of blocks, rounded UP so a partial last block isn't forgotten.
/// 100_000 / 2_000 = 50 blocks exactly here.
const BLOCKS: u32 = LIMIT.div_ceil(BLOCK);

// --- Static memory ----------------------------------------------------------
// On a microcontroller we avoid dynamic allocation, so long-lived things are
// stored in `static` variables (fixed places in RAM for the whole program).

/// Memory for core 1's call stack: 8 KiB. Core 0 uses the normal stack set up
/// by the linker; core 1 needs its own, which we provide here.
/// It's `static mut` because `spawn_core1` needs a `&mut` to it. Accessing a
/// `static mut` is `unsafe` in Rust, since the compiler can't prove nobody
/// else touches it — we promise we hand it out only once.
static mut CORE1_STACK: Stack<8192> = Stack::new();

/// Storage for the two executors. `Executor::run` requires a `&'static mut`
/// (the executor must live forever, because `run` never returns).
/// `StaticCell::init` places the value into the static and gives back that
/// `&'static mut`; it panics if you try to init the same cell twice.
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// Channels: thread-safe (well, core-safe) mailboxes.
//   Channel<MutexType, MessageType, Capacity>
// - CriticalSectionRawMutex: on RP2040 this uses a hardware spinlock, so it is
//   safe even when the two cores touch the channel at the same moment.
// - u32: the type of message carried.
// - 1: the channel holds at most one message; a second `send` waits until the
//   first message has been received.
// They're `static` so both cores can reach them without passing references.

/// core 0 -> core 1: "start counting your half" (payload = round number)
static JOBS: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();
/// core 1 -> core 0: number of primes core 1 found
static RESULTS: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();

/// Which core is executing this code: the SIO CPUID register (0xD000_0000)
/// reads 0 on core 0 and 1 on core 1.
///
/// This is raw hardware access: we read a 32-bit value directly from a memory
/// address that the chip maps to a hardware register. `read_volatile` tells
/// the compiler "really read it every time, don't optimize this away or cache
/// it". `unsafe` because dereferencing an arbitrary address can't be checked
/// by the compiler — we know from the RP2040 datasheet that it's valid.
fn core_id() -> u32 {
    unsafe { core::ptr::read_volatile(0xD000_0000 as *const u32) }
}

// --- The actual math (plain, synchronous code) ------------------------------

/// Simple trial-division primality test.
/// Deliberately not clever: we WANT it to be CPU-heavy so the benchmark shows
/// a measurable difference between one and two cores.
fn is_prime(n: u32) -> bool {
    if n < 2 {
        return false; // 0 and 1 are not prime
    }
    if n % 2 == 0 {
        return n == 2; // 2 is the only even prime
    }
    // Try odd divisors 3, 5, 7, ... up to sqrt(n).
    // (`d * d <= n` is the same as `d <= sqrt(n)` but avoids floating point,
    // which the Cortex-M0+ has no hardware for.)
    let mut d = 3;
    while d * d <= n {
        if n % d == 0 {
            return false; // found a divisor -> not prime
        }
        d += 2;
    }
    true
}

/// Count primes inside one block, i.e. numbers
/// [block * BLOCK, block * BLOCK + BLOCK), clipped to LIMIT.
fn count_block(block: u32) -> u32 {
    let start = block * BLOCK;
    let end = (start + BLOCK).min(LIMIT); // don't run past LIMIT on the last block
    // Iterator chain: take every n in the range, keep the primes, count them.
    (start..end).filter(|&n| is_prime(n)).count() as u32
}

/// Count primes in the blocks `first, first + step, ...`.
/// Yields between blocks so other tasks on the same core (USB!) keep running.
///
/// How it's called:
///   * single-core run: first = 0, step = 1  -> blocks 0,1,2,3,...,49
///   * core 0 in dual:  first = 0, step = 2  -> blocks 0,2,4,...,48
///   * core 1 in dual:  first = 1, step = 2  -> blocks 1,3,5,...,49
///
/// `tag` is just a label ("single"/"dual") for the log output.
///
/// It's `async` only because of the `yield_now().await` inside it. The prime
/// counting itself is ordinary blocking code; while a block is being counted,
/// nothing else on this core runs. That's why blocks are small-ish and we
/// yield after each one.
async fn count_blocks(first: u32, step: u32, tag: &str) -> u32 {
    let mut total = 0;
    let mut block = first;
    while block < BLOCKS {
        total += count_block(block);
        // Don't spam the log: only print roughly every 10th block.
        // Comparing with `first % 10` makes this work for both the even
        // (0,10,20,...) and the odd (1,11,21,...) sequences.
        if block % 10 == first % 10 {
            // `{:2}` = pad to width 2, so the columns line up.
            info!("  [{} on core {}] block {:2}/{} done", tag, core_id(), block + 1, BLOCKS);
        }
        block += step;
        // Cooperative multitasking: let the executor run any other ready task
        // (on core 0 that's the USB logger, which must service the USB bus
        // regularly or the host may think the device died). Then we continue.
        yield_now().await;
    }
    total
}

// --- Entry point ------------------------------------------------------------

/// The function the chip jumps to after reset. Return type `!` ("never")
/// means it never returns — there's nothing to return to.
#[cortex_m_rt::entry]
fn main() -> ! {
    // Initialize clocks, the timer, etc. and get ownership of all peripherals.
    // `p` is a struct with one field per hardware block (p.USB, p.PIN_13, ...).
    // Each can be taken only once, so the type system prevents two pieces of
    // code from accidentally driving the same pin.
    let p = embassy_rp::init(Default::default());

    // Start core 1 with its own executor.
    // Arguments: ownership token for core 1, the stack memory it will use,
    // and a closure that core 1 runs as its "main" function.
    spawn_core1(
        p.CORE1,
        // Turn the `static mut` into a `&mut`. `addr_of_mut!` gets a raw
        // pointer without creating an intermediate reference (the recommended
        // way to touch `static mut`), then `&mut *` converts it.
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        // `move ||` = closure that takes ownership of anything it captures.
        // Everything inside here executes ON CORE 1.
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            // `run` starts the scheduler loop and never returns.
            // The closure gets a `Spawner` used to start the initial tasks.
            // `core1_worker()` doesn't run the task yet; it produces a
            // "spawn token" (wrapped in a Result: it fails if the task is
            // already running). `spawner.spawn(...)` hands it to the executor.
            executor1.run(|spawner| spawner.spawn(core1_worker().unwrap()))
        },
    );

    // From here on we're back on core 0.

    // Core 0 executor: USB logger + coordinator.
    let driver = Driver::new(p.USB, Irqs); // USB driver (Irqs = the interrupt wiring from above)
    let led = Output::new(p.PIN_13, Level::Low); // Beetle RP2040 on-board LED = GP13, start OFF
    let executor0 = EXECUTOR0.init(Executor::new());
    // Start the scheduler on core 0 with two tasks. Never returns.
    executor0.run(|spawner| {
        spawner.spawn(usb_logger(driver).unwrap());
        spawner.spawn(core0_coordinator(spawner, led).unwrap());
    })
}

// --- Tasks ------------------------------------------------------------------
// `#[embassy_executor::task]` turns an async fn into a task the executor can
// run. Each task gets statically reserved memory, which is why its arguments
// must be `'static` (live forever).

/// Runs on core 0. Makes the board appear as a USB serial port (CDC-ACM) on
/// your computer and installs itself as the global `log` logger, so every
/// `info!(...)` anywhere — on EITHER core — ends up in your terminal.
/// 2048 = size of the internal text buffer in bytes.
/// LevelFilter::Info = show `info!`, `warn!`, `error!`; hide `debug!`/`trace!`.
/// This task runs forever.
#[embassy_executor::task]
async fn usb_logger(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(2048, log::LevelFilter::Info, driver);
}

/// Runs on core 1. Sits idle until core 0 sends a job, counts the odd blocks,
/// sends back the result, and waits for the next job. Forever.
#[embassy_executor::task]
async fn core1_worker() {
    // Logged before USB is up, so this line is usually lost; the
    // "core 1 got job" lines below prove core 1 is alive.
    info!("[core {}] worker ready", core_id());
    loop {
        // Wait (sleeping, not burning CPU) until core 0 puts a job in JOBS.
        let round = JOBS.receive().await;
        info!("[core {}] got job for round {}", core_id(), round);
        // Our half: blocks 1, 3, 5, ... (first = 1, step = 2).
        let primes = count_blocks(1, 2, "dual").await;
        // Hand the count back to core 0.
        RESULTS.send(primes).await;
    }
}

/// Runs on core 0. Drives the whole benchmark: single-core run, then
/// dual-core run, compare, print, repeat.
/// (`_spawner` is unused; the leading underscore silences the warning.)
#[embassy_executor::task]
async fn core0_coordinator(_spawner: Spawner, mut led: Output<'static>) {
    // Give the host time to enumerate the USB device and you time to open a terminal.
    // `(1..=5).rev()` counts 5, 4, 3, 2, 1.
    // `Timer::after(...).await` sleeps WITHOUT blocking: the USB task keeps
    // running in the meantime.
    for i in (1..=5).rev() {
        info!("starting in {}...", i);
        Timer::after(Duration::from_secs(1)).await;
    }

    let mut round = 1;
    loop {
        info!("");
        info!("===== round {} : primes below {} =====", round, LIMIT);

        // --- 1) single core -------------------------------------------------
        // Core 0 does all 50 blocks itself. Core 1 is idle (waiting on JOBS).
        info!("single-core run (core {} does everything)", core_id());
        let t0 = Instant::now(); // start stopwatch
        let single = count_blocks(0, 1, "single").await;
        let single_ms = t0.elapsed().as_millis(); // stop stopwatch
        info!("single-core: {} primes in {} ms", single, single_ms);

        // --- 2) both cores in parallel --------------------------------------
        info!("dual-core run (even blocks -> core 0, odd blocks -> core 1)");
        led.set_high(); // LED on while both cores are busy (visual cue)
        let t0 = Instant::now();
        JOBS.send(round).await; // wake core 1 — it starts on the odd blocks NOW
        // ...while core 0 simultaneously works on the even blocks:
        let mine = count_blocks(0, 2, "dual").await;
        // Core 0 is done with its half; wait until core 1 reports its half.
        // (Whichever core finishes last determines the total time.)
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
        // Sanity check: splitting the work must not change the answer.
        // (Expected value for LIMIT = 100_000 is 9592.)
        if mine + theirs == single {
            info!("results match");
        } else {
            info!("MISMATCH! {} vs {}", single, mine + theirs);
        }
        // Speed-up = single_ms / dual_ms, printed with 2 decimals.
        // We avoid floats (no FPU on Cortex-M0+, and float formatting is
        // big in code size), so we compute it ×100 as an integer:
        //   e.g. 1980 ms / 1000 ms -> 198 -> printed as "1.98x".
        // `.max(1)` prevents division by zero if dual_ms were somehow 0.
        let speedup_x100 = single_ms * 100 / dual_ms.max(1);
        info!("speed-up: {}.{:02}x", speedup_x100 / 100, speedup_x100 % 100);

        round += 1;
        Timer::after(Duration::from_secs(3)).await; // pause, then do it all again
    }
}