# RP2040 dual-core demo (Embassy, DFRobot Beetle RP2040)

Both RP2040 cores run their own Embassy executor:

| core | tasks |
|------|-------|
| 0 | USB CDC serial logger, coordinator |
| 1 | worker, which waits for jobs from core 0 over an `embassy_sync::Channel` |

Each round, the firmware counts the primes below 100 000 in two ways:

1. on core 0 alone,
2. split between core 0 and core 1 at the same time (even blocks go to core 0, odd blocks to core 1).

It prints both timings and the speed-up. The on-board LED (GP13) is lit during the dual-core part.

## Build

```bash
cargo build --release
```

Needs the `thumbv6m-none-eabi` target (`rustup target add thumbv6m-none-eabi`) and `elf2uf2-rs` (`cargo install elf2uf2-rs`).

## Flash

1. Hold **BOOT**, press and release **RST**, then release **BOOT**. The board shows up as the `RPI-RP2` drive.
2. Run:

```bash
cargo run --release
```

This builds the firmware, converts it to UF2 and copies it to the drive. The board reboots into the firmware.
You can also drag `rp2040_multicore.uf2` onto the `RPI-RP2` drive yourself.

## Console

The board shows up as a USB serial port. The firmware counts down for 5 s before it starts, so you have time to connect:

```bash
screen /dev/tty.usbmodem* 115200
```

Quit `screen` with `Ctrl-A`, then `K`, then `Y`. Other serial tools such as `tio` or `minicom` work too. The baud rate is ignored over USB CDC.

Expected output (the lines from both cores are interleaved because they run at the same time):

```
===== round 1 : primes below 100000 =====
single-core run (core 0 does everything)
  [single on core 0] block  1/50 done
  ...
single-core: 9592 primes in ... ms
dual-core run (even blocks -> core 0, odd blocks -> core 1)
[core 1] got job for round 1
  [dual on core 0] block  1/50 done
  [dual on core 1] block  2/50 done
  ...
dual-core:   9592 primes in ... ms  (core0: ..., core1: ...)
results match
speed-up: ...x        <- should be close to 2
```
