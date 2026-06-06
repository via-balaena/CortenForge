# D4 physical-pbit firmware

Firmware for the Teensy 4.1 that drives the magnetoelastic-cantilever rig and
streams sensor data to the host. Split in two:

| Crate | Target | Status | What it is |
|-------|--------|--------|------------|
| [`pbit-fw-core`](pbit-fw-core) | host + `thumbv7em` | **done, tested** | Pure `no_std` logic: drive-signal generation (the temperature knob) + the CSV log contract. Unit-tested on the host; a workspace member. |
| `pbit-fw` | `thumbv7em-none-eabihf` | **bring-up pending hardware** | The embedded binary: peripheral glue (ADC, PWM, USB-serial) that calls `pbit-fw-core`. A separate Cargo project, **excluded** from the host workspace. |

Why the split: everything with no hardware dependency (noise math, the CSV
format the S3 analysis tool parses) lives in `pbit-fw-core` so it can be tested
without a board. Only the peripheral wiring needs the real Teensy, and that's
the part we finish during bring-up.

## The embedded binary (`pbit-fw`) — design, written at bring-up

Built on the `teensy4-bsp` / `imxrt-hal` Rust BSP. One periodic timer ISR at the
sample rate (~10–20 kHz) does, each tick:

1. **Drive out** — `DriveGen::next_sample()` → `DriveGen::to_pwm_duty()` →
   FlexPWM duty. A one-resistor-one-cap low-pass on the PWM pin reconstructs the
   analog drive feeding the TPA3116 amp → shaker. (Teensy 4.x has no true DAC;
   PWM is sufficient because the accelerometer measures the *actual* injected
   vibration — we calibrate the bath from reality, not the command.)
2. **Sense in** — read N Hall channels through the CD74HC4067 analog mux + the
   ADC, plus the 3 accelerometer axes.
3. **Frame + stream** — pack a `LogRecord` and `write_csv()` it to the USB-serial
   port. The host captures the stream straight to a `.csv` the S3 tool ingests.

The drive PWM and the ADC sampling share the one MCU clock, so the sine probe's
phase (`DriveGen::sine_phase()`) is an exact reference for the synchrony metric
in the stochastic-resonance step.

### Provisional pin map (finalize against the board)

- FlexPWM drive out → RC filter → amp input.
- ADC0 ← mux common; mux select lines on 4 GPIO; one Hall per mux channel.
- ADC1 ← accelerometer X/Y/Z (ADXL335 analog).
- USB-serial: native (Teensy USB) for the data stream.

### Build / flash (once hardware is here)

```text
cd firmware/pbit-fw
cargo build --release            # target set in .cargo/config.toml
# flash via teensy_loader_cli or the Teensy bootloader button
```

## Data contract

`LogRecord` (in `pbit-fw-core`) is the single source of truth for the CSV format:

```text
t_us,drive,h0,h1,...,h{n_hall-1},ax,ay,az
```

The S3 analysis/calibration tool parses exactly this. If the format changes, it
changes in `pbit-fw-core::record` and both sides stay in sync.
