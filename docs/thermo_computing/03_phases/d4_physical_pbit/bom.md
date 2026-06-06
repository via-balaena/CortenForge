# D4 physical-pbit — rig spec + parts order list (BOM)

The hardware for the magnetoelastic-cantilever rig (see [`recon.md`](recon.md) for
the why). Quantities are scaled to carry the arc through **Step 3** (single bit →
resonance → coupled chain → small computing array) in one order, since the drive
system (shaker + amp) is shared across every element and only the elements,
magnets, sensors, and ADC channels scale up.

## Rig architecture (recap)

A thin spring-steel cantilever, clamped at one end, with permanent magnets near
its free tip pulling it toward two stable positions (Moon & Holmes 1979 — the
canonical experimental double-well). The cantilever + tuning magnets + sensor ride
on a **shaker**: broadband noise = the effective temperature, a sine = the
stochastic-resonance probe. A **Hall sensor** reads a tip magnet (continuous
position → `⟨δx²⟩`, ring-down, which-well state); an **accelerometer** on the base
measures the actual injected vibration so the bath is known from reality.

## Core order

| # | Search term (AliExpress) | Qty | Variant / note |
|---|---|---|---|
| 1 | `Teensy 4.1` | 2 | **buy genuine** (PJRC/DigiKey/Mouser), not a clone — see Sourcing |
| 2 | `TPA3116D2 mono amplifier board` | 2 | mono; one spare |
| 3 | `feeler gauge set steel` | 2 | assorted spring-steel blade thicknesses (find stiffness) |
| 4 | `spring steel shim sheet 0.1mm` | 1 | ~100×100 mm — cut identical cantilevers for a chain |
| 5 | `spring steel shim sheet 0.15mm` | 1 | second thickness |
| 6 | `N52 neodymium disc magnet assortment kit` | 1 | well tuning |
| 7 | `N52 neodymium disc magnet 4x2mm 50pcs` | 1 | tip magnets |
| 8 | `N52 neodymium disc magnet 6x3mm 50pcs` | 1 | tuning + inter-element coupling magnets |
| 9 | `tactile transducer bass shaker 4ohm` | 1 | **4 Ω, 40–50 W** (drives the whole array) |
| 10 | `CD74HC4067 16 channel analog mux module` | 2 | read 8+ Hall channels on one ADC |
| 11 | `perfboard prototype pcb assortment` | 1 | past breadboard as channels grow |
| 12 | `DRV5055 linear hall sensor module` | 10 | DRV5055**A1** (ratiometric analog); one per p-bit + spares |
| 13 | `ADXL335 module` | 2 | ±3 g analog; bath-level monitor |
| 14 | `BF350 strain gauge` | 1 pack | backup position sensor |
| 15 | `INA125 module` | 2 | instrumentation amp for the strain-gauge backup |
| 16 | `M3 standoff screw nut assortment kit` | 1 | clamp + adjustable magnet bracket |
| 17 | `breadboard jumper wire kit` | 1 | |
| 18 | `sorbothane anti vibration pad` | 1 | isolation feet (condo-floor-friendly) |
| 19 | `hookup wire kit 24awg` | 1 | |

**No power supply** — driven from the user's variable bench supply (set ~24 V,
current-limit 2–3 A for the amp).

## Optional (lab-grade, recommended)

| # | Search term | Qty | Note |
|---|---|---|---|
| 20 | `manual linear stage 13mm` | 2 | precise, repeatable magnet positioning → reproducible `ΔV` |
| 21 | `digital force gauge 10N` | 1 | static force–displacement → clean `ΔV`, `x₀` |
| 22 | `usb microscope 1000x` | 1 | independent video cross-check of which-well state |

**Rough total:** ~$120 core, ~$150 with optional (Teensy excluded; ~$35 genuine).

## Sourcing — AliExpress vs buy-genuine

- **Buy genuine (do NOT clone): Teensy 4.1 (#1).** PJRC/DigiKey/Mouser/SparkFun/
  Adafruit. AliExpress Teensy clones often carry counterfeit chips and fail; a
  dead brain costs another 2-week wait. A real distributor is also likely *faster*.
- **Verify before buying on AliExpress:** the Hall (#12) is genuinely **DRV5055A1**
  (not a cheaper `49E` sold under "linear hall"); and INA125 (#15) modules are
  uncommon — fall back to the `INA125P` chip on perfboard, or DigiKey/Mouser.
- **Everything else** is reliable on AliExpress (magnets especially).

Place two orders in parallel: AliExpress (slow boat, commodity) + a distributor
(Teensy, optionally DRV5055/INA125).

## Notes

- **Audio shield deliberately dropped.** An earlier draft specced a Teensy audio
  shield for a codec DAC; we drive the shaker with **PWM + a one-resistor-one-cap
  filter** instead. The Teensy 4.x has no true DAC, the audio-shield path is tied
  to PJRC's C++ library, and — decisively — the accelerometer measures the real
  injected vibration, so the drive needs no codec-grade fidelity. This keeps the
  firmware pure Rust on one chip with a single clock (phase-locked drive + capture
  for the SR step).
- **Open before the printed brackets (S1b) are finalized:** the shaker's mounting
  bolt pattern + impedance/wattage, and the chosen tip/tuning magnet diameters.
