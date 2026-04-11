# Thermodynamic Computing — CortenForge initiative

Long-horizon research line: **mechanical p-bits + Langevin thermostat
+ EBM + RL, design → simulate → 3D-print**.

**Status (2026-04-11):** Phases 1-6 + D1 + D2 complete in `sim-thermostat`.
D4 (sim-to-real) deferred — needs 6 foundation layers first (see
`docs/LAYER_1_SPEC.md`).

## What's here

| Path | What |
|------|------|
| `01_vision/` | Why we're building this — vision, research directions, synthesis |
| `02_foundations/` | Theoretical foundations — working principles, chassis design, open questions |

Phase specs, recon logs, and findings were deleted 2026-04-11 (code speaks
for itself). Full history preserved in git.

## Code

The implementation lives in `sim/L0/thermostat/`:
- `LangevinThermostat` — Phase 1 (equipartition validated)
- `DoubleWellPotential` — Phase 3 (Kramers escape rate validated)
- `PairwiseCoupling` — Phase 4 (Ising correlations validated)
- `IsingLearner` — Phase 5 (Boltzmann learning validated)
- `GibbsSampler` — Phase 6 (three-way distribution comparison)
- `RatchetPotential` — D1 (Brownian ratchet motor, CEM-optimized)
- `OscillatingField` — D2 (stochastic resonance, 4-algorithm comparison)
