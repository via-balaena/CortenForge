# CortenForge — Work Queue

## Critical Path (biological navigation paper)

| # | Item | Status |
|---|------|--------|
| 1 | **P2 — Stochastic resonance** (E. coli) — coupling sweep + ΔV operating envelope | **Validated** |
| 2 | **P4 — Metachronal phase-lag** (Ctenophore) — phase-lag sweep at 4 coupling strengths | **Validated** |
| 3 | **P11 — Deliberate instability** (Peregrine) — bifurcation sensitivity sweep | Failed (no sharp peak); data reframed as P2 envelope |
| 4 | **P6 — Scale-invariant encoding** (Octopus) — SR sweep at N=4, 8, 16 | **Next** |
| 5 | **P1 — Topological encoding** (E. coli) — phase ordering vs amplitude test | **Next** (parallel with P6) |
| 6 | Tighten chapter prose for publication | After P6+P1 |
| 7 | Contact Extropic / Normal Computing with mdbook link | After prose pass |

## Parallel Track (not blocking paper)

| # | Item | Status |
|---|------|--------|
| 8 | cf-design `param_gradient → mass_properties` wiring | Not started |
| 9 | cf-design ↔ sim-ml-chassis AD bridge | Not started |
| 10 | Sim Phases 7-11 — specs in `sim/docs/todo/` | Not started |
| 11 | Actuator examples visual review (2-10 of 10) | 40/40 PASS, needs visual review |
| 12 | GPU physics pipeline — spec at `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md` | Specced, not implemented |
| 13 | Item 10 grade sweep — 10/12 crates remaining | Deferred to post-ThermCircuitEnv |

## Later Experiments (after paper ships)

| # | Item | Gap |
|---|------|-----|
| 14 | P7 — Predictive forward model (Dragonfly) | experiment_4.rs exists, medium effort |
| 15 | P5 — Compressed command (Octopus) | Heterogeneous ΔV circuit, medium effort |
| 16 | Experiment 2: Ctenophore metachronal wave | Needs simplified drag model (no CFD) |
| 17 | Experiment 5: Peregrine vortex coupling | Needs reformulation to Langevin domain |
