# CortenForge — Work Queue

## Critical Path (biological navigation paper)

| # | Item | Status |
|---|------|--------|
| 1 | **P2 — Stochastic resonance** (E. coli) — coupling sweep + ΔV operating envelope | **Validated** |
| 2 | **P4 — Metachronal phase-lag** (Ctenophore) — phase-lag sweep at 4 coupling strengths | **Validated** |
| 3 | **P11 — Deliberate instability** (Peregrine) — bifurcation sensitivity sweep | Failed (no sharp peak); data reframed as P2 envelope |
| 4 | **P6 — Scale-invariant encoding** (Octopus) — SR sweep at N=4, 8, 16 | **Validated** |
| 5 | **P1 — Topological encoding** (E. coli) — phase ordering vs amplitude test | Failed (amplitude dominates in Langevin domain) |
| 6 | Tighten chapter prose for publication | **Done** (two passes + staleness audit) |
| 7 | Publicist review + outreach planning | **Next** |
| 8 | N-scaling experiment (sync increases with N?) | **Done** — approximately extensive, no power law |
| 9 | Contact Extropic / Normal Computing with mdbook link | After review |

## Parallel Track (not blocking paper)

| # | Item | Status |
|---|------|--------|
| 10 | cf-design `param_gradient → mass_properties` wiring | Not started |
| 11 | cf-design ↔ sim-ml-chassis AD bridge | Not started |
| 12 | Sim Phases 7-11 — specs in `sim/docs/todo/` | Not started |
| 13 | Actuator examples visual review (2-10 of 10) | 40/40 PASS, needs visual review |
| 14 | GPU physics pipeline — spec at `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md` | Specced, not implemented |
| 15 | Per-crate grade sweep — 10/12 crates remaining | Deferred to post-ThermCircuitEnv |

## Later Experiments (after paper ships)

| # | Item | Gap |
|---|------|-----|
| 16 | N-scaling: coupling crossover mapping (J=1.0–2.0) | ~3 hrs, maps weak/strong transition |
| 17 | N-scaling: (J, δ) surface (8 J × 30 δ) | ~6 hrs, academic completeness |
| 18 | P7 — Predictive forward model (Dragonfly) | experiment_4.rs exists, predicted to validate (stat-mech) |
| 19 | P5 — Compressed command (Octopus) | Heterogeneous ΔV circuit, medium effort |
