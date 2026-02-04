# Simulation — Future Work

Complete roadmap for the simulation engine. Phase 1 completed tasks 1–8, 12,
plus cleanup items C1–C2. Phase 2 covers the remaining 17 items.

## Priority Framework

| Axis | Definition |
|------|-----------|
| **RL Impact** | How directly this unblocks reinforcement-learning training workflows (batch stepping, observation fidelity, GPU throughput). |
| **Correctness** | Whether this fixes a simulation bug, stub, or semantic mismatch that produces wrong results today. |
| **Effort** | Implementation size: **S** (< 200 LOC), **M** (200–800 LOC), **L** (800–2 000 LOC), **XL** (> 2 000 LOC). |

Priority is **Correctness + RL Impact**, tie-broken by inverse Effort.

## Phase 2 Priority Table

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File |
|---|------|-----------|-------------|--------|---------------|------|
| 1 | `<default>` class resolution | High | **Critical** | S | None | [future_work_2.md](./future_work_2.md) |
| 2 | Contact condim (1/4/6) + friction cones | Medium | **Critical** | L | None | [future_work_2.md](./future_work_2.md) |
| 3 | `<contact>` pair/exclude | Medium | High | M | None | [future_work_2.md](./future_work_2.md) |
| 4 | Spatial tendons + wrapping | Low | High | M | None | [future_work_2.md](./future_work_2.md) |
| 5 | Site-transmission actuators | Low | High | M | #4 | [future_work_2.md](./future_work_2.md) |
| 6 | Height field + SDF collision (MJCF wiring) | Low | Medium | S | None | [future_work_3.md](./future_work_3.md) |
| 7 | Deferred sensors (JointLimitFrc, TendonLimitFrc) | Low | Medium | S | None | [future_work_3.md](./future_work_3.md) |
| 8 | `<general>` actuator MJCF attributes | Low | Medium | S | None | [future_work_3.md](./future_work_3.md) |
| 9 | Batched simulation | **High** | Low | L | None | [future_work_3.md](./future_work_3.md) |
| 10 | GPU acceleration | **High** | Low | XL | #9 | [future_work_3.md](./future_work_3.md) |
| 11 | Deformable body integration | Medium | Low | XL | None | [future_work_4.md](./future_work_4.md) |
| 12 | Analytical derivatives (mjd_*) | **High** | Low | XL | None | [future_work_4.md](./future_work_4.md) |
| 13 | Full implicit integrator | Low | Medium | L | None | [future_work_4.md](./future_work_4.md) |
| 14 | Keyframes, mocap bodies, user callbacks | Medium | Low | L | None | [future_work_4.md](./future_work_4.md) |
| 15 | Newton solver | Low | Low | XL | None | [future_work_5.md](./future_work_5.md) |
| 16 | Sleeping / body deactivation | Low | Low | M | None | [future_work_5.md](./future_work_5.md) |
| 17 | SOR relaxation for PGS | Low | Low | S | None | [future_work_5.md](./future_work_5.md) |

## Dependency Graph

```
   ┌───┐
   │ 4 │ Spatial Tendons
   └─┬─┘
     │
     ▼
   ┌───┐
   │ 5 │ Site-Transmission Actuators
   └───┘

   ┌───┐
   │ 9 │ Batched Simulation
   └─┬─┘
     │
     ▼
   ┌────┐
   │ 10 │ GPU Acceleration
   └────┘

   ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 1 │  │ 2 │  │ 3 │  │ 6 │  │ 7 │  │ 8 │  │ 11 │  │ 12 │  │ 13 │  │ 14 │  │ 15 │
   └───┘  └───┘  └───┘  └───┘  └───┘  └───┘  └────┘  └────┘  └────┘  └────┘  └────┘
   (all independent)
```

## File Map

| File | Phase | Group | Items | Description |
|------|-------|-------|-------|-------------|
| [future_work_1.md](./future_work_1.md) | 1 (complete) | All | P1 #1–12 | In-place Cholesky, sparse LDL, CG solver, tendon pipeline, muscle pipeline, sensors, integrators, general actuators |
| [future_work_2.md](./future_work_2.md) | 2 | A — Correctness | #1–5 | Default classes, contact condim, contact pair/exclude, spatial tendons, site-transmission actuators |
| [future_work_3.md](./future_work_3.md) | 2 | A+B — Correctness + Scaling | #6–10 | Height field/SDF, deferred sensors, general actuators, batched sim, GPU |
| [future_work_4.md](./future_work_4.md) | 2 | C — Physics Completeness | #11–14 | Deformable bodies, analytical derivatives, full implicit integrator, keyframes/mocap |
| [future_work_5.md](./future_work_5.md) | 2 | D — Quality of Life + Appendix | #15–17 | Newton solver, sleeping, SOR relaxation, deferred items, cross-reference |
