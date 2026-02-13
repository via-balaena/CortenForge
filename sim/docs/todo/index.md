# Simulation — Future Work

Complete roadmap for the simulation engine. Phase 1 (12 items) and Phase 2 (16 items)
are complete. Phase 3 covers the remaining gaps to full MuJoCo parity, ordered
foundationally — each layer builds on the previous.

## Priority Framework

| Axis | Definition |
|------|-----------|
| **RL Impact** | How directly this unblocks reinforcement-learning training workflows (batch stepping, observation fidelity, GPU throughput). |
| **Correctness** | Whether this fixes a simulation bug, stub, or semantic mismatch that produces wrong results today. |
| **Effort** | Implementation size: **S** (< 200 LOC), **M** (200–800 LOC), **L** (800–2 000 LOC), **XL** (> 2 000 LOC). |

Priority is **Correctness + RL Impact**, tie-broken by inverse Effort.

## Phase 2 Priority Table (Complete)

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 1 | `<default>` class resolution | High | **Critical** | S | None | [future_work_2.md](./future_work_2.md) | ✅ Complete |
| 2 | Contact condim (1/4/6) + friction cones | Medium | **Critical** | L | None | [future_work_2.md](./future_work_2.md) | ✅ Complete |
| 3 | `<contact>` pair/exclude | Medium | High | M | None | [future_work_2.md](./future_work_2.md) | ✅ Complete |
| 4 | Spatial tendons + wrapping | Low | High | M | None | [future_work_2.md](./future_work_2.md) | ✅ Complete |
| 5 | Site-transmission actuators | Low | High | M | #4 | [future_work_2.md](./future_work_2.md) | ✅ Complete |
| 6 | Height field + SDF collision (MJCF wiring) | Low | Medium | S | None | [future_work_3.md](./future_work_3.md) | ✅ Complete |
| 7 | Deferred sensors (JointLimitFrc, TendonLimitFrc) | Low | Medium | S | None | [future_work_3.md](./future_work_3.md) | ✅ Complete |
| 8 | `<general>` actuator MJCF attributes | Low | Medium | S | None | [future_work_3.md](./future_work_3.md) | ✅ Complete |
| 9 | Batched simulation | **High** | Low | L | None | [future_work_3.md](./future_work_3.md) | ✅ Complete |
| 10 | GPU acceleration | **High** | Low | XL | #9 | [future_work_3.md](./future_work_3.md) | ✅ Phase 10a Complete |
| 11 | Deformable body integration | Medium | Low | XL | None | [future_work_4.md](./future_work_4.md) | ✅ Complete |
| 12 | Analytical derivatives (mjd_*) | **High** | Low | XL | None | [future_work_4.md](./future_work_4.md) | ✅ Complete |
| 13 | Full implicit integrator | Low | Medium | L | None | [future_work_4.md](./future_work_4.md) | ✅ Complete |
| 14 | Keyframes, mocap bodies, user callbacks | Medium | Low | L | None | [future_work_4.md](./future_work_4.md) | ✅ Complete |
| 15 | Newton solver | Low | Low | XL | None | [future_work_5.md](./future_work_5.md) | ✅ Phase C Complete |
| 16 | Sleeping / body deactivation | Low | Low | XL | None | [future_work_5.md](./future_work_5.md) | ✅ Complete |
| ~~17~~ | ~~SOR relaxation for PGS~~ | — | — | — | — | [future_work_5.md](./future_work_5.md) | Dropped (MuJoCo has no SOR; Newton solver supersedes) |

## Phase 3 Priority Table

Items are ordered foundationally — each group builds on the previous. Within each
group, items proceed from most foundational to most independent.

### 3A — Foundation + Core Correctness

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 18 | `<include>` file support + `<compiler>` element | **High** | **Critical** | L | None | [future_work_6.md](./future_work_6.md) | ✅ Done |
| 19 | MuJoCo conformance test suite | Medium | **Critical** | XL | None (benefits from #18) | [future_work_6.md](./future_work_6.md) | |
| 20 | Contact margin/gap runtime effect | Medium | **Critical** | S | None | [future_work_6.md](./future_work_6.md) | |
| 21 | Noslip post-processor | Low | Medium | S | None | [future_work_6.md](./future_work_6.md) | |
| 22 | Body-transmission actuators (adhesion) | Low | High | M | None | [future_work_6.md](./future_work_6.md) | |

### 3B — Constraint + Physics Completeness

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 23 | Tendon equality constraints | Low | High | M | None | [future_work_7.md](./future_work_7.md) | |
| 24 | `solreffriction` per-direction solver params | Low | Medium | M | None | [future_work_7.md](./future_work_7.md) | |
| 25 | Fluid / aerodynamic forces | Medium | High | L | None | [future_work_7.md](./future_work_7.md) | |
| 26 | `<flex>` / `<flexcomp>` MJCF deformable parsing | Medium | Medium | L | None | [future_work_7.md](./future_work_7.md) | |
| 27 | Ball / free joint limits (swing-twist cones) | Low | High | M | None | [future_work_7.md](./future_work_7.md) | |

### 3C — Format Completeness + Edge-Case Physics

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 28 | `<composite>` element (procedural generation) | Medium | Medium | XL | #18 | [future_work_8.md](./future_work_8.md) | |
| 29 | URDF loader completeness | Low | Medium | M | None | [future_work_8.md](./future_work_8.md) | |
| 30 | Pyramidal friction cones | Low | Medium | L | None | [future_work_8.md](./future_work_8.md) | |
| 31 | CCD (continuous collision detection) | Low | Medium | L | None | [future_work_8.md](./future_work_8.md) | |
| 32 | Non-physics MJCF elements | Low | Low | S | None | [future_work_8.md](./future_work_8.md) | |

### 3D — Performance + Crate Hygiene

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 33 | SIMD utilization audit + wiring | Medium | Low | M | None | [future_work_9.md](./future_work_9.md) | |
| 34 | Standalone crate consolidation | Low | Low | M | None | [future_work_9.md](./future_work_9.md) | |

### 3E — GPU Pipeline

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 35 | GPU forward kinematics (10b) | **High** | Low | XL | #10 | [future_work_10.md](./future_work_10.md) | |
| 36 | GPU collision broad-phase (10c) | **High** | Low | L | #35 | [future_work_10.md](./future_work_10.md) | |
| 37 | GPU collision narrow-phase (10d) | **High** | Low | XL | #36 | [future_work_10.md](./future_work_10.md) | |
| 38 | GPU constraint solver (10e) | **High** | Low | XL | #37 | [future_work_10.md](./future_work_10.md) | |
| 39 | Full GPU step (10f) | **High** | Low | L | #35–38 | [future_work_10.md](./future_work_10.md) | |

## Dependency Graph

### Phase 2 (complete)

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
   │ 10 │ GPU Acceleration (Phase 10a)
   └────┘

   ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 1 │  │ 2 │  │ 3 │  │ 6 │  │ 7 │  │ 8 │  │ 11 │  │ 12 │  │ 13 │  │ 14 │  │ 15 │  │ 16 │
   └───┘  └───┘  └───┘  └───┘  └───┘  └───┘  └────┘  └────┘  └────┘  └────┘  └────┘  └────┘
   (all complete; #17 dropped)
```

### Phase 3

```
   Layer 0 — Foundation (unlocks loading + verification):
   ┌────┐       ┌────┐
   │ 18 │       │ 19 │
   └─┬──┘       └────┘
     │          <include> +      Conformance
     │          <compiler>       tests verify
     │          unlocks #28      everything
     ▼
   ┌────┐
   │ 28 │ <composite>
   └────┘

   Layer 1 — Core correctness (contact + solver + actuator):
   ┌────┐  ┌────┐  ┌────┐
   │ 20 │  │ 21 │  │ 22 │
   └────┘  └────┘  └────┘
   margin   noslip  body-trn

   Layer 2 — Constraint + physics completeness:
   ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 23 │  │ 24 │  │ 25 │  │ 26 │  │ 27 │
   └────┘  └────┘  └────┘  └────┘  └────┘
   tendon-eq solreffric fluid  flex   ball-lim

   Layer 3 — Format + edge-case:
   ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 29 │  │ 30 │  │ 31 │  │ 32 │
   └────┘  └────┘  └────┘  └────┘
   URDF    pyramidal CCD   MJCF-NP

   Layer 4 — Performance + hygiene:
   ┌────┐  ┌────┐
   │ 33 │  │ 34 │
   └────┘  └────┘
   SIMD    crate-cleanup

   Layer 5 — GPU pipeline (strict sequential chain):
   ┌────┐    ┌────┐    ┌────┐    ┌────┐    ┌────┐
   │ 10 │───▶│ 35 │───▶│ 36 │───▶│ 37 │───▶│ 38 │──┐
   └────┘    └────┘    └────┘    └────┘    └────┘  │
   10a done   10b FK    10c BP    10d NP    10e CS  │
                                                    ▼
                                                  ┌────┐
                                                  │ 39 │
                                                  └────┘
                                                  10f Full
```

## File Map

| File | Phase | Group | Items | Description |
|------|-------|-------|-------|-------------|
| [future_work_1.md](./future_work_1.md) | 1 (complete) | All | P1 #1–12 | In-place Cholesky, sparse LDL, CG solver, tendon pipeline, muscle pipeline, sensors, integrators, general actuators |
| [future_work_2.md](./future_work_2.md) | 2 (complete) | A — Correctness | #1–5 | Default classes, contact condim, contact pair/exclude, spatial tendons, site-transmission actuators |
| [future_work_3.md](./future_work_3.md) | 2 (complete) | A+B — Correctness + Scaling | #6–10 | Height field/SDF, deferred sensors, general actuators, batched sim, GPU Phase 10a |
| [future_work_4.md](./future_work_4.md) | 2 (complete) | C — Physics Completeness | #11–14 | Deformable bodies, analytical derivatives, full implicit integrator, keyframes/mocap |
| [future_work_5.md](./future_work_5.md) | 2 (complete) | D — Quality of Life + Appendix | #15–16 (~~#17~~ dropped) | Newton solver, sleeping, deferred items, cross-reference |
| [future_work_6.md](./future_work_6.md) | 3 | 3A — Foundation + Core Correctness | #18–22 | `<include>` + `<compiler>`, conformance tests, contact margin/gap, noslip, body transmission |
| [future_work_7.md](./future_work_7.md) | 3 | 3B — Constraint + Physics | #23–27 | Tendon equality, solreffriction, fluid forces, flex MJCF, ball/free joint limits |
| [future_work_8.md](./future_work_8.md) | 3 | 3C — Format + Edge-Case | #28–32 | Composite bodies, URDF, pyramidal cones, CCD, non-physics MJCF |
| [future_work_9.md](./future_work_9.md) | 3 | 3D — Performance + Hygiene | #33–34 | SIMD utilization, crate consolidation |
| [future_work_10.md](./future_work_10.md) | 3 | 3E — GPU Pipeline | #35–39 | GPU FK, broad-phase, narrow-phase, constraint solver, full GPU step |
