# Simulation — Future Work

Complete roadmap for the simulation engine. Phase 1 (12 items) and Phase 2 (16 items)
are complete. Phase 3 covers the remaining gaps to full MuJoCo parity, ordered
for optimal implementation sequence — parser fundamentals first, then contact
system, actuator/dynamics, constraints, physics, cleanup, and finally the
conformance test suite.

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

Items are ordered for optimal implementation sequence — each tier builds on
the previous. Within each tier, items proceed from most foundational to most
independent.

### 3A — MuJoCo Alignment (Items #18–45)

Core correctness fixes, missing features, and the conformance test suite that gates
them all. Items #19–44 are **prerequisites** to #45 (the conformance suite).

#### 3A-i: Parser Fundamentals (Items #18–22)

MJCF parsing gaps that cause incorrect model compilation. These must be correct
before any runtime physics testing is meaningful.

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 18 | `<include>` file support + `<compiler>` element | **High** | **Critical** | L | None | [future_work_6.md](./future_work_6.md) | ✅ Done |
| 19 | `<frame>` element parsing + `childclass` | Medium | High | M | #18 | [future_work_6.md](./future_work_6.md) | ✅ Done |
| 20 | `childclass` attribute (edge cases) | Medium | High | S | #19 | [future_work_6.md](./future_work_6.md) | ⚡ Partial (core in #19) |
| 21 | `<site>` euler/axisangle/xyaxes/zaxis orientation | Medium | High | S | None | [future_work_6.md](./future_work_6.md) | ⚡ Partial (core in #19) |
| 22 | Tendon `springlength` MJCF parsing | Low | Medium | S | None | [future_work_6.md](./future_work_6.md) | |

#### 3A-ii: Inertia + Contact Parameter Foundation (Items #23–27)

Inertia computation and the contact parameter combination layer — the foundation
that all contact constraint assembly builds on.

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 23 | `<compiler>` `exactmeshinertia` | Low | Medium | S | None | [future_work_7.md](./future_work_7.md) | |
| 24 | Friction combination: geometric mean → element-wise max | Medium | **Critical** | S | None | [future_work_7.md](./future_work_7.md) | |
| 25 | `geom/@priority` — contact priority | Medium | Medium | S | None (soft: after #24) | [future_work_7.md](./future_work_7.md) | |
| 26 | `solmix` attribute | Low | Medium | S | None | [future_work_7.md](./future_work_7.md) | |
| 27 | Contact margin/gap runtime effect | Medium | **Critical** | S | None | [future_work_7.md](./future_work_7.md) | |

#### 3A-iii: Constraint System Overhaul (Items #28–32)

Friction loss migration, PGS/CG unification, and advanced constraint features.
Items #28→#29→#30 form a strict dependency chain.

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 28 | Friction loss: tanh passive → Huber constraint rows (Newton) | Medium | **Critical** | M | None | [future_work_8.md](./future_work_8.md) | |
| 29 | Friction loss: tanh passive → constraint rows (PGS/CG) | Medium | **Critical** | M | #28 | [future_work_8.md](./future_work_8.md) | |
| 30 | PGS/CG unified constraints: penalty → solver rows | Medium | **Critical** | L | #29 | [future_work_8.md](./future_work_8.md) | |
| 31 | `solreffriction` per-direction solver params | Low | Medium | M | None | [future_work_8.md](./future_work_8.md) | |
| 32 | Pyramidal friction cones | Low | Medium | L | None | [future_work_8.md](./future_work_8.md) | |

#### 3A-iv: Noslip + Actuator/Dynamics + Tendon Equality (Items #33–37)

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 33 | Noslip post-processor (PGS/CG) | Low | Medium | S | None | [future_work_9.md](./future_work_9.md) | |
| 34 | `actlimited` / `actrange` — activation state clamping | **High** | **Critical** | S–M | None | [future_work_9.md](./future_work_9.md) | |
| 35 | `gravcomp` — body gravity compensation | Medium | High | S–M | None | [future_work_9.md](./future_work_9.md) | |
| 36 | Body-transmission actuators (adhesion) | Low | High | M | None | [future_work_9.md](./future_work_9.md) | |
| 37 | Tendon equality constraints | Low | High | M | None | [future_work_9.md](./future_work_9.md) | |

#### 3A-v: Constraint/Joint + Physics + Pipeline (Items #38–42)

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 38 | Ball / free joint limits (swing-twist cones) | Low | High | M | None | [future_work_10.md](./future_work_10.md) | |
| 39 | `wrap_inside` algorithm (tendon wrapping) | Low | High | M | None | [future_work_10.md](./future_work_10.md) | |
| 40 | Fluid / aerodynamic forces | Medium | High | L | None | [future_work_10.md](./future_work_10.md) | |
| 41 | `disableflags` — runtime disable flag effects | Low | Medium | M | None | [future_work_10.md](./future_work_10.md) | |
| 42 | `<flex>` / `<flexcomp>` MJCF deformable parsing | Medium | Medium | L | None | [future_work_10.md](./future_work_10.md) | |

#### 3A-vi: Cleanup + Conformance Test Suite (Items #43–45)

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 43 | Geom `shellinertia` | Low | Medium | S | None | [future_work_11.md](./future_work_11.md) | |
| 44 | Legacy crate deprecation | Low | Medium | S | None | [future_work_11.md](./future_work_11.md) | |
| 45 | MuJoCo conformance test suite | Medium | **Critical** | XL | #19–44 | [future_work_11.md](./future_work_11.md) | |

### 3B — Format Completeness + Performance (Items #46–50)

Closes remaining model format gaps and performance optimization. Lower priority
than 3A but needed for full parity.

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 46 | `<composite>` element (procedural generation) | Medium | Medium | XL | #18 | [future_work_12.md](./future_work_12.md) | |
| 47 | URDF loader completeness | Low | Medium | M | None | [future_work_12.md](./future_work_12.md) | |
| 48 | SIMD utilization audit + wiring | Medium | Low | M | None | [future_work_12.md](./future_work_12.md) | |
| 49 | Non-physics MJCF elements | Low | Low | S | None | [future_work_12.md](./future_work_12.md) | |
| 50 | CCD (continuous collision detection) | Low | Medium | L | None | [future_work_12.md](./future_work_12.md) | |

### 3C — API + Pipeline Completeness (Items #51–59)

Missing API surface, runtime behaviors, data fields, and derivatives.

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 51 | `cacc` / `cfrc_int` / `cfrc_ext` — body force accumulators | Medium | Low | M | None | [future_work_13.md](./future_work_13.md) | |
| 52 | `mj_inverse()` — inverse dynamics API | Medium | Low | M | #51 | [future_work_13.md](./future_work_13.md) | |
| 53 | `mj_step1` / `mj_step2` — split stepping API | Medium | Low | S | None | [future_work_13.md](./future_work_13.md) | |
| 54 | Heightfield collision gaps (hfield↔mesh/plane/hfield) | Low | Medium | L | None | [future_work_13.md](./future_work_13.md) | |
| 55 | `*_user` custom data fields | Low | Low | S | None | [future_work_13.md](./future_work_13.md) | |
| 56 | `subtree_linvel` / `subtree_angmom` as Data fields | Low | Low | S | None | [future_work_14.md](./future_work_14.md) | |
| 57 | `sdf_iterations` / `sdf_initpoints` option attributes | Low | Low | S | None | [future_work_14.md](./future_work_14.md) | |
| 58 | `mjd_smooth_pos` — analytical position derivatives | **High** | Low | L | None | [future_work_14.md](./future_work_14.md) | |
| 59 | `mj_name2id` / `mj_id2name` — public name lookup API | Medium | Low | S | None | [future_work_14.md](./future_work_14.md) | |

### 3D — Edge-Case Features + Infrastructure (Items #60–66)

Lower-priority features for uncommon models, plus extensibility infrastructure.

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 60 | `springinertia` — joint inertia-spring coupling | Low | Low | S | None | [future_work_15.md](./future_work_15.md) | |
| 61 | `slidercrank` actuator transmission | Low | Low | M | None | [future_work_15.md](./future_work_15.md) | |
| 62 | Missing sensor types (clock, jointactuatorfrc, etc.) | Low | Low | S | None | [future_work_15.md](./future_work_15.md) | |
| 63 | `dynprm` array size (3 → 10) | Low | Low | S | None | [future_work_15.md](./future_work_15.md) | |
| 64 | Ball/free joint spring energy | Low | Low | S | None | [future_work_15.md](./future_work_15.md) | |
| 65 | Mesh convex hull auto-computation | Low | Low | M | None | [future_work_16.md](./future_work_16.md) | |
| 66 | `<plugin>` / `<extension>` support | Low | Low | XL | None | [future_work_16.md](./future_work_16.md) | |

### 3E — GPU Pipeline (Items #67–71)

Progressively moves more pipeline stages to GPU, targeting full GPU-resident
simulation. Strict sequential chain — each phase depends on the previous.

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 67 | GPU forward kinematics (Phase 10b) | **High** | Low | XL | #10 | [future_work_17.md](./future_work_17.md) | |
| 68 | GPU collision broad-phase (Phase 10c) | **High** | Low | L | #67 | [future_work_17.md](./future_work_17.md) | |
| 69 | GPU collision narrow-phase (Phase 10d) | **High** | Low | XL | #68 | [future_work_17.md](./future_work_17.md) | |
| 70 | GPU constraint solver (Phase 10e) | **High** | Low | XL | #69 | [future_work_17.md](./future_work_17.md) | |
| 71 | Full GPU step (Phase 10f) | **High** | Low | L | #67–70 | [future_work_17.md](./future_work_17.md) | |

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
   Layer 0 — Foundation:
   ┌────┐
   │ 18 │ <include> + <compiler>  ✅ Done
   └─┬──┘
     │ unlocks #46
     ▼
   ┌────┐
   │ 46 │ <composite>
   └────┘

   3A — MuJoCo alignment (all prerequisites to #45):

   Tier 1 — Parser fundamentals (all independent):
   ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 19 │  │ 20 │  │ 21 │  │ 22 │  │ 23 │
   └────┘  └────┘  └────┘  └────┘  └────┘
   frame   chdcls  site-o  sprlen  exactm

   Tier 2 — Contact system (dependency chain + independents):
   ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 24 │  │ 25 │  │ 26 │  │ 27 │
   └────┘  └────┘  └────┘  └────┘
   fric-mx geom-p  solmix  margin     (all independent)

   ┌────┐
   │ 28 │ friction loss → Huber rows (Newton)
   └──┬─┘
     │
   ┌────┐
   │ 29 │ friction loss → constraint rows (PGS/CG)
   └──┬─┘
     │
   ┌────┐
   │ 30 │ PGS/CG unified constraints (penalty → solver)
   └────┘

   ┌────┐  ┌────┐  ┌────┐
   │ 31 │  │ 32 │  │ 33 │
   └────┘  └────┘  └────┘
   solrf   pyramdl noslip     (independent)

   Tier 3 — Actuator/dynamics (all independent):
   ┌────┐  ┌────┐  ┌────┐
   │ 34 │  │ 35 │  │ 36 │
   └────┘  └────┘  └────┘
   actlim  gravc   adhesn

   Tier 4 — Constraint/joint features (all independent):
   ┌────┐  ┌────┐  ┌────┐
   │ 37 │  │ 38 │  │ 39 │
   └────┘  └────┘  └────┘
   ten-eq  ball-lm wrap-i

   Tier 5 — Physics + pipeline (all independent):
   ┌────┐  ┌────┐  ┌────┐
   │ 40 │  │ 41 │  │ 42 │
   └────┘  └────┘  └────┘
   fluid   dsbflg  flex

   Tier 6 — Cleanup (all independent):
   ┌────┐  ┌────┐
   │ 43 │  │ 44 │
   └────┘  └────┘
   shell   legacy

         ┌─────────────────────────────────┐
         │              45                 │ MuJoCo conformance test suite
         └─────────────────────────────────┘
         (depends on all #19–44)

   3B — Format + performance (independent):
   ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 46 │  │ 47 │  │ 48 │  │ 49 │  │ 50 │
   └────┘  └────┘  └────┘  └────┘  └────┘
   compst  URDF    SIMD    MJCF-NP CCD

   3C — API + pipeline completeness:
   ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 51 │  │ 52 │  │ 53 │  │ 54 │  │ 55 │  │ 56 │  │ 57 │  │ 58 │  │ 59 │
   └──┬─┘  └──┬─┘  └────┘  └────┘  └────┘  └────┘  └────┘  └────┘  └────┘
   body-acc inverse
     │        │
     └────────┘
     (#52 depends on #51)

   3D — Edge-case features + infrastructure:
   ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 60 │  │ 61 │  │ 62 │  │ 63 │  │ 64 │  │ 65 │  │ 66 │
   └────┘  └────┘  └────┘  └────┘  └────┘  └────┘  └────┘
   sprgI   slider  sensor  dynprm  ballE   cvxhul  plugin

   3E — GPU pipeline (strict sequential chain):
   ┌────┐    ┌────┐    ┌────┐    ┌────┐    ┌────┐    ┌────┐
   │ 10 │───▶│ 67 │───▶│ 68 │───▶│ 69 │───▶│ 70 │───▶│ 71 │
   └────┘    └────┘    └────┘    └────┘    └────┘    └────┘
   10a done   10b FK    10c BP    10d NP    10e CS    10f Full
```

## File Map

| File | Phase | Group | Items | Description |
|------|-------|-------|-------|-------------|
| [future_work_1.md](./future_work_1.md) | 1 (complete) | All | P1 #1–12 | In-place Cholesky, sparse LDL, CG solver, tendon pipeline, muscle pipeline, sensors, integrators, general actuators |
| [future_work_2.md](./future_work_2.md) | 2 (complete) | A — Correctness | #1–5 | Default classes, contact condim, contact pair/exclude, spatial tendons, site-transmission actuators |
| [future_work_3.md](./future_work_3.md) | 2 (complete) | A+B — Correctness + Scaling | #6–10 | Height field/SDF, deferred sensors, general actuators, batched sim, GPU Phase 10a |
| [future_work_4.md](./future_work_4.md) | 2 (complete) | C — Physics Completeness | #11–14 | Deformable bodies, analytical derivatives, full implicit integrator, keyframes/mocap |
| [future_work_5.md](./future_work_5.md) | 2 (complete) | D — Quality of Life + Appendix | #15–16 (~~#17~~ dropped) | Newton solver, sleeping, deferred items, cross-reference |
| [future_work_6.md](./future_work_6.md) | 3 | 3A-i — Parser Fundamentals | #18–22 | ~~`<include>` + `<compiler>`~~ ✅, ~~`<frame>` + `childclass`~~ ✅, ~~`<site>` orientation~~ ✅, tendon `springlength` |
| [future_work_7.md](./future_work_7.md) | 3 | 3A-ii — Inertia + Contact Parameters | #23–27 | `exactmeshinertia`, friction combination, `geom/@priority`, `solmix`, contact margin/gap |
| [future_work_8.md](./future_work_8.md) | 3 | 3A-iii — Constraint System Overhaul | #28–32 | Friction loss (Newton + PGS/CG), PGS/CG unified constraints, `solreffriction`, pyramidal cones |
| [future_work_9.md](./future_work_9.md) | 3 | 3A-iv — Noslip + Actuator/Dynamics | #33–37 | Noslip PGS/CG, `actlimited`/`actrange`, `gravcomp`, adhesion actuators, tendon equality |
| [future_work_10.md](./future_work_10.md) | 3 | 3A-v — Constraint/Joint + Physics | #38–42 | Ball/free joint limits, `wrap_inside`, fluid forces, `disableflags`, flex/flexcomp |
| [future_work_11.md](./future_work_11.md) | 3 | 3A-vi — Cleanup + Conformance | #43–45 | Geom `shellinertia`, legacy crate deprecation, MuJoCo conformance test suite |
| [future_work_12.md](./future_work_12.md) | 3 | 3B — Format Completeness + Performance | #46–50 | `<composite>`, URDF completeness, SIMD audit, non-physics MJCF, CCD |
| [future_work_13.md](./future_work_13.md) | 3 | 3C — API + Pipeline Completeness | #51–55 | Body accumulators, `mj_inverse`, `step1`/`step2`, heightfield gaps, `*_user` data |
| [future_work_14.md](./future_work_14.md) | 3 | 3C — Data Fields + Derivatives + API | #56–59 | Subtree fields, SDF options, position derivatives, name lookup |
| [future_work_15.md](./future_work_15.md) | 3 | 3D — Edge-Case Features | #60–64 | `springinertia`, `slidercrank`, missing sensors, `dynprm`, ball/free spring energy |
| [future_work_16.md](./future_work_16.md) | 3 | 3D — Mesh + Plugin Infrastructure | #65–66 | Mesh convex hull, `<plugin>`/`<extension>` support |
| [future_work_17.md](./future_work_17.md) | 3 | 3E — GPU Pipeline | #67–71 | GPU FK, broad-phase, narrow-phase, constraint solver, full GPU step |
