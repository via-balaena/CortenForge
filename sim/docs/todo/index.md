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
| 20 | `childclass` attribute (edge cases) | Medium | High | S | #19 | [future_work_6.md](./future_work_6.md) | ✅ Done |
| 21 | `<site>` euler/axisangle/xyaxes/zaxis orientation | Medium | High | S | None | [future_work_6.md](./future_work_6.md) | ✅ Done |
| 22 | Tendon `springlength` MJCF parsing | Low | Medium | S | None | [future_work_6.md](./future_work_6.md) | ✅ Done |

#### 3A-precursor: Flex Solver Unification (Item #6B)

Unifies the deformable simulation pipeline with the rigid constraint solver.
Flex vertex DOFs join the global `qpos`/`qvel` state, edge constraints enter
the unified Jacobian, bending acts as passive forces, and the separate XPBD
solver is removed. Subsumes #42 (`<flex>`/`<flexcomp>` parsing).

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 6B | Flex Solver Unification | Medium | **Critical** | XL | None | [future_work_6b_precursor_to_7.md](./future_work_6b_precursor_to_7.md) | ✅ Done |

#### 3A-ii: Inertia + Contact Parameter Foundation (Items #23–27)

Inertia computation and the contact parameter combination layer — the foundation
that all contact constraint assembly builds on.

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 23 | `<compiler>` `exactmeshinertia` | Low | Medium | S | None | [future_work_7.md](./future_work_7.md) | ✅ Done |
| 24 | Friction combination: geometric mean → element-wise max | Medium | **Critical** | S | None | [future_work_7.md](./future_work_7.md) | ✅ Done |
| 25 | `geom/@priority` — contact priority | Medium | Medium | S | None (soft: after #24) | [future_work_7.md](./future_work_7.md) | ✅ Done |
| 26 | `solmix` attribute | Low | Medium | S | None | [future_work_7.md](./future_work_7.md) | ✅ Done |
| 27 | Contact margin/gap runtime effect | Medium | **Critical** | S | None | [future_work_7.md](./future_work_7.md) | ✅ Done |
| 27B | Flex child element parsing (`<contact>`, `<elasticity>`, `<edge>`) | Low | Medium | S–M | #27 | [future_work_7.md](./future_work_7.md) | ✅ Done |
| 27C | Passive edge spring-damper forces + `compute_edge_solref` cleanup | Low | Medium | S–M | #27B | [future_work_7.md](./future_work_7.md) | ✅ Done |
| 27D | Flex `body`/`node` attr parsing + `flexvert_bodyid` wiring | Low | High | S–M | #27B | [future_work_7.md](./future_work_7.md) | ✅ Done |
| 27E | `<flexcomp mass="...">` + deprecate non-standard `density` | Low | Medium | S | #27B | [future_work_7.md](./future_work_7.md) | ✅ Done |
| 27F | Body-coupled flex vertex integration (CRBA + FK) | Low | **Critical**† | L–XL | #27D, #27E | [future_work_7.md](./future_work_7.md) | ✅ Done |

†Critical for body-attached bare `<flex>`; irrelevant for `<flexcomp>` (all current tests).

#### 3A-iii: Constraint System Overhaul (Items #28–32)

Friction loss migration, PGS/CG unification, and advanced constraint features.
All items are independent (#29 is the core architectural migration).

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 28 | Friction loss solref/solimp: per-DOF/per-tendon params | Low | Medium | S | None | [future_work_8.md](./future_work_8.md) | ✅ |
| 29 | PGS/CG unified constraint migration (friction loss + penalty → solver rows) | Medium | **Critical** | L | None | [future_work_8.md](./future_work_8.md) | ✅ |
| 30 | Flex collision contype/conaffinity filtering | Low | Medium | S | None | [future_work_8.md](./future_work_8.md) | ✅ |
| 31 | `solreffriction` per-direction solver params | Low | Medium | M | None | [future_work_8.md](./future_work_8.md) | ✅ |
| 32 | Pyramidal friction cones | Low | Medium | L | None | [future_work_8.md](./future_work_8.md) | ✅ |

#### 3A-iv: Noslip + Actuator/Dynamics + Tendon Equality (Items #33–37)

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 33 | Noslip post-processor (PGS/CG) | Low | Medium | S | None | [future_work_9.md](./future_work_9.md) | ✅ |
| 34 | `actlimited` / `actrange` — activation state clamping | **High** | **Critical** | S–M | None | [future_work_9.md](./future_work_9.md) | ✅ |
| 35 | `gravcomp` — body gravity compensation | Medium | High | S–M | None | [future_work_9.md](./future_work_9.md) | ✅ |
| 36 | Body-transmission actuators (adhesion) | Low | High | M | None | [future_work_9.md](./future_work_9.md) | ✅ |
| 37 | Tendon equality constraints | Low | High | M | None | [future_work_9.md](./future_work_9.md) | ✅ |

#### 3A-v: Constraint/Joint + Physics + Pipeline (Items #38–42)

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 38 | Ball / free joint limits (rotation cone via quat log) | Low | High | M | None | [future_work_10.md](./future_work_10.md) | ✅ Complete |
| 39 | `wrap_inside` algorithm (tendon wrapping) | Low | High | M | None | [future_work_10.md](./future_work_10.md) | |
| 40 | Fluid / aerodynamic forces | Medium | High | L | None | [future_work_10.md](./future_work_10.md) | |
| 41 | `disableflags` — runtime disable flag effects | Low | Medium | M | None | [future_work_10.md](./future_work_10.md) | |
| ~~42~~ | ~~`<flex>` / `<flexcomp>` MJCF deformable parsing~~ | — | — | — | — | [future_work_10.md](./future_work_10.md) | Subsumed by §6B |
| 42A-i | Sparse flex edge Jacobian (`flexedge_J`) | Low | **Critical** | L | §6B, #27D | [future_work_10.md](./future_work_10.md) | |
| 42A-ii | `flex_rigid` / `flexedge_rigid` boolean arrays | Low | Low | S | §6B | [future_work_10.md](./future_work_10.md) | |
| 42A-iii | `flexedge_length` / `flexedge_velocity` Data fields | Low | Low | S | §6B | [future_work_10.md](./future_work_10.md) | |
| 42A-iv | Flex self-collision dispatch (BVH/SAP + narrowphase) | Medium | **Critical** | L | §42A-ii, §30 | [future_work_10.md](./future_work_10.md) | |
| 42A-v | Flex-flex cross-object collision filtering | Low | High | M | §42A-iv, §30 | [future_work_10.md](./future_work_10.md) | |
| 42B | Flex bending: cotangent Laplacian + trait abstraction (Phase A) | Medium | **Critical** | L | §6B | [future_work_10.md](./future_work_10.md) | |
| 42C | `FlexElasticityModel` trait (Phase B) | Medium | Medium | L | §42B | [future_work_10.md](./future_work_10.md) | |
| 42D | `ActuatorGainModel` trait (Phase C) | Medium | Low | M | §42B | [future_work_10.md](./future_work_10.md) | |
| 42E | `ContactSolver` trait (Phase E) | Medium | Low | XL | §42B | [future_work_10.md](./future_work_10.md) | |
| 42F | `SimBuilder` composition (Phase D) | **High** | Low | L | §42C, §42D, §42E | [future_work_10.md](./future_work_10.md) | |

#### 3A-vi: Cleanup + Conformance Test Suite (Items #43–45)

| # | Item | RL Impact | Correctness | Effort | Prerequisites | File | Status |
|---|------|-----------|-------------|--------|---------------|------|--------|
| 43 | Mesh `inertia` attribute (shell inertia + mode enum) | Low | Medium | S–M | None | [future_work_11.md](./future_work_11.md) | |
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

   ┌─────┐
   │ 6B  │ Flex Solver Unification  ✅ Done (subsumes #42)
   └─────┘

   3A — MuJoCo alignment (all prerequisites to #45):

   Tier 1 — Parser fundamentals (all done):
   ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 19 │  │ 20 │  │ 21 │  │ 22 │  │ 23 │  ✅ All done
   └────┘  └────┘  └────┘  └────┘  └────┘
   frame   chdcls  site-o  sprlen  exactm

   Tier 2 — Contact parameters + flex parsing:
   ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 24 │  │ 25 │  │ 26 │  │ 27 │  ✅ All done
   └────┘  └────┘  └────┘  └────┘
   fric-mx geom-p  solmix  margin

   ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐
   │ 27B │──│ 27C │  │ 27D │  │ 27E │  (27B/C/D ✅ done; 27E = flexcomp mass)
   └──┬──┘  └─────┘  └──┬──┘  └──┬──┘
      │                  │        │
      │                  │  ┌─────┐
      │                  └──│ 27F │  body-coupled flex (CRBA + FK) [L–XL]
      │                     └─────┘
      │                  │ unlocks #42A-i
      └──────────────────┴── 27E depends on 27B; 27F depends on 27D+27E
                        ▼

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

   Tier 5 — Physics + pipeline + trait architecture:
   ┌────┐  ┌────┐  ┌────┐
   │ 40 │  │ 41 │  │ 42 │ ✅ (subsumed by §6B)
   └────┘  └────┘  └────┘
   fluid   dsbflg  ~~flex~~

   Flex runtime infrastructure (§42A, not prerequisites to #45):
   ┌───────┐  ┌────────┐  ┌─────────┐
   │ 42A-i │  │ 42A-ii │  │ 42A-iii │
   └───────┘  └────────┘  └─────────┘
   edge-J     rigid-flg   len/vel     (42A-i depends on #27D)

   Trait architecture (§42B–F, not prerequisites to #45):
   ┌─────┐
   │ 42B │  Phase A: FlexBendingModel (proves the pattern)
   └──┬──┘
      │
      ├────▶┌─────┐
      │     │ 42C │  FlexElasticityModel (Phase B)
      │     └──┬──┘
      │        │
      ├────▶┌─────┐
      │     │ 42D │  ActuatorGainModel (Phase C)
      │     └──┬──┘
      │        │
      └────▶┌─────┐
            │ 42E │  ContactSolver (Phase E)
            └──┬──┘
               │
  ┌────────────┼────────────┐
  │            │            │
  ▼            ▼            ▼
┌──────────────────────────────┐
│             42F              │ SimBuilder (Phase D)
└──────────────────────────────┘
(depends on §42C, §42D, §42E)

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

## Recommended Implementation Order

Items are grouped into implementation batches. Within each batch, items share
enough context (same subsystem, same files) to minimize re-reading overhead.
Review after each batch — the test suite validates the full stack at each
checkpoint.

### Batch 1 — Contact Parameters + Defaults Refactor (4 items + defaults, S effort) ✅ Complete

**Items:** #24, #25, #26, #27 + {27B–F flex rabbit hole} + defaults system refactor
**Files:** `mujoco_pipeline.rs` (contact assembly), `model_builder.rs`, `types.rs`, `parser.rs`, `defaults.rs`
**Why first:** Natural next step after §6b. Small, independent, and unblock the
constraint system overhaul. All touch the same contact combination code path.

**Defaults refactor (completed alongside Batch 1):**
The contact parameter work exposed that the defaults system needed a full
Option<T> refactoring pass. All element/defaults struct fields now use
`Option<T>` with `is_none()` guards in apply functions (no sentinel-value
detection for new fields). The four-stage pipeline (types → parser → merge →
apply) is complete for all 8 element types: geom (24 fields), joint (14),
site (11), tendon (13), actuator (18), sensor (3), mesh (1), pair (7).
New fields added: tendon solref/solimp/margin, actuator group/actlimited/
actrange/actearly/lengthrange, geom fromto/mesh/hfield defaults, material
on geom/site/tendon. All 558 conformance + 281 MJCF tests passing.

### Batch 2 — Constraint System (4 items, S-L effort, all independent) ✅ Complete

**Items:** #28, #29, #30, #31, #32 (all independent; #29 is the core migration)
**Files:** `mujoco_pipeline.rs` (constraint assembly, PGS/CG solver, Newton solver)
**Why second:** #29 is the meatiest remaining correctness gap: unified PGS/CG
constraint migration (friction loss tanh removal + penalty elimination in one
pass). #28 is a small solref fix. #31 and #32 are independent features that
touch the same constraint code, so batch them here.

### Batch 3 — Actuator/Dynamics + Noslip (5 items, S-M effort)

**Items:** #33, #34, #35, #36, #37
**Files:** `mujoco_pipeline.rs` (actuator pipeline, passive forces), `model_builder.rs`
**Why third:** Mostly independent of each other. #34 (`actlimited`) is highest
RL-impact item remaining. Can parallelize within the batch.

### Batch 4 — Joint/Constraint Features + Physics + Trait Architecture (5+5+3 items)

**Items:** ~~#38~~ ✅, #39, #40, #41, #43 (3A prerequisites to #45)
**Also:** §42A-i, §42A-ii, §42A-iii (flex runtime infrastructure, not prerequisites to #45)
**Also:** §42B → §42C, §42D, §42E (parallel) → §42F (trait architecture, not prerequisites to #45)
**Files:** `mujoco_pipeline.rs` (joint limits, tendon wrapping, fluid forces, disable flags, flex runtime infra, trait extraction)
**Why fourth:** Self-contained features that don't depend on Batches 2-3. #43
(shellinertia) is tiny and batches naturally with the joint/physics work.
§42A-i (sparse edge Jacobian) depends on #27D but is not a prerequisite to #45.
§42A-ii/iii are pure optimizations with no dependencies beyond §6B.
§42B lands here as part of 3A-v. Once it proves the trait pattern, §42C/D/E can
proceed in parallel with Batches 5–9 as capacity allows — they're additive capability,
not correctness prerequisites. §42F (SimBuilder) is the capstone after §42C/D/E land.

### Batch 5 — Cleanup + Conformance (2 items)

**Items:** #44, #45
**Files:** Legacy crate cleanup, then the conformance test suite (XL)
**Why last in 3A:** #45 depends on all #19-44. This is the capstone — validates
everything above against MuJoCo reference outputs.

### Batch 6 — Format + Performance (3B, 5 items)

**Items:** #46, #47, #48, #49, #50
**Files:** Distributed across parser, SIMD, collision
**Why here:** Lower priority than 3A correctness. #46 (`<composite>`) is the
largest item. Can cherry-pick high-value items (#48 SIMD audit) earlier if
needed for RL throughput.

### Batch 7 — API Surface (3C, 9 items)

**Items:** #51 → #52 (chain), #53, #54, #55, #56, #57, #58, #59
**Why here:** Missing API surface. #58 (position derivatives) is high RL-impact
and can be pulled forward if needed.

### Batch 8 — Edge Cases (3D, 7 items)

**Items:** #60-#66
**Why last CPU:** Uncommon features. Implement on-demand as specific models need
them, or batch before final conformance.

### Batch 9 — GPU Pipeline (3E, 5 items, strict chain)

**Items:** #67 → #68 → #69 → #70 → #71
**Why last:** Independent of CPU correctness. Each phase depends on the previous.
Start only after 3A is stable.

---

**High-value items to pull forward if RL throughput is urgent:**
- #34 `actlimited`/`actrange` (Batch 3) — blocks many RL environments
- #58 `mjd_smooth_pos` (Batch 7) — analytical position derivatives for learning
- #48 SIMD audit (Batch 6) — throughput improvement across the board

## File Map

| File | Phase | Group | Items | Description |
|------|-------|-------|-------|-------------|
| [future_work_1.md](./future_work_1.md) | 1 (complete) | All | P1 #1–12 | In-place Cholesky, sparse LDL, CG solver, tendon pipeline, muscle pipeline, sensors, integrators, general actuators |
| [future_work_2.md](./future_work_2.md) | 2 (complete) | A — Correctness | #1–5 | Default classes, contact condim, contact pair/exclude, spatial tendons, site-transmission actuators |
| [future_work_3.md](./future_work_3.md) | 2 (complete) | A+B — Correctness + Scaling | #6–10 | Height field/SDF, deferred sensors, general actuators, batched sim, GPU Phase 10a |
| [future_work_4.md](./future_work_4.md) | 2 (complete) | C — Physics Completeness | #11–14 | Deformable bodies, analytical derivatives, full implicit integrator, keyframes/mocap |
| [future_work_5.md](./future_work_5.md) | 2 (complete) | D — Quality of Life + Appendix | #15–16 (~~#17~~ dropped) | Newton solver, sleeping, deferred items, cross-reference |
| [future_work_6.md](./future_work_6.md) | 3 | 3A-i — Parser Fundamentals | #18–22 | ~~`<include>` + `<compiler>`~~ ✅, ~~`<frame>` + `childclass`~~ ✅, ~~`<site>` orientation~~ ✅, ~~tendon `springlength`~~ ✅ |
| [future_work_6b_precursor_to_7.md](./future_work_6b_precursor_to_7.md) | 3 | 3A-precursor — Flex Solver Unification | #6B | ~~Flex solver unification~~ ✅ (subsumes #42) |
| [future_work_7.md](./future_work_7.md) | 3 | 3A-ii — Inertia + Contact Parameters | #23–27, #27B–F | ~~`exactmeshinertia`~~ ✅, ~~friction combination~~ ✅, ~~`geom/@priority`~~ ✅, ~~`solmix`~~ ✅, ~~contact margin/gap~~ ✅, ~~flex `<contact>` parsing~~ ✅, ~~passive edge forces~~ ✅, ~~flex body/node~~ ✅, ~~`<flexcomp mass>`~~ ✅, ~~body-coupled flex CRBA+FK~~ ✅ (Option A) |
| [future_work_8.md](./future_work_8.md) | 3 | 3A-iii — Constraint System Overhaul | #28,29,30,31,32 | ~~Friction loss solref~~ ✅, ~~PGS/CG unified constraints~~ ✅, ~~flex collision filtering~~ ✅, ~~`solreffriction`~~ ✅, ~~pyramidal cones~~ ✅ |
| [future_work_9.md](./future_work_9.md) | 3 | 3A-iv — Noslip + Actuator/Dynamics | #33–37 | ~~Noslip PGS/CG~~ ✅, ~~`actlimited`/`actrange`~~ ✅, ~~`gravcomp`~~ ✅, ~~adhesion actuators~~ ✅, ~~tendon equality~~ ✅ |
| [future_work_10.md](./future_work_10.md) | 3 | 3A-v — Constraint/Joint + Physics + Trait Architecture | #38–42, §42A-i–v, §42B–F | ~~Ball/free joint limits~~ ✅, `wrap_inside`, fluid forces, `disableflags`, flex edge Jacobian + rigid flags + pre-computed fields, flex self-collision dispatch, flex-flex collision, flex bending trait, elasticity trait, actuator gain trait, contact solver trait, `SimBuilder` |
| [future_work_11.md](./future_work_11.md) | 3 | 3A-vi — Cleanup + Conformance | #43–45 | Geom `shellinertia`, legacy crate deprecation, MuJoCo conformance test suite |
| [future_work_12.md](./future_work_12.md) | 3 | 3B — Format Completeness + Performance | #46–50 | `<composite>`, URDF completeness, SIMD audit, non-physics MJCF, CCD |
| [future_work_13.md](./future_work_13.md) | 3 | 3C — API + Pipeline Completeness | #51–55 | Body accumulators, `mj_inverse`, `step1`/`step2`, heightfield gaps, `*_user` data |
| [future_work_14.md](./future_work_14.md) | 3 | 3C — Data Fields + Derivatives + API | #56–59 | Subtree fields, SDF options, position derivatives, name lookup |
| [future_work_15.md](./future_work_15.md) | 3 | 3D — Edge-Case Features | #60–64 | `springinertia`, `slidercrank`, missing sensors, `dynprm`, ball/free spring energy |
| [future_work_16.md](./future_work_16.md) | 3 | 3D — Mesh + Plugin Infrastructure | #65–66 | Mesh convex hull, `<plugin>`/`<extension>` support |
| [future_work_17.md](./future_work_17.md) | 3 | 3E — GPU Pipeline | #67–71 | GPU FK, broad-phase, narrow-phase, constraint solver, full GPU step |
