# Simulation — Future Work

Complete roadmap for the simulation engine. Phase 1 (12 items) and Phase 2
(16 items) are complete. Phase 3A tiers i–iv (items #18–37) are complete.
Phase 3A-v complete through §41. Roadmap Phases 1–5 all done.
Next: Phases 6–11 (parallel work streams).

Remaining work is organized by the **v1.0 Roadmap** (see
[ROADMAP_V1.md](../ROADMAP_V1.md)) which triages all ~135 remaining tasks into
**~67 core v1.0** items and **~68 post-v1.0** items. v1.0 is defined as: a Rust
MuJoCo implementation that parses standard MJCF, produces conformant physics
results, and exposes the standard public API surface.

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

## Phase 3 — v1.0 Core + Post-v1.0 Extensions

### Completed (3A-i through 3A-iv)

All items #18–37 are complete. See individual future_work files for details.

| Tier | Items | Description | Status |
|------|-------|-------------|--------|
| 3A-i: Parser Fundamentals | #18–22 | `<include>`, `<frame>`, `childclass`, `<site>` orientation, tendon `springlength` | ✅ All done |
| 3A-precursor: Flex Unification | #6B | Flex solver unification (subsumes #42) | ✅ Done |
| 3A-ii: Inertia + Contact Params | #23–27, #27B–F | Contact parameters, flex parsing, body-coupled flex | ✅ All done |
| 3A-iii: Constraint System | #28–32 | Friction loss, PGS/CG unification, pyramidal cones | ✅ All done |
| 3A-iv: Noslip + Actuator/Dynamics | #33–37 | Noslip, actlimited, gravcomp, adhesion, tendon equality | ✅ All done |

### Completed (3A-v)

Items #38–41 are complete. §41 done (Roadmap Phase 2). Roadmap Phase 3 (Core API) and Phase 4 (Core Data Fields) also complete.

| # | Item | Correctness | Effort | File | Status |
|---|------|-------------|--------|------|--------|
| 38 | Ball / free joint limits | High | M | [future_work_10.md](./future_work_10.md) | ✅ |
| 39 | `wrap_inside` tendon wrapping | High | M | [future_work_10.md](./future_work_10.md) | ✅ |
| 40 | Fluid / aerodynamic forces | High | L | [future_work_10.md](./future_work_10.md) | ✅ |
| 40a | Fluid velocity derivatives | High | L | [future_work_10.md](./future_work_10.md) | ✅ |
| 40b | Tendon visualization data | Low | S | [future_work_10.md](./future_work_10.md) | ✅ |
| 40c | Sleep filtering for fluid forces | Medium | S | [future_work_10.md](./future_work_10.md) | ✅ |

### v1.0 Remaining Work

> **Full details with phase ordering:** [ROADMAP_V1.md](../ROADMAP_V1.md)

The roadmap organizes remaining core work into 13 phases. Here is the
master task table showing all v1.0 items, their source, and which roadmap
phase they belong to. Items are grouped by source file for easy cross-reference.

#### From future_work_10.md (§40d+)

| # | Item | Core v1.0? | Roadmap Phase | Status |
|---|------|-----------|---------------|--------|
| 40d | Sparse Jacobian for fluid derivatives | No — perf opt | Post-v1.0 | |
| 40e | Refactor `mj_jac_site` → `mj_jac_point` | No — refactor | Post-v1.0 | |
| ~~41~~ | ~~`disableflags`/`enableflags` runtime wiring~~ | **Yes** | Phase 2 | ✅ Done |
| ~~42~~ | ~~`<flex>`/`<flexcomp>` parsing~~ | — | — | Subsumed by §6B |
| 42A-i | Sparse flex edge Jacobian | **Yes** | Phase 10 | |
| 42A-ii | `flex_rigid`/`flexedge_rigid` flags | **Yes** | Phase 10 | |
| 42A-iii | `flexedge_length`/`flexedge_velocity` fields | **Yes** | Phase 10 | |
| 42A-iv | Flex self-collision (BVH/SAP + narrowphase) | **Yes** | Phase 10 | |
| 42A-v | Flex-flex cross-object collision | **Yes** | Phase 10 | |
| 42B | Flex bending: cotangent Laplacian | **Yes** | Phase 10 | |
| 42C | `FlexElasticityModel` trait (NeoHookean) | No — extension | Post-v1.0 | |
| 42D | `ActuatorGainModel` trait (SEA) | No — extension | Post-v1.0 | |
| 42E | `ContactSolver` trait (XPBD) | No — extension | Post-v1.0 | |
| 42F | `SimBuilder` composition | No — extension | Post-v1.0 | |

#### From future_work_10b–10j (Deferred Tasks DT-1–101)

~40 of 100 deferred tasks are core v1.0. Full list with phase assignments
in [ROADMAP_V1.md](../ROADMAP_V1.md). Key items by category:

**Correctness bugs (Phase 1):** ~~DT-74~~ (done), ~~DT-75~~ (done), ~~DT-35~~ (done), ~~DT-78~~ (done), ~~DT-16~~ (done), ~~DT-90~~ (done)

**Runtime flags (Phase 2):** ~~§41~~ (done, subsumes DT-61, DT-93, DT-94, DT-95); ~~DT-99~~ (done), ~~DT-100~~ (done);
DT-96 (lazy energy eval, post-v1.0), DT-97 (golden file generation, Phase 12)

**Core API (Phase 3):** ~~DT-21~~ (done), ~~DT-41~~ (done), ~~DT-79~~ (done)

**Actuators (Phase 5) ✅:** ~~DT-6~~ (done), ~~§63~~ (done), ~~DT-56~~ (done), ~~DT-57~~ (done), ~~DT-59~~ (done), ~~DT-77~~ (done), ~~DT-58~~ (done), ~~DT-8~~ (done), ~~DT-9~~ (partially done — parsing landed, runtime → DT-107/108), ~~DT-60~~ (subsumed by §41 S4.2a), ~~§61~~ (done), ~~DT-106~~ (done). Deferred to post-v1.0: DT-104, DT-105, DT-107, DT-108, DT-110, DT-111–116

**Sensors (Phase 6):** §62, DT-62, DT-63, DT-64, DT-102, DT-109. Deferred to post-v1.0: DT-120 (Camera), DT-118 (contactForce), DT-119 (ray-geom filter)

**Parsing/defaults (Phase 7):** DT-2, DT-3, DT-11, DT-13, DT-14, DT-85, DT-88

**Constraints/solver (Phase 8):** DT-19, DT-23, DT-25, DT-28, DT-32, DT-33, DT-39

**Collision (Phase 9):** DT-70

**Derivatives (Phase 11):** DT-47, DT-51, DT-52, DT-53, DT-54

**Post-v1.0:** DT-1, DT-4, DT-5, DT-7, DT-10, DT-12, DT-15, DT-17, DT-18,
DT-20, DT-22, DT-24, DT-26, DT-27, DT-29, DT-30, DT-31, DT-34, DT-36,
DT-37, DT-38, DT-40, DT-42, DT-43, DT-44, DT-45, DT-46, DT-48, DT-49,
DT-50, DT-55, DT-65, DT-66, DT-67, DT-68, DT-69, DT-71, DT-72, DT-73,
DT-76, DT-80, DT-81, DT-82, DT-83, DT-84, DT-86, DT-87, DT-89, DT-91, DT-92,
DT-96, DT-101

#### From future_work_11.md (#43–45)

| # | Item | Core v1.0? | Roadmap Phase | Status |
|---|------|-----------|---------------|--------|
| 43 | Mesh inertia modes (exact/shell/convex/legacy) | **Yes** | Phase 9 | |
| 44 | Legacy crate deprecation | No — cleanup | Post-v1.0 | |
| 45 | **MuJoCo conformance test suite** | **Yes** | **Phase 12 (gate)** | |

#### From future_work_12.md (#46–50)

| # | Item | Core v1.0? | Roadmap Phase | Status |
|---|------|-----------|---------------|--------|
| 46 | `<composite>` procedural body generation | **Yes** | Phase 13 | |
| 47 | URDF loader completeness | No — alt format | Post-v1.0 | |
| 48 | SIMD utilization audit | No — perf opt | Post-v1.0 | |
| 49 | Non-physics MJCF elements | No — rendering | Post-v1.0 | |
| 50 | Continuous collision detection (CCD) | **Yes** | Phase 9 | |

#### From future_work_13.md (#51–55)

| # | Item | Core v1.0? | Roadmap Phase | Status |
|---|------|-----------|---------------|--------|
| ~~51~~ | ~~`cacc`/`cfrc_int`/`cfrc_ext` body accumulators~~ | **Yes** | Phase 3 | ✅ Done |
| ~~52~~ | ~~`mj_inverse()` inverse dynamics~~ | **Yes** | Phase 3 | ✅ Done |
| ~~53~~ | ~~`step1()`/`step2()` split stepping API~~ | **Yes** | Phase 3 | ✅ Done |
| 54 | Heightfield collision gaps | **Yes** | Phase 9 | |
| 55 | `*_user` custom data fields | **Yes** | Phase 7 | |

#### From future_work_14.md (#56–59)

| # | Item | Core v1.0? | Roadmap Phase | Status |
|---|------|-----------|---------------|--------|
| ~~56~~ | ~~`subtree_linvel`/`subtree_angmom` Data fields~~ | **Yes** | Phase 4 | ✅ Done |
| 57 | `sdf_iterations`/`sdf_initpoints` options | **Yes** | Phase 9 | |
| 58 | `mjd_smooth_pos` position derivatives | **Yes** | Phase 11 | |
| ~~59~~ | ~~`mj_name2id`/`mj_id2name` name lookup~~ | **Yes** | Phase 3 | ✅ Done |

#### From future_work_15.md (#60–64a)

| # | Item | Core v1.0? | Roadmap Phase | Status |
|---|------|-----------|---------------|--------|
| 60 | `springinertia` joint attribute | **Yes** | Phase 7 | |
| 61 | `slidercrank` transmission | **Yes** | Phase 5 | |
| 62 | Missing sensor types | **Yes** | Phase 6 | |
| 63 | `dynprm` array 3→10 | **Yes** | Phase 5 | |
| 64 | Ball/free joint spring energy | **Yes** | Phase 7 | |
| 64a | `jnt_margin` limit activation | **Yes** | Phase 7 | |

#### From future_work_16.md (#65–66)

| # | Item | Core v1.0? | Roadmap Phase | Status |
|---|------|-----------|---------------|--------|
| 65 | Mesh convex hull auto-computation | **Yes** | Phase 9 | |
| 66 | Plugin/extension system | **Yes** | Phase 13 | |

#### From future_work_17.md (#67–71) — All Post-v1.0

| # | Item | Core v1.0? | Status |
|---|------|-----------|--------|
| 67 | GPU forward kinematics | No — GPU | |
| 68 | GPU collision broad-phase | No — GPU | |
| 69 | GPU collision narrow-phase | No — GPU | |
| 70 | GPU constraint solver | No — GPU | |
| 71 | Full GPU step | No — GPU | |

### v1.0 Roadmap Phase Summary

| Phase | Focus | Key Items | Count |
|-------|-------|-----------|-------|
| 1 | Correctness bugs | ~~DT-74~~ (done), ~~DT-75~~ (done), ~~DT-35~~ (done), ~~DT-78~~ (done), ~~DT-16~~ (done), ~~DT-90~~ (done) | 6 (6 done) |
| 2 | Runtime flag wiring | ~~§41~~ (done, subsumes DT-61, DT-93, DT-94, DT-95); ~~DT-99~~ (done), ~~DT-100~~ (done) | 1 (+2 follow-ups, all done) |
| 3 | Core API gaps | ~~DT-21~~ (done), ~~DT-41~~ (done), ~~§51~~ (done), ~~§52~~ (done), ~~§53~~ (done), ~~§59~~ (done), ~~DT-79~~ (done) | 7 (7 done) |
| 4 | Core Data fields | ~~§51~~ (done), ~~§56~~ (done) + CVEL fixes, lazy gates, sensor refactor, DT-103 | ✅ All done |
| 5 | Actuator completeness | ~~DT-6~~, ~~§63~~, ~~DT-56~~, ~~DT-57~~, ~~DT-59~~, ~~DT-77~~, ~~DT-8~~, ~~§61~~, ~~DT-9~~ (partial), ~~DT-58~~, ~~DT-60~~, ~~DT-106~~ — all done. Deferred: DT-104/105/107/108/110/111–116 → post-v1.0 | ✅ All done |
| 6 | Sensor completeness | §62, DT-62/63/64, DT-102, DT-109 | 6 |
| 7 | MJCF parsing & defaults | DT-2/3/11/13/14/85/88, §55/60/64/64a | 11 |
| 8 | Constraint & solver gaps | DT-19/23/25/28/32/33/39 | 7 |
| 9 | Collision completeness | §43/50/54/57/65, DT-70 | 6 |
| 10 | Flex pipeline | §42A-i–v, §42B | 6 |
| 11 | Derivatives | §58, DT-47/51/52/53/54 | 6 |
| 12 | **Conformance test suite** | **§45**, DT-97 (golden file generation) | **2** |
| 13 | Remaining core | §46, §66 | 2 |
| | | **Total core v1.0** | **~67** |

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

### Phase 3 — v1.0 Critical Path

```
   ┌─────────────────────────────────────────────────────┐
   │ Roadmap Phase 1: Fix correctness bugs               │
   │ ✓DT-74, ✓DT-75, ✓DT-35, ✓DT-78, ✓DT-16, ✓DT-90   │
   └──────────────────────┬──────────────────────────────┘
                          │
                          ▼
   ┌─────────────────────────────────────────────────────┐
   │ Roadmap Phase 2: Runtime flags                     ✓ │
   │ ✓§41, ✓DT-61, ✓DT-93, ✓DT-94, ✓DT-95               │
   └──────────────────────┬──────────────────────────────┘
                          │
                          ▼
   ┌─────────────────────────────────────────────────────┐
   │ Roadmap Phase 3: Core API                          ✓ │
   │ ✓DT-21, ✓DT-41, ✓§51, ✓§52, ✓§53, ✓§59, ✓DT-79    │
   └──────────────────────┬──────────────────────────────┘
                          │
                          ▼
   ┌──────────────────────────────────────────────────────────┐
   │ Roadmap Phases 4–11: Parallel work streams               │
   │                                                          │
   │  Phase 4: Data fields (✓§51, ✓§56)              ✅ DONE     │
   │  Phase 5: Actuators (10 core items)             ✅ DONE     │
   │  Phase 6: Sensors (6 items)                              │
   │  Phase 7: Parsing/defaults (11 items)                    │
   │  Phase 8: Constraints/solver (7 items)                   │
   │  Phase 9: Collision (6 items)                             │
   │  Phase 10: Flex pipeline (6 items)                       │
   │  Phase 11: Derivatives (6 items)                         │
   │                                                          │
   │  Note: §52→§51 dependency resolved. All independent.      │
   └──────────────────────┬───────────────────────────────────┘
                          │
                          ▼
         ┌─────────────────────────────────┐
         │    Roadmap Phase 12: §45        │
         │  MuJoCo conformance test suite  │
         │       (THE v1.0 GATE)           │
         └────────────────┬────────────────┘
                          │
                          ▼
   ┌──────────────────────────────────────┐
   │ Roadmap Phase 13: Remaining core     │
   │ §46 <composite>, §66 plugin system   │
   └──────────────────────────────────────┘
```

### Post-v1.0

```
   Trait architecture: §42C → §42D → §42E → §42F (sequential)

   GPU pipeline: §67 → §68 → §69 → §70 → §71 (strict chain)

   Performance: §40d, §40e, §48, DT-18/20/24/29/34/36/37/38/40/42/43/44/
                48/49/55/76/82/83/91/92 (all independent)

   Extensions: DT-4/26/27/30/50/66/67/68/73/86/87 (all independent)

   Low-priority compat: DT-1/5/7/10/12/15/17/22/31/65/72/80/81/84/89
                         §44/47/49 (all independent)
```

## File Map

| File | Phase | Items | Description |
|------|-------|-------|-------------|
| [future_work_1.md](./future_work_1.md) | 1 (complete) | P1 #1–12 | In-place Cholesky, sparse LDL, CG solver, tendon pipeline, muscle pipeline, sensors, integrators, general actuators |
| [future_work_2.md](./future_work_2.md) | 2 (complete) | #1–5 | Default classes, contact condim, contact pair/exclude, spatial tendons, site-transmission actuators |
| [future_work_3.md](./future_work_3.md) | 2 (complete) | #6–10 | Height field/SDF, deferred sensors, general actuators, batched sim, GPU Phase 10a |
| [future_work_4.md](./future_work_4.md) | 2 (complete) | #11–14 | Deformable bodies, analytical derivatives, full implicit integrator, keyframes/mocap |
| [future_work_5.md](./future_work_5.md) | 2 (complete) | #15–16 | Newton solver, sleeping (~~#17~~ dropped) |
| [future_work_6.md](./future_work_6.md) | 3 (complete) | #18–22 | `<include>`, `<frame>`, `childclass`, `<site>` orientation, tendon `springlength` |
| [future_work_6b_precursor_to_7.md](./future_work_6b_precursor_to_7.md) | 3 (complete) | #6B | Flex solver unification (subsumes #42) |
| [future_work_7.md](./future_work_7.md) | 3 (complete) | #23–27, #27B–F | Contact parameters, flex parsing, body-coupled flex |
| [future_work_8.md](./future_work_8.md) | 3 (complete) | #28–32 | Friction loss, PGS/CG unified constraints, pyramidal cones |
| [future_work_9.md](./future_work_9.md) | 3 (complete) | #33–37 | Noslip, actlimited, gravcomp, adhesion, tendon equality |
| [future_work_10.md](./future_work_10.md) | 3 (§41 done) | #38–42F | Joint limits ✅, tendon wrapping ✅, fluid forces ✅, fluid derivatives ✅, tendon viz ✅, fluid sleep ✅, ~~disableflags~~ ✅, flex runtime (Phase 10), trait architecture (post-v1.0) |
| [future_work_10b.md](./future_work_10b.md)–[10j](./future_work_10j.md) | 3 | DT-1–110 | Deferred task tracker (9 groups across 10b–10j) — ~44 core v1.0, ~53 post-v1.0, +~~DT-99~~ (done)/~~DT-100~~ (done)/DT-101 post-§41, DT-107–110 post-Spec D |
| [future_work_11.md](./future_work_11.md) | 3 | #43–45 | Mesh inertia, legacy crate deprecation, **conformance test suite** |
| [future_work_12.md](./future_work_12.md) | 3 | #46–50 | `<composite>`, URDF, SIMD, non-physics MJCF, CCD |
| [future_work_13.md](./future_work_13.md) | 3 | #51–55 | Body accumulators, `mj_inverse`, `step1`/`step2`, heightfield gaps, `*_user` data |
| [future_work_14.md](./future_work_14.md) | 3 | #56–59 | Subtree fields, SDF options, position derivatives, name lookup |
| [future_work_15.md](./future_work_15.md) | 3 | #60–64a | `springinertia`, `slidercrank`, missing sensors, `dynprm`, ball/free spring energy, `jnt_margin` |
| [future_work_16.md](./future_work_16.md) | 3 | #65–66 | Mesh convex hull, plugin/extension system |
| [future_work_17.md](./future_work_17.md) | 3 | #67–71 | GPU pipeline (all post-v1.0) |
