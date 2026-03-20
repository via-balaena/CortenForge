# Simulation — Future Work

Roadmap for the simulation engine. Phases 1–7 are complete. Phases 8–13
remain (parallel work streams).

## Active Documents

| Document | Purpose |
|----------|---------|
| **[POST_V1_ROADMAP.md](./POST_V1_ROADMAP.md)** | 121 open deferred tasks (DT-1–DT-180), organized by theme |
| **spec_fleshouts/** | Active phase specs (7–13) — see listing below |

## Active Spec Fleshouts

| Directory | Phase | Focus |
|-----------|-------|-------|
| `spec_fleshouts/phase7_mjcf_parsing_defaults/` | 7 | MJCF parsing & defaults |
| `spec_fleshouts/phase8_constraint_solver_gaps/` | 8 | Constraint & solver gaps |
| `spec_fleshouts/phase9_collision_completeness/` | 9 | Collision completeness |
| `spec_fleshouts/phase10_flex_pipeline/` | 10 | Flex pipeline |
| `spec_fleshouts/phase11_derivatives/` | 11 | Derivatives |
| `spec_fleshouts/phase12_conformance_test_suite/` | 12 | MuJoCo conformance test suite (v1.0 gate) |
| `spec_fleshouts/phase13_remaining_core/` | 13 | `<composite>`, plugin system |
| `spec_fleshouts/mesh_collision_fixes/` | 9 | Mesh collision specific fixes |
| `spec_fleshouts/s41_runtime_flags/` | 2 | Runtime flag wiring (complete) |
| `spec_fleshouts/v1_cleanup/` | — | v1.0 cleanup tasks |

## v1.0 Remaining Work (Phases 8–13)

> **Full details with phase ordering:** [ROADMAP_V1.md](../ROADMAP_V1.md)

| Phase | Focus | Key Items | Count |
|-------|-------|-----------|-------|
| 8 | Constraint & solver gaps | DT-19/23/25/28/32/33/39 | 7 |
| 9 | Collision completeness | §43/50/54/57/65, DT-70 | 6 |
| 10 | Flex pipeline | §42A-i–v, §42B | 6 |
| 11 | Derivatives | §58, DT-47/51/52/53/54 | 6 |
| 12 | **Conformance test suite** | **§45**, DT-97 (golden file generation) | **2** |
| 13 | Remaining core | §46, §66 | 2 |

### Dependency Graph (Open Phases)

```
   ┌──────────────────────────────────────────────────────────┐
   │ Phases 8–11: Parallel work streams                       │
   │                                                          │
   │  Phase 8: Constraints/solver (7 items)                   │
   │  Phase 9: Collision (6 items)                             │
   │  Phase 10: Flex pipeline (6 items)                       │
   │  Phase 11: Derivatives (6 items)                         │
   └──────────────────────┬───────────────────────────────────┘
                          │
                          ▼
         ┌─────────────────────────────────┐
         │    Phase 12: §45               │
         │  MuJoCo conformance test suite  │
         │       (THE v1.0 GATE)           │
         └────────────────┬────────────────┘
                          │
                          ▼
   ┌──────────────────────────────────────┐
   │ Phase 13: Remaining core             │
   │ §46 <composite>, §66 plugin system   │
   └──────────────────────────────────────┘
```

## Completed Work (Phases 1–7)

All completed. Phases 1–7 covered 45+ items across 18 spec documents.

| Phase | Focus | Status |
|-------|-------|--------|
| 1 | Foundation (12 items) | Cholesky, LDL, CG, tendons, muscles, sensors, integrators |
| 2 | Correctness (#1–16) | Defaults, contacts, tendons, actuators, batched sim, GPU, deformables, derivatives, Newton, sleeping |
| 3A-i | Parser fundamentals (#18–22) | `<include>`, `<frame>`, `childclass`, `<site>`, `springlength` |
| 3A-pre | Flex unification (#6B) | Flex DOFs unified into qpos/qvel |
| 3A-ii | Inertia + contact (#23–27) | Contact parameters, flex parsing, body-coupled flex |
| 3A-iii | Constraint system (#28–32) | Friction loss, PGS/CG unification, pyramidal cones |
| 3A-iv | Noslip + actuator (#33–37) | Noslip, actlimited, gravcomp, adhesion, tendon equality |
| 3A-v | Joint/fluid (#38–41) | Joint limits, tendon wrapping, fluid forces, disableflags |
| 4 | Core data fields | §51, §56, CVEL, lazy gates, sensor refactor |
| 5 | Actuator completeness | 12 items done, 12 deferred to post-v1.0 |
| 6 | Sensor completeness | 6 items done, 6 deferred to post-v1.0 |
| 7 | MJCF parsing & defaults | 11 items done, 4 deferred to post-v1.0 |

## Archived File Map

All completed spec documents are in `archived/`. The file map below
preserves the cross-reference for historical lookup.

| File | Items | Description |
|------|-------|-------------|
| [future_work_1.md](./archived/future_work_1.md) | P1 #1–12 | Foundation: Cholesky, LDL, CG, tendons, muscles, sensors, integrators |
| [future_work_2.md](./archived/future_work_2.md) | #1–5 | Default classes, contact condim, pair/exclude, spatial tendons, site-transmission |
| [future_work_3.md](./archived/future_work_3.md) | #6–10 | Height field/SDF, deferred sensors, general actuators, batched sim, GPU |
| [future_work_4.md](./archived/future_work_4.md) | #11–14 | Deformable bodies, analytical derivatives, implicit integrator, keyframes/mocap |
| [future_work_5.md](./archived/future_work_5.md) | #15–16 | Newton solver, sleeping |
| [future_work_6.md](./archived/future_work_6.md) | #18–22 | `<include>`, `<frame>`, `childclass`, `<site>`, `springlength` |
| [future_work_6b...](./archived/future_work_6b_precursor_to_7.md) | #6B | Flex solver unification |
| [future_work_7.md](./archived/future_work_7.md) | #23–27 | Contact parameters, flex parsing, body-coupled flex |
| [future_work_8.md](./archived/future_work_8.md) | #28–32 | Friction loss, PGS/CG, pyramidal cones |
| [future_work_9.md](./archived/future_work_9.md) | #33–37 | Noslip, actlimited, gravcomp, adhesion, tendon equality |
| [future_work_10.md](./archived/future_work_10.md) | #38–42F | Joint limits, fluid forces, flex runtime, trait architecture |
| [future_work_10b–10j](./archived/) | DT-1–110 | Deferred task tracker (9 files) |
| [future_work_11.md](./archived/future_work_11.md) | #43–45 | Mesh inertia, conformance test suite |
| [future_work_12.md](./archived/future_work_12.md) | #46–50 | `<composite>`, URDF, SIMD, CCD |
| [future_work_13.md](./archived/future_work_13.md) | #51–55 | Body accumulators, `mj_inverse`, `step1`/`step2`, heightfield |
| [future_work_14.md](./archived/future_work_14.md) | #56–59 | Subtree fields, SDF options, position derivatives, name lookup |
| [future_work_15.md](./archived/future_work_15.md) | #60–64a | `slidercrank`, missing sensors, `dynprm`, spring energy, `jnt_margin` |
| [future_work_16.md](./archived/future_work_16.md) | #65–66 | Mesh convex hull, plugin system |
| [future_work_17.md](./archived/future_work_17.md) | #67–71 | GPU pipeline |
| [DT16_DT90_SPEC.md](./archived/DT16_DT90_SPEC.md) | DT-16, DT-90 | Correctness bug specs |

Archived spec fleshouts: `archived/phase3_core_api/`, `archived/phase4_lazy_eval/`,
`archived/phase5_actuator_completeness/`, `archived/phase6_sensor_completeness/`.
