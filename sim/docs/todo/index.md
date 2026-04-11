# Simulation — Future Work

Phases 1-7 complete. Phases 8-13 remain (parallel work streams).
Completed phase specs deleted 2026-04-11 — preserved in git history.

## Trackers

| Document | What |
|----------|------|
| [V1_DEFERRED_ITEMS.md](./V1_DEFERRED_ITEMS.md) | 8 medium-priority items for v1.0 |
| [POST_V1_ROADMAP.md](./POST_V1_ROADMAP.md) | Full tracker — 179 DT items (56 done, 122 open) |
| [archived/future_work_10b-10j](./archived/) | Detailed per-item context for DT-1 through DT-160 |

## Phase Specs (Not Yet Executed)

| Directory | Phase | Focus |
|-----------|-------|-------|
| phase7_mjcf_parsing_defaults/ | 7 | MJCF parsing & defaults |
| phase8_constraint_solver_gaps/ | 8 | Constraint & solver gaps |
| phase9_collision_completeness/ | 9 | Collision completeness |
| phase10_flex_pipeline/ | 10 | Flex pipeline |
| phase11_derivatives/ | 11 | Derivatives |
| phase12_conformance_test_suite/ | 12 | MuJoCo conformance (v1.0 gate) |
| phase13_remaining_core/ | 13 | `<composite>`, plugin system |
| mesh_collision_fixes/ | — | Mesh collision specific fixes |
| s41_runtime_flags/ | — | Runtime flag wiring (complete) |
| v1_cleanup/ | — | v1.0 cleanup tasks |

## Phase Dependency Graph

```
Phases 8-11: parallel
  8: Constraints/solver
  9: Collision
  10: Flex pipeline
  11: Derivatives
       │
       ▼
  12: Conformance test suite (v1.0 gate)
       │
       ▼
  13: Remaining core (composite, plugins)
```
