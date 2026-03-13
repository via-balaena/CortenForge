# Future Work 10b — Deferred Item Tracker: Group 1 — Defaults & MJCF Parsing Gaps

Consolidated tracker for deferred sub-items identified during a systematic
audit of [future_work_1.md](./future_work_1.md) through
[future_work_9.md](./future_work_9.md). These are items that were explicitly
noted as "deferred", "out of scope", "follow-up", or "future work" within
completed tasks §1–§40 but were **not** assigned to any specific §41+ task.

Each item is numbered §DT-*XX* ("Deferred Tracker item XX") and organized
into thematic groups across files 10b–10j. Original files have been updated
with back-references to the corresponding §DT-*XX* entry.

**Items already tracked elsewhere are excluded.** If an item was later
addressed by a numbered task (e.g., pyramidal cones → §32, geom priority →
§25, flex self-collision → §42A-iv), it does not appear here.

---

## Index

| File | Group | Items | Count | T1 | T2 | T3 |
|------|-------|-------|-------|---:|---:|---:|
| [future_work_10b.md](./future_work_10b.md) | 1. Defaults & MJCF Parsing Gaps | DT-1 – DT-17, DT-123, DT-124 | 19 | 12 | 7 | 0 |
| [future_work_10c.md](./future_work_10c.md) | 2. Contact & Collision System | DT-18 – DT-27, ~~DT-94~~, ~~DT-95~~ (subsumed by §41), ~~DT-99~~ (done), ~~DT-100~~ (done), DT-101 | 15 | 2 | 8 | 5 |
| [future_work_10d.md](./future_work_10d.md) | 3. Tendon System | DT-28 – DT-35 | 8 | 2 | 4 | 2 |
| [future_work_10e.md](./future_work_10e.md) | 4. Solver Optimizations | DT-36 – DT-44 | 9 | 1 | 5 | 3 |
| [future_work_10f.md](./future_work_10f.md) | 5. Derivatives & Analytical Methods | DT-45 – DT-55, DT-157 – DT-159 | 14 | 2 | 9 | 3 |
| [future_work_10g.md](./future_work_10g.md) | 6. Actuator & Dynamics | DT-56 – DT-61, DT-107, DT-108, DT-110, DT-111 – DT-116 | 15 | 6 | 6 | 3 |
| [future_work_10h.md](./future_work_10h.md) | 7. Sensor Gaps ✅ | ~~DT-62~~ – ~~DT-64~~, DT-65, ~~DT-102~~, ~~DT-103~~ (done), ~~DT-109~~ — all core done (Phase 6). DT-65 deferred post-v1.0. | 7 | 1 | 4 | 2 |
| [future_work_10i.md](./future_work_10i.md) | 8. Flex / Deformable Body | DT-66 – DT-73, DT-85 – DT-90, DT-146 – DT-148 | 17 | 8 | 4 | 5 |
| [future_work_10j.md](./future_work_10j.md) | 9. Misc Pipeline & API | DT-74 – DT-84, DT-91 – DT-92, ~~DT-93~~ (subsumed by §41), DT-96 – DT-98, DT-160 | 17 | 7 | 4 | 6 |
| **Total** | | | **119** | **39** | **51** | **29** |

**Priority breakdown:** 24 Medium, 71 Low. No High — these are all sub-items
within completed tasks, not critical gaps.

**Tier key:**
- **T1** (plan + implement): Mechanical — parent spec already defines the "what."
  No iterative spec needed. 34 items.
- **T2** (grouped spec): Related items share one spec covering shared design
  decisions. Each item gets a "Step N" section. 46 items → ~15 spec groups.
- **T3** (individual spec): Algorithmic complexity, multiple valid approaches, or
  architectural decisions needing dedicated design. 26 items.

**T2 spec groups (cross-file):**
1. "Defaults Completeness" — DT-2, DT-11, DT-13, DT-14
2. "Actuator Attr Completeness" — DT-5 (remaining: ~~DT-8~~ done, ~~DT-9~~ partially done)
3. "Contact Force Cleanup" — DT-20, DT-24
4. "Solver Param Completeness" — DT-23, DT-32, DT-33
5. "XPBD Improvements" — DT-26, DT-27
6. "Tendon Joint Type Completeness" — DT-28, DT-31
7. "Sparse Storage" — DT-36, DT-37, DT-44, DT-48
8. "Solver Robustness" — DT-39, DT-40
9. "Derivative Extensions" — DT-47, DT-51, DT-52, DT-54
10. "FD Performance" — DT-49, DT-53
11. ~~"acc0 / dampratio" — DT-56, DT-57~~ **DONE** — Phase 5 Spec A
12. ~~"Length-Range Estimation" — DT-59, DT-77, DT-78~~ **DONE** — Phase 5 Spec A (DT-59, DT-77) + Phase 1 (DT-78)
13. "Sensor Completeness" — DT-62, DT-63, DT-64, DT-109
14. "Flex Collision Improvements" — DT-69 (+ DT-70 if not T3)
15. "Flexcomp Completeness" — DT-87, DT-88

---

## Group 1 — Defaults & MJCF Parsing Gaps (17 items)

**Spec approach:** T1 items implement directly; T2 items (DT-2/11/13/14) share one
"Defaults Completeness" spec; DT-5/8/9 share an "Actuator Attr Completeness" spec.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-1 | §1 | Mesh defaults — no `apply_to_mesh()` method; root-only mesh scale defaults deferred | Low | T1 |
| ~~DT-2~~ | §1 | ~~Equality constraint defaults — no `apply_to_equality()`, `solref`/`solimp` not in defaults structs~~ **DONE** — Phase 7 Spec A (commit `01ae59f`). `MjcfEqualityDefaults` struct, `apply_to_equality()` cascade. | Medium | T2 |
| ~~DT-3~~ | §6a | ~~File-based hfield loading from PNG (`<hfield file="terrain.png"/>`)~~ **DONE** — Phase 7 T1 (commit `cea5f4c`). PNG grayscale loading via `image` crate. | Low | T1 |
| DT-4 | §6b | `<sdf>` asset element for inline distance grids (no standard MuJoCo equivalent) | Low | T1 |
| DT-5 | §8 | `gaintype/biastype/dyntype="user"` — callback-based types require plugin system | Low | T2 |
| ~~DT-6~~ | §8 | ~~`actearly` parsed + defaultable but not wired to runtime (always standard order)~~ **DONE** — Phase 5 Session 1 (commit `dc12b8b`). Already wired; verified + 4 tests added. | Medium | T1 |
| DT-7 | §8 | `actdim` explicit override not supported (auto-detection only) | Low | T1 |
| ~~DT-8~~ | §8 | ~~Transmission types: `cranksite`, `slidersite`, `jointinparent` not supported~~ **Done** — Spec B (Phase 5 Session 7) | Low | T2 |
| ~~DT-9~~ | §8 | ~~`nsample`, `interp`, `delay` — MuJoCo 3.x interpolation actuator attributes~~ **Partially done** — parsing + model/data storage landed in Spec D (Phase 5 Session 12). Runtime interpolation → DT-107, `dyntype` gating → DT-108 | Low | T2 |
| DT-10 | §18 | Deferred `<compiler>` attributes: `fitaabb`, `usethread`, `alignfree`, `saveinertial`, `inertiagrouprange`, `<lengthrange>` child | Low | T1 |
| ~~DT-11~~ | §20 | ~~`range` not in `MjcfJointDefaults` as a defaultable attribute~~ **Already implemented** — verified during Phase 7 Spec A review (EGT-4). | Medium | T2 |
| DT-12 | §20 | Programmatic API enforcement that `worldbody.childclass` must be `None` | Low | T1 |
| ~~DT-13~~ | §22 | ~~`qpos_spring` not implemented — uses `qpos0` instead (equivalent only in default case)~~ **DONE** — Phase 7 Spec A (commit `3f70616`). `qpos_spring: Vec<f64>` on Model. | Medium | T2 |
| ~~DT-14~~ | §27 defaults | ~~Actuator type-specific defaults not yet defaultable (cylinder area/timeconst, muscle params)~~ **DONE** — Phase 7 Spec A (commit `01ae59f`). Shortcut names + type-specific fields on `MjcfActuatorDefaults`. | Medium | T2 |
| DT-15 | §27 defaults | Sentinel-value detection for `gear`/`kp`/`noise`/`cutoff` should migrate to `Option<T>`. Phase 7 Spec A added 14 new sentinel-detection fields in `apply_to_actuator()` (area, diameter, bias, muscle_timeconst, range, force, scale, lmin, lmax, vmax, fpmax, fvmax, gain) — these are the primary candidates for this migration. | Low | T1 |
| ~~DT-16~~ | §27B | ~~Flex `density` attribute location wrong — on `<flex>` in parser but not on `<flex>` in MuJoCo~~ **DONE** | Medium | T1 |
| DT-17 | §27 | Global `<option o_margin>` override deferred — per-geom margin is correct foundation. Phase 7 Spec B (§64a) implemented per-joint `jnt_margin`; this task covers the separate global `o_margin` option. | Low | T1 |
| DT-123 | Phase 7 Spec A | `IntVelocity` enum variant — `MjcfActuatorType` (types.rs:2292) is missing the `IntVelocity` variant. Defaults parsing works (`b"intvelocity"` dispatches to `parse_actuator_defaults()`), but concrete `<intvelocity>` elements cannot be created. Requires: variant in enum, parser match arm for concrete elements, expansion values in `builder/actuator.rs` (`gaintype=FIXED`, `biastype=AFFINE`, `dyntype=INTEGRATOR`). | Low | T1 |
| DT-124 | Phase 7 Spec A | Muscle sentinel detection for `<general dyntype="muscle">` path — MuJoCo's `mjs_setToMuscle()` uses `gainprm[0]==1` as sentinel (overwrites with 0.75 if equal). Only manifests for `<general dyntype="muscle" gainprm="1">`, not for `<muscle>` shortcut (which uses `range` field). CortenForge does not replicate this quirk. Known conformance divergence — rare edge case. | Low | T1 |
| DT-126 | Phase 7 Spec C | Camera user data (`cam_user`/`nuser_cam`) — MuJoCo supports per-camera user data (8th element type for `user` attribute). CortenForge Phase 7 Spec C implemented 7 element types (body, geom, joint, site, tendon, actuator, sensor). Camera is the 8th, deferred because the umbrella only listed 7 types. Requires: `user: Vec<f64>` on camera MJCF type, `cam_user: Vec<Vec<f64>>` + `nuser_cam: i32` on Model, parser + builder wiring. | Low | T1 |
