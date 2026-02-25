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
| [future_work_10b.md](./future_work_10b.md) | 1. Defaults & MJCF Parsing Gaps | DT-1 – DT-17 | 17 | 10 | 7 | 0 |
| [future_work_10c.md](./future_work_10c.md) | 2. Contact & Collision System | DT-18 – DT-27, ~~DT-94~~, ~~DT-95~~ (subsumed by §41), DT-99, DT-100, DT-101 | 15 | 2 | 8 | 5 |
| [future_work_10d.md](./future_work_10d.md) | 3. Tendon System | DT-28 – DT-35 | 8 | 2 | 4 | 2 |
| [future_work_10e.md](./future_work_10e.md) | 4. Solver Optimizations | DT-36 – DT-44 | 9 | 1 | 5 | 3 |
| [future_work_10f.md](./future_work_10f.md) | 5. Derivatives & Analytical Methods | DT-45 – DT-55 | 11 | 1 | 7 | 3 |
| [future_work_10g.md](./future_work_10g.md) | 6. Actuator & Dynamics | DT-56 – DT-61 | 6 | 2 | 3 | 1 |
| [future_work_10h.md](./future_work_10h.md) | 7. Sensor Gaps | DT-62 – DT-65 | 4 | 1 | 3 | 0 |
| [future_work_10i.md](./future_work_10i.md) | 8. Flex / Deformable Body | DT-66 – DT-73, DT-85 – DT-90 | 14 | 7 | 3 | 4 |
| [future_work_10j.md](./future_work_10j.md) | 9. Misc Pipeline & API | DT-74 – DT-84, DT-91 – DT-92, ~~DT-93~~ (subsumed by §41), DT-96 – DT-98 | 16 | 6 | 4 | 6 |
| **Total** | | | **100** | **32** | **45** | **23** |

**Priority breakdown:** 24 Medium, 71 Low. No High — these are all sub-items
within completed tasks, not critical gaps.

**Tier key:**
- **T1** (plan + implement): Mechanical — parent spec already defines the "what."
  No iterative spec needed. 32 items.
- **T2** (grouped spec): Related items share one spec covering shared design
  decisions. Each item gets a "Step N" section. 42 items → ~15 spec groups.
- **T3** (individual spec): Algorithmic complexity, multiple valid approaches, or
  architectural decisions needing dedicated design. 23 items.

**T2 spec groups (cross-file):**
1. "Defaults Completeness" — DT-2, DT-11, DT-13, DT-14
2. "Actuator Attr Completeness" — DT-5, DT-8, DT-9
3. "Contact Force Cleanup" — DT-20, DT-24
4. "Solver Param Completeness" — DT-23, DT-32, DT-33
5. "XPBD Improvements" — DT-26, DT-27
6. "Tendon Joint Type Completeness" — DT-28, DT-31
7. "Sparse Storage" — DT-36, DT-37, DT-44, DT-48
8. "Solver Robustness" — DT-39, DT-40
9. "Derivative Extensions" — DT-47, DT-51, DT-52, DT-54
10. "FD Performance" — DT-49, DT-53
11. "acc0 / dampratio" — DT-56, DT-57
12. "Length-Range Estimation" — DT-59, DT-77, DT-78
13. "Sensor Completeness" — DT-62, DT-63, DT-64
14. "Flex Collision Improvements" — DT-69 (+ DT-70 if not T3)
15. "Flexcomp Completeness" — DT-87, DT-88

---

## Group 1 — Defaults & MJCF Parsing Gaps (17 items)

**Spec approach:** T1 items implement directly; T2 items (DT-2/11/13/14) share one
"Defaults Completeness" spec; DT-5/8/9 share an "Actuator Attr Completeness" spec.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-1 | §1 | Mesh defaults — no `apply_to_mesh()` method; root-only mesh scale defaults deferred | Low | T1 |
| DT-2 | §1 | Equality constraint defaults — no `apply_to_equality()`, `solref`/`solimp` not in defaults structs | Medium | T2 |
| DT-3 | §6a | File-based hfield loading from PNG (`<hfield file="terrain.png"/>`) | Low | T1 |
| DT-4 | §6b | `<sdf>` asset element for inline distance grids (no standard MuJoCo equivalent) | Low | T1 |
| DT-5 | §8 | `gaintype/biastype/dyntype="user"` — callback-based types require plugin system | Low | T2 |
| DT-6 | §8 | `actearly` parsed + defaultable but not wired to runtime (always standard order) | Medium | T1 |
| DT-7 | §8 | `actdim` explicit override not supported (auto-detection only) | Low | T1 |
| DT-8 | §8 | Transmission types: `cranksite`, `slidersite`, `jointinparent` not supported | Low | T2 |
| DT-9 | §8 | `nsample`, `interp`, `delay` — MuJoCo 3.x interpolation actuator attributes | Low | T2 |
| DT-10 | §18 | Deferred `<compiler>` attributes: `fitaabb`, `usethread`, `alignfree`, `saveinertial`, `inertiagrouprange`, `<lengthrange>` child | Low | T1 |
| DT-11 | §20 | `range` not in `MjcfJointDefaults` as a defaultable attribute | Medium | T2 |
| DT-12 | §20 | Programmatic API enforcement that `worldbody.childclass` must be `None` | Low | T1 |
| DT-13 | §22 | `qpos_spring` not implemented — uses `qpos0` instead (equivalent only in default case) | Medium | T2 |
| DT-14 | §27 defaults | Actuator type-specific defaults not yet defaultable (cylinder area/timeconst, muscle params) | Medium | T2 |
| DT-15 | §27 defaults | Sentinel-value detection for `gear`/`kp`/`noise`/`cutoff` should migrate to `Option<T>` | Low | T1 |
| ~~DT-16~~ | §27B | ~~Flex `density` attribute location wrong — on `<flex>` in parser but not on `<flex>` in MuJoCo~~ **DONE** | Medium | T1 |
| DT-17 | §27 | Global `<option o_margin>` override deferred — per-geom margin is correct foundation | Low | T1 |
