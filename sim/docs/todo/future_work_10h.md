# Future Work 10h — Deferred Item Tracker: Group 7 — Sensor Gaps ✅

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 7 — Sensor Gaps (7 items) — All Core Items Done

> **Complete (2026-03-01).** All 6 core items (DT-62/63/64/102/103/109) done
> across Phase 4 (DT-103) and Phase 6 (DT-62/63/64/102/109). DT-65 deferred
> to post-v1.0. Phase 6 also discovered 5 new deferred items (DT-118–122)
> tracked in `future_work_15.md` and `ROADMAP_V1.md`.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| ~~DT-62~~ | §6 | ~~Frame sensor `objtype` attribute not parsed — fallback tries site → body → geom~~ **Done** — Phase 6 Spec A. Explicit `objtype` parsing + dispatch. | Medium | T2 |
| ~~DT-63~~ | §6 | ~~Frame sensor `reftype`/`refid` not wired — relative-frame measurements~~ **Done** — Phase 6 Spec B. Reference-frame transforms in all 9 frame sensor arms. | Low | T2 |
| ~~DT-64~~ | §6 | ~~Multi-geom Touch bodies — only first geom on body matched~~ **Done** — Phase 6 Spec A. Body-level aggregation matching MuJoCo. | Medium | T2 |
| DT-65 | §6 | User sensor `dim` attribute not parsed — `User.dim()` returns 0. Also requires `sensor_intprm` array (`mjmodel.h:1213`, `nsensor × mjNSENS=3`) for user/plugin sensor integer params (separate from `sensor_history`; see Phase 6 Spec D rubric EGT-8). **Deferred to post-v1.0.** | Low | T1 |
| ~~DT-102~~ | 4A.6 | ~~Geom-attached acc-stage sensors — FrameLinAcc/FrameAngAcc with `MjObjectType::Geom` return zeros.~~ **Done** — Phase 6 Spec A. Full `mj_objectAcceleration()` at geom position. | Low | T1 |
| ~~DT-103~~ | 4A.6 | ~~Extract `mj_objectAcceleration()` helper — the spatial-transform + Coriolis pattern.~~ **Done** — Phase 4 (commit `29501df`). 5 transport helpers in `dynamics/spatial.rs`, 6 sensor arms rewritten. Review: 6/6 A. | Low | T1 |
| ~~DT-109~~ | Spec D | ~~Sensor history attributes — `mjsSensor_` has identical `nsample`/`interp`/`delay`/`interval` attributes and model arrays. Sensor history contributes to `nhistory`.~~ **Done** — Phase 6 Spec D. 5 model fields, historyadr computation, validation. Runtime → DT-107. | Low | T2 |

### Deferred Items Discovered During Phase 6

These are tracked in `future_work_15.md` (full descriptions) and `ROADMAP_V1.md` (post-v1.0 Low-Priority MuJoCo Compat):

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-118 | Phase 6 Spec A | `mj_contactForce()` — touch sensor force reconstruction via full contact force vector | Low |
| DT-119 | Phase 6 Spec A | Ray-geom intersection filter for touch sensor. Depends on DT-118. | Low |
| DT-120 | Phase 6 Spec B | `MjObjectType::Camera` — frame sensor camera support (reftype="camera" currently warns + ignores) | Low |
| DT-121 | Phase 6 Spec C | `InsideSite` sensor (`mjSENS_INSIDESITE`) — MuJoCo 3.x sensor type not yet supported | Low |
| DT-122 | Phase 6 Spec C | Mesh/Hfield/SDF geom distance support for `GeomDist`/`GeomPoint`/`GeomNormal` sensors | Low |
