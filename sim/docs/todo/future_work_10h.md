# Future Work 10h — Deferred Item Tracker: Group 7 — Sensor Gaps

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 7 — Sensor Gaps (6 items)

**Spec approach:** DT-62/63/64 share a "Sensor Completeness" spec (T2 — all follow
the same parse→wire→test pattern). DT-65 implements directly (T1). DT-102
depends on DT-62 (objtype parsing) and implements directly (T1). DT-103
is a pure refactor triggered when a fourth consumer of the spatial-transform
pattern appears.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-62 | §6 | Frame sensor `objtype` attribute not parsed — fallback tries site → body → geom | Medium | T2 |
| DT-63 | §6 | Frame sensor `reftype`/`refid` not wired — relative-frame measurements | Low | T2 |
| DT-64 | §6 | Multi-geom Touch bodies — only first geom on body matched | Medium | T2 |
| DT-65 | §6 | User sensor `dim` attribute not parsed — `User.dim()` returns 0 | Low | T1 |
| DT-102 | 4A.6 | Geom-attached acc-stage sensors — FrameLinAcc/FrameAngAcc with `MjObjectType::Geom` return zeros. MuJoCo supports `mjOBJ_GEOM` via `mj_objectAcceleration`. Requires `geom_xpos`/`geom_xmat`/`geom_body[objid]`. Depends on DT-62 (objtype parsing). | Low | T1 |
| DT-103 | 4A.6 | Extract `mj_objectAcceleration()` helper — the spatial-transform + Coriolis pattern (cacc shift from `xpos[b]` to point, `omega × v_at_point` correction) is inlined in Accelerometer, FrameLinAcc, and FrameAngAcc sensor arms. Factor out into a shared utility in `sensor/` or `types/spatial.rs` when a fourth consumer appears. | Low | T1 |
