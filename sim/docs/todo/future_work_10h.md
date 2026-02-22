# Future Work 10h — Deferred Item Tracker: Group 7 — Sensor Gaps

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 7 — Sensor Gaps (4 items)

**Spec approach:** DT-62/63/64 share a "Sensor Completeness" spec (T2 — all follow
the same parse→wire→test pattern). DT-65 implements directly (T1).

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-62 | §6 | Frame sensor `objtype` attribute not parsed — fallback tries site → body → geom | Medium | T2 |
| DT-63 | §6 | Frame sensor `reftype`/`refid` not wired — relative-frame measurements | Low | T2 |
| DT-64 | §6 | Multi-geom Touch bodies — only first geom on body matched | Medium | T2 |
| DT-65 | §6 | User sensor `dim` attribute not parsed — `User.dim()` returns 0 | Low | T1 |
