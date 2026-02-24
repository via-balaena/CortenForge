# Future Work 10i — Deferred Item Tracker: Group 8 — Flex / Deformable Body

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 8 — Flex / Deformable Body (14 items)

**Spec approach:** DT-66/67/70/73 each need individual specs (T3 — new constraint
types, GPU architecture, or new collision pairs). DT-69 shares a "Flex Collision
Improvements" spec (T2) — DT-70 may fold in if scoped down, otherwise stays T3.
DT-87/88 share a "Flexcomp Completeness" spec (T2). The rest
(DT-68/71/72/85/86/89/90) implement directly (T1). Totals: 7 T1, 3 T2, 4 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-66 | §6B | `<equality><flex>` constraints — flex-flex coupling via equality constraints | Low | T3 |
| DT-67 | §6B | GPU flex pipeline — GPU acceleration of flex constraint solve (Phase 3E) | Low | T3 |
| DT-68 | §6B | Per-vertex material variation — all vertices share single material | Low | T1 |
| DT-69 | §6B | SAP integration for flex broadphase — currently brute-force O(V*G) | Low | T2 |
| DT-70 | §11 | Deformable-vs-Mesh/Hfield/SDF narrowphase — only primitives supported | Medium | T3 |
| DT-71 | §11 | Behavioral friction tests for deformable — deferred until DT-25 lands | Low | T1 |
| DT-72 | §36 | Flex contacts not wired for adhesion — AC12 test documented skip | Low | T1 |
| DT-73 | §6B | Volume constraints — no MuJoCo equivalent; emergent from continuum model | Low | T3 |
| DT-85 | §27B | Flex `<contact>` runtime attributes not wired: `internal`, `activelayers`, `vertcollide`, `passive` | Low | T1 |
| DT-86 | §27B | `elastic2d` keyword on `<flex><elasticity>` — model selection `[none, bend, stretch, both]` | Low | T1 |
| DT-87 | §27D | Shared-body flex vertices — multiple vertices referencing same body's DOFs not implemented | Low | T2 |
| DT-88 | §27E | `<flexcomp>` deferred physics attributes: `inertiabox`, `scale`, `quat`, `file` | Low | T2 |
| DT-89 | §27E | `<flexcomp>` deferred rendering attributes: `flatskin`, `material`, `rgba` | Low | T1 |
| DT-90 | §27E/§30 | `flex_friction` scalar → `Vector3<f64>` — torsional/rolling friction data lost | Low | T1 |
