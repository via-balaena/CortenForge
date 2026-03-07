# Future Work 10i — Deferred Item Tracker: Group 8 — Flex / Deformable Body

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 8 — Flex / Deformable Body (17 items)

**Spec approach:** DT-66/67/70/73 each need individual specs (T3 — new constraint
types, GPU architecture, or new collision pairs). DT-69 shares a "Flex Collision
Improvements" spec (T2) — DT-70 may fold in if scoped down, otherwise stays T3.
DT-87/88 share a "Flexcomp Completeness" spec (T2). The rest
(DT-68/71/72/85/86/89/90/148) implement directly (T1). Totals: 8 T1, 3 T2, 4 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-66 | §6B | `<equality><flex>` constraints — flex-flex coupling via equality constraints | Low | T3 |
| DT-67 | §6B | GPU flex pipeline — GPU acceleration of flex constraint solve (Phase 3E) | Low | T3 |
| DT-68 | §6B | Per-vertex material variation — all vertices share single material | Low | T1 |
| DT-69 | §6B | SAP integration for flex broadphase — currently brute-force O(V*G) | Low | T2 |
| DT-70 | §11 | Deformable-vs-Mesh/Hfield/SDF narrowphase — only primitives supported | Medium | T3 |
| DT-71 | §11 | Behavioral friction tests for deformable — DT-25 verification pass landed (Phase 8 Session 13, 7 tests). Additional behavioral coverage (friction coefficient sweep, stick-slip transition) remains. | Low | T1 |
| DT-72 | §36 | Flex contacts not wired for adhesion — AC12 test documented skip | Low | T1 |
| DT-73 | §6B | Volume constraints — no MuJoCo equivalent; emergent from continuum model | Low | T3 |
| ~~DT-85~~ | §27B | ~~Flex `<contact>` runtime attributes not wired: `internal`, `activelayers`, `vertcollide`, `passive`~~ **DONE** — Phase 7 T1 (commit `cf76731`). Parse + store + wire to Model arrays. | Low | T1 |
| DT-86 | §27B | `elastic2d` keyword on `<flex><elasticity>` — model selection `[none, bend, stretch, both]` | Low | T1 |
| DT-87 | §27D | Shared-body flex vertices — multiple vertices referencing same body's DOFs not implemented | Low | T2 |
| ~~DT-88~~ | §27E | ~~`<flexcomp>` deferred physics attributes: `inertiabox`, `scale`, `quat`, `file`~~ **DONE** — Phase 7 Spec C (commit `05ee0a5`). Documentation-fidelity only (not in MuJoCo 3.5.0 binary). | Low | T2 |
| DT-89 | §27E | `<flexcomp>` deferred rendering attributes: `flatskin`, `material`, `rgba` | Low | T1 |
| ~~DT-90~~ | §27E/§30 | ~~`flex_friction` scalar → `Vector3<f64>` — torsional/rolling friction data lost~~ **DONE** | Low | T1 |
| DT-146 | §42A-i | Sparse constraint assembly — MuJoCo's `mj_addConstraint()` accepts sparse J rows natively; CortenForge scatters into dense `efc_J` (DMatrix). Converting to sparse assembly is a performance optimization, not a conformance gap. Deferred from Phase 10 Spec A. | Low | T2 |
| DT-147 | §42A-i | `flex_edgeequality` dedicated flag — MuJoCo has a per-flex boolean flag indicating whether any equality constraint references this flex's edges; used in `skipjacobian` condition. CortenForge uses `flex_edge_solref != [0,0]` as a conservative proxy (may compute J for edges that don't need it, but never misses). Adding a dedicated flag is a minor optimization. Deferred from Phase 10 Spec A. | Low | T1 |
| DT-148 | §42B | `flex_hingeadr`/`flex_hingenum` per-flex hinge index arrays — Bridson bending path currently iterates all hinges and filters by `flexhinge_flexid[h] != flex_id`. Adding per-flex start/count arrays enables O(1) iteration. Pure performance optimization, no conformance impact. Deferred from Phase 10 Spec B. | Low | T1 |
