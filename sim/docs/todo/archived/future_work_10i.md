# Future Work 10i — Deferred Item Tracker: Group 8 — Flex / Deformable Body

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 8 — Flex / Deformable Body (24 items)

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
| DT-150 | §42A-iv | `activelayers` runtime filtering for flex self-collision — `flex_activelayers` is parsed and stored (Phase 7 T1) but not consumed at runtime. MuJoCo uses `activelayers` to filter which element layers participate in self-collision. Minimal conformance impact — affects only models using layer-based collision filtering. Deferred from Phase 10 Spec C. | Low | T2 |
| DT-151 | §42A-iv | Edge-edge tests for dim=3 tetrahedral self-collision — MuJoCo's `mj_collideFlexSelf()` performs edge-edge proximity tests between tet edges in addition to vertex-face tests. CortenForge implements vertex-face only; edge-edge contacts are secondary for tetrahedral meshes. Minor conformance gap for dim=3 self-collision models. Deferred from Phase 10 Spec C. | Low | T2 |
| DT-152 | §42A-iv | Barycentric force distribution on face side for flex self-collision contacts — current Jacobian applies force to nearest vertex (`flex_vertex2`) rather than distributing across face vertices via barycentric weights. Force direction is correct; only distribution across face vertices is approximate. Correct for free vertices (all current models). Deferred from Phase 10 Spec C. | Low | T2 |
| DT-153 | §42A-v | Island assignment for flex contacts — `island/mod.rs:297-306` (contact-to-island) and `island/mod.rs:453-467` (constraint-to-tree) use `geom_body[contact.geom*]` with bounds-check fallback. Flex contacts with sentinel `usize::MAX` geom indices are skipped from island assignment and constraint-to-tree lookup. Not a panic, but flex contacts won't be correctly assigned to islands when island-based constraint solving is active. Default single-island mode unaffected. Deferred from Phase 10 Spec D. | Low | T1 |
| DT-154 | §42A-v | Flex contact factory condim=6 mapping — all flex contact factories (`make_contact_flex_self`, `make_contact_flex_rigid`, `make_contact_flex_flex`) map condim via `1→1, 4→4, _→3`. This means `condim=6` produces `dim=3`, not `dim=6`. MuJoCo supports condim=6 for torsional+rolling friction. Pre-existing limitation across all flex contact types. Deferred from Phase 10 Spec D. | Low | T1 |
| DT-155 | §42A-v | S10 override test for flex-flex contacts (AC11/T10) — `ENABLE_OVERRIDE` test infrastructure is not wired for flex contact tests. T10 should verify that global override params (`o_margin`, `o_solref`, `o_solimp`, `o_friction`) replace per-flex params on all flex-flex contacts. The override logic itself works (verified in rigid contact specs); only the test harness is missing. Deferred from Phase 10 Spec D review. | Low | T1 |
| DT-156 | §42A-v | Narrowphase triangle-triangle contact count conformance gap — CortenForge produces 36 contacts for overlapping 3×3 flex grids where MuJoCo 3.5.0 produces 32. Root cause is in triangle-triangle intersection logic (Spec C territory). Force direction and contact behavior are correct; only contact count differs. Identified during Phase 10 Spec D review T2 test strengthening. | Low | T2 |
