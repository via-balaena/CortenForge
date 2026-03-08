# Spec D — Flex-Flex Cross-Object Collision (§42A-v / DT-143): Quality Rubric

Grades the Spec D spec on 10 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

Verified against MuJoCo 3.5.0 empirical tests. The umbrella's Spec D scope
statement is accurate with the following clarifications:

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| Broadphase via `canCollide2()` bodyflex index space | Confirmed. `canCollide2()` uses unified bodyflex index space `[0, nbody+nflex)` with standard `filterBitmask()` protocol. | In scope |
| Element-element narrowphase reuses Spec C primitives | Confirmed. Triangle-triangle contacts observed for dim=2 flex-flex pairs. However, Spec C's `collide_element_pair()` takes a single `flex_id` — adaptation needed for cross-object use. | In scope (with narrowphase adaptation) |
| Contact parameter combination via `contact_param_flex_flex()` | Confirmed. Priority + solmix protocol identical to flex-rigid. Verified: priority winner takes all, equal priority uses solmix-weighted solref/solimp + element-wise max friction. | In scope |
| `flex_rigid` gates flex-flex collision | **Not true.** `flex_rigid` only gates self-collision. MuJoCo produces 32 flex-flex contacts when flex 1 is fully rigid (`flex_rigid[0]=1`). | Correct: `flex_rigid` is NOT a gate for flex-flex |
| BVH acceleration for cross-body pair pruning | MuJoCo docs cite BVH per flex object. Not directly empirically verifiable from Python API, but structurally sound. | In scope |
| 3-session compressed cycle | Session plan was updated to full 5-session cycle (rubric → spec → implement → review-create → review-execute). | Adjust: 5 sessions, not 3 |

**Final scope:**
1. Broadphase: `filterBitmask()` on `flex_contype`/`flex_conaffinity` for all `(f1, f2)` pairs where `f1 < f2`
2. Contact parameter combination: `contact_param_flex_flex()` — priority + solmix protocol reading `flex_*` arrays for both sides
3. Contact factory: `make_contact_flex_flex()` — sentinel geoms, dual flex vertex tracking
4. Narrowphase adaptation: generalize Spec C's element-element primitives for cross-object use (two different flex IDs, additive margin/gap from both)
5. Midphase: BVH-based element pair pruning between two flex objects
6. Pipeline integration: dispatch loop after flex-self in `mj_collision()`
7. Constraint Jacobian: verify `compute_flex_self_contact_jacobian()` works for flex-flex (DOFs from two different flex objects)

---

## Empirical Ground Truth

All tests run against MuJoCo 3.5.0 via Python `mujoco` package. Model
configuration: two overlapping `flexcomp type="grid" count="3 3 1"` (9
vertices, 8 triangles per flex, dim=2) with `spacing="0.1 0.1 0.1"`,
`radius=0.02`, `mass=0.1`.

### Codebase context

| Component | File | Lines | Notes |
|-----------|------|-------|-------|
| Collision pipeline dispatch | `collision/mod.rs` | 376–567 | `mj_collision()` — add flex-flex after line 566 |
| Flex-rigid dispatch | `collision/mod.rs` | 569–642 | `mj_collision_flex()` — pattern to follow |
| Flex self-collision dispatch | `collision/mod.rs` | 644–692 | `mj_collision_flex_self()` — pattern to follow |
| Contact param (geom-geom) | `collision/mod.rs` | 117–170 | `contact_param()` — reference for param combination |
| Contact param (flex-rigid) | `collision/mod.rs` | 177–229 | `contact_param_flex_rigid()` — direct analog |
| Contact param (flex-self) | `collision/mod.rs` | 237–249 | `contact_param_flex_self()` — simpler case |
| Solmix/combine helpers | `collision/mod.rs` | 254–286 | `solmix_weight()`, `combine_solref()`, `combine_solimp()` |
| S10 override helpers | `collision/mod.rs` | 288–346 | `assign_margin/ref/imp/friction()` |
| Contact factory (flex-rigid) | `collision/flex_collide.rs` | 250–303 | `make_contact_flex_rigid()` — pattern |
| Contact factory (flex-self) | `collision/flex_collide.rs` | 310–351 | `make_contact_flex_self()` — closest pattern |
| Element-element dispatch | `collision/flex_collide.rs` | 650–667 | `collide_element_pair()` — takes single `flex_id` |
| Triangle-triangle narrow | `collision/flex_collide.rs` | 673–714 | `collide_triangles()` — hardcodes `make_contact_flex_self()` |
| Tet-tet narrow | `collision/flex_collide.rs` | 720–735 | `collide_tetrahedra()` — hardcodes `make_contact_flex_self()` |
| Edge-edge narrow | `collision/flex_collide.rs` | 740–788 | `collide_edges()` — hardcodes `make_contact_flex_self()` |
| Adjacency check | `collision/flex_collide.rs` | 633–645 | `elements_adjacent()` — NOT needed for cross-object |
| Midphase BVH | `collision/flex_collide.rs` | 845+ | Self-collision BVH — pattern for cross-object |
| Flex self-collision Jacobian | `constraint/jacobian.rs` | 149–192 | `compute_flex_self_contact_jacobian()` |
| Jacobian dispatch | `constraint/jacobian.rs` | 201–212 | `flex_vertex2.is_some()` → self-collision path |
| Assembly bodyweight | `constraint/assembly.rs` | 603–624 | `(Some(vi1), Some(vi2))` → flex vertex bodies |
| Contact struct | `types/contact_types.rs` | ~35–100 | `flex_vertex`, `flex_vertex2`, `geom1/geom2` |
| Model flex fields | `types/model.rs` | 321–423 | `flex_contype`, `flex_conaffinity`, `flex_priority`, etc. |
| FlexSelfCollide enum | `types/enums.rs` | — | `FlexSelfCollide` (None/Narrow/Bvh/Sap/Auto) |
| Parser (flex contact) | `mjcf/src/parser.rs` | — | Parses `contype`, `conaffinity` from `<contact>` |
| Builder (flex) | `mjcf/src/builder/flex.rs` | — | `process_flex_bodies()` |

**Critical adaptation points:**
- `collide_element_pair()` (line 650): accepts single `flex_id`. Cross-object needs two flex IDs with different margin/gap sources.
- `collide_triangles/tetrahedra/edges`: all call `make_contact_flex_self()` internally. Cross-object needs `make_contact_flex_flex()`.
- Constraint Jacobian: `compute_flex_self_contact_jacobian()` uses `flexvert_dofadr[vi1]` and `flexvert_dofadr[vi2]`. For cross-object, vi1 and vi2 come from different flex objects → different DOF addresses. **Should work without modification** since the function only reads DOF addresses, not flex IDs.
- Assembly bodyweight: already handles `(Some(vi1), Some(vi2))` via `flexvert_bodyid`. **Should work without modification.**

**Downstream contact consumers with `geom_body[contact.geom1]` — sentinel-unsafe sites:**

Flex-flex contacts use `geom1 = geom2 = usize::MAX` (sentinel). Several
downstream consumers index `model.geom_body[]` with `contact.geom1` without
bounds-checking. These are **pre-existing latent bugs for flex-self contacts
too** (same sentinel convention) that have been untriggered because Spec C
tests only test contact generation, not full constraint solving. Spec D will
expose them because flex-flex contacts will produce active constraint forces.

| Site | File | Lines | Issue | Severity |
|------|------|-------|-------|----------|
| Force distribution | `forward/acceleration.rs` | 510–511 | **Unguarded** `model.geom_body[contact.geom1]` in `mj_fwd_constraint()` contact force loop. Will PANIC for any contact with sentinel geom. | **CRITICAL** |
| Sleep/wake | `island/sleep.rs` | 557–558 | **Unguarded** `model.geom_body[contact.geom1]` in `mj_wake_contact()`. Will PANIC. | **CRITICAL** |
| Impedance bodyweight | `constraint/impedance.rs` | 489–498 | Guarded (falls back to body 0 = world). Wrong bodyweight for flex contacts — uses `invweight0[0]` = 0 instead of flex vertex masses. | MEDIUM |
| Island construction | `island/mod.rs` | 61–70 | Guarded (`continue` if out of bounds). Flex contacts skipped — no island connectivity between flex objects in contact. | MEDIUM |
| Touch sensor | `sensor/acceleration.rs` | 171–180 | Guarded (falls back to `usize::MAX`). Touch sensors never fire on flex-flex contacts because body match uses geom_body. | MEDIUM |

**Safe by design (verified):**

| Site | File | Lines | Why safe |
|------|------|-------|----------|
| Body transmission | `forward/actuation.rs` | 239 | `flex_vertex.is_some()` skip before geom_body access |
| Assembly bodyweight | `constraint/assembly.rs` | 603 | `(Some(vi1), Some(vi2))` early return before rigid path |
| Jacobian dispatch | `constraint/jacobian.rs` | 207–211 | `flex_vertex2.is_some()` early return for flex path |

### EGT-1: Contact encoding

MuJoCo 3.5.0. Two overlapping dim=2 grids, `pos="0 0 0"` and `pos="0.05 0.05 0"`.

```
contact.geom1 = -1, contact.geom2 = -1   (sentinel: no rigid geom)
contact.flex  = [0, 1]                    (flex indices of both objects)
contact.elem  = [e1, e2]                  (element indices within each flex)
contact.vert  = [-1, -1]                  (no per-vertex attribution)
```

CortenForge mapping:
- `geom1 = geom2 = usize::MAX` (Rust sentinel, matching flex-self convention)
- `flex_vertex = Some(nearest_vertex_from_flex1)` (needed for Jacobian DOF lookup)
- `flex_vertex2 = Some(nearest_vertex_from_flex2)` (needed for Jacobian DOF lookup)
- No separate `flex[]` / `elem[]` fields in Contact struct (flex ID derived from vertex via `flexvert_flexid[]`)

### EGT-2: Bitmask filtering

MuJoCo 3.5.0. `canCollide2()` uses `filterBitmask(ct1, ca1, ct2, ca2)`:
- `(ct1 & ca2) != 0 || (ct2 & ca1) != 0` → can collide
- Incompatible (ct=1/ca=1 vs ct=2/ca=2): **0 contacts** ✓
- Compatible (ct=1/ca=3 vs ct=2/ca=1): **32 contacts** ✓ (`1 & 1 = 1`)

The pair loop enumerates `(f1, f2)` where `f1 < f2` — no duplicate pairs,
no self-pair.

### EGT-3: Contact parameter combination (priority + solmix)

MuJoCo 3.5.0.

**Priority test:** flex1 priority=1, flex2 priority=0:
- Contact solref = `[0.05, 2.0]` (flex1's values) ✓
- Contact friction = `0.5` (flex1's value) ✓
- Winner takes all parameters.

**Solmix test:** flex1 (solmix=0.5, solref=[0.02,1.0], friction=0.8),
flex2 (solmix=1.5, solref=[0.04,2.0], friction=0.2), equal priority:
- mix = 0.5 / (0.5 + 1.5) = 0.25
- Contact solref = `[0.25×0.02 + 0.75×0.04, 0.25×1.0 + 0.75×2.0]` = `[0.035, 1.75]` ✓
- Contact friction = `max(0.8, 0.2)` = `0.8` ✓
- Friction uses element-wise max, NOT solmix-weighted.

### EGT-4: `flex_rigid` does NOT gate flex-flex collision

MuJoCo 3.5.0. flex1 all 9 vertices pinned (`<pin id="0 1 2 3 4 5 6 7 8"/>`),
`flex_rigid[0] = 1`:
- **ncon = 32** — identical to unpinned case.
- `flex_rigid` only gates self-collision (condition 1 in `mj_collision_flex_self()`).
- For flex-flex cross-object collision, no `flex_rigid` check exists.

### EGT-5: Margin/gap combination (additive)

MuJoCo 3.5.0. flex1 margin=0.05/gap=0.01, flex2 margin=0.03/gap=0.02:
- Contact `includemargin = 0.05` ✓
- Matches formula: `(0.05 + 0.03) - (0.01 + 0.02) = 0.05`
- Margin is additive (flex_margin[f1] + flex_margin[f2]).
- Gap is additive (flex_gap[f1] + flex_gap[f2]).

### EGT-6: Multi-flex pair enumeration

MuJoCo 3.5.0. Three flex objects (2×2 grids, overlapping):
- Pair (0,1): 4 contacts
- Pair (0,2): 4 contacts
- Pair (1,2): 4 contacts
- All `f1 < f2` pairs enumerated. No duplicate pairs.

### EGT-7: Condim combination

MuJoCo 3.5.0. flex1 condim=1, flex2 condim=6:
- Contact dim = `max(1, 6) = 6` ✓
- Condim combination follows `max()` rule (same as geom-geom and flex-rigid).

### EGT-8: Element-element contact counts

MuJoCo 3.5.0. Two overlapping 3×3 grids (8 elements each):
- 32 contacts from 64 possible element pairs (8×8).
- Each intersecting element pair produces exactly 1 contact.
- Contacts are element-level (no per-vertex attribution: `vert=[-1,-1]`).

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does for flex-flex collision — exact
> function names, field names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function/field/flag cited with source file, line range, and exact behavior. Specifically: `canCollide2()` unified bodyflex index space from `engine_collision_driver.c`, `filterBitmask()` protocol, `mj_contactParam()` flex-flex path, element-element narrowphase dispatch (triangle-triangle for dim=2, tet-tet for dim=3). Edge cases addressed with MuJoCo behavior: (1) incompatible bitmasks → zero contacts, (2) `flex_rigid` does NOT gate flex-flex, (3) same-flex pair excluded (f1 < f2 loop), (4) mixed flex dimensions, (5) zero-element flex, (6) all vertices pinned on one flex. Contact encoding documented: `geom=-1` sentinel, `flex[0]/flex[1]` = flex indices, `elem[0]/elem[1]` = element indices, `vert=[-1,-1]`. Numerical expectations from EGT-3 and EGT-5 included. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing specifics — or description based on docs rather than C source. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood, assumed, or invented. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source" gaps.

| Grade | Bar |
|-------|-----|
| **A+** | Every step written out: (1) broadphase pair loop `for f1 in 0..nflex, f2 in (f1+1)..nflex`, (2) bitmask filter `(ct1 & ca2) != 0 \|\| (ct2 & ca1) != 0`, (3) element pair enumeration (brute-force or BVH-accelerated), (4) element-element narrowphase dispatch by dim, (5) contact creation with `make_contact_flex_flex()`, (6) `contact_param_flex_flex()` with priority/solmix/max-friction. Margin/gap formula explicit: `margin = flex_margin[f1] + flex_margin[f2]`, `gap = flex_gap[f1] + flex_gap[f2]`. Narrowphase adaptation from Spec C's single-flex-id primitives to cross-object use clearly specified. An implementer can type it in without reading MuJoCo source. |
| **A** | Algorithm is complete. One or two minor details left implicit. |
| **B** | Algorithm structure clear but some steps hand-waved or deferred. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo → CortenForge translation called out: (1) `geom=-1` → `geom1/geom2 = usize::MAX` sentinel, (2) MuJoCo `contact.flex[0]/flex[1]` → CortenForge `flex_vertex/flex_vertex2` (vertex indices, flex ID derived via `flexvert_flexid[]`), (3) MuJoCo `contact.vert=[-1,-1]` → CortenForge stores nearest vertices for Jacobian DOF lookup, (4) margin/gap additive convention (matching flex-self), (5) friction `Vector3` → `[f64; 5]` unpacking `[x,x,y,z,z]`. Convention difference table present with verification that each mapping preserves numerical equivalence. |
| **A** | Major conventions documented. Minor mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable with concrete values.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has the three-part structure: (1) concrete input model, (2) exact expected value or tolerance from MuJoCo (EGT-verified), (3) what field to check. At minimum: (a) bitmask filter AC with incompatible bitmasks → 0 contacts (EGT-2), (b) contact param AC with priority winner → exact solref/friction values (EGT-3), (c) solmix AC with numerical expected values (EGT-3), (d) margin/gap AC with `includemargin` value (EGT-5), (e) `flex_rigid` non-gating AC (EGT-4), (f) multi-flex pair enumeration AC (EGT-6), (g) **sentinel safety AC**: full forward step with active flex-flex constraint forces completes without panic — verifies `mj_fwd_constraint()` force distribution and `mj_wake_contact()` handle sentinel `geom1/geom2 = usize::MAX` correctly. Code-review ACs explicitly labeled — at minimum one per sentinel-unsafe site in the codebase context table. |
| **A** | ACs are testable. Some lack exact MuJoCo-verified values. |
| **B** | ACs directionally correct but vague. |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Explicit edge case inventory: (1) incompatible bitmasks → zero contacts, (2) `flex_rigid[f]=true` on one flex → still collides, (3) single-element flex pair, (4) zero-element flex, (5) same-flex excluded (f1 < f2), (6) three+ flex objects all-pairs, (7) condim mismatch (max), (8) S10 global override applied, (9) **full forward step with active flex-flex constraint forces** (exercises sentinel-guarded downstream sites: force distribution, sleep/wake, island connectivity). Negative cases first-class. At least one MuJoCo conformance test per code path (contact count, param values from EGT). Non-trivial model test (two overlapping 3×3 grids). **Integration test:** full `mj_step()` equivalent with flex-flex contacts producing non-zero constraint forces — verifies no panic in `mj_fwd_constraint()`, `mj_wake_contact()`, island construction. Regression: existing flex-rigid and flex-self tests unaffected. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and cross-spec interactions explicit.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. Hard dependency on Spec C (§42A-iv) stated with commit hash. Spec C narrowphase primitives cited by function name and signature. Adaptation required for cross-object use clearly stated (single `flex_id` → two flex IDs, `make_contact_flex_self` → `make_contact_flex_flex`). No dependency on Spec A or Spec B stated and justified. Constraint Jacobian reuse verified (no modification needed). Assembly bodyweight reuse verified. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. **New additions:** (1) `mj_collide_flex_flex()`, (2) `contact_param_flex_flex()`, (3) `make_contact_flex_flex()`, (4) pipeline dispatch in `mj_collision()`. **Sentinel-unsafe downstream fixes (CRITICAL):** The spec must address all sites that index `model.geom_body[contact.geom1]` without bounds-checking — these will PANIC for flex-flex contacts (`geom1 = usize::MAX`). At minimum: (a) `acceleration.rs:510-511` force distribution loop, (b) `sleep.rs:557-558` wake-contact check. **Wrong-logic fixes:** (c) `impedance.rs:489-498` bodyweight fallback, (d) `island/mod.rs:61-70` island connectivity, (e) `sensor/acceleration.rs:171-180` touch sensor body match. These are pre-existing latent bugs for flex-self (same sentinel) but Spec D will expose them via active constraint forces. Narrowphase adaptation: if `collide_element_pair()` is generalized, verify self-collision callers still work. Existing test impact: flex-rigid/flex-self tests unaffected (different dispatch paths). |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Uniform terminology.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: "flex-flex" (not "cross-object" vs "inter-flex" inconsistently). Cross-references accurate between MuJoCo Reference, Specification sections, ACs, and Test Plan. File lists match between Specification and Files Affected. AC numbers match between AC section and Traceability Matrix. Edge case lists consistent across MuJoCo Reference and Test Plan. Consumer/caller counts match. `flex_id1`/`flex_id2` naming consistent throughout. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for same concept. |
| **C** | Contradictions between sections. |

### P9. Narrowphase Adaptation *(domain-specific)*

> Spec clearly defines how Spec C's single-flex-id narrowphase primitives are
> adapted for cross-object use without code duplication.

**Boundary with P1:** P1 grades whether the MuJoCo reference for flex-flex
narrowphase is correct. P9 grades whether the adaptation from Spec C's
self-collision primitives to cross-object use is well-specified in Rust terms.

| Grade | Bar |
|-------|-----|
| **A+** | Adaptation strategy explicitly chosen and justified: (a) generalize `collide_element_pair()` to accept two flex IDs, or (b) create cross-object variant, or (c) extract contact factory as parameter. Whichever strategy: (1) margin/gap formula change documented (`2*flex_margin[f]` → `flex_margin[f1] + flex_margin[f2]`), (2) contact factory change documented (`make_contact_flex_self` → `make_contact_flex_flex`), (3) adjacency filtering explicitly excluded (not needed for cross-object), (4) self-collision callers verified unbroken. Code structure specified at function-signature level. No code duplication of narrowphase geometry (triangle-triangle, tet-tet, edge-edge). |
| **A** | Adaptation clear. Minor details left to implementer. |
| **B** | Adaptation strategy stated but specifics unclear. Risk of code duplication. |
| **C** | No discussion of how to adapt Spec C's primitives. |

### P10. Pipeline Integration *(domain-specific)*

> Spec correctly places flex-flex collision in the dispatch pipeline with
> correct pair enumeration, no double-counting, and no missed pairs.

**Boundary with P2:** P2 grades whether the algorithm steps are complete.
P10 grades whether the pipeline placement and pair enumeration are correct
in the context of the existing collision dispatch sequence.

| Grade | Bar |
|-------|-----|
| **A+** | Pipeline position specified: after `mj_collision_flex_self()`, before end of `mj_collision()`. Pair enumeration: `for f1 in 0..nflex { for f2 in (f1+1)..nflex { ... } }` — verified against EGT-6 (three-flex all-pairs). No double-counting: `f1 < f2` ensures each pair is tested once. No missed pairs: all flex-flex combinations tested. No interaction with flex-rigid or flex-self paths (independent dispatch). Contact limit (`nconmax`) behavior specified if applicable. S10 global override integration specified (matching flex-self pattern). |
| **A** | Pipeline placement correct. Minor integration details left implicit. |
| **B** | Pipeline placement stated but interaction with existing paths unclear. |
| **C** | No pipeline integration discussion. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific functions (`canCollide2()`,
      `collide_element_pair()`, `contact_param_flex_flex()`), specific edge
      cases (6 enumerated in P1, 8 in P5), and specific EGT references.
      Two independent reviewers would agree on grades by pointing to specific
      spec lines.

- [x] **Non-overlap:** P1 vs P9 boundary: P1 grades MuJoCo reference
      correctness, P9 grades Rust adaptation strategy. P2 vs P10 boundary:
      P2 grades algorithm completeness, P10 grades pipeline placement and
      pair enumeration. No other significant overlaps.

- [x] **Completeness:** 10 criteria cover: MuJoCo reference (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), dependencies (P6),
      blast radius (P7), consistency (P8), narrowphase adaptation (P9),
      pipeline integration (P10). No meaningful gap could be A+ on all 10
      but still have a problem.

- [x] **Gradeability:** P1 → MuJoCo Reference section. P2 → Specification
      (S1..SN). P3 → Convention Notes. P4 → Acceptance Criteria. P5 → Test
      Plan + Traceability Matrix. P6 → Prerequisites + Execution Order.
      P7 → Blast Radius. P8 → cross-cutting. P9 → Specification (narrowphase
      sections). P10 → Specification (pipeline dispatch section).

- [x] **Conformance primacy:** P1 is tailored with `canCollide2()`,
      `filterBitmask()`, `engine_collision_driver.c`, 6 specific edge cases.
      P4 references EGT-verified values (EGT-2 through EGT-7). P5 requires
      MuJoCo conformance tests per code path. Rubric cannot produce A+ spec
      that diverges from MuJoCo.

- [x] **Empirical grounding:** EGT-1 through EGT-8 filled with MuJoCo 3.5.0
      verified values. Contact counts (32), parameter values (solref, friction),
      margin/gap formula, bitmask behavior all verified. Codebase context
      table has file:line references for every adaptation point.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, ...) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Specification (narrowphase adaptation sections), Files Affected |
| P10 | Specification (pipeline dispatch section), Execution Order |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
| P9. Narrowphase Adaptation | | |
| P10. Pipeline Integration | | |

**Overall: — (Rev 2)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P1 | Initial draft assumed `flex_rigid` gates flex-flex collision | EGT-4 empirical test | Corrected: `flex_rigid` does NOT gate flex-flex. Added to edge case inventory and P1 A+ bar. | Rubric Rev 1 |
| R2 | P9 | Rubric initially had no criterion for narrowphase adaptation | Rubric self-audit (completeness check) | Added P9: Narrowphase Adaptation — covers the single-flex-id → two-flex-id adaptation strategy | Rubric Rev 1 |
| R3 | P10 | Rubric initially had no criterion for pipeline dispatch placement | Rubric self-audit (completeness check) | Added P10: Pipeline Integration — covers pair enumeration, dispatch ordering, double-counting prevention | Rubric Rev 1 |
| R4 | Scope | Umbrella stated 3-session compressed cycle; session plan updated to 5 sessions | Session plan reading | Documented in Scope Adjustment table | Rubric Rev 1 |
| R5 | EGT | No empirical verification of condim combination | Rubric self-audit (conformance primacy) | Added EGT-7: condim = max(1, 6) = 6 verified | Rubric Rev 1 |
| R6 | P7 | `acceleration.rs:510-511` — unguarded `geom_body[contact.geom1]` in force distribution loop. Will PANIC for any contact with sentinel `geom1 = usize::MAX`. Pre-existing latent bug for flex-self, newly exposed by flex-flex. | Stress test (downstream consumer audit) | Added to P7 A+ bar as CRITICAL sentinel-unsafe site. Added to codebase context table. | Rubric Rev 2 |
| R7 | P7 | `sleep.rs:557-558` — unguarded `geom_body[contact.geom1]` in `mj_wake_contact()`. Same sentinel panic risk. | Stress test (downstream consumer audit) | Added to P7 A+ bar as CRITICAL sentinel-unsafe site. | Rubric Rev 2 |
| R8 | P7 | `island/mod.rs:61-70` — flex contacts skipped entirely in island construction. Flex-flex contacts won't create island connectivity → incorrect sleep grouping. | Stress test (downstream consumer audit) | Added to P7 A+ bar as wrong-logic site. | Rubric Rev 2 |
| R9 | P7 | `impedance.rs:489-498` — bodyweight falls back to world body (invweight=0) for flex contacts. Same issue exists for flex-self (pre-existing). | Stress test (downstream consumer audit) | Added to P7 A+ bar as wrong-logic site. | Rubric Rev 2 |
| R10 | P7 | `sensor/acceleration.rs:171-180` — touch sensor body match uses `geom_body` which returns `usize::MAX` for sentinel geoms → never matches. | Stress test (downstream consumer audit) | Added to P7 A+ bar as wrong-logic site. | Rubric Rev 2 |
| R11 | P5 | No test requiring full forward step with active flex-flex constraint forces. Without such a test, sentinel panics in downstream consumers go uncaught (as happened with Spec C's flex-self contacts). | Stress test (test coverage gap) | Added integration test requirement to P5 A+ bar. | Rubric Rev 2 |
