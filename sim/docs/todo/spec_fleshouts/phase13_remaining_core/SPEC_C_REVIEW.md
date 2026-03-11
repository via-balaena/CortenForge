# Composite Body Generation (§46) — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_C.md`
**Implementation session(s):** Session 10 (Sessions 10+11 merged due to scope correction)
**Reviewer:** AI agent
**Date:** 2026-03-11

---

## Purpose

This review verifies that the implementation matches the approved spec,
surfaces weakly implemented items that should be fixed now, and ensures
deferred work is properly tracked so nothing falls through the cracks.

**Five questions this review answers:**

1. **Closed?** Are the conformance gaps from the spec's Key Behaviors
   table actually closed?
2. **Faithful?** Does the implementation match the spec — every section,
   every AC, every convention note, every planned test?
3. **Predicted?** Did the blast radius match the spec's predictions, or
   were there surprises?
4. **Solid?** Are there any items that technically "work" but are weakly
   implemented (hacks, TODOs, incomplete edge cases, loose tolerances)?
5. **Tracked?** Is every piece of deferred or out-of-scope work tracked
   in `sim/docs/todo/` or `sim/docs/ROADMAP_V1.md`?

---

## 1. Key Behaviors Gap Closure

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| `<composite type="cable">` parsing | Parsed in `mjXReader`, generates bodies via `mjCComposite::Make()` | Silently skipped via `skip_element()` | | |
| Deprecated composite types | Return specific error messages per type | Silently skipped | | |
| Cable body chain | N-1 bodies in linear parent chain | Not implemented | | |
| Bishop frame propagation | `mjuu_updateFrame()` computes rotation-minimizing frame | Not implemented | | |
| Cable vertex generation | 4 curve shapes (LINE/COS/SIN/ZERO) with quat rotation | Not implemented | | |
| Contact excludes for adjacent bodies | N-2 exclude pairs auto-generated | Not implemented | | |
| Custom text metadata | `composite_{prefix}` text element added | Not implemented | | |

**Unclosed gaps:**
{To be filled during review execution}

---

## 2. Spec Section Compliance

### S1. Types — Composite data structures

**Grade:**

**Spec says:**
Define `CompositeType`, `CompositeShape`, `CompositeJoint`, `CompositeGeom`,
and `MjcfComposite` types in `sim/L0/mjcf/src/types.rs`. Add
`composites: Vec<MjcfComposite>` field to `MjcfBody`. Only fields needed for
cable generation included; deprecated-type-only fields omitted.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Parser — Parse `<composite>` element

**Grade:**

**Spec says:**
Add `parse_composite()` function in `sim/L0/mjcf/src/parser.rs` called from
`parse_body()` and `parse_worldbody()` when `<composite>` is encountered.
Parse all attributes (prefix, type, count, offset, quat, initial, curve, size,
vertex) and child elements (`<joint kind="main">`, `<geom>`, skip site/skin/plugin).
Replace `skip_element()` behavior. Include `parse_curve_shapes()` helper.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Cable expansion — Core generation logic

**Grade:**

**Spec says:**
New file `sim/L0/mjcf/src/builder/composite.rs` with `expand_composites()`,
`make_cable()`, `validate_cable()`, `generate_vertices()`, `update_frame()`,
and helpers (`build_cable_geom()`, `build_cable_joint()`). Follows
`builder/frame.rs` pattern. Returns excludes for insertion into
`MjcfModel.contact.excludes`. Cable generation: vertex gen from 4 curve shapes,
discrete Bishop frame propagation via `update_frame()`, body chain with naming
convention (B_first/B_{ix}/B_last), geom placement (fromto for capsule/cylinder,
pos+size for box), joint logic (ball default, free/none on first), sites on
first+last, excludes for adjacent pairs. Chain is linear nested hierarchy.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Pipeline integration

**Grade:**

**Spec says:**
In `sim/L0/mjcf/src/builder/mod.rs`: call `expand_composites()` after
`expand_frames()` and before `apply_discardvisual()`/`apply_fusestatic()`.
Add `mod composite;` module registration.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Cable body count: nbody=5, njnt=4, ngeom=4 for count=5 cable | T1 | | |
| AC2 | Cable body naming: B_first, B_1, B_2, B_last | T1 | | |
| AC3 | Cable body chain: parent chain B_first→world, B_1→B_first, etc. | T1 | | |
| AC4 | Cable initial="none": njnt=2, no joint on B_first | T2 | | |
| AC5 | Cable initial="free": njnt=3, first joint free with damping=0 | T3 | | |
| AC6 | Deprecated type errors: particle/grid/rope/loop/cloth return correct messages | T4 | | |
| AC7 | Cable with prefix: RB_first, RB_last, RJ_first, etc. | T5 | | |
| AC8 | Contact excludes: 3 exclude pairs for count=5 cable | T1 | | |
| AC9 | Boundary sites: S_first at [0,0,0], S_last at [edge_length,0,0] | T1 | | |
| AC10 | Geom placement: capsule fromto=[0,0,0, edge_length,0,0] | T1, T10 | | |
| AC11 | Pipeline ordering: expand_composites() after expand_frames(), before discardvisual | — (code review) | | |
| AC12 | Nested body composite: B_first is child of parent body | T6 | | |
| AC13 | Count=2 minimum: 1 body, 1 joint, 1 geom, 0 excludes, 2 sites (both on same body) | T7 | | |
| AC14 | Invalid geom type: sphere → error | T8 | | |
| AC15 | Uservert cable: 3 vertices → 2 bodies, positions from vertices | T11 | | |
| AC16 | Uservert/count mutual exclusion: both specified → error | T12 | | |
| AC17 | Count validation: count=1 → error, count=[5,5,1] → error | T12 | | |
| AC18 | Invalid initial value: initial="xyz" → error | T12 | | |

**Missing or failing ACs:**
{To be filled during review execution}

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Cable basic generation (count=5, curve, capsule) — body count, naming, chain, excludes, sites, geom placement | | | AC1, AC2, AC3, AC8, AC9, AC10 |
| T2 | Cable initial="none" — no joint on first body, njnt=2 | | | AC4 |
| T3 | Cable initial="free" — free joint with damping=0, njnt=3 | | | AC5 |
| T4 | Deprecated type errors — particle/grid/rope/loop/cloth error messages | | | AC6 |
| T5 | Cable with prefix="R" — all names prefixed | | | AC7 |
| T6 | Cable in nested body — B_first parent is body, not world | | | AC12 |
| T7 | Cable minimum count=2 — 1 body, dual sites, 0 excludes | | | AC13 |
| T8 | Invalid geom type — sphere → error | | | AC14 |
| T9 | Multiple composites — two cables (A and B) coexist | | | Supplementary |
| T10 | Cable with cylinder geom — cylinder fromto placement | | | AC10 |
| T11 | Cable with user-specified vertices — positions from vertices | | | AC15 |
| T12 | Validation error cases — count=1, count=[5,5,1], uservert+count, invalid initial, zero size | | | AC16, AC17, AC18 |

### Supplementary Tests

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T9 | Two cables with different prefixes in same model — no name collision | | |
| T11 | Uservert cable — exercises generate_vertices() uservert branch | | |
| T12 | Validation errors — exercises all error paths in validate_cable() | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| count=2 (minimum cable) | Single body with dual sites, no excludes | | | T7 |
| count=1 (too few) | Must error: need at least 2 vertices | | | T12 |
| count=[5,5,1] (multi-dim) | Must error: cable is 1D only | | | T12 |
| initial="none" (pinned) | No joint on first body | | | T2 |
| initial="free" (6-DOF root) | Free joint with zero damping | | | T3 |
| initial="xyz" (invalid) | Must error: unknown initial value | | | T12 |
| Deprecated types (5) | Must match MuJoCo error messages | | | T4 |
| Prefix naming | All elements get prefix | | | T5 |
| Nested body parent | Composite inside body, not worldbody | | | T6 |
| Invalid geom type | sphere/ellipsoid rejected | | | T8 |
| Multiple composites | Two cables with different prefixes coexist | | | T9 |
| Uservert cable | User-specified vertices instead of curve | | | T11 |
| Uservert + count conflict | Both specified → error | | | T12 |
| Zero size (no uservert) | Degenerate cable → error | | | T12 |

**Missing tests:**
{To be filled during review execution}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `<composite>` parsing: silently skipped → parsed and expanded for cable; error for deprecated types | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/mjcf/src/types.rs` — add composite types + `composites` field on MjcfBody (+120 lines) | | |
| `sim/L0/mjcf/src/parser.rs` — add `parse_composite()`, `parse_curve_shapes()`, call from parse_body/parse_worldbody (+130 lines) | | |
| `sim/L0/mjcf/src/builder/composite.rs` — new file: expand_composites, make_cable, validate_cable, generate_vertices, update_frame, helpers (+400 lines) | | |
| `sim/L0/mjcf/src/builder/mod.rs` — add `mod composite;` and expand_composites() call (+5 lines) | | |
| `sim/L0/mjcf/src/lib.rs` — update module exports, remove "composite not supported" doc note (~5 lines) | | |
| `sim/L0/mjcf/tests/` or `sim/L0/tests/` — new test file (+300 lines) | | |

{Unexpected files:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All existing 2,273+ tests | Pass (unchanged) — models without composite unaffected | | |
| `composite_model` conformance test | Pass (unchanged) — named "composite" but uses explicit bodies, not `<composite>` element | | |

**Unexpected regressions:**
{To be filled during review execution}

### Non-Modification Sites: Predicted vs Actual

| Predicted Non-Modified File | What it does | Why NOT modified (from spec) | Actually Untouched? | Notes |
|-----------------------------|-------------|-----------------------------|--------------------|-------|
| `builder/body.rs` | Body tree processing | Sees expanded bodies — no composite awareness needed | | |
| `builder/joint.rs` | Joint processing | Sees expanded joints — no change needed | | |
| `builder/geom.rs` | Geom processing | Sees expanded geoms — no change needed | | |
| `builder/contact.rs` | Exclude processing | Already handles `MjcfContactExclude` from parser — composite excludes use same path | | |

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Quaternion — `[w, x, y, z]` | Direct port — same layout | | |
| Body position — `body->pos` → `MjcfBody.pos: Vector3<f64>` | Direct port | | |
| Body orientation — `body->quat` → `MjcfBody.quat: Vector4<f64>` | Direct port | | |
| Geom fromto — 6-element array → `Option<[f64; 6]>` | Use `Some([...])` | | |
| Exclude — `mjsExclude` with body names → `MjcfContactExclude { body1, body2 }` | Map body names directly | | |
| Custom text — `mjsText` → not supported | Out of scope (DT-166) — metadata-only, no physics impact | | |
| Geom defaults — `def[0].spec` applied via `mjs_addGeom` | Copy parsed `<geom>` child attributes to each generated MjcfGeom | | |
| Joint defaults — `defjoint[JOINT][0].spec` | Copy parsed `<joint kind="main">` template to each generated MjcfJoint | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

{To be filled during review execution}

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Skin generation — rendering-only box-geom cable skin with bicubic interpolation | Out of Scope, bullet 1 | | DT-165 | |
| Plugin support — `mujoco.elasticity.cable` requires Spec D | Out of Scope, bullet 2 | | Spec D | |
| `<flexcomp>` element — separate infrastructure from `<composite>` | Out of Scope, bullet 3 | | | |
| `<replicate>` element — replaces `particle` composite | Out of Scope, bullet 4 | | | |
| Custom text generation — `composite_{prefix}` text metadata | Out of Scope, bullet 5 | | DT-166 | |
| Cable `<site>` template — `<site>` child property customization | Out of Scope, bullet 6 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution}

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution}

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

{To be filled during review execution}

---

## 9. Test Coverage Summary

**Domain test results:**
```
{To be filled during review execution}
```

**New tests added:**
**Tests modified:**
**Pre-existing test regressions:**

**Clippy:**
**Fmt:**

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | |
| Spec section compliance | 2 | |
| Acceptance criteria | 3 | |
| Test plan completeness | 4 | |
| Blast radius accuracy | 5 | |
| Convention fidelity | 6 | |
| Weak items | 7 | |
| Deferred work tracking | 8 | |
| Test health | 9 | |

**Overall:**

**Items fixed during review:**

**Items to fix before shipping:**

**Items tracked for future work:**
