# Layer C Trajectory Comparison — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_B.md`
**Implementation session(s):** Session 12
**Reviewer:** AI agent
**Date:** 2026-03-09

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

The spec's Key Behaviors table has a "CortenForge (current)" column showing
the conformance gap *before* implementation. Verify each gap is now closed.

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| Step pipeline | `mj_step()`: forward → checkAcc → advance | `Data::step()`: forward → check_acc → integrate. Same order. | | |
| Default integrator | Euler (`mjINT_EULER`) when no integrator specified | Euler when no integrator in MJCF. Same default. | | |
| Euler integration | Semi-implicit: qvel += dt*qacc; integratePos(qpos, qvel, dt) | Same semi-implicit pattern in `integrate/mod.rs` | | |
| Free joint quaternion | `mj_integratePos()` integrates on SO(3), normalizes after | Same — quaternion integration + normalization in integrate | | |
| qacc after step | From forward pass (pre-advance). Advance does not write qacc. | Same — qacc is from forward, integrate does not overwrite. | | |
| ctrl persistence | Ctrl persists across steps until overwritten | Same — `data.ctrl` is a `DVector<f64>`, persists across `step()` calls | | |
| Timestep | `<option timestep="0.002"/>` for all canonical models | Parsed from MJCF. Same timestep. | | |
| Activation integration | `mj_next_activation()` updates `data.act` per step | Same — activation integrated in `integrate/mod.rs` before velocity update | | |

**Unclosed gaps:**

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it. This is the core of the review.

**How to grade each section:**

- Read the spec section (algorithm, file path, MuJoCo equivalent, design
  decision, before/after code).
- Read the actual implementation.
- Compare. Grade honestly.

| Grade | Meaning |
|-------|---------|
| **Pass** | Implementation matches spec. Algorithm, file location, edge cases all correct. |
| **Weak** | Implementation works but deviates from spec, uses shortcuts, has loose tolerances, missing edge-case guards, or TODOs. **Fix before shipping.** |
| **Deviated** | Implementation intentionally diverged from spec (spec gap discovered during implementation). Deviation is documented and justified. Acceptable if the spec was updated. |
| **Missing** | Section not implemented. Must be either fixed or explicitly deferred with tracking. |

### S1. Trajectory comparison infrastructure

**Grade:**

**Spec says:**
Add `step_tolerance()` function, trajectory tolerance constants
(`TRAJ_BASE_SMOOTH`, `TRAJ_GROWTH_SMOOTH`, `TRAJ_BASE_CHAOTIC`,
`TRAJ_GROWTH_CHAOTIC`, `TRAJ_QACC_FACTOR`), and `TrajectoryDivergence` struct
to `common.rs`. Implement `compare_trajectory()` function that: loads reference
trajectory .npy files, steps CortenForge the same number of times, compares
per-step with step-aware tolerance, collects all divergences in a full sweep,
and panics with diagnostic report (model name, steps matched, first divergence,
worst divergence, total count). Add `mod layer_c;` to `mod.rs`.

**Architecture decisions to verify:**
- **AD-1 (Sweep-then-report):** Chose Option 2 — full sweep, then report
  summary. Must NOT panic on first divergence. Collect all divergences and
  report at end.
- **AD-2 (Per-field tolerance):** Chose Option 2 — per-field base tolerance.
  qacc uses `base_tol * TRAJ_QACC_FACTOR` (wider than qpos/qvel). Growth rate
  is shared across fields.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Non-contact trajectory tests

**Grade:**

**Spec says:**
Two tests: `layer_c_trajectory_pendulum` (pendulum, 100 steps, smooth
tolerance, no ctrl, no free joint) and `layer_c_trajectory_double_pendulum`
(double_pendulum, 100 steps, smooth tolerance, no ctrl, no free joint). Both
expected `#[ignore]`d with comment "CRBA/RNE xipos cascade — Phase 1 FK".
Tests call `compare_trajectory()` with model-specific parameters.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Contact trajectory test

**Grade:**

**Spec says:**
One test: `layer_c_trajectory_contact_scenario` (contact_scenario, 100 steps,
chaotic tolerance, no ctrl, `has_free_joint=true`). Uses sign-aware quaternion
comparison for qpos[3..7]. Expected `#[ignore]`d with comment citing constraint
forces wrong — contact.pos convention + efc_J assembly — Phase 3
collision/constraint.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Actuated trajectory test

**Grade:**

**Spec says:**
One test: `layer_c_trajectory_actuated_system` (actuated_system, 100 steps,
smooth tolerance, ctrl=[1.0, 0.5], no free joint). Ctrl set once before
stepping, persists across all 100 steps. Expected `#[ignore]`d with comment
"CRBA/RNE xipos cascade — Phase 1 FK".

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Subsystem trajectory tests

**Grade:**

**Spec says:**
Three tests: `layer_c_trajectory_tendon_model` (tendon_model, 100 steps,
smooth, no ctrl, no free joint), `layer_c_trajectory_sensor_model`
(sensor_model, 100 steps, smooth, no ctrl, no free joint),
`layer_c_trajectory_equality_model` (equality_model, 100 steps, smooth,
no ctrl, no free joint). All expected `#[ignore]`d — tendon/sensor cite
xipos cascade, equality cites xipos cascade + constraint Jacobian.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Composite trajectory test

**Grade:**

**Spec says:**
One test: `layer_c_trajectory_composite_model` (composite_model, 200 steps,
chaotic tolerance, ctrl=[1.0], no free joint). Longest trajectory in the suite.
Expected `#[ignore]`d with comment citing qacc wrong + constraint wrong — Phase
1 FK + Phase 3 collision/constraint.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Infrastructure compiles — `mod layer_c;` in mod.rs, `common.rs` has `step_tolerance()` + tolerance constants + `TrajectoryDivergence` struct, `layer_c.rs` has `compare_trajectory()` | — (code review) | | |
| AC2 | Pendulum trajectory — 100 steps, smooth tolerance, qpos/qvel/qacc vs reference | `layer_c_trajectory_pendulum` | | |
| AC3 | Double pendulum trajectory — 100 steps, smooth tolerance, qpos/qvel/qacc vs reference | `layer_c_trajectory_double_pendulum` | | |
| AC4 | Contact scenario trajectory — 100 steps, chaotic tolerance, sign-aware quaternion qpos[3..7] | `layer_c_trajectory_contact_scenario` | | |
| AC5 | Actuated + composite trajectory — actuated_system (100 steps, smooth, ctrl=[1.0, 0.5]) AND composite_model (200 steps, chaotic, ctrl=[1.0], full pipeline) | `layer_c_trajectory_actuated_system`, `layer_c_trajectory_composite_model` | | |
| AC6 | Tendon model trajectory — 100 steps, smooth tolerance | `layer_c_trajectory_tendon_model` | | |
| AC7 | Sensor + equality model trajectories — 100 steps each, smooth tolerance | `layer_c_trajectory_sensor_model`, `layer_c_trajectory_equality_model` | | |
| AC8 | Ctrl persistence across steps — ctrl set once, persists for all 100 steps (implicit via AC5) | `layer_c_trajectory_actuated_system` | | |
| AC9 | Free joint quaternion comparison — sign-aware (q ≡ -q) at each step for contact_scenario qpos[3..7] | `layer_c_trajectory_contact_scenario` | | |
| AC10 | Diagnostic output format — model name, steps matched, first divergence (step/field/DOF/expected/actual/diff/tol), worst divergence, total count | — (code review) | | |
| AC11 | All trajectory tests compile and run — `cargo test -p sim-conformance-tests --test mujoco_conformance` passes, all 8 tests `#[ignore]`d, no name conflicts | all tests | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

> **Test numbering note:** Spec uses T1–T8 labels. Implementation uses
> `layer_c_trajectory_{model}` naming convention. Mapping: T1=pendulum,
> T2=double_pendulum, T3=contact_scenario, T4=actuated_system,
> T5=tendon_model, T6=sensor_model, T7=equality_model, T8=composite_model.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Pendulum trajectory: 100 steps, smooth tolerance, no ctrl, no free joint | | `layer_c_trajectory_pendulum` | |
| T2 | Double pendulum trajectory: 100 steps, smooth tolerance, no ctrl, no free joint | | `layer_c_trajectory_double_pendulum` | |
| T3 | Contact scenario trajectory: 100 steps, chaotic tolerance, no ctrl, `has_free_joint=true` | | `layer_c_trajectory_contact_scenario` | |
| T4 | Actuated system trajectory: 100 steps, smooth tolerance, ctrl=[1.0, 0.5], no free joint | | `layer_c_trajectory_actuated_system` | |
| T5 | Tendon model trajectory: 100 steps, smooth tolerance, no ctrl, no free joint | | `layer_c_trajectory_tendon_model` | |
| T6 | Sensor model trajectory: 100 steps, smooth tolerance, no ctrl, no free joint | | `layer_c_trajectory_sensor_model` | |
| T7 | Equality model trajectory: 100 steps, smooth tolerance, no ctrl, no free joint | | `layer_c_trajectory_equality_model` | |
| T8 | Composite model trajectory: 200 steps, chaotic tolerance, ctrl=[1.0], no free joint | | `layer_c_trajectory_composite_model` | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review. These don't map 1:1
to ACs but strengthen the test suite.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Free joint quaternion sign ambiguity (q ≡ -q) | contact_scenario has qpos[3..7] quaternion. Without sign-aware comparison, q and -q (same orientation) would cause false divergence at every step. | | `layer_c_trajectory_contact_scenario` (T3) | |
| Ctrl persistence across 100+ steps | actuated_system and composite_model set ctrl once. If ctrl zeroed between steps, actuator forces disappear after step 1. | | `layer_c_trajectory_actuated_system` (T4), `layer_c_trajectory_composite_model` (T8) | |
| Long trajectory (200 steps) | composite_model runs 200 steps — 2× others. Amplifies accumulation effects and tests tolerance growth at larger step counts. | | `layer_c_trajectory_composite_model` (T8) | |
| qacc includes constraint solver output | qacc is M⁻¹(qfrc_total) including iterative solver forces (~1e-4). qacc needs separate wider tolerance via TRAJ_QACC_FACTOR. | | T3, T7, T8 (models with constraints) | |
| Zero ctrl for unactuated models | 6 of 8 models have nu=0 or leave ctrl at default zero. Empty ctrl slice must not cause index-out-of-bounds. | | T1, T2, T3, T5, T6, T7 | |
| Step-1 vs step-N>1 divergence patterns | 7 models diverge at step 1 (xipos cascade). contact_scenario may diverge at step N>1. Diagnostic must report actual first divergent step. | | All tests | |

**Missing tests:**

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| None — spec adds new tests only, no production code modified | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/tests/mujoco_conformance/layer_c.rs` (new file — 8 tests + `compare_trajectory()`, +250–300 lines) | | |
| `sim/L0/tests/mujoco_conformance/common.rs` (add `step_tolerance()`, tolerance constants, `TrajectoryDivergence` struct, +30–40 lines) | | |
| `sim/L0/tests/mujoco_conformance/mod.rs` (add `mod layer_c;`, +1 line) | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All Layer A tests (13) in `layer_a.rs` | Pass (unchanged) — separate module, no shared mutable state | | |
| All Layer B tests (43) in `layer_b.rs` | Pass (unchanged) — separate module, no shared mutable state | | |
| All Layer D tests (15) in `layer_d.rs` | Pass (unchanged) — separate module, no shared mutable state | | |
| All integration tests (57 modules) | Pass (unchanged) — separate test binary | | |
| `common.rs` additions | N/A — additive, new functions only, no modification to existing | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `qpos` — flat `mjtNum[nq]`, free joint `[x,y,z,w,qx,qy,qz]` | Direct comparison. For free joint quaternion (indices 3..7): use sign-aware comparison (q ≡ -q). Positional indices 0..3: direct comparison. | | |
| `qvel` — flat `mjtNum[nv]`, always length nv | Direct comparison, all DOFs. | | |
| `qacc` — flat `mjtNum[nv]`, post-forward, pre-advance | Direct comparison, all DOFs. | | |
| `ctrl` — flat `mjtNum[nu]`, set once, persists | Direct port — no translation needed. | | |
| Step call — `mj_step(model, data)` mutates data in place | Direct port — `data.step(&model)` mutates data in place. | | |
| Reference file — `{model}_trajectory_{field}.npy`, shape (nsteps, nq/nv) | Access step `s`, DOF `d`: `data[s * ndof + d]` where ndof = nq (qpos) or nv (qvel/qacc). | | |

---

## 7. Weak Implementation Inventory

Items that technically work but aren't solid. These should be fixed now —
"weak" items left unfixed tend to become permanent technical debt.

**What counts as weak:**

- `TODO` / `FIXME` / `HACK` comments in new code
- Hardcoded values that should come from the spec or MuJoCo
- Loose tolerances (e.g., 1e-3 where MuJoCo conformance demands 1e-10)
- Missing edge-case guards the spec calls for
- Placeholder error handling (e.g., `unwrap()` in non-test code, empty
  `Err(_) => {}` that swallows information)
- Functionality that "passes tests" but uses a different algorithm than
  the spec prescribes
- Dead code or commented-out code from debugging

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|

---

## 8. Deferred Work Tracker

Every item that was in the spec's scope but not fully implemented, plus
anything discovered during implementation or review that's out of scope.
**The goal: nothing deferred is untracked.**

For each item, verify it appears in at least one of:
- `sim/docs/todo/` (future_work file or spec_fleshout)
- `sim/docs/ROADMAP_V1.md`
- The umbrella spec's Out of Scope section (if applicable)

If it's not tracked anywhere, it's a review finding — add tracking now.

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Per-step sensordata trajectory comparison | Out of Scope, bullet 1 | | | |
| Per-step contact count structural check | Out of Scope, bullet 2 | | | |
| Layer B cross-reference diagnostic | Out of Scope, bullet 3 | | | |
| RK4 trajectory tests | Out of Scope, bullet 4 | | | |
| Fixing upstream conformance failures (xipos, collision, constraint) | Out of Scope, bullet 5 | | | |
| Flag combination trajectory tests | Out of Scope, bullet 6 | | | |

### Discovered During Implementation

Items that came up during implementation that weren't anticipated by the
spec. These are the most likely to be untracked.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Discovered During Review

Items found during this review that were not surfaced during
implementation. Reviews are a discovery mechanism — gaps in tracking,
missing match arms, untracked deferred items, and spec inaccuracies
often only become visible during side-by-side review.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Spec Gaps Found During Implementation

Items where the spec was wrong or incomplete and was (or should have been)
updated. Verify the spec was actually updated.

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|

---

## 9. Test Coverage Summary

Quick sanity check on test health after the implementation.

**Domain test results:**
```
sim-conformance-tests: N passed, 0 failed, M ignored
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

---

## Spec B #[ignore] Inventory

Preliminary inventory of `#[ignore]`d tests from Layer C, to be merged with
Spec A's inventory in Session 14.

| Test | Layer | #[ignore] reason (from spec) | First divergent step | Pipeline stage | Likely source phase | Severity |
|------|-------|------------------------------|---------------------|---------------|--------------------|---------|
| `layer_c_trajectory_pendulum` | C | CRBA/RNE xipos cascade | | FK → CRBA → RNE | Phase 1 FK | |
| `layer_c_trajectory_double_pendulum` | C | CRBA/RNE xipos cascade | | FK → CRBA → RNE | Phase 1 FK | |
| `layer_c_trajectory_contact_scenario` | C | constraint forces wrong — contact.pos convention + efc_J assembly | | collision → constraint | Phase 3 collision/constraint | |
| `layer_c_trajectory_actuated_system` | C | CRBA/RNE xipos cascade | | FK → CRBA → RNE | Phase 1 FK | |
| `layer_c_trajectory_tendon_model` | C | CRBA/RNE xipos cascade | | FK → CRBA → RNE | Phase 1 FK | |
| `layer_c_trajectory_sensor_model` | C | CRBA/RNE xipos cascade | | FK → CRBA → RNE | Phase 1 FK | |
| `layer_c_trajectory_equality_model` | C | CRBA/RNE xipos cascade + constraint Jacobian wrong | | FK + constraint | Phase 1 FK + Phase 3 constraint | |
| `layer_c_trajectory_composite_model` | C | qacc wrong + constraint wrong | | FK + collision/constraint | Phase 1 FK + Phase 3 collision/constraint | |
