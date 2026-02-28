# Spec D — Interpolation Actuator Attributes: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/SPEC_D.md`
**Implementation session(s):** Session 12 (from SESSION_PLAN.md)
**Reviewer:** AI agent
**Date:** 2026-02-27

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
| `nsample` parsing | Parsed as int from MJCF, stored in `actuator_history[i,0]` | **Not parsed** — attribute silently ignored | | |
| `interp` parsing | Parsed as string keyword (`"zoh"`, `"linear"`, `"cubic"` → 0, 1, 2), stored in `actuator_history[i,1]` | **Not parsed** — attribute silently ignored | | |
| `delay` parsing | Parsed as float from MJCF, stored in `actuator_delay[i]` | **Not parsed** — attribute silently ignored | | |
| `actuator_history` model array | `int[nu × 2]` — `[nsample, interp]` per actuator, present for ALL actuators | **Does not exist** | | |
| `actuator_historyadr` model array | `int[nu × 1]` — cumulative offset, -1 for no-history | **Does not exist** | | |
| `actuator_delay` model array | `mjtNum[nu × 1]` — present for all actuators, default 0.0 | **Does not exist** | | |
| `nhistory` model field | Total history buffer size (actuators + sensors) | **Does not exist** | | |
| `Data.history` buffer | `mjtNum[nhistory]` — pre-populated with metadata + past timestamps + zero values | **Does not exist** | | |
| Compiler validation | `delay > 0` with `nsample = 0` → error | **No validation** (attributes not parsed) | | |
| Default class inheritance | All three attributes inheritable | **Not applicable** (attributes not parsed) | | |
| `Data::reset()` history restoration | Restores pre-populated initial state (not zeros) | **Not applicable** | | |
| `Data::reset_to_keyframe()` history restoration | Restores same pre-populated initial state | **Not applicable** | | |

**Unclosed gaps:**
{To be filled in Session 14}

---

## 2. Spec Section Compliance

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

### S1. MJCF Type Additions

**Grade:**

**Spec says:**
Add three `Option<T>` fields (`nsample: Option<i32>`, `interp: Option<String>`,
`delay: Option<f64>`) to `MjcfActuator` and `MjcfActuatorDefaults` structs in
`sim/L0/mjcf/src/types.rs`. Add `None` defaults in `Default` impl.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Parser Updates

**Grade:**

**Spec says:**
Parse `nsample`, `interp`, `delay` in the **common** attribute section of
`parse_actuator_attrs()` (NOT inside the `<general>`-only gate). Also parse
in `parse_actuator_defaults()`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Defaults Pipeline

**Grade:**

**Spec says:**
Update `merge_actuator_defaults()` with `c.field.or(p.field)` pattern for
`nsample`/`delay` (Copy types) and `c.interp.clone().or_else(|| p.interp.clone())`
for `interp` (String). Update `apply_to_actuator()` with `if result.field.is_none()`
pattern for all three.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Model Type Additions

**Grade:**

**Spec says:**
Add `InterpolationType` enum to `enums.rs` (variants: `Zoh=0`, `Linear=1`,
`Cubic=2`) with `#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]`,
`#[default]` on `Zoh`, `FromStr` impl (lowercase keywords only), and
`From<i32>` impl (fallback to `Zoh`). Add five fields to `Model`
in `model.rs`: `actuator_nsample: Vec<i32>`, `actuator_interp: Vec<InterpolationType>`,
`actuator_historyadr: Vec<i32>`, `actuator_delay: Vec<f64>`, `nhistory: usize`.
Add empty init in `Model::empty()`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. ModelBuilder and Builder Pipeline

**Grade:**

**Spec says:**
Add three fields to `ModelBuilder` in `builder/mod.rs`. In `process_actuator()`
(builder/actuator.rs): push `nsample`, `interp` (parsed from String via
`FromStr`), and `delay`; add compiler validation `delay > 0 && nsample <= 0` → error.
In `build()` (builder/build.rs): transfer three arrays, then compute
`actuator_historyadr` (cumulative offset, -1 sentinel) and `nhistory` as
post-processing.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Data Integration

**Grade:**

**Spec says:**
Add `history: Vec<f64>` to `Data`. In `make_data()`: allocate `nhistory`-sized
buffer and pre-populate per-actuator: `metadata0=0.0`, `metadata1=float(nsample-1)`,
`times=[-(n)*ts, ..., -ts]`, `values=all 0.0`. In `reset()` and
`reset_to_keyframe()`: restore same pre-populated state (not just zero-fill).
Update `impl Clone for Data` with `history` field. Update
`data_reset_field_inventory` test `EXPECTED_SIZE`.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | `nsample` parsing — `nsample="4"` → `model.actuator_nsample[0]` = 4 | T1 | | |
| AC2 | `interp` keyword parsing — `"zoh"`, `"linear"`, `"cubic"` → `[Zoh, Linear, Cubic]` | T2 | | |
| AC3 | `delay` parsing — `delay="0.006"` → `model.actuator_delay[0]` = 0.006 ± 1e-15 | T1 | | |
| AC4 | `historyadr` multi-actuator — `nsample=[3,0,2]` → `historyadr=[0,-1,8]`, `nhistory=14` | T3 | | |
| AC5 | `Data.history` allocation — `data.history.len()` = 14 | T3 | | |
| AC6 | `Data.history` initial state — matches MuJoCo pre-populated buffer | T4 | | |
| AC7 | Default class inheritance — class "hist" with partial override | T5 | | |
| AC8 | Compiler validation — `delay="0.01"` with no `nsample` → Err containing `"setting delay > 0 without a history buffer (nsample must be > 0)"` | T6 | | |
| AC9 | Default values when attributes omitted — `nsample=0`, `interp=Zoh`, `delay=0.0`, `historyadr=-1`, `nhistory=0`, `data.history.len()=0` | T7 | | |
| AC10 | `Data::reset()` restores initial state after mutation | T8 | | |
| AC11 | `Data::reset_to_keyframe(model, 0)` restores initial state after mutation | T9 | | |
| AC12 | `interp` invalid keyword `"spline"` → error containing "interp" | T10 | | |
| AC13 | Attributes on shortcut types — `<position>` with `nsample`/`interp`/`delay` | T11 | | |
| AC14 | Code structure (code review — 7 checks): (1) new fields present in `Model`, `model_init.rs` (empty init), `Data`, `ModelBuilder`, `builder/build.rs` (transfer + post-process); (2) `InterpolationType` enum in `enums.rs` with `FromStr` and `From<i32>`; (3) `impl Clone for Data` includes `history` field; (4) `data_reset_field_inventory` test `EXPECTED_SIZE` updated; (5) parser places attributes in common section, not general-only gate; (6) both `merge_actuator_defaults()` and `apply_to_actuator()` updated; (7) no `unsafe` blocks in new code | — (code review) | | |

**Missing or failing ACs:**
{To be filled in Session 14}

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Basic nsample/delay parsing — `nsample="4"`, `delay="0.006"` on motor actuator | | | |
| T2 | All three interp keywords — three `<general>` actuators with `nsample="2"` each, `interp="zoh"`, `"linear"`, `"cubic"` | | | |
| T3 | Multi-actuator historyadr and nhistory — three actuators: `nsample="3"`, no `nsample` attr (defaults to 0), `nsample="2"` → `historyadr=[0,-1,8]`, `nhistory=14` | | | |
| T4 | Initial history buffer state — MuJoCo conformance, nsample=4, ts=0.002 and ts=0.01 | | | |
| T5 | Default class inheritance — class "hist" with partial override | | | |
| T6 | Compiler validation — `delay="0.01"`, no `nsample` → Err with `"setting delay > 0 without a history buffer (nsample must be > 0)"` | | | |
| T7 | Default values when attributes omitted | | | |
| T8 | `Data::reset()` restores pre-populated state — overwrite `data.history` with `[99.0; nhistory]`, call `reset(&model)`, assert matches fresh `make_data()` | | | |
| T9 | `Data::reset_to_keyframe(model, 0)` restores pre-populated state — same setup as T8, call `reset_to_keyframe`, assert matches fresh `make_data()` | | | |
| T10 | Invalid interp keyword `"spline"` → error | | | |
| T11 | Attributes on shortcut types — `<position>` with nsample/interp/delay | | | |
| T12 | `nsample=1` minimum valid — single-sample buffer, `nhistory=4`, `historyadr=0`, `meta1=0.0` | | | |
| T13 | `nsample=-1` negative — accepted, `historyadr=-1`, `nhistory=0` | | | |
| T14 | `interp="cubic"` with `nsample=2` — silently accepted | | | |
| T15 | Delay exceeds buffer capacity — `delay=0.1`, `nsample=2`, `timestep=0.002` — silently accepted | | | |
| T16 | `interp="linear"` with `nsample=0` — stores interp independently of nsample | | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| `nsample=0` (no history) | Default — must produce `historyadr=-1`, `nhistory=0` | | | Expected: T7, AC9 |
| `nsample=1` (minimum valid) | Smallest valid buffer: `nhistory=4` | | | Expected: T12 (supplementary) |
| `nsample` negative | MuJoCo silently accepts → `historyadr=-1` | | | Expected: T13 (supplementary) |
| `delay=0.0` with `nsample>0` | Valid — buffer allocated, no delay | | | Expected: T3, AC4 |
| `delay>0` with `nsample=0` | Compile error | | | Expected: T6, AC8 |
| `interp="cubic"` with `nsample<4` | MuJoCo silently accepts — no minimum nsample | | | Expected: T14 (supplementary) |
| Delay exceeding buffer capacity | MuJoCo silently accepts — no validation | | | Expected: T15 (supplementary) |
| Multiple actuators with mixed nsample | Tests cumulative historyadr | | | Expected: T3, AC4 |
| Attributes on `<position>` (shortcut) | Tests common-section parser placement | | | Expected: T11, AC13 |
| Default class with partial override | Tests inheritance pipeline | | | Expected: T5, AC7 |
| `interp` set without `nsample` | MuJoCo accepts: stores `[0, interp_value]` | | | Expected: T16 (supplementary) |

**Missing tests:**
{To be filled in Session 14}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| MJCF with `nsample`/`interp`/`delay` attributes: silently ignored → parsed and stored | | |
| `Model` struct has new fields (empty vecs for no-history models) | | |
| `Data` struct has `history` field (empty vec for no-history models) | | |
| `delay > 0` without `nsample > 0`: silently accepted → compile error | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/mjcf/src/types.rs` | | |
| `sim/L0/mjcf/src/parser.rs` | | |
| `sim/L0/mjcf/src/defaults.rs` | | |
| `sim/L0/core/src/types/model.rs` | | |
| `sim/L0/core/src/types/model_init.rs` | | |
| `sim/L0/core/src/types/enums.rs` | | |
| `sim/L0/core/src/types/data.rs` | | |
| `sim/L0/mjcf/src/builder/mod.rs` | | |
| `sim/L0/mjcf/src/builder/actuator.rs` | | |
| `sim/L0/mjcf/src/builder/build.rs` | | |
| `sim/L0/tests/integration/actuator_phase5.rs` | | |

Unexpected files changed (not in spec's prediction):

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

{To be filled in Session 14}

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All existing actuator tests (`actuator_phase5.rs`) | Pass (unchanged) — new fields have defaults | | |
| `data_reset_field_inventory` (`data.rs:1026`) | **Fails until EXPECTED_SIZE updated** — new `history: Vec<f64>` changes `size_of::<Data>()` | | |
| Phase 4 regression suite (39 tests) | Pass (unchanged) — Phase 5 does not modify Phase 4 code paths | | |
| Full sim domain baseline (2,148+ tests) | Pass (unchanged) — new fields are additive | | |

**Unexpected regressions:**
{To be filled in Session 14}

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `actuator_history` packing | Split into `actuator_nsample: Vec<i32>` and `actuator_interp: Vec<InterpolationType>` — two separate Vecs for type safety | | |
| `actuator_nsample` type | `Vec<i32>` (signed, matching MuJoCo) — overrides umbrella §1 `Vec<usize>` suggestion because MuJoCo accepts negative values | | |
| `actuator_historyadr` type | `Vec<i32>` (signed, `-1` sentinel matches MuJoCo) — direct port over Rust-idiomatic `Option<usize>` | | |
| `actuator_delay` type | `Vec<f64>` — direct port, no translation | | |
| `interp` attribute type | Parse string in MJCF parser, convert to `InterpolationType` enum via `FromStr` in builder. Variants: `Zoh` (default), `Linear`, `Cubic` | | |
| `InterpolationType` default | `InterpolationType::Zoh` with `#[default]` — overrides umbrella §1 `InterpType::None` suggestion because ZOH is the default interpolation method, not "none" | | |
| `nhistory` type | `usize` on `Model` — Rust `usize` for sizes | | |
| `Data.history` type | `Vec<f64>` sized to `nhistory` — direct port | | |
| MJCF attribute names | `nsample`, `interp`, `delay` — match MuJoCo exactly per umbrella Convention Registry §4 | | |
| Rust field names | `actuator_nsample`, `actuator_interp`, `actuator_delay`, `actuator_historyadr` — snake_case per Convention Registry §1 | | |

---

## 7. Weak Implementation Inventory

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
| | | | | |

{To be filled in Session 14}

**Severity guide:**
- **High** — Conformance risk. MuJoCo would produce different results.
  Fix before shipping.
- **Medium** — Code quality issue. Correct behavior but fragile, unclear,
  or not matching spec's prescribed approach. Fix if time permits, else
  track.
- **Low** — Style or minor robustness. No conformance risk. Track if not
  fixing now.

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Runtime delay/interpolation — `mj_forward` reading from history buffer, `mj_step` writing as circular buffer | Out of Scope, bullet 1 | | | |
| Sensor history attributes — `mjsSensor_` `nsample`/`interp`/`delay` and model arrays | Out of Scope, bullet 2 | | | |
| `mj_readCtrl()` / `mj_initCtrlHistory()` — runtime API functions consuming history buffer | Out of Scope, bullet 3 | | | |
| `mj_copyData` history copying — handled naturally by `Vec<f64>` Clone | Out of Scope, bullet 4 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled in Session 14}

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

{To be filled in Session 14}

---

## 9. Test Coverage Summary

**Domain test results:**
```
{To be filled in Session 14}
```

**New tests added:** {To be filled in Session 14}
**Tests modified:** {To be filled in Session 14}
**Pre-existing test regressions:** {To be filled in Session 14}

**Clippy:** {To be filled in Session 14}
**Fmt:** {To be filled in Session 14}

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

**Items to fix before shipping:**
{To be filled in Session 14}

**Items tracked for future work:**
{To be filled in Session 14}
