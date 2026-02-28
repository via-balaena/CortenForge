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
| `nsample` parsing | Parsed as int from MJCF, stored in `actuator_history[i,0]` | **Not parsed** — attribute silently ignored | Parsed as `i32`, stored in `model.actuator_nsample[i]` (T1 passes) | **Yes** |
| `interp` parsing | Parsed as string keyword (`"zoh"`, `"linear"`, `"cubic"` → 0, 1, 2), stored in `actuator_history[i,1]` | **Not parsed** — attribute silently ignored | Parsed as string keyword, converted to `InterpolationType` enum via `FromStr` in builder, stored in `model.actuator_interp[i]` (T2 passes) | **Yes** |
| `delay` parsing | Parsed as float from MJCF, stored in `actuator_delay[i]` | **Not parsed** — attribute silently ignored | Parsed as `f64`, stored in `model.actuator_delay[i]` (T1 passes) | **Yes** |
| `actuator_history` model array | `int[nu × 2]` — `[nsample, interp]` per actuator, present for ALL actuators | **Does not exist** | Split into `actuator_nsample: Vec<i32>` + `actuator_interp: Vec<InterpolationType>` — type-safe equivalent, present for ALL actuators | **Yes** (convention: split instead of packed) |
| `actuator_historyadr` model array | `int[nu × 1]` — cumulative offset, -1 for no-history | **Does not exist** | `actuator_historyadr: Vec<i32>` — cumulative offset, -1 sentinel (T3 passes: `[0, -1, 8]`) | **Yes** |
| `actuator_delay` model array | `mjtNum[nu × 1]` — present for all actuators, default 0.0 | **Does not exist** | `actuator_delay: Vec<f64>` — present for all actuators, default 0.0 (T7 passes) | **Yes** |
| `nhistory` model field | Total history buffer size (actuators + sensors) | **Does not exist** | `nhistory: usize` — actuator-only (sensor contribution deferred, known gap documented) (T3 passes: `nhistory=14`) | **Yes** (actuator portion) |
| `Data.history` buffer | `mjtNum[nhistory]` — pre-populated with metadata + past timestamps + zero values | **Does not exist** | `history: Vec<f64>` — pre-populated with metadata + past timestamps + zero values matching MuJoCo exactly (T4 passes) | **Yes** |
| Compiler validation | `delay > 0` with `nsample = 0` → error | **No validation** (attributes not parsed) | Returns `Err` with `"setting delay > 0 without a history buffer (nsample must be > 0)"` (T6 passes) | **Yes** |
| Default class inheritance | All three attributes inheritable | **Not applicable** (attributes not parsed) | All three attributes inheritable via `merge_actuator_defaults()` + `apply_to_actuator()` (T5 passes) | **Yes** |
| `Data::reset()` history restoration | Restores pre-populated initial state (not zeros) | **Not applicable** | Restores exact pre-populated state matching `make_data()` (T8 passes at 1e-15) | **Yes** |
| `Data::reset_to_keyframe()` history restoration | Restores same pre-populated initial state | **Not applicable** | Restores exact pre-populated state matching `make_data()` (T9 passes at 1e-15) | **Yes** |

**Unclosed gaps:** None. All 12 behaviors are closed.

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

**Grade:** Pass

**Spec says:**
Add three `Option<T>` fields (`nsample: Option<i32>`, `interp: Option<String>`,
`delay: Option<f64>`) to `MjcfActuator` and `MjcfActuatorDefaults` structs in
`sim/L0/mjcf/src/types.rs`. Add `None` defaults in `Default` impl.

**Implementation does:**
All three fields present in `MjcfActuator` (lines 2401-2406) and `MjcfActuatorDefaults` (lines 721-726) with correct types. `Default` impl for `MjcfActuator` initializes all three to `None` (lines 2502-2504). `MjcfActuatorDefaults` derives `Default` (auto-None).

**Gaps (if any):** None.

**Action:** None.

### S2. Parser Updates

**Grade:** Pass

**Spec says:**
Parse `nsample`, `interp`, `delay` in the **common** attribute section of
`parse_actuator_attrs()` (NOT inside the `<general>`-only gate). Also parse
in `parse_actuator_defaults()`.

**Implementation does:**
Parsed at lines 2070-2072 of `parse_actuator_attrs()` in the common section, before the `<general>`-only gate at line 2079. Also parsed at lines 806-808 of `parse_actuator_defaults()` in the common section after the general-specific block.

**Gaps (if any):** None. Parser placement is exactly as specified — critical for shortcut type conformance (verified by T11).

**Action:** None.

### S3. Defaults Pipeline

**Grade:** Pass

**Spec says:**
Update `merge_actuator_defaults()` with `c.field.or(p.field)` pattern for
`nsample`/`delay` (Copy types) and `c.interp.clone().or_else(|| p.interp.clone())`
for `interp` (String). Update `apply_to_actuator()` with `if result.field.is_none()`
pattern for all three.

**Implementation does:**
`merge_actuator_defaults()` at lines 780-782: `nsample: c.nsample.or(p.nsample)`, `interp: c.interp.clone().or_else(|| p.interp.clone())`, `delay: c.delay.or(p.delay)`. `apply_to_actuator()` at lines 399-407: all three using `if result.field.is_none()` pattern. `interp` uses `clone_from` (correct for String).

**Gaps (if any):** None.

**Action:** None.

### S4. Model Type Additions

**Grade:** Pass

**Spec says:**
Add `InterpolationType` enum to `enums.rs` (variants: `Zoh=0`, `Linear=1`,
`Cubic=2`) with `#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]`,
`#[default]` on `Zoh`, `FromStr` impl (lowercase keywords only), and
`From<i32>` impl (fallback to `Zoh`). Add five fields to `Model`
in `model.rs`: `actuator_nsample: Vec<i32>`, `actuator_interp: Vec<InterpolationType>`,
`actuator_historyadr: Vec<i32>`, `actuator_delay: Vec<f64>`, `nhistory: usize`.
Add empty init in `Model::empty()`.

**Implementation does:**
`InterpolationType` enum at `enums.rs:246-278` — all derives present (also adds `Hash` beyond spec minimum, harmless). `#[default]` on `Zoh`. `FromStr` matches exactly (lowercase only, error format matches spec). `From<i32>` matches (0→Zoh, 1→Linear, 2→Cubic, _→Zoh). Five Model fields at `model.rs:592-609`. `Model::empty()` at `model_init.rs:255-259` initializes all five. Re-exported from `lib.rs:199`.

**Gaps (if any):** None.

**Action:** None.

### S5. ModelBuilder and Builder Pipeline

**Grade:** Pass

**Spec says:**
Add three fields to `ModelBuilder` in `builder/mod.rs`. In `process_actuator()`
(builder/actuator.rs): push `nsample`, `interp` (parsed from String via
`FromStr`), and `delay`; add compiler validation `delay > 0 && nsample <= 0` → error.
In `build()` (builder/build.rs): transfer three arrays, then compute
`actuator_historyadr` (cumulative offset, -1 sentinel) and `nhistory` as
post-processing.

**Implementation does:**
Three fields on `ModelBuilder` at `mod.rs:546-548`. Initialized in `init.rs:135-137`. `process_actuator()` at `actuator.rs:264-283`: interp parsed via `FromStr`, validation at lines 275-279 (exact error string matches spec), pushes at 281-283. `build()` at `build.rs:264-268`: transfers three arrays, initializes historyadr/nhistory to empty/0. Post-processing at `build.rs:390-406`: cumulative historyadr computation with -1 sentinel, nhistory as offset cast to usize. Uses idiomatic `iter_mut().zip()` instead of indexed loop — cleaner than spec's pseudocode, same algorithm.

**Gaps (if any):** None.

**Action:** None.

### S6. Data Integration

**Grade:** Pass

**Spec says:**
Add `history: Vec<f64>` to `Data`. In `make_data()`: allocate `nhistory`-sized
buffer and pre-populate per-actuator: `metadata0=0.0`, `metadata1=float(nsample-1)`,
`times=[-(n)*ts, ..., -ts]`, `values=all 0.0`. In `reset()` and
`reset_to_keyframe()`: restore same pre-populated state (not just zero-fill).
Update `impl Clone for Data` with `history` field. Update
`data_reset_field_inventory` test `EXPECTED_SIZE`.

**Implementation does:**
`history: Vec<f64>` field at `data.rs:81`. `make_data()` at `model_init.rs:389-410`: allocates and pre-populates exactly as spec — metadata0=0.0, metadata1=(n-1) as f64, times=[-(n-k)*ts], values=0.0. `reset()` at `data.rs:874-891`: fills zero then re-populates (matching `mj_resetData`). `reset_to_keyframe()` at `data.rs:1028-1045`: identical logic. `Clone` at `data.rs:632`: `history: self.history.clone()`. `EXPECTED_SIZE` updated to 4128 at `data.rs:1073`.

**Gaps (if any):** None.

**Action:** None.

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | `nsample` parsing — `nsample="4"` → `model.actuator_nsample[0]` = 4 | T1 | **Pass** | `spec_d_t1_nsample_delay_parsing` |
| AC2 | `interp` keyword parsing — `"zoh"`, `"linear"`, `"cubic"` → `[Zoh, Linear, Cubic]` | T2 | **Pass** | `spec_d_t2_interp_keywords` |
| AC3 | `delay` parsing — `delay="0.006"` → `model.actuator_delay[0]` = 0.006 ± 1e-15 | T1 | **Pass** | Tested alongside nsample |
| AC4 | `historyadr` multi-actuator — `nsample=[3,0,2]` → `historyadr=[0,-1,8]`, `nhistory=14` | T3 | **Pass** | `spec_d_t3_historyadr_multi_actuator` |
| AC5 | `Data.history` allocation — `data.history.len()` = 14 | T3 | **Pass** | Combined in T3 |
| AC6 | `Data.history` initial state — matches MuJoCo pre-populated buffer | T4 | **Pass** | `spec_d_t4_initial_history_state`, both ts=0.002 and ts=0.01 variants |
| AC7 | Default class inheritance — class "hist" with partial override | T5 | **Pass** | `spec_d_t5_default_class_inheritance` |
| AC8 | Compiler validation — `delay="0.01"` with no `nsample` → Err containing `"setting delay > 0 without a history buffer (nsample must be > 0)"` | T6 | **Pass** | `spec_d_t6_delay_without_history_error` — assertion tightened in review (was `||`, now matches spec substring) |
| AC9 | Default values when attributes omitted — `nsample=0`, `interp=Zoh`, `delay=0.0`, `historyadr=-1`, `nhistory=0`, `data.history.len()=0` | T7 | **Pass** | `spec_d_t7_default_values_omitted` |
| AC10 | `Data::reset()` restores initial state after mutation | T8 | **Pass** | `spec_d_t8_reset_restores_history` |
| AC11 | `Data::reset_to_keyframe(model, 0)` restores initial state after mutation | T9 | **Pass** | `spec_d_t9_reset_to_keyframe_restores_history` |
| AC12 | `interp` invalid keyword `"spline"` → error containing "interp" | T10 | **Pass** | `spec_d_t10_invalid_interp_keyword` |
| AC13 | Attributes on shortcut types — `<position>` with `nsample`/`interp`/`delay` | T11 | **Pass** | `spec_d_t11_shortcut_type_parsing` |
| AC14 | Code structure (code review — 7 checks) | — | **Pass** | (1) Fields in Model/model_init/Data/ModelBuilder/build.rs: ✓ (2) InterpolationType in enums.rs with FromStr+From<i32>: ✓ (3) Clone includes history: ✓ data.rs:632 (4) EXPECTED_SIZE=4128: ✓ data.rs:1073 (5) Parser common section: ✓ parser.rs:2070-2072 (6) merge+apply both updated: ✓ defaults.rs:780-782,399-407 (7) No unsafe: ✓ grep confirms |

**Missing or failing ACs:** None. All 14 acceptance criteria pass.

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Basic nsample/delay parsing — `nsample="4"`, `delay="0.006"` on motor actuator | **Yes** | `spec_d_t1_nsample_delay_parsing` | Exact match |
| T2 | All three interp keywords — three `<general>` actuators with `nsample="2"` each, `interp="zoh"`, `"linear"`, `"cubic"` | **Yes** | `spec_d_t2_interp_keywords` | Exact match |
| T3 | Multi-actuator historyadr and nhistory — three actuators: `nsample="3"`, no `nsample` attr (defaults to 0), `nsample="2"` → `historyadr=[0,-1,8]`, `nhistory=14` | **Yes** | `spec_d_t3_historyadr_multi_actuator` | Exact match |
| T4 | Initial history buffer state — MuJoCo conformance, nsample=4, ts=0.002 and ts=0.01 | **Yes** | `spec_d_t4_initial_history_state` | Both timestep variants tested |
| T5 | Default class inheritance — class "hist" with partial override | **Yes** | `spec_d_t5_default_class_inheritance` | Exact match |
| T6 | Compiler validation — `delay="0.01"`, no `nsample` → Err with `"setting delay > 0 without a history buffer (nsample must be > 0)"` | **Yes** | `spec_d_t6_delay_without_history_error` | Assertion tightened in review |
| T7 | Default values when attributes omitted | **Yes** | `spec_d_t7_default_values_omitted` | Exact match |
| T8 | `Data::reset()` restores pre-populated state — overwrite `data.history` with `[99.0; nhistory]`, call `reset(&model)`, assert matches fresh `make_data()` | **Yes** | `spec_d_t8_reset_restores_history` | Exact match, 1e-15 tolerance |
| T9 | `Data::reset_to_keyframe(model, 0)` restores pre-populated state — same setup as T8, call `reset_to_keyframe`, assert matches fresh `make_data()` | **Yes** | `spec_d_t9_reset_to_keyframe_restores_history` | Exact match, 1e-15 tolerance |
| T10 | Invalid interp keyword `"spline"` → error | **Yes** | `spec_d_t10_invalid_interp_keyword` | Exact match |
| T11 | Attributes on shortcut types — `<position>` with nsample/interp/delay | **Yes** | `spec_d_t11_shortcut_type_parsing` | Exact match |
| T12 | `nsample=1` minimum valid — single-sample buffer, `nhistory=4`, `historyadr=0`, `meta1=0.0` | **Yes** | `spec_d_t12_nsample_minimum_valid` | Supplementary, exact match |
| T13 | `nsample=-1` negative — accepted, `historyadr=-1`, `nhistory=0` | **Yes** | `spec_d_t13_nsample_negative` | Supplementary, exact match |
| T14 | `interp="cubic"` with `nsample=2` — silently accepted | **Yes** | `spec_d_t14_cubic_insufficient_samples` | Supplementary, exact match |
| T15 | Delay exceeds buffer capacity — `delay=0.1`, `nsample=2`, `timestep=0.002` — silently accepted | **Yes** | `spec_d_t15_delay_exceeds_buffer` | Supplementary, exact match |
| T16 | `interp="linear"` with `nsample=0` — stores interp independently of nsample | **Yes** | `spec_d_t16_interp_without_nsample` | Supplementary, exact match |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| `nsample=0` (no history) | Default — must produce `historyadr=-1`, `nhistory=0` | **Yes** | T7, AC9 | |
| `nsample=1` (minimum valid) | Smallest valid buffer: `nhistory=4` | **Yes** | T12 | Boundary verified |
| `nsample` negative | MuJoCo silently accepts → `historyadr=-1` | **Yes** | T13 | |
| `delay=0.0` with `nsample>0` | Valid — buffer allocated, no delay | **Yes** | T3 | nsample=3,2 with default delay=0.0 |
| `delay>0` with `nsample=0` | Compile error | **Yes** | T6, AC8 | |
| `interp="cubic"` with `nsample<4` | MuJoCo silently accepts — no minimum nsample | **Yes** | T14 | |
| Delay exceeding buffer capacity | MuJoCo silently accepts — no validation | **Yes** | T15 | |
| Multiple actuators with mixed nsample | Tests cumulative historyadr | **Yes** | T3, AC4 | |
| Attributes on `<position>` (shortcut) | Tests common-section parser placement | **Yes** | T11, AC13 | |
| Default class with partial override | Tests inheritance pipeline | **Yes** | T5, AC7 | |
| `interp` set without `nsample` | MuJoCo accepts: stores `[0, interp_value]` | **Yes** | T16 | |

**Missing tests:** None. All 16 planned tests implemented. All 11 edge cases covered.

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| MJCF with `nsample`/`interp`/`delay` attributes: silently ignored → parsed and stored | **Yes** | Verified by T1, T2 |
| `Model` struct has new fields (empty vecs for no-history models) | **Yes** | 5 new fields, verified by T7 |
| `Data` struct has `history` field (empty vec for no-history models) | **Yes** | `data.history.len()==0` for no-history, verified by T7 |
| `delay > 0` without `nsample > 0`: silently accepted → compile error | **Yes** | Verified by T6 |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/mjcf/src/types.rs` | **Yes** | |
| `sim/L0/mjcf/src/parser.rs` | **Yes** | |
| `sim/L0/mjcf/src/defaults.rs` | **Yes** | |
| `sim/L0/core/src/types/model.rs` | **Yes** | |
| `sim/L0/core/src/types/model_init.rs` | **Yes** | |
| `sim/L0/core/src/types/enums.rs` | **Yes** | |
| `sim/L0/core/src/types/data.rs` | **Yes** | |
| `sim/L0/mjcf/src/builder/mod.rs` | **Yes** | |
| `sim/L0/mjcf/src/builder/actuator.rs` | **Yes** | |
| `sim/L0/mjcf/src/builder/build.rs` | **Yes** | |
| `sim/L0/tests/integration/actuator_phase5.rs` | **Yes** | |

Unexpected files changed (not in spec's prediction):

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| `sim/L0/core/src/lib.rs` | Re-export `InterpolationType` from public API — required for downstream crates to reference the enum. Trivial (1 line). |
| `sim/L0/mjcf/src/builder/init.rs` | Initialize 3 new `ModelBuilder` fields to `vec![]` — spec said "builder/mod.rs" but builder init is in a separate file. Trivial (3 lines). |
| `SESSION_PLAN.md` | Status update from Pending to Done. Expected docs change. |
| `SPEC_D.md` | Status updated to "Implemented". Expected docs change. |

All unexpected changes are trivial (re-exports, initializations, doc updates). No surprise behavioral changes.

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All existing actuator tests (`actuator_phase5.rs`) | Pass (unchanged) — new fields have defaults | **Pass (all pre-existing tests pass)** | No |
| `data_reset_field_inventory` (`data.rs:1073`) | **Fails until EXPECTED_SIZE updated** — new `history: Vec<f64>` changes `size_of::<Data>()` | **Updated to 4128, passes** | No |
| Phase 4 regression suite (39 tests) | Pass (unchanged) — Phase 5 does not modify Phase 4 code paths | **Pass (39/39)** | No |
| Full sim domain baseline (2,148+ tests) | Pass (unchanged) — new fields are additive | **Pass (2,176 passed, 0 failed)** | No |

**Unexpected regressions:** None.

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `actuator_history` packing | Split into `actuator_nsample: Vec<i32>` and `actuator_interp: Vec<InterpolationType>` — two separate Vecs for type safety | **Yes** | model.rs:592,596 |
| `actuator_nsample` type | `Vec<i32>` (signed, matching MuJoCo) — overrides umbrella §1 `Vec<usize>` suggestion because MuJoCo accepts negative values | **Yes** | model.rs:592, T13 verifies negative acceptance |
| `actuator_historyadr` type | `Vec<i32>` (signed, `-1` sentinel matches MuJoCo) — direct port over Rust-idiomatic `Option<usize>` | **Yes** | model.rs:600, T3 verifies -1 sentinel |
| `actuator_delay` type | `Vec<f64>` — direct port, no translation | **Yes** | model.rs:604 |
| `interp` attribute type | Parse string in MJCF parser, convert to `InterpolationType` enum via `FromStr` in builder. Variants: `Zoh` (default), `Linear`, `Cubic` | **Yes** | parser.rs:2071 stores String, actuator.rs:265-270 converts via FromStr |
| `InterpolationType` default | `InterpolationType::Zoh` with `#[default]` — overrides umbrella §1 `InterpType::None` suggestion because ZOH is the default interpolation method, not "none" | **Yes** | enums.rs:248 `#[default]` on Zoh |
| `nhistory` type | `usize` on `Model` — Rust `usize` for sizes | **Yes** | model.rs:609 |
| `Data.history` type | `Vec<f64>` sized to `nhistory` — direct port | **Yes** | data.rs:81 |
| MJCF attribute names | `nsample`, `interp`, `delay` — match MuJoCo exactly per umbrella Convention Registry §4 | **Yes** | parser.rs:2070-2072 uses exact MuJoCo names |
| Rust field names | `actuator_nsample`, `actuator_interp`, `actuator_delay`, `actuator_historyadr` — snake_case per Convention Registry §1 | **Yes** | All field names match spec |

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
| 1 | `actuator_phase5.rs` T6 (line 876) | Error message assertion used `contains("delay") \|\| contains("history")` — too loose to catch wrong-but-plausible errors. Spec requires `"setting delay > 0 without a history buffer"`. | Medium | **Fixed in this review session** — tightened to `contains("setting delay > 0 without a history buffer")` |

No TODO/FIXME/HACK comments in new code (verified by grep).
No `unsafe` blocks in new code (verified by grep).
No dead code or commented-out debugging code.
All tolerances are 1e-15 (matching MuJoCo conformance standard).

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
| Runtime delay/interpolation — `mj_forward` reading from history buffer, `mj_step` writing as circular buffer | Out of Scope, bullet 1 | `future_work_10g.md`, `ROADMAP_V1.md` Phase 5 | DT-107 | **Yes** |
| Sensor history attributes — `mjsSensor_` `nsample`/`interp`/`delay` and model arrays, sensor contribution to `nhistory` | Out of Scope, bullet 2 | `future_work_10h.md`, `ROADMAP_V1.md` Phase 6 | DT-109 | **Yes** |
| `dyntype` enum gating interpolation eligibility — restrict which `ActuatorDynamics` variants may use history buffer | Out of Scope (implicit — not gated in current impl) | `future_work_10g.md`, `ROADMAP_V1.md` Phase 5 | DT-108 | **Yes** |
| `actuator_plugin` model array — per-actuator plugin ID, depends on §66 plugin system | Out of Scope (implicit — plugin system not implemented) | `future_work_10g.md`, `ROADMAP_V1.md` Phase 5 | DT-110 | **Yes** |
| `mj_readCtrl()` / `mj_initCtrlHistory()` — runtime API functions consuming history buffer | Out of Scope, bullet 3 | Covered by DT-107 (runtime interpolation) | DT-107 | **Yes** |
| `mj_copyData` history copying — handled naturally by `Vec<f64>` Clone | Out of Scope, bullet 4 | N/A — handled by Rust Clone | N/A | **Yes** — `impl Clone for Data` includes `history` field (data.rs:632) |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| (none) | No gaps discovered during implementation | N/A | N/A | N/A |

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| (none) | Implementation matched spec exactly — no spec gaps discovered | N/A | |

---

## 9. Test Coverage Summary

**Domain test results:**
```
sim-conformance-tests: 1,015 passed, 0 failed, 1 ignored
sim-core:              423 passed, 0 failed, 0 ignored
sim-physics:           285 passed, 0 failed, 0 ignored
sim-mjcf:              187 passed, 0 failed, 0 ignored
sim-sensor:             63 passed, 0 failed, 0 ignored
sim-types:              52 passed, 0 failed, 0 ignored
sim-simd:               53 passed, 0 failed, 0 ignored
sim-constraint:         39 passed, 0 failed, 0 ignored
sim-tendon:             22 passed, 0 failed, 0 ignored
sim-muscle:              6 passed, 0 failed, 0 ignored
sim-urdf:                8 passed, 0 failed, 3 ignored
Total:               2,176 passed, 0 failed, 4+ ignored
```

**New tests added:** 16 (T1–T16 in `actuator_phase5.rs`, ~490 lines)
**Tests modified:** 1 (T6 assertion tightened during review — `||` → exact substring)
**Pre-existing test regressions:** None

**Clippy:** Clean (0 warnings on sim-core, sim-mjcf with `-D warnings`)
**Fmt:** Clean (no formatting issues)

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | **All 12 gaps closed** |
| Spec section compliance | 2 | **All 6 sections Pass** |
| Acceptance criteria | 3 | **14/14 Pass** |
| Test plan completeness | 4 | **16/16 tests implemented, 11/11 edge cases covered** |
| Blast radius accuracy | 5 | **All predictions matched, 2 trivial unexpected files (re-export + init)** |
| Convention fidelity | 6 | **All 10 conventions followed** |
| Weak items | 7 | **1 found, 1 fixed (T6 assertion tightened)** |
| Deferred work tracking | 8 | **All 4 out-of-scope items tracked, 0 gaps discovered** |
| Test health | 9 | **2,176 passed, 0 failed, clippy clean, fmt clean** |

**Overall:** **Pass.** Spec D implementation is complete and faithful. All conformance gaps are closed. All acceptance criteria pass. All planned tests implemented with correct assertions. No weak implementations remain after the T6 fix.

**Items to fix before shipping:**
1. ~~T6 assertion too loose~~ — **Fixed in this session.** Tightened from `contains("delay") || contains("history")` to `contains("setting delay > 0 without a history buffer")`.

**Items tracked for future work:**
1. Runtime delay/interpolation (mj_forward history consumption) — tracked in Spec D Out of Scope
2. Sensor history attributes — tracked in Spec D Out of Scope
3. `mj_readCtrl()` / `mj_initCtrlHistory()` — tracked in Spec D Out of Scope
