# Spec D — Sensor History Attributes: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_D.md`
**Implementation session(s):** Session 18
**Reviewer:** AI agent
**Date:** 2026-03-01

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
| Parse `nsample` on sensor elements | Stored in `sensor_history[2*i+0]` (`mjmodel.h:1218`) | **Not parsed** — field does not exist on `MjcfSensor` | | |
| Parse `interp` on sensor elements | Stored in `sensor_history[2*i+1]` as int 0/1/2 (`mjmodel.h:1218`) | **Not parsed** | | |
| Parse `delay` on sensor elements | Stored in `sensor_delay[i]` (`mjmodel.h:1219`) | **Not parsed** | | |
| Parse `interval` on sensor elements | Stored in `sensor_interval[2*i+0]` (period), phase=0.0 (`mjmodel.h:1221`) | **Not parsed** | | |
| Compute sensor `historyadr` | Cumulative offset, -1 for nsample<=0 (`mjmodel.h:1220`) | **Not computed** — field does not exist | | |
| Include sensors in `nhistory` | `nhistory = Σ(actuator) + Σ(sensor)` | **Actuator-only** (`model.rs:607–608`) | | |
| Validate delay/nsample | `delay > 0 && nsample <= 0` → error | **Not validated** for sensors | | |
| Validate negative interval | `interval < 0` → error | **Not validated** | | |
| Sensor defaults for history attrs | **No sensor defaults in MuJoCo** (EGT-11h) | N/A — `MjcfSensorDefaults` is a CortenForge extension | | |

**Unclosed gaps:**
{To be filled during review execution.}

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

### S1. Add 4 history fields to `MjcfSensor` struct

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/types.rs`. **MuJoCo equivalent:** `mjsSensor_::nsample/
interp/delay/interval` in `mjspec.h:731–734`.
Add 4 `Option<T>` fields to `MjcfSensor` struct (after `body2`, before
closing brace): `nsample: Option<i32>`, `interp: Option<String>`,
`delay: Option<f64>`, `interval: Option<f64>`. Use `Option<T>` (not
value-sentinel) because `nsample=0` means "no history" and must not be
confused with "not set." Update `impl Default for MjcfSensor` struct literal
(compiler-enforced): all 4 fields set to `None`. Add 4 fluent builder methods
(`with_nsample`, `with_interp`, `with_delay`, `with_interval`) with
`#[must_use]` and `Into<String>` for interp. `MjcfSensorDefaults`
(`types.rs:775`) is NOT modified — MuJoCo has no sensor default class
mechanism (EGT-11h, AD-3(b)).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Parse 4 history attributes in `parse_sensor_attrs()`

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/parser.rs`. **MuJoCo equivalent:** MJCF attribute
parsing wired via `mjxmacro.h:695–698`.
Add 4 attribute reads after the existing `user` parsing block (line 3521),
before the `sensor` return (line 3523). Uses existing `parse_int_attr()`
(line 3130) for `nsample`, existing `parse_float_attr()` (line 3125) for
`delay` and `interval`, and `get_attribute_opt()` for `interp` (raw string —
string→enum conversion happens in the builder, matching the actuator pattern
at `actuator.rs:264–270`). `parse_sensor_defaults()` (`parser.rs:909`) is
NOT modified — history attrs have no default mechanism in MuJoCo (EGT-11h,
AD-3(b)).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Add 4 accumulation fields to `ModelBuilder` struct

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/builder/mod.rs`. **MuJoCo equivalent:** Builder-internal
accumulation arrays.
Add 4 fields after `sensor_cutoff` (line 644), before `sensor_name_list`:
`sensor_nsample: Vec<i32>`, `sensor_interp: Vec<InterpolationType>`,
`sensor_delay: Vec<f64>`, `sensor_interval: Vec<(f64, f64)>`. Types match
the `Model` struct target fields. `InterpolationType` must be imported from
`sim_core` (already used in `actuator.rs`).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Initialize 4 new fields in `ModelBuilder::new()`

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/builder/init.rs`. **MuJoCo equivalent:** Builder
initialization.
Initialize 4 new fields to `vec![]` in struct literal, after
`sensor_cutoff: vec![]` (line 228), before `sensor_name_list`. Struct
literal pattern — compiler-enforced completeness (missing field = compile
error).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Validate and push history attrs in `process_sensors()`

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/builder/sensor.rs`. **MuJoCo equivalent:** MuJoCo
compiler sensor validation + array population.
Mirrors the actuator pattern at `builder/actuator.rs:264–283` exactly.
Order: (1) parse interp string→`InterpolationType` FIRST via
`.parse::<InterpolationType>().map_err(...)`, with `None` defaulting to
`InterpolationType::Zoh`; (2) validate `delay > 0.0 && nsample <= 0` →
error `"setting delay > 0 without a history buffer (nsample must be > 0)"`;
(3) validate `interval < 0.0` → error `"negative interval in sensor"`;
(4) push 4 fields (`sensor_nsample`, `sensor_interp`, `sensor_delay`,
`sensor_interval`) inside the SAME push block as existing 11 fields (lines
83–100). Same push block is critical: the `continue` at line 33 for
unsupported types skips all pushes, maintaining Vec alignment. A separate
loop would break this. Import `InterpolationType` at line 7. Update doc
comments at lines 2 and 16: change "13 pipeline sensor arrays" to
"17 pipeline sensor arrays". `apply_to_sensor()` (`defaults.rs:533`)
untouched — only applies `noise`/`cutoff`/`user` defaults.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Transfer 4 fields + init historyadr in `build.rs`

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/builder/build.rs`. **MuJoCo equivalent:** Model
construction (spec→model transfer).
Direct transfer for 4 attribute fields (same pattern as `sensor_noise`/
`sensor_cutoff`): `sensor_nsample: self.sensor_nsample`,
`sensor_interp: self.sensor_interp`, `sensor_delay: self.sensor_delay`,
`sensor_interval: self.sensor_interval`. Initialize
`sensor_historyadr: vec![]` (computed post-hoc, same pattern as
`actuator_historyadr: vec![]` at line 267). Add after `sensor_cutoff`
transfer (line 241), before `sensor_name` (line 242).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Add 5 fields to `Model` struct

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/types/model.rs`. **MuJoCo equivalent:**
`sensor_history`/`sensor_historyadr`/`sensor_delay`/`sensor_interval` in
`mjmodel.h:1218–1221`.
Add 5 fields after `sensor_cutoff` (line 514), before `sensor_name`
(line 516): `sensor_nsample: Vec<i32>` (MuJoCo: `sensor_history[2*i+0]`,
signed to accept negative values per EGT-5), `sensor_interp:
Vec<InterpolationType>` (MuJoCo: `sensor_history[2*i+1]` as int 0/1/2),
`sensor_historyadr: Vec<i32>` (MuJoCo: `sensor_historyadr`, -1 when
nsample<=0, offsets start after actuator history),
`sensor_delay: Vec<f64>` (MuJoCo: `sensor_delay`),
`sensor_interval: Vec<(f64, f64)>` (MuJoCo: `sensor_interval[2*i]` =
period, `sensor_interval[2*i+1]` = phase, phase always 0.0). Update
`nhistory` comment from "Total history buffer size (actuator contributions
only)" to "Total history buffer size (actuator + sensor contributions)"
with layout description: actuators first (offset 0 → actuator_total), then
sensors (actuator_total → nhistory). Formula per entity:
`nsample * (dim + 1) + 2`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S8. Initialize 5 new fields in `Model::empty()`

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/types/model_init.rs`. **MuJoCo equivalent:** Model
initialization.
Initialize 5 new fields to `vec![]` in struct literal, after
`sensor_cutoff: vec![]` (line 231), before `sensor_name: vec![]` (line 232).
Struct literal — compiler-enforced completeness.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S9. Extend post-hoc historyadr computation to include sensors

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/builder/build.rs`. **MuJoCo equivalent:** MuJoCo
compiler historyadr + nhistory computation.
Extends the existing actuator-only block at lines 387–406. The actuator loop
runs first (unchanged), producing `offset = actuator_total`. Then a sensor
loop appends, using `nsample * (dim + 1) + 2` per sensor with `nsample > 0`.
Sensor `dim` comes from `model.sensor_dim[i]`. Arithmetic: `nsample` is
`i32`, `dim` is `usize`. Cast sequence: `ns * (dim as i32 + 1) + 2` keeping
`offset: i32`. Layout: actuators first (offset 0 → act_total), sensors after
(act_total → nhistory). Final `model.nhistory = offset as usize` covers
both. `#[allow(clippy::cast_sign_loss)]` retained on the final cast.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S10. Update stale documentation

**Grade:**

**Spec says:**
Documentation-only changes — no behavioral impact. 6 stale doc comment sites
across 3 files:
**(a)** `sim/L0/core/src/types/data.rs:78` — change `"Actuator history
buffer"` to `"History buffer (actuators + sensors)"`.
**(b)** `sim/L0/core/src/types/enums.rs:253–254` — change `"Interpolation
method for actuator history buffer"` to `"Interpolation method for actuator
and sensor history buffers"`, update second line to reference both
`actuator_history` and `sensor_history`.
**(c)** `sim/L0/mjcf/src/builder/sensor.rs:2` — module doc: change "13
pipeline sensor arrays" to "17 pipeline sensor arrays".
**(d)** `sim/L0/mjcf/src/builder/sensor.rs:16` — function doc: change "13
pipeline sensor arrays" to "17 pipeline sensor arrays".
**(e)** `sim/L0/mjcf/src/builder/sensor.rs:28` — stale unsupported-type
comment that mentions "Jointlimitfrc, Tendonlimitfrc" but Spec C already
added support for these. Update to reflect current state.
**(f)** `sim/L0/mjcf/src/builder/sensor.rs:467` — stale `convert_sensor_type`
doc saying "Returns `None` for types not yet implemented" but all types are
now implemented. Update to reflect that all types are implemented.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Parse nsample attribute — `sensor_nsample[0] == 5` | T1 | | |
| AC2 | Parse interp attribute — `sensor_interp[0] == Linear` | T1 | | |
| AC3 | Parse delay attribute — `sensor_delay[0] == 0.02` | T1 | | |
| AC4 | Parse interval attribute — `sensor_interval[0] == (0.5, 0.0)` | T2 | | |
| AC5 | nhistory correct for dim=1 sensor — `nhistory == 12`, `historyadr == 0` | T3 | | |
| AC6 | nhistory correct for dim=3 sensor — `nhistory == 14`, `historyadr == 0` | T3 | | |
| AC7 | Mixed actuator+sensor layout — `act_historyadr[0]==0`, `sens_historyadr[0]==10`, `nhistory==18` | T4 | | |
| AC8 | Defaults — `nsample==0`, `interp==Zoh`, `delay==0.0`, `interval==(0.0,0.0)`, `historyadr==-1`, `nhistory==0` | T5 | | |
| AC9 | Validation — `delay > 0` without `nsample > 0` → error "delay > 0 without a history buffer" | T6 | | |
| AC10 | Validation — `interval < 0` → error "negative interval in sensor" | T7 | | |
| AC11 | Invalid interp "Linear" rejected → error "invalid interp keyword" | T8 | | |
| AC12 | 3 sensors offset accumulation — `historyadr == [0, 8, 20]`, `nhistory == 30` | T9 | | |
| AC13 | nsample=0 in middle — `historyadr == [0, -1, 8]`, `nhistory == 20` | T9 | | |
| AC14 | Full combination (all 4 attrs) — all fields + `nhistory == 22` | T10 | | |
| AC15 | Actuator stability — `act_historyadr==[0,10]`, `sens_historyadr==[16,24]`, `nhistory==36` | T11 | | |
| AC16 | Negative nsample=-1 accepted — `nsample==-1`, `historyadr==-1`, `nhistory==0` | T12 | | |
| AC17 | Negative delay=-0.01 accepted — `delay==-0.01`, no error | T12 | | |
| AC18 | interval=0.5 without nsample accepted — `interval==(0.5,0.0)`, `historyadr==-1` | T13 | | |
| AC19 | interp="linear" stored without nsample — `interp==Linear`, `historyadr==-1` | T13 | | |
| AC20 | Empty-string interp="" rejected → error "invalid interp keyword" | T8 | | |
| AC21 | Sensor-only model (no actuators) — `historyadr[0]==0`, `nhistory==8` | T14 | | |
| AC22 | Vec-length invariant — all 5 new Vecs have `len() == nsensor` | T9 | | |
| AC23 | delay+interval without nsample → delay error fires (interval doesn't override) | T6 | | |
| AC24 | Mixed actuator+multi-dim sensor — `act_historyadr[0]==0`, `sens_historyadr[0]==10`, `nhistory==24` | T16 | | |
| AC25 | Structural completeness (code review — not runtime) | — (code review) | | All 5 model fields in `empty()`, 4 attrs parsed, defaults NOT extended, push block alignment, doc comments updated |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Parse basic history attributes (nsample=5, interp="linear", delay=0.02) → AC1, AC2, AC3 | | | |
| T2 | Parse interval attribute (interval=0.5 → (0.5, 0.0)) → AC4 | | | |
| T3 | nhistory formula across dims: jointpos nsample=5 (dim=1, contrib=12), framepos nsample=3 (dim=3, contrib=14), framequat nsample=1 (dim=4, contrib=7) → `nhistory=33` → AC5, AC6 | | | |
| T4 | Mixed actuator+sensor buffer: motor nsample=4 (offset=0, contrib=10) + jointpos sensor nsample=3 (offset=10, contrib=8) → `nhistory=18` → AC7 | | | |
| T5 | Default values: no history attrs → `nsample==0`, `interp==Zoh`, `delay==0.0`, `interval==(0.0,0.0)`, `historyadr==-1`, `nhistory==0` → AC8 | | | |
| T6 | Validation — delay without nsample: (a) delay=0.01 only → error; (b) delay=0.01 + interval=0.5 → same error → AC9, AC23 | | | |
| T7 | Validation — negative interval: interval=-0.5 → error "negative interval in sensor" → AC10 | | | |
| T8 | Invalid interp keywords: "Linear", "ZOH", "1", "" → all 4 produce error "invalid interp keyword" → AC11, AC20 | | | |
| T9 | Multi-sensor accumulation: (a) 3 sensors nsample=3,5,2 (dims 1,1,3) → `historyadr==[0,8,20]`, `nhistory==30`; (b) nsample=3,0,5 (dim=1) → `historyadr==[0,-1,8]`, `nhistory==20`; verify Vec lengths==3 → AC12, AC13, AC22 | | | |
| T10 | Full combination: framepos nsample=5 interp="cubic" delay=0.02 interval=0.1 (dim=3) → all 5 fields + `nhistory==22` → AC14 | | | |
| T11 | Actuator stability: 2 actuators (nsample=4,2) + 2 sensors (nsample=3,5 dim=1) → `act_historyadr==[0,10]`, `sens_historyadr==[16,24]`, `nhistory==36` → AC15 | | | |
| T12 | Negative values: (a) nsample=-1 → stored as -1, historyadr=-1, nhistory=0; (b) nsample=5 delay=-0.01 → delay stored, no error → AC16, AC17 | | | |
| T13 | Attrs without nsample: (a) interval=0.5 only → stored, historyadr=-1; (b) interp="linear" only → stored, historyadr=-1 → AC18, AC19 | | | |
| T14 | Sensor-only model (no actuators): framepos nsample=3 (dim=3) → `historyadr[0]==0`, `nhistory==14`, `actuator_historyadr` empty → AC21 | | | |
| T15 | Build regression: existing model with sensors (no history attrs) still builds → all new Vecs have correct length, `nhistory==0` for sensors | | | |
| T16 | Mixed actuator+multi-dim sensor: motor nsample=4 (dim=1, contrib=10) + framepos nsample=3 (dim=3, contrib=14) → `nhistory==24` → AC24 | | | |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Test(s) / AC(s) | Tested? | Test Function | Notes |
|-----------|----------|-----------------|---------|---------------|-------|
| `nsample=0` → no history buffer | Default case; must not allocate | T5 / AC8 | | | |
| Negative `nsample=-1` → accepted | MuJoCo stores as-is; treated as no history | T12 / AC16 | | | |
| `delay > 0` without `nsample > 0` → error | MuJoCo validation — delay requires history buffer | T6 / AC9 | | | |
| Negative `delay=-0.01` → accepted | MuJoCo accepts negative delay (validation is strictly `> 0.0`) | T12 / AC17 | | | |
| `interval < 0` → error | MuJoCo rejects negative interval | T7 / AC10 | | | |
| `interval > 0` without `nsample` → accepted | interval is independent of history buffer | T13 / AC18 | | | |
| `interp` stored without `nsample` | MuJoCo stores interp even when nsample=0 | T13 / AC19 | | | |
| `interp` keyword mapping: zoh→0, linear→1, cubic→2 | Correct enum mapping from string | T1, T10 / AC2, AC14 | | | |
| Case-sensitive interp keywords | "Linear"/"ZOH"/"1" all rejected | T8 / AC11 | | | |
| Empty-string `interp=""` → error | Distinct from mis-cased keyword | T8 / AC20 | | | |
| `nsample=0` in middle of chain | Must not break offset accumulation | T9 / AC13 | | | |
| Multi-dim sensor nhistory formula (dim=1, dim=3, dim=4) | Different dims produce different buffer sizes; formula generality | T3 / AC5, AC6 | | | |
| `interval` attribute parsed and stored | New attribute not in umbrella original scope | T2 / AC4 | | | |
| Default values when no attrs specified | All 6 fields must have correct defaults | T5 / AC8 | | | |
| Mixed actuator + sensor layout (dim=1) | Sensors start after actuator total | T4 / AC7 | | | |
| Mixed actuator + multi-dim sensor (dim=3) | Verifies generalized formula in mixed layout | T16 / AC24 | | | |
| Sensor-only model (no actuators) | Sensors start at offset 0 | T14 / AC21 | | | |
| Full combination (all 4 attrs) | All attributes work together on one sensor | T10 / AC14 | | | |
| Actuator stability guarantee | Adding sensors must NOT shift actuator offsets | T11 / AC15 | | | |
| `delay + interval` without nsample | Interval does NOT override nsample requirement for delay | T6 / AC23 | | | |
| Vec lengths match nsensor | All new arrays aligned with existing sensor arrays | T9, T15 / AC22 | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Old Behavior | New Behavior | Actually Happened? | Notes |
|-----------------|-------------|-------------|-------------------|-------|
| `nhistory` includes sensors | Actuator contributions only | Actuator + sensor contributions | | |
| MJCF `nsample`/`interp`/`delay`/`interval` on sensors parsed and stored | Silently ignored — attrs not recognized on sensor elements | Parsed and stored in model; sensors get history buffers | | |
| Delay/nsample validation on sensors | No validation — `delay > 0` without `nsample` accepted silently | `delay > 0.0 && nsample <= 0` → error | | |
| Negative interval validation on sensors | No validation — negative `interval` accepted silently | `interval < 0.0` → error | | |

### Files Affected: Predicted vs Actual

| Predicted File | Change Description (from spec) | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------------------|-------------------|------------------------|
| `sim/L0/mjcf/src/types.rs` | Add 4 fields to `MjcfSensor`, update default, add 4 fluent methods (+30 lines) | | |
| `sim/L0/mjcf/src/parser.rs` | Parse 4 attrs in `parse_sensor_attrs()` (+4 lines) | | |
| `sim/L0/mjcf/src/builder/mod.rs` | Add 4 fields to `ModelBuilder` (+4 lines) | | |
| `sim/L0/mjcf/src/builder/init.rs` | Init 4 new fields to `vec![]` (+4 lines) | | |
| `sim/L0/mjcf/src/builder/sensor.rs` | Validate + push 4 history attrs, update doc comments (+25 lines) | | |
| `sim/L0/mjcf/src/builder/build.rs` | Transfer 4 fields + init historyadr + extend post-hoc computation (+20/~15 mod) | | |
| `sim/L0/core/src/types/model.rs` | Add 5 fields, update nhistory comment (+20 lines) | | |
| `sim/L0/core/src/types/model_init.rs` | Init 5 new fields to `vec![]` (+5 lines) | | |
| `sim/L0/core/src/types/data.rs` | Update doc comment (line 78) (~1 line) | | |
| `sim/L0/core/src/types/enums.rs` | Update doc comment (lines 253–254) (~2 lines) | | |
| `sim/L0/tests/integration/sensor_phase6.rs` | 16 new tests (+400 lines) | | |

{Unexpected files changed — to be filled during review execution:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Existing Test Impact: Predicted vs Actual

| Test Group | File / Count | Predicted Impact | Reason | Actual Impact | Surprise? |
|-----------|-------------|-----------------|--------|---------------|-----------|
| Sensor defaults tests | `defaults.rs` (2 tests: `test_apply_to_sensor`, `test_sensor_defaults_inheritance`) | Pass (unchanged) | History attrs not added to defaults per AD-3(b); `apply_to_sensor()` and `merge_sensor_defaults()` untouched | | |
| Sensor pipeline tests | Various (`sensor_phase6.rs`, `sensors_phase4.rs`, `mjcf_sensors.rs`) | Pass (unchanged) | New fields are additive; existing sensors get default values (nsample=0, interp=Zoh, delay=0.0, interval=(0.0,0.0), historyadr=-1) | | |
| Phase 5 actuator tests | `actuator_phase5.rs` (27 tests) | Pass (unchanged) | Actuator historyadr loop unchanged; sensor loop appended after. Actuator offsets not shifted. | | |
| Phase 6 Spec A+B+C tests | `sensor_phase6.rs` (55+ tests) | Pass (unchanged) | Parser changes add new attributes (don't modify existing). Builder changes add pushes inside existing block. Evaluation untouched (parse-and-store only). | | |
| Full sim domain | Various (2,238+ tests) | Pass (unchanged) | No evaluation-level changes; parse-and-store only. `make_data()` auto-grows `history` buffer with increased `nhistory`. | | |

**Unexpected regressions:**
{To be filled during review execution.}

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `sensor_history` → unpacked `sensor_nsample` + `sensor_interp` | Unpack: `sensor_history[2*i+0]` → `sensor_nsample[i]`, `sensor_history[2*i+1]` → `sensor_interp[i]` as `InterpolationType`. Mirrors actuator pattern at `model.rs:592–596`. | | |
| `sensor_interval` → `Vec<(f64, f64)>` tuple | `sensor_interval[2*i+0]` → `.0` (period), `sensor_interval[2*i+1]` → `.1` (phase). Matching `actuator_actrange: Vec<(f64, f64)>` precedent. | | |
| `nsample` type: `i32` (signed) | Direct port — signed to accept negative values per EGT-5. Matches `actuator_nsample: Vec<i32>` at `model.rs:592`. | | |
| `interp` int→enum: `InterpolationType` shared with actuators | `FromStr` at `enums.rs:267–278`: case-sensitive lowercase only. Shared, NOT duplicated. | | |
| `interp` keyword: case-sensitive lowercase only | `"zoh"` → 0, `"linear"` → 1, `"cubic"` → 2. `FromStr` impl already correct — no case-insensitive parsing. | | |
| MJCF attribute names: `nsample`, `interp`, `delay`, `interval` | Direct port — attribute names match MuJoCo exactly, no snake_case variants. | | |
| `MjcfSensor` field pattern: `Option<T>` (not value-sentinel) | Use `Option::unwrap_or(default)` in builder, not value-sentinel comparison. Differs from `noise: f64`/`cutoff: f64` pattern — documented as acceptable mixed pattern (rubric P8). | | |
| Field naming: `sensor_` prefix per umbrella §3.1 | All new fields prefixed `sensor_` — `sensor_nsample`, `sensor_interp`, `sensor_historyadr`, `sensor_delay`, `sensor_interval`. Direct port with unpacking (history → nsample + interp). | | |
| Sensor defaults: NOT extended for history attrs | History attrs parsed from sensor elements only. `apply_to_sensor()` (`defaults.rs:533`) untouched. `merge_sensor_defaults()` (`defaults.rs:815`) untouched. `parse_sensor_defaults()` (`parser.rs:909`) untouched. MuJoCo has NO sensor default class mechanism (EGT-11h). | | |
| Error message parity: behavior match, not text match | Conformance requires matching validation BEHAVIOR (what is accepted/rejected), not error message phrasing. Parse-time error messages have no message-text conformance requirement with MuJoCo. | | |

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
| | | | | |

{To be filled during review execution.}

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

Every item that was in the spec's scope but not fully implemented, plus
anything discovered during implementation that's out of scope. **The goal:
nothing deferred is untracked.**

For each item, verify it appears in at least one of:
- `sim/docs/todo/` (future_work file or spec_fleshout)
- `sim/docs/ROADMAP_V1.md`
- The umbrella spec's Out of Scope section (if applicable)

If it's not tracked anywhere, it's a review finding — add tracking now.

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Runtime sensor interpolation — `nsample`/`interp`/`delay` runtime behavior (reading from history buffer during `mj_forward`) | Out of Scope, bullet 1 | | DT-107/DT-108 | |
| Sensor history pre-population in `reset_data()` — MuJoCo pre-populates actuator history in `mj_resetData`; sensor pre-population follows same pattern but is runtime scope | Out of Scope, bullet 2 | | | |
| Adding history attrs to `MjcfSensorDefaults` — MuJoCo has no sensor default class mechanism (EGT-11h); CortenForge extension left as-is | Out of Scope, bullet 3 | | | |
| `sensor_intprm` array (`mjmodel.h:1213`) — separate from `sensor_history`; used for user/plugin sensor integer params; not needed for standard sensor types | Out of Scope, bullet 4 | | | |
| DT-65 (`User` sensor `dim` attribute) — deferred to Post-v1.0 | Out of Scope, bullet 5 | | DT-65 | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

{To be filled during review execution.}

---

## 9. Test Coverage Summary

Quick sanity check on test health after the implementation.

**Domain test results:**
```
{To be filled during review execution.}
```

**New tests added:** {count}
**Tests modified:** {count}
**Pre-existing test regressions:** {count — should be 0}

**Clippy:** {clean / N warnings — list them}
**Fmt:** {clean / issues}

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
1. {To be filled during review execution.}

**Items tracked for future work:**
1. {To be filled during review execution.}
