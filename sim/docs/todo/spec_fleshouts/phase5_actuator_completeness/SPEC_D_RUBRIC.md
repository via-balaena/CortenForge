# Spec D — Interpolation Actuator Attributes: Spec Quality Rubric

Grades the SPEC_D spec on 10 criteria. Target: A+ on every criterion before
implementation begins. A+ means "an implementer could build this without asking
a single clarifying question — and the result would produce numerically identical
output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
P2–P10 but has P1 wrong is worse than useless: it would produce a clean,
well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Empirical Ground Truth

The following was established by running MuJoCo 3.5.0 Python bindings and is
the authoritative reference for this rubric and the spec it grades. Any claim
in the spec that contradicts these results is wrong.

### Attribute parsing

- `interp` is a **string keyword** in MJCF: `"zoh"`, `"linear"`, `"cubic"`
  (lowercase only). Maps to integer 0, 1, 2 in `actuator_history[i, 1]`.
  Any other string (including `"ZOH"`, `"Linear"`, `"1"`) is an XML error.
- `nsample` is an integer. Default: 0. MuJoCo accepts negative values
  (e.g., `nsample="-1"` compiles with `actuator_history=[[-1, 0]]`,
  `historyadr=[-1]`) — no validation.
- `delay` is a float in seconds. Default: 0.0.

### Compiler validation

- **`delay > 0` with `nsample = 0`** → compile error:
  `"Error: setting delay > 0 without a history buffer"`.
- **`nsample > 0` with `delay = 0`** → valid (history buffer allocated,
  no delay applied).
- No validation on `nsample` sign or `interp` out-of-range (handled by
  keyword parsing — invalid keywords are XML errors, not compiler errors).
- **No validation on `interp="cubic"` with `nsample < 4`** — silently accepted
  (nsample=1,2,3 all work with cubic). Runtime behavior unspecified for
  insufficient sample points.
- **No validation on delay exceeding buffer capacity** — e.g., `delay=0.1` with
  `nsample=2, timestep=0.002` (buffer covers ~0.004s, delay asks for 0.1s
  lookback) is silently accepted.

### Compiled model layout

- `actuator_history`: `int[nu × 2]` — `[i, 0]` = nsample, `[i, 1]` = interp.
  **Always `nu×2` for ALL actuators** — even actuators with `nsample=0` get a
  row (storing `[0, interp_value]`). `interp` can be set independently of
  `nsample` (e.g., `interp="cubic"` with `nsample=0` stores `[0, 2]` — no error).
- `actuator_historyadr`: `int[nu × 1]` — cumulative offset into `Data.history`.
  Equals `-1` when `nsample <= 0` for that actuator.
- `actuator_delay`: `mjtNum[nu × 1]` — always present for all actuators
  (default 0.0, even when no history).

### History buffer allocation

- **Stride per actuator**: `2 * nsample + 2` mjtNums per actuator with
  `nsample > 0`.
- **Buffer layout per actuator** (verified empirically via `mj_initCtrlHistory`):
  `[metadata0, metadata1, time_0, time_1, ..., time_{n-1}, value_0, value_1, ..., value_{n-1}]`
  — 2 metadata slots, then `nsample` time stamps, then `nsample` **ctrl** values.
  NOT interleaved `(time, value)` pairs. **Stores ctrl, not act** — verified with
  `dyntype=integrator` actuator: after setting `ctrl=1.0` and stepping, history
  stores `value=1.0` (= ctrl) while `act=0.0` (activation stays at 0).
- **`nhistory`** = Σ `(2 * nsample_i + 2)` across all actuators (and sensors)
  with `nsample_i > 0`.
- **`historyadr[i]`** = cumulative sum of preceding allocations.

**Stride verification data (MuJoCo 3.5.0, exhaustive):**

| nsample | nhistory | 2n+2 | Match |
|---------|----------|------|-------|
| 1 | 4 | 4 | ✓ |
| 2 | 6 | 6 | ✓ |
| 3 | 8 | 8 | ✓ |
| 4 | 10 | 10 | ✓ |
| 5 | 12 | 12 | ✓ |
| 6 | 14 | 14 | ✓ |
| 7 | 16 | 16 | ✓ |
| 8 | 18 | 18 | ✓ |
| 10 | 22 | 22 | ✓ |
| 16 | 34 | 34 | ✓ |
| 32 | 66 | 66 | ✓ |

**Multi-actuator historyadr verification:**

| Model | nsample | historyadr | nhistory |
|-------|---------|-----------|----------|
| 1 act, ns=4 | [4] | [0] | 10 |
| 2 act, ns=3,5 | [3,5] | [0,8] | 20 |
| 3 act, ns=2,4,6 | [2,4,6] | [0,6,16] | 30 |
| 1 act, ns=1 | [1] | [0] | 4 |
| 2 act, ns=4,0 | [4,0] | [0,-1] | 10 |
| 3 act, ns=3,0,2 | [3,0,2] | [0,-1,8] | 14 |

Formula verified: `historyadr[j] = Σ_{i<j} (2*ns_i + 2)` for `ns_i > 0`.
Skipped actuators (`ns <= 0`) get `historyadr = -1` and do not consume buffer space.

### Initial history buffer state

MuJoCo pre-populates the history buffer when `MjData` is created (and restores
this state on `mj_resetData`). It does NOT zero-initialize.

**Per-actuator buffer initial state** (nsample=`n`, timestep=`ts`):
- `metadata0` = 0.0
- `metadata1` = float(nsample - 1)
- `times` = `[-(n)*ts, -(n-1)*ts, ..., -ts]` — `n` evenly-spaced past
  timestamps ending at `-ts` (NOT at 0).
- `values` = `[0.0, 0.0, ..., 0.0]` — all zero.

**Verification data (MuJoCo 3.5.0):**

| nsample | timestep | meta0 | meta1 | times | values |
|---------|----------|-------|-------|-------|--------|
| 4 | 0.002 | 0.0 | 3.0 | [-0.008, -0.006, -0.004, -0.002] | [0, 0, 0, 0] |
| 4 | 0.01 | 0.0 | 3.0 | [-0.04, -0.03, -0.02, -0.01] | [0, 0, 0, 0] |
| 2 | 0.002 | 0.0 | 1.0 | [-0.004, -0.002] | [0, 0] |
| 1 | 0.002 | 0.0 | 0.0 | [-0.002] | [0] |

**Multi-actuator initial state verified** (ns=3 + ns=2, ts=0.002):
- Act 0 (adr=0): meta=[0.0, 2.0], times=[-0.006, -0.004, -0.002], values=[0,0,0]
- Act 1 (adr=8): meta=[0.0, 1.0], times=[-0.004, -0.002], values=[0,0]
- Each actuator's times are computed independently from its own nsample.

**`mj_resetData` restores this state** — confirmed with `np.allclose` against
freshly-created `MjData`, including after 10 steps of mutation on a
multi-actuator model. `mj_resetData` does NOT just zero the buffer.

**`mj_resetDataKeyframe` also restores this state** — keyframe reset produces
identical history buffer to `mj_resetData` (verified with `np.allclose`).
CortenForge's `Data::reset_to_keyframe()` (`data.rs:964`) will need the same
history restoration logic as `Data::reset()`.

**`mj_step` writes to the buffer** — circular buffer behavior observed.
`metadata1` cycles through 0..nsample-1 as the write-head advances. The times
and values arrays are overwritten with current simulation time and ctrl values.

**Metadata slot summary across nsample 1–6:**

| nsample | meta0 | meta1 |
|---------|-------|-------|
| 1 | 0.0 | 0.0 |
| 2 | 0.0 | 1.0 |
| 3 | 0.0 | 2.0 |
| 4 | 0.0 | 3.0 |
| 5 | 0.0 | 4.0 |
| 6 | 0.0 | 5.0 |

### General stride formula

The actuator stride formula `2 * nsample + 2` is a special case of the general
formula for dim=1 channels:

**General stride** = `nsample * (dim + 1) + 2`

For actuators, dim=1 (single ctrl value), so stride = `nsample * 2 + 2`.
For sensors, dim varies. Buffer layout:
`[meta0, meta1, time_0..time_{n-1}, val_{0,0}..val_{n-1,dim-1}]`
= 2 + nsample + nsample * dim = 2 + nsample * (dim + 1).

**Sensor stride verification (MuJoCo 3.5.0):**

| sensor type | dim | nsample | nhistory | ns*(dim+1)+2 | Match |
|-------------|-----|---------|----------|--------------|-------|
| jointpos | 1 | 3 | 8 | 8 | ✓ |
| framepos | 3 | 4 | 18 | 18 | ✓ |
| framequat | 4 | 3 | 17 | 17 | ✓ |
| framequat | 4 | 5 | 27 | 27 | ✓ |

Sensor history is out of scope for this spec. The general formula is documented
here so the actuator stride formula (`2n+2`) is understood as the dim=1 case,
and future sensor history implementation has a verified starting point.

**`nhistory` scope caveat:** MuJoCo's `nhistory` includes BOTH actuator AND
sensor contributions. CortenForge does not currently parse sensor `nsample`.
For a model with actuator history (ns=3) AND sensor history (ns=4, dim=1),
MuJoCo reports `nhistory=18` (8+10), but CortenForge's actuator-only
computation would yield `nhistory=8`. **This is a known conformance gap** for
models that use sensor history. The spec must acknowledge this and document
how/when it will be resolved (e.g., when sensor history is implemented).
Ordering in the shared buffer: actuators first, then sensors
(`actuator_historyadr[0]=0`, `sensor_historyadr[0]=8` in the example above).

### Default class inheritance

- `nsample`, `interp`, `delay` are inheritable through `<default>` classes.
- Child elements can override individual attributes (e.g., inherit
  `interp="linear"` and `delay=0.02` from class, override `nsample=3`).

### Actuator type compatibility

- Verified on: `<position>`, `<motor>`, `<velocity>`, `<general>` — all accept
  `nsample`/`interp`/`delay`.
- `<muscle>` and `<adhesion>` failed in testing but due to pre-existing issues
  (lengthrange convergence, control range requirements) unrelated to history
  attributes. The attributes themselves are accepted on any actuator type.

### Forward dynamics interaction

**`mj_forward` READS from history** to compute delayed ctrl. With a delayed
actuator (delay=0.006, kp=100), setting `ctrl=1.0` on a fresh `MjData`
(history values all 0.0) and calling `mj_forward` produces
`actuator_force=[0.]` — the delayed value from the history buffer, NOT the
current ctrl. Without delay, the same ctrl produces `force=[100.]`.

This means the pre-populated initial buffer state **directly affects forward
dynamics from the very first call**. The initial buffer times and values are
not just a data structure detail — they determine what the simulation "sees"
as the delayed ctrl signal.

### CortenForge builder integration notes

The following was established by reading the CortenForge codebase and is
relevant for the spec to address:

- **Parser attribute gating**: `parser.rs:2069-2076` gates `gaintype`/`gainprm`/
  `biastype`/`biasprm`/`dyntype`/`dynprm` parsing to `MjcfActuatorType::General`
  only. `nsample`/`interp`/`delay` apply to ALL actuator types and must be parsed
  in the **common** attribute section (lines 2013-2052), NOT inside the general
  gate. Placing them in the wrong section is a silent bug — shortcuts would
  silently ignore these attributes.
- **Timestep location**: `model.timestep` (model.rs:682), not `model.opt.timestep`.
- **Builder has a 3-file pipeline**:
  (a) `builder/mod.rs:525-569` — `ModelBuilder` struct holds intermediate arrays
      (e.g., `actuator_trntype: Vec<ActuatorTransmission>`). New fields must be
      added here.
  (b) `builder/actuator.rs:15-456` — `process_actuator()` pushes per-actuator
      values to ModelBuilder arrays (e.g., `self.actuator_gear.push(...)`).
      New attributes pushed here.
  (c) `builder/build.rs:244-262` — `build()` moves arrays from ModelBuilder to
      final Model (e.g., `actuator_gear: self.actuator_gear`). New fields must
      be transferred here. **`historyadr` and `nhistory` computation** must happen
      in `build()` (around lines 382-386) AFTER all actuators are processed,
      since historyadr is cumulative across actuators.
- **Defaults has TWO functions** that need updating:
  (a) `merge_actuator_defaults()` (`defaults.rs:743-773`) — merges parent/child
      class defaults using `c.field.or(p.field)` pattern. New fields added here.
  (b) `apply_to_actuator()` (`defaults.rs:294-402`) — applies resolved defaults
      to an actuator using `if result.field.is_none() { ... }` pattern. New fields
      added here.
- **Test entry point**: `sim_mjcf::load_model(&mjcf)` in integration tests.
  Floating-point comparisons use `approx::assert_relative_eq!()`.
  Existing Phase 5 tests in `sim/L0/tests/integration/actuator_phase5.rs`.

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. MuJoCo conformance is the entire reason CortenForge exists.
> A spec can be A+ on P2–P10 and still be worthless if P1 is wrong — because
> an incorrect MuJoCo reference means every algorithm, every AC, and every
> test is verifying the wrong behavior. **Grade P1 first and grade it hardest.**
> If P1 is not A+, do not proceed to grading P2–P10 until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving. This is the
> single most important criterion: everything else in the spec is downstream
> of getting the MuJoCo reference right.

| Grade | Bar |
|-------|-----|
| **A+** | Spec cites the exact MuJoCo structures and fields with source files and line ranges: (1) `mjsActuator_` in `mjspec.h` — `nsample` (int, line 701), `interp` (int, line 702), `delay` (double, line 703), with exact C type and semantics; (2) `mjModel` in `mjmodel.h` — `actuator_history` (int, `nu×2`, line 1184: stores `[nsample, interp]` packed, present for ALL actuators including those with `nsample=0`), `actuator_historyadr` (int, `nu×1`, line 1185: cumulative offset into history buffer, -1 when no history), `actuator_delay` (mjtNum, `nu×1`, line 1186: present for ALL actuators, default 0.0); (3) `mjData` in `mjdata.h` — `history` buffer (mjtNum, `nhistory×1`, line 259), `nhistory` size field (mjmodel.h line 764). (4) Runtime API functions cited: `mj_readCtrl()`, `mj_initCtrlHistory()` in `mujoco.h` — documented as **out of scope** (runtime deferred), but cited to show what the stored attributes are consumed by. **MJCF keyword parsing documented:** `interp` takes **string keywords** `"zoh"`, `"linear"`, `"cubic"` (lowercase only) in XML, mapping to integers 0, 1, 2 in the compiled model — NOT integer attributes. Spec must cite this with verification evidence. **Compiler validation documented:** `delay > 0` with `nsample = 0` is a compile error (`"Error: setting delay > 0 without a history buffer"`); `nsample > 0` with `delay = 0` is valid; `nsample` negative values are silently accepted (`historyadr = -1`). **History buffer stride documented:** each actuator with `nsample > 0` occupies `2 * nsample + 2` mjtNums in the history buffer; layout is `[metadata0, metadata1, time_0..time_{n-1}, value_0..value_{n-1}]` — 2 metadata slots, then `nsample` timestamps, then `nsample` **ctrl** values (stores ctrl, NOT act — verified with dyntype=integrator actuator; verified layout via `mj_initCtrlHistory` — NOT interleaved pairs). **`historyadr` computation documented:** `historyadr[j] = Σ_{i<j} (2 * nsample_i + 2)` for actuators with `nsample_i > 0`, otherwise -1. **`nhistory` computation documented:** `nhistory = Σ (2 * nsample_i + 2)` across all actuators and sensors with `nsample_i > 0`. **Initial buffer state documented:** `make_data()` / `mj_resetData()` pre-populates the history buffer — NOT zero-initialized. Per-actuator: meta0 = 0.0, meta1 = float(nsample - 1), times = `[-(nsample)*ts, ..., -ts]` (evenly spaced ending at -timestep, NOT at 0), values = all 0.0. `reset()` restores this state (verified: `mj_resetData` output matches fresh `MjData`). Spec documents the exact pre-population formula with at least two verification data points at different timestep values. `mj_step` writes to the buffer as a circular buffer — documented as runtime concern, deferred, but noted so the initial state makes sense (the buffer is primed for immediate runtime use). **General stride formula documented:** the actuator stride `2*nsample+2` is a special case of `nsample*(dim+1)+2` for dim=1; sensor stride verified for dim=1,3,4 — explicitly out of scope but noted for future work. **Sensor parallel documented:** `mjsSensor_` has identical attributes (mjspec.h:731-733) and model arrays (mjmodel.h:1218-1220) — explicitly **out of scope** but noted for future work. **Default class inheritance documented:** all three attributes are inheritable; child override semantics verified. **`nhistory` conformance caveat documented:** MuJoCo's `nhistory` includes both actuator and sensor contributions; CortenForge's actuator-only computation will produce smaller `nhistory` for models with sensor history — spec documents this as a known gap and how it will be resolved when sensor history is implemented. **Additional non-validated edge cases documented:** `interp="cubic"` with `nsample < 4` (silently accepted by MuJoCo — no minimum nsample for cubic); delay exceeding buffer time capacity (silently accepted). **Forward dynamics interaction documented:** `mj_forward` READS from history to apply delayed ctrl — the pre-populated initial state directly affects forward dynamics from the first call (verified: delayed actuator with fresh MjData produces force=0 despite ctrl=1.0). This makes the initial buffer pre-population functionally important, not just a data layout detail. **Keyframe reset documented:** `mj_resetDataKeyframe` restores the same pre-populated initial history state as `mj_resetData` — CortenForge's `Data::reset_to_keyframe()` must do the same. Edge cases documented: `nsample = 0` (no history, `historyadr = -1`), `nsample > 0` with `delay = 0` (valid — buffer allocated, no delay), `delay > 0` with `nsample = 0` (compile error), `nsample` negative (accepted silently, `historyadr = -1`). Empirical verification table present with at least 3 models showing `historyadr` and `nhistory` match the stride formula. |
| **A** | All structures and fields described correctly from C source. Stride formula and historyadr computation present but without empirical verification data. |
| **B** | Correct at high level. Missing specifics: `interp` described as integer attribute (wrong — it's a keyword), historyadr described as "always -1" or "deferred", stride formula missing. |
| **C** | Partially correct. Attributes assumed from docs rather than C source. |

### P2. Algorithm Completeness

> Every step of MJCF parsing, model compilation, and data allocation is
> specified unambiguously. No "see MuJoCo source" or "TBD" gaps. Rust code
> is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | Six steps fully specified in Rust: (1) **MJCF type additions** — `MjcfActuator` struct gains `nsample: Option<i32>`, `interp: Option<String>` (keyword string, not integer), `delay: Option<f64>` fields; `MjcfActuatorDefaults` struct gains matching `Option` fields; XML attribute extraction code shown for both actuator parsing and defaults parsing (`parse_actuator_defaults` in `parser.rs`). **Critical**: parser must add these attributes to the **common** attribute section (`parser.rs:2013-2052`), NOT inside the `<general>`-only gate (`parser.rs:2069-2076`) — they apply to all actuator types. **Defaults pipeline**: both `merge_actuator_defaults()` (`defaults.rs:743-773`, merges parent/child class defaults using `c.field.or(p.field)` pattern) AND `apply_to_actuator()` (`defaults.rs:294-402`, applies resolved defaults to actuator using `if result.field.is_none() { ... }` pattern) must be updated with the three new fields; (2) **Model type additions** — `Model` struct gains history fields (design decision on packed vs split documented with rationale), `actuator_historyadr: Vec<i32>`, `actuator_delay: Vec<f64>`, and `nhistory: usize`; `model_init.rs` initializes appropriately; `ModelBuilder` struct (`builder/mod.rs:525-569`) gains intermediate array fields to carry parsed values through the builder pipeline; `InterpolationType` enum added to `enums.rs` (`Zoh`, `Linear`, `Cubic` variants) with `FromStr` (parses MJCF keyword strings) and `From<i32>` (converts MuJoCo integers) — placed in `enums.rs` alongside existing `ActuatorDynamics`/`ActuatorTransmission`; (3) **Builder actuator compilation** — `process_actuator()` (`builder/actuator.rs:15-456`) extracts parsed attributes from `MjcfActuator`, applies defaults through class hierarchy (`MjcfActuatorDefaults`), maps `interp` keyword string → integer/enum, pushes to ModelBuilder arrays (following existing push pattern at lines 211-262, 442-453); (4) **`historyadr` computation** — `build()` method (`builder/build.rs`) transfers arrays from ModelBuilder to Model (lines 244-262) and then computes cumulative `historyadr` offsets using `2 * nsample + 2` stride for each actuator with `nsample > 0`, assigns -1 for `nsample <= 0`, computes total `nhistory`. **Must happen in `build()` as a post-processing step** (around line 386, after `compute_actuator_params()`) since historyadr is cumulative across all actuators and cannot be computed until all actuators are processed; (5) **Compiler validation** — builder rejects `delay > 0` with `nsample = 0` (matching MuJoCo's compile error); (6) **Data integration** — `Data` struct gains `history: Vec<f64>` field; `Model::make_data()` in `model_init.rs` **pre-populates** the history buffer (NOT just `vec![0.0; nhistory]`) — for each actuator with `nsample > 0`, writes: meta0=0.0, meta1=float(nsample-1), times=`[-(nsample)*ts, ..., -ts]`, values=all 0.0 (matching MuJoCo's `MjData` initial state); `Data::reset()` in `data.rs` restores this same pre-populated initial state (NOT just zeroing — `mj_resetData` restores the pre-populated times); `Data::reset_to_keyframe()` in `data.rs` also restores this same pre-populated state (verified: `mj_resetDataKeyframe` produces identical history to `mj_resetData`); `impl Clone for Data` includes the new field (Data has a manual Clone impl — every field must be listed explicitly); the `data_reset_field_inventory` staleness guard test (`data.rs:1026`) has its `EXPECTED_SIZE` constant updated to reflect the new field. Real Rust code for every step — no pseudocode. **`InterpolationType` enum:** spec defines a Rust enum (`Zoh`, `Linear`, `Cubic`) with `FromStr`/`From<i32>` conversion. |
| **A** | All six steps present. Minor details left implicit (e.g., exact error message wording for validation, or Data::reset restoration strategy not specified). |
| **B** | Steps described at high level. Missing historyadr computation or validation logic. Historyadr described as "all -1" or "deferred." |
| **C** | Skeleton only — "add fields to Model." |

### P3. Convention Awareness

> Spec explicitly addresses MuJoCo → CortenForge convention differences and
> provides correct translations.

| Grade | Bar |
|-------|-----|
| **A+** | Convention table covers: (1) `actuator_history` packing — MuJoCo `int[nu×2]` flat with `[2*i+0]=nsample`, `[2*i+1]=interp`; CortenForge decision documented (packed `[i32; 2]` or split `Vec<i32>` + `Vec<InterpolationType>`) with porting rule and rationale; (2) `actuator_historyadr` — MuJoCo `int[nu×1]` with `-1` sentinel for no-history; CortenForge uses `Vec<i32>` (signed, matching MuJoCo sentinel) or `Vec<Option<usize>>` with explicit mapping; (3) `actuator_delay` — MuJoCo `mjtNum[nu×1]` → CortenForge `Vec<f64>` (direct); (4) `interp` attribute — MuJoCo MJCF takes string keywords `"zoh"`/`"linear"`/`"cubic"`, compiled model stores integers 0/1/2; CortenForge parses strings, stores `InterpolationType` enum; (5) `nhistory` — MuJoCo `mjtSize` on mjModel; CortenForge `usize` on Model; (6) MJCF attribute naming matches MuJoCo exactly per Convention Registry §4; (7) Rust field naming uses snake_case per Convention Registry §1; (8) Enum default convention — `#[default]` attribute on `Zoh` variant per Convention Registry §3. **Umbrella convention conflicts flagged:** the umbrella's Convention Registry §1 suggests `Vec<usize>` for `nsample` (unsigned — but MuJoCo uses signed `int` and accepts negative values), and `InterpType::None` as the default variant name (but the default is ZOH/zero-order-hold, not "none" — `None` is misleading). Spec must document whether to correct these or follow the umbrella, with rationale. Each porting rule explicit. |
| **A** | Major conventions documented. Minor inconsistencies left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch. |
| **C** | MuJoCo code pasted without adaptation to our conventions. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> model configurations, and MuJoCo-verified expected values.

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has the three-part structure: (1) concrete MJCF input, (2) exact expected value **verified against MuJoCo**, (3) field to check. ACs cover: (a) parsing each attribute with explicit values and verifying Model fields match MuJoCo output — at least one AC per attribute with MuJoCo-verified reference values from the Empirical Ground Truth table; (b) `historyadr` computation — multi-actuator model with mixed nsample values, expected `historyadr` values verified against MuJoCo (e.g., `[0, 8]` for `nsample=[3, 5]`); (c) `nhistory` computation — expected value verified against MuJoCo; (d) `Data.history` allocation — buffer length equals `nhistory`; (e) `Data.history` initial state — pre-populated metadata and times verified against MuJoCo (e.g., for nsample=4, timestep=0.002: meta0=0.0, meta1=3.0, times=[-0.008, -0.006, -0.004, -0.002], values=[0,0,0,0]); (f) default class inheritance — class with `nsample=8`/`interp="linear"`/`delay=0.02`, child overrides `nsample=3`, verify both actuators; (g) compiler validation — `delay > 0` with `nsample = 0` produces error; (h) `interp` keyword parsing — all three valid keywords produce correct integer, invalid keyword produces error; (i) default values when attributes omitted — `nsample=0`, `interp=0` (Zoh), `delay=0.0`, `historyadr=-1`; (j) `Data::reset()` restores the pre-populated initial state (not just zeros); (k) `Data::reset_to_keyframe()` also restores the same pre-populated initial state. Code-review ACs labeled as such: new fields present in `Model`, `model_init.rs`, `Data`, builder, Clone impl, staleness guard constant. |
| **A** | ACs are testable. Some lack MuJoCo-verified expected values (analytically derived only). |
| **B** | ACs directionally correct but vague ("fields should be stored correctly") or missing historyadr/nhistory verification. |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions. Each AC
> maps to at least one test. At least one test per major feature is a direct
> MuJoCo conformance test.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present — every AC maps to ≥1 test, every test maps to ≥1 AC or is in Supplementary Tests table with justification. Edge case inventory covers: `nsample=0` (no history — default, `historyadr=-1`), `nsample=1` (minimum valid, `nhistory=4`), `delay=0.0` with `nsample>0` (valid, buffer allocated), `delay>0` with `nsample=0` (compile error), all three `interp` keywords (`"zoh"`, `"linear"`, `"cubic"`), invalid `interp` keyword (error), multiple actuators with different nsample/interp/delay, attributes on different actuator shortcut types (motor, position, velocity, general — all accept these attributes), default class inheritance with partial override. **Conformance tests:** at least one test loads the same MJCF in MuJoCo and CortenForge, compares `actuator_history`, `actuator_historyadr`, `actuator_delay`, and `nhistory` values. **Data lifecycle tests:** `Data.history` buffer has correct length; initial state has correct pre-populated metadata and times (verified against MuJoCo values); `Data::reset()` restores the pre-populated initial state; `Data::reset_to_keyframe()` also restores the same pre-populated initial state (matching AC (k)). **Negative cases:** omitted attributes produce correct defaults; `nsample=0` produces `historyadr=-1`; `delay>0` without history is rejected. **Regression guard:** existing actuator tests still pass (new fields don't break existing construction). |
| **A** | Good coverage. Minor edge-case gaps (e.g., missing nsample=1 minimum or one interp keyword). |
| **B** | Happy path covered. Edge cases sparse. No MuJoCo conformance tests. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs stated.

| Grade | Bar |
|-------|-----|
| **A+** | States dependency on T1-b (§63 `dynprm` resize, commit `d4db634`) — already landed, no blocker. Notes Spec D is independent of Spec A, Spec B, and Spec C (per umbrella dependency graph — Step 2 parallel). States that `builder/actuator.rs` is a shared file but Spec D's changes (new attribute parsing in a separate block) do not conflict with other specs' additions. Notes Spec A and B have already landed — states their commit hashes so the spec is written against the current codebase state. Execution order is section-by-section with verification gates. Notes sensor history attributes are deferred (future work — documents the exact parallel with actuator history so future implementation is straightforward). |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description: (a) `sim/L0/mjcf/src/types.rs` — 3 new fields on `MjcfActuator`, 3 new fields on `MjcfActuatorDefaults`, `Default` impls updated; (b) `sim/L0/mjcf/src/parser.rs` — actuator parsing for 3 new attributes + `parse_actuator_defaults` updated for 3 new fields; (c) `sim/L0/mjcf/src/defaults.rs` — TWO functions updated: `merge_actuator_defaults()` (line 743, parent/child class merge) and `apply_to_actuator()` (line 294, resolved defaults → actuator application); (d) `sim/L0/core/src/types/model.rs` — new Vec fields for history/historyadr/delay/nhistory on `Model`; (e) `sim/L0/core/src/types/model_init.rs` — `Model::empty()` init for new fields + `Model::make_data()` allocates AND pre-populates `Data.history` buffer (metadata + times + values for each actuator with history); (f) `sim/L0/core/src/types/enums.rs` — new `InterpolationType` enum; (g) `sim/L0/core/src/types/data.rs` — new `history: Vec<f64>` field on `Data`, `Data::reset()` restores pre-populated initial state (not just zeros), `Data::reset_to_keyframe()` also restores pre-populated state, manual `impl Clone for Data` updated with new field, `data_reset_field_inventory` staleness guard test `EXPECTED_SIZE` constant updated; (h) `sim/L0/mjcf/src/builder/actuator.rs` — `process_actuator()` pushes new attribute values to ModelBuilder; (i) `sim/L0/mjcf/src/builder/mod.rs` — `ModelBuilder` struct gains new intermediate array fields; (j) `sim/L0/mjcf/src/builder/build.rs` — `build()` method transfers new arrays from ModelBuilder to Model AND computes `historyadr`/`nhistory` as a post-processing step (must happen after all actuators are processed, since historyadr is cumulative). **Risk note:** `parser.rs` has a `<general>`-only attribute gate (lines 2069-2076). The new attributes must be parsed in the common section (lines 2013-2052), not inside this gate. Placing them in the wrong section would cause shortcuts (position, motor, velocity) to silently ignore the attributes — a subtle conformance bug. **Behavioral change: additive only** — no existing behavior changes, no existing field values change. The only new runtime-visible change is `Data.history` buffer allocation and pre-population (never read until future runtime work). **Existing test impact: zero breakage expected** — new fields with empty vecs / zero defaults don't affect existing construction paths. Named specific test suites to verify: `cargo test -p sim-core -p sim-mjcf` (full domain, 2,148+ baseline). Backward-compat: all callers of `Model::empty()` and `Model::make_data()` unaffected (new fields initialize to defaults). **Note:** `Data` has a manual `impl Clone` (~60+ fields) — forgetting the new field is a silent bug. Spec must explicitly list the Clone line. No serde derives exist on `Model` or `Data`, so no serialization concern. |
| **A** | File list complete. Most impacts identified. |
| **B** | File list present but incomplete (e.g., missing `data.rs` or `parser.rs`). |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology
> throughout. Section references are accurate.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform: `nsample` (not `n_sample`, `num_samples`), `interp` (not `interpolation`, `interp_mode`), `delay` (not `time_delay`). Field names spelled identically in every section (Model fields, MJCF attributes, test assertions, convention table). AC numbers match between AC section, Test Plan, and Traceability Matrix. File paths in Specification sections match the Files Affected table. Edge cases in MuJoCo Reference appear in the Test Plan's Edge Case Inventory. Convention table entries consistent with Specification code snippets (same field names, same types, same defaults). The stride formula `2 * nsample + 2` appears consistently wherever historyadr or nhistory computation is described. The `interp` attribute is consistently described as a string keyword in MJCF and an integer/enum in the compiled model — no section accidentally treats it as an integer MJCF attribute. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Data Lifecycle Completeness *(domain-specific)*

> Every new model and data field has a complete lifecycle: declaration →
> initialization → population → consumption path documented. Fields that
> exist in MuJoCo's compiled model must exist in ours with correct values.

| Grade | Bar |
|-------|-----|
| **A+** | For each new field, spec traces the full lifecycle through all six phases: (1) **Declaration** — field type and doc comment in `model.rs` / `data.rs`; (2) **Initialization** — `Model::empty()` sets correct defaults for Model fields; `Model::make_data()` allocates AND **pre-populates** `Data.history` buffer (not just zero-allocates) — per-actuator: meta0=0.0, meta1=float(nsample-1), times=`[-(nsample)*ts, ..., -ts]`, values=all 0.0 (matching MuJoCo's initial `MjData` state, verified empirically); (3) **Population** — builder computes and stores correct values (including `historyadr` cumulative offsets and `nhistory` total), with exact code shown; (4) **Reset** — `Data::reset()` AND `Data::reset_to_keyframe()` both restore the pre-populated initial state (NOT just zeroing — `mj_resetData` and `mj_resetDataKeyframe` both restore pre-populated times, verified: both reset outputs match fresh make_data output via `np.allclose`); (5) **Clone** — `impl Clone for Data` (manual, not derived) includes the new `history` field; (6) **Future consumption** — spec documents the runtime consumers (`mj_forward` reads delayed ctrl from history, `mj_readCtrl()` analog, `mj_step` circular buffer writes) and states runtime is **deferred**, but the model/data fields are **correct now** so a future implementation needs no model-building changes. `historyadr` is computed correctly at build time (NOT deferred as -1 for all, NOT "placeholder"), matching MuJoCo's compiled model exactly. `nhistory` is computed. `Data.history` buffer is allocated to correct length AND pre-populated with correct initial state. The spec explicitly states: "After this spec lands, `Model` fields for history attributes will produce **identical values** to MuJoCo's `mjModel` for any MJCF model with actuator history attributes, and `Data.history` will have the same initial state as MuJoCo's `MjData`." No orphan fields, no dead allocations, no deferred computations that MuJoCo does at compile time. No missing Clone lines. |
| **A** | All fields have correct lifecycle. historyadr is computed but without proof of MuJoCo equivalence. |
| **B** | Fields declared and initialized but historyadr deferred or always -1. nhistory missing. |
| **C** | Fields added to struct without lifecycle documentation. |

### P10. Compiler Validation Completeness *(domain-specific)*

> Every validation rule MuJoCo applies during model compilation is documented
> and replicated. Missing a validation rule means accepting invalid models
> that MuJoCo rejects, which is a conformance bug.

| Grade | Bar |
|-------|-----|
| **A+** | Spec documents every MuJoCo compiler validation for these attributes and specifies equivalent CortenForge builder validation: (1) `delay > 0` with `nsample = 0` → error (verified empirically; exact MuJoCo error message cited); (2) `interp` keyword parsing — only `"zoh"`, `"linear"`, `"cubic"` accepted (lowercase); any other string is an XML error; (3) `nsample` negative values — MuJoCo silently accepts (verified empirically: `nsample=-1` → `historyadr=-1`); spec documents whether we match this behavior or add stricter validation (with rationale); (4) `interp="cubic"` with `nsample < 4` — MuJoCo silently accepts (no minimum nsample for cubic interp); spec documents whether to match or add validation; (5) delay exceeding buffer time capacity (e.g., delay=0.1 with nsample=2, ts=0.002) — MuJoCo silently accepts; spec documents this as a non-validated case. Each validation has a test in the test plan. Spec explicitly states which validations are "match MuJoCo exactly" vs "stricter than MuJoCo" (if any), with justification. |
| **A** | Major validations documented. One minor validation gap. |
| **B** | Some validations noted but others missing (e.g., delay>0 interaction not documented). |
| **C** | No validation discussion. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo C structs, fields,
      source files, line numbers, keyword values, stride formulas, and empirical
      verification data. Two independent reviewers would agree on grades by
      pointing to specific spec content.

- [x] **Non-overlap:** P1 covers MuJoCo reference accuracy. P2 covers
      algorithm/code completeness. P3 covers convention mappings. P4 covers AC
      structure. P5 covers test coverage. P6 covers ordering. P7 covers blast
      radius. P8 covers consistency. P9 covers data lifecycle (fields exist
      and are correct). P10 covers compiler validation (invalid input is
      rejected). No significant overlap.

- [x] **Completeness:** The 10 criteria cover all dimensions. Could the spec
      be A+ on P1–P8 but still have a gap? Yes — if fields are declared but
      historyadr is wrong (P9 catches) or if invalid models are accepted (P10
      catches). Could it be A+ on P1–P9 but miss validation? Yes — P10 catches.

- [x] **Gradeability:** Each criterion maps to specific spec sections (see
      mapping table below).

- [x] **Conformance primacy:** P1 is tailored with specific MuJoCo struct
      names, source files, field names, line numbers, MJCF keywords, stride
      formula, and empirical verification data. P4 requires MuJoCo-verified
      expected values. P5 requires conformance tests. P9 requires model-level
      equivalence with MuJoCo's compiled output. P10 requires matching MuJoCo's
      validation rules.

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
| P9 | Specification lifecycle annotations, Files Affected, Data struct changes |
| P10 | Specification validation sections, Test Plan negative cases |

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
| P9. Data Lifecycle Completeness | | |
| P10. Compiler Validation Completeness | | |

**Overall: —**

> **Grading note:** The Evidence column is not optional. For each grade, cite
> the specific spec content that justifies it. For each gap, state exactly
> what's missing. "Looks good" is not evidence.

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P1 | Rev 0 rubric described `interp` as integer attribute — empirical testing revealed it takes string keywords (`"zoh"`, `"linear"`, `"cubic"`) | Rubric review + MuJoCo empirical testing | Rewrote P1 A+ bar to require keyword documentation with verification evidence | Rubric Rev 1 |
| R2 | P1, P2 | Rev 0 rubric allowed `historyadr` to be "deferred" or "all -1" — empirical testing showed MuJoCo computes cumulative offsets at compile time using `2*nsample+2` stride | Rubric review + MuJoCo empirical testing | Rewrote P1 to require stride formula and historyadr computation; rewrote P2 to require step (4) historyadr computation; rewrote P9 to require correct (not deferred) computation | Rubric Rev 1 |
| R3 | P1 | Rev 0 rubric listed `delay>0 with nsample=0` as "edge case to document" — empirical testing showed it's a hard compile error | Rubric review + MuJoCo empirical testing | Rewrote P1 edge cases to require compile error documentation; added P10 for validation completeness | Rubric Rev 1 |
| R4 | P2 | Rev 0 rubric had 4 steps, missing historyadr computation and compiler validation as explicit steps | Rubric review | Expanded to 5 steps: MJCF types, Model types, builder compilation, historyadr computation, compiler validation | Rubric Rev 1 |
| R5 | P7 | Rev 0 rubric file list omitted `data.rs` (Data struct needs history buffer) | Rubric review | Added `data.rs` or equivalent to P7 file list | Rubric Rev 1 |
| R6 | — | Rev 0 rubric had no Empirical Ground Truth section — rubric was writing bars based on header-file assumptions, not verified MuJoCo behavior | Rubric review | Added Empirical Ground Truth section with verification data from MuJoCo Python tests | Rubric Rev 1 |
| R7 | — | Rev 0 rubric had 9 criteria — missing compiler validation as a distinct concern | Rubric review | Added P10 (Compiler Validation Completeness) | Rubric Rev 1 |
| R8 | P1 | Rev 1 rubric described history buffer layout as "nsample (time, value) pairs" — empirical `mj_initCtrlHistory` testing shows layout is `[meta0, meta1, times..., values...]` (contiguous blocks, NOT interleaved pairs) | Review round 2 + MuJoCo empirical testing | Fixed buffer layout description in both Empirical Ground Truth section and P1 A+ bar | Rubric Rev 2 |
| R9 | P2, P7, P9 | Rev 1 rubric omitted `Model::make_data()` as an allocation point, `Data::reset()` as a reset/restoration point, and `impl Clone for Data` (manual, ~60 fields) as a required update site | Review round 2 + codebase exploration | P2 expanded to 6 steps (added step 6: Data integration — make_data, reset, Clone). P7 file list now names make_data, reset, Clone explicitly. P9 lifecycle expanded to 6 phases including Reset and Clone. | Rubric Rev 2 |
| R10 | P3 | Rev 1 rubric referenced umbrella Convention Registry §1 without flagging that the umbrella suggests `Vec<usize>` for nsample (wrong — MuJoCo uses signed int, accepts negatives) and `InterpType::None` as default name (wrong — default is ZOH, not "none") | Review round 2 | P3 A+ bar now requires spec to flag and resolve these umbrella conflicts | Rubric Rev 2 |
| R11 | — | Rev 1 Empirical Ground Truth lacked stride verification across broad nsample range and MuJoCo version pinning | Review round 2 | Added exhaustive stride table (nsample 1–32, all verified), pinned to MuJoCo 3.5.0, added actuator type compatibility section, added ns=3,0,2 skip-actuator test to historyadr table | Rubric Rev 2 |
| R12 | P1, P2, P9 | Rev 2 rubric assumed history buffer is zero-initialized. Empirical testing shows MuJoCo pre-populates: meta0=0.0, meta1=float(nsample-1), times=`[-(nsample)*ts, ..., -ts]`, values=0.0. `mj_resetData` restores this state, not just zeros. | Review round 3 + MuJoCo empirical testing | Added "Initial history buffer state" section to Empirical Ground Truth with verification data. Updated P1 to require initial state documentation. Updated P2 step 6 to require pre-population (not `vec![0.0; nhistory]`). Updated P9 to require pre-populated initialization in phases 2 and 4. | Rubric Rev 3 |
| R13 | P1 | Rev 2 rubric lacked the general stride formula. Actuator stride `2n+2` is a special case of `nsample*(dim+1)+2` for dim=1. Sensor stride verified for dim=1,3,4. | Review round 3 + MuJoCo empirical testing | Added "General stride formula" section to Empirical Ground Truth. Updated P1 A+ bar to require general formula documentation. | Rubric Rev 3 |
| R14 | P4, P5 | Rev 2 rubric had no ACs or tests for Data.history initial state or Data::reset() behavior | Review round 3 | Added AC (e) for initial buffer state verification, AC (j) for reset behavior. Updated P5 to require Data lifecycle tests. | Rubric Rev 3 |
| R15 | P1 | Rev 2 rubric didn't document `mj_step` circular buffer writes to history. While runtime is deferred, understanding that the initial buffer state is designed for immediate runtime use (pre-populated times, write-head metadata) is essential context. | Review round 3 + MuJoCo empirical testing | Updated P1 A+ bar to require documentation of mj_step circular buffer behavior (as deferred runtime context). Added to Empirical Ground Truth initial state section. | Rubric Rev 3 |
| R16 | P1 | Rev 3 rubric didn't explicitly state that `actuator_history` is `nu×2` for ALL actuators (not just those with history), that `interp` can be set independently of `nsample` (e.g., `interp="cubic"` with `nsample=0` stores `[0, 2]` — no error), and that `actuator_delay` defaults to 0.0 for all actuators. | Review round 4 + MuJoCo empirical testing | Updated Compiled Model Layout section to explicitly document per-actuator universality of all three arrays. | Rubric Rev 4 |
| R17 | P1 | Rev 3 rubric described buffer values as "ctrl values" but didn't prove it. Empirical test with `dyntype=integrator` confirms: history stores ctrl (1.0), not act (0.0). | Review round 4 + MuJoCo empirical testing | Updated buffer layout description with ctrl-not-act verification evidence. | Rubric Rev 4 |
| R18 | — | Rev 3 had two stale references: P2 A-grade bar said "zeroing strategy" (should be "restoration strategy"), Gap Log R9 said "zeroing point" (should be "reset/restoration point"). Both contradicted the Rev 3 pre-population requirement. | Review round 4 consistency audit | Fixed both stale references. | Rubric Rev 4 |
| R19 | P1, P2, P7, P9 | `Data::reset_to_keyframe()` (`data.rs:964`) also needs to restore pre-populated history state. MuJoCo's `mj_resetDataKeyframe` produces identical history to `mj_resetData` (verified empirically). Rubric through Rev 4 only mentioned `reset()`, not `reset_to_keyframe()`. | Review round 5 + MuJoCo empirical testing | Added reset_to_keyframe to P1 A+ bar, P2 step 6, P7 file list, P9 Reset phase, P4 AC (k). | Rubric Rev 5 |
| R20 | P7 | `data.rs` has a staleness guard test `data_reset_field_inventory` (line 1026) with a hardcoded `EXPECTED_SIZE`. Adding a `history` field to `Data` will change `size_of::<Data>()` and fail this test. Spec must update the constant. | Review round 5 + codebase exploration | Added staleness guard constant update to P2 step 6 and P7 file change list. | Rubric Rev 5 |
| R21 | P1, P10 | MuJoCo's `nhistory` includes BOTH actuator and sensor contributions. CortenForge doesn't parse sensor `nsample`, so `nhistory` will be wrong for models with sensor history. This is a known conformance gap that the spec must acknowledge. | Review round 5 + MuJoCo empirical testing | Added nhistory scope caveat to Empirical Ground Truth and P1 A+ bar. | Rubric Rev 5 |
| R22 | P10 | MuJoCo performs no validation on: `interp="cubic"` with `nsample < 4` (accepted silently), delay exceeding buffer time capacity (accepted silently). These non-validated edge cases weren't documented. | Review round 5 + MuJoCo empirical testing | Added to Compiler Validation section of Empirical Ground Truth. Updated P10 A+ bar with items (4) and (5). | Rubric Rev 5 |
| R23 | P1 | `mj_forward` READS from history to apply delayed ctrl. The pre-populated initial buffer state directly affects forward dynamics — not just a data layout detail. A delayed actuator with fresh MjData produces force=0 despite ctrl=1.0 (verified empirically). The rubric through Rev 5 treated initial buffer state as only relevant for future runtime work. | Review round 6 + MuJoCo empirical testing | Added "Forward dynamics interaction" section to Empirical Ground Truth. Updated P1 A+ bar to require documenting this interaction. | Rubric Rev 6 |
| R24 | P2, P7 | CortenForge's `parser.rs` has a `<general>`-only attribute gate (lines 2069-2076) that would cause `nsample`/`interp`/`delay` to be silently ignored on shortcut types (position, motor, velocity) if placed in the wrong section. The spec must explicitly place parsing in the common attribute section (lines 2013-2052). | Review round 6 + codebase exploration | Updated P2 step 1 to require common-section placement with specific line references. Updated P7 with risk note about parser gate. | Rubric Rev 6 |
| R25 | P1 | Rubric through Rev 5 referenced `model.opt.timestep` in some contexts. The CortenForge timestep field is `model.timestep` (`model.rs:682`), not `model.opt.timestep`. Also added: `mj_copyData` copies history buffer, `mj_forward` does NOT write to history. | Review round 6 + codebase/empirical | Added CortenForge builder integration notes section to Empirical Ground Truth with correct timestep path. | Rubric Rev 6 |
| R26 | P2, P7 | Rubric through Rev 6 described "builder compilation" as a single step without identifying the 3-file pipeline: `builder/mod.rs` (ModelBuilder struct holds intermediate arrays), `builder/actuator.rs` (`process_actuator()` pushes per-actuator values), `builder/build.rs` (`build()` transfers arrays to Model and does post-processing). The spec writer needs to know all three files and the push→transfer→postprocess flow. | Review round 7 + codebase exploration | Updated P2 steps (2)-(4) with specific file references and the 3-file pipeline. Updated P7 items (h)-(j) with all three builder files. Updated Empirical Ground Truth builder notes. | Rubric Rev 7 |
| R27 | P2, P7 | `defaults.rs` has TWO functions that need updating — `merge_actuator_defaults()` (line 743, merges parent/child class defaults) AND `apply_to_actuator()` (line 294, applies resolved defaults to actuator). Rubric through Rev 6 only said "default class merge logic" in P7 without naming either function, and P2 step 1 didn't name them. Missing one function means default class inheritance silently fails. | Review round 7 + codebase exploration | Updated P2 step (1) to name both functions with line numbers and patterns. Updated P7 item (c) to name both functions. | Rubric Rev 7 |
| R28 | P2 | `historyadr` computation must happen in `build()` (`builder/build.rs`) AFTER all actuators are processed, since it requires cumulative sum across all actuators. This timing constraint was not explicit — placing it in `process_actuator()` (per-actuator) would be wrong since cumulative offsets aren't known until all actuators exist. | Review round 7 + codebase exploration | Updated P2 step (4) with explicit timing constraint and placement in `build()` around line 386 (post-processing section). | Rubric Rev 7 |
| R29 | P2 | `InterpolationType` enum creation was mentioned at the tail of P2's A+ bar but not assigned to any numbered step. P7 item (f) correctly listed `enums.rs`, but an implementer following P2's steps could miss that `enums.rs` needs modification. | Round 7 consistency audit | Folded `InterpolationType` enum into P2 step (2) (Model type additions) with explicit `enums.rs` file reference. | Rubric Rev 7 |
| R30 | P5 | P4 AC (k) requires `Data::reset_to_keyframe()` to restore pre-populated history, but P5 (test plan) only mentioned `Data::reset()` in the data lifecycle tests — `reset_to_keyframe()` was absent. AC (k) had no corresponding test requirement. | Round 7 consistency audit | Added `Data::reset_to_keyframe()` test requirement to P5 data lifecycle tests, referencing AC (k). | Rubric Rev 7 |
| R31 | P2 | P2 step (4) cited `builder/build.rs:21+` (vague, means entire file) while the Empirical Ground Truth used precise line ranges (244-262 for array transfer, 382-386 for post-processing). Inconsistent precision between sections. | Round 7 consistency audit | Updated P2 step (4) with precise line references matching the Empirical Ground Truth. | Rubric Rev 7 |
