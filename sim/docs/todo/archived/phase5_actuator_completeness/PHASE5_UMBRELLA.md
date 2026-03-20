# Phase 5 — Actuator Completeness: Umbrella Spec

**Status:** Complete (2026-02-27)
**Phase:** Roadmap Phase 5
**Tasks:** 10 (DT-6, DT-8, DT-9, DT-56, DT-57, DT-58, DT-59, DT-77, §61, §63) — all ship-complete
**Deliverables:** 2 T1 items + 4 sub-specs — all delivered across 19 sessions
**Test baseline:** 2,148+ domain tests (post-Phase 4) → 2,238+ (post-Phase 5)

---

## Scope

Phase 5 closes all actuator-related conformance gaps in the v1.0 roadmap.
**The goal is MuJoCo conformance** — after Phase 5, CortenForge produces
numerically identical results to MuJoCo for every supported actuator feature:
transmission types, dynamics variants, gain/bias parameter computation,
length-range estimation, and data array sizing. Every sub-spec exists to
close a specific conformance gap between "what MuJoCo does" and "what we do."

This umbrella spec coordinates the 4 sub-specs and 2 T1 items that comprise
Phase 5. It defines implementation order, file ownership, API contracts, and
shared conventions so the sub-specs can be written and implemented without
coordination conflicts.

> **Conformance mandate for sub-spec authors:** Each sub-spec must cite the
> exact MuJoCo C source (function, file, line range) for every behavior it
> implements. Acceptance criteria must include expected values derived from
> MuJoCo's output — not from hand-calculation or intuition. The MuJoCo C
> source code is the single source of truth. When the MuJoCo docs and the
> C source disagree, the C source wins.

---

## Task Assignment

Every Phase 5 task is assigned to exactly one deliverable. No orphans, no
overlaps. Each task closes a specific MuJoCo conformance gap.

| Task | Description | Deliverable | Rationale |
|------|-------------|-------------|-----------|
| DT-6 | `actearly` runtime wiring | **T1-a** (standalone) | Already parsed + wired; needs verification + tests |
| §63 | `dynprm` array 3→10 | **T1-b** (standalone) | Mechanical resize; prerequisite for all specs |
| DT-57 | `acc0` for non-muscle actuators | **Spec A** | Extends `compute_muscle_params()` |
| DT-56 | `dampratio` for position actuators | **Spec A** | Depends on DT-57's `acc0` |
| DT-59 | Bisection `lengthrange` for unlimited slides | **Spec A** | Extends `compute_muscle_params()` |
| DT-77 | Site-transmission `lengthrange` auto-estimation | **Spec A** | Extends `compute_muscle_params()` |
| DT-8 | Transmission types: `cranksite`, `slidersite`, `jointinparent` | **Spec B** | Adds enum variants + parsing |
| §61 | Slider-crank runtime computation | **Spec B** | Uses DT-8's enum variants for moment arms |
| DT-58 | Hill-type muscle dynamics variant | **Spec C** | Architectural decision: `sim-muscle` integration |
| DT-9 | `nsample`, `interp`, `delay` interpolation attrs | **Spec D** | Independent input-pipeline parsing |

### Sub-spec scope statements

Each sub-spec must identify the exact MuJoCo C functions it is porting, cite
them with source file and line ranges, and produce acceptance criteria with
MuJoCo-verified expected values.

**Spec A — acc0, dampratio, and length-range estimation** (DT-57, DT-56, DT-59, DT-77):
Build-time actuator parameter computation in `compute_muscle_params()`. Extends
the existing muscle-only function to handle non-muscle `acc0`, position actuator
`dampratio`, unlimited slide joint `lengthrange` bisection, and site-transmission
`lengthrange` auto-estimation. Primary changes in `muscle.rs` (computation)
and `builder/actuator.rs` (parsing `dampratio` attribute, calling the extended
compute function). MuJoCo reference: `mj_setLengthRange()` in
`engine_setconst.c`, `mj_setConst()` in `engine_setconst.c`.

**Spec B — Transmission types + slider-crank** (DT-8, §61):
New `ActuatorTransmission` enum variants (`SliderCrank`, `JointInParent`) plus
the runtime moment arm computation for slider-crank mechanisms. Parsing in
`builder/actuator.rs`, runtime in `actuation.rs`. `cranksite`/`slidersite`
are parsed as part of the `SliderCrank` transmission (not separate transmission
types — they're attributes of the slider-crank mechanism). MuJoCo reference:
`mj_transmission()` in `engine_forward.c`.

**Spec C — Hill-type muscle dynamics** (DT-58):
New `ActuatorDynamics::Muscle` subvariant or separate `HillMuscle` variant.
Architectural decision about integration with `sim-muscle` crate. Affects
`enums.rs`, `actuation.rs`, and the `sim-muscle` public API. MuJoCo reference:
`mj_muscleActuation()` in `engine_forward.c`, muscle dynamics in
`engine_forward.c`.

**Spec D — Interpolation actuator attributes** (DT-9):
MJCF parsing and model storage for `nsample`, `interp`, `delay`. New model
arrays, no runtime behavior change (attributes stored for future use by
MuJoCo 3.x-compatible controllers). Parsing in `builder/actuator.rs`, storage
in `model.rs`. MuJoCo reference: actuator element parsing in `user_model.cc`.

---

## Dependency Graph & Implementation Order

```
              ┌─────────┬─────────┐
              │  T1-a   │  T1-b   │  Step 1 (parallel)
              │  DT-6   │  §63    │
              │ actearly│ dynprm  │
              └─────────┴────┬────┘
                             │ (all specs depend on T1-b landing)
               ┌─────────────┼─────────────┐
               │             │             │
          ┌────▼────┐   ┌────▼────┐   ┌────▼────┐
          │ Spec A  │   │ Spec B  │   │ Spec D  │  Step 2 (parallel)
          │ acc0 +  │   │ transm  │   │ interp  │
          │ length  │   │ + crank │   │ attrs   │
          └────┬────┘   └─────────┘   └─────────┘
               │
          ┌────▼────┐
          │ Spec C  │  Step 3
          │ Hill    │
          │ muscle  │
          └─────────┘
```

### Implementation order with rationale

| Step | Deliverable | Rationale |
|------|-------------|-----------|
| **1** | **T1-a (DT-6) + T1-b (§63)** (parallel) | T1-b: `dynprm` resize affects every file that constructs or reads `dynprm`. Must land before sub-specs so they write against the new type. T1-a: verify `actearly` wiring. No downstream dependency — parallelizes with T1-b. |
| **2** | **Spec A, B, D** (parallel) | These three specs are independent — they touch different code paths (Spec A: `muscle.rs`, Spec B: `actuation.rs` transmissions, Spec D: parsing only). The one shared file (`builder/actuator.rs`) is safe because each spec adds new parsing code in different sections. |
| **3** | **Spec C** | Depends on Spec A because Hill muscle dynamics must integrate with the extended `compute_muscle_params()` that Spec A produces. Without knowing Spec A's new function signature, Spec C would write against a stale API. |

### Dependency edges

| From → To | Specific dependency |
|-----------|-------------------|
| T1-b → Spec A | Spec A's `muscle_activation_dynamics()` takes `&[f64; 10]` after resize |
| T1-b → Spec B | Spec B's test helpers construct `actuator_dynprm` with new size |
| T1-b → Spec C | Spec C's `HillMuscle` dynamics parameters live in the expanded `dynprm` |
| T1-b → Spec D | Spec D's model array initialization uses `[0.0; 10]` |
| Spec A → Spec C | Spec C integrates with `compute_muscle_params()` after Spec A extends it |

---

## File Ownership Matrix

Files touched by 2+ deliverables, with ownership sequence and handoff state.

### `sim/L0/core/src/types/enums.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec B | Add `SliderCrank` and `JointInParent` to `ActuatorTransmission` | 6 variants: Joint, Tendon, Site, Body, SliderCrank, JointInParent |
| 2 | Spec C | Add `HillMuscle` to `ActuatorDynamics` (or redesign Muscle variant) | 7 variants: None, Filter, FilterExact, Integrator, Muscle, HillMuscle, User |

No conflict: different enums.

### `sim/L0/core/src/types/model.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1-b (§63) | `actuator_dynprm: Vec<[f64; 3]>` → `Vec<[f64; 10]>` | Resized array |
| 2 | Spec B | Add `actuator_cranklength: Vec<f64>` (slider-crank parameter) | New field |
| 3 | Spec D | Add `actuator_nsample: Vec<usize>`, `actuator_interp: Vec<InterpType>`, `actuator_delay: Vec<f64>` | 3 new fields |

No conflict: T1-b modifies an existing field; Specs B and D add new fields.

### `sim/L0/core/src/types/model_init.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1-b (§63) | `actuator_dynprm: vec![]` unchanged (empty vec, type changes with model.rs) | Updated type |
| 2 | Spec B | Add init for `actuator_cranklength` | New field default |
| 3 | Spec D | Add init for `actuator_nsample`, `actuator_interp`, `actuator_delay` | 3 new field defaults |

No conflict: same pattern as `model.rs` — resize then append.

### `sim/L0/mjcf/src/builder/actuator.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1-b (§63) | Change `dynprm` default literal from `[0.0; 3]` to `[0.0; 10]`; update muscle defaults | Resized defaults |
| 2 | Spec A | Parse `dampratio` attribute on `<position>` actuators; call extended `compute_muscle_params()` during model build | New parsing + build-time call |
| 3 | Spec B | Parse `cranklength`, `slidersite`, `cranksite` attributes; map to `SliderCrank` transmission | New parsing block |
| 4 | Spec D | Parse `nsample`, `interp`, `delay` attributes; store in model arrays | New parsing block |

Low conflict risk: T1-b modifies existing defaults; Specs A, B, and D each add parsing in different attribute groups (`<position>` dampratio, `<general>` crank/slider, `<general>` interp).

### `sim/L0/core/src/forward/actuation.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1-a (DT-6) | Verify `actearly` wiring; add conformance tests | No code change (verification only) |
| 2 | Spec B | Add `SliderCrank` and `JointInParent` match arms in `mj_transmission()` | 2 new match arms |
| 3 | Spec C | Add `HillMuscle` match arm in activation dynamics dispatch | 1 new match arm |

No conflict: different match arms in different `match` blocks.

### `sim/L0/core/src/forward/muscle.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1-b (§63) | Update `muscle_activation_dynamics()` signature: `&[f64; 3]` → `&[f64; 10]` | Updated signature |
| 2 | Spec A | Extend `compute_muscle_params()`: remove muscle-only guard, add `acc0` for all actuator types, add `dampratio` resolution, add unlimited-slide bisection, add site-transmission estimation | Extended function (major) |
| 3 | Spec C | Add Hill-muscle integration point in or near `compute_muscle_params()` | New code section |

**This is the highest-risk file.** Spec A makes major changes to `compute_muscle_params()`. Spec C must integrate with the result. The API contract below defines the handoff.

### `sim/L0/core/src/sensor/mod.rs` (test helpers)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1-b (§63) | Update 3 test helper literals: `[0.0; 3]` → `[0.0; 10]` (lines 323, 370, 810) | Updated literals |

Single-owner after T1-b.

---

## API Contracts

Cross-spec API dependencies. Sub-specs must be written against these contracts,
not against the current (pre-Phase 5) codebase. These contracts exist to ensure
that the APIs evolve in a direction that supports MuJoCo conformance — every
signature change below is motivated by closing a conformance gap.

### Contract 1: `dynprm` array type (T1-b → all specs)

**Current:** `actuator_dynprm: Vec<[f64; 3]>`
**After T1-b:** `actuator_dynprm: Vec<[f64; 10]>`

All code that constructs `dynprm` arrays must use `[f64; 10]`. All code that
reads `dynprm` fields by index is unaffected (indices 0–2 are stable; 3–9
are zero-initialized).

`muscle_activation_dynamics()` signature changes:
```rust
// Before (current)
pub fn muscle_activation_dynamics(ctrl: f64, act: f64, dynprm: &[f64; 3]) -> f64

// After T1-b
pub fn muscle_activation_dynamics(ctrl: f64, act: f64, dynprm: &[f64; 10]) -> f64
```

The function body is unchanged (reads only `dynprm[0]`, `dynprm[1]`, `dynprm[2]`).

### Contract 2: `compute_muscle_params()` after Spec A (Spec A → Spec C)

**Current signature:**
```rust
impl Model {
    pub fn compute_muscle_params(&mut self) { ... }
}
```

**After Spec A — expected changes:**
1. The early return `if !has_muscles { return; }` is removed — the function now
   processes all actuator types (not just muscles) for `acc0` computation.
2. The function name may change to `compute_actuator_params()` to reflect its
   broader scope. Spec A will decide; Spec C must use whatever name Spec A
   chooses.
3. New logic sections:
   - `acc0` for non-muscle actuators (DT-57)
   - `dampratio` resolution for position actuators using `acc0` (DT-56)
   - Bisection-based `lengthrange` for unlimited slide joints (DT-59)
   - Site-transmission `lengthrange` estimation via FK (DT-77)

**What Spec C needs from this contract:**
- The function processes all actuator types, so Spec C can add HillMuscle
  handling in the same function.
- The `acc0` computation for non-muscle types is available, so HillMuscle
  actuators can also use `acc0` for F0 auto-computation.

### Contract 3: `ActuatorTransmission` enum after Spec B (defensive)

**Current:** `Joint`, `Tendon`, `Site`, `Body`
**After Spec B:** `Joint`, `Tendon`, `Site`, `Body`, `SliderCrank`, `JointInParent`

No other Phase 5 spec depends on this, but the extended enum means that any
exhaustive `match` on `ActuatorTransmission` (in `compute_muscle_params()`,
`mj_transmission()`) must add arms for the new variants. Spec A's changes to
`compute_muscle_params()` must include these arms if Spec B lands first.

**Resolution:** Spec A and Spec B can land in either order. If Spec A lands
first, it doesn't need arms for variants that don't exist yet. If Spec B lands
first, Spec A must add no-op arms for `SliderCrank` and `JointInParent` in
`compute_muscle_params()`. The recommended approach: land Spec A before Spec B
(both are in Step 2, but Spec A should go first within that step).

### Contract 4: `ActuatorDynamics` enum after Spec C (defensive)

**Current:** `None`, `Filter`, `FilterExact`, `Integrator`, `Muscle`, `User`
**After Spec C:** adds `HillMuscle` (position TBD — Spec C decides)

No other Phase 5 spec depends on this. All exhaustive `match` blocks on
`ActuatorDynamics` will need a new arm, but those are all within Spec C's
scope.

---

## Shared Convention Registry

Conventions decided once here. Sub-specs reference this section instead of
inventing their own. These conventions are chosen to match MuJoCo's behavior
and naming where possible — deviations from MuJoCo naming are noted and
justified.

### 1. New model array naming

All new actuator-related model arrays follow the existing `actuator_{name}`
pattern:

| Field | Type | Default | Spec |
|-------|------|---------|------|
| `actuator_cranklength` | `Vec<f64>` | `0.0` | B |
| `actuator_nsample` | `Vec<usize>` | `0` | D |
| `actuator_interp` | `Vec<InterpType>` | `InterpType::None` | D |
| `actuator_delay` | `Vec<f64>` | `0.0` | D |

### 2. Enum variant ordering

New variants are appended after existing variants, before `User` (which stays
last by convention, matching MuJoCo's `mjXXX_USER` pattern):

```rust
pub enum ActuatorTransmission {
    Joint, Tendon, Site, Body,  // existing
    SliderCrank, JointInParent, // Spec B (new, appended)
}

pub enum ActuatorDynamics {
    None, Filter, FilterExact, Integrator, Muscle, // existing
    HillMuscle,                                     // Spec C (new)
    User,                                           // stays last (matches MuJoCo mjDYN_USER)
}
```

`ActuatorTransmission` has no `User` variant — new variants are simply
appended. `ActuatorDynamics` has `User` last — new variants go before it.

### 3. Default values for new arrays

- Numeric fields default to `0.0` (not `f64::NAN` or sentinels).
- Boolean fields default to `false`.
- Enum fields default to their `#[default]` variant.
- New arrays are initialized in `model_init.rs` as empty `vec![]` (populated
  during builder compilation, same as all existing actuator arrays).

### 4. MJCF attribute naming

Match MuJoCo attribute names exactly:
- `cranklength` (not `crank_length`)
- `slidersite` (not `slider_site`)
- `cranksite` (not `crank_site`)
- `jointinparent` (not `joint_in_parent`)
- `nsample`, `interp`, `delay` (match MuJoCo 3.x names exactly)

Rust field names use snake_case: `actuator_cranklength`, `actuator_nsample`.

### 5. `dynprm` array post-resize

After §63, `dynprm` is `[f64; 10]` everywhere. Muscle defaults become:
```rust
[0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
//  ↑       ↑       ↑    └── elements 3-9: reserved (zero)
//  tau_act tau_deact tausmooth
```

All existing code that reads `dynprm[0]`, `dynprm[1]`, `dynprm[2]` is
unaffected. Code constructing `dynprm` literals must use `[f64; 10]`.

---

## Cross-Spec Blast Radius

### Behavioral interactions between specs

| Interaction | Analysis |
|------------|----------|
| §63 `dynprm` resize + Spec A `acc0` computation | **No conflict.** Spec A reads `dynprm` only for muscle activation dynamics (indices 0–2), which are unchanged by the resize. The resize adds zero-filled padding. |
| §63 `dynprm` resize + Spec C Hill muscle | **No conflict.** Hill muscle may use `dynprm[3..9]` for Hill-specific parameters. This is the *reason* for the resize — expanded parameter space for new dynamics types. |
| DT-6 `actearly` + Spec C Hill muscle | **Potential interaction.** `actearly` controls whether activation at time `t+h` is used for force computation. Hill muscle dynamics have different activation time constants. Spec C must document whether `actearly` applies to `HillMuscle` (it should — same activation ordering principle). |
| Spec A `acc0` extension + Spec C Hill muscle | **Dependency, not conflict.** Spec C's HillMuscle actuators benefit from `acc0` computation (for F0 auto-scaling). Spec A must not gate `acc0` on `dyntype == Muscle` only — it must compute for all actuator types, which naturally includes HillMuscle once Spec C adds it. |
| Spec B `SliderCrank` + Spec A `lengthrange` | **No conflict.** Slider-crank actuators use the `SliderCrank` transmission type, which Spec A's `compute_muscle_params()` will handle with a no-op or basic estimation arm (slider-crank muscles are exotic). |

### Existing test impact (cross-spec)

| Test area | Touched by | Conflict risk |
|-----------|-----------|--------------|
| `muscle.rs` unit tests (17 tests) | T1-b (literal resize), Spec A (new test cases) | **Low.** T1-b changes `[0.0; 3]` → `[0.0; 10]` in test helpers. Spec A adds new tests. No overlap in test assertions. |
| `actuation.rs` integration tests | DT-6 (new tests), Spec B (new tests), Spec C (new tests) | **None.** Each adds tests for different functionality. |
| `sensor/mod.rs` test helpers | T1-b only | **None.** Single-owner. |
| Phase 4 regression suite (39 tests) | None | **None.** Phase 5 does not modify any Phase 4 code paths. |
| Full sim domain baseline (2,148+ tests) | T1-b (dynprm resize propagation) | **Low.** The resize changes array types but not values. Any test that hardcodes `[f64; 3]` for dynprm will fail to compile (caught at build time, not runtime). |

### Test count changes

| Deliverable | Estimated new tests | Net change |
|-------------|-------------------|------------|
| T1-a (DT-6) | 2–4 (actearly conformance) | +2–4 |
| T1-b (§63) | 1–2 (parse 10-element dynprm) | +1–2 |
| Spec A | 8–12 (acc0 non-muscle, dampratio, lengthrange bisection, site estimation) | +8–12 |
| Spec B | 4–6 (slider-crank moment arm, jointinparent, singularity) | +4–6 |
| Spec C | 4–6 (HillMuscle activation, integration with sim-muscle) | +4–6 |
| Spec D | 3–4 (parse nsample/interp/delay, defaults, round-trip) | +3–4 |
| **Total** | **22–34** | **+22–34** |

---

## Phase-Level Acceptance Criteria

These are the aggregate gates that determine "Phase 5 complete." Individual
sub-specs have their own ACs for technical correctness. **The overarching
criterion: CortenForge's actuator behavior is numerically identical to MuJoCo's
for every feature implemented in Phase 5.**

### PH5-AC1: All 10 tasks ship-complete
Every task in the assignment table has landed and its sub-spec ACs (or T1
verification criteria) are met. Every sub-spec AC that asserts a numerical
value has that value verified against MuJoCo's actual output.

### PH5-AC2: No regression in existing test suite
All 2,148+ domain tests from the post-Phase 4 baseline pass. Zero test
failures attributable to Phase 5 changes.

### PH5-AC3: Quality gate passes
`cargo xtask check` passes (formatting, clippy, all tests).

### PH5-AC4: Aggregate test growth
Domain test count increases by at least 22 (lower bound of per-spec estimates).
At least one MuJoCo conformance test (expected value from running MuJoCo) per
sub-spec.

### PH5-AC5: Actuator conformance coverage
After Phase 5, the following MuJoCo actuator features are covered:

| Feature | Pre-Phase 5 | Post-Phase 5 |
|---------|------------|-------------|
| Transmission types | Joint, Tendon, Site, Body | + SliderCrank, JointInParent |
| Dynamics types | None, Filter, FilterExact, Integrator, Muscle, User | + HillMuscle |
| `acc0` computation | Muscle actuators only | All actuator types |
| `dampratio` for position actuators | Not implemented | Implemented via `acc0` |
| `lengthrange` estimation | Muscle + limited joints/tendons | + unlimited slide joints, + site-transmission |
| `dynprm` array size | 3 elements | 10 elements (MuJoCo parity) |
| `actearly` | Parsed, wired, unverified | Verified with conformance tests |
| Interpolation attributes | Not parsed | Parsed + stored (runtime deferred) |

---

## Out of Scope

Explicitly excluded from Phase 5. Each exclusion states its conformance impact
— is MuJoCo conformance affected by deferring this work?

- **DT-5** (`gaintype/biastype/dyntype="user"` callback types) — requires
  plugin system; tracked in Post-v1.0 Low-Priority MuJoCo Compat.
  *Conformance impact: minimal — user callback types are rarely used in standard
  MuJoCo models. No standard actuator type requires `user` dynamics.*
- **DT-7** (`actdim` explicit override) — auto-detection sufficient for v1.0;
  tracked in Post-v1.0. *Conformance impact: none for standard models —
  `actdim` auto-detection matches MuJoCo for all built-in actuator types.*
- **DT-60** (`jnt_actgravcomp` routing) — already done, subsumed by §41.
  *Conformance impact: none — already conformant.*
- **Muscle FLV curve improvements** — current Millard-based curves are correct;
  Hill-type is a new variant, not a replacement. *Conformance impact: none —
  existing curves match MuJoCo.*
- **Actuator performance optimization** — Phase 5 is correctness/completeness,
  not performance. *Conformance impact: none — performance does not affect
  numerical conformance.*
