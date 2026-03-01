# Spec D — Sensor History Attributes: Spec

**Status:** Draft
**Phase:** Roadmap Phase 6 — Sensor Completeness
**Effort:** S
**MuJoCo ref:** `mjsSensor_` in `mjspec.h`, lines 731–734; `sensor_history`/`sensor_historyadr`/`sensor_delay`/`sensor_interval` in `mjmodel.h`, lines 1218–1221; `mjxmacro.h:695–698` wiring
**MuJoCo version:** 3.5.0
**Prerequisites:**
- Phase 5 actuator history landed (actuator_nsample/interp/delay/historyadr in model.rs:592–609)
- Spec A parser refactoring (`parse_sensor_attrs()` at parser.rs:3453, `MjcfSensor` struct at types.rs:3097) — commit `28bc9f4`
- `InterpolationType` enum at enums.rs:256–279 (shared with actuators)
- Spec D is **independent of Specs B and C** — no runtime changes, no
  evaluation-level interaction. Spec B modifies frame sensor evaluation
  arms; Spec C adds new sensor types. Spec D is parse-and-store only.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Problem Statement

**Conformance gap** — MuJoCo stores 4 sensor history attributes per sensor
element (`nsample`, `interp`, `delay`, `interval`) in dedicated model arrays
(`sensor_history`, `sensor_historyadr`, `sensor_delay`, `sensor_interval` at
`mjmodel.h:1218–1221`). CortenForge does not parse or store these attributes
for sensors. The `nhistory` field (`model.rs:607–609`) is documented as
"actuator-only until sensor history is implemented."

The gap is parse-and-store only — runtime interpolation behavior (reading
from the history buffer during `mj_forward`) is deferred per the umbrella
spec §Out of Scope (tracked as DT-107/DT-108). After this spec lands,
sensor history attributes are parsed from MJCF, stored in the Model struct,
and `nhistory` includes sensor contributions. The combined actuator+sensor
history buffer layout matches MuJoCo's: actuators first, then sensors.

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream —
> the algorithm, the acceptance criteria, the tests — is derived from what's
> documented here. If this section is wrong, everything is wrong.

### Source locations

| Item | File | Lines | Description |
|------|------|-------|-------------|
| Spec struct | `mjspec.h` | 731–734 | `mjsSensor_::nsample`, `interp`, `delay`, `interval` |
| Model arrays | `mjmodel.h` | 1218–1221 | `sensor_history` (nsensor×2, packed [nsample,interp]), `sensor_historyadr` (nsensor), `sensor_delay` (nsensor), `sensor_interval` (nsensor×2, [period,phase]) |
| Xmacro wiring | `mjxmacro.h` | 695–698 | Maps spec fields to model arrays |
| **NOT** sensor_intprm | `mjmodel.h` | 1213 | `sensor_intprm` (nsensor×mjNSENS=3) is a **separate** array for user-defined integer params — NOT history |

### Attribute semantics

**`nsample`** (int, default 0): Number of samples in the circular history
buffer. When `nsample > 0`, a history buffer segment of size
`nsample * (dim + 1) + 2` is allocated. When `nsample <= 0` (including
negative values), `historyadr = -1` and no buffer is allocated.

**`interp`** (keyword string, default "zoh"): Interpolation order.
Case-sensitive lowercase only: `"zoh"` → 0, `"linear"` → 1, `"cubic"` → 2.
Stored in `sensor_history[2*i + 1]` as int. Stored even when `nsample = 0`
(EGT-11d). Invalid keywords (including `"Linear"`, `"ZOH"`, `"1"`, `""`)
are rejected (EGT-11c).

**`delay`** (float, default 0.0): Time delay in seconds. Validation:
`delay > 0.0 && nsample <= 0` → error "setting delay > 0 without a history
buffer" (EGT-4). Negative delay is accepted and stored (EGT-11j) — the check
is strictly `delay > 0.0`, not `delay != 0.0`.

**`interval`** (float, default 0.0): Sampling period in seconds. MJCF
attribute is a single float (period). Model stores `[period, phase]` pair
with phase initialized to `0.0`. Validation: `interval < 0.0` → error
"negative interval in sensor" (EGT-11b). `interval > 0` without `nsample`
is valid — interval is independent of the history buffer (EGT-11a). Positive
phase in MJCF is rejected (EGT-7), but since MJCF only accepts a single
float, the phase is always compiler-initialized to 0.0.

### History buffer layout

Per entity with `nsample > 0`:
```
[sample_0_dim_0, ..., sample_0_dim_D,   // sample 0: D+1 values (D data + 1 time)
 sample_1_dim_0, ..., sample_1_dim_D,   // sample 1: D+1 values
 ...
 sample_N-1_dim_0, ..., sample_N-1_dim_D,  // sample N-1: D+1 values
 write_index,                            // 1 value: current write position
 count]                                  // 1 value: number of samples written
```

Total: **`nsample * (dim + 1) + 2`**

For actuators (`dim=1`): `nsample * 2 + 2` — this is the formula already in
`build.rs:397`. The sensor generalization uses `nsample * (dim + 1) + 2`,
which reduces to the actuator formula when `dim=1` (V3 derivation in rubric).

### Combined buffer layout

MuJoCo allocates actuators first, then sensors contiguously:

```
|--- actuator history ---|--- sensor history ---|
|  offset 0 → act_total  |  act_total → nhistory |
```

`sensor_historyadr` values start at the total actuator history size, not at 0.
For a sensor-only model (no actuators), sensors start at offset 0.

Example (EGT-6): actuator `nsample=4` (dim=1) → offset=0, contribution=10;
sensor `nsample=3` (dim=1) → `sensor_historyadr=10`, contribution=8;
total `nhistory=18`.

Example (EGT-11g): 2 actuators (nsample=4,2) + 2 sensors (nsample=3,5, dim=1):
`actuator_historyadr=[0,10]`, `sensor_historyadr=[16,24]`, `nhistory=36`.

### Edge cases (MuJoCo 3.5.0 verified)

| # | Behavior | MuJoCo result | EGT ref |
|---|----------|---------------|---------|
| 1 | `nsample=0` | `historyadr=-1`, no nhistory contribution | EGT-2 |
| 2 | Negative `nsample=-1` | Accepted, stored as-is, `historyadr=-1`, no nhistory | EGT-5 |
| 3 | `delay > 0` without `nsample > 0` | Error: "setting delay > 0 without a history buffer" | EGT-4 |
| 4 | Negative `delay=-0.01` | Accepted and stored (no error) | EGT-11j |
| 5 | `interval < 0` | Error: "negative interval in sensor" | EGT-11b |
| 6 | `interval > 0` without `nsample` | Accepted; interval stored, `historyadr=-1` | EGT-11a |
| 7 | `interp` stored without `nsample` | `sensor_history=[0, 1]` for interp="linear", nsample=0 | EGT-11d |
| 8 | Case-sensitive interp | "Linear"/"ZOH"/"1"/"" all rejected | EGT-11c |
| 9 | `nsample=0` in middle of chain | `historyadr=-1`, does not break offset accumulation | EGT-11f |
| 10 | Sensor-only model (no actuators) | `sensor_historyadr=[0]`, sensors start at offset 0 | EGT-11k |
| 11 | `interval > 0, delay > 0, nsample=0` | Delay error fires — interval does NOT override nsample requirement | EGT-4 |
| 12 | MuJoCo has no sensor defaults | `<default><sensor nsample="5"/>` → "unrecognized element" | EGT-11h |

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Parse `nsample` on sensor elements | Stored in `sensor_history[2*i+0]` (`mjmodel.h:1218`) | **Not parsed** — field does not exist on `MjcfSensor` |
| Parse `interp` on sensor elements | Stored in `sensor_history[2*i+1]` as int 0/1/2 (`mjmodel.h:1218`) | **Not parsed** |
| Parse `delay` on sensor elements | Stored in `sensor_delay[i]` (`mjmodel.h:1219`) | **Not parsed** |
| Parse `interval` on sensor elements | Stored in `sensor_interval[2*i+0]` (period), phase=0.0 (`mjmodel.h:1221`) | **Not parsed** |
| Compute sensor `historyadr` | Cumulative offset, -1 for nsample<=0 (`mjmodel.h:1220`) | **Not computed** — field does not exist |
| Include sensors in `nhistory` | `nhistory = Σ(actuator) + Σ(sensor)` | **Actuator-only** (`model.rs:607–608`) |
| Validate delay/nsample | `delay > 0 && nsample <= 0` → error | **Not validated** for sensors |
| Validate negative interval | `interval < 0` → error | **Not validated** |
| Sensor defaults for history attrs | **No sensor defaults in MuJoCo** (EGT-11h) | N/A — `MjcfSensorDefaults` is a CortenForge extension |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `sensor_history` | Packed `int*` array, stride 2: `[nsample, interp]` per sensor (`mjmodel.h:1218`) | Unpacked: `sensor_nsample: Vec<i32>` + `sensor_interp: Vec<InterpolationType>` — mirrors actuator pattern at `model.rs:592–596` | Unpack: `sensor_history[2*i+0]` → `sensor_nsample[i]`, `sensor_history[2*i+1]` → `sensor_interp[i]` as `InterpolationType` |
| `sensor_interval` | `mjtNum*` stride 2: `[period, phase]` per sensor (`mjmodel.h:1221`) | `sensor_interval: Vec<(f64, f64)>` — tuple of `(period, phase)`, matching `actuator_actrange: Vec<(f64, f64)>` precedent | `sensor_interval[2*i+0]` → `sensor_interval[i].0`, `sensor_interval[2*i+1]` → `sensor_interval[i].1` |
| `nsample` type | `int` (signed) in MuJoCo | `i32` (signed) — matches `actuator_nsample: Vec<i32>` at `model.rs:592` | Direct port — signed to accept negative values per EGT-5 |
| `interp` int→enum | `int` 0/1/2 | `InterpolationType` enum (`enums.rs:256–279`) — shared with actuators, NOT duplicated | `FromStr` at `enums.rs:267–278`: case-sensitive lowercase only |
| `interp` keyword | Case-sensitive lowercase: `"zoh"`, `"linear"`, `"cubic"` | `InterpolationType::from_str()` already matches — no case-insensitive parsing | Direct port — `FromStr` impl is already correct |
| MJCF attribute names | `nsample`, `interp`, `delay`, `interval` | Same names, no snake_case variants | Direct port — attribute names match exactly |
| `MjcfSensor` field pattern | N/A | New fields use `Option<T>` (None = not specified), NOT value-sentinel like `noise: f64` / `cutoff: f64` where 0.0 = "not set" | Use `Option::unwrap_or(default)` in builder, not value-sentinel comparison |
| Field naming convention | MuJoCo: `sensor_history`, `sensor_historyadr`, `sensor_delay`, `sensor_interval` | CortenForge: `sensor_nsample`, `sensor_interp`, `sensor_historyadr`, `sensor_delay`, `sensor_interval` — follows existing `sensor_{name}` pattern per umbrella §3.1 | All new fields prefixed `sensor_` — direct port with unpacking (history → nsample + interp) |
| Sensor defaults | **MuJoCo has NO sensor default class mechanism** (EGT-11h) | `MjcfSensorDefaults` at `types.rs:775` is a pre-existing CortenForge extension for `noise`/`cutoff`/`user`. **NOT extended** for history attrs per AD-3(b). | History attrs parsed from sensor elements only. `apply_to_sensor()` at `defaults.rs:533` untouched. |
| Error message parity | MuJoCo error messages include entity context (e.g., "setting delay > 0 without a history buffer" with element name/id appended) | CortenForge error messages are CortenForge-specific text | **Conformance requires matching validation BEHAVIOR (what is accepted/rejected), not error message phrasing.** Parse-time error messages have no message-text conformance requirement with MuJoCo. |

### Actuator → Sensor History Parity

Direct field mapping between the existing actuator history implementation and
the new sensor history implementation. Types, defaults, and validation logic
must match exactly (with sensor-specific additions noted).

| Actuator field | Sensor field | Type | Default | Parity |
|---------------|-------------|------|---------|--------|
| `actuator_nsample` (`model.rs:592`) | `sensor_nsample` | `Vec<i32>` | `0` | Exact match |
| `actuator_interp` (`model.rs:596`) | `sensor_interp` | `Vec<InterpolationType>` | `Zoh` | Exact match |
| `actuator_historyadr` (`model.rs:600`) | `sensor_historyadr` | `Vec<i32>` | `-1` | Exact match |
| `actuator_delay` (`model.rs:604`) | `sensor_delay` | `Vec<f64>` | `0.0` | Exact match |
| *(no equivalent)* | `sensor_interval` | `Vec<(f64, f64)>` | `(0.0, 0.0)` | **Sensor-only** — MuJoCo rejects `<motor interval="0.5"/>` |

| Validation | Actuator (`actuator.rs:272–278`) | Sensor (this spec) | Parity |
|------------|--------------------------------|-------------------|--------|
| delay/nsample | `delay > 0.0 && nsample <= 0` → error | Same check, same message | Exact match |
| negative interval | N/A | `interval < 0.0` → error | **Sensor-only** |
| interp parsing order | Parsed BEFORE validation (`actuator.rs:264–270`) | Parsed BEFORE validation (S5) | Exact match |

| Defaults mechanism | Actuators | Sensors | Parity |
|-------------------|----------|---------|--------|
| Per-element defaults | `<motor nsample="5"/>` in `<default>` works | **No sensor defaults in MuJoCo** (EGT-11h) | **Parity break** — MuJoCo treats them differently |

---

## Specification

### S1. Add 4 history fields to `MjcfSensor` struct

**File:** `mjcf/src/types.rs`
**MuJoCo equivalent:** `mjsSensor_::nsample/interp/delay/interval` in `mjspec.h:731–734`
**Design decision:** Use `Option<T>` for all 4 fields (None = attribute not
specified in MJCF). This differs from the value-sentinel pattern used by
existing `noise: f64`/`cutoff: f64` (where 0.0 = "not set") but is correct:
an explicit `nsample=0` means "no history" and must not be treated as "not
set." The mixed pattern is documented as acceptable — changing either would
create a larger inconsistency (rubric P8).

Add 4 fields to `MjcfSensor` struct (after `body2`, before closing brace):

```rust
/// Number of history samples. MuJoCo: `mjsSensor_::nsample`.
/// Default: None (builder treats as 0).
pub nsample: Option<i32>,
/// Interpolation keyword. MuJoCo: `mjsSensor_::interp`.
/// Valid: "zoh", "linear", "cubic" (case-sensitive lowercase only).
pub interp: Option<String>,
/// Time delay in seconds. MuJoCo: `mjsSensor_::delay`.
/// Default: None (builder treats as 0.0).
pub delay: Option<f64>,
/// Sampling interval (period) in seconds. MuJoCo: `mjsSensor_::interval`.
/// Default: None (builder treats as 0.0). Phase is always 0.0.
pub interval: Option<f64>,
```

Update `impl Default for MjcfSensor` (struct literal — compiler-enforced):

```rust
nsample: None,
interp: None,
delay: None,
interval: None,
```

Add 4 fluent builder methods (after `with_user`):

```rust
/// Set history sample count.
#[must_use]
pub fn with_nsample(mut self, nsample: i32) -> Self {
    self.nsample = Some(nsample);
    self
}

/// Set interpolation type keyword ("zoh", "linear", "cubic").
#[must_use]
pub fn with_interp(mut self, interp: impl Into<String>) -> Self {
    self.interp = Some(interp.into());
    self
}

/// Set time delay in seconds.
#[must_use]
pub fn with_delay(mut self, delay: f64) -> Self {
    self.delay = Some(delay);
    self
}

/// Set sampling interval (period) in seconds.
#[must_use]
pub fn with_interval(mut self, interval: f64) -> Self {
    self.interval = Some(interval);
    self
}
```

**`MjcfSensorDefaults` (`types.rs:775`) is NOT modified** — MuJoCo has no
sensor default class mechanism (EGT-11h). History attrs are parsed from
sensor elements only, per AD-3(b).

### S2. Parse 4 history attributes in `parse_sensor_attrs()`

**File:** `mjcf/src/parser.rs`
**MuJoCo equivalent:** MJCF attribute parsing in `mjxmacro.h:695–698`
**Design decision:** Add attribute reads after existing `user` parsing (line
3521), before the `sensor` return. Uses existing `parse_int_attr()` (line
3130) and `parse_float_attr()` (line 3125) helpers. `interp` is parsed as
raw string via `get_attribute_opt()` — string→enum conversion happens in
the builder (matching the actuator pattern at `actuator.rs:264–270`).

Add after `user` parsing block (before `sensor` return at line 3523):

```rust
sensor.nsample = parse_int_attr(e, "nsample");
sensor.interp = get_attribute_opt(e, "interp");
sensor.delay = parse_float_attr(e, "delay");
sensor.interval = parse_float_attr(e, "interval");
```

**`parse_sensor_defaults()` (`parser.rs:909`) is NOT modified** — history
attrs have no default mechanism in MuJoCo (EGT-11h, AD-3(b)).

### S3. Add 4 accumulation fields to `ModelBuilder` struct

**File:** `mjcf/src/builder/mod.rs`
**MuJoCo equivalent:** Builder-internal accumulation arrays
**Design decision:** Follow the existing sensor field pattern at
`mod.rs:634–647`. Types match the Model struct target fields.

Add after `sensor_cutoff` (line 644), before `sensor_name_list`:

```rust
pub(crate) sensor_nsample: Vec<i32>,
pub(crate) sensor_interp: Vec<InterpolationType>,
pub(crate) sensor_delay: Vec<f64>,
pub(crate) sensor_interval: Vec<(f64, f64)>,
```

**Import:** `InterpolationType` must be imported in `mod.rs`. It is already
available from `sim_core` (used in `actuator.rs`). Add to the existing
`use sim_core::{...}` at the top of `mod.rs` if not already present.

### S4. Initialize 4 new fields in `ModelBuilder::new()`

**File:** `mjcf/src/builder/init.rs`
**MuJoCo equivalent:** Builder initialization
**Design decision:** Struct literal pattern — compiler-enforced completeness.

Add after `sensor_cutoff: vec![]` (line 228), before `sensor_name_list`:

```rust
sensor_nsample: vec![],
sensor_interp: vec![],
sensor_delay: vec![],
sensor_interval: vec![],
```

### S5. Validate and push history attrs in `process_sensors()`

**File:** `mjcf/src/builder/sensor.rs`
**MuJoCo equivalent:** MuJoCo compiler sensor validation + array population
**Design decision:** Mirrors the actuator pattern at `builder/actuator.rs:264–283`
exactly. Order: (1) parse interp string→enum FIRST, (2) validate delay/nsample,
(3) validate negative interval, (4) push 4 fields in the SAME push block as
existing 11 fields (lines 83–100). The same push block is critical: the
`continue` at line 33 for unsupported types skips all pushes, maintaining Vec
alignment. A separate loop would break this.

**Note on `apply_to_sensor()`:** The call at line 26 applies `noise`/`cutoff`/
`user` defaults from the CortenForge defaults system. It does NOT touch
history fields — per AD-3(b), history attrs are not part of the defaults
system. No code change needed in `apply_to_sensor()` (`defaults.rs:533`).

Add validation + interp parsing after object resolution, before the push
block (after line 81, before line 83):

```rust
// Parse interp keyword → InterpolationType (mirrors actuator.rs:264–270)
let interp = match &mjcf_sensor.interp {
    Some(s) => s
        .parse::<InterpolationType>()
        .map_err(|e| ModelConversionError { message: e })?,
    None => InterpolationType::Zoh,
};

// MuJoCo compiler validation: delay > 0 requires nsample > 0
let nsample = mjcf_sensor.nsample.unwrap_or(0);
let delay = mjcf_sensor.delay.unwrap_or(0.0);
if delay > 0.0 && nsample <= 0 {
    return Err(ModelConversionError {
        message: "setting delay > 0 without a history buffer (nsample must be > 0)"
            .into(),
    });
}

// MuJoCo compiler validation: negative interval rejected
let interval = mjcf_sensor.interval.unwrap_or(0.0);
if interval < 0.0 {
    return Err(ModelConversionError {
        message: "negative interval in sensor".into(),
    });
}
```

Add 4 pushes inside the existing push block (after `sensor_cutoff` push at
line 92, before `sensor_name_list` logic at line 93):

```rust
self.sensor_nsample.push(nsample);
self.sensor_interp.push(interp);
self.sensor_delay.push(delay);
self.sensor_interval.push((interval, 0.0));
```

**Import:** Add `InterpolationType` to the `use sim_core::{...}` at line 7.

Update doc comment at line 2: change "13 pipeline sensor arrays" to
"17 pipeline sensor arrays" (13 existing + 4 new attributes; `historyadr`
is a 5th derived field computed post-hoc, not a pipeline array).

Update doc comment at line 16: change "13 pipeline sensor arrays" to
"17 pipeline sensor arrays".

### S6. Transfer 4 fields + init historyadr in `build.rs`

**File:** `mjcf/src/builder/build.rs`
**MuJoCo equivalent:** Model construction (spec→model transfer)
**Design decision:** Direct transfer for the 4 attribute fields (same pattern
as `sensor_noise`/`sensor_cutoff`). `sensor_historyadr` is initialized to
`vec![]` and computed post-hoc (same pattern as `actuator_historyadr: vec![]`
at line 267).

Add after `sensor_cutoff` transfer (line 241), before `sensor_name` (line 242):

```rust
sensor_nsample: self.sensor_nsample,
sensor_interp: self.sensor_interp,
sensor_delay: self.sensor_delay,
sensor_interval: self.sensor_interval,
sensor_historyadr: vec![],
```

### S7. Add 5 fields to `Model` struct

**File:** `core/src/types/model.rs`
**MuJoCo equivalent:** `sensor_history`/`sensor_historyadr`/`sensor_delay`/`sensor_interval` in `mjmodel.h:1218–1221`
**Design decision:** Follows the actuator history field pattern at
`model.rs:589–604` exactly. Types match. Comments cite MuJoCo source.

Add after `sensor_cutoff` (line 514), before `sensor_name` (line 516):

```rust
/// History sample count per sensor (length `nsensor`).
/// MuJoCo: `sensor_history[2*i + 0]`.  Default: 0 (no history).
/// Signed to match MuJoCo — accepts negative values (treated as no history).
pub sensor_nsample: Vec<i32>,

/// Interpolation type per sensor (length `nsensor`).
/// MuJoCo: `sensor_history[2*i + 1]` as int 0/1/2.  Default: Zoh.
pub sensor_interp: Vec<InterpolationType>,

/// Cumulative offset into `Data.history` per sensor (length `nsensor`).
/// MuJoCo: `sensor_historyadr`.  Equals -1 when `nsample <= 0`.
/// Offsets start after actuator history (sensors are allocated after actuators).
pub sensor_historyadr: Vec<i32>,

/// Time delay per sensor in seconds (length `nsensor`).
/// MuJoCo: `sensor_delay`.  Default: 0.0.  Present for all sensors.
pub sensor_delay: Vec<f64>,

/// Sampling interval per sensor: `(period, phase)` (length `nsensor`).
/// MuJoCo: `sensor_interval[2*i]` = period, `sensor_interval[2*i+1]` = phase.
/// Phase is always initialized to 0.0 by the compiler.
pub sensor_interval: Vec<(f64, f64)>,
```

Update `nhistory` comment at line 606–608:

**Before:**
```rust
/// Total history buffer size (actuator contributions only).
/// MuJoCo: `nhistory` (includes sensors — ours is actuator-only until
/// sensor history is implemented).
```

**After:**
```rust
/// Total history buffer size (actuator + sensor contributions).
/// MuJoCo: `nhistory`.
/// Layout: actuators first (offset 0 → actuator_total), then sensors
/// (actuator_total → nhistory). Formula per entity: nsample * (dim + 1) + 2.
```

### S8. Initialize 5 new fields in `Model::empty()`

**File:** `core/src/types/model_init.rs`
**MuJoCo equivalent:** Model initialization
**Design decision:** Struct literal — compiler-enforced completeness.

Add after `sensor_cutoff: vec![]` (line 231), before `sensor_name: vec![]`
(line 232):

```rust
sensor_nsample: vec![],
sensor_interp: vec![],
sensor_historyadr: vec![],
sensor_delay: vec![],
sensor_interval: vec![],
```

### S9. Extend post-hoc historyadr computation to include sensors

**File:** `mjcf/src/builder/build.rs`
**MuJoCo equivalent:** MuJoCo compiler historyadr + nhistory computation
**Design decision:** Extends the existing actuator-only block at lines
387–406 rather than replacing it. The actuator loop runs first (unchanged),
producing `offset = actuator_total`. Then a sensor loop appends, using
`nsample * (dim + 1) + 2` per sensor with `nsample > 0`. Sensor `dim` comes
from `self.sensor_dim[i]`.

**Arithmetic:** `nsample` is `i32`, `dim` is `usize`. Computation only runs
when `ns > 0` (already guarded), so no negative-nsample wrapping. Cast
sequence: `ns * (dim as i32 + 1) + 2` keeping `offset: i32`.

Replace the block at lines 387–406:

**Before:**
```rust
// Compute actuator_historyadr and nhistory (cumulative across all actuators).
// Must run BEFORE compute_actuator_params() because that calls make_data()
// which reads actuator_historyadr for history buffer pre-population.
{
    let nu = model.actuator_nsample.len();
    let mut historyadr = vec![-1i32; nu];
    let mut offset: i32 = 0;
    for (adr, &ns) in historyadr.iter_mut().zip(model.actuator_nsample.iter()) {
        if ns > 0 {
            *adr = offset;
            offset += 2 * ns + 2;
        }
        // ns <= 0: historyadr stays -1
    }
    model.actuator_historyadr = historyadr;
    #[allow(clippy::cast_sign_loss)]
    {
        model.nhistory = offset as usize;
    }
}
```

**After:**
```rust
// Compute actuator_historyadr and sensor_historyadr, then nhistory
// (cumulative across all actuators, then all sensors).
// Layout: actuators first (offset 0 → act_total), sensors after (act_total → nhistory).
// Must run BEFORE compute_actuator_params() because that calls make_data()
// which reads actuator_historyadr for history buffer pre-population.
{
    // Actuator historyadr (unchanged logic)
    let nu = model.actuator_nsample.len();
    let mut act_historyadr = vec![-1i32; nu];
    let mut offset: i32 = 0;
    for (adr, &ns) in act_historyadr.iter_mut().zip(model.actuator_nsample.iter()) {
        if ns > 0 {
            *adr = offset;
            offset += 2 * ns + 2;
        }
    }
    model.actuator_historyadr = act_historyadr;

    // Sensor historyadr (appends after actuators)
    let nsens = model.sensor_nsample.len();
    let mut sens_historyadr = vec![-1i32; nsens];
    for (i, (adr, &ns)) in sens_historyadr
        .iter_mut()
        .zip(model.sensor_nsample.iter())
        .enumerate()
    {
        if ns > 0 {
            *adr = offset;
            // nsample * (dim + 1) + 2 — generalized formula
            let dim = model.sensor_dim[i] as i32;
            offset += ns * (dim + 1) + 2;
        }
    }
    model.sensor_historyadr = sens_historyadr;

    #[allow(clippy::cast_sign_loss)]
    {
        model.nhistory = offset as usize;
    }
}
```

### S10. Update stale documentation

**Design decision:** Update 6 doc comments that become stale after this spec
lands. These are documentation-only changes — no behavioral impact.

**(a) `data.rs:78`** — Change `"Actuator history buffer"` to
`"History buffer (actuators + sensors)"`.

**(b) `enums.rs:253–254`** — Change `"Interpolation method for actuator
history buffer"` to `"Interpolation method for actuator and sensor history
buffers"`. Update the second line to reference both `actuator_history` and
`sensor_history`.

**(c) `sensor.rs:2`** — Change "13 pipeline sensor arrays" to
"17 pipeline sensor arrays" in the module doc comment.

**(d) `sensor.rs:16`** — Change "13 pipeline sensor arrays" to
"17 pipeline sensor arrays" in the function doc comment.

**(e) `sensor.rs:28`** — The comment says "Unsupported type (Jointlimitfrc,
Tendonlimitfrc) — skip with log" but Spec C already added support for these
types. The `continue` path and its comment are now stale/dead code. Update
to reflect current state.

**(f) `sensor.rs:467`** — The doc comment says "Returns `None` for types not
yet implemented in the pipeline" but `convert_sensor_type` now returns
`Some(...)` for all types. Update to reflect that all types are implemented.

### Downstream consumer audit

**`model.nhistory` readers:** Searched workspace for code reading `nhistory`:
- `model_init.rs:388` (`make_data()`) — allocates `vec![0.0; self.nhistory]`.
  Auto-grows with increased nhistory. **No change needed.**
- `data.rs:874` (`reset_data()`) — pre-populates actuator history only.
  Sensor pre-population is runtime scope (deferred). **No change needed.**
- L1 (`sim-bevy`) — does not read `model.nhistory` directly. **No impact.**
- `sim-physics` — does not read `model.nhistory` directly. **No impact.**

---

## Acceptance Criteria

### AC1: Parse nsample attribute *(runtime test — analytically derived)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" nsample="5"/></sensor>`
**After:** Model build
**Assert:** `model.sensor_nsample[0] == 5`
**Field:** `Model.sensor_nsample`

### AC2: Parse interp attribute *(runtime test — analytically derived)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" nsample="5" interp="linear"/></sensor>`
**After:** Model build
**Assert:** `model.sensor_interp[0] == InterpolationType::Linear`
**Field:** `Model.sensor_interp`

### AC3: Parse delay attribute *(runtime test — analytically derived)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" nsample="5" delay="0.02"/></sensor>`
**After:** Model build
**Assert:** `model.sensor_delay[0] == 0.02`
**Field:** `Model.sensor_delay`

### AC4: Parse interval attribute *(runtime test — analytically derived)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" interval="0.5"/></sensor>`
**After:** Model build
**Assert:** `model.sensor_interval[0] == (0.5, 0.0)`
**Field:** `Model.sensor_interval`

### AC5: nhistory correct for single sensor *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" nsample="5"/></sensor>` (dim=1)
**After:** Model build
**Assert:** `model.nhistory == 12` (5 × (1+1) + 2 = 12), `model.sensor_historyadr[0] == 0`
**Field:** `Model.nhistory`, `Model.sensor_historyadr`
**MuJoCo 3.5.0 verified:** EGT-3

### AC6: nhistory correct for multi-dim sensor *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><framepos name="s" site="s1" nsample="3"/></sensor>` (dim=3)
**After:** Model build
**Assert:** `model.nhistory == 14` (3 × (3+1) + 2 = 14), `model.sensor_historyadr[0] == 0`
**Field:** `Model.nhistory`, `Model.sensor_historyadr`
**MuJoCo 3.5.0 verified:** EGT-3

### AC7: Mixed actuator + sensor historyadr layout *(runtime test — MuJoCo-verified)*
**Given:** MJCF with motor actuator `nsample=4` (dim=1) + jointpos sensor `nsample=3` (dim=1)
**After:** Model build
**Assert:** `model.actuator_historyadr[0] == 0`, `model.sensor_historyadr[0] == 10`, `model.nhistory == 18`
**Field:** `Model.actuator_historyadr`, `Model.sensor_historyadr`, `Model.nhistory`
**MuJoCo 3.5.0 verified:** EGT-6

### AC8: Default values when no history attrs specified *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1"/></sensor>` (no history attrs)
**After:** Model build
**Assert:** `model.sensor_nsample[0] == 0`, `model.sensor_interp[0] == InterpolationType::Zoh`, `model.sensor_delay[0] == 0.0`, `model.sensor_interval[0] == (0.0, 0.0)`, `model.sensor_historyadr[0] == -1`, `model.nhistory == 0`
**Field:** All 5 new model fields + `nhistory`
**MuJoCo 3.5.0 verified:** EGT-2

### AC9: Validation — delay without nsample *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" delay="0.01"/></sensor>`
**After:** Model build attempt
**Assert:** Error containing "setting delay > 0 without a history buffer"
**Field:** Build error
**MuJoCo 3.5.0 verified:** EGT-4

### AC10: Validation — negative interval *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" interval="-0.5"/></sensor>`
**After:** Model build attempt
**Assert:** Error containing "negative interval in sensor"
**Field:** Build error
**MuJoCo 3.5.0 verified:** EGT-11b

### AC11: Invalid interp keyword rejected *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" nsample="5" interp="Linear"/></sensor>`
**After:** Model build attempt
**Assert:** Error containing "invalid interp keyword"
**Field:** Build error
**MuJoCo 3.5.0 verified:** EGT-11c

### AC12: Multiple sensors with history — offset accumulation *(runtime test — MuJoCo-verified)*
**Given:** 3 sensors: `jointpos j1 nsample=3` (dim=1), `jointpos j2 nsample=5` (dim=1), `framepos s1 nsample=2` (dim=3)
**After:** Model build
**Assert:** `sensor_historyadr == [0, 8, 20]`, `nhistory == 30`
**Field:** `Model.sensor_historyadr`, `Model.nhistory`
**MuJoCo 3.5.0 verified:** EGT-11e

### AC13: nsample=0 in middle doesn't break chain *(runtime test — MuJoCo-verified)*
**Given:** 3 sensors: `jointpos j1 nsample=3` (dim=1), `jointpos j2` (no nsample), `jointpos j3 nsample=5` (dim=1)
**After:** Model build
**Assert:** `sensor_historyadr == [0, -1, 8]`, `nhistory == 20`
**Field:** `Model.sensor_historyadr`, `Model.nhistory`
**MuJoCo 3.5.0 verified:** EGT-11f

### AC14: Full combination — all 4 attrs on one sensor *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><framepos name="s" site="s1" nsample="5" interp="cubic" delay="0.02" interval="0.1"/></sensor>` (dim=3)
**After:** Model build
**Assert:** `sensor_nsample[0] == 5`, `sensor_interp[0] == InterpolationType::Cubic`, `sensor_delay[0] == 0.02`, `sensor_interval[0] == (0.1, 0.0)`, `sensor_historyadr[0] == 0`, `nhistory == 22` (5×(3+1)+2)
**Field:** All 5 new model fields + `nhistory`
**MuJoCo 3.5.0 verified:** EGT-11i

### AC15: Actuator stability — adding sensors doesn't shift actuator offsets *(runtime test — MuJoCo-verified)*
**Given:** 2 actuators (nsample=4,2) + 2 sensors (nsample=3,5, dim=1)
**After:** Model build
**Assert:** `actuator_historyadr == [0, 10]`, `sensor_historyadr == [16, 24]`, `nhistory == 36`
**Field:** `Model.actuator_historyadr`, `Model.sensor_historyadr`, `Model.nhistory`
**MuJoCo 3.5.0 verified:** EGT-11g

### AC16: Negative nsample accepted *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" nsample="-1"/></sensor>`
**After:** Model build
**Assert:** `sensor_nsample[0] == -1`, `sensor_historyadr[0] == -1`, `nhistory == 0`
**Field:** `Model.sensor_nsample`, `Model.sensor_historyadr`, `Model.nhistory`
**MuJoCo 3.5.0 verified:** EGT-5

### AC17: Negative delay accepted *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" nsample="5" delay="-0.01"/></sensor>`
**After:** Model build
**Assert:** `sensor_delay[0] == -0.01`, no error
**Field:** `Model.sensor_delay`
**MuJoCo 3.5.0 verified:** EGT-11j

### AC18: interval without nsample accepted *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" interval="0.5"/></sensor>`
**After:** Model build
**Assert:** `sensor_interval[0] == (0.5, 0.0)`, `sensor_historyadr[0] == -1`, no error
**Field:** `Model.sensor_interval`, `Model.sensor_historyadr`
**MuJoCo 3.5.0 verified:** EGT-11a

### AC19: interp stored even without nsample *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" interp="linear"/></sensor>`
**After:** Model build
**Assert:** `sensor_interp[0] == InterpolationType::Linear`, `sensor_historyadr[0] == -1`
**Field:** `Model.sensor_interp`, `Model.sensor_historyadr`
**MuJoCo 3.5.0 verified:** EGT-11d

### AC20: Empty-string interp rejected *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" nsample="5" interp=""/></sensor>`
**After:** Model build attempt
**Assert:** Error containing "invalid interp keyword"
**Field:** Build error
**MuJoCo 3.5.0 verified:** EGT-11c

### AC21: Sensor-only model (no actuators) *(runtime test — MuJoCo-verified)*
**Given:** MJCF with no actuators, sensor `jointpos nsample=3` (dim=1)
**After:** Model build
**Assert:** `sensor_historyadr[0] == 0`, `nhistory == 8`, `actuator_historyadr` is empty
**Field:** `Model.sensor_historyadr`, `Model.nhistory`, `Model.actuator_historyadr`
**MuJoCo 3.5.0 verified:** EGT-11k

### AC22: Vec-length invariant *(runtime test — analytically derived)*
**Given:** MJCF with 3 sensors (some with history, some without)
**After:** Model build
**Assert:** `sensor_nsample.len() == 3`, `sensor_interp.len() == 3`, `sensor_delay.len() == 3`, `sensor_interval.len() == 3`, `sensor_historyadr.len() == 3`
**Field:** All 5 new `Vec` fields

### AC23: delay + interval without nsample → delay error *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos name="s" joint="j1" delay="0.01" interval="0.5"/></sensor>`
**After:** Model build attempt
**Assert:** Error containing "setting delay > 0 without a history buffer"
**Field:** Build error
**MuJoCo 3.5.0 verified:** EGT-4 (interval does NOT override nsample requirement)

### AC24: Mixed actuator + multi-dim sensor layout *(runtime test — analytically derived)*
**Given:** Motor actuator on j1 with `nsample=4` (dim=1) + framepos sensor on s1 with `nsample=3` (dim=3)
**After:** Model build
**Assert:** `actuator_historyadr[0] == 0`, `sensor_historyadr[0] == 10`, `nhistory == 24` (actuator: 2×4+2=10, sensor: 3×(3+1)+2=14, total: 10+14=24)
**Field:** `Model.actuator_historyadr`, `Model.sensor_historyadr`, `Model.nhistory`

### AC25: Structural completeness *(code review — not a runtime test)*
- All 5 new model fields initialized in `Model::empty()` (`model_init.rs`)
- All 4 new attrs parsed in `parse_sensor_attrs()` (`parser.rs`)
- `parse_sensor_defaults()` NOT extended with history attrs (per AD-3)
- `apply_to_sensor()` (`defaults.rs`) NOT modified
- `merge_sensor_defaults()` (`defaults.rs`) NOT modified
- History attr pushes in same block as existing 11 fields at `sensor.rs:83–100`
- `model.rs` nhistory comment updated from actuator-only to actuator+sensor
- `sensor.rs` doc comments updated from 13 to 17
- `data.rs:78` doc comment updated from 'Actuator history buffer' to 'History buffer (actuators + sensors)'
- `enums.rs:253–254` doc comment updated to include sensor history usage
- `sensor.rs:28` and `sensor.rs:467` stale doc comments updated

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (parse nsample) | T1 | Direct |
| AC2 (parse interp) | T1 | Direct |
| AC3 (parse delay) | T1 | Direct |
| AC4 (parse interval) | T2 | Direct |
| AC5 (nhistory dim=1) | T3 | Direct |
| AC6 (nhistory multi-dim) | T3 | Direct |
| AC7 (mixed actuator+sensor) | T4 | Direct |
| AC8 (defaults) | T5 | Direct |
| AC9 (delay validation) | T6 | Direct |
| AC10 (negative interval) | T7 | Direct |
| AC11 (invalid interp) | T8 | Direct |
| AC12 (multi-sensor offsets) | T9 | Direct |
| AC13 (nsample=0 in middle) | T9 | Direct |
| AC14 (full combination) | T10 | Direct |
| AC15 (actuator stability) | T11 | Direct |
| AC16 (negative nsample) | T12 | Direct |
| AC17 (negative delay) | T12 | Direct |
| AC18 (interval without nsample) | T13 | Direct |
| AC19 (interp without nsample) | T13 | Direct |
| AC20 (empty interp) | T8 | Direct |
| AC21 (sensor-only model) | T14 | Direct |
| AC22 (vec-length invariant) | T9 | Direct (verified in multi-sensor test) |
| AC23 (delay+interval no nsample) | T6 | Direct |
| AC24 (mixed act+multi-dim sensor) | T16 | Direct |
| AC25 (structural) | — | Code review (manual) |

---

## Test Plan

All tests in `sim/L0/tests/integration/sensor_phase6.rs` (following the
Phase 5 pattern: `actuator_phase5.rs`).

### T1: Parse basic history attributes → AC1, AC2, AC3
**Model:** Single hinge joint + jointpos sensor with `nsample=5 interp="linear" delay="0.02"`.
**Assert:** `sensor_nsample[0] == 5`, `sensor_interp[0] == Linear`, `sensor_delay[0] == 0.02`.
**Expected source:** Analytically derived from MuJoCo semantics.

### T2: Parse interval attribute → AC4
**Model:** Single hinge joint + jointpos sensor with `interval="0.5"`.
**Assert:** `sensor_interval[0] == (0.5, 0.0)`.
**Expected source:** MuJoCo 3.5.0 verified (EGT-7).

### T3: nhistory formula — dim=1, dim=3, dim=4 → AC5, AC6
**Model:** Three sensors: `jointpos nsample=5` (dim=1), `framepos nsample=3` (dim=3), `framequat nsample=1` (dim=4).
**Assert:** Sensor 0: `historyadr=0`, contribution=12 (5×2+2). Sensor 1: `historyadr=12`, contribution=14 (3×4+2). Sensor 2: `historyadr=26`, contribution=7 (1×5+2). `nhistory=33`.
**Expected source:** MuJoCo 3.5.0 formula `nsample * (dim + 1) + 2` verified for all 3 dims (EGT-3).

### T4: Mixed actuator + sensor buffer layout → AC7
**Model:** Motor actuator on j1 with `nsample=4` + jointpos sensor on j1 with `nsample=3`.
**Assert:** `actuator_historyadr[0] == 0`, `sensor_historyadr[0] == 10`, `nhistory == 18`.
**Expected source:** MuJoCo 3.5.0 verified (EGT-6).

### T5: Default values — no history attrs → AC8
**Model:** Single jointpos sensor, no history attributes.
**Assert:** `sensor_nsample[0]==0`, `sensor_interp[0]==Zoh`, `sensor_delay[0]==0.0`, `sensor_interval[0]==(0.0,0.0)`, `sensor_historyadr[0]==-1`, `nhistory==0`.
**Expected source:** MuJoCo 3.5.0 verified (EGT-2).

### T6: Validation — delay without nsample → AC9, AC23
**Model:** Jointpos sensor with `delay="0.01"` (no nsample). Also test `delay="0.01" interval="0.5"` (no nsample).
**Assert:** Both produce error containing "setting delay > 0 without a history buffer".
**Expected source:** MuJoCo 3.5.0 verified (EGT-4).

### T7: Validation — negative interval → AC10
**Model:** Jointpos sensor with `interval="-0.5"`.
**Assert:** Error containing "negative interval in sensor".
**Expected source:** MuJoCo 3.5.0 verified (EGT-11b).

### T8: Invalid interp keywords → AC11, AC20
**Model:** Separate build attempts with `interp="Linear"`, `interp="ZOH"`, `interp="1"`, `interp=""`.
**Assert:** All 4 produce error containing "invalid interp keyword".
**Expected source:** MuJoCo 3.5.0 verified (EGT-11c).

### T9: Multi-sensor historyadr accumulation → AC12, AC13, AC22
**Model (a):** 3 sensors: jointpos j1 nsample=3, jointpos j2 nsample=5, framepos s1 nsample=2.
**Assert (a):** `historyadr == [0, 8, 20]`, `nhistory == 30`.
**Model (b):** 3 sensors: jointpos j1 nsample=3, jointpos j2 (no nsample), jointpos j3 nsample=5.
**Assert (b):** `historyadr == [0, -1, 8]`, `nhistory == 20`.
**Also verify:** All new Vec fields have `len() == 3`.
**Expected source:** MuJoCo 3.5.0 verified (EGT-11e, EGT-11f).

### T10: Full combination — all 4 attrs → AC14
**Model:** Framepos sensor with `nsample=5 interp="cubic" delay="0.02" interval="0.1"` (dim=3).
**Assert:** `nsample==5`, `interp==Cubic`, `delay==0.02`, `interval==(0.1,0.0)`, `historyadr==0`, `nhistory==22`.
**Expected source:** MuJoCo 3.5.0 verified (EGT-11i).

### T11: Actuator stability — sensors don't shift actuator offsets → AC15
**Model:** 2 motor actuators (nsample=4, nsample=2) + 2 jointpos sensors (nsample=3, nsample=5).
**Assert:** `actuator_historyadr==[0,10]`, `sensor_historyadr==[16,24]`, `nhistory==36`.
**Expected source:** MuJoCo 3.5.0 verified (EGT-11g).

### T12: Negative nsample and negative delay accepted → AC16, AC17
**Model (a):** Jointpos sensor with `nsample="-1"`.
**Assert (a):** `sensor_nsample[0] == -1`, `historyadr[0] == -1`, `nhistory == 0`.
**Model (b):** Jointpos sensor with `nsample="5" delay="-0.01"`.
**Assert (b):** `sensor_delay[0] == -0.01`, no error.
**Expected source:** MuJoCo 3.5.0 verified (EGT-5, EGT-11j).

### T13: Attributes stored without nsample → AC18, AC19
**Model (a):** Jointpos sensor with `interval="0.5"` (no nsample).
**Assert (a):** `interval==(0.5,0.0)`, `historyadr==-1`, no error.
**Model (b):** Jointpos sensor with `interp="linear"` (no nsample).
**Assert (b):** `interp==Linear`, `historyadr==-1`, no error.
**Expected source:** MuJoCo 3.5.0 verified (EGT-11a, EGT-11d).

### T14: Sensor-only model (no actuators) → AC21
**Model:** Body with site + framepos sensor nsample=3 (dim=3), no actuators.
**Assert:** `sensor_historyadr[0] == 0`, `nhistory == 14` (3×(3+1)+2), `actuator_historyadr` is empty.
**Expected source:** MuJoCo 3.5.0 verified (EGT-11k, adapted for dim=3).

### T15: Build regression — existing model without history attrs
**Model:** Any existing MJCF model from the test fixtures that has sensors (no history attrs).
**Assert:** Model builds successfully with same `nsensor`, `nsensordata`, and `nhistory == 0` for sensors. All new Vec fields have correct length.
**Rationale:** Proves Spec D changes don't regress existing sensor pipeline.

### T16: Mixed actuator + multi-dim sensor layout → AC24
**Model:** Motor actuator on j1 with `nsample=4` (dim=1) + framepos sensor on s1 with `nsample=3` (dim=3).
**Assert:** `actuator_historyadr[0] == 0`, `sensor_historyadr[0] == 10`, `nhistory == 24` (actuator: 10, sensor: 3×4+2=14, total: 24).
**Expected source:** Analytically derived from EGT-3 formula + EGT-6 layout pattern.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| `nsample=0` → no history buffer | Default case; must not allocate | T5 | AC8 |
| Negative `nsample=-1` → accepted | MuJoCo stores as-is; treated as no history | T12 | AC16 |
| `delay > 0` without `nsample > 0` → error | MuJoCo validation — delay requires history buffer | T6 | AC9 |
| Negative `delay=-0.01` → accepted | MuJoCo accepts negative delay (validation is strictly `> 0.0`) | T12 | AC17 |
| `interval < 0` → error | MuJoCo rejects negative interval | T7 | AC10 |
| `interval > 0` without `nsample` → accepted | interval is independent of history buffer | T13 | AC18 |
| `interp` stored without `nsample` | MuJoCo stores interp even when nsample=0 | T13 | AC19 |
| `interp` keyword mapping: zoh→0, linear→1, cubic→2 | Correct enum mapping from string | T1, T10 | AC2, AC14 |
| Case-sensitive interp keywords | "Linear"/"ZOH"/"1" all rejected | T8 | AC11 |
| Empty-string `interp=""` → error | Distinct from mis-cased keyword | T8 | AC20 |
| `nsample=0` in middle of chain | Must not break offset accumulation | T9 | AC13 |
| Multi-dim sensor nhistory formula (dim=1, dim=3, dim=4) | Different dims produce different buffer sizes; formula generality | T3 | AC5, AC6 |
| `interval` attribute parsed and stored | New attribute not in umbrella original scope | T2 | AC4 |
| Default values when no attrs specified | All 6 fields must have correct defaults | T5 | AC8 |
| Mixed actuator + sensor layout (dim=1) | Sensors start after actuator total | T4 | AC7 |
| Mixed actuator + multi-dim sensor (dim=3) | Verifies generalized formula in mixed layout | T16 | AC24 |
| Sensor-only model (no actuators) | Sensors start at offset 0 | T14 | AC21 |
| Full combination (all 4 attrs) | All attributes work together on one sensor | T10 | AC14 |
| Actuator stability guarantee | Adding sensors must NOT shift actuator offsets | T11 | AC15 |
| `delay + interval` without nsample | Interval does NOT override nsample requirement for delay | T6 | AC23 |
| Vec lengths match nsensor | All new arrays aligned with existing sensor arrays | T9, T15 | AC22 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T15 (build regression) | Existing model without history attrs still builds | Ensures Spec D doesn't regress — not tied to a specific AC but validates pipeline integrity |

---

## Risk & Blast Radius

**EGT-10 inventory:** 12 code modification sites across 7 files (S1–S9) +
4 non-modification sites per AD-3(b) + 6 stale documentation sites (S10) +
2 runtime non-modification sites (make_data, reset_data).

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `nhistory` includes sensors | Actuator contributions only | Actuator + sensor contributions | Toward MuJoCo | Code reading `model.nhistory` (allocation sizing) | None — `make_data()` already allocates `vec![0.0; self.nhistory]` and grows automatically |
| MJCF `nsample`/`interp`/`delay`/`interval` on sensors | Silently ignored | Parsed and stored in model | Toward MuJoCo | MJCF files with these attrs | None — previously ignored attrs now take effect |
| Delay/nsample validation on sensors | No validation | `delay > 0.0 && nsample <= 0` → error | Toward MuJoCo | MJCF with `delay` on sensors without `nsample` | Add `nsample` or remove `delay` |
| Negative interval validation | No validation | `interval < 0.0` → error | Toward MuJoCo | MJCF with negative `interval` | Fix to non-negative |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `mjcf/src/types.rs` | Add 4 fields to `MjcfSensor`, update default, add 4 fluent methods | +30 |
| `mjcf/src/parser.rs` | Parse 4 attrs in `parse_sensor_attrs()` | +4 |
| `mjcf/src/builder/mod.rs` | Add 4 fields to `ModelBuilder` | +4 |
| `mjcf/src/builder/init.rs` | Init 4 new fields to `vec![]` | +4 |
| `mjcf/src/builder/sensor.rs` | Validate + push 4 history attrs, update doc comments | +25 |
| `mjcf/src/builder/build.rs` | Transfer 4 fields + init historyadr + extend post-hoc computation | +20 / ~15 modified |
| `core/src/types/model.rs` | Add 5 fields, update nhistory comment | +20 |
| `core/src/types/model_init.rs` | Init 5 new fields to `vec![]` | +5 |
| `core/src/types/data.rs` | Update doc comment (line 78) | ~1 |
| `core/src/types/enums.rs` | Update doc comment (lines 253–254) | ~2 |
| `tests/integration/sensor_phase6.rs` | 15 new tests | +400 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_apply_to_sensor` | `defaults.rs:1220` | Pass (unchanged) | History attrs not added to defaults per AD-3(b) |
| `test_sensor_defaults_inheritance` | `defaults.rs:1259` | Pass (unchanged) | Same — defaults untouched |
| All sensor pipeline tests | Various | Pass (unchanged) | New fields are additive; existing sensors get default values (nsample=0, etc.) |
| Phase 5 actuator tests | `actuator_phase5.rs` | Pass (unchanged) | Actuator historyadr loop unchanged; sensor loop appended after |
| Full sim domain (2,238+ tests) | Various | Pass (unchanged) | No evaluation-level changes; parse-and-store only |

### Non-modification sites (explicitly NOT touched)

| File | Line | Function | Why not modified |
|------|------|----------|-----------------|
| `defaults.rs` | 533 | `apply_to_sensor()` | Only applies `noise`/`cutoff`/`user` — history attrs not in defaults per AD-3(b) |
| `defaults.rs` | 815 | `merge_sensor_defaults()` | Only merges `noise`/`cutoff`/`user` — same reason |
| `parser.rs` | 909 | `parse_sensor_defaults()` | Only parses `noise`/`cutoff`/`user` — MuJoCo has no sensor defaults (EGT-11h) |
| `types.rs` | 775 | `MjcfSensorDefaults` struct | CortenForge-only extension for `noise`/`cutoff`/`user` — not extended |
| `model_init.rs` | 388 | `make_data()` | Allocates `vec![0.0; self.nhistory]` — auto-grows with `nhistory`, no change needed |
| `data.rs` | 874 | `reset_data()` | Pre-populates actuator history only — sensor pre-population is runtime scope (deferred) |

---

## Execution Order

1. **S1** (types.rs — MjcfSensor fields) → compile check: struct literal completeness
2. **S2** (parser.rs — parse 4 attrs) → compile check: parser builds
3. **S3 + S4** (builder/mod.rs + builder/init.rs — ModelBuilder fields) → compile check: struct literal completeness
4. **S5** (builder/sensor.rs — validate + push) → run `cargo test -p sim-mjcf`
5. **S7 + S8** (model.rs + model_init.rs — Model fields) → compile check: struct literal completeness
6. **S6** (build.rs — transfer + historyadr) → run `cargo test -p sim-mjcf -p sim-core`
7. **S9** (build.rs — extend post-hoc computation) → run `cargo test -p sim-mjcf -p sim-core`
8. **S10** (doc comment updates) → no behavioral change
9. **Tests** (sensor_phase6.rs — T1–T16) → full domain test run

Rationale: S1→S2 establishes parse-side fields. S3→S4→S5 establishes builder
accumulation. S7→S8 establishes model storage. S6→S9 wires builder to model
and computes derived fields. S10 is cosmetic. Tests last, after all production
code is in place.

---

## Out of Scope

- **Runtime sensor interpolation** — `nsample`/`interp`/`delay` runtime behavior
  (reading from history buffer during `mj_forward`) — tracked as DT-107/DT-108.
  *Conformance impact: none for v1.0 — interpolation is a MuJoCo 3.x feature
  not exercised by standard conformance tests.*

- **Sensor history pre-population in `reset_data()`** — MuJoCo pre-populates
  actuator history in `mj_resetData`; sensor pre-population would follow the
  same pattern but is runtime scope. *Conformance impact: none until runtime
  interpolation is implemented.*

- **Adding history attrs to `MjcfSensorDefaults`** — MuJoCo has no sensor
  default class mechanism (EGT-11h). The existing CortenForge sensor defaults
  extension (`noise`/`cutoff`/`user`) is left as-is. A future reconciliation
  may remove the CortenForge extension or align it with MuJoCo's behavior.
  *Conformance impact: none — MuJoCo rejects sensor defaults entirely.*

- **`sensor_intprm` array** — `mjmodel.h:1213` (`nsensor × mjNSENS=3`).
  Separate from `sensor_history`. Used for user/plugin sensor integer params.
  Not needed for standard sensor types. *Conformance impact: minor — only
  affects user sensors with custom integer parameters.*

- **DT-65** (`User` sensor `dim` attribute) — deferred to Post-v1.0.
  *Conformance impact: minimal.*
