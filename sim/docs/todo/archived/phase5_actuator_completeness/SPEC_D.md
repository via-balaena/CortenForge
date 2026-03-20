# Spec D — Interpolation Actuator Attributes: Spec

**Status:** Implemented
**Phase:** Roadmap Phase 5 — Actuator Completeness
**Effort:** M
**MuJoCo ref:** `mjsActuator_` in `mjspec.h`, `mjModel` in `mjmodel.h`, `mjData` in `mjdata.h`
**MuJoCo version:** 3.5.0
**Prerequisites:**
- T1-b §63 `dynprm` resize (landed in `d4db634`)
- Spec A acc0/dampratio/lengthrange (landed in `a1cbbba`)
- Spec B transmission types + slider-crank (landed in `aa87169`)

**Independence:** Spec D is independent of Spec A, Spec B, and Spec C per the
umbrella dependency graph (Step 2 parallel). The only shared file is
`builder/actuator.rs` — Spec D adds new attribute parsing in a separate block
(interp/nsample/delay) that does not conflict with Spec A's dampratio parsing
or Spec B's cranklength/slidersite parsing.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Problem Statement

**Conformance gap.** MuJoCo supports three interpolation-related attributes on
all actuator types: `nsample` (history buffer sample count), `interp`
(interpolation method keyword: `"zoh"`, `"linear"`, `"cubic"`), and `delay`
(time delay in seconds). These attributes control a circular history buffer
used by `mj_forward` to apply delayed/interpolated ctrl signals. MuJoCo
stores these in compiled model arrays (`actuator_history`, `actuator_historyadr`,
`actuator_delay`) and allocates a pre-populated `Data.history` buffer.

CortenForge does not parse, store, or allocate any of these attributes.
Models containing `nsample`, `interp`, or `delay` attributes on actuators are
silently accepted with the attributes ignored — a conformance gap that affects
any MuJoCo model using actuator delay or history-based interpolation.

This spec closes the **model-building** conformance gap: MJCF parsing, compiled
model storage, history buffer allocation with pre-populated initial state, and
compiler validation. Runtime consumption of the history buffer (delayed ctrl
lookup, interpolation during `mj_forward`) is deferred to future work — but
the model and data structures will be **identical** to MuJoCo's after this
spec lands, so a future runtime implementation needs no model-building changes.

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream —
> the algorithm, the acceptance criteria, the tests — is derived from what's
> documented here. If this section is wrong, everything is wrong.

### Spec-level structures (`mjspec.h`)

`mjsActuator_` in `mjspec.h`:
- `nsample` (int, line 701) — number of history samples. Default: 0 (no history).
- `interp` (int, line 702) — interpolation type. Maps from MJCF keywords:
  `"zoh"` → 0, `"linear"` → 1, `"cubic"` → 2. **The MJCF attribute takes
  string keywords, not integers.** Invalid keywords (including `"ZOH"`,
  `"Linear"`, `"1"`) produce an XML parse error. Default: 0 (ZOH).
- `delay` (double, line 703) — time delay in seconds. Default: 0.0.

### Compiled model structures (`mjmodel.h`)

`mjModel` in `mjmodel.h`:
- `actuator_history` (int, `nu × 2`, line 1184) — packed `[nsample, interp]`
  per actuator. **Present for ALL actuators** including those with `nsample=0`.
  `interp` can be set independently of `nsample` (e.g., `interp="cubic"` with
  `nsample=0` stores `[0, 2]` — no error).
- `actuator_historyadr` (int, `nu × 1`, line 1185) — cumulative offset into
  `Data.history` buffer. Equals `-1` when `nsample <= 0` for that actuator.
- `actuator_delay` (mjtNum, `nu × 1`, line 1186) — present for ALL actuators.
  Default: 0.0 even when no history buffer.
- `nhistory` (int, line 764) — total size of history buffer. Computed as
  `Σ (2 * nsample_i + 2)` across all actuators **and sensors** with
  `nsample_i > 0`.

### Data structures (`mjdata.h`)

`mjData` in `mjdata.h`:
- `history` (mjtNum, `nhistory × 1`, line 259) — shared history buffer for
  actuators and sensors. Actuators occupy the first portion, sensors follow.

### History buffer stride and layout

Each actuator with `nsample > 0` occupies `2 * nsample + 2` mjtNums:

```
[metadata0, metadata1, time_0, time_1, ..., time_{n-1}, value_0, value_1, ..., value_{n-1}]
```

- 2 metadata slots
- `nsample` contiguous timestamps
- `nsample` contiguous **ctrl** values (NOT act — verified with
  `dyntype=integrator`: history stores ctrl=1.0 while act=0.0)
- NOT interleaved `(time, value)` pairs

The stride `2 * nsample + 2` is a special case of the general formula
`nsample * (dim + 1) + 2` for dim=1. Sensors use varying dim values:

| sensor type | dim | nsample | stride = ns*(dim+1)+2 | Verified |
|-------------|-----|---------|----------------------|----------|
| jointpos | 1 | 3 | 8 | Yes (MuJoCo 3.5.0) |
| framepos | 3 | 4 | 18 | Yes (MuJoCo 3.5.0) |
| framequat | 4 | 3 | 17 | Yes (MuJoCo 3.5.0) |
| framequat | 4 | 5 | 27 | Yes (MuJoCo 3.5.0) |

Sensor history is **out of scope** for this spec. The general formula is
documented here so future sensor history implementation has a verified
starting point.

### `historyadr` computation

`historyadr[j] = Σ_{i<j} (2 * nsample_i + 2)` for actuators with
`nsample_i > 0`. Actuators with `nsample <= 0` get `historyadr = -1` and
consume no buffer space.

Verification data (MuJoCo 3.5.0):

| Model | nsample | historyadr | nhistory |
|-------|---------|-----------|----------|
| 1 act, ns=4 | [4] | [0] | 10 |
| 2 act, ns=3,5 | [3,5] | [0,8] | 20 |
| 3 act, ns=2,4,6 | [2,4,6] | [0,6,16] | 30 |
| 1 act, ns=1 | [1] | [0] | 4 |
| 2 act, ns=4,0 | [4,0] | [0,-1] | 10 |
| 3 act, ns=3,0,2 | [3,0,2] | [0,-1,8] | 14 |

### `nhistory` computation

`nhistory = Σ (2 * nsample_i + 2)` across all actuators and sensors with
`nsample_i > 0`.

**Conformance caveat:** MuJoCo's `nhistory` includes both actuator AND sensor
contributions. CortenForge does not currently parse sensor `nsample`, so for
models that use sensor history, CortenForge's `nhistory` will be smaller than
MuJoCo's. This is a **known conformance gap**. Resolution: when sensor history
attributes are implemented (future spec), add sensor contributions to
`nhistory`. Ordering in the shared buffer: actuators first, then sensors.

### Stride verification (MuJoCo 3.5.0, exhaustive)

| nsample | nhistory | 2n+2 | Match |
|---------|----------|------|-------|
| 1 | 4 | 4 | Yes |
| 2 | 6 | 6 | Yes |
| 3 | 8 | 8 | Yes |
| 4 | 10 | 10 | Yes |
| 5 | 12 | 12 | Yes |
| 6 | 14 | 14 | Yes |
| 7 | 16 | 16 | Yes |
| 8 | 18 | 18 | Yes |
| 10 | 22 | 22 | Yes |
| 16 | 34 | 34 | Yes |
| 32 | 66 | 66 | Yes |

### Initial history buffer state

MuJoCo pre-populates the history buffer when `MjData` is created (and restores
this state on `mj_resetData` and `mj_resetDataKeyframe`). It does NOT
zero-initialize.

Per-actuator buffer initial state (nsample=`n`, timestep=`ts`):
- `metadata0` = 0.0
- `metadata1` = float(nsample - 1)
- `times` = `[-(n)*ts, -(n-1)*ts, ..., -ts]` — `n` evenly-spaced past
  timestamps ending at `-ts` (NOT at 0)
- `values` = `[0.0, 0.0, ..., 0.0]` — all zero

Verification data (MuJoCo 3.5.0):

| nsample | timestep | meta0 | meta1 | times | values |
|---------|----------|-------|-------|-------|--------|
| 4 | 0.002 | 0.0 | 3.0 | [-0.008, -0.006, -0.004, -0.002] | [0, 0, 0, 0] |
| 4 | 0.01 | 0.0 | 3.0 | [-0.04, -0.03, -0.02, -0.01] | [0, 0, 0, 0] |
| 2 | 0.002 | 0.0 | 1.0 | [-0.004, -0.002] | [0, 0] |
| 1 | 0.002 | 0.0 | 0.0 | [-0.002] | [0] |

Multi-actuator initial state (ns=3 + ns=2, ts=0.002):
- Act 0 (adr=0): meta=[0.0, 2.0], times=[-0.006, -0.004, -0.002], values=[0,0,0]
- Act 1 (adr=8): meta=[0.0, 1.0], times=[-0.004, -0.002], values=[0,0]
- Each actuator's times are computed independently from its own nsample.

`mj_resetData` restores this exact state — verified with `np.allclose` against
freshly-created `MjData`, including after 10 steps of mutation on a
multi-actuator model. `mj_resetDataKeyframe` also restores this same state.

`mj_step` writes to the buffer as a circular buffer — `metadata1` cycles
through 0..nsample-1 as the write-head advances, overwriting times and values
with current simulation time and ctrl values. This is **deferred** runtime
behavior, but explains why the initial state is pre-populated: the buffer is
primed for immediate runtime use.

### Forward dynamics interaction

`mj_forward` READS from the history buffer to compute delayed ctrl. With a
delayed actuator (delay=0.006, kp=100), setting `ctrl=1.0` on a fresh `MjData`
(history values all 0.0) and calling `mj_forward` produces
`actuator_force=[0.]` — the delayed value from the history buffer, NOT the
current ctrl. Without delay, the same ctrl produces `force=[100.]`.

This means the pre-populated initial buffer state **directly affects forward
dynamics from the very first call**. The initial buffer metadata and times are
not just a data structure detail — they determine what the simulation "sees"
as the delayed ctrl signal. Getting the initial state wrong would produce
incorrect forces from the first step.

### Compiler validation

- **`delay > 0` with `nsample = 0`** → compile error:
  `"Error: setting delay > 0 without a history buffer"`.
- **`nsample > 0` with `delay = 0`** → valid (buffer allocated, no delay).
- **`nsample` negative values** — silently accepted: `historyadr = -1`, no
  buffer allocation. MuJoCo performs no validation on `nsample` sign.
- **`interp="cubic"` with `nsample < 4`** — silently accepted. MuJoCo has no
  minimum nsample requirement for cubic interpolation.
- **Delay exceeding buffer capacity** (e.g., delay=0.1 with nsample=2,
  timestep=0.002 → buffer covers ~0.004s) — silently accepted. No validation.

**Validation categorization:** All five validation behaviors above match MuJoCo
exactly. CortenForge adds no stricter validation for these attributes — we
accept everything MuJoCo accepts and reject everything MuJoCo rejects.

### Default class inheritance

`nsample`, `interp`, and `delay` are inheritable through `<default>` classes.
Child elements can override individual attributes (e.g., inherit
`interp="linear"` and `delay=0.02` from class, override `nsample=3`).

### Actuator type compatibility

All actuator types accept `nsample`, `interp`, `delay`: `<position>`,
`<motor>`, `<velocity>`, `<general>` (and all other shortcuts). These are
common actuator attributes, not `<general>`-specific.

### Runtime API functions (deferred)

- `mj_readCtrl()` in `mujoco.h` — reads ctrl with optional delay/interpolation
- `mj_initCtrlHistory()` in `mujoco.h` — initializes history buffer

Both are documented as **out of scope** (runtime deferred). Cited here to show
what the stored attributes are consumed by.

### Sensor parallel

`mjsSensor_` has identical attributes (`nsample`, `interp`, `delay`) in
`mjspec.h` (lines 731-733) and model arrays (`sensor_history`,
`sensor_historyadr`, `sensor_delay`) in `mjmodel.h` (lines 1218-1220).
Sensor history is **out of scope** for this spec but noted for future work.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| `nsample` parsing | Parsed as int from MJCF, stored in `actuator_history[i,0]` | **Not parsed** — attribute silently ignored |
| `interp` parsing | Parsed as string keyword (`"zoh"`, `"linear"`, `"cubic"` → 0, 1, 2), stored in `actuator_history[i,1]` | **Not parsed** — attribute silently ignored |
| `delay` parsing | Parsed as float from MJCF, stored in `actuator_delay[i]` | **Not parsed** — attribute silently ignored |
| `actuator_history` model array | `int[nu × 2]` — `[nsample, interp]` per actuator, present for ALL actuators | **Does not exist** |
| `actuator_historyadr` model array | `int[nu × 1]` — cumulative offset, -1 for no-history | **Does not exist** |
| `actuator_delay` model array | `mjtNum[nu × 1]` — present for all actuators, default 0.0 | **Does not exist** |
| `nhistory` model field | Total history buffer size (actuators + sensors) | **Does not exist** |
| `Data.history` buffer | `mjtNum[nhistory]` — pre-populated with metadata + past timestamps + zero values | **Does not exist** |
| Compiler validation | `delay > 0` with `nsample = 0` → error | **No validation** (attributes not parsed) |
| Default class inheritance | All three attributes inheritable | **Not applicable** (attributes not parsed) |
| `Data::reset()` history restoration | Restores pre-populated initial state (not zeros) | **Not applicable** |
| `Data::reset_to_keyframe()` history restoration | Restores same pre-populated initial state | **Not applicable** |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `actuator_history` packing | `int[nu × 2]` flat: `[2*i+0]=nsample`, `[2*i+1]=interp` (C int) | Split into `actuator_nsample: Vec<i32>` and `actuator_interp: Vec<InterpolationType>` (Rust enum) | Use two separate Vecs instead of packed `[i32; 2]`. Rationale: split fields are type-safe (enum vs int) and follow CortenForge's existing pattern of one-Vec-per-attribute (e.g., `actuator_gaintype`, `actuator_biastype` are separate, not packed). The packed integer representation exists in MuJoCo for C array efficiency; Rust enums are more idiomatic and prevent storing invalid interp values. |
| `actuator_nsample` type | `int` (signed, accepts negatives) | `Vec<i32>` (signed, matching MuJoCo) | Direct port — use `i32` to preserve MuJoCo's acceptance of negative values. The umbrella Convention Registry §1 suggests `Vec<usize>` (unsigned) — this is **overridden** here because MuJoCo uses signed int and accepts negative values (`nsample=-1` → `historyadr=-1`). Using `usize` would require clamping or rejecting values MuJoCo accepts. |
| `actuator_historyadr` type | `int[nu × 1]` with `-1` sentinel | `Vec<i32>` (signed, `-1` sentinel matches MuJoCo) | Direct port — use `i32` with `-1` sentinel. `Option<usize>` would be more Rust-idiomatic but would diverge from MuJoCo's representation and complicate conformance testing (comparing `Some(0)` vs `0`). |
| `actuator_delay` type | `mjtNum[nu × 1]` | `Vec<f64>` | Direct port — no translation needed |
| `interp` attribute type | MJCF: string keyword (`"zoh"`, `"linear"`, `"cubic"`); compiled model: int (0, 1, 2) | MJCF: parsed as `String`; stored as `InterpolationType` enum | Parse string in MJCF parser, convert to `InterpolationType` enum via `FromStr` (`.parse()` in builder). Variants: `Zoh` (default), `Linear`, `Cubic`. |
| `InterpolationType` default | `"zoh"` (int 0) | `InterpolationType::Zoh` with `#[default]` | The umbrella Convention Registry §1 suggests `InterpType::None` as the name — this is **overridden** because the default interpolation IS zero-order hold, not "none." `None` would be misleading (suggests no interpolation exists, when in fact ZOH is the interpolation method). `Zoh` accurately names what the default does. |
| `nhistory` type | `mjtSize` (int) on `mjModel` | `usize` on `Model` | Direct port — Rust `usize` for sizes |
| `Data.history` type | `mjtNum[nhistory]` | `Vec<f64>` | Direct port — `Vec<f64>` sized to `nhistory` |
| MJCF attribute names | `nsample`, `interp`, `delay` | Same — match MuJoCo exactly per umbrella Convention Registry §4 | Direct port — no translation needed |
| Rust field names | N/A (C uses flat arrays) | `actuator_nsample`, `actuator_interp`, `actuator_delay`, `actuator_historyadr` | snake_case per Convention Registry §1 |

---

## Specification

> **Conformance rule:** Every algorithm in this section must produce numerically
> identical results to the MuJoCo C code cited in the MuJoCo Reference section.

### S1. MJCF Type Additions

**File:** `sim/L0/mjcf/src/types.rs`
**MuJoCo equivalent:** `mjsActuator_` in `mjspec.h` (fields `nsample`, `interp`, `delay`)
**Design decision:** Fields are `Option<T>` to distinguish "attribute present
in XML" from "use default." This matches the existing pattern for all optional
actuator attributes (e.g., `kv: Option<f64>`, `dampratio: Option<f64>`,
`actearly: Option<bool>`). `interp` is stored as `Option<String>` in the MJCF
layer because it's a keyword string in XML — conversion to `InterpolationType`
enum happens in the builder (S4), matching how `dyntype: Option<String>` is
converted to `ActuatorDynamics` in the builder.

**`MjcfActuator` struct** — add three fields in the common section (these
apply to ALL actuator types, not just `<general>`):

```rust
// In MjcfActuator struct, common section (near actearly/lengthrange):
pub nsample: Option<i32>,      // MuJoCo: int, default 0, accepts negatives
pub interp: Option<String>,    // MuJoCo: keyword "zoh"/"linear"/"cubic", default "zoh"
pub delay: Option<f64>,        // MuJoCo: double, default 0.0
```

**`MjcfActuatorDefaults` struct** — add matching fields:

```rust
// In MjcfActuatorDefaults struct:
pub nsample: Option<i32>,
pub interp: Option<String>,
pub delay: Option<f64>,
```

**`Default` impl for `MjcfActuator`** — add `None` for all three:

```rust
nsample: None,
interp: None,
delay: None,
```

### S2. Parser Updates

**File:** `sim/L0/mjcf/src/parser.rs`
**MuJoCo equivalent:** MJCF keyword parsing for actuator elements
**Design decision:** `nsample`, `interp`, `delay` are parsed in the **common**
attribute section of `parse_actuator_attrs()` (lines 2013-2062), NOT inside
the `<general>`-only gate (lines 2064-2076). These attributes apply to ALL
actuator types (`<position>`, `<motor>`, `<velocity>`, `<general>`, etc.).
Placing them inside the general gate would silently ignore them on shortcut
types — a conformance bug.

**`parse_actuator_attrs()`** — add after the existing common attributes
(near `dampratio`, before the general gate):

```rust
// Common attributes — apply to all actuator types
actuator.nsample = get_attribute_opt::<i32>(e, "nsample");
actuator.interp = get_attribute_opt::<String>(e, "interp");
actuator.delay = get_attribute_opt::<f64>(e, "delay");
```

**`parse_actuator_defaults()`** — add matching extraction (this function
parses `<default>` class attributes, same pattern):

```rust
defaults.nsample = get_attribute_opt::<i32>(e, "nsample");
defaults.interp = get_attribute_opt::<String>(e, "interp");
defaults.delay = get_attribute_opt::<f64>(e, "delay");
```

### S3. Defaults Pipeline

**File:** `sim/L0/mjcf/src/defaults.rs`
**MuJoCo equivalent:** Default class inheritance for actuator attributes
**Design decision:** Follow the existing pattern exactly. Both functions must
be updated — missing either one breaks default class inheritance silently.

**`merge_actuator_defaults()`** (line 743) — add in the struct literal using
the `c.field.or(p.field)` pattern (for `i32` and `f64` which are `Copy`):

```rust
nsample: c.nsample.or(p.nsample),
interp: c.interp.clone().or_else(|| p.interp.clone()),
delay: c.delay.or(p.delay),
```

**`apply_to_actuator()`** (line 294) — add using the existing
`if result.field.is_none()` pattern:

```rust
if result.nsample.is_none() {
    result.nsample = defaults.nsample;
}
if result.interp.is_none() {
    result.interp.clone_from(&defaults.interp);
}
if result.delay.is_none() {
    result.delay = defaults.delay;
}
```

### S4. Model Type Additions

**Files:** `sim/L0/core/src/types/model.rs`, `sim/L0/core/src/types/enums.rs`, `sim/L0/core/src/types/model_init.rs`
**MuJoCo equivalent:** `mjModel` fields `actuator_history`, `actuator_historyadr`, `actuator_delay`, `nhistory`
**Design decision:** Split MuJoCo's packed `actuator_history[nu × 2]` into two
separate Vecs (`actuator_nsample` and `actuator_interp`) for type safety.
See Convention Notes for full rationale.

**New fields on `Model`** (add in the actuator section, after `actuator_cranklength`):

```rust
/// History sample count per actuator (length `nu`).
/// MuJoCo: `actuator_history[2*i + 0]`.  Default: 0 (no history).
/// Signed to match MuJoCo — accepts negative values (treated as no history).
pub actuator_nsample: Vec<i32>,

/// Interpolation type per actuator (length `nu`).
/// MuJoCo: `actuator_history[2*i + 1]` as int 0/1/2.  Default: Zoh.
pub actuator_interp: Vec<InterpolationType>,

/// Cumulative offset into `Data.history` per actuator (length `nu`).
/// MuJoCo: `actuator_historyadr`.  Equals -1 when `nsample <= 0`.
pub actuator_historyadr: Vec<i32>,

/// Time delay per actuator in seconds (length `nu`).
/// MuJoCo: `actuator_delay`.  Default: 0.0.  Present for all actuators.
pub actuator_delay: Vec<f64>,

/// Total history buffer size (actuator contributions only).
/// MuJoCo: `nhistory` (includes sensors — ours is actuator-only until
/// sensor history is implemented).
pub nhistory: usize,
```

**`InterpolationType` enum** — add to `sim/L0/core/src/types/enums.rs`
(alongside existing `ActuatorDynamics`, `ActuatorTransmission`):

```rust
/// Interpolation method for actuator history buffer.
/// MuJoCo: `actuator_history[2*i + 1]` stores 0 (ZOH), 1 (linear), 2 (cubic).
/// MJCF keywords: `"zoh"`, `"linear"`, `"cubic"` (lowercase only).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InterpolationType {
    /// Zero-order hold (default). MuJoCo int value: 0.
    #[default]
    Zoh = 0,
    /// Linear interpolation. MuJoCo int value: 1.
    Linear = 1,
    /// Cubic interpolation. MuJoCo int value: 2.
    Cubic = 2,
}

impl std::str::FromStr for InterpolationType {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "zoh" => Ok(Self::Zoh),
            "linear" => Ok(Self::Linear),
            "cubic" => Ok(Self::Cubic),
            _ => Err(format!(
                "invalid interp keyword '{}': expected 'zoh', 'linear', or 'cubic'",
                s
            )),
        }
    }
}

impl From<i32> for InterpolationType {
    fn from(v: i32) -> Self {
        match v {
            0 => Self::Zoh,
            1 => Self::Linear,
            2 => Self::Cubic,
            _ => Self::Zoh, // fallback to default for out-of-range
        }
    }
}
```

**`Model::empty()` in `model_init.rs`** — add empty vecs:

```rust
actuator_nsample: vec![],
actuator_interp: vec![],
actuator_historyadr: vec![],
actuator_delay: vec![],
nhistory: 0,
```

### S5. ModelBuilder and Builder Pipeline

**File:** `sim/L0/mjcf/src/builder/mod.rs`, `sim/L0/mjcf/src/builder/actuator.rs`, `sim/L0/mjcf/src/builder/build.rs`
**MuJoCo equivalent:** Model compilation pipeline — parsing → builder → compiled model
**Design decision:** Follow the existing 3-file pipeline pattern exactly:
(a) `mod.rs` — `ModelBuilder` struct holds intermediate arrays,
(b) `actuator.rs` — `process_actuator()` pushes per-actuator values,
(c) `build.rs` — `build()` transfers arrays and computes `historyadr`/`nhistory`.

**`ModelBuilder` struct in `builder/mod.rs`** — add fields (in the actuator
section, after `actuator_cranklength`):

```rust
pub(crate) actuator_nsample: Vec<i32>,
pub(crate) actuator_interp: Vec<InterpolationType>,
pub(crate) actuator_delay: Vec<f64>,
```

Initialize in `ModelBuilder::new()` (or wherever the default is) as `vec![]`.

**`process_actuator()` in `builder/actuator.rs`** — add push calls in the
push block (near `actuator_cranklength` push, around line 262):

```rust
// Parse interp keyword → InterpolationType enum
let interp = match &actuator.interp {
    Some(s) => s.parse::<InterpolationType>().map_err(|e| {
        MjcfError::InvalidAttribute {
            element: "actuator".into(),
            attribute: "interp".into(),
            message: e,
        }
    })?,
    None => InterpolationType::Zoh,
};

self.actuator_nsample.push(actuator.nsample.unwrap_or(0));
self.actuator_interp.push(interp);
self.actuator_delay.push(actuator.delay.unwrap_or(0.0));
```

**Compiler validation** — add in `process_actuator()` after the push block
(or before it, near other validation):

```rust
// MuJoCo compiler validation: delay > 0 requires nsample > 0
let nsample = actuator.nsample.unwrap_or(0);
let delay = actuator.delay.unwrap_or(0.0);
if delay > 0.0 && nsample <= 0 {
    return Err(MjcfError::InvalidAttribute {
        element: "actuator".into(),
        attribute: "delay".into(),
        message: "setting delay > 0 without a history buffer (nsample must be > 0)".into(),
    });
}
```

**`build()` in `builder/build.rs`** — two changes:

(a) Transfer arrays in the struct literal (lines 244-262 region). Only the
three parsed arrays are transferred here — `actuator_historyadr` and `nhistory`
are intentionally omitted because they are computed post-hoc in step (b):

```rust
actuator_nsample: self.actuator_nsample,
actuator_interp: self.actuator_interp,
actuator_delay: self.actuator_delay,
// NOTE: actuator_historyadr and nhistory are NOT transferred here —
// they are computed as a post-processing step below.
```

(b) Compute `historyadr` and `nhistory` as a post-processing step AFTER all
actuators are processed (around line 386, after `compute_actuator_params()`).
This must happen in `build()` because `historyadr` is a cumulative sum across
all actuators — it cannot be computed in `process_actuator()`:

```rust
// Compute actuator_historyadr and nhistory (must be post-processing
// because historyadr is cumulative across all actuators)
let nu = model.actuator_nsample.len();
let mut historyadr = vec![-1i32; nu];
let mut offset: i32 = 0;
for i in 0..nu {
    let ns = model.actuator_nsample[i];
    if ns > 0 {
        historyadr[i] = offset;
        offset += 2 * ns + 2;
    }
    // ns <= 0: historyadr stays -1
}
model.actuator_historyadr = historyadr;
model.nhistory = offset as usize;
```

### S6. Data Integration

**File:** `sim/L0/core/src/types/data.rs`, `sim/L0/core/src/types/model_init.rs`
**MuJoCo equivalent:** `mjData.history` buffer — allocation, pre-population, reset, clone
**Design decision:** The history buffer is pre-populated in `make_data()` with
the exact same initial state MuJoCo uses: `metadata0=0.0`,
`metadata1=float(nsample-1)`, `times=[-(n)*ts, ..., -ts]`, `values=all 0.0`.
This is NOT just `vec![0.0; nhistory]`. `reset()` and `reset_to_keyframe()`
restore this same state. This is required because `mj_forward` reads from the
history buffer on the very first call — wrong initial state produces wrong
forces.

**`Data` struct in `data.rs`** — add field (in the Control / Actuation section):

```rust
/// Actuator history buffer (length `nhistory`).
/// Pre-populated with metadata + past timestamps + zero ctrl values.
/// MuJoCo: `mjData.history`.
pub history: Vec<f64>,
```

**`Model::make_data()` in `model_init.rs`** — allocate AND pre-populate:

```rust
// Allocate and pre-populate history buffer
history: {
    let mut buf = vec![0.0f64; self.nhistory];
    for i in 0..self.nu {
        let ns = self.actuator_nsample[i];
        if ns <= 0 {
            continue;
        }
        let adr = self.actuator_historyadr[i] as usize;
        let n = ns as usize;
        // metadata0 = 0.0 (already zero)
        // metadata1 = float(nsample - 1)
        buf[adr + 1] = (n - 1) as f64;
        // times = [-(n)*ts, -(n-1)*ts, ..., -ts]
        let ts = self.timestep;
        for k in 0..n {
            buf[adr + 2 + k] = -((n - k) as f64) * ts;
        }
        // values = all 0.0 (already zero)
    }
    buf
},
```

**`Data::reset()` in `data.rs`** — restore pre-populated initial state (NOT
just zero-fill). Add after the existing actuator reset lines:

```rust
// Restore history buffer to pre-populated initial state (matching mj_resetData)
self.history.fill(0.0);
for i in 0..model.actuator_nsample.len() {
    let ns = model.actuator_nsample[i];
    if ns <= 0 {
        continue;
    }
    let adr = model.actuator_historyadr[i] as usize;
    let n = ns as usize;
    // metadata1 = float(nsample - 1)
    self.history[adr + 1] = (n - 1) as f64;
    // times = [-(n)*ts, -(n-1)*ts, ..., -ts]
    let ts = model.timestep;
    for k in 0..n {
        self.history[adr + 2 + k] = -((n - k) as f64) * ts;
    }
}
```

**`Data::reset_to_keyframe()` in `data.rs`** — same history restoration:

```rust
// Restore history buffer to pre-populated initial state (matching mj_resetDataKeyframe)
self.history.fill(0.0);
for i in 0..model.actuator_nsample.len() {
    let ns = model.actuator_nsample[i];
    if ns <= 0 {
        continue;
    }
    let adr = model.actuator_historyadr[i] as usize;
    let n = ns as usize;
    self.history[adr + 1] = (n - 1) as f64;
    let ts = model.timestep;
    for k in 0..n {
        self.history[adr + 2 + k] = -((n - k) as f64) * ts;
    }
}
```

**`impl Clone for Data` in `data.rs`** — add the new field (Data has a manual
Clone impl with ~60+ fields — forgetting this field is a silent bug):

```rust
history: self.history.clone(),
```

**`data_reset_field_inventory` test in `data.rs`** — update `EXPECTED_SIZE`
constant. Adding a `Vec<f64>` field changes `size_of::<Data>()` by 24 bytes
(Vec is 3 words: pointer + length + capacity). New value:
`EXPECTED_SIZE = 4104 + 24 = 4128`. (Verify empirically — compiler may add
padding.)

**Conformance equivalence statement:** After this spec lands, `Model` fields
for history attributes will produce **identical values** to MuJoCo's `mjModel`
for any MJCF model with actuator history attributes (`actuator_nsample`,
`actuator_interp`, `actuator_historyadr`, `actuator_delay`, and `nhistory`
will match MuJoCo's `actuator_history`, `actuator_historyadr`,
`actuator_delay`, and `nhistory` respectively), and `Data.history` will have
the same initial state as MuJoCo's `MjData`. A future runtime implementation
needs no model-building changes.

---

## Acceptance Criteria

### AC1: `nsample` parsing *(runtime test — MuJoCo-verified)*
**Given:** MJCF model with one `<motor>` actuator, `nsample="4"`
**After:** `load_model(&mjcf)`
**Assert:** `model.actuator_nsample[0]` = 4
**Field:** `Model.actuator_nsample`

### AC2: `interp` keyword parsing *(runtime test — MuJoCo-verified)*
**Given:** MJCF model with three actuators: `interp="zoh"`, `interp="linear"`, `interp="cubic"`
**After:** `load_model(&mjcf)`
**Assert:** `model.actuator_interp` = `[InterpolationType::Zoh, InterpolationType::Linear, InterpolationType::Cubic]`
**Field:** `Model.actuator_interp`

### AC3: `delay` parsing *(runtime test — MuJoCo-verified)*
**Given:** MJCF model with one actuator, `nsample="4"`, `delay="0.006"`
**After:** `load_model(&mjcf)`
**Assert:** `model.actuator_delay[0]` = 0.006 ± 1e-15
**Field:** `Model.actuator_delay`

### AC4: `historyadr` computation — multi-actuator *(runtime test — MuJoCo-verified)*
**Given:** MJCF model with 3 actuators: first with `nsample="3"`, second with no `nsample` attribute (defaults to 0), third with `nsample="2"`
**After:** `load_model(&mjcf)`
**Assert:** `model.actuator_historyadr` = `[0, -1, 8]`; `model.nhistory` = 14
**Field:** `Model.actuator_historyadr`, `Model.nhistory`
**MuJoCo verification:** MuJoCo 3.5.0 produces `historyadr=[0, -1, 8]`, `nhistory=14` for this configuration.

### AC5: `Data.history` allocation *(runtime test — MuJoCo-verified)*
**Given:** Same 3-actuator model as AC4
**After:** `model.make_data()`
**Assert:** `data.history.len()` = 14
**Field:** `Data.history`

### AC6: `Data.history` initial state *(runtime test — MuJoCo-verified)*
**Given:** MJCF model with 1 actuator, `nsample="4"`, `timestep="0.002"`
**After:** `model.make_data()`
**Assert:** `data.history` = `[0.0, 3.0, -0.008, -0.006, -0.004, -0.002, 0.0, 0.0, 0.0, 0.0]`
**Field:** `Data.history`
**MuJoCo verification:** MuJoCo 3.5.0, Python bindings. meta0=0.0, meta1=3.0, times=[-0.008, -0.006, -0.004, -0.002], values=[0,0,0,0].

### AC7: Default class inheritance *(runtime test — analytically derived)*
**Given:** MJCF:
```xml
<mujoco>
  <default>
    <default class="hist">
      <general nsample="8" interp="linear" delay="0.02"/>
    </default>
  </default>
  <worldbody>
    <body>
      <joint type="hinge"/>
      <joint type="hinge" name="j2"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <general joint="joint0" class="hist"/>
    <general joint="j2" class="hist" nsample="3"/>
  </actuator>
</mujoco>
```
**After:** `load_model(&mjcf)`
**Assert:** Actuator 0: `nsample=8`, `interp=Linear`, `delay=0.02`. Actuator 1: `nsample=3`, `interp=Linear`, `delay=0.02`.
**Field:** `Model.actuator_nsample`, `Model.actuator_interp`, `Model.actuator_delay`

### AC8: Compiler validation — delay without history *(runtime test — MuJoCo-verified)*
**Given:** MJCF model with one actuator, `delay="0.01"`, no `nsample` (default 0)
**After:** `load_model(&mjcf)`
**Assert:** Returns `Err` with message `"setting delay > 0 without a history buffer (nsample must be > 0)"` (MuJoCo equivalent: `"Error: setting delay > 0 without a history buffer"`)
**Field:** Error result

### AC9: Default values when attributes omitted *(runtime test — MuJoCo-verified)*
**Given:** MJCF model with one `<motor>` actuator, no `nsample`/`interp`/`delay`
**After:** `load_model(&mjcf)`
**Assert:** `actuator_nsample[0]` = 0, `actuator_interp[0]` = `Zoh`, `actuator_delay[0]` = 0.0, `actuator_historyadr[0]` = -1, `nhistory` = 0, `data.history.len()` = 0
**Field:** `Model.actuator_nsample`, `Model.actuator_interp`, `Model.actuator_delay`, `Model.actuator_historyadr`, `Model.nhistory`, `Data.history`

### AC10: `Data::reset()` restores initial state *(runtime test — MuJoCo-verified)*
**Given:** Model with 1 actuator, `nsample="4"`, `timestep="0.002"`. Create data, mutate `data.history` arbitrarily, then call `data.reset(&model)`.
**After:** `data.reset(&model)`
**Assert:** `data.history` = `[0.0, 3.0, -0.008, -0.006, -0.004, -0.002, 0.0, 0.0, 0.0, 0.0]` (same as fresh `make_data()`)
**Field:** `Data.history`

### AC11: `Data::reset_to_keyframe()` restores initial state *(runtime test — MuJoCo-verified)*
**Given:** Same model as AC10. Create data, mutate history, then call `data.reset_to_keyframe(model, 0)`.
**After:** `data.reset_to_keyframe(model, 0)`
**Assert:** `data.history` matches fresh `make_data()` output
**Field:** `Data.history`

### AC12: `interp` invalid keyword *(runtime test — MuJoCo-verified)*
**Given:** MJCF model with `interp="spline"` (invalid keyword)
**After:** `load_model(&mjcf)`
**Assert:** Returns error containing "interp"
**Field:** Error result

### AC13: Attributes on shortcut types *(runtime test — analytically derived)*
**Given:** MJCF with `<position>` actuator having `nsample="3"`, `interp="linear"`, `delay="0.005"`
**After:** `load_model(&mjcf)`
**Assert:** `actuator_nsample[0]` = 3, `actuator_interp[0]` = `Linear`, `actuator_delay[0]` = 0.005
**Field:** `Model.actuator_nsample`, `Model.actuator_interp`, `Model.actuator_delay`
**Rationale:** Verifies these attributes are parsed in the common section, not inside the `<general>`-only gate.

### AC14: Code structure *(code review — not a runtime test)*
- New fields present in `Model`, `model_init.rs` (empty init), `Data`, `ModelBuilder`, `builder/build.rs` (transfer + post-process)
- `InterpolationType` enum in `enums.rs` with `FromStr` and `From<i32>`
- `impl Clone for Data` includes `history` field
- `data_reset_field_inventory` test `EXPECTED_SIZE` updated
- Parser places attribute extraction in common section, not general-only gate
- Both `merge_actuator_defaults()` and `apply_to_actuator()` updated
- No `unsafe` blocks in new code

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (nsample parsing) | T1 | Direct |
| AC2 (interp keyword parsing) | T2 | Direct |
| AC3 (delay parsing) | T1 | Direct (combined with nsample) |
| AC4 (historyadr multi-actuator) | T3 | Direct |
| AC5 (history allocation) | T3 | Direct (combined with historyadr) |
| AC6 (initial history state) | T4 | Direct |
| AC7 (default class inheritance) | T5 | Direct |
| AC8 (delay without history error) | T6 | Direct |
| AC9 (default values) | T7 | Direct |
| AC10 (reset restores state) | T8 | Direct |
| AC11 (reset_to_keyframe restores state) | T9 | Direct |
| AC12 (invalid interp keyword) | T10 | Direct |
| AC13 (shortcut type parsing) | T11 | Direct |
| AC14 (code structure) | — | Code review (manual) |

---

## Test Plan

All tests in `sim/L0/tests/integration/actuator_phase5.rs` (extending the
existing Phase 5 test file). MuJoCo-verified expected values from MuJoCo 3.5.0
Python bindings.

### T1: Basic nsample/delay parsing → AC1, AC3
MJCF model: one hinge joint, one `<motor>` actuator with `nsample="4"`,
`delay="0.006"`.
Assert: `model.actuator_nsample[0]` = 4, `model.actuator_delay[0]` = 0.006 ± 1e-15.
MuJoCo-verified: `actuator_history[0,0]` = 4, `actuator_delay[0]` = 0.006.

### T2: All three interp keywords → AC2
MJCF model: three hinge joints, three `<general>` actuators with `nsample="2"` each, `interp="zoh"`, `interp="linear"`, `interp="cubic"`.
Assert: `model.actuator_interp` = `[Zoh, Linear, Cubic]`.
MuJoCo-verified: `actuator_history[:,1]` = `[0, 1, 2]`.

### T3: Multi-actuator historyadr and nhistory → AC4, AC5
MJCF model: three hinge joints, three actuators with `nsample="3"`, `nsample="0"` (default, no attribute), `nsample="2"`.
Assert: `model.actuator_historyadr` = `[0, -1, 8]`, `model.nhistory` = 14, `data.history.len()` = 14.
MuJoCo-verified: historyadr=`[0, -1, 8]`, nhistory=14.

### T4: Initial history buffer state — MuJoCo conformance → AC6
MJCF model: one actuator, `nsample="4"`, model timestep=0.002.
Assert: `data.history` = `[0.0, 3.0, -0.008, -0.006, -0.004, -0.002, 0.0, 0.0, 0.0, 0.0]`.
MuJoCo-verified: MuJoCo 3.5.0 Python bindings, fresh `MjData`.
Also test with timestep=0.01: `data.history` = `[0.0, 3.0, -0.04, -0.03, -0.02, -0.01, 0.0, 0.0, 0.0, 0.0]`.

### T5: Default class inheritance → AC7
MJCF with default class "hist" setting `nsample="8"`, `interp="linear"`, `delay="0.02"`. Two actuators: act0 inherits class "hist" fully, act1 inherits class "hist" but overrides `nsample="3"`.
Assert act0: `nsample=8`, `interp=Linear`, `delay=0.02`.
Assert act1: `nsample=3`, `interp=Linear`, `delay=0.02`.
Analytically derived: MuJoCo default class merge semantics — child overrides specific attributes, inherits the rest.

### T6: Compiler validation — delay without history → AC8
MJCF with one actuator, `delay="0.01"`, no `nsample`.
Assert: `load_model()` returns `Err` with message `"setting delay > 0 without a history buffer (nsample must be > 0)"`.
MuJoCo-verified: MuJoCo 3.5.0 produces compile error `"Error: setting delay > 0 without a history buffer"`.

### T7: Default values when attributes omitted → AC9
MJCF with one `<motor>` actuator, no history attributes.
Assert model: `nsample=0`, `interp=Zoh`, `delay=0.0`, `historyadr=-1`, `nhistory=0`.
Assert data: `model.make_data()` → `data.history.len()=0` (empty buffer).
MuJoCo-verified: all defaults confirmed.

### T8: Data::reset() restores pre-populated state → AC10
Create model with nsample=4, timestep=0.002. Create data. Overwrite `data.history` with `[99.0; nhistory]`. Call `data.reset(&model)`.
Assert: `data.history` matches fresh `make_data()` output exactly (element-by-element).
MuJoCo-verified: `mj_resetData` output matches fresh `MjData` via `np.allclose`.

### T9: Data::reset_to_keyframe() restores pre-populated state → AC11
Same setup as T8. After mutating history, call `data.reset_to_keyframe(model, 0)`.
Assert: `data.history` matches fresh `make_data()` output.
MuJoCo-verified: `mj_resetDataKeyframe` produces identical history to `mj_resetData`.

### T10: Invalid interp keyword → AC12
MJCF with `interp="spline"`.
Assert: `load_model()` returns `Err` containing "interp".
MuJoCo-verified: MuJoCo rejects invalid keywords as XML parse errors.

### T11: Attributes on shortcut types → AC13
MJCF with `<position>` actuator having `nsample="3"`, `interp="linear"`, `delay="0.005"`.
Assert: `nsample=3`, `interp=Linear`, `delay=0.005`.
Analytically derived: verifies parser common-section placement (not inside general-only gate).

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| `nsample=0` (no history) | Default — must produce `historyadr=-1`, `nhistory=0` | T7 | AC9 |
| `nsample=1` (minimum valid) | Smallest valid buffer: `nhistory=4` | T12 | — (supplementary) |
| `nsample` negative | MuJoCo silently accepts → `historyadr=-1` | T13 | — (supplementary) |
| `delay=0.0` with `nsample>0` | Valid — buffer allocated, no delay | T3 (nsample=3,2 with no delay attr → default 0.0) | AC4 |
| `delay>0` with `nsample=0` | Compile error | T6 | AC8 |
| `interp="cubic"` with `nsample<4` | MuJoCo silently accepts — no minimum nsample | T14 | — (supplementary) |
| Delay exceeding buffer capacity | MuJoCo silently accepts — no validation | T15 | — (supplementary) |
| Multiple actuators with mixed nsample | Tests cumulative historyadr | T3 | AC4 |
| Attributes on `<position>` (shortcut) | Tests common-section parser placement | T11 | AC13 |
| Default class with partial override | Tests inheritance pipeline | T5 | AC7 |
| `interp` set without `nsample` | MuJoCo accepts: stores `[0, interp_value]` | T16 | — (supplementary) |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T12: `nsample=1` minimum valid | Single-sample buffer: `nhistory=4`, `historyadr=0`, meta1=0.0 | Boundary case — minimum valid allocation |
| T13: `nsample=-1` negative | Negative nsample accepted: `historyadr=-1`, `nhistory=0` | MuJoCo conformance — we must not reject values MuJoCo accepts |
| T14: `interp="cubic"` with `nsample=2` | Cubic interp with insufficient samples for cubic | MuJoCo conformance — silently accepted, we must not validate |
| T15: delay exceeds buffer capacity | `delay=0.1`, `nsample=2`, `timestep=0.002` | MuJoCo conformance — silently accepted, we must not validate |
| T16: `interp="linear"` with `nsample=0` | `interp` set independently of `nsample` | MuJoCo conformance — stores `[0, 1]` in actuator_history equivalent |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| MJCF with `nsample`/`interp`/`delay` attributes | Silently ignored | Parsed and stored in Model | Toward MuJoCo | Models using history attributes | None — transparent improvement |
| `Model` struct has new fields | Fields absent | Fields present (empty vecs for no-history models) | Toward MuJoCo | Code constructing `Model::empty()` | None — `empty()` initializes defaults |
| `Data` struct has `history` field | Field absent | Field present (empty vec for no-history models) | Toward MuJoCo | Code constructing Data via `make_data()` | None — `make_data()` handles allocation |
| `delay > 0` without `nsample > 0` | Silently accepted (attributes ignored) | Compile error | Toward MuJoCo | Models with invalid delay/nsample | Fix MJCF to add `nsample` or remove `delay` |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/mjcf/src/types.rs` | 3 new fields on `MjcfActuator`, 3 on `MjcfActuatorDefaults`, Default impls updated | +12 |
| `sim/L0/mjcf/src/parser.rs` | 3 attribute extractions in `parse_actuator_attrs()` common section + 3 in `parse_actuator_defaults()` | +8 |
| `sim/L0/mjcf/src/defaults.rs` | 3 merge lines in `merge_actuator_defaults()` + 3 apply blocks in `apply_to_actuator()` | +12 |
| `sim/L0/core/src/types/model.rs` | 5 new fields on `Model` with doc comments | +20 |
| `sim/L0/core/src/types/model_init.rs` | 5 new init lines in `Model::empty()` + history pre-population in `make_data()` | +25 |
| `sim/L0/core/src/types/enums.rs` | `InterpolationType` enum with `FromStr` + `From<i32>` | +35 |
| `sim/L0/core/src/types/data.rs` | 1 new field on `Data`, reset logic in `reset()` + `reset_to_keyframe()`, Clone line, EXPECTED_SIZE update | +25 |
| `sim/L0/mjcf/src/builder/mod.rs` | 3 new fields on `ModelBuilder` | +4 |
| `sim/L0/mjcf/src/builder/actuator.rs` | Push calls for 3 fields + interp parsing + delay validation | +20 |
| `sim/L0/mjcf/src/builder/build.rs` | 3 transfer lines + historyadr/nhistory computation (~15 lines) | +20 |
| `sim/L0/tests/integration/actuator_phase5.rs` | New test module for Spec D (T1–T16) | +250 |
| **Total** | | **~+430** |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| All existing actuator tests | `actuator_phase5.rs` | Pass (unchanged) | New fields have defaults; no existing behavior changes |
| `data_reset_field_inventory` | `data.rs:1026` | **Fails until EXPECTED_SIZE updated** | New `history: Vec<f64>` field changes `size_of::<Data>()` |
| Phase 4 regression suite (39 tests) | various | Pass (unchanged) | Phase 5 does not modify Phase 4 code paths |
| Full sim domain baseline (2,148+ tests) | various | Pass (unchanged) | New fields are additive; empty vecs for models without history |

**Risk note:** `parser.rs` has a `<general>`-only attribute gate (lines
2064-2076). The new attributes MUST be parsed in the common section (lines
2013-2062), NOT inside this gate. Placing them in the wrong section would
cause `<position>`, `<motor>`, `<velocity>` actuators to silently ignore
these attributes — a subtle conformance bug caught only by T11.

**Risk note:** `Data` has a manual `impl Clone` (~60+ fields). Forgetting the
new `history` field is a silent bug. AC14 (code review) explicitly checks this.

---

## Execution Order

1. **S4 first** — `InterpolationType` enum in `enums.rs` + Model fields in
   `model.rs` + `Model::empty()` in `model_init.rs`. These are pure type
   additions with no behavioral change. → verify: `cargo build -p sim-core`

2. **S1 next** — MJCF type additions in `types.rs`. Pure struct additions.
   → verify: `cargo build -p sim-mjcf`

3. **S2 + S3** — Parser and defaults pipeline. These depend on S1's types.
   → verify: `cargo build -p sim-mjcf`

4. **S5** — Builder pipeline: ModelBuilder fields, `process_actuator()` pushes,
   `build()` transfer + historyadr computation + validation. Depends on S1-S4.
   → verify: `cargo test -p sim-mjcf` (parsing round-trip tests)

5. **S6 last** — Data integration: `Data.history` field, `make_data()`
   pre-population, `reset()` restoration, `reset_to_keyframe()` restoration,
   Clone impl, EXPECTED_SIZE update. Depends on S4 (Model fields) and S5
   (historyadr computed).
   → verify: `cargo test -p sim-core -p sim-mjcf` (full domain)

After each section lands, run `cargo test -p sim-core -p sim-mjcf` to verify
no regressions.

---

## Out of Scope

- **Runtime delay/interpolation** — `mj_forward` reading from history buffer
  to compute delayed ctrl, `mj_step` writing to history buffer as circular
  buffer. Tracked as future actuator runtime work. Conformance impact: models
  with `delay > 0` will have correct model/data structures but delayed ctrl
  will not be applied at runtime. Acceptable for v1.0 — runtime delay support
  is not required for model-building conformance.
- **Sensor history attributes** — `mjsSensor_` has identical `nsample`/`interp`/
  `delay` attributes (mjspec.h:731-733) and model arrays (mjmodel.h:1218-1220).
  Deferred to future sensor spec. Conformance impact: `nhistory` will be
  actuator-only (smaller than MuJoCo's for models with sensor history). Known
  gap, documented in Convention Notes.
- **`mj_readCtrl()` / `mj_initCtrlHistory()`** — runtime API functions that
  consume the history buffer. Deferred with runtime delay. No conformance
  impact for model-building.
- **`mj_copyData` history copying** — `mj_copyData` copies the history buffer.
  CortenForge's `Data` clone (via manual `impl Clone`) will handle this
  naturally since `Vec<f64>` clones correctly. No special implementation needed.
