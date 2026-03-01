# Spec D — Sensor History Attributes: Spec Quality Rubric

Grades the Spec D spec (DT-109: sensor history attributes `nsample`, `interp`,
`delay`, `interval`) on 10 criteria. Target: A+ on every criterion before
implementation begins. A+ means "an implementer could build this without asking
a single clarifying question — and the result would produce numerically
identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment from Umbrella

The umbrella spec (DT-109) listed 3 sensor history attributes: `nsample`,
`interp`, `delay`. Empirical verification against MuJoCo 3.5.0 revealed a
4th attribute:

| Umbrella claim | MuJoCo 3.5.0 reality | Action |
|----------------|----------------------|--------|
| `nsample`, `interp`, `delay` | All three exist as MJCF attributes on sensor elements. Stored in `sensor_history` (packed), `sensor_delay` model arrays. | **In scope.** |
| *(not listed)* `interval` | **Exists.** `mjsSensor_::interval[2]` in `mjspec.h:734`. MJCF attribute `interval` (single float = period). Model array `sensor_interval` (nsensor × 2) in `mjmodel.h:1221`. | **Add to scope.** Parsing and model storage only — runtime interpolation deferred per umbrella §Out of Scope. |

**Spec D final scope: 4 attributes**
1. `nsample` — number of samples in history buffer (int)
2. `interp` — interpolation order: zoh=0, linear=1, cubic=2 (int)
3. `delay` — delay time in seconds (float)
4. `interval` — sampling period in seconds (float; second element = phase, init 0.0)

Plus 2 derived model arrays:
5. `sensor_historyadr` — cumulative offset into history buffer (-1 = no history)
6. `sensor_interval` — `[period, phase]` pair per sensor

---

## Empirical Ground Truth

All values verified against MuJoCo 3.5.0 via `uv run python3` against vendored
MuJoCo (`mujoco` package). The MuJoCo C source is the single source of truth.

### EGT-1: sensor_history packing

MuJoCo packs `nsample` and `interp` into a single `int*` array with stride 2:

```python
# nsample=5, interp=linear → sensor_history = [[5, 1]]
m.sensor_history  # [[5 1]]  — packed [nsample, interp] per sensor
```

Interp mapping: `zoh=0`, `linear=1`, `cubic=2`.

### EGT-2: Default values

```python
# No history attrs specified on sensor element
m.sensor_history     # [[0 0]]     — nsample=0, interp=0 (ZOH)
m.sensor_historyadr  # [-1]        — no history buffer
m.sensor_delay       # [0.]        — no delay
m.sensor_interval    # [[0. 0.]]   — no interval
m.nhistory           # 0           — no history buffer allocated
```

### EGT-3: nhistory formula

Per sensor with history (`nsample > 0`): **`nsample * (dim + 1) + 2`**

| nsample | dim | nhistory | Formula |
|---------|-----|----------|---------|
| 5 | 1 (jointpos) | 12 | 5 × (1+1) + 2 = 12 |
| 3 | 3 (framepos) | 14 | 3 × (3+1) + 2 = 14 |
| 1 | 4 (framequat) | 7 | 1 × (4+1) + 2 = 7 |
| 2 | 1 (jointpos) | 6 | 2 × (1+1) + 2 = 6 |
| 10 | 1 (jointpos) | 22 | 10 × (1+1) + 2 = 22 |
| 1 | 1 (jointpos) | 4 | 1 × (1+1) + 2 = 4 |
| 2 | 3 (framepos) | 10 | 2 × (3+1) + 2 = 10 |
| 5 | 3 (framepos) | 22 | 5 × (3+1) + 2 = 22 |
| 3 | 4 (framequat) | 17 | 3 × (4+1) + 2 = 17 |

All 9 cases verified empirically against MuJoCo 3.5.0 output.

For actuators (`dim=1` always): `nsample * (1+1) + 2 = 2 * nsample + 2` —
same formula, simplified. The existing actuator code at `build.rs:397`
uses `2 * ns + 2`; the sensor generalization is `nsample * (dim + 1) + 2`.

### EGT-4: Validation — delay without history buffer

```python
# delay > 0, nsample = 0 → REJECTED
# ValueError: Error: setting delay > 0 without a history buffer
# Element name '', id 0, line 6
```

This matches the actuator validation at `builder/actuator.rs:275–278`.

### EGT-5: Negative nsample accepted

```python
# nsample=-1 → history=[[-1, 0]], historyadr=[-1], nhistory=0
```

Negative values are accepted by MuJoCo, stored as-is, and treated as
"no history" (`historyadr = -1`, no nhistory contribution). Same behavior
as `nsample=0`.

### EGT-6: Mixed actuator + sensor history buffer layout

```python
# actuator: motor joint="j1" nsample=4 delay=0.01  (dim=1)
#   → actuator_history=[[4,0]], actuator_historyadr=[0]
#   → actuator contribution: 2*4+2 = 10
# sensor: jointpos joint="j1" nsample=3 delay=0.01  (dim=1)
#   → sensor_history=[[3,0]], sensor_historyadr=[10]  ← starts AFTER actuators
#   → sensor contribution: 3*(1+1)+2 = 8
# nhistory = 10 + 8 = 18
m.nhistory              # 18
m.actuator_historyadr   # [0]
m.sensor_historyadr     # [10]  ← offset = actuator total
```

**Key finding:** Sensors are allocated AFTER actuators in the combined history
buffer. `sensor_historyadr` values start at the total actuator history size,
not at 0. This is the MuJoCo layout that CortenForge must match.

### EGT-7: interval attribute

```python
# interval=0.5 → sensor_interval = [[0.5, 0.0]]
m.sensor_interval   # [[0.5 0. ]]  — period=0.5, phase=0.0

# interval="0.5 0.1" (attempt to set phase) → REJECTED
# ValueError: Error: positive phase in sensor
```

MJCF `interval` is a single float (the sampling period). The second element
(phase / `time_prev`) is initialized to `0.0` by the compiler. MuJoCo rejects
positive phase values in the MJCF attribute.

### EGT-8: sensor_intprm is NOT sensor_history

`mjmodel.h:1213`: `sensor_intprm` (nsensor × mjNSENS=3) is a **separate**
array from `sensor_history`. `intprm` stores user-defined integer parameters
(for plugin/user sensors). The spec must NOT conflate `sensor_intprm` with
`sensor_history` — they are distinct arrays with different purposes.

```c
int*  sensor_intprm;   // sensor parameters        (nsensor x mjNSENS)  — line 1213
int*  sensor_history;  // [nsample, interp]         (nsensor x 2)       — line 1218
```

### EGT-9: CortenForge field-name mapping

Exact CortenForge field names and locations for all fields referenced in
the MuJoCo C source. **Critical:** CortenForge uses snake_case with underscores
between words; MuJoCo concatenates. The spec MUST use CortenForge names in all
Rust pseudocode.

**Actuator history fields (already implemented — the pattern to mirror):**

| MuJoCo C field | CortenForge field | File | Line | Type |
|---------------|-------------------|------|------|------|
| `actuator_history[2*i+0]` | `model.actuator_nsample[i]` | `core/src/types/model.rs` | 592 | `Vec<i32>` |
| `actuator_history[2*i+1]` | `model.actuator_interp[i]` | `core/src/types/model.rs` | 596 | `Vec<InterpolationType>` |
| `actuator_historyadr[i]` | `model.actuator_historyadr[i]` | `core/src/types/model.rs` | 600 | `Vec<i32>` |
| `actuator_delay[i]` | `model.actuator_delay[i]` | `core/src/types/model.rs` | 604 | `Vec<f64>` |
| `nhistory` | `model.nhistory` | `core/src/types/model.rs` | 609 | `usize` |

**Sensor fields (existing — no history yet):**

| MuJoCo C field | CortenForge field | File | Line | Type |
|---------------|-------------------|------|------|------|
| `nsensor` | `model.nsensor` | `core/src/types/model.rs` | 492 | `usize` |
| `nsensordata` | `model.nsensordata` | `core/src/types/model.rs` | 494 | `usize` |
| `sensor_type[i]` | `model.sensor_type[i]` | `core/src/types/model.rs` | 496 | `Vec<MjSensorType>` |
| `sensor_dim[i]` | `model.sensor_dim[i]` | `core/src/types/model.rs` | 510 | `Vec<usize>` |
| `sensor_noise[i]` | `model.sensor_noise[i]` | `core/src/types/model.rs` | 512 | `Vec<f64>` |
| `sensor_cutoff[i]` | `model.sensor_cutoff[i]` | `core/src/types/model.rs` | 514 | `Vec<f64>` |

**Sensor history fields (TO BE ADDED by Spec D):**

| MuJoCo C field | Proposed CortenForge field | Type | Default |
|---------------|---------------------------|------|---------|
| `sensor_history[2*i+0]` | `model.sensor_nsample[i]` | `Vec<i32>` | `0` |
| `sensor_history[2*i+1]` | `model.sensor_interp[i]` | `Vec<InterpolationType>` | `InterpolationType::Zoh` |
| `sensor_historyadr[i]` | `model.sensor_historyadr[i]` | `Vec<i32>` | `-1` |
| `sensor_delay[i]` | `model.sensor_delay[i]` | `Vec<f64>` | `0.0` |
| `sensor_interval[2*i+0]` | *(see AD-1)* | *(see AD-1)* | `0.0` |
| `sensor_interval[2*i+1]` | *(see AD-1)* | *(see AD-1)* | `0.0` |

**MJCF parse-side fields:**

| MuJoCo spec field | CortenForge field | File | Line | Type |
|-------------------|-------------------|------|------|------|
| `mjsSensor_::nsample` | `MjcfSensor.nsample` *(to add)* | `mjcf/src/types.rs` | 3097–3127 | `Option<i32>` |
| `mjsSensor_::interp` | `MjcfSensor.interp` *(to add)* | `mjcf/src/types.rs` | 3097–3127 | `Option<String>` |
| `mjsSensor_::delay` | `MjcfSensor.delay` *(to add)* | `mjcf/src/types.rs` | 3097–3127 | `Option<f64>` |
| `mjsSensor_::interval` | `MjcfSensor.interval` *(to add)* | `mjcf/src/types.rs` | 3097–3127 | `Option<f64>` |
| `InterpolationType` enum | `InterpolationType` | `core/src/types/enums.rs` | 256–279 | enum (Zoh=0, Linear=1, Cubic=2) |

### EGT-10: Exhaustive match-site inventory

Every code location that must be modified when adding sensor history fields.
Organized by crate and exhaustiveness risk.

**Table 1: sim-mjcf types (struct definitions — forgetting a field is a silent bug)**

| File | Line range | Target | Exhaustive? | Required change |
|------|-----------|--------|-------------|-----------------|
| `mjcf/src/types.rs` | 3097–3127 | `MjcfSensor` struct | N/A (struct) | Add 4 new `Option` fields: `nsample`, `interp`, `delay`, `interval` |
| `mjcf/src/types.rs` | 3129–3147 | `MjcfSensor::default()` impl | **YES** (struct literal) | Add defaults for 4 new fields (`None` for all). **Compiler-enforced: manual `impl Default` uses `Self { ... }` struct literal — forgetting a field won't compile.** |
| `mjcf/src/types.rs` | 3195–3214 | `MjcfSensor` fluent API | N/A (methods) | Add 4 new fluent builder methods: `with_nsample(i32)`, `with_interp(&str)`, `with_delay(f64)`, `with_interval(f64)`. Pattern follows `with_noise`/`with_cutoff`/`with_user`. **Not compiler-enforced — omitting methods doesn't break compilation, just makes the API incomplete.** |
| `mjcf/src/types.rs` | 775–782 | `MjcfSensorDefaults` struct | N/A (struct) | **NOT MODIFIED** per AD-3(b). MuJoCo has no sensor defaults — history attrs parsed from elements only. Existing `noise`/`cutoff`/`user` fields are a pre-existing CortenForge extension, left as-is. |

**Table 2: sim-mjcf parser (no match — manual audit required)**

| File | Line range | Function | Exhaustive? | Required change |
|------|-----------|----------|-------------|-----------------|
| `mjcf/src/parser.rs` | 3453–3523 | `parse_sensor_attrs()` | N/A (no match) | Add 4 attribute reads: `parse_int_attr(e, "nsample")`, `get_attribute_opt(e, "interp")`, `parse_float_attr(e, "delay")`, `parse_float_attr(e, "interval")`. **Compiler will NOT catch omissions — attrs silently default to None.** |
| `mjcf/src/parser.rs` | 909–913 | `parse_sensor_defaults()` | N/A (no match) | **NOT MODIFIED** per AD-3(b). Parses `noise`/`cutoff`/`user` only — history attrs have no defaults in MuJoCo. |

**Table 3: sim-mjcf defaults (merge + apply — NOT modified for history attrs)**

| File | Line range | Function | Exhaustive? | Required change |
|------|-----------|----------|-------------|-----------------|
| `mjcf/src/defaults.rs` | 815–827 | `merge_sensor_defaults()` | N/A (field-by-field) | **NOT MODIFIED** per AD-3(b). Merges `noise`/`cutoff`/`user` only. History attrs have no default class inheritance in MuJoCo. |
| `mjcf/src/defaults.rs` | 533–557 | `apply_to_sensor()` | N/A (field-by-field) | **NOT MODIFIED** per AD-3(b). Applies `noise`/`cutoff`/`user` defaults only. History attrs are NOT applied from defaults — they are read directly from sensor elements. |

**Table 4: sim-mjcf builder (push to model arrays)**

| File | Line range | Function | Exhaustive? | Required change |
|------|-----------|----------|-------------|-----------------|
| `mjcf/src/builder/mod.rs` | 635–645 | `ModelBuilder` struct | N/A (struct) | Add 4 new `Vec` fields: `sensor_nsample`, `sensor_interp`, `sensor_delay`, `sensor_interval`. |
| `mjcf/src/builder/init.rs` | 219–229 | `ModelBuilder::new()` | **YES** (struct literal) | Add `vec![]` for 4 new fields. **Compiler-enforced: struct literal is incomplete without new fields.** |
| `mjcf/src/builder/sensor.rs` | 19–108 | `process_sensors()` | N/A (push loop) | Add validation (delay/nsample), parse interp string → `InterpolationType`, push to 4 new `Vec` fields. Mirror `builder/actuator.rs:264–283`. |
| `mjcf/src/builder/build.rs` | 230–242 | Model construction (sensor transfer) | N/A (field transfer) | Transfer 4 new sensor `Vec` fields from builder to model. **DANGER: forgetting a field means model has empty vec while builder had data.** |
| `mjcf/src/builder/build.rs` | 387–405 | Post-hoc historyadr + nhistory | N/A (computation) | Extend actuator-only historyadr loop to include sensors. Sensor addresses start after actuator total. Update `nhistory` to include sensor contributions. |

**Table 5: sim-core model (struct + init)**

| File | Line range | Target | Exhaustive? | Required change |
|------|-----------|--------|-------------|-----------------|
| `core/src/types/model.rs` | 490–516 | `Model` struct (sensor section) | N/A (struct) | Add 5 new fields: `sensor_nsample`, `sensor_interp`, `sensor_historyadr`, `sensor_delay`, `sensor_interval` |
| `core/src/types/model_init.rs` | 219–232 | `Model::empty()` (sensor section) | N/A (init) | Add `vec![]` for 5 new fields. **DANGER: forgetting a field causes compile error (struct literal incomplete) — this one IS compiler-enforced.** |

**Summary:** 12 modification sites across 7 files + 4 non-modification sites
(explicitly NOT touched per AD-3(b)). Of the 12 modification sites, 3 are
compiler-enforced (`Model::empty()` struct literal at `model_init.rs:219`,
`ModelBuilder::new()` struct literal at `init.rs:219`, `MjcfSensor::default()`
struct literal at `types.rs:3129`). 9 require manual audit — struct field
additions, parser attribute reads, fluent API methods, and builder
push/transfer are non-exhaustive patterns where the compiler will NOT catch
omissions. The 4 non-modification sites (`MjcfSensorDefaults`,
`parse_sensor_defaults()`, `merge_sensor_defaults()`, `apply_to_sensor()`)
handle only `noise`/`cutoff`/`user` — history attrs are parsed from elements
only, matching MuJoCo's behavior (EGT-11h, AD-3(b)).

### EGT-11: Edge-case interactions (stress-test empirical)

Interactions between attributes not covered by single-attribute EGTs.
Verified against MuJoCo 3.5.0.

**EGT-11a: `interval` without `nsample` — ACCEPTED**

```python
# interval=0.5, nsample omitted → sensor_interval stored, no history buffer
m.sensor_interval    # [[0.5 0. ]]
m.sensor_history     # [[0 0]]
m.sensor_historyadr  # [-1]
m.nhistory           # 0
```

`interval` is independent of `nsample`. A sensor can have a sampling period
without a history buffer. The spec must NOT validate `interval > 0 → nsample > 0`.

**EGT-11b: Negative `interval` — REJECTED**

```python
# interval=-0.5 → REJECTED
# ValueError: Error: negative interval in sensor
# Element name '', id 0, line 6
```

MuJoCo rejects negative interval. `interval=0` is accepted (no sampling period).
The spec must add validation: `interval < 0 → error`.

**EGT-11c: `interp` keyword is case-sensitive, rejects integers**

```python
# interp="1"       → REJECTED: "invalid keyword: '1'"
# interp="Linear"  → REJECTED: "invalid keyword: 'Linear'"
# interp="ZOH"     → REJECTED: "invalid keyword: 'ZOH'"
# interp="linear"  → ACCEPTED
```

Only lowercase keywords accepted: `"zoh"`, `"linear"`, `"cubic"`. CortenForge's
`InterpolationType::from_str()` (`enums.rs:270–273`) already matches this
behavior (lowercase only). But the spec must explicitly state this — an
implementer might add case-insensitive parsing "for convenience."

**EGT-11d: `interp` stored even without `nsample`**

```python
# interp="linear", nsample omitted → history=[[0, 1]], historyadr=[-1]
```

`interp` is stored in `sensor_history` even when `nsample=0`. The spec must
NOT skip storing interp when nsample is absent.

**EGT-11e: Multiple sensors with history — offset accumulation**

```python
# sensor 0: jointpos j1 nsample=3 (dim=1) → historyadr=0, contrib=3*2+2=8
# sensor 1: jointpos j2 nsample=5 (dim=1) → historyadr=8, contrib=5*2+2=12
# sensor 2: framepos s1 nsample=2 (dim=3) → historyadr=20, contrib=2*4+2=10
# nhistory = 8 + 12 + 10 = 30
m.sensor_historyadr  # [ 0  8 20]
m.nhistory           # 30
```

**EGT-11f: `nsample=0` sensor in the middle doesn't break chain**

```python
# sensor 0: nsample=3 → historyadr=0, contrib=8
# sensor 1: nsample=0 → historyadr=-1 (skipped)
# sensor 2: nsample=5 → historyadr=8, contrib=12
# nhistory = 8 + 12 = 20
m.sensor_historyadr  # [ 0 -1  8]
m.nhistory           # 20
```

**EGT-11g: Actuator + multiple sensors — full layout**

```python
# actuator 0: nsample=4 (dim=1) → act_historyadr=0, contrib=10
# actuator 1: nsample=2 (dim=1) → act_historyadr=10, contrib=6
# act_total = 16
# sensor 0: nsample=3 (dim=1) → sens_historyadr=16, contrib=8
# sensor 1: nsample=5 (dim=1) → sens_historyadr=24, contrib=12
# nhistory = 16 + 8 + 12 = 36
m.actuator_historyadr  # [ 0 10]
m.sensor_historyadr    # [16 24]
m.nhistory             # 36
```

**EGT-11h: MuJoCo does NOT support sensor defaults**

```python
# <default><sensor nsample="5"/></default>        → "unrecognized element"
# <default><jointpos nsample="5"/></default>       → "unrecognized element"
# <sensor><jointpos class="foo" .../></sensor>     → "unrecognized attribute: 'class'"
# <default><sensor noise="0.1"/></default>         → "unrecognized element"
```

MuJoCo 3.5.0 has **no** sensor default class mechanism. `<sensor>` is not a
valid element inside `<default>`. Per-type sensor elements (`<jointpos>`,
`<framepos>`, etc.) are also rejected inside `<default>`. Even the `class`
attribute on sensor elements is rejected. This means `noise`, `cutoff`, and
`user` also have no default mechanism in MuJoCo — they must be set per element.

Actuators DO support per-element defaults: `<motor nsample="5"/>` and
`<general nsample="5"/>` both work inside `<default>`.

CortenForge's `MjcfSensorDefaults` (`types.rs:775`), `parse_sensor_defaults()`
(`parser.rs:909`), `apply_to_sensor()` (`defaults.rs:533`), and
`merge_sensor_defaults()` (`defaults.rs:815`) are **pre-existing CortenForge
extensions** that accept MJCF not accepted by MuJoCo. The spec must document
this clearly.

**EGT-11i: Full combination — all 4 attributes on one sensor**

```python
# framepos site nsample=5 interp="cubic" delay=0.02 interval=0.1 (dim=3)
m.sensor_history     # [[5 2]]       — nsample=5, interp=cubic(2)
m.sensor_delay       # [0.02]
m.sensor_interval    # [[0.1 0. ]]   — period=0.1, phase=0.0
m.sensor_historyadr  # [0]
m.nhistory           # 22            — 5*(3+1)+2 = 22
```

All 4 attributes work together on a single sensor. The spec should include
a "full combination" AC verifying this end-to-end.

**EGT-11j: Negative delay — ACCEPTED**

```python
# delay=-0.01, nsample=5 → ACCEPTED
m.sensor_delay       # [-0.01]
m.sensor_historyadr  # [0]
m.nhistory           # 12     — 5*(1+1)+2 = 12 (history buffer allocated)

# delay=-0.01, nsample omitted → ACCEPTED (no error!)
m.sensor_delay       # [-0.01]
m.sensor_historyadr  # [-1]   — no history buffer
```

Negative delay is accepted and stored by MuJoCo. The validation `delay > 0 && nsample <= 0`
only fires for POSITIVE delay without nsample. Negative delay bypasses the check entirely.
Same behavior as negative nsample (EGT-5): accepted, stored, treated as "unusual but valid."
The spec must NOT add validation for negative delay — MuJoCo doesn't reject it.

**EGT-11k: Sensor-only model (no actuators)**

```python
# No actuators, sensor with nsample=3 (dim=1)
m.nu                    # 0
m.sensor_historyadr     # [0]    — starts at offset 0 (no actuator contribution)
m.nhistory              # 8     — 3*(1+1)+2 = 8
m.actuator_historyadr   # shape=(0,) — empty, no actuators
```

Sensors start at offset 0 when there are no actuators. The historyadr computation
must handle the empty-actuator case correctly (actuator loop produces offset=0,
sensor loop continues from 0).

### EGT-12: Analytical verification (V-derivations)

Physics/math derivations for all empirical values.

**V1: nhistory formula derivation**

The history buffer for each sensor/actuator stores a circular buffer of past
values plus metadata. Per MuJoCo's `engine_io.c`, the buffer layout per entity is:

```
[sample_0_dim_0, ..., sample_0_dim_D,   // sample 0: D+1 values (D data + 1 time)
 sample_1_dim_0, ..., sample_1_dim_D,   // sample 1: D+1 values
 ...
 sample_N-1_dim_0, ..., sample_N-1_dim_D,  // sample N-1: D+1 values
 write_index,                            // 1 value: current write position
 count]                                  // 1 value: number of samples written
```

Total: `nsample * (dim + 1) + 2`

- `nsample` samples, each storing `dim` data values + 1 timestamp = `dim + 1` per sample
- 2 metadata slots (write index, count)

For actuators (`dim=1`): `nsample * 2 + 2` — matches `build.rs:397` (`2 * ns + 2`).

**V2: Mixed buffer offset derivation**

Actuator with `nsample=4`, `dim=1`: contribution = `4 * 2 + 2 = 10`.
Sensor with `nsample=3`, `dim=1`: contribution = `3 * 2 + 2 = 8`.
Actuator is processed first → `actuator_historyadr[0] = 0`.
Sensor starts after actuator block → `sensor_historyadr[0] = 10`.
Total: `nhistory = 10 + 8 = 18`. ✓ (EGT-6 confirmed)

**V3: Actuator formula equivalence**

For `dim=1`: `nsample * (dim + 1) + 2 = nsample * (1 + 1) + 2 = 2 * nsample + 2`.
This is exactly the formula in `build.rs:397` (`offset += 2 * ns + 2`).
The sensor generalization preserves actuator behavior when `dim=1`. ✓

### EGT-13: Architectural decisions

Decisions the spec must make explicitly. Each lists options with trade-offs.

**AD-1: `sensor_interval` representation in CortenForge**

MuJoCo stores `sensor_interval` as `mjtNum*` with stride 2: `[period, phase]`
per sensor.

| Option | Description | Trade-off |
|--------|-------------|-----------|
| **(a) `Vec<(f64, f64)>`** | Tuple of `(period, phase)`. | Matches actuator-like pattern. Readable. But actuator history doesn't have an `interval` equivalent, so no direct precedent. |
| **(b) `Vec<[f64; 2]>`** | Fixed-size array `[period, phase]`. | Mirrors MuJoCo's stride-2 layout more directly. Less semantic than named fields. |
| **(c) Two separate `Vec<f64>`** | `sensor_interval_period: Vec<f64>` + `sensor_interval_phase: Vec<f64>`. | Maximum clarity. But MuJoCo stores them packed, and this creates 2 fields for 1 concept. |

**Recommended:** (a) — `Vec<(f64, f64)>` is idiomatic Rust for paired values and
matches the `actuator_actrange: Vec<(f64, f64)>` and `tendon_range: Vec<(f64, f64)>`
precedent in the Model struct.

**AD-2: `interval` attribute scope**

The umbrella spec (DT-109) lists only `nsample`/`interp`/`delay`. MuJoCo 3.5.0
also has `interval` on sensors (EGT-7 verified).

| Option | Description | Trade-off |
|--------|-------------|-----------|
| **(a) Include `interval` in Spec D** | Parse and store `interval` alongside the other 3 attrs. | Complete MuJoCo parity. 4th attribute adds minimal scope. |
| **(b) Defer `interval`** | Only implement the 3 umbrella-listed attrs. | Leaves a known conformance gap. Would need a follow-up DT-ID. |

**Recommended:** (a) — `interval` is trivially parseable (single float),
requires one new model field, and its omission would be a conformance gap.
The umbrella likely omitted it because it wasn't discovered during initial
scoping. Including it is minimal incremental effort.

**AD-3: Default class inheritance for sensor history attrs**

**CRITICAL DISCOVERY (R25):** MuJoCo 3.5.0 has **no** sensor default class
mechanism whatsoever (EGT-11h). ALL of these are rejected:
- `<default><sensor nsample="5"/></default>` → "unrecognized element"
- `<default><jointpos nsample="5"/></default>` → "unrecognized element"
- `<sensor><jointpos class="foo" .../></sensor>` → "unrecognized attribute: 'class'"
- `<default><sensor noise="0.1"/></default>` → "unrecognized element"

CortenForge's `MjcfSensorDefaults` (`types.rs:775`), `parse_sensor_defaults()`
(`parser.rs:909`), `apply_to_sensor()` (`defaults.rs:533`), and
`merge_sensor_defaults()` (`defaults.rs:815`) are **pre-existing CortenForge
extensions** — they accept MJCF that MuJoCo rejects. This predates Spec D.

| Option | Description | Trade-off |
|--------|-------------|-----------|
| **(a) Add to `MjcfSensorDefaults` anyway** | Extend the existing CortenForge-only mechanism. History attrs inherit through the CortenForge defaults system (even though MuJoCo has no such system). | Internally consistent with the `noise`/`cutoff`/`user` CortenForge extension. Users who rely on CortenForge's sensor defaults get history attrs "for free." But extending a non-MuJoCo mechanism adds surface area. |
| **(b) Skip defaults for history attrs** | Only parse history attrs from sensor elements, not from `<default>`. | History attrs follow MuJoCo's behavior exactly: no defaults. But creates an inconsistency: `noise`/`cutoff`/`user` have CortenForge defaults while `nsample`/`interp`/`delay`/`interval` don't. |
| **(c) Document and defer** | Spec D states the MuJoCo ground truth (no sensor defaults) and defers the decision to a follow-up. History attrs are parsed only from sensor elements. The existing CortenForge sensor defaults extension is documented as a known divergence for future reconciliation. | Safest: doesn't extend a non-MuJoCo mechanism. But leaves the overall sensor defaults question unresolved. |

**Recommended:** (b) — don't extend a non-MuJoCo mechanism further. Parse
history attrs from sensor elements only. Spec must clearly document that
MuJoCo has no sensor defaults, that CortenForge's existing
`noise`/`cutoff`/`user` defaults are a CortenForge extension, and that Spec D
intentionally does NOT add history attrs to this extension. This changes the
P2 pipeline: steps (0), (1.5), and (2) relating to `parse_sensor_defaults()`
and `merge_sensor_defaults()` / `apply_to_sensor()` should NOT include history
attrs — only the existing `noise`/`cutoff`/`user`. P5 testing: a "sensor with
class" test that expects nsample inheritance would be WRONG per MuJoCo behavior.

**Parity note for P9:** Actuators DO support per-element defaults (`<motor
nsample="5"/>` works). Sensor→actuator parity breaks at the defaults layer
because MuJoCo treats them differently.

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. MuJoCo conformance is the entire reason CortenForge exists.
> A spec can be A+ on every other criterion and still be worthless if P1 is
> wrong — because an incorrect MuJoCo reference means every algorithm, every
> AC, and every test is verifying the wrong behavior. **Grade P1 first and
> grade it hardest.** If P1 is not A+, do not proceed to grading other
> criteria until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving. This is the
> single most important criterion: everything else in the spec is downstream
> of getting the MuJoCo reference right. An error here propagates to every
> algorithm, every AC, and every test.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo field cited with source file and line range: `mjsSensor_::nsample/interp/delay/interval` in `mjspec.h:731–734`, `sensor_history/sensor_historyadr/sensor_delay/sensor_interval` in `mjmodel.h:1218–1221`, `mjxmacro.h:695–698` wiring. Default values stated (`nsample=0`, `interp=0` ZOH, `delay=0.0`, `interval=[0.0, 0.0]`). `sensor_intprm` (`mjmodel.h:1213`) explicitly distinguished from `sensor_history` — they are separate arrays. Edge cases addressed with verified MuJoCo behavior: (1) `nsample ≤ 0` → `historyadr = -1`, no nhistory contribution; (2) negative `nsample` accepted and stored (EGT-5); (3) `delay > 0` without `nsample > 0` → compiler error "setting delay > 0 without a history buffer" (EGT-4); (4) multi-dim sensor nhistory formula `nsample * (dim + 1) + 2` verified across dim=1, dim=3, dim=4 (EGT-3); (5) mixed actuator + sensor history buffer layout — sensors after actuators, `sensor_historyadr` offset by actuator total (EGT-6); (6) `interval` is single float in MJCF (period), second element (phase) initialized to `0.0`, positive phase rejected (EGT-7); (7) `sensor_history` packs `[nsample, interp]` per sensor — not to be confused with `sensor_intprm` (EGT-8); (8) `interval` without `nsample` is valid — interval is independent of history buffer (EGT-11a); (9) negative `interval` rejected with "negative interval in sensor" (EGT-11b); (10) `interp` keyword is case-sensitive, lowercase only — "Linear", "ZOH", integer "1" all rejected (EGT-11c); (11) `interp` stored even when `nsample=0` (EGT-11d); (12) negative `delay` accepted and stored — validation is `delay > 0.0` (strictly positive), not `delay != 0.0` (EGT-11j). Numerical expectations from MuJoCo 3.5.0 output included. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps — e.g., `interval` attribute omitted or nhistory formula not verified empirically. |
| **B** | Correct at high level, but missing specifics (e.g., "MuJoCo stores sensor history" without field names, array layout, or formula). |
| **C** | Partially correct. Some MuJoCo behavior assumed from docs rather than C source. `sensor_intprm` conflated with `sensor_history`. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Rust code is line-for-line implementable. The algorithm must
> reproduce MuJoCo's behavior exactly.

| Grade | Bar |
|-------|-----|
| **A+** | Spec D is parse-and-store only (no runtime algorithm), so "algorithm" = the builder pipeline: (1) parse 4 MJCF attributes in `parse_sensor_attrs()` (`parser.rs:3453`) → `MjcfSensor` fields (NO defaults step for history attrs — MuJoCo has no sensor defaults per EGT-11h; see AD-3), (2) existing `apply_to_sensor()` (`defaults.rs:533`, called at `sensor.rs:26`) applies `noise`/`cutoff`/`user` defaults only — spec must explicitly state that `apply_to_sensor()` does NOT touch history fields, to prevent an implementer from adding them there, (3) parse interp string → `InterpolationType` enum (shared with actuators, `enums.rs:267`) — **must come BEFORE validation** (matching actuator order at `actuator.rs:265`; MuJoCo reports interp errors before delay/nsample errors). Parse pattern: `sensor.interp.as_deref().map(|s| s.parse::<InterpolationType>()).transpose().map_err(|e| ModelConversionError { message: e })?.unwrap_or(InterpolationType::Zoh)`. **Invalid interp → `Err(ModelConversionError)`, NOT silent default to Zoh.** Empty string `interp=""` is also rejected. (4) validate in `process_sensors()` (`builder/sensor.rs:19`): (a) `delay > 0.0 && nsample <= 0` → error (EGT-4), (b) `interval < 0.0` → error "negative interval in sensor" (EGT-11b), (5) push history attrs in the **same push block** as existing 11 fields at `sensor.rs:83–100` — NOT in a separate loop (the `continue` at `sensor.rs:33` for unsupported types skips all pushes, maintaining Vec alignment; a separate loop would break this). **Unwrap defaults** for each field: `sensor.nsample.unwrap_or(0)` → `i32`, `sensor.interp.as_deref().map(|s| s.parse::<InterpolationType>()).transpose()?.unwrap_or(InterpolationType::Zoh)` → `InterpolationType`, `sensor.delay.unwrap_or(0.0)` → `f64`, `(sensor.interval.unwrap_or(0.0), 0.0)` → `(f64, f64)` per AD-1(a), (6) **transfer 4 fields** to `Model` in `build.rs:230` (`sensor_nsample`, `sensor_interp`, `sensor_delay`, `sensor_interval` — direct `self.field` transfer), **initialize 1 field** to `vec![]` (`sensor_historyadr` — computed post-hoc, same pattern as `actuator_historyadr: vec![]` at `build.rs:267`), (7) compute `sensor_historyadr` and extend `nhistory` in post-hoc block (`build.rs:387+`). **Non-modification sites:** (a) `make_data()` (`model_init.rs:388`) allocates `vec![0.0; self.nhistory]` — automatically grows with increased `nhistory`, no code change needed; (b) `reset_data()` (`data.rs:874`) pre-populates actuator history only — sensor history pre-population is runtime scope (deferred per umbrella §Out of Scope), no code change needed. **Defaults scope clarification:** `parse_sensor_defaults()` (`parser.rs:909`) is a separate code path that parses `noise`/`cutoff`/`user` — it must NOT be extended with history attrs, because MuJoCo doesn't support sensor defaults at all (EGT-11h). Each step written as implementable Rust with exact field names, types, and default values. The nhistory formula `nsample * (dim + 1) + 2` stated explicitly with V1 derivation (EGT-12) showing buffer layout and equivalence to actuator formula for dim=1 (V3). For every model/data field referenced, spec verifies it exists in CortenForge (citing file:line) or specifies its addition. No "TBD" or "see MuJoCo" gaps. |
| **A** | Pipeline complete. One or two minor details left implicit (e.g., exact error message text). |
| **B** | Pipeline structure clear but some steps hand-waved (e.g., "compute historyadr somehow"). |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo and provides the correct translation.

| Grade | Bar |
|-------|-----|
| **A+** | Convention difference table present (EGT-9 cross-reference): (1) MuJoCo `sensor_history` (packed `[nsample, interp]` as `int*` × 2) → CortenForge `sensor_nsample: Vec<i32>` + `sensor_interp: Vec<InterpolationType>` (unpacked, typed — matching actuator pattern at `model.rs:592–596`); (2) MuJoCo `sensor_interval` (`mjtNum*` × 2) → CortenForge representation per AD-1; (3) MuJoCo `int` nsample → CortenForge `i32` (signed, matching `actuator_nsample: Vec<i32>` at `model.rs:592`); (4) shared `InterpolationType` enum (`enums.rs:256–279`) reused from actuator history — NOT duplicated; (5) MuJoCo `int interp` value 0/1/2 → `InterpolationType::Zoh/Linear/Cubic` via `FromStr` (`enums.rs:267`) and `From<i32>` (`enums.rs:281`); (6) naming convention follows existing `sensor_{name}` pattern from umbrella §3.1; (7) MJCF attribute names match MuJoCo exactly: `nsample`, `interp`, `delay`, `interval` (not snake_case variants); (8) `interp` keyword is **case-sensitive, lowercase only** in MuJoCo (EGT-11c) — CortenForge's `FromStr` (`enums.rs:270–273`) already matches this behavior; spec must explicitly state "no case-insensitive parsing" to prevent well-intentioned divergence; (9) `MjcfSensor` history fields use `Option<T>` (None = not specified), NOT the value-sentinel pattern used by existing `noise`/`cutoff` (`f64`, where `0.0` = "not set"). This matters for defaults: the existing `apply_to_sensor()` uses `if result.noise == 0.0` — an explicit `nsample=0` means "no history" and must NOT be treated as "not set." History attrs use `Option::is_none()` semantics, not value-sentinel. **However** per AD-3 recommendation (b), history attrs are NOT part of defaults at all, so this is a documentation concern rather than a code concern; (10) MuJoCo has **no sensor default class mechanism** (EGT-11h) — CortenForge's `MjcfSensorDefaults` is a pre-existing extension. Spec must document this divergence. |
| **A** | Major conventions documented. Minor field-type choices left to implementer. |
| **B** | Some conventions noted but packing/unpacking not addressed. |
| **C** | MuJoCo layout copied without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations.

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has three-part structure: (1) concrete MJCF model, (2) exact expected value from MuJoCo 3.5.0, (3) field to check. Parse ACs verify: `nsample=5, interp="linear"` → `model.sensor_nsample[0] == 5`, `model.sensor_interp[0] == InterpolationType::Linear`. Builder ACs verify: `nhistory == 12` for `nsample=5, dim=1` (from EGT-3); `sensor_historyadr[0] == 10` for mixed model with actuator nsample=4 (from EGT-6, V2). Validation ACs: (a) `delay=0.01` without `nsample` → error containing "setting delay > 0 without a history buffer" (matching actuator pattern at `actuator.rs:277`, adapted with sensor context per R16/R47); (b) `interval=-0.5` → error containing "negative interval in sensor" (matching MuJoCo's exact message per EGT-11b). Interval AC: `interval=0.5` → `model.sensor_interval[0] == (0.5, 0.0)` (from EGT-7). Default AC: no attrs specified → `sensor_nsample[0]==0, sensor_interp[0]==Zoh, sensor_delay[0]==0.0, sensor_interval[0]==(0.0, 0.0), sensor_historyadr[0]==-1` (from EGT-2). Interaction AC: `interval=0.5` without `nsample` → accepted, `sensor_interval` stored, `historyadr == -1` (EGT-11a). Multi-sensor AC: 3+ sensors with different nsample/dim values → `sensor_historyadr` offsets accumulate correctly, `nsample=0` sensor in middle gets `historyadr=-1` without breaking chain (EGT-11e, EGT-11f). Actuator stability AC: adding sensors with history does NOT change `actuator_historyadr` values (EGT-11g). Full-combination AC: one sensor with all 4 attrs (`nsample=5, interp="cubic", delay=0.02, interval=0.1`) on a dim=3 sensor → verify all 5 model fields simultaneously (EGT-11i). Vec-length invariant AC: for a model with `nsensor` sensors (some with history, some without), all new arrays have length `nsensor`: `sensor_nsample.len() == nsensor`, `sensor_interp.len() == nsensor`, `sensor_delay.len() == nsensor`, `sensor_interval.len() == nsensor`, `sensor_historyadr.len() == nsensor`. Code-review ACs explicitly labeled with structural properties (e.g., "all 5 new model fields initialized in `Model::empty()`", "all 4 new attrs parsed in `parse_sensor_attrs()`", "`parse_sensor_defaults()` NOT extended with history attrs (per AD-3)", "history attr pushes in same block as existing 11 fields at `sensor.rs:83–100`", "`model.rs:607–608` nhistory comment updated from actuator-only to actuator+sensor", "`sensor.rs:3` doc comment updated from 13 to 17", "`data.rs:78` doc comment updated from 'Actuator history buffer' to 'History buffer (actuators + sensors)'", "`enums.rs:253–254` doc comment updated to include sensor history usage"). |
| **A** | ACs are testable. Some lack MuJoCo-verified values. |
| **B** | ACs directionally correct but vague. |
| **C** | ACs are aspirational. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions. Each AC
> maps to at least one test. Negative cases (feature NOT triggered) are tested.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present — every AC maps to ≥1 test. Explicit edge case inventory: (1) `nsample=0` → no history, historyadr=-1; (2) negative `nsample=-1` → accepted, stored, historyadr=-1 (EGT-5); (3) `delay > 0` without nsample → error (EGT-4); (4) multi-dim sensor nhistory (dim=1, dim=3, dim=4 — EGT-3); (5) mixed actuator + sensor historyadr offsets (EGT-6); (6) `interval` attribute parsed and stored (EGT-7); (7) default values when no attrs specified (EGT-2); (8) interp keyword mapping: "zoh"→0, "linear"→1, "cubic"→2 (EGT-1); (9) invalid interp string → error; (10) `interval` without `nsample` → accepted, `historyadr=-1` (EGT-11a); (11) negative `interval` → error (EGT-11b); (12) `interp` case sensitivity — "Linear"/"ZOH"/"1" all rejected (EGT-11c); (13) `interp` stored even without `nsample` (EGT-11d); (14) multiple sensors with history — offset accumulation (EGT-11e); (15) `nsample=0` sensor between two with history — chain unbroken (EGT-11f); (16) actuator + multiple sensors — full layout with actuator stability (EGT-11g); (17) full combination — all 4 attrs on one sensor (EGT-11i); (18) negative delay accepted and stored (EGT-11j); (19) sensor-only model (no actuators) — historyadr starts at 0 (EGT-11k); (20) `interp=""` (empty string) → error "invalid keyword: ''" — distinct from mis-cased valid keyword (R61); (21) `interval > 0, delay > 0, nsample=0` → delay error — interval does NOT override nsample requirement for delay (R63, EGT-4). Negative cases tested: no history attrs → all defaults. At least one MuJoCo conformance test (same model, verify `nhistory` and `sensor_historyadr` match MuJoCo output — state MuJoCo version and expected value source). **No sensor defaults tests for history attrs** — MuJoCo has no sensor default mechanism (EGT-11h, AD-3). A test expecting `nsample` inheritance through CortenForge's `MjcfSensorDefaults` would be WRONG per MuJoCo behavior. Build-time regression test: at least one existing MJCF model (without history attrs) builds with identical sensor count and addresses after the Spec D changes — proves no regression in the sensor pipeline. **Test file location:** all Spec D sensor history tests go in `sim/L0/tests/integration/sensor_phase6.rs` (following the Phase 5 pattern: `actuator_phase5.rs`). |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. No MuJoCo conformance test. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs are
> explicitly stated. No circular dependencies or implicit ordering.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. States that Spec D is independent of Specs B and C (no runtime changes, no evaluation-level interaction). Notes Spec A dependency: Spec A's parser refactoring established the `parse_sensor_attrs()` function and `MjcfSensor` struct that Spec D extends — prerequisite commit hash stated. Section ordering: (1) `types.rs` (add 4 `Option` fields to `MjcfSensor`) → (2) `parser.rs` (parse 4 attrs in `parse_sensor_attrs()`) → (3) `builder/mod.rs` (add 4 `Vec` fields to `ModelBuilder`) → (4) `builder/sensor.rs` (validate + push in `process_sensors()`) → (5) `model.rs` + `model_init.rs` (add 5 fields to `Model` + `Model::empty()`) → (6) `build.rs` (transfer 4 fields, init historyadr to `vec![]`, compute historyadr + nhistory post-hoc). **defaults.rs is NOT modified** — per AD-3(b), history attrs are not part of the CortenForge-only sensor defaults extension. The historyadr computation in build.rs MUST run after both actuator and sensor arrays are populated. Cross-spec interaction: Spec D extends `nhistory` from actuator-only to actuator+sensor — the existing actuator historyadr code (`build.rs:387–405`) is not replaced but extended. |
| **A** | Order clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break. Surprises are spec bugs.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list matching EGT-10 inventory (12 modification sites across 7 files + 4 non-modification sites per AD-3(b)). Per-file change description present. Behavioral changes: (1) `nhistory` now includes sensor contributions — moves toward MuJoCo conformance (previously documented as actuator-only at `model.rs:607–608`); (2) `sensor_historyadr` computed for sensors — new field, no existing code reads it (conformance-neutral). **Stale documentation sites** (6 total): (a) `model.rs:607–608` comment says "actuator-only until sensor history is implemented" — must be updated (R20); (b) `sensor.rs:3` doc comment says "13 pipeline sensor arrays" — must be updated to 17 (R36, R49); (c) `data.rs:78` doc comment says "Actuator history buffer" — must be updated to "History buffer (actuators + sensors)" (R37); (d) `enums.rs:253–254` doc comment says "Interpolation method for actuator history buffer" and references only `actuator_history` — must be updated to include sensor usage (R38); (e) `sensor.rs:28` doc comment says "Unsupported type (Jointlimitfrc, Tendonlimitfrc)" — these types are now supported, `continue` is dead code (R50); (f) `sensor.rs:467` doc comment says "Returns None for types not yet implemented" — `convert_sensor_type` now returns `Some(...)` for all types (R50). **Non-modification sites** (explicitly NOT touched by Spec D): (a) `make_data()` (`model_init.rs:388–409`) — allocates `vec![0.0; self.nhistory]`, automatically grows with increased `nhistory`, no code change needed (R41); (b) `reset_data()` (`data.rs:874–890`) — fills `history` with 0.0 then pre-populates actuator history only, sensor pre-population is runtime scope (deferred), no code change needed (R39). Existing test impact: (a) `defaults.rs` tests `test_apply_to_sensor` (`defaults.rs:1220`) and `test_sensor_defaults_inheritance` (`defaults.rs:1259`) — should NOT need update per AD-3(b): history attrs are not in defaults; (b) full sim domain 2,238+ test baseline verified green; (c) `MjcfSensorDefaults` struct is NOT modified per AD-3(b) — no test impact from defaults changes. 9 of 12 modification sites are non-exhaustive (compiler won't catch omissions) — danger zones flagged in EGT-10. 3 sites are compiler-enforced (`Model::empty()`, `ModelBuilder::new()`, `MjcfSensor::default()` — all struct literals). Downstream consumer audit: search workspace for any code reading `model.nhistory` — if L1 (`sim-bevy`) or `sim-physics` read this value, they must be listed with impact assessment. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. Some silent-bug sites not flagged. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology
> throughout. Section references are accurate.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: "sensor history attributes" everywhere (not "history buffer parameters" or "sensor interpolation attributes" in some places). Field names consistent: `sensor_nsample`/`sensor_interp`/`sensor_historyadr`/`sensor_delay`/`sensor_interval` used identically in all sections. AC numbers match traceability matrix. File paths in Specification match Files Affected table. Convention table field names match model field declarations. Edge case list consistent between MuJoCo Reference (12 edge cases in P1 bar) and Test Plan (21 edge cases in P5 bar) — P5 is a superset of P1 (adds multi-sensor chain, full combination, build regression, sensor-only model, empty-string interp, interval+delay interaction). EGT cross-references accurate (EGT numbers in criterion bars match actual EGT section numbers: EGT-1 through EGT-13). Attribute count consistent: "4 attributes" in scope, parse, defaults, builder sections (not "3" in some places and "4" in others — the `interval` addition must be propagated everywhere). Validation count consistent: 2 validations (delay/nsample, negative interval) named in P2 step (4) match P4 validation ACs. Interp parsing (P2 step 3) precedes validation (P2 step 4), matching actuator ordering at `actuator.rs:265–278` (R59). P7 stale documentation site count: 6 sites (model.rs:607–608, sensor.rs:3, data.rs:78, enums.rs:253–254, sensor.rs:28, sensor.rs:467). P2 pipeline step count: 7 steps (parse → apply_to_sensor [untouched] → parse interp → validate → push → transfer 4 + init 1 → compute post-hoc). P7 non-modification sites: 2 (make_data, reset_data). EGT-10 site count: 12 modification + 4 non-modification = 16 total inventoried sites. **Mixed sentinel patterns in `MjcfSensor`:** `noise: f64` / `cutoff: f64` use value-sentinel (0.0 = "not set"); new `nsample: Option<i32>` / `interp: Option<String>` / `delay: Option<f64>` / `interval: Option<f64>` use Option-sentinel. This mixed pattern is a known inconsistency within `MjcfSensor`, documented as acceptable — changing either pattern would create a larger inconsistency with the rest of the codebase (R52). |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Actuator–Sensor History Parity

> The sensor history implementation must mirror the actuator history
> implementation exactly in naming pattern, validation logic, and field
> structure. Divergence from the established actuator pattern creates
> maintenance burden and suggests one implementation is wrong.
>
> **Boundary with P1:** P1 grades whether the spec GOT the MuJoCo reference
> right. P9 grades whether the CortenForge implementation follows the
> actuator history pattern already established in the codebase (EGT-9
> actuator field table).

| Grade | Bar |
|-------|-----|
| **A+** | Spec explicitly maps each actuator history field to its sensor counterpart using EGT-9: `actuator_nsample` (`model.rs:592`) → `sensor_nsample` (both `Vec<i32>`), `actuator_interp` (`model.rs:596`) → `sensor_interp` (both `Vec<InterpolationType>`), `actuator_historyadr` (`model.rs:600`) → `sensor_historyadr` (both `Vec<i32>`), `actuator_delay` (`model.rs:604`) → `sensor_delay` (both `Vec<f64>`). Validation logic mirrors `builder/actuator.rs:272–278`: `delay > 0.0 && nsample <= 0` → error. **Precision:** the check is `delay > 0.0` (strictly positive), NOT `delay != 0.0` or `delay.abs() > 0.0` — negative delay is accepted by MuJoCo (EGT-11j). **Error message text:** the actuator error at `actuator.rs:277` says "setting delay > 0 without a history buffer (nsample must be > 0)" — spec must state whether sensor uses identical message or adapts it (e.g., "in sensor" vs "in actuator"). MuJoCo's error says "setting delay > 0 without a history buffer" with entity context appended — spec must match. Sensor has additional validation not present in actuators: `interval < 0` → error "negative interval in sensor" (EGT-11b). `InterpolationType` enum shared (not duplicated) — parser string→enum conversion identical to `builder/actuator.rs:265–269`. Spec explicitly confirms `FromStr` is case-sensitive lowercase-only (EGT-11c) — no "convenience" case-insensitive parsing that would diverge from MuJoCo. Builder pattern mirrors `builder/actuator.rs:264–283` (parse interp, validate delay/nsample, push fields) with sensor-specific additions (interval, interval validation). Spec states where sensor implementation intentionally differs from actuator: (1) nhistory formula generalized from `2 * nsample + 2` to `nsample * (dim + 1) + 2` (V3 derivation proves equivalence for dim=1); (2) sensor has `interval` field with no actuator equivalent — explicitly documented as sensor-only (MuJoCo rejects `<motor interval="0.5"/>` per R28); (3) sensor has negative-interval validation with no actuator equivalent; (4) **defaults parity break:** actuators support per-element defaults (`<motor nsample="5"/>` in `<default>` works), sensors do NOT — MuJoCo has no sensor default mechanism (EGT-11h). This is the ONE place where actuator→sensor parity fundamentally breaks. Spec must document this explicitly and state that history attrs are parsed from sensor elements only, not from defaults. **Error message conformance boundary:** parse-time error messages (invalid interp keyword, negative interval, delay-without-history) are CortenForge-specific — no message-text conformance requirement with MuJoCo. Conformance requires matching MuJoCo's validation BEHAVIOR (what is accepted/rejected), not the exact error message phrasing. CortenForge's `FromStr` error ("invalid interp keyword '...': expected 'zoh', 'linear', or 'cubic'") is more informative than MuJoCo's ("invalid keyword: '...'") — this is acceptable. |
| **A** | Parity documented. One minor divergence unexplained. |
| **B** | Some fields mirror actuator pattern, others diverge without justification. |
| **C** | No parity analysis. |

### P10. History Buffer Continuity

> Sensor history addresses must be contiguous with actuator history in
> `nhistory`. The combined buffer layout must match MuJoCo's: actuators
> first, then sensors, with `nhistory = actuator_total + sensor_total`.
>
> **Boundary with P2:** P2 grades whether the build pipeline steps are
> complete. P10 grades whether the combined actuator+sensor layout is correct
> and the existing actuator code is not broken by the extension.

| Grade | Bar |
|-------|-----|
| **A+** | Spec states the combined layout: `nhistory = Σ(actuator contributions) + Σ(sensor contributions)`. Sensor `historyadr` values start at `actuator_nhistory_total`, not at 0. The existing `build.rs:387–405` actuator-only loop is shown being extended (not replaced) — the actuator loop runs first (unchanged), then a sensor loop appends. Concrete ACs: (a) single actuator + single sensor: actuator `nsample=4` (dim=1) → offset=0, contribution=10; sensor `nsample=3` (dim=1) → `historyadr=10`, contribution=8; total `nhistory=18` (from EGT-6, V2); (b) multi-dim sensor: actuator `nsample=4` (dim=1) → 10; sensor `nsample=3` (dim=3, framepos) → `historyadr=10`, contribution=`3*(3+1)+2=14`; total `nhistory=24`; (c) multiple actuators + multiple sensors: 2 actuators (nsample=4,2) + 2 sensors (nsample=3,5, dim=1) → `actuator_historyadr=[0,10]`, `sensor_historyadr=[16,24]`, `nhistory=36` (EGT-11g); (d) `nsample=0` sensor in middle of chain: `sensor_historyadr=[0,-1,8]` — zero-sample sensor gets `-1` without breaking offset accumulation for subsequent sensors (EGT-11f); (e) **sensor-only model** (0 actuators): sensor `nsample=3` (dim=1) → `sensor_historyadr=[0]`, `nhistory=8`, `actuator_historyadr` empty (EGT-11k) — verifies the sensor loop correctly starts at offset 0 when no actuators contribute. Post-hoc computation order documented: actuators first, then sensors. **Actuator stability guarantee:** AC explicitly verifying that `actuator_historyadr` values are identical with and without sensor history — adding sensors must NOT shift actuator offsets. **Arithmetic safety:** The sensor formula `nsample * (dim + 1) + 2` involves mixed types (`nsample: i32`, `dim: usize`). Existing actuator formula stays in `i32`. Spec must specify cast sequence: only compute when `ns > 0` (already guarded), then `ns * (dim as i32 + 1) + 2` keeping `offset: i32` (`ns` is already `i32`, no redundant cast). The `ns > 0` guard prevents negative-nsample wrapping. |
| **A** | Layout correct. Offset example present but only single-dim case. |
| **B** | Layout described conceptually but no concrete example of mixed case. |
| **C** | No discussion of combined layout. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names exact MuJoCo source locations
      (`mjspec.h:731–734`, `mjmodel.h:1218–1221`, `mjxmacro.h:695–698`),
      exact CortenForge files with line numbers (EGT-9: `model.rs:592`,
      `model.rs:596`, `model.rs:600`, `model.rs:604`, `model.rs:609`,
      `builder/actuator.rs:264–283`, `build.rs:387–405`, `enums.rs:256–279`,
      `parser.rs:3453`, `parser.rs:909`, `defaults.rs:533`, `defaults.rs:815`),
      exact edge cases (12 in P1, 21 in P5), exact numerical expectations
      (EGT-3: 9 formula verifications, EGT-6: mixed model nhistory=18,
      EGT-11e–g: multi-sensor/multi-actuator layouts). EGT-10 inventory:
      12 modification sites + 4 non-modification sites (AD-3(b)), 3 compiler-
      enforced, 9 non-exhaustive. Two independent reviewers could assign
      grades by pointing to specific lines.

- [x] **Non-overlap:** P1 vs P9 boundary: P1 grades MuJoCo reference
      accuracy; P9 grades CortenForge actuator pattern parity (same content
      from different angle — P1 asks "is the MuJoCo behavior right?", P9
      asks "does the implementation follow the actuator precedent?"). P2 vs
      P10 boundary: P2 grades pipeline step completeness; P10 grades combined
      actuator+sensor buffer layout correctness. P3 vs P9: P3 grades
      MuJoCo→CortenForge convention translation (types, naming); P9 grades
      actuator→sensor pattern consistency within CortenForge. P4 vs P8: P4
      grades whether each AC cites a specific value; P8 grades whether values
      are mutually consistent across sections.

- [x] **Completeness:** 10 criteria cover: MuJoCo reference (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), dependencies (P6), blast
      radius (P7), consistency (P8), actuator parity (P9), buffer layout
      (P10). Spec D is parse-and-store — no runtime evaluation, no new enum
      variants, no postprocess changes. The two domain criteria (P9, P10)
      cover the dimensions unique to this task: mirroring the actuator
      pattern and getting the combined history buffer right.

- [x] **Gradeability:** Each criterion maps to specific spec sections:
      P1 → MuJoCo Reference + Key Behaviors; P2 → Specification (S1, S2, ...);
      P3 → Convention Notes + Specification code; P4 → Acceptance Criteria;
      P5 → Test Plan + Traceability Matrix + Edge Case Inventory;
      P6 → Prerequisites + Execution Order; P7 → Blast Radius + Files Affected;
      P8 → cross-cutting; P9 → Convention Notes + Specification (builder);
      P10 → Specification (build.rs extension) + ACs (mixed model).

- [x] **Conformance primacy:** P1 tailored with 4 MuJoCo source files,
      11 specific edge cases, MuJoCo 3.5.0 empirical values (EGT-1 through
      EGT-11). P4 requires MuJoCo-verified expected values. P5 requires
      MuJoCo conformance test. P10 requires mixed actuator+sensor
      verification against MuJoCo. Rubric cannot produce an A+ spec that
      diverges from MuJoCo's behavior — every numerical expectation is
      empirically verified and analytically derived (EGT-12 V1–V3).

- [x] **Stress-tested:** 53 "A+ but still broken" entries across 9 rounds:
      **Round 1 (R13–R24):** forgotten defaults code path (R13), insufficient
      multi-sensor ACs (R14), unverified attribute interactions (R15), error
      message text divergence (R16), actuator stability guarantee (R17),
      case-sensitivity conformance (R18), merge/apply ordering (R19), stale
      comments (R20), downstream consumers (R21), build regression (R22),
      cross-criterion consistency (R23), field existence verification (R24).
      **Round 2 (R25–R30):** MuJoCo has NO sensor defaults — entire
      `MjcfSensorDefaults` mechanism is CortenForge-only (R25), value-sentinel
      vs Option-sentinel pattern conflict (R26), ModelBuilder init is
      compiler-enforced (R27), actuators lack `interval` (R28), no
      full-combination AC (R29), stale internal counts (R30).
      **Round 3 (R31–R36):** `MjcfSensor::default()` is also
      compiler-enforced (R31), push-block atomicity for Vec alignment (R32),
      mixed i32/usize arithmetic in nhistory formula (R33), Vec-length
      invariant (R34), apply_to_sensor scope clarification (R35), stale
      doc comment at sensor.rs:3 (R36).
      **Round 4 (R37–R43):** stale `data.rs:78` doc comment (R37), stale
      `enums.rs:253–254` doc comment (R38), `reset_data()` non-modification
      (R39), transfer-vs-posthoc field distinction (R40), `make_data()`
      non-modification (R41), test file location unspecified (R42), stale
      doc site not in EGT-10 (R43).
      **Round 5 (R44–R48):** **EGT-10 vs AD-3(b) contradiction** — 4 rows
      told implementer to modify defaults files that AD-3(b) says NOT to
      modify (R44), P6 ordering included defaults.rs (R45), site counts
      stale after AD-3(b) correction (R46), validation error message text
      under-specified in P4 (R47), parse-time error conformance boundary
      not stated (R48).
      **Round 6 (R49–R53):** pipeline array count wrong — 13+5=18 should
      be 13+4=17 (R49), `convert_sensor_type` has no `None` paths —
      `continue` is dead code and 2 doc comments stale (R50), `MjcfSensor`
      fluent API methods missing from EGT-10 (R51), mixed value-sentinel
      and Option-sentinel patterns in `MjcfSensor` (R52), P4 code-review
      AC used vague "new count" instead of exact "17" (R53).
      **Round 7 (R54–R58):** **negative delay accepted** by MuJoCo —
      validation is `> 0.0` not `!= 0.0` (R54), P2 missing unwrap defaults
      for all 4 fields (R55), P10 redundant `ns as i32` cast (R56), no
      sensor-only AC in P10 (R57), validation condition precision — must
      use `delay > 0.0` not `delay.abs() > 0.0` (R58).
      **Round 8 (R59–R63):** **P2 validation/interp ordering wrong** —
      MuJoCo validates interp BEFORE delay/nsample, matching actuator
      pattern at `actuator.rs:265–278` (R59), invalid interp must return
      `Err(ModelConversionError)` not silent default to Zoh (R60), empty
      string `interp=""` rejected by MuJoCo — missing from P5 edge cases
      (R61), error type conversion `.map_err(|e| ModelConversionError {
      message: e })` unspecified (R62), `interval+delay` without nsample →
      delay error — validations are independent (R63).
      **Round 9 (R64–R65):** final verification pass — P4 Default AC
      missing `sensor_interval` (5th field) (R64), self-audit edge case
      count stale "19 in P5" → "21 in P5" (R65). **2 findings only —
      both minor consistency fixes. No structural or behavioral gaps found.
      Rubric is ready to lock.**

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
| P9 | Convention Notes, Specification (builder section), model fields |
| P10 | Specification (build.rs extension), Acceptance Criteria (mixed model AC) |

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
| P9. Actuator–Sensor History Parity | | |
| P10. History Buffer Continuity | | |

**Overall:**

> **Grading note:** The Evidence column is not optional. For each grade, cite
> the specific spec content that justifies it. For each gap, state exactly
> what's missing. "Looks good" is not evidence.

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope | Umbrella listed 3 attrs (nsample/interp/delay) — MuJoCo 3.5.0 also has `interval` (`mjsSensor_::interval[2]` at `mjspec.h:734`, `sensor_interval` at `mjmodel.h:1221`) | Phase 1 empirical testing (EGT-7) | Added `interval` to scope. Created Scope Adjustment section. Added AD-2 architectural decision. Propagated to all criterion bars, EGT-9, EGT-10. | Rev 1 |
| R2 | P10 | Initial draft did not specify sensor historyadr offset relative to actuator total | Phase 1 empirical testing (EGT-6) | Added mixed-model AC requirement to P10. Stated sensors start at `actuator_nhistory_total`. Added V2 derivation. | Rev 1 |
| R3 | P1 | `sensor_intprm` (`mjmodel.h:1213`) could be confused with `sensor_history` | Phase 1 C source review (EGT-8) | Added EGT-8 distinguishing the two arrays. Added to P1 A+ bar as explicit requirement. | Rev 1 |
| R4 | All | Scorecard was pre-filled with A+ — rubric grades the *spec*, not itself | Peer comparison (Spec A/B/C rubrics leave scorecard blank) | Cleared scorecard to blank. | Rev 2 |
| R5 | All | Empirical facts labeled "Fact 1–8" instead of "EGT-N" convention | Peer comparison (all peers use EGT-N labeling) | Renamed to EGT-1 through EGT-8. Updated all cross-references. | Rev 2 |
| R6 | P2, P10 | Missing analytical verification derivations (peers have V-derivations) | Peer comparison (Spec A has V1–V4, Spec C has V1–V11) | Added EGT-11 with V1 (buffer layout), V2 (mixed offset), V3 (actuator formula equivalence). | Rev 2 |
| R7 | P7 | Missing exhaustive match-site inventory with danger flags | Peer comparison (Spec C has 18 sites across 5 tables with exhaustiveness flags) | Added EGT-10 with 14 sites across 5 tables. Flagged 13 non-exhaustive danger zones. | Rev 2 |
| R8 | P6, P9 | Missing architectural decisions (peers have AD-N sections) | Peer comparison (Spec C has AD-1 through AD-4) | Added EGT-13 with AD-1 (interval representation), AD-2 (interval scope), AD-3 (defaults inheritance). | Rev 2 |
| R9 | P3, P9 | Missing CortenForge field-name mapping table | Peer comparison (Spec C has EGT-7 with file:line for every field) | Added EGT-9 with 3 sub-tables: actuator history (pattern), existing sensor, proposed sensor history. All with file:line. | Rev 2 |
| R10 | P8 | Attribute count consistency risk: umbrella says "3" but scope is now "4" | Self-audit (interval addition) | Added explicit P8 A+ bar requirement: "4 attributes" consistent everywhere. | Rev 2 |
| R11 | P5 | Missing default class inheritance edge case in test plan | Self-audit (AD-3 analysis) | Added to P5 A+ bar: "Default class inheritance test". | Rev 2 |
| R12 | P10 | Only single-dim mixed-model example; no multi-dim case | Self-audit (P10 A+ bar review) | Added second AC to P10 A+ bar: actuator nsample=4 (dim=1) + sensor nsample=3 (dim=3) → nhistory=24. | Rev 2 |
| R13 | P2 | `parse_sensor_defaults()` (`parser.rs:909`) is a **separate code path** from `parse_sensor_attrs()` — not called from it, not in the same function. A spec could describe sensor element parsing at A+ level while completely ignoring defaults parsing. The 4 new attributes must be parsed in BOTH functions independently. | "A+ but still broken" stress test (round 1) | Added step (0) to P2 A+ bar: `parse_sensor_defaults()` as separate pipeline step. Added to P4 code-review AC: "all 4 new attrs parsed in `parse_sensor_defaults()`". | Rev 3 |
| R14 | P10 | Only 1-sensor and 1-actuator+1-sensor ACs. No AC with 2+ sensors with history, no AC with nsample=0 sensor in the middle of the chain. `historyadr` offset accumulation across multiple sensors is a distinct bug surface — an implementation that hardcodes `sensor_historyadr[0] = actuator_total` but forgets to accumulate for subsequent sensors would pass all existing ACs. | "A+ but still broken" stress test (round 1) | Added ACs (c) and (d) to P10 A+ bar: 2 actuators + 2 sensors (EGT-11g), nsample=0 middle sensor (EGT-11f). Added edge cases (14)–(16) to P5 bar. | Rev 3 |
| R15 | P1 | `interval` without `nsample` — rubric assumed (but did not verify) that `interval` requires `nsample > 0`. MuJoCo 3.5.0 ACCEPTS `interval=0.5` without nsample (EGT-11a). Spec must NOT add validation coupling them. Also: negative `interval` rejected (EGT-11b), `interval=0` accepted. Three distinct interaction behaviors — all missing from P1. | "A+ but still broken" stress test (round 1) — empirical verification | Added edge cases (8)–(9) to P1 A+ bar. Added EGT-11a and EGT-11b. Added validation AC (b) and interaction AC to P4 bar. | Rev 3 |
| R16 | P9 | Error message text: actuator validation at `actuator.rs:277` says "setting delay > 0 without a history buffer (nsample must be > 0)". A spec that says "same message" would produce a sensor error containing no sensor context. MuJoCo appends element context. Spec must specify exact error wording for sensor validation, not just "same as actuator." Also: sensor has a SECOND validation (negative interval) with no actuator equivalent — P9 must document this as an intentional divergence. | "A+ but still broken" stress test (round 1) | Updated P9 A+ bar: error message text requirement, negative-interval validation as sensor-specific divergence item (3). | Rev 3 |
| R17 | P10 | No AC verifying actuator historyadr stability. An implementation could reset the offset counter or re-sort the combined array, changing actuator offsets. Adding sensor history must NOT change ANY actuator_historyadr value. This is the "primum non nocere" requirement for extending the history buffer. | "A+ but still broken" stress test (round 1) | Added actuator stability guarantee to P10 A+ bar. Added actuator stability AC to P4 bar. | Rev 3 |
| R18 | P3, P9 | `interp` keyword is case-sensitive in MuJoCo: "Linear", "ZOH", "1" all rejected (EGT-11c verified empirically). CortenForge's `FromStr` (`enums.rs:270–273`) is already lowercase-only — but a spec writer or implementer might add `.to_lowercase()` "for robustness." P3 must explicitly forbid case-insensitive parsing. Also: `interp` stored even when `nsample=0` (EGT-11d) — spec must NOT optimize by skipping interp when nsample is absent. | "A+ but still broken" stress test (round 1) — empirical verification | Added P3 rule (8): case-sensitive lowercase-only. Updated P9: `FromStr` case sensitivity confirmed. Added EGT-11c and EGT-11d. Added edge case (12)–(13) to P5 bar. | Rev 3 |
| R19 | P2 | Defaults merge→apply ordering not specified. `merge_sensor_defaults()` runs during class tree resolution (before sensor processing). `apply_to_sensor()` runs during `process_sensors()` (after merge). If spec describes both functions correctly but doesn't state their execution order, an implementer might call apply before merge — applying unmerged parent defaults. The existing code at `defaults.rs:815` (merge) and `defaults.rs:533` (apply) already has this order, but the spec must be explicit. | "A+ but still broken" stress test (round 1) | Added step (1.5) to P2 A+ bar: merge runs during class resolution, BEFORE process_sensors. Added multi-level defaults test to P5 bar: grandparent→parent→child chain. | Rev 3 |
| R20 | P7 | `model.rs:607–608` comment says "actuator-only until sensor history is implemented" — after Spec D, this comment is stale. A spec could implement sensor history correctly while leaving a misleading comment in the codebase. The rubric's P7 already mentions this line but didn't require updating the comment. | "A+ but still broken" stress test (round 1) | Added stale comment update to P7 A+ bar. Added code-review AC to P4: comment update required. | Rev 3 |
| R21 | P7 | No downstream consumer audit for `model.nhistory`. If `sim-bevy` (L1) or `sim-physics` reads `nhistory` to allocate buffers, changing it from actuator-only to actuator+sensor could change allocation sizes. All P7 analysis focused on L0 crates. | "A+ but still broken" stress test (round 1) | Added downstream consumer audit to P7 A+ bar: search workspace for `nhistory` readers. | Rev 3 |
| R22 | P5 | No build-time regression test. Spec D adds new fields to `MjcfSensor`, `MjcfSensorDefaults`, `ModelBuilder`, and `Model`. If any existing MJCF file fails to parse or build after these changes, it's a silent regression. P5 edge cases are all about NEW history attrs — no test verifies existing models still work. | "A+ but still broken" stress test (round 1) | Added build-time regression test to P5 A+ bar. | Rev 3 |
| R23 | P8 | P5 now has 16 edge cases but P1 has 11 — P8 must reconcile. P5 is a superset (adds multi-sensor, multi-level defaults, build regression). Also: validation count must be consistent — P2 step (3) names 2 validations (delay/nsample, negative interval), P4 must have matching validation ACs, P5 must have matching error tests. | "A+ but still broken" stress test (round 1) | Updated P8 A+ bar: P1 (11 edge cases) ⊂ P5 (16 edge cases) explicitly stated. Validation count consistency added. | Rev 3 |
| R24 | P2 | Field existence not verified. P2 pipeline references `InterpolationType` (`enums.rs:267`), `MjcfSensor` (`types.rs:3097`), `ModelBuilder` (`builder/mod.rs:635`). A spec could write correct pipeline pseudocode referencing fields that don't exist in CortenForge's current structs — producing compilation failure. For every field referenced, spec must verify it exists (citing file:line) or specify its addition. | "A+ but still broken" stress test (round 1) — adapted from Spec A R22 and Spec B R11 | Added field existence requirement to P2 A+ bar. | Rev 3 |
| R25 | P1, P3, P9 | **MuJoCo 3.5.0 does NOT support sensor defaults.** `<sensor>` inside `<default>` → "unrecognized element". Per-type elements (`<jointpos>`) inside `<default>` → also rejected. Even `class` attribute on sensor elements → rejected. MuJoCo sensors have NO default class mechanism whatsoever — not for `noise`, `cutoff`, or any attribute. CortenForge's `MjcfSensorDefaults` struct (`types.rs:775`), `parse_sensor_defaults()` (`parser.rs:909`), `apply_to_sensor()` (`defaults.rs:533`), and `merge_sensor_defaults()` (`defaults.rs:815`) are **CortenForge-only extensions** accepting MJCF that MuJoCo rejects. AD-3's recommendation to add history attrs to `MjcfSensorDefaults` is adding to a mechanism that has no MuJoCo basis. The spec must: (1) document this as a known pre-existing CortenForge extension, (2) state explicitly that MuJoCo has no sensor defaults, (3) decide whether to extend the CortenForge-only mechanism or not. P9 impact: actuators DO have per-element defaults (`<motor nsample="5"/>` works) — sensors do NOT, so actuator→sensor parity breaks at the defaults layer. | "A+ but still broken" stress test (round 2) — empirical verification | Updated AD-3 with discovery. Added EGT-11h. Updated P1/P3/P9 bars. | Rev 4 |
| R26 | P3 | `apply_to_sensor()` uses value-sentinel pattern: `if result.noise == 0.0` — treats 0.0 as "not set" (not `Option::is_none()`). For `noise`/`cutoff`, this is defensible (zero = no noise = default). But for `nsample`, an explicit `nsample=0` means "no history" and is semantically valid. If the apply function uses `== 0` pattern for nsample, an explicit `nsample=0` would get overridden by a default `nsample=5`. The new history attrs are `Option<T>` in `MjcfSensor` — the apply logic must use `.is_none()` sentinel, not value-sentinel. The spec must specify which pattern to use and WHY. | "A+ but still broken" stress test (round 2) | Added to P3 A+ bar: Option-sentinel vs value-sentinel pattern requirement. | Rev 4 |
| R27 | P7 | EGT-10 Table 4 row 1 (`builder/mod.rs:635`) says adding new Vec fields to `ModelBuilder` struct is non-exhaustive ("Exhaustive? N/A (struct)"). But `ModelBuilder::new()` in `init.rs:13` uses a struct literal — forgetting a new field IS a compiler error. The danger annotation is wrong: `ModelBuilder` has TWO modification sites (struct definition + init.rs), and init.rs IS compiler-enforced. Corrected exhaustiveness assessment changes the danger zone count from 13 non-exhaustive to 12. | "A+ but still broken" stress test (round 2) — codebase audit | Updated EGT-10 Table 4: added `init.rs:13` as compiler-enforced site. Fixed exhaustiveness count. | Rev 4 |
| R28 | P9 | Actuators DO NOT have `interval` attribute (empirically verified: `<motor interval="0.5"/>` → "unrecognized attribute"). So `interval` is sensor-only with no actuator counterpart. P9 already notes this in divergence item (2), but the spec must also address: is `interval` a sensor-only validation site? If the spec adds negative-interval validation inside `process_sensors()`, does the actuator builder need a matching check? No — actuators can't have interval. But the spec must STATE this to prevent an implementer from searching for the "actuator equivalent" and getting confused. | "A+ but still broken" stress test (round 2) — empirical verification | Updated P9: actuators confirmed to lack `interval`. Spec must state sensor-only explicitly. | Rev 4 |
| R29 | P4, P5 | No AC for the "full combination" — a sensor with ALL 4 attributes set simultaneously (`nsample=5, interp="cubic", delay=0.02, interval=0.1`). Individual attrs are tested but the combination exercises the complete builder path in a single sensor. MuJoCo 3.5.0 verified: `sensor_history=[[5,2]], sensor_delay=[0.02], sensor_interval=[[0.1,0.0]], nhistory=22` for dim=3 framepos. Without this AC, an interaction bug between interp parsing and delay validation could go undetected. | "A+ but still broken" stress test (round 2) — empirical verification | Added full-combination AC to P4 and P5 bars. Added EGT-11i. | Rev 4 |
| R30 | P8 | Stale count: self-audit checklist says "13 non-exhaustive danger zones" but R27 corrected this to 12 (init.rs is compiler-enforced). Also, modification site count needs update: 14 → 15 (init.rs is a separate site from mod.rs). | "A+ but still broken" stress test (round 2) — internal consistency | Updated self-audit counts. | Rev 4 |
| R31 | P7 | EGT-10 Table 1 says `MjcfSensor::default()` impl is "the highest-risk silent-bug site: forgetting a new field means it gets Rust's `Default` for the type." But `types.rs:3129–3147` shows a **manual `impl Default`** with a struct literal (`Self { name: ..., class: ..., ... }`). Forgetting a new field in a struct literal is a **compiler error**, not a silent bug. The DANGER annotation is wrong — this site IS compiler-enforced (3rd such site, after `Model::empty()` and `ModelBuilder::new()`). Corrected count: 3 compiler-enforced, 12 non-exhaustive. | "A+ but still broken" stress test (round 3) — codebase audit | Updated EGT-10 Table 1 danger flag. Updated summary count. | Rev 5 |
| R32 | P2 | History attr pushes must happen in the SAME push block as existing sensor fields at `builder/sensor.rs:83–100`. The existing code pushes 11 fields (type, datatype, objtype, objid, reftype, refid, adr, dim, noise, cutoff, name) inside the `for mjcf_sensor in sensors` loop, after object resolution. The `continue` at line 33 (unsupported sensor types) skips ALL pushes for that sensor — maintaining Vec length alignment. If history attr pushes are added in a SEPARATE loop or after the main loop, Vec lengths diverge for models with unsupported sensor types (Jointlimitfrc, Tendonlimitfrc). The spec must state: "push history attrs in the same push block as existing fields, at `sensor.rs:83–100`." | "A+ but still broken" stress test (round 3) — codebase audit | Added push-block atomicity requirement to P2 A+ bar. | Rev 5 |
| R33 | P10 | Sensor nhistory formula `nsample * (dim + 1) + 2` involves mixed-type arithmetic: `nsample` is `i32` (signed, to accept negatives per EGT-5), `dim` is `usize`. The existing actuator formula `2 * ns + 2` stays in `i32` (both operands `i32`). The sensor formula requires either: (a) cast `dim` to `i32` first → `ns * (dim as i32 + 1) + 2` (risks negative dim, but dim is always positive), or (b) cast `ns` to `usize` first → but `ns` can be negative (EGT-5), and `ns as usize` for negative `ns` wraps to huge number. Safe approach: only compute when `ns > 0` (already guarded by `if ns > 0`), then `ns as usize * (dim + 1) + 2`. The spec must specify the exact cast sequence to prevent overflow or wrapping. The `offset` accumulator at `build.rs:393` is `i32` — the sensor contribution must fit in `i32`. | "A+ but still broken" stress test (round 3) | Added arithmetic safety requirement to P10 A+ bar. | Rev 5 |
| R34 | P4 | No AC verifying Vec length invariants. After building a model with mixed history/no-history sensors, all new arrays must have length `nsensor` — even sensors without history attrs get entries (`nsample=0, interp=Zoh, delay=0.0, interval=(0.0,0.0), historyadr=-1`). MuJoCo 3.5.0 confirmed: `sensor_history.shape == (nsensor, 2)`, `sensor_delay.shape == (nsensor,)`, `sensor_interval.shape == (nsensor, 2)`, `sensor_historyadr.shape == (nsensor,)`. An implementation that only pushes for sensors with `nsample > 0` would have `sensor_nsample.len() < nsensor`. | "A+ but still broken" stress test (round 3) — empirical verification | Added Vec-length AC to P4 A+ bar. | Rev 5 |
| R35 | P2 | `process_sensors()` calls `self.resolver.apply_to_sensor()` at `sensor.rs:26` BEFORE the push block. This is where `noise`/`cutoff`/`user` defaults are applied. Per AD-3(b), history attrs are NOT in defaults. But the spec must explicitly state that `apply_to_sensor()` does NOT modify history fields — otherwise an implementer might add history attrs to the apply function thinking it's the "right place" for defaults. The spec must describe the data flow: parser sets `MjcfSensor.nsample = Some(5)` → `apply_to_sensor()` leaves `nsample` untouched → builder reads `sensor.nsample.unwrap_or(0)`. | "A+ but still broken" stress test (round 3) | Added apply_to_sensor scope clarification to P2 A+ bar. | Rev 5 |
| R36 | P2 | `sensor.rs:27–34` skips unsupported sensor types with `continue`. After Spec D, the doc comment at line 3 says "13 pipeline sensor arrays." This must be updated to reflect the new count (13 + 5 history = 18, or whatever the actual number is). A stale doc comment won't break compilation but misleads future implementers. | "A+ but still broken" stress test (round 3) — codebase audit | Added doc comment update to P7 A+ bar (stale documentation sites). | Rev 5 |
| R37 | P7 | `data.rs:78` doc comment says "Actuator history buffer (length `nhistory`)." After Spec D, the history buffer contains BOTH actuator and sensor history. This comment becomes misleading — it implies only actuators contribute. Must be updated to "History buffer (actuators + sensors)." Same pattern as the `model.rs:607–608` stale comment (R20) but in Data, not Model. | "A+ but still broken" stress test (round 4) — codebase audit | Added `data.rs:78` to P7 stale documentation sites. | Rev 6 |
| R38 | P7 | `enums.rs:253` doc comment says "Interpolation method for actuator history buffer." After Spec D, `InterpolationType` is shared between actuators and sensors. The doc comment misleadingly restricts it to actuators. Must be updated to "Interpolation method for history buffer" (or "for actuator/sensor history buffer"). `enums.rs:254` also says `actuator_history[2*i + 1]` — should mention `sensor_history[2*i + 1]` as well. | "A+ but still broken" stress test (round 4) — codebase audit | Added `enums.rs:253–254` to P7 stale documentation sites. | Rev 6 |
| R39 | P2, P7 | `reset_data()` (`data.rs:874–890`) pre-populates actuator history only (fills metadata + past timestamps). After Spec D increases `nhistory`, the `history.fill(0.0)` at line 877 zeroes the entire buffer (including the new sensor portion — correct for initial state). The actuator pre-population loop (lines 878–890) only touches actuator addresses — also correct. BUT: the spec doesn't explicitly state that `reset_data()` is NOT modified by Spec D. An implementer might add sensor pre-population logic here, which is runtime scope (deferred per umbrella §Out of Scope). P2 must state: "No changes to `reset_data()` — sensor history pre-population is runtime scope." P7 must state: "`reset_data()` is NOT a modification site." | "A+ but still broken" stress test (round 4) — codebase audit | Added reset_data() non-modification note to P2 and P7 bars. | Rev 6 |
| R40 | P2, P10 | `build.rs:267–269` initializes `actuator_historyadr: vec![]` and `nhistory: 0` in the Model construction block because they're computed post-hoc. The sensor equivalents (`sensor_historyadr`) must follow the same pattern: initialized to `vec![]` at construction, then filled in the post-hoc computation block. But the 4 directly-transferred fields (nsample, interp, delay, interval) are transferred via `self.sensor_nsample`, not initialized to defaults. The spec must distinguish: **4 fields transferred** from builder (nsample, interp, delay, interval at `build.rs:230`), **1 field computed post-hoc** (historyadr, initialized `vec![]` at construction, filled at `build.rs:387+`). This distinction is critical — mixing them up means either (a) forgetting the transfer and getting empty vecs, or (b) putting historyadr in the transfer block and skipping post-hoc computation. | "A+ but still broken" stress test (round 4) — codebase audit | Added transfer-vs-posthoc distinction to P2 A+ bar. | Rev 6 |
| R41 | P7 | `make_data()` (`model_init.rs:388–409`) allocates `vec![0.0f64; self.nhistory]` and pre-populates actuator history. After Spec D, `self.nhistory` is larger (includes sensors), so the allocation automatically grows. `make_data()` requires NO code changes — the increased `nhistory` is handled by the existing `vec![0.0; self.nhistory]` allocation. `reset_data()` similarly requires no changes (R39). The P7 blast radius must explicitly note these as **non-modification sites** that benefit from the increased `nhistory` automatically. An implementer who doesn't understand this might add redundant sensor allocation code in `make_data()`. | "A+ but still broken" stress test (round 4) — codebase audit | Added make_data()/reset_data() non-modification note to P7 bar. | Rev 6 |
| R42 | P5 | Test file location not specified. Phase 5 actuator tests are in `sim/L0/tests/integration/actuator_phase5.rs`. Phase 6 sensor history tests should have a named location. Without this, the implementer might scatter tests across multiple files or put them in unit tests within the builder crate. P5 should specify the test file (e.g., `sim/L0/tests/integration/sensor_phase6.rs` or `sim/L0/tests/integration/sensor_history.rs`). | "A+ but still broken" stress test (round 4) — peer comparison (Phase 5 pattern) | Added test file location to P5 bar. | Rev 6 |
| R43 | P7 | EGT-10 inventory has 15 modification sites, but `enums.rs:253–254` (stale doc comment) is a 16th site that requires modification. Strictly, EGT-10 counts "code modification" sites (struct fields, function logic), not documentation sites. But the `enums.rs` doc comment change is needed for P7 blast radius completeness. Decision: keep EGT-10 at 11 code sites (after AD-3(b) correction), add `enums.rs` to P7 stale documentation list alongside `model.rs:607–608`, `sensor.rs:3`, and `data.rs:78`. | "A+ but still broken" stress test (round 4) — internal consistency | Added `enums.rs:253–254` to P7 stale documentation sites. EGT-10 unchanged (doc comments are not code sites). | Rev 6 |
| R44 | P8 | **EGT-10 Tables 1/2/3 contradicted AD-3(b).** EGT-10 was written in Rev 2 (R7), before R25 discovered MuJoCo has no sensor defaults. Four EGT-10 rows told the implementer to add history attrs to `MjcfSensorDefaults` (Table 1 row 3), `parse_sensor_defaults()` (Table 2 row 2), `merge_sensor_defaults()` (Table 3 row 1), and `apply_to_sensor()` (Table 3 row 2). But AD-3(b) (Rev 4) says "don't extend a non-MuJoCo mechanism further." An implementer following EGT-10 would do exactly what AD-3(b) forbids. This is the most dangerous class of rubric bug: two sections of the same document giving contradictory instructions. | "A+ but still broken" stress test (round 5) — contradiction hunt | Updated all 4 EGT-10 rows to say "NOT MODIFIED per AD-3(b)." Updated Table 3 header. Corrected site count from 15 to 11. | Rev 7 |
| R45 | P6 | P6 ordering listed `defaults.rs` as step (3) in the implementation sequence: "(1) types.rs → (2) parser.rs → **(3) defaults.rs** → (4) builder/mod.rs → ...". But AD-3(b) says defaults.rs is NOT modified by Spec D. An implementer following P6's ordering would plan work on defaults.rs, open the file, and add code that shouldn't be there. The ordering must skip defaults.rs. | "A+ but still broken" stress test (round 5) — contradiction hunt | Removed defaults.rs from P6 ordering. Updated to 6-step sequence. | Rev 7 |
| R46 | P7, P8 | EGT-10 site count was stale: "15 modification sites across 9 files" (originally from Rev 2, partially updated in R27/R31 but never after R25). With AD-3(b) removing 4 defaults-related sites, correct count is 11 modification sites across 7 files + 4 non-modification sites. The EGT-10 summary, P7 A+ bar, and self-audit checklist all referenced the old count. Compiler-enforced count changed: 3 of 11 (not 3 of 15), non-exhaustive: 8 of 11 (not 12 of 15). | "A+ but still broken" stress test (round 5) — internal consistency | Updated EGT-10 summary, P7 bar, self-audit checklist counts. | Rev 7 |
| R47 | P4 | Validation error message text under-specified. P4 AC for delay validation says "error matching EGT-4" but doesn't state the **CortenForge** error message. EGT-4 shows MuJoCo's message ("Error: setting delay > 0 without a history buffer"). The actuator implementation at `actuator.rs:277` says "setting delay > 0 without a history buffer (nsample must be > 0)". The sensor message should match the actuator pattern with sensor context (R16 identified this but P4 didn't incorporate it into an AC). P4 should specify: (a) delay validation error text matches actuator pattern, (b) interval validation error text matches MuJoCo's "negative interval in sensor" (EGT-11b). | "A+ but still broken" stress test (round 5) — AC completeness | Added error message text ACs to P4 bar. | Rev 7 |
| R48 | P9 | Parse-time error messages (invalid interp keyword) diverge between MuJoCo and CortenForge. MuJoCo says "invalid keyword: 'Linear'" (EGT-11c). CortenForge's `FromStr` at `enums.rs:274` says "invalid interp keyword 'Linear': expected 'zoh', 'linear', or 'cubic'". This is acceptable — parse-time error messages are CortenForge-internal and don't affect conformance (conformance is about WHAT is rejected, not HOW the error is phrased). But P9 should explicitly state: "Parse-time error messages (invalid keyword, negative interval) are CortenForge-specific — no message-text conformance requirement. Validation BEHAVIOR must match MuJoCo (what's accepted/rejected), not validation MESSAGE TEXT." | "A+ but still broken" stress test (round 5) — conformance boundary | Added error message conformance boundary note to P9 bar. | Rev 7 |
| R49 | P7 | R36 says `sensor.rs:3` doc comment "13 pipeline sensor arrays" must be updated, and states "13 + 5 history = 18, or whatever the actual number is." The correct count is **13 + 4 = 17** — not 18. Only 4 new arrays are pushed in `process_sensors()` (nsample, interp, delay, interval). `sensor_historyadr` is the 5th model field but is computed post-hoc in `build.rs`, NOT pushed in `process_sensors()`. The doc comment at `sensor.rs:3` counts items pushed/computed in `process_sensors()`, which currently = 11 Vec pushes + nsensor + nsensordata = 13. After Spec D: 15 Vec pushes + nsensor + nsensordata = 17. R36's resolution estimate of "18" would itself produce a stale doc comment. | "A+ but still broken" stress test (round 6) — counting precision | Corrected count in R36's stale reference. P7 stale doc site `sensor.rs:3` now says "update to 17." | Rev 8 |
| R50 | P7 | `convert_sensor_type()` (`sensor.rs:468–507`) currently returns `Some(...)` for ALL `MjcfSensorType` variants. Jointlimitfrc and Tendonlimitfrc are now supported (lines 500–501). The `let Some(sensor_type) = ... else { continue }` at line 27 is **dead code** — no sensor type triggers the `continue`. The doc comment at `sensor.rs:28` ("Unsupported type (Jointlimitfrc, Tendonlimitfrc) — skip with log") is stale. The doc comment at `sensor.rs:467` ("Returns None for types not yet implemented") is also stale. R32's push-block atomicity requirement (history attrs in same push block as existing fields) remains correct for forward-compatibility, but the rubric should note that the `continue` path is currently dead code. P7 should list these 2 stale doc sites. | "A+ but still broken" stress test (round 6) — codebase audit | Added `sensor.rs:28` and `sensor.rs:467` to P7 stale documentation sites. Updated R32 with dead-code note. | Rev 8 |
| R51 | P7 | `MjcfSensor` has fluent builder methods (`with_noise`, `with_cutoff`, `with_user` at `types.rs:3195–3214`) used by tests and the Rust API. After Spec D adds 4 new `Option` fields, the API should have 4 new fluent methods: `with_nsample(i32)`, `with_interp(&str)`, `with_delay(f64)`, `with_interval(f64)`. Without these, tests must construct sensors with struct literals and `..Default::default()` — clunky and inconsistent with existing patterns. EGT-10 does NOT list `types.rs:3195–3214` (fluent API) as a modification site. Adding 4 fluent methods is a **12th modification site** (8 existing non-exhaustive + 3 compiler-enforced + 1 new = 12 total modification sites). Note: the factory methods (`jointpos()`, `gyro()`, etc.) use `..Default::default()` and are NOT modification sites — new `Option` fields default to `None` automatically. | "A+ but still broken" stress test (round 6) — API completeness | Added fluent builder methods to EGT-10 Table 1. Updated site count to 12. | Rev 8 |
| R52 | P8 | `MjcfSensor` uses MIXED sentinel patterns: `noise: f64` and `cutoff: f64` (value-sentinel, 0.0 = "not set") alongside proposed `nsample: Option<i32>` and `interp: Option<String>` (Option-sentinel). R26 identified the value-sentinel vs Option-sentinel conflict for defaults. But the mixed pattern also affects testing: existing sensor tests use `noise: 0.1` (direct value) while new tests use `nsample: Some(5)`. P8 should note this mixed pattern as a known inconsistency within `MjcfSensor` and state it's acceptable (the alternatives — changing `noise`/`cutoff` to `Option` or using value-sentinel for history attrs — are both worse). | "A+ but still broken" stress test (round 6) — pattern consistency | Added mixed-sentinel note to P8 bar. | Rev 8 |
| R53 | P2 | R36's resolution says `sensor.rs:3` doc comment should be updated "from 13 to new pipeline array count." But the new count was stated as "13 + 5 history = 18, or whatever the actual number is." The correct count is 17 (R49). The P2 bar references "`sensor.rs:3` doc comment updated from 13 to 17" in P4's code-review AC. The code-review AC should state the exact number (17), not "new pipeline array count." | "A+ but still broken" stress test (round 6) — precision | Updated P4 code-review AC to specify "17." | Rev 8 |
| R54 | P1, P5 | **Negative delay is ACCEPTED by MuJoCo 3.5.0.** Empirical verification: `delay=-0.01, nsample=5` → `delay=[-0.01], historyadr=[0], nhistory=12` (accepted, stored as-is). `delay=-0.01` without nsample → `delay=[-0.01], historyadr=[-1]` (also accepted — no error). The existing validation `delay > 0 && nsample <= 0` only fires for POSITIVE delay without nsample. Negative delay bypasses the check entirely. This matches actuator behavior at `actuator.rs:272` (`if delay > 0.0 && nsample <= 0`). P1 edge case (3) says "delay > 0 without nsample > 0 → error" — correct, but incomplete. It should also state: "negative delay is accepted and stored (same behavior as negative nsample per EGT-5)." P5 needs an edge case for negative delay. | "A+ but still broken" stress test (round 7) — empirical verification | Added EGT-11j (negative delay). Added edge case (18) to P5 bar. Updated P1 bar. | Rev 9 |
| R55 | P2 | P2 pipeline step (5) says "push history attrs in the same push block." But it doesn't specify the unwrap/default logic for each field. Only R35 mentions `sensor.nsample.unwrap_or(0)`. The builder needs unwrap defaults for ALL 4 fields: `nsample: sensor.nsample.unwrap_or(0)`, `interp: sensor.interp.as_deref().map(|s| s.parse::<InterpolationType>()).transpose()?.unwrap_or(InterpolationType::Zoh)`, `delay: sensor.delay.unwrap_or(0.0)`, `interval: (sensor.interval.unwrap_or(0.0), 0.0)`. Without specifying these, an implementer might use wrong defaults (e.g., `unwrap_or(-1)` for nsample, which would change semantics). P2 should list the exact unwrap expression for each field. | "A+ but still broken" stress test (round 7) — implementation precision | Added unwrap defaults to P2 pipeline step (5). | Rev 9 |
| R56 | P10 | P10 arithmetic safety says `ns as i32 * (dim as i32 + 1) + 2`. But `ns` is already `i32` — `ns as i32` is a no-op. The correct expression is `ns * (dim as i32 + 1) + 2` where `ns: i32` (already positive due to `ns > 0` guard) and `dim: usize` (cast to `i32`). The redundant cast is misleading — it suggests `ns` might be a different type. | "A+ but still broken" stress test (round 7) — code precision | Fixed P10 cast sequence to remove redundant `ns as i32`. | Rev 9 |
| R57 | P10 | All 4 P10 ACs include actuators: (a) 1 actuator + 1 sensor, (b) 1 actuator + 1 multi-dim sensor, (c) 2 actuators + 2 sensors, (d) nsample=0 sensor in chain. No AC for a **sensor-only model** (0 actuators, 1+ sensors with history). MuJoCo 3.5.0 verified: sensor-only model with `nsample=3, dim=1` → `sensor_historyadr=[0], nhistory=8, actuator_historyadr.shape=(0,)`. An implementation that hardcodes `sensor_offset = model.actuator_historyadr.last() + contribution` instead of continuing from the actuator loop's offset would crash on empty actuator arrays. EGT-11e implicitly tests this (historyadr=[0,8,20] with no actuators mentioned) but it's not an explicit P10 AC. | "A+ but still broken" stress test (round 7) — empirical verification | Added sensor-only AC (e) to P10 bar. Added EGT-11j verified value. | Rev 9 |
| R58 | P1 | `delay=-0.01` without nsample is ACCEPTED (R54 empirical). But `delay=+0.01` without nsample is REJECTED (EGT-4). This asymmetry means the validation is `delay > 0` (strictly positive), not `delay != 0` (non-zero). The P1 bar says "delay > 0 without nsample > 0 → error" which is correct. But the spec must specify the validation condition as `delay > 0.0` (not `delay.abs() > 0.0` or `delay != 0.0`). The actuator code at `actuator.rs:272` uses `delay > 0.0` — sensor must match exactly. Without this precision, an implementer might "improve" the check to catch negative delay too, breaking MuJoCo conformance. | "A+ but still broken" stress test (round 7) — conformance precision | Updated P9 parity: sensor validation must use `delay > 0.0` (not `!= 0.0`). | Rev 9 |
| R59 | P2, P9 | **P2 validation/interp ordering is wrong.** P2 lists validation at step (3) then interp parsing at step (4). But the actuator code at `actuator.rs:265–278` does interp FIRST (line 265), then delay/nsample validation (line 272). MuJoCo 3.5.0 confirms: when both `interp="invalid"` AND `delay=0.01` without nsample are present, MuJoCo reports the **interp error** ("invalid keyword: 'invalid'"), NOT the delay error. This means MuJoCo validates interp before delay/nsample. P2 must reorder: interp parsing first, then delay/nsample validation, matching actuator pattern. P9 should note this ordering parity. | "A+ but still broken" stress test (round 8) — empirical verification | Reordered P2 pipeline: interp parsing (3), validation (4). | Rev 10 |
| R60 | P2 | P2 step (4) says "parse interp string → InterpolationType enum" but doesn't specify error handling for invalid strings. The actuator code at `actuator.rs:267–268` uses `.parse::<InterpolationType>().map_err(|e| ModelConversionError { message: e })?` — returning a `ModelConversionError` on invalid keyword. Without this, an implementer might use `unwrap_or(InterpolationType::Zoh)`, silently accepting invalid interp keywords like "cubic_spline" or "NEAREST" and defaulting to Zoh. The spec must explicitly state: invalid interp → `Err(ModelConversionError)`, NOT a silent default. | "A+ but still broken" stress test (round 8) — error handling | Added error propagation requirement to P2 interp parsing step. | Rev 10 |
| R61 | P5 | Empty string interp (`interp=""`) is rejected by MuJoCo: "invalid keyword: ''". CortenForge's `FromStr` also rejects it (no match). But this edge case is not in P5's inventory. Edge case (9) says "invalid interp string → error" but doesn't test the empty-string boundary. Edge case (12) tests case sensitivity. The empty string is a distinct case — it's not a mis-cased valid keyword, it's an absent value that was explicitly set to nothing. P5 should list it explicitly. | "A+ but still broken" stress test (round 8) — empirical verification | Added edge case (20) to P5: `interp=""` → error. | Rev 10 |
| R62 | P2 | The interp `FromStr` parse returns `Err(String)`. But `process_sensors()` returns `Result<(), ModelConversionError>`. The error type conversion is needed: `.map_err(|e| ModelConversionError { message: e })`. The actuator code at `actuator.rs:268` does exactly this. P2 should specify the conversion pattern — without it, an implementer might use `.unwrap()` (panic), `?` with wrong error type (compile error), or `map_err(|_| ...)` (discarding the keyword from the message). | "A+ but still broken" stress test (round 8) — error type conversion | Added `.map_err(|e| ModelConversionError { message: e })` pattern to P2 interp step. | Rev 10 |
| R63 | P1, P5 | `interval > 0, delay > 0, nsample = 0` → MuJoCo rejects with "setting delay > 0 without a history buffer" (delay error, not interval). Interval being present does NOT override the nsample requirement for delay. This confirms: interval and delay validations are independent — interval=0.5 is fine without nsample (EGT-11a), but delay > 0 always requires nsample > 0 (EGT-4), regardless of interval. P5 should have an edge case verifying this interaction. | "A+ but still broken" stress test (round 8) — empirical verification | Added edge case (21) to P5: interval+delay without nsample → delay error. | Rev 10 |
| R64 | P4 | Default AC lists 4 of 5 new model fields: `sensor_nsample[0]==0, sensor_interp[0]==Zoh, sensor_delay[0]==0.0, sensor_historyadr[0]==-1` — but omits `sensor_interval[0]==(0.0, 0.0)`. A spec could score A+ on P4 while never verifying that the interval field has the correct default representation `(0.0, 0.0)` (the tuple form per AD-1). EGT-2 shows `sensor_interval = [[0. 0.]]` as the MuJoCo default — all 5 fields must be in the Default AC. | "A+ but still broken" stress test (round 9) — completeness audit | Added `sensor_interval[0]==(0.0, 0.0)` to P4 Default AC. | Rev 11 |
| R65 | P8 | Self-audit checklist at line 688 says "exact edge cases (12 in P1, 19 in P5)" — but P5 was updated to 21 edge cases in round 8 (R61 added (20), R63 added (21)). The self-audit count was not propagated. An implementer reading the self-audit would see "19" and think the P5 bar had 19 edge cases, then find 21 — confusing but not dangerous. Still a consistency violation within the rubric. | "A+ but still broken" stress test (round 9) — internal consistency | Updated self-audit from "19 in P5" to "21 in P5." | Rev 11 |
