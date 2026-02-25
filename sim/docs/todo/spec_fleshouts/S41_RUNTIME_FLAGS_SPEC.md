# §41 — Runtime Flag Wiring (+ DT-61)

**Status:** Not started
**Phase:** Roadmap Phase 2 — Runtime Flag Wiring
**Effort:** M
**Prerequisites:** None
**Subsumes:** DT-61 (DISABLE_GRAVITY flag)

---

## Overview

The MJCF parser reads 21 `<flag>` attributes into `MjcfFlag` boolean fields
(`types.rs:160-209`). The model builder wires **only** `ENABLE_SLEEP`
(`builder/mod.rs:720-722`). `DISABLE_ISLAND` has a runtime constant but is
hardcoded, not parsed from `MjcfFlag.island`.

All other 19 parsed flags are dead — never converted to `Model.disableflags`
/ `Model.enableflags` bitfields, never checked at runtime.

Additionally:
- The parser has a single `passive` field, but MuJoCo 3.x uses separate
  `spring` and `damper` flags.
- Three flags are not parsed at all: `fwdinv`, `invdiscrete`, `autoreset`.

This spec wires all 25 flags (19 disable + 6 enable) end-to-end: constants,
parser, builder, and runtime guards.

---

## MuJoCo Reference

Two `u32` bitfields from `mujoco.h`:

### Disable flags (`mjtDisableBit`, `mjNDISABLE = 19`)

| Constant | Bit | XML attr | Default | Pipeline effect |
|----------|-----|----------|---------|-----------------|
| `mjDSBL_CONSTRAINT` | 0 | `constraint` | enable | Skip constraint assembly + collision detection |
| `mjDSBL_EQUALITY` | 1 | `equality` | enable | Skip equality constraint rows |
| `mjDSBL_FRICTIONLOSS` | 2 | `frictionloss` | enable | Skip friction loss constraint rows |
| `mjDSBL_LIMIT` | 3 | `limit` | enable | Skip joint/tendon limit rows |
| `mjDSBL_CONTACT` | 4 | `contact` | enable | Skip collision detection + contact rows |
| `mjDSBL_SPRING` | 5 | `spring` | enable | Skip passive spring forces |
| `mjDSBL_DAMPER` | 6 | `damper` | enable | Skip passive damping forces |
| `mjDSBL_GRAVITY` | 7 | `gravity` | enable | Zero gravity in `mj_rne()` |
| `mjDSBL_CLAMPCTRL` | 8 | `clampctrl` | enable | Skip clamping ctrl to ctrlrange |
| `mjDSBL_WARMSTART` | 9 | `warmstart` | enable | Zero-initialize solver instead of warmstart |
| `mjDSBL_FILTERPARENT` | 10 | `filterparent` | enable | Disable parent-child collision filtering |
| `mjDSBL_ACTUATION` | 11 | `actuation` | enable | Skip actuator force computation |
| `mjDSBL_REFSAFE` | 12 | `refsafe` | enable | Skip `solref[0] >= 2*timestep` enforcement |
| `mjDSBL_SENSOR` | 13 | `sensor` | enable | Skip all sensor evaluation |
| `mjDSBL_MIDPHASE` | 14 | `midphase` | enable | Skip BVH midphase → brute-force broadphase |
| `mjDSBL_EULERDAMP` | 15 | `eulerdamp` | enable | Skip implicit damping in Euler integrator |
| `mjDSBL_AUTORESET` | 16 | `autoreset` | enable | Skip auto-reset on NaN/divergence |
| `mjDSBL_NATIVECCD` | 17 | `nativeccd` | enable | Fall back to libccd for convex collision |
| `mjDSBL_ISLAND` | 18 | `island` | disable | Skip island discovery → global solve (unique: MJCF default sets bit) |

### Enable flags (`mjtEnableBit`, `mjNENABLE = 6`)

| Constant | Bit | XML attr | Default | Pipeline effect |
|----------|-----|----------|---------|-----------------|
| `mjENBL_OVERRIDE` | 0 | `override` | disable | Enable contact parameter override |
| `mjENBL_ENERGY` | 1 | `energy` | disable | Enable potential + kinetic energy computation |
| `mjENBL_FWDINV` | 2 | `fwdinv` | disable | Enable forward/inverse comparison stats |
| `mjENBL_INVDISCRETE` | 3 | `invdiscrete` | disable | Discrete-time inverse dynamics |
| `mjENBL_MULTICCD` | 4 | `multiccd` | disable | Multi-point CCD for flat surfaces |
| `mjENBL_SLEEP` | 5 | `sleep` | disable | Body sleeping/deactivation |

---

## Specification

### S1. Flag constants — `types/enums.rs`

Add all 17 missing disable constants and 4 missing enable constants next to
the existing `ENABLE_SLEEP` (bit 5) and `DISABLE_ISLAND` (bit 18).

```rust
// ── Disable flags (mjtDisableBit) ──
pub const DISABLE_CONSTRAINT:   u32 = 1 << 0;
pub const DISABLE_EQUALITY:     u32 = 1 << 1;
pub const DISABLE_FRICTIONLOSS: u32 = 1 << 2;
pub const DISABLE_LIMIT:        u32 = 1 << 3;
pub const DISABLE_CONTACT:      u32 = 1 << 4;
pub const DISABLE_SPRING:       u32 = 1 << 5;
pub const DISABLE_DAMPER:       u32 = 1 << 6;
pub const DISABLE_GRAVITY:      u32 = 1 << 7;
pub const DISABLE_CLAMPCTRL:    u32 = 1 << 8;
pub const DISABLE_WARMSTART:    u32 = 1 << 9;
pub const DISABLE_FILTERPARENT: u32 = 1 << 10;
pub const DISABLE_ACTUATION:    u32 = 1 << 11;
pub const DISABLE_REFSAFE:      u32 = 1 << 12;
pub const DISABLE_SENSOR:       u32 = 1 << 13;
pub const DISABLE_MIDPHASE:     u32 = 1 << 14;
pub const DISABLE_EULERDAMP:    u32 = 1 << 15;
pub const DISABLE_AUTORESET:    u32 = 1 << 16;
pub const DISABLE_NATIVECCD:    u32 = 1 << 17;
// DISABLE_ISLAND already exists at 1 << 18
// NOTE on DISABLE_ISLAND semantics:
//   MJCF default: island="disable" → flag.island = false
//   Builder: !flag.island → DISABLE_ISLAND bit SET in disableflags
//   Result: default disableflags has bit 18 SET (island discovery OFF).
//   This is the ONLY disable flag whose default state sets the bit.
//   All other disable flags default to "enable" → bit NOT set.
//   MuJoCo recently moved island from mjtEnableBit to mjtDisableBit,
//   but kept the default behavior (islands off unless explicitly enabled).

// ── Enable flags (mjtEnableBit) ──
pub const ENABLE_OVERRIDE:    u32 = 1 << 0;
pub const ENABLE_ENERGY:      u32 = 1 << 1;
pub const ENABLE_FWDINV:      u32 = 1 << 2;
pub const ENABLE_INVDISCRETE: u32 = 1 << 3;
pub const ENABLE_MULTICCD:    u32 = 1 << 4;
// ENABLE_SLEEP already exists at 1 << 5
```

Also add a `disabled` helper for readable runtime checks:

```rust
/// Returns true if the given disable flag is set on the model.
#[inline]
pub fn disabled(model: &Model, flag: u32) -> bool {
    model.disableflags & flag != 0
}

/// Returns true if the given enable flag is set on the model.
#[inline]
pub fn enabled(model: &Model, flag: u32) -> bool {
    model.enableflags & flag != 0
}
```

### S2. Parser update — `mjcf/src/types.rs` + `parser.rs`

#### S2a. Split `passive` into `spring` + `damper`

Replace `MjcfFlag.passive: bool` with two fields:

```rust
pub spring: bool,   // default true — was `passive`
pub damper: bool,   // default true — was `passive`
```

**Note:** MuJoCo 3.3.6+ removed the `passive` XML attribute entirely,
replacing it with independent `spring` and `damper`. Our parser accepts
`passive` for backward compatibility with older MJCF files (maps to both
`spring` and `damper`), but new models should use the split attributes.
If `passive` is encountered, emit a `tracing::warn!` deprecation notice.

#### S2b. Add missing flag fields

```rust
pub fwdinv: bool,       // default false (enable flag)
pub invdiscrete: bool,  // default false (enable flag)
pub autoreset: bool,    // default true  (disable flag — enabled by default)
```

#### S2c. Update parser

In `parse_flag_attrs()`:

- Replace `flag.passive = parse_flag(e, "passive", ...)` with:
  ```rust
  // MuJoCo 3.3.6+ removed `passive`, replacing it with independent
  // `spring` and `damper`. Accept `passive` for backward compatibility
  // but warn on deprecation.
  let passive_override = get_attribute_opt(e, "passive");
  if let Some(ref v) = passive_override {
      tracing::warn!(
          "MJCF <flag passive=\"{}\"/> is deprecated since MuJoCo 3.3.6; \
           use <flag spring=\"{}\" damper=\"{}\"/> instead",
          v, v, v
      );
      let val = v != "disable";
      flag.spring = val;
      flag.damper = val;
  }
  // Explicit spring/damper override passive (order matters).
  flag.spring = parse_flag(e, "spring", flag.spring);
  flag.damper = parse_flag(e, "damper", flag.damper);
  ```
- Add: `flag.fwdinv = parse_flag(e, "fwdinv", flag.fwdinv);`
- Add: `flag.invdiscrete = parse_flag(e, "invdiscrete", flag.invdiscrete);`
- Add: `flag.autoreset = parse_flag(e, "autoreset", flag.autoreset);`

#### S2d. Fix docstrings

Several `MjcfFlag` field docs are wrong. Fix all to match MuJoCo semantics:

| Field | Current (wrong) | Correct |
|-------|-----------------|---------|
| `clampctrl` | "Coriolis/centrifugal forces" | "Clamping ctrl values to ctrlrange" |
| `frictionloss` | "friction limit constraints" | "Joint/tendon friction loss constraints" |
| `refsafe` | "reference configuration in computation" | "Solref time constant safety floor (solref[0] >= 2*timestep)" |
| `filterparent` | "filtering of contact pairs" | "Parent-child body collision filtering" |
| `eulerdamp` | "Euler angle damping" | "Implicit damping in Euler integrator" |
| `island` | "body sleeping/deactivation" | "Island discovery for constraint solving" |
| `multiccd` | "multiple CCD iterations" | "Multi-point CCD for flat surfaces" |
| `nativeccd` | "native CCD" | "Native CCD (vs libccd fallback for convex collision)" |

### S3. Builder wiring — `mjcf/src/builder/mod.rs`

Replace the single `ENABLE_SLEEP` wiring block in `apply_options()` with a
complete conversion function:

```rust
fn apply_flags(flag: &MjcfFlag, disableflags: &mut u32, enableflags: &mut u32) {
    // Disable flags: field true = feature enabled = bit NOT set.
    if !flag.constraint   { *disableflags |= DISABLE_CONSTRAINT; }
    if !flag.equality     { *disableflags |= DISABLE_EQUALITY; }
    if !flag.frictionloss { *disableflags |= DISABLE_FRICTIONLOSS; }
    if !flag.limit        { *disableflags |= DISABLE_LIMIT; }
    if !flag.contact      { *disableflags |= DISABLE_CONTACT; }
    if !flag.spring       { *disableflags |= DISABLE_SPRING; }
    if !flag.damper       { *disableflags |= DISABLE_DAMPER; }
    if !flag.gravity      { *disableflags |= DISABLE_GRAVITY; }
    if !flag.clampctrl    { *disableflags |= DISABLE_CLAMPCTRL; }
    if !flag.warmstart    { *disableflags |= DISABLE_WARMSTART; }
    if !flag.filterparent { *disableflags |= DISABLE_FILTERPARENT; }
    if !flag.actuation    { *disableflags |= DISABLE_ACTUATION; }
    if !flag.refsafe      { *disableflags |= DISABLE_REFSAFE; }
    if !flag.sensor       { *disableflags |= DISABLE_SENSOR; }
    if !flag.midphase     { *disableflags |= DISABLE_MIDPHASE; }
    if !flag.eulerdamp    { *disableflags |= DISABLE_EULERDAMP; }
    if !flag.autoreset    { *disableflags |= DISABLE_AUTORESET; }
    if !flag.nativeccd    { *disableflags |= DISABLE_NATIVECCD; }
    if !flag.island       { *disableflags |= DISABLE_ISLAND; }

    // Enable flags: field true = feature enabled = bit set.
    if flag.override_contacts { *enableflags |= ENABLE_OVERRIDE; }
    if flag.energy            { *enableflags |= ENABLE_ENERGY; }
    if flag.fwdinv            { *enableflags |= ENABLE_FWDINV; }
    if flag.invdiscrete       { *enableflags |= ENABLE_INVDISCRETE; }
    if flag.multiccd          { *enableflags |= ENABLE_MULTICCD; }
    if flag.sleep             { *enableflags |= ENABLE_SLEEP; }
}
```

This replaces the existing `if option.flag.sleep { ... }` block and now also
wires `DISABLE_ISLAND` from the parsed flag instead of leaving it hardcoded.

### S4. Runtime disable flag guards

Each guard follows this pattern:

```rust
let disabled = |flag: u32| model.disableflags & flag != 0;
```

Or uses the `disabled()` helper from S1. Below is the complete guard table
with exact file/function locations.

#### S4.1 `DISABLE_CONTACT` / `DISABLE_CONSTRAINT` — skip collision detection

**File:** `forward/mod.rs` — `forward_core()`
**Line ~161:** `crate::collision::mj_collision(model, self);`

MuJoCo's `mj_collision()` has an early return when **either** `DISABLE_CONTACT`
or `DISABLE_CONSTRAINT` is set — there is no point detecting contacts if the
solver won't run:

```rust
let skip_collision = disabled(model, DISABLE_CONTACT)
                  || disabled(model, DISABLE_CONSTRAINT);

if !skip_collision {
    crate::collision::mj_collision(model, self);
    if sleep_enabled && crate::island::mj_wake_collision(model, self) {
        crate::island::mj_update_sleep_arrays(model, self);
        crate::collision::mj_collision(model, self);
    }
} else {
    data.ncon = 0; // Explicit zero — don't rely on initialization
}
```

When either flag is set, `data.ncon` is explicitly zeroed at the guard site.

#### S4.2 `DISABLE_GRAVITY` — zero gravity in bias forces (DT-61)

**File:** `dynamics/rne.rs` — `mj_rne()`
**Line ~56-111:** Gravity contribution block.

Use an effective gravity vector:

```rust
let grav = if disabled(model, DISABLE_GRAVITY) {
    Vector3::zeros()
} else {
    model.gravity
};
```

Then replace `model.gravity` with `grav` in the gravity loop (line ~70):
`let gravity_force = -subtree_mass * grav;`

**File:** `dynamics/rne.rs` — `mj_gravcomp()`
**Line ~325:** Add `DISABLE_GRAVITY` and `DISABLE_DAMPER` to the early-return
guard. MuJoCo classifies gravity compensation as a passive damper-like force,
so it is gated on both:

```rust
if model.ngravcomp == 0
    || model.gravity.norm() == 0.0
    || disabled(model, DISABLE_GRAVITY)
    || disabled(model, DISABLE_DAMPER)
{
    return false;
}
```

**File:** `energy.rs` — `mj_energy_pos()`
**Line ~21-27:** Gravitational potential energy loop. Use same effective
gravity:

```rust
let grav = if disabled(model, DISABLE_GRAVITY) {
    Vector3::zeros()
} else {
    model.gravity
};
// ...
potential -= mass * grav.dot(&com);
```

#### S4.3 `DISABLE_CONSTRAINT` — skip constraint assembly + solver

**File:** `constraint/mod.rs` — `mj_fwd_constraint()` or
`mj_fwd_constraint_islands()`

Guard constraint assembly (not just the solver). MuJoCo skips
`mj_makeConstraint()` entirely when this flag is set — no equality, limit,
contact, or friction loss rows are assembled.

When disabled, `efc_force` stays zero, `qacc` = `qacc_smooth`
(unconstrained accelerations only).

**Also:** collision detection is skipped (see S4.1 — `DISABLE_CONSTRAINT`
gates `mj_collision()` too).

#### S4.4 `DISABLE_EQUALITY` — skip equality rows

**File:** `constraint/assembly.rs` — `assemble_unified_constraints()`

Guard the equality constraint assembly phase. When disabled, no equality rows
are added to the constraint Jacobian.

#### S4.5 `DISABLE_FRICTIONLOSS` — skip friction loss rows

**File:** `constraint/assembly.rs` — `assemble_unified_constraints()`

Guard the friction loss row assembly. When disabled, joint friction loss
constraints are not assembled.

#### S4.6 `DISABLE_LIMIT` — skip limit rows

**File:** `constraint/assembly.rs` — `assemble_unified_constraints()`

Guard both joint limit and tendon limit constraint assembly. When disabled,
joints/tendons can exceed their range without constraint forces.

#### S4.7 `DISABLE_SPRING` and `DISABLE_DAMPER` — independent passive force gating

**File:** `forward/passive.rs` — `mj_fwd_passive()`
**Line ~354+:** Joint/tendon/flex passive force visitor.

These are NOT skip-the-loop flags. They gate the force *components*
independently:

```rust
let has_spring = !disabled(model, DISABLE_SPRING);
let has_damper = !disabled(model, DISABLE_DAMPER);

// In passive force computation for each joint:
let spring_force = if has_spring { stiffness * displacement } else { 0.0 };
let damper_force = if has_damper { damping * velocity } else { 0.0 };
qfrc_passive[dof] += spring_force + damper_force;
```

Same pattern applies to:
- Joint springs/dampers
- Tendon springs/dampers
- Flex edge spring/damper forces
- Flex bending forces (spring component)
- Flex vertex damping

**Gravity compensation interaction:** MuJoCo gates `mj_gravcomp()` on
`DISABLE_DAMPER` (not `DISABLE_SPRING`). Gravity compensation is classified
as a passive damper-like force. Add a `DISABLE_DAMPER` guard in addition to
the `DISABLE_GRAVITY` guard from S4.2:

```rust
// mj_gravcomp early return:
if model.ngravcomp == 0
    || model.gravity.norm() == 0.0
    || disabled(model, DISABLE_GRAVITY)
    || disabled(model, DISABLE_DAMPER)   // MuJoCo bundles gravcomp with damper
{
    return false;
}
```

**Fluid force interaction (from §40):** When both `DISABLE_SPRING` AND
`DISABLE_DAMPER` are set, skip `mj_fluid()` entirely. Fluid forces are
passive forces that MuJoCo gates on this combination:

```rust
if !(disabled(model, DISABLE_SPRING) && disabled(model, DISABLE_DAMPER)) {
    mj_fluid(model, data);
}
```

#### S4.8 `DISABLE_ACTUATION` — skip actuator forces

**File:** `forward/mod.rs` — `forward_core()`
**Line ~191:** `actuation::mj_fwd_actuation(model, self);`

Guard the actuation call. When disabled, `qfrc_actuator` stays zero. Ctrl
values are still accepted but have no effect.

Note: `mj_transmission_site` and `mj_actuator_length` still run (they compute
transmission geometry, not forces). Only force generation is gated.

#### S4.9 `DISABLE_CLAMPCTRL` — skip ctrl clamping

**File:** `forward/actuation.rs` — `mj_fwd_actuation()`
**Line ~325:** The ctrl clamp line.

Guard the `.clamp(ctrlrange.0, ctrlrange.1)` call. When disabled, ctrl values
pass through unclamped.

#### S4.10 `DISABLE_SENSOR` — skip sensor evaluation

**File:** `forward/mod.rs` — `forward_core()`
**Lines ~178-179, 186-187, 214-216:** The four sensor calls.

Replace the `compute_sensors` guard with `compute_sensors && !disabled(...)`:

```rust
let sensors_active = compute_sensors && !disabled(model, DISABLE_SENSOR);
// ...
if sensors_active {
    crate::sensor::mj_sensor_pos(model, self);
}
```

#### S4.11 `DISABLE_WARMSTART` — zero-initialize solver

**File:** `constraint/solver/pgs.rs` — `pgs_solve_unified()`
**Line ~84-118:** Warmstart block.

Guard the warmstart copy. When disabled, solver starts from zero forces
instead of previous solution:

```rust
if disabled(model, DISABLE_WARMSTART) {
    // Zero-initialize: skip warmstart
    efc_force.fill(0.0);
} else {
    // Existing warmstart logic
}
```

Same pattern in Newton and CG solvers if they have warmstart blocks.

#### S4.12 `DISABLE_FILTERPARENT` — disable parent-child filtering

**File:** `collision/mod.rs` — `check_collision_affinity()`
**Line ~76-81:** Parent-child check.

When `DISABLE_FILTERPARENT` is set, skip the parent-child exclusion check
(allow parent-child geom pairs to collide):

```rust
if !disabled(model, DISABLE_FILTERPARENT) {
    // existing parent-child filter logic
    if is_parent_child { return false; }
}
```

#### S4.13 `DISABLE_MIDPHASE` — brute-force broadphase

**File:** `collision/mod.rs` — `mj_collision()`

When set, bypass BVH midphase and use brute-force all-pairs broadphase.
Check if the current collision pipeline already has a brute-force fallback;
if not, this flag is a no-op until BVH midphase is the default path.

#### S4.14 `DISABLE_EULERDAMP` — skip implicit damping in Euler

**File:** `forward/acceleration.rs` or `integrate/implicit.rs`

MuJoCo's `mj_Euler()` applies implicit damping only when **both** conditions
hold: eulerdamp is not disabled AND damper is not disabled. The compound
guard is:

```rust
let use_implicit_damp = !disabled(model, DISABLE_EULERDAMP)
                     && !disabled(model, DISABLE_DAMPER);

if use_implicit_damp {
    // Apply implicit damping integration (mass matrix factorization path)
}
```

When either flag is set, Euler uses explicit damping only (faster but less
stable at large timesteps).

#### S4.15 `DISABLE_REFSAFE` — skip solref time-constant clamping

**File:** `constraint/impedance.rs` — `compute_kbip()`

MuJoCo enforces `solref[0] >= 2 * timestep` when `REFSAFE` is enabled
(default). Our codebase does not currently implement this clamp. To implement:

```rust
let solref_0 = if !disabled(model, DISABLE_REFSAFE) && solref[0] > 0.0 {
    solref[0].max(2.0 * model.timestep)
} else {
    solref[0]
};
```

This requires threading `model` (or `timestep`) into `compute_kbip()`. If
the function signature cannot accept `model`, pass `timestep` as a parameter
and `None` when refsafe is disabled.

#### S4.16 `DISABLE_AUTORESET` — skip NaN auto-reset

Not currently implemented in our pipeline. Define the constant for
forward-compatibility. The runtime guard will be added when auto-reset
on divergence is implemented (currently no auto-reset exists).

#### S4.17 `DISABLE_NATIVECCD` — CCD fallback

Not currently relevant — we don't have native CCD vs libccd dispatch.
Define the constant for forward-compatibility. The runtime guard will be
added when native CCD is implemented.

### S5. Enable flag guards

#### S5.1 `ENABLE_ENERGY` — gate energy computation

**File:** `forward/mod.rs` — `forward_core()`
**Lines ~181, 194:** `mj_energy_pos` and `mj_energy_vel` calls.

Guard both energy calls. When `ENABLE_ENERGY` is not set, MuJoCo
**explicitly zeros** the energy fields (not left stale):

```rust
if enabled(model, ENABLE_ENERGY) {
    crate::energy::mj_energy_pos(model, self);
} else {
    self.energy_potential = 0.0;
}
// ...
if enabled(model, ENABLE_ENERGY) {
    crate::energy::mj_energy_vel(model, self);
} else {
    self.energy_kinetic = 0.0;
}
```

Currently both run unconditionally. MuJoCo only computes energy when
`ENABLE_ENERGY` is set; otherwise both fields are deterministically 0.0.

#### S5.2 `ENABLE_OVERRIDE` — contact parameter override

**File:** `collision/mod.rs` — contact parameter combination.

When enabled, override contact solver parameters (solref, solimp, margin,
gap) with global override values from `Model`. This requires override fields
to exist on Model (from `<option><override .../>`). If those fields are not
yet parsed, this flag is a no-op until they are.

#### S5.3 `ENABLE_FWDINV` — forward/inverse comparison

Not currently implementable — requires `mj_inverse()` (§52). Define the
constant; guard will be wired when inverse dynamics is implemented.

#### S5.4 `ENABLE_INVDISCRETE` — discrete inverse dynamics

Same as S5.3 — depends on §52.

#### S5.5 `ENABLE_MULTICCD` — multi-point CCD

Not currently implementable — depends on CCD implementation (§50). Define
the constant for forward-compatibility.

#### S5.6 `ENABLE_SLEEP` — already wired

No changes needed. Already functional.

### S6. Fluid sleep filtering (from §40)

**File:** `forward/passive.rs` — `mj_fluid()`
**Line ~303-316:** The per-body dispatch loop.

Replace `for body_id in 1..model.nbody` with sleep-filtered iteration:

```rust
let bodies = if sleep_enabled && data.nbody_awake < model.nbody {
    &data.body_awake_ind[..data.nbody_awake]
} else {
    // all bodies except world (0)
    // use 1..model.nbody range
};
```

This is a performance optimization — sleeping bodies have zero velocity so
fluid forces are zero — but it matches MuJoCo semantics.

---

## Implementation Order

1. **S1** — Define all flag constants in `enums.rs`.
2. **S2** — Parser changes: split `passive`, add missing fields, fix docs.
3. **S3** — Builder wiring: `apply_flags()` replaces single sleep-only block.
4. **S4.1-S4.7** — High-impact runtime guards: contact, gravity, constraint,
   equality, limit, frictionloss, spring/damper. These affect correctness.
5. **S4.8-S4.12** — Medium-impact guards: actuation, clampctrl, sensor,
   warmstart, filterparent.
6. **S4.13-S4.17** — Forward-compatibility stubs: midphase, eulerdamp,
   refsafe, autoreset, nativeccd.
7. **S5** — Enable flag guards: energy, override (others are stubs).
8. **S6** — Fluid sleep filtering.

Steps 1-3 form a single commit (infrastructure). Steps 4-8 can each be a
commit.

---

## Acceptance Criteria

### AC1 — Contact disable
`<flag contact="disable"/>` produces zero contacts. No collision detection
runs. `data.ncon == 0`. Other forces (gravity, springs, actuators) still
apply. Test: free-falling body with floor — body falls through floor.

### AC2 — Gravity disable (DT-61)
`<flag gravity="disable"/>` zeros gravitational force. A free body with only
gravity remains stationary (zero qacc). Other forces (springs, actuators)
still apply. Gravitational potential energy is zero. `mj_gravcomp()` returns
false.

### AC3 — Limit disable
`<flag limit="disable"/>` allows joints to exceed their range without
constraint forces. A hinge with `range="0 90"` rotates past 90 degrees
freely.

### AC4 — Equality disable
`<flag equality="disable"/>` deactivates all equality constraints (weld,
joint, tendon). Bodies linked by weld constraints separate freely.

### AC5 — Spring/damper independence
`<flag spring="disable" damper="enable"/>` disables spring stiffness but keeps
damping. A joint with `stiffness=100 damping=10` produces zero spring force
but non-zero damping force. Conversely, `<flag spring="enable"
damper="disable"/>` keeps spring but drops damping.

### AC6 — Actuation disable
`<flag actuation="disable"/>` zeros all actuator forces. `ctrl` values have
no effect on `qfrc_actuator`. Transmission geometry still computed.

### AC7 — Sensor disable
`<flag sensor="disable"/>` skips sensor evaluation. `sensordata` array is
not updated (retains previous values or zeros).

### AC8 — Warmstart disable
`<flag warmstart="disable"/>` starts solver from zero. Solver converges
(may need more iterations). Verify `efc_force` is not initialized from
`qacc_warmstart`.

### AC9 — Constraint disable
`<flag constraint="disable"/>` skips constraint assembly AND collision
detection. No equality, limit, contact, or friction loss constraints apply.
`data.ncon == 0`. `qacc == qacc_smooth`.

### AC10 — Fluid force gating
When both `DISABLE_SPRING` and `DISABLE_DAMPER` are set, `mj_fluid()` is
skipped. `qfrc_fluid` is zero. When only one is set, fluid forces still
compute.

### AC11 — Default bitfields
An unmodified `<option/>` with no `<flag>` produces
`disableflags == DISABLE_ISLAND` (bit 18 set, all others clear) and
`enableflags == 0`. The island bit is set because the MJCF default is
`island="disable"` → `flag.island = false` → bit set. All other flags
have their bits clear. Matches MuJoCo defaults.

### AC12 — Backward compat
`<flag passive="disable"/>` sets both `DISABLE_SPRING` and `DISABLE_DAMPER`.
Explicit `<flag passive="disable" spring="enable"/>` re-enables spring only.

### AC13 — Energy gating
Energy fields `energy_potential` and `energy_kinetic` are only computed when
`ENABLE_ENERGY` is set. When not set, both are **explicitly zeroed** to 0.0
(not left stale). This matches MuJoCo's deterministic zeroing behavior.

### AC14 — Fluid sleep filtering
`mj_fluid()` skips sleeping bodies. A scene with one sleeping and one awake
body computes fluid forces only for the awake body.

### AC15 — Filterparent disable
`<flag filterparent="disable"/>` allows parent-child geom pairs to collide.
A body with a child body that overlaps its parent geom produces a contact.

### AC16 — Frictionloss disable
`<flag frictionloss="disable"/>` removes joint friction loss constraints.
A joint with `frictionloss="10"` rotates freely without friction.

### AC17 — Refsafe disable
`<flag refsafe="disable"/>` allows `solref[0]` below `2*timestep`. Verify
no clamping occurs.

### AC18 — MuJoCo conformance
For each wired disable flag, compare 10-step trajectory against MuJoCo 3.4.0
with only that flag disabled. Tolerance: `1e-10` per `qacc` component. Flags
should produce exact gating.

### AC19 — Clampctrl disable
`<flag clampctrl="disable"/>` allows ctrl values outside ctrlrange. An
actuator with `ctrlrange="0 1"` accepts ctrl=2.0 without clamping.
`qfrc_actuator` reflects the unclamped ctrl value.

### AC20 — Eulerdamp disable
`<flag eulerdamp="disable"/>` skips implicit damping in Euler integrator.
Also verified: when `<flag damper="disable"/>` is set, implicit damping is
skipped regardless of `eulerdamp` setting (compound guard).

### AC21 — Gravity compensation + damper interaction
`<flag damper="disable"/>` also disables gravity compensation forces
(`qfrc_gravcomp`). Verify: a body with `gravcomp="1"` produces zero
gravity compensation force when damper is disabled, even if gravity is
enabled.

---

## Files

| File | Change |
|------|--------|
| `sim/L0/core/src/types/enums.rs` | S1: 17 disable + 4 enable constants, `disabled()`/`enabled()` helpers |
| `sim/L0/mjcf/src/types.rs` | S2: Split `passive` → `spring` + `damper`, add `fwdinv`/`invdiscrete`/`autoreset`, fix docstrings |
| `sim/L0/mjcf/src/parser.rs` | S2: Parse `spring`, `damper`, `passive` (compat), `fwdinv`, `invdiscrete`, `autoreset` |
| `sim/L0/mjcf/src/builder/mod.rs` | S3: `apply_flags()` replacing single sleep-only block |
| `sim/L0/core/src/forward/mod.rs` | S4.1, S4.8, S4.10, S5.1: Contact+constraint collision guard, actuation, sensor, energy guards |
| `sim/L0/core/src/dynamics/rne.rs` | S4.2: Gravity guard in `mj_rne()` + `mj_gravcomp()` |
| `sim/L0/core/src/energy.rs` | S4.2: Gravity guard in `mj_energy_pos()` |
| `sim/L0/core/src/constraint/mod.rs` | S4.3: Constraint solver guard |
| `sim/L0/core/src/constraint/assembly.rs` | S4.4-S4.6: Equality, frictionloss, limit row guards |
| `sim/L0/core/src/forward/passive.rs` | S4.7, S6: Spring/damper independence, fluid gating, fluid sleep filter |
| `sim/L0/core/src/forward/actuation.rs` | S4.9: Clampctrl guard |
| `sim/L0/core/src/constraint/solver/pgs.rs` | S4.11: Warmstart guard |
| `sim/L0/core/src/collision/mod.rs` | S4.12: Filterparent guard |
| `sim/L0/core/src/constraint/impedance.rs` | S4.15: Refsafe guard |

---

## Risk

- **Breaking:** Splitting `MjcfFlag.passive` into `spring`/`damper` breaks
  any code that reads `flag.passive`. Grep for `flag.passive` and update all
  call sites.
- **Energy regression:** Gating energy behind `ENABLE_ENERGY` means
  `data.energy_potential` and `data.energy_kinetic` will be 0.0 by default
  (matching MuJoCo, but differs from current always-compute behavior). Any
  test that asserts on energy values without setting `ENABLE_ENERGY` will
  fail.
- **Island default:** The default `disableflags` is `DISABLE_ISLAND` (not 0)
  because `island="disable"` is the MJCF default. Verify this matches the
  existing hardcoded `DISABLE_ISLAND` behavior — no behavioral change expected.
- **Low risk:** Most guards are additive (new early-return paths). All disable
  flags except island default to "off" (bit not set), preserving current
  behavior.

---

## Verification

After implementation, run sim domain tests:

```
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics \
  -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf \
  -p sim-types -p sim-simd
```

Expected: 2,007+ passed (baseline), 0 failed, plus new per-flag tests.
