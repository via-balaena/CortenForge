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
- The parser has a single `passive` field, but MuJoCo 3.4.0+ uses separate
  `spring` and `damper` flags (the `passive` attribute was removed).
- Three flags are not parsed at all: `fwdinv`, `invdiscrete`, `autoreset`.

This spec wires all 25 flags (19 disable + 6 enable) end-to-end: constants,
parser, builder, and runtime guards.

---

## MuJoCo Reference

**Verified against:** MuJoCo 3.4.0 and 3.5.0 source (`mjmodel.h`,
`engine_forward.c`, `engine_passive.c`, `engine_init.c`). Enums are
identical in both versions — no flags added between 3.4.0 and 3.5.0.

**Version history** (relevant for backward compatibility):

| Version | `mjNDISABLE` | `mjNENABLE` | Key changes |
|---------|-------------|-------------|-------------|
| 3.3.2–3.3.3 | 17 | 6 | Single `mjDSBL_PASSIVE` (bit 5). `mjENBL_ISLAND` (enable, bit 5). No `SLEEP`. |
| 3.3.7 | 19 | 5 | `PASSIVE` split → `SPRING` (bit 5) + `DAMPER` (bit 6). `ISLAND` moved from enable to disable (bit 18). All disable bits 7–17 shifted +1. `SLEEP` not yet added. |
| 3.4.0 | 19 | 6 | `mjENBL_SLEEP` added (enable, bit 5) — filled hole left by `ISLAND`'s departure. |
| 3.5.0 | 19 | 6 | Identical to 3.4.0. |

This history motivates the backward-compatibility handling of `passive`
in S2a — `passive` was a real MJCF attribute in MuJoCo ≤ 3.3.3.

Two `u32` bitfields from `mjmodel.h`:

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
| `mjDSBL_ISLAND` | 18 | `island` | enable | Skip island discovery → global solve |

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
// NOTE: Island follows the same default pattern as all other disable flags.
//   MJCF default: island="enable" → flag.island = true → bit NOT set.
//   Default disableflags = 0 (all bits clear). Matches MuJoCo's
//   mj_defaultOption() which sets opt->disableflags = 0.

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

**Note:** MuJoCo 3.4.0+ removed the `passive` XML attribute entirely,
replacing it with independent `spring` and `damper`. MuJoCo itself provides
no backward compatibility — `passive` is silently ignored (unrecognized
attributes are skipped). As a **CortenForge extension**, our parser accepts
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
  // MuJoCo 3.4.0+ removed `passive`, replacing it with independent
  // `spring` and `damper`. Accept `passive` for backward compatibility
  // but warn on deprecation.
  let passive_override = get_attribute_opt(e, "passive");
  if let Some(ref v) = passive_override {
      tracing::warn!(
          "MJCF <flag passive=\"{}\"/> is deprecated since MuJoCo 3.4.0; \
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
| `island` | "body sleeping/deactivation" | "Island discovery for parallel constraint solving" |
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

#### S4.1 `DISABLE_CONTACT` / `DISABLE_CONSTRAINT` — skip collision + contact rows

**Site 1 — `mj_collision()` collision detection:**
**File:** `collision/mod.rs`

The guard is **inside** `mj_collision()` itself (matching MuJoCo's
architecture — `mj_collision()` is always called, the function handles
its own early return):

```rust
pub fn mj_collision(model: &Model, data: &mut Data) {
    // Unconditional initialization — runs regardless of disable flags.
    // MuJoCo resets ncon, arena, and EFC arrays before checking flags.
    data.ncon = 0;
    data.contacts.clear();
    // If CortenForge has arena/EFC state equivalent to MuJoCo's
    // resetArena(d) + mj_clearEfc(d), reset it here unconditionally.

    if disabled(model, DISABLE_CONTACT)
        || disabled(model, DISABLE_CONSTRAINT)
        || model.nbodyflex() < 2  // no collision possible with < 2 bodies/flexes
    {
        return; // No collision detection needed
    }

    // ... collision detection logic ...
}
```

MuJoCo unconditionally resets `ncon`, calls `resetArena(d)`, and calls
`mj_clearEfc(d)` at the top of `mj_collision()`, **before** checking any
disable flags. Only after this cleanup does it check the three-condition
OR guard and potentially return early. The call site in `forward_core()`
remains unconditional — `mj_collision()` is always invoked.

Note: `nbodyflex` is `m->nbody + m->nflex` in MuJoCo (computed locally
inside the function).

**Site 2 — `mj_instantiateContact()` constraint assembly:**
**File:** `constraint/assembly.rs`

Even if contacts exist from a previous step (defense-in-depth), contact
constraint instantiation is independently gated:

```rust
if disabled(model, DISABLE_CONTACT) || data.ncon == 0 || model.nv == 0 {
    return;
}
```

This is a redundant guard — `mj_collision()` already zeroes `ncon` — but
matches MuJoCo's layered gating architecture.

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
**Line ~325:** Add `DISABLE_GRAVITY` to the early-return guard. MuJoCo's
`mj_gravcomp()` checks only `!ngravcomp`, `DISABLE_GRAVITY`, and zero gravity:

```rust
if model.ngravcomp == 0
    || model.gravity.norm() == 0.0
    || disabled(model, DISABLE_GRAVITY)
{
    return false;
}
```

Note: `mj_gravcomp()` does NOT check `DISABLE_DAMPER` directly. However,
gravcomp is indirectly skipped when both `DISABLE_SPRING` AND `DISABLE_DAMPER`
are set, because `mj_passive()` returns early before calling `mj_gravcomp()`.
See S4.7 for the `mj_passive()` top-level guard.

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

**File:** `forward/actuation.rs` — `mj_fwd_actuation()`

Actuator-level gravity compensation. MuJoCo adds `qfrc_gravcomp` to
`qfrc_actuator` for joints with `jnt_actgravcomp` set, gated on
`DISABLE_GRAVITY`. This is separate from the passive-level `mj_gravcomp()`:

```rust
if model.ngravcomp > 0
    && !disabled(model, DISABLE_GRAVITY)
    && model.gravity.norm() > 0.0
{
    for jnt in 0..model.njnt {
        if !model.jnt_actgravcomp[jnt] { continue; }
        let dofadr = model.jnt_dofadr[jnt];
        let dofnum = jnt_dofnum(model.jnt_type[jnt]);
        for i in 0..dofnum {
            data.qfrc_actuator[dofadr + i] += data.qfrc_gravcomp[dofadr + i];
        }
    }
}
```

#### S4.3 `DISABLE_CONSTRAINT` — skip constraint assembly + solver

**File:** `constraint/mod.rs` — `mj_fwd_constraint()` or
`mj_fwd_constraint_islands()`

Guard constraint assembly (not just the solver). MuJoCo skips
`mj_makeConstraint()` entirely when this flag is set — no equality, limit,
contact, or friction loss rows are assembled.

MuJoCo **unconditionally zeroes** `qfrc_constraint` at the top of
`mj_fwdConstraint()`, before checking whether any constraint rows exist:

```rust
data.qfrc_constraint[..model.nv].fill(0.0);

if data.nefc == 0 {
    // No constraints assembled — qacc = qacc_smooth, return.
    data.qacc.copy_from(&data.qacc_smooth);
    return;
}
```

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

**File:** `forward/passive.rs` — `mj_fwd_passive()` / `mj_passive()`

MuJoCo's passive force pipeline has a **hierarchical guard structure**:

##### S4.7a Top-level `mj_passive()` early return

MuJoCo **unconditionally zeros** all passive force vectors (`qfrc_spring`,
`qfrc_damper`, `qfrc_gravcomp`, `qfrc_fluid`, `qfrc_passive`) for awake
DOFs at the top of `mj_passive()`, **before** the early-return guard. This
zeroing happens regardless of whether the function continues or returns
early.

After zeroing, when **both** `DISABLE_SPRING` AND `DISABLE_DAMPER` are set,
`mj_passive()` returns immediately — skipping ALL passive sub-functions:

```rust
// Zero all passive force vectors unconditionally.
// MuJoCo uses indexed zeroing (mju_zeroInd) for awake DOFs only when
// sleep filtering is active, and full zeroing (mju_zero) otherwise.
let sleep_filter = enabled(model, ENABLE_SLEEP)
    && data.nbody_awake < model.nbody;

if sleep_filter {
    // Zero only awake DOFs (indexed by dof_awake_ind).
    for &dof in &data.dof_awake_ind[..data.ndof_awake] {
        data.qfrc_spring[dof] = 0.0;
        data.qfrc_damper[dof] = 0.0;
        data.qfrc_gravcomp[dof] = 0.0;
        data.qfrc_fluid[dof] = 0.0;
        data.qfrc_passive[dof] = 0.0;
    }
} else {
    data.qfrc_spring[..model.nv].fill(0.0);
    data.qfrc_damper[..model.nv].fill(0.0);
    data.qfrc_gravcomp[..model.nv].fill(0.0);
    data.qfrc_fluid[..model.nv].fill(0.0);
    data.qfrc_passive[..model.nv].fill(0.0);
}

if disabled(model, DISABLE_SPRING) && disabled(model, DISABLE_DAMPER) {
    // Early return — skips mj_springdamper, mj_gravcomp, mj_fluid,
    // mj_contactPassive. Force vectors are already zeroed above.
    return;
}
```

This is the ONLY mechanism that gates `mj_fluid()` and `mj_gravcomp()` on
spring/damper flags — neither function checks these flags internally.

##### S4.7b Component-level guards in `mj_springdamper()`

Within `mj_springdamper()`, spring and damper forces are gated independently:

```rust
let has_spring = !disabled(model, DISABLE_SPRING);
let has_damper = !disabled(model, DISABLE_DAMPER);

// In passive force computation for each joint:
let spring_force = if has_spring { stiffness * displacement } else { 0.0 };
let damper_force = if has_damper { damping * velocity } else { 0.0 };
qfrc_passive[dof] += spring_force + damper_force;
```

**MuJoCo implementation detail:** MuJoCo uses two gating patterns
interchangeably:
1. **Conditional blocks:** `if (has_spring) { ... }` for joint/DOF springs
2. **Multiplication-based zeroing:** `stiffness = m->tendon_stiffness[i] * has_spring`
   — multiplying the coefficient by the boolean (0 or 1) to zero it when
   disabled. This is equivalent but avoids branching.

Either approach is acceptable for CortenForge — the conditional block
pattern is clearer.

Same pattern applies to:
- Joint springs/dampers
- Tendon springs/dampers
- Flex edge spring/damper forces
- Flex bending forces (spring component)
- Flex vertex damping

##### S4.7c Sub-function guards

The following sub-functions have their own guards (independent of spring/damper):

- **`mj_gravcomp()`**: Gated on `DISABLE_GRAVITY` and zero gravity only
  (see S4.2). NOT gated on `DISABLE_DAMPER` directly.
- **`mj_fluid()`**: Gated on `model.opt.viscosity == 0 && model.opt.density == 0`
  only (global option fields, not per-body). NOT gated on spring/damper flags
  directly. Skipped only via the top-level `mj_passive()` early return (S4.7a).
- **`mj_contactPassive()`**: Gated on `DISABLE_CONTACT` (see S4.7d).

##### S4.7d `DISABLE_CONTACT` gates `mj_contactPassive()`

**File:** `forward/passive.rs` — `mj_contact_passive()`

MuJoCo's `mj_contactPassive()` has its own early-return guard:

```rust
if disabled(model, DISABLE_CONTACT) || data.ncon == 0 || model.nv == 0 {
    return;
}
```

This means `DISABLE_CONTACT` gates not only collision detection (S4.1) but
also passive contact forces (e.g., viscous contact damping).

##### S4.7e Aggregation into `qfrc_passive`

After all sub-functions run, `mj_passive()` aggregates the individual
force vectors into the final `qfrc_passive`:

```rust
// qfrc_passive = qfrc_spring + qfrc_damper
//              + (if gravcomp ran) qfrc_gravcomp
//              + (if fluid ran)    qfrc_fluid
for dof in 0..model.nv {
    data.qfrc_passive[dof] = data.qfrc_spring[dof] + data.qfrc_damper[dof];
}
if has_gravcomp {
    for dof in 0..model.nv {
        data.qfrc_passive[dof] += data.qfrc_gravcomp[dof];
    }
}
if has_fluid {
    for dof in 0..model.nv {
        data.qfrc_passive[dof] += data.qfrc_fluid[dof];
    }
}
```

MuJoCo's `mj_springdamper()` writes directly into both `qfrc_spring` and
`qfrc_damper`, while `mj_gravcomp()` and `mj_fluid()` return booleans
indicating whether they produced non-zero forces. The aggregation uses
these booleans to skip unnecessary additions. When sleep filtering is
active, aggregation also uses indexed iteration over awake DOFs only.

After aggregation, MuJoCo invokes the user callback (`mjcb_passive`)
and plugin dispatch (`mjPLUGIN_PASSIVE`), both of which can modify
`qfrc_passive` further.

#### S4.8 `DISABLE_ACTUATION` — skip actuator forces

MuJoCo gates actuation at **four** locations:

**Site 1 — `mj_fwd_actuation()`:**
**File:** `forward/actuation.rs`

Guard the actuation force computation. MuJoCo unconditionally zeroes the
per-actuator force vector (`actuator_force`) at the top of the function
before checking disable flags. Then, when disabled, it zeroes the
joint-space force vector (`qfrc_actuator`) and returns early:

```rust
// Unconditional — zero per-actuator forces regardless of disable flags.
data.actuator_force[..model.nu].fill(0.0);

if model.nu == 0 || disabled(model, DISABLE_ACTUATION) {
    data.qfrc_actuator.fill(0.0);
    return;
}
```

If CortenForge has a separate `actuator_force` array (distinct from
`qfrc_actuator`), it must be zeroed unconditionally before the guard.

**Site 2 — `mj_fwd_velocity()` actuator velocity:**
**File:** `forward/velocity.rs` (or equivalent)

When actuation is disabled, zero `actuator_velocity` instead of computing it:

```rust
if !disabled(model, DISABLE_ACTUATION) {
    // compute actuator_velocity = moment^T * qvel
} else {
    data.actuator_velocity.fill(0.0);
}
```

**Site 3 — `mj_advance()` activation state:**
**File:** `forward/mod.rs` or `integrate/mod.rs`

When actuation is disabled, skip activation state advancement:

```rust
if model.na > 0 && !disabled(model, DISABLE_ACTUATION) {
    // advance activation states (act += act_dot * dt)
}
```

**Site 4 — Control callback:**
If CortenForge has (or will have) a control callback mechanism, gate it on
`DISABLE_ACTUATION`:

```rust
if let Some(cb) = control_callback {
    if !disabled(model, DISABLE_ACTUATION) {
        cb(model, data);
    }
}
```

Note: `mj_transmission_site` and `mj_actuator_length` still run (they compute
transmission geometry, not forces). Only force generation and activation are
gated.

#### S4.9 `DISABLE_CLAMPCTRL` — skip ctrl clamping

**File:** `forward/actuation.rs` — `mj_fwd_actuation()`
**Line ~325:** The ctrl clamp line.

Guard the `.clamp(ctrlrange.0, ctrlrange.1)` call. When disabled, ctrl values
pass through unclamped.

#### S4.10 `DISABLE_SENSOR` — skip sensor evaluation

**File:** `sensor/mod.rs` — `mj_sensor_pos()`, `mj_sensor_vel()`,
`mj_sensor_acc()`

The guard is **inside** each sensor function (matching MuJoCo's architecture
where each `mj_sensorXxx()` checks the flag independently):

```rust
pub fn mj_sensor_pos(model: &Model, data: &mut Data) {
    if disabled(model, DISABLE_SENSOR) {
        return;
    }
    // ... sensor evaluation ...
}
```

The call sites in `forward_core()` remain unchanged — they still use the
existing `compute_sensors` / `skipsensor` logic. The `DISABLE_SENSOR` flag
is an additional independent gate inside the sensor functions themselves.

#### S4.11 `DISABLE_WARMSTART` — zero-initialize solver

**File:** `constraint/mod.rs` — called from `mj_fwd_constraint()`

MuJoCo implements warmstart as a standalone `warmstart()` function called
from `mj_fwdConstraint()`, not inside individual solvers. Match this:

```rust
fn warmstart(model: &Model, data: &mut Data) {
    if disabled(model, DISABLE_WARMSTART) {
        // Cold start: unconstrained accelerations, zero constraint forces
        data.qacc.copy_from(&data.qacc_smooth);
        data.efc_force.fill(0.0);
        return;
    }

    // Smart warmstart: copy qacc_warmstart → qacc, evaluate cost via
    // mj_constraintUpdate(), then compare against qacc_smooth cost.
    // MuJoCo falls back to qacc_smooth if warmstart produces higher cost.
    //
    // Solver-specific logic:
    //   PGS:       Compare force-based warmstart cost. If > 0, zero
    //              efc_force and qfrc_constraint instead of warmstarting.
    //   Newton/CG: Compare Gauss cost (acceleration-based). If warmstart
    //              cost > smooth cost, fall back to qacc_smooth.
    //
    // Island handling: unconstrained DOFs (index >= nidof) always get
    // qacc_smooth regardless of warmstart outcome.
}
```

Key detail: when disabled, `qacc` is set to `qacc_smooth` (unconstrained
accelerations) AND `efc_force` is zeroed. Both assignments are required.

When enabled, the warmstart path is NOT a simple copy — it evaluates the
cost of `qacc_warmstart` via `mj_constraintUpdate()` and falls back to
`qacc_smooth` if warmstart produces a worse starting point. This "smart
warmstart" behavior should be matched for full conformance.

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

When set, bypass BVH midphase and use brute-force all-pairs broadphase:

```rust
if !disabled(model, DISABLE_MIDPHASE) && bvh1 >= 0 && bvh2 >= 0 {
    mj_collide_tree(model, data, ...);
} else {
    // brute-force all-pairs narrowphase
}
```

Also affects flex self-collision midphase and BVH updates on flex objects.
Check if the current collision pipeline already has a brute-force fallback;
if not, this flag is a no-op until BVH midphase is the default path.

#### S4.13b `DISABLE_ISLAND` — solver constraint

Note: island-based solving has additional constraints beyond the flag:

```rust
let islands_supported = !disabled(model, DISABLE_ISLAND)
    && data.nisland > 0
    && model.opt.noslip_iterations == 0
    && matches!(model.opt.solver, Solver::CG | Solver::Newton);
```

Island solving only works with CG or Newton solvers and requires no noslip
iterations. PGS always uses global solving regardless of the island flag.

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

**Note on lazy evaluation:** MuJoCo wraps the `ENABLE_ENERGY` check inside
`if (!d->flg_energypos)` / `if (!d->flg_energyvel)` lazy-evaluation flags
to avoid redundant recomputation when energy was already computed earlier in
the same step (e.g., by a plugin or sensor). Our pipeline does not currently
have this lazy-eval mechanism, so the guard simplifies to a direct
`ENABLE_ENERGY` check. If lazy evaluation is added later, the guard should
be updated to match.

#### S5.2 `ENABLE_OVERRIDE` — contact parameter override

**File:** `collision/mod.rs` — broadphase margin + contact parameter combination.

When enabled, affects **two** locations:

1. **Broadphase margin:** During collision pair filtering, the margin used
   for broadphase expansion switches from per-geom to global override:
   ```rust
   let margin = if enabled(model, ENABLE_OVERRIDE) {
       0.5 * model.opt.o_margin
   } else {
       model.geom_margin[geom_id]
   };
   ```

2. **Contact parameters:** Override contact solver parameters (solref,
   solimp, margin, gap, friction) with global override values from `Model`
   (from `<option><override .../>`).

If override fields are not yet parsed, this flag is a no-op until they are.

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

### S7. Out of scope: `disableactuator` per-group bitmask

MuJoCo has a third bitfield `opt.disableactuator` (an `int`, 31 usable
bits) that disables actuators by group ID. It is checked via
`mj_actuatorDisabled()` inside `mj_fwdActuation()`:

```c
int mj_actuatorDisabled(const mjModel* m, int i) {
    int group = m->actuator_group[i];
    if (group < 0 || group > 30) return 0;
    return m->opt.disableactuator & (1 << group) ? 1 : 0;
}
```

This mechanism is orthogonal to `DISABLE_ACTUATION` (which kills ALL
actuators). Per-group disabling is deferred to a separate spec — it
requires `actuator_group` parsing and the `disableactuator` field on
`Model.opt`, neither of which exist yet.

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
not updated (retains previous/stale values — MuJoCo does NOT zero
`sensordata` on disable).

### AC8 — Warmstart disable
`<flag warmstart="disable"/>` starts solver from zero. Solver converges
(may need more iterations). Verify `efc_force` is not initialized from
`qacc_warmstart`.

### AC9 — Constraint disable
`<flag constraint="disable"/>` skips constraint assembly AND collision
detection. No equality, limit, contact, or friction loss constraints apply.
`data.ncon == 0`. `qacc == qacc_smooth`.

### AC10 — Passive top-level gating
When both `DISABLE_SPRING` and `DISABLE_DAMPER` are set, `mj_passive()`
returns early — skipping `mj_springdamper()`, `mj_gravcomp()`,
`mj_fluid()`, and `mj_contactPassive()`. All passive force vectors are
zero. When only one flag is set, `mj_passive()` runs normally (only the
individual spring or damper components are gated in `mj_springdamper()`).

### AC11 — Default bitfields
An unmodified `<option/>` with no `<flag>` produces `disableflags == 0`
(all bits clear) and `enableflags == 0`. All disable flags default to
"enable" (bit not set), including island. Matches MuJoCo's
`mj_defaultOption()` which sets `opt->disableflags = 0`.

**Codebase fix required:** The existing `MjcfFlag::default()` has
`island: false`, which causes the builder to set `DISABLE_ISLAND`. This
must be changed to `island: true` to match MuJoCo defaults.

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
with only that flag disabled. Tolerance: `1e-8` per `qacc` component.
(`1e-10` is unrealistic across different linear algebra backends and
floating-point accumulation orders; MuJoCo's own tests use `1e-8` or
looser.) Flags should produce exact gating — any residual delta should be
numerical noise, not a logic error.

### AC19 — Clampctrl disable
`<flag clampctrl="disable"/>` allows ctrl values outside ctrlrange. An
actuator with `ctrlrange="0 1"` accepts ctrl=2.0 without clamping.
`qfrc_actuator` reflects the unclamped ctrl value.

### AC20 — Eulerdamp disable
`<flag eulerdamp="disable"/>` skips implicit damping in Euler integrator.
Also verified: when `<flag damper="disable"/>` is set, implicit damping is
skipped regardless of `eulerdamp` setting (compound guard).

### AC21 — Passive force hierarchy
When both `DISABLE_SPRING` and `DISABLE_DAMPER` are set, ALL passive
sub-functions are skipped: `mj_springdamper()`, `mj_gravcomp()`,
`mj_fluid()`, and `mj_contactPassive()`. Verify: a body with
`gravcomp="1"` still receives gravity compensation when only ONE of
spring/damper is disabled (the `mj_passive()` early return requires BOTH).
Gravity compensation is only independently gated by `DISABLE_GRAVITY`.

### AC22 — Unconditional initialization before guards
`mj_collision()` unconditionally resets `ncon`, clears contacts, and
resets EFC arrays **before** checking disable flags. `mj_fwd_actuation()`
unconditionally zeroes `actuator_force` before the `DISABLE_ACTUATION`
guard. `mj_passive()` unconditionally zeroes all passive force vectors
before the spring/damper early-return guard. `mj_fwd_constraint()`
unconditionally zeroes `qfrc_constraint` before checking whether any
constraint rows exist. This "init-then-guard" pattern ensures
deterministic state regardless of which flags are set.

---

## Files

| File | Change |
|------|--------|
| `sim/L0/core/src/types/enums.rs` | S1: 17 disable + 4 enable constants, `disabled()`/`enabled()` helpers |
| `sim/L0/mjcf/src/types.rs` | S2: Split `passive` → `spring` + `damper`, add `fwdinv`/`invdiscrete`/`autoreset`, fix docstrings |
| `sim/L0/mjcf/src/parser.rs` | S2: Parse `spring`, `damper`, `passive` (compat), `fwdinv`, `invdiscrete`, `autoreset` |
| `sim/L0/mjcf/src/builder/mod.rs` | S3: `apply_flags()` replacing single sleep-only block |
| `sim/L0/core/src/forward/mod.rs` | S4.8, S5.1: Actuation activation advance guard, energy guards |
| `sim/L0/core/src/forward/velocity.rs` | S4.8: Actuator velocity guard |
| `sim/L0/core/src/sensor/mod.rs` | S4.10: Sensor disable guard (inside each sensor function) |
| `sim/L0/core/src/dynamics/rne.rs` | S4.2: Gravity guard in `mj_rne()` + `mj_gravcomp()` |
| `sim/L0/core/src/energy.rs` | S4.2: Gravity guard in `mj_energy_pos()` |
| `sim/L0/core/src/constraint/assembly.rs` | S4.1, S4.4-S4.6: Contact instantiation guard, equality, frictionloss, limit row guards |
| `sim/L0/core/src/forward/passive.rs` | S4.7, S6: mj_passive top-level guard, spring/damper independence, contactPassive guard, fluid sleep filter |
| `sim/L0/core/src/forward/actuation.rs` | S4.2, S4.9: Actuator-level gravcomp guard, clampctrl guard |
| `sim/L0/core/src/constraint/mod.rs` | S4.3, S4.11: Constraint assembly guard, warmstart function |
| `sim/L0/core/src/collision/mod.rs` | S4.1, S4.12: Collision disable guard (inside function), filterparent guard |
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
- **Island default fix:** The existing `MjcfFlag { island: false }` default
  is wrong — MuJoCo defaults to `disableflags = 0` (island enabled). Changing
  `island` to `true` may alter behavior if the hardcoded `DISABLE_ISLAND`
  constant was being set elsewhere. Audit all `DISABLE_ISLAND` usage sites.
- **Low risk:** Most guards are additive (new early-return paths). All disable
  flags default to "off" (bit not set), preserving current behavior.

---

## Verification

After implementation, run sim domain tests:

```
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics \
  -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf \
  -p sim-types -p sim-simd
```

Expected: 2,007+ passed (baseline), 0 failed, plus new per-flag tests.
