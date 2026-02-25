# §41 — Runtime Flag Wiring (+ DT-61)

**Status:** Not started
**Phase:** Roadmap Phase 2 — Runtime Flag Wiring
**Effort:** L (expanded to subsume DT-93, DT-94, DT-95)
**Prerequisites:** None
**Subsumes:** DT-61 (DISABLE_GRAVITY flag), DT-93 (auto-reset on NaN),
DT-94 (BVH midphase integration), DT-95 (global contact parameter override)

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
parser, builder, and runtime guards. It also implements the per-group
`disableactuator` bitmask for selective actuator disabling by group ID.

Additionally, this spec subsumes three features that are required for
complete flag guard coverage (previously deferred as DT-93, DT-94, DT-95):
- **S8:** Auto-reset on NaN/divergence (`DISABLE_AUTORESET` guard)
- **S9:** BVH midphase integration (`DISABLE_MIDPHASE` guard)
- **S10:** Global contact parameter override (`ENABLE_OVERRIDE` guard)

---

## MuJoCo Reference

**Verified against:** MuJoCo 3.4.0 and 3.5.0 source (`mjmodel.h`,
`engine_forward.c`, `engine_passive.c`, `engine_init.c`). Enums are
identical in both versions — no flags added between 3.4.0 and 3.5.0.

**Version history** (relevant for backward compatibility):

| Version | `mjNDISABLE` | `mjNENABLE` | Key changes |
|---------|-------------|-------------|-------------|
| 3.3.2–3.3.5 | 17 | 6 | Single `mjDSBL_PASSIVE` (bit 5). `mjENBL_ISLAND` (enable, bit 5). No `SLEEP`. |
| 3.3.6 | 19 | 5 | `PASSIVE` split → `SPRING` (bit 5) + `DAMPER` (bit 6). `passive` XML attr removed. `ISLAND` moved from enable to disable (bit 18). All disable bits 7–17 shifted +1. `SLEEP` not yet added. |
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

**Note:** MuJoCo 3.3.6+ removed the `passive` XML attribute entirely,
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

#### S2e. Fix `MjcfFlag::default()` — `island` must be `true`

**Bug:** `MjcfFlag::default()` currently has `island: false`. This causes
every model without an explicit `<flag island="enable"/>` to have
`DISABLE_ISLAND` set once `apply_flags()` is wired (S3).

**MuJoCo reference:** `mj_defaultOption()` sets `opt->disableflags = 0` —
all bits clear. All disable flags, including `ISLAND`, default to "enabled"
(bit not set). The MJCF convention is `island="enable"` by default.

**Fix:** In `types.rs`, `MjcfFlag::default()`: change `island: false` →
`island: true`.

**Verification:** After this change, `MjcfFlag::default()` must produce
`disableflags == 0` and `enableflags == 0` through `apply_flags()`. This
table is the definitive reference for all 25 fields:

| Field | Flag type | Correct default | Notes |
|-------|-----------|----------------|-------|
| `constraint` | disable | `true` | |
| `equality` | disable | `true` | |
| `frictionloss` | disable | `true` | |
| `limit` | disable | `true` | |
| `contact` | disable | `true` | |
| `spring` | disable | `true` | Replaces `passive` (S2a) |
| `damper` | disable | `true` | Replaces `passive` (S2a) |
| `gravity` | disable | `true` | |
| `clampctrl` | disable | `true` | |
| `warmstart` | disable | `true` | |
| `filterparent` | disable | `true` | |
| `actuation` | disable | `true` | |
| `refsafe` | disable | `true` | |
| `sensor` | disable | `true` | |
| `midphase` | disable | `true` | |
| `eulerdamp` | disable | `true` | |
| `autoreset` | disable | `true` | NEW field (S2b) |
| `nativeccd` | disable | `true` | |
| `island` | disable | **`true`** | **FIX** — currently `false` |
| `override_contacts` | enable | `false` | |
| `energy` | enable | `false` | |
| `fwdinv` | enable | `false` | NEW field (S2b) |
| `invdiscrete` | enable | `false` | NEW field (S2b) |
| `multiccd` | enable | `false` | |
| `sleep` | enable | `false` | |

**Risk mitigation:** Grep for all `DISABLE_ISLAND` usage sites. Currently
only `island/mod.rs:37` checks this flag. Verify that enabling island
discovery by default doesn't break tests. If it does, the tests were
relying on the wrong default and need fixing.

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

    let nbodyflex = model.nbody + model.nflex;  // MuJoCo computes inline
    if disabled(model, DISABLE_CONTACT)
        || disabled(model, DISABLE_CONSTRAINT)
        || nbodyflex < 2  // no collision possible with < 2 bodies/flexes
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

Note: `nbodyflex` is `m->nbody + m->nflex` in MuJoCo (computed as a local
variable inside the function). Both `model.nbody` and `model.nflex` exist
in CortenForge — compute inline.

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

#### S4.2a. Gravity compensation routing — `jnt_actgravcomp`

**Prerequisite for S4.2.** The `DISABLE_GRAVITY` guard in
`mj_fwd_actuation()` (S4.2) references `jnt_actgravcomp`, which doesn't
exist yet. The existing gravcomp infrastructure (`body_gravcomp`,
`ngravcomp`, `qfrc_gravcomp`, `mj_gravcomp()`) is complete but routing
is incomplete — `passive.rs:628-631` unconditionally adds all gravcomp
to `qfrc_passive` with a comment: "jnt_actgravcomp is not yet implemented."

MuJoCo routes gravcomp forces through two independent paths based on a
per-joint flag:
- `jnt_actgravcomp == false` (default): gravcomp → `qfrc_passive` (via
  `mj_passive()` in `engine_passive.c`)
- `jnt_actgravcomp == true`: gravcomp → `qfrc_actuator` (via
  `mj_fwdActuation()` in `engine_forward.c`)

In both cases, raw values live in `qfrc_gravcomp` first.

**Model field:**

```rust
/// Per-joint flag: if true, gravcomp force routes through `qfrc_actuator`
/// instead of `qfrc_passive`. Parsed from `<joint actuatorgravcomp="true"/>`.
/// Default: false (gravcomp goes to passive forces).
pub jnt_actgravcomp: Vec<bool>,  // length njnt
```

Initialize to `vec![false; njnt]` in `model_init.rs`.

**Parser:** Add `actuatorgravcomp` to joint attribute parsing in
`parser.rs`. Boolean attribute, default `false`. Wire through builder
to `Model.jnt_actgravcomp[jnt_idx]`.

**Runtime routing — passive side** (replaces `passive.rs:626-632`):

```rust
// Gravity compensation: compute into qfrc_gravcomp, then route.
// DOFs whose joint has jnt_actgravcomp == true are routed via
// mj_fwd_actuation() instead (S4.2).
if mj_gravcomp(model, data) {
    let use_sleep_filter = sleep_enabled && data.nv_awake < model.nv;
    let ndof = if use_sleep_filter { data.nv_awake } else { model.nv };
    for j in 0..ndof {
        let dof = if use_sleep_filter { data.dof_awake_ind[j] } else { j };
        let jnt = model.dof_jntid[dof];
        if !model.jnt_actgravcomp[jnt] {
            data.qfrc_passive[dof] += data.qfrc_gravcomp[dof];
        }
    }
}
```

**Runtime routing — actuation side** (in `mj_fwd_actuation()`, S4.2):

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

**`DISABLE_GRAVITY` interaction:** When `DISABLE_GRAVITY` is set,
`mj_gravcomp()` returns `false` (see S4.2 guard), so no gravcomp forces
are computed — neither path runs. When `DISABLE_ACTUATION` is set, the
actuation-side routing is skipped entirely (S4.8 returns early), but
passive-side routing still runs for `jnt_actgravcomp == false` joints.
This matches MuJoCo: disabling actuation doesn't disable passive gravcomp.

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

MuJoCo's passive force pipeline has a **hierarchical guard structure**.

##### S4.7-prereq: Separate spring/damper force arrays

MuJoCo 3.4.0+ maintains five separate passive force arrays: `qfrc_spring`,
`qfrc_damper`, `qfrc_gravcomp`, `qfrc_fluid`, `qfrc_passive`. CortenForge
has three (`qfrc_gravcomp`, `qfrc_fluid`, `qfrc_passive`) but spring and
damper forces are merged directly into `qfrc_passive` by the
`PassiveForceVisitor`.

The separation is required for three reasons:
1. `DISABLE_SPRING` and `DISABLE_DAMPER` gate spring and damper forces
   **independently**. Without separate arrays, the disable flag must be
   threaded into every joint visitor, tendon loop, and flex computation.
   With separate arrays, skip accumulation at the aggregation point.
2. MuJoCo's aggregation pattern (`qfrc_passive = spring + damper +
   gravcomp + fluid`) is cleaner and matches the unconditional zeroing
   in S4.7a.
3. Derivatives and sensors may reference individual force components.

**Data fields to add:**

```rust
/// Passive spring forces (length `nv`), zeroed each step.
pub qfrc_spring: DVector<f64>,
/// Passive damper forces (length `nv`), zeroed each step.
pub qfrc_damper: DVector<f64>,
```

**Refactor `mj_fwd_passive()`:** The `PassiveForceVisitor` currently writes
spring and damper contributions directly into `qfrc_passive`. Refactor to:

1. Write spring forces into `qfrc_spring`
2. Write damper forces into `qfrc_damper`
3. After all sub-functions (springdamper, gravcomp, fluid, contactPassive),
   aggregate:
   ```rust
   // Aggregation into qfrc_passive (matches MuJoCo mj_passive() pattern).
   // Uses boolean returns from sub-functions to skip unnecessary additions.
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

**Tendon and flex passive forces** also write spring/damper components
separately. The tendon loop (lines 366–430 in `passive.rs`) and flex
passive computation must be updated to target `qfrc_spring`/`qfrc_damper`
instead of `qfrc_passive` directly.

**Sleep-filtered aggregation:** When sleep filtering is active, aggregation
uses indexed iteration over `dof_awake_ind` (matching S4.7a's zeroing
pattern).

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
// Note: the sleep filter here uses DOF-level counts (nv_awake / nv),
// not body-level (nbody_awake). Body-level filtering is used in
// mj_fluid() for per-body iteration (S6), but the top-level
// mj_passive() zeroing operates at the DOF level.
let sleep_filter = enabled(model, ENABLE_SLEEP)
    && data.nv_awake < model.nv;

if sleep_filter {
    // Zero only awake DOFs (indexed by dof_awake_ind).
    for &dof in &data.dof_awake_ind[..data.nv_awake] {
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
// mj_springdamper() always writes into qfrc_spring and qfrc_damper
// (no boolean return — spring+damper are always aggregated).
mj_springdamper(model, data);

// mj_gravcomp() and mj_fluid() return bool indicating whether they
// produced non-zero forces. Use these to skip unnecessary additions.
let has_gravcomp: bool = mj_gravcomp(model, data);
let has_fluid: bool = mj_fluid(model, data);

// qfrc_passive = qfrc_spring + qfrc_damper
//              + (if has_gravcomp) qfrc_gravcomp
//              + (if has_fluid)    qfrc_fluid
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

**Implementation note on boolean returns:** `mj_springdamper()` has no
return value — it unconditionally writes into `qfrc_spring` and
`qfrc_damper`, so these are always aggregated. In contrast,
`mj_gravcomp()` returns `true` only when `ngravcomp > 0` (at least one
body has `gravcomp != 0`), and `mj_fluid()` returns `true` only when at
least one body has non-zero fluid interaction parameters. The aggregation
loop uses these booleans to skip the `+=` pass entirely when no forces
were produced, which matters for performance in large models. When sleep
filtering is active, aggregation also uses indexed iteration over awake
DOFs only.

After aggregation, MuJoCo invokes the user callback (`mjcb_passive`)
and plugin dispatch (`mjPLUGIN_PASSIVE`), both of which can modify
`qfrc_passive` further. **These callbacks are out of scope for §41** —
they are tracked separately under **DT-79** (user callback system). Once
DT-79 is implemented, the callback invocation point is here, immediately
after aggregation and before `qfrc_passive` is consumed downstream.

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

MuJoCo gates activation advancement per-actuator via
`mj_actuatorDisabled()`, which checks both `DISABLE_ACTUATION` (blanket)
and the per-group `disableactuator` bitmask (S7). Disabled actuators
get `act_dot = 0`, freezing their activation state without zeroing it:

```rust
for i in 0..model.nu {
    if !model.actuator_has_activation(i) { continue; }
    let disabled = disabled(model, DISABLE_ACTUATION)
                || actuator_disabled(model, i);
    let act_dot_val = if disabled { 0.0 } else { act_dot[j] };
    // mj_nextActivation(model, data, i, j, act_dot_val)
}
```

Note: `DISABLE_ACTUATION` also gates `mj_fwdActuation()` (Site 1),
so `act_dot` is never computed when it's set — the per-actuator check
here is redundant for that flag but required for per-group disabling.

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
        return;  // sensordata is NOT zeroed — intentional MuJoCo match
    }
    // ... sensor evaluation ...
}
```

**Important behavioral note:** Unlike `mj_collision()`, `mj_fwdActuation()`,
`mj_passive()`, and `mj_fwdConstraint()`, the sensor functions do **NOT**
zero their output arrays before the disable guard. When `DISABLE_SENSOR` is
set, `sensordata` retains values from the last step when sensors were
enabled. This is MuJoCo's actual behavior (verified in `engine_sensor.c` —
all three functions return immediately without touching `sensordata`).

**Rationale:** Sensors are read-only observers — they don't affect physics.
Stale sensor data cannot cause incorrect force accumulation or state
corruption. The "init-then-guard" pattern used for physics arrays
(`qfrc_constraint`, `actuator_force`, `qfrc_passive`, `ncon`) exists
because stale values in those arrays would produce wrong physics. Sensors
have no such risk.

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

Guard site for BVH midphase (fully specified in S9). When disabled, skip
BVH tree traversal and fall back to brute-force all-pairs narrowphase:

```rust
if !disabled(model, DISABLE_MIDPHASE) && has_bvh(model, geom1, geom2) {
    mj_collide_tree(model, data, geom1, geom2);
} else {
    // brute-force all-pairs narrowphase (SAP broadphase only)
}
```

Also affects flex self-collision midphase and BVH updates on flex objects.
See S9 for the full BVH integration specification.

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

Guard site for NaN/divergence auto-reset (fully specified in S8). When
disabled, validation warnings are still issued but the automatic reset
to initial state is skipped:

```rust
if !disabled(model, DISABLE_AUTORESET) {
    mj_reset_data(model, data);
}
```

See S8 for the full auto-reset specification including `mj_checkPos`,
`mj_checkVel`, `mj_checkAcc`, and the `mju_isBad` detection primitive.

#### S4.17 `DISABLE_NATIVECCD` — CCD fallback

**Constant only — no guard site.** Blocked by §50 (continuous collision
detection). No native CCD or libccd dispatch exists. Once §50 implements
CCD with a native-vs-libccd code path, the guard should gate fallback
to libccd for convex collision.

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

Guard sites for global contact parameter override (fully specified in S10).
When enabled, replaces per-geom/pair contact parameters with global
override values from `Model`. Affects 6 locations:

1. **Broadphase AABB margin** (2 sites: geoms + flexes)
2. **Narrowphase contact margin** (1 site: `mj_assignMargin`)
3. **Contact solref** (1 site: `mj_assignRef`)
4. **Contact solimp** (1 site: `mj_assignImp`)
5. **Contact friction** (1 site: `mj_assignFriction`)

Note: `<pair>` overrides (per-geom-pair) already work — this is the
*global* override mechanism, which is separate. Gap and condim are never
overridden by this mechanism.

See S10 for the full override specification including Model fields,
parser, assignment helpers, and all guard sites.
#### S5.3 `ENABLE_FWDINV` — forward/inverse comparison

**Constant only — no guard site.** Blocked by §52 (`mj_inverse()`,
Phase 3). Once §52 implements inverse dynamics, the guard should gate
the forward/inverse comparison statistics computation.

#### S5.4 `ENABLE_INVDISCRETE` — discrete inverse dynamics

**Constant only — no guard site.** Blocked by §52 (`mj_inverse()`,
Phase 3). Same dependency as S5.3.

#### S5.5 `ENABLE_MULTICCD` — multi-point CCD

**Constant only — no guard site.** Blocked by §50 (continuous collision
detection, Phase 9). Once §50 implements CCD, the guard should gate
multi-point CCD for flat surfaces.

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

### S7. Per-group actuator disabling (`disableactuator` bitmask)

MuJoCo has a third bitfield `opt.disableactuator` (an `int`, 31 usable
bits) that disables actuators by group ID. It is checked via
`mj_actuatorDisabled()` inside `mj_fwdActuation()` and `mj_advance()`:

```c
int mj_actuatorDisabled(const mjModel* m, int i) {
    int group = m->actuator_group[i];
    if (group < 0 || group > 30) return 0;
    return m->opt.disableactuator & (1 << group) ? 1 : 0;
}
```

This mechanism is orthogonal to `DISABLE_ACTUATION` (which kills ALL
actuators). Per-group disabling allows selective control — e.g., disable
group 2 actuators while keeping groups 0 and 1 active.

**Current state:** The parser already reads `actuator.group` (`parser.rs:2112`,
`types.rs` `group: Option<i32>`), but the value is never wired to the
compiled `Model`.

#### S7a. Model fields

Add to `Model` (next to `disableflags`/`enableflags`):

```rust
/// Per-group actuator disable bitmask. Bit `i` set = group `i` disabled.
/// Parsed from `<option actuatorgroupdisable="2 5"/>` (space-separated
/// list of group IDs, each 0–30) or set at runtime.
pub disableactuator: u32,

/// Group assignment per actuator (0–30). Default 0.
/// Parsed from `<actuator><general group="..."/>`.
pub actuator_group: Vec<i32>,
```

Initialize `disableactuator` to `0` (no groups disabled) in `model_init.rs`.
Initialize `actuator_group` to `vec![0; nu]` (all actuators in group 0).

#### S7b. Builder wiring

In the actuator builder, wire the parsed `group` to `Model.actuator_group`:

```rust
model.actuator_group[i] = actuator.group.unwrap_or(0);
```

Parse `actuatorgroupdisable` from `<option>` as a space-separated list of
integer group IDs (matching MuJoCo's XML attribute name). Each ID sets a
bit in `disableactuator`:

```rust
if let Some(groups_str) = get_attribute_opt(option_elem, "actuatorgroupdisable") {
    for token in groups_str.split_whitespace() {
        let group: i32 = token.parse().map_err(|_| {
            MjcfError::InvalidAttribute("actuatorgroupdisable", token.to_string())
        })?;
        if group < 0 || group > 30 {
            return Err(MjcfError::OutOfRange(
                "actuatorgroupdisable group", 0, 30, group
            ));
        }
        model.disableactuator |= 1u32 << (group as u32);
    }
}
```

#### S7c. `actuator_disabled()` helper

Add to `types/enums.rs` next to `disabled()`/`enabled()`:

```rust
/// Returns true if actuator `i` is disabled by group membership.
/// Groups outside 0–30 are never disabled (matches MuJoCo).
#[inline]
pub fn actuator_disabled(model: &Model, i: usize) -> bool {
    let group = model.actuator_group[i];
    if group < 0 || group > 30 { return false; }
    model.disableactuator & (1 << group as u32) != 0
}
```

#### S7d. Runtime guard sites

**Site 1 — `mj_fwdActuation()`:** Inside the per-actuator force loop,
skip force computation for disabled actuators:

```rust
for i in 0..model.nu {
    if actuator_disabled(model, i) { continue; }
    // ... compute actuator force ...
}
```

**Site 2 — `mj_advance()` activation state:** Per-actuator gating
replaces the blanket `DISABLE_ACTUATION` guard for activation:

```rust
for i in 0..model.nu {
    if !model.actuator_has_activation(i) { continue; }
    let act_dot_val = if actuator_disabled(model, i) { 0.0 } else { act_dot[j] };
    // mj_nextActivation(model, data, i, j, act_dot_val)
}
```

This matches MuJoCo's actual mechanism exactly — disabled actuators get
`act_dot = 0`, freezing their activation state without zeroing it.

### S8. Auto-reset on NaN/divergence (subsumes DT-93)

MuJoCo has three validation functions that detect divergent simulation
state and automatically reset to initial configuration. This is a safety
feature enabled by default — `DISABLE_AUTORESET` (S4.16) gates it.

**Verified against:** MuJoCo `engine_forward.c` (`mj_checkPos`,
`mj_checkVel`, `mj_checkAcc`) and `engine_util_misc.c` (`mju_isBad`).

#### S8a. Detection primitive — `is_bad()`

```rust
/// Maximum allowed value in qpos, qvel, qacc (matches MuJoCo's mjMAXVAL).
pub const MAX_VAL: f64 = 1e10;

/// Returns true if value is NaN, +inf, -inf, or exceeds MAX_VAL.
/// Matches MuJoCo's `mju_isBad()`.
#[inline]
pub fn is_bad(x: f64) -> bool {
    x.is_nan() || x > MAX_VAL || x < -MAX_VAL
}
```

**File:** `types/enums.rs` (next to `disabled()`/`enabled()` helpers).

#### S8b. Warning system

Add a warning tracking system to `Data`:

```rust
/// Warning types (matches MuJoCo's mjtWarning enum).
#[repr(usize)]
pub enum Warning {
    Inertia = 0,
    ContactFull = 1,
    ConstraintFull = 2,
    VgeomFull = 3,
    BadQpos = 4,
    BadQvel = 5,
    BadQacc = 6,
    BadCtrl = 7,
}
pub const NUM_WARNINGS: usize = 8;

/// Per-warning statistics.
pub struct WarningStat {
    pub last_info: i32,  // index that triggered the warning
    pub count: i32,      // cumulative count
}

// In Data:
pub warnings: [WarningStat; NUM_WARNINGS],
```

Initialize all to zero in `Data::new()`. The warning function:

```rust
fn mj_warning(data: &mut Data, warning: Warning, info: i32) {
    let w = &mut data.warnings[warning as usize];
    if w.count == 0 {
        tracing::warn!(
            "{} Time = {:.4}.",
            warning_text(warning, info),
            data.time
        );
    }
    w.last_info = info;
    w.count += 1;
}
```

#### S8c. Validation functions

**`mj_check_pos()` — before forward dynamics:**

```rust
pub fn mj_check_pos(model: &Model, data: &mut Data) {
    for i in 0..model.nq {
        if is_bad(data.qpos[i]) {
            mj_warning(data, Warning::BadQpos, i as i32);
            if !disabled(model, DISABLE_AUTORESET) {
                mj_reset_data(model, data);
            }
            // Re-set warning after reset (reset zeroes all warnings).
            data.warnings[Warning::BadQpos as usize].count += 1;
            data.warnings[Warning::BadQpos as usize].last_info = i as i32;
            return;
        }
    }
}
```

**`mj_check_vel()` — before forward dynamics (sleep-aware):**

```rust
pub fn mj_check_vel(model: &Model, data: &mut Data) {
    let sleep_filter = enabled(model, ENABLE_SLEEP) && data.nv_awake < model.nv;
    let nv = if sleep_filter { data.nv_awake } else { model.nv };

    for j in 0..nv {
        let i = if sleep_filter { data.dof_awake_ind[j] } else { j };
        if is_bad(data.qvel[i]) {
            mj_warning(data, Warning::BadQvel, i as i32);
            if !disabled(model, DISABLE_AUTORESET) {
                mj_reset_data(model, data);
            }
            data.warnings[Warning::BadQvel as usize].count += 1;
            data.warnings[Warning::BadQvel as usize].last_info = i as i32;
            return;
        }
    }
}
```

**`mj_check_acc()` — after forward dynamics (sleep-aware, re-runs forward):**

```rust
pub fn mj_check_acc(model: &Model, data: &mut Data) {
    let sleep_filter = enabled(model, ENABLE_SLEEP) && data.nv_awake < model.nv;
    let nv = if sleep_filter { data.nv_awake } else { model.nv };

    for j in 0..nv {
        let i = if sleep_filter { data.dof_awake_ind[j] } else { j };
        if is_bad(data.qacc[i]) {
            mj_warning(data, Warning::BadQacc, i as i32);
            if !disabled(model, DISABLE_AUTORESET) {
                mj_reset_data(model, data);
            }
            data.warnings[Warning::BadQacc as usize].count += 1;
            data.warnings[Warning::BadQacc as usize].last_info = i as i32;
            // Unlike checkPos/checkVel, re-run forward after reset
            // to recompute derived quantities from the reset state.
            if !disabled(model, DISABLE_AUTORESET) {
                mj_forward(model, data);
            }
            return;
        }
    }
}
```

**Key detail:** `mj_check_acc` calls `mj_forward()` after reset because
it runs AFTER the forward pass — the derived quantities (contacts, forces)
need recomputation from the freshly reset state. `mj_check_pos` and
`mj_check_vel` run BEFORE forward, so no recomputation is needed.

**Warning counter preservation:** `mj_reset_data()` zeroes all warning
counters. The explicit `count += 1` after reset ensures the triggering
warning survives the reset — users can detect that a reset occurred by
checking `data.warnings[BadQpos].count > 0`.

#### S8d. Ctrl validation (separate mechanism)

Bad ctrl values are caught inside `mj_fwd_actuation()` (not in the check
functions). Bad ctrl does NOT trigger a reset — it zeroes all ctrl values:

```rust
// Inside mj_fwd_actuation(), before force computation:
for i in 0..model.nu {
    if is_bad(data.ctrl[i]) {
        mj_warning(data, Warning::BadCtrl, i as i32);
        data.ctrl[..model.nu].fill(0.0);
        break;
    }
}
```

#### S8e. Pipeline integration

**In `mj_step()`:**
```rust
pub fn mj_step(model: &Model, data: &mut Data) {
    mj_check_pos(model, data);    // (1) validate qpos BEFORE forward
    mj_check_vel(model, data);    // (2) validate qvel BEFORE forward
    mj_forward(model, data);      // (3) forward dynamics
    mj_check_acc(model, data);    // (4) validate qacc AFTER forward
    // ... integrator ...
}
```

**In `mj_step1()`** (split stepping):
```rust
pub fn mj_step1(model: &Model, data: &mut Data) {
    mj_check_pos(model, data);
    mj_check_vel(model, data);
    // ... position-dependent computations ...
    // NOTE: no mj_check_acc — that's in step2
}
```

**In `mj_step2()`** (split stepping):
```rust
pub fn mj_step2(model: &Model, data: &mut Data) {
    // ... actuation, acceleration, constraint solve ...
    mj_check_acc(model, data);
    // ... integrator ...
}
```

#### S8f. Reset behavior

`mj_reset_data()` resets to the model's initial state (`qpos0`), NOT a
keyframe. It:
1. Zeroes all `Data` arrays (forces, contacts, constraints, sensors)
2. Copies `model.qpos0` → `data.qpos` (reference configuration)
3. Zeroes `qvel`, `qacc`, `ctrl`, `act`
4. Resets `data.time` to 0.0
5. Zeroes all warning counters
6. Resets contact/constraint counts to zero
7. Initializes mocap bodies from model

If a `mj_reset_data()` function doesn't exist yet, implement it as part
of this section. It is analogous to MuJoCo's `mj_resetData()` in
`engine_io.c`.

### S9. BVH midphase integration (subsumes DT-94)

`mid_phase.rs` (1,178 lines) contains a complete BVH implementation that
is not yet called from `collision/mod.rs`. This section integrates it
into the collision pipeline and wires the `DISABLE_MIDPHASE` guard.

**Verified against:** MuJoCo's `engine_collision_driver.c` midphase
architecture and existing CortenForge `mid_phase.rs` API.

#### S9a. Current collision pipeline

The current pipeline in `collision/mod.rs` has three mechanisms:

1. **SAP broadphase** (line ~314): Sweep-and-Prune produces candidate
   geom pairs → affinity filter → sleep filter → narrowphase dispatch.
   O(n log n) for coherent motion, O(n^2) worst case.
2. **Explicit pairs** (line ~357): User-defined `<pair>` entries bypass
   broadphase and affinity filtering.
3. **Flex-rigid brute force** (line ~402): O(V × G) all-pairs for flex
   vertices vs rigid geoms.

For mesh-mesh and mesh-convex collisions, the narrowphase currently tests
all triangles — no midphase culling.

#### S9b. BVH integration architecture

Midphase sits between broadphase and narrowphase. For geom pairs where
one or both geoms are meshes, the BVH culls triangle pairs before
narrowphase testing:

```
SAP broadphase → candidate geom pairs
    → affinity/sleep filter
    → for each pair:
        if mesh-mesh or mesh-convex:
            if !DISABLE_MIDPHASE:
                BVH midphase → candidate triangle pairs → narrowphase
            else:
                brute-force all triangles → narrowphase
        else:
            direct narrowphase (sphere-sphere, box-capsule, etc.)
```

#### S9c. Per-mesh BVH storage

Add BVH storage to the model's mesh data:

```rust
/// Pre-built BVH for each mesh geom. Built once during model compilation.
/// Index by mesh ID (not geom ID). None for non-mesh geoms.
pub mesh_bvh: Vec<Option<Bvh>>,
```

**Build phase (in model builder):** After mesh vertices/faces are loaded,
build a BVH for each mesh:

```rust
for mesh_id in 0..model.nmesh {
    let verts = &model.mesh_vert[mesh_vert_range(mesh_id)];
    let faces = &model.mesh_face[mesh_face_range(mesh_id)];
    model.mesh_bvh[mesh_id] = Some(bvh_from_triangle_mesh(verts, faces));
}
```

This uses the existing `bvh_from_triangle_mesh()` from `mid_phase.rs`.

#### S9d. Midphase dispatch in narrowphase

**File:** `collision/narrow.rs` or `collision/mesh_collide.rs`

When colliding two mesh geoms (or mesh vs convex), use BVH dual-tree
query to find overlapping triangle pairs:

```rust
fn collide_mesh_mesh_midphase(
    model: &Model, geom1: usize, geom2: usize,
) -> Vec<Contact> {
    let mesh1 = model.geom_dataid[geom1];
    let mesh2 = model.geom_dataid[geom2];
    let bvh1 = model.mesh_bvh[mesh1].as_ref().unwrap();
    let bvh2 = model.mesh_bvh[mesh2].as_ref().unwrap();

    // Transform BVH2's AABBs into BVH1's frame
    let tf1 = model_geom_transform(model, geom1);
    let tf2 = model_geom_transform(model, geom2);

    let candidate_pairs = query_bvh_pair(bvh1, bvh2, &tf1, &tf2);

    let mut contacts = Vec::new();
    for (tri1, tri2) in candidate_pairs {
        if let Some(contact) = triangle_triangle_test(
            model, geom1, geom2, tri1, tri2,
        ) {
            contacts.push(contact);
        }
    }
    contacts
}
```

For mesh vs convex primitive (sphere, capsule, box), use single-tree
BVH query with the primitive's AABB:

```rust
fn collide_mesh_primitive_midphase(
    model: &Model, mesh_geom: usize, prim_geom: usize,
) -> Vec<Contact> {
    let mesh_id = model.geom_dataid[mesh_geom];
    let bvh = model.mesh_bvh[mesh_id].as_ref().unwrap();
    let prim_aabb = geom_aabb(model, prim_geom);

    // Transform AABB into mesh frame
    let prim_aabb_local = transform_aabb_to_mesh_frame(
        &prim_aabb, model, mesh_geom,
    );

    let candidate_tris = bvh.query(&prim_aabb_local);

    let mut contacts = Vec::new();
    for tri_idx in candidate_tris {
        if let Some(contact) = triangle_primitive_test(
            model, mesh_geom, prim_geom, tri_idx,
        ) {
            contacts.push(contact);
        }
    }
    contacts
}
```

#### S9e. `DISABLE_MIDPHASE` guard

**File:** `collision/mod.rs` — inside narrowphase dispatch

```rust
let use_midphase = !disabled(model, DISABLE_MIDPHASE);

// In the narrowphase dispatch for mesh pairs:
if use_midphase && is_mesh(model, geom1) && is_mesh(model, geom2) {
    collide_mesh_mesh_midphase(model, geom1, geom2)
} else if use_midphase && (is_mesh(model, geom1) || is_mesh(model, geom2)) {
    collide_mesh_primitive_midphase(model, mesh_geom, prim_geom)
} else {
    // Existing brute-force path (all triangles tested)
    collide_geoms_brute(model, data, geom1, geom2)
}
```

When disabled, the existing all-triangle-pairs code path runs unmodified.

#### S9f. Flex midphase (future extension)

Flex self-collision (§42A-iv) could also benefit from BVH midphase. When
flex objects have many vertices, a BVH over flex elements would replace
the current O(V × G) brute-force. This is an incremental extension of
the same infrastructure and is tracked in §42A-iv. The `DISABLE_MIDPHASE`
guard should also gate flex midphase once it exists.

### S10. Global contact parameter override (subsumes DT-95)

Implements the global contact parameter override mechanism gated by
`ENABLE_OVERRIDE`. When enabled, all contacts use the same margin,
solref, solimp, and friction values from `Model.opt` instead of
per-geom/pair computed parameters.

**Verified against:** MuJoCo `engine_core_constraint.c` (4 assignment
functions) and `engine_collision_driver.c` (2 broadphase sites).

#### S10a. Model fields

Add override fields to the options struct (next to `disableflags`/
`enableflags` in `Model`):

```rust
/// Global contact margin override. Used when ENABLE_OVERRIDE is set.
/// Default: 0.0. Parsed from `<option o_margin="..."/>`.
pub o_margin: f64,

/// Global contact solver reference override [timeconst, dampratio].
/// Default: [0.02, 1.0]. Parsed from `<option o_solref="..."/>`.
pub o_solref: [f64; 2],  // mjNREF = 2

/// Global contact solver impedance override [dmin, dmax, width, midpoint, power].
/// Default: [0.9, 0.95, 0.001, 0.5, 2.0]. Parsed from `<option o_solimp="..."/>`.
pub o_solimp: [f64; 5],  // mjNIMP = 5

/// Global contact friction override [tangent1, tangent2, torsional, rolling1, rolling2].
/// Default: [1.0, 1.0, 0.005, 0.0001, 0.0001].
/// Parsed from `<option o_friction="..."/>`.
/// Note: MuJoCo stores 5D directly (unlike per-geom 3D friction which
/// gets unpacked to 5D). When override is active, tangent1 and tangent2
/// CAN differ (unlike per-geom where they're always equal).
pub o_friction: [f64; 5],
```

Initialize in `model_init.rs` with MuJoCo defaults:
```rust
o_margin: 0.0,
o_solref: [0.02, 1.0],
o_solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
o_friction: [1.0, 1.0, 0.005, 0.0001, 0.0001],
```

#### S10b. Parser wiring

Parse from `<option>` element (NOT `<option><override>` — these are
direct attributes on `<option>`, matching MuJoCo's XML schema):

```rust
// In option parsing:
model.o_margin = parse_attr_f64(option_elem, "o_margin", model.o_margin);
model.o_solref = parse_attr_array(option_elem, "o_solref", model.o_solref);
model.o_solimp = parse_attr_array(option_elem, "o_solimp", model.o_solimp);
model.o_friction = parse_attr_array(option_elem, "o_friction", model.o_friction);
```

The enable flag `override` is already parsed in `MjcfFlag.override_contacts`
and wired via `apply_flags()` (S3) to `ENABLE_OVERRIDE`.

#### S10c. Assignment helper functions

Add four helper functions matching MuJoCo's `engine_core_constraint.c`:

**File:** `constraint/contact_params.rs` (new) or `collision/params.rs`

```rust
/// Minimum friction coefficient (matches MuJoCo's mjMINMU = 1e-5).
pub const MIN_MU: f64 = 1e-5;

/// Assign contact margin, applying global override if enabled.
#[inline]
pub fn assign_margin(model: &Model, source: f64) -> f64 {
    if enabled(model, ENABLE_OVERRIDE) {
        model.o_margin
    } else {
        source
    }
}

/// Assign contact solref, applying global override if enabled.
#[inline]
pub fn assign_ref(model: &Model, source: &[f64; 2]) -> [f64; 2] {
    if enabled(model, ENABLE_OVERRIDE) {
        model.o_solref
    } else {
        *source
    }
}

/// Assign contact solimp, applying global override if enabled.
#[inline]
pub fn assign_imp(model: &Model, source: &[f64; 5]) -> [f64; 5] {
    if enabled(model, ENABLE_OVERRIDE) {
        model.o_solimp
    } else {
        *source
    }
}

/// Assign contact friction, applying global override if enabled.
/// Friction is always clamped to MIN_MU regardless of override.
#[inline]
pub fn assign_friction(model: &Model, source: &[f64; 5]) -> [f64; 5] {
    let src = if enabled(model, ENABLE_OVERRIDE) {
        model.o_friction
    } else {
        *source
    };
    [
        src[0].max(MIN_MU),
        src[1].max(MIN_MU),
        src[2].max(MIN_MU),
        src[3].max(MIN_MU),
        src[4].max(MIN_MU),
    ]
}
```

#### S10d. Guard sites (6 locations)

**Sites 1–2: Broadphase AABB margin expansion**

**File:** `collision/mod.rs` — AABB computation for broadphase

When computing AABBs for SAP broadphase, the per-geom margin is used
for AABB expansion. Override replaces it with `0.5 * o_margin`:

```rust
// For rigid body geoms:
let margin = if enabled(model, ENABLE_OVERRIDE) {
    0.5 * model.o_margin
} else {
    model.geom_margin[geom_id]
};

// For flex objects:
let margin = if enabled(model, ENABLE_OVERRIDE) {
    0.5 * model.o_margin
} else {
    model.flex_margin[flex_id]
};
```

The `0.5` factor is because AABB expansion is per-geom (half the contact
margin), while narrowphase uses the full `o_margin` to replace the summed
`margin[g1] + margin[g2]`.

**Sites 3–6: Contact parameter assignment**

**File:** `collision/mod.rs` or `collision/contact_params.rs`

After `mj_contact_param()` computes combined parameters from geom/flex
properties, the assignment helpers apply override:

```rust
// In contact creation (mj_setContact equivalent):
con.margin = assign_margin(model, combined_margin);
con.solref = assign_ref(model, &combined_solref);
con.solreffriction = assign_ref(model, &combined_solreffriction);  // same o_solref
con.solimp = assign_imp(model, &combined_solimp);
con.friction = assign_friction(model, &combined_friction);
```

**Key behaviors:**
- Override is **total replacement**: when enabled, combined parameters are
  computed but discarded.
- `solreffriction` uses the same `assign_ref` (and thus `o_solref`) — there
  is no separate `o_solreffriction`.
- Gap is **never overridden**: `includemargin = assign_margin(...) - gap`,
  so gap still matters.
- Condim is **never overridden**: contact dimensionality always comes from
  geom/flex properties.
- Override applies **only to contacts**, not to joint limits, tendon limits,
  equality constraints, or friction loss constraints.

---

## Implementation Order

1. **S1** — Define all flag constants + `is_bad()` + `Warning` enum in
   `enums.rs`. Define `MAX_VAL`, `disabled()`, `enabled()`, `actuator_disabled()`
   helpers.
2. **S2** — Parser changes: split `passive`, add missing fields, fix docs,
   fix `island` default (S2e), add `actuatorgravcomp` joint attribute.
3. **S3** — Builder wiring: `apply_flags()` replaces single sleep-only block.
4. **S4.7-prereq** — Add `qfrc_spring`, `qfrc_damper` Data fields. Refactor
   `mj_fwd_passive()` to write spring/damper into separate arrays and
   aggregate into `qfrc_passive` at the end.
5. **S4.2a** — Add `jnt_actgravcomp` Model field. Wire parser. Update
   `mj_fwd_passive()` gravcomp routing (passive-side) and prep actuation-side
   routing.
6. **S4.1-S4.7** — High-impact runtime guards: contact, gravity (+ actuation-
   side gravcomp routing), constraint, equality, limit, frictionloss,
   spring/damper.
7. **S4.8-S4.12** — Medium-impact guards: actuation, clampctrl, sensor,
   warmstart, filterparent.
8. **S4.14-S4.15** — Implementable guards: eulerdamp, refsafe.
   **S4.17** — Constant only (no guard site): nativeccd (blocked by §50).
9. **S5.1** — Energy guard. **S5.3-S5.5** — Constant only (no guard
   site): fwdinv/invdiscrete (blocked by §52), multiccd (blocked by §50).
   **S5.6** — Already wired.
10. **S6** — Fluid sleep filtering.
11. **S7** — Per-group actuator disabling: model fields, builder wiring,
    `actuator_disabled()` helper, runtime guards in actuation + advance.
12. **S8** — Auto-reset: warning system, `is_bad()` validation, `mj_checkPos`,
    `mj_checkVel`, `mj_checkAcc`, `mj_reset_data()`, pipeline integration.
    Wires `DISABLE_AUTORESET` guard (S4.16).
13. **S9** — BVH midphase: per-mesh BVH storage in Model, build BVHs during
    compilation, midphase dispatch in narrowphase, SAP fallback path.
    Wires `DISABLE_MIDPHASE` guard (S4.13).
14. **S10** — Global override: Model fields (`o_margin`, `o_solref`, `o_solimp`,
    `o_friction`), parser, assignment helpers, 6 guard sites.
    Wires `ENABLE_OVERRIDE` guard (S5.2).

Steps 1-3 form a single commit (infrastructure). Step 4 is a standalone
refactor commit. Steps 5-14 can each be a commit. Steps 12-14 are
independent of each other and can be implemented in any order.

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
`<flag sensor="disable"/>` skips sensor evaluation. `sensordata` retains
values from the last enabled step (NOT zeroed — intentional MuJoCo match).
This differs from the "init-then-guard" pattern because sensors are
observers, not force producers. See S4.10 rationale.

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
`mj_defaultOption()` which sets `opt->disableflags = 0`. Verified by
the S2e default audit table.

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

### AC22 — Per-group actuator disabling
`<option actuatorgroupdisable="2"/>` disables all group-2 actuators
(sets bit 2 in `disableactuator`). An actuator with `group="2"` produces
zero force; an actuator with `group="0"` is unaffected. Multiple groups:
`<option actuatorgroupdisable="2 5"/>` disables groups 2 and 5.
Activation state of disabled actuators freezes (act_dot = 0) but is not
zeroed. Actuators with `group < 0` or `group > 30` are never disabled by
this mechanism. Orthogonal to `DISABLE_ACTUATION` — both can be active
simultaneously.

### AC23 — Unconditional initialization before guards
`mj_collision()` unconditionally resets `ncon`, clears contacts, and
resets EFC arrays **before** checking disable flags. `mj_fwdActuation()`
unconditionally zeroes `actuator_force` before the `DISABLE_ACTUATION`
guard. `mj_passive()` unconditionally zeroes all passive force vectors
(`qfrc_spring`, `qfrc_damper`, `qfrc_gravcomp`, `qfrc_fluid`,
`qfrc_passive`) before the spring/damper early-return guard.
`mj_fwdConstraint()` unconditionally zeroes `qfrc_constraint` before
checking whether any constraint rows exist. This "init-then-guard"
pattern ensures deterministic state regardless of which flags are set.

### AC24 — Gravcomp routing
A joint with `actuatorgravcomp="true"` routes its gravity compensation
force through `qfrc_actuator` (not `qfrc_passive`). Verify: with
`actuatorgravcomp="false"` (default), gravcomp goes to `qfrc_passive`.
With `actuatorgravcomp="true"`, `qfrc_passive` does not contain gravcomp
for that DOF, but `qfrc_actuator` does. When `DISABLE_GRAVITY` is set,
neither path produces gravcomp.

### AC25 — Spring/damper force separation
`qfrc_spring` and `qfrc_damper` are maintained as separate arrays.
`qfrc_passive = qfrc_spring + qfrc_damper + qfrc_gravcomp + qfrc_fluid`.
When `DISABLE_SPRING` is set, `qfrc_spring` is zero for all DOFs.
When `DISABLE_DAMPER` is set, `qfrc_damper` is zero for all DOFs.
Individual arrays are independently verifiable.

### AC26 — Auto-reset on NaN (S8)
When `qpos` contains NaN, `mj_check_pos()` triggers a warning and resets
to initial state (`qpos0`). After reset, `data.warnings[BadQpos].count > 0`
(warning survives the reset). `data.time == 0.0`. Verify: NaN in `qvel`
triggers `mj_check_vel()` similarly. NaN in `qacc` triggers
`mj_check_acc()`, which also re-runs `mj_forward()` after reset.

### AC27 — Auto-reset threshold
Values exceeding `1e10` (positive or negative) in `qpos`/`qvel`/`qacc`
trigger auto-reset, not just NaN. Matches MuJoCo's `mjMAXVAL = 1e10`.

### AC28 — Autoreset disable
`<flag autoreset="disable"/>` skips the reset but warnings are still
issued. After a NaN event, `data.warnings[BadQpos].count > 0` but
`data.qpos` still contains NaN (no reset occurred).

### AC29 — Ctrl validation
Bad ctrl values (NaN or |ctrl| > 1e10) zero ALL ctrl values. No reset
occurs. Warning `BadCtrl` is issued. Verify: only the ctrl array is
zeroed, not qpos/qvel.

### AC30 — Sleep-aware validation
`mj_check_vel()` and `mj_check_acc()` only scan awake DOFs when sleep
is enabled. A sleeping DOF with NaN velocity does not trigger a reset.

### AC31 — BVH midphase (S9)
Mesh-mesh collision with BVH midphase enabled produces the same contacts
as brute-force all-pairs. A scene with two mesh objects colliding
generates identical contact points, normals, and depths with midphase
enabled vs disabled.

### AC32 — Midphase performance
For a scene with mesh geoms (>100 triangles per mesh), midphase is faster
than brute-force. Benchmark: midphase collision time < 50% of brute-force
time for meshes with >200 triangles.

### AC33 — Midphase disable
`<flag midphase="disable"/>` falls back to brute-force all-pairs for mesh
collisions. Contacts still generated (same result, slower).

### AC34 — Global override (S10)
`<flag override="enable"/>` with `<option o_margin="0.01" o_solref="0.1 1"
o_solimp="0.8 0.9 0.01 0.5 2" o_friction="0.5 0.5 0.01 0.001 0.001"/>`
replaces all per-geom contact parameters. Every contact in the scene uses
the override values. Gap and condim are NOT overridden.

### AC35 — Override broadphase margin
With override enabled, broadphase AABB expansion uses `0.5 * o_margin`
instead of `geom_margin[i]`. For a geom with `margin=0.1` and
`o_margin=0.02`, broadphase uses 0.01 (not 0.1).

### AC36 — Override friction clamping
Override friction values are clamped to `MIN_MU = 1e-5`. Setting
`o_friction="0 0 0 0 0"` produces `[1e-5, 1e-5, 1e-5, 1e-5, 1e-5]`.

### AC37 — Override disabled by default
Without `<flag override="enable"/>`, per-geom parameters are used even
when `o_*` fields are set in `<option>`. The override values are stored
but have no effect.

---

## Files

| File | Change |
|------|--------|
| `sim/L0/core/src/types/enums.rs` | S1: 17 disable + 4 enable constants, `disabled()`/`enabled()` helpers, `is_bad()`, `MAX_VAL`, `Warning` enum, `WarningStat` |
| `sim/L0/mjcf/src/types.rs` | S2: Split `passive` → `spring` + `damper`, add `fwdinv`/`invdiscrete`/`autoreset`, fix docstrings, fix `island` default (S2e) |
| `sim/L0/mjcf/src/parser.rs` | S2: Parse `spring`, `damper`, `passive` (compat), `fwdinv`, `invdiscrete`, `autoreset`, `actuatorgravcomp` on joints, `actuatorgroupdisable` on option, `o_margin`/`o_solref`/`o_solimp`/`o_friction` on option |
| `sim/L0/mjcf/src/builder/mod.rs` | S3: `apply_flags()` replacing single sleep-only block |
| `sim/L0/core/src/types/data.rs` | S4.7-prereq: Add `qfrc_spring`, `qfrc_damper` fields. S8b: Add `warnings: [WarningStat; NUM_WARNINGS]` |
| `sim/L0/core/src/types/model.rs` | S4.2a: Add `jnt_actgravcomp: Vec<bool>`. S7a: Add `disableactuator: u32`, `actuator_group: Vec<i32>`. S10a: Add `o_margin`, `o_solref`, `o_solimp`, `o_friction` |
| `sim/L0/core/src/types/model_init.rs` | S4.2a, S7a, S10a: Initialize new Model fields |
| `sim/L0/core/src/forward/mod.rs` | S4.8, S5.1, S8e: Actuation activation advance guard, energy guards, `mj_check_pos`/`mj_check_vel`/`mj_check_acc` pipeline integration |
| `sim/L0/core/src/forward/velocity.rs` | S4.8: Actuator velocity guard |
| `sim/L0/core/src/sensor/mod.rs` | S4.10: Sensor disable guard (inside each sensor function, no zeroing) |
| `sim/L0/core/src/dynamics/rne.rs` | S4.2: Gravity guard in `mj_rne()` + `mj_gravcomp()` |
| `sim/L0/core/src/energy.rs` | S4.2, S5.1: Gravity guard in `mj_energy_pos()`, energy enable guard |
| `sim/L0/core/src/constraint/assembly.rs` | S4.1, S4.4-S4.6: Contact instantiation guard, equality, frictionloss, limit row guards |
| `sim/L0/core/src/forward/passive.rs` | S4.7-prereq, S4.7, S4.2a, S6: Refactor to separate spring/damper arrays, mj_passive top-level guard, gravcomp routing, fluid sleep filter |
| `sim/L0/core/src/forward/actuation.rs` | S4.2, S4.2a, S4.9, S8d: Actuator-level gravcomp routing, clampctrl guard, ctrl validation |
| `sim/L0/core/src/constraint/mod.rs` | S4.3, S4.11: Constraint assembly guard, warmstart function |
| `sim/L0/core/src/collision/mod.rs` | S4.1, S4.12, S4.13, S9e, S10d: Collision disable guard, filterparent guard, midphase dispatch, broadphase margin override |
| `sim/L0/core/src/collision/narrow.rs` or `mesh_collide.rs` | S9d: Midphase dispatch for mesh-mesh and mesh-primitive pairs |
| `sim/L0/core/src/constraint/impedance.rs` | S4.15: Refsafe guard |
| `sim/L0/core/src/constraint/contact_params.rs` (new) | S10c: `assign_margin`, `assign_ref`, `assign_imp`, `assign_friction` helpers |
| `sim/L0/core/src/forward/check.rs` (new) | S8c: `mj_check_pos`, `mj_check_vel`, `mj_check_acc`, `mj_reset_data` |

---

## Risk

- **Breaking:** Splitting `MjcfFlag.passive` into `spring`/`damper` breaks
  any code that reads `flag.passive`. Grep for `flag.passive` and update all
  call sites.
- **Breaking:** Refactoring `mj_fwd_passive()` to write spring/damper into
  separate arrays (S4.7-prereq) touches the hot path for every joint, tendon,
  and flex. Must verify force values are identical before and after refactor.
- **Energy regression:** Gating energy behind `ENABLE_ENERGY` means
  `data.energy_potential` and `data.energy_kinetic` will be 0.0 by default
  (matching MuJoCo, but differs from current always-compute behavior). Any
  test that asserts on energy values without setting `ENABLE_ENERGY` will
  fail.
- **Island default fix:** The existing `MjcfFlag { island: false }` default
  is wrong — MuJoCo defaults to `disableflags = 0` (island enabled). Changing
  `island` to `true` may alter behavior if the hardcoded `DISABLE_ISLAND`
  constant was being set elsewhere. Audit all `DISABLE_ISLAND` usage sites
  (currently only `island/mod.rs:37`). See S2e.
- **Auto-reset behavior change (S8):** Adding `mj_check_pos`/`vel`/`acc`
  to the step pipeline means simulations that previously produced NaN will
  now silently reset. This matches MuJoCo but changes CortenForge behavior.
  Users relying on NaN propagation for debugging should set
  `<flag autoreset="disable"/>`.
- **BVH midphase integration (S9):** Mesh collision results must be
  identical with and without midphase. The BVH is a pure optimization —
  it must not change which contacts are generated, only the set of
  triangle pairs tested. Run mesh collision tests with midphase enabled
  and disabled and compare results.
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
