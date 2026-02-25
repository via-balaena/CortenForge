# §41 — Runtime Flag Wiring (+ DT-61)

**Status:** Not started
**Phase:** Roadmap Phase 2 — Runtime Flag Wiring
**Effort:** L (expanded to subsume DT-93, DT-94, DT-95)
**Prerequisites:** None
**Subsumes:** DT-60 (jnt_actgravcomp routing, pulled from Phase 5),
DT-61 (DISABLE_GRAVITY flag), DT-93 (auto-reset on NaN),
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

Additionally, this spec subsumes four features that are required for
complete flag guard coverage (previously deferred as DT-60, DT-93,
DT-94, DT-95):
- **S4.2a:** `jnt_actgravcomp` routing (DT-60, pulled from Phase 5 —
  required as prerequisite for the `DISABLE_GRAVITY` actuation-side guard)
- **S8:** Auto-reset on NaN/divergence (`DISABLE_AUTORESET` guard)
- **S9:** BVH midphase integration (`DISABLE_MIDPHASE` guard)
- **S10:** Global contact parameter override (`ENABLE_OVERRIDE` guard)

---

## MuJoCo Reference

**Verified against:** MuJoCo 3.4.0 and 3.5.0 source (`mjmodel.h`,
`engine_forward.c`, `engine_passive.c`, `engine_init.c`,
`engine_sensor.c`, `engine_support.c`, `engine_core_constraint.c`).
Enums are identical in both versions — no flags added between 3.4.0
and 3.5.0. Key behavioral details re-verified against `main` branch
(Feb 2026): `mj_passive()` spring+damper combined early-return guard,
`mj_fwdActuation()` unconditional `actuator_force` zeroing,
`mj_sensorXxx()` no-zero-on-disable, `mj_checkAcc()` post-reset
`mj_forward()` call, `mj_actuatorDisabled()` group-range check.

**Version history** (relevant for backward compatibility):

| Version | `mjNDISABLE` | `mjNENABLE` | Key changes |
|---------|-------------|-------------|-------------|
| 3.3.2–3.3.5 | 17 | 6 | Single `mjDSBL_PASSIVE` (bit 5). `mjENBL_ISLAND` (enable, bit 5). No `SLEEP`. |
| 3.3.6 | 19 | 5 | `PASSIVE` split → `SPRING` (bit 5) + `DAMPER` (bit 6). `passive` XML attr removed. `ISLAND` moved from enable to disable (bit 18). All disable bits 7–17 shifted +1. `SLEEP` not yet added. |
| 3.4.0 | 19 | 6 | `mjENBL_SLEEP` added (enable, bit 5) — filled hole left by `ISLAND`'s departure. |
| 3.5.0 | 19 | 6 | Identical to 3.4.0. |

Note: enable bits 0–4 (`OVERRIDE`, `ENERGY`, `FWDINV`, `INVDISCRETE`,
`MULTICCD`) are unchanged across all versions. Only bit 5 changed occupants:
`ISLAND` (3.3.2–3.3.5) → empty (3.3.6) → `SLEEP` (3.4.0+).

This history explains why `passive` was dropped in S2a — it was a real
MJCF attribute in MuJoCo ≤ 3.3.5, removed in 3.3.6. CortenForge targets
3.4.0+ and silently ignores it (matching MuJoCo's behavior).

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

Also add a `disabled` helper for readable runtime checks.

**Module placement:** Create a dedicated `types/flags.rs` module for all
flag-related helpers. `enums.rs` currently has zero dependencies on `Model`
and should stay that way. The `flags.rs` module depends on both `enums`
(for constants) and `model` (for `Model`), keeping concerns separated.

Flag constants remain in `enums.rs` (they're pure values with no
dependencies). Helpers that take `&Model` go in `flags.rs`.

**File:** `types/flags.rs` (new)

```rust
/// Returns true if the given disable flag is set on the model.
#[inline]
pub fn disabled(model: &Model, flag: u32) -> bool {
    debug_assert!(
        flag.is_power_of_two() && flag.trailing_zeros() <= 18,
        "disabled() called with non-disable flag: {flag:#x}"
    );
    model.disableflags & flag != 0
}

/// Returns true if the given enable flag is set on the model.
#[inline]
pub fn enabled(model: &Model, flag: u32) -> bool {
    debug_assert!(
        flag.is_power_of_two() && flag.trailing_zeros() <= 5,
        "enabled() called with non-enable flag: {flag:#x}"
    );
    model.enableflags & flag != 0
}
```

The `debug_assert!` guards catch namespace misuse — e.g.,
`disabled(model, ENABLE_SLEEP)` — in tests and debug builds without
runtime cost in release. Each flag constant is a single bit; the assert
verifies the bit position is within the valid range (0–18 for disable,
0–5 for enable).

**Design note (bare `u32` constants vs `bitflags`):** The spec uses bare
`u32` constants rather than the `bitflags` crate or a newtype. This
sacrifices exhaustiveness checking and debug formatting but maximizes
MuJoCo source fidelity — the constants map 1:1 to `mjtDisableBit` /
`mjtEnableBit` values, making cross-reference trivial. If a `bitflags`
migration is desired later, the constants are the source of truth and the
migration is mechanical.

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
attributes are skipped).

**Decision: drop `passive` entirely.** CortenForge is pre-v1.0 — there
are no downstream users to break, and no MJCF assets in
`sim/L0/tests/assets/` use `<flag passive=...>` (verified by grep). If
`passive` is encountered in an MJCF file, it will be silently ignored
(unrecognized attributes are already skipped by the parser), matching
MuJoCo's current behavior. No shim, no deprecation warning, no DT-98.
This follows the project principle: "Prefer breaking changes that fix
the architecture over non-breaking hacks that preserve a bad interface."

**DT-98 is retired** — the backward-compatibility shim it tracked is no
longer needed. Remove from `future_work_10j.md`.

#### S2b. Add missing flag fields

```rust
pub fwdinv: bool,       // default false (enable flag)
pub invdiscrete: bool,  // default false (enable flag)
pub autoreset: bool,    // default true  (disable flag — enabled by default)
```

#### S2c. Update parser

In `parse_flag_attrs()`:

- Remove `flag.passive = parse_flag(e, "passive", ...)`. The `passive`
  attribute no longer exists (S2a). If an MJCF file contains it, the
  parser's unrecognized-attribute handling silently skips it (matching
  MuJoCo's behavior). Replace with the two independent attributes:
  ```rust
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

**Verified:** No test MJCF files in `sim/L0/tests/assets/` contain
`island=` in any `<flag>` element (only the dm_control XML schema
definition references it). Programmatic `DISABLE_ISLAND` usage in tests
(`sleeping.rs` T34, T54, T88) sets the flag explicitly via
`model.disableflags |= DISABLE_ISLAND`, which is unaffected by the
default change.

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

#### S3b. Runtime warnings for stub-only flags

Five flags are wired through the parser and builder but have **no runtime
guard site** because their subsystems do not exist yet:

| Flag | Blocked by | Subsystem |
|------|------------|-----------|
| `ENABLE_FWDINV` | §52 | Inverse dynamics |
| `ENABLE_INVDISCRETE` | §52 | Discrete inverse dynamics |
| `ENABLE_MULTICCD` | §50 | Multi-point CCD |
| `DISABLE_NATIVECCD` | §50 | Native CCD dispatch |
| ~~`DISABLE_MIDPHASE`~~ | ~~DT-99 (S9-full)~~ **Done** | BVH midphase fully wired — `use_bvh` guard in `collide_with_mesh()` |

A user who sets these flags in MJCF gets silent no-op behavior — the flag
is stored but nothing checks it. This is a conformance trap: the XML
appears to work, but the physics are unchanged.

**Fix:** In `apply_flags()`, after wiring all flags, emit a
`tracing::warn!` for each stub-only flag that is set to its non-default
value:

```rust
// Stub-only flags: subsystem not yet implemented.
// Warn so users don't silently get no-op behavior.
if flag.fwdinv {
    tracing::warn!(
        "ENABLE_FWDINV set but inverse dynamics (§52) not implemented — flag has no effect"
    );
}
if flag.invdiscrete {
    tracing::warn!(
        "ENABLE_INVDISCRETE set but inverse dynamics (§52) not implemented — flag has no effect"
    );
}
if flag.multiccd {
    tracing::warn!(
        "ENABLE_MULTICCD set but CCD (§50) not implemented — flag has no effect"
    );
}
if !flag.nativeccd {  // default is true (enabled); warn when user disables it
    tracing::warn!(
        "DISABLE_NATIVECCD set but CCD (§50) not implemented — flag has no effect"
    );
}
// Note: DISABLE_MIDPHASE is not warned here — S9-full (DT-99) is complete.
// The flag gates BVH vs brute-force in collide_with_mesh() and is fully functional.
```

These warnings are removed when the corresponding subsystem ships. The
`tracing::warn!` fires once per model load (not per step), so there is
no performance concern.

**Cross-reference requirement:** When §50 (CCD) and §52 (inverse
dynamics) are specced, their specs **must** include a section titled
"S41 stub-only flag activation" listing which flags gain guard sites:
- §50 must wire: `ENABLE_MULTICCD`, `DISABLE_NATIVECCD`
- §52 must wire: `ENABLE_FWDINV`, `ENABLE_INVDISCRETE`

This ensures the connection is bidirectional — not only documented here
(where someone implementing §50 might not look) but also in the spec
they will actually be reading. Until those specs exist, this paragraph
is the canonical tracking point. When a stub-only flag gains its guard
site, remove the corresponding `tracing::warn!` in the same commit.

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
    || model.gravity.norm() < MIN_VAL
    || disabled(model, DISABLE_GRAVITY)
{
    return false;
}
```

Note: MuJoCo uses `mju_norm3(m->opt.gravity) < mjMINVAL` where
`mjMINVAL = 1e-15`. Define `MIN_VAL` in `types/validation.rs` (alongside
`MAX_VAL`) and use it here. Do NOT use exact `== 0.0` — floating-point
arithmetic can produce near-zero gravity vectors (e.g., after
interpolation or unit conversion) that would bypass this guard.

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
    && model.gravity.norm() >= MIN_VAL
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
    && model.gravity.norm() >= MIN_VAL
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

**Guard site — `mj_fwd_constraint()` in `constraint/mod.rs`:**

The `DISABLE_CONSTRAINT` flag gates `assemble_unified_constraints()`
(CortenForge's `mj_makeConstraint()` equivalent). The guard goes inside
`mj_fwd_constraint()`, **before** the assembly call:

```rust
// Inside mj_fwd_constraint():
data.qfrc_constraint[..model.nv].fill(0.0);

if disabled(model, DISABLE_CONSTRAINT) {
    // Skip assembly entirely — no equality, limit, contact, or
    // friction loss rows. nefc stays 0, falls through to the
    // nefc == 0 early exit below.
} else {
    assemble_unified_constraints(model, data, qacc_for_assembly);
}
```

This is the **primary** gate. `mj_collision()` also independently gates
on `DISABLE_CONSTRAINT` (S4.1), ensuring `ncon == 0` upstream. The
assembly-level guard here is defense-in-depth: even if contacts somehow
survived (e.g., from a previous step), no constraint rows are assembled.

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

**Causal chain when `DISABLE_CONSTRAINT` is set:** `mj_collision()` returns
early (S4.1) → `ncon` stays 0. `assemble_unified_constraints()` is
skipped (S4.3 guard above) → `nefc` stays 0. `mj_fwdConstraint()` hits
the `nefc == 0` early exit shown above → `qacc = qacc_smooth`, return.
The `DISABLE_CONSTRAINT` check gates assembly (upstream); the solver
reacts to the resulting empty constraint set (downstream) via the
existing `nefc == 0` path. No additional flag check is needed inside
the solver dispatch — the two-level gating (collision + assembly) is
sufficient.

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

**`PassiveForceVisitor` struct update:** The visitor needs two new fields to
carry the flag state into per-joint iteration:

```rust
struct PassiveForceVisitor<'a> {
    model: &'a Model,
    data: &'a mut Data,
    implicit_mode: bool,
    sleep_enabled: bool,
    has_spring: bool,  // NEW — !disabled(model, DISABLE_SPRING)
    has_damper: bool,  // NEW — !disabled(model, DISABLE_DAMPER)
}
```

These are computed once in `mj_fwd_passive()` (after the `nv == 0` and
spring+damper guards) and passed into the visitor, avoiding repeated
`disabled()` calls per joint.

**Tendon and flex passive forces** also write spring/damper components
separately. The tendon loop (lines 366–430 in `passive.rs`) and flex
passive computation must be updated to target `qfrc_spring`/`qfrc_damper`
instead of `qfrc_passive` directly.

**Sleep-filtered aggregation:** When sleep filtering is active, aggregation
uses indexed iteration over `dof_awake_ind` (matching S4.7a's zeroing
pattern).

##### S4.7a Top-level `mj_passive()` early return

**CortenForge safety guard (not in MuJoCo):** MuJoCo has NO explicit
`nv == 0` guard in `mj_passive()` — when `nv == 0`, `mju_zero(ptr, 0)`
is a harmless no-op and the function proceeds through its zeroing and
guard logic without issue. However, in Rust, iterating over empty slices
is safe but the control flow is clearer with an explicit guard. Add this
as a CortenForge-specific early return:

```rust
// CortenForge extension — not in MuJoCo (which relies on mju_zero(ptr, 0)
// being a no-op). Explicit guard for clarity and to skip unnecessary
// sleep-filter computation when there are no DOFs.
if model.nv == 0 {
    return;
}
```

**Why only `mj_passive()`:** Other `nv`-iterating functions
(`mj_fwd_actuation()`, `mj_fwd_constraint()`, check functions) do not
need this guard because their early-return conditions subsume `nv == 0`:
`mj_fwd_actuation()` guards on `nu == 0` (no actuators implies no DOFs
in practice), `mj_fwd_constraint()` guards on `nefc == 0`, and the check
functions' `0..nv` loops are trivially empty. `mj_passive()` is unique
because its first non-trivial operation is computing the sleep filter
(`enabled(model, ENABLE_SLEEP) && data.nv_awake < model.nv`), which
touches `nv_awake` — an `nv == 0` guard makes this path obviously
correct without reasoning about the sleep filter's behavior on empty
models. This is a readability choice, not a safety requirement.

After the `nv == 0` guard, there is **one** MuJoCo-matching early-return
guard: the spring+damper guard, which returns *after* unconditional
zeroing of all passive force vectors. When both `DISABLE_SPRING` AND
`DISABLE_DAMPER` are set, force vectors are zeroed and then the function
exits — skipping all sub-functions but leaving arrays in a clean state.

MuJoCo **unconditionally zeros** all passive force vectors (`qfrc_spring`,
`qfrc_damper`, `qfrc_gravcomp`, `qfrc_fluid`, `qfrc_passive`) for awake
DOFs before the spring+damper guard. This zeroing runs regardless of
whether the function continues past the guard or returns early.

When **both** `DISABLE_SPRING` AND `DISABLE_DAMPER` are set,
`mj_passive()` returns immediately — skipping ALL passive sub-functions:

```rust
// CortenForge nv == 0 guard already handled above.
//
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

> **User-facing note:** When both `DISABLE_SPRING` and `DISABLE_DAMPER` are
> set, fluid drag forces and gravity compensation are also suppressed. This
> matches MuJoCo's hierarchical guard architecture where `mj_passive()` gates
> all sub-functions (spring, damper, gravcomp, fluid) behind its top-level
> flag check. Document this interaction in the user-facing API docs for
> `<flag>` when those are written.

##### S4.7b Component-level guards in `mj_springdamper()`

> **Naming mapping:** MuJoCo's `mj_springdamper()` corresponds to
> CortenForge's `PassiveForceVisitor` (joint spring/damper) plus the
> inline tendon and flex spring/damper loops in `mj_fwd_passive()`.
> The refactor should wrap these into a `mj_springdamper(model, data)`
> function for MuJoCo naming conformance, or retain the visitor pattern
> with a comment documenting the equivalence.

Within `mj_springdamper()`, spring and damper forces are gated independently:

```rust
let has_spring = !disabled(model, DISABLE_SPRING);
let has_damper = !disabled(model, DISABLE_DAMPER);

// In passive force computation for each joint:
// Write to separate arrays (S4.7-prereq), NOT directly to qfrc_passive.
// Aggregation into qfrc_passive happens in S4.7e.
if has_spring {
    qfrc_spring[dof] += stiffness * displacement;
}
if has_damper {
    qfrc_damper[dof] += damping * velocity;
}
```

**MuJoCo implementation detail:** MuJoCo uses two gating patterns
interchangeably:
1. **Conditional blocks:** `if (has_spring) { ... }` for joint/DOF springs
2. **Multiplication-based zeroing:** `stiffness = m->tendon_stiffness[i] * has_spring`
   — multiplying the coefficient by the boolean (0 or 1) to zero it when
   disabled. This is equivalent but avoids branching.

Either approach is acceptable for CortenForge — the conditional block
pattern is clearer.

Same pattern applies to all 5 spring/damper write sites. Here is the
exhaustive redirection inventory (current write target → new target):

**Site 1 — Joint 1-DOF (hinge/slide) in `PassiveForceVisitor::visit_1dof_joint()`
(`passive.rs:654-673`):**
```rust
// BEFORE (writes both to qfrc_passive):
self.data.qfrc_passive[dof_adr] -= stiffness * (q - springref);  // spring
self.data.qfrc_passive[dof_adr] -= damping * qvel;               // damper

// AFTER (writes to separate arrays, gated by visitor fields):
if self.has_spring {
    self.data.qfrc_spring[dof_adr] -= stiffness * (q - springref);
}
if self.has_damper {
    self.data.qfrc_damper[dof_adr] -= damping * qvel;
}
```

**Site 2 — Joint multi-DOF (ball/free) in `PassiveForceVisitor::visit_multi_dof_joint()`
(`passive.rs:679-693`):**
```rust
// BEFORE:
self.data.qfrc_passive[dof_idx] -= dof_damping * qvel;  // damper only

// AFTER:
if self.has_damper {
    self.data.qfrc_damper[dof_idx] -= dof_damping * qvel;
}
// Note: ball/free joints have no spring (no stiffness field).
```

**Site 3 — Tendon spring/damper loop (`passive.rs:366-414`):**
```rust
// BEFORE: ten_force combines spring + damper, applied via J^T to qfrc_passive
// AFTER: Separate spring and damper force scalars, apply each independently:
let mut spring_force = 0.0;
let mut damper_force = 0.0;
if has_spring && k > 0.0 {
    // deadband spring logic unchanged
    spring_force = ...;
}
if has_damper && b > 0.0 {
    damper_force = -b * velocity;
}
data.ten_force[t] = spring_force + damper_force;  // diagnostic — unchanged semantics
if !implicit_mode {
    if spring_force != 0.0 {
        apply_tendon_force(model, &data.ten_J[t], ..., spring_force, &mut data.qfrc_spring);
    }
    if damper_force != 0.0 {
        apply_tendon_force(model, &data.ten_J[t], ..., damper_force, &mut data.qfrc_damper);
    }
}
```

**Site 4 — Flex vertex damping (`passive.rs:417-431`):**
```rust
// BEFORE:
data.qfrc_passive[dof_base + k] -= damp * data.qvel[dof_base + k];

// AFTER:
if has_damper {
    data.qfrc_damper[dof_base + k] -= damp * data.qvel[dof_base + k];
}
```

**Site 5 — Flex edge spring/damper + bending (`passive.rs:433-618`):**
```rust
// Edge spring (line ~475): qfrc_passive[dof+k] += ... → qfrc_spring[dof+k] += ...
// Edge damper (line ~490): qfrc_passive[dof+k] += ... → qfrc_damper[dof+k] += ...
// Bending spring (line ~564): qfrc_passive[dof+ax] += grad[ax] * fm
//   → Split fm into spring_mag and damper_mag components:
//     qfrc_spring[dof+ax] += grad[ax] * fm_spring  (gated on has_spring)
//     qfrc_damper[dof+ax] += grad[ax] * fm_damper  (gated on has_damper)
```

The bending force split (Site 5) is the trickiest — `force_mag` is
currently `spring_mag + damper_mag` with a shared per-vertex stability
clamp. After the split, clamp each component independently (the clamp
is per-vertex, so two passes through the vertex loop — or compute both
clamped magnitudes before the J^T application). Either approach preserves
the stability guarantee.

**Prescribed approach:** Compute both `spring_mag` and `damper_mag`
separately, clamp each independently using the same per-vertex stability
threshold, then apply each to `qfrc_spring` / `qfrc_damper` respectively.
Do NOT clamp the sum and then split — this changes the clamping
semantics (the sum-clamp limits total force, independent clamps limit
each component). Independent clamping is correct because each force
component can be independently unstable, and the spring/damper flags
gate them independently.

**Verification note for S4.7-prereq flex bending:** The intermediate
verification (lines 2674-2683) specifies machine-epsilon tolerance for
`qfrc_spring + qfrc_damper + qfrc_gravcomp + qfrc_fluid == qfrc_passive`.
For flex bending specifically, independent per-component clamping may
produce results that differ from the original sum-then-clamp by more
than machine epsilon when the clamp was previously active on the
combined force. This is an **intentional semantic change** (each component
clamped independently is more correct), not a regression. If the
verification fails on flex bending DOFs, compare the individual
`spring_mag` and `damper_mag` values to confirm the clamp is firing on
both branches correctly. Accept the diff if the only cause is
independent vs combined clamping.

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

**Interaction with S4.7a:** If both `DISABLE_SPRING` and `DISABLE_DAMPER`
are set, `mj_passive()` returns early before `mj_contactPassive()` is
reached — so the S4.7d guard never fires. If only `DISABLE_CONTACT` is
set but spring/damper are enabled, `mj_passive()` does NOT exit early,
but `mj_contactPassive()` returns early via its own guard. Both paths
are correct — the guards are layered, not redundant.

> **Current state:** `mj_contact_passive()` does not exist in CortenForge
> yet. MuJoCo's `mj_contactPassive()` computes viscous contact damping
> forces. Implementation is tracked as **DT-101** (see `future_work_10c.md`).
> When implemented, it must have the `DISABLE_CONTACT` early-return guard
> shown above. Until then, the guard site is a no-op — there is no code
> to gate.

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

// Aggregation: qfrc_passive = qfrc_spring + qfrc_damper
//              + (if has_gravcomp) qfrc_gravcomp
//              + (if has_fluid)    qfrc_fluid
// Sleep-filtered: iterate only over awake DOFs (matching S4.7a zeroing).
let sleep_filter = enabled(model, ENABLE_SLEEP)
    && data.nv_awake < model.nv;

if sleep_filter {
    for &dof in &data.dof_awake_ind[..data.nv_awake] {
        data.qfrc_passive[dof] = data.qfrc_spring[dof] + data.qfrc_damper[dof];
    }
    if has_gravcomp {
        for &dof in &data.dof_awake_ind[..data.nv_awake] {
            data.qfrc_passive[dof] += data.qfrc_gravcomp[dof];
        }
    }
    if has_fluid {
        for &dof in &data.dof_awake_ind[..data.nv_awake] {
            data.qfrc_passive[dof] += data.qfrc_fluid[dof];
        }
    }
} else {
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
}
```

**Implementation note on boolean returns:** `mj_springdamper()` has no
return value — it unconditionally writes into `qfrc_spring` and
`qfrc_damper`, so these are always aggregated. In contrast,
`mj_gravcomp()` returns `true` only when `ngravcomp > 0` (at least one
body has `gravcomp != 0`), and `mj_fluid()` returns `true` only when at
least one body has non-zero fluid interaction parameters. The aggregation
loop uses these booleans to skip the `+=` pass entirely when no forces
were produced, which matters for performance in large models.

After aggregation, the following additions to `qfrc_passive` happen in
order. Each writes additively (`+=`) to the already-aggregated vector:

1. **DT-101 (`mj_contactPassive()`)** — viscous contact damping forces.
   Not yet implemented; when it is, it must be called **HERE** (after
   aggregation, not before). MuJoCo calls `mj_contactPassive()` after
   the `spring + damper + gravcomp + fluid` aggregation, and it adds
   its forces to the already-computed `qfrc_passive`. Placing it before
   aggregation would silently lose its output (aggregation overwrites
   `qfrc_passive` with `= spring + damper + ...`). See S4.7d for the
   `DISABLE_CONTACT` guard it requires.

2. **DT-79 (user callback `mjcb_passive`)** — user-defined passive
   forces, plus plugin dispatch (`mjPLUGIN_PASSIVE`). Both can modify
   `qfrc_passive` further. Out of scope for §41.

Both insertion points are after aggregation and before `qfrc_passive`
is consumed downstream.

**Mandatory code comment:** The implemented `mj_fwd_passive()` must
include a placeholder comment at the post-aggregation insertion point:

```rust
// ── Post-aggregation additions to qfrc_passive ──
// DT-101: mj_contactPassive() goes HERE (after aggregation, not before).
//   Placing it before aggregation would lose its output — aggregation
//   overwrites qfrc_passive with `= spring + damper + ...`.
//   See §41 S4.7e for rationale. Gated on DISABLE_CONTACT (S4.7d).
// DT-79: User callback (mjcb_passive) + plugin dispatch goes HERE.
```

This prevents a future implementer from placing `mj_contactPassive()`
at the wrong point in the pipeline.

#### S4.8 `DISABLE_ACTUATION` — skip actuator forces

MuJoCo gates actuation at **four** locations:

**Site 1 — `mj_fwd_actuation()`:**
**File:** `forward/actuation.rs`

Guard the actuation force computation. MuJoCo has **two distinct arrays**:
- `actuator_force` (length `nu`) — per-actuator scalar forces
- `qfrc_actuator` (length `nv`) — joint-space force vector (sum of all
  actuator contributions mapped through the moment arm)

MuJoCo unconditionally zeroes `actuator_force` at the top of the function
before checking disable flags. Then, when disabled, it zeroes
`qfrc_actuator` and returns early:

```rust
// Unconditional — zero per-actuator forces regardless of disable flags.
// actuator_force is length nu (one scalar per actuator).
data.actuator_force[..model.nu].fill(0.0);

if model.nu == 0 || disabled(model, DISABLE_ACTUATION) {
    // qfrc_actuator is length nv (joint-space force vector).
    data.qfrc_actuator[..model.nv].fill(0.0);
    return;
}
```

Both arrays must exist in CortenForge. `actuator_force` is the per-actuator
output (used by sensors, diagnostics); `qfrc_actuator` is the joint-space
projection consumed by the integrator.

~~**Site 2 — actuator velocity computation:**~~ **REMOVED.**
**Verified against MuJoCo `engine_forward.c`:** `actuator_velocity` is
computed in `mj_fwdVelocity()` (velocity-dependent stage), NOT in
`mj_fwdActuation()`. It is `actuator_moment^T * qvel` — pure transmission
kinematics. MuJoCo computes it **unconditionally**, regardless of
`DISABLE_ACTUATION`. When actuation is disabled, `actuator_velocity`
remains valid (it reflects joint velocities projected through the
transmission, not actuator forces). Do NOT gate it.

**Current state (already correct):** CortenForge computes
`actuator_velocity` inside `mj_actuator_length()` (`forward/actuation.rs`),
but this function is already called from the velocity stage
(`forward/mod.rs:185`). The pipeline ordering is correct — velocity is
computed after velocity-FK, unconditionally, regardless of
`DISABLE_ACTUATION`. No pipeline-ordering fix is needed.

> **Optional organizational refactor — splitting `mj_actuator_length()`:**
> In MuJoCo, `mj_actuator_length()` computes both `actuator_length` and
> `actuator_velocity` in a single per-actuator loop. CortenForge matches
> this. For naming clarity, a new `mj_actuator_velocity()` function could
> be extracted containing just the velocity dispatch (`match
> model.actuator_trntype[i]`), leaving `mj_actuator_length()` computing
> only lengths. This is a **naming/organizational improvement**, not a
> correctness fix — the current code is functionally correct.
>
> If split: call `mj_actuator_velocity()` from `mj_fwd_velocity()`. The
> transmission-type match arm structure is trivial model reads —
> duplicating it across the two functions is fine (no meaningful
> computation is shared between the length and velocity paths).

> **Commit discipline:** If the split is done, it should be its own
> commit within §41 (before the flag guard commits), with the sim test
> suite run before and after to verify no behavioral change.

**Site 3 — Activation integration in `Data::integrate()`:**
**File:** `integrate/mod.rs` — lines 37–47

CortenForge's activation integration loop lives in `Data::integrate()`
(called from Euler/implicit integrators). It calls `mj_next_activation()`
per actuator. Add per-actuator disable gating to match MuJoCo's
`mj_actuatorDisabled()` check:

```rust
// In integrate/mod.rs, the activation integration loop:
for i in 0..model.nu {
    let act_adr = model.actuator_act_adr[i];
    let act_num = model.actuator_act_num[i];
    // Gate: blanket DISABLE_ACTUATION or per-group disabling
    let is_disabled = disabled(model, DISABLE_ACTUATION)
                   || actuator_disabled(model, i);
    for k in 0..act_num {
        let j = act_adr + k;
        let act_dot_val = if is_disabled { 0.0 } else { self.act_dot[j] };
        self.act[j] = mj_next_activation(model, i, self.act[j], act_dot_val);
    }
}
```

Disabled actuators get `act_dot = 0`, freezing their activation state
without zeroing it. The same gating must also be added to the RK4 path
in `integrate/rk4.rs` (lines 84–186), which has its own per-stage
activation integration loop.

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

**File:** `constraint/mod.rs` — called from inside `mj_fwd_constraint()`

MuJoCo implements warmstart as a standalone `warmstart()` function called
from inside `mj_fwdConstraint()`, not inside individual solvers. Match
this — `warmstart()` is called at the top of `mj_fwd_constraint()`,
after `assemble_unified_constraints()` and the `nefc == 0` early exit,
but **before** solver dispatch. It is NOT a separate call from
`forward_core()`:

```rust
// Inside mj_fwd_constraint():
assemble_unified_constraints(model, data, qacc_for_assembly);
if data.nefc == 0 { ... return; }
let warm = warmstart(model, data);
// ... solver dispatch uses `warm` ...
```

```rust
fn warmstart(model: &Model, data: &mut Data) -> bool {
    if disabled(model, DISABLE_WARMSTART) {
        // Cold start: unconstrained accelerations, zero constraint forces.
        // Bypasses the smart comparison entirely — each solver starts from
        // qacc_smooth + efc_force = 0.
        data.qacc.copy_from(&data.qacc_smooth);
        data.efc_force.fill(0.0);
        return false;  // cold start — solvers skip cost comparison
    }

    // Warm start: populate qacc from previous timestep's solution.
    // This is the actual warm data loading — MuJoCo's warmstart() in
    // engine_forward.c does this work before solver dispatch.
    data.qacc.copy_from(&data.qacc_warmstart);
    // efc_force is NOT populated here — each solver's smart comparison
    // computes it from qacc_warmstart via its own mechanism:
    //
    //   PGS (pgs.rs:84-121):
    //     Calls classify_constraint_states() with qacc_warmstart to compute
    //     warmstart forces, then evaluates dual cost = ½·f^T·AR·f + f^T·b.
    //     If dual_cost_warm < 0.0, keeps warmstart forces; otherwise falls
    //     back to zero forces (cold start).
    //
    //   Newton (newton.rs:78-96):
    //     Calls evaluate_cost_at() with qacc_warmstart and qacc_smooth.
    //     Compares total Gauss cost (acceleration-based). Selects whichever
    //     starting point has lower cost.
    //
    //   CG (cg.rs:57-79):
    //     Same evaluate_cost_at() pattern as Newton — compares warmstart
    //     vs smooth cost and selects the better starting point.
    //
    // Island handling: unconstrained DOFs (index >= nidof) always get
    // qacc_smooth regardless of warmstart outcome.

    true  // warm data loaded — solvers should run cost comparison
}
```

**Function contract:**
- `warmstart()` returns `bool`: `true` = warm data loaded (solvers should
  run `evaluate_cost_at()` / `classify_constraint_states()` comparison),
  `false` = cold start (solvers skip comparison, start from
  `qacc_smooth` + `efc_force = 0`).
- When returning `true`, `warmstart()` has already copied
  `qacc_warmstart` → `qacc`. Each solver then evaluates whether this
  warmstart point is better than `qacc_smooth` and selects the better
  starting point. The warm data loading is in THIS function, not
  deferred to the solvers.
- When returning `false`, `warmstart()` has already set `qacc =
  qacc_smooth` and `efc_force = 0`. Solvers skip their cost comparison
  entirely — there is nothing to compare against.
- Each solver receives this `bool` and conditionally skips its warmstart
  comparison block when `false`.
- Call site: `let warm = warmstart(model, data); solver.solve(model, data, warm);`

Key detail: when disabled, `qacc` is set to `qacc_smooth` (unconstrained
accelerations) AND `efc_force` is zeroed. Both assignments are required.

**Relationship between `warmstart()` and per-solver smart selection:**
The standalone `warmstart()` function and the per-solver warmstart
comparison are **two levels of the same mechanism**, not independent
alternatives:

1. `warmstart()` (this function) runs **before** solver dispatch. It
   handles data loading: either cold start (`qacc = qacc_smooth`,
   `efc_force = 0`) or warm start (`qacc = qacc_warmstart`).
2. Each solver's smart comparison (PGS dual cost, Newton/CG
   `evaluate_cost_at()`) runs **after** `warmstart()`. It evaluates
   whether the loaded warm data is better than `qacc_smooth` and
   selects the better starting point. This comparison is skipped
   when `warmstart()` returns `false`.

**`qacc_warmstart` saving is unconditional:** The end-of-step save
(`data.qacc_warmstart.copy_from(&data.qacc)` in `forward/mod.rs:97`)
runs regardless of `DISABLE_WARMSTART`. The flag gates **consumption**
of the warmstart cache, not its **population**. This matches MuJoCo:
even with warmstart disabled, the cache is maintained so re-enabling the
flag mid-simulation has immediate effect.

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

This requires threading `model` (or at minimum `timestep` and
`disableflags`) into `compute_kbip()`. Pass `model` directly — do not
use an `Option<f64>` for timestep or any other workaround. The function
already operates on model-derived parameters; adding `&Model` is the
clean fix.

**Signature change:**

```rust
// Before:
pub fn compute_kbip(solref: [f64; 2], solimp: [f64; 5]) -> (f64, f64)
// After:
pub fn compute_kbip(model: &Model, solref: [f64; 2], solimp: [f64; 5]) -> (f64, f64)
```

**Call sites to update (exhaustive — single call site):**
- `constraint/assembly.rs:223` — `let (k, b) = compute_kbip(sr, si);`
  → `let (k, b) = compute_kbip(model, sr, si);`
  (import at `assembly.rs:15` unchanged — function name stays the same)

No other call sites exist in the codebase.

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
be updated to match. **Tracked as DT-96** (post-v1.0 — only matters once
plugins or energy-dependent sensors exist).

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

### S6. Fluid sleep filtering — already implemented (§40c)

**Status:** Already done. §40c implemented body-level sleep filtering in
`mj_fluid()` (lines 280–320 of `forward/passive.rs`). The loop uses
`data.nbody_awake` / `data.body_awake_ind` indirection when sleep is
active, matching MuJoCo's pattern.

**No further work required in S6.** The only interaction with §41 is that
S4.7a's top-level `mj_passive()` early return (when both `DISABLE_SPRING`
AND `DISABLE_DAMPER` are set) also skips `mj_fluid()` — this is the
correct behavior (MuJoCo's `mj_fluid()` is not independently gated on
spring/damper flags; it's only skipped via the top-level early return).

### S7. Per-group actuator disabling (`disableactuator` bitmask)

MuJoCo has a third bitfield `opt.disableactuator` (an `int`, 31 usable
bits) that disables actuators by group ID. It is checked via
`mj_actuatorDisabled()` inside `mj_fwdActuation()` and the activation
integration loop (CortenForge: `Data::integrate()` in `integrate/mod.rs`):

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

Add to `types/flags.rs` (alongside `disabled()`/`enabled()`):

```rust
/// Returns true if actuator `i` is disabled by group membership.
/// Groups outside 0–30 are never disabled (matches MuJoCo).
#[inline]
pub fn actuator_disabled(model: &Model, i: usize) -> bool {
    let group = model.actuator_group[i];
    // Range check first — only groups 0–30 map to bitmask bits.
    // Cast to u32 only after confirming non-negative.
    if !(0..=30).contains(&group) { return false; }
    (model.disableactuator & (1u32 << (group as u32))) != 0
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

**Site 2 — `Data::integrate()` activation state:** Per-actuator gating
in `integrate/mod.rs` (lines 37–47) and `integrate/rk4.rs` (lines 84–186).
This is the **same code location** as S4.8 Site 3. The final
implementation must use the compound check from S4.8 (blanket
`DISABLE_ACTUATION` OR per-group `actuator_disabled()`):

```rust
for i in 0..model.nu {
    let act_adr = model.actuator_act_adr[i];
    let act_num = model.actuator_act_num[i];
    // Compound check: blanket DISABLE_ACTUATION OR per-group disabling.
    // Identical to S4.8 Site 3 — these are the same guard, not two
    // independent implementations.
    let is_disabled = disabled(model, DISABLE_ACTUATION)
                   || actuator_disabled(model, i);
    for k in 0..act_num {
        let j = act_adr + k;
        let act_dot_val = if is_disabled { 0.0 } else { self.act_dot[j] };
        self.act[j] = mj_next_activation(model, i, self.act[j], act_dot_val);
    }
}
```

This matches MuJoCo's actual mechanism exactly — disabled actuators get
`act_dot = 0`, freezing their activation state without zeroing it.
**Implementation note:** S4.8 Site 3 and S7d Site 2 describe the same
code location. Implement once with the compound check above — do not
create two separate guard blocks.

### S8. Auto-reset on NaN/divergence (subsumes DT-93)

MuJoCo has three validation functions that detect divergent simulation
state and automatically reset to initial configuration. This is a safety
feature enabled by default — `DISABLE_AUTORESET` (S4.16) gates it.

**Verified against:** MuJoCo `engine_forward.c` (`mj_checkPos`,
`mj_checkVel`, `mj_checkAcc`) and `engine_util_misc.c` (`mju_isBad`).

#### S8a. Detection primitive — `is_bad()`

```rust
/// Minimum meaningful value — values below this are treated as zero.
/// Matches MuJoCo's `mjMINVAL = 1e-15`. Used for gravity-norm checks
/// and other near-zero guards.
pub const MIN_VAL: f64 = 1e-15;

/// Maximum allowed value in qpos, qvel, qacc (matches MuJoCo's mjMAXVAL).
pub const MAX_VAL: f64 = 1e10;

/// Returns true if value is NaN, +inf, -inf, or exceeds MAX_VAL.
/// Matches MuJoCo's `mju_isBad()`.
#[inline]
pub fn is_bad(x: f64) -> bool {
    x.is_nan() || x > MAX_VAL || x < -MAX_VAL
}
```

**File:** `types/validation.rs` (new). These are numeric validation utilities,
not flag helpers — they don't depend on `Model` or flag constants. Keeping
them in `flags.rs` would conflate two unrelated concerns. `validation.rs`
contains `is_bad()`, `MAX_VAL`, and any future numeric sanity checks.

#### S8b. Warning system

**File:** `types/warning.rs` (new) — `Warning` enum, `WarningStat`,
`NUM_WARNINGS`, and `mj_warning()`. Separate from `flags.rs` (flag
helpers) and `enums.rs` (type enums) to keep module responsibilities
clear.

Add a warning tracking system to `Data`:

```rust
/// Warning types (matches MuJoCo's mjtWarning enum).
/// `repr(u8)` for compact storage; cast to `usize` for array indexing.
#[repr(u8)]
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

Initialize all to zero in `Data::new()`. Add a typed accessor to avoid
`as usize` casts scattered across check functions:

```rust
impl Data {
    /// Typed accessor for warning statistics.
    #[inline]
    pub fn warning_mut(&mut self, w: Warning) -> &mut WarningStat {
        &mut self.warnings[w as usize]
    }
}
```

The warning function:

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

**`mj_check_pos()` — before forward dynamics (NOT sleep-aware):**

Note: unlike `mj_check_vel` and `mj_check_acc`, position validation scans
ALL `nq` elements without sleep filtering. Two reasons:
1. **External mutation:** sleeping bodies can have externally-set bad qpos
   (via `mj_resetData`, user code, keyframe loading, etc.) that was never
   integrated — sleep filtering would miss these. Velocity and acceleration
   can only diverge through integration, so only awake DOFs need checking.
2. **Dimensional mismatch:** the sleep filtering infrastructure
   (`dof_awake_ind`, `nv_awake`) indexes by DOF (`nv`), not by generalized
   position element (`nq`). For quaternion joints, `nq != nv` (4 position
   elements vs 3 DOFs). There is no `qpos_awake_ind` array, so sleep-
   filtered position iteration would require building a mapping from
   DOF indices to qpos ranges — unnecessary complexity given reason 1.

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
            // If forward() fails (singular mass matrix at qpos0),
            // log and continue — the reset state is the best we can do.
            if !disabled(model, DISABLE_AUTORESET) {
                if let Err(e) = mj_forward(model, data) {
                    tracing::error!(
                        "mj_forward() failed after auto-reset (model's \
                         qpos0 produces non-recoverable error): {e:?}"
                    );
                }
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

**Reentrancy guard (critical):** The `mj_forward()` called from
`mj_check_acc()` must NOT itself invoke `mj_check_acc()` again, or
infinite recursion occurs if the reset state produces bad `qacc`. In
MuJoCo this is a non-issue because `mj_checkAcc` is called from
`mj_step()` — not from within `mj_forward()` — so the re-run
`mj_forward()` is a one-shot recomputation that never re-enters the
check. CortenForge must preserve this separation:

- `mj_forward()` (or `forward_core()`) does NOT call any check
  functions. It is a pure computation with no validation side-effects.
- `step()` orchestrates the check calls externally: `check_pos` →
  `check_vel` → `forward()` → `check_acc`.
- The `mj_forward()` inside `mj_check_acc()` is the same check-free
  `forward()` — no re-entry possible.

If the existing `forward_core()` already contains check calls, they must
be extracted as part of S8g. This is a **structural requirement**, not
optional — without it, a pathological model that diverges from `qpos0`
will stack overflow.

**Current state (verified):** `forward_core()` does NOT call any check
functions — the checks live in `step()` (`forward/mod.rs:65-66, 73, 81`).
The pipeline is already structurally correct.

**Mandatory code comment:** The implemented `forward_core()` must include
this comment at the top of the function body:

```rust
// INVARIANT: forward_core() must NOT call mj_check_pos, mj_check_vel,
// or mj_check_acc. mj_check_acc() calls forward() after auto-reset —
// if forward_core() called check functions, a model that diverges from
// qpos0 would cause infinite recursion. step() orchestrates the
// check → forward → check sequence externally. This function is a
// pure computation with no validation side-effects.
```

This is the pipeline structure:

```rust
// Current step() (forward/mod.rs:58-99):
pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
    // ...timestep validation...
    check::mj_check_pos(model, self)?;   // line 65
    check::mj_check_vel(model, self)?;   // line 66
    // ...
    self.forward(model)?;                // line 72/80
    check::mj_check_acc(model, self)?;   // line 73/81
    self.integrate(model);               // line 82
    // ...sleep + warmstart save...
}
```

**After S8g:** The structure stays the same, but the `?` is removed from
the check calls (they become void) and `forward()` stays untouched:

```rust
// After S8g refactor:
pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
    if model.timestep <= 0.0 || !model.timestep.is_finite() {
        return Err(StepError::InvalidTimestep);
    }
    check::mj_check_pos(model, self);    // void — auto-resets internally
    check::mj_check_vel(model, self);    // void — auto-resets internally
    match model.integrator {
        Integrator::RungeKutta4 => {
            self.forward(model)?;         // Result — CholeskyFailed propagates
            check::mj_check_acc(model, self);  // void — auto-resets + re-runs forward internally
            crate::integrate::rk4::mj_runge_kutta(model, self)?;
        }
        _ => {
            self.forward(model)?;
            check::mj_check_acc(model, self);
            self.integrate(model);
        }
    }
    // ...sleep update + warmstart save (unchanged)...
    Ok(())
}
```

No extraction from `forward_core()` is needed — the separation already
exists. The refactor is purely at the `check.rs` function level (change
signatures from `Result` to void, add internal auto-reset logic).

**Error handling for the post-reset `mj_forward()` call:** `forward()`
returns `Result<(), StepError>` (for `CholeskyFailed`/`LuSingular`).
If the post-reset `mj_forward()` fails, the model has a structural
problem (singular mass matrix at `qpos0`) that auto-reset cannot fix.
Handle this by logging the error via `tracing::error!` and returning
without further action — the reset state is the best we can do, and
propagating the error from inside `mj_check_acc()` would require
changing its void signature back to `Result`, defeating the S8 design.
In practice, a model whose `qpos0` produces a singular mass matrix is
broken at compilation time, not at runtime — this path should never
fire for well-formed models:

```rust
if !disabled(model, DISABLE_AUTORESET) {
    if let Err(e) = mj_forward(model, data) {
        tracing::error!(
            "mj_forward() failed after auto-reset (model's qpos0 \
             produces non-recoverable error): {e:?}"
        );
    }
}
```

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

**In `step()`** (simplified — shows check/forward ordering, not full
signature; see S8g for the actual `Result<(), StepError>` return type):
```rust
// Check calls are void (S8g) — no `?` propagation.
mj_check_pos(model, data);        // (1) validate qpos BEFORE forward
mj_check_vel(model, data);        // (2) validate qvel BEFORE forward
mj_forward(model, data)?;         // (3) forward dynamics (Result — propagates CholeskyFailed/LuSingular)
mj_check_acc(model, data);        // (4) validate qacc AFTER forward
// ... integrator ...
```

**Split stepping (§53):** When `mj_step1()`/`mj_step2()` are
implemented in §53, `mj_check_pos` and `mj_check_vel` belong in
`step1`; `mj_check_acc` belongs in `step2`. See §53 for placement.

#### S8f. Reset behavior

**File:** `core/src/reset.rs` (new)
**Signature:** `pub fn mj_reset_data(model: &Model, data: &mut Data)`

Free function, not a method on `Data`, because it requires `&Model` for
`qpos0` and mocap initialization. Placed in a dedicated `reset.rs`
module (not in `check.rs`) because `mj_reset_data()` is the equivalent
of MuJoCo's `mj_resetData()` — a general-purpose API function called
from multiple contexts (auto-reset, user-facing reset, keyframe loading,
etc.). Placing it in `check.rs` would couple a general-purpose function
to a specific caller and require a re-export refactor when other callers
appear. `reset.rs` imports into `check.rs` for the auto-reset path.

`mj_reset_data()` resets to the model's initial state (`qpos0`), NOT a
keyframe. The design rule is: **zero every field in Data, then restore
the fields that have non-zero initial values from Model.** This matches
MuJoCo's `mj_resetData()` in `engine_io.c`.

Exhaustive field inventory (grouped by category):

**1. State variables — restore from Model:**
- `data.qpos` ← `model.qpos0` (reference configuration)
- `data.qvel` ← zero (length `nv`)
- `data.qacc` ← zero (length `nv`)
- `data.qacc_warmstart` ← zero (length `nv`) — next step cold-starts
- `data.time` ← `0.0`

**2. Control / actuation — zero:**
- `data.ctrl` ← zero (length `nu`)
- `data.act` ← zero (length `na`)
- `data.act_dot` ← zero (length `na`)
- `data.qfrc_actuator` ← zero (length `nv`)
- `data.actuator_force` ← zero (length `nu`)
- `data.actuator_velocity` ← zero (length `nu`)
- `data.actuator_length` ← zero (length `nu`)
- `data.actuator_moment` ← zero (each length `nv`, count `nu`)

**3. Mocap — restore from Model:**
- `data.mocap_pos[i]` ← `model.body_pos[mocap_body_id]` for each mocap body
- `data.mocap_quat[i]` ← `model.body_quat[mocap_body_id]` for each mocap body

**4. Force vectors — zero:**
- `data.qfrc_passive` ← zero (length `nv`)
- `data.qfrc_spring` ← zero (length `nv`) — NEW (S4.7-prereq)
- `data.qfrc_damper` ← zero (length `nv`) — NEW (S4.7-prereq)
- `data.qfrc_gravcomp` ← zero (length `nv`)
- `data.qfrc_fluid` ← zero (length `nv`)
- `data.qfrc_constraint` ← zero (length `nv`)
- `data.qfrc_bias` ← zero (length `nv`)
- `data.qfrc_applied` ← zero (length `nv`)
- `data.qfrc_smooth` ← zero (length `nv`)
- `data.qfrc_frictionloss` ← zero (length `nv`)
- `data.xfrc_applied` ← zero (length `nbody`)

**5. Contact / constraint state — zero:**
- `data.ncon` ← `0`
- `data.contacts` ← clear
- `data.ne` ← `0`, `data.nf` ← `0`, `data.ncone` ← `0`
- `data.efc_force` ← zero, `data.efc_J` ← zero, etc. (all `efc_*` arrays)
- `data.solver_niter` ← `0`, `data.solver_nnz` ← `0`
- `data.solver_stat` ← clear
- `data.newton_solved` ← `false`
- `data.efc_cost` ← `0.0`
- `data.stat_meaninertia` ← `0.0`

**6. Sensor data — zero:**
- `data.sensordata` ← zero (length `nsensordata`)

**7. Energy — zero:**
- `data.energy_potential` ← `0.0`
- `data.energy_kinetic` ← `0.0`

**8. Warning counters — zero:**
- `data.warnings` ← all `WarningStat { last_info: 0, count: 0 }`

**9. Sleep state — reset to fully awake:**
- `data.nv_awake` ← `model.nv`
- `data.nbody_awake` ← `model.nbody`
- `data.ntree_awake` ← `model.ntree`
- `data.tree_asleep` ← all negative (awake)
- `data.tree_awake` ← all `true`
- `data.body_sleep_state` ← all `SleepState::Awake`
- `data.body_awake_ind` ← `[0, 1, ..., nbody-1]`
- `data.dof_awake_ind` ← `[0, 1, ..., nv-1]`
- `data.parent_awake_ind` ← full range
- `data.nparent_awake` ← `model.nbody`
- `data.tree_qpos_dirty` ← all `false`

**10. Island state — zero:**
- `data.nisland` ← `0`
- `data.tree_island` ← all `-1`
- All island mapping arrays ← zero/clear

**11. Computed quantities (FK outputs, RNE, etc.) — zero:**
- These are recomputed by the next `forward()` call, so zeroing is
  sufficient. No need to run FK during reset.

**Implementation note:** The simplest correct implementation is to
call `Model::make_data()` to get a fresh `Data`, then swap it in. But
this allocates. For allocation-free reset (important for RL batch
environments), implement field-by-field zeroing using the list above.
Either approach is acceptable for §41 — optimize if profiling shows
reset is a bottleneck.

**Staleness guard (required regardless of approach):** `Data` will
gain new fields in future specs. If `mj_reset_data()` uses
field-by-field zeroing, add a comment at the top listing the canonical
field inventory version:

```rust
// Field inventory version: §41 (25 fields in 11 categories).
// When adding fields to Data, update this function and the inventory
// in S41_RUNTIME_FLAGS_SPEC.md §S8f.
```

If `mj_reset_data()` uses `Model::make_data()` + swap, add a
`#[cfg(test)]` assertion comparing the reset `Data` against
`Model::make_data()` to catch divergence:

```rust
#[cfg(test)]
{
    let fresh = model.make_data();
    debug_assert_eq!(
        std::mem::size_of_val(&data), std::mem::size_of_val(&fresh),
        "Data struct size changed — mj_reset_data() may need updating"
    );
}
```

Either mechanism ensures new fields don't silently survive resets.

If a `mj_reset_data()` function doesn't exist yet, implement it as part
of this section. It is analogous to MuJoCo's `mj_resetData()` in
`engine_io.c`.

#### S8g. Migration from existing `check.rs` API

**Existing state:** `forward/check.rs` already contains `mj_check_pos`,
`mj_check_vel`, `mj_check_acc` — but they return `Result<(), StepError>`
and never mutate `Data`. The caller (`step()`) propagates the error via
`?`, aborting the step. This is idiomatic Rust but does NOT match MuJoCo's
auto-reset semantics.

**Required change:** Refactor the three check functions from error-returning
to void with internal state mutation:

| Aspect | Current | After S8 |
|--------|---------|----------|
| Check signature | `fn(model, data) -> Result<(), StepError>` | `fn(model, data: &mut Data)` |
| On bad value | Returns `Err(StepError::InvalidPosition)` | Calls `mj_warning()` + `mj_reset_data()` internally |
| Caller (`step()`) | `check::mj_check_pos(model, self)?;` | `check::mj_check_pos(model, self);` (no `?`) |
| `StepError` check variants | `InvalidPosition`, `InvalidVelocity`, `InvalidAcceleration` | **Remove** — replaced by `Warning::BadQpos/BadQvel/BadQacc` |
| `mj_check_acc` threshold | `is_finite()` only (no magnitude bound — asymmetric with `check_pos`/`check_vel` which already check `abs() > 1e10`) | `is_bad()` — unifies all three to use the same `> MAX_VAL` threshold (MuJoCo conformance). See **Risk: `mj_check_acc` behavior change** below for impact analysis. |

**Behavior change risk for `mj_check_acc`:** The current implementation
only rejects non-finite values (NaN/inf). After S8, it will also reject
values exceeding `MAX_VAL = 1e10` (positive or negative). Simulations
that produce large-but-finite accelerations (>1e10) will now auto-reset
where they previously continued. This is correct MuJoCo behavior, but
may cause regressions in models with extreme force scales. If this
surfaces, the model's force scales are the root problem, not the check.

**`StepError` pruning (not removal):** `StepError` has 6 variants:

| Variant | Source | After S8 |
|---------|--------|----------|
| `InvalidPosition` | check.rs | **Remove** — replaced by `Warning::BadQpos` |
| `InvalidVelocity` | check.rs | **Remove** — replaced by `Warning::BadQvel` |
| `InvalidAcceleration` | check.rs | **Remove** — replaced by `Warning::BadQacc` |
| `CholeskyFailed` | linalg.rs, hessian.rs | **Keep** — non-recoverable math error |
| `LuSingular` | linalg.rs | **Keep** — non-recoverable math error |
| `InvalidTimestep` | forward/mod.rs | **Keep** — configuration error |

Remove only the 3 check variants. Keep `StepError` with the remaining 3
math/config variants. `step()` **keeps returning `Result<(), StepError>`**.

**Deliberate CortenForge deviation from MuJoCo:** MuJoCo's `mj_step()`
returns `void` and silently absorbs Cholesky failures (producing
numerically degenerate results). This is inappropriate for Rust —
`CholeskyFailed`, `LuSingular`, and `InvalidTimestep` represent
non-recoverable errors that the caller must handle. Silently swallowing
them would violate Rust's error-handling philosophy and make debugging
impossible. Auto-reset is the right answer for NaN/divergence (transient,
recoverable); it is the wrong answer for singular mass matrices
(structural, non-recoverable).

**Final `step()` signature:** `pub fn step(&mut self, model: &Model) -> Result<(), StepError>`
(unchanged return type, but check-related `?` calls removed — only
`CholeskyFailed`/`LuSingular`/`InvalidTimestep` propagate).

**Affected tests (must update in same commit as S8g):**
- `batch_sim.rs:107` — `assert!(errors[1].is_some(), "env 1 should fail")`
  currently expects `step()` to return `Err` for NaN-injected qpos.
  After S8, `step()` returns `Ok(())`. Rewrite to:
  `assert!(data.divergence_detected(), "env 1 should auto-reset");`
- `gpu/tests/integration.rs:213-215` — hedged check
  `has_nan || errors[1].is_some()` — review and update to use
  `divergence_detected()` or warning counter checks.

**Migration path for users relying on `Result`-based NaN detection:**
`step()` still returns `Result<(), StepError>`, but NaN/divergence is
no longer surfaced as `Err` — it's handled internally via auto-reset.
To detect NaN events:
1. Set `<flag autoreset="disable"/>` to prevent silent resets
2. After each `step()`, check `data.warnings[Warning::BadQpos as usize].count`
   (or use `data.warning_mut(Warning::BadQpos).count`) to detect NaN events
3. Use the `divergence_detected()` convenience method (required — this
   is the primary API for detecting NaN/divergence events after the
   `Result` removal):
   ```rust
   impl Data {
       /// Returns true if any NaN/divergence warning fired since last clear.
       ///
       /// NOTE: This detects bad-state *events*, not necessarily resets.
       /// When `DISABLE_AUTORESET` is set, warnings still fire but no
       /// reset occurs. To distinguish: check `DISABLE_AUTORESET` flag
       /// state alongside this method. When autoreset is enabled (default),
       /// `divergence_detected() == true` implies a reset happened.
       pub fn divergence_detected(&self) -> bool {
           self.warnings[Warning::BadQpos as usize].count > 0
               || self.warnings[Warning::BadQvel as usize].count > 0
               || self.warnings[Warning::BadQacc as usize].count > 0
       }
   }
   ```

### S9. BVH midphase integration (subsumes DT-94)

**Implementation phasing:** S9 was implemented in two phases:

- **S9-stub (shipped with §41):** ✅ `DISABLE_MIDPHASE` constant,
  parser/builder wiring, guard site in `collision/mod.rs`.
- **S9-full (DT-99):** ✅ **Done.** Added `use_bvh: bool` parameter to
  all 5 mesh collision functions (`mesh_sphere_contact`,
  `mesh_capsule_contact`, `mesh_box_contact`, `mesh_mesh_contact`,
  `mesh_mesh_deepest_contact`). `collide_with_mesh()` computes
  `use_bvh = !disabled(model, DISABLE_MIDPHASE)` and threads it to all
  13 call sites. Brute-force fallback iterates all triangles / all
  triangle pairs. AC31/AC33 conformance tests verify equivalence.

`sim/L0/core/src/mid_phase.rs` contains the BVH implementation (top-down
median-split AABB tree). `TriangleMeshData` builds a BVH on construction.
The collision pipeline uses BVH queries by default and falls back to
brute-force when `DISABLE_MIDPHASE` is set.

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

**Implementation note on allocation:** The pseudo-code above uses
`Vec<Contact>` returns for clarity. The actual implementation should write
contacts directly into `data.contacts` (or a pre-allocated buffer) to
avoid per-call heap allocation in the hot collision path. MuJoCo writes
into a pre-allocated arena — match this pattern.

#### S9f. Flex midphase (future extension)

Flex self-collision (§42A-iv) could also benefit from BVH midphase. When
flex objects have many vertices, a BVH over flex elements would replace
the current O(V × G) brute-force. This is an incremental extension of
the same infrastructure and is tracked in §42A-iv. The `DISABLE_MIDPHASE`
guard should also gate flex midphase once it exists.

### S10. Global contact parameter override (subsumes DT-95)

**Implementation phasing:** Like S9, S10 is a self-contained feature
gated by a runtime flag. Split into two phases:

- **S10-stub (ships with §41):** Define `ENABLE_OVERRIDE` constant,
  wire parser/builder, add Model fields (`o_margin`, `o_solref`,
  `o_solimp`, `o_friction`) with defaults, parse from `<option>`.
  The flag exists but has no runtime effect yet.
- **S10-full (separate commit, can ship independently):** Assignment
  helper functions, 6 guard sites in broadphase/narrowphase/constraint.
  When this lands, the flag becomes functional. S10-full is tracked as
  **DT-100**.

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

**Note:** `parse_attr_f64()` and `parse_attr_array()` do not currently
exist in the parser. They must be implemented as part of S10b (or use
existing attribute-parsing infrastructure — check `get_attribute_opt`,
`parse_float`, etc. in `parser.rs` for patterns to follow).

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
///
/// This is the **sole** MIN_MU clamping site in the contact pipeline.
/// The non-override path (`contact_param()`) computes combined friction
/// via element-wise max but does NOT apply MIN_MU clamping — that
/// happens here, after source selection. Both override and non-override
/// friction values pass through the same clamp.
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

> **Note:** The flex broadphase (Site 2) currently uses brute-force O(V×G)
> all-pairs, not SAP, so the `0.5` factor does not apply there. The
> implementation correctly uses the full `o_margin` as a narrowphase
> detection threshold for flex-rigid contacts.

**Sites 3–6: Contact parameter assignment**

**File:** `collision/mod.rs` or `collision/contact_params.rs`

After `mj_contact_param()` computes combined parameters from geom/flex
properties, the assignment helpers apply override:

```rust
// In contact creation (mj_setContact equivalent):
con.margin = assign_margin(model, combined_margin);
con.solref = assign_ref(model, &combined_solref);
con.solreffriction = assign_ref(model, &combined_solreffriction);  // reuses o_solref — no separate o_solreffriction field
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

1. **S1** — Define all flag constants in `enums.rs`. Create `types/flags.rs`
   with `disabled()`, `enabled()`, `actuator_disabled()`. Create
   `types/validation.rs` with `is_bad()`, `MAX_VAL`, `MIN_VAL`. Create
   `types/warning.rs` with `Warning` enum, `WarningStat`, `NUM_WARNINGS`,
   `mj_warning()`.
2. **S2** — Parser changes: remove `passive` / add `spring`+`damper`, add missing fields, fix docs,
   fix `island` default (S2e), add `actuatorgravcomp` joint attribute.
3. **S3** — Builder wiring: `apply_flags()` replaces single sleep-only block.
4. **S4.7-prereq** — Add `qfrc_spring`, `qfrc_damper` Data fields. Refactor
   `mj_fwd_passive()` to write spring/damper into separate arrays and
   aggregate into `qfrc_passive` at the end.
   **Intermediate verification (required before proceeding):** Before this
   refactor, snapshot `qfrc_passive` values from an existing test run using
   a deterministic model with known force values (e.g., single pendulum
   with stiffness and damping — not humanoid, which is too complex to
   debug if values diverge). After refactor, verify per-DOF:
   `qfrc_spring[i] + qfrc_damper[i] + qfrc_gravcomp[i] + qfrc_fluid[i]
   == qfrc_passive[i]` to machine epsilon (`f64::EPSILON * scale`). Note:
   bitwise equality is ideal but may fail if aggregation order changes
   (floating-point non-associativity). Per-DOF comparison to machine
   epsilon is the practical bar. Run the full sim domain test suite
   before moving on — this is the highest-risk refactor in §41.
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
9. **S5.1** — Energy guard. **Before gating energy behind `ENABLE_ENERGY`**,
   update all existing tests that assert on `energy_potential` or
   `energy_kinetic` to set `ENABLE_ENERGY` on their model. Known affected
   tests (grep `energy_potential\|energy_kinetic\|total_energy`):
   - `sleeping.rs` T84 (`test_energy_continuous_across_sleep_transition`)
   - `newton_solver.rs` energy conservation check
   - `lib.rs` kinetic energy conservation
   - `batch.rs` batch-vs-sequential energy equality
   - Bevy examples (`double_pendulum`, `nlink_pendulum`, etc.)
   This must happen in the **same commit** as the energy guard to avoid
   a window where tests fail. **S5.3-S5.5** — Constant only (no guard
   site): fwdinv/invdiscrete (blocked by §52), multiccd (blocked by §50).
   **S5.6** — Already wired.
10. ~~**S6** — Fluid sleep filtering.~~ **Already done** (§40c).
11. **S7** — Per-group actuator disabling: model fields, builder wiring,
    `actuator_disabled()` helper, runtime guards in actuation + integration.
12. **S8** — Auto-reset: warning system, `is_bad()` validation (in
    `types/validation.rs`), refactor existing `mj_check_pos`/`mj_check_vel`/
    `mj_check_acc` from `Result<(), StepError>` to void+auto-reset (S8g),
    prune `StepError` (remove 3 check variants, keep `CholeskyFailed`/
    `LuSingular`/`InvalidTimestep`), `mj_reset_data()`, pipeline
    integration. Wires `DISABLE_AUTORESET` guard (S4.16).
13. **S9-stub** — `DISABLE_MIDPHASE` constant + parser/builder wiring +
    guard site in `collision/mod.rs` (always takes brute-force path).
14. **S10-stub** — `ENABLE_OVERRIDE` constant + parser/builder wiring +
    Model fields (`o_margin`, `o_solref`, `o_solimp`, `o_friction`) with
    defaults + parser for `<option>` attributes. Flag exists but has no
    runtime effect.
15. **S9-full** (**DT-99**, ✅ **Done**)
    — BVH midphase: `use_bvh` parameter on 5 mesh collision functions,
    `DISABLE_MIDPHASE` guard in `collide_with_mesh()`, brute-force
    fallback paths, AC31/AC33 conformance tests.
16. **S10-full** (**DT-100**, separate commit, can ship independently of §41)
    — Global override: assignment helpers, 6 guard sites in broadphase/
    narrowphase/constraint. Activates the flag from S10-stub.

Steps 1-3 form a single commit (infrastructure). Step 4 is a standalone
refactor commit. Steps 5-14 can each be a commit (skip step 10 — already
done). Steps 12-14 are independent of each other and can be implemented
in any order. Steps 15-16 are independent follow-up commits that can ship
after §41 merges.

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

### AC12 — `passive` attribute silently ignored
`<flag passive="disable"/>` has no effect — the attribute is silently
skipped by the parser (matching MuJoCo 3.3.6+ behavior). Only `spring`
and `damper` are recognized. Verify no error is raised and no flags are
set when `passive` is the only attribute on `<flag>`.

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

**Methodology — golden file generation (hard gate, not optional):**

The existing conformance test infrastructure (`sim-conformance-tests`) does
not yet have trajectory-level MuJoCo comparison. This must be established
as part of §41 — it is the primary MuJoCo conformance gate for flags.
**§41 does not merge without at least one golden file comparison working
end-to-end.** Differential testing (flag on vs off) is too weak to catch
subtle accumulation-order differences between CortenForge and MuJoCo.

**Minimum merge bar vs full coverage (DT-97 boundary):** §41 requires
the golden file infrastructure to be operational and **at least one flag**
(preferably `DISABLE_GRAVITY` — simple, exercises RNE and energy) to have
a passing golden comparison. This establishes the pattern and proves the
pipeline works. Full 25-flag golden coverage is tracked as **DT-97**
(Phase 12) and is NOT a merge blocker for §41. The bootstrap fallback
(manual capture, below) is acceptable only for the minimum-bar commit;
DT-97 must use the Python generation script.

1. **Canonical model:** Use a simple but representative MJCF that exercises
   all flag-gated subsystems: joints with stiffness/damping, actuators,
   contacts, limits, equality constraints, gravity. A scaled-down humanoid
   from `assets/mujoco_menagerie/` or a purpose-built `flag_test.xml` are
   both acceptable. The model must be checked into `sim/L0/tests/assets/`.
2. **Generation script:** `sim/L0/tests/scripts/gen_flag_golden.py` (new).
   Uses `uv run` + `mujoco` pip package. For each flag:
   - Sets only that flag (all others default)
   - Steps 10 times from default qpos0
   - Dumps `qacc` (and optionally `qfrc_passive`, `qfrc_actuator`,
     `qfrc_constraint`) per step to `.npy`
   - Output directory: `sim/L0/tests/assets/golden/flags/`
3. **Golden files checked into repo.** Regeneration instructions in a
   `README.md` alongside the script.
4. **Rust test:** Loads golden `.npy`, runs the same model + flag
   combination, compares element-wise with `1e-8` tolerance.

This pattern is reusable by §45 (full conformance suite).

**Exact generation commands (for reproducibility):**

```bash
# From repo root. Uses uv (per project convention — never pip).
cd sim/L0/tests/scripts
uv venv
uv pip install mujoco==3.4.0 numpy
uv run gen_flag_golden.py
# Output: ../assets/golden/flags/*.npy
```

The script must pin `mujoco==3.4.0` (not `>=3.4.0`) to ensure golden
files are reproducible across machines. If a newer MuJoCo version is
needed, update the pin and regenerate all golden files in a single
commit. Document the pinned version in the script's docstring and in
`sim/L0/tests/assets/golden/flags/README.md`.

**Rust-side `.npy` loading:** Use the `ndarray-npy` crate (already in
the Cargo ecosystem, MIT-licensed, zero unsafe). If adding a dev-
dependency is undesirable, a minimal `.npy` parser (header + f64 blob)
is ~30 lines. Either approach is acceptable.

**Bootstrap fallback:** If the `mujoco` pip package has version
compatibility issues or the Python generation script fails, the hard
gate can be satisfied with manually captured golden data: run `simulate`
(MuJoCo's GUI), set the flag, step 10 times, and export `qacc` via
`mj_printData()`. The `.npy` format is preferred (loadable by `ndarray`),
but a plain-text CSV fallback is acceptable for the bootstrap commit.
The Python script should be the long-term mechanism — manual capture is
only for unblocking §41 if needed.

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

### AC38 — Island default correctness (S2e)
A model with no `<flag>` element produces `disableflags == 0` (island bit
clear). Island discovery runs by default. All existing tests pass with the
corrected `island: true` default. Verify by grepping all `DISABLE_ISLAND`
usage sites and confirming that enabling island discovery by default does
not cause test regressions. If regressions occur, those tests were relying
on the wrong default and must be fixed as part of S2e — not papered over.

### AC39 — `mj_reset_data()` correctness (S8f)
`mj_reset_data()` resets `Data` to the model's initial state. Verify all
seven properties:
1. `data.qpos == model.qpos0` (reference configuration restored)
2. `data.qvel`, `data.qacc`, `data.ctrl`, `data.act` are all zero
3. `data.time == 0.0`
4. All warning counters are zero
5. `data.ncon == 0` and contact/constraint arrays are cleared
6. All force vectors (`qfrc_passive`, `qfrc_actuator`, `qfrc_constraint`,
   `qfrc_spring`, `qfrc_damper`, `qfrc_gravcomp`, `qfrc_fluid`) are zero
7. Mocap bodies are initialized from model (`mocap_pos`, `mocap_quat`)

### AC40 — Contact-passive and spring/damper interaction
When `DISABLE_CONTACT` is set but spring/damper are enabled,
`mj_contactPassive()` returns early via its own guard (S4.7d), but
`mj_springdamper()`, `mj_gravcomp()`, and `mj_fluid()` still run.
When both `DISABLE_SPRING` and `DISABLE_DAMPER` are set, ALL passive
sub-functions (including `mj_contactPassive()`) are skipped via the
top-level `mj_passive()` early return (S4.7a), regardless of the
`DISABLE_CONTACT` flag state. Verify both paths independently.

### AC41 — DISABLE_ACTUATION + per-group orthogonality
When `DISABLE_ACTUATION` is set AND `disableactuator` has bits set, ALL
actuators are disabled (blanket flag takes precedence — the per-group
mechanism is redundant but harmless). When only per-group disabling is
active, only those groups are disabled. When neither is set, all
actuators are active. Verify the three states independently. Also verify
that per-group disabling still freezes activation (`act_dot = 0`) when
`DISABLE_ACTUATION` is NOT set — the two mechanisms use the same
`act_dot = 0` freezing pattern but at different granularities.

### AC42 — DISABLE_CONSTRAINT cascading `nefc`
When `DISABLE_CONSTRAINT` is set: `mj_collision()` returns early (S4.1)
→ `data.ncon == 0`. `assemble_unified_constraints()` is skipped (S4.3
guard inside `mj_fwd_constraint()`) → `data.nefc == 0`.
`mj_fwdConstraint()` hits the `nefc == 0` early exit → `qacc ==
qacc_smooth`, `qfrc_constraint` is all zeros, `efc_force` is all zeros.
Verify the full causal chain, not just the final `qacc` result.

### AC43 — Sleep-filtered aggregation correctness
For a fully-awake model (no sleeping bodies), the sleep-filtered
aggregation path (S4.7e, using `dof_awake_ind` indirection) produces
bitwise-identical `qfrc_passive` values as the non-filtered path (direct
`0..nv` iteration). Test by running the same model with `ENABLE_SLEEP`
set but all bodies awake — the indexed path should produce identical
results to running with sleep disabled. This catches off-by-one errors
in the indexed iteration without requiring actual sleeping bodies.

### AC44 — Per-group actuator disabling with RK4
`<option integrator="RK4" actuatorgroupdisable="1"/>` disables group-1
actuators across **all four RK4 stages**. Verify: an actuator with
`group="1"` has `act_dot = 0` at every stage (not just the first).
Specifically, the activation state at the end of an RK4 step must match
the state produced by running 4 Euler sub-steps with the same
per-group disabling. This catches bugs where the gating is applied in
`integrate/mod.rs` (Euler) but missed in one or more stages of
`integrate/rk4.rs`.

### AC45 — `actuator_velocity` unconditional after pipeline move
`actuator_velocity` equals `actuator_moment^T * qvel` unconditionally,
regardless of `DISABLE_ACTUATION`. Verify by setting
`<flag actuation="disable"/>` and confirming `actuator_velocity` is
non-zero for a model with non-zero joint velocities. This validates the
pipeline-ordering move from `actuation.rs` to `velocity.rs` (S4.8
Site 2). Also verify that `actuator_velocity` values are identical
with and without `DISABLE_ACTUATION`.

### AC46 — `DISABLE_WARMSTART` + islands interaction
When island solving is active (`DISABLE_ISLAND` not set, `nisland > 0`,
CG or Newton solver, no noslip iterations) AND `DISABLE_WARMSTART` is
set: constrained DOFs within islands start from `qacc_smooth` +
`efc_force = 0` (cold start). Unconstrained DOFs (index >= nidof)
receive `qacc_smooth` regardless. Verify both DOF categories
independently. Also verify: `qacc_warmstart` is still saved at end of
step (consumption is gated, not population).

### AC47 — Split-stepping with auto-reset
*Deferred to §53 (split stepping API).* Auto-reset check placement
in `mj_step1()`/`mj_step2()` will be specified and verified there.
Forward reference: S8e documents the intended placement.

### AC48 — `nv == 0` passive force guard (CortenForge extension)
A model with zero DOFs (`nv == 0`) does not crash or produce
out-of-bounds access in `mj_passive()`. The function returns
immediately without touching any force vectors. This covers the edge
case of a purely static scene (all bodies welded to world). Note: this
is a CortenForge safety guard — MuJoCo has no explicit `nv == 0` check
in `mj_passive()` (it relies on `mju_zero(ptr, 0)` being a no-op).

---

## Files

| File | Change |
|------|--------|
| `sim/L0/core/src/types/enums.rs` | S1: 17 disable + 4 enable constants (pure values, no `Model` dependency). S8g: Remove `InvalidPosition`, `InvalidVelocity`, `InvalidAcceleration` variants from `StepError` (keep `CholeskyFailed`, `LuSingular`, `InvalidTimestep`). |
| `sim/L0/core/src/types/flags.rs` (new) | S1: `disabled()`, `enabled()`, `actuator_disabled()` helpers |
| `sim/L0/core/src/types/validation.rs` (new) | S8a: `is_bad()`, `MAX_VAL`, `MIN_VAL` (numeric validation, not flag-related) |
| `sim/L0/core/src/types/warning.rs` (new) | S8b: `Warning` enum, `WarningStat`, `NUM_WARNINGS`, `mj_warning()` |
| `sim/L0/mjcf/src/types.rs` | S2: Remove `passive`, add `spring` + `damper`, add `fwdinv`/`invdiscrete`/`autoreset`, fix docstrings, fix `island` default (S2e) |
| `sim/L0/mjcf/src/parser.rs` | S2: Parse `spring`, `damper` (remove `passive`), `fwdinv`, `invdiscrete`, `autoreset`, `actuatorgravcomp` on joints, `actuatorgroupdisable` on option, `o_margin`/`o_solref`/`o_solimp`/`o_friction` on option |
| `sim/L0/mjcf/src/builder/mod.rs` | S3: `apply_flags()` replacing single sleep-only block |
| `sim/L0/core/src/types/data.rs` | S4.7-prereq: Add `qfrc_spring`, `qfrc_damper` fields. S8b: Add `warnings: [WarningStat; NUM_WARNINGS]`, `divergence_detected()` method |
| `sim/L0/core/src/types/model.rs` | S4.2a: Add `jnt_actgravcomp: Vec<bool>`. S7a: Add `disableactuator: u32`, `actuator_group: Vec<i32>`. S10a: Add `o_margin`, `o_solref`, `o_solimp`, `o_friction` |
| `sim/L0/core/src/types/model_init.rs` | S4.2a, S7a, S10a: Initialize new Model fields |
| `sim/L0/core/src/forward/mod.rs` | S5.1, S8e, S8g: Energy guards, remove `?` error propagation from check calls (check functions become void+auto-reset, `step()` keeps `Result` for `CholeskyFailed`/`LuSingular`/`InvalidTimestep`), update `mj_check_acc` pipeline placement |
| `sim/L0/core/src/forward/actuation.rs` | S4.2, S4.2a, S4.8, S4.9, S8d: Actuation disable guard (Site 1), actuator-level gravcomp routing, clampctrl guard, ctrl validation. `actuator_velocity` stays here (already called from correct pipeline stage via `mj_actuator_length()`); optional split to `mj_actuator_velocity()` is organizational, not correctness |
| `sim/L0/core/src/forward/velocity.rs` | S4.8: Optional — if `mj_actuator_velocity()` is split out of `mj_actuator_length()`, call it from here. Current code is already correct (pipeline ordering is right, function just lives in `actuation.rs`) |
| `sim/L0/core/src/sensor/mod.rs` | S4.10: Sensor disable guard (inside each sensor function, no zeroing) |
| `sim/L0/core/src/dynamics/rne.rs` | S4.2: Gravity guard in `mj_rne()` + `mj_gravcomp()` |
| `sim/L0/core/src/energy.rs` | S4.2, S5.1: Gravity guard in `mj_energy_pos()`, energy enable guard |
| `sim/L0/core/src/constraint/assembly.rs` | S4.1, S4.4-S4.6: Contact instantiation guard, equality, frictionloss, limit row guards |
| `sim/L0/core/src/forward/passive.rs` | S4.7-prereq, S4.7, S4.2a: Refactor to separate spring/damper arrays, mj_passive top-level guard, gravcomp routing (S6 fluid sleep filter already done via §40c) |
| `sim/L0/core/src/constraint/mod.rs` | S4.3, S4.11: Constraint assembly guard, warmstart function |
| `sim/L0/core/src/collision/mod.rs` | S4.1, S4.12, S4.13, S9e, S10d: Collision disable guard, filterparent guard, midphase dispatch, broadphase margin override |
| `sim/L0/core/src/collision/narrow.rs` or `mesh_collide.rs` | S9d: Midphase dispatch for mesh-mesh and mesh-primitive pairs |
| `sim/L0/core/src/constraint/impedance.rs` | S4.15: Refsafe guard |
| `sim/L0/core/src/integrate/mod.rs` | S4.8 Site 3, S7d Site 2: Add per-actuator disable gating to activation integration loop |
| `sim/L0/core/src/integrate/rk4.rs` | S4.8 Site 3, S7d Site 2: Add per-actuator disable gating to RK4 activation integration |
| `sim/L0/core/src/constraint/contact_params.rs` (new) | S10c: `assign_margin`, `assign_ref`, `assign_imp`, `assign_friction` helpers |
| `sim/L0/core/src/reset.rs` (new) | S8f: `mj_reset_data()` — general-purpose reset to initial state. Imported by `check.rs` for auto-reset path. |
| `sim/L0/core/src/forward/check.rs` (refactor) | S8c, S8g: Refactor existing `mj_check_pos`, `mj_check_vel`, `mj_check_acc` from `Result<(), StepError>` returns to MuJoCo-matching void functions with internal auto-reset. See S8g for migration details. |
| `sim/L0/tests/scripts/gen_flag_golden.py` (new) | AC18: Python script to generate golden `.npy` reference data from MuJoCo. Uses `uv run` + `mujoco` pip package. |
| `sim/L0/tests/assets/golden/flags/` (new dir) | AC18: Golden `.npy` files for per-flag trajectory conformance testing |

---

## Risk

- **Breaking:** Removing `MjcfFlag.passive` field (replaced by `spring`/
  `damper`). Grep for `flag.passive` and update all call sites. Pre-v1.0
  so no external users affected.
- **Breaking:** Refactoring `mj_fwd_passive()` to write spring/damper into
  separate arrays (S4.7-prereq) touches the hot path for every joint, tendon,
  and flex. Must verify force values are identical before and after refactor.
- **Energy regression:** Gating energy behind `ENABLE_ENERGY` means
  `data.energy_potential` and `data.energy_kinetic` will be 0.0 by default
  (matching MuJoCo, but differs from current always-compute behavior).
  **Affected tests (must add `ENABLE_ENERGY` to model):**
  - `sleeping.rs` T84 (`test_energy_continuous_across_sleep_transition`)
    — asserts `energy_potential != 0.0` (lines 3335, 3591, 3618)
  - `newton_solver.rs` energy conservation check (lines 257, 264) —
    will vacuously pass (0.0 == 0.0) but test nothing meaningful
  - `lib.rs` kinetic energy conservation (lines 276, 283) — same issue
  - `batch.rs` batch-vs-sequential energy equality (lines 362–368) —
    will pass but meaninglessly (both 0.0)
  - Bevy examples (`double_pendulum`, `nlink_pendulum`, etc.) — will
    silently display 0.0 energy; not test failures but misleading
  **Grep pattern:** `energy_potential\|energy_kinetic\|total_energy`
- **Island default fix:** The existing `MjcfFlag { island: false }` default
  is wrong — MuJoCo defaults to `disableflags = 0` (island enabled). Changing
  `island` to `true` may alter behavior if the hardcoded `DISABLE_ISLAND`
  constant was being set elsewhere. Audit all `DISABLE_ISLAND` usage sites
  (currently only `island/mod.rs:37`). See S2e.
- **Auto-reset behavior change (S8):** The existing `check.rs` functions
  return `Result<(), StepError>`, propagating NaN as an error to the caller.
  S8g refactors them to void functions with internal auto-reset (MuJoCo
  pattern). This is a **public API change**: `step()` will no longer return
  `StepError::InvalidPosition`/`InvalidVelocity`/`InvalidAcceleration`.
  Users relying on error propagation for NaN debugging should set
  `<flag autoreset="disable"/>` and check `data.warnings` instead.
  Additionally, `mj_check_acc` currently checks only `is_finite()` — S8
  adds the `> MAX_VAL` threshold for MuJoCo conformance.
- **BVH midphase integration (S9):** Mesh collision results must be
  identical with and without midphase. The BVH is a pure optimization —
  it must not change which contacts are generated, only the set of
  triangle pairs tested. Run mesh collision tests with midphase enabled
  and disabled and compare results.
- **Behavior change — `mj_check_acc` threshold alignment (intentional MuJoCo
  conformance fix):** `mj_check_acc` currently rejects only non-finite values
  (`check.rs:46`). `check_pos`/`check_vel` already check `abs() > 1e10`.
  After S8, all three are unified via `is_bad()` to use the same `> MAX_VAL`
  threshold. Models with extreme force scales may now auto-reset where they
  previously continued. This is an intentional conformance fix (the asymmetry
  between `check_acc` and `check_pos`/`check_vel` was a CortenForge gap, not
  a MuJoCo design choice). If regressions surface, fix the model's force
  scales, don't weaken the check. See S8g migration table for the full
  before/after comparison.
- **Collision guard behavioral change (S4.1):** The current `mj_collision()`
  guards on `model.ngeom >= 2`. The spec replaces this with MuJoCo's
  `nbodyflex < 2` (`model.nbody + model.nflex < 2`). For standard
  rigid-body models these are equivalent, but for flex-only models (bodies
  with flex elements but 0 geoms) the guard now correctly allows collision
  detection where it was previously skipped. This is a MuJoCo conformance
  fix — the old guard was a CortenForge approximation.
- **Test regression — `batch_sim.rs` NaN error isolation (S8):**
  `batch_sim.rs:107` asserts `errors[1].is_some()` after injecting NaN
  into `qpos`. After S8, `step()` returns `Ok(())` for NaN (auto-reset
  handles it internally), so this assertion will fail. Must be rewritten
  to check `data.divergence_detected()` or
  `data.warnings[Warning::BadQpos as usize].count > 0`. Similarly,
  `gpu/tests/integration.rs:213-215` has a hedged NaN-error check
  (`has_nan || errors[1].is_some()`) that needs review post-S8.
  Both must be updated in the **same commit** as S8g.
- **Reentrancy invariant (S8c):** `forward_core()` must never call check
  functions (`mj_check_pos`, `mj_check_vel`, `mj_check_acc`).
  `mj_check_acc()` calls `mj_forward()` after auto-reset — if
  `forward_core()` contained check calls, a model that diverges from
  `qpos0` would stack overflow. Currently safe (verified), but a future
  refactor could break this silently. The mandatory code comment (S8c)
  documents the invariant at the enforcement site.
- **Stub-only flag cross-reference gap:** Four flags (`ENABLE_FWDINV`,
  `ENABLE_INVDISCRETE`, `ENABLE_MULTICCD`, `DISABLE_NATIVECCD`) have
  constants and parser/builder wiring but no runtime guard site. The
  `tracing::warn!` (S3b) provides runtime detection, but the reverse
  cross-reference requirement (S3b addendum) is the primary mitigation —
  S50 and S52 specs must include a "S41 stub-only flag activation"
  section when written.
- **`mj_reset_data()` field staleness:** The exhaustive field inventory
  (S8f, 11 categories) will go stale as future specs add Data fields.
  The staleness guard (S8f addendum) mitigates this via a code-level
  version comment or `#[cfg(test)]` size assertion.
- **Flex bending force split (S4.7b Site 5):** Independent per-component
  clamping changes semantics from the current sum-then-clamp. The S4.7b
  addendum prescribes the correct approach and documents the expected
  verification behavior.
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
