# S41 Runtime Flags — Implementation Audit Plan

**Spec:** `sim/docs/todo/spec_fleshouts/S41_RUNTIME_FLAGS_SPEC.md`
**Scope:** 10 spec sections (S1–S10), 48 acceptance criteria, ~30 files
**Goal:** Verify every spec item is correctly implemented — no gaps, no drift.
**Rubric:** `sim/docs/todo/S41_AUDIT_RUBRIC.md` (9 criteria, all must be A)

---

## How to Audit

### Methodology

Each checkbox is verified by **reading the implementation file** and confirming
the exact condition/value/pattern stated. The auditor records one of:

- **Pass** — implementation matches spec exactly
- **Fail (bug)** — spec is violated (wrong condition, missing guard, wrong default)
- **Fail (drift)** — implementation differs from spec but may be intentional
  (document rationale and flag for review)
- **Gap** — spec item not implemented at all

### Tools

- **File reads:** `Read` tool on the specific file/line cited
- **Grep:** for cross-cutting checks (re-exports, StepError variant removal, etc.)
- **Test run:** `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf -p sim-types -p sim-simd`

### Execution Order

Phases 1, 3, 5 have no dependencies — run in parallel.
Phase 2 is the deepest — run after Phase 1 confirms constants/helpers exist.
Phase 4 depends on Phase 2 context.
Phases 6, 7 are final sweeps.

```
Phase 1 (structural) ──┐
Phase 3 (data model) ──┼── run in parallel
Phase 5 (test names) ──┘
         │
         ▼
Phase 2 (runtime guards) ── deepest, most time-consuming
         │
         ▼
Phase 4 (auto-reset) ── depends on Phase 2 context
         │
         ▼
Phase 6 (cross-cutting) ── behavioral changes, regressions
         │
         ▼
Phase 7 (file inventory) ── final completeness sweep
         │
         ▼
     Full test run
```

---

## Phase 1: Structural Conformance (constants, types, wiring) — COMPLETE

> **Audited 2026-02-26.** 168/168 checks pass. Two findings fixed:
>
> - **1e (gap):** `mj_reset_data` free function missing — added `reset.rs` + re-export (`dc61896`)
> - **1f (drift):** `frictionloss` docstring had stale "Enable" prefix — removed (`9018d81`)

### 1a. S1 — Flag constants (`types/enums.rs`) — 26/26 pass

Verify bit position of every constant against MuJoCo's `mjtDisableBit` / `mjtEnableBit`:

**Disable constants (19):**

| Constant | Expected value | Check |
|----------|---------------|:---:|
| `DISABLE_CONSTRAINT` | `1 << 0` | [x] |
| `DISABLE_EQUALITY` | `1 << 1` | [x] |
| `DISABLE_FRICTIONLOSS` | `1 << 2` | [x] |
| `DISABLE_LIMIT` | `1 << 3` | [x] |
| `DISABLE_CONTACT` | `1 << 4` | [x] |
| `DISABLE_SPRING` | `1 << 5` | [x] |
| `DISABLE_DAMPER` | `1 << 6` | [x] |
| `DISABLE_GRAVITY` | `1 << 7` | [x] |
| `DISABLE_CLAMPCTRL` | `1 << 8` | [x] |
| `DISABLE_WARMSTART` | `1 << 9` | [x] |
| `DISABLE_FILTERPARENT` | `1 << 10` | [x] |
| `DISABLE_ACTUATION` | `1 << 11` | [x] |
| `DISABLE_REFSAFE` | `1 << 12` | [x] |
| `DISABLE_SENSOR` | `1 << 13` | [x] |
| `DISABLE_MIDPHASE` | `1 << 14` | [x] |
| `DISABLE_EULERDAMP` | `1 << 15` | [x] |
| `DISABLE_AUTORESET` | `1 << 16` | [x] |
| `DISABLE_NATIVECCD` | `1 << 17` | [x] |
| `DISABLE_ISLAND` | `1 << 18` | [x] |

**Enable constants (6):**

| Constant | Expected value | Check |
|----------|---------------|:---:|
| `ENABLE_OVERRIDE` | `1 << 0` | [x] |
| `ENABLE_ENERGY` | `1 << 1` | [x] |
| `ENABLE_FWDINV` | `1 << 2` | [x] |
| `ENABLE_INVDISCRETE` | `1 << 3` | [x] |
| `ENABLE_MULTICCD` | `1 << 4` | [x] |
| `ENABLE_SLEEP` | `1 << 5` | [x] |

- [x] `MIN_AWAKE: i32 = 10` also defined

### 1b. S1 — Flag helpers (`types/flags.rs`) — 12/12 pass

- [x] `disabled(model: &Model, flag: u32) -> bool` exists
  - [x] Body: `model.disableflags & flag != 0`
  - [x] `debug_assert!(flag.is_power_of_two() && flag.trailing_zeros() <= 18)`
  - [x] Marked `#[inline]`
- [x] `enabled(model: &Model, flag: u32) -> bool` exists
  - [x] Body: `model.enableflags & flag != 0`
  - [x] `debug_assert!(flag.is_power_of_two() && flag.trailing_zeros() <= 5)`
  - [x] Marked `#[inline]`
- [x] `actuator_disabled(model: &Model, i: usize) -> bool` exists
  - [x] Range check: `!(0..=30).contains(&group)` returns `false`
  - [x] Body: `(model.disableactuator & (1u32 << (group as u32))) != 0`
  - [x] Marked `#[inline]`

### 1c. S1 — Validation utilities (`types/validation.rs`) — 5/5 pass

- [x] `MIN_VAL: f64 = 1e-15`
- [x] `MAX_VAL: f64 = 1e10`
- [x] `is_bad(x: f64) -> bool` — checks `x.is_nan() || x > MAX_VAL || x < -MAX_VAL`
  - [x] Does NOT use `x.abs() > MAX_VAL` (NaN would fail that check differently)
  - [x] Marked `#[inline]`

### 1d. S1 — Warning system (`types/warning.rs`) — 14/14 pass

- [x] `Warning` enum with `#[repr(u8)]`:
  - [x] `Inertia = 0`, `ContactFull = 1`, `ConstraintFull = 2`, `VgeomFull = 3`
  - [x] `BadQpos = 4`, `BadQvel = 5`, `BadQacc = 6`, `BadCtrl = 7`
- [x] `NUM_WARNINGS: usize = 8`
- [x] `WarningStat` struct with `last_info: i32` and `count: i32`
- [x] `mj_warning(data: &mut Data, warning: Warning, info: i32)` function:
  - [x] Logs via `tracing::warn!` only when `count == 0` (first occurrence)
  - [x] Always updates `last_info` and increments `count`
- [x] `Data::warning_mut(w: Warning) -> &mut WarningStat` typed accessor

### 1e. S1 — Re-exports (`lib.rs`) — 6/6 pass (1 gap fixed)

> **Finding:** `mj_reset_data` was not implemented. Added `reset.rs` wrapper +
> `pub use reset::mj_reset_data;` in `lib.rs`. Fixed in `dc61896`.

- [x] All 25 flag constants exported from crate root
- [x] `disabled`, `enabled`, `actuator_disabled` exported
- [x] `is_bad`, `MAX_VAL`, `MIN_VAL` exported
- [x] `Warning`, `WarningStat`, `NUM_WARNINGS` exported
- [x] `mj_warning` exported
- [x] `mj_reset_data` exported

### 1f. S2 — Parser conformance (`mjcf/src/types.rs`) — 35/35 pass (1 drift fixed)

**`MjcfFlag` struct field audit — all 25 fields:**

| Field | Type | Default | Flag type | Check |
|-------|------|---------|-----------|:---:|
| `constraint` | `bool` | `true` | disable | [x] |
| `equality` | `bool` | `true` | disable | [x] |
| `frictionloss` | `bool` | `true` | disable | [x] |
| `limit` | `bool` | `true` | disable | [x] |
| `contact` | `bool` | `true` | disable | [x] |
| `spring` | `bool` | `true` | disable | [x] |
| `damper` | `bool` | `true` | disable | [x] |
| `gravity` | `bool` | `true` | disable | [x] |
| `clampctrl` | `bool` | `true` | disable | [x] |
| `warmstart` | `bool` | `true` | disable | [x] |
| `filterparent` | `bool` | `true` | disable | [x] |
| `actuation` | `bool` | `true` | disable | [x] |
| `refsafe` | `bool` | `true` | disable | [x] |
| `sensor` | `bool` | `true` | disable | [x] |
| `midphase` | `bool` | `true` | disable | [x] |
| `eulerdamp` | `bool` | `true` | disable | [x] |
| `autoreset` | `bool` | `true` | disable | [x] |
| `nativeccd` | `bool` | `true` | disable | [x] |
| `island` | `bool` | **`true`** | disable | [x] |
| `override_contacts` | `bool` | `false` | enable | [x] |
| `energy` | `bool` | `false` | enable | [x] |
| `fwdinv` | `bool` | `false` | enable | [x] |
| `invdiscrete` | `bool` | `false` | enable | [x] |
| `multiccd` | `bool` | `false` | enable | [x] |
| `sleep` | `bool` | `false` | enable | [x] |

- [x] NO `passive` field exists
- [x] `MjcfFlag::default()` produces `disableflags == 0` and `enableflags == 0` when run through `apply_flags()`

**Docstring audit — 8 fields had wrong docs:**

> **Finding:** `frictionloss` docstring said `"Enable joint/tendon friction loss constraints."`
> — stale "Enable" prefix. Fixed to `"Joint/tendon friction loss constraints."` in `9018d81`.

| Field | Correct docstring (per spec) | Check |
|-------|------------------------------|:---:|
| `clampctrl` | "Clamping ctrl values to ctrlrange" | [x] |
| `frictionloss` | "Joint/tendon friction loss constraints" | [x] |
| `refsafe` | "Solref time constant safety floor (solref[0] >= 2*timestep)" | [x] |
| `filterparent` | "Parent-child body collision filtering" | [x] |
| `eulerdamp` | "Implicit damping in Euler integrator" | [x] |
| `island` | "Island discovery for parallel constraint solving" | [x] |
| `multiccd` | "Multi-point CCD for flat surfaces" | [x] |
| `nativeccd` | "Native CCD (vs libccd fallback for convex collision)" | [x] |

### 1g. S2 — Parser attribute handling (`mjcf/src/parser.rs`) — 9/9 pass

- [x] `spring` parsed as flag attribute (replaces `passive`)
- [x] `damper` parsed as flag attribute (replaces `passive`)
- [x] `fwdinv` parsed as flag attribute
- [x] `invdiscrete` parsed as flag attribute
- [x] `autoreset` parsed as flag attribute
- [x] `passive` attribute silently ignored (no error — unrecognized attr skip)
- [x] `actuatorgravcomp` parsed on `<joint>` elements as boolean
- [x] `actuatorgroupdisable` parsed on `<option>` as space-separated int list
- [x] `o_margin`, `o_solref`, `o_solimp`, `o_friction` parsed on `<option>`

### 1h. S3 — Builder wiring (`mjcf/src/builder/mod.rs`) — 33/33 pass

**`apply_flags()` — exhaustive disable flag wiring (19 flags):**

Each follows the pattern: `if !flag.<field> { *disableflags |= DISABLE_<NAME>; }`

| Flag field → constant | Polarity correct (`!field → set`) | Check |
|----------------------|:-:|:---:|
| `!constraint → DISABLE_CONSTRAINT` | | [x] |
| `!equality → DISABLE_EQUALITY` | | [x] |
| `!frictionloss → DISABLE_FRICTIONLOSS` | | [x] |
| `!limit → DISABLE_LIMIT` | | [x] |
| `!contact → DISABLE_CONTACT` | | [x] |
| `!spring → DISABLE_SPRING` | | [x] |
| `!damper → DISABLE_DAMPER` | | [x] |
| `!gravity → DISABLE_GRAVITY` | | [x] |
| `!clampctrl → DISABLE_CLAMPCTRL` | | [x] |
| `!warmstart → DISABLE_WARMSTART` | | [x] |
| `!filterparent → DISABLE_FILTERPARENT` | | [x] |
| `!actuation → DISABLE_ACTUATION` | | [x] |
| `!refsafe → DISABLE_REFSAFE` | | [x] |
| `!sensor → DISABLE_SENSOR` | | [x] |
| `!midphase → DISABLE_MIDPHASE` | | [x] |
| `!eulerdamp → DISABLE_EULERDAMP` | | [x] |
| `!autoreset → DISABLE_AUTORESET` | | [x] |
| `!nativeccd → DISABLE_NATIVECCD` | | [x] |
| `!island → DISABLE_ISLAND` | | [x] |

**`apply_flags()` — exhaustive enable flag wiring (6 flags):**

Each follows the pattern: `if flag.<field> { *enableflags |= ENABLE_<NAME>; }`

| Flag field → constant | Polarity correct (`field → set`) | Check |
|----------------------|:-:|:---:|
| `override_contacts → ENABLE_OVERRIDE` | | [x] |
| `energy → ENABLE_ENERGY` | | [x] |
| `fwdinv → ENABLE_FWDINV` | | [x] |
| `invdiscrete → ENABLE_INVDISCRETE` | | [x] |
| `multiccd → ENABLE_MULTICCD` | | [x] |
| `sleep → ENABLE_SLEEP` | | [x] |

**Stub-only warnings (S3b):**

- [x] `tracing::warn!` fires when `flag.fwdinv == true` (non-default for enable)
- [x] `tracing::warn!` fires when `flag.invdiscrete == true` (non-default for enable)
- [x] `tracing::warn!` fires when `flag.multiccd == true` (non-default for enable)
- [x] `tracing::warn!` fires when `flag.nativeccd == false` (non-default for disable)
- [x] `DISABLE_MIDPHASE` is NOT warned (S9-full is complete)
- [x] Each warning message references the blocking spec (§50, §52)

**Other builder wiring:**

- [x] `model.disableactuator = option.actuatorgroupdisable`
- [x] `model.actuator_group[i] = actuator.group.unwrap_or(0)`

---

## Phase 2: Runtime Guard Sites

For each flag, verify: guard exists at the exact spec'd file/function, logic matches spec precisely (polarity, compound conditions, init-then-guard pattern), and behavioral semantics are correct.

### 2a. S4.1 — `DISABLE_CONTACT` / `DISABLE_CONSTRAINT` — collision + contact rows

**Site 1 — `mj_collision()` in `collision/mod.rs`:**

- [ ] Function is always called from `forward_core()` (no conditional at call site)
- [ ] **Unconditional init BEFORE guard:**
  - [ ] `data.ncon = 0`
  - [ ] `data.contacts.clear()`
  - [ ] Any arena/EFC equivalent also reset unconditionally
- [ ] Guard condition (3-way OR):
  - [ ] `disabled(model, DISABLE_CONTACT)`
  - [ ] `disabled(model, DISABLE_CONSTRAINT)`
  - [ ] `nbodyflex < 2` where `nbodyflex = model.nbody + model.nflex`
  - [ ] NOT `model.ngeom >= 2` (old CortenForge approximation was replaced)
- [ ] On guard match → `return` (early exit, no collision detection)

**Site 2 — contact instantiation in `constraint/assembly.rs`:**

- [ ] Guard: `disabled(model, DISABLE_CONTACT) || data.ncon == 0 || model.nv == 0`
- [ ] Redundant with Site 1 (defense-in-depth) — verify it exists anyway

**S4.3 — `DISABLE_CONSTRAINT` in `constraint/mod.rs` (`mj_fwd_constraint`):**

- [ ] **Unconditional init BEFORE guard:** `data.qfrc_constraint[..model.nv].fill(0.0)`
- [ ] Guard skips `assemble_unified_constraints()` when `disabled(model, DISABLE_CONSTRAINT)`
- [ ] When guard fires → `nefc` stays 0 → `nefc == 0` early exit triggers
- [ ] `nefc == 0` early exit: `data.qacc.copy_from(&data.qacc_smooth)` + return
- [ ] **Causal chain verified:** `mj_collision()` early return → `ncon = 0` → assembly skipped → `nefc = 0` → `qacc = qacc_smooth`, `qfrc_constraint` all zeros, `efc_force` all zeros

### 2b. S4.2 — `DISABLE_GRAVITY` — zero gravity in bias forces

**Site 1 — `mj_rne()` in `dynamics/rne.rs`:**

- [ ] Effective gravity vector: `let grav = if disabled(model, DISABLE_GRAVITY) { Vector3::zeros() } else { model.gravity };`
- [ ] `grav` used in gravity loop (not `model.gravity` directly)
- [ ] Pattern: `gravity_force = -subtree_mass * grav`

**Site 2 — `mj_gravcomp()` in `dynamics/rne.rs`:**

- [ ] Early-return guard: `model.ngravcomp == 0 || model.gravity.norm() < MIN_VAL || disabled(model, DISABLE_GRAVITY)`
- [ ] Returns `false` when guard fires (bool return for aggregation optimization)
- [ ] Uses `MIN_VAL` (1e-15), NOT `== 0.0`
- [ ] Does NOT check `DISABLE_DAMPER` directly (only indirectly via top-level `mj_passive()`)

**Site 3 — `mj_energy_pos()` in `energy.rs`:**

- [ ] Same effective gravity vector pattern
- [ ] `potential -= mass * grav.dot(&com)` uses `grav`, not `model.gravity`

**Site 4 — actuation-side gravcomp in `forward/actuation.rs` (`mj_fwd_actuation()`):**

- [ ] Guard: `model.ngravcomp > 0 && !disabled(model, DISABLE_GRAVITY) && model.gravity.norm() >= MIN_VAL`
- [ ] Iterates joints where `model.jnt_actgravcomp[jnt] == true`
- [ ] Adds `data.qfrc_gravcomp[dofadr + i]` to `data.qfrc_actuator[dofadr + i]`
- [ ] Uses `jnt_dofnum()` for DOF count per joint type
- [ ] This site only runs when `DISABLE_ACTUATION` is NOT set (inside the actuation function after the actuation guard)

### 2c. S4.2a — Gravity compensation routing (`jnt_actgravcomp`)

**Model field:**

- [ ] `jnt_actgravcomp: Vec<bool>` exists, length `njnt`, default all `false`
- [ ] Parsed from `<joint actuatorgravcomp="true"/>`

**Passive-side routing in `forward/passive.rs`:**

- [ ] After `mj_gravcomp()` computes `qfrc_gravcomp`, routing loop runs
- [ ] Sleep-filtered iteration (indexed via `dof_awake_ind` when sleep active)
- [ ] For each DOF: checks `model.jnt_actgravcomp[model.dof_jntid[dof]]`
  - [ ] If `false` (default): `data.qfrc_passive[dof] += data.qfrc_gravcomp[dof]`
  - [ ] If `true`: gravcomp NOT added to passive (routed via actuation-side instead)
- [ ] Old unconditional `qfrc_passive += qfrc_gravcomp` pattern is gone

**Interaction matrix:**

| `DISABLE_GRAVITY` | `DISABLE_ACTUATION` | `jnt_actgravcomp=true` DOF | `jnt_actgravcomp=false` DOF | Check |
|:-:|:-:|---|---|:---:|
| off | off | gravcomp → `qfrc_actuator` | gravcomp → `qfrc_passive` | [ ] |
| off | **on** | gravcomp NOT routed (actuation skipped) | gravcomp → `qfrc_passive` | [ ] |
| **on** | off | no gravcomp computed | no gravcomp computed | [ ] |
| **on** | **on** | no gravcomp computed | no gravcomp computed | [ ] |

### 2d. S4.4–S4.6 — Constraint sub-type guards (`constraint/assembly.rs`)

**S4.4 — `DISABLE_EQUALITY`:**

- [ ] Guard in `assemble_unified_constraints()` (or its equality sub-call)
- [ ] When set: no equality constraint rows assembled (weld, joint, tendon equality)

**S4.5 — `DISABLE_FRICTIONLOSS`:**

- [ ] Guard in `assemble_unified_constraints()` (or its friction loss sub-call)
- [ ] When set: no friction loss rows assembled

**S4.6 — `DISABLE_LIMIT`:**

- [ ] Guard in `assemble_unified_constraints()` (or its limit sub-call)
- [ ] When set: both joint limits AND tendon limits are skipped
- [ ] Verify tendon limits are also covered (not just joint limits)

### 2e. S4.7 — Spring/Damper Pipeline (highest-risk refactor)

#### S4.7-prereq — Separate force arrays

- [ ] `data.qfrc_spring: DVector<f64>` exists, length `nv`
- [ ] `data.qfrc_damper: DVector<f64>` exists, length `nv`
- [ ] Both initialized to zero in `Data::new()` / `make_data()`

#### S4.7a — Top-level `mj_passive()` early-return structure

Exact control flow order:

1. [ ] `nv == 0` early return (CortenForge extension — not in MuJoCo)
2. [ ] Sleep filter computation: `let sleep_filter = enabled(model, ENABLE_SLEEP) && data.nv_awake < model.nv`
   - [ ] Uses DOF-level counts (`nv_awake` / `nv`), NOT body-level (`nbody_awake`)
3. [ ] **Unconditional zeroing of ALL 5 passive force vectors:**
   - [ ] `qfrc_spring`
   - [ ] `qfrc_damper`
   - [ ] `qfrc_gravcomp`
   - [ ] `qfrc_fluid`
   - [ ] `qfrc_passive`
   - [ ] When `sleep_filter`: zeroes only awake DOFs via `dof_awake_ind[..nv_awake]`
   - [ ] When not sleep_filter: zeroes `[..model.nv]` range
4. [ ] Spring+damper guard: `if disabled(SPRING) && disabled(DAMPER) { return; }`
   - [ ] Returns AFTER zeroing (force vectors are clean)
   - [ ] Skips ALL sub-functions: springdamper, gravcomp, fluid, contactPassive
5. [ ] Sub-functions run: springdamper → gravcomp → fluid → (contactPassive when DT-101)
6. [ ] Aggregation (S4.7e)

#### S4.7b — Component-level spring/damper gating in `mj_springdamper()`

**`PassiveForceVisitor` struct (or equivalent):**

- [ ] Has `has_spring: bool` and `has_damper: bool` fields
- [ ] Computed once: `has_spring = !disabled(model, DISABLE_SPRING)`, `has_damper = !disabled(model, DISABLE_DAMPER)`

**All 5 write sites verified:**

**Site 1 — Joint 1-DOF (hinge/slide):**
- [ ] Spring writes to `data.qfrc_spring[dof_adr]` (gated on `has_spring`)
- [ ] Damper writes to `data.qfrc_damper[dof_adr]` (gated on `has_damper`)
- [ ] Neither writes directly to `qfrc_passive`

**Site 2 — Joint multi-DOF (ball/free):**
- [ ] Damper writes to `data.qfrc_damper[dof_idx]` (gated on `has_damper`)
- [ ] No spring for ball/free joints (no stiffness field)
- [ ] Does NOT write to `qfrc_passive`

**Site 3 — Tendon spring/damper:**
- [ ] Spring force and damper force computed as separate scalars
- [ ] Spring force applied via J^T to `qfrc_spring` (gated on `has_spring`)
- [ ] Damper force applied via J^T to `qfrc_damper` (gated on `has_damper`)
- [ ] `ten_force[t]` still = `spring_force + damper_force` (diagnostic unchanged)
- [ ] Does NOT apply combined force to `qfrc_passive`

**Site 4 — Flex vertex damping:**
- [ ] Writes to `data.qfrc_damper[dof_base + k]` (gated on `has_damper`)
- [ ] Does NOT write to `qfrc_passive`

**Site 5 — Flex edge spring/damper + bending:**
- [ ] Edge spring → `qfrc_spring` (gated)
- [ ] Edge damper → `qfrc_damper` (gated)
- [ ] Bending: `spring_mag` and `damper_mag` computed separately
  - [ ] Each clamped independently (not sum-then-clamp)
  - [ ] `qfrc_spring[dof+ax] += grad[ax] * fm_spring` (gated)
  - [ ] `qfrc_damper[dof+ax] += grad[ax] * fm_damper` (gated)

#### S4.7c — Sub-function guards (independent of spring/damper)

- [ ] `mj_gravcomp()` gated on `DISABLE_GRAVITY` + gravity norm + `ngravcomp` only
  - [ ] NOT gated on `DISABLE_SPRING` or `DISABLE_DAMPER` directly
  - [ ] Only indirectly skipped via S4.7a top-level early return
- [ ] `mj_fluid()` gated on `model.opt.viscosity == 0 && model.opt.density == 0` only
  - [ ] NOT gated on spring/damper flags directly
  - [ ] Only indirectly skipped via S4.7a top-level early return
- [ ] `mj_fluid()` uses body-level sleep filtering (`nbody_awake` / `body_awake_ind`)
  - [ ] Already implemented by §40c — spot-check only

#### S4.7d — `mj_contact_passive()` and `DISABLE_CONTACT`

- [ ] If `mj_contact_passive()` exists: guard is `disabled(CONTACT) || ncon == 0 || nv == 0`
- [ ] If not yet implemented (DT-101): verify placeholder/comment documents the guard requirement
- [ ] Interaction: if BOTH spring+damper disabled, `mj_passive()` returns before `mj_contact_passive()` is reached (S4.7a)

#### S4.7e — Aggregation into `qfrc_passive`

- [ ] `qfrc_passive = qfrc_spring + qfrc_damper` (always, no boolean gate)
- [ ] `+= qfrc_gravcomp` only if `mj_gravcomp()` returned `true`
- [ ] `+= qfrc_fluid` only if `mj_fluid()` returned `true`
- [ ] Sleep-filtered aggregation: iterates `dof_awake_ind[..nv_awake]` when sleep active
- [ ] Non-filtered aggregation: iterates `0..model.nv` otherwise
- [ ] **Post-aggregation insertion point comment present:**
  - [ ] References DT-101 (`mj_contactPassive()` goes AFTER aggregation)
  - [ ] References DT-79 (user callback + plugin dispatch)
  - [ ] Explains WHY after: aggregation overwrites `qfrc_passive` with `=`, not `+=`

### 2f. S4.8 — `DISABLE_ACTUATION` — skip actuator forces

**Site 1 — `mj_fwd_actuation()` in `forward/actuation.rs`:**

- [ ] **Unconditional init BEFORE guard:** `data.actuator_force[..model.nu].fill(0.0)`
- [ ] Guard: `model.nu == 0 || disabled(model, DISABLE_ACTUATION)`
- [ ] On guard match: `data.qfrc_actuator[..model.nv].fill(0.0)` + return
- [ ] Both `actuator_force` (length `nu`) and `qfrc_actuator` (length `nv`) exist

**Site 2 — `actuator_velocity` (REMOVED from actuation gating):**

- [ ] `actuator_velocity` computed unconditionally (in velocity stage, not actuation)
- [ ] Computed inside `mj_actuator_length()` called from `forward/mod.rs` velocity stage
- [ ] NOT gated on `DISABLE_ACTUATION`
- [ ] Values are `actuator_moment^T * qvel` — pure kinematics

**Site 3 — Activation integration in `integrate/mod.rs` (Euler):**

- [ ] Per-actuator compound check: `disabled(model, DISABLE_ACTUATION) || actuator_disabled(model, i)`
- [ ] When disabled: `act_dot_val = 0.0` (freeze activation, don't zero it)
- [ ] `self.act[j] = mj_next_activation(model, i, self.act[j], act_dot_val)` still called
- [ ] Activation state preserved — only `act_dot` is zeroed

**Site 3 (RK4) — Activation integration in `integrate/rk4.rs`:**

- [ ] Same compound check present in ALL 4 RK4 stages
- [ ] Not just the first stage — verify k1, k2, k3, k4 all gate

**Site 4 — Control callback:**

- [ ] If control callback exists: gated on `!disabled(DISABLE_ACTUATION)`
- [ ] If not yet implemented: verify no ungated callback exists

### 2g. S4.9 — `DISABLE_CLAMPCTRL` — skip ctrl clamping

- [ ] Guard in `mj_fwd_actuation()` around `.clamp(ctrlrange.0, ctrlrange.1)`
- [ ] When disabled: ctrl values pass through unclamped
- [ ] When enabled (default): ctrl clamped to `ctrlrange`

### 2h. S4.10 — `DISABLE_SENSOR` — skip sensor evaluation

- [ ] Guard inside `mj_sensor_pos()`: `if disabled(model, DISABLE_SENSOR) { return; }`
- [ ] Guard inside `mj_sensor_vel()`: same pattern
- [ ] Guard inside `mj_sensor_acc()`: same pattern
- [ ] Guard inside `mj_sensor_postprocess()` (if exists): same pattern
- [ ] **Critical: sensordata is NOT zeroed on disable** — retains values from last enabled step
  - [ ] This intentionally differs from the "init-then-guard" pattern used by physics arrays
  - [ ] Rationale: sensors are read-only observers, stale data can't cause force corruption
- [ ] Call sites in `forward_core()` remain unconditional (guard is inside functions)

### 2i. S4.11 — `DISABLE_WARMSTART` — zero-initialize solver

**`warmstart()` function in `constraint/mod.rs`:**

- [ ] Returns `bool` (not void)
- [ ] Called inside `mj_fwd_constraint()`, after assembly, after `nefc == 0` check, before solver dispatch
- [ ] NOT called from `forward_core()` as a separate step
- [ ] When `disabled(model, DISABLE_WARMSTART)`:
  - [ ] `data.qacc.copy_from(&data.qacc_smooth)` — unconstrained accelerations
  - [ ] `data.efc_force.fill(0.0)` — zero constraint forces
  - [ ] Returns `false` (cold start — solvers skip cost comparison)
- [ ] When NOT disabled:
  - [ ] `data.qacc.copy_from(&data.qacc_warmstart)` — previous solution
  - [ ] `efc_force` NOT populated here (each solver computes via its own mechanism)
  - [ ] Returns `true` (warm data loaded — solvers should run cost comparison)
- [ ] Each solver receives `bool` and conditionally skips warmstart comparison:
  - [ ] PGS: `classify_constraint_states()` / dual cost comparison
  - [ ] Newton: `evaluate_cost_at()` comparison
  - [ ] CG: `evaluate_cost_at()` comparison
- [ ] **`qacc_warmstart` saving is unconditional** (end-of-step in `forward/mod.rs`)
  - [ ] `data.qacc_warmstart.copy_from(&data.qacc)` runs regardless of `DISABLE_WARMSTART`
  - [ ] Flag gates consumption, not population

### 2j. S4.12 — `DISABLE_FILTERPARENT` — parent-child collision filtering

- [ ] Guard in `collision/mod.rs` (`check_collision_affinity()` or equivalent)
- [ ] Pattern: `if !disabled(model, DISABLE_FILTERPARENT) { /* existing parent-child filter */ }`
- [ ] When disabled: parent-child geom pairs ARE allowed to collide
- [ ] When enabled (default): parent-child geom pairs are excluded

### 2k. S4.13 — `DISABLE_MIDPHASE` — BVH midphase gating

- [ ] `use_bvh` parameter on all 5 mesh collision functions:
  - [ ] `mesh_sphere_contact`
  - [ ] `mesh_capsule_contact`
  - [ ] `mesh_box_contact`
  - [ ] `mesh_mesh_contact`
  - [ ] `mesh_mesh_deepest_contact`
- [ ] `collide_with_mesh()` computes `use_bvh = !disabled(model, DISABLE_MIDPHASE)`
- [ ] Threads `use_bvh` to all 13 call sites
- [ ] When `use_bvh = false`: brute-force fallback iterates all triangles / all triangle pairs
- [ ] When `use_bvh = true`: BVH tree traversal for candidate culling

### 2l. S4.13b — `DISABLE_ISLAND` — solver constraints

- [ ] Compound condition for island solving:
  ```
  !disabled(model, DISABLE_ISLAND)
  && data.nisland > 0
  && model.opt.noslip_iterations == 0
  && matches!(model.opt.solver, Solver::CG | Solver::Newton)
  ```
- [ ] PGS always uses global solving regardless of island flag
- [ ] When islands not supported: falls back to global constraint solve

### 2m. S4.14 — `DISABLE_EULERDAMP` — implicit damping in Euler

- [ ] **Compound guard:** `!disabled(model, DISABLE_EULERDAMP) && !disabled(model, DISABLE_DAMPER)`
- [ ] Both conditions must hold for implicit damping to apply
- [ ] When either flag is set: Euler uses explicit damping only
- [ ] File: `forward/acceleration.rs` or `integrate/implicit.rs`

### 2n. S4.15 — `DISABLE_REFSAFE` — solref time-constant clamping

- [ ] `compute_kbip()` signature takes `&Model` (not just `solref`, `solimp`)
- [ ] Guard: `if !disabled(model, DISABLE_REFSAFE) && solref[0] > 0.0`
- [ ] Clamp: `solref[0].max(2.0 * model.timestep)`
- [ ] When disabled: `solref[0]` passes through unclamped
- [ ] When enabled and `solref[0] <= 0.0`: no clamping (negative solref has different semantics)
- [ ] Single call site updated: `constraint/assembly.rs`

### 2o. S4.16 — `DISABLE_AUTORESET` — NaN auto-reset gating

- [ ] In `mj_check_pos()`: `if !disabled(model, DISABLE_AUTORESET) { mj_reset_data(model, data); }`
- [ ] In `mj_check_vel()`: same pattern
- [ ] In `mj_check_acc()`: same pattern + `mj_forward()` re-run after reset
- [ ] When disabled: warnings still fire, but no reset occurs
- [ ] `data.qpos` still contains bad values (user must handle)

### 2p. S4.17 — `DISABLE_NATIVECCD` — constant only

- [ ] Constant exists (`1 << 17`)
- [ ] Parser/builder wired (S2, S3)
- [ ] **No runtime guard site** (blocked by §50)
- [ ] `tracing::warn!` fires when set to non-default (S3b)

### 2q. S5.1 — `ENABLE_ENERGY` — gate energy computation

- [ ] In `forward/mod.rs` / `forward_core()`:
  - [ ] `mj_energy_pos()` only called when `enabled(model, ENABLE_ENERGY)`
  - [ ] `mj_energy_vel()` only called when `enabled(model, ENABLE_ENERGY)`
- [ ] When NOT enabled:
  - [ ] `data.energy_potential = 0.0` (explicitly zeroed, not left stale)
  - [ ] `data.energy_kinetic = 0.0` (explicitly zeroed, not left stale)
- [ ] Default behavior: energy NOT computed (disabled by default)
  - [ ] This is a change from previous always-compute behavior

### 2r. S5.2 — `ENABLE_OVERRIDE` — global contact parameter override

**Model fields (S10a):**

- [ ] `o_margin: f64` — default `0.0`
- [ ] `o_solref: [f64; 2]` — default `[0.02, 1.0]`
- [ ] `o_solimp: [f64; 5]` — default `[0.9, 0.95, 0.001, 0.5, 2.0]`
- [ ] `o_friction: [f64; 5]` — default `[1.0, 1.0, 0.005, 0.0001, 0.0001]`

**Assignment helpers (S10c):**

- [ ] `assign_margin(model, source) -> f64`: returns `o_margin` when override enabled
- [ ] `assign_ref(model, source) -> [f64; 2]`: returns `o_solref` when override enabled
- [ ] `assign_imp(model, source) -> [f64; 5]`: returns `o_solimp` when override enabled
- [ ] `assign_friction(model, source) -> [f64; 5]`:
  - [ ] Returns `o_friction` when override enabled, else `*source`
  - [ ] **Always** clamps each component to `MIN_MU = 1e-5` (both paths)
- [ ] `MIN_MU: f64 = 1e-5` constant defined

**Guard sites (S10d — 6 locations):**

Site 1 — Broadphase AABB margin (rigid geoms):
- [ ] Uses `0.5 * model.o_margin` instead of `model.geom_margin[geom_id]`
- [ ] `0.5` factor because AABB expansion is per-geom (half the contact margin)

Site 2 — Broadphase AABB margin (flex):
- [ ] Uses override margin for flex objects when enabled

Sites 3–6 — Contact parameter assignment:
- [ ] `con.margin = assign_margin(model, combined_margin)`
- [ ] `con.solref = assign_ref(model, &combined_solref)`
- [ ] `con.solimp = assign_imp(model, &combined_solimp)`
- [ ] `con.friction = assign_friction(model, &combined_friction)`
- [ ] `con.solreffriction` uses `assign_ref` (same `o_solref`, no separate `o_solreffriction`)

**Key behaviors:**

- [ ] Override is total replacement (computed params discarded)
- [ ] Gap is NEVER overridden
- [ ] Condim is NEVER overridden
- [ ] Override applies ONLY to contacts (not joints, tendons, equality)

### 2s. S5.3–S5.5 — Stub-only enable flags

- [ ] `ENABLE_FWDINV`: constant exists, no guard site (blocked by §52)
- [ ] `ENABLE_INVDISCRETE`: constant exists, no guard site (blocked by §52)
- [ ] `ENABLE_MULTICCD`: constant exists, no guard site (blocked by §50)
- [ ] Verify NO accidental guard sites exist for these flags

### 2t. S5.6 — `ENABLE_SLEEP` — already wired

- [ ] Spot-check: sleep filtering active in relevant functions
- [ ] No changes required by §41 — verify no regressions

### 2u. S7 — Per-group actuator disabling

**S7a — Model fields:**

- [ ] `disableactuator: u32` — default `0`, next to `disableflags`/`enableflags`
- [ ] `actuator_group: Vec<i32>` — length `nu`, default `0`
- [ ] Both initialized in `model_init.rs`

**S7b — Builder wiring:**

- [ ] `actuatorgroupdisable` parsed from `<option>` as space-separated int list
- [ ] Each group ID (0–30) sets bit: `disableactuator |= 1u32 << group`
- [ ] Out-of-range groups (< 0 or > 30) produce error
- [ ] `actuator_group[i] = actuator.group.unwrap_or(0)` wired from per-actuator parse

**S7c — `actuator_disabled()` helper:**

- [ ] Covered in Phase 1b above — verify here it's actually used at guard sites

**S7d — Runtime guard sites:**

Site 1 — `mj_fwd_actuation()` per-actuator force loop:
- [ ] `if actuator_disabled(model, i) { continue; }` inside per-actuator loop
- [ ] Skips force computation for disabled actuators (not just zeroing)

Site 2 — Activation integration (same location as S4.8 Site 3):
- [ ] Compound check: `disabled(ACTUATION) || actuator_disabled(model, i)`
- [ ] Single implementation, not two separate guard blocks
- [ ] Present in both Euler (`integrate/mod.rs`) and RK4 (`integrate/rk4.rs`)

**Interaction matrix — DISABLE_ACTUATION × per-group disable:**

| `DISABLE_ACTUATION` | Per-group mask | Force computation | Activation integration | Check |
|:-:|:-:|---|---|:---:|
| off | group NOT disabled | normal force + `act_dot` | normal | [ ] |
| off | group disabled | force skipped (`continue`) | `act_dot = 0.0` (freeze) | [ ] |
| **on** | any | all forces skipped (top-level guard) | all `act_dot = 0.0` | [ ] |

**Interaction matrix — DISABLE_CONTACT × spring/damper × contactPassive:**

| `DISABLE_CONTACT` | `DISABLE_SPRING + DAMPER` | `mj_contact_passive()` reached? | Spring/damper forces? | Check |
|:-:|:-:|:-:|:-:|:---:|
| off | off | yes (if DT-101 done) | yes | [ ] |
| off | **both on** | no (S4.7a early return) | no | [ ] |
| **on** | off | yes but `ncon==0` guard → no-op | yes | [ ] |
| **on** | **both on** | no (S4.7a early return) | no | [ ] |

---

## Phase 3: Data Model & Field Inventory — COMPLETE

> **Audited 2026-02-26.** 17/17 checks pass. No findings.

### 3a. New Model fields — 7/7 pass

| Field | Type | Default | Source | Check |
|-------|------|---------|--------|:---:|
| `jnt_actgravcomp` | `Vec<bool>` | `vec![false; njnt]` | `<joint actuatorgravcomp>` | [x] |
| `disableactuator` | `u32` | `0` | `<option actuatorgroupdisable>` | [x] |
| `actuator_group` | `Vec<i32>` | `vec![0; nu]` | `<actuator group>` | [x] |
| `o_margin` | `f64` | `0.0` | `<option o_margin>` | [x] |
| `o_solref` | `[f64; 2]` | `[0.02, 1.0]` | `<option o_solref>` | [x] |
| `o_solimp` | `[f64; 5]` | `[0.9, 0.95, 0.001, 0.5, 2.0]` | `<option o_solimp>` | [x] |
| `o_friction` | `[f64; 5]` | `[1.0, 1.0, 0.005, 0.0001, 0.0001]` | `<option o_friction>` | [x] |

### 3b. New Data fields — 5/5 pass

| Field | Type | Default | Check |
|-------|------|---------|:---:|
| `qfrc_spring` | `DVector<f64>` | zero, length `nv` | [x] |
| `qfrc_damper` | `DVector<f64>` | zero, length `nv` | [x] |
| `warnings` | `[WarningStat; NUM_WARNINGS]` | all zero | [x] |

- [x] `divergence_detected()` method on Data:
  - [x] Checks `warnings[BadQpos].count > 0 || warnings[BadQvel].count > 0 || warnings[BadQacc].count > 0`
  - [x] Has doc comment explaining it detects events, not necessarily resets

### 3c. Existing fields verified — 5/5 pass

- [x] `disableflags: u32` on Model (should already exist)
- [x] `enableflags: u32` on Model (should already exist)
- [x] `qfrc_gravcomp`, `qfrc_fluid`, `qfrc_passive` on Data (should already exist)

---

## Phase 4: Auto-Reset System (S8)

### 4a. Detection primitive (`types/validation.rs`)

- [ ] `is_bad()` correctly detects: NaN, +inf, -inf, `> MAX_VAL`, `< -MAX_VAL`
- [ ] `MAX_VAL = 1e10` matches MuJoCo's `mjMAXVAL`
- [ ] `MIN_VAL = 1e-15` matches MuJoCo's `mjMINVAL`

### 4b. Check functions (`forward/check.rs`)

**`mj_check_pos(model, data)` — position validation:**

- [ ] Scans ALL `nq` elements (NOT sleep-filtered)
  - Reason 1: sleeping bodies can have externally-set bad qpos
  - Reason 2: no `qpos_awake_ind` exists (`nq != nv` for quaternion joints)
- [ ] Uses `is_bad()` for each element
- [ ] On bad value:
  1. [ ] `mj_warning(data, Warning::BadQpos, i as i32)`
  2. [ ] `if !disabled(model, DISABLE_AUTORESET) { mj_reset_data(model, data); }`
  3. [ ] `data.warnings[BadQpos].count += 1` (post-reset preservation)
  4. [ ] `data.warnings[BadQpos].last_info = i as i32` (post-reset preservation)
  5. [ ] `return` (stop scanning)
- [ ] Return type is `()` (void), NOT `Result`

**`mj_check_vel(model, data)` — velocity validation:**

- [ ] Sleep-aware: `let sleep_filter = enabled(ENABLE_SLEEP) && data.nv_awake < model.nv`
- [ ] When sleep_filter: iterates `nv_awake` elements via `dof_awake_ind`
- [ ] When not: iterates `0..model.nv`
- [ ] Uses `is_bad()` for each element
- [ ] Same warning + reset + preservation pattern as `mj_check_pos()`
- [ ] Uses `Warning::BadQvel`
- [ ] Return type is `()` (void)

**`mj_check_acc(model, data)` — acceleration validation:**

- [ ] Sleep-aware (same pattern as `mj_check_vel()`)
- [ ] Uses `is_bad()` — checks `> MAX_VAL` threshold, not just `is_finite()`
  - [ ] This is a behavior change from the old `check.rs` which only checked `is_finite()`
- [ ] On bad value:
  1. [ ] `mj_warning(data, Warning::BadQacc, i as i32)`
  2. [ ] `if !disabled(model, DISABLE_AUTORESET) { mj_reset_data(model, data); }`
  3. [ ] Warning counter preservation (same pattern)
  4. [ ] **Re-runs `mj_forward(model, data)`** after reset
     - [ ] `mj_forward()` failure handled: `tracing::error!` + continue (no propagation)
     - [ ] This is the ONLY check function that re-runs forward
  5. [ ] `return`
- [ ] Return type is `()` (void)

### 4c. `StepError` pruning

- [ ] `InvalidPosition` variant **REMOVED**
- [ ] `InvalidVelocity` variant **REMOVED**
- [ ] `InvalidAcceleration` variant **REMOVED**
- [ ] `CholeskyFailed` **KEPT** (non-recoverable math error)
- [ ] `LuSingular` **KEPT** (non-recoverable math error)
- [ ] `InvalidTimestep` **KEPT** (configuration error)
- [ ] `step()` signature unchanged: `pub fn step(&mut self, model: &Model) -> Result<(), StepError>`
- [ ] Check calls in `step()` have NO `?` operator

### 4d. `mj_reset_data()` (`core/src/reset.rs`)

- [ ] Free function: `pub fn mj_reset_data(model: &Model, data: &mut Data)`
- [ ] Lives in `reset.rs`, not `check.rs`

**Exhaustive field reset inventory (11 categories):**

1. **State variables:**
   - [ ] `data.qpos ← model.qpos0`
   - [ ] `data.qvel ← zero(nv)`
   - [ ] `data.qacc ← zero(nv)`
   - [ ] `data.qacc_warmstart ← zero(nv)`
   - [ ] `data.time ← 0.0`

2. **Control / actuation:**
   - [ ] `data.ctrl ← zero(nu)`
   - [ ] `data.act ← zero(na)`
   - [ ] `data.act_dot ← zero(na)`
   - [ ] `data.qfrc_actuator ← zero(nv)`
   - [ ] `data.actuator_force ← zero(nu)`
   - [ ] `data.actuator_velocity ← zero(nu)`
   - [ ] `data.actuator_length ← zero(nu)`
   - [ ] `data.actuator_moment ← zero`

3. **Mocap:**
   - [ ] `data.mocap_pos[i] ← model.body_pos[mocap_body_id]`
   - [ ] `data.mocap_quat[i] ← model.body_quat[mocap_body_id]`

4. **Force vectors:**
   - [ ] `data.qfrc_passive ← zero(nv)`
   - [ ] `data.qfrc_spring ← zero(nv)`
   - [ ] `data.qfrc_damper ← zero(nv)`
   - [ ] `data.qfrc_gravcomp ← zero(nv)`
   - [ ] `data.qfrc_fluid ← zero(nv)`
   - [ ] `data.qfrc_constraint ← zero(nv)`
   - [ ] `data.qfrc_bias ← zero(nv)`
   - [ ] `data.qfrc_applied ← zero(nv)`
   - [ ] `data.qfrc_smooth ← zero(nv)`
   - [ ] `data.qfrc_frictionloss ← zero(nv)`
   - [ ] `data.xfrc_applied ← zero(nbody)`

5. **Contact / constraint:**
   - [ ] `data.ncon ← 0`, `data.contacts ← clear`
   - [ ] `data.ne ← 0`, `data.nf ← 0`, `data.ncone ← 0`
   - [ ] `data.efc_force ← zero`, `data.efc_J ← zero`, etc.
   - [ ] `data.solver_niter ← 0`, `data.solver_nnz ← 0`

6. **Sensor:** `data.sensordata ← zero(nsensordata)` — [ ]

7. **Energy:** `data.energy_potential ← 0.0`, `data.energy_kinetic ← 0.0` — [ ]

8. **Warnings:** all `WarningStat { last_info: 0, count: 0 }` — [ ]

9. **Sleep state (fully awake):**
   - [ ] `data.nv_awake ← model.nv`
   - [ ] `data.nbody_awake ← model.nbody`
   - [ ] `data.ntree_awake ← model.ntree`
   - [ ] `data.body_sleep_state ← all Awake`
   - [ ] `data.body_awake_ind ← [0..nbody-1]`
   - [ ] `data.dof_awake_ind ← [0..nv-1]`

10. **Island state:** `data.nisland ← 0`, mapping arrays cleared — [ ]

11. **Computed quantities:** zeroed (recomputed by next `forward()`) — [ ]

- [ ] Staleness guard: version comment or `#[cfg(test)]` size assertion

### 4e. Ctrl validation (S8d)

- [ ] Located in `mj_fwd_actuation()`, BEFORE force computation
- [ ] Loop: `for i in 0..model.nu { if is_bad(data.ctrl[i]) { ... } }`
- [ ] On bad ctrl:
  1. [ ] `mj_warning(data, Warning::BadCtrl, i as i32)`
  2. [ ] `data.ctrl[..model.nu].fill(0.0)` — zeros ALL ctrl
  3. [ ] `break` — stops scanning (one warning suffices)
- [ ] Does NOT trigger `mj_reset_data()` — only ctrl is zeroed

### 4f. Reentrancy invariant

- [ ] `forward_core()` does NOT call `mj_check_pos`, `mj_check_vel`, or `mj_check_acc`
- [ ] Mandatory comment at top of `forward_core()` documenting this invariant
  - [ ] Explains: `mj_check_acc()` calls `forward()` after reset — re-entry would stack overflow
- [ ] `step()` orchestrates externally: `check_pos → check_vel → forward → check_acc → integrate`
- [ ] The `forward()` inside `mj_check_acc()` is the same check-free `forward()` — no re-entry

### 4g. Test migration

- [ ] `batch_sim.rs:107` updated: no longer `assert!(errors[1].is_some())` — uses `divergence_detected()` or warning counter
- [ ] `gpu/tests/integration.rs:213-215` updated similarly
- [ ] All tests using `energy_potential` / `energy_kinetic` set `ENABLE_ENERGY` on model:
  - [ ] `sleeping.rs` T84
  - [ ] `newton_solver.rs` energy conservation check
  - [ ] `lib.rs` kinetic energy conservation
  - [ ] `batch.rs` batch-vs-sequential energy equality
  - [ ] Bevy examples (`double_pendulum`, `nlink_pendulum`, etc.)
- [ ] No test references `StepError::InvalidPosition`, `InvalidVelocity`, or `InvalidAcceleration`

---

## Phase 5: Test Coverage vs Acceptance Criteria

Cross-reference every AC with actual test functions from `runtime_flags.rs`
and `golden_flags.rs`.

| AC | Test function(s) | Key assertion | Check |
|----|-----------------|--------------|:---:|
| AC1 | `ac1_contact_disable` | `ncon == 0`, body falls through floor | [ ] |
| AC2 | `ac2_gravity_disable` | zero `qacc`, zero potential energy | [ ] |
| AC3 | `ac3_limit_disable` | joint exceeds `range`, zero constraint force | [ ] |
| AC4 | `ac4_equality_disable` | weld-linked bodies separate freely | [ ] |
| AC5 | `ac5_spring_damper_independence` | spring=off → zero spring, nonzero damper; and vice versa | [ ] |
| AC6 | `ac6_actuation_disable` | `qfrc_actuator` zero despite nonzero `ctrl` | [ ] |
| AC7 | `ac7_sensor_disable` | `sensordata` retains stale value (NOT zeroed) | [ ] |
| AC8 | `ac8_warmstart_disable` | solver converges from cold start, finite `qacc` | [ ] |
| AC9 | `ac9_constraint_disable` | `ncon == 0`, `qfrc_constraint == 0` | [ ] |
| AC10 | `ac10_passive_top_level_gating` | both spring+damper off → all passive sub-forces zero | [ ] |
| AC11 | `ac11_default_bitfields` | `disableflags == 0`, `enableflags == 0` | [ ] |
| AC12 | `ac12_passive_attribute_ignored` | no error, no flags set for `passive="disable"` | [ ] |
| AC13 | `ac13_energy_gating` | energy fields zero without `ENABLE_ENERGY`, nonzero with | [ ] |
| AC14 | `ac14_fluid_sleep_filtering` | sleeping body's `qfrc_fluid` DOFs zero, awake body nonzero | [ ] |
| AC15 | `ac15_filterparent_disable` | parent-child geom pair produces contact | [ ] |
| AC16 | `ac16_frictionloss_disable` | no `FrictionLoss` rows in `efc_type` | [ ] |
| AC17 | `ac17_refsafe_disable` | `solref[0]` below `2*timestep` produces different `efc_aref` | [ ] |
| AC18 | `ac18_golden_disable_gravity` | 10-step `qacc` within `1e-8` of MuJoCo `.npy` | [ ] |
| AC19 | `ac19_clampctrl_disable` | `ctrl=2.0` accepted with `ctrlrange="0 1"`, larger actuator force | [ ] |
| AC20 | `ac20_eulerdamp_disable` | implicit damping skipped, finite `qacc` | [ ] |
| AC21 | `ac21_passive_force_hierarchy` | one-of spring/damper off → damper+gravcomp still run | [ ] |
| AC22 | `ac22_per_group_actuator_disable` | group-2 zero, group-0 unaffected, activation freezes, group>30 immune | [ ] |
| AC23 | `ac23_unconditional_initialization` | stale forces cleared by init-then-guard pattern | [ ] |
| AC24 | `ac24_gravcomp_routing` | `jnt_actgravcomp=true` → gravcomp in `qfrc_actuator` not `qfrc_passive` | [ ] |
| AC25 | `ac25_spring_damper_force_separation` | `qfrc_passive == spring + damper + gravcomp + fluid` per DOF | [ ] |
| AC26 | `ac26_auto_reset_on_nan` | NaN in `qpos` → warning + reset to `qpos0`, `time == 0.0` | [ ] |
| AC27 | `ac27_auto_reset_threshold` | `|val| > 1e10` triggers reset, `BadQpos` warning | [ ] |
| AC28 | `ac28_autoreset_disable` | warning fires, no reset, `qpos` still NaN | [ ] |
| AC29 | `ac29_ctrl_validation` | bad ctrl → zeros ALL ctrl, `BadCtrl`, no qpos/qvel reset | [ ] |
| AC30 | `ac30_sleep_aware_validation` | sleeping DOF with NaN vel → no reset; awake DOF → fires | [ ] |
| AC31 | `ac31_midphase_matches_brute_force` | identical penetration depth midphase vs brute-force | [ ] |
| AC32 | **NO TEST** | midphase < 50% brute-force for >200 tri — **GAP** | [ ] |
| AC33 | `ac33_midphase_disable_flag_brute_force` | brute-force detects contacts, flag constant is bit 14 | [ ] |
| AC34 | **NO TEST** | `ENABLE_OVERRIDE` → all contacts use `o_*` values — **GAP** | [ ] |
| AC35 | `ac35_override_margin_solver_params` + `ac35b`–`ac35j` (10 tests) | margin, solref, solimp, friction, AABB, condim, gap, priority, XML e2e | [ ] |
| AC36 | `ac36_override_friction_clamping` + `ac36b` + `ac36c` | `MIN_MU` clamping: override, non-override, explicit pair | [ ] |
| AC37 | `ac37_override_disabled_by_default` | `ENABLE_OVERRIDE` clear in fresh model | [ ] |
| AC38 | `ac38_island_default_correctness` | `DISABLE_ISLAND` clear, island discovery runs | [ ] |
| AC39 | `ac39_reset_correctness` + `ac39b_reset_mocap` | 7 properties + mocap restoration | [ ] |
| AC40 | `ac40_contact_passive_spring_interaction` | `DISABLE_CONTACT` doesn't suppress spring/damper | [ ] |
| AC41 | `ac41_actuation_plus_pergroup_orthogonality` + `ac22_*` | 4 interaction states, activation freezing | [ ] |
| AC42 | `ac42_constraint_cascading` | full chain: `ncon=0 → nefc=0 → qacc=qacc_smooth` | [ ] |
| AC43 | `ac43_sleep_filtered_aggregation` | all-awake: `ENABLE_SLEEP` path == direct path | [ ] |
| AC44 | `ac44_pergroup_rk4` | group-disabled: zero effect across all 4 RK4 stages | [ ] |
| AC45 | `ac45_actuator_velocity_unconditional` | `actuator_velocity` nonzero despite `DISABLE_ACTUATION` | [ ] |
| AC46 | `ac46_warmstart_plus_islands` | island + cold start converges, `qacc_warmstart` saved | [ ] |
| AC47 | N/A | **Deferred to §53** | N/A |
| AC48 | `ac48_nv_zero_passive_guard` | zero-DOF model: no panic on `forward()` or `step()` | [ ] |

**Additional non-AC tests (helper/parsing):**

| Test | Covers | Check |
|------|--------|:---:|
| `flag_helpers_disabled` | `disabled()` helper correctness | [ ] |
| `flag_helpers_enabled` | `enabled()` helper correctness | [ ] |
| `flag_helpers_actuator_disabled` | `actuator_disabled()` helper correctness | [ ] |
| `s1_flag_parsing_disable` | All 16 MJCF disable attrs → correct `disableflags` bits | [ ] |
| `s1_flag_parsing_enable` | MJCF enable attrs → correct `enableflags` bits | [ ] |
| `s7_actuatorgroupdisable_parsing` | `actuatorgroupdisable` attr → correct `disableactuator` bits | [ ] |

**Test gaps (no test function found):**

- [ ] **AC32** — Midphase performance benchmark (may be intentionally omitted — verify)
- [ ] **AC34** — Global override end-to-end (may be covered by `ac35_*` suite — verify)

### Golden file infrastructure (AC18)

- [ ] `sim/L0/tests/scripts/gen_flag_golden.py` exists
- [ ] Script pins `mujoco==3.4.0` (exact pin, not `>=`)
- [ ] Script uses `uv run` (not pip)
- [ ] Script outputs to `sim/L0/tests/assets/golden/flags/*.npy`
- [ ] Golden test MJCF: `sim/L0/tests/assets/golden/flags/flag_golden_test.xml` exists
- [ ] Model exercises: joints with stiffness/damping, actuators, contacts, limits, equality, gravity
- [ ] At least `DISABLE_GRAVITY` has a passing golden comparison (minimum merge bar)
- [ ] Rust test loads `.npy`, runs same model+flag combination, compares element-wise at `1e-8`
- [ ] `.npy` loading via `ndarray-npy` or minimal parser

---

## Phase 6: Cross-cutting Concerns

### 6a. Mandatory code comments

- [ ] Reentrancy invariant comment at top of `forward_core()` (S8c)
- [ ] DT-101/DT-79 post-aggregation insertion point comment in `mj_passive()` (S4.7e)
- [ ] Field inventory version comment in `mj_reset_data()` (S8f)

### 6b. Behavioral changes — verify each is implemented and tests migrated

Every behavioral change from the spec's Risk section must be verified as
both (1) implemented correctly and (2) existing tests updated.

| # | Change | Spec | Affected code/tests | Check |
|---|--------|------|---------------------|:---:|
| B1 | `MjcfFlag.passive` field removed | S2a | grep for `flag.passive` — zero hits | [ ] |
| B2 | `island` default `false → true` | S2e | `MjcfFlag::default()`, `sleeping.rs` tests | [ ] |
| B3 | Energy gated behind `ENABLE_ENERGY` | S5.1 | `sleeping.rs` T84, `newton_solver.rs`, `lib.rs`, `batch.rs`, Bevy examples | [ ] |
| B4 | `mj_check_acc` threshold: `is_finite()` → `is_bad()` | S8g | `check.rs`, models with extreme forces | [ ] |
| B5 | `step()` no longer returns `Err` for NaN | S8g | `batch_sim.rs:107`, `gpu/tests/integration.rs:213-215` | [ ] |
| B6 | `StepError` loses 3 variants | S8g | grep for `InvalidPosition/Velocity/Acceleration` — zero hits | [ ] |
| B7 | Collision guard `ngeom >= 2` → `nbodyflex < 2` | S4.1 | `collision/mod.rs`, flex-only models | [ ] |
| B8 | Flex bending: sum-then-clamp → per-component clamp | S4.7b Site 5 | `passive.rs` flex bending, force comparison tests | [ ] |
| B9 | `qfrc_passive` refactored to aggregation of 4 sub-arrays | S4.7-prereq | `passive.rs`, all tests checking `qfrc_passive` values | [ ] |

### 6c. No regressions

- [ ] `batch_sim.rs` NaN test updated for auto-reset semantics
- [ ] `gpu/tests/integration.rs` NaN test updated
- [ ] Energy tests set `ENABLE_ENERGY` on their models
- [ ] No code references `StepError::InvalidPosition/Velocity/Acceleration`
- [ ] Island discovery enabled by default doesn't break tests (S2e)
- [ ] `qfrc_passive` values unchanged after aggregation refactor (S4.7-prereq intermediate verification)

### 6d. Edge cases & boundary conditions

Dedicated checks for every spec-identified edge case:

| # | Edge case | Spec | Check |
|---|-----------|------|:---:|
| E1 | `nv == 0` in `mj_passive()` — early return, no OOB | S4.7a, AC48 | [ ] |
| E2 | `nu == 0` in `mj_fwd_actuation()` — part of actuation guard | S4.8 | [ ] |
| E3 | `nq != nv` for quaternion joints — `mj_check_pos` NOT sleep-filtered | S8c | [ ] |
| E4 | `actuator_group < 0` or `> 30` — never disabled | S7c | [ ] |
| E5 | `solref[0] <= 0.0` — no refsafe clamping (negative solref = different semantics) | S4.15 | [ ] |
| E6 | `nbodyflex < 2` — no collision possible | S4.1 | [ ] |
| E7 | Post-reset `mj_forward()` failure in `mj_check_acc` — `tracing::error!`, no propagation | S8c | [ ] |
| E8 | Model whose `qpos0` produces singular mass matrix — pathological but handled | S8c | [ ] |
| E9 | Sleeping DOF with NaN velocity — not detected by sleep-filtered check | S8c, AC30 | [ ] |
| E10 | `qacc_warmstart` saved even when `DISABLE_WARMSTART` set — consumption gated, not population | S4.11 | [ ] |

### 6e. Full test suite

```
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics \
  -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf \
  -p sim-types -p sim-simd
```

- [ ] 2,007+ passed (baseline), 0 failed
- [ ] Plus new per-flag tests from §41

---

## Phase 7: File Inventory Verification

Every file from the spec's Files table verified for existence and stated changes.

| File | Expected changes | Check |
|------|-----------------|:---:|
| `core/src/types/enums.rs` | S1: 17 disable + 4 enable constants. S8g: 3 StepError variants removed | [ ] |
| `core/src/types/flags.rs` (new) | S1: `disabled()`, `enabled()`, `actuator_disabled()` | [ ] |
| `core/src/types/validation.rs` (new) | S8a: `is_bad()`, `MAX_VAL`, `MIN_VAL` | [ ] |
| `core/src/types/warning.rs` (new) | S8b: `Warning` enum, `WarningStat`, `NUM_WARNINGS`, `mj_warning()` | [ ] |
| `core/src/types/data.rs` | S4.7-prereq: `qfrc_spring`, `qfrc_damper`. S8b: `warnings`, `divergence_detected()` | [ ] |
| `core/src/types/model.rs` | S4.2a: `jnt_actgravcomp`. S7a: `disableactuator`, `actuator_group`. S10a: `o_margin`, `o_solref`, `o_solimp`, `o_friction` | [ ] |
| `core/src/types/model_init.rs` | S4.2a, S7a, S10a: initialize all new Model fields | [ ] |
| `core/src/lib.rs` | Re-exports for all new public items | [ ] |
| `core/src/reset.rs` (new) | S8f: `mj_reset_data()` | [ ] |
| `core/src/forward/mod.rs` | S5.1: energy guards. S8e/S8g: check call refactor (void, no `?`) | [ ] |
| `core/src/forward/actuation.rs` | S4.2: gravity gravcomp. S4.8: actuation guard. S4.9: clampctrl. S7d: per-group. S8d: ctrl validation | [ ] |
| `core/src/forward/velocity.rs` | S4.8: optional `mj_actuator_velocity()` split | [ ] |
| `core/src/forward/passive.rs` | S4.7: spring/damper refactor + guards. S4.2a: gravcomp routing | [ ] |
| `core/src/forward/check.rs` | S8c/S8g: refactored check functions (void + auto-reset) | [ ] |
| `core/src/forward/acceleration.rs` | S4.14: eulerdamp compound guard | [ ] |
| `core/src/dynamics/rne.rs` | S4.2: gravity guard in `mj_rne()` + `mj_gravcomp()` | [ ] |
| `core/src/energy.rs` | S4.2: gravity guard. S5.1: energy enable guard | [ ] |
| `core/src/sensor/position.rs` | S4.10: `DISABLE_SENSOR` early return | [ ] |
| `core/src/sensor/velocity.rs` | S4.10: `DISABLE_SENSOR` early return | [ ] |
| `core/src/sensor/acceleration.rs` | S4.10: `DISABLE_SENSOR` early return | [ ] |
| `core/src/constraint/mod.rs` | S4.3: constraint guard. S4.11: `warmstart()` function | [ ] |
| `core/src/constraint/assembly.rs` | S4.1: contact instantiation guard. S4.4–S4.6: equality/frictionloss/limit guards. S4.15: `compute_kbip()` call site | [ ] |
| `core/src/constraint/impedance.rs` | S4.15: `compute_kbip()` takes `&Model`, refsafe guard | [ ] |
| `core/src/constraint/contact_params.rs` (new) | S10c: `assign_margin`, `assign_ref`, `assign_imp`, `assign_friction`, `MIN_MU` | [ ] |
| `core/src/collision/mod.rs` | S4.1: collision guard. S4.12: filterparent. S9e: midphase dispatch. S10d: broadphase margin override | [ ] |
| `core/src/collision/mesh_collide.rs` | S9: `use_bvh` parameter on 5 functions, 13 call sites | [ ] |
| `core/src/integrate/mod.rs` | S4.8/S7d: per-actuator disable in Euler activation loop | [ ] |
| `core/src/integrate/rk4.rs` | S4.8/S7d: per-actuator disable in RK4 activation loop (all 4 stages) | [ ] |
| `core/src/island/mod.rs` | S4.13b: island compound guard | [ ] |
| `mjcf/src/types.rs` | S2: `MjcfFlag` field changes, docstrings, defaults | [ ] |
| `mjcf/src/parser.rs` | S2: new attrs. S4.2a: `actuatorgravcomp`. S7b: `actuatorgroupdisable`. S10b: `o_*` attrs | [ ] |
| `mjcf/src/builder/mod.rs` | S3: `apply_flags()`, S3b: stub-only warnings | [ ] |
| `tests/integration/runtime_flags.rs` | AC1–AC48 tests (minus AC32, AC34, AC47) | [ ] |
| `tests/integration/golden_flags.rs` | AC18 golden conformance test | [ ] |
| `tests/scripts/gen_flag_golden.py` (new) | AC18: MuJoCo golden data generation | [ ] |
| `tests/assets/golden/flags/` (new dir) | AC18: `.npy` files + test MJCF | [ ] |

---

## Appendix A: Spec Section → Audit Checkbox Cross-Reference

Every spec sub-section maps to one or more audit checkboxes.

| Spec section | Audit location |
|-------------|---------------|
| S1 (constants) | Phase 1a (bit position table) |
| S1 (helpers) | Phase 1b (`disabled`, `enabled`, `actuator_disabled`) |
| S1 (validation) | Phase 1c (`is_bad`, `MAX_VAL`, `MIN_VAL`) |
| S1 (warnings) | Phase 1d (`Warning` enum, `mj_warning`) |
| S1 (re-exports) | Phase 1e |
| S2a (passive → spring/damper) | Phase 1f (field audit: no `passive`, has `spring`+`damper`) |
| S2b (new fields) | Phase 1f (`fwdinv`, `invdiscrete`, `autoreset`) |
| S2c (parser update) | Phase 1g (attribute parsing) |
| S2d (docstring fixes) | Phase 1f (docstring audit table) |
| S2e (island default) | Phase 1f (`island: true`), Phase 6b (B2) |
| S3 (apply_flags) | Phase 1h (19 disable + 6 enable wiring tables) |
| S3b (stub-only warnings) | Phase 1h (4 warning checks) |
| S4.1 (contact/constraint collision) | Phase 2a (Site 1 + Site 2) |
| S4.2 (gravity) | Phase 2b (4 sites: rne, gravcomp, energy, actuation) |
| S4.2a (gravcomp routing) | Phase 2c (model field, passive-side, actuation-side, interaction matrix) |
| S4.3 (constraint assembly) | Phase 2a (S4.3 section) |
| S4.4 (equality) | Phase 2d |
| S4.5 (frictionloss) | Phase 2d |
| S4.6 (limit) | Phase 2d |
| S4.7-prereq (separate arrays) | Phase 2e — S4.7-prereq |
| S4.7a (passive top-level) | Phase 2e — S4.7a (6 checkboxes) |
| S4.7b (component-level) | Phase 2e — S4.7b (5 write sites) |
| S4.7c (sub-function guards) | Phase 2e — S4.7c |
| S4.7d (contact passive) | Phase 2e — S4.7d |
| S4.7e (aggregation) | Phase 2e — S4.7e |
| S4.8 (actuation) | Phase 2f (4 sites) |
| S4.9 (clampctrl) | Phase 2g |
| S4.10 (sensor) | Phase 2h |
| S4.11 (warmstart) | Phase 2i |
| S4.12 (filterparent) | Phase 2j |
| S4.13 (midphase) | Phase 2k |
| S4.13b (island) | Phase 2l |
| S4.14 (eulerdamp) | Phase 2m |
| S4.15 (refsafe) | Phase 2n |
| S4.16 (autoreset) | Phase 2o |
| S4.17 (nativeccd) | Phase 2p |
| S5.1 (energy) | Phase 2q |
| S5.2 (override) | Phase 2r (model fields, helpers, 6 guard sites) |
| S5.3 (fwdinv) | Phase 2s |
| S5.4 (invdiscrete) | Phase 2s |
| S5.5 (multiccd) | Phase 2s |
| S5.6 (sleep) | Phase 2t |
| S6 (fluid sleep) | Phase 2e — S4.7c (spot-check, §40c already done) |
| S7a (model fields) | Phase 2u — S7a, Phase 3a |
| S7b (builder wiring) | Phase 2u — S7b |
| S7c (actuator_disabled helper) | Phase 2u — S7c, Phase 1b |
| S7d (runtime guards) | Phase 2u — S7d |
| S8a (is_bad) | Phase 4a, Phase 1c |
| S8b (warning system) | Phase 1d, Phase 3b |
| S8c (check functions) | Phase 4b |
| S8d (ctrl validation) | Phase 4e |
| S8e (pipeline integration) | Phase 4f (step orchestration) |
| S8f (mj_reset_data) | Phase 4d (11-category inventory) |
| S8g (migration from Result) | Phase 4c (StepError pruning), Phase 4g (test migration) |
| S9a–S9f (BVH midphase) | Phase 2k (use_bvh, 5 functions, 13 call sites) |
| S10a (model fields) | Phase 2r (model fields), Phase 3a |
| S10b (parser wiring) | Phase 1g (`o_*` attrs) |
| S10c (assignment helpers) | Phase 2r (assign_* functions, MIN_MU) |
| S10d (guard sites) | Phase 2r (6 guard sites) |

---

Each phase produces a pass/fail/discrepancy for every checkbox.
Discrepancies are categorized: **bug** (spec violated), **drift** (implementation
differs but may be intentional), or **gap** (spec item not implemented).
