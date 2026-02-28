# Spec C — Hill-Type Muscle Dynamics: Spec

**Status:** Draft
**Phase:** Roadmap Phase 5 — Actuator Completeness
**Effort:** M
**MuJoCo ref:** `mju_muscleDynamics()` in `engine_util_misc.c`, lines 1482–1504;
`mj_fwdActuation()` in `engine_forward.c`, lines ~742–877
**MuJoCo version:** 3.5.0
**Prerequisites:**
- Spec A — acc0, dampratio, and length-range estimation (landed in `a1cbbba`).
  Provides `Model::compute_actuator_params()` in `muscle.rs:139–346`.
- T1-b — `dynprm` resize to `[f64; 10]` (landed in `d4db634`).
  Provides `actuator_dynprm: Vec<[f64; 10]>`.
- DT-6 — `actearly` verification (landed in `dc12b8b`).
  HillMuscle benefits from existing `actearly` wiring automatically.

> **Conformance mandate:** This spec exists to integrate a higher-fidelity
> Hill-type muscle model into CortenForge's actuation pipeline. It has a split
> conformance posture:
>
> 1. **Activation dynamics** — MUST produce numerically identical results to
>    MuJoCo's `mju_muscleDynamics()`. This is achieved by reusing sim-core's
>    existing `muscle_activation_dynamics()` function, which already matches
>    the MuJoCo C source exactly.
> 2. **Force generation** — is a CortenForge EXTENSION. MuJoCo has only one
>    muscle force model (piecewise-quadratic FLV). HillMuscle adds pennation
>    angle, compliant tendon, and full CE/PE/SE architecture from the
>    `sim-muscle` crate. This is explicitly non-MuJoCo behavior and is
>    documented as such throughout this spec.
>
> The MuJoCo C source code is the single source of truth for activation
> dynamics. The sim-muscle crate is the source of truth for the Hill-model
> force extension.

---

## Problem Statement

**Conformance gap:** MuJoCo has exactly one muscle dynamics type (`mjDYN_MUSCLE`)
and one muscle force model (`mju_muscleGain` / `mju_muscleBias`). CortenForge's
`ActuatorDynamics::Muscle` already maps to this exactly — activation dynamics via
`muscle_activation_dynamics()` (matching `mju_muscleDynamics()`), force via
`GainType::Muscle` / `BiasType::Muscle` (matching MuJoCo's piecewise-quadratic
FL/FV/FP curves). This is conformant and unchanged by this spec.

**Extension gap:** CortenForge's standalone `sim-muscle` crate provides a
higher-fidelity Hill-type muscle model with features MuJoCo does not have:
pennation angle (variable fiber angle affecting force projection), compliant
tendon (series elastic element with Newton-iteration equilibrium), and
separate fiber-state tracking (fiber length/velocity distinct from
musculotendon length/velocity). This model is currently unusable from the
MuJoCo-aligned simulation pipeline — it exists as a standalone crate with
its own `MuscleActuator` trait, disconnected from `mj_fwdActuation()`.

This spec adds `ActuatorDynamics::HillMuscle` — a new dynamics variant that:
1. Reuses MuJoCo-conformant activation dynamics (identical to `Muscle`).
2. Replaces MuJoCo's piecewise-quadratic force model with sim-muscle's
   Hill-type model for force generation.
3. Integrates cleanly into the existing `force = gain * input + bias`
   pipeline without modifying any existing actuator behavior.

---

## MuJoCo Reference

> **This is the most important section of the spec.** It establishes the
> conformant baseline that HillMuscle's activation dynamics must match, and
> the MuJoCo force model that HillMuscle's force generation intentionally
> differs from.

### MuJoCo activation dynamics — `mju_muscleDynamics()`

**Source:** `engine_util_misc.c`, lines 1482–1504 (MuJoCo 3.5.0).

```c
mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, const mjtNum prm[3]) {
  mjtNum ctrlclamp = mju_clip(ctrl, 0, 1);
  mjtNum actclamp = mju_clip(act, 0, 1);
  mjtNum tau_act = prm[0] * (0.5 + 1.5*actclamp);
  mjtNum tau_deact = prm[1] / (0.5 + 1.5*actclamp);
  mjtNum smoothing_width = prm[2];
  mjtNum dctrl = ctrlclamp - act;
  mjtNum tau = mju_muscleDynamicsTimescale(dctrl, tau_act, tau_deact, smoothing_width);
  return dctrl / mjMAX(mjMINVAL, tau);
}
```

**Algorithm:**
1. Clamp `ctrl` to `[0, 1]` and `act` to `[0, 1]`.
2. Compute activation-dependent effective time constants (Millard et al. 2013):
   - `tau_act_eff = prm[0] * (0.5 + 1.5 * act_clamped)` — slower activation at higher activation
   - `tau_deact_eff = prm[1] / (0.5 + 1.5 * act_clamped)` — faster deactivation at higher activation
3. Compute `dctrl = ctrl_clamped - act` (unclamped act for derivative).
4. Select time constant via `mju_muscleDynamicsTimescale()`.
5. Return `dctrl / max(mjMINVAL, tau)`.

**Timescale blending — `mju_muscleDynamicsTimescale()`**
(`engine_util_misc.c`, lines 1465–1478):

```c
mjtNum mju_muscleDynamicsTimescale(mjtNum dctrl, mjtNum tau_act,
                                    mjtNum tau_deact, mjtNum smoothing_width) {
  if (smoothing_width < mjMINVAL) {
    return dctrl >= 0 ? tau_act : tau_deact;       // hard switch
  }
  mjtNum x = mju_clip(0.5*(dctrl/smoothing_width + 1), 0, 1);
  mjtNum smooth = x*x*x * (x*(6*x - 15) + 10);   // quintic Hermite
  return tau_deact + (tau_act - tau_deact) * smooth;
}
```

Quintic Hermite smoothstep: `S(x) = 6x⁵ − 15x⁴ + 10x³`. C²-continuous.

**Parameters:** `dynprm[0]` = τ_act (default 0.01), `dynprm[1]` = τ_deact
(default 0.04), `dynprm[2]` = tausmooth (default 0.0 = hard switch).

**Edge cases:**
- `ctrl` outside `[0,1]`: clamped before use.
- `act` outside `[0,1]`: clamped for time constant computation, but `dctrl`
  uses unclamped `act` for the derivative (so `act` outside `[0,1]` naturally
  drives back toward the valid range).
- `tau < mjMINVAL` (1e-10): floored to `mjMINVAL` to prevent division by zero.
- `tausmooth < mjMINVAL`: hard switch (no blending).
- `dctrl == 0`: `act_dot = 0` (no change).

**Our implementation:** `muscle_activation_dynamics()` in
`sim/L0/core/src/forward/muscle.rs:103–124` matches this exactly, with
`EPS = 1e-10` ≡ `mjMINVAL`. Quintic sigmoid via `sigmoid()` at lines 85–93.

### MuJoCo three-enum actuation architecture — `mj_fwdActuation()`

**Source:** `engine_forward.c` (MuJoCo 3.5.0).

MuJoCo uses THREE independent enums for actuator force computation:

1. **`mjtDyn`** (dynamics) — dispatched at line ~742:
   `case mjDYN_MUSCLE: d->act_dot[i] = mju_muscleDynamics(ctrl, act, prm);`
   Computes `act_dot` (activation derivative). Output: activation `a`.

2. **`mjtGain`** (gain) — dispatched at line ~843:
   `case mjGAIN_MUSCLE: gain = mju_muscleGain(len, vel, lr, acc0, prm);`
   Computes gain (active force coefficient). For muscle: `-F0 * FL(L) * FV(V)`.

3. **`mjtBias`** (bias) — dispatched at line ~877:
   `case mjBIAS_MUSCLE: bias = mju_muscleBias(len, lr, acc0, prm);`
   Computes bias (passive force). For muscle: `-F0 * FP(L)`.

4. **Force formula:** `force = gain * input + bias`
   Where `input` = activation (from dynamics dispatch, potentially with
   `actearly` lookahead).

The `<muscle>` MJCF shortcut sets all three to their Muscle variants. A
`<general>` actuator can mix independently (e.g., `dyntype="muscle"
gaintype="affine"`).

### MuJoCo muscle force model — `mju_muscleGain()`, `mju_muscleBias()`

**For comparison only** — HillMuscle REPLACES this force model.

**`mju_muscleGainLength()`** (`engine_util_misc.c:1359–1384`):
- Active force-length curve: piecewise-quadratic bump, peak 1.0 at L=1.0.
- Returns 0 outside `[lmin, lmax]`. Four segments with smooth transitions.

**`mju_muscleGain()`** (`engine_util_misc.c:1388–1427`):
- `gain = -F0 * FL(L_norm) * FV(V_norm)`
- FL: calls `mju_muscleGainLength()` (above)
- FV: piecewise-quadratic concentric/eccentric (inline, 1408–1418)
- `gainprm[0..8]` = `[range_lo, range_hi, force(F0), scale, lmin, lmax, vmax, fpmax, fvmax]`
- Length normalization: `L_norm = range[0] + (actuator_length - lr[0]) / L0`
  where `L0 = (lr[1] - lr[0]) / (range[1] - range[0])`
- Velocity normalization: `V_norm = actuator_velocity / (L0 * vmax)`

**`mju_muscleBias()`** (`engine_util_misc.c:1431–1461`):
- `bias = -F0 * FP(L_norm)`
- FP: half-quadratic + linear passive force via `muscle_passive_force()`
- Same `gainprm` layout (muscles share `gainprm` for both gain and bias)

### Conformant vs Extension boundary

| Aspect | MuJoCo `mjDYN_MUSCLE` | HillMuscle (this spec) | Conformance |
|--------|----------------------|------------------------|-------------|
| Activation dynamics | `mju_muscleDynamics()` | Reuse `muscle_activation_dynamics()` | **IDENTICAL** — conformant |
| Active force-length | Piecewise-quadratic bump | Gaussian (sim-muscle) | **EXTENSION** — different |
| Force-velocity | Piecewise-quadratic | Hill hyperbolic | **EXTENSION** — different |
| Passive force | Half-quadratic + linear | Exponential rise | **EXTENSION** — different |
| Pennation angle | Not modeled | Variable `cos(α)` projection | **EXTENSION** — MuJoCo has none |
| Tendon compliance | Not modeled | Series elastic element | **EXTENSION** — MuJoCo has none |
| Fiber state | No separate state | `FiberState { length, velocity }` | **EXTENSION** — MuJoCo has none |
| F0 auto-computation | `scale / acc0` | Same formula | **IDENTICAL** — conformant |
| `actearly` | Supported | Supported (via existing wiring) | **IDENTICAL** — conformant |

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) | CortenForge (after Spec C) |
|----------|--------|-----------------------|---------------------------|
| Muscle activation dynamics | `mju_muscleDynamics()` in `engine_util_misc.c:1482` | `muscle_activation_dynamics()` in `muscle.rs:103` — exact match | Unchanged. HillMuscle reuses same function. |
| Muscle force: active | `mju_muscleGain()` piecewise-quadratic FL×FV | `GainType::Muscle` in `actuation.rs:513` — exact match | Unchanged for `Muscle`. HillMuscle uses sim-muscle Gaussian FL × Hill FV. |
| Muscle force: passive | `mju_muscleBias()` half-quadratic FP | `BiasType::Muscle` in `actuation.rs:542` — exact match | Unchanged for `Muscle`. HillMuscle uses sim-muscle exponential FP. |
| Hill-type dynamics | Not implemented (only `mjDYN_MUSCLE`) | Not implemented | New `ActuatorDynamics::HillMuscle` — activation via `muscle_activation_dynamics()`, force via sim-muscle |
| F0 resolution | `set0()` in `engine_setconst.c` | `compute_actuator_params()` Phase 4, `muscle.rs:334` — `Muscle` only | Widened to include `HillMuscle` |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Activation dynamics | `mju_muscleDynamics()`: `τ_eff = τ_base × (0.5 + 1.5×act)` (Millard) | `muscle_activation_dynamics()` in `muscle.rs:103` — identical | Direct port — HillMuscle reuses `muscle_activation_dynamics()`. Do NOT use sim-muscle's `ActivationDynamics::derivative()` (simple `(u−a)/τ`, no activation-dependent scaling). |
| `gainprm` layout | `[range_lo, range_hi, F0, scale, lmin, lmax, vmax, fpmax, fvmax]` (9 used, indices 0–8) | Same layout in `actuator_gainprm: [f64; 10]` | HillMuscle reuses indices 0–3 (range_lo, range_hi, F0, scale). Indices 4–9 are repurposed for Hill-specific parameters (see S3, MJCF Syntax section). |
| `dynprm` layout | `[τ_act, τ_deact, τ_smooth, 0, 0, 0, 0, 0, 0, 0]` | Same: `actuator_dynprm: [f64; 10]`, indices 0–2 used | Direct port — HillMuscle uses same `dynprm` layout as `Muscle`. |
| Length normalization | `L_norm = range[0] + (len − lr[0]) / L0` where `L0 = (lr[1]−lr[0]) / (range[1]−range[0])` | Same formula in `actuation.rs:519–520` | HillMuscle does NOT use MuJoCo's normalized-length formula. Instead: `fiber_length = (actuator_length − tendon_slack_length) / cos(α)`, then `norm_len = fiber_length / optimal_fiber_length`. Use `gainprm[4]` for `optimal_fiber_length` and `gainprm[5]` for `tendon_slack_length`. |
| Velocity normalization | `V_norm = vel / (L0 × vmax)` | Same formula in `actuation.rs:521` | HillMuscle does NOT use MuJoCo's velocity normalization. Instead: `fiber_vel = actuator_velocity / cos(α)`, then `norm_vel = fiber_vel / (optimal_fiber_length × max_contraction_velocity)`. Use `gainprm[4]` for `optimal_fiber_length` and `gainprm[6]` for `max_contraction_velocity` (in L_opt/s). |
| Force sign | `mju_muscleGain()` returns negative: `-F0 × FL × FV` | Same: `gain = -f0 * fl * fv` in `actuation.rs:525` | HillMuscle force computation must return **negative** gain (opposing motion convention). sim-muscle's force functions return positive magnitude — negate when assigning to `gain`. |
| Activation state ownership | `data->act[act_adr]` owned by MuJoCo's integration pipeline | `data.act[act_adr]` owned by sim-core's integration pipeline | sim-core owns activation state via `data.act[act_adr]`. sim-muscle's internal `ActivationState` struct is NOT used — HillMuscle activation is computed by sim-core's `muscle_activation_dynamics()` and stored in `data.act[]`. Do not construct sim-muscle's `ActivationState` or call `ActivationState::update()`. |
| Pennation angle units | N/A (MuJoCo has no pennation) | sim-muscle uses **radians** (`pennation_angle_optimal: f64`, hill.rs:73) | Store as radians in `gainprm[7]`. All `gainprm` values are stored in their native units (radians for angles). No degree→radian conversion during parsing — the user specifies radians directly in the `gainprm` array. |
| Moment arm | N/A (MuJoCo uses transmission system) | sim-core uses `actuator_moment` from `mj_transmission()`. sim-muscle's `HillMuscleConfig::moment_arm` converts fiber force → joint torque. | Set `moment_arm = 1.0` when constructing `HillMuscleConfig` from `gainprm`. All moment arm computation is handled by sim-core's transmission system. Prevents double-application. |

---

## Architecture Decision: Gain/Bias Type for HillMuscle

> **This is the defining architectural decision for Spec C.** The choice
> determines blast radius, code complexity, and conformance isolation.

### Alternatives evaluated

**Option 1: New `GainType::HillMuscle` / `BiasType::HillMuscle` variants**

Adds new enum variants to `GainType` and `BiasType`. Force computation
dispatches to new match arms in `actuation.rs:506` (gain) and `actuation.rs:536`
(bias).

*Pros:* Clean separation — HillMuscle force path is fully independent. The
`GainType::Muscle` path remains untouched.

*Cons:* Large blast radius. Every exhaustive `match` on `GainType` (6 sites)
and `BiasType` (6 sites) requires new arms. The `derivatives.rs` muscle-skip
logic (`GainType::Muscle => continue` at lines 531, 546, 1351, 1529;
`BiasType::Muscle => continue` at lines 534, 552) must be widened to also
skip `HillMuscle`. The `is_muscle` check in `mj_set_length_range()` (line 533)
must also be widened. **12+ match sites** require changes.

**Option 2: Conditional within existing `GainType::Muscle` / `BiasType::Muscle`**

Check `dyntype == HillMuscle` inside the Muscle gain/bias arms. If HillMuscle,
dispatch to sim-muscle; otherwise, standard MuJoCo piecewise-quadratic.

*Pros:* Minimal blast radius — no new enum variants, no new match arms needed
anywhere in derivatives.rs.

*Cons:* Entangles MuJoCo-conformant code with extension code. The `GainType::Muscle`
arm becomes a fork based on `dyntype`, which is architecturally unclean — `GainType`
and `dyntype` are supposed to be independent enum axes.

**Option 3: Compute total force in the dynamics dispatch phase**

HillMuscle's dynamics dispatch computes both `act_dot` AND the total force,
writing directly to `data.actuator_force[i]`. The gain/bias phase is bypassed
(gain=0, bias=0 or similar sentinel).

*Pros:* Zero blast radius on gain/bias enums. All HillMuscle logic is
self-contained in one match arm.

*Cons:* Breaks the `force = gain * input + bias` invariant. Force is
computed in a different pipeline phase, which makes debugging and logging
harder. The gain/bias dispatch would need to be skipped or produce dummy
values.

**Option 4: Use `GainType::User` / `BiasType::User` with per-actuator callbacks**

Register per-actuator callback functions. Architecturally clean but requires
callback infrastructure that doesn't exist yet (current callbacks are
model-global, not per-actuator).

*Pros:* Clean architecture, no enum proliferation.

*Cons:* Requires new callback infrastructure. Heavyweight for a single variant.

### Chosen approach: Option 1 — New `GainType::HillMuscle` / `BiasType::HillMuscle`

**Rationale:**

1. **Conformance isolation.** The MuJoCo-conformant `GainType::Muscle` path
   is completely untouched. No conditional branching within the conformant
   code path — zero risk of accidentally breaking MuJoCo conformance.

2. **Correctness of the `force = gain * input + bias` contract.** HillMuscle
   still produces `force = gain * activation + bias`, where `gain` is the
   active force component (−F0 × FL × FV × cos(α)) and `bias` is the passive
   force component (−F0 × FP × cos(α)). The contract is preserved.

3. **Derivatives pipeline compatibility.** The existing `GainType::Muscle =>
   continue` / `BiasType::Muscle => continue` pattern in `derivatives.rs`
   already uses FD fallback for muscles. HillMuscle is added alongside:
   `GainType::Muscle | GainType::HillMuscle => continue`. Same FD fallback,
   no new analytical derivative code needed.

4. **The blast radius is manageable.** While 12+ match sites need new arms,
   the change is mechanical — every new arm either mirrors the existing
   `Muscle` arm behavior (for skips/fallbacks) or adds the HillMuscle-specific
   force computation (2 sites in `actuation.rs`). No algorithmic complexity
   in any new arm.

**Blast radius quantification:**

| File | GainType sites | BiasType sites | Pattern |
|------|---------------|---------------|---------|
| `actuation.rs:506` | +1 (force computation) | — | New gain arm |
| `actuation.rs:536` | — | +1 (force computation) | New bias arm |
| `derivatives.rs:531` | +1 (`continue`) | — | Widen skip |
| `derivatives.rs:534` | — | +1 (`continue`) | Widen skip |
| `derivatives.rs:546` | +1 (`continue`) | — | Widen skip |
| `derivatives.rs:552` | — | +1 (`continue`) | Widen skip |
| `derivatives.rs:1351` | +1 (FD fallback) | — | Widen FD |
| `derivatives.rs:1529` | +1 (FD fallback) | — | Widen FD |
| `muscle.rs:533` | +1 (`is_muscle` widen) | +1 (`is_muscle` widen) | Widen LR filter |
| `builder/actuator.rs` | +1 (parse) | +1 (parse) | Add string→enum |

Total: 6 GainType arms + 5 BiasType arms + 1 shared `is_muscle` widening = 12 changes.

---

## State Management Decision

> **How does HillMuscle's fiber state persist across timesteps?**

### Alternatives evaluated

**Option 1: Widen `act_num`**

Set `act_num = 2` for HillMuscle. `data.act[act_adr]` = activation (slot 0),
`data.act[act_adr + 1]` = fiber length (slot 1). `act_dot[act_adr + 1]`
carries the fiber length derivative for integration.

*Pros:* Reuses existing infrastructure — `act_dot`, derivatives, RK4
integration all iterate over `act_adr..act_adr+act_num`.

*Cons:* Semantically overloads `act[]` — slot 0 is activation (dimensionless,
[0,1]), slot 1 is fiber length (meters). Different units, different dynamics.
The `actlimited`/`actrange` clamp in `actuation.rs` and `rk4.rs` would apply
the same range to both slots, which is wrong (activation range [0,1] vs fiber
length range [~0.05, ~0.30]).

**Option 2: New per-actuator state array**

Add `data.actuator_fiber_length: Vec<f64>` and
`data.actuator_fiber_velocity: Vec<f64>`.

*Pros:* Clean semantics — fiber state is separate from activation state.

*Cons:* Requires changes to `Data::new()`, `Data::reset()`,
`Data::reset_to_keyframe()`. New arrays are not integrated into the
standard `act_dot` → RK4 integration pipeline. Fiber state integration
must be done manually.

**Option 3: Rigid tendon only (no persistent fiber state)**

For v1, only support `rigid_tendon = true`. With a rigid tendon, fiber
length is deterministically computed from MTU length and pennation angle:
`L_fiber = (L_mt − L_slack) / cos(α)`. No persistent fiber state needed
— `act_num = 1` (activation only), fiber length computed on-the-fly during
force generation.

Fiber velocity is estimated from `actuator_velocity` (MTU velocity):
`V_fiber = V_mt / cos(α)`.

*Pros:* Zero additional state. `act_num = 1` (same as `Muscle`). No changes
to `Data` struct, RK4 integration, or reset logic. The force computation
is self-contained within the gain/bias dispatch — no state management
complexity.

*Cons:* Compliant tendon mode deferred. Users who need tendon compliance
must wait for a future spec.

### Chosen approach: Option 3 — Rigid tendon only, `act_num = 1`

**Rationale:**

1. **Minimal blast radius.** `act_num = 1` means HillMuscle's activation
   state management is identical to `Muscle`. No changes to `Data`, `reset()`,
   `reset_to_keyframe()`, RK4 integration, or derivatives pipeline for state
   management. The only changes are the force computation path.

2. **Rigid tendon is the common case.** sim-muscle defaults to
   `rigid_tendon = true`. MuJoCo's own muscle model has no tendon compliance.
   The rigid tendon path covers the majority of biomechanics use cases
   (fast-twitch muscles, short tendons, stiff tendons).

3. **Compliant tendon requires solving a separate ODE.** The compliant tendon
   path in sim-muscle uses Newton iteration to find fiber-tendon equilibrium
   at each step. This is fundamentally incompatible with the current
   integration pipeline (which assumes `act_dot` is the only activation
   derivative). Properly integrating compliant tendon dynamics requires
   either `act_num = 2` with custom integration, or a separate state array
   with its own integration step. This complexity belongs in a dedicated
   future spec.

4. **Progressive enhancement.** This spec delivers the Hill-type muscle
   force model (Gaussian FL, Hill FV, pennation, exponential passive force)
   as a strictly additive extension. Compliant tendon is a natural follow-on
   that can be added in a future spec without changing any API introduced here.

**Initialization and reset logic:** Because `act_num = 1` and the only
persistent state is activation (`data.act[act_adr]`), HillMuscle has no
additional initialization or reset requirements. `Data::new()`,
`Data::reset()`, and `Data::reset_to_keyframe()` handle HillMuscle
identically to `Muscle`: activation is initialized to `model.actuator_act0[i]`
(typically 0.0) on `new()`/`reset()`, or to the keyframe value on
`reset_to_keyframe()`. No `FiberState`, `prev_mt_length`, or other sim-muscle
internal state is constructed or stored — fiber geometry is computed on-the-fly
from `actuator_length` and `gainprm` during each force evaluation (see S3).
When compliant tendon support is added in a future spec, this section must be
revisited to add fiber state initialization.

**Interaction with P9 (Gain/Bias Type Architecture):** The state management
decision and the force-computation architecture are tightly coupled. Because
P9 chose Option 1 (new `GainType::HillMuscle` / `BiasType::HillMuscle`),
fiber state — if it existed — would be read in the **gain/bias dispatch
phase** (not the dynamics dispatch phase). With rigid tendon (Option 3), there
is no persistent fiber state, so this interaction is trivially satisfied:
the gain dispatch arm (S3) computes fiber length from `actuator_length`
and `gainprm` on-the-fly, with no state read. If a future spec adds
compliant tendon support (persistent `FiberState`), the fiber state would
be read in the `GainType::HillMuscle` arm (consistent with P9's architecture)
and integrated via either widened `act_num` or a separate state array
(consistent with Option 1 or Option 2 from this section).

**Deferred work:** Compliant tendon support tracked as future work. When
implemented, it will require either widening `act_num` or adding a separate
fiber state array. The architectural decision will be made in that spec.

---

## MJCF Syntax & Parameter Layout

### Element syntax

HillMuscle is configured via `<general dyntype="hillmuscle" ...>` only. There
is no `<hillmuscle>` shortcut element — this is a CortenForge extension, not
a MuJoCo convention. Adding a shortcut would imply MuJoCo compatibility that
does not exist.

When `dyntype="hillmuscle"` is parsed, the builder automatically sets:
- `gaintype = GainType::HillMuscle`
- `biastype = BiasType::HillMuscle`
- `actlimited = true`, `actrange = [0, 1]`
- `dynprm` defaults to `[0.01, 0.04, 0.0, ...]` (same as `Muscle`)

This auto-setting behavior is analogous to how `<muscle>` auto-sets
`gaintype = Muscle`, `biastype = Muscle`. The user can override `gaintype`
and `biastype` explicitly if desired (e.g., `dyntype="hillmuscle"
gaintype="affine"` for testing).

### `gainprm` index layout

HillMuscle reuses MuJoCo's muscle `gainprm` layout for indices 0–3 (which are
consumed by `compute_actuator_params()` for F0 resolution and length
normalization) and repurposes indices 4–9 for Hill-specific parameters.

| Index | MuJoCo Muscle | HillMuscle | Field | Default | Unit |
|-------|--------------|------------|-------|---------|------|
| 0 | `range_lo` (normalized length lower) | Same | `range_lo` | 0.75 | — |
| 1 | `range_hi` (normalized length upper) | Same | `range_hi` | 1.05 | — |
| 2 | `F0` (max isometric force, <0 = auto) | Same | `F0` / `max_isometric_force` | −1.0 (auto) | N |
| 3 | `scale` (F0 auto-scale factor) | Same | `scale` | 200.0 | N |
| 4 | `lmin` (FL curve lower) | `optimal_fiber_length` | `optimal_fiber_length` | 0.10 | m |
| 5 | `lmax` (FL curve upper) | `tendon_slack_length` | `tendon_slack_length` | 0.20 | m |
| 6 | `vmax` (max shortening velocity) | `max_contraction_velocity` | `max_contraction_velocity` | 10.0 | L_opt/s |
| 7 | `fpmax` (passive force at lmax) | `pennation_angle` | `pennation_angle_optimal` | 0.0 | **rad** |
| 8 | `fvmax` (eccentric force max) | `tendon_stiffness` | `tendon_stiffness` | 35.0 | — |
| 9 | (unused) | (reserved) | — | 0.0 | — |

**Rationale for index reuse:** Indices 0–3 MUST be preserved because
`compute_actuator_params()` reads `gainprm[0..3]` for all muscle-type
actuators (F0 auto-computation, length normalization for `lengthrange`).
Indices 4–8 are repurposed because their MuJoCo meanings (FL/FV curve
shape parameters) are not applicable to HillMuscle — HillMuscle uses
sim-muscle's curve types which are parameterized differently.

**Overflow strategy:** HillMuscle uses 9 of 10 `gainprm` slots (index 9
reserved). If future extensions (configurable curve shapes, compliant tendon
parameters) require more than 10 parameters, the overflow strategy is:
(a) use `biasprm[0..9]` for additional parameters (HillMuscle currently
does not use `biasprm` for force computation — indices are available), or
(b) use defaults for sub-parameters (e.g., Gaussian FL widths, FV curvature)
and only expose them when explicitly needed. Array widening (`gainprm` to
`[f64; 20]`) is a last resort requiring a separate spec.

### `biasprm` layout

HillMuscle does not use `biasprm` for force parameters. `biasprm` is left
at its default values. The passive force computation reads parameters from
`gainprm` (same as MuJoCo's `Muscle`, where gain and bias share the same
parameter layout). `biasprm` slots are reserved for future overflow if
HillMuscle needs >10 parameters.

### `dynprm` layout

Identical to `Muscle`: `[τ_act, τ_deact, τ_smooth, 0, ..., 0]` at indices
[0, 1, 2]. HillMuscle reuses `muscle_activation_dynamics()` which reads
exactly these three indices.

### MJCF attribute mapping

The `<general>` element's attributes map to `gainprm` indices:

```xml
<general name="hill_muscle"
         dyntype="hillmuscle"
         gainprm="0.75 1.05 -1 200 0.10 0.20 10.0 0.0 35.0"
         dynprm="0.01 0.04 0.0"
         .../>
```

Alternatively, if we add named attributes (future convenience, not required
for v1):
```xml
<general dyntype="hillmuscle"
         gainprm="0.75 1.05 -1 200"
         optlen="0.10" slacklen="0.20" vmax="10.0"
         pennation="0.0" tendonstiff="35.0"
         .../>
```

Named attributes are out of scope for this spec. All Hill-specific parameters
are specified via `gainprm` array indices.

### Default class inheritance

HillMuscle parameters (`gainprm`, `dynprm`, `biasprm`, `actlimited`,
`actrange`) inherit through `<default>` classes via standard MJCF class
inheritance — no special handling is needed. A `<default>` class containing
`<general dyntype="hillmuscle" gainprm="..."/>` applies to all `<general>`
actuators in that class scope, exactly as it does for `<muscle>` defaults.
The builder's existing `<default>` resolution runs before dyntype-specific
auto-setting, so user defaults are respected and only missing fields receive
the HillMuscle auto-set values.

### Builder validation

The builder validates HillMuscle-specific parameters during model compilation:

| Parameter | Valid range | Error if violated |
|-----------|-----------|-------------------|
| `optimal_fiber_length` (`gainprm[4]`) | `> 0.0` | "HillMuscle: optimal_fiber_length must be positive" |
| `tendon_slack_length` (`gainprm[5]`) | `>= 0.0` | "HillMuscle: tendon_slack_length must be non-negative" |
| `max_contraction_velocity` (`gainprm[6]`) | `> 0.0` | "HillMuscle: max_contraction_velocity must be positive" |
| `pennation_angle` (`gainprm[7]`) | `[0, π/2)` | "HillMuscle: pennation_angle must be in [0, π/2)" |
| `tendon_stiffness` (`gainprm[8]`) | `> 0.0` | "HillMuscle: tendon_stiffness must be positive" |

---

## Specification

### S1. Add `ActuatorDynamics::HillMuscle` enum variant

**File:** `core/src/types/enums.rs`
**MuJoCo equivalent:** No direct equivalent — `mjDYN_MUSCLE` is the only
MuJoCo muscle dynamics type. `HillMuscle` is a CortenForge extension.
**Design decision:** New variant placed before `User` (last by convention,
per umbrella §2). No payload — all parameters live in `gainprm`/`dynprm`
arrays (consistent with all other dynamics types).

**After:**
```rust
pub enum ActuatorDynamics {
    None,
    Filter,
    FilterExact,
    Integrator,
    Muscle,
    HillMuscle,  // CortenForge extension: Hill-type muscle with pennation + tendon
    User,
}
```

### S2. Add `GainType::HillMuscle` and `BiasType::HillMuscle` enum variants

**File:** `core/src/types/enums.rs`
**MuJoCo equivalent:** No direct equivalent. MuJoCo has `mjGAIN_MUSCLE` and
`mjBIAS_MUSCLE` only. These new variants handle the Hill-type force model.
**Design decision:** New variants placed before `User` in each enum.

**After:**
```rust
pub enum GainType {
    Fixed,
    Affine,
    Muscle,
    HillMuscle,  // CortenForge extension: Hill-type active force
    User,
}

pub enum BiasType {
    None,
    Affine,
    Muscle,
    HillMuscle,  // CortenForge extension: Hill-type passive force
    User,
}
```

### S3. Add HillMuscle force computation in `mj_fwd_actuation()`

**File:** `core/src/forward/actuation.rs`
**MuJoCo equivalent:** `mju_muscleGain()` in `engine_util_misc.c:1388–1427`
(gain), `mju_muscleBias()` in `engine_util_misc.c:1431–1461` (bias) — but
REPLACED by Hill-type model, not ported.
**Design decision:** The gain arm computes `−F0 × FL(L_norm) × FV(V_norm) ×
cos(α)` (active force). The bias arm computes `−F0 × FP(L_norm) × cos(α)`
(passive force). Both use sim-muscle's curve functions. The `gainprm` →
sim-muscle parameter mapping is performed inline.

**Gain dispatch arm** (add after `GainType::Muscle` arm, around line 526):

```rust
GainType::HillMuscle => {
    let prm = &model.actuator_gainprm[i];
    let lengthrange = model.actuator_lengthrange[i];

    // Parameters from gainprm layout (see MJCF Syntax section)
    let f0 = prm[2];                        // max isometric force (resolved by compute_actuator_params)
    let optimal_fiber_length = prm[4];       // m
    let tendon_slack_length = prm[5];        // m
    let max_contraction_velocity = prm[6];   // L_opt/s
    let pennation_angle = prm[7];            // rad
    // prm[8] = tendon_stiffness (unused in rigid tendon mode)

    // MTU geometry (rigid tendon)
    let mt_length = length;  // actuator_length from transmission
    let fiber_length = (mt_length - tendon_slack_length)
        / pennation_angle.cos().max(1e-10);
    let fiber_velocity = velocity  // actuator_velocity from transmission
        / pennation_angle.cos().max(1e-10);

    // Normalize
    let norm_len = fiber_length / optimal_fiber_length.max(1e-10);
    let norm_vel = fiber_velocity
        / (optimal_fiber_length * max_contraction_velocity).max(1e-10);

    // Force-length (Gaussian, sim-muscle ActiveForceLengthCurve)
    let fl = sim_muscle::curves::active_force_length(norm_len);

    // Force-velocity (Hill hyperbolic, sim-muscle ForceVelocityCurve)
    let fv = sim_muscle::curves::force_velocity(norm_vel);

    // Active gain: negative (opposing motion convention)
    -f0 * fl * fv * pennation_angle.cos()
}
```

**Bias dispatch arm** (add after `BiasType::Muscle` arm, around line 553):

```rust
BiasType::HillMuscle => {
    let prm = &model.actuator_gainprm[i];  // HillMuscle shares gainprm layout
    let f0 = prm[2];
    let optimal_fiber_length = prm[4];
    let tendon_slack_length = prm[5];
    let pennation_angle = prm[7];

    let mt_length = length;
    let fiber_length = (mt_length - tendon_slack_length)
        / pennation_angle.cos().max(1e-10);
    let norm_len = fiber_length / optimal_fiber_length.max(1e-10);

    // Passive force (exponential, sim-muscle PassiveForceLengthCurve)
    let fp = sim_muscle::curves::passive_force_length(norm_len);

    // Passive bias: negative (opposing motion convention)
    -f0 * fp * pennation_angle.cos()
}
```

**Note on sim-muscle dependency:** S3 requires `sim-core` to depend on
`sim-muscle`. This is a workspace-internal dependency (both are in
`sim/L0/`). The dependency is on sim-muscle's curve evaluation functions
only — NOT on `HillMuscle` struct or `MuscleActuator` trait. The specific
public functions needed are:

- `sim_muscle::curves::active_force_length(norm_len: f64) -> f64`
- `sim_muscle::curves::force_velocity(norm_vel: f64) -> f64`
- `sim_muscle::curves::passive_force_length(norm_len: f64) -> f64`

If these functions are not currently public at the required granularity,
S3 implementation must add thin public wrappers in `sim-muscle` that
evaluate the default curves. Alternatively, the curve formulas can be
inlined in `actuation.rs` to avoid the cross-crate dependency (see
implementation note below).

**Implementation note — inlining vs dependency:** The sim-muscle curves
(Gaussian FL, Hill FV, exponential FP) are simple mathematical functions
(~10 lines each). Inlining them in `actuation.rs` avoids adding a
crate dependency and keeps the force computation self-contained. This is
the recommended approach if the sim-muscle public API doesn't cleanly
expose individual curve evaluation. The formulas are:

```rust
/// Gaussian active force-length (sim-muscle default curve).
fn hill_active_fl(norm_len: f64) -> f64 {
    let (w_asc, w_desc) = (0.45, 0.56);
    let (l_min, l_max) = (0.5, 1.6);
    if norm_len < l_min || norm_len > l_max {
        return 0.0;
    }
    let w = if norm_len <= 1.0 { w_asc } else { w_desc };
    let x = (norm_len - 1.0) / w;
    (-x * x).exp()
}

/// Hill hyperbolic force-velocity (sim-muscle default curve).
fn hill_force_velocity(norm_vel: f64) -> f64 {
    let a = 0.25; // curvature
    let fv_max = 1.5; // eccentric plateau
    if norm_vel <= -1.0 {
        0.0
    } else if norm_vel <= 0.0 {
        // Concentric: Hill hyperbola
        a * (1.0 + norm_vel) / (a - norm_vel) * a / (a + 1.0)
    } else {
        // Eccentric: plateau approach
        1.0 + (fv_max - 1.0) * norm_vel / (norm_vel + a)
    }
}

/// Exponential passive force-length (sim-muscle default curve).
fn hill_passive_fl(norm_len: f64) -> f64 {
    let shape = 4.0;
    if norm_len <= 1.0 {
        0.0
    } else {
        (shape * (norm_len - 1.0)).exp_m1() / (shape.exp_m1())
    }
}
```

**Chosen approach: Inline the curve functions in `actuation.rs`.** This avoids
adding a `Cargo.toml` dependency from `sim-core` to `sim-muscle`, keeping the
layer boundaries clean (`sim-muscle` is a standalone library; `sim-core` is
the MuJoCo-aligned pipeline — they should not depend on each other). The three
inline functions above (~30 lines total) are the complete implementation. If
sim-muscle's curves are later updated, the inline copies must be synced — this
is acceptable because the curves are well-defined mathematical functions with
stable formulations.

### S4. Add `HillMuscle` dynamics dispatch in `mj_fwd_actuation()`

**File:** `core/src/forward/actuation.rs`
**MuJoCo equivalent:** `mjDYN_MUSCLE` dispatch in `mj_fwdActuation()`,
`engine_forward.c`, line ~742. HillMuscle activation dynamics are IDENTICAL
to MuJoCo's `mju_muscleDynamics()`.
**Design decision:** The HillMuscle dynamics arm is identical to the existing
`Muscle` arm — it calls `muscle_activation_dynamics()` and supports `actearly`.

**After** (add after `ActuatorDynamics::Muscle` arm, around line 461):

```rust
ActuatorDynamics::HillMuscle => {
    // Identical to Muscle: MuJoCo-conformant activation dynamics.
    let act_adr = model.actuator_act_adr[i];
    data.act_dot[act_adr] =
        muscle_activation_dynamics(ctrl, data.act[act_adr], &model.actuator_dynprm[i]);
    if model.actuator_actearly[i] {
        mj_next_activation(model, i, data.act[act_adr], data.act_dot[act_adr])
    } else {
        data.act[act_adr]
    }
}
```

### S5. Widen F0 resolution in `compute_actuator_params()`

**File:** `core/src/forward/muscle.rs`
**MuJoCo equivalent:** `set0()` in `engine_setconst.c` — F0 auto-computation
applies to all muscle-type actuators.
**Design decision:** The `dyntype == Muscle` guard in Phase 4 (line 336) is
widened to also include `HillMuscle`. Same F0 resolution formula:
`F0 = scale / acc0`.

**Lengthrange integration:** HillMuscle actuators receive lengthrange
estimation via the same pipeline as all other actuators — Phases 1–3b of
`compute_actuator_params()` are not gated on `dyntype`. Phase 1 (limit-based
copy) applies to all transmission types unconditionally. Phase 2 (acc0) runs
for all actuators. Phase 3b (simulation-based `mj_set_length_range`) filters
by `is_muscle` (widened in S6 to include HillMuscle). No changes needed for
Phases 1–3b; only the S6 `is_muscle` widening ensures HillMuscle passes the
mode filter.

**Before** (current code, `muscle.rs:335–345`):
```rust
// --- Phase 4: Resolve muscle F0 ---
for i in 0..self.nu {
    if self.actuator_dyntype[i] != ActuatorDynamics::Muscle {
        continue;
    }
    // ...
}
```

**After:**
```rust
// --- Phase 4: Resolve muscle F0 ---
for i in 0..self.nu {
    if !matches!(
        self.actuator_dyntype[i],
        ActuatorDynamics::Muscle | ActuatorDynamics::HillMuscle
    ) {
        continue;
    }
    if self.actuator_gainprm[i][2] < 0.0 {
        // force < 0 means auto-compute: F0 = scale / acc0
        self.actuator_gainprm[i][2] = self.actuator_gainprm[i][3] / self.actuator_acc0[i];
        // Sync biasprm (muscle layout: gain/bias share parameters)
        self.actuator_biasprm[i][2] = self.actuator_gainprm[i][2];
    }
}
```

### S6. Widen `is_muscle` in `mj_set_length_range()`

**File:** `core/src/forward/muscle.rs`
**MuJoCo equivalent:** `mj_setLengthRange()` mode filtering in
`engine_setconst.c`.
**Design decision:** The `is_muscle` check (line 533) uses `GainType::Muscle ||
BiasType::Muscle`. Since HillMuscle uses `GainType::HillMuscle` /
`BiasType::HillMuscle`, the check must be widened to include the new variants.

**Before** (`muscle.rs:533–534`):
```rust
let is_muscle = model.actuator_gaintype[i] == GainType::Muscle
    || model.actuator_biastype[i] == BiasType::Muscle;
```

**After:**
```rust
let is_muscle = matches!(
    model.actuator_gaintype[i],
    GainType::Muscle | GainType::HillMuscle
) || matches!(
    model.actuator_biastype[i],
    BiasType::Muscle | BiasType::HillMuscle
);
```

### S7. Update derivatives pipeline

**File:** `core/src/derivatives.rs`
**MuJoCo equivalent:** MuJoCo uses FD for all muscle-type derivatives.
**Design decision:** HillMuscle follows the same FD fallback pattern as Muscle.
Every `GainType::Muscle => continue` and `BiasType::Muscle => continue` is
widened to include the HillMuscle variants. Every `is_muscle` check on
`ActuatorDynamics` is widened. The integration derivative dispatch gets a new
arm identical to `Muscle` (approximate boundary-clamped derivatives, consumed
only by non-hybrid paths; the hybrid path uses FD).

**Changes (all mechanical — same pattern as existing Muscle arms):**

| File | Line | Current | After |
|------|------|---------|-------|
| `derivatives.rs` | 531 | `GainType::Muscle` => skip | `GainType::Muscle \| GainType::HillMuscle` => skip |
| `derivatives.rs` | 534 | `BiasType::Muscle` => skip | `BiasType::Muscle \| BiasType::HillMuscle` => skip |
| `derivatives.rs` | 539 | (no change — `None` vs `_ =>`) | HillMuscle falls through to `_ =>` (uses `data.act[act_adr]` — correct) |
| `derivatives.rs` | 546 | `GainType::Muscle \| GainType::User` => continue | `GainType::Muscle \| GainType::HillMuscle \| GainType::User` => continue |
| `derivatives.rs` | 552 | `BiasType::Muscle \| BiasType::User` => continue | `BiasType::Muscle \| BiasType::HillMuscle \| BiasType::User` => continue |
| `derivatives.rs` | 1142–1173 | (integration derivatives) | Add `ActuatorDynamics::HillMuscle =>` arm identical to `Muscle` arm |
| `derivatives.rs` | 1326–1328 | `ActuatorDynamics::Muscle` | `ActuatorDynamics::Muscle \| ActuatorDynamics::HillMuscle` |
| `derivatives.rs` | 1351 | `GainType::Muscle \| GainType::User` => FD | `GainType::Muscle \| GainType::HillMuscle \| GainType::User` => FD |
| `derivatives.rs` | 1529 | `GainType::Muscle \| GainType::User` => FD | `GainType::Muscle \| GainType::HillMuscle \| GainType::User` => FD |

**Non-exhaustive match sites (wildcard arms — no change needed):**

| File | Line | Pattern | Behavior |
|------|------|---------| ---------|
| `integrate/rk4.rs` | 102–110 | `FilterExact` + `_ =>` | HillMuscle falls through to `_ =>` Euler (correct) |
| `derivatives.rs` | 538–540 | `None` + `_ =>` | HillMuscle falls through to `_ =>` act (correct — has activation state) |
| `actuation.rs` | 382 | FilterExact-specific check | No match needed — `HillMuscle` is not FilterExact. This is a non-exhaustive `if` check, not a `match`; HillMuscle correctly does not enter this branch. |

**Builder match sites** (handled in S8, listed here for completeness):

| File | Line | Change |
|------|------|--------|
| `builder/actuator.rs` | 502 | Add `"hillmuscle" => HillMuscle` in `parse_dyntype()` |
| `builder/actuator.rs` | 301 | Add `HillMuscle => 1` in `act_num` dispatch |
| `builder/actuator.rs` | (gaintype/biastype resolution) | Add `"hillmuscle" => HillMuscle` in `parse_gaintype()` / `parse_biastype()` |
| `builder/actuator.rs` | (auto-set logic) | Auto-set gaintype/biastype when `dyntype = HillMuscle` |

### S8. Update MJCF builder

**File:** `mjcf/src/builder/actuator.rs`
**MuJoCo equivalent:** No MuJoCo equivalent (CortenForge extension).
**Design decision:** Parse `"hillmuscle"` in `parse_dyntype()`. Set
`act_num = 1` (same as Muscle — rigid tendon, activation state only).
Auto-set `gaintype` and `biastype` to `HillMuscle` variants. Add
`parse_gaintype()` and `parse_biastype()` support.

**`parse_dyntype()`** (add case, around line 508):
```rust
"hillmuscle" => Ok(ActuatorDynamics::HillMuscle),
```

**`parse_gaintype()`** (add case):
```rust
"hillmuscle" => Ok(GainType::HillMuscle),
```

**`parse_biastype()`** (add case):
```rust
"hillmuscle" => Ok(BiasType::HillMuscle),
```

**`act_num` determination** (line 301, add to existing match):
```rust
ActuatorDynamics::HillMuscle => 1,  // rigid tendon: activation state only
```

This can be combined with the existing arms:
```rust
ActuatorDynamics::Filter
| ActuatorDynamics::FilterExact
| ActuatorDynamics::Integrator
| ActuatorDynamics::Muscle
| ActuatorDynamics::HillMuscle
| ActuatorDynamics::User => 1,
```

**Auto-set gaintype/biastype** when `dyntype = HillMuscle` and gaintype/
biastype are not explicitly specified:

```rust
// After dyntype resolution, before gain/bias finalization:
if dyntype == ActuatorDynamics::HillMuscle {
    if gaintype_not_explicitly_set {
        gaintype = GainType::HillMuscle;
    }
    if biastype_not_explicitly_set {
        biastype = BiasType::HillMuscle;
    }
    // Default dynprm (same as Muscle)
    if dynprm_not_explicitly_set {
        dynprm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    }
    // Default gainprm for HillMuscle
    if gainprm_not_explicitly_set {
        gainprm = [0.75, 1.05, -1.0, 200.0, 0.10, 0.20, 10.0, 0.0, 35.0, 0.0];
    }
    // Enforce actlimited + actrange [0,1]
    actlimited = true;
    actrange = (0.0, 1.0);
}
```

**Validation** (during model compilation):
```rust
if dyntype == ActuatorDynamics::HillMuscle {
    let opt_len = gainprm[4];
    let slack_len = gainprm[5];
    let vmax = gainprm[6];
    let penn = gainprm[7];
    let stiff = gainprm[8];

    if opt_len <= 0.0 {
        return Err("HillMuscle: optimal_fiber_length (gainprm[4]) must be positive");
    }
    if slack_len < 0.0 {
        return Err("HillMuscle: tendon_slack_length (gainprm[5]) must be non-negative");
    }
    if vmax <= 0.0 {
        return Err("HillMuscle: max_contraction_velocity (gainprm[6]) must be positive");
    }
    if penn < 0.0 || penn >= std::f64::consts::FRAC_PI_2 {
        return Err("HillMuscle: pennation_angle (gainprm[7]) must be in [0, π/2)");
    }
    if stiff <= 0.0 {
        return Err("HillMuscle: tendon_stiffness (gainprm[8]) must be positive");
    }
}
```

---

## Acceptance Criteria

### AC1: HillMuscle activation dynamics match MuJoCo — hard switch *(runtime test, analytically derived)*

**Given:** `muscle_activation_dynamics(ctrl=0.8, act=0.3, prm=[0.01, 0.04, 0.0, 0,0,0,0,0,0,0])`.
This is called for HillMuscle via the same code path as Muscle.
**After:** Direct function call (no step needed).
**Assert:** `act_dot = 52.631578...` ± 1e-10.
**Field:** Return value of `muscle_activation_dynamics()`.
**Derivation:** V1 from rubric: `act_clamped=0.3`, `tau_act=0.01×0.95=0.0095`,
`dctrl=0.5>0` → activating → `tau=0.0095`, `act_dot=0.5/0.0095=52.6315789...`.

### AC2: HillMuscle activation dynamics — smooth blend *(runtime test, analytically derived)*

**Given:** `muscle_activation_dynamics(ctrl=0.55, act=0.5, prm=[0.01, 0.04, 0.2, 0,0,0,0,0,0,0])`.
**After:** Direct function call.
**Assert:** `act_dot = 2.798737...` ± 1e-6.
**Field:** Return value.
**Derivation:** V7 from rubric: `act_clamped=0.5`, `tau_act=0.0125`,
`tau_deact=0.032`, `dctrl=0.05`, `x=0.625`, `S(0.625)=0.724792...`,
`tau=0.017867...`, `act_dot=0.05/0.017867...=2.7987...`.

### AC3: HillMuscle force at isometric optimal length *(runtime test, analytically derived)*

**Given:** One hinge joint + one HillMuscle actuator with `gainprm = [0.75,
1.05, 1000.0, 200.0, 0.10, 0.20, 10.0, 0.0, 35.0, 0.0]` (F0=1000N,
L_opt=0.10m, L_slack=0.20m, pennation=0°, vmax=10 L_opt/s). Joint
transmission gear=1.0, limited range=[−0.5, 0.5]. `actuator_length` = 0.30
(= L_slack + L_opt = isometric at optimal). `actuator_velocity` = 0.0
(isometric). Activation = 1.0 (full activation).
**After:** Evaluate gain and bias dispatch.
**Assert:** `gain = −1000.0 × 1.0 × 1.0 × cos(0) = −1000.0` ± 1e-8.
`bias = −1000.0 × 0.0 × cos(0) = 0.0` (FP=0 at L_norm=1.0).
`force = gain × activation + bias = −1000.0 × 1.0 + 0.0 = −1000.0` ± 1e-8.
**Field:** `data.actuator_force[0]`.
**Derivation:** At optimal fiber length, FL=1.0 (Gaussian peak). At zero
velocity, FV=1.0 (Hill FV(0)=1). At zero pennation, cos(0)=1. Passive force
is zero at L_norm=1.0. So active force = −F0 and passive = 0.

### AC4: HillMuscle force with pennation angle *(runtime test, analytically derived)*

**Given:** Same as AC3 but `pennation_angle = 0.349066` (20° in radians,
`gainprm[7] = 0.349066`).
**After:** Evaluate gain dispatch.
**Assert:** `gain = −1000.0 × 1.0 × 1.0 × cos(0.349066)` = `−1000.0 × 0.93969...`
= `−939.69...` ± 1e-4.
**Field:** Gain output.
**Derivation:** `cos(20°) = cos(0.349066) = 0.93969...`. At optimal length
with zero velocity, FL=1 and FV=1, so gain = −F0 × cos(α).

### AC5: HillMuscle F0 auto-computation *(runtime test, analytically derived)*

**Given:** HillMuscle actuator with `gainprm[2] = −1.0` (auto-compute),
`gainprm[3] = 200.0` (scale). Model has known `acc0` after
`compute_actuator_params()`.
**After:** `compute_actuator_params()`.
**Assert:** `gainprm[2] = 200.0 / acc0` ± 1e-10. `biasprm[2] = gainprm[2]`.
**Field:** `model.actuator_gainprm[i][2]`, `model.actuator_biasprm[i][2]`.

### AC6: HillMuscle dynamics dispatch reuses `muscle_activation_dynamics()` *(runtime test)*

**Given:** Model with one `Muscle` actuator and one `HillMuscle` actuator,
both with identical `dynprm = [0.01, 0.04, 0.0, ...]`, identical `ctrl`,
identical initial `act`.
**After:** One call to `mj_fwd_actuation()`.
**Assert:** `act_dot` for both actuators are identical (± 0.0, exact match).
**Field:** `data.act_dot[muscle_act_adr]` == `data.act_dot[hill_act_adr]`.

### AC7: HillMuscle with `actearly = true` *(runtime test)*

**Given:** HillMuscle actuator with `actearly = true`, `ctrl = 1.0`,
initial `act = 0.0`.
**After:** `mj_fwd_actuation()`.
**Assert:** `input` (the value used for force computation) is the actearly-
predicted next-step activation, NOT the current activation 0.0. Specifically,
`input > 0.0`.
**Field:** `data.actuator_force[i]` != 0 (force is nonzero because input > 0).

### AC8: Existing Muscle actuators unaffected *(regression test)*

**Given:** Model with only standard `Muscle` actuators (no HillMuscle).
**After:** Full simulation step.
**Assert:** All `actuator_force`, `act_dot`, and `qfrc_actuator` values are
identical to pre-Spec-C values (± 0.0, exact match).
**Field:** All actuator output fields.

### AC9: Mixed Muscle + HillMuscle model *(regression test)*

**Given:** Model with one `Muscle` actuator AND one `HillMuscle` actuator
on the same joint.
**After:** Full simulation step.
**Assert:** The `Muscle` actuator produces identical `act_dot`, `gain`,
`bias`, and `force` as it would in a model without the HillMuscle actuator
present.
**Field:** `data.actuator_force[muscle_idx]`, `data.act_dot[muscle_act_adr]`.

### AC10: MJCF parsing round-trip *(runtime test)*

**Given:** MJCF XML containing `<general dyntype="hillmuscle" gainprm="0.75
1.05 -1 200 0.10 0.20 10.0 0.349066 35.0">`.
**After:** Parse model.
**Assert:**
- `actuator_dyntype[i] = ActuatorDynamics::HillMuscle`
- `actuator_gaintype[i] = GainType::HillMuscle`
- `actuator_biastype[i] = BiasType::HillMuscle`
- `actuator_gainprm[i][4] = 0.10` (optimal_fiber_length)
- `actuator_gainprm[i][7] = 0.349066` (pennation in radians)
- `actuator_act_num[i] = 1`
- `actuator_actlimited[i] = true`
- `actuator_actrange[i] = (0.0, 1.0)`
**Field:** Model actuator arrays.

### AC11: Builder validation rejects invalid parameters *(runtime test)*

**Given:** MJCF XML with `<general dyntype="hillmuscle" gainprm="0.75 1.05
-1 200 0.0 0.20 10.0 0.0 35.0">` (optimal_fiber_length = 0.0, invalid).
**After:** Parse model.
**Assert:** Parse fails with error containing "optimal_fiber_length must be positive".
**Field:** Error result.

### AC12: No `#[allow(unreachable_patterns)]` in modified files *(code review)*

Every new match arm is explicit. No unreachable pattern suppression added
anywhere.

### AC13: HillMuscle arm present in all exhaustive match sites *(code review)*

All 8 exhaustive `ActuatorDynamics` match sites, 6 `GainType` match sites,
and 6 `BiasType` match sites have explicit arms for the new variants. No
match site is left with a catch-all `_ =>` that silently swallows the new
variants.

### AC14: `lengthrange` mode filtering includes HillMuscle *(runtime test)*

**Given:** HillMuscle actuator with `LengthRangeMode::Muscle`.
**After:** `mj_set_length_range()`.
**Assert:** The actuator is NOT skipped by mode filtering (it passes the
`is_muscle` check).
**Field:** `model.actuator_lengthrange[i]`.

### AC15: HillMuscle force with zero lengthrange *(runtime test, analytically derived)*

**Given:** HillMuscle actuator with `lengthrange = (0.0, 0.0)` (e.g., unlimited
joint, no limit copy). `actuator_length = 0.30`, `optimal_fiber_length = 0.10`,
`tendon_slack_length = 0.20`, pennation = 0°.
**After:** Evaluate gain dispatch.
**Assert:** No division-by-zero or NaN. HillMuscle force path uses
`fiber_length / optimal_fiber_length` — not MuJoCo's `L0` normalization —
so zero lengthrange does not cause division-by-zero. Force = −F0 × FL(1.0) × FV(0) × cos(0) = −F0.
**Field:** `data.actuator_force[0]` is finite and matches AC3.

### AC16: Mixed gain/bias type override *(runtime test)*

**Given:** MJCF with `<general dyntype="hillmuscle" gaintype="affine"
gainprm="100 0 0">`. User explicitly overrides `gaintype` away from `HillMuscle`.
**After:** Parse model + `mj_fwd_actuation()`.
**Assert:** `actuator_dyntype[i] = ActuatorDynamics::HillMuscle`,
`actuator_gaintype[i] = GainType::Affine` (not `HillMuscle`). Activation
dynamics still use `muscle_activation_dynamics()`. Force uses Affine gain
formula (`gainprm[0]`) rather than Hill force.
**Field:** Model enum fields + `data.actuator_force[i]`.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (activation hard switch) | T1 | Direct |
| AC2 (activation smooth blend) | T2 | Direct |
| AC3 (isometric optimal force) | T3 | Direct |
| AC4 (pennation angle) | T4 | Direct |
| AC5 (F0 auto-computation) | T5 | Direct |
| AC6 (dynamics dispatch equivalence) | T6 | Direct |
| AC7 (actearly) | T7 | Direct |
| AC8 (Muscle regression) | T8 | Regression |
| AC9 (mixed model) | T9 | Integration |
| AC10 (MJCF parsing) | T10 | Direct |
| AC11 (validation) | T11 | Direct |
| AC12 (no unreachable_patterns) | — | Code review (manual) |
| AC13 (match-site exhaustiveness) | — | Code review (manual) |
| AC14 (lengthrange mode) | T12 | Direct |
| AC15 (zero lengthrange) | T13 | Edge case |
| AC16 (mixed gain/bias override) | T14 | Edge case |

---

## Test Plan

### T1: Activation dynamics — hard switch path → AC1

Model: Direct function call, no model needed.
```rust
let prm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
let act_dot = muscle_activation_dynamics(0.8, 0.3, &prm);
assert!((act_dot - 52.631578947368421).abs() < 1e-10);
```
Expected value: analytically derived from MuJoCo C source (V1 from rubric).
Tolerance: 1e-10 (matches `mjMINVAL`).

### T2: Activation dynamics — smooth blend path → AC2

```rust
let prm = [0.01, 0.04, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
let act_dot = muscle_activation_dynamics(0.55, 0.5, &prm);
assert!((act_dot - 2.798737).abs() < 1e-5);
```
Expected value: analytically derived (V7 from rubric). Tolerance: 1e-5
(floating-point representation of quintic Hermite intermediate values).

### T3: HillMuscle force — isometric at optimal length → AC3

Model: single hinge + HillMuscle actuator. F0=1000, L_opt=0.10, L_slack=0.20,
pennation=0°, vmax=10. Set `actuator_length=0.30`, `actuator_velocity=0.0`,
activation=1.0.

Assert: `actuator_force = −1000.0 ± 1e-8`.

Why: At optimal fiber length, FL(1.0) = exp(0) = 1.0. At zero velocity,
FV(0) = 1.0 (Hill FV). Passive FP(1.0) = 0. cos(0) = 1.
`force = (−1000 × 1 × 1 × 1) × 1.0 + (−1000 × 0 × 1) = −1000`.

### T4: HillMuscle force — pennation angle effect → AC4

Same as T3 but `pennation_angle = 0.349066` (20°).

Assert: `gain ≈ −939.69 ± 1e-2`.
`cos(20°) = 0.93969...`, so gain = −1000 × 1 × 1 × 0.93969 = −939.69.

### T5: F0 auto-computation for HillMuscle → AC5

Model: HillMuscle actuator with `gainprm[2] = −1.0` (auto F0),
`gainprm[3] = 200.0` (scale).

After `compute_actuator_params()`: `gainprm[2] = 200.0 / acc0`.

Assert: `gainprm[2] > 0` and `biasprm[2] == gainprm[2]`.

### T6: Dynamics dispatch equivalence — Muscle vs HillMuscle → AC6

Model: two actuators on the same joint, one `Muscle` and one `HillMuscle`,
identical `dynprm` and initial state.

After `mj_fwd_actuation()`:
Assert: `act_dot[muscle_adr] == act_dot[hill_adr]` (exact, ± 0.0).

### T7: Actearly interaction → AC7

Model: HillMuscle actuator with `actearly = true`. `ctrl = 1.0`, initial
`act = 0.0`.

After `mj_fwd_actuation()`:
Assert: `actuator_force != 0.0` (because actearly predicts nonzero next-step
activation, which becomes the input for force computation).

### T8: Muscle regression — no behavioral change → AC8

Run existing muscle test suite. All pre-Spec-C expected values must match
exactly.

### T9: Mixed Muscle + HillMuscle model → AC9

Model: one `Muscle` actuator and one `HillMuscle` actuator. Run a few steps.

Assert: The Muscle actuator's `act_dot` and `force` are identical to a
model-only-Muscle reference run.

### T10: MJCF parsing — `dyntype="hillmuscle"` → AC10

Parse:
```xml
<general name="hill1" dyntype="hillmuscle"
         gainprm="0.75 1.05 -1 200 0.10 0.20 10.0 0.349066 35.0"/>
```

Assert all fields per AC10.

### T11: Builder validation — invalid parameters → AC11

Parse with `optimal_fiber_length = 0.0` → expect error.
Parse with `pennation = 1.6` (> π/2) → expect error.
Parse with `max_contraction_velocity = −1.0` → expect error.

### T12: Length-range mode filtering → AC14

Model: HillMuscle actuator. `LengthRangeMode::Muscle`.

Assert: actuator is not skipped by `mj_set_length_range` mode filter (the
widened `is_muscle` check includes `GainType::HillMuscle`).

### T13: HillMuscle with zero lengthrange → AC15

Model: HillMuscle actuator on an unlimited joint (no joint limits, no tendon
limits). `lengthrange = (0.0, 0.0)`. Set `actuator_length = 0.30` (L_slack +
L_opt), `actuator_velocity = 0.0`.

Assert: `actuator_force` is finite (no NaN/Inf) and matches the expected
isometric force from AC3. HillMuscle's fiber-geometry normalization avoids
MuJoCo's `L0 = (lr[1]−lr[0]) / ...` path entirely — zero lengthrange is safe.

### T14: Mixed gain/bias type override → AC16

Parse:
```xml
<general name="hybrid" dyntype="hillmuscle" gaintype="affine"
         gainprm="100 0 0" dynprm="0.01 0.04 0.0"/>
```

Assert:
- `actuator_dyntype = HillMuscle`
- `actuator_gaintype = Affine` (user override honored, NOT auto-set to HillMuscle)
- `actuator_biastype = HillMuscle` (not overridden — auto-set applies)
- After `mj_fwd_actuation()` with `ctrl=1.0`: gain uses Affine formula
  (`gainprm[0] = 100`), not Hill force.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Zero pennation angle (α=0) | Degenerates: `cos(0)=1`, fiber force = MTU force. Most common case. | T3 | AC3 |
| Nonzero pennation (α=20°) | cos(α) < 1 reduces force. Validates projection. | T4 | AC4 |
| Zero lengthrange (lr[0] == lr[1]) | L0=0 in MuJoCo normalization causes division-by-zero. HillMuscle uses `fiber_length / optimal_fiber_length` instead — not affected. But `actuator_length` may be at an extreme making fiber_length ≤ 0. | T13 | AC15 |
| `ctrl` outside [0,1] | Clamped by `muscle_activation_dynamics()`. Same as Muscle. | T1 (V6 from rubric: ctrl=1.5) | AC1 |
| `act` at boundaries 0.0, 1.0 | Boundary time constants (V3, V4 from rubric). | T1 (add V3, V4 cases) | AC1 |
| `tausmooth > 0` | Quintic sigmoid blend instead of hard switch. Different codepath. | T2 | AC2 |
| `actearly = true` | Next-step activation used for force. | T7 | AC7 |
| F0 auto-computation (`gainprm[2] < 0`) | `F0 = scale / acc0`. Regression against Spec A. | T5 | AC5 |
| MJCF round-trip | `"hillmuscle"` string → enum → correct types. | T10 | AC10 |
| Invalid parameters | Validation catches bad inputs. | T11 | AC11 |
| Mixed Muscle + HillMuscle | Regression: existing Muscle behavior unchanged. | T8, T9 | AC8, AC9 |
| `LengthRangeMode::Muscle` with HillMuscle | Mode filter must include HillMuscle. | T12 | AC14 |
| Rigid tendon only (compliant tendon N/A) | Compliant tendon is out of scope for this spec. Only rigid tendon (`fiber_length = (L_mt − L_slack) / cos(α)`) is supported. N/A — no test needed; documented as out-of-scope deferred work. | — | — |
| Mixed gain/bias type override | `dyntype="hillmuscle" gaintype="affine"` — user overrides auto-set gain/bias type. The builder should honor the explicit override. The actuator then uses HillMuscle activation dynamics but Affine gain (no Hill force). | T14 | AC16 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T1 additional cases (V2–V6) | Additional activation dynamics verification points | Comprehensive coverage of rubric verification data — anchors conformance across boundary conditions |
| Hill FL curve unit test | `hill_active_fl()` at L=0.5, 1.0, 1.6, 0.49, 1.61 | Verifies Gaussian curve shape matches sim-muscle |
| Hill FV curve unit test | `hill_force_velocity()` at V=−1, 0, 0.5, −0.5 | Verifies Hill hyperbolic matches sim-muscle |
| Hill FP curve unit test | `hill_passive_fl()` at L=0.9, 1.0, 1.5 | Verifies exponential passive matches sim-muscle |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| New `ActuatorDynamics::HillMuscle` variant | Not available | New dynamics type for Hill-type muscle | N/A — extension | No existing code affected | None — additive |
| New `GainType::HillMuscle` variant | Not available | New gain type for Hill active force | N/A — extension | No existing code affected | None — additive |
| New `BiasType::HillMuscle` variant | Not available | New bias type for Hill passive force | N/A — extension | No existing code affected | None — additive |
| `compute_actuator_params()` Phase 4 | F0 only for `Muscle` | F0 for `Muscle \| HillMuscle` | Toward completeness | No existing behavior changes | None — widened guard |
| `mj_set_length_range()` `is_muscle` | `GainType::Muscle \|\| BiasType::Muscle` | Includes `HillMuscle` variants | Toward completeness | No existing behavior changes | None — widened check |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `core/src/types/enums.rs` | +3 enum variants (ActuatorDynamics, GainType, BiasType) | +6 |
| `core/src/forward/actuation.rs` | +1 dynamics arm, +1 gain arm, +1 bias arm | +60 |
| `core/src/forward/muscle.rs` | Widen F0 guard (line 336), widen `is_muscle` (line 533) | ~6 modified |
| `core/src/derivatives.rs` | Widen 8 match arms (6 GainType/BiasType skips + 1 is_muscle + 1 integration) | ~16 modified |
| `core/src/integrate/rk4.rs` | No changes needed (wildcard arm handles HillMuscle) | 0 |
| `mjcf/src/builder/actuator.rs` | +3 parse cases, +1 act_num arm, auto-set logic, validation | +40 |
| `core/src/forward/actuation.rs` (or new file) | Hill curve functions (FL, FV, FP) — inline or import | +30 |
| Test files (new) | T1–T12 + supplementary | +200 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `muscle_tests::*` (17 tests) | `forward/muscle.rs` | Pass (unchanged) | Tests use `ActuatorDynamics::Muscle`, not `HillMuscle` |
| `actuation` integration tests | `forward/actuation.rs` | Pass (unchanged) | Tests use existing actuator types |
| Phase 4 regression suite (39 tests) | `conformance/phase4.rs` | Pass (unchanged) | Phase 5 does not modify Phase 4 code paths |
| Builder/parser tests | `builder/actuator.rs` | Pass (unchanged) | Existing parse cases unmodified |
| Derivatives tests | `derivatives.rs` | Pass (unchanged) | Existing match arms unmodified (only widened) |
| Full sim domain baseline (2,148+ tests) | various | Pass (unchanged) | Extension only — no existing behavior modified |

**Test files that construct `ActuatorDynamics` values (must add HillMuscle test cases):**

| File | Line(s) | Context | Required change |
|------|---------|---------|-----------------|
| `forward/muscle.rs` | 822, 1077+ | Test helper model builders | Add HillMuscle model builder variant for T3–T7, T12–T14 |
| `builder/actuator.rs` | 927+ | MJCF parsing test suite | Add `dyntype="hillmuscle"` round-trip tests (T10, T11, T14) |
| `callbacks.rs` | integration tests | User callback tests | Verify HillMuscle does not interfere with `GainType::User` / `BiasType::User` callbacks |
| `derivatives.rs` | integration tests | Derivative tests | Verify HillMuscle FD fallback produces finite derivatives (T8, T9 regression) |

**Zero expected breakage.** All changes are additive (new enum variants, new
match arms). No existing match arm is modified, only widened with `|` patterns.
No `#[allow(unreachable_patterns)]` is added anywhere — every new arm is
explicit.

---

## Execution Order

1. **S1 + S2** (enum variants) → These are prerequisites for all other sections.
   Compile-check: `cargo build -p sim-core` will show all exhaustive match
   sites that need new arms.

2. **S8** (MJCF builder) → Parse `"hillmuscle"`, auto-set gain/bias types,
   `act_num`. Test: T10, T11 (parsing + validation).

3. **S4** (dynamics dispatch) → Add HillMuscle activation dynamics arm.
   Test: T1, T2, T6 (activation dynamics match Muscle exactly).

4. **S3** (gain/bias force computation) → Add HillMuscle force computation.
   Test: T3, T4 (force values at known states).

5. **S5** (F0 resolution) → Widen Phase 4 guard. Test: T5.

6. **S6** (lengthrange `is_muscle`) → Widen mode filter. Test: T12.

7. **S7** (derivatives) → Widen all skip/fallback arms. Test: T8, T9
   (regression — existing Muscle actuators unaffected).

After each section lands, run `cargo test -p sim-core -p sim-mjcf` to verify
no regressions.

---

## Out of Scope

- **Compliant tendon mode** — requires persistent fiber state (`act_num ≥ 2`
  or separate state array) and custom integration. Deferred to a future spec.
  *Conformance impact: none — MuJoCo has no compliant tendon.*

- **Named MJCF attributes** (`optlen`, `slacklen`, `pennation`) — convenience
  attributes for HillMuscle parameters. All parameters are accessible via
  `gainprm` indices. Named attributes are a UX improvement, not a
  functionality gap. Deferred.

- **`<hillmuscle>` shortcut element** — would be analogous to `<muscle>`.
  Not justified for a CortenForge extension with no MuJoCo precedent. Deferred.

- **Variable pennation angle** — sim-muscle computes pennation angle as a
  function of fiber length (`α = asin(w / L_fiber)` where `w = L_opt × sin(α₀)`).
  This spec uses the constant optimal pennation angle (`α₀`) for simplicity.
  Variable pennation is a refinement that can be added without API changes.
  *Conformance impact: none — MuJoCo has no pennation model.*

- **Configurable curve parameters** — sim-muscle's `MuscleForceCurves` has
  tunable shape parameters (Gaussian widths, FV curvature, etc.). This spec
  uses the default curve shapes. Configurable curves can be added by extending
  the `gainprm` layout (would require `biasprm` for overflow) or a separate
  parameter block. Deferred.

- **New `GainType::User` / `BiasType::User` callback infrastructure** —
  evaluated as Option 4 for the gain/bias architecture decision but rejected
  in favor of explicit enum variants. The callback system can be revisited
  in a future spec if needed.
