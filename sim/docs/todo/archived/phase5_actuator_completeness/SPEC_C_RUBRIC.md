# Spec C — Hill-Type Muscle Dynamics: Spec Quality Rubric

Grades the SPEC_C spec on 11 criteria. Target: A+ on every criterion before
implementation begins. A+ means "an implementer could build this without asking
a single clarifying question — and the result would produce numerically identical
output to MuJoCo for the MuJoCo-conformant subset, with clean extension for
Hill-specific behavior."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
P2–P11 but has P1 wrong is worse than useless: it would produce a clean,
well-tested implementation of the wrong behavior.

**Spec C uniqueness:** Unlike Specs A/B/D, which each port a specific MuJoCo C
function, Spec C is a CortenForge *extension*. MuJoCo has only one muscle
dynamics type (`mjDYN_MUSCLE`). Our existing `ActuatorDynamics::Muscle` already
maps to it exactly. Spec C adds `ActuatorDynamics::HillMuscle` — a higher-
fidelity variant that integrates the standalone `sim-muscle` crate's Hill-type
model (pennation angle, compliant tendon, full CE/PE/SE architecture) into the
sim-core actuation pipeline.

This means P1 has a split mandate:
1. **Activation dynamics**: Must match MuJoCo's `mju_muscleDynamics()` exactly
   (Millard 2013 with asymmetric time constants) — conformance is non-negotiable.
2. **Force generation**: Extends beyond MuJoCo's piecewise-quadratic FLV model
   by adding pennation, tendon compliance, and full fiber-state tracking from
   `sim-muscle`. This is explicitly non-MuJoCo behavior and must be documented
   as a CortenForge extension.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Empirical Ground Truth

### MuJoCo muscle activation dynamics (MuJoCo 3.5.0)

MuJoCo's `mju_muscleDynamics()` in `engine_util_misc.c` (lines 1482–1504)
implements Millard et al. (2013) activation dynamics:

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

- `dynprm[0]` = tau_act base (default 0.01)
- `dynprm[1]` = tau_deact base (default 0.04)
- `dynprm[2]` = tausmooth (default 0.0)
- Activation-dependent scaling: `tau_act_eff = prm[0] * (0.5 + 1.5 * act_clamped)`
- Hard switch at `dctrl == 0` when `tausmooth < mjMINVAL`; quintic sigmoid blend otherwise
- Output: `d(act)/dt` = `dctrl / max(mjMINVAL, tau)`

Our `muscle_activation_dynamics()` in `muscle.rs:103–124` matches this exactly
with `EPS = 1e-10` ≡ `mjMINVAL`.

**Timescale blending — `mju_muscleDynamicsTimescale()`** (engine_util_misc.c:1465–1478):

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

Quintic Hermite smoothstep: `S(x) = 6x⁵ − 15x⁴ + 10x³`. At `x=0`: `S=0`
(pure deactivation). At `x=0.5`: `S=0.5` (midpoint blend). At `x=1`: `S=1`
(pure activation).

**Analytical verification data:**

All values below are analytically derived from the C source code above. They
should be verified against MuJoCo Python bindings during implementation.

**Hard switch path (tausmooth=0):**

| # | ctrl | act | prm | tau_act_eff | tau_deact_eff | dctrl | tau (selected) | act_dot |
|---|------|-----|-----|-------------|---------------|-------|----------------|---------|
| V1 | 0.8 | 0.3 | [0.01, 0.04, 0.0] | 0.0095 | 0.042105… | +0.5 (activating) | 0.0095 | **52.631578…** |
| V2 | 0.2 | 0.7 | [0.01, 0.04, 0.0] | 0.0155 | 0.025806… | −0.5 (deactivating) | 0.025806… | **−19.375** |
| V3 | 1.0 | 0.0 | [0.01, 0.04, 0.0] | 0.005 | 0.08 | +1.0 (activating) | 0.005 | **200.0** |
| V4 | 0.0 | 1.0 | [0.01, 0.04, 0.0] | 0.025 | 0.02 | −1.0 (deactivating) | 0.02 | **−50.0** |
| V5 | 0.5 | 0.5 | [0.01, 0.04, 0.0] | 0.0125 | 0.032 | 0.0 | (irrelevant) | **0.0** |
| V6 | 1.5 | 0.3 | [0.01, 0.04, 0.0] | 0.0095 | 0.042105… | +0.7 (ctrl clamped to 1.0) | 0.0095 | **73.684210…** |

**Smooth blend path (tausmooth>0):**

| # | ctrl | act | prm | dctrl | x | S(x) | tau (blended) | act_dot |
|---|------|-----|-----|-------|---|------|---------------|---------|
| V7 | 0.55 | 0.5 | [0.01, 0.04, 0.2] | +0.05 | 0.625 | 0.724792… | 0.017867… | **2.7987…** |

**Derivation for V1:** `actclamp = 0.3`, `tau_act = 0.01 × (0.5 + 1.5×0.3) =
0.01 × 0.95 = 0.0095`. `dctrl = 0.8 − 0.3 = 0.5 > 0` → activating → `tau =
tau_act = 0.0095`. `act_dot = 0.5 / 0.0095 = 52.631578…`.

**Derivation for V2:** `actclamp = 0.7`, `tau_deact = 0.04 / (0.5 + 1.5×0.7) =
0.04 / 1.55 = 0.025806…`. `dctrl = 0.2 − 0.7 = −0.5 < 0` → deactivating →
`tau = tau_deact = 0.025806…`. `act_dot = −0.5 / 0.025806… = −19.375`.

**Derivation for V3 (boundary):** `actclamp = 0.0`, `tau_act = 0.01 × 0.5 =
0.005`. `dctrl = 1.0`. `act_dot = 1.0 / 0.005 = 200.0`.

**Derivation for V4 (boundary):** `actclamp = 1.0`, `tau_deact = 0.04 / 2.0 =
0.02`. `dctrl = −1.0`. `act_dot = −1.0 / 0.02 = −50.0`.

**Derivation for V7 (smooth blend):** `actclamp = 0.5`, `tau_act = 0.01 ×
(0.5 + 0.75) = 0.0125`, `tau_deact = 0.04 / 1.25 = 0.032`. `dctrl = 0.05`.
`x = clip(0.5 × (0.05/0.2 + 1), 0, 1) = 0.625`. `S(0.625) = 6×0.625⁵ −
15×0.625⁴ + 10×0.625³ = 0.572204… − 2.288818… + 2.441406… = 0.724792…`.
`tau = 0.032 + (0.0125 − 0.032) × 0.724792… = 0.017867…`. `act_dot =
0.05 / 0.017867… = 2.7987…`.

### MuJoCo muscle force model (for comparison — not for HillMuscle conformance)

MuJoCo's `mju_muscleGain()` in `engine_util_misc.c` (lines 1388–1427):
- `gain = -F0 * FL(L) * FV(V)` (piecewise-quadratic, no pennation, no tendon compliance)
- `prm[0..8]` = [range_lo, range_hi, force, scale, lmin, lmax, vmax, fpmax, fvmax]
- FL: `mju_muscleGainLength()` (lines 1359–1384) — piecewise-quadratic bump
- FV: inline in `mju_muscleGain()` — piecewise-quadratic concentric/eccentric

MuJoCo's `mju_muscleBias()` in `engine_util_misc.c` (lines 1431–1461):
- `bias = -F0 * FP(L)` (passive force, half-quadratic + linear)

### MuJoCo actuation pipeline dispatch (engine_forward.c)

In `mj_fwdActuation()`:
- **Dynamics dispatch** (line ~742): `case mjDYN_MUSCLE: d->act_dot[..] = mju_muscleDynamics(ctrl, act, prm);`
- **Gain dispatch** (line ~843): `case mjGAIN_MUSCLE: gain = mju_muscleGain(len, vel, lr, acc0, prm);`
- **Bias dispatch** (line ~877): `case mjBIAS_MUSCLE: bias = mju_muscleBias(len, lr, acc0, prm);`
- **Force formula**: `force = gain * input + bias` where `input` = activation

MuJoCo uses THREE independent enums: `mjtDyn` (dynamics), `mjtGain` (gain), `mjtBias` (bias).
The `<muscle>` shortcut sets all three to their Muscle variants. A `<general>` actuator can mix
independently (e.g., `dyntype="muscle" gaintype="affine"`).

### HillMuscle extension (sim-muscle crate)

The `sim-muscle` crate at `sim/L0/muscle/` provides:
- **Pennation angle** — fiber force projected through `cos(α)`, variable α
- **Compliant tendon** — series elastic element with slack length
- **Fiber state tracking** — separate fiber length/velocity from MTU length via
  `FiberState { length: f64, velocity: f64 }` (hill.rs:217–224)
- **Activation dynamics** — first-order with asymmetric τ (**WARNING:** sim-muscle
  uses simple `(u-a)/τ` without activation-dependent scaling; NOT the same as
  MuJoCo's Millard model — see convention conflict note below)
- **Full force computation** — combines activation × FL × FV × cos(α) + passive

Key types: `HillMuscle`, `HillMuscleConfig`, `FiberState`, `ActivationDynamics`,
`MuscleForceCurves`, `MuscleActuator` trait.

**`HillMuscleConfig` fields** (hill.rs:61–97) — these are the parameters P10
must map to `gainprm`/`biasprm` indices:

| Field | Type | Default | Unit | Description |
|-------|------|---------|------|-------------|
| `max_isometric_force` | f64 | 1000.0 | N | Maximum force at optimal length, zero velocity |
| `optimal_fiber_length` | f64 | 0.10 | m | Fiber length at maximum active force |
| `tendon_slack_length` | f64 | 0.20 | m | Length below which tendon bears no load |
| `pennation_angle_optimal` | f64 | 0.0 | **rad** | Fiber angle at optimal length |
| `max_contraction_velocity` | f64 | 10.0 | L_opt/s | Maximum shortening velocity |
| `activation_dynamics` | `ActivationDynamics` | (see below) | — | Time constants for activation/deactivation |
| `force_curves` | `MuscleForceCurves` | (defaults) | — | FL, FP, FV curve parameters |
| `moment_arm` | f64 | 0.05 | m | **WARNING**: sim-core uses transmission-based moment arms via `actuator_moment`, not this field |
| `tendon_stiffness` | f64 | 35.0 | — | Higher = stiffer (>100 ≈ rigid) |
| `rigid_tendon` | bool | true | — | If true, skip compliant tendon solve |

**Critical convention conflict — `moment_arm`:** sim-muscle's `HillMuscleConfig`
has a `moment_arm: f64` field for converting fiber force to joint torque. But
sim-core computes moment arms via the transmission system (`mj_transmission` →
`actuator_moment`). If HillMuscle uses both, the moment arm is double-applied.
The spec MUST resolve this: either (a) ignore `HillMuscleConfig::moment_arm`
and use sim-core's transmission-based moment arm, or (b) set moment_arm=1.0
when constructing `HillMuscleConfig` from `gainprm`.

**sim-muscle `ActivationDynamics` divergence:** sim-muscle's `ActivationDynamics`
(activation.rs:39–50) uses simple `(u - a) / tau` WITHOUT activation-dependent
time constants. MuJoCo's `mju_muscleDynamics` uses `tau_eff = tau_base × (0.5 +
1.5×act)`. For HillMuscle in sim-core, activation dynamics MUST use sim-core's
`muscle_activation_dynamics()` (which matches MuJoCo), NOT sim-muscle's
`ActivationDynamics::derivative()`. The `activation_dynamics` field in
`HillMuscleConfig` is not used for HillMuscle's activation in sim-core.

**State management in sim-muscle:** `HillMuscle` owns a `FiberState` (fiber length +
velocity) and a `prev_mt_length: f64` (previous musculotendon length for velocity
estimation). These are per-actuator persistent state that must survive across
timesteps. sim-core's `data.act[act_adr]` provides only 1 scalar (activation).
If HillMuscle needs fiber length as additional state, `act_num` must increase
to accommodate it, or a separate storage mechanism is needed.

### Existing CortenForge codebase state

**Exhaustive `match` sites on `ActuatorDynamics` that MUST add `HillMuscle` arms:**

| File | Line | Context | Required arm behavior |
|------|------|---------|-----------------------|
| `forward/actuation.rs` | 448 | Phase 1 dynamics dispatch (act_dot) | Reuse `muscle_activation_dynamics()` — identical to `Muscle` arm |
| `forward/actuation.rs` | 382 | FilterExact-specific check | No match needed — `HillMuscle` is not FilterExact |
| `forward/muscle.rs` | 336 | F0 resolution in `compute_actuator_params()` Phase 4 | Add HillMuscle arm — same F0 resolution logic as Muscle |
| `builder/actuator.rs` | 188 | `dyntype` from shortcut type | No change — HillMuscle is only via `<general dyntype="hillmuscle">` |
| `builder/actuator.rs` | 301 | `act_num` from dynamics type | Add `HillMuscle => N` (activation + optional fiber state) |
| `builder/actuator.rs` | 502 | `parse_dyntype()` string→enum | Add `"hillmuscle" => HillMuscle` |
| `derivatives.rs` | 1142 | Integration derivatives dispatch | Add HillMuscle arm — same as Muscle (approximate, FD fallback) |
| `derivatives.rs` | 1326–1328 | `is_muscle` flag for FD fallback | Widen to `ActuatorDynamics::Muscle \| ActuatorDynamics::HillMuscle` |

**Non-exhaustive match sites (wildcard arms — confirm no change needed):**

| File | Line | Pattern | Behavior |
|------|------|---------| ---------|
| `integrate/rk4.rs` | 102–110 | `FilterExact` + `_ =>` | HillMuscle falls through to `_ =>` Euler (correct) |
| `derivatives.rs` | 538–540 | `None` + `_ =>` | HillMuscle falls through to `_ =>` act (correct — has activation state) |
| `derivatives.rs` | 531–534 | `GainType::Muscle` / `BiasType::Muscle` skip | **DECISION NEEDED**: does HillMuscle use `GainType::Muscle`? If not, new GainType skip needed |
| `derivatives.rs` | 546, 552 | `GainType::Muscle` / `BiasType::Muscle` continue | Same — depends on GainType/BiasType decision |

**Test files that construct `ActuatorDynamics` values (must add HillMuscle cases):**

| File | Line(s) | Context |
|------|---------|---------|
| `forward/muscle.rs` | 822, 1077+ | Test helper model builders — may need HillMuscle variants |
| `builder/actuator.rs` | 927+ | MJCF parsing test suite — needs `dyntype="hillmuscle"` round-trip |
| `callbacks.rs` | integration tests | User callback tests — verify HillMuscle does not interfere |
| `derivatives.rs` | integration tests | Derivative tests — verify HillMuscle FD fallback |

**Critical architectural decision — GainType/BiasType for HillMuscle:**

The `GainType::Muscle` path in `actuation.rs:513–526` hardcodes MuJoCo's
piecewise-quadratic `FL * FV` formula. If HillMuscle uses a different force
model (pennation, compliant tendon), it CANNOT reuse `GainType::Muscle`.

Options the spec must evaluate:
1. **New GainType/BiasType variants** → cascade: `actuation.rs` (2 sites), `derivatives.rs` (4+ sites), `builder/actuator.rs` (parsing), `muscle.rs` (`is_muscle` checks). Major blast radius.
2. **HillMuscle computes total force in dynamics dispatch** → bypass gain/bias entirely, write `data.actuator_force[i]` directly. Minimal blast radius but breaks `force = gain * input + bias` invariant.
3. **Reuse `GainType::Muscle`/`BiasType::Muscle` with conditional** → check `dyntype == HillMuscle` inside the Muscle gain/bias path to dispatch differently. Smallest blast radius but entangles MuJoCo-conformant path with extension.
4. **Use `GainType::User`/`BiasType::User`** → delegate to callbacks. Requires callback infrastructure per-actuator. Architecturally clean but heavyweight.

The `is_muscle` check in `muscle.rs:533–534` (`mj_set_length_range`) uses
`GainType::Muscle || BiasType::Muscle` — if HillMuscle uses new gain/bias types,
this check must be widened.

**`GainType`/`BiasType` exhaustive match sites (if new variants added):**

| File | Line | Context |
|------|------|---------|
| `forward/actuation.rs` | 506 | Gain dispatch |
| `forward/actuation.rs` | 536 | Bias dispatch |
| `derivatives.rs` | 531, 534 | Muscle skip in `mjd_actuator_vel` |
| `derivatives.rs` | 544, 550 | Gain/bias velocity derivatives |
| `derivatives.rs` | 1343, 1351 | Gain dispatch in `mjd_state_transition` |
| `derivatives.rs` | 1521, 1529 | Gain dispatch in `mjd_inverse_FD` |
| `forward/muscle.rs` | 533–534 | `is_muscle` in `mj_set_length_range` |
| `builder/actuator.rs` | varies | Gain/bias type resolution |

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade it first and hardest. If P1 is not A+, do not proceed to
> grading other criteria until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. For Spec C, this has a split
> mandate: (1) activation dynamics MUST match `mju_muscleDynamics()` exactly,
> (2) force generation is a documented CortenForge extension that goes beyond
> MuJoCo's piecewise-quadratic FLV model.

| Grade | Bar |
|-------|-----|
| **A+** | Spec cites `mju_muscleDynamics()` (engine_util_misc.c:1482–1504) with exact algorithm: ctrl/act clamping to [0,1], activation-dependent time constants `tau_act_eff = prm[0] * (0.5 + 1.5*act)` / `tau_deact_eff = prm[1] / (0.5 + 1.5*act)`, quintic sigmoid blending via `mju_muscleDynamicsTimescale()` (1465–1478), `dctrl / max(mjMINVAL, tau)` output. Spec cites `mju_muscleGain()` (1388–1427), `mju_muscleBias()` (1431–1461), and `mju_muscleGainLength()` (1359–1384) for *comparison* — documenting what MuJoCo's standard muscle does so the spec can articulate how HillMuscle DIFFERS. Spec documents MuJoCo's three-enum architecture in `mj_fwdActuation()` (engine_forward.c): `mjtDyn` (line ~742 dispatch), `mjtGain` (line ~843 dispatch), `mjtBias` (line ~877 dispatch) — with the `force = gain * input + bias` formula. Spec explicitly distinguishes: (a) HillMuscle activation = IDENTICAL to MuJoCo's `mju_muscleDynamics()`, reusing `muscle_activation_dynamics()`; (b) HillMuscle force = CortenForge EXTENSION (pennation, compliant tendon, CE/PE/SE), NOT conformant to `mju_muscleGain`/`mju_muscleBias`. Edge cases documented: ctrl clamping [0,1], act clamping [0,1], tau floor at `mjMINVAL` (1e-10), `tausmooth < mjMINVAL` hard-switch, `actearly` interaction with HillMuscle, `gainprm` parameter layout [0..8]. C code snippet included for `mju_muscleDynamics()`. |
| **A** | MuJoCo functions cited correctly from C source. Minor gaps: three-enum architecture not fully documented, or conformant/extension boundary is implicit. |
| **B** | Correct at high level but missing conformant/extension split, or behavior described from docs rather than C source. |
| **C** | Partially correct. MuJoCo behavior misunderstood or spec implies HillMuscle activation diverges from MuJoCo without justification. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously in real Rust. For HillMuscle,
> this covers: (1) enum variant addition, (2) MJCF parsing, (3) activation
> dynamics dispatch, (4) force computation bridge to sim-muscle, (5) F0/acc0
> integration in `compute_actuator_params()`, (6) every match-site update.

| Grade | Bar |
|-------|-----|
| **A+** | All algorithms in real Rust: (1) `ActuatorDynamics::HillMuscle` variant added to `enums.rs` (before `User`, per umbrella convention §2); (2) MJCF parsing: `parse_dyntype()` handles `"hillmuscle"` → `HillMuscle`, `act_num` matches state management decision (see P11), `<general>` shortcut only (no `<hillmuscle>` element — extension, not MuJoCo shortcut); (3) activation dynamics: `actuation.rs` Phase 1 dispatch reuses `muscle_activation_dynamics()` with identical match arm to `Muscle`; (4) force computation: the **gain/bias type decision is explicit** — spec states which `GainType`/`BiasType` HillMuscle uses, how the force computation differs from `GainType::Muscle`, and the exact Rust code for the HillMuscle force path (parameter mapping from `actuator_gainprm`/`actuator_lengthrange` → sim-muscle inputs, sign convention, force output); (5) `compute_actuator_params()`: HillMuscle gets F0 resolution (same `scale/acc0` formula as Muscle) and lengthrange (same pipeline); (6) every match-site arm documented (see P7). An implementer can type this in without reading MuJoCo source or sim-muscle internals. |
| **A** | Algorithms complete. One or two details left implicit (e.g., exact parameter mapping from `gainprm` to `HillMuscleConfig`). |
| **B** | Algorithm structure clear but force computation bridge hand-waved ("delegate to sim-muscle"). |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses conventions where MuJoCo's muscle pipeline,
> CortenForge's sim-core actuation pipeline, and the sim-muscle crate all
> meet. Convention mismatches between these three layers cause silent bugs.

| Grade | Bar |
|-------|-----|
| **A+** | Convention difference table with porting rules covering: (1) `actuator_gainprm[0..8]` → HillMuscleConfig parameter mapping (range_lo/range_hi, F0, scale, lmin, lmax, vmax, fpmax, fvmax → optimal_fiber_length, max_isometric_force, max_contraction_velocity, etc.); (2) length normalization — sim-core uses `actuator_length` + `lengthrange` to compute normalized length `L = range[0] + (len - lr[0]) / L0`; sim-muscle uses absolute fiber length with separate `optimal_fiber_length`; porting rule specifies exact conversion; (3) velocity normalization — sim-core normalizes by `L0 * vmax`; sim-muscle uses `max_contraction_velocity` in L_opt/s; porting rule; (4) force sign — MuJoCo `mju_muscleGain` returns negative (`-F0*FL*FV`); sim-muscle `HillMuscle::compute_torque()` returns positive torque; porting rule negates; (5) activation state — sim-core owns `data.act[act_adr]`; sim-muscle has internal `ActivationState`; porting rule specifies who owns activation and how it's synchronized; (6) `dynprm` layout — `[tau_act, tau_deact, tausmooth, ...]` at indices [0,1,2]; same for HillMuscle (reuses `muscle_activation_dynamics`); (7) pennation angle units — sim-muscle uses **radians** (`pennation_angle_optimal: f64` at hill.rs:73); if `gainprm` stores degrees, porting rule converts; (8) moment arm conflict — sim-muscle's `HillMuscleConfig::moment_arm` converts fiber force to joint torque; sim-core uses transmission-based `actuator_moment` from `mj_transmission()`; porting rule states which is used and how double-application is prevented (e.g., set `moment_arm = 1.0` in config). Each rule verified to produce correct results. No empty cells. |
| **A** | Major conventions documented. Minor parameter-mapping gaps. |
| **B** | Some conventions noted, others not — risk of sign or normalization mismatch. |
| **C** | sim-muscle code used without adaptation to sim-core conventions. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. For HillMuscle, runtime ACs
> verify: (a) activation dynamics match MuJoCo exactly (reuses same function),
> (b) force generation produces correct Hill-model output, (c) pipeline
> integration (dispatch, transmission, F0 resolution) works end-to-end.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has the three-part structure: (1) concrete input model/state, (2) exact expected value or tolerance, (3) field to check. **At least one activation dynamics AC uses V1–V7 from this rubric's Empirical Ground Truth** — e.g., "V1: `muscle_activation_dynamics(ctrl=0.8, act=0.3, prm=[0.01, 0.04, 0.0])` produces `act_dot = 52.631578…`; we assert ± 1e-12." **At least one AC exercises the smooth blend path** (V7: `tausmooth=0.2`, expected `act_dot = 2.7987…`). All values verified against MuJoCo during implementation. **At least one Hill force AC has analytically-derived expected values** from known FL/FV/pennation calculation (e.g., isometric at optimal length → force = F0 × 1.0 × 1.0 × cos(0) = F0). **At least one F0 auto-computation AC** verifies `gainprm[2] = scale/acc0` for HillMuscle actuator (regression against Spec A's `compute_actuator_params`). **At least one state management AC** verifies fiber state persistence across timesteps (if applicable — see P11). Code-review ACs explicitly labeled with structural properties (e.g., "no `#[allow(unreachable_patterns)]` in any modified match block", "HillMuscle arm present in all 8+ exhaustive match sites"). |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague. |
| **C** | ACs are aspirational statements, not tests. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions. At least
> one MuJoCo conformance test for activation dynamics. Edge cases specific to
> Hill-model physics.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present — every AC maps to ≥1 test, every test maps to ≥1 AC or is in Supplementary Tests table with justification. Explicit edge case inventory: (1) zero lengthrange (lr[0] == lr[1] — L0 = 0, division guard), (2) `actearly` flag interaction with HillMuscle dynamics, (3) ctrl outside [0,1] (clamped by `muscle_activation_dynamics`), (4) act at boundaries 0.0 and 1.0, (5) rigid vs compliant tendon (both paths exercised), (6) zero pennation angle (degenerates: `cos(0) = 1`, fiber force = MTU force), (7) F0 auto-computation (`gainprm[2] < 0`), (8) HillMuscle with non-muscle gain/bias type (if architecturally possible — what happens?), (9) MJCF parsing round-trip (`dyntype="hillmuscle"` → `ActuatorDynamics::HillMuscle`), (10) fiber state initialization and persistence across multiple steps (if applicable — see P11), (11) `tausmooth > 0` smooth blending path (quintic sigmoid — tests the `mju_muscleDynamicsTimescale` codepath), (12) mixed Muscle + HillMuscle model — both actuator types in the same model, verify existing `Muscle` actuators produce identical results with or without HillMuscle present. Negative cases: existing `Muscle` actuators unaffected (regression). At least one MuJoCo conformance test for activation dynamics using rubric verification data (V1–V7, covering both hard-switch and smooth-blend paths). |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with Spec A's
> `compute_actuator_params()`, the sim-muscle crate, and the `dynprm` resize
> are explicitly stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order is unambiguous. Spec A dependency stated with commit hash `a1cbbba` and exact function: `Model::compute_actuator_params()` in `muscle.rs:139–346` (Contract 2 from umbrella). T1-b dependency stated: `dynprm` is `[f64; 10]` (commit `d4db634`). sim-muscle crate dependency specified: whether sim-core gains a `Cargo.toml` dependency on sim-muscle (workspace member), whether it's optional (feature-gated), and what public API surface is used. DT-6 `actearly` interaction noted (already wired, commit `dc12b8b` — HillMuscle benefits automatically). Each spec section states what it requires from prior sections. If GainType/BiasType changes are needed, the ordering between enum changes and dispatch changes is explicit. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break. For Spec C, the primary risk is the new enum
> variant cascade — and the *scope* of that cascade depends on the GainType/
> BiasType architectural decision.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. **sim-core files:** `enums.rs` (+1 `ActuatorDynamics` variant; +GainType/BiasType variants if needed), `actuation.rs` (+dynamics dispatch arm line 448, +force computation path — lines 506/536 if new gain/bias variants), `muscle.rs` (+F0 resolution arm line 336, +`is_muscle` widening line 533 if needed), `derivatives.rs` (+integration derivatives arm line 1142, +`is_muscle` widening line 1326, +gain/bias skips lines 531/534/544/550/1343/1351/1521/1529 if new variants). **sim-mjcf files:** `builder/actuator.rs` (+`parse_dyntype "hillmuscle"` line 502, +`act_num` arm line 301, +gain/bias resolution if HillMuscle has a shortcut). **sim-muscle (if dependency added):** no changes to sim-muscle itself (sim-core depends on it, not vice versa). **No behavioral changes to existing paths** — this adds a variant; existing `Muscle`, `Filter`, etc. match arms are untouched. Existing test impact: **zero breakage** (extension only). Phase 4 39-test suite and full 2,148+ baseline confirmed unaffected. Wildcard `_ =>` arms in `rk4.rs:107` and `derivatives.rs:540` confirmed to handle HillMuscle correctly (falls through to Euler integration and act-based input respectively). **If new GainType/BiasType variants are added:** all 8+ match sites listed in Empirical Ground Truth section must add arms — spec enumerates each with file, line number, and exact new arm behavior. **Match-site exhaustiveness:** spec enumerates all 8 exhaustive `ActuatorDynamics` sites (from Empirical Ground Truth table) with exact new arm behavior for each. Wildcard arms listed separately with confirmation they handle HillMuscle correctly. **Test files** that construct `ActuatorDynamics` values accounted for: `muscle.rs` test helpers (lines 822, 1077+), `builder/actuator.rs` tests (line 927+), `callbacks.rs` integration tests, `derivatives.rs` integration tests. The spec confirms no `#[allow(unreachable_patterns)]` is added anywhere — every arm is explicit. |
| **A** | File list complete. Most match sites identified. |
| **B** | File list present but incomplete. Some match sites missed. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology
> throughout. The conformant/extension boundary is drawn consistently.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: `HillMuscle` (enum variant name) everywhere — not "Hill", "HillType", or "HillMuscleType". The conformant/extension boundary stated identically in MuJoCo Reference, Algorithm, Convention Notes, and Test Plan. File paths match between Specification sections and Files Affected table. AC numbers match between AC section and Traceability Matrix. Edge cases consistent between MuJoCo Reference and Test Plan. The GainType/BiasType decision is consistent across all sections that reference force computation (no section assumes `GainType::Muscle` while another assumes a new variant). Consumer/caller counts match between sections. `dynprm` and `gainprm` parameter indices cited identically everywhere. `act_num` value consistent between P2 (algorithm), P10 (MJCF syntax), and P11 (state management). |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Gain/Bias Type Architecture *(domain-specific)*

> The defining architectural decision for Spec C: how does HillMuscle compute
> force within the existing `force = gain * input + bias` framework? MuJoCo's
> muscle force uses `GainType::Muscle` for active force (FL × FV) and
> `BiasType::Muscle` for passive force. HillMuscle needs a different force
> model. The spec must resolve this with a justified design decision, because
> the choice determines the blast radius (0–8+ additional match sites) and
> the complexity of the implementation.

| Grade | Bar |
|-------|-----|
| **A+** | Spec evaluates at least 3 alternatives for the force computation path: (1) new `GainType::HillMuscle`/`BiasType::HillMuscle` variants — full blast radius analysis of 8+ match sites, (2) conditional within existing `GainType::Muscle`/`BiasType::Muscle` path (check `dyntype == HillMuscle`) — minimal blast radius but entangles conformant/extension code, (3) bypass gain/bias by computing force directly in the dynamics dispatch or a new phase — breaks the `force = gain * input + bias` invariant but keeps gain/bias enums unchanged, (4) use `GainType::User`/`BiasType::User` with per-actuator callback — architecturally clean but heavyweight. For the chosen approach: trade-offs stated, blast radius quantified, interaction with `derivatives.rs` muscle-skip logic documented, interaction with `mj_set_length_range()` `is_muscle` check documented. |
| **A** | Architecture decided and justified. One alternative not considered. |
| **B** | Architecture stated but not justified, or multiple approaches left unresolved. |
| **C** | No architectural discussion. |

### P10. MJCF Syntax & Parameter Layout *(domain-specific)*

> HillMuscle is a CortenForge extension with no MuJoCo MJCF precedent.
> The spec must design the XML syntax for configuring HillMuscle actuators
> and define the `gainprm`/`biasprm` parameter layout that maps sim-core's
> fixed-size arrays to `HillMuscleConfig` fields. Getting the parameter
> layout wrong means every HillMuscle actuator silently uses wrong muscle
> properties.

| Grade | Bar |
|-------|-----|
| **A+** | Spec defines: (1) **Element syntax** — HillMuscle is configured via `<general dyntype="hillmuscle" ...>` only (no `<hillmuscle>` shortcut element — extension, not MuJoCo convention). If a shortcut IS proposed, full justification with implications for `gaintype`/`biastype` auto-setting and builder dispatch. (2) **`gainprm` index layout** — complete mapping from `actuator_gainprm[0..9]` to HillMuscle-specific parameters: which indices hold `optimal_fiber_length`, `tendon_slack_length`, `pennation_angle_optimal`, `max_contraction_velocity`, `max_isometric_force`, etc. Indices that overlap with MuJoCo's muscle `gainprm[0..8]` layout explicitly documented (reuse where possible, deviate with justification). If HillMuscle needs more than 10 parameters (the `gainprm` array size), the overflow strategy is documented (e.g., use `biasprm` for additional parameters, use defaults for sub-parameters like force curve shape, or propose array widening). (3) **`biasprm` layout** — if HillMuscle uses `biasprm` for passive force parameters, index mapping defined. (4) **`dynprm` layout** — reuses `[tau_act, tau_deact, tausmooth]` at indices [0,1,2] (identical to `Muscle`). (5) **Builder validation** — what errors if HillMuscle-specific parameters are missing or out of range (e.g., `optimal_fiber_length <= 0`, `pennation_angle >= π/2`). (6) **Default class inheritance** — HillMuscle parameters inherit through `<default>` classes (standard MJCF behavior, no special handling needed). An implementer can write the MJCF, parse it, and construct `HillMuscleConfig` without ambiguity. |
| **A** | Syntax and layout defined. Minor gaps in validation rules. |
| **B** | Syntax described but parameter layout incomplete or ambiguous. |
| **C** | No MJCF syntax discussion — "parse like muscle." |

### P11. State Management *(domain-specific)*

> HillMuscle may require per-actuator persistent state beyond the single
> scalar `data.act[act_adr]` that MuJoCo's `mjDYN_MUSCLE` uses. The sim-muscle
> crate's `HillMuscle` owns a `FiberState { length, velocity }` and a
> `prev_mt_length` — three additional scalars per actuator. The spec must
> resolve where this state lives in sim-core, how it's initialized, and how
> it interacts with the integration and derivative pipelines.

| Grade | Bar |
|-------|-----|
| **A+** | Spec resolves the state management question with a justified decision from at least 2 alternatives: (1) **Widen `act_num`** — set `act_num = 2` or `act_num = 3` for HillMuscle so `data.act[act_adr..act_adr+N]` holds `[activation, fiber_length, ...]`. Pro: reuses existing infrastructure (`act_dot`, derivatives, RK4 integration all iterate over `act_adr..act_adr+act_num`). Con: semantically overloads `act[]` — slot 0 is activation, slot 1 is fiber length (different units, different dynamics). (2) **New per-actuator state array** — add `data.actuator_fiber_state: Vec<FiberState>` or similar. Pro: clean semantics. Con: new field requires changes to `Data::new()`, `Data::reset()`, `Data::reset_to_keyframe()`, serialization, and integration pipeline. (3) **Delegate to sim-muscle** — sim-muscle owns `FiberState` internally; sim-core passes MTU length each step and sim-muscle updates fiber state. Pro: minimal sim-core changes. Con: sim-core loses control of integration (can't use RK4), state invisible to derivatives pipeline. For the chosen approach: initialization logic documented (`Data::new()`, `Data::reset()`, **and `Data::reset_to_keyframe()`** must set fiber state to `FiberState::at_optimal(optimal_fiber_length)` — see Spec D's `mj_resetDataKeyframe` precedent), interaction with `derivatives.rs` documented (does fiber state appear in the Jacobian?), interaction with RK4 integration documented (is fiber state integrated alongside activation?), **interaction with P9 documented** (the force-computation architecture determines WHERE fiber state is read — if P9 chooses "bypass gain/bias" then fiber state is read in dynamics dispatch; if P9 chooses "new GainType::HillMuscle" then fiber state is read in the gain dispatch phase). |
| **A** | State management resolved. Minor gap in initialization or reset logic. |
| **B** | State question acknowledged but resolution deferred or hand-waved. |
| **C** | State management not addressed — spec assumes `act_num = 1` without discussing fiber state. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo C functions
      (`mju_muscleDynamics` at engine_util_misc.c:1482,
      `mju_muscleDynamicsTimescale` at 1465, `mju_muscleGain` at 1388,
      `mju_muscleBias` at 1431, `mju_muscleGainLength` at 1359), specific
      CortenForge files with line numbers (`actuation.rs:448`, `muscle.rs:336`,
      `derivatives.rs:1142`, `builder/actuator.rs:301/502`, `enums.rs`),
      specific sim-muscle types with fields (`HillMuscleConfig` — 10 fields
      inventoried in Empirical GT), specific edge cases (ctrl clamping, act
      boundaries, tau floor, tausmooth smooth blend, actearly, zero pennation,
      rigid vs compliant tendon, F0 auto-computation, moment arm conflict),
      and concrete verification data (V1–V7 with analytical derivations). Two
      reviewers could independently assign the same grade by pointing to
      specific spec lines.

- [x] **Non-overlap:** P1 grades the MuJoCo reference and conformant/extension
      boundary. P2 grades algorithm completeness (all code paths specified). P9
      grades the force-computation architectural DECISION (which GainType/
      BiasType approach). P10 grades the MJCF syntax DESIGN (XML element, parameter
      layout, validation). P11 grades STATE MANAGEMENT (where fiber state lives,
      initialization, integration interaction).
      - P1 vs P9: P1 asks "did the spec correctly describe MuJoCo's three-enum
        architecture?"; P9 asks "did the spec make and justify a design decision
        about how HillMuscle fits into that architecture?"
      - P2 vs P10: P2 asks "is the algorithm for parsing and using parameters
        fully specified in Rust?"; P10 asks "is the MJCF syntax design itself
        sound?" (which indices, which validation rules, which element). P2 could
        be A+ (algorithm complete) while P10 is B (parameter layout ambiguous).
      - P9 vs P11: P9 asks "how does force flow through gain/bias?"; P11 asks
        "where does persistent per-actuator state live?" Force computation may
        depend on fiber state, but the storage decision is independent of the
        gain/bias architecture decision.
      - P7 vs P10: P7 grades the full BLAST RADIUS of all changes (files, tests,
        match sites). P10 grades the DESIGN QUALITY of the MJCF interface
        specifically. P7 could be A+ (all files listed) while P10 is B
        (parameter layout poorly designed).

- [x] **Completeness:** The 11 criteria cover: MuJoCo reference (P1), algorithm
      (P2), conventions (P3), acceptance criteria (P4), tests (P5), dependencies
      (P6), blast radius + match-site exhaustiveness (P7), consistency (P8),
      force-computation architecture (P9), MJCF syntax design (P10), state
      management (P11). Tested: could the spec be A+ on all criteria but still
      have a gap? If the spec chose the wrong GainType approach, P9 catches it.
      If it missed a match site, P7 catches it. If activation dynamics diverged
      from MuJoCo, P1 catches it. If the sim-core ↔ sim-muscle parameter
      mapping was wrong, P3 catches it. If the `gainprm` layout was ambiguous
      or overflowed 10 slots, P10 catches it. If fiber state was silently
      lost across resets, P11 catches it. If sim-muscle's activation dynamics
      were used instead of sim-core's MuJoCo-conformant version, P3 catches
      it (convention item 5 + Empirical GT divergence warning). If the moment
      arm was double-applied, P3 catches it (convention item 8).

- [x] **Gradeability:** Each criterion maps to specific spec sections:
      P1 → MuJoCo Reference section; P2 → Specification (S1, S2, ...);
      P3 → Convention Notes table; P4 → Acceptance Criteria; P5 → Test Plan +
      Traceability Matrix + Edge Case Inventory; P6 → Prerequisites + Execution
      Order; P7 → Risk & Blast Radius (Files Affected, Behavioral Changes,
      Existing Test Impact, Match-Site Table); P8 → cross-cutting all sections;
      P9 → Architecture / Design Decision section (must exist);
      P10 → MJCF Syntax section (element design, parameter layout, validation);
      P11 → State Management section (storage decision, initialization, reset,
      integration interaction).

- [x] **Conformance primacy:** P1 is tailored with specific MuJoCo C function
      names, source files, and the three-enum architecture. P1 A+ bar requires
      explicit conformant-vs-extension split. P4 requires verification data from
      this rubric's Empirical Ground Truth. P5 requires at least one MuJoCo
      conformance test. The rubric cannot produce an A+ spec where activation
      dynamics diverge from MuJoCo's behavior, or where the conformant/extension
      boundary is unclear.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, ...) — all code paths |
| P3 | Convention Notes table (8+ porting rules) |
| P4 | Acceptance Criteria (runtime + code-review) |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Files Affected, Behavioral Changes, Existing Test Impact, Match-Site Table) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Architecture / Design Decision section (force-computation approach) |
| P10 | MJCF Syntax section (element design, parameter layout, validation rules) |
| P11 | State Management section (storage decision, initialization, reset, integration) |

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
| P9. Gain/Bias Type Architecture | | |
| P10. MJCF Syntax & Parameter Layout | | |
| P11. State Management | | |

**Overall: {grade}**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P7 | Initial draft P7 said "at least 10 sites" generically instead of enumerating each match site with file, line, and required arm behavior. Spec A/B rubrics enumerate every site explicitly. | Self-audit (comparison with Spec A/B rubrics) | Rewrote P7 to enumerate all files explicitly with per-file change descriptions. Added Empirical Ground Truth tables listing every exhaustive match site with line numbers and required arm behavior. | Rev 2 |
| R2 | P2 | Initial draft P2 was vague about the force computation bridge — "bridge between sim-core and sim-muscle" without specifying how GainType/BiasType interact with HillMuscle. | Self-audit (comparison with Spec B P2 — 9 numbered items) | Rewrote P2 to enumerate 6 specific algorithm areas. Added explicit requirement that the GainType/BiasType decision is stated in the algorithm. | Rev 2 |
| R3 | P9 | Initial P9 "Integration Architecture" was too broad — covered dependency direction, state ownership, parameter flow, alternatives. These are important but miss the CORE architectural question: how does HillMuscle fit into `force = gain * input + bias`? This is the decision that determines blast radius. | Self-audit (codebase analysis of GainType/BiasType match sites) | Renamed P9 to "Gain/Bias Type Architecture". Focused A+ bar on the 4 concrete alternatives. Added GainType/BiasType match-site table to Empirical Ground Truth. | Rev 2 |
| R4 | P10 | Initial P10 "Enum Variant Exhaustiveness" overlapped too much with P7. After sharpening P9 to focus on the architectural decision, P10 now cleanly covers match-site enumeration (drilling into specific line numbers and arm behaviors) while P7 covers overall blast radius (files, tests, behavioral changes). | Self-audit (non-overlap analysis) | Rewrote P10 to explicitly include GainType/BiasType match sites (conditional on P9 decision) and test-file construction sites. | Rev 2 |
| R5 | P1 | Initial P1 didn't require documenting MuJoCo's three-enum architecture (`mjtDyn`, `mjtGain`, `mjtBias`) and the `force = gain * input + bias` formula. This is critical context — HillMuscle's force computation must fit into this framework. | Self-audit (reading `mj_fwdActuation` dispatch structure) | Added three-enum architecture documentation requirement to P1 A+ bar. Added `engine_forward.c` dispatch line references. | Rev 2 |
| R6 | P3 | Initial P3 had only 4 convention items. Missing: `dynprm` layout for HillMuscle (reuses [tau_act, tau_deact, tausmooth]); velocity normalization (sim-core vs sim-muscle). | Self-audit (comparing with Spec B P3 — 8 items) | Expanded P3 to 6 convention items with explicit porting rules. | Rev 2 |
| R7 | P4 | Initial P4 didn't require F0 auto-computation AC (regression against Spec A's `compute_actuator_params`). HillMuscle must work with `gainprm[2] < 0` → `F0 = scale/acc0`. | Self-audit (reading `compute_actuator_params()` Phase 4) | Added F0 auto-computation AC requirement to P4. | Rev 2 |
| R8 | Empirical GT | Initial Empirical Ground Truth section was missing: (a) `actuation.rs:382` FilterExact check, (b) all GainType/BiasType match sites, (c) test files that construct ActuatorDynamics values. | Self-audit (exhaustive codebase grep) | Added FilterExact check to ActuatorDynamics table. Added complete GainType/BiasType match-site table. Added note about test files. | Rev 2 |
| R9 | Gap Log | Initial rubric had empty gap log. Spec A rubric has 20 entries, Spec B has 23. Empty gap log signals insufficient self-audit. | Self-audit | Populated gap log with 9 findings from the self-audit rounds. | Rev 2 |
| R10 | Empirical GT | Empirical Ground Truth had C source code but zero analytical verification data. Spec D's rubric includes tables of actual MuJoCo run data. No rubric should lack concrete numerical reference values — they anchor P4's AC expectations and provide falsifiable test targets. | Peer comparison (Spec D rubric) | Added 6-row analytical verification table for `mju_muscleDynamics()` with full derivations for V1–V4. Noted that values should be verified against MuJoCo Python bindings during implementation. | Rev 3 |
| R11 | P10 | Rev 2 P10 "Match-Site Exhaustiveness" overlapped with P7 despite the self-audit's claim of separation. Comparing with Spec B's P10 (Geometric Correctness) — which grades a genuinely DIFFERENT mathematical dimension — Spec C's P10 was a sub-criterion of P7 rather than an independent dimension. Meanwhile, MJCF syntax design (XML element, `gainprm` layout, validation) was a missing dimension with no coverage. | Peer comparison (Spec B P10, Spec D P10) | Replaced P10 with "MJCF Syntax & Parameter Layout". Folded match-site exhaustiveness content (test files, `#[allow(unreachable_patterns)]` prohibition) into P7. New P10 grades a genuinely independent dimension: how HillMuscle is configured in XML. | Rev 3 |
| R12 | (new P11) | No criterion for per-actuator state management. sim-muscle's `HillMuscle` owns `FiberState { length, velocity }` + `prev_mt_length` — persistent state that must survive across timesteps. sim-core's `data.act[]` provides only 1 scalar (activation). Where fiber state lives determines `act_num`, initialization logic, reset behavior, derivative pipeline interaction, and RK4 integration scope. This is analogous to Spec D's P10 (Initial State Correctness) — missing state management silently corrupts the simulation. | Peer comparison (Spec A P11 Performance, Spec D P10 Initial State) + codebase analysis (sim-muscle `FiberState`) | Added P11 "State Management" as domain-specific criterion. A+ bar requires evaluation of at least 2 alternatives with trade-off analysis. | Rev 3 |
| R13 | P7 | Rev 2 P7 didn't include test-file construction sites or `#[allow(unreachable_patterns)]` prohibition — these were in the now-replaced P10. Content must be preserved in P7 to avoid losing coverage. | Self-audit (content migration check) | Merged P10's unique content into P7: test-file table moved to Empirical Ground Truth, match-site exhaustiveness and `#[allow(unreachable_patterns)]` prohibition added to P7 A+ bar. | Rev 3 |
| R14 | P2, P4, P5 | P2 referenced "(see P10)" for match sites. After P10 replacement, this cross-reference was stale. P4 and P5 needed updates for state management ACs and fiber state test cases. | Self-audit (internal consistency after P10/P11 changes) | P2 now references "(see P7)" for match sites. P4 adds state management AC requirement. P5 adds fiber state edge case (item 10). | Rev 3 |
| R15 | P8 | P8 didn't require consistency of `act_num` across sections. With P11 added, the `act_num` value (1 vs 2 vs 3) must be consistent between P2 (algorithm), P10 (MJCF syntax), and P11 (state management). | Self-audit (cross-criterion consistency analysis) | Added `act_num` consistency check to P8 A+ bar. | Rev 3 |
| R16 | Empirical GT | Test-file construction sites were in Rev 2 P10 A+ bar text only — not in a structured table in Empirical Ground Truth. Moving them to Empirical Ground Truth makes them referenceable by both P7 and future spec sections. | Self-audit (content migration) | Added "Test files that construct `ActuatorDynamics` values" table to Empirical Ground Truth section. | Rev 3 |
| R17 | Self-audit | Rev 2 non-overlap analysis covered P1/P9, P2/P10, P7/P10, P9/P7 — all referencing old P10. After P10 and P11 replacement, these boundary analyses were stale. | Self-audit (non-overlap refresh) | Rewrote non-overlap bullet with P2/P10, P9/P11, P7/P10 boundary analyses for new criteria. | Rev 3 |
| R18 | Empirical GT | All 6 verification cases (V1–V6) use `tausmooth=0` (hard switch path). The quintic sigmoid blend path (`tausmooth > 0`) is the harder codepath and was completely unverified. `mju_muscleDynamicsTimescale()` algorithm not included despite being cited in P1. | Self-audit (code path coverage analysis) | Added `mju_muscleDynamicsTimescale()` C source with quintic Hermite formula. Added V7 (smooth blend case: ctrl=0.55, act=0.5, tausmooth=0.2) with full derivation. | Rev 4 |
| R19 | Empirical GT, P10 | `HillMuscleConfig` fields not inventoried. P10 requires grading `gainprm` index layout → HillMuscleConfig mapping, but grader had no reference for what fields exist, their types, units, or defaults. | Self-audit (gradeability check on P10) | Added `HillMuscleConfig` field inventory table (10 fields) with types, defaults, and units from hill.rs:61–97. | Rev 4 |
| R20 | Empirical GT, P3 | sim-muscle's `ActivationDynamics` uses simple `(u-a)/τ` WITHOUT activation-dependent time constants. Empirical GT said "same Millard model" — WRONG. MuJoCo uses `τ_eff = τ_base × (0.5 + 1.5×act)`. This is a critical convention mismatch: if the spec naively delegates activation to sim-muscle, HillMuscle would use wrong activation dynamics. | Codebase analysis (activation.rs:39–50 vs muscle.rs:103–124) | Corrected Empirical GT: "same Millard model" → explicit WARNING about divergence. Added "sim-muscle ActivationDynamics divergence" note. Added P3 convention awareness: spec must state that sim-core's `muscle_activation_dynamics()` is used, NOT sim-muscle's `ActivationDynamics::derivative()`. | Rev 4 |
| R21 | Empirical GT, P3 | `HillMuscleConfig::moment_arm` field conflicts with sim-core's transmission-based moment arm (`actuator_moment` from `mj_transmission`). If both are used, moment arm is double-applied — silent physics bug. No convention item flagged this. | Codebase analysis (HillMuscleConfig field inventory) | Added critical convention conflict note to Empirical GT. Added P3 convention item (8) for moment arm conflict resolution. | Rev 4 |
| R22 | P3 | Only 6 convention items vs Spec B's 8. Missing: pennation angle units (radians in sim-muscle), moment arm conflict (sim-muscle vs sim-core transmission). | Peer comparison (Spec B P3 — 8 items) | Expanded to 8 convention items: (7) pennation angle units, (8) moment arm conflict. | Rev 4 |
| R23 | P5 | Missing edge cases: (11) `tausmooth > 0` smooth blending path, (12) mixed Muscle + HillMuscle model. The smooth blend path is a distinct codepath from the hard switch. Mixed-model regression testing ensures adding HillMuscle doesn't affect existing Muscle actuators. | Self-audit (edge case completeness) | Added edge cases (11) and (12) to P5. Updated verification data reference from V1–V6 to V1–V7. | Rev 4 |
| R24 | P11 | A+ bar mentioned `Data::new()` and `Data::reset()` but not `Data::reset_to_keyframe()`. Spec D's empirical GT explicitly verified that `mj_resetDataKeyframe` restores initial history state. Missing reset_to_keyframe would cause fiber state corruption after keyframe reset. Also: P9/P11 interaction not documented — where fiber state is read depends on the P9 architectural decision. | Self-audit (Spec D comparison + cross-criterion analysis) | Added `Data::reset_to_keyframe()` explicitly. Added P9/P11 interaction note: force-computation architecture determines where fiber state is read (dynamics dispatch vs gain dispatch). | Rev 4 |
| R25 | P10 | `gainprm` is `[f64; 10]`. HillMuscle needs at least 7 parameters (max_isometric_force, optimal_fiber_length, tendon_slack_length, pennation_angle, max_contraction_velocity, tendon_stiffness, rigid_tendon) plus MuJoCo's existing 3 (range_lo, range_hi, scale) = 10. Force curve sub-parameters (FL width, FV shape) would overflow. P10 didn't require an overflow strategy. | "A+ but still broken" thought experiment (parameter count analysis) | Added overflow clause to P10 A+ bar: if HillMuscle needs >10 parameters, the spec must document the overflow strategy (use biasprm, use defaults, or propose widening). | Rev 4 |
| R26 | Self-audit | Completeness bullet didn't mention the two critical convention traps discovered in Rev 4 (sim-muscle activation divergence, moment arm double-application). These are the kinds of bugs that slip through if the rubric's completeness analysis doesn't name them. | Self-audit (final review) | Updated completeness bullet to explicitly call out P3 catching activation divergence (convention item 5) and moment arm double-application (convention item 8). | Rev 4 |
