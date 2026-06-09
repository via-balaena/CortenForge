# G2 — muscle-DRIVEN twin + validate DYNAMICS (forces) vs OpenSim

**Status:** RECON + PR1 IN PROGRESS (2026-06-08). Builds on the leg-region
kinematics arc A1–A4 (`../leg_region/recon.md`, all merged #277–#285), which
validated the twin's *kinematics* (moment arms) against real OpenSim to sub-mm.
G2 extends that software-validation strategy from **kinematics → forces/dynamics**.

**Goal:** make the leg twin **muscle-driven** and prove its **muscle forces /
joint torques** match a literature-validated reference (gait2392 in OpenSim 4.6),
in software, no hardware — the same "bridge the OpenSim model as both template and
oracle" keystone the kinematics arc used.

**Why this is the foundational next step (decided 2026-06-08, head-engineer call):**
everything the capstone needs — RL control, the differentiable co-design loop,
sim-to-real — runs on *dynamics*, not kinematics. The force model was the weakest
validated link (the FOUNDATION-TRUST AUDIT flagged that the engine's native Hill
model is numerically tested but never reconciled vs OpenSim Thelen/Millard).
Foundational order: kinematics ✅ → **dynamics fidelity (G2)** → differentiability
(the soft↔rigid keystone crate) → co-design/RL. Don't compose control/optimization
on an unvalidated forward model — sim-to-real would fail silently.

---

## The spike (DONE) — measure the gap, then decide

First slice was a throwaway `#[ignore]` spike (now retired) measuring the engine's
**native Hill** muscle force vs **real OpenSim Millard**, the #1 G2 risk, front-loaded.

**Apples-to-apples setup** (kept for the productionized cross-check):
- OpenSim `Millard2012EquilibriumMuscle` with `ignore_tendon_compliance=True`
  (RIGID tendon) — matches our engine's algebraic fiber geometry (our Hill has no
  fiber-length state / equilibrium solve).
- ISOMETRIC (qvel = 0 → force-velocity factor 1 on both sides), so the gap
  isolates the force-LENGTH + passive + pennation model differences.
- the SAME parameters (F0, L0, Lts, pennation) on both sides.
- our force evaluated at OpenSim's reported MTU length, so the already-validated
  sub-mm geometry doesn't pollute the force-model gap.

Reference: `sim/L0/tests/assets/opensim_gait2392/gen_muscle_forces.py` →
vendored `muscle_forces_opensim.json` (4 muscles × knee ROM × activations 1.0/0.5).

**Result (the gap is real and structured):**

| Regime (normalized fiber length) | Hill gap vs Millard |
|---|---|
| Near optimal (plateau, ~0.85–1.1) | 0.5–3 % ✅ (confirms F0/L0/Lts setup is right) |
| Descending limb (>1.2) | −15 % to −27 % (worst −266 N, rect_fem) |
| Short fibers (<0.75) | +37 % to +61 % (Gaussian rises too steep) |
| Below ~0.5 | −100 % (our hard cutoff 0.5 vs Millard's ~0.44 floor) |
| High pennation (semimem 0.26 rad) | extra −15 % (fixed vs variable pennation) |

Overall RMSE 77.7 N, worst 292 N / 140 % (overall worst point) on muscles with
F0 ≈ 900–1365 N; the per-regime figures above are the worst within each band (e.g.
−266 N is the worst on rect_fem's descending limb). The knee ROM sweeps these
muscles across normalized fiber length **0.44–1.46**, so the gap is in play across
the working range. Clean signal: plateau agrees, limbs diverge ⇒ pure curve-*shape*
difference, not a setup bug. *(These figures are from the now-retired throwaway
spike — illustrative magnitudes that motivated the decision, not a CI-gated result;
the committed cross-check instead gates the Millard reproduction.)*

**Decision (user, "measure first then decide"): BUILD A FAITHFUL MILLARD MODEL.**
The gap is too large and too operating-point-dependent to wave away for a twin
whose whole validation strategy is borrowing OpenSim's literature validation (a
−27 % rect_fem error mid-flexion would silently mislead RL/co-design). gait2392
ships `Millard2012EquilibriumMuscle` ⇒ match *Millard's* curves (not Thelen) for
bit-faithfulness to the vendored model.

---

## Implementation decision — port the analytic curves, don't sample

OpenSim's Millard curves are `SmoothSegmentedFunction`s: C2 quintic-Bézier splines
built by `SmoothSegmentedFunctionFactory` from documented default parameters. Two
options to reproduce them:

- **(A) port the analytic Bézier** (`SmoothSegmentedFunction` +
  `SegmentedQuinticBezierToolkit` + the 3 factory constructors). Exact derivatives,
  no embedded OpenSim data tables, the engine becomes genuinely Millard-capable.
- (B) sample the curves and spline a vendored table. Less code, but an
  approximation with embedded data, and a data dependency in the engine core.

**Chosen: (A).** It's the principled "build the model" answer, keeps the engine
core data-free, and gives exact analytic derivatives for the differentiable
co-design loop downstream. The densely-sampled curves become the *validation*
ground truth, not the implementation. (Fallback to (B), same validation target,
only if the Bézier port had proved disproportionate — it did not.)

### What "faithful Millard" needs (rigid tendon, the matched regime)
1. **Active force-length curve** — `createFiberActiveForceLengthCurve(0.4441, 0.73,
   1.0, 1.8123, ylow=0, dydx=0.8616, curviness=1.0)`, 5 elbow segments.
2. **Passive (fiber) force-length curve** — `createFiberForceLengthCurve(eZero=0,
   eIso=0.7, kLow=0.2, kIso=2/0.7, curviness=0.75)`, 2 segments.
3. **Force-velocity curve** — `createFiberForceVelocityCurve(fmaxE=1.4, dydxC=0,
   dydxNearC=0.25, dydxIso=5.0, dydxE=0, dydxNearE=0.15, concCurv=0.6, eccCurv=0.9)`,
   4 segments. (Used at FV=1 isometrically in PR1; exercised in PR2.)
4. **Variable-width (constant-height) pennation**: `h = L0·sin(penn0)`;
   `fiber = sqrt((mtu−Lts)² + h²)`; `cos(penn) = (mtu−Lts)/fiber`.
5. **Force along path** = `F0·(a·AFL·FV + PFL)·cos(penn)` (+ `β·v̇_norm` fiber
   damping, 0 at isometric — PR2).

All four target muscles share OpenSim's *default* curve parameters and
`fiber_damping = 0.1` (asserted in the generator), so one shared curve set suffices.

### Min-fiber clamp (noted, immaterial to force here)
OpenSim clamps normalized fiber length to ~0.4441; for our muscle set the clamp
region is where AFL = 0 and PFL = 0 on both sides, so force = 0 regardless. The
port relies on the curve's flat extrapolation below domain (AFL → 0) instead of an
explicit clamp; the cross-check confirms this is exact for this scope.

---

## Validation (two tiers, both in CI, no OpenSim install)

`tools/cf-osim/tests/millard_force_cross_check.rs` grades against vendored JSON:

1. **CURVE unit check** — our `MillardCurves` (active-FL / passive-FL /
   force-velocity) vs `millard_curves_opensim.json` (200 dense samples per curve,
   `gen_millard_curves.py`). Gate 1e-5; **achieved ~1e-12** (Bézier + Newton
   reproduces `SmoothSegmentedFunction` to machine precision).
2. **FORCE integration check** — our `millard_isometric_path_force` vs
   `muscle_forces_opensim.json` (4 muscles × ROM × a∈{1.0,0.5}). Gate 1e-6 relative
   (generous headroom over cross-platform sin/sqrt last-ulp); **achieved worst
   |gap| 1.57e-9 N, RMSE 2e-10 N** — machine-exact, because our model *is*
   OpenSim's Millard evaluated at the same MTU length. The native Hill's 15–60 %
   gap is closed.

Plus a self-contained in-crate anchor test (`millard.rs::spotcheck`) so sim-core
has muscle-curve coverage without depending on cf-osim.

**Honesty:** PR1 validates the force MODEL at matched geometry (OpenSim's MTU
length). End-to-end (engine computes its own MTU) adds the sub-mm kinematic
residual → a tiny force residual, graded once the model is wired into the engine
(PR3). Force-VELOCITY is unvalidated until PR2 (the spike/PR1 are isometric).

---

## Slicing (each its own PR; n+1 cold-read cleanup; pre-PR local ultra-review)

- **G2-PR1 — faithful Millard curves + isometric force.** *(IN PROGRESS / largely
  done.)* `sim/L0/core/src/forward/millard.rs` (`SmoothSegmentedFunction`,
  `MillardCurves`, `millard_isometric_path_force`) + the two-tier cross-check + the
  two reference generators. Curves ~1e-12, isometric force worst ~5.5e-11 N vs real
  OpenSim. No engine pipeline changes yet (additive pub functions).
- **G2-PR2 — force-velocity (non-isometric).** *(DONE.)* `millard_path_force(.., mtu,
  mtu_vel, act)` adds the FV factor (fiber velocity = `cos(penn)·v_mt`, normalized by
  `L0·vmax`) + the `β·v̄` fiber-damping term (β=0.1); `millard_isometric_path_force`
  is now a `mtu_vel=0` wrapper. New reference `gen_muscle_force_velocity.py` →
  `muscle_force_velocity_opensim.json` (knee angle × speed × activation, shortening
  through lengthening). `millard_fv_cross_check.rs` grades (A) force and (B) an
  independent normalized-fiber-velocity check vs OpenSim's recorded value. **Result:
  machine-exact — worst |gap| 1.6e-11 N, fiber velocity 2.2e-16, across all 4 muscles.**
  **Two model behaviors the velocity sweep forced out (both validated):** (1) the
  tendon force is **floored at 0** — a muscle pulls, never pushes, so the damping term
  can't drive it negative; (2) at the **min-fiber clamp** (`norm_len < 0.4441`) OpenSim
  freezes fiber *velocity* to 0, not just length, so damping vanishes there too. PR1
  isometric check unchanged (both clamps are inert at v=0 / above the floor).
- **G2-PR3a — wire Millard into the engine as a driven actuator.** *(DONE.)*
  `ActuatorDynamics`/`GainType`/`BiasType::MillardMuscle` (`enums.rs`); the actuation
  dispatch (`actuation.rs`) computes the active gain `−F0·AFL·FV·cos` and the
  passive+damping bias `−F0·(PFL+β·v̄)·cos` via the new `millard_active_gain` /
  `millard_passive_bias` (sharing `fiber_kinematics` with `millard_path_force`, so no
  drift — both cross-checks stay machine-exact); `default_millard_curves()` (a
  process-wide `OnceLock`) avoids rebuilding the Bézier per step; `fiber.rs` F0
  auto-resolution includes Millard. The engine convention is force ≤ 0 (tension); the
  muscle's non-negative tendon floor (which the `β·v̄` damping can otherwise breach at
  fast shortening) is applied **in the force model** for MillardMuscle — `force.min(0)`,
  independent of any configured `actuator_forcerange`. **Validation
  (`actuation.rs::millard_engine_tests`): a programmatic Millard joint-actuator's
  `forward()` reproduces the standalone `millard_path_force` to <1e-9 across all
  regimes (isometric / eccentric / shortening-floored / min-fiber clamp)** — ties the
  engine pipeline to PR1/PR2's OpenSim-validated force. (`sim-mjcf` gets only the
  `act_num` arm; the `"millardmuscle"` MJCF parse is PR3b.)
- **G2-PR3b — emit + drive the leg twin + the dynamics gate.** *(DONE.)* The IR muscle
  carries optional `MuscleForce`; `cf_osim::parse_muscle` reads the 5 Millard params;
  `cf-mjcf-emit` emits a `<general dyntype="millardmuscle">` actuator per muscle on its
  spatial tendon; `sim-mjcf` parses `"millardmuscle"` (`compute_general_millardmuscle`
  + validation, unit-tested mirroring HillMuscle). **Validation** (`muscle_driven_dynamics.rs`):
  the **machine-exact gate** = the loaded twin's actuator force == standalone
  `millard_path_force` at its own actuator length (worst 2.3e-13 N) → proves the
  emit→parse→tendon-transmission→dispatch wiring; the **reported** end-to-end knee joint
  moment vs OpenSim (`gen_muscle_joint_moments.py`) is ~1–2% for the quads/bifemlh but
  ~25% for semimem — the dominant residual is the emit's **dropped-conditional via-point**
  (a conditional path point active over part of the ROM is dropped, bending the moment
  arm: the quads at deep flexion, semimem at extension where its conditional is active
  ≈0…−32°), a documented emit-geometry limit (A1 5 mm gate), not the force model. The
  **muscle-DRIVEN visual example** (`cf-msk-fit/examples/muscle_drive.rs`) sweeps the knee
  ROM with each muscle tinted/thickened by its live Millard force — the validated
  force-length dynamics made visible (the audit's deferred Hill visual example). Free
  forward dynamics (move under muscle force) is deferred — the coupled knee is driven
  kinematically, not via equality constraints (`R-coupled-dyn`).

## Risks / open items
- **R-implicit-deriv** — the analytic actuator Jacobian for Millard (the Bézier
  FV-curve derivative + the `β·v̄` damping derivative) is not yet implemented. So:
  (1) `mjd_transition` routes any model containing a Millard actuator to the **full
  finite-difference** path → the transition matrices (A,B) stay EXACT for derivative
  consumers; (2) the analytic arms in `derivatives/hybrid.rs` `continue` for Millard
  (like `User`), so the *forward* implicit integrator treats Millard's velocity
  dependence **explicitly** (valid, stable for β=0.1, not wrong dynamics). Forward
  dynamics with explicit integrators is fully exact. Implement the analytic curve
  derivative (a `SmoothSegmentedFunction::slope`) when an implicit-integrator
  differentiable use needs the in-loop analytic term.
- **R-pennation-edge** — semimem's tendon-slack ≈ MTU at deep flexion (along-tendon
  ~3.6 mm) drives pennation toward 90°; force is ~0 there (both sides), but if PR3's
  engine integration evaluates near the singularity, revisit the `cos_penn` floor.
- **R-minfiber** — OpenSim's `minimumFiberLength / L0 = max(0.4441, sin(penn0))` (the
  AFL-curve minimum vs the pennation-model minimum, verified by probing OpenSim 4.6).
  For all four gait2392 muscles `penn0 ≤ 0.26` (`sin ≤ 0.26 < 0.4441`), so the AFL
  floor 0.4441 dominates and our `MIN_NORM_FIBER_LENGTH = 0.4441` clamp is **exact and
  general for this family** — not a semimem coincidence. **Edge (high pennation,
  `penn0 > ~26°` ⇒ `sin(penn0) > 0.4441`):** OpenSim pins the fiber at `sin(penn0)`,
  but our constant-height `raw_norm_len = √(along² + h²)/L0` (`h = L0·sin(penn0)`)
  can never fall below `sin(penn0) > 0.4441`, so the 0.4441 clamp never fires and the
  fiber *re-grows* into the active domain as `|along|` increases (force hundreds of N
  where OpenSim gives 0). No gait2392 muscle reaches this; if PR3 wires one, clamp the
  fiber length to `max(0.4441, sin(penn0))·L0` and recompute `cos(penn)` from the
  clamped geometry. (Overlaps R-pennation-edge.)
- **R-FV** — *Retired by PR2:* force-velocity + fiber damping validated machine-exact
  (worst 1.6e-11 N) vs real OpenSim over a knee angle × speed × activation grid.
- **F0 provenance** — rect_fem_r's max isometric force *is* 1169 N in the gait2392
  XML and via `getMaxIsometricForce` (no divergence). The hazard is purely that a
  naive positional/line-order grep over `gait2392.osim` can pick up a *different*
  muscle's `<max_isometric_force>` (e.g. `glut_med1_r`'s 819 N, which sits before
  `rect_fem_r` in file order). PR1 is self-consistent (reads params from the JSON);
  PR3's emit must source F0 from the *parsed muscle object*, not a positional grep.
- **Elastic tendon** — out of scope (our engine + the comparison are rigid-tendon,
  matching `ignore_tendon_compliance`); revisit only if a muscle's compliant tendon
  matters for the capstone.

## OpenSim refs / regeneration
OpenSim 4.6 at `/tmp/osim-venv` (wiped on reboot; recreate:
`uv venv --python 3.12 /tmp/osim-venv && VIRTUAL_ENV=/tmp/osim-venv uv pip install opensim`).
Generators in `sim/L0/tests/assets/opensim_gait2392/`: `gen_muscle_forces.py`
(force reference), `gen_millard_curves.py` (curve reference). Millard source ported
from `opensim-org/opensim-core` `OpenSim/Common/{SmoothSegmentedFunction,
SegmentedQuinticBezierToolkit,SmoothSegmentedFunctionFactory}` +
`OpenSim/Actuators/{ActiveForceLength,FiberForceLength,ForceVelocity}Curve`.
