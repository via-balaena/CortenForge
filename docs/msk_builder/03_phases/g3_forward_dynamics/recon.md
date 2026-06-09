# G3 — FORWARD-DYNAMICS FIDELITY (close + validate activations → MOTION)

**Status:** RECON + SPIKE DONE (2026-06-08); architecture resolved (Option C).
**PR1 = PR #290 (CI green, open).** **PR2 SPIKE (2026-06-09) FOUND THE TWIN's
FORWARD DYNAMICS IS BROKEN — see the `★★ PR2 SPIKE FINDINGS` block below. PR2 is
NOT "add a gate"; it is "find + fix the forward-dynamics bugs the spike surfaced."**
The foundation-first arc after G2 (dynamics /
forces, all 4 PRs merged: #286/#287/#288/#289 → main `58fb2551`). G2 validated
muscle **forces** and instantaneous **joint torques** vs OpenSim machine-exact —
but only **statically** (poses set by hand). G3 closes and validates the
**forward-dynamics loop**: activations → forces → acceleration → **motion over
time**, the first time the twin moves under its own muscle force.

> **★ PR1 SHIPPED (2026-06-09) — coupled knee as a degree-N equality constraint.**
> Engine: `EqualityType::Joint` now evaluates a degree-≤10 polynomial (Horner over
> all 11 `eq_data` slots; backward-compatible — legacy linear/constant couplings are
> bit-identical), builder `take(11)`. Emit (`cf-mjcf-emit`): fits a **degree-8**
> polynomial to each coupled SimmSpline slide and emits `<equality><joint>` for all 8
> couplings (tibia tx/ty + patella x/y/z) with `solref="0.004 1"`/stiff `solimp`, plus
> ROM range limits on every coordinate joint and `<option timestep="0.001"/>`. The
> kinematic `qpos_targets` path is unchanged. **Validated** (`coupled_knee_equality.rs`
> + engine `test_joint_equality_degree8_*`): the constraint reproduces the kinematic
> SimmSpline pose to **0.15 mm** across ROM, and under forward dynamics the coupled
> slides hold the manifold to **~0.35 mm near extension / ~1.1 mm in deep flexion**
> (vs ~8.5 mm for a too-soft constraint). The coupled knee now MOVES under torque
> while staying coupled — the G3 keystone. So the "degree-4 today / 6 spare slots /
> retire the spike" notes below are the pre-PR1 analysis; the spike is deleted and
> its findings recorded here + in the PR1 banner. **Next = PR2** (OpenSim-`Manager`
> forward-dynamics accel/trajectory gate).

**OpenSim venv:** `/tmp/osim-venv` (gets wiped on reboot — recreate with
`uv venv --python 3.12 /tmp/osim-venv && VIRTUAL_ENV=/tmp/osim-venv uv pip install
opensim`; ~30 s, OpenSim 4.6). Confirmed present + `osim.Manager` available this
session (`opensim 4.6-2026-04-25`).

---

## Why this, why now (the foundation-first call)

G2 proved the **force model** (PR1/PR2: Millard curves + force-velocity,
machine-exact vs OpenSim) and the **instantaneous joint torque** (PR3b: emit →
MJCF → tendon transmission → Millard dispatch, force wiring exact to 2.3e-13 N).
But the validation was all at **hand-set poses** — the twin literally cannot MOVE
under its own muscle force yet. The coupled knee's dependent DOFs (the
tibiofemoral roll-glide "slides") are driven **kinematically**: the caller
evaluates the coupling spline and writes `qpos` every frame
(`muscle_drive.rs:162–177`, `Emitted::qpos_targets`). There is no equality
constraint holding the slides on the spline manifold while the knee accelerates,
so `data.forward()` reads a force at a *prescribed* pose; it never integrates the
loop forward.

Everything the capstone needs sits ON TOP of the forward dynamics:
- **Differentiability** (the soft↔rigid keystone) = differentiating *through* the
  forward sim. You cannot differentiate an un-closed loop.
- **RL / control** trains *on* the forward sim.
- The FOUNDATION-TRUST principle: *don't compose control/optimization on a forward
  model validated only at static forces/torques — sim-to-real fails silently.*

So the order is: forces ✅ → joint torques ✅ → **forward dynamics (G3)** →
differentiability → co-design/RL. Doing differentiability first would build the
moat on an unfinished foundation. (Decided 2026-06-08, head-engineer call,
user-confirmed "best foundation-first move".)

**This also turns the `muscle_drive` demo from a painted kinematic sweep into a
LIVING muscle-driven sim** — the twin moves under force, the payoff that makes G3
visible.

---

## ★★ PR2 SPIKE FINDINGS (2026-06-09) — the twin's forward dynamics is BROKEN

Branch `feat/msk-g3-forward-dynamics-gate` (off PR1). Built an OpenSim `Manager`
forward-dynamics oracle and a throwaway twin-side spike
(`tools/cf-mjcf-emit/tests/spike_forward_dynamics.rs`, `#[ignore]`). Setup: reduced
RIGHT LEG (5 DOF: hip 3 + knee + ankle; pelvis welded, contralateral/torso/
subtalar/mtp locked — matches the twin), rigid tendon, **gravity OFF** (isolate
muscle→accel), activations set directly, velocities zeroed. gait2392 segment
inertias injected into the twin (femur/tibia exact; foot = talus+calcn+toes
composite computed exactly about the talus frame). Confound gotchas nailed: OpenSim
`state.setU(0)` is required (per-coord `setSpeedValue` + `setValue(...,True)` leaves
residual velocity → garbage udot); lock-via-`set_locked` keeps mobilities + adds
constraints (don't read accel through it without zero velocity).

**Result: the twin's forward-dynamics qacc does NOT match OpenSim.** Three findings:

1. **INERTIA IS CORRECT.** Effective knee inertia (`qvelᵀMqvel` at unit knee speed,
   constraint-consistent) = **0.444** kg·m² vs OpenSim **0.442**. So the segment
   masses + coupled-knee inertia are right; the error is NOT inertia.

2. **JOINT ROM LIMITS FIRE MID-RANGE → phantom forces. ✅ FIXED (PR2a, 2026-06-09).**
   With ZERO force, zero velocity, gravity off, every joint mid-range, the no-force
   baseline knee udot was **638 rad/s²** (must be 0); only `qfrc_constraint` (≈86) was
   nonzero. **ROOT CAUSE: the emit's MJCF lacked `<compiler angle="radian"/>`, so
   MJCF's default `angle="degree"` converted the emitted joint `range` (which is in
   RADIANS) as degrees — shrinking e.g. the knee's `-2.0943951 0.17453293` ~57× into a
   bogus range (-0.0366, 0.00305) the joint is ALWAYS outside → the limit fires
   constantly.** Proven by a minimal single-hinge repro (range "-2.0 0.2" loaded as
   (-0.0349, 0.00349)). **FIX: emit `<compiler angle="radian"/>`** (`cf-mjcf-emit`);
   no-force baseline → exactly 0 with limits ON, and the deep-flexion error is no
   longer limit-driven. Regression test `emitted_joint_limits_are_radian_no_midrange_phantom`
   (asserts radian range + zero mid-range no-force qacc). The R-extrapolation limits
   PR1 added now actually work. (Latent in PR1 #290 — its loaded ranges were also
   degree-shrunk; PR2a corrects it.)

3. **RESIDUAL COUPLED-FORCE ERROR (limits off).** Even with limits disabled, the
   per-muscle knee udot is wrong: quads **~1.5–1.6× too high** (rect_fem twin +624
   vs OS +386), hamstrings **WRONG SIGN** (bifemlh twin +43 vs OS −99 flexion). The
   ankle whips (−3014 vs −360, downstream of the knee error). The muscle FORCES are
   machine-exact (G2/PR1) and inertia is right, so this is a **multi-DOF coupled-knee
   force-transmission error under dynamics** — the equality-constraint + separate-
   slide-DOF representation (PR1) reproduces KINEMATICS (0.35mm) but not the coupled
   generalized force in forward dynamics. (Also suspect: the free-ankle/foot
   dynamics, given the ankle whip; needs isolation.)

**OpenSim oracle scripts (throwaway, in `/tmp`): `g3_fd_oracle.py` (5-DOF reduced
fwd-dyn + effective-inertia KE), `g3_foot_inertia.py` (exact composite foot
inertia), `g3_permuscle2.py` (per-muscle).** They'll be productionized into a
`gen_forward_dynamics.py` once the bugs are understood.

**IMPLICATION / RESHAPED PLAN:** PR2 ≠ "add a gate." The spike (front-loading the
#1 risk, exactly as intended) found the twin does NOT move correctly under muscle
force. Closing G3 requires: **(PR2a) ✅ DONE 2026-06-09** — fixed the joint-limit
phantom (the `<compiler angle="radian"/>` bug above) + regression test;
**(PR2b)** find + fix the residual coupled-knee force-transmission error (finding 3:
quads ~1.5×, hams wrong-sign, ankle whips — may need a true function-coupled
reduced-coordinate joint in the engine instead of slide-DOFs+equality, OR isolate it
to the foot/ankle); **(PR2c)** THEN the OpenSim-Manager gate. The throwaway spike +
`/tmp/g3_*.py` oracle scripts stay for PR2b.

## G3 = two deliverables

1. **Make the coupled knee a real engine EQUALITY CONSTRAINT** (joint coupling),
   replacing the kinematic tie, so the knee moves under torque while the dependent
   slides stay coupled to the angle.
2. **VALIDATE the twin's forward dynamics vs OpenSim's OWN forward integration**
   (`Manager`): drive a known activation profile from a known state, integrate
   forward, check the resulting acceleration (and a short trajectory) match
   OpenSim — the end-to-end "activations → motion" check. Reuses the gait2392
   bridge + `/tmp/osim-venv`, no hardware.

---

## What already exists (cold-read findings, 2026-06-08)

### The engine equality constraint — production-ready, polynomial only

`EqualityType::Joint` couples one joint's qpos to another's via a polynomial:
**`q1 = c0 + c1·q2 + c2·q2² + c3·q2³ + c4·q2⁴`** (degree 4).

- enum: `sim/L0/core/src/types/enums.rs:332` (`EqualityType::{Joint,Connect,Weld,Tendon,Distance}`).
- storage: `sim/L0/core/src/types/model.rs:801` — `eq_type/eq_obj1id/eq_obj2id`,
  `eq_data: Vec<[f64;11]>` (Joint uses `[0..4]` for the 5 polycoef; **`[5..10]`
  are unused for Joint — 6 spare slots**), `eq_active/eq_solimp/eq_solref`.
- Jacobian: `sim/L0/core/src/constraint/equality.rs:211`
  `extract_joint_equality_jacobian` — constraint `q1 − poly(q2) = 0`, Jacobian
  `+1` at dof1, `−poly'(q2)` at dof2 with `poly'(q2)=c1+2c2·q2+3c3·q2²+4c4·q2³`;
  velocity err `qd1 − poly'(q2)·qd2`. `joint2_id==usize::MAX` → single-joint lock.
- solver: assembled (`constraint/equality_assembly.rs:23`) into `efc_J`, solved by
  Newton/CG/PGS in `mj_fwd_constraint` (`constraint/mod.rs:302`), mapped back as
  `qfrc_constraint = Jᵀλ`. Standard pipeline, nothing special needed.
- MJCF: `<equality><joint joint1=".." joint2=".." polycoef="c0 c1 c2 c3 c4"
  solref=".." solimp=".."/></equality>` — parser `mjcf/src/parser.rs:2411`, types
  `mjcf/src/types.rs:1869` (`MjcfJointEquality`, ctors `lock`/`lock_at`/`mimic`/
  `gear`), builder `mjcf/src/builder/equality.rs:222`.
- proven: `examples/fundamentals/sim-cpu/equality-constraints/{joint-mimic,joint-gear}`
  + `sim/L0/tests/integration/equality_constraints.rs` (mimic/gear/lock all hold
  under actuation to <0.1–0.15 rad).

### The coupled knee today — kinematic wrapper slides

gait2392 `knee_r` (`gait2392.osim:573`): one free hinge `knee_angle_r`
(rotation about z, `LinearFunction`) + **two coupled SimmSpline translations** on
the tibia, plus the patella moving-path-points.

| coupling | axis | knots | shape | excursion (×scale 1.14724) |
|---|---|---|---|---|
| translation1 (tx) | (1,0,0) | **12** | **wiggly — non-monotonic, multiple inflections over full ROM; single hump over working ROM** | ~10 mm |
| translation2 (ty) | (0,1,0) | 7 | smoother, near-monotonic | ~27 mm |

tx knots (rad → m, pre-scale): `x=-2.0944…2.0944`,
`y=-0.0032, 0.00179, 0.00411, 0.0041, 0.00212, -0.001, -0.0031, -0.005227,
-0.005435, -0.005574, -0.005435, -0.00525` — rises to a +0.0041 peak near
−1.4 rad, dips to −0.0056 near +0.49 rad: at least two turning points over the
full physical range, **one hump over the working sweep** (0 → −1.745 rad / 0→100°
flexion).

- IR: coupling is a `TransformFn::Spline(Spline)` per `TransformAxis`
  (`cf-msk-lib/src/ir.rs:38`); `Spline` is a **natural-cubic** re-derivation of
  OpenSim's SimmSpline (`cf-msk-lib/src/spline.rs`) — *its derivative feeds moment
  arms, which is why linear interp was rejected*.
- emit: each coupled slide → an **interposed wrapper body** with a free `slide`
  joint (`cf-mjcf-emit/src/lib.rs:262` `emit_body`, classify at `:232`); reported
  in `Emitted::driven` as `DrivenJoint{joint,coordinate,function}`. The caller
  evaluates `Emitted::qpos_targets(coords)` (`:76`) and **writes qpos each frame**
  — kinematic. A coupled *rotation* spline currently **panics** ("would need an
  equality constraint", `:254`); gait2392's knee couplings are all translations,
  so this arc doesn't need coupled-rotation support.
- patella moving points: `emit_patella` (`:375`) emits 3 coupled slide sub-bodies,
  each driven by its own SimmSpline — **also kinematic today**.
- deferred markers already in the tree: `muscle_drive.rs:16` ("Free forward
  dynamics … deferred … driven kinematically, not via equality constraints");
  `cf-mjcf-emit/src/lib.rs:11` ("the only way to reproduce the coupled knee
  without an equality constraint"); PR3b note `R-coupled-dyn`.

### The current dynamics gate (G2 PR3b) — static

`tools/cf-mjcf-emit/tests/muscle_driven_dynamics.rs`: **Gate A** = loaded twin's
`actuator_force` == standalone `millard_path_force` at its own `actuator_length`
(worst 2.3e-13 N — wiring proof, pose set by `qpos_targets`). **Reported** =
per-muscle knee joint moment (`−force × coupled_moment_arm`) vs vendored
`muscle_joint_moments_opensim.json` (~1–2% functional ROM; semimem ~25% at
extension = dropped-conditional via-point, an *emit-geometry* limit not a dynamics
bug). Both at **hand-set poses** — no integration.

### Reference generators (pattern to mirror for the G3 oracle)

`sim/L0/tests/assets/opensim_gait2392/gen_*.py` →
vendored `*_opensim.json`, cross-checked in CI with NO OpenSim. Existing:
`gen_moment_arms / gen_leg_moment_arms / gen_scaled_moment_arms / gen_millard_curves
/ gen_muscle_forces / gen_muscle_force_velocity / gen_muscle_joint_moments`. G3
adds `gen_forward_dynamics.py` (OpenSim `Manager` forward integration → qacc /
short trajectory under a known activation).

---

## THE central architectural decision — spline coupling vs polynomial constraint

The engine constraint is a **degree-4 polynomial**; gait2392 coupling is a
**natural-cubic SimmSpline**. A1–A4 earned a **sub-mm** kinematic validation
(scale morph 0.31 mm; moment arms sub-mm) — and a moment arm is `dL/dθ`, i.e. it
rides the coupling's **derivative**. So the constraint must reproduce the spline
faithfully in **both position and slope**, or G3 silently re-introduces a geometry
error and forfeits the kinematic validation we already banked. Three paths:

- **A. Quartic-fit polycoef (zero engine change).** Least-squares-fit `c0..c4` to
  each coupling spline over the working ROM; emit `<equality><joint polycoef>`.
  Cheapest, uses the already-validated `EqualityType::Joint`. **Risk:** tx is
  wiggly — a quartic may miss it, and slope error is worse than position error.
- **B. Spline equality constraint (faithful, engine change).** A new constraint
  flavor that evaluates a natural-cubic spline + its analytic derivative for the
  Jacobian. Bit-faithful, preserves the sub-mm validation, and matches the
  project's "build the faithful model" ethos (the same call we made choosing
  analytic Bezier over sample-and-spline for Millard in PR1). **Cost:** new
  constraint type in the solver; the spline math lives in cf-msk-lib, so sim-core
  needs either a ported evaluator or a generic poly-of-higher-degree carrier.
- **C. Higher-degree polynomial (small engine change).** `eq_data` already has 6
  spare slots → extend Joint to degree-10. A dense least-squares degree-10 fit
  matches a 12-knot spline well; minimal solver change (read more coeffs in the
  Jacobian + builder). **Risk:** Runge oscillation between knots (mitigated by
  dense-sample LSQ + the working-ROM restriction).

**Decision rule (the spike resolved it — see SPIKE RESULTS below):** measure the
fit's worst position AND slope error over the working ROM, at degrees 4/6/8.
- ≤ ~0.5 mm position **and** moment-arm impact ≲ the A1 5 mm emit budget at deg4 →
  ship **A** (minimal, validated path).
- Otherwise prefer **C** (degree-N poly in the spare `eq_data` slots) over **B** —
  C reuses the existing Joint constraint type + Jacobian shape with the smallest
  solver surface, and a dense LSQ poly over the bounded working ROM avoids Runge
  in practice. Fall to **B** only if no bounded-degree polynomial holds slope.

**→ RESOLVED: Option C — degree-8 polynomial equality constraints (extend the
engine's `Joint` constraint from degree-4 to degree-≤10 via the spare `eq_data`
slots).** The spike showed quartic fails the patella couplings (1.38 mm, see
below) but **degree-8 holds every coupling ≤ 0.17 mm in position** — and 9 coeffs
(c0..c8) fit inside the existing `eq_data: [f64;11]` (degree-10 ceiling). So C is
both *sufficient* (sub-0.2 mm everywhere, preserving the A1–A4 sub-mm validation)
and *minimal* (no new constraint type, reuse the Joint Jacobian shape; the engine
change is a Horner loop over all coeffs + `builder.take(11)` + variable-length
polycoef — Option B's spline constraint is unnecessary). The engine extension is
itself PR1's first step. (Head-engineer call, 2026-06-08, recorded post-spike.)

Two further coupling facts the spike/PRs must honor:
- **Every kinematically-driven slide must become a constraint or a weld.** A free
  slide joint with no constraint and no actuation will drift/fall under forward
  dynamics. The tibia tx/ty couplings carry the load path → equality constraints.
  The patella moving-point slides (3 each) also currently free-kinematic → either
  constrain them too, or (spike scope) weld/freeze them so the knee DOF is
  isolated. Full twin self-consistency = constrain all; the spike can isolate.
- **Sign / obj ordering.** Engine constraint is `q1 = poly(q2)`; set `q1 = slide`
  (dependent), `q2 = knee_angle_r` (independent). polycoef fit must be in the
  engine's knee-angle SIGN convention (knee flexion negative here) — get this
  wrong and the slide couples backwards.

---

## #1 RISK + THE SPIKE (front-loaded, throwaway, before any PR)

**The #1 risk has two faces:** (a) **fidelity** — can a bounded-degree polynomial
equality constraint reproduce the SimmSpline coupling (position AND slope) over
the working ROM to within budget; (b) **dynamics stability** — does the engine's
constraint solver actually hold the slides on the manifold while the knee
accelerates under muscle torque, without blow-up, and reproduce OpenSim's forward
acceleration. Front-load BOTH with one `#[ignore]` throwaway spike before slicing
PRs (the arc pattern — same as the G2 Hill-vs-Millard spike).

**Spike plan** (`tools/cf-mjcf-emit/tests/spike_forward_dynamics.rs`, `#[ignore]`,
retired after; throwaway gen script alongside):

1. **Fidelity.** Fit `c0..c4` (and `c0..c10`) by dense LSQ to the tx + ty coupling
   splines over the working ROM (0 → −1.745 rad + a margin). Report worst
   `|Δposition|` (mm) and worst `|Δslope|` (d(coupling)/dθ, mm/rad) vs the
   `Spline` evaluator, per degree. → resolves A vs C vs B.
2. **Emit constraints.** Build the knee twin; add `<equality><joint
   joint1=slide joint2=knee_angle_r polycoef=…>` for tx + ty (hand-construct the
   MJCF or a temporary emit tweak). Patella slides: weld/freeze for the spike to
   isolate the knee DOF.
3. **Stability + manifold-hold.** Set a known initial state (e.g. knee −30°,
   qvel 0), apply known muscle activations, `forward()`, then step forward N
   times. Assert: (i) after each `forward()` the slides satisfy
   `q_slide ≈ poly(q_knee)` within constraint tol (solver holds the manifold);
   (ii) the knee moves and the trajectory stays bounded (no blow-up); (iii)
   energy/behaviour sane.
4. **OpenSim accel match.** Same model + initial state + activations in OpenSim
   (`ignore_tendon_compliance=True`, activations set directly to bypass
   excitation→activation first-order dynamics — compare PURE forward mechanics),
   integrate one step with `Manager`, read knee `qacc` at t=0. Compare engine
   `qacc[knee]` vs OpenSim. The **t=0 single-step accel** is the cleanest gate
   (no integrator-divergence confound); a short trajectory is the reported
   living-sim signal.

**Confounds the spike must control (so a mismatch means the dynamics, not setup):**
segment **mass/inertia** must match between twin and OpenSim (emit gets inertia
from IR anthropometry; reconcile or read OpenSim's into the twin); **gravity**
on/off identical; **rigid tendon** both sides; **activation set directly** (no
activation dynamics) for the first gate; isolate to the knee DOF (hip/ankle
locked) so the comparison is 1-DOF clean.

**Spike output = a go/no-go + the A/C/B decision + the confound checklist that
the PR-level gate inherits.** If fidelity fails for all bounded degrees → B is
forced and PR1 grows. If the solver can't hold the manifold under dynamics → that
is the real finding and reshapes the whole arc (escalate before slicing).

### SPIKE RESULTS (2026-06-08, `tools/cf-mjcf-emit/tests/spike_forward_dynamics.rs`, `#[ignore]`, throwaway)

**GO. Both faces of the #1 risk retired.** Two stages ran:

**Stage 1 (R-fidelity) — poly vs SimmSpline, position + slope over working ROM
(−120…10°):**

| coupling (excursion) | deg4 |Δpos| | deg6 | deg8 |Δpos| |
|---|---|---|---|
| `tibia_r_tx` (11 mm, wiggly) | 0.35 mm | 0.21 | **0.14 mm** |
| `tibia_r_ty` (31 mm, smooth) | 0.23 mm | 0.07 | **0.009 mm** |
| `rect_fem_r_1_x` (57 mm) | 0.67 | 0.46 | **0.17 mm** |
| `rect_fem_r_1_y` (9 mm) | **1.38 mm** | 0.22 | **0.055 mm** |
| `vas_int_r_2_x` (59 mm) | 0.79 | 0.46 | **0.16 mm** |
| `vas_int_r_2_y` (6 mm) | **1.36 mm** | 0.31 | **0.055 mm** |

→ **quartic fails the patella couplings (1.36–1.38 mm); degree-8 holds all ≤ 0.17 mm.**
Confirms **Option C** (extend Joint to degree-≤10, fit deg8). Slope error is
non-monotonic in degree (a deg-6 position fit can wiggle worse in slope than deg4);
deg8 is best (smooth ty deg8 slope 0.32 mm/rad). The steep patella-x slopes
(~4–6 mm/rad at deg8) ride the same path that already carries the emit's
dropped-conditional moment-arm residual (the G2 documented ~1–2% / semimem ~25%),
so they're inside the existing geometry budget; the controlled, dominant metric is
the ≤0.17 mm position fidelity.

**Stage 2 (R-solver-stability) — equality constraint under dynamics:** a pendulum
knee hinge (swings under gravity = moves under torque) + a child slide
equality-coupled by a deg4 `tibia_tx` polyfit, ROM-limited, swung 1 s:
- **|slide − poly(knee)| = 0.25 mm** — the engine's `EqualityType::Joint` solver
  **holds the coupled slide on the constraint manifold while the knee accelerates
  under torque**, no blow-up, finite + bounded across 1000 steps. The named #1
  risk ("the solver keeping the knee slides coupled under dynamics") → **PASS.**
- physical |slide − SimmSpline(knee)| = 1.08 mm at deg4 (= fit error + solver
  tracking + edge); with deg8 (fit 0.14 mm) this drops to ~0.4 mm under dynamics.

**Stage 2b — NEW finding (R-extrapolation):** with NO joint range limit the
pendulum free-swung to −308°, far outside the fit ROM, where the **degree-4
polynomial extrapolates wildly while the SimmSpline clamps flat** (physical gap
blew to 98 mm — an extrapolation artifact, not a solver failure; solver tracking
stayed 0.65 mm even there). **Production lesson:** the polynomial equality
constraint is only faithful *inside* the fit ROM → the emitted twin MUST carry
the joint range limits (gait2392 `knee_angle_r` range −2.094…0.175 rad) so the
knee can't leave the ROM under dynamics. Carry into PR1.

**Deliberately NOT in the spike — the quantitative OpenSim-Manager accel match
(head-engineer call):** the prompt's "reproduces OpenSim forward accel" is the
end-to-end *validation*, but a meaningful muscle-activation→qacc comparison needs
the FULL emitted twin (Millard actuator + all couplings as constraints + ROM
limits) AND segment inertia reconciled against gait2392 — that's PR1+PR2-scale
setup, not a throwaway. A rushed spike comparison with unmatched inertia /
unconfined ROM would produce a *confounded, misleading* number — worse than none.
The spike's job was to de-risk the two NOVEL G3 mechanisms (coupling fidelity +
solver-holds-under-dynamics), both now retired; the engine's base rigid-body
forward dynamics is already validated by sim-core's 1330-test conformance suite.
So the OpenSim-Manager accel/trajectory gate is **PR2**, built properly with the
confound checklist above. (Decided 2026-06-08; surfacing for user visibility.)

---

## Proposed PR slicing (AFTER the spike confirms the approach — subject to revision)

> Slicing is the head-engineer's call and the spike may reshape it. Working draft:

- **PR1 — coupled knee as a degree-N equality constraint (engine + emit).**
  Two parts: **(1a) engine** — extend `EqualityType::Joint` from degree-4 to
  degree-≤10 (Horner loop over all 11 `eq_data` coeffs in
  `extract_joint_equality_jacobian` + its derivative; `builder/equality.rs`
  `.take(11)`; MJCF polycoef already variable-length). Add an engine-level test
  (a known deg8 coupling holds + its Jacobian derivative is correct). **(1b)
  emit** — `cf-mjcf-emit` fits a **degree-8** polynomial (dense LSQ, engine sign
  convention) to each coupled slide's SimmSpline over the working ROM and emits
  `<equality><joint joint1=slide joint2=knee_angle_r polycoef=…>`; constrains
  ALL couplings (tibia tx/ty **and** the patella moving-point slides — the quad
  moment arm rides the patella path, so faithful dynamics needs them); **emits the
  `knee_angle_r` joint range limit** so the constraint can't extrapolate outside
  ROM (R-extrapolation). Keep the kinematic `qpos_targets` path available
  (back-compat for the painted demo). New tests: the loaded twin holds the
  manifold statically (constraint residual ~0 across ROM = the constraint
  reproduces the old kinematic pose, ≤0.17 mm per spike) + the regenerated
  `knee_ref.xml` snapshot now carries the `<equality>` block. n+1 cold-read +
  pre-PR ultra-review.
- **PR2 — forward-dynamics gate vs OpenSim Manager.** `gen_forward_dynamics.py` →
  `forward_dynamics_opensim.json` (qacc at t=0 + short trajectory under a known
  activation, from a controlled 1-DOF setup with matched inertia/gravity). New
  cross-check test (CI, no OpenSim): drive the twin with the same activation from
  the same state, assert engine `qacc[knee]` matches OpenSim within budget;
  report the short trajectory. This is the end-to-end "activations → motion"
  validation. n+1 cold-read + pre-PR ultra-review.
- **PR3 — living muscle-driven sim (the demo payoff).** Convert `muscle_drive.rs`
  (or a new `muscle_forward.rs`) from the painted kinematic sweep into a true
  forward-dynamics run: set activations, integrate, watch the knee move under
  muscle force, slides riding the constraint manifold — cf-viewer PLY sequence
  tinted by live force (as today) but now MOVING under its own dynamics. Closes
  the G2 deferred "free forward dynamics" note.

(PR2 and PR3 may merge if small; PR1 may split if the spike forces option B.)

---

## Open risks / things to watch (carry into the spike + PRs)

- **R-fidelity** (the #1) — **RESOLVED by spike Stage 1:** degree-8 poly holds all
  couplings ≤0.17 mm position over the working ROM. Option C chosen.
- **R-solver-stability** — **RESOLVED by spike Stage 2:** the `Joint` constraint
  holds the coupled slide to 0.25 mm under torque-driven acceleration, no blow-up
  (solref `0.01 1`). PR-level may still tune `solref`/`solimp` for tightest
  tracking under muscle load (the gear example used `solref="0.05 1.0"`), but the
  mechanism works.
- **R-extrapolation** — **NEW (spike Stage 2b):** the polynomial constraint
  extrapolates wildly outside the fit ROM (deg4 free-swing to −308° → 98 mm gap)
  while the SimmSpline clamps. The emitted twin MUST carry the `knee_angle_r`
  range limit so the knee stays in ROM under dynamics. PR1 emits the limit.
- **R-patella**: 3 coupled slides per patella point — constrain (faithful,
  preserves quad moment arm under dynamics) vs weld (simpler, drops patella
  motion). Quad moment arm rides the patella path, so for muscle-DRIVEN dynamics
  the faithful answer is to constrain; the spike may weld to isolate, PRs decide.
- **R-inertia-match**: the forward-dynamics gate is only meaningful if segment
  mass/inertia match OpenSim. The emit's anthropometric inertia vs gait2392's
  must be reconciled (or the gate reads OpenSim's inertia into the twin). This is
  a real setup task for PR2, flagged early.
- **R-activation-dynamics**: first gate sets activation directly (pure mechanics);
  a later check could add the excitation→activation first-order ODE on both sides.
- **R-coupled-rotation**: gait2392 knee couplings are all translations, so the
  emit's coupled-rotation panic stays untouched this arc (note for completeness).
- **R-sign**: polycoef fit in the engine's knee-angle sign convention (flexion
  negative). Verify against a known pose before trusting the dynamics.

---

## Handoff anchors

- main clean + synced at `58fb2551` (G2 complete). This recon is the first G3
  artifact; nothing else changed.
- Engine equality constraint: `EqualityType::Joint`, `constraint/equality.rs:211`,
  MJCF `<equality><joint polycoef>`; degree-4 today, 6 spare `eq_data` slots.
- Coupled knee splines: `gait2392.osim:573` (`knee_r`), tx 12-knot wiggly / ty
  7-knot smooth; emitted as kinematic wrapper slide bodies
  (`cf-mjcf-emit/src/lib.rs:262`).
- OpenSim `Manager` forward integration available in `/tmp/osim-venv` (4.6).
- **SPIKE DONE then DELETED** (was `spike_forward_dynamics.rs`, `#[ignore]`,
  throwaway): R-fidelity → Option C (deg8, ≤0.17 mm); R-solver-stability → holds the
  manifold under dynamics; R-extrapolation → emit ROM limits. Findings live in the
  PR1 banner above.
- **PR1 IMPLEMENTED (branch `feat/msk-g3-equality-coupled-knee`, n+1 cold-read +
  pre-PR ultra-review done — 10 confirmed findings fixed, no correctness
  regressions; not yet pushed/merged pending user go-ahead).** See the ★ PR1
  SHIPPED banner up top.
- **NEXT: PR2 = OpenSim-`Manager` forward-dynamics accel/trajectory gate** (the
  deferred quantitative "activations→motion" check, built with matched
  inertia/gravity per the confound checklist). Do NOT push/open PRs without user
  go-ahead.
