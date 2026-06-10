# Keystone S4 — the soft-tape `VjpOp` crossing — RECON

*Active recon, opened 2026-06-10. The keystone leaf the whole differentiability arc was building
toward: ONE `tape.backward` that crosses BOTH engines. S1–S3 assembled the interface factor
derivatives and validated them piecewise against FD; S4 wires them onto the soft autograd `Tape`
as `VjpOp`s so a single reverse pass from a rigid-side scalar produces a gradient w.r.t. a
soft-side parameter. Sibling of `recon.md`, `s2_differentiability_recon.md`,
`s3_soft_pose_sensitivity_recon.md`. `MISSION.md`'s "genuinely open research problem" — the
gradient substrate the co-design optimizer ultimately consumes.*

> **Establish a single `Tape::backward` from a rigid-engine scalar outcome (a platen velocity)
> back to a soft-engine parameter (the soft load θ), crossing the soft↔rigid interface on ONE
> tape: chain the existing soft Newton adjoint (`NewtonStepVjp`, θ→x*) with a contact-force
> readout `VjpOp` (x*→force) and a NEW rigid-step `VjpOp` (xfrc→s') — the rigid engine has no
> tape, so wrapping its Jacobian as a chassis `VjpOp` is the crossing. FD-validated against the
> full coupled step.**

---

## 1. What exists, and the gap

| Piece | State |
|---|---|
| chassis `Tape` / `Var` / `VjpOp` / `push_custom` / `backward` | **available** (`sim_ml_chassis::autograd`). `backward` seeds the output cotangent with ones, walks in reverse, calls each `VjpOp::vjp(cot, parent_cotans)` accumulating into parents. |
| soft Newton adjoint on the tape | **available** — `Solver::step(tape, x_prev, v_prev, theta_var, dt)` pushes `x_final = push_custom([theta_var], x_final[n_dof], NewtonStepVjp)` (`push_newton_step_vjp`). `NewtonStepVjp::vjp` takes `∂L/∂x*` (shape `[n_dof]`) and accumulates `∂L/∂θ` into `theta_var`. **Load-only** (θ), which is exactly what S4's first crossing differentiates. |
| soft contact-force Jacobian `∂force/∂x*` | **derived (S3)** — `−κ·n̂⊗n̂` over active pairs (used in `contact_force_height_total_jacobian`). Not yet a `VjpOp`. |
| rigid step Jacobian `∂s'/∂xfrc` | **derived (S2)** — `rigid_step_probe` (= closed-form free-body `dt/m`; `xfrc` is NOT in `transition_derivatives`' `B`). Not yet a `VjpOp`. |
| **one `tape.backward` across both engines** | **MISSING — this leaf.** The rigid engine has NO tape; its derivative is dense `TransitionMatrices` / the `dt/m` probe. Nothing wraps it as a `VjpOp`, so a soft-tape backward stops at x*. |

**The obstacle (precise):** the soft engine differentiates on a `Tape`; the rigid engine
differentiates as dense Jacobians off-tape. To cross, the rigid Jacobian (and the contact-force
readout) must become `VjpOp` nodes ON the soft tape, so the chassis `backward` walks through them.

## 2. The chain (one coupled step, on one tape)

For a single staggered step the rigid pose is fixed (read at step start), so the contact plane is
a constant during the soft solve. With a soft **load** `θ` as the differentiable handle:

```text
 θ (leaf Var, [1] or [3·n_loaded])
   │  Solver::step  → push_custom([θ], x*, NewtonStepVjp)          [SOFT engine, exists]
   ▼
 x*  ([n_dof])
   │  push_custom([x*], force, ContactForceVjp)   vjp: ∂L/∂force → ∂L/∂x* = (−κ n̂⊗n̂)·(…)   [NEW]
   ▼
 force_on_soft  ([3] or fz [1])
   │  Tape::neg                                                     [chassis primitive]
   ▼
 xfrc = −force
   │  push_custom([xfrc], vz', RigidStepVjp)   vjp: ∂L/∂vz' → ∂L/∂xfrc = (∂s'/∂xfrc)ᵀ·(…)   [NEW — RIGID engine as a tape op]
   ▼
 vz'  ([1], the platen's next vertical velocity = the loss)
```

`tape.backward(vz')` seeds `1`, flows `vz' → xfrc → force → x* → θ`, and `grad(θ)` is the
cross-engine gradient `∂vz'/∂θ`. Each arrow is one tape node; `NewtonStepVjp` consumes the
`∂L/∂x*` the `ContactForceVjp` produces — they compose because `NewtonStepVjp` is linear in its
incoming cotangent (it does not care that the cotangent came from a physics op rather than a
scalar loss).

**Why θ (load), not material/pose, for THIS leaf.** `NewtonStepVjp` already differentiates x*
w.r.t. θ — so the crossing mechanism (rigid `VjpOp` chained onto the soft tape) can be built and
FD-validated NOW, without the soft material VJP (which does not exist — the next leaf) or routing
the S3 pose adjoint through the tape. θ is the handle the existing tape supports; the material VJP
leaf then swaps θ for material params through the same crossing. (Matches S2 recon D5: wrap the
rigid Jacobian as a `VjpOp` first; the material-param co-design gradient is its own sub-task.)

## 3. Decisions (head-engineer)

- **D1 — Differentiate a rigid outcome w.r.t. the soft LOAD θ.** The existing `NewtonStepVjp`
  supports it, so S4 isolates the *crossing* (the new rigid + force `VjpOp`s and the one-tape
  backward) from the *material-VJP* work. The scene: a soft block pinned at the base, pressed by a
  `sim-core` platen (contact), with a scalar `AxisZ` load θ on free soft vertices — θ changes x*,
  the contact force, hence the platen's next velocity. Genuinely cross-engine.
- **D2 — Wrap the RIGID step as a chassis `VjpOp` (`RigidStepVjp`); keep it engine-generic.**
  Output = the rigid next-state scalar(s); parent = the `xfrc` Var. `vjp` applies `(∂s'/∂xfrc)ᵀ`,
  computed via the S2 probe (closed-form `dt/m` for the free platen; FD column for generality).
  This is the reusable "rigid engine as a tape op" artifact (the co-design loop differentiates
  rigid outcomes generally). Lives in `sim-coupling` (the only crate linking both engines + the
  chassis tape).
- **D3 — `ContactForceVjp` for `force_on_soft(x*)`.** Parent = x_final Var; `vjp` applies the
  `−κ·n̂⊗n̂` contact-force Jacobian over the active set (the S3 factor), producing a `[n_dof]`
  cotangent into x*. Reuses the penalty readout.
- **D4 — FD oracle gates the tape gradient.** Mirror S1–S3: a black-box full-coupled-step FD
  (perturb θ, re-run soft solve → force → rigid step → vz', central-difference) is the independent
  oracle; the tape `grad(θ)` must match it. Engaged, stable-active-set regime (R3 cap).
- **D5 — Use the soft `step` (tape), not `replay_step`.** The crossing needs x* ON the tape; the
  coupling's forward path uses the tape-free `replay_step`. S4 adds a tape-based coupled step
  alongside it.

## 4. #1 RISK and the spike

**#1 RISK — does chaining a rigid `VjpOp` + a contact-force `VjpOp` onto the soft `NewtonStepVjp`
node produce a correct end-to-end gradient through ONE `tape.backward`?** Sub-risks: **(a)**
cotangent shape/sign plumbing across four nodes (vz' `[1]` → xfrc `[3]` → force `[3]` → x*
`[n_dof]` → θ); the `ContactForceVjp` must emit exactly the `[n_dof]` shape `NewtonStepVjp`
expects. **(b)** the soft `step` (tape, which also factors A for the adjoint) must converge to the
same x* the coupling's `replay_step` does, with a θ load + contact active. **(c)** the
engaged-regime / stable-active-set cap (R3) — same as S1–S3. **(d)** does `NewtonStepVjp` correctly
turn a *physics-sourced* `∂L/∂x*` (not a simple loss) into `∂L/∂θ`? (It is linear in the
cotangent, so it should.)

**SPIKE (throwaway, `#[ignore]`):** build the minimal tape chain of §2 for a loaded soft block +
platen contact, run one `tape.backward(vz')`, read `grad(θ)`, and compare to a full-coupled-step
central FD (perturb θ, re-solve soft + force + rigid step, difference vz'). Confirm match to FD
tolerance and a stable active set across the perturbation. Retires the crossing risk before the
production `VjpOp`s.

## 5. Sub-leaf ladder / slicing

- **Spike (THROWAWAY, `#[ignore]`).** §4 — prove the one-tape cross-engine backward matches FD.
- **PR1 — `RigidStepVjp` (the rigid engine as a tape op).** Public `VjpOp` in `sim-coupling`
  wrapping `∂s'/∂xfrc`; unit-test its `vjp` against `rigid_step_probe` / FD (the `dt/m` factor).
  The novel, standalone-testable artifact.
- **PR2 — the full crossing.** `ContactForceVjp` + a tape-based coupled step that chains
  `NewtonStepVjp` → `ContactForceVjp` → `neg` → `RigidStepVjp`; `tape.backward(vz')` → `grad(θ)`,
  FD-gated against the full coupled step.

Each leaf: n+1 cold-read + pre-PR local ultra-review; no push/PR without go-ahead.

**SPIKE DONE 2026-06-10 (throwaway, deleted) — ★ R1 + R2 RETIRED.** A `#[ignore]`'d test in
`sim-coupling` built the full §2 chain for a loaded soft block (top-face scalar `AxisZ` load θ) +
platen contact at `h=0.099`: `Solver::step` (tape) → `ContactForceVjp` (x*→fz) → `neg` →
`RigidStepVjp` (xfrc→vz'), then `tape.backward(vz')`:

| quantity | value |
|---|---|
| `∂vz'/∂θ` (one tape.backward) | **9.800414e-2** |
| full-coupled-step central FD | **9.800414e-2** |
| **rel** | **5.1e-10** |
| active set across `±ε` | 25 / 25 stable |
| `x*(tape) − x*(replay)` L2 | **0.0** (exact — R2 retired) |

So a single `tape.backward` from a rigid-engine scalar (`vz'`) to a soft-engine parameter (`θ`)
crosses both engines correctly: the rigid `VjpOp` + contact-force `VjpOp` compose with the
existing `NewtonStepVjp` (the physics-sourced `∂L/∂x*` flows into the load adjoint), and the tape
`step` converges bit-identically to `replay_step`. The crossing mechanism is confirmed; ready to
productionize the two `VjpOp`s + the FD-gated coupled-step gradient.

## 6. Risks

- **R1 (#1) — chained `VjpOp` correctness / shape+sign plumbing.** §4 spike measures it directly.
- **R2 — tape `step` vs `replay_step` divergence** (the tape path factors A for the adjoint).
  MITIGATION: assert x*(tape) == x*(replay) in the spike.
- **R3 — penalty active-set non-smoothness.** Engaged regime, documented cap (IPC deferred), as
  S1–S3.
- **R4 — `∂s'/∂xfrc` is off `transition_derivatives`' `B`** (ctrl-only) and `xfrc` auto-zeroes
  each step. MITIGATION: `RigidStepVjp` uses the S2 `rigid_step_probe` path (scratch `Data`,
  closed-form `dt/m` / FD column) — already validated.
- **R5 — `Data` is not `Clone`.** Reconstruct via `make_data` + copy qpos/qvel (the S2 pattern).

## 7. Validation gate (S4)

CI-runnable: (PR1) `RigidStepVjp::vjp` matches the `rigid_step_probe` / FD rigid factor; (PR2) a
single `tape.backward` from the platen's next velocity to `grad(θ)` agrees with a full-coupled-step
central FD in the contact-engaged regime. Honest scope: load-θ handle (material-param VJP is the
next leaf); single staggered step (plane fixed during the soft solve); engaged / stable-active-set
(penalty cap documented, IPC deferred); the rigid factor is the scalar `∂vz'/∂xfrc_z` (`= dt/m`,
exact for the free / axis-decoupled platen — a constrained or rotationally-coupled rigid body is
non-affine and needs the full `∂s'/∂xfrc`, out of scope). The committed gate checks two operating
points (`(h, θ) = (0.099, 5.0)` and `(0.097, 9.0)`); the soft `θ→x*→fz` map is the nonlinear leg,
the rigid leg is exactly affine.

## Key files / pointers

- chassis tape: `sim/L0/ml-chassis/src/autograd.rs` (`Tape`, `Var`, `VjpOp`, `push_custom`,
  `backward`, `grad_tensor`, `neg`).
- soft tape node: `sim/L0/soft/src/solver/backward_euler.rs` (`step`, `push_newton_step_vjp`);
  `differentiable/newton_vjp.rs` (`NewtonStepVjp`, load-only θ adjoint).
- rigid Jacobian: `sim/L0/core/src/derivatives/mod.rs` (`transition_derivatives`,
  `TransitionMatrices` — `B` is ctrl-only); the S2 `StaggeredCoupling::rigid_step_probe`.
- soft contact-force Jacobian (S3): `sim/L1/coupling/src/lib.rs`
  (`contact_force_height_total_jacobian`'s `−κ·n̂⊗n̂` implicit term); `contact/penalty.rs`.
- coupling: `sim/L1/coupling/src/lib.rs` (`StaggeredCoupling`, `rigid_step_probe`,
  `contact_force_at_height`).
