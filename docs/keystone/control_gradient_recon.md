# Control gradient through the coupled rollout — RECON

*Opened 2026-06-13. The POLICY/RL half of the co-design optimizer — the second
half of the mission's "one outer loop differentiating w.r.t. **both** design and
policy parameters" (`MISSION.md` §"What we must build" #2).*

The DESIGN half is done and merged: the keystone differentiability arc (S1–S5,
PRs #299–#303) made `∂(rigid outcome)/∂(soft material)` differentiable; the
time-adjoint (#306) + IPC carry-fix (#307) made the **multi-step** coupled
gradient `∂z_N/∂μ` machine-exact through contact make/break; and the co-design
optimizer consumed it (v1 #304 single-step, v2 #308 trajectory, v3 #309
normalized-residual). This recon opens the **control/policy** half: optimize the
**control inputs** applied each step so a coupled soft↔rigid trajectory hits a
target, with the gradient flowing through the same differentiable coupled
rollout tape.

---

## 1. What the substrate provides, and the gap

`StaggeredCoupling::coupled_trajectory_material_gradient(n_steps, param_idx)`
(`sim/L1/coupling/src/lib.rs`) rolls the coupled system forward `n_steps` on ONE
chassis tape, then a single `tape.backward(z_N)` accumulates `∂z_N/∂p` into the
**material** leaf `p` (μ or λ). The per-step tape is:

```
x_prev,v_prev,p,z_prev ─[TrajectoryStepVjp]→ x*  (one dynamic soft Newton step)
x*,x_prev              ─[VelVjp]→            v   (v = (x*−x_prev)/Δt)
x*,z_prev             ─[ContactForceTrajVjp]→ fz (Σ force_on_soft.z at x*, plane z−clr)
vz,fz                 ─[VzCarryVjp]→         vz' (vz' = a·vz − (Δt/m)·fz [+ Δt·g const])
z,vz                  ─[ZCarryVjp]→          z'  (z' = z + Δt·vz, the OLD velocity)
```

The rigid carry's velocity half is the semi-implicit-Euler update of the platen:
in the real `step`, the reaction routed onto the rigid body is
`xfrc_z = −force_on_soft.z − rigid_damping·vz`, and sim-core integrates
`vz' = vz + (Δt/m)·(xfrc_z + m·g) = a·vz − (Δt/m)·fz + Δt·g` with
`a = 1 − Δt·c/m`. `VzCarryVjp` carries `∂vz'/∂vz = a`, `∂vz'/∂fz = −Δt/m`.

**THE GAP — there is no control input.** The platen today is purely passive
(gravity + contact reaction + linear damping). A *policy* needs a **control
force** `u_k` applied to the rigid body each step, and the gradient of the
trajectory cost w.r.t. those control inputs, `∂J/∂u_k`, threaded through the same
coupled rollout.

## 2. Where a control input enters — and why the gradient is *already* on the tape

A per-step vertical control force `u_k` on the platen adds to the same
`xfrc_applied[body].z` the contact reaction already uses:

```
xfrc_z = −force_on_soft.z − rigid_damping·vz + u_k
  ⇒  vz' = a·vz − (Δt/m)·fz + (Δt/m)·u_k + Δt·g
```

So **`∂vz'/∂u_k = +Δt/m`** — the *same* `dt_over_m` scalar factor the rigid
carry already uses for the contact-force term (and that `RigidStepVjp`/S4 already
crosses for the single-step load gradient). The control input is just an
additional **additive, axis-aligned force on the platen** — structurally a leaf
on the existing tape feeding the existing `VzCarryVjp` velocity node, with the
opposite sign of the contact-force term (control pushes `+`, the reaction enters
as `−fz`).

**The two gradient paths a control input `u_j` has on `z_N`** (both captured by
the tape, exactly as the material gradient is):

1. **Direct rigid path.** `u_j` bumps `vz'` at step j (`+Δt/m`); that velocity
   integrates into `z` on subsequent steps (`ZCarryVjp`), so `z_N` moves.
2. **Indirect coupled path.** Moving the platen changes the contact-plane height
   `z_prev` at later steps → changes soft penetration → contact force `fz` →
   the soft re-equilibration and the reaction back on the platen. This is the
   genuinely *coupled* sensitivity — the whole point of differentiating through
   the soft↔rigid contact, not just rigid kinematics.

Because every one of those edges is already a node on the tape
(`ContactForceTrajVjp`, `TrajectoryStepVjp`, `VzCarryVjp`, `ZCarryVjp`), adding
the control leaf and a single `+Δt/m·u_k` term into the velocity node is all that
is structurally new. **Conjecture (the spike tests it): the control gradient is
available, FD-exact, and machine-clean for the same reason the material gradient
is — it rides the C⁰ contact force, and the rigid carry is now off-by-one-free
(#307).**

## 3. Design — open-loop first

Per the head-engineer call (and the prompt's lean):

- **Open-loop control schedule FIRST.** Params = a force sequence
  `u_0 … u_{N−1}` (one scalar per step), gradient `∂J/∂u_k` read off each leaf
  via one `tape.backward`. No policy network, no BPTT-through-policy — each `u_k`
  is an independent tape parameter leaf, exactly like the material `p` leaf, so
  the reverse pass already gives the whole gradient vector in one pass.
- **Closed-loop feedback policy LATER.** Params = `π_θ` weights,
  `u_k = π_θ(state_k)`; needs differentiating the policy's state dependence
  (a small extra chain `state_k → u_k`). The open-loop primitive is the
  substrate it composes onto; defer until open-loop is FD-exact and shipped.
- **Analytic-gradient (differentiable) policy optimization, NOT model-free RL.**
  The differentiable substrate is the whole point of the keystone. Reuse the
  chassis Adam + `CoDesignProblem` trait + the v3 `Normalized` wrapper.

## 4. The new primitive (keystone side, `sim-coupling`)

A method analogous to `coupled_trajectory_material_gradient`:

```rust
pub fn coupled_trajectory_control_gradient(&mut self, controls: &[f64])
    -> (f64, Vec<f64>)   // (z_N, [∂z_N/∂u_0 … ∂z_N/∂u_{N−1}])
```

- `controls.len()` = `n_steps`; `controls[k]` is the vertical control force on
  the platen at step k (newtons, world `+z`).
- Forward: identical to the material rollout BUT each step applies the control
  force, i.e. `sf[5] = −force_on_soft.z − rigid_damping·vz + controls[k]`.
- Tape: each `u_k` is a `param_tensor` leaf; the step-k velocity node becomes a
  **3-parent** `VzCarryVjp` with parents `[vz, fz, u_k]` and coefficients
  `[a, −Δt/m, +Δt/m]`. (Make `VzCarryVjp` carry an optional control parent, or a
  dedicated `dvz'/du = +Δt/m` coefficient — settle in the spike.)
- `tape.backward(z_N)`; read `grad_tensor(u_k_var)` for each k.

**No material leaf needed** for the control-only gradient (μ held fixed), but the
two are not mutually exclusive — a future *joint* design+policy gradient keeps
both the `p` leaf and the `u_k` leaves on one tape and reads all of them from one
backward pass (the mission's "both design and policy" in one outer loop). The
spike confirms the control-only path first; joint is a documented follow-on.

### Forward-only FD oracle (for the gate)

A tape-free companion `coupled_trajectory_control_z(controls) -> z_N` that runs
the REAL coupled rollout (`step`-equivalent with the control force injected) and
returns `z_N`. The gate central-differences it w.r.t. each `u_k` independently
(re-rolling the real Newton solves + sim-core steps at `u_k ± ε`) — an
INDEPENDENT oracle, the keystone discipline (NOT an affine identity: the contact
force re-solves nonlinearly each perturbation).

## 5. The cf-codesign consumer

A `ControlScheduleTarget` `CoDesignProblem` (`tools/cf-codesign`):

- `n_params()` = `n_steps`; `evaluate(controls)` builds a fresh coupling
  (`&mut` rollout ⇒ rebuild per evaluate, as the trajectory material target
  does), reads `(z_N, ∂z_N/∂u)` from `coupled_trajectory_control_gradient`, and
  returns `(½(z_N − target_z)², residual · ∂z_N/∂u)`.
- Inverse-design gate: recover a *known* control schedule `u*` (set
  `target_z = z_N(u*)`) from `u_0 = 0` — given a target platen behavior, the
  optimizer finds the control inputs that produce it.
- Reuse Adam + `Normalized`. Control forces are weakly-sensitive and
  wide-magnitude (same as μ): `∂z_N/∂u_k ~ small`, so the `Normalized` wrapper
  (loss_scale = 1/L² dimensionless residual; log-space optional — but a control
  force can be **zero or negative**, so log-space does NOT apply element-wise to
  the control vector — see §7) conditions it so DEFAULT-eps Adam works.

## 6. #1 RISK and the spike

**R1 — is the control gradient actually FD-exact through the coupled rollout, or
does some control-specific path break the machine-clean result the material
gradient enjoys?** The conjecture (§2) is that it rides the same C⁰ contact
force and the now-fixed rigid carry, so it should be machine-exact. But control
enters the rigid carry *directly* (not only through the soft solve), so the
sensitivity structure differs from the material one — the spike must MEASURE it,
not assume it. → **S0 spike (throwaway): thread an open-loop schedule onto the
tape, FD-validate `∂z_N/∂u_k` against an independent re-rolled coupled-FD oracle
across a real multi-step rollout (deeply engaged + through a make/break if
reachable).** Either outcome is decisive: exact ⇒ proceed straight to the
sliced primitive; a break localizes exactly which control path the penalty
non-smoothness bites (and gives a measured gate).

**R2 — open- vs closed-loop tractability.** The spike confirms open-loop is
FD-exact first; closed-loop is deferred and only opened once the open-loop
substrate is shipped.

## 7. Gotchas / carry-forward (from the design-half + keystone gotchas)

- **Log-space does NOT apply to control forces.** The v3 `Normalized` log-space
  lever assumes *positive* params (it reparametrizes `p = ln(param)`). A control
  force is signed (can be 0 or negative). For the control consumer use the
  **loss_scale (dimensionless residual) lever only**, NOT log-space — OR a
  different conditioning if the residual-norm lever alone can't keep the gradient
  above eps. The spike measures the gradient magnitude to decide. (This is a real
  difference from the design half, where μ > 0 made log-space the strong lever.)
- **`&mut self` rollout** (advances the real `Data`, not `Clone`) ⇒ rebuild a
  fresh coupling per `evaluate` / per FD perturbation.
- **The `dt/m` factor** comes from `rigid_vz_response(0.0).1` (free-body,
  affine), already used for `dt_over_m`. The control coefficient is `+dt_over_m`
  (sign opposite the contact term's `−dt_over_m`).
- **`tape.constant(f64)` shape pitfall** — scalar leaves are `[1]` parents
  (`constant_tensor(&[v], &[1])`), per the time-adjoint gotcha.
- **cf-codesign on the xtask Layer-Integrity exemption allowlist**; pre-commit
  hook = whole-tree `cargo fmt` + conventional commits `<type>(<scope>):`.
- **KEEP the merged design-half paths untouched** (`SoftMaterialTarget` /
  `SoftMaterialTrajectoryTarget` / their gates / the material gradient method).
- **Grade Documentation tier** bites private intra-doc links → plain backticks
  for non-pub items; run the FULL `cargo xtask grade <crate>` (all 8 tiers).

## 8. Ladder / slicing

- **S0** (throwaway spike): measure `∂z_N/∂u_k` FD-exactness + open-loop
  verdict. Delete after.
- **PR-A** (sim-coupling): `coupled_trajectory_control_gradient` +
  `coupled_trajectory_control_z` (FD oracle) + the 3-parent control-carry node +
  FD gate (`tests/coupled_control_gradient.rs`; later retired into the `control`
  row of `tests/coupling_grad_harness.rs`) + lib smoke. grade A.
- **PR-B** (cf-codesign): `ControlScheduleTarget` consumer + inverse-design gate
  + worked example + the bundled `Normalized::evaluate` is_finite hardening
  (deferred from the v3 review). grade A.

Each leaf: recon → S0 spike → sliced PR → n+1 cold-read → branch+commit →
post-commit local ultra-review → grade A → push (await go-ahead) → PR → CI →
squash-merge.

---

## ★ S0 SPIKE DONE 2026-06-13 — GRADIENT MACHINE-EXACT. PROCEED (open-loop).

Implemented `coupled_trajectory_control_gradient` + `coupled_trajectory_control_z`
+ the 3-parent `VzControlCarryVjp` directly (the spike IS the primitive), and
gated `∂z_N/∂u_k` against the INDEPENDENT re-rolled coupled-FD oracle
(`coupled_trajectory_control_z` re-runs the real Newton solves + sim-core steps
at `u_k ± ε` — not an affine identity) across a real 8-step engaged rollout with
a varied (signed) control schedule `[−1.5, +1.0, …]`:

```
z_N(tape) == z_N(rollout)  to machine zero (forward replays the real dynamics)
k=0: tape=1.529e-5 fd=1.529e-5 rel=3.4e-11
k=1: tape=1.471e-5 fd=1.471e-5 rel=7.8e-11
...
k=6: tape=5.000e-6 fd=5.000e-6 rel=2.9e-11
k=7: tape=0         fd=0         rel=0       ← last step's velocity bump never
                                               integrates into a height (z' = z +
                                               Δt·vz_OLD), captured exactly
```

**FINDINGS (the recon's §6 R1 + §3 decisions, all RETIRED/confirmed):**
1. **R1 RETIRED — the control gradient is available + machine-exact** (rel
   ~1e-11) through the full coupled rollout, for every control input from ONE
   `tape.backward`. It rides the same C⁰ contact force and the now-fixed
   (#307) rigid carry as the material gradient. No control-specific path breaks
   it. → **proceed straight to the sliced primitive (no separate research leaf).**
2. **Open-loop confirmed tractable + FD-exact.** Each `u_k` is an independent
   tape leaf; all N gradients come from one backward pass. Closed-loop feedback
   stays a documented follow-on.
3. **Gradient magnitude is tiny (~1e-5)** — weak-sensitivity confirmed, so the
   consumer needs `Normalized` conditioning (the loss_scale / dimensionless-
   residual lever). **The schedule is SIGNED (negative forces used) → log-space
   does NOT apply to the control vector** (recon §7 confirmed): use loss_scale
   only.
4. **The decreasing `∂z_N/∂u_k` with k, → 0 at the final step**, is the correct
   physical signature: a control bump at step k raises `vz`, which integrates
   into `z` over the *remaining* `N−1−k` steps (position carries the OLD
   velocity), so the last step contributes 0. A useful sanity invariant for the
   gate.

**VERDICT: keep the implementation (FD-exact = the real primitive); delete the
throwaway test and replace it with the proper FD gate + lib smoke (PR-A); then
the cf-codesign `ControlScheduleTarget` consumer (PR-B).**
