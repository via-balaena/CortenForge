# Closed-loop policy gradient through the coupled rollout — RECON

*Opened 2026-06-13. The **closed-loop** feedback-policy step of the co-design
optimizer's policy half — and the gateway to **joint** design+policy, the
mission's "one outer loop differentiating w.r.t. **both** design and policy
parameters" (`MISSION.md` §"What we must build" #2).*

The OPEN-LOOP control half is done and merged (PR #310, `73a37325`):
`StaggeredCoupling::coupled_trajectory_control_gradient(controls)` treats each
per-step platen force `u_k` as an **independent tape parameter leaf** and returns
`(z_N, [∂z_N/∂u_0 … ∂z_N/∂u_{N−1}])` from one `tape.backward(z_N)`. That is a
*schedule* — `u_k` is a free variable with no dependence on the state. This recon
opens the **closed-loop feedback policy**: `u_k = π_θ(state_k)`, where the params
are the **policy weights θ** (shared across every step) and the control at step k
is a *function of the step-k state* — a genuine feedback law. The gradient
`∂z_N/∂θ` must flow through the coupled rollout **with the state→control
recurrence** (the policy at step k sees state that depends on the policy's own
outputs at steps `< k`) — i.e. backprop-through-time (BPTT).

---

## 1. What the substrate provides

`coupled_trajectory_control_gradient` (`sim/L1/coupling/src/lib.rs`) builds, per
step, this tape (identical to the material rollout except the velocity carry):

```
x_prev,v_prev,μ,z_prev ─[TrajectoryStepVjp]→ x*   (one dynamic soft Newton step)
x*,x_prev              ─[VelVjp]→            v    (v = (x*−x_prev)/Δt)
x*,z_prev             ─[ContactForceTrajVjp]→ fz  (Σ force_on_soft.z at x*, plane z−clr)
vz,fz,u_k             ─[VzControlCarryVjp]→  vz'  (vz' = a·vz − (Δt/m)·fz + (Δt/m)·u_k)
z,vz                  ─[ZCarryVjp]→          z'   (z' = z + Δt·vz, the OLD velocity)
```

The state vars **`z_var` and `vz_var` are loop-carried**: each step's
`z_next_var`/`vz_next_var` becomes the next step's `z_var`/`vz_var`. They are
already chained across step boundaries on one tape (the whole point of the
time-adjoint, #306). In the open-loop method `u_k = control_vars[k]` is a leaf
created *before* the loop, with no edge to any state var.

## 2. The closed-loop realization — `u_k = π_θ(state_k)` is a tape NODE, not a new primitive

(Transferable lesson #1 from the open-loop half: *is it a new primitive or a
node on the existing tape?*) The answer here is **node**. The policy params θ are
leaves created once, before the loop. At step k, the control becomes

```
u_k = π_θ(z_var_k, vz_var_k)
```

— a sub-expression built on the tape from the θ leaves and the **current**
state vars `z_var`/`vz_var` (already on the tape). Its output `u_var` replaces the
leaf `control_vars[k]` as the third parent of `VzControlCarryVjp`. Nothing else in
the per-step tape changes.

Because `z_var`/`vz_var` at step k are downstream of `u_{<k}` (via the velocity
carry: `vz_{k} = VzControlCarryVjp(vz_{k-1}, fz_{k-1}, u_{k-1})`, and `z` integrates
`vz`), the edges `u_k ← z_var_k, vz_var_k` close the recurrence on the tape:

```
θ ──┬─→ u_0 ─→ vz_1 ─→ (u_1, z_2) ─→ vz_2 ─→ … ─→ z_N
    ├─→ u_1 ──────────↗
    └─→ u_k (every step reuses the SAME θ leaves)
```

One `tape.backward(z_N)` then accumulates `∂z_N/∂θ` over **all** paths —
the direct push at every step AND every recurrent path where θ at an early step
moved the state the policy reads later. **BPTT is free**: the tape already does
reverse-mode over the chained state; feeding the state into a policy node and the
policy output into the carry just adds edges. *Conjecture to MEASURE in S0:
`∂z_N/∂θ` is FD-exact through the closed-loop recurrence.*

## 3. Where the differentiable policy lives — design fork

The tape inside `coupled_trajectory_*_gradient` is a **private** local
`Tape::new()` in sim-coupling; the consumer (cf-codesign) cannot reach into it.
So the policy sub-expression must be built **inside sim-coupling**. The chassis
`Tape` exposes full elementwise scalar autograd on `[1]`-shaped tensors —
`add`/`sub`/`mul`/`neg`/`tanh`/`square`/`affine`/… (verified in
`sim/L0/ml-chassis/src/autograd.rs`), each carrying its own VJP. So a policy can
be built as a **tape sub-expression** and the chassis autograd carries both
`∂u/∂state` and `∂u/∂θ` automatically — **no hand-rolled analytic `PolicyVjp`
needed** (the fork's alternative, only required if the primitives were absent —
they are not).

**Decision (S0 confirms): a `DiffPolicy` trait in sim-coupling** that *emits* its
control output onto the tape:

```rust
pub struct PolicyState { pub z: Var, pub vz: Var }   // minimal observation (lesson #4)

pub trait DiffPolicy {
    fn n_params(&self) -> usize;
    /// Emit the scalar control var u_k from the (shared) param leaves and the
    /// current platen state. The chassis autograd carries ∂u/∂θ and ∂u/∂state.
    fn emit(&self, tape: &mut Tape, params: &[Var], state: PolicyState) -> Var;
}
```

This is general (an MLP `π_θ` is a follow-on `DiffPolicy` impl using
`affine`/`tanh`) and reuses chassis autograd. The first impl is a **linear
feedback law** `LinearFeedback` (§4).

## 4. Policy form — linear feedback first

`u_k = w_z·z_k + w_vz·vz_k + b`, params θ = `[w_z, w_vz, b]`. This is a genuine
feedback law (`u` depends on state), tiny (3 params), physically a PD-style
controller around an implicit setpoint (`z_ref = −b/w_z` when `w_vz=0`). `emit`:

```
t1 = mul(w_z, z);  t2 = mul(w_vz, vz);  u = add(add(t1, t2), b)
```

The `w_z`/`w_vz` gradients are the real BPTT test — they flow *only* through the
recurrence (a weight on the state at step k changes `u_k`, which moves later
state the policy reads). `b` is the open-loop-like bias. An MLP is the follow-on
once the linear law is FD-exact.

## 5. Conditioning — RE-MEASURE per param set (lesson #2)

The open-loop half found control forces are **signed** → v3's log-space lever
(needs positive params) does NOT apply → the `loss_scale` (dimensionless-residual)
lever ALONE + a physical-scale `lr` was the move. For the linear policy the
weights `w_z`/`w_vz`/`b` are **also signed** → same regime expected:
`Normalized` with `loss_scale = 1/L²`, `log_space = false`, control-appropriate
`lr`. *But the gradient scales differ per param* (`∂z_N/∂w_z ~ z·∂z_N/∂u ~ 0.1·1e-5`
vs `∂z_N/∂b ~ ∂z_N/∂u ~ 1e-5`), so the loss gradient is even smaller — measure the
`loss_scale` magnitude the recovery needs. A PD-gain (positive `k_p`, `k_d ≥ 0`)
parametrization, if used later, may re-open the log-space lever — re-measure then,
don't assume. **Any new norm/stopping computation must guard the GRADIENT's
finiteness** (lesson #3: `grad_inf` via `f64::max` masks a NaN grad) — already
done in `Normalized::evaluate` (#310).

## 6. The FD oracle (independent, not an affine identity)

A forward-only `coupled_trajectory_policy_z(policy, params) → z_N` re-rolls the
**real** closed-loop coupled rollout (the policy re-evaluated each step from the
real `self.data` state, the real Newton solves + sim-core steps), so central FD
`(z_N(θ+ε) − z_N(θ−ε))/2ε` per param is a genuinely independent oracle — it
re-runs the recurrence, not a linearization. This is the S3/S4/S5/#310 discipline.

## 7. Joint design+policy (the second leaf, the mission's "both")

Once closed-loop is FD-exact, `coupled_trajectory_joint_gradient` keeps **both**
the μ material leaf (the λ=4μ total `∂/∂μ + 4·∂/∂λ`) AND the θ policy leaves live
on one tape, and reads `(∂J/∂μ, ∂J/∂θ)` from ONE backward — the μ leaf is no
longer a `constant_tensor` but a `param_tensor` feeding `TrajectoryStepVjp`, and
the policy θ leaves feed the carry, exactly as in the separate methods. cf-codesign
`JointTarget` optimizes `(μ, θ)` in one outer loop. Gate: FD both blocks vs the
independent oracle + a joint inverse-design recovering a `(μ*, θ*)` behavior. This
literally realizes `MISSION.md`'s "one outer loop differentiating w.r.t. both
design and policy parameters".

## 8. Plan / ladder

- **S0 spike (throwaway, measure-first):** temp `coupled_trajectory_linear_policy_gradient`
  + `_z` in sim-coupling + temp test. Measure: is `∂z_N/∂[w_z, w_vz, b]` FD-exact
  through the closed-loop recurrence (esp. the recurrent `w_z`/`w_vz`)? What
  `loss_scale` does recovery need? → then **revert** (crate/tree clean).
- **PR1 (sim-coupling):** `DiffPolicy` trait + `LinearFeedback` + `PolicyState` +
  `coupled_trajectory_policy_gradient` + `coupled_trajectory_policy_z` + FD gate +
  lib smoke. grade A.
- **PR2 (cf-codesign):** `FeedbackPolicyTarget` `CoDesignProblem` (loss `½(z_N−z*)²`,
  reuses Adam `optimize` + `Normalized`) + loss-FD gate + closed-loop behavior
  recovery + negative control + worked example. grade A.
- **JOINT:** `coupled_trajectory_joint_gradient` + cf-codesign `JointTarget` +
  joint FD/recovery gate. grade A.

## 8b. S0 SPIKE RESULT (2026-06-13 — PASSED, throwaway reverted)

Temp `spike_linear_policy_{gradient,z}` in sim-coupling + temp test, run then
reverted (tree clean). Linear policy `u = w_z·z + w_vz·vz + b`, θ = `[-20, -5, 2]`,
12-step engaged rollout. One `tape.backward(z_N)` vs the independent re-rolled
closed-loop FD oracle:

| param | grad (tape) | rel err vs FD |
|-------|-------------|---------------|
| `w_z`  | 1.610e-5 | **2.3e-11** (recurrent: weight on state z) |
| `w_vz` | 2.189e-4 | **1.3e-8** (recurrent: velocity feedback; abs 2.9e-12 = FD floor) |
| `b`    | 1.376e-4 | **4.7e-11** (bias) |

Forward `z_N`: tape == oracle to `<1e-12`. **VERDICT (conjecture §2 confirmed):
closed-loop BPTT through the state→control recurrence is machine-exact and FREE
via the tape** — feeding the loop-carried state vars into a policy node (built from
chassis primitives) and the policy output into `VzControlCarryVjp` is all it takes;
no hand-rolled `PolicyVjp`. The `w_z`/`w_vz` gradients (which exist *only* through
the recurrence) being FD-exact is the decisive evidence. **Conditioning (§5
confirmed):** loss-grad scale `~1e-5..1e-4` per param → with a residual `O(1e-3)`
the loss gradient sits near/below eps → `loss_scale` lever needed; weights are
signed → log-space N/A → `loss_scale`-only + physical `lr`, the same regime as the
merged `ControlScheduleTarget`. → **Proceed to PR1 with the `DiffPolicy` trait
(§3) + `LinearFeedback` (§4).**

## 8c. BUILT (PR1 + JOINT, 2026-06-13 — grade A all crates)

**PR1 (sim-coupling):** `DiffPolicy` trait + `PolicyState` + `LinearFeedback` +
`coupled_trajectory_policy_gradient` (one `tape.backward` over the closed-loop
rollout) + `coupled_trajectory_policy_z` (forward oracle) + gate
`tests/coupled_policy_gradient.rs` (FD-exact per param, incl. the recurrent
`w_z`/`w_vz`; forward-match; feedback-weights-live) + lib smoke. The policy is a
tape sub-expression (chassis `mul`/`add`), so chassis autograd carries ∂u/∂θ and
∂u/∂state — no hand-rolled `PolicyVjp`. Design fork resolved → **trait that emits
onto the tape** (§3).

**PR2 (cf-codesign):** `FeedbackPolicyTarget<P>` `CoDesignProblem` (loss
`½(z_N−z*)²`, reuses Adam `optimize` + `Normalized`) + gate
`tests/policy_inverse_design.rs` (loss-FD; recovery to |z−tgt| 1.3e-12; refined
negative control) + worked example. **Refined conditioning finding:** the linear
policy's bias `b` adds to *every* step (`∂z/∂b ≈ N·per-step ~ 1e-4`, ~10× a single
open-loop control), so the raw loss gradient starts only marginally above eps — the
raw run is *partially* effective (stalls ~1e-9) rather than fully dead;
normalization buys ~3 orders of deep convergence.

**JOINT (sim-soft + sim-coupling + cf-codesign):**
- *sim-soft:* `trajectory_step_vjp_combined(param_weights)` + the private combined
  assembler — a single material design parent driving a linear combination of
  material params (`[1, 4]` = the λ=4μ stiffness scale) so its cotangent is the
  **total `∂/∂μ + 4∂/∂λ` in ONE backward**. Merged `trajectory_step_vjp`
  byte-untouched. Gate: combined `[1,4]` == single-sum (linearity, <1e-12) ==
  rebuild-line FD (<1e-5).
- *sim-coupling:* `coupled_trajectory_joint_gradient(policy, params, n_steps)` →
  `(z_N, ∂z_N/∂μ_total, [∂z_N/∂θ])` — BOTH the μ design leaf (combined-weights soft
  node) AND the θ policy leaves on one tape, both read from one backward. Gate
  `tests/coupled_joint_gradient.rs` (both blocks FD-exact; joint θ block ==
  policy-only **bit-identical** ⇒ the material leaf doesn't perturb policy, fusion
  sound) + lib smoke.
- *cf-codesign:* `JointTarget<P>` over `p = [ln μ, θ…]` — owns the **mixed
  conditioning**: positive μ log-reparametrized *internally* (`μ = exp(p[0])`, the
  `μ·(…)` chain rule applied in `evaluate`), signed θ linear, the whole `p`-vector
  then linear so one `loss_scale`-conditioned Adam loop with one `lr` drives both.
  Gate `tests/joint_inverse_design.rs` (both blocks loss-FD incl. log-μ chain rule;
  recovery moving BOTH μ 2e4→29996 AND θ to |z−tgt| 4.3e-13) + worked example.
  **Conditioning finding:** the load-bearing lever for the joint is the **log-μ
  reparam**, not `loss_scale` (the reparam already scales the gradient; `loss_scale`
  is headroom). No clean `loss_scale` negative control (the reparam already
  converges raw, and under-determination lets θ compensate for a stuck μ).

**★ This realizes `MISSION.md`'s "one outer loop differentiating w.r.t. both design
AND policy parameters."**

## 9. Scope / caveats (carry the keystone's)

Engaged / stable-active-set / hard-penalty (or IPC) regime; constant-normal plane;
minimal observation (platen z, vz — richer obs is a follow-on); single scalar
target `z_N` (under-determined for >1 policy param, so behavior-recovery not
unique-param-recovery, the honest open-loop framing). The merged paths
(`SoftMaterialTarget`/`SoftMaterialTrajectoryTarget`/`ControlScheduleTarget`,
`coupled_trajectory_{material,control}_gradient`, `VzCarryVjp`/`VzControlCarryVjp`)
stay byte-untouched.
