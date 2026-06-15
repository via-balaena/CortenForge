# Quaternion joints (ball / free) — recon + S0

*Keystone deepening. Written 2026-06-15. The articulated coupling (#312/#313/#315/#316)
is machine-exact for hinge / hinge-chain — but all of it is **raw-`qpos`, no quaternion**.
The capstone exo has a floating base (free joint) + ball joints (shoulder/hip). This leaf
makes the coupled gradient correct for quaternion joints (`nq ≠ nv`): the rigid state's
position coordinates are a curved manifold (SO(3)/SE(3)), so the gradient must live in the
**tangent** space, not raw `qpos`.*

## 1. The gap (precise)

`StaggeredCoupling::loaded_state_jacobian` (the FD carry used for any non-single-hinge
body) does, for column `c < nv`:
```
qp = qpos.clone(); qp[c] += eps;            // RAW perturbation of qpos[c]
... j[(r,c)] = (qpp[r] - qpm[r]) / (2·eps)  // RAW difference of qpos'[r]
```
For a quaternion joint, `qpos[c]` is an un-normalized quaternion component and the raw
difference is NOT the SO(3) tangent — so `J_state` is wrong. Near identity (small rotation)
raw ≈ tangent, which is why the flat free-platen passes; it breaks at real angles. The
state also stores `s[i] = qpos[i]` for `i < nv` (a raw `nv`-slice of the `nq`-vector — drops
one quaternion component), but those VALUES are inert (the VJPs use only the Jacobians
`jz`, `J_state`, `G` — all should be tangent).

## 2. S0 spike (THROWAWAY, measure-first) — gap CONFIRMED, partial fix found

A **ball-joint** arm (`<joint type="ball"/>`, `nq=4`, `nv=3`) with an off-COM tip whose
height depends on the ball ORIENTATION (so `∂tip_z/∂μ` flows THROUGH the quaternion DOFs),
tilted `θ=0.3`, pressing the soft block. Tape gradient vs the (correct) full-coupled FD
oracle. (A free body with COM contact is DEGENERATE — the COM-height contact plane is
orientation-independent, so the quaternion never enters the μ-gradient; even a 107° free
rotation stays machine-exact. The off-COM ball joint is the scene that exercises it.)

```
                                  rel error vs FD
                          RAW qpos        TANGENT loaded_state_jacobian
ball-joint n=1            (—)             2.55e-2
ball-joint n=2            (—)             3.18e-1
ball-joint n=4            1.15  (sign!)   9.82e-1
ball-joint n=6            0.72            9.84e-1
```

- **RAW is severely wrong — even the SIGN is wrong at n=4** (tape −6.5e-6 vs FD +4.4e-5).
- The forward MATCHES the oracle exactly at all n (the real sim integrates quaternions
  correctly via `mj_integrate_pos`); the defect is purely in the gradient.
- **Tangent FD on `loaded_state_jacobian`** (perturb via `mj_integrate_pos_explicit`,
  difference via `mj_differentiate_pos`) is NECESSARY (fixes the sign) but NOT SUFFICIENT —
  2.5% at n=1, growing. The n=1 error ⇒ a PER-STEP quaternion factor is still wrong, not
  just compounding.

## 3. The remaining touchpoint (hypothesis for PR1 — measure to confirm)

The carry's position-row term is `∂qpos'/∂w = Δt·G_vel` (`RigidStateCarryVjp.g_pos_dt = Δt`).
That is exact for a linear coordinate (`qpos' = qpos + Δt·qvel'`), but for a quaternion the
integrator is `q' = q ⊕ exp(Δt·qvel')`, whose tangent Jacobian is `∂(tangent q')/∂qvel' =
Δt·Jr(Δt·qvel')` — the SO(3) right-Jacobian `Jr`, which `Δt·G_vel` (i.e. `Jr = I`) omits.
This is the leading suspect for the n=1 residual (≈ `½·Δt·qvel` first-order). Likewise the
analytic single-hinge `J_state` path is hinge-only (unaffected), but the free/chain FD path
and the position-row term both need the integrator's tangent Jacobian.

## 4. Design plan (sliced PRs — measure-first each)

1. **PR1 — tangent FD carry + tangent position-row.** Make `loaded_state_jacobian` FD in
   tangent space (done in S0, env-gated — promote to production). Then pin the remaining
   per-step residual: replace the `Δt·G_vel` position-row with the true
   `∂(tangent qpos')/∂w` (FD it in tangent, or `Δt·Jr·G_vel` via `mjd_quat_integrate`).
   Gate the ball-joint scene machine-exact at n=1…6. CONFIRM the Jr hypothesis (§3) by
   measurement before coding the analytic form.
2. **PR2 — free joint (SE(3), `nq=7,nv=6`) with off-COM contact.** A free body whose contact
   point is off the COM (so orientation enters), machine-exact. (The flat platen already
   passes degenerately.)
3. **Follow-on** — analytic chain carry for nv>1 quaternion chains; the exo's full joint set.

## 5. Watch (carry forward)
- sim-core tangent ops: `mj_integrate_pos_explicit` (tangent→coord), `mj_differentiate_pos`
  (coord→tangent), `mjd_quat_integrate` (the quaternion-integration derivative) — all
  exported from `sim_core`. `transition_derivatives.A` is ALREADY tangent (`dq`), so the
  analytic hinge path is unaffected.
- The pose seam `jz` (`mj_jac_point` row 5) and `jlin` (`com_linear_jacobian`) are already
  tangent (`nv`-column Jacobians) — likely correct; re-validate in PR1.
- Keep the merged hinge/chain + scalar-platen paths byte-untouched (single_hinge / nq==nv
  unaffected; the change is in the FD carry + the position-row term for quaternion joints).

## 6. PR1 SHIPPED (2026-06-15) — ball joint machine-exact; a THIRD bug found + fixed

The two-touchpoint plan was right but INCOMPLETE — there was a third, foundational defect.

**Implemented (all in `sim-coupling`, except the sim-core fix):**
1. `loaded_state_jacobian` → tangent FD for `nq ≠ nv` (`loaded_state_jacobian_tangent`):
   position columns step via `mj_integrate_pos_explicit`, position rows difference via
   `mj_differentiate_pos`. The `nq == nv` path keeps the raw FD VERBATIM (byte-identical
   hinge/chain).
2. Position-row carry `G_pos = D·G_vel` with `D = ∂(tangent qpos')/∂qvel'` block-diagonal:
   `Δt` (Euclidean) / `Δt·J_r(Δt·qvel')` (quaternion), `J_r` = SO(3) right Jacobian
   (`right_jacobian_so3`, new). Replaces the scalar `g_pos_dt`. Euclidean ⇒ `Δt·G_vel`
   (byte-identical). **This `J_r` fixed the S0 2.5%@n=1 residual → 2e-7.**
3. **★ THE THIRD BUG (S0 missed it): `mj_differentiate_pos` had a lossy log map.** It read
   `sin(θ/2)` from `sqrt(1−w²)` and the angle from `acos(w)` — for a TINY FD rotation
   (`θ ~ Δt·ε ~ 1e-9`, the position-VELOCITY block) `w = 1 − O(1e-18)` rounds to exactly
   `1.0`, so `sqrt(1−w²)=0` → it returned ZERO. The position-velocity Jacobian
   `∂(tangent qpos')/∂qvel ≈ Δt·I` came out as ZERO → n=2 was 32% wrong. Fixed in
   `sim/L0/core/src/jacobian/position.rs` to the robust form (`sin_half = ‖vec‖` directly +
   `atan2`), matching MuJoCo's `mju_quat2Vel`. **This also fixes sim-core's own
   `mjd_transition_fd` for ball/free joints** (same caller). Strictly more accurate, all 642
   sim-core lib tests still green.

**Gate:** `ball_joint_gradient_matches_fd` at n=1,2,4 — machine-exact (rel 2e-7 / 1.7e-6 /
6.3e-6, FD-carry precision like the chain). **Refutes S0's wrong-sign rel-1.15@n=4.** A ball
joint is UNCONSTRAINED (no restoring stiffness), so the violent contact impulse launches the
arm into a chaotic regime by n≈6 (tip leaves the block, FD non-convergent — the GRADIENT is
ill-conditioned, not the tape); long-rollout robustness stays with the hinge/free/chain
gates. Two new lib unit tests: `right_jacobian_so3_matches_quaternion_expmap_fd` (math) and
`loaded_state_jacobian_ball_velocity_block_matches_analytic` (the atan2 regression guard).

**Convention note (carry forward to PR2):** `mj_differentiate_pos` / `right_jacobian_so3`
measure the output qpos' tangent at the NOMINAL output `q'` (the body-frame `q'⁻¹·q'_new`),
which is what `jz`/`jlin` consume. `mjd_quat_integrate` / `transition_derivatives.A` use a
DIFFERENT, internally-consistent convention (output tangent referenced at `q_old`), so their
position-POSITION blocks legitimately DIFFER from the FD carry — do NOT "reconcile" them.
