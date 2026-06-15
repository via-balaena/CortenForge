# Multi-link articulated coupling — recon + S0

*Keystone deepening. Written 2026-06-15. The articulated coupled gradient (#312/#313/#315)
is validated only for a SINGLE HINGE (`nv = 1`, raw `qpos`). The capstone exo is a
multi-joint mechanism, so the next reach is multi-link (`nv > 1`) and eventually
ball/free joints (`nq ≠ nv`, quaternion). The carry's analytic geometric stiffness
(#315) already falls back to the FD `loaded_state_jacobian` (general `nv`) for multi-link,
and the rest of the path (pose seam, wrench node, `G_vel = rigid_xfrc_column`) is written
`nv`-general — so the hypothesis was "a 2-link chain may already work via the FD carry."*

## S0 spike — FALSIFIED: the 2-link gradient is structurally broken (n=2)

A 2-link hinge chain (`nv = 2`, both axes Y, distal tip on the soft block), through the
EXISTING `coupled_trajectory_material_gradient_articulated` (FD-carry fallback), gated
against the full-coupled re-rollout FD:

```
2-link n=2: tape= 9.77e-8   FD= 5.63e-8   rel = 7.4e-1   ← 74% WRONG at n=2
2-link n=4: tape= 9.41e-7   FD= 6.82e-7   rel = 3.8e-1
```

The FD oracle is **fully converged** (identical across eps_rel 1e-3…1e-6) — so these are
GENUINE tape errors, not FD noise, and not the chaotic-bounce regime (a gentler,
heavier, near-equilibrium scene gives the same ~74% at n=2). **This is a structural nv=2
bug**, distinct from the off-COM moment eval-skew (which is negligible at n=2 — the
single hinge is machine-exact at n=2). The single-hinge (`nv = 1`) scope masked it.

So multi-link is NOT a trivial extension of the validated path — it is a real leaf:
localize and fix the structural nv>1 gradient error, THEN (optionally) the analytic chain
carry (the Jacobian Hessian + `∂M⁻¹/∂q`, #315's documented follow-on — and for the
violent multi-link step the FD carry may itself be a precision liability, so the analytic
carry may be load-bearing for correctness here, not just a short-rollout tightening),
THEN ball/free joints (quaternion, `nq ≠ nv`).

## ★★ RESOLVED (2026-06-15) — the fully-fresh formulation fixes nv=2

Localized: the 2-link forward MATCHES the oracle exactly (forward replay is fine); the
bug is purely in the gradient, in step 0's contribution. Root cause: the output is read
STALE (`tip_z_N = xipos(q_{N-1})`, attributed to `s_N`), and the stale-FK + §8a-drop
calibration relies on `∂qpos'/∂qvel = Δt·I` — true for a single hinge (no Coriolis),
FALSE for a chain (velocity coupling). The fix is the **fully-fresh matched formulation**
(fresh-FK contact pose + fresh output + true position-row carry `∂qpos'/∂w = Δt·G_vel`),
which is the correct differentiable formulation for ANY nv. Shipped; the 2-link gradient
now matches the full-coupled FD at n = 2/6/10 (`twolink_chain_gradient_matches_fd`). Full
write-up: `moment_residual_recon.md` §3f.

Accuracy note: the chain uses the FD `loaded_state_jacobian` carry (single-hinge has no
analytic `J_state`), so it is FD-CARRY precision (~1e-6), not the hinge's ~1e-9. The
analytic CHAIN carry (the Jacobian Hessian + `∂M⁻¹/∂q`) is the remaining follow-on for
machine-exactness at nv > 1; ball/free joints (quaternion, `nq ≠ nv`) are the leaf after.

## (historical) Suspects for the n=2 nv=2 bug

Localize with the per-step-μ-leaf tape vs per-step-truncated FD instrument (as in
`moment_residual_recon.md` §1). At n=2 the single hinge is exact via: step 0 from rest
(`dw/dμ ≈ 0`) + the lagged output. Candidates for what breaks at `nv = 2`:
1. The FD `loaded_state_jacobian` carry off-diagonal accuracy / the `J_state·s` contraction
   for the 4-vector state.
2. The §8a position-row drop generalized to `nv > 1` (zeroing `G`'s 2 position rows).
3. The pose-seam / wrench `∂·/∂s` attribution over a 2-DOF state.
4. `rigid_xfrc_column` (`G_vel`, `nv×6`) off-diagonal columns at `nv = 2` (validated in
   `rigid_multidof_response` for a 2-link at the primitive level — re-check in the coupled
   composition).
