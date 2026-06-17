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

## ★★★ RESOLVED (2026-06-17) — the analytic UNDAMPED chain `J_state` (machine-exact)

The "analytic CHAIN carry" follow-on above is now SHIPPED: the undamped serial-hinge chain
uses the analytic loaded transition `J_state = A + Δt·M⁻¹·(G − dMu)`
(`StaggeredCoupling::chain_state_jacobian`), machine-exact vs the full-coupled FD
(`twolink_chain_gradient_matches_fd`: n=2 rel 6e-10, n=6 5e-9), replacing the FD `J_state`.

Pieces (each S0-spike-validated):
- `A` = the UNLOADED transition (`transition_derivatives`) — but it MUST be evaluated at the
  CLEAN (`xfrc_applied = 0`) operating point (see bug #2).
- `G = ∂(Jᵀw)/∂q` — the geom-stiff Hessian, case-split closed form: for joint j (world axis
  `â_j`, anchor `o_j`, arm `r_j = p − o_j` to the contact COM `p`),
  `G[j,k] = (∂â_j/∂q_k)·(τ + r_j×f) + â_j·((∂r_j/∂q_k)×f)`, with `∂â_j/∂q_k = â_k×â_j`,
  `∂o_j/∂q_k = â_k×(o_j−o_k)` (both only when k is a STRICT ancestor of j), and
  `∂p/∂q_k = â_k×(p−o_k)`. Reduces to the single-hinge `(â×(â×r))·f` at j=k.
- `dMu = (∂M/∂q)·u`, `u = M⁻¹Jᵀw` (`sim_core::mass_directional_derivative`); the `∂M⁻¹/∂q`
  term `= −M⁻¹·dMu`. Vanishes for a single hinge (constant M); material for a chain.

### ★★ THE RE-SPIKE FOUND TWO BUGS the "it's just assembly" spec premise hid:

**Bug #1 (sim-core) — the Coriolis `∂S/∂q` ancestor term.** `mjd_rne_pos`'s Part B
(Coriolis) dropped `cvel[parent] ×_m ((∂S/∂q)·qvel)` as "zero for hinges" — TRUE only for
the SELF term, FALSE for an ANCESTOR: a hinge's world-frame axis rotates when an ancestor
rotates (`∂â_j/∂q_k = â_k×â_j`). Zero for a PARALLEL-axis chain (which is why
`n_link_pendulum` never caught it), material for a SPATIAL chain (the analytical position
columns / `transition_derivatives.A` velocity-position block were ~21% wrong). Added the
ancestor term; regression `analytical_transition_matches_fd_nonparallel_chain`. Part A
already had the analogue. **A single hinge is immune — a 1-DOF system has no Coriolis**,
which is why the single-hinge analytic `J_state` was always exact.

**Bug #2 (coupling) — `A` computed at a STALE operating point.** `chain_state_jacobian`
first read `A` from `self.data.transition_derivatives(...)`, but `self.data.xfrc_applied`
still holds the PREVIOUS step's contact wrench (sim-core's `step` does not clear it).
`transition_derivatives` derives its operating-point `qacc` from an internal nominal step
that consumes that stale wrench, and the chain's `∂(M·qacc)/∂q` term then uses the
contaminated `qacc` → `A` was the loaded transition w.r.t. the STALE wrench, so adding the
current-wrench `addend` double-counted. **Step 1 was exact (xfrc=0 from rest); step ≥2
compounded** — the classic "exact@n=1, drifts@n ⇒ wiring, not per-step" signature. Again a
single hinge is immune (`∂M/∂q = 0` ⇒ `qacc` operating point irrelevant). Fix: compute `A`
on a fresh `xfrc=0` scratch (with `ctrl` copied — the actuator input IS a held transition
input). This is also why the DIRECT `chain_state_jacobian_matches_fd_loaded` test passed
while the end-to-end failed: the direct test builds a fresh coupling (xfrc already 0).

### ★ METHOD LESSON
The direct per-step `J_state`-vs-FD test (tol 1e-5) hid both: bug #1 was small at the test's
modest qvel; bug #2 needs a stale xfrc that only exists mid-rollout. The END-TO-END
gradient-vs-rerolled-FD gate is what exposed them. Also: FD-carry was ALREADY ~1e-8 (not the
"~1e-6" the old docstrings claimed), so the analytic carry's win is determinism + a clean
base for the damped chain, more than raw precision. Use a STABLE rollout to judge (the stiff
2-link slams the underactuated distal link to qvel~600 — a diverged trajectory where even
the analytical `A` and FD legitimately disagree; not a real defect).

### ★ SCOPE: nv == 2 (single-hop ancestors); multi-hop is a follow-on
The analytic chain `J_state` (and bug #1's sim-core `∂S/∂q` fix) is VALIDATED for the 2-link
(single-hop ancestor) chain. A 3+-link chain has MULTI-HOP strict ancestors (the tip's
grandparent joint), where the sim-core Coriolis position derivative is STILL incomplete: a
spatial 3-link's unloaded `A` is ~10% off vs FD, while a PARALLEL 3-link is exact — isolating
the residual to the non-parallel multi-hop `∂S/∂q` path (the same class as bug #1, not yet
fully closed). `chain_state_jacobian` therefore gates to `nv == 2`; `nv > 2` declines to the
FD `loaded_state_jacobian` (machine-exact vs the re-rolled oracle at ~1e-8). Completing the
multi-hop sim-core term (so the analytic carry covers a full leg, nv ≥ 3) is the follow-on.

## ★★★★ RESOLVED (2026-06-17) — multi-hop Coriolis (bug #3): the bias-acceleration X_b transport

The `nv == 2` gate is LIFTED — the analytic chain `J_state` is now machine-exact for serial
hinge chains of any length (validated 2/3/4-link). The multi-hop residual (a spatial 3-link's
unloaded `A` ~10% off; parallel exact) was a THIRD missing term, distinct from bug #1:

`mjd_rne_pos`'s Part B (Coriolis) reconstructs the bias acceleration with a recursion that
must match `mj_rne`'s. `mj_rne` uses `a_B[b] = X_b(a_B[parent]) + cvel[parent] ×_m (S·qvel)`
— i.e. **X_b motion transport** of the parent bias accel (`linear += α_parent × r`). Part B
used a **simple copy** (no transport), with a comment claiming the identity was exact. It is
exact only when the parent bias accel has no angular part — true for the root's children and
for any single hop, so ≤2-link agreed; but a ≥3-link non-parallel chain has a nonzero
`α_parent`, and the dropped `α_parent × r` made `Sᵀ·cfrc_B` diverge from `mj_rne`'s
`qfrc_bias` by ~26% at the operating point. The DERIVATIVE was self-consistent with this wrong
forward model — which is why every per-step check (Dcvel, Dcacc, Dcfrc-local for all bodies)
passed while the projected `qDeriv_pos` was off: **the analytic was correctly differentiating
a forward reconstruction that itself didn't match the real bias.**

Fix: Part B forward now does X_b transport (operating point: `α_parent × r`; derivative:
X_b-transport of the parent's `dcacc` + the transport derivative `α_parent × (axis × r)` for
ancestors — mirroring Part A and `mj_rne`). Backward stays simple-add (as `mj_rne` does — the
old comment conflated forward-transport with X_bᵀ-backward). Regression:
`analytical_transition_matches_fd_nonparallel_chain` now sweeps 2/3/4-link spatial chains.
Coupling gate widened to `nv ≥ 2`; end-to-end `threelink_chain_gradient_matches_fd` (n = 2).

### ★★ METHOD LESSON (the decisive bisection)
The forward/derivative split is the trap: when every per-step DERIVATIVE check passes but the
final result is off, **FD the forward OPERATING POINT itself** (`Sᵀ·cfrc_B` vs `mj_rne`'s
`qfrc_bias`) — a derivative is only as right as the forward model it differentiates. The
ladder Dcvel → Dcacc → Dcfrc-local → operating-point-identity localized it in ~5 FD probes.
Also: parallel-vs-spatial and 2-link-vs-3-link toggles isolated it to "intermediate joint
non-parallel WITH a descendant" before any code was touched.
