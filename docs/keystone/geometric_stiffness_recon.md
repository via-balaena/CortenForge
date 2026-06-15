# Analytic geometric stiffness — recon + findings

*Keystone deepening. Written 2026-06-15. The queued leaf was: replace the
articulated coupling's **finite-differenced** loaded-state Jacobian
(`loaded_state_jacobian`) with the **analytic** geometric/load-stiffness term
`∂(Jᵀw)/∂q`, on the documented premise (from the multi-DOF #312 and contact-moment
#313 leaves) that this FD'd term IS the residual capping the articulated gradient at
~1e-3 over long rollouts — so making it analytic would make the gradient
machine-exact. The spike CONFIRMED the analytic term but FALSIFIED the premise. This
doc records the derivation, the measurements, and the re-diagnosis of the true cap.*

## 1. The analytic term (derivation)

The rigid `step` consumes `xfrc_applied[body]` as a spatial wrench `w = [τ; f]` at the
body COM, mapping it to generalized force `Jᵀw` (`mj_apply_ft`, `J = mj_jac_point` at
`xipos`, rows 0–2 angular, 3–5 linear). The LOADED single-step transition Jacobian
`J_state = ∂[qpos';qvel']/∂[qpos;qvel]` (wrench held) differs from the UNLOADED dense
`A` (`transition_derivatives`, which drops the applied wrench) by exactly the applied
wrench's **geometric/load stiffness**:

```
J_state = A + Δt·M⁻¹·∂(Jᵀw)/∂q   (velocity rows × qpos cols)
        + Δt·(that)              (position rows × qpos cols — semi-implicit qpos' = qpos + Δt·qvel')
```

For a **single hinge** (`nv = 1`, so `nq = nv`, raw `qpos` is a valid coordinate):

- `Jᵀw = â·τ + (â×r)·f`, with world axis `â`, moment arm `r = xipos − anchor`.
- As `θ` advances, the body rotates about `â`: `∂â/∂θ = 0` (the axis is the rotation
  axis) and `∂anchor/∂θ = 0` (the anchor is the fixed pivot), so only `r` rotates:
  `∂r/∂θ = â×r`, hence `∂(â×r)/∂θ = â×(â×r)`.
- The torque part drops (`∂â/∂θ = 0`). So

```
∂(Jᵀw)/∂θ = (â×(â×r))·f
```

`M` is configuration-independent for a single isolated hinge (the body inertia about
the fixed axis), so `∂M⁻¹/∂q = 0` and `Δt·M⁻¹·∂(Jᵀw)/∂q` is exact (no `∂M⁻¹/∂q` term).
The multi-link off-diagonal Jacobian Hessian (inter-joint Lie brackets) **and** the
`∂M⁻¹/∂q` term are the follow-on for the multi-link leaf.

Lives in sim-coupling (`StaggeredCoupling::analytic_state_jacobian`), composed from
`Data` (`xquat`, `xpos`, `xipos`, `qM`) + `transition_derivatives` for `A` — no
sim-core `∂J/∂q` machinery existed, and co-locating with the carry keeps the change
crate-local. A sim-core `mjd_`-style routine is the right home once multi-link
generalizes it.

## 2. Validation (the spike, machine-exact)

`analytic_state_jacobian_matches_fd_loaded` (lib unit test): for arbitrary held
wrenches (force AND torque components, nonzero `qvel`) the analytic `J_state` matches
the FD `loaded_state_jacobian` to **~5e-10 max relative error** (the FD floor) — incl.
the large geometric-stiffness entry `∂qvel'/∂qpos ≈ 40` which the analytic form
reproduces exactly while the FD carries noise. The free joint declines the analytic
path (`single_hinge` → `None`) and falls back to FD (raw `qpos` ≠ tangent for a
quaternion DOF — a pre-existing free-joint property the smoke tolerates).

## 3. The falsification (measure-first overturned the premise)

Threading the analytic `J_state` into `coupled_trajectory_material_gradient_articulated`
and FD-gating end-to-end (penalty, μ along λ = 4μ):

| horizon | FD `J_state` (old) | analytic `J_state` (new) |
|---------|--------------------|--------------------------|
| n = 6   | 1.5e-6             | **7.3e-7**               |
| n = 10  | 1.04e-3            | 1.05e-3 (unchanged)      |
| n = 12  | 4.67e-3            | 4.67e-3 (unchanged)      |

The analytic term **halves the n = 6 error but does nothing at n ≥ 10** — because the
analytic and FD `J_state` agree to 5e-10, so `J_state` precision was never the n ≥ 10
cap. The FD oracle is fully converged at n = 10 (identical from eps_rel 1e-3 to 1e-7),
so the 1e-3 is a genuine TAPE error, and n = 2 is machine-exact — i.e. it is a
higher-order term invisible at n = 2 and accruing with re-engagement, **not** an
attribution skew and **not** the geometric stiffness.

## 4. Re-diagnosis of the true cap (the off-COM moment's long-rollout gradient)

**Discriminator (decisive):** the SAME articulated path on a **free-joint platen** —
no off-COM moment (symmetric contact through the COM), `J = I` so no geometric
stiffness, FD `J_state` — is **machine-exact at every n** (1.3e-9 at n = 6, 3.7e-10 at
n = 10, 2.4e-10 at n = 12; `articulated_free_platen_exact_all_n`). So the shared
multi-DOF carry, the soft adjoint, and the §8a structure are all sound. **The residual
is the off-COM MOMENT's gradient over long rollouts.**

**It is smooth, not an active-set kink:** IPC reaches the same ~1e-3 order at n = 10
(7e-5 at n = 6, 1.4e-3 at n = 10, 4.4e-3 at n = 12) — it does NOT reduce it.

**Every standard structural fix was measured WORSE** (the stale-to-`s_k`, §8a-zeroed
calibration is optimal):

| experiment | n = 6 | n = 10 | n = 12 |
|------------|-------|--------|--------|
| baseline (analytic J_state) | 7.3e-7 | 1.05e-3 | 4.67e-3 |
| fresh-FK re-forward (tape + oracle) | 20% | 15% | 29% |
| full lag-attribution (pose seam + wrench-c + final objective → s_{i−1}) | 21% | 15% | 33% |
| wrench-c-only lag-attribution | 1.1e-6 | 1.13e-3 | — |
| true position-row wrench term (`∂qpos'/∂w = Δt·G_vel`) | 21% | 17% | 51% |

So both attributions in the prior code/memory are FALSE: the n ≥ 10 residual is NOT
"the FD'd geometric stiffness" (#312/#313 docstrings + this leaf's premise) and NOT
"the FD'd `∂(Jᵀτ)/∂q`" (the contact-moment leaf's secondary claim). The precise source
is the off-COM moment's long-rollout gradient — an open problem (see §6).

## 5. What shipped

- `analytic_state_jacobian` + `single_hinge` (sim-coupling): the machine-exact analytic
  `J_state` for the single hinge; the consumer uses it with the FD `loaded_state_jacobian`
  as the fallback (free joint, multi-link). Removes the FD approximation (deterministic,
  no eps) and is the clean base for the multi-link leaf.
- The n = 6 make/break gate tightened 1e-5 → 3e-6 (the analytic enables it; a float-noise
  margin over the measured 7.3e-7).
- `analytic_state_jacobian_matches_fd_loaded` (analytic ≡ FD loaded) +
  `articulated_free_platen_exact_all_n` (the discriminator) as permanent regression tests.
- All falsified attributions corrected across gate comments + docstrings.

## 6. The bookmark (next leaf)

> **★ RESOLVED 2026-06-15.** The "real machine-exact cap" below was FIXED (it was not
> architectural). The fully-fresh formulation (fresh-FK contact pose + fresh output +
> true position-row carry `∂qpos'/∂w = Δt·G_vel`) is machine-exact for single-hinge,
> free-joint, AND 2-link chain at every n. The earlier "every fix is worse" finding was
> from testing the matched-pair corrections individually. See
> `moment_residual_recon.md` §3f and `multilink_recon.md`.

The off-COM moment's long-rollout gradient is the real machine-exact cap for the
articulated-with-moment path. It is smooth, moment-specific, contact-model-independent,
exact through n ≈ 6, and resistant to fresh-FK / lag-attribution / the true position-row
term. A focused leaf should isolate it further — e.g. per-step adjoint vs a
per-step-truncated FD to localize WHICH factor's linearization drifts (the moment's
`∂w/∂x*` explicit-`rᵢ` vs via-`gᵢ` parts, or `∂w/∂h`, over the changing active set), and
whether the soft solve's contact-Jacobian linearization interacts with the moment arm
over re-engagement. Adequate for co-design in the meantime (DIRECTION + ~99.9%
magnitude).
