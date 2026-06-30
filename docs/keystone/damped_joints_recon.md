# Damped / actuated joints — recon + PR1

*Keystone deepening. Written 2026-06-15. Every prior keystone leaf built KINEMATIC
coverage (hinge → multi-link → ball → free base → off-COM moment → rotating normal).
This leaf begins the DYNAMIC coverage the powered exo needs: the coupled gradient
through joints with damping (and, as a follow-on, actuators). It removes a silent-wrong
gap — nothing panicked if you put `damping=` on a joint, the carry just used the bare
mass matrix and produced a quietly wrong gradient.*

## 1. The gap (precise)

Under the default **Euler** integrator (the keystone's), MuJoCo/sim-core's `eulerdamp`
treats joint **damping** implicitly: it solves `(M + Δt·D)·qacc = F` then
`qvel += Δt·qacc` (`integrate/mod.rs`, `forward/acceleration.rs`). So the contact
wrench reaches the next velocity through `M_impl = M + Δt·D`, where `D =
model.implicit_damping`. The coupling's carry used the **bare** `M`:

- `rigid_xfrc_column` → `G_vel = Δt·M⁻¹·Jᵀ` (the wrench→velocity response `G`).
- `analytic_state_jacobian` (single hinge) → the geometric-stiffness term `Δt·M⁻¹·∂(Jᵀw)/∂q`.

(Joint **stiffness** `K` is Euler-*explicit* — it does NOT enter `M_impl` under Euler;
only `ImplicitSpringDamper` uses `M + Δt·D + Δt²·K`. Joint *armature* is already in `M = qM`.)

The coupling's own free-platen `rigid_damping` knob (a contact-axis settling damping,
asserted 0 in the articulated path) is a SEPARATE thing — untouched by this leaf.

## 2. S0 spike (THROWAWAY) — gap CONFIRMED MATERIAL

`∂qvel'/∂xfrc` vs central FD of the real (eulerdamp) step, on a damped hinge + damped
2-link (`damping=0.5/0.3`):

```
                       Δt·(M+Δt·D)⁻¹·Jᵀ vs FD     bare Δt·M⁻¹·Jᵀ vs FD
damped hinge           4.2e-14  (machine-exact)    0.277  (28% wrong)
damped 2-link          1.3e-14                     9.93   (≈10× wrong)
```

The implicit factor is machine-exact; bare `M` is grossly wrong (worse for chains, where
the off-diagonal `M + Δt·D` coupling compounds). `implicit_damping` populates straight
from the MJCF `damping=` attr. **The gap is real; the fix is exactly `M → M + Δt·D`.**

## 3. PR1 — passive joint damping (the `M_impl` carry)

- `rigid_xfrc_column`: `M → M + Δt·D` (byte-identical when `D = 0`). This is `G_vel`,
  used by BOTH the FD and analytic `J_state` paths.
- The FD `loaded_state_jacobian` differentiates the real eulerdamp `step`, so it is
  ALREADY damping-correct — the multi-link / quaternion paths need no change.
- `analytic_state_jacobian` (single hinge): in PR1 this **declined under damping** (returned
  `None` → FD fallback), because the unloaded `A` from `transition_derivatives` forms its
  velocity rows with bare `M⁻¹` and so mismatched eulerdamp. **DONE (follow-on, 2026-06-17):**
  the damped single hinge now has an ANALYTIC `J_state` — the `M → M_impl = M + Δt·D`
  correction (rescale `A`'s bare-`M` velocity rows by `M/M_impl`, geom-stiff over `M_impl`,
  position rows from `θ' = θ + Δt·ω'`) makes it machine-exact (rel ~1e-10 vs FD). The 2-link
  CHAIN still uses the FD `J_state` (the analytic multi-link carry remains the open follow-on).

### Gates
- `rigid_multidof_response::damped_xfrc_column_matches_fd` — `G_vel` vs FD (damped hinge + 2-link).
- lib `analytic_state_jacobian_damped_matches_fd` — the damped analytic `J_state` vs FD (rel ~1e-10).
- `coupling_grad_harness::{damped-hinge,damped-2link}·material[μ]` rows — the coupled
  gradient vs the (damping-correct) full-coupled FD oracle, rel ~1e-6/1e-8;
  `damped_joint_gradient::damping_changes_the_gradient` — damped vs undamped `∂tip_z/∂μ`
  differ ≈40% (materiality; the one invariant the FD harness rows can't express).

## 4. Follow-ons
- **Analytic damped single-hinge `J_state`** — ✅ DONE (2026-06-17): the `M → M_impl` correction
  reconciles the unloaded `A` with eulerdamp, machine-exact (rel ~1e-10 vs FD), FD fallback
  dropped for the single hinge. See §3 above.
- **Analytic damped multi-link CHAIN `J_state`** — the 2-link+ damped chain still uses the FD
  `J_state`; the analytic carry (Jacobian Hessian + `∂M⁻¹/∂q`) for nv > 1 is the open follow-on.
- **Actuator dynamics** — state-dependent actuator forces (velocity/position actuators
  fold into the implicit damping; torque/general actuators carry control gradients) — the
  direct on-ramp to the powered exo.
- **Stiffness-implicit / non-Euler integrators** (`ImplicitSpringDamper` `M + Δt·D + Δt²·K`, RK4).
