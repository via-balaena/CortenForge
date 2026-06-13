# IPC contact — the real contact substrate — RECON

*Active recon, opened 2026-06-13. Replace the stepping-stone penalty contact
(`sim/L0/soft/src/contact/penalty.rs`) with **IPC** (Incremental Potential Contact,
Li et al. 2020) — the contact formulation the soft-body architecture committed to
from the start (`docs/studies/soft_body_architecture/src/40-contact/00-why-ipc.md`:
"IPC remains the destination"). This is the deepest software-executable foundation
move: the contact model sits beneath soft FEM, the soft↔rigid coupling, the
keystone gradient, and co-design — and it was knowingly laid as a placeholder.*

> **Add a C²-smooth divergent barrier energy as one more term in the Newton-on-
> potential solver, so contact becomes structurally non-penetrating AND smoothly
> differentiable in one mechanism — fixing the gradient degradation the keystone
> time-adjoint just measured, and removing the overlap that corrupts the
> per-element reward. Validate the barrier gradient through a contact make/break
> event against the exact gate the penalty contact fails.**

---

## 1. Why now — the architecture predicted the failure, the time-adjoint measured it

The soft-body architecture study (`40-contact/00-why-ipc.md`, claim 1) states the
thesis precisely: *"Penalty contact pops because its force is a discontinuous
function of the separation gap at the contact boundary; that same discontinuity
kills autograd (gradients are undefined at the boundary) and kills Newton's
convergence (the tangent jumps). IPC's C²-smooth barrier fixes the popping and the
non-differentiability with one mechanism."*

The keystone **time-adjoint leaf (PR #306, just merged) MEASURED exactly this**: the
composed multi-step coupled gradient is machine-exact while firmly engaged but
degrades to ~5–25% through marginal/bouncing contact, because the penalty force
`κ(d̂−sd)` is C⁰ at the active-set boundary `sd = d̂` (value continuous, derivative
kinks). That measured failure is now IPC's acceptance gate.

The architecture is also explicit (claim 2) that penalty is **structurally** wrong
for this stack regardless of smoothness: *"sim-soft cannot [tolerate overlap]
because the reward function reads contact pressure per element and overlap corrupts
the reward."* So the requirement is **zero penetration**, not merely a smoother
penalty (see §6 on why the existing quintic smoothing is not enough).

## 2. The barrier (the one new piece of math)

The IPC barrier (`40-contact/01-ipc-internals.md`):

```text
    b(d, d̂) = −(d − d̂)²·ln(d / d̂)      for 0 < d < d̂,    0 for d ≥ d̂
```

with the three load-bearing properties:
- **(a) diverges as d → 0⁺** → non-penetration enforced by infinite energy, not by a
  projection (the property penalty lacks);
- **(b) C² on (0, d̂]** → the Newton tangent and the autograd VJP are well-defined;
- **(c) `b(d̂) = b'(d̂) = b''(d̂) = 0`** → turning a pair on/off at the tolerance
  boundary introduces NO discontinuity in energy, force, OR tangent. **This is the
  exact property that kills the time-adjoint's kink** (penalty has b'(d̂⁻) = −κ ≠ 0).

Derivatives (analytic, for the gradient/Hessian assembly), with `r = d − d̂`:
```text
    b'(d)  = −2r·ln(d/d̂) − r²/d                         (= 2(d̂−d)ln(d/d̂) + (d̂−d)²/d)
    b''(d) = −2·ln(d/d̂) − 4r/d + r²/d²                  (→ b''(d̂)=0; → +∞ as d→0)
```
Contact energy added to the total potential: `E_contact(x) = κ·Σ_i b(d_i(x), d̂)`, so
the per-pair gradient is `κ·b'(d)·∂d/∂x` and the Hessian
`κ·(b''(d)·∂d/∂x⊗∂d/∂x + b'(d)·∂²d/∂x²)`. For a vertex-vs-rigid-**plane** pair the
distance is `d = sd` (signed distance), `∂d/∂x = n̂` (constant), `∂²d/∂x² = 0` — so the
Hessian is the rank-1 `κ·b''(d)·n̂⊗n̂` (the same shape as penalty's `κ·n̂⊗n̂`, with
`b''(d)` in place of the constant `1`). This is the keystone-coupling case.

## 3. How it slots in — the trait was designed for exactly this

The `ContactModel` trait (`contact/mod.rs`) already exposes `energy` / `gradient` /
`hessian` / `active_pairs` / `pose_residual_derivative`, and the solver assembles
contact generically: `active_pairs → gradient → residual` (`backward_euler.rs:1857`)
and `hessian → tangent A` (`:1958`). The architecture's claim 1 of Ch 01: *"once
contact is expressed as an added energy term, every piece of the solver below it
(Newton loop, sparse factorization, autograd VJP) inherits correctness
automatically."*

**Consequence — IPC is a drop-in new `ContactModel`, and the entire keystone
gradient stack works UNCHANGED:**
- `NewtonStepVjp` / `MaterialStepVjp` / `StateStepVjp` / `TrajectoryStepVjp` operate
  on the factored tangent `A`, which now carries the IPC barrier Hessian instead of
  penalty's — no change to the adjoint machinery.
- The only IPC-specific touches on the gradient PATH are the quantities that read the
  contact **force** directly (off the tangent): `pose_residual_derivative` (∂r/∂pose,
  S3) and the coupling's contact-force readout / `ContactForceTrajVjp`
  (∂fz/∂x*, ∂fz/∂pose). For penalty these used `−κ·n̂` / `κ·N_active`; for IPC they
  become `−κ·b'(d)·n̂` / the smooth `∂(κ b'(d))/∂pose` — and because `b'` is C¹ with
  `b'(d̂)=0`, these no longer kink at the boundary.

So the migration is: a new `IpcRigidContact` impl + swap it into the coupling +
update the two force-readout factors to use `b'`/`b''`. No rewrite of the solver or
the autograd.

## 4. The decisive question + the S0 spike

**Question:** does the IPC barrier gradient survive a contact make/break with the
smoothness the penalty lacks — i.e., does it close the time-adjoint's measured gate?

**S0 spike (throwaway):** implement `b`, `b'`, `b''` for the vertex-vs-plane case;
(1) FD-gate the barrier and its derivatives (and the assembled energy/gradient/
Hessian vs a re-solve, mirroring `material_sensitivity.rs`); (2) drop IPC into the
soft solver for the keystone block, and measure the single-step (and short
multi-step) `∂(outcome)/∂μ` vs full re-solve FD **as the plane sweeps THROUGH the
make/break boundary** — the exact sweep where penalty degraded. Success = the
analytic gradient tracks FD across the transition with **no boundary spike** (≪ the
penalty's 5–25%). Decisive and cheap; reuses the deeply-engaged `settle()` scaffold
from `soft_pose_sensitivity.rs` / `trajectory_step_vjp.rs`.

## 4a. VERDICT — S0 barrier spike done 2026-06-13 (throwaway, deleted). **MATH SOUND; C² FIX CONFIRMED.**

Throwaway `sim-soft/tests/zzz_ipc_barrier_spike.rs` (pure-function barrier, deleted):
- **`b'`, `b''` machine-exact vs central FD** (worst rel 6.5e-10 / 5.9e-10) — R1 retired.
- **C² at d̂ (no kink):** `b(d̂⁻)=b'(d̂⁻)=b''(d̂⁻)→0`; the force-slope `κ·b''(d)` vanishes
  ~linearly (`≈6(d̂−d)/d̂`) as d→d̂ — ratio to penalty's constant `κ` = **6.1e-2 →
  6.0e-3 → 6.0e-4** at d̂−{1e-4,1e-5,1e-6}. Penalty stays at `κ` then drops
  discontinuously to 0 at d̂ (the C⁰ kink the time-adjoint measured). **This is the
  exact property that closes the make/break gate.**
- **Diverges as d→0** (`|b'|=1e3`, b growing monotonically at d̂/1e5) — non-penetration.

So the barrier and its differentiability are confirmed; the remaining S0-level risk is
**R2 (does the divergent barrier converge in the Newton solve for the keystone
scene?)**, which needs the solver and is measured as the first step of PR1.

## 5. Ladder (sliced, each FD-gated; the established leaf pattern)

- **S0 spike:** barrier + derivatives FD-gated; gradient-through-make/break vs FD
  beats the penalty gate. (Measure first.)
- **PR1 (sim-soft):** `IpcRigidContact: ContactModel + ActivePairsFor` — barrier
  energy/gradient/Hessian + active set (`d < d̂`, no penetration so no interior
  cutoff hack) + `pose_residual_derivative` (the `b'` pose term). Gates: barrier
  C²/derivative FD; energy-grad-Hessian FD; a drop-and-rest non-penetration test
  (the property penalty cannot give — assert min distance stays `> 0`).
- **PR2 (sim-soft):** the contact-force readout / `per_pair_readout` for IPC (force =
  `−κ b'(d) n̂`), so the coupling and the keystone factors can consume it; re-point
  the keystone single-step + trajectory gradient factors to `b'`/`b''`.
- **PR3 (sim-coupling):** swap `PenaltyRigidContact → IpcRigidContact` in the
  coupling; **re-run the time-adjoint trajectory gate through make/break and show the
  degradation is gone** (the headline result — the gate IPC was built to beat).
- **Deferred (robustness, not needed for the small-dt keystone scene; follow-on
  leaves):** adaptive-κ schedule (`01-adaptive-kappa.md`), continuous collision
  detection / CCD (`02-ccd.md`) for large steps + tunneling, point-triangle /
  edge-edge primitives + self-contact (`03-self-contact.md`). v1 IPC = the
  vertex-vs-rigid-primitive barrier, the keystone case.

## 6. Why not just turn on the existing quintic smoothing?

`penalty.rs` already has a `smoothing_eps_m` quintic-Hermite ramp over
`(d̂, d̂+eps)` that is C² at the activation boundary — turning it on in the coupling
would likely shrink the time-adjoint's gradient kink cheaply. But it is **not** the
foundation move, for two reasons the architecture is explicit about: (1) smoothed
penalty still permits **interpenetration** (finite force, bounded by `κ·d̂`) — and
"overlap corrupts the [per-element] reward," the structural tripwire; (2) it does not
give the unconditional non-penetration / stability IPC's divergent barrier does. So
the smoothed penalty is at most a stop-gap gradient patch, not the contact substrate.
The S0 spike MAY measure smoothed-penalty as a baseline to quantify what
non-penetration vs smoothness each buy — but the deliverable is IPC.

## 7. Risks

- **R1 (low):** barrier derivative sign/algebra error → the S0 FD gate catches it
  immediately (independent re-solve).
- **R2 (medium):** the divergent barrier (`b''→∞` as d→0) can make the first Newton
  step diverge if the initial config is too deep / κ too large (the reason IPC has
  an adaptive-κ schedule). The keystone scene starts engaged-but-not-deep with small
  dt; if v1 hits convergence trouble, a fixed conservative κ + the existing LM
  regularization is the v1 mitigation, adaptive-κ the follow-on. Measure in S0.
- **R3 (low):** the keystone gradient factors assume penalty's constant Hessian
  curvature; re-pointing them to `b''(d)` is mechanical but must be FD-re-gated (the
  existing single-step + trajectory gates re-run under IPC are the check).
- **R4 (deferred):** no CCD in v1 → a fast enough body could tunnel through in one
  step. Out of scope for the small-dt keystone coupling; flagged for the robustness
  follow-on.

## 8. PR1–PR3 STATUS (2026-06-13)

- **PR1 ✅ MERGED-quality (committed `2da19b61`):** `IpcRigidContact` ContactModel
  (barrier b/b'/b''). Gates: barrier energy/grad/Hessian FD machine-exact (SPD);
  **R2 retired** — the divergent barrier CONVERGES over a 400-step loaded settle and
  NEVER penetrates (worst gap 6.3e-4 > 0); IPC pose vs re-solve FD rel 6.2e-10.
- **PR2 ✅ committed `a74040cf`:** the coupling's contact-force gradient factors use
  per-pair curvature `cᵥ = n̂ᵀ·H·n̂` (κ penalty / κ·b'' IPC). Behavior-preserving
  (all penalty gates unchanged); varying-cᵥ unit-tested.
- **PR3 ◑ PARTIAL (this commit):** `StaggeredCoupling<C = PenaltyRigidContact>` made
  generic over the contact via the local `PlaneContact` bridge trait — penalty
  default (all gates unchanged), IPC opt-in. **Single-step IPC coupled gradient is
  machine-exact** (rel 2e-9), forward rollout exact, non-penetration holds. The
  HEADLINE (multi-step make/break gradient clean) is **NOT yet fully delivered** —
  see §9.

## 9. OPEN — the multi-step IPC trajectory gradient residual (focused follow-up)

**Symptom.** The composed multi-step coupled trajectory gradient `dz_N/dμ` with IPC
matches the full-coupled FD oracle to only ~0.3–7% (κ-dependent, best ~0.3% at
κ≈3e3), NOT machine-clean — though far better than penalty's measured 5–25%.

**What's ruled out (measured):**
- Single-step coupled IPC gradient: machine-exact (rel 2e-9) at ALL engagement
  depths incl. marginal (sd≈0.0098). So the single-step factors are correct.
- Forward rollout: the tape's inlined forward == `step()` rollout to 0.0 (so the
  gradient is for the same trajectory the FD measures).
- FD oracle: converged across 5 decades of ε (so the residual is a real analytic
  error, not FD truncation).
- Floor / penetration: min gap 8–9e-3 ≫ floor; no penetration.

**Localization (the cold-start plan).** The residual is a **multi-step-only,
IPC-only** factor. The single-step path uses `run_crossing_tail` (material VJP +
`ContactForceVjp`, no pose/state parents); the multi-step path adds
`TrajectoryStepVjp` (4-parent: state + material + **pose**) and `ContactForceTrajVjp`
(the **z-parent** `∂fz/∂z = Σ κ·b''`). The sim-soft `trajectory_step_vjp.rs` gate
validated `TrajectoryStepVjp`'s four cotangents only with **penalty** contact; PR1's
pose check used the FORWARD `equilibrium_pose_sensitivity`, not the reverse
`TrajectoryStepVjp` pose cotangent with IPC in the tangent. **Next step:** re-run /
add an IPC variant of `sim-soft/tests/trajectory_step_vjp.rs` to check
`TrajectoryStepVjp`'s pose + state cotangents vs re-solve FD **with `IpcRigidContact`**
(varying b''). If those are clean → the bug is in the coupling chain
(`ContactForceTrajVjp` z-parent or the rigid carry); if off → it's the sim-soft fused
VJP under varying curvature. Repro: `cargo test -p sim-coupling --test
ipc_trajectory_gradient diag_ipc_multi_step_residual -- --ignored --nocapture`.
A secondary hypothesis: the scene is intrinsically **marginally engaged** (platen
weight balances the barrier near `d̂`, where b''→0 and b''' is large); a
firmly-engaged scene (smaller d̂ / heavier load) may be machine-clean — worth
constructing to separate "marginal regime" from "factor bug".

## 10. Scope & ritual

Per leaf: recon → S0 spike (measure, throwaway) → sliced PRs → n+1 cold-read →
pre-PR local ultra-review; no push without go-ahead; grade A per crate. v1 IPC =
the vertex-vs-rigid-primitive C² barrier (energy/gradient/Hessian/pose), drop-in for
penalty in the keystone coupling, validated to close the time-adjoint's measured
make/break gate and to enforce non-penetration. Sits beneath
[[project-keystone-soft-rigid-coupling]] and [[project-codesign-optimizer]]; lifts
the gradient-quality ceiling of the whole differentiable stack.
