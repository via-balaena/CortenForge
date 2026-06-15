# Rotating contact normal — recon + S0

*Keystone deepening. Written 2026-06-15. The differentiable soft↔rigid coupling presents
the rigid body to the soft block as a **flat horizontal plane** (normal hardcoded
`(0,0,−1)`, posed at the body COM height). The just-completed quaternion leaf made the body
ORIENTATION differentiable (ball + free), but that orientation reaches the contact ONLY
through the COM height and the off-COM moment — the contact NORMAL still ignores it. This
leaf makes the normal track the body orientation (`n̂ = R·(0,0,−1)`), so a tilted/rotating
device presents a tilted contact face — and makes that differentiable. It is the natural
completion of the S3 soft-pose-sensitivity work, which explicitly deferred this
(`s3_soft_pose_sensitivity_recon.md`: "constant-normal (plane) ... `∂n̂/∂δ = 0`").*

## 1. The gap (precise)

`StaggeredCoupling::build_contact` (`lib.rs:914`) builds `RigidPlane::new((0,0,−1), −height)`
every step — a fixed downward normal at `height = xipos.z − clearance`. So the contact force
`g_v = (dE/dsd)·n̂` is **always vertical**, regardless of body tilt. For a tilted device this
is first-order wrong: the real contact face tilts with the body, redirecting the force
(tangential component ~`sin θ`) and adding a twist `τ_z` the flat model drops.

The gradient is self-consistent with this (wrong) forward: the pose seam (`PoseSeamVjp`)
threads ONLY the scalar height `h(q)` with `∂h/∂q = J_z`; the normal is frozen (`∂n̂/∂q = 0`).
And the soft solve's pose-adjoint (`equilibrium_pose_sensitivity`,
`backward_euler.rs:2025`) is scoped to a plane TRANSLATION along `dir` with `∂n̂/∂δ = 0`.

## 2. S0 spike (THROWAWAY, measure-first) — gap CONFIRMED MATERIAL

Env-gated rotating normal in `build_contact` (`n̂ = R·(0,0,−1)`, `offset = xipos·n̂ +
clearance`, reduces to the fixed model at `R = I`); compared a tilted ball-joint forward
rollout fixed-vs-rotating. (Spike reverted; lib/tests byte-clean.)

```
ball, tilt θ=0.1 about Y          fixed normal        rotating normal
contact force_on_soft             [0, 0, −1391]       [−0.1, 2.0, −28]   (|horiz|/|f|: 0 → 0.072)
tip_z @ n=4                       0.1384              0.1127   (~18% lower)
tip_z @ n=6                       0.1962              0.1238   (~37% lower)
```

- The fixed model's force is **purely vertical by construction** (`|horiz|/|f| = 0` at ANY
  tilt) — it structurally cannot produce a tangential force or a tilted normal.
- The rotating normal redirects ~`sin θ` of the force tangentially → the rollout diverges
  ~18% (n=4) / ~37% (n=6), and is **more physically faithful AND more stable** (the body
  slides along the tilt instead of launching straight up).
- At θ=0.3 the rotating normal even DESTABILIZES the flat-tuned ball scene (the contact
  geometry genuinely differs) — gates must use a gentle tilt (θ≈0.1) where both are stable.

**Conclusion: the gap is material and qualitative. The leaf is justified.**

## 3. Design

**Plane parameterization** (reduces to the current model at `R = I`):
```
n̂(q)     = R(q)·(0,0,−1)                    (body's contact-face outward normal)
offset(q) = xipos(q)·n̂(q) + clearance
sd_v      = p_v·n̂ − offset = (p_v − xipos)·n̂ − clearance
```

**The normal's config Jacobian (reuses machinery already computed):**
```
∂n̂/∂q   = −[n̂]× · J_ang        (3×nv)   J_ang = mj_jac_point rows 0–2 (the angular Jacobian)
```
[a body world angular velocity `ω = J_ang·δqvel` rotates the normal by `δn̂ = ω×n̂ = −[n̂]×ω`.]
`mj_jac_point` already gives the angular rows (we use rows 3–5 for `jlin`, row 5 for `jz`) —
so `∂n̂/∂q` is cheap. `∂offset/∂q = n̂ᵀ·J_lin + xiposᵀ·(∂n̂/∂q)`.

**Soft contact-residual pose derivative (the S3 deferral, sim-soft):** per active pair the
residual gains the normal-rotation terms the current translation-only form drops:
```
∂g_v/∂pose = d²E/dsd² · (∂sd_v/∂pose) · n̂  +  (dE/dsd) · (∂n̂/∂pose)
             └ magnitude (sd shifts) ┘          └ NEW: direction (n̂ rotates) ┘
∂sd_v/∂pose = p_v·(∂n̂/∂pose) − ∂offset/∂pose     (also gains the p_v·∂n̂ part)
```

**Wrench dependence (sim-coupling `ContactWrenchTrajVjp`):** the routed wrench
`[τ; f] = Σ(...)·n̂` gains `∂f/∂n̂` and `∂τ/∂n̂` terms — currently the normal is a frozen
per-pair constant.

## 4. Scope — a CROSS-CRATE leaf (bigger than the quaternion leaf)

Unlike the quaternion leaf (which reused `nv`-general machinery), this genuinely extends the
contact model AND the soft pose-adjoint:

- **sim-soft** (the S3 deferral): `ContactModel::pose_residual_derivative` (penalty + IPC,
  `contact/mod.rs:175`, `penalty.rs:836`) and `assemble_pose_residual_grad` /
  `equilibrium_pose_sensitivity` must accept a general rigid-pose perturbation (translation +
  rotation, `∂n̂/∂δ ≠ 0`), not a scalar translation along `dir`. FD-validated against a
  re-solve (perturb the plane NORMAL, re-solve, central-difference) in
  `tests/soft_pose_sensitivity.rs`.
- **sim-coupling**: `build_contact` → rotating `(n̂, offset)`; the pose seam generalizes from
  the scalar `h(q)` (`PoseSeamVjp`, `J_z`) to the plane `(n̂(q), offset(q))` with their
  q-Jacobians; the contact force/wrench VJPs (`ContactForceTrajVjp`, `ContactWrenchTrajVjp`)
  add the `∂·/∂n̂` terms.
- `RigidPlane` ALREADY stores an arbitrary normal — no change to the type itself.

## 5. Sliced plan (measure-first each)

1. **PR1 (sim-soft) — the rotating-normal pose-residual derivative.** Extend
   `pose_residual_derivative` to the general pose (add the `∂n̂/∂δ` direction term + the
   `p_v·∂n̂` sd part). FD-validate `∂x*/∂(normal rotation)` against a re-solve. Self-contained,
   foundation-first (get the soft adjoint right before wiring). S0: confirm the `∂n̂/∂δ` term
   closed form vs FD.
2. **PR2 (sim-coupling) — wire the rotating normal into the coupled gradient.** `build_contact`
   rotating plane; the generalized `(n̂(q), offset(q))` seam (threading `∂n̂/∂q = −[n̂]×·J_ang`,
   `∂offset/∂q`); the wrench/force VJP `∂·/∂n̂` terms. Gate a gentle-tilt (θ≈0.1) ball/free
   scene machine-exact vs the full-coupled FD oracle (which now uses the rotating normal). S0:
   confirm `∂n̂/∂q` closed form vs FD before wiring.
3. **Follow-on (separate arc)** — tangential FRICTION (Coulomb/regularized stick-slip; an
   active-set complication, the heavier physics) and curved/posed-SDF contact.

## 6. Watch (carry forward)
- ★ **STABILITY:** the rotating normal destabilizes flat-tuned scenes at large tilt (θ=0.3
  ball diverged) — gates use θ≈0.1; the rotating normal is itself MORE stable than the flat
  model at a given tilt (tangential redirect vs vertical launch).
- ★ **Convention** (from the quaternion leaf): tangent quantities are body-frame; `mj_jac_point`
  angular rows are the world angular Jacobian — keep `∂n̂/∂q = −[n̂]×·J_ang` consistent with
  the seam's tangent convention (FD-validate, do not trust the sign blind).
- The active set shifts under a tilted plane (different vertices engage) — gate in the
  stable-active-set regime; IPC is the smooth cure (penalty active-set boundary is non-smooth).
- The forward + gradient must change together (a matched pair, like the fully-fresh
  formulation) — changing `build_contact` alone breaks the existing gradient gates.
- This COMPLETES `s3_soft_pose_sensitivity_recon.md`'s deferred `∂n̂/∂δ` term.
