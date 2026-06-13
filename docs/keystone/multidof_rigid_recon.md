# Multi-DOF rigid coupling — recon

*Keystone deepening. Written 2026-06-13. Grounds the leaf that removes the single
most load-bearing cap every coupling gradient carries: the rigid side is a
**free-body platen** where `∂vz'/∂fz = dt/m` is a SCALAR.*

## 1. The gap (one sentence)

Every merged coupling gradient — `coupled_trajectory_{material,control,policy,joint}_gradient`,
`RigidStepVjp`, the carry VJPs — models the rigid body as a **single free DOF**:
the contact reaction is a pure vertical force at the COM, the velocity response is
the scalar `∂vz'/∂fz = dt/m`, and the body pose that poses the contact plane is a
scalar height `z`. For an **articulated** mechanism (the capstone is a multi-joint
exo) a contact reaction at a point produces a *generalized* force coupled across
joints, the velocity response is the matrix `dt·M⁻¹·Jᵀ` (≠ `dt/m`), and the contact
geometry is posed by the body's full `(xpos, xquat)`.

## 2. The two engines (unchanged from the keystone recon)

- **Rigid = `sim-core`** (`sim/L0/core`): MuJoCo-aligned, differentiable ONLY as
  dense `TransitionMatrices{A,B,C,D}` (`derivatives/`, FD + analytic-velocity
  hybrid). **NO reverse-mode tape.** `A = ∂x_{t+1}/∂x_t` (state, `2nv+na` square),
  `B = ∂x_{t+1}/∂u_t` (control only).
- **Soft = `sim-soft`**: real reverse-mode IFT adjoint on the chassis `Tape`. The
  coupling builds ONE tape and wraps the rigid factors as chassis `VjpOp`s so a
  single `tape.backward` crosses both engines.

## 3. The crux — the xfrc column — RESOLVED by reading the engine

S2 recorded the open question: *"∂s'/∂xfrc MISSING — rigid B is ctrl-only, xfrc not
in B + auto-zeroed each step; needs FD or a new column-builder."* Reading
`forward/acceleration.rs` + `jacobian/mod.rs` resolves it analytically:

- `xfrc_applied[body]` (a `SpatialVector` `[τ(3); f(3)]`) is projected into the
  generalized force each `forward` via `mj_apply_ft(... point = xipos[body] ...)`
  → it contributes `Jᵀ·[τ; f]` to `scratch_force`/`qfrc`, where `J` is the body
  **COM** spatial Jacobian (`mj_jac_point` at `xipos`, rows 0–2 angular / 3–5
  linear).
- The (semi-implicit Euler) velocity update is `qvel' = qvel + dt·M⁻¹·qfrc`, so

  ```
  ∂qvel'/∂xfrc_applied[body]  =  dt · M⁻¹ · J_comᵀ        (nv × 6)
  ```

  This is the exact matrix-valued successor to the scalar `dt/m`. For the free
  platen `J_comᵀ` maps the linear-z column to `qvel[2]` with coefficient `dt/m`
  and the matrix collapses to the scalar — the merged path is the 1×1 special
  case. (With implicit joint stiffness/damping the factor is `dt·M_impl⁻¹·Jᵀ`;
  the keystone scenes have neither, so plain `M⁻¹`. **MEASURE** whether to read
  `M⁻¹` analytically or FD the column.)

- **Position row.** Semi-implicit Euler integrates `qpos` with the step's STARTING
  velocity, so the freshly-applied force does NOT reach `qpos` this step:
  `∂qpos'/∂xfrc ≈ 0` (the scalar case found `∂z'/∂fz = 0` exactly — the #307
  off-by-one). At matrix scale this means the position carry reads only the
  PRE-update velocity, exactly as `ZCarryVjp` already does. **MEASURE** it
  generalizes (it should — same integrator).

**⇒ Decision 1 is decided before the spike: the column is buildable.** Two routes,
both gateable: (a) analytic `dt·M⁻¹·Jᵀ` (the understanding + an oracle), (b) direct
FD over a scratch step — the generalization of the existing `rigid_step_probe`
(perturb each of the 6 `xfrc` components, read `qvel'`). The spike measures which is
cleaner/more robust; the lean is **FD the column** (reuses the proven scratch-`Data`
pattern, captures any implicit-integration correction, no `M⁻¹` plumbing) and use
the analytic form as the FD oracle.

## 4. The minimal scenes (decision 2)

The smallest case where `dt/m` is WRONG: a **single hinge** (`n_link_pendulum(1, L, m)`
— a link rotating about Y, point mass at the tip). A force at the tip maps to joint
torque through the moment arm `L`, so the effective inertia at the tip is
`(J M⁻¹ Jᵀ)⁻¹ = I_joint/L² ≠ m`. Then a **2-link** (`n_link_pendulum(2, …)`) for the
genuine off-diagonal coupling (a force on link-2's tip accelerates joint 1 too).

## 5. The contact moment (decision 3)

`mj_apply_ft` applies the `xfrc` force at the body **COM**. The merged coupling
routes a pure force at the COM — exact only for the symmetric platen where the
contact resultant passes through the COM. An off-COM/articulated contact at point
`r_c` must route the **full spatial force**: `xfrc.force = f`,
`xfrc.torque = (r_c − r_com) × f`. The moment is what drives the joints; omitting it
is wrong for the hinge. The spike confirms it is needed and threads it.

## 6. The pose seam (decision 6)

Today the contact plane is posed by the scalar `plane_height = xpos[body].z −
clearance` (`∂plane/∂z = 1`, constant normal). For an articulated body the contact
SDF is posed by the full `(xpos, xquat)`; the S3 soft pose-residual-derivative
(`equilibrium_pose_sensitivity`, today a plane translation along a fixed `dir`)
must compose with the body's spatial Jacobian `∂(xpos,xquat)/∂q = J`. The spike
measures whether the existing pose-sensitivity composes or needs a normal-rotation
term (S3 was constant-normal-plane scope).

## 7. Where it lives (decision 5 — head-eng call)

**Keep the platen path byte-untouched; add the multi-DOF path alongside.** The
generic `StaggeredCoupling<C: PlaneContact>` already carries `model`/`data`/`body`;
the generalization is (a) a `rigid_state_response` returning the full
`(∂state'/∂xfrc, A)` instead of the scalar `dt/m`, (b) a generalized state-carry
`VjpOp` (the dense `A` + the `nv×6` xfrc column) replacing the scalar
`Vz/ZCarryVjp` for the multi-DOF case, (c) the contact-point spatial-force routing.
The 1-DOF scalar VJPs and their gates stay as the special-case fast path (merged
gates GREEN). Mirror the IPC `StaggeredCoupling<C=Penalty>` default-generic
discipline. Settle the exact struct/trait shape after the spike.

## 8. Ladder

- **S0** (this session, THROWAWAY) — measure: (M1) hinge `∂qvel'/∂xfrc` analytic
  `dt·M⁻¹·Jᵀ` vs FD, confirm ≠ `dt/m` and the moment matters; (M2) 2-link
  off-diagonal; (M3) state-carry `A` vs `transition_derivatives` + the
  semi-implicit `∂qpos'/∂xfrc ≈ 0`; (M4 if cheap) a tiny coupled hinge-tip-on-block
  `∂(tip height after N steps)/∂μ` vs full-coupled FD — proves the gradient crosses
  both engines on a multi-DOF body. Delete the crate.
- **PR1** (sim-coupling) — the rigid multi-DOF primitive: a matrix `RigidStateVjp`
  (state' = `A·state + G·xfrc`, `G` the xfrc column) + a `rigid_state_response`
  probe, FD-gated on hinge + 2-link IN ISOLATION (the analogue of S2's rigid
  factor, no soft contact yet).
- **PR2** (sim-coupling) — the coupled multi-DOF scene: contact-point spatial-force
  routing + the pose seam, one `tape.backward` over an N-step hinge-on-block
  rollout `dJ/dμ` vs full-coupled FD oracle. Generalizes
  `coupled_trajectory_material_gradient` to the articulated body.

## 8a. S0 spike findings (2026-06-13, throwaway — measured, then deleted)

Throwaway `tests/zzz_multidof_spike.rs` (sim-coupling), gated vs independent FD.

- **M1 — the crux, RESOLVED machine-exact.** Single hinge (`n_link_pendulum(1)`,
  tilted 0.3 rad): the analytic xfrc column `dt·M⁻¹·J_comᵀ` matches FD over a real
  scratch step to **rel 6.7e-12**. The column is genuinely multi-component
  (nonzero τ_y, f_x, f_z entries for the Y-hinge — the moment-arm coupling) — the
  scalar `dt/m` is wrong on two counts: wrong `f_z` coefficient AND it drops the
  off-axis entries. **⇒ the xfrc column is analytically buildable and exact.
  Decision 1 is settled: no new column-builder research needed; the lean is the
  analytic `dt·M⁻¹·Jᵀ` (FD-gateable), or FD the column (the proven scratch-`Data`
  pattern). Either is machine-exact.**
- **M2 — off-diagonal, machine-exact.** 2-link: analytic vs FD **rel 6.5e-11**; a
  pure +z force on the distal link accelerates BOTH joints (`Δqvel = [−2.4e-2,
  +5.5e-2]`) — the genuine multi-DOF coupling the scalar drops.
- **M3 — state carry.** `transition_derivatives` gives the dense `A` (2nv×2nv) with
  real off-diagonal coupling directly; `B` is `nv×0` (confirms xfrc is NOT in `B` —
  the S2 cap stands, the column must be built separately).
- **M0 + the composition subtlety (the PR2 risk, flagged).** Direct single-step
  probe on the free platen: `z_new = z_old + Δt·vz_NEW` to **machine zero**
  (`vz_OLD` is off by Δt²·qacc) — so the bare integrator uses the UPDATED velocity,
  and `∂z'/∂fz = Δt²/m` (≠ 0). **But** the merged scalar `coupled_trajectory_material_gradient`
  wires the position carry to the PRE-update `vz_var` (`∂z'/∂fz = 0`), and the
  decisive experiment shows that wiring is the FD-correct one: flipping `ZCarryVjp`'s
  velocity parent `vz_var → vz_next_var` (the "M0-correct" structure) makes the
  merged gate FAIL at 9.5% rel; the merged `vz_var` wiring passes at 3e-8. Mapping
  small N: **n=1 gives EXACTLY 0 for both tape and FD** (the first soft step starts
  from rest → zero internal stress → `dfz/dμ ≈ 0`, so `dz_1/dμ = 0` is genuinely
  true, not a wiring artifact); n≥2 the `vz_var` tape matches FD to ~1e-9.
  **Lesson for PR2: the staggered-coupling position-carry timing does NOT follow
  naively from the bare integrator's `z = z + Δt·vz_new` — the contact-force/height
  staggering interacts with it. The composed multi-step carry MUST be FD-gated
  against a re-rolled full-coupled oracle (the keystone discipline), NOT derived
  from the bare-step linearization. Replicate the merged scalar structure
  (position carry on the pre-update velocity var; contact force a separate parent
  of the velocity node) generalized to vectors, and gate it — do not assume the
  dense `A`'s position rows compose correctly in the staggered loop until measured.**

**Verdict:** the rigid multi-DOF primitive (`A` + the `dt·M⁻¹·Jᵀ` column) is
machine-exact and ready (PR1, isolated, hinge/2-link FD gate). The coupled-scene
composition (contact moment + pose seam + carry timing) is the higher-risk follow-on
(PR2), to be FD-gated end-to-end.

## 8b. Slicing decision (head-eng call)

- **PR1 — the rigid multi-DOF primitive, IN ISOLATION.** A matrix-valued
  `RigidStateVjp` (state' = `A·state + G·xfrc`, `G = dt·M⁻¹·Jᵀ` the xfrc column) +
  a `rigid_state_response` probe (the generalization of `rigid_vz_response`),
  FD-gated on a hinge and a 2-link IN ISOLATION (no soft contact). This is the
  literal "generalize `dt/m` → `dt·M⁻¹·Jᵀ`" deliverable, machine-exact (M1/M2),
  low-risk — the analogue of the merged S2/S4 isolated rigid factor. Ships the
  reusable primitive every future articulated coupling consumes.
- **PR2 — the coupled articulated scene.** A hinge-tip-on-block scene: contact-point
  spatial-force routing (the moment `r×f`, decision 3), the moving-plane pose seam
  (S3 ∘ body Jacobian, decision 6), and the multi-DOF carry composed into
  `coupled_trajectory_*_gradient`, FD-gated end-to-end against a re-rolled
  full-coupled oracle (resolves the 8a composition-timing risk empirically).

## 9. Discipline (carry forward)

- FD-gate every new gradient against an INDEPENDENT re-rolled coupled-FD oracle
  (re-run the REAL sim at param±ε; never an affine identity).
- `Data` is NOT Clone → scratch rebuild (`make_data` + copy `qpos`/`qvel` +
  `forward`) for off-rollout probes. Trajectory methods are `&mut self`.
- KEEP merged platen paths byte-untouched; multi-DOF alongside.
- Pre-commit: whole-tree `cargo fmt` + conventional commits. Run the FULL
  `cargo xtask grade <crate>` (all 8 tiers — Documentation bites private intra-doc
  links) before push.
