# Contact moment — recon

*Keystone deepening. Written 2026-06-14. Grounds the leaf that removes the
pure-force-at-COM approximation BOTH the platen and the just-merged articulated
paths carry: the soft contact reaction is routed as a pure force at the rigid
body COM, dropping the contact moment `Σ rᵢ × fᵢ`. This is the natural completion
of the multi-DOF wrench routing (PR #312) — the rigid carry already CONSUMES a
6-vector wrench (`rigid_xfrc_column` is `nv × 6`); the gap is COMPUTING the moment
on the contact side and threading its gradient — and the first step of the
curved/distributed device-on-tissue contact arc the capstone needs.*

## 1. The gap (one sentence)

In `coupled_trajectory_articulated_z` (the forward oracle) and
`coupled_trajectory_material_gradient_articulated` (the tape), the contact
reaction is routed as `xfrc_applied[body] = [0,0,0; −Σfₓ, −Σf_y, −Σf_z]` — a
**pure force at the body COM**, torque rows (0–2) ZEROED — so the off-COM moment
`τ = Σ (rᵢ − c) × (−gᵢ)` (about the COM `c = xipos`, with per-pair soft-force
`gᵢ = force_on_softᵢ` at world point `rᵢ`) is dropped. The omission is currently
*consistent* between oracle and gradient (so the merged FD gate passes — the
gradient is exact for THAT model), but the forward model is **first-order wrong**
whenever the contact resultant misses the COM.

## 2. Why it bites in the existing scene (the moment is already large)

The merged articulated gate (`tests/articulated_trajectory_gradient.rs`) is a
**Y-hinge**: pivot at `(0,0,0.2)`, point mass at body-local `(0,0,−0.095)`,
started tilted `qpos = 0.3`. The soft block is `uniform_block(4, 0.1)` spanning
`x,y ∈ [0, 0.1]`, top face `z = 0.1`. The contact plane is an **infinite
horizontal** penalty plane (normal `−ẑ`) at `tip_z − clearance`, so the active set
is the WHOLE top face (all 25 top vertices), and the contact-force resultant acts
through the **block-top centroid `≈ (0.05, 0.05, 0.1)`**.

But the tilted tip COM (`xipos`) is at `x ≈ −0.028` (rotating `(0,0,−0.095)` about
`+Y` by `0.3`). So the resultant misses the COM in `x` by `≈ 0.078 m` → a real
`τ_y` about the hinge axis of magnitude `≈ 0.078 · |Σf_z|`. Crucially, mapped to
the hinge torque (`Jᵀ` about the pivot), routing the resultant **at the COM**
(`x ≈ −0.028`) vs **at the true contact centroid** (`x ≈ +0.05`) gives
**opposite-sign** hinge torque — the pure-force-at-COM model is not just inaccurate
but qualitatively wrong for this mechanism. So no new scene is needed; the existing
hinge is a strong, non-degenerate demonstrator. (Decision #2 — the spike confirms
the moment changes the forward rollout and the gradient materially.)

The platen scene (free joint) is symmetric — the resultant passes through the COM
— so its moment is `~0` and its (untouched) gradient stays machine-exact
(decision #4).

## 3. The math

### 3.1 The wrench (forward)

The reaction equivalent wrench at the COM `c = xipos[body]`:

```text
w_f = −Σ gᵢ                       (force, rows 3–5)
w_τ = −Σ (rᵢ − c) × gᵢ           (moment about c, rows 0–2)
```

with `gᵢ = force_on_softᵢ` (3-vector, read from the per-pair readout `.force_on_soft`)
and `rᵢ = positionᵢ` (the contacted soft vertex's world position, readout `.position`,
= `x*[vᵢ]`). For the flat constant-normal plane `gᵢ = (0,0,g_z)` (`g_z < 0`), so
`w_f = (0,0,−Σg_z)` (upward) and `w_τ = (−Σ(ry−cy)g_z, Σ(rx−cx)g_z, 0)` — only
`τ_x, τ_y` nonzero, `τ_z = 0`. The Y-hinge axis sees `τ_y`. (Tangential/`τ_z`
contributions arrive with curved/frictional contact — the NEXT arc.)

`xfrc_applied` is interpreted by the integrator as a wrench `[τ; f]` **at the body
COM `xipos`** (the same point `rigid_xfrc_column` builds `J` at — that's why the
merged multi-DOF carry validated machine-exact). So routing `[w_τ; w_f]` is exactly
"the net contact wrench about the COM" — the faithful forward model.

### 3.2 ∂w/∂x* (the gradient's contact half — decision #1's two parts)

Per active pair `i` with vertex `v` (`rᵢ = x*[3v..3v+3]`), curvature
`cᵥ = d²E/dsd²` and outward normal `n̂`, the soft-force Jacobian is
`∂gᵢ/∂x_v = −cᵥ n̂⊗n̂` (the S3 factor; `scatter_dfz_dxstar` is its z-row). Then the
6×3 block scattered into columns `3v`:

```text
force rows (3–5):  ∂w_f/∂x_v = −∂gᵢ/∂x_v = +cᵥ n̂⊗n̂
torque rows (0–2): ∂w_τ/∂x_v = [gᵢ]_×  +  cᵥ [rᵢ−c]_× (n̂⊗n̂)
                                └ explicit rᵢ ┘   └──── via gᵢ ────┘
```

derived from `∂(a×b)/∂a = −[b]_×`, `∂(a×b)/∂b = [a]_×` with `a = rᵢ−c`, `b = gᵢ`:

- explicit `rᵢ`-part: `∂w_τ/∂rᵢ = −(−[gᵢ]_×) = [gᵢ]_×`;
- `gᵢ`-part: `∂w_τ/∂gᵢ · ∂gᵢ/∂x_v = −[rᵢ−c]_× · (−cᵥ n̂⊗n̂) = cᵥ [rᵢ−c]_× n̂⊗n̂`.

The force rows generalize the existing scalar `∂fz/∂x*` from the z-row to the full
3×3; the torque rows are the new machinery. `[a]_×` is the skew matrix
`[[0,−az,ay],[az,0,−ax],[−ay,ax,0]]`.

### 3.3 ∂w/∂s (the moment's rigid-state feedback — decision #3)

`w_τ` also depends on the rigid state `s = [qpos; qvel]` through `c = xipos(q)`:

```text
∂w_τ/∂c = [w_f]_×            (from ∂[(rᵢ−c)×gᵢ]/∂c = [gᵢ]_×, summed and negated)
∂c/∂qpos = J_lin            (the COM linear Jacobian, mj_jac_point(xipos) rows 3–5)
⇒ ∂w_τ/∂qpos = [w_f]_× · J_lin   (3×nv);  ∂w/∂qvel = 0;  ∂w_f/∂s = 0
```

This is a NEW feedback edge `s → w → s'` the merged path lacks. It does NOT
double-count the loaded Jacobian `J_state`: `J_state` holds the wrench CONSTANT
while perturbing `q` (it captures "same wrench, moved config" — the geometric
stiffness `∂(Jᵀw)/∂q`), whereas `∂w/∂s` captures "config moved the wrench value via
`c(q)`". Orthogonal terms. **The spike measures whether `∂w/∂s` is material or
second-order-negligible** (decision #3); if negligible the node's `s` parent can be
dropped (simpler), if material it is threaded.

## 4. Where it lands on the tape (the carry generalization)

The merged articulated carry is `RigidStateCarryVjp { j_state, g: Vec<f64> }` with
parents `[s, fz]` and `g`'s velocity rows `= −rigid_xfrc_column[:,5]` (the f_z
column; the `−` because its parent `fz_var` is `force_on_soft.z`, not the reaction).
Generalize to the full wrench:

- **`ContactWrenchTrajVjp`** (NEW): output `w = [w_τ; w_f]` (`[6]`), parents
  `[x*, s]`, applying §3.2 + §3.3. Self-contained (already the reaction wrench, so
  no downstream `neg`). Leaves the platen's `ContactForceTrajVjp` **byte-untouched**.
- **`RigidStateCarryVjp`** generalized to parents `[s, w]` (`w` a `[6]` wrench):
  `g: Vec<f64>` → `G_vel: DMatrix` (`nv × 6` = `rigid_xfrc_column`, velocity rows;
  position rows zero, §8a). `∂L/∂w[k] = Σᵢ G_vel[(i,k)]·cot[nv+i]`. NO sign flip
  (`w` is the reaction wrench directly). `J_state` carry unchanged (it already
  takes a full `SpatialVector` wrench via `loaded_state_jacobian`).

`rigid_xfrc_column` is **already `nv × 6`** — the merged code used only column 5.
This leaf uses all six. The rigid side already maps the moment; we only compute it
and thread `∂w/∂x*` (+ maybe `∂w/∂s`).

## 5. Scope (in / out)

- **In:** the off-COM moment for the **articulated** path (oracle + gradient), flat
  constant-normal penalty plane (`τ_z = 0`), hinge / raw `[qpos;qvel]` state,
  `rigid_damping = 0` (inherited from the merged articulated scope).
- **Out (untouched):** the **platen** path (`coupled_trajectory_material_gradient`
  + its scalar carry/VjpOps + gates) — moment `~0` by symmetry, byte-untouched
  (decision #4). `step()` stays the platen-flavored forward (origin-posed plane,
  COM moment `~0`) with its documented omission note.
- **Out (next arc):** tangential/frictional contact (`τ_z`), curved/posed
  primitives (rotating normals), the analytic geometric-stiffness term.

## 5a. Build findings (what the measure-first build resolved)

- **Decision #2 (moment material?) — YES.** The S0 spike: at step 0 the arm-COM
  arm to the contact centroid is ~0.08 m, `τ_y ≈ −68 N·m`, and `tip_z_N` shifts
  ~7% between pure-force-at-COM and the routed moment. Non-degenerate.
- **The wrench node is machine-exact.** All three channels `∂w/∂x*`, `∂w/∂h`,
  `∂w/∂s` FD-match the real contact readout to ≤1e-5 (gated in the
  `contact_wrench_node_matches_readout_fd` lib unit test). The naive omission of
  the `∂w/∂h` (force/moment-vs-plane-height) channel cost ~7% — it is the moment
  generalization of the merged scalar `ContactForceTrajVjp`'s `∂fz/∂h`, and is
  required.
- **`G_vel` must be FRESH (the key composition fix, 5e-3 → 1e-3).** The real `step`
  maps the wrench through `J(q_k)` (it forwards at the post-integrate `qpos`), but
  `self.data`'s `xipos`/`qM` lag one step. Reading `rigid_xfrc_column` off the stale
  `self.data` maps the wrench through `J(q_{k-1})` — negligible for the slowly-varying
  linear (force) Jacobian (the merged force path's ~6e-6) but material for the
  rotational (moment) Jacobian. `fresh_xfrc_column` (a scratch forward at `qpos`,
  matching `J_state`'s eval point) removes it.
- **Decision #3 (`∂w/∂s`) — material and KEPT.** With fresh `G_vel`, including the
  `c(q)` moment-feedback improves the gradient (1.6e-3 → 1.0e-3); it is correct and
  retained. (Before the `G_vel` fix it was masked by the larger stale-`G_vel` error.)
- **The residual is the moment's FD'd GEOMETRIC STIFFNESS, not contact smoothness.**
  Under PENALTY the composed gradient is machine-exact through make/break (≤~2e-6 to
  n≈6) and the residual GROWS with n over longer rollouts (1.0e-3 at n=10, 5.7e-3 at
  n=15). Under IPC it reaches the same ~1e-3 ORDER (n=10: 1.4e-3; spike-measured, not
  gated) — IPC does NOT reduce it (it is even looser at small n: 7.3e-5 at n=6) → so
  the residual is NOT the penalty C⁰-kink; it is the moment's config-sensitive
  `∂(Jᵀτ)/∂q` carried only to FD precision in `J_state`, amplified ~150× over the
  force path's 6e-6 by the rotational Jacobian's sensitivity. The analytic
  geometric-stiffness term (the merged leaf's documented follow-on) is the cure.
  Adequate for co-design gradient descent (direction + 99.9% of magnitude).
- **Do NOT re-attribute the stale seam to the previous state var.** A "lagged
  attribution" (wiring the stale pose-seam / `c`-feedback to `s_{k-1}`) was tried and
  is WORSE (14%, breaks n=2): the merged path's attribution of the stale seam to the
  CURRENT `s_k` is the correct calibration. Re-forwarding `self.data` is likewise
  worse (3–30%). The stale-FK timing is load-bearing exactly as PR #312 found.

## 6. The load-bearing invariants to preserve (carry forward from PR #312)

1. **Stale-FK eval timing is load-bearing.** `height`, `jz`, `g`, and now `c` /
   `J_lin` all read `self.data` at the PRE-integrate FK config `step` leaves behind;
   `J_state` (scratch-forward) is post-step. Do NOT re-forward to align them — it
   breaks the §8a structure (~10%, verified) and the oracle reads the identical
   stale config. The moment's `c = xipos` and `J_lin` follow the SAME stale read.
2. **§8a force-drop carry.** The carry's POSITION rows stay zeroed (`∂qpos'/∂w = 0`);
   the wrench reaches qpos only next step through qvel. Keep for all 6 components.
3. **n=1 from rest ⇒ zero μ-gradient.** The first soft step from rest has `gᵢ ≈ 0`
   ⇒ `w ≈ 0` ⇒ `∂(tip_z_1)/∂μ = 0` exactly. The moment must preserve this (it does:
   `w_τ` is linear in `gᵢ`, also `≈ 0` at rest).
4. **FD-gate against an INDEPENDENT re-rolled coupled oracle** (real coupled sim at
   μ±ε, moment routed identically). `Data` is not `Clone` (scratch rebuild; `&mut
   self` trajectory methods → fresh coupling per FD perturbation).

## 7. Ritual / slicing

recon (this) → **S0 spike** (throwaway: measure the moment magnitude + the forward
tip_z_N moment-ON vs moment-OFF Δ [decision #2]; prototype the wrench node + matrix
carry and FD-gate; measure `∂w/∂s` [decision #3]) → **one PR** (the wrench node +
generalized carry + moment routing in the articulated oracle + gradient + the gate
re-baselined + a unit test for the skew/two-part `∂w/∂x*`) → n+1 cold-read → commit
→ post-commit local ultra-review → grade A → push (on go-ahead) → PR → CI →
squash-merge. One PR is right: the node, the carry, the routing, and the gate are
one indivisible change to the articulated path (the gate fails unless all land
together). Head-engineer owns the slicing call.
