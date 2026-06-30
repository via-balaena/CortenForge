# Keystone S5 — the soft material-parameter VJP — RECON

*Active recon, opened 2026-06-10. The final keystone leaf: differentiate the soft equilibrium (and,
through the S4 crossing, a rigid-side outcome) w.r.t. the soft **material parameters** — the gradient
the co-design optimizer actually consumes. The soft autograd tape (`NewtonStepVjp`) is **load-only**
today; this adds the material-parameter sensitivity path. Sibling of `recon.md`, `s2/s3/s4_*.md`.*

> **Establish `∂x*/∂(material param)` (forward) and `∂L/∂(material param)` (reverse VJP) for the soft
> Newton step — extending the IFT machinery (`∂x*/∂θ = −A⁻¹ ∂r/∂θ`, load-only) to a material-residual
> sensitivity `∂x*/∂p = −A⁻¹ ∂r/∂p`, where `∂r/∂p = ∂f_int/∂p` assembles from the material stress
> derivative `∂P/∂p`, reusing the same factored tangent `A`. FD-validated; then route μ through the
> S4 crossing so one `tape.backward` gives `∂(rigid outcome)/∂μ` — the co-design gradient.**

---

## 1. Where the material parameters enter

The dynamic backward-Euler residual is `r(x; p) = (M/Δt²)(x − x̂) + f_int(x; p) − f_ext(θ)`. The
material parameters `p` (NeoHookean `μ, λ`) enter ONLY through `f_int` via the first Piola stress
`P(F; p)` (`assemble_global_int_force`: `f_int[3v+i] += V·Σ_j P_ij·∂N_a/∂X_j` per element). So

```text
    ∂r/∂p = ∂f_int/∂p ,   ∂f_int[3v+i]/∂p += V·Σ_j (∂P/∂p)_ij · ∂N_a/∂X_j  (same loop, ∂P/∂p for P).
```

**The stress-parameter derivative (closed form).** NeoHookean
`P(F) = μ(F − F⁻ᵀ) + λ ln(J) F⁻ᵀ` (`neo_hookean.rs`), so

```text
    ∂P/∂μ = F − F⁻ᵀ ,        ∂P/∂λ = ln(J) · F⁻ᵀ .
```

Both are already-available quantities (`F`, `F⁻ᵀ`, `ln J` are computed in `first_piola`). The
mass term and `f_ext(θ)` are `p`-independent, so `∂r/∂p` is purely this elastic assembly — and it
needs **no contact term** (penalty contact does not depend on the material params), so the path is
the same with or without contact (contact only enters the tangent `A`).

## 2. The IFT material sensitivity (reusing `A`)

At the converged step `r(x*; p) = 0`, with `A = ∂r/∂x|_{x*}` (the SAME factored tangent the forward
Newton step, the load adjoint, and S3's pose sensitivity all use):

- **Forward** (co-design-usable, S3-style): `∂x*/∂p = −A⁻¹·(∂r/∂p)` — solve `A·w = −(∂r/∂p)_free`.
- **Reverse VJP** (the tape, `NewtonStepVjp`-style): for upstream cotangent `g = ∂L/∂x*`,
  `∂L/∂p = −λ^T·(∂r/∂p)_free` where `A·λ = g_free`. Same adjoint solve as `NewtonStepVjp`; the only
  new ingredient vs the load VJP is `(∂r/∂p)_free = (∂f_int/∂p)_free` in place of
  `(∂r/∂θ)_free = −e_{loaded.z}`.

So the material VJP is structurally `NewtonStepVjp` with a different (computed, not unit) RHS factor.

## 3. Riding the S4 crossing

S4 differentiated a rigid outcome w.r.t. the soft **load** θ because θ is a tape input to
`Solver::step`. To get `∂vz'/∂μ`, make μ a tape input to the soft step (a `MaterialStepVjp` node,
parent = `mu_var`), and the existing S4 chain (`ContactForceVjp` → `neg` → `RigidStepVjp`) is
unchanged:

```text
 μ (leaf) ─[Solver step + MaterialStepVjp]→ x* ─[ContactForceVjp]→ fz ─[neg]→ xfrc ─[RigidStepVjp]→ vz'
```

`tape.backward(vz')` then yields `grad(μ) = ∂vz'/∂μ` — the co-design gradient (a stiffer soft body
deforms less under the platen, changing the contact force, hence the platen's motion). The
keystone block ties `λ = 4μ`; co-design over μ then uses `dP/dμ = ∂P/∂μ + 4·∂P/∂λ` (chain rule).

## 4. #1 RISK and the spike

**#1 RISK — is `∂P/∂μ = F − F⁻ᵀ` (and the assembled `∂r/∂μ`) correct, and does `−A⁻¹·∂r/∂μ` match a
re-solve FD over μ?** Sub-risks: **(a)** the assembly sign/scale (mirror the `f_int` loop exactly);
**(b)** λ-coupling — validate μ with λ held fixed first (isolates `∂P/∂μ`), then the `λ=4μ` combo;
**(c)** the converged-tangent reuse (same as S3 — no contact needed for the material path, so
validate on a clean loaded block).

**SPIKE (throwaway, `#[ignore]`, sim-soft):** a pinned + loaded soft block (NO contact — isolates the
material physics), re-solve the soft step at `μ±ε` and central-difference `x_final` → FD `∂x*/∂μ`;
compare to analytic `−A⁻¹·(∂r/∂μ)` (assemble `∂r/∂μ` from `∂P/∂μ`, factor at `x_final`). Confirm
machine-tolerance agreement. Retires R1 before the production code.

**SPIKE DONE 2026-06-10 (throwaway, deleted) — ★ R1 RETIRED.** A `#[ignore]`'d unit test in
`backward_euler.rs` (private access to the assembly + `factor_at_position`): a pinned + top-loaded
(`AxisZ`, no contact) `n=2` block, `μ₀=3e4`, `λ=1.2e5` (fixed), static `dt`, small load (`θ=5`,
small-strain/valid). Analytic `∂x*/∂μ = −A⁻¹·(∂r/∂μ)` with `∂r/∂μ` assembled from `∂P/∂μ = F − F⁻ᵀ`
(mirroring the `f_int` loop, factor at `x_final`) vs a re-solve central FD over μ:
**`‖∂x*/∂μ‖_∞ = 2.1e-7`, analytic-vs-FD rel = 1.4e-9** (machine-tolerance). So `∂P/∂μ`, the
`∂r/∂μ` assembly, and the converged-tangent reuse are all correct — ready to productionize.

## 5. Decisions (head-engineer)

- **D1 — `Material::first_piola_param_grad(F) → Vec<∂P/∂p_k>`** (default empty; NeoHookean returns
  `[F − F⁻ᵀ, ln(J)·F⁻ᵀ]` in `(μ, λ)` order). The trait extension; mirrors how `gradient`/`tangent`
  expose F-derivatives. Default-empty = a material with no differentiable params (the
  `pose_residual_derivative` precedent).
- **D2 — forward sensitivity first (`equilibrium_material_sensitivity`), reverse VJP second.** S3
  shipped the forward sensitivity as its validated primitive; the reverse VJP (the tape) earns it
  after the physics is FD-gated. Both reuse `factor_at_position`.
- **D3 — validate on a loaded block (no contact).** The material path is contact-independent;
  isolating it is the cleanest gate. The contact-engaged crossing is the later leaf.
- **D4 — start with μ (λ fixed), then the λ=4μ combo for the crossing.** Cleanest FD first.

## 6. Sub-leaf ladder / slicing

- **Spike (THROWAWAY, `#[ignore]`).** §4 — retire R1 on a loaded block.
- **PR1 (sim-soft) — material-residual sensitivity + forward `∂x*/∂p`.**
  `Material::first_piola_param_grad` + `CpuNewtonSolver::equilibrium_material_sensitivity(x_final,
  dt, param_idx) → ∂x*/∂p`, FD-gated (re-solve at `p±ε`) on a loaded block; per-param (μ and λ).
- **PR2 (sim-soft) — the material VJP on the tape.** `MaterialStepVjp` (reverse: solve `A·λ = g`,
  contract `−λ^T ∂r/∂p`) + a tape entry making the material param a differentiable input;
  `tape.backward → ∂L/∂p`, FD-gated against the forward sensitivity.
- **PR3 (sim-coupling) — the material crossing.** `coupled_step_material_gradient(height, μ) →
  (vz', ∂vz'/∂μ)` routing μ through the S4 `ContactForceVjp`/`RigidStepVjp` chain; FD-gated against
  the full coupled step. **★ the co-design gradient.**

Each leaf: n+1 cold-read + pre-PR local ultra-review; no push/PR without go-ahead.

## Progress · PR1/PR2/PR3 DONE (2026-06-10) — ★ THE CO-DESIGN GRADIENT CROSSES BOTH ENGINES

- **PR1 (sim-soft) — forward material sensitivity.** `Material::first_piola_param_grad(F)` (default
  empty; `NeoHookean` → `[F − F⁻ᵀ, ln(J)·F⁻ᵀ]`) + `CpuNewtonSolver::equilibrium_material_sensitivity(
  x_final, dt, param_idx) → ∂x*/∂p` (the `f_int` assembly with `∂P/∂p` for `P`, then `−A⁻¹·` reusing
  the factor at `x_final`). Gate `tests/material_sensitivity.rs`: `∂P/∂μ`,`∂P/∂λ` vs FD ~1e-10;
  `∂x*/∂μ`,`∂x*/∂λ` vs re-solve FD ~1e-9 (loaded block, no contact); the default-empty contract
  (`Yeoh`). grade A.
- **PR2 (sim-soft) — the reverse material VJP on the tape.** `MaterialStepVjp` (mirrors
  `NewtonStepVjp`: solve `A·λ = g_free`, contract `−λ^T·(∂r/∂p)_free`) + `material_step_vjp(x_final,
  dt, param_idx)`; `∂r/∂p` assembly factored into a shared helper. Gate: a one-node tape `μ → x*`,
  `backward` (seed ones ⇒ `L = Σx*`), `grad(μ)` matches the forward-sum (<1e-10) AND a re-solve FD —
  proving the reverse VJP, not self-consistency. grade A.
- **PR3 (sim-coupling) — the co-design crossing.** `coupled_step_material_gradient(height, param_idx)
  → (vz', ∂vz'/∂p)` routes the material parameter through the SAME S4 chain (a shared `run_crossing_tail`
  appends `ContactForceVjp → neg → RigidStepVjp`; only the soft node differs — `MaterialStepVjp` vs the
  load adjoint); `coupled_step_material_vz` is the forward FD oracle (`mu`/`lambda` now stored on the
  coupling). Gated by the `material[μ]`/`material[λ]` rows of `tests/coupling_grad_harness.rs`: **∂vz'/∂μ tape = 1.869e-5 vs full-coupled FD =
  1.869e-5 (rel 7.5e-9)**, ∂vz'/∂λ rel 1.7e-9; + lib smoke. grade A (Coverage A+).

★ S5 = the co-design gradient `∂(rigid outcome)/∂(soft material)` now crosses BOTH engines via one
`tape.backward`. The keystone differentiability arc (forward → explicit → implicit-total → tape
crossing → material VJP) is COMPLETE. Beyond v1: non-NeoHookean params (Yeoh `C₂`); the `λ=4μ` total
via the documented linear combination; IPC for clean active-set gradients.

## 7. Risks

- **R1 (#1) — `∂P/∂p` / assembly correctness + IFT-vs-resolve.** §4 spike measures it directly.
- **R2 — λ=4μ coupling** in the crossing (chain rule). MITIGATION: validate μ-alone first; the
  combo is a documented linear combination of the per-param sensitivities.
- **R3 — making the material a tape input** (PR2) is a real soft-autograd extension (the load path
  is built around θ). MITIGATION: `MaterialStepVjp` mirrors `NewtonStepVjp`'s adjoint solve exactly
  — only the RHS factor differs; the forward sensitivity (PR1) de-risks the physics first.
- **R4 — non-NeoHookean materials** (Yeoh `C₂`) — out of scope for v1; `first_piola_param_grad`
  default-empty keeps them safe (no silent-wrong gradient — they just expose no params yet).

## 8. Validation gate (S5)

CI-runnable: (PR1) analytic `∂x*/∂p` agrees with a re-solve central FD on a loaded block (per μ, λ);
(PR2) `tape.backward → ∂L/∂p` agrees with the forward sensitivity + FD; (PR3) `∂vz'/∂μ` via one
`tape.backward` agrees with a full-coupled-step FD. Honest scope: NeoHookean `(μ, λ)`; engaged /
stable-active-set for the crossing; the `λ=4μ` combo documented.

## Key files / pointers

- Material: `sim/L0/soft/src/material/mod.rs` (`Material` trait), `neo_hookean.rs`
  (`first_piola` = `μ(F−F⁻ᵀ)+λ ln(J)F⁻ᵀ`).
- Solver assembly + factor: `sim/L0/soft/src/solver/backward_euler.rs`
  (`assemble_global_int_force` — the `f_int` loop to mirror; `factor_at_position`;
  `equilibrium_pose_sensitivity` — the S3 forward-sensitivity precedent to mirror).
- Soft autograd (load adjoint to mirror for the reverse VJP):
  `sim/L0/soft/src/differentiable/newton_vjp.rs`.
- The S4 crossing the material gradient rides: `sim/L1/coupling/src/lib.rs`
  (`coupled_step_load_gradient`, `ContactForceVjp`, `RigidStepVjp`).
