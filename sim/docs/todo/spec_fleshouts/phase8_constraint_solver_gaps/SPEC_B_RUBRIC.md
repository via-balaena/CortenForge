# Spec B — QCQP Cone Projection: Spec Quality Rubric

Grades the Spec B spec on 11 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

**Spec B scope:** QCQP-based friction cone projection for the PGS solver
and noslip post-processor (DT-19). Substantial infrastructure exists —
this is a **conformance verification and gap closure** spec, not greenfield.
The primary gap is the PGS elliptic projection: CortenForge uses simple
scaling (`project_elliptic_cone`), while MuJoCo uses a two-phase ray
update + QCQP approach.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

Spec B is part of the Phase 8 umbrella. Empirical verification against the
MuJoCo C source reveals significant scope refinements.

| Umbrella claim | MuJoCo reality | Action |
|----------------|----------------|--------|
| "`mj_projectConstraint()` in `engine_solver.c`" | **No such function exists.** The projection logic is embedded directly in `mj_solPGS()` (two-phase: ray update + QCQP for elliptic contacts, scalar projection for all others). The umbrella's function name was a shorthand. | **In scope** — spec must cite `mj_solPGS()` elliptic branch instead |
| "Verify `project_elliptic_cone()` matches MuJoCo's projection" | **It does NOT match.** CortenForge's `project_elliptic_cone()` (noslip.rs:15-57) is a simple scale-to-boundary projector. MuJoCo's PGS uses a two-phase algorithm: (1) ray update scaling force along current direction, then (2) QCQP minimization on the friction subproblem with the AR subblock. Fundamentally different. | **In scope** — replace `project_elliptic_cone` in PGS with two-phase ray+QCQP |
| "Verify noslip QCQP solvers match MuJoCo" | CortenForge's `noslip_qcqp2/3` (noslip.rs:76-286) use a different formulation than MuJoCo's `mju_QCQP2/3`. MuJoCo solves `min 0.5*x'*A*x + x'*b` with `b = res - A*old`. CortenForge solves `min ||f - f_unc||²_A` with `f_unc` from scalar GS update. **These produce different results** because CortenForge's bias `g = A_s * y_unc` != MuJoCo's bias `b_s = D*(res - A*old)`. | **In scope** — rewrite noslip QCQP to match MuJoCo's `mju_QCQP2/3/N` exactly |
| "Handle condim=6" | CortenForge noslip has a simple rescaling fallback for group_len > 3 (noslip.rs:586-603). MuJoCo uses `mju_QCQP()` (N-dim, up to dim=5) for condim=6. | **In scope** — add `qcqp_nd` for condim=6 |
| "Verify integration across all three solvers (PGS, CG, Newton)" | CG and Newton use primal `classify_constraint_states` (pgs.rs:272-488) with analytic zone-based evaluation — NOT QCQP. This is correct: MuJoCo CG/Newton also use `PrimalUpdateConstraint` (analytic zones), not `mj_solPGS`-style QCQP. | **In scope** as verification only — no changes to CG/Newton primal classifier |
| "Condim dispatch: frictionless → clamp, pyramidal → per-row, elliptic → QCQP" | Correct. CortenForge PGS already dispatches by ConstraintType (pgs.rs:128-207). Pyramidal = `max(0, f)`. Frictionless = `max(0, f)`. The gap is only in the elliptic branch. | **In scope** — verify existing non-elliptic projection, fix elliptic |

**Final scope:**
1. Replace PGS elliptic projection with MuJoCo's two-phase (ray + QCQP)
2. Rewrite noslip QCQP to match `mju_QCQP2/3/N` formulation exactly
   (bias construction, convergence checks, step guard, control flow)
3. Add `qcqp_nd` for condim=6 (noslip only — PGS also needs it)
4. Fix noslip pyramidal cost rollback threshold (1e-15 → 1e-10)
5. Add noslip iter==0 cost correction (`0.5*f²*R` for regularization removal)
6. Verify PGS scalar projection matches MuJoCo (equality, friction loss,
   limits, frictionless/pyramidal contacts)
7. Verify CG/Newton primal classifier handles elliptic cone zones correctly
8. Keep `project_elliptic_cone` for simple use cases or remove if unused

---

## Empirical Ground Truth

### MuJoCo behavioral verification

**MuJoCo version:** 3.x (open-source GitHub `google-deepmind/mujoco`, `main` branch)
**Source files verified:** `engine_solver.c` (PGS, noslip), `engine_util_solve.c` (QCQP)

### EGT-1: MuJoCo PGS two-phase elliptic projection

**MuJoCo C source:** `mj_solPGS()` in `engine_solver.c`, elliptic branch
(~lines 370-550).

The PGS loop iterates over all constraints. For elliptic contacts
(`efc_type == mjCNSTR_CONTACT_ELLIPTIC`), `dim = contact.dim` (3, 4, or 6).
For all other types, `dim = 1`.

**Phase 1 — Ray or Normal update:**

```c
// Extract dim×dim AR subblock
extractBlock(m, d, Athis, i, dim, 0);  // flg_subR=0 (regularized)

if (force[i] < mjMINVAL) {
    // Normal force too small: scalar normal update
    force[i] -= res[0] * ARinv[i];
    if (force[i] < 0) force[i] = 0;
    mju_zero(force+i+1, dim-1);  // clear friction
} else {
    // Ray update: scale along current force direction
    v = force[i..i+dim];  // ray = current force vector
    v1 = Athis * v;       // dim×dim matrix-vector product
    denom = v' * v1;
    if (denom >= mjMINVAL) {
        x = -v' * res / denom;       // optimal step along ray
        if (force[i] + x*v[0] < 0)   // guard normal non-negative
            x = -force[i] / v[0];    // v[0] == force[i] (just copied), so x = -1
        force[i..i+dim] += x * v;    // apply ray update
    }
}
```

Key: The ray update moves the ENTIRE force vector (normal + friction)
along the current direction. This is different from CortenForge which does
per-row scalar GS updates.

**Phase 2 — Friction QCQP (normal fixed):**

```c
// Extract friction-friction subblock: Ac = Athis[1:,1:]
// Compute adjusted bias: bc = res[1:] - Ac * oldforce[1:] + Athis[1:,0] * (force[i] - oldforce[0])
for (j = 0; j < dim-1; j++) {
    Ac[j*(dim-1)..] = Athis[(j+1)*dim+1..(j+1)*dim+dim];
    bc[j] = res[j+1] - dot(Ac[j,:], oldforce[1:]) + Athis[(j+1)*dim] * (force[i] - oldforce[0]);
}

if (force[i] < mjMINVAL) {
    mju_zero(force+i+1, dim-1);  // zero friction if normal ≈ 0
} else {
    // Dispatch QCQP by dimension
    if (dim == 3) flg_active = mju_QCQP2(v, Ac, bc, mu, force[i]);
    else if (dim == 4) flg_active = mju_QCQP3(v, Ac, bc, mu, force[i]);
    else flg_active = mju_QCQP(v, Ac, bc, mu, force[i], dim-1);

    // Ellipsoidal rescale if constraint active
    if (flg_active) {
        s = sqrt(force[i]^2 / max(MINVAL, Σ(v[j]/mu[j])²));
        v *= s;
    }
    force[i+1..i+dim] = v;
}
```

Key behavioral facts:
- `Ac` is the (dim-1)×(dim-1) friction-friction subblock of AR (regularized)
- `bc` is the adjusted residual incorporating the Phase 1 normal force change
- `oldforce` is saved BEFORE Phase 1 ray update
- `mu` = `contact.friction` array (per-direction friction coefficients)
- `force[i]` = current normal force (from Phase 1) used as cone radius
- Ellipsoidal rescale ensures exact cone boundary when QCQP is approximate
- PGS uses `flg_subR=0` (R included in AR matrix)

### EGT-2: MuJoCo QCQP solvers (`mju_QCQP2/3/N`)

**MuJoCo C source:** `engine_util_solve.c`

All three solve: `min 0.5*x'*A*x + x'*b  s.t.  Σ(x_i/d_i)² ≤ r²`

Where `d` = friction coefficients (mu), `r` = normal force (cone radius).

**Algorithm (uniform across 2/3/N):**

1. **Scale to unit sphere:** `b_s[i] = b[i] * d[i]`, `A_s[i][j] = A[i][j] * d[i] * d[j]`
   Constraint becomes `y'*y ≤ r²` where `y_i = x_i / d_i`

2. **Newton iteration on dual λ** (up to 20 iterations):
   - `P = (A_s + λI)⁻¹` (2×2: closed-form, 3×3: cofactor, N×N: Cholesky)
   - `v = -P * b_s` (unconstrained solution at current λ)
   - `val = v'*v - r²` (constraint violation)
   - If `val < 1e-10`: converged (inside or on boundary), break
   - `deriv = -2 * v' * P * v` (∂val/∂λ)
   - `delta = -val / deriv`
   - If `delta < 1e-10`: step too small, break
   - `λ += delta` (NO clamping to λ ≥ 0 — MuJoCo doesn't clamp!)

3. **Unscale:** `res[i] = v_final[i] * d[i]`

4. **Return:** `1` if `λ != 0` (constraint active), `0` otherwise

**Critical differences from CortenForge noslip QCQP:**
- MuJoCo starts `λ = 0` and does NOT clamp `λ ≥ 0`
- CortenForge `noslip_qcqp2` clamps `lam = lam.max(0.0)` — this is wrong
- MuJoCo checks `val < 1e-10` (convergence) and `delta < 1e-10` (step guard)
- CortenForge checks `phi.abs() < 1e-10` — the `abs()` is wrong (MuJoCo
  only checks `val < 1e-10`, NOT `|val| < 1e-10`)
- MuJoCo has step guard: `delta = -val/deriv; if (delta < 1e-10) break;`
- CortenForge has NO step guard on delta — only `dphi.abs() < MJ_MINVAL`.
  These are non-equivalent: MuJoCo guards step size, CortenForge guards
  derivative magnitude. MuJoCo would continue when derivative is tiny
  (letting delta be large), CortenForge would break.
- MuJoCo checks `det < 1e-10` for SPD, returns zero on failure
- CortenForge checks `det.abs() < MJ_MINVAL` (1e-15) — different threshold
- MuJoCo reuses `v1, v2` from the last loop iteration after break — no
  separate "final solve". CortenForge does a redundant final solve outside
  the loop (noslip.rs:140-165) and has a degenerate fallback (simple
  rescaling, noslip.rs:146-152). MuJoCo just returns zero on degenerate.
- MuJoCo's `deriv` formula uses `P*v*v` terms directly (cofactor already computed)
- CortenForge recomputes `M⁻¹*y` via a second solve (correct but slower)
- MuJoCo does NOT do final rescaling to cone boundary; CortenForge does
  (noslip.rs:159-162) — this is extra, not MuJoCo-conformant
- MuJoCo's `mju_QCQP` (N-dim) uses Cholesky factorization; CortenForge
  has no N-dim QCQP (noslip falls back to simple rescaling for dim > 3)

### EGT-3: MuJoCo PGS scalar projection (non-elliptic)

**MuJoCo C source:** `mj_solPGS()` scalar branch.

```c
// Unconstrained GS update
force[i] -= res[0] * ARinv[i];

// Project by constraint position (NOT by efc_type):
if (i >= ne && i < ne+nf) {
    // Friction loss: box clamp [-floss, +floss]
    force[i] = clamp(force[i], -floss[i], floss[i]);
} else if (i >= ne+nf) {
    // Limits + contacts: unilateral max(0, f)
    force[i] = max(0, force[i]);
}
// else: equality (i < ne): no projection
```

**Key fact:** MuJoCo dispatches by **index position** (`ne`, `nf` boundaries),
not by `efc_type`. CortenForge dispatches by `efc_type` (pgs.rs:175-193).
Both produce the same result because constraint ordering is:
`[0..ne) = equality`, `[ne..ne+nf) = friction`, `[ne+nf..nefc) = limits+contacts`.
CortenForge's `efc_type`-based dispatch is equivalent and correct.

### EGT-4: MuJoCo noslip QCQP

**MuJoCo C source:** `mj_solNoSlip()` in `engine_solver.c`.

Noslip processes two categories:
1. **Dry friction** (rows `ne..ne+nf`): scalar GS + box clamp (same as PGS
   but with `flg_subR=1` — unregularized)
2. **Contact friction** (rows `ne+nf..nefc`):
   - **Pyramidal:** 2×2 block solve on opposing edge pairs (same algorithm as
     CortenForge's Phase C in noslip.rs:622-706)
   - **Elliptic:** QCQP on friction rows only (skip normal row `i`, process
     `i+1..i+dim-1`). Same `mju_QCQP2/3/N` dispatch + ellipsoidal rescale.

**Key differences from PGS QCQP:**
- `flg_subR=1`: uses unregularized AR (subtracts R from diagonal)
- Only friction rows processed (normal force is fixed from main solver)
- Residual is relative to CURRENT forces (not saved old forces from Phase 1)
- `bc = res - Ac * oldforce` (where `oldforce` is friction forces before update)
- Same `mju_QCQP2/3/N` + ellipsoidal rescale as PGS

CortenForge noslip differences:
- Correct: uses unregularized Delassus (noslip.rs:402-459)
- Correct: only processes friction/contact rows (noslip.rs:346-394)
- **Wrong:** QCQP bias construction differs (uses `f_unc` from scalar GS,
  not `bc = res - Ac * old` from MuJoCo)
- **Wrong:** QCQP2/3 convergence criteria differ (see EGT-2)
- **Wrong:** condim=6 fallback uses simple rescaling instead of `mju_QCQP`
- **Wrong:** Pyramidal cost rollback threshold is `MJ_MINVAL` (1e-15) at
  noslip.rs:689; MuJoCo uses `costChange()` which thresholds at `1e-10`.
  CortenForge is more aggressive at reverting (reverts even on tiny cost
  increases between 1e-15 and 1e-10 that MuJoCo would accept).
- **Wrong:** Missing iter==0 cost correction. MuJoCo adds
  `0.5*force[i]²*R[i]` for all rows at iter 0 to account for the
  regularization cost removed by the unregularized noslip pass. CortenForge
  does not include this correction, which affects convergence behavior.

### EGT-5: MuJoCo PGS cost guard

**MuJoCo C source:** `costChange()` in `engine_solver.c`.

```c
// dim=1: use 1/ARinv (scalar diagonal)
if (dim == 1) Athis[0] = 1/ARinv[i];
improvement -= costChange(Athis, force+i, oldforce, res, dim);
```

```c
static mjtNum costChange(Athis, force, oldforce, res, dim) {
    delta = force - oldforce;
    if (dim == 1) change = 0.5*delta²*A + delta*res;
    else change = 0.5*delta'*A*delta + delta'*res;
    if (change > 1e-10) { force = oldforce; change = 0; }
    return change;
}
```

Key: Cost guard uses the dim×dim AR subblock `Athis`, not the full matrix.
CortenForge PGS computes cost change using the full AR matrix
(pgs.rs:220-255) which is numerically equivalent but the implementation
differs. For the scalar case, CortenForge's implementation at pgs.rs:196-204
is correct.

### EGT-6: CG/Newton primal classifier (zone-based, NOT QCQP)

**MuJoCo C source:** `PrimalUpdateConstraint` / `mj_constraintUpdate_impl`
in `engine_solver.c`.

CG and Newton use the **primal** constraint classifier, not dual QCQP.
For elliptic contacts, this uses three zones:
- **Top (Satisfied):** `μ·N ≥ T` — separated, zero force
- **Bottom (Quadratic):** `N + μ·T ≤ 0` — fully active, per-row force
- **Middle (Cone):** on cone surface, force from KKT

CortenForge's `classify_constraint_states` (pgs.rs:272-488) implements
this correctly. The cone Hessian (hessian.rs) is used by Newton only
(not CG — CG sets `flg_HessianCone=false`).

**No changes needed for CG/Newton primal classifier.** The QCQP cone
projection is PGS-only and noslip-only.

### Codebase Context

| Item | File | Line(s) | Status |
|------|------|---------|--------|
| PGS elliptic branch (scalar GS + simple scaling) | pgs.rs | 131-160 | **Replace** with two-phase ray+QCQP |
| `project_elliptic_cone()` (simple scale-to-boundary) | noslip.rs | 15-57 | **Keep** for `project_elliptic_cone` consumers; PGS will no longer call it |
| PGS scalar projection (equality, friction, limits, pyramidal) | pgs.rs | 161-207 | **Verify** — currently correct |
| PGS cost guard (full-matrix computation) | pgs.rs | 150-158, 196-254 | **Verify** — may need AR subblock for elliptic |
| `noslip_qcqp2()` | noslip.rs | 76-166 | **Rewrite** to match `mju_QCQP2` exactly |
| `noslip_qcqp3()` | noslip.rs | 172-286 | **Rewrite** to match `mju_QCQP3` exactly |
| condim=6 noslip fallback (simple rescaling) | noslip.rs | 586-603 | **Replace** with `qcqp_nd` matching `mju_QCQP` |
| Noslip elliptic bias construction (`f_unc` based) | noslip.rs | 517-533 | **Rewrite** to match `bc = res - Ac*old` |
| Noslip pyramidal 2×2 block solve | noslip.rs | 622-706 | **Fix** — cost rollback threshold is `MJ_MINVAL` (1e-15) vs MuJoCo's `1e-10` |
| Noslip iter==0 cost correction | noslip.rs | 478 | **Add** — MuJoCo adds `0.5*f²*R` at iter 0 for regularization removal |
| `classify_constraint_states()` (primal zone classifier) | pgs.rs | 272-488 | **Verify** — no changes expected (CG/Newton only) |
| Cone Hessian (Newton only) | hessian.rs | full | **Verify** — no changes expected |
| Primal line search (CG/Newton shared) | primal.rs | full | **Verify** — handles elliptic zones correctly |
| PGS `compute_delassus_regularized()` | pgs.rs | 21-55 | **No change** — PGS needs full AR |
| `ConstraintType` enum | enums.rs | 705-730 | **No change** — all types present |
| `efc_mu` field (per-row friction array) | data.rs | 354-356 | **No change** — `[f64; 5]` supports condim=6 |
| `efc_dim` field (per-row condim) | data.rs | ~356 | **No change** |
| `efc_id` field (per-row contact index) | data.rs | ~350 | Used for AR subblock extraction |
| Existing PGS tests | newton_solver.rs, unified_solvers.rs | various | **May break** if PGS elliptic results change |
| Existing noslip tests | noslip.rs | 23-792 | **May break** if noslip QCQP results change |
| Existing CG tests | cg_solver.rs | various | **Should not break** |

### EGT-7: AR subblock extraction

MuJoCo's `extractBlock()` extracts a dim×dim subblock from the AR matrix
(or A matrix for noslip). CortenForge's PGS has the full AR matrix as a
dense `DMatrix`. Extracting a subblock is trivial: `ar.slice((i,i),(dim,dim))`.

For the noslip, CortenForge already builds a submatrix `a_sub` (noslip.rs:422).
The noslip QCQP needs the friction-friction subblock from `a_sub`, which is
`a_sub.slice((gs, gs), (gl, gl))` where `gs`=group_start, `gl`=group_len.
This is already done at noslip.rs:543-575.

### EGT-8: `project_elliptic_cone` usage sites

`project_elliptic_cone` is currently called from exactly one place:
- PGS elliptic branch: pgs.rs:148

After Spec B, PGS will use the two-phase ray+QCQP algorithm instead.
`project_elliptic_cone` may be removed or kept as a utility. No other
callers exist in the codebase.

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade P1 first and grade it hardest. If P1 is not A+, do not
> proceed to grading other criteria until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Spec cites all of: (1) `mj_solPGS()` in `engine_solver.c` — complete two-phase elliptic projection: ray update with `v'*Athis*v` denominator, normal-force guard, then QCQP on friction-friction subblock `Ac` with adjusted bias `bc` incorporating normal force change from Phase 1; (2) `mju_QCQP2()` in `engine_util_solve.c` — exact scaling convention (`b_s = b*d`, `A_s = A*d_i*d_j`), Newton iteration with `λ` starting at 0 (NO clamping), convergence `val < 1e-10` (NOT `|val|`), step guard `delta < 1e-10` (NOT dphi guard), SPD check `det < 1e-10` (NOT `det.abs() < 1e-15`), up to 20 iterations, loop values reused directly (no separate final solve), unscale `res = v*d`, return `λ != 0`; (3) `mju_QCQP3()` — same algorithm with 3×3 cofactor inverse; (4) `mju_QCQP()` — N-dim with Cholesky, max dim=5; (5) `mj_solNoSlip()` in `engine_solver.c` — `flg_subR=1` (unregularized), elliptic QCQP on friction rows only with `bc = res - Ac*oldforce`, pyramidal 2×2 block solve with `costChange` threshold `1e-10`, iter-0 cost correction `0.5*f²*R` for regularization removal; (6) `costChange()` — AR subblock cost guard with revert threshold `1e-10`; (7) Ellipsoidal rescale: `s = sqrt(fn² / max(MINVAL, Σ(v_j/mu_j)²))` applied when `flg_active`. Edge cases addressed: zero normal force (clear friction, Phase 1 normal-only update), degenerate `det < 1e-10` (return zero — no rescaling fallback), `denom < mjMINVAL` (skip ray update), condim=1 (frictionless — never enters elliptic branch). |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing specifics or based on docs rather than C source. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood, assumed, or invented. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. Rust code is
> line-for-line implementable. The algorithm must reproduce MuJoCo's
> behavior exactly.

| Grade | Bar |
|-------|-----|
| **A+** | For PGS: exact Rust code for (1) AR subblock extraction from DMatrix, (2) ray update with all guards (normal < MINVAL, denom < MINVAL, force non-negativity), (3) QCQP bias construction (`bc[j] = res[j+1] - Σ Ac[j,k]*oldforce[1+k] + Athis[(j+1)*dim]*(force[i]-oldforce[0])`), (4) QCQP dispatch by dim, (5) ellipsoidal rescale. For QCQP2/3/N: exact Rust functions matching `mju_QCQP2/3/N` — scaling, Newton loop, convergence checks, unscaling, return value. For noslip: exact bias construction (`bc = res - Ac*old`), QCQP dispatch for condim=3/4/6, pyramidal 2×2 block solve verification. |
| **A** | Algorithm is complete and MuJoCo-conformant. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but some steps are hand-waved. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses CortenForge conventions where they differ from
> MuJoCo and provides the correct translation.

| Grade | Bar |
|-------|-----|
| **A+** | Convention table present covering: (1) MuJoCo `contact.friction[5]` → CortenForge `efc_mu[i]: [f64; 5]` (same 5-element layout); (2) MuJoCo `contact.dim` → CortenForge `efc_dim[i]` (per-row); (3) MuJoCo `efc_AR` flat `nefc×nefc` → CortenForge `DMatrix<f64>` (same dense); (4) MuJoCo `force` pointer arithmetic `force+i` → CortenForge `data.efc_force` DVector slice; (5) MuJoCo `contact.mu` (regularized friction, computed in `mj_makeImpedance`) → CortenForge `efc_mu[i][0]` (verify this is the same value); (6) MuJoCo `flg_subR` flag → CortenForge noslip unregularized submatrix (already correct); (7) MuJoCo `extractBlock` → CortenForge `ar.slice()` or manual extraction. Index convention: MuJoCo `Ac[j*(dim-1)+k]` row-major → CortenForge `[[f64; N]; N]` or nalgebra slice. |
| **A** | Major conventions documented. Minor mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has three-part structure: (1) concrete model (condim, friction coefficients, solver type), (2) exact expected behavior (force values, constraint satisfaction), (3) what to check. ACs include: (a) PGS with condim=3 elliptic contact — force values converge to same result as MuJoCo (state tolerance); (b) PGS ray update produces non-negative normal force even from adversarial initialization; (c) QCQP2 matches `mju_QCQP2` for a specific `(A, b, d, r)` input (exact values given); (d) QCQP3 matches `mju_QCQP3` similarly; (e) Noslip with condim=3 — friction forces satisfy cone constraint; (f) Noslip condim=6 — uses QCQP not simple rescaling; (g) CG/Newton elliptic cone zones unchanged (regression); (h) No regression on existing solver tests. Code-review ACs for structural correctness (QCQP function signature, cost guard threshold). |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague. |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory includes: zero normal force (friction zeroed), degenerate AR subblock (det < 1e-10 → zero result), unconstrained QCQP solution inside cone (no projection needed), condim=1 frictionless (never enters elliptic branch), condim=3 (2-DOF QCQP), condim=4 (3-DOF QCQP), condim=6 (5-DOF QCQP), mu=0 for one direction (degenerate cone), very large friction coefficient, very small normal force (boundary behavior), PGS ray update when all friction zero (pure normal), PGS ray update with denom < mjMINVAL (skip ray), noslip pyramidal pair solve with one zero edge (edge case of K1 < MINVAL), QCQP Newton step too small (delta < 1e-10 early exit), QCQP hitting max 20 iterations without converging. Negative cases: QCQP not called for pyramidal contacts, QCQP not called for CG/Newton. At least one direct conformance test: same model, same state, verify PGS `efc_force` matches MuJoCo output. Regression: all existing PGS, noslip, CG, Newton tests pass. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order is unambiguous. Section numbering (S1, S2, ...) with each section stating requirements from prior sections. S1 (QCQP utility functions) has no dependencies. S2 (PGS two-phase projection) depends on S1. S3 (noslip QCQP rewrite) depends on S1. S4 (verification + tests) depends on S2 and S3. Prerequisites: Phase 8 umbrella (`4574a03`), existing solver infrastructure (PGS, noslip, CG, Newton). No dependency on Spec A (different pipeline stage). |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and
> every existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. Behavioral changes: (1) PGS elliptic contact forces will change numerically (moves toward MuJoCo conformance — ray+QCQP vs simple scaling); (2) Noslip elliptic friction forces will change (QCQP formulation correction); (3) Noslip condim=6 will change (QCQP vs rescaling). CG/Newton: **no behavioral change** (primal classifier unchanged). Existing test impact: `test_pgs_contact_force_quantitative` may need updated expected values (PGS results change), `test_s32_elliptic_no_regression` may need update, noslip tests (`noslip.rs:23-792`) may need updated values. All changes move toward MuJoCo conformance. Crates affected: sim-core only (constraint/solver/). No parsing, no model fields, no assembly changes. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical
> terminology throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform: "QCQP" (not "QP" or "quadratic projection"), "ray update" (not "ray projection" or "normal update"), "ellipsoidal rescale" (not "cone rescale"), "AR subblock" (not "Delassus block"). Field names `efc_mu`, `efc_dim`, `efc_force` used identically everywhere. File paths in Specification match Files Affected. AC numbers match Traceability Matrix. Edge cases in MuJoCo Reference appear in Test Plan. Function count: 3 QCQP functions (qcqp2, qcqp3, qcqp_nd) consistent between MuJoCo Reference, Algorithm, and Test Plan. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Mathematical Formulation

> The QCQP mathematical formulation must be precisely stated with
> objective function, constraints, KKT conditions, Newton iteration
> derivation, and convergence criteria.

| Grade | Bar |
|-------|-----|
| **A+** | Spec states: (1) objective `min 0.5*x'*A*x + b'*x` with constraint `Σ(x_i/d_i)² ≤ r²`, (2) KKT: `(A + λ*C)*x = -b` where `C = diag(1/d²)`, (3) after scaling `y=x/d`: `(A_s + λI)*y = -b_s` with `y'*y ≤ r²`, (4) Newton: `φ(λ) = ||y(λ)||² - r²`, `φ'(λ) = -2*y(λ)' * (A_s+λI)⁻¹ * y(λ)`, `Δλ = -φ/φ'`, (5) convergence: `φ < 1e-10` OR `Δλ < 1e-10`, (6) SPD check: `det(A_s+λI) < 1e-10 → return zero`, (7) PGS-specific: objective for friction subproblem, how `Ac` and `bc` relate to the full AR and residual, (8) noslip-specific: same QCQP but with unregularized `A` and different `bc` construction. |
| **A** | Math is correct. One or two derivation steps left implicit. |
| **B** | Math stated at high level without full derivation. |
| **C** | No mathematical formulation. |

**Boundary with P1:** P1 grades whether the spec correctly describes what
MuJoCo *does* (function-by-function). P9 grades whether the spec correctly
derives *why* the algorithm works (mathematical correctness of the QCQP
formulation and its connection to the friction cone constraint).

### P10. Solver Integration Completeness

> Every solver type (PGS, CG, Newton, noslip) is addressed with the
> correct projection mechanism. The spec must explicitly state which
> solvers use QCQP and which use zone-based classification.

| Grade | Bar |
|-------|-----|
| **A+** | Explicit table mapping solver → projection mechanism: PGS → two-phase ray+QCQP (implement), CG → primal zone classifier (verify unchanged), Newton → primal zone classifier + cone Hessian (verify unchanged), noslip → QCQP (rewrite). For each solver, the spec states: what function implements projection, what inputs it receives, what outputs it produces, and whether changes are needed. Condim dispatch table per solver: condim=1 (all: scalar `max(0,f)`), condim=3 (PGS: QCQP2, noslip: QCQP2, CG/Newton: zone), condim=4 (PGS: QCQP3, noslip: QCQP3, CG/Newton: zone), condim=6 (PGS: QCQP_N, noslip: QCQP_N, CG/Newton: zone). |
| **A** | All solvers addressed. Minor gap in condim dispatch. |
| **B** | Some solvers addressed, others hand-waved. |
| **C** | Only one solver discussed. |

**Boundary with P2:** P2 grades whether the algorithm for each modified
component is fully specified. P10 grades whether the spec identifies ALL
components that need attention (including those that need verification
only, not modification).

### P11. Conformance Gap Traceability

> Every difference between CortenForge's current behavior and MuJoCo's
> behavior is explicitly enumerated, with the fix for each gap.

| Grade | Bar |
|-------|-----|
| **A+** | Gap table with columns: Gap ID, Current CortenForge, MuJoCo Behavior, Fix, AC that verifies fix. Gaps include: (G1) PGS simple scaling vs ray+QCQP, (G2) QCQP `λ` clamping (`max(0)` vs unclamped), (G3) QCQP convergence check (`abs(val)` vs `val`), (G4) QCQP SPD threshold (1e-15 vs 1e-10), (G5) noslip QCQP bias construction (f_unc vs bc=res-Ac*old), (G6) noslip condim=6 fallback (rescaling vs QCQP_N), (G7) noslip final rescale to cone boundary (extra, not in MuJoCo), (G8) PGS cost guard uses full matrix vs subblock, (G9) QCQP Newton step guard — MuJoCo exits on `delta < 1e-10`, CortenForge has no step guard (only guards `dphi`), (G10) QCQP "final solve outside loop" + degenerate rescaling fallback — MuJoCo reuses loop values, returns zero on degenerate, (G11) noslip pyramidal cost rollback threshold `MJ_MINVAL` (1e-15) vs MuJoCo `1e-10`, (G12) noslip iter==0 cost correction — MuJoCo adds `0.5*f²*R` to improvement, CortenForge doesn't. Each gap links to an AC that verifies the fix is correct. |
| **A** | Most gaps identified. Minor gaps missed. |
| **B** | Some gaps identified, systematic enumeration missing. |
| **C** | Gaps not tracked. |

**Boundary with P1:** P1 grades whether the MuJoCo reference is described
correctly. P11 grades whether the spec systematically enumerates all
deviations between the reference and the current codebase.

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific file:line references
      (pgs.rs:131-160, noslip.rs:76-286, noslip.rs:586-603), MuJoCo C
      function names (`mj_solPGS`, `mju_QCQP2/3/N`, `mj_solNoSlip`,
      `costChange`, `extractBlock`), exact thresholds (1e-10 convergence,
      1e-10 SPD check, 1e-10 cost guard, 20 Newton iterations), and
      concrete gap descriptions (λ clamping, `abs(val)` vs `val`). Two
      independent reviewers would agree on the grade by checking these items.

- [x] **Non-overlap:** P1 (MuJoCo reference) vs P9 (mathematical
      formulation): P1 grades what MuJoCo does (C code), P9 grades the
      mathematical derivation of why. P1 vs P11 (gap traceability): P1
      grades whether MuJoCo is correctly described, P11 grades whether all
      CortenForge deviations are enumerated. P2 (algorithm) vs P10
      (solver integration): P2 grades algorithm completeness for modified
      components, P10 grades coverage across all solver types.

- [x] **Completeness:** 11 criteria cover: MuJoCo conformance (P1),
      algorithm (P2), conventions (P3), ACs (P4), tests (P5), dependencies
      (P6), blast radius (P7), consistency (P8), mathematical formulation
      (P9), solver integration (P10), gap traceability (P11). The
      "verification + gap closure" nature of the spec (not greenfield) is
      reflected in the bars — P11 exists specifically because this is a
      conformance audit.

- [x] **Gradeability:** P1→MuJoCo Reference; P2→Specification (S1..SN);
      P3→Convention Notes; P4→Acceptance Criteria; P5→Test Plan; P6→
      Prerequisites/Execution Order; P7→Files Affected/Blast Radius;
      P8→cross-cutting; P9→Mathematical Formulation section;
      P10→Solver Integration table; P11→Conformance Gap table.

- [x] **Conformance primacy:** P1 is tailored with specific MuJoCo C
      functions (`mj_solPGS`, `mju_QCQP2/3/N`, `mj_solNoSlip`,
      `costChange`). P4 requires numerical conformance ACs. P5 requires
      MuJoCo-vs-CortenForge conformance tests. P11 systematically tracks
      every deviation.

- [x] **Empirical grounding:** EGT-1 through EGT-8 filled in with
      MuJoCo C source excerpts and CortenForge codebase line references.
      Every P1 A+ bar item has a corresponding EGT entry. Critical bugs
      found: λ clamping (EGT-2), convergence check (EGT-2), SPD threshold
      (EGT-2), missing step guard (EGT-2), QCQP control flow mismatch
      (EGT-2), QCQP bias construction (EGT-4), condim=6 fallback (EGT-4),
      noslip pyramidal cost threshold (EGT-4), noslip iter-0 correction
      (EGT-4), PGS projection architecture (EGT-1). Total: 12 conformance
      gaps (G1–G12) tracked in P11, 15 gap log entries (R1–R15).

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, ...) — algorithm at each stage |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Mathematical Formulation section (QCQP objective, KKT, Newton) |
| P10 | Solver Integration table (PGS, CG, Newton, noslip) |
| P11 | Conformance Gap table (current vs MuJoCo, fix, AC link) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
| P9. Mathematical Formulation | | |
| P10. Solver Integration Completeness | | |
| P11. Conformance Gap Traceability | | |

**Overall: — (Rev 3, post stress-test)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope Adjustment | Umbrella claims `mj_projectConstraint()` exists — no such function in MuJoCo source. Projection is inlined in `mj_solPGS()`. | Phase 1 (MuJoCo C source read) | Documented in Scope Adjustment. P1 A+ bar references `mj_solPGS()` instead. | Rev 1 |
| R2 | P1/EGT-2 | MuJoCo QCQP does NOT clamp λ ≥ 0; CortenForge `noslip_qcqp2` does (`lam.max(0.0)` at noslip.rs:137). This is a conformance bug. | Phase 1 (MuJoCo C source read) | Documented in EGT-2 as a critical difference. P11 gap G2 tracks the fix. | Rev 1 |
| R3 | P1/EGT-2 | MuJoCo QCQP convergence checks `val < 1e-10`; CortenForge uses `phi.abs() < 1e-10`. The `abs()` is non-conformant — MuJoCo exits on NEGATIVE val (inside cone) without further iteration. | Phase 1 (MuJoCo C source read) | Documented in EGT-2. P11 gap G3 tracks the fix. | Rev 1 |
| R4 | P1/EGT-2 | MuJoCo QCQP SPD check uses `det < 1e-10`; CortenForge uses `det.abs() < MJ_MINVAL` (1e-15). Different threshold AND different abs treatment. | Phase 1 (MuJoCo C source read) | Documented in EGT-2. P11 gap G4 tracks the fix. | Rev 1 |
| R5 | P1/EGT-1 | PGS elliptic projection is fundamentally different architecture: CortenForge uses per-row scalar GS + simple scaling; MuJoCo uses two-phase ray+QCQP. This is the primary conformance gap. | Phase 1 (MuJoCo C source read + CortenForge pgs.rs read) | Documented in EGT-1 and Scope Adjustment. P11 gap G1. | Rev 1 |
| R6 | P1/EGT-4 | Noslip QCQP bias construction differs: CortenForge uses `f_unc` from scalar GS update; MuJoCo uses `bc = res - Ac*old`. These produce different results because the scalar GS update distributes error differently than the block formulation. | Phase 1 (MuJoCo C source noslip read) | Documented in EGT-4. P11 gap G5. | Rev 1 |
| R7 | P1/EGT-4 | Noslip condim=6 uses simple rescaling fallback (noslip.rs:586-603) instead of `mju_QCQP` (N-dim). | Phase 1 (CortenForge noslip.rs read) | Documented in EGT-4. P11 gap G6. Need `qcqp_nd` function. | Rev 1 |
| R8 | P10 | CG/Newton use primal zone-based classification, not QCQP. Initially unclear whether CG/Newton need changes. Verified: MuJoCo CG/Newton also use primal classification (`PrimalUpdateConstraint`), not PGS-style QCQP. No changes needed. | Phase 1 (MuJoCo C source CG/Newton read) | Documented in EGT-6. P10 bar explicitly requires this verification. | Rev 1 |
| R9 | P5 | Missing edge case: MuJoCo QCQP returns zero vector when `det < 1e-10` (degenerate). CortenForge has different fallback behavior. Must test degenerate QCQP inputs. | Stress test (EGT-2 edge case analysis) | Added to P5 A+ bar edge case inventory: "degenerate AR subblock". | Rev 2 |
| R10 | P7 | `project_elliptic_cone` only called from PGS (pgs.rs:148). After Spec B, it becomes unused in PGS. Should the function be removed or kept? | Stress test (EGT-8 consumer audit) | Added EGT-8 documenting usage. Decision: keep for now (dead code removal is separate scope). Spec should note the change. | Rev 2 |
| R11 | P1/EGT-2 | CortenForge `noslip_qcqp2/3` does final rescale to cone boundary (noslip.rs:159-165, 279-285) that MuJoCo does NOT do in the QCQP functions themselves. MuJoCo only does the ellipsoidal rescale OUTSIDE the QCQP, in the caller (`mj_solPGS`, `mj_solNoSlip`), keyed on `flg_active`. | Stress test (MuJoCo C source QCQP read) | Documented in EGT-2. P11 gap G7 — rescale in QCQP is wrong location; should be in caller only. | Rev 2 |
| R12 | P1/EGT-2 | CortenForge QCQP has NO Newton step guard `delta < 1e-10`. MuJoCo computes `delta = -val/deriv` then breaks if `delta < 1e-10`. CortenForge only guards `dphi.abs() < MJ_MINVAL` (derivative magnitude) which is a different check. When derivative is tiny (1e-16) but phi is also tiny (1e-12), delta would be ~1e4 — MuJoCo continues, CortenForge breaks. | Stress test (MuJoCo C source QCQP2/3/N verified) | Added to EGT-2. P11 gap G9 — add `delta < 1e-10` guard, remove `dphi.abs()` guard. | Rev 3 |
| R13 | P1/EGT-2 | CortenForge QCQP has "final solve outside loop" (noslip.rs:140-165) with a degenerate fallback (simple rescaling). MuJoCo reuses `v1,v2` from the last loop iteration — no separate final solve. MuJoCo returns zero on degenerate `det < 1e-10`, never does rescaling fallback. The whole QCQP control flow structure is wrong. | Stress test (MuJoCo C source structure analysis) | Added to EGT-2. P11 gap G10 — rewrite QCQP to match MuJoCo's control flow (no final solve, no rescaling fallback). | Rev 3 |
| R14 | P1/EGT-4 | Noslip pyramidal 2×2 block solve cost rollback uses `MJ_MINVAL` (1e-15) threshold at noslip.rs:689. MuJoCo's `costChange()` uses `1e-10`. CortenForge is more aggressive at reverting — any cost increase > 1e-15 triggers revert, while MuJoCo tolerates increases up to 1e-10. | Stress test (MuJoCo costChange threshold vs CortenForge noslip.rs:689) | Added to EGT-4. P11 gap G11 — change threshold from `MJ_MINVAL` to `1e-10`. | Rev 3 |
| R15 | P1/EGT-4 | Noslip missing iter==0 cost correction. MuJoCo's `mj_solNoSlip` at iter 0 adds `0.5*force[i]²*R[i]` to improvement for all nefc rows. This accounts for the regularization cost that the unregularized noslip pass removes. CortenForge noslip does not include this correction, affecting convergence timing. | Stress test (MuJoCo mj_solNoSlip iter==0 branch) | Added to EGT-4. P11 gap G12 — add iter==0 regularization cost correction. | Rev 3 |
