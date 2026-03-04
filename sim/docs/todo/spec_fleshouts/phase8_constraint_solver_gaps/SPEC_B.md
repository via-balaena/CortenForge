# Spec B — QCQP Cone Projection

**Status:** Draft
**Phase:** Roadmap Phase 8 — Constraint & Solver Gaps
**Effort:** L (rewrite PGS elliptic branch + rewrite noslip QCQP + add N-dim QCQP)
**MuJoCo ref:** `mj_solPGS()` in `engine_solver.c`, elliptic branch;
`mju_QCQP2/3()` in `engine_util_solve.c`; `mju_QCQP()` in `engine_util_solve.c`;
`mj_solNoSlip()` in `engine_solver.c`; `costChange()` in `engine_solver.c`
**MuJoCo version:** 3.x (open-source `google-deepmind/mujoco`, `main` branch)
**Test baseline:** 1,900+ sim domain tests (post-Phase 7)
**Prerequisites:**
- Phase 8 umbrella (commit `4574a03`)
- Existing solver infrastructure: PGS (`pgs.rs`), noslip (`noslip.rs`),
  CG (`cg.rs`), Newton (`newton.rs`), primal classifier (`primal.rs`),
  cone Hessian (`hessian.rs`)

**Independence:** This spec is independent of Spec A (Solver Param & Margin
Completeness) per the umbrella dependency graph. Spec A modifies constraint
parameter wiring (`solref`/`solimp` fields, `tendon_margin`); Spec B modifies
force projection in solver iterations. Different pipeline stages: Spec A
operates at row assembly time, Spec B operates at solver iteration time.
Shared file: `assembly.rs` — Spec B does NOT modify it.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

Spec B is part of the Phase 8 umbrella. Empirical verification against the
MuJoCo C source reveals significant scope refinements from the umbrella's
original description.

| Umbrella claim | MuJoCo reality | Action |
|----------------|----------------|--------|
| "`mj_projectConstraint()` in `engine_solver.c`" | **No such function exists.** Projection logic is inlined in `mj_solPGS()` (two-phase: ray update + QCQP for elliptic, scalar for others). The umbrella's function name was shorthand. | **In scope** — cite `mj_solPGS()` elliptic branch |
| "Verify `project_elliptic_cone()` matches MuJoCo" | **It does NOT match.** CortenForge's `project_elliptic_cone()` (noslip.rs:22-57) is simple scale-to-boundary. MuJoCo PGS uses two-phase ray+QCQP. Fundamentally different. | **In scope** — replace PGS elliptic branch with two-phase |
| "Verify noslip QCQP matches MuJoCo" | CortenForge's `noslip_qcqp2/3` use a different formulation and have multiple conformance bugs (see Gap Table). | **In scope** — rewrite to match `mju_QCQP2/3/N` |
| "Handle condim=6" | CortenForge falls back to simple rescaling for group_len > 3. MuJoCo uses `mju_QCQP()` (N-dim, Cholesky). | **In scope** — add `qcqp_nd` |
| "Verify across all three solvers (PGS, CG, Newton)" | CG and Newton use primal `classify_constraint_states` (zone-based), NOT QCQP. MuJoCo CG/Newton also use `PrimalUpdateConstraint` (analytic zones). | **In scope** as verification only — no changes to CG/Newton |
| "Condim dispatch" | CortenForge PGS dispatches by `ConstraintType` (pgs.rs:128-207); MuJoCo dispatches by index position (`ne`, `nf` boundaries). Both produce identical results because constraint ordering is fixed. | **Verified correct** — no change needed |

**Final scope:**
1. Rewrite QCQP utility functions (`qcqp2`, `qcqp3`, add `qcqp_nd`) to
   match `mju_QCQP2/3/N` exactly (12 conformance fixes)
2. Replace PGS elliptic branch with MuJoCo's two-phase (ray update + QCQP)
3. Rewrite noslip elliptic QCQP (bias construction, dispatch for condim=6)
4. Fix noslip convergence: pyramidal cost threshold (1e-10), iter-0
   cost correction (`0.5*f²*R`)
5. Verify PGS scalar projection and CG/Newton primal classifier (no changes)
6. Remove `project_elliptic_cone` call from PGS (keep function as dead code
   for potential future use)

---

## Problem Statement

CortenForge's friction cone projection during PGS solver iterations and the
noslip post-processor does not match MuJoCo's behavior. This is a
**conformance gap** — MuJoCo implements QCQP-based cone projection in
`mj_solPGS()` (engine_solver.c) and `mj_solNoSlip()` (engine_solver.c);
CortenForge uses fundamentally different algorithms.

The gap has three dimensions:

1. **PGS elliptic projection architecture** (G1): CortenForge uses per-row
   scalar Gauss-Seidel updates followed by simple scaling to the cone
   boundary (`project_elliptic_cone`). MuJoCo uses a two-phase algorithm:
   (a) ray update scaling the entire force vector along its current direction,
   then (b) QCQP minimization on the friction subproblem with the
   regularized AR subblock. These produce different force vectors.

2. **QCQP solver conformance** (G2-G4, G9-G10): The existing `noslip_qcqp2/3`
   functions have 6 conformance bugs: wrong λ clamping, wrong convergence
   check, wrong SPD threshold, missing step guard, wrong control flow
   (final solve outside loop), and extra rescaling fallback.

3. **Noslip formulation** (G5-G8, G11-G12): The noslip elliptic QCQP uses
   a different bias construction than MuJoCo, is missing N-dim QCQP for
   condim=6, uses the wrong cost rollback threshold for pyramidal contacts,
   and is missing the iter-0 regularization cost correction.

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream —
> the algorithm, the acceptance criteria, the tests — is derived from what's
> documented here.

### PGS Two-Phase Elliptic Projection

**Source:** `mj_solPGS()` in `engine_solver.c`, elliptic branch.

The PGS loop iterates over all constraints. For elliptic contacts
(`efc_type == mjCNSTR_CONTACT_ELLIPTIC`), `dim = contact.dim` (3, 4, or 6).
For all other constraint types, `dim = 1` (scalar projection).

**Phase 1 — Ray update:**

```c
// Save old forces before any update
mju_copy(oldforce, force+i, dim);

// Compute full residual for the block: res = b + AR * force
// (using full nefc-wide AR rows, not just the subblock)
for (j=0; j<dim; j++) {
    res[j] = b[i+j];
    for (k=0; k<nefc; k++) res[j] += AR[(i+j)*nefc+k] * force[k];
}

// Extract dim×dim AR subblock: Athis[j*dim+k] = AR[(i+j)*nefc+(i+k)]
extractBlock(m, d, Athis, i, dim, 0);  // flg_subR=0 (regularized)

if (force[i] < mjMINVAL) {
    // Normal force too small: scalar normal update + clear friction
    force[i] -= res[0] * ARinv[i];    // ARinv = 1/AR[i,i]
    if (force[i] < 0) force[i] = 0;
    mju_zero(force+i+1, dim-1);
} else {
    // Ray update: scale along current force direction
    mju_copy(v, force+i, dim);           // v = current force vector
    mju_mulMatVec(v1, Athis, v, dim, dim); // v1 = Athis * v
    denom = mju_dot(v, v1, dim);          // denom = v' * Athis * v
    if (denom >= mjMINVAL) {
        x = -mju_dot(v, res, dim) / denom;  // optimal step along ray
        if (force[i] + x*v[0] < 0)          // guard: normal stays ≥ 0
            x = -force[i] / v[0];           // v[0] = force[i], so x = -1
        mju_addToScl(force+i, v, x, dim);   // force += x * v
    }
}
```

Key: The ray update moves the ENTIRE force vector (normal + friction) along
the current direction. This is fundamentally different from CortenForge's
per-row scalar GS updates.

**Phase 2 — Friction QCQP (normal fixed):**

```c
// Extract friction-friction subblock: Ac = Athis[1:,1:]
// Ac[(dim-1)×(dim-1)] — friction rows/cols only
for (j=0; j<dim-1; j++) {
    for (k=0; k<dim-1; k++)
        Ac[j*(dim-1)+k] = Athis[(j+1)*dim+(k+1)];
    // Build bias: bc includes normal force change from Phase 1
    bc[j] = res[j+1];
    for (k=0; k<dim-1; k++)
        bc[j] -= Ac[j*(dim-1)+k] * oldforce[1+k];
    bc[j] += Athis[(j+1)*dim+0] * (force[i] - oldforce[0]);
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
        mjtNum ssq = 0;
        for (j=0; j<dim-1; j++) ssq += (v[j]/mu[j]) * (v[j]/mu[j]);
        mjtNum s = mju_sqrt(force[i]*force[i] / mju_max(mjMINVAL, ssq));
        mju_scl(v, v, s, dim-1);
    }
    mju_copy(force+i+1, v, dim-1);
}

// Cost guard
improvement -= costChange(Athis, force+i, oldforce, res, dim);
```

**Key behavioral facts:**
- `Ac` is the (dim-1)×(dim-1) friction-friction subblock of AR (regularized)
- `bc` is the adjusted residual incorporating the Phase 1 normal force change
- `oldforce` is saved BEFORE Phase 1 ray update
- `mu` = `contact.friction[]` (per-direction friction coefficients)
- `force[i]` = current normal force (from Phase 1) used as cone radius
- Ellipsoidal rescale ensures exact cone boundary when QCQP is approximate
- Cost guard uses AR subblock `Athis`, not the full matrix

### QCQP Solvers (`mju_QCQP2/3/N`)

**Source:** `engine_util_solve.c`.

All three solve the same problem:

```
min  0.5 * x' * A * x  +  b' * x
s.t. Σ (x_i / d_i)² ≤ r²
```

Where `d` = friction coefficients (mu), `r` = normal force (cone radius).

**Algorithm (uniform across 2/3/N):**

1. **Scale to unit sphere:** `b_s[i] = b[i] * d[i]`,
   `A_s[i][j] = A[i][j] * d[i] * d[j]`.
   Constraint becomes `||y||² ≤ r²` where `y_i = x_i / d_i`.

2. **Newton iteration on dual λ** (up to 20 iterations):
   - Compute `P = (A_s + λI)⁻¹`
     - 2×2: closed-form inverse via determinant
     - 3×3: cofactor inverse
     - N×N: Cholesky factorization
   - `y = -P * b_s` (unconstrained solution at current λ)
   - `val = ||y||² - r²` (constraint violation)
   - If `val < 1e-10`: converged (inside or on boundary), break
   - `deriv = -2 * y' * P * y` (∂val/∂λ)
   - `delta = -val / deriv`
   - If `delta < 1e-10`: step too small, break
   - `λ += delta` (**NO** clamping to λ ≥ 0)

3. **SPD check:** If `det(A_s + λI) < 1e-10`, return zero vector with
   active = false.

4. **Unscale:** `result[i] = y_final[i] * d[i]`

5. **Return:** `active = (λ != 0)`

**`mju_QCQP2` (2×2, condim=3):**

```c
int mju_QCQP2(mjtNum res[2], const mjtNum A[4], const mjtNum b[2],
               const mjtNum d[2], mjtNum r) {
    // Scale: b_s = b*d, A_s = A*d_i*d_j
    mjtNum bs0 = b[0]*d[0], bs1 = b[1]*d[1];
    mjtNum As00 = A[0]*d[0]*d[0], As01 = A[1]*d[0]*d[1];
    mjtNum As11 = A[3]*d[1]*d[1];
    mjtNum r2 = r*r, lambda = 0, v0, v1;

    for (int iter=0; iter<20; iter++) {
        mjtNum M00 = As00+lambda, M11 = As11+lambda, M01 = As01;
        mjtNum det = M00*M11 - M01*M01;
        if (det < 1e-10) { res[0]=0; res[1]=0; return 0; }
        mjtNum idet = 1.0/det;
        v0 = -(M11*bs0 - M01*bs1)*idet;
        v1 = -(-M01*bs0 + M00*bs1)*idet;
        mjtNum val = v0*v0 + v1*v1 - r2;
        if (val < 1e-10) break;
        mjtNum pv0 = (M11*v0-M01*v1)*idet, pv1 = (-M01*v0+M00*v1)*idet;
        mjtNum deriv = -2*(v0*pv0+v1*pv1);
        mjtNum delta = -val/deriv;
        if (delta < 1e-10) break;
        lambda += delta;
    }
    res[0] = v0*d[0]; res[1] = v1*d[1];
    return (lambda != 0);
}
```

**`mju_QCQP3` (3×3, condim=4):** Same algorithm with 3×3 cofactor inverse.

**`mju_QCQP` (N×N, condim=6):** Same algorithm with Cholesky factorization
for `(A_s + λI)`. Maximum dimension is 5 (condim=6 → 5 friction DOFs).

### PGS Scalar Projection

**Source:** `mj_solPGS()` scalar branch.

```c
// Unconstrained GS update
force[i] -= res[0] * ARinv[i];

// Project by constraint position (NOT by efc_type):
if (i >= ne && i < ne+nf) {
    force[i] = clamp(force[i], -floss[i], floss[i]);  // friction loss
} else if (i >= ne+nf) {
    force[i] = max(0, force[i]);  // limits + contacts: unilateral
}
// else: equality (i < ne): no projection
```

MuJoCo dispatches by index position (`ne`, `nf` boundaries). CortenForge
dispatches by `efc_type` (pgs.rs:175-193). Both produce identical results
because constraint ordering is `[0..ne) = equality`, `[ne..ne+nf) = friction`,
`[ne+nf..nefc) = limits+contacts`. **No change needed.**

### Noslip QCQP

**Source:** `mj_solNoSlip()` in `engine_solver.c`.

Noslip processes two categories:

1. **Dry friction** (rows `ne..ne+nf`): scalar GS + box clamp (same as PGS
   but with `flg_subR=1` — unregularized)

2. **Contact friction** (rows `ne+nf..nefc`):
   - **Pyramidal:** 2×2 block solve on opposing edge pairs (same algorithm
     as CortenForge's Phase C in noslip.rs:622-706)
   - **Elliptic:** QCQP on friction rows only. Skip normal row `i`, process
     `i+1..i+dim-1`.

**Noslip elliptic QCQP bias construction:**

```c
// Save old friction forces
mju_copy(oldforce, force+i+1, dim-1);

// Compute residual at friction rows (using unregularized A)
for (j=0; j<dim-1; j++) {
    res[j] = b[i+1+j];
    for (k=0; k<nefc; k++) res[j] += A[(i+1+j)*nefc+k] * force[k];
}

// Extract friction-friction subblock: Ac from unregularized A
for (j=0; j<dim-1; j++)
    for (k=0; k<dim-1; k++)
        Ac[j*(dim-1)+k] = A[(i+1+j)*nefc+(i+1+k)];

// Build bias: bc = res - Ac * old
for (j=0; j<dim-1; j++) {
    bc[j] = res[j];
    for (k=0; k<dim-1; k++) bc[j] -= Ac[j*(dim-1)+k] * oldforce[k];
}

// QCQP dispatch + ellipsoidal rescale (same as PGS Phase 2)
if (dim == 3) flg_active = mju_QCQP2(v, Ac, bc, mu, force[i]);
else if (dim == 4) flg_active = mju_QCQP3(v, Ac, bc, mu, force[i]);
else flg_active = mju_QCQP(v, Ac, bc, mu, force[i], dim-1);

if (flg_active) {
    // Ellipsoidal rescale (same formula as PGS)
    ...
}
mju_copy(force+i+1, v, dim-1);
```

**Key differences from PGS QCQP:**
- `flg_subR=1`: uses **unregularized** Delassus (subtracts R from diagonal)
- Only friction rows processed (normal force is fixed from main solver)
- No Phase 1 ray update (normal is fixed)
- Bias: `bc = res - Ac * oldforce` (simpler than PGS Phase 2 bias)

**Noslip iter-0 cost correction:**

```c
// At iter == 0, add regularization cost for all rows
if (iter == 0) {
    for (j=0; j<nefc; j++)
        improvement += 0.5 * force[j] * force[j] * R[j];
}
```

This accounts for the regularization cost that the unregularized noslip
pass removes. Without this correction, the convergence check is biased.

**Noslip pyramidal cost rollback:**

```c
// costChange threshold is 1e-10 (NOT MJ_MINVAL = 1e-15)
if (change > 1e-10) {
    mju_copy(force+i, oldforce, 2);  // revert
    change = 0;
}
```

### Cost Guard (`costChange`)

**Source:** `costChange()` in `engine_solver.c`.

```c
static mjtNum costChange(const mjtNum* Athis, const mjtNum* force,
                         const mjtNum* oldforce, const mjtNum* res, int dim) {
    // delta = force - oldforce
    // change = 0.5 * delta' * Athis * delta + delta' * res
    mjtNum change;
    if (dim == 1) {
        mjtNum delta = force[0] - oldforce[0];
        change = 0.5*delta*delta*Athis[0] + delta*res[0];
    } else {
        change = 0;
        for (int j=0; j<dim; j++) {
            mjtNum delta_j = force[j] - oldforce[j];
            change += delta_j * res[j];
            for (int k=0; k<dim; k++) {
                mjtNum delta_k = force[k] - oldforce[k];
                change += 0.5 * delta_j * Athis[j*dim+k] * delta_k;
            }
        }
    }
    // Revert if cost increased
    if (change > 1e-10) {
        mju_copy(force, oldforce, dim);
        change = 0;
    }
    return change;
}
```

Key: Uses the dim×dim AR subblock `Athis`, not the full nefc×nefc matrix.
The revert threshold is `1e-10`.

### CG/Newton Primal Classifier

**Source:** `PrimalUpdateConstraint` / `mj_constraintUpdate_impl` in
`engine_solver.c`.

CG and Newton use the **primal** constraint classifier, NOT dual QCQP.
For elliptic contacts, this uses three zones:
- **Top (Satisfied):** `μ·N ≥ T` — separated, zero force
- **Bottom (Quadratic):** `N + μ·T ≤ 0` — fully active, per-row force
- **Middle (Cone):** on cone surface, force from KKT

CortenForge's `classify_constraint_states` (pgs.rs:272-488) implements this
correctly. The cone Hessian (hessian.rs) is used by Newton only. CG uses
`flg_HessianCone=false`.

**No changes needed for CG/Newton primal classifier.** QCQP cone projection
is PGS-only and noslip-only.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| PGS elliptic projection | Two-phase: ray update along current force direction + QCQP on friction subblock (`mj_solPGS` in `engine_solver.c`) | Per-row scalar GS + simple scaling (`project_elliptic_cone` at pgs.rs:148) — **fundamentally different** |
| QCQP λ update | No clamping: `λ += delta` (`mju_QCQP2` in `engine_util_solve.c`) | Clamps: `lam = lam.max(0.0)` (noslip.rs:137, 248) — **non-conformant** |
| QCQP convergence | `val < 1e-10` (one-sided) | `phi.abs() < 1e-10` (two-sided, noslip.rs:123, 234) — **non-conformant** |
| QCQP SPD check | `det < 1e-10` → return zero | `det.abs() < MJ_MINVAL` (1e-15, noslip.rs:113, 216) — **different threshold and abs treatment** |
| QCQP step guard | `delta < 1e-10` → break | `dphi.abs() < MJ_MINVAL` (noslip.rs:133, 244) — **different check** |
| QCQP control flow | Loop values reused directly, no final solve | Final solve outside loop + degenerate rescaling fallback (noslip.rs:140-165) — **extra code not in MuJoCo** |
| QCQP degenerate | Return zero vector | Simple rescaling fallback (noslip.rs:146-152) — **non-conformant** |
| Noslip elliptic bias | `bc = res - Ac * oldforce` | `f_unc` from scalar GS update (noslip.rs:528-533) — **different formulation** |
| Noslip condim=6 | `mju_QCQP()` (N-dim Cholesky) | Simple rescaling fallback (noslip.rs:586-603) — **non-conformant** |
| Noslip pyramidal cost threshold | `1e-10` (costChange) | `MJ_MINVAL` = 1e-15 (noslip.rs:689) — **over-aggressive revert** |
| Noslip iter-0 cost correction | `0.5*f²*R` added to improvement | Missing (noslip.rs:478) — **affects convergence** |
| PGS cost guard (elliptic) | AR subblock (`Athis`) with `costChange()` | Full-matrix reconstruction (`pgs_cost_change` at pgs.rs:220-255) — **numerically equivalent but different approach** |
| CG/Newton elliptic | Primal zone-based classifier (`PrimalUpdateConstraint`) | `classify_constraint_states` (pgs.rs:272-488) — **correct, no change needed** |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `contact.friction[5]` | C array, friction coefficients per direction | `efc_mu[i]: [f64; 5]` (data.rs:354) | Direct port — same 5-element layout |
| `contact.dim` | Per-contact dimension (1, 3, 4, or 6) | `efc_dim[i]: usize` (data.rs:356) — per-row, same value for all rows of a contact | Direct port — index with `data.efc_dim[i]` |
| `efc_AR` | Flat `nefc×nefc` row-major | `DMatrix<f64>` (nalgebra, column-major internally) | Use `ar[(i,j)]` — nalgebra handles storage order |
| `force+i` pointer arithmetic | C pointer to `force[i]` through `force[i+dim-1]` | `data.efc_force` `DVector` slice: `data.efc_force.as_slice()[i..i+dim]` | Use `.as_slice()` / `.as_mut_slice()` for contiguous access |
| `contact.mu` | Regularized friction (from `mj_makeImpedance`) | `efc_mu[i][0..5]` — same values, set in assembly | Verify `efc_mu` is populated with the same friction values as MuJoCo |
| `flg_subR` flag | `0` = regularized (include R), `1` = unregularized (subtract R) | PGS: always regularized. Noslip: unregularized submatrix built explicitly (noslip.rs:401-459) | PGS uses `ar` (includes R). Noslip uses `a_sub` (excludes R). Already correct. |
| `extractBlock` | Extract dim×dim subblock from flat nefc×nefc array | `ar.slice((i,i),(dim,dim))` from nalgebra `DMatrix` | Use nalgebra `slice` method |
| `Ac[j*(dim-1)+k]` | Row-major (dim-1)×(dim-1) flat array | `[[f64; N]; N]` fixed-size arrays for N=2,3; `Vec<f64>` flat row-major for N-dim | For `qcqp2/3`: use `[[f64; N]; N]`. For `qcqp_nd`: use flat `&[f64]` row-major. |
| `ARinv[i]` | `1.0 / AR[i,i]` precomputed scalar | `ar_diag_inv[i]` (pgs.rs:77-83) | Direct port — same computation |
| `efc_R[i]` | Per-constraint regularization scalar | `data.efc_R: Vec<f64>` (data.rs:335) | Used in noslip iter-0 correction (S4) |

### Solver Integration Table

| Solver | Projection mechanism | Applies to | Changes needed |
|--------|---------------------|-----------|----------------|
| **PGS** | Two-phase ray+QCQP for elliptic; scalar GS+project for others | All contacts (dual space) | **Implement** — replace per-row GS+scaling with two-phase |
| **CG** | Primal zone classifier (`classify_constraint_states`) | All contacts (primal space) | **Verify** — no change |
| **Newton** | Primal zone classifier + cone Hessian | All contacts (primal space) | **Verify** — no change |
| **Noslip** | QCQP on friction subblock (unregularized) | Elliptic contact friction only | **Rewrite** — fix bias, add qcqp_nd, fix thresholds |

**Condim dispatch per solver:**

| Condim | PGS | Noslip | CG/Newton |
|--------|-----|--------|-----------|
| 1 (frictionless) | Scalar `max(0, f)` | Not processed | Zone classifier |
| 3 (2 friction DOFs) | Ray+QCQP2+rescale | QCQP2+rescale | Zone classifier |
| 4 (3 friction DOFs) | Ray+QCQP3+rescale | QCQP3+rescale | Zone classifier |
| 6 (5 friction DOFs) | Ray+QCQP_ND+rescale | QCQP_ND+rescale | Zone classifier |
| Pyramidal | Scalar `max(0, f)` per row | 2×2 block solve | Zone classifier |

---

## Mathematical Formulation

### QCQP Problem Statement

The friction cone constraint for an elliptic contact with normal force `f_n`
and friction coefficients `d = (μ₁, μ₂, ..., μ_{m})` (where `m = dim-1`)
requires:

```
Σᵢ (xᵢ / dᵢ)² ≤ f_n²
```

The QCQP subproblem minimizes a quadratic objective subject to this constraint:

```
min   ½ x'Ax + b'x
s.t.  Σᵢ (xᵢ/dᵢ)² ≤ r²     (r = f_n = normal force)
```

Where:
- `x ∈ ℝᵐ` — friction force vector
- `A ∈ ℝᵐˣᵐ` — friction-friction subblock of the Delassus matrix
  (regularized for PGS, unregularized for noslip)
- `b ∈ ℝᵐ` — linear bias (derived from residual; differs between PGS and
  noslip contexts)
- `d ∈ ℝᵐ` — friction coefficients (mu array)
- `r ∈ ℝ` — cone radius (normal force)

### Scaling to Unit Sphere

Variable substitution `yᵢ = xᵢ / dᵢ` transforms the ellipsoidal constraint
to a spherical one:

```
min   ½ y'A_s y + b_s'y
s.t.  ||y||² ≤ r²
```

Where `A_s[i,j] = A[i,j] · dᵢ · dⱼ` and `b_s[i] = b[i] · dᵢ`.

### KKT Conditions

The Lagrangian: `L(y, λ) = ½ y'A_s y + b_s'y + λ(||y||² - r²)`

Stationarity: `(A_s + λI)y = -b_s`
Primal feasibility: `||y||² ≤ r²`
Dual feasibility: `λ ≥ 0`
Complementarity: `λ(||y||² - r²) = 0`

### Newton Iteration on Dual λ

Define the constraint function:

```
φ(λ) = ||y(λ)||² - r²
```

where `y(λ) = -(A_s + λI)⁻¹ b_s`.

The derivative:

```
φ'(λ) = -2 · y(λ)' · (A_s + λI)⁻¹ · y(λ)
```

Newton update: `Δλ = -φ(λ) / φ'(λ)`

**Convergence criteria (MuJoCo-specific):**
1. `φ(λ) < 1e-10` — converged (inside or on boundary). Note: one-sided
   check. Negative φ means inside cone; MuJoCo accepts this without further
   iteration.
2. `Δλ < 1e-10` — step too small.
3. `det(A_s + λI) < 1e-10` — degenerate, return zero vector.
4. Maximum 20 iterations.

**λ is NOT clamped.** MuJoCo starts at `λ = 0` and adds `delta` directly.
In practice, λ increases monotonically because φ > 0 (outside cone) and
φ' < 0 (since `(A_s + λI)⁻¹` is positive definite), making δ > 0.

### PGS-Specific Bias Construction

For PGS Phase 2, the bias `bc` is constructed from the full-block residual
and the Phase 1 normal force change:

```
bc[j] = res[j+1] - Σₖ Ac[j,k] · oldforce[1+k] + AR[(i+j+1), i] · (force[i] - oldforce[0])
```

This accounts for: the full residual at friction rows, minus the
friction-friction coupling at old forces, plus the cross-coupling from
the normal force change during Phase 1.

### Noslip-Specific Bias Construction

For noslip, no Phase 1 ray update exists (normal force is fixed). The bias is:

```
bc[j] = res[j] - Σₖ Ac[j,k] · oldforce[k]
```

Where `res[j]` is the full residual at friction row j computed with the
unregularized Delassus matrix, and `oldforce[k]` is the friction force
before update.

---

## Conformance Gap Table

Every difference between CortenForge's current behavior and MuJoCo's, with
the fix and AC that verifies it.

| Gap | Current CortenForge | MuJoCo Behavior | Fix | AC |
|-----|--------------------|-----------------|----|-----|
| G1 | PGS elliptic: per-row scalar GS + simple scaling (`project_elliptic_cone` at pgs.rs:148) | Two-phase ray update + QCQP on friction subblock (`mj_solPGS` elliptic branch) | S2: Replace PGS elliptic branch | AC4 |
| G2 | QCQP clamps `lam = lam.max(0.0)` (noslip.rs:137, 248) | No clamping: `lambda += delta` | S1: Remove `.max(0.0)` | AC1, AC2 |
| G3 | QCQP convergence: `phi.abs() < 1e-10` (noslip.rs:123, 234) | `val < 1e-10` (one-sided, not absolute value) | S1: Change to `val < CONV_TOL` | AC1, AC2 |
| G4 | QCQP SPD check: `det.abs() < MJ_MINVAL` (1e-15) (noslip.rs:113, 216) | `det < 1e-10` (one-sided, larger threshold) | S1: Change to `det < CONV_TOL` | AC11 |
| G5 | Noslip QCQP bias: uses `f_unc` from scalar GS update (noslip.rs:528-533) | `bc = res - Ac * oldforce` (block formulation) | S3: Rewrite bias construction | AC6 |
| G6 | Noslip condim=6: simple rescaling fallback (noslip.rs:586-603) | `mju_QCQP()` (N-dim Cholesky) | S1+S3: Add `qcqp_nd`, wire into noslip | AC7 |
| G7 | QCQP final rescale to cone boundary (noslip.rs:159-162, 279-285) | No rescale inside QCQP; ellipsoidal rescale done by CALLER only when `flg_active` | S1: Remove final rescale from QCQP functions | AC1, AC2 |
| G8 | PGS elliptic cost guard: full-matrix reconstruction (`pgs_cost_change` pgs.rs:220-255) | AR subblock `Athis` with `costChange()` | S2: Use AR subblock directly (numerically equivalent, cleaner) | AC4 |
| G9 | QCQP no step guard on delta (only `dphi.abs() < MJ_MINVAL` at noslip.rs:133, 244) | `delta < 1e-10` → break | S1: Add `delta < CONV_TOL` guard, remove `dphi.abs()` guard | AC1, AC2 |
| G10 | QCQP final solve outside loop + degenerate rescaling fallback (noslip.rs:140-165, 251-285) | Loop values reused directly; degenerate returns zero | S1: Remove final solve and fallback; reuse loop values | AC1, AC2, AC11 |
| G11 | Noslip pyramidal cost rollback: `MJ_MINVAL` = 1e-15 (noslip.rs:689) | `1e-10` (`costChange` threshold) | S4: Change threshold to `1e-10` | AC8 |
| G12 | Noslip missing iter-0 cost correction (noslip.rs:478 area) | `improvement += 0.5 * force[j]² * R[j]` at iter 0 | S4: Add iter-0 correction | AC8 |

---

## Architecture Decisions

### AD-1: QCQP Function Location

**Problem:** Where should the new QCQP functions live? They are used by both
PGS (S2) and noslip (S3).

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Keep in `noslip.rs` | Minimal file changes | Misleading: QCQP is not noslip-specific |
| 2 | New `qcqp.rs` module in `solver/` | Clean separation, clear ownership | One more file |
| 3 | Keep in `noslip.rs`, re-export from `solver/mod.rs` | Minimal change, accessible | Still misleading location |

**Chosen:** Option 2 — new `solver/qcqp.rs` module. The QCQP functions are
general-purpose friction cone projection utilities used by multiple solvers.
A dedicated module makes the architecture clear and matches MuJoCo's
`engine_util_solve.c` organization (QCQP functions are utility functions,
not part of any specific solver).

### AD-2: PGS Cost Guard Implementation

**Problem:** PGS cost guard for elliptic blocks currently reconstructs the
residual from the full matrix. MuJoCo uses the AR subblock directly.

**Chosen:** Use the AR subblock directly. Extract the dim×dim subblock at
the start of the elliptic branch (needed for ray update anyway), save the
residual before update, and compute `costChange` using the subblock. This
matches MuJoCo's approach exactly and avoids the complex residual
reconstruction in `pgs_cost_change`.

---

## Specification

### S1. QCQP Utility Functions

**File:** `core/src/constraint/solver/qcqp.rs` (new file)
**MuJoCo equivalent:** `mju_QCQP2()`, `mju_QCQP3()`, `mju_QCQP()` in
`engine_util_solve.c`
**Design decision:** AD-1 — new module for general-purpose QCQP utilities.
Each function matches its MuJoCo counterpart line-for-line. The functions
solve `min 0.5*x'*A*x + b'*x s.t. cone` and return `(result, active)`.
They do NOT perform ellipsoidal rescaling — that is done by the caller
(PGS/noslip) when `active == true`.

**New file: `solver/qcqp.rs`**

```rust
//! QCQP solvers for friction cone projection.
//!
//! Matches MuJoCo's `mju_QCQP2/3()` in `engine_util_solve.c` and
//! `mju_QCQP()` for N-dimensional problems.
//!
//! All functions solve: min 0.5*x'*A*x + b'*x  s.t.  Σ(x_i/d_i)² ≤ r²
//! Returns (result, active) where active = constraint was projected to boundary.

/// Convergence tolerance for QCQP Newton iteration.
/// Matches MuJoCo's hardcoded `1e-10` threshold.
const QCQP_TOL: f64 = 1e-10;

/// Maximum Newton iterations. Matches MuJoCo.
const QCQP_MAX_ITER: usize = 20;

/// 2D QCQP solver (condim=3, 2 friction DOFs).
///
/// Matches `mju_QCQP2` in `engine_util_solve.c`.
///
/// `a`: 2×2 Delassus subblock (row-major as [[f64; 2]; 2]).
/// `b`: 2-element bias vector.
/// `d`: 2-element friction coefficients (mu).
/// `r`: cone radius (normal force).
#[allow(clippy::many_single_char_names)]
pub fn qcqp2(a: [[f64; 2]; 2], b: [f64; 2], d: [f64; 2], r: f64) -> ([f64; 2], bool) {
    // Scale to unit sphere
    let bs = [b[0] * d[0], b[1] * d[1]];
    let a_s = [
        [a[0][0] * d[0] * d[0], a[0][1] * d[0] * d[1]],
        [a[1][0] * d[1] * d[0], a[1][1] * d[1] * d[1]],
    ];
    let r2 = r * r;

    // Newton iteration on dual λ
    let mut lam = 0.0_f64;
    let mut v0 = 0.0_f64;
    let mut v1 = 0.0_f64;

    for _ in 0..QCQP_MAX_ITER {
        let m00 = a_s[0][0] + lam;
        let m11 = a_s[1][1] + lam;
        let m01 = a_s[0][1];
        let det = m00 * m11 - m01 * m01;
        if det < QCQP_TOL {
            return ([0.0, 0.0], false);
        }
        let idet = 1.0 / det;

        // y = -(A_s + λI)⁻¹ * b_s
        v0 = -(m11 * bs[0] - m01 * bs[1]) * idet;
        v1 = -(-m01 * bs[0] + m00 * bs[1]) * idet;

        let val = v0 * v0 + v1 * v1 - r2;
        if val < QCQP_TOL {
            break;
        }

        // φ'(λ) = -2 * y' * (A_s+λI)⁻¹ * y
        let pv0 = (m11 * v0 - m01 * v1) * idet;
        let pv1 = (-m01 * v0 + m00 * v1) * idet;
        let deriv = -2.0 * (v0 * pv0 + v1 * pv1);

        let delta = -val / deriv;
        if delta < QCQP_TOL {
            break;
        }
        lam += delta;
    }

    // Unscale: x_i = y_i * d_i
    ([v0 * d[0], v1 * d[1]], lam != 0.0)
}

/// 3D QCQP solver (condim=4, 3 friction DOFs).
///
/// Matches `mju_QCQP3` in `engine_util_solve.c`. Uses 3×3 cofactor inverse.
#[allow(clippy::many_single_char_names)]
pub fn qcqp3(a: [[f64; 3]; 3], b: [f64; 3], d: [f64; 3], r: f64) -> ([f64; 3], bool) {
    // Scale to unit sphere
    let mut bs = [0.0_f64; 3];
    let mut a_s = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        bs[i] = b[i] * d[i];
        for j in 0..3 {
            a_s[i][j] = a[i][j] * d[i] * d[j];
        }
    }
    let r2 = r * r;

    let mut lam = 0.0_f64;
    let mut v = [0.0_f64; 3];

    for _ in 0..QCQP_MAX_ITER {
        // M = A_s + λI
        let mut m = a_s;
        m[0][0] += lam;
        m[1][1] += lam;
        m[2][2] += lam;

        // 3×3 cofactor inverse
        let cof00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        let cof01 = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]);
        let cof02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        let det = m[0][0] * cof00 + m[0][1] * cof01 + m[0][2] * cof02;
        if det < QCQP_TOL {
            return ([0.0, 0.0, 0.0], false);
        }
        let idet = 1.0 / det;

        let cof10 = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]);
        let cof11 = m[0][0] * m[2][2] - m[0][2] * m[2][0];
        let cof12 = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]);
        let cof20 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
        let cof21 = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]);
        let cof22 = m[0][0] * m[1][1] - m[0][1] * m[1][0];

        // y = -(M⁻¹) * b_s  (cofactor inverse is transposed)
        v[0] = -(cof00 * bs[0] + cof10 * bs[1] + cof20 * bs[2]) * idet;
        v[1] = -(cof01 * bs[0] + cof11 * bs[1] + cof21 * bs[2]) * idet;
        v[2] = -(cof02 * bs[0] + cof12 * bs[1] + cof22 * bs[2]) * idet;

        let val = v[0] * v[0] + v[1] * v[1] + v[2] * v[2] - r2;
        if val < QCQP_TOL {
            break;
        }

        // φ'(λ) = -2 * y' * M⁻¹ * y
        let pv0 = (cof00 * v[0] + cof10 * v[1] + cof20 * v[2]) * idet;
        let pv1 = (cof01 * v[0] + cof11 * v[1] + cof21 * v[2]) * idet;
        let pv2 = (cof02 * v[0] + cof12 * v[1] + cof22 * v[2]) * idet;
        let deriv = -2.0 * (v[0] * pv0 + v[1] * pv1 + v[2] * pv2);

        let delta = -val / deriv;
        if delta < QCQP_TOL {
            break;
        }
        lam += delta;
    }

    ([v[0] * d[0], v[1] * d[1], v[2] * d[2]], lam != 0.0)
}

/// N-dimensional QCQP solver (condim=6, up to 5 friction DOFs).
///
/// Matches `mju_QCQP` in `engine_util_solve.c`. Uses Cholesky factorization.
///
/// `a`: n×n Delassus subblock (flat row-major, length n*n).
/// `b`: n-element bias vector.
/// `d`: n-element friction coefficients.
/// `r`: cone radius.
/// `n`: dimension (number of friction DOFs).
pub fn qcqp_nd(a: &[f64], b: &[f64], d: &[f64], r: f64, n: usize) -> (Vec<f64>, bool) {
    // Scale to unit sphere
    let mut bs = vec![0.0_f64; n];
    let mut a_s = vec![0.0_f64; n * n];
    for i in 0..n {
        bs[i] = b[i] * d[i];
        for j in 0..n {
            a_s[i * n + j] = a[i * n + j] * d[i] * d[j];
        }
    }
    let r2 = r * r;

    let mut lam = 0.0_f64;
    let mut v = vec![0.0_f64; n];

    for _ in 0..QCQP_MAX_ITER {
        // M = A_s + λI
        let mut m_data = a_s.clone();
        for i in 0..n {
            m_data[i * n + i] += lam;
        }

        // Cholesky factorization: M = L * L^T
        let mut l = vec![0.0_f64; n * n];
        let mut spd = true;
        for i in 0..n {
            for j in 0..=i {
                let mut sum = m_data[i * n + j];
                for k in 0..j {
                    sum -= l[i * n + k] * l[j * n + k];
                }
                if i == j {
                    if sum < QCQP_TOL {
                        spd = false;
                        break;
                    }
                    l[i * n + j] = sum.sqrt();
                } else {
                    l[i * n + j] = sum / l[j * n + j];
                }
            }
            if !spd {
                break;
            }
        }
        if !spd {
            return (vec![0.0; n], false);
        }

        // Solve L * z = -b_s (forward substitution)
        let mut z = vec![0.0_f64; n];
        for i in 0..n {
            let mut sum = -bs[i];
            for k in 0..i {
                sum -= l[i * n + k] * z[k];
            }
            z[i] = sum / l[i * n + i];
        }

        // Solve L^T * y = z (back substitution)
        for i in (0..n).rev() {
            let mut sum = z[i];
            for k in (i + 1)..n {
                sum -= l[k * n + i] * v[k];
            }
            v[i] = sum / l[i * n + i];
        }

        let val: f64 = v.iter().map(|yi| yi * yi).sum::<f64>() - r2;
        if val < QCQP_TOL {
            break;
        }

        // P*v for derivative: solve L*L^T * pv = v
        let mut pz = vec![0.0_f64; n];
        for i in 0..n {
            let mut sum = v[i];
            for k in 0..i {
                sum -= l[i * n + k] * pz[k];
            }
            pz[i] = sum / l[i * n + i];
        }
        let mut pv = vec![0.0_f64; n];
        for i in (0..n).rev() {
            let mut sum = pz[i];
            for k in (i + 1)..n {
                sum -= l[k * n + i] * pv[k];
            }
            pv[i] = sum / l[i * n + i];
        }

        let deriv: f64 = -2.0 * v.iter().zip(pv.iter()).map(|(vi, pi)| vi * pi).sum::<f64>();

        let delta = -val / deriv;
        if delta < QCQP_TOL {
            break;
        }
        lam += delta;
    }

    // Unscale
    let result: Vec<f64> = v.iter().zip(d.iter()).map(|(yi, di)| yi * di).collect();
    (result, lam != 0.0)
}
```

**Module registration in `solver/mod.rs`:**

```rust
pub mod qcqp;
```

### S2. PGS Two-Phase Elliptic Projection

**File:** `core/src/constraint/solver/pgs.rs`, lines 131-160 (replace),
lines 220-255 (remove `pgs_cost_change` — no longer needed for elliptic)
**MuJoCo equivalent:** `mj_solPGS()` elliptic branch in `engine_solver.c`
**Design decision:** AD-2 — use AR subblock directly for cost guard. The
full `pgs_cost_change` function is removed since the new implementation
computes cost change inline with the AR subblock. The scalar cost guard
(pgs.rs:196-204) remains unchanged.

**Replace the elliptic branch (pgs.rs:131-160) with:**

```rust
// Elliptic contacts: two-phase ray update + QCQP
if matches!(ctype, ConstraintType::ContactElliptic) && dim > 1 {
    let mu = data.efc_mu[i];

    // Save old forces (before any update)
    let old_force: Vec<f64> = data.efc_force.as_slice()[i..i + dim].to_vec();

    // Compute full residual for the block
    let mut res = vec![0.0_f64; dim];
    for j in 0..dim {
        res[j] = data.efc_b[i + j];
        for c in 0..nefc {
            res[j] += ar[(i + j, c)] * data.efc_force[c];
        }
    }

    // ---- Phase 1: Ray update ----
    if data.efc_force[i] < MJ_MINVAL {
        // Normal force too small: scalar normal update + clear friction
        data.efc_force[i] -= res[0] * ar_diag_inv[i];
        if data.efc_force[i] < 0.0 {
            data.efc_force[i] = 0.0;
        }
        for j in 1..dim {
            data.efc_force[i + j] = 0.0;
        }
    } else {
        // Ray update: scale along current force direction
        let v: Vec<f64> = data.efc_force.as_slice()[i..i + dim].to_vec();

        // v1 = AR_block * v  (dim×dim subblock)
        let mut v1 = vec![0.0_f64; dim];
        for j in 0..dim {
            for k in 0..dim {
                v1[j] += ar[(i + j, i + k)] * v[k];
            }
        }

        let denom: f64 = v.iter().zip(v1.iter()).map(|(a, b)| a * b).sum();
        if denom >= MJ_MINVAL {
            let x_step = -v.iter().zip(res.iter()).map(|(a, b)| a * b).sum::<f64>() / denom;
            // Guard: normal force stays non-negative
            let x_clamped = if data.efc_force[i] + x_step * v[0] < 0.0 {
                -data.efc_force[i] / v[0]
            } else {
                x_step
            };
            for j in 0..dim {
                data.efc_force[i + j] += x_clamped * v[j];
            }
        }
    }

    // ---- Phase 2: Friction QCQP (normal fixed) ----
    let fn_current = data.efc_force[i];
    let fdim = dim - 1; // number of friction DOFs

    if fn_current < MJ_MINVAL {
        // Zero normal: clear all friction
        for j in 1..dim {
            data.efc_force[i + j] = 0.0;
        }
    } else {
        // Build friction-friction subblock Ac from AR
        // and adjusted bias bc
        let mut ac_flat = vec![0.0_f64; fdim * fdim];
        let mut bc = vec![0.0_f64; fdim];
        for j in 0..fdim {
            for k in 0..fdim {
                ac_flat[j * fdim + k] = ar[(i + 1 + j, i + 1 + k)];
            }
            // bc[j] = res[j+1] - Σ Ac[j,k]*old_friction[k] + AR[i+1+j, i] * Δnormal
            bc[j] = res[j + 1];
            for k in 0..fdim {
                bc[j] -= ac_flat[j * fdim + k] * old_force[1 + k];
            }
            bc[j] += ar[(i + 1 + j, i)] * (fn_current - old_force[0]);
        }

        // QCQP dispatch by dimension
        let (fric_result, active) = if fdim == 2 {
            let ac2 = [
                [ac_flat[0], ac_flat[1]],
                [ac_flat[2], ac_flat[3]],
            ];
            let (r, a) = crate::constraint::solver::qcqp::qcqp2(
                ac2, [bc[0], bc[1]], [mu[0], mu[1]], fn_current,
            );
            (r.to_vec(), a)
        } else if fdim == 3 {
            let ac3 = [
                [ac_flat[0], ac_flat[1], ac_flat[2]],
                [ac_flat[3], ac_flat[4], ac_flat[5]],
                [ac_flat[6], ac_flat[7], ac_flat[8]],
            ];
            let (r, a) = crate::constraint::solver::qcqp::qcqp3(
                ac3, [bc[0], bc[1], bc[2]], [mu[0], mu[1], mu[2]], fn_current,
            );
            (r.to_vec(), a)
        } else {
            crate::constraint::solver::qcqp::qcqp_nd(
                &ac_flat, &bc, &mu[..fdim], fn_current, fdim,
            )
        };

        // Ellipsoidal rescale if constraint was projected to boundary
        if active {
            let mut ssq = 0.0_f64;
            for j in 0..fdim {
                if mu[j] > MJ_MINVAL {
                    ssq += (fric_result[j] / mu[j]).powi(2);
                }
            }
            let s = (fn_current * fn_current / ssq.max(MJ_MINVAL)).sqrt();
            for j in 0..fdim {
                data.efc_force[i + 1 + j] = fric_result[j] * s;
            }
        } else {
            for j in 0..fdim {
                data.efc_force[i + 1 + j] = fric_result[j];
            }
        }
    }

    // ---- Cost guard using AR subblock ----
    {
        let mut cost_change = 0.0_f64;
        for j in 0..dim {
            let delta_j = data.efc_force[i + j] - old_force[j];
            cost_change += delta_j * res[j];
            for k in 0..dim {
                let delta_k = data.efc_force[i + k] - old_force[k];
                cost_change += 0.5 * delta_j * ar[(i + j, i + k)] * delta_k;
            }
        }
        if cost_change > 1e-10 {
            // Revert to old forces
            data.efc_force.as_mut_slice()[i..i + dim].copy_from_slice(&old_force);
        }
    }

    i += dim;
}
```

**Also:**
- Remove `use crate::constraint::solver::noslip::project_elliptic_cone;`
  from pgs.rs imports (line 13).
- Add `use crate::constraint::solver::qcqp;` (or use full path as shown).
- Remove `pgs_cost_change` function (pgs.rs:220-255) — it is no longer
  called from anywhere. The scalar cost guard (pgs.rs:196-204) remains.

### S3. Noslip Elliptic QCQP Rewrite

**File:** `core/src/constraint/solver/noslip.rs`, Phase B (lines 498-618)
**MuJoCo equivalent:** `mj_solNoSlip()` elliptic branch in `engine_solver.c`
**Design decision:** Replace the `f_unc`-based formulation with MuJoCo's
`bc = res - Ac * old` formulation. Use the new QCQP functions from S1.
Add `qcqp_nd` dispatch for condim=6.

**Replace Phase B (noslip.rs: the block inside `while fi < n` for
`NoslipRowKind::EllipticFriction`) with:**

```rust
if fi == group_start {
    let normal_force = data.efc_force[contact_efc_start];
    let mu = data.efc_mu[contact_efc_start];
    let fn_abs = normal_force.abs();

    // Save old friction forces
    let old_forces: Vec<f64> =
        (0..group_len).map(|j| f[group_start + j]).collect();

    // Compute residuals for all friction rows
    let mut residuals: Vec<f64> = Vec::with_capacity(group_len);
    for local_j in 0..group_len {
        let idx = group_start + local_j;
        let mut res = b_eff[idx];
        for k in 0..n {
            res += a_sub[(idx, k)] * f[k];
        }
        residuals.push(res);
    }

    if fn_abs < MJ_MINVAL {
        // Zero normal force: zero all friction
        for local_j in 0..group_len {
            f[group_start + local_j] = 0.0;
        }
    } else {
        // Build friction-friction subblock Ac and bias bc
        let mut ac_flat = vec![0.0_f64; group_len * group_len];
        let mut bc = vec![0.0_f64; group_len];
        for j in 0..group_len {
            for k in 0..group_len {
                ac_flat[j * group_len + k] =
                    a_sub[(group_start + j, group_start + k)];
            }
            // bc[j] = res[j] - Σ Ac[j,k] * old[k]
            bc[j] = residuals[j];
            for k in 0..group_len {
                bc[j] -= ac_flat[j * group_len + k] * old_forces[k];
            }
        }

        // QCQP dispatch
        let (fric_result, active) = if group_len == 2 {
            let ac2 = [
                [ac_flat[0], ac_flat[1]],
                [ac_flat[2], ac_flat[3]],
            ];
            let (r, a) = qcqp::qcqp2(
                ac2, [bc[0], bc[1]], [mu[0], mu[1]], fn_abs,
            );
            (r.to_vec(), a)
        } else if group_len == 3 {
            let ac3 = [
                [ac_flat[0], ac_flat[1], ac_flat[2]],
                [ac_flat[3], ac_flat[4], ac_flat[5]],
                [ac_flat[6], ac_flat[7], ac_flat[8]],
            ];
            let (r, a) = qcqp::qcqp3(
                ac3, [bc[0], bc[1], bc[2]], [mu[0], mu[1], mu[2]], fn_abs,
            );
            (r.to_vec(), a)
        } else {
            qcqp::qcqp_nd(
                &ac_flat, &bc, &mu[..group_len], fn_abs, group_len,
            )
        };

        // Ellipsoidal rescale if constraint active
        if active {
            let mut ssq = 0.0_f64;
            for j in 0..group_len {
                if mu[j] > MJ_MINVAL {
                    ssq += (fric_result[j] / mu[j]).powi(2);
                }
            }
            let s = (fn_abs * fn_abs / ssq.max(MJ_MINVAL)).sqrt();
            for j in 0..group_len {
                f[group_start + j] = fric_result[j] * s;
            }
        } else {
            for j in 0..group_len {
                f[group_start + j] = fric_result[j];
            }
        }
    }

    // Accumulate cost improvement for the whole group
    for local_j in 0..group_len {
        let idx = group_start + local_j;
        let delta = f[idx] - old_forces[local_j];
        improvement -= 0.5 * delta * delta * a_sub[(idx, idx)]
            + delta * residuals[local_j];
    }

    fi += group_len;
    continue;
}
```

**Also:**
- Add `use crate::constraint::solver::qcqp;` to noslip.rs imports.
- The old `noslip_qcqp2` and `noslip_qcqp3` functions (noslip.rs:76-286)
  become dead code. Remove them.
- `project_elliptic_cone` (noslip.rs:15-57) is no longer called from PGS
  (removed in S2). Keep the function for now as it may have future uses;
  mark with `#[allow(dead_code)]` if clippy complains.

### S4. Noslip Convergence and Cost Fixes

**File:** `core/src/constraint/solver/noslip.rs`
**MuJoCo equivalent:** `mj_solNoSlip()` in `engine_solver.c`
**Design decision:** Two targeted fixes that match MuJoCo's exact behavior.

**Fix 1: Pyramidal cost rollback threshold (noslip.rs:689)**

Change from:
```rust
if cost > MJ_MINVAL {
```

To:
```rust
if cost > 1e-10 {
```

This matches MuJoCo's `costChange()` threshold. The current `MJ_MINVAL`
(1e-15) is over-aggressive — it reverts updates on tiny cost increases
between 1e-15 and 1e-10 that MuJoCo would accept.

**Fix 2: Iter-0 cost correction (noslip.rs, inside the iteration loop)**

Add at the start of the iteration loop (after `for _iter in 0..noslip_iter`):

```rust
// Iter-0 cost correction: account for regularization cost removed by
// unregularized noslip pass. Matches MuJoCo's mj_solNoSlip iter==0 branch.
if _iter == 0 {
    for fi in 0..n {
        let row = noslip_rows[fi];
        improvement += 0.5 * f[fi] * f[fi] * data.efc_R[row];
    }
}
```

This requires changing `for _iter in 0..noslip_iter` to
`for iter_num in 0..noslip_iter` and using `iter_num == 0`.

### S5. Verification and Cleanup

**Files:** `pgs.rs`, `hessian.rs`, `primal.rs`, `solver/mod.rs`
**MuJoCo equivalent:** Various
**Design decision:** Verification-only section. No algorithmic changes.

**5a. Verify PGS scalar projection (pgs.rs:161-207):**
The scalar projection matches MuJoCo:
- Equality: no projection (correct)
- FrictionLoss: box clamp [-floss, +floss] (correct)
- Limits, frictionless, pyramidal: unilateral max(0, f) (correct)
- Scalar cost guard: `0.5*δ²*AR[i,i] + δ*res` with threshold 1e-10 (correct)
No changes needed.

**5b. Verify CG/Newton primal classifier (pgs.rs:272-488):**
The three-zone classifier for elliptic contacts:
- Top (Satisfied): `μ·N ≥ T` (correct)
- Bottom (Quadratic): `μ·N + T ≤ 0` (correct)
- Middle (Cone): KKT-derived forces (correct)
No changes needed.

**5c. Verify cone Hessian (hessian.rs):**
The cone Hessian is used by Newton only. Its correctness is orthogonal to
the QCQP projection (PGS/noslip only). No changes needed.

**5d. Module registration (solver/mod.rs):**
Add `pub mod qcqp;` to the solver module.

**5e. Remove dead code:**
- Remove `pgs_cost_change` function from pgs.rs (no longer called)
- Remove `noslip_qcqp2` and `noslip_qcqp3` from noslip.rs (replaced by S1)
- Remove `use crate::constraint::solver::noslip::project_elliptic_cone;`
  from pgs.rs
- Add `#[allow(dead_code)]` to `project_elliptic_cone` in noslip.rs if
  clippy flags it

---

## Acceptance Criteria

### AC1: QCQP2 matches MuJoCo for specific input *(runtime test, analytically derived)*
**Given:** `A = [[1.0, 0.0], [0.0, 1.0]]` (identity), `b = [-2.0, -2.0]`,
`d = [1.0, 1.0]`, `r = 1.0`
**After:** `qcqp2(A, b, d, r)`
**Assert:** `active == true`. Result = `[1/√2, 1/√2]` ≈ `[0.7071067811865476,
0.7071067811865476]` within `1e-10`. Cone constraint exactly satisfied:
`||x||² = 1.0`.
**Derivation:** A_s = I, b_s = [-2, -2]. Optimal λ = ||b_s||/r - 1 = 2√2 - 1
≈ 1.8284. y = -b_s/(1+λ) = [2, 2]/2√2 = [1/√2, 1/√2]. x = y·d = y.
**Field:** Return value of `qcqp2`

### AC2: QCQP3 matches MuJoCo for specific input *(runtime test, analytically derived)*
**Given:** `A = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]`
(identity), `b = [-3.0, -4.0, 0.0]`, `d = [1.0, 1.0, 1.0]`, `r = 1.0`
**After:** `qcqp3(A, b, d, r)`
**Assert:** `active == true`. Result = `[3/5, 4/5, 0]` = `[0.6, 0.8, 0.0]`
within `1e-10`. Cone constraint exactly satisfied: `||x||² = 1.0`.
**Derivation:** Optimal λ = ||b_s||/r - 1 = 5 - 1 = 4. y = [3, 4, 0]/5.
**Field:** Return value of `qcqp3`

### AC3: QCQP_ND for condim=6 (5D) *(runtime test, analytically derived)*
**Given:** `A` = 5×5 identity matrix, `b = [-1, -1, -1, -1, -1]`,
`d = [0.5, 0.5, 0.5, 0.5, 0.5]`, `r = 0.5`
**After:** `qcqp_nd(A, b, d, r, 5)`
**Assert:** `active == true`. All 5 elements = `√5/20` ≈ `0.11180339887`
within `1e-10`. Cone constraint: `Σ(x_i/0.5)² = 5·(√5/10)² = 0.25 = r²`.
**Derivation:** A_s = 0.25·I, b_s = [-0.5; 5]. Optimal λ = √5 - 0.25.
y_i = 0.5/√5. x_i = y_i·0.5 = 0.25/√5 = √5/20.
**Field:** Return value of `qcqp_nd`

### AC4: PGS with elliptic contact produces stable resting forces *(runtime test, stability)*
**Given:** Sphere (mass=1.0) on plane, gravity=-9.81, `solver="PGS"`,
`cone="elliptic"`, condim=3, friction=0.5, 500 steps at dt=0.001
**After:** 500 `mj_step()` calls
**Assert:** Ball z-position in [0.08, 0.15], z-acceleration < 2.0.
Normal force > 0 at rest. Friction forces satisfy cone constraint
`Σ(f_j/μ_j)² ≤ f_normal²`.
**Field:** `data.qpos[2]`, `data.qacc[2]`, `data.efc_force`
**Note:** AC4 is a stability/plausibility check (does the two-phase algorithm
produce physically reasonable behavior?). Exact numeric conformance against
MuJoCo is covered by AC15/T19 which compares `efc_force` element-by-element.

### AC5: PGS ray update preserves non-negative normal force *(runtime test)*
**Case A (cold start):**
**Given:** Sphere (mass=1.0) on plane, gravity=-9.81, `solver="PGS"`,
`cone="elliptic"`, condim=3, friction=0.5, `iterations="1"` (single PGS
pass). Cold start (zero initial forces).
**After:** 1 step (produces contacts, runs 1 PGS iteration)
**Assert:** For every ContactElliptic row, `efc_force[normal_row] >= 0.0`.

**Case B (adversarial warmstart):**
**Given:** Same model, but warmstart `efc_force` manually set to adversarial
values: normal row = `1e-16` (below `MJ_MINVAL = 1e-15`), friction rows =
`[100.0, 100.0]` (grossly violating cone). Single PGS iteration.
**Assert:** Normal force ≥ 0.0 after PGS iteration. The `force[i] < MINVAL`
branch fires (scalar normal update + friction zeroed). This directly
exercises the guard path that prevents negative normal force.
**Field:** `data.efc_force` at normal and friction rows

### AC6: Noslip QCQP produces correct bias construction *(runtime test)*
**Given:** Sphere on tilted plane (15°), `cone="elliptic"`, condim=3,
friction=0.5, PGS solver, noslip_iterations=50
**After:** 1 step
**Assert:**
1. Friction forces satisfy cone constraint `Σ(f_j/μ_j)² ≤ f_n²` within 1e-6.
2. Forces are finite and nonzero (tilted plane generates tangential load).
3. Tangential force direction opposes gravity component along tilted plane
   (sign check: friction forces push uphill).
**Field:** `data.efc_force`
**Note:** Noslip runs after PGS. The overall force conformance (noslip output)
is validated by T19/AC15 which compares final `efc_force` against MuJoCo.
AC6 specifically tests that the new bias construction `bc = res - Ac*old`
produces correct friction directions on a tilted surface (old bias used
`f_unc` from scalar GS, which could produce wrong direction).

### AC7: Noslip condim=6 uses QCQP not simple rescaling *(code review)*
**Assert:** Noslip Phase B code dispatches to `qcqp::qcqp_nd` for
`group_len > 3` (condim=6, 5 friction DOFs). No simple rescaling fallback
exists in the noslip elliptic code path.

### AC8: Noslip pyramidal cost threshold is 1e-10 *(code review + runtime test)*
**Assert (code review):** The cost rollback check in noslip Phase C uses
`cost > 1e-10`, not `cost > MJ_MINVAL`.
**Case A (basic runtime):**
**Given:** Sphere (mass=1.0) on plane, `solver="PGS"`, `cone="pyramidal"`,
condim=3, friction=0.5, `noslip_iterations="50"`. 1 step.
**Assert:** Noslip produces finite forces. All pyramidal rows have
`efc_force >= 0.0`. Existing test `test_noslip_pyramidal_reduces_slip`
must still pass.
**Case B (threshold discrimination):**
Construct or instrument a unit test on the noslip pyramidal inner loop where
a force update produces a cost increase of `5e-12` (between 1e-15 and 1e-10).
**Assert:** Under the new threshold (1e-10), the update is **accepted** (cost
increase is below threshold). Under the old threshold (1e-15), the update
would have been **reverted**. This verifies the threshold change has observable
effect. *(Implementation note: may require injecting a mock cost value or
constructing a near-degenerate 2×2 block that produces this cost magnitude.)*

### AC9: CG/Newton elliptic behavior unchanged *(runtime test, regression)*
**Given:** Sphere on plane, `solver="Newton"`, `cone="elliptic"`, condim=3.
10 steps.
**After:** 10 steps
**Assert:**
1. ContactElliptic rows emitted, dim=3. No ContactPyramidal rows.
2. `efc_force` values are **bitwise identical** to pre-Spec-B baseline
   (Spec B does not modify CG/Newton code, so results must be numerically
   unchanged). Capture baseline values before implementation; assert exact
   match within 0.0 tolerance.
**Field:** `data.efc_type`, `data.efc_dim`, `data.efc_force`

### AC10: No regression on existing solver tests *(regression)*
**Assert:** All existing tests in `unified_solvers.rs`, `newton_solver.rs`,
`noslip.rs`, `cg_solver.rs` continue to pass. Tests that use PGS with
elliptic contacts may have numerically different force values (moving toward
MuJoCo conformance) but must still satisfy their assertions.

### AC11: QCQP degenerate input returns zero *(runtime test)*
**Given:** `A = [[0.0, 0.0], [0.0, 0.0]]` (singular), `b = [1.0, 1.0]`,
`d = [0.5, 0.5]`, `r = 1.0`
**After:** `qcqp2(A, b, d, r)`
**Assert:** Returns `([0.0, 0.0], false)`. No rescaling fallback.
**Field:** Return value of `qcqp2`

### AC12: QCQP unconstrained solution inside cone *(runtime test)*
**Given:** `A = [[10.0, 0.0], [0.0, 10.0]]`, `b = [-1.0, -1.0]`,
`d = [1.0, 1.0]`, `r = 10.0` (very large cone)
**After:** `qcqp2(A, b, d, r)`
**Assert:** Unconstrained solution `x = [0.1, 0.1]`, `||x||² = 0.02 < 100`.
Returns `active == false` (no projection). Result equals unconstrained
solution.
**Field:** Return value of `qcqp2`

### AC13: Noslip iter-0 cost correction present *(code review)*
**Assert:** The noslip iteration loop adds `0.5 * f[fi]² * efc_R[row]` to
`improvement` at iteration 0 for all noslip rows.

### AC14: `project_elliptic_cone` no longer called from PGS *(code review)*
**Assert:** No `use` or call of `project_elliptic_cone` in pgs.rs. The
function remains in noslip.rs with `#[allow(dead_code)]` if needed.

### AC15: PGS elliptic forces match MuJoCo reference output *(runtime test, conformance)*
**Given:** Sphere (mass=1.0, radius=0.1) on plane, gravity=-9.81,
`solver="PGS"`, `cone="elliptic"`, condim=3, friction=0.5,
`solver_iterations="100"`, dt=0.002. Single step from default initial state.
**Reference:** Run the identical model through MuJoCo C (`mj_step`) and
extract `efc_force` for all contact rows at step 1. Record these as
`MUJOCO_REF_FORCE[0..dim-1]`.
**Assert:** CortenForge `efc_force` matches MuJoCo reference within 1e-8
for each element. This is the gold-standard direct conformance check.
**Implementation note:** The reference values MUST be extracted from MuJoCo C
during Session 10 (implementation) and embedded as literal constants in the
test. The test structure and model are fully specified here; only the expected
numeric values await extraction.
**Field:** `data.efc_force` vs MuJoCo `d->efc_force`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (QCQP2 conformance) | T1, T12, T13, T18 | Direct + edge cases (asymmetric mu, iteration exhaustion, step guard) |
| AC2 (QCQP3 conformance) | T2, T22, T23 | Direct + degenerate + inside cone |
| AC3 (QCQP_ND condim=6) | T3, T24 | Direct + degenerate |
| AC4 (PGS elliptic stable) | T4 | Direct |
| AC5 (PGS ray non-negative) | T5, T17 | Direct + edge cases (cold start, adversarial warmstart, pure-normal ray) |
| AC6 (noslip bias construction) | T6 | Direct |
| AC7 (noslip condim=6 code) | — | Code review (manual) |
| AC8 (noslip pyramidal threshold) | T7, T14, T25, code review | Direct + edge case (degenerate block) + threshold discrimination + code review |
| AC9 (CG/Newton regression) | T8, T16 | Regression + negative test (QCQP not used) |
| AC10 (no regression) | T9 | Regression |
| AC11 (QCQP degenerate) | T10, T21 | Edge case + zero mu |
| AC12 (QCQP inside cone) | T11, T20 | Edge case + large mu |
| AC13 (iter-0 cost correction) | — | Code review (manual) |
| AC14 (project_elliptic_cone removed from PGS) | T15 | Negative test (pyramidal bypasses QCQP) + code review |
| AC15 (PGS forces match MuJoCo) | T19 | Direct MuJoCo conformance |

**Supplementary tests (not AC-mapped):** T26 (QCQP2 random stress) provides
broad coverage across parameter space for AC1's QCQP2 conformance. T27
(noslip cone regression) validates existing `test_noslip_elliptic_cone_satisfied`
still passes. These are robustness checks beyond the AC framework.

---

## Test Plan

### T1: QCQP2 conformance — constrained case with exact values → AC1

Unit test calling `qcqp2` directly with `A = [[1.0, 0.0], [0.0, 1.0]]`,
`b = [-2.0, -2.0]`, `d = [1.0, 1.0]`, `r = 1.0`.

Verification:
1. Call `qcqp2(A, b, d, r)`.
2. Assert `active == true`.
3. Assert result ≈ `[0.7071067811865476, 0.7071067811865476]` within 1e-10.
4. Assert cone constraint: `x[0]² + x[1]² ≤ 1.0 + 1e-10`.

Expected value: Analytically derived — `[1/√2, 1/√2]`.

### T2: QCQP3 conformance — constrained case with exact values → AC2

Unit test with `A = I₃`, `b = [-3, -4, 0]`, `d = [1, 1, 1]`, `r = 1.0`.

Assert `active == true`. Result ≈ `[0.6, 0.8, 0.0]` within 1e-10.
Cone: `||x||² ≈ 1.0`.

### T3: QCQP_ND 5D — condim=6 with exact values → AC3

`A` = 5×5 identity, `b = [-1; 5]`, `d = [0.5; 5]`, `r = 0.5`.
Assert `active == true`. All elements ≈ `0.11180339887` (= √5/20) within 1e-10.
Cone: `Σ(x_i/0.5)² ≈ 0.25 = r²`.

### T4: PGS elliptic contact — sphere at rest → AC4

Integration test: sphere on plane, PGS solver, elliptic cone, condim=3,
friction=0.5, 500 steps. Assert ball rests near z=0.1, z-acceleration near
zero, forces satisfy cone constraint.

### T5: PGS ray update — non-negative normal → AC5

**Case A (cold start):** PGS ray update never produces negative normal force.
Model: sphere on plane, elliptic, 1 PGS iteration, cold start (zero forces).
After one PGS iteration, all normal rows should have `efc_force >= 0`.

**Case B (adversarial warmstart):** Same model, but manually set warmstart
forces to adversarial values: normal=1e-16 (below MJ_MINVAL=1e-15),
friction=[100, 100]. After one PGS iteration, normal ≥ 0 and friction
zeroed (scalar path fires because `force[i] < MINVAL`).

### T6: Noslip elliptic on tilted plane → AC6

Sphere on 15° tilted plane, elliptic cone, noslip_iterations=50. After 1 step,
friction forces are nonzero (tangential load from tilt) and satisfy cone
constraint.

### T7: Noslip pyramidal finite results → AC8

Model with pyramidal contacts + noslip. Assert finite forces, no NaN/Inf.
Existing test `test_noslip_pyramidal_reduces_slip` should still pass.

### T8: Newton/CG elliptic regression → AC9

Reuse `test_s32_elliptic_no_regression` model. Verify structural behavior
unchanged: ContactElliptic rows present, dim=3, no pyramidal rows.

### T9: Full regression suite → AC10

Run all existing tests: `cargo test -p sim-core -p sim-mjcf -p sim-constraint
-p sim-tendon -p sim-conformance-tests`. All must pass.

### T10: QCQP degenerate input → AC11

Unit test: `qcqp2([[0.0, 0.0], [0.0, 0.0]], [1.0, 1.0], [0.5, 0.5], 1.0)`.
Assert returns `([0.0, 0.0], false)`.

### T11: QCQP unconstrained (inside cone) → AC12

Unit test: `qcqp2([[10.0, 0.0], [0.0, 10.0]], [-1.0, -1.0], [1.0, 1.0], 10.0)`.
Unconstrained solution `[0.1, 0.1]` is inside cone. Assert `active == false`.
Assert result ≈ `[0.1, 0.1]` within 1e-10.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Zero normal force (friction zeroed) | PGS Phase 1 scalar path + Phase 2 clear | T5 | AC5 |
| Degenerate AR subblock (`det < 1e-10`) | QCQP returns zero, no rescaling | T10 | AC11 |
| Unconstrained inside cone (no projection) | λ stays 0, active=false | T11 | AC12 |
| Condim=1 frictionless | Never enters elliptic branch, scalar max(0,f) | T8, T9 | AC9, AC10 |
| Condim=3 (2-DOF QCQP) | Most common elliptic case | T1, T4, T6 | AC1, AC4, AC6 |
| Condim=4 (3-DOF QCQP) | Torsional friction | T2 | AC2 |
| Condim=6 (5-DOF QCQP_ND) | Rolling friction, requires N-dim solver | T3 | AC3, AC7 |
| PGS ray update with pure-normal force | v = [fn, 0, 0], tests ray direction | T17 | AC5 |
| PGS ray update denom < MINVAL | Skip ray update, keep old forces | T5 | AC5 |
| QCQP Newton step guard (delta < 1e-10) | Early exit before max iterations | T18 | AC1 |
| QCQP Newton max iterations (20) | Convergence check after all iters | T13 | AC1 |
| Noslip pyramidal cost threshold | Revert at 1e-10, not 1e-15 | T7 | AC8 |
| Noslip pyramidal near-degenerate 2×2 | Graceful handling when det ≈ 0 | T14 | AC8 |
| Noslip pyramidal threshold discrimination | Verify 1e-10 vs 1e-15 has effect | T25 | AC8 |
| QCQP3 degenerate (zero matrix) | 3D solver graceful failure | T22 | AC2 |
| QCQP3 inside cone | 3D solver unconstrained path | T23 | AC2 |
| QCQP_ND degenerate (5D zero matrix) | N-dim solver graceful failure | T24 | AC3 |
| Noslip iter-0 correction | Regularization cost accounted | code review | AC13 |
| mu ≈ 0 for one direction (asymmetric) | Degenerate cone dimension | T12 | AC1 |
| mu = 0 exactly for one direction | Division by zero in rescale | T21 | AC11 |
| Very large friction coefficient | Cone is very wide, solution inside | T11, T20 | AC12 |
| MuJoCo direct conformance | Gold-standard numeric match | T19 | AC15 |
| Pyramidal contacts bypass QCQP | PGS scalar path only, no QCQP | T15 | AC14 |
| CG/Newton bypass QCQP | Primal classifier, not dual | T16 | AC9 |

### T12: QCQP2 asymmetric mu — degenerate cone axis → AC1

Unit test calling `qcqp2` with `A = [[2.0, 0.0], [0.0, 2.0]]`,
`b = [-3.0, -3.0]`, `d = [1.0, 0.001]`, `r = 1.0`.

The highly asymmetric `d` creates a nearly degenerate ellipsoidal cone
(one axis 1000× shorter). Verify:
1. Result is finite (no NaN/Inf).
2. Cone constraint satisfied: `(x[0]/1.0)² + (x[1]/0.001)² ≤ 1.0 + 1e-8`.
3. `active == true` (unconstrained solution `[1.5, 1.5]` is far outside cone).

### T13: QCQP2 iteration exhaustion (ill-conditioned A) → AC1

Unit test with ill-conditioned `A = [[1.0, 0.999], [0.999, 1.0]]` (condition
number ≈ 2000), `b = [-10.0, -10.0]`, `d = [1.0, 1.0]`, `r = 0.5`.

This forces slow Newton convergence due to the near-singular shifted system.
Verify:
1. Result is finite.
2. Cone constraint satisfied: `||x||² ≤ 0.25 + 1e-8`.
3. If converged, `active == true`. If 20-iteration limit hit, result still
   satisfies feasibility (QCQP returns last iterate, caller applies rescale).

### T14: Noslip pyramidal near-degenerate 2×2 block → AC8

Integration test with pyramidal contacts where the unregularized 2×2 Delassus
subblock has near-zero determinant. Use a model with two nearly-coplanar
contacts (tangent planes differ by < 1°). Verify:
1. Noslip produces finite forces (no NaN/Inf).
2. Non-negative normal forces.
3. Pyramidal friction bounds respected.

### T15: Negative test — pyramidal contacts bypass QCQP → AC14

Integration test with pyramidal-only contacts (`condim=3`, `condtype=0`
pyramidal) using PGS solver. After one solver step:
1. Verify all contact rows are `ConstraintType::ContactPyramidal`.
2. Verify forces are non-negative (scalar `max(0, f)` path).
3. Structural: the PGS elliptic branch (which calls QCQP) is not entered
   because no `ContactElliptic` rows exist. *(Covered implicitly — if pyramidal
   forces are correct and no elliptic rows exist, QCQP was never called.)*

### T16: Negative test — CG/Newton use primal classifier, not QCQP → AC9

Integration test with elliptic contacts using Newton solver (`solver=2`).
After simulation:
1. Verify `ContactElliptic` rows are present in `efc_type`.
2. Verify forces satisfy cone constraint.
3. Verify results match pre-Spec-B baseline within 1e-10 (Newton does not use
   QCQP, so results must be **identical** to current behavior).

This test directly validates AC9: CG/Newton behavior unchanged by Spec B.

### T17: PGS ray update with pure-normal force → AC5

Unit test calling the PGS elliptic branch directly (or via one PGS iteration)
with warmstarted forces: `force = [1.0, 0.0, 0.0]` (non-zero normal, zero
friction). This makes `v = [1.0, 0.0, 0.0]` — a pure-normal ray.

Verify:
1. Ray update: `denom = v' * Athis * v = Athis[0,0]` (positive). Ray step
   `x = -v·res / denom` scales normal only.
2. After Phase 1, normal force updated along ray direction.
3. Phase 2 QCQP receives zero old friction → bias is just `res[1:]` + cross
   term from normal change.
4. Final forces satisfy cone constraint and normal ≥ 0.

### T18: QCQP2 delta step guard (early exit) → AC1

Unit test with inputs that trigger `delta < 1e-10` before convergence or
iteration exhaustion. Use `A = [[1e6, 0.0], [0.0, 1e6]]` (very stiff),
`b = [-1.0, -1.0]`, `d = [1.0, 1.0]`, `r = 0.5`.

Unconstrained solution: `[1e-6, 1e-6]` — well inside cone (r=0.5). The
Newton iteration starts at λ=0, computes φ(0) = 2e-12 - 0.25 < 0 (inside
cone), so it should return `active=false` immediately. Verify:
1. Returns `active == false`.
2. Result ≈ `[1e-6, 1e-6]`.

Alternative: if the above case exits via convergence check rather than step
guard, use `A = [[1.0, 0.0], [0.0, 1.0]]`, `b = [-0.71, -0.71]`,
`d = [1.0, 1.0]`, `r = 1.0`. Solution is near-boundary (`||x|| ≈ 1.004`),
requiring QCQP with λ very close to zero. After 1-2 iterations the step
should become negligible.

### T19: MuJoCo direct conformance — PGS elliptic efc_force → AC15

Integration test: sphere (mass=1.0, radius=0.1) on plane, gravity=-9.81,
`solver="PGS"`, `cone="elliptic"`, condim=3, friction=0.5,
`solver_iterations="100"`, dt=0.002. 1 step.

Reference values: extracted from MuJoCo C `mj_step` on the identical model.
**Placeholder** — exact numeric values to be populated during Session 10
(implementation) by running the model through MuJoCo Python bindings.

Assert: each element of `efc_force` matches MuJoCo reference within 1e-8.

### T20: QCQP2 with very large friction coefficients → AC12

Unit test: `A = [[1.0, 0.0], [0.0, 1.0]]`, `b = [-0.5, -0.5]`,
`d = [100.0, 100.0]`, `r = 1.0`.

Scaled problem: `b_s[i] = b[i]*d_i = -50`, `A_s[i,j] = A[i,j]*d_i*d_j = 10000*I`.
Unconstrained scaled solution: `y = A_s⁻¹ * (-b_s) = [5e-3, 5e-3]`,
`||y||² = 5e-5 << 1.0 = r²`. Solution is trivially inside cone.

Verify:
1. `active == false`.
2. Result: `x_i = y_i * d_i = 5e-3 * 100 = 0.5` → result ≈ `[0.5, 0.5]`.
3. Cone check: `(0.5/100)² + (0.5/100)² = 5e-5 << 1.0 = r²`.

### T21: QCQP2 with zero mu in one direction → AC11

Unit test: `A = [[1.0, 0.0], [0.0, 1.0]]`, `b = [-2.0, -2.0]`,
`d = [1.0, 0.0]`, `r = 1.0`.

`d[1] = 0.0` means the second friction direction has zero cone radius.
The scaled matrix `A_s[1,1] = A[1,1]*0*0 = 0` — singular.
**Expected behavior:** `det(A_s + λI)` involves a zero row/column in the
unshifted matrix. The function should detect `det < 1e-10` at λ=0 and return
`([0.0, 0.0], false)`. This exercises the degenerate path for zero friction.

Verify:
1. Returns `([0.0, 0.0], false)` — zero vector, not active.
2. No NaN/Inf.

### T22: QCQP3 degenerate input → AC2

Unit test: `A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]` (zero matrix),
`b = [1.0, 1.0, 1.0]`, `d = [1.0, 1.0, 1.0]`, `r = 1.0`.

Assert: returns `([0.0, 0.0, 0.0], false)`.

### T23: QCQP3 inside cone → AC2

Unit test: `A = [[10, 0, 0], [0, 10, 0], [0, 0, 10]]`, `b = [-1, -1, -1]`,
`d = [1, 1, 1]`, `r = 10.0`.

Unconstrained: `x = [0.1, 0.1, 0.1]`, `||x||² = 0.03 << 100 = r²`.
Assert: `active == false`, result ≈ `[0.1, 0.1, 0.1]`.

### T24: QCQP_ND degenerate input (5D) → AC3

Unit test: 5×5 zero matrix, `b = [1; 5]`, `d = [1; 5]`, `r = 1.0`.
Assert: returns `([0; 5], false)`.

### T25: Noslip pyramidal threshold discrimination → AC8

Unit test on noslip pyramidal inner loop (or mocked cost function). Construct
a force update that produces a cost increase of `5e-12`:
1. Under new threshold (1e-10): update **accepted** (5e-12 < 1e-10).
2. Verify the force update is retained (not rolled back).
This directly validates that the 1e-15 → 1e-10 threshold change has
observable behavioral effect.

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T26 (QCQP2 symmetric stress) | Multiple random (A, b, d, r) inputs to QCQP2, verify cone constraint satisfaction | Catch rounding bugs across parameter space |
| T27 (noslip cone satisfied) | Existing `test_noslip_elliptic_cone_satisfied` regression | Verify new QCQP still satisfies cone |

### MuJoCo Conformance Note

Direct numeric conformance against MuJoCo C outputs (running identical models
through `mj_step` and comparing `efc_force` vectors element-by-element) is the
gold standard. **T19** is the numbered conformance test. The model and assertion
structure are fully specified; the exact expected numeric values must be
extracted from MuJoCo C during Session 10 (implementation) using MuJoCo Python
bindings on the identical model. This is the only test with deferred expected
values — all other tests have analytically derived or structurally verified
assertions.

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| PGS elliptic force values | Per-row scalar GS + simple scaling | Two-phase ray+QCQP (matches MuJoCo) | **Toward MuJoCo** | PGS solver output, tests checking PGS force values | Update expected values if tests have exact PGS force assertions |
| Noslip elliptic forces | QCQP with `f_unc` bias, condim=6 rescaling | QCQP with `bc=res-Ac*old`, condim=6 Cholesky | **Toward MuJoCo** | Noslip output, tests checking slip reduction | Should still pass — cone constraint preserved |
| Noslip pyramidal cost threshold | Revert at 1e-15 | Revert at 1e-10 | **Toward MuJoCo** | Noslip pyramidal convergence rate | Less aggressive revert → may converge slightly faster |
| Noslip convergence timing | No iter-0 correction | Adds 0.5*f²*R at iter 0 | **Toward MuJoCo** | Noslip convergence check | More accurate improvement metric |
| CG/Newton elliptic | Primal zone classifier | Unchanged | N/A | Nobody | None — transparent |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `core/src/constraint/solver/qcqp.rs` | **New file** — QCQP2/3/ND functions | +250 |
| `core/src/constraint/solver/mod.rs` | Add `pub mod qcqp;` | +1 |
| `core/src/constraint/solver/pgs.rs` | Replace elliptic branch (131-160), remove `pgs_cost_change` (220-255), update imports | ~90 modified, -40 removed |
| `core/src/constraint/solver/noslip.rs` | Rewrite Phase B (498-618), remove `noslip_qcqp2/3` (76-286), fix pyramidal threshold (689), add iter-0 correction (478), update imports | ~80 modified, -210 removed, +10 added |
| `L0/tests/integration/` (new test file or additions) | New QCQP unit tests + PGS/noslip integration tests | +300 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_s32_elliptic_no_regression` | unified_solvers.rs:1702 | **Pass** (unchanged) | Uses Newton solver (primal classifier), not PGS |
| `test_pgs_contact_force_quantitative` | newton_solver.rs:1655 | **Pass** (unchanged) | Uses default cone (pyramidal), not elliptic |
| `test_pgs_cost_guard_dual_cost_nonpositive` | unified_solvers.rs:286 | **Pass** (unchanged) | Tests scalar PGS cost guard, not elliptic |
| `test_pgs_cost_guard_with_friction_loss_and_contacts` | unified_solvers.rs:362 | **Pass** (unchanged) | Tests friction loss + contacts with scalar PGS |
| `test_noslip_elliptic_cone_satisfied` | noslip.rs:682 | **Pass** (assertion is loose: s ≤ |fn| + 1e-6) | New QCQP still satisfies cone constraint |
| `test_pgs_noslip_reduces_slip` | noslip.rs:23 | **Pass** (assertion: slip reduces, not exact values) | New QCQP produces correct friction, slip still reduces |
| `test_noslip_preserves_normal_forces` | noslip.rs:220 | **Pass** (unchanged) | Noslip only modifies friction rows |
| `test_noslip_friction_loss_clamping` | noslip.rs:282 | **Pass** (unchanged) | Tests friction loss rows, not elliptic |
| `test_noslip_pyramidal_reduces_slip` | noslip.rs:359 | **May change numerically** | Pyramidal cost threshold change (1e-15→1e-10) may slightly change convergence |
| `test_noslip_pyramidal_forces_nonnegative` | noslip.rs:407 | **Pass** (unchanged) | Non-negativity preserved |
| `test_noslip_pyramidal_finite_results_all_solvers` | noslip.rs:447 | **Pass** (unchanged) | Finiteness preserved |
| `test_cg_friction_cone` | cg_solver.rs:221 | **Pass** (unchanged) | CG uses primal classifier, not QCQP |
| `test_newton_noslip_regression` | noslip.rs:118 | **May change numerically** | Noslip QCQP changes may produce slightly different forces |
| `test_noslip_processes_all_contacts_regardless_of_state` | noslip.rs:792 | **Pass** | Structural test, not value-dependent |
| `test_s31_solreffriction_affects_newton_and_pgs` | unified_solvers.rs:970 | **Pass** (unchanged) | Tests solreffriction, not cone projection |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `pgs.rs:272-488` | `classify_constraint_states` (primal classifier) | CG/Newton only, not QCQP. Already correct. |
| `hessian.rs` | Cone Hessian for Newton | Orthogonal to QCQP projection. Already correct. |
| `primal.rs` | Primal constraint update | Shared by CG/Newton, not PGS/noslip. |
| `assembly.rs` | Constraint row assembly | Different pipeline stage. Not touched by Spec B. |
| `noslip.rs:621-706` | Phase C pyramidal 2×2 block solve | Algorithm unchanged; only cost threshold fixed (line 689). |
| `noslip.rs:481-496` | Phase A friction loss | Scalar PGS + box clamp. Already correct. |

---

## Execution Order

1. **S1 (QCQP utility functions)** — no dependencies. Create `qcqp.rs` with
   `qcqp2`, `qcqp3`, `qcqp_nd`. Register in `solver/mod.rs`.
   → Run `cargo test` to verify compilation. Add unit tests T1, T2, T3,
   T10, T11, T12, T13, T18, T20, T21, T22, T23, T24, T26. Verify all
   QCQP conformance.

2. **S2 (PGS two-phase)** depends on S1. Replace PGS elliptic branch, remove
   `pgs_cost_change`, update imports.
   → Run domain tests. Add T4, T5, T15, T17, T19. Verify AC4, AC5, AC14,
   AC15. Existing PGS tests must pass.

3. **S3 (Noslip QCQP rewrite)** depends on S1. Rewrite Phase B, remove old
   QCQP functions, update imports.
   → Run domain tests. Add T6, T27. Verify AC6, AC7. Existing noslip tests
   must pass.

4. **S4 (Noslip convergence fixes)** — independent of S2/S3 but logically
   follows. Fix pyramidal threshold, add iter-0 correction.
   → Run domain tests. Add T7, T14, T25. Verify AC8, AC13.

5. **S5 (Verification + cleanup)** depends on S2, S3, S4. Verify PGS scalar,
   CG/Newton primal, cleanup dead code.
   → Run full domain tests. Add T8, T9, T16. Verify AC9, AC10.

---

## Performance Characterization

**PGS elliptic branch:** The new two-phase algorithm performs more work per
contact than the old per-row GS + simple scaling:
- Phase 1 ray update: O(dim²) for AR subblock multiply (was O(dim) for scalar GS)
- Phase 2 QCQP: O(dim³) for 2×2/3×3 inverse (was O(dim) for simple scaling)
- Overall: O(dim³) per contact per PGS iteration

For typical contacts (dim=3): 2×2 QCQP is trivial. For dim=6: 5×5 Cholesky
per Newton iteration (up to 20 iters × 5×5 = 500 scalar ops). This is
negligible compared to the O(nefc²) Delassus matrix computation.

**Noslip:** Same asymptotic cost as before (QCQP2/3 already existed). Adding
QCQP_ND for condim=6 is O(n³) with n=5, negligible.

**Acceptable:** Yes. The per-contact cost increase is bounded by the QCQP
dimension (max 5) and MuJoCo uses the same algorithm. No performance
regression for typical models.

---

## Out of Scope

- **DT-18** (Zero-friction condim downgrade) — Performance optimization, not
  conformance. *Conformance impact: none.*

- **PGS early termination / improvement tracking** (DT-128) — MuJoCo's PGS
  tracks per-iteration improvement and can exit early. CortenForge PGS
  currently runs fixed iterations. Deferred. *Conformance impact: affects
  convergence speed, not final result given enough iterations.*

- **`solreffriction` on contacts** — Per-direction solver params on contact
  friction rows. Different scope from QCQP projection. *Conformance impact:
  affects models with per-contact friction solver tuning.*

- **Spec A items** (DT-23, DT-33) — Solver parameter wiring, tendon margin.
  Independent spec, different pipeline stage. *Conformance impact: covered
  by Spec A.*

- **DT-25 verification** — Deformable-rigid friction. Separate session.
  *Conformance impact: covered by Session 13.*

- **Warmstart improvements** (DT-129) — PGS warmstart selection could use the
  new two-phase projection for better initial forces. Deferred.
  *Conformance impact: affects convergence speed, not final result.*

- **Dense AR matrix optimization** (DT-130) — PGS currently computes the full
  nefc×nefc AR matrix. MuJoCo uses sparse row-level operations. Performance
  optimization, not conformance. *Conformance impact: none.*
