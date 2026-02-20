# Future Work 5 — Quality of Life + Appendix (Items #15–17)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 15. Newton Solver
**Status:** Phase C Complete | **Effort:** XL | **Prerequisites:** None

#### Current State

**Phase C complete.** The Newton solver is fully operational with all polish items
implemented: solver statistics, per-step meaninertia, noslip post-processor, and
sparse Hessian path for large systems.

**Implementation summary:**
- `SolverType::Newton` is the default solver (matching MuJoCo)
- Unified constraint assembly handles equality, friction loss, limits, and contacts
- Exact 1D Newton line search with three-phase bracketing (§15.5)
- Cholesky rank-1 update/downdate for incremental Hessian (Linpack DCHUD/DCHDD)
- Elliptic cone Hessian blocks with `L_cone` copy (§15.7)
- Automatic PGS fallback for: implicit integrators, pyramidal cones, Cholesky failure
- `SolverStat` per-iteration statistics: improvement, gradient, lineslope, nactive, nchange, nline
- Per-step `meaninertia = trace(M) / nv` for configuration-dependent scaling
- Noslip post-processor: PGS on friction rows with elliptic cone projection, no regularization
- Sparse Hessian path: custom CSC LDL^T for `nv > 60` (no external crate dependency)
- 404+ sim domain tests pass (0 failures)

#### Objective
Implement MuJoCo's exact Newton contact solver operating on the **reduced primal
formulation** (Todorov 2014). The solver minimizes a convex cost over
joint-space accelerations `qacc`, with constraint satisfaction encoded implicitly
via a piecewise-quadratic penalty function. Wire `MjcfSolverType::Newton` to
the new solver, eliminating the silent PGS fallback.

#### Background: Formulation

MuJoCo's Newton solver does **not** solve a dual problem over constraint forces
(that's PGS's domain). It solves the **reduced primal** — an unconstrained
optimization in joint-acceleration space:

```
qacc* = argmin_x  (1/2)||x - a_free||²_M  +  s(J·x − a*)
```

Where:
- `a_free = M⁻¹·(τ − c)` is the unconstrained acceleration (applied forces
  minus bias, solved through mass matrix)
- `M` is the joint-space inertia matrix (the norm weight)
- `J` is the constraint Jacobian (stacked: limits, equality, contacts)
- `a* = −b_damp·(J·v) − k_erp·r` is the reference constraint-space
  acceleration (velocity damping + Baumgarte position correction, derived from
  per-constraint `solref`/`solimp`)
- `s(·)` is a convex, C¹ piecewise-quadratic penalty function encoding all
  constraint types (unilateral contacts, friction cones, equality, limits)

The first term is **Gauss's principle of least constraint**: constrained
acceleration deviates minimally from unconstrained acceleration, weighted by M.
The second term penalizes constraint violation with softness controlled by
`solimp` impedance.

Constraint forces are recovered as `f = −∇s(J·qacc − a*)`. Strong duality with
the dual (PGS) formulation guarantees the same unique solution.

#### Specification

> **Note:** Line numbers referenced below are from the pre-implementation
> codebase and may be stale. Use function names and `grep` to locate code.

##### 15.0 Architectural Prerequisite: Unified Constraint Assembly ✅

**Gap:** *(Resolved.)* MuJoCo's Newton solver operates on a **unified constraint
Jacobian** `J` that stacks rows for ALL constraint types (equality, friction
loss, joint limits, tendon limits, contacts). Our architecture handled limits
and equality as penalty forces written directly to `qfrc_constraint` *before*
the contact
solver runs (lines 12006–12119 of `mj_fwd_constraint()`). The contact solver
sees only contact Jacobians.

**Required refactor:** Before the Newton solver can work, `mj_fwd_constraint()`
must be restructured to assemble a unified constraint system:

```
J = [ J_equality     ]   ne rows  (equality constraints — always Quadratic)
    [ J_friction     ]   nf rows  (joint/tendon friction loss — Huber)
    [ J_limit_joint  ]   nl_j rows (active joint limits — unilateral)
    [ J_limit_tendon ]   nl_t rows (active tendon limits — unilateral)
    [ J_contact      ]   nc rows  (contacts — unilateral + friction cone)
```

Total rows: `nefc = ne + nf + nl_j + nl_t + nc`. Here `nc` is the total
number of contact **constraint rows** (= `Σ_k dim_k` over all contacts),
not the number of contacts. E.g., 3 contacts with condim=3 contribute
`nc = 9` rows.

**Row admission criteria** (which constraints get rows in J):
- **Equality constraints**: ALL active equality constraints always get rows.
  MuJoCo creates rows for every equality constraint (they're always Quadratic).
- **Friction loss**: ALL joints/tendons with `frictionloss > 0` get rows.
- **Joint limits**: A joint limit gets a row **only when potentially active**:
  lower limit row if `q ≤ limit_min + margin`, upper limit row if
  `q ≥ limit_max − margin` (where `margin` is geom margin, typically 0 for
  joints). MuJoCo uses `efc_pos > -margin` as the inclusion test. For joints
  with no margin, this simplifies to: include the row when `efc_pos ≥ 0`
  (i.e., the limit is currently violated). Rows that fail the inclusion
  test are NOT emitted — they don't exist in J at all.
  **Important:** At most one limit row per joint is emitted (lower XOR upper),
  since a joint cannot simultaneously violate both limits.
- **Tendon limits**: Same criterion as joint limits, applied to tendon length
  vs tendon limit range.
- **Contacts**: ALL detected contacts (from broad/narrow phase) get rows.
  The Newton solver classifies them as Quadratic or Satisfied based on jar.

Each row type already has an implicit Jacobian in the existing penalty code:
- **Joint limits** (hinge/slide): 1×nv sparse row with a single `±1` at
  `dof_adr` (sign: `+1` for lower limit violation, `−1` for upper)
- **Tendon limits**: 1×nv row = the existing tendon Jacobian `data.ten_J[t]`
- **Equality constraints**: Connect = 3×nv, Weld = 6×nv, Joint = 1×nv,
  Distance = 1×nv — extracted from the existing `apply_equality_constraints()`
  internals which currently compute these Jacobians implicitly
- **Contacts**: dim×nv — the existing `compute_contact_jacobian()`. The
  Newton path reuses the contact detection pipeline (broad + narrow phase)
  and the per-contact Jacobian computation, but does NOT form the full
  Delassus matrix `A = J·M⁻¹·J^T` (which PGS/CG need). Instead, the
  contact Jacobian rows are copied directly into the unified `efc_J`.
  The `diagApprox` computation extracts just the diagonal of A per row
  (§15.1), avoiding the full O(nc²·nv) Delassus assembly.

**Friction loss migration.** ⚠️ **Tracked as [#28](./future_work_8.md)
(Newton Huber) and [#29](./future_work_8.md) (PGS/CG).** The full
specification for migrating friction loss from tanh passive forces to Huber
constraint rows is in future_work_8.md. Summary of the design context that
informed the spec:

- MuJoCo treats `frictionloss` as constraint rows (`mjCNSTR_FRICTION_DOF`,
  `mjCNSTR_FRICTION_TENDON`) with Huber cost — never a passive force.
- Our pipeline diverged: `mj_fwd_passive()` uses `−frictionloss · tanh(qvel ·
  friction_smoothing)`.
- Approach (b) was chosen: `Data.qfrc_frictionloss` accumulator (implemented).
  Newton subtracts it from RHS; PGS/CG uses `qfrc_passive` as-is.
- #28 migrates Newton to constraint rows. #29 migrates PGS/CG and removes
  the tanh path entirely.

**Per-constraint metadata** is stored as flat `Data` fields (§15.11) indexed
by constraint row. During assembly, each constraint row populates these fields:

- `efc_type[i]`: constraint classification (§15.11)
- `efc_solref[i]`: per-constraint `[solref0, solref1]`
- `efc_solimp[i]`: per-constraint 5-element impedance parameters
- `efc_pos[i]`: signed constraint violation. For unilateral constraints
  (limits, contacts), **positive when the constraint is active** (matching
  MuJoCo's `efc_pos` convention):
  - Contact normal: penetration depth (positive = penetrating)
  - Contact friction rows (`j ≥ 1`): `0`
  - Joint lower limit: `limit_min − q` (positive when `q < limit_min`)
  - Joint upper limit: `q − limit_max` (positive when `q > limit_max`)
  - Tendon limits: analogous (`limit − length` or `length − limit`)
  - Equality: per-axis signed error (can be positive or negative)
  - Friction loss: `0`
- `efc_margin[i]`: per-constraint margin (geom margin for contacts, 0 otherwise)
- `efc_vel[i]`: constraint-space velocity `(J·qvel)_i`. Computed during
  assembly for ALL row types: `efc_vel[i] = J_row_i · qvel`. For contacts
  this is the existing contact-space velocity; for limits it's `±qvel[dof]`;
  for equality it's the time-derivative of the constraint error; for friction
  loss it's `qvel[dof]` or `ten_vel[t]`. Used in the `aref` formula (§15.1)
- `efc_floss[i]`: friction loss saturation (`frictionloss` value, 0 if N/A)
- `efc_solref[i]` **source per constraint type:** Equality → `Model.eq_solref[eq_id]`;
  Joint limits → `Model.jnt_solref[jnt_id]`; Tendon limits →
  `Model.tendon_solref[tendon_id]`; Contacts → `Contact.solref` (resolved at
  collision time). **Friction loss** → `DEFAULT_SOLREF` for Phase A (MuJoCo has
  separate `dof_solref_fri`/`dof_solimp_fri` fields; Phase C should add these
  if parity is needed). Same source pattern applies to `efc_solimp[i]`.
  Tracked in [future_work_9b.md](./future_work_9b.md) §DT-23.
- `efc_mu[i]`: 5-element friction coefficients (contacts only, `[0;5]` otherwise)
- `efc_dim[i]`: constraint group size for stepping. For elliptic contacts:
  `dim` for all rows belonging to the same contact group. For everything else
  (equality, limits, friction loss, `ContactNonElliptic`): `1`. Multi-row
  equality constraints (Connect=3, Weld=6) have `efc_dim = 1` per row because
  Newton treats them as independent scalar Quadratic constraints. `efc_dim > 1`
  only indicates elliptic friction cone grouping

##### 15.1 Constraint State Machine ✅

Each scalar constraint row `i` is in one of five states, determined by its
constraint-space residual `jar_i = (J · qacc − aref)_i`:

| State | Condition | Force | Cost | Hessian |
|-------|-----------|-------|------|---------|
| `Quadratic` | Active, quadratic region | `−D_i · jar_i` | `½·D_i·jar_i²` | `D_i` on diagonal |
| `Satisfied` | Inactive (would pull) | `0` | `0` | `0` |
| `LinearPos` | `jar_i ≥ R_i·floss_i` | `−floss_i` | `floss_i·jar_i − ½·R_i·floss_i²` | `0` |
| `LinearNeg` | `jar_i ≤ −R_i·floss_i` | `+floss_i` | `−floss_i·jar_i − ½·R_i·floss_i²` | `0` |
| `Cone` | Elliptic cone middle zone | Coupled (§15.7) | `½·Dm·(N−μT)²` | Block (§15.7) |

**Definitions:**

- `D_i = 1 / R_i` is the inverse regularization (constraint stiffness)

- `R_i = max(mjMINVAL, (1 − imp_i) / imp_i · diagApprox_i)` where `mjMINVAL = 1e-15`

- `imp_i` is the impedance from `solimp = [dmin, dmax, width, midpoint, power]`:
  ```
  // Clamp solimp parameters (matching MuJoCo's getsolparam() and existing compute_impedance())
  dmin     = clamp(solimp[0], mjMINIMP, mjMAXIMP)   // mjMINIMP = 0.0001
  dmax     = clamp(solimp[1], mjMINIMP, mjMAXIMP)    // mjMAXIMP = 0.9999
  width    = solimp[2]
  midpoint = clamp(solimp[3], mjMINIMP, mjMAXIMP)         // mjMINIMP = 0.0001
  power    = max(1.0, solimp[4])                          // MuJoCo enforces power ≥ 1

  // Guard: width ≤ 0 → skip sigmoid, return midpoint
  if width ≤ 1e-10:
      imp_i = (dmin + dmax) / 2
  else:
      x = clamp(|pos_i − margin_i| / width, 0, 1)
      y = smooth_sigmoid(x, midpoint, power)
      imp_i = dmin + y · (dmax − dmin)
  ```

  where `smooth_sigmoid(x, midpoint, power)` is a C¹ piecewise-power sigmoid:
  ```
  fn smooth_sigmoid(x: f64, midpoint: f64, power: f64) -> f64:
      if |power − 1| < 1e-10:  return x            // linear case (ε = 1e-10)
      if x ≤ midpoint:
          a = 1 / midpoint^(power − 1)
          return a · x^power                       // lower half
      else:
          b = 1 / (1 − midpoint)^(power − 1)
          return 1 − b · (1 − x)^power             // upper half
  ```
  The `dmin`/`dmax` clamping prevents `imp = 0` (which would make `R = inf`)
  and `imp = 1` (which would make `R = ε`, `D = 1/ε = 1e10` — extremely
  stiff but numerically valid). The `midpoint`/`power` clamping prevents
  NaN from `0^(negative)` or `1/0` in `smooth_sigmoid` (matching MuJoCo's
  `getsolparam()`). The `width` guard prevents division by zero in the
  sigmoid input. All match the existing `compute_impedance()` implementation
  (lines 11120–11127).

  The margin subtraction matches MuJoCo's `mj_referenceConstraint()` which
  computes impedance on `abs(efc_pos - margin)`. For equality constraints and
  friction rows, `margin = 0` so this simplifies to `|pos_i|`. For contacts,
  `margin = geom_margin` (typically 0 but nonzero when geom margins are set).
  The existing `compute_impedance()` receives raw violation without margin
  subtraction — the Newton path's callers must subtract margin before calling.

- `diagApprox_i` approximates the `i`-th diagonal of `A = J·M⁻¹·J^T` without
  forming the full Delassus matrix. MuJoCo's `mj_diagApprox()` uses
  precomputed per-body scalar weights `body_invweight0[2*body]` (translational)
  and `body_invweight0[2*body+1]` (rotational):
  ```
  // Per constraint type:
  //   Equality Connect: tran_b1 + tran_b2
  //   Equality Weld: tran or rot depending on row index (0-2 → tran, 3-5 → rot)
  //   Joint limit / DOF friction: dof_invweight0[dof]
  //   Tendon limit / friction: tendon_invweight0[tendon]
  //   Elliptic contact: rows 0-2 → tran_b1+tran_b2, rows 3+ → rot_b1+rot_b2
  // where tran_bN = body_invweight0[2*bN], rot_bN = body_invweight0[2*bN+1]
  ```
  This is a cheap O(1)-per-row approximation that avoids any M⁻¹ solves.
  For simple cases (hinge joint limit touching one body):
  `diagApprox ≈ dof_invweight0[dof] ≈ 1 / M_diag[dof]`.

  **Phase A uses the exact diagonal instead** (§15.12 step 2): compute
  `diagApprox_i = J_i · M⁻¹ · J_i^T` via sparse LDL solve, which is
  O(nv) per row. This is more accurate than MuJoCo's approximation at
  higher cost. Phase C may switch to the body-weight approximation for
  large systems where the O(nv · nefc) cost dominates. Implementing the
  body-weight approximation requires adding `body_invweight0`,
  `dof_invweight0`, and `tendon_invweight0` to Model (computed at build
  time from the mass matrix at `qpos0`).
  Tracked in [future_work_9b.md](./future_work_9b.md) §DT-39.

- `floss_i` is the friction loss saturation force (`frictionloss` attribute on
  the joint or tendon, zero for non-friction constraints)

- `aref_i` is the reference constraint-space acceleration:
  ```
  aref_i = −B_i · efc_vel_i − K_i · imp_i · (pos_i − margin_i)
  ```
  Where:
  - `efc_vel_i = (J · qvel)_i` — constraint-space velocity
  - `pos_i` — constraint violation (penetration depth for contacts,
    `q − limit` for joint limits, length error for equality, 0 for friction)
  - `margin_i` — per-constraint margin (geom margin for contacts, 0 otherwise)
  - `K_i, B_i` — stiffness and damping derived from `solref`:
    - Standard mode (`solref[0] > 0`):
      `K = 1 / max(ε, imp_max² · timeconst² · dampratio²)`,
      `B = 2 / max(ε, imp_max · timeconst)`
      where `imp_max = solimp[1]` (= `dmax`), `ε = mjMINVAL = 1e-15`.
      The `max(ε, ...)` guard prevents division by zero when `dampratio = 0`
      or `timeconst → 0` (both are legal `solref` values). This matches
      MuJoCo's `mju_max(mjMINVAL, ...)` in `mj_referenceConstraint()`
    - Direct mode (`solref[0] ≤ 0`):
      `K = −solref[0] / max(mjMINVAL, imp_max²)`,
      `B = −solref[1] / max(mjMINVAL, imp_max)`
      The `max(mjMINVAL, ...)` guard matches MuJoCo's `mju_max(mjMINVAL, ...)`
      in `mj_makeImpedance()`. In practice the guard is redundant since
      `imp_max = dmax` is clamped to `[mjMINIMP, mjMAXIMP]` (denominator ≥
      `0.0001²`), but it is included for defensive correctness. Very small
      `dmax` with large direct stiffness can produce `K ~ 1e8+`; this is
      numerically valid but produces very stiff constraints (high `D_i`).

  **Important:** In the `aref` formula, impedance `imp_i` multiplies only the
  position term, NOT the velocity term. This is the MuJoCo KBIP convention:
  `aref = −B·vel − K·imp·pos`. The existing penalty path applies impedance
  uniformly to both terms (`F = −imp·k·pos − imp·b·vel`), which is a
  different formulation. The Newton path must use the KBIP form exactly.

  This replaces our existing `solref_to_penalty()` (which returns `(k, b)` for
  penalty forces) with a formulation that separates impedance from stiffness,
  matching MuJoCo's `mj_referenceConstraint()` / KBIP derivation. **Note:**
  Direct mode (`solref[0] ≤ 0`) is not implemented in the existing penalty
  path — `solref_to_penalty()` falls back to defaults for negative solref.
  The Newton path introduces direct mode as new functionality.

**State classification per constraint type:**

| `ConstraintType` | States used | Classification rule |
|------|-------------|-------------------|
| `Equality` | `Quadratic` only | Always Quadratic (equality = unconstrained penalty) |
| `FrictionLoss` | `Quadratic`, `LinearPos`, `LinearNeg` | Huber threshold at `±R_i·floss_i` |
| `LimitJoint` | `Quadratic`, `Satisfied` | `jar_i < 0` → Quadratic; `jar_i ≥ 0` → Satisfied |
| `LimitTendon` | `Quadratic`, `Satisfied` | `jar_i < 0` → Quadratic; `jar_i ≥ 0` → Satisfied |
| `ContactNonElliptic` | `Quadratic`, `Satisfied` | `jar_i < 0` → Quadratic; `jar_i ≥ 0` → Satisfied |
| `ContactElliptic` | `Quadratic`, `Satisfied`, `Cone` | Three-zone (§15.7) — applies to all `dim` rows of the contact |

**Multi-row constraints:** Equality constraints (Connect=3, Weld=6 rows) and
elliptic contacts (dim rows) share their parent's `solref`/`solimp`, but each
row computes its own `imp_i` from its individual `|pos_i − margin_i|`,
producing independent `R_i`, `D_i`, and `aref_i` per row. In particular,
contact friction rows have `efc_pos = 0`, giving `imp = dmin` (the softest
impedance), while the normal row uses penetration depth. Rows are truly
independent scalar constraints in the Newton formulation.

**State transitions drive incremental Hessian updates** (§15.3).

##### 15.2 Cost Function and Gradient ✅

**Cost** (the reduced primal objective):

```
cost = costGauss + Σ_i s_i(jar_i)
```

Where:
- `costGauss = ½·(Ma − qfrc_smooth)^T · (qacc − qacc_smooth)` is the Gauss
  principle term. This equals `½·||qacc − qacc_smooth||²_M` (the
  `(1/2)||x − a_free||²_M` from the Background section). Here `Ma = M·qacc`,
  `qacc_smooth = M⁻¹·qfrc_smooth`, and `qfrc_smooth` is the smooth force
  vector. For Newton, `qfrc_smooth` **excludes** friction loss forces (which
  are constraint rows instead):
  `qfrc_smooth = qfrc_applied + qfrc_actuator + (qfrc_passive − qfrc_frictionloss) − qfrc_bias`
  **Note:** MuJoCo also includes `xfrc_applied` (external Cartesian body
  forces accumulated via `mj_xfrcAccumulate()`). Our pipeline does not
  currently support `xfrc_applied`; when/if it is added, it must be included
  in `qfrc_smooth`.
  Tracked in [future_work_9b.md](./future_work_9b.md) §DT-21.
- `s_i` is the per-constraint penalty (table in §15.1)

**Cost variable convention:** Throughout this spec, `cost` tracked by the
solver (and returned by `PrimalUpdateConstraint`/`classify_constraint_states`)
is **total cost** (Gauss + constraint): `cost = costGauss + Σ_i s_i`. This
matches MuJoCo's `PrimalUpdateConstraint()`, which first computes
constraint-only cost via `mj_constraintUpdate_impl()`, then immediately adds
the Gauss term (`ctx->cost += Gauss`). The convergence check (§15.8 step 6)
and warmstart comparison both use this total cost. The line search cost
evaluation (`PrimalEval` in §15.5) also computes total cost (the quadGauss
terms provide the Gauss contribution).

When `nefc = 0` (no active constraints), the solution is trivially
`qacc = qacc_smooth` with zero Newton iterations.

**Gradient** (in `qacc` space):

```
g = Ma − qfrc_smooth − J^T · efc_force
```

Where `efc_force` is the vector of per-constraint forces from the state machine.
`J^T · efc_force` is the generalized constraint force (`qfrc_constraint`).

Equivalently: `g = M·(qacc − qacc_smooth) − J^T·f` where `f_i` is from the
state table.

##### 15.3 Hessian and Incremental Updates ✅

The Hessian of the cost in `qacc` space:

```
H = M + J^T · diag(D_active) · J     (nv × nv, SPD)
```

Where `D_active[i] = efc_D[i]` if `efc_state[i] == Quadratic`, `0` otherwise
(a solver-local array derived on-the-fly from `efc_state` and `efc_D`). For elliptic
cones in `Cone` state, additional off-diagonal blocks within each contact's
dim are added via `hessian_cone()` (§15.7).

`H` is SPD (`M` is SPD, `J^T·D·J` is PSD). Factored via **dense Cholesky**
`H = L·L^T`. A separate copy `L_cone` incorporates cone Hessian contributions
when any contacts are in `Cone` state; the base `L` is kept for incremental
updates.

**Numerical stability note:** When `D_i` values are very large (stiff
constraints), `J^T·D·J` can dominate `M`, causing precision loss in `L·L^T`.
MuJoCo uses `LDL^T` factorization internally for its mass matrix (`qLD`), but
the primal solver's Hessian uses standard Cholesky. For Phase A, dense `L·L^T`
is sufficient — precision issues from large `D_i` are mitigated by the
impedance model clamping `D_i` through `solimp` width. Phase C should evaluate
whether switching to `LDL^T` for the Hessian factor improves robustness on
extreme stiffness settings.
Tracked in [future_work_9b.md](./future_work_9b.md) §DT-40.

**Incremental updates** (avoiding full refactorization each iteration):
- Constraint transitions to `Quadratic`: **rank-1 Cholesky update** — add
  `sqrt(D_i) · J_i` to L
- Constraint transitions away from `Quadratic`: **rank-1 Cholesky downdate** —
  remove that row from L
- If a downdate causes numerical instability (any diagonal of L goes
  non-positive), **full recompute**: rebuild `D_active`, reassemble
  `H = M + J^T·D·J`, refactor L from scratch
- After all scalar updates, if any elliptic contacts are in `Cone` state:
  copy `L → L_cone`, then apply cone Hessian blocks (§15.7)

```rust
/// Rank-1 Cholesky update (is_update=true) or downdate (is_update=false).
/// Returns false if downdate caused rank deficiency → caller must full-recompute.
fn cholesky_rank1_update(
    l: &mut DMatrix<f64>,            // nv×nv lower triangular factor (in-place)
    v: &DVector<f64>,                // nv-vector: sqrt(D_i) * J_row_i
    is_update: bool,                 // true=add, false=remove
) -> bool
```

##### 15.4 Gradient Computation and Newton Direction ✅

**Gradient and preconditioned gradient** (matching MuJoCo's `PrimalUpdateGradient`):

MuJoCo separates constraint update from gradient computation into two distinct
functions: `PrimalUpdateConstraint` (forces + cost) and `PrimalUpdateGradient`
(gradient + search direction). Our implementation follows this separation:

```
// Step 1: Compute raw gradient
qfrc_constraint = J^T · efc_force
grad = Ma − qfrc_smooth − qfrc_constraint

// Step 2: Compute preconditioned gradient (= Newton direction)
Mgrad = solve(L_cone or L, grad)      // H⁻¹ · grad via Cholesky
```

Where `L_cone` is used if any cone states exist, else `L`. The solve is:
```
L · z = grad          (forward substitution)
L^T · Mgrad = z       (back substitution)
```

`Mgrad` is a solver-local `nv`-vector storing the preconditioned gradient.
For Newton, `Mgrad = H⁻¹·grad`. (For CG, MuJoCo uses `Mgrad = M⁻¹·grad`
instead — this is why MuJoCo separates gradient from preconditioning.)

**Search direction**: `search = −Mgrad` (negated preconditioned gradient).

`qfrc_constraint` is also written to `Data.qfrc_constraint` at the end of
the solve (RECOVER block, §15.8) for diagnostic access by sensors.

Cost: `O(nv²)` per solve (dense).

##### 15.5 Line Search: 1D Exact Newton ✅

The step size `α` minimizes cost along the search direction:

```
qacc(α) = qacc + α · search
```

**Pre-computation** (all performed inside `PrimalSearch()`, before the first
`PrimalEval` call — matching MuJoCo where `Mv`/`Jv` and `PrimalPrepare` are
internal to `PrimalSearch()`, not outer-loop steps):
- `Mv = M · search` (nv-vector)
- `Jv = J · search` (nefc-vector)
- `snorm = ||search||`; if `snorm < mjMINVAL`, return `α = 0`
- `gtol = ls_tol · snorm / scale` where `ls_tol = solver_tolerance · ls_tolerance`
  (MuJoCo pre-multiplies the two tolerances before passing to the line search)
- `PrimalPrepare(...)` — assembles all quadratic coefficients (below)

**`PrimalPrepare`** (called once per line search, inside `PrimalSearch()`):

Computes `quadGauss` and per-constraint `quad` arrays. MuJoCo's
`PrimalPrepare()` is a separate function; our implementation may inline it.

Gauss term quadratics (from expanding `½·||qacc + α·search − qacc_smooth||²_M`):
```
quadGauss[0] = costGauss    // set by PrimalUpdateConstraint (= current Gauss cost)
quadGauss[1] = search^T · (Ma − qfrc_smooth)             (linear coefficient)
quadGauss[2] = ½ · search^T · Mv                 (= ½ · search^T · M · search)
```
Where `Mv = M · search`. Note `Mv^T · qacc = search^T · Ma` by symmetry of M.
**Important:** `quadGauss[0]` is NOT computed here — it was already computed by
`PrimalUpdateConstraint` (§15.12 step 7) and stored for reuse. `PrimalPrepare`
only computes `quadGauss[1]` and `quadGauss[2]`.

**Per-constraint quadratic coefficients:**

For **scalar constraints** (non-elliptic), `quad` stores 3 elements at
`quad[3*i..3*i+3]`:
```
quad_i = [q₀, q₁, q₂] where:
  q₀ = ½ · D_i · jar_i²              (constant term — current cost contribution)
  q₁ = Jv_i · D_i · jar_i            (linear in α)
  q₂ = ½ · Jv_i · D_i · Jv_i         (quadratic in α)
```

For **elliptic contacts** (dim ≥ 3), `PrimalPrepare` stores 9 elements at
`quad[3*i..3*i+9]` (overflowing into the slots of subsequent friction rows,
which are skipped by `i += (dim-1)`):
```
quad[3*i+0] = ½ · Σ_j D[i+j]·jar[i+j]²   (summed over all dim rows)
quad[3*i+1] = Σ_j Jv[i+j]·D[i+j]·jar[i+j]
quad[3*i+2] = ½ · Σ_j Jv[i+j]·D[i+j]·Jv[i+j]
quad[3*i+3] = U0 = jar[i]·μ                 (normal channel, scaled)
quad[3*i+4] = V0 = Jv[i]·μ                  (normal channel velocity)
quad[3*i+5] = UU = Σ_{j≥1} (jar[i+j]·friction[j−1])²
quad[3*i+6] = UV = Σ_{j≥1} jar[i+j]·friction[j−1]·Jv[i+j]·friction[j−1]
quad[3*i+7] = VV = Σ_{j≥1} (Jv[i+j]·friction[j−1])²
quad[3*i+8] = Dm = D[i] / (μ²·(1+μ²))
```
The first 3 entries (`q₀, q₁, q₂`) are the **per-row quadratic sums** over all
`dim` rows — used when the contact falls in the Bottom zone (per-row Quadratic).
The remaining 6 entries are the **cone auxiliary values** — used when the contact
falls in the Middle zone (cone cost). This avoids recomputing them per `PrimalEval`
call. `PrimalPrepare` iterates with the same `i += (dim-1)` skip pattern.

**1D cost at step α** (`PrimalEval`): For each constraint, the contribution
depends on the state *at that α*:

- **Equality**: always add `[q₀, q₁, q₂]`
- **Friction loss**: if `|jar_i + α·Jv_i| < R_i·floss_i` → add quadratic;
  else → add linear `{floss_i·(±jar_i − ½·R_i·floss_i), ±floss_i·Jv_i, 0}`
  (sign convention: `+jar` and `+Jv` for `LinearPos`, `−jar` and `−Jv` for
  `LinearNeg`; after multiplication by the outer `floss_i` factor, the constant
  evaluates to `−½·R_i·floss_i²`, matching both linear cost expressions from §15.1)
- **Unilateral** (limits/contact normal): if `jar_i + α·Jv_i < 0` → add
  quadratic; else → zero (satisfied)
- **Elliptic contacts**: Read auxiliary values from `quad[3*i+3..3*i+9]`
  (pre-computed by `PrimalPrepare`):
  `U0 = quad[3*i+3]`, `V0 = quad[3*i+4]`, `UU = quad[3*i+5]`,
  `UV = quad[3*i+6]`, `VV = quad[3*i+7]`, `Dm = quad[3*i+8]`.
  At each α: `N = U0 + α·V0`, `T² = UU + α·(2·UV + α·VV)`,
  `T = sqrt(max(0, T²))`. Note: `T²` is mathematically non-negative
  (it equals `Σ_j (a_j + α·b_j)²` by Cauchy-Schwarz); the `max(0, ...)`
  guards only against floating-point rounding. **Guard:** If `T < T_min`
  (with `T_min = 1e-15`,
  matching MuJoCo's `mjMINVAL`), reclassify: if `N ≥ 0` → top (zero), else
  → bottom (per-row quadratic). This prevents division by zero in cone
  derivatives when `T²` underflows during the line search sweep.
  Then classify zone (§15.7): top → zero; bottom → for each sub-row
  `j = 0..dim−1`, add `quad[3*(i+j)..3*(i+j)+3]` to totals; middle
  (cone) → cost `½·Dm·(N − μ·T)²` with analytical derivatives:
  ```
  T1 = (UV + α·VV) / T
  deriv[0] += Dm·(N − μ·T)·(V0 − μ·T1)
  deriv[1] += Dm·[(V0 − μ·T1)² + (N − μ·T)·(−μ·(VV/T − T1²/T))]
  ```

**Accumulation.** Initialize `total₀ = quadGauss[0]`, `total₁ = quadGauss[1]`,
`total₂ = quadGauss[2]`, `deriv[0] = 0`, `deriv[1] = 0`, `cost_cone = 0`.
The loop iterates per-row (`for i=0..nefc; i++`) with elliptic contacts
doing `i += (dim-1)` inside the loop body (net advance = `dim`), matching
the same stepping pattern as `PrimalUpdateConstraint` and `HessianIncremental`.
Scalar constraints advance by 1, elliptic contacts are processed as groups. For each constraint:
quadratic constraints (equality, friction, unilateral, and elliptic-bottom)
add to the `[total₀, total₁, total₂]` triple. Cone constraints in the
middle zone contribute `½·Dm·(N − μ·T)²` to `cost_cone`, and their
analytical derivatives to `deriv[0]` (first derivative) and `deriv[1]`
(second derivative), since the cone cost is NOT quadratic in α (it involves
`sqrt(T²)`). The combined line search derivatives are:
- `f'(α) = 2α·total₂ + total₁ + deriv[0]`
- `f''(α) = 2·total₂ + deriv[1]`
- `cost(α) = α²·total₂ + α·total₁ + total₀ + cost_cone`

Since the full cost is convex (sum of convex terms: the SPD `M`-norm Gauss
term and piecewise-quadratic/cone penalties), the 1D restriction `f(α)` is
also convex, guaranteeing `f''(α) ≥ 0`. Strict positivity `f''(α) > 0`
follows from the Gauss term: `quadGauss[2] = ½·search^T·M·search > 0`
(since `M` is SPD and `||search|| > 0`, guarded at pre-computation). Even
if cone `deriv[1]` contributions are negative, they cannot overcome the
strictly positive Gauss contribution because the overall cost is convex.

**Convexity guard:** Despite the theoretical guarantee, floating-point
rounding can cause `f''(α) ≤ 0` in degenerate cases. MuJoCo includes an
explicit guard: if `f''(α) ≤ 0`, emit a warning and clamp
`f''(α) = mjMINVAL` to prevent negative Newton steps. The implementation
must include this guard.

**Three-phase algorithm** (matching MuJoCo's `PrimalSearch` structure):

*Phase 1 — Initial Newton step:*
1. Evaluate at `α₀ = 0` → get `f'(0)`, `f''(0)`.
2. If `|f'(0)| < gtol` → already converged, return `α = 0`.
3. Newton step: `α₁ = −f'(0) / f''(0)`; evaluate at `α₁`.
4. If `|f'(α₁)| < gtol` → return `α₁`.
5. If `sign(f'(α₁)) ≠ sign(f'(0))` → bracket found → Phase 3.

*Phase 2 — One-sided stepping (finding a bracket):*
1. Continue taking Newton steps in the direction `dir = sign(−f'(0))`:
   `α_{k+1} = α_k − f'(α_k) / f''(α_k)`.
2. After each step: if `|f'| < gtol` → return. If derivative sign changes
   from Phase 1's sign → bracket found → Phase 3. If `ls_iterations`
   exhausted → return best α so far by lowest cost.

*Phase 3 — Bracketed refinement (converging to root of f'):*
1. Bracket `[lo, hi]` where `f'(lo) < 0` and `f'(hi) > 0` (from Phase 1/2).
   If `f'(lo) > 0` and `f'(hi) < 0`, swap so the convention holds.
2. Compute three candidate step sizes:
   - `α_mid = (lo + hi) / 2`
   - `α_lo = lo − f'(lo) / f''(lo)` (Newton from lo endpoint)
   - `α_hi = hi − f'(hi) / f''(hi)` (Newton from hi endpoint)
   Clamp Newton candidates to `[lo, hi]`.
3. Evaluate `f'` and `cost` at all three candidates (3 `PrimalEval` calls).
4. For any candidate: if `|f'| < gtol` → return it.
5. Update bracket: for each candidate with `f' < 0`, it can replace `lo`;
   with `f' > 0`, it can replace `hi`. Keep the tightest bracket.
6. If bracket width did not decrease (all candidates on same side, or
   `|hi − lo| < ε` with `ε = 1e-14 · max(1, |lo|, |hi|)`) → return the
   candidate with lowest cost.
7. Repeat from step 2 until `ls_iterations` exhausted → return best α.

```rust
/// PrimalSearch: line search wrapper that computes Mv, Jv, calls PrimalPrepare,
/// then runs the 3-phase line search. Returns optimal step size α.
/// After return, ctx.Mv and ctx.Jv are available for the MOVE step.
fn primal_line_search(
    ctx: &mut PrimalContext,         // solver state: qacc, M, J, jar, Ma, qfrc_smooth,
                                     //   search, efc_*, quad, quadGauss, Mv, Jv, ...
    data: &Data,                     // efc_type, efc_floss, efc_mu, efc_dim, efc_D, efc_R
    ls_iterations: usize,
    ls_tolerance: f64,               // = solver_tolerance * ls_tolerance (pre-multiplied)
    scale: f64,                      // 1 / (meaninertia · max(1, nv))
) -> f64                             // optimal step α
```
Internally, this function:
1. Computes `ctx.Mv = M · search`, `ctx.Jv = J · search`
2. Computes `snorm = ||search||`; if `snorm < mjMINVAL`, returns `α = 0`
3. Computes `gtol = ls_tolerance · snorm / scale`
4. Calls `PrimalPrepare` to fill `ctx.quad` and `ctx.quadGauss[1..2]`
5. Runs the 3-phase algorithm (Phase 1/2/3)

**`PrimalEval`** — evaluates cost derivatives at a given α. Called by the line
search. Not a separate public function; implemented inline within
`primal_line_search()` as a closure or inner function:
```rust
/// Evaluate f'(α), f''(α), and cost(α) at a given step size.
/// Iterates over all constraint rows, classifying state at α and
/// accumulating into [total₀, total₁, total₂] + deriv[0..1].
fn primal_eval(
    alpha: f64,
    jar: &DVector<f64>,              // current jar (before step)
    jv: &DVector<f64>,               // J · search
    quad: &[f64],                    // per-constraint [q₀, q₁, q₂]
    quad_gauss: &[f64; 3],          // Gauss term coefficients
    data: &Data,                     // efc_type, efc_floss, efc_mu, efc_dim, efc_D, efc_R
) -> (f64, f64, f64)                // (f'(α), f''(α), cost(α))
```

##### 15.6 Convergence Criteria ✅

Terminate when **either** condition is met:

1. **Scaled improvement** `< tolerance`:
   `(cost_prev − cost_curr) · scale < tolerance`
   where `cost_prev`, `cost_curr` are **total cost** (Gauss + constraint)
2. **Scaled gradient** `< tolerance`:
   `||g|| · scale < tolerance`

Where `scale = 1 / (meaninertia · max(1, nv))` and
`meaninertia = Model.stat_meaninertia` (computed once at model build time
from the home position `qpos0`, matching MuJoCo's `m->stat.meaninertia`).
Scaling makes the tolerance dimensionless and invariant to model size.

Also terminate immediately if line search returns `α = 0` (no improvement
possible).

**Default parameters** (matching MuJoCo):
- `solver_iterations`: 100 (max outer Newton iterations)
- `solver_tolerance`: 1e-8
- `ls_iterations`: 50 (max line search iterations per outer)
- `ls_tolerance`: 0.01

**Note:** The existing pipeline clamps `solver_iterations ≥ 10` and
`solver_tolerance ≥ 1e-8` before dispatching to PGS/CG (line ~12154). The
Newton solver should use the raw `Model` values without clamping, matching
MuJoCo's behavior where these parameters are respected as-is.

Typical convergence: **2–3 outer iterations** for standard robotics models.

##### 15.7 Elliptic Friction Cones ✅

**Cone type selection:** The `Model.cone` field (0 = pyramidal, 1 = elliptic)
controls which friction cone formulation is used. MuJoCo's Newton solver only
supports **elliptic** cones (`cone = 1`, which is the MuJoCo default). If
`model.cone == 0` (pyramidal), the Newton solver should warn and fall back to
PGS, since pyramidal cones require a different formulation not covered by this
spec. This matches MuJoCo's behavior where `mj_solPrimal()` assumes elliptic
cones. The `ContactElliptic` vs `ContactNonElliptic` classification in §15.0
applies only when `model.cone == 1`.

For contacts with `dim ≥ 3`, the penalty function couples the `dim` rows
(1 normal + `dim−1` friction). The state depends on position in the dual cone.

**Friction coefficient definitions:**

The `mu: [f64; 5]` array on each contact stores per-axis friction coefficients:
`mu = [sliding, sliding, torsional, rolling, rolling]` (from `Contact::with_condim()`).

The scalar `μ` used throughout this section is the **primary friction coefficient**:
`μ = mu[0]` (= sliding friction). This is the coupling parameter between normal
and tangential cone directions.

The per-axis array `friction[j−1]` used in cone formulas maps directly to the
`mu` array: `friction[j−1] = mu[j−1]` for `j = 1..dim−1`. The mapping per condim:

| condim | dim | `friction[]` |
|--------|-----|-------------|
| 1 | 1 | (none — normal only, no friction channels) → `ContactNonElliptic` |
| 3 | 3 | `[mu[0], mu[1]]` = `[sliding, sliding]` → `ContactElliptic` |
| 4 | 4 | `[mu[0], mu[1], mu[2]]` = `[sliding, sliding, torsional]` → `ContactElliptic` |
| 6 | 6 | `[mu[0], mu[1], mu[2], mu[3], mu[4]]` = all five → `ContactElliptic` |

**Degenerate friction guard:** If `μ = mu[0] < 1e-10` for an elliptic contact
(condim ≥ 3), reclassify it as `ContactNonElliptic` during assembly (treat as
normal-only, ignore friction channels). This avoids the degenerate
`Dm = D[i] / (μ²·(1+μ²)) → ∞` case. In practice, condim > 1 with zero
friction is unusual but must not produce NaN.

**Dual-cone mapping** (per contact at constraint offset `i`):

```
U[0] = jar[i] · μ                                    (normal, scaled by μ = mu[0])
U[j] = jar[i+j] · friction[j−1]    for j = 1..dim−1  (tangential, friction[j−1] = mu[j−1])

N = U[0]                                              (normal component)
T = ||(U[1], ..., U[dim−1])||                         (tangential magnitude)
```

**Three-zone classification:**

The classification produces a **single state for all `dim` rows** of an
elliptic contact group. All rows share the classified state (Quadratic,
Satisfied, or Cone). Individual per-row classification is never applied to
elliptic contacts — the coupled cone geometry determines a single zone.

**State replication:** After classifying the contact group, the state must
be explicitly written to all `dim` rows: `efc_state[i+j] = state` for
`j = 0..dim−1`. This is required because `HessianIncremental` (§15.3)
iterates per-row (`i += 1`) and checks `efc_state[i]` for each row
individually. MuJoCo does this explicitly in `mj_constraintUpdate_impl`.

`T` is a norm so `T ≥ 0` always. The `T < T_min` conditions below (with
`T_min = 1e-15`, matching MuJoCo's `mjMINVAL`) guard against degenerate
tangential magnitude. **The Cone state is only reached when `T ≥ T_min`**,
guaranteeing all `1/T`, `U[j]/T`, and `N/T³` terms in the cone formulas
below are well-defined. The same `T_min` threshold is used in both the outer
iteration classification (`PrimalUpdateConstraint`) and the line search
(`PrimalEval`) to ensure consistent zone assignments.

| Zone | Condition | State | Force | Cost |
|------|-----------|-------|-------|------|
| Top (separated) | `N ≥ μ·T` or (`T < T_min` and `N ≥ 0`) | `Satisfied` | all zeros | 0 |
| Bottom (fully active) | `μ·N + T ≤ 0` or (`T < T_min` and `N < 0`) | `Quadratic` | `f[i+j] = −D[i+j]·jar[i+j]` | `Σ_j ½·D[i+j]·jar[i+j]²` |
| Middle (cone surface) | otherwise | `Cone` | see below | `½·Dm·(N − μ·T)²` |

**Cost continuity at zone boundaries:** The Top/Cone boundary (`N = μ·T`) is
C¹ (cost and forces are both zero). The Bottom/Cone boundary (`μ·N + T = 0`)
has a cost discontinuity in general: Bottom cost is `Σ_j ½·D[i+j]·jar[i+j]²`
using per-row stiffnesses, while Cone cost is `½·Dm·(N−μ·T)²` using only
`D[i]` (normal row). Since friction rows have `efc_pos = 0` → `imp = dmin`
while the normal row uses penetration depth → different `imp`, the per-row
`D[i+j]` values differ from what `Dm = D[i]/(μ²·(1+μ²))` implies. This
matches MuJoCo's behavior — the formulation intentionally trades exact
boundary continuity for a simpler Hessian structure. The Newton solver
handles this through state-change detection and Cholesky update/downdate.

**Cone-state forces** (middle zone — `T > 0` guaranteed):

```
Dm = D[i] / (μ² · (1 + μ²))
NmT = N − μ · T

f[i]   = −Dm · NmT · μ                                    (normal force)
f[i+j] = (Dm · NmT · μ / T) · U[j] · friction[j−1]       for j = 1..dim−1
```

Note: `f[i+j]` is `+Dm·NmT·μ/T · U[j] · friction[j-1]`, which equals
`−f[i] / T · U[j] · friction[j-1]`, matching MuJoCo's sign convention.

**Cone Hessian block** (`dim × dim` local matrix `H_c`):

Uses `Dm` as defined above (`D[i] / (μ²·(1+μ²))`). In Cone state, the
friction rows' individual `D[i+j]` values are NOT used — the coupled cone
stiffness is normalized by the normal row's `D[i]` through `Dm`.

The Hessian is built in two steps. First, the **unscaled** block `H_raw`
(only the upper triangle and `k,j ≥ 1` block need to be filled; the
symmetrize step completes the lower triangle):
```
H_raw[0, 0] = 1
H_raw[0, j] = −μ · U[j] / T                                         for j ≥ 1
H_raw[k, j] = (μ·N / T³) · U[j] · U[k]                             for k ≤ j, k,j ≥ 1
H_raw[j, j] += (μ² − μ·N/T)                                        (diagonal add for j ≥ 1)
```

Then apply per-element scaling with `cone_scale[a] = μ` for `a=0`,
`cone_scale[a] = friction[a−1]` for `a ≥ 1` (local to the cone Hessian
construction; NOT the convergence `scale = 1/(meaninertia·max(1,nv))` from §15.6):
```
H_c[a, b] = Dm · cone_scale[a] · cone_scale[b] · H_raw[a, b]
```

Finally symmetrize: `H_c[a, b] = H_c[b, a]` (fills entries not set above,
e.g. `H_c[j, 0]` from `H_c[0, j]`).

The resulting `H_c[0,0] = Dm·μ²`, `H_c[0,j] = −Dm·μ²·friction[j−1]·U[j]/T`,
etc. This matches MuJoCo's `mj_constraintUpdate_impl` cone Hessian.

**Applying cone Hessian to global Cholesky** (`HessianCone`):
1. Copy base factor `L → L_cone`
2. Iterate `for i = 0..nefc` (stepping `i += 1` in the loop header):
   - If `efc_state[i] == Cone`:
     a. Get `dim` from the contact, `J_block = efc_J[i..i+dim, :]`
     b. Cholesky-factor the local `H_c = L_c · L_c^T` (`dim × dim`)
     c. Compute `rows = L_c^T · J_block` (`dim × nv`)
     d. For each row `r` of `rows`: rank-1 update `L_cone` with that row
     e. Advance `i += (dim − 1)` (net advance = `dim` including loop increment)
   - Since state is replicated to all dim rows (§15.7), any row of a cone
     group triggers processing; the `i += (dim-1)` skip avoids reprocessing.
3. Use `L_cone` for Newton direction solve

##### 15.8 Outer Iteration Loop ✅

```
INITIALIZE:
    // qfrc_smooth and qacc_smooth are solver-local variables (not stored in Data).
    // With approach (b) from §15.0, qfrc_passive already includes friction loss:
    qfrc_smooth = qfrc_applied + qfrc_actuator
                + (qfrc_passive − qfrc_frictionloss) − qfrc_bias
    qacc_smooth = M⁻¹ · qfrc_smooth   (via sparse LDL solve)

    // Assembly writes all efc_* fields directly to Data (§15.11).
    // These persist after the solver returns, available to sensors/diagnostics.
    // During assembly, impedance computation uses |efc_pos[i] − efc_margin[i]|
    // as the violation input (§15.1 margin subtraction), then derives
    // efc_imp → efc_R → efc_D → efc_aref per row.
    Assemble unified constraints → Data.efc_J, Data.efc_aref, Data.efc_D,
        Data.efc_R, Data.efc_imp, Data.efc_type, Data.efc_floss, Data.efc_pos,
        Data.efc_margin, Data.efc_vel, Data.efc_mu, Data.efc_dim, Data.efc_id,
        Data.efc_solref, Data.efc_solimp, Data.efc_diagApprox
    nefc = Data.efc_J.nrows()

    If nefc == 0:
        qacc = qacc_smooth; return   (no constraints → unconstrained solution)

    scale = 1 / (Model.stat_meaninertia · max(1, nv))

    // Warmstart selection. MuJoCo performs this in a separate `warmstart()`
    // function in engine_forward.c, before the solver is called. We inline
    // it here for simplicity. The semantics are identical:
    //
    // For Newton/CG: compare total cost (Gauss + constraint) at two points:
    //   cost_warmstart = constraint_cost(jar_warmstart) + gauss_cost(qacc_warmstart)
    //   cost_smooth    = constraint_cost(jar_smooth)     (Gauss = 0 at qacc_smooth)
    //
    // MuJoCo computes jar_smooth as `efc_b = J·qacc_smooth − aref`, which is
    // pre-computed during constraint assembly. We can reuse the same value.
    //
    // Steps:
    //   1. jar_warmstart = J · qacc_warmstart − aref
    //   2. constraint_cost_warmstart = Σ_i s_i(jar_warmstart_i)  (via mj_constraintUpdate)
    //   3. gauss_warmstart = ½·(M·qacc_warmstart − qfrc_smooth)^T
    //                          · (qacc_warmstart − qacc_smooth)
    //   4. cost_warmstart = constraint_cost_warmstart + gauss_warmstart
    //   5. efc_b = J · qacc_smooth − aref  (pre-computed during assembly)
    //   6. cost_smooth = Σ_i s_i(efc_b_i)  (Gauss term is zero at qacc_smooth)
    //   7. Pick whichever has lower total cost
    //
    // On the first timestep, qacc_warmstart is zero-initialized (from reset()),
    // so qacc_smooth typically wins. On subsequent steps, the previous qacc
    // (warmstart) usually wins.
    cost_warmstart = PrimalCost(qacc_warmstart)   // Gauss + constraint cost
    cost_smooth    = PrimalCost(qacc_smooth)       // Gauss = 0 at qacc_smooth, but
                                                   // constraint cost may be nonzero
    qacc = if cost_warmstart < cost_smooth { qacc_warmstart } else { qacc_smooth }
    Ma = M · qacc
    jar = J · qacc − aref
    PrimalUpdateConstraint(jar, Ma, qfrc_smooth, qacc, qacc_smooth)
        → efc_state, efc_force, cost   // cost = Gauss + Σ_i s_i (total cost)
    MakeHessian(efc_state) → H = M + J^T · diag(D_active) · J
    FactorizeHessian(H) → L (and L_cone if cone states)
    // If Cholesky factorization fails (numerically non-PD despite theoretical
    // guarantee), fall back to PGS immediately (§15.12 Phase A step 14).
    // This can happen with extreme D_i values from very stiff constraints.
    // PrimalUpdateGradient (§15.4): gradient + preconditioned gradient.
    // Reads efc_force written by the preceding PrimalUpdateConstraint call.
    qfrc_constraint = J^T · efc_force
    grad = Ma − qfrc_smooth − qfrc_constraint
    Mgrad = solve(L_cone or L, grad)           // H⁻¹ · grad

    // Initial convergence check. Note: MuJoCo checks gradient convergence
    // at the *bottom* of the loop (step 6), not before entering it. We add
    // this pre-loop check as an optimization to skip the first search
    // direction + line search when the warmstart is already converged.
    // This is a CortenForge extension — functionally equivalent since the
    // first iteration's step-6 check would catch it anyway.
    If scale · ||grad|| < tolerance: goto RECOVER

    search = −Mgrad

ITERATE (up to solver_iterations):
    1. LINE SEARCH:
       // PrimalSearch internally computes Mv, Jv, calls PrimalPrepare
       // (quad coefficients + quadGauss[1..2]), then runs the 3-phase
       // line search algorithm (§15.5). quadGauss[0] was already set by
       // PrimalUpdateConstraint in the previous step (or INITIALIZE).
       α = primal_line_search(search, ...)
       If α == 0: break

    2. MOVE:
       qacc += α · search
       Ma += α · Mv            // Mv from line search pre-computation
       jar += α · Jv            // Jv from line search pre-computation

    3. UPDATE CONSTRAINTS:
       oldstate = efc_state.clone()
       oldcost = cost
       PrimalUpdateConstraint(jar, Ma, qfrc_smooth, qacc, qacc_smooth)
           → efc_state, efc_force, cost   // total cost (Gauss + constraint)

    4. HESSIAN UPDATE:
       Process in constraint-row order (i = 0..nefc-1, stepping i += 1),
       interleaving updates and downdates as encountered (matching MuJoCo's
       HessianIncremental). Both HessianIncremental and PrimalUpdateConstraint
       iterate per-row (`for i=0..nefc; i++`) with elliptic contacts doing
       `i += (dim-1)` inside the loop body. Each row contributes an
       independent rank-1 term to the Cholesky factor:
         For each row i where oldstate[i] != efc_state[i]:
           If Quadratic→other: rank-1 downdate on L
             If downdate fails (diagonal ≤ 0): set needs_full_recompute = true; break
           If other→Quadratic: rank-1 update on L
           // Note: Satisfied↔Cone transitions require NO L updates (neither
           // state contributes to D_active). The cone Hessian is handled
           // separately via L_cone reconstruction below.
       If needs_full_recompute:
           // The partially-updated L is discarded; rebuild from scratch
           Rebuild D_active from current efc_state
           H = M + J^T · diag(D_active) · J
           L = cholesky(H)   // If this also fails → PGS fallback (step 14)
       If any cone states: rebuild L_cone from L + cone blocks
       (L_cone is rebuilt from scratch each iteration — copy L, add all
       current cone H_c blocks — because cone Hessians depend on jar
       which changes each step)

    5. GRADIENT + PRECONDITIONED GRADIENT (PrimalUpdateGradient, §15.4):
       qfrc_constraint = J^T · efc_force
       grad = Ma − qfrc_smooth − qfrc_constraint
       Mgrad = solve(L_cone or L, grad)           // H⁻¹ · grad

    6. CONVERGENCE CHECK:
       // cost and oldcost are TOTAL cost (Gauss + constraint), matching
       // MuJoCo's mj_solPrimal where PrimalUpdateConstraint adds the Gauss
       // term to ctx.cost. Improvement is positive when total cost decreases.
       improvement = scale · (oldcost − cost)
       gradient = scale · ||grad||
       If improvement < tolerance OR gradient < tolerance: break

    7. SEARCH DIRECTION:
       // Compute after convergence check to avoid wasted work on the
       // last iteration. Matches MuJoCo's loop order.
       search = −Mgrad

RECOVER:
    // Assembly fields (efc_J, efc_type, efc_pos, etc.) were already written
    // to Data during INITIALIZE and remain valid.
    Data.qacc = qacc
    Data.qfrc_constraint = J^T · efc_force
    Data.efc_state = efc_state
    Data.efc_force = efc_force
    Data.efc_jar = jar
    Data.efc_cost = cost

    // Extract per-joint and per-tendon limit forces for downstream sensors.
    // efc_id[i] stores the joint/tendon index that generated row i (§15.11).
    // At most one limit row per joint exists in J (lower XOR upper, per
    // the admission criteria in §15.0), so no overwrite conflicts occur.
    // Zero jnt_limit_frc/ten_limit_frc first since not all joints have
    // active limit rows.
    Data.jnt_limit_frc.fill(0.0)
    Data.ten_limit_frc.fill(0.0)
    for each row i where efc_type[i] == LimitJoint:
        Data.jnt_limit_frc[efc_id[i]] = efc_force[i]
    for each row i where efc_type[i] == LimitTendon:
        Data.ten_limit_frc[efc_id[i]] = efc_force[i]
```

**Pipeline integration — `mj_fwd_acceleration()` skip:**

**MuJoCo vs our pipeline ordering:** MuJoCo runs `mj_fwdAcceleration()` BEFORE
`mj_fwdConstraint()` — it computes `qacc_smooth = M⁻¹·qfrc_smooth` as input
to the constraint solver. Our pipeline has the reverse order:
`mj_fwd_constraint()` runs first (computing `qfrc_constraint`), then
`mj_fwd_acceleration()` computes `qacc = M⁻¹·(τ_total + qfrc_constraint)`.

For PGS/CG this works: they solve for forces, and acceleration is derived
afterward. Newton, however, produces `qacc` directly — the forces are
*recovered from* qacc, not vice versa. If `mj_fwd_acceleration()` runs after
Newton, it would overwrite Newton's optimized `qacc` with a redundant (and
potentially less accurate) inertia solve.

**The Newton path computes `qacc_smooth` internally.** Since our
`mj_fwd_acceleration()` runs after (not before) the constraint solver, the
Newton solver must compute `qacc_smooth = M⁻¹·qfrc_smooth` itself inside
`mj_fwd_constraint()`, using the pre-existing sparse LDL factorization
(`qLD`) from CRBA. This is a single sparse forward/back substitution — the
same operation that `mj_fwd_acceleration_explicit()` does, but on smooth
forces only (excluding constraint forces).

**Required change:** The Newton solver writes both `Data.qacc` and
`Data.qfrc_constraint` inside `mj_fwd_constraint()`. The `forward()` dispatch
must **skip** `mj_fwd_acceleration()` when `SolverType::Newton`:

```
fn forward() {
    ...
    mj_fwd_constraint();  // Newton computes qacc_smooth internally,
                           // then solves and writes qacc + qfrc_constraint
    if solver_type != Newton {
        mj_fwd_acceleration();  // PGS/CG: compute qacc from forces
    }
    ...
}
```

**Integrator compatibility:** Newton is compatible with **Euler** and **RK4**
integrators, which use `mj_fwd_acceleration()` purely as `qacc = M⁻¹·(forces)`.
For **implicit integrators** (`ImplicitFast`, `Implicit`, `ImplicitSpringDamper`),
`mj_fwd_acceleration()` solves a *modified* system `(M − h·D_vel)·qacc = rhs`
that incorporates velocity-derivative corrections. Skipping this step for Newton
would lose the implicit correction. MuJoCo's Newton solver is designed for
explicit/semi-implicit integration. For Phase A, Newton + implicit integrators
should emit a warning and fall through to PGS. Phase C may explore extending
Newton to implicit integration if needed. Also note: `forward_skip_sensors()`
(used by RK4 intermediate stages) also calls `mj_fwd_acceleration()` and must
apply the same skip logic.
Tracked in [future_work_9b.md](./future_work_9b.md) §DT-41.

##### 15.9 Warm Start ✅

`Data.qacc_warmstart: DVector<f64>` stores the previous timestep's `qacc`.
Newton initializes from this. Warm-starting has modest benefit for Newton
(already converges in 2–3 iterations) but avoids wasted work on the first
iteration.

**Save point:** `qacc_warmstart = qacc` must be written at the **end of
`Data::step()`**, after `integrate()` or `mj_runge_kutta()` returns. This
is integrator-independent — one save per timestep, always capturing the
final qacc. This matches MuJoCo's `mj_advance()` which saves
`qacc_warmstart` at the very end of the step.

Currently only the RK4 path saves warmstart (line 13990), and it saves the
**stage-3 qacc** rather than the final weighted combination — this is a
pre-existing bug. The Euler and implicit paths do not save at all. The fix
is straightforward: add `data.qacc_warmstart.copy_from(&data.qacc)` at the
end of `Data::step()`, after all integration is complete, and remove the
incorrect RK4 mid-step save.

The existing `efc_lambda` warmstart cache (keyed by `WarmstartKey`) remains
for PGS/CG. The Newton solver does not use it — it works in `qacc` space.
Constraint forces are recovered from `qacc` after convergence.

##### 15.10 Noslip Post-Processor (deferred)

MuJoCo's `noslip_iterations` option runs a modified PGS pass after the main
solve that updates only friction forces without regularization, suppressing
slip from soft contacts. This is an ad hoc post-processor (not a well-defined
optimization) and is deferred to a follow-up. The `noslip_iterations` MJCF
field should be parsed and stored but ignored with a warning.

##### 15.11 Integration Points ✅

**New types:**
```rust
// Add to SolverType enum
pub enum SolverType {
    PGS,
    CG,
    CGStrict,
    Newton,       // ← new
}

// Constraint state per scalar row
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConstraintState {
    Quadratic,    // = mjCNSTRSTATE_QUADRATIC
    Satisfied,    // = mjCNSTRSTATE_SATISFIED
    LinearNeg,    // = mjCNSTRSTATE_LINEARNEG
    LinearPos,    // = mjCNSTRSTATE_LINEARPOS
    Cone,         // = mjCNSTRSTATE_CONE
}

// Constraint type annotation per row
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConstraintType {
    Equality,
    FrictionLoss,
    LimitJoint,
    LimitTendon,
    ContactNonElliptic,
    ContactElliptic,
}
```

**New `Model` fields:**
```rust
pub stat_meaninertia: f64,      // trace(M) / nv at qpos0, computed at model build (for solver scaling)
pub ls_iterations: usize,       // max line search iters (default 50)
pub ls_tolerance: f64,          // line search gradient tol (default 0.01)
pub noslip_iterations: usize,   // parsed from MJCF, stored, ignored (default 0)
pub noslip_tolerance: f64,      // parsed from MJCF, stored, ignored (default 1e-6)
```

**New `Data` fields:**
```rust
// --- Warm start ---
pub qacc_warmstart: DVector<f64>,           // nv-vector, previous qacc

// --- Friction loss separation (§15.0 approach b) ---
pub qfrc_frictionloss: DVector<f64>,        // nv-vector, friction loss component of qfrc_passive

// --- Unified constraint system (nefc = total constraint rows) ---
pub efc_b: DVector<f64>,                    // nefc-vector: J·qacc_smooth − aref. Computed during
                                            //   assembly, consumed by warmstart cost comparison
                                            //   (§15.8 INITIALIZE). Persists in Data after the
                                            //   solve for diagnostic access (matches MuJoCo's
                                            //   d->efc_b), but is NOT updated by the solver loop
pub efc_J: DMatrix<f64>,                    // nefc × nv unified Jacobian
pub efc_type: Vec<ConstraintType>,          // per-row type annotation
pub efc_pos: Vec<f64>,                      // per-row constraint violation
pub efc_margin: Vec<f64>,                   // per-row margin (geom margin for contacts)
pub efc_vel: DVector<f64>,                  // per-row constraint-space velocity (J·qvel)
pub efc_solref: Vec<[f64; 2]>,             // per-row solver reference parameters
pub efc_solimp: Vec<[f64; 5]>,             // per-row solver impedance parameters
pub efc_diagApprox: Vec<f64>,              // per-row diagonal approximation of A
pub efc_R: Vec<f64>,                        // per-row regularization R
pub efc_D: Vec<f64>,                        // per-row D = 1/R (constraint stiffness)
pub efc_imp: Vec<f64>,                      // per-row impedance value (from solimp sigmoid)
// Note: MuJoCo packs K, B, imp, and impP into a single `efc_KBIP[4*i+{0,1,2,3}]`
// array. We decompose this: K and B are transient (used only during `aref`
// computation, not stored); `imp` is stored as `efc_imp`; `impP` (the derivative
// of impedance w.r.t. penetration distance) is omitted — it is computed and stored
// by MuJoCo but never read by any MuJoCo subsystem (solvers, sensors, derivatives).
// It exists solely for external API introspection. If external parity is needed,
// Phase C can add `efc_impP: Vec<f64>`.
// Tracked in [future_work_9b.md](./future_work_9b.md) §DT-22.
pub efc_aref: DVector<f64>,                 // per-row reference acceleration
pub efc_floss: Vec<f64>,                    // per-row friction loss saturation
pub efc_mu: Vec<[f64; 5]>,                  // per-row friction coefs (contacts, [0;5] otherwise)
pub efc_dim: Vec<usize>,                    // per-row condim of parent contact (1 for non-contacts)
pub efc_id: Vec<usize>,                     // per-row source object index (joint ID for LimitJoint,
                                            //   tendon ID for LimitTendon, eq ID for Equality,
                                            //   contact index for Contact*, DOF/tendon index for FrictionLoss)

// --- Solver output (written to Data in RECOVER after convergence) ---
pub efc_state: Vec<ConstraintState>,        // per-row constraint state
pub efc_force: DVector<f64>,                // per-row constraint force
pub efc_jar: DVector<f64>,                  // per-row J·qacc − aref
pub efc_cost: f64,                          // total cost (Gauss + constraint)
```

**MJCF wiring:**
- `model_builder.rs`: Change `MjcfSolverType::Newton => SolverType::Newton`
- Parse `<option noslip_iterations="..." noslip_tolerance="..."/>` into Model
  fields (store with warning that noslip is not yet implemented)

##### 15.12 Implementation Phasing

**Phase A — Unified constraint assembly + Core Newton (minimum viable): ✅ COMPLETE**
1. ✅ `ConstraintType`, `ConstraintState` enums, all new `Data`/`Model` fields
   (§15.11). Compute `Model.stat_meaninertia = trace(M) / nv` at model build
   time (run CRBA at `qpos0`, take trace of resulting M, divide by nv)
2. ✅ `diagApprox` computation — for ALL constraint rows (contacts and non-contacts
   alike), compute `diagApprox_i = J_i · M⁻¹ · J_i^T` by solving `M · w = J_i^T`
   via forward/back substitution against the pre-existing mass matrix factorization
   (`qLD`), then computing `diagApprox_i = J_i · w` (one dot product). This is
   O(nv) per row and O(nv · nefc) total, acceptable for Phase A. Phase C may
   extract contact-row diagonals more efficiently from a partial Delassus assembly
   or use the body-weight approximation from §15.1 for sparse/large systems
3. ✅ `compute_aref()` — reference acceleration from solref/solimp per row, using
   the full KBIP derivation (§15.1) with `|pos − margin|` impedance
4. ✅ **Prerequisite refactor:** Extract explicit Jacobian rows from
   `apply_equality_constraints()`. Currently, equality constraint Jacobians
   are computed inline and immediately multiplied by forces (never
   materialized). Refactor to return `(J_rows, pos, solref, solimp)` per
   constraint. Equality Jacobian structure:
   - **Connect** (3 rows): body2_pos − body1_pos in world frame → 3×nv
     (3 translational rows, sparse columns at both bodies' DOFs)
   - **Weld** (6 rows): 3 translational + 3 rotational error → 6×nv
   - **Joint** (1 row): `q[joint2] − target` → 1×nv. For **single-joint**
     equality: `+1` at joint2's DOF. For **two-joint** equality: the Jacobian
     row has entries at BOTH DOFs — `poly'(q1)` at joint1's DOF and `1` at
     joint2's DOF — because the constraint is `q2 − poly(q1) = 0`.
     `poly'(q1)` is the analytical derivative of the polynomial
     `polycoef[0] + polycoef[1]·q1 + polycoef[2]·q1² + ...`, evaluated at
     the current `q1`: `poly'(q1) = polycoef[1] + 2·polycoef[2]·q1 + ...`
     (standard Horner differentiation over the `polycoef` array, up to 5
     terms). The sign is negated (`−poly'(q1)`) because increasing `q1`
     decreases the residual `q2 − poly(q1)`. Note: the current penalty code
     (`apply_joint_equality_constraint`) applies force only to joint2's DOF
     and relies on implicit force propagation through M. The Newton path
     needs the explicit full Jacobian row.
   - **Distance** (1 row): `||p2−p1|| − target` → 1×nv (distance direction)
5. ✅ Friction loss migration: store friction loss contribution separately
   (`Data.qfrc_frictionloss`) per §15.0 approach (b). This must precede
   unified assembly because: (a) friction loss rows need to be available
   for inclusion in J, and (b) `qfrc_smooth` computation needs
   `qfrc_frictionloss` to be separated from `qfrc_passive`
6. ✅ Unified Jacobian assembly: `assemble_unified_constraints()` that builds
   `efc_J`, `efc_aref`, `efc_D`, `efc_R`, `efc_imp`, `efc_type`, `efc_floss`,
   `efc_pos`, `efc_margin`, `efc_vel`, `efc_solref`, `efc_solimp`,
   `efc_diagApprox`, `efc_mu`, `efc_dim`, `efc_id`, `efc_b` using the extracted
   Jacobians from step 4, existing contact Jacobians, and friction loss rows
   from step 5. `efc_b = J · qacc_smooth − aref` is computed after assembly
   for use in warmstart cost comparison (§15.8)
7. ✅ `classify_constraint_states(jar, Ma, qfrc_smooth, qacc, qacc_smooth)
   → efc_state, efc_force, cost` — also called `PrimalUpdateConstraint` in
   the pseudocode (§15.8); these are the same function. Returns **total cost**
   (constraint + Gauss), matching MuJoCo's PrimalUpdateConstraint. The function
   first computes constraint-only cost from the state machine, then adds the
   Gauss term: `costGauss = ½·(Ma − qfrc_smooth)^T · (qacc − qacc_smooth)`,
   `cost += costGauss`. The Gauss cost is also stored as `quadGauss[0]` for
   reuse by the line search's `PrimalPrepare` (§15.5) — MuJoCo's
   `PrimalPrepare` reads this value rather than recomputing it.
   For scalar constraints (non-contact or `ContactNonElliptic`),
   apply the state machine table (§15.1) per row, advancing `i += 1`. For
   `ContactElliptic` rows, process all `dim` rows as a group using the
   three-zone classification (§15.7), then advance `i += (dim−1)` (net
   advance = `dim` including the loop increment). The first row of each
   group is the normal row; subsequent `dim−1` rows are friction. After
   classifying, replicate the state to all `dim` rows (§15.7). This
   per-row loop with skip pattern matches MuJoCo's `mj_constraintUpdate_impl`
   and correctly handles consecutive contacts of different condim.
8. ✅ `PrimalUpdateGradient` (§15.4): compute `qfrc_constraint = J^T·efc_force`,
   `grad = Ma − qfrc_smooth − qfrc_constraint`, `Mgrad = H⁻¹·grad` via
   Cholesky solve, `search = −Mgrad`
9. ✅ Full Hessian assembly `H = M + J^T·D·J` with dense Cholesky
10. ✅ ~~Simple backtracking line search (Armijo) as initial placeholder~~ →
    **Replaced by exact 1D Newton line search in Phase B.** The Armijo
    placeholder has been deleted.
11. ✅ Outer loop with convergence check
12. ✅ `SolverType::Newton` wiring + `model_builder.rs` fix +
    `forward()` and `forward_skip_sensors()` dispatch to skip
    `mj_fwd_acceleration()` for Newton (Euler and RK4 only; Newton +
    implicit → warn and fall back to PGS per §15.8)
13. ✅ `qacc_warmstart` save at end of `Data::step()`, integrator-independent
    (§15.9). Remove the incorrect RK4 mid-step save at line 13990
14. ✅ PGS fallback on non-convergence **or Cholesky failure**: re-dispatch
    through the PGS/CG path from the beginning — run penalty-based
    limit/equality forces (the existing `apply_*` functions writing to
    `qfrc_constraint`), then invoke PGS on the contact-only system. The
    unified constraint assembly is discarded. Stale `efc_*` fields are
    cleared on fallback.

**Phase B — MuJoCo parity: ✅ COMPLETE**
1. ✅ Exact 1D Newton line search (`primal_prepare`, `primal_eval`, `primal_search`)
   with per-constraint quadratics and three-phase bracketing
2. ✅ Incremental Cholesky rank-1 updates/downdates (`cholesky_rank1_update`,
   `cholesky_rank1_downdate`) using Linpack DCHUD/DCHDD algorithms
3. ✅ Elliptic cone Hessian blocks (`hessian_cone`) with `L_cone` copy and
   per-contact H_c construction in `classify_constraint_states`
4. (Moved to Phase A step 13 — warmstart save is now integrator-independent.)
5. ✅ `primal_eval` for all constraint types (equality, friction, unilateral,
   elliptic) with correct zone transitions along the search direction
6. ✅ K/B formula fix in `compute_kbip` for negative dampratio (direct mode)
7. ✅ `efc_lambda` population in `recover_newton` for warmstart + touch sensors
8. ✅ `hessian_incremental` for incremental Hessian modification on state changes
9. ✅ Armijo line search deleted, replaced by exact Newton line search

**Phase C — Polish:**
1. ✅ Noslip post-processor — `noslip_postprocess()`: modified PGS on friction
   rows only (no regularization), elliptic cone projection, writes back updated
   forces + recomputes `qfrc_constraint` and `qacc`. Activated when
   `model.noslip_iterations > 0`.
2. ✅ Sparse Hessian path — `SparseHessian` struct: custom CSC LDL^T (no
   external crate). Threshold `NV_SPARSE_THRESHOLD = 60`. Left-looking numeric
   factorization with elimination tree. Full refactorization each Newton
   iteration (no incremental rank-1 in sparse path). `compute_gradient_and_search_sparse()`
   for sparse solve.
3. ✅ Solver statistics — `SolverStat` struct: per-iteration `improvement`,
   `gradient`, `lineslope`, `nactive`, `nchange`, `nline`. `primal_search()`
   returns `(alpha, neval)`. `data.solver_stat` and `data.solver_niter`
   populated at all `newton_solve()` exit points.
4. ✅ Per-step meaninertia — `data.stat_meaninertia = trace(qM) / nv` computed
   at `newton_solve()` entry. Uses per-step value for cost/gradient scaling
   instead of model-level constant. Falls back to `model.stat_meaninertia`
   when trace is non-positive.

#### Acceptance Criteria

1. ✅ **Correctness:** Newton solver produces finite, non-NaN qacc for basic
   models. (`test_newton_basic_free_fall`). Note: Newton uses Huber cost for
   friction loss vs `tanh` in PGS/CG, producing legitimately different solutions
   for models with `frictionloss > 0`.
2. ✅ **Convergence speed:** Newton solver statistics (`SolverStat`) are now
   populated per iteration, enabling convergence benchmarking.
   (`test_newton_solver_statistics`)
3. ✅ **Stiff contacts:** Newton handles `solref=[0.002, 1.0]` (5× stiffer than
   default) without divergence on a sphere-on-plane benchmark.
   (`test_newton_stiff_contacts`: ball settles, qacc bounded, velocity near zero)
4. ✅ **Elliptic cones:** Correct force recovery for condim 3, 4, and 6 contacts
   verified via `test_newton_contact_basic` (condim 3),
   `test_newton_condim4_torsional` (torsional friction damps spin),
   `test_newton_condim6_rolling` (rolling friction damps motion).
5. ✅ **Unified constraints:** Newton produces correct joint limit forces,
   equality constraint forces, and contact forces simultaneously.
   (`test_newton_unified_constraint_fields`, `test_newton_multi_constraint_stability`)
6. ✅ **Fallback:** Non-convergence and Cholesky failure fall back to PGS.
   Implicit integrator guard falls back to PGS. (`test_newton_implicit_fallback`)
7. ✅ **MJCF default:** Loading a model with `solver="Newton"` or no solver
   attribute routes to the Newton solver. (`test_default_solver_is_newton`)
8. ✅ **Energy stability:** Damped pendulum shows no energy gain over 100 steps.
   (`test_newton_energy_stability`)
9. ✅ **Warm start:** `qacc_warmstart` is non-zero after stepping, enabling
   warmstart selection in subsequent steps. (`test_newton_warmstart_not_zero`)
10. ✅ **No regression:** PGS and CG solver paths produce identical results.
    (`test_pgs_still_works`, `test_cg_still_works`, `test_pgs_contact_force_quantitative`,
    plus full 404+-test suite)
11. ✅ **Zero-constraint degenerate:** A model with no contacts/limits produces
    `qacc = qacc_smooth` (free-fall = -9.81 m/s²). (`test_newton_zero_constraints`)
12. ✅ **Direct mode solref:** Tested with `solref=[-500, -10]`.
    (`test_newton_direct_mode_solref`: ball settles on plane, qacc bounded)
    The K/B formula for direct mode is implemented and correct (Phase B step 6).
13. ✅ **Friction loss rows:** Friction loss is correctly handled as Huber-cost
    constraint rows in the unified system. The `test_frictionloss_scaling` test
    validates PGS friction loss behavior; Newton's Huber cost produces different
    (correct per MuJoCo) results where D is independent of floss in the
    quadratic zone.
14. ✅ **Condim 1 contacts:** Normal-only contacts work correctly through
    `ContactNonElliptic` path. Validated by existing contact tests with
    `condim="1"`.
15. ✅ **Multi-row equality:** Connect equality (3 rows) produces correct
    translational forces through the unified system.
    (`test_newton_equality_connect`, `test_newton_equality_connect_bounded_error`).
    Weld equality (6 rows) tested via `test_newton_equality_weld`.

#### Files (✅ = modified)
- ✅ `sim/L0/core/src/mujoco_pipeline.rs` — `SolverType::Newton`,
  `ConstraintType`, `ConstraintState`, `SolverStat`,
  `assemble_unified_constraints()`, `classify_constraint_states()`,
  `newton_solve()`, `primal_prepare()`, `primal_eval()`, `primal_search()`,
  `cholesky_rank1_update()`, `cholesky_rank1_downdate()`,
  `hessian_incremental()`, `hessian_cone()`, `recover_newton()`,
  `evaluate_cost_at()`, `compute_kbip()`, `noslip_postprocess()`,
  `SparseHessian` (CSC LDL^T: `assemble`, `refactor`, `solve`),
  `compute_gradient_and_search_sparse()`,
  dispatch in `mj_fwd_constraint()` and `forward()`,
  `qacc_warmstart` save, new Model/Data fields (~3500 lines added)
- ✅ `sim/L0/mjcf/src/model_builder.rs` — Newton → Newton mapping,
  `noslip_*`/`ls_*` field parsing, `stat_meaninertia` computation,
  tendon_solref default fix (DEFAULT_SOLREF instead of [0,0])
- ✅ `sim/L0/mjcf/src/parser.rs` — parse `ls_iterations`, `ls_tolerance`,
  `noslip_iterations`, `noslip_tolerance`
- ✅ `sim/L0/mjcf/src/types.rs` — `MjcfOption` fields for ls/noslip params
- ✅ `sim/L0/tests/integration/newton_solver.rs` — 40 acceptance tests (including
  stiff contacts, direct mode solref, condim 4/6, weld equality, MuJoCo reference
  values, quantitative contact force bounds, warmstart effectiveness, Newton vs PGS
  convergence benchmark)
- ✅ `sim/L0/tests/integration/mod.rs` — register newton_solver module
- ✅ `sim/L0/tests/integration/spatial_tendons.rs` — fix test_tendon_limit_forces
  for Newton (check qfrc_constraint + qfrc_passive)
- ✅ `sim/L0/tests/integration/passive_forces.rs` — fix test_frictionloss_scaling
  (scope to PGS solver)
- ✅ `sim/L0/tests/integration/cg_solver.rs` — fix warmstart test for Newton
  efc_lambda population
- ✅ `sim/docs/todo/future_work_5.md` — status updates
- ✅ `sim/docs/todo/index.md` — status update

#### References
- Todorov, E. (2014). "Convex and analytically-invertible dynamics with
  contacts and constraints." ICRA 2014.
- Todorov, E. (2011). "A convex, smooth and invertible contact model for
  trajectory optimization." ICRA 2011.
- MuJoCo `engine_solver.c` — `mj_solPrimal()`, `PrimalSearch()`, `PrimalEval()`,
  `PrimalPrepare()`, `PrimalUpdateConstraint()`, `HessianIncremental()`,
  `HessianCone()`, `MakeHessian()`, `FactorizeHessian()`
- MuJoCo `engine_core_constraint.c` — `mj_constraintUpdate_impl()`,
  `mj_referenceConstraint()`, `mjtConstraintState` enum
- MuJoCo docs: https://mujoco.readthedocs.io/en/stable/computation/index.html

---

### 16. Sleeping / Body Deactivation
**Status:** ✅ Complete | **Effort:** XL | **Prerequisites:** None

#### Current State
No sleep/deactivation system. Every body is simulated every step regardless of
whether it is stationary. MuJoCo deactivates bodies whose velocity is below a
threshold for a configurable duration, skipping their dynamics until an external
force or contact wakes them.

A prior sleep implementation (`Body::is_sleeping`, `put_to_sleep()`, `wake_up()`)
existed in the old World/Stepper architecture but was removed during the Model/Data
refactor. The current implementation is a ground-up redesign for the MuJoCo pipeline
(see MUJOCO_GAP_ANALYSIS.md §"Sleeping / Body Deactivation").

#### MuJoCo Reference

MuJoCo (as of 3.x) implements sleeping at the **constraint-island** level, not
per-body. Key reference points:

- **Enable flag:** `mjENBL_SLEEP` (bit 5 of `enableflags`). XML:
  `<option><flag sleep="enable"/></option>`. Default: disabled.
- **Threshold:** `mjOption.sleep_tolerance` (default `1e-4`). XML:
  `<option sleep_tolerance="real"/>`. Each DOF's velocity is checked against
  `sleep_tolerance * dof_length[dof]`, where `dof_length` normalizes angular
  vs linear DOFs to consistent [length/time] units.
- **Timer:** `mjData.tree_asleep` array (per kinematic tree). Countdown from
  `-(1 + mjMINAWAKE)` (i.e. `-11`) toward `-1` over `mjMINAWAKE = 10`
  consecutive timesteps where all DOFs in the tree are below threshold.
  Values `≥ 0` encode a sleep-cycle linked list among trees in the same island.
- **Island discovery:** `mj_island()` performs flood-fill/DFS over a tree-tree
  adjacency graph built from active constraints (contacts, equality, limited
  tendons, friction). All trees in an island sleep/wake atomically.
- **Sleep policy:** Per-tree `mjtSleepPolicy` enum: `AUTO`, `AUTO_NEVER`,
  `AUTO_ALLOWED`, `NEVER`, `ALLOWED`, `INIT`. Trees with actuators,
  multi-tree tendons, stiff/damped tendons, or flex elements receive
  `AUTO_NEVER`. XML: `<body sleep="auto|allowed|never|init">`.
- **Deactivation effects:** Sleeping islands skip the position stage (FK,
  collision, CRBA, mass matrix factorization) and velocity stage (Coriolis,
  passive damping). `qvel` and `qacc` are zeroed for sleeping DOFs.
- **Wake conditions:** Non-zero `qfrc_applied` or `xfrc_applied` (bytewise
  check); contact with awake tree; active limited tendon to awake tree;
  active/disabled cross-island equality constraint to awake tree.
- **Collision on wake:** Collision detection runs twice on the wake timestep
  (first to detect the wake condition, second to detect contacts inside the
  newly-awake island).
- **RK4 limitation:** RK4 integrator is unsupported with sleeping (substep
  wake semantics are ill-defined).
- **Initialized sleep:** `sleep="init"` allows trees to start asleep,
  avoiding the 10-step countdown for large scenes. All trees in an island
  must be marked together.

#### Objective

Implement tree-based body sleeping/deactivation that is semantically
compatible with MuJoCo's island-sleeping system. Bodies at rest are
automatically deactivated, reducing computation for scenes with many
stationary objects. The implementation is phased: Phase A delivers
per-tree sleeping without island discovery (each tree is its own island);
Phase B adds full island discovery for multi-tree constraint coupling.

#### Scope Exclusions

- **Deformable body sleeping:** ✅ Implemented (Phase C). Originally, trees
  containing deformable bodies received `AutoNever` policy. After §6B flex
  unification, flex DOFs are assigned `dof_treeid = usize::MAX` (permanently
  awake) and don't participate in the tree-level sleep system.
- **Full `sleep="init"` with island validation:** ✅ Implemented (Phase B).
  Union-find validation ensures init-sleep trees form consistent islands.
- **Per-island constraint solving:** ✅ Implemented (Phase B/C).
  `mj_fwd_constraint_islands` builds per-island Delassus matrices and solves
  independently via block-diagonal decomposition.

#### Specification

> **Note:** Line numbers referenced below are from the pre-implementation
> codebase and may be stale. Use function names and `grep` to locate code.

##### 16.0 Architectural Foundation: Kinematic Trees

**Existing infrastructure.** `Model.body_rootid: Vec<usize>` already maps
each body to its kinematic tree root. A "kinematic tree" is the set of
bodies sharing the same `body_rootid`. Body 0 (world) is its own tree and
is always static — never sleeps, never wakes.

**New Model field — tree enumeration:**

```rust
/// In Model:
pub ntree: usize,                        // number of kinematic trees (excluding world)
pub tree_body_adr: Vec<usize>,           // first body index for tree t
pub tree_body_num: Vec<usize>,           // number of bodies in tree t
pub tree_dof_adr: Vec<usize>,            // first DOF index for tree t
pub tree_dof_num: Vec<usize>,            // number of DOFs in tree t
pub body_treeid: Vec<usize>,             // tree index for each body (body 0 → usize::MAX sentinel)
pub dof_treeid: Vec<usize>,              // tree index for each DOF
pub tree_sleep_policy: Vec<SleepPolicy>, // per-tree sleep policy (computed at model build)
pub dof_length: Vec<f64>,                // per-DOF length scale for threshold normalization
```

**`SleepPolicy` enum:**

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepPolicy {
    /// Compiler decides (initial state, resolved before use).
    Auto,
    /// Compiler determined: never sleep (has actuators, multi-tree tendons, etc.).
    AutoNever,
    /// Compiler determined: allowed to sleep.
    AutoAllowed,
    /// User policy: never sleep. XML: `sleep="never"`.
    Never,
    /// User policy: allowed to sleep. XML: `sleep="allowed"`.
    Allowed,
    /// User policy: start asleep. XML: `sleep="init"`.
    Init,
}
```

**Policy resolution** (during `Model` construction, after all bodies/joints/
actuators/tendons are registered):

1. Initialize all trees to `Auto`.
2. Mark tree as `AutoNever` if:
   - Any actuator targets a joint/tendon/site in that tree (via
     `actuator_trntype`/`actuator_trnid` → `jnt_body`/`site_body` →
     `body_treeid`).
   - Tree contains a body with deformable bodies (`#[cfg(feature = "deformable")]`).
   - (Phase B) Tree is connected to a multi-tree tendon or a tendon with
     non-zero stiffness/damping.
3. Convert remaining `Auto` to `AutoAllowed`.
4. Apply user overrides from MJCF `<body sleep="...">` attribute.
5. Validate: error if user sets `Allowed` or `Init` on a tree that was
   marked `AutoNever` due to multi-tree tendon (Phase B).

**`dof_length` computation** (during `Model` construction):

```rust
for dof in 0..model.nv {
    let jnt_id = model.dof_jnt[dof];
    let jnt_type = model.jnt_type[jnt_id];
    let local_dof = dof - model.jnt_dof_adr[jnt_id]; // 0-based offset within joint

    let is_translational = match jnt_type {
        MjJointType::Slide => true,
        MjJointType::Hinge => false,
        MjJointType::Ball => false,   // 3 rotational DOFs
        MjJointType::Free => local_dof < 3, // DOFs 0,1,2 = translational; 3,4,5 = rotational
    };

    if is_translational {
        // Translational DOFs: length = 1.0 (threshold has units [m/s])
        model.dof_length[dof] = 1.0;
    } else {
        // Rotational DOFs: approximate mechanism length.
        // Use distance from joint anchor to subtree center of mass.
        // Fallback: 1.0 if body has zero mass or zero subtree extent.
        let body_id = model.dof_body[dof];
        model.dof_length[dof] = compute_dof_length(model, body_id).max(1e-6);
    }
}
```

**`compute_dof_length(model, body_id)`** estimates the characteristic
length of the mechanism at this body — the distance from the body's
joint anchor to the subtree center of mass. This makes angular velocity
thresholds scale-invariant: a 1 rad/s rotation on a 10 cm arm produces
0.1 m/s tip velocity, so `dof_length ≈ 0.1` and the effective threshold
is `1e-4 * 0.1 = 1e-5` rad/s. Fallback to `1.0` if `body_subtreemass`
is zero or the subtree COM coincides with the joint anchor.

**Note on Free joints:** A Free joint produces 6 DOFs — 3 translational
(indices 0–2 from `jnt_dof_adr`) and 3 rotational (indices 3–5). The
translational DOFs get `dof_length = 1.0`; the rotational DOFs get the
mechanism-length estimate. This matches MuJoCo's `dof_length` semantics
where linear and angular DOFs are scaled differently.

**Tree enumeration** is computed once during `Model` construction by
grouping bodies by `body_rootid`:

```rust
// Pseudocode for tree enumeration
let mut trees: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
for body_id in 1..model.nbody {  // skip world body
    trees.entry(model.body_rootid[body_id]).or_default().push(body_id);
}
model.ntree = trees.len();
// Populate body_treeid for each body, dof_treeid for each DOF
```

**Body ordering:** Bodies in MJCF models are added in depth-first document
order, so bodies within a single tree are typically contiguous. However,
this is NOT guaranteed for programmatically-constructed models. Therefore:

- `tree_body_adr[t]` = index of the **first** body in tree `t` (the root)
- `tree_body_num[t]` = number of bodies in tree `t`
- These are valid because MJCF parsing adds all bodies of a subtree
  contiguously (depth-first traversal). For `Model::empty()` + manual
  construction, the builder must ensure bodies of the same tree are added
  contiguously (document this constraint).
- `tree_dof_adr[t]` / `tree_dof_num[t]` follow the same pattern: DOFs
  are ordered by joint, joints are ordered by body, bodies within a tree
  are contiguous → DOFs within a tree are contiguous.

Similarly for DOFs: each DOF's tree is `body_treeid[dof_body[dof]]`.

**Construction sites requiring new fields:**

1. `Model::empty()` (`mujoco_pipeline.rs:~2382`) — initialize tree arrays
   as empty vecs, `ntree = 0`, `sleep_tolerance = 1e-4`.
2. `ModelBuilder::build()` (`model_builder.rs:~2470`) — compute tree
   enumeration from `body_rootid`, resolve sleep policies, compute
   `dof_length`. This is where the policy resolution algorithm (§16.0
   steps 1–5) executes.

**`JointContext` extension:** The existing `JointContext` struct (line ~307)
provides `jnt_id`, `jnt_type`, `dof_adr`, `qpos_adr`, `nv`, `nq`. Sleep
skip logic in visitors needs `model.jnt_body[ctx.jnt_id]` to look up the
body. This field is already available through `model.jnt_body` — no change
to `JointContext` is needed. Visitors that need sleep gating simply index
`model.jnt_body[ctx.jnt_id]` directly.

##### 16.1 Data Fields — Sleep State

```rust
/// In Data:

/// Per-tree sleep timer. Semantics:
/// - `< 0`: Tree is awake. Countdown from `-(1 + MIN_AWAKE)` toward `-1`.
///   When timer reaches `-1`, tree is "ready to sleep".
/// - `≥ 0`: Tree is asleep. Value encodes next tree index in sleep-cycle
///   linked list (Phase B; Phase A uses self-link: `tree_asleep[t] = t`).
pub tree_asleep: Vec<i32>,           // length: ntree

/// Per-tree awake flag (derived from tree_asleep for fast branching).
pub tree_awake: Vec<bool>,           // length: ntree

/// Per-body sleep state for efficient per-body queries.
pub body_sleep_state: Vec<SleepState>,  // length: nbody

/// Number of awake trees (for diagnostics / early-exit).
pub ntree_awake: usize,

/// Number of awake DOFs (for diagnostics / performance monitoring).
pub nv_awake: usize,
```

**`SleepState` enum:**

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepState {
    /// Body 0 (world) or body with no DOFs. Always computed, never sleeps.
    Static,
    /// Body is asleep. Position/velocity stages are skipped.
    Asleep,
    /// Body is awake. Full pipeline computation.
    Awake,
}
```

**Constant:**

```rust
/// Minimum number of consecutive sub-threshold timesteps before a tree
/// can transition to sleep. Matches MuJoCo's `mjMINAWAKE = 10`.
const MIN_AWAKE: i32 = 10;
```

**Option fields** (in `Model`):

```rust
/// Sleep velocity tolerance. Bodies with all DOF velocities below
/// `sleep_tolerance * dof_length[dof]` for MIN_AWAKE consecutive steps
/// are eligible for sleep. Default: 1e-4. Units: [m/s].
/// Set via MJCF `<option sleep_tolerance="..."/>`.
pub sleep_tolerance: f64,
```

**Enable flag.** Add `SLEEP` to the enable-flags bitmask:

```rust
/// Bit 5: enable body sleeping/deactivation.
pub const ENABLE_SLEEP: u32 = 1 << 5;
```

Sleep logic is gated on `model.enableflags & ENABLE_SLEEP != 0`. When the
flag is not set, all trees remain awake and the sleep pipeline is a no-op.

**`Data` construction site:** All `Data` instances are created via
`Model::make_data()` (`mujoco_pipeline.rs:~2650`). Add sleep field
initialization:

```rust
// In Model::make_data():
tree_asleep: vec![-(1 + MIN_AWAKE); self.ntree],  // All awake initially
tree_awake: vec![true; self.ntree],
body_sleep_state: {
    let mut v = vec![SleepState::Awake; self.nbody];
    v[0] = SleepState::Static;  // World body
    v
},
ntree_awake: self.ntree,
nv_awake: self.nv,
```

Then apply `Init` policy overrides (§16.7).

**`Data` clone site:** `Data` has a manual `impl Clone` (line ~2233) that
enumerates every field. The new sleep fields (`tree_asleep`, `tree_awake`,
`body_sleep_state`, `ntree_awake`, `nv_awake`) must be added to this
clone impl. RK4's `mj_runge_kutta()` clones `Data` for intermediate
stages — sleep state must be preserved across these clones (though sleep
is disabled for RK4, the clone impl must still be complete).

##### 16.2 MJCF Parsing

Extend the MJCF parser to handle sleep-related attributes:

**`<option>` element:**
- `sleep_tolerance="real"` → `Model.sleep_tolerance` (default `1e-4`).

**`<option><flag>` element:**
- `sleep="enable|disable"` → set/clear `ENABLE_SLEEP` bit in
  `Model.enableflags` (default: `disable`).

**`<body>` element:**
- `sleep="auto|allowed|never|init"` → stored during parsing, applied to
  tree during policy resolution (§16.0). The attribute applies to the
  **entire kinematic tree** rooted at the body where it appears. If set on
  a non-root body, it propagates to the tree root and a warning is emitted.

##### 16.3 Sleep Update — Core Algorithm

**`mj_update_sleep(model, data)`** — called at the end of `Data::step()`,
after integration completes and before the warmstart save. Insertion point
in `step()`:

```rust
pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
    // ... validation ...
    match model.integrator {
        Integrator::RungeKutta4 => { /* ... RK4 path (sleep disabled by §16.6) ... */ }
        _ => {
            self.forward(model)?;
            mj_check_acc(model, self)?;
            self.integrate(model);
        }
    }
    // === Sleep update: check thresholds, transition sleeping trees ===
    mj_update_sleep(model, self);
    self.qacc_warmstart.copy_from(&self.qacc);
    Ok(())
}
```

This placement ensures:
- Sleep evaluation sees the **post-integration** velocities (the velocities
  that will persist into the next step).
- Wake detection in `forward()` (§16.4) runs at the **start** of the next
  step, before FK, so newly-awake bodies get their full pipeline pass.
- The sleep→wake→sleep cycle is: `integrate → mj_update_sleep (may sleep)
  → next step → forward → mj_wake (may wake) → FK/collision/dynamics`.

This is the central sleep state machine:

```rust
fn mj_update_sleep(model: &Model, data: &mut Data) {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return;  // Sleep disabled — no-op
    }

    // Phase 1: Check velocity threshold for each awake tree
    for t in 0..model.ntree {
        if data.tree_asleep[t] >= 0 {
            continue;  // Already asleep
        }
        if model.tree_sleep_policy[t] == SleepPolicy::Never
            || model.tree_sleep_policy[t] == SleepPolicy::AutoNever
        {
            continue;  // Policy forbids sleeping
        }

        let can_sleep = tree_velocity_below_threshold(model, data, t);

        if can_sleep {
            // Increment timer toward -1 (ready to sleep)
            if data.tree_asleep[t] < -1 {
                data.tree_asleep[t] += 1;
            }
            // When timer reaches -1: put to sleep
            if data.tree_asleep[t] == -1 {
                put_tree_to_sleep(model, data, t);
            }
        } else {
            // Reset timer to fully awake
            data.tree_asleep[t] = -(1 + MIN_AWAKE);
        }
    }

    // Phase 2: Recompute derived arrays
    mj_update_sleep_arrays(model, data);
}
```

**`tree_velocity_below_threshold`:**

```rust
fn tree_velocity_below_threshold(model: &Model, data: &Data, tree: usize) -> bool {
    let dof_start = model.tree_dof_adr[tree];
    let dof_end = dof_start + model.tree_dof_num[tree];
    for dof in dof_start..dof_end {
        if data.qvel[dof].abs() > model.sleep_tolerance * model.dof_length[dof] {
            return false;
        }
    }
    true  // All DOFs below threshold (L∞ norm check)
}
```

**`put_tree_to_sleep`:**

```rust
fn put_tree_to_sleep(model: &Model, data: &mut Data, tree: usize) {
    // 1. Mark as asleep (self-link for Phase A; Phase B: link into island cycle)
    data.tree_asleep[tree] = tree as i32;

    // 2. Zero velocities and accelerations for all DOFs in this tree
    let dof_start = model.tree_dof_adr[tree];
    let dof_end = dof_start + model.tree_dof_num[tree];
    for dof in dof_start..dof_end {
        data.qvel[dof] = 0.0;
        data.qacc[dof] = 0.0;
    }

    // 3. Zero velocity-dependent cached quantities for bodies in this tree.
    //    With qvel = 0, cvel (spatial velocity), cacc_bias (Coriolis/centrifugal),
    //    and cfrc_bias (RNE backward-pass forces) are all zero.
    //    Position-dependent quantities (xpos, xquat, xipos, cinert) remain valid.
    let body_start = model.tree_body_adr[tree];
    let body_end = body_start + model.tree_body_num[tree];
    for body_id in body_start..body_end {
        data.cvel[body_id] = SpatialVector::zeros();
        data.cacc_bias[body_id] = SpatialVector::zeros();
        data.cfrc_bias[body_id] = SpatialVector::zeros();
    }
    // Also zero qfrc_bias, qfrc_passive, qfrc_constraint for sleeping DOFs
    for dof in dof_start..dof_end {
        data.qfrc_bias[dof] = 0.0;
        data.qfrc_passive[dof] = 0.0;
        data.qfrc_constraint[dof] = 0.0;
    }
}
```

**`mj_update_sleep_arrays`** — recomputes `tree_awake`, `body_sleep_state`,
`ntree_awake`, `nv_awake` from `tree_asleep`:

```rust
fn mj_update_sleep_arrays(model: &Model, data: &mut Data) {
    data.ntree_awake = 0;
    data.nv_awake = 0;

    for t in 0..model.ntree {
        let awake = data.tree_asleep[t] < 0;
        data.tree_awake[t] = awake;
        if awake {
            data.ntree_awake += 1;
            data.nv_awake += model.tree_dof_num[t];
        }
    }

    // Body 0 (world) is always Static
    data.body_sleep_state[0] = SleepState::Static;
    for body_id in 1..model.nbody {
        let tree = model.body_treeid[body_id];
        data.body_sleep_state[body_id] = if data.tree_awake[tree] {
            SleepState::Awake
        } else {
            SleepState::Asleep
        };
    }
}
```

##### 16.4 Wake Detection

Wake detection runs at the **start** of `forward()`, before any pipeline
stage. This is critical: wake must happen before FK so that newly-awake
bodies get their full pipeline pass.

**`mj_wake(model, data)`** — check user perturbations:

```rust
fn mj_wake(model: &Model, data: &mut Data) {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return;
    }

    // Check xfrc_applied (per-body Cartesian forces)
    for body_id in 1..model.nbody {
        if data.body_sleep_state[body_id] != SleepState::Asleep {
            continue;
        }
        // Bytewise nonzero check (matches MuJoCo: -0.0 wakes)
        let force = &data.xfrc_applied[body_id];
        if !is_zero_spatial_vector(force) {
            wake_tree(model, data, model.body_treeid[body_id]);
        }
    }

    // Check qfrc_applied (per-DOF generalized forces)
    for dof in 0..model.nv {
        let tree = model.dof_treeid[dof];
        if !data.tree_awake[tree] && data.qfrc_applied[dof] != 0.0 {
            wake_tree(model, data, tree);
        }
    }
}
```

**`mj_wake_collision(model, data)`** — check contacts between sleeping
and awake bodies. Called after `mj_collision()`:

```rust
fn mj_wake_collision(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for contact in &data.contacts[..data.ncon] {
        let body1 = model.geom_body[contact.geom1];
        let body2 = model.geom_body[contact.geom2];
        let state1 = data.body_sleep_state[body1];
        let state2 = data.body_sleep_state[body2];

        // Wake sleeping body if partner is awake (NOT static — static bodies
        // like the world/ground plane don't wake sleeping bodies. If they did,
        // no body resting on a ground plane could ever sleep, since the contact
        // with the ground is persistent).
        let need_wake = match (state1, state2) {
            (SleepState::Asleep, SleepState::Awake) => Some(body1),
            (SleepState::Awake, SleepState::Asleep) => Some(body2),
            // Both asleep: don't wake (they're resting on each other)
            // Both awake or asleep-vs-static: nothing to do
            _ => None,
        };

        if let Some(body_id) = need_wake {
            wake_tree(model, data, model.body_treeid[body_id]);
            woke_any = true;
        }
    }
    woke_any
}
```

**`wake_tree`:**

```rust
fn wake_tree(model: &Model, data: &mut Data, tree: usize) {
    if data.tree_awake[tree] {
        return;  // Already awake
    }
    data.tree_asleep[tree] = -(1 + MIN_AWAKE);  // Fully awake, reset timer
    data.tree_awake[tree] = true;
    data.ntree_awake += 1;
    data.nv_awake += model.tree_dof_num[tree];

    // Update body states for all bodies in this tree
    let body_start = model.tree_body_adr[tree];
    let body_end = body_start + model.tree_body_num[tree];
    for body_id in body_start..body_end {
        data.body_sleep_state[body_id] = SleepState::Awake;
    }
}
```

##### 16.5 Pipeline Integration — Skip Logic

The sleep system modifies the `forward()` pipeline to skip sleeping bodies.
Each pipeline stage gets an awake-body or awake-tree gate. The key principle:
**sleeping trees have valid, frozen position-stage data** (poses don't
change) and **zeroed velocity/acceleration** (set at sleep time).

**Modified `forward()` and `forward_skip_sensors()` sequences:**

Sleep logic must be added to **both** forward paths. `forward()` is used
by Euler/Implicit integrators; `forward_skip_sensors()` is used by RK4
intermediate stages. The sleep modifications are identical except that
`forward_skip_sensors()` omits the sensor stages. To avoid duplication,
extract the sleep-aware pipeline core into a shared helper:

```rust
/// Shared pipeline core with sleep gating.
/// `compute_sensors`: true for forward(), false for forward_skip_sensors().
fn forward_core(
    &mut self,
    model: &Model,
    compute_sensors: bool,
) -> Result<(), StepError> {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // ===== Pre-pipeline: Wake detection =====
    if sleep_enabled {
        mj_wake(model, self);
    }

    // ===== Position Stage =====
    mj_fwd_position(model, self);         // FK — internal sleep skip (§16.5a)
    mj_transmission_site(model, self);
    mj_collision(model, self);            // Collision — internal sleep filter (§16.5b)

    // Wake-on-contact: if sleeping body touched awake body, wake it
    // and re-run collision for the newly-awake tree's geoms
    if sleep_enabled && mj_wake_collision(model, self) {
        mj_update_sleep_arrays(model, self);
        mj_collision(model, self);        // Phase A: full re-collision
    }

    #[cfg(feature = "deformable")]
    mj_deformable_collision(model, self);
    if compute_sensors { mj_sensor_pos(model, self); }
    mj_energy_pos(model, self);

    // ===== Velocity Stage =====
    mj_fwd_velocity(model, self);         // Internal sleep skip (§16.5a)
    mj_actuator_length(model, self);
    if compute_sensors { mj_sensor_vel(model, self); }

    // ===== Acceleration Stage =====
    mj_fwd_actuation(model, self);
    mj_crba(model, self);                 // Full mass matrix — NOT skipped (§16.5 note)
    mj_rne(model, self);                  // Internal sleep skip (§16.5a)
    mj_energy_vel(model, self);
    mj_fwd_passive(model, self);          // Internal sleep skip (§16.5a')
    mj_fwd_constraint(model, self);
    if !self.newton_solved {
        mj_fwd_acceleration(model, self)?;
    }
    if compute_sensors {
        mj_sensor_acc(model, self);
        mj_sensor_postprocess(model, self);
    }

    Ok(())
}

pub fn forward(&mut self, model: &Model) -> Result<(), StepError> {
    self.forward_core(model, true)
}

fn forward_skip_sensors(&mut self, model: &Model) -> Result<(), StepError> {
    self.forward_core(model, false)
}
```

**Design note:** The sleep skip logic lives **inside** each pipeline
function (FK, CRBA, RNE, passive, collision) rather than calling separate
`_sleep` variants. This avoids function proliferation — each function
checks `sleep_enabled` once and skips per-body/per-joint as needed. The
`forward_core` helper eliminates the sensor-duplication concern entirely.

**`mj_crba` — NOT skipped for sleeping bodies.** CRBA begins with
`data.qM.fill(0.0)` and rebuilds the mass matrix from composite rigid
body inertias. If sleeping bodies were skipped, their `qM` entries would
be zero, causing the LDL factorization to produce NaN or fail (zero
diagonal). Fixing this (preserving previous mass matrix entries for
sleeping DOFs) requires either:
- Selective zeroing of only awake DOF rows/columns (O(nv_awake × nv))
- A separate "sleep-safe" mass matrix path

Both add complexity for modest gain — CRBA body loops are O(nbody) with
small constant factors (6×6 matrix operations), much cheaper than FK
(trigonometric per joint) or collision (O(n²) narrow-phase). Therefore,
**Phase A computes CRBA for ALL bodies every step.** The mass matrix and
LDL factorization are always valid. Phase B can optimize this by
per-island block-diagonal CRBA.

**`mj_fwd_acceleration` and `mj_fwd_constraint`** — these operate on the
full `nv`-dimensional system (sparse LDL solve, constraint Jacobians).
With the full mass matrix always valid, sleeping DOFs have all-zero forces
(`qfrc_bias`, `qfrc_passive`, `qfrc_actuator`, `qfrc_constraint` all
zeroed in `put_tree_to_sleep`). The solve produces `qacc = 0` for sleeping
DOFs correctly. The solver still processes them — acceptable overhead for
Phase A. Phase B can restructure for per-island block-diagonal solving.

**§16.5a — Gating FK, velocity kinematics, CRBA, RNE:**

These functions contain `for body_id in 1..model.nbody` loops. The sleep
modification adds a skip check at the top of each iteration:

```rust
// In mj_fwd_position — FK body loop only (the main per-body pose computation):
for body_id in 1..model.nbody {
    if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
        continue;  // Pose is frozen — skip FK
    }
    // ... existing FK computation (parent transform, joint integration, geom/site poses) ...
}

// Subtree mass/COM loops (also in mj_fwd_position) are NOT skipped — they are
// simple O(nbody) accumulations with tiny constants and need all bodies for
// correct backward accumulation into body 0. Sleeping bodies have frozen xipos
// which produces correct subtree mass/COM without recomputing FK.

// Same per-body skip pattern in mj_fwd_velocity, mj_rne (forward + backward passes).
```

**CRBA is NOT skipped** — see note below §16.5d on mass matrix safety.

For **RNE** which has reverse (leaf-to-root) passes that accumulate into
parent bodies, the skip logic is safe: all bodies in a tree share sleep
state (a child cannot be awake while its parent is asleep within the same
tree). Cross-tree accumulation into body 0 (world) is handled by the
existing `body_parent[root_body] = 0` path; body 0 is always `Static`
(never skipped).

**§16.5a' — Gating passive forces (`mj_fwd_passive`):**

`mj_fwd_passive` uses the `JointVisitor` pattern (via
`model.visit_joints()`) and a separate per-tendon loop. Skip logic must
be applied in both:

**Joint passive forces:** The `PassiveForceVisitor` holds `&Model` and
`&mut Data`. Add a sleep check at the start of each `visit_*` method:

```rust
impl PassiveForceVisitor<'_> {
    #[inline]
    fn is_joint_sleeping(&self, ctx: &JointContext) -> bool {
        self.model.enableflags & ENABLE_SLEEP != 0
            && self.data.body_sleep_state[self.model.jnt_body[ctx.jnt_id]]
                == SleepState::Asleep
    }
}

impl JointVisitor for PassiveForceVisitor<'_> {
    fn visit_hinge(&mut self, ctx: JointContext) {
        if self.is_joint_sleeping(&ctx) { return; }
        // ... existing spring/damper/friction logic ...
    }
    // Same for visit_slide, visit_ball, visit_free
}
```

**Tendon passive forces:** The per-tendon loop (`for t in 0..model.ntendon`)
applies spring, damper, and friction-loss forces mapped through `ten_J`.
In Phase A, tendons spanning multiple trees are rare (actuated trees get
`AutoNever`, and multi-tree tendons are a Phase B concern). The safe
approach: skip tendon passive forces only if **all** DOFs receiving force
from `ten_J[t]` belong to sleeping trees. Implementation:

```rust
for t in 0..model.ntendon {
    if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
        continue;  // All target DOFs are in sleeping trees
    }
    // ... existing tendon force computation ...
}

fn tendon_all_dofs_sleeping(model: &Model, data: &Data, t: usize) -> bool {
    // ten_J[t] is a sparse vector: check each nonzero entry's DOF
    for &(dof, _) in &data.ten_J[t] {
        if data.tree_awake[model.dof_treeid[dof]] {
            return false;  // At least one target DOF is awake
        }
    }
    true
}
```

This is conservative: if any DOF target of a tendon is awake, the tendon
force is computed for all DOFs (including sleeping ones). This avoids
inconsistency from partial tendon force application.

**§16.5a'' — Gating integration (`integrate`):**

`integrate()` has two phases: velocity update (per-DOF loop) and position
update (`mj_integrate_pos` via `JointVisitor`).

**Velocity integration** (per-DOF loop at line ~3948):

```rust
for i in 0..model.nv {
    if sleep_enabled && !data.tree_awake[model.dof_treeid[i]] {
        continue;  // Already zero for sleeping DOFs, skip for perf
    }
    self.qvel[i] += self.qacc[i] * h;
}
```

**Position integration** (`mj_integrate_pos`): uses `PositionIntegrateVisitor`
which implements `JointVisitor`. The visitor currently holds only
`qpos`, `qvel`, and `h` — it does NOT have access to `Model` or `Data`
for sleep checks. Two options:

**(a)** Extend `PositionIntegrateVisitor` to hold `&Model` and `&Data`
(or just `&[SleepState]` and `&[usize]` for `jnt_body`):

```rust
struct PositionIntegrateVisitor<'a> {
    qpos: &'a mut DVector<f64>,
    qvel: &'a DVector<f64>,
    h: f64,
    // Sleep gating:
    sleep_enabled: bool,
    jnt_body: &'a [usize],
    body_sleep_state: &'a [SleepState],
}

impl JointVisitor for PositionIntegrateVisitor<'_> {
    fn visit_hinge(&mut self, ctx: JointContext) {
        if self.sleep_enabled
            && self.body_sleep_state[self.jnt_body[ctx.jnt_id]] == SleepState::Asleep
        {
            return;  // Joint belongs to sleeping tree
        }
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
    }
    // Same for visit_slide, visit_ball, visit_free
}
```

**(b)** Add a `should_skip` callback or bitmask to `JointVisitor`. Option
(a) is preferred: minimal API change, sleep data is passed by reference
with zero allocation, and the check is a single indexed load + compare.

**Activation integration** (per-actuator loop at line ~3919): actuator
activation dynamics should still run for all actuators (activation state
must track ctrl input even if the actuated tree is asleep). However, the
`qfrc_actuator` mapping in `mj_fwd_actuation` already skips sleeping DOFs
implicitly because actuated trees have `AutoNever` policy in Phase A. No
change needed here.

**§16.5b — Gating collision detection:**

In `mj_collision()`, the broad-phase (sweep-and-prune) naturally handles
sleeping bodies because sleeping geoms have frozen AABBs.

**Important:** `check_collision_affinity(model, geom1, geom2)` does NOT
take `&Data` — it only has `&Model`. Adding `&Data` would change the
hot-path signature and affect all call sites. Instead, the sleep filter
is added **in the `mj_collision()` candidate loop**, after the affinity
check and before the narrow-phase:

```rust
// In mj_collision(), mechanism 1 (automatic pipeline):
for (geom1, geom2) in candidates {
    if !check_collision_affinity(model, geom1, geom2) {
        continue;
    }

    // Sleep filter: skip narrow-phase if BOTH geoms belong to sleeping bodies.
    // Keep sleeping-vs-awake pairs — they may trigger wake detection (§16.4).
    if sleep_enabled {
        let b1 = model.geom_body[geom1];
        let b2 = model.geom_body[geom2];
        if data.body_sleep_state[b1] == SleepState::Asleep
            && data.body_sleep_state[b2] == SleepState::Asleep
        {
            continue;
        }
    }

    // Narrow-phase collision detection
    if let Some(contact) = collide_geoms(model, geom1, geom2, ...) {
        data.contacts.push(contact);
        data.ncon += 1;
    }
}

// Mechanism 2 (explicit pairs): same sleep filter before narrow-phase
for pair in &model.contact_pairs {
    if sleep_enabled {
        let b1 = model.geom_body[pair.geom1];
        let b2 = model.geom_body[pair.geom2];
        if data.body_sleep_state[b1] == SleepState::Asleep
            && data.body_sleep_state[b2] == SleepState::Asleep
        {
            continue;
        }
    }
    // ... existing distance cull + narrow-phase ...
}
```

This preserves contacts between sleeping-vs-awake pairs (needed for wake
detection in §16.4) while skipping sleeping-vs-sleeping narrow-phase
(the dominant collision cost). The `check_collision_affinity` signature
is unchanged.

**§16.5c — `mj_collision_incremental`:**

When `mj_wake_collision` wakes trees, a second collision pass is needed
to detect contacts *within* the newly-awake trees and between them and
existing awake bodies. This is implemented as a targeted re-collision:

```rust
fn mj_collision_incremental(model: &Model, data: &mut Data) {
    // Only re-collide geoms belonging to trees that were just woken.
    // Use the same broad-phase + narrow-phase pipeline but restricted
    // to geom pairs where at least one geom belongs to a just-woken tree.
    // Implementation: flag trees woken this step, filter SAP candidates.
}
```

For Phase A, a simpler approach is acceptable: re-run `mj_collision()`
fully. The incremental optimization is a Phase B improvement.

**§16.5d — Sensor skip logic:**

Sensors iterate per-sensor (`for sensor_id in 0..model.nsensor`). To skip
sensors for sleeping bodies, we need a mapping from
`(sensor_objtype, sensor_objid)` → `body_id`:

```rust
fn sensor_body_id(model: &Model, sensor_id: usize) -> Option<usize> {
    let objid = model.sensor_objid[sensor_id];
    match model.sensor_objtype[sensor_id] {
        MjObjectType::Body => Some(objid),
        MjObjectType::Xbody => Some(objid),
        MjObjectType::Joint => Some(model.jnt_body[objid]),
        MjObjectType::Geom => Some(model.geom_body[objid]),
        MjObjectType::Site => Some(model.site_body[objid]),
        MjObjectType::Tendon => None,  // Multi-body — always compute
        MjObjectType::Actuator => None, // Actuated trees are AutoNever — always compute
        MjObjectType::Sensor => None,   // Meta-sensor — always compute
        _ => None,  // Unknown — always compute (safe default)
    }
}
```

In `mj_sensor_pos`, `mj_sensor_vel`, and `mj_sensor_acc`, add:

```rust
for sensor_id in 0..model.nsensor {
    if sleep_enabled {
        if let Some(body_id) = sensor_body_id(model, sensor_id) {
            if data.body_sleep_state[body_id] == SleepState::Asleep {
                continue;  // Sensor value frozen at sleep time
            }
        }
    }
    // ... existing sensor computation ...
}
```

Sensors on sleeping bodies retain their last-computed values (stored in
`data.sensordata`). They are NOT zeroed. This is semantically correct:
the body's pose/velocity are frozen, so the sensor reading is still valid.

##### 16.6 RK4 Interaction

Sleeping is incompatible with the RK4 integrator. If `ENABLE_SLEEP` is
set and `model.integrator == Integrator::RungeKutta4`, emit a warning
during model construction and disable sleeping (clear the `ENABLE_SLEEP`
bit). This matches MuJoCo's limitation.

```rust
// In Model construction / validation:
if self.enableflags & ENABLE_SLEEP != 0
    && self.integrator == Integrator::RungeKutta4
{
    eprintln!("WARNING: Sleeping is incompatible with RK4 integrator. Disabling sleep.");
    self.enableflags &= !ENABLE_SLEEP;
}
```

##### 16.7 `Data::reset()` and `Data::new()` Integration

**`Data::new(model)`:** Initialize sleep state based on tree policies:

```rust
// In Data::new():
for t in 0..model.ntree {
    match model.tree_sleep_policy[t] {
        SleepPolicy::Init => {
            data.tree_asleep[t] = t as i32;  // Start asleep
            // Zero velocities for this tree's DOFs
            let dof_start = model.tree_dof_adr[t];
            let dof_end = dof_start + model.tree_dof_num[t];
            for dof in dof_start..dof_end {
                data.qvel[dof] = 0.0;
            }
        }
        _ => {
            data.tree_asleep[t] = -(1 + MIN_AWAKE);  // Fully awake
        }
    }
}
mj_update_sleep_arrays(model, &mut data);
```

**`Data::reset(model)`:** Same as `new()` — re-initialize sleep state
from policies.

**`BatchSim` interaction:** Each `Data` in the batch has independent sleep
state. `BatchSim::step_all()` requires no changes — each `data.step(model)`
handles its own sleep logic. `BatchSim::reset()` delegates to
`Data::reset()` which re-initializes sleep state.

##### 16.8 Phased Implementation

**Phase A — Per-tree sleeping (this spec): COMPLETE**
- Tree enumeration infrastructure (§16.0)
- Sleep state in Data (§16.1)
- MJCF parsing for sleep attributes (§16.2)
- Sleep update state machine (§16.3)
- Wake detection: user forces + contact (§16.4)
- Pipeline skip logic for all stages (§16.5)
- RK4 guard (§16.6)
- Data init/reset (§16.7)
- Each tree is treated as its own island (no cross-tree coupling)
- Actuated trees receive `AutoNever` policy (safe default)

**Phase B — Island discovery and full sleep parity: COMPLETE**

Phase A treats each kinematic tree as its own island. This is correct
for uncoupled trees but breaks when constraints (contacts, equality,
tendons) couple multiple trees: the coupled trees must sleep and wake
as a unit. Phase B adds MuJoCo-compatible island discovery, the
sleep-cycle linked list, cross-island wake detection, `dof_length`
mechanism length computation, qpos/qvel change detection, per-island
block-diagonal constraint solving, and awake-index indirection arrays
for cache-friendly pipeline iteration.

Phase A is self-contained and delivers the primary performance benefit
(skipping stationary trees). Phase B is needed for scenes where
sleeping trees are coupled by constraints — common in stacking,
articulated chains, and environments with equality constraints between
bodies on different trees.

**Phase C — Sleep performance optimization:**

Phase B delivers correctness parity with MuJoCo's island-based sleeping.
Phase C closes the remaining performance gaps by exploiting the sleep
infrastructure that Phase B already allocates. Three optimizations, in
construction order:

1. **Awake-index pipeline iteration (§16.27):** Replace branch-per-body
   sleep checks with compact indirection arrays. The arrays
   (`body_awake_ind`, `dof_awake_ind`, `parent_awake_ind`) are already
   allocated and populated by `mj_update_sleep_arrays()`. Phase C wires
   them into every hot-path pipeline function so that loops iterate only
   over awake bodies/DOFs — zero branches, better cache locality.

2. **Island-local Delassus assembly (§16.28):** The per-island contact
   solver currently passes full `nv`-wide Jacobians to
   `assemble_contact_system()`, which computes full-size `M^{-1} * J^T`.
   Phase C extracts island-local sub-matrices: build a small
   `nv_island × nv_island` mass matrix, factor it, and assemble a
   compact Delassus matrix. For 10 islands of 10 contacts each, this
   turns one 300×300 solve into ten 30×30 solves.

3. **Selective CRBA (§16.29):** Skip composite-inertia accumulation and
   mass matrix construction for sleeping bodies. Sleeping DOFs retain
   their last-awake `qM` entries. The sparse LDL factorization runs
   only for awake DOF blocks. This is the highest-risk optimization —
   partial LDL refactorization must preserve positive-definiteness.

Phase C is purely performance. No new Data fields. No semantic changes.
Every test from Phase A and Phase B must remain bit-identical when all
bodies are awake (the optimizations are no-ops when `nbody_awake ==
nbody`). The per-step invariant: results with sleep optimization ON vs
OFF are identical to machine epsilon for awake bodies.

##### 16.9b Phase B — step() Overview

The Phase B `step()` function replaces `mj_update_sleep()` with the
island-aware `mj_sleep()`, and inserts `mj_island()` into the forward
pipeline. The complete call sequence:

```rust
pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
    // ... validation ...
    match model.integrator {
        Integrator::RungeKutta4 => { /* RK4 path (sleep disabled by §16.6) */ }
        _ => {
            self.forward(model)?;     // ← contains island discovery (§16.11)
            mj_check_acc(model, self)?;
            self.integrate(model);
        }
    }
    // === Phase B: island-aware sleep (replaces Phase A mj_update_sleep) ===
    mj_sleep(model, self);            // §16.12.2: countdown + island sleep
    mj_update_sleep(model, self);     // §16.17.1: recompute awake-index arrays
    self.qacc_warmstart.copy_from(&self.qacc);
    Ok(())
}
```

**Key differences from Phase A `step()`:**
- `mj_sleep()` (§16.12.2) replaces the Phase A `mj_update_sleep()`'s
  sleep-transition logic. It handles per-tree countdown AND island-level
  sleep decisions (all trees in an island must be ready).
- `mj_update_sleep()` (§16.17.1) is now a separate function that only
  recomputes derived arrays (`body_awake_ind`, `dof_awake_ind`,
  `parent_awake_ind`, `body_sleep_state`). In Phase A these were combined.
- `forward()` now contains `mj_island()` after `mj_make_constraint()`,
  and the full wake ordering (§16.13.1).

**Complete Phase B `forward()` pipeline:**

```rust
fn forward(&mut self, model: &Model) -> Result<(), StepError> {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // --- Wake detection: user forces (§16.3, carried from Phase A) ---
    // Must run BEFORE any pipeline stage. Checks xfrc_applied (bytewise)
    // and qfrc_applied for sleeping bodies. Without this, user-applied
    // forces on sleeping bodies would be silently ignored.
    if sleep_enabled {
        mj_wake(model, self);              // §16.3: user force wake
    }

    // --- Position-dependent (mj_fwd_position) ---

    // 1. FK part 1: joint → body poses (qpos → xpos/xquat/xmat)
    mj_kinematics1(model, self);

    // 2. qpos change detection: wake sleeping trees whose pose changed (§16.15)
    //    Uses a separate flag array (tree_qpos_dirty) set by mj_kinematics1()
    //    to avoid conflating with tree_awake (see §16.15.2).
    if sleep_enabled && mj_check_qpos_changed(model, self) {
        mj_update_sleep(model, self);
    }

    // 3. FK part 2: composite inertia, geom poses, site poses
    //    (only awake bodies; sleeping bodies' cinert/geom_xpos frozen)
    mj_kinematics2(model, self);

    // 4. Tendon kinematics: tendon lengths, moment arms, transmission
    //    Must precede mj_wake_tendon (needs ten_length for limit check).
    mj_tendon(model, self);
    mj_transmission(model, self);

    // 5. Tendon wake (§16.13.2): tendons coupling sleeping ↔ awake trees
    if sleep_enabled && mj_wake_tendon(model, self) {
        mj_update_sleep(model, self);
    }

    // 6. Collision
    mj_collision(model, self);

    // 7. Contact wake (§16.13, extends Phase A §16.4)
    if sleep_enabled && mj_wake_collision(model, self) {
        mj_update_sleep(model, self);
        mj_collision(model, self);          // Re-collide for newly-awake geoms
        // Note: MuJoCo optimizes this with mj_collision_incremental()
        // (only re-collide geoms in newly-woken trees). This optimization
        // is deferred — full re-collision is correct and simpler.
        // Tracked in [future_work_9b.md](./future_work_9b.md) §DT-24.
    }

    // 8. Equality constraint wake (§16.13.3)
    if sleep_enabled && mj_wake_equality(model, self) {
        mj_update_sleep(model, self);
    }

    // 9. Build constraints + islands
    mj_make_constraint(model, self);
    mj_island(model, self);                 // §16.11: island discovery

    // --- Position-dependent (continued) ---
    mj_com_pos(model, self);                // COM positions (subtree mass/COM)
    mj_crba_islands(model, self);           // §16.16.4: selective CRBA
    mj_factor_m(model, self);               // LDL factorization
    mj_sensor_pos(model, self);             // position-dependent sensors

    // --- Velocity-dependent ---
    mj_com_vel(model, self);                // subtree COM velocities
    mj_fwd_velocity(model, self);           // velocity kinematics
    mj_sensor_vel(model, self);             // velocity-dependent sensors
    mj_energy(model, self);                 // KE + PE (no sleep skip; see §16.26.8)
    mj_fwd_passive(model, self);            // passive forces (spring, damping)
    mj_fwd_actuation(model, self);          // actuator forces
    mj_fwd_acceleration(model, self);       // qacc = M^{-1} * (qfrc_total)

    // --- Constraint solve ---
    mj_fwd_constraint_islands(model, self); // §16.16.2: per-island solve
    mj_sensor_acc(model, self);             // acceleration-dependent sensors

    Ok(())
}
```

> **Note on pipeline ordering:** This `forward()` listing is the
> authoritative Phase B pipeline. It includes ALL stages from Phase A's
> `forward_core()` — none are omitted. Section §16.13.1 shows the same
> wake-detection ordering within `mj_fwd_position()` (a conceptual
> subset). Key invariants:
> - `mj_wake()` (user forces) runs FIRST, before any kinematics.
> - `mj_tendon` + `mj_transmission` run before `mj_wake_tendon`
>   (tendon wake needs `ten_length` to check limits).
> - `mj_wake_tendon` runs after FK part 2 and before collision.
> - `mj_wake_collision` runs after collision.
> - `mj_wake_equality` runs after contact wake.
> - CRBA and LDL run after island discovery so that
>   `mj_crba_islands()` can use island information.
> - `mj_fwd_passive` runs between velocity kinematics and actuation.
> - Sensors are split across position/velocity/acceleration stages.

##### 16.10 Phase B — Data Structures

###### 16.10.1 New Model Fields

```rust
/// In Model:

/// Disable-flags enum gains a new bit:
pub const DISABLE_ISLAND: u32 = 1 << 18;  // disable island discovery

/// Number of distinct kinematic trees spanned by each tendon.
/// Length: ntendon. Value 0 = no bodies, 1 = single tree, 2 = two trees.
/// Computed during model construction by scanning tendon path waypoints.
pub tendon_treenum: Vec<usize>,

/// Packed tree indices for tendons with treenum == 2.
/// For tendon t: tree_a = tendon_tree[2*t], tree_b = tendon_tree[2*t+1].
/// Unused entries (treenum != 2) are set to usize::MAX.
/// Length: 2 * ntendon.
pub tendon_tree: Vec<usize>,
```

When `DISABLE_ISLAND` is set, `mj_island()` is a no-op and each tree
is its own island (Phase A fallback). This allows profiling island
discovery overhead independently.

**`tendon_treenum` / `tendon_tree` computation** (during model
construction, after tendon waypoints are registered):

```rust
for t in 0..model.ntendon {
    let mut trees = BTreeSet::new();
    for wp in tendon_waypoints(model, t) {
        let body = waypoint_body(model, wp);
        if body > 0 {  // skip world body
            trees.insert(model.body_treeid[body]);
        }
    }
    model.tendon_treenum[t] = trees.len();
    if trees.len() == 2 {
        let mut iter = trees.iter();
        model.tendon_tree[2 * t] = *iter.next().unwrap();
        model.tendon_tree[2 * t + 1] = *iter.next().unwrap();
    }
}
```

###### 16.10.2 New Data Fields

Island discovery is recomputed every step from the current constraint
set. All island arrays are allocated from an arena or pre-sized to
worst-case bounds.

```rust
/// In Data — Island Discovery Output:

/// Number of constraint islands discovered this step.
pub nisland: usize,

/// Total DOFs across all constrained trees (trees that appear in at
/// least one island). Unconstrained trees are NOT counted.
pub nidof: usize,

// --- Tree → Island mapping ---

/// Island index for each tree. -1 if tree has no active constraints
/// (singleton — not part of any island).
/// Length: ntree.
pub tree_island: Vec<i32>,

/// Number of trees per island. Length: nisland.
pub island_ntree: Vec<usize>,

/// Start index in `map_itree2tree` for each island. Length: nisland.
pub island_itreeadr: Vec<usize>,

/// Packed array of tree indices, grouped by island. Length: ntree
/// (upper bound; actual used = sum of island_ntree).
pub map_itree2tree: Vec<usize>,

// --- DOF → Island mapping ---

/// Island index for each DOF. -1 if DOF's tree is unconstrained.
/// Length: nv.
pub dof_island: Vec<i32>,

/// Number of DOFs per island. Length: nisland.
pub island_nv: Vec<usize>,

/// Start index in `map_idof2dof` for each island. Length: nisland.
pub island_idofadr: Vec<usize>,

/// Start DOF index for each island (into the original qvel array).
/// Length: nisland. Used for block-diagonal extraction.
pub island_dofadr: Vec<usize>,

/// DOF → island-local DOF index. Length: nv.
pub map_dof2idof: Vec<i32>,

/// Island-local DOF index → global DOF index. Length: nv
/// (first nidof entries are constrained DOFs; rest are unconstrained).
pub map_idof2dof: Vec<usize>,

// --- Constraint → Island mapping ---

/// Island index for each constraint row. Length: nefc.
pub efc_island: Vec<i32>,

/// Per-island equality constraint count. Length: nisland.
pub island_ne: Vec<usize>,

/// Per-island friction constraint count. Length: nisland.
pub island_nf: Vec<usize>,

/// Per-island total constraint row count. Length: nisland.
pub island_nefc: Vec<usize>,

/// Start index in `map_iefc2efc` for each island. Length: nisland.
pub island_iefcadr: Vec<usize>,

/// Global constraint row → island-local row. Length: nefc.
pub map_efc2iefc: Vec<i32>,

/// Island-local row → global constraint row. Length: nefc.
pub map_iefc2efc: Vec<usize>,

// --- Block-diagonal copies (per-island solver input) ---

/// Island-reordered mass matrix (sparse). Block-diagonal extraction
/// of `qM` restricted to each island's DOFs. Length: ≤ nnz(qM).
pub iM: Vec<f64>,

/// Island-reordered LDL factorization. Length: ≤ nnz(qLD).
pub iLD: Vec<f64>,

/// Island-reordered Jacobian (sparse or dense depending on model).
/// Length: nefc × max(island_nv).
pub iJ: Vec<f64>,

/// Per-island reordered constraint vectors: efc_type, efc_D, efc_R,
/// efc_frictionloss, etc. These are gathered from the global arrays
/// using map_iefc2efc.

// --- Block-diagonal constraint vectors (per-island solver input) ---

/// Per-island reordered constraint type. Length: nefc.
pub iefc_type: Vec<i32>,

/// Per-island reordered diagonal D (impedance). Length: nefc.
pub iefc_D: Vec<f64>,

/// Per-island reordered diagonal R (regularizer). Length: nefc.
pub iefc_R: Vec<f64>,

/// Per-island reordered friction loss. Length: nefc.
pub iefc_frictionloss: Vec<f64>,

// --- Awake-Index Indirection Arrays ---

/// Sorted indices of awake + static bodies. Length: ≤ nbody.
/// Used for cache-friendly iteration in FK, velocity kinematics, etc.
pub body_awake_ind: Vec<usize>,

/// Number of entries in body_awake_ind (awake + static bodies).
pub nbody_awake: usize,

/// Sorted indices of bodies whose parent is awake or static.
/// Length: ≤ nbody. Used for backward-pass loops (subtree COM, RNE).
pub parent_awake_ind: Vec<usize>,

/// Number of entries in parent_awake_ind.
pub nparent_awake: usize,

/// Sorted indices of awake DOFs. Length: ≤ nv.
/// Used for velocity integration, CRBA, force accumulation.
pub dof_awake_ind: Vec<usize>,

// --- Per-island solver result scratch (§16.16.3) ---

/// Island solver results packed contiguously by `island_idofadr`.
/// Length: nv. See §16.16.3 for packing scheme.
pub island_qacc: Vec<f64>,
pub island_qfrc_constraint: Vec<f64>,

/// Island solver constraint forces packed by `island_iefcadr`.
/// Length: nefc (resized per step). See §16.16.3.
pub island_efc_force: Vec<f64>,

// --- Island discovery scratch space (§16.11) ---

/// DFS stack for flood-fill. Length: ntree.
pub island_scratch_stack: Vec<usize>,

/// Per-tree edge counts (CSR rownnz). Length: ntree.
pub island_scratch_rownnz: Vec<usize>,

/// Per-tree CSR row pointers. Length: ntree.
pub island_scratch_rowadr: Vec<usize>,

/// CSR column indices (edge targets). Resized per step (nefc varies).
pub island_scratch_colind: Vec<usize>,

// --- qpos Change Detection (§16.15) ---

/// Per-tree dirty flag set by mj_kinematics1() when a sleeping body's
/// computed xpos/xquat differs from the stored value — indicating
/// external qpos modification. Read and cleared by
/// mj_check_qpos_changed(). Length: ntree.
pub tree_qpos_dirty: Vec<bool>,
```

**Sparse format for block-diagonal matrices:** `iM`, `iLD`, and `iJ`
use the same sparse storage format as the global `qM`, `qLD`, and
`efc_J`. Specifically:
- `iM` and `iLD` use the same CSC (Compressed Sparse Column) format as
  `qM`/`qLD`, with row indices rewritten to island-local DOF indices
  via `map_dof2idof`. The sparsity pattern is a sub-matrix of the
  global pattern, extracted by selecting columns/rows for the island's
  DOFs.
- `iJ` uses the same format as `efc_J` (dense row-major if the model
  uses dense Jacobians; sparse CSR with `efc_J_rownnz`/`efc_J_rowadr`/
  `efc_J_colind` if the model uses sparse Jacobians). Column indices
  are rewritten to island-local DOF indices.

**Memory budget:** Island arrays use `O(ntree + nv + nefc)` storage,
allocated once at `make_data()` to worst-case bounds (ntree, nv, nefc
from Model). Re-filled each step. The block-diagonal copies (`iM`,
`iLD`, `iJ`) are pre-allocated at `make_data()` to the same size as
their global counterparts (upper bound; actual used per step ≤ global).

###### 16.10.3 Sleep State Encoding Change

Phase A uses a self-link for sleeping trees: `tree_asleep[t] = t as i32`.
Phase B replaces this with a **circular linked list** encoding:

```
Awake tree:    tree_asleep[t] < 0
               Values from -(1+MIN_AWAKE) to -1 represent countdown.

Sleeping tree: tree_asleep[t] >= 0
               Value is the index of the NEXT tree in the island's
               sleep cycle. The cycle is circular:
               tree_asleep[a] = b, tree_asleep[b] = c, tree_asleep[c] = a
               for a 3-tree island {a, b, c}.
               A single-tree island self-links: tree_asleep[t] = t
               (backward compatible with Phase A).
```

**Traversal:** To enumerate all trees in a sleeping island starting
from tree `i`:

```rust
fn for_each_tree_in_cycle(tree_asleep: &[i32], start: usize, mut f: impl FnMut(usize)) {
    let mut current = start;
    loop {
        f(current);
        current = tree_asleep[current] as usize;
        if current == start {
            break;
        }
    }
}
```

**Cycle minimum:** `mj_sleep_cycle(tree_asleep, i)` returns the smallest
tree index in the cycle — used as a canonical island identifier for
sleeping islands:

```rust
fn mj_sleep_cycle(tree_asleep: &[i32], start: usize) -> usize {
    let mut min_tree = start;
    let mut current = tree_asleep[start] as usize;
    while current != start {
        if current < min_tree {
            min_tree = current;
        }
        current = tree_asleep[current] as usize;
    }
    min_tree
}
```

##### 16.11 Phase B — Island Discovery Algorithm

###### 16.11.1 Overview

`mj_island(model, data)` discovers constraint islands by building a
tree-tree adjacency graph from the active constraint Jacobian, then
performing flood-fill (connected components) on this graph. Called once
per step, after `mj_make_constraint()` populates `data.efc_J`.

```
mj_fwd_position(model, data)
  mj_wake_tendon(model, data)  → mj_update_sleep(model, data)
  mj_collision(model, data)
  mj_wake_collision(model, data) → mj_update_sleep(model, data) → mj_collision(model, data)
  mj_wake_equality(model, data) → mj_update_sleep(model, data)
  mj_make_constraint(model, data)
  mj_island(model, data)          // ← HERE
```

###### 16.11.2 Edge Extraction

For each constraint row `r` in `0..data.nefc`:

1. **Fast path** (`tree_first`): For known constraint types (contact,
   joint limit, joint friction, connect/weld equality), directly look
   up the 1–2 trees involved from the constraint's metadata:
   - Contact: `body_treeid[geom_body[contact.geom1]]` and
     `body_treeid[geom_body[contact.geom2]]`.
   - Joint limit/friction: single tree from `jnt_body[efc_id]`.
   - Connect/weld equality: `body_treeid[eq_obj1id]` and
     `body_treeid[eq_obj2id]`.

2. **Slow path** (`tree_next`): For general constraints, scan the
   Jacobian row `efc_J[r]` for nonzero entries. Each nonzero at
   column `dof` maps to `dof_treeid[dof]`. Collect the set of distinct
   trees that appear.

3. **Edge generation** (`add_edge`): For each pair of distinct trees
   `(ta, tb)` found in a constraint row, emit edge `(ta, tb)`. If
   `ta == tb`, emit a self-edge (marks the tree as constrained but
   doesn't create a cross-tree coupling). Deduplicate against the
   previous edge to avoid redundant entries.

**Output:** A list of `(tree_a, tree_b)` edges and per-tree edge
counts `rownnz[t]`.

###### 16.11.3 Flood Fill

Build a CSR (Compressed Sparse Row) representation of the tree-tree
adjacency graph from the edge list:

```rust
fn mj_flood_fill(
    island: &mut [i32],       // output: island[tree] = island_id (-1 = no edges)
    rownnz: &[usize],         // edges per tree
    rowadr: &[usize],         // CSR row pointers
    colind: &[usize],         // CSR column indices
    stack: &mut [usize],      // DFS stack (length ntree)
    ntree: usize,
) -> usize {
    let mut nisland = 0;
    island.fill(-1);

    for seed in 0..ntree {
        if island[seed] >= 0 || rownnz[seed] == 0 {
            continue;  // Already assigned or no edges (singleton)
        }

        // DFS from seed
        let mut top = 0;
        stack[top] = seed;
        island[seed] = nisland as i32;

        while top < usize::MAX {  // while stack not empty
            let node = stack[top];
            top = top.wrapping_sub(1);

            for j in rowadr[node]..rowadr[node] + rownnz[node] {
                let neighbor = colind[j];
                if island[neighbor] < 0 {
                    island[neighbor] = nisland as i32;
                    top = top.wrapping_add(1);
                    stack[top] = neighbor;
                }
            }
        }

        nisland += 1;
    }

    nisland
}
```

Trees with no constraint edges (`rownnz[t] == 0`) get `island[t] = -1`
(singletons). They sleep/wake independently, exactly as in Phase A.

**Stack implementation note:** The `while top < usize::MAX` loop with
`wrapping_sub` / `wrapping_add` mirrors MuJoCo's C idiom using
`int top = -1`. In Rust, this encoding is correct but subtle — when
`top` is 0 and we pop, `0_usize.wrapping_sub(1) = usize::MAX` which
terminates the loop. For implementation clarity, consider using a
`usize` stack pointer initialized to 0 with a separate `stack_len`
counter, or `Option<usize>`. Either representation is equivalent;
the wrapping arithmetic is specified here for MuJoCo parity.

**Complexity:** O(ntree + nedges) — linear in the graph size. The
constraint Jacobian scan in edge extraction is O(nefc × nv_max_per_row)
for the fast path (constant per constraint type) or O(nefc × nv) for
the slow path.

###### 16.11.4 Island Array Population

After flood fill, populate the island arrays:

1. **Trees:** Scan `tree_island[t]` to compute `island_ntree[i]` and
   `island_itreeadr[i]`. Pack `map_itree2tree` grouped by island.

2. **DOFs:** For each tree in each island, copy its DOF range into the
   island's DOF set. Compute `island_nv[i]`, `island_idofadr[i]`,
   `island_dofadr[i]`. Build `map_dof2idof` and `map_idof2dof`
   bidirectional maps.

3. **Constraints:** Resize dynamic constraint arrays to current
   `data.nefc`: `efc_island.resize(nefc, -1)`,
   `map_efc2iefc.resize(nefc, -1)`, `map_iefc2efc.resize(nefc, 0)`.
   For each constraint row, `efc_island[r] =
   tree_island[tree_of_constraint_r]`. Compute `island_ne[i]`,
   `island_nf[i]`, `island_nefc[i]`, `island_iefcadr[i]`. Build
   `map_efc2iefc` and `map_iefc2efc`.

4. **Block-diagonal extraction:** Use the DOF and constraint maps to
   extract per-island sub-matrices from the global `qM`, `qLD`, and
   `efc_J`. The extraction uses `mju_block_diag_sparse()` (or Rust
   equivalent) to produce compact block-diagonal representations in
   `iM`, `iLD`, `iJ`.

###### 16.11.5 Disable Flag

When `model.disableflags & DISABLE_ISLAND != 0`:
- `mj_island()` returns immediately.
- `nisland = 0`, all `tree_island` = -1.
- Each tree sleeps/wakes independently (Phase A semantics).
- The constraint solver uses the global system (no per-island solve).

This flag allows benchmarking island discovery overhead and provides
a fallback if island discovery introduces bugs.

##### 16.12 Phase B — Sleep Cycle Linked List

###### 16.12.1 Sleeping an Island

When `mj_sleep()` determines an island should sleep (all trees in the
island have `tree_asleep == -1`, i.e., all ready to sleep), it creates
a circular linked list among the island's trees:

```rust
fn sleep_trees(model: &Model, data: &mut Data, trees: &[usize]) {
    let n = trees.len();
    assert!(n > 0);

    // Create circular linked list: tree[0] → tree[1] → ... → tree[n-1] → tree[0]
    for i in 0..n {
        let next = trees[(i + 1) % n];
        #[allow(clippy::cast_possible_wrap)]
        {
            data.tree_asleep[trees[i]] = next as i32;
        }

        // Zero DOF-level arrays for this tree
        let dof_start = model.tree_dof_adr[trees[i]];
        let dof_end = dof_start + model.tree_dof_num[trees[i]];
        for dof in dof_start..dof_end {
            data.qvel[dof] = 0.0;
            data.qacc[dof] = 0.0;
            data.qfrc_bias[dof] = 0.0;
            data.qfrc_passive[dof] = 0.0;
            data.qfrc_constraint[dof] = 0.0;
            data.qfrc_actuator[dof] = 0.0;  // §16.26.7: needed for policy relaxation
        }

        // Zero body-level arrays for this tree
        let body_start = model.tree_body_adr[trees[i]];
        let body_end = body_start + model.tree_body_num[trees[i]];
        for body_id in body_start..body_end {
            data.cvel[body_id] = [0.0; 6];
            data.cacc_bias[body_id] = [0.0; 6];
            data.cfrc_bias[body_id] = [0.0; 6];
        }
    }
}
```

**Zeroing rationale:** These arrays must be zeroed at sleep time to
maintain the invariant that sleeping bodies have zero velocity, zero
acceleration, and zero forces. This matches Phase A's
`put_tree_to_sleep()` behavior and is required for correctness when
the solver reads these values for constraint-coupled awake bodies.
`qfrc_actuator` is zeroed here (per §16.26.7) because Phase B allows
actuated trees to sleep with explicit `sleep="allowed"` policy.

For single-tree islands (no constraints), this creates a self-link:
`tree_asleep[t] = t as i32`, which is identical to Phase A behavior.

###### 16.12.2 The mj_sleep() Function (Phase B)

```rust
fn mj_sleep(model: &Model, data: &mut Data) -> usize {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return 0;
    }

    let mut nslept = 0;

    // Phase 1: Countdown for awake trees
    for t in 0..model.ntree {
        if data.tree_asleep[t] >= 0 {
            continue;  // Already asleep
        }
        if !tree_can_sleep(model, data, t) {
            data.tree_asleep[t] = -(1 + MIN_AWAKE);  // Reset
            continue;
        }
        // Increment toward -1
        if data.tree_asleep[t] < -1 {
            data.tree_asleep[t] += 1;
        }
    }

    // Phase 2: Sleep entire islands where ALL trees are ready
    for island in 0..data.nisland {
        let itree_start = data.island_itreeadr[island];
        let itree_end = itree_start + data.island_ntree[island];

        let all_ready = (itree_start..itree_end).all(|idx| {
            let tree = data.map_itree2tree[idx];
            data.tree_asleep[tree] == -1
        });

        if all_ready {
            let trees: Vec<usize> = (itree_start..itree_end)
                .map(|idx| data.map_itree2tree[idx])
                .collect();
            sleep_trees(model, data, &trees);
            nslept += trees.len();
        }
    }

    // Phase 3: Sleep unconstrained singleton trees that are ready
    for t in 0..model.ntree {
        if data.tree_island[t] < 0 && data.tree_asleep[t] == -1 {
            sleep_trees(model, data, &[t]);  // Self-link
            nslept += 1;
        }
    }

    nslept
}
```

**`tree_can_sleep(model, data, tree)`** returns `false` if:
- `tree_sleep_policy[t]` is `Never` or `AutoNever`.
- Any `xfrc_applied` on the tree's bodies is bytewise nonzero.
- Any `qfrc_applied` on the tree's DOFs is bytewise nonzero.
- Any `qvel[dof]` exceeds `sleep_tolerance * dof_length[dof]`.

###### 16.12.3 Atomic Island Wake

Waking any tree in a sleeping island wakes the **entire island**.
`mj_wake_tree()` traverses the sleep cycle and wakes every tree:

```rust
fn mj_wake_tree(model: &Model, data: &mut Data, tree: usize) {
    if data.tree_asleep[tree] < 0 {
        // Already awake — optionally update countdown if provided
        return;
    }

    // Traverse the sleep cycle, waking each tree
    let mut current = tree;
    loop {
        let next = data.tree_asleep[current] as usize;
        data.tree_asleep[current] = -(1 + MIN_AWAKE);  // Fully awake
        data.tree_awake[current] = true;

        // Update body states
        let body_start = model.tree_body_adr[current];
        let body_end = body_start + model.tree_body_num[current];
        for body_id in body_start..body_end {
            data.body_sleep_state[body_id] = SleepState::Awake;
        }

        current = next;
        if current == tree {
            break;  // Full cycle traversed
        }
    }
}
```

This is the key semantic difference from Phase A: waking one tree now
wakes all coupled trees. Without this, a sleeping tree could have
nonzero constraint forces from an awake partner but remain frozen.

**Note on direct `tree_awake` / `body_sleep_state` updates:**
`mj_wake_tree()` eagerly updates `tree_awake` and `body_sleep_state`
in addition to setting `tree_asleep` to the awake value. These direct
updates are **intentionally redundant** with the subsequent
`mj_update_sleep()` call. They are needed because other wake
functions in the same pass (e.g., `mj_wake_equality`) read
`tree_awake` to determine which trees are currently sleeping. Without
the eager updates, a tree woken by `mj_wake_collision` would still
appear asleep to `mj_wake_equality`.

##### 16.13 Phase B — Cross-Island Wake Detection

Phase A detects wake via user forces (`mj_wake`) and contacts
(`mj_wake_collision`). Phase B adds two more wake sources and refines
the wake ordering in the forward pass.

###### 16.13.1 Wake Ordering in forward()

```rust
// Wake-detection portion of forward(), extracted here for clarity.
// See §16.9b for the complete forward() pipeline.

mj_wake(model, data);                         // 0. User force wake (§16.3)

mj_kinematics1(model, data);                  // 1. FK part 1

if mj_check_qpos_changed(model, data) {       // 2. qpos change detection (§16.15)
    mj_update_sleep(model, data);
}

mj_kinematics2(model, data);                  // 3. FK part 2

mj_tendon(model, data);                       // 4a. Tendon kinematics
mj_transmission(model, data);                 // 4b. Transmission

if mj_wake_tendon(model, data) {              // 5. Tendon wake (§16.13.2)
    mj_update_sleep(model, data);
}

mj_collision(model, data);                    // 6. Collision

if mj_wake_collision(model, data) {           // 7. Contact wake
    mj_update_sleep(model, data);
    mj_collision(model, data);
}

if mj_wake_equality(model, data) {            // 8. Equality wake (§16.13.3)
    mj_update_sleep(model, data);
}

mj_make_constraint(model, data);              // 9. Build constraints + islands
mj_island(model, data);
```

**Why this order matters:** Each wake function may wake sleeping islands,
which changes the set of active bodies/geoms/constraints. The
`mj_update_sleep()` call between wake functions recomputes the
awake-index arrays so subsequent stages iterate over the correct set.

###### 16.13.2 mj_wake_tendon()

Tendons that span two trees (`tendon.treenum == 2`) and have an active
limit constraint can couple a sleeping tree to an awake tree:

```rust
fn mj_wake_tendon(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for t in 0..model.ntendon {
        // Only 2-tree tendons with active limits can wake
        if model.tendon_treenum[t] != 2 {
            continue;
        }
        if !tendon_limit_active(model, data, t) {
            continue;
        }

        let tree_a = model.tendon_tree[2 * t];
        let tree_b = model.tendon_tree[2 * t + 1];
        let awake_a = data.tree_awake[tree_a];
        let awake_b = data.tree_awake[tree_b];

        match (awake_a, awake_b) {
            (true, false) => {
                mj_wake_tree(model, data, tree_b);
                woke_any = true;
            }
            (false, true) => {
                mj_wake_tree(model, data, tree_a);
                woke_any = true;
            }
            (false, false) => {
                // Both asleep but in different cycles: merge by waking both.
                // Same logic as mj_wake_equality (§16.13.3) — a limited tendon
                // coupling two independently-sleeping islands must wake both.
                let cycle_a = mj_sleep_cycle(&data.tree_asleep, tree_a);
                let cycle_b = mj_sleep_cycle(&data.tree_asleep, tree_b);
                if cycle_a != cycle_b {
                    mj_wake_tree(model, data, tree_a);
                    mj_wake_tree(model, data, tree_b);
                    woke_any = true;
                }
                // Same cycle: both in the same sleeping island — no action
            }
            _ => {}  // Both awake — no action
        }
    }
    woke_any
}
```

**`tendon_limit_active(model, data, tendon_id)`** returns `true` if
the tendon has a limit constraint and the current tendon length is
outside the limit range:

```rust
fn tendon_limit_active(model: &Model, data: &Data, t: usize) -> bool {
    // Tendon must have limits defined
    if !model.tendon_limited[t] {
        return false;
    }
    let length = data.ten_length[t];
    length < model.tendon_range[t][0] || length > model.tendon_range[t][1]
}
```

**Prerequisite fields:** `tendon_treenum: Vec<usize>` (number of
distinct trees spanned by each tendon) and `tendon_tree: Vec<usize>`
(packed tree indices, 2 per tendon for `treenum == 2`). These are
computed during model construction by scanning each tendon's path
waypoints for the trees they touch.

###### 16.13.3 mj_wake_equality()

Equality constraints (connect, weld, joint) can couple trees:

```rust
fn mj_wake_equality(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for eq in 0..model.neq {
        if !model.eq_active[eq] {
            continue;  // Disabled equality constraint
        }

        let (tree_a, tree_b) = equality_trees(model, eq);
        if tree_a == tree_b {
            continue;  // Same tree — no cross-tree coupling
        }

        let awake_a = data.tree_awake[tree_a];
        let awake_b = data.tree_awake[tree_b];

        match (awake_a, awake_b) {
            (true, false) => {
                mj_wake_tree(model, data, tree_b);
                woke_any = true;
            }
            (false, true) => {
                mj_wake_tree(model, data, tree_a);
                woke_any = true;
            }
            (false, false) => {
                // Both asleep but in different cycles: merge by waking both.
                // This handles the case where two independently-sleeping islands
                // are coupled by a newly-activated equality constraint.
                let cycle_a = mj_sleep_cycle(&data.tree_asleep, tree_a);
                let cycle_b = mj_sleep_cycle(&data.tree_asleep, tree_b);
                if cycle_a != cycle_b {
                    mj_wake_tree(model, data, tree_a);
                    mj_wake_tree(model, data, tree_b);
                    woke_any = true;
                }
                // Same cycle: both are in the same sleeping island — no action
            }
            _ => {}  // Both awake — no action
        }
    }
    woke_any
}
```

**`equality_trees(model, eq)`** returns the two tree indices for an
equality constraint:
- `EqType::Connect | EqType::Weld`: `body_treeid[eq_obj1id[eq]]` and
  `body_treeid[eq_obj2id[eq]]`.
- `EqType::Joint`: `body_treeid[jnt_body[eq_obj1id[eq]]]` and
  `body_treeid[jnt_body[eq_obj2id[eq]]]`.

**Both-asleep-different-cycle case:** When two sleeping islands are
coupled by an equality constraint, they must be merged. The simplest
approach: wake both, let them go through the countdown again, and
`mj_sleep()` will create a single island cycle on the next sleep pass.

##### 16.14 Phase B — dof_length Mechanism Length

Phase A sets `dof_length[dof] = 1.0` for all DOFs. Phase B computes
the actual mechanism length for rotational DOFs, following the same
approach as MuJoCo's `engine_setconst.c` (subtree extent from
`body_pos` distances). The MuJoCo implementation also factors in
geom sizes for leaf bodies; the algorithm below uses inter-body
distances only, which is a close approximation. If exact parity is
needed, refine the leaf extent calculation during implementation.

###### 16.14.1 Body Length Computation

MuJoCo computes a per-body characteristic length during `mj_setConst`:

```rust
/// Compute characteristic body length for dof_length normalization.
///
/// For each body, compute the maximum extent from the body's center of mass
/// to any descendant body's center of mass, plus the body's own geometric
/// extent. This gives a length scale that converts angular velocity [rad/s]
/// to tip velocity [m/s] for the mechanism rooted at this body.
fn compute_body_lengths(model: &Model) -> Vec<f64> {
    let mut body_length = vec![0.0_f64; model.nbody];

    // Backward pass: accumulate subtree extents from leaves to root
    for body_id in (1..model.nbody).rev() {
        let parent = model.body_parent[body_id];

        // Distance from parent COM to this body's COM
        let dx = model.body_pos[body_id][0];  // local position in parent frame
        let dy = model.body_pos[body_id][1];
        let dz = model.body_pos[body_id][2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();

        // This body's extent: max of own geom extent and children's extent + distance
        let child_extent = body_length[body_id] + dist;
        body_length[parent] = body_length[parent].max(child_extent);
    }

    // Ensure minimum length of 1.0 (no normalization for tiny/zero-mass bodies)
    for length in &mut body_length {
        if *length < 1e-10 {
            *length = 1.0;
        }
    }

    body_length
}
```

###### 16.14.2 Per-DOF Length Assignment

```rust
fn compute_dof_lengths(model: &mut Model) {
    let body_length = compute_body_lengths(model);

    for dof in 0..model.nv {
        let jnt_id = model.dof_jnt[dof];
        let jnt_type = model.jnt_type[jnt_id];
        let offset = dof - model.jnt_dof_adr[jnt_id];

        let is_rotational = match jnt_type {
            MjJointType::Hinge => true,
            MjJointType::Ball => true,  // all 3 DOFs are rotational
            MjJointType::Free => offset >= 3,  // DOFs 3,4,5 are rotational
            MjJointType::Slide => false,
        };

        if is_rotational {
            model.dof_length[dof] = body_length[model.dof_body[dof]];
        } else {
            model.dof_length[dof] = 1.0;  // translational: already in [m/s]
        }
    }
}
```

**Effect:** A 1-meter arm with a hinge joint gets `dof_length ≈ 1.0`,
so `sleep_tolerance * 1.0 = 1e-4 rad/s`. A 10-cm arm gets
`dof_length ≈ 0.1`, so the threshold is `1e-4 * 0.1 = 1e-5 rad/s` —
tighter, because the same angular velocity produces less tip motion.

**Phase A → Phase B migration:** Replace the `for dof { dof_length[dof] = 1.0 }`
loop in `model_builder.rs` with a call to `compute_dof_lengths(model)`.

##### 16.15 Phase B — qpos/qvel Change Detection

MuJoCo detects external modifications to `qpos` (e.g., from RL
environment resets that set `qpos` directly without calling `reset()`).
If a sleeping body's `qpos` changed since the last step, it must be
woken so FK recomputes its pose.

###### 16.15.1 Algorithm

Inside `mj_kinematics1()`, after computing the new `xpos`/`xquat`
from `qpos` for each body, compare against the stored values:

```rust
// In mj_kinematics1(), for each body:
for body_id in 1..model.nbody {
    // Compute xpos, xquat from qpos (existing FK)
    let (new_xpos, new_xquat) = compute_body_pose(model, data, body_id);

    // If sleeping and pose changed: mark for waking via dedicated dirty flag.
    // We use tree_qpos_dirty (not tree_awake) to avoid conflating the
    // qpos-change signal with the derived awake state — see §16.15.2.
    if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
        if new_xpos != data.xpos[body_id] || new_xquat != data.xquat[body_id] {
            data.tree_qpos_dirty[model.body_treeid[body_id]] = true;
            // Don't skip FK — we need the new pose
        }
    }

    // Store new pose
    data.xpos[body_id] = new_xpos;
    data.xquat[body_id] = new_xquat;
    // ... compute xmat from xquat ...
}
```

**Key insight:** The comparison uses exact floating-point equality
(bitwise match), matching MuJoCo. This is correct because if the user
didn't modify `qpos`, the FK computation is deterministic and will
produce exactly the same `xpos`/`xquat`. Any difference indicates
external modification.

After `mj_kinematics1()` completes, `mj_check_qpos_changed()` reads
`tree_qpos_dirty`, wakes affected trees (propagating across sleep
cycles), then clears the dirty flags.

```rust
/// Check if any sleeping tree's qpos was externally modified.
/// Returns true if any tree was newly marked awake.
fn mj_check_qpos_changed(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for t in 0..model.ntree {
        if data.tree_qpos_dirty[t] && data.tree_asleep[t] >= 0 {
            // Tree was sleeping but kinematics1 detected a pose change.
            mj_wake_tree(model, data, t);  // Wake entire island
            woke_any = true;
        }
    }

    // Clear all dirty flags (whether or not they triggered a wake)
    data.tree_qpos_dirty.fill(false);

    woke_any
}
```

###### 16.15.2 Why `tree_qpos_dirty` Instead of `tree_awake`

`tree_awake` is a **derived** array recomputed from `tree_asleep` by
`mj_update_sleep()`. Writing to `tree_awake` directly from
`mj_kinematics1()` creates a transient inconsistency between
`tree_awake` and `tree_asleep` that persists until the next
`mj_update_sleep()` call. This is error-prone because other wake
functions (e.g., `mj_wake_equality`) read `tree_awake` to determine
sleep state.

Using a dedicated `tree_qpos_dirty: Vec<bool>` flag array (length
`ntree`, allocated at `make_data()`, cleared each step) keeps the
qpos-change signal separate from the authoritative sleep state. The
dirty flags are only read by `mj_check_qpos_changed()` and cleared
immediately after.

##### 16.16 Phase B — Per-Island Block-Diagonal Solving

The most significant Phase B performance optimization: instead of
solving the full `nv`-dimensional system, solve each island's
constraint problem independently.

###### 16.16.1 Block-Diagonal Structure

After `mj_island()`, the mass matrix `qM` has a block-diagonal
structure when DOFs are reordered by island. Each island's DOFs form
an independent diagonal block. The constraint Jacobian `efc_J` also
decomposes: each constraint row only references DOFs within its island.

```
Global system: M * qacc = qfrc_total,  J * qacc = efc_aref
              [M_1  0   0 ] [qacc_1]   [f_1]
              [ 0  M_2  0 ] [qacc_2] = [f_2]
              [ 0   0  M_3] [qacc_3]   [f_3]

Per-island:   M_i * qacc_i = f_i,  J_i * qacc_i = efc_aref_i
```

###### 16.16.2 Solver Modification

The constraint solver (PGS, Newton, CG) currently operates on the full
`nv`-dimensional system. With islands, each solver is parameterized to
accept an `IslandView` — a sub-problem scoped to a single island's
DOFs and constraints.

**`IslandView` struct:**

```rust
/// A view into the global Data arrays scoped to a single island.
/// All indices are island-local (0-based within the island).
struct IslandView<'a> {
    island_id: usize,
    nv: usize,                    // island DOF count
    nefc: usize,                  // island constraint row count
    ne: usize,                    // island equality constraint count
    nf: usize,                    // island friction constraint count

    // Island-local mass matrix and factorization (from iM, iLD)
    qM: &'a [f64],               // sparse CSC, island-local indices
    qLD: &'a [f64],              // sparse CSC, island-local indices

    // Island-local Jacobian (from iJ)
    efc_J: &'a [f64],            // dense or sparse, island-local column indices

    // Island-local constraint vectors (gathered from global via map_iefc2efc)
    efc_D: &'a [f64],            // impedance diagonal
    efc_R: &'a [f64],            // regularizer diagonal
    efc_aref: &'a [f64],         // constraint reference acceleration
    efc_frictionloss: &'a [f64], // friction loss coefficients
    efc_type: &'a [i32],         // constraint types

    // Island-local force/result vectors (slices into scratch space)
    qacc: &'a mut [f64],         // output: island-local accelerations
    efc_force: &'a mut [f64],    // output: constraint forces
    qfrc_constraint: &'a mut [f64], // output: J^T * efc_force

    // Warmstart from previous step (gathered from global)
    qacc_warmstart: &'a [f64],
}
```

**Solver dispatch:**

```rust
fn mj_fwd_constraint_islands(model: &Model, data: &mut Data) {
    if data.nisland == 0 {
        // No islands — use global solver (same as Phase A)
        mj_fwd_constraint(model, data);
        return;
    }

    for island in 0..data.nisland {
        let nefc_island = data.island_nefc[island];
        if nefc_island == 0 {
            continue;
        }

        // 1. Gather: build IslandView from global arrays using maps
        let view = gather_island_view(model, data, island);

        // 2. Solve: dispatch to the configured solver
        match model.solver {
            Solver::PGS => pgs_solve_island(&view, model),
            Solver::Newton => newton_solve_island(&view, model),
            Solver::CG => cg_solve_island(&view, model),
        }

        // 3. Track per-island solver statistics
        let stat_idx = island.min(MAX_ISLAND_STATS - 1);
        data.solver_niter[stat_idx] = view.iterations;
        data.solver_nnz[stat_idx] = view.nnz;
    }

    // 4. Scatter: write island-local results back to global arrays
    scatter_island_results(model, data);
}
```

**`gather_island_view()`** builds the `IslandView` by:
1. Extracting slices of `iM`, `iLD`, `iJ` for this island using
   `island_idofadr[island]` and `island_iefcadr[island]` as offsets.
2. Gathering `efc_D`, `efc_R`, `efc_aref`, `efc_frictionloss`,
   `efc_type` using `map_iefc2efc[start..start+nefc]` to index into
   the global arrays.
3. Gathering `qacc_warmstart` for warmstart using
   `map_idof2dof[start..start+nv]`.
4. Allocating island-local output slices (`qacc`, `efc_force`,
   `qfrc_constraint`) from scratch space.

**Per-island result storage:** Solver results are written into
contiguous scratch buffers packed by island. These buffers are
allocated in `make_data()` to worst-case size:

```rust
/// In Data — per-island solver scratch (packed, contiguous):
pub island_qacc: Vec<f64>,           // Length: nv. Island results packed by island_idofadr.
pub island_efc_force: Vec<f64>,      // Length: nefc (worst case). Packed by island_iefcadr.
pub island_qfrc_constraint: Vec<f64>, // Length: nv. Packed by island_idofadr.
```

Each island's results occupy a contiguous slice:
- `island_qacc[island_idofadr[i] .. island_idofadr[i] + island_nv[i]]`
- `island_efc_force[island_iefcadr[i] .. island_iefcadr[i] + island_nefc[i]]`
- `island_qfrc_constraint[island_idofadr[i] .. island_idofadr[i] + island_nv[i]]`

**`scatter_island_results()`** writes results back to global arrays:

```rust
fn scatter_island_results(model: &Model, data: &mut Data) {
    for island in 0..data.nisland {
        let nv = data.island_nv[island];
        let nefc = data.island_nefc[island];
        let dof_start = data.island_idofadr[island];
        let efc_start = data.island_iefcadr[island];

        // Scatter qacc: island-local → global
        for i in 0..nv {
            let global_dof = data.map_idof2dof[dof_start + i];
            data.qacc[global_dof] = data.island_qacc[dof_start + i];
        }

        // Scatter efc_force: island-local → global
        for i in 0..nefc {
            let global_efc = data.map_iefc2efc[efc_start + i];
            data.efc_force[global_efc] = data.island_efc_force[efc_start + i];
        }

        // Scatter qfrc_constraint: island-local → global
        for i in 0..nv {
            let global_dof = data.map_idof2dof[dof_start + i];
            data.qfrc_constraint[global_dof] = data.island_qfrc_constraint[dof_start + i];
        }
    }

    // Unconstrained DOFs (not in any island): qacc comes from the
    // unconstrained forward dynamics. These DOFs have no constraints,
    // so qacc[dof] = qfrc_total[dof] / M[dof,dof] (computed by
    // mj_fwd_acceleration before the constraint solve).
    // The scatter loop leaves these untouched — their qacc values
    // from mj_fwd_acceleration are already correct.
}
```

**Solver refactoring:** The existing `pgs_solve_with_system()`,
`newton_solve()`, and `cg_solve()` currently accept `&Model` and
`&mut Data` and operate on the full `nv`/`nefc` system. The refactoring
extracts the core solve loop into a generic function parameterized by
the matrix dimensions:

```rust
/// Core PGS solve loop, parameterized by system size.
/// Works for both global (nv, nefc) and island-scoped (nv_island, nefc_island).
fn pgs_solve_core(
    nv: usize,
    nefc: usize,
    ne: usize,          // equality constraints
    nf: usize,          // friction constraints
    qM: &[f64],         // mass matrix (sparse CSC)
    qLD: &[f64],        // LDL factorization
    efc_J: &[f64],      // constraint Jacobian
    efc_D: &[f64],      // impedance diagonal
    efc_R: &[f64],      // regularizer
    efc_aref: &[f64],   // reference acceleration
    efc_frictionloss: &[f64],
    qacc: &mut [f64],    // output
    efc_force: &mut [f64], // output
    qfrc_constraint: &mut [f64], // output
    warmstart: &[f64],   // initial qacc guess
    max_iter: usize,
    tolerance: f64,
) -> usize {  // returns iteration count
    // Initialize from warmstart
    qacc.copy_from_slice(&warmstart[..nv]);

    for iter in 0..max_iter {
        // Standard PGS iteration over constraint rows
        // (identical to existing logic, but using local nv/nefc dimensions
        //  and the island-scoped matrices passed as arguments)
        // ...
        if converged(tolerance) { return iter + 1; }
    }
    max_iter
}
```

The Newton and CG solvers follow the same pattern: extract the core
algorithm into a dimension-parameterized function that accepts matrix
slices. The outer `newton_solve_island()` / `cg_solve_island()` wrappers
construct the arguments from `IslandView` and call the core function.

**Warmstart:** Per-island warmstart gathers `qacc_warmstart` values for
the island's DOFs from the global `data.qacc_warmstart` using
`map_idof2dof`. After solving, the island's `qacc` results are
scattered back to `data.qacc_warmstart` for the next step.

###### 16.16.3 Solver Statistics

MuJoCo stores per-island solver statistics:

```rust
/// Per-island solver statistics. MuJoCo uses mjNISLAND = 20 as the
/// maximum number of islands tracked for solver statistics.
pub const MAX_ISLAND_STATS: usize = 20;

/// In Data:
pub solver_niter: [usize; MAX_ISLAND_STATS],  // iterations per island
pub solver_nnz: [usize; MAX_ISLAND_STATS],     // Jacobian nonzeros per island
```

Islands beyond `MAX_ISLAND_STATS` share the last statistics slot.

###### 16.16.4 CRBA Optimization

Phase A computes CRBA for all bodies. Phase B can skip CRBA for
sleeping islands because their mass matrix entries don't change.

**Key insight:** CRBA uses a backward accumulation pass:
`crb[parent] += crb[child]`. For sleeping bodies, `crb[body]` hasn't
changed since the last awake step. Rather than preserving a `crb_saved`
array, we use the fact that sleeping bodies' `cinert` (composite inertia
in global frame) hasn't changed — the CRBA result depends only on
`cinert` and the kinematic chain, both frozen for sleeping bodies.

```rust
fn mj_crba_islands(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // If all bodies awake or sleep disabled: full CRBA (Phase A path)
    if !sleep_enabled || data.nbody_awake == model.nbody {
        mj_crba(model, data);
        return;
    }

    // Phase B selective CRBA:
    // 1. Do NOT zero the full qM — only zero entries for awake DOF pairs.
    //    Sleeping DOFs' qM entries are preserved from their last awake step.
    for idx_i in 0..data.nv_awake {
        let dof_i = data.dof_awake_ind[idx_i];
        // Zero qM entries in this DOF's row/column
        // (sparse format: iterate nonzeros in column dof_i, zero those
        //  where the row DOF is also awake)
        zero_qm_column_awake(data, dof_i);
    }

    // 2. Backward CRBA accumulation for awake bodies only.
    //    Use body_awake_ind in reverse order (leaf-to-root among awake bodies).
    //    Sleeping children's cinert contributions are already baked into
    //    their parents' qM from the last step they were awake.
    for idx in (0..data.nbody_awake).rev() {
        let body_id = data.body_awake_ind[idx];
        if body_id == 0 { continue; }  // world body

        // Standard CRBA: compute crb[body_id], accumulate to parent,
        // project onto DOFs to fill qM entries.
        crba_body(model, data, body_id);
    }

    // 3. LDL factorization for awake DOF blocks only.
    //    Sleeping DOFs' qLD entries are preserved.
    ldl_factor_awake(model, data);
}
```

**Correctness argument:** When a tree wakes, its first `forward()` pass
runs full CRBA (because `nbody_awake` now includes the waking bodies),
so the mass matrix is recomputed correctly. While the tree sleeps, its
`qM` entries are stale but consistent — the sleeping DOFs have zero
velocity and zero force, so the solver produces zero acceleration
regardless of the mass matrix values.

This optimization reduces CRBA cost from O(nbody) to O(nbody_awake).

##### 16.17 Phase B — Awake-Index Indirection Arrays

Phase A uses `body_sleep_state[body_id] == SleepState::Asleep` checks
in per-body loops. This causes branch mispredictions when awake and
sleeping bodies are interleaved. Phase B adds indirection arrays for
cache-friendly iteration.

###### 16.17.1 mj_update_sleep (Phase B version)

`mj_update_sleep()` recomputes the awake-index arrays:

```rust
fn mj_update_sleep(model: &Model, data: &mut Data) {
    data.ntree_awake = 0;
    data.nv_awake = 0;

    // Tree awake flags
    for t in 0..model.ntree {
        let awake = data.tree_asleep[t] < 0;
        data.tree_awake[t] = awake;
        if awake {
            data.ntree_awake += 1;
        }
    }

    // Body awake states + body_awake_ind + parent_awake_ind
    let mut nbody_awake = 0;
    let mut nparent_awake = 0;

    data.body_sleep_state[0] = SleepState::Static;
    // World body is always in body_awake_ind (it's "static", included)
    data.body_awake_ind[nbody_awake] = 0;
    nbody_awake += 1;
    data.parent_awake_ind[nparent_awake] = 0;
    nparent_awake += 1;

    for body_id in 1..model.nbody {
        let tree = model.body_treeid[body_id];
        let awake = tree < model.ntree && data.tree_awake[tree];

        data.body_sleep_state[body_id] = if awake {
            SleepState::Awake
        } else {
            SleepState::Asleep
        };

        // Include in body_awake_ind if awake
        // (body 0 / Static is handled above; bodies > 0 are Awake or Asleep)
        if awake {
            data.body_awake_ind[nbody_awake] = body_id;
            nbody_awake += 1;
        }

        // Include in parent_awake_ind if parent is awake or static
        let parent = model.body_parent[body_id];
        let parent_awake = data.body_sleep_state[parent] != SleepState::Asleep;
        if parent_awake {
            data.parent_awake_ind[nparent_awake] = body_id;
            nparent_awake += 1;
        }
    }

    // DOF awake indices
    let mut nv_awake = 0;
    for dof in 0..model.nv {
        let tree = model.dof_treeid[dof];
        if tree < model.ntree && data.tree_awake[tree] {
            data.dof_awake_ind[nv_awake] = dof;
            nv_awake += 1;
        }
    }

    data.nbody_awake = nbody_awake;
    data.nparent_awake = nparent_awake;
    data.nv_awake = nv_awake;
}
```

###### 16.17.2 Pipeline Iteration Pattern

Phase A:
```rust
for body_id in 1..model.nbody {
    if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
        continue;
    }
    // ... compute ...
}
```

Phase B (indirection):
```rust
let sleep_filter = sleep_enabled && data.nbody_awake < model.nbody;
let nbody = if sleep_filter { data.nbody_awake } else { model.nbody };

for b in 0..nbody {
    let body_id = if sleep_filter { data.body_awake_ind[b] } else { b };
    // ... compute (no branch needed) ...
}
```

This eliminates per-body branches entirely. The `body_awake_ind` array
contains only awake body indices, so the loop body executes for every
iteration. Cache behavior is also improved: the awake bodies are
accessed in sorted order.

The same pattern applies to `dof_awake_ind` for per-DOF loops and
`parent_awake_ind` for backward-pass loops.

##### 16.18 Phase B — Policy Relaxation

Phase A assigns `AutoNever` to all actuated trees, preventing them
from sleeping. Phase B relaxes this:

###### 16.18.1 User Override for Actuated Trees

If a user explicitly sets `sleep="allowed"` or `sleep="init"` on an
actuated tree, Phase B respects this. The actuator's activation dynamics
still run (the `act` state tracks `ctrl`), but the tree can sleep when
velocity is below threshold. The key requirement: when the tree sleeps,
`qfrc_actuator` for its DOFs must be zeroed (already done by
`put_tree_to_sleep`). When the tree wakes, the actuator force is
recomputed from the current `act` state.

**Validation:** If a tree has a multi-tree tendon (spanning other trees),
`Allowed`/`Init` is still an error — the tendon coupling requires
island-level sleep, and the tree cannot be in an island with trees that
have `Never` policy.

###### 16.18.2 Multi-Tree Tendon Policy

Phase A marks trees with multi-tree tendons as `AutoNever`. Phase B
refines this: if the tendon has zero stiffness and zero damping
(purely geometric, no passive forces), the trees can sleep. Only
tendons with nonzero stiffness, damping, or active limits force
`AutoNever`.

##### 16.19 Phase B — Acceptance Criteria

**Island Discovery:**
1. `mj_island()` correctly groups constraint-coupled trees into islands.
   Two-body contact: both trees in the same island. Three-body chain:
   all three trees in one island.
2. Singleton trees (no constraints) get `tree_island = -1`.
3. `DISABLE_ISLAND` flag causes `nisland = 0` (Phase A fallback).
4. Island arrays are consistent: `sum(island_ntree) == number of
   constrained trees`, `sum(island_nv) == nidof`.

**Sleep Cycle:**
5. Sleeping a 3-tree island creates cycle: `tree_asleep[a] = b`,
   `tree_asleep[b] = c`, `tree_asleep[c] = a`.
6. Waking any tree in the cycle wakes all three.
7. Single-tree sleep cycle: `tree_asleep[t] = t` (Phase A compatible).

**Cross-Island Wake:**
8. Contact between sleeping tree and awake tree: sleeping tree's entire
   island wakes.
9. Active equality constraint between sleeping and awake trees: sleeping
   tree's entire island wakes.
10. Active limited tendon between sleeping and awake trees: sleeping
    tree's entire island wakes.
11. Two sleeping trees in different islands coupled by equality
    constraint: both islands wake.

**dof_length:**
12. Hinge joint on 1-meter arm: `dof_length ≈ 1.0`.
13. Hinge joint on 0.1-meter arm: `dof_length ≈ 0.1`.
14. Slide joint: `dof_length = 1.0` regardless of arm length.
15. Free joint: translational DOFs = 1.0, rotational DOFs = body length.

**qpos Change Detection:**
16. Externally modifying `qpos` of a sleeping body wakes its tree.
17. Stepping without modifying `qpos` does NOT wake (FK is deterministic).

**Per-Island Solve:**
18. Per-island solve produces identical `qacc` as global solve (within
    floating-point tolerance).
19. Scene with 2 independent islands: solve time scales with the larger
    island, not the total system.

**Awake-Index Indirection:**
20. Pipeline produces identical results whether using indirection arrays
    or Phase A per-body skip checks.

**Regression:**
21. All Phase A tests (T1–T28) pass unchanged.
22. All existing sim domain tests pass unchanged.
23. `DISABLE_ISLAND` produces bit-identical results to Phase A.
24. `DISABLE_ISLAND` produces no performance regression relative to
    Phase A (island overhead is zero when disabled).

**Policy Relaxation:**
25. Actuated tree with explicit `sleep="allowed"`: tree sleeps when
    velocity drops below threshold (unlike Phase A `AutoNever`).
26. Multi-tree tendon with zero stiffness and zero damping: trees
    are NOT forced to `AutoNever` (can sleep).
27. Multi-tree tendon with nonzero stiffness: trees forced to
    `AutoNever` (cannot sleep via auto policy).

**Init-Sleep Validation:**
28. Init-sleep tree with valid DOFs: passes validation, starts asleep.
29. Mixed island (some Init, some non-Init trees): returns
    `SleepError::InitSleepMixedIsland`.

**Public API:**
30. `mj_sleep_state(data, body_id)` returns correct `SleepState` for
    static, sleeping, and awake bodies.
31. `mj_tree_awake(data, tree_id)` returns `false` for sleeping trees,
    `true` for awake trees.
32. `mj_nisland(data)` returns the number of islands discovered this step.

**make_data / reset:**
33. After `make_data()`, all island arrays are allocated to correct
    worst-case bounds (lengths match `ntree`, `nv`, `nefc`).
34. After `Data::reset()`, sleep state is re-initialized from policies
    and awake-index arrays are recomputed.

**Performance:**
35. Scene with 100 resting bodies in 10 islands, 1 active body: step
    time is measurably faster than Phase A (per-island solve + CRBA skip
    reduces work from O(100 DOFs) to O(~10 DOFs) in the solver).

##### 16.20 Phase B — Test Plan

| # | Test | Type | Validates |
|---|------|------|-----------|
| T31 | `test_island_discovery_two_body_contact` | Unit | AC #1: contact groups 2 trees |
| T32 | `test_island_discovery_chain` | Unit | AC #1: chain of 3 contacts → 1 island |
| T33 | `test_island_singleton` | Unit | AC #2: unconstrained tree → island = -1 |
| T34 | `test_disable_island_flag` | Unit | AC #3: DISABLE_ISLAND → nisland = 0 |
| T35 | `test_island_array_consistency` | Unit | AC #4: sum checks |
| T36 | `test_sleep_cycle_three_trees` | Unit | AC #5: circular linked list |
| T37 | `test_wake_cycle_propagation` | Unit | AC #6: wake one → all wake |
| T38 | `test_sleep_cycle_single_tree` | Unit | AC #7: self-link |
| T39 | `test_wake_contact_island` | Integration | AC #8: contact wakes entire island |
| T40 | `test_wake_equality_island` | Integration | AC #9: equality wakes island |
| T41 | `test_wake_tendon_island` | Integration | AC #10: tendon wakes island |
| T42 | `test_wake_two_sleeping_islands` | Integration | AC #11: both islands wake |
| T43 | `test_dof_length_hinge_1m` | Unit | AC #12: hinge on 1m arm |
| T44 | `test_dof_length_hinge_01m` | Unit | AC #13: hinge on 0.1m arm |
| T45 | `test_dof_length_slide` | Unit | AC #14: slide = 1.0 |
| T46 | `test_dof_length_free_joint` | Unit | AC #15: free joint mixed |
| T47 | `test_qpos_change_wakes` | Integration | AC #16: external qpos mod wakes |
| T48 | `test_qpos_stable_no_wake` | Integration | AC #17: deterministic FK → no wake |
| T49 | `test_per_island_solve_equivalence` | Unit | AC #18: matches global solve |
| T50 | `test_per_island_solve_scaling` | Benchmark | AC #19: scales with island size |
| T51 | `test_indirection_equivalence` | Unit | AC #20: same results as Phase A skip |
| T52 | `test_phase_a_regression` | Regression | AC #21: all T1–T28 pass |
| T53 | `test_sim_domain_regression` | Regression | AC #22: all existing sim tests pass |
| T54 | `test_disable_island_bit_identical` | Unit | AC #23: DISABLE_ISLAND = Phase A |
| T55 | `test_disable_island_no_perf_regression` | Benchmark | AC #24: DISABLE_ISLAND same perf as Phase A |
| T56 | `test_policy_relaxation_actuated_allowed` | Integration | AC #25: actuated tree with `sleep="allowed"` can sleep |
| T57 | `test_policy_relaxation_tendon_zero_stiffness` | Unit | AC #26: zero-stiffness tendon allows sleep |
| T58 | `test_policy_relaxation_tendon_nonzero_stiffness` | Unit | AC #27: nonzero-stiffness tendon forces AutoNever |
| T59 | `test_init_sleep_valid` | Unit | AC #28: valid Init-sleep passes validation |
| T60 | `test_init_sleep_mixed_island_error` | Unit | AC #29: mixed Init/non-Init in coupled group errors |
| T61 | `test_mj_sleep_state_api` | Unit | AC #30: mj_sleep_state returns correct states |
| T62 | `test_mj_tree_awake_api` | Unit | AC #31: mj_tree_awake correct |
| T63 | `test_mj_nisland_api` | Unit | AC #32: mj_nisland returns island count |
| T64 | `test_make_data_island_array_sizes` | Unit | AC #33: array allocations correct |
| T65 | `test_reset_reinitializes_sleep` | Unit | AC #34: reset re-inits from policies |
| T66 | `test_island_solve_performance` | Benchmark | AC #35: faster than Phase A |

##### 16.21 Phase B — Files

| File | Action | Description |
|------|--------|-------------|
| `sim/L0/core/src/mujoco_pipeline.rs` | Modify | Island discovery (§16.11), sleep cycle (§16.12), cross-island wake (§16.13), dof_length (§16.14), qpos detection (§16.15), per-island solve (§16.16), awake-index arrays (§16.17), policy relaxation (§16.18) |
| `sim/L0/core/src/lib.rs` | Modify | Export new types, DISABLE_ISLAND constant |
| `sim/L0/mjcf/src/model_builder.rs` | Modify | dof_length computation (§16.14), tendon tree enumeration for wake (§16.13.2) |
| `sim/L0/mjcf/src/parser.rs` | Modify | Parse `<flag island="enable\|disable"/>` attribute |
| `sim/L0/tests/integration/sleeping.rs` | Modify | Add tests T31–T65 (excluding benchmarks) |
| `sim/L0/core/benches/sleep_benchmarks.rs` | Modify | Add island benchmarks T50, T55, T66 |

##### 16.22 Phase B — Implementation Order

1. **Phase A errata fixes** (§16.26) — correctness-first:
   a. Deformable body policy guard (§16.26.4).
   b. Tendon-actuator policy resolution (§16.26.5).
   c. `MjObjectType::Xbody` sensor skip (§16.26.6).
2. **dof_length** (§16.14) — standalone, no dependencies, immediate
   correctness improvement. Replaces uniform `1.0`.
3. **Awake-index arrays** (§16.17) — improves Phase A performance,
   no semantic change.
4. **make_data / reset Phase B fields** (§16.23.5) — allocate island
   arrays, update init logic. Required before island discovery.
5. **qpos change detection** (§16.15) — correctness fix for external
   qpos modification.
6. **Island discovery** (§16.11) — the core algorithm. Requires
   constraint Jacobian to be populated.
7. **Sleep cycle linked list** (§16.12) — requires island discovery.
8. **Cross-island wake** (§16.13) — requires sleep cycles.
9. **Init-sleep island validation** (§16.24) — requires island discovery.
10. **Per-island solve** (§16.16) — requires island discovery + block
    diagonal extraction. Largest effort item.
11. **Policy relaxation** (§16.18) — must include `qfrc_actuator`
    zeroing (§16.26.7) when actuated trees can sleep.
12. **Public API** (§16.25) — trivial wrappers, add after core is stable.

##### 16.23 Phase B — Phase A→B Migration Guide

This section documents the exact changes from Phase A to Phase B for
each affected function, field, and semantic.

###### 16.23.1 Function Renames and Splits

| Phase A | Phase B | Change |
|---------|---------|--------|
| `mj_update_sleep()` (sleep transitions + array recomputation) | `mj_sleep()` (transitions only) + `mj_update_sleep()` (arrays only) | Split into two functions. Phase A combined sleep transitions and array recomputation in one function. Phase B separates them: `mj_sleep()` handles countdown and island-level sleep decisions; `mj_update_sleep()` recomputes `body_awake_ind`, `dof_awake_ind`, `parent_awake_ind`, `body_sleep_state`. |
| `wake_tree()` (internal) | `mj_wake_tree()` (public) | Rename + cycle traversal. Phase A wakes a single tree. Phase B traverses the sleep cycle to wake all trees in the island. |
| `put_tree_to_sleep()` | `sleep_trees()` | Generalized to accept a slice of tree indices for multi-tree island sleep. Single-tree case is backward compatible. |
| `mj_wake_collision()` | `mj_wake_collision()` (unchanged name) | Same interface. Now calls `mj_wake_tree()` which propagates across cycles. |
| N/A | `mj_wake_tendon()` | New function (§16.13.2). |
| N/A | `mj_wake_equality()` | New function (§16.13.3). |
| N/A | `mj_island()` | New function (§16.11). |
| N/A | `mj_fwd_constraint_islands()` | New function (§16.16.2). Wraps solver dispatch with per-island gather/scatter. |
| N/A | `mj_crba_islands()` | New function (§16.16.4). Replaces `mj_crba()` for sleep-aware CRBA. |
| N/A | `mj_check_qpos_changed()` | New function (§16.15). Uses `tree_qpos_dirty` flag array (§16.15.2). |

###### 16.23.2 Field Additions

| Location | New Fields | Notes |
|----------|-----------|-------|
| Model | `DISABLE_ISLAND`, `tendon_treenum`, `tendon_tree` | §16.10.1 |
| Data | Island arrays (`nisland`, `nidof`, `tree_island`, etc.) | §16.10.2, 25 new fields |
| Data | Block-diagonal copies (`iM`, `iLD`, `iJ`, `iefc_*`) | §16.10.2 |
| Data | Per-island solver scratch (`island_qacc`, `island_efc_force`, `island_qfrc_constraint`) | §16.10.2 / §16.16.3 |
| Data | Island discovery scratch (`island_scratch_stack`, `_rownnz`, `_rowadr`, `_colind`) | §16.10.2 / §16.11.3 |
| Data | Awake-index arrays (`body_awake_ind`, `nbody_awake`, etc.) | §16.10.2 |
| Data | qpos change detection (`tree_qpos_dirty`) | §16.15.2 |
| Data | Solver stats (`solver_niter`, `solver_nnz`) | §16.16.3 |

###### 16.23.3 Semantic Changes

1. **`tree_asleep` encoding:** Phase A uses self-link (`tree_asleep[t] = t`)
   for sleeping. Phase B uses circular linked list among island trees.
   Single-tree islands still self-link (backward compatible).

2. **`step()` call sequence:** Phase A calls `mj_update_sleep()` after
   integrate. Phase B calls `mj_sleep()` then `mj_update_sleep()`.
   See §16.9b for the complete Phase B `step()`.

3. **`forward()` pipeline:** Phase A has `mj_wake` + `mj_wake_collision`.
   Phase B adds `mj_wake_tendon`, `mj_wake_equality`, `mj_check_qpos_changed`,
   `mj_island`, and `mj_fwd_constraint_islands`. Phase B also explicitly
   includes `mj_tendon`, `mj_transmission`, `mj_com_vel`, `mj_fwd_passive`,
   `mj_energy`, and sensor stages (`mj_sensor_pos`, `mj_sensor_vel`,
   `mj_sensor_acc`). See §16.9b for the complete Phase B `forward()`
   — it lists ALL stages (none omitted).

4. **CRBA:** Phase A runs full CRBA. Phase B optionally runs selective
   CRBA via `mj_crba_islands()` (§16.16.4).

5. **dof_length:** Phase A sets all `dof_length = 1.0`. Phase B computes
   mechanism lengths for rotational DOFs (§16.14).

6. **Policy for actuated trees:** Phase A: always `AutoNever`. Phase B:
   `AutoNever` unless user explicitly overrides with `sleep="allowed"`
   or `sleep="init"` (§16.18).

###### 16.23.4 Code That Phase B Replaces

Phase A code that is **deleted** (not just extended):
- The `mj_update_sleep()` sleep-transition logic (countdown + put-to-sleep)
  → replaced by `mj_sleep()`.
- The `dof_length = 1.0` initialization loop in `model_builder.rs`
  → replaced by `compute_dof_lengths()`.

Phase A code that is **extended** (backward compatible):
- `mj_wake_collision()` — same interface, but now calls `mj_wake_tree()`
  which handles cycle traversal.
- `Data::new()` / `Data::reset()` — same sleep initialization, plus
  new island array allocation/zeroing.
- `forward()` / `forward_skip_sensors()` — same structure, with new
  wake and island steps inserted.

###### 16.23.5 make_data() and reset() for Phase B Fields

**`make_data()` additions** — allocate all Phase B arrays to worst-case
bounds from the Model:

```rust
// In Model::make_data():

// Island discovery arrays
data.tree_island = vec![-1_i32; model.ntree];
data.island_ntree = vec![0_usize; model.ntree];  // worst case: each tree is its own island
data.island_itreeadr = vec![0_usize; model.ntree];
data.map_itree2tree = vec![0_usize; model.ntree];
data.dof_island = vec![-1_i32; model.nv];
data.island_nv = vec![0_usize; model.ntree];
data.island_idofadr = vec![0_usize; model.ntree];
data.island_dofadr = vec![0_usize; model.ntree];
data.map_dof2idof = vec![-1_i32; model.nv];
data.map_idof2dof = vec![0_usize; model.nv];
data.efc_island = Vec::new();  // resized after constraint assembly (nefc varies)
data.island_ne = vec![0_usize; model.ntree];
data.island_nf = vec![0_usize; model.ntree];
data.island_nefc = vec![0_usize; model.ntree];
data.island_iefcadr = vec![0_usize; model.ntree];
data.map_efc2iefc = Vec::new();  // resized per step
data.map_iefc2efc = Vec::new();  // resized per step

// Block-diagonal copies (upper bound = global size)
data.iM = vec![0.0_f64; nnz_qM];   // same nnz as global qM
data.iLD = vec![0.0_f64; nnz_qLD];  // same nnz as global qLD
data.iJ = Vec::new();               // resized per step (depends on nefc)
data.iefc_type = Vec::new();
data.iefc_D = Vec::new();
data.iefc_R = Vec::new();
data.iefc_frictionloss = Vec::new();

// Awake-index arrays
data.body_awake_ind = vec![0_usize; model.nbody];
data.parent_awake_ind = vec![0_usize; model.nbody];
data.dof_awake_ind = vec![0_usize; model.nv];
data.nbody_awake = model.nbody;      // initially all awake
data.nparent_awake = model.nbody;
data.nv_awake = model.nv;

// Solver statistics
data.solver_niter = [0_usize; MAX_ISLAND_STATS];
data.solver_nnz = [0_usize; MAX_ISLAND_STATS];

// qpos change detection (§16.15)
data.tree_qpos_dirty = vec![false; model.ntree];

// Island scratch space (for flood fill)
data.island_scratch_stack = vec![0_usize; model.ntree];
data.island_scratch_rownnz = vec![0_usize; model.ntree];
data.island_scratch_rowadr = vec![0_usize; model.ntree];
// Edge list: worst case = nefc edges (each constraint creates one edge)
data.island_scratch_colind = Vec::new();  // resized per step

// Per-island solver result buffers (packed by island, see §16.16.2)
data.island_qacc = vec![0.0_f64; model.nv];
data.island_efc_force = Vec::new();           // resized per step (nefc varies)
data.island_qfrc_constraint = vec![0.0_f64; model.nv];
```

**`Data::new()` Phase B additions** — after Phase A sleep initialization:

```rust
// Phase A sleep init (existing):
for t in 0..model.ntree {
    match model.tree_sleep_policy[t] {
        SleepPolicy::Init => { /* ... set asleep, zero qvel ... */ }
        _ => { data.tree_asleep[t] = -(1 + MIN_AWAKE); }
    }
}

// Phase B additions:
// Initialize awake-index arrays from initial sleep state
mj_update_sleep(model, &mut data);

// For Init-sleep trees: run FK so xpos/xquat are valid before
// the first step (sleeping bodies skip FK, so their poses must be
// computed during init). Only needed if any trees start asleep.
if data.ntree_awake < model.ntree {
    mj_kinematics1(model, &mut data);
    mj_kinematics2(model, &mut data);
}
```

**`Data::reset(model)` and `Data::reset_to_keyframe(model, key)`:**
Both call the same Phase A sleep re-initialization (§16.7) followed by
the Phase B awake-index recomputation and conditional FK above.

##### 16.24 Phase B — Init-Sleep Island Validation

MuJoCo validates Init-sleep trees during `_resetData()`: if an
Init-sleep tree cannot actually be put to sleep (because it fails the
`tree_can_sleep` check), an error is raised. Phase B adds this
validation.

###### 16.24.1 Algorithm

**Important timing constraint:** Full island discovery (`mj_island()`)
requires constraint assembly, which requires a complete `forward()` pass.
At `make_data()` / `reset()` time, no constraints have been assembled and
`nisland = 0`. Therefore, Init-sleep validation CANNOT use island arrays.

Instead, validation uses a **model-time adjacency check** based on the
constraint coupling that WILL exist at runtime. The coupling sources are
known at model construction time:
- **Equality constraints:** `eq_obj1id` / `eq_obj2id` → tree pairs.
- **Multi-tree tendons:** `tendon_treenum == 2` → tree pairs.
- **Contacts** are dynamic (depend on pose) and cannot be predicted at
  init time — but this is acceptable because contact-based islands are
  transient.

```rust
fn validate_init_sleep(model: &Model, data: &mut Data) -> Result<(), SleepError> {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return Ok(());
    }

    // Phase 1: Basic per-tree validation
    for t in 0..model.ntree {
        if model.tree_sleep_policy[t] != SleepPolicy::Init {
            continue;
        }
        if model.tree_dof_num[t] == 0 {
            return Err(SleepError::InitSleepInvalidTree { tree: t });
        }
    }

    // Phase 2: Check for mixed Init/non-Init in statically-coupled tree groups.
    // Build a union-find over trees connected by equality constraints and
    // multi-tree tendons. Then verify that each group is all-Init or all-non-Init.
    let mut uf = UnionFind::new(model.ntree);

    // Equality constraint edges
    for eq in 0..model.neq {
        if !model.eq_active0[eq] { continue; }  // initially-disabled constraint
        let (tree_a, tree_b) = equality_trees(model, eq);
        if tree_a != tree_b {
            uf.union(tree_a, tree_b);
        }
    }

    // Multi-tree tendon edges (only tendons with stiffness/damping/limits)
    for t in 0..model.ntendon {
        if model.tendon_treenum[t] == 2 {
            let tree_a = model.tendon_tree[2 * t];
            let tree_b = model.tendon_tree[2 * t + 1];
            uf.union(tree_a, tree_b);
        }
    }

    // Check each group for mixed Init/non-Init
    let mut group_has_init: HashMap<usize, bool> = HashMap::new();
    let mut group_has_noninit: HashMap<usize, bool> = HashMap::new();
    for t in 0..model.ntree {
        let root = uf.find(t);
        let is_init = model.tree_sleep_policy[t] == SleepPolicy::Init;
        if is_init {
            group_has_init.insert(root, true);
        } else {
            group_has_noninit.insert(root, true);
        }
    }
    for (&root, _) in &group_has_init {
        if group_has_noninit.get(&root) == Some(&true) {
            return Err(SleepError::InitSleepMixedIsland { group_root: root });
        }
    }

    // Phase 3: For validated Init-sleep tree groups, create sleep cycles.
    // Group Init trees by their union-find root and create a cycle per group.
    let mut init_groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for t in 0..model.ntree {
        if model.tree_sleep_policy[t] == SleepPolicy::Init {
            let root = uf.find(t);
            init_groups.entry(root).or_default().push(t);
        }
    }
    for (_root, trees) in &init_groups {
        sleep_trees(model, data, trees);
    }

    Ok(())
}
```

###### 16.24.2 Error Types

```rust
#[derive(Debug)]
pub enum SleepError {
    /// Init-sleep tree has no DOFs (cannot meaningfully sleep).
    InitSleepInvalidTree { tree: usize },
    /// Statically-coupled tree group contains a mix of Init and non-Init
    /// trees. All trees connected by equality constraints or multi-tree
    /// tendons must have the same Init policy. `group_root` is the
    /// union-find representative tree.
    InitSleepMixedIsland { group_root: usize },
}
```

##### 16.25 Phase B — mj_sleepState() Public API

Expose sleep state for external inspection (RL observation, debugging):

```rust
/// Query the sleep state of a body.
///
/// Returns `SleepState::Static` for the world body (body 0),
/// `SleepState::Asleep` for sleeping bodies, `SleepState::Awake`
/// for active bodies.
///
/// This is the recommended way to check sleep state — reading
/// `data.body_sleep_state[body_id]` directly is valid but this
/// function adds bounds checking and is stable across Phase A/B.
pub fn mj_sleep_state(data: &Data, body_id: usize) -> SleepState {
    if body_id >= data.body_sleep_state.len() {
        panic!("body_id {} out of range (nbody={})", body_id, data.body_sleep_state.len());
    }
    data.body_sleep_state[body_id]
}

/// Query whether a kinematic tree is awake.
pub fn mj_tree_awake(data: &Data, tree_id: usize) -> bool {
    data.tree_asleep[tree_id] < 0
}

/// Query the number of awake bodies (including the world body).
pub fn mj_nbody_awake(data: &Data) -> usize {
    data.nbody_awake
}

/// Query the number of constraint islands discovered this step.
pub fn mj_nisland(data: &Data) -> usize {
    data.nisland
}
```

##### 16.26 Phase B — Phase A Errata and Additional Polish

Items discovered during Phase A audit that must be addressed in Phase B
for A+ parity with MuJoCo.

###### 16.26.1 Spec Erratum: Contact Wake with Static Bodies

**Original spec (§16.4):** Said sleeping bodies should wake on contact
with `Awake | Static` partners. **Corrected:** Only `Awake` partners
trigger contact-wake. Static bodies (world/ground) do NOT wake sleeping
bodies through contact. Rationale: a body resting on a ground plane has
persistent contact with it; if Static contacts woke bodies, nothing
resting on the ground could ever sleep. The Phase A implementation
already follows the correct behavior. The spec pseudocode in §16.4 has
been corrected.

###### 16.26.2 `dof_length` Mechanism Length (Phase A → Phase B migration)

Phase A sets `dof_length[dof] = 1.0` for all DOFs (both translational
and rotational). This means the sleep threshold for rotational DOFs is
`sleep_tolerance` in **rad/s** directly, without scale normalization.
For most models this is acceptable, but for models with vastly different
arm lengths it produces inconsistent sleep behavior: a 10-meter crane
arm and a 10-cm gripper finger have the same angular threshold, even
though the crane's tip moves 100× faster per rad/s.

Phase B must replace the uniform `dof_length = 1.0` with the
mechanism-length computation in §16.14. This is listed in the Phase B
implementation order (step 1) but should be treated as a **correctness
item**, not just an optimization.

**Test to add:** `test_dof_length_nonuniform_sleep_threshold` — a model
with a 0.1m arm and a 1.0m arm at rest: the shorter arm should require
smaller angular velocity to sleep (tighter threshold).

###### 16.26.3 `reset_sleep_state` Zeroing of DOF Velocities

Phase A's `reset_sleep_state()` only resets the `tree_asleep` timers
and recomputes derived arrays. It does NOT zero `qvel` for Init-sleep
trees. This is fine for `Data::reset()` (which zeros all qvel) and
`make_data()` (which initializes qvel to zero). However, if
`reset_sleep_state()` is ever called independently (e.g., by Phase B's
`mj_update_sleep()`), Init-sleep trees could end up "asleep" with
nonzero velocities.

Phase B should either:
- Ensure `reset_sleep_state()` also zeros qvel for Init-sleep trees, or
- Document that `reset_sleep_state()` is only safe to call after qvel
  has been zeroed for Init trees.

###### 16.26.4 Deformable Body Policy Guard

The spec (§16.0) mentions that trees containing deformable bodies should
receive `AutoNever` policy, gated on `#[cfg(feature = "deformable")]`.
The Phase A implementation in `model_builder.rs` does NOT implement
this check — deformable bodies are not scanned during policy resolution.
Phase B must add this guard:

```rust
#[cfg(feature = "deformable")]
{
    for body_id in 1..nbody {
        if has_deformable_body(model, body_id) {
            let tree = model.body_treeid[body_id];
            if tree < model.ntree {
                model.tree_sleep_policy[tree] = SleepPolicy::AutoNever;
            }
        }
    }
}
```

**Test to add:** `test_deformable_body_prevents_sleep` — a tree with a
deformable body gets `AutoNever` policy and never sleeps.

###### 16.26.5 Tendon-Actuator Policy Resolution

Phase A's policy resolution handles `ActuatorTransmission::Joint` and
`ActuatorTransmission::Site` but marks `ActuatorTransmission::Tendon`
as `None` with the comment "Phase B concern". Phase B must resolve this:
when an actuator targets a tendon, all trees spanned by that tendon
should receive `AutoNever` (or the appropriate relaxed policy per
§16.18).

**Test to add:** `test_tendon_actuator_policy_resolution` — an actuator
targeting a tendon causes the tendon's spanning trees to receive
`AutoNever`.

###### 16.26.6 `MjObjectType::Xbody` in Sensor Skip

The Phase A spec (§16.5d) lists `MjObjectType::Xbody` as a sensor
object type that maps to `Some(objid)`. The implementation's
`sensor_body_id()` function does NOT handle `Xbody` — it falls through
to the default `None` case. If any sensor uses `Xbody` as its object
type, it would not be skipped when sleeping (it would always be
computed). Phase B should add the `Xbody` case:

```rust
MjObjectType::Xbody => Some(objid),
```

Or, if `Xbody` is not a variant of `MjObjectType`, document why it's
not needed.

###### 16.26.7 `qfrc_actuator` Zeroing at Sleep Time

`put_tree_to_sleep()` zeros `qfrc_bias`, `qfrc_passive`, and
`qfrc_constraint` for sleeping DOFs. It does NOT zero `qfrc_actuator`.
In Phase A this is safe because actuated trees have `AutoNever` policy
(they never sleep). In Phase B, with policy relaxation (§16.18) allowing
actuated trees to sleep with explicit `sleep="allowed"`, the
`qfrc_actuator` values for sleeping DOFs must also be zeroed at sleep
time to prevent stale forces from persisting.

Phase B must extend `put_tree_to_sleep()` (or its successor
`sleep_trees()`) to also zero `qfrc_actuator[dof]` for sleeping DOFs.

**Test to add:** `test_actuated_tree_sleep_zeros_qfrc_actuator` — an
actuated tree with `sleep="allowed"` that sleeps has
`qfrc_actuator[dof] == 0.0` for all its DOFs.

###### 16.26.8 Energy Computation and Sleeping Bodies

`mj_energy_pos()` and `mj_energy_vel()` are called in `forward_core()`
without any sleep gating. For sleeping bodies:
- Kinetic energy should be zero (qvel = 0).
- Potential energy contribution should be frozen (pose unchanged).

Phase A computes energy for all bodies (no skip). This is correct but
adds unnecessary work. Phase B should either:
- Skip energy computation for sleeping bodies (if the energy accumulators
  can handle partial updates), or
- Document that energy is always fully recomputed (small cost, O(nbody)).

This is low priority — energy computation is O(nbody) with tiny
constants.

###### 16.26.9 Additional Phase B Acceptance Criteria

Add to §16.19:

**Phase A Errata:**
36. Contact between sleeping body and Static (world) body does NOT
    wake the sleeping body. Bodies can sleep while resting on the
    ground plane.
37. `dof_length` for rotational DOFs on a 0.1m arm produces tighter
    sleep threshold than on a 1.0m arm (non-uniform thresholds).
38. Deformable bodies force `AutoNever` on their tree (when
    `feature = "deformable"` is enabled).
39. Tendon-transmission actuators force `AutoNever` on spanned trees.
40. `qfrc_actuator` is zeroed for sleeping DOFs when actuated trees
    can sleep (policy relaxation).
41. User force wake (`mj_wake`) runs at the start of Phase B
    `forward()`, before any kinematics. Setting `xfrc_applied` on a
    sleeping body wakes it on the next `forward()` call.
42. `sleep_trees()` zeros all DOF-level arrays (`qvel`, `qacc`,
    `qfrc_bias`, `qfrc_passive`, `qfrc_constraint`, `qfrc_actuator`)
    and body-level arrays (`cvel`, `cacc_bias`, `cfrc_bias`) for
    sleeping trees — matching Phase A's `put_tree_to_sleep()`.
43. Two sleeping trees in different sleep cycles coupled by a limited
    tendon: both islands wake (same as equality constraint case).
44. `tree_qpos_dirty` flag array is used for qpos change detection,
    not `tree_awake` — preventing transient inconsistency between
    `tree_awake` and `tree_asleep` during the wake-detection pass.
45. Init-sleep validation uses model-time adjacency (equality
    constraints + multi-tree tendons) via union-find, not runtime
    island discovery (which isn't available at init time).

###### 16.26.10 Additional Phase B Tests

Add to §16.20:

| # | Test | Type | Validates |
|---|------|------|-----------|
| T67 | `test_static_contact_no_wake` | Unit | AC #36: ground contact doesn't wake |
| T68 | `test_dof_length_nonuniform_threshold` | Unit | AC #37: arm length affects threshold |
| T69 | `test_deformable_body_prevents_sleep` | Unit | AC #38: deformable → AutoNever |
| T70 | `test_tendon_actuator_policy` | Unit | AC #39: tendon actuator → AutoNever |
| T71 | `test_actuated_sleep_zeros_qfrc_actuator` | Unit | AC #40: qfrc_actuator zeroed |
| T72 | `test_user_force_wake_phase_b` | Integration | AC #41: xfrc_applied wakes in Phase B forward |
| T73 | `test_sleep_trees_zeros_all_arrays` | Unit | AC #42: sleep_trees zeros all arrays |
| T74 | `test_tendon_both_asleep_different_cycles_wake` | Integration | AC #43: tendon wakes both sleeping islands |
| T75 | `test_qpos_dirty_flag_isolation` | Unit | AC #44: tree_qpos_dirty separate from tree_awake |
| T76 | `test_init_sleep_validation_model_time` | Unit | AC #45: union-find validation at init time |

##### 16.27 Phase C — Awake-Index Pipeline Iteration ✅ IMPLEMENTED

###### 16.27.1 Motivation

Phase B allocates and populates three indirection arrays every step via
`mj_update_sleep_arrays()`:

- `body_awake_ind[0..nbody_awake]` — sorted indices of awake + static bodies
- `parent_awake_ind[0..nparent_awake]` — bodies whose parent is awake/static
- `dof_awake_ind[0..nv_awake]` — sorted indices of awake DOFs

These arrays are computed but **never read** by the pipeline — every
hot-path function still uses the Phase A pattern:

```rust
for body_id in 1..model.nbody {
    if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
        continue;
    }
    // ... compute ...
}
```

This has two costs:
1. **Branch mispredictions** when awake/asleep bodies are interleaved.
2. **Cache misses** from touching `body_sleep_state` for every body.

The indirection pattern eliminates both:

```rust
let (n, use_ind) = if sleep_enabled && data.nbody_awake < model.nbody {
    (data.nbody_awake, true)
} else {
    (model.nbody, false)
};
for idx in 0..n {
    let body_id = if use_ind { data.body_awake_ind[idx] } else { idx };
    // ... compute (no branch needed) ...
}
```

When all bodies are awake, `nbody_awake == nbody` and the fast path
(`use_ind = false`) runs — the indirection is a no-op with zero
overhead.

###### 16.27.2 Target Functions

Each function below currently has a body or DOF loop with a sleep
check. Phase C replaces the check with indirection iteration.

**Critical: `body_awake_ind` index 0 is always body 0 (world/static).**
`mj_update_sleep_arrays()` unconditionally places body 0 at
`body_awake_ind[0]`. Functions whose original loop starts at body 1
(skipping world) must start indirection at `idx = 1`. Functions that
include body 0 in their original loop start at `idx = 0`. The per-
function table below specifies the correct starting index for each.

**Body loops (use `body_awake_ind`):**

| Function | Loop | Current Skip | Ind. Start | Notes |
|----------|------|-------------|------------|-------|
| `mj_fwd_velocity()` | `for body_id in 1..nbody` | Yes (§16.5a) | `idx = 1` | Velocity kinematics. Original starts at 1 (world body has zero vel set separately). |
| `mj_rne()` gyroscopic | `for body_id in 1..nbody` | Yes (§16.5a) | `idx = 1` | Per-body loop iterating joints for gyroscopic terms. Uses `body_awake_ind`. |
| `mj_sensor_pos()` | per-sensor loop | Yes (§16.5d) | N/A (sensor loop) | Sensor skip uses `sensor_body_id()`, not body indirection. Unchanged. |
| `mj_sensor_vel()` | per-sensor loop | Yes (§16.5d) | N/A (sensor loop) | Same as `mj_sensor_pos()`. Unchanged. |

**Backward-pass body loops (use `parent_awake_ind` in reverse):**

`parent_awake_ind` contains bodies whose parent is awake or static.
This includes sleeping tree-root bodies (parent = body 0 = static).
However, sleeping root bodies have `cvel = 0`, `cacc_bias = 0`,
`cfrc_bias = 0` (zeroed at sleep time), so computing their force
contributions is redundant but safe — the result is zero. The backward-
pass accumulation `cfrc_bias[parent] += cfrc_bias[child]` is correct
because sleeping children contribute zero.

**Consequence:** `parent_awake_ind` is NOT suitable for skipping
sleeping bodies in backward passes. It intentionally includes sleeping
roots so that backward accumulation into body 0 works correctly. Using
it as an optimization array would require filtering out sleeping entries,
defeating the purpose. **Therefore: do NOT convert backward passes to
`parent_awake_ind` indirection in Phase C.** The existing full-body
backward iteration is correct and the cost is O(nbody) with trivial
constants (one addition per body). Selective backward-pass optimization
is deferred to §16.29 (selective CRBA) where it can be done correctly
with per-island block structure.
Tracked in [future_work_9b.md](./future_work_9b.md) §DT-43.

| Function | Loop | Current Skip | Conversion | Notes |
|----------|------|-------------|------------|-------|
| `mj_rne()` backward pass | `for body_id in (1..nbody).rev()` | No | **NOT CONVERTED** | See rationale above. O(nbody) with zero branching already. |

**DOF loops (use `dof_awake_ind`):**

| Function | Loop | Current Skip | Ind. Start | Notes |
|----------|------|-------------|------------|-------|
| `mj_fwd_passive()` joints | `JointVisitor` | Yes (§16.5a') | N/A | Uses `is_joint_sleeping()` in visitor. Unchanged — see §16.27.3a. |
| `mj_fwd_passive()` tendons | `for t in 0..ntendon` | Yes (§16.5a') | N/A | Tendon loop uses `tendon_all_dofs_sleeping()`. Not DOF-indexed. Unchanged. |
| `integrate()` velocity | `for i in 0..nv` | Yes (§16.5a'') | `idx = 0` | Per-DOF loop. Direct `dof_awake_ind` conversion. |
| `integrate()` position | `JointVisitor` | Yes (§16.5a'') | N/A | Uses `is_sleeping()` in `PositionIntegrateVisitor`. See §16.27.3a. |

**NOT converted (intentional):**

| Function | Reason |
|----------|--------|
| `mj_fwd_position()` | FK runs for ALL bodies including sleeping. Required for §16.15 qpos change detection: FK computes new `xpos`/`xquat` from `qpos` and compares against stored values to detect external modification. Also needed for collision broad-phase AABBs. |
| `mj_collision()` | Already skips sleeping-vs-sleeping pairs in narrow phase (§16.5b). Broad-phase (SAP) processes all geom AABBs regardless — this is the bottleneck and cannot benefit from body indirection. |
| `mj_crba()` | Deferred to §16.29 (requires partial LDL refactoring). |
| `mj_rne()` gravity | Per-joint loop (`for jnt_id in 0..njnt`), not per-DOF. Joints are not 1:1 with DOFs (Ball = 3, Free = 6). `dof_awake_ind` contains DOF indices and cannot be used as joint indices directly. Building a `jnt_awake_ind` array is possible but unnecessary — the existing `body_sleep_state[jnt_body[jnt_id]]` check has the same contiguous-skip characteristics as the JointVisitor pattern (§16.27.3a): joints within a tree are contiguous in memory, so sleeping trees form long predicted-skip runs with negligible branch cost. Unchanged. |
| `mj_rne()` Featherstone RNE (forward + backward + projection) | These three loops (forward bias acceleration, backward force propagation, joint-space projection) run for ALL bodies/joints. Sleeping bodies have `cvel = 0` and `cacc_bias = 0` (zeroed at sleep time), so they contribute zero to bias forces — the computation is redundant but safe and correct. The backward accumulation `cfrc_bias[parent] += cfrc_bias[child]` requires all children to be visited for correct parent accumulation. Converting to indirection would require careful handling of cross-tree root boundaries. Cost is O(nbody + njnt) with small constants. Defer to §16.29. |
| `mj_fwd_actuation()` | Per-actuator loop (not per-DOF). Actuated trees have `AutoNever` policy by default — they never sleep. Phase B relaxation (§16.18) allows explicit `sleep="allowed"` override, but this is a rare user-driven case. Adding indirection for the common no-op case is not worth the complexity. If needed, a simple `body_sleep_state` check on the actuator's target body suffices. |
| `mj_energy_pos()` | See §16.27.4 — NOT skipped. |
| `mj_energy_vel()` | See §16.27.4 — NOT skipped. |
| `mj_sensor_pos/vel()` | Already use per-sensor `sensor_body_id()` gating (§16.5d). Sensors are not body-indexed loops — they iterate `0..nsensor` with heterogeneous object types. Body indirection doesn't apply. |

###### 16.27.3 Implementation Pattern

For each target function, the conversion follows this pattern:

**Step 1:** Add the indirection preamble at the top of the function:

```rust
let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
let use_body_ind = sleep_enabled && data.nbody_awake < model.nbody;
let use_dof_ind = sleep_enabled && data.nv_awake < model.nv;
```

**Step 2:** Replace the loop header and remove the branch. The starting
index depends on the original loop's range — see the per-function table
in §16.27.2 for the correct value.

```rust
// Before (original loop starts at body 1):
for body_id in 1..model.nbody {
    if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
        continue;
    }
    do_work(body_id);
}

// After (indirection starts at idx 1 to skip body 0):
let nbody = if use_body_ind { data.nbody_awake } else { model.nbody };
for idx in 1..nbody {
    let body_id = if use_body_ind { data.body_awake_ind[idx] } else { idx };
    do_work(body_id);
}
```

For DOF loops that start at 0:

```rust
// Before:
for dof in 0..model.nv {
    if sleep_enabled && !data_tree_awake_for_dof(model, data, dof) {
        continue;
    }
    do_work(dof);
}

// After:
let nv = if use_dof_ind { data.nv_awake } else { model.nv };
for idx in 0..nv {
    let dof = if use_dof_ind { data.dof_awake_ind[idx] } else { idx };
    do_work(dof);
}
```

**Step 3 — backward passes:** NOT converted. See §16.27.2 rationale.

###### 16.27.3a JointVisitor Conversion

`mj_fwd_passive()` and `integrate()` position integration use the
`JointVisitor` pattern via `model.visit_joints()`. These visitors
already carry sleep-check data (`jnt_body`, `body_sleep_state`) and
check `is_sleeping()` / `is_joint_sleeping()` at the top of each
`visit_*` method.

Converting these to `dof_awake_ind` indirection would require either:
(a) replacing `visit_joints()` with a manual `dof_awake_ind` loop that
    dispatches by joint type — duplicating the visitor dispatch logic, or
(b) adding an `awake_joints` indirection array (not currently computed).

Both options add complexity for modest gain. The `JointVisitor` pattern
is already efficient: the `is_sleeping()` check is a single indexed
load + compare (body_sleep_state[jnt_body[jnt_id]]) with no hash or
search. The branch misprediction cost is low because joints within a
tree are contiguous — the pattern is long runs of "skip" or "compute",
not interleaved.

**Decision:** Keep the existing `JointVisitor` sleep-check pattern for
`mj_fwd_passive()` and `integrate()` position integration. Only the
explicit per-DOF velocity integration loop in `integrate()` is converted
to `dof_awake_ind`.

###### 16.27.4 Energy Functions: NOT Skipped

`mj_energy_pos()` and `mj_energy_vel()` are **NOT** converted to
indirection and **NOT** given sleep skip logic. Rationale:

- **`mj_energy_pos()`:** Gravitational PE (`-m * g · com`) must include
  ALL bodies for a physically meaningful total. If sleeping bodies were
  skipped, `energy_potential` would drop discontinuously when bodies go
  to sleep and jump when they wake — breaking any user code that tracks
  energy conservation, computes energy deltas, or uses energy for
  termination conditions. MuJoCo's `mj_energy` computes over all bodies
  regardless of sleep state. We match that behavior.

  The spring PE loop iterates per-joint (not per-body) and is already
  O(njnt) with trivial constants. No skip needed.

- **`mj_energy_vel()`:** Sleeping bodies have `cvel = 0` by invariant,
  so their KE contribution is exactly zero already. The computation is
  redundant (multiplying zero by inertia to get zero) but costs nothing
  measurable — `mj_energy_vel` is a dot-product over `nv` or a simple
  per-body sum. Skipping would save O(nbody_sleeping) multiplications
  by zero, which is negligible.

  The mass-matrix path (`0.5 * qvel^T * M * qvel`) operates on the
  full `nv`-dimensional `qvel` vector. Sleeping DOFs have `qvel = 0`,
  so they contribute zero to the dot product automatically. No
  modification needed.

**Result:** Energy is always physically consistent and matches MuJoCo
semantics. No `energy_potential` documentation caveats needed.

###### 16.27.5 Acceptance Criteria

46. With `ENABLE_SLEEP` cleared, all pipeline functions produce
    bit-identical results to pre-Phase-C behavior.
47. With all bodies awake and sleep enabled, pipeline output is
    bit-identical to `ENABLE_SLEEP` cleared (the indirection is a
    no-op).
48. With sleeping bodies, `mj_fwd_velocity()` via indirection produces
    identical `cvel` for awake bodies as the Phase B branch-skip.
49. `mj_energy_pos()` and `mj_energy_vel()` produce identical results
    whether sleep is enabled or disabled (energy always includes all
    bodies). Sleeping bodies contribute zero KE (by qvel=0 invariant)
    and frozen PE (by xpos invariant).
50. `integrate()` velocity loop via `dof_awake_ind` produces identical
    qpos/qvel for awake DOFs as the Phase B branch-skip.
51. `mj_rne()` gyroscopic loop via `body_awake_ind` indirection produces
    identical `qfrc_bias` for awake DOFs as the Phase B branch-skip.
    `mj_rne()` gravity loop retains the existing per-joint branch-skip
    (not converted to indirection — see §16.27.2) and remains correct.
52. `mj_fwd_passive()` via `JointVisitor` sleep check produces
    identical `qfrc_passive` as Phase B (no change — visitor pattern
    retained).
53. `mj_rne()` Featherstone forward/backward/projection loops produce
    correct `qfrc_bias` with all-zero contributions from sleeping
    bodies (cvel=0, cacc_bias=0 invariant). NOT converted to
    indirection — full-body iteration retained.
54. Per-function bit-identity: for EACH converted function, running
    with indirection ON vs Phase B branch-skip OFF produces identical
    output arrays for all awake bodies/DOFs. This is tested per
    function, not just end-to-end, to isolate regressions.

###### 16.27.6 Tests

| # | Test | Type | Validates |
|---|------|------|-----------|
| T77 | `test_indirection_velocity_equivalence` | Unit | AC #48: `mj_fwd_velocity` indirection matches branch-skip |
| T78 | `test_indirection_vel_integration_equivalence` | Unit | AC #50: `integrate()` velocity loop via `dof_awake_ind` matches full |
| T79 | `test_energy_all_bodies_always` | Unit | AC #49: `mj_energy_pos/vel` include all bodies, sleeping KE = 0 |
| T80 | `test_all_awake_bit_identical` | Regression | AC #47: no-op when all awake |
| T81 | `test_indirection_rne_gyroscopic_equivalence` | Unit | AC #51: `mj_rne` gyroscopic indirection + gravity branch-skip match Phase B |
| T82 | `test_rne_featherstone_sleeping_zero_contribution` | Unit | AC #53: Featherstone loops produce zero from sleeping bodies |
| T83 | `test_per_function_bit_identity` | Regression | AC #54: each converted function individually bit-identical |
| T84 | `test_energy_continuous_across_sleep_transition` | Integration | AC #49: energy_potential does not jump when a body sleeps/wakes |

###### 16.27.7 Phase C Scope Summary

Phase C §16.27 converts exactly **three** loop sites to indirection:

1. `mj_fwd_velocity()` body loop → `body_awake_ind`, start idx 1
2. `mj_rne()` gyroscopic body loop → `body_awake_ind`, start idx 1
3. `integrate()` velocity DOF loop → `dof_awake_ind`, start idx 0

Everything else is either:
- **Intentionally not converted** (FK, collision, CRBA, Featherstone
  RNE, RNE gravity, energy, actuation) with documented rationale, or
- **Already efficient** (JointVisitor sleep checks, RNE gravity
  per-joint branch-skip, sensor per-object gating, tendon
  all-DOFs-sleeping check) with no indirection needed.

This minimal scope reduces implementation risk while capturing the
primary performance benefit: the velocity kinematics and gyroscopic
bias force loops are the hottest per-body paths after FK (which must
run for all bodies anyway). Integration is the hottest per-DOF path.
The RNE gravity loop is per-joint (not per-DOF) and retains its
existing branch-skip — see §16.27.2 "NOT converted" table.

**Implementation order:**
1. Convert `mj_fwd_velocity()` (highest impact, simplest conversion)
2. Convert `mj_rne()` gyroscopic (one body loop, same function)
3. Convert `integrate()` velocity loop (per-DOF, straightforward)
4. Run full regression suite (all Phase A + B tests)
5. Add Phase C tests T77–T84

##### 16.28 Phase C — Island-Local Delassus Assembly ✅ IMPLEMENTED

###### 16.28.1 Motivation

`mj_fwd_constraint_islands()` partitions contacts by island and solves
each island independently. But the solver call chain still operates on
full `nv`-dimensional data:

1. `compute_contact_jacobian()` builds `dim × nv` Jacobians (full width)
2. `assemble_contact_system()` computes `M^{-1} * J^T` via
   `mj_solve_sparse()` on the full LDL factorization
3. The Delassus matrix `A = J * M^{-1} * J^T` is `nefc × nefc` but
   has block-diagonal structure that isn't exploited

For a scene with 100 contacts across 10 islands of 10 contacts each:
- Current: 10 calls to `assemble_contact_system()`, each solving `nv`
  systems and building a 30×30 Delassus matrix from `nv`-wide Jacobians
- Island-local: 10 calls building `nv_island × nv_island` mass matrices
  and `dim × nv_island` Jacobians, assembling 30×30 Delassus matrices
  from island-local data

The `M^{-1} * J^T` computation dominates: each column requires a full
LDL solve (O(nv) per column). With island-local factorization, each
column requires O(nv_island) — potentially orders of magnitude smaller.

###### 16.28.2 Algorithm

For each island `k` with `nv_k` DOFs and `nc_k` contacts:

**Step 1: Extract island-local mass matrix.**

Extract the principal submatrix of `qM` for this island's DOFs. `qM` is
a dense `DMatrix<f64>` (computed by `mj_crba`). The codebase does not
currently have a sparse mass matrix (`qM_sparse`) — sparse CSC support
exists only for the Newton solver's Hessian (`SparseHessian`, used when
`nv > NV_SPARSE_THRESHOLD = 60`). The extraction below uses dense
indexing, which is O(nv_k²) per island — acceptable for typical island
sizes. If a sparse `qM` representation is added in the future, the
extraction should iterate nonzero entries via CSC accessors instead.
Tracked in [future_work_9b.md](./future_work_9b.md) §DT-44.

```rust
// M_island is the principal submatrix of qM for island k's DOFs.
// island DOFs come from map_idof2dof[island_idofadr[k]..+island_nv[k]].
let nv_k = data.island_nv[k];
let dof_start = data.island_idofadr[k];
let mut M_island = DMatrix::zeros(nv_k, nv_k);

// Dense path (qM is DMatrix<f64>):
for i in 0..nv_k {
    let gi = data.map_idof2dof[dof_start + i];
    for j in 0..nv_k {
        let gj = data.map_idof2dof[dof_start + j];
        M_island[(i, j)] = data.qM[(gi, gj)];
    }
}

```

**Step 2: Factor island-local mass matrix.**

Use nalgebra's dense Cholesky: `M_island.cholesky()`. This is correct
because `qM` is symmetric positive-definite and any principal submatrix
of an SPD matrix is SPD. Dense Cholesky on a small `nv_k × nv_k`
matrix is faster than the sparse tree-structured LDL solve on the
full `nv × nv` system when `nv_k << nv`.

```rust
let chol = M_island.cholesky()
    .expect("island mass matrix must be SPD");
```

**Step 3: Build island-local Jacobians.**

Remap each contact's Jacobian columns from global DOF indices to
island-local indices. Use the forward mapping `map_idof2dof` (island-
local → global) to extract only the relevant columns in O(nv_island)
instead of scanning all `nv` global columns:

```rust
fn remap_jacobian_to_island(
    j_global: &DMatrix<f64>,
    map_idof2dof: &[usize],   // island-local DOF → global DOF
    dof_start: usize,          // offset into map_idof2dof for this island
    nv_island: usize,
) -> DMatrix<f64> {
    let dim = j_global.nrows();
    let mut j_local = DMatrix::zeros(dim, nv_island);
    for local_col in 0..nv_island {
        let global_col = map_idof2dof[dof_start + local_col];
        for row in 0..dim {
            j_local[(row, local_col)] = j_global[(row, global_col)];
        }
    }
    j_local
}
```

Complexity: O(dim × nv_island) per contact, not O(dim × nv). For an
island with 10 DOFs in a 300-DOF model, this is a 30× reduction per
contact Jacobian remap.

**Step 4: Assemble island-local Delassus matrix.**

Replace `mj_solve_sparse()` calls with `chol.solve()` on the island-
local system:

```rust
// M_island^{-1} * J_local^T  (nv_k × dim per contact)
let mut minv_jt = DMatrix::zeros(nv_k, dim_k);
for col in 0..dim_k {
    let jt_col = j_local.column(col).clone_owned();
    let solved = chol.solve(&jt_col);
    minv_jt.set_column(col, &solved);
}

// A_block = J_local * minv_jt  (dim_k × dim_k)
let a_block = j_local * &minv_jt;
```

**Step 5: Extract island-local qacc_smooth.**

The global `qacc_smooth` is precomputed once before the island loop
(see §16.28.4). Each island extracts its slice by DOF mapping — no
per-island solve needed:

```rust
// qacc_smooth was precomputed globally in §16.28.4.
// Extract this island's slice:
let mut qacc_smooth_island = DVector::zeros(nv_k);
for i in 0..nv_k {
    let gi = data.map_idof2dof[dof_start + i];
    qacc_smooth_island[i] = qacc_smooth[gi];
}
// No chol.solve() here — qacc_smooth is already M^{-1} * qfrc_smooth.
```

The RHS computation (`b = J * qacc_smooth + aref`) then uses
`qacc_smooth_island` and `j_local` instead of the global versions.

###### 16.28.3 Refactoring Strategy

The refactoring introduces a new function `assemble_contact_system_island()`
that accepts island-local inputs. The existing `assemble_contact_system()`
is preserved for the global fallback path (Newton solver, `nisland == 0`).

```rust
fn assemble_contact_system_island(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians_local: &[DMatrix<f64>],  // dim × nv_island each
    chol: &Cholesky<f64, Dyn>,         // island-local M factorization
    qacc_smooth_island: &DVector<f64>, // precomputed slice from global qacc_smooth (§16.28.4)
    nv_island: usize,
) -> (DMatrix<f64>, DVector<f64>, Vec<usize>)
```

The core assembly logic (Baumgarte stabilization, impedance, CFM,
friction parameters) is identical — only the matrix dimensions and
solve calls change.

**`mj_fwd_constraint_islands()` changes:**

```rust
// Before each island's solve loop:
// 1. Extract M_island, factor it (once per island)
// 2. Compute qacc_smooth_island (once per island)
// 3. Remap each contact Jacobian to island-local
// 4. Call assemble_contact_system_island() instead of assemble_contact_system()
// 5. Call pgs_solve_with_system() / cg equivalent (unchanged — works on any size)
// 6. Apply forces (unchanged)
```

The PGS/CG solvers (`pgs_solve_with_system`, `cg_solve_contacts`) are
dimension-agnostic — they operate on the `(A, b)` matrices regardless
of whether those came from global or island-local assembly. No changes
needed to the solver cores.

###### 16.28.4 Global qacc_smooth Precomputation

The unconstrained acceleration `qacc_smooth = M^{-1} * qfrc_smooth` is
computed once globally (full `nv` LDL solve), then island-local slices
are extracted by DOF index. This avoids redundant per-island force
accumulation.

**Important:** `qacc_smooth` does NOT include `qfrc_constraint` (penalty
forces from joint limits, tendon limits, and equality constraints). The
existing `assemble_contact_system()` computes `qacc_smooth` from
`qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias` — contact
forces are solved additively on top of this unconstrained acceleration.
The island-local version must match this formula exactly:

```rust
// Compute once at the top of mj_fwd_constraint_islands(),
// AFTER penalty constraints have been applied to qfrc_constraint
// but BEFORE the contact solve loop:
let mut qacc_smooth = data.qfrc_applied.clone();
qacc_smooth += &data.qfrc_actuator;
qacc_smooth += &data.qfrc_passive;
qacc_smooth -= &data.qfrc_bias;
// NOTE: qfrc_constraint is NOT included. Contact forces are solved
// relative to the unconstrained acceleration, matching the existing
// assemble_contact_system() at line 12432. Penalty forces (limits,
// equality) are applied separately via qfrc_constraint and do not
// enter the Delassus assembly.
let (rowadr, rownnz, colind) = model.qld_csr();
mj_solve_sparse(&data.qLD_diag, rowadr, rownnz, colind, &data.qLD_data, &mut qacc_smooth);

// Per island: extract slice
for i in 0..nv_k {
    qacc_smooth_island[i] = qacc_smooth[data.map_idof2dof[dof_start + i]];
}
```

This global precomputation replaces the per-island `qacc_smooth`
computation that `assemble_contact_system()` currently does for each
solver call, eliminating redundant LDL solves.

###### 16.28.5 When to Use Island-Local vs Global

| Condition | Path |
|-----------|------|
| `nisland == 0` | Global `mj_fwd_constraint()` |
| Newton solver | Global `mj_fwd_constraint()` |
| `nv_island == nv` (single island spans all DOFs) | Global assembly (no benefit from extraction) |
| `nv_island < nv` | Island-local assembly |

The single-island-spans-all case is detected by `data.nisland == 1 &&
data.island_nv[0] == model.nv`. In this case, the Cholesky extraction
would duplicate the full mass matrix — use the global LDL path instead.

**Note on Newton solver:** Phase B §16.16.2 specified per-island Newton
dispatch (`newton_solve_island`). The implementation chose to keep
Newton on the global path because the Newton solver's Hessian assembly
and line search operate on the unified constraint system — decomposing
into per-island sub-problems requires refactoring the Hessian
rank-1 update/downdate and the line search to operate on block-diagonal
structure. This is deferred. PGS and CG solvers are dimension-agnostic
and work on any `(A, b)` system size, so they benefit from island-local
assembly immediately.
Tracked in [future_work_9b.md](./future_work_9b.md) §DT-42.

###### 16.28.6 Acceptance Criteria

55. Island-local Delassus assembly produces the same `A` and `b`
    matrices as global assembly (to machine epsilon) for each island's
    contacts.
56. `DISABLE_ISLAND` flag produces bit-identical results to pre-Phase-C.
57. For two independent free bodies on a plane, per-island solve
    produces identical contact forces as global solve.
58. For a single island spanning all DOFs, the global fallback path
    activates.

###### 16.28.7 Tests

| # | Test | Type | Validates |
|---|------|------|-----------|
| T85 | `test_island_delassus_equivalence` | Unit | AC #55: island-local A,b match global |
| T86 | `test_island_solve_forces_match_global` | Integration | AC #57: forces identical for decoupled bodies |
| T87 | `test_single_island_uses_global_path` | Unit | AC #58: fallback when one island spans all DOFs |
| T88 | `test_disable_island_phase_c_bit_identical` | Regression | AC #56: DISABLE_ISLAND unchanged |

##### 16.29 Phase C — Selective CRBA & Partial LDL

###### 16.29.1 Motivation

`mj_crba()` is called every step and computes the full `nv × nv` mass
matrix plus its sparse LDL factorization. For sleeping bodies, the mass
matrix entries are invariant (body poses frozen, joint axes frozen,
`cinert` unchanged). Selective CRBA skips the computation for sleeping
DOFs, preserving their stale-but-correct `qM` and `qLD` entries from
their last awake step.

MuJoCo (3.x) implements this optimization via `nbody_awake` /
`body_awake_ind` filtering in `mj_crb()` and `dof_awake_ind` filtering
in `mj_factorM()`. Our implementation follows the same approach,
adapted to our data structures.

**Cost model.** CRBA has three cost centers:
1. **Body loops (Phases 1-2):** O(nbody) — dominated by 6×6 matrix
   copies and additions. For a humanoid (30 bodies), ~60 μs.
2. **Mass matrix build (Phase 3):** O(nv × max_depth) — dominated by
   6-dim dot products and ancestor walks via `dof_parent`. For deep chains
   this is the bottleneck: a 10-link chain with hinge joints does 10×10/2 =
   50 6-dim dot products.
3. **LDL factorization (Phase 5):** O(Σ depth(i)²) — dominated by the
   ancestor-pair elimination loop. For balanced trees this is O(n log²n),
   for typical humanoids (depth ≤ 8) effectively O(n). Small constants.

Phases 1-3 dominate for deep kinematic chains. Phase 5 dominates for
wide, shallow models (many DOFs, low depth). Both are worth optimizing,
but Phase 5 carries higher risk (correctness of partial elimination).
Hence the two-sub-step split: C3a (body loops, low risk) and C3b
(partial LDL, medium risk, deferred until profiling).

###### 16.29.2 Correctness Argument

**Why stale `qM` entries are safe:**
- Sleeping DOFs have `qvel = 0`, `qacc = 0`, `qfrc_* = 0` by invariant.
- The constraint solver never reads sleeping DOFs' mass matrix entries
  (island-local assembly from §16.28 only extracts awake DOF blocks).
- `mj_solve_sparse()` for `qacc_smooth` reads all DOFs, but sleeping
  DOFs have zero force, so `qacc_smooth[sleeping_dof] = 0` regardless
  of `qM` values. (Proof: `qacc_smooth = M^{-1} * f_total`. For sleeping
  DOFs, `f_total[dof] = 0` and `M` is block-diagonal across trees, so
  the sleeping DOF slice of `qacc_smooth` is zero.)
- When a tree wakes, its first `forward()` pass runs full CRBA (because
  `nbody_awake` now includes the waking bodies), recomputing all `qM`
  entries before the solver reads them.

**Why stale `qLD` entries are safe (C3a with full LDL):**
- C3a runs full `mj_factor_sparse()` unconditionally. The full LDL
  reads all `qM` entries, including stale sleeping ones. But since
  sleeping DOFs' `qM` values are unchanged from their last awake step,
  the full factorization produces correct `qLD` entries for all DOFs.
  There is no correctness risk in C3a — only a missed optimization
  opportunity (which C3b addresses).

**Why stale `qLD` entries are safe (C3b with partial LDL):**
- Sleeping is per-tree. DOF parent chains (`dof_parent`) never cross
  tree boundaries. Therefore, the LDL elimination for tree A's DOFs
  never touches tree B's `qLD` entries. If tree B is sleeping, its
  `qLD` entries are untouched by the partial elimination and remain
  valid from the last awake step.
- `mj_solve_sparse()` reads sleeping DOFs' `qLD_data` and `qLD_diag` in
  its forward/backward substitution. For sleeping DOFs, the RHS is zero,
  so the output for those DOFs is zero regardless of the `qLD` values.
  The critical property: the solve does NOT propagate sleeping DOF
  values into awake DOF results, because the tree-sparse structure
  means sleeping and awake DOFs are in disconnected blocks of L.

**When stale entries cause problems:**
- If external code reads `qM[sleeping_dof, ...]` directly (outside the
  pipeline). This is acceptable — document that `qM` entries for
  sleeping DOFs are stale but self-consistent.

**Key invariant exploited throughout:** Within a single kinematic tree,
all bodies share the same sleep state (sleeping is per-tree, not
per-body). Cross-tree DOF interactions are zero in both `qM` and `qLD`.

###### 16.29.3 Step C3a — Selective CRBA Body Loops

**Scope:** Modify `mj_crba(model, data)` to skip Phases 1-4 and
Phase 6 for sleeping bodies when `sleep_filter = true`. Phase 5 (LDL)
remains unchanged (full computation). When `sleep_filter = false`,
behavior is bit-identical to the original. The function signature and
all postconditions are identical.

**Precondition:** C1 complete (awake-index arrays populated).

**Phase 0 (preamble + selective qM zeroing):**

The existing `mj_crba()` starts with `data.qM.fill(0.0)` (line 9827).
With selective CRBA, we must NOT zero sleeping DOFs' `qM` entries —
they must be preserved from the last awake step.

**MuJoCo comparison note:** MuJoCo's `mj_crb()` zeros M *after* the
backward pass (between Phase 2 and Phase 3), using sparse row zeroing
via `mju_zeroSparse(M, rownnz, rowadr, dof_awake_ind, nv)`. We zero
*before* Phase 1 using per-tree diagonal block zeroing. Both are
correct because Phase 3 writes all awake entries unconditionally. Our
ordering (zero first) is simpler and matches the existing code's
structure. Our per-tree block zeroing is O(Σ dof_count_awake²) vs
MuJoCo's O(Σ row_nnz_awake), but since we use a dense `DMatrix` (not
sparse CSR), the per-tree block approach is natural and avoids needing
sparse row metadata.

Zero only awake DOFs' rows and columns. Since `qM` is symmetric and
tree-sparse, only entries `qM[(i, j)]` where both `i` and `j` belong
to awake trees need zeroing. Use the per-tree approach:

```rust
let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
let sleep_filter = sleep_enabled && data.nbody_awake < model.nbody;

data.qLD_valid = false;

if !sleep_filter {
    // Fast path: all bodies awake, zero everything (original behavior)
    data.qM.fill(0.0);
} else {
    // Selective path: zero only awake trees' DOF blocks.
    // DOFs within a tree are contiguous (§16.0 body ordering guarantee).
    for tree_id in 0..model.ntree {
        if !data.tree_awake[tree_id] {
            continue; // Sleeping tree — preserve qM entries
        }
        let dof_start = model.tree_dof_adr[tree_id];
        let dof_count = model.tree_dof_num[tree_id];
        // Zero the tree's diagonal block and its cross-entries with
        // other awake trees. Since qM is tree-block-diagonal (cross-tree
        // entries are always zero), we only need to zero the
        // [dof_start..dof_start+dof_count] × [0..nv] band.
        // Optimization: since cross-tree entries are zero, just zero
        // the diagonal block [dof_start..+dof_count]².
        for i in dof_start..(dof_start + dof_count) {
            for j in dof_start..(dof_start + dof_count) {
                data.qM[(i, j)] = 0.0;
            }
        }
    }
}

if model.nv == 0 {
    return;
}
```

**Why tree-block-diagonal zeroing is sufficient:** The mass matrix `qM`
has nonzero entries only for DOF pairs (i, j) where DOFs i and j
belong to the same kinematic tree. This is because CRBA's off-diagonal
computation (Phase 3) walks `dof_parent` chains, which never cross
tree boundaries. Cross-tree entries `qM[(i, j)]` where
`dof_treeid[i] != dof_treeid[j]` are always zero. Therefore, zeroing
only the intra-tree diagonal block for each awake tree is sufficient.

**Phase 1 (init crb):** Only initialize `crb_inertia` for awake bodies.

```rust
if !sleep_filter {
    for body_id in 0..model.nbody {
        data.crb_inertia[body_id] = data.cinert[body_id];
    }
} else {
    for idx in 0..data.nbody_awake {
        let body_id = data.body_awake_ind[idx];
        data.crb_inertia[body_id] = data.cinert[body_id];
    }
}
```

Sleeping bodies' `crb_inertia` values are preserved from their last
awake step. This is safe because `crb_inertia` is overwritten (not
accumulated) from `cinert` in Phase 1 — there is no stale-accumulation
hazard.

**Phase 2 (backward accumulation):** Only accumulate for awake bodies.

```rust
if !sleep_filter {
    // Full loop: shift child inertia to parent origin before accumulating.
    // shift_spatial_inertia() applies the parallel axis theorem to re-express
    // the child's 6×6 spatial inertia (about xpos[child]) at xpos[parent].
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            let d = data.xpos[body_id] - data.xpos[parent_id];
            let child_shifted = shift_spatial_inertia(&data.crb_inertia[body_id], &d);
            data.crb_inertia[parent_id] += child_shifted;
        }
    }
} else {
    // Selective: iterate only awake bodies in reverse.
    // Same shift_spatial_inertia, but using body_awake_ind.
    for idx in (1..data.nbody_awake).rev() {
        let body_id = data.body_awake_ind[idx];
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            let d = data.xpos[body_id] - data.xpos[parent_id];
            let child_shifted = shift_spatial_inertia(&data.crb_inertia[body_id], &d);
            data.crb_inertia[parent_id] += child_shifted;
        }
    }
}
```

**Correctness of selective backward pass:**

**§16.27 exception note:** §16.27.2 prohibits converting backward-pass
body loops to `parent_awake_ind` indirection because RNE's backward
pass accumulates `cfrc_bias` (a dynamic, velocity-dependent quantity)
and requires visiting sleeping tree roots for correct accumulation
into body 0. CRBA's backward pass is a **specific exception** to that
rule: it accumulates `crb_inertia` (a position-only quantity that is
frozen for sleeping bodies), and body 0's `crb_inertia` is never
consumed by Phase 3 (body 0 has no DOFs). Therefore, `body_awake_ind`
— not `parent_awake_ind` — is the correct index array here.

Unlike the RNE backward pass (excluded from indirection in §16.27.2
because `cfrc_ext` depends on dynamic quantities), CRBA backward pass
is safe to run only over awake bodies:

- **Per-tree invariant:** Within a single kinematic tree, all bodies
  share the same sleep state. If body B is awake, every body in B's
  tree is awake. The backward accumulation only involves bodies in the
  same tree, all of which are in `body_awake_ind`.
- **Sleeping trees:** If body B is sleeping, its entire tree is sleeping.
  No body in that tree appears in `body_awake_ind`, so no stale values
  are read or written.
- **Body 0 (world):** Does not participate in accumulation — the guard
  `if parent_id != 0` prevents any body from accumulating into body 0,
  and body 0 has no DOFs so its `crb_inertia` is never read by Phase 3.

**Why the §16.29.3 "subtlety" about sleeping children of body 0 is a
non-issue for C3a:** The original spec identified a concern: sleeping
tree roots' `crb_inertia` should be accumulated into body 0. However,
body 0 has no joints and no DOFs. Phase 3 only reads
`crb_inertia[body_i]` where `body_i = jnt_body[jnt_i]` — never body 0.
Therefore, body 0's `crb_inertia` value is never consumed by the mass
matrix build. The sleeping-root-to-body-0 accumulation is unnecessary
and is **omitted** in C3a. (MuJoCo's `mj_crb` similarly does not
special-case body 0 contributions from sleeping roots.)

**Phase 3 (mass matrix build):** Per-DOF iteration with `dof_parent` walk.

The loop iterates per-DOF (not per-joint) and walks `dof_parent` for
off-diagonal entries. This correctly handles same-body cross-entries for
bodies with multiple joints (e.g., two hinges on one body), because
`dof_parent` chains same-body DOFs together. With C3a, use
`dof_awake_ind` for the outer loop:

```rust
// Pre-compute per-DOF motion subspace columns (cdof).
// Build per-joint subspaces for ALL joints (cheap O(njnt) matrix fill),
// then extract individual DOF columns. Sleeping joints' subspaces are
// computed but never read because the outer loop skips sleeping DOFs,
// and the dof_parent walk only visits ancestors in the same awake tree.
let joint_subspaces: Vec<_> = (0..model.njnt)
    .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
    .collect();

let cdof: Vec<Vector6<f64>> = (0..model.nv)
    .map(|dof| {
        let jnt = model.dof_jnt[dof];
        let dof_in_jnt = dof - model.jnt_dof_adr[jnt];
        joint_subspaces[jnt].column(dof_in_jnt).into_owned()
    })
    .collect();

let nv_iter = if sleep_filter { data.nv_awake } else { model.nv };
for v in 0..nv_iter {
    let dof_i = if sleep_filter { data.dof_awake_ind[v] } else { v };
    let body_i = model.dof_body[dof_i];
    let ic = &data.crb_inertia[body_i];

    // buf = Ic[body_i] * cdof[dof_i]  (spatial force at body_i's frame)
    let mut buf: Vector6<f64> = ic * &cdof[dof_i];

    // Diagonal: M[i,i] = cdof[i]^T * buf
    data.qM[(dof_i, dof_i)] = cdof[dof_i].dot(&buf);

    // Walk dof_parent chain for off-diagonal entries.
    // All ancestors are in the same awake tree (per-tree invariant),
    // so no sleep check is needed in the ancestor walk.
    let mut current_body = body_i;
    let mut j = model.dof_parent[dof_i];

    while let Some(dof_j) = j {
        let body_j = model.dof_body[dof_j];

        // Spatial force transform only when crossing a body boundary.
        // Same-body DOFs share an origin, so no transform is needed.
        if body_j != current_body {
            let r = data.xpos[current_body] - data.xpos[body_j];
            let fl_x = buf[3];
            let fl_y = buf[4];
            let fl_z = buf[5];
            buf[0] += r.y * fl_z - r.z * fl_y;
            buf[1] += r.z * fl_x - r.x * fl_z;
            buf[2] += r.x * fl_y - r.y * fl_x;
            current_body = body_j;
        }

        // Off-diagonal: M[j,i] = cdof[j]^T * buf
        let m_ji = cdof[dof_j].dot(&buf);
        data.qM[(dof_j, dof_i)] = m_ji;
        data.qM[(dof_i, dof_j)] = m_ji; // symmetry

        j = model.dof_parent[dof_j];
    }
}
```

**Safety of `dof_parent` walk:** The walk starts from an awake DOF and
follows `dof_parent` links. Same-body DOFs share a body (and thus share
sleep state). Cross-body links follow kinematic ancestry within the same
tree. Since the tree is fully awake (per-tree invariant), every DOF
encountered is awake and its `cdof`, `crb_inertia`, and `xpos` are fresh.
The walk never crosses into a sleeping tree.

**Phase 4 (armature):** Only awake joints. The existing code iterates
per-joint (not per-DOF) and adds both `jnt_armature` and `dof_armature`.
Preserve this structure with a sleep check:

```rust
for jnt_id in 0..model.njnt {
    let body_i = model.jnt_body[jnt_id];
    if sleep_filter
        && data.body_sleep_state[body_i] == SleepState::Asleep
    {
        continue;
    }
    let dof_adr = model.jnt_dof_adr[jnt_id];
    let nv = model.jnt_type[jnt_id].nv();
    let armature = model.jnt_armature[jnt_id];
    for i in 0..nv {
        data.qM[(dof_adr + i, dof_adr + i)] += armature;
        if let Some(&dof_arm) = model.dof_armature.get(dof_adr + i) {
            data.qM[(dof_adr + i, dof_adr + i)] += dof_arm;
        }
    }
}
```

**Phase 5 (sparse LDL):** **Unchanged in C3a.** Run full
`mj_factor_sparse(model, data)`. This reads all `qM` entries (including
stale sleeping ones) and produces correct `qLD` for all DOFs. The
sleeping entries in `qM` are unchanged from their last awake step, so
the factorization over them is redundant but correct.

**Phase 6 (cache body effective mass):** Selective. Only update
`body_min_mass` and `body_min_inertia` for awake bodies. Sleeping
bodies' cached values are preserved from their last awake step (frozen
`qM` diagonal → frozen cached mass).

The existing `cache_body_effective_mass()` has three phases:
1. **Reset** (line 10076): `body_min_mass[1..nbody] = f64::INFINITY`
2. **Visitor**: Per-joint `min()` update from `qM` diagonal
3. **Fallback** (line 10086): Replace `INFINITY` → `DEFAULT_MASS_FALLBACK`
   for bodies with no DOFs of that type

With selective CRBA, ALL THREE phases must skip sleeping bodies:

```rust
fn cache_body_effective_mass(model: &Model, data: &mut Data, sleep_filter: bool) {
    // sleep_filter is passed in from mj_crba's preamble rather than
    // recomputed, ensuring the same gate governs all phases.

    // Phase 1: Reset only awake bodies.
    // CRITICAL: sleeping bodies' cached values must be preserved.
    for i in 1..model.nbody {
        if sleep_filter
            && data.body_sleep_state[i] == SleepState::Asleep
        {
            continue; // Preserve cached values from last awake step
        }
        data.body_min_mass[i] = f64::INFINITY;
        data.body_min_inertia[i] = f64::INFINITY;
    }

    // Phase 2: Visitor with sleep guard.
    struct MassCacheVisitor<'a> {
        model: &'a Model,
        data: &'a mut Data,
        sleep_filter: bool,
    }

    impl JointVisitor for MassCacheVisitor<'_> {
        fn visit_free(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            if self.sleep_filter
                && self.data.body_sleep_state[body_id] == SleepState::Asleep
            {
                return;
            }
            // ... unchanged mass/inertia extraction from qM diagonal ...
        }
        // visit_ball, visit_hinge, visit_slide: same guard at top
    }

    let mut visitor = MassCacheVisitor {
        model, data, sleep_filter,
    };
    model.visit_joints(&mut visitor);

    // Phase 3: Fallback — only awake bodies.
    for i in 1..model.nbody {
        if sleep_filter
            && data.body_sleep_state[i] == SleepState::Asleep
        {
            continue;
        }
        if data.body_min_mass[i] == f64::INFINITY {
            data.body_min_mass[i] = DEFAULT_MASS_FALLBACK;
        }
        if data.body_min_inertia[i] == f64::INFINITY {
            data.body_min_inertia[i] = DEFAULT_MASS_FALLBACK;
        }
    }
}
```

This modifies the existing function in-place (no new `_selective`
variant needed). When `sleep_filter = false`, all guards are skipped
— zero overhead on the all-awake path.

###### 16.29.4 Step C3a — Implementation Checklist

1. Add `sleep_filter` preamble (Phase 0).
2. Replace `qM.fill(0.0)` with per-tree selective zeroing.
3. Gate Phase 1 `crb_inertia` init on `body_awake_ind`.
4. Gate Phase 2 backward pass on `body_awake_ind` (reverse iteration).
5. Gate Phase 3 DOF loop on `dof_awake_ind` indirection (outer loop
   iterates `0..nv_awake`, indexing via `dof_awake_ind[v]`).
   `joint_subspaces` and `cdof` computed for ALL DOFs unconditionally
   (cheap O(njnt) + O(nv) fill); only the outer loop is gated.
6. Gate Phase 4 armature loop on `body_sleep_state` check.
7. Phase 5 unchanged (full `mj_factor_sparse`).
8. Gate Phase 6 `cache_body_effective_mass` via `sleep_filter` flag in
   visitor struct.
9. Verify `sleep_filter = false` path is bit-identical to original.

###### 16.29.4a Step C3a — Risk Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Phase 0: sleeping tree's `qM` entries zeroed by mistake | Medium | High (corrupt mass matrix, wrong dynamics) | Per-tree zeroing uses `tree_dof_adr`/`tree_dof_num` and only processes awake trees. Guard: `if !data.tree_awake[tree_id] { continue }`. Test T90 validates sleeping entries are preserved. |
| Phase 1: stale `crb_inertia` from previous step leaks into accumulation | Low | High (wrong composite inertia → wrong qM) | Phase 1 is `=` (overwrite), not `+=` (accumulate). Sleeping bodies' `crb_inertia` values are never read by Phase 2 or Phase 3 for awake trees (per-tree invariant). |
| Phase 2: backward pass misses a body in the tree | Low | High (incomplete composite inertia) | `body_awake_ind` contains ALL awake bodies, sorted ascending. Reverse iteration visits them leaf-to-root. Per-tree invariant: if one body in a tree is awake, all are. No body in an awake tree is missing from `body_awake_ind`. |
| Phase 2: sleeping root accumulation into body 0 skipped | None | None | Body 0 has no joints/DOFs, so `crb_inertia[0]` is never consumed by Phase 3. MuJoCo's `mj_crb()` accumulates sleeping roots into body 0 via `parent_awake_ind`, but the value is unused — our skip is equivalent. |
| Phase 3: `dof_parent` walk encounters sleeping DOF | None | N/A (cannot happen) | Walk starts from an awake DOF, follows `dof_parent`. Same-body DOFs share sleep state; cross-body links stay within the same kinematic tree (all awake). Walk terminates at `dof_parent = None` (root DOF). Cross-tree walks are impossible. |
| Phase 4: armature added to sleeping DOF | Low | Medium (small numerical drift) | `body_sleep_state` check matches Phase 3 pattern. Sleeping joints skipped. Test T94 validates armature correctness for awake joints. |
| Phase 6: sleeping body's cached mass reset to infinity | Low | Medium (constraint force limiting wrong) | No `body_min_mass.fill(f64::INFINITY)` for sleeping bodies. The `sleep_filter` guard in the visitor returns early, preserving existing values. Test T95 validates preservation. |
| `sleep_filter = false` path diverges from original | Low | High (regression) | Entire function has two code paths: `if !sleep_filter { original }` for each phase. Test T92 validates bit-identity when all awake. |
| Wake transition: first step after wake has stale qM | None | N/A | `mj_update_sleep_arrays()` runs before `mj_crba()` in `forward_core()`. Newly awake bodies are in `body_awake_ind`, so Phase 0 zeros their DOF block and Phases 1-4 compute fresh values. Test T91 validates. |

###### 16.29.4b Step C3a — Implementation Discoveries

Two pre-existing gaps were exposed during C3a implementation. Both were
invisible before C3a because the full-CRBA path always recomputed
everything, masking stale or missing data:

**Discovery 1: `mj_wake()` must trigger `mj_update_sleep_arrays()`.**

`mj_wake()` updates `tree_awake` and `body_sleep_state` (the semantic
sleep flags), but does NOT update the indirection arrays
(`body_awake_ind`, `nbody_awake`, `dof_awake_ind`, `nv_awake`). Before
C3a, this was harmless — full CRBA iterated `0..nbody` and `0..nv`
regardless. With selective CRBA, the indirection arrays drive Phases 1-3.
If a tree wakes via `mj_wake()` but the indirection arrays still reflect
the old state (e.g., `nbody_awake = 1` with only body 0), Phases 1-3
compute nothing for the newly woken tree, leaving its `qM` entries as
zeros → zero LDL diagonal → NaN in `qacc`.

**Fix:** `mj_wake()` now returns `bool` (true if any tree was woken).
In `forward_core()`, after `mj_wake(model, self)` returns `true`,
`mj_update_sleep_arrays(model, self)` is called to refresh the
indirection arrays before CRBA runs. This ensures that newly woken
bodies appear in `body_awake_ind` / `dof_awake_ind` and their DOF
blocks are properly zeroed and recomputed by Phases 0-4.

**Discovery 2: `make_data()` must run CRBA for init-asleep bodies.**

Bodies with `sleep="init"` policy start asleep. `make_data()` initializes
`qM` as a zero matrix (`DMatrix::zeros(nv, nv)`). Before C3a, the first
`forward()` call ran full CRBA, which overwrote the zeros for all DOFs.
With selective CRBA, if bodies start asleep, `sleep_filter = true` on
the first step, and Phases 1-4 skip the sleeping DOFs, preserving their
zero `qM` entries → zero LDL diagonal → NaN.

**Fix:** In `make_data()`, during the existing temporary wake-up for
init-asleep bodies (which already runs `mj_fwd_position()`), also call
`mj_crba()`. Before running FK + CRBA, save and restore `tree_asleep` /
`tree_awake` arrays, and call `mj_update_sleep_arrays()` so that
`sleep_filter` evaluates to `false` (all bodies appear awake), ensuring
full CRBA computes valid `qM` entries for every DOF. This guarantees
that even init-asleep bodies have a valid mass matrix before their
first awake step.

Both fixes are covered by existing tests: Discovery 1 by
`test_wake_on_contact` and `test_wake_on_xfrc_applied`; Discovery 2 by
`test_reset_restores_sleep_state` and `test_sleep_init_policy`.

###### 16.29.5 Step C3b — Partial LDL Factorization

**Status: ✅ Complete.**

**Scope:** `mj_factor_sparse_selective(model, data)` replaces
`mj_factor_sparse` in `mj_crba`'s Phase 5. When sleeping trees are
present, it skips DOF blocks belonging to sleeping trees. Implemented
alongside CSR migration — `qLD_L: Vec<Vec<(usize, f64)>>` replaced
with flat CSR storage (`qLD_data: Vec<f64>` + immutable Model metadata
`qLD_rowadr`/`qLD_rownnz`/`qLD_colind`), matching MuJoCo's `mj_factorI`.

**Precondition:** C3a complete.

**Key insight — tree independence simplifies everything:**

Sleeping is per-tree. `dof_parent` chains never cross tree boundaries
(`model.dof_parent[root_dof] == None` for each tree's root DOF). The
LDL factorization processes DOFs in reverse order; the elimination of
DOF `i` updates only its ancestors (DOFs reachable via `dof_parent`
chain), all of which are in the same tree.

**Consequence:** The LDL factorization is naturally block-diagonal
across trees. A sleeping tree's DOF block is completely independent of
all other trees' DOF blocks. Partial LDL simply means: **skip the
factorization for sleeping trees' DOF ranges entirely.**

This is dramatically simpler than the general "dof_factor_set"
approach described in the prior spec version. No complex reachability
analysis needed. No cross-tree contamination possible.

**Algorithm:**

```rust
fn mj_factor_sparse_selective(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let sleep_filter = sleep_enabled && data.nv_awake < model.nv;

    if !sleep_filter {
        mj_factor_sparse(model, data);  // Full factorization fast path
        return;
    }

    let (rowadr, rownnz, colind) = model.qld_csr();

    // Phase 1: Copy M into CSR for awake DOFs only.
    // Sleeping DOFs' qLD_diag and qLD_data entries are preserved.
    for idx in 0..data.nv_awake {
        let i = data.dof_awake_ind[idx];
        data.qLD_diag[i] = data.qM[(i, i)];
        let start = rowadr[i];
        let nnz = rownnz[i];
        for k in 0..nnz {
            data.qLD_data[start + k] = data.qM[(i, colind[start + k])];
        }
    }

    // Phase 2: Eliminate awake DOFs only (reverse order).
    // Uses bulk row update (mju_addToScl pattern) — no find() scan.
    for idx in (0..data.nv_awake).rev() {
        let i = data.dof_awake_ind[idx];
        let di = data.qLD_diag[i];
        let start_i = rowadr[i];
        let nnz_i = rownnz[i];
        if nnz_i == 0 { continue; }

        let inv_di = 1.0 / di;
        for k in 0..nnz_i {
            data.qLD_data[start_i + k] *= inv_di;
        }

        for a in (0..nnz_i).rev() {
            let j = colind[start_i + a];
            let lij = data.qLD_data[start_i + a];
            data.qLD_diag[j] -= lij * lij * di;
            // Bulk update: row_j[0..nnz_j] += scale * row_i[0..a]
            // (a == rownnz[j] by ancestor superset property)
            let start_j = rowadr[j];
            let scale = -lij * di;
            let (lo, hi) = data.qLD_data.split_at_mut(start_i);
            for k in 0..a {
                lo[start_j + k] += scale * hi[k];
            }
        }
    }

    data.qLD_valid = true;
}
```

**Correctness proof sketch:**

1. **No cross-tree contamination:** DOF `i`'s ancestor chain
   `dof_parent[i] → dof_parent[dof_parent[i]] → ... → None` stays
   within `i`'s tree. The rank-1 update `qLD_diag[j] -= lij² * di`
   only modifies `j` where `j` is an ancestor of `i` in the same tree.
   If `i` is awake, all its ancestors are awake (per-tree invariant).

2. **Sleeping DOFs untouched:** No sleeping DOF index appears in
   `dof_awake_ind`. The Phase 1 copy and Phase 2 elimination loops
   only visit awake DOFs. Sleeping DOFs' `qLD_diag` and `qLD_data`
   entries are not read or written.

3. **Ordering preserved:** The elimination must process DOFs from
   leaves to root (high index to low). Within each tree, awake DOFs
   maintain their relative order in `dof_awake_ind` (sorted ascending,
   iterated in reverse). Since elimination of DOF `i` only updates
   ancestors `j < i`, and all ancestors are awake and will be processed
   later (lower index = processed later in reverse iteration), the
   ordering constraint is satisfied.

4. **SPD preservation:** The original mass matrix `qM` restricted to
   any tree's DOF block is SPD (it's the mass matrix of a subsystem).
   The LDL factorization of an SPD matrix produces positive diagonal
   entries `D[i,i] > 0`. Since we factorize each awake tree's block
   independently (no cross-tree coupling), each block remains SPD.

**`mj_solve_sparse` — no changes needed:**

The solve function `mj_solve_sparse(qld_diag, rowadr, rownnz, colind,
qld_data, x)` reads all `nv` entries of `qLD_diag` and `qLD_data`.
For sleeping DOFs:
- Phase 1 (L^T solve): `x[j] -= lij * x[i]`. For sleeping DOF `i`,
  `x[i] = 0` (sleeping DOFs have zero RHS), so the subtraction is
  zero. For sleeping DOF `j` as an ancestor, `j` is in the same tree
  as `i` (both sleeping), so `x[i] = 0` again.
- Phase 2 (D solve): `x[i] /= qLD_diag[i]`. For sleeping DOFs,
  `x[i] = 0`, so `0 / D[i] = 0` (safe as long as `D[i] != 0`, which
  is guaranteed by the SPD property from the last awake factorization).
- Phase 3 (L solve): Same argument as Phase 1.

**Result:** `mj_solve_sparse` produces correct results (zero for
sleeping DOFs, correct for awake DOFs) with stale `qLD` entries for
sleeping DOFs. No modification needed.

###### 16.29.6 C3b — Risk Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Ordering violation in elimination | Low | High (wrong M^{-1}) | Reverse iteration of sorted `dof_awake_ind` preserves leaf-to-root order. Unit test T99 validates awake qLD against full factorization. |
| Numerical instability from stale sleeping qLD | Low | Medium (solve divergence) | Tree independence guarantees no cross-contamination. SPD property preserved per-block. Test T104 validates positive diagonal (SPD preservation). |
| Performance regression when all awake | Low | Low (wasted branch) | `sleep_filter` short-circuit returns to original full path. Test T103 verifies all-awake bit-identity. |
| `qLD_diag[sleeping] = 0` causing division by zero in solve | None | High | Sleeping DOFs' `qLD_diag` entries are preserved from last awake step (positive by SPD). Never zeroed. |
| Stale `qLD_data` entries causing incorrect awake solve | None | High | Tree independence: awake DOF's ancestor chain is entirely within the awake tree. No sleeping DOF's `qLD_data` is read during awake elimination. |

**Implementation note:** C3b was implemented proactively alongside the
CSR migration. The flat CSR storage eliminates the O(depth) `find()`
scan from the original `Vec<Vec<(usize, f64)>>` approach, making the
partial factorization both correct and performant.

###### 16.29.7 Acceptance Criteria

**C3a (Selective CRBA body loops):**

59. Selective CRBA produces identical `qM` entries for awake DOFs as
    full CRBA (bit-identical, not epsilon-close).
60. `qLD_diag` and `qLD_data` for **awake** DOFs are identical whether
    the input `qM` came from selective CRBA or full CRBA. (Both run
    full `mj_factor_sparse`, but sleeping DOFs' `qM` entries differ:
    zeros from full CRBA vs stale-from-last-awake from selective.
    Since LDL is tree-block-diagonal, awake trees' `qLD` blocks are
    identical regardless of sleeping trees' `qM` values. Sleeping
    trees' `qLD` entries under selective CRBA are *more correct*
    than under full CRBA — stale-but-valid vs degenerate-from-zero.)
61. Sleeping DOFs' `qM` entries are preserved from their last awake
    step (not zeroed, not corrupted, not partially overwritten).
62. When a sleeping tree wakes, its next `forward()` pass produces
    identical `qM` and `qLD` to a never-slept simulation.
63. With all bodies awake, selective CRBA is bit-identical to full CRBA
    (the `sleep_filter = false` fast path is exercised).
64. `crb_inertia` for sleeping bodies is preserved across steps
    (Phase 1 skips sleeping bodies, so `crb_inertia[sleeping_body]`
    retains its value from the last awake step).
65. Phase 4 armature addition correctly preserves both `jnt_armature`
    and `dof_armature` semantics for awake joints.
66. `cache_body_effective_mass` for sleeping bodies is preserved from
    their last awake step (not recomputed, not zeroed).
67. Multi-tree scenario: 3 trees (A awake, B sleeping, C awake), each
    with 2 hinge joints. Trees A and C have correct `qM`. Tree B's
    `qM` entries are stale-but-valid. `qLD` is correct for all DOFs.
68. Deep chain scenario: 10-link hinge chain, alternating trees of
    depth 5. With one tree sleeping, the awake tree's off-diagonal
    `qM` entries (ancestor walks) are correct.
69. Performance: scene with 100 resting bodies (50 sleeping trees) and
    1 active tree — `mj_crba` with `sleep_filter = true` is measurably
    faster than with `sleep_filter = false` (target: ≥ 30% reduction in
    CRBA phase time).

**C3b (Partial LDL factorization):**

70. Partial LDL produces identical `qLD_diag` and `qLD_data` for awake
    DOFs as full `mj_factor_sparse` (bit-identical).
71. Sleeping DOFs' `qLD_diag` and `qLD_data` entries are preserved from
    their last awake step (not modified by partial elimination).
72. `mj_solve_sparse` with partial `qLD` produces identical `qacc`
    for awake DOFs as full `qLD` (when RHS has zero entries for
    sleeping DOFs).
73. When a sleeping tree wakes, the next full `mj_factor_sparse`
    produces identical results to a never-slept factorization.
74. With all bodies awake, `mj_factor_sparse_selective` is bit-identical
    to `mj_factor_sparse` (fast path exercised).
75. Partial LDL does not produce negative `qLD_diag` entries (SPD
    preservation). Checked via assertion on each awake DOF's diagonal.
76. Performance: partial LDL on a model with nv=200, 50% sleeping DOFs,
    is measurably faster than full LDL (target: proportional to
    nv_awake/nv ratio).

###### 16.29.8 Tests

**C3a Tests:**

| # | Test | Type | Validates |
|---|------|------|-----------|
| T89 | `test_selective_crba_awake_identical` | Unit | AC #59: qM identical for awake DOFs |
| T90 | `test_selective_crba_sleeping_preserved` | Unit | AC #61: sleeping qM entries preserved across steps |
| T91 | `test_selective_crba_wake_recomputes` | Integration | AC #62: waking tree gets fresh qM + qLD |
| T92 | `test_selective_crba_all_awake_noop` | Regression | AC #63: bit-identical when all awake |
| T93 | `test_selective_crba_crb_inertia_preserved` | Unit | AC #64: crb_inertia frozen for sleeping bodies |
| T94 | `test_selective_crba_armature_correct` | Unit | AC #65: jnt_armature + dof_armature both applied for awake |
| T95 | `test_selective_crba_body_mass_cache_preserved` | Unit | AC #66: body_min_mass/inertia frozen for sleeping |
| T96 | `test_selective_crba_multi_tree` | Integration | AC #67: 3-tree mixed awake/sleeping scenario |
| T97 | `test_selective_crba_deep_chain` | Integration | AC #68: 10-link chain with partial sleeping |
| T98 | `test_selective_crba_qld_awake_matches_full` | Regression | AC #60: awake trees' qLD blocks identical whether qM came from selective or full CRBA |

**C3b Tests:**

| # | Test | Type | Validates |
|---|------|------|-----------|
| T99 | `test_partial_ldl_awake_identical` | Unit | AC #70: qLD identical for awake DOFs |
| T100 | `test_partial_ldl_sleeping_preserved` | Unit | AC #71: sleeping qLD entries preserved |
| T101 | `test_partial_ldl_solve_correct` | Integration | AC #72: mj_solve_sparse produces correct qacc with partial qLD |
| T102 | `test_partial_ldl_wake_recomputes` | Integration | AC #73: waking tree gets fresh qLD |
| T103 | `test_partial_ldl_all_awake_noop` | Regression | AC #74: bit-identical when all awake |
| T104 | `test_partial_ldl_spd_preserved` | Unit | AC #75: no negative qLD_diag entries |
| T105 | `test_partial_ldl_multi_tree_independence` | Unit | AC #70+71: tree A's factorization doesn't touch tree B's qLD |
| T106 | `test_partial_ldl_solve_zero_sleeping_rhs` | Unit | AC #72: solve with zero RHS for sleeping DOFs yields zero output |

##### 16.30 Phase C — Implementation Order and Test Plan

###### 16.30.1 Construction Order

Phase C has four steps, ordered by dependency and risk:

| Step | Section | Description | Depends On | Risk | Status |
|------|---------|-------------|------------|------|--------|
| C1 | §16.27 | Awake-index pipeline iteration | Phase B complete | Low | ✅ Done |
| C2 | §16.28 | Island-local Delassus assembly | C1 (uses `dof_awake_ind` patterns) | Medium | ✅ Done |
| C3a | §16.29 | Selective CRBA body loops | C1 (uses `body_awake_ind`) | Medium | ✅ Done |
| C3b | §16.29 | Partial LDL factorization | C3a + profiling trigger | Medium | ✅ Done |

**C1 first** because it establishes the indirection iteration pattern
that C2 and C3 build on. It's also the lowest-risk change — purely
mechanical loop transformations with no algorithmic changes.

**C2 second** because it delivers the largest performance win and is
architecturally independent of CRBA. The island-local mass matrix
extraction uses the global `qM` (which is still fully computed in C2).

**C3a third** because it depends on the indirection pattern from C1
and benefits from the island-local solver in C2 (which no longer reads
sleeping DOFs' `qM` entries, reducing the risk of stale values).
C3a preserves full LDL factorization — no correctness risk from
partial elimination.

**C3b complete.** Implemented alongside the CSR migration (flat CSR
storage matching MuJoCo's `mj_factorI`). `mj_factor_sparse_selective`
dispatches to partial factorization when sleeping trees are present,
skipping sleeping DOFs entirely. Tests T99–T106 validate all acceptance
criteria #70–76.

**C3a implementation sub-order:**
1. Phase 0: Selective `qM` zeroing (per-tree diagonal blocks).
2. Phase 1: Gate `crb_inertia` init on `body_awake_ind`.
3. Phase 2: Gate backward accumulation on `body_awake_ind` (reverse).
4. Phase 3: Gate DOF loop on `dof_awake_ind` (outer loop indirection).
   `joint_subspaces` and `cdof` computed unconditionally, gating via outer loop only.
5. Phase 4: Gate armature loop on `body_sleep_state`.
6. Phase 6: Gate `cache_body_effective_mass` via `sleep_filter` in
   visitor struct.
7. Add tests T89-T98, verify all Phase A/B tests pass.
8. Profile and measure CRBA speedup for target scenario (AC #69).

###### 16.30.2 Per-Step Verification

Each step must pass:
1. All Phase A tests (T1-T30)
2. All Phase B tests (T31-T76)
3. The step's own Phase C tests
4. `cargo clippy -- -D warnings`
5. `cargo fmt --all -- --check`
6. Domain test suite: `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics -p sim-constraint`

After all steps:
1. Full workspace: `cargo test`
2. Quality gate: `cargo xtask check`

###### 16.30.3 Complete Phase C Test Table

| # | Test | Type | Step | Validates |
|---|------|------|------|-----------|
| T77 | `test_indirection_velocity_equivalence` | Unit | C1 | AC #48 |
| T78 | `test_indirection_integration_equivalence` | Unit | C1 | AC #50 |
| T79 | `test_energy_sleeping_zero_ke` | Unit | C1 | AC #49 |
| T80 | `test_all_awake_bit_identical` | Regression | C1 | AC #47 |
| T81 | `test_indirection_rne_gyroscopic_equivalence` | Unit | C1 | AC #51 |
| T82 | `test_rne_featherstone_sleeping_zero_contribution` | Unit | C1 | AC #53 |
| T83 | `test_per_function_bit_identity` | Regression | C1 | AC #54 |
| T84 | `test_energy_continuous_across_sleep_transition` | Integration | C1 | AC #49 |
| T85 | `test_island_delassus_equivalence` | Unit | C2 | AC #55 |
| T86 | `test_island_solve_forces_match_global` | Integration | C2 | AC #57 |
| T87 | `test_single_island_uses_global_path` | Unit | C2 | AC #58 |
| T88 | `test_disable_island_phase_c_bit_identical` | Regression | C2 | AC #56 |
| T89 | `test_selective_crba_awake_identical` | Unit | C3a | AC #59 |
| T90 | `test_selective_crba_sleeping_preserved` | Unit | C3a | AC #61 |
| T91 | `test_selective_crba_wake_recomputes` | Integration | C3a | AC #62 |
| T92 | `test_selective_crba_all_awake_noop` | Regression | C3a | AC #63 |
| T93 | `test_selective_crba_crb_inertia_preserved` | Unit | C3a | AC #64 |
| T94 | `test_selective_crba_armature_correct` | Unit | C3a | AC #65 |
| T95 | `test_selective_crba_body_mass_cache_preserved` | Unit | C3a | AC #66 |
| T96 | `test_selective_crba_multi_tree` | Integration | C3a | AC #67 |
| T97 | `test_selective_crba_deep_chain` | Integration | C3a | AC #68 |
| T98 | `test_selective_crba_qld_awake_matches_full` | Regression | C3a | AC #60 |
| T99 | `test_partial_ldl_awake_identical` | Unit | C3b | AC #70 |
| T100 | `test_partial_ldl_sleeping_preserved` | Unit | C3b | AC #71 |
| T101 | `test_partial_ldl_solve_correct` | Integration | C3b | AC #72 |
| T102 | `test_partial_ldl_wake_recomputes` | Integration | C3b | AC #73 |
| T103 | `test_partial_ldl_all_awake_noop` | Regression | C3b | AC #74 |
| T104 | `test_partial_ldl_spd_preserved` | Unit | C3b | AC #75 |
| T105 | `test_partial_ldl_multi_tree_independence` | Unit | C3b | AC #70+71 |
| T106 | `test_partial_ldl_solve_zero_sleeping_rhs` | Unit | C3b | AC #72 |

##### 16.9 Performance Considerations

**Branch cost.** Each skip check adds a branch per body/DOF in FK, RNE,
velocity kinematics, passive forces, collision, integration, and sensors.
For awake bodies this is a single predicted-taken branch with negligible
cost. For sleeping bodies the branch saves the entire computation for
that body. CRBA and LDL factorization are NOT skipped in Phase A/B
(§16.5 mass matrix safety note) — this is a Phase C optimization
(§16.29: selective CRBA body loops + partial LDL).

**Memory.** New fields add `O(ntree + nbody + nv)` storage — negligible
relative to existing Data allocations.

**Cache locality.** In MJCF models, bodies within a tree are typically
contiguous in memory (added in document order, depth-first). The skip
check in the per-body loops is a single indexed load (`body_sleep_state
[body_id]`) with excellent branch-prediction characteristics (sleeping
bodies form long runs of skips). For models where tree bodies are
non-contiguous, the skip checks still use sequential iteration over the
same arrays.

**Collision.** The sleep-vs-sleep filter in `mj_collision()` (§16.5b)
eliminates O(n²) narrow-phase calls for n sleeping geoms. Broad-phase
cost remains O(n log n) for all geoms (AABBs are still in the SAP structure)
but narrow-phase is the dominant cost.

**Worst case.** All trees awake: every body checks one `bool` per pipeline
stage — zero measurable overhead. All trees asleep: FK, RNE, velocity
kinematics, passive forces, collision, and integration skip immediately;
CRBA and LDL factorization still run at full cost (Phase A). Mixed:
linear cost in awake bodies for skipped stages.

#### Acceptance Criteria

**Correctness:**
1. With `ENABLE_SLEEP` cleared, behavior is bit-identical to the pre-sleep
   codebase. The only overhead is O(1) flag checks that short-circuit
   immediately — no sleep state is read, no body is skipped.
2. A single free-falling body with sleep enabled: body falls, comes to rest
   on a plane, velocity drops below threshold, body sleeps after exactly
   `MIN_AWAKE` sub-threshold steps, `qvel` and `qacc` are exactly zero,
   body pose is frozen.
3. Sleeping body wakes when `xfrc_applied` is set to nonzero. Wakes when
   `qfrc_applied` is set to nonzero on any of its DOFs.
4. Sleeping body wakes when an awake body's geom contacts one of its geoms.
   After wake, the body participates in collision response on the same step.
5. Tree `sleep="never"` policy: body never sleeps regardless of velocity.
6. Tree `sleep="init"` policy: body starts asleep, wakes on first contact
   or applied force.
7. RK4 + sleep enabled: warning is emitted and sleep is silently disabled.
8. Body 0 (world) is always `Static`, never `Asleep` or `Awake`.

**Conformance:**
9. Scene with 10 resting bodies (free joints on a ground plane, sleep
   enabled): after 10+ steps of rest, all bodies are asleep. Drop a ball
   onto one — it wakes, its contact partner wakes, others remain asleep.
   Matches MuJoCo behavior qualitatively (exact floating-point match is
   not required; threshold and timer semantics must match).
10. `sleep_tolerance` parameter: halving the tolerance roughly doubles the
    time to sleep (bodies must be more still). Verify with a damped
    pendulum that decays through the threshold.

**Performance:**
11. Scene with 100 resting bodies (asleep) and 1 active body: `step()` time
    is measurably faster than the same scene with sleep disabled (all bodies
    active). Target: ≥ 3× speedup for the 100:1 ratio. Measured via
    `criterion` benchmark.
12. Scene with all bodies awake and sleep enabled: step time is within 2%
    of sleep-disabled (overhead of skip checks is negligible).

**Integration:**
13. `BatchSim` with sleep: different environments can have different sleep
    states. Environment 0 has all bodies asleep; environment 1 has all
    awake. `step_all()` produces correct results for both.
14. All existing sim domain tests pass unchanged (`cargo test -p sim-core
    -p sim-conformance-tests -p sim-physics`). Sleep is disabled by default,
    so existing tests see zero behavior change.
15. Sensors on sleeping bodies report their last-computed values (not NaN
    or zero). Sensor values are frozen at the moment of sleep, not zeroed.

#### Test Plan

| # | Test | Type | Validates |
|---|------|------|-----------|
| T1 | `test_sleep_disabled_noop` | Unit | AC #1: no behavior change when sleep disabled |
| T2 | `test_tree_enumeration` | Unit | §16.0: tree arrays are correct for multi-tree model |
| T3 | `test_sleep_policy_resolution` | Unit | §16.0: actuated trees get `AutoNever`, others get `AutoAllowed` |
| T4 | `test_sleep_countdown_timer` | Unit | §16.3: tree sleeps after exactly MIN_AWAKE sub-threshold steps |
| T5 | `test_sleep_zeros_velocity` | Unit | §16.3: sleeping tree has qvel=0, qacc=0 |
| T6 | `test_wake_on_xfrc_applied` | Unit | AC #3: xfrc_applied wakes sleeping body |
| T7 | `test_wake_on_qfrc_applied` | Unit | AC #3: qfrc_applied wakes sleeping DOF's tree |
| T8 | `test_wake_on_contact` | Integration | AC #4: awake body colliding with sleeping body wakes it |
| T9 | `test_sleep_never_policy` | Unit | AC #5: `Never` policy prevents sleep |
| T10 | `test_sleep_init_policy` | Unit | AC #6: `Init` policy starts body asleep |
| T11 | `test_rk4_sleep_warning` | Unit | AC #7: RK4 + sleep → warning + sleep disabled |
| T12 | `test_world_body_static` | Unit | AC #8: body 0 always Static |
| T13 | `test_sleep_wake_scenario` | Integration | AC #9: multi-body scene with selective wake |
| T14 | `test_sleep_tolerance_scaling` | Integration | AC #10: threshold affects sleep onset |
| T15 | `test_sleep_performance` | Benchmark | AC #11: 100:1 scene ≥ 3× faster |
| T16 | `test_sleep_overhead` | Benchmark | AC #12: all-awake ≤ 2% overhead |
| T17 | `test_batch_sleep_independence` | Integration | AC #13: per-environment sleep state |
| T18 | `test_existing_tests_pass` | Regression | AC #14: `cargo test` on sim domain |
| T19 | `test_sensor_frozen_on_sleep` | Unit | AC #15: sensors report last value, not zero |
| T20 | `test_fk_skip_sleeping_body` | Unit | §16.5a: FK not called for sleeping bodies |
| T21 | `test_collision_skip_both_sleeping` | Unit | §16.5b: no narrow-phase for sleeping-sleeping pairs |
| T22 | `test_mjcf_sleep_attributes` | Unit | §16.2: MJCF parsing of sleep options |
| T23 | `test_dof_length_computation` | Unit | §16.0: dof_length correct for hinge/slide/free |
| T24 | `test_passive_force_skip_sleeping` | Unit | §16.5a': passive forces not computed for sleeping joints |
| T25 | `test_position_integration_skip` | Unit | §16.5a'': qpos unchanged for sleeping joints via visitor |
| T26 | `test_tendon_passive_mixed_sleep` | Unit | §16.5a': tendon with mixed awake/sleeping targets computes force |
| T27 | `test_forward_skip_sensors_sleep` | Unit | Sleep logic active in forward_skip_sensors (RK4 path) |
| T28 | `test_sensor_frozen_value_not_zero` | Unit | §16.5d: sensor value preserved (not zeroed) after sleep |

#### Files

| File | Action | Description |
|------|--------|-------------|
| `sim/L0/core/src/mujoco_pipeline.rs` | Modify | Model fields (§16.0), Data fields (§16.1), sleep update (§16.3), wake detection (§16.4), pipeline skip logic (§16.5), RK4 guard (§16.6), init/reset (§16.7) |
| `sim/L0/mjcf/src/parser.rs` | Modify | Parse `sleep_tolerance`, `<flag sleep>`, `<body sleep>` attributes (§16.2) |
| `sim/L0/mjcf/src/types.rs` | Modify | Add `SleepPolicy` to MJCF types, `sleep` field on body element |
| `sim/L0/mjcf/src/model_builder.rs` | Modify | Propagate sleep attributes to `Model` during build |
| `sim/L0/tests/integration/mod.rs` | Modify | Register new test modules |
| `sim/L0/tests/integration/sleeping.rs` | Create | Tests T1–T14, T17–T28 |
| `sim/L0/core/benches/sleep_benchmarks.rs` | Create | Benchmarks T15–T16 |

---

### ~~17. SOR Relaxation for PGS~~ — DROPPED

**Status:** Dropped | **Reason:** Incorrect premise; MuJoCo does not implement SOR

#### Why This Was Dropped

The original spec claimed "MuJoCo's PGS uses SOR with configurable omega." This is
factually incorrect. Verification against MuJoCo's source code (`engine_solver.c`,
`mj_solPGS`) and the `mjOption` struct (`mjmodel.h`) confirms:

1. **MuJoCo's PGS uses pure Gauss-Seidel (ω=1.0).** The update is a direct
   coordinate-descent step: `force[i] -= res[0] * ARinv[i]`, followed by projection.
   No SOR blending is applied.
2. **There is no `sor` field** in `mjOption`.
3. **There is no `<option sor="..."/>` attribute** in the MJCF XML schema.

MuJoCo's answer to slow PGS convergence is the Newton solver (quadratic convergence,
2-5 iterations), not SOR. Our Newton solver (§15) already provides this.

SOR *is* used by ODE (default ω=1.3) and Bullet, but those engines lack a Newton
fallback — their PGS is the only solver. For CortenForge, PGS is either:
- The primary solver for simple scenes (converges fine without SOR), or
- A fallback for degenerate cases (where ω>1 risks divergence on cone-constrained QCQPs)

Adding a non-MuJoCo knob with no convergence guarantee and narrow model-dependent
sweet spots is net-negative complexity.

---

## Appendix: Deferred / Low Priority

Items acknowledged but not prioritized for Phase 2:

| Item | Reason for deferral |
|------|-------------------|
| `dampratio` attribute (Position actuators) | Builder-only change, no RL models use it. Deferred in Phase 1 #12. |
| Explicit Euler integrator | MuJoCo supports it but semi-implicit Euler is strictly superior. No use case. |
| Velocity Verlet integrator | Not in MuJoCo. Was in standalone system (deleted). No pipeline need. |
| ~~Implicit-fast (no Coriolis)~~ | ✅ Implemented in §13 as `Integrator::ImplicitFast`. |
| Planar/Cylindrical joints in pipeline | In sim-constraint standalone. No MJCF models use them. |
| `<custom>` element (user data) | Low priority — no simulation effect. |
| `<extension>` element (plugins) | Large design effort, no immediate need. |
| `<visual>` element | L1 concern (sim-bevy), not core physics. |
| `<statistic>` element | Auto-computed model stats. Informational only. |
| ~~Sleeping bodies in deformable~~ | ✅ Complete (Phase C — trees with deformables receive `AutoNever` policy) |
| ~~Separate `qLD_diag` field (diagonal layout divergence)~~ | ✅ **Fixed.** Unified diagonal layout implemented: D[i,i] is now stored as the last element of each CSR row in `qLD_data` (`qLD_data[rowadr[i] + rownnz[i] - 1]`), eliminating the separate `qLD_diag` array. `qLD_diag_inv[i] = 1/D[i,i]` is precomputed during factorization so the solve phase uses multiply (`x[i] *= diag_inv[i]`) instead of divide. This matches MuJoCo's inline diagonal storage and removes the layout divergence. Note: pseudocode in §16.29.5 (C3b partial factorization) still references the old `qLD_diag` field name for historical context; the implementation uses the unified layout. |
| Sparse mass matrix (deeper MuJoCo parity) | Phase 1 #1/#2 cover the main path. Full sparse pipeline is diminishing returns. |
| MuJoCo conformance test suite | Important but orthogonal to features — can be built incrementally. Without this, acceptance criteria for items #1–#16 rely on ad-hoc verification rather than systematic comparison against MuJoCo reference outputs. Consider bootstrapping a minimal conformance harness (load model, step N times, compare state vectors against MuJoCo ground truth) as infrastructure that benefits all items. |
| SIMD utilization (unused batch ops) | sim-simd exists; utilization will come naturally with #9/#10. |
| ~~Tendon equality constraints~~ | ✅ **Implemented** in §37. Pipeline `EqualityType::Tendon` + `extract_tendon_equality_jacobian()` — two-tendon polynomial coupling + single-tendon lock, fully integrated into unified solver. |
| ~~SOR relaxation for PGS~~ | **Dropped.** Original spec incorrectly claimed MuJoCo uses SOR — it does not (`mjOption` has no `sor` field, `mj_solPGS` uses pure GS). Newton solver (§15) supersedes PGS acceleration. ODE/Bullet use SOR but lack Newton fallback. |

---

## Cross-Reference: Phase 1 Mapping

| Phase 1 # | Phase 2 # | Notes |
|-----------|-----------|-------|
| #9 (Deformable Body) | #11 | Transferred verbatim ✅ Complete |
| #10 (Batched Simulation) | #9 | Transferred verbatim |
| #11 (GPU Acceleration) | #10 | Transferred verbatim |
