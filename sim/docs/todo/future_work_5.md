# Future Work 5 — Quality of Life + Appendix (Items #15–17)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 15. Newton Solver
**Status:** Phase B Complete | **Effort:** XL | **Prerequisites:** None

#### Current State
The standalone Newton solver implementation was deleted during Phase 3 crate
consolidation (commit `a5cef72`) — `sim/L0/constraint/src/newton.rs` (2,273
lines) was removed. The pipeline has PGS and CG (PGD+BB) solvers.

**MJCF-layer remnants still exist.** `MjcfSolverType::Newton` is defined in
`types.rs:99` and is the MuJoCo default. The parser accepts `solver="Newton"`
(`parser.rs:177`). The model builder silently maps it to PGS:
`MjcfSolverType::PGS | MjcfSolverType::Newton => SolverType::PGS`
(`model_builder.rs:745`). Users who load standard MJCF models (which default to
Newton) get PGS without any warning.

MuJoCo's Newton solver uses analytical second-order derivatives of the constraint
cost and typically converges in 2–3 iterations, making it faster than PGS for
stiff problems. It is the recommended solver for most MuJoCo applications.

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

##### 15.0 Architectural Prerequisite: Unified Constraint Assembly

**Gap:** MuJoCo's Newton solver operates on a **unified constraint Jacobian**
`J` that stacks rows for ALL constraint types (equality, friction loss, joint
limits, tendon limits, contacts). Our architecture handles limits and equality
as penalty forces written directly to `qfrc_constraint` *before* the contact
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

**Friction loss migration.** MuJoCo has **always** treated `frictionloss` as
constraint rows (`mjCNSTR_FRICTION_DOF`, `mjCNSTR_FRICTION_TENDON`) with
Huber cost — it was never a passive force in MuJoCo. Our pipeline diverged
from MuJoCo here from the start: we handle frictionloss in `mj_fwd_passive()`
via `−frictionloss · tanh(qvel · friction_smoothing)` (where
`friction_smoothing` defaults to 1000.0, configurable on Model) written to
`qfrc_passive` (lines 9729–9738). This smooth `tanh` approximation differs
from MuJoCo's Huber-cost constraint semantics. For the Newton solver,
frictionloss must be handled as proper constraint rows matching MuJoCo:
- DOF friction: 1×nv sparse row with `1.0` at the DOF index
- Tendon friction: 1×nv row = `ten_J[t]`
- `floss = frictionloss` value from the joint/tendon

For PGS/CG backward compatibility, friction loss remains in `mj_fwd_passive()`
when the solver is not Newton. The Newton path skips friction loss in passive
forces and instead includes it in J.

**Friction loss skip mechanism:** `mj_fwd_passive()` runs before
`mj_fwd_constraint()` and currently always writes friction loss into
`qfrc_passive`. Two approaches (either is acceptable):
- **(a)** Pass `solver_type` to `mj_fwd_passive()`; when Newton, skip the
  friction loss loop. `qfrc_passive` then naturally excludes friction loss.
- **(b)** Always compute full `qfrc_passive`; store the friction loss
  contribution separately in a new `Data.qfrc_frictionloss: DVector<f64>`.
  The Newton path computes `qfrc_smooth` using
  `qfrc_passive − qfrc_frictionloss`. PGS/CG use `qfrc_passive` as-is.
Approach (b) is preferred because it avoids coupling `mj_fwd_passive()` to
solver type and preserves the friction loss vector for diagnostics and PGS
fallback (§15.12 Phase A step 14). With approach (b), PGS fallback can use
`qfrc_passive` directly (which includes friction loss) without re-running
`mj_fwd_passive()`.

**Compatibility:** PGS and CG continue to operate on contacts-only via the
existing path (penalty limits/equality + contact-only solver). Newton uses the
unified system. The dispatch in `mj_fwd_constraint()` branches: PGS/CG take
the existing contact-only path; Newton takes the new unified path. This avoids
regressing PGS/CG behavior.

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
- `efc_mu[i]`: 5-element friction coefficients (contacts only, `[0;5]` otherwise)
- `efc_dim[i]`: constraint group size for stepping. For elliptic contacts:
  `dim` for all rows belonging to the same contact group. For everything else
  (equality, limits, friction loss, `ContactNonElliptic`): `1`. Multi-row
  equality constraints (Connect=3, Weld=6) have `efc_dim = 1` per row because
  Newton treats them as independent scalar Quadratic constraints. `efc_dim > 1`
  only indicates elliptic friction cone grouping

##### 15.1 Constraint State Machine

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

##### 15.2 Cost Function and Gradient

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

##### 15.3 Hessian and Incremental Updates

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

##### 15.4 Gradient Computation and Newton Direction

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

##### 15.5 Line Search: 1D Exact Newton

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

##### 15.6 Convergence Criteria

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

##### 15.7 Elliptic Friction Cones

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

##### 15.8 Outer Iteration Loop

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

##### 15.9 Warm Start

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

##### 15.11 Integration Points

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

**Phase A — Unified constraint assembly + Core Newton (minimum viable):**
1. `ConstraintType`, `ConstraintState` enums, all new `Data`/`Model` fields
   (§15.11). Compute `Model.stat_meaninertia = trace(M) / nv` at model build
   time (run CRBA at `qpos0`, take trace of resulting M, divide by nv)
2. `diagApprox` computation — for ALL constraint rows (contacts and non-contacts
   alike), compute `diagApprox_i = J_i · M⁻¹ · J_i^T` by solving `M · w = J_i^T`
   via forward/back substitution against the pre-existing mass matrix factorization
   (`qLD`), then computing `diagApprox_i = J_i · w` (one dot product). This is
   O(nv) per row and O(nv · nefc) total, acceptable for Phase A. Phase C may
   extract contact-row diagonals more efficiently from a partial Delassus assembly
   or use the body-weight approximation from §15.1 for sparse/large systems
3. `compute_aref()` — reference acceleration from solref/solimp per row, using
   the full KBIP derivation (§15.1) with `|pos − margin|` impedance
4. **Prerequisite refactor:** Extract explicit Jacobian rows from
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
5. Friction loss migration: store friction loss contribution separately
   (`Data.qfrc_frictionloss`) per §15.0 approach (b). This must precede
   unified assembly because: (a) friction loss rows need to be available
   for inclusion in J, and (b) `qfrc_smooth` computation needs
   `qfrc_frictionloss` to be separated from `qfrc_passive`
6. Unified Jacobian assembly: `assemble_unified_constraints()` that builds
   `efc_J`, `efc_aref`, `efc_D`, `efc_R`, `efc_imp`, `efc_type`, `efc_floss`,
   `efc_pos`, `efc_margin`, `efc_vel`, `efc_solref`, `efc_solimp`,
   `efc_diagApprox`, `efc_mu`, `efc_dim`, `efc_id`, `efc_b` using the extracted
   Jacobians from step 4, existing contact Jacobians, and friction loss rows
   from step 5. `efc_b = J · qacc_smooth − aref` is computed after assembly
   for use in warmstart cost comparison (§15.8)
7. `classify_constraint_states(jar, Ma, qfrc_smooth, qacc, qacc_smooth)
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
8. `PrimalUpdateGradient` (§15.4): compute `qfrc_constraint = J^T·efc_force`,
   `grad = Ma − qfrc_smooth − qfrc_constraint`, `Mgrad = H⁻¹·grad` via
   Cholesky solve, `search = −Mgrad`
9. Full Hessian assembly `H = M + J^T·D·J` with dense Cholesky
10. Simple backtracking line search (Armijo) as initial placeholder: try
    `α = 1`, halve until cost decreases (`cost(α) < cost(0)`), up to
    20 halvings (i.e., minimum `α ≈ 1e-6`); return `α = 0` if no
    improvement found. Each halving evaluates full cost (Gauss +
    constraint) via a temporary `PrimalUpdateConstraint` call at the
    trial `qacc + α·search` — this is O(nefc) per evaluation, so
    the worst case is 20 constraint evaluations per outer iteration.
    The §15.5 exact 1D Newton line search replaces this in Phase B.
    The pseudocode in §15.8 calls `primal_line_search(...)`; for
    Phase A, this is the Armijo version with the same return signature
11. Outer loop with convergence check
12. `SolverType::Newton` wiring + `model_builder.rs` fix +
    `forward()` and `forward_skip_sensors()` dispatch to skip
    `mj_fwd_acceleration()` for Newton (Euler and RK4 only; Newton +
    implicit → warn and fall back to PGS per §15.8)
13. `qacc_warmstart` save at end of `Data::step()`, integrator-independent
    (§15.9). Remove the incorrect RK4 mid-step save at line 13990
14. PGS fallback on non-convergence **or Cholesky failure**: re-dispatch through the PGS/CG path
    from the beginning — run penalty-based limit/equality forces (the existing
    `apply_*` functions writing to `qfrc_constraint`), then invoke PGS on the
    contact-only system. The unified constraint assembly is discarded.
    **Important:** PGS uses `qfrc_passive` which includes friction loss
    (approach (b) keeps `qfrc_passive` complete), so no re-run of
    `mj_fwd_passive()` is needed. The CG fallback pattern applies.
    **Stale fields:** On fallback, the Newton-specific Data fields (`efc_*`,
    `efc_state`, `efc_force`, `efc_jar`, `efc_cost`) from the failed Newton
    solve must be cleared or overwritten. PGS writes its own `qfrc_constraint`
    and `qacc` (via `mj_fwd_acceleration()`), overriding Newton's values.
    The `efc_*` arrays should be zeroed/truncated to avoid stale unified-system
    data persisting alongside PGS's contact-only results.

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
1. Noslip post-processor (if needed)
2. Sparse Hessian path: when `nv > ~60` (articulated humanoids and above),
   switch from dense O(nv³) Cholesky to sparse supernodal LDL^T (CSC format).
   The sparsity pattern of `H = M + J^T·D·J` is fixed per assembly (depends on
   kinematic tree + active constraint topology), so symbolic factorization can
   be cached across Newton iterations within a timestep and only recomputed
   when `nefc` changes. Expected complexity: O(nv · fill²) where fill is the
   Cholesky fill-in, typically O(nv^1.5) for tree-structured robots. Evaluate
   `cholmod` (via `suitesparse` crate) or a custom supernodal implementation
3. Solver statistics: iteration count, cost, gradient norm, active constraints
4. Evaluate per-step `meaninertia = trace(M) / nv` vs the model-level
   constant from `qpos0`. The model-level constant (used in Phase A,
   matching MuJoCo) is an approximation that may degrade for models with
   large configuration-dependent inertia variation. Per-step computation
   is more accurate but adds O(nv) cost per solve

#### Acceptance Criteria

1. **Correctness:** Newton solver converges to the same `qacc` as PGS/CG
   (within `1e-6` relative tolerance) on conformance test models without
   `frictionloss`. (Models with `frictionloss > 0` use Huber cost in Newton
   vs. `tanh` approximation in PGS/CG, producing legitimately different
   solutions — tested separately in criterion 13.)
2. **Convergence speed:** Newton converges in ≤ 5 iterations on a stiff
   multi-contact scene where PGS requires ≥ 20 iterations.
3. **Stiff contacts:** Newton handles `solref=[0.002, 1.0]` (5× stiffer than
   default) without divergence on a sphere-on-plane benchmark.
4. **Elliptic cones:** Correct force recovery for condim 3, 4, and 6 contacts,
   verified against PGS solutions.
5. **Unified constraints:** Newton produces correct joint limit forces,
   equality constraint forces, and contact forces simultaneously on a model
   with all three active (e.g., a robot arm with joint limits, weld equalities,
   and floor contact).
6. **Fallback:** Non-convergence (max iterations exceeded or Cholesky failure)
   falls back to PGS using the existing contact-only system, matching the
   CG fallback pattern.
7. **MJCF default:** Loading a model with `solver="Newton"` (or no solver
   attribute, since Newton is MuJoCo's default) routes to the Newton solver,
   not PGS.
8. **Energy stability:** A 10-second free-fall + bounce simulation shows no
   energy gain (total energy monotonically decreases or stays constant within
   floating-point tolerance).
9. **Warm start:** Enabling `qacc_warmstart` reduces average iteration count
   by ≥ 20% on a multi-step simulation vs cold start.
10. **No regression:** PGS and CG solver paths produce identical results to
    before this change (unified assembly is Newton-only; existing contact-only
    path is untouched for PGS/CG).
11. **Zero-constraint degenerate:** A model with no contacts, limits, or
    equality constraints produces `qacc = qacc_smooth` (unconstrained solution)
    with 0 Newton iterations.
12. **Direct mode solref:** A contact with `solref=[-500, -10]` (direct
    stiffness/damping) produces the same constraint forces as standard mode
    `solref` with equivalent K/B values (within `1e-6` tolerance).
13. **Friction loss rows:** A model with `frictionloss > 0` on joints produces
    correct Huber-cost friction forces through the unified constraint system,
    matching the expected saturation behavior (linear outside `±R·floss`).
14. **Condim 1 contacts:** Normal-only contacts (`condim=1`) work correctly
    through the `ContactNonElliptic` path — pure unilateral constraint with
    no friction channels.
15. **Multi-row equality:** A weld equality constraint (6 rows) produces
    correct translational and rotational forces through the unified system.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`SolverType::Newton`,
  `ConstraintType`, `ConstraintState`,
  `assemble_unified_constraints()`, `classify_constraint_states()`,
  `newton_solve()`, `primal_line_search()`, `cholesky_rank1_update()`,
  `hessian_cone()`, dispatch in `mj_fwd_constraint()` and `forward()`,
  `qacc_warmstart` save, new Model fields, new Data fields)
- `sim/L0/mjcf/src/model_builder.rs` — modify (Newton → Newton mapping,
  `noslip_*` field parsing)
- `sim/L0/mjcf/src/parser.rs` — modify (parse `noslip_iterations`,
  `noslip_tolerance`)
- `sim/L0/mjcf/src/types.rs` — no change (Newton variant already exists)
- `sim/L0/tests/integration/newton_solver.rs` — new (acceptance tests)
- `sim/docs/MUJOCO_REFERENCE.md` — update (§4.4 add Newton solver docs)
- `sim/docs/MUJOCO_GAP_ANALYSIS.md` — update (§2 mark Newton as Implemented)
- `sim/docs/todo/future_work_5.md` — update status

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
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
No sleep/deactivation system. Every body is simulated every step regardless of
whether it is stationary. MuJoCo deactivates bodies whose velocity is below a
threshold for a configurable duration, skipping their dynamics until an external
force or contact wakes them.

#### Objective
Bodies at rest are automatically deactivated, reducing computation for scenes with
many stationary objects.

#### Specification

Per-body sleep state:

```rust
pub body_sleep_time: Vec<f64>,   // time at zero velocity (in Model or Data)
pub body_asleep: Vec<bool>,      // deactivation flag
```

In `Data::step()`, after integration:
1. For each body, if `|v| < sleep_threshold` for `sleep_duration` steps, set
   `body_asleep = true`.
2. Asleep bodies skip FK, force computation, and integration.
3. Wake on: external force applied, contact with awake body, `ctrl` change on
   attached actuator.

#### Acceptance Criteria
1. Stationary bodies deactivate after configurable duration.
2. Contact with an active body wakes sleeping bodies.
3. Scene with 100 resting bodies and 1 active body runs faster than all-active.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (sleep tracking, skip logic)

---

### 17. SOR Relaxation for PGS
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
PGS uses plain Gauss-Seidel (relaxation factor omega = 1.0). MuJoCo's PGS uses SOR
with configurable omega for accelerated convergence.

#### Objective
Add SOR relaxation parameter to PGS solver.

#### Specification

In `pgs_solve_with_system()` (`mujoco_pipeline.rs:8049`), after the GS update:

```rust
lambda_new = (1 - omega) * lambda_old + omega * lambda_gs;
```

where `omega` is read from `Model.solver_sor` (default 1.0, parsed from MJCF
`<option sor="..."/>`).

#### Acceptance Criteria
1. omega = 1.0 matches current behavior (regression).
2. omega = 1.3 converges in fewer iterations for a stiff contact benchmark.
3. omega < 1.0 (under-relaxation) is stable for pathological cases.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`pgs_solve_with_system()`,
  Model field)

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
| Sleeping bodies in deformable | Depends on #16. (#11 deformable pipeline ✅ complete) |
| Sparse mass matrix (deeper MuJoCo parity) | Phase 1 #1/#2 cover the main path. Full sparse pipeline is diminishing returns. |
| MuJoCo conformance test suite | Important but orthogonal to features — can be built incrementally. Without this, acceptance criteria for items #1–#17 rely on ad-hoc verification rather than systematic comparison against MuJoCo reference outputs. Consider bootstrapping a minimal conformance harness (load model, step N times, compare state vectors against MuJoCo ground truth) as infrastructure that benefits all items. |
| SIMD utilization (unused batch ops) | sim-simd exists; utilization will come naturally with #9/#10. |
| Tendon equality constraints | Standalone in sim-constraint. Pipeline tendons work; equality coupling is rare. Runtime warning at `mujoco_pipeline.rs:8493` fires when models include them — these constraints are silently ignored. |

---

## Cross-Reference: Phase 1 Mapping

| Phase 1 # | Phase 2 # | Notes |
|-----------|-----------|-------|
| #9 (Deformable Body) | #11 | Transferred verbatim ✅ Complete |
| #10 (Batched Simulation) | #9 | Transferred verbatim |
| #11 (GPU Acceleration) | #10 | Transferred verbatim |
