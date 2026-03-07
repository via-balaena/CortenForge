# Spec A — Sparse Flex Edge Jacobian: Spec

**Status:** Draft
**Phase:** Roadmap Phase 10 — Flex Pipeline
**Effort:** L
**MuJoCo ref:** `mj_flex()` in `engine_core_smooth.c`, lines ~1290–1330;
`makeFlexSparse()` in `engine_setconst.c`; `mj_springdamper()` in
`engine_passive.c`, lines ~671–679; `mj_fwdVelocity()` in `engine_forward.c`;
`mj_instantiateEquality()` in `engine_core_constraint.c`
**MuJoCo version:** 3.5.0
**Test baseline:** 1,900+ sim domain tests
**Prerequisites:**
- T1 Session 2 (commit `1f0230c`): provides `flex_rigid`, `flexedge_rigid`,
  `flexedge_length`, `flexedge_velocity`
- None other — Spec A is independent of Spec B (bending) and Spec C/D (collision)

**Independence:** This spec is independent of Spec B (§42B cotangent bending)
and Spec C/D (§42A-iv/v collision) per the umbrella dependency graph. Shared
file `passive.rs` is modified by T1 (boolean gates, landed), Spec A (force
projection), and Spec B (bending rewrite) — but at different code sections.
Spec A modifies the edge spring-damper force application (lines ~541–564);
Spec B modifies the bending section (lines ~567–699). No conflict.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

Empirical verification against MuJoCo C source discovered scope corrections
from the umbrella and `future_work_10.md` claims.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "Each edge has 2 rows (one per endpoint vertex)" | Each edge has **ONE row** in `flexedge_J`. The row encodes `vec^T * jacdif` — the edge-direction-projected differential Jacobian. | **Correct in spec** — 1 row per edge, not 2. |
| "Three inline ±direction patterns: edge spring-damper, bending, Newton penalty" | Bending forces do NOT use `flexedge_J` — they apply per-vertex forces directly via `body_dofadr`/`body_dofnum` on a 4-vertex diamond stencil. No "Newton penalty" consumer exists as a separate `flexedge_J` consumer. | **Drop bending and Newton penalty as consumers.** Actual consumers: (1) edge spring-damper, (2) velocity computation, (3) equality constraint Jacobian. |
| Edge velocity as separate §42A-iii item | MuJoCo computes `flexedge_velocity = J * qvel` using the sparse Jacobian (`mju_mulMatVecSparse` in `mj_fwdVelocity`). T1 implemented inline velocity computation. Spec A migrates this to `J * qvel`. | **Add velocity migration** as consumer site 2. |
| Constraint Jacobian not mentioned as consumer | MuJoCo's `mj_instantiateEquality` (`engine_core_constraint.c`) uses `flexedge_J` rows directly as `efc_J` constraint Jacobian for `mjEQ_FLEX` constraints. Our `assembly.rs:340–349` currently uses inline `±direction`. | **Add constraint assembly** as consumer site 3. |

**Final scope:**

1. CSR sparse structure on Model (`flexedge_J_rownnz`, `flexedge_J_rowadr`, `flexedge_J_colind`)
2. Runtime Jacobian computation on Data (`flexedge_J`) — one row per edge, `vec^T * jacdif`
3. Consumer migration: edge spring-damper (`passive.rs`), velocity (`flex.rs`), constraint Jacobian (`assembly.rs`)
4. Free-vertex identity: sparse Jacobian degenerates to `[-vec^T, +vec^T]` on 6 DOFs, bit-identical to current inline
5. Body-attached vertex forward-compatibility: infrastructure ready, deferred to §27D

---

## Problem Statement

**Conformance gap** — MuJoCo pre-computes a sparse edge Jacobian
(`flexedge_J`) in `mj_flex()` (`engine_core_smooth.c`) and uses it for three
distinct operations: (1) edge spring-damper force application via `J^T * force`
in `mj_springdamper()`, (2) edge velocity computation via `J * qvel` in
`mj_fwdVelocity()`, and (3) edge equality constraint Jacobian assembly in
`mj_instantiateEquality()`. CortenForge does not have this sparse Jacobian
infrastructure. Instead, it computes `±direction` inline at each consumer
site, assuming each vertex maps to exactly 3 consecutive translational DOFs.

For free vertices (all current models), the inline `±direction` pattern
produces numerically identical results to the sparse Jacobian — both
degenerate to the same `[-vec^T, +vec^T]` row. However, this inline pattern
is structurally non-conformant: MuJoCo computes the Jacobian once and reads
it three times; CortenForge computes it inline three times. The sparse
Jacobian also provides the foundation for body-attached vertices (§27D),
where `±direction` would produce incorrect force projection.

This spec adds the CSR sparse Jacobian infrastructure (Model fields for
sparsity structure, Data field for values), rewrites J computation to use
`vec^T * jacdif`, and migrates all three consumer sites to use the pre-computed
Jacobian.

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream —
> the algorithm, the acceptance criteria, the tests — is derived from what's
> documented here.

### CSR Structure — `makeFlexSparse()` in `engine_setconst.c`

Each edge `e` has exactly **one row** in the sparse Jacobian matrix. The row
encodes the edge-direction-projected differential Jacobian between the two
endpoint bodies. CSR metadata is computed once at model compile time:

- `flexedge_J_rownnz[e]` — number of non-zero entries for edge `e`
- `flexedge_J_rowadr[e]` — start index in `colind`/`J` arrays for edge `e`
- `flexedge_J_colind[j]` — DOF column index for non-zero entry `j`

The sparsity pattern is determined by calling `mj_jacDifPair(m, d, chain,
b1, b2, pos1, pos2, ..., issparse=1)`, which returns the union of DOFs
affecting the relative motion between the two endpoint bodies.

**For free flex vertices** (each vertex is its own body with 3 slide joints):
- Body `b1` has `body_dofnum = 3`, DOFs at `body_dofadr[b1]`
- Body `b2` has `body_dofnum = 3`, DOFs at `body_dofadr[b2]`
- `rownnz = dofnum(b1) + dofnum(b2)` = 6 for two free vertices
- `colind` contains the 6 DOF indices: `[dofadr(b1), dofadr(b1)+1, dofadr(b1)+2, dofadr(b2), dofadr(b2)+1, dofadr(b2)+2]`

**For pinned vertices** (`dofadr = usize::MAX`, `body_dofnum = 0`):
- The pinned vertex contributes 0 DOFs to the row
- `rownnz = dofnum(non_pinned_body)` (only the other endpoint's DOFs)
- Example: pinned v0 → free v1: `rownnz = 3`, `colind = [dofadr(b1), dofadr(b1)+1, dofadr(b1)+2]`

**For both vertices pinned** (`flex_rigid` case):
- `rownnz = 0` — no DOFs, no Jacobian to compute
- Skipped by the `flex_rigid[f]` gate

**Allocation:**
```
rowadr[0] = 0
rowadr[e] = rowadr[e-1] + rownnz[e-1]   for e > 0
total_nnz = sum(rownnz[e] for e in 0..nflexedge)
colind.len() = total_nnz
flexedge_J.len() = total_nnz
```

**Empirical verification** (MuJoCo 3.5.0, 3-vertex cable with pin on v0):

```
Model: nflexvert=3, nflexedge=2, nv=6
  v0: body=worldbody, dofnum=0 (pinned)
  v1: body=1, dofadr=0, dofnum=3 (free)
  v2: body=2, dofadr=3, dofnum=3 (free)

CSR Structure:
  rownnz = [3, 6]
  rowadr = [0, 3]
  colind = [0, 1, 2, 0, 1, 2, 3, 4, 5]
  Total nnz = 9

Edge 0 (pinned v0 → free v1): rownnz=3 (only v1's 3 DOFs)
Edge 1 (free v1 → free v2):   rownnz=6 (both vertices' DOFs)
```

### Runtime Jacobian Computation — `mj_flex()` in `engine_core_smooth.c`

For each edge `e` with endpoints `v1`, `v2`:

```c
// 1. Compute edge direction + length
vec = pos2 - pos1;
flexedge_length[e] = mju_normalize3(vec);  // normalizes in-place, returns length

// 2. Get differential body Jacobian (3×NV)
NV = mj_jacDifPair(m, d, chain, b1, b2, pos1, pos2,
                    jac1, jac2, jacdif, NULL, NULL, NULL, 1);

// 3. Project along edge direction: J_edge = vec^T * jacdif (1×NV)
mju_mulMatTVec(flexedge_J + rowadr[e], jacdif, vec, 3, NV);
```

**Meaning:** `J_edge[j] = vec · jacdif[:,j]` — the dot product of the unit
edge direction with each column of the 3×NV differential Jacobian.

**For free vertices:**
- `jac(b1)` = 3×3 identity at v1's 3 translational DOFs
- `jac(b2)` = 3×3 identity at v2's 3 translational DOFs
- `jacdif = jac(b2) - jac(b1)` = 3×6 matrix: `[-I₃, +I₃]`
- `J_edge = vec^T · [-I₃, +I₃]` = `[-vec^T, +vec^T]` (1×6)
- This is exactly the `±direction` pattern our code currently uses inline

**For pinned v0 → free v1:**
- `jac(worldbody)` = 3×0 (no DOFs)
- `jacdif = jac(b1)` = 3×3 identity (only v1's DOFs)
- `J_edge = vec^T · I₃ = +vec^T` (1×3)
- Force on pinned vertex: no DOFs → no force contribution

**Empirical verification** (MuJoCo 3.5.0, same 3-vertex cable):

```
Edge 0 (pinned v0 → free v1): dir=[1,0,0]
  J = [+1, 0, 0]  (only v1 DOFs; jac(worldbody)=0)

Edge 1 (free v1 → free v2): dir=[1,0,0]
  J = [-1, 0, 0, +1, 0, 0]  ([-vec^T, +vec^T] confirmed)

J*qvel verification (qvel=[0.1, 0.2, -0.3, 0.5, -0.1, 0.4]):
  Edge 0: J*qvel = 0.1 = flexedge_velocity[0] ✓
  Edge 1: J*qvel = 0.4 = flexedge_velocity[1] ✓

J^T*force verification (v1 displaced +0.05 in x):
  Edge 0: len=0.55, frc=-5.0, J^T*(-5) adds [-5,0,0] to DOFs [0,1,2]
  Edge 1: len=0.45, frc=+5.0, J^T*(+5) adds [-5,0,0,+5,0,0] to DOFs [0..5]
  qfrc_spring = [-10, 0, 0, 5, 0, 0] ✓ (matches MuJoCo output exactly)
```

### Skip Conditions — `mj_flex()` in `engine_core_smooth.c`

J computation is skipped when:

1. `flex_rigid[f]` — entire flex is rigid (all vertices have invmass == 0)
2. `skipjacobian` — no consumer needs J:
   ```c
   skipjacobian = !flex_edgeequality[f] && !flex_edgedamping[f]
                  && !flex_edgestiffness[f] && !flex_damping[f];
   ```

**CortenForge adaptation:** MuJoCo's `flex_edgeequality[f]` flag indicates
whether any edge equality constraint references this flex. CortenForge does
not have a dedicated `flex_edgeequality` flag. Our adaptation: always compute
J when any edge has `flex_edge_solref[f] != [0,0]` (default solref indicates
edge constraints are active), or when `flex_edgestiffness[f] != 0.0` or
`flex_edgedamping[f] != 0.0` or `flex_bend_damping[f] != 0.0`. This is
conservative — it may compute J for some edges that don't need it, but never
misses an edge that does. The cost is negligible for free vertices (6 FMAs
per edge).

**Simplified skip condition for CortenForge:**

```rust
let skip_jacobian = model.flex_edgestiffness[f] == 0.0
    && model.flex_edgedamping[f] == 0.0
    && model.flex_bend_damping[f] == 0.0
    && model.flex_edge_solref[f] == [0.0, 0.0];
```

### Consumer Site 1 — Edge Spring-Damper (`engine_passive.c`)

```c
// J^T * force for edge spring-damper
int end = m->flexedge_J_rowadr[e] + m->flexedge_J_rownnz[e];
for (int j = m->flexedge_J_rowadr[e]; j < end; j++) {
    int colind = m->flexedge_J_colind[j];
    mjtNum J = d->flexedge_J[j];
    d->qfrc_spring[colind] += J * frc_spring;
    d->qfrc_damper[colind] += J * frc_damper;
}
```

**Current CortenForge:** `passive.rs:541–564` — applies `±direction * force`
to `dof0+ax` and `dof1+ax` (3 DOFs per vertex). This is the inline pattern
that Spec A replaces with the `J^T * force` loop above.

### Consumer Site 2 — Edge Velocity (`engine_forward.c`)

```c
mju_mulMatVecSparse(d->flexedge_velocity, d->flexedge_J, d->qvel,
                    m->nflexedge,
                    m->flexedge_J_rownnz, m->flexedge_J_rowadr,
                    m->flexedge_J_colind, NULL);
```

`flexedge_velocity[e] = J[e,:] · qvel` — sparse matrix-vector product.

**Current CortenForge:** `dynamics/flex.rs:27–57` (`mj_flex_edge()`) computes
velocity inline from `flexvert_xpos` and `qvel`. After Spec A, velocity is
computed as `velocity[e] = sum(J[j] * qvel[colind[j]])` using the sparse
Jacobian.

### Consumer Site 3 — Constraint Jacobian (`engine_core_constraint.c`)

```c
// Sparse mode: pass J row directly as constraint Jacobian
mj_addConstraint(m, d, d->flexedge_J + m->flexedge_J_rowadr[e],
                 cpos, 0, 0, 1, mjCNSTR_EQUALITY, i,
                 m->flexedge_J_rownnz[e],
                 m->flexedge_J_colind + m->flexedge_J_rowadr[e]);
```

**Current CortenForge:** `constraint/assembly.rs:340–349` — populates
`efc_J` inline using `±direction[k]` at `dof0+k` and `dof1+k`. After Spec A,
this scatters `flexedge_J` values at `colind` positions into the dense `efc_J`
row (our constraint assembly uses dense `efc_J`, not sparse).

### Bending Does NOT Use `flexedge_J`

**MuJoCo source:** `engine_passive.c` bending section (lines ~520–530).

Bending forces are applied directly per-vertex using `body_dofadr`/`body_dofnum`
on a 4-vertex diamond stencil:

```c
for (int i = 0; i < 4; i++) {
    int bid = bodyid[v[i]];
    int body_dofadr = m->body_dofadr[bid];
    for (int x = 0; x < body_dofnum; x++) {
        d->qfrc_spring[body_dofadr+x] -= spring[3*i+x];
    }
}
```

Bending operates on a 4-vertex diamond stencil, not on edge forces. The edge
Jacobian projects *scalar edge forces*, not 4-vertex stencil forces. **Bending
is NOT a consumer of `flexedge_J`.**

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Edge Jacobian structure | 1 row per edge, CSR format, sparsity from `mj_jacDifPair`. `engine_setconst.c:makeFlexSparse()` | No sparse Jacobian. Inline `±direction` at each consumer site. |
| J computation | `vec^T * jacdif` per edge in `mj_flex()`. `engine_core_smooth.c:~1290–1330` | Not implemented. Direction computed inline at each consumer. |
| Edge spring-damper force | `J^T * force` sparse loop. `engine_passive.c:~671–679` | Inline `±direction * force` to `dof0+ax`, `dof1+ax`. `passive.rs:541–564` |
| Edge velocity | `J * qvel` sparse mat-vec. `engine_forward.c:mj_fwdVelocity()` | Inline `(vel1-vel0)·direction`. `dynamics/flex.rs:27–57` |
| Constraint Jacobian | Scatter `flexedge_J` row into `efc_J`. `engine_core_constraint.c:mj_instantiateEquality()` | Inline `±direction[k]` to `efc_J[(row, dof+k)]`. `assembly.rs:340–349` |
| Rigid flex skip | `flex_rigid[f]` skips J computation | `flex_rigid[f]` skips passive force loop (T1). J computation not implemented. |
| Zero-length edge | `dist < epsilon → skip J` | `dist < 1e-10 → skip` in passive and velocity |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Edge vertex indexing | `flex_edge[2*(ebase+e)+0/1]` — flex-local, offset by edge base | `flexedge_vert[e] = [v0, v1]` — global vertex indices | Direct port — our `flexedge_vert[e]` gives global vertex indices directly; no base offset needed. |
| Vertex body ID | `flex_vertbodyid[vbase+v]` — flex-local offset | `flexvert_bodyid[v]` — global vertex index | Direct port — our arrays are globally indexed. |
| Vertex DOF address | `body_dofadr[bodyid]` + `body_dofnum[bodyid]` | `flexvert_dofadr[v]` — direct DOF address per vertex. `usize::MAX` for pinned. | Use `flexvert_dofadr[v]` directly. For `body_dofnum`, look up `model.body_dof_num[model.flexvert_bodyid[v]]`. |
| Pinned vertex sentinel | `body_dofnum == 0` (worldbody has dofnum=0) | `flexvert_dofadr[v] == usize::MAX` | Check `dofadr == usize::MAX` to identify pinned vertices. Both are equivalent: pinned vertices are parented to worldbody which has `body_dof_num = 0`. |
| Differential Jacobian | `mj_jacDifPair()` — sparse 3×NV differential Jacobian | No equivalent function. For free vertices: `jacdif = [-I₃, +I₃]` is trivially constructed. | For free vertices, construct `jacdif` inline: columns for v0 body get `-I₃`, columns for v1 body get `+I₃`. For body-attached (§27D future): implement `mj_jacDifPair()`. |
| J data layout | `flexedge_J[rowadr[e] + j]` — flat CSR data array | `data.flexedge_J[rowadr + j]` — same flat layout | Direct port — identical CSR data layout. |
| Force target arrays | `d->qfrc_spring`, `d->qfrc_damper` | `data.qfrc_spring`, `data.qfrc_damper` (DVector) | Direct port — index with `[colind]` instead of `[dof+ax]`. |
| Body DOF count | `body_dofnum[bodyid]` (int) | `model.body_dof_num[bodyid]` (usize) | Direct port — same semantics, different name. |
| Vector arithmetic | `mjtNum vec[3]`, pointer arithmetic `vec[k]` | `nalgebra::Vector3<f64>`, index via `vec[k]` | Direct port — nalgebra `Vector3` supports `[]` indexing with same semantics. No translation needed. |

---

## Architecture Decisions

### AD-1: Free-vertex inline `jacdif` vs generic `mj_jacDifPair()`

**Problem:** MuJoCo uses `mj_jacDifPair()` to compute the 3×NV differential
Jacobian for any pair of bodies. CortenForge does not have this function.
Should we implement a full `mj_jacDifPair()` or construct `jacdif` inline
for free vertices?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Implement full `mj_jacDifPair()` | Handles body-attached vertices immediately | Complex; needs kinematic chain traversal; no test cases for body-attached flex until §27D |
| 2 | Inline `jacdif` for free vertices only | Simple; correct for all current models; bit-identical to current | Does not handle body-attached vertices |

**Chosen:** Option 2 — For free vertices, `jacdif` is trivially `[-I₃, +I₃]`
(or a subset for pinned vertices). This is provably identical to
`mj_jacDifPair()` output for free 3-DOF bodies. Body-attached vertex support
(requiring full `mj_jacDifPair()`) is deferred to §27D, which is the feature
that creates body-attached flex vertices. The CSR infrastructure built here
is forward-compatible: when §27D adds `mj_jacDifPair()`, the J computation
changes but all consumers remain identical.

### AD-2: Velocity computation placement

**Problem:** MuJoCo computes `flexedge_velocity = J * qvel` in
`mj_fwdVelocity()` (velocity stage). T1 currently computes velocity inline
in `mj_flex_edge()` (position stage, after FK). Where should Spec A place
the J-based velocity computation?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Keep in `mj_flex_edge()` (position stage) | J and velocity computed together; single function; `qvel` is available at this point | Differs from MuJoCo's stage placement |
| 2 | Move to velocity stage | Matches MuJoCo's `mj_fwdVelocity()` placement | Splits J computation and velocity into different stages; adds complexity for no behavior difference |

**Chosen:** Option 1 — Keep velocity computation in `mj_flex_edge()` alongside
J computation. This differs from MuJoCo's placement in `mj_fwdVelocity()`
(velocity stage), but produces numerically identical results because `qvel`
is an input to the forward pipeline — it is available at both the position
and velocity stages and does not change between them. **Conformance impact:
none** — the `flexedge_velocity` values are identical regardless of which
stage computes them. Computing J and `velocity = J * qvel` in the same
function avoids splitting related logic across pipeline stages.

---

## Specification

### S1. CSR Structure on Model — Builder Allocation

**File:** `sim/L0/core/src/types/model.rs` (line ~450, after `flexedge_rigid`)
and `sim/L0/core/src/types/model_init.rs` (line ~193, after `flexedge_rigid`)
and `sim/L0/mjcf/src/builder/flex.rs` (line ~268, after edge extraction)
**MuJoCo equivalent:** `makeFlexSparse()` in `engine_setconst.c`
**Design decision:** CSR metadata is computed at build time because the
sparsity pattern depends only on topology (which bodies the edge endpoints
belong to), not on runtime state. This matches MuJoCo, where `makeFlexSparse()`
runs during model compilation.

**Model fields** (add after `flexedge_rigid`):

```rust
// --- Sparse edge Jacobian CSR structure (computed at build time) ---
/// Number of non-zero entries per edge Jacobian row. Length `nflexedge`.
/// For free vertices: rownnz = dofnum(b1) + dofnum(b2).
/// For pinned vertex: rownnz = dofnum(non_pinned_body).
/// For both pinned: rownnz = 0.
pub flexedge_J_rownnz: Vec<usize>,
/// Start index in colind/J arrays for each edge. Length `nflexedge`.
/// rowadr[0] = 0, rowadr[e] = rowadr[e-1] + rownnz[e-1].
pub flexedge_J_rowadr: Vec<usize>,
/// DOF column indices for non-zero entries. Length `total_nnz`.
/// Contains the DOF indices for each endpoint body, in order:
/// [dofs_of_body(v0), dofs_of_body(v1)].
pub flexedge_J_colind: Vec<usize>,
```

**Model init** (add after `flexedge_rigid: vec![]`):

```rust
flexedge_J_rownnz: vec![],
flexedge_J_rowadr: vec![],
flexedge_J_colind: vec![],
```

**Builder allocation** (in `process_flex_bodies()`, after edge extraction
loop at line ~268, before hinge extraction):

```rust
// Allocate CSR structure for sparse edge Jacobian.
// Sparsity pattern: union of DOFs from the two endpoint bodies.
let nedge = self.nflexedge;
let mut rownnz = Vec::with_capacity(nedge);
let mut rowadr = Vec::with_capacity(nedge);
let mut colind = Vec::new();
let mut adr: usize = 0;

for e in 0..nedge {
    let [v0, v1] = self.flexedge_vert[e];
    let b0 = self.flexvert_bodyid[v0];
    let b1 = self.flexvert_bodyid[v1];
    let dn0 = self.body_dof_num[b0];
    let dn1 = self.body_dof_num[b1];
    let nnz = dn0 + dn1;

    rownnz.push(nnz);
    rowadr.push(adr);

    // Append DOF column indices: body0's DOFs first, then body1's DOFs
    let da0 = self.body_dof_adr[b0];
    for k in 0..dn0 {
        colind.push(da0 + k);
    }
    let da1 = self.body_dof_adr[b1];
    for k in 0..dn1 {
        colind.push(da1 + k);
    }

    adr += nnz;
}

self.flexedge_J_rownnz = rownnz;
self.flexedge_J_rowadr = rowadr;
self.flexedge_J_colind = colind;
```

**Note on pinned vertices:** Pinned vertices are parented to worldbody, which
has `body_dof_num[0] = 0`. The loop naturally produces `dn0 = 0` for pinned
v0, contributing zero column indices — no special-case code needed.

### S2. Runtime Jacobian Computation on Data — `mj_flex_edge()` Rewrite

**File:** `sim/L0/core/src/types/data.rs` (line ~130, after `flexedge_velocity`)
and `sim/L0/core/src/types/model_init.rs` (line ~491, after `flexedge_velocity`)
and `sim/L0/core/src/dynamics/flex.rs` (lines 21–58, `mj_flex_edge()` rewrite)
**MuJoCo equivalent:** `mj_flex()` in `engine_core_smooth.c`, lines ~1290–1330
**Design decision:** J computation and length/velocity are computed together
in `mj_flex_edge()` because they share the edge direction normalization step
(AD-2). The direction `vec` is computed once and used for both `flexedge_length`
(its magnitude) and `flexedge_J` (the projection `vec^T * jacdif`).

**Data field** (add after `flexedge_velocity`):

```rust
/// Sparse edge Jacobian values (CSR data array).
/// Length `total_nnz` (sum of `model.flexedge_J_rownnz`).
/// `flexedge_J[rowadr[e] + j]` is the Jacobian value for edge `e`,
/// column `colind[rowadr[e] + j]`.
pub flexedge_J: Vec<f64>,
```

**Data init** (add after `flexedge_velocity`):

```rust
flexedge_J: vec![0.0; model.flexedge_J_colind.len()],
```

**Data Clone** (add after `flexedge_velocity`):

```rust
flexedge_J: self.flexedge_J.clone(),
```

**Data reset** (add after `flexedge_velocity.fill(0.0)`):

```rust
self.flexedge_J.fill(0.0);
```

**`mj_flex_edge()` rewrite** — replaces the current inline velocity
computation with J computation + `velocity = J * qvel`:

```rust
/// Pre-compute sparse edge Jacobian (`flexedge_J`), edge lengths
/// (`flexedge_length`), and edge velocities (`flexedge_velocity`).
///
/// Called after `mj_flex()` populates `flexvert_xpos`, before
/// `mj_fwd_passive()`. Length and J share the direction normalization
/// step (no double computation).
///
/// MuJoCo ref: `mj_flex()` in `engine_core_smooth.c` (J computation),
/// `mj_fwdVelocity()` in `engine_forward.c` (velocity = J * qvel).
pub fn mj_flex_edge(model: &Model, data: &mut Data) {
    for f in 0..model.nflex {
        // Per-flex skip conditions (evaluated once per flex, not per edge)
        let is_rigid = model.flex_rigid[f];
        let skip_jacobian = is_rigid
            || (model.flex_edgestiffness[f] == 0.0
                && model.flex_edgedamping[f] == 0.0
                && model.flex_bend_damping[f] == 0.0
                && model.flex_edge_solref[f] == [0.0, 0.0]);

        // Iterate over edges belonging to this flex
        let edge_start = model.flex_edgeadr[f];
        let edge_count = model.flex_edgenum[f];

        for e in edge_start..edge_start + edge_count {
            let [v0, v1] = model.flexedge_vert[e];
            let diff = data.flexvert_xpos[v1] - data.flexvert_xpos[v0];
            let dist = diff.norm();

            // Always compute length — even for rigid flex (MuJoCo does).
            data.flexedge_length[e] = dist;

            if dist < 1e-10 {
                data.flexedge_velocity[e] = 0.0;
                // J values remain zero (from reset)
                continue;
            }

            // Rigid flex: length computed above, skip J + velocity
            if is_rigid {
                continue;
            }

            let vec = diff / dist; // unit edge direction

            if skip_jacobian {
                // Still compute velocity inline (cheaper than J path)
                let dof0 = model.flexvert_dofadr[v0];
                let dof1 = model.flexvert_dofadr[v1];
                let vel0 = if dof0 == usize::MAX {
                    Vector3::zeros()
                } else {
                    Vector3::new(data.qvel[dof0], data.qvel[dof0 + 1], data.qvel[dof0 + 2])
                };
                let vel1 = if dof1 == usize::MAX {
                    Vector3::zeros()
                } else {
                    Vector3::new(data.qvel[dof1], data.qvel[dof1 + 1], data.qvel[dof1 + 2])
                };
                data.flexedge_velocity[e] = (vel1 - vel0).dot(&vec);
                continue;
            }

            // Compute J_edge = vec^T * jacdif
            // For free vertices: jacdif = [-I₃, +I₃]
            // J_edge = [-vec^T, +vec^T]
            let rowadr = model.flexedge_J_rowadr[e];
            let rownnz = model.flexedge_J_rownnz[e];
            let b0 = model.flexvert_bodyid[v0];
            let b1 = model.flexvert_bodyid[v1];
            let dn0 = model.body_dof_num[b0];
            let dn1 = model.body_dof_num[b1];

            // Body 0 contribution: -vec^T (for free vertex, 3 entries)
            for k in 0..dn0 {
                data.flexedge_J[rowadr + k] = -vec[k];
            }
            // Body 1 contribution: +vec^T (for free vertex, 3 entries)
            for k in 0..dn1 {
                data.flexedge_J[rowadr + dn0 + k] = vec[k];
            }

            // Velocity = J * qvel (sparse dot product)
            let mut vel = 0.0;
            for j in 0..rownnz {
                let col = model.flexedge_J_colind[rowadr + j];
                vel += data.flexedge_J[rowadr + j] * data.qvel[col];
            }
            data.flexedge_velocity[e] = vel;
        }
    }
}
```

**Note:** The J computation `[-vec^T, +vec^T]` for free vertices produces
exactly the same values as the current inline `direction` computation.
The velocity `J * qvel` is algebraically identical to `(vel1 - vel0) · vec`
for free vertices:

```
J * qvel = sum(-vec[k] * qvel[dof0+k]) + sum(+vec[k] * qvel[dof1+k])
         = -vec · vel0 + vec · vel1
         = (vel1 - vel0) · vec  ✓
```

**Per-flex iteration:** This rewrite iterates edges per-flex using the
existing `flex_edgeadr[f]` and `flex_edgenum[f]` Model fields (populated at
build time in `build.rs:48`). This enables per-flex skip conditions
(`flex_rigid[f]`, `skip_jacobian`) to be evaluated once per flex rather than
per edge.

### S3. Consumer Migration — Edge Spring-Damper (`passive.rs`)

**File:** `sim/L0/core/src/forward/passive.rs` (lines ~541–564)
**MuJoCo equivalent:** `mj_springdamper()` in `engine_passive.c`, lines ~671–679
**Design decision:** Replace the inline `±direction * force` application with
a unified `J^T * force` sparse loop. The loop reads from `flexedge_J`,
`flexedge_J_rowadr`, `flexedge_J_rownnz`, and `flexedge_J_colind`. This is
a single code path that handles both free and (future) body-attached vertices.

**Before** (current code, `passive.rs:541–564`):

```rust
if frc_spring != 0.0 {
    if dof0 < model.nv {
        for ax in 0..3 {
            data.qfrc_spring[dof0 + ax] -= direction[ax] * frc_spring;
        }
    }
    if dof1 < model.nv {
        for ax in 0..3 {
            data.qfrc_spring[dof1 + ax] += direction[ax] * frc_spring;
        }
    }
}
if frc_damper != 0.0 {
    if dof0 < model.nv {
        for ax in 0..3 {
            data.qfrc_damper[dof0 + ax] -= direction[ax] * frc_damper;
        }
    }
    if dof1 < model.nv {
        for ax in 0..3 {
            data.qfrc_damper[dof1 + ax] += direction[ax] * frc_damper;
        }
    }
}
```

**After** (sparse J^T * force):

```rust
// J^T * force — unified sparse loop replacing inline ±direction.
let rowadr = model.flexedge_J_rowadr[e];
let rownnz = model.flexedge_J_rownnz[e];
for j in 0..rownnz {
    let col = model.flexedge_J_colind[rowadr + j];
    let jval = data.flexedge_J[rowadr + j];
    if frc_spring != 0.0 {
        data.qfrc_spring[col] += jval * frc_spring;
    }
    if frc_damper != 0.0 {
        data.qfrc_damper[col] += jval * frc_damper;
    }
}
```

**Numerical identity proof for free vertices:**

The J row for edge `e` between free v0 and free v1 is `[-vec^T, +vec^T]`.
The `colind` entries are `[dof0, dof0+1, dof0+2, dof1, dof1+1, dof1+2]`.

```
J^T * frc_spring:
  col=dof0+0: J=-vec[0] → qfrc_spring[dof0+0] += -vec[0] * frc_spring
  col=dof0+1: J=-vec[1] → qfrc_spring[dof0+1] += -vec[1] * frc_spring
  col=dof0+2: J=-vec[2] → qfrc_spring[dof0+2] += -vec[2] * frc_spring
  col=dof1+0: J=+vec[0] → qfrc_spring[dof1+0] += +vec[0] * frc_spring
  col=dof1+1: J=+vec[1] → qfrc_spring[dof1+1] += +vec[1] * frc_spring
  col=dof1+2: J=+vec[2] → qfrc_spring[dof1+2] += +vec[2] * frc_spring
```

This is exactly the same as the old code: `qfrc_spring[dof0+ax] -=
direction[ax] * frc_spring` and `qfrc_spring[dof1+ax] += direction[ax] *
frc_spring`. The signs match because `J[0..3] = -vec` and the new code
uses `+=` (not `-=`).

**Variables no longer needed:** `dof0`, `dof1`, `direction` can be removed
from the edge spring-damper section. The `direction` variable is currently
computed at line ~520 (`let direction = diff / dist`). After S3, `direction`
is only used for force computation magnitude (via `rest_len - dist`), not for
force application. The `diff` and `dist` variables remain needed for the
force magnitude calculation.

**Cleanup:** Remove the `dof0` and `dof1` lookups at lines ~517–518 from
the edge spring-damper section. The `direction` computation at line ~520 is
also no longer needed — the edge direction is already embedded in `flexedge_J`.
However, `diff / dist` is not used elsewhere in this section (force magnitude
uses `dist` and `rest_len`, not `direction`), so it can be removed.

### S4. Consumer Migration — Velocity (`flex.rs`)

**File:** `sim/L0/core/src/dynamics/flex.rs` (lines 27–57, `mj_flex_edge()`)
**MuJoCo equivalent:** `mj_fwdVelocity()` in `engine_forward.c`
**Design decision:** Velocity computation is absorbed into the S2 rewrite of
`mj_flex_edge()`. The current inline velocity computation (lines 44–56) is
replaced by `velocity[e] = J * qvel` using the sparse Jacobian values
computed in the same function. This is not a separate code change — it is
part of the S2 rewrite. Documented as a separate S-section for traceability
against the rubric's P9 (consumer migration completeness).

**Before** (current `mj_flex_edge()`, lines 44–56):

```rust
let dof0 = model.flexvert_dofadr[v0];
let dof1 = model.flexvert_dofadr[v1];
let vel0 = if dof0 == usize::MAX {
    Vector3::zeros()
} else {
    Vector3::new(data.qvel[dof0], data.qvel[dof0 + 1], data.qvel[dof0 + 2])
};
let vel1 = if dof1 == usize::MAX {
    Vector3::zeros()
} else {
    Vector3::new(data.qvel[dof1], data.qvel[dof1 + 1], data.qvel[dof1 + 2])
};
data.flexedge_velocity[e] = (vel1 - vel0).dot(&direction);
```

**After** (within S2's `mj_flex_edge()` rewrite):

```rust
// Velocity = J * qvel (sparse dot product)
let mut vel = 0.0;
for j in 0..rownnz {
    let col = model.flexedge_J_colind[rowadr + j];
    vel += data.flexedge_J[rowadr + j] * data.qvel[col];
}
data.flexedge_velocity[e] = vel;
```

**Numerical identity:** proven in S2's note — `J * qvel = (vel1 - vel0) · vec`
for free vertices.

### S5. Consumer Migration — Constraint Jacobian (`assembly.rs`)

**File:** `sim/L0/core/src/constraint/assembly.rs` (lines ~336–349)
**MuJoCo equivalent:** `mj_instantiateEquality()` in `engine_core_constraint.c`
**Design decision:** Our constraint assembly uses a dense `efc_J` matrix
(DMatrix), not sparse. The migration scatters `flexedge_J` values at `colind`
positions into the dense `efc_J` row. This replaces the inline `±direction[k]`
pattern. MuJoCo passes the sparse J row directly to `mj_addConstraint()`; we
scatter into dense because our assembly infrastructure is dense.

**Before** (current `assembly.rs:336–349`):

```rust
// Jacobian: ∂C/∂x_v0 = -direction, ∂C/∂x_v1 = +direction
// (§27F) Pinned vertices (dofadr=usize::MAX) have zero Jacobian columns.
let dof0 = model.flexvert_dofadr[v0];
let dof1 = model.flexvert_dofadr[v1];
if dof0 != usize::MAX {
    for k in 0..3 {
        data.efc_J[(row, dof0 + k)] = -direction[k];
    }
}
if dof1 != usize::MAX {
    for k in 0..3 {
        data.efc_J[(row, dof1 + k)] = direction[k];
    }
}
```

**After** (scatter from sparse J):

```rust
// Scatter pre-computed edge Jacobian into dense efc_J row.
// Replaces inline ±direction computation.
let rowadr = model.flexedge_J_rowadr[e];
let rownnz = model.flexedge_J_rownnz[e];
for j in 0..rownnz {
    let col = model.flexedge_J_colind[rowadr + j];
    data.efc_J[(row, col)] = data.flexedge_J[rowadr + j];
}
```

**Numerical identity:** The scattered values are `[-vec^T, +vec^T]` at the
same DOF positions as the old code's `-direction[k]` at `dof0+k` and
`+direction[k]` at `dof1+k`. Bit-identical for free vertices.

**Note on `direction` variable:** The `direction` variable computed at line
~333 (`let direction = diff / dist`) is still needed for `vel_error`
computation at line ~362 (`(vel1 - vel0).dot(&direction)`). After S5, the
Jacobian scatter replaces the `±direction` usage for `efc_J`, but `direction`
remains used for `vel_error`. Alternatively, `vel_error` can be read from
`data.flexedge_velocity[e]` (pre-computed by S2). This is a secondary cleanup
within S5:

```rust
// Velocity error: read from pre-computed Data field instead of inline
let vel_error = data.flexedge_velocity[e];
```

This eliminates the `vel0`, `vel1` inline velocity computation (lines 352–362)
and the remaining `direction` usage, making the constraint section fully
dependent on pre-computed Data fields.

---

## Acceptance Criteria

### AC1: CSR structure correctness — 3-vertex cable with pin *(runtime test)*
**Given:** 3-vertex cable flex (v0 pinned to worldbody, v1 and v2 free),
`edge_stiffness = 100.0`
**After:** Model build
**Assert:** `flexedge_J_rownnz = [3, 6]`, `flexedge_J_rowadr = [0, 3]`,
`flexedge_J_colind = [dof(v1), dof(v1)+1, dof(v1)+2, dof(v1), dof(v1)+1, dof(v1)+2, dof(v2), dof(v2)+1, dof(v2)+2]`
(total nnz = 9)
**Field:** `Model.flexedge_J_rownnz`, `Model.flexedge_J_rowadr`, `Model.flexedge_J_colind`

### AC2: CSR structure — all-free 4-vertex square shell *(runtime test)*
**Given:** 4-vertex square shell flex (all vertices free, 5 edges:
4 boundary + 1 diagonal), `edge_stiffness = 100.0`
**After:** Model build
**Assert:** Every edge has `rownnz = 6` (each edge between two 3-DOF free
vertices). `total_nnz = 5 * 6 = 30`.
**Field:** `Model.flexedge_J_rownnz`, `Model.flexedge_J_colind.len()`

### AC3: Free-vertex Jacobian identity *(runtime test)*
**Given:** 2-vertex cable flex (both free), initial edge direction = `[1, 0, 0]`,
`edge_stiffness = 100.0`
**After:** `mj_flex_edge()` (position stage)
**Assert:** `flexedge_J = [-1, 0, 0, +1, 0, 0]` (1×6 row = `[-vec^T, +vec^T]`)
**Field:** `Data.flexedge_J`

### AC4: Free-vertex simulation regression — bit-identical *(runtime test)*
**Given:** Any flex model from the existing test suite (cable with gravity,
shell with bending, etc.)
**After:** 10 simulation steps
**Assert:** `qfrc_spring`, `qfrc_damper`, `qpos`, `qvel` values after Spec A
are bit-identical to pre-Spec-A values (zero tolerance). This confirms the
sparse J path produces identical results to inline `±direction` for free
vertices.
**Field:** `Data.qfrc_spring`, `Data.qfrc_damper`, `Data.qpos`, `Data.qvel`

### AC5: Velocity via J*qvel matches inline *(runtime test)*
**Given:** 3-vertex cable under gravity, `edge_stiffness = 100.0`, after 5
simulation steps (vertices have non-zero velocity)
**After:** `mj_flex_edge()`
**Assert:** `flexedge_velocity[e]` values are bit-identical to values computed
by the old inline method `(vel1 - vel0) · direction` for all edges.
**Field:** `Data.flexedge_velocity`

### AC6: Constraint Jacobian migration *(runtime test)*
**Given:** 3-vertex cable with edge constraints (`solref = [-100, -10]`),
v0 pinned
**After:** Constraint assembly
**Assert:** `efc_J` row for edge `e` has non-zero values at exactly the
`colind` positions from `flexedge_J_colind`, and the values match
`flexedge_J` at those positions. All other `efc_J` columns are zero.
**Field:** `Data.efc_J`

### AC7: Pinned vertex — zero J contribution *(runtime test)*
**Given:** 3-vertex cable, v0 pinned (dofadr = usize::MAX)
**After:** `mj_flex_edge()`
**Assert:** Edge 0 (pinned v0 → free v1) has `rownnz = 3` and J values
correspond to `+vec^T` only (no v0 contribution). `J^T * force` applies
force only to v1's DOFs.
**Field:** `Data.flexedge_J`, `Data.qfrc_spring`

### AC8: Rigid flex skip *(runtime test)*
**Given:** Flex with all vertices pinned (`flex_rigid[f] = true`), vertices
at spatially separated positions
**After:** `mj_flex_edge()`
**Assert:** All `flexedge_J` values for this flex remain zero (from reset).
`flexedge_velocity` is 0.0 for all edges. `flexedge_length` IS computed
(non-zero for separated vertices). `flexedge_J_rownnz` values are all 0.
**Field:** `Data.flexedge_J`, `Data.flexedge_velocity`, `Data.flexedge_length`

### AC9: Zero-length edge skip *(runtime test)*
**Given:** Two flex vertices at identical positions (dist < 1e-10)
**After:** `mj_flex_edge()`
**Assert:** `flexedge_J` values for this edge remain zero. `flexedge_velocity`
is 0.0. `flexedge_length` is 0.0 (or near-zero). No division by zero.
**Field:** `Data.flexedge_J`, `Data.flexedge_velocity`, `Data.flexedge_length`

### AC10: No orphan inline ±direction sites *(code review)*
After Spec A, no flex code in `sim/L0/core/src/` contains inline `±direction`
force application patterns (`direction[ax] * force` applied to `dof0+ax` /
`dof1+ax`). All force/Jacobian application uses the sparse `flexedge_J`.
Verify with these grep patterns (all must return zero matches in the
specified scope):
- `grep -n 'direction\[ax\].*frc\|direction\[k\].*efc_J' passive.rs assembly.rs`
- `grep -n 'dof0 + ax\|dof1 + ax\|dof0 + k\|dof1 + k' passive.rs assembly.rs dynamics/flex.rs`
  Scope: edge spring-damper section of `passive.rs`, flex edge constraint
  section of `assembly.rs`, and the entire `dynamics/flex.rs` (fully
  rewritten by S2). The bending section of `passive.rs` is NOT modified by
  Spec A and may still use per-vertex DOF patterns — exclude it from scan.

### AC11: Multi-flex model *(runtime test)*
**Given:** Model with two flex objects: a 2-vertex cable and a 4-vertex shell,
different `edge_stiffness` values
**After:** `mj_flex_edge()` + `mj_fwd_passive()`
**Assert:** CSR structure has correct `rowadr` offsets spanning both flexes.
Each flex's edges have independent J values. `qfrc_spring` contributions
are correct for both.
**Field:** `Model.flexedge_J_rowadr`, `Data.flexedge_J`, `Data.qfrc_spring`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (CSR pinned cable) | T1 | Direct |
| AC2 (CSR all-free shell) | T2 | Direct |
| AC3 (free-vertex J identity) | T3 | Direct |
| AC4 (simulation regression) | T4 | Regression |
| AC5 (velocity J*qvel) | T5 | Direct |
| AC6 (constraint J migration) | T6 | Direct |
| AC7 (pinned vertex zero J) | T7 | Edge case |
| AC8 (rigid flex skip) | T8 | Edge case |
| AC9 (zero-length edge skip) | T9 | Edge case |
| AC10 (no orphan inline sites) | — | Code review (manual) |
| AC11 (multi-flex model) | T10 | Integration |

---

## Test Plan

### T1: CSR structure — pinned cable → AC1
3-vertex cable: v0 pinned, v1 free, v2 free. Build model. Assert
`flexedge_J_rownnz = [3, 6]`, `flexedge_J_rowadr = [0, 3]`,
`flexedge_J_colind` contains v1's 3 DOFs for edge 0 and v1+v2's 6 DOFs
for edge 1. Total colind length = 9.

### T2: CSR structure — all-free shell → AC2
4-vertex square shell (2 triangles, 5 edges). All vertices free. Build model.
Assert every `rownnz = 6`. Total colind length = 30.

### T3: Free-vertex Jacobian values → AC3
2-vertex cable, both free, initial positions at (0,0,0) and (1,0,0).
Edge direction = `[1,0,0]`. After `mj_flex_edge()`, assert
`flexedge_J = [-1, 0, 0, +1, 0, 0]` (exact, zero tolerance).

MuJoCo-verified: `[-vec^T, +vec^T]` confirmed in EGT-2 empirical data
(MuJoCo 3.5.0).

### T4: Simulation regression — bit-identical → AC4
Use existing flex cable test model (e.g., from `flex_unified.rs`). Run 10
steps pre-Spec-A (capture `qpos`, `qvel`, `qfrc_spring`, `qfrc_damper`).
Run same model post-Spec-A. Assert all values are bit-identical (exact f64
equality, zero tolerance). This is the cardinal regression test: if the sparse
J path does not produce bit-identical results for free vertices, Spec A is
wrong.

**Implementation note:** This test can be structured as a comparison of
`qfrc_spring` and `qfrc_damper` values against expected constants computed
from a known model configuration, rather than literally running pre/post
comparisons. The expected values are the same because the math is identical.

### T5: Velocity J*qvel → AC5
3-vertex cable under gravity, `edge_stiffness = 100.0`. Run 5 steps so
vertices acquire velocity. After `mj_flex_edge()`, compute expected velocity
inline: `expected = (vel1 - vel0) · direction`. Assert
`flexedge_velocity[e] == expected` (exact, zero tolerance) for all edges.

### T6: Constraint Jacobian scatter → AC6
3-vertex cable with edge constraints (`solref = [-100, -10]`),
`edge_stiffness = 0.0`, `edge_damping = 0.0`, v0 pinned. The zero
stiffness/damping verifies the negative case: J is computed solely for the
constraint consumer path (skip_jacobian is false because `flex_edge_solref
!= [0, 0]`). After constraint assembly, inspect `efc_J` row for each edge.
For edge 0 (pinned v0 → free v1): non-zero values only at v1's 3 DOFs,
matching `flexedge_J[rowadr[0]..rowadr[0]+3]`. For edge 1: non-zero at
v1+v2's 6 DOFs, matching `flexedge_J[rowadr[1]..rowadr[1]+6]`.

### T7: Pinned vertex — zero contribution → AC7
3-vertex cable, v0 pinned. After `mj_flex_edge()`, edge 0 has `rownnz = 3`.
`flexedge_J` values for edge 0 are `[+vec[0], +vec[1], +vec[2]]` (no v0
contribution). Apply spring force: `qfrc_spring` has non-zero values only at
v1's DOFs for edge 0's contribution.

### T8: Rigid flex skip → AC8
Flex with all 3 vertices pinned (`flex_rigid = true`). After
`mj_flex_edge()`, all `flexedge_J` values are 0.0. `flexedge_velocity` is
0.0 for all edges. `flexedge_length` values are computed (length doesn't
depend on J).

**Note:** For rigid flex, length IS still computed (MuJoCo computes
`flexedge_length` even for rigid flex — the skip only applies to J and
velocity). Verify that length is non-zero for edges with spatially separated
vertices.

### T9: Zero-length edge → AC9
Two vertices at position (0,0,0) and (0,0,0) (coincident). After
`mj_flex_edge()`, `flexedge_length[e] ≈ 0.0`, `flexedge_velocity[e] = 0.0`,
`flexedge_J` values are 0.0 for this edge. No panic or NaN.

### T10: Multi-flex model → AC11
Model with two flex objects: cable (2 vertices, 1 edge, stiffness=50) and
shell (4 vertices, 5 edges, stiffness=200). After build: CSR `rowadr`
correctly offsets the shell's edges past the cable's edge. After
`mj_flex_edge()` + `mj_fwd_passive()`: J values are independent per-flex,
`qfrc_spring` accumulates contributions from both flexes correctly.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Pinned vertex (dofadr=usize::MAX) | rownnz must exclude pinned body's DOFs; J must not write to invalid indices | T1, T7 | AC1, AC7 |
| Fully rigid flex (all pinned) | J computation must be skipped entirely | T8 | AC8 |
| Zero-length edge (dist < 1e-10) | Must skip J computation without NaN/panic from division by zero | T9 | AC9 |
| Mixed rigid/non-rigid edges | Within one flex, some edges rigid (both endpoints pinned), others not | T1 | AC1 |
| Single-vertex flex (no edges) | nflexedge=0 for this flex; all CSR arrays empty for this flex, no out-of-bounds access in mj_flex_edge() | S1 | AC2, AC8 |
| Both vertices on same body | §27D-only; impossible for current free-vertex models. rownnz = dofnum(body). Noted for forward-compat but untestable until §27D. | — | — |
| Disabled spring+damper, active constraint | J must still be computed when stiffness=0 and damping=0 but edge constraints are active (solref != [0,0]) | T6 | AC6 |
| Multi-flex CSR offsets | rowadr must span across flex boundaries correctly | T10 | AC11 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| S1: nflexedge=0 model | Model with flex that has 1 vertex and 0 edges | Verifies CSR arrays are empty, no out-of-bounds access in mj_flex_edge() |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Edge spring force application | Inline `±direction * force` to `dof0+ax`, `dof1+ax` | Sparse `J^T * force` to `colind[j]` positions | Toward MuJoCo | `passive.rs` edge loop consumers | None — transparent change (bit-identical for free vertices) |
| Edge velocity computation | Inline `(vel1 - vel0) · direction` | Sparse `J * qvel` | Toward MuJoCo | `dynamics/flex.rs` | None — transparent change (bit-identical for free vertices) |
| Constraint Jacobian assembly | Inline `±direction[k]` to `efc_J[(row, dof+k)]` | Scatter `flexedge_J[j]` to `efc_J[(row, colind[j])]` | Toward MuJoCo | `assembly.rs` flex edge section | None — transparent change (bit-identical for free vertices) |
| Constraint velocity error | Inline `(vel1-vel0)·direction` in assembly | Read from `data.flexedge_velocity[e]` | Toward MuJoCo | `assembly.rs` flex edge section | None — transparent change |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/types/model.rs` | Add 3 CSR fields (`flexedge_J_rownnz`, `flexedge_J_rowadr`, `flexedge_J_colind`) | +10 |
| `sim/L0/core/src/types/model_init.rs` | Init CSR fields to empty vecs | +3 |
| `sim/L0/core/src/types/data.rs` | Add `flexedge_J: Vec<f64>`, add to Clone impl, add to reset | +6 |
| `sim/L0/core/src/dynamics/flex.rs` | Rewrite `mj_flex_edge()`: J computation + `velocity = J * qvel` | ~60 modified |
| `sim/L0/core/src/forward/passive.rs` | Replace inline `±direction` force application with sparse `J^T * force` | ~24 modified → ~8 |
| `sim/L0/core/src/constraint/assembly.rs` | Replace inline `±direction` Jacobian + inline velocity with sparse scatter + Data read | ~20 modified → ~8 |
| `sim/L0/mjcf/src/builder/flex.rs` | Allocate CSR structure after edge extraction | +25 |
| `sim/L0/tests/integration/flex_unified.rs` | New tests: T1–T10 + S1 | +200–250 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| All 43 `flex_unified.rs` tests | `sim/L0/tests/integration/flex_unified.rs` | **Pass (unchanged)** | Sparse J^T*force produces bit-identical results to inline ±direction for free vertices. No expected values change. |
| Flex conformance tests | `sim-conformance-tests` | **Pass (unchanged)** | Same reason — bit-identical results. |
| Non-flex tests | All other test files | **Pass (unchanged)** | No code changes outside flex subsystem. |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `passive.rs:567–699` | Bending force section (dihedral angle spring-damper) | Bending does NOT use `flexedge_J` (EGT-7). Bending uses 4-vertex diamond stencil with per-vertex body DOF application. Modified by Spec B, not Spec A. |
| `dynamics/flex.rs:14–19` | `mj_flex()` — vertex xpos sync from body FK | Not modified — this populates `flexvert_xpos` which is an input to `mj_flex_edge()`. No change needed. |
| `forward/mod.rs:311–312` | Pipeline call sites for `mj_flex()` and `mj_flex_edge()` | Not modified — same functions, same call order. Signature unchanged. |

---

## Execution Order

1. **S1 first** (CSR structure on Model + builder) — establishes the sparsity
   pattern that S2 reads. Cannot compute J values without knowing `rowadr`,
   `rownnz`, `colind`. → Run tests: verify CSR structure for pinned cable
   (T1) and all-free shell (T2).

2. **S2 after S1** (J computation + velocity migration in `mj_flex_edge()`)
   — fills `flexedge_J` and migrates velocity. Requires S1's CSR fields
   on Model and `flexedge_J` Data field. → Run tests: verify J values (T3),
   velocity (T5), rigid skip (T8), zero-length (T9).

3. **S3 after S2** (passive.rs consumer migration) — reads `flexedge_J`
   computed by S2. → Run tests: full regression (T4), pinned vertex force
   (T7), multi-flex (T10). All existing flex tests must pass.

4. **S4 is part of S2** (velocity migration) — no separate implementation
   step. Documented as separate section for traceability.

5. **S5 after S2** (assembly.rs consumer migration) — reads `flexedge_J`
   computed by S2. Independent of S3 — can be done before or after S3.
   → Run tests: constraint Jacobian scatter (T6).

After each section: `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests`

---

## Out of Scope

- **Body-attached flex vertices** (§27D) — The CSR infrastructure built here
  is partially forward-compatible. Consumers (S3/S4/S5) are fully
  forward-compatible — they read from CSR without assumptions about body
  structure. However, S1's CSR allocation assumes each vertex has a unique
  body (`rownnz = dn0 + dn1` without deduplication). For same-body vertices
  (possible under §27D), MuJoCo's `mj_jacDifPair()` produces
  `rownnz = dof_count(body)` (DOFs appear once), not `2 * dof_count`. S1's
  allocation and S2's J computation must both be updated when §27D lands:
  S1 must detect same-body edges and deduplicate DOF columns; S2 must use
  `mj_jacDifPair()` instead of inline `[-vec^T, +vec^T]`.
  Tracked as §27D. *Conformance impact: affects body-attached flex models
  only. Not a gap for v1.0 (all current models use free vertices, where each
  vertex has its own body).*

- **Full `mj_jacDifPair()` implementation** — Required only for §27D.
  The inline free-vertex J computation is proven identical. Tracked as §27D
  dependency. *Conformance impact: none for free-vertex models.*

- **Sparse constraint assembly** — MuJoCo's `mj_addConstraint()` accepts
  sparse J rows natively. Our assembly uses dense `efc_J`. Converting to
  sparse assembly is an optimization, not a conformance gap. Tracked as
  post-v1.0. *Conformance impact: none — dense scatter produces identical
  `efc_J` values.*

- **GPU flex pipeline** (DT-67) — Post-v1.0. *Conformance impact: none.*

- **`flex_edgeequality` flag** — MuJoCo has a per-flex flag indicating whether
  any equality constraint references this flex's edges. CortenForge uses
  `flex_edge_solref != [0,0]` as a conservative proxy. Adding a dedicated
  flag is deferred. *Conformance impact: may compute J for edges that don't
  need it — conservative, never incorrect.*
