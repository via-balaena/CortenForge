# Future Work 6B — Flex Solver Unification: Deformable DOFs in Unified Constraint Pipeline

**Precursor to [future_work_7.md](./future_work_7.md) (Items #24–27).**

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

This item unifies the deformable simulation pipeline with the rigid constraint
solver, matching MuJoCo's flex architecture. Flex vertex DOFs join the global
`qpos`/`qvel` state, edge constraints enter the unified Jacobian, bending acts
as passive forces in `mj_fwd_passive()`, and the separate XPBD solver is
removed. The `sim-deformable` crate is fully deprecated.

**Why this blocks #24–27:** The current dual code path (rigid contacts via
`make_contact_from_geoms()` + deformable contacts via
`mj_deformable_collision()`) means every contact parameter fix must be applied
in two places with different data structures. Unification creates a single
contact path, making friction combination (#24), priority (#25), solmix (#26),
and margin/gap (#27) each a single-site change.

**Subsumes:** Item #42 (`<flex>`/`<flexcomp>` MJCF parsing, `future_work_10.md`).

---

### 6B. Flex Solver Unification
**Status:** Implemented | **Effort:** XL | **Prerequisites:** None | **Blocks:** #24, #25, #26, #27

#### Current State

CortenForge has two completely separate simulation paths for rigid and
deformable bodies. They share no state vectors, no mass matrix, no constraint
solver, and no contact data structures.

##### Rigid Pipeline (what we keep)

| Component | Location | Role |
|-----------|----------|------|
| `Model.nq`, `Model.nv` | `mujoco_pipeline.rs:1147–1149` | Generalized coordinate dimensions (rigid joints only) |
| `Data.qpos/qvel/qacc` | `mujoco_pipeline.rs:2080–2084` | Global state vectors (rigid DOFs only) |
| `Data.qM` | `mujoco_pipeline.rs:2206` | Joint-space inertia matrix (nv × nv) |
| `Data.qLD_data/qLD_diag_inv` | `mujoco_pipeline.rs:2229–2234` | Sparse L^T D L factorization |
| `Data.contacts: Vec<Contact>` | `mujoco_pipeline.rs:2280` | Rigid-rigid contacts |
| `assemble_unified_constraints()` | `mujoco_pipeline.rs:14231` | Unified Jacobian assembly (equality, friction loss, limits, contacts) |
| `finalize_row!` macro | `mujoco_pipeline.rs:14336` | Per-row metadata: impedance, KBIP, aref, diagApprox, R, D |
| `mj_crba()` | `mujoco_pipeline.rs:10051` | Composite rigid body algorithm → M + LDL |
| `mj_factor_sparse()` | `mujoco_pipeline.rs:19680` | Sparse LDL factorization |
| `ConstraintType` enum | `mujoco_pipeline.rs:868` | {Equality, FrictionLoss, LimitJoint, LimitTendon, ContactNonElliptic, ContactElliptic, FlexEdge} |
| `extract_distance_jacobian()` | `mujoco_pipeline.rs:17555` | 1D distance equality Jacobian (pattern reuse for flex edges) |
| `add_body_point_jacobian_row()` | `mujoco_pipeline.rs:17624` | Kinematic chain Jacobian for body point (reuse for flex-rigid contacts) |

##### Deformable Pipeline (what gets replaced)

| Component | Location | Fate |
|-----------|----------|------|
| `DeformableBody` trait | `deformable/lib.rs:173` | **Deleted.** Replaced by Model `flex_*` arrays. |
| `XpbdSolver` | `deformable/solver.rs` | **Deleted.** Unified Newton/PGS/CG solver handles all constraints. |
| `Data.deformable_bodies` | `mujoco_pipeline.rs:2404` | **Deleted.** Flex state lives in `qpos`/`qvel`. |
| `Data.deformable_solvers` | `mujoco_pipeline.rs:2407` | **Deleted.** |
| `Data.deformable_contacts` | `mujoco_pipeline.rs:2410` | **Deleted.** Flex-rigid contacts are regular `Contact` entries. |
| `DeformableContact` struct | `mujoco_pipeline.rs:1786–1813` | **Deleted.** |
| `mj_deformable_collision()` | `mujoco_pipeline.rs:18935` | **Replaced** by vertex-vs-geom collision in unified `mj_collision()`. |
| `solve_deformable_contacts()` | `mujoco_pipeline.rs:18860` | **Deleted.** Contact impulses computed by unified solver. |
| `mj_deformable_step()` | `mujoco_pipeline.rs:19273` | **Deleted.** Edge constraints in `mj_fwd_constraint()`, bending in `mj_fwd_passive()`. |
| `deformable` feature flag | `sim-core/Cargo.toml:37` | **Deleted.** Flex is always available (zero-cost when nflex=0). |

##### sim-deformable Crate Migration

| Component | Current location | Destination |
|-----------|-----------------|-------------|
| Cloth grid generation | `deformable/cloth.rs` | `sim-mjcf/model_builder.rs` (`<flexcomp type="grid">` expansion) |
| SoftBody tetrahedralization | `deformable/soft_body.rs` | `sim-mjcf/model_builder.rs` (`<flexcomp type="box">` expansion) |
| CapsuleChain construction | `deformable/capsule_chain.rs` | `sim-mjcf/model_builder.rs` (1D flex construction) |
| Material presets | `deformable/material.rs` | `sim-mjcf/model_builder.rs` (default constants) |
| Constraint topology extraction | `deformable/constraints.rs` | `sim-mjcf/model_builder.rs` (build-time edge/hinge topology extraction) |
| Skinning (visual deformation) | `deformable/skinning.rs` | `sim-bevy` or standalone crate (rendering, not physics) |
| XPBD solver | `deformable/solver.rs` | **Deleted.** |
| Deformable mesh types | `deformable/mesh.rs` | `sim-core/mujoco_pipeline.rs` (Model fields) |

##### Key Architectural Divergence

| Aspect | MuJoCo | CortenForge (current) |
|--------|--------|----------------------|
| Vertex DOFs | Part of global `qpos`/`qvel` | Separate trait-object positions/velocities |
| Mass matrix | M includes vertex diagonal blocks | Separate per-body `inv_masses` |
| Edge constraints | Equality rows in unified Jacobian | XPBD position-based projection |
| Bending constraints | Equality rows in unified Jacobian | XPBD dihedral angle projection |
| Volume constraints | Equality rows in unified Jacobian | XPBD tetrahedron projection |
| Flex-rigid contacts | Regular `Contact` entries | Separate `DeformableContact` type |
| Solver | Unified Newton/PGS/CG (all constraints) | Split: rigid Newton/PGS/CG + separate XPBD |
| MJCF | `<flex>`/`<flexcomp>` parsed | Skipped (programmatic API only) |

#### MuJoCo Authoritative Semantics

These rules govern all design decisions. Every specification choice traces to
MuJoCo's flex implementation.

- **S1 — Flex DOFs are global.** Each flex vertex contributes 3 DOFs to `qpos`
  (position x,y,z) and 3 DOFs to `qvel` (velocity vx,vy,vz). Total DOFs:
  `nv = nv_rigid + 3 * nflexvert`. For flex vertices, `nq == nv` (no
  quaternions — positions are Euclidean).

- **S2 — Mass matrix is block-diagonal for flex.** Each vertex contributes a
  3×3 diagonal block `m_i * I_3` to the mass matrix M. No coupling between
  vertices in M (coupling arises from constraints, not inertia). Pinned
  vertices have `m = ∞` (equivalently: `inv_mass = 0`, which produces zero
  acceleration).

- **S3 — Edge-length constraints are soft equality constraints.** Each edge
  `(v_i, v_j)` with rest length `L_0` generates one constraint row:
  `C = ||x_i - x_j|| - L_0`. The constraint enters the unified Jacobian as an
  equality-type row with `solref`/`solimp` derived from material stiffness.
  This is NOT a hard constraint — the solver softens it via impedance,
  matching MuJoCo's treatment of flex edges as soft constraints with
  configurable `solref` (frequency + damping).

- **S4 — Flex contacts are regular contacts.** When a flex vertex contacts a
  rigid geom, the resulting `Contact` has the same structure as rigid-rigid
  contacts. The contact Jacobian has: (a) identity-like entries on the flex
  vertex DOF columns, (b) kinematic-chain entries on the rigid body DOF
  columns. The contact enters `assemble_unified_constraints()` alongside all
  other contacts. Friction, condim, solref, solimp all work identically.

- **S5 — Forward kinematics is trivial.** Flex vertex positions are directly
  read from `qpos` — no kinematic chain traversal needed. In MuJoCo:
  `xpos[vertex] = qpos[flex_qposadr[vertex] .. +3]`.

- **S6 — Integration is linear.** Flex vertex position integration is
  `qpos += qvel * dt` (semi-implicit Euler). No quaternion exponential map
  needed. Damping applied as passive forces before integration.

- **S7 — `<flex>` MJCF element.** MuJoCo parses `<flex>` inside `<deformable>`:
  `dim` (1=cable, 2=shell, 3=solid), `<vertex>` positions, `<element>`
  connectivity, material properties (`young`, `poisson`, `damping`,
  `thickness`), collision properties (`selfcollide`, `radius`, `margin`,
  `friction`, `condim`, `solref`, `solimp`). `<flexcomp>` is the procedural
  generator (`type="grid"`, `count`, `spacing`).

- **S8 — Constraint stiffness from material.** Edge constraint `solref` is
  derived from Young's modulus. For a rod-like element with cross-section A and
  length L: effective spring constant `k = E * A / L`. MuJoCo maps this to
  `solref[0] = 2π / sqrt(k / m_eff)` (natural period) and `solref[1]` from
  material damping. The exact mapping follows MuJoCo's
  `mj_setFlexSolref()`.

#### Objective

1. Extend `Model` with flex data arrays (vertices, edges, elements, materials).
2. Extend `Data.qpos/qvel/qacc` to include flex vertex DOFs.
3. Extend the mass matrix M and sparse LDL factorization for flex diagonal blocks.
4. Add flex edge constraints to `assemble_unified_constraints()`. Bending as passive forces in `mj_fwd_passive()`.
5. Unify flex-rigid contacts into the regular `Contact` pipeline.
6. Parse `<flex>` and `<flexcomp>` from MJCF.
7. Fully deprecate the `sim-deformable` crate (migrate useful code, delete rest).
8. Remove the `deformable` feature flag (flex is always available, zero-cost when empty).

#### What This Item Does NOT Cover

- **Flex-flex collision (self-collision).** Vertex-vertex and vertex-triangle
  self-collision requires BVH acceleration structures and is a separate item.
  The `selfcollide` attribute is parsed and stored but collision is not
  generated.
- **`<equality><flex>` constraints.** Flex-flex coupling via equality
  constraints is deferred.
- **GPU flex pipeline.** GPU acceleration of flex constraint solve is Phase 3E.
- **Per-vertex material variation.** All vertices in a flex share a single
  material. Per-element material variation is deferred.
- **Body-attached flex vertices.** The `flexvert_bodyid` field is parsed and
  stored, but vertices attached to rigid bodies (kinematic boundary conditions)
  are treated as pinned (`mass = 1e20`, `invmass = 0`) rather than tracking body motion. Full
  body-vertex coupling (vertex moves with body) is deferred.

#### Specification

##### P1. Model Data Extension

Add flex dimensions and data arrays to `Model` (`mujoco_pipeline.rs`,
after the existing geom arrays ~line 1360):

```rust
// ==================== Flex Bodies ====================
/// Number of flex objects.
pub nflex: usize,
/// Total flex vertices across all flex objects.
pub nflexvert: usize,
/// Total flex edges across all flex objects.
pub nflexedge: usize,
/// Total flex elements (triangles for dim=2, tetrahedra for dim=3).
pub nflexelem: usize,
/// Total bending hinges (pairs of adjacent elements sharing an edge).
pub nflexhinge: usize,

// --- Per-flex arrays (length nflex) ---
/// Dimensionality: 1 (cable), 2 (shell), 3 (solid).
pub flex_dim: Vec<usize>,
/// First vertex index in flexvert_* arrays.
pub flex_vertadr: Vec<usize>,
/// Number of vertices in this flex.
pub flex_vertnum: Vec<usize>,
/// First edge index in flexedge_* arrays.
pub flex_edgeadr: Vec<usize>,
/// Number of edges in this flex.
pub flex_edgenum: Vec<usize>,
/// First element index in flexelem_* arrays.
pub flex_elemadr: Vec<usize>,
/// Number of elements in this flex.
pub flex_elemnum: Vec<usize>,
/// Per-flex material: Young's modulus (Pa).
pub flex_young: Vec<f64>,
/// Per-flex material: Poisson's ratio (0–0.5).
pub flex_poisson: Vec<f64>,
/// Per-flex material: damping coefficient.
pub flex_damping: Vec<f64>,
/// Per-flex material: thickness (for dim=2 shells).
pub flex_thickness: Vec<f64>,
/// Per-flex: contact friction coefficient (scalar).
pub flex_friction: Vec<f64>,
/// Per-flex: contact solver reference [timeconst, dampratio].
pub flex_solref: Vec<[f64; 2]>,
/// Per-flex: contact solver impedance.
pub flex_solimp: Vec<[f64; 5]>,
/// Per-flex: contact condim (1, 3, 4, or 6).
pub flex_condim: Vec<i32>,
/// Per-flex: collision margin for broadphase expansion.
pub flex_margin: Vec<f64>,
/// Per-flex: self-collision enabled.
pub flex_selfcollide: Vec<bool>,
/// Per-flex: edge constraint solref (derived from young/poisson/damping).
pub flex_edge_solref: Vec<[f64; 2]>,
/// Per-flex: edge constraint solimp.
pub flex_edge_solimp: Vec<[f64; 5]>,
/// Per-flex: bending stiffness (passive spring, N·m/rad).
/// dim=2 (shell): D = E·t³ / (12·(1−ν²))  (Kirchhoff-Love plate theory).
/// dim=3 (solid): D = E  (characteristic stiffness; gradient provides geometry).
pub flex_bend_stiffness: Vec<f64>,
/// Per-flex: bending damping (passive damper, N·m·s/rad).
/// Proportional damping: b = damping × k_bend.
pub flex_bend_damping: Vec<f64>,
/// Per-flex: density. Units depend on dim:
/// dim=1 (cable): linear density kg/m.
/// dim=2 (shell): volumetric density kg/m³ (multiplied by thickness for area density).
/// dim=3 (solid): volumetric density kg/m³.
pub flex_density: Vec<f64>,

// --- Per-vertex arrays (length nflexvert) ---
/// Start index in qpos for this vertex (3 consecutive DOFs).
pub flexvert_qposadr: Vec<usize>,
/// Start index in qvel for this vertex (3 consecutive DOFs).
pub flexvert_dofadr: Vec<usize>,
/// Vertex mass (kg).
pub flexvert_mass: Vec<f64>,
/// Inverse mass (0 for pinned vertices).
pub flexvert_invmass: Vec<f64>,
/// Collision radius per vertex.
pub flexvert_radius: Vec<f64>,
/// Which flex object this vertex belongs to (for material lookup).
pub flexvert_flexid: Vec<usize>,
/// Optional rigid body attachment (usize::MAX = free vertex).
pub flexvert_bodyid: Vec<usize>,

// --- Per-edge arrays (length nflexedge) ---
/// Edge connectivity: vertex index pair [v0, v1].
pub flexedge_vert: Vec<[usize; 2]>,
/// Rest length of each edge.
pub flexedge_length0: Vec<f64>,
/// Effective cross-section area per edge (m²). Precomputed during build():
/// dim=1: π * radius², dim=2: thickness * dual_edge_len, dim=3: vol^{2/3} / L.
pub flexedge_crosssection: Vec<f64>,
/// Which flex object this edge belongs to.
pub flexedge_flexid: Vec<usize>,

// --- Per-element arrays (length nflexelem) ---
/// Element connectivity: vertex indices. Length varies by dim:
/// dim=1: 2 (edge), dim=2: 3 (triangle), dim=3: 4 (tetrahedron).
/// Stored as flat Vec<usize> with flexelem_dataadr/datanum for indexing.
pub flexelem_data: Vec<usize>,
/// Start index in flexelem_data for this element.
pub flexelem_dataadr: Vec<usize>,
/// Number of vertex indices per element (2, 3, or 4).
pub flexelem_datanum: Vec<usize>,
/// Rest volume of each element (for volume constraints, dim=3 only).
pub flexelem_volume0: Vec<f64>,
/// Which flex object this element belongs to.
pub flexelem_flexid: Vec<usize>,

// --- Per-hinge arrays (bending constraints, length nflexhinge) ---
// A hinge is a pair of adjacent elements sharing an edge.
// dim=2: two triangles sharing an edge → 4 distinct vertices.
// dim=3: two tetrahedra sharing a face → 5 distinct vertices (but bending
//         constraint acts on the 4 vertices of the shared face + 2 opposite,
//         simplified to the same 4-vertex dihedral formulation).
/// Hinge vertex indices: [v_e0, v_e1, v_opp_a, v_opp_b] where (v_e0, v_e1)
/// is the shared edge and v_opp_a, v_opp_b are opposite vertices.
pub flexhinge_vert: Vec<[usize; 4]>,
/// Rest dihedral angle (radians).
pub flexhinge_angle0: Vec<f64>,
/// Which flex object this hinge belongs to.
pub flexhinge_flexid: Vec<usize>,
```

**nq/nv extension** (in Model builder / `build()` function):

```rust
// After computing nq_rigid, nv_rigid from joints:
let nq_flex = 3 * nflexvert;
let nv_flex = 3 * nflexvert;
model.nq = nq_rigid + nq_flex;
model.nv = nv_rigid + nv_flex;
```

Store `nv_rigid` and `nq_rigid` for boundary between rigid and flex DOFs:

```rust
/// Number of rigid generalized position coordinates (before flex DOFs).
pub nq_rigid: usize,
/// Number of rigid velocity DOFs (before flex DOFs).
pub nv_rigid: usize,
```

**Default initialization** when `nflex == 0`: all flex arrays are empty, `nq == nq_rigid`, `nv == nv_rigid`. Zero runtime cost.

**Address table computation** (during `build()`):

```rust
let mut qpos_cursor = nq_rigid;
let mut dof_cursor = nv_rigid;
for i in 0..nflexvert {
    model.flexvert_qposadr[i] = qpos_cursor;
    model.flexvert_dofadr[i] = dof_cursor;
    qpos_cursor += 3;
    dof_cursor += 3;
}
```

##### P2. Data Extension

**Grow global state vectors** (`make_data()`, `mujoco_pipeline.rs:3117+`):

```rust
data.qpos = DVector::zeros(model.nq);   // now includes flex positions
data.qvel = DVector::zeros(model.nv);   // now includes flex velocities
data.qacc = DVector::zeros(model.nv);   // now includes flex accelerations
data.qacc_warmstart = DVector::zeros(model.nv);
data.qfrc_applied = DVector::zeros(model.nv);
data.qfrc_bias = DVector::zeros(model.nv);
data.qfrc_passive = DVector::zeros(model.nv);
data.qfrc_constraint = DVector::zeros(model.nv);
```

**Initialize flex vertex positions** from Model vertex data:

```rust
for i in 0..model.nflexvert {
    let adr = model.flexvert_qposadr[i];
    data.qpos[adr]     = initial_positions[i].x;
    data.qpos[adr + 1] = initial_positions[i].y;
    data.qpos[adr + 2] = initial_positions[i].z;
}
```

**Mass matrix** — `qM` grows to `(model.nv × model.nv)`. Existing allocation
already uses `model.nv` for dimensions; once `nv` includes flex DOFs, qM
automatically grows. The CRBA fills rigid DOFs; Phase P4 fills flex diagonal.

**Sparse LDL storage** — extend `qLD_rowadr`, `qLD_rownnz`, `qLD_colind` for
flex DOFs. Each flex DOF is diagonal-only (no ancestors in kinematic tree):
`nnz_per_flex_dof = 1` (just the diagonal). Total additional: `3 * nflexvert`
entries in `qLD_data`.

**Add flex pose array:**

```rust
/// World-frame flex vertex positions (copied from qpos each step).
pub flexvert_xpos: Vec<Vector3<f64>>,
```

Allocated as `vec![Vector3::zeros(); model.nflexvert]`.

**Remove deformable fields:**

```rust
// DELETE these three fields:
// pub deformable_bodies: Vec<Box<dyn DeformableBody + Send + Sync>>,
// pub deformable_solvers: Vec<XpbdSolver>,
// pub deformable_contacts: Vec<DeformableContact>,
```

Also remove the `DeformableContact` struct definition (lines 1786–1813).

##### P3. Forward Kinematics for Flex Vertices

Add `mj_fwd_position_flex()` immediately after `mj_fwd_position()` in
`forward_core()` (`mujoco_pipeline.rs:4498+`):

```rust
/// Copy flex vertex positions from qpos to flexvert_xpos.
/// Trivial O(nflexvert) — flex vertices ARE their positions.
fn mj_fwd_position_flex(model: &Model, data: &mut Data) {
    for i in 0..model.nflexvert {
        let adr = model.flexvert_qposadr[i];
        data.flexvert_xpos[i] = Vector3::new(
            data.qpos[adr],
            data.qpos[adr + 1],
            data.qpos[adr + 2],
        );
    }
}
```

No-op when `nflexvert == 0`.

##### P4. Mass Matrix Extension (CRBA + LDL)

After the existing `mj_crba()` computes M for rigid DOFs (`mujoco_pipeline.rs:
10051+`), append diagonal entries for flex vertex masses:

```rust
/// Extend mass matrix M with flex vertex diagonal blocks.
/// Each vertex contributes m_i * I_3 (3 diagonal entries).
fn mj_crba_flex(model: &Model, data: &mut Data) {
    for i in 0..model.nflexvert {
        let mass = model.flexvert_mass[i];
        let dof_base = model.flexvert_dofadr[i];
        for k in 0..3 {
            let dof = dof_base + k;
            data.qM[(dof, dof)] = mass;
        }
    }
}
```

**Sparse LDL extension** — flex DOFs are diagonal-only (no off-diagonal
ancestors). After `mj_factor_sparse()` processes rigid DOFs, append flex
diagonals:

```rust
/// Extend sparse LDL factorization with flex diagonal entries.
/// For diagonal-only DOFs: L = I, D = M, so D_inv = 1/M.
fn mj_factor_flex(model: &Model, data: &mut Data) {
    for i in 0..model.nflexvert {
        let mass = model.flexvert_mass[i];
        let dof_base = model.flexvert_dofadr[i];
        for k in 0..3 {
            let dof = dof_base + k;
            // Diagonal entry in qLD_data
            let ld_adr = model.qLD_rowadr[dof]; // points to diagonal
            data.qLD_data[ld_adr] = mass;
            // Precomputed inverse
            data.qLD_diag_inv[dof] = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        }
    }
    // Mark factorization as valid (flex portion updated)
    data.qLD_valid = true;
}
```

**Pinned vertices:** `mass = 1e20` (large finite, not INFINITY to avoid IEEE
NaN from `INF * 0` in `Ma = M * qacc`) → `inv_mass = 0.0` → `qacc = 0`
(vertex does not move). `qLD_diag_inv = 0.0` produces zero acceleration from
any force, matching MuJoCo's behavior for welded vertices.

**Sparse structure for flex DOFs** — during `build()`, append to LDL sparsity:

```rust
for i in 0..nflexvert {
    let dof_base = flexvert_dofadr[i];
    for k in 0..3 {
        let dof = dof_base + k;
        model.qLD_rowadr[dof] = ld_nnz_cursor;
        model.qLD_rownnz[dof] = 1; // diagonal only
        model.qLD_colind.push(dof); // self-referencing diagonal
        ld_nnz_cursor += 1;
    }
}
model.qLD_nnz = ld_nnz_cursor;
```

##### P5. Collision Detection Unification

Replace `mj_deformable_collision()` with flex vertex collision via
`mj_collision_flex()` (`mujoco_pipeline.rs:5586`), called from outside the
`if ngeom >= 2` SAP guard in `mj_collision()`.

**Approach:** Brute-force O(V×G) broadphase for simplicity. Each flex vertex is
tested against every rigid geom. SAP integration deferred to when nflexvert is
large enough to warrant it (MuJoCo uses BVH midphase for this, which is a
separate optimization item).

```rust
/// Detect contacts between flex vertices and rigid geoms.
/// Uses brute-force broadphase (O(V*G)) for simplicity — SAP integration
/// deferred to when nflexvert is large enough to warrant it.
/// No-op when nflexvert == 0.
fn mj_collision_flex(model: &Model, data: &mut Data) {
    if model.nflexvert == 0 {
        return;
    }

    for vi in 0..model.nflexvert {
        // Skip pinned vertices (invmass == 0, immovable)
        if model.flexvert_invmass[vi] <= 0.0 {
            continue;
        }

        let vpos = data.flexvert_xpos[vi];
        let radius = model.flexvert_radius[vi];
        let margin = model.flex_margin[model.flexvert_flexid[vi]];

        for gi in 0..model.ngeom {
            if let Some((depth, normal, contact_pos)) =
                narrowphase_sphere_geom(vpos, radius + margin, gi, model, ...)
            {
                let contact = make_contact_flex_rigid(model, vi, gi, contact_pos, normal, depth);
                data.contacts.push(contact);
                data.ncon += 1;
            }
        }
    }
}
```

**Integration with `mj_collision()`:** Called **outside** the `if ngeom >= 2`
SAP guard, since flex collision does not use SAP:

```rust
fn mj_collision(model: &Model, data: &mut Data) {
    if model.ngeom >= 2 {
        // ... existing SAP broadphase + rigid-rigid narrowphase ...
    }
    // Flex-rigid collision (independent of SAP, brute-force O(V*G))
    mj_collision_flex(model, data);
}
```

**`make_contact_flex_rigid()`** — creates a `Contact` using the standard
combination rules (element-wise max for friction, max condim, combined
solref/solimp). Does NOT take `data` parameter (only needs Model):

```rust
fn make_contact_flex_rigid(
    model: &Model,
    vertex_idx: usize,
    geom_idx: usize,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
) -> Contact {
    let flex_id = model.flexvert_flexid[vertex_idx];
    // ... friction combination, condim, solref/solimp ...

    Contact {
        pos, normal, depth,
        // For flex contacts, geom1 = geom2 = the rigid geom index.
        // The vertex index is stored exclusively in flex_vertex.
        // This ensures model.geom_body[contact.geom1] is always valid
        // (flex vertices have no geom index).
        geom1: geom_idx,
        geom2: geom_idx,
        flex_vertex: Some(vertex_idx),
        // ...
    }
}
```

**Contact type flag** — a field on `Contact` discriminates flex-rigid from
rigid-rigid contacts:

```rust
/// If Some(vertex_idx), this is a flex-rigid contact.
/// `geom1` and `geom2` both point to the rigid geom (so
/// `model.geom_body[contact.geom1]` is always valid).
/// The flex vertex index is stored here, not in geom1/geom2.
/// If None, this is a standard rigid-rigid contact.
pub flex_vertex: Option<usize>,
```

Default: `None`. Set during `mj_collision_flex()`.

---

**P5a. Contact Jacobian for flex-rigid contacts.**

The contact Jacobian assembly (`compute_contact_jacobian`, called from
`assemble_unified_constraints()`) must distinguish flex-rigid contacts from
rigid-rigid contacts via `contact.flex_vertex`:

For a flex vertex `v` contacting rigid geom attached to body `b`:
```
J[row, flexvert_dofadr[v] + 0..3] = ±frame_direction  (vertex side: trivial)
J[row, body_b_chain_dofs]         = ∓frame_direction   (rigid side: kinematic chain)
```

The vertex side has no kinematic chain — the Jacobian is just the contact frame
direction projected onto the vertex's 3 DOF columns. The rigid side uses the
existing `add_body_point_jacobian_row()` pattern.

---

**P5b. Contact velocity and force helpers for PGS/CG solver.**

The PGS/CG contact solver computes relative velocity at contact points for the
Delassus RHS. Three helper functions handle flex-rigid contacts:

```rust
/// Read flex vertex velocity directly from qvel (translational only).
fn compute_flex_vertex_velocity(model: &Model, data: &Data, vertex_idx: usize) -> Vector3<f64> {
    let dof_base = model.flexvert_dofadr[vertex_idx];
    Vector3::new(data.qvel[dof_base], data.qvel[dof_base + 1], data.qvel[dof_base + 2])
}

/// Relative velocity at a contact, handling both rigid-rigid and flex-rigid.
/// For flex-rigid: vel_vertex - vel_rigid (vertex is "body2", rigid is "body1").
fn compute_contact_relative_velocity(model, data, contact) -> Vector3<f64> {
    if let Some(vertex_idx) = contact.flex_vertex {
        let v_vertex = compute_flex_vertex_velocity(model, data, vertex_idx);
        let rigid_body = model.geom_body[contact.geom1];
        let v_rigid = compute_point_velocity(data, rigid_body, contact.pos);
        v_vertex - v_rigid
    } else {
        // Standard rigid-rigid
        vel2 - vel1
    }
}

/// Relative angular velocity: flex vertices have no angular DOFs.
/// For flex-rigid: 0 - omega_rigid = -omega_rigid.
fn compute_contact_relative_angular_velocity(model, data, contact) -> Vector3<f64> {
    if contact.flex_vertex.is_some() {
        let rigid_body = model.geom_body[contact.geom1];
        -compute_body_angular_velocity(data, rigid_body)
    } else {
        omega2 - omega1
    }
}
```

---

**P5c. Contact force application for flex-rigid contacts.**

After the PGS/CG solver produces contact impulses (lambda), forces are applied
back to bodies/vertices via `apply_solved_contact_lambda()`:

```rust
fn apply_solved_contact_lambda(model, data, contact, lambda) {
    let world_force = normal * lambda[0] + tangent1 * lambda[1] + tangent2 * lambda[2];

    if let Some(vertex_idx) = contact.flex_vertex {
        // Flex-rigid: vertex gets +force directly on DOFs, rigid gets -force via chain.
        let dof_base = model.flexvert_dofadr[vertex_idx];
        data.qfrc_constraint[dof_base + 0..3] += world_force;

        let rigid_body = model.geom_body[contact.geom1];
        apply_contact_force(model, data, rigid_body, contact.pos, -world_force);

        // Flex vertices have no angular DOFs — rigid body absorbs all torques.
        if dim >= 4 { apply_contact_torque(rigid_body, -torsional_torque); }
        if dim >= 6 { apply_contact_torque(rigid_body, -rolling_torque); }
    } else {
        // Standard rigid-rigid: both bodies via kinematic chain
    }
}
```

**Narrowphase functions** — reuse existing `collide_*` implementations.
The vertex acts as a small sphere of radius `flexvert_radius[vi]`. Supports
Plane, Sphere, Box, Capsule, Cylinder, and Ellipsoid geom types.

##### P6. Edge-Length Constraints in Unified Jacobian

Add flex edge constraints to `assemble_unified_constraints()` in the row
counting (Phase 1) and population (Phase 3) sections.

**New `ConstraintType` variant:**

```rust
/// Flex edge-length constraint (soft equality).
FlexEdge,
```

**Row counting** (insert after contact counting, `mujoco_pipeline.rs:14304`):

```rust
// Flex edge-length constraints
for e in 0..model.nflexedge {
    nefc += 1; // 1 row per edge
}
```

**Jacobian assembly** (insert after contact assembly):

```rust
// --- 3g: Flex edge-length constraints ---
for e in 0..model.nflexedge {
    let [v0, v1] = model.flexedge_vert[e];
    let x0 = data.flexvert_xpos[v0];
    let x1 = data.flexvert_xpos[v1];
    let diff = x1 - x0;
    let dist = diff.norm();
    let rest_len = model.flexedge_length0[e];

    if dist < 1e-10 {
        // Degenerate: zero-length edge, skip
        // (fill with zeros, finalize_row! handles it)
        finalize_row!(
            model.flex_edge_solref[model.flexedge_flexid[e]],
            model.flex_edge_solimp[model.flexedge_flexid[e]],
            0.0, 0.0, 0.0, 0.0,
            ConstraintType::FlexEdge, 1, e, [0.0; 5]
        );
        continue;
    }

    let direction = diff / dist;
    let pos_error = dist - rest_len; // positive = stretched, negative = compressed

    // Jacobian: ∂C/∂x_v0 = -direction, ∂C/∂x_v1 = +direction
    let dof0 = model.flexvert_dofadr[v0];
    let dof1 = model.flexvert_dofadr[v1];
    for k in 0..3 {
        data.efc_J[(row, dof0 + k)] = -direction[k];
        data.efc_J[(row, dof1 + k)] = direction[k];
    }

    // Velocity: relative velocity projected onto edge direction
    let vel0 = Vector3::new(
        data.qvel[dof0], data.qvel[dof0 + 1], data.qvel[dof0 + 2],
    );
    let vel1 = Vector3::new(
        data.qvel[dof1], data.qvel[dof1 + 1], data.qvel[dof1 + 2],
    );
    let vel_error = (vel1 - vel0).dot(&direction);

    let flex_id = model.flexedge_flexid[e];
    finalize_row!(
        model.flex_edge_solref[flex_id],
        model.flex_edge_solimp[flex_id],
        pos_error,
        0.0,       // margin
        vel_error,
        0.0,       // friction loss
        ConstraintType::FlexEdge,
        1,         // dim
        e,         // id (edge index)
        [0.0; 5]   // mu (no friction on edge constraints)
    );
}
```

**Penalty path** — for PGS/CG solvers (non-Newton), edge constraints are
applied via penalty-based Baumgarte stabilization in `apply_flex_edge_constraints()`
(`mujoco_pipeline.rs:14576`). This is the explicit-integration counterpart of
the unified Jacobian assembly above:

```rust
/// Apply flex edge-length constraint forces in the penalty path.
/// F = -k * pos_error - b * vel_error, applied via J^T to qfrc_constraint.
fn apply_flex_edge_constraints(model: &Model, data: &mut Data) {
    for e in 0..model.nflexedge {
        // ... same pos_error/vel_error computation as Jacobian path ...
        let (k, b) = solref_to_penalty(model.flex_edge_solref[flex_id], ...);
        let force_mag = -k * pos_error - b * vel_error;
        // Apply: F_v0 = -direction * force_mag, F_v1 = +direction * force_mag
        // Writes to qfrc_constraint (not qfrc_passive).
        // Skips pinned vertices (invmass <= 0).
    }
    // NOTE: Bending forces are in mj_fwd_passive() (passive spring-damper).
    // Volume constraints removed — MuJoCo has no dedicated volume mechanism.
}
```

Called from both PGS and Newton penalty paths (before the iterative solve).

---

**Constraint stiffness derivation** — during `build()`, compute edge solref.

**Correction (from #27C):** The original framing of material-derived per-edge
solref was incorrect. MuJoCo does not derive edge constraint solref from
Young's modulus. Instead: (1) edge constraints use eq_solref from the parent
equality constraint, and (2) `<edge stiffness="..." damping="..."/>` drives
passive spring-damper forces in `engine_passive.c`. See
`future_work_7.md` #27C for details. The function has been renamed from
`compute_edge_solref_from_material()` to `compute_edge_solref()` and is now
a documented passthrough (using flex-level solref), not a stub awaiting
material-derived implementation.

##### P7. Bending and Volume

**MuJoCo architecture reference** (from `engine_passive.c` and `engine_forward.c`):

| Mechanism | MuJoCo | CortenForge | Pipeline stage |
|---|---|---|---|
| Edge-length | Soft equality constraint in solver (`mjEQ_FLEX`) | `FlexEdge` in `ConstraintType` | `assemble_unified_constraints()` |
| Bending (dihedral) | Passive spring-damper force in `qfrc_spring` | Passive force in `mj_fwd_passive()` | Before constraint solve |
| Volume preservation | No dedicated mechanism (emergent from SVK) | Not implemented | N/A |

Bending is **not** a constraint row. MuJoCo computes bending as explicit passive
forces that contribute to `qacc_smooth`, influencing the solver indirectly via
the RHS. Volume preservation has no dedicated mechanism in MuJoCo — it is an
emergent property of continuum elasticity (Saint-Venant-Kirchhoff), which is
out of scope until §7.

---

**P7a. Bending passive forces** (dim=2 shells, dim=3 solids):

For each hinge (pair of adjacent elements sharing an edge), compute a
spring-damper force on the dihedral angle, applied via the transpose Jacobian
to `qfrc_passive`. This matches MuJoCo's `engine_passive.c` architecture.

**Model fields:**

```rust
/// Per-flex: bending stiffness (passive spring, N·m/rad).
/// dim=2 (shell): D = E·t³ / (12·(1−ν²))  (Kirchhoff-Love plate theory).
/// dim=3 (solid): D = E  (characteristic stiffness; gradient provides geometry).
pub flex_bend_stiffness: Vec<f64>,
/// Per-flex: bending damping (passive damper, N·m·s/rad).
/// Proportional damping: b = damping × k_bend.
pub flex_bend_damping: Vec<f64>,
```

**Force computation** (in `mj_fwd_passive()`, after flex vertex damping):

```rust
// Flex bending passive forces: spring-damper on dihedral angle (Bridson et al. 2003).
// MuJoCo computes bending as passive forces in engine_passive.c, NOT as constraint rows.
// Force = -k_bend * (theta - theta0) - b_bend * d(theta)/dt, applied via J^T.
let dt = model.timestep;

for h in 0..model.nflexhinge {
    let [ve0, ve1, va, vb] = model.flexhinge_vert[h];
    let flex_id = model.flexhinge_flexid[h];
    let k_bend_raw = model.flex_bend_stiffness[flex_id];
    let b_bend = model.flex_bend_damping[flex_id];
    if k_bend_raw <= 0.0 && b_bend <= 0.0 { continue; }

    // Dihedral angle via atan2 (Bridson et al. 2003 gradient)
    // theta = atan2(sin_theta, cos_theta)
    // angle_error = theta - rest_angle
    // theta_dot = J · qvel

    let spring_mag = -k_bend_raw * angle_error;
    let damper_mag = -b_bend * theta_dot;
    let force_mag = spring_mag + damper_mag;

    // Apply via J^T to qfrc_passive, with per-vertex stability clamp.
    // fm_max = 1/(dt² * |grad| * invmass) — geometry-aware, prevents instability
    // regardless of mesh deformation state or Bridson gradient magnitudes.
    let grads = [(ve0, grad_e0), (ve1, grad_e1), (va, grad_a), (vb, grad_b)];
    for &(v_idx, grad) in &grads {
        let invmass = model.flexvert_invmass[v_idx];
        if invmass > 0.0 {
            let grad_norm = grad.norm();
            let mut fm = force_mag;
            if grad_norm > 0.0 {
                let fm_max = 1.0 / (dt * dt * grad_norm * invmass);
                fm = fm.clamp(-fm_max, fm_max);
            }
            let dof = model.flexvert_dofadr[v_idx];
            for ax in 0..3 {
                data.qfrc_passive[dof + ax] += grad[ax] * fm;
            }
        }
    }
}
```

The dihedral gradient uses the Bridson et al. 2003 formulation (same geometry
as in P6 edge constraints). Stability is ensured by a per-vertex force magnitude
clamp: `fm_max = 1/(dt² * |grad| * invmass)`. This accounts for the
geometry-dependent Bridson gradient norms and per-vertex mass, preventing
instability even as the mesh deforms and gradient magnitudes change.

> **Note:** MuJoCo uses a different bending discretization (Wardetzky cotangent
> Laplacian) that produces constant-coefficient forces and needs no clamp. See
> [§42B](./future_work_10.md) for a spec to support both models via a trait.

**Stiffness derivation from material properties:**

| Flex dim | Formula | Source |
|---|---|---|
| 2 (shell) | `D = E · t³ / (12 · (1 − ν²))` | Kirchhoff-Love thin plate theory |
| 3 (solid) | `D = E` (gradient provides geometric scaling) | Characteristic stiffness |
| 1 (cable) | `0.0` (no bending for cables) | N/A |

Damping: `b_bend = damping × k_bend` (proportional damping model).

---

**P7b. Volume constraints — NOT IMPLEMENTED (matches MuJoCo):**

MuJoCo has no dedicated volume preservation mechanism for flex bodies.
Volume preservation is an emergent property of the continuum elasticity model
(Saint-Venant-Kirchhoff), which is planned for §7. The previously-specified
`FlexVolume` constraint type has been removed as it had no MuJoCo equivalent.

---

**Bending hinge extraction** — during `build()`, find all pairs of elements
sharing an edge and compute the hinge topology. Algorithm
(migrated from `deformable/cloth.rs:290-331`):

```rust
/// Extract bending hinges from element connectivity.
/// For each edge shared by exactly 2 elements, create a hinge with the
/// shared-edge vertices and the two opposite vertices.
fn extract_hinges(
    elements: &[Vec<usize>],  // element connectivity
    positions: &[Vector3<f64>],
    flex_id: usize,
) -> Vec<FlexHinge> {
    let mut edge_elements: HashMap<(usize, usize), Vec<usize>> = HashMap::new();

    for (elem_idx, verts) in elements.iter().enumerate() {
        // Generate all edges of this element
        let edges = match verts.len() {
            3 => vec![(verts[0], verts[1]), (verts[1], verts[2]), (verts[0], verts[2])],
            4 => vec![
                (verts[0], verts[1]), (verts[0], verts[2]), (verts[0], verts[3]),
                (verts[1], verts[2]), (verts[1], verts[3]), (verts[2], verts[3]),
            ],
            _ => continue,
        };
        for (a, b) in edges {
            let key = if a < b { (a, b) } else { (b, a) };
            edge_elements.entry(key).or_default().push(elem_idx);
        }
    }

    let mut hinges = Vec::new();
    for ((v_e0, v_e1), elems) in &edge_elements {
        if elems.len() != 2 { continue; } // boundary edge or manifold issue

        // Find opposite vertices
        let v_opp_a = elements[elems[0]].iter()
            .find(|&&v| v != *v_e0 && v != *v_e1)
            .copied().unwrap();
        let v_opp_b = elements[elems[1]].iter()
            .find(|&&v| v != *v_e0 && v != *v_e1)
            .copied().unwrap();

        // Compute rest dihedral angle
        let rest_angle = compute_dihedral_angle(
            positions[*v_e0], positions[*v_e1],
            positions[v_opp_a], positions[v_opp_b],
        );

        hinges.push(FlexHinge {
            vert: [*v_e0, *v_e1, v_opp_a, v_opp_b],
            angle0: rest_angle,
            flex_id,
        });
    }
    hinges
}
```

##### P8. Integration Extension

**Velocity integration** — already handled. The existing loop in `integrate()`
(`mujoco_pipeline.rs:4637`) iterates over all `model.nv` DOFs:
`qvel[i] += qacc[i] * h`. Once `nv` includes flex DOFs, this loop automatically
covers flex velocity integration with zero additional code.

**Position integration** — needs explicit flex handling. The existing
`mj_integrate_pos()` (`mujoco_pipeline.rs:20255`) uses `visit_joints()` which
only iterates over joints (not flex vertices). Add `mj_integrate_pos_flex()`
after `mj_integrate_pos()` in `integrate()`:

```rust
/// Position integration for flex vertices: qpos += qvel * h.
/// Trivial linear update (no quaternions, no SO(3) manifold).
fn mj_integrate_pos_flex(model: &Model, data: &mut Data, h: f64) {
    for i in 0..model.nflexvert {
        let dof_base = model.flexvert_dofadr[i];
        let qpos_base = model.flexvert_qposadr[i];

        if model.flexvert_invmass[i] == 0.0 {
            continue; // Pinned vertex: skip integration
        }

        for k in 0..3 {
            data.qpos[qpos_base + k] += h * data.qvel[dof_base + k];
        }
    }
}
```

Call site in `integrate()` (after `mj_integrate_pos()`):
```rust
mj_integrate_pos(model, self, h);
mj_integrate_pos_flex(model, self, h);  // NEW: flex vertex positions
mj_normalize_quat(model, self);
// DELETE: mj_deformable_step(model, self);
```

**Passive forces for flex** — add to `mj_fwd_passive()`:

```rust
// Flex vertex damping: qfrc_passive[dof] = -damping * qvel[dof]
for i in 0..model.nflexvert {
    let flex_id = model.flexvert_flexid[i];
    let damp = model.flex_damping[flex_id];
    if damp <= 0.0 { continue; }
    let dof_base = model.flexvert_dofadr[i];
    for k in 0..3 {
        data.qfrc_passive[dof_base + k] -= damp * data.qvel[dof_base + k];
    }
}
```

**Gravity for flex** — add to `mj_rne()` or bias force computation.
Skip pinned vertices (`invmass == 0`) to avoid writing huge values
(`1e20 * 9.81`) into `qfrc_bias`, which can cause numerical issues in the
Newton solver's `meaninertia` computation:

```rust
// Flex vertex gravity: direct force on translational DOFs
// Skip pinned vertices (invmass == 0, mass == 1e20) to avoid huge values in qfrc_bias.
for i in 0..model.nflexvert {
    if model.flexvert_invmass[i] == 0.0 {
        continue; // Pinned vertex: skip gravity
    }
    let mass = model.flexvert_mass[i];
    let dof_base = model.flexvert_dofadr[i];
    for k in 0..3 {
        data.qfrc_bias[dof_base + k] -= mass * model.gravity[k];
    }
}
```

(Sign convention: `qfrc_bias` is subtracted from applied forces to get net
force, so gravity contribution is negative.)

##### P9. MJCF Parsing (`<flex>` and `<flexcomp>`)

Parse `<flex>` and `<flexcomp>` elements from `<deformable>` in
`parser.rs` and wire into `model_builder.rs`.

**`<flex>` parsing** — add to `parse_deformable()` (or new `parse_flex()`):

```rust
fn parse_flex(e: &Element) -> Result<MjcfFlex, ParseError> {
    let name = get_attribute_opt(e, "name").map(String::from);
    let dim: usize = get_attribute_opt(e, "dim")
        .map(|s| s.parse().unwrap_or(2))
        .unwrap_or(2);

    // Material properties
    let young = parse_float_attr(e, "young", 1e6);
    let poisson = parse_float_attr(e, "poisson", 0.0);
    let damping = parse_float_attr(e, "damping", 0.0);
    let thickness = parse_float_attr(e, "thickness", 0.001);
    let density = parse_float_attr(e, "density", 1000.0);

    // Collision properties
    let friction = parse_float_attr(e, "friction", 1.0);
    let condim = parse_int_attr(e, "condim", 3);
    let margin = parse_float_attr(e, "margin", 0.0);
    let selfcollide = parse_bool_attr(e, "selfcollide", false);

    // Parse child elements: <vertex>, <element>, <body>
    let vertices = parse_flex_vertices(e)?;
    let elements = parse_flex_elements(e, dim)?;
    let body_map = parse_flex_bodies(e)?;

    // solref/solimp: inherit from <option> defaults or per-flex attributes
    let solref = parse_solref(e, DEFAULT_SOLREF);
    let solimp = parse_solimp(e, DEFAULT_SOLIMP);

    Ok(MjcfFlex { name, dim, young, poisson, damping, thickness,
        density, friction, condim, margin, selfcollide, vertices,
        elements, body_map, solref, solimp })
}
```

**`<flexcomp>` parsing** — procedural mesh generation:

```rust
fn parse_flexcomp(e: &Element) -> Result<MjcfFlex, ParseError> {
    let comp_type = get_attribute_opt(e, "type").unwrap_or("grid");
    let count = parse_int3_attr(e, "count", [10, 10, 1]);
    let spacing = parse_float_attr(e, "spacing", 0.02);

    // Generate vertices and elements based on type
    let (vertices, elements, dim) = match comp_type {
        "grid" => generate_grid(count, spacing),      // 2D shell
        "box" => generate_box(count, spacing),         // 3D solid
        "cylinder" => generate_cylinder(count, spacing),
        _ => return Err(ParseError::UnsupportedFlexcompType(comp_type)),
    };

    // Wrap as MjcfFlex with parsed material/collision attributes
    // (same parsing as <flex> for material and collision attrs)
    ...
}
```

The grid/box/cylinder generators migrate from `sim-deformable`
(`cloth.rs:Cloth::grid()`, `soft_body.rs:SoftBody::box_mesh()`).

**Vertex mass computation** — element-based mass lumping (migrated from
`cloth.rs:244-262` and `soft_body.rs:179-199`):

```rust
/// Compute per-vertex masses via element-based mass lumping.
/// Each element's mass is distributed equally to its vertices.
/// Vertices shared by multiple elements accumulate mass from all.
fn compute_vertex_masses(flex: &MjcfFlex) -> Vec<f64> {
    let mut masses = vec![0.0f64; flex.vertices.len()];

    match flex.dim {
        1 => {
            // Cable: mass = linear_density * segment_length
            // linear_density derived from Young's/cross-section or explicit
            let density = flex.density.unwrap_or(1.0); // kg/m
            for elem in &flex.elements {
                let v0 = elem[0];
                let v1 = elem[1];
                let len = (flex.vertices[v1] - flex.vertices[v0]).norm();
                let mass_per_vert = density * len / 2.0;
                masses[v0] += mass_per_vert;
                masses[v1] += mass_per_vert;
            }
        }
        2 => {
            // Shell: mass = area_density * triangle_area / 3
            // area_density = volumetric_density * thickness
            let area_density = flex.density.unwrap_or(1000.0) * flex.thickness;
            for elem in &flex.elements {
                let (v0, v1, v2) = (elem[0], elem[1], elem[2]);
                let e1 = flex.vertices[v1] - flex.vertices[v0];
                let e2 = flex.vertices[v2] - flex.vertices[v0];
                let area = e1.cross(&e2).norm() / 2.0;
                let mass_per_vert = area_density * area / 3.0;
                masses[v0] += mass_per_vert;
                masses[v1] += mass_per_vert;
                masses[v2] += mass_per_vert;
            }
        }
        3 => {
            // Solid: mass = density * tet_volume / 4
            let density = flex.density.unwrap_or(1000.0); // kg/m³
            for elem in &flex.elements {
                let (va, vb, vc, vd) = (elem[0], elem[1], elem[2], elem[3]);
                let e1 = flex.vertices[vb] - flex.vertices[va];
                let e2 = flex.vertices[vc] - flex.vertices[va];
                let e3 = flex.vertices[vd] - flex.vertices[va];
                let volume = e1.dot(&e2.cross(&e3)).abs() / 6.0;
                let mass_per_vert = density * volume / 4.0;
                masses[va] += mass_per_vert;
                masses[vb] += mass_per_vert;
                masses[vc] += mass_per_vert;
                masses[vd] += mass_per_vert;
            }
        }
        _ => {}
    }

    // Ensure minimum mass (prevents zero-mass free-floating vertices)
    for m in &mut masses {
        if *m < 1e-10 {
            *m = 0.001; // 1 gram minimum
        }
    }

    masses
}
```

This is standard mass lumping: each element's total mass (density × measure)
is distributed equally to its vertices. Vertices shared by multiple elements
accumulate mass from all adjacent elements. This matches the existing
implementation in `cloth.rs:244-262` and `soft_body.rs:179-199`, and is
consistent with MuJoCo's vertex mass computation for flex bodies.

**Model builder wiring** — in `build()`, after processing bodies/joints/geoms,
process flex objects:

```rust
fn process_flex(&mut self, flex: &MjcfFlex) {
    let flex_id = self.nflex;
    self.nflex += 1;

    let vert_start = self.nflexvert;
    // Compute per-vertex masses via element-based mass lumping
    let vertex_masses = compute_vertex_masses(flex);
    // Add vertices
    for (i, pos) in flex.vertices.iter().enumerate() {
        let mass = vertex_masses[i];
        let inv_mass = if mass > 0.0 && mass < f64::INFINITY { 1.0 / mass } else { 0.0 };
        self.flexvert_positions.push(*pos);
        self.flexvert_mass.push(mass);
        self.flexvert_invmass.push(inv_mass);
        self.flexvert_flexid.push(flex_id);
        self.nflexvert += 1;
    }

    // Extract edges from element connectivity
    let edges = extract_edges(&flex.elements, flex.dim);
    let edge_start = self.nflexedge;
    for (v0, v1) in &edges {
        let rest_len = (flex.vertices[*v1] - flex.vertices[*v0]).norm();
        // Cross-section area for stiffness derivation (see compute_edge_solref)
        let cross_section = match flex.dim {
            1 => std::f64::consts::PI * flex.radius.powi(2),  // cable
            2 => flex.thickness * rest_len,  // shell: thickness × dual edge
            3 => rest_len * rest_len,        // solid: L² approximation
            _ => 1.0,
        };
        self.flexedge_vert.push([vert_start + v0, vert_start + v1]);
        self.flexedge_length0.push(rest_len);
        self.flexedge_crosssection.push(cross_section);
        self.flexedge_flexid.push(flex_id);
        self.nflexedge += 1;
    }

    // Extract bending hinges from adjacent elements
    let hinges = extract_hinges(&flex.elements, &flex.vertices, flex_id);
    let hinge_start = self.nflexhinge;
    for h in &hinges {
        self.flexhinge_vert.push([
            vert_start + h.vert[0], vert_start + h.vert[1],
            vert_start + h.vert[2], vert_start + h.vert[3],
        ]);
        self.flexhinge_angle0.push(h.angle0);
        self.flexhinge_flexid.push(flex_id);
        self.nflexhinge += 1;
    }

    // Compute rest volumes for tetrahedra (dim=3)
    let elem_start = self.nflexelem;
    for elem in &flex.elements {
        let adr = self.flexelem_data.len();
        for &v in elem {
            self.flexelem_data.push(vert_start + v);
        }
        self.flexelem_dataadr.push(adr);
        self.flexelem_datanum.push(elem.len());
        self.flexelem_flexid.push(flex_id);

        let rest_vol = if elem.len() == 4 {
            let e1 = flex.vertices[elem[1]] - flex.vertices[elem[0]];
            let e2 = flex.vertices[elem[2]] - flex.vertices[elem[0]];
            let e3 = flex.vertices[elem[3]] - flex.vertices[elem[0]];
            // Signed volume — matches constraint assembly convention.
            // Both rest_vol and current volume use signed values so
            // pos_error = volume - rest_vol is consistent regardless of winding.
            e1.dot(&e2.cross(&e3)) / 6.0
        } else {
            0.0
        };
        self.flexelem_volume0.push(rest_vol);
        self.nflexelem += 1;
    }

    // Store per-flex data
    self.flex_dim.push(flex.dim);
    self.flex_vertadr.push(vert_start);
    self.flex_vertnum.push(flex.vertices.len());
    self.flex_edgeadr.push(edge_start);
    self.flex_edgenum.push(edges.len());
    self.flex_elemadr.push(elem_start);
    self.flex_elemnum.push(flex.elements.len());
    // ... material, collision, solref, solimp, bend_solref, volume_solref
}
```

##### P10. sim-deformable Crate Deprecation

1. **Migrate mesh generators** — move `Cloth::grid()`, `SoftBody::box_mesh()`,
   `CapsuleChain::new()` logic into `model_builder.rs` as `<flexcomp>`
   expansion functions.

2. **Migrate material presets** — move `MaterialPreset` enum and default values
   into `model_builder.rs` as constants.

3. **Migrate constraint topology extraction** — move edge extraction, bending
   hinge finding, tetrahedron volume computation into `mujoco_pipeline.rs`
   (called during `build()`).

4. **Relocate skinning** — move `skinning.rs` to `sim-bevy` or a standalone
   `sim-skinning` crate (it's visual, not physics).

5. **Delete sim-deformable crate** — remove from workspace `Cargo.toml`,
   delete `sim/L0/deformable/` directory.

6. **Remove feature flag** — delete `deformable` feature from `sim-core/Cargo.toml`.
   All `#[cfg(feature = "deformable")]` annotations removed. Flex arrays are
   always present on Model/Data (zero-length when nflex=0, zero runtime cost).

7. **Update dependents** — remove `sim-deformable` from test crate
   (`sim/L0/tests/Cargo.toml`), physics crate (`sim/L0/physics/Cargo.toml`).

#### Acceptance Criteria

##### AC1 — 2D Shell (Cloth): Gravity Drape

A 10×10 grid cloth (`<flexcomp type="grid" count="10 10 1" spacing="0.02">`)
pinned at two corners, dropped under gravity for 2 seconds. Cloth should drape
realistically (vertices descend, edges stretch < 5% of rest length, no
explosion, no NaN).

##### AC2 — 1D Cable (Rope): Catenary Shape

A 20-segment cable (`dim=1`) pinned at both ends, sag under gravity. Verify
shape approximates a catenary. Maximum sag within 10% of analytical solution
for the given cable stiffness.

##### AC3 — 3D Solid (Soft Body): Compression Test

A 5×5×5 tetrahedral mesh resting on a plane under gravity. Volume preservation:
total volume at equilibrium within 5% of rest volume (depends on Poisson
ratio). No element inversion (all tetrahedra positive volume).

##### AC4 — Flex-Rigid Contact: Cloth on Sphere

Cloth draping over a rigid sphere. Contacts activate and resolve. Contact
forces push cloth vertices away from sphere. Final configuration: cloth conforms
to sphere surface with no penetration > 1mm.

##### AC5 — Edge Constraint Stiffness

Two vertices connected by a single edge with known Young's modulus. Apply
stretch force. At equilibrium, extension matches `ΔL = F * L / (E * A)` within
1% (for small deformations).

##### AC6 — Bending Stiffness

A flat strip of 3 triangles (4 vertices in a row). Apply vertical force to tip.
Deflection matches Euler-Bernoulli beam formula within 5% for small deflections.

##### AC7 — Volume Preservation (REMOVED)

Volume constraints have no MuJoCo equivalent — volume preservation is an
emergent property of continuum elasticity (SVK), planned for §7. The
`FlexVolume` constraint type and this acceptance test have been removed.
The original spec described: a single tetrahedron under uniform compression,
volume change bounded by Poisson ratio.

##### AC8 — Pinned Vertices: Zero Motion

Vertices marked as pinned (`mass = 1e20`, `invmass = 0`) have exactly zero velocity and
acceleration after any number of steps. `qpos` at pinned DOFs unchanged.

##### AC9 — Rigid-Only Regression

All existing rigid-only tests pass unchanged. When `nflex == 0`, the pipeline
produces bit-identical results to the pre-unification code.

##### AC10 — MJCF `<flex>` Round-Trip

Parse `<flex dim="2">` with explicit vertices and elements. Model loads
successfully. `nflex == 1`, `nflexvert` matches input count.

##### AC11 — MJCF `<flexcomp>` Expansion

Parse `<flexcomp type="grid" count="5 5 1" spacing="0.04">`. Model loads.
`nflexvert == 25`, `nflexedge` matches expected triangle mesh edge count.

##### AC12 — Mass Matrix Correctness

For a model with 1 flex vertex (mass=2.0, unpinned) and no rigid bodies:
`qM = diag(2, 2, 2)`. `qLD_diag_inv = [0.5, 0.5, 0.5]`. Gravity produces
`qacc = [0, 0, -9.81]` (or gravity direction).

##### AC13 — Mixed Rigid + Flex: No Cross-Contamination

A model with a pendulum (rigid, 1 hinge DOF) and a 4-vertex cloth (12 flex
DOFs). Mass matrix is block-diagonal (no coupling). Pendulum swings
independently of cloth. Cloth falls independently of pendulum (until contact).

##### AC14 — Newton Solver Convergence

The Newton solver converges with flex constraints present. Residual decreases
monotonically (or with controlled backtracking). Iteration count for a cloth +
rigid scene is within 2× of the rigid-only scene.

##### AC15 — Feature Flag Removed

`cargo build -p sim-core` succeeds without `--features deformable`. The
`sim-deformable` crate no longer exists in the workspace.

#### Implementation Order

Each phase must compile and pass `cargo test -p sim-core` before proceeding.

1. **P1: Model data extension.** Add all `flex_*` fields to Model with empty
   defaults. Add `nq_rigid`/`nv_rigid`. `nq`/`nv` computation extended.
   All existing tests pass (empty flex arrays → zero additional DOFs).

2. **P2: Data extension.** `make_data()` allocates larger qpos/qvel/qacc when
   `nflexvert > 0`. Add `flexvert_xpos`. Remove `deformable_*` fields
   (temporarily breaks deformable feature — OK, we're replacing it).

3. **P3: Forward kinematics.** Add `mj_fwd_position_flex()` to `forward_core()`.
   No-op when nflexvert=0.

4. **P4: Mass matrix.** Add `mj_crba_flex()` and `mj_factor_flex()`. Extend
   sparse LDL structure for flex diagonal DOFs.

5. **P5: Collision unification.** Add `mj_collision_flex()`. Flex-rigid contacts
   produce regular `Contact` entries. Add `flex_vertex` field to Contact.
   Update contact Jacobian assembly to handle flex-rigid contacts.

6. **P6: Edge constraints.** Add `FlexEdge` to `ConstraintType`. Add edge
   constraint assembly in `assemble_unified_constraints()`.

7. **P7: Bending passive forces.** Compute bending as passive spring-damper
   forces in `mj_fwd_passive()` (matching MuJoCo `engine_passive.c`).
   Volume constraints removed — MuJoCo has no dedicated volume mechanism.

8. **P8: Integration.** Add `mj_integrate_pos_flex()` for position integration.
   Velocity integration is free (existing `nv`-sized loop). Add gravity and
   damping for flex vertices in `mj_rne()` and `mj_fwd_passive()`. Remove
   `mj_deformable_step()` call site.

9. **P9: MJCF parsing.** Parse `<flex>` and `<flexcomp>`. Wire into Model
   builder.

10. **P10: Deprecation.** Migrate useful code from sim-deformable. Delete
    crate. Remove feature flag. Update all dependents.

#### Risk Mitigations

1. **Sparse LDL factorization with extended matrix.** Flex DOFs are diagonal-only
   (no off-diagonal ancestors), so the factorization extension is trivial. The
   rigid portion is unchanged. **Mitigation:** Unit test `qLD` for a mixed
   rigid+flex model against dense Cholesky.

2. **Numerical stability (vertex mass ranges).** Vertex masses can be very small
   (cloth: ~0.001 kg) while rigid body masses are large (~10 kg). This creates
   condition number issues in the mass matrix. **Mitigation:** The diagonal
   structure of flex M blocks means no amplification — each vertex is
   independent. The constraint solver's regularization (via `efc_R`) handles
   ill-conditioning.

3. **Convergence regression.** XPBD converges unconditionally (position-based).
   The unified velocity-based solver may require more iterations for stiff
   materials. **Mitigation:** Tune `solref` to match effective XPBD stiffness.
   Monitor residual convergence in acceptance tests.

4. **Breaking API changes.** `register_deformable()` is deleted. Any user code
   using it must switch to Model-builder API. **Mitigation:** Compile errors
   (not silent breakage). Migration guide in commit message.

5. **Contact Jacobian correctness for flex-rigid.** The vertex side of the
   Jacobian is trivial (identity-like), but the rigid side must correctly
   traverse the kinematic chain. **Mitigation:** Reuse proven
   `add_body_point_jacobian_row()` for the rigid side. Validate with
   finite-difference Jacobian test.

6. **Bending constraint Jacobian.** Dihedral angle gradient is numerically
   sensitive near 0° and 180°. **Mitigation:** Add epsilon clamping for
   near-degenerate hinges. Fall back to zero Jacobian (constraint becomes
   inactive) for degenerate cases.

7. **Volume constraint sign.** Inverted tetrahedra (negative volume) can occur
   under extreme deformation. **Mitigation:** Track element inversion; log
   warning but don't crash. Constraint still applies (restoring force toward
   positive volume).

#### Files

| File | Changes |
|------|---------|
| `sim/L0/core/src/mujoco_pipeline.rs` | Model: add `flex_*` fields, `nq_rigid`, `nv_rigid`. Data: remove deformable fields, add `flexvert_xpos`, extend allocations. Pipeline: add `mj_fwd_position_flex()`, `mj_crba_flex()`, `mj_factor_flex()`, `mj_collision_flex()`, `mj_integrate_pos_flex()`, flex constraint assembly in `assemble_unified_constraints()`, flex gravity in `mj_rne()`, flex damping in `mj_fwd_passive()`. Remove: `DeformableContact`, `mj_deformable_collision()`, `solve_deformable_contacts()`, `mj_deformable_step()`. |
| `sim/L0/mjcf/src/parser.rs` | Add `parse_flex()`, `parse_flexcomp()` inside `parse_deformable()`. |
| `sim/L0/mjcf/src/types.rs` | Add `MjcfFlex` struct with all flex MJCF attributes. |
| `sim/L0/mjcf/src/model_builder.rs` | Add `process_flex()`. Migrate mesh generators from sim-deformable. Compute edge/hinge topology. Compute `solref` and bending stiffness from material. |
| `sim/L0/tests/integration/` | New: `flex_unified.rs` (AC1–AC15). Update or remove `deformable_contact.rs`. |
| `sim/L0/tests/Cargo.toml` | Remove `sim-deformable` dependency. |
| `sim/L0/core/Cargo.toml` | Remove `sim-deformable` optional dependency and `deformable` feature. |
| `sim/L0/physics/Cargo.toml` | Remove `sim-deformable` dependency. |
| `sim/L0/deformable/` | **Delete entire directory** (after migration). |
| `sim/L1/bevy/` | Move skinning code here (or standalone crate). |
| `Cargo.toml` (workspace) | Remove `sim-deformable` from members. |
| `sim/docs/todo/index.md` | Add 6B reference. Mark #42 as subsumed. |
| `sim/docs/MUJOCO_GAP_ANALYSIS.md` | Update flex status. |
| `sim/docs/ARCHITECTURE.md` | Update deformable architecture description. |
