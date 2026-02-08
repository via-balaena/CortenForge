# Future Work 4 — Physics Completeness (Items #11–14)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 11. Deformable Body Pipeline Integration
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

*Transferred from [future_work_1.md](./future_work_1.md) #9.*

#### Current State

sim-deformable is a standalone 7,733-line crate (86 tests) with zero coupling to the
MuJoCo pipeline:

| Component | Location | Description |
|-----------|----------|-------------|
| `XpbdSolver` | `solver.rs:134` | XPBD constraint solver. `step(&mut self, body: &mut dyn DeformableBody, gravity: Vector3<f64>, dt: f64)` (`solver.rs:196`). Configurable substeps (1–4), damping, sleeping. |
| `DeformableBody` trait | `lib.rs:173` | Common interface: `positions()`, `velocities()`, `inverse_masses()`, `constraints()`, `material()`, `bounding_box()`. Implemented by Cloth, SoftBody, CapsuleChain. |
| `Cloth` | `cloth.rs` | Triangle meshes with distance + dihedral bending constraints. `thickness` field (`cloth.rs:50`). 6 presets. |
| `SoftBody` | `soft_body.rs` | Tetrahedral meshes with distance + volume constraints. 6 presets. |
| `CapsuleChain` | `capsule_chain.rs` | 1D particle chains with distance + bending constraints. `radius` field (`capsule_chain.rs:41`). 5 presets. |
| `Material` | `material.rs:94` | `friction: f64` (default 0.5). 14 presets. Unused by any collision system. |
| `ConstraintType::Collision` | `constraints.rs:43` | Enum variant defined but no `Constraint::Collision(...)` variant in the `Constraint` enum — dead code. |
| `VertexFlags::COLLIDING` | `types.rs:55` | Bitflag defined but never set. |

sim-physics re-exports the crate behind the `deformable` feature flag
(`physics/src/lib.rs:109-110`). `Model` (`mujoco_pipeline.rs:788`) and `Data`
(`mujoco_pipeline.rs:1595`) have zero deformable-related fields. The pipeline
functions `mj_collision()` (`mujoco_pipeline.rs:3886`), `mj_fwd_constraint()`
(`mujoco_pipeline.rs:11504`), and `Data::integrate()` (`mujoco_pipeline.rs:3175`)
operate exclusively on rigid bodies.

#### MuJoCo Reference

MuJoCo 3.x introduces flexible bodies via `<flex>` and `<flexcomp>` elements
(under `<deformable>` in MJCF). Key reference points:

- **`mjModel` flex fields:** `flex_vertadr`, `flex_vertnum`, `flex_edgeadr`,
  `flex_edgenum`, `flex_elemadr`, `flex_elemnum`, `flex_dim`, `flex_radius`,
  `flex_stiffness`, `flex_damping`, `flex_friction`, `flex_condim`, `flex_solref`,
  `flex_solimp`, `flex_contype`, `flex_conaffinity`.
- **Collision:** `engine_collision_driver.c` — `mj_collidePlaneFlex` (vertex-plane),
  `mj_collideGeomElem` (rigid geom vs flex element), `mj_collideFlexInternal`
  (self-collision). Flex elements get per-element AABBs with `flex_radius` margin.
- **Pipeline stages:** Flex position update in `mj_flex()` after FK; flex collision
  in `mj_collision()`; flex passive forces (edge stiffness/damping) in
  `mj_fwd_passive()`; flex contacts in unified constraint solver.
- **Contact model:** Each flex vertex produces contacts with `condim` from
  `flex_condim`. Contact Jacobian maps vertex velocity (3-DOF) to constraint space.
  Friction from `flex_friction` combined with rigid `geom_friction` via geometric
  mean (same rule as rigid-rigid).

**Design divergence:** MuJoCo flex bodies are position-based dynamics driven by the
constraint solver (edge-length equality constraints). CortenForge uses a separate
XPBD solver with its own substep loop. This spec couples XPBD to the rigid pipeline
via a **split-solve** approach (contact forces first, then XPBD projection) rather
than unifying all constraints into a single solver. This keeps the XPBD crate
self-contained and avoids restructuring the PGS/CG solver for mixed DOF types.

#### Objective

Couple deformable bodies to the rigid-body pipeline so that deformable vertices
participate in collision detection and contact force resolution alongside rigid
geoms, while XPBD internal constraints (stretch, bend, volume) remain solved by the
existing `XpbdSolver`.

#### Scope Exclusions

- **MJCF `<flex>`/`<flexcomp>` parsing:** Out of scope. Deformable bodies are
  registered programmatically via `Data::register_deformable()`. MJCF parsing for
  deformable elements is a separate future item.
- **Deformable-deformable collision:** Out of scope. Only deformable-vs-rigid geom
  contacts are supported. Deformable self-collision and deformable-deformable
  collision require BVH infrastructure that doesn't exist.
- **Deformable-vs-Mesh/Hfield/SDF narrowphase:** Out of scope. Only primitive geom
  types (Sphere, Box, Capsule, Cylinder, Ellipsoid, Plane) are supported for
  vertex-vs-geom contacts. Mesh/Hfield/SDF would require per-triangle or per-cell
  vertex queries.
- **GPU batch deformable stepping:** Out of scope. `BatchSim` will step deformable
  bodies sequentially per-environment; GPU-accelerated deformable solve is a separate
  item.
- **Skinned mesh pipeline:** `skinning.rs` is a visual-only module. Not touched here.

#### Specification

##### Step 0 — Feature gate

All deformable integration is gated behind a `deformable` feature flag on
`sim-core`. When disabled, `Model` and `Data` have zero deformable fields, and no
deformable code is compiled. This ensures zero overhead for rigid-only users.

```toml
# sim/L0/core/Cargo.toml
[features]
deformable = ["sim-deformable"]

[dependencies]
sim-deformable = { workspace = true, optional = true }
```

##### Step 1 — Model fields for deformable configuration

Add deformable metadata to `Model` (after `enableflags`, ~`mujoco_pipeline.rs:1180`):

```rust
// --- Deformable configuration (gated by #[cfg(feature = "deformable")]) ---

/// Contact dimension for deformable-rigid contacts (1, 3, 4, or 6).
/// Default: 3 (normal + 2D sliding friction). Applies to all deformable bodies.
#[cfg(feature = "deformable")]
pub deformable_condim: i32,

/// Solver reference parameters for deformable-rigid contacts.
#[cfg(feature = "deformable")]
pub deformable_solref: [f64; 2],

/// Solver impedance parameters for deformable-rigid contacts.
#[cfg(feature = "deformable")]
pub deformable_solimp: [f64; 5],
```

Default values (same as MuJoCo geom defaults, see `mujoco_pipeline.rs:12822`):

```rust
deformable_condim: 3,
deformable_solref: [0.02, 1.0],       // [timeconst, dampratio]
deformable_solimp: [0.9, 0.95, 0.001, 0.5, 2.0],  // [d0, dwidth, width, midpoint, power]
```

These are model-level defaults. Per-body overrides are not supported in this
iteration (simplifies the contact pipeline — all deformable contacts use the same
solver parameters).

**Construction sites:** `Model` has no `Default` impl. Two struct-literal construction
sites must be updated with these cfg-gated fields:

1. `Model::empty()` (`mujoco_pipeline.rs:1880`) — factory for programmatic model
   building and tests. Initialize with defaults above.
2. `ModelBuilder::build()` (`model_builder.rs:2305`) — MJCF-driven construction.
   Initialize with defaults above (MJCF `<flex>` parsing is out of scope, so these
   are always model-level defaults for now).

##### Step 2 — Data fields for deformable state

Add deformable runtime state to `Data` (after `energy_kinetic`, ~`mujoco_pipeline.rs:1812`):

```rust
// --- Deformable state (gated by #[cfg(feature = "deformable")]) ---

/// Registered deformable bodies. Boxed trait objects allow heterogeneous
/// Cloth/SoftBody/CapsuleChain storage. `Send` bound required because
/// `BatchSim::step_all()` uses rayon `par_iter_mut()` which requires
/// `Data: Send`. All three concrete types (Cloth, SoftBody, CapsuleChain)
/// are `Send` (composed of `Vec`, `f64`, nalgebra types — all `Send`).
#[cfg(feature = "deformable")]
pub deformable_bodies: Vec<Box<dyn DeformableBody + Send>>,

/// Per-body XPBD solver instances. Parallel array with `deformable_bodies`.
#[cfg(feature = "deformable")]
pub deformable_solvers: Vec<XpbdSolver>,

/// Deformable-rigid contacts detected this step. Separate from `data.contacts`
/// because the rigid contact solver operates on `data.contacts` (rigid-rigid)
/// and `data.deformable_contacts` (deformable-rigid) with different Jacobian
/// structures.
#[cfg(feature = "deformable")]
pub deformable_contacts: Vec<DeformableContact>,
```

**`Data` Clone + Debug compatibility:** `Data` derives `Clone` and `Debug`
(`mujoco_pipeline.rs:1593`). `Box<dyn DeformableBody + Send>` is neither `Clone`
nor `Debug` by default. Adding this field would break both `#[derive(Clone)]`
and `#[derive(Debug)]`. Fix:

1. **Clone:** Add a `clone_box(&self) -> Box<dyn DeformableBody + Send>` method
   to the `DeformableBody` trait (see Step 12) and replace `#[derive(Clone)]` on
   `Data` with a manual `impl Clone` that calls `clone_box()` for the deformable
   bodies Vec (cfg-gated). **Boilerplate note:** `Data` has ~71 fields, so a
   manual `impl Clone` is substantial. Mitigation: use a `clone_from_derive!`
   helper macro that clones all fields via `self.field.clone()` and overrides
   only the deformable fields, OR structure it as `let mut cloned = /* clone
   all non-deformable fields */; cloned.deformable_bodies = ...;`. The exact
   approach is left to implementation — the key requirement is that
   `clone_box()` is called for `deformable_bodies` and all other fields use
   their natural `Clone` impl.
2. **Debug:** Add `std::fmt::Debug` as a supertrait to `DeformableBody`
   (`pub trait DeformableBody: std::fmt::Debug { ... }`). All three concrete
   implementations (`Cloth`, `SoftBody`, `CapsuleChain`) already derive `Debug`,
   so this is non-breaking in practice. This makes `dyn DeformableBody + Send`
   implement `Debug`, which lets `#[derive(Debug)]` work on `Data` — so only
   `Clone` needs a manual impl. Alternatively, both could be manual impls, but
   adding `Debug` as a supertrait is simpler and more useful (enables debug
   printing of trait objects in user code).

`XpbdSolver` already derives `Clone` and `Debug`, and `DeformableContact` is a
plain data struct that derives both.

The `DeformableContact` struct (new, in `mujoco_pipeline.rs`):

```rust
/// A contact between a deformable vertex and a rigid geom.
#[derive(Debug, Clone)]
#[cfg(feature = "deformable")]
pub struct DeformableContact {
    /// Index into `deformable_bodies` array.
    pub deformable_idx: usize,
    /// Vertex index within the deformable body.
    pub vertex_idx: usize,
    /// Rigid geom index (into `model.geom_*` arrays).
    pub geom_idx: usize,
    /// Contact position in world frame (closest point on the rigid geom surface,
    /// consistent with the rigid-rigid `Contact.pos` convention).
    pub pos: Vector3<f64>,
    /// Contact normal (from rigid surface toward deformable vertex, unit vector).
    /// Points in the direction that pushes the vertex out of penetration.
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = overlapping).
    pub depth: f64,
    /// Combined sliding friction: sqrt(material.friction * geom_friction.x).
    pub friction: f64,
    /// Contact dimension (from model.deformable_condim).
    /// Reserved for friction cone projection (deferred — see solver note).
    pub dim: usize,
    /// 5-element friction vector [sliding1, sliding2, torsional, rolling1, rolling2].
    /// Reserved for friction cone projection (deferred — see solver note).
    pub mu: [f64; 5],
    /// Solver reference parameters (used by ERP computation in solver).
    pub solref: [f64; 2],
    /// Solver impedance parameters.
    /// Reserved for impedance scaling (deferred — see solver note).
    pub solimp: [f64; 5],
    /// Tangent frame vectors for friction cone.
    /// Reserved for friction cone projection (deferred — see solver note).
    pub frame: [Vector3<f64>; 2],
}
```

##### Step 3 — Registration and allocation API

```rust
impl Data {
    /// Register a deformable body for simulation.
    ///
    /// The body is consumed and stored internally. Returns the index into
    /// `deformable_bodies` for later reference.
    #[cfg(feature = "deformable")]
    pub fn register_deformable(
        &mut self,
        body: Box<dyn DeformableBody + Send>,
        solver_config: SolverConfig,
    ) -> usize {
        let idx = self.deformable_bodies.len();
        self.deformable_bodies.push(body);
        self.deformable_solvers.push(XpbdSolver::new(solver_config));
        idx
    }

    /// Get a reference to a registered deformable body.
    #[cfg(feature = "deformable")]
    pub fn deformable(&self, idx: usize) -> &dyn DeformableBody { ... }

    /// Get a mutable reference to a registered deformable body.
    #[cfg(feature = "deformable")]
    pub fn deformable_mut(&mut self, idx: usize) -> &mut dyn DeformableBody { ... }
}
```

`Model::make_data()` (`mujoco_pipeline.rs:2133`) initializes the new fields:

```rust
#[cfg(feature = "deformable")]
deformable_bodies: Vec::new(),
#[cfg(feature = "deformable")]
deformable_solvers: Vec::new(),
#[cfg(feature = "deformable")]
deformable_contacts: Vec::with_capacity(256),
```

**Reset behavior:** `Data::reset()` (`mujoco_pipeline.rs:2993`) must be extended
for deformable state when the `deformable` feature is enabled:

```rust
#[cfg(feature = "deformable")]
{
    self.deformable_contacts.clear();
    for body in &mut self.deformable_bodies {
        // Zero velocities but preserve positions — deformable bodies
        // don't have a "default pose" in Model (they're registered
        // dynamically). Users who need full position reset should
        // re-register the body or implement their own snapshot/restore.
        for vel in body.velocities_mut() {
            *vel = Vector3::zeros();
        }
        body.clear_forces();
    }
}
```

This mirrors the rigid reset pattern: `qvel.fill(0.0)` but `qpos` is restored
from `model.qpos0`. Since deformable bodies are registered on `Data` (not
`Model`), there's no model-level default pose. Positions are preserved so
deformable geometry isn't lost on reset. Callers who need position reset should
snapshot positions at registration time and restore manually.

##### Step 4 — Collision detection: `mj_deformable_collision()`

New function, called from `Data::forward()` after `mj_collision()`:

```rust
/// Detect contacts between deformable vertices and rigid geoms.
///
/// For each deformable body, expands vertex positions by the body's collision
/// margin (radius for CapsuleChain, thickness/2 for Cloth, default 0.005 for
/// SoftBody) to form per-vertex AABBs. Tests these against rigid geom AABBs, then runs
/// narrowphase vertex-vs-geom closest-point queries.
#[cfg(feature = "deformable")]
fn mj_deformable_collision(model: &Model, data: &mut Data) {
    data.deformable_contacts.clear();

    if data.deformable_bodies.is_empty() {
        return; // Zero overhead when no deformables registered
    }

    // ── Phase 1: Clear COLLIDING flags (mutable borrow, see Step 9) ──
    for body in &mut data.deformable_bodies {
        for flag in body.vertex_flags_mut() {
            flag.remove(VertexFlags::COLLIDING);
        }
    }

    // ── Phase 2: Detect contacts (shared borrow of deformable_bodies) ──

    // Pre-compute rigid geom AABBs (reuse from mj_collision if cached,
    // otherwise recompute — O(ngeom)).
    let rigid_aabbs: Vec<Aabb> = (0..model.ngeom)
        .map(|g| aabb_from_geom(model.geom_type[g], model.geom_size[g],
                                data.geom_xpos[g], data.geom_xmat[g]))
        .collect();

    for (deform_idx, body) in data.deformable_bodies.iter().enumerate() {
        let margin = body.collision_margin();
        let positions = body.positions();
        let material = body.material();

        for (vert_idx, pos) in positions.iter().enumerate() {
            // Skip pinned vertices — they are fixed in world and cannot collide
            if body.inverse_masses()[vert_idx] <= 0.0 {
                continue;
            }

            // Vertex AABB: point ± margin
            let vert_aabb = Aabb::from_center(
                *pos, Vector3::new(margin, margin, margin),
            );

            // Broadphase: test vertex AABB against all rigid geom AABBs
            for geom_idx in 0..model.ngeom {
                if !vert_aabb.overlaps(&rigid_aabbs[geom_idx]) {
                    continue;
                }

                // Narrowphase: vertex-vs-geom closest point
                if let Some(contact) = collide_vertex_geom(
                    pos, margin,
                    model, geom_idx,
                    data.geom_xpos[geom_idx], data.geom_xmat[geom_idx],
                    material, deform_idx, vert_idx,
                ) {
                    data.deformable_contacts.push(contact);
                }
            }
        }
    }

    // ── Phase 3: Set COLLIDING flags on contacted vertices (see Step 9) ──
    for contact in &data.deformable_contacts {
        data.deformable_bodies[contact.deformable_idx]
            .vertex_flags_mut()[contact.vertex_idx]
            .insert(VertexFlags::COLLIDING);
    }
}
```

`collision_margin()` is called directly on the body (trait method, see Step 12).

Body type overrides for `collision_margin()`:
- `CapsuleChain`: return `self.config.radius` (capsule geometry)
- `Cloth`: return `self.config.thickness / 2.0` (shell thickness)
- `SoftBody`: uses trait default (`0.005`). Surface vertices need a non-zero
  margin for broadphase overlap detection; zero margin creates degenerate
  point AABBs that almost never trigger broadphase overlap in floating-point.

##### Step 5 — Narrowphase: `collide_vertex_geom()`

New function implementing vertex-vs-rigid-geom closest-point queries:

```rust
/// Compute contact between a deformable vertex and a rigid geom.
///
/// The vertex is treated as a sphere with the given margin radius.
/// Returns DeformableContact if penetrating, None otherwise.
fn collide_vertex_geom(
    vertex_pos: &Point3<f64>,
    vertex_margin: f64,
    model: &Model,
    geom_idx: usize,
    geom_pos: Vector3<f64>,
    geom_mat: Matrix3<f64>,
    material: &Material,
    deform_idx: usize,
    vert_idx: usize,
) -> Option<DeformableContact>
```

The function dispatches on `model.geom_type[geom_idx]`. Each case computes
`d_surface` (signed distance from vertex to geom surface, negative = inside).
The contact depth is then `depth = vertex_margin - d_surface` (positive when
the margin-sphere overlaps the geom). A contact is generated when `depth > 0`.

| Geom type | `d_surface` computation | Notes |
|-----------|-------------------------|-------|
| `Plane` | `d = n · (v - p)` (signed distance, positive = outside). | Normal = plane normal. Trivial. |
| `Sphere` | `d = \|v - center\| - radius`. Normal = `(v - center) / \|v - center\|`. | Handle degenerate `\|v - center\| ≈ 0` with fallback normal. |
| `Box` | Clamp vertex to box in local frame: `closest = clamp(v_local, -half, +half)`. `d = \|v - closest\|` (outside) or `-min_axis_depth` (inside). | Same algorithm as `collide_sphere_box()` (`mujoco_pipeline.rs:5222`). |
| `Capsule` | Closest point on axis segment to vertex. `d = \|v - closest\| - capsule_radius`. | Use `closest_point_segment()` (`mujoco_pipeline.rs:5851`). Capsule axis = `geom_mat.column(2) * half_length`. |
| `Cylinder` | Project vertex onto axis and radial surface. Three cases: endcap, barrel, edge. | Same 3-case dispatch as `collide_cylinder_sphere()` (`mujoco_pipeline.rs:5325`). |
| `Ellipsoid` | Scale vertex into unit-sphere space (divide by radii), compute `d` as sphere, transform back. | Approximate — normal inexact for non-spherical ellipsoids but sufficient. |
| `Mesh`, `Hfield`, `Sdf` | Return `None`. | Out of scope (see Scope Exclusions). |

For all cases: `depth = vertex_margin - d_surface`. Contact generated when
`depth > 0`. Normal points from rigid geom surface toward deformable vertex
(push direction — the direction to move the vertex to resolve penetration).
This matches the Sphere/Plane normals above (`(v - center)` points away from
the sphere, `plane_normal` points away from the plane's solid half-space).

**Friction combination** (same rule as `make_contact_from_geoms()`,
`mujoco_pipeline.rs:4189`):

```rust
let deform_friction = material.friction;  // material.rs:94
let rigid_friction = model.geom_friction[geom_idx];  // Vector3: [sliding, torsional, rolling]

let sliding = (deform_friction * rigid_friction.x).sqrt();  // Geometric mean
let torsional = (deform_friction * rigid_friction.y).sqrt();
let rolling = (deform_friction * rigid_friction.z).sqrt();
```

**Tangent frame computation:** Reuse `compute_tangent_frame(normal)` to build
orthonormal tangent vectors `t1`, `t2` for friction cone projection. Same function
used by rigid-rigid contacts.

**Solver parameters:** Use `model.deformable_solref` and `model.deformable_solimp`
(model-level defaults from Step 1). No per-vertex overrides.

##### Step 6 — Contact solver coupling

Deformable-rigid contacts are solved **separately** from rigid-rigid contacts,
after the rigid contact solve completes. This avoids modifying the rigid PGS/CG
solver's Jacobian and mass-matrix assembly (which assumes rigid-body kinematic
chains).

**In `mj_fwd_constraint()` (`mujoco_pipeline.rs:11504`), append:**

```rust
#[cfg(feature = "deformable")]
if !data.deformable_contacts.is_empty() {
    solve_deformable_contacts(model, data);
}
```

**Helper functions** (all in `mujoco_pipeline.rs`):

Two new helpers are needed. Two existing functions are reused:

**Existing — reuse directly:**

- `compute_point_velocity(data, body_id, point) -> Vector3<f64>`
  (`mujoco_pipeline.rs:11783`): Computes rigid body velocity at a world-space
  point using spatial velocity (`data.cvel`). O(1) — no Jacobian assembly.
  Already used by the rigid-rigid contact solver.

- `apply_contact_force(model, data, body_id, contact_point, force)`
  (`mujoco_pipeline.rs:11801`): Maps a world-space force at a contact point
  to generalized forces via kinematic-chain J^T walk. Accumulates into
  `data.qfrc_constraint`. Already used by the rigid-rigid contact solver
  for reaction forces.

**New:**

```rust
/// Compute the effective inverse mass of a rigid body at a contact point
/// projected onto a contact normal direction.
///
/// inv_m_eff = n^T * J * M^{-1} * J^T * n
/// where J is the 3×nv translational Jacobian at the contact point.
///
/// **NEW — does not exist in the codebase.**
/// Requires the full 3×nv Jacobian (not just the x-row from the existing
/// `compute_body_jacobian_at_point()`). The 3-row Jacobian is built with
/// the same kinematic-chain walk as the existing function but stores all
/// 3 components (j_col.x/y/z) in rows 0/1/2.
fn compute_rigid_inv_mass_at_point(
    model: &Model,
    data: &Data,
    body_id: usize,
    point: Vector3<f64>,
    normal: &Vector3<f64>,
) -> f64 {
    if body_id == 0 { return 0.0; } // World body = infinite mass

    // Build 3×nv translational Jacobian at point.
    // Same walk as compute_body_jacobian_at_point() (line 9339-9401)
    // but stores all 3 rows (x, y, z).
    let mut jac = DMatrix::zeros(3, model.nv);
    // ... kinematic chain walk, storing j_col.x/y/z in rows 0/1/2 ...

    // j_n = J^T * n  (nv×3 * 3×1 = nv×1)
    let n = DVector::from_column_slice(normal.as_slice());
    let j_n = jac.transpose() * &n;
    // inv_m = j_n^T * M^{-1} * j_n
    // Use factored mass matrix (qLD from CRBA, Data fields at line 1731)
    // to solve M^{-1} * j_n via mj_solve_sparse() (line 12122).
    let mut m_inv_jn = j_n.clone();
    mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut m_inv_jn);
    j_n.dot(&m_inv_jn)
}

/// Compute error reduction parameter (ERP) from solver reference parameters.
///
/// MuJoCo solref = [timeconst, dampratio]:
///   erp = dt / (dt + 2 * timeconst * dampratio)
///
/// NEW — does not exist in the codebase.
#[inline]
fn compute_erp(dt: f64, solref: &[f64; 2]) -> f64 {
    let timeconst = solref[0];
    let dampratio = solref[1];
    dt / (dt + 2.0 * timeconst * dampratio)
}
```

The `solve_deformable_contacts()` function:

```rust
/// Solve deformable-rigid contact forces using Jacobi-style iteration.
///
/// Each deformable vertex is a 3-DOF point mass (no rotational DOFs).
/// The contact Jacobian for a deformable vertex is the identity matrix
/// (vertex velocity maps directly to contact-frame velocity via the
/// contact normal and tangent frame).
///
/// **Jacobi vs Gauss-Seidel:** The rigid-rigid PGS solver updates
/// impulses immediately per-contact (true Gauss-Seidel). This solver
/// uses a two-phase batch approach instead: Phase 1 reads all velocities
/// and computes impulses, Phase 2 applies them. This is Jacobi-style
/// (stale velocities within an iteration). The reason is borrow safety:
/// true GS requires interleaved mutable access to `deformable_bodies`
/// while iterating `deformable_contacts`, which would require unsafe
/// indexing or interior mutability. Jacobi converges more slowly but
/// is correct — the outer iteration loop compensates. For most scenes
/// (few multi-contact vertices), the difference is negligible.
///
/// Three-phase algorithm for borrow safety:
/// 1. Read phase: iterate contacts, compute impulses, accumulate into
///    scratch buffer `impulses: Vec<(contact_idx, Vector3<f64>)>`.
/// 2a. Write phase: apply accumulated impulses to vertex velocities.
/// 2b. Reaction phase: apply reaction forces to rigid bodies via
///     apply_contact_force() (requires &mut Data, so contact fields
///     must be copied into locals first).
fn solve_deformable_contacts(model: &Model, data: &mut Data) {
    // Scratch buffer for per-contact impulses.
    // Each entry: (contact_index, impulse_vector).
    // One entry per contact, not per unique vertex — a vertex with
    // multiple contacts gets multiple entries.
    let mut impulses: Vec<(usize, Vector3<f64>)> =
        Vec::with_capacity(data.deformable_contacts.len());

    let dt = model.timestep;

    // ── Phase 1: Read-only solve (immutable borrow of deformable_bodies) ──
    for _iter in 0..model.solver_iterations {
        let mut max_delta = 0.0_f64;
        impulses.clear();

        for (contact_idx, contact) in data.deformable_contacts.iter().enumerate() {
            let body = &data.deformable_bodies[contact.deformable_idx];
            let inv_m = body.inverse_masses()[contact.vertex_idx];

            if inv_m <= 0.0 { continue; } // Pinned vertex

            let v_deform = body.velocities()[contact.vertex_idx];

            // Rigid body velocity at contact point.
            // Reuse existing compute_point_velocity() (line 11783) which
            // uses spatial velocity (data.cvel) — O(1), no Jacobian needed.
            let rigid_body_idx = model.geom_body[contact.geom_idx];
            let v_rigid = compute_point_velocity(
                data, rigid_body_idx, contact.pos,
            );

            // Relative velocity in contact frame
            let v_rel = v_deform - v_rigid;
            let v_n = v_rel.dot(&contact.normal);

            // Normal impulse with Baumgarte correction
            let erp = compute_erp(dt, &contact.solref);
            let rhs = v_n + erp * contact.depth / dt;

            // Effective inverse mass (deformable point mass + rigid contribution)
            let inv_m_eff = inv_m + compute_rigid_inv_mass_at_point(
                model, data, rigid_body_idx, contact.pos, &contact.normal,
            );

            let lambda_n = (-rhs / inv_m_eff).max(0.0);
            max_delta = max_delta.max(lambda_n.abs());

            // Accumulate impulse for this contact (normal direction only).
            //
            // DEFERRED: Friction cone projection. For condim >= 3, the
            // impulse should be projected into the friction cone using
            // project_elliptic_cone() (line 9920), same algorithm as
            // rigid-rigid PGS. This requires computing tangent impulses
            // from the tangent-frame velocity components and clamping via
            // the mu vector. The struct fields `friction`, `mu`, `dim`,
            // `solimp`, and `frame` are populated for this purpose but
            // NOT consumed by this implementation. Adding friction cone
            // projection is a follow-up — the normal-only solver is
            // functionally complete for frictionless contacts and serves
            // as the scaffolding for the full solver.
            let impulse = contact.normal * lambda_n;
            impulses.push((contact_idx, impulse));
        }

        // ── Phase 2a: Apply velocity updates to deformable vertices ──
        // Borrows data.deformable_contacts (shared) and
        // data.deformable_bodies (mutable) — safe because they are
        // separate fields of Data.
        for &(contact_idx, ref impulse) in &impulses {
            let contact = &data.deformable_contacts[contact_idx];
            let body = &mut data.deformable_bodies[contact.deformable_idx];
            let inv_m = body.inverse_masses()[contact.vertex_idx];
            body.velocities_mut()[contact.vertex_idx] += impulse * inv_m;
        }

        // ── Phase 2b: Apply reaction impulses to rigid bodies ──
        // Reuse existing apply_contact_force() (line 11801) which does
        // the J^T kinematic-chain walk internally.
        //
        // BORROW SAFETY: apply_contact_force() takes &mut Data, which
        // conflicts with any live reference into Data's fields. We must
        // NOT hold a reference into data.deformable_contacts across the
        // call. Solution: copy needed values (geom_idx, pos) into locals
        // BEFORE calling apply_contact_force().
        for &(contact_idx, ref impulse) in &impulses {
            // Copy contact fields into locals to release the borrow of
            // data.deformable_contacts before the &mut data call.
            let geom_idx = data.deformable_contacts[contact_idx].geom_idx;
            let pos = data.deformable_contacts[contact_idx].pos;
            let rigid_body_idx = model.geom_body[geom_idx];
            // apply_contact_force handles body_id == 0 (world) internally.
            // Negative impulse = reaction force (Newton's third law).
            apply_contact_force(model, data, rigid_body_idx, pos, -*impulse);
        }

        if max_delta < model.solver_tolerance {
            break;
        }
    }
}
```

**Borrow analysis:** Phase 1 borrows `data.deformable_bodies` (shared:
`velocities()`, `inverse_masses()`) and `data.deformable_contacts` (shared)
simultaneously — both are immutable, so this is safe. Phase 2a borrows
`data.deformable_contacts` immutably (to look up contact by index) and
`data.deformable_bodies` mutably (via `velocities_mut()`) — safe because
they are separate fields. Phase 2b calls `apply_contact_force(model, data,
...)` which takes `&mut Data` — this conflicts with any live reference into
Data's fields. The borrow of `data.deformable_contacts[idx]` is released
before the `apply_contact_force` call by copying `geom_idx` and `pos` into
local variables. The two phases alternate each iteration, never overlapping.
`impulses` is a local scratch buffer keyed by contact index, avoiding the
ambiguity of `(deform_idx, vertex_idx)` which is non-unique when a vertex
has multiple contacts.

##### Step 7 — XPBD stepping

After contact forces are applied to vertex velocities, run XPBD constraint
projection. Extract the stepping logic into a free function so it can be called
from both `Data::integrate()` and `mj_runge_kutta()`:

```rust
/// Step all deformable bodies through XPBD.
///
/// Extracted as a free function because it must be called from two sites:
/// - `Data::integrate()` (Euler / ImplicitSpringDamper path)
/// - `mj_runge_kutta()` (RK4 path — does NOT call `integrate()`)
///
/// Contact impulses must already be applied to vertex velocities
/// (in solve_deformable_contacts) before calling this function.
#[cfg(feature = "deformable")]
fn mj_deformable_step(model: &Model, data: &mut Data) {
    if data.deformable_bodies.is_empty() {
        return;
    }

    let gravity = model.gravity;
    let dt = model.timestep;

    // XPBD step for each deformable body.
    // Gravity IS passed to XpbdSolver::step() because the XPBD algorithm
    // applies gravity internally during its predict-velocities phase
    // (solver.rs:222-229). The rigid pipeline does NOT apply gravity to
    // deformable vertices — that responsibility belongs to the XPBD solver.
    //
    // Borrow split: `deformable_solvers` and `deformable_bodies` are
    // separate Vec fields on Data, so we can borrow them independently
    // by destructuring or using iterators.
    for (solver, body) in data.deformable_solvers.iter_mut()
        .zip(data.deformable_bodies.iter_mut())
    {
        solver.step(body.as_mut(), gravity, dt);
    }
}
```

**Call site 1 — `Data::integrate()` (Euler / ImplicitSpringDamper):**

```rust
// In Data::integrate(), after mj_integrate_pos() and mj_normalize_quat():
#[cfg(feature = "deformable")]
mj_deformable_step(model, self);
```

**Call site 2 — `mj_runge_kutta()` (RK4):**

`Data::step()` (`mujoco_pipeline.rs:3042`) dispatches to `mj_runge_kutta()`
for `Integrator::RungeKutta4`. This function (`mujoco_pipeline.rs:12631`) does
NOT call `Data::integrate()` — it handles position/velocity integration
internally via Butcher tableau stages. If XPBD stepping only lived in
`integrate()`, deformable bodies would never be stepped under RK4.

```rust
// In mj_runge_kutta(), after the final position/velocity assembly
// and before returning Ok(()):
#[cfg(feature = "deformable")]
mj_deformable_step(model, data);
```

Note: RK4 sub-stage evaluations call `forward_skip_sensors()` which includes
`mj_fwd_constraint()` → `solve_deformable_contacts()`. So contact impulses
are computed at each RK4 stage. The XPBD step runs once at the end of the
full RK4 step (not per-stage), consistent with RK4 being a multi-evaluation
integrator for the rigid DOFs while XPBD handles its own sub-stepping
internally.

**Split-solve ordering rationale:** Contact forces push vertices away from rigid
surfaces. XPBD then adjusts vertices to satisfy internal constraints (stretch,
bend, volume). For compliant deformable bodies (cloth, rope), XPBD moves vertices
only slightly, so contact penetration remains small. For stiff bodies where XPBD
significantly displaces vertices back into rigid geoms, users should increase
`SolverConfig::num_substeps` on the XPBD solver (which subdivides the timestep
internally, re-applying forces at each sub-interval).

A full re-detect + re-solve loop (the "Option B" from the original spec) is
**not implemented** in this iteration. If needed, it can be added as a follow-up
by wrapping Steps 4–7 in a configurable iteration count.

##### Step 8 — `ConstraintType::Collision` implementation

Add `CollisionConstraint` to sim-deformable (`constraints.rs`). Requires adding
`Vector3` to the existing import (`use nalgebra::{Point3, Vector3};` — currently
only `Point3` is imported at `constraints.rs:26`):

```rust
/// Collision constraint for deformable-rigid contacts.
///
/// Maintains separation between a deformable vertex and a rigid surface.
/// Used by the XPBD solver to enforce contact constraints during the
/// internal substep loop.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CollisionConstraint {
    /// Vertex index.
    pub vertex: usize,
    /// Contact normal (from rigid surface toward vertex — push direction).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = penetrating).
    pub depth: f64,
    /// Compliance (0 = rigid contact).
    pub compliance: f64,
    /// Accumulated Lagrange multiplier.
    lambda: f64,
}

impl CollisionConstraint {
    pub fn new(vertex: usize, normal: Vector3<f64>, depth: f64) -> Self {
        Self { vertex, normal, depth, compliance: 0.0, lambda: 0.0 }
    }

    pub fn solve(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        let w = inv_masses[self.vertex];
        if w <= 0.0 { return 0.0; }

        // Constraint: C = -depth (negative when penetrating)
        // Gradient: normal direction
        let c = -self.depth;
        if c >= 0.0 { return 0.0; } // Not penetrating

        let alpha_tilde = self.compliance / (dt * dt);
        let delta_lambda = (-c - alpha_tilde * self.lambda) / (w + alpha_tilde);
        let delta_lambda = delta_lambda.max(-self.lambda); // Clamp: no adhesion

        positions[self.vertex] += self.normal * (w * delta_lambda);
        self.lambda += delta_lambda;

        c.abs()
    }
}
```

Add `Collision(CollisionConstraint)` variant to the `Constraint` enum:

```rust
pub enum Constraint {
    Distance(DistanceConstraint),
    Bending(BendingConstraint),
    Volume(VolumeConstraint),
    FlexEdge(FlexEdgeConstraint),
    Collision(CollisionConstraint),  // NEW
}
```

Add `Collision` match arms to all four `Constraint` methods:

```rust
// constraint_type():
Self::Collision(_) => ConstraintType::Collision,

// vertices():
Self::Collision(c) => {
    let mut v = SmallVec::new();
    v.push(c.vertex);
    v
}

// compliance():
Self::Collision(c) => c.compliance,

// solve():
Self::Collision(c) => c.solve(positions, inv_masses, dt),
```

**Note:** The `CollisionConstraint` is used internally by the XPBD solver for
optional position-level contact projection. The primary contact resolution happens
in `solve_deformable_contacts()` (velocity-level PGS). These are complementary:
PGS handles the impulse-level contact, while `CollisionConstraint` can be injected
into the XPBD constraint list for position-level cleanup in subsequent substeps.

**Lambda accumulation caveat:** The existing XPBD solver (`solver.rs:310-327`)
copies each constraint before solving it (`let mut c = *c;`), which means
`lambda` starts at 0.0 each iteration. This is a pre-existing behavior affecting
ALL constraint types, not specific to `CollisionConstraint`. For collision
constraints with `compliance = 0`, this is benign: `alpha_tilde = 0`, so the
lambda term drops out and the constraint reduces to a simple position projection.
Fixing cross-iteration lambda accumulation is a solver improvement outside the
scope of this spec.

##### Step 9 — `VertexFlags::COLLIDING` activation

Integrated directly into `mj_deformable_collision()` (Step 4) as Phase 1
(clear) and Phase 3 (set). See the three-phase structure in the Step 4 code.

**Borrow analysis:** Phase 1 (clear flags) and Phase 3 (set flags) use
mutable borrows of `data.deformable_bodies`. Phase 2 (detect contacts) uses
shared borrows. The three phases are sequential, so borrows never overlap.
In Phase 3, `data.deformable_contacts` is borrowed immutably (to read
contact indices) while `data.deformable_bodies[idx]` is borrowed mutably
through indexing — safe under NLL because they are separate fields.

##### Step 10 — Pipeline integration in `Data::forward()` and `Data::step()`

Modify `Data::forward()` (`mujoco_pipeline.rs:3075`) to insert deformable collision
after rigid collision:

```rust
pub fn forward(&mut self, model: &Model) -> Result<(), StepError> {
    // ... existing position stage ...
    mj_fwd_position(model, self);
    mj_transmission_site(model, self);
    mj_collision(model, self);               // Rigid-rigid contacts
    #[cfg(feature = "deformable")]
    mj_deformable_collision(model, self);    // Deformable-rigid contacts (NEW)
    mj_sensor_pos(model, self);
    mj_energy_pos(model, self);

    // ... existing velocity + acceleration stages (unchanged) ...
    // mj_fwd_constraint() now also calls solve_deformable_contacts()
}
```

Also modify `Data::forward_skip_sensors()` (`mujoco_pipeline.rs:3139`), which is
the RK4 sub-stage evaluation function. It has the same structure as `forward()` but
skips sensor computations. Insert `mj_deformable_collision()` after
`mj_collision()` (between lines 3142 and 3143):

```rust
fn forward_skip_sensors(&mut self, model: &Model) -> Result<(), StepError> {
    // ========== Position Stage ==========
    mj_fwd_position(model, self);
    mj_collision(model, self);
    #[cfg(feature = "deformable")]
    mj_deformable_collision(model, self);    // NEW — required for RK4 sub-stages
    mj_energy_pos(model, self);
    // ... rest unchanged ...
}
```

Without this, RK4 sub-stages would not detect deformable-rigid contacts, causing
`mj_fwd_constraint()` → `solve_deformable_contacts()` to operate on stale/empty
contact data during RK4 stages 2–4.

No changes to `Data::step()` (`mujoco_pipeline.rs:3032`) — it calls `forward()`
then `integrate()` (or `mj_runge_kutta()`), both paths now include deformable
stages.

##### Step 11 — BatchSim support

`BatchSim` (`batch.rs:64`) stores `Vec<Data>` — each `Data` independently owns its
deformable bodies and solvers. No changes to `BatchSim` are needed because:

1. `Data::step()` already calls `forward()` → `integrate()`, which now includes
   deformable collision + solve + XPBD stepping.
2. Each `Data` has its own `deformable_bodies` and `deformable_solvers`.
3. Rayon parallelism in `step_all()` works unchanged — each env's `Data` is
   stepped independently.

The only requirement is that callers register deformable bodies on each `Data`
in the batch. The `Send` bound on `Box<dyn DeformableBody + Send>` (Step 2)
ensures `Data` remains `Send` for rayon parallelism:

```rust
let model = Arc::new(Model::...);
let mut batch = BatchSim::new(Arc::clone(&model), 64);
for env in batch.envs_mut() {
    let rope: Box<dyn DeformableBody + Send> =
        Box::new(CapsuleChain::new("rope", ...));
    env.register_deformable(rope, SolverConfig::realtime());
}
```

##### Step 12 — `DeformableBody` trait extension

Three changes to `DeformableBody` trait in `sim/L0/deformable/src/lib.rs`:

**1. Add `Debug` supertrait:**

```rust
// Change from:
pub trait DeformableBody {
// To:
pub trait DeformableBody: std::fmt::Debug {
```

Required because `Data` derives `Debug` and will contain
`Vec<Box<dyn DeformableBody + Send>>`. Without `Debug` as a supertrait, the
trait object doesn't implement `Debug`, breaking `#[derive(Debug)]` on `Data`.
All three concrete implementations already derive `Debug`, so this is
non-breaking in practice.

**2. Add `collision_margin()` default method:**

```rust
/// Get the collision margin (radius/thickness) for broadphase expansion.
/// Default: 0.005 (5 mm). Override for bodies with geometry-specific margins.
fn collision_margin(&self) -> f64 { 0.005 }
```

**3. Add `clone_box()` required method:**

```rust
/// Clone this deformable body into a new Box.
///
/// Required because `Data` derives `Clone` (`mujoco_pipeline.rs:1593`) but
/// `Box<dyn DeformableBody + Send>` is not `Clone` by default. This is the
/// standard Rust "clone_box" pattern for clonable trait objects.
///
/// All three implementations (Cloth, SoftBody, CapsuleChain) derive `Clone`
/// and are `Send`, so the implementation is trivial: `Box::new(self.clone())`.
fn clone_box(&self) -> Box<dyn DeformableBody + Send>;
```

`collision_margin()` is a default-method addition — **not a breaking change**.
Existing implementations compile without modification. `CapsuleChain` and `Cloth`
override it with geometry-specific margins (see Step 4a). `SoftBody` uses the
default.

`Debug` supertrait and `clone_box()` are both **breaking changes** that require
all `DeformableBody` implementors to derive `Debug` (already true for all three
concrete types) and add:

```rust
fn clone_box(&self) -> Box<dyn DeformableBody + Send> {
    Box::new(self.clone())
}
```

With `Debug` as a supertrait, `#[derive(Debug)]` on `Data` continues to work
unchanged. Only `#[derive(Clone)]` must be replaced with a manual `impl Clone
for Data` that calls `clone_box()`:

```rust
// Replace #[derive(Debug, Clone)] with #[derive(Debug)] + manual Clone impl.
//
// Data has ~71 fields. To avoid 71 lines of `field: self.field.clone()`,
// use a two-step approach:
// 1. Temporarily remove the deformable_bodies field from Data (or use
//    unsafe transmute / mem::ManuallyDrop to clone the non-trait-object
//    portion via derive).
// 2. Override deformable_bodies with clone_box().
//
// Practical approach: since all 68 non-deformable fields and the 2 other
// deformable fields (solvers, contacts) implement Clone naturally, the
// simplest implementation is a full field-by-field clone. This is verbose
// but correct and generated once. Example (abbreviated):
impl Clone for Data {
    fn clone(&self) -> Self {
        Self {
            // ... all ~68 existing fields: `field: self.field.clone(),` ...
            // (generated at implementation time from the current field list)
            #[cfg(feature = "deformable")]
            deformable_bodies: self.deformable_bodies
                .iter()
                .map(|b| b.clone_box())
                .collect(),
            #[cfg(feature = "deformable")]
            deformable_solvers: self.deformable_solvers.clone(),
            #[cfg(feature = "deformable")]
            deformable_contacts: self.deformable_contacts.clone(),
        }
    }
}
```

#### Acceptance Criteria

1. A rigid sphere (r=0.1, m=1.0) dropped from height 0.5 onto a `Cloth` with
   `Material::preset(Rubber)` comes to rest with contact force magnitude within
   10% of `m*g` (±0.981 N). The cloth deforms but does not penetrate more than
   `2 × cloth.thickness`.
2. A `CapsuleChain` (10 segments, `radius=0.01`) draped over a rigid box edge
   reaches static equilibrium within 500 timesteps (dt=1/60). Max velocity of any
   vertex < 1e-4 m/s at equilibrium.
3. `Material.friction` affects contact forces: **deferred until friction cone
   projection is implemented** (see solver note in Step 6). For this iteration,
   verify that `DeformableContact.friction` is correctly populated with the
   geometric mean `sqrt(material.friction * geom_friction.x)`. Behavioral
   friction tests (sliding vs stationary on tilted plane) require friction cone
   projection and are acceptance criteria for that follow-up.
4. `ConstraintType::Collision` variant is implemented with `CollisionConstraint`
   struct. Solving a collision constraint with `depth = 0.01` (penetrating) pushes
   the vertex out by `0.01 / (1 + α̃)` in the normal direction. With
   `compliance = 0`, the vertex moves the full `0.01 * inv_mass / inv_mass = 0.01`.
5. A model with zero deformable bodies has zero overhead: `mj_deformable_collision`
   returns immediately, no `DeformableContact` allocation, no XPBD stepping.
   `#[cfg(feature = "deformable")]` gates all deformable code at compile time.
6. `VertexFlags::COLLIDING` is set on vertices in contact and cleared each step.
   A cloth draped on a sphere has COLLIDING set on ~20-40% of vertices (those near
   the sphere surface).
7. Contact normal direction is consistent: normal points from rigid surface toward
   deformable vertex (push direction). `DeformableContact.depth > 0` means
   penetrating. Pushing vertex along `normal * depth` resolves penetration.
8. Friction combination uses geometric mean: `sqrt(material.friction * geom_friction.x)`.
   Verified by comparing contact friction values against hand-computed expected
   values for at least 3 material/geom friction combinations.
9. All 86 existing sim-deformable tests pass unchanged.
10. All existing sim-core tests pass unchanged (deformable feature is additive).
11. `cargo test -p sim-core --features deformable` runs at least 10 new integration
    tests covering criteria 1–2 and 4–8 (criterion 3 behavioral friction tests are
    deferred until friction cone projection is implemented).
12. `cargo clippy -p sim-core --features deformable -- -D warnings` passes clean.

#### Files

- `sim/L0/deformable/src/lib.rs` — modify: add `Debug` supertrait,
  `collision_margin()` default method, and `clone_box()` required method to
  `DeformableBody` trait
- `sim/L0/deformable/src/constraints.rs` — modify: add `CollisionConstraint` struct,
  add `Collision(CollisionConstraint)` variant to `Constraint` enum, update all
  match arms
- `sim/L0/deformable/src/capsule_chain.rs` — modify: implement `collision_margin()`
  (return `self.config.radius`) and `clone_box()` (trivial: `Box::new(self.clone())`)
- `sim/L0/deformable/src/cloth.rs` — modify: implement `collision_margin()`
  (return `self.config.thickness / 2.0`) and `clone_box()`
- `sim/L0/deformable/src/soft_body.rs` — modify: implement `clone_box()` (uses
  default `collision_margin()` of 0.005)
- `sim/L0/core/Cargo.toml` — modify: add `deformable` feature flag with
  `sim-deformable` optional dependency
- `sim/L0/core/src/mujoco_pipeline.rs` — modify: add `DeformableContact` struct,
  add deformable fields to `Model` and `Data` (cfg-gated), update `Model::empty()`
  (~line 1880) with cfg-gated field defaults, replace
  `#[derive(Debug, Clone)]` on `Data` with `#[derive(Debug)]` + manual
  `impl Clone` (~71 fields, see Step 2 for mitigation; `Debug` derive works
  because `DeformableBody` now has `Debug` supertrait), add
  `register_deformable()`, extend `Data::reset()` for deformable state, add
  `mj_deformable_collision()`, `collide_vertex_geom()`,
  `solve_deformable_contacts()` (reuses existing `compute_point_velocity()` and
  `apply_contact_force()`), `compute_rigid_inv_mass_at_point()` (new, builds
  3×nv Jacobian internally), `compute_erp()` (new), `mj_deformable_step()` (new,
  called from both `integrate()` and `mj_runge_kutta()`), modify `forward()`,
  `forward_skip_sensors()` (RK4 sub-stage path), `integrate()`, and
  `mj_runge_kutta()` to call deformable stages
- `sim/L0/mjcf/src/model_builder.rs` — modify: add cfg-gated deformable fields
  to `ModelBuilder::build()` struct literal (~line 2305) with default values
- `sim/L0/core/src/batch.rs` — no changes (deformable state lives in `Data`)
- `sim/L0/tests/integration/` — new: `deformable_contact.rs` test file

---

### 12. Analytical Derivatives (mjd_*)
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

#### Current State
No analytical derivative infrastructure exists. The pipeline computes contact
Jacobians (`compute_contact_jacobian()` at line 7619) and body-point Jacobians
(`compute_body_jacobian_at_point()` at line 7534) for the constraint solver, but
these are not exposed as a general derivative API.

MuJoCo provides `mjd_transitionFD` (finite-difference derivatives of the full
transition function) and analytical derivatives for specific pipeline stages. These
are critical for model-based RL (iLQR, DDP), trajectory optimization, and system
identification.

#### Objective
Provide derivatives of the simulation transition function
`(q_{t+1}, v_{t+1}) = f(q_t, v_t, u_t)` with respect to state and control.

#### Specification

This item is intentionally sparse — the implementation strategy (analytical vs
finite-difference vs automatic differentiation) should be chosen at design time:

**Option A (pragmatic):** Finite-difference derivatives via `mjd_transitionFD`
pattern — step the simulation with perturbed inputs and compute centered
differences. Easy to implement, works with the existing pipeline, O(nv + nu) cost
per derivative computation.

**Option B (performant):** Analytical derivatives for each pipeline stage. Requires
deriving and implementing gradients through FK, contact, constraint solver, and
integration. Matches MuJoCo's `mjd_*` functions. Much more complex but O(1)
additional cost.

**Option C (modern):** Automatic differentiation via dual numbers or tape-based AD.
Would require making the pipeline generic over scalar type or using an AD library.

#### Acceptance Criteria
1. `d(q_{t+1})/d(q_t)`, `d(q_{t+1})/d(v_t)`, `d(q_{t+1})/d(u_t)` are computable.
2. Derivatives agree with finite differences to within numerical precision.
3. Derivative computation cost is documented (FD: ~O(nv * step_cost), analytical: ~O(step_cost)).

#### Files
- `sim/L0/core/src/` — new module (derivatives)

---

### 13. Full Implicit Integrator
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
`Integrator::ImplicitSpringDamper` implements diagonal-only implicit integration:

- Per-DOF stiffness K and damping D from joint properties
- Solves `(M + h*D + h^2*K) * v_new = M*v_old + h*f_ext - h*K*(q - q_eq)`
- Diagonal K and D — no off-diagonal coupling
- Tendon springs/dampers explicitly skipped in implicit mode
  (`mujoco_pipeline.rs:7403–7406`, comment: "Tendon springs/dampers couple multiple
  joints (non-diagonal K/D), so they cannot be absorbed into the existing diagonal
  implicit modification.")

MuJoCo's full implicit integrator handles the complete mass-matrix coupling
including off-diagonal stiffness and damping terms from tendons and other
multi-joint coupling elements.

#### Objective
Extend the implicit integrator to handle full (non-diagonal) stiffness and damping
matrices, including tendon spring/damper coupling across joints.

#### Specification

Replace diagonal K/D vectors with sparse K/D matrices. In `mj_fwd_passive()`,
compute off-diagonal entries from tendon Jacobians:

```
K_ij += J_tendon[i] * k_tendon * J_tendon[j]
D_ij += J_tendon[i] * d_tendon * J_tendon[j]
```

The implicit solve becomes `(M + h*D + h^2*K) * v = rhs` with sparse M+hD+h^2K.
Reuse the existing sparse Cholesky infrastructure from Task #2 (Phase 1).

#### Acceptance Criteria
1. With tendon springs, implicit integrator produces stable integration where
   explicit Euler diverges.
2. Diagonal-only case (no tendons) matches current `ImplicitSpringDamper` output
   (regression test).
3. Off-diagonal K/D terms are sparse — zero overhead for joints without coupling.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_fwd_passive()`,
  `mj_fwd_acceleration_implicit()`, implicit parameter storage)

---

### 14. Keyframes, Mocap Bodies, User Callbacks
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
None of these features exist in the pipeline. The only reference to "keyframe" in
the codebase is a doc comment in sim-constraint's equality module.

#### Objective
Support MJCF `<keyframe>` state snapshots for quick reset, `mocap` bodies for
externally driven poses, and (deferred) user callback hooks for custom logic
injection during simulation.

#### Specification

**Keyframes:**

Store named state snapshots in `Model`:

```rust
pub struct Keyframe {
    pub name: String,
    pub qpos: DVector<f64>,
    pub qvel: DVector<f64>,
    pub act: DVector<f64>,
    pub ctrl: DVector<f64>,
}
```

`Data::reset_to_keyframe(&mut self, &Model, keyframe_idx: usize)` restores
state. Parse from MJCF `<keyframe><key name="..." qpos="..."/></keyframe>`.

**Mocap bodies:**

Bodies with `mocap="true"` have externally driven position/orientation. They
participate in collision but are not integrated — their pose is set directly
via `data.mocap_pos[i]` and `data.mocap_quat[i]`.

```rust
// In Data:
pub mocap_pos: Vec<Vector3<f64>>,
pub mocap_quat: Vec<UnitQuaternion<f64>>,
```

Pipeline skips FK and integration for mocap bodies; collision uses the
mocap-set pose directly.

**User callbacks (deferred):**

MuJoCo's `mjcb_*` hooks (passive force, control, sensor, collision filter) are
useful but require careful API design for Rust's ownership model. Defer to a
follow-up — keyframes and mocap are higher priority.

#### Acceptance Criteria
1. `Data::reset_to_keyframe()` restores exact state from MJCF `<keyframe>`.
2. Mocap body pose is externally settable and affects collision but not integration.
3. Mocap bodies have zero mass contribution to the system.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (Model fields, Data fields,
  `Data::step()` skip for mocap, `reset_to_keyframe()`)
- `sim/L0/mjcf/src/parser.rs` — modify (parse `<keyframe>`, `mocap` attribute)
- `sim/L0/mjcf/src/model_builder.rs` — modify (build keyframe/mocap data)
