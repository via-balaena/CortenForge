# Future Work 4 — Physics Completeness (Items #11–14)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 11. Deformable Body Pipeline Integration
**Status:** ✅ Complete | **Effort:** XL | **Prerequisites:** None

*Transferred from [future_work_1.md](./future_work_1.md) #9.*

#### Pre-Implementation State (historical)

sim-deformable was a standalone 7,733-line crate (86 tests) with zero coupling to the
MuJoCo pipeline. All items below have been resolved by the implementation:

| Component | Location | Description |
|-----------|----------|-------------|
| `XpbdSolver` | `solver.rs:134` | XPBD constraint solver. Now called from `mj_deformable_step()` in pipeline. |
| `DeformableBody` trait | `lib.rs:173` | Common interface. Extended with `collision_margin()`, `clone_box()`, `Debug` supertrait. |
| `Cloth` | `cloth.rs` | Triangle meshes. Now participates in deformable-rigid contact. |
| `SoftBody` | `soft_body.rs` | Tetrahedral meshes. Now participates in deformable-rigid contact. |
| `CapsuleChain` | `capsule_chain.rs` | 1D particle chains. Now participates in deformable-rigid contact. |
| `Material` | `material.rs:94` | `friction: f64` (default 0.5). Now used by `mj_deformable_collision()` for geometric friction. |
| `Constraint::Collision(CollisionConstraint)` | `constraints.rs` | ~~Dead code~~ → fully implemented with `solve()` method. |
| `VertexFlags::COLLIDING` | `types.rs:55` | ~~Never set~~ → set/cleared each step by `mj_deformable_collision()`. |

`Model` has cfg-gated deformable solver params (`deformable_condim`, `deformable_solref`,
`deformable_solimp`). `Data` has cfg-gated fields (`deformable_bodies`, `deformable_solvers`,
`deformable_contacts`). Pipeline functions `mj_deformable_collision()`,
`solve_deformable_contacts()`, and `mj_deformable_step()` are wired into `forward()`,
`mj_fwd_constraint()`, `integrate()`, and `mj_runge_kutta()`.

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

#### Implementation Notes ✅

Implemented via split-solve approach: velocity-level Jacobi PGS contact solver
for deformable-rigid coupling, followed by position-level correction, followed
by XPBD internal constraint projection. All changes `#[cfg(feature = "deformable")]`-gated.

**Phase A — sim-deformable trait/constraint changes:**
- Added `Debug` supertrait, `collision_margin()` default method, `clone_box()` required method
- Added `CollisionConstraint` struct with `solve()` returning pre-correction depth
- Implemented on all three body types; `CapsuleChain` returns `config.radius`,
  `Cloth` returns `config.thickness / 2.0`, `SoftBody` uses default 0.005
- All trait object bounds use `Send + Sync` for bevy `Resource` compatibility

**Phase B — sim-core Model/Data fields:**
- `DeformableContact` struct (cfg-gated): vertex/body/geom indices, contact frame,
  depth, friction, solver parameters
- Model fields: `deformable_condim`, `deformable_solref`, `deformable_solimp`
- Data fields: `deformable_bodies: Vec<Box<dyn DeformableBody + Send + Sync>>`,
  `deformable_solvers: Vec<XpbdSolver>`, `deformable_contacts: Vec<DeformableContact>`
- Manual `impl Clone for Data` (~71 fields) with cfg-gated `clone_box()` for trait objects
- Registration API: `register_deformable()`, `deformable()`, `deformable_mut()`
- `Data::reset()` clears contacts, zeros velocities, clears forces

**Phase C — Collision detection (`mj_deformable_collision()`):**
- Phase 1: Clear `VertexFlags::COLLIDING`
- Phase 2: Per-body collision with rigid geoms (broadphase AABB + narrowphase dispatch)
- Phase 3: Set `VertexFlags::COLLIDING` on contacted vertices
- Phase 4: Position-level correction (enhancement beyond spec — prevents penetration drift)
- `collide_vertex_geom()`: plane, sphere, box, capsule, cylinder, ellipsoid dispatch
- Plane broadphase bypass (enhancement — infinite planes can't use thin-slab AABBs)

**Phase D — Contact solver (`solve_deformable_contacts()`):**
- Jacobi-style 3-phase solver with Baumgarte stabilization
- `compute_erp()`, `compute_rigid_inv_mass_at_point()` helpers
- Early exit on solver tolerance convergence

**Phase E — Pipeline wiring:**
- `mj_deformable_step()`: steps all XPBD solvers
- Inserted into `forward()`, `forward_skip_sensors()`, `mj_fwd_constraint()`,
  `integrate()`, and `mj_runge_kutta()`

**Phase F — MJCF builder:** cfg-gated default fields in `ModelBuilder::build()`

**Phase G — Integration tests (12 tests):**
1. `test_zero_deformable_zero_overhead`
2. `test_cloth_sphere_contact_force`
3. `test_cloth_penetration_bounded`
4. `test_capsule_chain_on_box_equilibrium`
5. `test_collision_constraint_solve`
6. `test_vertex_flags_colliding`
7. `test_contact_normal_direction`
8. `test_friction_combination`
9. `test_pinned_vertex_no_collision`
10. `test_deformable_clone`
11. `test_deformable_reset`
12. `test_rk4_with_deformable`

---

### 12. Analytical Derivatives (mjd_*)
**Status:** ✅ Complete (Parts 1 & 2, Steps 0–12) | **Effort:** XL | **Prerequisites:** None

#### Current State

**Both Part 1 (Steps 0–7) and Part 2 (Steps 8–12) are complete.** The derivative
infrastructure is implemented in `sim-core/src/derivatives.rs` (~1685 lines) with
30+ integration tests in `sim/L0/tests/integration/derivatives.rs`. Public API:

| Function / Type | Description | Part |
|-----------------|-------------|------|
| `TransitionMatrices` | Struct: A (state), B (control), C/D (sensor, `None` for now) | 1 |
| `DerivativeConfig` | Struct: eps, centered, use_analytical | 1 |
| `mjd_transition_fd()` | Pure FD transition Jacobians (centered/forward, any integrator) | 1 |
| `mjd_smooth_vel()` | Analytical `∂(qfrc_smooth)/∂qvel` → `Data.qDeriv` | 1 |
| `mjd_quat_integrate()` | SO(3) Jacobians for quaternion integration (right Jacobian + adjoint) | 2 |
| `mjd_transition_hybrid()` | Hybrid FD+analytical: analytical velocity/activation columns, FD position columns | 2 |
| `mjd_transition()` | Public dispatch: FD-only or hybrid based on `DerivativeConfig.use_analytical` | 2 |
| `Data::transition_derivatives()` | Convenience method on `Data` | 2 |
| `validate_analytical_vs_fd()` | Compares analytical vs FD Jacobians, returns max error | 2 |
| `fd_convergence_check()` | Verifies FD convergence at two epsilon scales | 2 |
| `max_relative_error()` | Element-wise max relative error between two matrices | 2 |

Data fields added: `qDeriv` (nv×nv), `deriv_Dcvel`/`deriv_Dcacc`/`deriv_Dcfrc`
(per-body 6×nv Jacobians). Visibility changed to `pub(crate)`:
`spatial_cross_motion`, `spatial_cross_force`, `joint_motion_subspace`,
`mj_solve_sparse`, `cholesky_solve_in_place`.

Pre-Part 1 state (historical — the table below describes what existed before
Part 1 was implemented). The pipeline computed several Jacobians internally for
the constraint solver, but these were not exposed as a general derivative API:

| Component | Location | Description |
|-----------|----------|-------------|
| `compute_contact_jacobian()` | `mujoco_pipeline.rs:9647` | dim×nv contact Jacobian for PGS/CG. Not public. |
| `compute_body_jacobian_at_point()` | `mujoco_pipeline.rs:9561` | 1×nv translational Jacobian. `#[allow(dead_code)]`. |
| Tendon Jacobians | `mujoco_pipeline.rs:7454,7488` | 1×nv rows accumulated during `mj_fwd_tendon_fixed` / `mj_fwd_tendon_spatial`. Stored in `data.ten_J` (line 1819). |
| `mj_differentiate_pos()` | `mujoco_pipeline.rs:13212` | Position-space finite differences (qpos₁, qpos₂) → qvel. Public. |
| `mj_integrate_pos_explicit()` | `mujoco_pipeline.rs:13330` | Velocity integration on SO(3) manifold. Public. |
| `mj_crba()` | `mujoco_pipeline.rs:8745` | Mass matrix M(q) via CRBA. Dense nv×nv in `data.qM`. |
| `mj_factor_sparse()` | `mujoco_pipeline.rs:12862` (called at line 8903) | L^T D L factorization for O(nv) solve. |
| `mj_solve_sparse()` | `mujoco_pipeline.rs:12920` | Solve L^T D L x = b. Private (needs `pub(crate)`). |
| `mj_rne()` | `mujoco_pipeline.rs:9113` | Bias forces (gravity + gyroscopic + Coriolis) via Featherstone RNE. |
| `mj_fwd_passive()` | `mujoco_pipeline.rs:9400` | Diagonal springs/dampers + tendon springs/dampers + friction loss. |
| `mj_fwd_actuation()` | `mujoco_pipeline.rs:8592` | Gain/bias force model + activation dynamics + transmission. |
| `mj_fwd_acceleration_implicit()` | `mujoco_pipeline.rs:12967` | Solves `(M + hD + h²K) v_new = RHS`. |
| `joint_motion_subspace()` | `mujoco_pipeline.rs:9025` | 6×6 motion subspace per joint. Private (needs `pub(crate)`). |
| `spatial_cross_motion()` | `mujoco_pipeline.rs:112` | Spatial motion cross product. Private (needs `pub(crate)`). |
| `spatial_cross_force()` | `mujoco_pipeline.rs:135` | Spatial force cross product. Private (needs `pub(crate)`). |
| `Model.implicit_damping` | `mujoco_pipeline.rs:1213` | Cached diagonal D per-DOF. |
| `Model.implicit_stiffness` | `mujoco_pipeline.rs:1210` | Cached diagonal K per-DOF. |

The existing `mj_differentiate_pos()` and `mj_integrate_pos_explicit()` handle
the nq↔nv mapping for quaternion joints (Ball, Free) and are foundational
building blocks for both FD and analytical derivative paths. The sparse L^T D L
factorization provides O(nv) inverse-mass-matrix solves. The `joint_motion_subspace`
and `spatial_cross_*` functions are needed by the analytical Coriolis matrix
computation.

#### MuJoCo Reference

MuJoCo provides two families of derivative functions:

**Finite-difference (transition-level):**

- **`mjd_transitionFD`**: Computes discrete-time Jacobians of the full step
  function `(x_t, u_t) → (x_{t+1}, s_{t+1})` where the state vector is
  `x = [qpos (mapped to nv tangent), qvel, act]` (dimension `2*nv + na`)
  and `s` is the sensor output (dimension `nsensordata`). Returns four matrices:
  - **A**: `∂x_{t+1}/∂x_t` — `(2*nv+na) × (2*nv+na)` state transition
  - **B**: `∂x_{t+1}/∂u_t` — `(2*nv+na) × nu` control influence
  - **C**: `∂s_{t+1}/∂x_t` — `nsensordata × (2*nv+na)` sensor-state
  - **D**: `∂s_{t+1}/∂u_t` — `nsensordata × nu` sensor-control
  Uses centered or forward differences. Does NOT support RK4 integrator.

- **`mjd_inverseFD`**: Computes continuous-time Jacobians of inverse dynamics
  `(q, v, a) → (f, s)`. Returns `∂f/∂q`, `∂f/∂v`, `∂f/∂a` (each nv×nv) plus
  sensor Jacobians and optionally `∂M/∂q` (sparse tensor).

**Analytical (stage-level, velocity-only):**

- **`mjd_smooth_vel`**: Analytical `∂(smooth forces)/∂qvel`. Stored in
  `mjData.qDeriv` (sparse nv×nv). Computes derivatives of passive forces,
  actuator forces, and bias forces w.r.t. velocity. Used internally by
  `mjd_transitionFD` for improved accuracy via analytical-in-FD hybrid.
- **`mjd_passive_vel`**: Adds `∂(passive forces)/∂qvel` to `qDeriv`.
- **`mjd_actuator_vel`**: Adds `∂(actuator forces)/∂qvel` to `qDeriv`.
- **`mjd_rne_vel`**: Subtracts `∂(bias forces)/∂qvel` from `qDeriv`.
- **`mjd_subQuat`**: Quaternion subtraction derivatives (two 3×3 Jacobians).
- **`mjd_quatIntegrate`**: Quaternion integration derivatives (3×3 each for
  orientation and velocity).

**Hybrid approach (MuJoCo's actual `mjd_transitionFD` implementation):**

MuJoCo's `mjd_transitionFD` is **not** pure FD. It uses a hybrid: analytical
derivatives where available (velocity columns via `qDeriv`, integration
derivatives for Euler), and FD only for the hard parts (position columns,
complex actuators). The velocity columns of A use:

```
∂v_{t+1}/∂v_t = I + h · M⁻¹ · qDeriv   (Euler)
```

This avoids FD for velocity columns, halving the cost for many models.

**Key design insight from Todorov (2014):** MuJoCo's convex contact formulation
makes inverse dynamics analytically invertible — the forward solver is iterative
(PGS/CG/Newton) but the inverse is closed-form. FD through a single `step()`
captures the full contact-aware dynamics at the cost of
`O(2·(nv+na) + 2·nu)` extra `step()` calls for centered differences.

**Design divergence:** MuJoCo stores `qDeriv` in sparse format (band-limited
by the kinematic tree). CortenForge uses dense `DMatrix<f64>` because target
models have `nv < 100`, making the O(nv²) storage (~80 KB for nv=100) negligible
and avoiding the complexity of sparse row/column indexing. Additionally, MuJoCo's
`mjd_transitionFD` writes sensor derivatives (C, D matrices) — we defer these
(see Scope Exclusions) and reserve `Option` fields for future implementation.
The bias force velocity derivative (`mjd_rne_vel`) uses the same chain-rule
propagation approach as MuJoCo (forward/backward Jacobian accumulation through
the kinematic tree) but stores per-body Jacobians in dense 6×nv matrices
rather than MuJoCo's sparse band-limited format.

#### Objective

Provide a four-phase derivative infrastructure that incrementally builds from
black-box FD to a hybrid analytical+FD approach:

1. **Phase A**: Pure FD transition derivatives (validation backbone)
2. **Phase B**: Analytical smooth-force velocity derivatives (`qDeriv`)
3. **Phase C**: Analytical integration derivatives (quaternion chain rules)
4. **Phase D**: Hybrid FD+analytical transition derivatives (MuJoCo's approach)

Each phase is independently useful and testable. Phase A alone unblocks
iLQR/DDP/MPC workflows. Phase D provides ~2x speedup by eliminating FD for
velocity and activation columns.

#### Scope Exclusions

- **Full position-analytical derivatives** (`∂FK/∂q`, `∂M/∂q`): Massive
  complexity (tensor differentiation through CRBA, RNE). Deferred. Position
  columns remain FD even in Phase D.
- **Contact-analytical derivatives** (implicit function theorem through PGS/CG):
  Research-grade. Deferred. Contact sensitivity is captured through FD.
- **Sensor derivatives** (C, D matrices): `TransitionMatrices` reserves `Option`
  fields. Implementation deferred.
- **Sparse derivative storage**: All matrices are dense. Sparse storage is a
  follow-up for nv > 100.
- **Parallel FD computation**: Each perturbation column requires sequential
  `step()`. Parallelism deferred.
- **Automatic differentiation** (dual numbers, enzyme): No changes to scalar
  type genericity.
- **`mjd_inverseFD`**: Inverse dynamics derivatives deferred. Forward transition
  derivatives are the primary RL/MPC deliverable.
- **`mjd_subQuat`**: Quaternion subtraction Jacobians (two 3×3 matrices for
  `d(q1 ⊖ q2)/d(q1)` and `d(q1 ⊖ q2)/d(q2)`). Not needed for transition
  derivatives — `mj_differentiate_pos` handles the full nq→nv mapping.
  Could be useful for constraint derivatives (deferred).
- **Skip-stage optimization**: MuJoCo's `mj_forwardSkip()` avoids recomputing
  position-dependent FK/collision when perturbing only velocities or controls.
  Deferred — black-box `step()` is correct and simpler. See Step 2 DEFERRED
  note.

#### Specification

**Phase → step mapping:**

| Phase | Steps | Description |
|-------|-------|-------------|
| — | 0–1 | Shared types: `TransitionMatrices`, `DerivativeConfig` |
| A | 2 | Pure FD transition derivatives |
| B | 3–7 | Analytical smooth-force velocity derivatives (`qDeriv`) |
| C | 8 | Analytical integration derivatives |
| D | 9 | Hybrid FD+analytical transition derivatives |
| — | 10–12 | Public API, validation utilities, module structure |

**Implementation plan:**

| Part | Steps | Deliverables | Independently useful? |
|------|-------|--------------|-----------------------|
| **Part 1** | 0–7 | Types, pure FD (`mjd_transition_fd`), analytical qDeriv (`mjd_smooth_vel`) | Yes — FD unblocks iLQR/DDP/MPC workflows. `mjd_smooth_vel` validates against FD. |
| **Part 2** | 8–12 | Integration derivatives, hybrid (`mjd_transition_hybrid`), public API dispatch, validation utilities, module exports | Yes — hybrid provides ~2× speedup over pure FD. |

Part 1 acceptance criteria: 1–27. Part 2 acceptance criteria: 28–41.

##### Step 0 — `TransitionMatrices` output struct

New public struct in `sim/L0/core/src/derivatives.rs`:

```rust
/// Discrete-time transition Jacobians of the simulation step function.
///
/// Given the transition `x_{t+1} = f(x_t, u_t)` where:
/// - `x = [dq, qvel, act]` is the state in tangent space (dim `2*nv + na`)
/// - `u = ctrl` is the control input (dim `nu`)
///
/// The matrices encode first-order linearization:
/// `δx_{t+1} ≈ A · δx_t + B · δu_t`
///
/// # Tangent-space representation
///
/// Position perturbations `dq` live in the tangent space of the configuration
/// manifold (dimension `nv`), not in coordinate space (dimension `nq`). This
/// avoids singularities from quaternion constraints and matches MuJoCo's
/// `mjd_transitionFD` convention. The mapping between tangent and coordinate
/// space uses `mj_integrate_pos_explicit` (tangent → coordinate) and
/// `mj_differentiate_pos` (coordinate → tangent).
///
/// # State vector layout
///
/// ```text
/// x[0..nv]           = dq   (position tangent: δqpos mapped to velocity space)
/// x[nv..2*nv]        = qvel (joint velocities)
/// x[2*nv..2*nv+na]   = act  (actuator activations)
/// ```
#[derive(Debug, Clone)]
pub struct TransitionMatrices {
    /// State transition matrix `∂x_{t+1}/∂x_t`.
    /// Dimensions: `(2*nv + na) × (2*nv + na)`.
    pub A: DMatrix<f64>,

    /// Control influence matrix `∂x_{t+1}/∂u_t`.
    /// Dimensions: `(2*nv + na) × nu`.
    pub B: DMatrix<f64>,

    /// Sensor-state Jacobian `∂s_{t+1}/∂x_t`.
    /// `None` until sensor derivatives are implemented (see Scope Exclusions).
    pub C: Option<DMatrix<f64>>,

    /// Sensor-control Jacobian `∂s_{t+1}/∂u_t`.
    /// `None` until sensor derivatives are implemented.
    pub D: Option<DMatrix<f64>>,
}

// No Default impl — dimensions depend on the model. Construct via
// mjd_transition_fd() or mjd_transition_hybrid().
```

##### Step 1 — `DerivativeConfig` configuration struct

```rust
/// Configuration for derivative computation.
///
/// # Panics
///
/// `mjd_transition_fd` (and hybrid) will panic if `eps` is non-positive,
/// non-finite, or greater than `1e-2` (large perturbations invalidate the
/// linearization assumption). Use `DerivativeConfig::default()` unless you
/// have a specific reason to change epsilon.
#[derive(Debug, Clone)]
pub struct DerivativeConfig {
    /// Perturbation magnitude for finite differences.
    /// Default: `1e-6` (centered differences).
    /// Must be in `(0, 1e-2]`. Typical range: `1e-8` to `1e-4`.
    pub eps: f64,

    /// Use centered differences (more accurate, 2x cost) vs forward differences.
    /// Centered: O(ε²) error. Forward: O(ε) error.
    /// Default: `true`.
    pub centered: bool,

    /// Use hybrid analytical+FD method (Phase D) when available.
    ///
    /// When true and the integrator supports it (Euler or ImplicitSpringDamper),
    /// velocity columns of A and simple actuator columns of B use analytical
    /// derivatives from `qDeriv`. Position columns always use FD.
    /// Falls back to pure FD for RK4 or when analytical derivatives are
    /// unavailable.
    ///
    /// Default: `true`.
    pub use_analytical: bool,
}

impl Default for DerivativeConfig {
    fn default() -> Self {
        Self { eps: 1e-6, centered: true, use_analytical: true }
    }
}
```

**Validation** (at the top of `mjd_transition_fd` and `mjd_transition_hybrid`):

```rust
assert!(config.eps.is_finite() && config.eps > 0.0 && config.eps <= 1e-2,
    "DerivativeConfig::eps must be in (0, 1e-2], got {}", config.eps);
```

##### Step 2 — Phase A: `mjd_transition_fd()` — pure FD

```rust
/// Compute finite-difference Jacobians of the simulation transition function.
///
/// Linearizes `x_{t+1} = f(x_t, u_t)` around the current state by perturbing
/// each component of state and control, stepping the simulation, and computing
/// differences. The input `data` must have a valid `forward()` result.
///
/// # Difference formulas
///
/// Centered (O(ε²) error):
///   `∂f/∂x_i ≈ (f(x + ε·e_i) − f(x − ε·e_i)) / (2·ε)`
///
/// Forward (O(ε) error):
///   `∂f/∂x_i ≈ (f(x + ε·e_i) − f(x)) / ε`
///
/// Centered differences are recommended (2x cost but ε² error vs ε error).
/// For ε = 1e-6, centered achieves ~1e-12 error vs ~1e-6 for forward.
///
/// # Cost
///
/// - Centered: `2 · (2·nv + na + nu)` calls to `step()`.
/// - Forward:  `1 + (2·nv + na + nu)` calls to `step()` (the `1` is for
///   the nominal `y_0 = f(x)` evaluation).
///
/// # Quaternion handling
///
/// Position perturbations operate in tangent space (dimension `nv`) via
/// `mj_integrate_pos_explicit()`. Output differences use
/// `mj_differentiate_pos()` to map back to tangent space.
///
/// # Contact handling
///
/// FD naturally captures contact transitions. If a perturbation causes a
/// contact to activate/deactivate, the derivative reflects this discontinuity.
///
/// # Errors
///
/// Returns `StepError` if any perturbed `step()` call fails. Relevant
/// `StepError` variants (defined at `mujoco_pipeline.rs:748`):
/// - `InvalidPosition` / `InvalidVelocity`: NaN/Inf in perturbed state
/// - `InvalidAcceleration`: singular mass matrix on perturbed configuration
/// - `CholeskyFailed`: implicit integrator's `(M + hD + h²K)` not positive
///   definite at the perturbed state
///
/// In practice these are rare — perturbations are small (ε ~ 1e-6) so the
/// perturbed state is close to the nominal.
///
/// No derivative-specific error type is needed; `StepError` from the
/// perturbed `step()` is the only failure mode. The function itself does
/// no fallible computation beyond calling `step()`.
///
/// # Thread safety
///
/// Takes `&Data` (shared reference) and immediately clones to a scratch
/// buffer. Multiple callers can compute derivatives from the same nominal
/// state concurrently. The function allocates one `Data` clone plus the
/// output matrices — no global state, no `unsafe`.
pub fn mjd_transition_fd(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<TransitionMatrices, StepError>
```

**Internal algorithm (four phases):**

Phase 0 — Save nominal state and clone scratch:

```rust
let mut scratch = data.clone();
let eps = config.eps;
let nv = model.nv;
let na = model.na;
let nx = 2 * nv + na;

let qpos_0 = data.qpos.clone();
let qvel_0 = data.qvel.clone();
let act_0 = data.act.clone();
let ctrl_0 = data.ctrl.clone();
let time_0 = data.time;
// efc_lambda is HashMap<WarmstartKey, Vec<f64>> — clone is O(n_contacts).
// This is acceptable because FD already does O(nx + nu) full step() calls.
let efc_lambda_0 = data.efc_lambda.clone();

// For forward differences, compute nominal output by stepping unperturbed:
let y_0 = if !config.centered {
    scratch.step(model)?;
    let y = extract_state(model, &scratch, &qpos_0);
    // Restore scratch to nominal for subsequent perturbations.
    // All derived fields (xpos, qM, qfrc_*, etc.) are recomputed by
    // forward() inside step(), so only input fields need restoration.
    scratch.qpos.copy_from(&qpos_0);
    scratch.qvel.copy_from(&qvel_0);
    scratch.act.copy_from(&act_0);
    scratch.ctrl.copy_from(&ctrl_0);
    scratch.time = time_0;
    scratch.efc_lambda = efc_lambda_0.clone();
    Some(y)
} else {
    None
};
```

Phase 1 — State perturbation (A matrix, nx columns), for each `i in 0..nx`:

```rust
if i < nv {
    // Position tangent: mj_integrate_pos_explicit maps dq[i]=±eps to coordinates.
    // The velocity `dq` with `dt=1.0` produces a tangent-space displacement of
    // exactly `eps` in direction `i`: qpos_out = qpos_0 ⊕ (1.0 · dq).
    // For scalar joints this is addition; for quaternion joints (Ball/Free)
    // it applies the exponential map, keeping the quaternion on the unit sphere.
    let mut dq = DVector::zeros(nv);
    dq[i] = eps;
    mj_integrate_pos_explicit(model, &mut scratch.qpos, &qpos_0, &dq, 1.0);
    scratch.qvel.copy_from(&qvel_0);
    scratch.act.copy_from(&act_0);
} else if i < 2 * nv {
    // Velocity: direct addition
    scratch.qpos.copy_from(&qpos_0);
    scratch.qvel.copy_from(&qvel_0);
    scratch.qvel[i - nv] += eps;
    scratch.act.copy_from(&act_0);
} else {
    // Activation: direct addition
    scratch.qpos.copy_from(&qpos_0);
    scratch.qvel.copy_from(&qvel_0);
    scratch.act.copy_from(&act_0);
    scratch.act[i - 2 * nv] += eps;
}
scratch.ctrl.copy_from(&ctrl_0);
scratch.time = time_0;
scratch.efc_lambda = efc_lambda_0.clone();
scratch.step(model)?;
let y_plus = extract_state(model, &scratch, &qpos_0);

// For centered: restore nominal (qpos, qvel, act, ctrl, time, efc_lambda),
// apply -eps perturbation, step to get y_minus, then:
//   A.column(i) = (y_plus - y_minus) / (2.0 * eps);
// For forward:
//   A.column(i) = (y_plus - y_0) / eps;
```

Phase 2 — Control perturbation (B matrix, nu columns): same pattern
(restore `qpos`, `qvel`, `act`, `ctrl`, `time`, `efc_lambda` to nominal,
then `scratch.ctrl[j] += eps`, step, extract, difference).

Phase 3 — `extract_state()`:

```rust
/// Extract state vector in tangent space from simulation data.
///
/// Returns a `DVector<f64>` of length `2·nv + na`:
/// - `[0..nv]`: position tangent via `mj_differentiate_pos()`
/// - `[nv..2·nv]`: `data.qvel`
/// - `[2·nv..2·nv+na]`: `data.act`
///
/// The `qpos_ref` argument is the nominal qpos used to compute the
/// tangent-space displacement: `dq = qpos ⊖ qpos_ref` where `⊖` handles
/// quaternion subtraction for Ball/Free joints.
fn extract_state(
    model: &Model,
    data: &Data,
    qpos_ref: &DVector<f64>,
) -> DVector<f64> {
    let nv = model.nv;
    let na = model.na;
    let mut x = DVector::zeros(2 * nv + na);
    // Position tangent (handles quaternion joints)
    let mut dq = DVector::zeros(nv);
    mj_differentiate_pos(model, &mut dq, qpos_ref, &data.qpos, 1.0);
    x.rows_mut(0, nv).copy_from(&dq);
    // Velocity (direct copy)
    x.rows_mut(nv, nv).copy_from(&data.qvel);
    // Activation (direct copy)
    if na > 0 {
        x.rows_mut(2 * nv, na).copy_from(&data.act);
    }
    x
}
```

**Borrow analysis:** `mjd_transition_fd` takes `&Data` (shared) and immediately
clones into `scratch`. All mutations go through `scratch`. The original `data`
is only read (for `qpos_0`, `qvel_0`, etc. and for `extract_state` with
forward differences). No aliasing issues.

**Scratch reuse:** Single clone upfront. Input state fields (`qpos`, `qvel`,
`act`, `ctrl`, `time`, `efc_lambda`) restored before each perturbation.
`step()` overwrites computed fields (`xpos`, `xquat`, `qM`, `qfrc_*`, etc.)
— no explicit restore needed for those. `time` must be restored because
`step()` calls `self.time += h` (line 3464); without restoration, successive
perturbations would step from increasingly future times.

**Warmstart:** Before each perturbed `step()`, copy nominal `efc_lambda` into
scratch. `efc_lambda` is `HashMap<WarmstartKey, Vec<f64>>` — the clone cost is
O(n_active_contacts), negligible vs. the `step()` call itself. Without this,
perturbed solves start from zero/stale warmstart, causing noisy derivatives
near contact transitions. MuJoCo does the same: `mjd_transitionFD` copies
`efc_force` before each perturbed step.

**Precondition:** The input `data` must have valid `forward()` results.
`mjd_transition_fd` does NOT call `forward()` on the input — it calls `step()`
on the scratch clone, which internally calls `forward()`. The nominal state
must already be at a consistent post-`forward()` snapshot.

**RK4 support:** Works naturally (black-box `step()` dispatch). RK4 is ~4x
costlier per column (4 forward evaluations per step).

| Integrator | FD cost (centered) |
|------------|--------------------|
| Euler | `2 · (2·nv + na + nu)` steps |
| ImplicitSpringDamper | `2 · (2·nv + na + nu)` steps |
| RungeKutta4 | `2 · (2·nv + na + nu)` steps (each 4x costlier) |

**DEFERRED: Skip-stage optimization.** MuJoCo uses `mj_forwardSkip()` to
avoid recomputing position-dependent quantities (FK, collision, inertia) when
only perturbing velocities or controls. This reduces per-column cost for
velocity and control perturbations by ~50%. CortenForge's current `step()`
calls `forward()` unconditionally (no skip mechanism). Adding skip support
is a follow-up optimization — the black-box `step()` approach is correct
and simpler.

##### Step 3 — Phase B: `qDeriv` — Analytical smooth-force velocity derivatives

`qDeriv` stores `∂(qfrc_smooth)/∂qvel` where
`qfrc_smooth = qfrc_passive + qfrc_actuator − qfrc_bias`. All three components
have analytically computable velocity derivatives.

**New Data field** (in `mujoco_pipeline.rs`, after `scratch_v_new`, ~line 1889):

```rust
/// Analytical derivative of smooth forces w.r.t. velocity: ∂(qfrc_smooth)/∂qvel.
/// Dense nv × nv matrix. Populated by `mjd_smooth_vel()`.
///
/// Components:
///   ∂(qfrc_passive)/∂qvel  = diagonal damping (+ tendon damping J^T·b·J
///                            in explicit mode only; skipped for ImplicitSpringDamper)
///   ∂(qfrc_actuator)/∂qvel = affine velocity-dependent gain/bias terms
///   −∂(qfrc_bias)/∂qvel    = −C(q,v) (Coriolis matrix)
///
/// MuJoCo equivalent: `mjData.qDeriv` (sparse nv×nv). Dense here because
/// nv < 100 for target use cases.
#[allow(non_snake_case)]
pub qDeriv: DMatrix<f64>,
```

**Per-body Jacobian scratch buffers** for `mjd_rne_vel()` chain-rule
propagation (in `mujoco_pipeline.rs`, after `body_min_inertia`, ~line 1935):

```rust
/// Scratch Jacobian ∂(cvel)/∂(qvel) per body (length `nbody`, each 6 × nv).
/// Used by `mjd_rne_vel()` for chain-rule derivative propagation.
pub deriv_Dcvel: Vec<DMatrix<f64>>,
/// Scratch Jacobian ∂(cacc)/∂(qvel) per body (length `nbody`, each 6 × nv).
pub deriv_Dcacc: Vec<DMatrix<f64>>,
/// Scratch Jacobian ∂(cfrc)/∂(qvel) per body (length `nbody`, each 6 × nv).
pub deriv_Dcfrc: Vec<DMatrix<f64>>,
```

**Initialization in `make_data()`** (~line 2309):

```rust
qDeriv: DMatrix::zeros(model.nv, model.nv),
deriv_Dcvel: vec![DMatrix::zeros(6, model.nv); model.nbody],
deriv_Dcacc: vec![DMatrix::zeros(6, model.nv); model.nbody],
deriv_Dcfrc: vec![DMatrix::zeros(6, model.nv); model.nbody],
```

**Clone:** Add to manual `impl Clone for Data`:

```rust
qDeriv: self.qDeriv.clone(),
deriv_Dcvel: self.deriv_Dcvel.clone(),
deriv_Dcacc: self.deriv_Dcacc.clone(),
deriv_Dcfrc: self.deriv_Dcfrc.clone(),
```

##### Step 4 — `mjd_passive_vel()`: Passive force velocity derivatives

```rust
/// Compute ∂(qfrc_passive)/∂qvel and add to data.qDeriv.
///
/// Per-DOF damping (all joint types):
///   qfrc_passive[i] -= damping[i] · qvel[i]
///   ⇒ ∂/∂qvel[i] = −damping[i]  (diagonal)
///
/// Tendon damping (explicit mode only):
///   qfrc_passive += J^T · (−b · J · qvel)
///   ⇒ ∂/∂qvel = −b · J^T · J   (rank-1 update per tendon)
///
/// Friction loss (tanh-smoothed) is NOT included (MuJoCo also omits it).
///
/// # Per-DOF damping source
///
/// Uses `model.implicit_damping[i]` — the canonical per-DOF damping vector
/// that merges `jnt_damping[jnt_id]` for Hinge/Slide joints and
/// `dof_damping[dof_idx]` for Ball/Free joints (see `compute_implicit_params`,
/// line 2506). This mirrors what `mj_fwd_passive()` uses: `jnt_damping` at
/// line 9496 (Hinge/Slide) and `dof_damping` at line 9514 (Ball/Free).
/// Always populated by model construction (`n_link_pendulum` calls
/// `compute_implicit_params()` at line 2987; MJCF model builder calls it
/// at `model_builder.rs:2512`).
///
/// # Implicit mode guard
///
/// In `ImplicitSpringDamper` mode, `mj_fwd_passive()` skips per-DOF
/// spring/damper forces (line 9488) and tendon spring/damper forces
/// (line 9417). The damping is instead absorbed into the implicit mass
/// matrix modification `(M + h·D + h²·K)`. This function must match:
/// per-DOF damping is included regardless of mode (needed by both Euler
/// and implicit velocity derivative formulas), but tendon damping is
/// skipped in implicit mode because tendon damping forces are not applied
/// in `mj_fwd_passive` for implicit mode.
///
/// See Step 9 for how the implicit velocity derivative formula adjusts
/// `qDeriv` via `+D` to account for the damping being in the mass matrix
/// rather than in `f_ext`.
fn mjd_passive_vel(model: &Model, data: &mut Data) {
    // Per-DOF damping: diagonal entries.
    // Included for all integrator modes — the Euler path uses this directly
    // in qDeriv; the implicit path's Step 9 adjustment adds D back to cancel
    // the −D contributed here, leaving only the actuator + bias terms in the
    // implicit formula's f_ext derivative.
    for i in 0..model.nv {
        data.qDeriv[(i, i)] += -model.implicit_damping[i];
    }

    // Tendon damping: −b · J^T · J (rank-1 outer product per tendon).
    // Skipped in implicit mode: mj_fwd_passive skips tendon damping forces
    // when implicit_mode is true (line 9417), so there is no tendon damping
    // contribution to differentiate. Including it would produce incorrect
    // qDeriv for ImplicitSpringDamper integrator.
    let implicit_mode = model.integrator == Integrator::ImplicitSpringDamper;
    if !implicit_mode {
        for t in 0..model.ntendon {
            let b = model.tendon_damping[t];
            if b <= 0.0 { continue; }
            let j = &data.ten_J[t];
            for r in 0..model.nv {
                if j[r] == 0.0 { continue; }
                for c in 0..model.nv {
                    if j[c] == 0.0 { continue; }
                    data.qDeriv[(r, c)] += -b * j[r] * j[c];
                }
            }
        }
    }
}
```

**Borrow analysis:** Reads `model.implicit_damping`, `model.tendon_damping`,
`model.integrator`, `data.ten_J`. Writes `data.qDeriv`. No aliasing — `ten_J`
and `qDeriv` are separate fields of `Data`.

**Complexity:** O(nv) diagonal + O(ntendon · nnz_J²). Tendon Jacobians are
typically very sparse (only DOFs along the tendon path), so effectively
O(nv + ntendon · depth²). In implicit mode the tendon loop is skipped entirely.

##### Step 5 — `mjd_actuator_vel()`: Actuator force velocity derivatives

```rust
/// Compute ∂(qfrc_actuator)/∂qvel and add to data.qDeriv.
///
/// For each actuator with GainType::Affine or BiasType::Affine:
///   force = gain(L, V) · input + bias(L, V)
///   ∂force/∂V = gainprm[2] · input + biasprm[2]
///
/// The velocity V maps to qvel through the transmission:
///   Joint:  V = gear · qvel[dof_adr]
///   Tendon: V = gear · J · qvel
///   Site:   V = moment^T · qvel
///
/// The force maps to qfrc through the same transmission (moment vector).
///
/// Combined: ∂qfrc/∂qvel += moment · ∂force/∂V · ∂V/∂qvel
///                        = moment · (∂force/∂V) · moment^T
///
/// Muscle actuators (GainType::Muscle) are SKIPPED here. Their velocity
/// derivatives involve piecewise FLV curve gradients and are captured via
/// FD in Phase D.
fn mjd_actuator_vel(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        if matches!(model.actuator_gaintype[i], GainType::Muscle) { continue; }
        if matches!(model.actuator_biastype[i], BiasType::Muscle) { continue; }

        let input = match model.actuator_dyntype[i] {
            ActuatorDynamics::None => data.ctrl[i],
            _ => data.act[model.actuator_act_adr[i]],
        };

        let dgain_dv = match model.actuator_gaintype[i] {
            GainType::Fixed => 0.0,
            GainType::Affine => model.actuator_gainprm[i][2],
            GainType::Muscle => continue,
        };

        let dbias_dv = match model.actuator_biastype[i] {
            BiasType::None => 0.0,
            BiasType::Affine => model.actuator_biasprm[i][2],
            BiasType::Muscle => continue,
        };

        let dforce_dv = dgain_dv * input + dbias_dv;
        if dforce_dv.abs() < 1e-30 { continue; }

        let gear = model.actuator_gear[i][0];
        let trnid = model.actuator_trnid[i][0];

        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                let dof_adr = model.jnt_dof_adr[trnid];
                // moment = gear, ∂V/∂qvel[dof] = gear
                // ∂qfrc[dof]/∂qvel[dof] += gear² · ∂force/∂V
                data.qDeriv[(dof_adr, dof_adr)] += gear * gear * dforce_dv;
            }
            ActuatorTransmission::Tendon => {
                let j = &data.ten_J[trnid];
                // moment = gear · J^T, ∂V/∂qvel = gear · J
                // ∂qfrc/∂qvel += gear² · ∂force/∂V · J^T · J
                let scale = gear * gear * dforce_dv;
                for r in 0..model.nv {
                    if j[r] == 0.0 { continue; }
                    for c in 0..model.nv {
                        if j[c] == 0.0 { continue; }
                        data.qDeriv[(r, c)] += scale * j[r] * j[c];
                    }
                }
            }
            ActuatorTransmission::Site => {
                let moment = &data.actuator_moment[i];
                // moment is nv-dim, ∂V/∂qvel = moment
                // ∂qfrc/∂qvel += ∂force/∂V · moment · moment^T
                for r in 0..model.nv {
                    if moment[r] == 0.0 { continue; }
                    for c in 0..model.nv {
                        if moment[c] == 0.0 { continue; }
                        data.qDeriv[(r, c)] += dforce_dv * moment[r] * moment[c];
                    }
                }
            }
        }
    }
}
```

**Borrow analysis:** Reads `data.ctrl`, `data.act`, `data.ten_J`,
`data.actuator_moment`. Writes `data.qDeriv`. All separate fields.

**Multi-DOF joints:** Joint transmission applies force to the first DOF only
(`qfrc_actuator[dof_adr] += gear * force`, line 8688). This is consistent with
MuJoCo — to control all DOFs of a Ball/Free joint, use multiple actuators (one
per DOF). The spec's scalar diagonal entry `qDeriv[(dof_adr, dof_adr)]` is
therefore correct for all joint types.

##### Step 6 — `mjd_rne_vel()`: Bias force velocity derivative via chain-rule RNE

```rust
/// Compute ∂(qfrc_bias)/∂qvel and SUBTRACT from data.qDeriv.
///
/// The bias force `qfrc_bias = C(q)·v + g(q)` contains:
/// 1. Gravity `g(q)`: ∂/∂qvel = 0 (position-only, excluded).
/// 2. Coriolis/centrifugal terms: quadratic in velocity.
///
/// Because the Coriolis/centrifugal force is QUADRATIC in velocity,
/// `∂(bias)/∂v` depends on the current velocity. A naive column-perturbation
/// (running RNE with `v = e_j`) would only evaluate `bias(q, e_j)`, not the
/// derivative at the actual v. MuJoCo's `mjd_rne_vel` (engine_derivative.c)
/// uses chain-rule propagation through the kinematic tree — this spec follows
/// the same approach.
///
/// # Algorithm: Chain-rule derivative propagation
///
/// Single-pass O(nbody) algorithm. Propagates per-body Jacobian matrices
/// `Dcvel[b]`, `Dcacc[b]`, `Dcfrc[b]` (each 6 × nv, dense) through the
/// kinematic tree, accumulating the effect of each DOF's velocity on every
/// body's acceleration and force. The actual `data.qvel` is used in the
/// chain-rule terms — not unit vectors.
///
/// # Complexity
///
/// O(nbody · nv) with constant factor ~36 (6×6 matrix-matrix products in
/// the backward pass dominate). For nv ~ nbody ~ 30 (humanoid): ~33k flops.
/// Dense Jacobian storage: 3 × nbody × 6 × nv floats.
/// For nv=30, nbody=31: ~45 KB. Acceptable for target models (nv < 100).
///
/// # MuJoCo equivalence
///
/// MuJoCo's `mjd_rne_vel` uses sparse per-body Jacobians (band-limited by
/// kinematic tree depth). We use dense matrices because nv < 100 and sparse
/// indexing adds implementation complexity without significant memory savings.
fn mjd_rne_vel(model: &Model, data: &mut Data)
```

**Body ordering invariant:** Bodies are indexed in topological order
(parent index < child index), guaranteed by `compute_ancestors()` (line 2544).
The forward pass `for b in 1..nbody` visits parents before children; the
backward pass `for b in (1..nbody).rev()` visits children before parents.
This is the same ordering used by `mj_rne()` (line 9113).

**Motion subspace convention:** `joint_motion_subspace()` (line 9025) returns
a `SMatrix<f64, 6, 6>` regardless of joint type. Only the first `ndof`
columns contain the motion subspace vectors (1 for Hinge/Slide, 3 for Ball,
6 for Free); remaining columns are zero. The pseudocode below uses
`S[:, d]` for `d in 0..ndof`, which naturally selects only the active columns.

**Algorithm detail:**

```
Initialize:
  Dcvel[world] = 0 (6 × nv)
  Dcacc[world] = 0 (6 × nv)

Forward pass (root to leaves), for each body b in 1..nbody:
  parent = model.body_parent[b]

  // Propagate velocity Jacobian from parent
  Dcvel[b] = Dcvel[parent]

  // For each joint j of body b (with DOFs dof_adr..dof_adr+ndof):
  for each joint j in body b:
    S = joint_motion_subspace(model, data, j)

    // Direct contribution: ∂(cvel)/∂(v_dof) += S[:, d]
    for d in 0..ndof:
      Dcvel[b][:, dof_adr + d] += S[:, d]

  // Propagate acceleration Jacobian from parent
  Dcacc[b] = Dcacc[parent]

  // For each joint j of body b:
  for each joint j in body b:
    S = joint_motion_subspace(model, data, j)

    // Recompute v_joint = S · qvel[dof_adr..dof_adr+ndof]
    v_joint = Σ_d S[:, d] · qvel[dof_adr + d]

    // The velocity-product acceleration is:
    //   cdof_dot = crossMotion(cvel[parent], v_joint)
    // Its derivative w.r.t. qvel has two terms:
    //
    // Term 1 (direct): ∂(crossMotion(cvel_parent, S·v))/∂v_dof
    //   = crossMotion(cvel_parent, S[:, d])  when dof == dof_adr+d
    for d in 0..ndof:
      Dcacc[b][:, dof_adr + d] += spatial_cross_motion(cvel[parent], S[:, d])

    // Term 2 (chain rule through cvel_parent):
    //   ∂(crossMotion(cvel_parent, v_joint))/∂(cvel_parent) · Dcvel[parent]
    //   = mjd_cross_motion_vel(v_joint) · Dcvel[parent]
    // This captures how ancestor velocity perturbations affect cdof_dot.
    let mat = mjd_cross_motion_vel(v_joint)  // 6×6
    Dcacc[b] += mat · Dcvel[parent]  // 6×nv += 6×6 · 6×nv

Backward pass — two phases (mirrors mj_rne lines 9315-9341):

  Phase 1: Compute local body force derivatives (for each body b in 1..nbody):
    // Dcfrc[b] starts at 0 (zeroed at function entry).
    // Assign the local contribution — children have NOT accumulated yet.
    Dcfrc[b] = I[b] · Dcacc[b]
             + crossForce_vel(I[b] · cvel[b]) · Dcvel[b]
             + crossForce_frc(cvel[b]) · I[b] · Dcvel[b]
    //
    // The three terms differentiate cfrc = I·a_bias + v ×* (I·v):
    //   1. I · Dcacc: inertia × acceleration derivative (linear in a_bias)
    //   2. crossForce_vel(I·v) · Dcvel: d(v ×* f)/dv with f = I·v held fixed
    //   3. crossForce_frc(v) · I · Dcvel: d(v ×* f)/df with df = I·Dcvel

  Phase 2: Accumulate to parent (leaves to root, for b in (1..nbody).rev()):
    Dcfrc[parent] += Dcfrc[b]

    // After this loop, Dcfrc[b] contains the derivative of the subtree
    // force rooted at body b, matching cfrc_bias accumulation in mj_rne.

Projection (same as standard RNE, for each joint jnt_id in 0..njnt):
  body_id = model.jnt_body[jnt_id]
  for d in 0..ndof:
    qDeriv[dof_adr + d, :] -= S[:, d]^T · Dcfrc[body_id]
```

**New helper functions** (in `derivatives.rs`):

```rust
/// Derivative of spatial_cross_motion(v, s) w.r.t. v.
/// Returns 6×6 matrix M such that d(v ×_m s)/dv = M · dv.
/// Used in chain-rule propagation of bias acceleration derivatives.
fn mjd_cross_motion_vel(s: &SpatialVector) -> Matrix6<f64>

/// Derivative of spatial_cross_force(v, f) w.r.t. v.
/// Returns 6×6 matrix M such that d(v ×_f f)/dv = M · dv.
fn mjd_cross_force_vel(f: &SpatialVector) -> Matrix6<f64>

/// Derivative of spatial_cross_force(v, f) w.r.t. f.
/// Returns 6×6 matrix M such that d(v ×_f f)/df = M · df.
fn mjd_cross_force_frc(v: &SpatialVector) -> Matrix6<f64>
```

These are small 6×6 matrices derived from the spatial cross product definitions
at `mujoco_pipeline.rs:112` and `:135`.

**Scratch buffers:** Uses `data.deriv_Dcvel`, `data.deriv_Dcacc`,
`data.deriv_Dcfrc` (per-body 6×nv Jacobian matrices, defined in Step 3).
Each is zeroed at the start of `mjd_rne_vel()` before accumulation.

**Memory:** 3 × nbody × 6 × nv × 8 bytes. For nv=30, nbody=31: ~45 KB.
For nv=100, nbody=101: ~1.5 MB. Acceptable for target use cases.

**Borrow analysis:** `mjd_rne_vel` takes `&mut Data` to write `deriv_Dcvel`,
`deriv_Dcacc`, `deriv_Dcfrc`, and `qDeriv`. Reads `data.cvel`, `data.cinert`,
`data.qvel`, `data.xpos`, `data.xquat`. All separate fields — no aliasing.

**Required visibility changes in `mujoco_pipeline.rs`:**

The following private functions must be made `pub(crate)` for use by
`derivatives.rs`:

- `mj_solve_sparse()` (line 12920) — O(nv) M⁻¹ solve
- `joint_motion_subspace()` (line 9025) — 6×6 motion subspace per joint
- `spatial_cross_motion()` (line 112) — spatial motion cross product
- `spatial_cross_force()` (line 135) — spatial force cross product

##### Step 7 — `mjd_smooth_vel()`: Combined analytical dispatch

```rust
/// Compute ∂(qfrc_smooth)/∂qvel analytically and store in data.qDeriv.
///
/// qfrc_smooth = qfrc_passive + qfrc_actuator − qfrc_bias
///
/// Populates `data.qDeriv` with three analytical contributions:
///   1. ∂(passive)/∂qvel via `mjd_passive_vel()`
///   2. ∂(actuator)/∂qvel via `mjd_actuator_vel()`
///   3. −∂(bias)/∂qvel via `mjd_rne_vel()`
///
/// # Preconditions
///
/// `data.forward(model)` must have been called (FK, velocity FK, actuation,
/// CRBA, RNE outputs needed).
pub fn mjd_smooth_vel(model: &Model, data: &mut Data) {
    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);
    mjd_actuator_vel(model, data);
    mjd_rne_vel(model, data);
}
```

**Visibility:** `mjd_smooth_vel` and `mjd_transition_fd` are `pub` (public API).
`mjd_passive_vel`, `mjd_actuator_vel`, `mjd_rne_vel`, `mjd_cross_motion_vel`,
`mjd_cross_force_vel`, `mjd_cross_force_frc`, and `extract_state` are private
to the `derivatives` module (implementation details, tested indirectly through
`mjd_smooth_vel` and FD comparison).

**Part 1 module structure:** At the end of Part 1, add to `sim/L0/core/src/lib.rs`:

```rust
/// Simulation transition derivatives (FD and analytical).
pub mod derivatives;

pub use derivatives::{
    DerivativeConfig, TransitionMatrices,
    mjd_transition_fd, mjd_smooth_vel,
};
```

Part 2 expanded these exports with `mjd_transition`, `mjd_transition_hybrid`,
`mjd_quat_integrate`, `validate_analytical_vs_fd`, `fd_convergence_check`,
and `max_relative_error`.

**Module-level documentation:** `derivatives.rs` must have a module doc comment
explaining:

1. The four-phase strategy (A/B/C/D) and which phase each function belongs to
2. The tangent-space convention (nv, not nq) with a pointer to `TransitionMatrices`
3. The relationship to MuJoCo's `mjd_*` API family
4. A "Quick start" example showing `mjd_transition_fd()` usage

**Backwards compatibility:** All Part 1 changes are **strictly additive**. No
existing public API is modified. The `derivatives` module is new; the four
`pub(crate)` visibility changes on existing private functions expose them only
within the crate (not to downstream users). Data gains four new fields
(`qDeriv`, `deriv_Dcvel`, `deriv_Dcacc`, `deriv_Dcfrc`) which are
initialized automatically in `make_data()` and cloned in `impl Clone for Data`.
No breaking changes.

**Test fixture construction:** Acceptance criteria requiring models with
actuators, tendons, or specific damping should construct test fixtures by
modifying `Model::n_link_pendulum()` output programmatically — this is the
established pattern in the existing test suite (e.g., line 14524 adds
actuators by pushing onto `model.actuator_*` fields and setting `model.nu`).
For tendons, set `model.ntendon` and push onto `tendon_type`, `tendon_damping`,
etc. For damping, set `model.jnt_damping[i]` directly and rebuild
`implicit_damping` via the model's initialization path.

##### Step 8 — Phase C: Analytical integration derivatives

```rust
/// Quaternion integration derivatives.
///
/// Given the integration formula (from `integrate_quaternion`, line 13119):
///   q_new = q_old · exp(ω · h / 2)
///
/// where `exp` is the quaternion exponential map, this function computes
/// two 3×3 Jacobians in the tangent space:
///
/// **Jacobian 1 — `dqnew_dqold`: ∂(q_new_tangent)/∂(q_old_tangent)**
///
///   This is the rotation matrix `R(−ω·h)` (inverse rotation by the
///   integration angle), corresponding to the adjoint representation:
///
///   ```text
///   dqnew_dqold = exp(−[ω·h]×)
///               = I − sin(θ)/θ · [θ̂]× + (1−cos(θ))/θ² · [θ̂]×²
///   ```
///
///   where `θ = ||ω||·h` and `θ̂ = ω·h`. For `θ < 1e-8`: `dqnew_dqold ≈ I`.
///
/// **Jacobian 2 — `dqnew_domega`: ∂(q_new_tangent)/∂ω**
///
///   This is `h` times the right Jacobian of SO(3):
///
///   ```text
///   dqnew_domega = h · J_r(ω·h)
///                = h · (I − (1−cos(θ))/θ² · [θ̂]× + (θ−sin(θ))/θ³ · [θ̂]×²)
///   ```
///
///   where `θ = ||ω||·h` and `θ̂ = ω·h`. For `θ < 1e-8`: `dqnew_domega ≈ h·I`.
///
///   The right Jacobian `J_r` relates differential angular velocity to
///   the differential change in the resulting quaternion, accounting for
///   the non-commutativity of rotations.
///
/// **Numerical stability:** When `θ < 1e-8`, both Jacobians use their
/// small-angle limits (I and h·I respectively) to avoid division by zero.
/// The Rodrigues series converges rapidly, so the 1e-8 threshold provides
/// full double-precision accuracy.
///
/// **MuJoCo equivalence:** MuJoCo's `mjd_quatIntegrate` (engine_derivative.c)
/// computes the same two Jacobians, written into two 9-element arrays.
/// Our version returns nalgebra `Matrix3` types.
///
/// **Standalone testability:** `mjd_quat_integrate` should be validated
/// against FD of the quaternion integration function itself: perturb
/// `quat` and `omega` independently, integrate, map back to tangent space,
/// and verify the Jacobians match FD columns to within `1e-6`.
pub fn mjd_quat_integrate(
    quat: &UnitQuaternion<f64>,
    omega: &Vector3<f64>,
    h: f64,
) -> (Matrix3<f64>, Matrix3<f64>)
```

```rust
/// Integration Jacobian blocks for one Euler or ImplicitSpringDamper step.
///
/// # Semi-implicit Euler
///
/// The integrator is semi-implicit (symplectic): velocity is updated FIRST,
/// then position uses the NEW velocity (see `integrate()`, line 3442):
///
///   qvel_{t+1} = qvel_t + h · qacc          (velocity update)
///   qpos_{t+1} = qpos_t ⊕ h · qvel_{t+1}   (position uses NEW velocity)
///   act_{t+1}  = act_t + h · act_dot         (or FilterExact variant)
///
/// This means `∂qpos/∂qvel` is w.r.t. the NEW velocity (qvel_{t+1}), and
/// the position-velocity block in A requires the chain rule:
///   ∂qpos_{t+1}/∂qvel_t = dqpos_dqvel · ∂qvel_{t+1}/∂qvel_t
///                        = dqpos_dqvel · (I + h · M⁻¹ · qDeriv)  [Euler]
///
/// For scalar joints (Hinge/Slide):
///   ∂qpos_{t+1}/∂qpos_t = I  (identity; scalar position is linear)
///   ∂qpos_{t+1}/∂qvel_{t+1} = h · I   (= dqpos_dqvel)
///
/// For quaternion joints (Ball/Free):
///   Uses mjd_quat_integrate() for the 3×3 blocks:
///   - dqpos_dqpos block = dqnew_dqold (from mjd_quat_integrate)
///   - dqpos_dqvel block = dqnew_domega (from mjd_quat_integrate)
///   For Ball: 3×3 block at (dof_adr, dof_adr)
///   For Free: linear part uses h·I (3×3 at dof_adr..dof_adr+3),
///             angular part uses mjd_quat_integrate (3×3 at dof_adr+3..dof_adr+6)
///
/// # ImplicitSpringDamper
///
///   (M + h·D + h²·K) · v_new = M · v_old + h · f_ext − h · K · (q − q_eq)
///   ∂v_new/∂v_old = (M + h·D + h²·K)⁻¹ · M           (integration only)
///   ∂v_new/∂q     = −(M + h·D + h²·K)⁻¹ · h · K      (spring restoring)
///
/// Note: these are the INTEGRATION Jacobians (how integration transforms
/// velocities). The FULL velocity column in the A matrix also includes
/// the force-velocity coupling via qDeriv — see Step 9 for the combined
/// formula: ∂v⁺/∂v = (M + hD + h²K)⁻¹ · (M + h·(qDeriv + D)).
///
/// **Important:** The implicit `∂v_new/∂q = −(M+hD+h²K)⁻¹ · h·K` term
/// captures how spring restoring forces couple position to next-step
/// velocity. This is NOT computed analytically here — it is captured
/// automatically by FD in the position columns of A (which always use FD).
/// `IntegrationDerivatives` only stores the velocity→velocity and
/// velocity→position blocks needed by the analytical velocity columns.
///
/// # Activation integration
///
/// Per-actuator activation derivatives depend on `ActuatorDynamics` type:
///
/// | DynType | ∂act⁺/∂act | ∂act⁺/∂act_dot | Notes |
/// |---------|------------|----------------|-------|
/// | `Filter` (Euler) | `1 − h/τ` | `h` | `act_dot = (ctrl−act)/τ`, so `∂act_dot/∂act = −1/τ`, chain: `∂act⁺/∂act = 1 + h·(−1/τ) = 1−h/τ` |
/// | `FilterExact` | `exp(−h/τ)` | `τ·(1−exp(−h/τ))` | Exact: `act⁺ = act + act_dot·τ·(1−exp(−h/τ))` |
/// | `Integrator` | `1` | `h` | `act_dot = ctrl` (no dependence on act) |
/// | `Muscle` | `1` if `act ∈ (0,1)`, `0` at boundaries | `h` | **Approximate only** — ignores `∂act_dot/∂act` from act-dependent time constants. Not consumed by hybrid path (Muscle always uses FD fallback in Step 9). |
/// | `None` | N/A | N/A | No activation state (na_i = 0) |
///
/// **Filter ∂act⁺/∂act derivation:** The combined update is:
///   `act⁺ = act + h · (ctrl − act)/τ = act · (1 − h/τ) + h·ctrl/τ`
///   So `∂act⁺/∂act = 1 − h/τ`.
///
/// For FilterExact, the exact integration gives:
///   `act⁺ = act + act_dot · τ · (1 − exp(−h/τ))`
///   `     = act + ((ctrl−act)/τ) · τ · (1 − exp(−h/τ))`
///   `     = act · exp(−h/τ) + ctrl · (1 − exp(−h/τ))`
///   So `∂act⁺/∂act = exp(−h/τ)`.
struct IntegrationDerivatives {
    /// ∂qpos_{t+1}/∂qpos_t in tangent space. nv × nv, block-diagonal per joint.
    dqpos_dqpos: DMatrix<f64>,
    /// ∂qpos_{t+1}/∂qvel_{t+1} in tangent space. nv × nv, block-diagonal.
    dqpos_dqvel: DMatrix<f64>,
    /// ∂act_{t+1}/∂act_t. na × na, diagonal.
    dact_dact: DMatrix<f64>,
    /// ∂act_{t+1}/∂act_dot. na × na, diagonal.
    /// **Currently unused** by the hybrid path — `act_dot` does not depend on
    /// `qvel` for any existing dyntype, so the activation-velocity cross term
    /// `∂act⁺/∂v = dact_dactdot · ∂act_dot/∂v = 0`. Retained for future
    /// dynamics types where `act_dot` may depend on velocity.
    dact_dactdot: DMatrix<f64>,
}

fn compute_integration_derivatives(
    model: &Model,
    data: &Data,
) -> IntegrationDerivatives
```

**Borrow analysis:** `compute_integration_derivatives` takes `&Model`, `&Data`
(shared). Returns owned struct. Pure computation, no mutation.

**Joint dispatch in `compute_integration_derivatives`:**

```
for each joint jnt_id:
  dof_adr = model.jnt_dof_adr[jnt_id]
  match model.jnt_type[jnt_id]:
    Hinge | Slide:
      dqpos_dqpos[(dof_adr, dof_adr)] = 1.0
      dqpos_dqvel[(dof_adr, dof_adr)] = h
    Ball:
      omega = qvel[dof_adr..dof_adr+3]
      quat = extract quaternion from qpos at jnt qpos_adr
      (dq_dq, dq_dw) = mjd_quat_integrate(&quat, &omega, h)
      dqpos_dqpos[dof_adr..+3, dof_adr..+3] = dq_dq
      dqpos_dqvel[dof_adr..+3, dof_adr..+3] = dq_dw
    Free:
      // Linear part (dof_adr..dof_adr+3):
      dqpos_dqpos[i, i] = 1.0  for i in dof_adr..dof_adr+3
      dqpos_dqvel[i, i] = h    for i in dof_adr..dof_adr+3
      // Angular part (dof_adr+3..dof_adr+6):
      omega = qvel[dof_adr+3..dof_adr+6]
      quat = extract quaternion from qpos at jnt qpos_adr+3
      (dq_dq, dq_dw) = mjd_quat_integrate(&quat, &omega, h)
      dqpos_dqpos[dof_adr+3..+3, dof_adr+3..+3] = dq_dq
      dqpos_dqvel[dof_adr+3..+3, dof_adr+3..+3] = dq_dw

for each actuator i:
  act_adr = model.actuator_act_adr[i]
  act_num = model.actuator_act_num[i]
  for k in 0..act_num:
    j = act_adr + k
    match model.actuator_dyntype[i]:
      Filter:
        tau = max(dynprm[0], 1e-10)
        dact_dact[(j, j)] = 1.0 - h / tau
        dact_dactdot[(j, j)] = h
      FilterExact:
        tau = max(dynprm[0], 1e-10)
        dact_dact[(j, j)] = exp(-h / tau)
        dact_dactdot[(j, j)] = tau * (1.0 - exp(-h / tau))
      Integrator:
        dact_dact[(j, j)] = 1.0
        dact_dactdot[(j, j)] = h
      Muscle:
        // Clamped to [0,1]: derivative is 0 at boundaries, 1 otherwise
        let at_boundary = data.act[j] <= 0.0 || data.act[j] >= 1.0;
        dact_dact[(j, j)] = if at_boundary { 0.0 } else { 1.0 };
        dact_dactdot[(j, j)] = if at_boundary { 0.0 } else { h };
      None:
        // No activation state — skip
```

##### Step 9 — Phase D: `mjd_transition_hybrid()` — hybrid analytical+FD

```rust
/// Compute hybrid analytical+FD transition derivatives.
///
/// # Strategy (MuJoCo approach)
///
/// The state transition A matrix has block structure:
///
/// ```text
/// A = [∂q⁺/∂q   ∂q⁺/∂v   ∂q⁺/∂act ]
///     [∂v⁺/∂q   ∂v⁺/∂v   ∂v⁺/∂act ]
///     [∂act⁺/∂q ∂act⁺/∂v ∂act⁺/∂act]
/// ```
///
/// **Position columns** (0..nv): FD. Position derivatives involve ∂FK/∂q,
/// ∂M/∂q, ∂contacts/∂q — analytically complex.
///
/// **Velocity columns** (nv..2·nv): Analytical via qDeriv.
///   Euler:    ∂v⁺/∂v = I + h · M⁻¹ · qDeriv
///   Implicit: ∂v⁺/∂v = (M + h·D + h²·K)⁻¹ · (M + h · qDeriv_adjusted)
///   ∂q⁺/∂v = dqpos_dqvel · ∂v⁺/∂v  (chain rule through integration)
///
/// **Activation columns** (2·nv..2·nv+na): Analytical.
///   ∂act⁺/∂act from IntegrationDerivatives.
///   ∂v⁺/∂act via chain: ∂force/∂act · transmission → ∂qfrc/∂act → M⁻¹
///   ∂q⁺/∂act = dqpos_dqvel · ∂v⁺/∂act
///
/// **B matrix**: Analytical for simple actuators (DynType::None with
/// Fixed/Affine gain), FD for complex (Filter/Muscle dynamics).
///
/// # Cost savings
///
/// For a 30-DOF humanoid (nv=30, nu=21, na=0):
///   Pure FD centered: 2·(60+21) = 162 step() calls
///   Hybrid centered:  2·30 = 60 step() calls + O(nv²) analytical
///   Speedup: ~2.7x
///
/// # Fallback
///
/// If `config.use_analytical == false` or integrator is RK4, falls back to
/// pure FD (Phase A).
pub fn mjd_transition_hybrid(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<TransitionMatrices, StepError>
```

**Internal algorithm — concrete steps:**

```
// === Setup ===
let h = model.timestep;
let nv = model.nv;
let na = model.na;
let nu = model.nu;
let nx = 2 * nv + na;

// 1. Populate qDeriv via analytical smooth-force derivatives.
//    Clone data to get a mutable working copy. The clone preserves:
//    - qLD_diag/qLD_L (sparse LDL factorization of M, from mj_crba)
//    - scratch_m_impl (Cholesky L of M+hD+h²K, from mj_fwd_acceleration_implicit)
//    - qM (unfactored dense mass matrix)
//    mjd_smooth_vel writes ONLY qDeriv and deriv_Dc* scratch buffers.
let mut data_work = data.clone();
mjd_smooth_vel(model, &mut data_work);

// 2. Compute integration derivatives (pure function, no mutation).
let integ = compute_integration_derivatives(model, data);

// 3. Allocate output matrices.
let mut A = DMatrix::zeros(nx, nx);
let mut B = DMatrix::zeros(nx, nu);
```

**3a. Velocity columns of A (analytical) — Euler path:**

```
// Euler: ∂v⁺/∂v = I + h · M⁻¹ · qDeriv
// Uses sparse LDL factorization from mj_crba → mj_factor_sparse.
// data_work.qLD_diag / data_work.qLD_L are copies of the original
// (mjd_smooth_vel does NOT touch mass matrix fields).

let mut dvdv = DMatrix::identity(nv, nv);
for j in 0..nv {
    // Solve M · x = qDeriv[:, j] using sparse LDL
    let mut col = data_work.qDeriv.column(j).clone_owned();
    mj_solve_sparse(&data_work.qLD_diag, &data_work.qLD_L, &mut col);
    // dvdv[:, j] += h · M⁻¹ · qDeriv[:, j]
    for i in 0..nv {
        dvdv[(i, j)] += h * col[i];
    }
}
// ∂q⁺/∂v = dqpos_dqvel · dvdv  (chain rule: position uses NEW velocity)
let dqdv = &integ.dqpos_dqvel * &dvdv;

// Fill velocity columns of A:
//   A[0..nv,    nv..2*nv] = dqdv       (position-velocity block)
//   A[nv..2*nv, nv..2*nv] = dvdv       (velocity-velocity block)
//   A[2*nv..,   nv..2*nv] = 0          (activation-velocity: no coupling)
```

**3b. Velocity columns of A (analytical) — ImplicitSpringDamper path:**

```
// Implicit: ∂v⁺/∂v = (M + hD + h²K)⁻¹ · (M + h · (qDeriv + D))
//
// Key invariant: after forward() with ImplicitSpringDamper, the field
// data.scratch_m_impl contains the Cholesky L factor (lower triangle)
// of (M + h·D + h²·K), computed by mj_fwd_acceleration_implicit()
// at line 13058: `cholesky_in_place(&mut data.scratch_m_impl)`.
// The clone data_work.scratch_m_impl preserves this factored L.
//
// The RHS matrix (M + h·(qDeriv + D)) is constructed column-by-column.
// D = model.implicit_damping (diagonal per-DOF damping vector).
// qM = data_work.qM (unfactored dense mass matrix, still valid).

let d = &model.implicit_damping;  // diagonal D
let mut dvdv = DMatrix::zeros(nv, nv);
for j in 0..nv {
    // Build RHS column j: rhs[i] = qM[(i,j)] + h * (qDeriv[(i,j)] + D[i]·δ(i,j))
    let mut rhs = DVector::zeros(nv);
    for i in 0..nv {
        rhs[i] = data_work.qM[(i, j)]
                + h * data_work.qDeriv[(i, j)]
                + if i == j { h * d[i] } else { 0.0 };
    }
    // Solve (M+hD+h²K) · x = rhs using the pre-factored Cholesky L
    cholesky_solve_in_place(&data_work.scratch_m_impl, &mut rhs);
    dvdv.column_mut(j).copy_from(&rhs);
}
// Chain rule through position integration: same as Euler path
let dqdv = &integ.dqpos_dqvel * &dvdv;
// Fill velocity columns of A (same layout as Euler)
```

**Required visibility change:** `cholesky_solve_in_place` (line 12862 in
`mujoco_pipeline.rs`) must be changed from private to `pub(crate)` so that
`derivatives.rs` can call it for the implicit hybrid path.

**4. Activation columns of A (analytical or FD):**

Activation column `k` (state index `2*nv + k`) affects the next state through:
1. The activation integration: `∂act⁺/∂act` (from `IntegrationDerivatives`)
2. The force chain: `act → force → qfrc → qacc → v⁺ → q⁺`

The force chain depends on the actuator type:

| `ActuatorDynamics` | `∂force/∂act` | Analytical? | Notes |
|-------------------|---------------|-------------|-------|
| `Filter`/`FilterExact`/`Integrator` | `gain` (from `gainprm[0]` for Fixed gain) | Yes | `input = act`, `force = gain · act + bias`, `∂force/∂act = gain` |
| `Muscle` | piecewise FLV gradient | **No — FD fallback** | Muscle FLV curves have non-trivial derivatives |
| `None` | N/A | N/A | No activation state |

```
for each actuator i with act_num > 0:
  act_adr = model.actuator_act_adr[i]
  for k in 0..act_num:
    j = act_adr + k  // state index: 2*nv + j

    if actuator is Muscle:
      // FD fallback: perturb act[j] ±eps, step, extract, difference
      // (same as Phase A position perturbation)
      continue

    // Analytical path: compute ∂force/∂act = gain at current state.
    // Inline gain computation (mirrors mj_fwd_actuation lines 8649-8698):
    let gain = match model.actuator_gaintype[i] {
        GainType::Fixed => model.actuator_gainprm[i][0],
        GainType::Affine => {
            model.actuator_gainprm[i][0]
            + model.actuator_gainprm[i][1] * data.actuator_length[i]
            + model.actuator_gainprm[i][2] * data.actuator_velocity[i]
        }
        GainType::Muscle => unreachable!("Muscle uses FD fallback"),
    };
    let moment = &data.actuator_moment[i];  // nv-dim transmission vector

    // ∂qfrc/∂act = moment · gain
    let dqfrc_dact: DVector<f64> = moment * gain;

    // ∂v⁺/∂act = h · M⁻¹ · dqfrc_dact  [Euler]
    //          = (M+hD+h²K)⁻¹ · h · dqfrc_dact  [Implicit]
    let mut dvdact = h * dqfrc_dact;
    // Solve using integrator-appropriate factorization:
    match model.integrator {
        Euler => mj_solve_sparse(
            &data_work.qLD_diag, &data_work.qLD_L, &mut dvdact),
        ImplicitSpringDamper => cholesky_solve_in_place(
            &data_work.scratch_m_impl, &mut dvdact),
        RungeKutta4 => unreachable!("hybrid path excludes RK4"),
    }

    // Fill activation column of A:
    //   A[0..nv,   2*nv+j] = dqpos_dqvel · dvdact   (position)
    //   A[nv..2*nv, 2*nv+j] = dvdact                  (velocity)
    //   A[2*nv+j,  2*nv+j] = integ.dact_dact[(j,j)]   (activation self)
```

**5. Position columns of A (FD — same as Phase A):**

Position columns always use FD because analytical `∂FK/∂q`, `∂M/∂q`, and
`∂contacts/∂q` derivatives are massive complexity (see Scope Exclusions).

```
// For each position tangent direction i in 0..nv:
//   Clone scratch from nominal data
//   Perturb: mj_integrate_pos_explicit(model, &mut scratch.qpos, &qpos_0, &dq, 1.0)
//            where dq[i] = ±eps
//   step(model) on scratch
//   extract_state → y_plus / y_minus
//   A[:, i] = (y_plus - y_minus) / (2·eps)  [centered]
```

This is identical to Phase A's position perturbation loop (Step 2). Note that
FD through `step()` already captures the full `∂x⁺/∂q` including integration
effects (`dqpos_dqpos` from `IntegrationDerivatives`), so no separate
multiplication by `dqpos_dqpos` is needed for position columns.

**6. B matrix (analytical or FD):**

B columns are analytical or FD depending on `ActuatorDynamics`:

| `ActuatorDynamics` | B column method | Formula |
|-------------------|-----------------|---------|
| `None` (no activation) | **Analytical** | `∂v⁺/∂ctrl = h · M⁻¹ · moment · gain` |
| `Filter`/`FilterExact` | **FD** | ctrl → act_dot → act⁺ → force (indirect path through next-step activation) |
| `Integrator` | **FD** | ctrl → act_dot → act⁺ (same indirect path) |
| `Muscle` | **FD** | Complex activation dynamics |

For `DynType::None` (direct actuators, no activation state):

```
for each actuator i with dyntype == None:
  // Inline gain (same as activation columns — mirrors mj_fwd_actuation):
  let gain = match model.actuator_gaintype[i] {
      GainType::Fixed => model.actuator_gainprm[i][0],
      GainType::Affine => {
          model.actuator_gainprm[i][0]
          + model.actuator_gainprm[i][1] * data.actuator_length[i]
          + model.actuator_gainprm[i][2] * data.actuator_velocity[i]
      }
      GainType::Muscle => unreachable!("Muscle has DynType::Muscle, not None"),
  };
  let moment = &data.actuator_moment[i];

  // ∂qfrc/∂ctrl = moment · gain
  let dqfrc_dctrl = moment * gain;

  // ∂v⁺/∂ctrl = h · M⁻¹ · dqfrc_dctrl  [Euler]
  //           = (M+hD+h²K)⁻¹ · h · dqfrc_dctrl  [Implicit]
  let mut dvdctrl = h * dqfrc_dctrl;
  // Dispatch to integrator-appropriate solve:
  match model.integrator {
      Euler => mj_solve_sparse(
          &data_work.qLD_diag, &data_work.qLD_L, &mut dvdctrl),
      ImplicitSpringDamper => cholesky_solve_in_place(
          &data_work.scratch_m_impl, &mut dvdctrl),
      RungeKutta4 => unreachable!("hybrid path excludes RK4"),
  };

  // Fill B column i:
  //   B[0..nv, i]    = dqpos_dqvel · dvdctrl   (position)
  //   B[nv..2*nv, i] = dvdctrl                  (velocity)
  //   B[2*nv.., i]   = 0                        (no activation)
```

For actuators with dynamics (Filter, FilterExact, Integrator, Muscle), use
FD: perturb `ctrl[i] ±eps`, step, extract, difference. The ctrl→act_dot→act
path makes one-step analytical derivatives intractable without unrolling the
activation integration.

**Position rows of A (chain rule):**

The position block of A is filled as follows:
- Position columns (0..nv): from FD (step 5 above)
- Velocity columns (nv..2*nv): `A[0..nv, nv..2*nv] = dqpos_dqvel · dvdv`
- Activation columns: `A[0..nv, 2*nv+j] = dqpos_dqvel · dvdact`

The `dqpos_dqpos` from `IntegrationDerivatives` is used in position columns
only to provide the position-position block `A[0..nv, 0..nv]` — but since
position columns use FD, `dqpos_dqpos` is NOT directly used in hybrid mode.
It remains available for future analytical position columns (if ever
implemented).

**ImplicitSpringDamper velocity columns — derivation:**

The implicit integrator solves
(see `mj_fwd_acceleration_implicit`, line 13008):

```
(M + h·D + h²·K) · v_new = M · v_old + h · f_ext − h·K·(q − q_eq)
```

where `f_ext = qfrc_applied + qfrc_actuator + qfrc_passive(friction) +
qfrc_constraint − qfrc_bias`. Crucially, in implicit mode `qfrc_passive`
does NOT contain spring/damper forces (see `mj_fwd_passive`, line 9403:
`implicit_mode` skips per-DOF springs and dampers). Those forces are
captured implicitly via the `h·D + h²·K` modification.

Taking `∂v_new/∂v_old`:

```
∂v⁺/∂v = (M + h·D + h²·K)⁻¹ · (M + h · ∂f_ext/∂v)
```

The velocity-dependent terms in `f_ext` are `qfrc_actuator` and `−qfrc_bias`
(Coriolis). The `qfrc_passive(friction)` velocity derivative is intentionally
omitted (MuJoCo also omits friction derivatives in `mjd_passive_vel`).

Now, `qDeriv = ∂(passive)/∂v + ∂(actuator)/∂v − ∂(bias)/∂v` where
`∂(passive)/∂v = −D` (diagonal damping). But in implicit mode, damping is
NOT in `f_ext` — it's in `(M + hD + h²K)`. So the `−D` term in `qDeriv`
is wrong for implicit mode. Fix: add `D` back:

```
∂f_ext/∂v = qDeriv + D = ∂(actuator)/∂v − ∂(bias)/∂v
∂v⁺/∂v = (M + h·D + h²·K)⁻¹ · (M + h · (qDeriv + D))
```

This reuses the Cholesky factorization of `(M + hD + h²K)` already computed
by `mj_fwd_acceleration_implicit()` and stored in `data.scratch_m_impl`
(lower triangle contains L factor after `cholesky_in_place`, line 13058).
The clone `data_work.scratch_m_impl` preserves this factored L. The solve
uses `cholesky_solve_in_place(&data_work.scratch_m_impl, &mut rhs_col)`
for each column of the RHS matrix.

**Tendon damping consistency:** `mjd_passive_vel` (Step 4) skips tendon
damping in implicit mode, matching `mj_fwd_passive` which also skips tendon
spring/damper forces in implicit mode (lines 9417-9430). This means `qDeriv`
contains only diagonal per-DOF damping from passive forces (plus actuator
and bias terms), and the `+D` adjustment exactly cancels the diagonal
`−D` contribution, leaving `∂f_ext/∂v = ∂(actuator)/∂v − ∂(bias)/∂v`.
The full implicit integrator (spec #13) with non-diagonal K/D matrices
would need to revisit this when tendon coupling is absorbed into the
implicit mass matrix.

##### Step 10 — Public API dispatch

```rust
/// Compute transition derivatives using the best available method.
///
/// When `config.use_analytical == true` and the integrator is Euler or
/// ImplicitSpringDamper, uses hybrid analytical+FD (Phase D). Otherwise
/// falls back to pure FD (Phase A).
pub fn mjd_transition(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<TransitionMatrices, StepError> {
    let can_analytical = config.use_analytical
        && !matches!(model.integrator, Integrator::RungeKutta4);
    if can_analytical {
        mjd_transition_hybrid(model, data, config)
    } else {
        mjd_transition_fd(model, data, config)
    }
}
```

**Convenience method on `Data`:**

```rust
impl Data {
    /// Compute transition derivatives at the current state.
    pub fn transition_derivatives(
        &self,
        model: &Model,
        config: &DerivativeConfig,
    ) -> Result<TransitionMatrices, StepError> {
        mjd_transition(model, self, config)
    }
}
```

##### Step 11 — Validation utilities

```rust
/// Compare matrices element-wise, returning max relative error and location.
/// Uses `floor` to prevent division-by-zero for near-zero entries.
pub fn max_relative_error(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    floor: f64,
) -> (f64, (usize, usize))

/// Compare FD and hybrid derivatives to validate analytical implementation.
/// Returns `(max_error_A, max_error_B)` — the max relative errors between
/// pure-FD and hybrid A/B matrices.
///
/// The caller checks the returned errors against their desired tolerance.
pub fn validate_analytical_vs_fd(
    model: &Model,
    data: &Data,
) -> Result<(f64, f64), StepError> {
    let fd = mjd_transition(model, data,
        &DerivativeConfig { use_analytical: false, ..Default::default() })?;
    let hybrid = mjd_transition(model, data,
        &DerivativeConfig { use_analytical: true, ..Default::default() })?;
    let (err_a, _) = max_relative_error(&fd.A, &hybrid.A, 1e-10);
    let (err_b, _) = max_relative_error(&fd.B, &hybrid.B, 1e-10);
    Ok((err_a, err_b))
}

/// Check FD convergence by comparing derivatives at two epsilon scales.
///
/// Computes `mjd_transition_fd` at `eps` and `eps/10` (both centered),
/// then compares the A matrices via `max_relative_error`. Returns `true`
/// if the max relative error is below `tol`.
///
/// This validates that the FD approximation is in the convergent regime:
/// if decreasing epsilon by 10x doesn't significantly change the result,
/// the derivative estimate is stable.
///
/// # Example
///
/// ```ignore
/// let converged = fd_convergence_check(&model, &data, 1e-6, 1e-3)?;
/// assert!(converged, "FD not converged at eps=1e-6");
/// ```
pub fn fd_convergence_check(
    model: &Model,
    data: &Data,
    eps: f64,
    tol: f64,
) -> Result<bool, StepError> {
    let c1 = DerivativeConfig { eps, centered: true, use_analytical: false };
    let c2 = DerivativeConfig { eps: eps / 10.0, centered: true, use_analytical: false };
    let d1 = mjd_transition_fd(model, data, &c1)?;
    let d2 = mjd_transition_fd(model, data, &c2)?;
    let (err, _) = max_relative_error(&d1.A, &d2.A, 1e-10);
    Ok(err < tol)
}
```

##### Step 12 — Module structure and exports

`derivatives.rs` already exists from Part 1. Update `sim/L0/core/src/lib.rs`
to expand the re-exports:

```rust
/// Simulation transition derivatives (FD, analytical, and hybrid).
pub mod derivatives;

pub use derivatives::{
    DerivativeConfig, TransitionMatrices,
    mjd_transition, mjd_transition_fd, mjd_transition_hybrid,
    mjd_smooth_vel, mjd_quat_integrate,
    max_relative_error, validate_analytical_vs_fd, fd_convergence_check,
};
```

**Visibility summary for Part 2 public items in `derivatives.rs`:**

| Item | Visibility | Rationale |
|------|-----------|-----------|
| `mjd_transition` | `pub` | Primary API entry point |
| `mjd_transition_hybrid` | `pub` | Advanced users who want explicit hybrid |
| `mjd_quat_integrate` | `pub` | Useful standalone for SO(3) tooling |
| `max_relative_error` | `pub` | Validation utility for downstream tests |
| `validate_analytical_vs_fd` | `pub` | One-call validation |
| `fd_convergence_check` | `pub` | One-call convergence check |
| `IntegrationDerivatives` | private | Internal struct, not part of API |
| `compute_integration_derivatives` | private | Internal, consumed by hybrid |
| `Data::transition_derivatives` | `pub` | Convenience method on Data |

#### Acceptance Criteria

**Phase A — FD correctness:**

1. **A matrix dimensions**: For `Model::n_link_pendulum(3, 1.0, 0.1)` (nv=3,
   na=0, nu=0), `mjd_transition_fd()` returns A with shape `6×6`, B with shape
   `6×0`. No panic.
2. **Centered vs forward convergence**: For the same 3-link pendulum, centered
   FD (ε=1e-6) and forward FD (ε=1e-6) produce A matrices that agree to within
   `1e-4` relative error, demonstrating O(ε²) vs O(ε).
3. **B matrix structure**: For a 3-link pendulum with 3 torque actuators (nu=3),
   B has shape `6×3`. The velocity rows (`B[3..6, :]`) have entries
   ≈ `h · M⁻¹ · moment` (to within `1e-4`). The position rows (`B[0..3, :]`)
   have entries ≈ `h² · M⁻¹ · moment` (semi-implicit Euler: position uses the
   updated velocity, so `∂q⁺/∂ctrl = h · ∂v⁺/∂ctrl = h² · M⁻¹ · moment`).
4. **Quaternion handling**: For a model with a Ball joint (nq=4, nv=3) and a
   Free joint (nq=7, nv=6), A dimensions are `(2·nv+na) × (2·nv+na)` (NOT
   `2·nq+na`). After each perturbation, quaternion components of `scratch.qpos`
   remain unit-norm to within `1e-14`.
5. **Activation derivatives**: For a model with 2 actuators using
   `ActuatorDynamics::Filter` (na=2), the activation block of A
   (`A[2*nv..2*nv+2, 2*nv..2*nv+2]`) captures the filter time constant:
   diagonal ≈ `1 − h/τ` (to within `1e-4`).
6. **Contact sensitivity**: For a sphere (r=0.1, m=1.0) resting on a ground
   plane, perturbing position below the surface produces a non-zero
   `∂qvel⁺/∂qpos` derivative reflecting contact stiffness from `solref`.
   The magnitude is > 0 (contact active) and proportional to `solref[0]`.
7. **Integrator coverage**: `mjd_transition_fd()` returns `Ok(...)` for all
   three integrators (Euler, ImplicitSpringDamper, RungeKutta4) on the 3-link
   pendulum. RK4 A matrix differs from Euler A by more than `1e-4` (higher
   order captures different dynamics).
8. **FD convergence**: For the 3-link pendulum, computing `mjd_transition_fd`
   with `eps = 1e-6` and `eps = 1e-7` (both centered) produces A matrices
   that agree to within `1e-3` relative error, confirming the derivative is
   epsilon-independent in the convergent regime.
9. **Zero-control baseline**: For nu=0, B has 0 columns. No panic, no
   out-of-bounds access.
10. **Zero-activation baseline**: For na=0, state dimension is `2·nv`. A is
    `(2*nv) × (2*nv)`. No panic.
11. **extract_state tangent correctness**: For a model with a Free joint
    (nq=7, nv=6), `extract_state` returns a vector of length `2*nv` (not
    `2*nq`). The position tangent block `x[0..nv]` is computed via
    `mj_differentiate_pos` (not raw qpos subtraction).

**Phase B — Analytical qDeriv:**

12. **Passive damping diagonal**: For a 3-link pendulum with
    `damping = [0.5, 1.0, 1.5]`, `mjd_passive_vel()` sets
    `qDeriv[(i,i)] = −damping[i]` exactly. Off-diagonal entries from passive
    forces are zero (no tendons).
13. **Tendon damping off-diagonal**: For a model with one tendon spanning 2 DOFs
    (J = [j0, j1, 0, ...]) and damping `b`, `mjd_passive_vel()` produces
    `qDeriv[(r,c)] = −b · J[r] · J[c]` for all r,c. Verified against FD of
    `qfrc_passive` w.r.t. `qvel` to within `1e-8`.
14. **Actuator velocity derivative**: For a model with one Affine actuator
    (gainprm[2] = g_v, biasprm[2] = b_v, Joint transmission, gear = g),
    `mjd_actuator_vel()` adds `g² · (g_v · input + b_v)` to
    `qDeriv[(dof, dof)]`. Verified against FD of `qfrc_actuator` w.r.t.
    `qvel` to within `1e-8`.
15. **Bias force velocity derivative**: For a 2-link pendulum at a
    non-equilibrium configuration (`qpos = [0.3, 0.7]`, `qvel = [1.0, 0.5]`),
    `mjd_rne_vel()` produces `∂(qfrc_bias)/∂qvel` matching FD of
    `qfrc_bias` w.r.t. `qvel` (column-by-column, ε=1e-7) to within `1e-6`.
16. **qDeriv combined**: `mjd_smooth_vel()` zeroes `qDeriv` before accumulating.
    For a model with dampers, tendons, and actuators, the combined `qDeriv`
    matches FD of `(qfrc_passive + qfrc_actuator − qfrc_bias)` w.r.t. `qvel`
    to within `1e-5`.
17. **Scratch buffer isolation**: `deriv_Dcvel`, `deriv_Dcacc`, `deriv_Dcfrc`
    are NOT aliased with `data.cvel`, `data.cacc_bias`, `data.cfrc_bias`.
    Calling `mjd_rne_vel()` does not corrupt the actual body velocities/forces.
    Verified by comparing `data.cvel` before and after.

**Multi-DOF joint coverage:**

18. **Ball/Free joint FD**: For a model with a Free joint (nv=6, nq=7),
    `mjd_transition_fd()` produces a `12×12` A matrix. The position-velocity
    block `A[0..6, 6..12]` is approximately `h · I₆` (identity scaled by
    timestep, to within `1e-3`), confirming tangent-space derivatives handle
    all 6 DOFs correctly.
19. **Ball joint Coriolis**: For a model with a Ball joint (nv=3), `mjd_rne_vel()`
    produces a 3×3 Coriolis sub-matrix. At `qpos` giving a non-trivial
    orientation, the Coriolis matrix is nonzero (gyroscopic terms). Matches
    FD of `qfrc_bias` w.r.t. `qvel` to within `1e-6`.

**Part 1 infrastructure:**

20. **Config validation**: `mjd_transition_fd()` panics when called with
    `eps = 0.0`, `eps = -1.0`, `eps = f64::NAN`, or `eps = 0.1` (> 1e-2).
    No panic with `eps = 1e-6` (default).
21. **Warmstart copying**: Perturbed FD solves use nominal warmstart.
    `solver_niter` for perturbed steps ≤ 2× nominal `solver_niter`.
22. **Scratch state restoration**: After `mjd_transition_fd()` completes,
    the original `data` is unmodified (`&Data` borrow). Internally, every
    perturbed `step()` starts from identical nominal state: `qpos`, `qvel`,
    `act`, `ctrl`, `time`, and `efc_lambda` are all restored before each
    perturbation. Verified by checking that centered FD at `eps=1e-6`
    produces identical A matrices when called twice on the same `(model, data)`.
23. **Tendon damping implicit guard**: For a model with one tendon (damping
    `b > 0`) and `Integrator::ImplicitSpringDamper`, `mjd_passive_vel()`
    does NOT add tendon damping terms to `qDeriv` (tendon damping is skipped
    in implicit mode by `mj_fwd_passive`, line 9417). The diagonal per-DOF
    damping entries are still present.
24. **Performance**: FD derivatives for `Model::n_link_pendulum(6, 1.0, 0.1)`
    (nv=6, nu=0, na=0) complete in < 10ms (wall clock, release mode).
25. **`cargo clippy -p sim-core -- -D warnings`** passes clean.
26. **All existing sim-core tests pass unchanged** (additive module, no behavior
    changes to existing code).
27. **At least 22 new tests** in `sim/L0/tests/integration/derivatives.rs`
    covering Part 1 criteria (1–23).

---

**Part 2 acceptance criteria (Steps 8–12):**

**Phase C — Integration derivatives:**

28. **mjd_quat_integrate standalone**: For a non-trivial quaternion and angular
    velocity (`ω = [1.0, 0.5, -0.3]`, `h = 0.001`), the two Jacobians from
    `mjd_quat_integrate` match FD of the quaternion integration function
    (perturb `quat` tangent and `omega` independently, integrate, map to
    tangent, compute FD columns) to within `1e-6`.
29. **Integration derivatives scalar joints**: For a 3-link hinge pendulum,
    `compute_integration_derivatives` returns `dqpos_dqpos ≈ I₃` and
    `dqpos_dqvel ≈ h·I₃`. Verified to machine precision.
30. **Integration derivatives Ball joint**: For a model with a Ball joint,
    `dqpos_dqvel` block is approximately `h·I₃` at zero angular velocity
    and deviates from `h·I₃` at `ω = [2.0, 1.0, -1.5]` (right Jacobian
    correction). Both cases match FD to within `1e-5`.
31. **Activation integration FilterExact**: For `ActuatorDynamics::FilterExact`
    with `τ = 0.05`, `h = 0.001`: `dact_dact[(j,j)] ≈ exp(-h/τ) ≈ 0.9802`.
    Verified against FD of the activation integration to within `1e-6`.

**Phase D — Hybrid:**

32. **Hybrid vs FD agreement**: `validate_analytical_vs_fd()` returns
    `max_error_A < 1e-3` and `max_error_B < 1e-3` for a 3-link pendulum with
    torque actuators (Euler integrator).
33. **Hybrid velocity columns**: For a damped 3-link pendulum (damping=1.0),
    velocity columns of A from hybrid match pure FD velocity columns to within
    `1e-4` relative error (element-wise via `max_relative_error`).
34. **Hybrid correctness with B matrix**: For `Model::n_link_pendulum(10, 1.0, 0.1)`
    with 10 torque actuators added via `add_torque_actuators(&mut model, 10)`
    (nu=10, na=0, all `DynType::None` Joint actuators), hybrid A and B matrices
    match pure FD A and B to within `1e-3`. This confirms both position-FD and
    velocity/control-analytical paths produce consistent results.
35. **ImplicitSpringDamper analytical**: For a stiff spring system
    (stiffness=1000, damping=10) with `Integrator::ImplicitSpringDamper`, hybrid
    velocity derivatives use the `(M + hD + h²K)⁻¹` correction. The velocity
    block of A matches pure FD to within `1e-3`.
36. **Activation columns analytical**: For a model with `ActuatorDynamics::Filter`
    (na=2), hybrid computes activation columns of A analytically:
    `∂v⁺/∂act` via `h · M⁻¹ · moment · gain`. Matches FD to within `1e-3`.
37. **mjd_transition dispatch**: `mjd_transition()` with `use_analytical=true`
    and Euler returns the same result as `mjd_transition_hybrid()`. With
    `use_analytical=false` or RK4, returns same as `mjd_transition_fd()`.
38. **Data convenience method**: `data.transition_derivatives(model, config)`
    returns the same result as `mjd_transition(model, &data, config)`.

**Part 2 infrastructure:**

39. **FD convergence utility**: `fd_convergence_check()` returns `true` for
    epsilon in `[1e-4, 1e-8]` with tolerance `1e-3`.
40. **`cargo clippy -p sim-core -- -D warnings`** still passes clean.
41. **At least 14 additional tests** covering Part 2 criteria (28–39).

#### Files

**Part 1 (Steps 0–7):**

| File | Action | Description |
|------|--------|-------------|
| `sim/L0/core/src/derivatives.rs` | **new** | `TransitionMatrices`, `DerivativeConfig`, `mjd_transition_fd()`, `extract_state()`, `mjd_smooth_vel()`, `mjd_passive_vel()`, `mjd_actuator_vel()`, `mjd_rne_vel()`, `mjd_cross_motion_vel()`, `mjd_cross_force_vel()`, `mjd_cross_force_frc()` |
| `sim/L0/core/src/lib.rs` | **modify** | Add `pub mod derivatives;` and re-export `DerivativeConfig`, `TransitionMatrices`, `mjd_transition_fd`, `mjd_smooth_vel` |
| `sim/L0/core/src/mujoco_pipeline.rs` | **modify** | Add `qDeriv: DMatrix<f64>` and `deriv_Dcvel`/`deriv_Dcacc`/`deriv_Dcfrc` per-body Jacobian fields to `Data` (~line 1889). Initialize in `make_data()` (~line 2309). Add to `impl Clone for Data` (~line 1939). Change visibility of `mj_solve_sparse`, `joint_motion_subspace`, `spatial_cross_motion`, `spatial_cross_force` from private to `pub(crate)` |
| `sim/L0/tests/integration/mod.rs` | **modify** | Add `mod derivatives;` |
| `sim/L0/tests/integration/derivatives.rs` | **new** | 20+ integration tests covering Part 1 acceptance criteria (1–19) |

**Part 2 (Steps 8–12):**

| File | Action | Description |
|------|--------|-------------|
| `sim/L0/core/src/derivatives.rs` | **modify** | Add `IntegrationDerivatives`, `mjd_transition_hybrid()`, `mjd_transition()`, `compute_integration_derivatives()`, `mjd_quat_integrate()`, `max_relative_error()`, `validate_analytical_vs_fd()`, `fd_convergence_check()`, `impl Data { fn transition_derivatives() }` |
| `sim/L0/core/src/mujoco_pipeline.rs` | **modify** | Change visibility of `cholesky_solve_in_place` from private to `pub(crate)` (needed by hybrid ImplicitSpringDamper path) |
| `sim/L0/core/src/lib.rs` | **modify** | Expand re-exports: add `mjd_transition`, `mjd_transition_hybrid`, `mjd_quat_integrate`, `max_relative_error`, `validate_analytical_vs_fd`, `fd_convergence_check` |
| `sim/L0/tests/integration/derivatives.rs` | **modify** | Add tests for Part 2 acceptance criteria (28–39) |

---

### 13. Full Implicit Integrator
**Status:** ✅ Complete | **Effort:** XL | **Prerequisites:** #12 (Derivatives)

#### Current State

`Integrator::ImplicitSpringDamper` (`:733–741`) implements a diagonal-only implicit
scheme that absorbs per-DOF joint stiffness K and damping D into the mass matrix:

```
(M + h*D_diag + h²*K_diag) v_new = M*v_old + h*f_ext − h*K_diag*(q − q_eq)
```

- K, D stored as `DVector<f64>` — `Model::implicit_stiffness` (`:1210`),
  `Model::implicit_damping` (`:1213`), `Model::implicit_springref` (`:1216`).
- Tendon spring/damper forces are **skipped** in implicit mode
  (`mj_fwd_passive()` `:9454–9467`) because their J^T coupling is non-diagonal.
- The `ImplicitFast` MJCF variant is parsed (`types.rs:27`) but maps to the
  same diagonal solver (`model_builder.rs:616–618`).

**What MuJoCo actually does:** MuJoCo's implicit integrators (`implicit` and
`implicitfast`) do not construct a stiffness matrix K. They are implicit **only
in velocity-dependent forces**. Spring forces (position-dependent) remain
explicit — already evaluated in `qfrc_passive` at the current `qpos`. The system
solved is:

```
(M − h·D) · qacc = qfrc_smooth + qfrc_applied + qfrc_constraint
```

where `qfrc_smooth = qfrc_passive + qfrc_actuator − qfrc_bias` (note: `qfrc_applied`
is NOT part of `qfrc_smooth` — it is a user-set external force with
`∂qfrc_applied/∂qvel = 0`, added separately to the RHS).

D = `∂(qfrc_smooth)/∂(qvel)` is the **full analytical velocity-derivative
Jacobian** (`qDeriv`), assembled by `mjd_smooth_vel()` (derivatives.rs:972) from:

1. **DOF damping** (diagonal): `D[i,i] += −dof_damping[i]`
2. **Tendon damping** (off-diagonal via J^T B J): `D += −b_t · J_ten^T · J_ten`
3. **Actuator velocity derivatives** (off-diagonal via J^T B J):
   `D += dF_act/dV · J_act^T · J_act`
4. **Coriolis/centripetal derivatives** (full, asymmetric):
   `D += −∂(qfrc_bias)/∂(qvel)` via recursive Newton-Euler chain-rule
5. **Fluid drag derivatives** (6×6 body-level): drag-velocity Jacobian per body.
   *Not yet implemented in CortenForge — fluid drag computation exists
   (`Model::wind` `:1153`, `density` `:1157`, `viscosity` `:1159`) but
   `mjd_passive_vel` does not compute drag-velocity Jacobians. This is a known
   gap — acceptable because fluid drag is rare in RL workloads. Can be added
   later as a standalone sub-task.*

The two variants differ in:

| | `implicit` | `implicitfast` |
|---|---|---|
| Coriolis derivatives | included | **skipped** |
| D symmetry | asymmetric | symmetrized: `D ← (D + D^T)/2` |
| Factorization | LU | Cholesky (cheaper, symmetric) |
| MuJoCo storage | `qLU` (nD pattern) | `qH` (nC pattern) |

**Why explicit damping + D correction is NOT double-counting:** Both
`qfrc_passive` (RHS) and `qDeriv` (LHS) include damping, but they play
mathematically distinct roles. The implicit scheme is a linearized
backward-Euler step:

```
We want:   M · a_new = f(v_old + h · a_new)        (implicit in velocity)
Taylor:    f(v_old + h·a) ≈ f(v_old) + h·(∂f/∂v)·a = f(v_old) + h·D·a
Rearrange: (M − h·D) · a = f(v_old)
```

The RHS `f(v_old)` is the force at the *current* velocity — it includes the
explicit damping term `−b·v`. The LHS `−h·D` is the *derivative* of that force
w.r.t. velocity — it includes `−b` on the diagonal. These are the function
value and its Jacobian respectively; both are needed for Newton-style
linearization. MuJoCo confirms this: `mj_passive()` computes damping into
`qfrc_passive` unconditionally (no integrator gate), and `mjd_passive_vel`
independently adds `−dof_damping` to `qDeriv`.

#### Objective

Replace the diagonal `(M + h*D + h²*K)` scheme with MuJoCo-conformant implicit
integration using the full velocity-derivative Jacobian `qDeriv`. Implement both
`implicitfast` (symmetric, Cholesky) and `implicit` (asymmetric, LU) variants.

**Phasing:** Implement in two sub-phases:
- **Phase A:** `implicitfast` — symmetric D, dense Cholesky factorization.
- **Phase B:** `implicit` — asymmetric D, dense LU factorization.

#### Specification

##### Sub-task 13.A: `Integrator::ImplicitFast`

**A.1. Extend `Integrator` enum** (`mujoco_pipeline.rs:733`)

Add `ImplicitFast` and `Implicit` variants alongside existing
`ImplicitSpringDamper`. Both `Integrator` and `StepError` should carry
`#[non_exhaustive]` per STANDARDS.md (enums that may grow):

```rust
#[non_exhaustive]
pub enum Integrator {
    #[default]
    Euler,
    RungeKutta4,
    ImplicitSpringDamper,  // Keep for backward compat (diagonal-only)
    Implicit,              // Full asymmetric D, LU factorization
    ImplicitFast,          // Symmetric D (no Coriolis), Cholesky factorization
}
```

Retain `ImplicitSpringDamper` as a distinct variant so existing tests and models
that depend on the diagonal-only behavior continue to work without any change.

Update `model_builder.rs:616–618` to map `MjcfIntegrator::ImplicitFast` →
`Integrator::ImplicitFast` (currently both map to `ImplicitSpringDamper`).

**A.2. Modify `mj_fwd_passive()` to restore tendon spring/damper forces**
(`mujoco_pipeline.rs:9437`)

The `implicit_mode` flag (`:9440`) currently gates **all** tendon spring and damper
forces. In the new implicit variants, all passive forces are computed explicitly
into `qfrc_passive` — matching MuJoCo, where `mj_passive()` has no integrator
gate. The implicit correction comes entirely from the `(M − h·D)` matrix on the
LHS.

The existing `implicit_mode` check (`:9440`) is:

```rust
let implicit_mode = model.integrator == Integrator::ImplicitSpringDamper;
```

This is already correct — `implicit_mode` is true only for the legacy diagonal
variant, not for the new `Implicit`/`ImplicitFast`. So the joint visitor
(`PassiveForceVisitor::visit_1dof_joint` `:9525`) already computes spring and
damper forces for the new variants.

The tendon loop conditional (`:9454`) also correctly gates on `implicit_mode`:

```rust
if !implicit_mode {
    // spring + damper computed
}
```

Since `implicit_mode` is false for `ImplicitFast`/`Implicit`, tendon springs and
dampers are computed. **No code change needed here** — the existing gate is
already correct for the new variants.

**A.3. Fix `mjd_passive_vel` tendon-derivative gate** (`derivatives.rs:484`)

Currently `mjd_passive_vel` skips tendon damping derivatives when
`implicit_mode` is true (`:484–504`). This was correct for the diagonal-only
scheme (tendon damping forces were skipped, so there's nothing to differentiate).
For the new implicit variants, tendon damping forces ARE computed explicitly AND
their derivatives must be included in D.

The current condition is:

```rust
let implicit_mode = model.integrator == Integrator::ImplicitSpringDamper;
if !implicit_mode { ... tendon damping J^T B J ... }
```

This is already correct for the new variants — `implicit_mode` is false for
`ImplicitFast`/`Implicit`, so tendon damping J^T B J terms ARE accumulated.
**No code change needed** — the condition works as-is because the new enum
variants don't match `ImplicitSpringDamper`.

**A.4. Implement `mj_fwd_acceleration_implicitfast()`**
(`mujoco_pipeline.rs`, new function near `:13008`)

```rust
fn mj_fwd_acceleration_implicitfast(model: &Model, data: &mut Data) -> Result<(), StepError> {
    let h = model.timestep;

    // Step 1: Assemble qDeriv = ∂(qfrc_smooth)/∂(qvel)
    //   Components: DOF damping + tendon damping J^T B J + actuator vel derivatives
    //   Coriolis terms SKIPPED for implicitfast.
    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);    // DOF damping (diag) + tendon damping (J^T B J)
    mjd_actuator_vel(model, data);   // Actuator velocity derivatives (J^T B J)
    // mjd_rne_vel() intentionally omitted — this is what makes it "fast"

    // Step 2: Symmetrize D ← (D + D^T) / 2
    //   Without Coriolis, D is nearly symmetric (tendon/actuator J^T B J terms are
    //   symmetric by construction). Symmetrization cleans up any residual asymmetry
    //   and guarantees Cholesky applicability.
    for i in 0..model.nv {
        for j in (i + 1)..model.nv {
            let avg = 0.5 * (data.qDeriv[(i, j)] + data.qDeriv[(j, i)]);
            data.qDeriv[(i, j)] = avg;
            data.qDeriv[(j, i)] = avg;
        }
    }

    // Step 3: Form M_hat = M − h·D into scratch_m_impl
    //   M is SPD (from CRBA). For well-posed models (damping/drag forces oppose
    //   velocity), D is NSD and M − h·D is SPD, guaranteeing Cholesky succeeds.
    //   However, actuators with positive velocity feedback (dforce_dv > 0) can
    //   make D non-NSD, in which case Cholesky may fail — same as MuJoCo.
    //   See Known Approximations #7.
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        for j in 0..model.nv {
            data.scratch_m_impl[(i, j)] -= h * data.qDeriv[(i, j)];
        }
    }

    // Step 4: RHS = qfrc_smooth + qfrc_applied + qfrc_constraint
    //   qfrc_smooth = qfrc_passive + qfrc_actuator − qfrc_bias  (velocity-dependent)
    //   qfrc_applied is user-set external force (∂/∂qvel = 0, not part of qfrc_smooth)
    //   qfrc_passive includes ALL spring/damper/friction forces (explicit).
    data.scratch_rhs.copy_from(&data.qfrc_applied);
    data.scratch_rhs += &data.qfrc_actuator;
    data.scratch_rhs += &data.qfrc_passive;
    data.scratch_rhs += &data.qfrc_constraint;
    data.scratch_rhs -= &data.qfrc_bias;

    // Step 5: Solve (M − h·D) · qacc = rhs via dense Cholesky
    //   After this, scratch_m_impl holds the Cholesky factors (L where M−hD = L·L^T).
    //   These persist in data and are reused by mjd_transition_hybrid (D.4) for
    //   derivative column solves — the derivative pass clones data, inheriting
    //   the factored scratch_m_impl.
    cholesky_in_place(&mut data.scratch_m_impl)?;
    data.qacc.copy_from(&data.scratch_rhs);
    cholesky_solve_in_place(&data.scratch_m_impl, &mut data.qacc);

    Ok(())
}
```

**Data flow note:** This function writes `data.qDeriv` (forward-pass D, without
Coriolis) and `data.scratch_m_impl` (Cholesky factors of `M − h·D`). Both
persist in `data` after return. The derivative pass (`mjd_transition_hybrid`,
D.4) clones `data` into `data_work`, then calls `mjd_smooth_vel` which
overwrites `data_work.qDeriv` with the **full** D (including Coriolis via
`mjd_rne_vel`). However, `data_work.scratch_m_impl` is NOT overwritten — it
retains the forward-pass Cholesky factors. This means the derivative pass uses
`(M − h·D_fast)⁻¹ · D_full` — mixing the fast LHS with the full RHS. See
KA#8 for analysis.

Unlike the legacy `ImplicitSpringDamper` (which solves for `v_new` directly
and back-computes `qacc = (v_new − v_old)/h`), both new variants solve for
`qacc` directly. The `scratch_v_new` buffer (`:1888`) is NOT used by the new
variants. Velocity integration (`qvel += h * qacc`) is handled by the
standard path in `Data::integrate()` (`:3474`).

**Key design decisions:**
- **Dense Cholesky** (not sparse L^T D L): The existing codebase uses dense
  `DMatrix<f64>` for qM (`:1763`), qDeriv (`:1937`), and all scratch buffers.
  The sparse L^T D L path (`mj_factor_sparse` `:12899`) operates on the tree
  topology of M but cannot handle the **filled-in** pattern from D (tendon/actuator
  coupling adds entries outside the tree envelope). Dense Cholesky on the
  `scratch_m_impl` matrix is correct and efficient for the target nv < 100 range.
- **qacc-only output:** The function computes `qacc` but does NOT update `qvel`.
  Velocity integration (`qvel += h * qacc`) is handled by the existing Euler
  integration path in `Data::integrate()` (`:3474`), avoiding special-case
  dispatch. Position integration then uses the updated velocity (semi-implicit).

**A.5. Wire dispatch** (`mujoco_pipeline.rs`)

The `mj_fwd_acceleration` wrapper (`:12797`) already has an `nv == 0` early
return (`:12798`). The new functions are called through this wrapper, so they
do NOT need their own `nv == 0` guard.

All three matches below are **exhaustive** (no wildcard `_`) and will fail
compilation without the new arms. These are the `mujoco_pipeline.rs` counterpart
to the `derivatives.rs` matches documented in D.4.

Forward acceleration dispatch (`:12802`):

```rust
match model.integrator {
    Integrator::ImplicitSpringDamper => mj_fwd_acceleration_implicit(model, data),
    Integrator::ImplicitFast => mj_fwd_acceleration_implicitfast(model, data),
    Integrator::Implicit => mj_fwd_acceleration_implicit_full(model, data),
    Integrator::Euler | Integrator::RungeKutta4 => {
        mj_fwd_acceleration_explicit(model, data);
        Ok(())
    }
}
```

Velocity integration dispatch (`:3474`): `ImplicitFast` and `Implicit` compute
`qacc` only, so velocity is updated via the standard Euler path:

```rust
match model.integrator {
    Integrator::Euler | Integrator::ImplicitFast | Integrator::Implicit => {
        for i in 0..model.nv {
            self.qvel[i] += self.qacc[i] * h;
        }
    }
    Integrator::ImplicitSpringDamper => {
        // Velocity already updated by mj_fwd_acceleration_implicit
        // (legacy path solves for v_new directly, not qacc)
    }
    Integrator::RungeKutta4 => unreachable!(),
}
```

**A.6. Wire `step()` dispatch** (`mujoco_pipeline.rs:3303`)

Add `ImplicitFast` and `Implicit` alongside Euler/ImplicitSpringDamper in the
non-RK4 branch:

```rust
Integrator::Euler | Integrator::ImplicitSpringDamper
    | Integrator::ImplicitFast | Integrator::Implicit => {
    self.forward(model)?;
    mj_check_acc(model, self)?;
    self.integrate(model);
}
```

##### Sub-task 13.B: `Integrator::Implicit` (full asymmetric)

**B.1. Implement `mj_fwd_acceleration_implicit_full()`**

Same structure as `mj_fwd_acceleration_implicitfast` with three differences:
- **Include** `mjd_rne_vel(model, data)` (Coriolis derivatives)
- **No symmetrization** — D is asymmetric
- **LU factorization** instead of Cholesky

```rust
fn mj_fwd_acceleration_implicit_full(model: &Model, data: &mut Data) -> Result<(), StepError> {
    let h = model.timestep;

    // Step 1: Assemble qDeriv = ∂(qfrc_smooth)/∂(qvel) — ALL components
    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);
    mjd_actuator_vel(model, data);
    mjd_rne_vel(model, data);  // Full Coriolis — O(nv² · nbody)

    // Step 2: No symmetrization — D is asymmetric (Coriolis terms break symmetry)

    // Step 3: Form M_hat = M − h·D into scratch_m_impl
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        for j in 0..model.nv {
            data.scratch_m_impl[(i, j)] -= h * data.qDeriv[(i, j)];
        }
    }

    // Step 4: RHS = qfrc_smooth + qfrc_applied + qfrc_constraint
    data.scratch_rhs.copy_from(&data.qfrc_applied);
    data.scratch_rhs += &data.qfrc_actuator;
    data.scratch_rhs += &data.qfrc_passive;
    data.scratch_rhs += &data.qfrc_constraint;
    data.scratch_rhs -= &data.qfrc_bias;

    // Step 5: Factor (M − h·D) = P·L·U, then solve for qacc
    //   Factors persist in scratch_m_impl + scratch_lu_piv for derivative reuse (D.4).
    //   The derivative pass clones data, inheriting the factored scratch_m_impl and
    //   scratch_lu_piv, then calls lu_solve_factored for each derivative column.
    lu_factor_in_place(&mut data.scratch_m_impl, &mut data.scratch_lu_piv)?;
    data.qacc.copy_from(&data.scratch_rhs);
    lu_solve_factored(&data.scratch_m_impl, &data.scratch_lu_piv, &mut data.qacc);

    Ok(())
}
```

**B.2. Implement LU factorization and solve** (new functions in `mujoco_pipeline.rs`)

Split into two functions so the derivative pass (D.4) can reuse the factored
matrix for multiple solves:

```rust
/// Factor A = P·L·U in place. Stores L (unit lower) and U (upper) in `a`.
/// Stores pivot permutation in `piv`. O(n³/3).
fn lu_factor_in_place(
    a: &mut DMatrix<f64>,
    piv: &mut [usize],
) -> Result<(), StepError> {
    let n = a.nrows();
    for k in 0..n {
        // Partial pivot: find max |a[i,k]| for i in k..n
        let mut max_val = a[(k, k)].abs();
        let mut max_row = k;
        for i in (k + 1)..n {
            let v = a[(i, k)].abs();
            if v > max_val {
                max_val = v;
                max_row = i;
            }
        }
        if max_val < 1e-30 {
            return Err(StepError::LuSingular);
        }
        piv[k] = max_row;

        if max_row != k {
            for j in 0..n {
                let tmp = a[(k, j)];
                a[(k, j)] = a[(max_row, j)];
                a[(max_row, j)] = tmp;
            }
        }

        for i in (k + 1)..n {
            a[(i, k)] /= a[(k, k)];
            for j in (k + 1)..n {
                a[(i, j)] -= a[(i, k)] * a[(k, j)];
            }
        }
    }
    Ok(())
}

/// Solve P·L·U·x = b using pre-computed factors. Non-destructive on `a`/`piv`.
/// Can be called multiple times for different RHS vectors.
fn lu_solve_factored(a: &DMatrix<f64>, piv: &[usize], x: &mut DVector<f64>) {
    let n = a.nrows();

    // Apply row permutation to RHS
    for k in 0..n {
        if piv[k] != k {
            x.swap_rows(k, piv[k]);
        }
    }

    // Forward substitution (L·y = Pb)
    for i in 1..n {
        for k in 0..i {
            x[i] -= a[(i, k)] * x[k];
        }
    }

    // Back substitution (U·x = y)
    for i in (0..n).rev() {
        for k in (i + 1)..n {
            x[i] -= a[(i, k)] * x[k];
        }
        x[i] /= a[(i, i)];
    }
}
```

**B.3. Add `LuSingular` to `StepError`** (`mujoco_pipeline.rs:748`)

```rust
#[non_exhaustive]
pub enum StepError {
    // ... existing variants ...
    /// LU decomposition failed (zero pivot in M − h·D).
    LuSingular,
}
```

Update the `Display` impl (`mujoco_pipeline.rs:763`) — this is an exhaustive match
and will fail compilation without the new arm:

```rust
Self::LuSingular => {
    write!(f, "LU decomposition failed in implicit integration")
}
```

**B.4. Wire dispatch** — covered in A.5 above.

##### Sub-task 13.C: Update `model_builder.rs` and MJCF Mapping

**C.1.** Map MJCF variants to distinct core integrators (`model_builder.rs:616`):

```rust
// Before:
MjcfIntegrator::ImplicitSpringDamper | MjcfIntegrator::ImplicitFast => {
    Integrator::ImplicitSpringDamper
}

// After:
MjcfIntegrator::ImplicitSpringDamper => Integrator::ImplicitSpringDamper,
MjcfIntegrator::ImplicitFast => Integrator::ImplicitFast,
MjcfIntegrator::Implicit => Integrator::Implicit,
```

**C.2.** Add `Implicit` variant to MJCF types (`types.rs:15`):

```rust
pub enum MjcfIntegrator {
    Euler,
    RK4,
    Implicit,              // NEW: full implicit (asymmetric D, LU)
    ImplicitFast,          // Symmetric D, Cholesky
    ImplicitSpringDamper,  // Legacy diagonal-only (keep for regression tests)
}
```

**C.3.** Update parser and serialization (`types.rs:33, :44`):

Update `from_str` (`:33`):
```rust
"implicit" => Some(Self::Implicit),
"implicitfast" => Some(Self::ImplicitFast),
"implicitspringdamper" => Some(Self::ImplicitSpringDamper),
```

Update `as_str` (`:44`) — exhaustive match, will fail compilation without new arm.
Note: `ImplicitSpringDamper` currently returns `"implicit"` from `as_str()`;
change it to `"implicitspringdamper"` so the new `Implicit` variant owns `"implicit"`:
```rust
Self::Implicit => "implicit",
Self::ImplicitSpringDamper => "implicitspringdamper",
```

MuJoCo models specifying `integrator="implicit"` now get the full implicit solver.
Models wanting the old diagonal behavior must use `"implicitspringdamper"`.

**⚠ Breaking change:** The string `"implicit"` currently maps to
`ImplicitSpringDamper` (diagonal-only). After this change, it maps to the new
full `Implicit` variant with asymmetric D and LU factorization. Any existing
MJCF models using `integrator="implicit"` will silently change integration
behavior. Migration: switch to `integrator="implicitspringdamper"` to preserve
the old diagonal-only behavior. Per project principles, this breaking change is
intentional — it aligns the parser with MuJoCo's semantics.

##### Sub-task 13.D: Derivatives Interop

**D.1. Force-derivative assembly (`mjd_smooth_vel`):** `mjd_smooth_vel`
(derivatives.rs:972) is already correctly structured with three additive
components. The `implicit_mode` gates in `mjd_passive_vel` (`:484`) use
`== Integrator::ImplicitSpringDamper` comparisons, so the new variants
automatically get correct behavior (tendon derivatives included).

**D.2. Actuator velocity derivatives (`mjd_actuator_vel`):** The existing
implementation (`:527–603`) correctly handles Joint, Tendon, and Site
transmissions. **Muscle actuators are skipped** (`:529–531`). This matches
MuJoCo.

**D.3. Coriolis derivatives (`mjd_rne_vel`):** (`:732–952`) computes the full
Coriolis velocity Jacobian. Already implemented and tested by Task #12. No
changes needed.

**D.4. Transition derivative dispatch (`mjd_transition_hybrid`):** Three
exhaustive match sites in `derivatives.rs` must be updated to handle the new
variants. These compute analytical transition matrices `∂x_{t+1}/∂x_t`:

**(a) `dvdv` computation** (`:1226`): The velocity-velocity Jacobian block.

For `ImplicitFast`/`Implicit`, the implicit update is `v⁺ = v + h·qacc` where
`(M − h·D)·qacc = f(v)`. Differentiating w.r.t. v:

```
∂v⁺/∂v = I + h · (M − h·D)⁻¹ · ∂f/∂v = I + h · (M − h·D)⁻¹ · qDeriv
```

This has the same structure as the Euler case (`I + h · M⁻¹ · qDeriv`) but
with `(M − h·D)⁻¹` replacing `M⁻¹`. Note: unlike `ImplicitSpringDamper`
(which uses `(M+hD+h²K)⁻¹ · (M + h·(qDeriv + D))` with an extra `+D`
correction), the new variants do NOT need the `+D` term because damping forces
are computed explicitly into `qfrc_passive` and therefore already captured by
`qDeriv`. In `ImplicitSpringDamper`, damping is absorbed into the matrix and
excluded from `qfrc_passive`, requiring the separate `+D` compensation.

Implementation: use `scratch_m_impl`
(which holds the factorized M − h·D from the forward step) to solve each
column of qDeriv:

```rust
Integrator::ImplicitFast => {
    // ∂v⁺/∂v = I + h · (M − h·D)⁻¹ · qDeriv
    // scratch_m_impl holds Cholesky factors of (M − h·D) from forward pass
    let mut dvdv = DMatrix::identity(nv, nv);
    for j in 0..nv {
        let mut col = data_work.qDeriv.column(j).clone_owned();
        cholesky_solve_in_place(&data_work.scratch_m_impl, &mut col);
        for i in 0..nv {
            dvdv[(i, j)] += h * col[i];
        }
    }
    dvdv
}
Integrator::Implicit => {
    // Same formula but scratch_m_impl holds LU factors
    let mut dvdv = DMatrix::identity(nv, nv);
    for j in 0..nv {
        let mut col = data_work.qDeriv.column(j).clone_owned();
        lu_solve_factored(&data_work.scratch_m_impl, &data_work.scratch_lu_piv, &mut col);
        for i in 0..nv {
            dvdv[(i, j)] += h * col[i];
        }
    }
    dvdv
}
```

Note: this requires the factorized `scratch_m_impl` to persist from the forward
pass into the derivative computation. Currently `mj_fwd_acceleration_implicit`
leaves `scratch_m_impl` holding the Cholesky/LU factors after solving, so this
is automatically available. However, LU factorization and solve must be split
into separate functions (`lu_factor_in_place` + `lu_solve_factored`) so the
derivative pass can reuse the factors for multiple column solves. See B.2, E.2.

**(b) Activation/control column solves** (`:1320`, `:1485`): These solve
`(effective mass)⁻¹ · rhs` for derivative columns. Same dispatch pattern:

```rust
Integrator::ImplicitFast => {
    cholesky_solve_in_place(&data_work.scratch_m_impl, &mut dvdact);
}
Integrator::Implicit => {
    lu_solve_factored(&data_work.scratch_m_impl, &data_work.scratch_lu_piv, &mut dvdact);
}
```

**(c) Analytical derivative gate** (`:1565`): `can_analytical` uses
`!matches!(model.integrator, Integrator::RungeKutta4)`. New variants will
correctly default to `can_analytical = true`, which is correct — both
`ImplicitFast` and `Implicit` support analytical derivatives.

**D.5. Fluid drag velocity derivatives:** Not yet implemented. `mjd_passive_vel`
does not compute drag-velocity Jacobians. In MuJoCo, this contributes 6×6
body-level coupling blocks to D via `addJTBJSparse`. For models with non-zero
`density`/`viscosity`, the implicit integrator will underestimate D (missing drag
linearization). Acceptable for Phase A/B — fluid drag is rare in RL workloads.
Can be added as a follow-up sub-task when needed.

##### Sub-task 13.E: Scratch Buffer Adjustments

**E.1.** The existing `scratch_m_impl` (`:1882`) is `DMatrix::zeros(nv, nv)` —
sufficient for the dense M − h·D matrix. No new scratch buffers needed for
Phase A.

**E.2. LU factor/solve split (Phase B):** The derivative interop (D.4) requires
solving multiple RHS vectors against the same factorized `(M − h·D)`. A combined factor+solve function would destroy the factors during solve. For
`Implicit`, split into:

- `lu_factor_in_place(a, piv)` — factorize A = P·L·U, store factors in `a`,
  permutation in `piv`
- `lu_solve_factored(a, piv, x)` — solve using pre-computed factors (reusable)

Add `scratch_lu_piv: Vec<usize>` field to Data, initialized to `vec![0; nv]`.
Update both `Model::make_data()` (`:2340`) and the manual `Clone` impl for Data
(`:2050`) — both use explicit struct literals, so the compiler will enforce this.
The forward pass calls `lu_factor_in_place`, then `lu_solve_factored` for qacc.
The derivative pass calls `lu_solve_factored` for each column of qDeriv. For
`ImplicitFast`, `cholesky_solve_in_place` already supports multiple solves
against the same factors (Cholesky is non-destructive on the factor matrix).

#### Dependency Graph

```
A.1 (Integrator enum + #[non_exhaustive])
 ├─→ A.4 (mj_fwd_acceleration_implicitfast)
 │    └─→ A.5 (forward accel + velocity + step dispatch wiring)
 ├─→ C.1 + C.2 + C.3 (model_builder + MJCF types + parser)
 ├─→ D.4 (derivatives.rs match site updates: :1226, :1320, :1485)
 ├─→ B.1 (mj_fwd_acceleration_implicit_full)
 │    ├─→ B.2 (lu_factor_in_place + lu_solve_factored)
 │    │    └─→ B.3 (StepError::LuSingular + Display arm + #[non_exhaustive])
 │    └─→ E.2 (scratch_lu_piv field in Data)
 └─→ (A.2, A.3 — no code changes needed; verified correct by construction)
```

Note: A.2 and A.3 require **no code changes** because the existing
`== Integrator::ImplicitSpringDamper` comparisons automatically exclude the new
variants. They are listed for documentation/verification only.

Note: D.4 is required even for Phase A (`ImplicitFast`) because adding the new
enum variant to the exhaustive matches at `:1226`, `:1320`, `:1485` is needed
for compilation. The `ImplicitFast` arms use `cholesky_solve_in_place` on
`scratch_m_impl`. The `Implicit` arms (Phase B) use `lu_solve_factored`.

#### Known Approximations & Non-Conformances

1. **Dense storage for qDeriv and M_hat:** MuJoCo uses CSR with a tree-topology
   sparsity pattern for D, and off-tree-envelope entries from tendon/actuator
   coupling are silently dropped. Our dense `DMatrix` avoids this — we capture
   **all** coupling terms regardless of tree topology. This is more correct than
   MuJoCo for tendon paths crossing kinematic branches, but O(nv²) in memory
   vs MuJoCo's O(nv·depth). Acceptable for target nv < 100.

2. **Muscle actuator velocity derivatives skipped** (`:529–531`): Matches MuJoCo.
   Muscle FLV curve gradients are piecewise and not cleanly linearizable.

3. **Fluid drag velocity derivatives not implemented** (D.5): Models with
   non-zero `density`/`viscosity` will have incomplete D. Acceptable for
   initial implementation; add as follow-up when drag is used in practice.

4. **No `skipfactor` / factorization reuse:** MuJoCo's `mj_implicitSkip` allows
   reusing a previously computed factorization when `skipfactor > 0`, amortizing
   cost across steps. Not implemented. Can be added later as an optimization.

5. **No sleep filtering:** MuJoCo filters awake/asleep DOFs to reduce system
   size. Not implemented (sleeping is Task #16). When sleeping is added, the
   implicit solver should operate only on awake DOFs.

6. **Constraint force derivatives excluded:** Matches MuJoCo — neither `implicit`
   nor `implicitfast` includes `∂(qfrc_constraint)/∂(qvel)`.

7. **Cholesky SPD precondition not guaranteed:** `ImplicitFast` uses Cholesky,
   which requires `M − h·D` to be SPD. This holds when D is NSD (all
   velocity-derivative components are dissipative). However, actuators with
   positive velocity feedback (`gainprm[2] > 0`, or `gainprm[2] < 0` with
   negative `ctrl` input) produce `dforce_dv > 0`, making the actuator
   contribution to D positive-semi-definite rather than NSD. For sufficiently
   large positive `dforce_dv` or large `h`, `M − h·D` may lose positive-
   definiteness and Cholesky fails with `StepError::CholeskyFailed`. This
   matches MuJoCo's behavior — `implicitfast` can also fail with exotic
   actuator configurations. Mitigation: use `Implicit` (LU, no SPD
   requirement) for models with positive velocity feedback.

8. **ImplicitFast derivative uses full qDeriv with fast-approximated LHS:**
   The forward pass builds `(M − h·D_sym)` (Coriolis excluded, D symmetrized)
   and stores its Cholesky factors in `data.scratch_m_impl`. The derivative
   pass (`mjd_transition_hybrid`) clones data, then calls `mjd_smooth_vel`
   (`:972`) which **unconditionally** calls all three components: `mjd_passive_vel`,
   `mjd_actuator_vel`, AND `mjd_rne_vel` — there is no integrator gate in
   `mjd_smooth_vel`. This overwrites `data_work.qDeriv` with the full D
   (Coriolis included, unsymmetrized), while `data_work.scratch_m_impl` retains
   the forward-pass Cholesky factors. The analytical transition derivative is
   therefore `dvdv = I + h · (M − h·D_sym)⁻¹ · D_full`, mixing the fast
   approximation in the LHS with the full derivative in the RHS.

   **Impact:** This produces slightly different derivatives than a pure
   finite-difference of the actual `ImplicitFast` integration step would. The
   discrepancy is proportional to `h · ‖D_coriolis‖ / ‖M‖` and is small for
   typical timesteps. It does not cause numerical failure. MuJoCo sidesteps
   this entirely by using finite-difference derivatives exclusively.

   **Design rationale:** We intentionally keep `mjd_smooth_vel` ungated (always
   full D) because: (1) it matches MuJoCo's separation of forward vs derivative
   paths, (2) the full D gives more accurate derivatives even when the forward
   integration is approximate, and (3) adding an integrator gate to
   `mjd_smooth_vel` would complicate the API for marginal benefit. If exact
   consistency between forward and derivative is needed, use finite-difference
   derivatives (`config.use_analytical = false`).

#### Acceptance Criteria

**Correctness:**

1. **Tendon-coupled damping stability:** A two-joint arm connected by a tendon
   with `damping=100`, `dt=0.01` produces stable integration under `ImplicitFast`
   where `Euler` diverges. Verify energy is bounded over 1000 steps.

2. **Tendon-coupled actuator stability:** A tendon-driven actuator with
   `gainprm="0 0 -10"` (velocity-dependent gain) produces stable `ImplicitFast`
   integration where `Euler` diverges.

3. **Diagonal regression (`ImplicitSpringDamper`):** Existing implicit integration
   tests (`sim/L0/tests/integration/implicit_integration.rs`) pass unchanged.
   The `ImplicitSpringDamper` variant is untouched.

4. **`ImplicitFast` zero-damping equivalence:** For a model with zero joint
   damping, zero tendon damping, no velocity-dependent actuators, and no fluid
   drag, `ImplicitFast` produces `qacc` matching `Euler` within `1e-10`
   relative tolerance (D = 0, so both solve `M·qacc = f`, but via different
   factorizations — sparse L^T D L vs dense LL^T — causing different
   floating-point rounding).

5. **`Implicit` vs `ImplicitFast` delta:** For a model with Coriolis forces
   (e.g., spinning double pendulum at high angular velocity), `Implicit` and
   `ImplicitFast` produce different trajectories. Verify both are stable but
   `Implicit` has lower energy drift (Coriolis correction improves conservation).

6. **MuJoCo conformance — `ImplicitFast`:** Run the standard conformance model
   suite with `integrator="implicitfast"`. For models without muscles, `qacc`
   matches MuJoCo's `implicitfast` output within `max_relative_error < 1e-6`
   per DOF per step. Document any models that exceed this tolerance with
   root-cause analysis (expected source: dense vs sparse D — our dense D
   captures cross-branch coupling that MuJoCo's sparse D drops).

7. **MuJoCo conformance — `Implicit`:** Same as above with `integrator="implicit"`.
   Tolerance `max_relative_error < 1e-6`.

8. **Tendon spring explicit treatment:** For a model with tendon stiffness but
   no tendon damping, verify that `ImplicitFast` produces the same `qfrc_passive`
   as `Euler` (tendon spring forces are explicit in both).

9. **Analytical vs FD derivative consistency (`ImplicitFast`):** For a model
   with joint damping and tendon damping (but no Coriolis-dominant dynamics),
   compute transition matrices analytically (`use_analytical = true`) and via
   centered FD (`use_analytical = false`). The velocity-velocity block `dvdv`
   should match within `max_element_error < 1e-3` (relaxed due to KA#8:
   analytical uses full D in RHS but fast-approximated LHS).

10. **Analytical vs FD derivative consistency (`Implicit`):** Same test with
    `integrator="implicit"`. Should match within `max_element_error < 1e-5`
    (tighter than ImplicitFast because both LHS and RHS use full D — no
    KA#8 mismatch).

**Performance:**

11. **No regression for Euler/RK4:** Euler and RK4 step times are unchanged
    (implicit code is not on their path).

12. **ImplicitFast overhead:** For a 30-DOF humanoid, `ImplicitFast` step time is
    within 3× of `Euler` step time. The overhead comes from assembling qDeriv
    (O(nv²) for J^T B J) + dense Cholesky (O(nv³/3)).

13. **Implicit overhead:** For the same humanoid, `Implicit` step time is within
    5× of `Euler` (Coriolis derivatives via `mjd_rne_vel` are O(nv² · nbody)).

**Safety:**

14. Zero `unwrap()`/`expect()` in new code. All error paths return
    `Result<(), StepError>`.

15. `lu_factor_in_place` returns `Err(StepError::LuSingular)` if any pivot
    magnitude is below `1e-30`. (Note: `cholesky_in_place` (`:12832`) uses
    `diag <= 0.0` — a different check because Cholesky requires positive
    diagonals, while LU requires non-zero pivots. The `1e-30` threshold for
    LU is conservative against near-singular matrices.)

16. **Cholesky failure on positive velocity feedback (KA #7):** A single-DOF
    actuator with `gainprm="0 0 1000"` (strong positive velocity feedback) and
    large `ctrl > 0` causes `ImplicitFast` to return
    `Err(StepError::CholeskyFailed)`. Same model under `Implicit` (LU) succeeds
    (LU has no SPD requirement). Verifies the documented limitation in KA #7.

#### Files

| File | Action | Changes |
|------|--------|---------|
| `sim/L0/core/src/mujoco_pipeline.rs` | **modify** | Extend `Integrator` enum (`:733`) with `Implicit`, `ImplicitFast`, add `#[non_exhaustive]`; add `LuSingular` to `StepError` (`:748`) + add arm to `Display for StepError` (`:763`); add `mj_fwd_acceleration_implicitfast()` and `mj_fwd_acceleration_implicit_full()` (new functions near `:13008`); add `lu_factor_in_place()` + `lu_solve_factored()` (split LU for derivative reuse); add `scratch_lu_piv: Vec<usize>` to Data (`:1882`); update forward acceleration dispatch (`:12802`); update velocity integration dispatch (`:3474`); update `step()` dispatch (`:3303`) |
| `sim/L0/core/src/derivatives.rs` | **modify** | Update exhaustive matches at `:1226` (dvdv — add `ImplicitFast`/`Implicit` arms with `(M−hD)⁻¹` solve), `:1320` (dvdact solve dispatch), `:1485` (dvdctrl solve dispatch). All three need new arms for both variants. |
| `sim/L0/mjcf/src/types.rs` | **modify** | Add `Implicit` variant to `MjcfIntegrator` (`:15`); update `from_str` (`:33`): `"implicit"` → `Implicit`, keep `"implicitspringdamper"` → `ImplicitSpringDamper`; update `as_str` (`:44`): add `Implicit` → `"implicit"`, change `ImplicitSpringDamper` → `"implicitspringdamper"` |
| `sim/L0/mjcf/src/parser.rs` | **modify** | Update `test_parse_option_integrator_types` (`:2482`): `("implicit", MjcfIntegrator::Implicit)` replaces `("implicit", MjcfIntegrator::ImplicitSpringDamper)`; add `("implicit", MjcfIntegrator::Implicit)` test case |
| `sim/L0/mjcf/src/model_builder.rs` | **modify** | Separate `ImplicitFast` and `ImplicitSpringDamper` mapping (`:616`); add `Implicit` → `Integrator::Implicit` |
| `sim/L0/tests/integration/implicit_integration.rs` | **modify** | Add tests for acceptance criteria 1–10, 16: tendon stability, actuator stability, diagonal regression, zero-damping equivalence, Coriolis delta, MuJoCo conformance ×2, tendon spring explicit, derivative consistency ×2, Cholesky failure on positive velocity feedback |
| `sim/docs/MUJOCO_REFERENCE.md` | **modify** | Update implicit path description (`:554–568`) to document both `implicit` and `implicitfast` algorithms, D matrix assembly via `mjd_smooth_vel`, linearization derivation, and factorization strategy |
| `sim/docs/MUJOCO_CONFORMANCE.md` | **modify** | Add conformance entries for `implicit` and `implicitfast` integrators |

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
