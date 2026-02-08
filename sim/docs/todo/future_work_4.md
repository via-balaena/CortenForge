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
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

#### Current State

No derivative infrastructure exists. The pipeline computes several Jacobians
internally for the constraint solver, but these are not exposed as a general
derivative API:

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
The Coriolis matrix computation uses column-perturbation RNE (O(nv·nbody))
rather than full analytical Featherstone velocity derivatives, trading some
redundant computation for implementation simplicity as a pragmatic first step.

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

Part 1 acceptance criteria: 1–22. Part 2 acceptance criteria: 23–32.

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
// efc_lambda is HashMap<WarmstartKey, Vec<f64>> — clone is O(n_contacts).
// This is acceptable because FD already does O(nx + nu) full step() calls.
let efc_lambda_0 = data.efc_lambda.clone();

// For forward differences, compute nominal output by stepping unperturbed:
let y_0 = if !config.centered {
    scratch.step(model)?;
    let y = extract_state(model, &scratch, &qpos_0);
    // Restore scratch to nominal for subsequent perturbations
    scratch.qpos.copy_from(&qpos_0);
    scratch.qvel.copy_from(&qvel_0);
    scratch.act.copy_from(&act_0);
    scratch.ctrl.copy_from(&ctrl_0);
    scratch.efc_lambda = efc_lambda_0.clone();
    Some(y)
} else {
    None
};
```

Phase 1 — State perturbation (A matrix, nx columns), for each `i in 0..nx`:

```rust
if i < nv {
    // Position tangent: mj_integrate_pos_explicit maps dq[i]=±eps to coordinates
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
scratch.efc_lambda = efc_lambda_0.clone();
scratch.step(model)?;
let y_plus = extract_state(model, &scratch, &qpos_0);

// For centered: also compute y_minus with -eps, then:
//   A.column(i) = (y_plus - y_minus) / (2.0 * eps);
// For forward:
//   A.column(i) = (y_plus - y_0) / eps;
```

Phase 2 — Control perturbation (B matrix, nu columns): same pattern with
`scratch.ctrl[j] += eps`.

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

**Scratch reuse:** Single clone upfront. State fields (`qpos`, `qvel`, `act`,
`ctrl`) restored before each perturbation. `step()` overwrites computed fields
(`xpos`, `xquat`, `qM`, `qfrc_*`, etc.) — no explicit restore needed for
those.

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
///   ∂(qfrc_passive)/∂qvel  = diagonal damping + tendon damping J^T·b·J
///   ∂(qfrc_actuator)/∂qvel = affine velocity-dependent gain/bias terms
///   −∂(qfrc_bias)/∂qvel    = −C(q,v) (Coriolis matrix)
///
/// MuJoCo equivalent: `mjData.qDeriv` (sparse nv×nv). Dense here because
/// nv < 100 for target use cases.
#[allow(non_snake_case)]
pub qDeriv: DMatrix<f64>,
```

**Additional scratch buffers** for `mjd_rne_vel()` column-perturbation
(in `mujoco_pipeline.rs`, after `body_min_inertia`, ~line 1935):

```rust
/// Scratch body velocity for analytical Coriolis computation (length `nbody`).
pub deriv_cvel: Vec<SpatialVector>,
/// Scratch bias acceleration for analytical Coriolis computation (length `nbody`).
pub deriv_cacc_bias: Vec<SpatialVector>,
/// Scratch bias force for analytical Coriolis computation (length `nbody`).
pub deriv_cfrc_bias: Vec<SpatialVector>,
```

**Initialization in `make_data()`** (~line 2309):

```rust
qDeriv: DMatrix::zeros(model.nv, model.nv),
deriv_cvel: vec![SpatialVector::zeros(); model.nbody],
deriv_cacc_bias: vec![SpatialVector::zeros(); model.nbody],
deriv_cfrc_bias: vec![SpatialVector::zeros(); model.nbody],
```

**Clone:** Add to manual `impl Clone for Data`:

```rust
qDeriv: self.qDeriv.clone(),
deriv_cvel: self.deriv_cvel.clone(),
deriv_cacc_bias: self.deriv_cacc_bias.clone(),
deriv_cfrc_bias: self.deriv_cfrc_bias.clone(),
```

##### Step 4 — `mjd_passive_vel()`: Passive force velocity derivatives

```rust
/// Compute ∂(qfrc_passive)/∂qvel and add to data.qDeriv.
///
/// Per-DOF damping (all joint types):
///   qfrc_passive[i] -= damping[i] · qvel[i]
///   ⇒ ∂/∂qvel[i] = −damping[i]  (diagonal)
///
/// Tendon damping:
///   qfrc_passive += J^T · (−b · J · qvel)
///   ⇒ ∂/∂qvel = −b · J^T · J   (rank-1 update per tendon)
///
/// Friction loss (tanh-smoothed) is NOT included (MuJoCo also omits it).
///
/// Reuses `model.implicit_damping[i]` (cached diagonal, line 1213) for
/// per-DOF terms. Tendon damping uses `data.ten_J[t]` (line 1819).
fn mjd_passive_vel(model: &Model, data: &mut Data) {
    // Per-DOF damping: diagonal entries
    for i in 0..model.nv {
        data.qDeriv[(i, i)] += -model.implicit_damping[i];
    }

    // Tendon damping: −b · J^T · J (rank-1 outer product per tendon)
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
```

**Borrow analysis:** Reads `model.implicit_damping`, `model.tendon_damping`,
`data.ten_J`. Writes `data.qDeriv`. No aliasing — `ten_J` and `qDeriv` are
separate fields of `Data`.

**Complexity:** O(nv) diagonal + O(ntendon · nnz_J²). Tendon Jacobians are
typically very sparse (only DOFs along the tendon path), so effectively
O(nv + ntendon · depth²).

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

##### Step 6 — `mjd_rne_vel()`: Coriolis matrix via column-perturbation RNE

```rust
/// Compute ∂(qfrc_bias)/∂qvel and SUBTRACT from data.qDeriv.
///
/// The bias force has two velocity-dependent components:
/// 1. Gravity: ∂/∂qvel = 0 (position-only).
/// 2. Coriolis/centrifugal: C(q,v) · v where C is the Coriolis matrix.
///
/// # Algorithm: Column-perturbation of velocity-only RNE
///
/// For each DOF j, compute C[:,j] by running the velocity-dependent part of
/// RNE with qvel = e_j (j-th unit vector):
///   1. Forward pass with v = e_j → body velocities, bias accelerations
///   2. Backward pass → bias forces (gravity excluded)
///   3. Project to joint space → column j of C
///
/// # Complexity
///
/// O(nv · nbody). For nv ~ nbody ~ 30 (humanoid): ~27,000 small matrix-vector
/// operations (6×6 spatial inertia). Fast for models up to ~100 DOFs.
///
/// # Optimization note
///
/// The Coriolis matrix satisfies C + C^T = dM/dt. This could halve the work
/// (compute upper triangle and mirror). Deferred.
fn mjd_rne_vel(model: &Model, data: &mut Data) {
    for j in 0..model.nv {
        let col = rne_velocity_column(model, data, j);
        for i in 0..model.nv {
            data.qDeriv[(i, j)] -= col[i];
        }
    }
}
```

**`rne_velocity_column()` helper:**

```rust
/// Compute one column of the Coriolis matrix C(q, ·) by running a
/// **velocity-only** variant of RNE with qvel = e_j (j-th unit vector).
///
/// This is NOT the same as calling `mj_rne()` with qvel = e_j. The
/// velocity-only variant differs in two critical ways:
///
/// 1. **No gravity:** The root body has `a_bias = 0` (not `-g`). Gravity is
///    position-dependent and does not contribute to ∂(bias)/∂qvel.
///
/// 2. **No parent bias acceleration propagation:** The standard RNE forward
///    pass sets `a_bias[b] = a_bias[parent] + ...`, which would propagate
///    gravity and other position-dependent accelerations. The velocity-only
///    variant computes ONLY the velocity-product term:
///    `a_bias[b] = v_parent ×_m v_joint` (no a_parent term).
///    This is correct because a_parent contains only position-dependent
///    terms (gravity) when qvel = e_j, and we want only velocity derivatives.
///
/// Uses `data.deriv_cvel`, `data.deriv_cacc_bias`, `data.deriv_cfrc_bias` as
/// scratch buffers (pre-allocated, length nbody each). Reads position-dependent
/// FK quantities (`xpos`, `xquat`, `cinert`) which are stable.
///
/// # Algorithm
///
/// Initialization:
///   v[world] = 0,  a_bias[world] = 0   (no gravity!)
///
/// Forward pass (root to leaves):
///   Let v_joint = S_b · e_j[dof_b]     (nonzero only if dof_b == j)
///   v[b] = v[parent] + v_joint
///   a_bias[b] = spatial_cross_motion(v[parent], v_joint)
///
///   Note: for bodies whose DOF ≠ j, v_joint = 0, so v[b] = v[parent] and
///   a_bias[b] = 0. Velocities propagate unchanged; bias accelerations
///   are zero until body b's parent chain includes the DOF j body.
///
/// Backward pass (leaves to root):
///   f[b] = I[b] · a_bias[b] + spatial_cross_force(v[b], I[b] · v[b])
///   f[parent] += f[b]
///
/// Projection:
///   C[dof, j] = S_b^T · f[b]   for each joint's DOFs
fn rne_velocity_column(model: &Model, data: &mut Data, dof_j: usize) -> DVector<f64>
```

**Borrow analysis:** `rne_velocity_column` takes `&mut Data` because it writes
to `data.deriv_cvel`, `data.deriv_cacc_bias`, `data.deriv_cfrc_bias`. It reads
`data.xpos`, `data.xquat`, `data.cinert` which are not modified. The caller
(`mjd_rne_vel`) reads the returned `DVector` and writes to `data.qDeriv` between
calls — no overlap with the scratch buffers.

**Relationship to `mj_rne()`:** This function implements a stripped-down RNE
that shares the spatial algebra primitives (`joint_motion_subspace`,
`spatial_cross_motion`, `spatial_cross_force`, body `cinert`) but does NOT
call `mj_rne()` directly. The full `mj_rne()` includes gravity, gyroscopic
terms, and uses actual qvel — none of which apply here.

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
`mjd_passive_vel`, `mjd_actuator_vel`, `mjd_rne_vel`, `rne_velocity_column`,
and `extract_state` are private to the `derivatives` module (implementation
details, tested indirectly through `mjd_smooth_vel` and FD comparison).

**Part 1 module structure:** At the end of Part 1, add to `sim/L0/core/src/lib.rs`:

```rust
/// Simulation transition derivatives (FD and analytical).
pub mod derivatives;

pub use derivatives::{
    DerivativeConfig, TransitionMatrices,
    mjd_transition_fd, mjd_smooth_vel,
};
```

Part 2 will expand these exports with `mjd_transition`, `mjd_transition_hybrid`,
and validation utilities.

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
(`qDeriv`, `deriv_cvel`, `deriv_cacc_bias`, `deriv_cfrc_bias`) which are
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
/// Given: q_new = q_old · exp(ω · h / 2)  (Euler quaternion integration)
///
/// Computes:
/// - d(q_new_tangent)/d(q_old_tangent): 3×3 matrix (identity for small ωh)
/// - d(q_new_tangent)/d(ω): 3×3 matrix (h · I for small ωh, Rodrigues
///   formula derivative for finite ωh)
///
/// MuJoCo equivalent: `mjd_quatIntegrate`.
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
/// then position uses the NEW velocity (see `integrate()`, line 3435):
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
///   ∂qpos_{t+1}/∂qpos_t = I
///   ∂qpos_{t+1}/∂qvel_{t+1} = h · I   (= dqpos_dqvel)
///
/// For quaternion joints (Ball/Free):
///   Uses mjd_quat_integrate() for the 3×3 blocks.
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
/// # Activation integration
///
/// Filter/Integrator: ∂act/∂act = 1, ∂act/∂act_dot = h
/// FilterExact: ∂act/∂act = 1, ∂act/∂act_dot = τ · (1 − exp(−h/τ))
/// Muscle (clamped): derivative is 0 at [0,1] boundaries, 1 otherwise.
struct IntegrationDerivatives {
    /// ∂qpos_{t+1}/∂qpos_t in tangent space. nv × nv, block-diagonal per joint.
    dqpos_dqpos: DMatrix<f64>,
    /// ∂qpos_{t+1}/∂qvel_{t+1} in tangent space. nv × nv, block-diagonal.
    dqpos_dqvel: DMatrix<f64>,
    /// ∂act_{t+1}/∂act_t. na × na, diagonal.
    dact_dact: DMatrix<f64>,
    /// ∂act_{t+1}/∂act_dot. na × na, diagonal.
    dact_dactdot: DMatrix<f64>,
}

fn compute_integration_derivatives(
    model: &Model,
    data: &Data,
) -> IntegrationDerivatives
```

**Borrow analysis:** `compute_integration_derivatives` takes `&Model`, `&Data`
(shared). Returns owned struct. Pure computation, no mutation.

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

**Internal algorithm sketch:**

```rust
// 1. Populate qDeriv via analytical smooth-force derivatives
let mut data_work = data.clone();
mjd_smooth_vel(model, &mut data_work);

// 2. Compute integration derivatives
let integ = compute_integration_derivatives(model, data);

// 3. Velocity columns of A (analytical)
// Euler: ∂v⁺/∂v = I + h · M⁻¹ · qDeriv
//   Uses L^T D L factorization (from mj_crba → mj_factor_sparse)
// Implicit: ∂v⁺/∂v = (M + hD + h²K)⁻¹ · (M + h·(qDeriv + D))
//   Uses Cholesky factorization of M_impl (from mj_fwd_acceleration_implicit)
//
// Euler path: compute M⁻¹ · qDeriv column-by-column via nv sparse solves
let mut dvdv = DMatrix::identity(nv, nv);
for j in 0..nv {
    let mut col = data_work.qDeriv.column(j).clone_owned();
    mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut col);
    for i in 0..nv { dvdv[(i, j)] += h * col[i]; }
}
// ∂q⁺/∂v = dqpos_dqvel · dvdv
let dqdv = &integ.dqpos_dqvel * &dvdv;
// Fill velocity columns of A

// 4. Activation columns of A (analytical)
// For each act state k, find owning actuator, compute:
//   ∂force/∂act_k = gain (input = act, so d(gain·act)/d(act) = gain)
//   ∂qfrc/∂act_k = moment · gain
//   ∂v⁺/∂act_k = h · M⁻¹ · ∂qfrc/∂act_k
// Muscle gain uses FD fallback.

// 5. Position columns of A (FD, same as Phase A)
// Clone scratch, perturb position tangent ±eps, step, extract, difference.

// 6. B matrix
// Simple actuators (None/Fixed/Affine): analytical
//   ∂force/∂ctrl = gain
//   ∂qfrc/∂ctrl = moment · gain
//   ∂v⁺/∂ctrl = h · M⁻¹ · moment · gain
// Complex actuators (Filter/Muscle): FD
```

**ImplicitSpringDamper velocity columns:** The implicit integrator solves
(see `mj_fwd_acceleration_implicit`, line 12967):

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
by `mj_fwd_acceleration_implicit()` and stored in `data.scratch_m_impl`.
Compute `(M + h·(qDeriv + D))` column-by-column and solve via
`cholesky_solve_in_place()`.

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
/// Returns `(max_error_A, max_error_B)`.
pub fn validate_analytical_vs_fd(
    model: &Model,
    data: &Data,
    tol: f64,
) -> Result<(f64, f64), StepError> {
    let fd = mjd_transition(model, data,
        &DerivativeConfig { use_analytical: false, ..Default::default() })?;
    let hybrid = mjd_transition(model, data,
        &DerivativeConfig { use_analytical: true, ..Default::default() })?;
    let (err_a, _) = max_relative_error(&fd.A, &hybrid.A, 1e-10);
    let (err_b, _) = max_relative_error(&fd.B, &hybrid.B, 1e-10);
    Ok((err_a, err_b))
}

/// Check FD convergence by comparing eps and eps/10 derivatives.
pub fn fd_convergence_check(
    model: &Model,
    data: &Data,
    eps: f64,
    tol: f64,
) -> Result<bool, StepError>
```

##### Step 12 — Module structure and exports

New file `sim/L0/core/src/derivatives.rs`. Add to `sim/L0/core/src/lib.rs`:

```rust
/// Simulation transition derivatives (FD, analytical, and hybrid).
pub mod derivatives;

pub use derivatives::{
    DerivativeConfig, TransitionMatrices,
    mjd_transition, mjd_transition_fd, mjd_transition_hybrid,
    mjd_smooth_vel,
};
```

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
15. **Coriolis matrix**: For a 2-link pendulum at `qvel = [1.0, 0.5]`,
    `mjd_rne_vel()` produces Coriolis matrix C(q,v) matching FD of `qfrc_bias`
    w.r.t. `qvel` (column-by-column, ε=1e-7) to within `1e-6`.
16. **qDeriv combined**: `mjd_smooth_vel()` zeroes `qDeriv` before accumulating.
    For a model with dampers, tendons, and actuators, the combined `qDeriv`
    matches FD of `(qfrc_passive + qfrc_actuator − qfrc_bias)` w.r.t. `qvel`
    to within `1e-5`.
17. **Scratch buffer isolation**: `deriv_cvel`, `deriv_cacc_bias`,
    `deriv_cfrc_bias` are NOT aliased with `data.cvel`, `data.cacc_bias`,
    `data.cfrc_bias`. Calling `mjd_rne_vel()` does not corrupt the actual body
    velocities/forces. Verified by comparing `data.cvel` before and after.

**Part 1 infrastructure:**

18. **Warmstart copying**: Perturbed FD solves use nominal warmstart.
    `solver_niter` for perturbed steps ≤ 2× nominal `solver_niter`.
19. **Performance**: FD derivatives for `Model::n_link_pendulum(6, 1.0, 0.1)`
    (nv=6, nu=0, na=0) complete in < 10ms (wall clock, release mode).
20. **`cargo clippy -p sim-core -- -D warnings`** passes clean.
21. **All existing sim-core tests pass unchanged** (additive module, no behavior
    changes to existing code).
22. **At least 17 new tests** in `sim/L0/tests/integration/derivatives.rs`
    covering Part 1 criteria (1–17).

---

**Part 2 acceptance criteria (Steps 8–12):**

**Phase C/D — Integration + Hybrid:**

23. **Hybrid vs FD agreement**: `validate_analytical_vs_fd()` returns
    `max_error_A < 1e-3` and `max_error_B < 1e-3` for a 3-link pendulum with
    torque actuators (Euler integrator).
24. **Hybrid velocity columns**: For a damped 3-link pendulum (damping=1.0),
    velocity columns of A from hybrid match pure FD columns to within `1e-4`
    relative error (element-wise via `max_relative_error`).
25. **Hybrid cost savings**: For a 10-DOF model (nu=10, na=0, all simple Joint
    actuators), hybrid mode calls `step()` at most `2 · nv` times (centered).
    Pure FD would require `2 · (2·nv + nu) = 60` calls. Hybrid requires
    `2 · nv = 20`. Verified by counting (add a step counter in test).
26. **ImplicitSpringDamper analytical**: For a stiff spring system
    (stiffness=1000, damping=10) with `Integrator::ImplicitSpringDamper`, hybrid
    velocity derivatives use the `(M + hD + h²K)⁻¹` correction. The velocity
    block of A matches pure FD to within `1e-3`.
27. **Activation columns analytical**: For a model with `ActuatorDynamics::Filter`
    (na=2), hybrid computes activation columns of A analytically:
    `∂v⁺/∂act` via `h · M⁻¹ · moment · gain`. Matches FD to within `1e-3`.
28. **mjd_transition dispatch**: `mjd_transition()` with `use_analytical=true`
    and Euler returns the same result as `mjd_transition_hybrid()`. With
    `use_analytical=false` or RK4, returns same as `mjd_transition_fd()`.
29. **Data convenience method**: `data.transition_derivatives(model, config)`
    returns the same result as `mjd_transition(model, &data, config)`.

**Part 2 infrastructure:**

30. **FD convergence utility**: `fd_convergence_check()` returns `true` for
    epsilon in `[1e-4, 1e-8]` with tolerance `1e-3`.
31. **`cargo clippy -p sim-core -- -D warnings`** still passes clean.
32. **At least 10 additional tests** covering Part 2 criteria (23–29).

#### Files

**Part 1 (Steps 0–7):**

| File | Action | Description |
|------|--------|-------------|
| `sim/L0/core/src/derivatives.rs` | **new** | `TransitionMatrices`, `DerivativeConfig`, `mjd_transition_fd()`, `extract_state()`, `mjd_smooth_vel()`, `mjd_passive_vel()`, `mjd_actuator_vel()`, `mjd_rne_vel()`, `rne_velocity_column()` |
| `sim/L0/core/src/lib.rs` | **modify** | Add `pub mod derivatives;` and re-export `DerivativeConfig`, `TransitionMatrices`, `mjd_transition_fd`, `mjd_smooth_vel` |
| `sim/L0/core/src/mujoco_pipeline.rs` | **modify** | Add `qDeriv: DMatrix<f64>` and `deriv_cvel`/`deriv_cacc_bias`/`deriv_cfrc_bias` scratch fields to `Data` (~line 1889). Initialize in `make_data()` (~line 2309). Add to `impl Clone for Data` (~line 1939). Change visibility of `mj_solve_sparse`, `joint_motion_subspace`, `spatial_cross_motion`, `spatial_cross_force` from private to `pub(crate)` |
| `sim/L0/tests/integration/mod.rs` | **modify** | Add `mod derivatives;` |
| `sim/L0/tests/integration/derivatives.rs` | **new** | 17+ integration tests covering Part 1 acceptance criteria (1–17) |

**Part 2 (Steps 8–12):**

| File | Action | Description |
|------|--------|-------------|
| `sim/L0/core/src/derivatives.rs` | **modify** | Add `IntegrationDerivatives`, `mjd_transition_hybrid()`, `mjd_transition()`, `compute_integration_derivatives()`, `mjd_quat_integrate()`, `max_relative_error()`, `validate_analytical_vs_fd()`, `fd_convergence_check()`, `impl Data { fn transition_derivatives() }` |
| `sim/L0/core/src/lib.rs` | **modify** | Expand re-exports: add `mjd_transition`, `mjd_transition_hybrid` |
| `sim/L0/tests/integration/derivatives.rs` | **modify** | Add tests for Part 2 acceptance criteria (18–29) |

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
