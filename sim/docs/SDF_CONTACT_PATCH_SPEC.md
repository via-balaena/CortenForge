# SDF Contact Patch Spec — Octree Detection + Analytical Normals

## Status: Draft — pending review

---

## 1. Problem

Two fundamental deficiencies in the SDF-SDF collision pipeline prevent
correct rotational dynamics for geometry-driven mechanisms.

### 1.1 Grid normal tangential error

SDF grid normals are computed via centered finite differences on trilinear-
interpolated distance values. For curved surfaces, the Cartesian grid
introduces tangential error proportional to `cell_size / radius`.

**Concrete failure:** a hinge pendulum (shaft in cylindrical bore) damps to
zero even with frictionless contacts. The bore surface is cylindrical — true
normals are purely radial. Grid normals leak axial components because the
Cartesian grid can't exactly represent a cylindrical iso-surface:

```
True cylindrical normal:  (cos θ, sin θ, 0)                — purely radial
Grid normal (0.5mm cell): (0.95 cos θ, 0.93 sin θ, 0.08)  — axial leak
                                                       ↑
                                            Virtual friction force
                                            that damps free rotation
```

The error magnitude is O(cell_size / radius). For a 5mm bore at 0.5mm cell
size, ~10% tangential leak — equivalent to μ ≈ 0.1 friction on a surface
configured as frictionless.

The constraint solver uses the contact normal to define the tangent frame.
With a purely radial normal, the two tangent directions are circumferential
and axial — both free DOF for a hinge. With the leaky grid normal, one
tangent direction acquires a radial component, coupling friction into a DOF
that should be unconstrained.

### 1.2 Contact flooding

`trace_surface_into_other()` (operations.rs) visits every grid cell within
2×cell_size of the source surface and tests penetration into the destination
SDF. For a 20³ grid, ~2000 cells are near-surface. Both A→B and B→A passes
produce 50–200+ raw contacts per SDF-SDF pair.

After spatial dedup and capping (`cap_sdf_contacts` at `sdf_maxcontact`),
the solver still receives far more contacts than needed. Problems:

- **Solver cost O(nefc²):** 50 contacts = 2500 constraint interactions per
  PGS iteration. 8 contacts = 64 interactions.
- **Conflicting normals:** nearby contacts with slightly different grid
  normals create competing constraints that slow convergence.
- **Wasted rows:** 200 contacts constraining ≤6 DOF means ≥194 redundant
  constraint rows that contribute noise, not information.

### 1.3 The deeper problem: the grid is a discretization of an analytical tree

The SDF grid is generated from a `Solid` CSG expression tree in cf-design.
The Solid already provides:

- `evaluate(point)` — exact SDF value at any point
- `gradient(point)` — exact analytical gradient (surface normal)
- `evaluate_interval(aabb)` — conservative (lo, hi) bounds over any region
- `bounds()` — shape bounding box

The grid discretizes this into a fixed-resolution voxel array, losing
precision. The collision pipeline then tries to reconstruct geometry from
the grid — getting worse results than the original analytical tree could
provide directly.

The current architecture uses the grid for both detection AND precision.
The correct architecture: **use the analytical tree for precision, and use
spatial acceleration (octree + interval evaluation) for detection.**

### 1.4 What this blocks

| Mechanism | Failure mode |
|-----------|-------------|
| Hinge (pin in bore) | Rotational damping from axial normal leak |
| Bearing | Same + axial friction from radial leak |
| Ball joint | Rotation damped in all axes |
| Prismatic joint | Linear friction from transverse leak |
| Gear teeth | Tooth contact forces have spurious tangential components |

---

## 2. Solution overview

Replace the grid-centric collision pipeline with an analytical-first
architecture:

```
Old:  grid scan ALL cells → hundreds of raw contacts → dedup → cap
      (grid normals)        (grid positions)           (noisy)

New:  octree prune → leaf cells at contact surface → Newton-exact contacts
      (interval eval)  (both surfaces present)       (CSG normals, 4-8 total)
```

Three foundational changes:

| Change | What | Why |
|--------|------|-----|
| PhysicsShape trait refactor | `distance()`, `gradient()`, `bounds()` required; `evaluate_interval()` optional | Analytical evaluation is the primary interface, not the grid |
| Octree bi-surface detection | Recursive subdivision with interval pruning replaces grid scan | Detection resolution adapts to contact geometry, not grid voxel count |
| Three-tier dispatch | Convex → Octree → Grid fallback | Each tier is the best available algorithm for the shapes involved |

---

## 3. PhysicsShape trait refactor

### 3.1 The foundational change

Make analytical evaluation the required interface. The collision pipeline
calls `distance()` and `gradient()` — not the grid — for all point queries.
Add `evaluate_interval()` as the gate for octree detection.

```rust
pub trait PhysicsShape: Send + Sync + std::fmt::Debug {
    // === REQUIRED: every shape provides these ===

    /// Signed distance at a local-frame point.
    /// Negative inside, positive outside, zero on surface.
    fn distance(&self, local_point: &Point3<f64>) -> Option<f64>;

    /// Outward unit normal at a local-frame point.
    fn gradient(&self, local_point: &Point3<f64>) -> Option<Vector3<f64>>;

    /// Axis-aligned bounding box in local frame.
    fn bounds(&self) -> Aabb;

    /// Distance from shape center to surface along a local-frame direction.
    /// Returns Some for convex shapes, None for concave.
    fn effective_radius(&self, local_dir: &Vector3<f64>) -> Option<f64>;

    /// The underlying SDF grid.
    /// Used by GPU compute shaders, SDF-vs-primitive contacts, and flex
    /// vertex collision. All shapes carry grids (even AnalyticalShape
    /// generates one for GPU upload).
    fn sdf_grid(&self) -> &SdfGrid;

    // === OPTIONAL: octree acceleration ===

    /// Conservative (min, max) field bounds over a local-frame AABB.
    ///
    /// If lo > 0: shape is entirely outside this region.
    /// If hi < 0: shape is entirely inside this region.
    /// If lo ≤ 0 ≤ hi: shape surface may cross this region.
    ///
    /// Enables octree-based contact detection (Tier 2). Returns None
    /// if interval evaluation is unavailable (grid-only shapes).
    fn evaluate_interval(&self, local_aabb: &Aabb) -> Option<(f64, f64)> {
        None
    }
}
```

### 3.2 Why `distance()` and `gradient()` are required, not defaulted

They are the PRIMARY evaluation interface. Making them required forces every
shape to provide them explicitly:

- **Grid-backed shapes** (ShapeSphere, ShapeConvex, ShapeConcave): implement
  by delegating to `self.grid.distance()` / `self.grid.gradient()`. This
  is a 2-line implementation per shape, and makes the delegation explicit.

- **AnalyticalShape** (new, cf-design): implements via
  `self.solid.evaluate()` / `self.solid.gradient()`. Exact analytical
  evaluation — the whole point.

If these were defaulted (delegating to `sdf_grid()` internally), a shape
author could forget to override them and silently get grid precision when
analytical precision was intended. Required methods prevent this.

### 3.3 Why `sdf_grid()` stays required

Six call sites depend on unconditional grid access:

| Site | Usage |
|------|-------|
| `sdf_collide.rs:87` | SDF-vs-primitive contacts (sphere, box, capsule, etc.) |
| `shape.rs:89` | SDF-SDF grid fallback (Tier 3) |
| `shape.rs:122` | SDF-Plane grid fallback |
| `flex_narrow.rs:223` | Flex vertex vs SDF |
| GPU trait dispatch | Grid indices for GPU compute shaders |
| Model builder | AABB/rbound computation |

Making `sdf_grid()` return `Option` would require updating all 6 sites to
handle `None`. This is mechanical but has no functional benefit today —
every shape carries a grid (even AnalyticalShape needs one for GPU upload).

The clean path: keep `sdf_grid()` required now. After the octree path
proves out, the grid dependency shrinks naturally (primitive contacts
migrate to `distance()`/`gradient()`, GPU evolves). The `Option` split
becomes a cleanup, not a foundation change.

### 3.4 Migration for existing shapes

| Shape | distance() | gradient() | bounds() | evaluate_interval() |
|-------|-----------|-----------|---------|-------------------|
| ShapeSphere | `Some(p.coords.norm() - self.radius)` | `Some(p.coords.try_normalize(…))` | Sphere AABB from radius | None (uses Tier 1 convex path) |
| ShapeConvex | `self.grid.distance(*p)` | `self.grid.gradient(*p)` | `self.grid.aabb()` | None (uses Tier 1 or Tier 3) |
| ShapeConcave | `self.grid.distance(*p)` | `self.grid.gradient(*p)` | `self.grid.aabb()` | None (uses Tier 3) |
| **AnalyticalShape** | `Some(self.solid.evaluate(p))` | Normalized `self.solid.gradient(p)` | `self.solid.bounds()` | `Some(self.solid.evaluate_interval(aabb))` |

ShapeSphere gets analytical `distance()`/`gradient()` — these are trivial
closed-form expressions. ShapeConvex/ShapeConcave delegate to the grid
(same precision as before, but now through the explicit trait methods
instead of the grid directly).

---

## 4. AnalyticalShape in cf-design

### 4.1 The type

```rust
// cf-design/src/mechanism/shapes.rs (new file)

/// PhysicsShape backed by an analytical CSG tree.
///
/// Provides exact SDF evaluation, exact analytical gradients, and
/// conservative interval bounds for octree detection. The grid is
/// kept for GPU compute shaders and legacy primitive contacts.
#[derive(Debug)]
pub struct AnalyticalShape {
    grid: Arc<SdfGrid>,
    solid: Arc<Solid>,
    hint: ShapeHint,
}

impl PhysicsShape for AnalyticalShape {
    fn distance(&self, p: &Point3<f64>) -> Option<f64> {
        Some(self.solid.evaluate(p))
    }

    fn gradient(&self, p: &Point3<f64>) -> Option<Vector3<f64>> {
        let g = self.solid.gradient(p);
        let norm = g.norm();
        if norm > 1e-10 { Some(g / norm) } else { None }
    }

    fn bounds(&self) -> Aabb {
        // Analytical bounds, falling back to grid if solid is infinite
        self.solid.bounds().unwrap_or_else(|| self.grid.aabb())
    }

    fn effective_radius(&self, local_dir: &Vector3<f64>) -> Option<f64> {
        match self.hint {
            ShapeHint::Sphere(r) => Some(r),
            ShapeHint::Convex => ray_march_solid(&self.solid, local_dir),
            ShapeHint::Concave => None,
        }
    }

    fn sdf_grid(&self) -> &SdfGrid { &self.grid }

    fn evaluate_interval(&self, aabb: &Aabb) -> Option<(f64, f64)> {
        Some(self.solid.evaluate_interval(aabb))
    }
}
```

### 4.2 Model builder integration

The model builder (line 170 of `model_builder.rs`) has the `solid` variable
in scope:

```rust
// Before:
let shape: Arc<dyn PhysicsShape> = match solid.shape_hint() {
    ShapeHint::Sphere(radius) => Arc::new(ShapeSphere::new(grid, radius)),
    ShapeHint::Convex => Arc::new(ShapeConvex::new(grid)),
    ShapeHint::Concave => Arc::new(ShapeConcave::new(grid)),
};

// After:
let shape: Arc<dyn PhysicsShape> = Arc::new(
    AnalyticalShape::new(grid, Arc::new(solid.clone()), solid.shape_hint())
);
```

**Clone cost:** `Part::solid()` returns `&Solid` (not Arc). `solid.clone()`
deep-copies the CSG expression tree. For typical mechanism shapes (5–10
FieldNode nodes): ~500 bytes, negligible. For complex bio-inspired shapes:
could be more. Future optimization: store `Arc<Solid>` in Part.

**All cf-design shapes get full analytical capability.** MJCF-loaded shapes
continue to use ShapeSphere/ShapeConvex/ShapeConcave with grid-based
implementations (no regression).

### 4.3 Analytical evaluation cost

| CSG complexity | Nodes | Cost vs grid lookup |
|---------------|-------|-------------------|
| Bare primitive (cylinder) | 1 | ~2× |
| Simple mechanism (subtract + translate) | 3–5 | ~5× |
| Complex bio-inspired (smooth union of 10+) | 20+ | ~50× |

The octree evaluates `evaluate_interval` at each node (~100-500 interval
evaluations per pair). Newton refinement does ~6 point evaluations per
contact (3 steps × distance + gradient). With 8 contacts: ~48 analytical
calls. Total cost is O(100-600) analytical evaluations per collision pair
— comparable to the ~2000 grid lookups of the old overlap scan.

---

## 5. Octree bi-surface detection

### 5.1 Core concept

Given two shapes A and B with poses, find all contact points using a single
octree traversal in world space. The octree's root is the intersection of
the two shapes' world-space AABBs. Each node queries both shapes' interval
bounds to determine if contact is possible in that region. Leaf cells
where both surfaces are present yield exact contact points via Newton
iteration.

This replaces the O(N³) grid scan with O(contact_surface_area × log(1/tolerance))
detection that adapts to the geometry, not the grid resolution.

### 5.2 Algorithm

```rust
fn octree_contact_detect(
    a: &dyn PhysicsShape, pose_a: &Pose,
    b: &dyn PhysicsShape, pose_b: &Pose,
    margin: f64,
    max_contacts: usize,
) -> Vec<SdfContact> {
    // 1. Overlap AABB in world space
    let world_a = world_aabb(&a.bounds(), pose_a);
    let world_b = world_aabb(&b.bounds(), pose_b);
    let root = world_a.intersection(&world_b).expanded(margin);
    if root.is_empty() { return vec![]; }

    // 2. Target cell size (matches grid resolution for comparability)
    let cell_size = a.sdf_grid().cell_size().min(b.sdf_grid().cell_size());
    let max_depth = (root.max_extent() / cell_size).log2().ceil() as u32;

    // 3. Recursive octree traversal
    let mut leaf_contacts = Vec::new();
    octree_recurse(
        &root, 0, max_depth,
        a, pose_a, b, pose_b,
        margin, cell_size,
        &mut leaf_contacts,
    );

    // 4. Cluster + select representatives
    let patches = cluster_contacts(&leaf_contacts, cell_size);
    select_representatives(&patches, max_contacts)
}
```

### 5.3 Octree recursion with interval pruning

```rust
fn octree_recurse(
    cell: &Aabb,         // world-space AABB of current octree node
    depth: u32,
    max_depth: u32,
    a: &dyn PhysicsShape, pose_a: &Pose,
    b: &dyn PhysicsShape, pose_b: &Pose,
    margin: f64,
    target_cell_size: f64,
    contacts: &mut Vec<SdfContact>,
) {
    // Transform cell to each shape's local frame (conservative AABB)
    let local_a = conservative_local_aabb(cell, pose_a);
    let local_b = conservative_local_aabb(cell, pose_b);

    // Evaluate interval bounds on both shapes
    let Some((lo_a, hi_a)) = a.evaluate_interval(&local_a) else { return };
    let Some((lo_b, hi_b)) = b.evaluate_interval(&local_b) else { return };

    // Adaptive depth threshold: at coarser levels, allow more interior
    // penetration before pruning (surfaces could be near a corner even
    // if the cell center is deep inside)
    let cell_extent = cell.max_extent();
    let depth_threshold = -cell_extent;

    // === PRUNING RULES ===

    // Rule 1: A entirely outside this cell by more than margin
    if lo_a > margin { return; }

    // Rule 2: B entirely outside this cell by more than margin
    if lo_b > margin { return; }

    // Rule 3: A entirely deep inside (surface not in this cell)
    if hi_a < depth_threshold { return; }

    // Rule 4: B entirely deep inside (surface not in this cell)
    if hi_b < depth_threshold { return; }

    // === LEAF: extract contacts ===
    if cell_extent <= target_cell_size || depth >= max_depth {
        extract_leaf_contacts(cell, a, pose_a, b, pose_b, margin, contacts);
        return;
    }

    // === SUBDIVIDE into 8 children ===
    let mid = cell.center();
    for child in octree_children(cell.min, mid, cell.max) {
        octree_recurse(
            &child, depth + 1, max_depth,
            a, pose_a, b, pose_b,
            margin, target_cell_size, contacts,
        );
    }
}
```

**`conservative_local_aabb`**: transforms a world-space AABB to a shape's
local frame by transforming all 8 corners and re-bounding. This is
conservative — the local AABB is larger than the true projection of the
world cell. At 45° rotation, up to √3× bloat per axis. The octree
compensates by subdividing further in bloated regions.

```rust
fn conservative_local_aabb(world_cell: &Aabb, pose: &Pose) -> Aabb {
    let corners = world_cell.corners();
    Aabb::from_points(corners.iter().map(|c| pose.inverse_transform_point(c)))
}
```

### 5.4 Leaf contact extraction

At a leaf cell, both shapes' surfaces may be present. Extract up to 2
contacts (one from each surface):

```rust
fn extract_leaf_contacts(
    cell: &Aabb,
    a: &dyn PhysicsShape, pose_a: &Pose,
    b: &dyn PhysicsShape, pose_b: &Pose,
    margin: f64,
    contacts: &mut Vec<SdfContact>,
) {
    let center = cell.center();
    let half_size = cell.max_extent() * 0.5;
    let max_displacement = cell.max_extent() * 3.0;

    // Evaluate both shapes at cell center
    let p_a = pose_a.inverse_transform_point(&center);
    let p_b = pose_b.inverse_transform_point(&center);
    let Some(d_a) = a.distance(&p_a) else { return };
    let Some(d_b) = b.distance(&p_b) else { return };

    // --- Contact from A's surface penetrating B ---
    if d_a.abs() < half_size * 2.0 && d_b < margin {
        if let Some(c) = newton_contact(
            &p_a, a, pose_a, b, pose_b, margin, max_displacement,
        ) {
            contacts.push(c);
        }
    }

    // --- Contact from B's surface penetrating A ---
    if d_b.abs() < half_size * 2.0 && d_a < margin {
        if let Some(c) = newton_contact(
            &p_b, b, pose_b, a, pose_a, margin, max_displacement,
        ) {
            // Negate normal: B's outward normal points B→A, need A→B
            let mut c = c;
            c.normal = -c.normal;
            contacts.push(c);
        }
    }
}
```

### 5.5 Newton contact refinement

Project a seed point onto the source shape's exact zero-level set, then
compute depth from the destination shape:

```rust
fn newton_contact(
    seed_local: &Point3<f64>,
    src: &dyn PhysicsShape,     // shape whose surface we project onto
    src_pose: &Pose,
    dst: &dyn PhysicsShape,     // shape we check penetration against
    dst_pose: &Pose,
    margin: f64,
    max_displacement: f64,
) -> Option<SdfContact> {
    let mut p = *seed_local;
    let p_original = p;

    // Baseline gradient for consistency check
    let baseline_grad = src.gradient(&p)?;

    // Newton iteration: project onto src's zero-level set
    for _ in 0..3 {
        let d = src.distance(&p)?;
        if d.abs() < 1e-10 { break; }
        let n = src.gradient(&p)?;
        p -= n * d;

        // Safety: max displacement guard
        if (p - p_original).norm() > max_displacement {
            return None;
        }
    }

    // Safety: normal consistency guard
    let n_local = src.gradient(&p)?;
    if n_local.dot(&baseline_grad) < 0.0 {
        return None;  // converged to wrong surface
    }

    // Transform to world, check penetration into dst
    let world_point = src_pose.transform_point(&p);
    let p_dst = dst_pose.inverse_transform_point(&world_point);
    let d_dst = dst.distance(&p_dst)?;
    if d_dst >= margin - 1e-10 {
        return None;  // not actually penetrating (with float tolerance)
    }

    let n_world = src_pose.rotation * n_local;
    Some(SdfContact {
        point: world_point,
        normal: n_world,  // src's outward normal (caller handles A→B sign)
        penetration: (-d_dst).max(0.0),
    })
}
```

**Convergence:** For exact SDFs (sphere, cuboid, cylinder, hard booleans),
|∇f| = 1 everywhere, so `p -= d * n̂` is true Newton with quadratic
convergence (2 iterations to machine precision). For smooth booleans in the
blend region, |∇f| < 1 and the step overshoots slightly — convergence is
linear but still fast (3 iterations suffice). Contact points are on the
surface, outside the blend region, so this is rarely an issue.

**Safety guards:**
- **Max displacement** (3× cell extent): prevents Newton from jumping to a
  different zero-level set on concave shapes (e.g., bore surface → outer
  cylinder on `cyl.subtract(inner_cyl)`).
- **Normal consistency** (dot product > 0): catches convergence to the
  wrong surface by comparing the refined normal against the initial
  gradient direction at the seed point.
- **Penetration check** (d_dst < margin): rejects contacts where the
  refined point doesn't actually penetrate the other shape.

### 5.6 Pruning effectiveness

| Scenario | Grid cells (old) | Octree interval evals | Leaf cells | Reduction |
|----------|----------------|---------------------|-----------|-----------|
| Sphere stacking (20³) | 16,000 | ~80 | ~20 | 200× |
| Pin in socket (30³) | 54,000 | ~300 | ~60 | 180× |
| Gear mesh (40³) | 128,000 | ~800 | ~150 | 160× |
| Separated bodies | 16,000+ | 1 | 0 | ∞ |

The octree evaluates `evaluate_interval` at each node — the interval
arithmetic prunes entire subtrees. For a sphere, the first few levels
quickly eliminate 7/8 of children at each step, converging on the ~1-cell-
thick surface shell. For concave shapes, more children survive (both inner
and outer surfaces), but the total is still far fewer than the full grid.

**AABB rotation penalty:** For heavily-rotated bodies (45°), the
conservative local AABB is up to √3× larger per axis. The octree
compensates by subdividing further, adding ~1 extra depth level. Total
interval evaluations increase ~2-3× for rotated bodies. Still far better
than grid scan.

**Full-overlap degenerate case:** If both shapes are deeply overlapping
(centers coincident), most octree cells contain both surfaces and can't be
pruned. The octree degenerates toward full-volume scanning. This is correct
— deep penetration is a transient state that the solver resolves within a
few steps, after which the overlap region shrinks.

### 5.7 Interval tightness and pruning limits

The octree's pruning effectiveness depends on how tight the interval bounds
are. Three sources of looseness:

**Concave shapes (Subtract/Intersect):** Interval arithmetic for
`max(a, -b)` is conservative — it can't capture the correlation between
operands. For a tube (`cylinder.subtract(inner_cylinder)`), the interval
near the bore surface may span a wider range than the actual field values.
The octree compensates by subdividing further, but concave shapes produce
~2-3× more leaf cells than convex shapes of similar size.

**Smooth booleans:** The `k/4` correction expands the lower bound. For
`k = 1mm` and `cell_size = 0.5mm`, the correction is 50% of the cell size,
causing one extra octree level near the blend surface. For larger blend
radii or finer grids, the effect is more pronounced. This is correct (the
blend truly adds material) but reduces pruning efficiency in blend regions.

**Lipschitz-bounded shapes** (ellipsoid, superellipsoid, gyroid, loft,
helix, twist, bend): These use 27-point sampling + Lipschitz correction.
For highly anisotropic shapes (e.g., 100:1 ellipsoid), the Lipschitz
constant can be very large (L=100), making the correction term dwarf the
cell size. The octree effectively can't prune for such shapes — it
degrades to near-brute-force scanning of the overlap volume.

**Mitigation:** This is a known limitation of interval arithmetic. Shapes
with high Lipschitz constants should use the grid fallback (Tier 3) instead
of the octree. The `evaluate_interval` implementation could detect this:
if the returned interval is wider than, say, 10× the AABB diagonal, it's
too loose to be useful and should return `None` to force Tier 3. This
heuristic can be added to `AnalyticalShape::evaluate_interval()`.

For typical mechanism shapes (cylinders, cuboids, sphere subtractions),
the intervals are tight and pruning is effective.

### 5.8 Seed point selection

Leaf contact extraction uses the cell center as the Newton iteration seed
point (§5.4). Why the cell center?

1. **Central location:** The cell center is equidistant from all cell
   boundaries. If the surface crosses the cell, the center is within
   `cell_size/2` of the surface — a good starting point for Newton.

2. **Deterministic:** No dependency on surface location within the cell.
   Adjacent cells always use different seed points, reducing the chance
   of duplicate contacts converging to the same surface point.

3. **Concave safety:** For concave shapes with multiple surfaces in one
   cell, the center has no bias toward either surface. Newton converges to
   whichever surface the gradient at the center points toward. The safety
   guards (max displacement, normal consistency) reject if it picks the
   wrong one.

**Future enhancement:** Adaptive seed selection. Instead of the cell center,
start from the point where the SDF value is closest to zero (found during
interval evaluation by evaluating at a few sample points). This would
improve convergence for surfaces near cell boundaries.

---

## 6. Three-tier dispatch

### 6.1 Updated `compute_shape_contact`

```rust
pub fn compute_shape_contact(
    a: &dyn PhysicsShape,
    pose_a: &Pose,
    b: &dyn PhysicsShape,
    pose_b: &Pose,
    margin: f64,
    max_contacts: usize,
) -> Vec<SdfContact> {
    let dir = separation_direction(pose_a, pose_b)
        .unwrap_or_else(Vector3::z);
    let local_dir_a = pose_a.rotation.inverse() * dir;
    let local_dir_b = pose_b.rotation.inverse() * (-dir);

    // Tier 1: Both convex → analytical single contact [UNCHANGED]
    if let (Some(r_a), Some(r_b)) = (
        a.effective_radius(&local_dir_a),
        b.effective_radius(&local_dir_b),
    ) {
        let contact_point = pose_a.position + dir * (r_a - ...);
        let stable_normal = stabilize_direction(dir);
        return vec![SdfContact { ... }];
    }

    // Tier 2: Both support intervals → octree detection [NEW]
    // Probe: check if evaluate_interval is available on both shapes
    if a.evaluate_interval(&a.bounds()).is_some()
        && b.evaluate_interval(&b.bounds()).is_some()
    {
        return octree_contact_detect(
            a, pose_a, b, pose_b, margin, max_contacts,
        );
    }

    // Tier 3: Grid fallback → surface tracing [EXISTING]
    sdf_sdf_contact_raw(a.sdf_grid(), pose_a, b.sdf_grid(), pose_b, margin)
}
```

### 6.2 Dispatch in `sdf_collide.rs`

The SDF-SDF branch in `collide_with_sdf()` currently checks GPU before
CPU. With the octree, the priority becomes:

```rust
// 1. Both convex → analytical (Tier 1, CPU, cheap, exact)
// 2. Both support intervals → octree (Tier 2, CPU, analytical quality)
// 3. GPU available → GPU grid tracing (Tier 3 accelerated)
// 4. CPU grid fallback → sdf_sdf_contact_raw (Tier 3)
```

Octree takes priority over GPU because:
- It produces fewer, higher-quality contacts (analytical normals)
- It doesn't require GPU readback latency
- cf-design shapes (the ones that need mechanism precision) always have
  `evaluate_interval`

GPU remains for MJCF shapes or scenes with many SDF pairs where grid
tracing on CPU is too slow.

**`cap_sdf_contacts()` stays active** for Tier 3 and GPU paths. The octree
(Tier 2) naturally produces sparse contacts and uses its own sampling
pipeline. Only Tier 3/GPU flood the solver.

### 6.3 SDF-Plane contacts

`compute_shape_plane_contact` also gets octree detection. The plane's
interval over any AABB is trivially exact:

```rust
fn plane_interval(aabb: &Aabb, normal: &Vector3<f64>, offset: f64) -> (f64, f64) {
    let corners = aabb.corners();
    let dists: [f64; 8] = corners.map(|c| c.coords.dot(normal) - offset);
    (dists.iter().copied().fold(f64::MAX, f64::min),
     dists.iter().copied().fold(f64::MIN, f64::max))
}
```

The octree is one-sided: the SDF shape's interval prunes based on surface
presence, the plane interval prunes based on which side of the plane the
cell is on. This replaces the full-grid iteration in `sdf_plane_contact`
for shapes with `evaluate_interval`.

Shapes without `evaluate_interval` fall back to the existing
`sdf_plane_contact` grid-based path.

---

## 7. Contact sampling

### 7.1 Patch clustering

Contacts from the octree form natural clusters (leaf cells near different
parts of the contact surface). Cluster using connected-component labeling
on the octree leaf cells:

```
1. Build adjacency: two leaf cells are connected if they share a face
   (6-connectivity in the octree grid)
2. BFS/DFS flood-fill to find connected components
3. Each component is one contact patch
```

This correctly identifies ring-shaped patches (ball-in-socket = one ring),
separated patches (both sides of a bore = two patches), and complex shapes.

### 7.2 Per-patch representative selection

For each patch, select contacts maximizing spatial coverage:

```
1. Always include the deepest penetration point
2. Greedy farthest-point: add the contact maximally distant from all
   already-selected contacts
3. Budget: max(2, max_contacts / num_patches) per patch
```

Total contacts per pair: typically 4–8.

### 7.3 DOF coverage

| Mechanism | Patches | Per-patch | Total | Constrained DOF |
|-----------|---------|-----------|-------|-----------------|
| Hinge (pin in bore) | 2 (sides) | 2–3 | 4–6 | 2 trans + 2 rot |
| Ball joint (socket) | 1 (ring) | 4 | 4 | 3 translational |
| Prismatic (rail) | 2 (sides) | 2 | 4 | 2 trans + 2 rot |
| Flat (box on floor) | 1 (rect) | 4 | 4 | 1 trans + 2 rot |

Budget from `model.sdf_maxcontact`, not hard-coded.

---

## 8. Grid fallback (Tier 3)

### 8.1 When it activates

Tier 3 activates when either shape lacks `evaluate_interval`:
- MJCF-imported SDF shapes (grid-only, no Solid)
- Any shape where `evaluate_interval` returns None

### 8.2 Existing path preserved

`sdf_sdf_contact_raw()` (operations.rs) is unchanged. It does the full
grid scan with spatial dedup. `cap_sdf_contacts()` in `sdf_collide.rs`
caps the result at `sdf_maxcontact`.

### 8.3 Normal refinement for grid contacts

Even grid-only shapes benefit from Newton refinement using their
`distance()` / `gradient()` trait methods. For grid-backed shapes, these
delegate to the grid — but Newton iteration on the trilinear surface
improves normals from O(cell_size) to O(cell_size²) error.

Apply `newton_contact` (§5.5) to each grid-detected contact after
dedup + capping. This is a pure improvement with no behavioral risk.

**Critical: cap BEFORE refinement.** Refining 200 grid contacts to have
identical analytical normals creates a rank-deficient constraint Jacobian.
Cap to `sdf_maxcontact` first, then refine the survivors.

---

## 9. GPU integration

### 9.1 Current path

GPU path (`GpuSdfCollision::sdf_sdf_contacts`) mirrors the CPU grid scan
on the GPU. It produces raw contacts with the same flooding, deduplicates
on CPU.

### 9.2 Integration strategy

**Phase A (this spec):** CPU octree replaces CPU grid fallback for shapes
with `evaluate_interval`. GPU path unchanged.

**Phase B (future, GPU hybrid):** GPU raw contacts → CPU selection +
Newton refinement:

```rust
let raw = gpu.sdf_sdf_contacts(sdf_id, &sdf_pose, other_sdf_id, &other_pose, margin);
let patches = cluster_contacts(&raw, cell_size);
let mut selected = select_representatives(&patches, model.sdf_maxcontact);
for c in &mut selected {
    refine_against_best_shape(c, shape_a, &sdf_pose, shape_b, &other_pose);
}
```

**Phase C (future, optional):** GPU overlap-guided dispatch — limit compute
shader to the overlap AABB region.

---

## 10. Testing strategy

### 10.1 Unit tests — octree detection

| Test | Setup | Pass criteria |
|------|-------|---------------|
| `octree_sphere_sphere` | Two overlapping spheres | Contacts match analytical depth |
| `octree_sphere_bore` | Sphere in cylindrical bore | Contacts on both bore sides |
| `octree_pin_in_socket` | Pin in socket (concave) | ≥ 2 patches, radial normals |
| `octree_separated` | Two non-overlapping shapes | Empty result, ≤ 1 interval eval |
| `octree_pruning_count` | Pin in socket, count interval evals | < 5% of grid cell count |
| `octree_matches_grid` | Various geometries | Contacts within cell_size of grid results |

### 10.2 Unit tests — Newton refinement

| Test | Setup | Pass criteria |
|------|-------|---------------|
| `newton_sphere_exact` | Seed near sphere surface | Refined to machine precision |
| `newton_cylinder_radial` | Seed on bore surface | Axial normal component < 1e-8 |
| `newton_csg_subtract` | Seed on bore (cyl - inner_cyl) | Matches Solid::gradient() |
| `newton_displacement_guard` | Seed far from surface | Rejected, returns None |
| `newton_normal_flip_guard` | Seed near CSG seam | Rejected if normal flips |

### 10.3 Integration tests — dispatch

| Test | Setup | Pass criteria |
|------|-------|---------------|
| `dispatch_tier1` | ShapeSphere × ShapeSphere | Exactly 1 analytical contact |
| `dispatch_tier2` | AnalyticalShape × AnalyticalShape | Octree path used, 4–8 contacts |
| `dispatch_tier3` | ShapeConvex × ShapeConcave | Grid fallback used, capped |
| `dispatch_tier2_over_gpu` | AnalyticalShape pair with GPU | Octree used, GPU skipped |

### 10.4 Regression tests

| Test | Pass criteria |
|------|---------------|
| `sphere_stacking` | Gap stable through 2000 steps |
| `concave_phase1a/b/c` | Bowl + sphere (existing tests) |
| `concave_phase2a/b/c` | Tube + pin (existing tests) |
| `concave_phase3a/b/c` | Socket + flanged pin (existing tests) |

### 10.5 Dynamics validation — the critical test

```rust
#[test]
fn hinge_pendulum_energy_conservation() {
    // Setup: pin-in-bore hinge, pendulum arm, gravity
    // Contacts: frictionless (friction=0.0 on both geoms)
    // Initial: arm horizontal (maximum potential energy)
    //
    // Run: 5000 steps at 2kHz (2.5 seconds, several swing periods)
    //
    // Pass criteria:
    //   - Energy loss < 1% per period
    //   - (Old system: > 50% loss per period from virtual friction)
}
```

### 10.6 Performance benchmarks

| Benchmark | Metric | Target |
|-----------|--------|--------|
| `octree_vs_grid_scan` | Interval evals vs cell lookups | ≥ 50× fewer for pin-in-socket |
| `contact_count` | Per SDF-SDF pair | 4–8 (Tier 2) vs 50–200 (Tier 3) |
| `solver_convergence` | PGS iterations to tolerance | ≥ 3× fewer (fewer, better contacts) |

---

## 11. Implementation phases

### Phase 1 — PhysicsShape trait extension

**Files:** `sim-core/src/sdf/shape.rs`, `sim-core/src/sdf/shapes/sphere.rs`,
`sim-core/src/sdf/shapes/convex.rs`, `sim-core/src/sdf/shapes/concave.rs`
**Effort:** Small
**Risk:** None (additive — new required methods with trivial implementations)

Add `distance()`, `gradient()`, `bounds()` as required methods. Add
`evaluate_interval()` with `None` default. Implement for all three existing
shapes. ShapeSphere gets analytical implementations; ShapeConvex/ShapeConcave
delegate to the grid.

### Phase 2 — AnalyticalShape in cf-design

**Files:** `cf-design/src/mechanism/shapes.rs` (new),
`cf-design/src/mechanism/model_builder.rs`
**Effort:** Medium
**Risk:** Low (regression test against existing stacking + concave tests)

Create AnalyticalShape wrapping `Arc<SdfGrid>` + `Arc<Solid>` + ShapeHint.
Wire into model builder. All cf-design shapes now provide exact evaluation
and interval bounds.

### Phase 3 — Octree detection algorithm

**Files:** `sim-core/src/sdf/octree_detect.rs` (new)
**Effort:** Medium-large
**Risk:** Low (standalone module with unit tests, not yet integrated)

Implement `octree_contact_detect()`, `octree_recurse()`,
`extract_leaf_contacts()`, `newton_contact()`, `conservative_local_aabb()`.
Adapt the octree recursion pattern from `adaptive_dc.rs:228-307`.
Standalone unit tests for sphere-sphere, sphere-bore, pin-in-socket.

### Phase 4 — Three-tier dispatch integration

**Files:** `sim-core/src/sdf/shape.rs`, `sim-core/src/collision/sdf_collide.rs`
**Effort:** Small-medium
**Risk:** Medium (integration — full regression suite)

Update `compute_shape_contact` with three-tier dispatch. The `max_contacts`
parameter addition touches 14 call sites (2 production in `sdf_collide.rs`,
12 in tests across `shape.rs` and `concave_collision_tests.rs`). Production
callers pass `model.sdf_maxcontact`; test callers pass a reasonable default.

Update `sdf_collide.rs` to route octree before GPU.
Keep `cap_sdf_contacts()` for Tier 3 and GPU paths.
Run all existing collision tests.

### Phase 5 — SDF-Plane octree + grid refinement

**Files:** `sim-core/src/sdf/shape.rs`, `sim-core/src/sdf/octree_detect.rs`
**Effort:** Small
**Risk:** Low

Add `plane_interval()` helper. Update `compute_shape_plane_contact` with
octree path for shapes with `evaluate_interval`. Apply Newton refinement
to Tier 3 grid fallback contacts (after capping).

### Phase 6 — Dynamics validation

**Files:** `sim/L0/tests/integration/` (new tests),
`cf-design/src/mechanism/` (integration tests with AnalyticalShape)
**Effort:** Medium
**Risk:** None (tests only)

Implement hinge pendulum energy conservation test and performance benchmarks.

Also implement the octree integration tests that require cf-design shapes
(deferred from Phase 3 because they need AnalyticalShape, which lives in
cf-design and is unavailable to sim-core unit tests):

| Test | Setup | Pass criteria |
|------|-------|---------------|
| `octree_sphere_bore` | AnalyticalShape sphere in cylindrical bore | Contacts on both bore sides, radial normals |
| `octree_pin_in_socket` | AnalyticalShape pin in socket (concave CSG) | ≥ 2 patches, radial normals, no axial leak |
| `octree_pruning_count` | Pin in socket, instrument interval eval count | < 5% of equivalent grid cell count |
| `octree_matches_grid` | Various AnalyticalShape geometries | Octree contacts within cell_size of grid-path results |
| `newton_cylinder_radial` | AnalyticalShape cylinder bore contact | Axial normal component < 1e-8 |
| `newton_csg_subtract` | AnalyticalShape bore (cyl - inner_cyl) | Normal matches Solid::gradient() exactly |

---

## Appendix A — Key files

| File | Role |
|------|------|
| `sim/L0/core/src/sdf/shape.rs` | PhysicsShape trait + `compute_shape_contact` dispatch |
| `sim/L0/core/src/sdf/operations.rs` | Grid-based surface tracing (Tier 3 fallback) |
| `sim/L0/core/src/sdf/shapes/*.rs` | ShapeSphere, ShapeConvex, ShapeConcave |
| `sim/L0/core/src/collision/sdf_collide.rs` | Pipeline dispatch + contact capping |
| `design/cf-geometry/src/sdf.rs` | SdfGrid (distance, gradient, aabb) |
| `design/cf-geometry/src/aabb.rs` | Aabb (intersection, corners, is_empty, center) |
| `design/cf-design/src/solid.rs` | Solid CSG tree (evaluate, gradient, bounds, shape_hint) |
| `design/cf-design/src/interval.rs` | Interval arithmetic for evaluate_interval |
| `design/cf-design/src/gradient.rs` | Analytical gradient for all FieldNode types |
| `design/cf-design/src/adaptive_dc.rs:228-307` | Reference octree recursion pattern |
| `design/cf-design/src/mechanism/model_builder.rs` | cf-design → sim-core Model construction |

## Appendix B — CSG gradient reference

`Solid::gradient()` provides analytical gradients for all node types:

| Node | Gradient rule |
|------|--------------|
| Sphere | `p / \|p\|` |
| Cuboid | Nearest-face normal |
| Cylinder | Radial: `(x, y, 0) / √(x²+y²)` ; Cap: `(0, 0, ±1)` |
| Capsule | Sphere-traced along segment |
| Torus | `(p_xy - R·p̂_xy, p_z) / \|…\|` |
| Cone | Analytical cone normal |
| Union(A, B) | `∇A` if `A ≤ B`, else `∇B` |
| Subtract(A, B) | `∇A` if `A ≥ -B`, else `-∇B` |
| Intersect(A, B) | `∇A` if `A ≥ B`, else `∇B` |
| SmoothUnion(A, B, k) | `h·∇A + (1-h)·∇B`, `h = clamp(0.5 + 0.5·(B-A)/k)` |
| Translate(A, v) | `∇A(p - v)` |
| Rotate(A, R) | `R · ∇A(R⁻¹·p)` |
| Scale(A, s) | `∇A(p/s)` |

## Appendix C — Interval evaluation reference

`Solid::evaluate_interval(aabb)` provides conservative bounds for all nodes:

| Node | Interval rule | Tightness |
|------|-------------|-----------|
| Sphere | `norm_interval(aabb) - r` | Exact |
| Cuboid | Interval arithmetic on box SDF | Exact |
| Cylinder | 2D norm interval + Z interval | Exact |
| Union(A, B) | `(min(lo_a, lo_b), min(hi_a, hi_b))` | Tight |
| Subtract(A, B) | `(max(lo_a, -hi_b), max(hi_a, -hi_b))` | Tight |
| SmoothUnion(A, B, k) | `(min(lo_a, lo_b) - k/4, min(hi_a, hi_b))` | Moderate |
| Translate(A, v) | Shift AABB, evaluate child | Exact |
| Rotate(A, R) | Rotate corners, build AABB, evaluate child | Conservative |
| Lipschitz fallback | 27-point sample + L×diagonal correction | Conservative |

The mesher (adaptive_dc.rs) uses these intervals to prune 80-99% of octree
cells. The collision octree uses the same intervals.

## Appendix D — Comparison with existing approaches

| Feature | Grid scan (current) | Octree (this spec) | MuJoCo |
|---------|--------------------|--------------------|--------|
| Detection | O(N³) grid cells | O(surface_area × log(1/tol)) | GJK/EPA (convex only) |
| Normal | Grid finite differences | CSG analytical gradient | Analytical (primitives) |
| Contacts | 50–200+ → cap | 4–8 strategic | 1–6 per pair |
| Non-convex | Grid multi-contact | Octree + Newton | Not supported |
| Rotation accuracy | O(cell_size/radius) | Machine precision | Exact (primitives) |
| Detection resolution | Fixed (grid cell_size) | Adaptive (octree depth) | N/A |
| Scaling | O(resolution³) | O(contact_area × log) | O(vertex_count) |

CortenForge's advantage: analytical normals and interval-based detection
for arbitrary CSG combinations, with resolution that adapts to the contact
geometry rather than a fixed grid. Non-convex geometry (sockets, gears,
bearings) that MuJoCo can't handle at all.
