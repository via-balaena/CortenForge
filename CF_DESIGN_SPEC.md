# CF_DESIGN_SPEC — Implicit Surface Design Kernel

> **Status**: Phase 1 complete — 2026-03-16
> **Crate**: `cf-design` (`crates/cf-design/`)
> **Layer**: 0 (pure Rust, zero framework dependencies)
> **Depends on**: `cf-geometry` (IndexedMesh, SdfGrid, Shape, Aabb, ConvexHull, Bvh)
> **Consumed by**: `sim-mjcf` (MJCF generation), `sim-core` (collision shapes), `mesh-io` (STL/3MF export), `sim-bevy` (visualization)

---

## 1. The Problem

CortenForge has a complete physics simulation pipeline (sim-core, 79/79
conformance) and a unified geometric kernel (cf-geometry, 10 shape variants,
GJK/EPA, BVH). But there is no way to *design* geometry inside CortenForge.

The current workflow:

```
External CAD tool  →  export STL  →  load into sim  →  simulate
```

This has three problems:

1. **Manual collision mesh workflow** — the exported mesh is not necessarily a
   good collider. Users create separate collision geometry manually.
2. **AI-hostile** — an AI cannot drive Fusion 360. It can write code.
3. **Design/sim gap** — iterate in CAD, re-export, re-load, re-simulate. Slow
   and error-prone.

The desired workflow:

```
Code (human or AI)  →  geometry IS the collider  →  simulate  →  3D print
```

One source of truth. No export step. No approximation gap.

---

## 2. The Vision

CortenForge is an AI-native, simulation-first, manufacturing-ready platform for
bio-inspired mechatronics. The design system must:

1. **Be code-first** — an AI (or human) writes Rust code that defines geometry.
   No GUI required (though one could be built on top).
2. **Produce colliders by default** — the design artifact IS the simulation
   artifact. Same `Arc<IndexedMesh>`, same `Shape`.
3. **Produce printable geometry** — the same artifact exports to STL/3MF for
   3D printing.
4. **Excel at organic shapes** — bio-inspired mechanisms have smooth blends,
   variable thickness, organic transitions. Not sharp chamfers and fillet radii.
5. **Be AI-generatable** — the representation must be natural for an LLM to
   produce. Mathematical functions are more natural than vertex lists or B-Rep
   topology.

---

## 3. Why Implicit Surfaces

Three established approaches exist for computational geometry:

| Approach | Representation | Booleans | Organic shapes | AI-friendly | Sharp features |
|----------|---------------|----------|----------------|-------------|----------------|
| **B-Rep** (Parasolid, Fornjot) | NURBS surfaces + topological adjacency | Complex (surface intersection) | Hard (requires fillet ops) | Hard (topology is intricate) | Excellent |
| **CSG tree** (TinkerCAD, OpenSCAD) | Tree of primitives + boolean ops | Native (tree structure) | Poor (sharp boolean edges) | Medium (tree is structured) | Good |
| **Implicit / SDF** (libfive, nTop) | Math function f(x,y,z) → f64 | Trivial (min/max) | Native (smooth blends) | Excellent (it's just a function) | Requires dual contouring |

For CortenForge's use case — AI-generated bio-inspired mechanisms for 3D
printing and simulation — **implicit surfaces are the most natural fit**.

### 3.1 How It Works

A shape is a scalar field. Every point in space has a value:

- **Negative** → inside the solid
- **Positive** → outside the solid
- **Zero** → on the surface

```rust
// A sphere centered at the origin with radius r
fn sphere(r: f64, p: Point3<f64>) -> f64 {
    p.coords.norm() - r
}

// A box with half-extents h
fn box(h: Vector3<f64>, p: Point3<f64>) -> f64 {
    let q = p.coords.map(|v| v.abs()) - h;
    q.map(|v| v.max(0.0)).norm() + q.x.max(q.y.max(q.z)).min(0.0)
}
```

### 3.1.1 SDF vs F-Rep: An Important Distinction

A **signed distance field (SDF)** is a special case where the value at every
point equals the exact Euclidean distance to the nearest surface (negative
inside). An **f-rep** (functional representation) is the general case: any
function where `f(p) < 0` defines the interior.

**In practice, most composed operations produce f-reps, not true SDFs.** Only
a few primitives (sphere, cuboid, capsule) are exact SDFs. After any boolean,
smooth blend, twist, or bend, the distance property is corrupted. The
zero-isosurface (the boundary) remains correct, but the magnitude of the field
no longer equals Euclidean distance.

This matters because:
- **`shell()` and `round()` require exact distance.** Applied to an f-rep,
  wall thickness and rounding radius will be non-uniform.
- **Sphere tracing (ray marching) is safe** with f-reps that underestimate
  distance (they converge slower but never overshoot).
- **Meshing (marching cubes, dual contouring) only needs the sign and gradient
  direction** — not exact distance. Both work correctly on f-reps.

cf-design uses the term **"field"** for the general case and reserves **"SDF"**
for functions with exact Euclidean distance. The `Solid` type stores a field.
Operations that require exact distance (Section 3.5) document this requirement.

**Distance correctness classes:**

| Class | Property | Safe for |
|-------|----------|----------|
| **Exact SDF** | `|f(p)| = dist(p, surface)` | Everything (shell, round, offset, ray march, mesh) |
| **Lower bound** | `|f(p)| ≤ dist(p, surface)` | Ray marching (slow convergence), meshing |
| **F-rep** | Sign correct, magnitude unrelated to distance | Meshing only |

### 3.2 Booleans Are Trivial

The underlying math (cf-design implements these as `FieldNode` variants, not
raw functions — see Section 8.3):

```rust
fn union(a: f64, b: f64) -> f64     { a.min(b) }
fn subtract(a: f64, b: f64) -> f64  { a.max(-b) }
fn intersect(a: f64, b: f64) -> f64 { a.max(b) }
```

No surface intersection algorithms. No topological surgery. Just `min` and `max`.

**Distance note:** `min`/`max` produce exact distance in one half-space and a
lower bound in the other. The zero-isosurface is always correct. These are
safe for meshing and ray marching, but `shell()` or `round()` applied after a
boolean will produce slightly non-uniform results near the boolean seam.

### 3.3 Organic Blends Are Free

This is the killer feature for bio-inspired design. Smooth boolean operations
produce organic transitions with zero additional complexity:

```rust
/// Smooth union — blends two shapes with radius k.
/// k = 0 → sharp boolean. k > 0 → organic fillet.
fn smooth_union(a: f64, b: f64, k: f64) -> f64 {
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    b * (1.0 - h) + a * h - k * h * (1.0 - h)
}

fn smooth_subtract(a: f64, b: f64, k: f64) -> f64 {
    smooth_union(a, -b, k)  // negate b, then smooth union
}

fn smooth_intersect(a: f64, b: f64, k: f64) -> f64 {
    -smooth_union(-a, -b, k)  // De Morgan on smooth union
}
```

A tendon blending into a bone attachment point is `smooth_union(bone, tendon, 2.0)`.
A fingertip rounding into a finger body is the same operation with a different
blend radius. B-Rep requires explicit fillet surface construction for this. CSG
cannot do it at all.

**Distance note:** Smooth booleans produce a **lower bound** — the field
underestimates true distance everywhere in the blend region. This is safe for
ray marching and meshing, but `shell()` applied after `smooth_union` will
produce walls that are too thick in blend regions and too thin elsewhere. For
bio-inspired shapes, this non-uniformity is often acceptable (organic structures
have variable wall thickness naturally).

**Order matters:** Smooth booleans are **not associative**.
`smooth_union(a, smooth_union(b, c, k), k) ≠ smooth_union(smooth_union(a, b, k), c, k)`.
When composing multiple smooth operations, the order of composition affects the
blend shape. The API should offer an n-ary `smooth_union_all(shapes, k)` variant
for symmetric blending.

### 3.4 Transforms Are Coordinate Substitution

The math behind transforms is coordinate substitution — evaluate the field at
a transformed point instead of the original:

```
translate(f, offset)(p) = f(p - offset)
rotate(f, R)(p)         = f(R⁻¹ · p)
scale_uniform(f, s)(p)  = f(p / s) · s
```

In cf-design, these become `FieldNode::Translate`, `FieldNode::Rotate`, and
`FieldNode::ScaleUniform` nodes in the expression tree. The `Solid` builder
API exposes them as method chains:

```rust
let part = Solid::sphere(5.0)
    .translate(vec3![10.0, 0.0, 0.0])
    .rotate(UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0));
```

### 3.5 Advanced Operations

All expressible as field manipulations:

```rust
// Shell — hollow out a solid to wall thickness t
// ⚠ Requires exact SDF input for uniform wall thickness.
fn shell(sdf: f64, thickness: f64) -> f64 {
    sdf.abs() - thickness
}

// Round — add radius to all edges
// ⚠ Requires exact SDF input for uniform rounding.
fn round(sdf: f64, radius: f64) -> f64 {
    sdf - radius
}

// Offset — grow or shrink a part uniformly
// ⚠ Requires exact SDF input for uniform offset.
// Positive = grow (add material), Negative = shrink (remove material).
// Use with manufacturing clearance: offset(pin, -clearance/2.0)
fn offset(sdf: f64, distance: f64) -> f64 {
    sdf - distance
}

// Onion — create concentric shells
fn onion(sdf: f64, thickness: f64) -> f64 {
    (sdf.abs() - thickness).abs()
}

// Repeat — infinite repetition with spacing s
fn repeat(p: Point3, spacing: Vector3) -> Point3 {
    Point3::from(p.coords.zip_map(&spacing, |c, s| c - s * (c / s).round()))
}

// Twist — rotate cross-section along an axis
// ⚠ Distorts the distance field. Lipschitz constant ≈ sqrt(1 + (k*r)²)
// where r is distance from twist axis. Meshing resolution must be
// increased for large k or large parts.
fn twist(p: Point3, k: f64) -> Point3 {
    let angle = k * p.z;
    let (s, c) = angle.sin_cos();
    point![c * p.x - s * p.y, s * p.x + c * p.y, p.z]
}

// Bend — curve a straight part around an axis
// ⚠ Same Lipschitz distortion as twist.
fn bend(p: Point3, k: f64) -> Point3 {
    let angle = k * p.x;
    let (s, c) = angle.sin_cos();
    point![c * p.x - s * p.y, s * p.x + c * p.y, p.z]
}
```

These operations are compositional — twist a bent shell with repeated holes is
just function composition. No special-case algorithms.

**Distance note on deformations:** Twist and bend distort the distance field —
distances are **overestimated** away from the deformation axis. This means thin
features far from the axis may be **silently lost** during meshing if the grid
is too coarse. The mesher should account for the Lipschitz constant when
computing required resolution (see Section 5.5).

### 3.6 Bio-Inspired Primitives

Shapes that are natural as implicit functions but nightmares in B-Rep:

```rust
// Superellipsoid — tunable between box, cylinder, sphere, diamond
fn superellipsoid(radii: Vector3, n1: f64, n2: f64, p: Point3) -> f64 {
    let x = (p.x / radii.x).abs().powf(2.0 / n2);
    let y = (p.y / radii.y).abs().powf(2.0 / n2);
    let z = (p.z / radii.z).abs().powf(2.0 / n1);
    ((x + y).powf(n2 / n1) + z).powf(n1 / 2.0) - 1.0
}

// Logarithmic spiral — nautilus shell, horn, tendril
fn log_spiral(a: f64, b: f64, thickness: f64, p: Point3) -> f64 {
    let r = (p.x * p.x + p.y * p.y).sqrt();
    let theta = p.y.atan2(p.x);
    let r_spiral = a * (b * theta).exp();
    (r - r_spiral).abs() - thickness
}

// Gyroid — minimal surface lattice for lightweight infill
fn gyroid(scale: f64, thickness: f64, p: Point3) -> f64 {
    let s = scale;
    let val = (s * p.x).sin() * (s * p.y).cos()
            + (s * p.y).sin() * (s * p.z).cos()
            + (s * p.z).sin() * (s * p.x).cos();
    val.abs() - thickness
}
```

A gyroid-infilled bone structure with smooth tendon attachment points? That is
four function calls composed together. In B-Rep, this is not expressible at all.

---

## 4. The Design Primitive: Mechanism

A bio-inspired mechanism is not a single shape — it is an assembly of shapes
connected by joints. The `Mechanism` is the atomic unit of design.

### 4.1 Why Mechanism, Not Shape

The things a user (or AI) designs are mechanisms:
- A 3-finger gripper (5 parts, 6 joints)
- A tendon-driven finger (3 phalanges, 2 revolute joints, 1 cable)
- A bio-inspired leg (femur, tibia, foot, 3 joints, 2 muscles)

The things sim-core simulates are MJCF models:
- Bodies with geometry
- Joints connecting bodies
- Tendons routing through sites
- Actuators driving joints

These are the same structure. A `Mechanism` maps 1:1 to an MJCF model.

### 4.2 Mechanism API

```rust
pub struct Mechanism {
    name: String,
    parts: Vec<Part>,
    joints: Vec<JointDef>,
    tendons: Vec<TendonDef>,
    actuators: Vec<ActuatorDef>,
    print_profile: Option<PrintProfile>,
}

pub struct Part {
    name: String,
    solid: Solid,
    material: Material,
    flex_zones: Vec<FlexZone>,  // compliant mechanism support
}

pub struct Material {
    pub name: String,
    pub density: f64,                         // kg/m³
    pub youngs_modulus: Option<f64>,           // Pa — for flex zone stiffness
    pub color: Option<[f64; 4]>,              // RGBA for visualization
    pub process: Option<ManufacturingProcess>, // FDM, SLA, SLS, etc.
}

pub enum ManufacturingProcess {
    Fdm { layer_height: f64, infill: f64 },
    Sla { layer_height: f64 },
    Sls,
    Machined,
}

pub struct JointDef {
    name: String,
    parent: String,       // part name
    child: String,        // part name
    kind: JointKind,
    anchor: Point3<f64>,
    axis: Vector3<f64>,
    range: Option<(f64, f64)>,
}

pub enum JointKind {
    Revolute,
    Prismatic,
    Ball,
    Free,
}

/// Tendon routing through a mechanism.
/// Defines both the physical channel geometry (carved from parts) and the
/// simulation-side tendon (MJCF <tendon> with <site> waypoints).
pub struct TendonDef {
    name: String,
    /// Ordered waypoints — each references a part and a local-frame position.
    /// The tendon routes through these points in order.
    waypoints: Vec<TendonWaypoint>,
    /// Channel radius — the cylindrical void carved along the tendon path
    /// in each part. Set to 0.0 for surface-routed tendons (no channel).
    channel_radius: f64,
    /// Tendon stiffness for MJCF generation.
    stiffness: Option<f64>,
    /// Tendon damping for MJCF generation.
    damping: Option<f64>,
}

pub struct TendonWaypoint {
    part: String,              // which part this waypoint is on
    position: Point3<f64>,     // local-frame position within the part
}

/// A thin cross-section of a monolithic part that acts as a flexure joint.
/// For simulation, the part is auto-split at flex zones into separate MJCF
/// bodies connected by joints with spring-damper properties derived from
/// the material and cross-section geometry.
pub struct FlexZone {
    name: String,
    /// Plane defining the flex zone cross-section.
    plane: Plane,
    /// Width of the thin section (along the plane normal).
    width: f64,
    /// Joint type to generate at this flex zone.
    kind: JointKind,
    /// Axis of rotation/translation (in part-local frame).
    axis: Vector3<f64>,
}

/// Manufacturing clearance profile for a target process.
pub struct PrintProfile {
    /// Default clearance between mating surfaces (mm).
    /// FDM typical: 0.3, SLA typical: 0.15.
    pub clearance: f64,
    /// Minimum wall thickness the process can produce (mm).
    pub min_wall: f64,
    /// Minimum hole diameter the process can produce (mm).
    pub min_hole: f64,
}
```

### 4.3 Three Outputs From One Definition

```rust
impl Mechanism {
    /// Generate MJCF model for simulation.
    /// Each Part becomes a <body> with a <geom>.
    /// Each JointDef becomes a <joint>.
    /// Each TendonDef becomes a <tendon> with <site> waypoints.
    /// FlexZones auto-split their Part into sub-bodies.
    /// Tendon channels are subtracted from part geometry automatically.
    pub fn to_mjcf(&self, resolution: f64) -> String;

    /// Generate collision shapes for cf-geometry.
    /// During design iteration: SdfGrid (fast, no meshing).
    /// For final simulation: IndexedMesh via dual contouring.
    pub fn to_shapes(&self, mode: ShapeMode) -> Vec<(String, Shape)>;

    /// Generate meshes for 3D printing.
    /// One mesh per Part, per material. Clearances applied if PrintProfile set.
    /// Multi-material parts produce multiple meshes.
    pub fn to_stl_kit(&self, tolerance: f64) -> Vec<(String, IndexedMesh)>;

    /// Validate mechanism against manufacturing constraints.
    /// Checks min wall thickness, min hole diameter, clearances.
    pub fn validate(&self) -> Vec<DesignWarning>;
}

pub enum ShapeMode {
    /// Fast: evaluate field on grid, use Shape::Sdf for collision.
    /// Best for design iteration (no meshing cost).
    Sdf { resolution: f64 },

    /// Accurate: dual contour to mesh, use Shape::TriangleMesh for collision.
    /// Best for final simulation and RL training.
    Mesh { tolerance: f64 },
}

pub enum DesignWarning {
    WallTooThin { part: String, location: Point3<f64>, thickness: f64, min: f64 },
    HoleTooSmall { part: String, location: Point3<f64>, diameter: f64, min: f64 },
    InsufficientClearance { part_a: String, part_b: String, gap: f64, min: f64 },
    FeatureBelowResolution { part: String, feature_size: f64, resolution: f64 },
}
```

### 4.4 Example: Bio-Inspired Gripper

```rust
use cf_design::prelude::*;

// --- Define part geometry using Solid builder (constructs FieldNode trees) ---

// Palm — main body with tendon channels
let palm = Solid::cuboid(vec3![18.0, 22.0, 4.0])
    .subtract(Solid::cylinder(1.5, 50.0).translate(vec3![8., 0., 0.]))   // channel 1
    .subtract(Solid::cylinder(1.5, 50.0).translate(vec3![-8., 0., 0.]))  // channel 2
    .subtract(Solid::cylinder(1.5, 50.0));                                // channel 3

// Finger — bone-like phalanx with organic grip pad
let finger = Solid::capsule(3.0, 12.0)
    .translate(vec3![0., 12., 0.])
    .smooth_union(
        Solid::ellipsoid(vec3![4., 3., 1.5]).translate(vec3![0., 22., 0.]),
        2.0,  // organic blend radius at grip surface
    );

// --- Assemble into a mechanism ---

let pla = Material::new("PLA", 1250.0);  // 1250 kg/m³

let gripper = Mechanism::new("bio_gripper")
    .part("palm", palm, pla.clone())
    .part("finger_1", finger.clone(), pla.clone())
    .part("finger_2", finger.clone().mirror(Axis::X), pla.clone())

    // Knuckle joints — fingers rotate relative to palm
    .joint("knuckle_1", JointDef {
        parent: "palm".into(),
        child: "finger_1".into(),
        kind: JointKind::Revolute,
        anchor: point![10., 20., 0.],
        axis: Vector3::x(),
        range: Some((-0.1, 1.8)),
        ..Default::default()
    })
    .joint("knuckle_2", JointDef {
        parent: "palm".into(),
        child: "finger_2".into(),
        kind: JointKind::Revolute,
        anchor: point![-10., 20., 0.],
        axis: Vector3::x(),
        range: Some((-0.1, 1.8)),
        ..Default::default()
    })

    // Tendon — routes through palm and finger, carves channel automatically
    .tendon("flexor_1", TendonDef {
        waypoints: vec![
            TendonWaypoint { part: "palm".into(), position: point![8., -10., 0.] },
            TendonWaypoint { part: "palm".into(), position: point![8., 18., 0.] },
            TendonWaypoint { part: "finger_1".into(), position: point![0., 5., 0.] },
            TendonWaypoint { part: "finger_1".into(), position: point![0., 20., 0.] },
        ],
        channel_radius: 1.5,
        stiffness: Some(100.0),
        damping: Some(5.0),
    })

    // Manufacturing constraints
    .print_profile(PrintProfile {
        clearance: 0.3,   // FDM clearance
        min_wall: 1.0,    // mm
        min_hole: 2.0,    // mm
    });

// --- Three outputs from one definition ---

// Design iteration: SDF colliders (no meshing, instant feedback)
let shapes = gripper.to_shapes(ShapeMode::Sdf { resolution: 0.5 });

// Production simulation: meshed colliders (triangle-level accuracy)
let shapes = gripper.to_shapes(ShapeMode::Mesh { tolerance: 0.1 });

// Manufacturing: per-part STLs with clearances applied
let stls = gripper.to_stl_kit(0.1);

// Full MJCF model: bodies, joints, tendons, geoms — ready for sim-core
let mjcf = gripper.to_mjcf(0.2);

// Validate against print constraints before sending to printer
let warnings = gripper.validate();
```

---

## 5. Meshing Pipeline

The implicit field must be converted to an `IndexedMesh` for manufacturing
(STL export) and high-fidelity collision (Shape::TriangleMesh). Two algorithms
are relevant:

### 5.1 Marching Cubes

- Evaluates field on a regular grid, extracts triangles at zero-crossings
- Simple to implement, well-understood
- **Weakness**: rounds off sharp features (90-degree edges become beveled)
- For bio-inspired shapes this is acceptable — organic shapes don't have sharp edges

**Variant selection**: The original MC (Lorensen & Cline, 15 cases) has face
ambiguities that produce cracks and topological errors. Phase 1 should use
**MC33** (Chernyaev, corrected by Custodio et al. 2013) or the **asymptotic
decider** (Nielson & Hamann) for face ambiguity resolution. Alternatively,
naive MC + mesh-repair as safety net is acceptable for Phase 1 if MC33
complexity is prohibitive.

### 5.2 Manifold Dual Contouring

- Places vertices inside cells using gradient (normal) information
- Preserves sharp features where they exist (pin holes, flat faces)
- **Must use Manifold DC** (Schaefer, Ju, Warren) — not classical DC, which
  produces non-manifold geometry when surface sheets share a cell
- QEF solver must use SVD with cell-clamping and centroid bias to prevent
  vertex placement at infinity (common with flat surfaces)
- **Analytic gradients strongly preferred** over finite differences — FD
  gradient noise causes false sharp-feature detection and QEF instability

### 5.3 Recommended Approach

**Phase 1**: Marching cubes (MC33 or naive + repair). Simple, correct,
sufficient for organic bio shapes. Feed output through mesh-repair for
manifold guarantee.

**Phase 2**: Manifold Dual Contouring for parts that need sharp features (pin
holes, mating surfaces). Use marching cubes as fallback.

**Phase 3**: Adaptive resolution via octree — fine grid near the surface and
thin features, coarse grid in bulk interior. Requires **interval arithmetic**
to prune octree cells that are entirely inside or outside (see Section 5.6).

### 5.4 Mesh Quality for 3D Printing

3D printers require:
- **Manifold** — no holes, no self-intersections, no non-manifold edges
- **Watertight** — closed surface, consistent winding
- **Reasonable triangle count** — not millions of triangles for a simple part

The pipeline:

```
Implicit field
  → marching cubes / dual contouring (Phase 1/2)
    → mesh-repair (manifold guarantee, degeneracy removal)
      → IndexedMesh (cf-geometry canonical type)
        → Shape::TriangleMesh (collision)
        → mesh-io::write_stl (manufacturing)
```

mesh-repair already exists in the codebase. The meshing algorithm feeds into
the existing pipeline.

### 5.5 Minimum Feature Size

Features thinner than `2 × cell_size` may be silently lost during meshing —
if both sides of a thin wall fall within the same grid cell, no zero-crossing
is detected.

**Constraint**: `min_feature_size > 2 × cell_size` (or equivalently,
`cell_size < min_feature_size / 2`).

The mesher should validate this and emit a `DesignWarning::FeatureBelowResolution`
when the `PrintProfile.min_wall` is close to the meshing resolution. For
deformation operations (twist, bend), the effective resolution must account
for the Lipschitz scaling factor (Section 3.5).

### 5.6 Performance: Interval Arithmetic

At manufacturing resolution (0.05mm cells on a 50mm part = 1000³ = 1 billion
evaluations), naive grid evaluation is prohibitively slow. The key acceleration
is **interval arithmetic pruning**:

1. Evaluate the field over a cell's bounding box using interval arithmetic
   (each coordinate is a range `[lo, hi]` instead of a point).
2. If the result interval is entirely `> 0` → cell is outside, skip.
3. If entirely `< 0` → cell is inside, skip.
4. Otherwise → subdivide and recurse (octree).

This typically eliminates 90%+ of cells from expensive point-wise evaluation.
libfive's entire meshing architecture is built on this principle. Its successor
**Fidget** (also by Matt Keeter) achieves further speedup via JIT compilation
of the expression tree to native SIMD code.

**Architectural implication**: Interval arithmetic requires the field to be
represented as an **expression tree** (inspectable structure), not an opaque
closure (`dyn Fn`). An opaque closure cannot be interval-evaluated. See
Section 8.3 for how this affects the `Solid` representation.

---

## 6. Integration With cf-geometry

cf-geometry already provides the infrastructure that cf-design needs:

| cf-geometry type | Role in cf-design |
|-----------------|-------------------|
| `IndexedMesh` | Meshed output of implicit fields — the universal exchange format |
| `SdfGrid` | Discretized implicit field for fast collision during design iteration |
| `Shape::Sdf` | Collision shape wrapping SdfGrid — sim-core collides against this directly |
| `Shape::TriangleMesh` | Collision shape wrapping meshed output — for high-fidelity simulation |
| `Aabb` | Bounding box for each Part — defines evaluation domain for marching cubes |
| `Bvh` | Acceleration structure built from meshed output |
| `ConvexHull` | Auto-generated via convex decomposition for GJK/EPA collision |

### 6.1 Design Iteration Path (Fast)

```
Solid (FieldNode tree)
  → evaluate on grid → SdfGrid
    → Shape::Sdf { data: Arc<SdfGrid> }
      → sim-core collides via ray marching / gradient descent
```

No meshing. Change a parameter, re-evaluate the field, simulate again instantly.
cf-geometry's `Shape::Sdf` already supports ray_cast and closest_point queries.

### 6.2 Production Path (Accurate)

```
Solid (FieldNode tree)
  → dual contouring → IndexedMesh
    → mesh-repair → clean IndexedMesh
      → bvh_from_mesh() → Bvh
        → Shape::TriangleMesh { mesh: Arc<IndexedMesh>, bvh: Arc<Bvh> }
          → sim-core collides via BVH + Moller-Trumbore
```

Full triangle-level collision accuracy. Same path used for STL export.

### 6.3 Zero New Types

cf-design introduces **zero new geometric types**. Everything flows through
cf-geometry's existing types. This is by design — cf-geometry is the canonical
geometric kernel, and cf-design is a producer of geometry, not a parallel
representation.

---

## 7. Integration With sim-core

### 7.1 MJCF Generation (Primary Path)

`Mechanism::to_mjcf()` generates a complete MJCF XML string that sim-mjcf
can parse via `load_model(xml)`. Each `Part` becomes a `<body>` with a
`<geom>`. Each `JointDef` becomes a `<joint>`. Each `TendonDef` becomes a
`<tendon>` with `<site>` waypoints. Tendons and actuators map to their MJCF
equivalents.

For geom types, the MJCF path uses mesh geoms (`type="mesh"`) with embedded
vertex data — the meshed `IndexedMesh` is serialized as MJCF `<mesh>` asset
elements. This is the most compatible path since sim-mjcf fully supports mesh
geoms.

**Important limitation**: MJCF `type="sdf"` geoms cannot be populated via
MJCF parsing — sim-mjcf always sets `geom_sdf = None` (SDF geoms are
programmatic-only). The MJCF path therefore always meshes parts first.

### 7.2 Direct Model Injection (Fast Path)

For tighter integration and SDF-based collision during design iteration,
cf-design can populate `Model` fields directly:

1. Start with `Model::empty()`
2. Push `geom_type = GeomType::Sdf`
3. Push the `SdfGrid` into `model.sdf_data`
4. Set `geom_sdf[i] = Some(sdf_id)`

sim-core's SDF collision pipeline is fully implemented (`sdf_collide.rs` —
11 shape-pair dispatchers). This path bypasses MJCF serialization and enables
SDF-native collision during design iteration.

**Note**: There is no ergonomic `Model` builder API in sim-core today. Direct
injection requires manually populating parallel arrays (`body_parent`,
`jnt_type`, `geom_type`, `geom_size`, etc.). A future `ModelBuilder` API in
sim-core would improve this.

### 7.3 Mass Properties

Implicit fields enable mass property computation via volumetric integration.
For a field `f` with uniform density `rho`:

```
mass    = rho * ∫∫∫ [f(p) < 0] dp
center  = (1/mass) * rho * ∫∫∫ p * [f(p) < 0] dp
inertia = rho * ∫∫∫ (|p|²I - p*pᵀ) * [f(p) < 0] dp
```

This is evaluated numerically on the same grid used for SdfGrid construction.
No separate mass property computation needed — it falls out of the field
evaluation.

**Note**: sim-core currently has no automatic mass computation for SDF geoms
(falls through to default mass `0.001`). cf-design must compute mass properties
from the implicit field and inject them via explicit `<inertial>` elements in
MJCF or direct `Model` field population.

---

## 8. Ship of Theseus: Upgradability

The `Solid` type (internal to `Part`) is **opaque**. Consumers never see the
internal representation. This enables incremental replacement of the kernel
without breaking any downstream code.

### 8.1 Upgrade Path

```
Phase 1 (ship):
  Internal: FieldNode expression tree (implicit field)
  Meshing:  Marching cubes (MC33) + interval arithmetic pruning
  Queries:  SdfGrid evaluation

Phase 2 (quality):
  Internal: FieldNode expression tree
  Meshing:  Manifold Dual Contouring (preserves sharp features)
  Queries:  SdfGrid + adaptive refinement + analytic gradients

Phase 3 (hybrid, if Fornjot matures):
  Internal: Fornjot B-Rep for analytic shapes, FieldNode for organic
  Meshing:  Fornjot tessellation for B-Rep, dual contouring for implicit
  Queries:  Fornjot exact queries for B-Rep, SdfGrid for implicit

Phase 4 (full B-Rep, if needed):
  Internal: Fornjot B-Rep
  Meshing:  Fornjot tessellation
  Queries:  Fornjot exact queries
  New APIs: fillet(), chamfer(), sweep_along_curve() (only possible with B-Rep)
```

Each phase swaps internals. The public API only **grows** (new methods in later
phases). It never breaks. Consumers see `IndexedMesh` and `Shape` — always.

### 8.2 The Opaque Boundary

```rust
pub struct Solid {
    // Private. Consumers cannot pattern-match.
    // Day 1: holds an expression tree (FieldNode).
    // Day N: could hold a Fornjot Shell.
    inner: SolidRepr,
}

// All interaction through methods:
impl Solid {
    pub fn union(self, other: Solid) -> Solid;
    pub fn subtract(self, other: Solid) -> Solid;
    pub fn smooth_union(self, other: Solid, k: f64) -> Solid;
    pub fn mesh(&self, tolerance: f64) -> IndexedMesh;
    pub fn sdf_grid(&self, resolution: f64) -> SdfGrid;
    pub fn bounds(&self) -> Aabb;
    pub fn evaluate(&self, point: &Point3<f64>) -> f64;
    // Phase 4 additions (only with B-Rep backend):
    // pub fn fillet(&self, edges: &EdgeSelection, radius: f64) -> Solid;
    // pub fn chamfer(&self, edges: &EdgeSelection, distance: f64) -> Solid;
}
```

### 8.3 Expression Tree vs Opaque Closure

This is the most consequential internal architecture decision. Two options:

| | **Opaque closure** (`Box<dyn Fn(Point3) -> f64>`) | **Expression tree** (`enum FieldNode { ... }`) |
|---|---|---|
| Simplicity | Very simple — any Rust closure works | Requires defining all node types |
| Interval arithmetic | **Impossible** — cannot inspect the function | **Possible** — traverse the tree with interval inputs |
| Analytic gradients | **Impossible** — no derivative information | **Possible** — reverse-mode AD on the tree |
| Tape optimization | **Impossible** — opaque black box | **Possible** — CSE, constant folding, dead branch pruning |
| SIMD batching | Limited — one point at a time | **Possible** — evaluate 4-8 points per tree walk |
| Lipschitz analysis | **Impossible** — cannot bound the function | **Possible** — compute per-node Lipschitz constants |
| Custom user functions | Trivially supported | Requires a `UserFn` leaf node |
| Performance ceiling | Low (naive grid walk) | High (libfive/Fidget-class, 100-1000x faster) |

**Decision: Expression tree from Phase 1.**

The performance gap is too large to ignore. An opaque closure forecloses interval
arithmetic, which is the primary meshing acceleration (90%+ cell pruning). Building
on closures and retrofitting to an expression tree later would be a painful rewrite.

The expression tree also enables analytic gradients (needed for dual contouring in
Phase 2) and Lipschitz analysis (needed for deformation-aware resolution scaling).

Custom user functions are supported via a `UserFn` leaf node that wraps a closure
plus an interval-evaluation bound. This preserves the "just write a function"
ergonomics for users who need custom primitives, at the cost of coarser interval
pruning for those specific nodes.

```rust
/// Internal representation — not public API.
enum FieldNode {
    // Primitives
    Sphere { radius: f64 },
    Cuboid { half_extents: Vector3<f64> },
    Cylinder { radius: f64, half_height: f64 },
    Capsule { radius: f64, half_height: f64 },
    Torus { major: f64, minor: f64 },
    Pipe { path: Polyline, radius: f64 },
    // ... all Section 9 primitives

    // Boolean
    Union(Box<FieldNode>, Box<FieldNode>),
    Subtract(Box<FieldNode>, Box<FieldNode>),
    Intersect(Box<FieldNode>, Box<FieldNode>),
    SmoothUnion(Box<FieldNode>, Box<FieldNode>, f64),

    // Transforms
    Translate(Box<FieldNode>, Vector3<f64>),
    Rotate(Box<FieldNode>, UnitQuaternion<f64>),
    ScaleUniform(Box<FieldNode>, f64),

    // Domain
    Shell(Box<FieldNode>, f64),
    Round(Box<FieldNode>, f64),
    Offset(Box<FieldNode>, f64),
    Twist(Box<FieldNode>, f64),
    Repeat(Box<FieldNode>, Vector3<f64>),

    // Escape hatch for custom functions
    UserFn {
        eval: Box<dyn Fn(Point3<f64>) -> f64 + Send + Sync>,
        /// Conservative interval bound — user provides a function that
        /// returns (min, max) over a bounding box. If None, interval
        /// pruning is disabled for this subtree.
        interval: Option<Box<dyn Fn(&Aabb) -> (f64, f64) + Send + Sync>>,
        bounds: Aabb,
    },
}
```

---

## 9. SDF Primitive Library

cf-design ships with a comprehensive set of SDF primitives. These are the
building blocks that humans and AIs compose to create parts.

### 9.1 Geometric Primitives

All primitives operate in local space (centered at origin, identity orientation).
Transforms are applied via coordinate substitution (Section 3.4).

| Primitive | Parameters | Notes |
|-----------|-----------|-------|
| `sphere` | radius | Exact Euclidean distance |
| `cuboid` | half_extents (Vector3) | Exact distance, sharp edges |
| `cylinder` | radius, half_height | Aligned to Z axis |
| `capsule` | radius, half_height | Cylinder with hemispherical caps |
| `ellipsoid` | radii (Vector3) | Approximate distance (not exact SDF) |
| `torus` | major_radius, minor_radius | Donut shape |
| `cone` | radius, height | Truncated cone with optional top radius |
| `plane` | normal, offset | Half-space |

### 9.2 Path-Based Primitives

Essential for tendon channels, hoses, spines, and any tubular structure:

| Primitive | Parameters | Use Case |
|-----------|-----------|----------|
| `pipe` | polyline path, radius | Cylindrical tube along a path — tendon channels, wire routing |
| `pipe_spline` | control points, radius | Smooth pipe along a Catmull-Rom/Bezier spline |
| `loft` | cross_sections: &[(f64, CrossSection)] | Smoothly varying cross-section along an axis — femur, tapered links |

The `pipe` and `pipe_spline` primitives compute `distance_to_path(p) - radius`,
where `distance_to_path` is the minimum distance from the query point to the
centerline curve. For a polyline, this is the minimum distance to each segment.
For a spline, this uses iterative closest-point refinement.

The `loft` primitive interpolates cross-section parameters (e.g., ellipse radii)
along a path using cubic interpolation, then evaluates the interpolated
cross-section at each query point's projected position along the path.

### 9.3 Bio-Inspired Primitives

| Primitive | Parameters | Use Case |
|-----------|-----------|----------|
| `superellipsoid` | radii, n1, n2 | Tunable between box/cylinder/sphere/diamond. **Approximate distance** (not exact SDF, like ellipsoid). |
| `log_spiral` | a, b, thickness | Shells, horns, tendrils |
| `gyroid` | scale, thickness | Lightweight lattice infill |
| `schwarz_p` | scale, thickness | Minimal surface infill (alternative to gyroid) |
| `helix` | radius, pitch, thickness | Springs, coils, DNA-like structures |

### 9.4 Boolean Operations

| Operation | Parameters | Behavior |
|-----------|-----------|----------|
| `union` | a, b | Sharp union (min) |
| `subtract` | a, b | Sharp subtraction (max of a, -b) |
| `intersect` | a, b | Sharp intersection (max) |
| `smooth_union` | a, b, k | Organic blend with radius k |
| `smooth_subtract` | a, b, k | Smooth subtraction with radius k |
| `smooth_intersect` | a, b, k | Smooth intersection with radius k |
| `smooth_union_all` | shapes, k | N-ary symmetric smooth union (avoids associativity issue) |

### 9.5 Domain Operations

| Operation | Parameters | Effect |
|-----------|-----------|--------|
| `translate` | offset | Move shape |
| `rotate` | quaternion | Rotate shape |
| `scale_uniform` | factor | Uniform scale (preserves SDF property) |
| `mirror` | axis | Reflect across plane |
| `shell` | thickness | Hollow out to wall thickness |
| `round` | radius | Add rounding to all edges |
| `elongate` | half_extents | Stretch shape along axes |
| `repeat` | spacing | Infinite repetition |
| `repeat_bounded` | spacing, count | Finite repetition |
| `twist` | rate | Rotate cross-section along axis |
| `bend` | rate | Curve along axis |
| `offset` | distance | Grow/shrink uniformly (clearance adjustment) |

---

## 10. Implementation Sessions

26 sessions across 5 phases. Each session is scoped to one conversation.

**Terminology note**: "Phase" in this section refers to the implementation
schedule. Sections 5.3 and 8.1 use "Phase" for subsystem evolution roadmaps
(meshing quality tiers, upgrade path) — those are different numbering systems.

### Phase 1: Foundation (Sessions 1–6) — COMPLETE

**Goal**: Expression tree, field evaluation, marching cubes meshing, core primitives.

**Branch**: `cf-design-phase1` | **Tests**: 207 pass | **Completed**: 2026-03-16

**Source files**: `crates/cf-design/src/{lib.rs, field_node.rs, evaluate.rs, interval.rs, solid.rs, bounds.rs, mesher.rs}`

**What shipped**:
- `FieldNode` expression tree (pub(crate)) with 10 primitives, 7 booleans, 4 transforms, 4 domain ops, UserFn escape hatch
- `Solid` opaque public API with full builder pattern: primitives → booleans → transforms → domain ops → mesh/evaluate/sdf_grid
- Point evaluation, interval evaluation (conservative bounds), AABB bounds computation
- Marching cubes mesher with interval arithmetic pruning (80%+ cell pruning on simple primitives, 60-79% on composed trees)
- `Solid::mesh(tolerance) -> IndexedMesh` — watertight, manifold, CCW winding
- `Solid::sdf_grid(resolution) -> Option<SdfGrid>` — uniform grid evaluation with padding
- `Solid::bounds() -> Option<Aabb>`, `Solid::evaluate(point) -> f64`, `Solid::evaluate_interval(aabb) -> (f64, f64)`
- Catmull-Rom spline with Newton refinement for PipeSpline
- Edge vertex deduplication cache in mesher for proper indexed meshes

**Deviations from spec**:
- Crate structure is flat (`src/*.rs`) instead of nested `src/field/`, `src/mesh/` modules. Simpler for Phase 1 scope; can reorganize when Phase 2 adds mechanisms.
- `sdf_grid` returns `Option<SdfGrid>` (not bare `SdfGrid`) — `None` for infinite geometry (bare Plane). Matches `bounds() -> Option<Aabb>` pattern.
- Used classic MC (Lorensen & Cline) instead of MC33 — sufficient for organic bio shapes. MC33 or dual contouring deferred to Phase 4 (Session 20).
- No mesh-repair fallback needed — MC output is inherently manifold with edge deduplication.
- No `thiserror` dependency — not needed yet (no error types in Phase 1).

**Session 1: Crate Skeleton + Geometric Primitives** — COMPLETE
- Delivered: 8 geometric primitives (Sphere, Cuboid, Cylinder, Capsule, Ellipsoid, Torus, Cone, Plane). `FieldNode` expression tree. Point + interval evaluation for all. `Solid` opaque type with constructors.

**Session 2: Boolean Operations + Transforms** — COMPLETE
- Delivered: 7 boolean variants (Union, Subtract, Intersect, SmoothUnion, SmoothSubtract, SmoothIntersect, SmoothUnionAll). 4 transform variants (Translate, Rotate, ScaleUniform, Mirror). Log-sum-exp for order-independent SmoothUnionAll.

**Session 3: Domain Operations + UserFn** — COMPLETE
- Delivered: 4 domain ops (Shell, Round, Offset, Elongate). UserFn leaf node with optional interval bound closure. Full interval evaluation for all.

**Session 4: Path-Based Primitives** — COMPLETE
- Delivered: Pipe (polyline path) + PipeSpline (Catmull-Rom spline). Newton refinement for closest-point-on-spline. Lipschitz-corrected interval evaluation.

**Session 5: Marching Cubes Mesher** — COMPLETE
- Delivered: Marching cubes with interval arithmetic pruning + edge vertex cache. `Solid::mesh(tolerance)`, `Solid::bounds()`. Sphere pruning 80%+. Mesh topology validation (watertight, manifold, CCW winding).

**Session 6: SDF Grid Evaluator + Integration Tests** — COMPLETE
- Delivered: `Solid::sdf_grid(resolution)`. 6 integration mesh tests (composed trees → valid mesh). 3 pruning ratio tests on composed trees (union 72.5%, subtract 79.3%, smooth_union_all 61.1%). 4 SdfGrid correctness tests.

### Phase 2: Mechanism (Sessions 7–12)

**Goal**: Multi-part assemblies with joints and tendons, MJCF generation.

**Session 7: Part + Joint + Material + Print Types**
- Scope: `Material` type (density, Young's modulus, color, manufacturing
  process). `Part` struct (name, solid, material, flex_zones). `JointDef`,
  `JointKind` (Revolute, Prismatic, Ball, Free). `PrintProfile` (clearance,
  min_wall, min_hole). `DesignWarning` enum. Unit tests for type construction
  and field validation (e.g., range min < max, positive clearance).
- Entry: Session 1 complete (Solid type exists)
- Exit: `cargo test -p cf-design` passes. All types constructible with valid
  and invalid inputs handled.

**Session 8: Tendon + Actuator + Channel Subtraction**
- Scope: `TendonDef`, `TendonWaypoint`. `ActuatorDef`. Tendon channel
  subtraction algorithm — given a tendon's waypoint path through a part,
  subtract a pipe channel from the part's solid. Tests: part with tendon
  channel has correct void geometry; channel follows waypoint path; multiple
  tendons through same part produce independent channels.
- Entry: Sessions 4, 7 complete (pipe primitives + Part type)
- Exit: `cargo test -p cf-design` passes. Channel subtraction produces
  correct geometry verified by field evaluation along channel centerline.

**Session 9: Mechanism Builder API**
- Scope: `Mechanism` struct. Builder pattern: `.part()`, `.joint()`,
  `.tendon()`, `.actuator()`, `.print_profile()`. Validation: joint references
  valid parts, tendon waypoints reference valid parts, no orphan parts, joint
  anchor within part bounds. Tests for builder ergonomics and validation error
  messages. The bio-gripper example from Section 4.4 should compile.
- Entry: Sessions 7, 8 complete
- Exit: `cargo test -p cf-design` passes. Bio-gripper example compiles and
  validates. Invalid mechanisms produce clear errors.

**Session 10: MJCF Generation**
- Scope: `Mechanism::to_mjcf(resolution) -> String`. Each Part → `<body>` +
  `<geom type="mesh">`. Each JointDef → `<joint>`. Each TendonDef →
  `<tendon>` + `<site>` waypoints. Each ActuatorDef → `<actuator>`. Mesh
  parts via `Solid::mesh()`, embed as MJCF `<mesh>` asset elements.
  Integration test: generated XML is well-formed and `sim-mjcf` can parse it.
- Entry: Sessions 5, 9 complete (mesher + Mechanism)
- Exit: `cargo test -p cf-design` passes. Generated MJCF parseable by
  sim-mjcf. Round-trip: build Mechanism → to_mjcf → parse → inspect model
  structure matches expectation.

**Session 11: Shape + STL Generation + Mass Properties**
- Scope: `Mechanism::to_shapes(ShapeMode) -> Vec<(String, Shape)>`. SDF mode:
  evaluate field on grid → `SdfGrid` → `Shape::Sdf`. Mesh mode: mesh →
  `IndexedMesh` + `Bvh` → `Shape::TriangleMesh`.
  `Mechanism::to_stl_kit(tolerance) -> Vec<(String, IndexedMesh)>` with
  clearance offsets applied via `Solid::offset()`. Volumetric mass computation
  from implicit field (mass, center of mass, inertia tensor via grid
  integration). Tests for each output path.
- Entry: Sessions 5, 6, 9 complete (mesher + SdfGrid + Mechanism)
- Exit: `cargo test -p cf-design` passes. Shapes usable by sim-core. STLs
  reflect clearance offsets. Mass properties within 2% of analytic values for
  simple primitives.

**Session 12: Validation + Model Injection + Integration**
- Scope: `Mechanism::validate() -> Vec<DesignWarning>`. Checks: wall thickness
  (via medial axis sampling), hole diameter, inter-part clearances,
  feature-below-resolution. Direct `Model` injection path (`Model::empty()` +
  push geoms/bodies/joints, bypassing MJCF). Full integration test:
  `Mechanism` → `to_mjcf()` → sim-mjcf parse → sim-core `step()`.
- Entry: Sessions 10, 11 complete
- Exit: `cargo test -p cf-design` passes. Full Phase 2 exit criteria met:
  a multi-part mechanism (2-finger gripper with tendon channels) can be
  defined in code, simulated in sim-core, exported as printable STLs with
  manufacturing clearances, and validated against print constraints.

### Phase 3: Bio-Inspired Library (Sessions 13–18)

**Goal**: Rich primitive library for organic shapes, advanced operations.

**Session 13: Bio-Inspired Primitives**
- Scope: `FieldNode` variants for Superellipsoid, LogSpiral, Gyroid, SchwarzP,
  Helix. Point and interval evaluation for each. Analytic gradient formulas
  for each new node type (wired into traversal in Session 19). `Solid` builder
  methods (`superellipsoid()`, `log_spiral()`, `gyroid()`, `schwarz_p()`,
  `helix()`). Tests for each primitive's field correctness and meshability.
- Entry: Session 5 complete (mesher needed to verify meshability)
- Exit: `cargo test -p cf-design` passes. All bio primitives mesh to valid
  manifold geometry.

**Session 14: Loft + Twist + Bend**
- Scope: Loft primitive — variable cross-section along a path via cubic
  interpolation of cross-section parameters. `FieldNode` variants for Twist
  and Bend domain operations. Point and interval evaluation with Lipschitz
  constant computation for twist/bend. Analytic gradient formulas for each
  new node type (wired into traversal in Session 19). `Solid` builder methods
  (`loft()`, `twist()`, `bend()`). Tests including Lipschitz-aware resolution
  warnings.
- Entry: Sessions 4, 5 complete (path infrastructure + mesher for testing)
- Exit: `cargo test -p cf-design` passes. Lofted shapes mesh correctly.
  Twist/bend produce correct deformations. Lipschitz constants computed.

**Session 15: Repeat + Lipschitz-Aware Resolution**
- Scope: `FieldNode` variants for Repeat (infinite) and RepeatBounded (finite
  count). Lipschitz constant propagation through expression tree (each node
  type reports its Lipschitz factor). Resolution scaling recommendations
  based on accumulated Lipschitz constant. Integration with mesher for
  adaptive cell sizing near high-Lipschitz regions. Analytic gradient formulas
  for Repeat/RepeatBounded (wired into traversal in Session 19). Tests for
  repeated patterns and resolution adaptation.
- Entry: Sessions 5, 14 complete (mesher + deformation ops)
- Exit: `cargo test -p cf-design` passes. Repeated patterns mesh correctly.
  Thin features in high-Lipschitz regions are not silently lost.

**Session 16: Lattice Infill + Variable-Radius Blending**
- Scope: Lattice infill operations — gyroid/schwarz_p intersected with solid
  envelope for lightweight internal structure. Variable-radius smooth blending
  (blend radius varies spatially via a scalar field). `Solid` builder methods
  (`infill()`, `smooth_union_variable()`). Tests for infilled parts
  (correct internal structure, watertight exterior) and variable blending.
- Entry: Session 13 complete (gyroid/schwarz_p primitives)
- Exit: `cargo test -p cf-design` passes. Infilled parts mesh correctly with
  connected internal lattice and intact outer shell.

**Session 17: FlexZone Auto-Splitting**
- Scope: `FlexZone` auto-splitting — identify thin cross-sections in a
  monolithic Part, split into sub-bodies, generate joints with spring-damper
  properties derived from material Young's modulus and cross-section geometry.
  Tests: verify split geometry via field evaluation (sub-bodies have correct
  bounds, thin section correctly identified, spring-damper stiffness matches
  analytic expectation for simple beam geometries).
- Entry: Sessions 9, 14 complete (Mechanism + deformation ops)
- Exit: `cargo test -p cf-design` passes. FlexZone splitting produces correct
  sub-body geometry with physically plausible joint stiffness. Full MJCF
  integration deferred to Session 18.

**Session 18: Parameterized Part Templates**
- Scope: Parameterized part templates (`finger()`, `link()`, `bracket()`) —
  helper functions returning pre-composed `Solid` + `Part` configurations
  with sensible defaults. Templates compose primitives, booleans, organic
  blends, and optionally FlexZones/lattice infill. Full Phase 3 integration
  test: bio-inspired gripper with organic blends, gyroid-infilled bones,
  smooth tendon channels, and compliant finger joints — defined, simulated,
  and exported for printing.
- Entry: Sessions 12, 16, 17 complete (Phase 2 pipeline + lattice + FlexZone)
- Exit: `cargo test -p cf-design` passes. Full Phase 3 exit criteria met:
  a bio-inspired gripper with organic blends, gyroid-infilled bones, smooth
  tendon channels, and compliant finger joints can be defined, simulated,
  and exported for printing.

### Phase 4: Mesh Quality + Performance (Sessions 19–23)

**Goal**: Production-quality meshing for manufacturing, performance at scale.

**Session 19: Analytic Gradient Computation**
- Scope: Reverse-mode AD traversal infrastructure on `FieldNode` expression
  tree. `Solid::gradient(point) -> Vector3<f64>`. Implement gradient (∇f) for
  all Phase 1 node types (primitives, booleans, transforms, domain ops,
  path-based). Wire in gradient formulas from Phase 3 sessions (S13–S15) if
  those sessions have completed. Tests: analytic gradient vs finite
  differences for all covered node types.
  **Note**: Phase 3 sessions (S13–S15) prepare gradient formulas for their
  node types. If those sessions have not yet run when S19 executes, their
  gradients are added when they run, following the pattern established here.
- Entry: Session 6 complete
- Exit: `cargo test -p cf-design` passes. Gradients match finite differences
  within 1e-6 tolerance for all Phase 1 node types.

**Session 20: Manifold Dual Contouring + QEF**
- Scope: Manifold DC algorithm (Schaefer, Ju, Warren — not classical DC).
  SVD-based QEF solver with cell-clamping and centroid bias to prevent vertex
  placement at infinity. Uses analytic gradients from Session 19. Tests:
  sharp feature preservation on cuboids (edges, corners), pin holes (circular
  features), flat faces.
- Entry: Session 19 complete
- Exit: `cargo test -p cf-design` passes. DC meshes preserve sharp features
  that marching cubes rounds off. Meshes are manifold.

**Session 21: Octree-Adaptive Meshing**
- Scope: Octree construction with interval arithmetic cell pruning. Adaptive
  refinement — fine cells near the surface and thin features, coarse cells in
  bulk interior/exterior. Integration with DC mesher (Session 20). Tests:
  resolution adaptation correctness, cell count reduction vs uniform grid.
- Entry: Session 20 complete
- Exit: `cargo test -p cf-design` passes. Adaptive meshing reduces cell count
  by 10x+ vs uniform grid while preserving surface accuracy within tolerance.

**Session 22: SIMD + Multithreaded Evaluation**
- Scope: Batch 4–8 point evaluations per expression tree walk (SIMD lanes).
  Rayon-based multithreaded grid/octree evaluation. Performance benchmarks:
  naive single-thread vs SIMD vs multithreaded on reference parts.
- Entry: Session 5 complete (applicable to any mesher)
- Exit: `cargo test -p cf-design` passes. Measurable speedup on benchmark
  parts (target: 4x+ from SIMD, near-linear thread scaling).

**Session 23: Simplification + Tolerance Meshing + 3MF**
- Scope: Mesh simplification pass (edge collapse with quadric error metric).
  Tolerance-driven meshing — user specifies max surface deviation, mesher
  adapts resolution automatically. Integration test with mesh-io's 3MF export
  (multi-material support). Tests: simplified meshes within tolerance, 3MF
  round-trip.
- Entry: Sessions 20, 21 complete
- Exit: `cargo test -p cf-design` passes. Full Phase 4 exit criteria met:
  meshed parts are manifold, watertight, preserve sharp features at specified
  tolerance. 50mm part at 0.1mm resolution meshes in under 5 seconds.

### Phase 5: Differentiable Design (Sessions 24–26)

**Goal**: Gradient-based design optimization through simulation.

**Session 24: Parameterized Solid**
- Scope: Design variables with named parameters. Parameterized `FieldNode`
  variants (parameters by name instead of literal constants).
  `Solid::with_param(name, value)` for declaring parameters.
  `Solid::set_param(name, value)` for re-evaluation without rebuilding the
  tree. Tests: parameterized sphere radius, parameterized blend radius —
  verify re-evaluation produces correct field after parameter change.
- Entry: Session 6 complete
- Exit: `cargo test -p cf-design` passes. Parameterized solids re-evaluate
  correctly after parameter changes.

**Session 25: Field Gradient w.r.t. Design Parameters**
- Scope: ∂f/∂params via chain rule through expression tree. Extends Session
  19's spatial gradient (∇f w.r.t. x,y,z) to parameter-space gradient
  (∂f/∂param_i). Tests: parameter gradients match finite differences for
  parameterized primitives, booleans, and transforms.
- Entry: Sessions 19, 24 complete
- Exit: `cargo test -p cf-design` passes. Parameter gradients within 1e-6 of
  finite differences.

**Session 26: sim-core Integration + Optimization Loop**
- Scope: Connect parameter gradients to sim-core's `mj_derivatives` for
  end-to-end design-through-physics gradient. Design optimization loop:
  objective → simulate → gradient → update params → re-mesh → repeat.
  Integration test: parameterized gripper finger optimized for grasp force.
- Entry: Sessions 12, 25 complete (Mechanism integration + param gradients)
- Exit: `cargo test -p cf-design` passes. Full Phase 5 exit criteria met:
  a parameterized gripper finger can be optimized to maximize grasp force
  by backpropagating through the simulation.

### Session Dependencies

Each session's prerequisites (verified against entry criteria):

```
S1:  (none)              — cf-geometry exists
S2:  S1                  — booleans compose primitives
S3:  S1                  — domain ops modify primitives
S4:  S1                  — path primitives independent of booleans/domain
S5:  S1, S2, S3, S4      — mesher needs all node types + interval eval
S6:  S5                  — SDF grid + integration tests need mesher
S7:  S1                  — Part wraps Solid; Material defined here
S8:  S4, S7              — tendon channels use pipe primitives + Part
S9:  S7, S8              — Mechanism assembles Parts + Joints + Tendons
S10: S5, S9              — MJCF generation meshes parts
S11: S5, S6, S9          — shape/STL gen needs mesher + SdfGrid + Mechanism
S12: S10, S11            — validation + integration needs all outputs
S13: S5                  — bio primitives need mesher to verify meshability
S14: S4, S5              — loft reuses path infra; mesher needed for testing
S15: S5, S14             — repeat needs mesher; Lipschitz needs deformation ops
S16: S13                 — lattice intersects gyroid/schwarz_p with solid
S17: S9, S14             — FlexZone needs Mechanism + deformation ops
S18: S12, S16, S17       — templates + integration test needs Phase 2 pipeline
S19: S6                  — gradient infra + Phase 1 node gradients
S20: S19                 — DC uses analytic gradients
S21: S20                 — octree adaptive DC
S22: S5                  — SIMD/threading applicable to any mesher
S23: S20, S21            — simplification on DC output
S24: S6                  — parameterized Solid extends evaluation
S25: S19, S24            — param gradients need AD + parameterized Solid
S26: S12, S25            — sim-core integration needs mechanism + param gradients
```

**Parallelism opportunities:**
- S2, S3, S4 can all start after S1 (three independent workstreams)
- S7 can start after S1 (Phase 2 types overlap with Phase 1 meshing)
- S13, S14 can start after S5 (Phase 3 primitives overlap with Phase 2)
- S19, S22, S24 can start after S5/S6 (Phases 4–5 foundations overlap with Phase 2)
- Phase 3 (S13–S18) and Phase 4 (S19–S23) can significantly overlap

### Kill Signals

Conditions that mean the spec needs fundamental rework:

1. **Interval pruning failure.** If interval evaluation prunes <50% of cells
   on simple primitives (sphere, cuboid), the expression tree architecture is
   not delivering its key advantage over opaque closures. **Test**: Session 5.

2. **MC mesh quality collapse.** If marching cubes output requires >50% of
   triangles to be repaired by mesh-repair, the mesher is fundamentally broken
   and needs a different algorithm or approach. **Test**: Session 5.

3. **MJCF round-trip failure.** If generated MJCF cannot be parsed by sim-mjcf
   without changes to sim-mjcf's parser, the MJCF generation approach is wrong
   and needs to be redesigned. **Test**: Session 10.

4. **Path-based SDF non-manifold.** If pipe/pipe_spline SDFs produce
   non-manifold meshes at reasonable resolution (cell_size < radius/2), the
   closest-point-on-path algorithm needs fundamental rework. **Test**: Session 5.

5. **Session 5 exceeds 2 conversations.** Marching cubes is the hardest
   algorithm in Phase 1. If it cannot be completed in one session (with one
   continuation allowed), the session scoping is wrong — split further or
   reduce scope.

**Threshold**: If Phase 1 takes more than 8 sessions (2 over budget), stop and
reassess session scoping for all subsequent phases.

### Incremental Value

Each phase is independently positive. No valley of despair.

| Stop after | State | Value |
|------------|-------|-------|
| Phase 1 | `Solid` with composed primitives, meshing, SdfGrid. | **Positive**: AI can code-generate geometry. Meshes flow into cf-geometry. |
| Phase 2 | Mechanisms with joints, tendons, MJCF. | **Strongly positive**: design → simulate → print pipeline exists. |
| Phase 3 | Bio-inspired primitives, lattice infill, compliant mechanisms. | **Very positive**: organic shapes, gyroid infill, flex zones. |
| Phase 4 | Production meshing, sharp features, performance. | **Maximum for manufacturing**: print-ready output at scale. |
| Phase 5 | Differentiable design optimization. | **Maximum**: gradient-based design through simulation. |

**Stopping points within Phase 2**: Sessions 10, 11, and 12 are each
independently valuable once Session 9 is complete. Session 10 alone provides
design → simulate (via MJCF). Session 11 adds direct shape generation + STL
export. Session 12 adds validation + a faster Model injection path. Any of
these is a valid stopping point.

---

## 11. Crate Structure

### 11.1 Current (Phase 1)

Flat module layout — simple and sufficient for the current scope. Will
reorganize into nested modules when Phase 2 adds mechanisms.

```
crates/cf-design/
├── Cargo.toml
└── src/
    ├── lib.rs              Crate root — re-exports Solid, deny(clippy::unwrap_used, ...)
    ├── field_node.rs       FieldNode expression tree enum (pub(crate))
    ├── evaluate.rs         Point evaluation — evaluate(Point3) -> f64
    ├── interval.rs         Interval evaluation — evaluate_interval(Aabb) -> (f64, f64)
    ├── bounds.rs           AABB computation — bounds() -> Option<Aabb>
    ├── mesher.rs           Marching cubes + interval pruning + edge cache
    └── solid.rs            Opaque Solid type, public builder API, sdf_grid
```

### 11.2 Planned (Phase 2+)

```
crates/cf-design/
├── Cargo.toml
├── src/
│   ├── lib.rs
│   ├── field_node.rs       → may split into field/ modules as node count grows
│   ├── evaluate.rs
│   ├── interval.rs
│   ├── bounds.rs
│   ├── mesher.rs           → may split into mesh/ when DC arrives (Session 20)
│   ├── solid.rs
│   ├── mechanism/          Phase 2 (Sessions 7–12)
│   │   ├── mod.rs          Mechanism type, builder API
│   │   ├── part.rs         Part (named solid + material + flex zones)
│   │   ├── material.rs     Material, ManufacturingProcess
│   │   ├── joint.rs        JointDef, JointKind
│   │   ├── tendon.rs       TendonDef, TendonWaypoint, channel subtraction
│   │   ├── actuator.rs     ActuatorDef (motor, muscle)
│   │   ├── mjcf.rs         MJCF XML generation
│   │   ├── mass.rs         Volumetric mass property computation
│   │   ├── validate.rs     DesignWarning, manufacturing constraint checks
│   │   └── print.rs        PrintProfile, clearance application
│   └── export/
│       ├── stl.rs          Per-part STL generation
│       └── shapes.rs       cf-geometry Shape generation (SdfGrid + TriangleMesh)
└── tests/                  Integration tests (if needed beyond inline tests)
```

### 11.3 Dependencies

```toml
[dependencies]
cf-geometry = { workspace = true }   # IndexedMesh, SdfGrid, Shape, Aabb
nalgebra    = { workspace = true }   # Linear algebra

[dev-dependencies]
approx      = { workspace = true }   # Float comparison
```

**Layer 0 purity**: Zero Bevy, zero GPU, zero framework dependencies.

---

## 12. Reference Projects

| Project | What it does | What we learn from it |
|---------|-------------|----------------------|
| [libfive](https://libfive.com/) | Open-source implicit CAD kernel (Matt Keeter) | Architecture, meshing algorithms, API design |
| [nTop](https://www.ntop.com/) | Commercial implicit modeling for bio/lattice | Proof that implicit works for manufacturing |
| [Inigo Quilez SDF](https://iquilezles.org/articles/distfunctions/) | Comprehensive SDF primitive reference | Distance function implementations |
| [Fornjot](https://www.fornjot.app/) | Rust B-Rep CAD kernel | Potential future Ship of Theseus swap target |
| [OpenSCAD](https://openscad.org/) | Code-first CSG modeler | UX inspiration for code-first design |
| [Curv](https://github.com/curv3d/curv) | Functional geometry language | Language design for SDF composition |
| [Fidget](https://github.com/mkeeter/fidget) | JIT-compiled implicit surface evaluator (Keeter) | Expression tree + interval arithmetic + SIMD — the performance architecture to emulate |
| [IQ smooth min](https://iquilezles.org/articles/smin/) | Smooth minimum reference (Inigo Quilez) | Distance correctness analysis for smooth booleans |

### 12.1 Key Papers

- Keeter, M. (2020). "Massively Parallel Rendering of Complex Closed-Form Implicit Surfaces" — GPU evaluation of implicit fields
- Ju, T. et al. (2002). "Dual Contouring of Hermite Data" — sharp-feature-preserving meshing
- Schaefer, Ju, Warren (2007). "Manifold Dual Contouring" — non-manifold fix for dual contouring
- Custodio et al. (2013). "Practical considerations on MC33 topological correctness" — MC33 bug fixes
- Lorensen & Cline (1987). "Marching Cubes" — the foundational isosurface algorithm
- Hart, J.C. (1996). "Sphere Tracing" — ray marching for implicit surfaces
- Barbier (2025). "Lipschitz Pruning for F-Rep CSG Trees" — hierarchical acceleration for complex trees

---

## 13. Scope Boundaries

Explicit statements about what cf-design does NOT target, to prevent scope
creep and set correct expectations:

1. **Soft/deformable bodies** — cf-design targets rigid-body mechanisms.
   Pneumatic soft grippers, silicone actuators, and cloth require FEM or
   soft-body solvers (future `sim-fem` or `sim-soft` crate). The geometry
   (SDF with internal voids) is expressible; the simulation is not.

2. **Precision mechanical elements** — Involute gear teeth, ball screws,
   bearings, and other elements requiring sub-0.01mm tolerance are
   limited by SDF grid resolution and meshing approximation. These are better
   served by imported B-Rep geometry (STEP/IGES via mesh-io) or the Phase 3
   hybrid approach. cf-design is optimized for organic/bio shapes, not
   watchmaking.

3. **Parametric history** — cf-design does not maintain a parametric history
   tree (like Fusion 360's timeline). The `Solid` is a live expression tree,
   but there is no undo/redo, no feature suppression, no design intent
   capture beyond the code itself. The code IS the parametric history.

4. **GUI / direct manipulation** — cf-design is code-first. A Bevy-based
   visual editor could be built on top (Layer 1), but cf-design itself is
   Layer 0 (pure Rust, no framework dependencies).

---

## 14. Open Questions

1. **Non-uniform scale**: Non-uniform scaling breaks the distance property
   (Lipschitz constant becomes direction-dependent). Phase 1 restricts to
   uniform scale. If non-uniform scale is needed later, it produces a bound
   (not exact distance), and `shell`/`round`/`offset` must be disallowed after
   non-uniform scale.

2. ~~**Gradient computation**~~: **Resolved** — deferred to Phase 4 (Session
   19). Phase 1 uses marching cubes which doesn't need gradients. Dual
   contouring (Session 20) uses analytic gradients from Session 19.

3. ~~**Lattice structures for infill**~~: **Resolved** — first-class in
   cf-design (Session 13 for gyroid/schwarz_p primitives, Session 16 for
   lattice infill operations). Primary bio-inspired use case warrants
   first-class support.

4. **Assembly constraints**: Beyond joints, should cf-design support
   geometric constraints (coaxial, tangent, coincident) for defining how
   parts relate? Or is this over-engineering for the code-first use case
   where the user specifies positions directly?

5. **Multi-material zones**: Should a single `Part` support multiple material
   regions (via a material field function `material(p) -> MaterialId`), or
   should multi-material parts be expressed as overlapping single-material
   parts? The former is more elegant but requires material-aware meshing and
   3MF export. The latter is simpler but loses single-part semantics.

6. ~~**Compliant mechanism auto-splitting**~~: **Resolved** — planar cuts at
   the `FlexZone` plane, with geometry analysis to compute effective hinge
   stiffness from Young's modulus and cross-section geometry (Session 17).
   Spring-damper properties are derived analytically for simple beam
   geometries.

---

## 15. Decision Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-03-15 | Implicit surfaces over B-Rep and CSG | Best fit for AI-native bio-inspired design. Organic blends are free. AI generates functions more naturally than topology. |
| 2026-03-15 | Mechanism as design primitive | Maps 1:1 to MJCF. One definition → simulate + print. |
| 2026-03-15 | Expression tree, not opaque closure | Required for interval arithmetic (90%+ meshing speedup), analytic gradients, Lipschitz analysis. Closures foreclose all three. |
| 2026-03-15 | Ship of Theseus upgradeability | `Solid` is opaque. Internals can be swapped to Fornjot B-Rep without API breaks. |
| 2026-03-15 | Uniform scale only (Phase 1) | Non-uniform breaks distance property. Defer to later phase. |
| 2026-03-15 | Rigid bodies only (Phase 1-5) | Soft/deformable requires FEM. Geometry is expressible; simulation is not. |
| 2026-03-15 | Analytic gradients deferred to Phase 4 (Session 19) | Phase 1 uses marching cubes (no gradients needed). Dual contouring (Phase 4) needs them. |
| 2026-03-15 | Gyroid/schwarz_p first-class in cf-design | Primary bio-inspired use case. Lattice infill is a core workflow, not an add-on. |
| 2026-03-15 | FlexZone uses planar cuts + beam stiffness analysis | Planar cuts at flex zone plane. Spring-damper stiffness derived from Young's modulus + cross-section geometry. |

---

*This document is the guiding spec for cf-design. All implementation work
should reference it. Update it as design decisions are made.*

*Last updated: 2026-03-15 (stress-tested, v3 — session plan added)*
