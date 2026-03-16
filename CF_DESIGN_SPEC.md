# CF_DESIGN_SPEC — Implicit Surface Design Kernel

> **Status**: Draft — 2026-03-15
> **Crate**: `cf-design` (planned, `crates/cf-design/`)
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

## 10. Phased Roadmap

### Phase 1: Foundation

**Goal**: Expression tree, field evaluation, marching cubes meshing, core primitives.

- [ ] Crate skeleton (`crates/cf-design/`)
- [ ] `FieldNode` expression tree (Section 8.3)
- [ ] Point evaluation: `FieldNode::evaluate(Point3) -> f64`
- [ ] Interval evaluation: `FieldNode::evaluate_interval(Aabb) -> (f64, f64)`
- [ ] Geometric primitives (Section 9.1 — sphere, cuboid, cylinder, capsule, torus, cone, plane)
- [ ] Path-based primitives (Section 9.2 — pipe, pipe_spline)
- [ ] Boolean operations (union, subtract, intersect)
- [ ] Smooth boolean operations (smooth_union, smooth_subtract, smooth_intersect, smooth_union_all)
- [ ] Transform operations (translate, rotate, scale_uniform, mirror)
- [ ] Domain operations (shell, round, offset, elongate)
- [ ] `UserFn` leaf node for custom functions
- [ ] Marching cubes mesher (MC33 or naive + repair) → `IndexedMesh`
- [ ] SDF grid evaluator → `SdfGrid` (via `SdfGrid::from_fn`)
- [ ] `Solid` opaque type with builder API wrapping `FieldNode`
- [ ] `Material` type (density, Young's modulus, color, process)
- [ ] Integration tests: primitive → mesh → valid IndexedMesh (manifold, watertight)

**Exit criteria**: A `Solid` can be built from composed primitives (including
pipe for channels), meshed to `IndexedMesh`, and the mesh passes mesh-repair
validation. Interval evaluation prunes 80%+ of grid cells during meshing.

### Phase 2: Mechanism

**Goal**: Multi-part assemblies with joints and tendons, MJCF generation.

- [ ] `Part` type (named solid + material + optional flex zones)
- [ ] `JointDef` type (revolute, prismatic, ball, free)
- [ ] `TendonDef` type (waypoints + channel_radius + stiffness/damping)
- [ ] `ActuatorDef` type (motor, muscle, general)
- [ ] `PrintProfile` type (clearance, min_wall, min_hole)
- [ ] `Mechanism` type (parts + joints + tendons + actuators + print_profile)
- [ ] Tendon channel subtraction: auto-carve pipe channels from parts along tendon paths
- [ ] `to_mjcf()` — generate valid MJCF XML (bodies, joints, tendons, geoms)
- [ ] `to_shapes()` — generate cf-geometry `Shape` values (SdfGrid or TriangleMesh)
- [ ] `to_stl_kit()` — generate per-part IndexedMesh with clearances applied
- [ ] `validate()` — check min wall, min hole, clearances, feature resolution
- [ ] Volumetric mass property computation from implicit fields
- [ ] Direct Model injection path (SDF geoms, bypassing MJCF)
- [ ] Integration test: Mechanism → MJCF → sim-mjcf parses → sim-core simulates

**Exit criteria**: A multi-part mechanism (e.g., 2-finger gripper with tendon
channels) can be defined in code, simulated in sim-core, exported as printable
STLs with manufacturing clearances, and validated against print constraints.

### Phase 3: Bio-Inspired Library

**Goal**: Rich primitive library for organic shapes, advanced operations.

- [ ] Bio-inspired primitives (Section 9.3 — superellipsoid, log_spiral, gyroid, schwarz_p, helix)
- [ ] Loft primitive (variable cross-section along a path)
- [ ] Advanced domain operations (twist, bend, repeat, repeat_bounded)
- [ ] Lipschitz-aware resolution scaling for twist/bend
- [ ] Lattice infill operations (gyroid/schwarz_p intersected with solid)
- [ ] Variable-radius smooth blending
- [ ] `FlexZone` support (compliant mechanisms — auto-split to sub-bodies)
- [ ] Parameterized part templates (finger, link, bracket, etc.)

**Exit criteria**: A bio-inspired gripper with organic blends, gyroid-infilled
bones, smooth tendon channels, and compliant finger joints can be defined,
simulated, and exported for printing.

### Phase 4: Mesh Quality + Performance

**Goal**: Production-quality meshing for manufacturing, performance at scale.

- [ ] Manifold Dual Contouring mesher (Schaefer/Ju/Warren)
- [ ] SVD-based QEF solver with cell-clamping and centroid bias
- [ ] Analytic gradient computation via reverse-mode AD on expression tree
- [ ] Octree-adaptive meshing with interval arithmetic pruning
- [ ] SIMD batched evaluation (4-8 points per tree walk)
- [ ] Multithreaded grid evaluation
- [ ] Mesh simplification pass (reduce triangle count without losing accuracy)
- [ ] Tolerance-driven meshing (user specifies max deviation, mesher adapts)
- [ ] 3MF export support (via mesh-io — supports multi-material natively)

**Exit criteria**: Meshed parts meet 3D printing requirements (manifold,
watertight, reasonable triangle count) and preserve pin holes, mating surfaces,
and flat faces at specified tolerance. Meshing a 50mm part at 0.1mm resolution
completes in under 5 seconds.

### Phase 5: Differentiable Design (Future)

**Goal**: Gradient-based design optimization through simulation.

- [ ] Parameterized `Solid` (design variables with partial derivatives)
- [ ] Gradient of field with respect to design parameters
- [ ] Integration with sim-core's `mj_derivatives` for end-to-end gradients
- [ ] Design optimization loop: objective → simulate → gradient → update params

**Exit criteria**: A parameterized gripper finger can be optimized to maximize
grasp force by backpropagating through the simulation.

---

## 11. Crate Structure

```
crates/cf-design/
├── Cargo.toml
├── src/
│   ├── lib.rs              Re-exports, crate documentation
│   ├── solid.rs            Opaque Solid type, builder API
│   ├── field/
│   │   ├── mod.rs          FieldNode expression tree, evaluate, evaluate_interval
│   │   ├── primitives.rs   Geometric primitives (sphere, cuboid, cylinder, ...)
│   │   ├── path.rs         Path-based primitives (pipe, pipe_spline, loft)
│   │   ├── bio.rs          Bio-inspired primitives (superellipsoid, gyroid, ...)
│   │   ├── boolean.rs      Union, subtract, intersect (sharp + smooth)
│   │   ├── transform.rs    Translate, rotate, scale, mirror
│   │   ├── domain.rs       Shell, round, offset, elongate, repeat, twist, bend
│   │   ├── interval.rs     Interval arithmetic evaluation for octree pruning
│   │   └── gradient.rs     Analytic gradient computation (Phase 2+)
│   ├── mesh/
│   │   ├── mod.rs          Meshing module documentation
│   │   ├── marching_cubes.rs   Phase 1 mesher (MC33 or naive + repair)
│   │   ├── dual_contouring.rs  Phase 4 mesher (Manifold DC)
│   │   └── grid.rs         Field → SdfGrid evaluation
│   ├── mechanism/
│   │   ├── mod.rs          Mechanism type, builder API
│   │   ├── part.rs         Part (named solid + material + flex zones)
│   │   ├── material.rs     Material, ManufacturingProcess
│   │   ├── joint.rs        JointDef, JointKind
│   │   ├── tendon.rs       TendonDef, TendonWaypoint, channel subtraction
│   │   ├── actuator.rs     ActuatorDef (motor, muscle)
│   │   ├── flex_zone.rs    FlexZone, compliant mechanism auto-splitting
│   │   ├── print.rs        PrintProfile, clearance application
│   │   ├── validate.rs     DesignWarning, manufacturing constraint checks
│   │   ├── mjcf.rs         MJCF XML generation
│   │   └── mass.rs         Volumetric mass property computation
│   └── export/
│       ├── mod.rs          Export module documentation
│       ├── stl.rs          Per-part STL generation
│       └── shapes.rs       cf-geometry Shape generation (SdfGrid + TriangleMesh)
└── tests/
    ├── primitives_tests.rs
    ├── boolean_tests.rs
    ├── interval_tests.rs
    ├── meshing_tests.rs
    ├── mechanism_tests.rs
    ├── tendon_tests.rs
    └── integration_tests.rs
```

### 11.1 Dependencies

```toml
[dependencies]
cf-geometry = { path = "../cf-geometry" }   # IndexedMesh, SdfGrid, Shape, Aabb
nalgebra    = { workspace = true }          # Linear algebra
thiserror   = { workspace = true }          # Error types

[dev-dependencies]
approx      = { workspace = true }          # Float comparison
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

2. **Gradient computation**: Expression tree enables analytic gradients via
   reverse-mode AD. This is preferred over finite differences for dual
   contouring (Phase 2). Should analytic gradient support be part of Phase 1
   (extra work but enables better meshing from the start) or deferred to
   Phase 2?

3. **Lattice structures for infill**: Should gyroid/schwarz_p be first-class
   operations in cf-design, or a separate `cf-lattice` crate? Given that
   lattice infill is a primary bio-inspired use case, first-class seems right.

4. **Assembly constraints**: Beyond joints, should cf-design support
   geometric constraints (coaxial, tangent, coincident) for defining how
   parts relate? Or is this over-engineering for the code-first use case
   where the user specifies positions directly?

5. **Multi-material zones**: Should a single `Part` support multiple material
   regions (via a material field function `material(p) -> MaterialId`), or
   should multi-material parts be expressed as overlapping single-material
   parts? The former is more elegant but requires material-aware meshing and
   3MF export. The latter is simpler but loses single-part semantics.

6. **Compliant mechanism auto-splitting**: The `FlexZone` concept (Section 4.2)
   requires auto-splitting a monolithic part into sub-bodies for MJCF. How
   should the splitting algorithm work? Simple planar cuts? Or does the flex
   zone geometry need to be analyzed to determine the effective hinge axis and
   stiffness?

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

---

*This document is the guiding spec for cf-design. All implementation work
should reference it. Update it as design decisions are made.*

*Last updated: 2026-03-15 (stress-tested, v2)*
