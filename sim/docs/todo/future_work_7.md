# Future Work 7 — Phase 3A: Inertia + Contact Parameter Foundation (Items #23–27)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

All items are prerequisites to #45 (MuJoCo Conformance Test Suite). This file
covers inertia computation and the contact parameter combination layer — the
foundation that all contact constraint assembly builds on.

---

### 23. `<compiler>` `exactmeshinertia` + Full Inertia Tensor Pipeline
**Status:** ✅ Done | **Effort:** M | **Prerequisites:** None

#### Current State

**Parsing gap:** The `MjcfCompiler` struct (`types.rs:274-310`) has 15 fields
but does not include `exactmeshinertia`. The parser (`parser.rs:312-441`)
skips this attribute entirely. The attribute is silently ignored in MJCF input.

**Mesh inertia gap — completely broken:** When inertia is computed from geoms,
mesh-type geoms fall through to hardcoded placeholders:

- `compute_geom_mass()` (`model_builder.rs:4396-4426`): The `match` on
  `geom_type` handles Sphere, Box, Capsule, Cylinder. All other types
  (including Mesh) fall to `_ => 0.001` — a hardcoded tiny volume.
- `compute_geom_inertia()` (`model_builder.rs:4435-4513`): Same pattern.
  Mesh falls to `_ => Vector3::new(0.001, 0.001, 0.001)` — a hardcoded
  tiny diagonal inertia.

This means any body with `<geom type="mesh">` and no explicit `<inertial>`
gets mass ≈ `density * 0.001` and near-zero diagonal inertia, regardless
of the actual mesh geometry.

**Inertia accumulation gap — diagonal only:** `compute_inertia_from_geoms()`
(`model_builder.rs:4348-4393`) has two pre-existing architectural problems:

1. **Diagonal-only accumulation:** It accumulates `Vector3<f64>` (diagonal
   inertia only), discarding off-diagonal terms. This is correct only when
   all geoms are aligned to the body frame principal axes — which is rarely
   true for meshes or rotated primitives.
2. **No geom orientation handling:** The parallel axis theorem application
   (`model_builder.rs:4382-4389`) uses `geom.pos` for the displacement but
   ignores `geom.quat` entirely. A rotated geom's local inertia tensor
   must be rotated (`R * I_local * Rᵀ`) before applying the parallel axis
   theorem. The function always returns `UnitQuaternion::identity()`.

The eigendecomposition pattern for extracting principal axes from a full 3×3
tensor already exists in `extract_inertial_properties()` (`model_builder.rs:
4065-4121`), where it handles `fullinertia` → `symmetric_eigen()` → principal
values + orientation quaternion.

**Mesh data is available:** `ModelBuilder.mesh_data: Vec<Arc<TriangleMeshData>>`
(`model_builder.rs:447`) and `ModelBuilder.mesh_name_to_id: HashMap<String,
usize>` (`model_builder.rs:443`) are populated during asset processing (before
body processing). `TriangleMeshData` (`sim/L0/core/src/mesh.rs:89-101`)
provides `vertices() -> &[Point3<f64>]` and `triangles() -> &[Triangle]` where
`Triangle { v0, v1, v2 }` are vertex indices.

#### MuJoCo Authoritative Semantics

These rules govern all behavior. Every specification decision and acceptance
criterion traces to one or more:

- **S1 — Attribute type:** `exactmeshinertia` is `[bool]`, default `"false"`.
  Part of the `<compiler>` element. When `"true"`, mesh inertia is computed
  using exact integration over the triangle mesh surface. When `"false"`,
  MuJoCo uses a coarser algorithm (union-of-pyramids from centroid, which is
  exact for convex meshes but overcounts for non-convex ones).

- **S2 — Scope:** Only affects `<geom type="mesh">` elements where body
  inertia is auto-computed from geoms (no explicit `<inertial>`, and
  `inertiafromgeom` is `"true"` or `"auto"` with no `<inertial>`). Bodies
  with explicit `<inertial>` are completely unaffected.

- **S3 — Algorithm (exact mode, `true`):** Signed tetrahedron decomposition
  (Mirtich 1996 / Eberly). For each triangle in the mesh, form a tetrahedron
  with the origin and compute its signed volume contribution to mass, center
  of mass, and all 6 independent elements of the inertia tensor. Produces
  exact results for any closed triangle mesh (convex or non-convex).

- **S4 — Algorithm (approximate mode, `false`):** Union-of-pyramids from
  centroid. Exact for convex meshes, overcounts volume for non-convex.

- **S5 — Deprecation:** MuJoCo 3.2.5+ deprecated `exactmeshinertia` in favor
  of per-mesh `<mesh inertia="..."/>` attribute. CortenForge implements the
  pre-3.2.5 compiler flag. `shellinertia` (per-geom, item #43) is out of
  scope.

- **S6 — Non-mesh geoms:** Primitive geoms (sphere, box, cylinder, capsule,
  ellipsoid) are completely unaffected by this flag. Their inertia is always
  computed via exact analytical formulas.

#### CortenForge Design Decision: Single Exact Algorithm

MuJoCo distinguishes between `true` (exact) and `false` (approximate) modes.
CortenForge uses the **exact signed-volume algorithm for both modes**:

1. The current mesh inertia is a `0.001` placeholder — there is no "current
   approximate behavior" to preserve.
2. The exact algorithm (Mirtich 1996) is simple, numerically stable, and
   costs O(n) per mesh (one pass over triangles). There is no performance
   reason to maintain two algorithms.
3. The flag is parsed and stored for MJCF round-trip fidelity, but both
   values produce identical results.

This is strictly correct: the exact algorithm IS exact for convex meshes
(matching MuJoCo's `false` mode for convex meshes) and also correct for
non-convex meshes (matching MuJoCo's `true` mode).

#### Objective

1. Parse `exactmeshinertia` from `<compiler>` and store it in `MjcfCompiler`.
2. Implement exact mesh inertia computation using signed tetrahedron
   decomposition.
3. Refactor `compute_inertia_from_geoms()` from diagonal-only to full 3×3
   tensor accumulation with geom orientation handling and eigendecomposition.
4. Replace the `0.001` placeholders in `compute_geom_mass()` and
   `compute_geom_inertia()` with actual mesh inertia computation.

#### What This Item Does NOT Cover

- **Per-mesh `<mesh inertia="...">`**: MuJoCo 3.2.5+ attribute. Separate
  future work item.
- **`shellinertia`**: Per-geom attribute (item #43). Shell (surface-only)
  inertia vs solid inertia. Out of scope.
- **Non-mesh geom inertia improvements**: The existing analytical formulas
  for sphere, box, cylinder, capsule, ellipsoid are correct and unchanged.
- **`inertiafromgeom` semantics**: The existing `True`/`Auto`/`False` dispatch
  (`model_builder.rs:1045-1066`) is correct and unchanged. This item only
  changes what happens inside the `compute_inertia_from_geoms()` path.

#### Specification

##### B1. Parsing — `MjcfCompiler` Field

Add `exactmeshinertia` to the `MjcfCompiler` struct (`types.rs:274-310`):

```rust
// After the existing `alignfree` field (line 309):
/// Use exact mesh inertia computation (signed tetrahedron decomposition).
/// When false, same algorithm is used (CortenForge always uses exact).
/// Parsed for MJCF round-trip fidelity.
pub exactmeshinertia: bool,
```

Add to `Default` impl (`types.rs:312-334`):

```rust
// After `alignfree: false` (line 331):
exactmeshinertia: false,
```

##### B2. Parsing — `parse_compiler_attrs()`

Add parsing in `parse_compiler_attrs()` (`parser.rs:312-441`), in the
"Deferred attributes" section after `alignfree` (line 438):

```rust
// A13. exactmeshinertia
if let Some(emi) = get_attribute_opt(e, "exactmeshinertia") {
    compiler.exactmeshinertia = emi == "true";
}
```

This follows the identical pattern as `fitaabb` (line 431), `usethread`
(line 434), and `alignfree` (line 437) — boolean attributes parsed via
string comparison.

##### C1. Algorithm — `compute_mesh_inertia()`

New function in `model_builder.rs`. Computes exact mass properties of a
triangle mesh using signed tetrahedron decomposition (Mirtich 1996).

**Signature:**

```rust
/// Compute exact mass properties of a triangle mesh using signed
/// tetrahedron decomposition (Mirtich 1996).
///
/// Returns `(volume, com, inertia_at_com)` where:
/// - `volume` is the signed volume (positive for outward-facing normals)
/// - `com` is the center of mass (assuming uniform density)
/// - `inertia_at_com` is the full 3×3 inertia tensor about the COM
///   (assuming unit density; multiply by actual density for physical values)
fn compute_mesh_inertia(
    mesh: &TriangleMeshData,
) -> (f64, Vector3<f64>, Matrix3<f64>)
```

**Algorithm pseudocode:**

```
total_volume = 0
com_accum = [0, 0, 0]

// Inertia integrals (products of vertex coordinates)
// We need: ∫x², ∫y², ∫z², ∫xy, ∫xz, ∫yz over the solid volume
xx = yy = zz = xy = xz = yz = 0

for each triangle (a, b, c) in mesh:
    // Signed volume of tetrahedron formed with origin
    // V_tet = (a × b) · c / 6
    cross = a × b
    det = cross · c
    vol = det / 6.0

    total_volume += vol

    // COM contribution: centroid of tet = (a + b + c) / 4, weighted by vol
    com_accum += vol * (a + b + c) / 4.0

    // Second-moment integrals over tetrahedron (origin, a, b, c):
    // For a tet with one vertex at origin, the integrals are:
    //   ∫x² dV = det/60 * (a.x² + b.x² + c.x² + a.x*b.x + a.x*c.x + b.x*c.x)
    //   ∫xy dV = det/120 * (2*a.x*a.y + 2*b.x*b.y + 2*c.x*c.y
    //            + a.x*b.y + a.y*b.x + a.x*c.y + a.y*c.x + b.x*c.y + b.y*c.x)
    // (and similarly for yy, zz, xz, yz)

    // Accumulate all 6 independent integrals using the formulas above.

com = com_accum / total_volume

// Shift second moments to COM using parallel axis theorem:
//   I_com = I_origin - total_volume * (d·d * I₃ - d ⊗ d)
// where d = com, and I_origin is built from the accumulated integrals:
//   I_origin[0,0] = yy + zz    (Ixx = ∫(y²+z²) dV)
//   I_origin[1,1] = xx + zz    (Iyy = ∫(x²+z²) dV)
//   I_origin[2,2] = xx + yy    (Izz = ∫(x²+y²) dV)
//   I_origin[0,1] = I_origin[1,0] = -xy  (Ixy = -∫xy dV)
//   I_origin[0,2] = I_origin[2,0] = -xz
//   I_origin[1,2] = I_origin[2,1] = -yz

return (total_volume, com, I_com)
```

**Important notes:**

- The integrals assume unit density. Actual mass = `density * volume`,
  actual inertia = `density * I_com`.
- Signed volumes handle non-convex meshes correctly: interior regions
  contribute positive volume, exterior overcounts cancel.
- If `|total_volume| < 1e-10`, the mesh is degenerate (e.g., all triangles
  coplanar). Fall back to AABB-based approximation: treat the mesh's
  bounding box as a solid box and use the box inertia formula.

**Zero-volume fallback:**

```rust
if total_volume.abs() < 1e-10 {
    // Degenerate mesh: use AABB as approximation
    let (aabb_min, aabb_max) = mesh.aabb();
    let extents = aabb_max - aabb_min;
    let volume = extents.x * extents.y * extents.z;
    let com = nalgebra::center(&aabb_min, &aabb_max).coords;
    // Box inertia: I_ii = m/12 * (a² + b²) for the other two dimensions
    let c = volume / 12.0; // unit density
    let inertia = Matrix3::from_diagonal(&Vector3::new(
        c * (extents.y.powi(2) + extents.z.powi(2)),
        c * (extents.x.powi(2) + extents.z.powi(2)),
        c * (extents.x.powi(2) + extents.y.powi(2)),
    ));
    return (volume, com, inertia);
}
```

##### D1. Pipeline Refactor — `compute_geom_mass()` Mesh Support

Replace the `_ => 0.001` fallback in `compute_geom_mass()` (`model_builder.rs:
4396-4426`) with actual mesh volume computation.

**BEFORE** (`model_builder.rs:4401-4423`):

```rust
let volume = match geom.geom_type {
    MjcfGeomType::Sphere => { ... },
    MjcfGeomType::Box => { ... },
    MjcfGeomType::Capsule => { ... },
    MjcfGeomType::Cylinder => { ... },
    _ => 0.001,
};
```

**AFTER:** Add a `mesh_data: Option<&TriangleMeshData>` parameter. The
function signature becomes:

```rust
fn compute_geom_mass(
    geom: &MjcfGeom,
    mesh_data: Option<&TriangleMeshData>,
) -> f64
```

Add a `Mesh` arm before the catch-all:

```rust
MjcfGeomType::Mesh => {
    if let Some(mesh) = mesh_data {
        let (volume, _, _) = compute_mesh_inertia(mesh);
        volume.abs() // Use absolute value for mass
    } else {
        0.001 // No mesh data available (shouldn't happen)
    }
}
```

Keep `_ => 0.001` for remaining types (Plane, Hfield) which have no
meaningful volume.

##### D2. Pipeline Refactor — `compute_geom_inertia()` Full Tensor Return

Change `compute_geom_inertia()` (`model_builder.rs:4435-4513`) to return
`Matrix3<f64>` instead of `Vector3<f64>`. This is required because mesh
inertia produces off-diagonal terms, and even rotated primitive geoms
contribute off-diagonal terms after rotation.

**BEFORE signature:**

```rust
fn compute_geom_inertia(geom: &MjcfGeom) -> Vector3<f64>
```

**AFTER signature:**

```rust
fn compute_geom_inertia(
    geom: &MjcfGeom,
    mesh_data: Option<&TriangleMeshData>,
) -> Matrix3<f64>
```

**Changes:**

- All primitive arms (Sphere, Box, Cylinder, Capsule, Ellipsoid) return
  `Matrix3::from_diagonal(&Vector3::new(...))` instead of `Vector3::new(...)`.
  (Primitives aligned to geom-local axes have diagonal-only inertia.)
- Add `Mesh` arm:

```rust
MjcfGeomType::Mesh => {
    if let Some(mesh) = mesh_data {
        let (volume, _, inertia_unit) = compute_mesh_inertia(mesh);
        let density = geom.density;
        let mass_actual = geom.mass.unwrap_or(density * volume.abs());
        let scale = if volume.abs() > 1e-10 {
            mass_actual / volume.abs()
        } else {
            density
        };
        inertia_unit * scale
    } else {
        Matrix3::from_diagonal(&Vector3::new(0.001, 0.001, 0.001))
    }
}
```

- Catch-all `_ =>` returns `Matrix3::from_diagonal(&Vector3::new(0.001, ...))`.

##### D3. Pipeline Refactor — `compute_inertia_from_geoms()` Full Tensor Accumulation

Refactor `compute_inertia_from_geoms()` (`model_builder.rs:4348-4393`) from
diagonal-only accumulation to full 3×3 tensor accumulation with geom
orientation and eigendecomposition.

**BEFORE signature:**

```rust
fn compute_inertia_from_geoms(
    geoms: &[MjcfGeom],
) -> (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>)
```

**AFTER signature:**

```rust
fn compute_inertia_from_geoms(
    geoms: &[MjcfGeom],
    mesh_lookup: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
) -> (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>)
```

**Algorithm (second pass — inertia accumulation):**

Replace the diagonal-only accumulation (`model_builder.rs:4377-4389`) with:

```rust
// Second pass: accumulate full 3×3 inertia tensor about COM
let mut inertia_tensor = Matrix3::zeros();
for geom in geoms {
    let mesh = resolve_mesh(geom, mesh_lookup, mesh_data);
    let geom_mass = compute_geom_mass(geom, mesh.as_deref());
    let geom_inertia = compute_geom_inertia(geom, mesh.as_deref());

    // 1. Rotate local inertia to body frame: I_rot = R * I_local * Rᵀ
    let r = UnitQuaternion::from_quaternion(Quaternion::new(
        geom.quat[0], geom.quat[1], geom.quat[2], geom.quat[3],
    ));
    let rot = r.to_rotation_matrix();
    let i_rotated = rot * geom_inertia * rot.transpose();

    // 2. Parallel axis theorem (full tensor):
    //    I_shifted = I_rotated + m * (d·d * I₃ - d ⊗ d)
    let d = geom.pos - com;
    let d_sq = d.dot(&d);
    let parallel_axis = geom_mass
        * (Matrix3::identity() * d_sq - d * d.transpose());
    inertia_tensor += i_rotated + parallel_axis;
}
```

**Eigendecomposition (new — replaces `UnitQuaternion::identity()` return):**

After accumulation, eigendecompose the full tensor to extract principal
inertia and orientation. This reuses the exact pattern from
`extract_inertial_properties()` (`model_builder.rs:4087-4108`):

```rust
// Eigendecompose to get principal axes
let eigen = inertia_tensor.symmetric_eigen();
let principal_inertia = Vector3::new(
    eigen.eigenvalues[0].abs(),
    eigen.eigenvalues[1].abs(),
    eigen.eigenvalues[2].abs(),
);

// Eigenvectors form rotation to principal axes
let mut rot = eigen.eigenvectors;
if rot.determinant() < 0.0 {
    rot.set_column(2, &(-rot.column(2)));
}
let iquat = UnitQuaternion::from_rotation_matrix(
    &nalgebra::Rotation3::from_matrix(&rot),
);

(total_mass, principal_inertia, com, iquat)
```

**Helper function for mesh lookup:**

```rust
/// Resolve mesh data for a geom, if it is a mesh-type geom.
fn resolve_mesh(
    geom: &MjcfGeom,
    mesh_lookup: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
) -> Option<Arc<TriangleMeshData>> {
    if geom.geom_type == MjcfGeomType::Mesh {
        geom.mesh.as_ref()
            .and_then(|name| mesh_lookup.get(name))
            .and_then(|&id| mesh_data.get(id))
            .cloned()
    } else {
        None
    }
}
```

##### D4. Pipeline Wiring — Call Site Update

Update the two call sites of `compute_inertia_from_geoms()`:

1. **`InertiaFromGeom::True` branch** (`model_builder.rs:1046`):

```rust
// BEFORE:
compute_inertia_from_geoms(&resolved_geoms)
// AFTER:
compute_inertia_from_geoms(
    &resolved_geoms,
    &self.mesh_name_to_id,
    &self.mesh_data,
)
```

2. **`InertiaFromGeom::Auto` branch** (`model_builder.rs:1051`):

```rust
// Same change — pass mesh lookup tables
compute_inertia_from_geoms(
    &resolved_geoms,
    &self.mesh_name_to_id,
    &self.mesh_data,
)
```

The `InertiaFromGeom::False` branch (`model_builder.rs:1054-1064`) is
unchanged — it never calls `compute_inertia_from_geoms()`.

#### Acceptance Criteria

##### AC1 — Parser: Default Value

```xml
<mujoco>
  <worldbody><body><geom type="sphere" size="0.1"/></body></worldbody>
</mujoco>
```
Parse succeeds. `compiler.exactmeshinertia == false` (MuJoCo default).

##### AC2 — Parser: Explicit True/False

```xml
<mujoco>
  <compiler exactmeshinertia="true"/>
  <worldbody><body><geom type="sphere" size="0.1"/></body></worldbody>
</mujoco>
```
`compiler.exactmeshinertia == true`.

```xml
<mujoco>
  <compiler exactmeshinertia="false"/>
  <worldbody><body><geom type="sphere" size="0.1"/></body></worldbody>
</mujoco>
```
`compiler.exactmeshinertia == false`.

##### AC3 — Unit Cube Mesh: Analytical Match

Create a unit cube mesh (8 vertices, 12 triangles, edge length 1.0,
centered at origin). With `density=1000.0`:

- Expected mass: `1000.0 * 1.0 = 1000.0` kg
- Expected diagonal inertia: `I_xx = I_yy = I_zz = (1000/12) * (1² + 1²) = 166.667` kg·m²
- Expected COM: `(0, 0, 0)`
- Expected off-diagonal: `0` (symmetric cube → diagonal tensor)

Assert `mass ≈ 1000.0` (±0.01), `inertia ≈ [166.667, 166.667, 166.667]` (±0.1).

```xml
<mujoco>
  <compiler exactmeshinertia="true"/>
  <asset>
    <mesh name="cube" vertex="
      -0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5
      -0.5 -0.5  0.5  0.5 -0.5  0.5  0.5 0.5  0.5  -0.5 0.5  0.5"
      face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/>
  </asset>
  <worldbody>
    <body>
      <geom type="mesh" mesh="cube" density="1000"/>
    </body>
  </worldbody>
</mujoco>
```

##### AC4 — Asymmetric Mesh: Off-Diagonal Terms

Create an L-shaped mesh (two joined cubes, asymmetric about all axes).
The inertia tensor should have off-diagonal terms and the resulting
`iquat` should NOT be identity.

Assert:
- `iquat != UnitQuaternion::identity()` (non-trivial principal axes)
- Principal inertia values are all positive
- `I_xx != I_yy` or `I_yy != I_zz` (asymmetric)

##### AC5 — Explicit Inertial Overrides Mesh

```xml
<mujoco>
  <compiler exactmeshinertia="true"/>
  <asset>
    <mesh name="cube" vertex="..." face="..."/>
  </asset>
  <worldbody>
    <body>
      <inertial mass="42.0" pos="0 0 0" diaginertia="1 2 3"/>
      <geom type="mesh" mesh="cube"/>
    </body>
  </worldbody>
</mujoco>
```

Body mass is `42.0`, not the mesh-computed mass. Diagonal inertia is
`[1, 2, 3]`, not mesh-computed. The `exactmeshinertia` flag is irrelevant
when explicit `<inertial>` is present. (Gated by `inertiafromgeom`
semantics, not by this item — we just verify the existing gate works.)

##### AC6 — COM Offset (Parallel Axis Theorem)

Place a mesh geom at `pos="1 0 0"` in a body with no other geoms.
The body COM should be at `(1, 0, 0)` plus the mesh's local COM.
The inertia should reflect the parallel axis theorem shift.

Verify by comparing with a mesh geom at `pos="0 0 0"` — the inertia
should differ by `m * d²` on the appropriate axes.

##### AC7 — Zero-Volume Degenerate Mesh Fallback

Create a degenerate mesh (all triangles coplanar, zero enclosed volume):

```xml
<mujoco>
  <asset>
    <mesh name="flat" vertex="0 0 0  1 0 0  1 1 0  0 1 0"
          face="0 1 2  0 2 3"/>
  </asset>
  <worldbody>
    <body>
      <geom type="mesh" mesh="flat" density="1000"/>
    </body>
  </worldbody>
</mujoco>
```

The mesh has zero volume (all vertices in z=0 plane). The fallback
should use the AABB dimensions (1×1×0) for mass/inertia approximation.
Mass should be 0 (zero-volume AABB) or a small positive value. The
simulation should not panic or produce NaN/Inf.

##### AC8 — `exactmeshinertia=true` == `exactmeshinertia=false`

```xml
<!-- Model A -->
<mujoco>
  <compiler exactmeshinertia="true"/>
  <asset><mesh name="cube" vertex="..." face="..."/></asset>
  <worldbody><body><geom type="mesh" mesh="cube"/></body></worldbody>
</mujoco>

<!-- Model B -->
<mujoco>
  <compiler exactmeshinertia="false"/>
  <asset><mesh name="cube" vertex="..." face="..."/></asset>
  <worldbody><body><geom type="mesh" mesh="cube"/></body></worldbody>
</mujoco>
```

Both models produce identical mass, COM, inertia, and iquat (because
CortenForge uses the same exact algorithm for both modes).

##### AC9 — Multi-Geom Body: Mixed Mesh + Primitive

```xml
<mujoco>
  <compiler exactmeshinertia="true"/>
  <asset><mesh name="cube" vertex="..." face="..."/></asset>
  <worldbody>
    <body>
      <geom type="sphere" size="0.1" pos="0 0 0" density="1000"/>
      <geom type="mesh" mesh="cube" pos="1 0 0" density="1000"/>
    </body>
  </worldbody>
</mujoco>
```

The body inertia should be the correct aggregation of:
1. Sphere inertia (analytical, at pos 0,0,0)
2. Cube mesh inertia (exact, at pos 1,0,0)
3. Full parallel axis theorem with cross terms

Total mass = sphere mass + cube mass.
COM = weighted average of sphere center and cube center.
The off-axis placement means `iquat` should be non-identity.

#### Implementation Order

Each step must compile and pass existing tests before proceeding.

1. **B1: `types.rs`** — Add `exactmeshinertia: bool` field to `MjcfCompiler`
   struct and `Default` impl.

2. **B2: `parser.rs`** — Add parsing in `parse_compiler_attrs()`. Run
   `cargo test -p sim-mjcf` to verify no regressions.

3. **C1: `model_builder.rs`** — Add `compute_mesh_inertia()` function.
   No callers yet — just ensure it compiles.

4. **D1: `model_builder.rs`** — Refactor `compute_geom_mass()`: add
   `mesh_data` parameter, add Mesh arm. Update call sites to pass `None`
   temporarily. Run `cargo test -p sim-mjcf` — all existing tests pass
   (no mesh geoms in existing tests use auto-inertia).

5. **D2: `model_builder.rs`** — Refactor `compute_geom_inertia()`: change
   return type to `Matrix3<f64>`, add `mesh_data` parameter, add Mesh arm.
   Update `compute_inertia_from_geoms()` to handle `Matrix3` return
   (temporarily extract diagonal for backwards compat).

6. **D3: `model_builder.rs`** — Full tensor refactor of
   `compute_inertia_from_geoms()`: add parameters, full 3×3 accumulation
   with geom orientation, eigendecomposition. Add `resolve_mesh()` helper.

7. **D4: `model_builder.rs`** — Update call sites at lines 1046 and 1051
   to pass `&self.mesh_name_to_id` and `&self.mesh_data`.

8. **Tests** — Write all 9 acceptance tests in
   `sim/L0/tests/integration/exactmeshinertia.rs`. Register module in
   `sim/L0/tests/integration/mod.rs`.

#### Risk Mitigations

1. **Eigendecomposition edge cases:** Degenerate meshes (zero volume,
   coplanar triangles) → AABB fallback with warning. Volume threshold
   `1e-10` prevents division by zero.

2. **Numerical precision of signed volumes:** For near-degenerate meshes,
   signed volumes can nearly cancel. The `1e-10` threshold catches this.
   For well-formed meshes, the algorithm is numerically stable (each
   tetrahedron contributes independently).

3. **Existing test breakage:** The current mesh inertia is `0.001` —
   any tests that depend on this specific value will change. Audit
   existing tests before D1 to identify affected assertions.

4. **Geom orientation rotation:** The rotation `R * I * Rᵀ` must use
   the correct rotation direction. `UnitQuaternion::to_rotation_matrix()`
   returns the matrix that rotates from local to world frame, which is
   what we need.

5. **Mesh winding order:** The signed volume formula assumes consistent
   outward-facing normals. Meshes with inconsistent winding will produce
   incorrect (possibly negative) volumes. The `volume.abs()` in mass
   computation handles the sign, but inverted winding produces incorrect
   COM and inertia. This matches MuJoCo's behavior — it also requires
   consistent winding.

#### Files

- `sim/L0/mjcf/src/types.rs` — `MjcfCompiler` field + Default
- `sim/L0/mjcf/src/parser.rs` — parsing in `parse_compiler_attrs()`
- `sim/L0/mjcf/src/model_builder.rs` — `compute_mesh_inertia()`,
  `compute_geom_mass()`, `compute_geom_inertia()`,
  `compute_inertia_from_geoms()`, `resolve_mesh()`, call site updates
- `sim/L0/tests/integration/exactmeshinertia.rs` — 9 acceptance tests
- `sim/L0/tests/integration/mod.rs` — register module

---

## Batch 1 Interaction Model — Contact Parameter Combination (#24–#27)

Items #24–#27 all converge in a single MuJoCo function: `mj_contactParam()` in
`engine_collision_driver.c`. Understanding this function as a whole is essential
before specifying any individual item. This section documents the complete MuJoCo
reference semantics, the shared control flow, the data flow through our codebase,
and the shared test fixtures.

### MuJoCo Reference: `mj_contactParam()` — Complete Semantics

MuJoCo's `mj_contactParam()` computes contact parameters (condim, gap, solref,
solimp, friction) for a collision pair. It is called once per contact pair in
`mj_collideGeoms()`, after narrow-phase collision detection, before contact
finalization.

**Inputs:** Two entities (geom or flex), identified by indices. Each provides:
priority, condim, gap, solmix, solref[2], solimp[5], friction[3].

**Complete control flow** (from MuJoCo source):

```
┌─────────────────────────────────────────────────────────────┐
│  1. LOAD parameters from geom1/flex1 and geom2/flex2        │
│     priority, condim, gap, solmix, solref, solimp, friction │
├─────────────────────────────────────────────────────────────┤
│  2. GAP: always additive                                    │
│     gap = gap1 + gap2                                  #27  │
├─────────────────────────────────────────────────────────────┤
│  3. PRIORITY CHECK                                     #25  │
│     if priority1 > priority2:                               │
│       condim, solref, solimp, friction ← geom1 (verbatim)  │
│       GOTO 6 (unpack friction)                              │
│     if priority1 < priority2:                               │
│       condim, solref, solimp, friction ← geom2 (verbatim)  │
│       GOTO 6 (unpack friction)                              │
│     // else: equal priority → fall through to combination   │
├─────────────────────────────────────────────────────────────┤
│  4. EQUAL PRIORITY — combine parameters                     │
│     4a. condim = max(condim1, condim2)                      │
│     4b. solmix weight:                                 #26  │
│         if both >= mjMINVAL: mix = s1/(s1+s2)              │
│         if both < mjMINVAL:  mix = 0.5                     │
│         if only s1 < mjMINVAL: mix = 0.0                   │
│         if only s2 < mjMINVAL: mix = 1.0                   │
│     4c. solref combination:                            #26  │
│         if solref1[0] > 0 AND solref2[0] > 0:             │
│           solref[i] = mix*solref1[i] + (1-mix)*solref2[i]  │
│         else (direct reference):                            │
│           solref[i] = min(solref1[i], solref2[i])           │
│     4d. solimp = mix*solimp1 + (1-mix)*solimp2        #26  │
│     4e. friction[i] = max(friction1[i], friction2[i]) #24  │
├─────────────────────────────────────────────────────────────┤
│  5. UNPACK 3→5 friction                                     │
│     mu[0]=fri[0], mu[1]=fri[0],                             │
│     mu[2]=fri[1],                                           │
│     mu[3]=fri[2], mu[4]=fri[2]                              │
├─────────────────────────────────────────────────────────────┤
│  6. SET on contact:                                         │
│     includemargin = margin - gap                       #27  │
│     exclude = (dist >= includemargin)                  #27  │
└─────────────────────────────────────────────────────────────┘
```

**Key constants:**
- `mjMINVAL ≈ 1e-15` — threshold for treating solmix as zero
- `mjNREF = 2` — solref array length
- `mjNIMP = 5` — solimp array length

### MuJoCo Reference: Margin/Gap Pipeline (#27)

Margin and gap flow through the entire collision pipeline, not just `mj_contactParam()`:

```
┌──────────────────────────────────────────────────────────────────┐
│  BROADPHASE: AABB expansion                                      │
│  makeAAMM() expands bounding box by geom_margin[i] per geom.    │
│  This ensures margin-separated geom pairs enter narrow-phase.    │
├──────────────────────────────────────────────────────────────────┤
│  MIDPHASE: Bounding sphere filter                                │
│  mj_filterSphere(): dist > rbound1 + rbound2 + margin → skip   │
│  margin = geom_margin[g1] + geom_margin[g2]  (or pair.margin)   │
├──────────────────────────────────────────────────────────────────┤
│  NARROW-PHASE: Collision functions receive margin parameter      │
│  collisionFunc(m, d, con, g1, g2, margin)                        │
│  Functions return contacts when dist < margin (surfaces may be   │
│  separated). con->dist is signed distance (negative = overlap).  │
├──────────────────────────────────────────────────────────────────┤
│  CONTACT CREATION: mj_setContact()                               │
│  con->includemargin = margin - gap                               │
│  con->exclude = (con->dist >= includemargin)                     │
│  Excluded contacts exist in the array but generate no constraint │
│  rows — they are "gap-buffered" contacts.                        │
├──────────────────────────────────────────────────────────────────┤
│  CONSTRAINT ASSEMBLY: mj_instantiateContact()                    │
│  efc_pos[row] = con->dist                                        │
│  efc_margin[row] = con->includemargin                            │
│  getimpedance(): x = (pos - margin) / solimp[2]                 │
│  aref = -b*vel - k*imp*(pos - margin)                            │
│  Margin shifts the constraint equilibrium: zero force at         │
│  dist = includemargin, force onset for dist < includemargin.     │
└──────────────────────────────────────────────────────────────────┘
```

### Our Codebase: Current State

**Contact parameter combination** lives in `mujoco_pipeline.rs`:

| Function | Lines | What It Does | Status |
|----------|-------|-------------|--------|
| `make_contact_from_geoms()` | 6054–6089 | Rigid-rigid contact creation | Friction uses geometric mean (BUG) |
| `make_contact_flex_rigid()` | 5776–5828 | Flex-rigid contact creation | Friction uses element-wise max (correct) |
| `combine_solver_params()` | 2091–2110 | Combine solref/solimp | Uses min/max (no solmix weighting) |
| `apply_pair_overrides()` | 6021–6043 | Override from `<contact><pair>` | Skips margin/gap |
| `collide_geoms()` dispatcher | ~5833 | Routes to type-specific collider | No margin parameter |
| Broadphase (automatic) | 5500–5535 | SAP + affinity filter | No margin in AABB expansion |
| Broadphase (explicit pairs) | 5537–5573 | Distance cull + collision | Uses `pair.margin` in distance cull |
| `assemble_contact_system()` | ~15410 | Contact → constraint rows | `margin` hardcoded to `0.0` |
| `compute_aref()` | 14911–14913 | Reference acceleration | Signature correct, receives `0.0` |
| `compute_impedance()` | 14685–14744 | Position-dependent impedance | Receives `violation` (no margin offset) |

**Model fields:**

| Field | Exists? | Parsed from `<geom>`? | Used at Runtime? |
|-------|---------|----------------------|-----------------|
| `geom_friction: Vec<Vector3<f64>>` | ✅ | ✅ | ✅ (geometric mean — BUG) |
| `geom_solref: Vec<[f64; 2]>` | ✅ | ✅ | ✅ (element-wise min) |
| `geom_solimp: Vec<[f64; 5]>` | ✅ | ✅ | ✅ (element-wise max) |
| `geom_condim: Vec<i32>` | ✅ | ✅ | ✅ (max) |
| `geom_margin: Vec<f64>` | ✅ | ❌ (zeroed) | ❌ (ignored) |
| `geom_gap: Vec<f64>` | ✅ | ❌ (zeroed) | ❌ (ignored) |
| `geom_priority: Vec<i32>` | ❌ | — | — |
| `geom_solmix: Vec<f64>` | ❌ | — | — |

**MJCF parsing** (`parser.rs` / `model_builder.rs` / `types.rs`):

| Attribute | In `MjcfGeom` struct? | Parsed? | In defaults cascade? |
|-----------|----------------------|---------|---------------------|
| `friction` | ✅ `Vector3<f64>` | ✅ | ✅ |
| `solref` | ✅ `Option<[f64; 2]>` | ✅ | ❌ |
| `solimp` | ✅ `Option<[f64; 5]>` | ✅ | ❌ |
| `condim` | ✅ `i32` | ✅ | ❌ (only contype/conaffinity cascade) |
| `margin` | ❌ | ❌ | ❌ |
| `gap` | ❌ | ❌ | ❌ |
| `priority` | ❌ | ❌ | ❌ |
| `solmix` | ❌ | ❌ | ❌ |

**Contact struct** (`mujoco_pipeline.rs:1862–1896`):

| Field | Type | Status |
|-------|------|--------|
| `includemargin` | `bool` | Always `false`. Needs to become `f64` (#27). |
| `mu` | `[f64; 5]` | Populated from combination rule. |
| `solref` | `[f64; 2]` | Populated from `combine_solver_params()`. |
| `solimp` | `[f64; 5]` | Populated from `combine_solver_params()`. |
| `depth` | `f64` | Penetration depth (positive = overlap). |

### Unified Refactoring Target: `contact_param()`

All four items converge on creating a **single** `contact_param()` function that
mirrors MuJoCo's `mj_contactParam()`. Rather than patching four pieces into
three existing functions, we extract the parameter combination into one function:

```rust
/// Contact parameter combination — MuJoCo `mj_contactParam()` equivalent.
///
/// Computes combined contact parameters from two geoms.
/// Handles priority (#25), solmix (#26), friction max (#24), and gap (#27).
///
/// Returns: (condim, gap, solref, solimp, friction[5])
fn contact_param(
    model: &Model,
    geom1: usize,
    geom2: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    // 1. Load parameters
    let priority1 = model.geom_priority[geom1];
    let priority2 = model.geom_priority[geom2];
    let gap = model.geom_gap[geom1] + model.geom_gap[geom2];

    // 2. Priority check
    if priority1 != priority2 {
        let winner = if priority1 > priority2 { geom1 } else { geom2 };
        let fri = model.geom_friction[winner];
        return (
            model.geom_condim[winner],
            gap,
            model.geom_solref[winner],
            model.geom_solimp[winner],
            [fri.x, fri.x, fri.y, fri.z, fri.z],  // 3→5 unpack
        );
    }

    // 3. Equal priority — combine
    let condim = model.geom_condim[geom1].max(model.geom_condim[geom2]);

    // 3a. Solmix weight
    let s1 = model.geom_solmix[geom1];
    let s2 = model.geom_solmix[geom2];
    let mix = solmix_weight(s1, s2);

    // 3b. Solref combination
    let solref1 = model.geom_solref[geom1];
    let solref2 = model.geom_solref[geom2];
    let solref = if solref1[0] > 0.0 && solref2[0] > 0.0 {
        // Standard reference: weighted average
        [mix * solref1[0] + (1.0 - mix) * solref2[0],
         mix * solref1[1] + (1.0 - mix) * solref2[1]]
    } else {
        // Direct reference: element-wise min
        [solref1[0].min(solref2[0]), solref1[1].min(solref2[1])]
    };

    // 3c. Solimp: weighted average
    let solimp1 = model.geom_solimp[geom1];
    let solimp2 = model.geom_solimp[geom2];
    let solimp = std::array::from_fn(|i| mix * solimp1[i] + (1.0 - mix) * solimp2[i]);

    // 3d. Friction: element-wise max
    let f1 = model.geom_friction[geom1];
    let f2 = model.geom_friction[geom2];
    let fri = [f1.x.max(f2.x), f1.x.max(f2.x),   // sliding1, sliding2
               f1.y.max(f2.y),                      // torsional
               f1.z.max(f2.z), f1.z.max(f2.z)];    // rolling1, rolling2

    (condim, gap, solref, solimp, fri)
}

/// Compute solmix weight, matching MuJoCo's edge-case handling.
/// Returns weight for geom1 (geom2 weight = 1 - mix).
fn solmix_weight(s1: f64, s2: f64) -> f64 {
    const MJ_MINVAL: f64 = 1e-15;
    if s1 >= MJ_MINVAL && s2 >= MJ_MINVAL {
        s1 / (s1 + s2)
    } else if s1 < MJ_MINVAL && s2 < MJ_MINVAL {
        0.5
    } else if s1 < MJ_MINVAL {
        0.0  // geom2 dominates
    } else {
        1.0  // geom1 dominates
    }
}
```

This function replaces:
- The friction combination in `make_contact_from_geoms()` (currently geometric mean)
- The `combine_solver_params()` function (currently min/max, no solmix)
- Adds priority gating (currently absent)
- Adds gap combination (currently absent)

**`make_contact_from_geoms()` becomes a thin wrapper:**

```rust
fn make_contact_from_geoms(
    model: &Model,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
    geom1: usize,
    geom2: usize,
    margin: f64,  // #27: effective margin for this pair
) -> Contact {
    let (condim, gap, solref, solimp, mu) = contact_param(model, geom1, geom2);
    let includemargin = margin - gap;

    Contact::with_condim_full(
        pos, normal, depth, geom1, geom2, mu, condim, solref, solimp, includemargin,
    )
}
```

**`make_contact_flex_rigid()` gets an equivalent `contact_param_flex_rigid()`:**

MuJoCo's `mj_contactParam()` is entity-agnostic: it uses `(f1 < 0) ? geom_* : flex_*`
to select parameter sources. Our implementation splits into two functions (geom-geom
and flex-rigid) rather than using sentinel indices, which is more Rust-idiomatic.

**New Model fields required** (add alongside `flex_friction`, `flex_solref`, etc.):
- `flex_priority: Vec<i32>` — per-flex contact priority (default 0)
- `flex_solmix: Vec<f64>` — per-flex solver mix weight (default 1.0)
- `flex_gap: Vec<f64>` — per-flex contact gap (default 0.0)
- `flex_friction` upgrade: `Vec<f64>` → `Vec<Vector3<f64>>` — per-flex 3-component
  friction `[slide, spin, roll]` (MuJoCo stores `nflex x 3`; our current scalar is
  a simplification that loses torsional/rolling data)

**Note:** Until `flex_friction` is upgraded to `Vec<Vector3<f64>>`, the function uses
the scalar value for all three components (matching the current `make_contact_flex_rigid`
behavior at line 5787–5791).

```rust
/// Compute combined contact parameters for a flex-rigid collision pair.
///
/// Mirrors `contact_param()` for geom-geom pairs, but reads flex_* fields
/// for the flex entity and geom_* fields for the rigid entity. Follows
/// MuJoCo's `mj_contactParam()` with f1=flex_id, f2=-1 (geom).
///
/// Returns: (condim, gap, solref, solimp, friction[5])
fn contact_param_flex_rigid(
    model: &Model,
    flex_id: usize,
    geom_idx: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    // 1. Load parameters from both entities
    let priority_flex = model.flex_priority[flex_id];
    let priority_geom = model.geom_priority[geom_idx];
    let gap = model.flex_gap[flex_id] + model.geom_gap[geom_idx];

    // 2. Priority check — higher priority entity's params win entirely
    if priority_flex > priority_geom {
        let f = model.flex_friction[flex_id]; // scalar until Vec<Vector3> upgrade
        return (
            model.flex_condim[flex_id],
            gap,
            model.flex_solref[flex_id],
            model.flex_solimp[flex_id],
            [f, f, f, f, f],  // scalar → uniform 5-element unpack
        );
    }
    if priority_geom > priority_flex {
        let f = model.geom_friction[geom_idx];
        return (
            model.geom_condim[geom_idx],
            gap,
            model.geom_solref[geom_idx],
            model.geom_solimp[geom_idx],
            [f.x, f.x, f.y, f.z, f.z],  // 3→5 unpack
        );
    }

    // 3. Equal priority — combine
    let condim = model.flex_condim[flex_id].max(model.geom_condim[geom_idx]);

    // 3a. Solmix weight (flex is entity 1, geom is entity 2)
    let s1 = model.flex_solmix[flex_id];
    let s2 = model.geom_solmix[geom_idx];
    let mix = solmix_weight(s1, s2);

    // 3b. Solref combination
    let solref1 = model.flex_solref[flex_id];
    let solref2 = model.geom_solref[geom_idx];
    let solref = if solref1[0] > 0.0 && solref2[0] > 0.0 {
        [mix * solref1[0] + (1.0 - mix) * solref2[0],
         mix * solref1[1] + (1.0 - mix) * solref2[1]]
    } else {
        [solref1[0].min(solref2[0]), solref1[1].min(solref2[1])]
    };

    // 3c. Solimp: weighted average
    let solimp1 = model.flex_solimp[flex_id];
    let solimp2 = model.geom_solimp[geom_idx];
    let solimp = std::array::from_fn(|i| mix * solimp1[i] + (1.0 - mix) * solimp2[i]);

    // 3d. Friction: element-wise max
    // Flex friction is currently scalar (applied to all components).
    // When upgraded to Vec<Vector3<f64>>, use flex_friction[flex_id].x/.y/.z.
    let ff = model.flex_friction[flex_id];
    let gf = model.geom_friction[geom_idx];
    let fri = [ff.max(gf.x), ff.max(gf.x),   // sliding1, sliding2
               ff.max(gf.y),                    // torsional
               ff.max(gf.z), ff.max(gf.z)];    // rolling1, rolling2

    (condim, gap, solref, solimp, fri)
}
```

**`make_contact_flex_rigid()` becomes a thin wrapper** (mirrors `make_contact_from_geoms`):

```rust
fn make_contact_flex_rigid(
    model: &Model,
    vertex_idx: usize,
    geom_idx: usize,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
    margin: f64,  // #27: effective margin for this pair
) -> Contact {
    let flex_id = model.flexvert_flexid[vertex_idx];
    let (condim, gap, solref, solimp, mu) = contact_param_flex_rigid(model, flex_id, geom_idx);
    let includemargin = margin - gap;

    Contact::with_condim_full(
        pos, normal, depth, geom_idx, geom_idx, mu, condim, solref, solimp, includemargin,
    )
}
```

### Shared Data Flow Diagram

```
   MJCF XML
      │
      ▼
  ┌─────────────┐     New fields: margin, gap (#27),
  │  parser.rs   │     priority (#25), solmix (#26)
  │  parse_geom  │     on MjcfGeom struct
  └──────┬───────┘
         │
         ▼
  ┌──────────────────┐     New Model fields:
  │ model_builder.rs  │     geom_priority: Vec<i32>     (#25)
  │ process_geom()    │     geom_solmix: Vec<f64>       (#26)
  │                   │     geom_margin: Vec<f64>        (#27, already exists, wire parsing)
  │                   │     geom_gap: Vec<f64>           (#27, already exists, wire parsing)
  └──────┬────────────┘
         │
         ▼
  ┌──────────────────────┐
  │ mujoco_pipeline.rs    │
  │                       │
  │  Broadphase:          │     #27: Add margin to AABB expansion
  │  SAP + distance cull  │         + automatic contact distance cull
  │         │             │
  │         ▼             │
  │  Narrow-phase:        │     #27: Pass margin to collide_*() functions
  │  collide_*() funcs    │         Return contacts when dist < margin
  │         │             │
  │         ▼             │
  │  contact_param()      │     #24: friction max
  │  (NEW unified func)   │     #25: priority gating
  │         │             │     #26: solmix weighting
  │         │             │     #27: gap combination
  │         ▼             │
  │  make_contact_from_   │     #27: includemargin = margin - gap
  │  geoms() (updated)    │         Contact.includemargin: f64
  │         │             │
  │         ▼             │
  │  apply_pair_overrides │     #27: set includemargin from pair
  │  (for explicit pairs) │
  │         │             │
  │         ▼             │
  │  assemble_contact_    │     #27: efc_margin = contact.includemargin
  │  system()             │         pos - margin in impedance + aref
  └───────────────────────┘
```

### Implementation Order Within Batch

The items must land in a specific order to avoid rework:

```
  #24 (friction max)          — foundation: fix the combination rule
    │
    ▼
  #25 (priority)              — adds priority gate BEFORE combination
    │
    ▼
  #26 (solmix)                — replaces combine_solver_params() with
    │                           solmix-weighted averaging
    │
    ▼
  contact_param() unification — merge #24+#25+#26 into single function,
    │                           delete combine_solver_params()
    │
    ▼
  #27 (margin/gap)            — last: broadest refactoring scope,
                                touches narrow-phase + broadphase + assembly
```

**Why this order:**
1. **#24 first** — 3-line fix, no new fields, no new parsing. Immediate correctness win.
2. **#25 second** — adds `geom_priority` field + parsing. The priority gate wraps around
   the combination rule that #24 just fixed. If #24 isn't done, the fallback path
   (equal priority) would still use geometric mean.
3. **#26 third** — adds `geom_solmix` field + parsing. Replaces `combine_solver_params()`
   with solmix-weighted averaging. This is when we create the unified `contact_param()`
   function (or refactor into it), since #24+#25+#26 are now all in place.
4. **#27 last** — broadest scope (touches every `collide_*` function, broadphase,
   Contact struct, constraint assembly). Depends on `contact_param()` existing to
   get gap from it. Landing this first would create unnecessary merge conflicts
   with #24/#25/#26.

### Shared Test Fixture

All four items can be validated against a single canonical test model:

```xml
<mujoco>
  <default>
    <geom condim="3"/>
  </default>

  <worldbody>
    <!-- Ground plane: high friction, priority 0 (default) -->
    <geom name="ground" type="plane" size="5 5 0.1"
          friction="0.8 0.01 0.001"
          solref="0.02 1.0" solimp="0.9 0.95 0.001 0.5 2.0"
          margin="0.003" gap="0.001"
          solmix="1.0"/>

    <!-- Test body: low friction, different solver params -->
    <body pos="0 0 0.5">
      <freejoint/>
      <geom name="box" type="box" size="0.1 0.1 0.1"
            friction="0.3 0.005 0.0001"
            solref="0.04 0.8" solimp="0.8 0.9 0.002 0.6 3.0"
            margin="0.002" gap="0.0005"
            solmix="2.0"/>
    </body>

    <!-- Priority test body -->
    <body pos="0.5 0 0.5">
      <freejoint/>
      <geom name="priority_box" type="box" size="0.1 0.1 0.1"
            friction="0.5 0.008 0.0003"
            priority="1"
            solref="0.03 0.9" solimp="0.85 0.92 0.0015 0.55 2.5"
            margin="0.001" gap="0.0"
            solmix="1.0"/>
    </body>
  </worldbody>
</mujoco>
```

**Expected results per item:**

**#24 — Friction (ground ↔ box, equal priority):**
- `sliding = max(0.8, 0.3) = 0.8` (not `sqrt(0.24) ≈ 0.490`)
- `torsional = max(0.01, 0.005) = 0.01`
- `rolling = max(0.001, 0.0001) = 0.001`
- `mu = [0.8, 0.8, 0.01, 0.001, 0.001]`

**#25 — Priority (ground ↔ priority_box):**
- `priority_box` has `priority=1`, ground has `priority=0`
- Winner: `priority_box` → use its params verbatim:
  - `friction = [0.5, 0.5, 0.008, 0.0003, 0.0003]`
  - `solref = [0.03, 0.9]`
  - `solimp = [0.85, 0.92, 0.0015, 0.55, 2.5]`
  - `condim = 3` (from priority_box)

**#26 — Solmix (ground ↔ box, equal priority):**
- `mix = solmix_ground / (solmix_ground + solmix_box) = 1.0 / (1.0 + 2.0) = 1/3`
- `w_ground = 1/3, w_box = 2/3`
- Standard reference (both `solref[0] > 0`):
  - `solref[0] = (1/3)*0.02 + (2/3)*0.04 = 0.03333...`
  - `solref[1] = (1/3)*1.0  + (2/3)*0.8  = 0.8666...`
- `solimp[0] = (1/3)*0.9  + (2/3)*0.8  = 0.8333...`
- `solimp[1] = (1/3)*0.95 + (2/3)*0.9  = 0.9166...`
- `solimp[2] = (1/3)*0.001+ (2/3)*0.002= 0.001666...`
- `solimp[3] = (1/3)*0.5  + (2/3)*0.6  = 0.5666...`
- `solimp[4] = (1/3)*2.0  + (2/3)*3.0  = 2.6666...`

**#27 — Margin/Gap (ground ↔ box):**
- `effective_margin = 0.003 + 0.002 = 0.005`
- `effective_gap = 0.001 + 0.0005 = 0.0015`
- `includemargin = 0.005 - 0.0015 = 0.0035`
- Contact activates when `dist < 0.005` (surfaces up to 5mm apart)
- Contact excluded when `dist >= 0.0035` (gap buffer zone)
- Force onset at `dist < 0.0035`

### Cross-Item Edge Cases

| Scenario | Items Involved | Expected Behavior |
|----------|---------------|-------------------|
| Both frictions zero | #24 | `max(0, 0) = 0` — frictionless contact (condim=1 behavior) |
| One priority higher, both solmix nonzero | #25, #26 | Priority wins — solmix is NOT consulted |
| Equal priority, both solmix zero | #26 | `mix = 0.5` — equal weighting |
| Equal priority, one solmix zero | #26 | The nonzero-solmix geom's params used entirely |
| Margin > 0 but gap = 0 | #27 | `includemargin = margin` — no buffer zone, force onset at `dist < margin` |
| Margin = 0 and gap = 0 (default) | #27 | `includemargin = 0.0` — current behavior preserved exactly |
| Pair override on prioritized contact | #25, pair | Pair overrides ALL params including priority-selected ones |
| Direct solref (solref[0] < 0) with solmix | #26 | Uses `min()` instead of weighted average |
| Flex-rigid with priority | #25 | Same `mj_contactParam()` logic, reads `flex_priority` |

---

### 24. Friction Combination Rule: Geometric Mean → Element-Wise Max
**Status:** ✅ Done | **Effort:** S | **Prerequisites:** None

#### MuJoCo Reference

`engine_collision_driver.c`, `mj_contactParam()`, equal-priority branch:

```c
// friction: max
for (int i=0; i < 3; i++) {
    fri[i] = mju_max(friction1[i], friction2[i]);
}

// unpack 5D friction
friction[0] = fri[0];  // sliding1
friction[1] = fri[0];  // sliding2 (= sliding1, MuJoCo symmetry)
friction[2] = fri[1];  // torsional
friction[3] = fri[2];  // rolling1
friction[4] = fri[2];  // rolling2 (= rolling1, MuJoCo symmetry)
```

MuJoCo stores 3 friction values per geom (`geom_friction[g*3 + {0,1,2}]` =
sliding, torsional, rolling) and unpacks to 5 for the contact (`mu[5]` =
sliding1, sliding2, torsional, rolling1, rolling2). The 3→5 unpack duplicates
sliding and rolling into both tangent/rolling directions. This is important
for condim=4 (elliptic cone) and condim=6 (full torsional+rolling).

#### Current State

Two contact creation paths use **different** friction combination rules:

| Path | Function | Line | Rule | Correct? |
|------|----------|------|------|----------|
| Rigid-rigid | `make_contact_from_geoms()` | 6054 | `sqrt(f1 * f2)` (geometric mean) | **NO** |
| Flex-rigid | `make_contact_flex_rigid()` | 5776 | `flex_f.max(rigid_f)` (element-wise max) | Yes |

**Exact buggy code** (`make_contact_from_geoms()`, lines 6068–6070):
```rust
let sliding = (f1.x * f2.x).sqrt();    // WRONG: geometric mean
let torsional = (f1.y * f2.y).sqrt();   // WRONG: geometric mean
let rolling = (f1.z * f2.z).sqrt();     // WRONG: geometric mean
```

This was documented as known non-conformance in `future_work_2.md` (Decision D3),
deferred from Phase 2 as "better done as a deliberate conformance task."

**Impact:** Every rigid-rigid contact between geoms with asymmetric friction values
produces wrong friction coefficients. Example: geoms with friction 0.3 and 0.7
produce `sqrt(0.21) ≈ 0.458` instead of `0.7`. This affects all trajectory
comparison tests involving rigid-rigid contacts.

#### Specification

##### S1. Fix friction combination in `make_contact_from_geoms()`

Replace lines 6068–6070:

```rust
// BEFORE (geometric mean — wrong):
let sliding = (f1.x * f2.x).sqrt();
let torsional = (f1.y * f2.y).sqrt();
let rolling = (f1.z * f2.z).sqrt();

// AFTER (element-wise max — matches MuJoCo):
let sliding = f1.x.max(f2.x);
let torsional = f1.y.max(f2.y);
let rolling = f1.z.max(f2.z);
```

No other changes to this function. The 3→5 friction unpack already happens in
`Contact::with_condim()` which sets `mu = [sliding, sliding, torsional, rolling,
rolling]` — this matches MuJoCo's unpack pattern.

##### S2. Verify flex-rigid path is already correct

`make_contact_flex_rigid()` (line 5786–5791) already uses `flex_f.max(rigid_f.x)`
etc. No change needed. Confirm with a regression test.

##### S3. No change to explicit pairs

`apply_pair_overrides()` (line 6021) overwrites friction entirely from
`ContactPair.friction` — the combination rule is bypassed. No change needed.

##### S4. Doc correction

Search for any comment or doc that says "geometric mean" in the context of
friction combination. Replace with "element-wise max." Known locations:
- Comment in `parser.rs` around line 1487: "When two geoms collide, their
  params are combined (min for solref, max for solimp)" — this omits friction;
  update to include friction max.
- Any reference in `future_work_2.md` Decision D3.

#### Acceptance Criteria

1. **Unit test — asymmetric friction:** Two box geoms with `friction=[0.3, 0.005,
   0.0001]` and `friction=[0.7, 0.01, 0.001]` collide. Verify:
   - `contact.mu = [0.7, 0.7, 0.01, 0.001, 0.001]`
   - NOT `[0.458, 0.458, 0.00707, 0.000316, 0.000316]` (old geometric mean)
2. **Unit test — symmetric friction:** Two geoms with identical `friction=[0.5, 0.005,
   0.0001]`. Both rules produce the same result (`max(x,x) = x = sqrt(x*x)`).
   Verify `contact.mu = [0.5, 0.5, 0.005, 0.0001, 0.0001]`.
3. **Unit test — zero friction:** One geom with `friction=[0.0, 0.0, 0.0]`, other
   with `friction=[0.5, 0.005, 0.0001]`. Verify `contact.mu = [0.5, 0.5, 0.005,
   0.0001, 0.0001]`. (Geometric mean would produce all zeros — this is the worst
   case for the old bug.)
4. **Flex-rigid regression:** Flex-rigid contact with `flex_friction=0.4` and
   `geom_friction=[0.6, 0.008, 0.0002]`. Verify `contact.mu = [0.6, 0.6, 0.008,
   0.0002, 0.0002]` — unchanged from before.
5. **Pair override regression:** Explicit `<contact><pair friction="0.9 0.9 0.02
   0.003 0.003">` overrides combination. Verify `contact.mu = [0.9, 0.9, 0.02,
   0.003, 0.003]` regardless of geom frictions.
6. **Existing test audit:** Run full sim test suite. Any test that hardcoded
   geometric mean expected values must be updated to element-wise max values.
   List affected tests in the PR.

#### Implementation Risk

**Minimal.** This is a 3-line change in a single function. The only risk is
existing tests that depend on the old (wrong) geometric mean behavior — these
must be found and updated, not suppressed.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `make_contact_from_geoms()` (lines 6068–6070)
- `sim/L0/tests/integration/` — new `contact_friction_combination.rs` test file
- `sim/L0/mjcf/src/parser.rs` — comment fix (line ~1487)

---

### 25. `geom/@priority` — Contact Priority
**Status:** ✅ Done | **Effort:** S | **Prerequisites:** #24

#### MuJoCo Reference

`engine_collision_driver.c`, `mj_contactParam()`, priority branch:

```c
// different priority: copy from item with higher priority
if (priority1 > priority2) {
    *condim = condim1;
    mju_copy(solref, solref1, mjNREF);
    mju_copy(solimp, solimp1, mjNIMP);
    mju_copy(fri, friction1, 3);
}
else if (priority1 < priority2) {
    *condim = condim2;
    mju_copy(solref, solref2, mjNREF);
    mju_copy(solimp, solimp2, mjNIMP);
    mju_copy(fri, friction2, 3);
}
// else: equal priority → fall through to combination rules (#24, #26)
```

When priorities differ, the higher-priority geom's **condim, solref, solimp, AND
friction** are all copied verbatim. No combination occurs. Solmix is not
consulted. Gap is still additive (gap always adds, priority or not — see #27).

MuJoCo default: `priority = 0` for all geoms.

#### Current State

- `geom_priority` field: **does not exist** on `Model` struct.
- `priority` attribute: **not parsed** from `<geom>` elements.
- `MjcfGeom` struct (`types.rs:983`): no `priority` field.
- `MjcfGeomDefaults` struct (`types.rs:594`): no `priority` field.
- Contact parameter combination always falls through to the equal-priority
  combination path (currently geometric mean for friction, min/max for
  solref/solimp).

#### Specification

##### S1. Add `priority` to `MjcfGeom` struct

In `sim/L0/mjcf/src/types.rs`, add to `MjcfGeom`:

```rust
pub priority: i32,  // default: 0 (matching MuJoCo)
```

Default in `MjcfGeom::default()`: `priority: 0`.

##### S2. Parse `priority` from `<geom>` elements

In `sim/L0/mjcf/src/parser.rs`, in `parse_geom_attrs()`:

```rust
geom.priority = parse_int_attr(e, "priority").unwrap_or(0);
```

##### S3. Add `priority` to defaults cascade

In `sim/L0/mjcf/src/types.rs`, add to `MjcfGeomDefaults`:

```rust
pub priority: Option<i32>,
```

In `sim/L0/mjcf/src/parser.rs`, in `parse_geom_defaults()`:

```rust
defaults.priority = parse_int_attr(e, "priority");
```

In `sim/L0/mjcf/src/defaults.rs`, in `apply_to_geom()`:

```rust
if geom.priority == 0 {  // still at default
    if let Some(priority) = defaults.priority { result.priority = priority; }
}
```

##### S4. Add `geom_priority` to `Model`

In `sim/L0/core/src/mujoco_pipeline.rs`, add to `Model`:

```rust
pub geom_priority: Vec<i32>,
```

Initialize in `model_builder.rs` `process_geom()`:

```rust
self.geom_priority.push(geom.priority);
```

##### S5. Priority gate in `make_contact_from_geoms()`

Add priority check **before** the friction combination (which #24 fixed to
element-wise max). This is the first step toward the unified `contact_param()`
function described in the Interaction Model:

```rust
fn make_contact_from_geoms(
    model: &Model,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
    geom1: usize,
    geom2: usize,
) -> Contact {
    let p1 = model.geom_priority[geom1];
    let p2 = model.geom_priority[geom2];

    if p1 != p2 {
        // Higher priority wins — use its params verbatim
        let winner = if p1 > p2 { geom1 } else { geom2 };
        let f = model.geom_friction[winner];
        let condim = model.geom_condim[winner];
        let solref = model.geom_solref[winner];
        let solimp = model.geom_solimp[winner];
        return Contact::with_condim(
            pos, normal, depth, geom1, geom2,
            f.x, f.y, f.z, condim, solref, solimp,
        );
    }

    // Equal priority — fall through to combination rules
    let f1 = model.geom_friction[geom1];
    let f2 = model.geom_friction[geom2];
    let sliding = f1.x.max(f2.x);     // #24: element-wise max
    let torsional = f1.y.max(f2.y);
    let rolling = f1.z.max(f2.z);
    let condim = model.geom_condim[geom1].max(model.geom_condim[geom2]);
    let (solref, solimp) = combine_solver_params(
        model.geom_solref[geom1], model.geom_solimp[geom1],
        model.geom_solref[geom2], model.geom_solimp[geom2],
    );
    Contact::with_condim(
        pos, normal, depth, geom1, geom2,
        sliding, torsional, rolling, condim, solref, solimp,
    )
}
```

##### S6. Priority in flex-rigid contacts

`make_contact_flex_rigid()` also needs a priority check. MuJoCo's
`mj_contactParam()` uses the same function for flex-rigid (passing flex index
via `f1`/`f2` parameters). We need `flex_priority: Vec<i32>` on Model (default
0), populated from `<flex>` element parsing. The same priority gate applies.

**Note:** `flex_priority` does not exist yet. Add it to Model and initialize to
0 for all flex bodies (we don't parse `<flex priority="...">` yet — matching
MuJoCo default). This ensures flex-rigid contacts behave correctly when a geom
has nonzero priority.

##### S7. Explicit pairs bypass priority

`apply_pair_overrides()` already overwrites all contact parameters (condim,
friction, solref, solimp). Priority does not affect explicit pairs because the
pair's parameters are specified directly. No change needed here.

#### Acceptance Criteria

1. **Priority wins — all params:** Geom A (`priority=1`, `friction=[0.5, 0.008,
   0.0003]`, `solref=[0.03, 0.9]`, `solimp=[0.85, 0.92, 0.0015, 0.55, 2.5]`)
   vs Geom B (`priority=0`, `friction=[0.8, 0.01, 0.001]`, `solref=[0.02, 1.0]`,
   `solimp=[0.9, 0.95, 0.001, 0.5, 2.0]`). Contact uses A's params verbatim:
   - `mu = [0.5, 0.5, 0.008, 0.0003, 0.0003]`
   - `solref = [0.03, 0.9]`
   - `solimp = [0.85, 0.92, 0.0015, 0.55, 2.5]`
   - `condim = 3` (from A)
2. **Equal priority — combination:** Both geoms `priority=0` (default). Falls
   through to element-wise max for friction, min/max for solref/solimp. Same
   as #24 behavior.
3. **Default priority regression:** All existing tests pass unchanged. Default
   `priority=0` for all geoms means all contacts use the equal-priority path,
   which is exactly what exists today (after #24 fix).
4. **Negative priority:** Geom A `priority=-1`, Geom B `priority=0`. B wins
   (higher priority). MuJoCo allows negative priorities.
5. **Priority does not affect gap:** Gap is always additive regardless of priority.
   Geom A `priority=1, gap=0.001`, Geom B `priority=0, gap=0.002`. Combined
   gap = 0.003 (not 0.001).
6. **Pair override ignores priority:** Explicit `<contact><pair>` overrides all
   params regardless of geom priorities.
7. **Flex-rigid with priority:** Flex body (`priority=0`) vs geom (`priority=1`).
   Geom wins — its friction/solver params are used verbatim.
8. **MJCF round-trip:** Parse `<geom priority="2"/>`, verify
   `model.geom_priority[i] == 2`.

#### Implementation Risk

**Low.** Adds a new field + parsing + a conditional branch before the existing
combination logic. The conditional is a pure superset — it adds a new code path
(priority differs) while leaving the existing path (equal priority) unchanged.
Default `priority=0` means the new code path is never taken unless the model
explicitly sets priority, so regression risk is near zero.

#### Files

- `sim/L0/mjcf/src/types.rs` — `MjcfGeom.priority`, `MjcfGeomDefaults.priority`
- `sim/L0/mjcf/src/parser.rs` — parse `priority` in `parse_geom_attrs()` and
  `parse_geom_defaults()`
- `sim/L0/mjcf/src/defaults.rs` — cascade `priority` in `apply_to_geom()`
- `sim/L0/mjcf/src/model_builder.rs` — `self.geom_priority.push(geom.priority)`
  in `process_geom()`
- `sim/L0/core/src/mujoco_pipeline.rs` — `Model.geom_priority: Vec<i32>`,
  priority gate in `make_contact_from_geoms()`, `flex_priority: Vec<i32>` on
  Model (initialized to 0)
- `sim/L0/tests/integration/` — new tests in `contact_priority.rs`

---

### 26. `solmix` — Solver Parameter Mixing Weight
**Status:** ✅ Done | **Effort:** S | **Prerequisites:** #24, #25

#### MuJoCo Reference

`engine_collision_driver.c`, `mj_contactParam()`, equal-priority branch:

```c
// compute solver mix factor
mjtNum mix;
if (solmix1 >= mjMINVAL && solmix2 >= mjMINVAL) {
    mix = solmix1 / (solmix1 + solmix2);
} else if (solmix1 < mjMINVAL && solmix2 < mjMINVAL) {
    mix = 0.5;
} else if (solmix1 < mjMINVAL) {
    mix = 0.0;
} else {
    mix = 1.0;
}

// reference standard: mix
if (solref1[0] > 0 && solref2[0] > 0) {
    for (int i=0; i < mjNREF; i++) {
        solref[i] = mix*solref1[i] + (1-mix)*solref2[i];
    }
}
// reference direct: min
else {
    for (int i=0; i < mjNREF; i++) {
        solref[i] = mju_min(solref1[i], solref2[i]);
    }
}

// impedance: mix
for (int i=0; i < mjNIMP; i++) {
    solimp[i] = mix*solimp1[i] + (1-mix)*solimp2[i];
}
```

Key semantics:
- `mjMINVAL ≈ 1e-15`: threshold for treating solmix as "zero"
- **Standard solref** (`solref[0] > 0`): weighted average using mix factor
- **Direct solref** (`solref[0] <= 0`): element-wise minimum, ignoring solmix
- **Solimp**: always weighted average using mix factor
- **Friction**: NOT affected by solmix (always element-wise max)
- **Condim**: NOT affected by solmix (always max)
- Solmix is only consulted in the **equal-priority** branch. When priorities
  differ (#25), the winner's params are copied verbatim.

MuJoCo default: `solmix = 1.0` for all geoms. With equal solmix on both geoms,
`mix = 0.5` — equal weighting. This gives different results from our current
min/max approximation.

#### Current State

- `geom_solmix` field: **does not exist** on `Model`.
- `solmix` attribute: **not parsed** from `<geom>`.
- `MjcfGeom` struct: no `solmix` field.
- `combine_solver_params()` (line 2091): uses element-wise min for solref and
  element-wise max for solimp — **wrong**. Should use solmix-weighted average.
- Documented as known divergence in `future_work_2.md`.

**Concrete divergence example** (default solmix=1.0 on both geoms, equal weight):
- Our code: `solref = [min(0.02, 0.04), min(1.0, 0.8)] = [0.02, 0.8]`
- MuJoCo:   `solref = [0.5*0.02 + 0.5*0.04, 0.5*1.0 + 0.5*0.8] = [0.03, 0.9]`

#### Specification

##### S1. Add `solmix` to `MjcfGeom` struct

In `sim/L0/mjcf/src/types.rs`, add to `MjcfGeom`:

```rust
pub solmix: f64,  // default: 1.0 (matching MuJoCo)
```

Default in `MjcfGeom::default()`: `solmix: 1.0`.

##### S2. Parse `solmix` from `<geom>` elements

In `sim/L0/mjcf/src/parser.rs`, in `parse_geom_attrs()`:

```rust
geom.solmix = parse_float_attr(e, "solmix").unwrap_or(1.0);
```

##### S3. Add `solmix` to defaults cascade

In `MjcfGeomDefaults`: add `pub solmix: Option<f64>`.
Parse in `parse_geom_defaults()`: `defaults.solmix = parse_float_attr(e, "solmix")`.
Cascade in `apply_to_geom()`:

```rust
if (geom.solmix - 1.0).abs() < 1e-10 {  // still at default
    if let Some(solmix) = defaults.solmix { result.solmix = solmix; }
}
```

##### S4. Add `geom_solmix` to `Model`

In `sim/L0/core/src/mujoco_pipeline.rs`, add to `Model`:

```rust
pub geom_solmix: Vec<f64>,
```

Initialize in `process_geom()`:

```rust
self.geom_solmix.push(geom.solmix);
```

##### S5. Replace `combine_solver_params()` with solmix-weighted combination

Delete `combine_solver_params()` (lines 2091–2110) and replace with a new
function that implements the MuJoCo solmix logic:

```rust
/// Compute solmix weight, matching MuJoCo's edge-case handling.
fn solmix_weight(s1: f64, s2: f64) -> f64 {
    const MJ_MINVAL: f64 = 1e-15;
    if s1 >= MJ_MINVAL && s2 >= MJ_MINVAL {
        s1 / (s1 + s2)
    } else if s1 < MJ_MINVAL && s2 < MJ_MINVAL {
        0.5
    } else if s1 < MJ_MINVAL {
        0.0  // geom2 dominates
    } else {
        1.0  // geom1 dominates
    }
}

/// Combine solref using solmix weight.
/// Standard reference (solref[0] > 0): weighted average.
/// Direct reference (solref[0] <= 0): element-wise minimum.
fn combine_solref(
    solref1: [f64; 2],
    solref2: [f64; 2],
    mix: f64,
) -> [f64; 2] {
    if solref1[0] > 0.0 && solref2[0] > 0.0 {
        [mix * solref1[0] + (1.0 - mix) * solref2[0],
         mix * solref1[1] + (1.0 - mix) * solref2[1]]
    } else {
        [solref1[0].min(solref2[0]),
         solref1[1].min(solref2[1])]
    }
}

/// Combine solimp using solmix weight (always weighted average).
fn combine_solimp(
    solimp1: [f64; 5],
    solimp2: [f64; 5],
    mix: f64,
) -> [f64; 5] {
    std::array::from_fn(|i| mix * solimp1[i] + (1.0 - mix) * solimp2[i])
}
```

##### S6. Update `make_contact_from_geoms()` to use solmix

In the equal-priority branch (after #25's priority gate):

```rust
// Equal priority — combine with solmix weighting
let mix = solmix_weight(model.geom_solmix[geom1], model.geom_solmix[geom2]);
let solref = combine_solref(
    model.geom_solref[geom1], model.geom_solref[geom2], mix,
);
let solimp = combine_solimp(
    model.geom_solimp[geom1], model.geom_solimp[geom2], mix,
);

// Friction: element-wise max (NOT affected by solmix) — #24
let f1 = model.geom_friction[geom1];
let f2 = model.geom_friction[geom2];
let sliding = f1.x.max(f2.x);
let torsional = f1.y.max(f2.y);
let rolling = f1.z.max(f2.z);
```

##### S7. Create unified `contact_param()` function

At this point, #24 (friction max), #25 (priority gate), and #26 (solmix) are all
in place. Refactor `make_contact_from_geoms()` to call a unified `contact_param()`
function as described in the Interaction Model. This is the natural refactoring
point — all three parameter combination changes are complete and can be
consolidated.

Delete `combine_solver_params()` entirely.

##### S8. Flex-rigid solmix

Add `flex_solmix: Vec<f64>` to Model (default 1.0), analogous to #25's
`flex_priority`. Update `make_contact_flex_rigid()` to use solmix-weighted
combination in the equal-priority branch.

#### Acceptance Criteria

1. **Default solmix — equal weight:** Two geoms with default `solmix=1.0`,
   `solref=[0.02, 1.0]` and `solref=[0.04, 0.8]`. Contact `solref = [0.03, 0.9]`
   (weighted average with mix=0.5). NOT `[0.02, 0.8]` (old min behavior).
2. **Asymmetric solmix:** Geom A `solmix=2.0`, Geom B `solmix=1.0`. `mix =
   2/(2+1) = 2/3`. Contact:
   - `solref[0] = (2/3)*solref_A[0] + (1/3)*solref_B[0]`
   - `solimp[i] = (2/3)*solimp_A[i] + (1/3)*solimp_B[i]`
3. **Both solmix zero:** `mix = 0.5`. Equal weighting. No NaN/inf.
4. **One solmix zero:** Geom A `solmix=0`, Geom B `solmix=1.0`. `mix = 0.0` →
   Geom B's params used entirely. Verify `solref = solref_B`, `solimp = solimp_B`.
5. **Direct solref (negative):** Geom A `solref=[-100, 1]`, Geom B `solref=[-200,
   1]`. Uses element-wise min regardless of solmix: `solref = [-200, 1]`.
6. **Mixed standard/direct solref:** Geom A `solref=[0.02, 1.0]` (standard), Geom
   B `solref=[-100, 1]` (direct). One is positive, one is negative → `solref1[0]
   > 0 && solref2[0] > 0` is false → uses element-wise min: `solref = [-100, 1]`.
7. **Solmix does not affect friction:** Geom A `solmix=100, friction=[0.3, ...]`,
   Geom B `solmix=1, friction=[0.8, ...]`. Friction = `max(0.3, 0.8) = 0.8`
   regardless of solmix weights.
8. **Priority bypasses solmix:** Geom A `priority=1, solmix=1`, Geom B
   `priority=0, solmix=100`. A wins — solmix is not consulted.
9. **Default regression:** All existing tests pass. Default `solmix=1.0` with
   equal weights produces `(0.5*a + 0.5*b)` which differs from old `min(a,b)` /
   `max(a,b)`. Tests that hardcoded old min/max expected values for solref/solimp
   must be updated. List affected tests.
10. **Pair override bypasses solmix:** Explicit `<contact><pair>` overrides
    solref/solimp directly.

#### Implementation Risk

**Moderate — behavioral change.** Unlike #24 (which only changes the wrong
geometric mean to the correct max), this changes the correct-but-approximate
min/max combination to the correct solmix-weighted average. The default
`solmix=1.0` produces `mix=0.5` (equal weight), which gives **different**
results from element-wise min (solref) and max (solimp):

| Parameter | Old (min/max) | New (mix=0.5) | Change |
|-----------|---------------|---------------|--------|
| solref[0] | min(0.02, 0.04) = 0.02 | 0.5*0.02 + 0.5*0.04 = 0.03 | +50% |
| solref[1] | min(1.0, 0.8) = 0.8 | 0.5*1.0 + 0.5*0.8 = 0.9 | +12.5% |
| solimp[0] | max(0.9, 0.8) = 0.9 | 0.5*0.9 + 0.5*0.8 = 0.85 | -5.6% |

This means existing trajectory tests with asymmetric solver parameters will
produce different contact forces after this change. This is **correct** (matches
MuJoCo), but requires test updates. For models where both geoms have identical
solver parameters, there is zero change (averaging identical values = the value).

**Mitigation:** Run the full sim test suite after implementing. Any test failures
are expected and indicate the test was relying on the old approximate behavior.
Update expected values to match MuJoCo reference.

#### Files

- `sim/L0/mjcf/src/types.rs` — `MjcfGeom.solmix`, `MjcfGeomDefaults.solmix`
- `sim/L0/mjcf/src/parser.rs` — parse `solmix` in `parse_geom_attrs()` and
  `parse_geom_defaults()`
- `sim/L0/mjcf/src/defaults.rs` — cascade `solmix` in `apply_to_geom()`
- `sim/L0/mjcf/src/model_builder.rs` — `self.geom_solmix.push(geom.solmix)`
- `sim/L0/core/src/mujoco_pipeline.rs` — `Model.geom_solmix: Vec<f64>`,
  `solmix_weight()`, `combine_solref()`, `combine_solimp()`, delete old
  `combine_solver_params()`, create unified `contact_param()`,
  `flex_solmix: Vec<f64>` on Model
- `sim/L0/tests/integration/` — new tests in `contact_solmix.rs`

---

### 27. Contact Margin/Gap Runtime Effect
**Status:** ✅ Done | **Effort:** S–M | **Prerequisites:** #24, #25, #26

#### MuJoCo Reference

Margin and gap flow through four stages of the collision pipeline:

**Stage 1 — Broadphase AABB expansion** (`engine_collision_driver.c`, `makeAAMM`):
```c
mjtNum margin = mjENABLED(mjENBL_OVERRIDE) ? 0.5*m->opt.o_margin : m->geom_margin[geom];
_aamm[j]   = cen - m->geom_rbound[geom] - margin;
_aamm[j+3] = cen + m->geom_rbound[geom] + margin;
```

**Stage 2 — Margin combination** (`mj_collideGeoms`):
```c
// Automatic contacts: sum geom margins
margin = mj_assignMargin(m, m->geom_margin[g1] + m->geom_margin[g2]);
// Explicit pairs: use pair margin
margin = mj_assignMargin(m, m->pair_margin[ipair]);
```

**Stage 3 — Narrow-phase** — collision functions receive `margin` parameter:
```c
num = collisionFunc(m, d, con, g1, g2, margin);
```
Contacts are returned when `dist < margin` (surfaces may still be separated).

**Stage 4 — Contact creation** (`mj_collideGeoms` + `mj_setContact`):
```c
// Gap from mj_contactParam (additive, unaffected by priority)
mj_contactParam(m, &condim, &gap, solref, solimp, friction, g1, g2, -1, -1);

// Set includemargin = margin - gap
mj_setContact(m, con + i, condim, margin-gap, solref, solreffriction, solimp, friction);

// In mj_setContact:
con->includemargin = includemargin;
con->exclude = (con->dist >= includemargin);
```

**Stage 5 — Constraint assembly** (`engine_core_constraint.c`):
```c
// Contact pos and margin flow into constraint rows
efc_pos[row] = con->dist;
efc_margin[row] = con->includemargin;

// Impedance uses margin-shifted position
x = (pos - margin) / solimp[2];  // in getimpedance()

// Reference acceleration uses margin
aref = -b*vel - k*imp*(pos - margin);
```

**Key semantics:**
- `margin` controls **contact activation** — wider detection zone
- `gap` controls **force onset** — creates a buffer zone within the margin where
  contacts exist but produce no force
- `includemargin = margin - gap` is the force onset threshold
- `exclude = (dist >= includemargin)` — excluded contacts are detected but
  produce no constraint rows
- Gap is always additive from `mj_contactParam()`, unaffected by priority
- MuJoCo defaults: `margin = 0.0`, `gap = 0.0`

**con->dist vs depth convention:** In MuJoCo, `con->dist` is signed distance
(negative = penetration). In our codebase, `Contact.depth` is penetration depth
(positive = penetration). The relationship: `depth = -dist`. The constraint
assembly must account for this sign convention.

#### Current State

| Component | Status | Location |
|-----------|--------|----------|
| `Model.geom_margin: Vec<f64>` | Exists, zeroed | `mujoco_pipeline.rs:1349` |
| `Model.geom_gap: Vec<f64>` | Exists, zeroed | `mujoco_pipeline.rs:1351` |
| `<geom margin="...">` parsing | **NOT parsed** | `model_builder.rs` TODO comment |
| `<geom gap="...">` parsing | **NOT parsed** | `model_builder.rs` TODO comment |
| `ContactPair.margin` | Parsed from `<pair>` | `types.rs` |
| `ContactPair.gap` | Parsed from `<pair>` | `types.rs` |
| Broadphase (automatic) | No margin in AABB | SAP at line 5500 |
| Broadphase (explicit pairs) | Uses `pair.margin` ✅ | Distance cull at line 5558 |
| `collide_*()` signatures | No margin parameter | All return `Option<Contact>` |
| `collide_*()` activation | `penetration > 0.0` | All narrow-phase functions |
| `Contact.includemargin` | `bool`, always `false` | `mujoco_pipeline.rs:1874` |
| `apply_pair_overrides()` | Skips margin/gap | Comment "NOT applied here" |
| `assemble_contact_system()` | `margin = 0.0` hardcoded | Line ~15410 |
| `compute_aref()` | Correct signature, receives `0.0` | Line 14911 |
| `compute_impedance()` | Receives `violation` (no margin offset) | Line 14685 |
| `data.efc_margin` | Always `0.0` | Line ~15148 |

#### Specification

##### S1. Parse `margin` and `gap` from `<geom>` elements

Add to `MjcfGeom` struct (`types.rs`):

```rust
pub margin: f64,  // default: 0.0
pub gap: f64,     // default: 0.0
```

Parse in `parse_geom_attrs()` (`parser.rs`):

```rust
geom.margin = parse_float_attr(e, "margin").unwrap_or(0.0);
geom.gap = parse_float_attr(e, "gap").unwrap_or(0.0);
```

Add to `MjcfGeomDefaults` and cascade in `apply_to_geom()`. Only cascade when
still at default (0.0).

Wire in `process_geom()` (`model_builder.rs`):

```rust
self.geom_margin.push(geom.margin);
self.geom_gap.push(geom.gap);
```

This replaces the current zeroed initialization.

##### S2. Compute effective margin in collision dispatch

In the broadphase/collision dispatch code, compute the effective margin before
calling narrow-phase:

**Automatic contacts** — modify the SAP candidate processing loop (line ~5506):

```rust
// Compute effective margin for this pair
let margin = model.geom_margin[geom1] + model.geom_margin[geom2];

// Narrow-phase — pass margin
if let Some(contact) = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin) {
    data.contacts.push(contact);
    data.ncon += 1;
}
```

**Explicit pairs** — already uses `pair.margin` in distance cull (line 5558).
Pass it to narrow-phase too:

```rust
let margin = pair.margin;  // pair margin overrides geom margins
if let Some(mut contact) = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin) {
    apply_pair_overrides(&mut contact, pair);
    data.contacts.push(contact);
    data.ncon += 1;
}
```

##### S3. Add margin to broadphase AABB expansion (automatic contacts)

The SAP (sweep-and-prune) builds AABBs per geom. Currently AABBs use only
`geom_rbound`. For margin to work, AABBs must be expanded by margin:

```rust
let margin = model.geom_margin[geom_id];
let aabb_min = geom_pos - geom_rbound - margin;
let aabb_max = geom_pos + geom_rbound + margin;
```

Without this, SAP may reject geom pairs that are within margin distance but
not overlapping. The explicit pair path already handles this (line 5558).

**Performance note:** Expanding AABBs by margin increases the number of
candidate pairs that enter narrow-phase. For typical margins (1-5mm), this is
negligible. For large margins, SAP becomes less effective — but this matches
MuJoCo's behavior.

##### S4. Thread margin through `collide_geoms()` dispatcher

Update `collide_geoms()` signature to accept margin:

```rust
fn collide_geoms(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64,  // NEW
) -> Option<Contact>
```

Inside `collide_geoms()`, pass `margin` to the type-specific collision function.

##### S5. Update all `collide_*()` function signatures

Every narrow-phase function gets a `margin: f64` parameter and changes its
activation condition:

**Full list of affected functions** (from codebase audit):

| Function | Line | Current Check | New Check |
|----------|------|---------------|-----------|
| `collide_sphere_sphere()` | ~6887 | `penetration > 0.0` | `penetration > -margin` |
| `collide_capsule_capsule()` | ~6938 | `penetration > 0.0` | `penetration > -margin` |
| `collide_sphere_capsule()` | ~6999 | `penetration > 0.0` | `penetration > -margin` |
| `collide_sphere_box()` | ~7087 | `penetration > 0.0` | `penetration > -margin` |
| `collide_capsule_box()` | ~7416 | `penetration > 0.0` | `penetration > -margin` |
| `collide_box_box()` | ~7559 | `penetration > 0.0` | `penetration > -margin` |
| `collide_cylinder_sphere()` | ~7190 | `penetration > 0.0` | `penetration > -margin` |
| `collide_cylinder_capsule()` | ~7318 | `penetration > 0.0` | `penetration > -margin` |
| `collide_with_plane()` | ~6483 | `penetration > 0.0` | `penetration > -margin` |
| `collide_with_mesh()` | ~6311 | `penetration > 0.0` | `penetration > -margin` |
| `collide_with_hfield()` | ~6120 | `penetration > 0.0` | `penetration > -margin` |
| `collide_with_sdf()` | ~6197 | `penetration > 0.0` | `penetration > -margin` |
| `collide_mesh_plane()` | ~6436 | `penetration > 0.0` | `penetration > -margin` |
| `collide_cylinder_plane_impl()` | ~6701 | `penetration > 0.0` | `penetration > -margin` |

When margin=0.0 (default), `-margin = -0.0 = 0.0`, so `penetration > -0.0` is
equivalent to `penetration > 0.0` — **zero behavioral change for default case**.

When `penetration ∈ (-margin, 0]`, the contact is created with `depth =
penetration` (which is negative/zero). This represents a "pre-contact" — surfaces
are separated but within the margin zone.

##### S6. Change `Contact.includemargin` from `bool` to `f64`

In the `Contact` struct (line 1874), replace:

```rust
// BEFORE:
pub includemargin: bool,

// AFTER:
/// Distance threshold for constraint force onset.
/// `includemargin = effective_margin - effective_gap`
/// Constraint is excluded when `dist >= includemargin` (MuJoCo convention).
/// In our depth convention (positive = overlap): excluded when
/// `depth <= -includemargin`.
pub includemargin: f64,
```

**Breaking change.** Search for all uses of `contact.includemargin` in the
codebase and update:
- Construction sites in `Contact::new()`, `Contact::with_condim()` — change
  `false` to `0.0`
- Any conditional checks `if contact.includemargin` → remove or replace with
  `if contact.includemargin > 0.0`

##### S7. Set `includemargin` in contact creation

In `make_contact_from_geoms()` (or the unified `contact_param()` + wrapper):

```rust
fn make_contact_from_geoms(
    model: &Model,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
    geom1: usize,
    geom2: usize,
    margin: f64,  // effective margin (already computed in dispatch)
) -> Contact {
    let (condim, gap, solref, solimp, mu) = contact_param(model, geom1, geom2);
    let includemargin = margin - gap;

    let mut contact = Contact::with_condim_full(
        pos, normal, depth, geom1, geom2, mu, condim, solref, solimp, includemargin,
    );

    // MuJoCo exclude logic: contact exists but produces no constraint row
    // when dist >= includemargin. In our convention: depth <= -includemargin.
    // Note: we don't need an explicit exclude flag — the constraint assembly
    // already handles this via impedance (violation = depth + includemargin;
    // when violation <= 0, impedance and force are zero).
    contact
}
```

##### S8. Set `includemargin` in `apply_pair_overrides()`

In `apply_pair_overrides()` (line 6021), add:

```rust
// Margin/gap from explicit pair
contact.includemargin = pair.margin - pair.gap;
```

Remove the comment "margin/gap are NOT applied here."

##### S9. Wire `includemargin` into constraint assembly

In `assemble_contact_system()` (line ~15410), replace:

```rust
// BEFORE:
let margin = 0.0_f64;

// AFTER:
let margin = contact.includemargin;
```

This flows into:

1. **`data.efc_margin[row] = margin`** — already populated at line ~15148, just
   needs the correct value instead of 0.0.

2. **`compute_impedance(solimp, violation)`** — the `finalize_row!` macro already
   computes `violation = (pos_val - margin_val).abs()` at line ~15158. With the
   sign convention fix (`pos = -depth`) and `margin = includemargin`, this becomes:
   ```
   violation = |(-depth) - includemargin| = |-(depth + includemargin)| = depth + includemargin
   ```
   (When both `depth >= 0` and `includemargin >= 0`, which is the active constraint case.)

   **No code change needed in the macro.** The `.abs()` makes it sign-agnostic.

   Example: `depth = -0.001` (slightly separated) and `includemargin = 0.004`:
   `violation = |(-(-0.001)) - 0.004| = |0.001 - 0.004| = 0.003` → impedance is
   computed for 3mm of effective violation past the threshold. Correct.

   Example: `depth = 0.002` (penetrating) and `includemargin = 0.004`:
   `violation = |(-0.002) - 0.004| = |-0.006| = 0.006` → 6mm total violation. Correct.

3. **`compute_aref(k, b, imp, pos, margin, vel)`** — the formula is correct but
   the `pos` argument **must be negated** for contacts. The `finalize_row!` macro
   at line ~15424 currently passes `contact.depth` (positive = penetrating), but
   the `aref` formula assumes MuJoCo's signed-distance convention where
   `pos < 0` means penetration.

   **Definitive sign convention resolution:**

   In MuJoCo, `efc_pos = con->dist` (negative = penetrating). The aref formula:
   ```
   aref = -b*vel - k*imp*(dist - includemargin)
   ```
   When penetrating: `dist < 0`, `includemargin >= 0`, so `dist - includemargin < 0`,
   and `-k*imp*(negative) = +positive` → aref is positive (restorative, pushing apart).

   Our code currently passes `pos = depth` (positive = penetrating):
   ```
   aref = -b*vel - k*imp*(depth - 0.0) = -b*vel - k*imp*depth
   ```
   With `depth > 0`, aref is negative — **this is a sign error** that produces
   destabilizing acceleration (pulling objects together instead of apart).

   **Fix** (line ~15424 in `mujoco_pipeline.rs`):
   ```rust
   // BEFORE:
   let pos = if r == 0 { contact.depth } else { 0.0 };

   // AFTER: negate depth to match MuJoCo's dist convention (negative = penetrating)
   let pos = if r == 0 { -contact.depth } else { 0.0 };
   ```

   With the fix and `margin = includemargin`:
   ```
   aref = -b*vel - k*imp*(-depth - includemargin)
        = -b*vel + k*imp*(depth + includemargin)
   ```
   This produces positive aref (restorative) when `depth > 0` (penetrating) or
   when `depth + includemargin > 0` (within margin zone). Verified against MuJoCo
   source: `mj_referenceConstraint()` in `engine_core_constraint.c`.

   **Note:** The impedance computation at line ~15158 is safe — it already uses
   `.abs()`: `let violation = (pos_val - margin_val).abs();`. With `pos = -depth`
   and `margin = includemargin`: `violation = |(-depth) - includemargin|` =
   `depth + includemargin` (when both positive), matching MuJoCo's absolute
   penetration past the threshold.

   **Cross-check with other constraint types:** Joint limits (line ~15289) and
   tendon limits (line ~15350) already follow MuJoCo's `dist` convention
   (negative = violated). Contacts were the only type passing a positive-penetration
   value. This fix unifies the sign convention across all constraint types.

##### S10. Flex-rigid margin

Flex contacts already have `flex_margin: Vec<f64>` on Model (used in flex-rigid
broadphase at line 5595). Wire this through to narrow-phase:

```rust
let margin = model.flex_margin[flex_id] + model.geom_margin[geom_idx];
```

Add `flex_gap: Vec<f64>` to Model (default 0.0) for gap combination.

##### S11. Global `<option o_margin>` override (deferred)

MuJoCo supports `<option o_margin="...">` which replaces all per-geom margins.
The `mj_assignMargin()` helper checks `mjENABLED(mjENBL_OVERRIDE)` and substitutes
`0.5 * opt.o_margin` when the override flag is set.

**Defer this.** Add a TODO comment at the margin computation site. Implement when
a model requires it. The per-geom margin system is the correct foundation.

#### Acceptance Criteria

1. **Margin activation:** Sphere (`radius=0.05`, `margin=0.002`) above plane
   (`margin=0.002`) at height `0.051` (surface gap = 0.001). Effective margin =
   0.004 > 0.001 gap → contact created. `depth ≈ -0.001`. `includemargin =
   0.004 - 0.0 = 0.004`. `violation = -0.001 + 0.004 = 0.003 > 0` → constraint
   force engages.
2. **No-margin regression:** Same sphere at height `0.051` with `margin=0.0` on
   both geoms. Effective margin = 0.0. Activation: `penetration > -0.0` = 0.0 →
   no contact (surfaces separated). All existing tests pass unchanged.
3. **Gap buffer zone:** Sphere with `margin=0.005, gap=0.003` above plane with
   `margin=0.005, gap=0.002`. Effective margin = 0.01, effective gap = 0.005,
   `includemargin = 0.005`. At distance 0.006: contact created (within margin
   0.01), `violation = -0.006 + 0.005 = -0.001 < 0` → constraint **inactive**
   (in gap buffer). At distance 0.003: `violation = -0.003 + 0.005 = 0.002 > 0`
   → constraint **active**.
4. **Pair override:** Explicit `<contact><pair margin="0.01" gap="0.005">` between
   two geoms that have `margin=0.001, gap=0.0`. Contact uses pair values:
   `includemargin = 0.01 - 0.005 = 0.005`. Geom margins ignored.
5. **efc_margin populated:** After `forward()`, verify `data.efc_margin[row] ==
   contact.includemargin` for every normal-direction constraint row. Friction
   rows have `efc_margin = 0.0`.
6. **Force magnitude scales with violation:** Sphere descending through margin
   zone. At 5 heights from `dist = +0.004` to `dist = -0.002` (with margin=0.005,
   gap=0.0), constraint force increases monotonically. Plot force vs distance
   and verify smooth onset.
7. **Broadphase expansion:** Two geoms at distance 0.09 with `rbound=0.04` each
   and `margin=0.01` each. Without margin: `0.09 > 0.04 + 0.04 = 0.08` → SAP
   candidates. With margin: `0.09 < 0.04 + 0.04 + 0.01 + 0.01 = 0.1` → still
   candidates. Verify contact is detected.
8. **MuJoCo reference comparison:** Load a sphere-on-plane model with
   `margin=0.004, gap=0.001` in both CortenForge and MuJoCo. At 5 separation
   distances spanning `[-0.002, +0.003]`:
   - Contact count matches
   - `efc_margin` matches
   - `qfrc_constraint` matches within `1e-8`
9. **Breaking change audit:** All code that previously used `contact.includemargin`
   as a bool compiles and works correctly with the f64 type.

#### Implementation Risk

**Moderate — broad refactoring scope.** The margin parameter must be threaded
through ~15 narrow-phase functions. Each change is mechanical (add parameter,
change threshold), but the total surface area is large. Risks:

1. **Sign convention:** Our `depth` convention (positive = overlap) is opposite
   to MuJoCo's `dist` convention (negative = overlap). The constraint assembly
   must correctly account for this. Verify with a simple sphere-plane test.
2. **Missing collision function:** If any `collide_*` function is missed, it will
   not activate contacts within the margin zone for that geometry pair type.
   Audit the full list against the table above.
3. **Performance:** Expanding AABBs by margin increases SAP candidate count. For
   small margins (<5mm), this is negligible. For very large margins, it could
   slow broadphase. This is acceptable (matches MuJoCo behavior).
4. **Breaking struct change:** `includemargin: bool → f64` will cause compile
   errors at all construction sites and any conditional checks. This is
   caught at compile time, so risk is low.

**Mitigation:** Default `margin=0.0` and `gap=0.0` produce identical behavior to
current code (`-margin = -0.0 = 0.0`, `includemargin = 0.0`). All existing tests
pass without change. New behavior is only activated when margin/gap are explicitly
set in MJCF.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs`:
  - `Contact` struct — `includemargin: bool → f64`
  - `Contact::new()`, `Contact::with_condim()` — update construction
  - `collide_geoms()` — add `margin: f64` parameter
  - All 14 `collide_*()` functions — add `margin: f64`, change activation check
  - `make_contact_from_geoms()` — add `margin: f64`, set `includemargin`
  - `make_contact_flex_rigid()` — add `margin: f64`, set `includemargin`
  - `apply_pair_overrides()` — set `includemargin = pair.margin - pair.gap`
  - Broadphase SAP loop — compute margin, pass to `collide_geoms()`
  - Broadphase explicit pair loop — pass margin to `collide_geoms()`
  - SAP AABB construction — expand by `geom_margin`
  - `assemble_contact_system()` — replace `margin = 0.0` with
    `margin = contact.includemargin`, fix impedance violation computation
- `sim/L0/mjcf/src/types.rs` — `MjcfGeom.margin`, `MjcfGeom.gap`,
  `MjcfGeomDefaults.margin`, `MjcfGeomDefaults.gap`
- `sim/L0/mjcf/src/parser.rs` — parse `margin`/`gap` from `<geom>`
- `sim/L0/mjcf/src/defaults.rs` — cascade `margin`/`gap`
- `sim/L0/mjcf/src/model_builder.rs` — wire into `Model.geom_margin`/`geom_gap`
- `sim/L0/tests/integration/` — new `contact_margin_gap.rs`

---

### 27B. Flex Child Element Parsing — `<contact>`, `<elasticity>`, `<edge>` Structural Fix
**Status:** Not started | **Effort:** S–M | **Prerequisites:** #27

#### Problem

`parse_flex_attrs()` reads nearly all attributes from the `<flex>` / `<flexcomp>`
element directly. In MuJoCo, most of these belong on **child elements**:

```xml
<flex name="cloth" dim="2" radius="0.005">
  <contact priority="1" solmix="2.0" gap="0.001"
          friction="0.5 0.005 0.0001" condim="4" margin="0.01"
          solref="0.02 1" solimp="0.9 0.95 0.001 0.5 2"
          selfcollide="auto"/>
  <elasticity young="1e6" poisson="0.3" damping="0.01" thickness="0.001"/>
  <edge stiffness="100" damping="0.5"/>
  <vertex pos="..."/>
  <element data="..."/>
</flex>

<flexcomp name="soft" type="grid" count="10 10 1" dim="2" radius="0.005">
  <contact priority="1" solmix="2.0" gap="0.001"/>
  <elasticity young="5e5" poisson="0.2"/>
</flexcomp>
```

**Full mismatch table:**

| Attribute | Our parser reads from | MuJoCo actual location | Impact |
|-----------|----------------------|----------------------|--------|
| `name` | `<flex>` | `<flex>` | Correct |
| `dim` | `<flex>` | `<flex>` | Correct |
| `radius` | `<flex>` | `<flex>` | Correct |
| `young` | `<flex>` | `<flex><elasticity>` | **Wrong** — silently lost on conformant MJCF |
| `poisson` | `<flex>` | `<flex><elasticity>` | **Wrong** — silently lost on conformant MJCF |
| `damping` | `<flex>` | `<flex><elasticity>` | **Wrong** — silently lost on conformant MJCF |
| `thickness` | `<flex>` | `<flex><elasticity>` | **Wrong** — silently lost on conformant MJCF |
| `density` | `<flex>` | Not on `<flex>` at all | **Wrong** — deferred (see below) |
| `friction` | `<flex>` | `<flex><contact>` | **Wrong** — silently lost on conformant MJCF |
| `condim` | `<flex>` | `<flex><contact>` | **Wrong** — silently lost on conformant MJCF |
| `margin` | `<flex>` | `<flex><contact>` | **Wrong** — silently lost on conformant MJCF |
| `solref` | `<flex>` | `<flex><contact>` | **Wrong** — silently lost on conformant MJCF |
| `solimp` | `<flex>` | `<flex><contact>` | **Wrong** — silently lost on conformant MJCF |
| `selfcollide` | `<flex>` as `bool` | `<flex><contact>` as keyword `[none,narrow,bvh,sap,auto]` | **Wrong** — location + type |

Our parser has **three child-element bugs**:

1. **`<flex>` parser** (`parse_flex()`, `parser.rs:2215`): Handles `vertex`,
   `element`, `pin` children, but `<contact>`, `<elasticity>`, and `<edge>`
   all fall to `_ => skip_element()` — silently discarded.

2. **`<flexcomp>` parser** (`parse_flexcomp()`, `parser.rs:2395`): Skips ALL
   child elements with `Ok(_) => {}`. Everything except top-level attrs is lost.

3. **`parse_flex_attrs()`**: Reads contact + elasticity attrs from the top-level
   element. This is non-conformant — these attrs don't exist on `<flex>` or
   `<flexcomp>` in MuJoCo. Only works if the user writes non-standard MJCF.

Additionally, items #24–#27 added `flex_priority`, `flex_solmix`, and `flex_gap`
Model fields wired into `contact_param_flex_rigid()`, but these are always
hardcoded to defaults because parsing doesn't exist.

#### MuJoCo Authoritative Semantics

##### Direct `<flex>` attributes (correct in our parser)

`name`, `group`, `dim`, `radius`, `material`, `rgba`, `flatskin`, `body`, `node`.

Also has data arrays: `vertex`, `element`, `texcoord`, `elemtexcoord` — these
are structural data handled by `parse_flex()`, not configuration attrs.

Of these, our `MjcfFlex` supports: `name`, `dim`, `radius`. The rest are out
of scope for #27B: `group`, `material`, `rgba`, `flatskin` are visual/
organizational; `body` and `node` are physics-relevant (vertex-to-body and
vertex-to-DOF mapping) but are a pre-existing parsing gap unrelated to
child element structure. Our `model_builder` derives body/DOF mapping from
the enclosing `<body>` hierarchy rather than the `body`/`node` attributes.

##### `<flex><contact>` / `<flexcomp><contact>` attributes

| Attribute | Type | Default | MjcfFlex field | Model field | Status |
|-----------|------|---------|---------------|-------------|--------|
| `priority` | int | 0 | **New** | `flex_priority` | Parse in #27B |
| `solmix` | real | 1 | **New** | `flex_solmix` | Parse in #27B |
| `gap` | real | 0 | **New** | `flex_gap` | Parse in #27B |
| `friction` | real(3) | "1 0.005 0.0001" | `friction: f64` (scalar) | `flex_friction` | Parse in #27B (1st component) |
| `condim` | int | 3 | `condim` | `flex_condim` | Relocate in #27B |
| `margin` | real | 0 | `margin` | `flex_margin` | Relocate in #27B |
| `solref` | real(2) | "0.02 1" | `solref` | `flex_solref` | Relocate in #27B |
| `solimp` | real(5) | "0.9 0.95 0.001 0.5 2" | `solimp` | `flex_solimp` | Relocate in #27B |
| `selfcollide` | keyword `[none,narrow,bvh,sap,auto]` | "auto" | `selfcollide: bool` -> **`Option<String>`** | `flex_selfcollide` | Fix type + relocate |
| `contype` | int | 1 | — | — | Deferred (no runtime) |
| `conaffinity` | int | 1 | — | — | Deferred (no runtime) |
| `internal` | bool | false | — | — | Deferred |
| `activelayers` | int | 1 | — | — | Deferred |
| `vertcollide` | bool | false | — | — | Deferred |
| `passive` | bool | false | — | — | Deferred |

**Note on `selfcollide`:** MuJoCo's `selfcollide` is a keyword
`[none, narrow, bvh, sap, auto]` controlling the self-collision broadphase
algorithm, not a boolean. `"auto"` (the default) means MuJoCo picks;
`"narrow"` uses pair-wise narrowphase only; `"bvh"` uses bounding volume
hierarchy; `"sap"` uses sweep-and-prune; `"none"` disables self-collision.
Our runtime currently checks `if model.flex_selfcollide[i]` as a simple
enable/disable. Fix: change `MjcfFlex.selfcollide` from `bool` to
`Option<String>`. `None` = absent (uses default "auto"); `Some(kw)` = explicit
keyword. Model field stays `Vec<bool>` — builder maps
`flex.selfcollide.as_deref() != Some("none")` to the bool until algorithm-specific
support is added. Note: all keywords except `"none"` enable self-collision.

**Note on `friction`:** MuJoCo's `friction` is `real(3)` — sliding, torsional,
rolling. Our `MjcfFlex.friction` is `f64` (scalar = sliding only). The parser
must handle multi-component input by taking the first value to avoid silent
`parse::<f64>()` failure on `"1 0.005 0.0001"`. The upgrade from `f64` to
`Vector3<f64>` is a pre-existing gap (noted in #24 spec) and out of scope.

**Note on deferred attrs:** `contype`, `conaffinity`, `internal`,
`activelayers`, `vertcollide`, `passive` require runtime support (flex collision
filtering, layer-based self-collision, passive force flags) that doesn't exist.
Parsing without runtime wiring would be misleading. Add when runtime is ready.

##### `<flex><elasticity>` / `<flexcomp><elasticity>` attributes

| Attribute | Type | Default | MjcfFlex field | Model field | Status |
|-----------|------|---------|---------------|-------------|--------|
| `young` | real | 0 | `young` | `flex_young` | Relocate in #27B |
| `poisson` | real | 0 | `poisson` | `flex_poisson` | Relocate in #27B |
| `damping` | real | 0 | `damping` | `flex_damping` | Relocate in #27B |
| `thickness` | real | -1 (sentinel: "not set") | `thickness` | `flex_thickness` | Relocate in #27B |
| `elastic2d` | keyword `[none,bend,stretch,both]` | "none" | — | — | Deferred (not implemented) |

**Note on default changes:** Our `MjcfFlex::default()` currently uses `young: 1e6`
and `thickness: 0.001`. MuJoCo defaults are `young: 0` and `thickness: -1`.
This is a **breaking change** — models relying on our non-conformant defaults
will need to add explicit `<elasticity young="1e6" thickness="0.001"/>`. This
is the correct fix per CLAUDE.md: "Prefer breaking changes that fix the
architecture over non-breaking hacks that preserve a bad interface."

**Note on `selfcollide` default change:** Our `MjcfFlex::default()` currently
uses `selfcollide: false`. MuJoCo default is `"auto"` (enabled). After this
change, models without explicit `selfcollide` attr will have self-collision
enabled (matching MuJoCo). This is also a breaking change but is correct.

##### `<flex><edge>` attributes

| Attribute | Type | Default | MjcfFlex field | Model field | Status |
|-----------|------|---------|---------------|-------------|--------|
| `stiffness` | real | 0 | **New** | **New** (`flex_edgestiffness`) | Parse + store + wire in #27B |
| `damping` | real | 0 | **New** | **New** (`flex_edgedamping`) | Parse + store + wire in #27B |

##### `<flexcomp><edge>` additional attributes

`<flexcomp><edge>` has the same `stiffness`/`damping` plus these extra attrs
that control edge equality constraints generated during `<flexcomp>` expansion:

| Attribute | Type | Default | MjcfFlex field | Model field | Status |
|-----------|------|---------|---------------|-------------|--------|
| `equality` | keyword `[none,true,vert]` | "none" | — | — | Deferred (flexcomp expansion) |
| `solref` | real(2) | "0.02 1" | — | — | Deferred (flexcomp expansion) |
| `solimp` | real(5) | "0.9 0.95 0.001 0.5 2" | — | — | Deferred (flexcomp expansion) |

**Note on `<flexcomp><edge>` extras:** In MuJoCo, `<flexcomp>` is a procedural
generator that creates a `<flex>` plus associated equality constraints for edge
enforcement. The `equality`/`solref`/`solimp` on `<flexcomp><edge>` control
these generated equality constraints. `equality` is a keyword: `"none"` (no
constraint, the default), `"true"` (per-edge equality constraints), or `"vert"`
(averaged vertex constraints). Since our `<flexcomp>` expansion doesn't yet
generate equality constraints, these attrs are deferred. They do NOT appear
on `<flex><edge>` — parsed flex bodies never have these attributes.

**Note on MuJoCo architecture:** `<edge stiffness>` and `<edge damping>` are
**passive spring-damper coefficients**, NOT constraint solver parameters. In
MuJoCo (`engine_passive.c`), these drive direct forces:
```
frc_spring = stiffness * (rest_length - current_length)
frc_damper = -damping * edge_velocity
```
These accumulate into `qfrc_spring` / `qfrc_damper` (our `qfrc_passive`),
applied *before* the constraint solver. They are entirely separate from the
constraint-based `flex_edge_solref` which comes from the equality constraint
path (`mjEQ_FLEX`, using `eq_solref`).

**Note on `stiffness` scope:** MuJoCo docs state that `<edge stiffness>` is
"Only for 1D flex" (cables/strands). For 2D/3D flex bodies, edge stiffness
comes from the FEM elasticity model (Young's modulus + Poisson ratio via
`<elasticity>`) or from plugins. We parse and store the value for all dims
(matching MuJoCo's parser behavior — it accepts the attr regardless), but the
passive force path in #27C should document that this is physically meaningful
only for `dim=1` flex bodies.

Our current `compute_edge_solref_from_material()` passthrough stub conflates
these two mechanisms. #27C will address the runtime wiring — for now, #27B
parses and stores the values and adds Model fields so the passive force path
can use them.

##### `density` on `MjcfFlex`

MuJoCo has no `density` attribute on `<flex>` or its children. Our
`MjcfFlex.density` field and its parsing from top-level are non-conformant.
In MuJoCo, flex mass is set via `<flexcomp mass="...">` (total mass) or
computed from material properties. **Out of scope for #27B** — affects mass
computation logic, not element structure. Remove the top-level `density` parse
from `parse_flex_attrs()` but keep the field for internal use by `<flexcomp>`
procedural generation.

#### Implementation Plan

##### S1. Update `MjcfFlex` struct

Add new fields to `MjcfFlex` (`types.rs:3182`):

```rust
/// Contact priority (default 0). Higher priority geom's params win.
pub priority: i32,
/// Solver parameter mixing weight (default 1.0).
pub solmix: f64,
/// Contact gap — buffer zone within margin (default 0.0).
pub gap: f64,
/// Self-collision broadphase mode. MuJoCo keyword: [none, narrow, bvh, sap, auto].
/// None = absent (default "auto"); Some("none") = disabled; other = enabled.
pub selfcollide: Option<String>,
/// Passive edge spring stiffness (from <edge> child). Default 0.0 = disabled.
/// Used for direct spring-damper forces in the passive force path.
pub edge_stiffness: f64,
/// Passive edge damping coefficient (from <edge> child). Default 0.0 = disabled.
pub edge_damping: f64,
```

Change existing field:
```rust
// BEFORE:
pub selfcollide: bool,
// AFTER:
// Replaced by Option<String> above — remove the bool field
```

Update `MjcfFlex::default()` — add new fields and fix existing defaults to
match MuJoCo:
```rust
// New fields:
priority: 0,
solmix: 1.0,
gap: 0.0,
selfcollide: None,    // was: selfcollide: false
edge_stiffness: 0.0,
edge_damping: 0.0,
// Fix existing defaults to match MuJoCo XML spec:
young: 0.0,          // was: 1e6 — MuJoCo default is 0
thickness: -1.0,     // was: 0.001 — MuJoCo default is -1 (sentinel: "not set")
```

##### S2. Extract child element parsing helpers

Create three helpers in `parser.rs`:

```rust
/// Parse `<contact>` child element attributes into MjcfFlex.
fn parse_flex_contact_attrs(e: &BytesStart, flex: &mut MjcfFlex) {
    if let Some(s) = get_attribute_opt(e, "priority") {
        flex.priority = s.parse().unwrap_or(0);
    }
    if let Some(s) = get_attribute_opt(e, "solmix") {
        flex.solmix = s.parse().unwrap_or(1.0);
    }
    if let Some(s) = get_attribute_opt(e, "gap") {
        flex.gap = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "friction") {
        // MuJoCo friction is real(3): "slide torsion rolling".
        // MjcfFlex.friction is f64 (scalar = sliding component only).
        // Parse first whitespace-separated value to handle both "0.5" and
        // "0.5 0.005 0.0001" without silent parse::<f64>() failure.
        flex.friction = s.split_whitespace()
            .next()
            .and_then(|t| t.parse().ok())
            .unwrap_or(1.0);
    }
    if let Some(s) = get_attribute_opt(e, "condim") {
        flex.condim = s.parse().unwrap_or(3);
    }
    if let Some(s) = get_attribute_opt(e, "margin") {
        flex.margin = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "solref") {
        let vals: Vec<f64> = s.split_whitespace()
            .filter_map(|t| t.parse().ok()).collect();
        if vals.len() >= 2 { flex.solref = [vals[0], vals[1]]; }
    }
    if let Some(s) = get_attribute_opt(e, "solimp") {
        let vals: Vec<f64> = s.split_whitespace()
            .filter_map(|t| t.parse().ok()).collect();
        if vals.len() >= 5 {
            flex.solimp = [vals[0], vals[1], vals[2], vals[3], vals[4]];
        }
    }
    if let Some(s) = get_attribute_opt(e, "selfcollide") {
        flex.selfcollide = Some(s);
    }
}

/// Parse `<elasticity>` child element attributes into MjcfFlex.
fn parse_flex_elasticity_attrs(e: &BytesStart, flex: &mut MjcfFlex) {
    if let Some(s) = get_attribute_opt(e, "young") {
        flex.young = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "poisson") {
        flex.poisson = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "damping") {
        flex.damping = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "thickness") {
        flex.thickness = s.parse().unwrap_or(-1.0);
    }
}

/// Parse `<edge>` child element attributes into MjcfFlex.
/// These are passive spring-damper coefficients (not constraint params).
fn parse_flex_edge_attrs(e: &BytesStart, flex: &mut MjcfFlex) {
    if let Some(s) = get_attribute_opt(e, "stiffness") {
        flex.edge_stiffness = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "damping") {
        flex.edge_damping = s.parse().unwrap_or(0.0);
    }
}
```

##### S3. Wire child elements in `parse_flex()`

In `parse_flex()` (`parser.rs:2215`), add arms for `contact`, `elasticity`,
and `edge` in both `Event::Start` and `Event::Empty` match branches:

```rust
b"contact" => {
    parse_flex_contact_attrs(e, &mut flex);
    // For Event::Start, consume closing tag:
    skip_element(reader, &elem_name)?;
}
b"elasticity" => {
    parse_flex_elasticity_attrs(e, &mut flex);
    skip_element(reader, &elem_name)?;
}
b"edge" => {
    parse_flex_edge_attrs(e, &mut flex);
    skip_element(reader, &elem_name)?;
}
```

##### S4. Wire child elements in `parse_flexcomp()`

In `parse_flexcomp()` (`parser.rs:2410-2422`), replace the blind `Ok(_) => {}`
loop with proper child element dispatch:

```rust
loop {
    match reader.read_event_into(&mut buf) {
        Ok(Event::Start(ref e)) => {
            let elem_name = e.name().as_ref().to_vec();
            match elem_name.as_slice() {
                b"contact" => {
                    parse_flex_contact_attrs(e, &mut flex);
                    skip_element(reader, &elem_name)?;
                }
                b"elasticity" => {
                    parse_flex_elasticity_attrs(e, &mut flex);
                    skip_element(reader, &elem_name)?;
                }
                b"edge" => {
                    parse_flex_edge_attrs(e, &mut flex);
                    skip_element(reader, &elem_name)?;
                }
                b"pin" => {
                    if let Some(s) = get_attribute_opt(e, "id") {
                        let ids: Vec<usize> = s.split_whitespace()
                            .filter_map(|t| t.parse().ok()).collect();
                        flex.pinned.extend(ids);
                    }
                    skip_element(reader, &elem_name)?;
                }
                _ => { skip_element(reader, &elem_name)?; }
            }
        }
        Ok(Event::Empty(ref e)) => {
            let elem_name = e.name().as_ref().to_vec();
            match elem_name.as_slice() {
                b"contact" => parse_flex_contact_attrs(e, &mut flex),
                b"elasticity" => parse_flex_elasticity_attrs(e, &mut flex),
                b"edge" => parse_flex_edge_attrs(e, &mut flex),
                b"pin" => {
                    if let Some(s) = get_attribute_opt(e, "id") {
                        let ids: Vec<usize> = s.split_whitespace()
                            .filter_map(|t| t.parse().ok()).collect();
                        flex.pinned.extend(ids);
                    }
                }
                _ => {}
            }
        }
        Ok(Event::End(ref e)) if e.name().as_ref() == b"flexcomp" => break,
        Ok(Event::Eof) => {
            return Err(MjcfError::XmlParse("unexpected EOF in flexcomp".into()));
        }
        Ok(_) => {}
        Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
    }
    buf.clear();
}
```

**Note:** Also adds `<pin>` handling to `parse_flexcomp()` — currently skipped.

**Note on `parse_flexcomp_empty()`:** Self-closing `<flexcomp ... />` elements
(`parser.rs:2431`) call `parse_flex_attrs()` but can't have children. After S5
strips `parse_flex_attrs()`, a self-closing flexcomp correctly gets default
values for all contact/elasticity/edge params. No changes needed — just be aware
this path exists.

##### S5. Clean up `parse_flex_attrs()`

Remove all attrs that belong on child elements:

**Remove** (lines 2340-2387):
- `young`, `poisson`, `damping`, `thickness` -> now on `<elasticity>`
- `friction`, `condim`, `margin`, `solref`, `solimp`, `selfcollide` -> now on `<contact>`
- `density` -> not a MuJoCo `<flex>` attr (keep field, remove parse)

**Keep** (lines 2334-2338, 2364-2366):
- `name` — direct `<flex>` attr
- `dim` — direct `<flex>` attr
- `radius` — direct `<flex>` attr

After cleanup, `parse_flex_attrs()` becomes:

```rust
fn parse_flex_attrs(e: &BytesStart) -> MjcfFlex {
    let mut flex = MjcfFlex::default();
    if let Some(s) = get_attribute_opt(e, "name") {
        flex.name = s;
    }
    if let Some(s) = get_attribute_opt(e, "dim") {
        flex.dim = s.parse().unwrap_or(2);
    }
    if let Some(s) = get_attribute_opt(e, "radius") {
        flex.radius = s.parse().unwrap_or(0.005);
    }
    flex
}
```

No existing tests use top-level flex contact or elasticity attrs (verified via
grep). This is safe.

##### S6. Update `selfcollide` throughout

Change `MjcfFlex.selfcollide` from `bool` to `Option<String>`. Update
references:

- `model_builder.rs`:
  `self.flex_selfcollide.push(flex.selfcollide.as_deref() != Some("none"))`
  When absent (None), defaults to `true` (MuJoCo default is `"auto"` = enabled).
  `Some("none")` maps to `false`. All other keywords map to `true`.
  (Model field stays `Vec<bool>` — runtime semantics unchanged until algorithm
  support is added)
- No runtime changes — `model.flex_selfcollide[i]` remains `bool` on Model.

##### S7. Wire new fields into model builder

In `process_flex()` (`model_builder.rs:3002-3004`), replace hardcoded defaults
and add new pushes:

```rust
// BEFORE:
self.flex_gap.push(0.0);
self.flex_priority.push(0);
self.flex_solmix.push(1.0);

// AFTER:
self.flex_gap.push(flex.gap);
self.flex_priority.push(flex.priority);
self.flex_solmix.push(flex.solmix);
```

Add new fields for edge stiffness/damping in three places:
```rust
// 1. ModelBuilder struct (model_builder.rs, after flex_selfcollide):
flex_edgestiffness: Vec<f64>,
flex_edgedamping: Vec<f64>,

// 2. ModelBuilder init (model_builder.rs, in new()):
flex_edgestiffness: vec![],
flex_edgedamping: vec![],

// 3. Model struct (mujoco_pipeline.rs, after flex_selfcollide):
pub flex_edgestiffness: Vec<f64>,  // passive edge spring stiffness
pub flex_edgedamping: Vec<f64>,    // passive edge damping coefficient

// 4. Model default init (mujoco_pipeline.rs):
flex_edgestiffness: vec![],
flex_edgedamping: vec![],

// 5. In process_flex():
self.flex_edgestiffness.push(flex.edge_stiffness);
self.flex_edgedamping.push(flex.edge_damping);

// 6. In build() transfer:
flex_edgestiffness: self.flex_edgestiffness,
flex_edgedamping: self.flex_edgedamping,
```

##### S8. No runtime changes needed for contact path

The runtime contact code (`contact_param_flex_rigid()`, `make_contact_flex_rigid()`,
constraint assembly) already reads Model fields — the contact parameter wiring
is complete once S7 lands.

The new `flex_edgestiffness` / `flex_edgedamping` Model fields are stored but
not yet consumed by the passive force path. Wiring them into the passive force
computation (spring-damper forces on edges, analogous to MuJoCo's
`engine_passive.c`) is deferred to #27C.

#### Acceptance Criteria

1. **`<flex><contact>` round-trip:** `<flex dim="2"><contact priority="2"
   solmix="3.0" gap="0.01" friction="0.5" condim="4" margin="0.02"
   selfcollide="bvh"/><vertex pos="0 0 0 1 0 0 0 1 0"/><element
   data="0 1 2"/></flex>` produces `model.flex_priority[i] == 2`,
   `model.flex_solmix[i] == 3.0`, `model.flex_gap[i] == 0.01`,
   `model.flex_friction[i] == 0.5`, `model.flex_condim[i] == 4`,
   `model.flex_margin[i] == 0.02`, `model.flex_selfcollide[i] == true`.
2. **`<flex><elasticity>` round-trip:** `<flex dim="2"><elasticity young="5e5"
   poisson="0.3" damping="0.01" thickness="0.002"/><vertex pos="0 0 0 1 0 0
   0 1 0"/><element data="0 1 2"/></flex>` produces `model.flex_young[i] ==
   5e5`, `model.flex_poisson[i] == 0.3`, `model.flex_damping[i] == 0.01`,
   `model.flex_thickness[i] == 0.002`.
3. **`<flex><edge>` round-trip:** `<flex dim="2"><edge stiffness="100"
   damping="0.5"/><vertex pos="0 0 0 1 0 0 0 1 0"/><element data="0 1 2"/>
   </flex>` produces `model.flex_edgestiffness[i] == 100.0`,
   `model.flex_edgedamping[i] == 0.5`.
4. **`<flexcomp><contact>` round-trip:** `<flexcomp type="grid" dim="2"
   count="3 3 1"><contact priority="1" gap="0.005"/></flexcomp>` produces
   `model.flex_priority[i] == 1`, `model.flex_gap[i] == 0.005`.
5. **`<flexcomp><elasticity>` round-trip:** `<flexcomp type="grid" dim="2"
   count="3 3 1"><elasticity young="2e5"/></flexcomp>` produces
   `model.flex_young[i] == 2e5`.
6. **Multi-component friction:** `<flex dim="2"><contact friction="0.8 0.005
   0.0001"/><vertex pos="0 0 0 1 0 0 0 1 0"/><element data="0 1 2"/></flex>`
   parses `model.flex_friction[i] == 0.8` (first component, sliding).
   Single-value `friction="0.5"` also works.
7. **Default preservation:** Omitting all child elements produces defaults:
   `priority=0`, `solmix=1.0`, `gap=0.0`, `friction=1.0`, `condim=3`,
   `margin=0.0`, `solref=[0.02,1]`, `solimp=[0.9,0.95,0.001,0.5,2]`,
   `selfcollide=true` (MuJoCo default "auto" = enabled), `young=0`,
   `poisson=0.0`, `damping=0.0`, `thickness=-1`, `edge_stiffness=0.0`,
   `edge_damping=0.0`.
8. **Top-level attrs no longer parsed:** `<flex young="5e5" friction="0.5">`
   without child elements ignores both attributes (conformant with MuJoCo).
9. **`selfcollide` keyword:** `<flex dim="2"><contact selfcollide="auto"/>
   <vertex pos="0 0 0 1 0 0 0 1 0"/><element data="0 1 2"/></flex>` stores
   `flex.selfcollide == Some("auto")` and `model.flex_selfcollide[i] == true`.
   `selfcollide="none"` produces `flex.selfcollide == Some("none")` and
   `model.flex_selfcollide[i] == false`. Omitting `selfcollide` produces
   `flex.selfcollide == None` and `model.flex_selfcollide[i] == true` (MuJoCo
   default is `"auto"` = enabled).
10. **All existing tests pass** — no regressions. (Verified: no existing tests
    use top-level flex contact or elasticity attrs.)

#### Files

- `sim/L0/mjcf/src/types.rs` — add 5 fields to `MjcfFlex` (`priority`,
  `solmix`, `gap`, `edge_stiffness`, `edge_damping`), change `selfcollide`
  from `bool` to `Option<String>`
- `sim/L0/mjcf/src/parser.rs` — three child-element helpers
  (`parse_flex_contact_attrs`, `parse_flex_elasticity_attrs`,
  `parse_flex_edge_attrs`), wire into `parse_flex()` + `parse_flexcomp()`,
  strip `parse_flex_attrs()` to `name`/`dim`/`radius` only
- `sim/L0/mjcf/src/model_builder.rs` — wire `flex.gap/priority/solmix`,
  update `selfcollide` push to `flex.selfcollide.as_deref() != Some("none")`, push
  `edge_stiffness`/`edge_damping` to new Model fields
- `sim/L0/core/src/mujoco_pipeline.rs` — add `flex_edgestiffness: Vec<f64>`
  and `flex_edgedamping: Vec<f64>` to Model struct

---

### 27C. Passive Edge Spring-Damper Forces + `compute_edge_solref_from_material()` Cleanup
**Status:** Not started | **Effort:** S–M | **Prerequisites:** #27B

#### Problem

MuJoCo's flex edge system uses **two separate mechanisms** for edge-length
enforcement (verified against `engine_passive.c` and `engine_core_constraint.c`):

1. **Passive spring-damper forces** (`engine_passive.c`): Controlled by
   `flex_edgestiffness[f]` and `flex_edgedamping[f]` (from `<edge stiffness="..."
   damping="..."/>`). Applied as direct forces to `qfrc_spring` / `qfrc_damper`
   before the constraint solver:
   ```
   frc_spring = stiffness * (rest_length - current_length)
   frc_damper = -damping * edge_velocity
   ```

2. **Equality constraints** (`engine_core_constraint.c`): Flex edge equality
   (`mjEQ_FLEX`) creates constraint rows per non-rigid edge. Solver parameters
   come from `eq_solref` / `eq_solimp` on the parent equality constraint, NOT
   from material-derived values.

Our codebase conflates these two mechanisms:
- `compute_edge_solref_from_material()` is a passthrough stub that was supposed
  to derive `flex_edge_solref` from Young's modulus. But MuJoCo does NOT derive
  edge constraint solref from material properties — the constraint solref comes
  from the equality constraint definition.
- After #27B, we have `model.flex_edgestiffness` and `model.flex_edgedamping`
  stored but not consumed by any passive force computation.

#### What this task does

1. **Wire passive edge spring-damper forces** into the Newton penalty path
   and/or passive force computation:
   - If `flex_edgestiffness[flex_id] > 0` or `flex_edgedamping[flex_id] > 0`,
     apply spring-damper forces to each non-rigid edge
   - Force accumulates into `qfrc_passive` (our analog of MuJoCo's
     `qfrc_spring` + `qfrc_damper`)

2. **Clean up `compute_edge_solref_from_material()`**: Either remove the stub
   entirely (if edge constraints should just use `flex.solref` directly, which
   is the current behavior and appears to match MuJoCo's equality constraint
   path), or document clearly that it's an intentional passthrough, not a stub
   awaiting implementation.

3. **Document the dual-mechanism architecture** in a code comment so future
   contributors understand the distinction between passive forces (stiffness/
   damping) and constraint-based edge enforcement (solref/solimp).

#### Reference

- **MuJoCo `engine_passive.c`**: Passive spring-damper force loop over flex
  edges — uses `flex_edgestiffness`, `flex_edgedamping`, `flexedge_length0`,
  `flexedge_length`, `flexedge_velocity`, sparse Jacobian `flexedge_J`
- **MuJoCo `engine_core_constraint.c`**: Flex edge equality constraints
  (`mjEQ_FLEX`) — uses `eq_solref`/`eq_solimp`, not material-derived values
- **Prior documentation:** `future_work_6b_precursor_to_7.md` S8 (line 138)
  — note that the "material-derived per-edge solref" framing was incorrect;
  MuJoCo uses a separate passive force mechanism instead
- **MuJoCo `engine_setconst.c`**: `makeFlexSparse()` — skips Jacobian
  computation when `!flex_edgeequality && !flex_edgedamping && !flex_edgestiffness`

#### Implementation Plan

##### S1. Add passive edge spring-damper forces in `mj_fwd_passive()`

Add a new section in `mj_fwd_passive()` (`mujoco_pipeline.rs:11569`), after the
flex vertex damping loop and before the flex bending passive forces. This matches
MuJoCo's `engine_passive.c` ordering.

```rust
// Flex edge passive spring-damper forces.
// MuJoCo architecture: <edge stiffness="..." damping="..."/> drives passive
// forces (engine_passive.c), separate from constraint-based edge enforcement
// (mjEQ_FLEX in engine_core_constraint.c which uses eq_solref/eq_solimp).
for e in 0..model.nflexedge {
    let flex_id = model.flexedge_flexid[e];
    let stiffness = model.flex_edgestiffness[flex_id];
    let damping = model.flex_edgedamping[flex_id];

    if stiffness == 0.0 && damping == 0.0 {
        continue;
    }

    let [v0, v1] = model.flexedge_vert[e];

    // Skip edges where both vertices are pinned (rigid edge)
    if model.flexvert_invmass[v0] == 0.0 && model.flexvert_invmass[v1] == 0.0 {
        continue;
    }

    let x0 = data.flexvert_xpos[v0];
    let x1 = data.flexvert_xpos[v1];
    let diff = x1 - x0;
    let dist = diff.norm();
    if dist < 1e-10 {
        continue;
    }

    let direction = diff / dist;
    let rest_len = model.flexedge_length0[e];

    // Spring force: stiffness * (rest_length - current_length)
    // Positive when compressed (restoring), negative when stretched.
    let frc_spring = stiffness * (rest_len - dist);

    // Damping force: -damping * edge_velocity
    // edge_velocity = d(dist)/dt = (v1 - v0) · direction
    let dof0 = model.flexvert_dofadr[v0];
    let dof1 = model.flexvert_dofadr[v1];
    let vel0 = Vector3::new(data.qvel[dof0], data.qvel[dof0 + 1], data.qvel[dof0 + 2]);
    let vel1 = Vector3::new(data.qvel[dof1], data.qvel[dof1 + 1], data.qvel[dof1 + 2]);
    let edge_velocity = (vel1 - vel0).dot(&direction);
    let frc_damper = -damping * edge_velocity;

    let force_mag = frc_spring + frc_damper;

    // Apply via J^T: edge Jacobian is ±direction for the two endpoint DOFs.
    // F_v0 = -direction * force_mag (pulls v0 toward v1 when stretched)
    // F_v1 = +direction * force_mag (pulls v1 toward v0 when stretched)
    if model.flexvert_invmass[v0] > 0.0 {
        for ax in 0..3 {
            data.qfrc_passive[dof0 + ax] -= direction[ax] * force_mag;
        }
    }
    if model.flexvert_invmass[v1] > 0.0 {
        for ax in 0..3 {
            data.qfrc_passive[dof1 + ax] += direction[ax] * force_mag;
        }
    }
}
```

**Design notes:**
- No stability clamp needed (unlike bending): edge spring forces are bounded
  by `stiffness * rest_len` which is a sane quantity for reasonable stiffness
  values. MuJoCo's `engine_passive.c` doesn't clamp edge forces either.
- No `flexedge_velocity` Data field needed: we compute the scalar edge velocity
  inline from `qvel` and the edge direction vector, same pattern as the Newton
  penalty path (line 14835).
- Uses `qfrc_passive` directly. MuJoCo separates into `qfrc_spring` and
  `qfrc_damper` but our architecture combines them into `qfrc_passive`.
- **1D scope note:** MuJoCo docs say `<edge stiffness>` is "Only for 1D flex"
  (cables). For 2D/3D, elasticity comes from FEM via `<elasticity>`. The code
  applies to all dims (matching MuJoCo's runtime — it doesn't gate on dim), but
  users should only set nonzero `<edge stiffness>` for `dim=1` flex bodies.

**Known architectural differences vs MuJoCo `engine_passive.c`:**

These are deliberate simplifications that match our existing flex infrastructure:

1. **`has_spring`/`has_damping` global flags:** MuJoCo multiplies stiffness
   by `has_spring` (from `mjDSBL_SPRING`) and damping by `has_damping` (from
   `mjDSBL_DAMPER`). These are **separate** disable flags — you can disable
   springs while keeping dampers active. Our codebase has no `disableflags`-
   based passive force gating — `mj_fwd_passive()` always runs. Same gap
   exists for all our passive forces (bending, vertex damping, etc.). Tracked
   as part of #41 (`disableflags`).

2. **`flex_rigid[f]` / `flexedge_rigid[e]`:** MuJoCo skips entire rigid flex
   bodies and individual rigid edges via dedicated boolean arrays. We don't have
   these fields — we use `flexvert_invmass[v] == 0.0` per-vertex instead. This
   is semantically equivalent for edges (a rigid edge has both endpoints with
   `invmass == 0`), but less efficient (checks per-edge vs one check per-flex).
   Not a correctness issue.

3. **Sparse Jacobian (`flexedge_J`) vs inline `±direction`:** MuJoCo uses a
   pre-computed sparse edge Jacobian `flexedge_J` for `J^T * force`, which
   handles the general case where flex vertices may be attached to complex
   multi-DOF bodies. Our code computes the Jacobian inline as `±direction`
   applied to `flexvert_dofadr`, which assumes each vertex maps to 3 consecutive
   translational DOFs. **This matches our existing architecture** — the bending
   passive force loop (`mj_fwd_passive()` line 11642-11689) and the Newton
   penalty path (line 14827-14854) both use the same `flexvert_dofadr` + 3-DOF
   inline pattern. Adding sparse Jacobian support is a larger infrastructure
   change that would affect all flex force paths, not just edge stiffness.

4. **`flexedge_length` / `flexedge_velocity` pre-computation:** MuJoCo reads
   pre-computed `d->flexedge_length[e]` and `d->flexedge_velocity[e]`. We
   compute both inline from `flexvert_xpos` and `qvel`. Same results, slightly
   less efficient but avoids adding Data fields. Matches the pattern used by
   our Newton penalty path (line 14811-14835).

##### S2. Clean up `compute_edge_solref_from_material()`

Replace the stub with a clear passthrough that documents the architecture:

```rust
/// Compute per-flex edge constraint solref.
///
/// MuJoCo architecture note: Edge constraint solref comes from the equality
/// constraint definition (eq_solref), not from material properties. The
/// <edge stiffness="..." damping="..."/> attributes control passive spring-
/// damper forces (applied in mj_fwd_passive), not constraint solver params.
///
/// Our current architecture uses flex-level solref for all edge constraints,
/// which matches MuJoCo's behavior when no explicit equality constraint
/// overrides are defined.
fn compute_edge_solref(flex: &MjcfFlex) -> [f64; 2] {
    flex.solref
}
```

Changes:
- Rename from `compute_edge_solref_from_material()` to `compute_edge_solref()`
  — removes the misleading "from_material" suffix
- Replace the dead-code guard (`if flex.young <= 0.0 || ...`) with a direct
  return — the function is an intentional passthrough, not a stub
- Update the call site at `model_builder.rs:2990`
- Update the doc comment on `Model.flex_edge_solref` (`mujoco_pipeline.rs:1420`)
  from "derived from young/poisson/damping" to "per-flex solref passthrough"

##### S3. Update documentation comment on `flex_edge_solref`

In `mujoco_pipeline.rs:1420`:
```rust
// BEFORE:
/// Per-flex: edge constraint solref (derived from young/poisson/damping).
pub flex_edge_solref: Vec<[f64; 2]>,

// AFTER:
/// Per-flex: edge constraint solref. Uses flex-level solref (from
/// <contact solref="..."/>). MuJoCo derives this from eq_solref on the
/// parent equality constraint; our architecture uses flex.solref as the
/// default, which matches MuJoCo when no explicit equality override exists.
pub flex_edge_solref: Vec<[f64; 2]>,
```

##### S4. Update `future_work_6b_precursor_to_7.md` S8 reference

Add a correction note at `future_work_6b_precursor_to_7.md:775-793` marking
the "material-derived per-edge solref" framing as superseded:

```
**Correction (from #27C):** The original framing of material-derived per-edge
solref was incorrect. MuJoCo does not derive edge constraint solref from
Young's modulus. Instead: (1) edge constraints use eq_solref from the parent
equality constraint, and (2) <edge stiffness/damping> drives passive spring-
damper forces in engine_passive.c. See future_work_7.md #27C for details.
```

#### Acceptance Criteria

1. **Edge spring force applied:** A flex with `<edge stiffness="100"/>` and one
   edge stretched to 1.1× rest length produces a nonzero `qfrc_passive`
   contribution on both edge endpoint DOFs. Force magnitude matches:
   `100.0 * (rest_len - 1.1 * rest_len) = -100.0 * 0.1 * rest_len`.
2. **Edge damping force applied:** A flex with `<edge damping="10"/>` and
   nonzero vertex velocities along an edge produces a velocity-dependent
   `qfrc_passive` contribution.
3. **Zero stiffness/damping skip:** A flex with default `<edge/>` (both 0.0) or
   no `<edge>` child produces no passive edge forces — `qfrc_passive` unchanged
   by the edge loop.
4. **Pinned vertex skip:** An edge where both vertices are pinned
   (`invmass == 0.0`) is skipped. An edge where one vertex is pinned applies
   force only to the non-pinned vertex.
5. **Rename compiles:** `compute_edge_solref_from_material` no longer exists.
   `compute_edge_solref` used at `model_builder.rs:2990`. No other references.
6. **Doc comment updated:** `Model.flex_edge_solref` comment no longer
   references Young's modulus or material derivation.
7. **6b correction note added:** `future_work_6b_precursor_to_7.md` S8 section
   contains correction note referencing #27C.
8. **All existing tests pass** — no regressions. The passive force is additive
   to `qfrc_passive` and does not affect the constraint path.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — passive edge spring-damper loop in
  `mj_fwd_passive()`, Model field doc comment update
- `sim/L0/mjcf/src/model_builder.rs` — rename
  `compute_edge_solref_from_material()` → `compute_edge_solref()`, simplify body
- `sim/docs/todo/future_work_6b_precursor_to_7.md` — correction note on S8

---

### 27D. Flex `body` / `node` Attribute Parsing + `flexvert_bodyid` Wiring
**Status:** Not started | **Effort:** S–M | **Prerequisites:** #27B

#### Discovery Context

Discovered during #27B/#27C spec review (measure-twice pass). MuJoCo's `<flex>`
element has two direct attributes that our parser completely ignores:

- `body` — `string(nvert)`: space-separated list of body names, one per vertex.
  Determines which rigid body each flex vertex is attached to. Controls DOF
  mapping: vertices attached to a body inherit that body's DOFs instead of
  getting their own 3 translational DOFs.
- `node` — `string(nnode)`, optional: names of bodies to use as flex nodes
  (alternative to `<vertex>` for attaching flex vertices to existing bodies).

#### Current State

**Parser (`parser.rs:2331-2389`):** `parse_flex_attrs()` currently parses 14
attributes (name, dim, young, poisson, damping, thickness, density, friction,
condim, margin, radius, solref, solimp, selfcollide) but does NOT parse `body`
or `node`. Both are silently ignored. **Note:** After #27B, `parse_flex_attrs()`
will be stripped to only `name`/`dim`/`radius` — the contact/elasticity attrs
move to child element helpers. The `body`/`node`/`group` additions in this spec
apply to the post-#27B version of `parse_flex_attrs()`.

**Model builder (`model_builder.rs:3177`):** `flexvert_bodyid` is hardcoded:
```rust
flexvert_bodyid: vec![usize::MAX; self.nflexvert],  // Free vertices: no body attachment
```
All flex vertices are treated as free (unattached to any rigid body), regardless
of what the MJCF `body` attribute specifies. `usize::MAX` is the sentinel for
"no body attachment".

**MjcfFlex struct (`types.rs:3182-3217`):** No `body` or `node` fields exist.

**Impact:** Any MJCF model that uses `<flex body="body1 body2 ...">` to attach
flex vertices to rigid bodies will silently ignore the attachment. All vertices
get independent translational DOFs instead of inheriting body DOFs. This produces
wrong physics for body-attached flex models (e.g., cloth attached to a rigid
frame, cables tethered to moving bodies).

#### MuJoCo Authoritative Semantics

- **S1 — `body` attribute:** `string(nvert)`, **required** on `<flex>`.
  Space-separated list of body names, one per vertex. Length must exactly equal
  the number of vertices. Each name references a body in the kinematic tree.
  MuJoCo resolves this during compilation to populate `m->flexvert_bodyid[v]`
  (integer body ID per vertex). **Key insight:** In MuJoCo, there are no truly
  "free" flex vertices — every vertex is attached to a body. What our
  architecture treats as "free 3-DOF vertices" corresponds to `<flexcomp>`
  auto-generating a per-vertex body (each with its own DOFs). When multiple
  vertices share the same body name, they share that body's DOFs and move
  rigidly with it. When each vertex has a unique body, each gets independent
  DOFs through its body.

- **S2 — `node` attribute:** `string(nnode)`, optional. Names of bodies to serve
  as flex nodes. Alternative to explicit `<vertex>` positions — vertex positions
  are taken from the named bodies' positions at compile time. Also sets
  `flexvert_bodyid` for each node vertex. Mutually exclusive with `<vertex>`
  child element / `vertex` direct attribute.

- **S3 — `group` attribute:** `int`, default `"0"`. Visualization group (0-5).
  Out of scope for physics but should be parsed for completeness.

- **S4 — DOF mapping consequences:** When `flexvert_bodyid[v] != -1` (body-
  attached), the vertex's DOF address (`flexvert_dofadr[v]`) points to that
  body's DOFs instead of independent translational DOFs. This means:
  - The vertex count does NOT contribute to `nv` (total DOFs)
  - Jacobians for forces on that vertex use the body's full Jacobian
  - `flexvert_invmass[v]` reflects the body's inertia, not vertex mass
  - Our current inline `±direction` force application (3 consecutive translational
    DOFs at `flexvert_dofadr[v]`) would be WRONG for body-attached vertices

- **S5 — Interaction with sparse Jacobian (§42A-i):** Body-attached vertices
  are the primary reason MuJoCo uses the sparse `flexedge_J` Jacobian instead
  of inline `±direction`. When a vertex is attached to a multi-DOF body (e.g.,
  a free body with 6 DOFs), forces must be projected through the full body
  Jacobian. This is tracked as a separate runtime gap (§42A-i).

#### Specification

1. **Add fields to `MjcfFlex`** (`types.rs`):
   ```rust
   /// Body names for vertex attachment (one per vertex). Required on <flex>.
   /// For <flexcomp>, auto-generated during expansion (one body per vertex).
   pub body: Vec<String>,
   /// Node body names (alternative to <vertex> positions).
   pub node: Vec<String>,
   /// Visualization group (0-5).
   pub group: i32,
   ```

2. **Parse in `parse_flex_attrs()`** (`parser.rs`):
   ```rust
   if let Some(s) = get_attribute_opt(e, "body") {
       flex.body = s.split_whitespace().map(|t| t.to_string()).collect();
   }
   if let Some(s) = get_attribute_opt(e, "node") {
       flex.node = s.split_whitespace().map(|t| t.to_string()).collect();
   }
   if let Some(s) = get_attribute_opt(e, "group") {
       flex.group = s.parse().unwrap_or(0);
   }
   ```

3. **Wire `flexvert_bodyid` in model builder** (`model_builder.rs`):
   - For `<flex>`: `body` is required — resolve each body name to a body ID
     using the existing body name → ID mapping. Populate `flexvert_bodyid[v]`
     with the resolved ID. Validate: `flex.body.len()` must equal vertex count.
   - For `<flexcomp>`: `body` is auto-generated during expansion — each vertex
     gets its own auto-generated body name (e.g., `"flexname_0"`). Our current
     `process_flex_bodies()` creates free 3-DOF vertices, which is functionally
     equivalent to per-vertex bodies with 3 translational DOFs. The `flexcomp`
     path should populate `body` during expansion so the model builder handles
     both paths uniformly.
   - Handle `node` attribute: resolve body names, extract positions as vertex
     positions, set `flexvert_bodyid` accordingly.

4. **DOF allocation adjustment** (for shared-body vertices):
   - When multiple vertices reference the same body, they share that body's
     DOFs — each such vertex does NOT allocate new DOFs
   - `flexvert_dofadr[v]` should point to the body's DOF start
   - This requires changes to the flex DOF allocation in `process_flex_bodies()`
   - For `<flexcomp>` with per-vertex bodies (the common case), every vertex
     has a unique body, so each still gets independent DOFs. The DOF-sharing
     only matters when a user writes `<flex body="same same same ...">`
   - **Note:** Full correctness for bodies with non-trivial DOFs (e.g., free
     joint = 6 DOFs) also requires sparse Jacobian support (§42A-i). Without
     §42A-i, body-attached vertices will have correct DOF mapping but forces
     will still use the simplified `±direction` inline Jacobian.

#### Acceptance Criteria

1. `<flex body="body1 body2 body3" ...>` correctly resolves body names to IDs
   in `flexvert_bodyid`.
2. Vertices sharing the same body name share DOFs — no duplicate DOF allocation.
3. `flexvert_dofadr` for shared-body vertices points to the body's DOF start.
4. `<flex node="bodyA bodyB" ...>` resolves node body names, extracts positions,
   and wires `flexvert_bodyid`.
5. `group` attribute parsed and stored (even if not used for physics).
6. Validation error if `body` length doesn't match vertex count on `<flex>`.
7. `<flexcomp>` path still works — auto-generated per-vertex bodies produce
   independent DOFs per vertex (same behavior as current architecture).
8. All existing flex tests pass (existing tests use `<flexcomp>` which is
   unaffected by the `body` attr parsing).

#### Files

- `sim/L0/mjcf/src/types.rs` — add `body`, `node`, `group` fields to `MjcfFlex`
- `sim/L0/mjcf/src/parser.rs` — parse `body`, `node`, `group` in `parse_flex_attrs()`
- `sim/L0/mjcf/src/model_builder.rs` — resolve body names → IDs, wire
  `flexvert_bodyid`, adjust DOF allocation for body-attached vertices
