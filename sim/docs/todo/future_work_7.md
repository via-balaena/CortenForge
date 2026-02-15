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

### 24. Friction Combination Rule: Geometric Mean → Element-Wise Max
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

`make_contact_from_geoms()` (`mujoco_pipeline.rs:5690`) combines friction
coefficients from two contacting geoms using **geometric mean**:

```rust
let sliding = (f1.x * f2.x).sqrt();
let torsional = (f1.y * f2.y).sqrt();
let rolling = (f1.z * f2.z).sqrt();
```

The same pattern is used for deformable-rigid contacts (`mujoco_pipeline.rs:19230`).

MuJoCo uses **element-wise maximum** (when geom priorities are equal):

```c
// engine_collision_driver.c — mj_contactParam()
for (int i=0; i < 3; i++) {
    fri[i] = mju_max(friction1[i], friction2[i]);
}
```

This divergence was documented as "known non-conformance" in `future_work_2.md`
(Decision D3) and explicitly deferred: "better done as a deliberate conformance
task with before/after validation, not buried in a condim refactor."

**Impact:** Every contact between geoms with asymmetric friction values produces
wrong friction coefficients. This affects every trajectory comparison test.

#### Objective

Switch friction combination from geometric mean to element-wise max, matching
MuJoCo's `mj_contactParam()` behavior for equal-priority geoms.

#### Specification

1. **`make_contact_from_geoms()`** (`mujoco_pipeline.rs:5690`): Change three lines:
   ```rust
   let sliding = f1.x.max(f2.x);
   let torsional = f1.y.max(f2.y);
   let rolling = f1.z.max(f2.z);
   ```

2. **Deformable-rigid contacts** (`mujoco_pipeline.rs:19230`): Same change pattern.
   Deformable material has a single `friction: f64` combined with rigid geom's
   per-axis friction:
   ```rust
   let sliding = deform_friction.max(rigid_friction.x);
   let torsional = deform_friction.max(rigid_friction.y);
   let rolling = deform_friction.max(rigid_friction.z);
   ```

3. **`<contact><pair>` override**: When a `<pair>` element specifies explicit
   `friction`, it overrides the combination rule entirely — no change needed here.

4. **Geom priority**: MuJoCo uses `geom/@priority` to select which geom's
   friction wins outright (higher priority geom's friction used directly). Our
   parser does not read `priority`. This is out of scope — `priority` is item
   #25. For equal-priority geoms (the only case we handle), element-wise max is
   correct.

5. **Doc correction**: Update any documentation that claims geometric mean
   "matching MuJoCo's combination rule" — this is factually wrong. State
   element-wise max.

#### Acceptance Criteria

1. `make_contact_from_geoms()` uses `f1.max(f2)` per component.
2. Deformable-rigid contact friction uses `max()` combination.
3. Existing contact tests updated — any test that relied on geometric mean
   behavior must be re-validated. Expected: tolerance tightening, not loosening.
4. New test: two geoms with `friction="0.3"` and `friction="0.7"` produce
   contact with `sliding = 0.7` (not `sqrt(0.21) ≈ 0.458`).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `make_contact_from_geoms()`, deformable contacts
- `sim/L0/tests/integration/` — contact friction tests

---

### 25. `geom/@priority` — Contact Priority
**Status:** Not started | **Effort:** S | **Prerequisites:** None (soft dep: after #24)

#### Current State

Not parsed, not stored, not used. Zero references in codebase. Contact parameter
combination currently always uses element-wise max for friction and element-wise
min/max for solref/solimp, regardless of priority.

#### Objective

Parse `priority` from `<geom>`, store on Model, and use it in contact parameter
combination to select the higher-priority geom's parameters outright when
priorities differ.

#### Specification

1. **MJCF parsing**: Parse `priority` (int, default 0) from `<geom>`. Also parse
   from `<default>` class.
2. **Model storage**: Add `geom_priority: Vec<i32>` to `Model`.
3. **Contact parameter combination** (in `make_contact_from_geoms()` or equivalent):
   - If `priority[g1] > priority[g2]`: use g1's friction, solref, solimp, solmix.
   - If `priority[g1] < priority[g2]`: use g2's friction, solref, solimp, solmix.
   - If `priority[g1] == priority[g2]`: use the existing combination rule
     (element-wise max for friction — or weighted average after #26 solmix).
4. **Interaction with #24 (friction combination)**: Priority check happens BEFORE
   the combination rule. If priorities differ, no combination is needed. This is
   why #25 has a soft dependency on #24 — the combination rule it falls back to
   should be the correct MuJoCo rule, not the current approximation.

#### Acceptance Criteria

1. Geom with `priority="1"` vs geom with `priority="0"`: the priority-1 geom's
   friction/solver params are used verbatim.
2. Equal priority geoms use the existing combination rule.
3. Default priority (0) matches MuJoCo behavior.
4. `<contact><pair>` explicit pairs respect priority overrides (if specified).

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse priority
- `sim/L0/core/src/mujoco_pipeline.rs` — Model field, contact parameter selection

---

### 26. `solmix` Attribute
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Not parsed. Current implementation uses element-wise min for solref and
element-wise max for solimp as an approximation. Documented as known divergence
in `future_work_2.md`.

#### Objective

Parse `solmix` from `<geom>` and use it for weighted averaging of solver
parameters in contact pairs.

#### Specification

1. **MJCF parsing**: Parse `solmix` (float, default 1.0) from `<geom>`. Also
   parse from `<default>` class.
2. **Model storage**: Add `geom_solmix: Vec<f64>` to `Model`.
3. **Contact parameter combination** (MuJoCo's `mj_contactParam` logic):
   - Compute mixing weight: `w1 = solmix[g1] / (solmix[g1] + solmix[g2])`,
     `w2 = 1 - w1`.
   - `solref_combined = w1 * solref[g1] + w2 * solref[g2]` (weighted average).
   - `solimp_combined = w1 * solimp[g1] + w2 * solimp[g2]` (weighted average).
   - Friction: NOT affected by solmix — friction uses element-wise max (#24) or
     priority override (#25).
4. **Edge case**: If both `solmix` values are 0, use equal weights (0.5/0.5).
5. **Interaction with #25 (priority)**: If priorities differ, the higher-priority
   geom's parameters are used directly — solmix is not consulted. Solmix only
   applies when priorities are equal.

#### Acceptance Criteria

1. Two geoms with `solmix="1"` (default) produce equal-weighted combination.
2. Geom with `solmix="2"` vs `solmix="1"`: first geom has 2/3 weight.
3. Solmix=0 edge case handled without NaN/inf.
4. `<contact><pair>` can override solref/solimp directly (bypass solmix).
5. Existing contact tests pass with default solmix=1 (regression).

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse solmix
- `sim/L0/core/src/mujoco_pipeline.rs` — Model field, weighted combination in
  contact parameter assembly

---

### 27. Contact Margin/Gap Runtime Effect
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

`geom_margin` and `geom_gap` are stored on `Model` as `Vec<f64>` (one per geom,
initialized to 0.0). `ContactPair` has `margin` and `gap` fields, correctly parsed
from `<contact><pair>` MJCF elements. However, margin and gap have **no runtime
effect** on contact activation or constraint assembly:

- **Narrow-phase**: All collision functions (e.g., `collide_sphere_sphere`) generate
  contacts only when `penetration > 0.0`. Margin is not checked.
  (`mujoco_pipeline.rs`, `collide_sphere_sphere`, line ~6509.)
- **Constraint assembly**: `margin` is hardcoded to `0.0` in `assemble_contact_system`
  (`mujoco_pipeline.rs`, line ~14600: `let margin = 0.0_f64;`). The `compute_aref()`
  function accepts a `margin` parameter but always receives 0.0.
- **`apply_pair_overrides`**: Applies condim, friction, solref, solimp from
  `ContactPair` but skips margin/gap. Comment at line ~5664: "margin/gap are NOT
  applied here."
- **Broad-phase**: `pair.margin` IS used in the bounding-sphere distance cull for
  explicit pairs (line ~5431). This is correct and should be preserved.
- **Geom-level parsing**: `<geom margin="..." gap="...">` attributes are NOT parsed.
  `model_builder.rs` comment: "geom-level not yet parsed, default to 0.0."
- **`efc_margin`**: The `Data` struct has an `efc_margin: Vec<f64>` field (per
  constraint row), currently always populated with 0.0.

MuJoCo's margin/gap system involves three distinct locations: contact detection
(wider activation), constraint violation computation (shifted reference), and the
`aref` stabilization formula. All three must be wired up.

#### Objective

Wire `margin` and `gap` into the full pipeline — MJCF parsing, contact detection,
and constraint assembly — so that contacts activate at the correct distance and
Baumgarte stabilization uses the correct reference point.

#### Specification

##### S1. Margin/gap combination rule (MuJoCo semantics)

MuJoCo **sums** margins and gaps from both geoms (source:
`engine_collision_driver.c`, `mj_collideGeoms`):

```
effective_margin = geom_margin[g1] + geom_margin[g2]
effective_gap    = geom_gap[g1]    + geom_gap[g2]
includemargin    = effective_margin - effective_gap
```

For explicit `<contact><pair>` contacts, the pair's `margin` and `gap` attributes
**override** the geom-summed values entirely (pair takes precedence).

The `includemargin` value is the distance threshold below which a contact generates
a constraint. It is stored on the `Contact` struct and propagated to `efc_margin`
during constraint assembly.

##### S2. Geom-level MJCF parsing

Parse `margin` and `gap` attributes from `<geom>` elements. These cascade through
the defaults system (existing infrastructure handles attribute cascading):

```xml
<geom type="sphere" size="0.1" margin="0.002" gap="0.001"/>
```

Default values: `margin = 0.0`, `gap = 0.0` (preserves current behavior when
attributes are absent).

Update `Model.geom_margin` and `Model.geom_gap` arrays, which are already
allocated (currently zeroed). Wire through `model_builder.rs` → `process_geom()`.

##### S3. Narrow-phase contact activation

Change the activation condition in all narrow-phase collision functions from:

```rust
if penetration > 0.0 { ... }  // current
```

to:

```rust
if penetration > -margin { ... }  // with margin
```

where `margin` is the effective margin computed per S1. When a contact is generated
with `penetration ∈ (-margin, 0]`, the contact's `depth` field stores the
actual geometric penetration (which is negative — surfaces are separated). The
`includemargin` field on the Contact stores the margin value.

This means the contact exists in the constraint system but may produce zero force
(if `depth > includemargin`, the constraint violation `r > 0` so no corrective
force is applied). This is MuJoCo's mechanism for "pre-contact" — smooth force
onset rather than sudden activation.

Affected functions: `collide_sphere_sphere`, `collide_sphere_plane`,
`collide_capsule_plane`, `collide_box_plane`, `collide_sphere_capsule`,
and all other `collide_*` variants. Each function must receive the effective
margin for the geom pair.

Implementation approach: pass `margin` as a parameter to each `collide_*`
function. Compute `margin` in the dispatch function (`collide_geoms` or explicit
pair loop) using S1, and thread it through.

##### S4. Contact struct: store `includemargin`

The `Contact` struct already has a `pub includemargin: bool` field (currently
always `false`). Replace this with a numeric field:

```rust
/// Distance threshold used for constraint activation.
/// `includemargin = effective_margin - effective_gap`.
/// Constraint violation: `r = depth - includemargin`.
/// Contact exists when `depth > -effective_margin` (see S3).
pub includemargin: f64,
```

Set by `make_contact_from_geoms()` or `apply_pair_overrides()`.

##### S5. `apply_pair_overrides`: apply margin/gap from ContactPair

In `apply_pair_overrides()`, set `contact.includemargin = pair.margin - pair.gap`
when the contact comes from an explicit `<contact><pair>`. Remove the existing
comment "margin/gap are NOT applied here."

For automatic (Mechanism 1) contacts, set
`contact.includemargin = (geom_margin[g1] + geom_margin[g2]) - (geom_gap[g1] + geom_gap[g2])`
in `make_contact_from_geoms()`.

##### S6. Constraint assembly: populate `efc_margin`

In `assemble_contact_system` (line ~14598), replace the hardcoded
`let margin = 0.0_f64` with:

```rust
let margin = contact.includemargin;
```

This value flows into:
1. `data.efc_margin[row] = margin` — per constraint row (normal direction only;
   friction rows get `efc_margin = 0.0`, matching MuJoCo).
2. `compute_aref(k, b, imp, pos, margin, vel)` — the existing function already
   computes `aref = -b * vel - k * imp * (pos - margin)`. With nonzero margin,
   the constraint equilibrium shifts to `pos = margin` instead of `pos = 0`.

No changes to `compute_aref()` itself — the function is already correct; it just
needs a nonzero margin input.

##### S7. Interaction with global `<option>` override

MuJoCo supports `<option o_margin="...">` which overrides all per-geom and
per-pair margins globally. This is a low-priority extension — defer unless existing
models require it. Note in code with a TODO.

#### Acceptance Criteria

1. **Margin activation**: A sphere at height 1mm above a plane with
   `geom_margin="0.002"` (each geom) generates a contact. Effective margin =
   `0.002 + 0.002 = 0.004 > 0.001` (distance). Contact `depth ≈ -0.001`
   (negative, surfaces separated), `includemargin = 0.004`, constraint
   `r = -0.001 - 0.004 = -0.005 < 0` → constraint force engages.
2. **Gap buffer**: With `margin="0.004"` and `gap="0.003"`, `includemargin = 0.001`.
   A sphere at distance 0.002 above the plane generates a contact (within margin)
   but `r = -0.002 - 0.001 = -0.003 < 0` → force engages. At distance 0.0005,
   still in contact, stronger force. Verify force magnitude scales with
   `|r|` (deeper violation → stronger force).
3. **Pair override**: An explicit `<contact><pair margin="0.01" gap="0.005">`
   uses `includemargin = 0.005`, ignoring geom-level margins.
4. **Zero margin/gap regression**: All existing tests pass unchanged (default
   margin/gap = 0.0 preserves `penetration > 0.0` activation and
   `efc_margin = 0.0` in assembly).
5. **MuJoCo reference**: Compare contact count and forces against MuJoCo 3.4.0
   for a sphere-on-plane test at 5 separation distances spanning
   `[-0.002, +0.003]` with `margin=0.004, gap=0.001`. Contact count and
   `qfrc_constraint` must match within tolerance `1e-8`.
6. **`efc_margin` populated**: After `forward()`, `data.efc_margin[normal_row]`
   equals `contact.includemargin` for every active contact.

#### Implementation Notes

**Narrow-phase function signature change.** Every `collide_*` function currently
takes `(model, geom1, geom2, pos1, pos2, size1, size2) -> Option<Contact>`. Add
a `margin: f64` parameter. The dispatch function computes the effective margin
(S1) before calling the specific collider.

**Existing broad-phase is correct.** The bounding-sphere cull at line ~5431
already adds `pair.margin` to the distance check. For automatic contacts, add
`geom_margin[g1] + geom_margin[g2]` to the broad-phase distance threshold.

**Breaking change to Contact struct.** Replacing `includemargin: bool` with
`includemargin: f64` is a breaking change. Any existing code that checks
`contact.includemargin` as a bool must be updated. Search for all uses.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — all `collide_*` functions,
  `make_contact_from_geoms()`, `apply_pair_overrides()`,
  `assemble_contact_system()` (margin wiring), broad-phase distance check
- `sim/L0/mjcf/src/parser.rs` — parse `margin`/`gap` from `<geom>` elements
- `sim/L0/mjcf/src/model_builder.rs` — wire geom-level margin/gap into
  `Model.geom_margin`/`Model.geom_gap` in `process_geom()`
- `sim/L0/mjcf/src/defaults.rs` — ensure margin/gap cascade through defaults
- `sim/L0/tests/integration/` — new test file `contact_margin_gap.rs`
