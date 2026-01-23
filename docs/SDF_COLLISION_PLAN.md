# SDF Collision Implementation Plan

Complete implementation of all SDF (Signed Distance Field) collision combinations for comprehensive physics simulation support.

## Current Status

| Combination | Status | Priority | Complexity |
|-------------|--------|----------|------------|
| Sdf ↔ Sphere | ✅ Done | - | - |
| Sdf ↔ Capsule | ✅ Done | - | - |
| Sdf ↔ Box | ✅ Done | - | - |
| Sdf ↔ Cylinder | ✅ Done | High | Low |
| Sdf ↔ Ellipsoid | ✅ Done | High | Low |
| Sdf ↔ ConvexMesh | ✅ Done | High | Medium |
| Sdf ↔ Plane | ✅ Done | Low | Low |
| Sdf ↔ TriangleMesh | ✅ Done | Medium | Medium |
| Sdf ↔ HeightField | ✅ Done | Medium | Medium |
| Sdf ↔ Sdf | ✅ Done | Medium | High |

---

## Implementation Strategy

### General Approach

SDFs provide signed distance at any point via trilinear interpolation. The existing implementations use **point sampling**:
- **Sphere**: Query SDF at sphere center, contact if `distance < radius`
- **Capsule**: Sample 5 points along axis, find deepest penetration
- **Box**: Test all 8 corners against SDF

This point-sampling approach extends naturally to other shapes.

### Key Files

- `sim-core/src/sdf.rs` - SDF data structure and contact functions
- `sim-core/src/world.rs` - Collision dispatch (add new match arms around line 1835)

---

## Milestone 1: Cylinder & Ellipsoid (Point Sampling)

**Priority:** High - Common shapes in robotics (joints, links)

### 1.1 Sdf ↔ Cylinder

**Approach:** Sample points on cylinder surface (caps + lateral surface)

```rust
/// Query an SDF for contact with a cylinder.
///
/// Samples points on cylinder caps and around the lateral surface.
pub fn sdf_cylinder_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    cylinder_pose: &Pose,
    half_height: f64,
    radius: f64,
) -> Option<SdfContact>
```

**Sampling strategy:**
- 2 cap centers (top/bottom)
- 8 points around each cap edge (16 total)
- 8 points around cylinder middle circumference
- Total: 26 sample points

**Implementation steps:**
1. Add `sdf_cylinder_contact()` to `sdf.rs`
2. Add dispatch arms in `world.rs` for `(Sdf, Cylinder)` and `(Cylinder, Sdf)`
3. Add unit tests

### 1.2 Sdf ↔ Ellipsoid

**Approach:** Sample points on ellipsoid surface using spherical coordinates

```rust
/// Query an SDF for contact with an ellipsoid.
///
/// Samples points on ellipsoid surface at multiple latitudes/longitudes.
pub fn sdf_ellipsoid_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    ellipsoid_pose: &Pose,
    radii: &Vector3<f64>,
) -> Option<SdfContact>
```

**Sampling strategy:**
- 6 axis-aligned points (±x, ±y, ±z scaled by radii)
- 8 diagonal points (corners of inscribed cube, projected to surface)
- 12 edge midpoints
- Total: 26 sample points (or configurable for accuracy)

**Implementation steps:**
1. Add `sdf_ellipsoid_contact()` to `sdf.rs`
2. Add dispatch arms in `world.rs`
3. Add unit tests

---

## Milestone 2: ConvexMesh (Vertex Sampling)

**Priority:** High - Essential for complex robot parts

### 2.1 Sdf ↔ ConvexMesh

**Approach:** Test all vertices of the convex mesh against SDF

```rust
/// Query an SDF for contact with a convex mesh.
///
/// Tests all mesh vertices against the SDF, returning deepest penetration.
pub fn sdf_convex_mesh_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    mesh_pose: &Pose,
    vertices: &[Point3<f64>],
) -> Option<SdfContact>
```

**Implementation notes:**
- Similar to box contact but with arbitrary vertex count
- For large vertex counts, could add early-out based on AABB overlap
- Vertex positions are in mesh local space, need transform to world then to SDF local

**Implementation steps:**
1. Add `sdf_convex_mesh_contact()` to `sdf.rs`
2. Add dispatch arms in `world.rs`
3. Add unit tests with various convex shapes

---

## Milestone 3: Plane (Grid Sampling)

**Priority:** Low - Rare use case but completeness matters

### 3.1 Sdf ↔ Plane

**Approach:** Sample SDF grid points against infinite plane

```rust
/// Query an SDF for contact with an infinite plane.
///
/// Samples SDF boundary points and interior points against plane.
pub fn sdf_plane_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    plane_normal: &Vector3<f64>,
    plane_offset: f64,
) -> Option<SdfContact>
```

**Implementation notes:**
- Plane is infinite, so we check if SDF intersects plane
- Sample SDF grid points, find those below plane
- Use SDF gradient at contact point for normal (or plane normal)
- Could optimize by only checking SDF surface region (where |distance| < threshold)

**Implementation steps:**
1. Add `sdf_plane_contact()` to `sdf.rs`
2. Add dispatch arms in `world.rs`
3. Add unit tests

---

## Milestone 4: TriangleMesh (Vertex + Edge Sampling)

**Priority:** Medium - Complex geometry interactions

### 4.1 Sdf ↔ TriangleMesh

**Approach:** Sample mesh vertices and edge midpoints against SDF

```rust
/// Query an SDF for contact with a triangle mesh.
///
/// Samples mesh vertices and optionally edge midpoints against the SDF.
pub fn sdf_triangle_mesh_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    mesh: &TriangleMeshData,
    mesh_pose: &Pose,
) -> Option<SdfContact>
```

**Implementation notes:**
- Use BVH for broad phase (check if mesh AABB overlaps SDF AABB)
- Sample all unique vertices against SDF
- Optionally sample edge midpoints for better accuracy
- For very large meshes, could use BVH to only test triangles near SDF surface

**Sampling strategy:**
- All mesh vertices (from BVH or direct access)
- Edge midpoints for triangles that might intersect (optional, configurable)

**Implementation steps:**
1. Add `sdf_triangle_mesh_contact()` to `sdf.rs`
2. Add helper to extract unique vertices from `TriangleMeshData`
3. Add dispatch arms in `world.rs`
4. Add unit tests

---

## Milestone 5: HeightField (Grid Point Sampling)

**Priority:** Medium - Terrain interaction

### 5.1 Sdf ↔ HeightField

**Approach:** Sample SDF against height field grid points in overlap region

```rust
/// Query an SDF for contact with a height field.
///
/// Finds intersection between SDF and height field surfaces.
pub fn sdf_heightfield_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    heightfield: &HeightFieldData,
    heightfield_pose: &Pose,
) -> Option<SdfContact>
```

**Implementation notes:**
- Compute AABB overlap between SDF and height field
- For each height field cell in overlap region:
  - Compute height field surface point
  - Query SDF distance at that point
  - If penetrating, record contact
- Return deepest contact

**Implementation steps:**
1. Add `sdf_heightfield_contact()` to `sdf.rs`
2. Add dispatch arms in `world.rs`
3. Add unit tests

---

## Milestone 6: Sdf ↔ Sdf (Dual Sampling)

**Priority:** Medium - Complex but powerful for soft body interaction

**Complexity:** High - Two implicit surfaces

### 6.1 Sdf ↔ Sdf

**Approach:** Sample grid points of each SDF against the other

```rust
/// Query two SDFs for contact.
///
/// Samples surface points of each SDF against the other.
pub fn sdf_sdf_contact(
    sdf_a: &SdfCollisionData,
    pose_a: &Pose,
    sdf_b: &SdfCollisionData,
    pose_b: &Pose,
) -> Option<SdfContact>
```

**Implementation notes:**
- Compute AABB overlap between both SDFs
- Strategy 1: Sample SDF-A surface points (where |dist| < threshold) against SDF-B
- Strategy 2: March along overlapping region looking for sign changes
- Could use marching cubes-like approach for surface extraction

**Simpler approach:**
1. Find AABB overlap region
2. Create sample grid in overlap region
3. For each sample point:
   - Query both SDFs
   - If both negative (inside both surfaces), we have penetration
   - Penetration depth = min(|dist_a|, |dist_b|)
   - Normal from gradient of SDF with smaller absolute distance

**Implementation steps:**
1. Add `sdf_sdf_contact()` to `sdf.rs`
2. Add dispatch arm in `world.rs`
3. Add unit tests with overlapping SDF spheres/boxes

---

## Testing Strategy

### Unit Tests (per milestone)

Each contact function should have tests for:
1. **No collision** - Shapes clearly separated
2. **Shallow penetration** - Just touching
3. **Deep penetration** - Significant overlap
4. **Transformed poses** - Non-identity rotations and translations
5. **Edge cases** - Degenerate shapes, boundary conditions

### Integration Tests

- Full simulation with SDF objects colliding with all shape types
- Performance benchmarks for large-scale scenarios

### Test Shapes

```rust
// Test SDFs
let sphere_sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 0.5);
let box_sdf = SdfCollisionData::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);

// Test primitives
let cylinder = CollisionShape::cylinder(0.5, 0.3); // half_height, radius
let ellipsoid = CollisionShape::ellipsoid(Vector3::new(0.5, 0.3, 0.4));
// etc.
```

---

## Performance Considerations

### Sample Count Trade-offs

| Shape | Samples | Accuracy | Performance |
|-------|---------|----------|-------------|
| Cylinder | 26 | Good | Fast |
| Ellipsoid | 26 | Good | Fast |
| ConvexMesh | N vertices | Exact for vertices | O(N) |
| TriangleMesh | N vertices + edges | Good | O(N) |
| HeightField | Grid cells in overlap | Good | O(overlap area) |
| Sdf-Sdf | Sample grid | Approximate | O(overlap volume) |

### Optimization Opportunities

1. **AABB early-out**: Skip detailed checks if bounding boxes don't overlap
2. **Adaptive sampling**: More samples near surface, fewer far away
3. **Caching**: Cache transformed vertices for multi-contact scenarios
4. **SIMD**: Batch SDF queries for multiple points

---

## Implementation Order

Recommended order based on priority and dependencies:

1. **Milestone 1**: Cylinder & Ellipsoid (2-3 hours)
   - High value, straightforward extension of existing patterns

2. **Milestone 2**: ConvexMesh (1-2 hours)
   - Similar to box but arbitrary vertices

3. **Milestone 3**: Plane (1 hour)
   - Simple, low priority but quick

4. **Milestone 4**: TriangleMesh (2-3 hours)
   - Needs vertex extraction, BVH integration

5. **Milestone 5**: HeightField (2-3 hours)
   - Grid-based sampling in overlap region

6. **Milestone 6**: Sdf-Sdf (3-4 hours)
   - Most complex, dual implicit surface intersection

**Total estimated effort:** 12-16 hours

---

## Checklist

### Milestone 1: Cylinder & Ellipsoid
- [x] Implement `sdf_cylinder_contact()` in `sdf.rs`
- [x] Add Sdf-Cylinder dispatch in `world.rs`
- [x] Add Cylinder-Sdf dispatch (flipped) in `world.rs`
- [x] Unit tests for cylinder contact
- [x] Implement `sdf_ellipsoid_contact()` in `sdf.rs`
- [x] Add Sdf-Ellipsoid dispatch in `world.rs`
- [x] Add Ellipsoid-Sdf dispatch (flipped) in `world.rs`
- [x] Unit tests for ellipsoid contact

### Milestone 2: ConvexMesh
- [x] Implement `sdf_convex_mesh_contact()` in `sdf.rs`
- [x] Add Sdf-ConvexMesh dispatch in `world.rs`
- [x] Add ConvexMesh-Sdf dispatch (flipped) in `world.rs`
- [x] Unit tests for convex mesh contact

### Milestone 3: Plane
- [x] Implement `sdf_plane_contact()` in `sdf.rs`
- [x] Add Sdf-Plane dispatch in `world.rs`
- [x] Add Plane-Sdf dispatch (flipped) in `world.rs`
- [x] Unit tests for plane contact

### Milestone 4: TriangleMesh
- [x] Add vertex extraction helper to `TriangleMeshData`
- [x] Implement `sdf_triangle_mesh_contact()` in `sdf.rs`
- [x] Add Sdf-TriangleMesh dispatch in `world.rs`
- [x] Add TriangleMesh-Sdf dispatch (flipped) in `world.rs`
- [x] Unit tests for triangle mesh contact

### Milestone 5: HeightField
- [x] Implement `sdf_heightfield_contact()` in `sdf.rs`
- [x] Add Sdf-HeightField dispatch in `world.rs`
- [x] Add HeightField-Sdf dispatch (flipped) in `world.rs`
- [x] Unit tests for height field contact

### Milestone 6: Sdf-Sdf
- [x] Implement `sdf_sdf_contact()` in `sdf.rs`
- [x] Add Sdf-Sdf dispatch in `world.rs`
- [x] Unit tests for SDF-SDF contact

### Final
- [ ] Integration tests
- [ ] Performance benchmarks
- [x] Update documentation
- [x] Update FEATURE_IMPLEMENTATION_CHECKLIST.md
