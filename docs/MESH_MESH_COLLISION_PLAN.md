# Mesh-Mesh Collision Implementation Plan

**Goal:** Enable non-convex triangle mesh vs triangle mesh collision detection for humanoid robot simulation.

**Priority:** HIGH - Required for realistic humanoid robot interactions (self-collision, robot-robot, complex environment interaction).

---

## Current State

### What's Implemented
- `TriangleMesh` collision shape variant in `sim-core/src/world.rs`
- `TriangleMeshData` with vertices, triangles, AABB, and BVH (`sim-core/src/mesh.rs`)
- Triangle-primitive collisions: sphere, capsule, box
- BVH (Bounding Volume Hierarchy) for acceleration
- MJCF mesh loading (files + embedded data)

### What's Missing
- `TriangleMesh ↔ TriangleMesh` collision returns `None` (falls through in dispatch)
- ~~No triangle-triangle intersection algorithm~~ ✅ **COMPLETED** (Milestone 1)
- No dual-BVH traversal for mesh pairs

---

## Implementation Approach

### Option A: Triangle-Triangle Intersection (Recommended)

**Pros:** True non-convex support, handles any mesh geometry
**Cons:** More complex, requires careful numerical handling

#### Phase 1: Triangle-Triangle Intersection Test
1. Implement Möller-Trumbore ray-triangle intersection as foundation
2. Implement triangle-triangle intersection (coplanar + non-coplanar cases)
3. Compute contact point, normal, and penetration depth

**Key Algorithm:** Use separating axis theorem (SAT) with 13 axes:
- 2 triangle normals
- 9 edge cross products (3 edges × 3 edges)
- Plus edge-edge closest point for contact generation

#### Phase 2: BVH-BVH Traversal
1. Implement dual-tree traversal for two BVHs
2. Prune branches where AABBs don't overlap
3. Collect candidate triangle pairs at leaves
4. Test each candidate pair with triangle-triangle algorithm

#### Phase 3: Contact Generation
1. For intersecting triangles, compute contact manifold
2. Handle edge-edge, vertex-face, and face-face contacts
3. Merge nearby contacts to reduce constraint count

### Option B: Convex Decomposition

**Pros:** Reuses existing convex-convex code, simpler
**Cons:** Preprocessing step, approximate for highly non-convex shapes

1. Use V-HACD or similar to decompose mesh into convex hulls
2. Store decomposition alongside mesh
3. Test convex hull pairs using GJK/EPA
4. Already have `ConvexMesh` support (could extend)

### Option C: Signed Distance Field (SDF) Proxy

**Pros:** Fast queries, smooth gradients
**Cons:** Memory intensive, loses sharp features

1. Voxelize mesh into SDF
2. Use mesh-SDF collision (point sampling on one mesh, SDF query on other)
3. Good for soft body / deformable interaction

---

## Recommended Implementation Plan

### Milestone 1: Triangle-Triangle Foundation ✅ COMPLETED

**Status:** Implemented in commit `5eb1264` on `feature-branch`

**Files modified:**
- `sim/sim-core/src/mesh.rs` - Added triangle-triangle intersection

**What was implemented:**
- `TriTriContact` struct for intersection results
- `triangle_triangle_intersection()` function using 13-axis SAT algorithm
- `triangle_aabbs_overlap()` for quick AABB rejection
- `test_separating_axis()` for SAT axis testing
- `compute_contact_point()` for contact point generation based on axis type
- `closest_points_on_segments()` for edge-edge contact computation

**Algorithm implemented:**
1. Quick rejection: AABB of tri_a vs AABB of tri_b
2. SAT with 13 potential separating axes (2 face normals + 9 edge cross products)
3. Skip degenerate axes (parallel edges)
4. If no separating axis found → intersection detected
5. Compute contact from minimum penetration axis
6. Contact point computed based on axis type (FaceA, FaceB, or EdgeEdge)

**Tests added (14 new tests):**
- Coplanar overlapping/separate triangles
- Crossing triangles
- Parallel separate/overlapping triangles
- Edge-edge contacts
- Vertex-face contacts
- AABB rejection
- Contact normal direction verification
- Segment closest points
- Identical triangles
- Various interpenetration scenarios

```rust
/// Result of triangle-triangle intersection test
pub struct TriTriContact {
    /// Contact point (world space)
    pub point: Point3<f64>,
    /// Contact normal (from mesh A toward mesh B)
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = interpenetrating)
    pub depth: f64,
    /// Triangle index in mesh A
    pub tri_a: usize,
    /// Triangle index in mesh B
    pub tri_b: usize,
}

/// Test intersection between two triangles
pub fn triangle_triangle_intersection(
    tri_a: &[Point3<f64>; 3],
    tri_b: &[Point3<f64>; 3],
) -> Option<TriTriContact>
```

### Milestone 2: Dual-BVH Traversal ✅ COMPLETED

**Status:** Implemented on `feature-branch`

**Files modified:**
- `sim/sim-core/src/mid_phase.rs` - Added `query_bvh_pair()` and `transform_aabb()`
- `sim/sim-core/src/mesh.rs` - Added `mesh_mesh_contact()` and `mesh_mesh_deepest_contact()`

**What was implemented:**

1. **`query_bvh_pair()`** in `mid_phase.rs`:
   - Dual-tree traversal for two BVHs using world-space transforms
   - Computes relative transform from mesh B to mesh A's local space
   - Returns `Vec<(usize, usize)>` of candidate triangle index pairs
   - Uses existing `query_pairs()` method with AABB transformation

2. **`transform_aabb()`** helper function:
   - Transforms an AABB by an isometry
   - Correctly handles rotation by transforming all 8 corners and computing new bounds

3. **`mesh_mesh_contact()`** in `mesh.rs`:
   - Uses `query_bvh_pair()` to find candidate triangle pairs
   - Transforms triangles to world space
   - Calls `triangle_triangle_intersection()` from Milestone 1
   - Returns all detected `MeshContact` structs

4. **`mesh_mesh_deepest_contact()`** convenience function:
   - Returns only the contact with maximum penetration depth
   - Useful for constraint solving

**Tests added (11 new tests):**
- `test_query_bvh_pair_identity_transform` - BVH pair with identity transforms
- `test_query_bvh_pair_with_translation` - BVH pair with translations
- `test_query_bvh_pair_with_rotation` - BVH pair with rotations
- `test_transform_aabb` - AABB transformation correctness
- `test_mesh_mesh_contact_overlapping_cubes` - Two overlapping cube meshes
- `test_mesh_mesh_contact_separate_cubes` - Non-intersecting cubes
- `test_mesh_mesh_contact_rotated_cubes` - Rotated overlapping cubes
- `test_mesh_mesh_contact_tetrahedra` - Two overlapping tetrahedra
- `test_mesh_mesh_deepest_contact` - Deepest contact convenience function
- `test_mesh_mesh_contact_identical_position` - Fully overlapping meshes
- `test_mesh_mesh_contact_edge_touching` - Edge-to-edge touching meshes

```rust
/// Query two BVHs for potentially colliding triangle pairs
pub fn query_bvh_pair(
    bvh_a: &Bvh,
    bvh_b: &Bvh,
    transform_a: &Isometry3<f64>,
    transform_b: &Isometry3<f64>,
) -> Vec<(usize, usize)>  // (tri_index_a, tri_index_b) pairs

/// Compute contacts between two triangle meshes
pub fn mesh_mesh_contact(
    mesh_a: &TriangleMeshData,
    pose_a: &Pose,
    mesh_b: &TriangleMeshData,
    pose_b: &Pose,
) -> Vec<MeshContact>

/// Get the deepest contact from a mesh-mesh collision test
pub fn mesh_mesh_deepest_contact(
    mesh_a: &TriangleMeshData,
    pose_a: &Pose,
    mesh_b: &TriangleMeshData,
    pose_b: &Pose,
) -> Option<MeshContact>
```

### Milestone 3: Integration & Dispatch ✅ COMPLETED

**Status:** Implemented on `feature-branch`

**Files modified:**
- `sim/sim-core/src/world.rs` - Added TriangleMesh ↔ TriangleMesh case in collision dispatch

**What was implemented:**

Added the `TriangleMesh ↔ TriangleMesh` match arm in `detect_contact_pair()`:

```rust
// TriangleMesh-TriangleMesh
(
    CollisionShape::TriangleMesh { data: mesh_a },
    CollisionShape::TriangleMesh { data: mesh_b },
) => {
    let contact = crate::mesh::mesh_mesh_deepest_contact(
        mesh_a,
        &body_a.state.pose,
        mesh_b,
        &body_b.state.pose,
    )?;

    Some(ContactPoint::new(
        contact.point,
        contact.normal,
        contact.penetration,
        body_a.id,
        body_b.id,
    ))
}
```

This integrates the dual-BVH traversal and triangle-triangle intersection from Milestones 1 and 2 into the main collision dispatch system.

### Milestone 4: Testing & Optimization ✅ COMPLETED

**Status:** Implemented on `feature-branch`

**Files modified/added:**
- `sim/sim-core/src/world.rs` - Added 7 integration tests for TriangleMesh ↔ TriangleMesh via World API
- `sim/sim-core/benches/collision_benchmarks.rs` - Added performance benchmarks
- `sim/sim-core/Cargo.toml` - Added criterion and rand dev-dependencies

**Integration tests added (7 new tests):**
1. `test_triangle_mesh_mesh_contact` - Two overlapping cube meshes
2. `test_triangle_mesh_mesh_no_contact` - Separated cube meshes
3. `test_triangle_mesh_mesh_tetrahedra` - Overlapping tetrahedra
4. `test_triangle_mesh_mesh_rotated` - Rotated overlapping cubes
5. `test_triangle_mesh_mesh_identical_position` - Fully overlapping meshes
6. `test_triangle_mesh_mesh_contact_normal_is_unit` - Validates contact normals
7. `test_triangle_mesh_mesh_multiple_contacts` - Validates multiple contacts

**Performance benchmarks added:**
- `mesh_mesh_collision` - Varying mesh complexity (12 to 3072 triangles)
- `mesh_mesh_rotated` - Tests with various rotation angles
- `mesh_mesh_separate` - Early rejection via BVH
- `target_10k_pairs` - Key performance metric (see results below)
- `humanoid_self_collision` - Simulates 10 body parts, all pairs and adjacent pairs

**Benchmark Results (optimized build):**
- **36,864 pairs (192×192 triangles):** 172 µs (0.17 ms) ✅ Well under 5ms target
- **589,824 pairs (768×768 triangles):** 657 µs (0.66 ms) ✅ Well under 5ms target
- **Humanoid self-collision (10 parts, 45 pairs):** 516 µs (0.5 ms) ✅
- **Adjacent pairs only (9 pairs):** 166 µs (0.17 ms) ✅

**Future optimizations (optional, not required for current targets):**
- SIMD for triangle-triangle math (leverage `sim-simd`)
- Parallel BVH traversal for large meshes
- Contact caching between frames

---

## Technical Details

### Triangle-Triangle SAT Algorithm

```
Axes to test:
1. Normal of triangle A
2. Normal of triangle B
3-11. Cross products: edge_A[i] × edge_B[j] for i,j in 0..3

For each axis:
  - Project both triangles onto axis
  - Check for overlap in projections
  - Track minimum overlap (penetration depth candidate)

If all axes show overlap:
  - Triangles intersect
  - Contact normal = axis with minimum overlap
  - Contact point = average of overlapping region
```

### Contact Point Computation

For the separating axis with minimum penetration:
1. **Face-face:** Average of intersection polygon vertices
2. **Edge-edge:** Closest points on the two edges
3. **Vertex-face:** The penetrating vertex

### Numerical Robustness

- Use epsilon tolerance for coplanarity: `|dot(n_a, n_b)| > 1 - 1e-6`
- Clamp cross products to avoid degenerate axes
- Handle near-parallel edges gracefully

---

## Dependencies

- `nalgebra` - Already used for linear algebra
- `sim-simd` - For SIMD optimization of hot paths
- No new external dependencies required

---

## Performance Considerations

### Expected Performance
- BVH query: O(log n × log m) for n, m triangles
- Triangle-triangle test: O(1) per pair, ~50-100 FLOPs
- Typical humanoid (5k triangles): ~1-5ms for full self-collision

### Optimization Opportunities
1. **Temporal coherence:** Cache BVH traversal state between frames
2. **Spatial hashing:** For dense contact regions
3. **LOD:** Use simplified collision mesh vs render mesh
4. **Island splitting:** Parallelize independent collision groups

---

## MuJoCo Comparison

MuJoCo handles mesh collision via:
1. Convex hull approximation (default)
2. Convex decomposition (user-provided)
3. No native non-convex mesh-mesh support

Our implementation will **exceed MuJoCo's capabilities** by supporting true non-convex mesh-mesh collision.

---

## Success Criteria

- [x] Triangle-triangle intersection with correct contact generation ✅ (Milestone 1)
- [x] Dual-BVH traversal with O(log n) pruning ✅ (Milestone 2)
- [x] Integrated in collision dispatch ✅ (Milestone 3)
- [x] Unit tests for basic cases ✅ (14 tests in M1 + 11 tests in M2 + 7 tests in M4 = 32 total)
- [x] Humanoid self-collision benchmark working ✅ (Milestone 4) - 516 µs for 10 parts
- [x] Performance: <5ms for 10k triangle pair test ✅ (Milestone 4) - 172 µs for 36k pairs

---

## Timeline Estimate

| Milestone | Effort | Dependencies | Status |
|-----------|--------|--------------|--------|
| Triangle-triangle intersection | 2-3 days | None | ✅ **DONE** |
| Dual-BVH traversal | 1-2 days | M1 | ✅ **DONE** |
| Integration & dispatch | 1 day | M2 | ✅ **DONE** |
| Testing & optimization | 1-2 days | M3 | ✅ **DONE** |
| **Total** | **5-8 days** | | ✅ **COMPLETE** |

---

## References

1. Möller, T. "A Fast Triangle-Triangle Intersection Test" (1997)
2. Ericson, C. "Real-Time Collision Detection" (2004) - Chapter 5
3. van den Bergen, G. "Collision Detection in Interactive 3D Environments" (2003)
4. MuJoCo docs: https://mujoco.readthedocs.io/en/stable/modeling.html#mesh
