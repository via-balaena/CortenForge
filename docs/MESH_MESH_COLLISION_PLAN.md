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
- No triangle-triangle intersection algorithm
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

### Milestone 1: Triangle-Triangle Foundation (2-3 days)

**Files to modify:**
- `sim-core/src/mesh.rs` - Add triangle-triangle intersection

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

**Algorithm steps:**
1. Quick rejection: AABB of tri_a vs AABB of tri_b
2. Coplanarity test (special case handling)
3. SAT with 13 potential separating axes
4. If no separating axis found → intersection
5. Compute contact from minimum penetration axis

### Milestone 2: Dual-BVH Traversal (1-2 days)

**Files to modify:**
- `sim-core/src/mid_phase.rs` - Add dual-tree query
- `sim-core/src/mesh.rs` - Add mesh-mesh query function

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
    pose_a: &Isometry3<f64>,
    mesh_b: &TriangleMeshData,
    pose_b: &Isometry3<f64>,
) -> Vec<MeshContact>
```

### Milestone 3: Integration & Dispatch (1 day)

**Files to modify:**
- `sim-core/src/world.rs` - Wire up in collision dispatch

```rust
// In detect_contact_pair(), add case:
(CollisionShape::TriangleMesh { data: mesh_a }, CollisionShape::TriangleMesh { data: mesh_b }) => {
    mesh_mesh_contact(mesh_a, &pose_a, mesh_b, &pose_b)
        .into_iter()
        .next()  // Return deepest contact
}
```

### Milestone 4: Testing & Optimization (1-2 days)

**Tests to add:**
1. Two cubes (mesh form) interpenetrating
2. Tetrahedron vs tetrahedron
3. Complex humanoid self-collision scenario
4. Performance benchmark: 1000-triangle mesh pairs

**Optimizations:**
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

- [ ] Triangle-triangle intersection with correct contact generation
- [ ] Dual-BVH traversal with O(log n) pruning
- [ ] Integrated in collision dispatch
- [ ] Unit tests for basic cases
- [ ] Humanoid self-collision demo working
- [ ] Performance: <5ms for 10k triangle pair test

---

## Timeline Estimate

| Milestone | Effort | Dependencies |
|-----------|--------|--------------|
| Triangle-triangle intersection | 2-3 days | None |
| Dual-BVH traversal | 1-2 days | M1 |
| Integration & dispatch | 1 day | M2 |
| Testing & optimization | 1-2 days | M3 |
| **Total** | **5-8 days** | |

---

## References

1. Möller, T. "A Fast Triangle-Triangle Intersection Test" (1997)
2. Ericson, C. "Real-Time Collision Detection" (2004) - Chapter 5
3. van den Bergen, G. "Collision Detection in Interactive 3D Environments" (2003)
4. MuJoCo docs: https://mujoco.readthedocs.io/en/stable/modeling.html#mesh
