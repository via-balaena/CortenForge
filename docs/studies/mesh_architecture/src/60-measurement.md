# mesh-measure — dimensions, OBB, cross-sections, distance

What this part covers (when authored to depth):

## Scope and tier

`mesh-measure` is L0 — analytical operations on `IndexedMesh` that report geometric properties. Pure read operations; no mesh mutation. Four sub-areas:

1. **Dimensions** — axis-aligned bounding box and derived width/depth/height/volume scalars.
2. **Oriented bounding box (OBB)** — minimum-volume bounding box via PCA on vertex positions.
3. **Cross-sections** — plane-mesh intersection producing 2D contours with area + perimeter metrics.
4. **Distance** — point-to-point, point-to-mesh distance queries.

Operations are independent — each is a one-shot pure function over a mesh. No shared state; no precomputation between calls. Trivially parallelizable across multiple meshes if a use case ever needs that.

## Cross-sections — the most generative tool

```rust
let section = cross_section(&mesh, point_on_plane, plane_normal);
// section.area, section.perimeter, section.contours: Vec<Polyline>
```

Plane-mesh intersection produces a `CrossSection` with the contour polylines (one or more closed loops where the plane cuts the surface), plus aggregate area and perimeter scalars. `cross_sections` (plural) accepts a list of planes for batch sampling.

Cross-sections are **the most generatively-useful primitive in `mesh-measure`** — applications include:

- **Soft-body slice visualization.** Sweep a plane through a deforming body; render contours as a 2D animation; compare to the 3D rendered mesh to verify deformation makes sense at every depth.
- **Mold parting-line preview.** A horizontal cross-section through a part shows what its silhouette looks like at the parting plane — the casting domain's parting-surface determination starts here.
- **Manufacturing constraint surfaces.** Wall-thickness verification can sample the part along axial directions and report the minimum wall thickness per slice.
- **Volume integration via slabs.** Rule-of-thumb volume estimation by integrating cross-section areas along an axis; used as a sanity check against `IndexedMesh::aabb()`-derived volume estimates.

The depth pass covers the algorithmic specifics (per-triangle plane intersection, edge-walking to assemble closed contours, handling of the degenerate cases where the plane grazes a vertex or coincides with a face) and the cross-section's role in soft-body visualization examples (the [`mesh-measure-cross-section`](80-examples.md) example walks through the planar-slicing surface end-to-end).

## Dimensions and OBB

```rust
let dims = dimensions(&mesh);
// dims.aabb, dims.width, dims.depth, dims.height, dims.volume

let obb = oriented_bounding_box(&mesh);
// obb.center, obb.axes (3×3), obb.half_extents, obb.volume
```

`Dimensions` exposes the AABB plus derived scalars (axis-aligned width/depth/height; AABB volume = width × depth × height; *not* mesh volume, which would require integration).

`OrientedBoundingBox` is computed via PCA on vertex positions — the principal axes of the position covariance matrix become the OBB axes; half-extents are the per-axis ranges in the rotated frame. Minimum-volume in the asymptotic sense (PCA-based OBB is optimal as vertex density approaches surface density; near-optimal in practice).

The depth pass covers: when AABB is sufficient (axis-aligned scenes, dimensioning); when OBB is necessary (auto-orientation, pose-invariant size matching, tightest-fit calculations); the PCA assumption (vertex sampling represents surface density — fails for unevenly-tessellated meshes); and the workaround (resample mesh to uniform vertex density before OBB if the mesh is unevenly tessellated).

## Distance queries

```rust
let dist = distance_to_mesh(point, &mesh);
let closest = closest_point_on_mesh(point, &mesh);
// also: measure_distance for pairs of points (trivial wrapper)
```

Point-to-mesh distance is unsigned (no inside/outside test — for that, use `mesh-sdf::point_in_mesh` plus this distance for the magnitude). Closest-point query returns the actual position on the surface, useful for projection operations.

The depth pass covers the closest-point algorithm (per-triangle closest-point check, accelerated by spatial hash for large meshes), the relationship to `mesh-sdf::SignedDistanceField` (mesh-sdf adds the sign and amortizes setup cost across many queries; mesh-measure's `distance_to_mesh` is the one-off pattern), and when each is the right tool.

## What's NOT in `mesh-measure`

- **Topology metrics** (Euler characteristic, genus, number of holes). These live in `mesh-repair` adjacent to the validation surface.
- **Surface area / mesh volume integration.** The `Dimensions::volume` field is AABB volume, not mesh volume. True surface-integration mesh volume is a separate operation; not currently in any crate but a reasonable addition (probably belongs in `mesh-measure`).
- **Curvature estimation.** Per-vertex Gaussian / mean curvature; useful for surface analysis but not currently implemented. Future work if a use case bites.
- **Shape descriptors / shape signatures.** Heat kernel signatures, harmonic shape descriptors — out of scope; research-frontier territory.

The depth pass enumerates these and discusses where each would live if added.
