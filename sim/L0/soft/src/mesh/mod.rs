//! `Mesh` trait — tet-mesh storage abstraction.
//!
//! Ten items: counts, vertex lookup, positions, adjacency, quality,
//! per-tet materials, per-tet interface flags, boundary triangles,
//! and a structural-equality predicate used by change-detection.
//! Three impls: [`SingleTetMesh`] (1 tet, walking-skeleton spec §2),
//! [`HandBuiltTetMesh`] (multi-tet hand-authored scenes — Phase 2/4/5
//! gate fixtures), and `SdfMeshedTetMesh` (Phase 3 BCC + Labelle-
//! Shewchuk Isosurface Stuffing pipeline; lives in `sdf_bridge`).

use std::collections::{BTreeSet, HashMap};

use nalgebra::Point3;

use crate::Vec3;
use crate::material::{BuildableFromField, Material, MaterialField, NeoHookean};

pub mod enrich;
pub mod hand_built;
pub mod quality;
pub mod single_tet;

pub use enrich::{Tet10Topology, enrich_tet4_to_tet10};
pub use hand_built::HandBuiltTetMesh;
pub use single_tet::SingleTetMesh;

/// Tetrahedron index into a [`Mesh`]. `u32` caps at 4 B tets; skeleton
/// needs only `0`.
pub type TetId = u32;

/// Vertex index into a [`Mesh`]. `u32` caps at 4 B vertices; skeleton
/// needs `0..4`.
pub type VertexId = u32;

/// Per-element neighbour graph (tet-face adjacency). Skeleton has none
/// — single tet, no neighbours.
#[derive(Clone, Debug, Default)]
pub struct MeshAdjacency;

/// Per-tet quality metrics. All four fields are indexed by [`TetId`]
/// and have length equal to [`Mesh::n_tets`] for the owning mesh.
///
/// Populated at mesh construction time by
/// [`quality::compute_metrics`].
#[derive(Clone, Debug, Default)]
pub struct QualityMetrics {
    /// Inscribed-sphere-over-circumscribed-sphere ratio per tet.
    /// Equals 1/3 for a regular tet (theoretical max); approaches 0
    /// as tets degenerate to slivers.
    pub aspect_ratio: Vec<f64>,
    /// Minimum interior dihedral angle (radians) across the 6 edges,
    /// per tet.
    pub dihedral_min: Vec<f64>,
    /// Maximum interior dihedral angle (radians) across the 6 edges,
    /// per tet.
    pub dihedral_max: Vec<f64>,
    /// Signed tetrahedron volume per tet. Positive for right-handed
    /// orientation, negative for left-handed; magnitude equals the
    /// geometric volume.
    pub signed_volume: Vec<f64>,
}

/// Tet-mesh storage surface — read-only view of topology, positions,
/// and precomputed adjacency / quality.
///
/// Generic over the per-tet material type `M`. NH consumers omit the
/// type parameter (defaults to [`NeoHookean`]); Yeoh consumers (row
/// 23+) write `Mesh<Yeoh>` explicitly. Per arc memo D10 — row picks
/// one material model, no in-row mixing.
pub trait Mesh<M: Material = NeoHookean>: Send + Sync {
    /// Number of tetrahedra in the mesh.
    fn n_tets(&self) -> usize;

    /// Number of vertices in the mesh.
    fn n_vertices(&self) -> usize;

    /// Four-vertex indices for the given tetrahedron.
    fn tet_vertices(&self, tet: TetId) -> [VertexId; 4];

    /// Rest-configuration vertex positions, indexed by `VertexId`.
    fn positions(&self) -> &[Vec3];

    /// Neighbour graph (face-adjacency) — cold lookup.
    fn adjacency(&self) -> &MeshAdjacency;

    /// Precomputed quality metrics for the mesh.
    fn quality(&self) -> &QualityMetrics;

    /// Per-tet material parameters, indexed by [`TetId`]. Length equals
    /// [`Mesh::n_tets`].
    ///
    /// Populated at mesh-construction time by sampling a
    /// [`MaterialField`] at each tet's centroid (Tet4 default per Part 7
    /// §02 §00 — sub-leaf
    /// `70-sdf-pipeline/02-material-assignment/00-sampling.md`). Read by
    /// the Newton hot path; not re-sampled per iteration.
    fn materials(&self) -> &[M];

    /// Per-tet interface flags, indexed by [`TetId`]. Length equals
    /// [`Mesh::n_tets`].
    ///
    /// Populated at mesh-construction time by the `|φ(x_c)| < L_e`
    /// rule per Part 7 §02 §01 — sub-leaf
    /// `70-sdf-pipeline/02-material-assignment/01-composition.md`. A
    /// tet whose centroid lands within one mean-edge-length of the
    /// material-interface SDF's zero set carries `true`; all other
    /// tets carry `false`. The flag tracks the optional
    /// [`MaterialField::interface_sdf`] slot attached at field
    /// construction; meshes built from a field with no interface SDF
    /// (uniform / `LayeredScalarField`-only) carry an all-`false`
    /// vector of length `n_tets`.
    ///
    /// Diagnostic-only per Phase 4 scope memo Decision K — the Newton
    /// hot path
    /// ([`assemble_global_int_force`](crate::CpuNewtonSolver) and
    /// [`assemble_free_hessian_triplets`](crate::CpuNewtonSolver))
    /// does NOT branch on this flag. Phase H adaptive refinement
    /// (Part 7 Ch 03) is the eventual consumer; the Phase 4
    /// consumer is Part 11 mesh-quality reporting.
    fn interface_flags(&self) -> &[bool];

    /// Outward-oriented boundary triangles. Length equals the number
    /// of unique boundary faces; interior faces (shared between two
    /// tets) are culled at construction time.
    ///
    /// Populated at mesh-construction time by extracting boundary
    /// triangles from the tet topology. Outward winding is derived
    /// purely from the right-handed tet vertex order — each
    /// constructor in this crate enforces `signed_volume > 0`, so the
    /// four faces of a right-handed tet `[v0, v1, v2, v3]` orient as:
    ///
    /// - opposite v0 → `(v1, v2, v3)`
    /// - opposite v1 → `(v0, v3, v2)`
    /// - opposite v2 → `(v0, v1, v3)`
    /// - opposite v3 → `(v0, v2, v1)`
    ///
    /// Diagnostic / visualization surface — the Newton hot path never
    /// touches it. Sister cache to [`Mesh::materials`] and
    /// [`Mesh::interface_flags`] (populated once at construction,
    /// exposed by reference for zero-copy access).
    fn boundary_faces(&self) -> &[[VertexId; 3]];

    /// Structural equality: two meshes are structurally equal when they
    /// share vertex count, tet count, and per-tet vertex indices (Ch 00
    /// §02 mesh claim 3).
    ///
    /// Restricted to same-`M` comparisons (mixing NH and Yeoh meshes
    /// is rejected by D10's "no in-row mixing" rule). Trait method
    /// signatures can't carry an extra generic over a different `M`
    /// while staying object-safe; if a future use case needs
    /// cross-`M` topology comparison, lift the topology methods into
    /// a separate `MeshTopology` supertrait then.
    fn equals_structurally(&self, other: &dyn Mesh<M>) -> bool;
}

/// Sample a [`MaterialField`] at each tet's centroid, returning the
/// per-tet `Vec<M>` cache that backs [`Mesh::materials`].
///
/// Pins the Tet4 evaluation point to the centroid `(v0 + v1 + v2 + v3) /
/// 4` per Part 7 §02 §00 — Phase H Tet10 will sample at four Gauss
/// points instead and bypass this helper. Walked in `tet_id` order
/// (single-threaded) for I-5 determinism carry-forward.
///
/// Generic over `M: BuildableFromField` so mesh constructors can build
/// either NH or Yeoh caches by parameterizing on the mesh's material
/// type. The dispatch goes through [`BuildableFromField::cache_from_field`],
/// which panics if the field's variant doesn't match `M`.
#[must_use]
pub(crate) fn materials_from_field<M: BuildableFromField>(
    positions: &[Vec3],
    tets: &[[VertexId; 4]],
    field: &MaterialField,
) -> Vec<M> {
    M::cache_from_field(positions, tets, field)
}

/// Compute the per-tet interface-flag cache that backs
/// [`Mesh::interface_flags`].
///
/// Implements the book Part 7 §02 §01 rule `|φ(x_c)| < L_e`, where
/// `φ` is the SDF attached to `field` via
/// [`MaterialField::with_interface_sdf`], `x_c` is the tet centroid,
/// and `L_e` is the arithmetic mean of the tet's six edge lengths
/// (book's "tet's representative edge length"). When `field` carries
/// no interface SDF, every tet is flagged `false` — the natural
/// answer for uniform fields and `LayeredScalarField`-only fields,
/// and a clean grep target via the all-`false` payload.
///
/// Walked in `tet_id` order (single-threaded) for I-5 determinism
/// carry-forward (sister of [`materials_from_field`]).
//
// `from_fn` over the SDF-`Some` arm needs the centroid-and-edge-length
// math inline; the helper is small and tightly read by mesh
// constructors, so factoring it into named inner helpers would add
// noise without information.
#[must_use]
pub(crate) fn interface_flags_from_field(
    positions: &[Vec3],
    tets: &[[VertexId; 4]],
    field: &MaterialField,
) -> Vec<bool> {
    let Some(sdf) = field.interface_sdf() else {
        return vec![false; tets.len()];
    };
    tets.iter()
        .map(|&tv| {
            let v0 = positions[tv[0] as usize];
            let v1 = positions[tv[1] as usize];
            let v2 = positions[tv[2] as usize];
            let v3 = positions[tv[3] as usize];
            let centroid = (v0 + v1 + v2 + v3) * 0.25;
            // Six-edge mean per Part 7 §02 §01: edges are the (4 choose 2)
            // = 6 vertex pairs (0,1)(0,2)(0,3)(1,2)(1,3)(2,3). Dividing
            // the sum by 6 once is one rounding instead of six per-edge
            // accumulator-divides.
            let l_e = ((v1 - v0).norm()
                + (v2 - v0).norm()
                + (v3 - v0).norm()
                + (v2 - v1).norm()
                + (v3 - v1).norm()
                + (v3 - v2).norm())
                / 6.0;
            sdf.eval(Point3::from(centroid)).abs() < l_e
        })
        .collect()
}

/// Compute the per-mesh boundary-face cache that backs
/// [`Mesh::boundary_faces`].
///
/// A face appears in exactly one tet iff it lies on the mesh boundary:
/// interior faces are shared between two neighbouring tets with
/// opposite winding, so the canonical-key counting step below collapses
/// each interior pair into a single 2-count entry. Boundary faces drop
/// out as 1-count entries, retaining the outward winding emitted by
/// the right-handed-tet face convention:
///
/// - opposite v0 → `(v1, v2, v3)`
/// - opposite v1 → `(v0, v3, v2)`
/// - opposite v2 → `(v0, v1, v3)`
/// - opposite v3 → `(v0, v2, v1)`
///
/// Each constructor in this crate enforces right-handed tet orientation
/// (`signed_volume > 0`), so the topological winding rule is sound
/// without re-checking positions here. Sister of
/// [`materials_from_field`] and [`interface_flags_from_field`].
///
/// Walked in `tet_id` order (single-threaded) for I-5 determinism
/// carry-forward; the `HashMap` counting pass is order-independent
/// (lookups are black-box queries), so output determinism comes from
/// the input slice walk, not from the map.
#[must_use]
pub(crate) fn boundary_faces_from_topology(tets: &[[VertexId; 4]]) -> Vec<[VertexId; 3]> {
    let mut oriented: Vec<[VertexId; 3]> = Vec::with_capacity(tets.len() * 4);
    for &t in tets {
        oriented.push([t[1], t[2], t[3]]); // opposite v0
        oriented.push([t[0], t[3], t[2]]); // opposite v1
        oriented.push([t[0], t[1], t[3]]); // opposite v2
        oriented.push([t[0], t[2], t[1]]); // opposite v3
    }

    // Canonical key = sorted vertex triple. Boundary faces hit once;
    // interior faces hit twice (with opposite winding from the two
    // neighbouring tets — sort collapses both into the same key).
    let mut counts: HashMap<[VertexId; 3], u32> = HashMap::with_capacity(oriented.len());
    for face in &oriented {
        let mut key = *face;
        key.sort_unstable();
        *counts.entry(key).or_insert(0) += 1;
    }

    oriented
        .into_iter()
        .filter(|face| {
            let mut key = *face;
            key.sort_unstable();
            counts[&key] == 1
        })
        .collect()
}

/// Per-vertex tributary area on the deformed boundary surface —
/// barycentric lumped over incident boundary triangles.
///
/// Each boundary triangle contributes one third of its (deformed) area
/// to each of its three vertices: `area(v) = ⅓ · Σ area(f)` over
/// boundary faces `f` incident to `v`, with `area(f)` evaluated at the
/// supplied `positions` (so a squished buffer reports a spread-out
/// patch in real time). The returned vector is indexed by
/// [`VertexId`] and has length `positions.len()`; interior vertices —
/// those touched by no boundary face — carry `0.0`.
///
/// The one-third split makes the per-vertex areas a faithful partition
/// of the surface: their sum equals the total boundary surface area
/// exactly (each triangle's full area is dealt out across its three
/// vertices, `3 · ⅓ = 1`). That invariant is what lets a downstream
/// contact-pressure readout decompose a total force into
/// `Σ pressure(v) · area(v) · n̂(v)` without leaking or inventing area.
///
/// Pure geometry — no contact model, no solver state. `positions` are
/// the deformed vertex positions and `boundary_faces` the outward
/// triangles from [`Mesh::boundary_faces`]; the winding does not
/// matter here (area is the cross-product *magnitude*). Walked in face
/// order for determinism.
///
/// # Panics
///
/// Panics with an out-of-bounds index if any `boundary_faces` entry
/// references a `VertexId` `>= positions.len()`. The two slices must
/// share one mesh's `VertexId` space — pass `mesh.positions()` (or a
/// deformed copy of the same length) together with
/// `mesh.boundary_faces()` from that same mesh, as every index-by-
/// `VertexId` helper in this crate requires.
#[must_use]
pub fn boundary_vertex_areas(positions: &[Vec3], boundary_faces: &[[VertexId; 3]]) -> Vec<f64> {
    let mut areas = vec![0.0; positions.len()];
    for &[a, b, c] in boundary_faces {
        // One third of each triangle's deformed area lands on each of
        // its vertices; `triangle_area` is the crate's canonical
        // ½‖(b−a)×(c−a)‖ (shared with the tet-quality metrics).
        let third_area = quality::triangle_area(
            positions[a as usize],
            positions[b as usize],
            positions[c as usize],
        ) / 3.0;
        areas[a as usize] += third_area;
        areas[b as usize] += third_area;
        areas[c as usize] += third_area;
    }
    areas
}

/// Collect every vertex referenced by at least one tet, ascending.
///
/// Walks `mesh.tet_vertices(0..n_tets)`, accumulates ids into a
/// `BTreeSet` (sorted-iteration determinism per scope memo §3
/// Decision M D-3), returns the sorted `Vec<VertexId>`.
///
/// Used to filter unreferenced "orphan" lattice points out of
/// spatial-predicate boundary conditions on mesher-generated meshes:
/// `SdfMeshedTetMesh` retains every BCC lattice point in
/// `positions()`, including lattice corners in BCC tets that fell
/// entirely outside the SDF's zero set. Orphans have no
/// element-energy contribution and would silently zero a load
/// applied to them. The III-3 test
/// (`tests/sdf_forward_map_gradcheck.rs`) filters its max-z load
/// candidate through `referenced_vertices` before the argmax per
/// scope memo §3 Decision K.
//
// `as TetId` is the Mesh-trait API tax: `n_tets()` returns `usize`
// while `tet_vertices()` takes `TetId = u32`. Phase 3 meshes stay
// well below `u32::MAX`.
#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub fn referenced_vertices<M: Material>(mesh: &dyn Mesh<M>) -> Vec<VertexId> {
    let mut set: BTreeSet<VertexId> = BTreeSet::new();
    for tet_id in 0..mesh.n_tets() as TetId {
        for v in mesh.tet_vertices(tet_id) {
            set.insert(v);
        }
    }
    set.into_iter().collect()
}

#[cfg(test)]
mod boundary_vertex_areas_tests {
    use super::quality::triangle_area;
    use super::{HandBuiltTetMesh, Mesh, VertexId, boundary_vertex_areas};
    use crate::Vec3;
    use crate::material::MaterialField;

    /// One triangle: each of its three vertices gets exactly one third
    /// of the triangle's area, and the three shares sum back to the
    /// full area.
    #[test]
    fn single_triangle_splits_one_third_each() {
        // Right triangle in the z=0 plane with legs 3 and 4 → area 6.
        let positions = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(3.0, 0.0, 0.0),
            Vec3::new(0.0, 4.0, 0.0),
        ];
        let faces: [[VertexId; 3]; 1] = [[0, 1, 2]];

        let areas = boundary_vertex_areas(&positions, &faces);

        assert_eq!(areas.len(), 3);
        for a in areas {
            assert!(
                (a - 2.0).abs() < 1e-15,
                "expected 6/3 = 2 per vertex, got {a}"
            );
        }
    }

    /// A vertex referenced by no boundary face carries zero tributary
    /// area (interior-vertex case).
    #[test]
    fn unreferenced_vertex_is_zero() {
        let positions = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            // Vertex 3 sits in no face.
            Vec3::new(0.5, 0.5, 9.0),
        ];
        let faces: [[VertexId; 3]; 1] = [[0, 1, 2]];

        let areas = boundary_vertex_areas(&positions, &faces);

        // Triangle (legs 1×1) has area ½; one third (1/6) lands on each
        // of its three vertices. Vertex 3 is in no face, so it keeps the
        // additive-identity zero. Asserting the whole `Vec` (slice
        // equality, not a bare float `==`) pins the exact partition.
        let sixth = 1.0 / 6.0;
        assert_eq!(areas, vec![sixth, sixth, sixth, 0.0]);
    }

    /// Winding does not change the area (cross-product magnitude is
    /// orientation-blind): a face and its reverse give identical areas.
    #[test]
    fn reversed_winding_same_area() {
        let positions = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ];
        let forward = boundary_vertex_areas(&positions, &[[0, 1, 2]]);
        let reversed = boundary_vertex_areas(&positions, &[[0, 2, 1]]);
        assert_eq!(forward, reversed);
    }

    /// The load-bearing invariant on real topology: per-vertex areas
    /// partition the boundary exactly. On a unit-edge-scaled cube the
    /// sum equals the analytic total surface area `6·edge²`, and it
    /// also equals the independently summed per-face areas.
    #[test]
    fn cube_areas_sum_to_total_surface_area() {
        let edge = 0.1;
        let field = MaterialField::uniform(1.0e5, 4.0e5);
        let mesh = HandBuiltTetMesh::uniform_block(2, edge, &field);
        let positions = mesh.positions();
        let faces = mesh.boundary_faces();

        let areas = boundary_vertex_areas(positions, faces);
        let summed: f64 = areas.iter().sum();

        // Independent reference: sum each boundary triangle's own area.
        let face_total: f64 = faces
            .iter()
            .map(|&[a, b, c]| {
                triangle_area(
                    positions[a as usize],
                    positions[b as usize],
                    positions[c as usize],
                )
            })
            .sum();

        let analytic = 6.0 * edge * edge;
        assert!(
            (summed - face_total).abs() < 1e-15,
            "Σ per-vertex areas {summed} must equal Σ per-face areas {face_total}",
        );
        assert!(
            (summed - analytic).abs() < 1e-15,
            "Σ per-vertex areas {summed} must equal cube surface area {analytic}",
        );
    }
}
