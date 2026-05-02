//! mesh-lattice-shape-bounded — boundary-conforming TPMS lattice
//! clipped to an analytical sphere SDF via `with_shape_sdf`.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.8. Skeleton commit per
//! `§6.2 #19` — the 30 mm³ bbox centered at origin (`min = (-15,
//! -15, -15)`, `max = (15, 15, 15)`) + `LatticeParams::gyroid(10.0)`
//! preset + sphere SDF of radius 12 mm centered at origin
//! (`Arc::new(|p| p.coords.norm() - 12.0)`) + resolution 15 fixture,
//! the direct `is_outside_shape` predicate anchors (interior /
//! exterior + default `false` when no SDF set), the with-vs-without
//! trimmed-vertex-count anchor (the SDF-clipped lattice has strictly
//! fewer vertices than the un-trimmed comparison), and the per-vertex
//! `(v - origin).norm() < sphere_radius + voxel_size + cushion` bound
//! all land in `§6.2 #20`. Boundary-conforming counterpart to §5.5
//! `mesh-lattice-tpms-gyroid` (same TPMS path, but trimmed to a
//! mathematical shape rather than the bbox); complementary to §5.9
//! `mesh-lattice-mesh-bounded-infill` (mesh-bounded composite path
//! via `generate_infill`).

fn main() {
    println!(
        "example-mesh-mesh-lattice-shape-bounded: skeleton (§6.2 #19). \
         Sphere-SDF fixture + is_outside_shape predicate anchors + \
         with-vs-without trimmed-vertex-count + per-vertex distance \
         bound land in §6.2 #20."
    );
}
