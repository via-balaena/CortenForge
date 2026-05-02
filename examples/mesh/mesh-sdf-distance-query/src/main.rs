//! mesh-sdf-distance-query — signed-distance + inside/outside +
//! closest-point + bulk-query coverage of the `mesh-sdf` public surface.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.4. Skeleton commit per
//! `§6.2 #9` — the octahedron fixture (6 verts + 8 tris with parity-
//! flipped winding for the 4 octants where `sx*sy*sz = -1`) +
//! 14 SDF query points + `closest_point_on_triangle` /
//! `ray_triangle_intersect` / `point_segment_distance_squared` direct
//! anchors + 1000-point cubic grid PLY via `save_ply_attributed` with
//! `extras["signed_distance"]` lands in `§6.2 #10`. First example
//! outside `mesh-measure`.

fn main() {
    println!(
        "example-mesh-mesh-sdf-distance-query: skeleton (§6.2 #9). \
         Octahedron fixture + SDF anchors + bulk-grid PLY land in §6.2 #10."
    );
}
