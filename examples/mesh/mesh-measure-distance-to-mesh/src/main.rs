//! mesh-measure-distance-to-mesh — point-to-point + point-to-mesh +
//! symmetric Hausdorff distance composed from the public surface.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.3. Skeleton commit per
//! §6.2 #7 — the two-cube fixture (`cube_a` at `[0, 1]³` and `cube_b`
//! at `[2, 3]³`, two SEPARATE `IndexedMesh` instances per spec §5.3)
//! + point-to-point + point-to-mesh + Hausdorff anchors land in §6.2 #8.

fn main() {
    println!(
        "example-mesh-mesh-measure-distance-to-mesh: skeleton (§6.2 #7). \
         Two-cube fixture + Hausdorff anchors land in §6.2 #8."
    );
}
