//! mesh-lattice-tpms-gyroid — TPMS surface generation via the
//! gyroid implicit function + marching-cubes vertex-soup output.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.5. Skeleton commit per
//! `§6.2 #12` — the 30mm³ bbox + `cell_size` 10mm + density 0.5 +
//! resolution 15 + `wall_thickness` 1.5 fixture + free-fn `gyroid`
//! / `density_to_threshold` / `make_shell` anchors +
//! `generate_lattice` `cell_count` + vertex-soup signature
//! (`vertex_count == 3 × triangle_count` bit-exact, F10) +
//! per-vertex SDF surface verification land in `§6.2 #13` +
//! `§6.2 #14`. First lattice example.

fn main() {
    println!(
        "example-mesh-mesh-lattice-tpms-gyroid: skeleton (§6.2 #12). \
         Gyroid fixture + free-fn anchors + generate_lattice + \
         soup-signature + per-vertex SDF verification land in §6.2 #13 + #14."
    );
}
