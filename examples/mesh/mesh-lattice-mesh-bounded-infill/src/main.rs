//! mesh-lattice-mesh-bounded-infill — FDM-style shell + lattice
//! composite via `generate_infill` on a hand-authored 50 mm cube.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.9. Skeleton commit per
//! `§6.2 #23` — the 50 mm × 50 mm × 50 mm hollow-cube fixture
//! (`min = (0, 0, 0)`, `max = (50, 50, 50)`; 8 verts / 12 tris,
//! hand-authored with outward-normal winding) fed to
//! `generate_infill` under the `InfillParams::for_fdm` preset
//! (per `infill.rs:94-105`: `cubic(5.0)` lattice + `infill_percentage
//! = 0.2` + `shell_thickness = 1.2` + `connect_to_shell = true` +
//! `solid_caps = true` + `solid_cap_layers = 4`; the fixture commit
//! overrides `cell_size` to `10.0` via `.with_cell_size(10.0)` for
//! visual readability),
//! the pre-fix anchors capturing v0.7 `generate_infill` baseline
//! behavior (`shell == mesh.clone()` per `infill.rs:353`,
//! `shell_volume == bounds_volume - interior_volume` bbox-heuristic
//! per `infill.rs:363-371`), the F6 gap-fix sub-arc (a/d/e/c/b per
//! `§6.5` ordering), and the post-fix anchors (signed-volume
//! integrals + offset-shell `vertex_count` + solid caps + lattice-to-
//! shell connections) all land in subsequent commits within this PR.
//! Composite counterpart to §5.8 `mesh-lattice-shape-bounded` (the
//! analytical-SDF-trimmed path); both ship in v1.0.

fn main() {
    println!(
        "example-mesh-mesh-lattice-mesh-bounded-infill: skeleton (§6.2 #23). \
         50 mm cube fixture + InfillParams::for_fdm preset + pre-fix anchor \
         capture (v0.7 `shell == mesh.clone()` + bbox-heuristic volumes) land \
         in the next `chore` commit. F6 gap-fix sub-arc (a/d/e/c/b per §6.5) \
         + post-fix anchors land at §6.2 #24-#29.",
    );
}
