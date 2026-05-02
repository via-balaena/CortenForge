//! mesh-lattice-strut-cubic — cubic strut lattice generation +
//! the 3MF beam-data export precursor.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.6. Skeleton commit per
//! `§6.2 #15` — the 25mm³ bbox + `cell_size` 5mm + `strut_thickness`
//! 1.0mm + density 1.0 + `beam_export` ON fixture, the direct
//! `generate_strut` / `generate_strut_tapered` / `combine_struts` /
//! `estimate_strut_volume` anchors, and the `generate_lattice` +
//! `cell_count` + `total_strut_length` + `BeamLatticeData` anchors
//! land in `§6.2 #16`. Strut counterpart to §5.5 `mesh-lattice-
//! tpms-gyroid` — cylindrical beams between grid nodes, NOT a TPMS
//! isosurface.

fn main() {
    println!(
        "example-mesh-mesh-lattice-strut-cubic: skeleton (§6.2 #15). \
         Cubic-strut fixture + free-fn anchors + generate_lattice + \
         BeamLatticeData verification land in §6.2 #16."
    );
}
