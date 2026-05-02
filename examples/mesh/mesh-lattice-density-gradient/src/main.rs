//! mesh-lattice-density-gradient — variable-density lattice via
//! `DensityMap` on the octet-truss preset.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.7. Skeleton commit per
//! `§6.2 #17` — the 30 mm³ bbox + `octet_truss(7.5)` preset + `Gradient`
//! density map (`from_density = 0.1` at z=0, `to_density = 0.5` at
//! z=30) + `with_strut_thickness(0.6)` + `with_beam_export(true)`
//! fixture, the direct `DensityMap` evaluation anchors (4 variants:
//! `Uniform`, `Gradient`, `Radial`, `Function` + the
//! `evaluate_with_distance` demo on `SurfaceDistance`), the
//! `LatticeParams::density_at` accessor anchor, and the
//! `generate_lattice` + `cell_count == 64` + per-beam `r1` density-
//! modulation anchor (top-half mean ≈ 1.41× bottom-half mean per
//! `r1 = strut_thickness/2 × density.sqrt()`) all land in `§6.2 #18`.
//! Density-modulated counterpart to §5.6 `mesh-lattice-strut-cubic` —
//! richer octet-truss geometry better visualizes the gradient
//! (thin bottom struts → thick top struts).

fn main() {
    println!(
        "example-mesh-mesh-lattice-density-gradient: skeleton (§6.2 #17). \
         Density-gradient fixture + DensityMap variant anchors + \
         generate_lattice + per-beam r1 density anchor land in §6.2 #18."
    );
}
