//! scalar-field/stress-test — the sim-soft multi-material `ScalarField` superset.
//!
//! One headless validator covering the sim-soft spatial material-field surface
//! — sharp-CSG partition and smooth-sigmoid blend — folded from two former
//! per-concept examples, each now a module. Under Rule B, per-tet
//! library-behaviour correctness lives in the lib's own tests
//! (`sdf_material_tagging.rs` for the layered partition,
//! `blended_material_composition.rs` for the smoothstep composition); these
//! modules are the runnable pipeline DEMONSTRATIONS that spot-check the real
//! mesher outputs against independent oracles + emit the cf-view artifacts:
//!
//! - [`layered`] — `LayeredScalarField` sharp CSG step over a 3-shell
//!   concentric `SphereSdf` partition: each tet's `mesh.materials()` (μ, λ)
//!   read via the public accessors matches its shell's Lamé pair, plus the
//!   `interface_flags`-all-false contract (no `with_interface_sdf` call) and a
//!   centroid-inside-body geometry sanity.
//! - [`blended`] — `BlendedScalarField` smooth cubic-Hermite smoothstep between
//!   two Lamé regions: `mesh.materials()` snaps bit-exactly to the endpoint
//!   constants outside the band and grades strictly between them inside it, and
//!   `mesh.interface_flags()` are mixed (the `with_interface_sdf` band fires) —
//!   the complement of `layered`'s all-false.
//!
//! The two are complementary, not subsuming: `blended` demonstrates the
//! `BlendedScalarField::sample` + `with_interface_sdf` composition path (graded
//! materials + populated interface flags) that `layered` cannot reach; `layered`
//! demonstrates the sharp 3-shell partition + the no-interface-SDF all-false path.
//!
//! Each module self-gates and aborts (exit 101) on any mismatch, so
//! `cargo xtask run-validators` runs it red-or-green. Each also writes its
//! human-inspectable per-tet centroid PLY to `out/`
//! (`material_layer_assignment.ply`, `material_blend.ply`) for the
//! `cf-viewer` visual-review path.

mod blended;
mod layered;

use anyhow::Result;

fn main() -> Result<()> {
    layered::run()?;
    blended::run()?;
    Ok(())
}
