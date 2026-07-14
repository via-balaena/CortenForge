//! scalar-field/stress-test — the sim-soft multi-material `ScalarField` superset.
//!
//! One headless validator covering the sim-soft spatial material-field surface
//! — sharp-CSG partition and smooth-sigmoid blend — folded from two former
//! per-concept examples (each now a module preserving its hand-authored fixture
//! + oracle checks verbatim):
//!
//! - [`layered`] — `LayeredScalarField` sharp CSG step over a 3-shell
//!   concentric `SphereSdf` partition: categorical material-tag partition
//!   (`{0, 1, 2}`, exactly 3 unique ids), exact per-shell populations, and the
//!   `interface_flags`-all-false contract (no `with_interface_sdf` call).
//! - [`blended`] — `BlendedScalarField` smooth cubic-Hermite smoothstep between
//!   two Lamé regions: monotone μ gradient across the band, bit-exact `s=0`/`s=1`
//!   snap outside it, and the positive per-tet `interface_flags` book rule
//!   (`|φ(x_c)| < L_e`, IV-6) — the exact complement of `layered`'s all-false.
//!
//! The two are complementary, not subsuming: `blended` exercises code paths
//! (`BlendedScalarField::sample` FMA chain, `with_interface_sdf`) that `layered`
//! cannot reach, and `layered` is the sole coverage of the no-interface-SDF
//! partition path + categorical cardinality.
//!
//! Each module self-gates against closed-form / partition / topology oracles
//! and aborts (exit 101) on any mismatch, so `cargo xtask run-validators` runs
//! it red-or-green. Each also writes its human-inspectable per-tet centroid PLY
//! to `out/` (`material_layer_assignment.ply`, `material_blend.ply`) for the
//! `cf-viewer` visual-review path.

mod blended;
mod layered;

use anyhow::Result;

fn main() -> Result<()> {
    layered::run()?;
    blended::run()?;
    Ok(())
}
