//! measure/stress-test — the `mesh-measure` domain validation superset.
//!
//! One headless validator covering the full `mesh-measure` public
//! surface, folded from three former per-surface examples (each now a
//! module preserving its hand-authored fixture + oracle checks verbatim):
//!
//! - [`bounding_box`] — `dimensions` + `oriented_bounding_box` (AABB,
//!   PCA-derived OBB, brick-alone extent recovery) on a two-shape fixture.
//! - [`cross_section`] — `cross_section` + `cross_sections` +
//!   `circumference_at_height` + `area_at_height` (shoelace area/perimeter,
//!   centroid, plane-normal normalization, out-of-mesh) on a UV-cylinder.
//! - [`distance`] — `measure_distance` + `closest_point_on_mesh` +
//!   `distance_to_mesh` + symmetric Hausdorff between two vertex-disjoint
//!   cubes.
//!
//! Each module self-gates against closed-form / analytic oracles and
//! aborts (exit 101) on any mismatch, so `cargo xtask run-validators`
//! runs it red-or-green. Each also writes its human-inspectable PLY
//! artifact to `out/` (`mesh.ply`, `cylinder.ply`, `two_cubes.ply`) for
//! the `cf-viewer` visual-review path.

mod bounding_box;
mod cross_section;
mod distance;

use anyhow::Result;

fn main() -> Result<()> {
    bounding_box::run()?;
    cross_section::run()?;
    distance::run()?;
    Ok(())
}
