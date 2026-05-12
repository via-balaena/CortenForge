//! Per-layer pour-volume integration and mass-budget tracking (F2).
//!
//! [`crate::CastSpec::compute_pour_volumes`] returns one
//! [`PourVolume`] per cast layer carrying the shell volume in cubic
//! metres and the corresponding pour mass in kilograms. Shell volume
//! is the new silicone added by THAT pour: for layer 0 it is
//! `vol(layers[0].body ∖ plug)`, for layer `N > 0` it is
//! `vol(layers[N].body ∖ layers[N-1].body)` — the innermost-first
//! cumulative-body convention from [`crate::CastLayer`] is what
//! makes those CSG differences correspond to one physical pour.
//!
//! Integration: sample each shell's SDF on a [`mesh_offset::ScalarGrid`]
//! at [`crate::CastSpec::mesh_cell_size_m`] and count voxels where
//! the value is strictly negative. Volume = count × `cell_size`³. The
//! Riemann-sum approximation has bias ≲ one cell of surface area ×
//! `cell_size` / 2; at 2 mm cells on a 4-6 mm shell that is well below
//! the 2 lb budget's safety margin.

use cf_design::Solid;
use mesh_offset::ScalarGrid;

use crate::error::{CastError, CastTarget};

/// 2 lb in kilograms via NIST's exact pound-to-kilogram conversion
/// (`1 lb = 0.453_592_37 kg`).
///
/// The per-silicone single-pour budget for the layered-silicone-device
/// v1.0 cast. Recommended default for
/// [`crate::CastSpec::mass_budget_kg`]; the field is required so
/// callers must set it explicitly even when adopting the default —
/// there is no silent fallback.
pub const DEFAULT_MASS_BUDGET_KG: f64 = 0.907_184_74;

/// Cells of padding on each side of the integration AABB. Matches
/// `mesher::GRID_PADDING_CELLS` so the pour-volume grid covers the
/// shell with the same headroom marching cubes uses for the mold
/// cup. Two cells keeps the shell's iso-surface comfortably inside
/// the grid interior at any reasonable cell size.
const GRID_PADDING_CELLS: usize = 2;

/// Per-layer pour-volume summary returned by
/// [`crate::CastSpec::compute_pour_volumes`].
///
/// The pour mass enforces the per-silicone single-pour budget at
/// export time. Same-material layers' masses summed across the
/// device may exceed the per-pour limit and still fit user holdings;
/// that aggregate check is deferred to F3 procedure-spec generation
/// where layer-to-material mapping is rendered for the workshop.
#[derive(Debug, Clone)]
pub struct PourVolume {
    /// Index into [`crate::CastSpec::layers`]; innermost-first.
    pub layer_index: usize,
    /// Carried-through copy of the layer's material display name.
    pub material_display_name: String,
    /// Shell volume (only the new material poured for this layer) in
    /// cubic metres. Riemann-sum approximation at
    /// [`crate::CastSpec::mesh_cell_size_m`]; see module docstring
    /// for the bias estimate.
    pub shell_volume_m3: f64,
    /// Pour mass in kilograms — `shell_volume_m3 *
    /// material.density_kg_m3`. Compared against
    /// [`crate::CastSpec::mass_budget_kg`] by
    /// [`crate::CastSpec::export_molds`] before any STL is written.
    pub pour_mass_kg: f64,
}

/// Integrate the volume of `solid`'s negative-SDF region by counting
/// grid corners where `solid.evaluate < 0`, multiplied by
/// `cell_size_m³`.
///
/// `target` labels the operation for the [`CastError::InfiniteBounds`]
/// error path when `solid` is unbounded — pour-volume integration of
/// an unbounded shell is not meaningful.
//
// Module-internal helper — `crate::spec::CastSpec::compute_pour_volumes`
// is the only caller. Bare `pub` (not `pub(crate)`) matches
// clippy::pub_self_in_priv_modules's preference for non-redundant
// visibility on items inside private modules.
pub fn integrate_negative_sdf_volume(
    solid: &Solid,
    cell_size_m: f64,
    target: CastTarget,
) -> Result<f64, CastError> {
    let bounds = solid.bounds().ok_or(CastError::InfiniteBounds(target))?;
    let grid = ScalarGrid::from_bounds(bounds.min, bounds.max, cell_size_m, GRID_PADDING_CELLS);
    let (nx, ny, nz) = grid.dimensions();

    let mut neg_count: usize = 0;
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                if solid.evaluate(&p) < 0.0 {
                    neg_count += 1;
                }
            }
        }
    }

    let cell_volume = cell_size_m * cell_size_m * cell_size_m;
    // `neg_count` is bounded by `nx * ny * nz`, the total grid corner
    // count. For any practical cf-cast cell size and device geometry
    // this stays comfortably under 2^53 (f64 mantissa width): a 1 m³
    // body at 1 mm cells produces ~10⁹ corners, four orders of
    // magnitude below the precision threshold.
    #[allow(clippy::cast_precision_loss)]
    let volume = neg_count as f64 * cell_volume;
    Ok(volume)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use approx::assert_relative_eq;
    use nalgebra::Vector3;

    use super::{DEFAULT_MASS_BUDGET_KG, integrate_negative_sdf_volume};
    use crate::error::CastTarget;
    use cf_design::Solid;

    #[test]
    fn default_mass_budget_is_two_pounds_to_kg_exact() {
        // NIST exact conversion: 1 lb = 0.453_592_37 kg. Summing
        // (rather than `2.0 * x`) sidesteps clippy::suboptimal_flops's
        // mul_add suggestion which is unnecessary here — addition of
        // two identical exact-decimal f64s is itself bit-exact.
        let lb_to_kg = 0.453_592_37_f64;
        assert!((DEFAULT_MASS_BUDGET_KG - (lb_to_kg + lb_to_kg)).abs() < f64::EPSILON);
    }

    #[test]
    fn integrate_centered_cuboid_recovers_analytic_volume() {
        // 20 × 20 × 20 mm cuboid in meters → analytic volume = 8e-6
        // m³. At 1 mm cells the Riemann sum bias is bounded by
        // surface-area × cell_size / 2 = 24e-4 × 0.001 / 2 = 1.2e-6
        // (15 % bias). Tolerance is generous to absorb both bias
        // and grid-padding rounding.
        let cuboid = Solid::cuboid(Vector3::new(0.010, 0.010, 0.010));
        let vol =
            integrate_negative_sdf_volume(&cuboid, 0.001, CastTarget::LayerBody { layer_index: 0 })
                .unwrap();
        assert_relative_eq!(vol, 8.0e-6, max_relative = 0.20);
    }

    #[test]
    fn integrate_nested_shell_subtraction_recovers_analytic_shell_volume() {
        // Outer cuboid 30×30×30 mm minus inner 20×20×20 mm cuboid
        // = analytic shell volume 27e-6 − 8e-6 = 19e-6 m³. Pins
        // that `Solid::subtract` semantics flow correctly through
        // SDF integration: shells produced by CSG diff sample as
        // the difference of cumulative volumes, with the
        // Riemann-bias bounded by both surfaces' areas × half a
        // cell.
        let outer = Solid::cuboid(Vector3::new(0.015, 0.015, 0.015));
        let inner = Solid::cuboid(Vector3::new(0.010, 0.010, 0.010));
        let shell = outer.subtract(inner);
        let vol =
            integrate_negative_sdf_volume(&shell, 0.001, CastTarget::LayerBody { layer_index: 1 })
                .unwrap();
        assert_relative_eq!(vol, 19.0e-6, max_relative = 0.20);
    }

    #[test]
    fn integrate_unbounded_solid_errors_with_target_label() {
        // `Solid::plane` is unbounded — integration cannot sample a
        // finite grid. Error surfaces the supplied `target` label
        // for caller diagnosis.
        use crate::error::CastError;
        let plane = Solid::plane(Vector3::new(0.0, 0.0, 1.0), 0.0);
        let err =
            integrate_negative_sdf_volume(&plane, 0.001, CastTarget::LayerBody { layer_index: 3 })
                .unwrap_err();
        assert!(matches!(
            err,
            CastError::InfiniteBounds(CastTarget::LayerBody { layer_index: 3 })
        ));
    }
}
