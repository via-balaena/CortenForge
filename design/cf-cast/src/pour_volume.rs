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
//! Integration: sample each shell's SDF at the CENTRE of every cell of a
//! [`mesh_offset::ScalarGrid`] at
//! [`crate::CastSpec::compute_pour_volumes`]'s integration cell, and count
//! the cells whose value is strictly negative. Volume = count ×
//! `cell_size`³.
//!
//! ⚠⚠ **Centres, not corners, and the difference is not cosmetic.** Corner
//! sampling used a strict `< 0`, so a face lying exactly ON a grid plane
//! contributed nothing — and since the grid origin derives from the solid's
//! own bounds, axis-aligned faces land on grid planes by construction. A
//! 4 mm wall at a 2 mm cell is two corner planes; losing one is losing half
//! the wall. Whether it IS lost turns on float rounding at the boundary,
//! which is why the old error wandered with the cell size instead of
//! shrinking. Measured at 2 mm against exact analytic volumes: a cuboid
//! shell read **−26 %**, a spherical shell −0.6 %. The error was
//! anti-conservative — it told the bencher to mix LESS than the mold holds.
//! Centre sampling makes axis-aligned geometry exact and leaves only
//! curvature, which converges with the cell.
//! → `a_thin_walled_box_does_not_lose_a_quarter_of_its_volume`
//!
//! The integration cell is floored at [`POUR_VOLUME_MIN_CELL_SIZE_M`]
//! and is **decoupled from `mesh_cell_size_m`** (§MA-17/S2). The grid
//! bake is cubic in `1/cell_size`; at the 0.5 mm production cup cell a
//! pour-volume grid tied to `mesh_cell_size_m` is 35 M cells per layer
//! and burns 15+ min. So pour-volume never samples finer than 2 mm, while
//! coarser prototyping meshes still integrate at their own cell.
//!
//! ⚠ The figure that used to justify that floor — *"441.24 g @ 2 mm vs
//! 442.91 g @ 1.5 mm on the `base_mold` shells"* — compares two BIASED
//! estimates and says nothing about either one's accuracy. Two biased
//! numbers agreeing is not evidence. What the floor actually costs is now
//! measurable against exact volumes: axis-aligned geometry is exact at any
//! cell, and curvature contributes roughly +1 % at 2 mm on a 4 mm spherical
//! shell, shrinking as the cell does.

use cf_design::Solid;
use mesh_offset::ScalarGrid;
use nalgebra::Point3;

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

/// Floor on the pour-volume integration cell size (§MA-17/S2).
///
/// Pour-volume integration is a midpoint voxel count for a *mass-budget*
/// estimate, not a surface mesh, while the dense grid bake is cubic in
/// `1/cell_size`. What the floor costs is curvature error only — about
/// +1 % at 2 mm on a 4 mm spherical shell; axis-aligned geometry is exact
/// at any cell. (It previously cited a sub-0.4 % figure that was a
/// convergence comparison, not an accuracy one — see the module docstring.)
///
/// Flooring the integration cell here keeps it cheap and
/// **decoupled from `mesh_cell_size_m`**, so a fine production cup cell
/// (0.5 mm) doesn't pay a 35 M-cell, 15+ min integration per layer for
/// no budget-relevant accuracy.
///
/// This is a *floor* (`mesh_cell_size_m.max(..)`): a coarse prototyping
/// mesh (e.g. 3 mm) still integrates at its own cell. Contrast the
/// auxiliary-part *ceilings* (`PLATFORM_MAX_CELL_SIZE_M` etc.), which
/// cap the coarsest cell for surface quality; here the concern is the
/// opposite — never sample *finer* than the budget needs.
pub const POUR_VOLUME_MIN_CELL_SIZE_M: f64 = 0.002;

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
    /// cubic metres. Riemann-sum approximation at the integration cell,
    /// which is `mesh_cell_size_m` floored up to at least
    /// [`POUR_VOLUME_MIN_CELL_SIZE_M`] — decoupled from the mesh cell
    /// (§MA-17/S2); see module docstring for the bias estimate.
    pub shell_volume_m3: f64,
    /// Pour mass in kilograms — `shell_volume_m3 *
    /// material.density_kg_m3`. Compared against
    /// [`crate::CastSpec::mass_budget_kg`] by
    /// [`crate::CastSpec::export_molds_v2`] before any STL is written.
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

    // ★★★ SAMPLE CELL CENTRES, NOT GRID CORNERS. A corner count uses a strict
    // `< 0`, so a face lying exactly ON a grid plane contributes nothing and a
    // slab of thickness `T` can yield `T/h − 1` planes instead of `T/h`.
    // ⚠ CAN, not does: whether the boundary corner reads inside or outside
    // comes down to the last bits of `origin + k·cell`, which is precisely why
    // the old error was NON-MONOTONE in the cell size rather than a bias that
    // shrank. Because
    // `ScalarGrid::from_bounds` derives the origin from the solid's own bounds,
    // axis-aligned faces land on grid planes by construction — and at 2 cells
    // through a wall, losing one plane is losing half the wall.
    //
    // Measured at the 2 mm production cell against exact analytic volumes,
    // corners vs centres:
    //
    //   spherical shell, 4 mm wall   −0.64 %  →  +1.33 %
    //   cuboid shell,    4 mm wall  −25.90 %  →  −0.00 %
    //   cuboid shell,    6 mm wall  −26.23 %  →  +0.00 %
    //   wheel tire (flat sides)      −5.52 %  →  −5.38 %
    //
    // The midpoint offset also makes the remainder CONVERGE. On the wheel tire
    // the centre rule reads −5.38 % at 2 mm, +0.39 % at 1 mm and −0.10 % at
    // 0.5 mm, where the corner rule wandered (−5.5 %, −1.4 %, −2.2 %) because
    // its error was an alignment artifact rather than a discretisation one.
    //
    // ⇒ the cell count is `n − 1` per axis: `n` corners bound `n − 1` cells.
    let half_cell = cell_size_m / 2.0;
    let mut neg_count: usize = 0;
    for iz in 0..nz.saturating_sub(1) {
        for iy in 0..ny.saturating_sub(1) {
            for ix in 0..nx.saturating_sub(1) {
                let corner = grid.position(ix, iy, iz);
                let centre = Point3::new(
                    corner.x + half_cell,
                    corner.y + half_cell,
                    corner.z + half_cell,
                );
                if solid.evaluate(&centre) < 0.0 {
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

    use super::{
        DEFAULT_MASS_BUDGET_KG, POUR_VOLUME_MIN_CELL_SIZE_M, integrate_negative_sdf_volume,
    };
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
        // 20 × 20 × 20 mm cuboid → 8e-6 m³, sampled at 1 mm cells.
        //
        // ⚠ This asserted a 20 % tolerance, with a comment computing a 15 %
        // bias and calling the tolerance "generous to absorb" it. The bias was
        // accommodated rather than diagnosed: it came from counting grid
        // CORNERS, which drops a whole plane wherever an axis-aligned face
        // lands on the grid. Sampling cell CENTRES makes an axis-aligned box
        // exact whenever the cell divides its extent, which 1 mm does here.
        //
        // ⚠⚠ BUT THIS TEST IS NOT THE REGRESSION GATE, and it looked like one.
        // Reverting to corner sampling leaves it PASSING at 1e-9: the boundary
        // corner lands at −0.01000000000000000021, just outside, while the far
        // side lands just inside — so the count comes to exactly 20³. The old
        // rule was exact here by floating-point luck, which is the same
        // fragility that made its error non-monotone in the cell size.
        // `a_thin_walled_box_does_not_lose_a_quarter_of_its_volume` and the
        // nested-shell test below are what actually catch the reversion.
        let cuboid = Solid::cuboid(Vector3::new(0.010, 0.010, 0.010));
        let vol =
            integrate_negative_sdf_volume(&cuboid, 0.001, CastTarget::LayerBody { layer_index: 0 })
                .unwrap();
        assert_relative_eq!(vol, 8.0e-6, max_relative = 1e-9);
    }

    #[test]
    fn integrate_nested_shell_subtraction_recovers_analytic_shell_volume() {
        // Outer cuboid 30×30×30 mm minus inner 20×20×20 mm cuboid
        // = analytic shell volume 27e-6 − 8e-6 = 19e-6 m³. Pins
        // that `Solid::subtract` semantics flow correctly through
        // SDF integration.
        //
        // ⚠ A SHELL is where the corner rule was worst: this one has a 5 mm
        // wall, so at 1 mm cells it lost one plane in five. Same 20 % tolerance
        // as above, same accommodation. Centre sampling makes it exact.
        let outer = Solid::cuboid(Vector3::new(0.015, 0.015, 0.015));
        let inner = Solid::cuboid(Vector3::new(0.010, 0.010, 0.010));
        let shell = outer.subtract(inner);
        let vol =
            integrate_negative_sdf_volume(&shell, 0.001, CastTarget::LayerBody { layer_index: 1 })
                .unwrap();
        assert_relative_eq!(vol, 19.0e-6, max_relative = 1e-9);
    }

    #[test]
    fn a_thin_walled_box_does_not_lose_a_quarter_of_its_volume() {
        // ★ THE REGRESSION GATE. Under corner sampling this shell read
        // −25.90 %: a 4 mm wall at a 2 mm cell is two corner planes, and an
        // axis-aligned face landing on the grid costs one of them. The
        // bencher was told to mix three quarters of what the mold holds.
        //
        // ⚠ It hid for four months because every cast was SCAN-DERIVED and
        // curved — a sphere shell reads −0.64 % on the same rule. The wheel is
        // the first parametric subject, and the first with flat faces in its
        // shell.
        let outer = Vector3::new(0.030, 0.030, 0.025);
        for wall_m in [0.004_f64, 0.006] {
            let inner = outer - Vector3::new(wall_m, wall_m, wall_m);
            let shell = Solid::cuboid(outer).subtract(Solid::cuboid(inner));
            let exact = 8.0 * (outer.x * outer.y * outer.z - inner.x * inner.y * inner.z);
            let vol = integrate_negative_sdf_volume(
                &shell,
                POUR_VOLUME_MIN_CELL_SIZE_M,
                CastTarget::LayerBody { layer_index: 0 },
            )
            .unwrap();
            assert_relative_eq!(vol, exact, max_relative = 1e-9);
        }
    }

    #[test]
    fn the_error_on_a_curved_shell_shrinks_with_the_cell() {
        // Curvature is the error that REMAINS, and the point of the midpoint
        // rule is that what remains converges. The corner rule's did not: on
        // the wheel tire it read −5.5 %, −1.4 %, −2.2 % at 2 / 1 / 0.5 mm,
        // wandering because the error was an alignment artifact.
        //
        // ⚠ Asserts the TREND, not a pinned value — a tolerance per cell size
        // would pass a rule that got the right answers for the wrong reason.
        let (ro, ri) = (0.030_f64, 0.024);
        let shell = Solid::sphere(ro).subtract(Solid::sphere(ri));
        let exact = 4.0 / 3.0 * std::f64::consts::PI * (ro.powi(3) - ri.powi(3));
        let err = |cell: f64| {
            let v = integrate_negative_sdf_volume(
                &shell,
                cell,
                CastTarget::LayerBody { layer_index: 0 },
            )
            .unwrap();
            ((v - exact) / exact).abs()
        };
        let (coarse, fine) = (err(0.002), err(0.000_5));
        assert!(
            fine < coarse,
            "refining 2 mm → 0.5 mm must reduce the error; got {coarse:.5} → {fine:.5}"
        );
        assert!(
            coarse < 0.05,
            "even the coarse cell should stay under 5 %; got {coarse:.5}"
        );
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
