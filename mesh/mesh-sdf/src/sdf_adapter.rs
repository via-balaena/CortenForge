//! `cf_geometry::Sdf` adapter blankets for mesh-sdf's signed-distance
//! types.
//!
//! Hosts the two adapter impls that bridge mesh-sdf's
//! [`Signed`](crate::Signed) composition + [`CachedGridSdf`](crate::CachedGridSdf)
//! cached grid to the workspace-wide [`Sdf`](cf_geometry::Sdf) trait.
//! Orphan-rule-clean: `Signed` and `CachedGridSdf` are mesh-sdf's own
//! types, so the impls live wherever mesh-sdf decides — and putting
//! them next to the trait import in this module keeps the surface
//! self-contained.

use cf_geometry::Sdf;
use nalgebra::{Point3, Vector3};

use crate::{CachedGridSdf, Sign, Signed, UnsignedDistance};

/// Every composed [`Signed<D, S>`] satisfies the [`Sdf`] contract —
/// blanket adapter spanning the decomposed [`UnsignedDistance`] +
/// [`Sign`] traits.
///
/// `eval` delegates to [`Signed::evaluate`] (negative inside, positive
/// outside, zero on the surface).
///
/// `grad` is a central finite-difference approximation with step size
/// `1e-6` (matching cf-design's existing `Solid::user_fn` finite-diff
/// fallback). Mesh SDFs have piecewise-smooth gradients with
/// discontinuities across face boundaries, so an analytic gradient is
/// not available in closed form. Within `1e-6` of a face boundary the
/// computed gradient may be biased toward one side's outward normal;
/// consumers requiring boundary-stable gradients should query at points
/// well within face interiors.
///
/// **Sign-oracle caveat.** The reliability of `eval`'s sign branch is
/// determined by the wrapped `S: Sign` oracle.
/// [`crate::PseudoNormalSign`] is reliable on watertight, well-formed
/// meshes but **fragile** on decimated / cleaned scans with cap fans
/// whose winding flipped during reconstruction or high-valence apex
/// vertices. Consumers deriving an SDF from a cf-scan-prep cleaned
/// scan should prefer [`crate::FloodFillSign`] in place of
/// [`crate::PseudoNormalSign`]. See
/// `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md` for the rationale.
impl<D, S> Sdf for Signed<D, S>
where
    D: UnsignedDistance,
    S: Sign,
{
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.evaluate(p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        let eps = 1e-6;
        let inv_2eps = 0.5 / eps;
        Vector3::new(
            (self.evaluate(Point3::new(p.x + eps, p.y, p.z))
                - self.evaluate(Point3::new(p.x - eps, p.y, p.z)))
                * inv_2eps,
            (self.evaluate(Point3::new(p.x, p.y + eps, p.z))
                - self.evaluate(Point3::new(p.x, p.y - eps, p.z)))
                * inv_2eps,
            (self.evaluate(Point3::new(p.x, p.y, p.z + eps))
                - self.evaluate(Point3::new(p.x, p.y, p.z - eps)))
                * inv_2eps,
        )
    }
}

/// [`CachedGridSdf`] implements [`Sdf`] via its native trilinear-
/// interpolated signed distance + analytic-on-the-trilinear gradient.
/// The grid IS the smoothed source, so `eval` reads directly and
/// `grad` central-differences the trilinear interpolant at the lattice
/// spacing — exact for the piecewise-trilinear representation, no
/// `Signed::grad`-style 1e-6 finite-difference step needed.
impl Sdf for CachedGridSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.signed_distance(p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        self.gradient(p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_fixtures::unit_tetrahedron;
    use approx::assert_relative_eq;

    /// Construct the canonical `Signed<TriMeshDistance, PseudoNormalSign>`
    /// composition — the blanket `impl<D, S> Sdf for Signed<D, S>`
    /// is what these tests pin.
    #[allow(clippy::expect_used)]
    fn mesh_sdf_for_tetrahedron() -> Signed<crate::TriMeshDistance, crate::PseudoNormalSign> {
        let distance = crate::TriMeshDistance::new(unit_tetrahedron())
            .expect("tetrahedron fixture has four faces");
        let sign = crate::PseudoNormalSign::from_distance(&distance);
        Signed { distance, sign }
    }

    #[test]
    fn mesh_sdf_eval_sign_convention_in_contact_band() {
        let sdf = mesh_sdf_for_tetrahedron();

        // Strictly inside (negative): on the bottom-centroid → apex
        // axis halfway up.
        let interior = Point3::new(0.5, 0.289, 0.4);
        assert!(Sdf::eval(&sdf, interior) < 0.0);

        // Strictly outside (positive): one unit below the bottom-face
        // centroid — the closest face is unambiguously the bottom
        // (outward `-z`), and the signed distance is exactly 1.
        let exterior_below = Point3::new(0.5, 0.289, -1.0);
        assert_relative_eq!(Sdf::eval(&sdf, exterior_below), 1.0, epsilon = 1e-12,);
    }

    #[test]
    fn mesh_sdf_grad_below_bottom_face_approximates_outward_normal() {
        let sdf = mesh_sdf_for_tetrahedron();

        // Probe directly below the bottom-face centroid (0.5, 0.289, 0)
        // at z = -1 — the closest face is the bottom triangle (winding
        // `[0, 2, 1]`, outward normal -z), and the central-difference
        // gradient should match that normal up to the float-equality
        // tolerance of mesh-sdf's distance computation.
        let p = Point3::new(0.5, 0.289, -1.0);
        let g = Sdf::grad(&sdf, p);
        assert_relative_eq!(g, -Vector3::z(), epsilon = 1e-6);
    }

    /// **The scale regime `Sdf::grad`'s absolute `1e-6` step is valid in — measured, because
    /// nothing has ever pinned it.**
    ///
    /// The blanket `grad` above central-differences with a **hardcoded absolute** step. That is
    /// the same shape of defect as the pseudo-normal area floor pinned in `sdf.rs`: a constant
    /// with a dimension, applied to geometry of unknown size. And the two pull in **opposite**
    /// directions — the area floor wants coordinates large enough for triangle areas to clear
    /// `f32::EPSILON/2`, while this step wants them small enough that a `1e-6` displacement is
    /// still resolvable. Any proposal to normalise mesh coordinates internally has to satisfy
    /// both at once, so both regimes have to be known.
    ///
    /// ## Why `‖∇φ‖` is the right instrument
    ///
    /// A tessellated sphere's signed distance is a genuine distance function — to the
    /// *polyhedron*, not the ideal sphere — so `‖∇φ‖ = 1` **exactly**, everywhere it is
    /// differentiable, at every radius. The polyhedral gradient's *direction* differs from the
    /// ideal radial one by the facet's angular deviation, and that error is scale-invariant, so
    /// direction cannot separate tessellation from precision. **Magnitude can**: any departure
    /// from 1 is numerical and nothing else.
    ///
    /// The mechanism under test: `TriMeshDistance::distance` computes an `f64` norm between the
    /// query point and a projection that came back through **`f32`**, so the projection carries
    /// ~`6e-8` *relative* error — which at extent `E` is ~`6e-8·E` absolute. Dividing that by
    /// the fixed `2h = 2e-6` gives a gradient error growing **linearly in `E`**. Probes sit at a
    /// fixed *relative* position (`1.5·r`) so the geometry is identical across the sweep and
    /// only the coordinate magnitude changes.
    ///
    /// Reports the whole sweep and asserts only the anchor: radius 1.0 — inside the regime
    /// `sdf.rs`'s sibling scale-regime gate already covers — must give a unit-magnitude
    /// gradient. The boundary is printed, not pinned, until it has been read once.
    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn grad_step_scale_regime() {
        // Deterministic, well-spread probe directions; no RNG, no trig-table coincidences with
        // the sphere's own tessellation seams.
        let dirs: Vec<Vector3<f64>> = (0..64)
            .map(|i| {
                let z = 2.0f64.mul_add(f64::from(i) / 63.0, -1.0) * 0.98;
                let phi = f64::from(i) * 2.399_963_229_728_653; // golden angle
                let r = (1.0 - z * z).sqrt();
                Vector3::new(r * phi.cos(), r * phi.sin(), z).normalize()
            })
            .collect();

        println!(
            "\n{:>10} {:>12} {:>14} {:>14} {:>12}",
            "radius", "h/radius", "max |‖g‖-1|", "rms |‖g‖-1|", "max ang°"
        );
        let mut anchor_mag_err = f64::NAN;
        for radius in [1e-3_f64, 1e-2, 1e-1, 1.0, 10.0, 100.0, 1000.0] {
            let distance =
                crate::TriMeshDistance::new(crate::test_fixtures::uv_sphere(radius, 24, 48))
                    .expect("sphere fixture is non-empty");
            let sign = crate::PseudoNormalSign::from_distance(&distance);
            let sdf = Signed { distance, sign };

            let (mut max_mag, mut sum_sq, mut max_ang) = (0.0_f64, 0.0_f64, 0.0_f64);
            for d in &dirs {
                let p = Point3::from(d * (1.5 * radius));
                let g = Sdf::grad(&sdf, p);
                let mag_err = (g.norm() - 1.0).abs();
                max_mag = max_mag.max(mag_err);
                sum_sq += mag_err * mag_err;
                if g.norm() > 0.0 {
                    let cos = (g.normalize().dot(d)).clamp(-1.0, 1.0);
                    max_ang = max_ang.max(cos.acos().to_degrees());
                }
            }
            let rms = (sum_sq / dirs.len() as f64).sqrt();
            println!(
                "{radius:>10.0e} {:>12.2e} {max_mag:>14.3e} {rms:>14.3e} {max_ang:>12.4}",
                1e-6 / radius,
            );
            if (radius - 1.0).abs() < f64::EPSILON {
                anchor_mag_err = max_mag;
            }
        }

        // The anchor, and only the anchor: radius 1.0 sits inside the regime the sibling
        // scale-regime gate in `sdf.rs` already covers, so a unit-magnitude gradient there is
        // the minimum this adapter must deliver. Everything else is reported so the boundary
        // can be read off and pinned deliberately rather than frozen at whatever today gives.
        assert!(
            anchor_mag_err < 1e-3,
            "at radius 1.0 the gradient magnitude must be 1 to within 1e-3 (a distance \
             function's gradient is a unit vector by definition) — got an error of \
             {anchor_mag_err:.3e}"
        );
    }

    /// `impl Sdf for CachedGridSdf` — `eval` reads the cached signed
    /// grid directly; `grad` central-differences the trilinear
    /// interpolant. Verifies the adapter honors the [`Sdf`] sign
    /// convention end-to-end on a known fixture (signed-distance
    /// negative interior, positive below the bottom face, outward
    /// gradient pointing -z below).
    #[allow(clippy::expect_used)]
    #[test]
    fn cached_grid_sdf_adapter_honors_sdf_sign_convention() {
        let distance = crate::TriMeshDistance::new(unit_tetrahedron())
            .expect("tetrahedron fixture has four faces");
        let bounds =
            mesh_types::Aabb::new(Point3::new(-1.0, -1.0, -1.5), Point3::new(2.0, 2.0, 2.0));
        let (cached, report) = crate::CachedGridSdf::build(
            &distance,
            bounds,
            0.1,
            crate::WALL_THRESHOLD_FACTOR_DEFAULT,
        )
        .expect("CachedGridSdf builds for the tetrahedron fixture");
        assert_eq!(report.inside_components, 1);

        // Interior probe (negative).
        let interior = Point3::new(0.5, 0.289, 0.4);
        assert!(Sdf::eval(&cached, interior) < 0.0);

        // Exterior probe below the bottom face — gradient should
        // point predominantly -z (outward from the body).
        let below = Point3::new(0.5, 0.289, -1.0);
        assert!(Sdf::eval(&cached, below) > 0.0);
        let g = Sdf::grad(&cached, below);
        assert!(
            g.z < -0.5,
            "below-bottom-face gradient should point -z, got {g:?}"
        );
    }
}
