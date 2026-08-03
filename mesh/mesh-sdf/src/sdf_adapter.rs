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
    /// ## Measured — and the band is about **one decade wide**
    ///
    /// | radius | h/radius | max ‖g‖−1 | max ang° | |
    /// |---|---|---|---|---|
    /// | 1e-3 | 1.0e-3 | **8.7e2** | **179.94** | sign broken: triangles under the area floor |
    /// | 1e-2 | 1.0e-4 | 5.25e-4 | 2.92 | **best** |
    /// | 5e-2 | 2.0e-5 | 2.17e-3 | 2.92 | *the FSU disc's own metre-frame extent* |
    /// | 1e-1 | 1.0e-5 | 5.61e-3 | 2.93 | |
    /// | 1e0 | 1.0e-6 | 3.80e-2 | 4.83 | |
    /// | 1e1 | 1.0e-7 | 5.41e-1 | 23.95 | |
    /// | 1e2 | 1.0e-8 | 2.86e0 | 114.23 | |
    /// | 1e3 | 1.0e-9 | 2.62e1 | 129.93 | |
    ///
    /// **Two failure modes, one at each end.** Below the band the *area floor* kills the sign,
    /// and because `grad` central-differences a **signed** value a sign flip between the two
    /// probes yields a spurious gradient of order `φ/h` — the 179.94° is the gradient pointing
    /// backwards. Above the band the fixed step is swamped by the `f32` projection error,
    /// growing **linearly in `E`** exactly as predicted (≈ ×10 per decade).
    ///
    /// ⚠ **The FSU disc's metre-frame extent (0.0502) sits inside this band by accident** —
    /// nobody chose it; it is where `scale = 1e-3` happened to land. That also means every
    /// endplate projection in the arc carries a few × 1e-3 of gradient error, which had never
    /// been measured.
    ///
    /// ⚠⚠ **This is why normalising mesh coordinates internally is not sufficient on its own.**
    /// A target extent chosen to clear the area floor with margin (≥ 4, per the disc's R1
    /// sweep) lands `grad` at ~19 % error. The two constants have to be made *dimensionless* —
    /// `h` relative first, normalisation second — or fixing one breaks the other.
    ///
    /// ## What is asserted, and what deliberately is not
    ///
    /// The anchor is **1e-2**, the measured best, at 2× headroom. The large-`E` rows are
    /// **not** pinned by magnitude: they are a defect this crate intends to remove, and
    /// freezing their values would document the bug as intended behaviour. Instead the
    /// **presence** of the defect is asserted — so when `h` becomes relative that assert fires,
    /// and whoever fixes it is forced to restate this regime rather than quietly widen it.
    ///
    /// ⚠ An earlier version anchored at radius 1.0 on the grounds that `sdf.rs`'s sibling gate
    /// covers it. It does — for the **sign**. The sign and gradient regimes are different
    /// bands, and 1.0 is comfortable for one while already 3.8 % wrong for the other.
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
        let (mut anchor_mag_err, mut disc_mag_err, mut large_e_mag_err) =
            (f64::NAN, f64::NAN, f64::NAN);
        // 0.05 is the FSU disc's own extent once `prepare_disc_at` has rescaled it to metres.
        for radius in [1e-3_f64, 1e-2, 5e-2, 1e-1, 1.0, 10.0, 100.0, 1000.0] {
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
            if (radius - 1e-2).abs() < 1e-12 {
                anchor_mag_err = max_mag;
            }
            if (radius - 5e-2).abs() < 1e-12 {
                disc_mag_err = max_mag;
            }
            if (radius - 100.0).abs() < 1e-12 {
                large_e_mag_err = max_mag;
            }
        }

        // (1) THE ANCHOR — the measured best of the band, at ~2x headroom. A distance
        //     function's gradient is a unit vector by definition, so this is the floor of what
        //     the adapter must deliver anywhere it is usable at all.
        assert!(
            anchor_mag_err < 1e-3,
            "at radius 1e-2 the gradient magnitude must be 1 to within 1e-3 — got \
             {anchor_mag_err:.3e}"
        );

        // (2) THE REGIME THE FSU DISC ACTUALLY RUNS IN. Its metre-frame extent is 0.0502, and
        //     every endplate projection in that arc walks points along this gradient. Pinned
        //     loosely (1e-2) because the point is that it is USABLE, not that it is precise.
        assert!(
            disc_mag_err < 1e-2,
            "at radius 5e-2 — the FSU disc's own metre-frame extent — the gradient must still \
             be usable, got {disc_mag_err:.3e}"
        );

        // (3) ⚠ THE DEFECT, ASSERTED AS PRESENT RATHER THAN PINNED BY VALUE.
        //     A fixed absolute step cannot survive large coordinates, and today it does not.
        //     This is deliberately NOT a magnitude pin: freezing 2.86e0 would document the bug
        //     as intended behaviour. Asserting only that the defect EXISTS means that when `h`
        //     is made relative to the geometry, THIS ASSERT FAILS — forcing whoever fixes it to
        //     come here and restate the regime instead of quietly widening a tolerance.
        assert!(
            large_e_mag_err > 1.0,
            "at radius 1e2 the fixed 1e-6 step is expected to be swamped by f32 projection \
             error (measured 2.86e0), but the error is now {large_e_mag_err:.3e}. If `grad`'s \
             step has been made relative to the geometry, this test has done its job — delete \
             this assert and re-measure the whole band, do NOT relax it."
        );
    }

    // ── α.0: is remedy D independent of remedy F? ────────────────────────────────────
    //
    // `project-mesh-sdf-trust-arc` carries "★★ ORDER IS FORCED: F then D" on the
    // grounds that normalising to clear the area floor puts `grad` at ~19 % error. The
    // three tests below exist to check that, because the claim was measured on an
    // adapter (`cf_fsu_model`'s test-only `NormalisedOracle`) that queries the inner
    // oracle **in the scaled frame** — and remedy D does not.

    /// Scale every vertex of `m` by exactly `s`. Mirrors `sdf.rs`'s helper of the same
    /// name; duplicated rather than shared because the two modules pin different
    /// properties and a shared fixture would couple them.
    fn scaled(m: &mesh_types::IndexedMesh, s: f64) -> mesh_types::IndexedMesh {
        let mut out = m.clone();
        for v in &mut out.vertices {
            *v = mesh_types::Point3::new(v.x * s, v.y * s, v.z * s);
        }
        out
    }

    /// Smallest triangle area in `m` — the quantity parry's `DEFAULT_EPSILON / 2` area
    /// floor is applied to.
    fn min_triangle_area(m: &mesh_types::IndexedMesh) -> f64 {
        m.faces
            .iter()
            .map(|f| {
                let (a, b, c) = (
                    m.vertices[f[0] as usize],
                    m.vertices[f[1] as usize],
                    m.vertices[f[2] as usize],
                );
                0.5 * (b - a).cross(&(c - a)).norm()
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// **Remedy D, expressed exactly as it would land inside `mesh-sdf`.**
    ///
    /// The mesh is scaled by `2^k` at *construction*; every query is lifted into that
    /// internal frame and the answer brought back. The load-bearing property is what
    /// this adapter does **not** do: it never changes the frame the caller queries in.
    ///
    /// Implemented as an [`UnsignedDistance`] + [`Sign`] pair rather than as a direct
    /// [`Sdf`] impl on purpose — composing them through [`Signed`] means the blanket
    /// `impl Sdf for Signed<D, S>` **under test** supplies `grad`, unmodified. A
    /// hand-written `grad` here would be a copy of the code it is meant to measure.
    struct NormalisedDistance {
        inner: crate::TriMeshDistance,
        s: f64,
    }

    impl NormalisedDistance {
        fn lift(p: Point3<f64>, s: f64) -> Point3<f64> {
            Point3::new(p.x * s, p.y * s, p.z * s)
        }
    }

    impl UnsignedDistance for NormalisedDistance {
        fn distance(&self, p: Point3<f64>) -> f64 {
            self.inner.distance(Self::lift(p, self.s)) / self.s
        }
        fn closest_point(&self, p: Point3<f64>) -> Point3<f64> {
            let q = self.inner.closest_point(Self::lift(p, self.s));
            Point3::new(q.x / self.s, q.y / self.s, q.z / self.s)
        }
    }

    /// The sign half of the same internal normalisation — shares the scaled BVH with
    /// [`NormalisedDistance`] the way `PseudoNormalSign::from_distance` shares the
    /// unscaled one.
    struct NormalisedSign {
        inner: crate::PseudoNormalSign,
        s: f64,
    }

    impl Sign for NormalisedSign {
        fn is_inside(&self, p: Point3<f64>) -> bool {
            self.inner.is_inside(NormalisedDistance::lift(p, self.s))
        }
    }

    /// Today's composition: oracle built and queried in the caller's frame.
    #[allow(clippy::expect_used)]
    fn plain_sdf(
        mesh: &mesh_types::IndexedMesh,
    ) -> Signed<crate::TriMeshDistance, crate::PseudoNormalSign> {
        let distance = crate::TriMeshDistance::new(mesh.clone()).expect("fixture is non-empty");
        let sign = crate::PseudoNormalSign::from_distance(&distance);
        Signed { distance, sign }
    }

    /// Remedy D's composition: oracle built on the mesh times `s`, queried in the
    /// caller's frame.
    #[allow(clippy::expect_used)]
    fn normalised_sdf(
        mesh: &mesh_types::IndexedMesh,
        s: f64,
    ) -> Signed<NormalisedDistance, NormalisedSign> {
        let inner = crate::TriMeshDistance::new(scaled(mesh, s)).expect("fixture is non-empty");
        let sign = NormalisedSign {
            inner: crate::PseudoNormalSign::from_distance(&inner),
            s,
        };
        Signed {
            distance: NormalisedDistance { inner, s },
            sign,
        }
    }

    /// The 64 deterministic probe directions `grad_step_scale_regime` uses — same
    /// spread, no RNG, no coincidence with the sphere's own tessellation seams.
    fn probe_dirs() -> Vec<Vector3<f64>> {
        (0..64)
            .map(|i| {
                let z = 2.0f64.mul_add(f64::from(i) / 63.0, -1.0) * 0.98;
                let phi = f64::from(i) * 2.399_963_229_728_653; // golden angle
                let r = (1.0 - z * z).sqrt();
                Vector3::new(r * phi.cos(), r * phi.sin(), z).normalize()
            })
            .collect()
    }

    /// Largest `|‖∇φ‖ − 1|` over the probe set at `1.5·radius` — the instrument
    /// `grad_step_scale_regime` established. A polyhedron's signed distance is a genuine
    /// distance function, so any departure from 1 is numerical and nothing else.
    fn max_grad_magnitude_error<S: Sdf>(sdf: &S, radius: f64, dirs: &[Vector3<f64>]) -> f64 {
        dirs.iter().fold(0.0_f64, |acc, d| {
            let p = Point3::from(d * (1.5 * radius));
            acc.max((Sdf::grad(sdf, p).norm() - 1.0).abs())
        })
    }

    /// **α.0 — the arc's "F must precede D" is refuted, and this is the measurement.**
    ///
    /// ## The claim being tested
    ///
    /// `project-mesh-sdf-trust-arc` records "★★ ORDER IS FORCED: F then D", reasoning
    /// that a target extent chosen to clear the area floor lands `grad` at ~19 % error.
    /// That number came from `cf_fsu_model`'s test-only `NormalisedOracle`, whose own
    /// doc comment says what it does: it queries the inner oracle **in the scaled
    /// frame**, so the effective step becomes `1e-6 / s` *in that frame*.
    ///
    /// Remedy D is not that. D normalises **internally** — scale at construction,
    /// unscale at query — so the frame the caller queries in never moves. `Sdf::grad`
    /// still central-differences at `h = 1e-6` in caller units; the `f32` projection
    /// error is still `~6e-8·E·s` internally and `~6e-8·E` after the unscale. Both
    /// terms of the gradient's error budget are unchanged, so D should be **exactly**
    /// neutral for `grad` — not approximately, bit for bit.
    ///
    /// ## What is asserted
    ///
    /// Componentwise `to_bits` equality between today's oracle and the internally
    /// normalised one, at every radius in the band `grad_step_scale_regime` measured,
    /// for two non-trivial scales. Bit-identity is the right strength here: an
    /// approximate assert would pass for a normalisation that perturbs results
    /// slightly, and "does not perturb results" is the entire claim.
    ///
    /// ⚠ **This includes the radii where `grad` is already badly wrong** (1e1, 1e2 carry
    /// 5.4e-1 and 2.9e0 of magnitude error today). Bit-identity there says the
    /// normalised oracle is **equally wrong**, which is the honest statement of the
    /// finding: **D neither helps nor hurts `grad`.** It does not license shipping D as
    /// a gradient fix — F remains owed. It licenses shipping them in either order.
    #[test]
    fn internal_normalisation_leaves_grad_bit_identical() {
        let dirs = probe_dirs();
        let mut checked = 0usize;

        println!(
            "\n{:>10} {:>14} {:>14} {:>10}",
            "radius", "plain max err", "norm max err", "bit-equal"
        );
        for radius in [1e-2_f64, 5e-2, 1e-1, 1.0, 10.0, 100.0] {
            let mesh = crate::test_fixtures::uv_sphere(radius, 24, 48);
            let plain = plain_sdf(&mesh);
            let plain_err = max_grad_magnitude_error(&plain, radius, &dirs);

            for k in [3_i32, 13] {
                let s = 2.0_f64.powi(k);
                let norm = normalised_sdf(&mesh, s);
                for d in &dirs {
                    let p = Point3::from(d * (1.5 * radius));
                    let gp = Sdf::grad(&plain, p);
                    let gn = Sdf::grad(&norm, p);
                    for (axis, (a, b)) in [(gp.x, gn.x), (gp.y, gn.y), (gp.z, gn.z)]
                        .into_iter()
                        .enumerate()
                    {
                        assert_eq!(
                            a.to_bits(),
                            b.to_bits(),
                            "r={radius}, 2^{k}, axis {axis} at {p:?}: internal normalisation \
                             moved the gradient — plain {a:.17e}, normalised {b:.17e}. Then D is \
                             NOT neutral for `grad` and the arc's forced F-then-D order stands."
                        );
                    }
                    checked += 1;
                }
                let norm_err = max_grad_magnitude_error(&norm, radius, &dirs);
                println!(
                    "{radius:>10.0e} {plain_err:>14.3e} {norm_err:>14.3e} {:>10}",
                    "yes"
                );
            }
        }

        // Non-vacuity: 6 radii × 2 scales × 64 probes.
        assert!(
            checked >= 700,
            "only {checked} probe comparisons ran — the sweep collapsed and this test proves \
             nothing"
        );
    }

    /// **α.0's second half: D fixes what it is for, and the fix is visible in `grad`.**
    ///
    /// At radius 1e-3 this fixture's smallest triangles sit under parry's area floor, so
    /// the pseudo-normal is zeroed and the sign reads "inside" at any distance.
    /// `grad` central-differences a **signed** value, so a flip between the two probes
    /// yields a spurious gradient of order `φ/h` — the 8.7e2 / 179.94° row of
    /// `grad_step_scale_regime`.
    ///
    /// Internal normalisation lifts the mesh clear of the floor, and the scale is
    /// **derived from the mesh's own smallest triangle** rather than fixed — which is
    /// what makes D's constant dimensionless. `MARGIN` is deliberately generous and is
    /// **not** the shipping value: R1's disc sweep showed targets that leave the
    /// smallest triangle at 0.05× and 0.74× of the floor still meshing correctly, so
    /// margin — not success — is the criterion, and choosing it is α.1's job.
    #[test]
    #[allow(clippy::cast_possible_truncation)]
    fn internal_normalisation_repairs_the_sub_floor_grad() {
        const MARGIN: f64 = 1.0e4;
        let dirs = probe_dirs();
        let radius = 1e-3;
        let mesh = crate::test_fixtures::uv_sphere(radius, 24, 48);

        let floor = f64::from(f32::EPSILON) / 2.0;
        let min_area = min_triangle_area(&mesh);
        // Precondition, checked rather than assumed: this radius really is in the broken
        // regime. If the fixture ever changes so it is not, the repair assert below would
        // pass vacuously.
        assert!(
            min_area < floor,
            "r={radius}: fixture's smallest triangle ({min_area:.3e}) already clears the floor \
             ({floor:.3e}) — this test is measuring nothing"
        );

        // s ≥ sqrt(MARGIN · floor / min_area), rounded up to a power of two so the rescale
        // is a pure exponent shift.
        let k = (MARGIN * floor / min_area).sqrt().log2().ceil();
        let s = 2.0_f64.powi(k as i32);
        let norm = normalised_sdf(&mesh, s);
        let plain = plain_sdf(&mesh);

        let plain_err = max_grad_magnitude_error(&plain, radius, &dirs);
        let norm_err = max_grad_magnitude_error(&norm, radius, &dirs);
        println!(
            "\nr={radius:.0e}  min_area {min_area:.3e} vs floor {floor:.3e}  ->  2^{k} = {s}\n\
             plain max |‖g‖-1| {plain_err:.3e}   normalised {norm_err:.3e}"
        );

        assert!(
            plain_err > 1.0,
            "at r={radius} today's oracle is expected to be sign-broken (measured 8.7e2), got \
             {plain_err:.3e} — if the area floor no longer bites here, re-measure the whole band"
        );
        // Predicted before running: the surviving error is the f32 projection term,
        // ~6e-8·E/(2h) ≈ 4.5e-5 at E ≈ 1.5e-3. Asserted at 1e-3 — 20x headroom over the
        // prediction, and still three decades below the broken value it replaces.
        assert!(
            norm_err < 1e-3,
            "internal normalisation should put r={radius} back inside the usable band, got \
             {norm_err:.3e}"
        );
    }

    /// **α.0's control — and the half that explains where the arc's 19 % came from.**
    ///
    /// Without this, the bit-identity above would be consistent with "scaling a mesh
    /// does not affect `grad`", which is false and would make the first test prove
    /// nothing specific.
    ///
    /// This arm rescales the **caller's frame**: the oracle is built on the mesh times
    /// `s` *and queried at* `p·s`, which is what a caller-side rescale (remedy A) or
    /// `cf_fsu_model`'s `NormalisedOracle` does. The geometry is identical; only the
    /// coordinate magnitude the fixed `1e-6` step is measured against has changed, and
    /// `grad` degrades ≈ ×10 per decade exactly as `grad_step_scale_regime` predicts.
    ///
    /// **The two arms differ in one thing only — the frame the caller queries in.** That
    /// is the distinction the "F then D" ordering missed.
    #[test]
    fn caller_frame_rescale_moves_grad_where_internal_normalisation_does_not() {
        let dirs = probe_dirs();
        let radius = 1.0;
        let s = 1024.0; // 2^10
        let mesh = crate::test_fixtures::uv_sphere(radius, 24, 48);

        let plain = plain_sdf(&mesh);
        let plain_err = max_grad_magnitude_error(&plain, radius, &dirs);

        // Same geometry, but the caller now works in the scaled frame.
        let rescaled = plain_sdf(&scaled(&mesh, s));
        let rescaled_err = max_grad_magnitude_error(&rescaled, radius * s, &dirs);

        println!(
            "\ncaller-frame rescale by {s}: plain max |‖g‖-1| {plain_err:.3e}  ->  \
             {rescaled_err:.3e}"
        );

        assert!(
            rescaled_err > 10.0 * plain_err,
            "a 2^10 caller-frame rescale must visibly degrade `grad` (the fixed 1e-6 step is \
             now 1024x smaller relative to the geometry) — plain {plain_err:.3e}, rescaled \
             {rescaled_err:.3e}. If it does not, the bit-identity result above is consistent \
             with `grad` being scale-insensitive in general and proves nothing specific."
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
