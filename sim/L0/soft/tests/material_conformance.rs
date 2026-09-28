//! F3 (soft-contact recon §12, §14a, §16w): `sim-soft`'s material impls against the explicit solver's shared math.
//!
//! The explicit solver evaluates `sim-soft`'s compressible Yeoh (neo-Hookean at `C₂ = 0`) through its own shared
//! math, written in the displacement gradient `H = F − I` so that a quantity zero at rest is never a difference of
//! two numbers near one (`sim_soft_explicit::f64`'s `material.rs`). These tests pin that the two are one law: the
//! strain energy and the first Piola–Kirchhoff stress agree at f64 from near rest out to principal stretches below 0.2
//! and above 2 (asserted), rotated or not, and both call an element invalid exactly where `det F ≤ 0`. The stress is
//! compared whole and as the executor runs it, split into the μ terms per element and the λ term's pressure.
//!
//! The bar is absolute, a millionth of a millionth of the moduli, scaled by how far `F` is from zero: `sim-soft`
//! forms `F − F⁻ᵀ`, `ln det F` and `I₁ − 3` directly, so near rest its rounding is that of the moduli, not of the
//! small stress. A dropped term reads far past it (`a_dropped_yeoh_term_is_seen`).

use nalgebra::Matrix3;
use sim_soft::{InversionHandling, Material, NeoHookean, Yeoh};
use sim_soft_explicit::f64 as shared;

/// Deterministic `SplitMix64`.
struct Rng(u64);

impl Rng {
    const fn bits(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform on [−1, 1).
    #[allow(clippy::cast_precision_loss)] // >>11 leaves 53 bits: exact in f64.
    fn sym(&mut self) -> f64 {
        2.0f64.mul_add((self.bits() >> 11) as f64 / (1u64 << 53) as f64, -1.0)
    }

    /// A rotation: a random unit axis turned by up to π.
    fn rotation(&mut self) -> Matrix3<f64> {
        let axis = nalgebra::Vector3::new(self.sym(), self.sym(), self.sym());
        let axis = nalgebra::Unit::try_new(axis, 1e-3).unwrap_or_else(nalgebra::Vector3::z_axis);
        *nalgebra::Rotation3::from_axis_angle(&axis, std::f64::consts::PI * self.sym()).matrix()
    }
}

/// Materials: (μ, λ, C₂) in Pa. A near-incompressible neo-Hookean (ν 0.4995), a Yeoh at the product's ν 0.49, and a
/// Yeoh at the silicone catalog's λ = 4μ (ν 0.4).
const MATERIALS: [(f64, f64, f64); 3] = [
    (3.0e4, 2.0 * 3.0e4 * 0.4995 / 0.001, 0.0),
    (1.2e5, 2.0 * 1.2e5 * 0.49 / 0.02, 4.0e3),
    (5.0e4, 4.0 * 5.0e4, 1.5e3),
];

/// How far each drawn `F` is from a rotation: the entries of `S` in `Q (I + S)` are uniform on ±scale.
const SCALES: [f64; 5] = [1e-3, 1e-2, 1e-1, 0.3, 0.6];

/// Draws per material, scale and rotation.
const DRAWS: usize = 200;

/// `F` as the shared math's displacement gradient: `F − I`, row-major.
fn displacement_gradient(f: &Matrix3<f64>) -> [f64; 9] {
    std::array::from_fn(|k| f[(k / 3, k % 3)] - if k / 3 == k % 3 { 1.0 } else { 0.0 })
}

/// The shared math's material.
const fn shared_material(mu: f64, lambda: f64, c2: f64) -> shared::Material {
    shared::Material {
        mu,
        lambda,
        c2,
        viscosity: 0.0,
        density: 1.0,
    }
}

/// The deformation gradients drawn: at each scale, [`DRAWS`] of `Q (I + S)` with `S`'s entries uniform on ±scale,
/// `Q` the identity and then a random rotation, each with `det F` at least 0.05.
fn gradients() -> Vec<Matrix3<f64>> {
    let mut rng = Rng(0x0F3_C0DE);
    let mut out = Vec::new();
    for scale in SCALES {
        for rotated in [false, true] {
            let mut drawn = 0;
            while drawn < DRAWS {
                let stretch = Matrix3::from_fn(|_, _| scale * rng.sym()) + Matrix3::identity();
                let turn = if rotated {
                    rng.rotation()
                } else {
                    Matrix3::identity()
                };
                let f = turn * stretch;
                if f.determinant() >= 0.05 {
                    out.push(f);
                    drawn += 1;
                }
            }
        }
    }
    out
}

/// The bar at `f` for a material with moduli `(μ, λ, C₂)`.
fn bar(f: &Matrix3<f64>, (mu, lambda, c2): (f64, f64, f64)) -> f64 {
    let size = f.norm_squared();
    1e-12 * (mu + lambda + c2.abs()) * (1.0 + size) * (1.0 + size)
}

/// The stress the executor runs at a uniform dilation: the μ terms per element, and the λ term as its pressure times
/// `cof F` (selective averaged nodal pressure, which at one dilation is the element's own).
fn split_stress(h: [f64; 9], material: shared::Material) -> [f64; 9] {
    let pressure = shared::pressure_lambda_term(shared::gradient_dilation(h), material.lambda);
    shared::mat3_add(
        shared::first_piola_mu_terms(h, material),
        shared::mat3_scale(shared::deformation_cofactor(h), pressure),
    )
}

/// The largest excess over the bar of the energy's difference, the whole stress's and the split stress's, per law.
fn worst_excess(law: &dyn Material, moduli: (f64, f64, f64), gradients: &[Matrix3<f64>]) -> f64 {
    let material = shared_material(moduli.0, moduli.1, moduli.2);
    let mut worst = 0.0_f64;
    for f in gradients {
        let h = displacement_gradient(f);
        let energy = (law.energy(f) - shared::energy_density(h, material)).abs();
        let expected = law.first_piola(f);
        let off = |stress: [f64; 9]| {
            (0..9).fold(0.0_f64, |most, k| {
                most.max((expected[(k / 3, k % 3)] - stress[k]).abs())
            })
        };
        let stress = off(shared::first_piola(h, material)).max(off(split_stress(h, material)));
        worst = worst.max(energy.max(stress) / bar(f, moduli));
    }
    worst
}

#[test]
fn yeohs_energy_and_stress_are_the_shared_maths() {
    let gradients = gradients();
    assert_eq!(gradients.len(), SCALES.len() * 2 * DRAWS);
    // The principal stretches drawn: the singular values of F.
    let (least, most) = gradients
        .iter()
        .fold((f64::INFINITY, 0.0_f64), |(low, high), f| {
            let stretches = f.singular_values();
            (low.min(stretches.min()), high.max(stretches.max()))
        });
    eprintln!("MARGIN principal stretches drawn from {least:.3} to {most:.3}");
    assert!(
        least < 0.2 && most > 2.0,
        "the draws must reach from near-flat to twice as long"
    );
    for moduli in MATERIALS {
        let law = Yeoh::from_lame_and_c2(moduli.0, moduli.1, moduli.2);
        let excess = worst_excess(&law, moduli, &gradients);
        eprintln!("MARGIN Yeoh {moduli:?}: worst difference {excess:e} of the bar");
        assert!(
            excess <= 1.0,
            "Yeoh {moduli:?}: the energy or stress differs from the shared math's by {excess:e} of the bar"
        );
    }
}

#[test]
fn neo_hookeans_energy_and_stress_are_the_shared_maths_at_zero_c2() {
    let gradients = gradients();
    for (mu, lambda, _) in MATERIALS {
        let law = NeoHookean::from_lame(mu, lambda);
        let excess = worst_excess(&law, (mu, lambda, 0.0), &gradients);
        assert!(
            excess <= 1.0,
            "neo-Hookean ({mu}, {lambda}): the energy or stress differs from the shared math's by {excess:e} of \
             the bar"
        );
    }
}

/// A wrong term is not within the bar: `C₂` dropped from the shared material reads far past it wherever the strain
/// is not tiny.
#[test]
fn a_dropped_yeoh_term_is_seen() {
    let gradients: Vec<_> = gradients()
        .into_iter()
        .filter(|f| (f.transpose() * f - Matrix3::identity()).norm() > 1e-2)
        .collect();
    let (mu, lambda, c2) = MATERIALS[1];
    let law = Yeoh::from_lame_and_c2(mu, lambda, c2);
    let excess = worst_excess(&law, (mu, lambda, 0.0), &gradients);
    assert!(excess > 1e3, "C₂ dropped reads only {excess:e} of the bar");
}

#[test]
fn both_call_an_element_invalid_exactly_where_det_f_is_not_positive() {
    for law in [
        &Yeoh::from_lame_and_c2(1.0, 1.0, 1.0) as &dyn Material,
        &NeoHookean::from_lame(1.0, 1.0),
    ] {
        assert_eq!(
            law.validity().inversion,
            InversionHandling::RequireOrientation
        );
    }
    // Flat elements, `det F` exactly 0: a side squashed to nothing, and two rows equal.
    for f in [
        Matrix3::from_diagonal(&nalgebra::Vector3::new(1.0, 1.0, 0.0)),
        Matrix3::new(1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0),
    ] {
        assert!(
            shared::is_inverted(shared::gradient_dilation(displacement_gradient(&f))),
            "a flat element must read invalid: {f}"
        );
    }
    let mut rng = Rng(0x1_4B7E);
    let (mut inverted, mut upright) = (0, 0);
    for _ in 0..20_000 {
        let f = Matrix3::from_fn(|_, _| rng.sym()) + Matrix3::identity();
        let det = f.determinant();
        // Away from a flat element, where the two ways of forming det F may round to opposite signs.
        if det.abs() < 1e-9 {
            continue;
        }
        let flagged = shared::is_inverted(shared::gradient_dilation(displacement_gradient(&f)));
        assert_eq!(flagged, det <= 0.0, "det F {det}");
        if flagged {
            inverted += 1;
        } else {
            upright += 1;
        }
    }
    assert!(
        inverted > 1_000 && upright > 1_000,
        "{inverted} inverted, {upright} upright"
    );
}
