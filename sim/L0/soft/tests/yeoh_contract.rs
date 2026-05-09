//! Yeoh contract — math cross-checks for the production
//! [`sim_soft::Yeoh`] struct.
//!
//! Closed forms documented in `material/yeoh.rs` and the Yeoh arc memo
//! §"Math derivations":
//!
//! - `ψ = C₁(I₁ − 3) + C₂(I₁ − 3)² − μ ln J + (λ/2)(ln J)²`
//! - `P = μ(F − F⁻ᵀ) + λ (ln J) F⁻ᵀ + 4 C₂ (I₁ − 3) F`
//! - `C_ijkl = C_NH_ijkl + 4 C₂ (I₁ − 3) δ_ik δ_jl + 8 C₂ F_ij F_kl`
//!
//! Cross-checks (Yeoh arc memo §"Validation cascade"):
//!
//! 1. **NH bit-exact at C₂=0** — every Yeoh output equals NH on a
//!    battery of `F`. Validates the locked D2 contract; the production
//!    impl uses the additive-decomposition + FMA pattern (Spike 1
//!    finding) load-bearing for bit-exactness.
//! 2. **Hand-derived scalar uniaxial closed-form** — for `F = diag(s,1,1)`,
//!    matrix-form `ψ`, `P_11`, `P_22` match the scalar closed-form
//!    derived by hand-substituting the diagonal `F`. Catches
//!    transcription bugs.
//! 3. **FD ψ → P** — central FD of `energy` matches `first_piola` at
//!    the nontrivial off-diagonal-rich `F` from `material_fd.rs`.
//! 4. **FD P → tangent** — central FD of `first_piola` matches
//!    `tangent` at the same point.
//! 5. **Rest-config vanishing** — `ψ(I) = 0` and `P(I) = 0` for every
//!    silicone anchor's Yeoh; textbook hyperelastic constraint.
//!
//! Companion checks living elsewhere:
//!
//! - **Calibration arithmetic** — `M_100 = 3.5·C₁ + 14·C₂` per
//!   anchor: `material/silicone_table.rs::tests::c2_calibration_reproduces_published_100_pct_modulus`.
//! - **Validity bound population** — `to_yeoh()` round-trips
//!   `(max, min) = (0.8·λ_break, 0.30)`:
//!   `silicone_table.rs::tests::to_yeoh_round_trips_yeoh_fields_for_each_anchor`.
//! - **`λ_break` ↔ `ε_break` ASTM-D412 conversion + 0.8 rupture margin**
//!   — pins `validity_max = 0.8 · (1 + ε_break / 100)` per anchor:
//!   `silicone_table.rs::tests::validity_max_pins_to_80_pct_of_one_plus_elongation_at_break`.
//! - **Solver-side gate trip behavior** — driven via row-23 (F4)
//!   integration scenarios; out of scope for this contract file.

mod generic_mesh_smoke {
    //! Smoke test that the F4.0 generic-mesh refactor actually
    //! instantiates `SingleTetMesh<Yeoh>` end-to-end (not just
    //! type-checks via the lib aliases).
    //!
    //! The unit tests in `material_field_sample.rs` cover
    //! `MaterialField::from_yeoh_fields` + `sample_yeoh` in isolation;
    //! this test exercises the full mesh-build path through
    //! `BuildableFromField::cache_from_field` and
    //! `Mesh<Yeoh>::materials()`.

    use sim_soft::{
        ConstantField, Field, Material, MaterialField, Mesh, SingleTetMesh, Yeoh,
        material::silicone_table::ECOFLEX_00_30,
    };

    /// Build a `SingleTetMesh<Yeoh>` from a uniform Yeoh field
    /// (`ECOFLEX_00_30` calibration), confirm the mesh's `materials()`
    /// reads back as `&[Yeoh]`, and that the constructed Yeoh struct
    /// at the mesh's centroid round-trips the input field's `(μ, λ, c2)`
    /// scalars (within FMA-FP noise — sample uses the same arithmetic
    /// the centroid path traverses).
    #[test]
    fn single_tet_mesh_yeoh_materials_round_trip() {
        let mu_field: Box<dyn Field<f64>> = Box::new(ConstantField::new(ECOFLEX_00_30.mu));
        let c2_field: Box<dyn Field<f64>> = Box::new(ConstantField::new(ECOFLEX_00_30.c2));
        let lambda_field: Box<dyn Field<f64>> = Box::new(ConstantField::new(ECOFLEX_00_30.lambda));
        let field = MaterialField::from_yeoh_fields(mu_field, c2_field, lambda_field);

        let mesh: SingleTetMesh<Yeoh> = SingleTetMesh::<Yeoh>::new_yeoh(&field);
        assert_eq!(mesh.n_tets(), 1);

        let materials: &[Yeoh] = mesh.materials();
        assert_eq!(materials.len(), 1);
        assert_eq!(materials[0].mu().to_bits(), ECOFLEX_00_30.mu.to_bits());
        assert_eq!(
            materials[0].lambda().to_bits(),
            ECOFLEX_00_30.lambda.to_bits()
        );
        assert_eq!(materials[0].c2().to_bits(), ECOFLEX_00_30.c2.to_bits());
    }

    /// `Mesh<Yeoh>::materials()` returns Yeoh structs that satisfy the
    /// `Material` trait surface — `energy`/`first_piola`/`tangent` compile
    /// against the generic mesh without explicit type annotations.
    /// This is the load-bearing call site for the row-23 solver path.
    #[test]
    fn yeoh_mesh_materials_satisfy_material_trait() {
        use nalgebra::Matrix3;

        let mu_field: Box<dyn Field<f64>> = Box::new(ConstantField::new(ECOFLEX_00_30.mu));
        let c2_field: Box<dyn Field<f64>> = Box::new(ConstantField::new(ECOFLEX_00_30.c2));
        let lambda_field: Box<dyn Field<f64>> = Box::new(ConstantField::new(ECOFLEX_00_30.lambda));
        let field = MaterialField::from_yeoh_fields(mu_field, c2_field, lambda_field);
        let mesh = SingleTetMesh::<Yeoh>::new_yeoh(&field);

        let materials = mesh.materials();
        let f = Matrix3::<f64>::identity();
        // ψ(I) = 0, P(I) = 0 — already covered by suite (e), but doing
        // it through the mesh.materials() generic call site here
        // confirms the Mesh<Yeoh> -> Material trait dispatch works.
        assert_eq!(materials[0].energy(&f).to_bits(), 0.0_f64.to_bits());
        let p = materials[0].first_piola(&f);
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(p[(i, j)].to_bits(), 0.0_f64.to_bits());
            }
        }
    }
}

use approx::assert_relative_eq;
use nalgebra::{Matrix3, Vector3};

use sim_soft::{Material, NeoHookean, Yeoh};

// ECOFLEX_00_30 calibration per Yeoh arc memo line 81.
const MU: f64 = 23_000.0; // Pa
const LAMBDA: f64 = 92_000.0; // Pa (ν=0.40 convention: λ = 4μ)
const C2: f64 = 2_050.0; // Pa (Yeoh C₂)

// FD step + tolerances match `material_fd.rs` precedent (square-root of
// f64 eps, 5-digit relative bar with absolute slack covering FD noise at
// μ ≈ 10⁵).
const H: f64 = 1.5e-8;
const MAX_RELATIVE: f64 = 1.0e-5;
const EPSILON: f64 = 1.0e-3;

// Hand-derived scalar closed-forms for F = diag(s, 1, 1):
//
//   I₁ = s² + 2,  J = s,  ln J = ln s,  I₁ − 3 = s² − 1,  F⁻ᵀ = diag(1/s, 1, 1)
//
// substituted into the matrix formulas. Plain ops (no `mul_add`) so the
// test code mirrors the derivation literally — the rel-tol used in
// suite (b) is 1e-12, which absorbs any FMA/non-FMA ULP drift.
#[allow(clippy::suboptimal_flops)]
fn analytic_uniaxial_energy(s: f64, mu: f64, lambda: f64, c2: f64) -> f64 {
    let c1 = 0.5 * mu;
    let i1m3 = s * s - 1.0;
    let ln_s = s.ln();
    c1 * i1m3 + c2 * i1m3 * i1m3 - mu * ln_s + 0.5 * lambda * ln_s * ln_s
}

#[allow(clippy::suboptimal_flops)]
fn analytic_uniaxial_p11(s: f64, mu: f64, lambda: f64, c2: f64) -> f64 {
    let c1 = 0.5 * mu;
    let i1m3 = s * s - 1.0;
    let ln_s = s.ln();
    2.0 * s * (c1 + 2.0 * c2 * i1m3) + (lambda * ln_s - mu) / s
}

#[allow(clippy::suboptimal_flops)]
fn analytic_uniaxial_p22(s: f64, mu: f64, lambda: f64, c2: f64) -> f64 {
    let c1 = 0.5 * mu;
    let i1m3 = s * s - 1.0;
    let ln_s = s.ln();
    2.0 * (c1 + 2.0 * c2 * i1m3) + (lambda * ln_s - mu)
}

#[allow(clippy::missing_const_for_fn)]
fn nontrivial_f() -> Matrix3<f64> {
    // Same pose as `material_fd.rs::nontrivial_f` so the spike reuses the
    // off-diagonal-rich F that exercises every term of the closed-form
    // tangent. det F ≈ 0.987 > 0.
    Matrix3::new(
        1.10, 0.05, 0.02, //
        0.04, 0.90, 0.01, //
        0.03, 0.02, 1.00, //
    )
}

fn diag(s1: f64, s2: f64, s3: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(s1, s2, s3))
}

// ---- Suite (a): NH bit-exact at C₂=0 -----------------------------------

#[test]
fn yeoh_at_c2_zero_matches_neo_hookean_bit_exactly_on_battery() {
    let nh = NeoHookean::from_lame(MU, LAMBDA);
    let yeoh = Yeoh::from_lame_and_c2(MU, LAMBDA, 0.0);

    let battery = [
        Matrix3::<f64>::identity(),
        diag(1.01, 1.0, 1.0),
        diag(1.5, 1.0, 1.0),
        diag(1.0, 1.0, 1.0 / 1.05),
        nontrivial_f(),
    ];

    for f in &battery {
        let nh_e = nh.energy(f);
        let y_e = yeoh.energy(f);
        assert_eq!(
            nh_e.to_bits(),
            y_e.to_bits(),
            "energy mismatch at C₂=0: NH={nh_e}, Yeoh={y_e}, F={f:?}"
        );

        let nh_p = nh.first_piola(f);
        let y_p = yeoh.first_piola(f);
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(
                    nh_p[(i, j)].to_bits(),
                    y_p[(i, j)].to_bits(),
                    "first_piola[{i},{j}] mismatch at C₂=0"
                );
            }
        }

        let nh_t = nh.tangent(f);
        let y_t = yeoh.tangent(f);
        for r in 0..9 {
            for c in 0..9 {
                assert_eq!(
                    nh_t[(r, c)].to_bits(),
                    y_t[(r, c)].to_bits(),
                    "tangent[{r},{c}] mismatch at C₂=0"
                );
            }
        }
    }
}

// ---- Suite (b): hand-derived scalar uniaxial closed-form ---------------

#[test]
fn yeoh_uniaxial_matrix_form_matches_scalar_closed_form() {
    let yeoh = Yeoh::from_lame_and_c2(MU, LAMBDA, C2);

    // Stretches span tensile + compressive within Yeoh validity for
    // ECOFLEX_00_30 (max_principal_stretch = 8.0, min = 0.3).
    for s in [0.5, 0.8, 1.0, 1.2, 1.5, 2.0] {
        let f = diag(s, 1.0, 1.0);

        let e_matrix = yeoh.energy(&f);
        let e_scalar = analytic_uniaxial_energy(s, MU, LAMBDA, C2);
        assert_relative_eq!(e_matrix, e_scalar, max_relative = 1e-12, epsilon = 1e-9);

        let p_matrix = yeoh.first_piola(&f);
        let p11 = analytic_uniaxial_p11(s, MU, LAMBDA, C2);
        let p22 = analytic_uniaxial_p22(s, MU, LAMBDA, C2);
        assert_relative_eq!(p_matrix[(0, 0)], p11, max_relative = 1e-12, epsilon = 1e-9);
        assert_relative_eq!(p_matrix[(1, 1)], p22, max_relative = 1e-12, epsilon = 1e-9);
        assert_relative_eq!(p_matrix[(2, 2)], p22, max_relative = 1e-12, epsilon = 1e-9);

        for i in 0..3 {
            for j in 0..3 {
                if i != j {
                    assert!(
                        p_matrix[(i, j)].abs() < 1e-9,
                        "off-diagonal P[{i},{j}]={} at s={s}",
                        p_matrix[(i, j)]
                    );
                }
            }
        }
    }
}

// ---- Suite (c): FD ψ → P at nontrivial F -------------------------------

#[test]
fn yeoh_first_piola_matches_central_fd_of_energy() {
    let yeoh = Yeoh::from_lame_and_c2(MU, LAMBDA, C2);
    let f0 = nontrivial_f();
    let p = yeoh.first_piola(&f0);

    for i in 0..3 {
        for j in 0..3 {
            let mut f_pos = f0;
            f_pos[(i, j)] += H;
            let mut f_neg = f0;
            f_neg[(i, j)] -= H;
            let fd = (yeoh.energy(&f_pos) - yeoh.energy(&f_neg)) / (2.0 * H);
            assert_relative_eq!(
                fd,
                p[(i, j)],
                max_relative = MAX_RELATIVE,
                epsilon = EPSILON
            );
        }
    }
}

// ---- Suite (d): FD P → tangent at nontrivial F -------------------------

#[test]
fn yeoh_tangent_matches_central_fd_of_first_piola() {
    let yeoh = Yeoh::from_lame_and_c2(MU, LAMBDA, C2);
    let f0 = nontrivial_f();
    let tangent = yeoh.tangent(&f0);

    for k in 0..3 {
        for l in 0..3 {
            let mut f_pos = f0;
            f_pos[(k, l)] += H;
            let mut f_neg = f0;
            f_neg[(k, l)] -= H;
            let p_pos = yeoh.first_piola(&f_pos);
            let p_neg = yeoh.first_piola(&f_neg);
            let dp_dfkl = (p_pos - p_neg) / (2.0 * H);
            for i in 0..3 {
                for j in 0..3 {
                    let row = i + 3 * j;
                    let col = k + 3 * l;
                    assert_relative_eq!(
                        dp_dfkl[(i, j)],
                        tangent[(row, col)],
                        max_relative = MAX_RELATIVE,
                        epsilon = EPSILON
                    );
                }
            }
        }
    }
}

// ---- Suite (e): rest-config vanishing for each anchor -----------------

/// At `F = I` every Yeoh output is identically zero (textbook
/// constraint for any reasonable hyperelastic): I₁ = 3 so I₁ − 3 = 0,
/// J = 1 so ln J = 0, and `(F − F⁻ᵀ) = 0`. Pinned for every silicone
/// anchor's `to_yeoh()` so the rest-state ground truth stays bit-zero
/// across the table.
#[test]
fn yeoh_psi_and_p_vanish_at_identity_per_anchor() {
    use sim_soft::material::silicone_table::{
        DRAGON_SKIN_10A, DRAGON_SKIN_15, DRAGON_SKIN_20A, DRAGON_SKIN_30A, ECOFLEX_00_10,
        ECOFLEX_00_20, ECOFLEX_00_30, ECOFLEX_00_50,
    };
    let anchors = [
        ECOFLEX_00_10,
        ECOFLEX_00_20,
        ECOFLEX_00_30,
        ECOFLEX_00_50,
        DRAGON_SKIN_10A,
        DRAGON_SKIN_15,
        DRAGON_SKIN_20A,
        DRAGON_SKIN_30A,
    ];
    let id = Matrix3::<f64>::identity();
    for anchor in &anchors {
        let yeoh = anchor.to_yeoh();
        // ψ(I) = 0: every term collapses to zero in f64 (0·anything,
        // 0·0, etc.) so bit-exact match against +0.0 is the right
        // contract here, not a relative-tolerance check.
        assert_eq!(yeoh.energy(&id).to_bits(), 0.0_f64.to_bits());
        // P(I) = 0 matrix.
        let p = yeoh.first_piola(&id);
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(p[(i, j)].to_bits(), 0.0_f64.to_bits());
            }
        }
    }
}
