//! POD basis unit properties, on synthetic snapshots — no solver, milliseconds.
//!
//! Sibling of `reduced_pod_basis.rs`, which is the R1.0 *physics* gate and is
//! release-only (52 dynamic solves, ~21 s). That gate answers "does a subspace
//! represent this material's deformation"; this file answers "does the decomposition
//! do what a decomposition is supposed to do", and it runs in debug so a regression in
//! the linear algebra surfaces on an ordinary `cargo test` rather than only under
//! `--release`.
//!
//! The properties here are the ones whose violation is silent. A basis that is not
//! orthonormal, or that cannot reproduce its own training data at full rank, still
//! returns plausible-looking numbers — as the R1.0 pilot found when mass-weighted modes
//! were re-weighted after fitting and every projection collapsed to zero.

#![allow(
    // Fixtures build a rank-3 ensemble from small integer indices; every value is far
    // below 2^53, so the mantissa concern cannot arise.
    clippy::cast_precision_loss,
    // A fixture that fails to fit is a broken test, not a runtime condition to handle,
    // and the error-path tests assert on the variant `unwrap_err` returns.
    clippy::expect_used,
    clippy::unwrap_used
)]

use sim_soft::solver::backward_euler::reduced::{Inner, PodBasis, PodError, SnapshotSet};

const N_FREE: usize = 12;

/// Three fixed, linearly independent shapes; snapshots are combinations of them, so the
/// ensemble has exact rank 3 regardless of how many snapshots are taken.
fn shape(k: usize, i: usize) -> f64 {
    let x = i as f64 / N_FREE as f64;
    match k {
        0 => 1.0 + 0.1 * x,
        1 => (2.0 * std::f64::consts::PI * x).sin(),
        _ => (4.0 * std::f64::consts::PI * x).cos(),
    }
}

fn rank3_set() -> SnapshotSet {
    let mut s = SnapshotSet::new(N_FREE);
    for (a, b, c) in [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 0.5, -0.25),
        (-0.5, 2.0, 0.75),
        (0.3, -1.2, 0.9),
    ] {
        let u: Vec<f64> = (0..N_FREE)
            .map(|i| a * shape(0, i) + b * shape(1, i) + c * shape(2, i))
            .collect();
        s.push(&u);
    }
    s
}

/// Varied positive masses, so `Inner::Mass` is genuinely a different inner product from
/// `Inner::Euclidean` rather than a uniform rescaling that would hide a weighting bug.
fn masses() -> Vec<f64> {
    (0..N_FREE).map(|i| 0.5 + 0.7 * (i as f64)).collect()
}

#[test]
fn modes_are_orthonormal_in_their_own_inner_product() {
    for inner in [Inner::Euclidean, Inner::Mass] {
        let b = PodBasis::fit(&rank3_set(), inner, &masses(), 1.0, 3).expect("fit");
        for i in 0..b.n_modes() {
            let mut e = vec![0.0; b.n_modes()];
            e[i] = 1.0;
            // project(reconstruct(e)) is the i-th column of ΦᵀMΦ, which must be eᵢ.
            for (j, qj) in b.project(&b.reconstruct(&e)).iter().enumerate() {
                let want = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (qj - want).abs() < 1e-10,
                    "{inner:?}: (ΦᵀMΦ)[{i},{j}] = {qj:.3e}, expected {want}"
                );
            }
        }
    }
}

#[test]
fn full_rank_basis_reproduces_its_training_data_exactly() {
    let set = rank3_set();
    for inner in [Inner::Euclidean, Inner::Mass] {
        // The ensemble spans exactly 3 shapes, so 3 modes must reproduce every snapshot.
        let b = PodBasis::fit(&set, inner, &masses(), 1.0, 8).expect("fit");
        assert_eq!(
            b.n_modes(),
            3,
            "{inner:?}: rank-3 ensemble should yield 3 modes"
        );
        for u in set.columns() {
            let err = b.projection_error(u);
            assert!(err < 1e-10, "{inner:?}: training snapshot error {err:.3e}");
        }
    }
}

#[test]
fn truncation_loses_energy_monotonically() {
    let set = rank3_set();
    let mut prev = f64::MAX;
    for r in 1..=3 {
        let b = PodBasis::fit(&set, Inner::Mass, &masses(), 1.0, r).expect("fit");
        let worst = set
            .columns()
            .iter()
            .map(|u| b.projection_error(u))
            .fold(0.0_f64, f64::max);
        assert!(
            worst <= prev + 1e-12,
            "error rose when adding a mode: r={r} gave {worst:.3e} vs {prev:.3e}"
        );
        // Strictly positive below full rank. Without this the whole test — and
        // `full_rank_basis_reproduces_its_training_data_exactly` with it — is satisfied
        // by a `projection_error` stub that always returns 0.0.
        if r < 3 {
            assert!(
                worst > 1e-6,
                "r={r} on a rank-3 ensemble should leave real error, got {worst:.3e}"
            );
        }
        prev = worst;
    }
}

#[test]
fn mass_and_euclidean_products_give_genuinely_different_bases() {
    // Every other test in this file passes if `Inner::Mass` silently degenerates to
    // `Inner::Euclidean` — orthonormality, reconstruction and truncation all hold for a
    // Euclidean basis consistently used. Mass weighting is exactly where R1.0's bug
    // lived, so something has to assert the two products are not the same computation.
    //
    // `masses()` is deliberately non-uniform; a uniform mass WOULD make them agree up to
    // scaling, which is why the fixture varies it.
    let set = rank3_set();
    let m = masses();
    let eu = PodBasis::fit(&set, Inner::Euclidean, &m, 1.0, 1).expect("fit");
    let ma = PodBasis::fit(&set, Inner::Mass, &m, 1.0, 1).expect("fit");

    // Compare the leading mode as a direction (sign is arbitrary in an eigendecomposition).
    let mut e1 = vec![0.0; 1];
    e1[0] = 1.0;
    let (a, b) = (eu.reconstruct(&e1), ma.reconstruct(&e1));
    let dot: f64 = a.iter().zip(&b).map(|(x, y)| x * y).sum();
    let na = a.iter().map(|x| x * x).sum::<f64>().sqrt();
    let nb = b.iter().map(|x| x * x).sum::<f64>().sqrt();
    let cos = (dot / (na * nb)).abs();
    assert!(
        cos < 0.999,
        "leading Euclidean and mass modes are collinear (|cos| = {cos:.6}) — the mass \
         weighting is not reaching the decomposition"
    );
}

#[test]
fn rejects_an_empty_ensemble() {
    let empty = SnapshotSet::new(N_FREE);
    assert_eq!(
        PodBasis::fit(&empty, Inner::Euclidean, &masses(), 1.0, 4).unwrap_err(),
        PodError::Empty
    );
}

#[test]
fn rejects_an_all_zero_ensemble() {
    let mut z = SnapshotSet::new(N_FREE);
    z.push(&[0.0; N_FREE]);
    z.push(&[0.0; N_FREE]);
    assert_eq!(
        PodBasis::fit(&z, Inner::Euclidean, &masses(), 1.0, 4).unwrap_err(),
        PodError::NoContent
    );
}

#[test]
fn rejects_a_mass_slice_of_the_wrong_length() {
    // The realistic slip: the solver's full-DOF `mass_per_dof` reaching the parameter
    // that wants `mass_per_free_dof`. Truncating would silently build a wrong basis.
    let long = vec![1.0; N_FREE * 3];
    assert_eq!(
        PodBasis::fit(&rank3_set(), Inner::Mass, &long, 1.0, 3).unwrap_err(),
        PodError::MassLen {
            got: N_FREE * 3,
            want: N_FREE,
        }
    );
    let short = vec![1.0; N_FREE - 1];
    assert!(matches!(
        PodBasis::fit(&rank3_set(), Inner::Mass, &short, 1.0, 3).unwrap_err(),
        PodError::MassLen { .. }
    ));
    // Euclidean ignores the mass slice entirely, so a wrong length is not an error there.
    assert!(PodBasis::fit(&rank3_set(), Inner::Euclidean, &long, 1.0, 3).is_ok());
}

#[test]
fn rejects_a_non_positive_mass() {
    let mut bad = masses();
    bad[4] = 0.0;
    assert_eq!(
        PodBasis::fit(&rank3_set(), Inner::Mass, &bad, 1.0, 3).unwrap_err(),
        PodError::BadMass(4)
    );
    bad[4] = f64::NAN;
    assert_eq!(
        PodBasis::fit(&rank3_set(), Inner::Mass, &bad, 1.0, 3).unwrap_err(),
        PodError::BadMass(4)
    );
}

#[test]
fn energy_selection_stops_early_and_max_modes_caps() {
    let set = rank3_set();
    // A loose energy target must stop before full rank...
    let loose = PodBasis::fit(&set, Inner::Mass, &masses(), 0.5, 8).expect("fit");
    assert!(
        loose.n_modes() < 3,
        "0.5 energy kept {} modes",
        loose.n_modes()
    );
    assert!(loose.retained_energy_fraction() >= 0.5);
    // ...and max_modes must bind even when the energy target is never met.
    let capped = PodBasis::fit(&set, Inner::Mass, &masses(), 1.0, 2).expect("fit");
    assert_eq!(capped.n_modes(), 2);
    // Singular values are reported in full, truncation notwithstanding.
    assert!(capped.singular_values().len() >= 3);
}
