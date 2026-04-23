//! Finite-difference vs. analytic derivative test for `Tet4`.
//!
//! Central FD of `shape_functions` vs. `shape_gradients` at a
//! nontrivial point inside the reference tet per spec §9 step 3 "unit
//! FD-vs-analytic test per trait before moving on". Since the Tet4
//! shape functions are linear in $\xi$, the analytic gradients are
//! constant and central FD is exact in arithmetic — only roundoff
//! separates FD from the closed form, giving ~12 digits agreement.
//! The 5-digit `MAX_RELATIVE` is the cross-trait convention from
//! `material_fd.rs` and has ~7 digits of slack here.

use approx::assert_relative_eq;

use sim_soft::{Element, Tet4, Vec3};

const H: f64 = 1.5e-8;
const MAX_RELATIVE: f64 = 1.0e-5;

// Entries near zero (off-diagonal gradients of N_1..N_3) fall below
// any sensible relative tolerance; keep an absolute slack comparable
// to the `material_fd.rs` convention so the two test files share a
// numerical-tolerance story.
const EPSILON: f64 = 1.0e-6;

// `xi_0 = (0.3, 0.2, 0.1)` sums to 0.6 < 1 — inside the reference
// tet. Picking a nontrivial point is belt-and-braces; the gradients
// are constant so any valid `xi` would do, but this guards against
// an implementation that accidentally depends on `xi`.
#[test]
fn shape_gradients_match_central_fd_of_shape_functions() {
    let tet = Tet4;
    let xi0 = Vec3::new(0.3, 0.2, 0.1);
    let grad = tet.shape_gradients(xi0);

    for b in 0..3 {
        let mut delta = Vec3::zeros();
        delta[b] = H;
        let n_pos = tet.shape_functions(xi0 + delta);
        let n_neg = tet.shape_functions(xi0 - delta);
        let fd = (n_pos - n_neg) / (2.0 * H);
        for a in 0..4 {
            assert_relative_eq!(
                fd[a],
                grad[(a, b)],
                max_relative = MAX_RELATIVE,
                epsilon = EPSILON
            );
        }
    }
}
