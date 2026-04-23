//! Finite-difference vs. analytic derivative tests for `NeoHookean`.
//!
//! Two FD checks per spec §9 step 3 "unit FD-vs-analytic test per trait
//! before moving on":
//!
//! 1. `first_piola` vs. central FD of `energy`.
//! 2. `tangent` vs. central FD of `first_piola`.
//!
//! Both at `h = 1.5e-8` per spec R-3 (central FD, square root of f64 eps),
//! relative tolerance `1e-5` per spec §5 "5-digit relative bar". The
//! test point is a nontrivial `F` with small off-diagonal entries so
//! every term of the closed-form tangent contributes — `F = I` and
//! diagonal `F` collapse the off-diagonal `F⁻ᵀ` products to zero and
//! would not exercise the general path.

use approx::assert_relative_eq;
use nalgebra::Matrix3;

use sim_soft::{Material, NeoHookean};

const H: f64 = 1.5e-8;
const MAX_RELATIVE: f64 = 1.0e-5;

// Absolute slack covers FD truncation + roundoff at `h = √eps`. Central
// FD noise is bounded by `|y| · eps_f64 / h ≈ |y| · 1.5e-8`; for P at
// µ ≈ 10^5 this is ~1.5e-3, for ∂P/∂F at µ ≈ 10^5 it is the same scale.
// `1e-3` passes comfortably while being far below any nontrivial entry.
const EPSILON: f64 = 1.0e-3;

const fn material() -> NeoHookean {
    NeoHookean {
        mu: 1.0e5,
        lambda: 4.0e5,
    }
}

// Small off-diagonal entries activate every term in the closed-form
// tangent (see module doc). `det F ≈ 0.987 > 0`, well inside the
// `RequireOrientation` validity domain. Not marked `const fn` because
// `Matrix3::new` is non-const in nalgebra 0.34; test-code ergonomic
// scope is tight enough that the nursery lint would be noise.
#[allow(clippy::missing_const_for_fn)]
fn nontrivial_f() -> Matrix3<f64> {
    Matrix3::new(
        1.10, 0.05, 0.02, //
        0.04, 0.90, 0.01, //
        0.03, 0.02, 1.00, //
    )
}

#[test]
fn first_piola_matches_central_fd_of_energy() {
    let mat = material();
    let f0 = nontrivial_f();
    let p = mat.first_piola(&f0);

    for i in 0..3 {
        for j in 0..3 {
            let mut f_pos = f0;
            f_pos[(i, j)] += H;
            let mut f_neg = f0;
            f_neg[(i, j)] -= H;
            let fd = (mat.energy(&f_pos) - mat.energy(&f_neg)) / (2.0 * H);
            assert_relative_eq!(
                fd,
                p[(i, j)],
                max_relative = MAX_RELATIVE,
                epsilon = EPSILON
            );
        }
    }
}

#[test]
fn tangent_matches_central_fd_of_first_piola() {
    let mat = material();
    let f0 = nontrivial_f();
    let tangent = mat.tangent(&f0);

    for k in 0..3 {
        for l in 0..3 {
            let mut f_pos = f0;
            f_pos[(k, l)] += H;
            let mut f_neg = f0;
            f_neg[(k, l)] -= H;
            let p_pos = mat.first_piola(&f_pos);
            let p_neg = mat.first_piola(&f_neg);
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
