//! I-3 factor-on-tape: `faer::sparse::linalg::solvers::Llt` must own its
//! data after the source matrix is dropped.
//!
//! The walking-skeleton VJP (step 5) stashes `Llt` on the chassis tape
//! via `Tape::push_custom`; that only works if `Llt` is `Send + Sync`
//! **and** doesn't borrow from the source matrix. Scope §11 S-3
//! (Round 1) verified this from the `faer` 0.24 source; this test
//! verifies it operationally.
//!
//! Protocol per spec §6:
//!
//! 1. Assemble a 12×12 synthetic SPD (diag-dominant symmetric).
//! 2. Build `SymbolicLlt` once + `Llt::try_new_with_symbolic`.
//! 3. Drop the source `SparseColMat` (inner scope).
//! 4. Solve for two different RHSes with the surviving factor.
//! 5. Verify `A · x_i ≈ b_i` against the retained dense reference at
//!    `1e-12` (well below the `1e-10` skeleton tolerance).

use faer::linalg::solvers::SolveCore;
use faer::prelude::Reborrow;
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::{SparseColMat, Triplet};
use faer::{Conj, MatMut, Side};

const N: usize = 12;

/// Dense reference SPD matrix used both to build the sparse input and
/// to verify post-solve residuals. Diagonal 12.0, off-diagonal 0.5 —
/// strictly diagonally dominant (12 > 11 · 0.5), guaranteed SPD.
fn reference_spd() -> [[f64; N]; N] {
    let mut a = [[0.0f64; N]; N];
    for (i, row) in a.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = if i == j { 12.0 } else { 0.5 };
        }
    }
    a
}

/// Dense `A · x` (reference check, not perf-sensitive).
fn matvec(a: &[[f64; N]; N], x: &[f64; N]) -> [f64; N] {
    let mut y = [0.0; N];
    for (i, row) in a.iter().enumerate() {
        for (j, &a_ij) in row.iter().enumerate() {
            y[i] = a_ij.mul_add(x[j], y[i]);
        }
    }
    y
}

/// Lower-triangle triplets in sorted (col, row) order — deterministic
/// CSR build per scope §15 D-4.
fn lower_triangle_triplets(a: &[[f64; N]; N]) -> Vec<Triplet<usize, usize, f64>> {
    let mut triplets = Vec::with_capacity(N * (N + 1) / 2);
    for col in 0..N {
        for (row, a_row) in a.iter().enumerate().skip(col) {
            triplets.push(Triplet::new(row, col, a_row[col]));
        }
    }
    triplets
}

// Test fixture uses `.expect(...)` on faer builder calls — test-harness
// boundary, where expect-with-message IS the right failure mode (the
// panic message is the test-failure diagnostic).
#[allow(clippy::expect_used)]
#[test]
fn llt_factor_owns_data_after_source_drop() {
    let a_dense = reference_spd();

    // Factor in an inner scope so the source `SparseColMat` is dropped
    // before we solve. `Llt` must survive.
    let llt: Llt<usize, f64> = {
        let triplets = lower_triangle_triplets(&a_dense);
        let a_sparse: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(N, N, &triplets).expect("build sparse");
        let symbolic = SymbolicLlt::<usize>::try_new(a_sparse.symbolic(), Side::Lower)
            .expect("symbolic analysis");
        let factor = Llt::<usize, f64>::try_new_with_symbolic(symbolic, a_sparse.rb(), Side::Lower)
            .expect("numeric factorization");
        // `a_sparse` and `triplets` drop at scope exit; `symbolic` is
        // consumed by `try_new_with_symbolic`. If `Llt` borrowed from any
        // of them, the subsequent solve would be use-after-free and this
        // test would either fail compilation (lifetime error) or produce
        // garbage residuals. `Llt: Send + Sync` with no lifetime param
        // on the struct is what makes step 5's `Tape::push_custom` work.
        factor
    };

    // Two different RHSes — verify consistency across both.
    let b1: [f64; N] = [
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
    ];
    let b2: [f64; N] = [
        -1.0, 2.5, -3.5, 4.0, -5.0, 6.5, -7.0, 8.5, -9.0, 10.5, -11.0, 12.5,
    ];

    for b in [&b1, &b2] {
        let mut x = *b;
        {
            let rhs = MatMut::from_column_major_slice_mut(&mut x, N, 1);
            llt.solve_in_place_with_conj(Conj::No, rhs);
        }
        // Reconstruct b from A · x; compare to the original.
        let b_reconstructed = matvec(&a_dense, &x);
        for (i, (&expected, &got)) in b.iter().zip(b_reconstructed.iter()).enumerate() {
            let err = (expected - got).abs();
            assert!(
                err < 1e-12,
                "I-3 factor survives source drop failed at row {i}: \
                 expected b[{i}] = {expected}, got A·x[{i}] = {got}, |err| = {err:e}"
            );
        }
    }
}
