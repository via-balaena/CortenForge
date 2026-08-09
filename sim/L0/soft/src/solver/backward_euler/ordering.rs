//! Fill-reducing ordering for the free-DOF sparse Cholesky factor.
//!
//! # Why this module exists
//!
//! Numeric sparse Cholesky is **~78 % of a solve** (profiled on
//! `run_indentation`; assembly is 1.0 % and everything else under 0.5 %), and
//! its cost is set almost entirely by the *fill* the elimination ordering
//! admits. faer ships approximate-minimum-degree (AMD) and hard-wires it:
//! [`SymbolicLlt::try_new`] passes `Default::default()` for the ordering and
//! keeps its inner [`SymbolicCholesky`] private, so there is no way to hand it
//! a different permutation through that type.
//!
//! Minimum-degree loses badly on 3D meshes. Measured on this crate's own Tet10
//! free-DOF tangent patterns, AMD's fill per row runs 435 → 714 → 974 across a
//! 4.25× DOF increase, while nested dissection holds 322 → 458 → 648 — a
//! 1.65× → 2.14× numeric-factorization speedup that is still widening at 27k
//! DOF. So this module does two things faer cannot:
//!
//! 1. [`nested_dissection_permutation`] turns the assembled free-DOF sparsity
//!    pattern into a nested-dissection permutation via `feral-metis`.
//! 2. [`OrderedLlt`] is the owned `symbolic + numeric` pair that faer's
//!    `Llt`/`SymbolicLlt` wrappers would be if their constructors took an
//!    ordering. It is a faithful re-implementation of those wrappers over the
//!    public low-level API ([`factorize_symbolic_cholesky`],
//!    [`SymbolicCholesky::factorize_numeric_llt`], [`LltRef`]) — same scratch
//!    allocation, same global parallelism, same solve entry point.
//!
//! # What this does NOT cover
//!
//! The A2 **LU fallback** keeps its own ordering. faer's
//! [`factorize_symbolic_lu`](faer::sparse::linalg::lu::factorize_symbolic_lu)
//! calls COLAMD unconditionally and exposes no custom-permutation hook — there
//! is no `SymmetricOrdering::Custom` equivalent on the LU side to reach for.
//! That is a limitation, not an oversight: the fallback is a cold path (it
//! engages only when the tangent is indefinite at the current Newton iterate),
//! it receives the matrix in its original ordering, and it therefore neither
//! benefits from nor interferes with the permutation chosen here.
//!
//! # Bit-exactness
//!
//! Changing the elimination order changes the order of the floating-point
//! accumulations inside the factorization, so it changes the last bits of the
//! solve. This is not a regression — it is the same class of difference as a
//! compiler reassociation — but it does mean byte-golden fixtures that pin a
//! sparse-solver result move when the ordering moves.

use std::sync::Arc;

use faer::dyn_stack::{MemBuffer, MemStack};
use faer::linalg::cholesky::llt::factor::{LltParams, LltRegularization};
use faer::perm::PermRef;
use faer::sparse::SparseColMatRef;
use faer::sparse::linalg::LltError;
use faer::sparse::linalg::cholesky::{
    CholeskySymbolicParams, LltRef, SymbolicCholesky, SymmetricOrdering,
    factorize_symbolic_cholesky,
};
use faer::{Conj, MatMut, Side, Spec};

/// A fill-reducing permutation, in the two-array form faer consumes.
///
/// `forward[i]` is the ORIGINAL index that lands at permuted position `i`
/// (new-to-old, matching `feral-metis`'s output convention and faer's own
/// `perm_fwd`); `inverse` is its inverse. Held together because
/// [`PermRef::new_checked`] wants both and checks they agree.
pub(super) struct FillReducingPermutation {
    forward: Vec<usize>,
    inverse: Vec<usize>,
}

impl FillReducingPermutation {
    /// Borrow as the `PermRef` faer's symbolic factorization takes.
    fn as_ref(&self) -> PermRef<'_, usize> {
        PermRef::new_checked(&self.forward, &self.inverse, self.forward.len())
    }
}

/// Compute a nested-dissection permutation of an `n × n` symmetric pattern.
///
/// `lower_triangle` holds `(col, row)` keys of the lower triangle — the same
/// [`BTreeSet`](std::collections::BTreeSet) contents the solver assembles its
/// sparsity pattern into. The full-symmetric graph `feral-metis` requires is
/// reflected out of it here.
///
/// Returns `None` — meaning *fall back to faer's AMD* — when the pattern
/// cannot be handed to `feral-metis` at all: an index that does not fit `i32`,
/// or an ordering failure. A `None` is a performance loss, never a correctness
/// one, which is why it degrades silently rather than panicking inside an L0
/// solver constructor.
pub(super) fn nested_dissection_permutation(
    lower_triangle: &std::collections::BTreeSet<(usize, usize)>,
    n: usize,
) -> Option<FillReducingPermutation> {
    let (col_ptr, row_idx) = full_symmetric_csc_i32(lower_triangle, n)?;
    let pattern = feral_metis::CscPattern::new(n, &col_ptr, &row_idx)?;
    let forward_i32 = feral_metis::metis_order(&pattern).ok()?;

    // `metis_order` returns new-to-old. Invert it, and validate as we go: a
    // permutation with a repeated or out-of-range entry would be accepted by
    // `PermRef::new_checked` only to corrupt every solve, so it is checked
    // here where the failure is still recoverable.
    let mut forward = Vec::with_capacity(n);
    let mut inverse = vec![usize::MAX; n];
    for (new, &old) in forward_i32.iter().enumerate() {
        let old = usize::try_from(old).ok()?;
        if old >= n || inverse[old] != usize::MAX {
            return None;
        }
        inverse[old] = new;
        forward.push(old);
    }
    if forward.len() != n {
        return None;
    }

    Some(FillReducingPermutation { forward, inverse })
}

/// Reflect a lower-triangle `(col, row)` key set into the full-symmetric,
/// row-sorted, 0-based `i32` CSC pattern `feral-metis` requires.
///
/// Returns `None` if `n` or the reflected non-zero count overflows `i32`.
///
/// The diagonal is emitted as-is; `feral-metis` drops self-loops when it
/// builds its adjacency graph.
fn full_symmetric_csc_i32(
    lower_triangle: &std::collections::BTreeSet<(usize, usize)>,
    n: usize,
) -> Option<(Vec<i32>, Vec<i32>)> {
    i32::try_from(n).ok()?;

    // Pass 1: count each column's entries. `(c, r)` with `c != r` contributes
    // to both column `c` and column `r`.
    let mut counts = vec![0usize; n];
    for &(c, r) in lower_triangle {
        if c >= n || r >= n {
            return None;
        }
        counts[c] += 1;
        if c != r {
            counts[r] += 1;
        }
    }

    // Pass 2: prefix-sum into column pointers. Kept in `usize` throughout and
    // narrowed to `i32` only at the boundary, so no cast can lose a bit.
    let mut col_start = Vec::with_capacity(n + 1);
    let mut running = 0usize;
    col_start.push(0usize);
    for &count in &counts {
        running += count;
        col_start.push(running);
    }
    let nnz = running;

    // Pass 3: scatter. `cursor[c]` is the next free slot in column `c`.
    let mut cursor: Vec<usize> = col_start[..n].to_vec();
    let mut row_idx = vec![0i32; nnz];
    for &(c, r) in lower_triangle {
        let (ri, ci) = (i32::try_from(r).ok()?, i32::try_from(c).ok()?);
        row_idx[cursor[c]] = ri;
        cursor[c] += 1;
        if c != r {
            row_idx[cursor[r]] = ci;
            cursor[r] += 1;
        }
    }

    // Pass 4: `CscPattern::new` requires ascending row indices within each
    // column, and the reflected entries arrive out of order.
    for window in col_start.windows(2) {
        row_idx[window[0]..window[1]].sort_unstable();
    }

    let col_ptr: Option<Vec<i32>> = col_start
        .into_iter()
        .map(|p| i32::try_from(p).ok())
        .collect();
    Some((col_ptr?, row_idx))
}

/// The symbolic Cholesky factor, built against a caller-chosen ordering.
///
/// `Arc`-shared for the same reason faer's `SymbolicLlt` is: the solver holds
/// one for its lifetime and every numeric refactor clones it per Newton
/// iteration, so the clone must be a refcount bump rather than a copy of the
/// elimination tree.
pub(super) type SharedSymbolicCholesky = Arc<SymbolicCholesky<usize>>;

/// Build the symbolic Cholesky factor of `pattern`'s lower triangle under
/// `permutation`, or under AMD when no permutation was obtained.
///
/// # Errors
///
/// Propagates faer's symbolic-factorization failure, which for a valid pattern
/// means out-of-memory.
pub(super) fn symbolic_cholesky(
    pattern: faer::sparse::SymbolicSparseColMatRef<'_, usize>,
    permutation: Option<&FillReducingPermutation>,
) -> Result<SharedSymbolicCholesky, faer::sparse::FaerError> {
    let ordering = permutation.map_or(SymmetricOrdering::Amd, |p| {
        SymmetricOrdering::Custom(p.as_ref())
    });
    factorize_symbolic_cholesky(
        pattern,
        Side::Lower,
        ordering,
        CholeskySymbolicParams::default(),
    )
    .map(Arc::new)
}

/// A numeric `LLᵀ` factor over a [`SharedSymbolicCholesky`].
///
/// This is faer's `Llt<usize, f64>` re-expressed over the low-level API so it
/// can carry a custom ordering: the same owned `(symbolic, values)` pair, the
/// same numeric entry point, and the same solve. Owned rather than borrowing
/// so it can live in the solver's `FactorInner` enum without threading a
/// lifetime through every factor-site signature.
pub(super) struct OrderedLlt {
    symbolic: SharedSymbolicCholesky,
    values: Vec<f64>,
}

impl OrderedLlt {
    /// Numerically factor `mat` (lower triangle read) against `symbolic`.
    ///
    /// # Errors
    ///
    /// [`LltError::Numeric`] when the matrix is not numerically positive
    /// definite — the non-PD pivot the A2 LU fallback and the LM retry both
    /// key off — or an allocation failure.
    pub(super) fn try_new_with_symbolic(
        symbolic: SharedSymbolicCholesky,
        mat: SparseColMatRef<'_, usize, f64>,
        side: Side,
    ) -> Result<Self, LltError> {
        let parallelism = faer::get_global_parallelism();
        // Every parameter faer's own `Llt::try_new_with_symbolic` leaves at its
        // default is left at its default here too — an unregularized factor,
        // stock `LltParams` — so the only difference between the two paths is
        // the ordering.
        let regularization = LltRegularization::default();
        let params = Spec::<LltParams, f64>::default();
        let mut values = vec![0.0_f64; symbolic.len_val()];
        symbolic.factorize_numeric_llt::<f64>(
            &mut values,
            mat,
            side,
            regularization,
            parallelism,
            MemStack::new(&mut MemBuffer::try_new(
                symbolic.factorize_numeric_llt_scratch::<f64>(parallelism, params),
            )?),
            params,
        )?;
        Ok(Self { symbolic, values })
    }

    /// Solve `A x = rhs` in place through the stored factor.
    pub(super) fn solve_in_place_with_conj(&self, conj: Conj, rhs: MatMut<'_, f64>) {
        let parallelism = faer::get_global_parallelism();
        let n_rhs_cols = rhs.ncols();
        LltRef::<'_, usize, f64>::new(&self.symbolic, &self.values).solve_in_place_with_conj(
            conj,
            rhs,
            parallelism,
            MemStack::new(&mut MemBuffer::new(
                self.symbolic
                    .solve_in_place_scratch::<f64>(n_rhs_cols, parallelism),
            )),
        );
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        // Same rationale as the sibling `tests` module: "fail loudly with a
        // clear message" is the whole vocabulary of a test.
        clippy::expect_used,
        // The chain fixture indexes a `usize` loop counter into `sin`/`cos` to
        // get a non-degenerate right-hand side. `n` is 200 here, nowhere near
        // f64's 52-bit mantissa.
        clippy::cast_precision_loss
    )]

    use std::collections::BTreeSet;

    use faer::sparse::{SparseColMat, Triplet};
    use faer::{Conj, Mat, Side};

    use super::{OrderedLlt, nested_dissection_permutation, symbolic_cholesky};

    /// A 1D chain `n`-node Laplacian + diagonal shift: SPD, and sparse enough
    /// that an ordering has something to do.
    fn chain_pattern(n: usize) -> (BTreeSet<(usize, usize)>, SparseColMat<usize, f64>) {
        let mut lower = BTreeSet::new();
        let mut triplets: Vec<Triplet<usize, usize, f64>> = Vec::new();
        for i in 0..n {
            lower.insert((i, i));
            triplets.push(Triplet::new(i, i, 4.0));
            if i + 1 < n {
                // Lower-triangle key is `(col, row)`.
                lower.insert((i, i + 1));
                triplets.push(Triplet::new(i + 1, i, -1.0));
            }
        }
        let mat = SparseColMat::try_new_from_triplets(n, n, &triplets).expect("chain pattern");
        (lower, mat)
    }

    /// The permutation must be a permutation: every index exactly once, and
    /// `forward`/`inverse` mutually consistent.
    #[test]
    fn nested_dissection_permutation_is_a_valid_permutation() {
        let n = 200;
        let (lower, _) = chain_pattern(n);
        let perm = nested_dissection_permutation(&lower, n).expect("ordering");
        let mut seen = vec![false; n];
        for (new, &old) in perm.forward.iter().enumerate() {
            assert!(!seen[old], "index {old} appears twice in the permutation");
            seen[old] = true;
            assert_eq!(perm.inverse[old], new, "inverse disagrees with forward");
        }
        assert!(seen.into_iter().all(|s| s), "permutation is not surjective");
    }

    /// End-to-end convention check: a solve through the custom-ordered factor
    /// must reproduce the right-hand side.
    ///
    /// This is the test that pins the `forward`/`inverse` orientation. A
    /// swapped convention still produces a valid-looking permutation and a
    /// factorization that succeeds — it just solves the wrong system, which
    /// only a residual catches.
    #[test]
    fn custom_ordered_solve_matches_the_original_system() {
        let n = 200;
        let (lower, mat) = chain_pattern(n);
        let perm = nested_dissection_permutation(&lower, n).expect("ordering");
        let symbolic = symbolic_cholesky(mat.symbolic(), Some(&perm)).expect("symbolic");
        let factor = OrderedLlt::try_new_with_symbolic(symbolic, mat.as_ref(), Side::Lower)
            .expect("numeric");

        // Solve `A x = b` for a known `b`, then recompute `A x` and compare.
        let b: Mat<f64> = Mat::from_fn(n, 1, |i, _| (i as f64).sin());
        let mut x = b.clone();
        factor.solve_in_place_with_conj(Conj::No, x.as_mut());

        let mut residual = 0.0_f64;
        for i in 0..n {
            let mut ax = 4.0 * x[(i, 0)];
            if i > 0 {
                ax -= x[(i - 1, 0)];
            }
            if i + 1 < n {
                ax -= x[(i + 1, 0)];
            }
            residual = residual.max((ax - b[(i, 0)]).abs());
        }
        assert!(
            residual < 1.0e-12,
            "custom-ordered solve left residual {residual:e} — the permutation \
             convention is wrong, or the factor is not solving the original system"
        );
    }

    /// The custom-ordered factor must agree with faer's own AMD-ordered one to
    /// solver tolerance. Not bit-equal — a different elimination order
    /// reassociates the accumulations — but the same linear system.
    #[test]
    fn custom_ordered_solve_agrees_with_faer_amd() {
        use faer::linalg::solvers::SolveCore;
        use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};

        let n = 200;
        let (lower, mat) = chain_pattern(n);
        let perm = nested_dissection_permutation(&lower, n).expect("ordering");
        let symbolic = symbolic_cholesky(mat.symbolic(), Some(&perm)).expect("symbolic");
        let ours = OrderedLlt::try_new_with_symbolic(symbolic, mat.as_ref(), Side::Lower)
            .expect("numeric");

        let amd_symbolic = SymbolicLlt::try_new(mat.symbolic(), Side::Lower).expect("amd symbolic");
        let amd = Llt::<usize, f64>::try_new_with_symbolic(amd_symbolic, mat.as_ref(), Side::Lower)
            .expect("amd numeric");

        let b: Mat<f64> = Mat::from_fn(n, 1, |i, _| (i as f64).cos());
        let mut ours_x = b.clone();
        let mut amd_x = b;
        ours.solve_in_place_with_conj(Conj::No, ours_x.as_mut());
        amd.solve_in_place_with_conj(Conj::No, amd_x.as_mut());

        let worst = (0..n).fold(0.0_f64, |acc, i| {
            acc.max((ours_x[(i, 0)] - amd_x[(i, 0)]).abs())
        });
        assert!(
            worst < 1.0e-12,
            "custom-ordered and AMD-ordered solves disagree by {worst:e}"
        );
    }

    /// An empty pattern must not panic — `n = 0` is reachable for a fully
    /// pinned mesh.
    #[test]
    fn empty_pattern_orders_without_panicking() {
        let lower = BTreeSet::new();
        let perm = nested_dissection_permutation(&lower, 0).expect("empty ordering");
        assert!(perm.forward.is_empty());
    }
}
