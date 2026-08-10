//! Fill-reducing ordering for the free-DOF sparse Cholesky factor.
//!
//! # Why this module exists
//!
//! Numeric sparse Cholesky is **~78 % of a solve**, and its cost is set almost
//! entirely by the *fill* the elimination ordering admits. That share is
//! corroborated end-to-end rather than only by a profiler: a 2.12× isolated
//! factorization speedup predicts `1/(0.22 + 0.78/2.12)` = 1.70× on a whole
//! gate, and `bonded_layer_indentation` measured 1.73×.
//!
//! (An earlier version of this line also carried a per-bucket breakdown —
//! "assembly 1.0 %, everything else under 0.5 %" — attributed to a profiling
//! run with no artefact in the repo, over a `run_indentation` that resolves to
//! two different functions. The buckets did not sum, and nothing could
//! regenerate them, so they are gone rather than dressed up.)
//!
//! faer ships approximate-minimum-degree (AMD) and hard-wires it:
//! [`SymbolicLlt::try_new`](faer::sparse::linalg::solvers::SymbolicLlt::try_new)
//! passes `Default::default()` for the ordering and keeps its inner
//! [`SymbolicCholesky`] private, so there is no way to hand it a different
//! permutation through that type.
//!
//! Minimum-degree loses badly on 3D meshes. Measured on this crate's own Tet10
//! free-DOF tangent patterns, AMD's fill per row runs 435 → 714 → 974 across a
//! 4.25× DOF increase, while nested dissection holds 334 → 458 → 648 — a
//! 1.61× → 2.12× numeric-factorization speedup that is still widening at 27k
//! DOF. It is not a win at every size, though; see
//! [`NESTED_DISSECTION_MIN_FREE_DOF`] for the measured crossover. So this
//! module does two things faer cannot:
//!
//! 1. [`nested_dissection_permutation`] turns the assembled free-DOF sparsity
//!    pattern into a nested-dissection permutation via `feral-metis`.
//! 2. [`OrderedLlt`] is the owned `symbolic + numeric` pair that faer's
//!    `Llt`/`SymbolicLlt` wrappers would be if their constructors took an
//!    ordering. It is a faithful re-implementation of those wrappers over the
//!    public low-level API ([`factorize_symbolic_cholesky`],
//!    `SymbolicCholesky::factorize_numeric_llt`, [`LltRef`]) — same scratch
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

/// Free-DOF count below which faer's AMD is kept.
///
/// Nested dissection is an asymptotic win, not a universal one, and it is paid
/// for on a different schedule from the benefit: a one-off **symbolic** charge
/// per solver construction, repaid a little on every **numeric**
/// factorization. So the threshold is a break-even, not a speed comparison.
///
/// Measured by `factorization_fill_growth` with rayon enabled — the shipped
/// configuration, which matters because parallelism shrinks ND's numeric
/// advantage while leaving its symbolic cost untouched:
///
/// | `n_free` | extra symbolic | saved per factorization | break-even |
/// |---|---|---|---|
/// | 348 | +1.1 ms | ND is SLOWER | never |
/// | 2,112 | +13.7 ms | 0.3 ms | 46 |
/// | 6,444 | +51.2 ms | 4.8 ms | **10.7** |
/// | 14,496 | +133.7 ms | 26.3 ms | 5.1 |
/// | 27,420 | +272.6 ms | 82.5 ms | 3.3 |
///
/// The other half is the rate at which a solver performs factorizations, from
/// `factorization_rate_per_step`: **≈2.2 per `step`**, flat across 348 → 6,444
/// free DOF (one Newton solve plus the IFT adjoint factor). Break-even
/// therefore converts to *steps*: ~21 at 2,112, ~5 at 6,444, ~1.5 at 27,420.
///
/// **6,444 is the smallest measured size where break-even lands inside
/// ordinary use.** Every ramp, probe and envelope gate in this repo steps a
/// solver far more than five times, so ND repays there — corroborated
/// end-to-end by `bonded_layer_indentation`, which is 1.25× faster *including*
/// the symbolic charge. Below it the requirement climbs steeply (~21 steps at
/// 2,112) and a short solve would be made SLOWER by ordering it.
///
/// The constant is the measurement, not a round number near it. It was
/// previously 6,000 — and 2,000 before that — each set just BELOW the smallest
/// size where ND was measured to win, quietly extending it into territory
/// nothing had measured while the doc claimed the opposite.
///
/// ⚠ **Known limit, stated rather than hidden.** This is calibrated for
/// multi-step use. A solver constructed for a SINGLE step performs ~2.2
/// factorizations and repays the ordering at no size measured here; such a
/// caller pays a small one-off penalty above the threshold. That is the right
/// trade for this codebase, where solvers are reused, but it is a calibration
/// and not a law — re-derive against fresh runs of both producers if the
/// parallelism configuration or the Newton tolerances change.
pub(super) const NESTED_DISSECTION_MIN_FREE_DOF: usize = 6444;

/// Compute a nested-dissection permutation of an `n × n` symmetric pattern.
///
/// `lower_triangle` holds `(col, row)` keys of the lower triangle — the same
/// [`BTreeSet`](std::collections::BTreeSet) contents the solver assembles its
/// sparsity pattern into. The full-symmetric graph `feral-metis` requires is
/// reflected out of it here.
///
/// Returns `None` — meaning *use faer's AMD* — when `n` is below
/// [`NESTED_DISSECTION_MIN_FREE_DOF`], or when the pattern cannot be handed to
/// `feral-metis` at all: an index that does not fit `i32`, or an ordering
/// failure. A `None` is a performance choice, never a correctness one, which is
/// why it degrades silently rather than panicking inside an L0 solver
/// constructor.
pub(super) fn nested_dissection_permutation(
    lower_triangle: &std::collections::BTreeSet<(usize, usize)>,
    n: usize,
) -> Option<FillReducingPermutation> {
    if n < NESTED_DISSECTION_MIN_FREE_DOF {
        return None;
    }
    compute_nested_dissection_permutation(lower_triangle, n)
}

/// [`nested_dissection_permutation`] without the size policy.
///
/// Exists for the `factorization_fill_growth` diagnostic, whose whole job is to
/// measure where that policy's crossover actually is — it has to be able to
/// order a pattern the policy would decline. Production goes through
/// [`nested_dissection_permutation`].
pub(super) fn compute_nested_dissection_permutation(
    lower_triangle: &std::collections::BTreeSet<(usize, usize)>,
    n: usize,
) -> Option<FillReducingPermutation> {
    let (col_ptr, row_idx) = full_symmetric_csc_i32(lower_triangle, n)?;
    let pattern = feral_metis::CscPattern::new(n, &col_ptr, &row_idx)?;
    let forward_i32 = feral_metis::metis_order(&pattern).ok()?;
    invert_new_to_old(&forward_i32, n)
}

/// Turn a new-to-old permutation into the validated `(forward, inverse)` pair
/// faer consumes, or `None` if it is not a permutation of `0..n`.
///
/// Split out from [`compute_nested_dissection_permutation`] so the validation
/// is reachable without going through `feral-metis`. That matters: METIS
/// returns valid permutations, so a test that can only feed this function
/// METIS output cannot tell whether the validation exists at all.
///
/// The check is not defensive padding. `PermRef::new_checked` would accept a
/// repeated or out-of-range entry as a shape, and the corruption would surface
/// as silently wrong solves rather than an error — so it is caught here, where
/// falling back to AMD is still an option.
fn invert_new_to_old(forward_i32: &[i32], n: usize) -> Option<FillReducingPermutation> {
    if forward_i32.len() != n {
        return None;
    }
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
    Some(FillReducingPermutation { forward, inverse })
}

/// Reflect a lower-triangle `(col, row)` key set into the full-symmetric,
/// row-sorted, 0-based `i32` CSC pattern `feral-metis` requires.
///
/// Returns `None` if `n` or the reflected non-zero count overflows `i32`.
///
/// The diagonal is emitted as-is; `feral-metis` drops self-loops when it
/// builds its adjacency graph.
///
/// The solver's producers only ever insert the lower triangle
/// (`construct.rs` and `fbar.rs` both guard on `row_free >= col_free`), but
/// this does not depend on that: reflecting is orientation-agnostic, and an
/// entry present in BOTH triangles would merely emit a duplicate, which
/// `CscPattern` permits and `feral-metis`'s adjacency builder drops.
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
    // Bail on an `i32`-overflowing pattern HERE, before the `row_idx`
    // allocation below — the overflow case is the one where memory is already
    // scarce, and allocating 4·nnz bytes only to discard them is the worst
    // possible moment to do it.
    let nnz = running;
    i32::try_from(nnz).ok()?;

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
        // Read the global here rather than storing it on the factor, because
        // that is what faer's own wrapper does (`sparse/solvers.rs`, both the
        // factor and the solve). Nothing in this workspace calls
        // `set_global_parallelism`, so the two readings cannot disagree today;
        // matching faer keeps it that way if one ever does.
        let parallelism = faer::get_global_parallelism();
        // Every parameter faer's own `Llt::try_new_with_symbolic` leaves at its
        // default is left at its default here too — an unregularized factor,
        // stock `LltParams` — so the only difference between the two paths is
        // the ordering.
        let regularization = LltRegularization::default();
        let params = Spec::<LltParams, f64>::default();
        // `try_reserve_exact` rather than `vec![0.0; n]` so an oversized factor
        // REPORTS `OutOfMemory` instead of aborting the process — the same
        // contract faer's own wrapper offers, and the one this method's
        // `# Errors` section promises.
        let len_val = symbolic.len_val();
        let mut values = Vec::new();
        values
            .try_reserve_exact(len_val)
            .map_err(|_| faer::sparse::FaerError::OutOfMemory)?;
        values.resize(len_val, 0.0_f64);
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
    ///
    /// ⚠ Unlike [`Self::try_new_with_symbolic`], this **aborts** rather than
    /// reporting an allocation failure, and that asymmetry is deliberate: it is
    /// inherited from faer, whose own `Llt` splits the same way for a
    /// structural reason. Its solve is a `SolveCore` **trait** method returning
    /// `()`, so `MemBuffer::new` is forced — there is no channel to report OOM
    /// through — while its constructor is an inherent fallible method that uses
    /// `try_new`. Mirroring it here keeps the two paths interchangeable.
    ///
    /// Making this fallible would not be a local change: the enclosing
    /// `FactorInner`'s other arm is faer's own `Lu`, whose solve can never
    /// populate such an error, so a `Result` would thread through
    /// `solve_base_in_place`, `solve_free_in_place` and every adjoint and tape
    /// consumer to carry a variant only half the enum can produce.
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

    use super::{
        FillReducingPermutation, OrderedLlt, compute_nested_dissection_permutation,
        full_symmetric_csc_i32, invert_new_to_old, nested_dissection_permutation,
        symbolic_cholesky,
    };

    /// A 3D 7-point Laplacian on a `k x k x k` grid: SPD, and the canonical
    /// shape where a fill-reducing ordering earns its keep.
    ///
    /// The chain fixture below cannot serve this purpose. A 1D chain is a
    /// PERFECT-ELIMINATION graph — it admits zero fill under any ordering — so
    /// every ordering agrees on it and no test built on it can distinguish a
    /// good ordering from a bad one. It also happens to be row-sorted already,
    /// which hides the reflection's sort. A 3D grid has neither property.
    fn grid3d_pattern(k: usize) -> (BTreeSet<(usize, usize)>, SparseColMat<usize, f64>) {
        let idx = |i: usize, j: usize, l: usize| (i * k + j) * k + l;
        let n = k * k * k;
        let mut lower = BTreeSet::new();
        let mut triplets: Vec<Triplet<usize, usize, f64>> = Vec::new();
        for i in 0..k {
            for j in 0..k {
                for l in 0..k {
                    let v = idx(i, j, l);
                    lower.insert((v, v));
                    triplets.push(Triplet::new(v, v, 6.0));
                    for (di, dj, dl) in [(1, 0, 0), (0, 1, 0), (0, 0, 1)] {
                        let (ni, nj, nl) = (i + di, j + dj, l + dl);
                        if ni < k && nj < k && nl < k {
                            let w = idx(ni, nj, nl);
                            let (lo, hi) = (v.min(w), v.max(w));
                            // Lower-triangle key is `(col, row)` with row >= col.
                            lower.insert((lo, hi));
                            triplets.push(Triplet::new(hi, lo, -1.0));
                        }
                    }
                }
            }
        }
        let mat = SparseColMat::try_new_from_triplets(n, n, &triplets).expect("grid pattern");
        (lower, mat)
    }

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

    /// The guard against a malformed permutation must actually reject one.
    ///
    /// ⚠ The previous version called `compute_nested_dissection_permutation`
    /// and checked the result was a valid permutation — which tests
    /// `feral_metis::metis_order`, not this module. The guard could be deleted
    /// entirely and it stayed green, because METIS returns valid permutations.
    /// Driving `invert_new_to_old` directly is what makes the guard reachable.
    #[test]
    fn a_malformed_permutation_is_rejected_rather_than_installed() {
        // Valid: a cyclic shift.
        const N: i32 = 8;
        let shift: Vec<i32> = (0..N).map(|i| (i + 3) % N).collect();
        let perm = invert_new_to_old(&shift, shift.len()).expect("a cyclic shift is a permutation");
        for (new, &old) in perm.forward.iter().enumerate() {
            assert_eq!(perm.inverse[old], new, "forward/inverse disagree");
        }

        // Repeated index — the case that would corrupt every solve if it
        // reached `PermRef::new_checked`.
        assert!(
            invert_new_to_old(&[0, 1, 1, 3], 4).is_none(),
            "a permutation with a repeated index was accepted"
        );
        // Out of range.
        assert!(
            invert_new_to_old(&[0, 1, 2, 9], 4).is_none(),
            "a permutation with an out-of-range index was accepted"
        );
        // Wrong length.
        assert!(
            invert_new_to_old(&[0, 1, 2], 4).is_none(),
            "a permutation of the wrong length was accepted"
        );
        // Negative index.
        assert!(
            invert_new_to_old(&[0, 1, -1, 3], 4).is_none(),
            "a permutation with a negative index was accepted"
        );
    }

    /// The `forward`/`inverse` orientation must be the FILL-REDUCING one.
    ///
    /// ⚠ A residual check cannot establish this, and an earlier version of this
    /// test claimed it could. faer's `SymmetricOrdering::Custom` copies both
    /// arrays and uses the same pair to permute the matrix and to un-permute
    /// the solution, so ANY mutually-consistent `(forward, inverse)` pair
    /// solves the original system exactly — a swapped convention still returns
    /// the right answer, to the last bit. What it loses is the fill reduction,
    /// because the inverse of a nested-dissection order is not itself a
    /// nested-dissection order.
    ///
    /// So the observable is `nnz(L)`, on a 3D grid where orderings actually
    /// differ. On the chain fixture this test would be vacuous in a second way:
    /// a chain admits no fill under any ordering at all.
    #[test]
    fn permutation_orientation_is_the_fill_reducing_one() {
        let k = 18;
        let (lower, mat) = grid3d_pattern(k);
        let n = k * k * k;
        let perm = compute_nested_dissection_permutation(&lower, n).expect("ordering");

        let correct = symbolic_cholesky(mat.symbolic(), Some(&perm)).expect("symbolic");

        // The same permutation with its two arrays exchanged: still a valid,
        // mutually-consistent permutation, still solves correctly, no longer
        // fill-reducing.
        let swapped = FillReducingPermutation {
            forward: perm.inverse,
            inverse: perm.forward,
        };
        let swapped = symbolic_cholesky(mat.symbolic(), Some(&swapped)).expect("symbolic");

        assert!(
            correct.len_val() < swapped.len_val(),
            "the permutation is installed with its arrays exchanged: as declared it \
             admits {} non-zeros, exchanged it admits {}. Both solve correctly, so \
             only fill reveals the swap — see `FillReducingPermutation`'s field docs \
             for the new-to-old convention.",
            correct.len_val(),
            swapped.len_val()
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
        let perm = compute_nested_dissection_permutation(&lower, n).expect("ordering");
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

    /// The factor outlives the matrix it was built from, and crosses threads.
    ///
    /// Both properties are load-bearing and neither is obvious: the factor is
    /// stashed on the autodiff tape inside `NewtonStepVjp`, and `VjpOp` is
    /// declared `Send + Sync`, so a factor that borrowed from the assembled
    /// tangent — or that held a non-`Sync` handle — would break reverse mode
    /// rather than this module. `tests/invariant_3_factor.rs` pins the same
    /// property one level down, on the faer types this wrapper is built from.
    #[test]
    fn factor_owns_its_data_and_crosses_threads() {
        const fn require_send_sync<T: Send + Sync>() {}
        require_send_sync::<OrderedLlt>();

        let n = 2400;
        let factor = {
            let (lower, mat) = chain_pattern(n);
            let perm = compute_nested_dissection_permutation(&lower, n).expect("ordering");
            let symbolic = symbolic_cholesky(mat.symbolic(), Some(&perm)).expect("symbolic");
            OrderedLlt::try_new_with_symbolic(symbolic, mat.as_ref(), Side::Lower).expect("numeric")
            // `mat` and `perm` drop here — the factor must not have borrowed.
        };

        let mut x: Mat<f64> = Mat::from_fn(n, 1, |i, _| (i as f64).sin());
        std::thread::spawn(move || {
            factor.solve_in_place_with_conj(Conj::No, x.as_mut());
            // A solve through a dangling factor would not reliably produce a
            // finite answer; assert it did.
            assert!(
                (0..n).all(|i| x[(i, 0)].is_finite()),
                "solve through a moved, source-dropped factor produced non-finite output"
            );
        })
        .join()
        .expect("solve thread panicked");
    }

    /// The size policy must gate at the sizes it was DERIVED from.
    ///
    /// ⚠ An earlier version computed both bounds from
    /// `NESTED_DISSECTION_MIN_FREE_DOF` itself, so it held for any value of
    /// the constant and could not catch the threshold typo its doc advertised.
    /// The anchors here are the literal measured sizes from
    /// `factorization_fill_growth`: 2,112, where break-even is ~46
    /// factorizations and the ordering does not pay, and 6,444, where it is
    /// ~10.7 and does. Setting the constant anywhere outside that bracket now
    /// fails.
    #[test]
    fn size_policy_gates_at_the_sizes_it_was_derived_from() {
        /// Measured: break-even ~46 factorizations, ~21 steps. Too expensive.
        const MEASURED_BELOW_CROSSOVER: usize = 2112;
        /// Measured: break-even ~10.7 factorizations, ~5 steps. Pays off.
        const MEASURED_AT_CROSSOVER: usize = 6444;

        let (lower, _) = chain_pattern(MEASURED_BELOW_CROSSOVER);
        assert!(
            nested_dissection_permutation(&lower, MEASURED_BELOW_CROSSOVER).is_none(),
            "n_free = {MEASURED_BELOW_CROSSOVER} took nested dissection, but at that \
             size the ordering needs ~21 steps to repay itself — the threshold has \
             dropped below the measurement it was derived from"
        );

        let (lower, _) = chain_pattern(MEASURED_AT_CROSSOVER);
        assert!(
            nested_dissection_permutation(&lower, MEASURED_AT_CROSSOVER).is_some(),
            "n_free = {MEASURED_AT_CROSSOVER} fell back to AMD, but that is the \
             measured size where nested dissection starts paying — the threshold \
             has risen above its own derivation and the ordering is dead weight"
        );
    }

    /// An empty pattern must not panic.
    ///
    /// ⚠ This guards the DIAGNOSTIC entry point, not production. An earlier
    /// version justified itself with "`n = 0` is reachable for a fully pinned
    /// mesh" — a fully pinned mesh does reach `n_free = 0`, but it never
    /// reaches the code below: production goes through
    /// [`nested_dissection_permutation`], which returns `None` at
    /// `n < NESTED_DISSECTION_MIN_FREE_DOF` and so short-circuits long before
    /// `n = 0`. What is actually covered is
    /// [`compute_nested_dissection_permutation`] — which `factorization_fill_growth`
    /// calls directly, bypassing the size policy.
    ///
    /// What it covers is not a loop iteration anywhere: at `n = 0` every loop in
    /// `full_symmetric_csc_i32` has an empty iterator — `col_start` is `[0]`, so
    /// even `windows(2)` yields nothing. What is exercised is that the builder
    /// returns `Some(([0], []))` rather than failing its `i32` conversions, that
    /// `CscPattern::new` and `metis_order` accept a zero-vertex graph, and that
    /// [`invert_new_to_old`] returns an empty permutation rather than `None` for
    /// a zero-length input.
    #[test]
    fn empty_pattern_orders_without_panicking() {
        let lower = BTreeSet::new();

        // The CSC shape the doc above claims, asserted rather than reasoned
        // about: one column pointer and no rows. `CscPattern` requires
        // `col_ptr.len() == n + 1`, so the lone `0` is load-bearing, not slack.
        let (col_ptr, row_idx) = full_symmetric_csc_i32(&lower, 0).expect("empty csc");
        assert_eq!(
            col_ptr,
            vec![0],
            "empty pattern must still emit `n + 1` pointers"
        );
        assert!(row_idx.is_empty(), "empty pattern emitted rows");

        let perm = compute_nested_dissection_permutation(&lower, 0).expect("empty ordering");
        assert!(perm.forward.is_empty());
        assert!(perm.inverse.is_empty());
    }
}
