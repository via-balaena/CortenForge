# Sparse-solver papers

The iterative-solver and preconditioner cluster cited inline from [Part 8 Ch 02](../../80-gpu/02-sparse-solvers.md) (GPU sparse solvers — CG, MINRES, AMG, preconditioning). Three clusters: the CG/MINRES Krylov-method pair, the AMG multigrid literature, and the mixed-precision iterative-refinement background. Each anchor below is traced to the specific Ch 02 sub-leaf and claim the cluster's paper grounds.

## CG, MINRES, and Krylov methods

## Saad 2003 {#saad-2003}

*Iterative Methods for Sparse Linear Systems*, 2nd edition. SIAM (Other Titles in Applied Mathematics, vol. 82), 2003. Author: Yousef Saad (University of Minnesota). ISBN 978-0-89871-534-7.

The canonical textbook for Krylov-subspace iterative methods on sparse matrices. Chapters 6–8 cover CG, MINRES, GMRES, and short-recurrence variants; Chapters 9–10 cover preconditioner construction (Jacobi, ILU, IC, multigrid). Cited inline from [Part 8 Ch 02 §00 CG](../../80-gpu/02-sparse-solvers/00-cg.md) as the reference for Hestenes-Stiefel CG (Chapter 6), from [Part 8 Ch 02 §01 MINRES](../../80-gpu/02-sparse-solvers/01-minres.md) as the textbook treatment of symmetric indefinite Krylov methods, and from [§03 preconditioning](../../80-gpu/02-sparse-solvers/03-preconditioning.md) for the tiered preconditioner strategy. The 2nd edition's website at [www-users.cs.umn.edu/~saad/books.html](https://www-users.cs.umn.edu/~saad/books.html) hosts an authorized PDF of the book; Pass 3 may add chapter-level page anchors.

## Paige & Saunders 1975 {#paige-saunders-1975}

*Solution of Sparse Indefinite Systems of Linear Equations.* SIAM Journal on Numerical Analysis 12(4), pp. 617–629, 1975. Authors: Christopher Paige (McGill), Michael Saunders (Stanford). DOI [10.1137/0712047](https://doi.org/10.1137/0712047).

The MINRES original — minimum-residual iterative solver for symmetric indefinite systems via Lanczos tridiagonalization plus QR updates. The short-recurrence property (three-term orthogonalization against the last two iterates, not all previous) is the paper's central contribution and is what keeps MINRES's per-iteration memory bounded. Cited inline from [Part 8 Ch 02 §01 MINRES](../../80-gpu/02-sparse-solvers/01-minres.md) as the algorithmic reference `sim-soft`'s GPU MINRES implementation follows. The same authors' earlier 1975 Lanczos-related work (Paige's 1971 PhD thesis on the Lanczos algorithm's numerical stability) underpins the short-recurrence derivation but is not separately anchored here — the 1975 SIAM paper is self-contained for the algorithm itself.

## Algebraic multigrid

## Trottenberg, Oosterlee & Schüller 2001 {#trottenberg-oosterlee-schuller-2001}

*Multigrid.* Academic Press, 2001. Authors: Ulrich Trottenberg, Cornelis W. Oosterlee, Anton Schüller (all at GMD-SCAI at press time; Trottenberg and Schüller continued at Fraunhofer SCAI after the July 2001 Fraunhofer absorption, Oosterlee moved to CWI/TU Delft around the same period). ISBN 978-0-12-701070-0.

The canonical multigrid textbook. Covers both geometric multigrid (Chapters 2–4) and algebraic multigrid (Appendix A by Klaus Stüben, which became the [standalone Stüben 1999 reference](#stuben-1999)). Cited inline from [Part 8 Ch 02 §02 AMG](../../80-gpu/02-sparse-solvers/02-amg.md) as the background text for V-cycle structure, smoother choice (Jacobi vs. Gauss-Seidel), and convergence analysis. `sim-soft`'s AMG implementation follows the textbook's V-cycle shape and the Stüben-appendix coarsening; the Chapter-5-onward full multigrid and F-cycle variants are available but not used in Phase E.

## Stüben 1999 {#stuben-1999}

*Algebraic Multigrid (AMG): An Introduction with Applications.* Journal of Computational and Applied Mathematics 128(1–2), pp. 281–309, 2001; originally circulated as GMD Report 70, November 1999. Author: Klaus Stüben (GMD-SCAI, later Fraunhofer SCAI). DOI [10.1016/S0377-0427(00)00516-1](https://doi.org/10.1016/S0377-0427(00)00516-1).

The book uses the "Stüben 1999" shorthand because the GMD Report 70 predates the 2001 JCAM publication and was the version the classical-AMG community cited for two years before the journal paper appeared; the DOI above resolves to the journal version.

The practitioner-facing survey of classical (Ruge-Stüben) algebraic multigrid. Specifies the strong-connection threshold ($\theta \in [0.25, 0.5]$), the greedy C/F-point selection algorithm, and the classical interpolation-weight formula that `sim-soft`'s AMG implementation uses. Cited inline from [Part 8 Ch 02 §02 AMG](../../80-gpu/02-sparse-solvers/02-amg.md) as the algorithmic reference for the coarsening pass and the interpolation operator; from [§03 preconditioning](../../80-gpu/02-sparse-solvers/03-preconditioning.md) as the tie-in to Saad 2003's preconditioner chapters. The Stüben survey is the single most-cited reference for "classical AMG" in the modern literature; the more recent Smoothed Aggregation (Vaněk et al.) and BoomerAMG (hypre) variants are not anchored in Pass 1 because `sim-soft`'s GPU implementation commits to classical Ruge-Stüben.

## Mixed-precision iterative refinement

## Carson & Higham 2018 {#carson-higham-2018}

*Accelerating the Solution of Linear Systems by Iterative Refinement in Three Precisions.* SIAM Journal on Scientific Computing 40(2), pp. A817–A847, 2018. Authors: Erin Carson (Courant Institute of Mathematical Sciences, NYU; later Charles University, Prague), Nicholas Higham (University of Manchester). DOI [10.1137/17M1140819](https://doi.org/10.1137/17M1140819).

The modern reference for mixed-precision iterative refinement — compute the factorization (or SpMV) at lower precision, refine the residual at higher precision, recover full accuracy. `sim-soft`'s GPU CG runs SpMV and preconditioner apply in `f32` and accumulates dot products in `f64` (per [Ch 02 parent Claim 3](../../80-gpu/02-sparse-solvers.md)); the single-step iterative refinement at `f64` SpMV that follows CG convergence is the technique Carson and Higham's paper formalizes. Cited inline from [Part 8 Ch 02 §00 CG](../../80-gpu/02-sparse-solvers/00-cg.md) as the theoretical grounding for the mixed-precision gradient-consistency claim ([Phase E 5-digit gradcheck](../../110-crate/04-testing/03-gradcheck.md)).

## Implementations

## faer {#faer}

*faer.* Pure-Rust sparse + dense linear algebra crate. Repo: [`github.com/sarah-quinones/faer-rs`](https://github.com/sarah-quinones/faer-rs). Maintainer: sarah-ek. License: MIT. Accessed: 2026-04-20.

The crate `sim-soft` commits to for CPU sparse linear algebra per [Part 11 Ch 03 Phase B](../../110-crate/03-build-order.md#the-committed-order). Two features make it load-bearing for the stack: (a) **pure Rust** — no FFI, no C/C++ dependency added to the build graph; (b) **factorizations as first-class objects** — `Llt<f64>`, Sparse LU, and Sparse Cholesky can be re-applied to arbitrary right-hand sides, which is exactly the shape [Part 6 Ch 02 IFT adjoint](../../60-differentiability/02-implicit-function.md) needs (one factor, many RHSes per backward pass). [Part 11 Ch 04 §03 gradcheck](../../110-crate/04-testing/03-gradcheck.md) pins the `Llt<f64>` factor-on-tape as the mechanism that makes IFT cheap at the tape level. sarah-ek's shipping cadence on sparse direct and iterative solvers lines up with `sim-soft`'s Phase B timing per [Part 11 Ch 03's justification](../../110-crate/03-build-order.md#the-committed-order). Cited inline from [Part 11 Ch 03 Phase B](../../110-crate/03-build-order.md#the-committed-order), [Part 11 Ch 04 §03 gradcheck](../../110-crate/04-testing/03-gradcheck.md), and as the CPU oracle for [Part 8 Ch 02 §00 CG](../../80-gpu/02-sparse-solvers/00-cg.md)'s GPU port.
