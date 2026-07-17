//! stretch/stress-test — the sim-soft uniaxial-stretch validation superset.
//!
//! One headless validator covering the sim-soft uniaxial-stretch surface on the
//! canonical compressible Neo-Hookean baseline (`μ = 1e5`, `Λ = 4e5` Pa,
//! `ν ≈ 0.4`) — a deliberate solver → constitutive → assembly ladder, folded
//! from three former per-concept examples (each now a module preserving its
//! hand-authored fixture + oracle checks verbatim):
//!
//! - [`single_tet`] (row 4) — `SkeletonSolver::step` end-to-end on
//!   `SoftScene::one_tet_cube`: a `θ = 10 N` `+ẑ` traction on `v_3` with
//!   `v_0..v_2` Dirichlet-pinned, converging in 3 Newton iters. Anchors the
//!   walking-skeleton solver path with 12 IV-1 `x_final` bit-pins + an exact
//!   iter-count pin + a dimensional displacement band.
//! - [`neo_hookean`] (row 5) — the traction-free uniaxial force-stretch
//!   CURVE (`F = diag(λ, λ_t, λ_t)`, `λ_t` from a 1-D inner Newton) driven
//!   through the real `NeoHookean` across a 12-point sweep and emitted as
//!   JSON (the artifact). No solver — the constitutive law is exercised in
//!   isolation. Demonstration self-gates: inner-Newton convergence,
//!   traction-free `P_22`/`P_33`, `P_11` monotonicity + sign, sweep-in-
//!   domain. Constitutive closed-form CORRECTNESS is owned by the `sim-soft`
//!   `NeoHookean` lib tests (`diag(s,1,1)` + `diag(a,b,b)`); the JSON/plot
//!   carry the real observed curve only (no analytic overlay).
//! - [`multi_element`] (row 6) — Phase 2 multi-element FEM assembly on a
//!   27-vertex / 48-tet `HandBuiltTetMesh::uniform_block(2)` under uniform
//!   Dirichlet stretch (`λ = 1.20`, one interior vertex free), quasi-static via
//!   `cfg.density = 0`: per-tet `F` uniform `diag(λ, 1, 1)` across all 48 tets
//!   and `P_11` / `P_22` / ψ vs closed form, plus 10 sparse-tier bit-pins.
//!
//! The three are complementary, not subsuming: [`single_tet`] is the sole
//! coverage of the single-element end-to-end `Solver::step` path; [`neo_hookean`]
//! is the constitutive-law direct DEMONSTRATION (no solver, F hand-set — the
//! force-stretch curve; correctness owned by the lib as noted above);
//! [`multi_element`] is the sole coverage of the multi-element assembly path
//! (local → global stitch across 48 tets, sparse Cholesky on the free block,
//! and a per-tet uniformity oracle) — an axis neither single-element sibling
//! can reach.
//!
//! Each module self-gates and aborts (exit 101) on any mismatch, so
//! `cargo xtask run-validators` runs it red-or-green. The gate KIND varies
//! by module: [`single_tet`] and [`multi_element`] carry closed-form /
//! bit-equal / convergence oracles; [`neo_hookean`] is a demonstration whose
//! constitutive correctness lives in the `sim-soft` lib tests, so it gates
//! only on demonstration-integrity + curve-shape properties. Each writes its
//! human-inspectable JSON trace to a namespaced `out/<module>/` directory;
//! [`neo_hookean`] additionally has a companion `plot.py` (PEP 723 `uv run`)
//! that renders its sweep to `out/neo_hookean/`.

mod multi_element;
mod neo_hookean;
mod single_tet;

use anyhow::Result;

fn main() -> Result<()> {
    single_tet::run()?;
    neo_hookean::run()?;
    multi_element::run()?;
    Ok(())
}
