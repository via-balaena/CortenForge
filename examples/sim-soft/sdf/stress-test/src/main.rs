//! sdf/stress-test — the sim-soft `Sdf` domain validation superset.
//!
//! One headless validator covering the sim-soft SDF surface — primitive
//! field eval, sharp-CSG composition, and SDF → tet meshing — folded from
//! three former per-concept examples (each now a module preserving its
//! hand-authored fixture + oracle checks verbatim):
//!
//! - [`sphere_eval`] — `Sdf` trait contract on `SphereSdf` (`‖p‖ − 1`
//!   signed distance, `p/‖p‖` unit gradient) plus a discrete
//!   Eikonal-property diagnostic on the sampled grid.
//! - [`hollow_shell`] — sharp-CSG `DifferenceSdf` combinator
//!   (`φ = max(φ_a, −φ_b)`): branch tie-break determinism and
//!   inward-normal cavity semantics on `SphereSdf{1.0} \ SphereSdf{0.5}`.
//! - [`sdf_to_tet`] — `SdfMeshedTetMesh::from_sdf` BCC + Labelle-Shewchuk
//!   Isosurface Stuffing pipeline: exact counts, Theorem-1 quality bounds,
//!   Euler-characteristic topology, and volume convergence on a solid sphere.
//!
//! Each module self-gates against closed-form / analytic / topology oracles
//! and aborts (exit 101) on any mismatch, so `cargo xtask run-validators`
//! runs it red-or-green. Each also writes its human-inspectable PLY artifact
//! to `out/` (`sdf_grid.ply`, `sdf_slice.ply`, `sphere_boundary.ply`) for the
//! `cf-viewer` visual-review path.

mod hollow_shell;
mod sdf_to_tet;
mod sphere_eval;

use anyhow::Result;

fn main() -> Result<()> {
    sphere_eval::run()?;
    hollow_shell::run()?;
    sdf_to_tet::run()?;
    Ok(())
}
