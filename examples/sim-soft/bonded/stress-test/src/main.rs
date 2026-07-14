//! bonded/stress-test — the sim-soft bonded-multi-material validation superset.
//!
//! One headless validator covering the sim-soft *bonded multi-material* surface:
//! two solver-driven scenes whose shared-vertex (C⁰-continuous, no-slip)
//! multi-material bodies are validated against a closed-form elasticity
//! solution, folded from two former per-concept examples (each now a module
//! preserving its hand-authored fixture + oracle checks verbatim):
//!
//! - [`bilayer_beam`] (row 10) — a shared-vertex bonded bilayer cantilever
//!   beam (`(0.5, 0.1, 0.1) m`, `(NX, NY, NZ) = (20, 8, 8)` → 1701 verts /
//!   7680 tets) under a `1 N` tip load, meshed via
//!   `HandBuiltTetMesh::cantilever_bilayer_beam` with an inline half-space
//!   `Field<f64>` partitioning tets into region A (`1×` Ecoflex, `z < H/2`)
//!   and region B (`2×` Decision J composite, `z ≥ H/2`). The tip displacement
//!   is matched against the **Euler-Bernoulli composite (transformed-section)
//!   beam** analytic within 30 % at h/2; a strict-between-uniform-bounds
//!   inequality is the IV-2-lens-β discriminator. First IV-2 shared-vertex
//!   displacement-continuity gate at production scale.
//! - [`lame_shells`] (row 11) — a three-shell concentric hollow silicone
//!   sphere (`DifferenceSdf` of two `SphereSdf`s, `R_OUTER = 0.10 m`,
//!   `R_CAVITY = 0.04 m`, 6456 tets) meshed via
//!   `SoftScene::layered_silicone_sphere` (BCC + Labelle-Shewchuk) with a
//!   3-shell `MaterialField` per Decision J's `1× / 2× / 1×` symmetry, under a
//!   per-vertex radially-outward pressure traction (`LoadAxis::FullVector`) on
//!   the cavity surface and a fixed Dirichlet pin on the outer surface. Four
//!   Saint-Venant-averaged radial-displacement readouts (cavity-wall +
//!   per-shell) are matched against the **piecewise-Lamé thick-shell 6×6
//!   closed-form** within 30 % at h/2; the same IV-2-lens-β strict-between
//!   cavity-wall gate runs three full solver passes.
//!
//! The two are complementary, not subsuming: [`bilayer_beam`] is the sole
//! coverage of the hand-built shared-vertex bilayer + Euler-Bernoulli
//! composite-beam bending path (axial tip force, `HandBuiltTetMesh`);
//! [`lame_shells`] is the sole coverage of the SDF-meshed hollow-body 3-shell +
//! piecewise-Lamé pressurized-shell path (radial `FullVector` pressure +
//! Dirichlet outer, `SdfMeshedTetMesh` / `SoftScene`) — a mesher, boundary
//! condition, and closed-form analytic that its bilayer sibling cannot reach.
//!
//! Each module self-gates against closed-form / bit-equal / convergence oracles
//! and aborts (exit 101) on any mismatch, so `cargo xtask run-validators` runs
//! it red-or-green. Each writes its cf-view PLY point-cloud artifact to `out/`
//! (`bilayer_beam.ply` / `concentric_lame_shells.ply`; distinct filenames, no
//! namespacing needed).

mod bilayer_beam;
mod lame_shells;

use anyhow::Result;

fn main() -> Result<()> {
    bilayer_beam::run()?;
    lame_shells::run()?;
    Ok(())
}
