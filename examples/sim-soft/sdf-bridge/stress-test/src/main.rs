//! sdf-bridge/stress-test — the sim-soft SDF-bridge validation superset.
//!
//! One headless validator covering both directions of the `cf_design::Sdf`
//! trait as the SDF interop spine — the example counterpart to the sim-soft
//! library's `sim/L0/soft/src/sdf_bridge/` module — folded from two former
//! per-concept examples (each now a module preserving its hand-authored fixture
//! + oracle checks verbatim):
//!
//! - [`mesh_scan`] (row 15) — the **scan → design** direction: a mesh-derived
//!   SDF (`mesh_sdf::Signed<TriMeshDistance, PseudoNormalSign>`) satisfies
//!   `cf_design::Sdf` (PR3 F2). A programmatic 12-triangle cube fixture
//!   (`R = 1.0`) is round-tripped through a runtime-written binary STL on disk
//!   (`save_stl` → `load_mesh`, no checked-in asset), then every numerical
//!   anchor dispatches through `&dyn cf_design::Sdf`: closed-form L∞-ball SDF at
//!   face / edge / vertex / interior probes, a finite-difference gradient
//!   face-band check, an STL round-trip agreement gate, and a 17³ = 4913 bulk
//!   grid consistency pass (the `PseudoNormalSign` inside-set proven equal to
//!   the closed cube `[−R, R]³` = 9³ = 729 grid points, the strict-interior
//!   heuristic bucket = 7³ = 343, and their boundary-shell difference = 386 —
//!   all closed-form geometric identities, none a captured empirical count).
//! - [`solid_to_sim`] (row 16) — the **design → sim** direction: a typed
//!   `cf_design::Solid` CSG body (`Solid::sphere(R_OUTER).subtract(sphere(
//!   R_CAVITY))`) coerces to `&dyn Sdf` and drives
//!   `sim_soft::SdfMeshedTetMesh::from_sdf` (PR3 F1 + F3 re-export). HEADLINE A
//!   proves the bridge is bit-preserving — the typed-`Solid` mesh
//!   `equals_structurally` a `DifferenceSdf<SphereSdf>` baseline with per-vertex
//!   positions bit-equal — then a single-material pressurization FEM readout
//!   validates the cavity-wall mean against single-material Lamé (within 30 %)
//!   and pins it bit-equal to row 11's uniform-1× capture (cross-row continuity).
//!
//! The two are complementary, not subsuming: [`mesh_scan`] is the sole coverage
//! of the mesh-SDF → `cf_design::Sdf` direction (triangle-soup source, closed-
//! form cube oracle); [`solid_to_sim`] is the sole coverage of the typed-`Solid`
//! → sim-soft meshing + FEM direction (CSG source, bridge-equivalence + Lamé
//! oracle) — opposite ends of the same trait spine.
//!
//! Each module self-gates against closed-form / bit-equal / convergence oracles
//! and aborts (exit 101) on any mismatch, so `cargo xtask run-validators` runs
//! it red-or-green. Each writes its cf-view artifacts to `out/`
//! (`mesh_scan`: `cube_scan.stl` + `sdf_grid.ply`; `solid_to_sim`:
//! `shell_zslab.ply`; distinct filenames, no namespacing needed).

mod mesh_scan;
mod solid_to_sim;

use anyhow::Result;

fn main() -> Result<()> {
    mesh_scan::run()?;
    solid_to_sim::run()?;
    Ok(())
}
