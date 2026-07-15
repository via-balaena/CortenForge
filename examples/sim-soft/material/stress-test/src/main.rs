//! material/stress-test — the sim-soft silicone-material validation superset.
//!
//! One headless validator covering the sim-soft *material-reference* surface:
//! the production silicone constants table validated against closed-form
//! compressible Neo-Hookean stress + energy. Currently a single module (the
//! `material` domain has one capability example); the dispatcher shape matches
//! the sibling `<domain>/stress-test/` folds (sdf / scalar-field / stretch /
//! bonded) so a second material example would slot in as another `mod` + `run()`
//! call with no restructuring.
//!
//! - [`material_table`] (row 19) — direct-eval consumer of F4's
//!   `sim_soft::material::silicone_table`: iterates the seven `pub const
//!   SiliconeMaterial` entries (`{ECOFLEX_00_10/20/30/50, DRAGON_SKIN_10A/20A/
//!   30A}`), dispatches each via `SiliconeMaterial::to_neo_hookean()`, and
//!   probes the resulting `NeoHookean` at `F = diag(2.0, 1, 1)` (the data-sheet
//!   `σ_100 = 100 % engineering strain` anchor). Seven anchor groups (ν = 0.40
//!   invariant + rest-config-zero + closed-form `P_11` / `P_22` / ψ at rel-tol
//!   `1e-12` + hardness ordering + 21 captured-bit self-pins). No solver, no
//!   mesh — the constitutive table is the artifact.
//!
//! The module self-gates against closed-form / bit-equal / ordering oracles and
//! aborts (exit 101) on any mismatch, so `cargo xtask run-validators` runs it
//! red-or-green. Output is `out/silicone_materials.json` (a
//! programmatic-consumption lookup; JSON-only, no cf-view artifact per inventory
//! Q4 row 19).

mod material_table;

use anyhow::Result;

fn main() -> Result<()> {
    material_table::run()?;
    Ok(())
}
