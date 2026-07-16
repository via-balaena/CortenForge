//! material/stress-test — the sim-soft silicone-material reference-table demo.
//!
//! One headless `demo` over the sim-soft *material-reference* surface: the
//! production silicone constants table, its compressible-Neo-Hookean stress +
//! energy shown alongside each entry's Smooth-On data-sheet value. Currently a
//! single module (the `material` domain has one capability example); the
//! dispatcher shape matches the sibling `<domain>/stress-test/` folds (sdf /
//! scalar-field / stretch / bonded) so a second material example would slot in
//! as another `mod` + `run()` call with no restructuring.
//!
//! - [`material_table`] (row 19) — direct-eval consumer of F4's
//!   `sim_soft::material::silicone_table`: iterates the seven `pub const
//!   SiliconeMaterial` entries (`{ECOFLEX_00_10/20/30/50, DRAGON_SKIN_10A/20A/
//!   30A}`), dispatches each via `SiliconeMaterial::to_neo_hookean()`, and
//!   probes the resulting `NeoHookean` at `F = diag(2.0, 1, 1)` (the data-sheet
//!   `σ_100 = 100 % engineering strain` anchor). No solver, no mesh — the
//!   constitutive table is the artifact.
//!
//! This is a demonstration, not a validator: it does NOT self-gate. The
//! underlying correctness (`NeoHookean` closed-form stress/energy + transverse
//! symmetry; the `λ = 4μ` and hardness-ordering invariants) is validated by
//! `neo_hookean.rs` and `silicone_table.rs` unit tests. Output is
//! `out/silicone_materials.json` (a programmatic-consumption lookup; JSON-only,
//! no cf-view artifact per inventory Q4 row 19).

mod material_table;

use anyhow::Result;

fn main() -> Result<()> {
    material_table::run()?;
    Ok(())
}
