//! lattice/stress-test — the `mesh-lattice` domain validation superset.
//!
//! One headless validator covering the `mesh-lattice` generation paths,
//! folded from five former per-crate examples (each now a module
//! preserving its hand-authored fixture + oracle checks verbatim):
//!
//! - [`tpms_gyroid`] — TPMS implicit-surface path: the `gyroid` field +
//!   `density_to_threshold` + `make_shell` closed-form anchors, then
//!   `generate_lattice`, verifying every marching-cubes vertex lands on
//!   the analytical shell surface.
//! - [`strut_cubic`] — cubic strut path: `generate_strut` /
//!   `combine_struts` / `estimate_strut_volume` free fns (closed-form
//!   cylinder/cone volumes), then `generate_lattice` with beam export —
//!   the `BeamLatticeData` 3MF precursor (216 grid nodes + 540 beams).
//! - [`density_gradient`] — the full `DensityMap` subsystem (`Uniform` /
//!   `Gradient` / `Radial` / `Function` / `SurfaceDistance` / `StressField`), with
//!   per-beam `r1 = strut_thickness/2 · √density` spatial modulation on
//!   an octet-truss.
//! - [`shape_bounded`] — analytical-SDF clipping via `with_shape_sdf`:
//!   a gyroid trimmed to a sphere, verifying the `is_outside_shape`
//!   predicate and lattice confinement within the bounding shape.
//! - [`mesh_bounded_infill`] — the composite FDM pipeline via
//!   `generate_infill` on a watertight input mesh: shell + lattice +
//!   caps + connections, with the §Q-5 hollow-shell winding guard. Its
//!   50 mm cube fixture lives in the [`mesh_bounded_infill::fixture`]
//!   submodule.
//!
//! Each module self-gates against closed-form / analytic oracles and
//! aborts (exit 101, or `Err` from `run()`) on any mismatch, so
//! `cargo xtask run-validators` runs it red-or-green. Each also writes
//! its human-inspectable PLY artifacts to `out/` for the `cf-viewer`
//! visual-review path.

mod density_gradient;
mod mesh_bounded_infill;
mod shape_bounded;
mod strut_cubic;
mod tpms_gyroid;

use anyhow::Result;

fn main() -> Result<()> {
    tpms_gyroid::run()?;
    strut_cubic::run()?;
    density_gradient::run()?;
    shape_bounded::run()?;
    mesh_bounded_infill::run()?;
    Ok(())
}
