//! shell-generation/stress-test — the `mesh-shell` domain validation
//! superset.
//!
//! One headless validator for `mesh_shell::ShellBuilder`, folded from the
//! former `shell-generation-fast` / `-high-quality` examples (each now a
//! module). It exercises both wall-generation methods on the same 10mm
//! cube geometry and self-gates against closed-form oracles:
//!
//! - [`fast`] — the `.fast()` preset (normal-based offset) on an
//!   open-topped box. Duplicates each input vertex along its averaged
//!   normal (1:1 inner↔outer correspondence), closes the open top with a
//!   rim, and checks the per-vertex offset table + the rim's consistent
//!   winding.
//! - [`high_quality`] — the `.high_quality()` preset (SDF + marching
//!   cubes) on a closed cube. Marches the outer surface from the SDF
//!   level set for UNIFORM perpendicular wall thickness regardless of
//!   input triangulation, at the cost of MC re-triangulation (no 1:1
//!   correspondence).
//!
//! The load-bearing contrast is **triangulation-dependent vs uniform wall
//! thickness**: the normal method skews perpendicular thickness by the
//! incident-triangle count at each vertex (down to 0.667mm at a 3:1 vertex
//! for a nominal 2mm wall), while the SDF method holds 2mm to within
//! half a voxel everywhere.
//!
//! Both methods now produce **consistently-wound** shells: the SDF path
//! via the 11.5.x per-face-flip engine fix, and the normal path via the
//! rim-winding fix in `mesh_shell::shell::rim::generate_rim` (rim quads
//! wound to oppose the reversed-inner / original-outer surface edges).
//! (Historically the normal path shipped a rim-winding inconsistency —
//! watertight + manifold + printable, but `has_consistent_winding ==
//! false`; that quirk is gone, guarded by the lib test
//! `test_normal_shell_on_open_box_has_consistent_winding`.)
//!
//! Each module self-gates against its oracle and aborts (exit 101) on any
//! mismatch, so `cargo xtask run-validators` runs it red-or-green. Each
//! also writes its shell to `out/` (`fast_shell.ply`, `hq_shell.ply`) for
//! the `cf-viewer` visual-review path.

mod common;
mod fast;
mod high_quality;

use anyhow::Result;

fn main() -> Result<()> {
    fast::run()?;
    high_quality::run()?;
    Ok(())
}
