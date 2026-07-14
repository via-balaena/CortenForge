//! offset/stress-test — the `mesh-offset` domain validation superset.
//!
//! One headless validator for `mesh_offset::offset_mesh`, folded from
//! the two former `mesh-offset-inward` / `mesh-offset-outward` examples
//! (each now a module). It exercises SDF + marching-cubes offset of a
//! unit cube in both directions and self-gates against closed-form
//! oracles:
//!
//! - [`outward`] — dilation (`d = +0.1`): **Steiner-Minkowski rounding**.
//!   The offset surface grows rounded edges and corners, so the volume
//!   follows `V_d = 1 + 6d + 3πd² + (4π/3)d³` (cube + face slabs + edge
//!   quarter-cylinders + corner sphere octants).
//! - [`inward`] — erosion (`d = -0.1`): **polytope preservation**. A
//!   convex polytope offset inward stays a smaller sharp polytope (no
//!   features to round), so the volume is the exact `V_d = (1 + 2d)³`.
//!
//! Both directions now come straight out of `offset_mesh` as clean,
//! watertight, outward-wound (`signed_volume > 0`) 2-manifolds — the
//! §Q-5 winding fix (`mesh-offset/src/marching_cubes.rs`, guarded by
//! `marching_cubes_produces_outward_winding_on_cube_sdf`) and the MC
//! edge-vertex cache mean no per-face flip or `weld_vertices` follow-up
//! is needed. (Historically these examples demonstrated a per-face
//! winding-flip workaround for inside-out vertex-soup output; both the
//! inside-out winding and the vertex-soup are gone, so that narrative
//! retired with §Q-5.)
//!
//! The [`inward`] module additionally demonstrates the **grid-alignment
//! topology pitfall**: when the eroded cube's flat faces land exactly on
//! marching-cubes sample planes the SDF is zero across a whole face,
//! MC's cell classification is ambiguous, and the output picks up
//! spurious handles (non-zero genus) even though it stays closed and its
//! volume stays correct. Nudging the resolution off-grid restores a
//! clean genus-0 manifold. Outward offset is robust to the same
//! alignment because its rounded Steiner surface never coincides with a
//! full sample plane — the [`outward`] module asserts that contrast.
//!
//! Each module self-gates against its oracle and aborts (exit 101) on
//! any mismatch, so `cargo xtask run-validators` runs it red-or-green.
//! Each also writes its clean genus-0 mesh to `out/` (`inward.ply`,
//! `outward.ply`) for the `cf-viewer` visual-review path.

mod diag;
mod inward;
mod outward;

use anyhow::Result;

fn main() -> Result<()> {
    outward::run()?;
    inward::run()?;
    Ok(())
}
