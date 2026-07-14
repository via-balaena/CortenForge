//! printability/stress-test — the `mesh-printability` domain validation superset.
//!
//! One headless validator covering the v0.8 manufacturability-detector
//! inventory, folded from seven former per-crate examples (each now a
//! module preserving its hand-authored fixture + oracle checks verbatim):
//!
//! - [`long_bridge`] — `LongBridge` (§6.2) on a 24-vertex H-shape: the
//!   20 mm middle bridge emits while the 5 mm cantilevers silent-drop;
//!   cross-detector coincidence (bridge midpoint ≡ middle-overhang
//!   centroid) + SLS policy-skip semantics.
//! - [`trapped_volume`] — `TrappedVolume` (§6.3) on a cube + sealed
//!   sphere cavity: analytical `(4/3)·π·r³` volume, voxel centroid, and
//!   the per-technology severity fork (FDM `Info`; SLA/SLS/MJF `Critical`).
//! - [`self_intersecting`] — `SelfIntersecting` (§6.4) on two
//!   interpenetrating cylinders: the `max_reported = 100` cap
//!   biconditional, `face_a < face_b` ordering, and a convex-cylinder
//!   no-false-positive control (`== 0`).
//! - [`small_feature`] — `SmallFeature` (§6.5) on a cube + 0.2 mm hex
//!   burr: the SmallFeature↔ThinWall two-detector convergence (Warning
//!   vs Critical), divergence-theorem volume, and negative controls.
//! - [`orientation`] — `build_up_direction` (Gap L) on a leaning
//!   cylinder: `with_build_up_direction` / `apply_orientation`
//!   equivalence + closed-form overhang-centroid envelope (radial band +
//!   downhill azimuth arc).
//! - [`technology_sweep`] — the cross-technology severity matrix: one
//!   hollow box with a 0.4 mm top wall validated under FDM/SLA/SLS/MJF,
//!   each failing `is_printable()` for a different reason. Its FDM slice
//!   is the canonical hollow-box `ThinWall` oracle (subsumes the former
//!   `printability-thin-wall` example, dropped in this fold).
//! - [`showcase`] — the capstone: a 528-vertex, five-shell bracket
//!   exercising all six detectors at once, with `>=` predicates that
//!   absorb cross-detector co-flag drift on a realistic multi-part mesh.
//!
//! Each module self-gates against closed-form / analytic oracles and
//! aborts (exit 101) on any mismatch, so `cargo xtask run-validators`
//! runs it red-or-green. Each also writes its human-inspectable PLY
//! artifacts to `out/<module>/` for the `cf-viewer` visual-review path.
//!
//! `--release` is recommended: the `showcase` and `trapped_volume`
//! voxel-grid `TrappedVolume` detectors are tractable in release, slow
//! in debug.

mod long_bridge;
mod orientation;
mod self_intersecting;
mod showcase;
mod small_feature;
mod technology_sweep;
mod trapped_volume;

use anyhow::Result;

fn main() -> Result<()> {
    long_bridge::run()?;
    trapped_volume::run()?;
    self_intersecting::run()?;
    small_feature::run()?;
    orientation::run()?;
    technology_sweep::run()?;
    showcase::run()?;
    Ok(())
}
