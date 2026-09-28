//! Lowering into the explicit solver (plan §14a, §15g step 6, §16w): a meshed wall, how it is held, and the path
//! of the rigid scan pressed into it, as `sim-soft-explicit`'s data.

pub mod hold;
pub mod model;
pub mod path;

pub use hold::{Plane, Skin};
pub use model::{Lowered, Lowering, LoweringError, lower};

use crate::obstacle::ObstacleBake;

/// The product's bake (plan §16u, §16w).
///
/// A coarse grid at 0.5 mm with a 4 mm margin, a fine one at 0.0625 mm (Jon, 2026-09-27), and a band of eight fine
/// cells. The band started at four; on `base_mold` a frictionless run's deepest predicted point reached 0.91 of that,
/// past half, so §16w's rule set it to twice that point, rounded up to whole fine cells. Every run reads the deepest
/// predicted point and the corrections the coarse grid answered.
pub const PRODUCT_BAKE: ObstacleBake = ObstacleBake {
    coarse_cell: 0.000_5,
    fine_cell: 0.000_062_5,
    band: 0.000_5,
    margin: 0.004,
};

/// How far outside the scan every wall node lies at the run's start: plan §15b's start gap, taken as a clearance
/// (plan §16w).
pub const START_CLEARANCE: f64 = 0.005;

/// How far the sampled path may stray from the fitted one: G2's floor (fit plan §5, Jon, 2026-09-27).
pub const SAMPLING_BAR: f64 = 0.000_02;
