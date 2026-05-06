//! Up-axis convention: input frame → Bevy Y-up coordinate swap.
//!
//! Workspace input data (PLYs, soft-mesh node positions, sim-core rigid-body
//! coords) is typically Z-up (mesh-v1.0 build-plate convention) or Y-up.
//! Bevy is internally Y-up. The mapping from input → Bevy lives here so every
//! downstream consumer (scene placement, lights, orbit camera, mesh
//! conversion) can treat positions as Bevy-Y-up without re-deriving the
//! conversion.
//!
//! The mapping is a single-axis swap parameterized by `--up=<+X|+Y|+Z>`
//! (or equivalently a [`UpAxis`] resource):
//!
//! - [`UpAxis::PlusZ`]: input `(x, y, z)` → Bevy `(x, z, y)` — workspace
//!   default; Y↔Z swap inverts handedness, so triangle winding flips.
//! - [`UpAxis::PlusY`]: input `(x, y, z)` → Bevy `(x, y, z)` — identity;
//!   no winding flip.
//! - [`UpAxis::PlusX`]: input `(x, y, z)` → Bevy `(y, x, z)` — X↔Y swap
//!   inverts handedness, so triangle winding flips.
//!
//! Stored as a Bevy `Resource`; systems read `Res<UpAxis>` and call the
//! [`UpAxis::to_bevy_point`] / [`UpAxis::to_bevy_normal`] / [`UpAxis::flips_winding`]
//! methods at the boundary.

#![allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy meshes

use bevy::prelude::Resource;
use mesh_types::{Point3, Vector3};

/// Which input axis maps to Bevy's +Y. `+Z` is the workspace default
/// (mesh-v1.0 build-plate convention; matches the legacy `f3d --up=+Z`).
#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UpAxis {
    /// Input X axis is up — `(x, y, z)` → Bevy `(y, x, z)`. Winding flips.
    PlusX,
    /// Input Y axis is up — identity. Winding does not flip.
    PlusY,
    /// Input Z axis is up — `(x, y, z)` → Bevy `(x, z, y)`. Winding flips.
    /// Workspace default.
    #[default]
    PlusZ,
}

impl UpAxis {
    /// `true` when the swap inverts handedness (parity-flipping). Triangle
    /// winding must be reversed in this case to keep CCW front-facing.
    #[must_use]
    pub fn flips_winding(self) -> bool {
        !matches!(self, Self::PlusY)
    }

    /// Project an input-frame `Point3<f64>` to a Bevy `[f32; 3]` (Y-up)
    /// under this up-axis convention.
    ///
    /// - [`UpAxis::PlusZ`] — `(x, y, z)` → `(x, z, y)` (workspace default).
    /// - [`UpAxis::PlusY`] — identity.
    /// - [`UpAxis::PlusX`] — `(x, y, z)` → `(y, x, z)`.
    #[inline]
    #[must_use]
    pub fn to_bevy_point(self, p: &Point3<f64>) -> [f32; 3] {
        let (x, y, z) = (p.x as f32, p.y as f32, p.z as f32);
        match self {
            Self::PlusX => [y, x, z],
            Self::PlusY => [x, y, z],
            Self::PlusZ => [x, z, y],
        }
    }

    /// Project an input-frame `Vector3<f64>` normal to a Bevy `[f32; 3]`
    /// (Y-up). Same swap matrix as [`UpAxis::to_bevy_point`]; normals
    /// transform identically to points under axis-only swaps (no inversion
    /// needed for parity flips because PBR's `cull_mode = None` +
    /// `double_sided = true` handles winding).
    #[inline]
    #[must_use]
    pub fn to_bevy_normal(self, n: &Vector3<f64>) -> [f32; 3] {
        let (x, y, z) = (n.x as f32, n.y as f32, n.z as f32);
        match self {
            Self::PlusX => [y, x, z],
            Self::PlusY => [x, y, z],
            Self::PlusZ => [x, z, y],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn up_axis_default_is_plus_z() {
        assert_eq!(UpAxis::default(), UpAxis::PlusZ);
    }

    #[test]
    fn up_axis_flips_winding_for_reflective_swaps_only() {
        assert!(UpAxis::PlusX.flips_winding());
        assert!(!UpAxis::PlusY.flips_winding());
        assert!(UpAxis::PlusZ.flips_winding());
    }

    /// Z-up → Y-up swap on a known position. Physics `(1, 2, 3)` should
    /// land in Bevy as `(1, 3, 2)` (Y/Z swapped).
    #[test]
    fn to_bevy_point_plus_z_swaps_y_and_z() {
        let p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(UpAxis::PlusZ.to_bevy_point(&p), [1.0_f32, 3.0, 2.0]);
    }

    /// `+Y` is identity: the input is already Bevy-Y-up, no swap.
    #[test]
    fn to_bevy_point_plus_y_is_identity() {
        let p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(UpAxis::PlusY.to_bevy_point(&p), [1.0_f32, 2.0, 3.0]);
    }

    /// `+X` swaps X↔Y so input X (the up axis) lands in Bevy +Y.
    #[test]
    fn to_bevy_point_plus_x_swaps_x_and_y() {
        let p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(UpAxis::PlusX.to_bevy_point(&p), [2.0_f32, 1.0, 3.0]);
    }

    /// Normals follow the same swap matrix as points.
    #[test]
    fn to_bevy_normal_matches_point_swap() {
        let n = Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(UpAxis::PlusZ.to_bevy_normal(&n), [1.0_f32, 3.0, 2.0]);
        assert_eq!(UpAxis::PlusY.to_bevy_normal(&n), [1.0_f32, 2.0, 3.0]);
        assert_eq!(UpAxis::PlusX.to_bevy_normal(&n), [2.0_f32, 1.0, 3.0]);
    }
}
