//! Live texture preview — a coarse, representative textured-proxy mesh for a
//! frontend's "tune the ridges" UI (Cendrillon's Texture step).
//!
//! This is **not** a cast artifact: it textures a simple capsule proxy with
//! the *same* canal field the cast applies (so the ridge pattern matches what
//! a real cast produces), but at a coarse cell size so it re-meshes in
//! milliseconds — fast enough to update live as the user drags a slider. The
//! real part is cast at full detail by the normal pipeline.

use cf_design::{Aabb, Solid};
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_types::IndexedMesh;
use nalgebra::Point3;

use crate::canal::{CanalSpec, build_canal_plug};

/// MC grid padding (matches the production mesher's `GRID_PADDING_CELLS`).
const PREVIEW_GRID_PADDING_CELLS: usize = 2;

/// Mesh a Z-aligned capsule (radius / half-height in meters) textured with
/// `spec`'s canal field, at `cell_size_m` marching-cubes resolution.
///
/// Reuses [`build_canal_plug`] — the exact field the cast composes — so the
/// rings / texture / pinch / relief / orientation render faithfully. Pass a
/// coarse `cell_size_m` (≈1.5–2 mm) for an interactive, near-instant preview;
/// a feature-free `spec` (no rings, zero amplitudes) yields the smooth proxy.
#[must_use]
pub fn preview_textured_capsule(
    spec: &CanalSpec,
    radius_m: f64,
    half_height_m: f64,
    cell_size_m: f64,
) -> IndexedMesh {
    let body = Solid::capsule(radius_m, half_height_m);
    let centerline = vec![
        Point3::new(0.0, 0.0, -half_height_m),
        Point3::new(0.0, 0.0, half_height_m),
    ];
    let textured = build_canal_plug(&body, &centerline, None, spec);

    let bounds = textured.bounds().unwrap_or_else(|| {
        let r = radius_m + spec.suction_bulge_m + 0.002;
        let h = half_height_m + radius_m + 0.002;
        Aabb::new(Point3::new(-r, -r, -h), Point3::new(r, r, h))
    });
    let mut grid = ScalarGrid::from_bounds(
        bounds.min,
        bounds.max,
        cell_size_m,
        PREVIEW_GRID_PADDING_CELLS,
    );
    let (nx, ny, nz) = grid.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                grid.set(ix, iy, iz, textured.evaluate(&p));
            }
        }
    }
    marching_cubes(&grid, &MarchingCubesConfig::default())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn smooth_spec_meshes_a_clean_capsule() {
        // A feature-free spec → a plain capsule (no radial modulation).
        let mut spec = CanalSpec::iter1();
        spec.rings.clear();
        spec.texture_amp_m = 0.0;
        spec.dsection_depth_m = 0.0;
        spec.suction_bulge_m = 0.0;
        let mesh = preview_textured_capsule(&spec, 0.010, 0.040, 0.002);
        assert!(!mesh.vertices.is_empty(), "the capsule meshes");
        // Radius is ~10 mm everywhere in the cylindrical mid-section.
        let mid: Vec<f64> = mesh
            .vertices
            .iter()
            .filter(|v| v.z.abs() < 0.030)
            .map(|v| v.x.hypot(v.y))
            .collect();
        let max_r = mid.iter().copied().fold(0.0_f64, f64::max);
        assert!((max_r - 0.010).abs() < 0.001, "smooth ~10 mm, got {max_r}");
    }

    #[test]
    fn rings_modulate_the_preview_surface() {
        // The default iter1 rings should leave a visible radial dip.
        let spec = CanalSpec::iter1();
        let mesh = preview_textured_capsule(&spec, 0.010, 0.040, 0.0015);
        let radii: Vec<f64> = mesh
            .vertices
            .iter()
            .filter(|v| v.z.abs() < 0.030)
            .map(|v| v.x.hypot(v.y))
            .collect();
        let max_r = radii.iter().copied().fold(0.0_f64, f64::max);
        let min_r = radii.iter().copied().fold(f64::MAX, f64::min);
        assert!(
            max_r - min_r > 0.001,
            "rings should modulate the surface ({min_r}..{max_r})"
        );
    }
}
