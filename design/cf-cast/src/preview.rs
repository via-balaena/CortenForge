//! Live texture preview — a coarse, representative textured-proxy mesh for a
//! frontend's "tune the ridges" UI (Cendrillon's Texture steps).
//!
//! Not a cast artifact. It textures a simple proxy (a flat-floor plug or a
//! rounded shell) with the *same* canal field the cast applies — so the ridge
//! pattern matches a real cast — but at a coarse cell size, so it re-meshes in
//! milliseconds: fast enough to update live as the user drags a slider. The
//! real part is cast at full detail by the normal pipeline.

use cf_design::{Aabb, Solid};
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::canal::{CanalSpec, build_canal_plug};

/// MC grid padding (matches the production mesher's `GRID_PADDING_CELLS`).
const PREVIEW_GRID_PADDING_CELLS: usize = 2;

/// Mesh a Z-aligned **plug proxy** (radius / half-height in meters) textured
/// with `spec`'s canal field, at `cell_size_m` marching-cubes resolution.
///
/// The proxy is a cylinder with a **flat bottom** (z = −`half_height_m`, the
/// mouth / cap-plane floor, where the canal frame's `frac = 0` sits) and a
/// **domed top** (a hemisphere, the deep end) — resembling the real plug far
/// better than a both-ends-rounded capsule.
///
/// Reuses [`build_canal_plug`] — the exact field the cast composes — so the
/// rings / texture / pinch / relief / orientation render faithfully. Pass a
/// coarse `cell_size_m` (≈1.5–2 mm) for an interactive, near-instant preview;
/// a feature-free `spec` (no rings, zero amplitudes) yields the smooth proxy.
#[must_use]
pub fn preview_textured_plug(
    spec: &CanalSpec,
    radius_m: f64,
    half_height_m: f64,
    cell_size_m: f64,
) -> IndexedMesh {
    // Flat-bottomed cylinder + a top hemisphere dome = a plug-shaped proxy
    // (the interior ridges live inside this).
    let body = Solid::cylinder(radius_m, half_height_m)
        .union(Solid::sphere(radius_m).translate(Vector3::new(0.0, 0.0, half_height_m)));
    mesh_textured_proxy(&body, spec, half_height_m, cell_size_m)
}

/// Mesh a Z-aligned **shell proxy** — a both-ends-rounded capsule (the
/// device's outer body) textured with `spec`'s field.
///
/// The exterior shell ridges live on this surface; the rounded shape
/// distinguishes it from the flat-floor plug. Otherwise identical to
/// [`preview_textured_plug`].
#[must_use]
pub fn preview_textured_capsule(
    spec: &CanalSpec,
    radius_m: f64,
    half_height_m: f64,
    cell_size_m: f64,
) -> IndexedMesh {
    let body = Solid::capsule(radius_m, half_height_m);
    mesh_textured_proxy(&body, spec, half_height_m, cell_size_m)
}

/// Shared: compose the canal field onto `body` (mouth = the −Z end) and mesh it
/// at `cell_size_m`.
fn mesh_textured_proxy(
    body: &Solid,
    spec: &CanalSpec,
    half_height_m: f64,
    cell_size_m: f64,
) -> IndexedMesh {
    let centerline = vec![
        Point3::new(0.0, 0.0, -half_height_m), // −Z end = mouth (frac 0)
        Point3::new(0.0, 0.0, half_height_m),
    ];
    let textured = build_canal_plug(body, &centerline, None, spec);

    let bounds = textured.bounds().unwrap_or_else(|| {
        let r = 0.05 + spec.suction_bulge_m;
        Aabb::new(Point3::new(-r, -r, -r), Point3::new(r, r, r))
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
    fn smooth_spec_meshes_a_clean_plug() {
        // A feature-free spec → a plain plug proxy (no radial modulation).
        let mut spec = CanalSpec::iter1();
        spec.rings.clear();
        spec.texture_amp_m = 0.0;
        spec.dsection_depth_m = 0.0;
        spec.suction_bulge_m = 0.0;
        let mesh = preview_textured_plug(&spec, 0.010, 0.040, 0.002);
        assert!(!mesh.vertices.is_empty(), "the proxy meshes");
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
        let mesh = preview_textured_plug(&spec, 0.010, 0.040, 0.0015);
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
