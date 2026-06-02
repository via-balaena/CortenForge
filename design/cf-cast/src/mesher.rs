//! Internal SDF → [`ScalarGrid`] → marching cubes → mm-scaled mesh pipeline.

use cf_design::Solid;
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_types::IndexedMesh;
use nalgebra::Point3;

use crate::error::{CastError, CastTarget};

/// Far-cell sentinel magnitude (m) for the narrow-band fill. Surface-free grid
/// points get `±FAR_SENTINEL_M` carrying their region's inside/outside sign;
/// marching cubes reads only the SIGN of far cells (all-same-sign → no
/// triangle), so the magnitude is immaterial as long as it dwarfs any cf-cast
/// feature. 1 m is far beyond any mold dimension.
const FAR_SENTINEL_M: f64 = 1.0;

/// Coarse-block edge (in cells) for the narrow-band fill. The coarse pass spends
/// one SDF eval per block to classify it surface-free vs near-surface; 8 keeps
/// that pass cheap while the refined near-surface band dominates the work.
const NARROW_BAND_BLOCK_CELLS: usize = 8;

/// Meters → millimeters scale factor at the cf-design → printer
/// boundary. cf-design works in meters; STL output and
/// [`mesh_printability::PrinterConfig::fdm_default`]'s build-volume
/// check work in mm.
pub const METERS_TO_MM: f64 = 1000.0;

/// Cells of padding on each side of the input AABB. Two cells gives
/// marching cubes a clean view of the surface (one cell on each side
/// of the iso-crossing) without inflating grid size meaningfully.
const GRID_PADDING_CELLS: usize = 2;

/// Sample `solid`'s SDF onto a [`ScalarGrid`] at `cell_size_m`
/// resolution, run marching cubes, and rescale vertices from meters to
/// millimeters in-place. Returns the resulting [`IndexedMesh`] in mm
/// coordinates.
///
/// `target` labels the operation for error reporting — per-layer
/// mold output uses [`CastTarget::Mold`], the shared plug uses
/// [`CastTarget::Plug`].
pub fn solid_to_mm_mesh(
    solid: &Solid,
    cell_size_m: f64,
    target: CastTarget,
) -> Result<IndexedMesh, CastError> {
    let bounds = solid.bounds().ok_or(CastError::InfiniteBounds(target))?;

    let mut grid = ScalarGrid::from_bounds(bounds.min, bounds.max, cell_size_m, GRID_PADDING_CELLS);

    // Narrow-band fill: evaluate only the near-surface band, sentinel the rest.
    // Bit-identical to the full bake (see docs/CF_CAST_NARROW_BAND_MC_RECON.md +
    // the `narrow_band_fill_is_bit_identical_to_full` test) — the speedup is in
    // skipping the ~99% of cells far from the surface. Setting `CF_CAST_FULL_GRID`
    // forces the dense bake (escape hatch / A-B oracle).
    if std::env::var_os("CF_CAST_FULL_GRID").is_some() {
        fill_grid_full(&mut grid, solid);
    } else {
        fill_grid_narrow_band(&mut grid, solid);
    }

    let mut mesh = marching_cubes(&grid, &MarchingCubesConfig::default());

    if mesh.vertices.is_empty() || mesh.faces.is_empty() {
        return Err(CastError::MeshingEmpty(target));
    }

    scale_in_place(&mut mesh, METERS_TO_MM);
    Ok(mesh)
}

/// Multiply every vertex coordinate of `mesh` by `factor` in place.
pub fn scale_in_place(mesh: &mut IndexedMesh, factor: f64) {
    for v in &mut mesh.vertices {
        v.x *= factor;
        v.y *= factor;
        v.z *= factor;
    }
}

/// Fill every grid point with the exact SDF (the dense bake). Retained as the
/// reference path for the narrow-band byte-identity test.
// Grid-index → f64 casts are exact + non-negative (bounded by the grid
// dimensions); no precision/sign loss.
#[allow(clippy::cast_precision_loss)]
fn fill_grid_full(grid: &mut ScalarGrid, solid: &Solid) {
    let (nx, ny, nz) = grid.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                grid.set(ix, iy, iz, solid.evaluate(&p));
            }
        }
    }
}

/// Fill `grid` from `solid`'s SDF evaluating ONLY the near-surface band — the
/// ~1-Lipschitz "narrow-band" skip (see `docs/CF_CAST_NARROW_BAND_MC_RECON.md`).
///
/// Coarse-to-fine: each `K³`-cell block is classified by one SDF eval at its
/// centre — if `|SDF(centre)| > circumradius` the block is provably surface-free
/// (1-Lipschitz) and gets a sign-correct sentinel; otherwise it (plus a 1-point
/// halo) is evaluated exactly. The result is bit-identical to [`fill_grid_full`]
/// followed by marching cubes: skipped regions are all-same-sign (no triangles),
/// and every surface-crossing cell's 8 corners are exact (the halo guarantees
/// this even across a skip/refine boundary). The win is skipping the ~99% of
/// cells far from the (thin, 2-D) surface.
// Grid-index → f64 casts are exact + non-negative; the circumradius carries a
// conservative +2-cell margin (robust to a field marginally over 1-Lipschitz;
// the byte-identity test is the exactness gate).
#[allow(clippy::cast_precision_loss)]
fn fill_grid_narrow_band(grid: &mut ScalarGrid, solid: &Solid) {
    let (nx, ny, nz) = grid.dimensions();
    let cell = grid.cell_size();
    let origin = grid.origin();
    let k = NARROW_BAND_BLOCK_CELLS;
    // Point position without borrowing `grid` (so we can `set` freely).
    let pos = |ix: usize, iy: usize, iz: usize| {
        Point3::new(
            origin.x + ix as f64 * cell,
            origin.y + iy as f64 * cell,
            origin.z + iz as f64 * cell,
        )
    };
    // Block centre + circumradius (centre → farthest corner) + a 2-cell margin
    // (covers the cell that spans into the next block, plus Lipschitz safety).
    let block_classify = |bx: usize, by: usize, bz: usize| {
        let ex = (bx + k).min(nx);
        let ey = (by + k).min(ny);
        let ez = (bz + k).min(nz);
        let lo = pos(bx, by, bz);
        let hi = pos(ex - 1, ey - 1, ez - 1);
        let centre = Point3::new(
            f64::midpoint(lo.x, hi.x),
            f64::midpoint(lo.y, hi.y),
            f64::midpoint(lo.z, hi.z),
        );
        let circumradius = (hi - centre).norm() + 2.0 * cell;
        let sdf_c = solid.evaluate(&centre);
        (ex, ey, ez, sdf_c, circumradius)
    };

    // Pass 1 — sentinel-fill every surface-free (skip) block.
    let mut bx = 0;
    while bx < nx {
        let mut by = 0;
        while by < ny {
            let mut bz = 0;
            while bz < nz {
                let (ex, ey, ez, sdf_c, r) = block_classify(bx, by, bz);
                if sdf_c.abs() > r {
                    let sentinel = if sdf_c < 0.0 {
                        -FAR_SENTINEL_M
                    } else {
                        FAR_SENTINEL_M
                    };
                    for iz in bz..ez {
                        for iy in by..ey {
                            for ix in bx..ex {
                                grid.set(ix, iy, iz, sentinel);
                            }
                        }
                    }
                }
                bz += k;
            }
            by += k;
        }
        bx += k;
    }

    // Pass 2 — exact-eval every near-surface (refine) block PLUS a 1-point halo,
    // overwriting sentinels at shared boundaries (so a surface cell straddling a
    // skip/refine seam has all 8 corners exact). Runs after pass 1 so the halo's
    // exact values win.
    let mut bx = 0;
    while bx < nx {
        let mut by = 0;
        while by < ny {
            let mut bz = 0;
            while bz < nz {
                let (ex, ey, ez, sdf_c, r) = block_classify(bx, by, bz);
                if sdf_c.abs() <= r {
                    let hx0 = bx.saturating_sub(1);
                    let hy0 = by.saturating_sub(1);
                    let hz0 = bz.saturating_sub(1);
                    let hx1 = (ex + 1).min(nx);
                    let hy1 = (ey + 1).min(ny);
                    let hz1 = (ez + 1).min(nz);
                    for iz in hz0..hz1 {
                        for iy in hy0..hy1 {
                            for ix in hx0..hx1 {
                                grid.set(ix, iy, iz, solid.evaluate(&pos(ix, iy, iz)));
                            }
                        }
                    }
                }
                bz += k;
            }
            by += k;
        }
        bx += k;
    }
}

#[cfg(test)]
mod tests {
    // Workspace lint policy denies `unwrap()` in lib code but warns
    // in tests. Localized allow makes assertion failures read clearly
    // without panic-handling boilerplate at every probe. Same
    // convention as cf-design tests.
    #![allow(clippy::unwrap_used)]

    use approx::assert_relative_eq;
    use mesh_types::{IndexedMesh, Point3};

    use super::{METERS_TO_MM, scale_in_place, solid_to_mm_mesh};
    use crate::error::CastTarget;
    use cf_design::Solid;
    use nalgebra::Vector3;

    #[test]
    fn meters_to_mm_is_exactly_1000() {
        // The whole unit boundary collapses to one constant; pinning
        // it as a bit-exact integer eliminates any chance of a future
        // refactor accidentally shifting the scale.
        assert!((METERS_TO_MM - 1000.0).abs() < f64::EPSILON);
    }

    #[test]
    fn scale_in_place_multiplies_every_vertex() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(1.0, 2.0, 3.0));
        mesh.vertices.push(Point3::new(-0.5, 0.0, 4.25));
        mesh.faces.push([0, 1, 0]); // dummy face so the mesh is not "empty"

        scale_in_place(&mut mesh, METERS_TO_MM);

        assert_relative_eq!(mesh.vertices[0].x, 1000.0);
        assert_relative_eq!(mesh.vertices[0].y, 2000.0);
        assert_relative_eq!(mesh.vertices[0].z, 3000.0);
        assert_relative_eq!(mesh.vertices[1].x, -500.0);
        assert_relative_eq!(mesh.vertices[1].y, 0.0);
        assert_relative_eq!(mesh.vertices[1].z, 4250.0);
    }

    #[test]
    fn solid_to_mm_mesh_emits_non_empty_for_sphere() {
        // 10 mm radius sphere in meters; 1 mm cell size → MC should
        // produce a closed mesh well inside the f64 range and well
        // above the empty-mesh threshold.
        let sphere = Solid::sphere(0.010);
        let mesh = solid_to_mm_mesh(&sphere, 0.001, CastTarget::Mold { layer_index: 0 }).unwrap();

        assert!(!mesh.vertices.is_empty(), "MC should produce vertices");
        assert!(!mesh.faces.is_empty(), "MC should produce faces");

        // Sanity: scaled to mm, vertices should lie within ±15 mm of
        // origin (10 mm radius + 2-cell padding × 1 mm cells = 12 mm
        // grid half-extent, well under 15).
        for v in &mesh.vertices {
            assert!(
                v.x.abs() <= 15.0 && v.y.abs() <= 15.0 && v.z.abs() <= 15.0,
                "vertex {v:?} outside expected mm range after scale"
            );
        }
    }

    #[test]
    fn solid_to_mm_mesh_meshes_translated_cuboid() {
        // Pins that `Solid::bounds()` flows correctly when the input
        // is translated off the origin. The sphere test above covers
        // origin-centred geometry; this one exercises a non-zero
        // origin so the grid sampling and MC don't latently assume
        // axis-aligned at zero.
        let cuboid = Solid::cuboid(Vector3::new(0.005, 0.005, 0.005))
            .translate(Vector3::new(0.020, 0.0, 0.0));
        let mesh =
            solid_to_mm_mesh(&cuboid, 0.001, CastTarget::Plug { layer_index: None }).unwrap();
        assert!(!mesh.vertices.is_empty());
        // Translated by +20 mm in x; vertices should lie roughly at
        // x ∈ [15, 25] mm (5 mm half-extent + cell padding).
        let x_min = mesh
            .vertices
            .iter()
            .map(|v| v.x)
            .fold(f64::INFINITY, f64::min);
        let x_max = mesh
            .vertices
            .iter()
            .map(|v| v.x)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (12.0..28.0).contains(&x_min) && (12.0..28.0).contains(&x_max),
            "translated cuboid x-range should be near 20 mm: [{x_min}, {x_max}]"
        );
    }

    /// THE narrow-band gate: marching cubes over the sparse (narrow-band) grid
    /// must be **bit-identical** to marching cubes over the dense (full) grid —
    /// across several solids that exercise multiple coarse blocks + skip/refine
    /// boundaries (sphere, off-origin cuboid, and a CSG union). If this ever
    /// diverges (a field marginally over 1-Lipschitz), the +2-cell circumradius
    /// margin needs widening. Same vertices (bit-exact: identical points are
    /// evaluated identically) and same faces.
    #[test]
    fn narrow_band_fill_is_bit_identical_to_full() {
        use super::{fill_grid_full, fill_grid_narrow_band};
        use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};

        let solids: Vec<Solid> = vec![
            Solid::sphere(0.012),
            Solid::cuboid(Vector3::new(0.006, 0.006, 0.006))
                .translate(Vector3::new(0.020, 0.0, 0.0)),
            Solid::sphere(0.012).union(
                Solid::cuboid(Vector3::new(0.005, 0.005, 0.013))
                    .translate(Vector3::new(0.009, 0.0, 0.0)),
            ),
        ];
        let cell = 0.0015; // enough cells to span several K=8 blocks + their seams
        for (i, solid) in solids.iter().enumerate() {
            let bounds = solid.bounds().unwrap();
            let mut g_full = ScalarGrid::from_bounds(bounds.min, bounds.max, cell, 2);
            let mut g_nb = ScalarGrid::from_bounds(bounds.min, bounds.max, cell, 2);
            fill_grid_full(&mut g_full, solid);
            fill_grid_narrow_band(&mut g_nb, solid);
            let m_full = marching_cubes(&g_full, &MarchingCubesConfig::default());
            let m_nb = marching_cubes(&g_nb, &MarchingCubesConfig::default());

            assert!(
                !m_full.vertices.is_empty(),
                "solid {i}: full grid should mesh"
            );
            assert_eq!(
                m_nb.vertices.len(),
                m_full.vertices.len(),
                "solid {i}: narrow-band vertex count must match full",
            );
            assert_eq!(m_nb.faces, m_full.faces, "solid {i}: faces must match full");
            for (a, b) in m_nb.vertices.iter().zip(&m_full.vertices) {
                assert!(
                    (a.x - b.x).abs() < 1e-12
                        && (a.y - b.y).abs() < 1e-12
                        && (a.z - b.z).abs() < 1e-12,
                    "solid {i}: vertex mismatch narrow={a:?} full={b:?}",
                );
            }
        }
    }
}
