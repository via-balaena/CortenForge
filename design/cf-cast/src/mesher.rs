//! Internal SDF → [`ScalarGrid`] → marching cubes → mm-scaled mesh pipeline.

use cf_design::Solid;
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_types::IndexedMesh;

use crate::error::{CastError, CastTarget};

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
    let (nx, ny, nz) = grid.dimensions();

    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                grid.set(ix, iy, iz, solid.evaluate(&p));
            }
        }
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
}
