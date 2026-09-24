// The obstacle's signed distance, looked up in a baked grid with the clamped
// semantics the product's CPU path uses (`cf_geometry::SdfGrid`'s
// `distance_clamped` and `gradient_clamped`, `design/cf-geometry/src/sdf.rs`):
//
// - a point outside the grid is clamped onto it, so it reads the nearest
//   face's value;
// - the distance is the trilinear interpolation at the clamped point;
// - the normal is the centered difference at ±½ cell along each axis, each
//   probe clamped again, normalized, or +z where it is degenerate.
//
// That is seven trilinear lookups, the "probes". The shared math owns every
// index, clamp, weight and the combination; the executor only fetches:
//
//   for probe in 0..SDF_PROBE_COUNT:
//       c = sdf_probe_coordinate(point, grid, probe)
//       values[probe] = sdf_trilinear(c, <the grid values at sdf_cell_corners(c, grid)>)
//   sample = sdf_combine(values, grid)
//
// Points are in the obstacle's body frame and grid values are distances in
// the same units; negative is inside the obstacle.

/// A baked distance grid's layout. Values are stored with x fastest, then y,
/// then z: the value at sample `(i, j, k)` is at `(k · size_y + j) · size_x + i`
/// (`cf_geometry::SdfGrid`'s order).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SdfGridLayout {
    /// The grid's minimum corner, x.
    pub origin_x: R,
    /// The grid's minimum corner, y.
    pub origin_y: R,
    /// The grid's minimum corner, z.
    pub origin_z: R,
    /// The spacing between samples, the same along every axis.
    pub cell_size: R,
    /// Samples along x (at least 1).
    pub size_x: u32,
    /// Samples along y (at least 1).
    pub size_y: u32,
    /// Samples along z (at least 1).
    pub size_z: u32,
}

/// The distance and outward normal at a point.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SdfSample {
    /// Signed distance; negative inside the obstacle.
    pub distance: R,
    /// Unit outward normal, the normalized distance gradient; +z where the
    /// gradient is degenerate.
    pub normal: [R; 3],
}

/// The number of trilinear lookups per sample: the point, then ±½ cell
/// along x, y and z.
pub const SDF_PROBE_COUNT: u32 = 7;

/// A gradient shorter than this is degenerate and the normal falls back to
/// +z. The CPU path's threshold (`sdf.rs:524`), adopted at both precisions;
/// the GPU shader it replaces used 1e-5 (plan §14c).
pub const SDF_DEGENERATE_GRADIENT: R = 1e-10;

/// A point's grid coordinates, `(p − origin) / cell`, clamped onto the grid.
#[allow(clippy::cast_precision_loss, clippy::cast_lossless)]
#[must_use]
pub fn sdf_grid_coordinate(point: [R; 3], grid: SdfGridLayout) -> [R; 3] {
    [
        ((point[0] - grid.origin_x) / grid.cell_size).clamp(0.0, (grid.size_x - 1) as R),
        ((point[1] - grid.origin_y) / grid.cell_size).clamp(0.0, (grid.size_y - 1) as R),
        ((point[2] - grid.origin_z) / grid.cell_size).clamp(0.0, (grid.size_z - 1) as R),
    ]
}

/// The grid coordinates of one probe: probe 0 is the point itself, clamped;
/// probes 1–6 step ½ cell from it along +x, −x, +y, −y, +z, −z, and are
/// clamped again.
#[allow(clippy::cast_precision_loss, clippy::cast_lossless)]
#[must_use]
pub fn sdf_probe_coordinate(point: [R; 3], grid: SdfGridLayout, probe: u32) -> [R; 3] {
    let center = sdf_grid_coordinate(point, grid);
    let step_x = if probe == 1 {
        0.5
    } else if probe == 2 {
        -0.5
    } else {
        0.0
    };
    let step_y = if probe == 3 {
        0.5
    } else if probe == 4 {
        -0.5
    } else {
        0.0
    };
    let step_z = if probe == 5 {
        0.5
    } else if probe == 6 {
        -0.5
    } else {
        0.0
    };
    [
        (center[0] + step_x).clamp(0.0, (grid.size_x - 1) as R),
        (center[1] + step_y).clamp(0.0, (grid.size_y - 1) as R),
        (center[2] + step_z).clamp(0.0, (grid.size_z - 1) as R),
    ]
}

/// The storage indices of the eight grid values around a clamped grid
/// coordinate, in the order 000, 100, 010, 110, 001, 101, 011, 111 (x
/// fastest). On the far face the upper corner repeats the lower one.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
#[must_use]
pub fn sdf_cell_corners(coordinate: [R; 3], grid: SdfGridLayout) -> [u32; 8] {
    let x0 = coordinate[0].floor() as u32;
    let y0 = coordinate[1].floor() as u32;
    let z0 = coordinate[2].floor() as u32;
    let x1 = (x0 + 1).min(grid.size_x - 1);
    let y1 = (y0 + 1).min(grid.size_y - 1);
    let z1 = (z0 + 1).min(grid.size_z - 1);
    let plane = grid.size_x * grid.size_y;
    let row0 = y0 * grid.size_x;
    let row1 = y1 * grid.size_x;
    let slab0 = z0 * plane;
    let slab1 = z1 * plane;
    [
        slab0 + row0 + x0,
        slab0 + row0 + x1,
        slab0 + row1 + x0,
        slab0 + row1 + x1,
        slab1 + row0 + x0,
        slab1 + row0 + x1,
        slab1 + row1 + x0,
        slab1 + row1 + x1,
    ]
}

/// Trilinear interpolation of the eight `corners` (in `sdf_cell_corners`'
/// order) at a clamped grid coordinate, interpolating along x, then y, then
/// z, as the CPU path does.
#[must_use]
pub fn sdf_trilinear(coordinate: [R; 3], corners: [R; 8]) -> R {
    let fx = coordinate[0] - coordinate[0].floor();
    let fy = coordinate[1] - coordinate[1].floor();
    let fz = coordinate[2] - coordinate[2].floor();
    let v00 = corners[0] + fx * (corners[1] - corners[0]);
    let v10 = corners[2] + fx * (corners[3] - corners[2]);
    let v01 = corners[4] + fx * (corners[5] - corners[4]);
    let v11 = corners[6] + fx * (corners[7] - corners[6]);
    let v0 = v00 + fy * (v10 - v00);
    let v1 = v01 + fy * (v11 - v01);
    v0 + fz * (v1 - v0)
}

/// The distance and normal from the seven probes' values (in probe order).
#[must_use]
pub fn sdf_combine(values: [R; 7], grid: SdfGridLayout) -> SdfSample {
    let two_eps = 2.0 * (grid.cell_size * 0.5);
    let gradient = [
        (values[1] - values[2]) / two_eps,
        (values[3] - values[4]) / two_eps,
        (values[5] - values[6]) / two_eps,
    ];
    let norm = vec3_length(gradient);
    let degenerate = norm <= SDF_DEGENERATE_GRADIENT;
    let guarded_norm = if degenerate { 1.0 } else { norm };
    SdfSample {
        distance: values[0],
        normal: vec3_select(
            degenerate,
            [0.0, 0.0, 1.0],
            [
                gradient[0] / guarded_norm,
                gradient[1] / guarded_norm,
                gradient[2] / guarded_norm,
            ],
        ),
    }
}
