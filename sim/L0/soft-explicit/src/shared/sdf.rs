// The obstacle's signed distance, looked up in a baked grid: a tricubic
// (Catmull–Rom) interpolant of the grid's values, and its exact gradient as
// the normal (plan §16o).
//
// - a point outside the grid is clamped onto it, so it reads the nearest
//   face;
// - the distance is the tricubic interpolation of the 4 × 4 × 4 samples
//   around the point, a sample past a face extrapolated linearly from the
//   two inside it;
// - the normal is that interpolant's gradient, normalized, or +z where it is
//   degenerate.
//
// A trilinear interpolant of the 11 mm mandrel at A/20 is off by up to 5.7
// µm (its nose; about 3 µm on its shank), and bumps that large fed energy
// into the kinematic contact in a long frictionless hold (plan §16o). This
// one is off by 0.053 µm away from the seam where the nose meets the shank,
// and 0.9 µm across it, where the true surface's curvature jumps
// (`tests/sdf_lookup.rs`). Past a face the grid is extended linearly, so a
// plane is exact to every face. The shared math owns every index, clamp,
// weight and the combination; the executor only fetches:
//
//   c = sdf_grid_coordinate(point, grid)
//   ix, iy, iz = sdf_tricubic_axis(c[0], size_x), (c[1], size_y), (c[2], size_z)
//   values[(k·4 + j)·4 + i] = grid[sdf_grid_index(ix[i], iy[j], iz[k], grid)]
//   sample = sdf_tricubic(c, values, grid)
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
    /// The spacing between samples, the same along every axis; positive.
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

/// The number of grid values one lookup reads: 4 × 4 × 4.
pub const SDF_TRICUBIC_COUNT: u32 = 64;

/// A gradient shorter than this is degenerate and the normal falls back to
/// +z (`cf-geometry`'s threshold, `sdf.rs:524`, adopted at both precisions).
pub const SDF_DEGENERATE_GRADIENT: R = 1e-10;

/// A point's grid coordinates, `(p − origin) / cell`, clamped onto the grid.
// `u32 as R`: exact for any grid under 2^24 samples a side at f32, and
// lossless at f64, where clippy prefers `R::from`, which the subset lacks.
#[allow(clippy::cast_precision_loss, clippy::cast_lossless)]
#[must_use]
pub fn sdf_grid_coordinate(point: [R; 3], grid: SdfGridLayout) -> [R; 3] {
    [
        ((point[0] - grid.origin_x) / grid.cell_size).clamp(0.0, (grid.size_x - 1) as R),
        ((point[1] - grid.origin_y) / grid.cell_size).clamp(0.0, (grid.size_y - 1) as R),
        ((point[2] - grid.origin_z) / grid.cell_size).clamp(0.0, (grid.size_z - 1) as R),
    ]
}

/// The first sample of the cell a lookup interpolates in, along one axis.
///
/// `coordinate` is the clamped grid coordinate ([`sdf_grid_coordinate`]) on
/// an axis of `size` samples. The cell starts at its floor, kept one short of
/// the last sample, so a point on the far face is at the end of the last
/// cell.
// `R as u32` on a coordinate clamped to [0, size − 1]: its floor is a
// non-negative whole number that fits.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
#[must_use]
pub fn sdf_tricubic_base(coordinate: R, size: u32) -> u32 {
    (coordinate.floor() as u32).min(size.max(2) - 2)
}

/// The four sample indices along one axis that a lookup reads.
///
/// They are the cell's two and one either side, clamped onto the `size`
/// samples. A clamped one is not used: [`sdf_tricubic_ends`] extrapolates the
/// grid past the face instead.
#[must_use]
pub fn sdf_tricubic_axis(coordinate: R, size: u32) -> [u32; 4] {
    let base = sdf_tricubic_base(coordinate, size);
    let last = size - 1;
    let below = if base > 0 { base - 1 } else { 0 };
    [below, base, (base + 1).min(last), (base + 2).min(last)]
}

/// The storage index of sample `(column, row, layer)`: x fastest, then y,
/// then z.
#[must_use]
pub const fn sdf_grid_index(column: u32, row: u32, layer: u32, grid: SdfGridLayout) -> u32 {
    (layer * grid.size_y + row) * grid.size_x + column
}

/// Catmull–Rom weights of the four samples at fraction `t` of the cell.
#[must_use]
pub const fn sdf_tricubic_weights(t: R) -> [R; 4] {
    let t2 = t * t;
    let t3 = t2 * t;
    [
        0.5 * (-t3 + 2.0 * t2 - t),
        0.5 * (3.0 * t3 - 5.0 * t2 + 2.0),
        0.5 * (-3.0 * t3 + 4.0 * t2 + t),
        0.5 * (t3 - t2),
    ]
}

/// The weights' derivatives with respect to `t`: they give the
/// interpolant's slope, per cell.
#[must_use]
pub const fn sdf_tricubic_slopes(t: R) -> [R; 4] {
    let t2 = t * t;
    [
        0.5 * (-3.0 * t2 + 4.0 * t - 1.0),
        0.5 * (9.0 * t2 - 10.0 * t),
        0.5 * (-9.0 * t2 + 8.0 * t + 1.0),
        0.5 * (3.0 * t2 - 2.0 * t),
    ]
}

/// `weights` for a cell at a face of the grid.
///
/// Where the sample below the cell (`low`) or above it (`high`) lies past the
/// face, the grid is extended linearly from the cell's two samples instead
/// (`2 v₀ − v₁` below), which moves that sample's weight onto them. A plane is
/// then exact up to every face.
#[must_use]
pub fn sdf_tricubic_ends(weights: [R; 4], low: bool, high: bool) -> [R; 4] {
    let below: R = if low { 1.0 } else { 0.0 };
    let above: R = if high { 1.0 } else { 0.0 };
    [
        weights[0] * (1.0 - below),
        weights[1] + 2.0 * below * weights[0] - above * weights[3],
        weights[2] - below * weights[0] + 2.0 * above * weights[3],
        weights[3] * (1.0 - above),
    ]
}

/// The four samples along one axis, weighted: `Σ weights[i] · sample i`.
#[must_use]
pub const fn sdf_dot4(weights: [R; 4], first: R, second: R, third: R, fourth: R) -> R {
    weights[0] * first + weights[1] * second + weights[2] * third + weights[3] * fourth
}

/// The distance and normal at grid coordinate `coordinate`, from the 64
/// grid values [`sdf_tricubic_axis`] names, stored `(k · 4 + j) · 4 + i`.
///
/// Catmull–Rom in each axis: it passes through the grid's values; it
/// reproduces a field quadratic in each axis exactly where its four samples
/// lie inside the grid, and a plane exactly everywhere (the grid is extended
/// linearly past its faces, [`sdf_tricubic_ends`]). The normal is the
/// interpolant's own gradient, so the distance and the normal describe one
/// surface.
// `u32 as R`: exact for any grid under 2^24 samples a side at f32.
#[allow(clippy::cast_precision_loss, clippy::cast_lossless)]
#[must_use]
pub fn sdf_tricubic(coordinate: [R; 3], values: [R; 64], grid: SdfGridLayout) -> SdfSample {
    let bx = sdf_tricubic_base(coordinate[0], grid.size_x);
    let by = sdf_tricubic_base(coordinate[1], grid.size_y);
    let bz = sdf_tricubic_base(coordinate[2], grid.size_z);
    let tx = coordinate[0] - bx as R;
    let ty = coordinate[1] - by as R;
    let tz = coordinate[2] - bz as R;
    let lx = bx == 0;
    let ly = by == 0;
    let lz = bz == 0;
    let hx = bx + 2 >= grid.size_x;
    let hy = by + 2 >= grid.size_y;
    let hz = bz + 2 >= grid.size_z;
    let wx = sdf_tricubic_ends(sdf_tricubic_weights(tx), lx, hx);
    let sx = sdf_tricubic_ends(sdf_tricubic_slopes(tx), lx, hx);
    let wy = sdf_tricubic_ends(sdf_tricubic_weights(ty), ly, hy);
    let sy = sdf_tricubic_ends(sdf_tricubic_slopes(ty), ly, hy);
    let wz = sdf_tricubic_ends(sdf_tricubic_weights(tz), lz, hz);
    let sz = sdf_tricubic_ends(sdf_tricubic_slopes(tz), lz, hz);
    let v00 = sdf_dot4(wx, values[0], values[1], values[2], values[3]);
    let d00 = sdf_dot4(sx, values[0], values[1], values[2], values[3]);
    let v10 = sdf_dot4(wx, values[4], values[5], values[6], values[7]);
    let d10 = sdf_dot4(sx, values[4], values[5], values[6], values[7]);
    let v20 = sdf_dot4(wx, values[8], values[9], values[10], values[11]);
    let d20 = sdf_dot4(sx, values[8], values[9], values[10], values[11]);
    let v30 = sdf_dot4(wx, values[12], values[13], values[14], values[15]);
    let d30 = sdf_dot4(sx, values[12], values[13], values[14], values[15]);
    let v01 = sdf_dot4(wx, values[16], values[17], values[18], values[19]);
    let d01 = sdf_dot4(sx, values[16], values[17], values[18], values[19]);
    let v11 = sdf_dot4(wx, values[20], values[21], values[22], values[23]);
    let d11 = sdf_dot4(sx, values[20], values[21], values[22], values[23]);
    let v21 = sdf_dot4(wx, values[24], values[25], values[26], values[27]);
    let d21 = sdf_dot4(sx, values[24], values[25], values[26], values[27]);
    let v31 = sdf_dot4(wx, values[28], values[29], values[30], values[31]);
    let d31 = sdf_dot4(sx, values[28], values[29], values[30], values[31]);
    let v02 = sdf_dot4(wx, values[32], values[33], values[34], values[35]);
    let d02 = sdf_dot4(sx, values[32], values[33], values[34], values[35]);
    let v12 = sdf_dot4(wx, values[36], values[37], values[38], values[39]);
    let d12 = sdf_dot4(sx, values[36], values[37], values[38], values[39]);
    let v22 = sdf_dot4(wx, values[40], values[41], values[42], values[43]);
    let d22 = sdf_dot4(sx, values[40], values[41], values[42], values[43]);
    let v32 = sdf_dot4(wx, values[44], values[45], values[46], values[47]);
    let d32 = sdf_dot4(sx, values[44], values[45], values[46], values[47]);
    let v03 = sdf_dot4(wx, values[48], values[49], values[50], values[51]);
    let d03 = sdf_dot4(sx, values[48], values[49], values[50], values[51]);
    let v13 = sdf_dot4(wx, values[52], values[53], values[54], values[55]);
    let d13 = sdf_dot4(sx, values[52], values[53], values[54], values[55]);
    let v23 = sdf_dot4(wx, values[56], values[57], values[58], values[59]);
    let d23 = sdf_dot4(sx, values[56], values[57], values[58], values[59]);
    let v33 = sdf_dot4(wx, values[60], values[61], values[62], values[63]);
    let d33 = sdf_dot4(sx, values[60], values[61], values[62], values[63]);
    let v0 = sdf_dot4(wy, v00, v10, v20, v30);
    let gx0 = sdf_dot4(wy, d00, d10, d20, d30);
    let gy0 = sdf_dot4(sy, v00, v10, v20, v30);
    let v1 = sdf_dot4(wy, v01, v11, v21, v31);
    let gx1 = sdf_dot4(wy, d01, d11, d21, d31);
    let gy1 = sdf_dot4(sy, v01, v11, v21, v31);
    let v2 = sdf_dot4(wy, v02, v12, v22, v32);
    let gx2 = sdf_dot4(wy, d02, d12, d22, d32);
    let gy2 = sdf_dot4(sy, v02, v12, v22, v32);
    let v3 = sdf_dot4(wy, v03, v13, v23, v33);
    let gx3 = sdf_dot4(wy, d03, d13, d23, d33);
    let gy3 = sdf_dot4(sy, v03, v13, v23, v33);
    let distance = sdf_dot4(wz, v0, v1, v2, v3);
    let per_cell = 1.0 / grid.cell_size;
    let gradient = [
        sdf_dot4(wz, gx0, gx1, gx2, gx3) * per_cell,
        sdf_dot4(wz, gy0, gy1, gy2, gy3) * per_cell,
        sdf_dot4(sz, v0, v1, v2, v3) * per_cell,
    ];
    let norm = vec3_length(gradient);
    let degenerate = norm <= SDF_DEGENERATE_GRADIENT;
    let guarded_norm = if degenerate { 1.0 } else { norm };
    SdfSample {
        distance,
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
