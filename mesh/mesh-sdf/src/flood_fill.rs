//! Flood-fill sign oracle + cached signed-distance grid.
//!
//! Ports the 3-region (Inside / Outside / Wall) flood-fill that
//! cf-device-design's `sdf_layers::build_cached_scan_sdf` and
//! `insertion_sim::build_grid_sdf` each carry inline today (~150 LOC
//! per copy). Generic over any [`UnsignedDistance`] — typically a
//! [`TriMeshDistance`], but a [`CachedGridSdf`] also satisfies the
//! trait so this composes recursively if a caller ever wants it.
//!
//! Two primitives ship together because they share all of the build
//! machinery: [`FloodFillSign`] is a sign-only cache; [`CachedGridSdf`]
//! also caches the signed-distance grid for O(1) trilinear queries.
//!
//! See `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md` §3 and §5 for the
//! design choices (3-region wall band, 0.75 × cell_size threshold,
//! 6-connected BFS, multi-source label expansion) and why the
//! pre-existing pseudo-normal sign branch was unreliable enough to
//! motivate this defense.
//!
//! ⚠ **That motivation is real but one-sided, and reading it as
//! "flood-fill superseded pseudo-normal" is a mistake.** This oracle
//! trades a *topology* failure for a *spatial* one: its sign is a
//! lattice lookup and is unreliable within a cell of the surface,
//! where `PseudoNormalSign` is exact. See [`FloodFillSign`]'s
//! near-surface table. Pick by where you query.
//!
//! [`TriMeshDistance`]: crate::TriMeshDistance

use std::collections::VecDeque;
// `std::time::Instant::now()` panics at runtime on `wasm32-unknown-unknown`
// (`unimplemented!()` in stdlib). Keep the import gated so the WASM build
// doesn't drag unused-import warnings either; build_secs reports 0.0 on
// WASM via the cfg branches below.
#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;

use mesh_types::Aabb;
use nalgebra::{Point3, Vector3};

use crate::oracle::{FloodFillError, FloodFillReport, Region, Sign, Signed, UnsignedDistance};

/// Build-time defaults documented per spec §3.
///
/// `wall_threshold = WALL_THRESHOLD_FACTOR_DEFAULT × cell_size` is the
/// half-width of the band the flood treats as a barrier. 0.75 mirrors
/// the value cf-device-design ships in production (both sdf_layers
/// and insertion_sim land on it) and keeps the wall above the
/// 6-connectivity-watertight minimum of 0.5 × cell_size with a margin
/// against face-plane alignment artifacts without over-thickening the
/// band.
pub const WALL_THRESHOLD_FACTOR_DEFAULT: f64 = 0.75;

// ── Private grid storage ──────────────────────────────────────────────
//
// Layer 0 lives below mesh-offset (which owns the public `ScalarGrid`),
// so the storage is hand-rolled here rather than imported. The surface
// is intentionally minimal — `FloodFillSign` and `CachedGridSdf` are
// the only consumers and they each touch a small slice of it.

/// Dense `Vec<f64>` lattice. Lattice points sit at `origin + (ix, iy,
/// iz) × cell_size`; row-major flat layout with `x` varying fastest.
#[derive(Debug, Clone)]
struct SdfGrid {
    values: Vec<f64>,
    dims: [usize; 3],
    origin: Point3<f64>,
    cell_size: f64,
}

impl SdfGrid {
    fn filled(dims: [usize; 3], origin: Point3<f64>, cell_size: f64, values: Vec<f64>) -> Self {
        debug_assert_eq!(values.len(), dims[0] * dims[1] * dims[2]);
        Self {
            values,
            dims,
            origin,
            cell_size,
        }
    }

    #[inline]
    fn flat_index(dims: [usize; 3], ix: usize, iy: usize, iz: usize) -> usize {
        let [nx, ny, _] = dims;
        iz * nx * ny + iy * nx + ix
    }

    #[inline]
    fn get(&self, ix: usize, iy: usize, iz: usize) -> f64 {
        self.values[Self::flat_index(self.dims, ix, iy, iz)]
    }

    /// Trilinear interpolation at world point `p`, clamped to grid
    /// bounds. Out-of-bounds queries return the value at the nearest
    /// edge — the cached grids built by [`CachedGridSdf::build`]
    /// extend `cell_size` past the body in every direction, so the
    /// clamped sample is meaningful at the boundary.
    //
    // `nx - 1` etc. are non-negative grid extents; `clamp` keeps the
    // `as f64` precise + the `as usize` round-down within bounds, so
    // every cast here is gated by the explicit clamp/min above.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    fn sample_trilinear(&self, p: Point3<f64>) -> f64 {
        let [nx, ny, nz] = self.dims;
        if nx == 0 || ny == 0 || nz == 0 {
            return 0.0;
        }
        let fx = ((p.x - self.origin.x) / self.cell_size).clamp(0.0, (nx - 1) as f64);
        let fy = ((p.y - self.origin.y) / self.cell_size).clamp(0.0, (ny - 1) as f64);
        let fz = ((p.z - self.origin.z) / self.cell_size).clamp(0.0, (nz - 1) as f64);
        let ix0 = (fx.floor() as usize).min(nx.saturating_sub(2));
        let iy0 = (fy.floor() as usize).min(ny.saturating_sub(2));
        let iz0 = (fz.floor() as usize).min(nz.saturating_sub(2));
        let ix1 = (ix0 + 1).min(nx - 1);
        let iy1 = (iy0 + 1).min(ny - 1);
        let iz1 = (iz0 + 1).min(nz - 1);
        let tx = fx - ix0 as f64;
        let ty = fy - iy0 as f64;
        let tz = fz - iz0 as f64;

        let c000 = self.get(ix0, iy0, iz0);
        let c100 = self.get(ix1, iy0, iz0);
        let c010 = self.get(ix0, iy1, iz0);
        let c110 = self.get(ix1, iy1, iz0);
        let c001 = self.get(ix0, iy0, iz1);
        let c101 = self.get(ix1, iy0, iz1);
        let c011 = self.get(ix0, iy1, iz1);
        let c111 = self.get(ix1, iy1, iz1);

        let c00 = c000 * (1.0 - tx) + c100 * tx;
        let c10 = c010 * (1.0 - tx) + c110 * tx;
        let c01 = c001 * (1.0 - tx) + c101 * tx;
        let c11 = c011 * (1.0 - tx) + c111 * tx;
        let c0 = c00 * (1.0 - ty) + c10 * ty;
        let c1 = c01 * (1.0 - ty) + c11 * ty;
        c0 * (1.0 - tz) + c1 * tz
    }

    /// Central-difference gradient on the trilinear interpolant.
    /// Uses `cell_size` as the step; near the grid boundary the
    /// out-of-bounds half falls back to the clamped edge value so
    /// the gradient stays finite (one-sided in spirit, half-magnitude
    /// in practice).
    fn gradient(&self, p: Point3<f64>) -> Vector3<f64> {
        let h = self.cell_size;
        let inv_2h = 0.5 / h;
        let dx = self.sample_trilinear(Point3::new(p.x + h, p.y, p.z))
            - self.sample_trilinear(Point3::new(p.x - h, p.y, p.z));
        let dy = self.sample_trilinear(Point3::new(p.x, p.y + h, p.z))
            - self.sample_trilinear(Point3::new(p.x, p.y - h, p.z));
        let dz = self.sample_trilinear(Point3::new(p.x, p.y, p.z + h))
            - self.sample_trilinear(Point3::new(p.x, p.y, p.z - h));
        Vector3::new(dx * inv_2h, dy * inv_2h, dz * inv_2h)
    }
}

// ── 6-connected neighbour helper (private) ────────────────────────────

#[inline]
fn neighbours6(i: usize, nx: usize, ny: usize, nz: usize) -> [Option<usize>; 6] {
    let plane = nx * ny;
    let iz = i / plane;
    let rem = i % plane;
    let iy = rem / nx;
    let ix = rem % nx;
    [
        (ix > 0).then(|| i - 1),
        (ix + 1 < nx).then(|| i + 1),
        (iy > 0).then(|| i - nx),
        (iy + 1 < ny).then(|| i + nx),
        (iz > 0).then(|| i - plane),
        (iz + 1 < nz).then(|| i + plane),
    ]
}

/// Lattice dimensions covering `bounds` at `cell_size`, sample points
/// inclusive of both bounds endpoints.
///
/// Matches `insertion_sim::build_grid_sdf`'s dim formula:
/// `ceil(span / cell_size).max(1) + 1`. Differs slightly from
/// `ScalarGrid::from_bounds`'s rounding so the lattice spacing stays
/// exactly `cell_size` (sdf_layers gets this same property by setting
/// `padding = 0`).
//
// `s / cell_size` is non-negative + finite for valid bounds (verified
// at the caller), and the `+ 1` keeps the value in usize range for any
// scan-scale geometry. The `as usize` cast trims a finite ceil-rounded
// non-negative f64, which is its specified use.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn lattice_dims(bounds: Aabb, cell_size: f64) -> [usize; 3] {
    let span = bounds.max - bounds.min;
    let dim = |s: f64| (s / cell_size).ceil().max(1.0) as usize + 1;
    [dim(span.x), dim(span.y), dim(span.z)]
}

// ── Shared build core ─────────────────────────────────────────────────

/// Build the unsigned-distance lattice + flood-fill labels in one pass.
/// Shared between [`FloodFillSign::build`] and [`CachedGridSdf::build`].
fn build_classified_grid<D: UnsignedDistance + ?Sized>(
    distance: &D,
    bounds: Aabb,
    cell_size: f64,
    wall_threshold_factor: f64,
) -> Result<ClassifiedGrid, FloodFillError> {
    if !cell_size.is_finite() || cell_size <= 0.0 {
        return Err(FloodFillError::NonPositiveCellSize { cell_size });
    }
    let span = bounds.max - bounds.min;
    if span.x <= 0.0 || span.y <= 0.0 || span.z <= 0.0 {
        return Err(FloodFillError::DegenerateBounds);
    }

    #[cfg(not(target_arch = "wasm32"))]
    let started = Instant::now();
    let dims = lattice_dims(bounds, cell_size);
    let [nx, ny, nz] = dims;
    let n = nx * ny * nz;
    let origin = bounds.min;
    let wall_threshold_m = wall_threshold_factor * cell_size;

    // Pass 1 — unsigned distance at every lattice node. Lattice
    // indices fit in usize; the `as f64` cast is precise for any
    // workshop-scale grid (`< 2^52` lattice nodes per axis).
    #[allow(clippy::cast_precision_loss)]
    let world = |ix: usize, iy: usize, iz: usize| {
        Point3::new(
            origin.x + ix as f64 * cell_size,
            origin.y + iy as f64 * cell_size,
            origin.z + iz as f64 * cell_size,
        )
    };
    let mut unsigned = vec![0.0_f64; n];
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = world(ix, iy, iz);
                unsigned[SdfGrid::flat_index(dims, ix, iy, iz)] = distance.distance(p);
            }
        }
    }

    // Pass 2 — wall classification + flood from bbox corners.
    let mut region: Vec<Region> = unsigned
        .iter()
        .map(|&u| {
            if u < wall_threshold_m {
                Region::Wall
            } else {
                Region::Inside
            }
        })
        .collect();
    let wall_cells = region.iter().filter(|r| **r == Region::Wall).count();

    let flat = |ix: usize, iy: usize, iz: usize| SdfGrid::flat_index(dims, ix, iy, iz);
    let corners = [
        flat(0, 0, 0),
        flat(nx - 1, 0, 0),
        flat(0, ny - 1, 0),
        flat(nx - 1, ny - 1, 0),
        flat(0, 0, nz - 1),
        flat(nx - 1, 0, nz - 1),
        flat(0, ny - 1, nz - 1),
        flat(nx - 1, ny - 1, nz - 1),
    ];
    let mut flood: VecDeque<usize> = VecDeque::new();
    for &c in &corners {
        if region[c] == Region::Inside {
            region[c] = Region::Outside;
            flood.push_back(c);
        }
    }
    if flood.is_empty() {
        return Err(FloodFillError::NoOutsideSeed { wall_threshold_m });
    }
    while let Some(i) = flood.pop_front() {
        for j in neighbours6(i, nx, ny, nz).into_iter().flatten() {
            if region[j] == Region::Inside {
                region[j] = Region::Outside;
                flood.push_back(j);
            }
        }
    }

    // Pass 3 — multi-source label expansion into the wall band.
    let mut expand: VecDeque<usize> = (0..n).filter(|&i| region[i] != Region::Wall).collect();
    while let Some(i) = expand.pop_front() {
        let label = region[i];
        for j in neighbours6(i, nx, ny, nz).into_iter().flatten() {
            if region[j] == Region::Wall {
                region[j] = label;
                expand.push_back(j);
            }
        }
    }
    // Isolated wall pockets with no path to a labelled neighbour
    // default to Inside (matches cf-device-design's posture; safe
    // because such pockets sit deep in the wall band where their
    // signed magnitude is sub-threshold).
    for r in &mut region {
        if *r == Region::Wall {
            *r = Region::Inside;
        }
    }

    // Pass 4 — connected-component count over the post-expansion
    // Inside region (flood-leak detector). Healthy = 1.
    let mut seen = vec![false; n];
    let mut inside_components = 0usize;
    for start in 0..n {
        if region[start] != Region::Inside || seen[start] {
            continue;
        }
        inside_components += 1;
        seen[start] = true;
        let mut comp: VecDeque<usize> = VecDeque::from([start]);
        while let Some(i) = comp.pop_front() {
            for j in neighbours6(i, nx, ny, nz).into_iter().flatten() {
                if region[j] == Region::Inside && !seen[j] {
                    seen[j] = true;
                    comp.push_back(j);
                }
            }
        }
    }

    let inside_cells = region.iter().filter(|r| **r == Region::Inside).count();
    let outside_cells = n - inside_cells;

    // Sign-only: store a tight `Vec<bool>` (1 byte / cell, same as
    // `Region`). The shared core hands callers both the bool mask AND
    // the unsigned magnitudes — `CachedGridSdf::build` consumes the
    // magnitudes to form the signed grid; `FloodFillSign::build`
    // discards them.
    let inside_flags: Vec<bool> = region.iter().map(|r| *r == Region::Inside).collect();

    let mut min_signed = f64::INFINITY;
    for (i, &u) in unsigned.iter().enumerate() {
        let signed = if inside_flags[i] { -u } else { u };
        if signed < min_signed {
            min_signed = signed;
        }
    }
    if !min_signed.is_finite() {
        min_signed = 0.0;
    }

    #[cfg(not(target_arch = "wasm32"))]
    let build_secs = started.elapsed().as_secs_f64();
    // wasm32 has no `Instant`; report 0.0 so callers can still display
    // the field without a runtime panic. See the import-site comment.
    #[cfg(target_arch = "wasm32")]
    let build_secs = 0.0_f64;
    let report = FloodFillReport {
        dims,
        inside_cells,
        outside_cells,
        wall_cells,
        inside_components,
        wall_threshold_m,
        build_secs,
        min_signed_distance_m: min_signed,
    };

    Ok(ClassifiedGrid {
        inside_flags,
        unsigned,
        dims,
        origin,
        cell_size,
        report,
    })
}

struct ClassifiedGrid {
    inside_flags: Vec<bool>,
    unsigned: Vec<f64>,
    dims: [usize; 3],
    origin: Point3<f64>,
    cell_size: f64,
    report: FloodFillReport,
}

// ── FloodFillSign ─────────────────────────────────────────────────────

/// Sign-only cache built from a flood-fill over an unsigned-distance
/// oracle.
///
/// Composes with any [`UnsignedDistance`]; the typical pairing is
/// [`Signed<TriMeshDistance, FloodFillSign>`](Signed) for an
/// SDF whose magnitudes stay exact (parry queries the source mesh
/// every call) and whose sign is **topology-blind** — non-manifold
/// edges, inverted-winding cap fans, and high-valence apex vertices
/// all evaluate the same way they would on a clean mesh. That is the
/// property worth having, and `PseudoNormalSign` does not have it.
///
/// # ⚠⚠ The sign is a LATTICE lookup, so it is unreliable NEAR THE SURFACE
///
/// `is_inside` rounds to the nearest node (see below), which makes the
/// sign **piecewise-constant on cell-sized boxes**. The error is worst
/// *at* the surface — essentially a coin flip — and decays to exact by
/// one cell out; at half a cell it is already down to a few per cent,
/// so "within half a cell" understates how sharply it is concentrated
/// on the zero crossing. Measured on an analytic sphere at `cell = 0.1`
/// (`flood_fill_sign_is_unreliable_within_a_cell_of_the_surface`):
///
/// | distance from surface | signs wrong |
/// |---|---|
/// | 0.05 cell | **47.5 %** |
/// | 0.30 cell | 21.8 % |
/// | 0.50 cell | 6.8 % |
/// | ≥ 1.00 cell | **0 %** |
///
/// This doc previously called the sign "rock-solid" with no spatial
/// caveat. The rounding was documented; the consequence was not — and
/// every near-surface consumer sits exactly in the band where the
/// claim fails. **Choose by where you query**, not by which oracle
/// sounds sturdier: topology-blind far from the surface, but for
/// contact, projection-to-surface, or zero-isosurface extraction,
/// `PseudoNormalSign` is exact at the surface and fails only when a
/// pseudo-normal is zeroed — which
/// [`TriMeshDistance::health`](crate::TriMeshDistance::health) now
/// measures directly.
///
/// Build via [`FloodFillSign::build`]. The build cost is one
/// unsigned-distance query per lattice node plus three linear passes
/// over the grid (classify + BFS + multi-source label expansion); on
/// the iter-1 sock (~25 × 25 × 45 nodes at 3 mm cells) it lands in
/// the high tens of milliseconds.
///
/// `is_inside(p)` rounds `p` to the nearest lattice node and reads
/// its stored label. Queries outside the build bounds are reported
/// as **outside** (defensive default — the body is enclosed by
/// construction).
///
/// [`TriMeshDistance`]: crate::TriMeshDistance
#[derive(Debug, Clone)]
pub struct FloodFillSign {
    inside_flags: Vec<bool>,
    dims: [usize; 3],
    origin: Point3<f64>,
    cell_size: f64,
}

impl FloodFillSign {
    /// Build over `distance` covering `bounds` at `cell_size` lattice
    /// spacing. `wall_threshold_factor` controls the wall band as a
    /// multiple of `cell_size`; [`WALL_THRESHOLD_FACTOR_DEFAULT`]
    /// (0.75) matches cf-device-design's shipped value and is what
    /// most callers want.
    ///
    /// # Errors
    ///
    /// - [`FloodFillError::NonPositiveCellSize`] when `cell_size` is
    ///   non-finite or non-positive.
    /// - [`FloodFillError::DegenerateBounds`] when `bounds` has a
    ///   non-positive span on any axis.
    /// - [`FloodFillError::NoOutsideSeed`] when every corner of the
    ///   lattice falls within the wall band — the bounds margin must
    ///   extend further past the body so at least one corner is
    ///   outside it.
    pub fn build<D: UnsignedDistance + ?Sized>(
        distance: &D,
        bounds: Aabb,
        cell_size: f64,
        wall_threshold_factor: f64,
    ) -> Result<(Self, FloodFillReport), FloodFillError> {
        let ClassifiedGrid {
            inside_flags,
            unsigned: _,
            dims,
            origin,
            cell_size,
            report,
        } = build_classified_grid(distance, bounds, cell_size, wall_threshold_factor)?;
        Ok((
            Self {
                inside_flags,
                dims,
                origin,
                cell_size,
            },
            report,
        ))
    }

    /// Lattice spacing this sign cache was built at.
    #[must_use]
    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }

    /// Lattice dimensions (`[nx, ny, nz]`).
    #[must_use]
    pub fn dims(&self) -> [usize; 3] {
        self.dims
    }

    /// World-space origin of the lattice (`(0, 0, 0)` node).
    #[must_use]
    pub fn origin(&self) -> Point3<f64> {
        self.origin
    }

    // Round-to-nearest-lattice-node converts a finite f64 to a usize
    // index after explicit non-finite + negative + over-range guards
    // below; the casts are gated by those checks, so truncation /
    // sign loss are by construction not reachable.
    #[inline]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn lookup_node(&self, p: Point3<f64>) -> Option<usize> {
        let [nx, ny, nz] = self.dims;
        if nx == 0 || ny == 0 || nz == 0 {
            return None;
        }
        let fx = (p.x - self.origin.x) / self.cell_size;
        let fy = (p.y - self.origin.y) / self.cell_size;
        let fz = (p.z - self.origin.z) / self.cell_size;
        if !(fx.is_finite() && fy.is_finite() && fz.is_finite()) {
            return None;
        }
        let ix = fx.round();
        let iy = fy.round();
        let iz = fz.round();
        if ix < 0.0 || iy < 0.0 || iz < 0.0 {
            return None;
        }
        let ix = ix as usize;
        let iy = iy as usize;
        let iz = iz as usize;
        if ix >= nx || iy >= ny || iz >= nz {
            return None;
        }
        Some(SdfGrid::flat_index(self.dims, ix, iy, iz))
    }
}

impl Sign for FloodFillSign {
    fn is_inside(&self, p: Point3<f64>) -> bool {
        self.lookup_node(p)
            .map(|i| self.inside_flags[i])
            .unwrap_or(false)
    }
}

// ── CachedGridSdf ─────────────────────────────────────────────────────

/// Full signed-distance grid built from a flood-fill over an
/// unsigned-distance oracle.
///
/// Owns: the signed-distance grid (unsigned magnitudes × flood-fill
/// sign per cell) and the [`FloodFillSign`] (retained for `is_inside`
/// queries between lattice nodes). Queries are O(1) — trilinear
/// interpolation of the signed grid.
///
/// **Picking a cache**:
/// - [`FloodFillSign`] composed with [`TriMeshDistance`] keeps every
///   unsigned-distance query exact at the cost of one parry BVH
///   query per call. Right choice for one-shot consumers like
///   `cf-cast-cli`'s marching-cubes sweep where each cell is touched
///   once.
/// - [`CachedGridSdf`] (this type) bakes the unsigned distance into
///   the lattice once and replaces every per-call parry query with
///   trilinear interpolation. Right choice when the same SDF is
///   queried many times (cf-device-design's preview re-uses one
///   `CachedScanSdf` across frames; insertion-sim samples one
///   `GridSdf` at every BCC tet node).
///
/// `eval(p)` returns negative inside, positive outside. The
/// `cf-design::Sdf` impl lives in cf-design (orphan rule).
///
/// [`TriMeshDistance`]: crate::TriMeshDistance
#[derive(Debug, Clone)]
pub struct CachedGridSdf {
    grid: SdfGrid,
    sign: FloodFillSign,
}

impl CachedGridSdf {
    /// Build over `distance` covering `bounds` at `cell_size` lattice
    /// spacing. Same `wall_threshold_factor` semantics as
    /// [`FloodFillSign::build`].
    ///
    /// # Errors
    ///
    /// Forwards [`FloodFillError`] variants from the shared flood-fill
    /// core — see [`FloodFillSign::build`].
    pub fn build<D: UnsignedDistance + ?Sized>(
        distance: &D,
        bounds: Aabb,
        cell_size: f64,
        wall_threshold_factor: f64,
    ) -> Result<(Self, FloodFillReport), FloodFillError> {
        let ClassifiedGrid {
            inside_flags,
            unsigned,
            dims,
            origin,
            cell_size,
            report,
        } = build_classified_grid(distance, bounds, cell_size, wall_threshold_factor)?;

        let signed: Vec<f64> = inside_flags
            .iter()
            .zip(&unsigned)
            .map(|(&inside, &u)| if inside { -u } else { u })
            .collect();

        let grid = SdfGrid::filled(dims, origin, cell_size, signed);
        let sign = FloodFillSign {
            inside_flags,
            dims,
            origin,
            cell_size,
        };
        Ok((Self { grid, sign }, report))
    }

    /// Signed-distance value at world point `p` — trilinear
    /// interpolation of the cached grid (negative inside, positive
    /// outside). Out-of-bounds queries return the clamped edge value.
    #[must_use]
    pub fn signed_distance(&self, p: Point3<f64>) -> f64 {
        self.grid.sample_trilinear(p)
    }

    /// Analytic gradient of the trilinear interpolant at `p` via
    /// central differences with step `cell_size`. Magnitude
    /// approximates `‖∇φ‖ ≈ 1` for points off the surface; direction
    /// approximates the surface normal.
    #[must_use]
    pub fn gradient(&self, p: Point3<f64>) -> Vector3<f64> {
        self.grid.gradient(p)
    }

    /// Borrow the underlying sign oracle.
    #[must_use]
    pub fn sign(&self) -> &FloodFillSign {
        &self.sign
    }

    /// Lattice spacing this cache was built at.
    #[must_use]
    pub fn cell_size(&self) -> f64 {
        self.grid.cell_size
    }

    /// Lattice dimensions (`[nx, ny, nz]`).
    #[must_use]
    pub fn dims(&self) -> [usize; 3] {
        self.grid.dims
    }

    /// World-space origin of the lattice (`(0, 0, 0)` node).
    #[must_use]
    pub fn origin(&self) -> Point3<f64> {
        self.grid.origin
    }
}

impl UnsignedDistance for CachedGridSdf {
    fn distance(&self, p: Point3<f64>) -> f64 {
        self.signed_distance(p).abs()
    }

    /// Closest-point estimate via one Newton step along the
    /// (interpolated) gradient: `p − φ(p) · ∇φ / ‖∇φ‖`. Exact only
    /// when the gradient is unit-length; CachedGridSdf trilinear
    /// gradients are approximately unit-length away from the surface
    /// and degrade smoothly into the wall band.
    fn closest_point(&self, p: Point3<f64>) -> Point3<f64> {
        let phi = self.signed_distance(p);
        let g = self.gradient(p);
        let n = g.norm();
        if n > 1e-12 { p - (phi / n) * g } else { p }
    }
}

impl Sign for CachedGridSdf {
    /// ⚠ **Delegates to [`FloodFillSign`], so this is the LATTICE lookup — not the
    /// interpolated field [`CachedGridSdf::signed_distance`] answers from.** The same
    /// object therefore reports differently near the surface depending on which method you
    /// call: at 0.05 cell, ~48 % of `is_inside` answers are wrong against ~8 % of
    /// `signed_distance` signs
    /// (`cached_grid_signed_distance_does_not_inherit_the_lattice_sign_defect`).
    ///
    /// Interpolating is **a factor, not a fix** — the stored node values are
    /// `magnitude × that node's own flood-fill label`, so a mislabelled node still puts the
    /// zero crossing in the wrong place. Prefer `signed_distance` near the surface, and do
    /// not treat either as exact there.
    fn is_inside(&self, p: Point3<f64>) -> bool {
        self.sign.is_inside(p)
    }
}

// ── Convenience builders ──────────────────────────────────────────────

/// Build a `Signed<TriMeshDistance, FloodFillSign>` from a triangle
/// mesh — one BVH allocation, one flood-fill cache.
///
/// ⚠ **Recommended for cleaned scans queried AWAY from the surface,
/// not everywhere.** The composition inherits [`FloodFillSign`]'s
/// lattice-lookup sign, so within a cell of the surface the signed
/// distance carries an exact magnitude with a possibly-wrong sign —
/// see that type's near-surface table. Consumers that query the zero
/// band (contact, projection, iso-0 extraction) should read it before
/// adopting this.
///
/// Otherwise: distance magnitudes stay exact via the parry BVH,
/// sign comes from the topology-blind flood-fill so cap-fan winding
/// flips and high-valence apex vertices can't poison far-field
/// queries.
///
/// **Ownership.** `mesh` is consumed (moved into the underlying
/// [`crate::TriMeshDistance`] which retains it for `mesh()` access
/// and the parry BVH). Clone before calling if the caller still
/// needs the original.
///
/// # Errors
///
/// - [`crate::error::SdfError::EmptyMesh`] when `mesh` has no faces
///   (forwarded from [`crate::TriMeshDistance::new`]).
/// - [`FloodFillError`] from the flood-fill build — see
///   [`FloodFillSign::build`].
pub fn flood_filled_sdf(
    mesh: mesh_types::IndexedMesh,
    bounds: Aabb,
    cell_size: f64,
    wall_threshold_factor: f64,
) -> Result<
    (
        Signed<crate::TriMeshDistance, FloodFillSign>,
        FloodFillReport,
    ),
    FloodFilledSdfBuildError,
> {
    let distance = crate::TriMeshDistance::new(mesh).map_err(FloodFilledSdfBuildError::Sdf)?;
    let (sign, report) = FloodFillSign::build(&distance, bounds, cell_size, wall_threshold_factor)
        .map_err(FloodFilledSdfBuildError::Flood)?;
    Ok((Signed { distance, sign }, report))
}

/// Error variants for [`flood_filled_sdf`].
///
/// Named `…BuildError` (rather than just `FloodFilledSdfError`) so the
/// shape of the failure — *building* the SDF from a mesh + bounds —
/// is visible at the call site without having to read the variants.
#[derive(Debug, thiserror::Error)]
pub enum FloodFilledSdfBuildError {
    /// Construction of the underlying [`crate::TriMeshDistance`] failed.
    #[error(transparent)]
    Sdf(#[from] crate::SdfError),
    /// The flood-fill build failed.
    #[error(transparent)]
    Flood(#[from] FloodFillError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PseudoNormalSign, TriMeshDistance};
    use approx::assert_relative_eq;
    use mesh_types::{IndexedMesh, Point3};

    /// **`FloodFillSign::is_inside` is a LATTICE lookup, so its sign is piecewise-constant
    /// on cell-sized boxes — and AT the surface it is close to a coin flip.**
    ///
    /// `lookup_node` rounds the query to the nearest node and reads that node's label. The
    /// mechanism has always been documented; the *consequence* was not, and the type's own
    /// doc called the sign "rock-solid" with no spatial caveat while every near-surface
    /// consumer sits exactly in the band where it is not.
    ///
    /// This pins the shape of the error rather than a single number, because the shape is
    /// the thing a caller needs: **unusable at the surface, exact from one cell out.**
    ///
    /// ⚠ **If this test fails because the near-surface arm got BETTER, that is the R4 fix
    /// landing — update this gate, do not delete it.** It pins a known defect on purpose,
    /// which is the only honest way to ship one.
    ///
    /// ⚠ **Is the fixture's own tessellation a confound?** It is the obvious objection: the
    /// shells are placed against the *analytic* sphere while the flood fill was built from a
    /// polyhedron. No — checked both ways. The polyhedron deviates inward by at most
    /// `r(1 − cos(π/2n_lat))` ≈ 1.2e-3, against a closest sampling offset of 5.0e-3, so both
    /// shells sit unambiguously on their intended side with ~4× margin. And empirically the
    /// answer does not move with tessellation: 16×32 / 32×64 / 48×96 give 47.3 / 47.7 /
    /// 47.7 % at 0.05 cell. The error is a property of the lattice, not of the mesh.
    #[test]
    fn flood_fill_sign_is_unreliable_within_a_cell_of_the_surface() {
        let r = 1.0_f64;
        let cell = 0.1_f64;
        let mesh = crate::test_fixtures::uv_sphere(r, 32, 64);
        let bounds = Aabb::new(Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0));
        let (sdf, _report) =
            flood_filled_sdf(mesh, bounds, cell, WALL_THRESHOLD_FACTOR_DEFAULT).expect("build");

        // Shell sampler: points at `d` inside and outside the true sphere surface.
        let wrong_at = |d: f64| {
            let (mut wrong, mut total) = (0usize, 0usize);
            let n = 40;
            for i in 0..n {
                for j in 0..n {
                    let th = std::f64::consts::PI * (f64::from(i) + 0.5) / f64::from(n);
                    let ph = 2.0 * std::f64::consts::PI * (f64::from(j) + 0.5) / f64::from(n);
                    for (sgn, want_inside) in [(-1.0_f64, true), (1.0, false)] {
                        let rr = r + sgn * d;
                        let p = Point3::new(
                            rr * th.sin() * ph.cos(),
                            rr * th.sin() * ph.sin(),
                            rr * th.cos(),
                        );
                        total += 1;
                        if sdf.sign.is_inside(p) != want_inside {
                            wrong += 1;
                        }
                    }
                }
            }
            #[allow(clippy::cast_precision_loss)]
            let pct = 100.0 * wrong as f64 / total as f64;
            println!(
                "  offset {:.3} = {:.2} cell | wrong {wrong}/{total} ({pct:.1} %)",
                d,
                d / cell
            );
            pct
        };

        println!("\nFloodFillSign::is_inside vs the analytic sphere:");
        let at_005 = wrong_at(0.05 * cell);
        let _ = wrong_at(0.30 * cell);
        let at_050 = wrong_at(0.50 * cell);
        let at_100 = wrong_at(1.00 * cell);
        let at_200 = wrong_at(2.00 * cell);

        // ── The defect, pinned. Non-vacuity: it must be LARGE, not merely non-zero. ──
        assert!(
            at_005 > 30.0,
            "only {at_005:.1} % of signs are wrong at 0.05 cell from the surface. Either the \
             lattice lookup stopped being piecewise-constant (good — see the note above) or \
             this fixture no longer samples the band, in which case the assertions below \
             prove nothing"
        );
        assert!(
            at_050 > 1.0,
            "the half-cell shell is already clean ({at_050:.1} %), so the fixture's cell is \
             too coarse relative to the sampling to show the transition"
        );

        // ── The regime that IS trustworthy, which is what makes the caveat actionable. ──
        assert!(
            at_100 == 0.0 && at_200 == 0.0,
            "signs are wrong beyond a full cell from the surface ({at_100:.1} %, {at_200:.1} %) \
             — then the failure is not merely near-surface and the doc's promised safe \
             regime does not exist"
        );
    }

    /// **The same type answers with different fidelity depending on which method you call.**
    ///
    /// [`CachedGridSdf::signed_distance`] interpolates **stored signed values**, so it
    /// crosses zero smoothly instead of stepping; `impl Sign for CachedGridSdf` delegates
    /// straight to [`FloodFillSign`], so `is_inside` on the very same object does not.
    ///
    /// ★ **Measured, and the measurement corrected the hypothesis.** The first version of
    /// this gate asserted the interpolated path was *clean*. It is not: at 0.05 cell it is
    /// still ~8 % wrong, against ~48 % for the lattice lookup — **5–6× better, not fixed.**
    /// The reason is that the stored node values are `magnitude × the node's own flood-fill
    /// label`, so a mislabelled node contributes a wrongly-signed value and interpolation
    /// merely puts the zero crossing in the wrong place instead of stepping to it.
    ///
    /// So the honest claim, which is what the docs now make: **both accessors degrade
    /// within a cell of the surface**, and choosing the interpolated one buys a factor, not
    /// a guarantee.
    ///
    /// ⚠ This gate measures the **contrast between the two accessors**, not correctness. A
    /// fault that moves both equally — a globally inverted label set, say — leaves the
    /// ratio intact and passes here; that is
    /// `flood_fill_sign_is_unreliable_within_a_cell_of_the_surface`'s far-field arm to
    /// catch, and it does (verified by inverting `inside_flags`).
    #[test]
    fn cached_grid_signed_distance_does_not_inherit_the_lattice_sign_defect() {
        let r = 1.0_f64;
        let cell = 0.1_f64;
        let mesh = crate::test_fixtures::uv_sphere(r, 32, 64);
        let bounds = Aabb::new(Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0));
        let distance = TriMeshDistance::new(mesh).expect("scalable");
        let (grid, _report) =
            CachedGridSdf::build(&distance, bounds, cell, WALL_THRESHOLD_FACTOR_DEFAULT)
                .expect("build");

        let d = 0.05 * cell;
        let (mut interp_wrong, mut lookup_wrong, mut total) = (0usize, 0usize, 0usize);
        let n = 40;
        for i in 0..n {
            for j in 0..n {
                let th = std::f64::consts::PI * (f64::from(i) + 0.5) / f64::from(n);
                let ph = 2.0 * std::f64::consts::PI * (f64::from(j) + 0.5) / f64::from(n);
                for (sgn, want_inside) in [(-1.0_f64, true), (1.0, false)] {
                    let rr = r + sgn * d;
                    let p = Point3::new(
                        rr * th.sin() * ph.cos(),
                        rr * th.sin() * ph.sin(),
                        rr * th.cos(),
                    );
                    total += 1;
                    if (grid.signed_distance(p) < 0.0) != want_inside {
                        interp_wrong += 1;
                    }
                    if grid.is_inside(p) != want_inside {
                        lookup_wrong += 1;
                    }
                }
            }
        }
        println!(
            "\nat 0.05 cell: signed_distance wrong {interp_wrong}/{total} | is_inside wrong \
             {lookup_wrong}/{total}"
        );

        assert!(
            lookup_wrong * 3 > total,
            "`CachedGridSdf::is_inside` is not reproducing the lattice defect \
             ({lookup_wrong}/{total}), so the contrast this test draws is invisible and the \
             doc it backs would be unearned"
        );
        assert!(
            interp_wrong * 3 < lookup_wrong,
            "`signed_distance` ({interp_wrong}/{total}) is not decisively better than \
             `is_inside` ({lookup_wrong}/{total}). Then the two accessors must be documented \
             as equally unsafe near the surface"
        );
        // ★ And the half that matters more, because it is the one a reader will get wrong:
        //   interpolation is NOT a fix. Pinned so nobody upgrades "better" into "safe".
        assert!(
            interp_wrong * 100 > total,
            "`signed_distance` is now under 1 % wrong at 0.05 cell ({interp_wrong}/{total}). \
             If the trilinear path became genuinely clean that is good news and this gate \
             should be re-anchored — but the doc that calls it 'a factor, not a guarantee' \
             must be corrected in the same change"
        );
    }

    // ── Synthetic fixtures (spec §9) ──────────────────────────────────

    /// Watertight square-base pyramid — sanity case; both oracles
    /// must produce correct sign everywhere.
    fn closed_pyramid() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(-1.0, -1.0, 0.0)); // 0
        mesh.vertices.push(Point3::new(1.0, -1.0, 0.0)); // 1
        mesh.vertices.push(Point3::new(1.0, 1.0, 0.0)); // 2
        mesh.vertices.push(Point3::new(-1.0, 1.0, 0.0)); // 3
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0)); // 4 apex
        // Base (outward normal -z, CCW from below).
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // Sides (outward CCW).
        mesh.faces.push([0, 1, 4]);
        mesh.faces.push([1, 2, 4]);
        mesh.faces.push([2, 3, 4]);
        mesh.faces.push([3, 0, 4]);
        mesh
    }

    /// Same pyramid with the BASE winding inverted — base normals now
    /// point +z (INTO the body) instead of -z (out of it). Mimics the
    /// pre-B cf-scan-prep auto-cap output where `auto_cap_open_boundaries`
    /// emitted inward-pointing cap-fan triangles ([[project-cf-scan-prep-cap-winding-concern]]).
    ///
    /// A pseudo-normal sign query at `(0, 0, -3)` finds the base as
    /// the closest face, asks `(probe − closest) · normal`, gets
    /// `(0, 0, -3) · (0, 0, +1) = -3 < 0`, and reports "inside" — the
    /// canonical failure mode this fixture exists to pin.
    fn dome_with_inward_cap() -> IndexedMesh {
        let mut mesh = closed_pyramid();
        // Replace [0, 2, 1] + [0, 3, 2] with [0, 1, 2] + [0, 2, 3] —
        // same triangles, reversed winding.
        mesh.faces[0] = [0, 1, 2];
        mesh.faces[1] = [0, 2, 3];
        mesh
    }

    /// Same shape as [`closed_pyramid`] with *explicit* outward base
    /// winding — kept as the named fixture-pair counterpart to
    /// [`dome_with_inward_cap`] so the symmetry between
    /// "pre-B (broken)" and "post-B (target)" is legible in test
    /// names. Both oracles must agree everywhere on this fixture.
    /// Construction-wise it is literally `closed_pyramid()`; the
    /// naming carries the spec §9 semantic (post-B target).
    fn dome_with_outward_cap() -> IndexedMesh {
        closed_pyramid()
    }

    /// Hemisphere-style dome with a 32-fan apex vertex over a
    /// 32-sided regular polygon base + base-centroid cap-fan.
    /// Exercises the pseudo-normal aggregation at a near-singular
    /// vertex (32 incident triangles, each with a slightly different
    /// outward normal).
    fn high_valence_dome_apex() -> IndexedMesh {
        const N: usize = 32;
        const R: f64 = 1.0;
        const H: f64 = 0.6;
        let mut mesh = IndexedMesh::new();

        // 0..N: ring at z=0 on the unit circle.
        for i in 0..N {
            #[allow(clippy::cast_precision_loss)]
            let theta = (i as f64) * 2.0 * std::f64::consts::PI / (N as f64);
            mesh.vertices
                .push(Point3::new(R * theta.cos(), R * theta.sin(), 0.0));
        }
        let apex = N;
        let base_center = N + 1;
        mesh.vertices.push(Point3::new(0.0, 0.0, H)); // apex
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0)); // base center

        for i in 0..N {
            let a = i as u32;
            let b = ((i + 1) % N) as u32;
            // Side: outward CCW (viewed from outside, ring vertices
            // wind toward the apex).
            mesh.faces.push([a, b, apex as u32]);
            // Base cap-fan: outward CCW (viewed from below, i.e.
            // outward normal -z).
            mesh.faces.push([b, a, base_center as u32]);
        }
        mesh
    }

    fn aabb(min: Point3<f64>, max: Point3<f64>) -> Aabb {
        Aabb::new(min, max)
    }

    fn pyramid_bounds() -> Aabb {
        // 3 mm of margin past the unit pyramid in every direction
        // (well clear of the apex at z=1 and the base at z=0).
        aabb(Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0))
    }

    // ── FloodFillSign + CachedGridSdf — core machinery ───────────────

    #[test]
    fn flood_fill_sign_correct_on_closed_pyramid() {
        let mesh = closed_pyramid();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        let (sign, report) =
            FloodFillSign::build(&distance, pyramid_bounds(), 0.1, 0.75).expect("flood-fill build");

        assert_eq!(
            report.inside_components, 1,
            "healthy fixture must produce one inside component"
        );

        // Deep inside the pyramid.
        assert!(sign.is_inside(Point3::new(0.0, 0.0, 0.5)));
        // Far above the apex.
        assert!(!sign.is_inside(Point3::new(0.0, 0.0, 1.5)));
        // Far below the base.
        assert!(!sign.is_inside(Point3::new(0.0, 0.0, -1.0)));
        // Lateral.
        assert!(!sign.is_inside(Point3::new(1.5, 0.0, 0.5)));
    }

    /// **Load-bearing for B's gate.** PseudoNormalSign returns the
    /// wrong sign for points "below" the inverted base — the base
    /// pseudo-normal points +z (into the body) so a probe at z < 0
    /// gets `(probe − closest) · normal < 0` → claims inside.
    /// FloodFillSign is topology-blind and returns correct sign.
    #[test]
    fn pseudo_normal_wrong_flood_fill_right_on_inward_cap() {
        let mesh = dome_with_inward_cap();
        let distance = TriMeshDistance::new(mesh.clone()).expect("non-empty mesh");
        let pseudo = PseudoNormalSign::from_distance(&distance);
        let (flood, report) =
            FloodFillSign::build(&distance, pyramid_bounds(), 0.1, 0.75).expect("flood build");

        assert_eq!(report.inside_components, 1);

        // Probe well below the (inverted) base — the canonical
        // failure direction.
        let probe_below = Point3::new(0.0, 0.0, -1.0);
        assert!(
            pseudo.is_inside(probe_below),
            "PseudoNormalSign should incorrectly report this probe as INSIDE — \
             that's the documented failure mode this fixture exists to pin"
        );
        assert!(
            !flood.is_inside(probe_below),
            "FloodFillSign must correctly report the probe below the inverted \
             cap as OUTSIDE — this is the workshop-iter-1 sign-defense gate"
        );

        // Deep inside the body — both oracles should agree.
        let interior = Point3::new(0.0, 0.0, 0.5);
        assert!(flood.is_inside(interior));
    }

    #[test]
    fn flood_fill_matches_pseudo_normal_on_outward_cap() {
        let mesh = dome_with_outward_cap();
        let distance = TriMeshDistance::new(mesh.clone()).expect("non-empty mesh");
        let pseudo = PseudoNormalSign::from_distance(&distance);
        let (flood, report) =
            FloodFillSign::build(&distance, pyramid_bounds(), 0.1, 0.75).expect("flood build");

        assert_eq!(report.inside_components, 1);

        // Spec §9: post-B target — both oracles agree on the
        // outward-winding fixture.
        let probes_outside = [
            Point3::new(0.0, 0.0, 1.5),  // above apex
            Point3::new(0.0, 0.0, -1.0), // below base
            Point3::new(1.5, 0.0, 0.5),  // far +x
            Point3::new(-1.5, 0.0, 0.5),
            Point3::new(0.0, 1.5, 0.5),
            Point3::new(0.0, -1.5, 0.5),
        ];
        for p in probes_outside {
            assert!(!pseudo.is_inside(p), "pseudo: outside at {p:?}");
            assert!(!flood.is_inside(p), "flood: outside at {p:?}");
        }

        let probe_inside = Point3::new(0.0, 0.0, 0.4);
        assert!(pseudo.is_inside(probe_inside));
        assert!(flood.is_inside(probe_inside));
    }

    #[test]
    fn flood_fill_health_on_high_valence_apex() {
        let mesh = high_valence_dome_apex();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        let (flood, report) = FloodFillSign::build(
            &distance,
            aabb(Point3::new(-1.5, -1.5, -1.0), Point3::new(1.5, 1.5, 1.5)),
            0.05,
            0.75,
        )
        .expect("flood build");

        // Flood-fill health: one connected inside component.
        assert_eq!(
            report.inside_components, 1,
            "32-fan dome must produce a single connected inside region"
        );

        // Interior probe near the dome center.
        assert!(flood.is_inside(Point3::new(0.0, 0.0, 0.3)));
        // Exterior probes — well above apex, well below base, lateral.
        assert!(!flood.is_inside(Point3::new(0.0, 0.0, 1.4)));
        assert!(!flood.is_inside(Point3::new(0.0, 0.0, -0.9)));
        assert!(!flood.is_inside(Point3::new(1.4, 0.0, 0.3)));
    }

    // ── CachedGridSdf ────────────────────────────────────────────────

    #[test]
    fn cached_grid_sdf_signs_match_flood_fill() {
        let mesh = closed_pyramid();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        let (cached, report) =
            CachedGridSdf::build(&distance, pyramid_bounds(), 0.1, 0.75).expect("build");

        assert_eq!(report.inside_components, 1);

        // Interior probe — negative signed distance, is_inside true,
        // gradient finite. Gradient MAGNITUDE varies across the
        // pyramid interior because the polyhedral SDF has a medial
        // axis (loci equidistant from multiple faces) where
        // `‖∇φ‖` drops well below 1.0; pinning the magnitude here
        // would test the trilinear-smoothed interior structure
        // rather than the build. Direction is pinned separately on
        // exterior probes below where the surface is single-face.
        let inside = Point3::new(0.0, 0.0, 0.4);
        let phi_in = cached.signed_distance(inside);
        assert!(
            phi_in < 0.0,
            "interior signed distance must be negative, got {phi_in}"
        );
        assert!(cached.is_inside(inside));
        let grad_in = cached.gradient(inside);
        assert!(
            grad_in.x.is_finite() && grad_in.y.is_finite() && grad_in.z.is_finite(),
            "interior gradient must be finite, got {grad_in:?}"
        );

        // Exterior probe — positive signed distance + gradient
        // approximates the surface outward normal (here +z above
        // the apex). Single-face region so `‖∇φ‖ ≈ 1`.
        let outside = Point3::new(0.0, 0.0, 1.5);
        let phi_out = cached.signed_distance(outside);
        assert!(phi_out > 0.0);
        assert!(!cached.is_inside(outside));
        // The exterior probe is 0.5 units above the apex; trilinear-
        // interpolated distance should be close to 0.5.
        assert_relative_eq!(phi_out, 0.5, epsilon = 0.05);
        let grad_out = cached.gradient(outside);
        assert!(
            grad_out.z > 0.5,
            "above-apex gradient must point predominantly +z, got {grad_out:?}"
        );
    }

    #[test]
    fn cached_grid_sdf_implements_unsigned_distance_via_abs() {
        let mesh = closed_pyramid();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        let (cached, _) =
            CachedGridSdf::build(&distance, pyramid_bounds(), 0.1, 0.75).expect("build");

        let p = Point3::new(0.0, 0.0, 0.4);
        let signed = cached.signed_distance(p);
        let unsigned = <CachedGridSdf as UnsignedDistance>::distance(&cached, p);
        assert_relative_eq!(unsigned, signed.abs(), epsilon = 1e-12);
    }

    // ── Error paths ──────────────────────────────────────────────────

    #[test]
    fn flood_fill_rejects_non_positive_cell_size() {
        let mesh = closed_pyramid();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        let err = FloodFillSign::build(&distance, pyramid_bounds(), 0.0, 0.75)
            .expect_err("non-positive cell_size must fail");
        assert!(matches!(err, FloodFillError::NonPositiveCellSize { .. }));
    }

    #[test]
    fn flood_fill_rejects_degenerate_bounds() {
        let mesh = closed_pyramid();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        let degenerate = aabb(Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
        let err = FloodFillSign::build(&distance, degenerate, 0.1, 0.75)
            .expect_err("degenerate bounds must fail");
        assert!(matches!(err, FloodFillError::DegenerateBounds));
    }

    #[test]
    fn flood_fill_bails_when_corners_are_within_wall_band() {
        let mesh = closed_pyramid();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        // Tight bounds (just past the pyramid) + huge wall threshold
        // factor: the corners land within the wall band.
        let tight = aabb(
            Point3::new(-1.05, -1.05, -0.05),
            Point3::new(1.05, 1.05, 1.05),
        );
        let err = FloodFillSign::build(&distance, tight, 0.1, 50.0)
            .expect_err("absurd wall band must trip NoOutsideSeed");
        assert!(matches!(err, FloodFillError::NoOutsideSeed { .. }));
    }

    // ── flood_filled_sdf helper ───────────────────────────────────────

    /// Smoke-test the one-call ergonomic helper end-to-end. Pins the
    /// API contract (signature + return shape + sign behavior) so
    /// future refactors of the helper don't regress its surface.
    #[test]
    fn flood_filled_sdf_helper_round_trips_through_signed() {
        let mesh = closed_pyramid();
        let (sdf, report) = flood_filled_sdf(mesh, pyramid_bounds(), 0.1, 0.75)
            .expect("flood_filled_sdf builds for a healthy mesh");

        assert_eq!(report.inside_components, 1);

        // The returned `Signed<TriMeshDistance, FloodFillSign>` carries
        // the full unsigned-distance + flood-fill-sign composition.
        let inside = Point3::new(0.0, 0.0, 0.5);
        let outside = Point3::new(0.0, 0.0, 1.5);

        assert!(sdf.evaluate(inside) < 0.0);
        assert!(sdf.evaluate(outside) > 0.0);
        assert!(sdf.is_inside(inside));
        assert!(!sdf.is_inside(outside));
        assert!(sdf.unsigned_distance(outside) > 0.0);

        // `mesh()` accessor (the `impl<S: Sign> Signed<TriMeshDistance, S>`)
        // works on the recommended FloodFillSign composition too.
        assert_eq!(sdf.mesh().faces.len(), 6);
    }

    /// Empty mesh propagates through as the underlying
    /// `SdfError::EmptyMesh` variant of the build error.
    #[test]
    fn flood_filled_sdf_helper_propagates_empty_mesh_error() {
        let empty = IndexedMesh::new();
        let err =
            flood_filled_sdf(empty, pyramid_bounds(), 0.1, 0.75).expect_err("empty mesh must fail");
        assert!(matches!(err, FloodFilledSdfBuildError::Sdf(_)));
    }

    /// `wall_threshold_factor` so large that every corner sits inside
    /// the wall band propagates through as the underlying
    /// `FloodFillError::NoOutsideSeed` variant.
    #[test]
    fn flood_filled_sdf_helper_propagates_flood_error() {
        let mesh = closed_pyramid();
        let tight = aabb(
            Point3::new(-1.05, -1.05, -0.05),
            Point3::new(1.05, 1.05, 1.05),
        );
        let err = flood_filled_sdf(mesh, tight, 0.1, 50.0).expect_err("absurd wall band must fail");
        assert!(matches!(err, FloodFilledSdfBuildError::Flood(_)));
    }

    // ── CachedGridSdf::closest_point ─────────────────────────────────

    /// `closest_point` for an exterior probe directly above the apex
    /// should step back to roughly the apex itself — the gradient
    /// points predominantly +z in that region (single-face), so one
    /// Newton step along `-(φ / ‖∇φ‖) · ∇φ` lands near the surface.
    #[test]
    fn cached_grid_sdf_closest_point_recovers_apex() {
        let mesh = closed_pyramid();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        let (cached, _) =
            CachedGridSdf::build(&distance, pyramid_bounds(), 0.1, 0.75).expect("build");

        let probe = Point3::new(0.0, 0.0, 1.5);
        let cp = <CachedGridSdf as UnsignedDistance>::closest_point(&cached, probe);
        // The apex sits at (0, 0, 1); trilinear-grid Newton step lands
        // within one cell of it. Generous epsilon — the grid is
        // 0.1-cell coarse and the apex is a singular point.
        assert!(
            (cp - Point3::new(0.0, 0.0, 1.0)).norm() < 0.2,
            "closest_point should recover near the apex, got {cp:?}"
        );
    }

    // ── inside_components > 1 (flood-leak / multi-body detector) ─────

    /// Two spatially-separated tetrahedra in one bounds → flood from
    /// the corners marks everything-but-the-tetrahedra as Outside,
    /// leaving two disconnected Inside regions. Pins the `> 1`
    /// branch of the connected-component count that healthy fixtures
    /// can't exercise.
    fn two_disjoint_tetrahedra() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // Tet A near origin.
        for (x, y, z) in [
            (0.0, 0.0, 0.0),
            (0.4, 0.0, 0.0),
            (0.2, 0.346, 0.0),
            (0.2, 0.115, 0.327),
        ] {
            mesh.vertices.push(Point3::new(x, y, z));
        }
        // Tet B shifted +x by 2.0 (well separated from A in our bounds).
        for (x, y, z) in [
            (2.0, 0.0, 0.0),
            (2.4, 0.0, 0.0),
            (2.2, 0.346, 0.0),
            (2.2, 0.115, 0.327),
        ] {
            mesh.vertices.push(Point3::new(x, y, z));
        }
        // Tet A faces (CCW outward).
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh.faces.push([2, 0, 3]);
        // Tet B faces (same winding, shifted indices).
        mesh.faces.push([4, 6, 5]);
        mesh.faces.push([4, 5, 7]);
        mesh.faces.push([5, 6, 7]);
        mesh.faces.push([6, 4, 7]);
        mesh
    }

    #[test]
    fn flood_fill_inside_components_two_for_disjoint_bodies() {
        let mesh = two_disjoint_tetrahedra();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        let (_sign, report) = FloodFillSign::build(
            &distance,
            aabb(Point3::new(-0.5, -0.5, -0.5), Point3::new(3.0, 1.0, 1.0)),
            0.05,
            0.75,
        )
        .expect("flood build");

        assert_eq!(
            report.inside_components, 2,
            "two disjoint tetrahedra must yield two Inside components"
        );
    }
}
