//! A rigid obstacle's surface, baked into the explicit solver's grids (plan
//! §15g step 6, §16u).
//!
//! Each sample's distance is exact, to the mesh's own triangles, and its sign
//! is the parity of a ray's crossings of the surface ([`ParitySign`]), so the
//! mesh must be closed; its winding does not matter, and a region it encloses
//! twice reads outside. On the product scan the flood fill's sign disagreed
//! with both others within a quarter cell of the surface, and the
//! pseudo-normals' in one region away from it. Parity agreed with the flood
//! fill away from the surface and with the pseudo-normals at it, but for a
//! handful of samples (plan §16u). Nothing is smoothed.
//!
//! Two grids, in the mesh's frame and units: a coarse one over the mesh's box
//! and a margin around it, and a fine one near the surface, stored in bricks
//! ([`FineGrid`]). A brick is baked wherever a lookup at a point within
//! [`ObstacleBake::band`] of the surface reads, so every such point reads the
//! fine grid. A point beyond the band reads the fine grid where all its
//! stencil's bricks happen to be baked, and the coarse one elsewhere.

use std::collections::HashMap;

use mesh_sdf::{ParitySign, Sign, TriMeshDistance, UnsignedDistance};
use mesh_types::IndexedMesh;
use nalgebra::Point3;
use sim_soft_explicit::executor::FineGrid;
use sim_soft_explicit::f64::{SDF_BRICK, SDF_NO_BRICK, SdfGridLayout, sdf_bricks};

/// How to bake an obstacle: its grids' spacings, how far from the surface the
/// fine grid must answer, and how far past the mesh both reach.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ObstacleBake {
    /// The coarse grid's spacing.
    pub coarse_cell: f64,
    /// The fine grid's spacing.
    pub fine_cell: f64,
    /// Every point within this distance of the surface reads the fine grid; no
    /// more than the margin, since the grids end there.
    pub band: f64,
    /// How far both grids reach past the mesh's box, on every side.
    pub margin: f64,
}

/// An obstacle's grids, in the mesh's frame and units: the coarse grid, and
/// the fine grid near the surface. Negative inside.
#[derive(Clone, Debug)]
pub struct BakedObstacle {
    /// The coarse grid's layout.
    pub grid: SdfGridLayout,
    /// The coarse grid's values, in [`SdfGridLayout`]'s order.
    pub values: Vec<f64>,
    /// The fine grid.
    pub fine: FineGrid,
}

/// Why a mesh could not be baked.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ObstacleBakeError {
    /// A spacing, the band or the margin is not positive and finite, or the
    /// band is wider than the margin.
    Bake,
    /// The mesh has no faces, more vertices than a face can name, a face that
    /// names a vertex it does not have, a vertex that is not finite, or no
    /// triangle with area.
    Mesh,
    /// The mesh, its coincident vertices welded, is not closed: these edges
    /// are not each shared by exactly two faces.
    Open {
        /// Edges with a face count other than two.
        edges: usize,
    },
    /// A grid would hold more samples than a `u32` can count.
    TooLarge,
}

impl std::fmt::Display for ObstacleBakeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Bake => write!(
                f,
                "the bake's spacings, band and margin must be positive and finite, and the band no wider than \
                 the margin"
            ),
            Self::Mesh => write!(
                f,
                "the mesh has no faces, more vertices than a face can name, a face naming a missing vertex, a \
                 vertex that is not finite, or no triangle with area"
            ),
            Self::Open { edges } => write!(
                f,
                "the mesh is not closed: {edges} edges are not each shared by exactly two faces"
            ),
            Self::TooLarge => write!(f, "a grid would hold more samples than a u32 can count"),
        }
    }
}

impl std::error::Error for ObstacleBakeError {}

/// A closed surface's exact signed distance: the distance to its triangles,
/// negative inside by the parity of a ray's crossings (plan §16u).
///
/// What the bake samples, and what else must read the same surface: the
/// lowering's start, which keeps a wall clear of the scan (plan §16w).
pub struct SignedDistance {
    distance: TriMeshDistance,
    sign: ParitySign,
    low: Point3<f64>,
    high: Point3<f64>,
}

impl SignedDistance {
    /// The signed distance of `mesh`, its exactly coincident vertices welded.
    ///
    /// # Errors
    /// [`ObstacleBakeError::Mesh`] when the mesh is empty, broken, not finite
    /// or without area, and [`ObstacleBakeError::Open`] when it is not closed
    /// once welded.
    pub fn new(mesh: &IndexedMesh) -> Result<Self, ObstacleBakeError> {
        if mesh.faces.is_empty()
            || u32::try_from(mesh.vertices.len()).is_err()
            || mesh
                .faces
                .iter()
                .flatten()
                .any(|&v| v as usize >= mesh.vertices.len())
            || !mesh
                .vertices
                .iter()
                .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite())
        {
            return Err(ObstacleBakeError::Mesh);
        }
        let surface = welded(mesh);
        let open = open_edges(&surface);
        if open > 0 {
            return Err(ObstacleBakeError::Open { edges: open });
        }
        let (mut low, mut high) = (surface.vertices[0], surface.vertices[0]);
        for v in &surface.vertices {
            low = low.inf(v);
            high = high.sup(v);
        }
        let sign = ParitySign::new(&surface).map_err(|_| ObstacleBakeError::Mesh)?;
        let distance = TriMeshDistance::new(surface).map_err(|_| ObstacleBakeError::Mesh)?;
        Ok(Self {
            distance,
            sign,
            low,
            high,
        })
    }

    /// The signed distance at `p`: negative inside.
    #[must_use]
    pub fn signed(&self, p: Point3<f64>) -> f64 {
        let d = self.distance.distance(p);
        if self.sign.is_inside(p) { -d } else { d }
    }

    /// The unsigned distance at `p`.
    #[must_use]
    pub fn unsigned(&self, p: Point3<f64>) -> f64 {
        self.distance.distance(p)
    }
}

/// Bake `mesh`, a closed surface, into the explicit solver's grids.
///
/// # Errors
/// [`ObstacleBakeError`] when the bake's numbers are not positive and finite
/// or its band is wider than its margin, the mesh is empty, broken, not finite
/// or without area, it is not closed once welded, or a grid would be too large
/// to count.
pub fn bake_obstacle(
    mesh: &IndexedMesh,
    bake: ObstacleBake,
) -> Result<BakedObstacle, ObstacleBakeError> {
    check(bake)?;
    bake_surface(&SignedDistance::new(mesh)?, bake)
}

/// Bake a surface's signed distance into the explicit solver's grids.
///
/// # Errors
/// [`ObstacleBakeError`] when the bake's numbers are not positive and finite
/// or its band is wider than its margin, or a grid would be too large to count.
pub fn bake_surface(
    surface: &SignedDistance,
    bake: ObstacleBake,
) -> Result<BakedObstacle, ObstacleBakeError> {
    check(bake)?;
    let margin = nalgebra::Vector3::repeat(bake.margin);
    let (low, high) = (surface.low - margin, surface.high + margin);
    let grid = layout(low, high, bake.coarse_cell, true)?;
    let values = at_each(samples(grid), |n| {
        surface.signed(position(grid, sample_of(grid, n)))
    });
    let fine = fine_bricks(surface, low, high, bake)?;
    Ok(BakedObstacle { grid, values, fine })
}

/// Whether the bake's numbers are positive and finite, and its band no wider than its margin.
fn check(bake: ObstacleBake) -> Result<(), ObstacleBakeError> {
    let numbers = [bake.coarse_cell, bake.fine_cell, bake.band, bake.margin];
    if numbers.iter().all(|x| x.is_finite() && *x > 0.0) && bake.band <= bake.margin {
        Ok(())
    } else {
        Err(ObstacleBakeError::Bake)
    }
}

/// The fine grid over `[low, high]`: the bricks a lookup within the band of the surface reads.
fn fine_bricks(
    surface: &SignedDistance,
    low: Point3<f64>,
    high: Point3<f64>,
    bake: ObstacleBake,
) -> Result<FineGrid, ObstacleBakeError> {
    // A brick is needed when a point within the band reads one of its samples. A lookup reads the samples of its
    // cell and one either side, all within 2√3 cells of its point, so such a sample is within the band and 2√3
    // cells of the surface. Candidates first, by their centres: a brick's samples lie within 3.5√3 cells of its
    // centre, so a brick whose centre is further than the band and 5.5√3 cells has none that near. Then each
    // candidate is kept only if one of its samples is that near.
    let fine_grid = layout(low, high, bake.fine_cell, false)?;
    let brick_counts = [
        sdf_bricks(fine_grid.size_x),
        sdf_bricks(fine_grid.size_y),
        sdf_bricks(fine_grid.size_z),
    ];
    let bricks = brick_counts.iter().map(|&n| n as usize).product::<usize>();
    let root3 = 3.0_f64.sqrt();
    let near = bake.band + 2.0 * root3 * bake.fine_cell;
    let candidate = near + 3.5 * root3 * bake.fine_cell;
    let half = f64::from(SDF_BRICK - 1) / 2.0;
    let centre_distances = at_each(bricks, |n| {
        let brick = brick_of(brick_counts, n);
        let centre = brick.map(|b| f64::from(b * SDF_BRICK) + half);
        surface.unsigned(Point3::new(
            fine_grid.origin_x + centre[0] * bake.fine_cell,
            fine_grid.origin_y + centre[1] * bake.fine_cell,
            fine_grid.origin_z + centre[2] * bake.fine_cell,
        ))
    });
    let candidates: Vec<(usize, [u32; 3])> = centre_distances
        .iter()
        .enumerate()
        .filter(|&(_, &d)| d <= candidate)
        .map(|(n, _)| (n, brick_of(brick_counts, n)))
        .collect();
    let per_brick = (SDF_BRICK * SDF_BRICK * SDF_BRICK) as usize;
    // Each candidate's samples, kept only if one is near: a dropped brick's are let go as they are made.
    let kept = at_each(candidates.len(), |c| {
        let brick = candidates[c].1;
        let values: Vec<f64> = (0..SDF_BRICK * SDF_BRICK * SDF_BRICK)
            .map(|within| {
                let sample = [
                    brick[0] * SDF_BRICK + within % SDF_BRICK,
                    brick[1] * SDF_BRICK + within / SDF_BRICK % SDF_BRICK,
                    brick[2] * SDF_BRICK + within / (SDF_BRICK * SDF_BRICK),
                ];
                surface.signed(position(fine_grid, sample))
            })
            .collect();
        values.iter().any(|v| v.abs() <= near).then_some(values)
    });
    let mut map = vec![SDF_NO_BRICK; bricks];
    let mut fine_values = Vec::new();
    for ((n, _), values) in candidates.iter().zip(kept) {
        if let Some(values) = values {
            map[*n] = u32::try_from(fine_values.len() / per_brick)
                .map_err(|_| ObstacleBakeError::TooLarge)?;
            fine_values.extend_from_slice(&values);
        }
    }
    if u32::try_from(fine_values.len()).is_err() {
        return Err(ObstacleBakeError::TooLarge);
    }
    Ok(FineGrid {
        grid: fine_grid,
        map,
        values: fine_values,
    })
}

/// `mesh` with its coincident vertices merged: those at exactly the same
/// position, as a triangle soup repeats them.
fn welded(mesh: &IndexedMesh) -> IndexedMesh {
    let mut index: HashMap<[u64; 3], u32> = HashMap::new();
    let mut surface = IndexedMesh::new();
    // Adding 0.0 turns −0.0 into 0.0, so the two land on one key.
    let key = |p: &Point3<f64>| {
        [
            (p.x + 0.0).to_bits(),
            (p.y + 0.0).to_bits(),
            (p.z + 0.0).to_bits(),
        ]
    };
    let renumbered: Vec<u32> = mesh
        .vertices
        .iter()
        .map(|p| {
            *index.entry(key(p)).or_insert_with(|| {
                surface.vertices.push(*p);
                // `SignedDistance::new` refuses a mesh with more vertices than a u32 counts.
                #[allow(clippy::cast_possible_truncation)]
                let index = (surface.vertices.len() - 1) as u32;
                index
            })
        })
        .collect();
    surface.faces = mesh
        .faces
        .iter()
        .map(|face| face.map(|v| renumbered[v as usize]))
        .collect();
    surface
}

/// How many of `surface`'s edges are not each shared by exactly two faces.
fn open_edges(surface: &IndexedMesh) -> usize {
    let mut faces_at: HashMap<(u32, u32), u32> = HashMap::new();
    for face in &surface.faces {
        for (a, b) in [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])] {
            *faces_at.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    faces_at.values().filter(|&&faces| faces != 2).count()
}

/// The layout of a grid of spacing `cell` whose samples span `[low, high]`. A
/// dense grid's samples must be counted in a u32; a bricked lattice stores only
/// its bricks, so only its bricks must be.
fn layout(
    low: Point3<f64>,
    high: Point3<f64>,
    cell: f64,
    dense: bool,
) -> Result<SdfGridLayout, ObstacleBakeError> {
    let side = |extent: f64| {
        let samples = (extent / cell).ceil() + 1.0;
        if samples < f64::from(u32::MAX - SDF_BRICK) {
            // A whole number of samples under u32::MAX.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            Ok(samples as u32)
        } else {
            Err(ObstacleBakeError::TooLarge)
        }
    };
    let grid = SdfGridLayout {
        origin_x: low.x,
        origin_y: low.y,
        origin_z: low.z,
        cell_size: cell,
        size_x: side(high.x - low.x)?,
        size_y: side(high.y - low.y)?,
        size_z: side(high.z - low.z)?,
    };
    // Three u32s multiply past a u64, not a u128.
    let count = |n: u32| {
        if dense {
            u128::from(n)
        } else {
            u128::from(sdf_bricks(n))
        }
    };
    if count(grid.size_x) * count(grid.size_y) * count(grid.size_z) > u128::from(u32::MAX) {
        return Err(ObstacleBakeError::TooLarge);
    }
    Ok(grid)
}

/// How many samples a grid holds.
const fn samples(grid: SdfGridLayout) -> usize {
    grid.size_x as usize * grid.size_y as usize * grid.size_z as usize
}

/// The sample at storage index `n`: x fastest, then y, then z.
const fn sample_of(grid: SdfGridLayout, n: usize) -> [u32; 3] {
    let (x, y) = (grid.size_x as usize, grid.size_y as usize);
    // Each is under its axis's u32 size.
    #[allow(clippy::cast_possible_truncation)]
    [(n % x) as u32, (n / x % y) as u32, (n / (x * y)) as u32]
}

/// The brick at the brick map's index `n`: x fastest, then y, then z.
const fn brick_of(counts: [u32; 3], n: usize) -> [u32; 3] {
    let (x, y) = (counts[0] as usize, counts[1] as usize);
    // Each is under its axis's u32 count.
    #[allow(clippy::cast_possible_truncation)]
    [(n % x) as u32, (n / x % y) as u32, (n / (x * y)) as u32]
}

/// A sample's position.
fn position(grid: SdfGridLayout, sample: [u32; 3]) -> Point3<f64> {
    Point3::new(
        grid.origin_x + f64::from(sample[0]) * grid.cell_size,
        grid.origin_y + f64::from(sample[1]) * grid.cell_size,
        grid.origin_z + f64::from(sample[2]) * grid.cell_size,
    )
}

/// `f` of `0..count`, on every core where there are threads to give (native);
/// in order on wasm32, which has none.
pub(crate) fn at_each<T: Send>(count: usize, f: impl Fn(usize) -> T + Send + Sync) -> Vec<T> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        use rayon::prelude::*;
        (0..count).into_par_iter().map(f).collect()
    }
    #[cfg(target_arch = "wasm32")]
    {
        (0..count).map(f).collect()
    }
}
