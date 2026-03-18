//! Adaptive dual contouring mesher using octree subdivision.
//!
//! Combines an octree with the QEF vertex placement from dual contouring.
//! The octree prunes whole interior/exterior subtrees hierarchically instead
//! of testing each cell individually — giving 10x+ cell reduction vs the
//! uniform grid while preserving sharp features.
//!
//! All surface-crossing leaves end up at the finest octree depth (aligned to
//! `cell_size`), so standard DC face generation applies directly.

use std::collections::{HashMap, HashSet};

use cf_geometry::{Aabb, IndexedMesh};
use nalgebra::{Matrix3, Point3, Vector3};

use crate::field_node::FieldNode;
use crate::mesher::MeshStats;

/// Maximum octree depth to prevent infinite recursion.
const MAX_DEPTH: u32 = 20;

/// Statistics from adaptive meshing.
#[allow(dead_code)]
pub struct AdaptiveStats {
    pub mesh_stats: MeshStats,
    pub max_depth: u32,
    pub leaf_count: usize,
    pub surface_leaf_count: usize,
    /// Number of interval evaluations performed by the octree.
    /// Comparable to `cells_total` in uniform DC (one eval per cell).
    pub interval_evals: usize,
}

/// Extract an isosurface mesh using adaptive octree dual contouring.
///
/// Builds an aligned octree where surface-crossing leaves are on a regular
/// grid at `cell_size` resolution. Interior/exterior regions are pruned
/// hierarchically by the octree, avoiding per-cell enumeration.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]
pub fn mesh_field_adaptive(
    node: &FieldNode,
    bounds: &Aabb,
    cell_size: f64,
) -> (IndexedMesh, AdaptiveStats) {
    // ── Align the root AABB to a power-of-2 grid ─────────────────────
    let size = bounds.size();
    let max_extent = size.x.max(size.y).max(size.z);
    let depth_needed = (max_extent / cell_size).log2().ceil() as u32;
    let target_depth = depth_needed.min(MAX_DEPTH);
    let root_size = cell_size * f64::from(1_u32 << target_depth);

    let center = bounds.center();
    let half = root_size * 0.5;
    let root_aabb = Aabb::new(
        Point3::new(center.x - half, center.y - half, center.z - half),
        Point3::new(center.x + half, center.y + half, center.z + half),
    );
    let origin = root_aabb.min;

    // ── Build octree and collect surface-crossing leaves ──────────────
    let mut surface_cells: HashSet<(usize, usize, usize)> = HashSet::new();
    let mut total_leaves = 0_usize;
    let mut pruned_leaves = 0_usize;
    let mut max_depth_seen = 0_u32;
    let mut interval_evals = 0_usize;

    build_octree_recursive(
        node,
        &root_aabb,
        0,
        target_depth,
        cell_size,
        origin,
        &mut surface_cells,
        &mut total_leaves,
        &mut pruned_leaves,
        &mut max_depth_seen,
        &mut interval_evals,
    );

    let surface_count = surface_cells.len();

    // ── Phase 1: Cell processing ─────────────────────────────────────
    let mut mesh = IndexedMesh::new();
    let mut corner_cache: HashMap<(usize, usize, usize), f64> = HashMap::new();
    let mut cell_vertices: HashMap<(usize, usize, usize), u32> = HashMap::new();

    for &(cx, cy, cz) in &surface_cells {
        let cell_min = Point3::new(
            (cx as f64).mul_add(cell_size, origin.x),
            (cy as f64).mul_add(cell_size, origin.y),
            (cz as f64).mul_add(cell_size, origin.z),
        );
        let cell_max = Point3::new(
            ((cx + 1) as f64).mul_add(cell_size, origin.x),
            ((cy + 1) as f64).mul_add(cell_size, origin.y),
            ((cz + 1) as f64).mul_add(cell_size, origin.z),
        );
        let cell_aabb = Aabb::new(cell_min, cell_max);

        let mut values = [0.0_f64; 8];
        let mut corners = [Point3::origin(); 8];
        for (i, &(dx, dy, dz)) in CORNER_OFFSETS.iter().enumerate() {
            let gx = cx + dx;
            let gy = cy + dy;
            let gz = cz + dz;
            let p = Point3::new(
                (gx as f64).mul_add(cell_size, origin.x),
                (gy as f64).mul_add(cell_size, origin.y),
                (gz as f64).mul_add(cell_size, origin.z),
            );
            corners[i] = p;
            values[i] = *corner_cache
                .entry((gx, gy, gz))
                .or_insert_with(|| node.evaluate(&p));
        }

        let has_inside = values.iter().any(|&v| v < 0.0);
        let has_outside = values.iter().any(|&v| v >= 0.0);
        if !has_inside || !has_outside {
            continue;
        }

        let mut hermite_points = Vec::new();
        let mut hermite_normals = Vec::new();

        for &(c0, c1) in &EDGE_CORNERS {
            if (values[c0] < 0.0) != (values[c1] < 0.0) {
                let crossing = interpolate_edge(corners[c0], corners[c1], values[c0], values[c1]);
                let normal = node.gradient(&crossing);
                hermite_points.push(crossing);
                hermite_normals.push(normal);
            }
        }

        if hermite_points.is_empty() {
            continue;
        }

        let vertex = solve_qef(&hermite_points, &hermite_normals, &cell_aabb);
        let idx = mesh.vertices.len() as u32;
        mesh.vertices.push(vertex);
        cell_vertices.insert((cx, cy, cz), idx);
    }

    // ── Phase 2: Face generation ─────────────────────────────────────
    let (nx, ny, nz) = grid_extent(&surface_cells);

    // X-axis edges
    for gz in 1..=nz {
        for gy in 1..=ny {
            for gx in 0..=nx {
                if let Some(lo_inside) =
                    edge_sign_change(&corner_cache, (gx, gy, gz), (gx + 1, gy, gz))
                {
                    let cells = [
                        (gx, gy, gz),
                        (gx, gy - 1, gz),
                        (gx, gy - 1, gz - 1),
                        (gx, gy, gz - 1),
                    ];
                    emit_quad(&cell_vertices, &mut mesh, &cells, lo_inside);
                }
            }
        }
    }

    // Y-axis edges
    for gz in 1..=nz {
        for gy in 0..=ny {
            for gx in 1..=nx {
                if let Some(lo_inside) =
                    edge_sign_change(&corner_cache, (gx, gy, gz), (gx, gy + 1, gz))
                {
                    let cells = [
                        (gx, gy, gz - 1),
                        (gx - 1, gy, gz - 1),
                        (gx - 1, gy, gz),
                        (gx, gy, gz),
                    ];
                    emit_quad(&cell_vertices, &mut mesh, &cells, lo_inside);
                }
            }
        }
    }

    // Z-axis edges
    for gz in 0..=nz {
        for gy in 1..=ny {
            for gx in 1..=nx {
                if let Some(lo_inside) =
                    edge_sign_change(&corner_cache, (gx, gy, gz), (gx, gy, gz + 1))
                {
                    let cells = [
                        (gx, gy, gz),
                        (gx - 1, gy, gz),
                        (gx - 1, gy - 1, gz),
                        (gx, gy - 1, gz),
                    ];
                    emit_quad(&cell_vertices, &mut mesh, &cells, lo_inside);
                }
            }
        }
    }

    let stats = AdaptiveStats {
        mesh_stats: MeshStats {
            cells_total: total_leaves,
            cells_pruned: pruned_leaves,
        },
        max_depth: max_depth_seen,
        leaf_count: total_leaves,
        surface_leaf_count: surface_count,
        interval_evals,
    };

    (mesh, stats)
}

// ── Octree construction ──────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
fn build_octree_recursive(
    node: &FieldNode,
    aabb: &Aabb,
    depth: u32,
    target_depth: u32,
    cell_size: f64,
    origin: Point3<f64>,
    surface_cells: &mut HashSet<(usize, usize, usize)>,
    total_leaves: &mut usize,
    pruned_leaves: &mut usize,
    max_depth: &mut u32,
    interval_evals: &mut usize,
) {
    *interval_evals += 1;
    let (lo, hi) = node.evaluate_interval(aabb);

    // Entirely inside or outside — prune this subtree
    if lo > 0.0 || hi < 0.0 {
        let levels_remaining = target_depth - depth;
        let cells_per_axis = 1_usize << levels_remaining;
        let cells_in_subtree = cells_per_axis * cells_per_axis * cells_per_axis;
        *total_leaves += cells_in_subtree;
        *pruned_leaves += cells_in_subtree;
        return;
    }

    // At target depth — this is a fine cell on the regular grid
    if depth >= target_depth {
        *total_leaves += 1;
        *max_depth = (*max_depth).max(depth);

        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let ix = ((aabb.min.x - origin.x) / cell_size).round() as usize;
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let iy = ((aabb.min.y - origin.y) / cell_size).round() as usize;
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let iz = ((aabb.min.z - origin.z) / cell_size).round() as usize;

        surface_cells.insert((ix, iy, iz));
        return;
    }

    // Subdivide into 8 children
    let mid = aabb.center();
    let min = aabb.min;
    let max = aabb.max;

    let children = [
        Aabb::new(
            Point3::new(min.x, min.y, min.z),
            Point3::new(mid.x, mid.y, mid.z),
        ),
        Aabb::new(
            Point3::new(mid.x, min.y, min.z),
            Point3::new(max.x, mid.y, mid.z),
        ),
        Aabb::new(
            Point3::new(min.x, mid.y, min.z),
            Point3::new(mid.x, max.y, mid.z),
        ),
        Aabb::new(
            Point3::new(mid.x, mid.y, min.z),
            Point3::new(max.x, max.y, mid.z),
        ),
        Aabb::new(
            Point3::new(min.x, min.y, mid.z),
            Point3::new(mid.x, mid.y, max.z),
        ),
        Aabb::new(
            Point3::new(mid.x, min.y, mid.z),
            Point3::new(max.x, mid.y, max.z),
        ),
        Aabb::new(
            Point3::new(min.x, mid.y, mid.z),
            Point3::new(mid.x, max.y, max.z),
        ),
        Aabb::new(
            Point3::new(mid.x, mid.y, mid.z),
            Point3::new(max.x, max.y, max.z),
        ),
    ];

    for child in &children {
        build_octree_recursive(
            node,
            child,
            depth + 1,
            target_depth,
            cell_size,
            origin,
            surface_cells,
            total_leaves,
            pruned_leaves,
            max_depth,
            interval_evals,
        );
    }
}

// ── DC helpers (self-contained) ──────────────────────────────────────────

fn grid_extent(surface_cells: &HashSet<(usize, usize, usize)>) -> (usize, usize, usize) {
    let mut nx = 0_usize;
    let mut ny = 0_usize;
    let mut nz = 0_usize;
    for &(cx, cy, cz) in surface_cells {
        nx = nx.max(cx + 1);
        ny = ny.max(cy + 1);
        nz = nz.max(cz + 1);
    }
    (nx, ny, nz)
}

fn edge_sign_change(
    cache: &HashMap<(usize, usize, usize), f64>,
    p0: (usize, usize, usize),
    p1: (usize, usize, usize),
) -> Option<bool> {
    let v0 = *cache.get(&p0)?;
    let v1 = *cache.get(&p1)?;
    if (v0 < 0.0) == (v1 < 0.0) {
        return None;
    }
    Some(v0 < 0.0)
}

fn emit_quad(
    cell_vertices: &HashMap<(usize, usize, usize), u32>,
    mesh: &mut IndexedMesh,
    cells: &[(usize, usize, usize); 4],
    lo_inside: bool,
) {
    let Some(&q0) = cell_vertices.get(&cells[0]) else {
        return;
    };
    let Some(&q1) = cell_vertices.get(&cells[1]) else {
        return;
    };
    let Some(&q2) = cell_vertices.get(&cells[2]) else {
        return;
    };
    let Some(&q3) = cell_vertices.get(&cells[3]) else {
        return;
    };

    if lo_inside {
        mesh.faces.push([q0, q1, q2]);
        mesh.faces.push([q0, q2, q3]);
    } else {
        mesh.faces.push([q0, q2, q1]);
        mesh.faces.push([q0, q3, q2]);
    }
}

#[must_use]
fn interpolate_edge(p0: Point3<f64>, p1: Point3<f64>, v0: f64, v1: f64) -> Point3<f64> {
    let denom = v1 - v0;
    if denom.abs() < f64::EPSILON {
        return Point3::from((p0.coords + p1.coords) * 0.5);
    }
    let t = (-v0 / denom).clamp(0.0, 1.0);
    Point3::from(p0.coords * (1.0 - t) + p1.coords * t)
}

#[allow(clippy::cast_precision_loss)]
#[must_use]
fn solve_qef(points: &[Point3<f64>], normals: &[Vector3<f64>], cell_aabb: &Aabb) -> Point3<f64> {
    if points.is_empty() {
        return cell_aabb.center();
    }

    let n = points.len() as f64;
    let mass_point = points
        .iter()
        .fold(Vector3::zeros(), |acc, p| acc + p.coords)
        / n;

    let mut ata = Matrix3::<f64>::zeros();
    let mut atb = Vector3::<f64>::zeros();

    for (p, ni) in points.iter().zip(normals.iter()) {
        let d = ni.dot(&(p.coords - mass_point));
        ata += ni * ni.transpose();
        atb += *ni * d;
    }

    let svd = ata.svd(true, true);
    let max_sv = svd.singular_values.max();
    let threshold = 1e-6 * max_sv;

    let u = svd.u.unwrap_or_else(Matrix3::identity);
    let vt = svd.v_t.unwrap_or_else(Matrix3::identity);

    let mut v_prime = Vector3::zeros();
    for k in 0..3 {
        let sv = svd.singular_values[k];
        if sv > threshold {
            let coeff = u.column(k).dot(&atb) / sv;
            v_prime += coeff * Vector3::new(vt[(k, 0)], vt[(k, 1)], vt[(k, 2)]);
        }
    }

    let vertex = Point3::from(v_prime + mass_point);

    Point3::new(
        vertex.x.clamp(cell_aabb.min.x, cell_aabb.max.x),
        vertex.y.clamp(cell_aabb.min.y, cell_aabb.max.y),
        vertex.z.clamp(cell_aabb.min.z, cell_aabb.max.z),
    )
}

/// Corner offsets within a cell.
const CORNER_OFFSETS: [(usize, usize, usize); 8] = [
    (0, 0, 0),
    (1, 0, 0),
    (1, 1, 0),
    (0, 1, 0),
    (0, 0, 1),
    (1, 0, 1),
    (1, 1, 1),
    (0, 1, 1),
];

/// Edge-to-corner mapping.
const EDGE_CORNERS: [(usize, usize); 12] = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
];

// ── Parallel adaptive DC ──────────────────────────────────────────────────

/// Depth threshold below which octree construction spawns rayon tasks.
/// Above this depth, falls back to sequential to avoid task overhead.
const PAR_DEPTH_THRESHOLD: u32 = 3;

/// Parallel version of [`mesh_field_adaptive`].
///
/// Uses rayon for:
/// 1. Octree construction (parallel at top levels)
/// 2. Phase 1 cell processing (parallel corner eval + gradient)
///
/// Phase 2 (face generation) remains sequential (pure bookkeeping).
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]
pub fn mesh_field_adaptive_par(
    node: &FieldNode,
    bounds: &Aabb,
    cell_size: f64,
) -> (IndexedMesh, AdaptiveStats) {
    use rayon::prelude::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    // ── Align the root AABB to a power-of-2 grid ─────────────────────
    let size = bounds.size();
    let max_extent = size.x.max(size.y).max(size.z);
    let depth_needed = (max_extent / cell_size).log2().ceil() as u32;
    let target_depth = depth_needed.min(MAX_DEPTH);
    let root_size = cell_size * f64::from(1_u32 << target_depth);

    let center = bounds.center();
    let half = root_size * 0.5;
    let root_aabb = Aabb::new(
        Point3::new(center.x - half, center.y - half, center.z - half),
        Point3::new(center.x + half, center.y + half, center.z + half),
    );
    let origin = root_aabb.min;

    // ── Build octree in parallel and collect surface-crossing leaves ──
    let total_leaves = AtomicUsize::new(0);
    let pruned_leaves = AtomicUsize::new(0);
    let interval_evals = AtomicUsize::new(0);

    let surface_cells = build_octree_par(
        node,
        &root_aabb,
        0,
        target_depth,
        cell_size,
        origin,
        &total_leaves,
        &pruned_leaves,
        &interval_evals,
    );

    let total_leaves = total_leaves.load(Ordering::Relaxed);
    let pruned_leaves = pruned_leaves.load(Ordering::Relaxed);
    let interval_evals = interval_evals.load(Ordering::Relaxed);
    let surface_count = surface_cells.len();

    // ── Phase 1: Parallel cell processing ─────────────────────────────
    // Each cell independently evaluates its 8 corners + gradients.
    // No shared corner cache — accepts ~2x redundant evals for full parallelism.
    let cell_list: Vec<(usize, usize, usize)> = surface_cells.into_iter().collect();

    // Process cells in parallel, collecting (cell_coord, vertex, corner_values)
    let cell_results: Vec<_> = cell_list
        .par_iter()
        .filter_map(|&(cx, cy, cz)| {
            let cell_min = Point3::new(
                (cx as f64).mul_add(cell_size, origin.x),
                (cy as f64).mul_add(cell_size, origin.y),
                (cz as f64).mul_add(cell_size, origin.z),
            );
            let cell_max = Point3::new(
                ((cx + 1) as f64).mul_add(cell_size, origin.x),
                ((cy + 1) as f64).mul_add(cell_size, origin.y),
                ((cz + 1) as f64).mul_add(cell_size, origin.z),
            );
            let cell_aabb = Aabb::new(cell_min, cell_max);

            // Evaluate 8 corners (in batches of 4 using batch evaluator)
            let mut corners = [Point3::origin(); 8];
            for (i, &(dx, dy, dz)) in CORNER_OFFSETS.iter().enumerate() {
                let gx = cx + dx;
                let gy = cy + dy;
                let gz = cz + dz;
                corners[i] = Point3::new(
                    (gx as f64).mul_add(cell_size, origin.x),
                    (gy as f64).mul_add(cell_size, origin.y),
                    (gz as f64).mul_add(cell_size, origin.z),
                );
            }

            let batch0 = [corners[0], corners[1], corners[2], corners[3]];
            let batch1 = [corners[4], corners[5], corners[6], corners[7]];
            let vals0 = node.evaluate_batch(&batch0);
            let vals1 = node.evaluate_batch(&batch1);
            let values = [
                vals0[0], vals0[1], vals0[2], vals0[3], vals1[0], vals1[1], vals1[2], vals1[3],
            ];

            let has_inside = values.iter().any(|&v| v < 0.0);
            let has_outside = values.iter().any(|&v| v >= 0.0);
            if !has_inside || !has_outside {
                // Still need corner values for face generation, store them
                let corner_vals: Vec<((usize, usize, usize), f64)> = CORNER_OFFSETS
                    .iter()
                    .enumerate()
                    .map(|(i, &(dx, dy, dz))| ((cx + dx, cy + dy, cz + dz), values[i]))
                    .collect();
                return Some(((cx, cy, cz), None, corner_vals));
            }

            let mut hermite_points = Vec::new();
            let mut hermite_normals = Vec::new();

            for &(c0, c1) in &EDGE_CORNERS {
                if (values[c0] < 0.0) != (values[c1] < 0.0) {
                    let crossing =
                        interpolate_edge(corners[c0], corners[c1], values[c0], values[c1]);
                    let normal = node.gradient(&crossing);
                    hermite_points.push(crossing);
                    hermite_normals.push(normal);
                }
            }

            if hermite_points.is_empty() {
                let corner_vals: Vec<((usize, usize, usize), f64)> = CORNER_OFFSETS
                    .iter()
                    .enumerate()
                    .map(|(i, &(dx, dy, dz))| ((cx + dx, cy + dy, cz + dz), values[i]))
                    .collect();
                return Some(((cx, cy, cz), None, corner_vals));
            }

            let vertex = solve_qef(&hermite_points, &hermite_normals, &cell_aabb);
            let corner_vals: Vec<((usize, usize, usize), f64)> = CORNER_OFFSETS
                .iter()
                .enumerate()
                .map(|(i, &(dx, dy, dz))| ((cx + dx, cy + dy, cz + dz), values[i]))
                .collect();
            Some(((cx, cy, cz), Some(vertex), corner_vals))
        })
        .collect();

    // ── Merge results sequentially ────────────────────────────────────
    let mut mesh = IndexedMesh::new();
    let mut corner_cache: HashMap<(usize, usize, usize), f64> = HashMap::new();
    let mut cell_vertices: HashMap<(usize, usize, usize), u32> = HashMap::new();

    for (cell_coord, vertex_opt, corner_vals) in &cell_results {
        for &(key, val) in corner_vals {
            corner_cache.entry(key).or_insert(val);
        }
        if let Some(vertex) = vertex_opt {
            let idx = mesh.vertices.len() as u32;
            mesh.vertices.push(*vertex);
            cell_vertices.insert(*cell_coord, idx);
        }
    }

    // ── Phase 2: Face generation (sequential) ─────────────────────────
    let (nx, ny, nz) = grid_extent_from_cache(&corner_cache);

    // X-axis edges
    for gz in 1..=nz {
        for gy in 1..=ny {
            for gx in 0..=nx {
                if let Some(lo_inside) =
                    edge_sign_change(&corner_cache, (gx, gy, gz), (gx + 1, gy, gz))
                {
                    let cells = [
                        (gx, gy, gz),
                        (gx, gy - 1, gz),
                        (gx, gy - 1, gz - 1),
                        (gx, gy, gz - 1),
                    ];
                    emit_quad(&cell_vertices, &mut mesh, &cells, lo_inside);
                }
            }
        }
    }

    // Y-axis edges
    for gz in 1..=nz {
        for gy in 0..=ny {
            for gx in 1..=nx {
                if let Some(lo_inside) =
                    edge_sign_change(&corner_cache, (gx, gy, gz), (gx, gy + 1, gz))
                {
                    let cells = [
                        (gx, gy, gz - 1),
                        (gx - 1, gy, gz - 1),
                        (gx - 1, gy, gz),
                        (gx, gy, gz),
                    ];
                    emit_quad(&cell_vertices, &mut mesh, &cells, lo_inside);
                }
            }
        }
    }

    // Z-axis edges
    for gz in 0..=nz {
        for gy in 1..=ny {
            for gx in 1..=nx {
                if let Some(lo_inside) =
                    edge_sign_change(&corner_cache, (gx, gy, gz), (gx, gy, gz + 1))
                {
                    let cells = [
                        (gx, gy, gz),
                        (gx - 1, gy, gz),
                        (gx - 1, gy - 1, gz),
                        (gx, gy - 1, gz),
                    ];
                    emit_quad(&cell_vertices, &mut mesh, &cells, lo_inside);
                }
            }
        }
    }

    let stats = AdaptiveStats {
        mesh_stats: MeshStats {
            cells_total: total_leaves,
            cells_pruned: pruned_leaves,
        },
        max_depth: target_depth,
        leaf_count: total_leaves,
        surface_leaf_count: surface_count,
        interval_evals,
    };

    (mesh, stats)
}

/// Build octree in parallel at top levels, sequential below threshold.
#[allow(
    clippy::too_many_arguments,
    clippy::too_many_lines,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn build_octree_par(
    node: &FieldNode,
    aabb: &Aabb,
    depth: u32,
    target_depth: u32,
    cell_size: f64,
    origin: Point3<f64>,
    total_leaves: &std::sync::atomic::AtomicUsize,
    pruned_leaves: &std::sync::atomic::AtomicUsize,
    interval_evals: &std::sync::atomic::AtomicUsize,
) -> HashSet<(usize, usize, usize)> {
    use rayon::prelude::*;
    use std::sync::atomic::Ordering;

    interval_evals.fetch_add(1, Ordering::Relaxed);
    let (lo, hi) = node.evaluate_interval(aabb);

    if lo > 0.0 || hi < 0.0 {
        let levels_remaining = target_depth - depth;
        let cells_per_axis = 1_usize << levels_remaining;
        let cells_in_subtree = cells_per_axis * cells_per_axis * cells_per_axis;
        total_leaves.fetch_add(cells_in_subtree, Ordering::Relaxed);
        pruned_leaves.fetch_add(cells_in_subtree, Ordering::Relaxed);
        return HashSet::new();
    }

    if depth >= target_depth {
        total_leaves.fetch_add(1, Ordering::Relaxed);

        let ix = ((aabb.min.x - origin.x) / cell_size).round() as usize;
        let iy = ((aabb.min.y - origin.y) / cell_size).round() as usize;
        let iz = ((aabb.min.z - origin.z) / cell_size).round() as usize;

        let mut set = HashSet::new();
        set.insert((ix, iy, iz));
        return set;
    }

    let mid = aabb.center();
    let min = aabb.min;
    let max = aabb.max;

    let children = [
        Aabb::new(
            Point3::new(min.x, min.y, min.z),
            Point3::new(mid.x, mid.y, mid.z),
        ),
        Aabb::new(
            Point3::new(mid.x, min.y, min.z),
            Point3::new(max.x, mid.y, mid.z),
        ),
        Aabb::new(
            Point3::new(min.x, mid.y, min.z),
            Point3::new(mid.x, max.y, mid.z),
        ),
        Aabb::new(
            Point3::new(mid.x, mid.y, min.z),
            Point3::new(max.x, max.y, mid.z),
        ),
        Aabb::new(
            Point3::new(min.x, min.y, mid.z),
            Point3::new(mid.x, mid.y, max.z),
        ),
        Aabb::new(
            Point3::new(mid.x, min.y, mid.z),
            Point3::new(max.x, mid.y, max.z),
        ),
        Aabb::new(
            Point3::new(min.x, mid.y, mid.z),
            Point3::new(mid.x, max.y, max.z),
        ),
        Aabb::new(
            Point3::new(mid.x, mid.y, mid.z),
            Point3::new(max.x, max.y, max.z),
        ),
    ];

    if depth < PAR_DEPTH_THRESHOLD {
        // Parallel: spawn rayon tasks for each child
        let child_sets: Vec<HashSet<(usize, usize, usize)>> = children
            .par_iter()
            .map(|child_aabb| {
                build_octree_par(
                    node,
                    child_aabb,
                    depth + 1,
                    target_depth,
                    cell_size,
                    origin,
                    total_leaves,
                    pruned_leaves,
                    interval_evals,
                )
            })
            .collect();

        let mut merged = HashSet::new();
        for set in child_sets {
            merged.extend(set);
        }
        merged
    } else {
        // Sequential below threshold
        let mut surface_cells = HashSet::new();
        let mut tl = 0_usize;
        let mut pl = 0_usize;
        let mut md = 0_u32;
        let mut ie = 0_usize;

        for child_aabb in &children {
            build_octree_recursive(
                node,
                child_aabb,
                depth + 1,
                target_depth,
                cell_size,
                origin,
                &mut surface_cells,
                &mut tl,
                &mut pl,
                &mut md,
                &mut ie,
            );
        }

        total_leaves.fetch_add(tl, Ordering::Relaxed);
        pruned_leaves.fetch_add(pl, Ordering::Relaxed);
        interval_evals.fetch_add(ie, Ordering::Relaxed);

        surface_cells
    }
}

/// Grid extent from corner cache (for parallel version).
fn grid_extent_from_cache(cache: &HashMap<(usize, usize, usize), f64>) -> (usize, usize, usize) {
    let mut nx = 0_usize;
    let mut ny = 0_usize;
    let mut nz = 0_usize;
    for &(gx, gy, gz) in cache.keys() {
        nx = nx.max(gx);
        ny = ny.max(gy);
        nz = nz.max(gz);
    }
    (nx, ny, nz)
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn check_topology(mesh: &IndexedMesh) -> (bool, bool) {
        let mut directed: HashMap<(u32, u32), usize> = HashMap::new();
        for face in &mesh.faces {
            for i in 0..3 {
                *directed.entry((face[i], face[(i + 1) % 3])).or_insert(0) += 1;
            }
        }

        let mut boundary = 0_usize;
        let mut non_manifold = 0_usize;
        for (&(a, b), &count) in &directed {
            if count > 1 {
                non_manifold += 1;
            }
            if directed.get(&(b, a)).copied().unwrap_or(0) == 0 {
                boundary += 1;
            }
        }

        (boundary == 0, non_manifold == 0)
    }

    fn assert_mesh_valid(mesh: &IndexedMesh, label: &str) {
        assert!(!mesh.is_empty(), "{label}: mesh should not be empty");
        let (watertight, manifold) = check_topology(mesh);
        assert!(watertight, "{label}: mesh should be watertight");
        assert!(manifold, "{label}: mesh should be manifold");
        assert!(
            mesh.signed_volume() > 0.0,
            "{label}: mesh should have positive signed volume (CCW winding), got {}",
            mesh.signed_volume()
        );
    }

    fn sphere_bounds(r: f64, margin: f64) -> Aabb {
        Aabb::new(
            Point3::new(-r - margin, -r - margin, -r - margin),
            Point3::new(r + margin, r + margin, r + margin),
        )
    }

    #[test]
    fn adaptive_sphere_valid() {
        let node = FieldNode::Sphere { radius: 5.0 };
        let bounds = sphere_bounds(5.0, 0.5);
        let (mesh, stats) = mesh_field_adaptive(&node, &bounds, 0.5);

        assert_mesh_valid(&mesh, "adaptive_sphere");
        assert!(
            stats.surface_leaf_count > 0,
            "Should have surface-crossing leaves"
        );
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn adaptive_sphere_cell_reduction() {
        // Use a finer resolution where the octree advantage is clear.
        // At cell_size=0.25, the sphere surface shell is thin relative
        // to the grid, so most interior/exterior is pruned hierarchically.
        let node = FieldNode::Sphere { radius: 5.0 };
        let cell = 0.25;
        let bounds = sphere_bounds(5.0, cell);

        let (_, stats) = mesh_field_adaptive(&node, &bounds, cell);

        // Uniform DC: size=11, nx=ceil(11/0.25)=44, total = 44^3 = 85,184
        let uniform_total = 44_usize * 44 * 44;

        // The adaptive mesher only processes surface-crossing leaf cells.
        // For a sphere, the shell is ~1 cell thick, so surface cells ≈
        // 4π r² / cell² ≈ 5,000. That's 17x fewer than the uniform total.
        let ratio = uniform_total as f64 / stats.surface_leaf_count.max(1) as f64;
        assert!(
            ratio > 10.0,
            "Adaptive should process 10x+ fewer cells than uniform total ({}): got {} surface leaves, ratio {:.1}x",
            uniform_total,
            stats.surface_leaf_count,
            ratio
        );
    }

    #[test]
    fn adaptive_sphere_volume() {
        let node = FieldNode::Sphere { radius: 5.0 };
        let bounds = sphere_bounds(5.0, 0.5);
        let (mesh, _) = mesh_field_adaptive(&node, &bounds, 0.5);
        let expected = 4.0 / 3.0 * PI * 125.0;
        let actual = mesh.volume();
        let error = (actual - expected).abs() / expected;
        assert!(
            error < 0.15,
            "Adaptive sphere volume error {:.1}% exceeds 15% (expected {expected:.1}, got {actual:.1})",
            error * 100.0
        );
    }

    #[test]
    fn adaptive_cuboid_valid() {
        let node = FieldNode::Cuboid {
            half_extents: Vector3::new(2.0, 3.0, 4.0),
        };
        let bounds = Aabb::new(Point3::new(-2.5, -3.5, -4.5), Point3::new(2.5, 3.5, 4.5));
        let (mesh, _) = mesh_field_adaptive(&node, &bounds, 0.5);
        assert_mesh_valid(&mesh, "adaptive_cuboid");
    }

    #[test]
    fn adaptive_cuboid_sharp_corners() {
        let half = Vector3::new(2.0, 2.0, 2.0);
        let cell = 0.5;
        let node = FieldNode::Cuboid { half_extents: half };
        let bounds = Aabb::new(Point3::new(-2.5, -2.5, -2.5), Point3::new(2.5, 2.5, 2.5));

        let (adaptive_mesh, _) = mesh_field_adaptive(&node, &bounds, cell);
        let (mc_mesh, _) = crate::mesher::mesh_field(&node, &bounds, cell);

        let true_corners: Vec<Point3<f64>> = [
            (-1.0_f64, -1.0, -1.0),
            (-1.0, -1.0, 1.0),
            (-1.0, 1.0, -1.0),
            (-1.0, 1.0, 1.0),
            (1.0, -1.0, -1.0),
            (1.0, -1.0, 1.0),
            (1.0, 1.0, -1.0),
            (1.0, 1.0, 1.0),
        ]
        .iter()
        .map(|&(x, y, z)| Point3::new(x * 2.0, y * 2.0, z * 2.0))
        .collect();

        let adaptive_max_dev = max_corner_deviation(&adaptive_mesh, &true_corners);
        let mc_max_dev = max_corner_deviation(&mc_mesh, &true_corners);

        assert!(
            adaptive_max_dev < mc_max_dev,
            "Adaptive DC should preserve corners better than MC: adaptive dev = {adaptive_max_dev:.4}, MC dev = {mc_max_dev:.4}"
        );
    }

    fn max_corner_deviation(mesh: &IndexedMesh, true_corners: &[Point3<f64>]) -> f64 {
        true_corners
            .iter()
            .map(|tc| {
                mesh.vertices
                    .iter()
                    .map(|v| (v - tc).norm())
                    .fold(f64::MAX, f64::min)
            })
            .fold(0.0_f64, f64::max)
    }

    #[test]
    fn adaptive_topology_manifold() {
        let node = FieldNode::Sphere { radius: 5.0 };
        let bounds = sphere_bounds(5.0, 0.5);
        let (mesh, _) = mesh_field_adaptive(&node, &bounds, 0.5);
        let (watertight, manifold) = check_topology(&mesh);
        assert!(watertight, "Adaptive sphere mesh should be watertight");
        assert!(manifold, "Adaptive sphere mesh should be manifold");
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn adaptive_vs_uniform_dc_regression() {
        let node = FieldNode::Sphere { radius: 5.0 };
        let cell = 0.5;
        let bounds = sphere_bounds(5.0, cell);

        let (adaptive_mesh, _) = mesh_field_adaptive(&node, &bounds, cell);
        let (dc_mesh, _) = crate::dual_contouring::mesh_field_dc(&node, &bounds, cell);

        let expected = 4.0 / 3.0 * PI * 125.0;
        let adaptive_error = (adaptive_mesh.volume() - expected).abs() / expected;

        assert!(
            adaptive_error < 0.20,
            "Adaptive volume error {:.1}% is too large",
            adaptive_error * 100.0
        );

        let ratio = adaptive_mesh.vertex_count() as f64 / dc_mesh.vertex_count() as f64;
        assert!(
            (0.3..3.0).contains(&ratio),
            "Adaptive vertex count ({}) should be similar to DC ({}), ratio = {ratio:.2}",
            adaptive_mesh.vertex_count(),
            dc_mesh.vertex_count()
        );
    }

    #[test]
    fn adaptive_pin_hole_valid() {
        let cuboid = FieldNode::Cuboid {
            half_extents: Vector3::new(3.0, 3.0, 3.0),
        };
        let cylinder = FieldNode::Cylinder {
            radius: 1.0,
            half_height: 4.0,
        };
        let node = FieldNode::Subtract(Box::new(cuboid), Box::new(cylinder));
        let cell = 0.4;
        let bounds = Aabb::new(Point3::new(-3.4, -3.4, -4.4), Point3::new(3.4, 3.4, 4.4));
        let (mesh, _) = mesh_field_adaptive(&node, &bounds, cell);
        assert_mesh_valid(&mesh, "adaptive_pin_hole");
    }

    // ── Parallel adaptive DC tests ────────────────────────────────────

    #[test]
    fn par_sphere_valid() {
        let node = FieldNode::Sphere { radius: 5.0 };
        let bounds = sphere_bounds(5.0, 0.5);
        let (mesh, stats) = mesh_field_adaptive_par(&node, &bounds, 0.5);
        assert_mesh_valid(&mesh, "par_sphere");
        assert!(
            stats.surface_leaf_count > 0,
            "Should have surface-crossing leaves"
        );
    }

    #[test]
    fn par_sphere_volume() {
        let node = FieldNode::Sphere { radius: 5.0 };
        let bounds = sphere_bounds(5.0, 0.5);
        let (mesh, _) = mesh_field_adaptive_par(&node, &bounds, 0.5);
        let expected = 4.0 / 3.0 * PI * 125.0;
        let actual = mesh.volume();
        let error = (actual - expected).abs() / expected;
        assert!(
            error < 0.15,
            "Par sphere volume error {:.1}% exceeds 15% (expected {expected:.1}, got {actual:.1})",
            error * 100.0
        );
    }

    #[test]
    fn par_vs_seq_volume_match() {
        let node = FieldNode::Sphere { radius: 5.0 };
        let cell = 0.5;
        let bounds = sphere_bounds(5.0, cell);

        let (seq_mesh, _) = mesh_field_adaptive(&node, &bounds, cell);
        let (par_mesh, _) = mesh_field_adaptive_par(&node, &bounds, cell);

        let seq_vol = seq_mesh.volume();
        let par_vol = par_mesh.volume();
        let rel_err = (par_vol - seq_vol).abs() / seq_vol;
        assert!(
            rel_err < 0.01,
            "Par volume should be within 1% of seq: seq={seq_vol:.2}, par={par_vol:.2}, err={:.2}%",
            rel_err * 100.0
        );
    }

    #[test]
    fn par_cuboid_valid() {
        let node = FieldNode::Cuboid {
            half_extents: Vector3::new(2.0, 3.0, 4.0),
        };
        let bounds = Aabb::new(Point3::new(-2.5, -3.5, -4.5), Point3::new(2.5, 3.5, 4.5));
        let (mesh, _) = mesh_field_adaptive_par(&node, &bounds, 0.5);
        assert_mesh_valid(&mesh, "par_cuboid");
    }

    #[test]
    fn par_pin_hole_valid() {
        let cuboid = FieldNode::Cuboid {
            half_extents: Vector3::new(3.0, 3.0, 3.0),
        };
        let cylinder = FieldNode::Cylinder {
            radius: 1.0,
            half_height: 4.0,
        };
        let node = FieldNode::Subtract(Box::new(cuboid), Box::new(cylinder));
        let cell = 0.4;
        let bounds = Aabb::new(Point3::new(-3.4, -3.4, -4.4), Point3::new(3.4, 3.4, 4.4));
        let (mesh, _) = mesh_field_adaptive_par(&node, &bounds, cell);
        assert_mesh_valid(&mesh, "par_pin_hole");
    }

    #[test]
    fn par_topology_manifold() {
        let node = FieldNode::Sphere { radius: 5.0 };
        let bounds = sphere_bounds(5.0, 0.5);
        let (mesh, _) = mesh_field_adaptive_par(&node, &bounds, 0.5);
        let (watertight, manifold) = check_topology(&mesh);
        assert!(watertight, "Parallel sphere mesh should be watertight");
        assert!(manifold, "Parallel sphere mesh should be manifold");
    }

    // ── Benchmarks (ignored by default, run with --ignored) ───────────

    #[test]
    #[ignore = "benchmark — run with --ignored"]
    #[allow(clippy::similar_names)]
    fn bench_adaptive_sphere() {
        use std::time::Instant;

        let node = FieldNode::Sphere { radius: 5.0 };
        let cell = 0.1;
        let bounds = sphere_bounds(5.0, cell);

        // Warmup
        let _ = mesh_field_adaptive(&node, &bounds, cell);

        // (a) Sequential adaptive DC
        let t0 = Instant::now();
        let (seq_mesh, _) = mesh_field_adaptive(&node, &bounds, cell);
        let seq_secs = t0.elapsed().as_secs_f64();

        // (b) Parallel adaptive DC (batch + rayon)
        let t1 = Instant::now();
        let (par_mesh, _) = mesh_field_adaptive_par(&node, &bounds, cell);
        let par_secs = t1.elapsed().as_secs_f64();

        println!(
            "Sequential: {:.1}ms, verts={}",
            seq_secs * 1000.0,
            seq_mesh.vertex_count()
        );
        println!(
            "Parallel:   {:.1}ms, verts={}",
            par_secs * 1000.0,
            par_mesh.vertex_count()
        );
        if par_secs > 0.0 {
            println!("Speedup:    {:.1}x", seq_secs / par_secs);
        }
    }
}
