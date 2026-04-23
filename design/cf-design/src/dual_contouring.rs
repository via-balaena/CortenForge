//! Manifold dual contouring mesher with QEF vertex placement.
//!
//! Extracts a triangle mesh from an implicit surface field at the zero
//! isosurface. Unlike marching cubes ([`crate::mesher`]), dual contouring
//! places one vertex per sign-changing cell via Quadric Error Function (QEF)
//! minimization, producing meshes that preserve sharp features (edges,
//! corners) of the implicit surface.
//!
//! Grid cells that are entirely inside or entirely outside (determined via
//! [`FieldNode::evaluate_interval`]) are skipped, typically pruning 80%+ of
//! cells for simple primitives.

use std::collections::HashMap;

use cf_geometry::{Aabb, IndexedMesh};
use nalgebra::{Matrix3, Point3, Vector3};

use crate::field_node::FieldNode;
use crate::mesher::MeshStats;

/// Extract an isosurface mesh from a field node using dual contouring.
///
/// `cell_size` controls the voxel resolution (smaller = finer mesh).
/// Interval arithmetic prunes cells that are entirely inside or outside
/// the surface.
///
/// Each sign-changing cell gets one vertex placed via QEF minimization
/// using the field gradient at edge-crossing points. Faces are generated
/// from sign-changing grid edges shared by 4 cells, producing quads that
/// are split into triangles with CCW winding (outward-facing normals).
// Index/count conversion bounded by domain.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]
pub fn mesh_field_dc(node: &FieldNode, bounds: &Aabb, cell_size: f64) -> (IndexedMesh, MeshStats) {
    let size = bounds.size();
    let nx = ((size.x / cell_size).ceil() as usize).max(1);
    let ny = ((size.y / cell_size).ceil() as usize).max(1);
    let nz = ((size.z / cell_size).ceil() as usize).max(1);

    let origin = bounds.min;

    let mut mesh = IndexedMesh::new();
    let mut corner_cache: HashMap<(usize, usize, usize), f64> = HashMap::new();
    let mut cell_vertices: HashMap<(usize, usize, usize), u32> = HashMap::new();

    let mut cells_total = 0_usize;
    let mut cells_pruned = 0_usize;

    // ── Phase 1: Cell processing ─────────────────────────────────────────
    // For each cell, evaluate corners, detect sign changes, and place a
    // vertex via QEF minimization of hermite data (crossing point + gradient).

    for cz in 0..nz {
        for cy in 0..ny {
            for cx in 0..nx {
                cells_total += 1;

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

                // Interval pruning: skip cells entirely inside or outside
                let (lo, hi) = node.evaluate_interval(&cell_aabb);
                if lo > 0.0 || hi < 0.0 {
                    cells_pruned += 1;
                    continue;
                }

                // Evaluate 8 corners (with caching across shared faces)
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

                // Check for sign change among corners
                let has_inside = values.iter().any(|&v| v < 0.0);
                let has_outside = values.iter().any(|&v| v >= 0.0);
                if !has_inside || !has_outside {
                    continue;
                }

                // Collect hermite data from sign-changing edges:
                // (edge-crossing position, surface normal at crossing)
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
                    continue;
                }

                // Solve QEF to place the optimal vertex for this cell
                let vertex = solve_qef(&hermite_points, &hermite_normals, &cell_aabb);
                let idx = mesh.vertices.len() as u32;
                mesh.vertices.push(vertex);
                cell_vertices.insert((cx, cy, cz), idx);
            }
        }
    }

    // ── Phase 2: Face generation ─────────────────────────────────────────
    // For each internal grid edge with a sign change, the 4 surrounding
    // cells contribute one vertex each, forming a quad (split into 2
    // triangles). Winding is determined by the field sign direction.

    // X-axis edges: from grid point (gx, gy, gz) to (gx+1, gy, gz)
    // 4 cells giving +x normal: (gx,gy,gz), (gx,gy-1,gz), (gx,gy-1,gz-1), (gx,gy,gz-1)
    for gz in 1..nz {
        for gy in 1..ny {
            for gx in 0..nx {
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

    // Y-axis edges: from grid point (gx, gy, gz) to (gx, gy+1, gz)
    // 4 cells giving +y normal: (gx,gy,gz-1), (gx-1,gy,gz-1), (gx-1,gy,gz), (gx,gy,gz)
    for gz in 1..nz {
        for gy in 0..ny {
            for gx in 1..nx {
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

    // Z-axis edges: from grid point (gx, gy, gz) to (gx, gy, gz+1)
    // 4 cells giving +z normal: (gx,gy,gz), (gx-1,gy,gz), (gx-1,gy-1,gz), (gx,gy-1,gz)
    for gz in 0..nz {
        for gy in 1..ny {
            for gx in 1..nx {
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

    (
        mesh,
        MeshStats {
            cells_total,
            cells_pruned,
        },
    )
}

/// Check if a grid edge has a sign change.
///
/// Returns `Some(true)` if the first point is inside (v < 0) and the second
/// is outside, `Some(false)` for the reverse, or `None` if no sign change
/// or either endpoint is not in the cache.
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

/// Emit a quad (2 triangles) from the 4 cell vertices surrounding a
/// sign-changing grid edge.
///
/// `cells` are in the order that gives a positive-axis normal when
/// `lo_inside` is `true` (lower grid point inside, upper outside).
/// When `lo_inside` is `false`, the winding is reversed.
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
        // Normal in positive axis direction (outward)
        mesh.faces.push([q0, q1, q2]);
        mesh.faces.push([q0, q2, q3]);
    } else {
        // Normal in negative axis direction (reversed winding)
        mesh.faces.push([q0, q2, q1]);
        mesh.faces.push([q0, q3, q2]);
    }
}

/// Find the zero-crossing point along an edge via linear interpolation.
fn interpolate_edge(p0: Point3<f64>, p1: Point3<f64>, v0: f64, v1: f64) -> Point3<f64> {
    let denom = v1 - v0;
    if denom.abs() < f64::EPSILON {
        return Point3::from((p0.coords + p1.coords) * 0.5);
    }
    let t = (-v0 / denom).clamp(0.0, 1.0);
    Point3::from(p0.coords * (1.0 - t) + p1.coords * t)
}

/// Solve the Quadric Error Function to find the optimal vertex position.
///
/// Minimizes `Σ (nᵢ · (v - pᵢ))²` where `(pᵢ, nᵢ)` are hermite data
/// (edge-crossing points + surface normals). Uses SVD of the 3×3 normal
/// equation matrix `AᵀA` for stability.
///
/// The solution is biased toward the mass point (centroid of crossing
/// points) to handle underdetermined systems, and clamped to the cell
/// AABB to prevent vertices at infinity.
// Precision loss acceptable for approximate / visualization values.
#[allow(clippy::cast_precision_loss)]
fn solve_qef(points: &[Point3<f64>], normals: &[Vector3<f64>], cell_aabb: &Aabb) -> Point3<f64> {
    if points.is_empty() {
        return cell_aabb.center();
    }

    // Mass point (centroid of edge-crossing points) — the natural bias
    // for underdetermined systems. Shifting coordinates here means the
    // SVD minimum-norm solution defaults to the mass point.
    let n = points.len() as f64;
    let mass_point = points
        .iter()
        .fold(Vector3::zeros(), |acc, p| acc + p.coords)
        / n;

    // Build normal equations: A^T A and A^T b
    // where A has rows n_i^T, b has elements n_i · (p_i - mass_point)
    let mut ata = Matrix3::<f64>::zeros();
    let mut atb = Vector3::<f64>::zeros();

    for (p, ni) in points.iter().zip(normals.iter()) {
        let d = ni.dot(&(p.coords - mass_point));
        ata += ni * ni.transpose();
        atb += *ni * d;
    }

    // SVD of the 3×3 A^T A matrix with singular value thresholding
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

    // Clamp to cell AABB to prevent vertices at infinity
    Point3::new(
        vertex.x.clamp(cell_aabb.min.x, cell_aabb.max.x),
        vertex.y.clamp(cell_aabb.min.y, cell_aabb.max.y),
        vertex.z.clamp(cell_aabb.min.z, cell_aabb.max.z),
    )
}

// ── Grid geometry constants ──────────────────────────────────────────────

/// Corner offsets within a cell: maps corner index to (dx, dy, dz) offset
/// from the cell's minimum grid coordinate.
const CORNER_OFFSETS: [(usize, usize, usize); 8] = [
    (0, 0, 0), // corner 0
    (1, 0, 0), // corner 1
    (1, 1, 0), // corner 2
    (0, 1, 0), // corner 3
    (0, 0, 1), // corner 4
    (1, 0, 1), // corner 5
    (1, 1, 1), // corner 6
    (0, 1, 1), // corner 7
];

/// Edge-to-corner mapping: each cell edge connects two corners.
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

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field_node::Val;
    use std::f64::consts::PI;

    /// Validate mesh topology: check watertight + manifold.
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

    #[test]
    fn dc_sphere_valid() {
        let node = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let bounds = node.bounds().map(|b| b.expanded(0.5));
        let (mesh, _) = mesh_field_dc(&node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        assert_mesh_valid(&mesh, "dc_sphere");
    }

    #[test]
    fn dc_sphere_volume() {
        let node = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let bounds = node.bounds().map(|b| b.expanded(0.5));
        let (mesh, _) = mesh_field_dc(&node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        let expected = 4.0 / 3.0 * PI * 125.0;
        let actual = mesh.volume();
        let error = (actual - expected).abs() / expected;
        assert!(
            error < 0.15,
            "DC sphere volume error {:.1}% exceeds 15% (expected {expected:.1}, got {actual:.1})",
            error * 100.0
        );
    }

    #[test]
    fn dc_cuboid_valid() {
        let node = FieldNode::Cuboid {
            half_extents: Vector3::new(2.0, 3.0, 4.0),
        };
        let bounds = node.bounds().map(|b| b.expanded(0.5));
        let (mesh, _) = mesh_field_dc(&node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        assert_mesh_valid(&mesh, "dc_cuboid");
    }

    #[test]
    fn dc_cuboid_sharp_corners() {
        let half = Vector3::new(2.0, 2.0, 2.0);
        let cell = 0.5;
        let node = FieldNode::Cuboid { half_extents: half };
        let bounds = node.bounds().map(|b| b.expanded(cell));
        let aabb = bounds.unwrap_or(Aabb::empty());

        let (dc_mesh, _) = mesh_field_dc(&node, &aabb, cell);
        let (mc_mesh, _) = crate::mesher::mesh_field(&node, &aabb, cell);

        // True corners of the cuboid
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

        let dc_max_dev = max_corner_deviation(&dc_mesh, &true_corners);
        let mc_max_dev = max_corner_deviation(&mc_mesh, &true_corners);

        assert!(
            dc_max_dev < mc_max_dev,
            "DC should preserve corners better than MC: DC dev = {dc_max_dev:.4}, MC dev = {mc_max_dev:.4}"
        );
    }

    /// Maximum distance from any true corner to its closest mesh vertex.
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
    fn dc_cuboid_flat_faces() {
        let half = Vector3::new(3.0, 3.0, 3.0);
        let cell = 0.5;
        let node = FieldNode::Cuboid { half_extents: half };
        let bounds = node.bounds().map(|b| b.expanded(cell));
        let (mesh, _) = mesh_field_dc(&node, &bounds.unwrap_or(Aabb::empty()), cell);

        // Check +x face: vertices near x=3 (and away from edges) should
        // have x very close to 3.0.
        let face_verts: Vec<&Point3<f64>> = mesh
            .vertices
            .iter()
            .filter(|v| (v.x - 3.0).abs() < cell && v.y.abs() < 2.0 && v.z.abs() < 2.0)
            .collect();

        assert!(
            !face_verts.is_empty(),
            "should have DC vertices near the +x face"
        );
        for v in &face_verts {
            assert!(
                (v.x - 3.0).abs() < cell * 0.1,
                "DC vertex on +x face should be coplanar: x = {:.4}, expected 3.0",
                v.x
            );
        }
    }

    #[test]
    fn dc_pin_hole_valid() {
        let cuboid = FieldNode::Cuboid {
            half_extents: Vector3::new(3.0, 3.0, 3.0),
        };
        let cylinder = FieldNode::Cylinder {
            radius: 1.0,
            half_height: 4.0,
        };
        let node = FieldNode::Subtract(Box::new(cuboid), Box::new(cylinder));
        let cell = 0.4;
        let bounds = node.bounds().map(|b| b.expanded(cell));
        let (mesh, _) = mesh_field_dc(&node, &bounds.unwrap_or(Aabb::empty()), cell);
        assert_mesh_valid(&mesh, "dc_pin_hole");
    }

    #[test]
    // Precision loss acceptable for approximate / visualization values.
    #[allow(clippy::cast_precision_loss)]
    fn dc_sphere_regression() {
        let node = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let cell = 0.5;
        let bounds = node.bounds().map(|b| b.expanded(cell));
        let aabb = bounds.unwrap_or(Aabb::empty());

        let (dc_mesh, _) = mesh_field_dc(&node, &aabb, cell);
        let (mc_mesh, _) = crate::mesher::mesh_field(&node, &aabb, cell);

        // DC vertex count should be in the same ballpark as MC
        let ratio = dc_mesh.vertex_count() as f64 / mc_mesh.vertex_count() as f64;
        assert!(
            (0.3..3.0).contains(&ratio),
            "DC vertex count ({}) should be similar to MC ({}), ratio = {ratio:.2}",
            dc_mesh.vertex_count(),
            mc_mesh.vertex_count()
        );

        // Volume accuracy should be comparable
        let expected = 4.0 / 3.0 * PI * 125.0;
        let dc_error = (dc_mesh.volume() - expected).abs() / expected;
        assert!(
            dc_error < 0.20,
            "DC sphere volume error {:.1}% is too large",
            dc_error * 100.0,
        );
    }

    // ── Composed shape tests (V1 hardening) ─────────────────────────

    #[test]
    fn dc_smooth_union_manifold() {
        let a = FieldNode::Sphere {
            radius: Val::from(3.0),
        };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere {
                radius: Val::from(3.0),
            }),
            Vector3::new(4.0, 0.0, 0.0),
        );
        let node = FieldNode::SmoothUnion(Box::new(a), Box::new(b), Val::from(2.0));
        let cell = 0.4;
        let bounds = node.bounds().map(|b| b.expanded(cell));
        let (mesh, _) = mesh_field_dc(&node, &bounds.unwrap_or(Aabb::empty()), cell);
        assert_mesh_valid(&mesh, "dc_smooth_union");
        // Volume should be more than one sphere, less than two
        let one_sphere = 4.0 / 3.0 * PI * 27.0;
        assert!(
            mesh.volume() > one_sphere,
            "smooth union volume should exceed single sphere"
        );
        assert!(
            mesh.volume() < 2.0 * one_sphere,
            "smooth union volume should be less than two spheres"
        );
    }

    #[test]
    fn dc_helix_manifold() {
        let node = FieldNode::Helix {
            radius: 3.0,
            pitch: 4.0,
            thickness: 0.8,
            turns: 3.0,
        };
        let cell = 0.4;
        let bounds = node.bounds().map(|b| b.expanded(cell));
        let (mesh, _) = mesh_field_dc(&node, &bounds.unwrap_or(Aabb::empty()), cell);
        assert_mesh_valid(&mesh, "dc_helix");
    }

    #[test]
    fn dc_superellipsoid_manifold() {
        let node = FieldNode::Superellipsoid {
            radii: Vector3::new(3.0, 4.0, 2.0),
            n1: 0.5,
            n2: 0.5,
        };
        let cell = 0.4;
        let bounds = node.bounds().map(|b| b.expanded(cell));
        let (mesh, _) = mesh_field_dc(&node, &bounds.unwrap_or(Aabb::empty()), cell);
        assert_mesh_valid(&mesh, "dc_superellipsoid");
    }

    #[test]
    // Precision loss acceptable for approximate / visualization values.
    #[allow(clippy::cast_precision_loss)]
    fn dc_pruning_ratio() {
        let node = FieldNode::Sphere {
            radius: Val::from(5.0),
        };
        let bounds = node.bounds().map(|b| b.expanded(0.5));
        let (_, stats) = mesh_field_dc(&node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        let pruned_ratio = stats.cells_pruned as f64 / stats.cells_total as f64;
        assert!(
            pruned_ratio > 0.80,
            "interval pruning should eliminate >80% of cells, got {:.1}%",
            pruned_ratio * 100.0
        );
    }
}
