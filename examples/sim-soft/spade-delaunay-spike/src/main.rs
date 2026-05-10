//! THROWAWAY SPIKE — **BANKED-AS-WRONG-ARCHITECTURE** per sim-soft
//! viz arc memo §"Spike findings (2026-05-10) — Delaunay-of-centroids
//! architecture FALSIFIED" (`project_sim_soft_viz_arc.md`).
//!
//! This crate iterated through 8 fix attempts on the
//! Delaunay-of-centroids approach (vanilla → max-edge → α-shape →
//! aspect-ratio → centroid-in-body SDF → analytical boundary samples
//! → constrained Delaunay → boundary-region prune →
//! any-vertex-outside). Each fix patched one geometry-specific
//! symptom and exposed a different one. The architecture itself was
//! geometry-specific and would not have generalized to the
//! production goal (organic scanned shapes for cavity-fitting
//! layered silicone molds).
//!
//! Pivoted at 2026-05-10 to **tet-mesh-native primitives**
//! (`boundary_surface()` + `slab_cut()`) which use the existing tet
//! connectivity that sim-soft's BCC + Isosurface Stuffing already
//! produces. See the viz arc memo §"Pivoted architecture —
//! tet-mesh-native primitives" for details.
//!
//! This crate is preserved as a banked record of the failed
//! architecture, NOT as live code. Future maintainers: do not
//! revive the Delaunay approach without re-reading the viz arc memo
//! first — the spike findings document why it doesn't work.
//!
//! Per pattern (mm) at the row 21 v1.5 PARKED memo: spike-before-lock
//! discipline. The spike's negative result IS the load-bearing
//! finding; the F1 architecture is now built on a foundation that
//! the spike falsified.
//!
//! # Run
//!
//! Pre-condition: row 24's x-slab PLY must exist at
//! `examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_xslab_final.ply`.
//! Run row 24 first if missing (`cargo run -p
//! example-sim-soft-scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp
//! --release`).
//!
//! ```sh
//! cargo run -p spade-delaunay-spike --release
//! ```
//!
//! Emits `out/sleeve_xslab_triangulated.ply` (next to the spike crate)
//! with the same per-vertex scalars as the input PLY plus a face
//! list from the Delaunay triangulation, post-filter.
//!
//! Then visually inspect:
//!
//! ```sh
//! cargo run -p cf-viewer --release -- \
//!   examples/sim-soft/spade-delaunay-spike/out/sleeve_xslab_triangulated.ply
//! ```

// Banked-as-wrong-architecture spike code per the docstring header
// above; preserved as a learning record, NOT held to production
// quality. Blanket `#![allow(warnings)]` so the wrong-architecture
// scratch logic doesn't have to meet A-grade clippy standards. Future
// maintainers reading this file: see the viz arc memo, not this code.
#![allow(warnings)]
#![allow(clippy::all)]
#![allow(clippy::pedantic)]

use std::path::Path;

use anyhow::Result;
use cf_design::Solid;
use mesh_io::{load_ply_attributed, save_ply_attributed};
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use nalgebra::Vector3;
use sim_soft::Sdf;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation};

// Geometry constants — must match row 24's main.rs (the source of the
// PLY we're triangulating). Hard-coded here rather than imported
// because the spike is throwaway; F1 will lift these as parameters
// to `sim_soft::viz::slab_mesh()`.
const SCAN_HX: f64 = 0.020;
const SCAN_HY: f64 = 0.015;
const SCAN_HZ: f64 = 0.040;
const WRAP_THICKNESS: f64 = 0.014;
const CELL_SIZE: f64 = 0.004;

/// Spacing between analytical boundary vertices on the SDF=0 surface.
/// Sample at CELL_SIZE / 2 = 2 mm so adjacent boundary samples are
/// closer than the typical inter-centroid spacing on the BCC
/// projection — the Delaunay then snaps interior triangles to
/// the boundary samples instead of zigzagging between centroids.
const BOUNDARY_SAMPLE_SPACING_M: f64 = CELL_SIZE / 2.0;

/// Pre-prune radius around the SDF=0 surface. Centroids within this
/// distance of any body boundary are dropped before triangulation,
/// so the analytical boundary samples (at SDF=0 exactly) dominate
/// the near-boundary triangulation. Without this prune, constrained
/// Delaunay produces V-shaped notches at the outer boundary where
/// near-boundary BCC centroids compete with the analytical
/// samples. Distance computed via `body.eval(centroid).abs()` —
/// drops centroids on EITHER side of any zero crossing.
const BOUNDARY_PRUNE_RADIUS_M: f64 = CELL_SIZE / 4.0;

/// Vertex passed to `spade` carrying the source-PLY index so we can
/// look up positions + scalars after triangulation. `spade` requires
/// `HasPosition` for vertex insertion + a `Point2`-shaped position
/// accessor.
#[derive(Clone, Copy, Debug)]
struct IndexedPoint {
    idx: usize,
    y: f64,
    z: f64,
}

impl HasPosition for IndexedPoint {
    type Scalar = f64;

    fn position(&self) -> Point2<f64> {
        Point2::new(self.y, self.z)
    }
}

/// Filter triangles by max edge length. Row 24's BCC lattice spacing
/// is `CELL_SIZE = 0.004 m`; centroid spacing in the slab is
/// approximately CELL_SIZE × √2 / 2 ≈ 2.83 mm at the densest
/// orientation. Cavity-bridging triangles span the inner-cavity gap
/// (~30 mm wide along y or 80 mm along z), so they have at least one
/// edge ≥ 30 mm. A max-edge cutoff of 1.5 × CELL_SIZE = 6 mm cleanly
/// separates the two regimes.
const MAX_EDGE_LENGTH_M: f64 = 0.006;

/// Centroid-in-body filter tolerance (m). A triangle is kept iff its
/// 2D centroid evaluates ≤ -ε against the body SDF (i.e. strictly
/// inside the body with at least ε margin). The ε margin handles
/// floating-point sloppiness at the boundary; centroids exactly on
/// the surface are dropped, but BCC-lattice centroids are typically
/// CELL_SIZE/4 ≈ 1 mm away from the boundary so this is rarely
/// binding.
const SDF_INTERIOR_MARGIN_M: f64 = 1.0e-6;

/// Reconstruct the row 24 body SDF for the centroid-in-body filter.
/// Same recipe as `examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/src/main.rs`'s
/// `build_scan_solid()` + `build_outer_envelope()` + `build_sleeve_body()`.
fn build_body_sdf() -> Solid {
    let scan = Solid::cuboid(Vector3::new(SCAN_HX, SCAN_HY, SCAN_HZ));
    let outer = scan.clone().offset(WRAP_THICKNESS);
    outer.subtract(scan)
}

/// Sample analytical boundary vertices along the body's rectangular
/// boundaries (outer envelope at ±(SCAN_H + WRAP) and inner cavity
/// at ±SCAN_H). Returns two ORDERED LOOPS of 2D y-z coordinates
/// (outer + inner), each ordered counter-clockwise so consecutive
/// vertices form a proper boundary edge for `ConstrainedDelaunayTriangulation`.
///
/// Each loop omits the wraparound — the constraint API's
/// `closed=true` flag will add the closing edge.
fn sample_boundary_loops() -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
    let outer_hy = SCAN_HY + WRAP_THICKNESS;
    let outer_hz = SCAN_HZ + WRAP_THICKNESS;
    let inner_hy = SCAN_HY;
    let inner_hz = SCAN_HZ;

    // Sample one rectangle's perimeter in CCW order. Returns the
    // open loop (no wraparound vertex).
    let sample_rectangle_ccw = |hy: f64, hz: f64| -> Vec<(f64, f64)> {
        let n_y = ((2.0 * hy) / BOUNDARY_SAMPLE_SPACING_M).round() as usize;
        let n_z = ((2.0 * hz) / BOUNDARY_SAMPLE_SPACING_M).round() as usize;
        let dy = 2.0 * hy / (n_y as f64);
        let dz = 2.0 * hz / (n_z as f64);
        let mut loop_points = Vec::new();
        // Bottom edge: (-hy, -hz) → (+hy, -hz), exclusive of the end
        // corner (which is the start of the right edge).
        for i in 0..n_y {
            loop_points.push((-hy + (i as f64) * dy, -hz));
        }
        // Right edge: (+hy, -hz) → (+hy, +hz), exclusive of end.
        for i in 0..n_z {
            loop_points.push((hy, -hz + (i as f64) * dz));
        }
        // Top edge: (+hy, +hz) → (-hy, +hz) (reversed-y), exclusive
        // of end.
        for i in 0..n_y {
            loop_points.push((hy - (i as f64) * dy, hz));
        }
        // Left edge: (-hy, +hz) → (-hy, -hz) (reversed-z), exclusive
        // of end (closes the loop via wraparound).
        for i in 0..n_z {
            loop_points.push((-hy, hz - (i as f64) * dz));
        }
        loop_points
    };

    let outer_loop = sample_rectangle_ccw(outer_hy, outer_hz);
    let inner_loop = sample_rectangle_ccw(inner_hy, inner_hz);
    (outer_loop, inner_loop)
}

/// Find the nearest existing centroid in 2D y-z to the given point;
/// return its index. Used to copy scalars from the nearest centroid
/// to a new boundary vertex, so the heatmap doesn't have ε-thin
/// dark/empty artefacts at the boundary.
fn nearest_centroid_idx(centroids: &[Point3<f64>], y: f64, z: f64) -> usize {
    let mut best_idx = 0;
    let mut best_dist_sq = f64::INFINITY;
    for (i, c) in centroids.iter().enumerate() {
        let dy = c.y - y;
        let dz = c.z - z;
        let d2 = dy * dy + dz * dz;
        if d2 < best_dist_sq {
            best_dist_sq = d2;
            best_idx = i;
        }
    }
    best_idx
}

fn main() -> Result<()> {
    let workspace_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .and_then(|p| p.parent())
        .ok_or_else(|| anyhow::anyhow!("cannot resolve workspace root"))?;
    let input_path = workspace_root
        .join("examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp")
        .join("out/sleeve_xslab_final.ply");
    let mesh = load_ply_attributed(&input_path)
        .map_err(|e| anyhow::anyhow!("cannot load row 24 PLY at {input_path:?}: {e:?}"))?;

    let raw_centroid_positions: Vec<Point3<f64>> = mesh.geometry.vertices.clone();
    let n_raw_centroids = raw_centroid_positions.len();
    println!("loaded {n_raw_centroids} raw centroids from {input_path:?}");
    println!(
        "scalar extras: {:?}",
        mesh.extras.keys().collect::<Vec<_>>()
    );

    // Pre-prune: drop centroids within BOUNDARY_PRUNE_RADIUS of any
    // SDF=0 surface. The analytical boundary samples (added below)
    // dominate the near-boundary region without competition, which
    // eliminates the V-notch artifacts that constrained Delaunay
    // produces when nearby BCC centroids compete with boundary
    // samples for triangulation.
    let body_for_prune = build_body_sdf();
    let mut centroid_positions: Vec<Point3<f64>> = Vec::new();
    let mut centroid_orig_idx: Vec<usize> = Vec::new();
    let mut n_pruned = 0usize;
    for (idx, p) in raw_centroid_positions.iter().enumerate() {
        let sdf_at_centroid = body_for_prune.eval(*p);
        if sdf_at_centroid.abs() < BOUNDARY_PRUNE_RADIUS_M {
            n_pruned += 1;
            continue;
        }
        centroid_positions.push(*p);
        centroid_orig_idx.push(idx);
    }
    let n_centroids = centroid_positions.len();
    println!(
        "pruned {n_pruned} centroids within {} m of body boundary; {n_centroids} kept",
        BOUNDARY_PRUNE_RADIUS_M
    );

    // Sample analytical boundary vertices on the body's rectangular
    // SDF=0 surfaces (outer envelope + inner cavity), in
    // counter-clockwise loop order so they can be added as
    // `ConstrainedDelaunayTriangulation` boundary constraints.
    // Scalars for boundary vertices are copied from the nearest
    // existing centroid so the heatmap stays continuous across the
    // boundary.
    let (outer_loop_2d, inner_loop_2d) = sample_boundary_loops();
    let n_outer = outer_loop_2d.len();
    let n_inner = inner_loop_2d.len();
    let n_boundary = n_outer + n_inner;
    println!("sampled {n_outer} outer + {n_inner} inner = {n_boundary} boundary vertices");

    // Build the augmented vertex list: pruned centroids first,
    // then outer-boundary loop, then inner-boundary loop.
    let mut all_positions: Vec<Point3<f64>> = centroid_positions.clone();
    let mut nn_for_boundary: Vec<usize> = Vec::with_capacity(n_boundary);
    for &(y, z) in outer_loop_2d.iter().chain(inner_loop_2d.iter()) {
        let nn = nearest_centroid_idx(&centroid_positions, y, z);
        nn_for_boundary.push(nn);
        all_positions.push(Point3::new(0.0, y, z));
    }
    let n_total = all_positions.len();
    println!(
        "triangulating {n_total} vertices total ({n_centroids} centroids + {n_boundary} boundary)"
    );

    // Track outer + inner loop start indices in the augmented set
    // for the constraint-edge insertion below.
    let outer_loop_start = n_centroids;
    let inner_loop_start = n_centroids + n_outer;

    // Project to 2D (y, z) for triangulation.
    let indexed_points: Vec<IndexedPoint> = all_positions
        .iter()
        .enumerate()
        .map(|(idx, p)| IndexedPoint {
            idx,
            y: p.y,
            z: p.z,
        })
        .collect();

    // Triangulate via `ConstrainedDelaunayTriangulation` so we can
    // add the outer + inner rectangle boundaries as edge
    // constraints. Without constraints, vanilla Delaunay produces
    // chamfered corners where nearby BCC centroids fall inside the
    // corner-triangle's circumcircle and hijack the corner. With
    // constraint edges enforcing the rectangle perimeter, the
    // corners stay sharp.
    let mut delaunay: ConstrainedDelaunayTriangulation<IndexedPoint> =
        ConstrainedDelaunayTriangulation::new();
    for ip in &indexed_points {
        delaunay
            .insert(*ip)
            .map_err(|e| anyhow::anyhow!("delaunay insert failed: {e:?}"))?;
    }

    // Add outer boundary loop constraint edges (closed loop).
    for i in 0..n_outer {
        let from_idx = outer_loop_start + i;
        let to_idx = outer_loop_start + ((i + 1) % n_outer);
        delaunay
            .add_constraint_edge(indexed_points[from_idx], indexed_points[to_idx])
            .map_err(|e| anyhow::anyhow!("outer boundary constraint failed: {e:?}"))?;
    }
    // Inner boundary loop constraint edges.
    for i in 0..n_inner {
        let from_idx = inner_loop_start + i;
        let to_idx = inner_loop_start + ((i + 1) % n_inner);
        delaunay
            .add_constraint_edge(indexed_points[from_idx], indexed_points[to_idx])
            .map_err(|e| anyhow::anyhow!("inner boundary constraint failed: {e:?}"))?;
    }
    let n_constraints = delaunay.num_constraints();
    let n_triangles_raw = delaunay.num_inner_faces();
    println!(
        "constrained delaunay: {n_constraints} constraint edges, {n_triangles_raw} triangles (pre-filter)"
    );

    // Combined filter: max-edge-length AND centroid-in-body.
    //
    // - Max-edge-length is a cheap pre-filter for cavity-spanning
    //   triangles (≥ 30 mm cavity width). BCC-grid triangles have
    //   all edges ≤ ~5.7 mm (CELL_SIZE × √2). The 6 mm cutoff
    //   cleanly separates the two regimes; this filter alone runs
    //   in O(F) without SDF evaluation.
    // - Centroid-in-body is the rigorous principled filter. Compute
    //   each triangle's centroid in 3D (with x ≈ 0 from the slab
    //   cut), evaluate the body SDF, drop if outside (SDF > 0 means
    //   point is in the cavity OR outside the outer envelope).
    //   Catches BOTH cavity-spanning triangles (centroid in cavity)
    //   AND convex-hull fan-out artefacts at corners (centroid in
    //   the small triangular gap between body's concave boundary
    //   and the Delaunay convex hull). No threshold tuning needed
    //   — the SDF defines "in body" exactly.
    //
    // Both must be satisfied for the triangle to keep. The
    // max-edge pre-filter is a perf optimisation (skip SDF eval on
    // obvious cavity-spanners); the SDF filter is the load-bearing
    // correctness gate.
    // Pre-compute SDF at every vertex once (cheap; reused across all
    // triangles incident to that vertex).
    let body = build_body_sdf();
    let vertex_sdf: Vec<f64> = all_positions.iter().map(|p| body.eval(*p)).collect();

    let mut faces: Vec<[u32; 3]> = Vec::new();
    let mut n_filtered_edge = 0usize;
    let mut n_filtered_outside_vertex = 0usize;
    let mut n_filtered_centroid = 0usize;
    let mut max_kept_edge_m: f64 = 0.0;
    for face in delaunay.inner_faces() {
        let verts = face.vertices();
        let i0 = verts[0].data().idx;
        let i1 = verts[1].data().idx;
        let i2 = verts[2].data().idx;
        let p0 = all_positions[i0];
        let p1 = all_positions[i1];
        let p2 = all_positions[i2];
        let e01 = (p0 - p1).norm();
        let e12 = (p1 - p2).norm();
        let e20 = (p2 - p0).norm();
        let max_edge = e01.max(e12).max(e20);
        if max_edge >= MAX_EDGE_LENGTH_M {
            n_filtered_edge += 1;
            continue;
        }
        // Drop triangles where ANY vertex is outside the body.
        // The analytical rectangular boundary samples at the corners
        // are outside the actual rounded outer envelope (`Solid::offset`
        // rounds corners by `WRAP_THICKNESS`), so corner triangles
        // that span them produce spike artefacts even when the
        // triangle's centroid happens to land inside the body
        // (deep interior vertex pulls the average inward). The
        // any-vertex-outside filter is more conservative than
        // centroid-in-body and catches these.
        let max_vertex_sdf = vertex_sdf[i0].max(vertex_sdf[i1]).max(vertex_sdf[i2]);
        if max_vertex_sdf > -SDF_INTERIOR_MARGIN_M {
            n_filtered_outside_vertex += 1;
            continue;
        }
        // Centroid-in-body as a backstop for any case the
        // any-vertex-outside test misses.
        let centroid_2d = (p0 + p1.coords + p2.coords) / 3.0;
        let centroid_3d = nalgebra::Point3::new(centroid_2d.x, centroid_2d.y, centroid_2d.z);
        let sdf_val = body.eval(centroid_3d);
        if sdf_val > -SDF_INTERIOR_MARGIN_M {
            n_filtered_centroid += 1;
            continue;
        }
        faces.push([i0 as u32, i1 as u32, i2 as u32]);
        if max_edge > max_kept_edge_m {
            max_kept_edge_m = max_edge;
        }
    }
    let n_triangles_kept = faces.len();
    println!(
        "kept {n_triangles_kept} triangles \
         (filtered {n_filtered_edge} by max-edge, {n_filtered_outside_vertex} by any-vertex-outside, {n_filtered_centroid} by centroid-in-body backstop)"
    );
    println!("max kept edge = {:.3} mm", max_kept_edge_m * 1000.0);

    // Build output mesh: augmented vertex set (centroids + boundary
    // samples), faces from the filtered triangulation, scalars
    // extended for boundary vertices via nearest-neighbour copy
    // from the original centroid set. Boundary vertices preserve
    // smooth heatmap continuity at the edges.
    let mut out_geom = IndexedMesh::new();
    for v in &all_positions {
        out_geom.vertices.push(Point3::new(v.x, v.y, v.z));
    }
    out_geom.faces = faces;

    let mut out_mesh = AttributedMesh::new(out_geom);
    // Extend zone_ids if present. After prune, scalars need to be
    // re-indexed: kept centroids → original-index lookup → original
    // scalar value, then boundary-vertex scalars from nearest-neighbour.
    if let Some(orig_zone_ids) = &mesh.zone_ids {
        let mut extended: Vec<u32> = centroid_orig_idx
            .iter()
            .map(|&i| orig_zone_ids[i])
            .collect();
        // nn_for_boundary indices reference the PRUNED centroid set; map
        // them through centroid_orig_idx to get original-source indices.
        for &nn_idx in &nn_for_boundary {
            let orig = centroid_orig_idx[nn_idx];
            extended.push(orig_zone_ids[orig]);
        }
        out_mesh.zone_ids = Some(extended);
    }
    // Extend each extra scalar similarly.
    for (name, values) in &mesh.extras {
        let mut extended: Vec<f32> = centroid_orig_idx.iter().map(|&i| values[i]).collect();
        for &nn_idx in &nn_for_boundary {
            let orig = centroid_orig_idx[nn_idx];
            extended.push(values[orig]);
        }
        out_mesh.insert_extra(name, extended)?;
    }

    let out_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("out")
        .join("sleeve_xslab_triangulated.ply");
    std::fs::create_dir_all(out_path.parent().unwrap())?;
    save_ply_attributed(&out_mesh, &out_path, true)?;
    println!("wrote {out_path:?}");

    println!();
    println!("View in cf-view (psi_j_per_m3 is the headline scalar):");
    println!(
        "  cargo run -p cf-viewer --release -- {}",
        out_path.display()
    );

    Ok(())
}
