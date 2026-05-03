//! Hand-authored 50 mm × 50 mm × 50 mm watertight cube fixture used
//! by `main.rs`.
//!
//! Mirrors `mesh-types::unit_cube` (`mesh/mesh-types/src/mesh.rs:20-54`)
//! scaled by 50; topology + outward-CCW winding are bit-identical (the
//! `mesh-types` precedent passes its own
//! `unit_cube_volume == 1.0` + `!is_inside_out()` tests, so the
//! winding is already validated upstream — we lift it directly).
//!
//! `verify_fixture` locks math-pass-first anchors per
//! `feedback_math_pass_first_handauthored`: bit-exact per-vertex
//! coordinates, per-face cross-product cosine > `0.9999` against the
//! outward-normal direction, AABB at `(0, 0, 0)` / `(50, 50, 50)`, and
//! `signed_volume() == 125_000.0` within `1e-9`. Clean exit-0 from
//! `verify_fixture` IS the math-pass anchor on the fixture itself.

use anyhow::{Result, ensure};
use mesh_types::{Bounded, IndexedMesh, Point3, Vector3};

/// Edge length of the fixture cube in mm.
pub const SIDE: f64 = 50.0;

/// 8 vertices of the `[0, SIDE]³` cube — index order locked by
/// [`FACES`]. Identical to `mesh-types::unit_cube` (mesh.rs:24-31)
/// scaled by `SIDE`.
pub const VERTS: [[f64; 3]; 8] = [
    [0.0, 0.0, 0.0],    // 0
    [SIDE, 0.0, 0.0],   // 1
    [SIDE, SIDE, 0.0],  // 2
    [0.0, SIDE, 0.0],   // 3
    [0.0, 0.0, SIDE],   // 4
    [SIDE, 0.0, SIDE],  // 5
    [SIDE, SIDE, SIDE], // 6
    [0.0, SIDE, SIDE],  // 7
];

/// 12 outward-CCW triangles — bit-identical to
/// `mesh-types::unit_cube` (mesh.rs:35-51); topology is
/// scale-invariant.
pub const FACES: [[u32; 3]; 12] = [
    // Bottom face (z=0); outward = -z
    [0, 2, 1],
    [0, 3, 2],
    // Top face (z=SIDE); outward = +z
    [4, 5, 6],
    [4, 6, 7],
    // Front face (y=0); outward = -y
    [0, 1, 5],
    [0, 5, 4],
    // Back face (y=SIDE); outward = +y
    [3, 7, 6],
    [3, 6, 2],
    // Left face (x=0); outward = -x
    [0, 4, 7],
    [0, 7, 3],
    // Right face (x=SIDE); outward = +x
    [1, 2, 6],
    [1, 6, 5],
];

/// Outward-normal direction per face, indexed identically to [`FACES`].
pub const OUTWARD_NORMALS: [[f64; 3]; 12] = [
    [0.0, 0.0, -1.0],
    [0.0, 0.0, -1.0], // bottom
    [0.0, 0.0, 1.0],
    [0.0, 0.0, 1.0], // top
    [0.0, -1.0, 0.0],
    [0.0, -1.0, 0.0], // front
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0], // back
    [-1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0], // left
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0], // right
];

/// Tight tolerance for per-vertex / AABB / signed-volume anchors on
/// integer-spatial inputs.
pub const TIGHT_TOL: f64 = 1e-12;

/// Slightly looser tolerance for `signed_volume` on the cube.
///
/// `50³` stays integer in `f64` (well below `2^53`), so the
/// divergence-theorem sum lands bit-exact, but we keep a small
/// cushion against possible future re-orderings of the summation.
pub const VOLUME_TOL: f64 = 1e-9;

/// Cosine-similarity threshold for outward-winding anchors.
///
/// Every face's normalized cross product against its outward
/// direction exceeds `1 - 1e-4` (in practice every face hits exactly
/// `1.0` since the cube's edges are axis-aligned and the cross
/// product is a unit-magnitude axis vector).
pub const WINDING_COSINE_THRESHOLD: f64 = 0.9999;

/// Builds an axis-aligned `[0, side]³` cube.
///
/// Same outward-CCW topology as [`FACES`]. Used by [`cube_50mm`] and
/// by the `InteriorTooSmall` error-path anchor in `main.rs` (a 5 mm
/// cube fed to `for_fdm + cell_size 4`).
#[must_use]
pub fn cube_at_origin(side: f64) -> IndexedMesh {
    let mut mesh = IndexedMesh::with_capacity(VERTS.len(), FACES.len());
    for v in VERTS {
        mesh.vertices.push(Point3::new(
            v[0] / SIDE * side,
            v[1] / SIDE * side,
            v[2] / SIDE * side,
        ));
    }
    for f in FACES {
        mesh.faces.push(f);
    }
    mesh
}

/// Builds the 50 mm cube fixture.
///
/// 8 vertices at integer coordinates in `[0, SIDE]³`, 12 outward-CCW
/// triangles. Watertight (every edge shared by exactly two
/// triangles), validated by [`verify_fixture`].
#[must_use]
pub fn cube_50mm() -> IndexedMesh {
    cube_at_origin(SIDE)
}

/// Math-pass-first verification on the fixture. Clean exit-0 from
/// this fn IS the math-pass anchor.
///
/// # Errors
///
/// Returns the first anchor that fails. Anchors:
/// 1. `vertex_count == 8` and `face_count == 12`.
/// 2. Per-vertex coordinates within [`TIGHT_TOL`] of the expected
///    integer-spatial layout.
/// 3. Per-face cross-product cosine similarity against the
///    outward-normal direction exceeds [`WINDING_COSINE_THRESHOLD`].
/// 4. AABB bit-exact at `(0, 0, 0)` / `(SIDE, SIDE, SIDE)` within
///    [`TIGHT_TOL`].
/// 5. `signed_volume() == SIDE³` within [`VOLUME_TOL`].
pub fn verify_fixture(mesh: &IndexedMesh) -> Result<()> {
    ensure!(
        mesh.vertex_count() == 8,
        "expected 8 vertices, got {}",
        mesh.vertex_count()
    );
    ensure!(
        mesh.face_count() == 12,
        "expected 12 triangles, got {}",
        mesh.face_count()
    );

    for (i, v) in mesh.vertices.iter().enumerate() {
        let expected = VERTS[i];
        ensure!(
            (v.x - expected[0]).abs() < TIGHT_TOL
                && (v.y - expected[1]).abs() < TIGHT_TOL
                && (v.z - expected[2]).abs() < TIGHT_TOL,
            "vertex {i} drift: got ({:.15}, {:.15}, {:.15}), expected {expected:?}",
            v.x,
            v.y,
            v.z,
        );
    }

    for (fi, face) in mesh.faces.iter().enumerate() {
        let v0 = mesh.vertices[face[0] as usize];
        let v1 = mesh.vertices[face[1] as usize];
        let v2 = mesh.vertices[face[2] as usize];
        let edge_a = v1 - v0;
        let edge_b = v2 - v0;
        let cross = edge_a.cross(&edge_b);
        let normal = cross.normalize();
        let outward = Vector3::new(
            OUTWARD_NORMALS[fi][0],
            OUTWARD_NORMALS[fi][1],
            OUTWARD_NORMALS[fi][2],
        );
        let cosine = normal.dot(&outward);
        ensure!(
            cosine > WINDING_COSINE_THRESHOLD,
            "face {fi} winding drift: cosine vs outward {outward:?} = {cosine:.6} (want > {WINDING_COSINE_THRESHOLD})",
        );
    }

    let aabb = mesh.aabb();
    ensure!(
        aabb.min.x.abs() < TIGHT_TOL
            && aabb.min.y.abs() < TIGHT_TOL
            && aabb.min.z.abs() < TIGHT_TOL,
        "aabb.min drift: {:?}",
        aabb.min,
    );
    ensure!(
        (aabb.max.x - SIDE).abs() < TIGHT_TOL
            && (aabb.max.y - SIDE).abs() < TIGHT_TOL
            && (aabb.max.z - SIDE).abs() < TIGHT_TOL,
        "aabb.max drift: {:?}",
        aabb.max,
    );

    let sv = mesh.signed_volume();
    let expected_volume = SIDE * SIDE * SIDE;
    ensure!(
        (sv - expected_volume).abs() < VOLUME_TOL,
        "signed_volume drift: got {sv:.9}, want {expected_volume}",
    );

    Ok(())
}
