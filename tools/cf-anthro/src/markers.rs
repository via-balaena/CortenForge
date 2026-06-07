//! Small marker meshes for visualizing detected landmarks in cf-viewer
//! `--assembly` mode (each piece gets its own color). Markers protrude beyond
//! the limb surface so they read clearly against an opaque scan.

use mesh_types::{IndexedMesh, Point3, Vector3};
use std::f64::consts::TAU;

/// A capped cylinder (tube) of radius `r` between arbitrary points `a` and `b`,
/// `n` sides — for bones and muscle/tendon paths in any 3D orientation (unlike
/// [`rod`], whose box is axis-aligned). Degenerate (a≈b) → empty.
pub fn tube(a: Point3<f64>, b: Point3<f64>, r: f64, n: usize) -> IndexedMesh {
    let axis = b - a;
    let len = axis.norm();
    if len < 1e-9 {
        return IndexedMesh::new();
    }
    let w = axis / len;
    // Orthonormal basis perpendicular to the axis.
    let t = if w.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u = t.cross(&w).normalize();
    let v = w.cross(&u);

    let mut vertices = Vec::with_capacity(2 * n + 2);
    for j in 0..n {
        let ang = TAU * (j as f64) / (n as f64);
        let dir = u * ang.cos() + v * ang.sin();
        vertices.push(a + dir * r);
        vertices.push(b + dir * r);
    }
    let ca = vertices.len();
    vertices.push(a);
    let cb = vertices.len();
    vertices.push(b);

    let mut faces = Vec::with_capacity(4 * n);
    for j in 0..n {
        let (a0, b0) = ((2 * j) as u32, (2 * j + 1) as u32);
        let k = (j + 1) % n;
        let (a1, b1) = ((2 * k) as u32, (2 * k + 1) as u32);
        faces.push([a0, a1, b1]);
        faces.push([a0, b1, b0]);
        faces.push([ca as u32, a1, a0]); // cap a
        faces.push([cb as u32, b0, b1]); // cap b
    }
    IndexedMesh { vertices, faces }
}

/// An open cylindrical band (a "bracelet") of radius `r`, height `h`, centered
/// at `(cx, cy, z)` with its axis along +z — rings the limb at a landmark.
pub fn band(cx: f64, cy: f64, z: f64, r: f64, h: f64, n: usize) -> IndexedMesh {
    let mut vertices = Vec::with_capacity(2 * n);
    for j in 0..n {
        let a = TAU * (j as f64) / (n as f64);
        let (x, y) = (cx + r * a.cos(), cy + r * a.sin());
        vertices.push(Point3::new(x, y, z - 0.5 * h));
        vertices.push(Point3::new(x, y, z + 0.5 * h));
    }
    let mut faces = Vec::with_capacity(2 * n);
    for j in 0..n {
        let (b0, t0) = ((2 * j) as u32, (2 * j + 1) as u32);
        let k = (j + 1) % n;
        let (b1, t1) = ((2 * k) as u32, (2 * k + 1) as u32);
        faces.push([b0, b1, t1]);
        faces.push([b0, t1, t0]);
    }
    IndexedMesh { vertices, faces }
}

/// An axis-aligned cube marker of half-size `half` centered at `c` — for point
/// landmarks (e.g. the epicondyle extents).
pub fn cube(c: Point3<f64>, half: f64) -> IndexedMesh {
    let s = half;
    let v = |dx: f64, dy: f64, dz: f64| Point3::new(c.x + dx * s, c.y + dy * s, c.z + dz * s);
    let vertices = vec![
        v(-1.0, -1.0, -1.0),
        v(1.0, -1.0, -1.0),
        v(1.0, 1.0, -1.0),
        v(-1.0, 1.0, -1.0),
        v(-1.0, -1.0, 1.0),
        v(1.0, -1.0, 1.0),
        v(1.0, 1.0, 1.0),
        v(-1.0, 1.0, 1.0),
    ];
    let faces = vec![
        [0, 2, 1],
        [0, 3, 2], // bottom
        [4, 5, 6],
        [4, 6, 7], // top
        [0, 1, 5],
        [0, 5, 4], // -y
        [2, 3, 7],
        [2, 7, 6], // +y
        [1, 2, 6],
        [1, 6, 5], // +x
        [3, 0, 4],
        [3, 4, 7], // -x
    ];
    IndexedMesh { vertices, faces }
}

/// A thin square-section rod between two points (a segment marker, e.g. the
/// limb axis or the epicondyle bar). Axis-aligned box spanning a..b, inflated by
/// `r` on **every** axis so it keeps thickness even when the endpoints share a
/// coordinate (e.g. a purely horizontal bar at one z).
pub fn rod(a: Point3<f64>, b: Point3<f64>, r: f64) -> IndexedMesh {
    let (lo, hi) = (
        Point3::new(a.x.min(b.x) - r, a.y.min(b.y) - r, a.z.min(b.z) - r),
        Point3::new(a.x.max(b.x) + r, a.y.max(b.y) + r, a.z.max(b.z) + r),
    );
    let center = Point3::new(
        (lo.x + hi.x) * 0.5,
        (lo.y + hi.y) * 0.5,
        (lo.z + hi.z) * 0.5,
    );
    let mut m = cube(center, 1.0);
    // Stretch the unit cube to the target extents.
    let (hx, hy, hz) = (
        (hi.x - lo.x) * 0.5,
        (hi.y - lo.y) * 0.5,
        (hi.z - lo.z) * 0.5,
    );
    for v in &mut m.vertices {
        v.x = center.x + (v.x - center.x) * hx;
        v.y = center.y + (v.y - center.y) * hy;
        v.z = center.z + (v.z - center.z) * hz;
    }
    m
}
