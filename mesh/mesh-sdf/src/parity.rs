//! A sign oracle by the parity of a ray's crossings.
//!
//! A point is inside a closed surface when a ray from it crosses the surface an
//! odd number of times. That holds whatever the surface's winding, and leans on
//! neither a closest feature, as the pseudo-normals do, nor a flood. On the
//! product scan the flood fill's sign disagreed with both others within a
//! quarter cell of the surface, and the pseudo-normals' in one region away from
//! it; parity agreed with the flood fill away from the surface and with the
//! pseudo-normals at it, but for a handful of samples (soft-contact recon §16u).
//!
//! **Closed surfaces only.** A ray can leave through a hole without being
//! counted, so an open surface has no inside for this oracle. And the count is
//! even-odd: where closed parts overlap, the region enclosed twice reads
//! outside (`a_region_enclosed_twice_reads_outside`).

use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::{SdfError, SdfResult, Sign};

/// Inside when a ray from the point crosses the surface an odd number of
/// times: the majority of three rays.
///
/// A ray through an edge or a vertex can count one crossing twice, or none;
/// the majority outvotes one such ray, but not two. No axis-aligned or 45°
/// face lies along any of the three directions.
///
/// Built once. For each direction the triangles are binned by their shadow on
/// the plane across it, so a query tests only the triangles its ray can meet.
#[derive(Debug, Clone)]
pub struct ParitySign {
    triangles: Vec<[Point3<f64>; 3]>,
    rays: [RayBins; 3],
}

/// One ray direction and the triangles binned by their shadow across it.
#[derive(Debug, Clone)]
struct RayBins {
    direction: Vector3<f64>,
    /// Two unit vectors across the direction, and across each other.
    across: [Vector3<f64>; 2],
    /// The shadow's lowest corner, in those two coordinates.
    low: [f64; 2],
    /// A bin's side.
    cell: f64,
    /// Bins along each of the two coordinates.
    size: [usize; 2],
    /// Where each bin's triangles start in `members`; one more entry than bins.
    starts: Vec<usize>,
    /// The triangles of every bin, bin after bin.
    members: Vec<u32>,
}

/// The rays' directions, before normalising.
const DIRECTIONS: [[f64; 3]; 3] = [
    [0.5773, 0.5791, 0.5759],
    [-0.6123, 0.3141, 0.7254],
    [0.2718, -0.8182, 0.5064],
];

impl ParitySign {
    /// Bin `mesh`'s triangles for the three rays.
    ///
    /// # Errors
    /// [`SdfError::EmptyMesh`] for a mesh with no faces, and
    /// [`SdfError::FaceIndexOutOfRange`] for a face that names a vertex the
    /// mesh does not have.
    pub fn new(mesh: &IndexedMesh) -> SdfResult<Self> {
        if mesh.faces.is_empty() {
            return Err(SdfError::EmptyMesh);
        }
        let vertex_count = mesh.vertices.len();
        let mut triangles = Vec::with_capacity(mesh.faces.len());
        for face in &mesh.faces {
            let mut corners = [Point3::origin(); 3];
            for (corner, &index) in corners.iter_mut().zip(face) {
                *corner =
                    *mesh
                        .vertices
                        .get(index as usize)
                        .ok_or(SdfError::FaceIndexOutOfRange {
                            index,
                            vertex_count,
                        })?;
            }
            triangles.push(corners);
        }
        // A bin a mean edge across: a ray's bin then holds the few triangles
        // near its shadow. `RayBins::new` coarsens it for a scattered mesh.
        let edges: f64 = triangles
            .iter()
            .map(|[a, b, c]| (b - a).norm() + (c - b).norm() + (a - c).norm())
            .sum();
        // A face count converts exactly below 2^53.
        #[allow(clippy::cast_precision_loss)]
        let mean_edge = edges / (3 * triangles.len()) as f64;
        let cell = if mean_edge > 0.0 { mean_edge } else { 1.0 };
        let rays = DIRECTIONS.map(|d| RayBins::new(&triangles, Vector3::from(d).normalize(), cell));
        Ok(Self { triangles, rays })
    }
}

impl RayBins {
    fn new(triangles: &[[Point3<f64>; 3]], direction: Vector3<f64>, cell: f64) -> Self {
        let seed = if direction.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let first = (seed - direction * seed.dot(&direction)).normalize();
        let across = [first, direction.cross(&first)];
        let shadow = |p: &Point3<f64>| [p.coords.dot(&across[0]), p.coords.dot(&across[1])];
        let mut low = [f64::INFINITY; 2];
        let mut high = [f64::NEG_INFINITY; 2];
        for triangle in triangles {
            for corner in triangle {
                let s = shadow(corner);
                for axis in 0..2 {
                    low[axis] = low[axis].min(s[axis]);
                    high[axis] = high[axis].max(s[axis]);
                }
            }
        }
        let extent = [high[0] - low[0], high[1] - low[1]];
        // A face count converts exactly below 2^53.
        #[allow(clippy::cast_precision_loss)]
        let most = 4.0 * triangles.len() as f64;
        // No finer than splits the shadow's box into four bins a face, or its
        // longer side into four: a mesh of small parts far apart would
        // otherwise bin into more cells than memory holds. The bins then number
        // at most twelve a face, and one. A triangle is listed in every bin its
        // shadow's box covers, so a few triangles far larger than the rest can
        // add many more entries than that.
        let cell = cell
            .max((extent[0] * extent[1] / most).sqrt())
            .max(extent[0].max(extent[1]) / most);
        // The shadow's extent over that cell: non-negative, and at most four a
        // face along either side.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let size = extent.map(|e| (e / cell).floor() as usize + 1);
        let bin_of = |value: f64, axis: usize| -> usize {
            // Finite and floored at zero; clamped to the last bin below.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let bin = ((value - low[axis]) / cell).floor().max(0.0) as usize;
            bin.min(size[axis] - 1)
        };
        // Each triangle's shadow box, in bins.
        let boxes: Vec<[usize; 4]> = triangles
            .iter()
            .map(|triangle| {
                let s = triangle.map(|corner| shadow(&corner));
                let lo = [0, 1].map(|a| s[0][a].min(s[1][a]).min(s[2][a]));
                let hi = [0, 1].map(|a| s[0][a].max(s[1][a]).max(s[2][a]));
                [
                    bin_of(lo[0], 0),
                    bin_of(hi[0], 0),
                    bin_of(lo[1], 1),
                    bin_of(hi[1], 1),
                ]
            })
            .collect();
        let mut counts = vec![0_usize; size[0] * size[1]];
        for &[a0, a1, b0, b1] in &boxes {
            for b in b0..=b1 {
                for a in a0..=a1 {
                    counts[b * size[0] + a] += 1;
                }
            }
        }
        let mut starts = Vec::with_capacity(counts.len() + 1);
        let mut total = 0;
        for &count in &counts {
            starts.push(total);
            total += count;
        }
        starts.push(total);
        let mut next = starts.clone();
        let mut members = vec![0_u32; total];
        for (index, &[a0, a1, b0, b1]) in (0_u32..).zip(&boxes) {
            for b in b0..=b1 {
                for a in a0..=a1 {
                    let bin = b * size[0] + a;
                    members[next[bin]] = index;
                    next[bin] += 1;
                }
            }
        }
        Self {
            direction,
            across,
            low,
            cell,
            size,
            starts,
            members,
        }
    }

    /// How many of `triangles` the ray from `origin` crosses.
    fn crossings(&self, triangles: &[[Point3<f64>; 3]], origin: Point3<f64>) -> usize {
        let s = [
            origin.coords.dot(&self.across[0]) - self.low[0],
            origin.coords.dot(&self.across[1]) - self.low[1],
        ];
        if s.iter().any(|&x| x < 0.0) {
            return 0;
        }
        // Non-negative and finite here.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let bin = s.map(|x| (x / self.cell).floor() as usize);
        if bin[0] >= self.size[0] || bin[1] >= self.size[1] {
            return 0;
        }
        let index = bin[1] * self.size[0] + bin[0];
        self.members[self.starts[index]..self.starts[index + 1]]
            .iter()
            .filter(|&&t| crosses(&triangles[t as usize], origin, self.direction))
            .count()
    }
}

/// Whether the ray from `origin` along `direction` meets `triangle` ahead of
/// its origin (Möller–Trumbore, either side of the triangle).
fn crosses(triangle: &[Point3<f64>; 3], origin: Point3<f64>, direction: Vector3<f64>) -> bool {
    let [a, b, c] = *triangle;
    let (ab, ac) = (b - a, c - a);
    let p = direction.cross(&ac);
    let det = ab.dot(&p);
    if det == 0.0 {
        return false;
    }
    let to = origin - a;
    let u = to.dot(&p) / det;
    let q = to.cross(&ab);
    let v = direction.dot(&q) / det;
    let t = ac.dot(&q) / det;
    u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t > 0.0
}

impl Sign for ParitySign {
    fn is_inside(&self, p: Point3<f64>) -> bool {
        self.rays
            .iter()
            .filter(|ray| ray.crossings(&self.triangles, p) % 2 == 1)
            .count()
            >= 2
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PseudoNormalSign;
    use crate::test_fixtures::uv_sphere;

    /// A unit cube as a triangle soup, three vertices a face, wound outward; or inward.
    fn cube_soup(inward: bool) -> IndexedMesh {
        let corner = |i: usize| {
            Point3::new(
                f64::from(u8::from(i & 1 != 0)),
                f64::from(u8::from(i & 2 != 0)),
                f64::from(u8::from(i & 4 != 0)),
            )
        };
        let faces = [
            [0, 2, 1],
            [1, 2, 3],
            [4, 5, 6],
            [5, 7, 6],
            [0, 1, 4],
            [1, 5, 4],
            [2, 6, 3],
            [3, 6, 7],
            [0, 4, 2],
            [2, 4, 6],
            [1, 3, 5],
            [3, 7, 5],
        ];
        let mut soup = IndexedMesh::new();
        for face in faces {
            let first = u32::try_from(soup.vertices.len()).unwrap();
            let order = if inward { [0, 2, 1] } else { [0, 1, 2] };
            for k in order {
                soup.vertices.push(corner(face[k]));
            }
            soup.faces.push([first, first + 1, first + 2]);
        }
        soup
    }

    #[test]
    fn parity_tells_inside_from_outside_by_a_cubes_edges_and_corners_either_winding() {
        for inward in [false, true] {
            let sign = ParitySign::new(&cube_soup(inward)).unwrap();
            for (point, inside) in [
                (Point3::new(0.5, 0.5, 0.5), true),
                (Point3::new(0.999, 0.999, 0.5), true),
                (Point3::new(1.001, 1.001, 0.5), false),
                (Point3::new(0.001, 0.001, 0.001), true),
                (Point3::new(-0.001, -0.001, -0.001), false),
                (Point3::new(1.001, 0.5, 0.5), false),
                (Point3::new(0.5, 0.5, 1e-9), true),
                (Point3::new(2.0, 2.0, 2.0), false),
                (Point3::new(-3.0, 0.5, 0.5), false),
            ] {
                assert_eq!(sign.is_inside(point), inside, "{point} (inward {inward})");
            }
        }
    }

    /// `mesh` scaled by `scale` about the origin, then moved by `offset`.
    fn moved(mut mesh: IndexedMesh, scale: f64, offset: Vector3<f64>) -> IndexedMesh {
        for v in &mut mesh.vertices {
            *v = Point3::from(v.coords * scale + offset);
        }
        mesh
    }

    /// Two meshes as one.
    fn joined(mut a: IndexedMesh, b: &IndexedMesh) -> IndexedMesh {
        let first = u32::try_from(a.vertices.len()).unwrap();
        a.vertices.extend(&b.vertices);
        a.faces.extend(b.faces.iter().map(|f| f.map(|v| v + first)));
        a
    }

    #[test]
    fn a_region_enclosed_twice_reads_outside() {
        // Two closed cubes that overlap, no face of one in a plane of the other: the even-odd count reads the
        // overlap as outside, and each cube's own part as inside.
        let mesh = joined(
            cube_soup(false),
            &moved(cube_soup(false), 1.0, Vector3::new(0.5, 0.2, 0.3)),
        );
        let sign = ParitySign::new(&mesh).unwrap();
        for (point, inside) in [
            (Point3::new(0.75, 0.6, 0.65), false),
            (Point3::new(0.25, 0.5, 0.5), true),
            (Point3::new(1.25, 0.6, 0.65), true),
            (Point3::new(2.0, 2.0, 2.0), false),
        ] {
            assert_eq!(sign.is_inside(point), inside, "{point}");
        }
    }

    #[test]
    fn a_mesh_of_small_parts_far_apart_bins_into_few_cells() {
        // Two cubes a micrometre across, a metre apart: along every axis, where binned a mean edge across a
        // direction's shadow would hold about 10¹² bins; and along the first ray's first cross-axis, where that
        // ray's shadow is a metre long and a micrometre wide. Capped, at most twelve a face and one.
        let tiny = moved(cube_soup(false), 1e-6, Vector3::zeros());
        let across = ParitySign::new(&tiny).unwrap().rays[0].across[0];
        for apart in [Vector3::repeat(1.0), across] {
            let mesh = joined(tiny.clone(), &moved(tiny.clone(), 1.0, apart));
            let sign = ParitySign::new(&mesh).unwrap();
            for ray in &sign.rays {
                assert!(
                    ray.starts.len() - 1 <= 12 * mesh.faces.len() + 1,
                    "{:?}",
                    ray.size
                );
            }
            let centre = Vector3::repeat(5e-7);
            for (point, inside) in [
                (Point3::from(centre), true),
                (Point3::from(centre + apart), true),
                (Point3::from(centre + apart / 2.0), false),
                (Point3::new(2e-6, 5e-7, 5e-7), false),
            ] {
                assert_eq!(sign.is_inside(point), inside, "{point}");
            }
        }
    }

    #[test]
    fn one_rays_miscount_is_outvoted_either_way() {
        // A stray triangle across the first ray only, a point outside the cube and a point inside: the first ray
        // counts one crossing too many from each, and the other two outvote it.
        let first = Vector3::from(DIRECTIONS[0]).normalize();
        let e1 = first.cross(&Vector3::x()).normalize();
        let e2 = first.cross(&e1);
        for (point, stray_at, inside) in [
            (Point3::new(2.0, 2.0, 2.0), 1.0, false),
            (Point3::new(0.5, 0.5, 0.5), 3.0, true),
        ] {
            let mut mesh = cube_soup(false);
            let c = point + first * stray_at;
            let first_vertex = u32::try_from(mesh.vertices.len()).unwrap();
            mesh.vertices.extend([
                c + e1 * 0.1,
                c + e1 * -0.05 + e2 * 0.0866,
                c + e1 * -0.05 + e2 * -0.0866,
            ]);
            mesh.faces
                .push([first_vertex, first_vertex + 1, first_vertex + 2]);
            let sign = ParitySign::new(&mesh).unwrap();
            let counts = sign
                .rays
                .each_ref()
                .map(|ray| ray.crossings(&sign.triangles, point) % 2 == 1);
            assert_eq!(counts, [!inside, inside, inside], "{point}");
            assert_eq!(sign.is_inside(point), inside, "{point}");
        }
    }

    #[test]
    fn parity_agrees_with_the_sphere_and_with_the_pseudo_normals_on_a_clean_mesh() {
        // A closed, cleanly wound sphere of 64 × 128 facets: a point further from the radius than the facets' sag
        // (under 2e-3 of it) is inside exactly when nearer the centre, by both signs.
        let radius = 0.02;
        let mesh = uv_sphere(radius, 64, 128);
        let parity = ParitySign::new(&mesh).unwrap();
        let pseudo = PseudoNormalSign::new(&mesh).unwrap();
        let mut checked = 0;
        for n in 0..3000_u32 {
            let t = f64::from(n);
            let direction = Vector3::new(
                (t * 0.618_034).fract() - 0.5,
                (t * 0.414_214).fract() - 0.5,
                (t * 0.732_051).fract() - 0.5,
            );
            if direction.norm() < 1e-3 {
                continue;
            }
            let r = radius * (0.2 + 1.6 * (t * 0.271_828).fract());
            if (r - radius).abs() < 0.004 * radius {
                continue;
            }
            let p = Point3::from(direction.normalize() * r);
            assert_eq!(parity.is_inside(p), r < radius, "{p}");
            assert_eq!(pseudo.is_inside(p), r < radius, "{p}");
            checked += 1;
        }
        assert!(checked > 2500, "{checked}");
    }

    #[test]
    fn a_mesh_with_no_faces_or_a_missing_vertex_is_refused() {
        assert!(matches!(
            ParitySign::new(&IndexedMesh::new()),
            Err(SdfError::EmptyMesh)
        ));
        let mut broken = cube_soup(false);
        broken.faces[3][1] = 1_000;
        assert!(matches!(
            ParitySign::new(&broken),
            Err(SdfError::FaceIndexOutOfRange { index: 1_000, .. })
        ));
    }
}
