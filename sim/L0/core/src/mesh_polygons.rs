//! A mesh's polygons as MuJoCo's compiler merges them (`mjCMesh::MakePolygons`
//! and `MakePolygonNormals`, with the `MeshPolygon` helper,
//! `user_mesh.cc:2722-3005`): the coplanar triangles of its convex hull (or of
//! its own faces, without a hull) merged into polygonal faces, each face's
//! vertex path and unit normal, and for each vertex the faces it lies on. The
//! multi-contact of MuJoCo's native convex collision clips these faces.
//!
//! MuJoCo keeps the faces it merges in a `std::set` ordered by their normals;
//! this keeps them in a vector searched by the same comparison, which orders
//! them as the set does where the comparison is a strict weak order (the
//! normals of faces it does not merge more than its tolerance apart).

use nalgebra::Point3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::constraint::impedance::MJ_MINVAL;

/// `mjEPS` (`user_util.h:29`).
const EPS: f64 = 1e-14;
/// Two triangles merge where their normals' cosine is past this
/// (`PolygonCmp`, the cosine of 1.6e-3).
const MERGE_COS: f64 = 0.999_998_72;

/// A mesh's polygonal faces (MuJoCo's `mesh_polyvert`, `mesh_polynormal` and
/// `mesh_polymap`).
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MeshPolygons {
    /// Each polygon's vertices, as indices of the mesh's, in order around it.
    pub paths: Vec<Vec<usize>>,
    /// Each polygon's unit normal, from its first three vertices.
    pub normals: Vec<[f64; 3]>,
    /// For each mesh vertex, the polygons it lies on, in polygon order.
    pub map: Vec<Vec<usize>>,
}

/// `mjuu_makenormal`: the unit normal of the triangle `a`, `b`, `c`; where
/// its norm is below `mjEPS`, MuJoCo's (1, 0, 0), divided by that norm all
/// the same.
fn make_normal(a: &Point3<f64>, b: &Point3<f64>, c: &Point3<f64>) -> [f64; 3] {
    let ab = [b.x - a.x, b.y - a.y, b.z - a.z];
    let ac = [c.x - a.x, c.y - a.y, c.z - a.z];
    let mut n = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let nrm = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if nrm < EPS {
        n = [1.0, 0.0, 0.0];
    }
    n.map(|x| x / nrm)
}

/// `PolygonCmp`: whether a face of normal `n1` goes before one of `n2`.
/// Normals within the merge tolerance are of one polygon; others are ordered
/// by their first component apart by more than mjMINVAL, larger first.
fn polygon_less(n1: &[f64; 3], n2: &[f64; 3]) -> bool {
    let dot = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];
    if dot > MERGE_COS {
        return false;
    }
    for (a, b) in n1.iter().zip(n2) {
        if (a - b).abs() > MJ_MINVAL {
            return a > b;
        }
    }
    false
}

/// One polygon as it is merged (`MeshPolygon`): its boundary edges, each
/// with the island it belongs to while faces that share no edge yet are
/// apart, and the normal of its first face.
struct Merging {
    edges: Vec<(usize, usize)>,
    islands: Vec<usize>,
    nisland: usize,
    normal: [f64; 3],
}

impl Merging {
    fn new(verts: &[Point3<f64>], [v1, v2, v3]: [usize; 3]) -> Self {
        Self {
            edges: vec![(v1, v2), (v2, v3), (v3, v1)],
            islands: vec![0, 0, 0],
            nisland: 1,
            normal: make_normal(&verts[v1], &verts[v2], &verts[v3]),
        }
    }

    /// `CombineIslands`: the larger island renumbered as the smaller, the
    /// ones past it down by one.
    fn combine_islands(&mut self, island1: usize, island2: usize) -> usize {
        let (keep, gone) = if island2 < island1 {
            (island2, island1)
        } else {
            (island1, island2)
        };
        for island in &mut self.islands {
            if *island == gone {
                *island = keep;
            } else if *island > gone {
                *island -= 1;
            }
        }
        keep
    }

    /// `InsertFace`: each edge of the face that runs the other way along a
    /// boundary edge joins them (and the islands either side); the face's
    /// other edges join the boundary.
    fn insert_face(&mut self, [v1, v2, v3]: [usize; 3]) {
        let mut island: Option<usize> = None;
        let mut add = [true; 3];
        for (k, (a, b)) in [(v1, v2), (v2, v3), (v3, v1)].into_iter().enumerate() {
            let Some(i) = self.edges.iter().position(|&e| e == (b, a)) else {
                continue;
            };
            let found = self.islands[i];
            island = match island {
                None => Some(found),
                Some(current) if current != found => {
                    self.nisland -= 1;
                    Some(self.combine_islands(current, found))
                }
                same => same,
            };
            add[k] = false;
            self.edges.remove(i);
            self.islands.remove(i);
        }
        let island = island.unwrap_or_else(|| {
            self.nisland += 1;
            self.nisland - 1
        });
        for (k, edge) in [(v1, v2), (v2, v3), (v3, v1)].into_iter().enumerate() {
            if add[k] {
                self.edges.push(edge);
                self.islands.push(island);
            }
        }
    }

    /// `Paths`: each island's boundary traced from its first edge until it
    /// closes. As MuJoCo's, the trace never follows the boundary's first edge
    /// after the start, and an island it cannot close gives no path.
    fn paths(&self) -> Vec<Vec<usize>> {
        if self.edges.len() == 3 {
            return vec![vec![self.edges[0].0, self.edges[1].0, self.edges[2].0]];
        }
        let mut paths = Vec::new();
        for i in 0..self.nisland {
            let Some(start) = self.islands.iter().position(|&island| island == i) else {
                continue; // MuJoCo's "should not occur"
            };
            let mut path = vec![self.edges[start].0, self.edges[start].1];
            let mut next = self.edges[start].1;
            for _ in 0..self.edges.len() {
                let Some(k) = (1..self.edges.len())
                    .find(|&k| self.islands[k] == i && self.edges[k].0 == next)
                else {
                    continue;
                };
                next = self.edges[k].1;
                if next == path[0] {
                    paths.push(path);
                    break;
                }
                path.push(next);
            }
        }
        paths
    }
}

/// `MakePolygons` over `faces`, outward triangles of mesh vertex indices,
/// with `MakePolygonNormals`' normals: each polygon's from its first three
/// vertices. Paths of fewer than three vertices are dropped.
pub fn make_polygons(verts: &[Point3<f64>], faces: &[[usize; 3]]) -> MeshPolygons {
    let mut merging: Vec<Merging> = Vec::new();
    for &face in faces {
        let candidate = Merging::new(verts, face);
        let i = merging.partition_point(|p| polygon_less(&p.normal, &candidate.normal));
        if i < merging.len() && !polygon_less(&candidate.normal, &merging[i].normal) {
            merging[i].insert_face(face);
        } else {
            merging.insert(i, candidate);
        }
    }

    let mut out = MeshPolygons {
        map: vec![Vec::new(); verts.len()],
        ..MeshPolygons::default()
    };
    for path in merging.iter().flat_map(Merging::paths) {
        if path.len() < 3 {
            continue;
        }
        out.normals.push(make_normal(
            &verts[path[0]],
            &verts[path[1]],
            &verts[path[2]],
        ));
        out.paths.push(path);
    }
    for (i, path) in out.paths.iter().enumerate() {
        for &v in path {
            out.map[v].push(i);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::make_polygons;

    /// Faces of one normal merge by island: a fan whose second triangle
    /// joins the first and third after both are in, and a triangle apart in
    /// the same plane, give two polygons, the fan's boundary and the lone
    /// triangle, each vertex on its own.
    #[test]
    fn coplanar_faces_apart_merge_by_island() {
        let verts = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.5, 0.8),
            (0.8, 1.5),
            (0.0, 1.0),
            (3.0, 0.0),
            (4.0, 0.0),
            (3.0, 1.0),
        ]
        .map(|(x, y)| Point3::new(x, y, 0.0));
        let faces = [[0, 1, 2], [0, 3, 4], [5, 6, 7], [0, 2, 3]];
        let polygons = make_polygons(&verts, &faces);
        assert_eq!(polygons.paths, vec![vec![0, 1, 2, 3, 4], vec![5, 6, 7]]);
        assert_eq!(polygons.normals, vec![[0.0, 0.0, 1.0]; 2]);
        let mut map = vec![vec![0]; 5];
        map.extend(vec![vec![1]; 3]);
        assert_eq!(polygons.map, map);
    }
}
