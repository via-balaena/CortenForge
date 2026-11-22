// Ported from Qhull, libqhull_r (poly2_r.c's qh_vertexneighbors), and from
// MuJoCo 3.5.0's mjCMesh::MakeGraph
// (src/user/user_mesh.cc, Apache-2.0, Copyright 2021 DeepMind Technologies
// Limited), which builds its graph from qhull's lists.
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// poly2_r.c: Copyright (c) 1993-2020 The Geometry Center.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! The hull as MuJoCo stores it: `MakeGraph`'s four arrays read from qhull's
//! vertex and facet lists once `qh_triangulate` and `qh_vertexneighbors` ran;
//! and `qh_vertexneighbors`, which the merges call too.

use crate::error::Error;
use crate::set;
use crate::state::{DIM, Qhull};

/// A convex hull as MuJoCo 3.5.0's `mesh_graph` holds it.
///
/// `mjCMesh::MakeGraph` (`user_mesh.cc:1861-2043`) reads it from qhull:
/// qhull's vertices in its vertex-list order, each with its neighbours, and
/// its triangles in its facet-list order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeshGraph {
    /// Each hull vertex's index among the points (`vert_globalid`).
    pub vertex_point: Vec<u32>,
    /// Where each hull vertex's neighbours start in [`Self::edges`]
    /// (`vert_edgeadr`).
    pub vertex_edges: Vec<u32>,
    /// Each hull vertex's neighbours by hull-vertex index, in the order its
    /// facets meet them, each list ended by -1 (`edge_localid`).
    pub edges: Vec<i32>,
    /// Each triangle's points, by index among the points, as `MakeGraph`
    /// orders them: a facet's vertices, the first two swapped where qhull's
    /// facet is top-oriented (`face_globalid`).
    pub faces: Vec<[u32; DIM]>,
}

impl MeshGraph {
    /// The graph as MuJoCo's `mesh_graph` lays it out: the vertex count, the
    /// face count, then `vert_edgeadr`, `vert_globalid`, `edge_localid` and
    /// `face_globalid`.
    #[must_use]
    pub fn to_mujoco(&self) -> Vec<i32> {
        let mut out = Vec::with_capacity(
            2 + 2 * self.vertex_point.len() + self.edges.len() + 3 * self.faces.len(),
        );
        out.push(self.vertex_point.len() as i32);
        out.push(self.faces.len() as i32);
        out.extend(self.vertex_edges.iter().map(|&a| a as i32));
        out.extend(self.vertex_point.iter().map(|&p| p as i32));
        out.extend(self.edges.iter().copied());
        out.extend(self.faces.iter().flatten().map(|&p| p as i32));
        out
    }
}

impl Qhull<'_> {
    /// `qh_vertexneighbors` (`poly2_r.c:3910-3930`): each vertex's facets, in
    /// facet-list order, where no merge made them already.
    pub(crate) fn vertexneighbors(&mut self) {
        if self.VERTEXneighbors {
            return;
        }
        self.vertex_visit += 1;
        let vv = self.vertex_visit;
        for facet in self.facets_from(self.facet_list) {
            if self.f(facet).visible {
                continue;
            }
            for vertex in self.f(facet).vertices.clone() {
                if self.v(vertex).visitid != vv {
                    self.vm(vertex).visitid = vv;
                    self.vm(vertex).neighbors = Vec::with_capacity(DIM);
                }
                self.vm(vertex).neighbors.push(facet);
            }
        }
        self.VERTEXneighbors = true;
    }

    /// MuJoCo's `MakeGraph` after qhull returns (`user_mesh.cc:1898-2027`):
    /// for each vertex its neighbour facets' other vertices, first
    /// appearances, then -1; for each facet its vertices, the first two
    /// swapped where it is top-oriented.
    pub(crate) fn graph(&self) -> Result<MeshGraph, Error> {
        let numpoints = self.points.len();
        let vertices = self.vertices_from(self.vertex_list);
        let facet_list = self.facets_from(self.facet_list);
        let mut vertex_point = Vec::with_capacity(vertices.len());
        let mut vertex_edges = Vec::with_capacity(vertices.len());
        let mut edges: Vec<i32> = Vec::with_capacity(vertices.len() + 3 * facet_list.len());
        // `ok` (`:1915`): a point id out of range ends the vertex or its
        // facet's loop and is reported once the faces are filled (`:2000-2008`).
        let mut ok = true;
        for &vertex in &vertices {
            let pid = self.pointid(vertex);
            if pid < 0 || pid as usize >= numpoints {
                ok = false;
                break;
            }
            vertex_edges.push(edges.len() as u32);
            vertex_point.push(pid as u32);
            let start = edges.len();
            let mut i = 0;
            while let Some(facet) = set::get(&self.v(vertex).neighbors, i) {
                i += 1;
                let mut cnt = 0;
                let mut j = 0;
                while let Some(vertex1) = set::get(&self.f(facet).vertices, j) {
                    j += 1;
                    cnt += 1;
                    let pid1 = self.pointid(vertex1);
                    if pid1 < 0 || pid1 as usize >= numpoints {
                        ok = false;
                        break;
                    }
                    if pid != pid1 && !edges[start..].contains(&pid1) {
                        edges.push(pid1);
                    }
                }
                if cnt != DIM {
                    return Err(Error::Graph {
                        message: "Qhull did not return triangle",
                    });
                }
            }
            edges.push(-1);
        }
        if edges.len() != vertices.len() + DIM * facet_list.len() {
            return Err(Error::Graph {
                message: "Wrong size in convex hull graph",
            });
        }
        let mut faces = Vec::with_capacity(facet_list.len());
        for &facet in &facet_list {
            let ind: [usize; DIM] = if self.f(facet).toporient {
                [1, 0, 2]
            } else {
                [0, 1, 2]
            };
            let mut face = [0_u32; DIM];
            let mut ii = 0;
            while let Some(vertex1) = set::get(&self.f(facet).vertices, ii) {
                if ii >= DIM {
                    return Err(Error::Graph {
                        message: "Qhull did not return triangle",
                    });
                }
                face[ind[ii]] = self.vpoint(vertex1);
                ii += 1;
            }
            faces.push(face);
        }
        if !ok {
            return Err(Error::Graph {
                message: Error::GRAPH_WARNING,
            });
        }
        // Global ids to local ones in the edges.
        for e in &mut edges {
            if *e >= 0 {
                let Some(local) = vertex_point.iter().position(|&p| p as i32 == *e) else {
                    return Err(Error::Graph {
                        message: "Vertex id not found in convex hull",
                    });
                };
                *e = local as i32;
            }
        }
        Ok(MeshGraph {
            vertex_point,
            vertex_edges,
            edges,
            faces,
        })
    }
}
