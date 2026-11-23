// Ported from TriangleMeshDistance (https://github.com/InteractiveComputerGraphics/TriangleMeshDistance,
// `TriangleMeshDistance/include/tmd/TriangleMeshDistance.h` at 2cb643de, the
// commit MuJoCo 3.5.0 builds with), translated to Rust and changed: its signed
// distance, as MuJoCo's compiler asks it for a mesh's octree, over a mesh's
// vertices and faces as slices, its tree sorted by libc++'s `std::sort`
// ([`super::cxx_sort`]); by the CortenForge authors, 2026-11. Its header
// carries this notice and licence:
//
// The MIT License (MIT)
//
// Copyright (c) 2021 José Antonio Fernández Fernández
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//! A point's signed distance to a triangle mesh, as TriangleMeshDistance's
//! `signed_distance` gives it (`TriangleMeshDistance.h:269-313`): the
//! nearest point of the mesh found down a tree of bounding spheres
//! (`_build_tree`, `_query`), Ericson's point–triangle distance
//! (`point_triangle_sq_unsigned`), and its sign from the angle-weighted
//! pseudonormal of the vertex, edge or face it lies on (`_construct`). Each
//! sum and product is in the header's order. The vertex pseudonormals' angles
//! are `acos` of the platform's maths library.

use std::collections::HashMap;

use super::cxx_sort;

type V3 = [f64; 3];

fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// `a * v`, the scalar first (`operator*(const FLOAT_I&, const Vec3r&)`).
fn scale(a: f64, v: V3) -> V3 {
    [a * v[0], a * v[1], a * v[2]]
}

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// `Vec3r::cross`, its middle term as the header writes it.
fn cross(v: V3, u: V3) -> V3 {
    [
        v[1] * u[2] - v[2] * u[1],
        -v[0] * u[2] + v[2] * u[0],
        v[0] * u[1] - v[1] * u[0],
    ]
}

fn norm(v: V3) -> f64 {
    dot(v, v).sqrt()
}

/// `Vec3r::normalized` (and `normalize`): each component over the norm.
fn normalized(v: V3) -> V3 {
    let n = norm(v);
    [v[0] / n, v[1] / n, v[2] / n]
}

/// `std::max(a, b)`: `a` unless `a < b`.
fn max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// `std::min(a, b)`: `a` unless `b < a`.
fn min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// The mesh entity a point is nearest (`NearestEntity`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Entity {
    V0,
    V1,
    V2,
    E01,
    E12,
    E02,
    F,
}

#[derive(Debug, Clone, Copy)]
struct Sphere {
    center: V3,
    radius: f64,
}

const NO_SPHERE: Sphere = Sphere {
    center: [0.0; 3],
    radius: 0.0,
};

/// A node of the tree: a leaf where `left` is `None`, `right` then its
/// triangle.
#[derive(Debug, Clone, Copy)]
struct Node {
    bv_left: Sphere,
    bv_right: Sphere,
    left: Option<usize>,
    right: usize,
}

/// A triangle as the tree is built over it: its vertices and its index.
#[derive(Clone, Copy)]
struct Triangle {
    vertices: [V3; 3],
    id: usize,
}

/// A triangle mesh's distance field (`tmd::TriangleMeshDistance`).
pub struct MeshDistance<'a> {
    vertices: &'a [V3],
    triangles: &'a [[usize; 3]],
    nodes: Vec<Node>,
    pseudonormals_triangles: Vec<V3>,
    pseudonormals_edges: Vec<[V3; 3]>,
    pseudonormals_vertices: Vec<V3>,
}

impl<'a> MeshDistance<'a> {
    /// The field of `triangles` over `vertices` (`construct`, `_construct`).
    /// `triangles` is not empty (the header exits the process where it is)
    /// and names vertices of `vertices`.
    pub fn new(vertices: &'a [V3], triangles: &'a [[usize; 3]]) -> Self {
        let mut tree: Vec<Triangle> = triangles
            .iter()
            .enumerate()
            .map(|(id, t)| Triangle {
                vertices: t.map(|v| vertices[v]),
                id,
            })
            .collect();
        let mut field = Self {
            vertices,
            triangles,
            nodes: vec![Node {
                bv_left: NO_SPHERE,
                bv_right: NO_SPHERE,
                left: None,
                right: 0,
            }],
            pseudonormals_triangles: Vec::with_capacity(triangles.len()),
            pseudonormals_edges: Vec::with_capacity(triangles.len()),
            pseudonormals_vertices: vec![[0.0; 3]; vertices.len()],
        };
        let n = tree.len();
        field.build_tree(0, &mut tree, 0, n);
        field.make_pseudonormals();
        field
    }

    /// `_construct`'s pseudonormals: each face's unit normal; each vertex's
    /// the sum of its faces' weighted by their angle there, made unit; each
    /// edge's the sum of its faces', made unit.
    fn make_pseudonormals(&mut self) {
        let n_vertices = self.vertices.len() as u64;
        let key = |i: usize, j: usize| (i.min(j) as u64) * n_vertices + i.max(j) as u64;
        let mut edge_normals: HashMap<u64, V3> = HashMap::new();
        let mut add_edge_normal = |i: usize, j: usize, normal: V3| {
            edge_normals
                .entry(key(i, j))
                .and_modify(|sum| *sum = add(*sum, normal))
                .or_insert(normal);
        };
        for t in self.triangles {
            let [a, b, c] = t.map(|v| self.vertices[v]);
            let triangle_normal = normalized(cross(sub(b, a), sub(c, a)));
            self.pseudonormals_triangles.push(triangle_normal);
            let alpha_0 = dot(normalized(sub(b, a)), normalized(sub(c, a))).acos();
            let alpha_1 = dot(normalized(sub(a, b)), normalized(sub(c, b))).acos();
            let alpha_2 = dot(normalized(sub(b, c)), normalized(sub(a, c))).acos();
            for (v, alpha) in [(t[0], alpha_0), (t[1], alpha_1), (t[2], alpha_2)] {
                let sum = &mut self.pseudonormals_vertices[v];
                *sum = add(*sum, scale(alpha, triangle_normal));
            }
            add_edge_normal(t[0], t[1], triangle_normal);
            add_edge_normal(t[1], t[2], triangle_normal);
            add_edge_normal(t[0], t[2], triangle_normal);
        }
        for n in &mut self.pseudonormals_vertices {
            *n = normalized(*n);
        }
        for t in self.triangles {
            let edge = |i: usize, j: usize| normalized(edge_normals[&key(i, j)]);
            self.pseudonormals_edges
                .push([edge(t[0], t[1]), edge(t[1], t[2]), edge(t[0], t[2])]);
        }
    }

    /// `_build_tree` of `tree[begin..end]` at `node_id`: its bounding sphere.
    fn build_tree(
        &mut self,
        node_id: usize,
        tree: &mut [Triangle],
        begin: usize,
        end: usize,
    ) -> Sphere {
        let n_triangles = end - begin;
        if n_triangles == 1 {
            self.nodes[node_id].left = None;
            self.nodes[node_id].right = tree[begin].id;
            let [v0, v1, v2] = tree[begin].vertices;
            let s = add(add(v0, v1), v2);
            let center = [s[0] / 3.0, s[1] / 3.0, s[2] / 3.0];
            let radius = max(
                max(norm(sub(v0, center)), norm(sub(v1, center))),
                norm(sub(v2, center)),
            );
            return Sphere { center, radius };
        }
        let mut top = [f64::MIN; 3];
        let mut bottom = [f64::MAX; 3];
        let mut center = [0.0; 3];
        for tri in &tree[begin..end] {
            for p in tri.vertices {
                center = add(center, p);
                for c in 0..3 {
                    top[c] = max(top[c], p[c]);
                    bottom[c] = min(bottom[c], p[c]);
                }
            }
        }
        // `center /= 3*n_triangles`: an int, cast to double.
        #[allow(clippy::cast_precision_loss)] // as the header's cast
        let count = (3 * n_triangles) as f64;
        center = [center[0] / count, center[1] / count, center[2] / count];
        let diagonal = sub(top, bottom);
        // `std::max_element`: the first of the largest.
        let mut split_dim = 0;
        for c in 1..3 {
            if diagonal[split_dim] < diagonal[c] {
                split_dim = c;
            }
        }
        let mut radius_sq: f64 = 0.0;
        for tri in &tree[begin..end] {
            for v in tri.vertices {
                let d = sub(center, v);
                radius_sq = max(radius_sq, dot(d, d));
            }
        }
        let sphere = Sphere {
            center,
            radius: radius_sq.sqrt(),
        };
        cxx_sort::sort(&mut tree[begin..end], &mut |a: &Triangle, b: &Triangle| {
            a.vertices[0][split_dim] < b.vertices[0][split_dim]
        });
        // `(int)(0.5 * (begin + end))`.
        #[allow(
            clippy::cast_precision_loss,
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss
        )]
        let mid = (0.5 * (begin + end) as f64) as usize;
        let left = self.push_node();
        self.nodes[node_id].left = Some(left);
        let bv_left = self.build_tree(left, tree, begin, mid);
        self.nodes[node_id].bv_left = bv_left;
        let right = self.push_node();
        self.nodes[node_id].right = right;
        let bv_right = self.build_tree(right, tree, mid, end);
        self.nodes[node_id].bv_right = bv_right;
        sphere
    }

    fn push_node(&mut self) -> usize {
        self.nodes.push(Node {
            bv_left: NO_SPHERE,
            bv_right: NO_SPHERE,
            left: None,
            right: 0,
        });
        self.nodes.len() - 1
    }

    /// `signed_distance(point).distance`: the distance to the nearest point
    /// of the mesh, negative where the point lies behind the pseudonormal
    /// there.
    pub fn signed_distance(&self, p: V3) -> f64 {
        let mut result = Nearest {
            distance: f64::MAX,
            entity: Entity::F,
            barycentric: [0.0; 3],
            triangle: usize::MAX,
        };
        self.query(&mut result, 0, p);
        let t = self.triangles[result.triangle];
        let pseudonormal = match result.entity {
            Entity::V0 => self.pseudonormals_vertices[t[0]],
            Entity::V1 => self.pseudonormals_vertices[t[1]],
            Entity::V2 => self.pseudonormals_vertices[t[2]],
            Entity::E01 => self.pseudonormals_edges[result.triangle][0],
            Entity::E12 => self.pseudonormals_edges[result.triangle][1],
            Entity::E02 => self.pseudonormals_edges[result.triangle][2],
            Entity::F => self.pseudonormals_triangles[result.triangle],
        };
        let [b0, b1, b2] = result.barycentric;
        let nearest = add(
            add(
                scale(b0, self.vertices[t[0]]),
                scale(b1, self.vertices[t[1]]),
            ),
            scale(b2, self.vertices[t[2]]),
        );
        let u = sub(p, nearest);
        result.distance
            * if dot(u, pseudonormal) >= 0.0 {
                1.0
            } else {
                -1.0
            }
    }

    /// `_query` from `node`: the nearer child's sphere first, each entered
    /// where it is nearer than the nearest point found.
    fn query(&self, result: &mut Nearest, node: usize, p: V3) {
        let node = self.nodes[node];
        let Some(left) = node.left else {
            let t = self.triangles[node.right];
            let (distance_sq, entity, barycentric) = point_triangle_sq_unsigned(
                p,
                self.vertices[t[0]],
                self.vertices[t[1]],
                self.vertices[t[2]],
            );
            if distance_sq < result.distance * result.distance {
                result.entity = entity;
                result.barycentric = barycentric;
                result.distance = distance_sq.sqrt();
                result.triangle = node.right;
            }
            return;
        };
        let d_left = norm(sub(p, node.bv_left.center)) - node.bv_left.radius;
        let d_right = norm(sub(p, node.bv_right.center)) - node.bv_right.radius;
        if d_left < d_right {
            if d_left < result.distance {
                self.query(result, left, p);
            }
            if d_right < result.distance {
                self.query(result, node.right, p);
            }
        } else {
            if d_right < result.distance {
                self.query(result, node.right, p);
            }
            if d_left < result.distance {
                self.query(result, left, p);
            }
        }
    }
}

/// The nearest point found so far (`tmd::Result`, less its point).
struct Nearest {
    distance: f64,
    entity: Entity,
    barycentric: V3,
    triangle: usize,
}

/// `point_triangle_sq_unsigned` (Ericson's, as the header changes it): the
/// squared distance from `p` to the triangle `a b c`, the entity nearest and
/// the nearest point's barycentric coordinates.
#[allow(clippy::many_single_char_names)] // the header's names
fn point_triangle_sq_unsigned(p: V3, a: V3, b: V3, c: V3) -> (f64, Entity, V3) {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let bc = sub(c, b);
    let sq = |q: V3| {
        let d = sub(p, q);
        dot(d, d)
    };

    let snom = dot(sub(p, a), ab);
    let sdenom = dot(sub(p, b), sub(a, b));
    let tnom = dot(sub(p, a), ac);
    let tdenom = dot(sub(p, c), sub(a, c));
    if snom <= 0.0 && tnom <= 0.0 {
        return (sq(a), Entity::V0, [1.0, 0.0, 0.0]);
    }

    let unom = dot(sub(p, b), bc);
    let udenom = dot(sub(p, c), sub(b, c));
    if sdenom <= 0.0 && unom <= 0.0 {
        return (sq(b), Entity::V1, [0.0, 1.0, 0.0]);
    }
    if tdenom <= 0.0 && udenom <= 0.0 {
        return (sq(c), Entity::V2, [0.0, 0.0, 1.0]);
    }

    let n = cross(ab, ac);

    let vc = dot(n, cross(sub(a, p), sub(b, p)));
    if vc <= 0.0 && snom >= 0.0 && sdenom >= 0.0 {
        let arc = snom / (snom + sdenom);
        let bary = [1.0 - arc, arc, 0.0];
        return (
            sq(add(scale(bary[0], a), scale(bary[1], b))),
            Entity::E01,
            bary,
        );
    }

    let va = dot(n, cross(sub(b, p), sub(c, p)));
    if va <= 0.0 && unom >= 0.0 && udenom >= 0.0 {
        let arc = unom / (unom + udenom);
        let bary = [0.0, 1.0 - arc, arc];
        return (
            sq(add(scale(bary[1], b), scale(bary[2], c))),
            Entity::E12,
            bary,
        );
    }

    let vb = dot(n, cross(sub(c, p), sub(a, p)));
    if vb <= 0.0 && tnom >= 0.0 && tdenom >= 0.0 {
        let arc = tnom / (tnom + tdenom);
        let bary = [1.0 - arc, 0.0, arc];
        return (
            sq(add(scale(bary[0], a), scale(bary[2], c))),
            Entity::E02,
            bary,
        );
    }

    let u = va / (va + vb + vc);
    let v = vb / (va + vb + vc);
    let w = 1.0 - u - v;
    let nearest = add(add(scale(u, a), scale(v, b)), scale(w, c));
    (sq(nearest), Entity::F, [u, v, w])
}
