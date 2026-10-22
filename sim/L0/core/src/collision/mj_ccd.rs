//! MuJoCo's native convex collision detection, ported statement for
//! statement: `mjc_ccd` (`engine_collision_gjk.c:2208-2357`) with its GJK
//! (`gjk`, `gjkIntersect` and the distance subalgorithm `subdistance`, `S3D`,
//! `S2D`, `S1D`, `:171-809`), its EPA (the starting polytopes `polytope2`,
//! `polytope3` and `polytope4`, the horizon, `epa` and `epaWitness`,
//! `:815-1436`) and its multi-contact (`multicontact` and `polygonClip`,
//! `:1442-2184`), with the support functions and the object set-up it reads
//! (`engine_collision_convex.c:95-451`, a height field's prism among them,
//! `:709-767`); the entry points that call it, `mjc_Convex` (`:917-1000`) and
//! `mjc_ConvexHField` (`:1185-1363`); and `mjc_PlaneConvex` (`:1042-1138`),
//! which calls no solver, only the support function libccd's callers take
//! (`mjccd_support`, `:495-698`). The arithmetic is plain, as MuJoCo's C
//! compiled without contracting multiply-adds (book C1).
//!
//! It departs from MuJoCo twice. In `S3D`'s fourth face MuJoCo leaves the
//! fourth point's weight as an earlier face set it
//! (`engine_collision_gjk.c:633-643`), which can read a pair apart as an
//! overlap; this sets it to 0, as cf-geometry's `gjk_distance` does (registry
//! `D-GJK-S3D`). And [`mjc_convex_hfield`]'s box tests take a field's highest
//! point as its top where it passes `size[2]` (its doc says why).

use nalgebra::{Matrix3, Point3, Vector3};

use super::mj_primitive::{
    MAX_CONPAIR, Mat9, RawContact, add3, dot3, mat_t_mat, mat_t_vec, mat_vec, normalize3, scl3,
    sub3, transpose,
};
use crate::constraint::impedance::MJ_MINVAL;
use crate::heightfield::HeightFieldData;
use crate::linalg::mju_dot;
use crate::mesh_polygons::MeshPolygons;
use crate::quat::quat_to_mat;
use crate::types::{Data, ENABLE_MULTICCD, GeomType, Model, enabled, make_frame};

/// `mjMINVAL²`.
const MINVAL2: f64 = MJ_MINVAL * MJ_MINVAL;
/// `mjMAXVAL²` (`mjMAXVAL` is 1e10, `mjmodel.h:26`).
const MAXVAL2: f64 = 1e10 * 1e10;
/// `mjMAX_LIMIT`: the distance of a pair found apart beyond the cutoff.
pub(super) const MAX_LIMIT: f64 = f64::MAX;
/// `mjMAX_EPA_ITERATIONS`: the iterations MuJoCo's static polytope storage
/// is sized for.
const MAX_EPA_ITERATIONS: usize = 170;
/// `mjFACE_TOL`: two face normals face each other past this cosine (of 1.6e-3).
const FACE_TOL: f64 = 0.999_998_72;
/// `mjEDGE_TOL`: an edge lies along a face below this sine (of 1.6e-3).
const EDGE_TOL: f64 = 0.001_599_999_31;
/// `mjMESH_HILLCLIMB_MIN`: a mesh with fewer vertices is searched vertex by
/// vertex.
const MESH_HILLCLIMB_MIN: usize = 10;
/// `mjMAX_POLYVERT`: the most corners of a mesh face, or faces at a vertex,
/// the multi-contact reads.
const MAX_POLYVERT: usize = 150;

// ---------------------------------------------------------------------------
// Objects and their support functions
// ---------------------------------------------------------------------------

/// A mesh as MuJoCo's support functions read it.
#[derive(Clone, Copy)]
pub(super) struct CcdMesh<'a> {
    /// Every vertex in the mesh's frame, in its order (`mesh_vert`).
    pub(super) verts: &'a [Point3<f64>],
    /// The convex hull's graph (`mesh_graph`), if the mesh has one.
    pub(super) hull: Option<HullGraph<'a>>,
    /// The mesh's polygons, which the multi-contact clips.
    pub(super) polygons: &'a MeshPolygons,
}

/// A convex hull's vertex graph: each hull vertex's index among the mesh's
/// vertices, and its neighbours among the hull's vertices.
#[derive(Clone, Copy)]
pub(super) struct HullGraph<'a> {
    pub(super) vertex: &'a [usize],
    pub(super) neighbours: &'a [Vec<u32>],
}

/// The support function an object takes (`mjCCDObj::support`).
#[derive(Clone, Copy)]
enum Support<'a> {
    /// `mjc_pointSupport`: a sphere shrunk to its centre.
    Point,
    /// `mjc_lineSupport`: a capsule shrunk to its segment.
    Line,
    Sphere,
    Capsule,
    Ellipsoid,
    Cylinder,
    Box,
    /// `mjc_meshSupport`: every vertex in turn.
    Mesh(&'a [Point3<f64>]),
    /// `mjc_hillclimbSupport`: up the hull's graph.
    HillClimb(&'a [Point3<f64>], HullGraph<'a>),
    /// `mjc_prism_support`: a height field's triangular prism, its three
    /// bottom corners then its three top ones (`mjCCDObj::prism`).
    Prism([[f64; 3]; 6]),
    /// `mjc_flexSupport`: a flex element's vertices (`count` of them, one
    /// for a flex vertex) in the frame the collision takes, and the flex's
    /// radius.
    Flex {
        vertices: [[f64; 3]; 4],
        count: usize,
        radius: f64,
    },
}

/// A geom as `mjc_ccd` reads it (`mjCCDObj`, set up as `mjc_initCCDObj`).
pub(super) struct CcdObj<'a> {
    /// The geom's type; `None` for a flex element or vertex (MuJoCo's
    /// `mjGEOM_FLEX`), which no geom-type test takes.
    geom_type: Option<GeomType>,
    pos: [f64; 3],
    mat: Mat9,
    size: [f64; 3],
    margin: f64,
    support: Support<'a>,
    mesh: Option<CcdMesh<'a>>,
    /// The last support point's vertex on a box or mesh (`vertindex`).
    vertindex: Option<usize>,
    /// The hull vertex the last hill climb ended at, where the next starts
    /// (`meshindex`).
    meshindex: Option<usize>,
}

impl<'a> CcdObj<'a> {
    /// `mjc_initCCDObj` for a geom at `pos`, turned by the row-major `mat`;
    /// `None` for a type `mjc_ccd` does not take here (a plane, a height
    /// field, a signed distance field) or a mesh without vertices.
    pub(super) fn new(
        geom_type: GeomType,
        pos: [f64; 3],
        mat: Mat9,
        size: [f64; 3],
        margin: f64,
        mesh: Option<CcdMesh<'a>>,
    ) -> Option<Self> {
        let support = match geom_type {
            GeomType::Sphere => Support::Sphere,
            GeomType::Capsule => Support::Capsule,
            GeomType::Ellipsoid => Support::Ellipsoid,
            GeomType::Cylinder => Support::Cylinder,
            GeomType::Box => Support::Box,
            GeomType::Mesh => {
                let mesh = mesh.filter(|m| !m.verts.is_empty())?;
                match mesh.hull {
                    Some(hull) if mesh.verts.len() >= MESH_HILLCLIMB_MIN => {
                        Support::HillClimb(mesh.verts, hull)
                    }
                    _ => Support::Mesh(mesh.verts),
                }
            }
            GeomType::Plane | GeomType::Hfield | GeomType::Sdf => return None,
        };
        Some(Self {
            geom_type: Some(geom_type),
            pos,
            mat,
            size,
            margin,
            support,
            mesh,
            vertindex: None,
            meshindex: None,
        })
    }

    /// A flex element or vertex (`mjc_initCCDObj` without a geom, then
    /// `mjc_setCCDObjFlex`, `engine_collision_convex.c:709-775`): its
    /// vertices (one for a vertex) and centre in the collision's frame (an
    /// element's box's centre, `flexelem_aabb`; a vertex's position,
    /// `mjc_center`, `:95-115`), the flex's radius, and the margin, which its
    /// support adds itself.
    pub(super) fn flex(vertices: &[[f64; 3]], center: [f64; 3], radius: f64, margin: f64) -> Self {
        let mut corners = [[0.0; 3]; 4];
        corners[..vertices.len()].copy_from_slice(vertices);
        Self {
            geom_type: None,
            pos: center,
            mat: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            size: [0.0; 3],
            margin,
            support: Support::Flex {
                vertices: corners,
                count: vertices.len(),
                radius,
            },
            mesh: None,
            vertindex: None,
            meshindex: None,
        }
    }

    /// A height field's prism (`mjc_initCCDObj` for a height field, without
    /// a margin), its bottom at `-base` (`mjc_ConvexHField`,
    /// `engine_collision_convex.c:1310`), its corners set by
    /// [`add_prism_vert`](Self::add_prism_vert).
    fn prism(base: f64) -> Self {
        let mut prism = [[0.0; 3]; 6];
        for corner in &mut prism[..3] {
            corner[2] = -base;
        }
        Self {
            geom_type: Some(GeomType::Hfield),
            pos: [0.0; 3],
            mat: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            size: [0.0; 3],
            margin: 0.0,
            support: Support::Prism(prism),
            mesh: None,
            vertindex: None,
            meshindex: None,
        }
    }

    /// `addPrismVert` (`:1165-1182`): the prism's corners each moved one
    /// back, the bottom corner and the top corner at `(x, y)` added last, the
    /// top at height `z`.
    fn add_prism_vert(&mut self, x: f64, y: f64, z: f64) {
        if let Support::Prism(prism) = &mut self.support {
            prism.copy_within(1..3, 0);
            prism.copy_within(4..6, 3);
            prism[2][0] = x;
            prism[5][0] = x;
            prism[2][1] = y;
            prism[5][1] = y;
            prism[5][2] = z;
        }
    }

    /// `mjc_center`: the geom's position; a prism's corners' mean
    /// (`mjc_prism_center`, `:119-127`).
    fn center(&self) -> [f64; 3] {
        if let Support::Prism(prism) = &self.support {
            let mut res = [0.0; 3];
            for corner in prism {
                res = add3(&res, corner);
            }
            return scl3(&res, 1.0 / 6.0);
        }
        self.pos
    }

    /// `localToGlobal`: `mat · local + pos`.
    fn to_global(&self, local: &[f64; 3]) -> [f64; 3] {
        add3(&mat_vec(&self.mat, local), &self.pos)
    }

    /// The support point along the unit direction `dir`, without the margin
    /// (`engine_collision_convex.c:159-430`).
    fn support(&mut self, dir: &[f64; 3]) -> [f64; 3] {
        let (m, pos, size) = (&self.mat, &self.pos, &self.size);
        match self.support {
            Support::Point => self.pos,
            Support::Line => {
                let dot = m[2] * dir[0] + m[5] * dir[1] + m[8] * dir[2];
                let scl = if dot >= 0.0 { size[1] } else { -size[1] };
                [
                    m[2] * scl + pos[0],
                    m[5] * scl + pos[1],
                    m[8] * scl + pos[2],
                ]
            }
            Support::Sphere => {
                let radius = size[0];
                [
                    radius * dir[0] + pos[0],
                    radius * dir[1] + pos[1],
                    radius * dir[2] + pos[2],
                ]
            }
            Support::Capsule => {
                let local_dir = mat_t_vec(m, dir);
                let mut local = scl3(&local_dir, size[0]);
                local[2] += if local_dir[2] >= 0.0 {
                    size[1]
                } else {
                    -size[1]
                };
                self.to_global(&local)
            }
            Support::Ellipsoid => {
                let local_dir = mat_t_vec(m, dir);
                let mut local: [f64; 3] = std::array::from_fn(|i| local_dir[i] * size[i]);
                let norm2 = local[0] * local[0] + local[1] * local[1] + local[2] * local[2];
                if norm2 < MINVAL2 {
                    return [
                        m[0] * size[0] + pos[0],
                        m[3] * size[0] + pos[1],
                        m[6] * size[0] + pos[2],
                    ];
                }
                let norm_inv = 1.0 / norm2.sqrt();
                for (x, s) in local.iter_mut().zip(size) {
                    *x *= norm_inv * s;
                }
                self.to_global(&local)
            }
            Support::Cylinder => {
                let local_dir = mat_t_vec(m, dir);
                let n2 = local_dir[0] * local_dir[0] + local_dir[1] * local_dir[1];
                let scl = if n2 >= MINVAL2 {
                    size[0] / n2.sqrt()
                } else {
                    0.0
                };
                let z = if local_dir[2] >= 0.0 {
                    size[1]
                } else {
                    -size[1]
                };
                self.to_global(&[scl * local_dir[0], scl * local_dir[1], z])
            }
            Support::Box => {
                let local_dir = mat_t_vec(m, dir);
                let local: [f64; 3] = std::array::from_fn(|i| {
                    if local_dir[i] >= 0.0 {
                        size[i]
                    } else {
                        -size[i]
                    }
                });
                // The corner's index, a bit per positive coordinate.
                self.vertindex = Some(
                    usize::from(local[0] > 0.0)
                        | usize::from(local[1] > 0.0) << 1
                        | usize::from(local[2] > 0.0) << 2,
                );
                self.to_global(&local)
            }
            Support::Mesh(verts) => self.mesh_support(verts, dir),
            Support::HillClimb(verts, hull) => self.hill_climb_support(verts, hull, dir),
            // `mjc_prism_support` (`:434-451`): the best of the three bottom
            // corners where `dir` points down, else of the three top ones.
            Support::Prism(prism) => {
                let start = if dir[2] < 0.0 { 0 } else { 3 };
                let mut best = start;
                let mut max = dot3(&prism[start], dir);
                for (i, corner) in prism.iter().enumerate().take(start + 3).skip(start + 1) {
                    let tmp = dot3(corner, dir);
                    if tmp > max {
                        best = i;
                        max = tmp;
                    }
                }
                prism[best]
            }
            // `mjc_flexSupport` (`engine_collision_convex.c:455-491`): the
            // vertex furthest along `dir` (the first of equals), pushed out by
            // the flex's radius and half the margin in one step.
            Support::Flex {
                vertices,
                count,
                radius,
            } => {
                let mut best = vertices[0];
                let mut max = dot3(&best, dir);
                for vertex in &vertices[1..count] {
                    let dot = dot3(vertex, dir);
                    if dot > max {
                        max = dot;
                        best = *vertex;
                    }
                }
                let scl = radius + 0.5 * self.margin;
                [
                    best[0] + dir[0] * scl,
                    best[1] + dir[1] * scl,
                    best[2] + dir[2] * scl,
                ]
            }
        }
    }

    /// `mjc_meshSupport`: the vertex farthest along `dir`, the last support
    /// vertex first, then every vertex in order; a later vertex wins only by
    /// a larger product.
    fn mesh_support(&mut self, verts: &[Point3<f64>], dir: &[f64; 3]) -> [f64; 3] {
        let local_dir = mat_t_vec(&self.mat, dir);
        let mut max = -f64::from(f32::MAX);
        let mut imax = 0;
        if let Some(i) = self.vertindex {
            imax = i;
            max = dot_point(&local_dir, &verts[i]);
        }
        for (i, v) in verts.iter().enumerate() {
            let vdot = dot_point(&local_dir, v);
            if vdot > max {
                max = vdot;
                imax = i;
            }
        }
        self.vertindex = Some(imax);
        let v = &verts[imax];
        self.to_global(&[v.x, v.y, v.z])
    }

    /// `mjc_hillclimbSupport`: from the hull vertex the last climb ended at,
    /// to the best neighbour of each vertex in turn until none is better than
    /// the best seen. The vertex a pass starts from is not weighed on its own.
    fn hill_climb_support(
        &mut self,
        verts: &[Point3<f64>],
        hull: HullGraph<'_>,
        dir: &[f64; 3],
    ) -> [f64; 3] {
        let local_dir = mat_t_vec(&self.mat, dir);
        let mut max = -f64::from(f32::MAX);
        let mut prev = None;
        let mut imax = self.meshindex.unwrap_or(0);
        while prev != Some(imax) {
            prev = Some(imax);
            for &sub in &hull.neighbours[imax] {
                let sub = sub as usize;
                let vdot = dot_point(&local_dir, &verts[hull.vertex[sub]]);
                if vdot > max {
                    max = vdot;
                    imax = sub;
                }
            }
        }
        self.meshindex = Some(imax);
        let g = hull.vertex[imax];
        self.vertindex = Some(g);
        let v = &verts[g];
        self.to_global(&[v.x, v.y, v.z])
    }

    /// The sphere-and-capsule set-up of `mjc_ccd` (`:2229-2258`): a sphere
    /// becomes its centre and a capsule its segment, without a margin; the
    /// radius and half the margin, added back after GJK.
    fn shrink(&mut self) -> f64 {
        let support = match self.geom_type {
            Some(GeomType::Sphere) => Support::Point,
            Some(GeomType::Capsule) => Support::Line,
            _ => return 0.0,
        };
        let full_margin = self.size[0] + 0.5 * self.margin;
        self.support = support;
        self.margin = 0.0;
        full_margin
    }

    /// `mjccd_support` (`engine_collision_convex.c:495-698`), the support
    /// function libccd's callers take, for the two types `mjc_PlaneConvex`
    /// gives it here: an ellipsoid, its direction scaled by its sizes,
    /// normalised (`mju_normalize3`) and scaled again; a mesh, up its hull's
    /// graph from the hull vertex the last search ended at (else the first),
    /// weighing that vertex first, else every vertex in turn. Either is pushed
    /// out by half the margin along the direction. Where the graph's search
    /// ends is `meshindex`: a hull vertex, or without a graph a mesh vertex.
    fn ccd_support(&mut self, dir: &[f64; 3]) -> [f64; 3] {
        let local_dir = mat_t_vec(&self.mat, dir);
        let mut res = if let Some(mesh) = self.mesh {
            self.ccd_mesh_vertex(&mesh, &local_dir)
        } else {
            let mut res: [f64; 3] = std::array::from_fn(|i| local_dir[i] * self.size[i]);
            normalize3(&mut res);
            std::array::from_fn(|i| res[i] * self.size[i])
        };
        for (r, d) in res.iter_mut().zip(local_dir) {
            *r += d * self.margin / 2.0;
        }
        add3(&mat_vec(&self.mat, &res), &self.pos)
    }

    /// The mesh vertex of [`ccd_support`](Self::ccd_support) along the local
    /// direction `local_dir`, or the origin where none is found (MuJoCo warns
    /// "mesh_support could not find support vertex": every product NaN).
    fn ccd_mesh_vertex(&mut self, mesh: &CcdMesh<'_>, local_dir: &[f64; 3]) -> [f64; 3] {
        let best = match mesh.hull {
            None => {
                let mut tmp = -1e10;
                let mut best = None;
                for (i, v) in mesh.verts.iter().enumerate() {
                    let vdot = dot_point(local_dir, v);
                    if vdot > tmp {
                        tmp = vdot;
                        best = Some(i);
                    }
                }
                self.meshindex = best;
                best
            }
            Some(hull) => {
                let mut best = self.meshindex.unwrap_or(0);
                let mut tmp = dot_point(local_dir, &mesh.verts[hull.vertex[best]]);
                let mut change = true;
                while change {
                    change = false;
                    for &sub in &hull.neighbours[best] {
                        let sub = sub as usize;
                        let vdot = dot_point(local_dir, &mesh.verts[hull.vertex[sub]]);
                        if vdot > tmp {
                            tmp = vdot;
                            best = sub;
                            change = true;
                        }
                    }
                }
                self.meshindex = Some(best);
                Some(hull.vertex[best])
            }
        };
        best.map_or([0.0; 3], |i| {
            let v = &mesh.verts[i];
            [v.x, v.y, v.z]
        })
    }
}

/// `dot3f`: a direction against a vertex.
fn dot_point(a: &[f64; 3], v: &Point3<f64>) -> f64 {
    a[0] * v.x + a[1] * v.y + a[2] * v.z
}

// ---------------------------------------------------------------------------
// Configuration and status
// ---------------------------------------------------------------------------

/// What `mjc_ccd` is asked (`mjCCDConfig`).
#[derive(Clone, Copy)]
pub(super) struct CcdConfig {
    pub(super) max_iterations: usize,
    pub(super) tolerance: f64,
    /// The contacts to recover; 0 asks only for the distance.
    pub(super) max_contacts: usize,
    /// The largest distance to recover; at 0 a pair apart reads `MAX_LIMIT`.
    pub(super) dist_cutoff: f64,
}

/// How EPA's starting polytope came out (`mjEPAStatus`); MuJoCo's numbers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum EpaStatus {
    NoContact = -1,
    Success = 0,
    P2Nonconvex = 2,
    P3BadNormal = 4,
    P3InvalidV4 = 5,
    P3InvalidV5 = 6,
    P3MissingOrigin = 7,
    P3OriginOnFace = 8,
    P4MissingOrigin = 9,
}

/// A point of the Minkowski difference and the two support points it is
/// made of (`Vertex`).
#[derive(Clone, Copy, Default)]
struct Vertex {
    /// `vert1 − vert2`.
    vert: [f64; 3],
    vert1: [f64; 3],
    vert2: [f64; 3],
    /// The support points' box corner or mesh vertex (`index1`, `index2`).
    index1: Option<usize>,
    index2: Option<usize>,
}

/// What `mjc_ccd` found (`mjCCDStatus`).
pub(super) struct CcdStatus {
    /// The distance, negative in penetration; `MAX_LIMIT` for a pair apart
    /// past the cutoff.
    pub(super) dist: f64,
    /// The witness points on each geom; the first `nx` are set.
    pub(super) x1: [[f64; 3]; MAX_CONPAIR],
    pub(super) x2: [[f64; 3]; MAX_CONPAIR],
    pub(super) nx: usize,
    max_iterations: usize,
    tolerance: f64,
    max_contacts: usize,
    dist_cutoff: f64,
    pub(super) gjk_iterations: usize,
    pub(super) epa_iterations: usize,
    pub(super) epa_status: EpaStatus,
    simplex: [Vertex; 4],
    pub(super) nsimplex: usize,
}

impl CcdStatus {
    fn new(config: &CcdConfig) -> Self {
        Self {
            dist: 0.0,
            x1: [[0.0; 3]; MAX_CONPAIR],
            x2: [[0.0; 3]; MAX_CONPAIR],
            nx: 0,
            max_iterations: config.max_iterations,
            tolerance: config.tolerance,
            max_contacts: config.max_contacts,
            dist_cutoff: config.dist_cutoff,
            gjk_iterations: 0,
            epa_iterations: 0,
            epa_status: EpaStatus::NoContact,
            simplex: [Vertex::default(); 4],
            nsimplex: 0,
        }
    }

    /// GJK's exits for a pair apart: no simplex, no witness, `MAX_LIMIT`.
    const fn apart(&mut self, k: usize) {
        self.gjk_iterations = k;
        self.nsimplex = 0;
        self.nx = 0;
        self.dist = MAX_LIMIT;
    }
}

// ---------------------------------------------------------------------------
// mjc_ccd
// ---------------------------------------------------------------------------

/// `mjc_ccd`: the distance between two convex geoms, negative in
/// penetration, with the witness points. GJK runs from the geoms' centres; a
/// sphere or capsule first runs as its centre or segment and is inflated
/// back by its radius where that leaves the pair more than `tolerance`
/// apart. Within the tolerance, and with more than one simplex point, EPA
/// recovers the penetration from a polytope grown from GJK's simplex, and
/// with more than one contact asked, the multi-contact clips the two
/// touching faces.
pub(super) fn mjc_ccd(config: &CcdConfig, o1: &mut CcdObj<'_>, o2: &mut CcdObj<'_>) -> CcdStatus {
    let mut status = CcdStatus::new(config);
    status.x1[0] = o1.center();
    status.x2[0] = o2.center();

    let shrinks =
        |o: &CcdObj<'_>| matches!(o.geom_type, Some(GeomType::Sphere | GeomType::Capsule));
    if shrinks(o1) || shrinks(o2) {
        let (support1, support2) = (o1.support, o2.support);
        let (margin1, margin2) = (o1.margin, o2.margin);
        let full_margin1 = o1.shrink();
        let full_margin2 = o2.shrink();

        status.dist_cutoff += full_margin1 + full_margin2;
        gjk(&mut status, o1, o2);
        status.dist_cutoff = config.dist_cutoff;

        o1.margin = margin1;
        o2.margin = margin2;
        o1.support = support1;
        o2.support = support2;

        // Shallow penetration: inflate the contact.
        if status.dist > status.tolerance {
            inflate(&mut status, full_margin1, full_margin2);
            if status.dist > status.dist_cutoff {
                status.dist = MAX_LIMIT;
            }
            return status;
        }
        if config.max_contacts == 0 {
            status.nx = 0;
            status.dist = 0.0;
            return status;
        }

        // Deep penetration: GJK and EPA again on the whole geoms.
        status.gjk_iterations = 0;
        status.x1[0] = o1.center();
        status.x2[0] = o2.center();
    }
    gjk(&mut status, o1, o2);

    if config.max_contacts == 0 {
        return status;
    }
    if status.dist <= config.tolerance && status.nsimplex > 1 {
        status.dist = 0.0; // touching, until EPA says more
        let mut pt = Polytope::new(config.max_iterations);
        let ret = match status.nsimplex {
            2 => polytope2(&mut pt, &mut status, o1, o2),
            3 => polytope3(&mut pt, &status, o1, o2),
            _ => polytope4(&mut pt, &mut status, o1, o2),
        };
        status.epa_status = ret;
        if ret == EpaStatus::Success
            && let Some(face) = epa(&mut status, &mut pt, o1, o2)
            && config.max_contacts > 1
        {
            multicontact(&pt, face, &mut status, o1, o2);
        }
    }
    status
}

/// `inflate`: the first witness pair moved apart along their difference by
/// each geom's radius and half margin, the distance by both.
fn inflate(status: &mut CcdStatus, margin1: f64, margin2: f64) {
    let mut n = sub3(&status.x2[0], &status.x1[0]);
    normalize3(&mut n);
    if margin1 != 0.0 {
        for (x, ni) in status.x1[0].iter_mut().zip(n) {
            *x += margin1 * ni;
        }
    }
    if margin2 != 0.0 {
        for (x, ni) in status.x2[0].iter_mut().zip(n) {
            *x -= margin2 * ni;
        }
    }
    status.dist -= margin1 + margin2;
}

// ---------------------------------------------------------------------------
// GJK
// ---------------------------------------------------------------------------

/// `discreteGeoms`: both geoms boxes, meshes or height fields without a
/// margin, for which GJK converges in finitely many steps.
fn discrete_geoms(o1: &CcdObj<'_>, o2: &CcdObj<'_>) -> bool {
    if o1.margin != 0.0 || o2.margin != 0.0 {
        return false;
    }
    let discrete =
        |t: Option<GeomType>| matches!(t, Some(GeomType::Mesh | GeomType::Box | GeomType::Hfield));
    discrete(o1.geom_type) && discrete(o2.geom_type)
}

/// `gjk`: from the difference of the first witness points, each step takes
/// the Minkowski difference's support point along `−x` and stops once the
/// Frank–Wolfe gap `x · (x − s)` is below `tolerance² / 2` (0 for two
/// discrete geoms); else `x` becomes the simplex's point nearest the origin.
/// Without a distance cutoff a separating plane ends it apart, and a
/// tetrahedron hands it to `gjkIntersect` once; with a cutoff, a support
/// plane farther than the cutoff ends it apart.
// MuJoCo's loop, one function.
#[allow(clippy::too_many_lines)]
fn gjk(status: &mut CcdStatus, o1: &mut CcdObj<'_>, o2: &mut CcdObj<'_>) {
    let get_dist = status.dist_cutoff > 0.0;
    let mut backup_gjk = !get_dist;
    let mut n = 0;
    let mut k = 0;
    let kmax = status.max_iterations;
    let mut x_k = sub3(&status.x1[0], &status.x2[0]);
    let mut lambda = [1.0, 0.0, 0.0, 0.0];
    let cutoff2 = status.dist_cutoff * status.dist_cutoff;
    let epsilon = if discrete_geoms(o1, o2) {
        0.0
    } else {
        0.5 * status.tolerance * status.tolerance
    };
    // MuJoCo leaves `x_norm` unset (`engine_collision_gjk.c:186`) and reads it
    // when no iteration runs (a `max_iterations` of 0); here it is 0.
    let mut x_norm = 0.0;

    while k < kmax {
        // MuJoCo keeps the squared norm where it stops here.
        x_norm = dot3(&x_k, &x_k);
        if x_norm < MINVAL2 {
            break;
        }
        x_norm = x_norm.sqrt();
        status.simplex[n] = gjk_support(o1, o2, &x_k, x_norm);
        let s_k = status.simplex[n].vert;

        if dot3(&x_k, &sub3(&x_k, &s_k)) < epsilon {
            if k == 0 {
                n = 1;
            }
            break;
        }

        if !get_dist {
            if dot3(&x_k, &s_k) > 0.0 {
                status.apart(k);
                return;
            }
        } else if status.dist_cutoff < MAX_LIMIT {
            let vs = dot3(&x_k, &s_k);
            let vv = dot3(&x_k, &x_k);
            if dot3(&x_k, &s_k) > 0.0 && vs * vs / vv >= cutoff2 {
                status.apart(k);
                return;
            }
        }

        if n == 3 && backup_gjk {
            status.gjk_iterations = k;
            if let Some(contact) = gjk_intersect(status, o1, o2) {
                status.nx = 0;
                status.dist = if contact { 0.0 } else { MAX_LIMIT };
                return;
            }
            k = status.gjk_iterations;
            backup_gjk = false;
        }

        lambda = subdistance(n + 1, &status.simplex);
        n = 0;
        for i in 0..4 {
            if lambda[i] == 0.0 {
                continue;
            }
            status.simplex[n] = status.simplex[i];
            lambda[n] = lambda[i];
            n += 1;
        }
        // MuJoCo's "should not occur".
        if n < 1 {
            status.apart(k);
            return;
        }

        let verts = status.simplex.map(|v| v.vert);
        let x_next = lincomb(&lambda[..n], &verts);
        if equal3(&x_next, &x_k) {
            break;
        }
        x_k = x_next;
        // A tetrahedron about the origin.
        if n == 4 {
            x_norm = 0.0;
            break;
        }
        k += 1;
    }

    if n > 0 {
        status.x1[0] = lincomb(&lambda[..n], &status.simplex.map(|v| v.vert1));
        status.x2[0] = lincomb(&lambda[..n], &status.simplex.map(|v| v.vert2));
    }
    status.nx = 1;
    status.gjk_iterations = k;
    status.nsimplex = n;
    status.dist = x_norm;
}

/// `support`: the Minkowski difference's support point along `dir`, each
/// geom's support point pushed out by half its margin (a flex's support
/// adds its own, `engine_collision_gjk.c:300-326`).
fn support(o1: &mut CcdObj<'_>, o2: &mut CcdObj<'_>, dir: &[f64; 3], dir_neg: &[f64; 3]) -> Vertex {
    let mut vert1 = o1.support(dir);
    if o1.margin > 0.0 && o1.geom_type.is_some() {
        let margin = 0.5 * o1.margin;
        for (v, d) in vert1.iter_mut().zip(dir) {
            *v += d * margin;
        }
    }
    let mut vert2 = o2.support(dir_neg);
    if o2.margin > 0.0 && o2.geom_type.is_some() {
        let margin = 0.5 * o2.margin;
        for (v, d) in vert2.iter_mut().zip(dir_neg) {
            *v += d * margin;
        }
    }
    Vertex {
        vert: sub3(&vert1, &vert2),
        vert1,
        vert2,
        index1: o1.vertindex,
        index2: o2.vertindex,
    }
}

/// `gjkSupport`: the support point along `−x_k / |x_k|`, the direction
/// scaled by the norm's reciprocal.
fn gjk_support(o1: &mut CcdObj<'_>, o2: &mut CcdObj<'_>, x_k: &[f64; 3], x_norm: f64) -> Vertex {
    let dir_neg = scl3(x_k, 1.0 / x_norm);
    let dir = scl3(&dir_neg, -1.0);
    support(o1, o2, &dir, &dir_neg)
}

/// `signedDistance`: a face's unit normal and its plane's distance from the
/// origin; `MAX_LIMIT`, and the normal unscaled, where it cannot be scaled.
fn signed_distance(v1: &Vertex, v2: &Vertex, v3: &Vertex) -> (f64, [f64; 3]) {
    let normal = cross3(&sub3(&v3.vert, &v1.vert), &sub3(&v2.vert, &v1.vert));
    let norm2 = dot3(&normal, &normal);
    if norm2 > MINVAL2 && norm2 < MAXVAL2 {
        let normal = scl3(&normal, 1.0 / norm2.sqrt());
        return (dot3(&normal, &v1.vert), normal);
    }
    (MAX_LIMIT, normal)
}

/// `gjkIntersect`: whether the origin is inside the Minkowski difference,
/// from GJK's tetrahedron, by replacing the vertex opposite the face nearest
/// the origin; `None` where it cannot tell. On contact the tetrahedron is
/// GJK's simplex.
// MuJoCo's ternaries: a NaN distance takes the second face, as in C.
#[allow(clippy::bool_to_int_with_if)]
fn gjk_intersect(status: &mut CcdStatus, o1: &mut CcdObj<'_>, o2: &mut CcdObj<'_>) -> Option<bool> {
    let mut simplex = status.simplex;
    let mut s = [0, 1, 2, 3];
    let mut k = status.gjk_iterations;
    let kmax = status.max_iterations;
    while k < kmax {
        let faces = [(2, 1, 3), (0, 2, 3), (1, 0, 3), (0, 1, 2)];
        let mut dist = [0.0; 4];
        let mut normals = [[0.0; 3]; 4];
        for (f, (a, b, c)) in faces.into_iter().enumerate() {
            (dist[f], normals[f]) = signed_distance(&simplex[s[a]], &simplex[s[b]], &simplex[s[c]]);
        }

        // The origin on a face's plane: no convergence.
        if dist.contains(&0.0) {
            status.gjk_iterations = k;
            return None;
        }

        let i = if dist[0] < dist[1] { 0 } else { 1 };
        let j = if dist[2] < dist[3] { 2 } else { 3 };
        let index = if dist[i] < dist[j] { i } else { j };

        // The origin inside: EPA takes the tetrahedron.
        if dist[index] > 0.0 {
            status.nsimplex = 4;
            status.simplex = s.map(|q| simplex[q]);
            status.gjk_iterations = k;
            return Some(true);
        }

        let dir = normals[index];
        simplex[s[index]] = support(o1, o2, &dir, &dir.map(|d| -d));
        if dot3(&dir, &simplex[s[index]].vert) < 0.0 {
            status.nsimplex = 0;
            status.gjk_iterations = k;
            return Some(false);
        }

        // Keep the tetrahedron's orientation.
        s.swap((index + 1) & 3, (index + 2) & 3);
        k += 1;
    }
    status.gjk_iterations = k;
    None
}

/// `lincomb`: `Σ coef[i] · points[i]` over `coef`, summed in order.
fn lincomb(coef: &[f64], points: &[[f64; 3]]) -> [f64; 3] {
    std::array::from_fn(|k| {
        let mut sum = coef[0] * points[0][k];
        for (c, p) in coef.iter().zip(points).skip(1) {
            sum += c * p[k];
        }
        sum
    })
}

/// `projectOriginPlane`: the origin projected onto the plane of three
/// points, its normal tried as three cross products in turn; `None` where
/// one is zero.
fn project_origin_plane(v1: &[f64; 3], v2: &[f64; 3], v3: &[f64; 3]) -> Option<[f64; 3]> {
    let diff21 = sub3(v2, v1);
    let diff31 = sub3(v3, v1);
    let diff32 = sub3(v3, v2);

    for (n, v) in [
        (cross3(&diff32, &diff21), v2),
        (cross3(&diff21, &diff31), v1),
    ] {
        let nv = dot3(&n, v);
        let nn = dot3(&n, &n);
        if nn == 0.0 {
            return None;
        }
        if nv != 0.0 && nn > MJ_MINVAL {
            return Some(scl3(&n, nv / nn));
        }
    }
    let n = cross3(&diff31, &diff32);
    let nv = dot3(&n, v3);
    let nn = dot3(&n, &n);
    Some(scl3(&n, nv / nn))
}

/// `projectOriginLine`: the origin projected onto the line of two points.
fn project_origin_line(v1: &[f64; 3], v2: &[f64; 3]) -> [f64; 3] {
    let diff = sub3(v2, v1);
    let scl = -(dot3(v2, &diff) / dot3(&diff, &diff));
    [
        v2[0] + scl * diff[0],
        v2[1] + scl * diff[1],
        v2[2] + scl * diff[2],
    ]
}

/// `sameSign2`: both positive or both negative.
fn same_sign(a: f64, b: f64) -> bool {
    (a > 0.0 && b > 0.0) || (a < 0.0 && b < 0.0)
}

/// `subdistance` (after Montanari et al., ACM Transactions on Graphics
/// 2017): the barycentric coordinates, over the simplex's first `n` points,
/// of its point nearest the origin; a point the nearest point does not weigh
/// takes exactly 0.
fn subdistance(n: usize, simplex: &[Vertex; 4]) -> [f64; 4] {
    let [s1, s2, s3, s4] = simplex.map(|v| v.vert);
    let mut lambda = [0.0; 4];
    match n {
        4 => lambda = s3d(&s1, &s2, &s3, &s4),
        3 => lambda[..3].copy_from_slice(&s2d(&s1, &s2, &s3)),
        2 => lambda[..2].copy_from_slice(&s1d(&s1, &s2)),
        _ => lambda[0] = 1.0,
    }
    lambda
}

/// `S3D`: a tetrahedron's signed volumes; where the origin is outside, the
/// nearest of the faces it is outside, the first such face whatever its
/// distance.
// MuJoCo's cofactor names C41..C44.
#[allow(clippy::similar_names)]
fn s3d(s1: &[f64; 3], s2: &[f64; 3], s3: &[f64; 3], s4: &[f64; 3]) -> [f64; 4] {
    let c41 = -det3(s2, s3, s4);
    let c42 = det3(s1, s3, s4);
    let c43 = -det3(s1, s2, s4);
    let c44 = det3(s1, s2, s3);
    let m_det = c41 + c42 + c43 + c44;
    let comp = [
        same_sign(m_det, c41),
        same_sign(m_det, c42),
        same_sign(m_det, c43),
        same_sign(m_det, c44),
    ];
    if comp.iter().all(|&inside| inside) {
        return [c41 / m_det, c42 / m_det, c43 / m_det, c44 / m_det];
    }

    // The face opposite each point, and where its weights go.
    let faces: [([&[f64; 3]; 3], [usize; 3]); 4] = [
        ([s2, s3, s4], [1, 2, 3]),
        ([s1, s3, s4], [0, 2, 3]),
        ([s1, s2, s4], [0, 1, 3]),
        ([s1, s2, s3], [0, 1, 2]),
    ];
    let mut lambda = [0.0; 4];
    let mut dmin = MAX_LIMIT;
    for (i, ([p1, p2, p3], slots)) in faces.iter().enumerate() {
        if comp[i] {
            continue;
        }
        let face = s2d(p1, p2, p3);
        let x = lincomb(&face, &[**p1, **p2, **p3]);
        let d = dot3(&x, &x);
        if i == 0 || d < dmin {
            // MuJoCo's fourth face leaves the fourth point's weight as an
            // earlier face set it (`:633-643`); ours sets it to 0 (registry
            // `D-GJK-S3D`).
            if i < 3 || !mujoco_stale_weight() {
                lambda[i] = 0.0;
            }
            for (slot, w) in slots.iter().zip(face) {
                lambda[*slot] = w;
            }
            dmin = d;
        }
    }
    lambda
}

#[cfg(test)]
thread_local! {
    /// Set by the golden test, to compute `S3D` as MuJoCo does.
    static STALE_S3D: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Whether `S3D`'s fourth face keeps MuJoCo's stale weight: only where the
/// golden test asks, to show that it is the one difference from MuJoCo.
#[cfg(test)]
fn mujoco_stale_weight() -> bool {
    STALE_S3D.with(std::cell::Cell::get)
}

#[cfg(not(test))]
const fn mujoco_stale_weight() -> bool {
    false
}

/// The three 2×2 minors `M_14`, `M_24`, `M_34` of a triangle's 4×4 matrix
/// (`S2D` and `triAffineCoord`), as MuJoCo writes the sums.
fn triangle_minors(s1: &[f64; 3], s2: &[f64; 3], s3: &[f64; 3]) -> [f64; 3] {
    [
        s2[1] * s3[2] - s2[2] * s3[1] - s1[1] * s3[2] + s1[2] * s3[1] + s1[1] * s2[2]
            - s1[2] * s2[1],
        s2[0] * s3[2] - s2[2] * s3[0] - s1[0] * s3[2] + s1[2] * s3[0] + s1[0] * s2[2]
            - s1[2] * s2[0],
        s2[0] * s3[1] - s2[1] * s3[0] - s1[0] * s3[1] + s1[1] * s3[0] + s1[0] * s2[1]
            - s1[1] * s2[0],
    ]
}

/// The largest minor and the two axes the triangle shadows most: the axis
/// of the first minor whose magnitude is at least the others' is dropped.
fn shadow_axes(minors: &[f64; 3]) -> (f64, usize, usize) {
    let [mu1, mu2, mu3] = minors.map(f64::abs);
    if mu1 >= mu2 && mu1 >= mu3 {
        (minors[0], 1, 2)
    } else if mu2 >= mu3 {
        (minors[1], 0, 2)
    } else {
        (minors[2], 0, 1)
    }
}

/// The signed areas `C31`, `C32`, `C33` of the point `p` with each pair of a
/// triangle's corners, in the plane of axes `x` and `y`, as MuJoCo writes
/// the sums.
fn triangle_cofactors(
    s1: &[f64; 3],
    s2: &[f64; 3],
    s3: &[f64; 3],
    p: &[f64; 3],
    x: usize,
    y: usize,
) -> [f64; 3] {
    [
        p[x] * s2[y] + p[y] * s3[x] + s2[x] * s3[y] - p[x] * s3[y] - p[y] * s2[x] - s3[x] * s2[y],
        p[x] * s3[y] + p[y] * s1[x] + s3[x] * s1[y] - p[x] * s1[y] - p[y] * s3[x] - s1[x] * s3[y],
        p[x] * s1[y] + p[y] * s2[x] + s1[x] * s2[y] - p[x] * s2[y] - p[y] * s1[x] - s2[x] * s1[y],
    ]
}

/// `S2D`: a triangle's signed areas in the plane it shadows most; where the
/// origin's projection is outside, the nearest of the edges it is outside.
fn s2d(s1: &[f64; 3], s2: &[f64; 3], s3: &[f64; 3]) -> [f64; 3] {
    let Some(p_o) = project_origin_plane(s1, s2, s3) else {
        let [l1, l2] = s1d(s1, s2);
        return [l1, l2, 0.0];
    };
    let (m_max, x, y) = shadow_axes(&triangle_minors(s1, s2, s3));
    let c = triangle_cofactors(s1, s2, s3, &p_o, x, y);
    let comp = c.map(|ci| same_sign(m_max, ci));
    if comp.iter().all(|&inside| inside) {
        return c.map(|ci| ci / m_max);
    }

    // The edge opposite each corner, and where its weights go.
    let edges: [([&[f64; 3]; 2], [usize; 2]); 3] =
        [([s2, s3], [1, 2]), ([s1, s3], [0, 2]), ([s1, s2], [0, 1])];
    let mut lambda = [0.0; 3];
    let mut dmin = MAX_LIMIT;
    for (i, ([p1, p2], slots)) in edges.iter().enumerate() {
        if comp[i] {
            continue;
        }
        let edge = s1d(p1, p2);
        let x = lincomb(&edge, &[**p1, **p2]);
        let d = dot3(&x, &x);
        if i == 0 || d < dmin {
            lambda[i] = 0.0;
            for (slot, w) in slots.iter().zip(edge) {
                lambda[*slot] = w;
            }
            dmin = d;
        }
    }
    lambda
}

/// `S1D`: a segment's weights by the origin's projection along the axis the
/// segment shadows most (the last of the largest); the second end alone
/// where the projection is outside.
fn s1d(s1: &[f64; 3], s2: &[f64; 3]) -> [f64; 2] {
    let p_o = project_origin_line(s1, s2);
    let mut mu_max = s1[0] - s2[0];
    let mut index = 0;
    for i in 1..3 {
        let mu = s1[i] - s2[i];
        if mu.abs() >= mu_max.abs() {
            mu_max = mu;
            index = i;
        }
    }
    let c1 = p_o[index] - s2[index];
    let c2 = s1[index] - p_o[index];
    if same_sign(mu_max, c1) && same_sign(mu_max, c2) {
        [c1 / mu_max, c2 / mu_max]
    } else {
        [0.0, 1.0]
    }
}

// ---------------------------------------------------------------------------
// EPA
// ---------------------------------------------------------------------------

/// Where a face is in the polytope's map of candidate faces (`Face::index`).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Slot {
    /// Not in the map (`-1`).
    Out,
    /// Deleted from the polytope (`-2`).
    Deleted,
    /// At this position in the map.
    At(usize),
}

/// A face of the polytope (`Face`).
#[derive(Clone, Copy)]
struct Face {
    /// Its corners, indices into the polytope's vertices.
    verts: [usize; 3],
    /// The faces across its edges `[v1, v2]`, `[v2, v3]`, `[v3, v1]`.
    adj: [usize; 3],
    /// The origin projected onto its plane.
    v: [f64; 3],
    /// `|v|²`.
    dist2: f64,
    slot: Slot,
}

/// EPA's polytope (`Polytope`): its vertices, its faces, the map of faces
/// EPA may expand next, and the horizon of the latest support point.
struct Polytope {
    verts: Vec<Vertex>,
    faces: Vec<Face>,
    max_faces: usize,
    map: Vec<usize>,
    /// The horizon's faces and, for each, the edge on the horizon.
    horizon: Vec<(usize, usize)>,
}

impl Polytope {
    /// MuJoCo's storage: room for the faces of `mjMAX_EPA_ITERATIONS`
    /// iterations, or of `max_iterations` past that (`:2304-2333`).
    fn new(max_iterations: usize) -> Self {
        let max_faces = 6 * max_iterations.max(MAX_EPA_ITERATIONS);
        Self {
            verts: Vec::with_capacity(5 + max_iterations),
            faces: Vec::with_capacity(6 * max_iterations),
            max_faces,
            map: Vec::new(),
            horizon: Vec::new(),
        }
    }

    /// `insertVertex`.
    fn insert_vertex(&mut self, v: Vertex) -> usize {
        self.verts.push(v);
        self.verts.len() - 1
    }

    /// `deleteFace`: out of the map, the map's last face moved into its
    /// place, and marked deleted.
    fn delete_face(&mut self, f: usize) {
        if let Slot::At(i) = self.faces[f].slot
            && let Some(last) = self.map.pop()
            && i < self.map.len()
        {
            self.map[i] = last;
            self.faces[last].slot = Slot::At(i);
        }
        self.faces[f].slot = Slot::Deleted;
    }

    /// `attachFace`: a face on three vertices, its squared distance from the
    /// origin; 0 where its plane has no normal.
    fn attach_face(&mut self, v: [usize; 3], adj: [usize; 3]) -> f64 {
        let projection = project_origin_plane(
            &self.verts[v[2]].vert,
            &self.verts[v[1]].vert,
            &self.verts[v[0]].vert,
        );
        let mut face = Face {
            verts: v,
            adj,
            v: [0.0; 3],
            dist2: 0.0,
            slot: Slot::Out,
        };
        let Some(p) = projection else {
            self.faces.push(face);
            return 0.0;
        };
        face.v = p;
        face.dist2 = dot3(&p, &p);
        self.faces.push(face);
        face.dist2
    }

    /// Every face into the map, in order (the starting polytopes).
    fn map_all(&mut self) {
        for (i, face) in self.faces.iter_mut().enumerate() {
            self.map.push(i);
            face.slot = Slot::At(i);
        }
    }

    /// `horizon`: delete the face `f` and, from its three edges, every face
    /// `w` sees, collecting the edges where seen faces meet unseen ones.
    fn horizon(&mut self, f: usize, w: &[f64; 3]) {
        self.delete_face(f);
        let face = self.faces[f];
        for (k, (adj, vertex)) in [(0, 1), (1, 2), (2, 0)].into_iter().enumerate() {
            let adj = face.adj[adj];
            let adj_edge = get_edge(&self.faces[adj], face.verts[vertex]);
            // MuJoCo checks the first neighbour without asking if it is deleted.
            if (k == 0 || self.faces[adj].slot != Slot::Deleted)
                && !self.horizon_rec(adj, adj_edge, w)
            {
                self.horizon.push((adj, adj_edge));
            }
        }
    }

    /// `horizonRec`: whether `w` sees the face (by more than mjMINVAL); a
    /// seen face is deleted and its two other edges searched in turn.
    fn horizon_rec(&mut self, f: usize, e: usize, w: &[f64; 3]) -> bool {
        let face = self.faces[f];
        if dot3(&face.v, w) - face.dist2 > MJ_MINVAL {
            self.delete_face(f);
            for k in 1..3 {
                let i = (e + k) % 3;
                let adj = face.adj[i];
                if self.faces[adj].slot != Slot::Deleted {
                    let adj_edge = get_edge(&self.faces[adj], face.verts[(i + 1) % 3]);
                    if !self.horizon_rec(adj, adj_edge, w) {
                        self.horizon.push((adj, adj_edge));
                    }
                }
            }
            return true;
        }
        false
    }
}

/// `getEdge`: the edge of a face that starts at `vertex`.
fn get_edge(face: &Face, vertex: usize) -> usize {
    if face.verts[0] == vertex {
        0
    } else if face.verts[1] == vertex {
        1
    } else {
        2
    }
}

/// `epaSupport`: a new vertex of the polytope along `d / dnorm` (along x
/// where `dnorm` is within mjMINVAL of 0).
fn epa_support(
    pt: &mut Polytope,
    o1: &mut CcdObj<'_>,
    o2: &mut CcdObj<'_>,
    d: &[f64; 3],
    dnorm: f64,
) -> usize {
    let (dir, dir_neg) = if dnorm > MJ_MINVAL {
        let dir = d.map(|x| x / dnorm);
        (dir, scl3(&dir, -1.0))
    } else {
        ([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0])
    };
    pt.insert_vertex(support(o1, o2, &dir, &dir_neg))
}

/// `replaceSimplex3`: GJK's simplex becomes one face of the polytope, and
/// the polytope empties.
fn replace_simplex3(pt: &mut Polytope, status: &mut CcdStatus, v: [usize; 3]) {
    status.nsimplex = 3;
    for (s, &i) in status.simplex.iter_mut().zip(&v) {
        *s = pt.verts[i];
    }
    pt.faces.clear();
    pt.verts.clear();
    pt.map.clear();
}

/// `sameSide`: the origin and `p3` on the same side of the plane of `p0`,
/// `p1`, `p2`.
fn same_side(p0: &[f64; 3], p1: &[f64; 3], p2: &[f64; 3], p3: &[f64; 3]) -> bool {
    let n = cross3(&sub3(p1, p0), &sub3(p2, p0));
    let dot1 = dot3(&n, &sub3(p3, p0));
    let dot2 = dot3(&n, &scl3(p0, -1.0));
    same_sign(dot1, dot2)
}

/// `testTetra`: the origin inside the tetrahedron.
fn test_tetra(p0: &[f64; 3], p1: &[f64; 3], p2: &[f64; 3], p3: &[f64; 3]) -> bool {
    same_side(p0, p1, p2, p3)
        && same_side(p1, p2, p3, p0)
        && same_side(p2, p3, p0, p1)
        && same_side(p3, p0, p1, p2)
}

/// `rotmat`: the turn by 120 degrees about `axis`, row-major.
// MuJoCo's names for the axis' components.
#[allow(clippy::many_single_char_names)]
fn rotmat(axis: &[f64; 3]) -> Mat9 {
    let n = norm3(axis);
    let (u1, u2, u3) = (axis[0] / n, axis[1] / n, axis[2] / n);
    let sin = 0.866_025_403_78;
    let cos = -0.5;
    [
        cos + u1 * u1 * (1.0 - cos),
        u1 * u2 * (1.0 - cos) - u3 * sin,
        u1 * u3 * (1.0 - cos) + u2 * sin,
        u2 * u1 * (1.0 - cos) + u3 * sin,
        cos + u2 * u2 * (1.0 - cos),
        u2 * u3 * (1.0 - cos) - u1 * sin,
        u1 * u3 * (1.0 - cos) - u2 * sin,
        u2 * u3 * (1.0 - cos) + u1 * sin,
        cos + u3 * u3 * (1.0 - cos),
    ]
}

/// `rayTriangle`: whether the ray from `v1` through `v2` meets the triangle
/// `v3`, `v4`, `v5` (the three volumes of one sign, zero included).
fn ray_triangle(v1: &[f64; 3], v2: &[f64; 3], v3: &[f64; 3], v4: &[f64; 3], v5: &[f64; 3]) -> bool {
    let diff12 = sub3(v2, v1);
    let diff13 = sub3(v3, v1);
    let diff14 = sub3(v4, v1);
    let diff15 = sub3(v5, v1);
    let vol1 = det3(&diff13, &diff14, &diff12);
    let vol2 = det3(&diff14, &diff15, &diff12);
    let vol3 = det3(&diff15, &diff13, &diff12);
    (vol1 >= 0.0 && vol2 >= 0.0 && vol3 >= 0.0) || (vol1 <= 0.0 && vol2 <= 0.0 && vol3 <= 0.0)
}

/// `polytope2`: from GJK's segment, a hexahedron of the segment's ends and
/// three support points along directions 120 degrees apart about it; where
/// a face passes within mjMINVAL of the origin, that face as a triangle
/// (`polytope3`).
fn polytope2(
    pt: &mut Polytope,
    status: &mut CcdStatus,
    o1: &mut CcdObj<'_>,
    o2: &mut CcdObj<'_>,
) -> EpaStatus {
    let v1 = status.simplex[0].vert;
    let v2 = status.simplex[1].vert;
    let diff = sub3(&v2, &v1);

    // The axis of the smallest component, so the cross product is largest.
    let mut value = MAX_LIMIT;
    let mut index = 0;
    for (i, d) in diff.iter().enumerate() {
        if d.abs() < value {
            value = d.abs();
            index = i;
        }
    }
    let mut e = [0.0; 3];
    e[index] = 1.0;
    let d1 = cross3(&e, &diff);
    let r = rotmat(&diff);
    let d2 = mat_vec(&r, &d1);
    let d3 = mat_vec(&r, &d2);

    let v1i = pt.insert_vertex(status.simplex[0]);
    let v2i = pt.insert_vertex(status.simplex[1]);
    let v3i = epa_support(pt, o1, o2, &d1, norm3(&d1));
    let v4i = epa_support(pt, o1, o2, &d2, norm3(&d2));
    let v5i = epa_support(pt, o1, o2, &d3, norm3(&d3));
    let (v3, v4, v5) = (pt.verts[v3i].vert, pt.verts[v4i].vert, pt.verts[v5i].vert);

    for (v, adj) in [
        ([v1i, v3i, v4i], [1, 3, 2]),
        ([v1i, v5i, v3i], [2, 4, 0]),
        ([v1i, v4i, v5i], [0, 5, 1]),
        ([v2i, v4i, v3i], [5, 0, 4]),
        ([v2i, v3i, v5i], [3, 1, 5]),
        ([v2i, v5i, v4i], [4, 2, 3]),
    ] {
        if pt.attach_face(v, adj) < MINVAL2 {
            replace_simplex3(pt, status, v);
            return polytope3(pt, status, o1, o2);
        }
    }
    if !ray_triangle(&v1, &v2, &v3, &v4, &v5) {
        return EpaStatus::P2Nonconvex;
    }
    pt.map_all();
    EpaStatus::Success
}

/// `triAffineCoord`: the affine coordinates of `p` on a triangle, in the
/// plane it shadows most.
fn tri_affine_coord(v1: &[f64; 3], v2: &[f64; 3], v3: &[f64; 3], p: &[f64; 3]) -> [f64; 3] {
    let (m_max, x, y) = shadow_axes(&triangle_minors(v1, v2, v3));
    triangle_cofactors(v1, v2, v3, p, x, y).map(|c| c / m_max)
}

/// `triPointIntersect`: `p` on the triangle, within mjMINVAL.
fn tri_point_intersect(v1: &[f64; 3], v2: &[f64; 3], v3: &[f64; 3], p: &[f64; 3]) -> bool {
    let lambda = tri_affine_coord(v1, v2, v3, p);
    if lambda.iter().any(|&l| l < 0.0) {
        return false;
    }
    let pr: [f64; 3] =
        std::array::from_fn(|i| v1[i] * lambda[0] + v2[i] * lambda[1] + v3[i] * lambda[2]);
    norm3(&sub3(&pr, p)) < MJ_MINVAL
}

/// `polytope3`: from GJK's triangle, a hexahedron of its corners and the
/// support points along its two normals.
fn polytope3(
    pt: &mut Polytope,
    status: &CcdStatus,
    o1: &mut CcdObj<'_>,
    o2: &mut CcdObj<'_>,
) -> EpaStatus {
    let v1 = status.simplex[0].vert;
    let v2 = status.simplex[1].vert;
    let v3 = status.simplex[2].vert;

    let n = cross3(&sub3(&v2, &v1), &sub3(&v3, &v1));
    let n_norm = norm3(&n);
    if n_norm < MJ_MINVAL {
        return EpaStatus::P3BadNormal;
    }
    let n_neg = scl3(&n, -1.0);

    let v1i = pt.insert_vertex(status.simplex[0]);
    let v2i = pt.insert_vertex(status.simplex[1]);
    let v3i = pt.insert_vertex(status.simplex[2]);
    let v5i = epa_support(pt, o1, o2, &n_neg, n_norm);
    let v4i = epa_support(pt, o1, o2, &n, n_norm);
    let v4 = pt.verts[v4i].vert;
    let v5 = pt.verts[v5i].vert;

    if tri_point_intersect(&v1, &v2, &v3, &v4) {
        return EpaStatus::P3InvalidV4;
    }
    if tri_point_intersect(&v1, &v2, &v3, &v5) {
        return EpaStatus::P3InvalidV5;
    }
    if status.dist > 10.0 * MJ_MINVAL
        && !test_tetra(&v1, &v2, &v3, &v4)
        && !test_tetra(&v1, &v2, &v3, &v5)
    {
        return EpaStatus::P3MissingOrigin;
    }

    for (v, adj) in [
        ([v4i, v1i, v2i], [1, 3, 2]),
        ([v4i, v3i, v1i], [2, 4, 0]),
        ([v4i, v2i, v3i], [0, 5, 1]),
        ([v5i, v2i, v1i], [5, 0, 4]),
        ([v5i, v1i, v3i], [3, 1, 5]),
        ([v5i, v3i, v2i], [4, 2, 3]),
    ] {
        if pt.attach_face(v, adj) < MINVAL2 {
            return EpaStatus::P3OriginOnFace;
        }
    }
    pt.map_all();
    EpaStatus::Success
}

/// `polytope4`: GJK's tetrahedron; where a face passes within mjMINVAL of
/// the origin, that face as a triangle (`polytope3`).
fn polytope4(
    pt: &mut Polytope,
    status: &mut CcdStatus,
    o1: &mut CcdObj<'_>,
    o2: &mut CcdObj<'_>,
) -> EpaStatus {
    let v1 = pt.insert_vertex(status.simplex[0]);
    let v2 = pt.insert_vertex(status.simplex[1]);
    let v3 = pt.insert_vertex(status.simplex[2]);
    let v4 = pt.insert_vertex(status.simplex[3]);

    for (v, adj) in [
        ([v1, v2, v3], [1, 3, 2]),
        ([v1, v4, v2], [2, 3, 0]),
        ([v1, v3, v4], [0, 3, 1]),
        ([v4, v3, v2], [2, 0, 1]),
    ] {
        if pt.attach_face(v, adj) < MINVAL2 {
            replace_simplex3(pt, status, v);
            return polytope3(pt, status, o1, o2);
        }
    }
    let [p1, p2, p3, p4] = [v1, v2, v3, v4].map(|i| pt.verts[i].vert);
    if !test_tetra(&p1, &p2, &p3, &p4) {
        return EpaStatus::P4MissingOrigin;
    }
    pt.map_all();
    EpaStatus::Success
}

/// `epaWitness`: the witness points on each geom at the face's projection
/// of the origin, and the signed distance between them.
fn epa_witness(pt: &Polytope, f: usize) -> (f64, [f64; 3], [f64; 3]) {
    let face = &pt.faces[f];
    let [a, b, c] = face.verts.map(|i| pt.verts[i]);
    let lambda = tri_affine_coord(&a.vert, &b.vert, &c.vert, &face.v);
    let x1 = lincomb(&lambda, &[a.vert1, b.vert1, c.vert1]);
    let x2 = lincomb(&lambda, &[a.vert2, b.vert2, c.vert2]);
    (-face.dist2.sqrt(), x1, x2)
}

/// `epa`: the face of the polytope nearest the origin, expanded through the
/// support point along its normal until the support plane is within
/// `tolerance` (mjMINVAL for two discrete geoms, which also stop on a
/// repeated support point) of it; the face, and its witness points in the
/// status. `None` where the polytope cannot grow.
fn epa(
    status: &mut CcdStatus,
    pt: &mut Polytope,
    o1: &mut CcdObj<'_>,
    o2: &mut CcdObj<'_>,
) -> Option<usize> {
    let mut upper = MAX_LIMIT;
    let mut upper2 = MAX_LIMIT;
    let mut face: Option<usize> = None;
    let discrete = discrete_geoms(o1, o2);
    let tolerance = if discrete {
        MJ_MINVAL
    } else {
        status.tolerance
    };

    let kmax = status.max_iterations;
    let mut k = 0;
    while k < kmax {
        let pface = face;

        // The face nearest the origin, a lower bound on the depth.
        let mut lower2 = MAX_LIMIT;
        for &f in &pt.map {
            if pt.faces[f].dist2 < lower2 {
                face = Some(f);
                lower2 = pt.faces[f].dist2;
            }
        }
        let f = match face {
            Some(f) if lower2 <= upper2 => f,
            _ => {
                face = pface;
                break;
            }
        };
        if lower2 <= 0.0 {
            log::warn!("EPA: origin lies on affine hull of face");
            break;
        }

        let lower = lower2.sqrt();
        let fv = pt.faces[f].v;
        let wi = epa_support(pt, o1, o2, &fv, lower);
        let w = pt.verts[wi];
        let upper_k = dot3(&fv, &w.vert) / lower;
        if upper_k < upper {
            upper = upper_k;
            upper2 = upper * upper;
        }
        if upper - lower < tolerance {
            break;
        }

        if discrete
            && pt.verts[..wi]
                .iter()
                .any(|v| v.index1 == w.index1 && v.index2 == w.index2)
        {
            break;
        }

        pt.horizon(f, &w.vert);
        // At least one face was deleted, so fewer than three edges is a
        // numerical failure.
        if pt.horizon.len() < 3 {
            face = None;
            break;
        }

        let nfaces = pt.faces.len();
        let nedges = pt.horizon.len();
        if nedges > pt.max_faces - nfaces {
            log::warn!("EPA: out of memory for faces on expanding polytope");
            break;
        }

        // A face from w to each horizon edge, each adjacent to the next.
        for i in 0..nedges {
            let cur = nfaces + i;
            let prev = if i == 0 { nfaces + nedges - 1 } else { cur - 1 };
            let next = nfaces + (i + 1) % nedges;
            let (hor_index, hor_edge) = pt.horizon[i];
            let hor_face = &mut pt.faces[hor_index];
            let v1 = hor_face.verts[hor_edge];
            let v2 = hor_face.verts[(hor_edge + 1) % 3];
            hor_face.adj[hor_edge] = cur;
            let dist2 = pt.attach_face([wi, v2, v1], [prev, hor_index, next]);
            if dist2 == 0.0 {
                face = None;
                break;
            }
            if dist2 >= lower2 && dist2 <= upper2 {
                let slot = pt.map.len();
                pt.map.push(cur);
                pt.faces[cur].slot = Slot::At(slot);
            }
        }
        pt.horizon.clear();

        if pt.map.is_empty() || face.is_none() {
            break;
        }
        k += 1;
    }

    status.epa_iterations = k;
    if let Some(f) = face {
        let (dist, x1, x2) = epa_witness(pt, f);
        status.dist = dist;
        status.x1[0] = x1;
        status.x2[0] = x2;
        status.nx = 1;
    } else {
        status.nx = 0;
        status.dist = 0.0;
    }
    face
}

// ---------------------------------------------------------------------------
// Multi-contact
// ---------------------------------------------------------------------------

/// `area4`: a quadrilateral's area.
// MuJoCo's names for the corners.
#[allow(clippy::many_single_char_names)]
fn area4(a: &[f64; 3], b: &[f64; 3], c: &[f64; 3], d: &[f64; 3]) -> f64 {
    let ad = sub3(d, a);
    let db = sub3(b, d);
    let bc = sub3(c, b);
    let ca = sub3(a, c);
    let g = add3(&cross3(&ad, &db), &cross3(&bc, &ca));
    0.5 * norm3(&g)
}

/// `polygonQuad`: the polygon's corners of a convex quadrilateral of
/// largest area, found by MuJoCo's rotating search.
// MuJoCo's names for the corners.
#[allow(clippy::many_single_char_names)]
fn polygon_quad(polygon: &[[f64; 3]]) -> [usize; 4] {
    let n = polygon.len();
    let next = |i: usize| if i == n - 1 { 0 } else { i + 1 };
    let area = |a: usize, b: usize, c: usize, d: usize| {
        area4(&polygon[a], &polygon[b], &polygon[c], &polygon[d])
    };
    let (mut b, mut c, mut d) = (1, 2, 3);
    let mut res = [0, 1, 2, 3];
    let mut m = area(0, 1, 2, 3);
    for a in 0..n {
        loop {
            let m_next = area(a, b, c, next(d));
            if m_next <= m {
                break;
            }
            m = m_next;
            d = next(d);
            res = [a, b, c, d];
            loop {
                let m_next = area(a, b, next(c), d);
                if m_next <= m {
                    break;
                }
                m = m_next;
                c = next(c);
                res = [a, b, c, d];
            }
            loop {
                let m_next = area(a, next(b), c, d);
                if m_next <= m {
                    break;
                }
                m = m_next;
                b = next(b);
                res = [a, b, c, d];
            }
        }
        if b == a {
            b = next(b);
            if c == b {
                c = next(c);
                if d == c {
                    d = next(d);
                }
            }
        }
    }
    res
}

/// `planeNormal`: the normal of the plane through the edge `v1`–`v2`
/// perpendicular to the face of normal `n`, and its offset.
fn plane_normal(v1: &[f64; 3], v2: &[f64; 3], n: &[f64; 3]) -> ([f64; 3], f64) {
    let v3 = add3(v1, n);
    let res = cross3(&sub3(v2, v1), &sub3(&v3, v1));
    let d = dot3(&res, v1);
    (res, d)
}

/// `halfspace`: `p` on the inner side of the plane through `a` of normal
/// `n`, within mjMINVAL.
fn halfspace(a: &[f64; 3], n: &[f64; 3], p: &[f64; 3]) -> bool {
    dot3(&sub3(p, a), n) > -MJ_MINVAL
}

/// `planeIntersect`: where the segment `a`–`b` crosses the plane, as the
/// parameter along it (`MAX_LIMIT` when parallel), and the point when the
/// parameter is within [0, 1].
fn plane_intersect(pn: &[f64; 3], pd: f64, a: &[f64; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
    let ab = sub3(b, a);
    let temp = dot3(pn, &ab);
    if temp == 0.0 {
        return None;
    }
    let t = (pd - dot3(pn, a)) / temp;
    (0.0..=1.0)
        .contains(&t)
        .then(|| std::array::from_fn(|i| a[i] + t * ab[i]))
}

/// `polygonClip`: `face2` clipped by the planes through `face1`'s edges
/// perpendicular to its normal `n`; the clipped polygon's corners become the
/// witness points on geom 2, less `dir` on geom 1. With at most four
/// contacts asked, more than four corners are pruned to the largest
/// quadrilateral; past `mjMAXCONPAIR` the first are kept; else repeated
/// corners are dropped.
fn polygon_clip(
    status: &mut CcdStatus,
    face1: &[[f64; 3]],
    face2: &[[f64; 3]],
    n: &[f64; 3],
    dir: &[f64; 3],
) {
    let nface1 = face1.len();
    if nface1 < 3 {
        return;
    }
    let planes: Vec<([f64; 3], f64)> = (0..nface1)
        .map(|i| plane_normal(&face1[i], &face1[(i + 1) % nface1], n))
        .collect();

    let mut polygon = face2.to_vec();
    for (e, (pn, pd)) in planes.iter().enumerate() {
        let np = polygon.len();
        let mut clipped = Vec::with_capacity(2 * np);
        for i in 0..np {
            let p = polygon[i];
            let q = polygon[if i < np - 1 { i + 1 } else { 0 }];
            let inside1 = halfspace(&face1[e], pn, &p);
            let inside2 = halfspace(&face1[e], pn, &q);
            if !inside1 && !inside2 {
                continue;
            }
            if inside1 && inside2 {
                clipped.push(q);
                continue;
            }
            if let Some(point) = plane_intersect(pn, *pd, &p, &q) {
                clipped.push(point);
            }
            if inside2 {
                clipped.push(q);
            }
        }
        polygon = clipped;
    }
    if polygon.is_empty() {
        return;
    }

    if status.max_contacts < 5 && polygon.len() > 4 {
        status.nx = 4;
        for (i, corner) in polygon_quad(&polygon).into_iter().enumerate() {
            status.x2[i] = polygon[corner];
            status.x1[i] = sub3(&status.x2[i], dir);
        }
        return;
    }
    if polygon.len() > MAX_CONPAIR {
        status.nx = MAX_CONPAIR;
        for (i, corner) in polygon.iter().take(MAX_CONPAIR).enumerate() {
            status.x2[i] = *corner;
            status.x1[i] = sub3(corner, dir);
        }
        return;
    }
    let mut k = 0;
    for corner in &polygon {
        if status.x2[..k].iter().any(|x| equal3(x, corner)) {
            continue;
        }
        status.x2[k] = *corner;
        status.x1[k] = sub3(corner, dir);
        k += 1;
    }
    status.nx = k;
}

/// `globalcoord`: `mat · l (+ pos)`.
fn global_coord(mat: &Mat9, pos: Option<&[f64; 3]>, l: [f64; 3]) -> [f64; 3] {
    let res = mat_vec(mat, &l);
    pos.map_or(res, |p| add3(&res, p))
}

/// `boxNormals2`: the box face whose normal is within the face tolerance of
/// `n`, and its index.
fn box_normals2(mat: &Mat9, n: &[f64; 3]) -> Option<([f64; 3], usize)> {
    const NORMALS: [[f64; 3]; 6] = [
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
    ];
    let local_n = mat_t_vec(mat, n);
    let local_n = scl3(&local_n, 1.0 / dot3(&local_n, &local_n).sqrt());
    NORMALS
        .iter()
        .position(|normal| dot3(&local_n, normal) > FACE_TOL)
        .map(|i| (global_coord(mat, None, NORMALS[i]), i))
}

/// The sign MuJoCo's `boxNormals` gives an axis over the corners `v`: +1 if
/// every corner has the axis' bit, −1 if none has, else 0.
fn axis_sign(v: &[usize], bit: usize) -> i8 {
    i8::from(v.iter().all(|&i| i & bit != 0)) - i8::from(v.iter().all(|&i| i & bit == 0))
}

/// `boxNormals`: the normals of the box faces a feature of `dim` corners can
/// touch, and their indices (`+x, −x, +y, −y, +z, −z`); a feature on no one
/// face takes the face aligned with `dir`, if any.
// MuJoCo's names for the axes' signs.
#[allow(clippy::many_single_char_names)]
fn box_normals(mat: &Mat9, dim: usize, v: [usize; 3], dir: &[f64; 3]) -> Vec<([f64; 3], usize)> {
    let fallback = || box_normals2(mat, dir).into_iter().collect();
    match dim {
        3 => {
            let [x, y, z] = [1, 2, 4].map(|bit| axis_sign(&v, bit));
            let normal = global_coord(mat, None, [x, y, z].map(f64::from));
            let mut ind = [0, 2, 4]
                .into_iter()
                .zip([x, y, z])
                .filter(|&(_, s)| s != 0)
                .map(|(i, _)| i);
            let first = ind.next();
            match (first, ind.next()) {
                (Some(i), None) => vec![(normal, i + usize::from(x + y + z == -1))],
                _ => fallback(),
            }
        }
        2 => {
            let [x, y, z] = [1, 2, 4].map(|bit| axis_sign(&v[..2], bit));
            let mut res = Vec::new();
            for (axis, s) in [x, y, z].into_iter().enumerate() {
                if s != 0 {
                    let mut l = [0.0; 3];
                    l[axis] = f64::from(s);
                    res.push((global_coord(mat, None, l), 2 * axis + usize::from(s < 0)));
                }
            }
            if res.len() == 2 { res } else { fallback() }
        }
        1 => (0..3)
            .map(|axis| {
                let positive = v[0] & (1 << axis) != 0;
                let mut l = [0.0; 3];
                l[axis] = if positive { 1.0 } else { -1.0 };
                (
                    global_coord(mat, None, l),
                    2 * axis + usize::from(!positive),
                )
            })
            .collect(),
        _ => Vec::new(),
    }
}

/// `boxEdgeNormals`: the unit directions of the box edges a feature of `dim`
/// corners lies on (the edge itself; or the three from a corner), each with
/// its other end.
// MuJoCo's names for the corner's coordinates.
#[allow(clippy::many_single_char_names)]
fn box_edge_normals(
    o: &CcdObj<'_>,
    dim: usize,
    v1: &[f64; 3],
    v2: &[f64; 3],
    v1i: usize,
) -> Vec<([f64; 3], [f64; 3])> {
    let unit = |end: [f64; 3]| {
        let mut d = sub3(&end, v1);
        normalize3(&mut d);
        (d, end)
    };
    match dim {
        2 => vec![unit(*v2)],
        1 => {
            let s = &o.size;
            let x = if v1i & 1 != 0 { s[0] } else { -s[0] };
            let y = if v1i & 2 != 0 { s[1] } else { -s[1] };
            let z = if v1i & 4 != 0 { s[2] } else { -s[2] };
            [[-x, y, z], [x, -y, z], [x, y, -z]]
                .into_iter()
                .map(|l| unit(global_coord(&o.mat, Some(&o.pos), l)))
                .collect()
        }
        _ => Vec::new(),
    }
}

/// `boxFace`: a box face's four corners, from its index.
fn box_face(o: &CcdObj<'_>, idx: usize) -> Vec<[f64; 3]> {
    let [sx, sy, sz] = o.size;
    let corners: [[f64; 3]; 4] = match idx {
        0 => [[sx, sy, sz], [sx, sy, -sz], [sx, -sy, -sz], [sx, -sy, sz]],
        1 => [
            [-sx, sy, -sz],
            [-sx, sy, sz],
            [-sx, -sy, sz],
            [-sx, -sy, -sz],
        ],
        2 => [[-sx, sy, -sz], [sx, sy, -sz], [sx, sy, sz], [-sx, sy, sz]],
        3 => [
            [-sx, -sy, sz],
            [sx, -sy, sz],
            [sx, -sy, -sz],
            [-sx, -sy, -sz],
        ],
        4 => [[-sx, sy, sz], [sx, sy, sz], [sx, -sy, sz], [-sx, -sy, sz]],
        5 => [
            [sx, sy, -sz],
            [-sx, sy, -sz],
            [-sx, -sy, -sz],
            [sx, -sy, -sz],
        ],
        _ => return Vec::new(),
    };
    corners
        .into_iter()
        .map(|l| global_coord(&o.mat, Some(&o.pos), l))
        .collect()
}

/// `alignedFaces`: the first normals of `v` and `w` that face each other
/// within the face tolerance.
fn aligned_faces(v: &[[f64; 3]], w: &[[f64; 3]]) -> Option<(usize, usize)> {
    for (i, vi) in v.iter().enumerate() {
        for (j, wj) in w.iter().enumerate() {
            if dot3(vi, wj) < -FACE_TOL {
                return Some((i, j));
            }
        }
    }
    None
}

/// `alignedFaceEdge`: the first edge and face, by face, with the edge along
/// the face within the edge tolerance: (edge, face).
fn aligned_face_edge(edge: &[[f64; 3]], face: &[[f64; 3]]) -> Option<(usize, usize)> {
    for (i, fi) in face.iter().enumerate() {
        for (j, ej) in edge.iter().enumerate() {
            if dot3(ej, fi).abs() < EDGE_TOL {
                return Some((j, i));
            }
        }
    }
    None
}

/// `simplexDim`: how many distinct corners a face of the polytope has on one
/// geom (1, 2 or 3); a repeated second corner gives its place to the third.
fn simplex_dim(index: &mut [Option<usize>; 3], vert: &mut [[f64; 3]; 3]) -> usize {
    if index[0] != index[1] {
        return if index[2] == index[0] || index[2] == index[1] {
            2
        } else {
            3
        };
    }
    if index[0] != index[2] {
        index[1] = index[2];
        vert[1] = vert[2];
        return 2;
    }
    1
}

/// `intersect`: up to two values `a` and `b` share, in `a`'s order.
fn intersect(a: &[usize], b: &[usize]) -> Vec<usize> {
    let mut res = Vec::with_capacity(2);
    for x in a {
        for y in b {
            if x == y {
                res.push(*x);
                if res.len() == 2 {
                    return res;
                }
            }
        }
    }
    res
}

/// `meshNormals`: the normals of the mesh faces a feature of `dim` vertices
/// lies on, and their indices: the one face of three vertices; up to two
/// faces of an edge; up to `mjMAX_POLYVERT` faces at a vertex.
fn mesh_normals(
    o: &CcdObj<'_>,
    polygons: &MeshPolygons,
    dim: usize,
    v: [usize; 3],
) -> Vec<([f64; 3], usize)> {
    let map = &polygons.map;
    let normal = |p: usize| (global_coord(&o.mat, None, polygons.normals[p]), p);
    match dim {
        3 => {
            let edgeset = intersect(&map[v[0]], &map[v[1]]);
            intersect(&edgeset, &map[v[2]])
                .first()
                .map(|&p| vec![normal(p)])
                .unwrap_or_default()
        }
        2 => intersect(&map[v[0]], &map[v[1]])
            .into_iter()
            .map(normal)
            .collect(),
        1 => map[v[0]]
            .iter()
            .take(MAX_POLYVERT)
            .map(|&p| normal(p))
            .collect(),
        _ => Vec::new(),
    }
}

/// `meshEdgeNormals`: the unit directions of the mesh edges a feature of
/// `dim` vertices lies on (the edge itself; or from a vertex, the edge to
/// the vertex before it on each face it lies on), each with its other end.
fn mesh_edge_normals(
    o: &CcdObj<'_>,
    mesh: &CcdMesh<'_>,
    dim: usize,
    v1: &[f64; 3],
    v2: &[f64; 3],
    v1i: usize,
) -> Vec<([f64; 3], [f64; 3])> {
    let unit = |end: [f64; 3]| {
        let mut d = sub3(&end, v1);
        normalize3(&mut d);
        (d, end)
    };
    match dim {
        2 => vec![unit(*v2)],
        1 => mesh.polygons.map[v1i]
            .iter()
            .take(MAX_POLYVERT)
            .filter_map(|&p| {
                let path = &mesh.polygons.paths[p];
                let j = path.iter().position(|&v| v == v1i)?;
                let prev = path[if j == 0 { path.len() - 1 } else { j - 1 }];
                let v = &mesh.verts[prev];
                Some(unit(global_coord(&o.mat, Some(&o.pos), [v.x, v.y, v.z])))
            })
            .collect(),
        _ => Vec::new(),
    }
}

/// `meshFace`: a mesh face's corners, up to `mjMAX_POLYVERT`, in reverse
/// order.
fn mesh_face(o: &CcdObj<'_>, mesh: &CcdMesh<'_>, idx: usize) -> Vec<[f64; 3]> {
    let path = &mesh.polygons.paths[idx];
    path[..path.len().min(MAX_POLYVERT)]
        .iter()
        .rev()
        .map(|&i| {
            let v = &mesh.verts[i];
            global_coord(&o.mat, Some(&o.pos), [v.x, v.y, v.z])
        })
        .collect()
}

/// The face normals a feature of a geom can touch (`boxNormals`,
/// `meshNormals`).
fn face_normals(
    o: &CcdObj<'_>,
    dim: usize,
    index: &[Option<usize>; 3],
    dir: &[f64; 3],
) -> Vec<([f64; 3], usize)> {
    let [Some(a), Some(b), Some(c)] = *index else {
        return Vec::new();
    };
    match (o.geom_type, &o.mesh) {
        (Some(GeomType::Box), _) => box_normals(&o.mat, dim, [a, b, c], dir),
        (Some(GeomType::Mesh), Some(mesh)) => mesh_normals(o, mesh.polygons, dim, [a, b, c]),
        _ => Vec::new(),
    }
}

/// The edge directions a feature of a geom lies on (`boxEdgeNormals`,
/// `meshEdgeNormals`).
fn edge_normals(
    o: &CcdObj<'_>,
    dim: usize,
    vert: &[[f64; 3]; 3],
    index: &[Option<usize>; 3],
) -> Vec<([f64; 3], [f64; 3])> {
    let Some(v1i) = index[0] else {
        return Vec::new();
    };
    match (o.geom_type, &o.mesh) {
        (Some(GeomType::Box), _) => box_edge_normals(o, dim, &vert[0], &vert[1], v1i),
        (Some(GeomType::Mesh), Some(mesh)) => {
            mesh_edge_normals(o, mesh, dim, &vert[0], &vert[1], v1i)
        }
        _ => Vec::new(),
    }
}

/// A geom's face from its index (`boxFace`, `meshFace`).
fn face_polygon(o: &CcdObj<'_>, idx: usize) -> Vec<[f64; 3]> {
    match (o.geom_type, &o.mesh) {
        (Some(GeomType::Box), _) => box_face(o, idx),
        (Some(GeomType::Mesh), Some(mesh)) => mesh_face(o, mesh, idx),
        _ => Vec::new(),
    }
}

/// `multicontact`: from EPA's face, the features each geom touches with; if
/// two faces face each other, or an edge lies along a face, the one clipped
/// by the other gives the contacts (`polygonClip`), each displaced from geom
/// 2 to geom 1 by the face normal scaled to the first contact's length.
fn multicontact(pt: &Polytope, f: usize, status: &mut CcdStatus, o1: &CcdObj<'_>, o2: &CcdObj<'_>) {
    let face = pt.faces[f];
    let corners = face.verts.map(|i| pt.verts[i]);
    let mut index1 = corners.map(|v| v.index1);
    let mut index2 = corners.map(|v| v.index2);
    let mut vert1 = corners.map(|v| v.vert1);
    let mut vert2 = corners.map(|v| v.vert2);
    let nface1 = simplex_dim(&mut index1, &mut vert1);
    let nface2 = simplex_dim(&mut index2, &mut vert2);

    let dir = sub3(&status.x2[0], &status.x1[0]);
    let dir_neg = sub3(&status.x1[0], &status.x2[0]);
    let (n1, idx1): (Vec<_>, Vec<_>) = face_normals(o1, nface1, &index1, &dir_neg)
        .into_iter()
        .unzip();
    let (n2, idx2): (Vec<_>, Vec<_>) = face_normals(o2, nface2, &index2, &dir).into_iter().unzip();

    // Two faces facing each other, else an edge of the lesser feature along
    // the other geom's face.
    let mut endverts = Vec::new();
    let (i, j, edgecon1, edgecon2) = if let Some((i, j)) = aligned_faces(&n1, &n2) {
        (i, j, false, false)
    } else if nface1 < 3 && nface1 <= nface2 {
        let (edges, ends): (Vec<_>, Vec<_>) = edge_normals(o1, nface1, &vert1, &index1)
            .into_iter()
            .unzip();
        let Some((i, j)) = aligned_face_edge(&edges, &n2) else {
            return;
        };
        endverts = ends;
        (i, j, true, false)
    } else if nface2 < 3 {
        let (edges, ends): (Vec<_>, Vec<_>) = edge_normals(o2, nface2, &vert2, &index2)
            .into_iter()
            .unzip();
        let Some((i, j)) = aligned_face_edge(&edges, &n1) else {
            return;
        };
        endverts = ends;
        (i, j, false, true)
    } else {
        return;
    };

    let face1 = if edgecon1 {
        vec![corners[0].vert1, endverts[i]]
    } else {
        face_polygon(o1, if edgecon2 { idx1[j] } else { idx1[i] })
    };
    let face2 = if edgecon2 {
        vec![corners[0].vert2, endverts[i]]
    } else {
        face_polygon(o2, idx2[j])
    };

    // MuJoCo approximates each contact's displacement by the face normal
    // scaled to the first contact's length.
    let dnorm = norm3(&dir);
    if edgecon1 {
        let approx_dir = scl3(&n2[j], dnorm);
        polygon_clip(status, &face2, &face1, &n2[j], &approx_dir);
    } else if edgecon2 {
        let approx_dir = scl3(&n1[j], -dnorm);
        polygon_clip(status, &face1, &face2, &n1[j], &approx_dir);
    } else {
        let approx_dir = scl3(&n2[j], dnorm);
        polygon_clip(status, &face1, &face2, &n1[i], &approx_dir);
    }
}

// ---------------------------------------------------------------------------
// mjc_Convex
// ---------------------------------------------------------------------------

impl<'a> CcdObj<'a> {
    /// `mjc_initCCDObj` for geom `g` of `model` at `pos`, turned by `mat`:
    /// a mesh with its vertices, its hull's graph where every hull vertex is
    /// one of its vertices, and its polygons.
    pub(super) fn of_geom(
        model: &'a Model,
        g: usize,
        pos: &Vector3<f64>,
        mat: &Matrix3<f64>,
        margin: f64,
    ) -> Option<Self> {
        let mesh = model.geom_mesh[g].map(|id| {
            let data = &*model.mesh_data[id];
            CcdMesh {
                verts: data.vertices(),
                hull: data
                    .convex_hull()
                    .filter(|_| !data.hull_vertex().is_empty())
                    .map(|hull| HullGraph {
                        vertex: data.hull_vertex(),
                        neighbours: &hull.adjacency,
                    }),
                polygons: data.polygons(),
            }
        });
        let size = model.geom_size[g];
        Self::new(
            model.geom_type[g],
            [pos.x, pos.y, pos.z],
            std::array::from_fn(|i| mat[(i / 3, i % 3)]),
            [size.x, size.y, size.z],
            margin,
            mesh,
        )
    }
}

/// `mjc_CCDIteration`, its native branch (`engine_collision_convex.c:788-818`):
/// `mjc_ccd` asked for up to `max_contacts` contacts; where the geoms
/// penetrate, one per witness pair, its distance `margin + dist`, its normal
/// the first witness less the second, normalised, and its position their
/// midpoint.
fn ccd_iteration(
    model: &Model,
    o1: &mut CcdObj<'_>,
    o2: &mut CcdObj<'_>,
    max_contacts: usize,
    margin: f64,
) -> Vec<RawContact> {
    let config = CcdConfig {
        max_iterations: model.ccd_iterations,
        tolerance: model.ccd_tolerance,
        max_contacts,
        dist_cutoff: 0.0,
    };
    let st = mjc_ccd(&config, o1, o2);
    if st.dist < 0.0 {
        (0..st.nx)
            .map(|i| {
                let mut normal = sub3(&st.x1[i], &st.x2[i]);
                normalize3(&mut normal);
                RawContact {
                    dist: margin + st.dist,
                    pos: std::array::from_fn(|k| 0.5 * (st.x1[i][k] + st.x2[i][k])),
                    normal,
                }
            })
            .collect()
    } else {
        Vec::new()
    }
}

/// `maxContacts`: the contacts one pass of `mjc_ccd` recovers: 1 with a
/// margin; 8 for two boxes; 4 for two boxes or meshes under MULTICCD; else 1.
/// A mesh without polygons gives the multi-contact no faces to clip, so its
/// pair takes 1 (MuJoCo's compiler merges polygons for every mesh; here a
/// `TriangleMeshData` never given `compute_convex_hull` has none).
fn max_contacts(model: &Model, o1: &CcdObj<'_>, o2: &CcdObj<'_>) -> usize {
    if o1.margin > 0.0 || o2.margin > 0.0 {
        return 1;
    }
    let unmerged = |o: &CcdObj<'_>| o.mesh.is_some_and(|m| m.polygons.paths.is_empty());
    match (o1.geom_type, o2.geom_type) {
        (Some(GeomType::Box), Some(GeomType::Box)) => 8,
        (Some(GeomType::Box | GeomType::Mesh), Some(GeomType::Box | GeomType::Mesh))
            if enabled(model, ENABLE_MULTICCD) && !unmerged(o1) && !unmerged(o2) =>
        {
            4
        }
        _ => 1,
    }
}

/// `mju_mulMatMat3`.
fn mat_mat(a: &Mat9, b: &Mat9) -> Mat9 {
    std::array::from_fn(|i| {
        let (r, c) = (i / 3, i % 3);
        a[3 * r] * b[c] + a[3 * r + 1] * b[3 + c] + a[3 * r + 2] * b[6 + c]
    })
}

/// `mju_rotateFrame`: a frame turned by `rot` about `origin`.
fn rotate_frame(origin: &[f64; 3], rot: &Mat9, xmat: &mut Mat9, xpos: &mut [f64; 3]) {
    *xmat = mat_mat(rot, xmat);
    let rel = sub3(origin, xpos);
    let moved = sub3(&mat_vec(rot, &rel), &rel);
    *xpos = sub3(xpos, &moved);
}

/// `mju_dist3`.
fn dist3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let d = sub3(a, b);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// `mjc_Convex` (`engine_collision_convex.c:917-1000`): the contacts of two
/// convex geoms, `g1` first, at `pos1`, `mat1` and `pos2`, `mat2`, each
/// inflated by half of `margin`. One pass of `mjc_ccd`, which recovers several
/// contacts for two boxes or meshes; for other pairs under MULTICCD, but a
/// sphere's or an ellipsoid's, four more passes with the first geom turned
/// by ±1e-3 rad about each tangent of the first contact, and the second
/// back, keeping each new contact farther than 1e-3 of the smaller bounding
/// radius from all before it, at the first contact's distance.
// MuJoCo's pair context, as the other collision functions take it.
#[allow(clippy::too_many_arguments)]
pub(super) fn mjc_convex(
    model: &Model,
    g1: usize,
    pos1: &Vector3<f64>,
    mat1: &Matrix3<f64>,
    g2: usize,
    pos2: &Vector3<f64>,
    mat2: &Matrix3<f64>,
    margin: f64,
) -> Vec<RawContact> {
    const RELATIVE_TOLERANCE: f64 = 1e-3;
    // `mji_axisAngle2Quat` of MuJoCo's ±1e-3 rad turn: the sine and cosine of
    // its half, correctly rounded, as literals (`rotmat`'s sine is one), so
    // the turn does not take the platform's libm's last bit.
    const HALF_TURN_SIN: f64 = 4.999_999_791_666_669e-4;
    const HALF_TURN_COS: f64 = 0.999_999_875_000_002_6;
    let (Some(mut o1), Some(mut o2)) = (
        CcdObj::of_geom(model, g1, pos1, mat1, margin),
        CcdObj::of_geom(model, g2, pos2, mat2, margin),
    ) else {
        return Vec::new();
    };
    let max_contacts = max_contacts(model, &o1, &o2);
    let mut con = ccd_iteration(model, &mut o1, &mut o2, max_contacts, margin);
    if max_contacts > 1 {
        return con;
    }

    let round = |t: Option<GeomType>| matches!(t, Some(GeomType::Ellipsoid | GeomType::Sphere));
    if con.len() == 1
        && enabled(model, ENABLE_MULTICCD)
        && !round(o1.geom_type)
        && !round(o2.geom_type)
    {
        let saved = (o1.pos, o1.mat, o2.pos, o2.mat);
        let (_, tangents) = make_frame(&Vector3::from(con[0].normal), &Vector3::zeros());
        let (r1, r2) = (model.geom_rbound[g1], model.geom_rbound[g2]);
        let tolerance = RELATIVE_TOLERANCE * if r1 < r2 { r1 } else { r2 };
        let origin = con[0].pos;
        for axis in tangents {
            for s in [-HALF_TURN_SIN, HALF_TURN_SIN] {
                let rot = quat_to_mat(&[HALF_TURN_COS, axis.x * s, axis.y * s, axis.z * s]);
                rotate_frame(&origin, &rot, &mut o1.mat, &mut o1.pos);
                rotate_frame(&origin, &transpose(&rot), &mut o2.mat, &mut o2.pos);
                // `mjc_isDistinctContact`: farther than the tolerance from each.
                if let Some(mut new) = ccd_iteration(model, &mut o1, &mut o2, 1, margin).pop()
                    && !con.iter().any(|c| dist3(&c.pos, &new.pos) <= tolerance)
                {
                    new.dist = con[0].dist;
                    con.push(new);
                }
                (o1.pos, o1.mat, o2.pos, o2.mat) = saved;
            }
        }
    }
    con
}

/// `maxplanemesh`: the most contacts of a plane and a mesh.
const MAX_PLANE_MESH: usize = 3;
/// `tolplanemesh`: a further plane–mesh contact is at least this fraction of
/// the mesh's radius from the first.
const TOL_PLANE_MESH: f64 = 0.3;

/// `mjc_PlaneConvex` (`engine_collision_convex.c:1042-1138`): the contacts of
/// the plane at `plane_pos`, its normal the third column of `plane_mat`, and
/// the ellipsoid or mesh `obj` (made without a margin), listed at a distance
/// up to `margin`. The first at the support point along the plane's inward
/// normal (`mjccd_support`), midway between it and the plane. A mesh adds up
/// to two more, each a vertex closer to the plane than `margin` (strictly) and
/// at least 0.3 of `rbound` from the first contact's position: with a hull
/// graph, among the first vertex's neighbours, in the graph's order; without
/// one, every other vertex in order. Each contact's normal is the plane's.
pub(super) fn mjc_plane_convex(
    plane_pos: &[f64; 3],
    plane_mat: &Mat9,
    obj: &mut CcdObj<'_>,
    rbound: f64,
    margin: f64,
) -> Vec<RawContact> {
    let normal = [plane_mat[2], plane_mat[5], plane_mat[8]];
    let dir = [-plane_mat[2], -plane_mat[5], -plane_mat[8]];
    let vec = obj.ccd_support(&dir);
    let dist = dot3(&normal, &sub3(&vec, plane_pos));
    if dist > margin {
        return Vec::new();
    }
    let first = add3(&vec, &scl3(&normal, -0.5 * dist));
    let mut con = vec![RawContact {
        dist,
        pos: first,
        normal,
    }];
    let Some(mesh) = obj.mesh else {
        return con;
    };

    let local_dir = mat_t_vec(&obj.mat, &dir);
    let threshold = dot3(&normal, &sub3(&obj.pos, plane_pos)) - margin;
    // `addplanemesh`: the vertex as a contact, unless too near the first.
    let add = |con: &mut Vec<RawContact>, v: &Point3<f64>| {
        let pnt = add3(&mat_vec(&obj.mat, &[v.x, v.y, v.z]), &obj.pos);
        if dist3(&pnt, &first) < TOL_PLANE_MESH * rbound {
            return;
        }
        let dist = dot3(&normal, &sub3(&pnt, plane_pos));
        con.push(RawContact {
            dist,
            pos: add3(&pnt, &scl3(&normal, -0.5 * dist)),
            normal,
        });
    };
    match mesh.hull {
        None => {
            for (i, v) in mesh.verts.iter().enumerate() {
                if con.len() >= MAX_PLANE_MESH {
                    break;
                }
                if dot_point(&local_dir, v) > threshold && Some(i) != obj.meshindex {
                    add(&mut con, v);
                }
            }
        }
        Some(hull) => {
            if let Some(best) = obj.meshindex {
                for &sub in &hull.neighbours[best] {
                    if con.len() >= MAX_PLANE_MESH {
                        break;
                    }
                    let v = &mesh.verts[hull.vertex[sub as usize]];
                    if dot_point(&local_dir, v) > threshold {
                        add(&mut con, v);
                    }
                }
            }
        }
    }
    con
}

/// `mjc_ConvexHField` (`engine_collision_convex.c:1185-1363`): the contacts
/// of a height field `hfield` of size `size` (its half-extents, top and
/// base), at `pos1` turned by `mat1`, and the convex geom `obj` (made at its
/// pose, without a margin), whose bounding radius is `rbound`. The geom is
/// taken into the field's frame. A pair whose geom's bounding sphere, or then
/// its box from its support points along the field's axes, is clear of the
/// field's box by `margin` has none. Else each triangular prism of the grid
/// under that box, the cells split along (c − 1, r)–(c, r + 1) as a window of
/// three corners slides along each row, its top raised by `margin`, goes to
/// `mjc_ccd` against the geom inflated by half the margin, for one contact
/// where they overlap: its distance `mjc_ccd`'s (not raised back by the
/// margin), its normal the first witness less the second, normalised, its
/// position their midpoint, back in the world frame. A prism whose three top
/// corners are below the geom's lowest point is skipped; at most
/// `mjMAXCONPAIR`.
///
/// The fields' tops are `size[2]` in MuJoCo, whose elevations are
/// normalised to [0, 1]; sim-mjcf's are not yet (book Rigid-loading L45), so
/// the two box tests take the higher of `size[2]` and the field's highest
/// point, the same where the elevations are within [0, 1].
// MuJoCo's arguments, the field's data split out as sim-core holds it.
#[allow(clippy::too_many_arguments)]
pub(super) fn mjc_convex_hfield(
    hfield: &HeightFieldData,
    size: &[f64; 4],
    pos1: &[f64; 3],
    mat1: &Mat9,
    obj: &mut CcdObj<'_>,
    rbound: f64,
    margin: f64,
    (max_iterations, tolerance): (usize, f64),
) -> Vec<RawContact> {
    let (nrow, ncol) = (hfield.depth(), hfield.width());
    // For a field of one row or column MuJoCo lists nothing (its column loop,
    // or both, is empty); this returns before dividing by `ncol - 1` or
    // `nrow - 1`.
    if nrow < 2 || ncol < 2 {
        return Vec::new();
    }
    let [size0, size1, size2, size3] = *size;
    let top = size2.max(hfield.max_height());

    let pos = mat_t_vec(mat1, &sub3(&obj.pos, pos1));
    let radius = rbound + margin;
    if size0 < pos[0] - radius
        || -size0 > pos[0] + radius
        || size1 < pos[1] - radius
        || -size1 > pos[1] + radius
        || top < pos[2] - radius
        || -size3 > pos[2] + radius
    {
        return Vec::new();
    }

    obj.mat = mat_t_mat(mat1, &obj.mat);
    obj.pos = pos;
    let mut bound = |axis: usize, sign: f64| {
        let mut dir = [0.0; 3];
        dir[axis] = sign;
        obj.support(&dir)[axis]
    };
    let (xmax, xmin) = (bound(0, 1.0), bound(0, -1.0));
    let (ymax, ymin) = (bound(1, 1.0), bound(1, -1.0));
    let (zmax, zmin) = (bound(2, 1.0), bound(2, -1.0));
    if xmin - margin > size0
        || xmax + margin < -size0
        || ymin - margin > size1
        || ymax + margin < -size1
        || zmin - margin > top
        || zmax + margin < -size3
    {
        return Vec::new();
    }

    obj.margin = margin;
    field_prisms(
        hfield,
        size,
        (pos1, mat1),
        obj,
        margin,
        [xmin, xmax, ymin, ymax, zmin],
        (max_iterations, tolerance),
    )
}

/// The prisms of a height field under a box in its frame,
/// `[xmin, xmax, ymin, ymax]` with the box's lowest point `zmin`, each
/// against `obj` (in the field's frame, its margin set) by `mjc_ccd` for one
/// contact, back in the world frame (`pos1`, `mat1`): the loop
/// `mjc_ConvexHField` and `mjc_HFieldElem` share
/// (`engine_collision_convex.c:1310-1363`, `:1735-1807`). The cells split
/// along (c − 1, r)–(c, r + 1) as a window of three corners slides along each
/// row, each prism's top raised by `margin`; a prism whose three top corners
/// are below `zmin` is skipped; at most `mjMAXCONPAIR`.
fn field_prisms(
    hfield: &HeightFieldData,
    size: &[f64; 4],
    (pos1, mat1): (&[f64; 3], &Mat9),
    obj: &mut CcdObj<'_>,
    margin: f64,
    [xmin, xmax, ymin, ymax, zmin]: [f64; 5],
    (max_iterations, tolerance): (usize, f64),
) -> Vec<RawContact> {
    let (nrow, ncol) = (hfield.depth(), hfield.width());
    let [size0, size1, _, size3] = *size;
    // A grid's extents are far below 2^52, exact as floats.
    #[allow(clippy::cast_precision_loss)]
    let (cols, rows) = ((ncol - 1) as f64, (nrow - 1) as f64);
    // The sub-grid under the geom's box. A cast to an integer saturates, and
    // takes a NaN to 0, where C's leaves it undefined. The first column and
    // row are kept within the grid as well, so `cmin + 1` cannot overflow:
    // past the last column MuJoCo's column loop is empty, though its row loop
    // reads past the grid (:1315-1316, :1177); ours lists nothing.
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let (cmin, cmax, rmin, rmax) = (
        (((xmin + size0) / (2.0 * size0) * cols).floor() as i64).clamp(0, ncol as i64 - 1),
        (((xmax + size0) / (2.0 * size0) * cols).ceil() as i64).min(ncol as i64 - 1),
        (((ymin + size1) / (2.0 * size1) * rows).floor() as i64).clamp(0, nrow as i64 - 1),
        (((ymax + size1) / (2.0 * size1) * rows).ceil() as i64).min(nrow as i64 - 1),
    );

    let (dx, dy) = ((2.0 * size0) / cols, (2.0 * size1) / rows);
    let config = CcdConfig {
        max_iterations,
        tolerance,
        max_contacts: 1,
        dist_cutoff: 0.0,
    };
    let mut prism = CcdObj::prism(size3);
    // `addPrismVert`: the grid point at column `c` and row `r + 1 - i`, each
    // within the grid (as the sub-grid is), so not negative, and exact as a
    // float.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    let add = |prism: &mut CcdObj<'_>, r: i64, c: i64, i: i64| {
        let row = r + 1 - i;
        let height = hfield.get(c as usize, row as usize).unwrap_or(0.0);
        prism.add_prism_vert(
            dx * c as f64 - size0,
            dy * row as f64 - size1,
            height + margin,
        );
    };
    let mut con = Vec::new();
    for r in rmin..rmax {
        add(&mut prism, r, cmin, 0);
        add(&mut prism, r, cmin, 1);
        for c in cmin + 1..=cmax {
            for i in 0..2 {
                add(&mut prism, r, c, i);
                if let Support::Prism(p) = &prism.support
                    && p[3][2] < zmin
                    && p[4][2] < zmin
                    && p[5][2] < zmin
                {
                    continue;
                }
                // `mjc_penetration`, native: one contact where they overlap,
                // unless its normal is libccd's zero vector (`ccdVec3Eq`
                // against the origin, :1331: each component's magnitude below
                // `CCD_EPS`, `DBL_EPSILON`).
                let st = mjc_ccd(&config, &mut prism, obj);
                if st.dist < 0.0 {
                    let mut dir = sub3(&st.x1[0], &st.x2[0]);
                    normalize3(&mut dir);
                    if dir.iter().all(|v| v.abs() < f64::EPSILON) {
                        continue;
                    }
                    let vec: [f64; 3] = std::array::from_fn(|k| 0.5 * (st.x1[0][k] + st.x2[0][k]));
                    con.push(RawContact {
                        dist: st.dist,
                        pos: add3(&mat_vec(mat1, &vec), pos1),
                        normal: mat_vec(mat1, &dir),
                    });
                    if con.len() >= MAX_CONPAIR {
                        return con;
                    }
                }
            }
        }
    }
    con
}

/// `mjc_ConvexElem` (`engine_collision_convex.c:1627-1653`): `o1` (a geom,
/// or a flex element or vertex) against the flex element `o2` by `mjc_ccd`,
/// one contact (`mjc_CCDIteration`: its distance `margin + dist`); with a
/// flex of dim 2 on either side each contact's normal then turned to the
/// geom's own (`mjc_fixNormal`; `fix` that geom's type, position, turn and
/// size, `None` without a geom). MuJoCo turns it only under native CCD, the
/// one sim-core runs (registry `D-NATIVECCD`).
pub(super) fn mjc_convex_elem(
    model: &Model,
    o1: &mut CcdObj<'_>,
    o2: &mut CcdObj<'_>,
    margin: f64,
    fix: Option<(GeomType, [f64; 3], &Mat9, [f64; 3])>,
) -> Vec<RawContact> {
    let mut contacts = ccd_iteration(model, o1, o2, 1, margin);
    if let Some((geom_type, pos, mat, size)) = fix {
        for contact in &mut contacts {
            fix_normal(geom_type, &pos, mat, &size, contact);
        }
    }
    contacts
}

/// `mjc_fixNormal` (`:1473-1621`) with a geom on the first side only, the
/// second a flex of dim 2: an ellipsoid's or a cylinder's own normal at the
/// contact point in place of `mjc_ccd`'s, where it is found. (Its sphere and
/// capsule branches never run for a flex: against a triangle those take the
/// raw functions, `engine_collision_driver.c:1931-1960`.)
fn fix_normal(
    geom_type: GeomType,
    gpos: &[f64; 3],
    mat: &Mat9,
    size: &[f64; 3],
    contact: &mut RawContact,
) {
    let pos = mat_t_vec(mat, &sub3(&contact.pos, gpos));
    let mut nrm = mat_t_vec(mat, &contact.normal);
    let processed = match geom_type {
        GeomType::Ellipsoid => {
            if size[0] < MJ_MINVAL || size[1] < MJ_MINVAL || size[2] < MJ_MINVAL {
                false
            } else {
                let dst = pos[0] * pos[0] / (size[0] * size[0])
                    + pos[1] * pos[1] / (size[1] * size[1])
                    + pos[2] * pos[2] / (size[2] * size[2]);
                if dst <= 1.0 {
                    ellipsoid_inside(&mut nrm, &pos, size)
                } else {
                    ellipsoid_outside(&mut nrm, &pos, size)
                }
            }
        }
        // The round wall's normal, unless within 5% of a cap's height or
        // not 4 times nearer the round wall than the flat one.
        GeomType::Cylinder => {
            let flat = (size[1] - pos[2].abs()).abs();
            let round = (size[0] - mju_dot(&pos[..2], &pos[..2]).sqrt()).abs();
            if pos[2].abs() > 0.95 * size[1] || flat < 0.25 * round {
                false
            } else {
                nrm = [pos[0], pos[1], 0.0];
                true
            }
        }
        _ => false,
    };
    if processed {
        normalize3(&mut nrm);
        contact.normal = mat_vec(mat, &nrm);
    }
}

/// `mjc_ellipsoidInside` (`:1369-1420`): the normal of an ellipsoid of
/// half-sizes `size` at the point where the ray from `pos` (inside) along
/// `nrm` leaves it, iterated to 1e-6, at most 30 times; whether one was
/// found.
fn ellipsoid_inside(nrm: &mut [f64; 3], pos: &[f64; 3], size: &[f64; 3]) -> bool {
    let s2inv = [
        1.0 / (size[0] * size[0]),
        1.0 / (size[1] * size[1]),
        1.0 / (size[2] * size[2]),
    ];
    let coef_c =
        pos[0] * pos[0] * s2inv[0] + pos[1] * pos[1] * s2inv[1] + pos[2] * pos[2] * s2inv[2] - 1.0;
    if coef_c > 0.0 {
        return false;
    }
    normalize3(nrm);
    for iter in 0..30 {
        let coef_a =
            nrm[0] * nrm[0] * s2inv[0] + nrm[1] * nrm[1] * s2inv[1] + nrm[2] * nrm[2] * s2inv[2];
        let coef_b =
            pos[0] * nrm[0] * s2inv[0] + pos[1] * nrm[1] * s2inv[1] + pos[2] * nrm[2] * s2inv[2];
        // The quadratic's discriminant, B² − AC, as MuJoCo's.
        #[allow(clippy::suspicious_operation_groupings)]
        let det = coef_b * coef_b - coef_a * coef_c;
        if det < MJ_MINVAL || coef_a < MJ_MINVAL {
            return iter > 0;
        }
        let x = (-coef_b + det.sqrt()) / coef_a;
        if x < 0.0 {
            return iter > 0;
        }
        let pnt = std::array::from_fn::<f64, 3, _>(|k| pos[k] + nrm[k] * x);
        let mut newnrm = [pnt[0] * s2inv[0], pnt[1] * s2inv[1], pnt[2] * s2inv[2]];
        normalize3(&mut newnrm);
        let dif = sub3(nrm, &newnrm);
        let change = (dif[0] * dif[0] + dif[1] * dif[1] + dif[2] * dif[2]).sqrt();
        *nrm = newnrm;
        if change < 1e-6 {
            break;
        }
    }
    true
}

/// `mjc_ellipsoidOutside` (`:1424-1469`): the normal of an ellipsoid of
/// half-sizes `size` at its point nearest `pos` (outside), by Newton's
/// method on the multiplier, to 1e-6, at most 30 times.
fn ellipsoid_outside(nrm: &mut [f64; 3], pos: &[f64; 3], size: &[f64; 3]) -> bool {
    let s2 = [size[0] * size[0], size[1] * size[1], size[2] * size[2]];
    let ps2 = [
        pos[0] * pos[0] * s2[0],
        pos[1] * pos[1] * s2[1],
        pos[2] * pos[2] * s2[2],
    ];
    let mut la = 0.0;
    for _ in 0..30 {
        let r = [1.0 / (s2[0] + la), 1.0 / (s2[1] + la), 1.0 / (s2[2] + la)];
        let val = ps2[0] * r[0] * r[0] + ps2[1] * r[1] * r[1] + ps2[2] * r[2] * r[2] - 1.0;
        if val < 1e-6 {
            break;
        }
        let deriv = -2.0
            * (ps2[0] * r[0] * r[0] * r[0]
                + ps2[1] * r[1] * r[1] * r[1]
                + ps2[2] * r[2] * r[2] * r[2]);
        if deriv > -MJ_MINVAL {
            break;
        }
        let delta = -val / deriv;
        if delta < 1e-6 {
            break;
        }
        la += delta;
    }
    *nrm = [
        pos[0] / (s2[0] + la),
        pos[1] / (s2[1] + la),
        pos[2] / (s2[2] + la),
    ];
    normalize3(nrm);
    true
}

/// `mjc_HFieldElem` (`:1657-1816`): a height field (`hfield`, `size`, at
/// `pos1` turned by `mat1`) against a flex element (its world `vertices`,
/// its box's `center`, the flex's `radius`), within `margin`. The element is
/// taken into the field's frame; nothing when its box (over its vertices,
/// not their radius) is clear of the field's by the margin; else the prisms
/// under that box against the element, a prism below the box's lowest
/// vertex skipped. The field's top is the higher of `size[2]` and its
/// highest point, as `mjc_convex_hfield` takes it. MuJoCo sets only its
/// prism object's centre and support (`:1663-1664`), never calling
/// `mjc_initCCDObj`, so GJK's support reads the object's margin and geom
/// unset (`engine_collision_gjk.c:304`), undefined in C; this prism is
/// `mjc_ConvexHField`'s (`:1220`), margin 0, and `mj_triangle`'s
/// `convex_route_contacts_as_mujoco` holds its contacts to the oracle's.
// MuJoCo's arguments, the field and the element split out as sim-core holds them.
#[allow(clippy::too_many_arguments)]
pub(super) fn mjc_hfield_elem(
    hfield: &HeightFieldData,
    size: &[f64; 4],
    pos1: &[f64; 3],
    mat1: &Mat9,
    vertices: &[[f64; 3]],
    center: &[f64; 3],
    radius: f64,
    margin: f64,
    ccd: (usize, f64),
) -> Vec<RawContact> {
    let (nrow, ncol) = (hfield.depth(), hfield.width());
    if nrow < 2 || ncol < 2 || vertices.is_empty() {
        return Vec::new();
    }
    let [size0, size1, size2, size3] = *size;
    let top = size2.max(hfield.max_height());
    let local: Vec<[f64; 3]> = vertices
        .iter()
        .map(|v| mat_t_vec(mat1, &sub3(v, pos1)))
        .collect();
    // `mju_min` and `mju_max` over the vertices.
    let mut lo = local[0];
    let mut hi = local[0];
    for v in &local[1..] {
        for k in 0..3 {
            lo[k] = if lo[k] <= v[k] { lo[k] } else { v[k] };
            hi[k] = if hi[k] >= v[k] { hi[k] } else { v[k] };
        }
    }
    if lo[0] - margin > size0
        || hi[0] + margin < -size0
        || lo[1] - margin > size1
        || hi[1] + margin < -size1
        || lo[2] - margin > top
        || hi[2] + margin < -size3
    {
        return Vec::new();
    }
    let center = mat_t_vec(mat1, &sub3(center, pos1));
    let mut elem = CcdObj::flex(&local, center, radius, margin);
    field_prisms(
        hfield,
        size,
        (pos1, mat1),
        &mut elem,
        margin,
        [lo[0], hi[0], lo[1], hi[1], lo[2]],
        ccd,
    )
}

/// `mj_geomDistanceCCD` (`engine_support.c:527-552`): the distance of `g1`
/// and `g2` at their poses in `data`, without their margins, from `mjc_ccd`
/// asked for one contact up to `distmax`, and the witness points from `g1`
/// to `g2` where it gives them (else zeros); a distance past `distmax` reads
/// `distmax`.
#[must_use]
pub fn geom_distance_ccd(
    model: &Model,
    data: &Data,
    g1: usize,
    g2: usize,
    distmax: f64,
) -> (f64, [f64; 6]) {
    let mut fromto = [0.0; 6];
    let (Some(mut o1), Some(mut o2)) = (
        CcdObj::of_geom(model, g1, &data.geom_xpos[g1], &data.geom_xmat[g1], 0.0),
        CcdObj::of_geom(model, g2, &data.geom_xpos[g2], &data.geom_xmat[g2], 0.0),
    ) else {
        return (distmax, fromto);
    };
    let config = CcdConfig {
        max_iterations: model.ccd_iterations,
        tolerance: model.ccd_tolerance,
        max_contacts: 1,
        dist_cutoff: distmax,
    };
    let st = mjc_ccd(&config, &mut o1, &mut o2);
    if st.nx > 0 {
        fromto[..3].copy_from_slice(&st.x1[0]);
        fromto[3..].copy_from_slice(&st.x2[0]);
    }
    (if st.dist < distmax { st.dist } else { distmax }, fromto)
}

// ---------------------------------------------------------------------------
// Vector helpers of engine_collision_gjk.c
// ---------------------------------------------------------------------------

/// `equal3`: every component within mjMINVAL.
fn equal3(v1: &[f64; 3], v2: &[f64; 3]) -> bool {
    (v1[0] - v2[0]).abs() < MJ_MINVAL
        && (v1[1] - v2[1]).abs() < MJ_MINVAL
        && (v1[2] - v2[2]).abs() < MJ_MINVAL
}

/// `norm3`.
fn norm3(v: &[f64; 3]) -> f64 {
    dot3(v, v).sqrt()
}

/// `cross3`.
fn cross3(v1: &[f64; 3], v2: &[f64; 3]) -> [f64; 3] {
    [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ]
}

/// `det3`: the determinant of the columns `v1`, `v2`, `v3`, as `v1 · (v2 × v3)`.
fn det3(v1: &[f64; 3], v2: &[f64; 3], v3: &[f64; 3]) -> f64 {
    v1[0] * (v2[1] * v3[2] - v2[2] * v3[1])
        + v1[1] * (v2[2] * v3[0] - v2[0] * v3[2])
        + v1[2] * (v2[0] * v3[1] - v2[1] * v3[0])
}

#[cfg(test)]
#[path = "mj_ccd_tests.rs"]
mod tests;
