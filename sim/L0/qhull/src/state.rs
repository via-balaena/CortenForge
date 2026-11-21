// Ported from Qhull, libqhull_r (libqhull_r.h, poly_r.c, poly2_r.c, global_r.c;
// geom_r.h's fmax_, maximize_ and minimize_; user_r.h's constants).
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// libqhull_r.h, poly_r.c, poly2_r.c, global_r.c, geom_r.h: Copyright (c)
// 1993-2020 The Geometry Center.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! The hull under construction: qhull's `qhT`, `facetT`, `vertexT` and
//! `ridgeT` for the 3-d path, and the list operations every routine uses.
//!
//! Each facet, vertex and ridge lives in an arena at the index qhull gives it as
//! its id (`qh_newfacet`, `qh_newvertex`, `qh_newridge` number them from 0, the
//! sentinel tails taking 0), and stays there once deleted: qhull never reuses
//! an id. Qhull's pointer lists (the facet list with its `facet_tail`
//! sentinel, the vertex list with `vertex_tail`) are the `next`/`previous`
//! links kept here as ids.

use crate::error::Error;

/// Every coordinate, normal and point in the port: 3-d, as MuJoCo calls qhull.
pub(crate) const DIM: usize = 3;

/// A facet's id: its index in [`Qhull::facets`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub(crate) struct FacetId(pub(crate) u32);

impl FacetId {
    /// An empty slot (qhull's NULL), as `qh_matchnewfacets` clears a new
    /// facet's neighbours before matching them.
    pub(crate) const NULL: Self = Self(u32::MAX);
    /// The arena index.
    pub(crate) const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A vertex's id: its index in [`Qhull::vertices`]. Facet vertex sets are
/// kept in decreasing id order, as qhull keeps them.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub(crate) struct VertexId(pub(crate) u32);

impl VertexId {
    /// The arena index.
    pub(crate) const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A ridge's id: its index in [`Qhull::ridges`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub(crate) struct RidgeId(pub(crate) u32);

impl RidgeId {
    /// An empty slot (qhull's NULL).
    pub(crate) const NULL: Self = Self(u32::MAX);
    /// The arena index.
    pub(crate) const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A list `qh_prependfacet` prepends to.
#[derive(Clone, Copy, Debug)]
pub(crate) enum ListHead {
    /// `qh.facet_next` (`qh_furthestnext`).
    FacetNext,
    /// `qh.visible_list` (`qh_willdelete`).
    Visible,
}

/// An input point's index (qhull's `qh_pointid`).
pub(crate) type PointId = u32;

/// qhull's `facetT`, the fields the 3-d path reads.
#[derive(Clone, Debug, Default)]
pub(crate) struct Facet {
    /// The furthest outside point's distance (`furthestdist`).
    pub(crate) furthestdist: f64,
    /// The furthest computed distance of a point above it (`maxoutside`).
    pub(crate) maxoutside: f64,
    /// The hyperplane's offset from the origin.
    pub(crate) offset: f64,
    /// The hyperplane's unit normal, once set (`qh_setfacetplane`).
    pub(crate) normal: Option<[f64; DIM]>,
    /// The centrum, once computed (`qh_getcentrum`; qhull's `center` under
    /// `qh_AScentrum`).
    pub(crate) center: Option<[f64; DIM]>,
    /// The `f` union: the replacement facet of a visible one, a new facet's
    /// same cycle, a horizon facet's new cycle, a tricoplanar's owner. Which
    /// one is meant is the caller's, as in qhull.
    pub(crate) f: Option<FacetId>,
    /// The previous facet in its list, `None` at the head.
    pub(crate) previous: Option<FacetId>,
    /// The next facet in its list, `None` only for the tail sentinel.
    pub(crate) next: Option<FacetId>,
    /// Its vertices, decreasing by id; a simplicial facet's first is its apex.
    pub(crate) vertices: Vec<VertexId>,
    /// Its ridges, `None` (qhull's NULL set) until `qh_makeridges` or
    /// `qh_makenew_nonsimplicial` makes the set.
    pub(crate) ridges: Option<Vec<RidgeId>>,
    /// Its neighbours; a simplicial facet's `k`th lies opposite its `k`th
    /// vertex.
    pub(crate) neighbors: Vec<FacetId>,
    /// The points outside it, the furthest last; `None` is qhull's NULL set.
    pub(crate) outsideset: Option<Vec<PointId>>,
    /// The points coplanar with it, the furthest last; `None` is qhull's NULL.
    pub(crate) coplanarset: Option<Vec<PointId>>,
    /// The visit mark (`qh.visit_id`).
    pub(crate) visitid: u32,
    /// Its id, which is also its arena index.
    pub(crate) id: u32,
    /// The number of merges into it, at most `qh_MAXnummerge`.
    pub(crate) nummerge: u16,
    pub(crate) tricoplanar: bool,
    pub(crate) newfacet: bool,
    pub(crate) visible: bool,
    pub(crate) toporient: bool,
    pub(crate) simplicial: bool,
    pub(crate) seen: bool,
    pub(crate) flipped: bool,
    pub(crate) good: bool,
    pub(crate) dupridge: bool,
    pub(crate) mergeridge: bool,
    pub(crate) mergeridge2: bool,
    pub(crate) coplanarhorizon: bool,
    pub(crate) mergehorizon: bool,
    pub(crate) tested: bool,
    pub(crate) keepcentrum: bool,
    pub(crate) newmerge: bool,
    pub(crate) degenerate: bool,
    pub(crate) redundant: bool,
    /// Deleted (`qh_delfacet`); qhull frees it.
    pub(crate) deleted: bool,
}

/// qhull's `vertexT`, the fields the 3-d path reads.
#[derive(Clone, Debug, Default)]
pub(crate) struct Vertex {
    /// The next vertex in its list, `None` only for the tail sentinel.
    pub(crate) next: Option<VertexId>,
    /// The previous vertex in its list, `None` at the head.
    pub(crate) previous: Option<VertexId>,
    /// Its point, `None` for the tail sentinel.
    pub(crate) point: Option<PointId>,
    /// Its neighbouring facets (`qh_vertexneighbors`, kept up by merges).
    pub(crate) neighbors: Vec<FacetId>,
    /// Its id, which is also its arena index.
    pub(crate) id: u32,
    /// The visit mark (`qh.vertex_visit`).
    pub(crate) visitid: u32,
    pub(crate) seen: bool,
    /// To be deleted, on `qh.del_vertices`.
    pub(crate) deleted: bool,
    pub(crate) delridge: bool,
    pub(crate) newfacet: bool,
    pub(crate) partitioned: bool,
    /// Deleted (`qh_delvertex`); qhull frees it.
    pub(crate) freed: bool,
}

/// qhull's `ridgeT`.
#[derive(Clone, Debug, Default)]
pub(crate) struct Ridge {
    /// Its vertices, decreasing by id.
    pub(crate) vertices: Vec<VertexId>,
    pub(crate) top: Option<FacetId>,
    pub(crate) bottom: Option<FacetId>,
    pub(crate) tested: bool,
    pub(crate) nonconvex: bool,
    pub(crate) simplicialtop: bool,
    pub(crate) simplicialbot: bool,
    /// Deleted (`qh_delridge`); qhull frees it.
    pub(crate) deleted: bool,
}

/// qhull's `qhT` for one run of `qhull Qt` (with `TA` under `maxhullvert`)
/// over 3-d points: the arenas, the lists, and the constants and counters the
/// 3-d path reads. Field names are qhull's, so each routine reads as its
/// original.
#[allow(non_snake_case)] // qhull's field names, kept so each port reads as its source
pub(crate) struct Qhull<'p> {
    /// The input points (`qh.first_point`, `qh.num_points`).
    pub(crate) points: &'p [[f64; DIM]],
    pub(crate) facets: Vec<Facet>,
    pub(crate) vertices: Vec<Vertex>,
    pub(crate) ridges: Vec<Ridge>,

    // The lists (`qh.facet_list` …), each `None` where qhull's is NULL.
    pub(crate) facet_list: Option<FacetId>,
    pub(crate) facet_tail: Option<FacetId>,
    pub(crate) facet_next: Option<FacetId>,
    pub(crate) newfacet_list: Option<FacetId>,
    pub(crate) visible_list: Option<FacetId>,
    pub(crate) vertex_list: Option<VertexId>,
    pub(crate) vertex_tail: Option<VertexId>,
    pub(crate) newvertex_list: Option<VertexId>,
    /// Vertices to delete (`qh.del_vertices`).
    pub(crate) del_vertices: Vec<VertexId>,

    pub(crate) num_facets: i32,
    pub(crate) num_vertices: i32,
    pub(crate) num_visible: i32,
    pub(crate) num_outside: i32,
    pub(crate) facet_id: u32,
    pub(crate) vertex_id: u32,
    pub(crate) ridge_id: u32,
    pub(crate) visit_id: u32,
    pub(crate) vertex_visit: u32,
    pub(crate) first_newfacet: u32,
    pub(crate) furthest_id: i32,
    /// The facet a coplanar point is being repartitioned from, against a
    /// recursion (`qh.repart_facetid`).
    pub(crate) repart_facetid: u32,
    pub(crate) retry_addpoint: i32,

    // Options: `qhull Qt`'s defaults in 3-d (`qh_initqhull_globals`,
    // `global_r.c:1610-1682`), and `TA` (`qh.STOPadd`).
    /// `TA n`'s `n`, 0 when not given.
    pub(crate) STOPadd: i32,
    pub(crate) NARROWhull: bool,
    pub(crate) ZEROall_ok: bool,
    pub(crate) WAScoplanar: bool,
    pub(crate) findbestnew: bool,
    pub(crate) findbest_notsharp: bool,
    pub(crate) maxoutdone: bool,
    pub(crate) NEWfacets: bool,
    pub(crate) NEWtentative: bool,
    pub(crate) VERTEXneighbors: bool,
    pub(crate) hasTriangulation: bool,
    pub(crate) POSTmerging: bool,
    pub(crate) DOcheckmax: bool,

    // Constants of the input (`qh_maxmin`, `qh_detroundoff`).
    pub(crate) MAXabs_coord: f64,
    pub(crate) MAXsumcoord: f64,
    pub(crate) MAXwidth: f64,
    pub(crate) MINlastcoord: f64,
    pub(crate) MAXlastcoord: f64,
    pub(crate) NEARzero: [f64; DIM],
    pub(crate) DISTround: f64,
    pub(crate) ANGLEround: f64,
    pub(crate) MINdenom_1: f64,
    pub(crate) MINdenom: f64,
    pub(crate) MINdenom_1_2: f64,
    pub(crate) MINdenom_2: f64,
    pub(crate) premerge_cos: f64,
    pub(crate) postmerge_cos: f64,
    pub(crate) premerge_centrum: f64,
    pub(crate) postmerge_centrum: f64,
    pub(crate) ONEmerge: f64,
    pub(crate) NEARinside: f64,
    pub(crate) MINvisible: f64,
    pub(crate) MAXcoplanar: f64,
    pub(crate) MINoutside: f64,
    pub(crate) WIDEfacet: f64,
    pub(crate) MAXoutside: f64,
    pub(crate) max_outside: f64,
    pub(crate) max_vertex: f64,
    pub(crate) min_vertex: f64,
    pub(crate) cos_max: f64,
    pub(crate) centrum_radius: f64,
    pub(crate) interior_point: [f64; DIM],

    /// `zzval_(Ztotmerge)`: the merge count, which decides
    /// `qh_USEfindbestnew` (`user_r.h:903`).
    pub(crate) Ztotmerge: i32,
    /// The facet merges to make (`qh.facet_mergeset`), taken from the end.
    pub(crate) facet_mergeset: Vec<crate::merge::Merge>,
    /// The degenerate and redundant merges (`qh.degen_mergeset`).
    pub(crate) degen_mergeset: Vec<crate::merge::Merge>,
}

/// qhull's `REALmax` (`DBL_MAX`).
pub(crate) const REAL_MAX: f64 = f64::MAX;
/// qhull's `REALmin` (`DBL_MIN`).
pub(crate) const REAL_MIN: f64 = f64::MIN_POSITIVE;
/// qhull's `REALepsilon` (`DBL_EPSILON`).
pub(crate) const REAL_EPSILON: f64 = f64::EPSILON;

/// `fmax_` (`geom_r.h:35`): `b` where `a < b`, else `a`.
pub(crate) fn fmax_(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// `maximize_` (`geom_r.h:51`).
pub(crate) fn maximize_(maxval: &mut f64, val: f64) {
    if *maxval < val {
        *maxval = val;
    }
}

/// `minimize_` (`geom_r.h:59`).
pub(crate) fn minimize_(minval: &mut f64, val: f64) {
    if *minval > val {
        *minval = val;
    }
}

impl<'p> Qhull<'p> {
    /// The state `qh_zero`, `qh_init_A`, `qh_initflags` and `qh_init_B` leave
    /// for `qhull Qt` (and `TA stop_add` where `stop_add > 0`) over `points`:
    /// `qh_initqhull_start2`'s defaults (`global_r.c:1961-2014`) with the merge
    /// options `qh_initqhull_globals` sets in 3-d (`:1610-1682`).
    pub(crate) fn new(points: &'p [[f64; DIM]], stop_add: i32) -> Self {
        Self {
            points,
            facets: Vec::new(),
            vertices: Vec::new(),
            ridges: Vec::new(),
            facet_list: None,
            facet_tail: None,
            facet_next: None,
            newfacet_list: None,
            visible_list: None,
            vertex_list: None,
            vertex_tail: None,
            newvertex_list: None,
            del_vertices: Vec::new(),
            num_facets: 0,
            num_vertices: 0,
            num_visible: 0,
            num_outside: 0,
            facet_id: 0,
            vertex_id: 0,
            ridge_id: 0,
            visit_id: 0,
            vertex_visit: 0,
            first_newfacet: 0,
            furthest_id: -1,
            repart_facetid: 0,
            retry_addpoint: 0,
            STOPadd: stop_add,
            NARROWhull: false,
            ZEROall_ok: true,
            WAScoplanar: false,
            findbestnew: false,
            findbest_notsharp: false,
            maxoutdone: false,
            NEWfacets: false,
            NEWtentative: false,
            VERTEXneighbors: false,
            hasTriangulation: false,
            POSTmerging: false,
            DOcheckmax: true,
            MAXabs_coord: 0.0,
            MAXsumcoord: 0.0,
            MAXwidth: -REAL_MAX,
            MINlastcoord: 0.0,
            MAXlastcoord: 0.0,
            NEARzero: [0.0; DIM],
            DISTround: 0.0,
            ANGLEround: 0.0,
            MINdenom_1: fmax_(1.0 / REAL_MAX, REAL_MIN),
            MINdenom: 0.0,
            MINdenom_1_2: 0.0,
            MINdenom_2: 0.0,
            premerge_cos: REAL_MAX,
            postmerge_cos: REAL_MAX,
            premerge_centrum: 0.0,
            postmerge_centrum: 0.0,
            ONEmerge: 0.0,
            NEARinside: 0.0,
            MINvisible: REAL_MAX,
            MAXcoplanar: REAL_MAX,
            MINoutside: 0.0,
            WIDEfacet: 0.0,
            MAXoutside: 0.0,
            max_outside: 0.0,
            max_vertex: 0.0,
            min_vertex: 0.0,
            cos_max: 0.0,
            centrum_radius: 0.0,
            interior_point: [0.0; DIM],
            Ztotmerge: 0,
            facet_mergeset: Vec::new(),
            degen_mergeset: Vec::new(),
        }
    }

    /// A point's coordinates.
    pub(crate) fn point(&self, id: PointId) -> &'p [f64; DIM] {
        &self.points[id as usize]
    }

    /// A facet.
    pub(crate) fn f(&self, id: FacetId) -> &Facet {
        &self.facets[id.index()]
    }

    /// A facet, to change.
    pub(crate) fn fm(&mut self, id: FacetId) -> &mut Facet {
        &mut self.facets[id.index()]
    }

    /// A vertex.
    pub(crate) fn v(&self, id: VertexId) -> &Vertex {
        &self.vertices[id.index()]
    }

    /// A vertex, to change.
    pub(crate) fn vm(&mut self, id: VertexId) -> &mut Vertex {
        &mut self.vertices[id.index()]
    }

    /// A vertex's point, which every vertex but the tail sentinel has.
    pub(crate) fn vpoint(&self, id: VertexId) -> PointId {
        self.v(id).point.unwrap_or(0)
    }

    /// A vertex's coordinates.
    pub(crate) fn vcoords(&self, id: VertexId) -> &'p [f64; DIM] {
        self.point(self.vpoint(id))
    }

    /// A ridge.
    pub(crate) fn r(&self, id: RidgeId) -> &Ridge {
        &self.ridges[id.index()]
    }

    /// A ridge, to change.
    pub(crate) fn rm(&mut self, id: RidgeId) -> &mut Ridge {
        &mut self.ridges[id.index()]
    }

    /// A facet's normal, which every facet the routines read has been given.
    pub(crate) fn normal(&self, id: FacetId) -> [f64; DIM] {
        self.f(id).normal.unwrap_or([0.0; DIM])
    }

    /// The facets of `list`, as `FORALLfacet_` walks them (to the tail
    /// sentinel, which has no `next`), collected first.
    pub(crate) fn facets_from(&self, list: Option<FacetId>) -> Vec<FacetId> {
        let mut out = Vec::new();
        let mut cur = list;
        while let Some(f) = cur {
            let next = self.f(f).next;
            if next.is_none() {
                break;
            }
            out.push(f);
            cur = next;
        }
        out
    }

    /// The vertices of `list`, as `FORALLvertex_` walks them, collected first.
    pub(crate) fn vertices_from(&self, list: Option<VertexId>) -> Vec<VertexId> {
        let mut out = Vec::new();
        let mut cur = list;
        while let Some(v) = cur {
            let next = self.v(v).next;
            if next.is_none() {
                break;
            }
            out.push(v);
            cur = next;
        }
        out
    }

    /// `qh_newfacet` (`poly_r.c:1088-1112`).
    pub(crate) fn newfacet(&mut self) -> FacetId {
        let id = self.facet_id;
        self.facet_id += 1;
        self.facets.push(Facet {
            id,
            maxoutside: self.DISTround,
            simplicial: true,
            good: true,
            newfacet: true,
            ..Facet::default()
        });
        FacetId(id)
    }

    /// `qh_newvertex` (`poly2_r.c:2924-2942`).
    pub(crate) fn newvertex(&mut self, point: Option<PointId>) -> VertexId {
        let id = self.vertex_id;
        self.vertex_id += 1;
        self.vertices.push(Vertex {
            point,
            id,
            ..Vertex::default()
        });
        VertexId(id)
    }

    /// `qh_newridge` (`poly_r.c:1123-1136`).
    pub(crate) fn newridge(&mut self) -> RidgeId {
        // qhull's `ridge->id`: the arena index.
        let id = self.ridge_id;
        self.ridge_id += 1;
        self.ridges.push(Ridge::default());
        RidgeId(id)
    }

    /// `qh_appendfacet` (`poly_r.c:38-57`): before the tail sentinel.
    pub(crate) fn appendfacet(&mut self, facet: FacetId) {
        let Some(tail) = self.facet_tail else { return };
        if Some(tail) == self.newfacet_list {
            self.newfacet_list = Some(facet);
            if Some(tail) == self.visible_list {
                self.visible_list = Some(facet);
            }
        }
        if Some(tail) == self.facet_next {
            self.facet_next = Some(facet);
        }
        let prev = self.f(tail).previous;
        self.fm(facet).previous = prev;
        self.fm(facet).next = Some(tail);
        match prev {
            Some(p) => self.fm(p).next = Some(facet),
            None => self.facet_list = Some(facet),
        }
        self.fm(tail).previous = Some(facet);
        self.num_facets += 1;
    }

    /// `qh_appendvertex` (`poly_r.c:75-90`): before the tail sentinel.
    pub(crate) fn appendvertex(&mut self, vertex: VertexId) {
        let Some(tail) = self.vertex_tail else { return };
        if Some(tail) == self.newvertex_list {
            self.newvertex_list = Some(vertex);
        }
        self.vm(vertex).newfacet = true;
        let prev = self.v(tail).previous;
        self.vm(vertex).previous = prev;
        self.vm(vertex).next = Some(tail);
        match prev {
            Some(p) => self.vm(p).next = Some(vertex),
            None => self.vertex_list = Some(vertex),
        }
        self.vm(tail).previous = Some(vertex);
        self.num_vertices += 1;
    }

    /// `qh_prependfacet` (`poly2_r.c:3220-3240`): the facet before the head of
    /// the list `head` names, which becomes it (the tail sentinel where the
    /// list is empty).
    pub(crate) fn prependfacet(&mut self, facet: FacetId, head: ListHead) {
        let current = match head {
            ListHead::FacetNext => self.facet_next,
            ListHead::Visible => self.visible_list,
        };
        let Some(list) = current.or(self.facet_tail) else {
            return;
        };
        let prevfacet = self.f(list).previous;
        self.fm(facet).previous = prevfacet;
        if let Some(p) = prevfacet {
            self.fm(p).next = Some(facet);
        }
        self.fm(list).previous = Some(facet);
        self.fm(facet).next = Some(list);
        if self.facet_list == Some(list) {
            self.facet_list = Some(facet);
        }
        if self.facet_next == Some(list) {
            self.facet_next = Some(facet);
        }
        match head {
            ListHead::FacetNext => self.facet_next = Some(facet),
            ListHead::Visible => self.visible_list = Some(facet),
        }
        self.num_facets += 1;
    }

    /// `qh_removefacet` (`poly_r.c:1188-1206`).
    pub(crate) fn removefacet(&mut self, facet: FacetId) {
        let next = self.f(facet).next;
        let previous = self.f(facet).previous;
        if Some(facet) == self.newfacet_list {
            self.newfacet_list = next;
        }
        if Some(facet) == self.facet_next {
            self.facet_next = next;
        }
        if Some(facet) == self.visible_list {
            self.visible_list = next;
        }
        if let Some(p) = previous {
            self.fm(p).next = next;
            if let Some(n) = next {
                self.fm(n).previous = Some(p);
            }
        } else {
            self.facet_list = next;
            if let Some(n) = next {
                self.fm(n).previous = None;
            }
        }
        self.num_facets -= 1;
    }

    /// `qh_removevertex` (`poly_r.c:1219-1233`).
    pub(crate) fn removevertex(&mut self, vertex: VertexId) {
        let next = self.v(vertex).next;
        let previous = self.v(vertex).previous;
        if Some(vertex) == self.newvertex_list {
            self.newvertex_list = next;
        }
        if let Some(p) = previous {
            self.vm(p).next = next;
            if let Some(n) = next {
                self.vm(n).previous = Some(p);
            }
        } else {
            self.vertex_list = next;
            if let Some(n) = next {
                self.vm(n).previous = None;
            }
        }
        self.num_vertices -= 1;
    }

    /// `qh_delfacet` (`poly_r.c:268-301`): out of its list, its sets freed.
    pub(crate) fn delfacet(&mut self, facet: FacetId) {
        self.removefacet(facet);
        let f = self.fm(facet);
        f.neighbors = Vec::new();
        f.ridges = None;
        f.vertices = Vec::new();
        f.outsideset = None;
        f.coplanarset = None;
        f.deleted = true;
    }

    /// `qh_delvertex` (`poly2_r.c:1548-1560`): qhull stops for a deleted
    /// vertex that was not partitioned as a coplanar point (QH6395).
    pub(crate) fn delvertex(&mut self, vertex: VertexId) -> Result<(), Error> {
        if self.v(vertex).deleted && !self.v(vertex).partitioned {
            return Err(Error::qhull(
                6395,
                "vertex was deleted but it was not partitioned as a coplanar point",
            ));
        }
        self.removevertex(vertex);
        let v = self.vm(vertex);
        v.neighbors = Vec::new();
        v.freed = true;
        Ok(())
    }

    /// `qh_delridge` (`poly2_r.c:1530-1536`).
    pub(crate) fn delridge(&mut self, ridge: RidgeId) {
        let r = self.rm(ridge);
        r.vertices = Vec::new();
        r.deleted = true;
    }

    /// `qh_resetlists` (`poly2_r.c:3391-3427`); its statistics dropped.
    pub(crate) fn resetlists(&mut self, reset_visible: bool) {
        for v in self.vertices_from(self.newvertex_list) {
            self.vm(v).newfacet = false;
        }
        self.newvertex_list = None;
        self.first_newfacet = 0;
        for f in self.facets_from(self.newfacet_list) {
            let facet = self.fm(f);
            facet.newfacet = false;
            facet.dupridge = false;
        }
        self.newfacet_list = None;
        if reset_visible {
            let mut cur = self.visible_list;
            while let Some(v) = cur {
                if !self.f(v).visible {
                    break;
                }
                self.fm(v).f = None;
                self.fm(v).visible = false;
                cur = self.f(v).next;
            }
            self.num_visible = 0;
        }
        self.visible_list = None;
        self.NEWfacets = false;
        self.NEWtentative = false;
    }

    /// `qh_pointid` of a vertex's point: `qh_IDnone` (-3) for none.
    pub(crate) fn pointid(&self, vertex: VertexId) -> i32 {
        self.v(vertex).point.map_or(-3, |p| p as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_vertex_deleted_but_not_partitioned_stops_qhull() {
        let mut qh = Qhull::new(&[], 0);
        let vertex = qh.newvertex(None);
        qh.vm(vertex).deleted = true;
        assert!(matches!(
            qh.delvertex(vertex),
            Err(Error::Qhull { code: 6395, .. })
        ));
    }
}
