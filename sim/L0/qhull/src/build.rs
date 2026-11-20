// Ported from Qhull, libqhull_r (libqhull_r.c, poly_r.c, poly2_r.c, merge_r.c's
// qh_checkzero).
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// libqhull_r.c, poly_r.c, poly2_r.c: Copyright (c) 1993-2020 The Geometry Center.
// merge_r.c: Copyright (c) 1993-2020 C.B. Barber.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! The hull built point by point (`qh_qhull`): the initial simplex, the
//! outside sets, and for each furthest point its visible facets, the cone of
//! new facets over their horizon, and the points repartitioned onto it.

use crate::error::Error;
use crate::set;
use crate::state::{DIM, FacetId, PointId, Qhull, REAL_MAX, VertexId, fmax_, maximize_, minimize_};

/// `qh_MAXnarrow` (`user_r.h:919`).
const MAX_NARROW: f64 = -0.999_999_99;
/// `qh_RATIOcoplanaroutside` (`user_r.h:820`).
const RATIO_COPLANAROUTSIDE: f64 = 30.0;
/// `qh_WIDEmaxoutside` (`user_r.h:983`).
const WIDE_MAXOUTSIDE: f64 = 100.0;
/// `qh_WIDEmaxoutside2` (`user_r.h:993`).
const WIDE_MAXOUTSIDE2: f64 = 10.0 * WIDE_MAXOUTSIDE;
/// `qh_HASHfactor` (`user_r.h:430`).
const HASH_FACTOR: i32 = 2;

impl Qhull<'_> {
    /// `qh_qhull` (`libqhull_r.c:60-119`) for `qhull Qt`: the build, then, where
    /// no `TA` stopped it, `qh_check_maxout`. Merging is
    /// [`Self::premerge`]'s, after each point.
    pub(crate) fn qhull(&mut self) -> Result<(), Error> {
        self.initbuild()?;
        self.buildhull()?;
        // `!qh->STOPadd`: under `TA` none of this runs, stopped early or not.
        if self.STOPadd == 0 {
            if self.ZEROall_ok && !self.WAScoplanar {
                self.DOcheckmax = false;
            }
            // Otherwise `qh_initmergesets`, neither post-merge (3-d,
            // `PREmerge`), and `qh_all_vertexmerges`, which merges no vertex
            // on this path.
            if self.DOcheckmax {
                self.check_maxout()?;
            }
            // `KEEPnearinside` holds.
            if !self.maxoutdone {
                self.nearcoplanar();
            }
        }
        Ok(())
    }

    /// `qh_initbuild` (`poly2_r.c:2111-2225`).
    fn initbuild(&mut self) -> Result<(), Error> {
        self.furthest_id = -1;
        self.facet_id = 0;
        self.vertex_id = 0;
        self.ridge_id = 0;
        self.visit_id = 0;
        self.vertex_visit = 0;
        self.maxoutdone = false;
        let maxpoints = self.maxmin();
        self.detroundoff();
        let tail = self.newfacet();
        self.facet_list = Some(tail);
        self.newfacet_list = Some(tail);
        self.facet_tail = Some(tail);
        self.num_facets = 0;
        self.num_vertices = 0;
        self.num_visible = 0;
        let vtail = self.newvertex(None);
        self.vertex_list = Some(vtail);
        self.newvertex_list = Some(vtail);
        self.vertex_tail = Some(vtail);
        let vertices = self.initialvertices(&maxpoints)?;
        self.initialhull(&vertices)?;
        self.partitionall(&vertices)?;
        self.resetlists(true);
        self.facet_next = self.facet_list;
        self.furthestnext();
        self.cos_max = self.premerge_cos;
        self.centrum_radius = self.premerge_centrum;
        Ok(())
    }

    /// `qh_initialvertices` for 3-d (`poly2_r.c:2348-2431`): the simplex's
    /// vertices, made in its order and kept in decreasing id order.
    fn initialvertices(&mut self, maxpoints: &[PointId]) -> Result<Vec<VertexId>, Error> {
        let simplex = self.maxsimplex(maxpoints)?;
        let mut vertices = Vec::with_capacity(DIM + 1);
        for p in simplex {
            let v = self.newvertex(Some(p));
            set::setaddnth(&mut vertices, 0, v);
        }
        Ok(vertices)
    }

    /// `qh_createsimplex` (`poly2_r.c:1491-1519`): one facet per vertex, each
    /// on the others, alternately top-oriented, all neighbours of each other.
    fn createsimplex(&mut self, vertices: &[VertexId]) {
        let mut toporient = true;
        for (vertex_i, &vertex) in vertices.iter().enumerate() {
            let newfacet = self.newfacet();
            self.fm(newfacet).vertices = set::setnew_delnthsorted(vertices, vertex_i, 0);
            if toporient {
                self.fm(newfacet).toporient = true;
            }
            self.appendfacet(newfacet);
            self.fm(newfacet).newfacet = true;
            self.appendvertex(vertex);
            toporient = !toporient;
        }
        for newfacet in self.facets_from(self.newfacet_list) {
            let others: Vec<FacetId> = self
                .facets_from(self.newfacet_list)
                .into_iter()
                .filter(|&f| f != newfacet)
                .collect();
            self.fm(newfacet).neighbors = others;
        }
    }

    /// `qh_initialhull` (`poly2_r.c:2244-2328`): the simplex, oriented so
    /// the interior point lies below every facet.
    fn initialhull(&mut self, vertices: &[VertexId]) -> Result<(), Error> {
        self.createsimplex(vertices);
        self.resetlists(true);
        self.facet_next = self.facet_list;
        self.interior_point = self.getcenter(vertices);
        let Some(firstfacet) = self.facet_list else {
            return Ok(());
        };
        self.setfacetplane(firstfacet);
        if self.f(firstfacet).flipped {
            self.fm(firstfacet).flipped = false;
        }
        let dist = self.distplane(&self.interior_point, firstfacet);
        if dist > self.DISTround {
            for facet in self.facets_from(self.facet_list) {
                let f = self.fm(facet);
                f.toporient = !f.toporient;
            }
            self.setfacetplane(firstfacet);
        }
        for facet in self.facets_from(self.facet_list) {
            if facet != firstfacet {
                self.setfacetplane(facet);
            }
        }
        for facet in self.facets_from(self.facet_list) {
            if self.f(facet).flipped {
                self.fm(facet).flipped = false;
            }
            let dist = self.distplane(&self.interior_point, facet);
            if dist > self.DISTround {
                for facet in self.facets_from(self.facet_list) {
                    self.fm(facet).flipped = false;
                    let f = self.fm(facet);
                    f.toporient = !f.toporient;
                    self.orientoutside(facet);
                }
                break;
            }
        }
        let mut minangle = REAL_MAX;
        for facet in self.facets_from(self.facet_list) {
            if !self.checkflipped(facet, true) {
                return Err(Error::qhull(
                    6154,
                    "Initial simplex is flat (facet is coplanar with the interior point)",
                ));
            }
            for neighbor in self.f(facet).neighbors.clone() {
                let angle = Self::getangle(&self.normal(facet), &self.normal(neighbor));
                minimize_(&mut minangle, angle);
            }
        }
        if minangle < MAX_NARROW {
            self.NARROWhull = true;
        }
        self.checkpolygon_marks();
        self.checkconvex_initial()
    }

    /// What `qh_checkpolygon` (`poly2_r.c:1200-1400`) leaves on the initial
    /// simplex besides its checks, which cannot fail on it: every vertex
    /// seen, its visit mark its count of facets, and `qh.vertex_visit`
    /// advanced past them.
    fn checkpolygon_marks(&mut self) {
        let vertices = self.vertices_from(self.vertex_list);
        for &v in &vertices {
            self.vm(v).seen = false;
            self.vm(v).visitid = 0;
        }
        let facets = self.facets_from(self.facet_list);
        for &facet in &facets {
            for v in self.f(facet).vertices.clone() {
                self.vm(v).visitid += 1;
                self.vm(v).seen = true;
            }
        }
        self.vertex_visit += facets.len() as u32;
        self.vertex_visit += 1;
    }

    /// `qh_checkconvex` with `qh_DATAfault` over the initial simplex
    /// (`poly2_r.c:539-673`): each vertex clearly below its opposite facet.
    fn checkconvex_initial(&self) -> Result<(), Error> {
        for facet in self.facets_from(self.facet_list) {
            if self.f(facet).flipped {
                return Err(Error::qhull(
                    6113,
                    "f is flipped (interior point is outside)",
                ));
            }
            for (neighbor_i, &neighbor) in self.f(facet).neighbors.iter().enumerate() {
                let vertex = self.f(facet).vertices[neighbor_i];
                let dist = self.distplane(self.vcoords(vertex), neighbor);
                if dist >= -self.DISTround {
                    return Err(if dist > self.DISTround {
                        Error::qhull(6114, "initial simplex is not convex")
                    } else {
                        Error::qhull(
                            6379,
                            "initial simplex is not convex (a vertex within roundoff of the opposite facet)",
                        )
                    });
                }
            }
        }
        Ok(())
    }

    /// `qh_partitionall` (`libqhull_r.c:1053-1133`): every point not a
    /// vertex into the outside set of the first facet it is clearly above,
    /// the furthest last; the rest by `qh_partitionpoint` (merging).
    fn partitionall(&mut self, vertices: &[VertexId]) -> Result<(), Error> {
        let numpoints = self.points.len();
        let mut pointset: Vec<Option<PointId>> = (0..numpoints as PointId).map(Some).collect();
        for &v in vertices {
            let id = self.vpoint(v) as usize;
            pointset[id] = None;
        }
        self.num_outside = 0;
        let distoutside = self.dist_outside();
        let mut point_end = numpoints;
        for facet in self.facets_from(self.facet_list) {
            let mut outsideset: Vec<PointId> = Vec::new();
            let mut bestpoint: Option<PointId> = None;
            let mut bestdist = -REAL_MAX;
            let mut kept = 0;
            for i in 0..point_end {
                let Some(point) = pointset[i] else { continue };
                let dist = self.distplane_point(point, facet);
                if dist < distoutside {
                    pointset[kept] = Some(point);
                    kept += 1;
                } else {
                    self.num_outside += 1;
                    match bestpoint {
                        None => {
                            bestpoint = Some(point);
                            bestdist = dist;
                        }
                        Some(b) if dist > bestdist => {
                            outsideset.push(b);
                            bestpoint = Some(point);
                            bestdist = dist;
                        }
                        Some(_) => outsideset.push(point),
                    }
                }
            }
            if let Some(b) = bestpoint {
                outsideset.push(b);
                let f = self.fm(facet);
                f.outsideset = Some(outsideset);
                f.furthestdist = bestdist;
            } else {
                self.fm(facet).outsideset = None;
            }
            point_end = kept;
        }
        self.findbestnew = true;
        for &entry in pointset.iter().take(point_end) {
            if let Some(point) = entry
                && let Some(start) = self.facet_list
            {
                self.partitionpoint(point, start)?;
            }
        }
        self.findbestnew = false;
        Ok(())
    }

    /// `qh_furthestnext` (`poly2_r.c:2003-2029`): the facet of the furthest
    /// outside point moved to `qh.facet_next`.
    fn furthestnext(&mut self) {
        let mut bestfacet = None;
        let mut bestdist = -REAL_MAX;
        for facet in self.facets_from(self.facet_list) {
            if self.f(facet).outsideset.is_some() {
                let dist = self.f(facet).furthestdist;
                if dist > bestdist {
                    bestfacet = Some(facet);
                    bestdist = dist;
                }
            }
        }
        if let Some(b) = bestfacet {
            self.removefacet(b);
            self.prependfacet(b, crate::state::ListHead::FacetNext);
        }
    }

    /// `qh_buildhull` (`libqhull_r.c:589-635`): each next furthest point
    /// added, until none is outside or `TA` stops it.
    fn buildhull(&mut self) -> Result<(), Error> {
        self.facet_next = self.facet_list;
        while let Some((furthest, facet)) = self.nextfurthest() {
            self.num_outside -= 1;
            // qhull's test as written (`libqhull_r.c:621`).
            #[allow(clippy::int_plus_one)]
            let stop = self.STOPadd > 0 && (self.num_vertices - DIM as i32 - 1 >= self.STOPadd - 1);
            if stop {
                return Ok(());
            }
            self.addpoint(furthest, facet)?;
        }
        if self.NARROWhull {
            self.outcoplanar()?;
        }
        Ok(())
    }

    /// `qh_nextfurthest` without `qh.RANDOMoutside` (`libqhull_r.c:931-1035`):
    /// the last (furthest) point of the first facet from `qh.facet_next`
    /// with a non-empty outside set.
    fn nextfurthest(&mut self) -> Option<(PointId, FacetId)> {
        while let Some(facet) = self.facet_next {
            if Some(facet) == self.facet_tail {
                return None;
            }
            let Some(size) = self.f(facet).outsideset.as_ref().map(Vec::len) else {
                self.facet_next = self.f(facet).next;
                continue;
            };
            if size == 0 {
                self.fm(facet).outsideset = None;
                self.facet_next = self.f(facet).next;
                continue;
            }
            if self.NARROWhull {
                // `notfurthest` is set only under options MuJoCo does not pass
                // and after a pinched apex, which is refused, so
                // `qh_furthestout` is never called here.
                // `qh_COMPUTEfurthest` is 0: the stored distance.
                let dist = self.f(facet).furthestdist;
                if dist < self.MINoutside {
                    self.facet_next = self.f(facet).next;
                    continue;
                }
            }
            let point = self.fm(facet).outsideset.as_mut().and_then(Vec::pop);
            return point.map(|p| (p, facet));
        }
        None
    }

    /// `qh_addpoint` without `checkdist` (`libqhull_r.c:176-317`).
    fn addpoint(&mut self, furthest: PointId, facet: FacetId) -> Result<(), Error> {
        self.maxoutdone = false;
        self.detmaxoutside();
        self.furthest_id = furthest as i32;
        self.findhorizon(furthest, facet)?;
        self.buildcone(furthest)?;
        self.retry_addpoint = 0;
        self.findbestnew = false;
        self.premerge()?;
        if self.use_findbestnew() {
            self.findbestnew = true;
        } else {
            for newfacet in self.facets_from(self.newfacet_list) {
                if !self.f(newfacet).simplicial {
                    self.findbestnew = true;
                    break;
                }
            }
        }
        self.partitionvisible()?;
        self.findbestnew = false;
        self.findbest_notsharp = false;
        self.deletevisible()?;
        self.NEWfacets = false;
        self.resetlists(true);
        // `qh_all_vertexmerges`: no vertex merge is ever queued on this path.
        // `qh_freemergesets` stops qhull where a merge is left (QH6389).
        if !self.facet_mergeset.is_empty() || !self.degen_mergeset.is_empty() {
            return Err(Error::qhull(6389, "expecting empty mergesets"));
        }
        Ok(())
    }

    /// `qh_findhorizon` (`libqhull_r.c:803-890`): the facets visible from
    /// `point`, from `facet` across neighbours, moved to the end of the
    /// facet list as `qh.visible_list`; the rest met are the horizon.
    fn findhorizon(&mut self, point: PointId, facet: FacetId) -> Result<(), Error> {
        let p = *self.point(point);
        self.removefacet(facet);
        self.appendfacet(facet);
        self.num_visible = 1;
        self.visible_list = Some(facet);
        self.fm(facet).visible = true;
        self.fm(facet).f = None;
        self.visit_id += 1;
        let visit = self.visit_id;
        let mut numhorizon = 0;
        let mut cur = self.visible_list;
        while let Some(visible) = cur {
            if !self.f(visible).visible {
                break;
            }
            self.fm(visible).visitid = visit;
            let mut i = 0;
            while let Some(neighbor) = set::get(&self.f(visible).neighbors, i) {
                i += 1;
                if self.f(neighbor).visitid == visit {
                    continue;
                }
                self.fm(neighbor).visitid = visit;
                let dist = self.distplane(&p, neighbor);
                if dist > self.MINvisible {
                    self.removefacet(neighbor);
                    self.appendfacet(neighbor);
                    self.fm(neighbor).visible = true;
                    self.fm(neighbor).f = None;
                    self.num_visible += 1;
                } else if dist >= -self.MAXcoplanar {
                    numhorizon += 1;
                    self.fm(neighbor).coplanarhorizon = true;
                    if dist > 0.0 {
                        maximize_(&mut self.max_outside, dist);
                        maximize_(&mut self.max_vertex, dist);
                        maximize_(&mut self.fm(neighbor).maxoutside, dist);
                    } else {
                        minimize_(&mut self.min_vertex, dist);
                    }
                } else {
                    numhorizon += 1;
                    self.fm(neighbor).coplanarhorizon = false;
                }
            }
            cur = self.f(visible).next;
        }
        if numhorizon == 0 {
            return Err(Error::qhull(
                6168,
                "empty horizon.  It was above all facets.",
            ));
        }
        Ok(())
    }

    /// `qh_buildcone` without `ONLYgood` or `MERGEpinched`
    /// (`libqhull_r.c:401-437`): the new facets, their neighbours matched,
    /// their hyperplanes, the vertices' neighbours updated.
    fn buildcone(&mut self, furthest: PointId) -> Result<VertexId, Error> {
        self.first_newfacet = self.facet_id;
        self.NEWtentative = false;
        let apex = self.makenewfacets(furthest)?;
        self.matchnewfacets()?;
        self.makenewplanes();
        self.update_vertexneighbors_cone();
        Ok(apex)
    }

    /// `qh_makenewfacets` (`poly2_r.c:2489-2530`): the apex vertex, and a new
    /// facet for each horizon ridge of each visible facet.
    fn makenewfacets(&mut self, point: PointId) -> Result<VertexId, Error> {
        self.newfacet_list = self.facet_tail;
        self.newvertex_list = self.vertex_tail;
        let apex = self.newvertex(Some(point));
        self.appendvertex(apex);
        self.visit_id += 1;
        // Declared outside the loop, as qhull declares them: a visible facet
        // whose cone gives no new facet keeps the previous one's.
        let mut newfacet: Option<FacetId> = None;
        let mut newfacet2: Option<FacetId> = None;
        let mut cur = self.visible_list;
        while let Some(visible) = cur {
            if !self.f(visible).visible {
                break;
            }
            let mut i = 0;
            while let Some(neighbor) = set::get(&self.f(visible).neighbors, i) {
                i += 1;
                self.fm(neighbor).seen = false;
            }
            if self.f(visible).ridges.is_some() {
                self.fm(visible).visitid = self.visit_id;
                newfacet2 = self.makenew_nonsimplicial(visible, apex)?;
            }
            if self.f(visible).simplicial {
                newfacet = self.makenew_simplicial(visible, apex);
            }
            if newfacet2.is_some() {
                newfacet = newfacet2;
            }
            if let Some(n) = newfacet {
                self.fm(visible).f = Some(n);
            }
            if let Some(ridges) = self.fm(visible).ridges.as_mut()
                && let Some(first) = ridges.first_mut()
            {
                *first = <crate::state::RidgeId as set::SetElem>::NULL;
            }
            if let Some(first) = self.fm(visible).neighbors.first_mut() {
                *first = FacetId::NULL;
            }
            cur = self.f(visible).next;
        }
        self.NEWfacets = true;
        Ok(apex)
    }

    /// `qh_makenew_simplicial` (`poly_r.c:726-757`): a new facet over each
    /// unseen, not visible neighbour of a simplicial visible facet.
    fn makenew_simplicial(&mut self, visible: FacetId, apex: VertexId) -> Option<FacetId> {
        let mut newfacet = None;
        let mut i = 0;
        while let Some(neighbor) = set::get(&self.f(visible).neighbors, i) {
            i += 1;
            if !self.f(neighbor).seen && !self.f(neighbor).visible {
                let (mut vertices, horizonskip, _visibleskip) =
                    self.facetintersect(neighbor, visible);
                vertices[0] = apex;
                let toporient = if self.f(neighbor).toporient {
                    horizonskip & 0x1 == 1
                } else {
                    (horizonskip & 0x1) ^ 0x1 == 1
                };
                let nf = self.makenewfacet(vertices, toporient, neighbor);
                if self.f(neighbor).coplanarhorizon {
                    self.fm(nf).f = Some(nf);
                    self.fm(nf).mergehorizon = true;
                }
                self.fm(neighbor).neighbors[horizonskip] = nf;
                newfacet = Some(nf);
            }
        }
        newfacet
    }

    /// `qh_facetintersect` with `prepend` 1 (`poly_r.c:374-420`): the
    /// vertices `facet_a` shares with its neighbour `facet_b`, a slot before
    /// them, and where each is in the other's neighbours.
    fn facetintersect(&self, facet_a: FacetId, facet_b: FacetId) -> (Vec<VertexId>, usize, usize) {
        let skip_a = self
            .f(facet_a)
            .neighbors
            .iter()
            .take(DIM)
            .position(|&n| n == facet_b)
            .unwrap_or(0);
        let skip_b = self
            .f(facet_b)
            .neighbors
            .iter()
            .take(DIM)
            .position(|&n| n == facet_a)
            .unwrap_or(0);
        let intersect = set::setnew_delnthsorted(&self.f(facet_a).vertices, skip_a, 1);
        (intersect, skip_a, skip_b)
    }

    /// `qh_makenewfacet` (`poly_r.c:531-549`): its vertices moved to the end
    /// of the vertex list where not new already, the facet appended with its
    /// horizon facet as its first neighbour.
    pub(crate) fn makenewfacet(
        &mut self,
        vertices: Vec<VertexId>,
        toporient: bool,
        horizon: FacetId,
    ) -> FacetId {
        for &v in &vertices {
            if !self.v(v).newfacet {
                self.removevertex(v);
                self.appendvertex(v);
            }
        }
        let newfacet = self.newfacet();
        self.fm(newfacet).vertices = vertices;
        if toporient {
            self.fm(newfacet).toporient = true;
        }
        self.fm(newfacet).neighbors.push(horizon);
        self.appendfacet(newfacet);
        newfacet
    }

    /// `qh_matchnewfacets` (`poly_r.c:937-1030`): each new facet's other
    /// neighbours, by hashing its ridges.
    pub(crate) fn matchnewfacets(&mut self) -> Result<(), Error> {
        let mut numnew = 0;
        for newfacet in self.facets_from(self.newfacet_list) {
            numnew += 1;
            let n = &mut self.fm(newfacet).neighbors;
            n.truncate(1);
            n.resize(DIM, FacetId::NULL);
        }
        let hashsize = newhashtable(numnew * (DIM as i32 - 1));
        let mut hash_table: Vec<FacetId> = vec![FacetId::NULL; hashsize];
        let mut hashcount = 0;
        for newfacet in self.facets_from(self.newfacet_list) {
            if !self.f(newfacet).simplicial {
                return Err(Error::qhull(
                    6377,
                    "expecting simplicial facets on qh.newfacet_list",
                ));
            }
            for newskip in 1..DIM {
                self.matchneighbor(newfacet, newskip, &mut hash_table, &mut hashcount)?;
            }
        }
        // `matchneighbor` refuses every duplicate ridge, so what is left
        // unmatched stops qhull (QH6108).
        if hashcount != 0 {
            return Err(Error::qhull(6108, "neighbors did not match up"));
        }
        Ok(())
    }

    /// `qh_matchneighbor` (`poly_r.c:818-893`) where the ridge's other facet
    /// is found unmatched or not yet hashed; a ridge of more than two new
    /// facets (a dupridge) is not ported.
    fn matchneighbor(
        &mut self,
        newfacet: FacetId,
        newskip: usize,
        hash_table: &mut [FacetId],
        hashcount: &mut i32,
    ) -> Result<(), Error> {
        let hashsize = hash_table.len();
        let skipelem = self.f(newfacet).vertices[newskip];
        let mut scan = gethash(hashsize, &self.f(newfacet).vertices, 1, skipelem);
        let mut newfound = false;
        loop {
            let facet = hash_table[scan];
            if facet == FacetId::NULL {
                break;
            }
            if facet == newfacet {
                newfound = true;
            } else if let Some((skip, same)) =
                matchvertices(&self.f(newfacet).vertices, newskip, &self.f(facet).vertices)
            {
                let ismatch = same == (self.f(newfacet).toporient ^ self.f(facet).toporient);
                let matchfacet = self.f(facet).neighbors[skip];
                if ismatch && matchfacet == FacetId::NULL {
                    self.fm(facet).neighbors[skip] = newfacet;
                    self.fm(newfacet).neighbors[newskip] = facet;
                    *hashcount -= 1;
                    return Ok(());
                }
                return Err(Error::Unported {
                    routine: "qh_matchneighbor (a ridge of more than two new facets)",
                });
            }
            scan += 1;
            if scan >= hashsize {
                scan = 0;
            }
        }
        if !newfound {
            hash_table[scan] = newfacet;
        }
        *hashcount += 1;
        Ok(())
    }

    /// `qh_makenewplanes` (`poly_r.c:566-577`): a hyperplane for each new
    /// facet not to be merged into its horizon.
    fn makenewplanes(&mut self) {
        for newfacet in self.facets_from(self.newfacet_list) {
            if !self.f(newfacet).mergehorizon {
                self.setfacetplane(newfacet);
            }
        }
    }

    /// `qh_update_vertexneighbors_cone` (`poly_r.c:1387-1447`): with vertex
    /// neighbours, the visible facets out of them and the new facets in;
    /// either way, each vertex left only on visible facets marked deleted.
    fn update_vertexneighbors_cone(&mut self) {
        if self.VERTEXneighbors {
            for vertex in self.vertices_from(self.newvertex_list) {
                let mut i = 0;
                while let Some(neighbor) = set::get(&self.v(vertex).neighbors, i) {
                    if self.f(neighbor).visible {
                        set::setdelnth(&mut self.vm(vertex).neighbors, i);
                    } else {
                        i += 1;
                    }
                }
            }
            for newfacet in self.facets_from(self.newfacet_list) {
                for v in self.f(newfacet).vertices.clone() {
                    self.vm(v).neighbors.push(newfacet);
                }
            }
            let mut cur = self.visible_list;
            while let Some(visible) = cur {
                if !self.f(visible).visible {
                    break;
                }
                for vertex in self.f(visible).vertices.clone() {
                    if !self.v(vertex).newfacet && !self.v(vertex).deleted {
                        // FOREACHneighbor_ to the first neighbour not visible.
                        let mut j = 0;
                        let mut all_visible = true;
                        while let Some(n) = set::get(&self.v(vertex).neighbors, j) {
                            j += 1;
                            if !self.f(n).visible {
                                all_visible = false;
                                break;
                            }
                        }
                        if all_visible {
                            self.vm(vertex).deleted = true;
                            self.del_vertices.push(vertex);
                        } else {
                            set::setdel(&mut self.vm(vertex).neighbors, visible);
                        }
                    }
                }
                cur = self.f(visible).next;
            }
        } else {
            let mut cur = self.visible_list;
            while let Some(visible) = cur {
                if !self.f(visible).visible {
                    break;
                }
                for vertex in self.f(visible).vertices.clone() {
                    if !self.v(vertex).newfacet && !self.v(vertex).deleted {
                        self.vm(vertex).deleted = true;
                        self.del_vertices.push(vertex);
                    }
                }
                cur = self.f(visible).next;
            }
        }
    }

    /// `qh_partitionvisible` without `allpoints` (`libqhull_r.c:1454-1514`):
    /// each visible facet's outside and coplanar points onto the new facets,
    /// and each deleted vertex's point as a coplanar point.
    fn partitionvisible(&mut self) -> Result<(), Error> {
        let mut cur = self.visible_list;
        while let Some(visible) = cur {
            if !self.f(visible).visible {
                break;
            }
            if self.f(visible).outsideset.is_some() || self.f(visible).coplanarset.is_some() {
                let newfacet = match self.getreplacement(visible)? {
                    Some(n) => n,
                    None => match self.newfacet_list {
                        Some(n) => n,
                        None => {
                            return Err(Error::qhull(6170, "all new facets deleted as qh.visible"));
                        }
                    },
                };
                if self.f(newfacet).next.is_none() {
                    return Err(Error::qhull(6170, "all new facets deleted as qh.visible"));
                }
                if let Some(outside) = self.f(visible).outsideset.clone() {
                    self.num_outside -= outside.len() as i32;
                    for point in outside {
                        self.partitionpoint(point, newfacet)?;
                    }
                }
                if let Some(coplanar) = self.f(visible).coplanarset.clone() {
                    let allnew = self.findbestnew;
                    for point in coplanar {
                        self.partitioncoplanar(point, newfacet, None, allnew)?;
                    }
                }
            }
            cur = self.f(visible).next;
        }
        if !self.del_vertices.is_empty() {
            let mut i = 0;
            while let Some(vertex) = set::get(&self.del_vertices, i) {
                i += 1;
                if let Some(point) = self.v(vertex).point
                    && !self.v(vertex).partitioned
                {
                    let Some(start) = self.newfacet_list.filter(|&n| Some(n) != self.facet_tail)
                    else {
                        return Err(Error::qhull(6284, "all new facets deleted or none defined"));
                    };
                    self.partitioncoplanar(point, start, None, true)?;
                    self.vm(vertex).partitioned = true;
                }
            }
        }
        Ok(())
    }

    /// `qh_getreplacement` (`poly_r.c:505-515`): a chain longer than
    /// `qh.facet_id` stops qhull (`qh_infiniteloop`, QH6149).
    pub(crate) fn getreplacement(&self, visible: FacetId) -> Result<Option<FacetId>, Error> {
        let mut count: u32 = 0;
        let mut result = Some(visible);
        while let Some(r) = result {
            if !self.f(r).visible {
                break;
            }
            result = self.f(r).f;
            if count > self.facet_id {
                return Err(Error::qhull(6149, "potential infinite loop detected"));
            }
            count += 1;
        }
        Ok(result)
    }

    /// `qh_deletevisible` (`poly_r.c:323-349`).
    fn deletevisible(&mut self) -> Result<(), Error> {
        let mut cur = self.visible_list;
        while let Some(visible) = cur {
            if !self.f(visible).visible {
                break;
            }
            let next = self.f(visible).next;
            self.delfacet(visible);
            cur = next;
        }
        self.num_visible = 0;
        for vertex in std::mem::take(&mut self.del_vertices) {
            self.delvertex(vertex)?;
        }
        Ok(())
    }

    /// `qh_partitionpoint` (`libqhull_r.c:1326-1418`) with `KEEPnearinside`:
    /// the point into the outside set of its best facet, or kept as a
    /// coplanar point.
    pub(crate) fn partitionpoint(&mut self, point: PointId, facet: FacetId) -> Result<(), Error> {
        let best = if self.findbestnew {
            self.findbestnew(point, facet, false)?
        } else {
            self.findbest(point, facet, false, true)?
        };
        let bestfacet = best.facet;
        let bestdist = best.dist;
        let mut isoutside = best.isoutside;
        if self.f(bestfacet).visible {
            return Err(Error::qhull(
                6293,
                "cannot partition a point into a visible facet",
            ));
        }
        if self.NARROWhull && bestdist >= -self.NEARinside {
            isoutside = true;
        }
        if isoutside {
            let mut isnewoutside = false;
            let empty = self
                .f(bestfacet)
                .outsideset
                .as_ref()
                .is_none_or(Vec::is_empty);
            if empty {
                self.fm(bestfacet)
                    .outsideset
                    .get_or_insert_with(Vec::new)
                    .push(point);
                if !self.NARROWhull || bestdist > self.MINoutside {
                    isnewoutside = true;
                }
                self.fm(bestfacet).furthestdist = bestdist;
            } else {
                let previousdist = self.f(bestfacet).furthestdist;
                if previousdist < bestdist {
                    self.fm(bestfacet)
                        .outsideset
                        .get_or_insert_with(Vec::new)
                        .push(point);
                    self.fm(bestfacet).furthestdist = bestdist;
                    if self.NARROWhull
                        && previousdist < self.MINoutside
                        && bestdist >= self.MINoutside
                    {
                        isnewoutside = true;
                    }
                } else {
                    set::setappend2ndlast(
                        self.fm(bestfacet).outsideset.get_or_insert_with(Vec::new),
                        point,
                    );
                }
            }
            if isnewoutside && self.facet_next != Some(bestfacet) {
                if self.f(bestfacet).newfacet {
                    if let Some(next) = self.facet_next
                        && self.f(next).newfacet
                    {
                        self.facet_next = self.newfacet_list;
                    }
                } else {
                    self.removefacet(bestfacet);
                    self.appendfacet(bestfacet);
                    if self.newfacet_list.is_some() {
                        self.fm(bestfacet).newfacet = true;
                    }
                }
            }
            self.num_outside += 1;
        } else if bestdist >= -self.MAXcoplanar {
            // `(KEEPcoplanar + KEEPnearinside) || bestdist > max_outside`:
            // `KEEPnearinside` holds.
            let allnew = self.findbestnew;
            self.partitioncoplanar(point, bestfacet, Some(bestdist), allnew)?;
        } else if bestdist >= -self.NEARinside {
            let allnew = self.findbestnew;
            self.partitioncoplanar(point, bestfacet, Some(bestdist), allnew)?;
        }
        Ok(())
    }

    /// `qh_partitioncoplanar` with `KEEPnearinside` (`libqhull_r.c:1173-1289`):
    /// the point onto the coplanar set of its best facet, the furthest last;
    /// one too far above it is repartitioned or raises `qh.max_outside`.
    pub(crate) fn partitioncoplanar(
        &mut self,
        point: PointId,
        facet: FacetId,
        dist: Option<f64>,
        allnew: bool,
    ) -> Result<(), Error> {
        self.WAScoplanar = true;
        let (bestfacet, bestdist) = match dist {
            None => {
                let best = if allnew {
                    self.findbestnew(point, facet, true)?
                } else {
                    self.findbest(point, facet, true, false)?
                };
                if best.dist < -self.NEARinside {
                    self.repart_facetid = 0;
                    return Ok(());
                }
                (best.facet, best.dist)
            }
            Some(d) => (facet, d),
        };
        if self.f(bestfacet).visible {
            return Err(Error::qhull(
                6405,
                "cannot partition a coplanar point into a visible facet",
            ));
        }
        if bestdist > self.max_outside {
            let mut repartition = false;
            if dist.is_none() && facet != bestfacet {
                let angle = Self::getangle(&self.normal(facet), &self.normal(bestfacet));
                if angle < 0.0 {
                    repartition = true;
                }
            }
            if !repartition && bestdist > self.MAXoutside * RATIO_COPLANAROUTSIDE {
                // A point well above a twisted or hidden facet: repartitioned
                // as an outside point, once (qhull warns, QH7085/QH7086).
                if self.f(facet).id == self.f(bestfacet).id {
                    if self.f(facet).id == self.repart_facetid {
                        return Err(Error::qhull(
                            6404,
                            "infinite loop due to recursive call to qh_partitionpoint",
                        ));
                    }
                    self.repart_facetid = self.f(facet).id;
                }
                // `point == qh.coplanar_apex` (QH6425): only
                // `qh_buildcone_mergepinched` sets it.
                repartition = true;
            }
            if repartition {
                let oldfindbest = self.findbestnew;
                self.findbestnew = false;
                self.partitionpoint(point, bestfacet)?;
                self.findbestnew = oldfindbest;
                self.repart_facetid = 0;
                return Ok(());
            }
            self.repart_facetid = 0;
            self.max_outside = bestdist;
        }
        let oldfurthest = self
            .f(bestfacet)
            .coplanarset
            .as_ref()
            .and_then(|s| s.last().copied());
        let dist2 = oldfurthest.map_or(0.0, |p| self.distplane_point(p, bestfacet));
        let coplanar = self.fm(bestfacet).coplanarset.get_or_insert_with(Vec::new);
        if oldfurthest.is_none() || dist2 < bestdist {
            coplanar.push(point);
        } else {
            set::setappend2ndlast(coplanar, point);
        }
        Ok(())
    }

    /// `qh_outcoplanar` (`poly2_r.c:3060-3077`): a narrow hull's remaining
    /// outside points made coplanar points.
    fn outcoplanar(&mut self) -> Result<(), Error> {
        for facet in self.facets_from(self.facet_list) {
            if let Some(outside) = self.f(facet).outsideset.clone() {
                for point in outside {
                    self.num_outside -= 1;
                    let dist = self.distplane_point(point, facet);
                    let allnew = self.findbestnew;
                    self.partitioncoplanar(point, facet, Some(dist), allnew)?;
                }
            }
            self.fm(facet).outsideset = None;
        }
        Ok(())
    }

    /// `qh_checkzero` over the new facets (`merge_r.c:785-861`): whether each
    /// is clearly convex to its neighbours and its horizon.
    pub(crate) fn checkzero(&mut self) -> bool {
        let facetlist = self.newfacet_list;
        for facet in self.facets_from(facetlist) {
            let horizon = self.f(facet).neighbors[0];
            if !self.f(horizon).simplicial {
                self.ZEROall_ok = false;
                return false;
            }
            let f = self.f(facet);
            if f.flipped || f.dupridge || f.normal.is_none() {
                self.ZEROall_ok = false;
                return false;
            }
        }
        for facet in self.facets_from(facetlist) {
            self.vertex_visit += 1;
            let vv = self.vertex_visit;
            let mut horizon = None;
            for (neighbor_i, &neighbor) in self.f(facet).neighbors.clone().iter().enumerate() {
                if neighbor_i == 0 {
                    horizon = Some(neighbor);
                    continue;
                }
                let vertex = self.f(facet).vertices[neighbor_i];
                self.vm(vertex).visitid = vv;
                let dist = self.distplane(self.vcoords(vertex), neighbor);
                if dist >= -2.0 * self.DISTround {
                    self.ZEROall_ok = false;
                    return false;
                }
            }
            if let Some(horizon) = horizon {
                for vertex in self.f(horizon).vertices.clone() {
                    if self.v(vertex).visitid != vv {
                        let dist = self.distplane(self.vcoords(vertex), facet);
                        if dist >= -2.0 * self.DISTround {
                            self.ZEROall_ok = false;
                            return false;
                        }
                        break;
                    }
                }
            }
        }
        true
    }

    /// `qh_check_maxout` (`poly2_r.c:198-332`): `qh.max_outside` from every
    /// point's best facet among those coplanar with its own, the facets'
    /// `maxoutside` raised to their points, then `qh_nearcoplanar`.
    fn check_maxout(&mut self) -> Result<(), Error> {
        let minvertex_base = fmin_(self.min_vertex, -(self.ONEmerge + self.DISTround));
        let mut maxoutside = 0.0;
        let mut mindist = 0.0;
        if self.VERTEXneighbors {
            for vertex in self.vertices_from(self.vertex_list) {
                let mut i = 0;
                while let Some(neighbor) = set::get(&self.v(vertex).neighbors, i) {
                    i += 1;
                    let dist = self.distplane(self.vcoords(vertex), neighbor);
                    if dist < mindist {
                        mindist = dist;
                    }
                }
            }
            self.min_vertex = mindist;
        }
        let maxoutside_base = fmax_(self.max_outside, self.ONEmerge + self.DISTround);
        let facets = self.pointfacet();
        for (facet_i, facet) in facets.into_iter().enumerate() {
            let Some(facet) = facet else { continue };
            let point = *self.point(facet_i as PointId);
            let mut dist = self.distplane(&point, facet);
            self.findbesthorizon(true, &point, facet, &mut dist);
            // `bestfacet` is never NULL; a point far above a twisted or hidden
            // facet is qhull's warning (QH7087, QH7088), not an error.
            if dist >= maxoutside {
                maxoutside = dist;
            }
        }
        if maxoutside > self.DISTround {
            for facet in self.facets_from(self.facet_list) {
                if maxoutside < self.f(facet).maxoutside {
                    maxoutside = self.f(facet).maxoutside;
                }
            }
        }
        self.max_outside = maxoutside;
        self.nearcoplanar();
        self.maxoutdone = true;
        if maxoutside / maxoutside_base > WIDE_MAXOUTSIDE {
            return Err(Error::qhull(
                6297,
                "large increase in qh.max_outside during post-processing",
            ));
        } else if maxoutside_base > self.ONEmerge * WIDE_MAXOUTSIDE2 {
            if maxoutside > self.ONEmerge * WIDE_MAXOUTSIDE2 {
                return Err(Error::qhull(
                    6298,
                    "a facet merge, vertex merge, vertex, or coplanar point produced a wide facet",
                ));
            }
        } else if self.min_vertex / minvertex_base > WIDE_MAXOUTSIDE {
            return Err(Error::qhull(
                6354,
                "large increase in qh.min_vertex during post-processing",
            ));
        } else if minvertex_base < -(self.ONEmerge * WIDE_MAXOUTSIDE2)
            && self.min_vertex < -(self.ONEmerge * WIDE_MAXOUTSIDE2)
        {
            return Err(Error::qhull(
                6380,
                "a facet or vertex merge produced a wide facet",
            ));
        }
        Ok(())
    }

    /// `qh_pointfacet` (`poly2_r.c:3156-3179`): each point's facet — a
    /// vertex's first facet in the list, a coplanar or outside point's own.
    fn pointfacet(&mut self) -> Vec<Option<FacetId>> {
        let mut facets = vec![None; self.points.len()];
        self.vertex_visit += 1;
        let vv = self.vertex_visit;
        for facet in self.facets_from(self.facet_list) {
            for vertex in self.f(facet).vertices.clone() {
                if self.v(vertex).visitid != vv {
                    self.vm(vertex).visitid = vv;
                    facets[self.vpoint(vertex) as usize] = Some(facet);
                }
            }
            for &point in self.f(facet).coplanarset.iter().flatten() {
                facets[point as usize] = Some(facet);
            }
            for &point in self.f(facet).outsideset.iter().flatten() {
                facets[point as usize] = Some(facet);
            }
        }
        facets
    }

    /// `qh_nearcoplanar` without `KEEPcoplanar` or `KEEPinside`
    /// (`poly2_r.c:2793-2825`): the coplanar sets freed.
    fn nearcoplanar(&mut self) {
        for facet in self.facets_from(self.facet_list) {
            self.fm(facet).coplanarset = None;
        }
    }
}

/// `fmin_` (`geom_r.h:43`).
fn fmin_(a: f64, b: f64) -> f64 {
    if a > b { b } else { a }
}

/// `qh_newhashtable` (`poly2_r.c:2899-2916`): an odd size, not a multiple of
/// 3 or 5, at least twice `newsize`.
const fn newhashtable(newsize: i32) -> usize {
    let mut size = ((newsize + 1) * HASH_FACTOR) | 0x1;
    while size % 3 == 0 || size % 5 == 0 {
        size += 2;
    }
    size as usize
}

/// `qh_gethash` (`poly_r.c:436-494`) over vertex ids instead of addresses:
/// the slot of the ridge `set[firstindex..]` less `skipelem`. Which slot only
/// orders the probing; a match is the same for any hash where no ridge has
/// more than two new facets (MEASURED: 0 graphs change over 7,167 inputs).
fn gethash(hashsize: usize, set: &[VertexId], firstindex: usize, skipelem: VertexId) -> usize {
    let mut hash: u32 = 0;
    for &v in &set[firstindex..] {
        if v != skipelem {
            hash = hash.wrapping_add(v.0.wrapping_mul(2_654_435_761));
        }
    }
    (hash % hashsize as u32) as usize
}

/// `qh_matchvertices` with `firstindex` 1 (`poly_r.c:1054-1076`): whether
/// `b` holds `a` less its `skip_a`th vertex and one more, and where that one
/// is, with whether the two skips have the same parity.
fn matchvertices(a: &[VertexId], skip_a: usize, b: &[VertexId]) -> Option<(usize, bool)> {
    let mut ib = 1;
    let mut skip_b_plus1: Option<usize> = None;
    let mut ia = 1;
    while ia < a.len() {
        if ia != skip_a {
            loop {
                let elem_b = b.get(ib).copied();
                ib += 1;
                if elem_b == Some(a[ia]) {
                    break;
                }
                if skip_b_plus1.is_some() {
                    return None;
                }
                skip_b_plus1 = Some(ib);
            }
        }
        ia += 1;
    }
    let skip_b = skip_b_plus1.unwrap_or(ib + 1) - 1;
    let same = (skip_a & 0x1) == (skip_b & 0x1);
    Some((skip_b, same))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_chain_of_replacements_that_loops_stops_qhull() {
        let mut qh = Qhull::new(&[], 0);
        let (a, b) = (qh.newfacet(), qh.newfacet());
        for (facet, replace) in [(a, b), (b, a)] {
            qh.fm(facet).visible = true;
            qh.fm(facet).f = Some(replace);
        }
        assert!(matches!(
            qh.getreplacement(a),
            Err(Error::Qhull { code: 6149, .. })
        ));
        qh.fm(b).visible = false;
        assert_eq!(qh.getreplacement(a), Ok(Some(b)));
    }
}
