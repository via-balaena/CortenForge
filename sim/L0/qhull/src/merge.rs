// Ported from Qhull, libqhull_r (merge_r.c; poly2_r.c's qh_addfacetvertex and
// qh_vertexintersect_new; qset_r.c's qh_setequal_except).
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// merge_r.c: Copyright (c) 1993-2020 C.B. Barber.
// poly2_r.c, qset_r.c: Copyright (c) 1993-2020 The Geometry Center.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! Merging after each point (`qh_premerge`, qhull's `C-0`): the new facets
//! coplanar with their horizon merged into it, flipped facets into their best
//! neighbour, and each non-convex ridge's facets into one, then the vertices
//! the merges left shared renamed. Vertex merges for pinched vertices, merge
//! cycles of more than one facet, twisted and mirror merges are not reached by
//! any measured input and return [`Error::Unported`].

use crate::error::Error;
use crate::set;
use crate::state::{
    DIM, FacetId, ListHead, Qhull, REAL_MAX, RidgeId, VertexId, fmax_, maximize_, minimize_,
};

/// `qh_BESTcentrum` (`user_r.h:661`).
const BEST_CENTRUM: usize = 20;
/// `qh_BESTcentrum2` (`user_r.h:662`).
const BEST_CENTRUM2: usize = 2;
/// `qh_BESTnonconvex` (`user_r.h:673`).
const BEST_NONCONVEX: usize = 15;
/// `qh_MAXnewcentrum` (`user_r.h:766`).
const MAX_NEWCENTRUM: usize = 5;
/// `qh_MAXnummerge` (`libqhull_r.h:321`).
const MAX_NUMMERGE: u16 = 511;
/// `qh_WIDEmaxoutside` (`user_r.h:983`).
const WIDE_MAXOUTSIDE: f64 = 100.0;

/// `mergeType` (`merge_r.h:37-56`), numbered as qhull numbers them: the sort
/// takes a lower type first.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub(crate) enum MergeType {
    Coplanar = 1,
    Concave = 3,
    ConcaveCoplanar = 4,
    Flip = 6,
    Degen = 10,
    Redundant = 11,
    CoplanarHorizon = 13,
}

/// qhull's `mergeT` for a facet merge: its vertex and ridge fields serve only
/// the vertex merges, which are not ported.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Merge {
    pub(crate) facet1: FacetId,
    pub(crate) facet2: FacetId,
    pub(crate) mergetype: MergeType,
    pub(crate) distance: f64,
}

/// The distances `qh_mergefacet` is told the merge moves vertices by:
/// qhull's `mindist`/`maxdist` pointers, NULL for a merge into a coplanar
/// horizon and a redundant facet's.
#[derive(Clone, Copy, Debug)]
pub(crate) struct MergeDist {
    pub(crate) mindist: f64,
    pub(crate) maxdist: f64,
}

impl Qhull<'_> {
    /// `qh_premerge` (`merge_r.c:75-99`).
    pub(crate) fn premerge(&mut self) -> Result<(), Error> {
        if self.checkzero() {
            return Ok(());
        }
        self.centrum_radius = self.premerge_centrum;
        self.cos_max = self.premerge_cos;
        // Each call takes `qh.newfacet_list` as it is then: a merge moves the
        // facet it merges into to the list's end (`qh_mergefacet`), so the
        // list's head can change between them.
        self.mark_dupridges(self.newfacet_list)?;
        let mut othermerge = false;
        self.mergecycle_all(self.newfacet_list, &mut othermerge)?;
        // `qh_forcedmerges` (`merge_r.c:1542-1642`) merges only dupridges,
        // which `qh_matchnewfacets` does not let through.
        self.flippedmerges(self.newfacet_list, &mut othermerge)?;
        self.POSTmerging = false;
        self.getmergeset_initial(self.newfacet_list)?;
        self.all_merges(othermerge)
    }

    /// `qh_mark_dupridges` (`merge_r.c:2192-2269`) where no new facet has a
    /// dupridge: the merge-ridge flags cleared.
    fn mark_dupridges(&mut self, facetlist: Option<FacetId>) -> Result<(), Error> {
        for facet in self.facets_from(facetlist) {
            let f = self.fm(facet);
            f.mergeridge2 = false;
            f.mergeridge = false;
        }
        for facet in self.facets_from(facetlist) {
            if self.f(facet).dupridge {
                return Err(Error::Unported {
                    routine: "qh_mark_dupridges",
                });
            }
        }
        Ok(())
    }

    /// `qh_mergecycle_all` (`merge_r.c:2903-2978`): each new facet coplanar
    /// with its horizon facet merged into it.
    fn mergecycle_all(
        &mut self,
        facetlist: Option<FacetId>,
        wasmerge: &mut bool,
    ) -> Result<(), Error> {
        let mut cycles = 0;
        let mut cur = facetlist;
        while let Some(facet) = cur {
            let Some(nextfacet) = self.f(facet).next else {
                break;
            };
            if self.f(facet).normal.is_some() {
                cur = Some(nextfacet);
                continue;
            }
            if !self.f(facet).mergehorizon {
                return Err(Error::qhull(6225, "f without normal"));
            }
            let horizon = self.f(facet).neighbors[0];
            if self.f(facet).f == Some(facet) {
                let vertices = self.f(facet).vertices.clone();
                let apex = vertices[0];
                for &v in &vertices {
                    if v != apex {
                        self.vm(v).delridge = true;
                    }
                }
                self.fm(horizon).f = None;
                self.mergefacet(facet, horizon, MergeType::CoplanarHorizon, None, true)?;
            } else {
                return Err(Error::Unported {
                    routine: "qh_mergecycle",
                });
            }
            cycles += 1;
            cur = Some(nextfacet);
        }
        if cycles > 0 {
            for newfacet in self.facets_from(self.newfacet_list) {
                if self.f(newfacet).coplanarhorizon {
                    self.test_redundant_neighbors(newfacet)?;
                    // `qh_maybe_duplicateridges`: `qh.CHECKduplicates` is off.
                    self.fm(newfacet).coplanarhorizon = false;
                }
            }
            self.merge_degenredundant()?;
            *wasmerge = true;
        }
        Ok(())
    }

    /// `qh_flippedmerges` (`merge_r.c:1459-1508`): each flipped new facet
    /// merged into its best neighbour.
    fn flippedmerges(
        &mut self,
        facetlist: Option<FacetId>,
        wasmerge: &mut bool,
    ) -> Result<(), Error> {
        for facet in self.facets_from(facetlist) {
            if self.f(facet).flipped && !self.f(facet).visible {
                self.appendmergeset(facet, facet, MergeType::Flip, 0.0)?;
            }
        }
        let othermerges = std::mem::take(&mut self.facet_mergeset);
        let mut nummerge = 0;
        for merge in &othermerges {
            let facet1 = merge.facet1;
            if merge.mergetype != MergeType::Flip || self.f(facet1).visible {
                continue;
            }
            let (neighbor, _dist, mindist, maxdist) = self.findbestneighbor(facet1)?;
            self.mergefacet(
                facet1,
                neighbor,
                merge.mergetype,
                Some(MergeDist { mindist, maxdist }),
                false,
            )?;
            nummerge += 1;
        }
        for merge in othermerges {
            if !(self.f(merge.facet1).visible || self.f(merge.facet2).visible) {
                self.facet_mergeset.push(merge);
            }
        }
        self.merge_degenredundant()?;
        if nummerge > 0 {
            *wasmerge = true;
        }
        Ok(())
    }

    /// `qh_getmergeset_initial` (`merge_r.c:1775-1816`): each new facet
    /// tested against each neighbour, the merges sorted.
    fn getmergeset_initial(&mut self, facetlist: Option<FacetId>) -> Result<(), Error> {
        self.visit_id += 1;
        let visit = self.visit_id;
        for facet in self.facets_from(facetlist) {
            self.fm(facet).visitid = visit;
            let mut i = 0;
            while let Some(neighbor) = set::get(&self.f(facet).neighbors, i) {
                i += 1;
                if self.f(neighbor).visitid != visit {
                    let simplicial = self.f(facet).simplicial && self.f(neighbor).simplicial;
                    if self.test_appendmerge(facet, neighbor, simplicial)? {
                        let ridges = self.f(neighbor).ridges.clone().unwrap_or_default();
                        let mut j = 0;
                        while let Some(ridge) = set::get(&ridges, j) {
                            j += 1;
                            if Some(facet) == self.otherfacet(ridge, neighbor) {
                                self.rm(ridge).nonconvex = true;
                                break;
                            }
                        }
                    }
                }
            }
            self.fm(facet).tested = true;
            for ridge in self.f(facet).ridges.clone().unwrap_or_default() {
                if ridge == <RidgeId as set::SetElem>::NULL {
                    break;
                }
                self.rm(ridge).tested = true;
            }
        }
        self.sort_mergeset();
        Ok(())
    }

    /// `qh_getmergeset` (`merge_r.c:1698-1748`): each untested new facet's
    /// untested or non-convex ridges tested, one per neighbour, the merges
    /// sorted.
    fn getmergeset(&mut self, facetlist: Option<FacetId>) -> Result<(), Error> {
        self.visit_id += 1;
        let visit = self.visit_id;
        for facet in self.facets_from(facetlist) {
            if self.f(facet).tested {
                continue;
            }
            self.fm(facet).visitid = visit;
            let mut i = 0;
            while let Some(neighbor) = set::get(&self.f(facet).neighbors, i) {
                i += 1;
                self.fm(neighbor).seen = false;
            }
            let ridges = self.f(facet).ridges.clone().unwrap_or_default();
            let mut j = 0;
            while let Some(ridge) = set::get(&ridges, j) {
                j += 1;
                if self.r(ridge).tested && !self.r(ridge).nonconvex {
                    continue;
                }
                let Some(neighbor) = self.otherfacet(ridge, facet) else {
                    continue;
                };
                if self.f(neighbor).seen {
                    self.rm(ridge).tested = true;
                    self.rm(ridge).nonconvex = false;
                } else if self.f(neighbor).visitid != visit {
                    self.fm(neighbor).seen = true;
                    self.rm(ridge).nonconvex = false;
                    let simplicial = self.r(ridge).simplicialbot && self.r(ridge).simplicialtop;
                    if self.test_appendmerge(facet, neighbor, simplicial)? {
                        self.rm(ridge).nonconvex = true;
                    }
                    self.rm(ridge).tested = true;
                }
            }
            self.fm(facet).tested = true;
        }
        self.sort_mergeset();
        Ok(())
    }

    /// `qsort` by `qh_compare_facetmerge` (`merge_r.c:895-903`), made stable:
    /// a higher merge type first, then a larger distance first, so
    /// `qh_all_merges`, taking from the end, merges the lowest type and the
    /// flattest first. Equal merges keep their order (MEASURED: macOS's
    /// `qsort` leaves every graph so over 7,167 inputs).
    fn sort_mergeset(&mut self) {
        self.facet_mergeset.sort_by(|a, b| {
            b.mergetype.cmp(&a.mergetype).then_with(|| {
                // `a->distance < b->distance ? 1 : -1`: the larger first.
                b.distance
                    .partial_cmp(&a.distance)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
        });
    }

    /// `qh_test_appendmerge` (`merge_r.c:4839-4861`) without an angle
    /// (`qh.cos_max` is `REALmax`).
    fn test_appendmerge(
        &mut self,
        facet: FacetId,
        neighbor: FacetId,
        _simplicial: bool,
    ) -> Result<bool, Error> {
        self.test_centrum_merge(facet, neighbor)
    }

    /// `qh_test_centrum_merge` (`merge_r.c:4895-4943`): a merge appended
    /// where either facet's centrum is not clearly below the other.
    fn test_centrum_merge(&mut self, facet: FacetId, neighbor: FacetId) -> Result<bool, Error> {
        if self.f(facet).center.is_none() {
            let c = self.getcentrum(facet);
            self.fm(facet).center = Some(c);
        }
        let center = self.f(facet).center.unwrap_or_default();
        let dist = self.distplane(&center, neighbor);
        let mut isconcave = false;
        let mut iscoplanar = false;
        if dist > self.centrum_radius {
            isconcave = true;
        } else if dist >= -self.centrum_radius {
            iscoplanar = true;
        }
        if self.f(neighbor).center.is_none() {
            let c = self.getcentrum(neighbor);
            self.fm(neighbor).center = Some(c);
        }
        let ncenter = self.f(neighbor).center.unwrap_or_default();
        let dist2 = self.distplane(&ncenter, facet);
        if dist2 > self.centrum_radius {
            isconcave = true;
        } else if !iscoplanar && dist2 >= -self.centrum_radius {
            iscoplanar = true;
        }
        if !isconcave && !iscoplanar {
            return Ok(false);
        }
        if isconcave && iscoplanar {
            if dist > dist2 {
                self.appendmergeset(facet, neighbor, MergeType::ConcaveCoplanar, dist)?;
            } else {
                self.appendmergeset(neighbor, facet, MergeType::ConcaveCoplanar, dist2)?;
            }
        } else if isconcave {
            let mergedist = fmax_(dist, dist2);
            self.appendmergeset(facet, neighbor, MergeType::Concave, mergedist)?;
        } else {
            let mergedist = fmin_(dist.abs(), dist2.abs());
            self.appendmergeset(facet, neighbor, MergeType::Coplanar, mergedist)?;
        }
        Ok(true)
    }

    /// `qh_appendmergeset` (`merge_r.c:414-493`) for the merge types the 3-d
    /// path makes.
    fn appendmergeset(
        &mut self,
        facet: FacetId,
        neighbor: FacetId,
        mergetype: MergeType,
        dist: f64,
    ) -> Result<(), Error> {
        if self.f(facet).redundant || self.f(neighbor).redundant {
            return Ok(());
        }
        if self.f(facet).degenerate && mergetype == MergeType::Degen {
            return Ok(());
        }
        if self.f(neighbor).flipped && !self.f(facet).flipped {
            return Err(Error::qhull(
                6355,
                "cannot merge a non-flipped facet into a flipped one",
            ));
        }
        let merge = Merge {
            facet1: facet,
            facet2: neighbor,
            mergetype,
            distance: dist,
        };
        match mergetype {
            MergeType::Degen => {
                self.fm(facet).degenerate = true;
                if self
                    .degen_mergeset
                    .last()
                    .is_none_or(|m| m.mergetype == MergeType::Degen)
                {
                    self.degen_mergeset.push(merge);
                } else {
                    // a degenerate merge before the redundant ones (`:462-467`)
                    self.degen_mergeset.insert(0, merge);
                }
            }
            MergeType::Redundant => {
                self.fm(facet).redundant = true;
                self.degen_mergeset.push(merge);
            }
            // `mergetype < MRGdegen`
            _ => self.facet_mergeset.push(merge),
        }
        Ok(())
    }

    /// `qh_all_merges` without vertex neighbours' test (`merge_r.c:223-333`):
    /// the merges taken from the end of the sorted set, then the set refilled
    /// from the new facets' ridges, until none is left; then the shared
    /// vertices reduced.
    fn all_merges(&mut self, mut othermerge: bool) -> Result<(), Error> {
        let mut wasmerge = false;
        while !self.facet_mergeset.is_empty() || !self.degen_mergeset.is_empty() {
            if !self.degen_mergeset.is_empty() {
                self.merge_degenredundant()?;
                wasmerge = true;
            }
            while let Some(merge) = self.facet_mergeset.pop() {
                let (facet1, facet2, mergetype) = (merge.facet1, merge.facet2, merge.mergetype);
                if self.f(facet1).visible || self.f(facet2).visible {
                    continue;
                }
                if mergetype == MergeType::Coplanar
                    && ((!self.f(facet1).tested && self.f(facet1).newfacet)
                        || (!self.f(facet2).tested && self.f(facet2).newfacet))
                {
                    continue;
                }
                self.merge_nonconvex(facet1, facet2, mergetype)?;
                self.merge_degenredundant()?;
                wasmerge = true;
            }
            self.getmergeset(self.newfacet_list)?;
        }
        if self.VERTEXneighbors && (wasmerge || othermerge) {
            othermerge = false;
            let _ = othermerge;
            if self.reducevertices()? {
                return Err(Error::Unported {
                    routine: "qh_all_merges (a merge after reducing vertices)",
                });
            }
        }
        Ok(())
    }

    /// `qh_merge_nonconvex` without `AVOIDold` (`merge_r.c:2585-2648`): the
    /// new facet, or the other, merged into its best neighbour, whichever is
    /// nearer.
    fn merge_nonconvex(
        &mut self,
        facet1: FacetId,
        facet2: FacetId,
        mergetype: MergeType,
    ) -> Result<(), Error> {
        let (bestfacet, facet2) = if self.f(facet1).newfacet {
            (facet1, facet2)
        } else {
            (facet2, facet1)
        };
        let (bestneighbor, dist, mindist, maxdist) = self.findbestneighbor(bestfacet)?;
        let (neighbor, dist2, mindist2, maxdist2) = self.findbestneighbor(facet2)?;
        let (merging, merged, d) = if dist < dist2 {
            (bestfacet, bestneighbor, MergeDist { mindist, maxdist })
        } else {
            (
                facet2,
                neighbor,
                MergeDist {
                    mindist: mindist2,
                    maxdist: maxdist2,
                },
            )
        };
        self.mergefacet(merging, merged, mergetype, Some(d), false)
    }

    /// `qh_findbestneighbor` (`merge_r.c:1391-1432`): the neighbour a facet
    /// merges into with the least change, and the distances.
    fn findbestneighbor(&mut self, facet: FacetId) -> Result<(FacetId, f64, f64, f64), Error> {
        let size = self.f(facet).vertices.len();
        let mut dist = REAL_MAX;
        let mut mindist = 0.0;
        let mut maxdist = 0.0;
        let mut bestfacet = None;
        let testcentrum = size > BEST_CENTRUM2 * DIM + BEST_CENTRUM;
        if testcentrum && self.f(facet).center.is_none() {
            let c = self.getcentrum(facet);
            self.fm(facet).center = Some(c);
        }
        if size > DIM + BEST_NONCONVEX {
            let ridges = self.f(facet).ridges.clone().unwrap_or_default();
            let mut j = 0;
            while let Some(ridge) = set::get(&ridges, j) {
                j += 1;
                if self.r(ridge).nonconvex
                    && let Some(neighbor) = self.otherfacet(ridge, facet)
                {
                    self.findbest_test(
                        testcentrum,
                        facet,
                        neighbor,
                        &mut bestfacet,
                        &mut dist,
                        &mut mindist,
                        &mut maxdist,
                    );
                }
            }
        }
        if bestfacet.is_none() {
            let mut i = 0;
            while let Some(neighbor) = set::get(&self.f(facet).neighbors, i) {
                i += 1;
                self.findbest_test(
                    testcentrum,
                    facet,
                    neighbor,
                    &mut bestfacet,
                    &mut dist,
                    &mut mindist,
                    &mut maxdist,
                );
            }
        }
        let Some(best) = bestfacet else {
            return Err(Error::qhull(6095, "no neighbors for facet"));
        };
        if testcentrum {
            let (_, mn, mx) = self.getdistance(facet, best);
            mindist = mn;
            maxdist = mx;
        }
        Ok((best, dist, mindist, maxdist))
    }

    /// `qh_findbest_test` (`merge_r.c:1336-1362`).
    #[allow(clippy::too_many_arguments)] // qhull's out-parameters, kept as its callers pass them
    fn findbest_test(
        &mut self,
        testcentrum: bool,
        facet: FacetId,
        neighbor: FacetId,
        bestfacet: &mut Option<FacetId>,
        distp: &mut f64,
        mindistp: &mut f64,
        maxdistp: &mut f64,
    ) {
        if self.f(facet).flipped
            && self.f(neighbor).flipped
            && bestfacet.is_some_and(|b| !self.f(b).flipped)
        {
            return;
        }
        let (dist, mindist, maxdist) = if testcentrum {
            let center = self.f(facet).center.unwrap_or_default();
            let mut dist = self.distplane(&center, neighbor);
            dist *= DIM as f64;
            if dist < 0.0 {
                (-dist, dist, 0.0)
            } else {
                (dist, 0.0, dist)
            }
        } else {
            self.getdistance(facet, neighbor)
        };
        if dist < *distp {
            *bestfacet = Some(neighbor);
            *mindistp = mindist;
            *maxdistp = maxdist;
            *distp = dist;
        }
    }

    /// `qh_mergefacet` (`merge_r.c:3362-3519`): `facet1` merged into
    /// `facet2`, which moves to the end of the facet list as a new facet;
    /// `facet1` becomes visible.
    pub(crate) fn mergefacet(
        &mut self,
        facet1: FacetId,
        facet2: FacetId,
        mergetype: MergeType,
        dists: Option<MergeDist>,
        mergeapex: bool,
    ) -> Result<(), Error> {
        if self.f(facet1).tricoplanar || self.f(facet2).tricoplanar {
            return Err(Error::qhull(
                6226,
                "merge does not work for tricoplanar facets",
            ));
        }
        self.Ztotmerge += 1;
        if let Some(d) = dists {
            let mut mintwisted = WIDE_MAXOUTSIDE * self.ONEmerge;
            maximize_(&mut mintwisted, self.f(facet1).maxoutside);
            maximize_(&mut mintwisted, self.f(facet2).maxoutside);
            if d.maxdist > mintwisted || -d.mindist > mintwisted {
                return Err(Error::qhull(6347, "wide merge"));
            }
        }
        if facet1 == facet2 || self.f(facet1).visible || self.f(facet2).visible {
            return Err(Error::qhull(
                6099,
                "either the facets are the same or one is visible",
            ));
        }
        if self.num_facets - self.num_visible <= DIM as i32 + 1 {
            return Err(Error::qhull(
                6227,
                "Only 4 facets remain.  The input is too degenerate or the convexity constraints are too strong.",
            ));
        }
        if !self.VERTEXneighbors {
            self.vertexneighbors();
        }
        self.makeridges(facet1);
        self.makeridges(facet2);
        if let Some(d) = dists {
            maximize_(&mut self.max_outside, d.maxdist);
            maximize_(&mut self.max_vertex, d.maxdist);
            maximize_(&mut self.fm(facet2).maxoutside, d.maxdist);
            minimize_(&mut self.min_vertex, d.mindist);
            if !self.f(facet2).keepcentrum
                && (d.maxdist > self.WIDEfacet || d.mindist < -self.WIDEfacet)
            {
                self.fm(facet2).keepcentrum = true;
            }
        }
        let nummerge = self.f(facet1).nummerge + self.f(facet2).nummerge + 1;
        self.fm(facet2).nummerge = nummerge.min(MAX_NUMMERGE);
        self.fm(facet2).newmerge = true;
        self.fm(facet2).dupridge = false;
        self.updatetested(facet1, facet2);
        if self.f(facet1).vertices.len() == DIM {
            self.mergesimplex(facet1, facet2, mergeapex)?;
        } else {
            self.vertex_visit += 1;
            let vv = self.vertex_visit;
            for v in self.f(facet2).vertices.clone() {
                self.vm(v).visitid = vv;
            }
            self.mergeneighbors(facet1, facet2);
            self.mergevertices(facet1, facet2);
            self.mergeridges(facet1, facet2);
            self.mergevertex_neighbors(facet1, facet2);
            if !self.f(facet2).newfacet {
                self.newvertices(facet2);
            }
        }
        self.removefacet(facet2);
        self.appendfacet(facet2);
        self.fm(facet2).newfacet = true;
        self.fm(facet2).tested = false;
        if mergetype != MergeType::CoplanarHorizon {
            self.test_redundant_neighbors(facet2)?;
            self.test_degen_neighbors(facet1)?;
            self.degen_redundant_facet(facet2)?;
            // `qh_maybe_duplicateridges`: `qh.CHECKduplicates` is off.
        }
        self.willdelete(facet1, Some(facet2));
        Ok(())
    }

    /// `qh_makeridges` (`merge_r.c:2074-2139`): a simplicial facet given a
    /// ridge for each neighbour it has none with.
    pub(crate) fn makeridges(&mut self, facet: FacetId) {
        if !self.f(facet).simplicial {
            return;
        }
        self.fm(facet).simplicial = false;
        let mut i = 0;
        while let Some(neighbor) = set::get(&self.f(facet).neighbors, i) {
            i += 1;
            self.fm(neighbor).seen = false;
        }
        for ridge in self.f(facet).ridges.clone().unwrap_or_default() {
            if ridge == <RidgeId as set::SetElem>::NULL {
                break;
            }
            if let Some(o) = self.otherfacet(ridge, facet) {
                self.fm(o).seen = true;
            }
        }
        let neighbors = self.f(facet).neighbors.clone();
        for (neighbor_i, &neighbor) in neighbors.iter().enumerate() {
            if !self.f(neighbor).seen {
                let ridge = self.newridge();
                let vertices = set::setnew_delnthsorted(&self.f(facet).vertices, neighbor_i, 0);
                let toporient = self.f(facet).toporient ^ (neighbor_i & 0x1 == 1);
                let nsimplicial = self.f(neighbor).simplicial;
                let tested = self.f(facet).tested;
                let r = self.rm(ridge);
                r.vertices = vertices;
                if toporient {
                    r.top = Some(facet);
                    r.bottom = Some(neighbor);
                    r.simplicialtop = true;
                    r.simplicialbot = nsimplicial;
                } else {
                    r.top = Some(neighbor);
                    r.bottom = Some(facet);
                    r.simplicialtop = nsimplicial;
                    r.simplicialbot = true;
                }
                if tested {
                    r.tested = true;
                }
                self.fm(facet)
                    .ridges
                    .get_or_insert_with(Vec::new)
                    .push(ridge);
                self.fm(neighbor)
                    .ridges
                    .get_or_insert_with(Vec::new)
                    .push(ridge);
            }
        }
    }

    /// `otherfacet_` (`libqhull_r.h:871`): the ridge's facet that is not
    /// `facet`.
    pub(crate) fn otherfacet(&self, ridge: RidgeId, facet: FacetId) -> Option<FacetId> {
        let r = self.r(ridge);
        if r.top == Some(facet) {
            r.bottom
        } else {
            r.top
        }
    }

    /// `qh_updatetested` (`merge_r.c:5396-5422`).
    fn updatetested(&mut self, facet1: FacetId, facet2: FacetId) {
        self.fm(facet2).tested = false;
        for ridge in self.f(facet1).ridges.clone().unwrap_or_default() {
            if ridge == <RidgeId as set::SetElem>::NULL {
                break;
            }
            self.rm(ridge).tested = false;
        }
        if self.f(facet2).center.is_none() {
            return;
        }
        let size = self.f(facet2).vertices.len();
        if !self.f(facet2).keepcentrum {
            if size > DIM + MAX_NEWCENTRUM {
                self.fm(facet2).keepcentrum = true;
            }
        } else if size <= DIM + MAX_NEWCENTRUM && size == DIM {
            self.fm(facet2).keepcentrum = false;
        }
        if !self.f(facet2).keepcentrum {
            self.fm(facet2).center = None;
            for ridge in self.f(facet2).ridges.clone().unwrap_or_default() {
                if ridge == <RidgeId as set::SetElem>::NULL {
                    break;
                }
                self.rm(ridge).tested = false;
            }
        }
    }

    // qhull's routine kept whole, so it reads against its source.
    #[allow(clippy::too_many_lines)]
    /// `qh_mergesimplex` (`merge_r.c:3738-3840`): a simplicial `facet1`
    /// merged into `facet2`, its vertex opposite `facet2` added to it.
    fn mergesimplex(
        &mut self,
        facet1: FacetId,
        facet2: FacetId,
        mergeapex: bool,
    ) -> Result<(), Error> {
        let mut isnew = false;
        let opposite;
        if mergeapex {
            opposite = self.f(facet1).vertices[0];
            if !self.f(facet2).newfacet {
                self.newvertices(facet2);
            }
            if self.f(facet2).vertices.first() != Some(&opposite) {
                set::setaddnth(&mut self.fm(facet2).vertices, 0, opposite);
                isnew = true;
            }
        } else {
            for v in self.f(facet1).vertices.clone() {
                self.vm(v).seen = false;
            }
            let ridges = self.f(facet1).ridges.clone().unwrap_or_default();
            let mut j = 0;
            while let Some(ridge) = set::get(&ridges, j) {
                j += 1;
                if self.otherfacet(ridge, facet1) == Some(facet2) {
                    for v in self.r(ridge).vertices.clone() {
                        self.vm(v).seen = true;
                        self.vm(v).delridge = true;
                    }
                    break;
                }
            }
            let Some(&first_unseen) = self.f(facet1).vertices.iter().find(|&&v| !self.v(v).seen)
            else {
                return Err(Error::qhull(
                    0,
                    "qh_mergesimplex: no vertex of facet1 opposite facet2",
                ));
            };
            opposite = first_unseen;
            isnew = self.addfacetvertex(facet2, opposite);
            if !self.f(facet2).newfacet {
                self.newvertices(facet2);
            } else if !self.v(opposite).newfacet {
                self.removevertex(opposite);
                self.appendvertex(opposite);
            }
        }
        for vertex in self.f(facet1).vertices.clone() {
            if vertex == opposite && isnew {
                set::setreplace(&mut self.vm(vertex).neighbors, facet1, facet2);
            } else {
                set::setdel(&mut self.vm(vertex).neighbors, facet1);
                if set::get(&self.v(vertex).neighbors, 1).is_none() {
                    self.mergevertex_del(vertex, facet2);
                }
            }
        }
        self.visit_id += 1;
        let visit = self.visit_id;
        let mut i = 0;
        while let Some(neighbor) = set::get(&self.f(facet2).neighbors, i) {
            i += 1;
            self.fm(neighbor).visitid = visit;
        }
        // FOREACHridge_(facet1->ridges), the set live: a deleted ridge's slot
        // takes the last ridge, read next (`ridgep--`).
        let mut j = 0;
        while let Some(ridge) = self.f(facet1).ridges.as_ref().and_then(|r| set::get(r, j)) {
            let Some(otherfacet) = self.otherfacet(ridge, facet1) else {
                break;
            };
            if otherfacet == facet2 {
                self.delridge_merge(ridge);
                set::setdel(&mut self.fm(facet2).neighbors, facet1);
                continue;
            }
            j += 1;
            if self.f(otherfacet).dupridge && !set::setin(&self.f(otherfacet).neighbors, facet1) {
                return Err(Error::qhull(
                    6356,
                    "a dupridge facet is not a neighbor of facet1",
                ));
            }
            self.fm(facet2)
                .ridges
                .get_or_insert_with(Vec::new)
                .push(ridge);
            if self.f(otherfacet).visitid == visit {
                if self.f(otherfacet).simplicial {
                    self.makeridges(otherfacet);
                }
                if self.f(otherfacet).neighbors.first() == Some(&facet1) {
                    set::setdel(&mut self.fm(otherfacet).neighbors, facet2);
                    set::setreplace(&mut self.fm(otherfacet).neighbors, facet1, facet2);
                } else {
                    set::setdel(&mut self.fm(otherfacet).neighbors, facet1);
                }
            } else {
                self.fm(facet2).neighbors.push(otherfacet);
                set::setreplace(&mut self.fm(otherfacet).neighbors, facet1, facet2);
                self.fm(otherfacet).visitid = visit;
            }
            let r = self.rm(ridge);
            if r.top == Some(facet1) {
                r.top = Some(facet2);
                r.simplicialtop = false;
            } else {
                r.bottom = Some(facet2);
                r.simplicialbot = false;
            }
        }
        Ok(())
    }

    /// `qh_mergevertex_del` (`merge_r.c:3852-3860`): a vertex left on one facet
    /// deleted from `facet2`.
    fn mergevertex_del(&mut self, vertex: VertexId, facet2: FacetId) {
        set::setdelsorted(&mut self.fm(facet2).vertices, vertex);
        self.vm(vertex).deleted = true;
        self.del_vertices.push(vertex);
    }

    /// `qh_newvertices` (`merge_r.c:4133-4142`): a facet's vertices moved to the
    /// new vertex list.
    fn newvertices(&mut self, facet: FacetId) {
        for v in self.f(facet).vertices.clone() {
            if !self.v(v).newfacet {
                self.removevertex(v);
                self.appendvertex(v);
            }
        }
    }

    /// `qh_delridge_merge` (`merge_r.c:1028-1054`) with no vertex merge
    /// queued.
    fn delridge_merge(&mut self, ridge: RidgeId) {
        if self.r(ridge).nonconvex {
            self.copynonconvex(ridge);
        }
        for v in self.r(ridge).vertices.clone() {
            self.vm(v).delridge = true;
        }
        if let Some(top) = self.r(ridge).top
            && let Some(r) = self.fm(top).ridges.as_mut()
        {
            set::setdel(r, ridge);
        }
        if let Some(bottom) = self.r(ridge).bottom
            && let Some(r) = self.fm(bottom).ridges.as_mut()
        {
            set::setdel(r, ridge);
        }
        self.delridge(ridge);
    }

    /// `qh_copynonconvex` (`merge_r.c:937-954`): another ridge between the
    /// same facets marked non-convex.
    fn copynonconvex(&mut self, atridge: RidgeId) {
        let (Some(facet), Some(otherfacet)) = (self.r(atridge).top, self.r(atridge).bottom) else {
            return;
        };
        self.rm(atridge).nonconvex = false;
        let ridges = self.f(facet).ridges.clone().unwrap_or_default();
        let mut j = 0;
        while let Some(ridge) = set::get(&ridges, j) {
            j += 1;
            if (Some(otherfacet) == self.r(ridge).top || Some(otherfacet) == self.r(ridge).bottom)
                && ridge != atridge
            {
                self.rm(ridge).nonconvex = true;
                break;
            }
        }
    }

    /// `qh_willdelete` (`merge_r.c:5524-5541`): the facet onto the visible
    /// list, replaced by `replace`.
    pub(crate) fn willdelete(&mut self, facet: FacetId, replace: Option<FacetId>) {
        self.removefacet(facet);
        self.prependfacet(facet, ListHead::Visible);
        self.num_visible += 1;
        let f = self.fm(facet);
        f.visible = true;
        f.f = replace;
        if let Some(r) = f.ridges.as_mut()
            && let Some(first) = r.first_mut()
        {
            *first = <RidgeId as set::SetElem>::NULL;
        }
        if let Some(first) = f.neighbors.first_mut() {
            *first = FacetId::NULL;
        }
    }

    /// `qh_merge_degenredundant` (`merge_r.c:2490-2563`): each queued
    /// redundant facet merged into its replacement, each degenerate one into
    /// its best neighbour. The count of merges.
    pub(crate) fn merge_degenredundant(&mut self) -> Result<i32, Error> {
        let mut nummerges = 0;
        let mut mergedfacets: Vec<FacetId> = Vec::new();
        while let Some(merge) = self.degen_mergeset.pop() {
            let (facet1, facet2, mergetype) = (merge.facet1, merge.facet2, merge.mergetype);
            if self.f(facet1).visible {
                continue;
            }
            self.fm(facet1).degenerate = false;
            self.fm(facet1).redundant = false;
            if mergetype == MergeType::Redundant {
                let Some(facet3) = self.getreplacement(facet2) else {
                    return Err(Error::qhull(
                        6097,
                        "redundant facet's replacement was deleted",
                    ));
                };
                if !mergedfacets.contains(&facet3) {
                    mergedfacets.push(facet3);
                }
                if facet1 == facet3 {
                    continue;
                }
                self.mergefacet(facet1, facet3, mergetype, None, false)?;
                nummerges += 1;
            } else {
                let size = set::count(&self.f(facet1).neighbors);
                if size == 0 {
                    self.willdelete(facet1, None);
                    for vertex in self.f(facet1).vertices.clone() {
                        set::setdel(&mut self.vm(vertex).neighbors, facet1);
                        if set::get(&self.v(vertex).neighbors, 0).is_none() {
                            self.vm(vertex).deleted = true;
                            self.del_vertices.push(vertex);
                        }
                    }
                    nummerges += 1;
                } else if size < DIM {
                    let (bestneighbor, _dist, mindist, maxdist) = self.findbestneighbor(facet1)?;
                    self.mergefacet(
                        facet1,
                        bestneighbor,
                        mergetype,
                        Some(MergeDist { mindist, maxdist }),
                        false,
                    )?;
                    nummerges += 1;
                }
            }
        }
        Ok(nummerges)
    }

    /// `qh_degen_redundant_facet` (`merge_r.c:973-1008`): a redundant or
    /// degenerate merge queued for the facet.
    fn degen_redundant_facet(&mut self, facet: FacetId) -> Result<(), Error> {
        if self.f(facet).flipped {
            return Ok(());
        }
        let mut i = 0;
        while let Some(neighbor) = set::get(&self.f(facet).neighbors, i) {
            i += 1;
            if self.f(neighbor).flipped {
                continue;
            }
            self.vertex_visit += 1;
            let vv = self.vertex_visit;
            for v in self.f(neighbor).vertices.clone() {
                self.vm(v).visitid = vv;
            }
            if self
                .f(facet)
                .vertices
                .iter()
                .all(|&v| self.v(v).visitid == vv)
            {
                self.appendmergeset(facet, neighbor, MergeType::Redundant, 0.0)?;
                return Ok(());
            }
        }
        if set::count(&self.f(facet).neighbors) < DIM {
            self.appendmergeset(facet, facet, MergeType::Degen, 0.0)?;
        }
        Ok(())
    }

    /// `qh_test_redundant_neighbors` (`merge_r.c:5196-5232`): each neighbour
    /// whose vertices are all the facet's queued as redundant.
    fn test_redundant_neighbors(&mut self, facet: FacetId) -> Result<(), Error> {
        if set::count(&self.f(facet).neighbors) < DIM {
            self.appendmergeset(facet, facet, MergeType::Degen, 0.0)?;
            return Ok(());
        }
        self.vertex_visit += 1;
        let vv = self.vertex_visit;
        for v in self.f(facet).vertices.clone() {
            self.vm(v).visitid = vv;
        }
        let mut i = 0;
        while let Some(neighbor) = set::get(&self.f(facet).neighbors, i) {
            i += 1;
            let n = self.f(neighbor);
            if n.degenerate || n.redundant || n.dupridge {
                continue;
            }
            if self.f(facet).flipped && !n.flipped {
                continue;
            }
            if n.vertices.iter().all(|&v| self.v(v).visitid == vv) {
                self.appendmergeset(neighbor, facet, MergeType::Redundant, 0.0)?;
            }
        }
        Ok(())
    }

    /// `qh_test_degen_neighbors` (`merge_r.c:4958-4977`).
    fn test_degen_neighbors(&mut self, facet: FacetId) -> Result<(), Error> {
        let mut i = 0;
        while let Some(neighbor) = set::get(&self.f(facet).neighbors, i) {
            i += 1;
            let n = self.f(neighbor);
            if n.degenerate || n.redundant || n.dupridge {
                continue;
            }
            if set::count(&n.neighbors) < DIM {
                self.appendmergeset(neighbor, neighbor, MergeType::Degen, 0.0)?;
            }
        }
        Ok(())
    }

    /// `qh_reducevertices` in 3-d (`merge_r.c:4279-4338`): each newly merged
    /// facet's extra vertices removed and its shared vertices renamed.
    /// Whether a degenerate or redundant merge followed.
    fn reducevertices(&mut self) -> Result<bool, Error> {
        if self.merge_degenredundant()? > 0 {
            return Err(Error::Unported {
                routine: "qh_reducevertices (a degenerate merge first)",
            });
        }
        for newfacet in self.facets_from(self.newfacet_list) {
            if self.f(newfacet).newmerge && self.remove_extravertices(newfacet) {
                return Err(Error::Unported {
                    routine: "qh_reducevertices (an extra vertex removed)",
                });
            }
        }
        for newfacet in self.facets_from(self.newfacet_list) {
            if !self.f(newfacet).newmerge {
                continue;
            }
            self.fm(newfacet).newmerge = false;
            // FOREACHvertex_ over the facet's vertices, live: a rename deletes
            // the vertex from the set and the slot is read again.
            let mut i = 0;
            while let Some(vertex) = set::get(&self.f(newfacet).vertices, i) {
                i += 1;
                if self.v(vertex).delridge && self.rename_sharedvertex(vertex, newfacet)?.is_some()
                {
                    if self.merge_degenredundant()? > 0 {
                        return Err(Error::Unported {
                            routine: "qh_reducevertices (a degenerate merge after a rename)",
                        });
                    }
                    i -= 1;
                }
            }
        }
        for vertex in self.vertices_from(self.newvertex_list) {
            if self.v(vertex).delridge && !self.v(vertex).deleted {
                self.vm(vertex).delridge = false;
            }
        }
        Ok(false)
    }

    /// `qh_remove_extravertices` (`merge_r.c:4403-4436`): whether the facet
    /// held a vertex on none of its ridges.
    fn remove_extravertices(&mut self, facet: FacetId) -> bool {
        if self.f(facet).simplicial {
            return false;
        }
        for v in self.f(facet).vertices.clone() {
            self.vm(v).seen = false;
        }
        for ridge in self.f(facet).ridges.clone().unwrap_or_default() {
            if ridge == <RidgeId as set::SetElem>::NULL {
                break;
            }
            for v in self.r(ridge).vertices.clone() {
                self.vm(v).seen = true;
            }
        }
        self.f(facet).vertices.iter().any(|&v| !self.v(v).seen)
    }

    /// `qh_rename_sharedvertex` in 3-d (`merge_r.c:4588-4634`): a vertex on
    /// only two facets renamed to another vertex they share, where one makes
    /// no duplicate ridge.
    fn rename_sharedvertex(
        &mut self,
        vertex: VertexId,
        facet: FacetId,
    ) -> Result<Option<VertexId>, Error> {
        if set::count(&self.v(vertex).neighbors) != 2 {
            return Ok(None);
        }
        let n = &self.v(vertex).neighbors;
        let neighbor_a = if n[0] == facet { n[1] } else { n[0] };
        self.visit_id += 1;
        self.fm(neighbor_a).visitid = self.visit_id;
        let mut ridges = Vec::new();
        self.vertexridges_facet(vertex, facet, &mut ridges);
        let mut vertices =
            vertexintersect_new(self, &self.f(facet).vertices, &self.f(neighbor_a).vertices);
        set::setdel(&mut vertices, vertex);
        let newvertex = self.find_newvertex(vertex, &mut vertices, &ridges);
        if let Some(nv) = newvertex {
            self.renamevertex(vertex, nv, &ridges, facet, neighbor_a)?;
        }
        Ok(newvertex)
    }

    /// `qh_vertexridges` (`merge_r.c:5443-5464`): the ridges of the vertex's
    /// neighbours that hold it, each neighbour's but the last unless `all`.
    fn vertexridges(&mut self, vertex: VertexId, allneighbors: bool) -> Vec<RidgeId> {
        let mut ridges = Vec::new();
        self.visit_id += 2;
        let visit = self.visit_id;
        let neighbors: Vec<FacetId> = (0..)
            .map_while(|i| set::get(&self.v(vertex).neighbors, i))
            .collect();
        for &n in &neighbors {
            self.fm(n).visitid = visit;
        }
        for (k, &n) in neighbors.iter().enumerate() {
            if k + 1 < neighbors.len() || allneighbors {
                self.vertexridges_facet(vertex, n, &mut ridges);
            }
        }
        ridges
    }

    /// `qh_vertexridges_facet` in 3-d (`merge_r.c:5484-5511`): the facet's
    /// ridges to visited neighbours that hold the vertex.
    fn vertexridges_facet(&mut self, vertex: VertexId, facet: FacetId, ridges: &mut Vec<RidgeId>) {
        let visit = self.visit_id;
        let last_i = DIM - 2;
        let fr = self.f(facet).ridges.clone().unwrap_or_default();
        let mut j = 0;
        while let Some(ridge) = set::get(&fr, j) {
            j += 1;
            let Some(neighbor) = self.otherfacet(ridge, facet) else {
                continue;
            };
            if self.f(neighbor).visitid == visit {
                let rv = &self.r(ridge).vertices;
                if rv.first() == Some(&vertex) || rv.get(last_i) == Some(&vertex) {
                    ridges.push(ridge);
                }
            }
        }
        self.fm(facet).visitid = visit - 1;
    }

    /// `qh_find_newvertex` (`merge_r.c:1109-1190`): of `vertices`, the one on
    /// the most of `ridges` that renaming `oldvertex` to makes no duplicate
    /// ridge.
    fn find_newvertex(
        &mut self,
        oldvertex: VertexId,
        vertices: &mut Vec<VertexId>,
        ridges: &[RidgeId],
    ) -> Option<VertexId> {
        for &ridge in ridges {
            for v in self.r(ridge).vertices.clone() {
                self.vm(v).seen = false;
            }
        }
        for &v in vertices.iter() {
            self.vm(v).visitid = 0;
            self.vm(v).seen = true;
        }
        for &ridge in ridges {
            for v in self.r(ridge).vertices.clone() {
                if self.v(v).seen {
                    self.vm(v).visitid += 1;
                }
            }
        }
        // FOREACHvertex_ with a delete and a repeat of the slot.
        let mut i = 0;
        while let Some(v) = set::get(vertices, i) {
            if self.v(v).visitid == 0 {
                set::setdelnth(vertices, i);
            } else {
                i += 1;
            }
        }
        let maxvisit = ridges.len() as u32;
        if self.vertex_visit < maxvisit {
            self.vertex_visit = maxvisit;
        }
        if vertices.is_empty() {
            return None;
        }
        // `qsort` by `qh_comparevisit` (`merge_r.c:910-918`), made stable.
        vertices.sort_by_key(|&v| self.v(v).visitid);
        let hashsize = newhashtable(ridges.len());
        let mut hash_table: Vec<RidgeId> = vec![<RidgeId as set::SetElem>::NULL; hashsize];
        for &ridge in ridges {
            self.hashridge(&mut hash_table, ridge, oldvertex);
        }
        for &vertex in vertices.iter() {
            let newridges = self.vertexridges(vertex, false);
            let clash = newridges
                .iter()
                .any(|&r| self.hashridge_find(&hash_table, r, vertex, oldvertex));
            if !clash {
                return Some(vertex);
            }
        }
        None
    }

    /// `qh_hashridge` (`merge_r.c:1956-1970`).
    fn hashridge(&self, hash_table: &mut [RidgeId], ridge: RidgeId, oldvertex: VertexId) {
        let hashsize = hash_table.len();
        let mut hash = gethash_vertices(hashsize, &self.r(ridge).vertices, oldvertex);
        loop {
            let ridge_a = hash_table[hash];
            if ridge_a == <RidgeId as set::SetElem>::NULL {
                hash_table[hash] = ridge;
                break;
            } else if ridge_a == ridge {
                break;
            }
            hash += 1;
            if hash == hashsize {
                hash = 0;
            }
        }
    }

    /// `qh_hashridge_find` (`merge_r.c:1999-2021`): whether a hashed ridge
    /// has `ridge`'s vertices with `oldvertex` for `vertex`.
    fn hashridge_find(
        &self,
        hash_table: &[RidgeId],
        ridge: RidgeId,
        vertex: VertexId,
        oldvertex: VertexId,
    ) -> bool {
        let hashsize = hash_table.len();
        let mut hash = gethash_vertices(hashsize, &self.r(ridge).vertices, vertex);
        loop {
            let ridge_a = hash_table[hash];
            if ridge_a == <RidgeId as set::SetElem>::NULL {
                return false;
            }
            if ridge_a != ridge
                && setequal_except(
                    &self.r(ridge).vertices,
                    vertex,
                    &self.r(ridge_a).vertices,
                    oldvertex,
                )
            {
                return true;
            }
            hash += 1;
            if hash == hashsize {
                hash = 0;
            }
        }
    }

    /// `qh_renamevertex` for a vertex shared by two facets
    /// (`merge_r.c:4735-4810`).
    fn renamevertex(
        &mut self,
        oldvertex: VertexId,
        newvertex: VertexId,
        ridges: &[RidgeId],
        oldfacet: FacetId,
        neighbor_a: FacetId,
    ) -> Result<(), Error> {
        for &ridge in ridges {
            if self.renameridgevertex(ridge, oldvertex, newvertex)? {
                let (Some(top), Some(bottom)) = (self.r(ridge).top, self.r(ridge).bottom) else {
                    continue;
                };
                let topsize = self.f(top).vertices.len();
                let bottomsize = self.f(bottom).vertices.len();
                let top_degen = topsize < DIM
                    || (topsize == DIM
                        && !self.f(top).simplicial
                        && set::setin(&self.f(top).vertices, newvertex));
                let bottom_degen = bottomsize < DIM
                    || (bottomsize == DIM
                        && !self.f(bottom).simplicial
                        && set::setin(&self.f(bottom).vertices, newvertex));
                if !top_degen && !bottom_degen {
                    self.maybe_duplicateridge(ridge)?;
                }
            }
        }
        if set::count(&self.v(oldvertex).neighbors) == 2 {
            let neighbors: Vec<FacetId> = (0..)
                .map_while(|i| set::get(&self.v(oldvertex).neighbors, i))
                .collect();
            for neighbor in neighbors {
                set::setdelsorted(&mut self.fm(neighbor).vertices, oldvertex);
                self.degen_redundant_facet(neighbor)?;
            }
            self.vm(oldvertex).deleted = true;
            self.del_vertices.push(oldvertex);
        } else {
            set::setdelsorted(&mut self.fm(oldfacet).vertices, oldvertex);
            set::setdel(&mut self.vm(oldvertex).neighbors, oldfacet);
            if self.remove_extravertices(neighbor_a) {
                return Err(Error::Unported {
                    routine: "qh_renamevertex (an extra vertex removed)",
                });
            }
        }
        self.degen_redundant_facet(oldfacet)?;
        Ok(())
    }

    /// `qh_renameridgevertex` (`merge_r.c:4659-4696`): `oldvertex` replaced
    /// in the ridge, kept sorted; the ridge deleted (`false`) where it held
    /// `newvertex` already.
    fn renameridgevertex(
        &mut self,
        ridge: RidgeId,
        oldvertex: VertexId,
        newvertex: VertexId,
    ) -> Result<bool, Error> {
        let Some(oldnth) = self.r(ridge).vertices.iter().position(|&v| v == oldvertex) else {
            return Err(Error::qhull(6424, "oldvertex not in ridge"));
        };
        set::setdelnthsorted(&mut self.rm(ridge).vertices, oldnth);
        let mut nth = 0;
        let rv = self.r(ridge).vertices.clone();
        for &vertex in &rv {
            if vertex == newvertex {
                if self.r(ridge).nonconvex {
                    self.copynonconvex(ridge);
                }
                self.delridge_merge(ridge);
                return Ok(false);
            }
            if self.v(vertex).id < self.v(newvertex).id {
                break;
            }
            nth += 1;
        }
        set::setaddnth(&mut self.rm(ridge).vertices, nth, newvertex);
        let r = self.rm(ridge);
        r.simplicialtop = false;
        r.simplicialbot = false;
        if oldnth.abs_diff(nth) % 2 == 1 {
            std::mem::swap(&mut r.top, &mut r.bottom);
        }
        Ok(true)
    }

    /// `qh_maybe_duplicateridge` in 3-d (`merge_r.c:2285-2317`): another ridge
    /// of a merged neighbour with the same vertices would be a pinched vertex
    /// to merge, which is not ported.
    fn maybe_duplicateridge(&self, ridge_a: RidgeId) -> Result<(), Error> {
        let last = DIM - 2;
        for neighbor in [self.r(ridge_a).top, self.r(ridge_a).bottom]
            .into_iter()
            .flatten()
        {
            if !self.f(neighbor).simplicial && self.f(neighbor).nummerge > 0 {
                let ridges = self.f(neighbor).ridges.clone().unwrap_or_default();
                let mut j = 0;
                while let Some(ridge) = set::get(&ridges, j) {
                    j += 1;
                    let (rv, av) = (&self.r(ridge).vertices, &self.r(ridge_a).vertices);
                    if ridge != ridge_a && rv.first() == av.first() && rv.get(last) == av.get(last)
                    {
                        return Err(Error::Unported {
                            routine: "qh_maybe_duplicateridge (a pinched vertex)",
                        });
                    }
                }
            }
        }
        Ok(())
    }

    /// `qh_addfacetvertex` (`poly2_r.c:33-49`): the vertex into the facet's
    /// vertices, kept decreasing by id; whether it was not there.
    fn addfacetvertex(&mut self, facet: FacetId, newvertex: VertexId) -> bool {
        let nid = self.v(newvertex).id;
        let vertices = &self.f(facet).vertices;
        let mut vertex_i = vertices.len();
        for (i, &v) in vertices.iter().enumerate() {
            let id = self.v(v).id;
            if id < nid {
                vertex_i = i;
                break;
            } else if id == nid {
                return false;
            }
        }
        set::setaddnth(&mut self.fm(facet).vertices, vertex_i, newvertex);
        true
    }

    /// `qh_mergeneighbors` (`merge_r.c:3622-3648`).
    fn mergeneighbors(&mut self, facet1: FacetId, facet2: FacetId) {
        self.visit_id += 1;
        let visit = self.visit_id;
        let mut i = 0;
        while let Some(neighbor) = set::get(&self.f(facet2).neighbors, i) {
            i += 1;
            self.fm(neighbor).visitid = visit;
        }
        let mut i = 0;
        while let Some(neighbor) = set::get(&self.f(facet1).neighbors, i) {
            i += 1;
            if self.f(neighbor).visitid == visit {
                if self.f(neighbor).simplicial {
                    self.makeridges(neighbor);
                }
                if self.f(neighbor).neighbors.first() == Some(&facet1) {
                    set::setdel(&mut self.fm(neighbor).neighbors, facet2);
                    set::setreplace(&mut self.fm(neighbor).neighbors, facet1, facet2);
                } else {
                    set::setdel(&mut self.fm(neighbor).neighbors, facet1);
                }
            } else if neighbor != facet2 {
                self.fm(facet2).neighbors.push(neighbor);
                set::setreplace(&mut self.fm(neighbor).neighbors, facet1, facet2);
            }
        }
        set::setdel(&mut self.fm(facet1).neighbors, facet2);
        set::setdel(&mut self.fm(facet2).neighbors, facet1);
    }

    /// `qh_mergevertices` (`merge_r.c:3916-3943`): the two decreasing vertex
    /// sets merged into `facet2`'s.
    fn mergevertices(&mut self, facet1: FacetId, facet2: FacetId) {
        let v1 = self.f(facet1).vertices.clone();
        let v2 = self.f(facet2).vertices.clone();
        let id = |v: VertexId| self.v(v).id;
        let mut merged = Vec::with_capacity(v1.len() + v2.len());
        let mut k = 0;
        for &vertex in &v1 {
            if k >= v2.len() || id(vertex) > id(v2[k]) {
                merged.push(vertex);
            } else {
                while k < v2.len() && id(v2[k]) > id(vertex) {
                    merged.push(v2[k]);
                    k += 1;
                }
                if k >= v2.len() || id(v2[k]) < id(vertex) {
                    merged.push(vertex);
                } else {
                    merged.push(v2[k]);
                    k += 1;
                }
            }
        }
        merged.extend_from_slice(&v2[k..]);
        self.fm(facet2).vertices = merged;
    }

    /// `qh_mergeridges` (`merge_r.c:3670-3692`).
    fn mergeridges(&mut self, facet1: FacetId, facet2: FacetId) {
        // FOREACHridge_(facet2->ridges), live, repeating a deleted slot.
        let mut j = 0;
        while let Some(ridge) = self.f(facet2).ridges.as_ref().and_then(|r| set::get(r, j)) {
            if self.r(ridge).top == Some(facet1) || self.r(ridge).bottom == Some(facet1) {
                self.delridge_merge(ridge);
            } else {
                j += 1;
            }
        }
        for ridge in self.f(facet1).ridges.clone().unwrap_or_default() {
            if ridge == <RidgeId as set::SetElem>::NULL {
                break;
            }
            let r = self.rm(ridge);
            if r.top == Some(facet1) {
                r.top = Some(facet2);
                r.simplicialtop = false;
            } else {
                r.bottom = Some(facet2);
                r.simplicialbot = false;
            }
            self.fm(facet2)
                .ridges
                .get_or_insert_with(Vec::new)
                .push(ridge);
        }
    }

    /// `qh_mergevertex_neighbors` (`merge_r.c:3878-3899`).
    fn mergevertex_neighbors(&mut self, facet1: FacetId, facet2: FacetId) {
        let vv = self.vertex_visit;
        for vertex in self.f(facet1).vertices.clone() {
            if self.v(vertex).visitid == vv {
                set::setdel(&mut self.vm(vertex).neighbors, facet1);
                if set::get(&self.v(vertex).neighbors, 1).is_none() {
                    self.mergevertex_del(vertex, facet2);
                }
            } else {
                set::setreplace(&mut self.vm(vertex).neighbors, facet1, facet2);
            }
        }
    }
}

/// `fmin_` (`geom_r.h:43`).
fn fmin_(a: f64, b: f64) -> f64 {
    if a > b { b } else { a }
}

/// `qh_newhashtable`'s size (`poly2_r.c:2899-2916`).
const fn newhashtable(newsize: usize) -> usize {
    let mut size = ((newsize + 1) * 2) | 0x1;
    while size.is_multiple_of(3) || size.is_multiple_of(5) {
        size += 2;
    }
    size
}

/// `qh_gethash` over a ridge's vertices less `skip`, on vertex ids.
fn gethash_vertices(hashsize: usize, set: &[VertexId], skip: VertexId) -> usize {
    let mut hash: u32 = 0;
    for &v in set {
        if v != skip {
            hash = hash.wrapping_add(v.0.wrapping_mul(2_654_435_761));
        }
    }
    (hash % hashsize as u32) as usize
}

/// `qh_vertexintersect_new` (`poly2_r.c:3871-3888`): the vertices both
/// decreasing sets hold.
fn vertexintersect_new(qh: &Qhull<'_>, a: &[VertexId], b: &[VertexId]) -> Vec<VertexId> {
    let mut out = Vec::with_capacity(DIM - 1);
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        if a[i] == b[j] {
            out.push(a[i]);
            i += 1;
            j += 1;
        } else if qh.v(a[i]).id > qh.v(b[j]).id {
            i += 1;
        } else {
            j += 1;
        }
    }
    out
}

/// `qh_setequal_except` with both skips given (`qset_r.c:631-664`): whether
/// `a` less `skip_a` equals `b` less `skip_b`, each found once.
fn setequal_except(a: &[VertexId], skip_a: VertexId, b: &[VertexId], skip_b: VertexId) -> bool {
    let (mut i, mut j, mut skip) = (0, 0, 0);
    loop {
        if a.get(i) == Some(&skip_a) {
            skip += 1;
            i += 1;
        }
        if b.get(j) == Some(&skip_b) {
            skip += 1;
            j += 1;
        }
        let Some(&elem_a) = a.get(i) else { break };
        i += 1;
        let elem_b = b.get(j).copied();
        j += 1;
        if Some(elem_a) != elem_b {
            return false;
        }
    }
    skip == 2 && b.get(j).is_none()
}
