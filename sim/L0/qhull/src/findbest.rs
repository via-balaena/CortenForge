// Ported from Qhull, libqhull_r (geom_r.c's qh_findbest, qh_findbestnew,
// qh_findbesthorizon; geom2_r.c's qh_sharpnewfacets; user_r.h's macros).
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// geom_r.c, geom2_r.c: Copyright (c) 1993-2020 The Geometry Center.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! The facet a point is furthest above, by a directed search from a start
//! facet (`qh_findbest`), over the new facets (`qh_findbestnew`), and among
//! the coplanar facets around a best one (`qh_findbesthorizon`).

use crate::error::Error;
use crate::set;
use crate::state::{FacetId, PointId, Qhull, REAL_MAX, fmax_, minimize_};

/// The facet a point is best placed above, with its distance and whether it
/// is clearly outside (`dist`, `isoutside` of qhull's out-parameters).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Best {
    pub(crate) facet: FacetId,
    pub(crate) dist: f64,
    pub(crate) isoutside: bool,
}

impl Qhull<'_> {
    /// `qh_USEfindbestnew` (`user_r.h:903`): after 50 merges.
    pub(crate) const fn use_findbestnew(&self) -> bool {
        self.Ztotmerge > 50
    }

    /// `qh_DISToutside` (`user_r.h:730`) under merging.
    pub(crate) fn dist_outside(&self) -> f64 {
        let factor = if self.use_findbestnew() { 2.0 } else { 1.0 };
        factor * fmax_(2.0 * self.MINoutside, self.max_outside)
    }

    /// `qh_SEARCHdist` (`user_r.h:888`).
    fn search_dist(&self) -> f64 {
        let factor = if self.use_findbestnew() { 2.0 } else { 1.0 };
        factor
            * (self.max_outside + 2.0 * self.DISTround + fmax_(self.MINvisible, self.MAXcoplanar))
    }

    /// `qh_findbest` with `noupper` (no facet is upper Delaunay in a convex
    /// hull) (`geom_r.c:156-258`).
    pub(crate) fn findbest(
        &mut self,
        point: PointId,
        startfacet: FacetId,
        bestoutside: bool,
        isnewfacets: bool,
    ) -> Result<Best, Error> {
        let p = *self.point(point);
        let mut bestdist = -REAL_MAX / 2.0;
        let mut bestfacet: Option<FacetId> = None;
        self.visit_id += 1;
        let visitid = self.visit_id;
        let mut testhorizon = true;
        if !self.f(startfacet).flipped {
            let dist = self.distplane(&p, startfacet);
            if !bestoutside && dist >= self.MINoutside {
                return Ok(Best {
                    facet: startfacet,
                    dist,
                    isoutside: true,
                });
            }
            bestdist = dist;
            bestfacet = Some(startfacet);
        }
        self.fm(startfacet).visitid = visitid;
        let mut facet = Some(startfacet);
        while let Some(cur) = facet {
            let mut switched: Option<FacetId> = None;
            let mut i = 0;
            while let Some(neighbor) = set::get(&self.f(cur).neighbors, i) {
                i += 1;
                if !self.f(neighbor).newfacet && isnewfacets {
                    continue;
                }
                if self.f(neighbor).visitid == visitid {
                    continue;
                }
                self.fm(neighbor).visitid = visitid;
                if !self.f(neighbor).flipped {
                    let dist = self.distplane(&p, neighbor);
                    if dist > bestdist {
                        if !bestoutside && dist >= self.MINoutside {
                            return Ok(Best {
                                facet: neighbor,
                                dist,
                                isoutside: true,
                            });
                        }
                        bestfacet = Some(neighbor);
                        bestdist = dist;
                        switched = Some(neighbor);
                        break;
                    }
                }
            }
            facet = switched;
        }
        if isnewfacets {
            match bestfacet {
                None => {
                    let Some(newfacets) = self.newfacet_list else {
                        return Err(Error::qhull(
                            6001,
                            "merging has formed and deleted a cone of new facets.  Can not continue.",
                        ));
                    };
                    let found = self.findbestnew(point, newfacets, bestoutside)?;
                    bestfacet = Some(found.facet);
                    bestdist = found.dist;
                    testhorizon = false;
                }
                Some(best) => {
                    if !self.findbest_notsharp && bestdist < -self.DISTround {
                        if self.sharpnewfacets() {
                            let found = self.findbestnew(point, best, bestoutside)?;
                            bestfacet = Some(found.facet);
                            bestdist = found.dist;
                            testhorizon = false;
                            self.findbestnew = true;
                        } else {
                            self.findbest_notsharp = true;
                        }
                    }
                }
            }
        }
        let Some(mut best) = bestfacet else {
            return Err(Error::Unported {
                routine: "qh_findbestlower",
            });
        };
        if testhorizon {
            best = self.findbesthorizon(false, &p, best, &mut bestdist);
        }
        Ok(Best {
            facet: best,
            dist: bestdist,
            isoutside: bestdist >= self.MINoutside,
        })
    }

    /// `qh_findbesthorizon` with `noupper` (`geom_r.c:306-401`): from
    /// `startfacet`, the best facet among it and its neighbours within the
    /// search distance, searched breadth by breadth through qhull's coplanar
    /// facet set. With `ischeckmax`, each facet's `maxoutside` grows to the
    /// distance found.
    pub(crate) fn findbesthorizon(
        &mut self,
        ischeckmax: bool,
        point: &[f64; 3],
        startfacet: FacetId,
        bestdist: &mut f64,
    ) -> FacetId {
        let mut bestfacet = startfacet;
        let mut nextfacet: Option<FacetId> = None;
        self.visit_id += 1;
        let visitid = self.visit_id;
        if ischeckmax && *bestdist > self.f(startfacet).maxoutside {
            self.fm(startfacet).maxoutside = *bestdist;
        }
        let searchdist = self.search_dist();
        let mut minsearch = *bestdist - searchdist;
        if ischeckmax {
            minimize_(&mut minsearch, -searchdist);
        }
        // qhull's `qh.coplanarfacetset`, used as a stack.
        let mut coplanarfacetset: Vec<FacetId> = Vec::new();
        self.fm(startfacet).visitid = visitid;
        let mut facet = startfacet;
        loop {
            let is_5x_minsearch =
                ischeckmax && self.f(facet).nummerge > 10 && self.f(facet).neighbors.len() > 100;
            let mut i = 0;
            while let Some(neighbor) = set::get(&self.f(facet).neighbors, i) {
                i += 1;
                if self.f(neighbor).visitid == visitid {
                    continue;
                }
                self.fm(neighbor).visitid = visitid;
                if !self.f(neighbor).flipped {
                    let dist = self.distplane(point, neighbor);
                    if dist > *bestdist {
                        if !ischeckmax {
                            minsearch = dist - searchdist;
                            if dist > *bestdist + searchdist {
                                coplanarfacetset.clear();
                            }
                        }
                        bestfacet = neighbor;
                        *bestdist = dist;
                    } else if is_5x_minsearch {
                        if dist < 5.0 * minsearch {
                            continue;
                        }
                    } else if dist < minsearch {
                        continue;
                    }
                    if ischeckmax && dist > self.f(neighbor).maxoutside {
                        self.fm(neighbor).maxoutside = dist;
                    }
                }
                if let Some(n) = nextfacet {
                    coplanarfacetset.push(n);
                }
                nextfacet = Some(neighbor);
            }
            match nextfacet.take() {
                Some(n) => facet = n,
                None => match coplanarfacetset.pop() {
                    Some(n) => facet = n,
                    None => break,
                },
            }
        }
        bestfacet
    }

    /// `qh_findbestnew` (`geom_r.c:446-519`): every new facet from
    /// `startfacet` to the end of the list, then from the new facet list's
    /// head to `startfacet`, then `qh_findbesthorizon` about the best.
    pub(crate) fn findbestnew(
        &mut self,
        point: PointId,
        startfacet: FacetId,
        bestoutside: bool,
    ) -> Result<Best, Error> {
        let p = *self.point(point);
        let mut bestdist = -REAL_MAX / 2.0;
        let mut bestfacet: Option<FacetId> = None;
        self.visit_id += 1;
        let visitid = self.visit_id;
        if self.f(startfacet).next.is_none() {
            return Err(Error::qhull(
                6001,
                "merging has formed and deleted a cone of new facets.  Can not continue.",
            ));
        }
        let isdistoutside = !bestoutside;
        let distoutside = if isdistoutside {
            self.dist_outside()
        } else {
            0.0
        };
        for pass in 0..2 {
            let mut cur = if pass == 0 {
                Some(startfacet)
            } else {
                self.newfacet_list
            };
            while let Some(facet) = cur {
                if self.f(facet).next.is_none() {
                    break;
                }
                if facet == startfacet && pass == 1 {
                    break;
                }
                self.fm(facet).visitid = visitid;
                if !self.f(facet).flipped {
                    let dist = self.distplane(&p, facet);
                    if dist > bestdist {
                        bestfacet = Some(facet);
                        if isdistoutside && dist >= distoutside {
                            return Ok(Best {
                                facet,
                                dist,
                                isoutside: true,
                            });
                        }
                        bestdist = dist;
                    }
                }
                cur = self.f(facet).next;
            }
        }
        let start = bestfacet.unwrap_or(startfacet);
        let best = self.findbesthorizon(false, &p, start, &mut bestdist);
        Ok(Best {
            facet: best,
            dist: bestdist,
            isoutside: bestdist >= self.MINoutside,
        })
    }

    /// `qh_sharpnewfacets` (`geom2_r.c:2131-2155`): whether two new facets'
    /// normals lie in different quadrants.
    fn sharpnewfacets(&self) -> bool {
        let mut quadrant = [false; 3];
        for facet in self.facets_from(self.newfacet_list) {
            let normal = self.normal(facet);
            if Some(facet) == self.newfacet_list {
                for k in (0..3).rev() {
                    quadrant[k] = normal[k] > 0.0;
                }
            } else {
                for k in (0..3).rev() {
                    if quadrant[k] != (normal[k] > 0.0) {
                        return true;
                    }
                }
            }
        }
        false
    }
}
