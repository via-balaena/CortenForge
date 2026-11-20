// Ported from Qhull, libqhull_r (poly2_r.c's qh_triangulate and its helpers;
// poly_r.c's qh_update_vertexneighbors and qh_makenew_nonsimplicial).
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// poly_r.c, poly2_r.c: Copyright (c) 1993-2020 The Geometry Center.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! `Qt`: each non-simplicial facet a merge made split into triangles about its
//! first vertex (`qh_triangulate`), the null triangles dropped, one triangle of
//! each owning the facet's coplanar and outside points.

use crate::error::Error;
use crate::set;
use crate::state::{FacetId, Qhull, VertexId};

impl Qhull<'_> {
    // qhull's routine kept whole, so it reads against its source.
    #[allow(clippy::too_many_lines)]
    /// `qh_triangulate` (`poly2_r.c:3482-3626`).
    pub(crate) fn triangulate(&mut self) -> Result<(), Error> {
        if self.hasTriangulation {
            return Ok(());
        }
        self.visit_id += 1;
        self.newvertex_list = self.vertex_tail;
        let mut triangulated_facet_list: Option<FacetId> = None;
        let mut triangulated_vertex_list: Option<VertexId> = None;
        let mut cur = self.facet_list;
        while let Some(facet) = cur {
            let Some(nextfacet) = self.f(facet).next else {
                break;
            };
            if !(self.f(facet).visible || self.f(facet).simplicial) {
                if triangulated_facet_list.is_none() {
                    triangulated_facet_list = Some(facet);
                }
                self.triangulate_facet(facet, &mut triangulated_vertex_list)?;
            }
            cur = Some(nextfacet);
        }
        let mut cur = triangulated_facet_list;
        while let Some(facet) = cur {
            let Some(nextfacet) = self.f(facet).next else {
                break;
            };
            if !self.f(facet).visible {
                if let Some(ridges) = &self.f(facet).ridges {
                    if !ridges.is_empty() {
                        return Err(Error::qhull(6161, "ridges still defined for a facet"));
                    }
                    self.fm(facet).ridges = None;
                }
                let v = &self.f(facet).vertices;
                if v.first() == v.get(1) {
                    self.triangulate_null(facet)?;
                }
            }
            cur = Some(nextfacet);
        }
        self.visible_list = self.facet_tail;
        // qhull pops each merge and runs `qh_triangulate_mirror` for a mirror
        // one only (`:3534-3542`); `triangulate_link` refuses before it would
        // queue one, so what is left is freed.
        self.degen_mergeset.clear();
        self.newvertex_list = triangulated_vertex_list;
        self.visible_list = None;
        self.update_vertexneighbors();
        self.resetlists(false);
        // The tricoplanar facets whose two neighbours are one facet's
        // triangles: degenerate, never owners.
        for facet in self.facets_from(triangulated_facet_list) {
            if self.f(facet).tricoplanar && !self.f(facet).visible {
                let neighbors = self.f(facet).neighbors.clone();
                let owner_of = |qh: &Self, n: FacetId| {
                    if qh.f(n).tricoplanar {
                        qh.f(n).f
                    } else {
                        Some(n)
                    }
                };
                let mut orig_neighbor = None;
                for (neighbor_i, &neighbor) in neighbors.iter().enumerate() {
                    if neighbor_i == 0 {
                        orig_neighbor = owner_of(self, neighbor);
                    } else if orig_neighbor == owner_of(self, neighbor) {
                        self.fm(facet).degenerate = true;
                        break;
                    }
                }
            }
        }
        let mut owner: Option<FacetId> = None;
        let mut visible: Option<FacetId> = None;
        let mut cur = triangulated_facet_list;
        while let Some(facet) = cur {
            let Some(mut nextfacet) = self.f(facet).next else {
                break;
            };
            if self.f(facet).visible {
                if self.f(facet).tricoplanar {
                    self.delfacet(facet);
                    self.num_visible -= 1;
                } else {
                    if let Some(v) = visible
                        && owner.is_none()
                    {
                        self.delfacet(v);
                        self.num_visible -= 1;
                    }
                    visible = Some(facet);
                    owner = None;
                }
            } else if self.f(facet).tricoplanar {
                let Some(vis) = visible.filter(|&v| self.f(facet).f == Some(v)) else {
                    return Err(Error::qhull(
                        6162,
                        "tricoplanar facet not owned by its visible, non-simplicial facet",
                    ));
                };
                if owner.is_some() {
                    self.fm(facet).f = owner;
                } else if !self.f(facet).degenerate {
                    owner = Some(facet);
                    if let Some(n) = self.f(vis).next {
                        nextfacet = n;
                    }
                    let coplanarset = self.fm(vis).coplanarset.take();
                    let outsideset = self.fm(vis).outsideset.take();
                    let f = self.fm(facet);
                    f.keepcentrum = true;
                    f.coplanarset = coplanarset;
                    f.outsideset = outsideset;
                    self.fm(vis).center = None;
                    self.fm(vis).normal = None;
                    self.delfacet(vis);
                    self.num_visible -= 1;
                }
            }
            self.fm(facet).degenerate = false;
            cur = Some(nextfacet);
        }
        if let Some(v) = visible
            && owner.is_none()
        {
            self.delfacet(v);
            self.num_visible -= 1;
        }
        self.hasTriangulation = true;
        Ok(())
    }

    /// `qh_triangulate_facet` (`poly2_r.c:3671-3735`): a fan of triangles
    /// about the facet's first vertex, one per ridge.
    fn triangulate_facet(
        &mut self,
        facet_a: FacetId,
        first_vertex: &mut Option<VertexId>,
    ) -> Result<(), Error> {
        self.first_newfacet = self.facet_id;
        let mut i = 0;
        while let Some(neighbor) = set::get(&self.f(facet_a).neighbors, i) {
            i += 1;
            let n = self.fm(neighbor);
            n.seen = false;
            n.coplanarhorizon = false;
        }
        self.visible_list = self.facet_tail;
        self.newfacet_list = self.facet_tail;
        self.fm(facet_a).visitid = self.visit_id;
        let apex = self.f(facet_a).vertices[0];
        self.makenew_nonsimplicial(facet_a, apex)?;
        self.willdelete(facet_a, None);
        let (normal, center, offset, maxoutside, good) = {
            let f = self.f(facet_a);
            (f.normal, f.center, f.offset, f.maxoutside, f.good)
        };
        for newfacet in self.facets_from(self.newfacet_list) {
            let f = self.fm(newfacet);
            f.tricoplanar = true;
            f.f = Some(facet_a);
            f.degenerate = false;
            f.good = good;
            f.keepcentrum = false;
            f.normal = normal;
            f.center = center;
            f.offset = offset;
            f.maxoutside = maxoutside;
        }
        self.matchnewfacets()?;
        if first_vertex.is_none() {
            *first_vertex = self.newvertex_list;
        }
        self.newvertex_list = None;
        self.visible_list = None;
        self.update_vertexneighbors();
        self.resetlists(false);
        Ok(())
    }

    /// `qh_triangulate_null` (`poly2_r.c:3816-3824`): a triangle with a vertex
    /// twice deleted, its two neighbours made each other's.
    fn triangulate_null(&mut self, facet_a: FacetId) -> Result<(), Error> {
        let neighbor = self.f(facet_a).neighbors[0];
        let otherfacet = self.f(facet_a).neighbors[1];
        self.triangulate_link(facet_a, neighbor, facet_a, otherfacet)?;
        self.willdelete(facet_a, None);
        Ok(())
    }

    /// `qh_triangulate_link` (`poly2_r.c:3745-3769`): `facet_a` and `facet_b`
    /// made each other's neighbours in place of `oldfacet_a` and `oldfacet_b`.
    /// Where they are each other's already, qhull queues a mirror merge that
    /// deletes both (`qh_triangulate_mirror`): not ported. Where one lists
    /// the other and not the reverse, qhull stops (QH6163).
    fn triangulate_link(
        &mut self,
        oldfacet_a: FacetId,
        facet_a: FacetId,
        oldfacet_b: FacetId,
        facet_b: FacetId,
    ) -> Result<(), Error> {
        let a_lists_b = set::setin(&self.f(facet_a).neighbors, facet_b);
        let b_lists_a = set::setin(&self.f(facet_b).neighbors, facet_a);
        if a_lists_b != b_lists_a {
            return Err(Error::qhull(
                6163,
                "neighbors do not match for null facet or mirrored facets",
            ));
        }
        if a_lists_b {
            return Err(Error::Unported {
                routine: "qh_triangulate_mirror",
            });
        }
        set::setreplace(&mut self.fm(facet_b).neighbors, oldfacet_b, facet_a);
        set::setreplace(&mut self.fm(facet_a).neighbors, oldfacet_a, facet_b);
        Ok(())
    }

    /// `qh_update_vertexneighbors` (`poly_r.c:1279-1344`) as `qh_triangulate`
    /// calls it: no visible list, new facets appended to their vertices'
    /// neighbours, visible facets dropped from the new vertices'.
    fn update_vertexneighbors(&mut self) {
        if !self.VERTEXneighbors {
            return;
        }
        for vertex in self.vertices_from(self.newvertex_list) {
            let mut any = false;
            let mut i = 0;
            while let Some(neighbor) = set::get(&self.v(vertex).neighbors, i) {
                if self.f(neighbor).visible {
                    self.vm(vertex).neighbors[i] = FacetId::NULL;
                    any = true;
                }
                i += 1;
            }
            if any {
                set::setcompact(&mut self.vm(vertex).neighbors);
            }
        }
        for newfacet in self.facets_from(self.newfacet_list) {
            if self.first_newfacet != 0 && self.f(newfacet).id >= self.first_newfacet {
                for v in self.f(newfacet).vertices.clone() {
                    self.vm(v).neighbors.push(newfacet);
                }
            } else {
                for v in self.f(newfacet).vertices.clone() {
                    set::setunique(&mut self.vm(v).neighbors, newfacet);
                }
            }
        }
    }

    /// `qh_makenew_nonsimplicial` (`poly_r.c:621-692`) without
    /// `NEWtentative`: a new facet over each ridge to a horizon facet, which
    /// takes the ridge where the horizon facet is not simplicial. Its last new
    /// facet.
    pub(crate) fn makenew_nonsimplicial(
        &mut self,
        visible: FacetId,
        apex: VertexId,
    ) -> Result<Option<FacetId>, Error> {
        let mut newfacet = None;
        let ridges = self.f(visible).ridges.clone().unwrap_or_default();
        let mut j = 0;
        while let Some(ridge) = set::get(&ridges, j) {
            j += 1;
            let Some(neighbor) = self.otherfacet(ridge, visible) else {
                continue;
            };
            if self.f(neighbor).visible {
                if self.f(neighbor).visitid == self.visit_id {
                    // Deleted on its second visit.
                    self.delridge(ridge);
                }
            } else {
                let toporient = self.r(ridge).top == Some(visible);
                let mut vertices = Vec::with_capacity(3);
                vertices.push(apex);
                vertices.extend_from_slice(&self.r(ridge).vertices);
                let nf = self.makenewfacet(vertices, toporient, neighbor);
                newfacet = Some(nf);
                if self.f(neighbor).coplanarhorizon {
                    self.fm(nf).mergehorizon = true;
                    if self.f(neighbor).seen {
                        let samecycle = self.f(neighbor).f.unwrap_or(nf);
                        let next = self.f(samecycle).f;
                        self.fm(nf).f = next;
                        self.fm(samecycle).f = Some(nf);
                    } else {
                        self.fm(nf).f = Some(nf);
                        self.fm(neighbor).f = Some(nf);
                    }
                }
                if self.f(neighbor).seen {
                    if self.f(neighbor).simplicial {
                        return Err(Error::qhull(
                            6105,
                            "simplicial facet sharing two ridges with a visible facet",
                        ));
                    }
                    self.fm(neighbor).neighbors.push(nf);
                } else {
                    set::setreplace(&mut self.fm(neighbor).neighbors, visible, nf);
                }
                if self.f(neighbor).simplicial {
                    if let Some(r) = self.fm(neighbor).ridges.as_mut() {
                        set::setdel(r, ridge);
                    }
                    self.delridge(ridge);
                } else {
                    self.fm(nf).ridges.get_or_insert_with(Vec::new).push(ridge);
                    let r = self.rm(ridge);
                    if toporient {
                        r.top = Some(nf);
                        r.simplicialtop = true;
                    } else {
                        r.bottom = Some(nf);
                        r.simplicialbot = true;
                    }
                }
            }
            self.fm(neighbor).seen = true;
        }
        Ok(newfacet)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Three facets, `null` between `a` and `b`, each listing `null`; `a` and
    /// `b` listing each other where asked.
    fn three_facets(a_lists_b: bool, b_lists_a: bool) -> (Qhull<'static>, [FacetId; 3]) {
        let mut qh = Qhull::new(&[], 0);
        let [null, a, b] = [qh.newfacet(), qh.newfacet(), qh.newfacet()];
        qh.fm(null).neighbors = vec![a, b];
        qh.fm(a).neighbors = if a_lists_b { vec![null, b] } else { vec![null] };
        qh.fm(b).neighbors = if b_lists_a { vec![null, a] } else { vec![null] };
        (qh, [null, a, b])
    }

    #[test]
    fn a_null_facet_s_neighbours_become_each_other_s() {
        let (mut qh, [null, a, b]) = three_facets(false, false);
        assert_eq!(qh.triangulate_link(null, a, null, b), Ok(()));
        assert_eq!(qh.f(a).neighbors, [b]);
        assert_eq!(qh.f(b).neighbors, [a]);
    }

    #[test]
    fn neighbours_already_each_other_s_are_a_mirror_merge_not_ported() {
        let (mut qh, [null, a, b]) = three_facets(true, true);
        assert_eq!(
            qh.triangulate_link(null, a, null, b),
            Err(Error::Unported {
                routine: "qh_triangulate_mirror"
            })
        );
    }

    #[test]
    fn neighbours_listed_one_way_stop_qhull() {
        for (a_lists_b, b_lists_a) in [(true, false), (false, true)] {
            let (mut qh, [null, a, b]) = three_facets(a_lists_b, b_lists_a);
            assert!(matches!(
                qh.triangulate_link(null, a, null, b),
                Err(Error::Qhull { code: 6163, .. })
            ));
        }
    }
}
