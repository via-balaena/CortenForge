// Ported from Qhull, libqhull_r (geom_r.c, geom2_r.c, poly_r.c's qh_checkflipped,
// geom_r.h's det2_ and det3_, user_r.h's constants).
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// geom_r.c, geom2_r.c, poly_r.c, geom_r.h: Copyright (c) 1993-2020 The Geometry
// Center.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! Hyperplanes, distances and the input's roundoff constants, each in qhull's
//! order of operations: the oracle compiles qhull with `-ffp-contract=off`
//! and Rust never contracts, so the same order gives the same bits.

use crate::error::Error;
use crate::state::{
    DIM, FacetId, PointId, Qhull, REAL_EPSILON, REAL_MAX, VertexId, fmax_, maximize_, minimize_,
};

/// `qh_RATIOmaxsimplex` (`user_r.h:833`).
const RATIO_MAXSIMPLEX: f64 = 1.0e-3;
/// `qh_RATIOnearinside` (`user_r.h:847`).
const RATIO_NEARINSIDE: f64 = 5.0;
/// `qh_WIDEcoplanar` (`user_r.h:947`).
const WIDE_COPLANAR: f64 = 6.0;

/// `det2_` (`geom_r.h:69`).
fn det2(a1: f64, a2: f64, b1: f64, b2: f64) -> f64 {
    a1 * b2 - a2 * b1
}

/// `det3_` (`geom_r.h:80`).
#[allow(clippy::too_many_arguments)] // qhull's macro: the 3×3 matrix by rows
fn det3(a1: f64, a2: f64, a3: f64, b1: f64, b2: f64, b3: f64, c1: f64, c2: f64, c3: f64) -> f64 {
    a1 * det2(b2, b3, c2, c3) - b1 * det2(a2, a3, c2, c3) + c1 * det2(a2, a3, b2, b3)
}

/// `qh_divzero` (`geom2_r.c:446-469`): `numer/denom`, or 0 and `true` where
/// the quotient would overflow `mindenom1`'s bound.
fn divzero(numer: f64, denom: f64, mindenom1: f64) -> (f64, bool) {
    if numer < mindenom1 && numer > -mindenom1 {
        let numerx = numer.abs();
        let denomx = denom.abs();
        if numerx < denomx {
            return (numer / denom, false);
        }
        return (0.0, true);
    }
    let temp = denom / numer;
    if temp > mindenom1 || temp < -mindenom1 {
        (numer / denom, false)
    } else {
        (0.0, true)
    }
}

impl Qhull<'_> {
    /// `qh_distplane` (`geom_r.c:50-99`): the point's signed distance to the
    /// facet's hyperplane, summed left to right as qhull's 3-d case does.
    pub(crate) fn distplane(&self, point: &[f64; DIM], facet: FacetId) -> f64 {
        let f = self.f(facet);
        let normal = f.normal.unwrap_or([0.0; DIM]);
        f.offset + point[0] * normal[0] + point[1] * normal[1] + point[2] * normal[2]
    }

    /// `qh_distplane` for an input point.
    pub(crate) fn distplane_point(&self, point: PointId, facet: FacetId) -> f64 {
        self.distplane(self.point(point), facet)
    }

    /// `qh_normalize2` with neither `minnorm` nor `ismin` (`geom_r.c:845-918`),
    /// as every 3-d caller passes them.
    fn normalize2(&self, normal: &mut [f64; DIM], toporient: bool) {
        let mut norm =
            (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        if norm > self.MINdenom {
            if !toporient {
                norm = -norm;
            }
            normal[0] /= norm;
            normal[1] /= norm;
            normal[2] /= norm;
        } else if norm == 0.0 {
            let temp = (1.0 / DIM as f64).sqrt();
            for c in normal.iter_mut() {
                *c = temp;
            }
        } else {
            if !toporient {
                norm = -norm;
            }
            for k in 0..DIM {
                let (temp, zerodiv) = divzero(normal[k], norm, self.MINdenom_1);
                if zerodiv {
                    // `qh_maxabsval` (`geom2_r.c:1058-1071`) over the normal as
                    // divided so far: the first of the largest magnitudes.
                    let mut maxval = -REAL_MAX;
                    let mut maxp = 0;
                    for (j, c) in normal.iter().enumerate() {
                        if c.abs() > maxval {
                            maxval = c.abs();
                            maxp = j;
                        }
                    }
                    let temp = if normal[maxp] * norm >= 0.0 {
                        1.0
                    } else {
                        -1.0
                    };
                    *normal = [0.0; DIM];
                    normal[maxp] = temp;
                    return;
                }
                normal[k] = temp;
            }
        }
    }

    /// `qh_sethyperplane_det` for 3-d (`geom_r.c:1152-1221`): the plane
    /// through `rows` (its first is `point0`) by 2×2 determinants. `true`
    /// where a row lies off the plane beyond `qh.DISTround`.
    fn sethyperplane_det(
        &self,
        rows: &[[f64; DIM]; DIM],
        toporient: bool,
        normal: &mut [f64; DIM],
    ) -> (f64, bool) {
        let point0 = rows[0];
        let dx = |a: usize, b: usize| rows[a][0] - rows[b][0];
        let dy = |a: usize, b: usize| rows[a][1] - rows[b][1];
        let dz = |a: usize, b: usize| rows[a][2] - rows[b][2];
        normal[0] = det2(dy(2, 0), dz(2, 0), dy(1, 0), dz(1, 0));
        normal[1] = det2(dx(1, 0), dz(1, 0), dx(2, 0), dz(2, 0));
        normal[2] = det2(dx(2, 0), dy(2, 0), dx(1, 0), dy(1, 0));
        self.normalize2(normal, toporient);
        let offset = -(point0[0] * normal[0] + point0[1] * normal[1] + point0[2] * normal[2]);
        let maxround = self.DISTround;
        let mut nearzero = false;
        for i in (0..DIM).rev() {
            // `point != point0`: rows 1 and 2 (the facet's other vertices).
            if i != 0 {
                let point = rows[i];
                let dist =
                    offset + (point[0] * normal[0] + point[1] * normal[1] + point[2] * normal[2]);
                if dist > maxround || dist < -maxround {
                    nearzero = true;
                    break;
                }
            }
        }
        (offset, nearzero)
    }

    /// `qh_gausselim` (`geom_r.c:620-671`) of `numrow` rows of `numcol`
    /// columns, rows swapped by index as qhull swaps their pointers. Returns
    /// whether a pivot came at or under `qh.NEARzero`.
    fn gausselim(
        &self,
        rows: &mut [[f64; DIM]],
        order: &mut [usize],
        numrow: usize,
        sign: &mut bool,
    ) -> bool {
        let numcol = DIM;
        let mut nearzero = false;
        for k in 0..numrow {
            let mut pivot_abs = rows[order[k]][k].abs();
            let mut pivoti = k;
            for (i, &row) in order.iter().enumerate().take(numrow).skip(k + 1) {
                let temp = rows[row][k].abs();
                if temp > pivot_abs {
                    pivot_abs = temp;
                    pivoti = i;
                }
            }
            if pivoti != k {
                order.swap(pivoti, k);
                *sign = !*sign;
            }
            if pivot_abs <= self.NEARzero[k] {
                nearzero = true;
                if pivot_abs == 0.0 {
                    continue; // LABELnextcol
                }
            }
            let pivotrow = rows[order[k]];
            let pivot = pivotrow[k];
            for &ai in order.iter().take(numrow).skip(k + 1) {
                let n = rows[ai][k] / pivot;
                for (c, p) in rows[ai].iter_mut().zip(pivotrow).take(numcol).skip(k + 1) {
                    *c -= n * p;
                }
            }
        }
        nearzero
    }

    /// `qh_backnormal` (`geom_r.c:560-597`): the normal from the upper
    /// triangular `rows`. Returns whether a diagonal was zero.
    fn backnormal(
        &self,
        rows: &[[f64; DIM]],
        order: &[usize],
        numrow: usize,
        sign: bool,
        normal: &mut [f64; DIM],
    ) -> bool {
        let numcol = DIM;
        let mut zerocol: Option<usize> = None;
        normal[numcol - 1] = if sign { -1.0 } else { 1.0 };
        for i in (0..numrow).rev() {
            let row = &rows[order[i]];
            normal[i] = 0.0;
            for j in (i + 1)..numcol {
                normal[i] -= row[j] * normal[j];
            }
            let diagonal = row[i];
            if diagonal.abs() > self.MINdenom_2 {
                normal[i] /= diagonal;
            } else {
                let (q, waszero) = divzero(normal[i], diagonal, self.MINdenom_1_2);
                normal[i] = q;
                if waszero {
                    zerocol = Some(i);
                    normal[i] = if sign { -1.0 } else { 1.0 };
                    for c in normal.iter_mut().skip(i + 1) {
                        *c = 0.0;
                    }
                }
            }
        }
        zerocol.is_some()
    }

    /// `qh_sethyperplane_gauss` for 3-d (`geom_r.c:1250-1281`): the plane by
    /// Gaussian elimination of the rows less `point0`.
    fn sethyperplane_gauss(
        &self,
        rows: &mut [[f64; DIM]],
        point0: &[f64; DIM],
        toporient: bool,
        normal: &mut [f64; DIM],
    ) -> (f64, bool) {
        let mut sign = toporient;
        let mut order = [0usize, 1, 2];
        let mut nearzero = self.gausselim(rows, &mut order, DIM - 1, &mut sign);
        for k in (0..DIM - 1).rev() {
            if rows[order[k]][k] < 0.0 {
                sign = !sign;
            }
        }
        let nearzero2 = self.backnormal(rows, &order, DIM - 1, sign, normal);
        if nearzero2 {
            nearzero = true;
        }
        self.normalize2(normal, true);
        let mut offset = -(point0[0] * normal[0]);
        for k in 1..DIM {
            offset -= point0[k] * normal[k];
        }
        (offset, nearzero)
    }

    /// `qh_setfacetplane` for 3-d (`geom_r.c:971-1105`): the facet's normal
    /// and offset from its vertices, by determinants, by Gaussian elimination
    /// where they come out near zero; then `qh_checkflipped`.
    pub(crate) fn setfacetplane(&mut self, facet: FacetId) {
        let verts = self.f(facet).vertices.clone();
        let point0 = *self.vcoords(verts[0]);
        let mut rows = [[0.0; DIM]; DIM];
        for (i, &v) in verts.iter().enumerate().take(DIM) {
            rows[i] = *self.vcoords(v);
        }
        let toporient = self.f(facet).toporient;
        let mut normal = [0.0; DIM];
        let (mut offset, nearzero) = self.sethyperplane_det(&rows, toporient, &mut normal);
        if nearzero {
            // Each vertex but `point0`, less `point0`, in the facet's order.
            let mut grows = [[0.0; DIM]; DIM];
            let mut i = 0;
            // `vertex->point != point0`: the point pointers, so every vertex
            // but the first.
            for &v in &verts {
                if self.vpoint(v) != self.vpoint(verts[0]) {
                    let coord = self.vcoords(v);
                    for k in 0..DIM {
                        grows[i][k] = coord[k] - point0[k];
                    }
                    i += 1;
                }
            }
            let (o, nz) = self.sethyperplane_gauss(&mut grows, &point0, toporient, &mut normal);
            offset = o;
            let f = self.fm(facet);
            f.normal = Some(normal);
            f.offset = offset;
            if nz {
                self.orientoutside(facet);
            }
        } else {
            let f = self.fm(facet);
            f.normal = Some(normal);
            f.offset = offset;
        }
        self.checkflipped(facet, true);
    }

    /// `qh_orientoutside` (`geom2_r.c:1405-1417`): the normal turned to point
    /// away from the interior point.
    pub(crate) fn orientoutside(&mut self, facet: FacetId) -> bool {
        let dist = self.distplane(&self.interior_point, facet);
        if dist > 0.0 {
            let f = self.fm(facet);
            if let Some(n) = f.normal.as_mut() {
                for c in n.iter_mut() {
                    *c = -*c;
                }
            }
            f.offset = -f.offset;
            return true;
        }
        false
    }

    /// `qh_checkflipped` with no distance asked for (`poly_r.c:237-257`):
    /// `false`, and the facet marked flipped, where the interior point is not
    /// clearly below it (`allerror`) or is above it.
    pub(crate) fn checkflipped(&mut self, facet: FacetId, allerror: bool) -> bool {
        if self.f(facet).flipped {
            return false;
        }
        let dist = self.distplane(&self.interior_point, facet);
        if (allerror && dist >= -self.DISTround) || (!allerror && dist > 0.0) {
            self.fm(facet).flipped = true;
            return false;
        }
        true
    }

    /// `qh_getangle` (`geom_r.c:685-698`): the dot product, summed from the
    /// first coordinate (qhull's loop counts `k` down but reads forward).
    pub(crate) fn getangle(a: &[f64; DIM], b: &[f64; DIM]) -> f64 {
        let mut angle = 0.0;
        for k in 0..DIM {
            angle += a[k] * b[k];
        }
        angle
    }

    /// `qh_getcenter` (`geom_r.c:710-729`): the vertices' mean.
    pub(crate) fn getcenter(&self, vertices: &[VertexId]) -> [f64; DIM] {
        let count = vertices.len() as f64;
        let mut center = [0.0; DIM];
        for (k, c) in center.iter_mut().enumerate() {
            *c = 0.0;
            for &v in vertices {
                *c += self.vcoords(v)[k];
            }
            *c /= count;
        }
        center
    }

    /// `qh_projectpoint` (`geom_r.c:935-947`).
    fn projectpoint(&self, point: &[f64; DIM], facet: FacetId, dist: f64) -> [f64; DIM] {
        let normal = self.normal(facet);
        let mut p = [0.0; DIM];
        for k in 0..DIM {
            p[k] = point[k] - dist * normal[k];
        }
        p
    }

    /// `qh_getcentrum` (`geom_r.c:741-753`): the vertices' mean projected
    /// onto the hyperplane.
    pub(crate) fn getcentrum(&self, facet: FacetId) -> [f64; DIM] {
        let point = self.getcenter(&self.f(facet).vertices);
        let dist = self.distplane(&point, facet);
        self.projectpoint(&point, facet, dist)
    }

    /// `qh_getdistance` (`geom_r.c:769-796`): of `facet`'s vertices not in
    /// `neighbor`, the larger magnitude of their extreme distances to it, and
    /// the extremes.
    pub(crate) fn getdistance(&mut self, facet: FacetId, neighbor: FacetId) -> (f64, f64, f64) {
        for v in self.f(facet).vertices.clone() {
            self.vm(v).seen = false;
        }
        for v in self.f(neighbor).vertices.clone() {
            self.vm(v).seen = true;
        }
        let mut mind = 0.0;
        let mut maxd = 0.0;
        for v in self.f(facet).vertices.clone() {
            if !self.v(v).seen {
                let dist = self.distplane(self.vcoords(v), neighbor);
                if dist < mind {
                    mind = dist;
                } else if dist > maxd {
                    maxd = dist;
                }
            }
        }
        let (mindist, maxdist) = (mind, maxd);
        mind = -mind;
        (if maxd > mind { maxd } else { mind }, mindist, maxdist)
    }

    /// `qh_determinant` for 2 and 3 rows (`geom2_r.c:84-113`), with whether it
    /// is near zero.
    fn determinant(&self, rows: &[[f64; DIM]], dim: usize) -> (f64, bool) {
        if dim == 2 {
            let det = det2(rows[0][0], rows[0][1], rows[1][0], rows[1][1]);
            (det, det.abs() < 10.0 * self.NEARzero[1])
        } else {
            let det = det3(
                rows[0][0], rows[0][1], rows[0][2], rows[1][0], rows[1][1], rows[1][2], rows[2][0],
                rows[2][1], rows[2][2],
            );
            (det, det.abs() < 10.0 * self.NEARzero[2])
        }
    }

    /// `qh_detsimplex` (`geom2_r.c:335-362`): the determinant of the first
    /// `dim` points less `apex`.
    fn detsimplex(&self, apex: PointId, points: &[PointId], dim: usize) -> (f64, bool) {
        let a = self.point(apex);
        let mut rows = [[0.0; DIM]; DIM];
        for (i, &p) in points.iter().enumerate().take(dim) {
            let c = self.point(p);
            for k in 0..dim {
                rows[i][k] = c[k] - a[k];
            }
        }
        self.determinant(&rows, dim)
    }

    /// `qh_distround` (`geom2_r.c:408-423`).
    fn distround(maxabs: f64, maxsumabs: f64) -> f64 {
        let mut maxdistsum = (DIM as f64).sqrt() * maxabs;
        minimize_(&mut maxdistsum, maxsumabs);
        REAL_EPSILON * (DIM as f64 * maxdistsum * 1.01 + maxabs)
    }

    /// `qh_detmaxoutside` (`geom2_r.c:181-189`).
    pub(crate) fn detmaxoutside(&mut self) {
        let mut maxoutside = fmax_(self.max_outside, self.ONEmerge + self.DISTround);
        maximize_(&mut maxoutside, self.MINoutside);
        self.MAXoutside = maxoutside;
    }

    /// `qh_maxmin` (`geom2_r.c:1100-1170`): each axis's extreme points (min
    /// then max, by axis) and the input's extents.
    pub(crate) fn maxmin(&mut self) -> Vec<PointId> {
        self.max_outside = 0.0;
        self.MAXabs_coord = 0.0;
        self.MAXwidth = -REAL_MAX;
        self.MAXsumcoord = 0.0;
        self.min_vertex = 0.0;
        self.WAScoplanar = false;
        self.ZEROall_ok = true;
        let mut set = Vec::with_capacity(2 * DIM);
        for k in 0..DIM {
            let mut minimum: PointId = 0;
            let mut maximum: PointId = 0;
            for (i, p) in self.points.iter().enumerate() {
                if self.point(maximum)[k] < p[k] {
                    maximum = i as PointId;
                } else if self.point(minimum)[k] > p[k] {
                    minimum = i as PointId;
                }
            }
            if k == DIM - 1 {
                self.MINlastcoord = self.point(minimum)[k];
                self.MAXlastcoord = self.point(maximum)[k];
            }
            let maxcoord = fmax_(self.point(maximum)[k], -self.point(minimum)[k]);
            let temp = self.point(maximum)[k] - self.point(minimum)[k];
            maximize_(&mut self.MAXwidth, temp);
            maximize_(&mut self.MAXabs_coord, maxcoord);
            self.MAXsumcoord += maxcoord;
            set.push(minimum);
            set.push(maximum);
            self.NEARzero[k] = 80.0 * self.MAXsumcoord * REAL_EPSILON;
        }
        set
    }

    /// `qh_detroundoff` (`geom2_r.c:221-316`) for `qhull Qt` in 3-d.
    pub(crate) fn detroundoff(&mut self) {
        self.DISTround = Self::distround(self.MAXabs_coord, self.MAXsumcoord);
        self.MINdenom = self.MINdenom_1 * self.MAXabs_coord;
        self.MINdenom_1_2 = (self.MINdenom_1 * DIM as f64).sqrt();
        self.MINdenom_2 = self.MINdenom_1_2 * self.MAXabs_coord;
        self.ANGLEround = 1.01 * DIM as f64 * REAL_EPSILON;
        self.premerge_centrum += 2.0 * self.DISTround;
        self.postmerge_centrum += 2.0 * self.DISTround;
        let mut maxangle = 1.0;
        minimize_(&mut maxangle, self.premerge_cos);
        minimize_(&mut maxangle, self.postmerge_cos);
        self.ONEmerge = (DIM as f64).sqrt() * self.MAXwidth * (1.0 - maxangle * maxangle).sqrt()
            + self.DISTround;
        let maxrho = DIM as f64 * self.premerge_centrum + self.DISTround;
        maximize_(&mut self.ONEmerge, maxrho);
        let maxrho = DIM as f64 * self.postmerge_centrum + self.DISTround;
        maximize_(&mut self.ONEmerge, maxrho);
        self.NEARinside = self.ONEmerge * RATIO_NEARINSIDE;
        if self.MINvisible > REAL_MAX / 2.0 {
            self.MINvisible = self.premerge_centrum;
        }
        if self.MAXcoplanar > REAL_MAX / 2.0 {
            self.MAXcoplanar = self.MINvisible;
        }
        self.MINoutside = 2.0 * self.MINvisible;
        self.WIDEfacet = self.MINoutside;
        maximize_(&mut self.WIDEfacet, WIDE_COPLANAR * self.MAXcoplanar);
        maximize_(&mut self.WIDEfacet, WIDE_COPLANAR * self.MINvisible);
        self.max_vertex = self.DISTround;
        self.min_vertex = -self.DISTround;
        self.detmaxoutside();
    }

    /// `qh_maxsimplex` for 3-d from no simplex (`geom2_r.c:1232-1350`): the
    /// initial simplex's four points, first from the extreme points
    /// `maxpoints`, then from every point where those span too little.
    pub(crate) fn maxsimplex(&self, maxpoints: &[PointId]) -> Result<Vec<PointId>, Error> {
        if self.MAXwidth <= 0.0 {
            return Err(Error::qhull(
                6421,
                "qh.MAXwidth required for qh_maxsimplex.  Used to estimate determinate",
            ));
        }
        let dim = DIM;
        let mut simplex: Vec<PointId> = Vec::with_capacity(dim + 1);
        let mut maxx: Option<PointId> = None;
        let mut minx: Option<PointId> = None;
        let mut mincoord = REAL_MAX;
        let mut maxcoord = -REAL_MAX;
        // `qh_setsize(maxpoints) >= 2` always holds: `qh_maxmin` gives six.
        for &p in maxpoints {
            if maxcoord < self.point(p)[0] {
                maxcoord = self.point(p)[0];
                maxx = Some(p);
            }
            if mincoord > self.point(p)[0] {
                mincoord = self.point(p)[0];
                minx = Some(p);
            }
        }
        let mut maxdet = maxcoord - mincoord;
        if let Some(p) = minx
            && !simplex.contains(&p)
        {
            simplex.push(p);
        }
        if simplex.len() < 2
            && let Some(p) = maxx
            && !simplex.contains(&p)
        {
            simplex.push(p);
        }
        let sizinit = simplex.len();
        if sizinit < 2 {
            return Err(Error::qhull(
                6013,
                "input is less than 3-dimensional since all points have the same x coordinate",
            ));
        }
        let mut maxnearzero = false;
        for i in sizinit..=dim {
            let prevdet = maxdet;
            let mut maxpoint: Option<PointId> = None;
            maxdet = -1.0;
            for &p in maxpoints {
                if !simplex.contains(&p) && Some(p) != maxpoint {
                    let (det, nearzero) = self.detsimplex(p, &simplex, i);
                    let det = det.abs();
                    if det > maxdet {
                        maxdet = det;
                        maxpoint = Some(p);
                        maxnearzero = nearzero;
                    }
                }
            }
            let mut maybe_falsenarrow = false;
            let targetdet = prevdet * self.MAXwidth;
            let mindet = 10.0 * RATIO_MAXSIMPLEX * targetdet;
            if maxdet > 0.0 {
                let ratio = maxdet / targetdet;
                if ratio < RATIO_MAXSIMPLEX {
                    maybe_falsenarrow = true;
                }
            }
            if maxpoint.is_none() || maxnearzero || maybe_falsenarrow {
                for p in 0..self.points.len() as PointId {
                    if !maxpoints.contains(&p) && !simplex.contains(&p) {
                        let (det, nearzero) = self.detsimplex(p, &simplex, i);
                        let det = det.abs();
                        if det > maxdet {
                            maxdet = det;
                            maxpoint = Some(p);
                            maxnearzero = nearzero;
                            if !maxnearzero && maxdet > mindet {
                                break;
                            }
                        }
                    }
                }
            }
            let Some(p) = maxpoint else {
                return Err(Error::qhull(6014, "not enough points available"));
            };
            simplex.push(p);
        }
        Ok(simplex)
    }
}
