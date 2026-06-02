//! The seam-placement solver (§3.5/§3.6 of
//! `docs/CF_CAST_SEAM_PLACEMENT_RECON.md`).
//!
//! [`place_fasteners`] is a **pure, deterministic** function (no RNG, no clock)
//! that places fasteners on a clean [`SeamProfile`] loop. It replaces the
//! geometry-blind uniform bolt/dowel loops + the three ad-hoc pour paths with
//! one constraint-aware solver, run once per fastener class.
//!
//! ## Algorithm (subdivision-first, §3.5)
//! 1. **Seeds** (geometry → required positions, §3.4) resolve to fixed
//!    placements: a *pour pierce* expands into two brackets (the pierce itself is
//!    un-bolt-able — its outward normal is ∥ the bore axis, S0 §7.1 — so step
//!    ALONG the loop to the nearest feasible station each side); an *anchor*
//!    (corner / registration extreme) snaps to the nearest feasible station.
//! 2. The seeds + the boundaries of the infeasible intervals **partition** the
//!    loop. Each feasible arc is subdivided into `ceil(len / max_pitch)` evenly
//!    spaced points — the count *emerges*, no magic `8`.
//! 3. If an even subdivision point lands in a hole the interval scan stepped
//!    over, it is **nudged** to the nearest feasible station in its gap. NOTE:
//!    the recon (§3.5) envisioned a masked farthest-point (Poisson-disk + Lloyd)
//!    fallback for a "genuinely hole-riddled" mask, but the full-`d` feasibility
//!    scan makes that unreachable here — an arc is infeasible only when *no*
//!    offset works, so every physical obstacle (pour bore, dowel boss, band
//!    pinch — all ≫ `arc_step`) splits into its own feasible interval upstream
//!    and even-fill never lands in a hole. The only residual case is a
//!    sub-`arc_step` hole (not physical for a fastener), which the deterministic
//!    local nudge handles in a fraction of the code.
//!
//! ## Variable radial offset (§3.2)
//! A placement is a free 2-D point `C(s, d) = P(s) + d·n̂(s)`: the solver searches
//! both the arc length `s` and the outboard offset `d`. `d` is floored by the
//! inboard washer-vs-cup-wall clearance ([`Feasibility::d_floor`]) and chosen to
//! maximise the clearance margin — which centres the footprint in a free band but
//! pushes it outboard next to an exclusion (reproducing the S0 per-side apex
//! asymmetry, 14.5 vs 11.5 mm).
//!
//! ## Run twice (§3.6)
//! Dowels first (footprint = hole+wall, seeds = axis extremes, `fill = None`),
//! then bolts (footprint = washer R, seeds = pour brackets + corners,
//! `fill = Some(max_pitch)`, with the dowel footprints **and their bosses** added
//! to `excluded`). This module is a pure addition; the build pipeline does not
//! call it yet (wired in S3).

// 2-D seam-plane geometry: f64↔index casts (grid/station counts) are bounded and
// non-negative, and short coordinate names (x, z, a, b, c, d, n) are idiomatic
// for the point/offset math here — same rationale + allow set as `seam_profile`.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::many_single_char_names,
    clippy::similar_names
)]

use crate::seam_profile::{SeamIndex, SeamProfile};
use crate::silhouette_2d::Point2;

/// Default even-contact bolt pitch (30 mm, §3.7).
///
/// For a hand-tightened silicone-mould flange the governing rule is even contact
/// pressure (the silicone head ≈ 1 kPa is negligible), so this is a workmanship
/// constant, not a pressure calculation. S0 (§7.1) measured the as-built 49–56 mm
/// pitch as too sparse; 30 mm → ~14 bolts on the 394 mm `base_mold` seam.
pub const DEFAULT_MAX_PITCH_M: f64 = 0.030;

/// Floor for the scan resolutions (0.1 mm). Guards the `arc_step` / `d_step` the
/// solver divides by: a zero or negative step would otherwise produce an
/// infinite loop count (`f64::INFINITY as usize` saturates to `usize::MAX`).
/// 0.1 mm is also a principled floor — it is finer than the 0.5 mm silhouette
/// grid the loop is built from, so a finer scan resolves nothing real anyway.
const MIN_STEP_M: f64 = 1e-4;

/// The feasibility regime for a fastener footprint (§3.3, incremental band).
///
/// A trial centre `C` of footprint radius `ρ` is feasible iff its disk lies in
/// the flange band `inner ≤ signed_distance ≤ width` (the Lipschitz centre test
/// `inner + ρ ≤ sd(C) ≤ width − ρ`) *and* it clears every [`Exclusion`]. The
/// radial offset `d` is searched in `[d_floor, d_max]`.
#[derive(Debug, Clone, Copy)]
pub struct Feasibility {
    /// Inboard band edge: minimum signed distance of the footprint centre.
    pub inner: f64,
    /// Outboard band edge: maximum signed distance of the footprint centre.
    pub width: f64,
    /// Minimum radial offset — the computed inboard washer-vs-cup-wall-step
    /// clearance floor (§3.2; the surviving half of the old 13 mm rationale).
    pub d_floor: f64,
    /// Maximum radial offset searched (typically the band `width`).
    pub d_max: f64,
    /// Arc-length resolution of the feasibility / seed scans.
    pub arc_step: f64,
    /// Radial-offset resolution of the `d` search.
    pub d_step: f64,
}

impl Feasibility {
    /// An incremental-band regime with the default scan resolutions
    /// (1 mm arc, 0.5 mm radial).
    #[must_use]
    pub const fn band(inner: f64, width: f64, d_floor: f64, d_max: f64) -> Self {
        Self {
            inner,
            width,
            d_floor,
            d_max,
            arc_step: 0.001,
            d_step: 0.0005,
        }
    }

    /// Override the scan resolutions (arc step, radial step).
    #[must_use]
    pub const fn with_steps(mut self, arc_step: f64, d_step: f64) -> Self {
        self.arc_step = arc_step;
        self.d_step = d_step;
        self
    }
}

/// A region of the seam plane that footprints must avoid (§3.4).
///
/// The pour is modelled **once, correctly**: its exclusion is a swept
/// [`Channel`](Self::Channel) (the bore projected onto the seam), not a point
/// disk; a [`Disk`](Self::Disk) excludes an already-placed fastener's footprint
/// or boss (e.g. the dowel footprints + bosses removed from the bolt run, §3.6).
#[derive(Debug, Clone, Copy)]
pub enum Exclusion {
    /// A circular keep-out (a placed footprint or boss).
    Disk {
        /// Centre in the seam plane.
        center: Point2,
        /// Keep-out radius.
        radius: f64,
    },
    /// A capsule keep-out: the segment `a→b` expanded by `half_width`
    /// (the pour bore projected onto the seam).
    Channel {
        /// Segment start.
        a: Point2,
        /// Segment end.
        b: Point2,
        /// Half-width (capsule radius).
        half_width: f64,
    },
}

impl Exclusion {
    /// Signed distance from `p` to the region surface (negative = inside).
    fn clearance(&self, p: Point2) -> f64 {
        match *self {
            Self::Disk { center, radius } => dist(p, center) - radius,
            Self::Channel { a, b, half_width } => point_to_segment(p, a, b) - half_width,
        }
    }
}

/// How a seed (a geometry-derived required position) is resolved (§3.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeedKind {
    /// A single required position (a high-curvature corner or a dowel
    /// registration extreme): snaps to the nearest feasible station to the hint.
    Anchor,
    /// A pour-bore pierce: expands into two brackets, stepping ALONG the loop to
    /// the nearest feasible station on each side (the pierce itself is
    /// un-bolt-able, S0 §7.1).
    PourPierce,
}

/// A geometry-derived required position, given as an arc-length hint (§3.4). The
/// solver resolves the actual feasible station + radial offset.
#[derive(Debug, Clone, Copy)]
pub struct Seed {
    /// Arc-length hint along the loop (wrapped into `[0, perimeter)`).
    pub arc: f64,
    /// How the seed resolves.
    pub kind: SeedKind,
}

/// One fastener class for a single [`place_fasteners`] run (§3.6).
#[derive(Debug, Clone)]
pub struct FastenerClass {
    /// Footprint radius: the washer radius (bolts) or hole+wall (dowels). Drives
    /// feasibility + exclusion clearance (the physical part that must fit).
    pub footprint_radius: f64,
    /// Inter-fastener spacing radius for de-duplication: how far apart two
    /// same-class placements must be. For a demand flange this is the BOSS
    /// radius (`footprint + boss_wall_margin`) so adjacent bosses don't merge
    /// into a fused blob; for plate/none it equals `footprint_radius` (no
    /// boss). Decoupled from `footprint_radius` so spacing is boss-aware while
    /// feasibility/clearance stays washer-based (fasteners keep their radial
    /// band, just space farther along the loop).
    pub separation_radius: f64,
    /// Even-fill pitch: `Some(max_pitch)` subdivides the feasible arcs to that
    /// pitch (bolts); `None` places only the seeds (loose dowels).
    pub fill: Option<f64>,
    /// Required positions (pour brackets + corners for bolts; axis extremes for
    /// dowels). Order is immaterial — the solver sorts.
    pub seeds: Vec<Seed>,
}

/// Why a [`Placement`] is where it is (diagnostics + test assertions).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlacementOrigin {
    /// A resolved seed (pour bracket / corner / registration extreme).
    Seed,
    /// A feasible-interval boundary (placed as close to an obstacle as feasible).
    Boundary,
    /// An even subdivision point.
    Fill,
    /// An even-fill point nudged off a sub-`arc_step` hole (the fallback
    /// repair, §3.5 — see the module docs for why a full Poisson-disk sampler
    /// is unnecessary).
    Fallback,
}

/// A resolved fastener position.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placement {
    /// Arc length along the loop.
    pub arc: f64,
    /// Outboard radial offset `d` from the loop point.
    pub radial: f64,
    /// Seam-plane centre `C(s, d) = P(s) + d·n̂(s)`.
    pub center: Point2,
    /// Why this placement exists.
    pub origin: PlacementOrigin,
}

/// Place one fastener class on `profile` (§3.5/§3.6). Pure + deterministic.
///
/// Returns the placements sorted by arc length. See the module docs for the
/// algorithm; see [`FastenerClass`] / [`Feasibility`] / [`Exclusion`] for the
/// inputs.
#[must_use]
pub fn place_fasteners(
    profile: &SeamProfile,
    feasibility: &Feasibility,
    excluded: &[Exclusion],
    class: &FastenerClass,
) -> Vec<Placement> {
    let solver = Solver {
        profile,
        index: profile.spatial_index(),
        feas: feasibility,
        excluded,
        footprint_r: class.footprint_radius,
        perim: profile.perimeter(),
        arc_step: feasibility.arc_step.max(MIN_STEP_M),
        d_step: feasibility.d_step.max(MIN_STEP_M),
    };
    // Minimum separation between two placements: their footprints (radius
    // `footprint_radius`) overlap when their centres are within a footprint
    // *diameter*, so two placements closer than that cannot both exist — keep
    // the first (lowest arc). This both de-duplicates coincident seeds/boundaries
    // and collapses the two edge bolts of a too-short feasible island into one.
    // (Deliberately NOT tied to the scan resolution `arc_step`.)
    let dedup_tol = (2.0 * class.separation_radius).max(1e-4);

    // 1. resolve seeds → fixed anchors.
    let mut anchors: Vec<Placement> = Vec::new();
    for seed in &class.seeds {
        match seed.kind {
            SeedKind::Anchor => {
                if let Some(r) = solver.snap_anchor(seed.arc) {
                    anchors.push(r.placement(PlacementOrigin::Seed));
                }
            }
            SeedKind::PourPierce => {
                if let Some(r) = solver.step_to_feasible(seed.arc, 1.0) {
                    anchors.push(r.placement(PlacementOrigin::Seed));
                }
                if let Some(r) = solver.step_to_feasible(seed.arc, -1.0) {
                    anchors.push(r.placement(PlacementOrigin::Seed));
                }
            }
        }
    }
    dedup_by_arc(&mut anchors, solver.perim, dedup_tol);

    // 2. dowels (no fill): seeds only. A non-positive / non-finite pitch is
    // misuse (it would also overflow the subdivision count) — treat as no fill.
    let Some(pitch) = class.fill.filter(|p| p.is_finite() && *p > 0.0) else {
        return anchors;
    };

    // 3. bolts: subdivide the feasible intervals around the fixed anchors.
    let anchor_arcs: Vec<f64> = anchors.iter().map(|p| p.arc).collect();
    let mut out = anchors;
    for interval in solver.feasible_intervals() {
        match interval {
            Interval::Closed => {
                let inside = sorted_arcs_in(&anchor_arcs, |_| true);
                if inside.is_empty() {
                    // even ring: n points, n equal gaps ≤ pitch.
                    let n = (solver.perim / pitch).ceil().max(1.0) as usize;
                    for i in 0..n {
                        let a = solver.perim * i as f64 / n as f64;
                        if let Some(r) = solver.resolve(a) {
                            out.push(r.placement(PlacementOrigin::Fill));
                        }
                    }
                } else {
                    let len = inside.len();
                    for w in 0..len {
                        let a = inside[w];
                        let glen = if len == 1 {
                            solver.perim
                        } else {
                            (inside[(w + 1) % len] - a).rem_euclid(solver.perim)
                        };
                        solver.fill_gap(a, glen, pitch, &mut out);
                    }
                }
            }
            Interval::Open { lo, hi } => {
                // The feasible run goes FORWARD from `lo` to `hi`, which can wrap
                // past arc 0 (when the infeasible zone — e.g. the pour — sits away
                // from arc 0, the complementary feasible run is the part that
                // wraps, so `hi < lo` numerically). Work in forward offsets from
                // `lo` in `[0, span]` so the fill covers the *feasible* arc, not
                // the short infeasible gap a numeric `hi - lo` would pick.
                let span = (hi - lo).rem_euclid(solver.perim);
                let mut offs = vec![0.0, span];
                for &a in &anchor_arcs {
                    let o = (a - lo).rem_euclid(solver.perim);
                    if o > dedup_tol && o < span - dedup_tol {
                        offs.push(o);
                    }
                }
                offs.sort_by(f64::total_cmp);
                // place the boundaries themselves (as close to the obstacle as
                // feasible) unless an anchor already sits there.
                for &b in &[lo, hi] {
                    if !anchor_arcs.iter().any(|&a| {
                        let g = (a - b).rem_euclid(solver.perim);
                        g.min(solver.perim - g) <= dedup_tol
                    }) {
                        if let Some(r) = solver.resolve(b) {
                            out.push(r.placement(PlacementOrigin::Boundary));
                        }
                    }
                }
                for w in 0..offs.len().saturating_sub(1) {
                    let a = lo + offs[w];
                    let glen = offs[w + 1] - offs[w];
                    solver.fill_gap(a, glen, pitch, &mut out);
                }
            }
        }
    }

    out.sort_by(|p, q| p.arc.total_cmp(&q.arc));
    dedup_by_arc(&mut out, solver.perim, dedup_tol);
    out
}

/// Snap a master placement's seam-plane `center` onto `profile`, returning the
/// nearest feasible placement (§3.8 per-layer snap).
///
/// The seam-placement solver runs once on the **outermost** layer's loop; each
/// inner layer reuses that shared angular pattern by mapping every master
/// `center` to the arc on this layer's loop nearest it ([`SeamProfile::nearest_arc`]),
/// then re-resolving the best radial offset there (the inner loop sits at a
/// different radius, so `d` is re-fit per layer). If that station is infeasible
/// the search steps outward to the nearest feasible one (same `snap_anchor`
/// rule the seeds use). Returns `None` when no feasible station exists near
/// `center` — the caller then drops the position from the **whole** shared set
/// (a position is kept only if it is feasible on every layer).
#[must_use]
pub fn snap_placement(
    profile: &SeamProfile,
    feasibility: &Feasibility,
    excluded: &[Exclusion],
    footprint_radius: f64,
    center: Point2,
) -> Option<Placement> {
    let solver = Solver {
        profile,
        index: profile.spatial_index(),
        feas: feasibility,
        excluded,
        footprint_r: footprint_radius,
        perim: profile.perimeter(),
        arc_step: feasibility.arc_step.max(MIN_STEP_M),
        d_step: feasibility.d_step.max(MIN_STEP_M),
    };
    solver
        .snap_anchor(profile.nearest_arc(center))
        .map(|r| r.placement(PlacementOrigin::Seed))
}

// --- internals ---

/// A maximal feasible run along the loop.
enum Interval {
    /// The whole loop is feasible.
    Closed,
    /// A feasible arc `[lo, hi]` bounded by infeasible territory.
    Open { lo: f64, hi: f64 },
}

/// A feasible trial centre resolved at one arc length.
#[derive(Debug, Clone, Copy)]
struct Resolved {
    arc: f64,
    radial: f64,
    center: Point2,
    margin: f64,
}

impl Resolved {
    const fn placement(self, origin: PlacementOrigin) -> Placement {
        Placement {
            arc: self.arc,
            radial: self.radial,
            center: self.center,
            origin,
        }
    }
}

struct Solver<'a> {
    profile: &'a SeamProfile,
    index: SeamIndex<'a>,
    feas: &'a Feasibility,
    excluded: &'a [Exclusion],
    footprint_r: f64,
    perim: f64,
    /// `feas.arc_step` / `feas.d_step`, each floored at [`MIN_STEP_M`] so the
    /// loop counts derived from them can never overflow to `usize::MAX`.
    arc_step: f64,
    d_step: f64,
}

impl Solver<'_> {
    /// Resolve the best feasible radial offset at arc length `s`, or `None` if
    /// the whole radial range is infeasible there.
    ///
    /// "Best" = the offset maximising the **clearance margin** — the minimum
    /// distance to every active constraint (both band edges and every
    /// exclusion). This single robust objective yields exactly the two
    /// behaviours the design wants: in a free band the margin peaks at the band
    /// centre `(inner+width)/2` → symmetric flange walls around the hole (the
    /// §B/§M intent); next to an exclusion it slides outboard to clear it. It
    /// reproduces S0's measured per-side apex offsets (≈ 11 mm free, ≈ 14 mm
    /// hugging the bore) from one rule. Ties break to the smallest `d`, so the
    /// scan is deterministic.
    fn resolve(&self, s: f64) -> Option<Resolved> {
        let arc = s.rem_euclid(self.perim);
        let p = self.profile.point_at(arc);
        let n = self.profile.outward_normal_at(arc);
        let lo = self.feas.inner + self.footprint_r;
        let hi = self.feas.width - self.footprint_r;
        let steps = ((self.feas.d_max - self.feas.d_floor) / self.d_step)
            .floor()
            .max(0.0) as i64;
        let mut best: Option<Resolved> = None;
        for k in 0..=steps {
            let d = self.feas.d_floor + k as f64 * self.d_step;
            let c = Point2::new(p.x + n.x * d, p.z + n.z * d);
            let sd = self.index.signed_distance(c);
            if sd < lo || sd > hi {
                continue;
            }
            let mut excl = f64::INFINITY;
            let mut blocked = false;
            for e in self.excluded {
                let clr = e.clearance(c) - self.footprint_r;
                if clr < 0.0 {
                    blocked = true;
                    break;
                }
                excl = excl.min(clr);
            }
            if blocked {
                continue;
            }
            let margin = (sd - lo).min(hi - sd).min(excl);
            // strictly-greater keeps the smaller `d` on a tie.
            if best.is_none_or(|b| margin > b.margin + 1e-12) {
                best = Some(Resolved {
                    arc,
                    radial: d,
                    center: c,
                    margin,
                });
            }
        }
        best
    }

    /// Nearest feasible station to `hint` (the hint itself if feasible), stepping
    /// outward both directions (+ tried first → deterministic tie-break).
    fn snap_anchor(&self, hint: f64) -> Option<Resolved> {
        if let Some(r) = self.resolve(hint) {
            return Some(r);
        }
        let max_k = (self.perim / 2.0 / self.arc_step).floor() as usize;
        for k in 1..=max_k {
            let off = k as f64 * self.arc_step;
            // + tried before − → deterministic tie-break toward increasing arc.
            if let Some(r) = self.resolve(hint + off) {
                return Some(r);
            }
            if let Some(r) = self.resolve(hint - off) {
                return Some(r);
            }
        }
        None
    }

    /// First feasible station stepping from `hint` in `dir` (±1) — the pour
    /// bracket (§3.4). Starts at the pierce (usually infeasible) and walks out.
    fn step_to_feasible(&self, hint: f64, dir: f64) -> Option<Resolved> {
        let max_k = (self.perim / 2.0 / self.arc_step).floor() as usize;
        for k in 0..=max_k {
            let off = k as f64 * self.arc_step;
            if let Some(r) = self.resolve(hint + dir * off) {
                return Some(r);
            }
        }
        None
    }

    /// Maximal feasible runs along the loop (§3.5 partition). `Closed` if every
    /// sampled station is feasible; otherwise the open feasible arcs.
    fn feasible_intervals(&self) -> Vec<Interval> {
        let m = ((self.perim / self.arc_step).ceil() as usize).max(8);
        let arc_of = |k: usize| self.perim * k as f64 / m as f64;
        let flag: Vec<bool> = (0..m).map(|k| self.resolve(arc_of(k)).is_some()).collect();
        let true_count = flag.iter().filter(|b| **b).count();
        if true_count == 0 {
            return Vec::new();
        }
        if true_count == m {
            return vec![Interval::Closed];
        }
        // start at an infeasible sample so feasible runs never wrap the seam.
        let start = flag.iter().position(|b| !*b).unwrap_or(0);
        let mut intervals = Vec::new();
        let mut run_start: Option<usize> = None;
        for off in 0..m {
            let idx = (start + off) % m;
            if flag[idx] {
                run_start.get_or_insert(off);
            } else if let Some(rs) = run_start.take() {
                intervals.push(Interval::Open {
                    lo: arc_of((start + rs) % m),
                    hi: arc_of((start + off - 1) % m),
                });
            }
        }
        if let Some(rs) = run_start {
            intervals.push(Interval::Open {
                lo: arc_of((start + rs) % m),
                hi: arc_of((start + m - 1) % m),
            });
        }
        intervals
    }

    /// Fill the gap of length `glen` starting at arc `a` with even subdivision
    /// points (≤ `pitch` apart). Each point is verified feasible; one that lands
    /// in a sub-`arc_step` hole the interval scan stepped over is nudged to the
    /// nearest feasible station in the gap (§3.5; see module docs for why a full
    /// Poisson-disk sampler is unnecessary). A point with no feasible station in
    /// the whole gap is dropped — which cannot happen for a physical obstacle
    /// (those split into their own feasible intervals upstream).
    fn fill_gap(&self, a: f64, glen: f64, pitch: f64, out: &mut Vec<Placement>) {
        let segs = (glen / pitch).ceil().max(1.0) as usize;
        for i in 1..segs {
            let s = a + glen * i as f64 / segs as f64;
            if let Some(r) = self.resolve(s) {
                out.push(r.placement(PlacementOrigin::Fill));
            } else if let Some(r) = self.nudge_to_feasible(s, a, a + glen, pitch) {
                out.push(r.placement(PlacementOrigin::Fallback));
            }
        }
    }

    /// Nudge an infeasible even-fill point at arc `s` to the nearest feasible
    /// station strictly inside the open gap `(lo, hi)`, searching outward both
    /// directions at a fine step (+ before − → deterministic tie-break). Returns
    /// `None` if the entire gap is infeasible. The step is a small fraction of
    /// `pitch` (independent of `arc_step`), so the nudge lands within a hole's
    /// half-width of `s` — well inside the even-fill slack on any real gap.
    fn nudge_to_feasible(&self, s: f64, lo: f64, hi: f64, pitch: f64) -> Option<Resolved> {
        let step = (pitch / 60.0).max(1e-4);
        let reach = ((hi - lo) / 2.0).max(0.0);
        let max_k = (reach / step).floor() as usize;
        for k in 1..=max_k {
            let off = k as f64 * step;
            if s + off < hi {
                if let Some(r) = self.resolve(s + off) {
                    return Some(r);
                }
            }
            if s - off > lo {
                if let Some(r) = self.resolve(s - off) {
                    return Some(r);
                }
            }
        }
        None
    }
}

/// Drop placements within `tol` arc length of an earlier one (keeps the lower
/// arc). Operates in place on an arc-sorted-or-not list; sorts first.
fn dedup_by_arc(items: &mut Vec<Placement>, perim: f64, tol: f64) {
    items.sort_by(|p, q| p.arc.total_cmp(&q.arc));
    let mut kept: Vec<Placement> = Vec::with_capacity(items.len());
    for p in items.iter() {
        let clash = kept.iter().any(|k| {
            let g = (p.arc - k.arc).abs();
            g.min(perim - g) <= tol
        });
        if !clash {
            kept.push(*p);
        }
    }
    *items = kept;
}

fn sorted_arcs_in(arcs: &[f64], keep: impl Fn(f64) -> bool) -> Vec<f64> {
    let mut v: Vec<f64> = arcs.iter().copied().filter(|a| keep(*a)).collect();
    v.sort_by(f64::total_cmp);
    v
}

fn dist(a: Point2, b: Point2) -> f64 {
    let dx = b.x - a.x;
    let dz = b.z - a.z;
    dx.hypot(dz)
}

/// Distance from `p` to segment `a→b`.
fn point_to_segment(p: Point2, a: Point2, b: Point2) -> f64 {
    let vx = b.x - a.x;
    let vz = b.z - a.z;
    let len2 = vx.mul_add(vx, vz * vz);
    if len2 <= 0.0 {
        return dist(p, a);
    }
    let t = (((p.x - a.x) * vx + (p.z - a.z) * vz) / len2).clamp(0.0, 1.0);
    dist(p, Point2::new(a.x + vx * t, a.z + vz * t))
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::panic,
        clippy::expect_used,
        clippy::float_cmp
    )]

    use super::*;
    use crate::silhouette_2d::SeamPlaneBasis;

    const PITCH: f64 = 0.030;
    const WASHER_R: f64 = 0.0055;

    fn y_basis() -> SeamPlaneBasis {
        SeamPlaneBasis::y_normal(0.0)
    }

    fn circle(r: f64, n: usize) -> Vec<Point2> {
        (0..n)
            .map(|i| {
                let th = std::f64::consts::TAU * (i as f64) / (n as f64);
                Point2::new(r * th.cos(), r * th.sin())
            })
            .collect()
    }

    fn ellipse(a: f64, b: f64, n: usize) -> Vec<Point2> {
        (0..n)
            .map(|i| {
                let th = std::f64::consts::TAU * (i as f64) / (n as f64);
                Point2::new(a * th.cos(), b * th.sin())
            })
            .collect()
    }

    fn profile(raw: &[Point2]) -> SeamProfile {
        SeamProfile::from_polyline(raw, y_basis(), 0.001, 0.002).unwrap()
    }

    /// The default `base_mold`-like band: `[2, 20] mm`, washer footprint,
    /// `d_floor = 2 mm`, searched out to the band edge.
    fn band() -> Feasibility {
        Feasibility::band(0.002, 0.020, 0.002, 0.020)
    }

    fn bolts(seeds: Vec<Seed>) -> FastenerClass {
        FastenerClass {
            footprint_radius: WASHER_R,
            separation_radius: WASHER_R,
            fill: Some(PITCH),
            seeds,
        }
    }

    /// Max wrap-around arc gap between consecutive (arc-sorted) placements.
    fn max_gap(out: &[Placement], perim: f64) -> f64 {
        let mut arcs: Vec<f64> = out.iter().map(|p| p.arc).collect();
        arcs.sort_by(f64::total_cmp);
        let mut max = 0.0_f64;
        for i in 0..arcs.len() {
            let g = (arcs[(i + 1) % arcs.len()] - arcs[i]).rem_euclid(perim);
            max = max.max(g);
        }
        max
    }

    #[test]
    fn circle_even_ring_count_and_spacing() {
        let prof = profile(&circle(0.030, 256));
        let out = place_fasteners(&prof, &band(), &[], &bolts(vec![]));
        // count emerges from max_pitch: ceil(perimeter / pitch).
        let expected = (prof.perimeter() / PITCH).ceil() as usize;
        assert_eq!(out.len(), expected, "count should be ceil(perim/pitch)");
        // evenly spaced in arc length.
        let sp = prof.perimeter() / out.len() as f64;
        for i in 0..out.len() {
            let g = (out[(i + 1) % out.len()].arc - out[i].arc).rem_euclid(prof.perimeter());
            assert!((g - sp).abs() < 0.05 * sp, "uneven gap {g} vs {sp}");
        }
        // every placement is feasible (inside the band, clears nothing-excluded).
        for p in &out {
            let sd = prof.signed_distance(p.center);
            assert!(
                (0.002 + WASHER_R - 1e-9..=0.020 - WASHER_R + 1e-9).contains(&sd),
                "placement out of band: sd={sd}"
            );
        }
    }

    #[test]
    fn ellipse_count_and_even_arc_spacing() {
        let prof = profile(&ellipse(0.045, 0.025, 400));
        let out = place_fasteners(&prof, &band(), &[], &bolts(vec![]));
        let expected = (prof.perimeter() / PITCH).ceil() as usize;
        assert_eq!(out.len(), expected);
        assert!(
            max_gap(&out, prof.perimeter()) <= PITCH + 1e-9,
            "max gap exceeds pitch on the ellipse"
        );
    }

    #[test]
    fn anchor_seed_is_honored() {
        let prof = profile(&circle(0.030, 256));
        let hint = 0.05;
        let out = place_fasteners(
            &prof,
            &band(),
            &[],
            &bolts(vec![Seed {
                arc: hint,
                kind: SeedKind::Anchor,
            }]),
        );
        let seeded: Vec<&Placement> = out
            .iter()
            .filter(|p| p.origin == PlacementOrigin::Seed)
            .collect();
        assert_eq!(seeded.len(), 1, "exactly one anchor seed");
        // a circle is feasible everywhere, so the anchor lands exactly on the hint.
        assert!(
            (seeded[0].arc - hint).abs() < 1e-6,
            "anchor moved off its hint"
        );
        assert!(max_gap(&out, prof.perimeter()) <= PITCH + 1e-9);
    }

    #[test]
    fn pour_pierce_brackets_straddle_and_clear_the_channel() {
        let prof = profile(&circle(0.030, 256));
        // a channel piercing the loop near arc 0 (radial spoke through the ring).
        let pierce_arc = 0.0;
        let p = prof.point_at(pierce_arc);
        let outn = prof.outward_normal_at(pierce_arc);
        let channel = Exclusion::Channel {
            a: Point2::new(p.x - outn.x * 0.005, p.z - outn.z * 0.005),
            b: Point2::new(p.x + outn.x * 0.030, p.z + outn.z * 0.030),
            half_width: 0.006,
        };
        let out = place_fasteners(
            &prof,
            &band(),
            &[channel],
            &bolts(vec![Seed {
                arc: pierce_arc,
                kind: SeedKind::PourPierce,
            }]),
        );
        let seeds: Vec<&Placement> = out
            .iter()
            .filter(|p| p.origin == PlacementOrigin::Seed)
            .collect();
        assert_eq!(seeds.len(), 2, "pour pierce → two brackets");
        // one each side of the pierce, both clearing the channel by the washer.
        let signed: Vec<f64> = seeds
            .iter()
            .map(|s| {
                let g = (s.arc - pierce_arc).rem_euclid(prof.perimeter());
                if g <= prof.perimeter() / 2.0 {
                    g
                } else {
                    g - prof.perimeter()
                }
            })
            .collect();
        assert!(
            signed.iter().any(|&g| g > 0.0) && signed.iter().any(|&g| g < 0.0),
            "brackets must straddle the pierce: {signed:?}"
        );
        for s in &seeds {
            assert!(
                channel.clearance(s.center) >= WASHER_R - 1e-9,
                "bracket washer fouls the channel"
            );
        }
        // and no placement at all intrudes on the channel.
        for p in &out {
            assert!(channel.clearance(p.center) >= WASHER_R - 1e-9);
        }
    }

    /// Regression: when the infeasible zone (the pour channel) sits AWAY from
    /// arc 0, the complementary feasible run wraps past arc 0, so the
    /// `Interval::Open { lo, hi }` it produces has `lo > hi` numerically. The fill
    /// must still cover the whole feasible arc (forward/modular), not collapse to
    /// the short infeasible gap — which previously dropped the entire ring to just
    /// the two pour brackets.
    #[test]
    fn pour_pierce_away_from_arc_zero_still_fills_the_ring() {
        let prof = profile(&circle(0.030, 256));
        let perim = prof.perimeter();
        // pierce at ~0.3 of the loop — its infeasible zone does NOT straddle arc 0.
        let pierce_arc = perim * 0.3;
        let p = prof.point_at(pierce_arc);
        let outn = prof.outward_normal_at(pierce_arc);
        let channel = Exclusion::Channel {
            a: Point2::new(p.x - outn.x * 0.005, p.z - outn.z * 0.005),
            b: Point2::new(p.x + outn.x * 0.030, p.z + outn.z * 0.030),
            half_width: 0.006,
        };
        let out = place_fasteners(
            &prof,
            &band(),
            &[channel],
            &bolts(vec![Seed {
                arc: pierce_arc,
                kind: SeedKind::PourPierce,
            }]),
        );
        // the ring should be (nearly) fully populated, not just the 2 brackets.
        let expected = (perim / PITCH).ceil() as usize;
        assert!(
            out.len() >= expected - 2,
            "wrapping feasible interval must still fill the ring: got {} vs ~{expected}",
            out.len()
        );
        // coverage gap never exceeds the pitch by much (one small obstacle aside).
        assert!(
            max_gap(&out, perim) <= PITCH + 0.012,
            "coverage gap too large: {} mm",
            max_gap(&out, perim) * 1000.0
        );
        // and nothing intrudes on the channel.
        for pl in &out {
            assert!(channel.clearance(pl.center) >= WASHER_R - 1e-9);
        }
    }

    #[test]
    fn disk_exclusion_is_respected_and_leaves_a_gap() {
        let prof = profile(&circle(0.030, 256));
        // a disk sitting on the offset ring near arc 0.
        let p = prof.point_at(0.0);
        let outn = prof.outward_normal_at(0.0);
        let disk_c = Point2::new(p.x + outn.x * 0.011, p.z + outn.z * 0.011);
        let disk = Exclusion::Disk {
            center: disk_c,
            radius: 0.008,
        };
        let out = place_fasteners(&prof, &band(), &[disk], &bolts(vec![]));
        assert!(!out.is_empty());
        for pl in &out {
            assert!(
                dist(pl.center, disk_c) >= 0.008 + WASHER_R - 1e-9,
                "placement footprint overlaps the disk exclusion"
            );
        }
        // the obstacle forces an open interval → boundary placements appear.
        assert!(
            out.iter().any(|p| p.origin == PlacementOrigin::Boundary),
            "expected boundary placements bracketing the hole"
        );
    }

    #[test]
    fn holed_mask_nudges_fill_point_off_a_subscan_hole() {
        // A thin RADIAL channel blocks every `d` at its arc (you can't dodge it
        // by changing the offset, unlike a small disk). Made finer than a
        // deliberately-coarse interval scan, it slips between the feasibility
        // samples yet lands on an even-subdivision point — the sub-`arc_step`
        // hole that engages the nudge repair (§3.5). (A physical, ≫`arc_step`
        // obstacle would split into its own interval and never reach here.)
        let prof = profile(&circle(0.030, 256));
        // 50 mm scan → only ~4 samples around the ring, blind to a ~4 mm hole.
        let coarse = band().with_steps(0.050, 0.0005);
        let footprint = 0.001;
        let class = FastenerClass {
            footprint_radius: footprint,
            separation_radius: footprint,
            fill: Some(PITCH),
            // one anchor → a single wrap gap subdivided across the whole ring.
            seeds: vec![Seed {
                arc: 0.0,
                kind: SeedKind::Anchor,
            }],
        };
        // put the channel exactly on an even-subdivision point (3/segs around).
        let segs = (prof.perimeter() / PITCH).ceil() as usize;
        let hole_arc = prof.perimeter() * 3.0 / segs as f64;
        let p = prof.point_at(hole_arc);
        let outn = prof.outward_normal_at(hole_arc);
        let channel = Exclusion::Channel {
            a: Point2::new(p.x - outn.x * 0.005, p.z - outn.z * 0.005),
            b: Point2::new(p.x + outn.x * 0.030, p.z + outn.z * 0.030),
            half_width: 0.001,
        };
        let out = place_fasteners(&prof, &coarse, &[channel], &class);
        assert!(
            out.iter().any(|p| p.origin == PlacementOrigin::Fallback),
            "thin hole on an even point should engage the nudge repair"
        );
        for pl in &out {
            assert!(
                channel.clearance(pl.center) >= footprint - 1e-9,
                "a placement landed inside the hole"
            );
        }
        // the nudge moves the point only within the even-fill slack, so coverage
        // stays at ≤ pitch.
        assert!(max_gap(&out, prof.perimeter()) <= PITCH + 1e-9);
    }

    #[test]
    fn dowels_are_seeds_only() {
        let prof = profile(&ellipse(0.045, 0.025, 400));
        // two registration extremes on the long axis (arc 0 and the far side).
        let dowels = FastenerClass {
            footprint_radius: 0.004,
            separation_radius: 0.004,
            fill: None,
            seeds: vec![
                Seed {
                    arc: 0.0,
                    kind: SeedKind::Anchor,
                },
                Seed {
                    arc: prof.perimeter() / 2.0,
                    kind: SeedKind::Anchor,
                },
            ],
        };
        let out = place_fasteners(&prof, &band(), &[], &dowels);
        assert_eq!(out.len(), 2, "fill=None → only the seeds");
        assert!(out.iter().all(|p| p.origin == PlacementOrigin::Seed));
    }

    #[test]
    fn deterministic_and_seed_order_independent() {
        let prof = profile(&circle(0.030, 256));
        let p = prof.point_at(0.0);
        let outn = prof.outward_normal_at(0.0);
        let channel = Exclusion::Channel {
            a: Point2::new(p.x - outn.x * 0.005, p.z - outn.z * 0.005),
            b: Point2::new(p.x + outn.x * 0.030, p.z + outn.z * 0.030),
            half_width: 0.006,
        };
        let seeds_a = vec![
            Seed {
                arc: 0.0,
                kind: SeedKind::PourPierce,
            },
            Seed {
                arc: 0.10,
                kind: SeedKind::Anchor,
            },
        ];
        let mut seeds_b = seeds_a.clone();
        seeds_b.reverse();
        let run = |seeds| place_fasteners(&prof, &band(), &[channel], &bolts(seeds));
        let r1 = run(seeds_a.clone());
        let r2 = run(seeds_a);
        let r3 = run(seeds_b);
        assert_eq!(r1, r2, "same input must give the same output");
        assert_eq!(r1, r3, "seed order must not change the output");
    }

    /// §3.8 per-layer snap: a master placement solved on the OUTER loop maps to
    /// the angularly-corresponding arc on an INNER loop, with the radial offset
    /// re-fit to the inner loop's band (the inner loop sits at a smaller radius).
    #[test]
    fn snap_placement_maps_to_inner_loop_at_same_angle() {
        let outer = profile(&circle(0.030, 256));
        let inner = profile(&circle(0.020, 256));
        let master = place_fasteners(&outer, &band(), &[], &bolts(vec![]));
        // pick the master placement nearest arc 0 (a point on the +x axis).
        let m = master
            .iter()
            .min_by(|a, b| {
                let ga = a.arc.min(outer.perimeter() - a.arc);
                let gb = b.arc.min(outer.perimeter() - b.arc);
                ga.total_cmp(&gb)
            })
            .unwrap();
        // its center sits outboard of the outer loop on ~the +x axis.
        assert!(m.center.x > 0.020 && m.center.z.abs() < 0.004);
        let snapped = snap_placement(&inner, &band(), &[], WASHER_R, m.center).unwrap();
        // lands at the same angle (still on the +x side, near z = 0) but in the
        // INNER loop's flange band (center x in [inner_radius + band]).
        assert!(snapped.center.x > 0.020, "snapped inboard of inner body");
        assert!(snapped.center.z.abs() < 0.004, "snapped off the +x angle");
        let sd = inner.signed_distance(snapped.center);
        assert!(
            (0.002 + WASHER_R - 1e-9..=0.020 - WASHER_R + 1e-9).contains(&sd),
            "snapped placement out of the inner band: sd={sd}"
        );
    }

    /// `snap_placement` returns `None` when no feasible station exists near the
    /// hint (here: a band whose offset range can never be reached) — the caller
    /// drops the shared position.
    #[test]
    fn snap_placement_returns_none_when_band_unreachable() {
        let prof = profile(&circle(0.020, 256));
        // band requires sd ≥ 50 mm but the radial search tops out at 6 mm.
        let unreachable = Feasibility::band(0.050, 0.060, 0.002, 0.006);
        let hint = Point2::new(0.026, 0.0);
        assert!(snap_placement(&prof, &unreachable, &[], WASHER_R, hint).is_none());
    }

    /// Degenerate inputs (non-positive pitch / scan steps) must not hang — the
    /// loop counts derive from `perim / step`, which would be `usize::MAX` if a
    /// step were zero. A non-positive pitch falls back to seeds-only; a zero
    /// step is floored. (No assertion on count — only that it terminates.)
    #[test]
    fn degenerate_steps_and_pitch_terminate() {
        let prof = profile(&circle(0.030, 256));
        // zero pitch → seeds only, no subdivision blow-up.
        let zero_pitch = FastenerClass {
            footprint_radius: WASHER_R,
            separation_radius: WASHER_R,
            fill: Some(0.0),
            seeds: vec![Seed {
                arc: 0.05,
                kind: SeedKind::Anchor,
            }],
        };
        assert_eq!(place_fasteners(&prof, &band(), &[], &zero_pitch).len(), 1);
        // zero scan steps → floored, still produces a sane ring.
        let zero_steps = band().with_steps(0.0, 0.0);
        let out = place_fasteners(&prof, &zero_steps, &[], &bolts(vec![]));
        assert!(!out.is_empty());
    }
}
