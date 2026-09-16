//! What a member carries, and whether its section can take it.
//!
//! Every other check in this crate reads *geometry*: where parts are, whether
//! they interpenetrate, how many joints hold them. None of them reads a
//! **load**, and an assembly can be geometrically perfect and structurally
//! absurd — every part in the right place, every clearance met, and a member
//! an order of magnitude past yield.
//!
//! ## Why this belongs in a crate that knows nothing
//!
//! *"No member may be stressed past what its material allows"* is true of a
//! boat, a wing spar, a robot's forearm and a chassis rail. It names no domain.
//! What it needs that a [`Mechanism`] does not carry — the masses, which way is
//! down, how many g, what each material allows — **the caller supplies**, in
//! keeping with the rule at the crate root.
//!
//! ## The model, stated plainly
//!
//! Each part is treated as a **cantilever from its parent joint**, carrying the
//! weight of everything hanging below it in the joint tree:
//!
//! ```text
//! supported = the part and every descendant through joints
//! load      = supported mass x g x g_factor
//! lever     = distance from the part's anchor to the supported centre of
//!             mass, measured perpendicular to `down`
//! moment    = load x lever
//! Z         = elastic section modulus about the WEAK axis, sampled from the
//!             part's own solid
//! stress    = moment / Z
//! ```
//!
//! ## ⛔ What this cannot see — read before trusting a green result
//!
//! - **A part with nothing holding it is not scored.** Never any joint's child
//!   at all, or a child of a FREE joint — which is a joint in the tree but not
//!   a support, so there is no reaction to cantilever against. The second case
//!   used to be scored from the part's own body origin, which on a vehicle
//!   sits mid-structure; the chassis rail read 0.69x and then 1.17x when the
//!   wheels gained mass, and neither number meant anything. Both cases now
//!   report as [`Unmeasured::NotSupported`] rather than as a figure.
//! - **A member held at both ends is over-estimated.** A tie rod or a braced
//!   diagonal is not a cantilever, and this reports it as one.
//! - ⛔⛔ **THE TREE DECIDES THE LOAD PATH, AND A SYMMETRIC PAIR SPLITS
//!   WILDLY.** Of two seat rails carrying one driver between them, the tree
//!   gives the load to whichever one happens to be the parent: measured on the
//!   trike, `seat_back_rail_left` read 5.3x yield and `seat_back_rail_right`
//!   read **zero**. The pair together carries it, so the honest figure is
//!   nearer half. ⇒ **Read a flagged member as a CANDIDATE and check how many
//!   siblings really share its load** — the number is an upper bound on one
//!   side and a floor of zero on the other.
//! - ⚠ **A leaf member reads zero.** Its mass sits at its own origin, so the
//!   lever is nil and no moment appears — where a real distributed load on a
//!   cantilever gives `wL²/2`. Self-weight bending is not modelled.
//! - **Linkages are not load paths here.** The subtree walk follows joints
//!   only, so a part braced by a linkage reads worse than it is.
//! - **Bending only.** No torsion, no buckling, no fatigue, no stress
//!   concentration at a weld or a bore — and a thin tube fails by buckling
//!   long before it reaches yield in bending.
//! - **The section is taken at the centroid**, so a member thin at its root and
//!   fat in the middle reads stronger than it is.
//!
//! ⚠ It is a **screen**, not an analysis: it is built to catch the member that
//! is wrong by a factor, not to certify the one that is right by 20%.

use std::collections::{HashMap, HashSet, VecDeque};

use cf_design::mechanism::Mechanism;
use nalgebra::{Matrix2, Matrix3, Point3, Vector3};

use crate::Origins;

/// Standard gravity, mm/s², matching the mm-kg-s unit system a [`Mechanism`]
/// is built in.
const GRAVITY_MM_S2: f64 = 9810.0;

/// A part's mass and where that mass sits.
///
/// ⚠ **The caller integrates, not this crate.** Working out a part's mass from
/// an SDF means picking a grid cell fine enough for its thinnest feature and
/// checking the integration converged — a judgement about the part, which is
/// the caller's to make. Anything that has already done it has these numbers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassPoint {
    /// Kilograms.
    pub kg: f64,
    /// Centre of mass, in the same frame as [`Origins`].
    pub world_com_mm: Vector3<f64>,
}

/// Mass and centre of mass for every part, by name.
pub type MassMap = HashMap<String, MassPoint>;

/// The loading to check against, and what each material will take.
#[derive(Debug, Clone)]
pub struct LoadCase {
    /// Multiple of gravity applied to every supported mass. `1.0` is static;
    /// `3.0` is a common kerb-strike or hard-landing screen.
    pub g_factor: f64,
    /// Which way the load acts. Need not be normalised.
    pub down: Vector3<f64>,
    /// Allowable stress in `MPa`, keyed by the material's name. A material absent
    /// from this map is **not checked** and is reported with an allowable of
    /// `None`, because inventing a number for it would be worse than saying so.
    pub allowable_mpa: HashMap<String, f64>,
    /// Grid step for sampling a part's cross-section, in millimetres.
    ///
    /// ⚠ **Not monotone.** A tube section lands within ~1.5% at 0.5 mm and
    /// ~0.5% at 0.25 mm, but the error does not shrink smoothly: the extreme
    /// fibre distance is a *maximum over samples*, so it is sensitive to how
    /// the grid happens to align with a curved boundary. Fine for catching a
    /// member that is wrong by a factor; do not read the third digit.
    pub section_cell_mm: f64,
}

impl LoadCase {
    /// A static 1 g case pulling along −z, with no allowables declared.
    #[must_use]
    pub fn static_1g() -> Self {
        Self {
            g_factor: 1.0,
            down: -Vector3::z(),
            allowable_mpa: HashMap::new(),
            section_cell_mm: 0.5,
        }
    }

    /// Set the g multiplier.
    #[must_use]
    pub const fn at_g(mut self, g_factor: f64) -> Self {
        self.g_factor = g_factor;
        self
    }

    /// Declare what a material allows, in `MPa`.
    #[must_use]
    pub fn allowing(mut self, material: impl Into<String>, mpa: f64) -> Self {
        self.allowable_mpa.insert(material.into(), mpa);
        self
    }
}

/// Why a member could not be measured.
///
/// ⚠ Four of these five are the CALLER's input being incomplete rather than the
/// geometry defeating the sampler. They are kept apart because the remedy
/// differs: one is "your map is missing an entry", the other is "this shape
/// cannot be read at this cell".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Unmeasured {
    /// Its section could not be sampled — a bent member whose centroid misses
    /// its own material, one with no bounds, one that samples to nothing.
    Section,
    /// [`Origins`] has no entry for it, so there is nothing to cantilever from.
    NoOrigin,
    /// Its subtree carries no positive mass in the [`MassMap`].
    NoMass,
    /// Some part of its subtree is missing from the [`MassMap`], so the load it
    /// carries is understated by an unknown amount.
    ///
    /// ⛔⛔ **This is the dangerous one.** The member is perfectly measurable
    /// and its number would look entirely ordinary — it is merely too small, by
    /// however much the absent children weigh.
    IncompleteSubtree,
    /// Its parent joint holds it in no direction — a free joint is not a
    /// support, so there is nothing to cantilever from.
    ///
    /// ⛔⛔ **Not scored rather than scored badly.** A root anchored to the
    /// world by a free joint used to be measured from its own body origin,
    /// which on a vehicle sits mid-structure: the trike's chassis rail read
    /// 0.69x, then 1.17x when the wheels got heavier, and neither figure meant
    /// anything. A number that looks like an answer is worse than no number,
    /// and it will eventually cross a threshold and be argued about.
    NotSupported,
    /// A mass, a centre of mass, an anchor or a load factor was not finite.
    ///
    /// ⚠ Withheld rather than reported, because `NaN > 1.0` is `false`: a `NaN`
    /// utilisation reads as "not over yield" at every call site that asks.
    NotFinite,
}

impl Unmeasured {
    /// Whether nothing the caller could supply would make this member
    /// measurable.
    ///
    /// ★ The distinction matters at the call site. [`Self::NotSupported`] is
    /// structural — a body on a free joint has no reaction to cantilever
    /// against, and no finer probe or fuller mass map changes that, so a
    /// consumer should pass over it. Every other reason is the caller's input
    /// or the caller's probe, and a consumer should treat those as a FAILURE
    /// of the screen rather than a pass for the parts.
    #[must_use]
    pub const fn is_structural(self) -> bool {
        matches!(self, Self::NotSupported)
    }
}

/// A member the screen could not measure, and why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Unmeasurable {
    /// The part.
    pub part: String,
    /// What stopped it.
    pub why: Unmeasured,
}

/// What the screen measured, and what it could not.
///
/// ⛔⛔ **The second field is the point.** A screen that quietly drops what it
/// cannot read always looks clean — it reports "27 members, 2 past yield" while
/// seven went unmeasured, and the reader has no way to know. Unmeasured is
/// reported here, never dropped.
///
/// ⚠ That sentence was once FALSE in this very function. Three of its four
/// early exits pushed nothing, so a member absent from [`Origins`] or
/// [`MassMap`] vanished from both fields while the doc above promised it could
/// not. Every exit now names the member.
#[derive(Debug, Clone, PartialEq)]
pub struct Screen {
    /// One entry per member whose section could be sampled, worst first.
    pub members: Vec<MemberLoad>,
    /// Every member that hangs from a joint and was **not measured**.
    ///
    /// ⛔ **Unmeasured is not sound.** Treat a non-empty list as a failure of
    /// the screen, not as a pass for the parts in it.
    pub unmeasured: Vec<Unmeasurable>,
}

/// What one member carries and what that does to it.
#[derive(Debug, Clone, PartialEq)]
pub struct MemberLoad {
    /// The part.
    pub part: String,
    /// Its material's name, as declared on the part.
    pub material: String,
    /// Mass of this part and everything hanging below it, kilograms.
    pub supported_kg: f64,
    /// Distance from the anchor to the supported centre of mass, measured
    /// perpendicular to `down`.
    pub lever_mm: f64,
    /// `supported_kg × g × g_factor`, newtons.
    pub load_n: f64,
    /// Elastic section modulus about the weak axis, mm³.
    pub section_modulus_mm3: f64,
    /// `moment / Z`, `MPa`.
    pub stress_mpa: f64,
    /// What the material allows, if the caller declared it.
    pub allowable_mpa: Option<f64>,
    /// `stress / allowable`. `None` when the material was not declared.
    ///
    /// ★ **1.0 is the edge, not the target.** This carries no safety factor:
    /// the caller sets one by declaring an allowable below yield.
    pub utilisation: Option<f64>,
}

/// Screen every member that hangs from a joint.
///
/// Returns a [`Screen`]: one entry per measurable member, **worst utilisation
/// first** with unscored materials after the scored ones, plus the names of
/// every member that could not be measured at all.
///
/// ⛔ Read [`Screen::unmeasured`] before reading the members. It is the list the
/// header count does not include.
///
/// See the module docs for what this cannot see.
#[must_use]
pub fn member_loads(
    mechanism: &Mechanism,
    origins: &Origins,
    masses: &MassMap,
    case: &LoadCase,
) -> Screen {
    let mut unmeasured: Vec<Unmeasurable> = Vec::new();
    let Some(down) = case.down.try_normalize(1e-12) else {
        return Screen {
            members: Vec::new(),
            unmeasured,
        };
    };
    let children = child_index(mechanism);
    let parent_joint: HashMap<&str, &cf_design::JointDef> =
        mechanism.joints().iter().map(|j| (j.child(), j)).collect();

    let mut out = Vec::new();
    for part in mechanism.parts() {
        let name = part.name();
        let Some(joint) = parent_joint.get(name) else {
            continue; // never any joint's child: nothing to cantilever from.
        };
        let mut withhold = |why| {
            unmeasured.push(Unmeasurable {
                part: name.to_owned(),
                why,
            });
        };
        if joint.kind().dof() >= 6 {
            withhold(Unmeasured::NotSupported);
            continue;
        }
        let Some(&anchor) = origins.get(name) else {
            withhold(Unmeasured::NoOrigin);
            continue;
        };
        let supported = subtree(&children, name);
        let Some((supported_kg, com)) = combined_mass(&supported, masses) else {
            withhold(Unmeasured::IncompleteSubtree);
            continue;
        };
        if supported_kg <= 0.0 {
            withhold(Unmeasured::NoMass);
            continue;
        }
        if !supported_kg.is_finite()
            || !com.iter().all(|c| c.is_finite())
            || !anchor.iter().all(|c| c.is_finite())
            || !case.g_factor.is_finite()
        {
            withhold(Unmeasured::NotFinite);
            continue;
        }
        let Some((modulus, _)) =
            section_modulus(part.solid(), case.section_cell_mm).filter(|&(z, _)| z > 0.0)
        else {
            withhold(Unmeasured::Section);
            continue;
        };

        // Only the component across the load direction bends the member. A
        // mass hanging straight below its anchor puts the member in
        // compression, not bending, and this correctly reports no moment.
        let offset = com - anchor;
        let lever_mm = (offset - down * offset.dot(&down)).norm();
        let load_n = supported_kg * (GRAVITY_MM_S2 / 1000.0) * case.g_factor;
        let stress_mpa = load_n * lever_mm / modulus;

        let material = part.material().name.clone();
        let allowable_mpa = case.allowable_mpa.get(&material).copied();
        out.push(MemberLoad {
            part: name.to_string(),
            material,
            supported_kg,
            lever_mm,
            load_n,
            section_modulus_mm3: modulus,
            stress_mpa,
            allowable_mpa,
            utilisation: allowable_mpa.map(|a| stress_mpa / a),
        });
    }

    out.sort_by(|a, b| {
        b.utilisation
            .unwrap_or(f64::NEG_INFINITY)
            .total_cmp(&a.utilisation.unwrap_or(f64::NEG_INFINITY))
            .then_with(|| b.stress_mpa.total_cmp(&a.stress_mpa))
    });
    unmeasured.sort_by(|lhs, rhs| lhs.part.cmp(&rhs.part));
    Screen {
        members: out,
        unmeasured,
    }
}

/// Parent → children, through joints only.
fn child_index(mechanism: &Mechanism) -> HashMap<&str, Vec<&str>> {
    let mut out: HashMap<&str, Vec<&str>> = HashMap::new();
    for j in mechanism.joints() {
        out.entry(j.parent()).or_default().push(j.child());
    }
    out
}

/// A part and everything below it in the joint tree.
fn subtree<'a>(children: &HashMap<&'a str, Vec<&'a str>>, root: &'a str) -> Vec<&'a str> {
    let mut seen: HashSet<&str> = HashSet::from([root]);
    let mut queue: VecDeque<&str> = VecDeque::from([root]);
    let mut out = vec![root];
    while let Some(node) = queue.pop_front() {
        for next in children.get(node).map_or(&[][..], Vec::as_slice) {
            if seen.insert(next) {
                out.push(next);
                queue.push_back(next);
            }
        }
    }
    out
}

/// Total mass of a subtree and where it acts, or `None` when the map does not
/// cover all of it.
///
/// ⛔⛔ **Missing is not zero.** Summing what happens to be present and carrying
/// on is how a parent comes to be scored against a fraction of the load it
/// really holds — a number that is silently small and looks entirely ordinary.
/// If any member of the subtree is absent the total is not knowable, and the
/// caller is told so rather than handed the part of it that was.
fn combined_mass(parts: &[&str], masses: &MassMap) -> Option<(f64, Vector3<f64>)> {
    let mut kg = 0.0;
    let mut moment = Vector3::zeros();
    for name in parts {
        let m = masses.get(*name)?;
        kg += m.kg;
        moment += m.world_com_mm * m.kg;
    }
    if kg <= 0.0 {
        return Some((0.0, Vector3::zeros()));
    }
    Some((kg, moment / kg))
}

/// Elastic section modulus about the weak axis, and the axis it was taken
/// normal to.
///
/// The member's own long axis is found by principal-component analysis of its
/// sampled interior, the section is taken at the sampled centroid, and the
/// 2×2 area-inertia tensor in that plane is diagonalised so the WEAK axis is
/// found rather than whichever one happened to line up with the sampling basis.
fn section_modulus(solid: &cf_design::Solid, cell_mm: f64) -> Option<(f64, Vector3<f64>)> {
    if !cell_mm.is_finite() || cell_mm <= 0.0 {
        return None;
    }
    let (centroid, axis, coarse) = long_axis_and_centroid(solid, cell_mm)?;

    // An orthonormal basis across the section.
    let seed = if axis.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let across_u = axis.cross(&seed).normalize();
    let across_v = axis.cross(&across_u).normalize();

    // Sample the plane through the centroid, normal to the long axis.
    //
    // ⚠ **The half-extent comes from the coarse points, not from the body.**
    // Using the body diagonal is "safe" and ruinous: a 2300 mm rail would have
    // its section sampled over a 2300 x 2300 mm plane at half a millimetre —
    // 21 million evaluations to read a 31.75 mm tube. The section can be no
    // wider than the material already found lying off the axis, plus a margin
    // for what the coarse grid stepped over.
    let reach = coarse.reach;
    let area_cell = cell_mm * cell_mm;
    let mut sum = (0.0_f64, 0.0_f64);
    let mut cell_pts: Vec<(f64, f64)> = Vec::new();
    let side = steps_across(2.0 * reach, cell_mm);
    for i in 0..=side {
        let a = as_f64(i).mul_add(cell_mm, -reach);
        for j in 0..=side {
            let b = as_f64(j).mul_add(cell_mm, -reach);
            let p = centroid + across_u * a + across_v * b;
            if solid.evaluate(&Point3::from(p)) < 0.0 {
                cell_pts.push((a, b));
                sum.0 += a;
                sum.1 += b;
            }
        }
    }
    if cell_pts.len() < 4 {
        return None;
    }

    let n = as_f64(cell_pts.len());
    let (ca, cb) = (sum.0 / n, sum.1 / n);
    let (mut about_a, mut about_b, mut product) = (0.0, 0.0, 0.0);
    for &(a, b) in &cell_pts {
        let (da, db) = (a - ca, b - cb);
        about_a += db * db * area_cell; // bending about the `a` axis
        about_b += da * da * area_cell;
        product -= da * db * area_cell;
    }
    let tensor = Matrix2::new(about_a, product, product, about_b);
    let eig2 = tensor.symmetric_eigen();
    let (weak, strong) = if eig2.eigenvalues[0] <= eig2.eigenvalues[1] {
        (0, 1)
    } else {
        (1, 0)
    };
    let i_min = eig2.eigenvalues[weak];
    // The extreme fibre is measured across the NEUTRAL axis of the weak
    // bending mode, which is the strong principal direction.
    //
    // ⚠⚠ **Plus half a cell, and that half cell is not a fudge.** A sample
    // stands for the CELL around it, so the outermost sample's material
    // reaches half a cell further out than its centre. Measuring to the centre
    // biases `Z` high by exactly `(n + 1) / n` on a rectangle — one-sided, and
    // high means a member reads LESS stressed than it is, which is the wrong
    // direction for a screen. Carrying the half cell turns that into a
    // `1 - 1/n²` shortfall: an order smaller, and conservative.
    let neutral = eig2.eigenvectors.column(strong);
    let c = cell_pts
        .iter()
        .map(|&(a, b)| (a - ca).mul_add(neutral[0], (b - cb) * neutral[1]).abs())
        .fold(0.0_f64, f64::max)
        + cell_mm / 2.0;
    if c <= 0.0 {
        return None;
    }
    Some((i_min / c, axis))
}

/// What the coarse interior pass found: where the section sits and how far it
/// reaches off the axis.
struct Coarse {
    reach: f64,
}

/// Find the member's own long axis and centroid by sampling its interior.
///
/// ⛔⛔ **The step is set by the member's SMALLEST span, not its largest.**
/// Scaling it to the longest dimension is the obvious thing and it is wrong: a
/// 2300 mm chassis rail of 31.75 mm tube then gets probed every 96 mm, lands
/// inside a 2 mm wall essentially never, and the whole member is dropped for
/// want of eight interior points. The step has to resolve the SECTION, and the
/// section lives on the short axes.
///
/// ⚠ Adaptive, because even min-span/12 misses a thin enough wall. It halves
/// until it has points or reaches the caller's cell, which bounds the work: a
/// member that truly cannot be sampled costs four extra passes, not an
/// unbounded search.
fn long_axis_and_centroid(
    solid: &cf_design::Solid,
    cell_mm: f64,
) -> Option<(Vector3<f64>, Vector3<f64>, Coarse)> {
    let bounds = solid.bounds()?;
    let (lo, hi) = (bounds.min, bounds.max);
    let span = hi - lo;
    let min_span = span.x.min(span.y).min(span.z);
    let mut coarse = (min_span / 12.0).max(cell_mm);
    let mut pts: Vec<Vector3<f64>> = Vec::new();
    for _ in 0..5 {
        pts.clear();
        let (nx, ny, nz) = (
            steps_across(span.x, coarse),
            steps_across(span.y, coarse),
            steps_across(span.z, coarse),
        );
        for iz in 0..nz {
            let z = (as_f64(iz) + 0.5).mul_add(coarse, lo.z);
            for iy in 0..ny {
                let y = (as_f64(iy) + 0.5).mul_add(coarse, lo.y);
                for ix in 0..nx {
                    let x = (as_f64(ix) + 0.5).mul_add(coarse, lo.x);
                    let p = Vector3::new(x, y, z);
                    if solid.evaluate(&Point3::from(p)) < 0.0 {
                        pts.push(p);
                    }
                }
            }
        }
        if pts.len() >= 64 || coarse <= cell_mm {
            break;
        }
        coarse = (coarse / 2.0).max(cell_mm);
    }
    if pts.len() < 8 {
        return None;
    }

    let count = as_f64(pts.len());
    let centroid = pts.iter().sum::<Vector3<f64>>() / count;
    let mut cov = Matrix3::zeros();
    for p in &pts {
        let d = p - centroid;
        cov += d * d.transpose();
    }
    let eig = cov.symmetric_eigen();
    let long = (0..3).max_by(|&lhs, &rhs| eig.eigenvalues[lhs].total_cmp(&eig.eigenvalues[rhs]))?;
    let axis = Vector3::from(eig.eigenvectors.column(long)).normalize();

    let reach = pts
        .iter()
        .map(|p| {
            let d = p - centroid;
            (d - axis * d.dot(&axis)).norm()
        })
        .fold(0.0_f64, f64::max)
        + coarse * 2.0;
    Some((centroid, axis, Coarse { reach }))
}

/// A count as a float.
///
/// Every `usize` reaching this is a tally of sample points, and `usize -> f64`
/// is exact below 2^53. A grid that reached nine quadrillion points would have
/// exhausted memory many orders before it lost a bit of mantissa — so the cast
/// is lossless in every reachable case, and this is the one place that says so
/// rather than eight `#[allow]`s that do not.
// The reachability argument above is the justification; see the doc comment.
#[allow(clippy::cast_precision_loss)]
const fn as_f64(n: usize) -> f64 {
    n as f64
}

/// How many whole steps of `step` span `extent`, at least one.
///
/// ⚠ An integer count, deliberately. Walking a float cursor with `while x < hi`
/// accumulates rounding across thousands of steps and makes the sample grid
/// depend on where the body happens to sit.
fn steps_across(extent: f64, step: f64) -> usize {
    if !extent.is_finite() || !step.is_finite() || step <= 0.0 || extent <= 0.0 {
        return 1;
    }
    // The guard above rules out the negative and non-finite quotients; what is
    // left is a positive finite number whose `ceil()` is a whole value, and a
    // step count that overflowed usize would have exhausted memory first.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n = (extent / step).ceil() as usize;
    n.max(1)
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use cf_design::{JointDef, JointKind, Material, Part, Solid};

    use super::*;

    /// A z-aligned tube, long enough that its own axis is the long one.
    fn tube(name: &str, od: f64, wall: f64, material: &str) -> Part {
        Part::new(
            name,
            Solid::cylinder(od / 2.0, 100.0).subtract(Solid::cylinder(od / 2.0 - wall, 120.0)),
            Material::new(material, 7850.0),
        )
    }

    /// `base` — `arm` — `tip`, with the mass out at the tip.
    fn fixture(tip_com: Vector3<f64>) -> (Mechanism, Origins, MassMap) {
        let m = Mechanism::builder("t")
            .part(tube("base", 25.4, 2.0, "steel"))
            .part(tube("arm", 25.4, 2.0, "steel"))
            .part(tube("tip", 25.4, 2.0, "steel"))
            .joint(JointDef::new(
                "j1",
                "base",
                "arm",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::y(),
            ))
            .joint(JointDef::new(
                "j2",
                "arm",
                "tip",
                JointKind::Fixed,
                Point3::origin(),
                Vector3::y(),
            ))
            .build();
        let origins = Origins::from([
            ("base".to_owned(), Vector3::zeros()),
            ("arm".to_owned(), Vector3::zeros()),
            ("tip".to_owned(), tip_com),
        ]);
        let masses = MassMap::from([
            (
                "base".to_owned(),
                MassPoint {
                    kg: 1.0,
                    world_com_mm: Vector3::zeros(),
                },
            ),
            (
                "arm".to_owned(),
                MassPoint {
                    kg: 1.0,
                    world_com_mm: tip_com / 2.0,
                },
            ),
            (
                "tip".to_owned(),
                MassPoint {
                    kg: 10.0,
                    world_com_mm: tip_com,
                },
            ),
        ]);
        (m, origins, masses)
    }

    fn arm_of(loads: Screen) -> MemberLoad {
        loads.members.into_iter().find(|l| l.part == "arm").unwrap()
    }

    /// ★★★ **The only gate here that pins an ABSOLUTE stress**, and the reason
    /// it exists: every other one is relative — more mass is more stress,
    /// double the g and the stress doubles — and a screen can satisfy all of
    /// them while being wrong by a constant factor. A tenfold gravity error, a
    /// thousandfold mm-versus-metre slip of exactly the kind this repo has
    /// already been bitten by, and a centre of mass averaged instead of
    /// mass-weighted each pass every relative check in this file.
    ///
    /// Worked by hand, independently of the code:
    ///
    /// ```text
    /// section   25.4 x 2.0 tube -> Z = pi(25.4^4 - 21.4^4)/(32 x 25.4)
    ///                                = 798.169 mm^3
    /// masses    1 kg at x=100, 9 kg at x=900
    /// com       (1x100 + 9x900)/10 = 820 mm      <- WEIGHTED, not 500
    /// load      10 kg x 9.81 m/s^2 = 98.10 N
    /// moment    98.10 N x 820 mm   = 80 442 N.mm
    /// stress    80 442 / 798.169   = 100.78 MPa
    /// ```
    ///
    /// ⚠ The sampled `Z` runs a little under the closed form by construction —
    /// see the direction assertion in
    /// [`a_section_modulus_matches_the_closed_form_for_a_tube`] — so the
    /// measured stress sits a little ABOVE 100.78. The tolerance covers that
    /// and nothing like a factor.
    #[test]
    fn the_stress_of_a_hand_computed_cantilever_comes_out_right() {
        let m = Mechanism::builder("t")
            .part(tube("base", 25.4, 2.0, "steel"))
            .part(tube("arm", 25.4, 2.0, "steel"))
            .part(tube("tip", 25.4, 2.0, "steel"))
            .joint(JointDef::new(
                "j1",
                "base",
                "arm",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::y(),
            ))
            .joint(JointDef::new(
                "j2",
                "arm",
                "tip",
                JointKind::Fixed,
                Point3::origin(),
                Vector3::y(),
            ))
            .build();
        let origins = Origins::from([
            ("base".to_owned(), Vector3::zeros()),
            ("arm".to_owned(), Vector3::zeros()),
            ("tip".to_owned(), Vector3::new(900.0, 0.0, 0.0)),
        ]);
        let masses = MassMap::from([
            (
                "base".to_owned(),
                MassPoint {
                    kg: 1.0,
                    world_com_mm: Vector3::zeros(),
                },
            ),
            (
                "arm".to_owned(),
                MassPoint {
                    kg: 1.0,
                    world_com_mm: Vector3::new(100.0, 0.0, 0.0),
                },
            ),
            (
                "tip".to_owned(),
                MassPoint {
                    kg: 9.0,
                    world_com_mm: Vector3::new(900.0, 0.0, 0.0),
                },
            ),
        ]);

        // ⚠ A finer cell than the 0.5 mm default, on purpose. At 0.5 the
        // sampled `Z` runs about 2% under the closed form and the comparison
        // goes slack; at 0.25 it is inside half a percent, which leaves the
        // tolerance below tight enough to catch a real scale error rather than
        // merely a gross one.
        let mut case = LoadCase::static_1g();
        case.section_cell_mm = 0.25;
        let arm = arm_of(member_loads(&m, &origins, &masses, &case));
        assert!(
            (arm.supported_kg - 10.0).abs() < 1e-9,
            "supported {} kg, hand figure 10",
            arm.supported_kg
        );
        assert!(
            (arm.lever_mm - 820.0).abs() < 1e-6,
            "lever {:.3} mm, hand figure 820 — an UNWEIGHTED mean would read 500",
            arm.lever_mm
        );
        assert!(
            (arm.load_n - 98.10).abs() < 1e-6,
            "load {:.4} N, hand figure 98.10",
            arm.load_n
        );
        let want = 100.783;
        assert!(
            (arm.stress_mpa - want).abs() < want * 0.02,
            "stress {:.3} MPa against a hand-computed {want:.3}",
            arm.stress_mpa
        );
    }

    /// The instrument, against arithmetic it cannot influence.
    #[test]
    fn a_section_modulus_matches_the_closed_form_for_a_tube() {
        let (od, wall) = (25.4, 2.0);
        let solid =
            Solid::cylinder(od / 2.0, 100.0).subtract(Solid::cylinder(od / 2.0 - wall, 120.0));
        let (z, axis) = section_modulus(&solid, 0.25).unwrap();
        let id: f64 = od - 2.0 * wall;
        let exact = std::f64::consts::PI * (od.powi(4) - id.powi(4)) / (32.0 * od);
        assert!(
            (z - exact).abs() < exact * 0.02,
            "sampled Z {z:.1}, closed form {exact:.1}"
        );
        // ★★ The DIRECTION, not just the magnitude. A screen that reads Z high
        // reports a member as less stressed than it is, which is the one error
        // a screen must not make. Measuring the extreme fibre to the outer edge
        // of the outermost cell rather than its centre is what buys this.
        assert!(
            z <= exact * 1.001,
            "Z came out ABOVE the closed form ({z:.2} vs {exact:.2})"
        );
        assert!(
            axis.z.abs() > 0.99,
            "the long axis of a z-aligned tube should be z, got {axis:?}"
        );
    }

    /// ⛔ The bug this was written for. The coarse pass's step has to resolve
    /// the member's SECTION, which lives on its SHORT axes; scaling it to the
    /// long one drops every slender member. On the trike that silently lost
    /// the chassis rail and BOTH swingarms — seven of thirty-four — while the
    /// header counted only the survivors.
    ///
    /// ⚠ **Demonstrated failing only with BOTH halves of the fix reverted.**
    /// The short-axis step and the adaptive refinement are redundant on
    /// purpose — either alone rescues this tube — so neither is individually
    /// gated and a regression in one would pass. That redundancy is wanted
    /// (min-span/12 is 2.6 mm against this 2 mm wall, which is close), but the
    /// cost is stated rather than hidden: this gate witnesses the bug as it
    /// occurred, not each contributing factor.
    #[test]
    fn a_long_slender_tube_is_still_sampled() {
        // 2300 mm of 31.75 x 2.0: the chassis rail that went missing, at an
        // aspect ratio of 72.
        let (od, wall) = (31.75, 2.0);
        let solid =
            Solid::cylinder(od / 2.0, 1150.0).subtract(Solid::cylinder(od / 2.0 - wall, 1200.0));
        let (z, axis) = section_modulus(&solid, 0.5).unwrap();
        let id: f64 = od - 2.0 * wall;
        let exact = std::f64::consts::PI * (od.powi(4) - id.powi(4)) / (32.0 * od);
        assert!(
            (z - exact).abs() < exact * 0.03,
            "a 72:1 tube sampled Z {z:.1} against {exact:.1}"
        );
        assert!(
            z <= exact * 1.001,
            "Z above the closed form: {z:.2}/{exact:.2}"
        );
        assert!(axis.z.abs() > 0.99, "long axis should be z, got {axis:?}");
    }

    /// ⛔⛔ Unmeasured is REPORTED, never dropped. A screen that quietly omits
    /// what it cannot read always looks clean.
    #[test]
    fn a_member_that_cannot_be_sampled_is_named_not_dropped() {
        let (m, origins, masses) = fixture(Vector3::new(500.0, 0.0, 0.0));
        // A cell coarser than the whole part: nothing can be sampled.
        let mut case = LoadCase::static_1g();
        case.section_cell_mm = 10_000.0;
        let screen = member_loads(&m, &origins, &masses, &case);
        assert!(
            screen.members.is_empty(),
            "nothing should have been measurable: {:?}",
            screen.members
        );
        assert!(
            screen.unmeasured.iter().any(|u| u.part == "arm"),
            "the unmeasured member must be named, got {:?}",
            screen.unmeasured
        );
    }

    /// ⛔ A member the caller never placed. Silently dropping it shrinks the
    /// denominator as well as the numerator, so the header reads clean.
    #[test]
    fn a_member_missing_from_origins_is_named_not_dropped() {
        let (m, mut origins, masses) = fixture(Vector3::new(500.0, 0.0, 0.0));
        origins.remove("arm");
        let screen = member_loads(&m, &origins, &masses, &LoadCase::static_1g());
        assert!(
            !screen.members.iter().any(|l| l.part == "arm"),
            "unplaceable member was scored anyway"
        );
        assert!(
            screen
                .unmeasured
                .iter()
                .any(|u| u.part == "arm" && u.why == Unmeasured::NoOrigin),
            "got {:?}",
            screen.unmeasured
        );
    }

    /// ⛔⛔ The dangerous one: a member whose CHILD has no mass is still
    /// perfectly measurable, and its number would look entirely ordinary while
    /// being short by whatever the child weighs.
    #[test]
    fn a_member_whose_subtree_is_missing_a_mass_is_withheld_not_understated() {
        let reach = Vector3::new(500.0, 0.0, 0.0);
        let (m, origins, mut gappy) = fixture(reach);
        let whole = member_loads(&m, &origins, &gappy, &LoadCase::static_1g())
            .members
            .into_iter()
            .find(|l| l.part == "arm")
            .unwrap();

        gappy.remove("tip"); // the 10 kg at the end of the lever
        let screen = member_loads(&m, &origins, &gappy, &LoadCase::static_1g());

        assert!(
            !screen.members.iter().any(|l| l.part == "arm"),
            "arm was scored against a subtree the map does not cover: {:?}",
            screen.members
        );
        assert!(
            screen
                .unmeasured
                .iter()
                .any(|u| u.part == "arm" && u.why == Unmeasured::IncompleteSubtree),
            "got {:?}",
            screen.unmeasured
        );
        // What the old behaviour would have reported, for the record: the arm
        // carrying its own kilogram instead of eleven.
        assert!(
            whole.supported_kg > 10.0,
            "fixture no longer loads the arm through its child"
        );
    }

    /// ⚠ `NaN > 1.0` is false, so a `NaN` utilisation reads as "not over yield"
    /// at every call site. Withheld, not reported.
    #[test]
    fn a_non_finite_mass_is_withheld_rather_than_scored_as_nan() {
        let (m, origins, mut masses) = fixture(Vector3::new(500.0, 0.0, 0.0));
        masses.get_mut("tip").unwrap().kg = f64::NAN;
        let screen = member_loads(&m, &origins, &masses, &LoadCase::static_1g());
        assert!(
            !screen
                .members
                .iter()
                .any(|l| l.stress_mpa.is_nan() || l.utilisation.is_some_and(f64::is_nan)),
            "a NaN reached the members list: {:?}",
            screen.members
        );
        assert!(
            screen
                .unmeasured
                .iter()
                .any(|u| u.why == Unmeasured::NotFinite),
            "got {:?}",
            screen.unmeasured
        );
    }

    /// A weightless subtree is named too — it is not a member that passed.
    #[test]
    fn a_member_carrying_no_mass_at_all_is_named() {
        let (m, origins, mut masses) = fixture(Vector3::new(500.0, 0.0, 0.0));
        for v in masses.values_mut() {
            v.kg = 0.0;
        }
        let screen = member_loads(&m, &origins, &masses, &LoadCase::static_1g());
        assert!(screen.members.is_empty(), "{:?}", screen.members);
        assert!(
            screen
                .unmeasured
                .iter()
                .any(|u| u.why == Unmeasured::NoMass),
            "got {:?}",
            screen.unmeasured
        );
    }

    /// ★ The whole reason the 2x2 tensor is diagonalised rather than read off
    /// whichever basis the sampler happened to build: a member bends about its
    /// WEAK axis, and on a rectangle the two differ four-fold.
    #[test]
    fn the_weak_axis_is_found_on_an_asymmetric_section() {
        let solid = Solid::cuboid(Vector3::new(20.0, 5.0, 100.0));
        let (z, _) = section_modulus(&solid, 0.25).unwrap();
        let weak = 40.0 * 10.0_f64.powi(3) / 12.0 / 5.0;
        let strong = 10.0 * 40.0_f64.powi(3) / 12.0 / 20.0;
        assert!(
            (z - weak).abs() < weak * 0.01,
            "got {z:.1}; weak axis is {weak:.1}, strong is {strong:.1}"
        );
    }

    /// ⛔⛔ A FREE joint is a joint, but it is not a support. Scoring against
    /// it produces a number from an arbitrary origin — the trike's chassis
    /// rail read 0.69x, then 1.17x once the wheels got heavier, and crossing
    /// 1.0 made a meaningless figure start failing a gate.
    #[test]
    fn a_member_hanging_from_a_free_joint_is_not_scored() {
        let m = Mechanism::builder("t")
            .part(tube("base", 25.4, 2.0, "steel"))
            .part(tube("arm", 25.4, 2.0, "steel"))
            .joint(JointDef::new(
                "j",
                "world",
                "base",
                JointKind::Free,
                Point3::origin(),
                Vector3::z(),
            ))
            .joint(JointDef::new(
                "j2",
                "base",
                "arm",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::y(),
            ))
            .build();
        let origins = Origins::from([
            ("base".to_owned(), Vector3::zeros()),
            ("arm".to_owned(), Vector3::new(500.0, 0.0, 0.0)),
        ]);
        let masses = MassMap::from([
            (
                "base".to_owned(),
                MassPoint {
                    kg: 50.0,
                    world_com_mm: Vector3::new(300.0, 0.0, 0.0),
                },
            ),
            (
                "arm".to_owned(),
                MassPoint {
                    kg: 5.0,
                    world_com_mm: Vector3::new(900.0, 0.0, 0.0),
                },
            ),
        ]);
        let screen = member_loads(&m, &origins, &masses, &LoadCase::static_1g());
        assert!(
            !screen.members.iter().any(|l| l.part == "base"),
            "a body on a free joint was scored: {:?}",
            screen.members
        );
        assert!(
            screen
                .unmeasured
                .iter()
                .any(|u| u.part == "base" && u.why == Unmeasured::NotSupported),
            "got {:?}",
            screen.unmeasured
        );
        // The member that DOES hang from a real joint is still scored.
        assert!(screen.members.iter().any(|l| l.part == "arm"));
    }

    /// The stated blind spot, gated so it cannot quietly stop being true.
    #[test]
    fn a_part_with_no_parent_joint_is_not_reported() {
        let (m, origins, masses) = fixture(Vector3::new(500.0, 0.0, 0.0));
        let loads = member_loads(&m, &origins, &masses, &LoadCase::static_1g()).members;
        assert!(
            !loads.iter().any(|l| l.part == "base"),
            "the root has nothing to cantilever from and must not be scored: {loads:?}"
        );
        assert!(loads.iter().any(|l| l.part == "arm"));
    }

    #[test]
    fn a_member_carrying_more_mass_is_more_stressed() {
        let reach = Vector3::new(500.0, 0.0, 0.0);
        let (m, origins, mut masses) = fixture(reach);
        let light = arm_of(member_loads(&m, &origins, &masses, &LoadCase::static_1g())).stress_mpa;

        masses.get_mut("tip").unwrap().kg = 100.0;
        let heavy = arm_of(member_loads(&m, &origins, &masses, &LoadCase::static_1g())).stress_mpa;

        assert!(
            heavy > light * 5.0,
            "ten times the tip mass read {heavy:.1} MPa against {light:.1}"
        );
    }

    #[test]
    fn a_longer_lever_is_more_stressed() {
        let (m1, o1, ms1) = fixture(Vector3::new(500.0, 0.0, 0.0));
        let short = arm_of(member_loads(&m1, &o1, &ms1, &LoadCase::static_1g())).stress_mpa;
        let (m2, o2, ms2) = fixture(Vector3::new(1500.0, 0.0, 0.0));
        let long = arm_of(member_loads(&m2, &o2, &ms2, &LoadCase::static_1g())).stress_mpa;
        assert!(
            long > short * 2.5,
            "tripling the reach read {long:.1} MPa against {short:.1}"
        );
    }

    #[test]
    fn doubling_the_g_factor_doubles_the_stress() {
        let (m, origins, masses) = fixture(Vector3::new(500.0, 0.0, 0.0));
        let one = arm_of(member_loads(&m, &origins, &masses, &LoadCase::static_1g())).stress_mpa;
        let case = LoadCase::static_1g().at_g(2.0);
        let two = arm_of(member_loads(&m, &origins, &masses, &case)).stress_mpa;
        assert!(
            (two - one * 2.0).abs() < one * 1e-9,
            "{two} is not twice {one}"
        );
    }

    /// ⚠ Load hanging straight below its anchor is compression, not bending,
    /// and reporting a moment for it would make every vertical post look
    /// loaded.
    #[test]
    fn a_mass_hanging_straight_below_its_anchor_bends_nothing() {
        let (m, origins, masses) = fixture(Vector3::new(0.0, 0.0, -500.0));
        let arm = arm_of(member_loads(&m, &origins, &masses, &LoadCase::static_1g()));
        assert!(
            arm.lever_mm < 1e-9 && arm.stress_mpa < 1e-9,
            "a plumb load bent the member: {arm:?}"
        );
    }

    /// ⛔ An undeclared material is reported, not scored. Inventing an
    /// allowable would turn a silence into a false pass.
    #[test]
    fn a_material_with_no_declared_allowable_is_reported_unscored() {
        let (m, origins, masses) = fixture(Vector3::new(500.0, 0.0, 0.0));
        let bare = arm_of(member_loads(&m, &origins, &masses, &LoadCase::static_1g()));
        assert!(bare.utilisation.is_none() && bare.allowable_mpa.is_none());
        assert!(bare.stress_mpa > 0.0, "stress is still measured");

        let case = LoadCase::static_1g().allowing("steel", 460.0);
        let scored = arm_of(member_loads(&m, &origins, &masses, &case));
        let util = scored.utilisation.unwrap();
        assert!(
            (util - scored.stress_mpa / 460.0).abs() < 1e-12,
            "utilisation {util} is not stress over allowable"
        );
    }

    /// Worst first, so a caller that reads one entry reads the right one.
    #[test]
    fn the_worst_utilisation_is_reported_first() {
        let (m, origins, masses) = fixture(Vector3::new(500.0, 0.0, 0.0));
        let case = LoadCase::static_1g().allowing("steel", 460.0);
        let loads = member_loads(&m, &origins, &masses, &case).members;
        let utils: Vec<f64> = loads.iter().filter_map(|l| l.utilisation).collect();
        assert!(utils.len() >= 2, "expected both arm and tip scored");
        assert!(
            utils.windows(2).all(|w| w[0] >= w[1]),
            "not sorted worst first: {utils:?}"
        );
    }
}
