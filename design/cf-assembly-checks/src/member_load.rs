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
//! - **A part that is never a joint's child is not reported at all**, having
//!   nothing to cantilever from. ⚠ That is narrower than "the root": a root
//!   anchored to the world by a joint — `parent: "world"`, which is not a
//!   declared part — IS scored. What is true of such a root is worse than
//!   silence: the lever is measured from a body origin sitting in the middle
//!   of the structure rather than at a support, so the number is meaningless
//!   rather than absent.
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
    /// Allowable stress in MPa, keyed by the material's name. A material absent
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
    pub fn at_g(mut self, g_factor: f64) -> Self {
        self.g_factor = g_factor;
        self
    }

    /// Declare what a material allows, in MPa.
    #[must_use]
    pub fn allowing(mut self, material: impl Into<String>, mpa: f64) -> Self {
        self.allowable_mpa.insert(material.into(), mpa);
        self
    }
}

/// What the screen measured, and what it could not.
///
/// ⛔⛔ **The second field is the point.** A screen that quietly drops what it
/// cannot read always looks clean — it reports "27 members, 2 past yield" while
/// seven went unmeasured, and the reader has no way to know. Unmeasured is
/// reported here, never dropped.
#[derive(Debug, Clone, PartialEq)]
pub struct Screen {
    /// One entry per member whose section could be sampled, worst first.
    pub members: Vec<MemberLoad>,
    /// Parts that hang from a joint and were **not measured**: a bent member
    /// whose centroid misses its own material, a part with no bounds, one that
    /// samples to nothing at this cell.
    ///
    /// ⛔ **Unmeasured, not sound.** Treat a non-empty list as a failure of the
    /// screen, not a pass for the parts.
    pub unsampled: Vec<String>,
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
    /// `moment / Z`, MPa.
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
/// ⛔ Read [`Screen::unsampled`] before reading the members. It is the list the
/// header count does not include.
///
/// See the module docs for the five things this cannot see.
#[must_use]
pub fn member_loads(
    mechanism: &Mechanism,
    origins: &Origins,
    masses: &MassMap,
    case: &LoadCase,
) -> Screen {
    let mut unsampled = Vec::new();
    let Some(down) = case.down.try_normalize(1e-12) else {
        return Screen {
            members: Vec::new(),
            unsampled,
        };
    };
    let children = child_index(mechanism);
    let has_parent: HashSet<&str> = mechanism
        .joints()
        .iter()
        .map(cf_design::JointDef::child)
        .collect();

    let mut out = Vec::new();
    for part in mechanism.parts() {
        let name = part.name();
        if !has_parent.contains(name) {
            continue; // the root: nothing to cantilever from. A stated blind spot.
        }
        let Some(&anchor) = origins.get(name) else {
            continue;
        };
        let supported = subtree(&children, name);
        let (supported_kg, com) = combined_mass(&supported, masses);
        if supported_kg <= 0.0 {
            continue;
        }
        let Some((modulus, _)) =
            section_modulus(part.solid(), case.section_cell_mm).filter(|&(z, _)| z > 0.0)
        else {
            unsampled.push(name.to_string());
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
    unsampled.sort();
    Screen {
        members: out,
        unsampled,
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
        for next in children.get(node).map(Vec::as_slice).unwrap_or(&[]) {
            if seen.insert(next) {
                out.push(next);
                queue.push_back(next);
            }
        }
    }
    out
}

/// Total mass of a set of parts and where it acts.
fn combined_mass(parts: &[&str], masses: &MassMap) -> (f64, Vector3<f64>) {
    let mut kg = 0.0;
    let mut moment = Vector3::zeros();
    for name in parts {
        if let Some(m) = masses.get(*name) {
            kg += m.kg;
            moment += m.world_com_mm * m.kg;
        }
    }
    if kg <= 0.0 {
        return (0.0, Vector3::zeros());
    }
    (kg, moment / kg)
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
    let bounds = solid.bounds()?;
    let lo = bounds.min;
    let hi = bounds.max;

    // A coarse interior sample: enough to find the long axis and the centroid,
    // cheap enough that it is not the cost of the check.
    //
    // ⛔⛔ **The step is set by the member's SMALLEST span, not its largest.**
    // Scaling it to the longest dimension is the obvious thing and it is wrong:
    // a 2300 mm chassis rail of 31.75 mm tube then gets probed every 96 mm,
    // lands inside a 2 mm wall essentially never, and the whole member is
    // dropped for want of eight interior points. The step has to resolve the
    // SECTION, and the section lives on the short axes.
    //
    // ⚠ Adaptive, because even min-span/12 misses a thin enough wall. It
    // halves until it has points or reaches the caller's cell, which bounds
    // the work: a member that truly cannot be sampled costs four extra passes,
    // not an unbounded search.
    let span = hi - lo;
    let min_span = span.x.min(span.y).min(span.z);
    let mut coarse = (min_span / 12.0).max(cell_mm);
    let mut pts: Vec<Vector3<f64>> = Vec::new();
    for _ in 0..5 {
        pts.clear();
        let mut walk = lo.z + coarse * 0.5;
        while walk < hi.z {
            let mut y = lo.y + coarse * 0.5;
            while y < hi.y {
                let mut x = lo.x + coarse * 0.5;
                while x < hi.x {
                    let p = Vector3::new(x, y, walk);
                    if solid.evaluate(&Point3::from(p)) < 0.0 {
                        pts.push(p);
                    }
                    x += coarse;
                }
                y += coarse;
            }
            walk += coarse;
        }
        if pts.len() >= 64 || coarse <= cell_mm {
            break;
        }
        coarse = (coarse / 2.0).max(cell_mm);
    }
    if pts.len() < 8 {
        return None;
    }

    let centroid = pts.iter().sum::<Vector3<f64>>() / pts.len() as f64;
    let mut cov = Matrix3::zeros();
    for p in &pts {
        let d = p - centroid;
        cov += d * d.transpose();
    }
    let eig = cov.symmetric_eigen();
    let long = (0..3).max_by(|&a, &b| eig.eigenvalues[a].total_cmp(&eig.eigenvalues[b]))?;
    let axis = Vector3::from(eig.eigenvectors.column(long)).normalize();

    // An orthonormal basis across the section.
    let seed = if axis.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u = axis.cross(&seed).normalize();
    let v = axis.cross(&u).normalize();

    // Sample the plane through the centroid, normal to the long axis.
    //
    // ⚠ **The half-extent comes from the coarse points, not from the body.**
    // Using the body diagonal is "safe" and ruinous: a 2300 mm rail would have
    // its section sampled over a 2300 x 2300 mm plane at half a millimetre —
    // 21 million evaluations to read a 31.75 mm tube. The section can be no
    // wider than the material already found lying off the axis, plus a margin
    // for what the coarse grid stepped over.
    let reach = pts
        .iter()
        .map(|p| {
            let d = p - centroid;
            (d - axis * d.dot(&axis)).norm()
        })
        .fold(0.0_f64, f64::max)
        + coarse * 2.0;
    let area_cell = cell_mm * cell_mm;
    let mut sum = Vector3::zeros();
    let mut cell_pts: Vec<(f64, f64)> = Vec::new();
    let mut a = -reach;
    while a <= reach {
        let mut b = -reach;
        while b <= reach {
            let p = centroid + u * a + v * b;
            if solid.evaluate(&Point3::from(p)) < 0.0 {
                cell_pts.push((a, b));
                sum += Vector3::new(a, b, 0.0);
            }
            b += cell_mm;
        }
        a += cell_mm;
    }
    if cell_pts.len() < 4 {
        return None;
    }

    let n = cell_pts.len() as f64;
    let (ca, cb) = (sum.x / n, sum.y / n);
    let (mut i_aa, mut i_bb, mut i_ab) = (0.0, 0.0, 0.0);
    for &(a, b) in &cell_pts {
        let (da, db) = (a - ca, b - cb);
        i_aa += db * db * area_cell; // bending about the `a` axis
        i_bb += da * da * area_cell;
        i_ab -= da * db * area_cell;
    }
    let tensor = Matrix2::new(i_aa, i_ab, i_ab, i_bb);
    let eig2 = tensor.symmetric_eigen();
    let (weak, strong) = if eig2.eigenvalues[0] <= eig2.eigenvalues[1] {
        (0, 1)
    } else {
        (1, 0)
    };
    let i_min = eig2.eigenvalues[weak];
    // The extreme fibre is measured across the NEUTRAL axis of the weak
    // bending mode, which is the strong principal direction.
    let neutral = eig2.eigenvectors.column(strong);
    let c = cell_pts
        .iter()
        .map(|&(a, b)| ((a - ca) * neutral[0] + (b - cb) * neutral[1]).abs())
        .fold(0.0_f64, f64::max);
    if c <= 0.0 {
        return None;
    }
    Some((i_min / c, axis))
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
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
            (z - exact).abs() < exact * 0.05,
            "sampled Z {z:.1}, closed form {exact:.1}"
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
            (z - exact).abs() < exact * 0.08,
            "a 72:1 tube sampled Z {z:.1} against {exact:.1}"
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
            screen.unsampled.contains(&"arm".to_owned()),
            "the unmeasured member must be named, got {:?}",
            screen.unsampled
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
            (z - weak).abs() < weak * 0.06,
            "got {z:.1}; weak axis is {weak:.1}, strong is {strong:.1}"
        );
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
        let (m, origins, masses) = fixture(reach);
        let light = arm_of(member_loads(&m, &origins, &masses, &LoadCase::static_1g())).stress_mpa;

        let mut heavier = masses.clone();
        heavier.get_mut("tip").unwrap().kg = 100.0;
        let heavy = arm_of(member_loads(&m, &origins, &heavier, &LoadCase::static_1g())).stress_mpa;

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
