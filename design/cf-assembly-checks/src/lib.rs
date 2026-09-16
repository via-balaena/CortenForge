//! Assembly checks that do not know what they are checking.
//!
//! A [`Mechanism`] describes an assembly: parts, where they sit, and how they
//! are joined. That description is the same whether the assembly is a reverse
//! trike, a gripper, a boat or a humanoid — fifteen crates and examples in this
//! workspace already build one. The checks that read it should be the same too.
//!
//! ## What belongs here, and what does not
//!
//! ★ **A check belongs here if it can be stated without naming a domain.**
//! *"No two parts that are not joined may occupy the same space"* is true of a
//! boat. *"The rollover threshold must exceed µ"* is not — it is a ground
//! vehicle's question, and it lives in `cf-vehicle`.
//!
//! ⚠⚠ **THE CALLER SUPPLIES ANYTHING DOMAIN-SHAPED.** These checks need facts a
//! `Mechanism` does not carry — which parts are wheels, what counts as travel,
//! which pairs may legitimately touch. They take those as arguments and never
//! infer them. **The moment this crate learns the word "wheel" it is a vehicle
//! library again**, and the next machine cannot use it.
//!
//! ## Why the checks report rather than judge
//!
//! Every function here returns what it measured. None of them decide whether
//! the answer is acceptable, because the threshold is a property of the machine
//! and not of the check: a rider's torso and thigh are *supposed* to overlap at
//! the hip, and two frame tubes are supposed to overlap at a node.

use std::collections::{HashMap, HashSet, VecDeque};

use cf_design::mechanism::Mechanism;
use nalgebra::{Point3, Vector3};

/// Where every part sits, by name, in a common frame.
pub type Origins = HashMap<String, Vector3<f64>>;

/// How many of one part's probe points fall inside another.
///
/// ⚠ **An extent, not a distance.** `Solid::evaluate` on a CSG solid is a
/// *bound* rather than a true distance — it has read a 40 mm interpenetration
/// as 0.8 — so only the sign is trustworthy. Counting points inside is a
/// measure of how much of one part is swallowed by the other, and that is what
/// separates a real collision from parts meeting at a shared node.
#[derive(Debug, Clone, PartialEq)]
pub struct Overlap {
    /// One part.
    pub a: String,
    /// The other.
    pub b: String,
    /// Probe points of either found inside the other.
    pub points: usize,
    /// The largest share of ONE part that is inside the other, in `0.0..=1.0`.
    ///
    /// ★ **Scale-invariant, and that is the point.** A raw point count depends
    /// on how big the parts are and how fine the probe is, so a threshold tuned
    /// on one assembly is wrong on the next — a threshold of 120 points read
    /// node contact correctly on a 108 kg trike and misread it on the same
    /// vehicle at car scale. A *fraction* means the same thing on a gripper and
    /// on a boat: touching at a node swallows a few percent of a part, and
    /// occupying its space swallows tens of percent.
    pub fraction: f64,
    /// How far apart the two are in the joint-and-linkage graph.
    ///
    /// ⚠ **Reported, but NOT a discriminator, and that was measured.** Parts
    /// far apart in the tree are often adjacent in space: a brace ending on a
    /// tower, a panel resting on its rails, two arms sharing a ball joint. On
    /// one assembly, six of the eight pairs more than two hops apart were
    /// entirely legitimate. **[`Overlap::points`] is what separates them** —
    /// node contact ran 2–36 points and true interpenetration 257–6735.
    pub hops: usize,
}

/// Every pair of parts that occupies the same space without being joined.
///
/// This is the check that catches a part which is the right shape, the right
/// mass and in the **wrong place** — a defect no mass or volume gate can see,
/// because each of them reads one part at a time in its own frame.
///
/// `probe_mm` is the mesh resolution for the probe points; finer costs more and
/// finds smaller overlaps.
///
/// Pairs joined by a joint or a linkage are skipped: they touch by
/// construction, and a scan that reported them would be switched off.
#[must_use]
pub fn overlapping_pairs(mechanism: &Mechanism, origins: &Origins, probe_mm: f64) -> Vec<Overlap> {
    let parts = mechanism.parts();
    let joined = adjacency(mechanism);

    let mut probes: HashMap<&str, Vec<Point3<f64>>> = HashMap::new();
    for p in parts {
        let mesh = p.solid().mesh(probe_mm).geometry;
        probes.insert(
            p.name(),
            mesh.vertices.iter().map(|v| Point3::from(*v)).collect(),
        );
    }

    let mut out = Vec::new();
    for (i, pa) in parts.iter().enumerate() {
        for pb in parts.iter().skip(i + 1) {
            let (na, nb) = (pa.name(), pb.name());
            if joined.contains(&(na.to_owned(), nb.to_owned())) {
                continue;
            }
            let (Some(&oa), Some(&ob)) = (origins.get(na), origins.get(nb)) else {
                continue;
            };
            let inside = |from: &[Point3<f64>], shift: Vector3<f64>, into: &cf_design::Solid| {
                from.iter()
                    .filter(|v| into.evaluate(&Point3::from(v.coords + shift)) < 0.0)
                    .count()
            };
            let a_in_b = inside(&probes[na], oa - ob, pb.solid());
            let b_in_a = inside(&probes[nb], ob - oa, pa.solid());
            let points = a_in_b + b_in_a;
            if points > 0 {
                let share = |n: usize, of: usize| {
                    if of == 0 { 0.0 } else { n as f64 / of as f64 }
                };
                out.push(Overlap {
                    a: na.to_owned(),
                    b: nb.to_owned(),
                    points,
                    fraction: share(a_in_b, probes[na].len()).max(share(b_in_a, probes[nb].len())),
                    hops: hops_between(mechanism, na, nb),
                });
            }
        }
    }
    out.sort_by(|x, y| {
        y.fraction
            .total_cmp(&x.fraction)
            .then(x.a.cmp(&y.a))
            .then(x.b.cmp(&y.b))
    });
    out
}

/// What an assembly declares itself to be.
///
/// ⚠ Asserted because **every walk over a collection passes trivially when the
/// collection is empty**, and an empty `Mechanism` builds happily. Pinning the
/// counts is what stops a whole suite of oracles from silently checking
/// nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Counts {
    /// Parts in the assembly.
    pub parts: usize,
    /// Joints that are welds — a weld is the absence of a joint downstream.
    pub welds: usize,
    /// Degrees of freedom summed over the joint tree, before linkages.
    pub tree_dof: usize,
    /// Loop-closing constraints.
    pub linkages: usize,
    /// Degrees of freedom the linkages take back.
    pub held_dof: usize,
}

impl Counts {
    /// Degrees of freedom the machine actually has.
    #[must_use]
    pub const fn free_dof(&self) -> usize {
        self.tree_dof.saturating_sub(self.held_dof)
    }
}

/// Count what the assembly holds, for comparison against what it should.
#[must_use]
pub fn counts(mechanism: &Mechanism) -> Counts {
    Counts {
        parts: mechanism.parts().len(),
        welds: mechanism
            .joints()
            .iter()
            .filter(|j| j.kind().is_weld())
            .count(),
        tree_dof: mechanism.joints().iter().map(|j| j.kind().dof()).sum(),
        linkages: mechanism.linkages().len(),
        held_dof: mechanism
            .linkages()
            .iter()
            .map(|l| l.kind().constrained_dof())
            .sum(),
    }
}

/// Pairs joined by a joint or a linkage, both ways round.
fn adjacency(mechanism: &Mechanism) -> HashSet<(String, String)> {
    let mut out = HashSet::new();
    let mut add = |a: &str, b: &str| {
        out.insert((a.to_owned(), b.to_owned()));
        out.insert((b.to_owned(), a.to_owned()));
    };
    for j in mechanism.joints() {
        add(j.parent(), j.child());
    }
    for l in mechanism.linkages() {
        add(l.a(), l.b());
    }
    out
}

/// Shortest path between two parts through joints and linkages.
fn hops_between(mechanism: &Mechanism, from: &str, to: &str) -> usize {
    let mut adj: HashMap<&str, Vec<&str>> = HashMap::new();
    for j in mechanism.joints() {
        adj.entry(j.parent()).or_default().push(j.child());
        adj.entry(j.child()).or_default().push(j.parent());
    }
    for l in mechanism.linkages() {
        adj.entry(l.a()).or_default().push(l.b());
        adj.entry(l.b()).or_default().push(l.a());
    }
    let mut seen: HashMap<&str, usize> = HashMap::from([(from, 0)]);
    let mut queue: VecDeque<&str> = VecDeque::from([from]);
    while let Some(node) = queue.pop_front() {
        let d = seen[node];
        if node == to {
            return d;
        }
        for next in adj.get(node).map(Vec::as_slice).unwrap_or(&[]) {
            if !seen.contains_key(next) {
                seen.insert(next, d + 1);
                queue.push_back(next);
            }
        }
    }
    usize::MAX
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use cf_design::{JointDef, JointKind, LinkageDef, LinkageKind, Material, Part, Solid};

    use super::*;

    fn ball(name: &str) -> Part {
        Part::new(name, Solid::sphere(10.0), Material::new("steel", 7850.0))
    }

    /// A chain `a`—`b`—`c`, with `c` placed back on top of `a`.
    ///
    /// ⚠ Every part must be joined to something: `Mechanism::build` rejects an
    /// orphan. So the overlapping pair has to be two ENDS of a chain rather
    /// than a free-floating part — which is the realistic case anyway, since a
    /// real assembly's collisions are between things two or more hops apart.
    fn fixture() -> (Mechanism, Origins) {
        let hinge = |name: &str, parent: &str, child: &str| {
            JointDef::new(
                name,
                parent,
                child,
                JointKind::Revolute,
                Point3::origin(),
                Vector3::y(),
            )
        };
        let m = Mechanism::builder("t")
            .part(ball("a"))
            .part(ball("b"))
            .part(ball("c"))
            .joint(hinge("j1", "a", "b"))
            .joint(hinge("j2", "b", "c"))
            .build();
        let origins = Origins::from([
            ("a".to_owned(), Vector3::zeros()),
            ("b".to_owned(), Vector3::new(40.0, 0.0, 0.0)),
            ("c".to_owned(), Vector3::new(1.0, 0.0, 0.0)),
        ]);
        (m, origins)
    }

    /// Two parts in the same place with nothing joining them is the finding.
    #[test]
    fn an_unjoined_pair_in_the_same_space_is_reported() {
        let (m, origins) = fixture();
        let found = overlapping_pairs(&m, &origins, 2.0);
        let pair: Vec<&str> = found
            .iter()
            .map(|o| {
                if o.a == "c" {
                    o.b.as_str()
                } else {
                    o.a.as_str()
                }
            })
            .collect();
        assert!(
            found.iter().any(|o| o.a == "c" || o.b == "c"),
            "c sits inside a and was not reported: {found:?}"
        );
        assert!(
            found.iter().all(|o| o.points > 0),
            "reported a zero overlap"
        );
        assert!(!pair.is_empty());
    }

    /// ⚠ **Joined parts are skipped, and this is the gate for it.**
    ///
    /// Parts that share a joint touch by construction. A scan that reported
    /// them would be drowned in false positives and switched off — which is
    /// exactly what happened twice while this was still inline in a vehicle
    /// example: a wishbone "entering" the upright hanging off its own ball
    /// joint, and a body welded to the frame it is welded to.
    #[test]
    fn parts_that_share_a_joint_are_not_reported() {
        let (m, mut origins) = fixture();
        // put b right on top of a — they share a joint, so it must stay silent
        origins.insert("b".to_owned(), Vector3::new(1.0, 0.0, 0.0));
        origins.insert("c".to_owned(), Vector3::new(80.0, 0.0, 0.0));
        let found = overlapping_pairs(&m, &origins, 2.0);
        assert!(
            !found
                .iter()
                .any(|o| { (o.a == "a" && o.b == "b") || (o.a == "b" && o.b == "a") }),
            "a and b share a joint and must not be reported: {found:?}"
        );
    }

    /// A linkage joins a pair just as a joint does.
    #[test]
    fn parts_tied_by_a_linkage_are_not_reported_either() {
        let (_, mut origins) = fixture();
        // tie the two ends together: now they are joined and must stay silent
        let m = Mechanism::builder("t")
            .part(ball("a"))
            .part(ball("b"))
            .part(ball("c"))
            .joint(JointDef::new(
                "j1",
                "a",
                "b",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::y(),
            ))
            .joint(JointDef::new(
                "j2",
                "b",
                "c",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::y(),
            ))
            .linkage(LinkageDef::new(
                "tie",
                "a",
                "c",
                LinkageKind::Ball,
                Point3::origin(),
            ))
            .build();
        origins.insert("c".to_owned(), Vector3::new(1.0, 0.0, 0.0));
        assert!(
            overlapping_pairs(&m, &origins, 2.0).is_empty(),
            "a linkage joins its pair as surely as a joint does"
        );
    }

    /// Graph distance is reported, and it is not the discriminator.
    #[test]
    fn graph_distance_is_reported_beside_the_extent() {
        let (m, origins) = fixture();
        let found = overlapping_pairs(&m, &origins, 2.0);
        let o = found.first().expect("an overlap");
        assert_eq!(o.hops, 2, "a and c are two hops apart, through b");
        assert!(o.points > 0);
    }

    /// The counts an assembly declares itself to have.
    #[test]
    fn counts_report_the_tree_and_what_the_linkages_take_back() {
        let m = Mechanism::builder("t")
            .part(ball("a"))
            .part(ball("b"))
            .part(ball("c"))
            .joint(JointDef::new(
                "hinge",
                "a",
                "b",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::y(),
            ))
            .joint(JointDef::new(
                "weld",
                "a",
                "c",
                JointKind::Fixed,
                Point3::origin(),
                Vector3::y(),
            ))
            .linkage(LinkageDef::new(
                "tie",
                "b",
                "c",
                LinkageKind::Ball,
                Point3::origin(),
            ))
            .build();

        let c = counts(&m);
        assert_eq!(c.parts, 3);
        assert_eq!(c.welds, 1, "a weld is a joint that emits none");
        assert_eq!(c.tree_dof, 1, "one hinge; the weld costs nothing");
        assert_eq!(c.linkages, 1);
        assert_eq!(c.held_dof, 3, "a ball linkage holds three");
        assert_eq!(
            c.free_dof(),
            0,
            "the linkage takes back more than the tree has"
        );
    }
}
