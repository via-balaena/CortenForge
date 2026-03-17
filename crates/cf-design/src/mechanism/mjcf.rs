//! MJCF XML generation from a validated mechanism.
//!
//! Converts a [`super::Mechanism`] into a MuJoCo-compatible MJCF XML string.
//! The generated XML is self-contained: mesh vertex/face data is embedded
//! inline in `<mesh>` asset elements (no external files).
//!
//! # Body hierarchy
//!
//! The kinematic tree is derived from joint parent/child relationships:
//! - The **root part** (appears as joint parent but never as child) becomes
//!   the top-level `<body>` under `<worldbody>`.
//! - Each child part nests inside its parent's `<body>`.
//! - Joints are placed on the child body (MJCF convention).
//!
//! # MJCF element mapping
//!
//! | cf-design type | MJCF element |
//! |---------------|-------------|
//! | [`Part`](super::Part) | `<body>` + `<geom type="mesh">` + `<mesh>` asset |
//! | [`JointDef`](super::JointDef) (Revolute) | `<joint type="hinge">` |
//! | [`JointDef`](super::JointDef) (Prismatic) | `<joint type="slide">` |
//! | [`JointDef`](super::JointDef) (Ball) | `<joint type="ball">` |
//! | [`JointDef`](super::JointDef) (Free) | `<freejoint>` |
//! | [`TendonDef`](super::TendonDef) | `<spatial>` tendon with `<site>` waypoints |
//! | [`ActuatorDef`](super::ActuatorDef) (Motor) | `<general>` actuator |
//! | [`ActuatorDef`](super::ActuatorDef) (Muscle) | `<muscle>` actuator |

// `String::write_fmt` is infallible — `let _ = write!(string, ...)` is correct.
#![allow(clippy::let_underscore_must_use)]

use std::collections::HashMap;
use std::fmt::Write as _;

use super::actuator::ActuatorKind;
use super::builder::Mechanism;
use super::joint::{JointDef, JointKind};
use super::part::Part;
use super::tendon::TendonDef;

// ── Public entry point ──────────────────────────────────────────────────

/// Generate MJCF XML from a validated mechanism.
///
/// Each part is meshed at the given `resolution` via
/// [`Solid::mesh`](crate::Solid::mesh). The generated XML embeds vertex/face
/// data inline in `<mesh>` asset elements.
///
/// # Panics
///
/// Panics if `resolution` is not positive and finite.
pub(super) fn generate(mechanism: &Mechanism, resolution: f64) -> String {
    assert!(
        resolution > 0.0 && resolution.is_finite(),
        "MJCF resolution must be positive and finite, got {resolution}"
    );

    let mut xml = String::new();

    let _ = writeln!(xml, "<mujoco model=\"{}\">", esc(mechanism.name()));
    let _ = writeln!(xml, "  <compiler angle=\"radian\"/>");

    write_assets(&mut xml, mechanism, resolution);
    write_worldbody(&mut xml, mechanism);
    write_tendons(&mut xml, mechanism);
    write_actuators(&mut xml, mechanism);

    let _ = write!(xml, "</mujoco>");
    // No trailing newline — caller can add one if desired.
    xml
}

// ── XML helpers ─────────────────────────────────────────────────────────

/// Escape XML special characters in attribute values.
fn esc(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

// ── Asset section ───────────────────────────────────────────────────────

fn write_assets(xml: &mut String, mechanism: &Mechanism, resolution: f64) {
    let parts = mechanism.parts();
    if parts.is_empty() {
        return;
    }

    let _ = writeln!(xml, "  <asset>");
    for part in parts {
        let mesh = part.solid().mesh(resolution);

        let _ = write!(xml, "    <mesh name=\"{}_mesh\"", esc(part.name()));

        // Inline vertex data: flat x y z triplets.
        if !mesh.vertices.is_empty() {
            let _ = write!(xml, " vertex=\"");
            for (i, v) in mesh.vertices.iter().enumerate() {
                if i > 0 {
                    let _ = write!(xml, " ");
                }
                let _ = write!(xml, "{} {} {}", v.x, v.y, v.z);
            }
            let _ = write!(xml, "\"");
        }

        // Inline face data: flat index triplets.
        if !mesh.faces.is_empty() {
            let _ = write!(xml, " face=\"");
            for (i, f) in mesh.faces.iter().enumerate() {
                if i > 0 {
                    let _ = write!(xml, " ");
                }
                let _ = write!(xml, "{} {} {}", f[0], f[1], f[2]);
            }
            let _ = write!(xml, "\"");
        }

        let _ = writeln!(xml, "/>");
    }
    let _ = writeln!(xml, "  </asset>");
}

// ── Worldbody ───────────────────────────────────────────────────────────

fn write_worldbody(xml: &mut String, mechanism: &Mechanism) {
    let _ = writeln!(xml, "  <worldbody>");

    let parts = mechanism.parts();
    if parts.is_empty() {
        let _ = writeln!(xml, "  </worldbody>");
        return;
    }

    // Build lookups.
    let part_by_name: HashMap<&str, &Part> = parts.iter().map(|p| (p.name(), p)).collect();

    // child_part → parent_part (first joint wins for degenerate cases).
    let mut child_parent: HashMap<&str, &str> = HashMap::new();
    // child_part → [joints attached to it].
    let mut joints_on: HashMap<&str, Vec<&JointDef>> = HashMap::new();

    for joint in mechanism.joints() {
        child_parent
            .entry(joint.child())
            .or_insert_with(|| joint.parent());
        joints_on.entry(joint.child()).or_default().push(joint);
    }

    // parent → [children], preserving part declaration order.
    let mut children_of: HashMap<&str, Vec<&str>> = HashMap::new();
    for part in parts {
        if let Some(&parent) = child_parent.get(part.name()) {
            children_of.entry(parent).or_default().push(part.name());
        }
    }

    // Roots: parts that never appear as a child in any joint.
    let roots: Vec<&str> = parts
        .iter()
        .map(Part::name)
        .filter(|n| !child_parent.contains_key(n))
        .collect();

    // Site map: part_name → [(site_name, [x, y, z])].
    let sites = build_site_map(mechanism.tendons());

    for root in &roots {
        if let Some(part) = part_by_name.get(root) {
            write_body(
                xml,
                part,
                &part_by_name,
                &joints_on,
                &children_of,
                &sites,
                2,
            );
        }
    }

    let _ = writeln!(xml, "  </worldbody>");
}

/// Build `part_name → Vec<(site_name, [x, y, z])>` from tendon waypoints.
fn build_site_map(tendons: &[TendonDef]) -> HashMap<&str, Vec<(String, [f64; 3])>> {
    let mut map: HashMap<&str, Vec<(String, [f64; 3])>> = HashMap::new();
    for tendon in tendons {
        for (i, wp) in tendon.waypoints().iter().enumerate() {
            let name = format!("{}_s{i}", tendon.name());
            let pos = [wp.position().x, wp.position().y, wp.position().z];
            map.entry(wp.part()).or_default().push((name, pos));
        }
    }
    map
}

/// Recursively write a `<body>` element and its child subtree.
#[allow(clippy::too_many_arguments)]
fn write_body(
    xml: &mut String,
    part: &Part,
    part_by_name: &HashMap<&str, &Part>,
    joints_on: &HashMap<&str, Vec<&JointDef>>,
    children_of: &HashMap<&str, Vec<&str>>,
    sites: &HashMap<&str, Vec<(String, [f64; 3])>>,
    indent: usize,
) {
    let pad = " ".repeat(indent);

    let _ = writeln!(xml, "{pad}<body name=\"{}\">", esc(part.name()));

    // Joints (present only when this part is a child body).
    if let Some(jlist) = joints_on.get(part.name()) {
        for joint in jlist {
            write_joint(xml, joint, indent + 2);
        }
    }

    // Geom referencing the mesh asset, with material density for mass computation.
    let density = part.material().density;
    let _ = writeln!(
        xml,
        "{pad}  <geom type=\"mesh\" mesh=\"{}_mesh\" density=\"{density}\"/>",
        esc(part.name())
    );

    // Tendon waypoint sites on this body.
    if let Some(slist) = sites.get(part.name()) {
        for (name, pos) in slist {
            let _ = writeln!(
                xml,
                "{pad}  <site name=\"{name}\" pos=\"{} {} {}\"/>",
                pos[0], pos[1], pos[2]
            );
        }
    }

    // Nested child bodies.
    if let Some(kids) = children_of.get(part.name()) {
        for kid in kids {
            if let Some(child_part) = part_by_name.get(kid) {
                write_body(
                    xml,
                    child_part,
                    part_by_name,
                    joints_on,
                    children_of,
                    sites,
                    indent + 2,
                );
            }
        }
    }

    let _ = writeln!(xml, "{pad}</body>");
}

/// Write a single `<joint>` or `<freejoint>` element.
fn write_joint(xml: &mut String, joint: &JointDef, indent: usize) {
    let pad = " ".repeat(indent);

    let type_str = match joint.kind() {
        JointKind::Revolute => "hinge",
        JointKind::Prismatic => "slide",
        JointKind::Ball => "ball",
        JointKind::Free => {
            let _ = writeln!(xml, "{pad}<freejoint name=\"{}\"/>", esc(joint.name()));
            return;
        }
    };

    let _ = write!(
        xml,
        "{pad}<joint name=\"{}\" type=\"{type_str}\"",
        esc(joint.name())
    );

    // Anchor position.
    let p = joint.anchor();
    let _ = write!(xml, " pos=\"{} {} {}\"", p.x, p.y, p.z);

    // Axis (hinge and slide only — ball joints have no axis in MJCF).
    if matches!(joint.kind(), JointKind::Revolute | JointKind::Prismatic) {
        let a = joint.axis();
        let _ = write!(xml, " axis=\"{} {} {}\"", a.x, a.y, a.z);
    }

    // Range constraint.
    if let Some((lo, hi)) = joint.range() {
        let _ = write!(xml, " range=\"{lo} {hi}\" limited=\"true\"");
    }

    let _ = writeln!(xml, "/>");
}

// ── Tendons ─────────────────────────────────────────────────────────────

fn write_tendons(xml: &mut String, mechanism: &Mechanism) {
    if mechanism.tendons().is_empty() {
        return;
    }

    let _ = writeln!(xml, "  <tendon>");
    for tendon in mechanism.tendons() {
        let _ = write!(xml, "    <spatial name=\"{}\"", esc(tendon.name()));

        if let Some(k) = tendon.stiffness() {
            let _ = write!(xml, " stiffness=\"{k}\"");
        }
        if let Some(d) = tendon.damping() {
            let _ = write!(xml, " damping=\"{d}\"");
        }

        let _ = writeln!(xml, ">");

        for (i, _) in tendon.waypoints().iter().enumerate() {
            let _ = writeln!(xml, "      <site site=\"{}_s{i}\"/>", tendon.name());
        }

        let _ = writeln!(xml, "    </spatial>");
    }
    let _ = writeln!(xml, "  </tendon>");
}

// ── Actuators ───────────────────────────────────────────────────────────

fn write_actuators(xml: &mut String, mechanism: &Mechanism) {
    if mechanism.actuators().is_empty() {
        return;
    }

    let _ = writeln!(xml, "  <actuator>");
    for act in mechanism.actuators() {
        let tag = match act.kind() {
            ActuatorKind::Motor => "general",
            ActuatorKind::Muscle => "muscle",
        };

        let _ = write!(
            xml,
            "    <{tag} name=\"{}\" tendon=\"{}\"",
            esc(act.name()),
            esc(act.tendon())
        );

        let (flo, fhi) = act.force_range();
        let _ = write!(xml, " forcerange=\"{flo} {fhi}\"");

        if let Some((clo, chi)) = act.ctrl_range() {
            let _ = write!(xml, " ctrlrange=\"{clo} {chi}\"");
        }

        let _ = writeln!(xml, "/>");
    }
    let _ = writeln!(xml, "  </actuator>");
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use crate::{
        ActuatorDef, ActuatorKind, JointDef, JointKind, Material, Mechanism, Part, Solid,
        TendonDef, TendonWaypoint,
    };

    // ── Helpers ─────────────────────────────────────────────────────

    fn pla() -> Material {
        Material::new("PLA", 1250.0)
    }

    fn sphere_part(name: &str) -> Part {
        Part::new(name, Solid::sphere(5.0), pla())
    }

    fn cuboid_part(name: &str) -> Part {
        Part::new(name, Solid::cuboid(Vector3::new(5.0, 5.0, 5.0)), pla())
    }

    /// Build a minimal two-part mechanism (palm + finger, one revolute joint).
    fn two_part_mechanism() -> Mechanism {
        Mechanism::builder("two_part")
            .part(cuboid_part("palm"))
            .part(cuboid_part("finger"))
            .joint(JointDef::new(
                "knuckle",
                "palm",
                "finger",
                JointKind::Revolute,
                Point3::new(5.0, 0.0, 0.0),
                Vector3::x(),
            ))
            .build()
    }

    // Use coarse resolution for fast tests.
    const RES: f64 = 2.0;

    // ── 1. Single part ──────────────────────────────────────────────

    #[test]
    fn single_part_mjcf() {
        let m = Mechanism::builder("solo").part(sphere_part("ball")).build();

        let xml = m.to_mjcf(RES);

        assert!(xml.contains("<mujoco model=\"solo\">"));
        assert!(xml.contains("<compiler angle=\"radian\"/>"));
        assert!(xml.contains("<asset>"));
        assert!(xml.contains("<mesh name=\"ball_mesh\""));
        assert!(xml.contains("</asset>"));
        assert!(xml.contains("<worldbody>"));
        assert!(xml.contains("<body name=\"ball\">"));
        assert!(xml.contains("<geom type=\"mesh\" mesh=\"ball_mesh\" density=\"1250\"/>"));
        assert!(xml.contains("</body>"));
        assert!(xml.contains("</worldbody>"));
        assert!(xml.contains("</mujoco>"));
        // No tendon or actuator sections.
        assert!(!xml.contains("<tendon>"));
        assert!(!xml.contains("<actuator>"));
    }

    // ── 2. Two-part hierarchy ───────────────────────────────────────

    #[test]
    fn two_part_mjcf_structure() {
        let xml = two_part_mechanism().to_mjcf(RES);

        // Palm is root (never a child).
        assert!(xml.contains("<body name=\"palm\">"));
        // Finger is nested inside palm.
        assert!(xml.contains("<body name=\"finger\">"));
        // Joint is on the child body (finger), not on palm.
        assert!(xml.contains("<joint name=\"knuckle\" type=\"hinge\""));

        // Verify nesting: palm opens before finger, finger closes before palm.
        let palm_open = xml.find("<body name=\"palm\">").unwrap();
        let finger_open = xml.find("<body name=\"finger\">").unwrap();
        let finger_close = xml[finger_open..].find("</body>").unwrap() + finger_open;
        let palm_close = xml[finger_close + 1..].find("</body>").unwrap() + finger_close + 1;
        assert!(palm_open < finger_open);
        assert!(finger_open < finger_close);
        assert!(finger_close < palm_close);
    }

    // ── 3. Joint type mapping ───────────────────────────────────────

    #[test]
    fn joint_types_mapped() {
        let cases = [
            (JointKind::Revolute, "hinge"),
            (JointKind::Prismatic, "slide"),
            (JointKind::Ball, "ball"),
        ];

        for (kind, expected_type) in cases {
            let m = Mechanism::builder("jtest")
                .part(sphere_part("a"))
                .part(sphere_part("b"))
                .joint(JointDef::new(
                    "j",
                    "a",
                    "b",
                    kind,
                    Point3::origin(),
                    Vector3::z(),
                ))
                .build();

            let xml = m.to_mjcf(RES);
            assert!(
                xml.contains(&format!("type=\"{expected_type}\"")),
                "expected type=\"{expected_type}\" for {kind:?}, got:\n{xml}"
            );
        }
    }

    #[test]
    fn joint_type_free_mapped() {
        let m = Mechanism::builder("free_test")
            .part(sphere_part("a"))
            .part(sphere_part("b"))
            .joint(JointDef::new(
                "fj",
                "a",
                "b",
                JointKind::Free,
                Point3::origin(),
                Vector3::z(),
            ))
            .build();

        let xml = m.to_mjcf(RES);
        assert!(
            xml.contains("<freejoint name=\"fj\"/>"),
            "expected <freejoint>, got:\n{xml}"
        );
    }

    // ── 4. Joint range ──────────────────────────────────────────────

    #[test]
    fn joint_range_emitted() {
        let m = Mechanism::builder("ranged")
            .part(sphere_part("a"))
            .part(sphere_part("b"))
            .joint(
                JointDef::new(
                    "j",
                    "a",
                    "b",
                    JointKind::Revolute,
                    Point3::origin(),
                    Vector3::z(),
                )
                .with_range(-1.5, 1.5),
            )
            .build();

        let xml = m.to_mjcf(RES);
        assert!(xml.contains("range=\"-1.5 1.5\""), "missing range attr");
        assert!(xml.contains("limited=\"true\""), "missing limited attr");
    }

    #[test]
    fn joint_without_range_has_no_limited() {
        let xml = two_part_mechanism().to_mjcf(RES);
        assert!(!xml.contains("limited="), "unexpected limited attr");
        assert!(!xml.contains("range="), "unexpected range attr");
    }

    // ── 5. Tendon sites on bodies ───────────────────────────────────

    #[test]
    fn tendon_sites_on_bodies() {
        let m = Mechanism::builder("tendon_test")
            .part(cuboid_part("palm"))
            .part(cuboid_part("finger"))
            .joint(JointDef::new(
                "j",
                "palm",
                "finger",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .tendon(TendonDef::new(
                "flex",
                vec![
                    TendonWaypoint::new("palm", Point3::new(1.0, 2.0, 3.0)),
                    TendonWaypoint::new("palm", Point3::new(1.0, 4.0, 3.0)),
                    TendonWaypoint::new("finger", Point3::new(0.0, 1.0, 0.0)),
                    TendonWaypoint::new("finger", Point3::new(0.0, 5.0, 0.0)),
                ],
                1.0,
            ))
            .build();

        let xml = m.to_mjcf(RES);

        // Sites on palm body.
        assert!(xml.contains("site name=\"flex_s0\" pos=\"1 2 3\""));
        assert!(xml.contains("site name=\"flex_s1\" pos=\"1 4 3\""));
        // Sites on finger body.
        assert!(xml.contains("site name=\"flex_s2\" pos=\"0 1 0\""));
        assert!(xml.contains("site name=\"flex_s3\" pos=\"0 5 0\""));

        // Palm sites should appear inside the palm body (before finger body opens).
        let palm_open = xml.find("<body name=\"palm\">").unwrap();
        let finger_open = xml.find("<body name=\"finger\">").unwrap();
        let s0_pos = xml.find("flex_s0").unwrap();
        let s2_pos = xml.find("flex_s2").unwrap();
        assert!(
            s0_pos > palm_open && s0_pos < finger_open,
            "s0 not in palm body"
        );
        assert!(s2_pos > finger_open, "s2 not in finger body");
    }

    // ── 6. Tendon spatial element ───────────────────────────────────

    #[test]
    fn tendon_spatial_element() {
        let m = Mechanism::builder("t")
            .part(cuboid_part("a"))
            .part(cuboid_part("b"))
            .joint(JointDef::new(
                "j",
                "a",
                "b",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .tendon(
                TendonDef::new(
                    "cable",
                    vec![
                        TendonWaypoint::new("a", Point3::origin()),
                        TendonWaypoint::new("b", Point3::new(0.0, 5.0, 0.0)),
                    ],
                    0.5,
                )
                .with_stiffness(200.0)
                .with_damping(10.0),
            )
            .build();

        let xml = m.to_mjcf(RES);
        assert!(xml.contains("<tendon>"));
        assert!(xml.contains("<spatial name=\"cable\""));
        assert!(xml.contains("stiffness=\"200\""));
        assert!(xml.contains("damping=\"10\""));
        assert!(xml.contains("<site site=\"cable_s0\"/>"));
        assert!(xml.contains("<site site=\"cable_s1\"/>"));
        assert!(xml.contains("</spatial>"));
        assert!(xml.contains("</tendon>"));
    }

    // ── 7. Actuator motor ───────────────────────────────────────────

    #[test]
    fn actuator_motor() {
        let m = Mechanism::builder("act")
            .part(cuboid_part("a"))
            .part(cuboid_part("b"))
            .joint(JointDef::new(
                "j",
                "a",
                "b",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .tendon(TendonDef::new(
                "t",
                vec![
                    TendonWaypoint::new("a", Point3::origin()),
                    TendonWaypoint::new("b", Point3::new(0.0, 5.0, 0.0)),
                ],
                0.5,
            ))
            .actuator(
                ActuatorDef::new("motor_1", "t", ActuatorKind::Motor, (-50.0, 50.0))
                    .with_ctrl_range(-1.0, 1.0),
            )
            .build();

        let xml = m.to_mjcf(RES);
        assert!(xml.contains("<actuator>"));
        assert!(xml.contains("<general name=\"motor_1\" tendon=\"t\""));
        assert!(xml.contains("forcerange=\"-50 50\""));
        assert!(xml.contains("ctrlrange=\"-1 1\""));
        assert!(xml.contains("</actuator>"));
    }

    // ── 8. Actuator muscle ──────────────────────────────────────────

    #[test]
    fn actuator_muscle() {
        let m = Mechanism::builder("musc")
            .part(cuboid_part("a"))
            .part(cuboid_part("b"))
            .joint(JointDef::new(
                "j",
                "a",
                "b",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .tendon(TendonDef::new(
                "t",
                vec![
                    TendonWaypoint::new("a", Point3::origin()),
                    TendonWaypoint::new("b", Point3::new(0.0, 5.0, 0.0)),
                ],
                0.5,
            ))
            .actuator(ActuatorDef::new(
                "bicep",
                "t",
                ActuatorKind::Muscle,
                (0.0, 200.0),
            ))
            .build();

        let xml = m.to_mjcf(RES);
        assert!(xml.contains("<muscle name=\"bicep\" tendon=\"t\""));
        assert!(xml.contains("forcerange=\"0 200\""));
        // No ctrl_range set → attribute absent.
        assert!(!xml.contains("ctrlrange="));
    }

    // ── 9. Bio-gripper round trip ───────────────────────────────────

    #[test]
    fn bio_gripper_round_trip() {
        let palm = Solid::cuboid(Vector3::new(18.0, 22.0, 4.0));
        let finger = Solid::capsule(3.0, 12.0);
        let mat = pla();

        let m = Mechanism::builder("bio_gripper")
            .part(Part::new("palm", palm, mat.clone()))
            .part(Part::new("finger_1", finger.clone(), mat.clone()))
            .part(Part::new("finger_2", finger, mat))
            .joint(
                JointDef::new(
                    "knuckle_1",
                    "palm",
                    "finger_1",
                    JointKind::Revolute,
                    Point3::new(10.0, 20.0, 0.0),
                    Vector3::x(),
                )
                .with_range(-0.1, 1.8),
            )
            .joint(
                JointDef::new(
                    "knuckle_2",
                    "palm",
                    "finger_2",
                    JointKind::Revolute,
                    Point3::new(-10.0, 20.0, 0.0),
                    Vector3::x(),
                )
                .with_range(-0.1, 1.8),
            )
            .tendon(
                TendonDef::new(
                    "flexor_1",
                    vec![
                        TendonWaypoint::new("palm", Point3::new(8.0, -10.0, 0.0)),
                        TendonWaypoint::new("palm", Point3::new(8.0, 18.0, 0.0)),
                        TendonWaypoint::new("finger_1", Point3::new(0.0, 5.0, 0.0)),
                        TendonWaypoint::new("finger_1", Point3::new(0.0, 20.0, 0.0)),
                    ],
                    1.5,
                )
                .with_stiffness(100.0)
                .with_damping(5.0),
            )
            .actuator(
                ActuatorDef::new("motor_1", "flexor_1", ActuatorKind::Motor, (-50.0, 50.0))
                    .with_ctrl_range(-1.0, 1.0),
            )
            .build();

        let xml = m.to_mjcf(RES);

        // ── Structural checks ───────────────────────────────────────
        // 3 bodies.
        assert!(xml.contains("<body name=\"palm\">"));
        assert!(xml.contains("<body name=\"finger_1\">"));
        assert!(xml.contains("<body name=\"finger_2\">"));

        // 3 mesh assets.
        assert!(xml.contains("<mesh name=\"palm_mesh\""));
        assert!(xml.contains("<mesh name=\"finger_1_mesh\""));
        assert!(xml.contains("<mesh name=\"finger_2_mesh\""));

        // 2 joints on child bodies.
        assert!(xml.contains("<joint name=\"knuckle_1\" type=\"hinge\""));
        assert!(xml.contains("<joint name=\"knuckle_2\" type=\"hinge\""));

        // 1 spatial tendon with 4 site refs.
        assert!(xml.contains("<spatial name=\"flexor_1\""));
        assert!(xml.contains("<site site=\"flexor_1_s0\"/>"));
        assert!(xml.contains("<site site=\"flexor_1_s1\"/>"));
        assert!(xml.contains("<site site=\"flexor_1_s2\"/>"));
        assert!(xml.contains("<site site=\"flexor_1_s3\"/>"));

        // 4 site declarations on bodies.
        assert!(xml.contains("site name=\"flexor_1_s0\""));
        assert!(xml.contains("site name=\"flexor_1_s3\""));

        // 1 actuator.
        assert!(xml.contains("<general name=\"motor_1\" tendon=\"flexor_1\""));

        // Palm is root → finger bodies nested inside.
        let palm_pos = xml.find("<body name=\"palm\">").unwrap();
        let f1_pos = xml.find("<body name=\"finger_1\">").unwrap();
        let f2_pos = xml.find("<body name=\"finger_2\">").unwrap();
        assert!(palm_pos < f1_pos);
        assert!(palm_pos < f2_pos);
    }

    // ── 10. Mesh data embedded ──────────────────────────────────────

    #[test]
    fn mesh_data_embedded() {
        let m = Mechanism::builder("mesh_test")
            .part(sphere_part("ball"))
            .build();

        let xml = m.to_mjcf(RES);

        // The mesh element should have vertex and face data.
        assert!(xml.contains("vertex=\""), "missing vertex data");
        assert!(xml.contains("face=\""), "missing face data");

        // Extract the vertex attribute to verify it has actual coordinates.
        let mesh_line = xml
            .lines()
            .find(|l| l.contains("<mesh name=\"ball_mesh\""))
            .unwrap();
        assert!(
            mesh_line.contains("vertex=\""),
            "mesh element missing vertex attr"
        );
        assert!(
            mesh_line.contains("face=\""),
            "mesh element missing face attr"
        );
    }

    // ── 11. XML well-formed ─────────────────────────────────────────

    #[test]
    fn xml_well_formed() {
        let m = two_part_mechanism();
        let xml = m.to_mjcf(RES);

        // Verify matching open/close tags for all structural elements.
        let tag_pairs = [
            ("<mujoco", "</mujoco>"),
            ("<asset>", "</asset>"),
            ("<worldbody>", "</worldbody>"),
        ];
        for (open, close) in tag_pairs {
            let open_count = xml.matches(open).count();
            let close_count = xml.matches(close).count();
            assert_eq!(
                open_count, close_count,
                "mismatched {open} / {close}: {open_count} opens vs {close_count} closes"
            );
        }

        // Each <body> has a matching </body>.
        let body_opens = xml.matches("<body ").count();
        let body_closes = xml.matches("</body>").count();
        assert_eq!(
            body_opens, body_closes,
            "mismatched <body> / </body>: {body_opens} vs {body_closes}"
        );

        // Self-closing tags are well-formed.
        assert!(xml.contains("/>"), "expected self-closing tags");
        // No unclosed non-self-closing tags (basic check: every < has a matching >).
        assert_eq!(
            xml.matches('<').count(),
            xml.matches('>').count(),
            "mismatched < and > counts"
        );
    }
}
