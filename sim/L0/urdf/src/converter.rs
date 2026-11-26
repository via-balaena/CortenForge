//! URDF to MJCF conversion.
//!
//! Converts URDF XML to MJCF XML, following `MuJoCo`'s approach of having
//! a single compilation path. This ensures that URDF models benefit from
//! all the validation and optimization in the MJCF → Model pipeline.
//!
//! ## Design Rationale
//!
//! `MuJoCo` itself converts URDF to its internal representation before compilation.
//! Rather than duplicating the Model construction logic, we convert URDF → MJCF
//! and use the existing `sim_mjcf::load_model()` function.
//!
//! This gives us:
//! - Single source of truth for Model construction
//! - All MJCF features (defaults, actuators, sensors) available
//! - Consistent validation and error handling
//!
//! ## URDF → MJCF Mapping
//!
//! | URDF | MJCF |
//! |------|------|
//! | `<robot>` | `<mujoco>` |
//! | `<link>` | `<body>` (the link named `world`: `<worldbody>`) |
//! | `<joint>` | `<joint>` (inside child body; a planar one: two slides and a hinge) |
//! | `<inertial>` | `<inertial>` |
//! | `<collision>` | `<geom>` |
//! | `<visual>` | a `<geom>` of no contacts in group 1, only under `discardvisual="false"` |
//!
//! ## Key Differences
//!
//! 1. **Tree structure**: URDF uses flat links + joints with parent/child refs.
//!    MJCF uses nested body elements. We build the tree during conversion.
//!
//! 2. **Joint placement**: In URDF, joint origin defines where child attaches.
//!    In MJCF, body pos is relative to parent and joint rotates in place.
//!
//! 3. **Inertia frame**: URDF inertia is at link origin by default.
//!    MJCF inertia can have its own pos/quat within the body.

use std::collections::{HashMap, HashSet};
use std::fmt::Write;

use crate::error::{Result, UrdfError};
use crate::parser::parse_urdf_str;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::types::{
    UrdfElement, UrdfGeometry, UrdfJoint, UrdfJointLimit, UrdfJointType, UrdfLink, UrdfOrigin,
    UrdfRobot,
};

/// The link name MuJoCo 3.5.0's importer reads as the world body
/// (`xml_urdf.cc:727`).
const WORLD: &str = "world";

/// A name or path as an MJCF attribute's text: `&`, `<`, `>`, `"` and `'`
/// written as entities, so the document the converter writes reads the text
/// the URDF held.
fn escape(text: &str) -> std::borrow::Cow<'_, str> {
    let escaped = quick_xml::escape::escape(text);
    // A carriage return as a character reference: written as itself, the
    // reader would take it for a line end and read `\n` (measured: a robot
    // named `a&#13;b` is "a\rb" in MuJoCo 3.5.0).
    if escaped.contains('\r') {
        std::borrow::Cow::Owned(escaped.replace('\r', "&#13;"))
    } else {
        escaped
    }
}

/// A mesh file's path as the converter writes it: under `strippath`, the
/// importer's default for a URDF, its `package://` or `file://` prefix
/// stripped ([`strip_package_prefix`]), for the compiler strips the path to
/// the file's name; otherwise as written, as MuJoCo 3.5.0's importer keeps it
/// (measured: a `package://` mesh under `strippath="false"` is refused by
/// MuJoCo, "Error opening file").
fn mesh_file(path: &str, strippath: bool) -> String {
    if strippath {
        strip_package_prefix(path)
    } else {
        path.to_owned()
    }
}

/// Strip `package://.../<file>` prefix, returning just the filename.
/// Also strips `file://` prefix. Passes through plain paths unchanged.
fn strip_package_prefix(path: &str) -> String {
    if let Some(rest) = path.strip_prefix("package://") {
        // "package://pkg_name/meshes/file.stl" → "meshes/file.stl"
        if let Some(idx) = rest.find('/') {
            rest[idx + 1..].to_string()
        } else {
            rest.to_string()
        }
    } else if let Some(rest) = path.strip_prefix("file://") {
        rest.to_string()
    } else {
        path.to_string()
    }
}

/// Whether a URDF `xyz` or `rpy` holds anything but `+0`s: MuJoCo 3.5.0's
/// importer reads each one written, however small (`Origin`,
/// `xml_urdf.cc:648-669`), and starts from the origin and the identity,
/// which `+0`s are.
fn written(v: &Vector3<f64>) -> bool {
    v.iter().any(|x| x.to_bits() != 0)
}

/// A URDF tensor's six moments in the order MuJoCo 3.5.0's importer fills a
/// body's `fullinertia` with them (`ixx iyy izz ixy ixz iyz`,
/// `xml_urdf.cc:252-258`).
fn full_moments(inertia: &crate::types::UrdfInertia) -> [f64; 6] {
    [
        inertia.ixx,
        inertia.iyy,
        inertia.izz,
        inertia.ixy,
        inertia.ixz,
        inertia.iyz,
    ]
}

/// The principal moments and the inertial frame of a link whose `<inertial>`
/// origin turns (its `rpy` [`written`]), in the link's frame: the moments
/// MuJoCo 3.5.0's `mjuu_fullInertia` makes of the tensor as written, and the
/// frame the origin's turn composed with the tensor's principal axes, which is
/// what MuJoCo's importer computes (`xml_urdf.cc:272-283`) before its compiler
/// overwrites the frame with the unturned axes (ledger L372;
/// D-URDF-INERTIAL-TURN). The tensor is R·I·Rᵀ, R the turn; its moments, and
/// every check made of them, are MuJoCo's: a negative moment is raised to 0
/// as MuJoCo's compiler raises it to `boundinertia` (`std::max`,
/// `user_objects.cc:2450-2454`): one `mjuu_eig3`'s sort, which swaps only
/// moments more than 1e-12 apart, does not place last.
///
/// `None` where nothing turns; where MuJoCo refuses the tensor (its last
/// moment after that sort below mjEPS), which the converter then writes
/// unturned for sim-mjcf to refuse in MuJoCo's words; and where the moments
/// or the frame are not finite (a tensor near the largest double), which
/// sim-mjcf then refuses unturned (D-INERTIA-NONFINITE). A component of the
/// frame below the smallest normal double is written as a zero of its sign:
/// MJCF's reader, as MuJoCo's, refuses a written subnormal, and the frame
/// moves by less than 2.3e-308.
fn turned_frame(moments: &[f64; 6], rpy: &Vector3<f64>) -> Option<([f64; 3], UnitQuaternion<f64>)> {
    if !written(rpy) {
        return None;
    }
    let (principal, [w, x, y, z]) = sim_mjcf::full_inertia(moments).ok()?;
    let axes = UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z));
    let frame = UnitQuaternion::from_euler_angles(rpy.x, rpy.y, rpy.z) * axes;
    if !principal
        .iter()
        .chain(frame.coords.iter())
        .all(|v| v.is_finite())
    {
        return None;
    }
    let normal = |v: f64| {
        if v.abs() < f64::MIN_POSITIVE {
            0.0_f64.copysign(v)
        } else {
            v
        }
    };
    let frame = Quaternion::new(
        normal(frame.w),
        normal(frame.i),
        normal(frame.j),
        normal(frame.k),
    );
    let raised = principal.map(|moment| if moment < 0.0 { 0.0 } else { moment });
    Some((raised, UnitQuaternion::new_unchecked(frame)))
}

/// A geom's ` pos` and ` euler` attributes for its URDF origin, each where
/// [`written`].
fn pose_attrs(origin: &UrdfOrigin) -> String {
    let mut attrs = String::new();
    let (pos, rpy) = (origin.xyz, origin.rpy);
    if written(&pos) {
        write!(attrs, r#" pos="{} {} {}""#, pos.x, pos.y, pos.z).ok();
    }
    if written(&rpy) {
        write!(attrs, r#" euler="{} {} {}""#, rpy.x, rpy.y, rpy.z).ok();
    }
    attrs
}

/// Convert URDF XML string to MJCF XML string.
///
/// This is the primary conversion function. The resulting MJCF can be
/// passed directly to `sim_mjcf::load_model()`.
///
/// # Example
///
/// ```ignore
/// use sim_urdf::urdf_to_mjcf;
/// use sim_mjcf::load_model;
///
/// let urdf = r#"<robot name="arm">...</robot>"#;
/// let mjcf = urdf_to_mjcf(urdf)?;
/// let model = load_model(&mjcf)?;
/// ```
pub fn urdf_to_mjcf(urdf_xml: &str) -> Result<String> {
    let robot = parse_urdf_str(urdf_xml)?;
    robot_to_mjcf(&robot)
}

/// Convert a parsed `UrdfRobot` to MJCF XML string.
pub fn robot_to_mjcf(robot: &UrdfRobot) -> Result<String> {
    Converter::new(robot)?.convert()
}

/// A mimic constraint collected during tree walk.
struct MimicEntry {
    /// The follower joint name (the joint that has `<mimic>`).
    follower: String,
    /// The leader joint name.
    leader: String,
    /// Polynomial coefficients: position = multiplier * leader + offset.
    multiplier: f64,
    offset: f64,
}

/// A `<limit>`'s MJCF attributes, as MuJoCo 3.5.0's URDF importer reads it
/// (`xml_urdf.cc:500-508`): one bound written, or a lower above the upper,
/// leaves the joint unlimited (`limited="false"`); otherwise the importer
/// leaves `limited` to the compiler (`mjLIMITED_AUTO`). Under `autolimits`
/// the compiler limits such a range only where lower < upper (`islimited`,
/// `user_objects.cc:184-189`), and sim-mjcf refuses an automatic one with
/// lower >= upper (D-AUTO-RANGE), so that range is written
/// `limited="false"`, which limits it no more. Without `autolimits` every
/// range the importer leaves automatic is written so, for the compiler to
/// refuse a written range as MuJoCo's does (`checklimited`, `:171-181`). A
/// bound not written is 0.
fn limit_attrs(limit: &UrdfJointLimit, autolimits: bool) -> String {
    let (lower, upper) = (limit.lower.unwrap_or(0.0), limit.upper.unwrap_or(0.0));
    let bad = limit.lower.is_some() != limit.upper.is_some() || lower > upper;
    let automatic = !bad && (lower < upper || !autolimits);
    let limited = if automatic { "" } else { r#" limited="false""# };
    format!(r#"{limited} range="{lower} {upper}""#)
}

/// A `<dynamics>`'s damping and friction as MJCF attributes, each where
/// positive.
fn dynamics_attrs(joint: &UrdfJoint) -> String {
    let mut attrs = String::new();
    if let Some(dynamics) = &joint.dynamics {
        if dynamics.damping > 0.0 {
            write!(attrs, r#" damping="{}""#, dynamics.damping).ok();
        }
        if dynamics.friction > 0.0 {
            write!(attrs, r#" frictionloss="{}""#, dynamics.friction).ok();
        }
    }
    attrs
}

/// A planar joint's two slide axes, as MuJoCo 3.5.0's importer makes them
/// from the axis as written (`xml_urdf.cc:448-449`): the first two columns
/// of `mjuu_quat2mat(mjuu_z2quat(axis))` (`user_util.cc:203-240`, `:395-408`),
/// in their operations and order.
// `mjuu_quat2mat`'s identity shortcut is an exact `==` (a zero of either
// sign passes it).
#[allow(clippy::float_cmp)]
fn planar_axes(axis: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    // mjuu_z2quat: about = z × axis, normalised by mjuu_normvec, which leaves
    // a vector of squared length below mjEPS as it is and returns 0, and one
    // within mjEPS of unit length unscaled.
    let z = [0.0, 0.0, 1.0];
    let mut about = [
        z[1] * axis.z - z[2] * axis.y,
        z[2] * axis.x - z[0] * axis.z,
        z[0] * axis.y - z[1] * axis.x,
    ];
    let squared = about[0] * about[0] + about[1] * about[1] + about[2] * about[2];
    let length = if squared < MJ_EPS {
        0.0
    } else {
        let length = squared.sqrt();
        if (length - 1.0).abs() > MJ_EPS {
            about = about.map(|c| c / length);
        }
        length
    };
    if length < 1e-10 {
        about = [1.0, 0.0, 0.0];
    }
    let angle = length.atan2(axis.z);
    let (sin, cos) = ((angle / 2.0).sin(), (angle / 2.0).cos());
    let q = [cos, about[0] * sin, about[1] * sin, about[2] * sin];
    // mjuu_quat2mat, its identity shortcut included; columns 0 and 1.
    if q == [1.0, 0.0, 0.0, 0.0] {
        return (Vector3::x(), Vector3::y());
    }
    let (q00, q01, q02, q03) = (q[0] * q[0], q[0] * q[1], q[0] * q[2], q[0] * q[3]);
    let (q11, q12, q13) = (q[1] * q[1], q[1] * q[2], q[1] * q[3]);
    let (q22, q23, q33) = (q[2] * q[2], q[2] * q[3], q[3] * q[3]);
    (
        Vector3::new(q00 + q11 - q22 - q33, 2.0 * (q12 + q03), 2.0 * (q13 - q02)),
        Vector3::new(2.0 * (q12 - q03), q00 - q11 + q22 - q33, 2.0 * (q23 + q01)),
    )
}

/// MuJoCo's `mjEPS`.
const MJ_EPS: f64 = 1e-14;

/// A mesh asset collected during geom conversion.
struct MeshAssetEntry {
    /// The asset's name, as MuJoCo 3.5.0's importer names it
    /// ([`mesh_stem`]; a later scale of one stem appends its index).
    name: String,
    /// File path from URDF (package:// prefix stripped).
    file: String,
    /// Optional scale.
    scale: Option<nalgebra::Vector3<f64>>,
}

/// A mesh file's name stem: its text past the last `/` or `\`, cut at the
/// last `.`, as MuJoCo 3.5.0's importer names a mesh (`mjuu_strippath`,
/// `mjuu_stripext`, `xml_urdf.cc:593-594`).
fn mesh_stem(file: &str) -> &str {
    let base = file.rsplit(['/', '\\']).next().unwrap_or(file);
    base.rfind('.').map_or(base, |dot| &base[..dot])
}

/// Whether two mesh scales are the one MuJoCo 3.5.0's importer compares
/// exactly, an unwritten scale being 1 1 1 (`xml_urdf.cc:583-608`).
fn same_scale(a: Option<Vector3<f64>>, b: Option<Vector3<f64>>) -> bool {
    let one = Vector3::new(1.0, 1.0, 1.0);
    a.unwrap_or(one) == b.unwrap_or(one)
}

/// The elements of `link` MuJoCo 3.5.0's importer makes geoms of, in document
/// order: its collisions, and its visuals where it keeps them (`discardvisual`
/// false, `xml_urdf.cc:318`).
fn geom_elements(link: &UrdfLink, discardvisual: bool) -> Vec<UrdfElement> {
    link.elements_in_order()
        .into_iter()
        .filter(|e| !discardvisual || matches!(e, UrdfElement::Collision(_)))
        .collect()
}

/// Each link's geoms ([`geom_elements`]) with their names, as MuJoCo 3.5.0's
/// importer names them (`xml_urdf.cc:326-336`, `:349-359`): links in document
/// order, a link's geoms in order, a name kept only where no earlier geom
/// has it, a repeat left unnamed with MuJoCo's warning; an empty name is no
/// name, without a warning (measured: two `name=""` give geoms
/// `[None, None]`).
fn geom_names(robot: &UrdfRobot) -> HashMap<&str, Vec<(UrdfElement, Option<String>)>> {
    let discardvisual = robot.mujoco.discardvisual();
    let mut seen = HashSet::new();
    let mut names = HashMap::new();
    for link in &robot.links {
        let kept = geom_elements(link, discardvisual)
            .into_iter()
            .map(|element| {
                let written = match element {
                    UrdfElement::Visual(i) => link.visuals[i].name.as_ref(),
                    UrdfElement::Collision(i) => link.collisions[i].name.as_ref(),
                };
                let name = written.filter(|n| !n.is_empty()).and_then(|name| {
                    if seen.insert(name.as_str()) {
                        return Some(name.clone());
                    }
                    tracing::warn!(
                        "Geom with duplicate name '{name}' encountered in URDF, creating an unnamed geom."
                    );
                    None
                });
                (element, name)
            })
            .collect();
        names.insert(link.name.as_str(), kept);
    }
    names
}

/// A visual geom's attributes, as MuJoCo 3.5.0's importer makes one: no
/// contacts, group 1, density 0 (`xml_urdf.cc:538-543`).
const VISUAL: &str = r#" contype="0" conaffinity="0" group="1" density="0""#;

/// Attributes as written in an MJCF start tag, each value escaped.
fn attribute_text(attributes: &[(String, String)]) -> String {
    let mut text = String::new();
    for (name, value) in attributes {
        write!(text, r#" {name}="{}""#, escape(value)).ok();
    }
    text
}

/// One section's MJCF lines: `tag` with its attributes, and its child
/// `child_tag` with the child's where written.
fn section_lines(
    tag: &str,
    attributes: &[(String, String)],
    child_tag: &str,
    child: Option<&[(String, String)]>,
) -> Vec<String> {
    let open = format!("<{tag}{}", attribute_text(attributes));
    match child {
        Some(child) => vec![
            format!("{open}>"),
            format!("  <{child_tag}{}/>", attribute_text(child)),
            format!("</{tag}>"),
        ],
        None => vec![format!("{open}/>")],
    }
}

/// Attributes of a URDF's section `tag`, or of its child where given,
/// checked as sim-mjcf's reader checks them in a converted model, alone in an
/// MJCF model, in sim-mjcf's words. sim-mjcf reads them as MuJoCo 3.5.0's MJCF
/// reader does, but for those it does not read, which it does not check
/// (`<size memory>`, `nkey`, …: ledger L315, L455; `<lengthrange>`: L37).
///
/// # Errors
///
/// [`UrdfError::Section`] with sim-mjcf's error.
pub fn check_section(
    tag: &str,
    attributes: &[(String, String)],
    child: Option<(&str, &[(String, String)])>,
) -> Result<()> {
    let (child_tag, child_attributes) = child.map_or(("", None), |(t, a)| (t, Some(a)));
    let mut lines = vec!["<mujoco>".to_owned()];
    lines.extend(section_lines(tag, attributes, child_tag, child_attributes));
    lines.push("</mujoco>".to_owned());
    sim_mjcf::parse_mjcf_str(&lines.join("\n"))
        .map(drop)
        .map_err(UrdfError::Section)
}

/// Internal converter state.
struct Converter<'a> {
    robot: &'a UrdfRobot,
    /// Map from link name to link
    links: HashMap<&'a str, &'a UrdfLink>,
    /// Map from child link name to joint connecting it to parent
    child_to_joint: HashMap<&'a str, &'a UrdfJoint>,
    /// Map from parent link name to list of child joints
    parent_to_joints: HashMap<&'a str, Vec<&'a UrdfJoint>>,
    /// Each link's geoms, in order, with their names ([`geom_names`]).
    geom_names: HashMap<&'a str, Vec<(UrdfElement, Option<String>)>>,
    /// Output buffer
    output: String,
    /// Current indentation level
    indent: usize,
    /// Mesh assets collected during tree walk.
    mesh_assets: Vec<MeshAssetEntry>,
    /// Mimic constraints collected during tree walk.
    mimic_constraints: Vec<MimicEntry>,
}

impl<'a> Converter<'a> {
    /// Index the robot's links and joints, refusing what MuJoCo 3.5.0's
    /// importer refuses (`xml_urdf.cc`; measured): a link name given twice
    /// ("repeated URDF name"), a joint naming a link the robot has not ("URDF
    /// joint parent or child missing"), and a link with a second parent joint
    /// ("URDF body has multiple parents"), whether or not the joints close a
    /// loop. So the walk from a link no joint enters reaches each link once.
    /// A joint whose child is `world` is refused ([`UrdfError::WorldChild`]).
    fn new(robot: &'a UrdfRobot) -> Result<Self> {
        let mut links = HashMap::new();
        for link in &robot.links {
            if links.insert(link.name.as_str(), link).is_some() {
                return Err(UrdfError::DuplicateLink(link.name.clone()));
            }
        }

        let mut child_to_joint = HashMap::new();
        let mut parent_to_joints: HashMap<&str, Vec<&UrdfJoint>> = HashMap::new();
        for joint in &robot.joints {
            for link in [&joint.parent, &joint.child] {
                if !links.contains_key(link.as_str()) {
                    return Err(UrdfError::undefined_link(link, &joint.name));
                }
            }
            if joint.child == WORLD {
                return Err(UrdfError::WorldChild {
                    joint: joint.name.clone(),
                });
            }
            if child_to_joint.insert(joint.child.as_str(), joint).is_some() {
                return Err(UrdfError::KinematicLoop(format!(
                    "link '{}' has multiple parent joints",
                    joint.child
                )));
            }
            parent_to_joints
                .entry(joint.parent.as_str())
                .or_default()
                .push(joint);
        }

        Ok(Self {
            robot,
            links,
            child_to_joint,
            parent_to_joints,
            geom_names: geom_names(robot),
            output: String::with_capacity(4096),
            indent: 0,
            mesh_assets: Vec::new(),
            mimic_constraints: Vec::new(),
        })
    }

    fn convert(&mut self) -> Result<String> {
        let roots = self.root_links()?;

        // Write MJCF header
        // A robot without a name is written without one, so the model takes
        // MuJoCo 3.5.0's default name, as its importer gives it
        // (`xml_urdf.cc:115-117`: the name is set only where written).
        match &self.robot.name {
            Some(name) => self.write_line(&format!(r#"<mujoco model="{}">"#, escape(name))),
            None => self.write_line("<mujoco>"),
        }
        self.indent += 1;

        self.write_sections();
        self.write_line("");

        // Pre-scan all links for mesh geometries and emit <asset> block
        self.collect_mesh_assets();
        if !self.mesh_assets.is_empty() {
            self.write_line("<asset>");
            self.indent += 1;
            let assets = std::mem::take(&mut self.mesh_assets);
            for ma in &assets {
                let mut attrs =
                    format!(r#"name="{}" file="{}""#, escape(&ma.name), escape(&ma.file));
                if let Some(s) = &ma.scale {
                    write!(attrs, r#" scale="{} {} {}""#, s.x, s.y, s.z).ok();
                }
                self.write_line(&format!("<mesh {attrs}/>"));
            }
            // Restore so convert_geom can reference asset names
            self.mesh_assets = assets;
            self.indent -= 1;
            self.write_line("</asset>");
            self.write_line("");
        }

        // Write worldbody
        self.write_line("<worldbody>");
        self.indent += 1;

        let mut opened = HashSet::new();
        for root in roots {
            if root == WORLD {
                // MuJoCo 3.5.0's importer makes the link named `world` the
                // world body (`AddToTree`, `xml_urdf.cc:727`): its geoms are
                // the world's, the links its joints enter are top-level
                // bodies. Its `<inertial>`, which MuJoCo stores as the world
                // body's mass, inertia, centre and frame, has no MJCF form
                // under `<worldbody>` (D-URDF-WORLD-INERTIAL); its tensor is
                // checked as MuJoCo checks the world body's.
                self.check_world_tensor()?;
                self.write_geoms(WORLD)?;
                opened.insert(WORLD.to_string());
                let children = self.child_links(WORLD);
                self.convert_tree(children, &mut opened)?;
            } else {
                self.convert_tree(vec![(root.to_string(), None)], &mut opened)?;
            }
        }
        // With one parent joint per link, a link no root reaches has parent
        // joints that lead into a loop; MuJoCo refuses it ("URDF body not
        // found").
        if let Some(link) = self.robot.links.iter().find(|l| !opened.contains(&l.name)) {
            return Err(UrdfError::KinematicLoop(format!(
                "link '{}' is reached from no root link: its parent joints lead into a loop",
                link.name
            )));
        }

        self.indent -= 1;
        self.write_line("</worldbody>");

        // Write equality constraints for mimic joints
        for mimic in &self.mimic_constraints {
            self.check_mimic(mimic)?;
        }
        if !self.mimic_constraints.is_empty() {
            self.write_line("");
            self.write_line("<equality>");
            self.indent += 1;
            // Take ownership to avoid borrow conflict with write_line
            let constraints = std::mem::take(&mut self.mimic_constraints);
            for mc in &constraints {
                self.write_line(&format!(
                    r#"<joint joint1="{}" joint2="{}" polycoef="{} {} 0 0 0"/>"#,
                    escape(&mc.follower),
                    escape(&mc.leader),
                    mc.offset,
                    mc.multiplier,
                ));
            }
            self.indent -= 1;
            self.write_line("</equality>");
        }

        self.indent -= 1;
        self.write_line("</mujoco>");

        Ok(std::mem::take(&mut self.output))
    }

    /// Write the model's sections as MuJoCo 3.5.0's importer sets them up.
    /// The `<compiler>`: radians and `eulerseq="XYZ"`, as the importer reads a
    /// URDF (`xml_urdf.cc:111`; it resolves `rpy` as extrinsic x-y-z,
    /// `mjs_resolveOrientation(quat, 0, "XYZ", …)`, `:664-666`, whatever a
    /// compiler's own sequence), `fusestatic`, `discardvisual` and
    /// `strippath` true, its URDF defaults (`xml.cc:299-301`), then the URDF's
    /// own `<compiler>`'s attributes over them, in its order, and its
    /// `<lengthrange>`. The URDF's `<option>` and `<size>` as written; without
    /// an `<option>`, the model has MJCF's defaults, as MuJoCo's: gravity
    /// (0, 0, -9.81), a timestep of 0.002, energy not computed.
    fn write_sections(&mut self) {
        let mut compiler: Vec<(String, String)> = [
            ("angle", "radian"),
            ("eulerseq", "XYZ"),
            ("fusestatic", "true"),
            ("discardvisual", "true"),
            ("strippath", "true"),
        ]
        .map(|(name, value)| (name.to_owned(), value.to_owned()))
        .into();
        let mujoco = &self.robot.mujoco;
        if let Some(section) = &mujoco.compiler {
            // `angle` and `eulerseq` stay as written above, whatever a robot
            // built in code holds (the parser keeps neither).
            let kept = section
                .attributes
                .iter()
                .filter(|(name, _)| name != "angle" && name != "eulerseq");
            for (name, value) in kept {
                match compiler.iter_mut().find(|(written, _)| written == name) {
                    Some(slot) => slot.1.clone_from(value),
                    None => compiler.push((name.clone(), value.clone())),
                }
            }
        }
        let lengthrange = mujoco.compiler.as_ref().and_then(|c| c.child.clone());
        let lines = [
            section_lines("compiler", &compiler, "lengthrange", lengthrange.as_deref()),
            mujoco.option.as_ref().map_or_else(Vec::new, |o| {
                section_lines("option", &o.attributes, "flag", o.child.as_deref())
            }),
            mujoco
                .size
                .as_ref()
                .map_or_else(Vec::new, |s| section_lines("size", &s.attributes, "", None)),
        ];
        for line in lines.iter().flatten() {
            self.write_line(line);
        }
    }

    /// Every link no joint enters, in document order: MuJoCo 3.5.0's importer
    /// writes each one's tree under the world (`xml_urdf.cc`).
    fn root_links(&self) -> Result<Vec<&'a str>> {
        let roots: Vec<&'a str> = self
            .robot
            .links
            .iter()
            .map(|link| link.name.as_str())
            .filter(|name| !self.child_to_joint.contains_key(name))
            .collect();
        // A robot without links is an empty world, as MuJoCo 3.5.0's importer
        // loads it (measured: `<robot name="r"/>`, one body); with links, none
        // a root, the joints make a loop, which the parser refuses before this
        // in a document and this refuses in a robot built in code.
        if roots.is_empty() && !self.robot.links.is_empty() {
            return Err(UrdfError::NoRootLink);
        }
        Ok(roots)
    }

    /// Write a link's `<body>` and its subtree, depth-first, children in
    /// order, without recursing: a URDF chain nests one body per link, and a
    /// recursion per link overflowed a 2-mebibyte caller's stack at 5000 links
    /// (ledger L303). Each link written is added to `opened`. `links` are
    /// the first links, in order, each with the joint entering it.
    fn convert_tree(
        &mut self,
        links: Vec<(String, Option<UrdfJoint>)>,
        opened: &mut HashSet<String>,
    ) -> Result<()> {
        enum Step {
            Open(String, Option<Box<UrdfJoint>>),
            Close,
        }
        let mut stack: Vec<Step> = links
            .into_iter()
            .rev()
            .map(|(link, joint)| Step::Open(link, joint.map(Box::new)))
            .collect();
        while let Some(step) = stack.pop() {
            match step {
                Step::Open(link, joint) => {
                    let children = self.open_link(&link, joint.as_deref())?;
                    opened.insert(link);
                    stack.push(Step::Close);
                    stack.extend(
                        children.into_iter().rev().map(|(child, child_joint)| {
                            Step::Open(child, child_joint.map(Box::new))
                        }),
                    );
                }
                Step::Close => {
                    self.indent -= 1;
                    self.write_line("</body>");
                }
            }
        }
        Ok(())
    }

    /// Write a link's `<body>` opening, its joint, inertial and geoms, and
    /// return its child links with their joints; [`Self::convert_tree`] closes
    /// the body after them.
    fn open_link(
        &mut self,
        link_name: &str,
        parent_joint: Option<&UrdfJoint>,
    ) -> Result<Vec<(String, Option<UrdfJoint>)>> {
        let (name, inertial) = {
            let link = self.link(link_name)?;
            (link.name.clone(), link.inertial)
        };

        // Determine body position from parent joint origin
        let (pos, rpy) = if let Some(joint) = parent_joint {
            (joint.origin.xyz, joint.origin.rpy)
        } else {
            (nalgebra::Vector3::zeros(), nalgebra::Vector3::zeros())
        };

        // Start body element
        let pos_str = format!("{} {} {}", pos.x, pos.y, pos.z);
        let euler_str = format!("{} {} {}", rpy.x, rpy.y, rpy.z);

        let name = escape(&name);
        if written(&rpy) {
            self.write_line(&format!(
                r#"<body name="{name}" pos="{pos_str}" euler="{euler_str}">"#
            ));
        } else {
            self.write_line(&format!(r#"<body name="{name}" pos="{pos_str}">"#));
        }
        self.indent += 1;

        // Write joint if this link has a parent joint (not root)
        if let Some(joint) = parent_joint {
            self.convert_joint(joint)?;
        }

        // Write inertial properties
        if let Some(inertial) = &inertial {
            self.convert_inertial(inertial)?;
        }

        self.write_geoms(link_name)?;
        Ok(self.child_links(link_name))
    }

    /// The link named `link_name`.
    fn link(&self, link_name: &str) -> Result<&'a UrdfLink> {
        self.links
            .get(link_name)
            .copied()
            .ok_or_else(|| UrdfError::undefined_link(link_name, "converter"))
    }

    /// Write a link's geoms in document order, named as [`geom_names`] names
    /// them, each with the colour MuJoCo 3.5.0's importer gives it
    /// (`rgba`): its collisions, and its visuals where the importer keeps
    /// them (`discardvisual` false, `xml_urdf.cc:318-319`), as a visual geom
    /// is made: no contacts, group 1, density 0 (`:538-543`). Under
    /// `discardvisual`, its default for a URDF, the importer makes none.
    fn write_geoms(&mut self, link_name: &str) -> Result<()> {
        let link = self.link(link_name)?;
        let geoms = self.geom_names.get(link_name).cloned().unwrap_or_default();
        for (element, name) in geoms {
            match element {
                UrdfElement::Visual(i) => {
                    let visual = &link.visuals[i];
                    self.convert_geom(
                        &visual.geometry,
                        &visual.origin,
                        name.as_ref(),
                        visual.rgba.as_ref(),
                        VISUAL,
                    )?;
                }
                UrdfElement::Collision(i) => {
                    let collision = &link.collisions[i];
                    self.convert_geom(
                        &collision.geometry,
                        &collision.origin,
                        name.as_ref(),
                        collision.rgba.as_ref(),
                        "",
                    )?;
                }
            }
        }
        Ok(())
    }

    /// The links `link_name`'s joints enter, in order, each with its joint.
    fn child_links(&self, link_name: &str) -> Vec<(String, Option<UrdfJoint>)> {
        self.parent_to_joints
            .get(link_name)
            .map(|joints| {
                joints
                    .iter()
                    .map(|j| (j.child.clone(), Some((*j).clone())))
                    .collect()
            })
            .unwrap_or_default()
    }

    #[allow(clippy::unreachable)] // Planar variant dispatched to convert_planar_joint above
    fn convert_joint(&mut self, joint: &UrdfJoint) -> Result<()> {
        // Planar joints decompose into 2 slides + 1 hinge (3 DOF)
        if joint.joint_type == UrdfJointType::Planar {
            return self.convert_planar_joint(joint);
        }

        let (joint_type, axis_needed) = match joint.joint_type {
            UrdfJointType::Revolute | UrdfJointType::Continuous => ("hinge", true),
            UrdfJointType::Prismatic => ("slide", true),
            UrdfJointType::Fixed => return Ok(()), // Fixed joints = no joint in MJCF
            UrdfJointType::Floating => ("free", false),
            // A ball's axis is (0, 0, 1) whatever is written (MuJoCo's
            // compiler fixes it), so none is written.
            UrdfJointType::Spherical => ("ball", false),
            UrdfJointType::Planar => unreachable!("handled above"),
        };

        let mut attrs = format!(r#"name="{}" type="{}""#, escape(&joint.name), joint_type);

        // Add axis for hinge/slide joints
        if axis_needed {
            let axis = joint.axis;
            write!(attrs, r#" axis="{} {} {}""#, axis.x, axis.y, axis.z).ok();
        }

        // A continuous joint is left unlimited, as URDF defines it
        // (D-URDF-CONTINUOUS-LIMIT: MuJoCo limits it), but where
        // `autolimits="false"`, under which MuJoCo's compiler refuses a range
        // its importer left automatic (`checklimited`), its range is written
        // so: refused, or loaded unlimited, as MuJoCo's is. A floating
        // joint's range is written as the importer reads it
        // (`xml_urdf.cc:498-503`, every joint's but a fixed one's), and limits
        // nothing: MuJoCo's compiler makes a free joint unlimited before any
        // check of its range (`user_objects.cc:2895-2898`).
        let autolimits = self.robot.mujoco.autolimits();
        if let Some(limit) = &joint.limit
            && (matches!(
                joint.joint_type,
                UrdfJointType::Revolute
                    | UrdfJointType::Prismatic
                    | UrdfJointType::Spherical
                    | UrdfJointType::Floating
            ) || (joint.joint_type == UrdfJointType::Continuous && !autolimits))
        {
            attrs.push_str(&limit_attrs(limit, autolimits));
        }
        attrs.push_str(&dynamics_attrs(joint));

        self.write_line(&format!("<joint {attrs}/>"));

        // Collect mimic constraint for post-worldbody emission
        if let Some(mimic) = &joint.mimic {
            self.mimic_constraints.push(MimicEntry {
                follower: joint.name.clone(),
                leader: mimic.joint.clone(),
                multiplier: mimic.multiplier,
                offset: mimic.offset,
            });
        }

        Ok(())
    }

    /// Write a planar joint as MuJoCo 3.5.0's URDF importer builds it
    /// (`xml_urdf.cc`): slides `_TX` and `_TY` along the first two columns of
    /// the rotation taking z to the joint's axis, then a hinge `_RZ` about the
    /// axis. The `<limit>` limits `_TX` alone; the `<dynamics>` goes on all
    /// three.
    fn convert_planar_joint(&mut self, joint: &UrdfJoint) -> Result<()> {
        let (tx, ty) = planar_axes(&joint.axis);
        let normal = joint.axis;
        let autolimits = self.robot.mujoco.autolimits();
        let limit = joint
            .limit
            .as_ref()
            .map(|l| limit_attrs(l, autolimits))
            .unwrap_or_default();
        let dynamics = dynamics_attrs(joint);
        let name = escape(&joint.name);
        self.write_line(&format!(
            r#"<joint name="{name}_TX" type="slide" axis="{} {} {}"{limit}{dynamics}/>"#,
            tx.x, tx.y, tx.z,
        ));
        self.write_line(&format!(
            r#"<joint name="{name}_TY" type="slide" axis="{} {} {}"{dynamics}/>"#,
            ty.x, ty.y, ty.z,
        ));
        self.write_line(&format!(
            r#"<joint name="{name}_RZ" type="hinge" axis="{} {} {}"{dynamics}/>"#,
            normal.x, normal.y, normal.z,
        ));
        Ok(())
    }

    /// A `<mimic>`'s two joints, refused where either is spherical or floating:
    /// the joint equality this crate makes of a mimic (D-URDF-MIMIC) couples
    /// hinge and slide joints only, as MuJoCo 3.5.0's compiler requires of
    /// the MJCF it is written as ("only HINGE and SLIDE joint allowed in
    /// constraint", `user_objects.cc:6034-6041`), as sim-mjcf's builder
    /// does too; this names the URDF's joint.
    fn check_mimic(&self, mimic: &MimicEntry) -> Result<()> {
        for name in [&mimic.follower, &mimic.leader] {
            let kind = match self.robot.joints.iter().find(|joint| &joint.name == name) {
                Some(joint) if joint.joint_type == UrdfJointType::Spherical => "spherical",
                Some(joint) if joint.joint_type == UrdfJointType::Floating => "floating",
                _ => continue,
            };
            return Err(UrdfError::invalid_attribute(
                "joint",
                format!("mimic in joint {}", mimic.follower),
                format!("joint '{name}' is {kind}: a mimic couples hinge and slide joints only"),
            ));
        }
        Ok(())
    }

    /// The link named `world`'s tensor, refused where MuJoCo 3.5.0's compiler
    /// refuses the world body's: its `mjuu_fullInertia` runs on every body
    /// whose `fullinertia` the importer defined, the world's too
    /// (`user_objects.cc:2412-2416`), and the importer defines it for any
    /// tensor but six zeros (`xml_urdf.cc:260-270`); nothing else about the
    /// world's `<inertial>` is checked (measured: a negative mass, a tensor
    /// breaking the triangle load). It is checked before sim-mjcf compiles
    /// the rest, so of two faults the world's is named before a moving body's
    /// and a bad `timestep`, as MuJoCo names it, and before a mesh's missing
    /// file, which MuJoCo names first (ledger L353; all measured).
    fn check_world_tensor(&self) -> Result<()> {
        let Some(inertial) = self
            .links
            .get(WORLD)
            .and_then(|link| link.inertial.as_ref())
        else {
            return Ok(());
        };
        let moments = full_moments(&inertial.inertia);
        if moments.iter().all(|&moment| moment == 0.0) {
            return Ok(());
        }
        sim_mjcf::full_inertia(&moments)
            .map(|_| ())
            .map_err(|message| UrdfError::invalid_inertia(WORLD, message))
    }

    fn convert_inertial(&mut self, inertial: &crate::types::UrdfInertial) -> Result<()> {
        let pos = inertial.origin.xyz;
        let mass = inertial.mass;
        let moments = full_moments(&inertial.inertia);
        // MuJoCo 3.5.0's importer fills the body's `fullinertia` with the six
        // moments, a diagonal tensor's too (`xml_urdf.cc:252-258`), so its
        // compiler takes the tensor's principal axes as the inertial frame,
        // the moments in decreasing order but for moments within 1e-12 of each
        // other (`mjuu_fullInertia`, `mjuu_eig3`).
        let inertia_attr = if moments.iter().all(|&moment| moment == 0.0) {
            // Six zeros are a tensor not given (`:260-270`), as `diaginertia`
            // zeros are in MJCF. MuJoCo keeps the origin's turn as such a
            // body's inertial frame, and its fusion turns the body's position
            // by it (D-FUSESTATIC-INERTIA); this crate's fusion places the
            // mass at its centre, which no frame moves, so none is written.
            r#"diaginertia="0 0 0""#.to_string()
        } else if let Some(([a, b, c], turn)) = turned_frame(&moments, &inertial.origin.rpy) {
            format!(
                r#"diaginertia="{a} {b} {c}" quat="{} {} {} {}""#,
                turn.w, turn.i, turn.j, turn.k
            )
        } else {
            let [ixx, iyy, izz, ixy, ixz, iyz] = moments;
            format!(r#"fullinertia="{ixx} {iyy} {izz} {ixy} {ixz} {iyz}""#)
        };

        // MJCF requires an inertial's `pos` (MuJoCo 3.5.0 refuses one without
        // it), so it is written even at the origin.
        let attrs = format!(
            r#"pos="{} {} {}" mass="{mass}" {inertia_attr}"#,
            pos.x, pos.y, pos.z
        );

        self.write_line(&format!("<inertial {attrs}/>"));

        Ok(())
    }

    /// Write one geom: its shape, its name, its colour where one is set
    /// (each a single-precision value, which sim-mjcf reads back exactly),
    /// `kind`'s attributes (a visual's [`VISUAL`]; a collision has none), its
    /// pose.
    fn convert_geom(
        &mut self,
        geometry: &UrdfGeometry,
        origin: &UrdfOrigin,
        name: Option<&String>,
        rgba: Option<&[f64; 4]>,
        kind: &str,
    ) -> Result<()> {
        let colour = rgba.map_or_else(String::new, |[r, g, b, a]| {
            format!(r#" rgba="{r} {g} {b} {a}""#)
        }) + kind;
        let (geom_type, size) = match geometry {
            UrdfGeometry::Box { size } => {
                // URDF box size is full dimensions, MJCF uses half-extents
                let half = format!("{} {} {}", size.x / 2.0, size.y / 2.0, size.z / 2.0);
                ("box", half)
            }
            UrdfGeometry::Cylinder { radius, length } => {
                // MJCF cylinder: size="radius half_length"
                ("cylinder", format!("{} {}", radius, length / 2.0))
            }
            UrdfGeometry::Sphere { radius } => ("sphere", format!("{radius}")),
            UrdfGeometry::Capsule { radius, length } => {
                // MJCF capsule: size="radius half_length", as MuJoCo's importer halves it
                ("capsule", format!("{} {}", radius, length / 2.0))
            }
            UrdfGeometry::Mesh { filename, scale } => {
                let stripped = mesh_file(filename, self.robot.mujoco.strippath());
                if let Some(asset) = self.mesh_asset(&stripped, *scale) {
                    let mesh_ref = asset.name.clone();
                    // Mesh geoms use type="mesh" mesh="asset_name" (no size)
                    let mut attrs = format!(r#"type="mesh" mesh="{}""#, escape(&mesh_ref));
                    if let Some(n) = name {
                        write!(attrs, r#" name="{}""#, escape(n)).ok();
                    }
                    attrs.push_str(&colour);
                    attrs.push_str(&pose_attrs(origin));
                    self.write_line(&format!("<geom {attrs}/>"));
                }
                return Ok(());
            }
        };

        let mut attrs = format!(r#"type="{geom_type}" size="{size}""#);

        // Add name if present
        if let Some(n) = name {
            write!(attrs, r#" name="{}""#, escape(n)).ok();
        }
        attrs.push_str(&colour);

        attrs.push_str(&pose_attrs(origin));

        self.write_line(&format!("<geom {attrs}/>"));

        Ok(())
    }

    /// Every mesh the geoms name ([`geom_elements`]), as MuJoCo 3.5.0's
    /// importer makes them (`xml_urdf.cc:576-626`), links and their geoms in
    /// document order: one per file stem and scale, the first scale of a stem
    /// named by the stem, each later one by the stem and its index. A geom
    /// names the asset of its own stem and scale, where MuJoCo gives a geom of
    /// a stem's later scale, met a second time, the stem's first mesh
    /// (`:604-616`, `:632`; measured:
    /// scales 1, 2, 2 give the third geom the scale-1 mesh;
    /// `D-URDF-MESH-SCALE`).
    fn collect_mesh_assets(&mut self) {
        let discardvisual = self.robot.mujoco.discardvisual();
        let strippath = self.robot.mujoco.strippath();
        let geoms: Vec<&UrdfGeometry> = self
            .robot
            .links
            .iter()
            .flat_map(|link| {
                geom_elements(link, discardvisual)
                    .into_iter()
                    .map(move |element| match element {
                        UrdfElement::Visual(i) => &link.visuals[i].geometry,
                        UrdfElement::Collision(i) => &link.collisions[i].geometry,
                    })
            })
            .collect();
        for geometry in geoms {
            if let UrdfGeometry::Mesh { filename, scale } = geometry {
                let file = mesh_file(filename, strippath);
                if self.mesh_asset(&file, *scale).is_some() {
                    continue;
                }
                let stem = mesh_stem(&file);
                let index = self
                    .mesh_assets
                    .iter()
                    .filter(|asset| mesh_stem(&asset.file) == stem)
                    .count();
                let name = if index == 0 {
                    stem.to_string()
                } else {
                    format!("{stem}{index}")
                };
                self.mesh_assets.push(MeshAssetEntry {
                    name,
                    file,
                    scale: *scale,
                });
            }
        }
    }

    /// The mesh asset a geom of `file` and `scale` names: the one of its
    /// file's stem and scale.
    fn mesh_asset(&self, file: &str, scale: Option<Vector3<f64>>) -> Option<&MeshAssetEntry> {
        let stem = mesh_stem(file);
        self.mesh_assets
            .iter()
            .find(|asset| mesh_stem(&asset.file) == stem && same_scale(asset.scale, scale))
    }

    fn write_line(&mut self, line: &str) {
        for _ in 0..self.indent {
            self.output.push_str("    ");
        }
        self.output.push_str(line);
        self.output.push('\n');
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    /// A robot without a name loads with MuJoCo 3.5.0's default name,
    /// "MuJoCo Model", as its importer gives it, and `name=""` with the empty
    /// one (measured on these three documents).
    #[test]
    fn a_robot_without_a_name_takes_mujocos_default_name() {
        for (head, want) in [
            ("<robot>", "MuJoCo Model"),
            (r#"<robot name="">"#, ""),
            (r#"<robot name="r">"#, "r"),
        ] {
            let urdf = format!(r#"{head}<link name="base"/></robot>"#);
            let model = crate::load_urdf_model(&urdf).expect("loads");
            assert_eq!(model.name, want, "{urdf}");
        }
    }

    /// Text the URDF holds reaches the converted model as written, a carriage
    /// return too, which written as itself the reader would take for a line
    /// end (measured: a robot named `a&#13;b` is "a\rb" in MuJoCo 3.5.0).
    #[test]
    fn a_carriage_return_in_a_name_is_kept() {
        let model = crate::load_urdf_model(r#"<robot name="a&#13;b"><link name="base"/></robot>"#)
            .expect("loads");
        assert_eq!(model.name, "a\rb");
    }

    /// A robot built in code whose `<compiler>` holds an `angle` or `eulerseq`
    /// is converted in radians and "XYZ", as a parsed one is (the parser keeps
    /// neither: MuJoCo 3.5.0's importer reads a URDF so whatever they say).
    #[test]
    fn a_built_robots_angle_and_eulerseq_are_not_written() {
        let mut robot = UrdfRobot::new("r").with_link(crate::types::UrdfLink::new("base"));
        robot.mujoco.compiler = Some(crate::types::UrdfSection {
            attributes: vec![
                ("angle".to_owned(), "degree".to_owned()),
                ("eulerseq".to_owned(), "zyx".to_owned()),
            ],
            child: None,
        });
        let mjcf = robot_to_mjcf(&robot).expect("converts");
        assert!(
            mjcf.contains(r#"angle="radian" eulerseq="XYZ""#)
                && !mjcf.contains("degree")
                && !mjcf.contains("zyx"),
            "{mjcf}"
        );
    }

    #[test]
    fn test_simple_urdf_to_mjcf() {
        let urdf = r#"<?xml version="1.0"?>
            <robot name="test_arm">
                <link name="base_link">
                    <inertial>
                        <mass value="1.0"/>
                        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
                    </inertial>
                </link>
                <link name="link1">
                    <inertial>
                        <mass value="0.5"/>
                        <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
                    </inertial>
                </link>
                <joint name="joint1" type="revolute">
                    <parent link="base_link"/>
                    <child link="link1"/>
                    <origin xyz="0 0 0.5"/>
                    <axis xyz="0 1 0"/>
                    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
                </joint>
            </robot>
        "#;

        let mjcf = urdf_to_mjcf(urdf).expect("conversion should succeed");

        // Verify structure
        assert!(mjcf.contains(r#"<mujoco model="test_arm">"#));
        assert!(mjcf.contains(r#"<body name="base_link""#));
        assert!(mjcf.contains(r#"<body name="link1""#));
        assert!(mjcf.contains(r#"<joint name="joint1" type="hinge""#));
        assert!(mjcf.contains(r#"axis="0 1 0""#));
        // A limit of two bounds, lower below upper, is left to the
        // compiler's `autolimits`, as MuJoCo's importer leaves it.
        assert!(mjcf.contains(r#" range="-3.14 3.14""#) && !mjcf.contains("limited="));
    }

    /// MJCF requires an inertial's `pos` (MuJoCo 3.5.0 refuses one without
    /// it), so an inertial at the link's origin is written with `pos="0 0 0"`.
    #[test]
    fn an_inertial_at_the_origin_is_written_with_its_pos() {
        let mjcf = urdf_to_mjcf(
            r#"<robot name="r">
                <link name="base">
                    <inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
                </link>
            </robot>"#,
        )
        .expect("converts");
        assert!(mjcf.contains(r#"<inertial pos="0 0 0" mass="1""#), "{mjcf}");
    }

    /// A link whose principal moments break A + B >= C is refused, as
    /// MuJoCo 3.5.0's own URDF import refuses it (measured, with its
    /// words): the tensor (0.1, 0.2, 0.3, 0.02, 0.01, 0.03) has moments
    /// (0.096, 0.195, 0.309).
    #[test]
    fn a_link_whose_moments_break_the_triangle_is_refused() {
        let urdf = r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="2"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0.02" ixz="0.01" iyz="0.03"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#;
        let error = crate::load_urdf_model(urdf).err().map(|e| e.to_string());
        assert!(
            error
                .as_deref()
                .is_some_and(|e| e.contains("inertia must satisfy A + B >= C")),
            "{error:?}"
        );
    }

    /// A mesh collision's origin is written as read, however small, and a -0
    /// keeps its sign, as a primitive's is: MuJoCo 3.5.0's importer reads
    /// each `xyz` and `rpy` written (`xml_urdf.cc:640`, `:648-669`).
    #[test]
    fn a_mesh_geoms_origin_is_written_as_read() {
        let mjcf = urdf_to_mjcf(
            r#"<robot name="r"><link name="base"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="1e-11 -0 0" rpy="0.1 0.2 0.3"/><geometry><mesh filename="part.obj"/></geometry></collision></link></robot>"#,
        )
        .expect("converts");
        assert!(
            mjcf.contains(
                r#"type="mesh" mesh="part" pos="0.00000000001 -0 0" euler="0.1 0.2 0.3""#
            ),
            "{mjcf}"
        );
    }

    #[test]
    fn test_fixed_joints_omitted() {
        let urdf = r#"<?xml version="1.0"?>
            <robot name="fixed_test">
                <link name="base"/>
                <link name="sensor_mount"/>
                <joint name="fixed_joint" type="fixed">
                    <parent link="base"/>
                    <child link="sensor_mount"/>
                    <origin xyz="0 0 0.1"/>
                </joint>
            </robot>
        "#;

        let mjcf = urdf_to_mjcf(urdf).expect("conversion should succeed");

        // Fixed joints should not appear in MJCF
        assert!(!mjcf.contains("fixed_joint"));
        // But the body should still be nested correctly
        assert!(mjcf.contains(r#"<body name="sensor_mount""#));
    }

    #[test]
    fn test_geometry_conversion() {
        let urdf = r#"<?xml version="1.0"?>
            <robot name="geom_test">
                <link name="base">
                    <collision>
                        <geometry>
                            <box size="1.0 2.0 3.0"/>
                        </geometry>
                    </collision>
                    <collision>
                        <geometry>
                            <sphere radius="0.5"/>
                        </geometry>
                        <origin xyz="0 0 1"/>
                    </collision>
                </link>
            </robot>
        "#;

        let mjcf = urdf_to_mjcf(urdf).expect("conversion should succeed");

        // Box should have half-extents
        assert!(mjcf.contains(r#"type="box" size="0.5 1 1.5""#));
        // Sphere
        assert!(mjcf.contains(r#"type="sphere" size="0.5""#));
        assert!(mjcf.contains(r#"pos="0 0 1""#));
    }

    /// Wrap `body` in a two-link robot joined by `joint_xml`.
    fn robot_with_joint(joint_xml: &str) -> String {
        format!(
            r#"<?xml version="1.0"?>
            <robot name="jt">
                <link name="base"/>
                <link name="child"/>
                {joint_xml}
            </robot>"#
        )
    }

    /// Pull the `axis="x y z"` off the joint named `name` in emitted MJCF.
    fn axis_of(mjcf: &str, name: &str) -> nalgebra::Vector3<f64> {
        let needle = format!(r#"name="{name}""#);
        let at = mjcf.find(&needle).expect("joint is present");
        let tail = &mjcf[at..];
        let a = tail.find(r#"axis=""#).expect("joint has an axis") + 6;
        let b = tail[a..].find('"').expect("axis is terminated") + a;
        let nums: Vec<f64> = tail[a..b]
            .split_whitespace()
            .map(|t| t.parse().expect("axis component parses"))
            .collect();
        nalgebra::Vector3::new(nums[0], nums[1], nums[2])
    }

    /// ★★ A planar joint has no MJCF equivalent, so it is decomposed into two
    /// in-plane slides plus a hinge about the plane normal — 3 DOF, matching
    /// URDF's semantics.
    ///
    /// The load-bearing part is that the two slide axes form an ORTHONORMAL
    /// basis of the plane: a decomposition that emitted non-orthogonal or
    /// non-unit axes would still produce three joints and still look right in
    /// a `contains` assertion, while silently skewing every planar mechanism.
    #[test]
    fn planar_joint_decomposes_into_an_orthonormal_three_dof_basis() {
        let mjcf = urdf_to_mjcf(&robot_with_joint(
            r#"<joint name="pj" type="planar">
                 <parent link="base"/><child link="child"/>
                 <axis xyz="0 0 1"/>
               </joint>"#,
        ))
        .expect("conversion should succeed");

        let sx = axis_of(&mjcf, "pj_TX");
        let sy = axis_of(&mjcf, "pj_TY");
        let hinge = axis_of(&mjcf, "pj_RZ");

        assert!(mjcf.contains(r#"name="pj_TX" type="slide""#));
        assert!(mjcf.contains(r#"name="pj_TY" type="slide""#));
        assert!(mjcf.contains(r#"name="pj_RZ" type="hinge""#));

        for (label, v) in [("TX", sx), ("TY", sy), ("RZ", hinge)] {
            assert!(
                (v.norm() - 1.0).abs() < 1e-12,
                "{label} axis must be unit, got norm {}",
                v.norm()
            );
        }
        assert!(sx.dot(&sy).abs() < 1e-12, "slide axes must be orthogonal");
        assert!(
            sx.dot(&hinge).abs() < 1e-12 && sy.dot(&hinge).abs() < 1e-12,
            "both slides must lie in the plane the hinge is normal to"
        );
    }

    /// ★ The in-plane basis turns z onto the normal about z × normal, which
    /// vanishes for a normal along -z; the basis then turns about x
    /// ([`planar_axes`]), and it must be as valid as any other rather than
    /// `NaN`s.
    #[test]
    fn planar_decomposition_survives_a_normal_along_minus_z() {
        let mjcf = urdf_to_mjcf(&robot_with_joint(
            r#"<joint name="pj" type="planar">
                 <parent link="base"/><child link="child"/>
                 <axis xyz="0 0 -1"/>
               </joint>"#,
        ))
        .expect("conversion should succeed");

        let sx = axis_of(&mjcf, "pj_TX");
        let sy = axis_of(&mjcf, "pj_TY");

        assert!(
            sx.iter().all(|c| c.is_finite()) && sy.iter().all(|c| c.is_finite()),
            "crossing the normal with a parallel seed would produce NaNs"
        );
        assert!((sx.norm() - 1.0).abs() < 1e-12);
        assert!(sx.dot(&sy).abs() < 1e-12);
    }

    /// The three joint types no other test reaches map to their MJCF
    /// counterparts. (Revolute is covered by `test_simple_urdf_to_mjcf`, fixed
    /// by `test_fixed_joints_omitted`, planar by the decomposition tests above
    /// — between them all six URDF types are exercised.)
    ///
    /// `floating` additionally takes the `axis_needed = false` path: a `free`
    /// joint carrying an axis attribute would be malformed MJCF.
    #[test]
    fn the_remaining_joint_types_map_to_their_mjcf_counterparts() {
        let prismatic = urdf_to_mjcf(&robot_with_joint(
            r#"<joint name="p" type="prismatic">
                 <parent link="base"/><child link="child"/><axis xyz="1 0 0"/>
               </joint>"#,
        ))
        .expect("prismatic converts");
        assert!(prismatic.contains(r#"name="p" type="slide""#));

        let continuous = urdf_to_mjcf(&robot_with_joint(
            r#"<joint name="c" type="continuous">
                 <parent link="base"/><child link="child"/><axis xyz="0 0 1"/>
               </joint>"#,
        ))
        .expect("continuous converts");
        assert!(continuous.contains(r#"name="c" type="hinge""#));

        let floating = urdf_to_mjcf(&robot_with_joint(
            r#"<joint name="f" type="floating">
                 <parent link="base"/><child link="child"/>
               </joint>"#,
        ))
        .expect("floating converts");
        assert!(floating.contains(r#"name="f" type="free""#));
        let at = floating.find(r#"name="f""#).expect("joint present");
        let decl_end = floating[at..].find("/>").expect("joint closes") + at;
        assert!(
            !floating[at..decl_end].contains("axis="),
            "a free joint must not carry an axis attribute"
        );
    }

    /// ★ A `continuous` joint is unbounded, so a `<limit>` on it must NOT
    /// become an MJCF range, but under `autolimits="false"` (where MuJoCo
    /// refuses it, D-URDF-CONTINUOUS-LIMIT). Emitting a range here would
    /// silently clamp a joint URDF says spins freely.
    #[test]
    fn a_limit_on_a_continuous_joint_is_not_emitted_as_a_range() {
        let mjcf = urdf_to_mjcf(&robot_with_joint(
            r#"<joint name="c" type="continuous">
                 <parent link="base"/><child link="child"/><axis xyz="0 0 1"/>
                 <limit lower="-1" upper="1" effort="10" velocity="1"/>
               </joint>"#,
        ))
        .expect("conversion should succeed");

        assert!(mjcf.contains(r#"name="c" type="hinge""#));
        // No range at all: an automatic one, written without `limited`,
        // would be limited, clamping free rotation.
        assert!(
            !mjcf.contains("range=") && !mjcf.contains("limited="),
            "continuous joints are unbounded; a range would clamp free rotation: {mjcf}"
        );
    }

    /// Damping and friction are emitted only when positive — the zero default
    /// must not litter every joint with `damping="0"`.
    #[test]
    fn joint_dynamics_are_emitted_only_when_positive() {
        let with = urdf_to_mjcf(&robot_with_joint(
            r#"<joint name="d" type="revolute">
                 <parent link="base"/><child link="child"/><axis xyz="0 0 1"/>
                 <dynamics damping="0.7" friction="0.25"/>
               </joint>"#,
        ))
        .expect("conversion should succeed");
        assert!(with.contains(r#"damping="0.7""#));
        assert!(with.contains(r#"frictionloss="0.25""#));

        let without = urdf_to_mjcf(&robot_with_joint(
            r#"<joint name="d" type="revolute">
                 <parent link="base"/><child link="child"/><axis xyz="0 0 1"/>
                 <dynamics damping="0" friction="0"/>
               </joint>"#,
        ))
        .expect("conversion should succeed");
        assert!(!without.contains("damping="));
        assert!(!without.contains("frictionloss="));
    }

    /// A URDF cylinder is (radius, full length); MJCF wants (radius,
    /// HALF length). Getting this wrong doubles every cylinder.
    #[test]
    fn cylinder_length_is_halved_for_mjcf() {
        let urdf = r#"<?xml version="1.0"?>
            <robot name="cyl">
                <link name="base">
                    <collision><geometry>
                        <cylinder radius="0.25" length="2.0"/>
                    </geometry></collision>
                </link>
            </robot>"#;

        let mjcf = urdf_to_mjcf(urdf).expect("conversion should succeed");
        assert!(
            mjcf.contains(r#"type="cylinder" size="0.25 1""#),
            "MJCF cylinder size is (radius, half-length); got: {mjcf}"
        );
    }

    /// A visual geom is not written: MuJoCo 3.5.0's importer makes none under
    /// URDF's `discardvisual` (measured: a link with one visual sphere has no
    /// geom). `b8a6ffba` wrote it with `contype` 0, for `discardvisual` to drop.
    #[test]
    fn a_visual_geom_is_not_written() {
        let urdf = r#"<?xml version="1.0"?>
            <robot name="vis">
                <link name="base">
                    <visual><geometry><sphere radius="0.5"/></geometry></visual>
                </link>
            </robot>"#;

        let mjcf = urdf_to_mjcf(urdf).expect("conversion should succeed");
        assert!(!mjcf.contains("<geom"), "{mjcf}");
    }

    /// `package://` and `file://` URIs are resolver-specific; MJCF wants a
    /// path relative to the mesh directory.
    #[test]
    fn mesh_uri_schemes_are_stripped_to_a_relative_path() {
        assert_eq!(
            strip_package_prefix("package://my_robot/meshes/link.stl"),
            "meshes/link.stl",
            "the package NAME is dropped along with the scheme"
        );
        assert_eq!(
            strip_package_prefix("file:///abs/link.stl"),
            "/abs/link.stl"
        );
        assert_eq!(strip_package_prefix("meshes/link.stl"), "meshes/link.stl");
        // A package URI with no path after the name has nothing to strip.
        assert_eq!(strip_package_prefix("package://only_pkg"), "only_pkg");
    }

    /// Every link being some joint's child means the kinematic tree has no
    /// root — a cycle. That is rejected rather than silently picking a link.
    #[test]
    fn a_robot_with_no_root_link_is_rejected() {
        let urdf = r#"<?xml version="1.0"?>
            <robot name="cycle">
                <link name="a"/>
                <link name="b"/>
                <joint name="j1" type="fixed">
                    <parent link="a"/><child link="b"/>
                </joint>
                <joint name="j2" type="fixed">
                    <parent link="b"/><child link="a"/>
                </joint>
            </robot>"#;

        assert!(
            urdf_to_mjcf(urdf).is_err(),
            "a cyclic tree has no root link and must not convert"
        );
    }

    /// A sphere collision named `name` (none if `None`) of radius `radius`.
    fn collision(name: Option<&str>, radius: f64) -> String {
        let name = name.map_or(String::new(), |n| format!(r#" name="{n}""#));
        format!(r#"<collision{name}><geometry><sphere radius="{radius}"/></geometry></collision>"#)
    }

    /// A link with an inertial of mass 1 and `inner`.
    fn massive_link(name: &str, inner: &str) -> String {
        format!(
            r#"<link name="{name}"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial>{inner}</link>"#
        )
    }

    /// A revolute joint `name` from `parent` to `child`.
    fn hinge(name: &str, parent: &str, child: &str) -> String {
        format!(
            r#"<joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{child}"/><origin xyz="0 0 1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>"#
        )
    }

    /// A collision name repeated in the robot is kept by the first link in the
    /// document that has it and the later geom is unnamed, as MuJoCo 3.5.0's
    /// importer names them (measured: the child link `arm`, written first,
    /// keeps `c`; geom names `[None, 'c']` in body order, its radius-0.2 geom
    /// named). `b8a6ffba` kept `c` on both.
    #[test]
    fn a_repeated_collision_name_is_kept_by_the_first_link_written() {
        let urdf = format!(
            r#"<robot name="r">{}{}{}</robot>"#,
            massive_link("arm", &collision(Some("c"), 0.2)),
            massive_link("base", &collision(Some("c"), 0.3)),
            hinge("j", "base", "arm"),
        );
        let mjcf = urdf_to_mjcf(&urdf).expect("converts");
        assert_eq!(mjcf.matches(r#"name="c""#).count(), 1, "{mjcf}");
        assert!(
            mjcf.contains(r#"type="sphere" size="0.2" name="c""#),
            "{mjcf}"
        );
        let model = sim_mjcf::load_model(&mjcf).expect("loads");
        let named: Vec<_> = model.geom_name.iter().map(Option::as_deref).collect();
        assert_eq!(named, [None, Some("c")]);
        assert_eq!(model.geom_size[1].x.to_bits(), 0.2_f64.to_bits());
    }

    /// A visual's name never meets a collision's: MuJoCo 3.5.0's importer
    /// discards visuals under URDF's `discardvisual` before naming them
    /// (measured: a collision and a visual both named `c` load, geom names
    /// `['c', 'd']`).
    #[test]
    fn a_visual_name_never_meets_a_collisions() {
        let urdf = format!(
            r#"<robot name="r">{}{}{}</robot>"#,
            massive_link(
                "base",
                &format!(
                    r#"{}<visual name="c"><geometry><sphere radius="0.1"/></geometry></visual>"#,
                    collision(Some("c"), 0.1)
                )
            ),
            massive_link("arm", &collision(Some("d"), 0.1)),
            hinge("j", "base", "arm"),
        );
        let mjcf = urdf_to_mjcf(&urdf).expect("converts");
        assert_eq!(mjcf.matches(r#"name="c""#).count(), 1, "{mjcf}");
        let model = sim_mjcf::load_model(&mjcf).expect("loads");
        let named: Vec<_> = model.geom_name.iter().map(Option::as_deref).collect();
        assert_eq!(named, [Some("c"), Some("d")]);
    }

    /// The link named `world` is the world body, as MuJoCo 3.5.0's importer
    /// makes it (measured: bodies `['world', 'base']`, geoms `['floor', 'b']`
    /// on bodies 0 and 1). `b8a6ffba` wrote it as a body named
    /// `world`, which only fusion folded into the world. Its `<inertial>` is
    /// not stored, where MuJoCo stores it as the world body's mass, inertia,
    /// centre and frame (measured: `body_mass[0]` 1; `D-URDF-WORLD-INERTIAL`);
    /// its tensor is checked (`check_world_tensor`).
    #[test]
    fn the_link_named_world_is_the_world_body() {
        let urdf = format!(
            r#"<robot name="r">{}{}{}</robot>"#,
            massive_link("world", &collision(Some("floor"), 0.1)),
            massive_link("base", &collision(Some("b"), 0.1)),
            hinge("j0", "world", "base"),
        );
        let mjcf = urdf_to_mjcf(&urdf).expect("converts");
        assert!(!mjcf.contains(r#"<body name="world""#), "{mjcf}");
        let model = sim_mjcf::load_model(&mjcf).expect("loads");
        assert_eq!(model.body_mass[0].to_bits(), 0.0_f64.to_bits());
        let bodies: Vec<_> = model.body_name.iter().map(Option::as_deref).collect();
        assert_eq!(bodies, [Some("world"), Some("base")]);
        let geoms: Vec<_> = model.geom_name.iter().map(Option::as_deref).collect();
        assert_eq!(geoms, [Some("floor"), Some("b")]);
        assert_eq!(model.geom_body, [0, 1]);
    }

    /// A joint whose child is `world` is refused: MuJoCo 3.5.0 refuses a
    /// moving one ("joint found in world body", measured) and, for a fixed
    /// one, moves the world body to the joint's origin (measured: `body_pos`
    /// (0, 0, 3)), which this crate refuses too (`D-URDF-WORLD-CHILD`).
    #[test]
    fn a_joint_entering_world_is_refused() {
        for kind in ["revolute", "fixed"] {
            let urdf = format!(
                r#"<robot name="r">{}<link name="world"/><joint name="j" type="{kind}"><parent link="base"/><child link="world"/><origin xyz="0 0 3"/><axis xyz="0 0 1"/></joint></robot>"#,
                massive_link("base", "")
            );
            let err = urdf_to_mjcf(&urdf).expect_err(kind);
            assert!(
                matches!(&err, UrdfError::WorldChild { joint } if joint == "j"),
                "{err}"
            );
            assert!(
                err.to_string().contains("joint found in world body"),
                "{err}"
            );
        }
    }

    /// Under `discardvisual="false"` a visual is a geom, made with its mesh in
    /// document order among the link's collisions, as MuJoCo 3.5.0's importer
    /// makes them (measured on this document, a visual and a collision of one file:
    /// meshes `tet` at scale 1 and `tet1` at scale 2, geoms `v` on `tet` with
    /// no contacts in group 1 and `c` on `tet1`).
    #[test]
    fn a_kept_visual_and_its_mesh_are_made_in_document_order() {
        let mjcf = urdf_to_mjcf(concat!(
            r#"<robot name="r"><mujoco><compiler discardvisual="false"/></mujoco>"#,
            r#"<link name="base"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial>"#,
            r#"<visual name="v"><geometry><mesh filename="tet.stl"/></geometry></visual>"#,
            r#"<collision name="c"><geometry><mesh filename="tet.stl" scale="2 2 2"/></geometry></collision>"#,
            r#"</link></robot>"#,
        ))
        .expect("converts");
        let at = |text: &str| mjcf.find(text).expect(text);
        assert!(
            at(r#"<mesh name="tet" file="tet.stl"/>"#)
                < at(r#"<mesh name="tet1" file="tet.stl" scale="2 2 2"/>"#)
        );
        assert!(
            at(
                r#"<geom type="mesh" mesh="tet" name="v" contype="0" conaffinity="0" group="1" density="0"/>"#
            ) < at(r#"<geom type="mesh" mesh="tet1" name="c"/>"#)
        );
    }

    /// A mesh's `package://` prefix is stripped under `strippath`, the
    /// importer's default, and kept under `strippath="false"`, where MuJoCo
    /// 3.5.0 keeps the path as written (measured with a mesh file: a
    /// `package://` or `file://` path refused, "Error opening file", as the
    /// written path is here; a relative one loads).
    #[test]
    fn a_mesh_paths_prefix_is_stripped_only_under_strippath() {
        let urdf = |compiler: &str| {
            format!(
                r#"<robot name="r">{compiler}<link name="base"><collision><geometry><mesh filename="package://pkg/meshes/tet.stl"/></geometry></collision></link></robot>"#
            )
        };
        let default = urdf_to_mjcf(&urdf("")).expect("converts");
        assert!(default.contains(r#"file="meshes/tet.stl""#), "{default}");
        let kept = urdf_to_mjcf(&urdf(r#"<mujoco><compiler strippath="false"/></mujoco>"#))
            .expect("converts");
        assert!(
            kept.contains(r#"file="package://pkg/meshes/tet.stl""#),
            "{kept}"
        );
    }

    /// Mesh assets are named as MuJoCo 3.5.0's importer names them: by the
    /// file's stem, a later scale of a stem by the stem and its index, and a
    /// geom of a stem and scale met before names that asset (measured: one
    /// file at scales 1, 2, 1, 3 → meshes `['part', 'part1', 'part2']`, geoms'
    /// meshes 0, 1, 0, 2; two files under one collision name → meshes
    /// `['a', 'b']`, which `b8a6ffba` named both `c` and refused). Here two
    /// files give meshes by their stems, `a` and `b`, a `\` separating a
    /// path's parts too (`mjuu_strippath`).
    #[test]
    fn meshes_are_named_by_file_stem_and_scale() {
        let mesh = |name: &str, file: &str, scale: &str| {
            format!(
                r#"<collision name="{name}"><geometry><mesh filename="{file}"{scale}/></geometry></collision>"#
            )
        };
        let urdf = format!(
            r#"<robot name="r">{}{}{}</robot>"#,
            massive_link("base", &mesh("c", "package://p/meshes/part.stl", "")),
            massive_link(
                "arm",
                &format!(
                    "{}{}{}{}{}",
                    mesh("d", "meshes/part.stl", r#" scale="2 2 2""#),
                    mesh("e", "meshes/part.stl", ""),
                    mesh("f", "part.stl", r#" scale="3 3 3""#),
                    mesh("g", "meshes/a.stl", ""),
                    mesh("h", "meshes\\b.obj", "")
                )
            ),
            hinge("j", "base", "arm"),
        );
        let mjcf = urdf_to_mjcf(&urdf).expect("converts");
        for asset in [
            r#"<mesh name="part" file="meshes/part.stl"/>"#,
            r#"<mesh name="part1" file="meshes/part.stl" scale="2 2 2"/>"#,
            r#"<mesh name="part2" file="part.stl" scale="3 3 3"/>"#,
            r#"<mesh name="a" file="meshes/a.stl"/>"#,
            r#"<mesh name="b" file="meshes\b.obj"/>"#,
        ] {
            assert!(mjcf.contains(asset), "{asset} in {mjcf}");
        }
        for (geom, asset) in [("c", "part"), ("d", "part1"), ("e", "part"), ("f", "part2")] {
            assert!(
                mjcf.contains(&format!(r#"mesh="{asset}" name="{geom}""#)),
                "geom {geom} names {asset} in {mjcf}"
            );
        }
    }

    /// A `<mimic>` becomes a joint equality, the follower's position the
    /// leader's times the multiplier plus the offset; MuJoCo 3.5.0's
    /// importer does not read `<mimic>` (measured: `neq` 0;
    /// `D-URDF-MIMIC`).
    #[test]
    fn a_mimic_becomes_a_joint_equality() {
        let urdf = format!(
            r#"<robot name="r"><link name="base"/>{}{}{}<joint name="follower" type="revolute"><parent link="base"/><child link="b"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/><mimic joint="j" multiplier="2" offset="0.1"/></joint></robot>"#,
            massive_link("a", ""),
            massive_link("b", ""),
            hinge("j", "base", "a"),
        );
        let model = sim_mjcf::load_model(&urdf_to_mjcf(&urdf).expect("converts")).expect("loads");
        assert_eq!(model.neq, 1);
        assert_eq!(
            model.eq_data[0][..2]
                .iter()
                .map(|x| x.to_bits())
                .collect::<Vec<_>>(),
            [0.1_f64.to_bits(), 2.0_f64.to_bits()]
        );
    }

    /// The joint named `name`'s axis in `model`, as bits.
    fn axis_bits(model: &sim_core::Model, name: &str) -> [u64; 3] {
        let j = model.jnt_name_to_id[name];
        let a = model.jnt_axis[j];
        [a.x, a.y, a.z].map(f64::to_bits)
    }

    /// A joint without an `<axis>` turns about (1, 0, 0), as URDF and
    /// MuJoCo 3.5.0's importer define it (measured: `jnt_axis` [1, 0, 0] for
    /// a revolute, a prismatic and a continuous joint); `b8a6ffba` took z.
    #[test]
    fn a_joint_without_an_axis_turns_about_x() {
        for kind in ["revolute", "prismatic", "continuous"] {
            let urdf = format!(
                r#"<robot name="r"><link name="base"/>{}<joint name="j" type="{kind}"><parent link="base"/><child link="a"/><limit lower="-1" upper="1"/></joint></robot>"#,
                massive_link("a", "")
            );
            let model = sim_mjcf::load_model(&urdf_to_mjcf(&urdf).expect("converts")).expect(kind);
            assert_eq!(
                axis_bits(&model, "j"),
                [1.0, 0.0, 0.0].map(f64::to_bits),
                "{kind}"
            );
        }
    }

    /// An axis is kept as written, so the MJCF loader refuses one too small,
    /// as MuJoCo 3.5.0 refuses it (measured: "axis too small in joint", a
    /// revolute's and a planar's), and normalises the rest as MuJoCo does;
    /// `b8a6ffba` made a zero axis z.
    #[test]
    fn an_axis_is_kept_as_written() {
        for kind in ["revolute", "planar"] {
            let urdf = format!(
                r#"<robot name="r"><link name="base"/>{}<joint name="j" type="{kind}"><parent link="base"/><child link="a"/><axis xyz="0 0 0"/></joint></robot>"#,
                massive_link("a", "")
            );
            let err =
                sim_mjcf::load_model(&urdf_to_mjcf(&urdf).expect("converts")).expect_err(kind);
            assert!(err.to_string().contains("axis too small"), "{kind}: {err}");
        }
    }

    /// A planar joint's axes are MuJoCo 3.5.0's, from the axis as written
    /// (`mjuu_z2quat`; each case's `jnt_axis` measured, as bits): an axis
    /// written short of unit length within about 1e-7 of z gives `_TY`
    /// (0, 1, 0), where `b8a6ffba`, from the unit axis, gave
    /// (0, 0.99999999999987, -5e-7) (ledger L316); an axis whose product
    /// with z is within mjEPS of unit length keeps it unscaled
    /// (`mjuu_normvec`), and so do MuJoCo's slide axes. Every row is compared
    /// bit for bit on macOS arm64 (the rows through `sin` and `cos` round by
    /// the platform's library), to 1e-15 elsewhere.
    // MuJoCo's values as bit patterns, compared exactly.
    #[allow(clippy::unreadable_literal)]
    #[test]
    fn a_planar_joints_axes_are_mujocos() {
        const ROWS: &[(&str, [[u64; 3]; 3])] = &[
            (
                "",
                [
                    [0x3cb0000000000000, 0, 0xbff0000000000000],
                    [0, 0x3ff0000000000000, 0],
                    [0x3ff0000000000000, 0, 0],
                ],
            ),
            (
                "0 0 1",
                [
                    [0x3ff0000000000000, 0, 0],
                    [0, 0x3ff0000000000000, 0],
                    [0, 0, 0x3ff0000000000000],
                ],
            ),
            (
                "0 5e-8 0.1",
                [
                    [0x3ff0000000000000, 0, 0],
                    [0, 0x3ff0000000000000, 0],
                    [0, 0x3ea0c6f7a0b5eb3e, 0x3feffffffffffb9a],
                ],
            ),
            (
                "3 0 0",
                [
                    [0x3cb0000000000000, 0, 0xbff0000000000000],
                    [0, 0x3ff0000000000000, 0],
                    [0x3ff0000000000000, 0, 0],
                ],
            ),
            (
                "1.000000000000005 0 0",
                [
                    [0xbcf6800000000000, 0, 0xbff0000000000017],
                    [0, 0x3ff0000000000018, 0],
                    [0x3ff0000000000017, 0, 0],
                ],
            ),
            (
                "0 0 -1",
                [
                    [0x3ff0000000000000, 0, 0],
                    [0, 0xbff0000000000000, 0x3ca1a62633145c07],
                    [0, 0, 0xbff0000000000000],
                ],
            ),
            (
                "0.6000000000000003 0.8 0",
                [
                    [0x3fe47ae147ae147a, 0xbfdeb851eb851ebc, 0xbfe3333333333336],
                    [0xbfdeb851eb851ebc, 0x3fd70a3d70a3d70e, 0xbfe999999999999a],
                    [0x3fe3333333333336, 0x3fe999999999999a, 0],
                ],
            ),
        ];
        let here = cfg!(all(target_os = "macos", target_arch = "aarch64"));
        for &(axis, mujoco) in ROWS {
            let axis_element = if axis.is_empty() {
                String::new()
            } else {
                format!(r#"<axis xyz="{axis}"/>"#)
            };
            let urdf = format!(
                r#"<robot name="r"><link name="base"/>{}<joint name="pj" type="planar"><parent link="base"/><child link="a"/>{axis_element}</joint></robot>"#,
                massive_link("a", "")
            );
            let model = sim_mjcf::load_model(&urdf_to_mjcf(&urdf).expect("converts")).expect(axis);
            for (joint, want) in ["pj_TX", "pj_TY", "pj_RZ"].into_iter().zip(mujoco) {
                let got = axis_bits(&model, joint);
                let exact = got == want;
                let close = got.iter().zip(&want).all(|(&g, &w)| {
                    let (g, w) = (f64::from_bits(g), f64::from_bits(w));
                    (g - w).abs() <= 1e-15
                });
                assert!(
                    if here { exact } else { close },
                    "axis {axis:?} {joint}: {:?}, MuJoCo {:?}",
                    got.map(f64::from_bits),
                    want.map(f64::from_bits)
                );
            }
        }
    }

    /// A `<limit effort>` is parsed and not mapped: MuJoCo 3.5.0's importer
    /// makes it the joint's actuator force range, ±|effort| where above 0
    /// (measured: effort 5 gives `jnt_actfrcrange` (-5, 5)), which this
    /// crate's model does not have (`D-URDF-EFFORT`).
    #[test]
    fn an_effort_is_parsed_and_not_mapped() {
        let urdf = format!(
            r#"<robot name="r"><link name="base"/>{}<joint name="j" type="revolute"><parent link="base"/><child link="a"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="5" velocity="1"/></joint></robot>"#,
            massive_link("a", "")
        );
        let robot = crate::parser::parse_urdf_str(&urdf).expect("parses");
        let limit = robot.joints[0].limit.as_ref().expect("a limit");
        assert_eq!(limit.effort.to_bits(), 5.0_f64.to_bits());
        let mjcf = urdf_to_mjcf(&urdf).expect("converts");
        assert!(
            !mjcf.contains("actuatorfrc") && !mjcf.contains("5\""),
            "{mjcf}"
        );
    }

    /// A geom of a mesh file's second scale, met again, names that scale's
    /// asset, where MuJoCo 3.5.0's importer gives it the file's first mesh
    /// (measured: scales 1, 2, 2 give meshes `part`, `part1` and
    /// `geom_dataid` [0, 1, 0]): its `meshname` stays the stem when it finds the
    /// scale among the stem's meshes past the first (`xml_urdf.cc:604-616`,
    /// `:632`), so it scales the geom by the first scale it read
    /// (`D-URDF-MESH-SCALE`, wrong-value).
    #[test]
    fn a_meshs_repeated_later_scale_names_its_own_asset() {
        let mesh = |name: &str, scale: &str| {
            format!(
                r#"<collision name="{name}"><geometry><mesh filename="meshes/part.stl"{scale}/></geometry></collision>"#
            )
        };
        let urdf = format!(
            r#"<robot name="r">{}{}{}</robot>"#,
            massive_link("base", &mesh("c", "")),
            massive_link(
                "arm",
                &format!(
                    "{}{}",
                    mesh("d", r#" scale="2 2 2""#),
                    mesh("e", r#" scale="2 2 2""#)
                )
            ),
            hinge("j", "base", "arm"),
        );
        let mjcf = urdf_to_mjcf(&urdf).expect("converts");
        assert!(mjcf.contains(r#"mesh="part1" name="d""#), "{mjcf}");
        assert!(mjcf.contains(r#"mesh="part1" name="e""#), "{mjcf}");
    }

    /// An empty collision name is no name: MuJoCo 3.5.0's importer leaves two
    /// `name=""` collisions unnamed, without a warning (measured: geoms
    /// `[None, None]`); `b8a6ffba` wrote the first `name=""`.
    #[test]
    fn an_empty_collision_name_is_no_name() {
        let urdf = format!(
            r#"<robot name="r">{}</robot>"#,
            massive_link(
                "base",
                &format!("{}{}", collision(Some(""), 0.1), collision(Some(""), 0.2))
            ),
        );
        let mjcf = urdf_to_mjcf(&urdf).expect("converts");
        assert!(!mjcf.contains(r#"name="""#), "{mjcf}");
    }

    /// Every name and path the converter writes reaches sim-mjcf's reader as
    /// the URDF held it: the model's, a body's, a joint's, a planar joint's
    /// three, a mimic's two joints, a mesh's name and file, and a geom's name
    /// and mesh. Each ends in `"&lt;` as the reader gives it, which written
    /// unescaped would end its attribute at the `"`.
    #[test]
    fn names_with_markup_characters_round_trip() {
        // A name as the URDF writes it, and as the reader gives it.
        let written = |stem: &str| format!("{stem}&quot;&amp;lt;");
        let read = |stem: &str| format!("{stem}\u{22}&lt;");
        let mesh = format!(
            r#"<collision name="{}"><geometry><mesh filename="{}.stl"/></geometry></collision>"#,
            written("gm"),
            written("m"),
        );
        let urdf = format!(
            r#"<robot name="{}">{}{}{}{}{}<joint name="{}" type="planar"><parent link="{}"/><child link="{}"/><axis xyz="0 0 1"/></joint><joint name="{}" type="revolute"><parent link="{}"/><child link="{}"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/><mimic joint="{}"/></joint></robot>"#,
            written("r"),
            massive_link(&written("b"), ""),
            massive_link(&written("a"), &mesh),
            massive_link(&written("c"), &collision(Some(&written("gs")), 0.1)),
            massive_link(&written("d"), ""),
            hinge(&written("j"), &written("b"), &written("a")),
            written("p"),
            written("a"),
            written("c"),
            written("f"),
            written("b"),
            written("d"),
            written("j"),
        );
        let model =
            sim_mjcf::parse_mjcf_str(&urdf_to_mjcf(&urdf).expect("converts")).expect("parses");
        let (mut bodies, mut joints, mut geoms) = (Vec::new(), Vec::new(), Vec::new());
        let mut stack: Vec<_> = model.worldbody.children.iter().collect();
        while let Some(body) = stack.pop() {
            bodies.push(body.name.clone());
            joints.extend(body.joints.iter().map(|joint| joint.name.clone()));
            geoms.extend(
                body.geoms
                    .iter()
                    .map(|geom| (geom.name.clone(), geom.mesh.clone())),
            );
            stack.extend(&body.children);
        }
        for list in [&mut bodies, &mut joints] {
            list.sort();
        }
        geoms.sort();
        assert_eq!(model.name, read("r"));
        assert_eq!(
            model
                .meshes
                .iter()
                .map(|mesh| (mesh.name.clone(), mesh.file.clone()))
                .collect::<Vec<_>>(),
            [(read("m"), Some(format!("{}.stl", read("m"))))]
        );
        assert_eq!(bodies, ["a", "b", "c", "d"].map(read));
        let planar = read("p");
        let mut expected = vec![read("f"), read("j")];
        expected.extend(["_RZ", "_TX", "_TY"].map(|axis| format!("{planar}{axis}")));
        expected.sort();
        assert_eq!(joints, expected);
        assert_eq!(
            geoms,
            [
                (Some(read("gm")), Some(read("m"))),
                (Some(read("gs")), None)
            ]
        );
        assert_eq!(
            model
                .equality
                .joints
                .iter()
                .map(|equality| (equality.joint1.clone(), equality.joint2.clone()))
                .collect::<Vec<_>>(),
            [(read("f"), Some(read("j")))]
        );
    }
}
