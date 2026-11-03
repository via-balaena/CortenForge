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
//! | `<link>` | `<body>` |
//! | `<joint>` | `<joint>` (inside child body) |
//! | `<inertial>` | `<inertial>` |
//! | `<collision>` | `<geom>` |
//! | `<visual>` | `<geom class="visual">` (optional) |
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
use nalgebra::Vector3;

use crate::types::{
    UrdfGeometry, UrdfJoint, UrdfJointLimit, UrdfJointType, UrdfLink, UrdfOrigin, UrdfRobot,
};

/// The link name MuJoCo 3.5.0's importer reads as the world body
/// (`xml_urdf.cc:727`).
const WORLD: &str = "world";

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
/// (`xml_urdf.cc`): one bound written, or a lower above the upper, leaves the
/// joint unlimited; otherwise its limit is automatic, which limits it only
/// where lower < upper. A bound not written is 0.
fn limit_attrs(limit: &UrdfJointLimit) -> String {
    let (lower, upper) = (limit.lower.unwrap_or(0.0), limit.upper.unwrap_or(0.0));
    let limited = limit.lower.is_some() && limit.upper.is_some() && lower < upper;
    format!(r#" limited="{limited}" range="{lower} {upper}""#)
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
/// from the axis as written (`xml_urdf.cc:445-446`): the first two columns
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
/// `mjuu_stripext`, `xml_urdf.cc:592-593`).
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

/// Each link's collision geoms' names, as MuJoCo 3.5.0's importer names
/// them (`xml_urdf.cc:345-358`): links in document order, a link's
/// collisions in order, a name kept only where no earlier collision has it,
/// a repeat left unnamed with MuJoCo's warning. Its visual geoms, which it
/// discards before naming under URDF's `discardvisual`, take none.
fn collision_names(robot: &UrdfRobot) -> HashMap<&str, Vec<Option<String>>> {
    let mut seen = HashSet::new();
    let mut names = HashMap::new();
    for link in &robot.links {
        let kept = link
            .collisions
            .iter()
            .map(|collision| {
                let name = collision.name.as_ref()?;
                if seen.insert(name.as_str()) {
                    return Some(name.clone());
                }
                tracing::warn!(
                    "Geom with duplicate name '{name}' encountered in URDF, creating an unnamed geom."
                );
                None
            })
            .collect();
        names.insert(link.name.as_str(), kept);
    }
    names
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
    /// Each link's collision geoms' names ([`collision_names`]).
    collision_names: HashMap<&'a str, Vec<Option<String>>>,
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
            collision_names: collision_names(robot),
            output: String::with_capacity(4096),
            indent: 0,
            mesh_assets: Vec::new(),
            mimic_constraints: Vec::new(),
        })
    }

    fn convert(&mut self) -> Result<String> {
        let roots = self.root_links()?;

        // Write MJCF header
        self.write_line(&format!(r#"<mujoco model="{}">"#, self.robot.name));
        self.indent += 1;

        // Write options
        self.write_line(r#"<option gravity="0 0 -9.81" timestep="0.002">"#);
        self.indent += 1;
        self.write_line(r#"<flag energy="enable"/>"#);
        self.indent -= 1;
        self.write_line(r"</option>");
        self.write_line("");

        // Write compiler settings for URDF compatibility
        self.write_line(r#"<compiler angle="radian" eulerseq="xyz" fusestatic="true" discardvisual="true" strippath="true"/>"#);
        self.write_line("");

        // Pre-scan all links for mesh geometries and emit <asset> block
        self.collect_mesh_assets();
        if !self.mesh_assets.is_empty() {
            self.write_line("<asset>");
            self.indent += 1;
            let assets = std::mem::take(&mut self.mesh_assets);
            for ma in &assets {
                let mut attrs = format!(r#"name="{}" file="{}""#, ma.name, ma.file);
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
                // body's mass, has no MJCF form under `<worldbody>`.
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
        if !self.mimic_constraints.is_empty() {
            self.write_line("");
            self.write_line("<equality>");
            self.indent += 1;
            // Take ownership to avoid borrow conflict with write_line
            let constraints = std::mem::take(&mut self.mimic_constraints);
            for mc in &constraints {
                self.write_line(&format!(
                    r#"<joint joint1="{}" joint2="{}" polycoef="{} {} 0 0 0"/>"#,
                    mc.follower, mc.leader, mc.offset, mc.multiplier,
                ));
            }
            self.indent -= 1;
            self.write_line("</equality>");
        }

        self.indent -= 1;
        self.write_line("</mujoco>");

        Ok(std::mem::take(&mut self.output))
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
        if roots.is_empty() {
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

        if rpy.norm() < 1e-10 {
            self.write_line(&format!(r#"<body name="{name}" pos="{pos_str}">"#));
        } else {
            self.write_line(&format!(
                r#"<body name="{name}" pos="{pos_str}" euler="{euler_str}">"#
            ));
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

    /// Write a link's collision geoms, named as [`collision_names`] names
    /// them, then its visual geoms (`contype` 0), unnamed.
    fn write_geoms(&mut self, link_name: &str) -> Result<()> {
        let link = self.link(link_name)?;
        let names = self
            .collision_names
            .get(link_name)
            .cloned()
            .unwrap_or_default();
        for (collision, name) in link.collisions.iter().zip(names) {
            self.convert_geom(&collision.geometry, &collision.origin, name.as_ref(), false)?;
        }
        for visual in &link.visuals {
            self.convert_geom(&visual.geometry, &visual.origin, None, true)?;
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
            UrdfJointType::Planar => unreachable!("handled above"),
        };

        let mut attrs = format!(r#"name="{}" type="{}""#, joint.name, joint_type);

        // Add axis for hinge/slide joints
        if axis_needed {
            let axis = joint.axis;
            write!(attrs, r#" axis="{} {} {}""#, axis.x, axis.y, axis.z).ok();
        }

        // A continuous joint is left unlimited, as URDF defines it
        // (D-URDF-CONTINUOUS-LIMIT: MuJoCo limits it).
        if let Some(limit) = &joint.limit
            && matches!(
                joint.joint_type,
                UrdfJointType::Revolute | UrdfJointType::Prismatic
            )
        {
            attrs.push_str(&limit_attrs(limit));
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
        let limit = joint.limit.as_ref().map(limit_attrs).unwrap_or_default();
        let dynamics = dynamics_attrs(joint);
        let name = &joint.name;
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

    fn convert_inertial(&mut self, inertial: &crate::types::UrdfInertial) -> Result<()> {
        let pos = inertial.origin.xyz;
        let rpy = inertial.origin.rpy;
        let mass = inertial.mass;
        let inertia = &inertial.inertia;

        // MJCF inertial uses diaginertia (ixx, iyy, izz) when the tensor is
        // diagonal, or fullinertia (ixx, iyy, izz, ixy, ixz, iyz) when any
        // off-diagonal term is present.
        let inertia_attr = if inertia.ixy.abs() > 1e-10
            || inertia.ixz.abs() > 1e-10
            || inertia.iyz.abs() > 1e-10
        {
            format!(
                r#"fullinertia="{} {} {} {} {} {}""#,
                inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz
            )
        } else {
            format!(
                r#"diaginertia="{} {} {}""#,
                inertia.ixx, inertia.iyy, inertia.izz
            )
        };

        // MJCF requires an inertial's `pos` (MuJoCo 3.5.0 refuses one without
        // it), so it is written even at the origin.
        let mut attrs = format!(
            r#"pos="{} {} {}" mass="{mass}" {inertia_attr}"#,
            pos.x, pos.y, pos.z
        );

        // Add orientation if non-zero. Not beside a full tensor, which
        // `<inertial>` refuses (as MuJoCo's reader does): MuJoCo 3.5.0's own
        // URDF import drops an inertial origin's rotation (measured: a link's
        // tensor with `rpy="0.3 0.5 0.7"` is the one without), and sim-mjcf,
        // which reads only an `<inertial>`'s `quat`, dropped the `euler` this
        // wrote beside one (ledger L372).
        if rpy.norm() > 1e-10 && !inertia_attr.starts_with("fullinertia") {
            write!(attrs, r#" euler="{} {} {}""#, rpy.x, rpy.y, rpy.z).ok();
        }

        self.write_line(&format!("<inertial {attrs}/>"));

        Ok(())
    }

    fn convert_geom(
        &mut self,
        geometry: &UrdfGeometry,
        origin: &UrdfOrigin,
        name: Option<&String>,
        is_visual: bool,
    ) -> Result<()> {
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
            UrdfGeometry::Mesh { filename, scale } => {
                let stripped = strip_package_prefix(filename);
                if let Some(asset) = self.mesh_asset(&stripped, *scale) {
                    let mesh_ref = asset.name.clone();
                    // Mesh geoms use type="mesh" mesh="asset_name" (no size)
                    let mut attrs = format!(r#"type="mesh" mesh="{mesh_ref}""#);
                    if let Some(n) = name {
                        write!(attrs, r#" name="{n}""#).ok();
                    }
                    let pos = origin.xyz;
                    if pos.norm() > 1e-10 {
                        write!(attrs, r#" pos="{} {} {}""#, pos.x, pos.y, pos.z).ok();
                    }
                    let rpy = origin.rpy;
                    if rpy.norm() > 1e-10 {
                        write!(attrs, r#" euler="{} {} {}""#, rpy.x, rpy.y, rpy.z).ok();
                    }
                    if is_visual {
                        attrs.push_str(r#" contype="0" conaffinity="0""#);
                    }
                    self.write_line(&format!("<geom {attrs}/>"));
                }
                return Ok(());
            }
        };

        let mut attrs = format!(r#"type="{geom_type}" size="{size}""#);

        // Add name if present
        if let Some(n) = name {
            write!(attrs, r#" name="{n}""#).ok();
        }

        // Add position if non-zero
        let pos = origin.xyz;
        if pos.norm() > 1e-10 {
            write!(attrs, r#" pos="{} {} {}""#, pos.x, pos.y, pos.z).ok();
        }

        // Add orientation if non-zero
        let rpy = origin.rpy;
        if rpy.norm() > 1e-10 {
            write!(attrs, r#" euler="{} {} {}""#, rpy.x, rpy.y, rpy.z).ok();
        }

        // Visual geoms don't participate in collision
        if is_visual {
            attrs.push_str(r#" contype="0" conaffinity="0""#);
        }

        self.write_line(&format!("<geom {attrs}/>"));

        Ok(())
    }

    /// Every mesh the links' geoms name, as MuJoCo 3.5.0's importer makes
    /// them (`xml_urdf.cc:576-626`): one per file stem and scale, the first
    /// scale of a stem named by the stem, each later one by the stem and its
    /// index; collisions first, links in document order, as MuJoCo meets
    /// them, then the visual geoms' meshes, which MuJoCo never makes under
    /// URDF's `discardvisual`.
    fn collect_mesh_assets(&mut self) {
        let geoms = self
            .robot
            .links
            .iter()
            .flat_map(|link| link.collisions.iter().map(|collision| &collision.geometry));
        let visuals = self
            .robot
            .links
            .iter()
            .flat_map(|link| link.visuals.iter().map(|visual| &visual.geometry));
        for geometry in geoms.chain(visuals) {
            if let UrdfGeometry::Mesh { filename, scale } = geometry {
                let file = strip_package_prefix(filename);
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
        assert!(mjcf.contains(r#"limited="true" range="-3.14 3.14""#));
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

    /// A link's full tensor in a turned inertial origin loads, as the same
    /// tensor unturned: MuJoCo 3.5.0's URDF import drops the turn (measured,
    /// both tensors (0.5, 0.4, 0.3, 0.05, 0.02, 0.01) in the link's frame),
    /// and `<inertial>` refuses a `fullinertia` beside an `euler`.
    #[test]
    fn a_full_tensor_in_a_turned_origin_loads_as_mujocos_urdf_import() {
        let urdf = |rpy: &str| {
            format!(
                r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0.1 0 0"{rpy}/><mass value="2"/><inertia ixx="0.5" iyy="0.4" izz="0.3" ixy="0.05" ixz="0.02" iyz="0.01"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#
            )
        };
        let turned = crate::load_urdf_model(&urdf(r#" rpy="0.3 0.5 0.7""#)).expect("loads");
        let plain = crate::load_urdf_model(&urdf("")).expect("loads");
        assert_eq!(turned.body_inertia, plain.body_inertia);
        assert_eq!(turned.body_iquat, plain.body_iquat);
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
    /// become an MJCF range — only revolute and prismatic are limited. Emitting
    /// a range here would silently clamp a joint URDF says spins freely.
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
        assert!(
            !mjcf.contains("limited="),
            "continuous joints are unbounded; a range would clamp free rotation"
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

    /// Visual geometry is excluded from collision so it cannot perturb the
    /// physics it is only meant to depict.
    #[test]
    fn visual_geometry_is_excluded_from_collision() {
        let urdf = r#"<?xml version="1.0"?>
            <robot name="vis">
                <link name="base">
                    <visual><geometry><sphere radius="0.5"/></geometry></visual>
                </link>
            </robot>"#;

        let mjcf = urdf_to_mjcf(urdf).expect("conversion should succeed");
        assert!(mjcf.contains(r#"contype="0" conaffinity="0""#));
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
    /// named). At the parent both kept `c`.
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

    /// A visual geom is written unnamed: MuJoCo 3.5.0's importer discards
    /// visuals under URDF's `discardvisual` before naming them, so a visual's
    /// name never meets a collision's (measured: a collision and a visual both
    /// named `c` load, geom names `['c', 'd']`).
    #[test]
    fn a_visual_geom_is_written_unnamed() {
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
    /// on bodies 0 and 1). At the parent it was written as a body named
    /// `world`, which only fusion folded into the world. Its `<inertial>` is
    /// dropped, where MuJoCo stores it as the world body's mass (measured:
    /// `body_mass[0]` 1; `D-URDF-WORLD-INERTIAL`).
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

    /// Mesh assets are named as MuJoCo 3.5.0's importer names them: by the
    /// file's stem, a later scale of a stem by the stem and its index, and a
    /// geom of a stem and scale met before names that asset (measured: one
    /// file at scales 1, 2, 1, 3 → meshes `['part', 'part1', 'part2']`, geoms'
    /// meshes 0, 1, 0, 2; two files under one collision name → meshes
    /// `['a', 'b']`, which at the parent were both named `c` and refused).
    /// A `\` separates a path's parts too (`mjuu_strippath`).
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
    /// a revolute, a prismatic and a continuous joint); the parent took z.
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
    /// the parent made a zero axis z.
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
    /// (0, 1, 0), where the parent, from the unit axis, gave
    /// (0, 0.99999999999987, -5e-7) (ledger L316); an axis whose product
    /// with z is within mjEPS of unit length keeps it unscaled
    /// (`mjuu_normvec`), and so do MuJoCo's slide axes. The rows through `sin`
    /// and `cos` are compared bit for bit on macOS arm64 only, elsewhere to
    /// 1e-15.
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
}
