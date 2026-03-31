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
use crate::types::{UrdfGeometry, UrdfJoint, UrdfJointType, UrdfLink, UrdfOrigin, UrdfRobot};

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
    let mut converter = Converter::new(robot);
    converter.convert()
}

/// A mimic constraint collected during tree walk.
struct MimicEntry {
    /// The follower joint name (the joint that has <mimic>).
    follower: String,
    /// The leader joint name.
    leader: String,
    /// Polynomial coefficients: position = multiplier * leader + offset.
    multiplier: f64,
    offset: f64,
}

/// A mesh asset collected during geom conversion.
struct MeshAssetEntry {
    /// Unique asset name for MJCF reference.
    name: String,
    /// File path from URDF (package:// prefix stripped).
    file: String,
    /// Optional scale.
    scale: Option<nalgebra::Vector3<f64>>,
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
    fn new(robot: &'a UrdfRobot) -> Self {
        // Build lookup maps
        let mut links = HashMap::new();
        for link in &robot.links {
            links.insert(link.name.as_str(), link);
        }

        let mut child_to_joint = HashMap::new();
        let mut parent_to_joints: HashMap<&str, Vec<&UrdfJoint>> = HashMap::new();
        for joint in &robot.joints {
            child_to_joint.insert(joint.child.as_str(), joint);
            parent_to_joints
                .entry(joint.parent.as_str())
                .or_default()
                .push(joint);
        }

        Self {
            robot,
            links,
            child_to_joint,
            parent_to_joints,
            output: String::with_capacity(4096),
            indent: 0,
            mesh_assets: Vec::new(),
            mimic_constraints: Vec::new(),
        }
    }

    fn convert(&mut self) -> Result<String> {
        // Find root link (not a child of any joint)
        let root_link = self.find_root_link()?;

        // Write MJCF header
        self.write_line(&format!(r#"<mujoco model="{}">"#, self.robot.name));
        self.indent += 1;

        // Write options
        self.write_line(r#"<option gravity="0 0 -9.81" timestep="0.002"/>"#);
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

        // Convert the kinematic tree starting from root
        self.convert_link(root_link, None)?;

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

    fn find_root_link(&self) -> Result<&'a str> {
        let child_links: HashSet<&str> = self.child_to_joint.keys().copied().collect();

        for link in &self.robot.links {
            if !child_links.contains(link.name.as_str()) {
                return Ok(&link.name);
            }
        }

        Err(UrdfError::NoRootLink)
    }

    fn convert_link(&mut self, link_name: &str, parent_joint: Option<&UrdfJoint>) -> Result<()> {
        // Clone data we need from the link to avoid borrow conflicts
        let (name, inertial, collisions, visuals) = {
            let link = self
                .links
                .get(link_name)
                .ok_or_else(|| UrdfError::undefined_link(link_name, "converter"))?;
            (
                link.name.clone(),
                link.inertial,
                link.collisions.clone(),
                link.visuals.clone(),
            )
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

        // Write collision geoms
        for collision in &collisions {
            self.convert_geom(
                &collision.geometry,
                &collision.origin,
                collision.name.as_ref(),
                false,
            )?;
        }

        // Write visual geoms (as separate geoms with contype=0)
        for visual in &visuals {
            self.convert_geom(&visual.geometry, &visual.origin, visual.name.as_ref(), true)?;
        }

        // Recursively convert child links - clone to avoid borrow conflicts
        let child_joints: Vec<_> = self
            .parent_to_joints
            .get(link_name)
            .map(|joints| {
                joints
                    .iter()
                    .map(|j| (j.child.clone(), (*j).clone()))
                    .collect()
            })
            .unwrap_or_default();

        for (child_name, child_joint) in &child_joints {
            self.convert_link(child_name, Some(child_joint))?;
        }

        self.indent -= 1;
        self.write_line("</body>");

        Ok(())
    }

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

        // Add limits for revolute/prismatic
        if let Some(limit) = &joint.limit {
            if matches!(
                joint.joint_type,
                UrdfJointType::Revolute | UrdfJointType::Prismatic
            ) {
                write!(
                    attrs,
                    r#" limited="true" range="{} {}""#,
                    limit.lower, limit.upper
                )
                .ok();
            }
        }

        // Add dynamics (damping and friction)
        if let Some(dynamics) = &joint.dynamics {
            if dynamics.damping > 0.0 {
                write!(attrs, r#" damping="{}""#, dynamics.damping).ok();
            }
            if dynamics.friction > 0.0 {
                write!(attrs, r#" frictionloss="{}""#, dynamics.friction).ok();
            }
        }

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

    /// Convert a planar joint into 2 slide joints + 1 hinge joint (3 DOF).
    ///
    /// The joint axis defines the plane normal. Two orthogonal in-plane axes
    /// are computed, giving translation in the plane + rotation about the normal.
    fn convert_planar_joint(&mut self, joint: &UrdfJoint) -> Result<()> {
        let normal = joint.axis.normalize();

        // Compute two orthogonal in-plane axes
        let arbitrary = if normal.x.abs() < 0.9 {
            nalgebra::Vector3::x()
        } else {
            nalgebra::Vector3::y()
        };
        let in_plane_x = normal.cross(&arbitrary).normalize();
        let in_plane_y = normal.cross(&in_plane_x).normalize();

        // Slide along in-plane X
        self.write_line(&format!(
            r#"<joint name="{}_slide_x" type="slide" axis="{} {} {}"/>"#,
            joint.name, in_plane_x.x, in_plane_x.y, in_plane_x.z,
        ));
        // Slide along in-plane Y
        self.write_line(&format!(
            r#"<joint name="{}_slide_y" type="slide" axis="{} {} {}"/>"#,
            joint.name, in_plane_y.x, in_plane_y.y, in_plane_y.z,
        ));
        // Hinge about normal
        self.write_line(&format!(
            r#"<joint name="{}_hinge" type="hinge" axis="{} {} {}"/>"#,
            joint.name, normal.x, normal.y, normal.z,
        ));

        Ok(())
    }

    fn convert_inertial(&mut self, inertial: &crate::types::UrdfInertial) -> Result<()> {
        let pos = inertial.origin.xyz;
        let rpy = inertial.origin.rpy;
        let mass = inertial.mass;
        let inertia = &inertial.inertia;

        // MJCF inertial uses diagonal inertia (ixx, iyy, izz)
        // and optional fullinertia for off-diagonal terms
        let diag_inertia = format!("{} {} {}", inertia.ixx, inertia.iyy, inertia.izz);

        let mut attrs = format!(r#"mass="{mass}" diaginertia="{diag_inertia}""#);

        // Add position if non-zero
        if pos.norm() > 1e-10 {
            write!(attrs, r#" pos="{} {} {}""#, pos.x, pos.y, pos.z).ok();
        }

        // Add orientation if non-zero
        if rpy.norm() > 1e-10 {
            write!(attrs, r#" euler="{} {} {}""#, rpy.x, rpy.y, rpy.z).ok();
        }

        // Check for off-diagonal inertia terms
        if inertia.ixy.abs() > 1e-10 || inertia.ixz.abs() > 1e-10 || inertia.iyz.abs() > 1e-10 {
            // Use fullinertia instead: ixx, iyy, izz, ixy, ixz, iyz
            let full = format!(
                "{} {} {} {} {} {}",
                inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz
            );
            // Replace diaginertia with fullinertia
            attrs = format!(r#"mass="{mass}" fullinertia="{full}""#);
            if pos.norm() > 1e-10 {
                write!(attrs, r#" pos="{} {} {}""#, pos.x, pos.y, pos.z).ok();
            }
            if rpy.norm() > 1e-10 {
                write!(attrs, r#" euler="{} {} {}""#, rpy.x, rpy.y, rpy.z).ok();
            }
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
            UrdfGeometry::Mesh { filename, .. } => {
                // Look up the pre-collected mesh asset by filename
                let stripped = strip_package_prefix(filename);
                if let Some(asset) = self.mesh_assets.iter().find(|a| a.file == stripped) {
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

    /// Pre-scan all links to collect mesh asset entries.
    fn collect_mesh_assets(&mut self) {
        let mut counter = 0usize;
        for link in &self.robot.links {
            // Check collision geometries
            for collision in &link.collisions {
                if let UrdfGeometry::Mesh { filename, scale } = &collision.geometry {
                    let stripped = strip_package_prefix(filename);
                    // Avoid duplicate assets for the same file
                    if !self.mesh_assets.iter().any(|a| a.file == stripped) {
                        let name = if let Some(n) = &collision.name {
                            n.clone()
                        } else {
                            counter += 1;
                            format!("{}_mesh_{counter}", link.name)
                        };
                        self.mesh_assets.push(MeshAssetEntry {
                            name,
                            file: stripped,
                            scale: *scale,
                        });
                    }
                }
            }
            // Check visual geometries
            for visual in &link.visuals {
                if let UrdfGeometry::Mesh { filename, scale } = &visual.geometry {
                    let stripped = strip_package_prefix(filename);
                    if !self.mesh_assets.iter().any(|a| a.file == stripped) {
                        let name = if let Some(n) = &visual.name {
                            n.clone()
                        } else {
                            counter += 1;
                            format!("{}_mesh_{counter}", link.name)
                        };
                        self.mesh_assets.push(MeshAssetEntry {
                            name,
                            file: stripped,
                            scale: *scale,
                        });
                    }
                }
            }
        }
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
}
