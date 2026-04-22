//! URDF validation utilities.
//!
//! Validates the kinematic structure and physical properties of parsed URDF.

use std::collections::{HashMap, HashSet};

use crate::error::{Result, UrdfError};
use crate::types::{UrdfGeometry, UrdfRobot};

/// Validation result containing the root link and kinematic structure.
#[derive(Debug)]
pub struct ValidationResult {
    /// The root link name (link with no parent joint).
    pub root_link: String,
    /// Map from link name to its parent joint name.
    pub link_parent_joint: HashMap<String, String>,
    /// Map from link name to its child joint names.
    pub link_child_joints: HashMap<String, Vec<String>>,
    /// Topologically sorted link names (root first).
    pub sorted_links: Vec<String>,
}

/// Validate a URDF robot model.
///
/// This checks:
/// - All joints reference valid links
/// - No duplicate link or joint names
/// - Exactly one root link (no parent)
/// - No kinematic loops
/// - Valid mass properties (if present)
/// - Finite geometry dimensions on all visual and collision shapes
///
/// # Errors
///
/// Returns an error if validation fails.
pub fn validate(robot: &UrdfRobot) -> Result<ValidationResult> {
    // Check for duplicate names
    check_duplicates(robot)?;

    // Build link name set
    let link_names: HashSet<&str> = robot.links.iter().map(|l| l.name.as_str()).collect();

    // Validate joint references and build parent/child maps
    let mut link_parent_joint: HashMap<String, String> = HashMap::new();
    let mut link_child_joints: HashMap<String, Vec<String>> = HashMap::new();

    // Initialize child joints map for all links
    for link in &robot.links {
        link_child_joints.insert(link.name.clone(), Vec::new());
    }

    for joint in &robot.joints {
        // Check parent link exists
        if !link_names.contains(joint.parent.as_str()) {
            return Err(UrdfError::undefined_link(&joint.parent, &joint.name));
        }
        // Check child link exists
        if !link_names.contains(joint.child.as_str()) {
            return Err(UrdfError::undefined_link(&joint.child, &joint.name));
        }

        // Record parent joint for child link
        if link_parent_joint.contains_key(&joint.child) {
            return Err(UrdfError::KinematicLoop(format!(
                "link '{}' has multiple parent joints",
                joint.child
            )));
        }
        link_parent_joint.insert(joint.child.clone(), joint.name.clone());

        // Record child joint for parent link
        link_child_joints
            .entry(joint.parent.clone())
            .or_default()
            .push(joint.name.clone());
    }

    // Find root links (links with no parent joint)
    let root_links: Vec<&str> = robot
        .links
        .iter()
        .filter(|l| !link_parent_joint.contains_key(&l.name))
        .map(|l| l.name.as_str())
        .collect();

    if root_links.is_empty() {
        return Err(UrdfError::NoRootLink);
    }
    if root_links.len() > 1 {
        return Err(UrdfError::MultipleRootLinks(
            root_links.iter().map(|s| (*s).to_string()).collect(),
        ));
    }

    let root_link = root_links[0].to_string();

    // Topological sort (detect cycles)
    let sorted_links = topological_sort(robot, &root_link, &link_child_joints)?;

    // Validate mass properties
    validate_mass_properties(robot)?;

    // Validate geometry dimensions
    validate_geometry(robot)?;

    Ok(ValidationResult {
        root_link,
        link_parent_joint,
        link_child_joints,
        sorted_links,
    })
}

/// Check for duplicate link and joint names.
fn check_duplicates(robot: &UrdfRobot) -> Result<()> {
    let mut link_names = HashSet::new();
    for link in &robot.links {
        if !link_names.insert(&link.name) {
            return Err(UrdfError::DuplicateLink(link.name.clone()));
        }
    }

    let mut joint_names = HashSet::new();
    for joint in &robot.joints {
        if !joint_names.insert(&joint.name) {
            return Err(UrdfError::DuplicateJoint(joint.name.clone()));
        }
    }

    Ok(())
}

/// Topological sort of links starting from root.
fn topological_sort(
    robot: &UrdfRobot,
    root: &str,
    link_child_joints: &HashMap<String, Vec<String>>,
) -> Result<Vec<String>> {
    let mut sorted = Vec::new();
    let mut visited = HashSet::new();
    let mut visiting = HashSet::new();

    // Build joint to child link map
    let joint_to_child: HashMap<&str, &str> = robot
        .joints
        .iter()
        .map(|j| (j.name.as_str(), j.child.as_str()))
        .collect();

    fn visit(
        link: &str,
        link_child_joints: &HashMap<String, Vec<String>>,
        joint_to_child: &HashMap<&str, &str>,
        visited: &mut HashSet<String>,
        visiting: &mut HashSet<String>,
        sorted: &mut Vec<String>,
    ) -> Result<()> {
        if visited.contains(link) {
            return Ok(());
        }
        if visiting.contains(link) {
            return Err(UrdfError::KinematicLoop(format!(
                "cycle detected involving link '{link}'"
            )));
        }

        visiting.insert(link.to_string());

        if let Some(child_joints) = link_child_joints.get(link) {
            for joint_name in child_joints {
                if let Some(&child_link) = joint_to_child.get(joint_name.as_str()) {
                    visit(
                        child_link,
                        link_child_joints,
                        joint_to_child,
                        visited,
                        visiting,
                        sorted,
                    )?;
                }
            }
        }

        visiting.remove(link);
        visited.insert(link.to_string());
        sorted.push(link.to_string());

        Ok(())
    }

    visit(
        root,
        link_child_joints,
        &joint_to_child,
        &mut visited,
        &mut visiting,
        &mut sorted,
    )?;

    // Reverse to get root-first order
    sorted.reverse();
    Ok(sorted)
}

/// Validate mass properties of all links.
fn validate_mass_properties(robot: &UrdfRobot) -> Result<()> {
    for link in &robot.links {
        if let Some(ref inertial) = link.inertial {
            // Check mass is positive
            if inertial.mass <= 0.0 {
                return Err(UrdfError::invalid_mass(&link.name, inertial.mass));
            }
            if !inertial.mass.is_finite() {
                return Err(UrdfError::invalid_mass(&link.name, inertial.mass));
            }

            // Check inertia tensor has non-negative diagonal elements
            let i = &inertial.inertia;
            if i.ixx < 0.0 || i.iyy < 0.0 || i.izz < 0.0 {
                return Err(UrdfError::invalid_inertia(
                    &link.name,
                    "diagonal elements must be non-negative",
                ));
            }

            // Check for NaN/Inf
            if !i.ixx.is_finite()
                || !i.iyy.is_finite()
                || !i.izz.is_finite()
                || !i.ixy.is_finite()
                || !i.ixz.is_finite()
                || !i.iyz.is_finite()
            {
                return Err(UrdfError::invalid_inertia(
                    &link.name,
                    "inertia values must be finite",
                ));
            }

            // Check triangle inequality for physical validity
            // For a valid inertia tensor: Ixx + Iyy >= Izz (and cyclic)
            // This is a necessary (but not sufficient) condition
            if i.ixx + i.iyy < i.izz || i.iyy + i.izz < i.ixx || i.izz + i.ixx < i.iyy {
                // This is a warning-level issue, not an error
                // Many URDFs have slightly invalid inertias
            }
        }
    }

    Ok(())
}

/// Validate geometry dimensions on all visual and collision shapes.
///
/// Rejects non-finite (`NaN` or ±Inf) numeric fields at the URDF ingress
/// boundary so that downstream mesh construction and collision code
/// can assume finite geometry. Positivity is not checked here (some URDFs
/// intentionally ship with degenerate shapes; see the triangle-inequality
/// precedent in `validate_mass_properties`).
fn validate_geometry(robot: &UrdfRobot) -> Result<()> {
    for link in &robot.links {
        for visual in &link.visuals {
            check_geometry_finite(&visual.geometry, &link.name)?;
        }
        for collision in &link.collisions {
            check_geometry_finite(&collision.geometry, &link.name)?;
        }
    }
    Ok(())
}

/// Check that all numeric fields of a geometry are finite.
fn check_geometry_finite(geom: &UrdfGeometry, link_name: &str) -> Result<()> {
    match geom {
        UrdfGeometry::Box { size } => {
            if !size.x.is_finite() || !size.y.is_finite() || !size.z.is_finite() {
                return Err(UrdfError::invalid_geometry(
                    link_name,
                    "box size must be finite",
                ));
            }
        }
        UrdfGeometry::Cylinder { radius, length } => {
            if !radius.is_finite() || !length.is_finite() {
                return Err(UrdfError::invalid_geometry(
                    link_name,
                    "cylinder radius and length must be finite",
                ));
            }
        }
        UrdfGeometry::Sphere { radius } => {
            if !radius.is_finite() {
                return Err(UrdfError::invalid_geometry(
                    link_name,
                    "sphere radius must be finite",
                ));
            }
        }
        UrdfGeometry::Mesh { scale: Some(s), .. } => {
            if !s.x.is_finite() || !s.y.is_finite() || !s.z.is_finite() {
                return Err(UrdfError::invalid_geometry(
                    link_name,
                    "mesh scale must be finite",
                ));
            }
        }
        UrdfGeometry::Mesh { scale: None, .. } => {}
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::types::{
        UrdfCollision, UrdfInertia, UrdfInertial, UrdfJoint, UrdfJointType, UrdfLink, UrdfOrigin,
        UrdfVisual,
    };
    use nalgebra::Vector3;

    fn make_robot_with_chain() -> UrdfRobot {
        // base -> link1 -> link2
        UrdfRobot::new("test")
            .with_link(UrdfLink::new("base"))
            .with_link(UrdfLink::new("link1"))
            .with_link(UrdfLink::new("link2"))
            .with_joint(UrdfJoint::new(
                "j1",
                UrdfJointType::Revolute,
                "base",
                "link1",
            ))
            .with_joint(UrdfJoint::new(
                "j2",
                UrdfJointType::Revolute,
                "link1",
                "link2",
            ))
    }

    #[test]
    fn test_valid_chain() {
        let robot = make_robot_with_chain();
        let result = validate(&robot).expect("should validate");

        assert_eq!(result.root_link, "base");
        assert_eq!(result.sorted_links, vec!["base", "link1", "link2"]);
    }

    #[test]
    fn test_duplicate_link() {
        let robot = UrdfRobot::new("test")
            .with_link(UrdfLink::new("base"))
            .with_link(UrdfLink::new("base")); // Duplicate

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::DuplicateLink(_))));
    }

    #[test]
    fn test_undefined_link_reference() {
        let robot = UrdfRobot::new("test")
            .with_link(UrdfLink::new("base"))
            .with_joint(UrdfJoint::new(
                "j1",
                UrdfJointType::Fixed,
                "base",
                "nonexistent",
            ));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::UndefinedLink { .. })));
    }

    #[test]
    fn test_no_root_link() {
        // A cycle with no root
        let robot = UrdfRobot::new("test")
            .with_link(UrdfLink::new("a"))
            .with_link(UrdfLink::new("b"))
            .with_joint(UrdfJoint::new("j1", UrdfJointType::Fixed, "a", "b"))
            .with_joint(UrdfJoint::new("j2", UrdfJointType::Fixed, "b", "a"));

        let result = validate(&robot);
        // This should fail because 'a' has multiple parent joints
        assert!(result.is_err());
    }

    #[test]
    fn test_multiple_roots() {
        let robot = UrdfRobot::new("test")
            .with_link(UrdfLink::new("root1"))
            .with_link(UrdfLink::new("root2"));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::MultipleRootLinks(_))));
    }

    #[test]
    fn test_invalid_mass() {
        let robot =
            UrdfRobot::new("test").with_link(UrdfLink::new("base").with_inertial(UrdfInertial {
                mass: -1.0, // Invalid
                ..Default::default()
            }));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::InvalidMass { .. })));
    }

    #[test]
    fn test_invalid_inertia() {
        let robot =
            UrdfRobot::new("test").with_link(UrdfLink::new("base").with_inertial(UrdfInertial {
                mass: 1.0,
                inertia: UrdfInertia {
                    ixx: -0.1, // Invalid
                    ..Default::default()
                },
                ..Default::default()
            }));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::InvalidInertia { .. })));
    }

    #[test]
    fn test_tree_structure() {
        // base -> (link1, link2)
        let robot = UrdfRobot::new("test")
            .with_link(UrdfLink::new("base"))
            .with_link(UrdfLink::new("link1"))
            .with_link(UrdfLink::new("link2"))
            .with_joint(UrdfJoint::new(
                "j1",
                UrdfJointType::Revolute,
                "base",
                "link1",
            ))
            .with_joint(UrdfJoint::new(
                "j2",
                UrdfJointType::Revolute,
                "base",
                "link2",
            ));

        let result = validate(&robot).expect("should validate");
        assert_eq!(result.root_link, "base");
        assert_eq!(result.sorted_links[0], "base");
        // link1 and link2 order doesn't matter, both are children of base
        assert!(result.sorted_links.contains(&"link1".to_string()));
        assert!(result.sorted_links.contains(&"link2".to_string()));
    }

    fn make_link_with_collision(link_name: &str, geometry: UrdfGeometry) -> UrdfLink {
        UrdfLink::new(link_name).with_collision(UrdfCollision {
            name: None,
            origin: UrdfOrigin::default(),
            geometry,
        })
    }

    #[test]
    fn test_invalid_geometry_box_nan() {
        let robot = UrdfRobot::new("test").with_link(make_link_with_collision(
            "base",
            UrdfGeometry::Box {
                size: Vector3::new(1.0, f64::NAN, 1.0),
            },
        ));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::InvalidGeometry { .. })));
    }

    #[test]
    fn test_invalid_geometry_cylinder_inf() {
        let robot = UrdfRobot::new("test").with_link(make_link_with_collision(
            "base",
            UrdfGeometry::Cylinder {
                radius: f64::INFINITY,
                length: 1.0,
            },
        ));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::InvalidGeometry { .. })));
    }

    #[test]
    fn test_invalid_geometry_sphere_nan() {
        let robot = UrdfRobot::new("test").with_link(make_link_with_collision(
            "base",
            UrdfGeometry::Sphere { radius: f64::NAN },
        ));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::InvalidGeometry { .. })));
    }

    #[test]
    fn test_invalid_geometry_mesh_scale_nan() {
        let robot =
            UrdfRobot::new("test").with_link(UrdfLink::new("base").with_collision(UrdfCollision {
                name: None,
                origin: UrdfOrigin::default(),
                geometry: UrdfGeometry::Mesh {
                    filename: "foo.obj".to_string(),
                    scale: Some(Vector3::new(1.0, f64::NAN, 1.0)),
                },
            }));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::InvalidGeometry { .. })));
    }

    #[test]
    fn test_invalid_geometry_visual_detected() {
        // Same NaN check applies to <visual> as to <collision>.
        let robot =
            UrdfRobot::new("test").with_link(UrdfLink::new("base").with_collision(UrdfCollision {
                name: None,
                origin: UrdfOrigin::default(),
                geometry: UrdfGeometry::Sphere { radius: 1.0 },
            }));
        // Now mutate to add a bad visual alongside the good collision.
        let mut robot = robot;
        robot.links[0].visuals.push(UrdfVisual {
            name: None,
            origin: UrdfOrigin::default(),
            geometry: UrdfGeometry::Box {
                size: Vector3::new(f64::NAN, 1.0, 1.0),
            },
            material: None,
        });

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::InvalidGeometry { .. })));
    }

    #[test]
    fn test_valid_geometry_passes() {
        // Mesh with no scale, sphere with finite radius — should validate.
        let robot = UrdfRobot::new("test")
            .with_link(make_link_with_collision(
                "base",
                UrdfGeometry::Sphere { radius: 0.5 },
            ))
            .with_link(make_link_with_collision(
                "link1",
                UrdfGeometry::Mesh {
                    filename: "foo.obj".to_string(),
                    scale: None,
                },
            ))
            .with_joint(UrdfJoint::new("j1", UrdfJointType::Fixed, "base", "link1"));

        let result = validate(&robot);
        assert!(result.is_ok());
    }
}
