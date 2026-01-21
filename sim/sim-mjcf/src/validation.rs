//! MJCF validation utilities.
//!
//! Validates the kinematic structure and physical properties of parsed MJCF.

use std::collections::{HashMap, HashSet};

use crate::error::{MjcfError, Result};
use crate::types::{MjcfBody, MjcfModel};

/// Validation result containing the flattened body tree structure.
#[derive(Debug)]
pub struct ValidationResult {
    /// All body names in topological order (parents before children).
    pub sorted_bodies: Vec<String>,
    /// Map from body name to parent body name.
    pub body_parent: HashMap<String, String>,
    /// Map from body name to child body names.
    pub body_children: HashMap<String, Vec<String>>,
    /// All joint names.
    pub joint_names: Vec<String>,
    /// Map from joint name to body name.
    pub joint_to_body: HashMap<String, String>,
    /// All actuator names.
    pub actuator_names: Vec<String>,
}

/// Validate an MJCF model.
///
/// This checks:
/// - No duplicate body names
/// - No duplicate joint names
/// - No duplicate actuator names
/// - Actuators reference valid joints
/// - Valid mass properties (if present)
///
/// # Errors
///
/// Returns an error if validation fails.
pub fn validate(model: &MjcfModel) -> Result<ValidationResult> {
    // Check for duplicate names
    let mut body_names = HashSet::new();
    let mut joint_names = HashSet::new();
    let mut actuator_names = HashSet::new();

    let mut sorted_bodies = Vec::new();
    let mut body_parent: HashMap<String, String> = HashMap::new();
    let mut body_children: HashMap<String, Vec<String>> = HashMap::new();
    let mut joint_to_body: HashMap<String, String> = HashMap::new();
    let mut all_joint_names = Vec::new();

    // Traverse body tree
    fn traverse_body(
        body: &MjcfBody,
        parent_name: Option<&str>,
        body_names: &mut HashSet<String>,
        joint_names: &mut HashSet<String>,
        sorted_bodies: &mut Vec<String>,
        body_parent: &mut HashMap<String, String>,
        body_children: &mut HashMap<String, Vec<String>>,
        joint_to_body: &mut HashMap<String, String>,
        all_joint_names: &mut Vec<String>,
    ) -> Result<()> {
        // Check for duplicate body name
        if !body_names.insert(body.name.clone()) {
            return Err(MjcfError::DuplicateBody(body.name.clone()));
        }

        sorted_bodies.push(body.name.clone());

        // Set parent relationship
        if let Some(parent) = parent_name {
            body_parent.insert(body.name.clone(), parent.to_string());
            body_children
                .entry(parent.to_string())
                .or_default()
                .push(body.name.clone());
        }

        // Initialize children list
        body_children.entry(body.name.clone()).or_default();

        // Check joints
        for joint in &body.joints {
            if !joint.name.is_empty() {
                if !joint_names.insert(joint.name.clone()) {
                    return Err(MjcfError::DuplicateJoint(joint.name.clone()));
                }
                joint_to_body.insert(joint.name.clone(), body.name.clone());
                all_joint_names.push(joint.name.clone());
            }
        }

        // Validate inertial if present
        if let Some(ref inertial) = body.inertial {
            // Check mass is positive
            if inertial.mass <= 0.0 {
                return Err(MjcfError::invalid_mass(&body.name, inertial.mass));
            }
            if !inertial.mass.is_finite() {
                return Err(MjcfError::invalid_mass(&body.name, inertial.mass));
            }

            // Check inertia tensor if specified
            if let Some(diag) = inertial.diaginertia {
                if diag.x < 0.0 || diag.y < 0.0 || diag.z < 0.0 {
                    return Err(MjcfError::invalid_inertia(
                        &body.name,
                        "diagonal elements must be non-negative",
                    ));
                }
                if !diag.x.is_finite() || !diag.y.is_finite() || !diag.z.is_finite() {
                    return Err(MjcfError::invalid_inertia(
                        &body.name,
                        "inertia values must be finite",
                    ));
                }
            }
        }

        // Recurse to children
        for child in &body.children {
            traverse_body(
                child,
                Some(&body.name),
                body_names,
                joint_names,
                sorted_bodies,
                body_parent,
                body_children,
                joint_to_body,
                all_joint_names,
            )?;
        }

        Ok(())
    }

    // Traverse from worldbody children (worldbody itself is not included)
    for body in &model.worldbody.children {
        traverse_body(
            body,
            None, // Root bodies have no parent (worldbody is implicit)
            &mut body_names,
            &mut joint_names,
            &mut sorted_bodies,
            &mut body_parent,
            &mut body_children,
            &mut joint_to_body,
            &mut all_joint_names,
        )?;
    }

    // Check actuators
    let mut all_actuator_names = Vec::new();
    for actuator in &model.actuators {
        if !actuator.name.is_empty() {
            if !actuator_names.insert(actuator.name.clone()) {
                return Err(MjcfError::DuplicateActuator(actuator.name.clone()));
            }
            all_actuator_names.push(actuator.name.clone());
        }

        // Check that joint reference is valid
        if let Some(ref joint_name) = actuator.joint {
            if !joint_names.contains(joint_name) {
                return Err(MjcfError::undefined_joint(
                    joint_name,
                    format!("actuator '{}'", actuator.name),
                ));
            }
        }
    }

    Ok(ValidationResult {
        sorted_bodies,
        body_parent,
        body_children,
        joint_names: all_joint_names,
        joint_to_body,
        actuator_names: all_actuator_names,
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::types::{MjcfActuator, MjcfGeom, MjcfInertial, MjcfJoint};
    use nalgebra::Vector3;

    fn make_simple_model() -> MjcfModel {
        MjcfModel::new("test").with_body(
            MjcfBody::new("base")
                .with_geom(MjcfGeom::sphere(0.1))
                .with_child(
                    MjcfBody::new("link1")
                        .with_joint(MjcfJoint::hinge("joint1", Vector3::z()))
                        .with_geom(MjcfGeom::sphere(0.05)),
                ),
        )
    }

    #[test]
    fn test_valid_model() {
        let model = make_simple_model();
        let result = validate(&model).expect("should validate");

        assert_eq!(result.sorted_bodies, vec!["base", "link1"]);
        assert_eq!(result.joint_names, vec!["joint1"]);
        assert_eq!(result.body_parent.get("link1"), Some(&"base".to_string()));
    }

    #[test]
    fn test_duplicate_body() {
        let model = MjcfModel::new("test")
            .with_body(MjcfBody::new("duplicate"))
            .with_body(MjcfBody::new("duplicate"));

        let result = validate(&model);
        assert!(matches!(result, Err(MjcfError::DuplicateBody(_))));
    }

    #[test]
    fn test_duplicate_joint() {
        let model = MjcfModel::new("test").with_body(
            MjcfBody::new("base")
                .with_joint(MjcfJoint::hinge("j1", Vector3::z()))
                .with_child(
                    MjcfBody::new("link1").with_joint(MjcfJoint::hinge("j1", Vector3::z())), // Duplicate
                ),
        );

        let result = validate(&model);
        assert!(matches!(result, Err(MjcfError::DuplicateJoint(_))));
    }

    #[test]
    fn test_actuator_invalid_joint() {
        let model = MjcfModel::new("test")
            .with_body(MjcfBody::new("base").with_joint(MjcfJoint::hinge("joint1", Vector3::z())))
            .with_actuator(MjcfActuator::motor("motor1", "nonexistent_joint"));

        let result = validate(&model);
        assert!(matches!(result, Err(MjcfError::UndefinedJoint { .. })));
    }

    #[test]
    fn test_invalid_mass() {
        let model = MjcfModel::new("test").with_body(MjcfBody {
            name: "base".to_string(),
            inertial: Some(MjcfInertial {
                mass: -1.0, // Invalid
                ..Default::default()
            }),
            ..Default::default()
        });

        let result = validate(&model);
        assert!(matches!(result, Err(MjcfError::InvalidMass { .. })));
    }

    #[test]
    fn test_invalid_inertia() {
        let model = MjcfModel::new("test").with_body(MjcfBody {
            name: "base".to_string(),
            inertial: Some(MjcfInertial {
                mass: 1.0,
                diaginertia: Some(Vector3::new(-0.1, 0.2, 0.3)), // Invalid negative
                ..Default::default()
            }),
            ..Default::default()
        });

        let result = validate(&model);
        assert!(matches!(result, Err(MjcfError::InvalidInertia { .. })));
    }

    #[test]
    fn test_nested_bodies() {
        let model = MjcfModel::new("test").with_body(
            MjcfBody::new("root")
                .with_child(MjcfBody::new("level1a").with_child(MjcfBody::new("level2")))
                .with_child(MjcfBody::new("level1b")),
        );

        let result = validate(&model).expect("should validate");

        assert_eq!(result.sorted_bodies.len(), 4);
        assert_eq!(result.sorted_bodies[0], "root");

        // Check parent relationships
        assert_eq!(result.body_parent.get("level1a"), Some(&"root".to_string()));
        assert_eq!(result.body_parent.get("level1b"), Some(&"root".to_string()));
        assert_eq!(
            result.body_parent.get("level2"),
            Some(&"level1a".to_string())
        );
    }

    #[test]
    fn test_empty_model() {
        let model = MjcfModel::new("empty");
        let result = validate(&model);
        assert!(result.is_ok());
    }
}
