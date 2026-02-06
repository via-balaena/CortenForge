//! MJCF validation utilities.
//!
//! Validates the kinematic structure and physical properties of parsed MJCF.

use std::collections::{HashMap, HashSet};

use crate::error::{MjcfError, Result};
use crate::types::{MjcfBody, MjcfModel, MjcfOption, MjcfTendonType};

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

/// Validate MJCF simulation options.
///
/// This checks that all option values are physically reasonable:
/// - Timestep is positive and not too large
/// - Iteration counts are reasonable
/// - Tolerance is positive
/// - Impedance ratio is positive
/// - Density and viscosity are non-negative
///
/// # Errors
///
/// Returns an error if any option value is invalid.
pub fn validate_option(option: &MjcfOption) -> Result<()> {
    // Timestep validation
    if !option.timestep.is_finite() || option.timestep <= 0.0 {
        return Err(MjcfError::invalid_option(
            "timestep",
            format!("must be positive and finite, got {}", option.timestep),
        ));
    }
    if option.timestep > 1.0 {
        return Err(MjcfError::invalid_option(
            "timestep",
            format!(
                "value {} is unusually large (> 1 second), likely an error",
                option.timestep
            ),
        ));
    }

    // Iteration count validation
    if option.iterations == 0 {
        return Err(MjcfError::invalid_option(
            "iterations",
            "must be at least 1",
        ));
    }

    // Tolerance validation
    if !option.tolerance.is_finite() || option.tolerance <= 0.0 {
        return Err(MjcfError::invalid_option(
            "tolerance",
            format!("must be positive and finite, got {}", option.tolerance),
        ));
    }

    // Impedance ratio validation
    if !option.impratio.is_finite() || option.impratio <= 0.0 {
        return Err(MjcfError::invalid_option(
            "impratio",
            format!("must be positive and finite, got {}", option.impratio),
        ));
    }

    // Constraint solver tuning validation
    if !option.regularization.is_finite() || option.regularization <= 0.0 {
        return Err(MjcfError::invalid_option(
            "regularization",
            format!("must be positive and finite, got {}", option.regularization),
        ));
    }
    if !option.default_eq_stiffness.is_finite() || option.default_eq_stiffness <= 0.0 {
        return Err(MjcfError::invalid_option(
            "default_eq_stiffness",
            format!(
                "must be positive and finite, got {}",
                option.default_eq_stiffness
            ),
        ));
    }
    if !option.default_eq_damping.is_finite() || option.default_eq_damping <= 0.0 {
        return Err(MjcfError::invalid_option(
            "default_eq_damping",
            format!(
                "must be positive and finite, got {}",
                option.default_eq_damping
            ),
        ));
    }
    if !option.max_constraint_vel.is_finite() || option.max_constraint_vel <= 0.0 {
        return Err(MjcfError::invalid_option(
            "max_constraint_vel",
            format!(
                "must be positive and finite, got {}",
                option.max_constraint_vel
            ),
        ));
    }
    if !option.max_constraint_angvel.is_finite() || option.max_constraint_angvel <= 0.0 {
        return Err(MjcfError::invalid_option(
            "max_constraint_angvel",
            format!(
                "must be positive and finite, got {}",
                option.max_constraint_angvel
            ),
        ));
    }
    if !option.friction_smoothing.is_finite() || option.friction_smoothing <= 0.0 {
        return Err(MjcfError::invalid_option(
            "friction_smoothing",
            format!(
                "must be positive and finite, got {}",
                option.friction_smoothing
            ),
        ));
    }

    // Gravity vector validation
    if !option.gravity.iter().all(|v| v.is_finite()) {
        return Err(MjcfError::invalid_option(
            "gravity",
            "must have finite components",
        ));
    }

    // Wind vector validation
    if !option.wind.iter().all(|v| v.is_finite()) {
        return Err(MjcfError::invalid_option(
            "wind",
            "must have finite components",
        ));
    }

    // Density validation (can be zero to disable)
    if !option.density.is_finite() || option.density < 0.0 {
        return Err(MjcfError::invalid_option(
            "density",
            format!("must be non-negative and finite, got {}", option.density),
        ));
    }

    // Viscosity validation
    if !option.viscosity.is_finite() || option.viscosity < 0.0 {
        return Err(MjcfError::invalid_option(
            "viscosity",
            format!("must be non-negative and finite, got {}", option.viscosity),
        ));
    }

    // Override validation (if set)
    if option.o_margin >= 0.0 && !option.o_margin.is_finite() {
        return Err(MjcfError::invalid_option(
            "o_margin",
            format!("must be finite when positive, got {}", option.o_margin),
        ));
    }

    Ok(())
}

/// Validate an MJCF model.
///
/// This checks:
/// - No duplicate body names
/// - No duplicate joint names
/// - No duplicate actuator names
/// - Actuators reference valid joints
/// - Valid mass properties (if present)
/// - Valid simulation options
///
/// # Errors
///
/// Returns an error if validation fails.
pub fn validate(model: &MjcfModel) -> Result<ValidationResult> {
    // Validate options first
    validate_option(&model.option)?;
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

/// Validate tendon definitions against model structure.
///
/// Returns Err on fatal issues (unknown references), matching the existing
/// validation pattern which uses `Result<()>` with `MjcfError`.
pub fn validate_tendons(model: &MjcfModel) -> Result<()> {
    // Collect joint, site, and geom names from the body tree (joints/sites/geoms
    // are nested inside MjcfBody, not top-level on MjcfModel).
    let mut joint_names = HashSet::new();
    let mut site_names = HashSet::new();
    let mut geom_names = HashSet::new();
    fn collect_names(
        body: &MjcfBody,
        joints: &mut HashSet<String>,
        sites: &mut HashSet<String>,
        geoms: &mut HashSet<String>,
    ) {
        for joint in &body.joints {
            joints.insert(joint.name.clone());
        }
        for site in &body.sites {
            sites.insert(site.name.clone());
        }
        for geom in &body.geoms {
            if let Some(ref name) = geom.name {
                geoms.insert(name.clone());
            }
        }
        for child in &body.children {
            collect_names(child, joints, sites, geoms);
        }
    }
    // Collect from worldbody itself (may have sites/geoms) and all children
    collect_names(
        &model.worldbody,
        &mut joint_names,
        &mut site_names,
        &mut geom_names,
    );

    for tendon in &model.tendons {
        // Fixed tendons must reference existing joints with valid coefficients
        if tendon.tendon_type == MjcfTendonType::Fixed {
            if tendon.joints.is_empty() {
                return Err(MjcfError::invalid_option(
                    "tendon",
                    format!("Fixed tendon '{}' has no joint entries", tendon.name),
                ));
            }
            for (joint_name, coef) in &tendon.joints {
                if !joint_names.contains(joint_name.as_str()) {
                    return Err(MjcfError::invalid_option(
                        "tendon",
                        format!(
                            "Tendon '{}' references unknown joint '{}'",
                            tendon.name, joint_name
                        ),
                    ));
                }
                if !coef.is_finite() {
                    return Err(MjcfError::invalid_option(
                        "tendon",
                        format!(
                            "Tendon '{}' has non-finite coefficient {} for joint '{}'",
                            tendon.name, coef, joint_name
                        ),
                    ));
                }
            }
        }

        // Spatial tendons: validate path_elements ordering, name resolution, and constraints
        if tendon.tendon_type == MjcfTendonType::Spatial {
            use crate::types::SpatialPathElement;

            let site_count = tendon
                .path_elements
                .iter()
                .filter(|e| matches!(e, SpatialPathElement::Site { .. }))
                .count();

            // Rule 3: must have at least 2 sites
            if site_count < 2 {
                return Err(MjcfError::invalid_option(
                    "tendon",
                    format!(
                        "Spatial tendon '{}' needs at least 2 sites, has {}",
                        tendon.name, site_count
                    ),
                ));
            }

            // Rule 1: path must start and end with a Site
            if let Some(first) = tendon.path_elements.first() {
                if !matches!(first, SpatialPathElement::Site { .. }) {
                    return Err(MjcfError::invalid_option(
                        "tendon",
                        format!(
                            "Spatial tendon '{}' path must start with a Site element",
                            tendon.name
                        ),
                    ));
                }
            }
            if let Some(last) = tendon.path_elements.last() {
                if !matches!(last, SpatialPathElement::Site { .. }) {
                    return Err(MjcfError::invalid_option(
                        "tendon",
                        format!(
                            "Spatial tendon '{}' path must end with a Site element",
                            tendon.name
                        ),
                    ));
                }
            }

            // Rules 2, 6: validate element adjacency
            for (i, elem) in tendon.path_elements.iter().enumerate() {
                match elem {
                    SpatialPathElement::Geom { .. } => {
                        // Rule 2: every Geom must be immediately followed by a Site
                        let next = tendon.path_elements.get(i + 1);
                        if !matches!(next, Some(SpatialPathElement::Site { .. })) {
                            return Err(MjcfError::invalid_option(
                                "tendon",
                                format!(
                                    "Spatial tendon '{}': Geom element at position {} \
                                     must be immediately followed by a Site element",
                                    tendon.name, i
                                ),
                            ));
                        }
                    }
                    SpatialPathElement::Pulley { divisor } => {
                        // Rule 7: divisor must be positive
                        if *divisor <= 0.0 {
                            return Err(MjcfError::invalid_option(
                                "tendon",
                                format!(
                                    "Spatial tendon '{}': Pulley at position {} \
                                     has non-positive divisor {}",
                                    tendon.name, i, divisor
                                ),
                            ));
                        }
                        // Rule 6: Pulley must not be immediately followed by a Geom
                        let next = tendon.path_elements.get(i + 1);
                        if matches!(next, Some(SpatialPathElement::Geom { .. })) {
                            return Err(MjcfError::invalid_option(
                                "tendon",
                                format!(
                                    "Spatial tendon '{}': Pulley at position {} \
                                     must not be immediately followed by a Geom element",
                                    tendon.name, i
                                ),
                            ));
                        }
                    }
                    SpatialPathElement::Site { .. } => {}
                }
            }

            // Rule 8: each pulley-delimited branch must contain at least 2 sites (warning)
            {
                let mut branch_start = 0;
                let mut branch_idx = 0usize;
                for (i, elem) in tendon.path_elements.iter().enumerate() {
                    if matches!(elem, SpatialPathElement::Pulley { .. }) {
                        let branch_sites = tendon.path_elements[branch_start..i]
                            .iter()
                            .filter(|e| matches!(e, SpatialPathElement::Site { .. }))
                            .count();
                        if branch_sites < 2 {
                            tracing::warn!(
                                "Spatial tendon '{}': branch {} (elements {}..{}) \
                                 has fewer than 2 sites",
                                tendon.name,
                                branch_idx,
                                branch_start,
                                i
                            );
                        }
                        branch_start = i + 1;
                        branch_idx += 1;
                    }
                }
                // Final branch (after last pulley or entire path if no pulleys)
                let branch_sites = tendon.path_elements[branch_start..]
                    .iter()
                    .filter(|e| matches!(e, SpatialPathElement::Site { .. }))
                    .count();
                if branch_sites < 2 {
                    tracing::warn!(
                        "Spatial tendon '{}': branch {} (elements {}..) \
                         has fewer than 2 sites",
                        tendon.name,
                        branch_idx,
                        branch_start
                    );
                }
            }

            // Rule 4: all referenced names must resolve
            for elem in &tendon.path_elements {
                match elem {
                    SpatialPathElement::Site { site } => {
                        if !site_names.contains(site.as_str()) {
                            return Err(MjcfError::invalid_option(
                                "tendon",
                                format!(
                                    "Tendon '{}' references unknown site '{}'",
                                    tendon.name, site
                                ),
                            ));
                        }
                    }
                    SpatialPathElement::Geom { geom, sidesite } => {
                        if !geom_names.contains(geom.as_str()) {
                            return Err(MjcfError::invalid_option(
                                "tendon",
                                format!(
                                    "Tendon '{}' references unknown geom '{}'",
                                    tendon.name, geom
                                ),
                            ));
                        }
                        if let Some(ss) = sidesite {
                            if !site_names.contains(ss.as_str()) {
                                return Err(MjcfError::invalid_option(
                                    "tendon",
                                    format!(
                                        "Tendon '{}' references unknown sidesite '{}'",
                                        tendon.name, ss
                                    ),
                                ));
                            }
                        }
                    }
                    SpatialPathElement::Pulley { .. } => {}
                }
            }
        }

        // Parameter validation
        if tendon.stiffness < 0.0 {
            return Err(MjcfError::invalid_option(
                "tendon",
                format!(
                    "Tendon '{}' has negative stiffness {}",
                    tendon.name, tendon.stiffness
                ),
            ));
        }
        if tendon.damping < 0.0 {
            return Err(MjcfError::invalid_option(
                "tendon",
                format!(
                    "Tendon '{}' has negative damping {}",
                    tendon.name, tendon.damping
                ),
            ));
        }
        if tendon.frictionloss < 0.0 {
            return Err(MjcfError::invalid_option(
                "tendon",
                format!(
                    "Tendon '{}' has negative frictionloss {}",
                    tendon.name, tendon.frictionloss
                ),
            ));
        }
        if tendon.limited {
            if let Some((min, max)) = tendon.range {
                if min >= max {
                    return Err(MjcfError::invalid_option(
                        "tendon",
                        format!(
                            "Tendon '{}' has invalid range [{}, {}]",
                            tendon.name, min, max
                        ),
                    ));
                }
            } else {
                return Err(MjcfError::invalid_option(
                    "tendon",
                    format!("Tendon '{}' is limited but has no range", tendon.name),
                ));
            }
        }
    }
    Ok(())
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

    // ========================================================================
    // Option validation tests
    // ========================================================================

    #[test]
    fn test_option_validation_valid() {
        let option = MjcfOption::default();
        assert!(validate_option(&option).is_ok());
    }

    #[test]
    fn test_option_validation_invalid_timestep() {
        let mut option = MjcfOption::default();

        // Negative timestep
        option.timestep = -0.001;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "timestep"
        ));

        // Zero timestep
        option.timestep = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "timestep"
        ));

        // Very large timestep
        option.timestep = 2.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "timestep"
        ));

        // NaN timestep
        option.timestep = f64::NAN;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "timestep"
        ));
    }

    #[test]
    fn test_option_validation_invalid_iterations() {
        let mut option = MjcfOption::default();
        option.iterations = 0;

        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "iterations"
        ));
    }

    #[test]
    fn test_option_validation_invalid_tolerance() {
        let mut option = MjcfOption::default();

        option.tolerance = -1e-8;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "tolerance"
        ));

        option.tolerance = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "tolerance"
        ));
    }

    #[test]
    fn test_option_validation_invalid_impratio() {
        let mut option = MjcfOption::default();

        option.impratio = -1.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "impratio"
        ));

        option.impratio = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "impratio"
        ));
    }

    #[test]
    fn test_option_validation_invalid_gravity() {
        let mut option = MjcfOption::default();
        option.gravity = Vector3::new(f64::NAN, 0.0, -9.81);

        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "gravity"
        ));
    }

    #[test]
    fn test_option_validation_invalid_density() {
        let mut option = MjcfOption::default();
        option.density = -1.0;

        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "density"
        ));
    }

    #[test]
    fn test_option_validation_invalid_viscosity() {
        let mut option = MjcfOption::default();
        option.viscosity = -0.1;

        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "viscosity"
        ));
    }

    #[test]
    fn test_option_validation_invalid_solver_tuning() {
        let mut option = MjcfOption::default();

        // regularization
        option.regularization = -1.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "regularization"
        ));
        option.regularization = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "regularization"
        ));
        option.regularization = 1e-6; // restore

        // default_eq_stiffness
        option.default_eq_stiffness = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "default_eq_stiffness"
        ));
        option.default_eq_stiffness = f64::NAN;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "default_eq_stiffness"
        ));
        option.default_eq_stiffness = 10000.0; // restore

        // default_eq_damping
        option.default_eq_damping = -100.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "default_eq_damping"
        ));
        option.default_eq_damping = 1000.0; // restore

        // max_constraint_vel
        option.max_constraint_vel = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "max_constraint_vel"
        ));
        option.max_constraint_vel = f64::INFINITY;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "max_constraint_vel"
        ));
        option.max_constraint_vel = 1.0; // restore

        // max_constraint_angvel
        option.max_constraint_angvel = -0.5;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "max_constraint_angvel"
        ));
        option.max_constraint_angvel = 1.0; // restore

        // friction_smoothing
        option.friction_smoothing = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidOption { option, .. }) if option == "friction_smoothing"
        ));
    }
}
