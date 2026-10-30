//! MJCF validation utilities.
//!
//! Validates the kinematic structure and physical properties of parsed MJCF.

use std::collections::{HashMap, HashSet};

use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::stack::MAX_BODY_TREE_DEPTH;
use crate::types::{
    MjcfBody, MjcfFrame, MjcfGeom, MjcfModel, MjcfOption, MjcfTendon, MjcfTendonType,
    SpatialPathElement, TfAuto,
};

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
    let finite = |attribute: &str, value: f64| -> Result<()> {
        if value.is_finite() {
            Ok(())
        } else {
            Err(MjcfError::non_finite("option", attribute, value))
        }
    };
    let positive = |attribute: &str, value: f64| -> Result<()> {
        finite(attribute, value)?;
        if value > 0.0 {
            Ok(())
        } else {
            Err(MjcfError::invalid_value(
                "option",
                attribute,
                format!("must be positive, got {value}"),
            ))
        }
    };
    let non_negative = |attribute: &str, value: f64| -> Result<()> {
        finite(attribute, value)?;
        if value >= 0.0 {
            Ok(())
        } else {
            Err(MjcfError::invalid_value(
                "option",
                attribute,
                format!("must be non-negative, got {value}"),
            ))
        }
    };

    positive("timestep", option.timestep)?;
    if option.timestep > 1.0 {
        return Err(MjcfError::invalid_value(
            "option",
            "timestep",
            format!(
                "value {} is unusually large (> 1 second), likely an error",
                option.timestep
            ),
        ));
    }
    if option.iterations == 0 {
        return Err(MjcfError::invalid_value(
            "option",
            "iterations",
            "must be at least 1",
        ));
    }
    positive("tolerance", option.tolerance)?;
    positive("impratio", option.impratio)?;
    positive("regularization", option.regularization)?;
    positive("friction_smoothing", option.friction_smoothing)?;
    for &v in option.gravity.iter() {
        finite("gravity", v)?;
    }
    for &v in option.wind.iter() {
        finite("wind", v)?;
    }
    // Density can be zero, which disables it.
    non_negative("density", option.density)?;
    non_negative("viscosity", option.viscosity)?;
    // The override margin, when set (non-negative).
    if option.o_margin >= 0.0 {
        finite("o_margin", option.o_margin)?;
    }
    Ok(())
}

/// Validate an MJCF model, as the loader validates it.
///
/// The model is checked once every element has its class and its frames and
/// composites are expanded, before `discardvisual` and `fusestatic` remove
/// any element (MuJoCo 3.5.0 checks names before it fuses or discards,
/// measured). The result describes that model, the bodies a frame or a
/// composite makes among its bodies.
///
/// This checks:
/// - No duplicate body names
/// - No duplicate joint names
/// - No duplicate actuator names
/// - Actuators reference valid joints
/// - Valid mass properties (if present)
/// - Valid simulation options
/// - Tendons reference valid joints, sites and geoms, with valid parameters
/// - A body and frame tree no deeper than the loader takes
///
/// # Errors
///
/// Returns an error if validation fails, or if the model is refused before
/// it (an unknown class, an element its frame or composite refuses).
pub fn validate(model: &MjcfModel) -> Result<ValidationResult> {
    crate::stack::on_large_stack(|| {
        check_tree_depth(&model.worldbody)?;
        crate::builder::front_end(model).map(|(_, _, checked)| checked)
    })
}

/// [`validate`]'s checks, on a model whose classes are applied and whose
/// frames and composites are expanded (`builder::front_end`).
pub fn validate_resolved(model: &MjcfModel) -> Result<ValidationResult> {
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

    // Validate geoms anchored directly on worldbody (e.g., ground planes).
    // Descendant-body geoms are validated inside traverse_body.
    for geom in &model.worldbody.geoms {
        check_geom_finite(geom, "world")?;
    }

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
        // Check for duplicate body name. Anonymous bodies (empty name) are
        // exempt — mirrors the joint-name guard below and the builder's
        // is_empty() contract for body name lookup.
        if !body.name.is_empty() && !body_names.insert(body.name.clone()) {
            return Err(MjcfError::duplicate_name(ElementKind::Body, &body.name));
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
                    return Err(MjcfError::duplicate_name(ElementKind::Joint, &joint.name)
                        .at(Location::named(ElementKind::Joint, &joint.name).in_body(&body.name)));
                }
                joint_to_body.insert(joint.name.clone(), body.name.clone());
                all_joint_names.push(joint.name.clone());
            }
        }

        // Validate inertial if present
        if let Some(ref inertial) = body.inertial {
            // Check mass is positive
            let at = || Location::named(ElementKind::Body, &body.name);
            if inertial.mass <= 0.0 {
                return Err(MjcfError::invalid_value(
                    "inertial",
                    "mass",
                    format!("must be positive, got {}", inertial.mass),
                )
                .at(at()));
            }
            if !inertial.mass.is_finite() {
                return Err(MjcfError::non_finite("inertial", "mass", inertial.mass).at(at()));
            }

            // Check inertia tensor if specified
            if let Some(diag) = inertial.diaginertia {
                if diag.x < 0.0 || diag.y < 0.0 || diag.z < 0.0 {
                    return Err(MjcfError::invalid_value(
                        "inertial",
                        "diaginertia",
                        "diagonal elements must be non-negative",
                    )
                    .at(at()));
                }
                if let Some(v) = diag.iter().find(|v| !v.is_finite()) {
                    return Err(MjcfError::non_finite("inertial", "diaginertia", *v).at(at()));
                }
            }
        }

        // Validate geom dimensions
        for geom in &body.geoms {
            check_geom_finite(geom, &body.name)?;
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

    // Collect site names from body tree for actuator validation.
    let mut site_names_set = HashSet::new();
    fn collect_site_names(body: &MjcfBody, names: &mut HashSet<String>) {
        for site in &body.sites {
            names.insert(site.name.clone());
        }
        for child in &body.children {
            collect_site_names(child, names);
        }
    }
    for site in &model.worldbody.sites {
        site_names_set.insert(site.name.clone());
    }
    for body in &model.worldbody.children {
        collect_site_names(body, &mut site_names_set);
    }

    // Check actuators
    let mut all_actuator_names = Vec::new();
    for (i, actuator) in model.actuators.iter().enumerate() {
        let at = || Location::nth(ElementKind::Actuator, Some(&actuator.name), i);
        if !actuator.name.is_empty() {
            if !actuator_names.insert(actuator.name.clone()) {
                return Err(MjcfError::duplicate_name(
                    ElementKind::Actuator,
                    &actuator.name,
                ));
            }
            all_actuator_names.push(actuator.name.clone());
        }

        // Check that joint reference is valid
        if let Some(ref joint_name) = actuator.joint
            && !joint_names.contains(joint_name)
        {
            return Err(MjcfError::undefined(ElementKind::Joint, joint_name).at(at()));
        }

        // Check that site reference is valid
        if let Some(ref site_name) = actuator.site
            && !site_names_set.contains(site_name.as_str())
        {
            return Err(MjcfError::undefined(ElementKind::Site, site_name).at(at()));
        }

        // Check that refsite reference is valid
        if let Some(ref refsite_name) = actuator.refsite
            && !site_names_set.contains(refsite_name.as_str())
        {
            return Err(MjcfError::undefined(ElementKind::Site, refsite_name).at(at()));
        }

        // Check mutual exclusivity of transmission targets
        let target_count = [
            actuator.joint.is_some(),
            actuator.tendon.is_some(),
            actuator.site.is_some(),
            actuator.body.is_some(),
        ]
        .iter()
        .filter(|&&v| v)
        .count();
        if target_count > 1 {
            return Err(
                MjcfError::conflicting("actuator", &["joint", "tendon", "site", "body"]).at(at()),
            );
        }
    }

    check_tendons(model)?;

    Ok(ValidationResult {
        sorted_bodies,
        body_parent,
        body_children,
        joint_names: all_joint_names,
        joint_to_body,
        actuator_names: all_actuator_names,
    })
}

/// Refuses a tree of bodies and frames below `worldbody` deeper than
/// [`MAX_BODY_TREE_DEPTH`], counting each body and each frame as a level,
/// read without recursing: every pass after it recurses once per level.
pub fn check_tree_depth(worldbody: &MjcfBody) -> Result<()> {
    enum Node<'a> {
        Body(&'a MjcfBody),
        Frame(&'a MjcfFrame),
    }
    let mut stack: Vec<(Node<'_>, usize)> = vec![(Node::Body(worldbody), 0)];
    while let Some((node, depth)) = stack.pop() {
        if depth > MAX_BODY_TREE_DEPTH {
            return Err(MjcfError::too_deep(MAX_BODY_TREE_DEPTH));
        }
        let (bodies, frames) = match node {
            Node::Body(b) => (&b.children, &b.frames),
            Node::Frame(f) => (&f.bodies, &f.frames),
        };
        stack.extend(bodies.iter().map(|b| (Node::Body(b), depth + 1)));
        stack.extend(frames.iter().map(|f| (Node::Frame(f), depth + 1)));
    }
    Ok(())
}

/// Check that a geom's dimensional numeric fields are finite.
///
/// Rejects NaN or ±Inf in the user-supplied `size` vector and the optional
/// `fromto` array at the validation boundary so that downstream mass/volume
/// computation, mesh construction, and collision code can assume finite
/// geometry. Other numeric fields (orientation, friction, density, solver
/// params) are out of scope here; mass/inertia have separate validation
/// paths via the inertial check, and the rest are deferred as a follow-up
/// audit candidate.
fn check_geom_finite(geom: &MjcfGeom, body_name: &str) -> Result<()> {
    let at = || {
        Location::named(ElementKind::Geom, geom.name.as_deref().unwrap_or("")).in_body(body_name)
    };
    // A geom's `fromto` is its pose and size already (the resolve pass refuses
    // a non-finite one).
    if let Some(v) = geom.size.iter().find(|v| !v.is_finite()) {
        return Err(MjcfError::non_finite("geom", "size", *v).at(at()));
    }
    Ok(())
}

/// Each tendon's references to joints, sites and geoms, and its parameters.
fn check_tendons(model: &MjcfModel) -> Result<()> {
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

    for (i, tendon) in model.tendons.iter().enumerate() {
        let checked = match tendon.tendon_type {
            MjcfTendonType::Fixed => validate_fixed_tendon(tendon, i, &joint_names),
            MjcfTendonType::Spatial => validate_spatial_tendon(tendon, &site_names, &geom_names),
        }
        .and_then(|()| validate_tendon_params(tendon));
        checked.map_err(|e| {
            e.with_location_if_unknown(Location::nth(ElementKind::Tendon, Some(&tendon.name), i))
        })?;
    }
    Ok(())
}

/// Validate a fixed tendon: joints must exist and coefficients must be finite.
fn validate_fixed_tendon(
    tendon: &MjcfTendon,
    index: usize,
    joint_names: &HashSet<String>,
) -> Result<()> {
    if tendon.joints.is_empty() {
        return Err(MjcfError::invalid_model(
            "fixed tendon has no joint entries",
        ));
    }
    for (joint_name, coef) in &tendon.joints {
        if !joint_names.contains(joint_name.as_str()) {
            return Err(MjcfError::undefined(ElementKind::Joint, joint_name));
        }
        if !coef.is_finite() {
            return Err(
                MjcfError::non_finite("joint", "coef", *coef).at(Location::named(
                    ElementKind::Joint,
                    joint_name,
                )
                .within_nth(ElementKind::Tendon, Some(&tendon.name), index)),
            );
        }
    }
    Ok(())
}

/// Validate a spatial tendon: path structure, adjacency rules, and name resolution.
fn validate_spatial_tendon(
    tendon: &MjcfTendon,
    site_names: &HashSet<String>,
    geom_names: &HashSet<String>,
) -> Result<()> {
    let site_count = tendon
        .path_elements
        .iter()
        .filter(|e| matches!(e, SpatialPathElement::Site { .. }))
        .count();

    // Rule 3: must have at least 2 sites
    if site_count < 2 {
        return Err(MjcfError::invalid_model(format!(
            "spatial tendon needs at least 2 sites, has {site_count}"
        )));
    }

    // Rule 1: path must start and end with a Site
    if let Some(first) = tendon.path_elements.first()
        && !matches!(first, SpatialPathElement::Site { .. })
    {
        return Err(MjcfError::invalid_model(
            "spatial tendon path must start with a Site element",
        ));
    }
    if let Some(last) = tendon.path_elements.last()
        && !matches!(last, SpatialPathElement::Site { .. })
    {
        return Err(MjcfError::invalid_model(
            "spatial tendon path must end with a Site element",
        ));
    }

    // Rules 2, 6: validate element adjacency
    for (i, elem) in tendon.path_elements.iter().enumerate() {
        match elem {
            SpatialPathElement::Geom { .. } => {
                // Rule 2: every Geom must be immediately followed by a Site
                let next = tendon.path_elements.get(i + 1);
                if !matches!(next, Some(SpatialPathElement::Site { .. })) {
                    return Err(MjcfError::invalid_model(format!(
                        "spatial tendon: Geom element at position {i} \
                         must be immediately followed by a Site element"
                    )));
                }
            }
            SpatialPathElement::Pulley { divisor } => {
                // Rule 7: divisor must be positive
                if *divisor <= 0.0 {
                    return Err(MjcfError::invalid_value(
                        "pulley",
                        "divisor",
                        format!("Pulley at position {i} has non-positive divisor {divisor}"),
                    ));
                }
                // Rule 6: Pulley must not be immediately followed by a Geom
                let next = tendon.path_elements.get(i + 1);
                if matches!(next, Some(SpatialPathElement::Geom { .. })) {
                    return Err(MjcfError::invalid_model(format!(
                        "spatial tendon: Pulley at position {i} \
                         must not be immediately followed by a Geom element"
                    )));
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
                    return Err(MjcfError::undefined(ElementKind::Site, site));
                }
            }
            SpatialPathElement::Geom { geom, sidesite } => {
                if !geom_names.contains(geom.as_str()) {
                    return Err(MjcfError::undefined(ElementKind::Geom, geom));
                }
                if let Some(ss) = sidesite
                    && !site_names.contains(ss.as_str())
                {
                    return Err(MjcfError::undefined(ElementKind::Site, ss));
                }
            }
            SpatialPathElement::Pulley { .. } => {}
        }
    }

    Ok(())
}

/// Validate tendon physical parameters: frictionloss, range. A negative
/// stiffness or damping loads, as MuJoCo 3.5.0 loads and applies it
/// (`engine_passive.c:453-473`); a negative frictionloss does not, as
/// MuJoCo makes no constraint row for one (`engine_core_constraint.c:737`).
fn validate_tendon_params(tendon: &MjcfTendon) -> Result<()> {
    let frictionloss = tendon.frictionloss.unwrap_or(0.0);
    if frictionloss < 0.0 {
        return Err(MjcfError::invalid_value(
            "tendon",
            "frictionloss",
            format!("negative frictionloss {frictionloss}"),
        ));
    }
    if tendon.limited == Some(TfAuto::True) {
        if let Some((min, max)) = tendon.range {
            if min >= max {
                return Err(MjcfError::invalid_value(
                    "tendon",
                    "range",
                    format!("invalid range [{min}, {max}]"),
                ));
            }
        } else {
            return Err(MjcfError::invalid_value(
                "tendon",
                "range",
                "the tendon is limited but has no range",
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::types::{MjcfActuator, MjcfGeom, MjcfGeomType, MjcfInertial, MjcfJoint};
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
        assert!(matches!(
            result,
            Err(MjcfError::DuplicateName {
                kind: ElementKind::Body,
                ..
            })
        ));
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
        assert!(matches!(
            result,
            Err(MjcfError::DuplicateName {
                kind: ElementKind::Joint,
                ..
            })
        ));
    }

    #[test]
    fn test_actuator_invalid_joint() {
        let model = MjcfModel::new("test")
            .with_body(MjcfBody::new("base").with_joint(MjcfJoint::hinge("joint1", Vector3::z())))
            .with_actuator(MjcfActuator::motor("motor1", "nonexistent_joint"));

        let result = validate(&model);
        assert!(matches!(
            result,
            Err(MjcfError::UndefinedReference {
                kind: ElementKind::Joint,
                ..
            })
        ));
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
        assert!(matches!(
            result,
            Err(MjcfError::InvalidValue { ref attribute, .. }) if attribute == "mass"
        ));
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
        assert!(matches!(
            result,
            Err(MjcfError::InvalidValue { ref attribute, .. }) if attribute == "diaginertia"
        ));
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
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "timestep"
        ));

        // Zero timestep
        option.timestep = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "timestep"
        ));

        // Very large timestep
        option.timestep = 2.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "timestep"
        ));

        // NaN timestep
        option.timestep = f64::NAN;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::NonFinite { attribute, .. }) if attribute == "timestep"
        ));
    }

    #[test]
    fn test_option_validation_invalid_iterations() {
        let mut option = MjcfOption::default();
        option.iterations = 0;

        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "iterations"
        ));
    }

    #[test]
    fn test_option_validation_invalid_tolerance() {
        let mut option = MjcfOption::default();

        option.tolerance = -1e-8;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "tolerance"
        ));

        option.tolerance = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "tolerance"
        ));
    }

    #[test]
    fn test_option_validation_invalid_impratio() {
        let mut option = MjcfOption::default();

        option.impratio = -1.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "impratio"
        ));

        option.impratio = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "impratio"
        ));
    }

    #[test]
    fn test_option_validation_invalid_gravity() {
        let mut option = MjcfOption::default();
        option.gravity = Vector3::new(f64::NAN, 0.0, -9.81);

        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::NonFinite { attribute, .. }) if attribute == "gravity"
        ));
    }

    #[test]
    fn test_option_validation_invalid_density() {
        let mut option = MjcfOption::default();
        option.density = -1.0;

        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "density"
        ));
    }

    #[test]
    fn test_option_validation_invalid_viscosity() {
        let mut option = MjcfOption::default();
        option.viscosity = -0.1;

        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "viscosity"
        ));
    }

    #[test]
    fn test_option_validation_invalid_solver_tuning() {
        let mut option = MjcfOption::default();

        // regularization
        option.regularization = -1.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "regularization"
        ));
        option.regularization = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "regularization"
        ));
        option.regularization = 1e-6; // restore

        // friction_smoothing
        option.friction_smoothing = 0.0;
        assert!(matches!(
            validate_option(&option),
            Err(MjcfError::InvalidValue { attribute, .. }) if attribute == "friction_smoothing"
        ));
    }

    // ========================================================================
    // Geom-finite validation tests
    // ========================================================================

    fn body_with_geom(name: &str, geom: MjcfGeom) -> MjcfBody {
        MjcfBody::new(name).with_geom(geom)
    }

    /// Asserts that `model` is refused for a non-finite `attribute` of an
    /// unnamed geom in `body`.
    fn assert_geom_non_finite(model: &MjcfModel, attribute: &str, body: &str) {
        let err = validate(model).unwrap_err();
        assert!(
            matches!(&err, MjcfError::NonFinite { element, attribute: a, .. }
                if element == "geom" && a == attribute),
            "{err}"
        );
        assert!(
            err.to_string()
                .ends_with(&format!("(geom in body '{body}')")),
            "{err}"
        );
    }

    #[test]
    fn test_invalid_geom_sphere_nan() {
        let model =
            MjcfModel::new("test").with_body(body_with_geom("base", MjcfGeom::sphere(f64::NAN)));
        assert_geom_non_finite(&model, "size", "base");
    }

    #[test]
    fn test_invalid_geom_box_nan() {
        let model = MjcfModel::new("test").with_body(body_with_geom(
            "base",
            MjcfGeom::box_shape(Vector3::new(1.0, f64::NAN, 1.0)),
        ));
        assert_geom_non_finite(&model, "size", "base");
    }

    #[test]
    fn test_invalid_geom_capsule_pos_inf() {
        let model = MjcfModel::new("test").with_body(body_with_geom(
            "base",
            MjcfGeom::capsule(f64::INFINITY, 0.5),
        ));
        assert_geom_non_finite(&model, "size", "base");
    }

    #[test]
    fn test_invalid_geom_cylinder_neg_inf() {
        // Build a cylinder via the size vec directly (no constructor sugar).
        let cylinder = MjcfGeom {
            geom_type: Some(MjcfGeomType::Cylinder),
            size: vec![0.1, f64::NEG_INFINITY],
            ..Default::default()
        };
        let model = MjcfModel::new("test").with_body(body_with_geom("base", cylinder));
        assert_geom_non_finite(&model, "size", "base");
    }

    #[test]
    fn test_invalid_geom_fromto_nan() {
        let geom = MjcfGeom {
            geom_type: Some(MjcfGeomType::Cylinder),
            size: vec![0.1, 0.5],
            fromto: Some([0.0, 0.0, 0.0, 1.0, f64::NAN, 0.0]),
            ..Default::default()
        };
        let model = MjcfModel::new("test").with_body(body_with_geom("base", geom));
        assert_geom_non_finite(&model, "fromto", "base");
    }

    #[test]
    fn test_invalid_geom_in_nested_body() {
        // Deep-nested body with a NaN geom is still caught.
        let model = MjcfModel::new("test").with_body(MjcfBody::new("root").with_child(
            MjcfBody::new("link1").with_child(body_with_geom("link2", MjcfGeom::sphere(f64::NAN))),
        ));
        assert_geom_non_finite(&model, "size", "link2");
    }

    #[test]
    fn test_invalid_geom_on_worldbody() {
        // Worldbody-anchored geoms (e.g., ground planes) are also validated.
        let mut model = MjcfModel::new("test");
        model.worldbody.geoms.push(MjcfGeom::sphere(f64::NAN));
        assert_geom_non_finite(&model, "size", "world");
    }

    #[test]
    fn test_valid_geoms_pass() {
        // Mixed geom types with finite dimensions across nested bodies + worldbody.
        let mut model = MjcfModel::new("test").with_body(
            MjcfBody::new("base")
                .with_geom(MjcfGeom::sphere(0.1))
                .with_geom(MjcfGeom::box_shape(Vector3::new(0.2, 0.3, 0.4)))
                .with_child(MjcfBody::new("link1").with_geom(MjcfGeom::capsule(0.05, 0.5))),
        );
        model
            .worldbody
            .geoms
            .push(MjcfGeom::box_shape(Vector3::new(10.0, 10.0, 0.01)));
        assert!(validate(&model).is_ok());
    }
}
