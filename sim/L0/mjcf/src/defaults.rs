//! Default class resolution and application.
//!
//! This module implements MuJoCo's default class inheritance system, allowing
//! elements to inherit default values from named classes with support for
//! hierarchical inheritance.
//!
//! # MuJoCo Default System
//!
//! In MJCF, the `<default>` element defines named classes of default values.
//! Elements can reference a class via the `class` attribute, and will inherit
//! all applicable defaults from that class. Default classes can also inherit
//! from parent classes, forming an inheritance hierarchy.
//!
//! # Example
//!
//! ```xml
//! <default>
//!     <joint damping="0.5"/>
//!     <default class="arm">
//!         <joint damping="1.0" armature="0.1"/>
//!         <geom rgba="0.8 0.2 0.2 1"/>
//!     </default>
//! </default>
//! ```
//!
//! A joint with `class="arm"` would inherit:
//! - `damping="1.0"` (from "arm" class, overriding root)
//! - `armature="0.1"` (from "arm" class)
//!
//! A joint without a class would inherit:
//! - `damping="0.5"` (from root defaults)

use std::collections::HashMap;

use crate::types::{
    MjcfActuator, MjcfActuatorDefaults, MjcfContactPair, MjcfDefault, MjcfGeom, MjcfGeomDefaults,
    MjcfJoint, MjcfJointDefaults, MjcfMeshDefaults, MjcfModel, MjcfPairDefaults, MjcfSensor,
    MjcfSensorDefaults, MjcfSite, MjcfSiteDefaults, MjcfTendon, MjcfTendonDefaults,
};

/// Resolves and applies default classes to MJCF elements.
///
/// The resolver builds an inheritance chain for each default class and
/// provides methods to apply resolved defaults to joints, geoms, actuators,
/// tendons, and sensors.
#[derive(Debug, Clone)]
pub struct DefaultResolver {
    /// Map from class name to resolved defaults (with inheritance applied).
    resolved_defaults: HashMap<String, MjcfDefault>,
}

impl DefaultResolver {
    /// Create a new resolver from a list of default classes.
    ///
    /// This builds the inheritance hierarchy and resolves all defaults.
    #[must_use]
    pub fn new(defaults: &[MjcfDefault]) -> Self {
        let resolved_defaults = Self::resolve_all(defaults);
        Self { resolved_defaults }
    }

    /// Create a resolver from an MJCF model.
    #[must_use]
    pub fn from_model(model: &MjcfModel) -> Self {
        Self::new(&model.defaults)
    }

    /// Get the resolved defaults for a class name.
    ///
    /// Returns `None` if the class name is not found in the resolved defaults.
    #[must_use]
    pub fn get_defaults(&self, class: Option<&str>) -> Option<&MjcfDefault> {
        let class_name = class.unwrap_or("");
        self.resolved_defaults.get(class_name)
    }

    /// Get the resolved joint defaults for a class.
    #[must_use]
    pub fn joint_defaults(&self, class: Option<&str>) -> Option<&MjcfJointDefaults> {
        self.get_defaults(class).and_then(|d| d.joint.as_ref())
    }

    /// Get the resolved geom defaults for a class.
    #[must_use]
    pub fn geom_defaults(&self, class: Option<&str>) -> Option<&MjcfGeomDefaults> {
        self.get_defaults(class).and_then(|d| d.geom.as_ref())
    }

    /// Get the resolved actuator defaults for a class.
    #[must_use]
    pub fn actuator_defaults(&self, class: Option<&str>) -> Option<&MjcfActuatorDefaults> {
        self.get_defaults(class).and_then(|d| d.actuator.as_ref())
    }

    /// Get the resolved tendon defaults for a class.
    #[must_use]
    pub fn tendon_defaults(&self, class: Option<&str>) -> Option<&MjcfTendonDefaults> {
        self.get_defaults(class).and_then(|d| d.tendon.as_ref())
    }

    /// Get the resolved sensor defaults for a class.
    #[must_use]
    pub fn sensor_defaults(&self, class: Option<&str>) -> Option<&MjcfSensorDefaults> {
        self.get_defaults(class).and_then(|d| d.sensor.as_ref())
    }

    /// Get the resolved mesh defaults for a class.
    #[must_use]
    pub fn mesh_defaults(&self, class: Option<&str>) -> Option<&MjcfMeshDefaults> {
        self.get_defaults(class).and_then(|d| d.mesh.as_ref())
    }

    /// Get the resolved site defaults for a class.
    #[must_use]
    pub fn site_defaults(&self, class: Option<&str>) -> Option<&MjcfSiteDefaults> {
        self.get_defaults(class).and_then(|d| d.site.as_ref())
    }

    /// Get the resolved pair defaults for a class.
    #[must_use]
    pub fn pair_defaults(&self, class: Option<&str>) -> Option<&MjcfPairDefaults> {
        self.get_defaults(class).and_then(|d| d.pair.as_ref())
    }

    /// Apply defaults to a joint, returning a new joint with defaults applied.
    ///
    /// Values explicitly set on the joint take precedence over defaults.
    #[must_use]
    pub fn apply_to_joint(&self, joint: &MjcfJoint) -> MjcfJoint {
        let mut result = joint.clone();

        if let Some(defaults) = self.joint_defaults(joint.class.as_deref()) {
            // Only apply defaults where the joint doesn't have an explicit value
            // Joint type: only if still at default
            if result.joint_type == crate::types::MjcfJointType::Hinge {
                if let Some(jtype) = defaults.joint_type {
                    result.joint_type = jtype;
                }
            }

            // Axis: only if still at default [0, 0, 1]
            if result.axis == nalgebra::Vector3::z() {
                if let Some(axis) = defaults.axis {
                    result.axis = axis;
                }
            }

            // Limited: apply default only if not explicitly set on element
            if result.limited.is_none() {
                if let Some(limited) = defaults.limited {
                    result.limited = Some(limited);
                }
            }

            // Damping: apply default if zero (assumed not explicitly set)
            if result.damping == 0.0 {
                if let Some(damping) = defaults.damping {
                    result.damping = damping;
                }
            }

            // Stiffness: apply default if zero
            if result.stiffness == 0.0 {
                if let Some(stiffness) = defaults.stiffness {
                    result.stiffness = stiffness;
                }
            }

            // Armature: apply default if zero
            if result.armature == 0.0 {
                if let Some(armature) = defaults.armature {
                    result.armature = armature;
                }
            }
        }

        result
    }

    /// Apply defaults to a geom, returning a new geom with defaults applied.
    #[must_use]
    pub fn apply_to_geom(&self, geom: &MjcfGeom) -> MjcfGeom {
        let mut result = geom.clone();

        if let Some(defaults) = self.geom_defaults(geom.class.as_deref()) {
            // Geom type: only if still at default (Sphere)
            if result.geom_type == crate::types::MjcfGeomType::Sphere {
                if let Some(gtype) = defaults.geom_type {
                    result.geom_type = gtype;
                }
            }

            // Friction: apply default if at MuJoCo default values
            if (result.friction - nalgebra::Vector3::new(1.0, 0.005, 0.0001)).norm() < 1e-10 {
                if let Some(friction) = defaults.friction {
                    result.friction = friction;
                }
            }

            // Density: apply default if at default (1000.0)
            if (result.density - 1000.0).abs() < 1e-10 {
                if let Some(density) = defaults.density {
                    result.density = density;
                }
            }

            // Mass: apply default if None
            if result.mass.is_none() {
                result.mass = defaults.mass;
            }

            // RGBA: apply default if at default gray
            if (result.rgba - nalgebra::Vector4::new(0.5, 0.5, 0.5, 1.0)).norm() < 1e-10 {
                if let Some(rgba) = defaults.rgba {
                    result.rgba = rgba;
                }
            }

            // Contype: apply default if at default (1)
            if result.contype == 1 {
                if let Some(contype) = defaults.contype {
                    result.contype = contype;
                }
            }

            // Conaffinity: apply default if at default (1)
            if result.conaffinity == 1 {
                if let Some(conaffinity) = defaults.conaffinity {
                    result.conaffinity = conaffinity;
                }
            }
        }

        result
    }

    /// Apply defaults to an actuator, returning a new actuator with defaults applied.
    #[must_use]
    pub fn apply_to_actuator(&self, actuator: &MjcfActuator) -> MjcfActuator {
        let mut result = actuator.clone();

        if let Some(defaults) = self.actuator_defaults(actuator.class.as_deref()) {
            // Gear: apply default if at default [1, 0, 0, 0, 0, 0]
            let default_gear = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            if result
                .gear
                .iter()
                .zip(default_gear.iter())
                .all(|(a, b)| (a - b).abs() < 1e-10)
            {
                if let Some(gear) = defaults.gear {
                    result.gear = gear;
                }
            }

            // Control range: apply default if None
            if result.ctrlrange.is_none() {
                result.ctrlrange = defaults.ctrlrange;
            }

            // Force range: apply default if None
            if result.forcerange.is_none() {
                result.forcerange = defaults.forcerange;
            }

            // Control limited: apply default only if not explicitly set on element
            if result.ctrllimited.is_none() {
                if let Some(ctrllimited) = defaults.ctrllimited {
                    result.ctrllimited = Some(ctrllimited);
                }
            }

            // Force limited: apply default only if not explicitly set on element
            if result.forcelimited.is_none() {
                if let Some(forcelimited) = defaults.forcelimited {
                    result.forcelimited = Some(forcelimited);
                }
            }

            // kp: apply default if at default (1.0)
            if (result.kp - 1.0).abs() < 1e-10 {
                if let Some(kp) = defaults.kp {
                    result.kp = kp;
                }
            }

            // kv: apply default if not explicitly set
            if result.kv.is_none() {
                if let Some(kv) = defaults.kv {
                    result.kv = Some(kv);
                }
            }

            // Apply <general>-specific defaults.
            // These only matter for <general> actuators, but we apply them
            // unconditionally — the model builder ignores them for shortcut types.
            if result.gaintype.is_none() {
                result.gaintype.clone_from(&defaults.gaintype);
            }
            if result.biastype.is_none() {
                result.biastype.clone_from(&defaults.biastype);
            }
            if result.dyntype.is_none() {
                result.dyntype.clone_from(&defaults.dyntype);
            }
            if result.gainprm.is_none() {
                result.gainprm.clone_from(&defaults.gainprm);
            }
            if result.biasprm.is_none() {
                result.biasprm.clone_from(&defaults.biasprm);
            }
            if result.dynprm.is_none() {
                result.dynprm.clone_from(&defaults.dynprm);
            }
        }

        result
    }

    /// Apply defaults to a site, returning a new site with defaults applied.
    #[must_use]
    pub fn apply_to_site(&self, site: &MjcfSite) -> MjcfSite {
        let mut result = site.clone();

        // Sites don't have a class field in the current implementation,
        // so we apply root defaults
        if let Some(defaults) = self.site_defaults(None) {
            // Type: apply default if at default sphere
            if result.site_type == "sphere" {
                if let Some(site_type) = &defaults.site_type {
                    result.site_type.clone_from(site_type);
                }
            }

            // Size: apply default if at default
            if result.size == vec![0.01] {
                if let Some(size) = &defaults.size {
                    result.size.clone_from(size);
                }
            }

            // RGBA: apply default if at default red
            if (result.rgba - nalgebra::Vector4::new(1.0, 0.0, 0.0, 1.0)).norm() < 1e-10 {
                if let Some(rgba) = defaults.rgba {
                    result.rgba = rgba;
                }
            }
        }

        result
    }

    /// Apply defaults to a tendon, returning a new tendon with defaults applied.
    ///
    /// Values explicitly set on the tendon take precedence over defaults.
    #[must_use]
    pub fn apply_to_tendon(&self, tendon: &MjcfTendon) -> MjcfTendon {
        let mut result = tendon.clone();

        if let Some(defaults) = self.tendon_defaults(tendon.class.as_deref()) {
            // Range: apply default if None
            if result.range.is_none() {
                result.range = defaults.range;
            }

            // Limited: apply default only if not explicitly set on element
            if result.limited.is_none() {
                if let Some(limited) = defaults.limited {
                    result.limited = Some(limited);
                }
            }

            // Stiffness: apply default if zero
            if result.stiffness == 0.0 {
                if let Some(stiffness) = defaults.stiffness {
                    result.stiffness = stiffness;
                }
            }

            // Damping: apply default if zero
            if result.damping == 0.0 {
                if let Some(damping) = defaults.damping {
                    result.damping = damping;
                }
            }

            // Friction loss: apply default if zero
            if result.frictionloss == 0.0 {
                if let Some(frictionloss) = defaults.frictionloss {
                    result.frictionloss = frictionloss;
                }
            }

            // Width: apply default if at default value (0.003)
            if (result.width - 0.003).abs() < 1e-10 {
                if let Some(width) = defaults.width {
                    result.width = width;
                }
            }

            // RGBA: apply default if at default gray
            if (result.rgba - nalgebra::Vector4::new(0.5, 0.5, 0.5, 1.0)).norm() < 1e-10 {
                if let Some(rgba) = defaults.rgba {
                    result.rgba = rgba;
                }
            }
        }

        result
    }

    /// Apply defaults to a sensor, returning a new sensor with defaults applied.
    ///
    /// Values explicitly set on the sensor take precedence over defaults.
    #[must_use]
    pub fn apply_to_sensor(&self, sensor: &MjcfSensor) -> MjcfSensor {
        let mut result = sensor.clone();

        if let Some(defaults) = self.sensor_defaults(sensor.class.as_deref()) {
            // Noise: apply default if zero
            if result.noise == 0.0 {
                if let Some(noise) = defaults.noise {
                    result.noise = noise;
                }
            }

            // Cutoff: apply default if zero
            if result.cutoff == 0.0 {
                if let Some(cutoff) = defaults.cutoff {
                    result.cutoff = cutoff;
                }
            }

            // User: apply default if empty
            if result.user.is_empty() {
                if let Some(ref user) = defaults.user {
                    result.user.clone_from(user);
                }
            }
        }

        result
    }

    /// Apply defaults to a contact pair, returning a new pair with defaults applied.
    ///
    /// Values explicitly set on the pair take precedence over defaults.
    #[must_use]
    pub fn apply_to_pair(&self, pair: &MjcfContactPair) -> MjcfContactPair {
        let mut result = pair.clone();
        if let Some(defaults) = self.pair_defaults(pair.class.as_deref()) {
            if result.condim.is_none() {
                result.condim = defaults.condim;
            }
            if result.friction.is_none() {
                result.friction = defaults.friction;
            }
            if result.solref.is_none() {
                result.solref = defaults.solref;
            }
            if result.solreffriction.is_none() {
                result.solreffriction = defaults.solreffriction;
            }
            if result.solimp.is_none() {
                result.solimp = defaults.solimp;
            }
            if result.margin.is_none() {
                result.margin = defaults.margin;
            }
            if result.gap.is_none() {
                result.gap = defaults.gap;
            }
        }
        result
    }

    /// Resolve all defaults by building inheritance chains.
    fn resolve_all(defaults: &[MjcfDefault]) -> HashMap<String, MjcfDefault> {
        // First, build a map of raw defaults
        let raw_map: HashMap<&str, &MjcfDefault> =
            defaults.iter().map(|d| (d.class.as_str(), d)).collect();

        // Resolve each default class
        let mut resolved = HashMap::new();

        for default in defaults {
            let resolved_default = Self::resolve_single(&default.class, &raw_map);
            resolved.insert(default.class.clone(), resolved_default);
        }

        // Always include root defaults (empty class)
        if !resolved.contains_key("") {
            resolved.insert(String::new(), MjcfDefault::default());
        }

        resolved
    }

    /// Resolve a single default class by merging with parent chain.
    fn resolve_single(class: &str, raw_map: &HashMap<&str, &MjcfDefault>) -> MjcfDefault {
        // Build inheritance chain from root to this class
        let mut chain = Vec::new();
        let mut current = class;

        while let Some(default) = raw_map.get(current) {
            chain.push(*default);
            match &default.parent_class {
                Some(parent) => current = parent,
                None => break,
            }
        }

        // Reverse to go from root to specific
        chain.reverse();

        // Merge chain
        let mut result = MjcfDefault {
            class: class.to_string(),
            parent_class: None,
            ..Default::default()
        };

        for default in chain {
            result = Self::merge_defaults(&result, default);
        }

        result
    }

    /// Merge two defaults, with `child` values overriding `parent` values.
    fn merge_defaults(parent: &MjcfDefault, child: &MjcfDefault) -> MjcfDefault {
        MjcfDefault {
            class: child.class.clone(),
            parent_class: child.parent_class.clone(),
            joint: Self::merge_joint_defaults(parent.joint.as_ref(), child.joint.as_ref()),
            geom: Self::merge_geom_defaults(parent.geom.as_ref(), child.geom.as_ref()),
            actuator: Self::merge_actuator_defaults(
                parent.actuator.as_ref(),
                child.actuator.as_ref(),
            ),
            tendon: Self::merge_tendon_defaults(parent.tendon.as_ref(), child.tendon.as_ref()),
            sensor: Self::merge_sensor_defaults(parent.sensor.as_ref(), child.sensor.as_ref()),
            mesh: Self::merge_mesh_defaults(parent.mesh.as_ref(), child.mesh.as_ref()),
            site: Self::merge_site_defaults(parent.site.as_ref(), child.site.as_ref()),
            pair: Self::merge_pair_defaults(parent.pair.as_ref(), child.pair.as_ref()),
        }
    }

    fn merge_joint_defaults(
        parent: Option<&MjcfJointDefaults>,
        child: Option<&MjcfJointDefaults>,
    ) -> Option<MjcfJointDefaults> {
        match (parent, child) {
            (None, None) => None,
            (Some(p), None) => Some(p.clone()),
            (None, Some(c)) => Some(c.clone()),
            (Some(p), Some(c)) => Some(MjcfJointDefaults {
                joint_type: c.joint_type.or(p.joint_type),
                limited: c.limited.or(p.limited),
                axis: c.axis.or(p.axis),
                damping: c.damping.or(p.damping),
                stiffness: c.stiffness.or(p.stiffness),
                armature: c.armature.or(p.armature),
            }),
        }
    }

    fn merge_geom_defaults(
        parent: Option<&MjcfGeomDefaults>,
        child: Option<&MjcfGeomDefaults>,
    ) -> Option<MjcfGeomDefaults> {
        match (parent, child) {
            (None, None) => None,
            (Some(p), None) => Some(p.clone()),
            (None, Some(c)) => Some(c.clone()),
            (Some(p), Some(c)) => Some(MjcfGeomDefaults {
                geom_type: c.geom_type.or(p.geom_type),
                friction: c.friction.or(p.friction),
                density: c.density.or(p.density),
                mass: c.mass.or(p.mass),
                rgba: c.rgba.or(p.rgba),
                contype: c.contype.or(p.contype),
                conaffinity: c.conaffinity.or(p.conaffinity),
            }),
        }
    }

    fn merge_actuator_defaults(
        parent: Option<&MjcfActuatorDefaults>,
        child: Option<&MjcfActuatorDefaults>,
    ) -> Option<MjcfActuatorDefaults> {
        match (parent, child) {
            (None, None) => None,
            (Some(p), None) => Some(p.clone()),
            (None, Some(c)) => Some(c.clone()),
            (Some(p), Some(c)) => Some(MjcfActuatorDefaults {
                ctrlrange: c.ctrlrange.or(p.ctrlrange),
                forcerange: c.forcerange.or(p.forcerange),
                gear: c.gear.or(p.gear),
                kp: c.kp.or(p.kp),
                kv: c.kv.or(p.kv),
                ctrllimited: c.ctrllimited.or(p.ctrllimited),
                forcelimited: c.forcelimited.or(p.forcelimited),
                gaintype: c.gaintype.clone().or_else(|| p.gaintype.clone()),
                biastype: c.biastype.clone().or_else(|| p.biastype.clone()),
                dyntype: c.dyntype.clone().or_else(|| p.dyntype.clone()),
                gainprm: c.gainprm.clone().or_else(|| p.gainprm.clone()),
                biasprm: c.biasprm.clone().or_else(|| p.biasprm.clone()),
                dynprm: c.dynprm.clone().or_else(|| p.dynprm.clone()),
            }),
        }
    }

    fn merge_tendon_defaults(
        parent: Option<&MjcfTendonDefaults>,
        child: Option<&MjcfTendonDefaults>,
    ) -> Option<MjcfTendonDefaults> {
        match (parent, child) {
            (None, None) => None,
            (Some(p), None) => Some(p.clone()),
            (None, Some(c)) => Some(c.clone()),
            (Some(p), Some(c)) => Some(MjcfTendonDefaults {
                range: c.range.or(p.range),
                limited: c.limited.or(p.limited),
                stiffness: c.stiffness.or(p.stiffness),
                damping: c.damping.or(p.damping),
                frictionloss: c.frictionloss.or(p.frictionloss),
                width: c.width.or(p.width),
                rgba: c.rgba.or(p.rgba),
            }),
        }
    }

    fn merge_sensor_defaults(
        parent: Option<&MjcfSensorDefaults>,
        child: Option<&MjcfSensorDefaults>,
    ) -> Option<MjcfSensorDefaults> {
        match (parent, child) {
            (None, None) => None,
            (Some(p), None) => Some(p.clone()),
            (None, Some(c)) => Some(c.clone()),
            (Some(p), Some(c)) => Some(MjcfSensorDefaults {
                noise: c.noise.or(p.noise),
                cutoff: c.cutoff.or(p.cutoff),
                user: c.user.clone().or_else(|| p.user.clone()),
            }),
        }
    }

    fn merge_mesh_defaults(
        parent: Option<&MjcfMeshDefaults>,
        child: Option<&MjcfMeshDefaults>,
    ) -> Option<MjcfMeshDefaults> {
        match (parent, child) {
            (None, None) => None,
            (Some(p), None) => Some(p.clone()),
            (None, Some(c)) => Some(c.clone()),
            (Some(p), Some(c)) => Some(MjcfMeshDefaults {
                scale: c.scale.or(p.scale),
            }),
        }
    }

    fn merge_site_defaults(
        parent: Option<&MjcfSiteDefaults>,
        child: Option<&MjcfSiteDefaults>,
    ) -> Option<MjcfSiteDefaults> {
        match (parent, child) {
            (None, None) => None,
            (Some(p), None) => Some(p.clone()),
            (None, Some(c)) => Some(c.clone()),
            (Some(p), Some(c)) => Some(MjcfSiteDefaults {
                site_type: c.site_type.clone().or_else(|| p.site_type.clone()),
                size: c.size.clone().or_else(|| p.size.clone()),
                rgba: c.rgba.or(p.rgba),
            }),
        }
    }

    fn merge_pair_defaults(
        parent: Option<&MjcfPairDefaults>,
        child: Option<&MjcfPairDefaults>,
    ) -> Option<MjcfPairDefaults> {
        match (parent, child) {
            (None, None) => None,
            (Some(p), None) => Some(p.clone()),
            (None, Some(c)) => Some(c.clone()),
            (Some(p), Some(c)) => Some(MjcfPairDefaults {
                condim: c.condim.or(p.condim),
                friction: c.friction.or(p.friction),
                solref: c.solref.or(p.solref),
                solreffriction: c.solreffriction.or(p.solreffriction),
                solimp: c.solimp.or(p.solimp),
                margin: c.margin.or(p.margin),
                gap: c.gap.or(p.gap),
            }),
        }
    }
}

impl Default for DefaultResolver {
    fn default() -> Self {
        Self::new(&[])
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::types::MjcfJointType;
    use approx::assert_relative_eq;
    use nalgebra::Vector4;

    #[test]
    fn test_empty_resolver() {
        let resolver = DefaultResolver::new(&[]);
        assert!(resolver.joint_defaults(None).is_none());
        assert!(resolver.geom_defaults(None).is_none());
    }

    #[test]
    fn test_root_defaults() {
        let defaults = vec![MjcfDefault {
            class: String::new(),
            parent_class: None,
            joint: Some(MjcfJointDefaults {
                damping: Some(0.5),
                ..Default::default()
            }),
            geom: Some(MjcfGeomDefaults {
                density: Some(2000.0),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults);

        let joint_defaults = resolver
            .joint_defaults(None)
            .expect("should have joint defaults");
        assert_relative_eq!(joint_defaults.damping.unwrap(), 0.5, epsilon = 1e-10);

        let geom_defaults = resolver
            .geom_defaults(None)
            .expect("should have geom defaults");
        assert_relative_eq!(geom_defaults.density.unwrap(), 2000.0, epsilon = 1e-10);
    }

    #[test]
    fn test_named_class() {
        let defaults = vec![
            MjcfDefault {
                class: String::new(),
                parent_class: None,
                joint: Some(MjcfJointDefaults {
                    damping: Some(0.5),
                    ..Default::default()
                }),
                ..Default::default()
            },
            MjcfDefault {
                class: "arm".to_string(),
                parent_class: Some(String::new()),
                joint: Some(MjcfJointDefaults {
                    damping: Some(1.0),
                    armature: Some(0.1),
                    ..Default::default()
                }),
                ..Default::default()
            },
        ];

        let resolver = DefaultResolver::new(&defaults);

        // Root class should have damping 0.5
        let root_defaults = resolver.joint_defaults(None).expect("root defaults");
        assert_relative_eq!(root_defaults.damping.unwrap(), 0.5, epsilon = 1e-10);

        // Arm class should have damping 1.0 (overridden) and armature 0.1
        let arm_defaults = resolver.joint_defaults(Some("arm")).expect("arm defaults");
        assert_relative_eq!(arm_defaults.damping.unwrap(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(arm_defaults.armature.unwrap(), 0.1, epsilon = 1e-10);
    }

    #[test]
    fn test_inheritance_chain() {
        let defaults = vec![
            MjcfDefault {
                class: String::new(),
                parent_class: None,
                joint: Some(MjcfJointDefaults {
                    damping: Some(0.1),
                    stiffness: Some(10.0),
                    ..Default::default()
                }),
                ..Default::default()
            },
            MjcfDefault {
                class: "robot".to_string(),
                parent_class: Some(String::new()),
                joint: Some(MjcfJointDefaults {
                    damping: Some(0.5),
                    ..Default::default()
                }),
                ..Default::default()
            },
            MjcfDefault {
                class: "arm".to_string(),
                parent_class: Some("robot".to_string()),
                joint: Some(MjcfJointDefaults {
                    armature: Some(0.01),
                    ..Default::default()
                }),
                ..Default::default()
            },
        ];

        let resolver = DefaultResolver::new(&defaults);

        let arm_defaults = resolver.joint_defaults(Some("arm")).expect("arm defaults");
        // damping should be 0.5 (from "robot", not root's 0.1)
        assert_relative_eq!(arm_defaults.damping.unwrap(), 0.5, epsilon = 1e-10);
        // stiffness should be 10.0 (inherited from root through robot)
        assert_relative_eq!(arm_defaults.stiffness.unwrap(), 10.0, epsilon = 1e-10);
        // armature should be 0.01 (from "arm")
        assert_relative_eq!(arm_defaults.armature.unwrap(), 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_apply_to_joint() {
        let defaults = vec![MjcfDefault {
            class: "test".to_string(),
            parent_class: None,
            joint: Some(MjcfJointDefaults {
                damping: Some(2.0),
                stiffness: Some(100.0),
                joint_type: Some(MjcfJointType::Slide),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults);

        // Joint with no explicit values
        let joint = MjcfJoint {
            class: Some("test".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_joint(&joint);
        assert_relative_eq!(resolved.damping, 2.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.stiffness, 100.0, epsilon = 1e-10);
        assert_eq!(resolved.joint_type, MjcfJointType::Slide);

        // Joint with explicit damping should keep it
        let joint_with_damping = MjcfJoint {
            class: Some("test".to_string()),
            damping: 0.5, // Explicitly set
            ..Default::default()
        };

        let resolved = resolver.apply_to_joint(&joint_with_damping);
        assert_relative_eq!(resolved.damping, 0.5, epsilon = 1e-10); // Kept explicit value
        assert_relative_eq!(resolved.stiffness, 100.0, epsilon = 1e-10); // Got default
    }

    #[test]
    fn test_apply_to_geom() {
        let defaults = vec![MjcfDefault {
            class: "visual".to_string(),
            parent_class: None,
            geom: Some(MjcfGeomDefaults {
                rgba: Some(Vector4::new(1.0, 0.0, 0.0, 1.0)),
                density: Some(500.0),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults);

        let geom = MjcfGeom {
            class: Some("visual".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_geom(&geom);
        assert_relative_eq!(resolved.rgba.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.rgba.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.density, 500.0, epsilon = 1e-10);
    }

    #[test]
    fn test_apply_to_actuator() {
        let defaults = vec![MjcfDefault {
            class: "motor".to_string(),
            parent_class: None,
            actuator: Some(MjcfActuatorDefaults {
                gear: Some([100.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ctrlrange: Some((-1.0, 1.0)),
                kp: Some(50.0),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults);

        let actuator = MjcfActuator {
            class: Some("motor".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_actuator(&actuator);
        assert_relative_eq!(resolved.gear[0], 100.0, epsilon = 1e-10);
        assert_eq!(resolved.ctrlrange, Some((-1.0, 1.0)));
        assert_relative_eq!(resolved.kp, 50.0, epsilon = 1e-10);
    }

    #[test]
    fn test_tendon_defaults() {
        let defaults = vec![MjcfDefault {
            class: "cable".to_string(),
            parent_class: None,
            tendon: Some(MjcfTendonDefaults {
                stiffness: Some(1000.0),
                damping: Some(10.0),
                width: Some(0.01),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults);

        let tendon_defaults = resolver
            .tendon_defaults(Some("cable"))
            .expect("tendon defaults");
        assert_relative_eq!(tendon_defaults.stiffness.unwrap(), 1000.0, epsilon = 1e-10);
        assert_relative_eq!(tendon_defaults.damping.unwrap(), 10.0, epsilon = 1e-10);
        assert_relative_eq!(tendon_defaults.width.unwrap(), 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_sensor_defaults() {
        let defaults = vec![MjcfDefault {
            class: "noisy".to_string(),
            parent_class: None,
            sensor: Some(MjcfSensorDefaults {
                noise: Some(0.01),
                cutoff: Some(100.0),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults);

        let sensor_defaults = resolver
            .sensor_defaults(Some("noisy"))
            .expect("sensor defaults");
        assert_relative_eq!(sensor_defaults.noise.unwrap(), 0.01, epsilon = 1e-10);
        assert_relative_eq!(sensor_defaults.cutoff.unwrap(), 100.0, epsilon = 1e-10);
    }

    #[test]
    fn test_nonexistent_class() {
        let defaults = vec![MjcfDefault {
            class: "existing".to_string(),
            parent_class: None,
            joint: Some(MjcfJointDefaults {
                damping: Some(1.0),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults);

        // Nonexistent class should return None
        assert!(resolver.joint_defaults(Some("nonexistent")).is_none());
    }

    #[test]
    fn test_apply_to_tendon() {
        use crate::types::MjcfTendon;

        let defaults = vec![MjcfDefault {
            class: "cable".to_string(),
            parent_class: None,
            tendon: Some(MjcfTendonDefaults {
                stiffness: Some(1000.0),
                damping: Some(10.0),
                width: Some(0.01),
                rgba: Some(Vector4::new(1.0, 0.0, 0.0, 1.0)),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults);

        // Tendon with no explicit values
        let tendon = MjcfTendon {
            class: Some("cable".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_tendon(&tendon);
        assert_relative_eq!(resolved.stiffness, 1000.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.damping, 10.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.width, 0.01, epsilon = 1e-10);
        assert_relative_eq!(resolved.rgba.x, 1.0, epsilon = 1e-10);

        // Tendon with explicit stiffness should keep it
        let tendon_with_stiffness = MjcfTendon {
            class: Some("cable".to_string()),
            stiffness: 500.0,
            ..Default::default()
        };

        let resolved = resolver.apply_to_tendon(&tendon_with_stiffness);
        assert_relative_eq!(resolved.stiffness, 500.0, epsilon = 1e-10); // Kept explicit
        assert_relative_eq!(resolved.damping, 10.0, epsilon = 1e-10); // Got default
    }

    #[test]
    fn test_apply_to_sensor() {
        use crate::types::MjcfSensor;

        let defaults = vec![MjcfDefault {
            class: "noisy".to_string(),
            parent_class: None,
            sensor: Some(MjcfSensorDefaults {
                noise: Some(0.01),
                cutoff: Some(100.0),
                user: Some(vec![1.0, 2.0, 3.0]),
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults);

        // Sensor with no explicit values
        let sensor = MjcfSensor {
            class: Some("noisy".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_sensor(&sensor);
        assert_relative_eq!(resolved.noise, 0.01, epsilon = 1e-10);
        assert_relative_eq!(resolved.cutoff, 100.0, epsilon = 1e-10);
        assert_eq!(resolved.user, vec![1.0, 2.0, 3.0]);

        // Sensor with explicit noise should keep it
        let sensor_with_noise = MjcfSensor {
            class: Some("noisy".to_string()),
            noise: 0.05,
            ..Default::default()
        };

        let resolved = resolver.apply_to_sensor(&sensor_with_noise);
        assert_relative_eq!(resolved.noise, 0.05, epsilon = 1e-10); // Kept explicit
        assert_relative_eq!(resolved.cutoff, 100.0, epsilon = 1e-10); // Got default
    }

    #[test]
    fn test_tendon_defaults_inheritance() {
        use crate::types::MjcfTendon;

        let defaults = vec![
            MjcfDefault {
                class: String::new(), // Root default
                parent_class: None,
                tendon: Some(MjcfTendonDefaults {
                    stiffness: Some(100.0),
                    damping: Some(1.0),
                    ..Default::default()
                }),
                ..Default::default()
            },
            MjcfDefault {
                class: "strong".to_string(),
                parent_class: Some(String::new()),
                tendon: Some(MjcfTendonDefaults {
                    stiffness: Some(1000.0), // Override stiffness
                    ..Default::default()     // Inherit damping
                }),
                ..Default::default()
            },
        ];

        let resolver = DefaultResolver::new(&defaults);

        let tendon = MjcfTendon {
            class: Some("strong".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_tendon(&tendon);
        assert_relative_eq!(resolved.stiffness, 1000.0, epsilon = 1e-10); // From "strong"
        assert_relative_eq!(resolved.damping, 1.0, epsilon = 1e-10); // Inherited from root
    }

    #[test]
    fn test_sensor_defaults_inheritance() {
        use crate::types::MjcfSensor;

        let defaults = vec![
            MjcfDefault {
                class: String::new(), // Root default
                parent_class: None,
                sensor: Some(MjcfSensorDefaults {
                    noise: Some(0.001),
                    cutoff: Some(50.0),
                    ..Default::default()
                }),
                ..Default::default()
            },
            MjcfDefault {
                class: "filtered".to_string(),
                parent_class: Some(String::new()),
                sensor: Some(MjcfSensorDefaults {
                    cutoff: Some(10.0),   // Override cutoff with lower value
                    ..Default::default()  // Inherit noise
                }),
                ..Default::default()
            },
        ];

        let resolver = DefaultResolver::new(&defaults);

        let sensor = MjcfSensor {
            class: Some("filtered".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_sensor(&sensor);
        assert_relative_eq!(resolved.noise, 0.001, epsilon = 1e-10); // Inherited from root
        assert_relative_eq!(resolved.cutoff, 10.0, epsilon = 1e-10); // From "filtered"
    }
}
