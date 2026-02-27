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
    MjcfJoint, MjcfJointDefaults, MjcfMesh, MjcfMeshDefaults, MjcfModel, MjcfPairDefaults,
    MjcfSensor, MjcfSensorDefaults, MjcfSite, MjcfSiteDefaults, MjcfTendon, MjcfTendonDefaults,
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
            // All Option<T> fields: element value wins if set, otherwise inherit default.
            if result.joint_type.is_none() {
                result.joint_type = defaults.joint_type;
            }
            if result.pos.is_none() {
                result.pos = defaults.pos;
            }
            if result.axis.is_none() {
                result.axis = defaults.axis;
            }
            if result.limited.is_none() {
                if let Some(limited) = defaults.limited {
                    result.limited = Some(limited);
                }
            }
            if result.ref_pos.is_none() {
                result.ref_pos = defaults.ref_pos;
            }
            if result.spring_ref.is_none() {
                result.spring_ref = defaults.spring_ref;
            }
            if result.damping.is_none() {
                result.damping = defaults.damping;
            }
            if result.stiffness.is_none() {
                result.stiffness = defaults.stiffness;
            }
            if result.armature.is_none() {
                result.armature = defaults.armature;
            }
            if result.frictionloss.is_none() {
                result.frictionloss = defaults.frictionloss;
            }
            if result.group.is_none() {
                result.group = defaults.group;
            }
            if result.range.is_none() {
                result.range = defaults.range;
            }
            if result.solref_limit.is_none() {
                result.solref_limit = defaults.solref_limit;
            }
            if result.solimp_limit.is_none() {
                result.solimp_limit = defaults.solimp_limit;
            }
            if result.solreffriction.is_none() {
                result.solreffriction = defaults.solreffriction;
            }
            if result.solimpfriction.is_none() {
                result.solimpfriction = defaults.solimpfriction;
            }
            if result.actuatorgravcomp.is_none() {
                result.actuatorgravcomp = defaults.actuatorgravcomp;
            }
        }

        result
    }

    /// Apply defaults to a geom, returning a new geom with defaults applied.
    #[must_use]
    pub fn apply_to_geom(&self, geom: &MjcfGeom) -> MjcfGeom {
        let mut result = geom.clone();

        if let Some(defaults) = self.geom_defaults(geom.class.as_deref()) {
            // All Option<T> fields: element value wins if set, otherwise inherit default.
            if result.geom_type.is_none() {
                result.geom_type = defaults.geom_type;
            }
            if result.friction.is_none() {
                result.friction = defaults.friction;
            }
            if result.density.is_none() {
                result.density = defaults.density;
            }
            if result.mass.is_none() {
                result.mass = defaults.mass;
            }
            if result.rgba.is_none() {
                result.rgba = defaults.rgba;
            }
            if result.contype.is_none() {
                result.contype = defaults.contype;
            }
            if result.conaffinity.is_none() {
                result.conaffinity = defaults.conaffinity;
            }
            if result.condim.is_none() {
                result.condim = defaults.condim;
            }
            if result.priority.is_none() {
                result.priority = defaults.priority;
            }
            if result.solmix.is_none() {
                result.solmix = defaults.solmix;
            }
            if result.margin.is_none() {
                result.margin = defaults.margin;
            }
            if result.gap.is_none() {
                result.gap = defaults.gap;
            }
            if result.solref.is_none() {
                result.solref = defaults.solref;
            }
            if result.solimp.is_none() {
                result.solimp = defaults.solimp;
            }
            if result.group.is_none() {
                result.group = defaults.group;
            }

            if result.pos.is_none() {
                result.pos = defaults.pos;
            }
            if result.quat.is_none() {
                result.quat = defaults.quat;
            }
            if result.euler.is_none() {
                result.euler = defaults.euler;
            }
            if result.axisangle.is_none() {
                result.axisangle = defaults.axisangle;
            }
            if result.xyaxes.is_none() {
                result.xyaxes = defaults.xyaxes;
            }
            if result.zaxis.is_none() {
                result.zaxis = defaults.zaxis;
            }

            // Exotic geom defaults
            if result.fromto.is_none() {
                result.fromto = defaults.fromto;
            }
            if result.mesh.is_none() {
                result.mesh.clone_from(&defaults.mesh);
            }
            if result.hfield.is_none() {
                result.hfield.clone_from(&defaults.hfield);
            }

            // Rendering
            if result.material.is_none() {
                result.material.clone_from(&defaults.material);
            }

            // Fluid forces
            if result.fluidshape.is_none() {
                result.fluidshape = defaults.fluidshape;
            }
            if result.fluidcoef.is_none() {
                result.fluidcoef = defaults.fluidcoef;
            }
        }

        result
    }

    /// Apply defaults to an actuator, returning a new actuator with defaults applied.
    #[must_use]
    pub fn apply_to_actuator(&self, actuator: &MjcfActuator) -> MjcfActuator {
        let mut result = actuator.clone();

        if let Some(defaults) = self.actuator_defaults(actuator.class.as_deref()) {
            // #todo: gear, kp, and sensor noise/cutoff use sentinel-value detection
            // (== default) instead of Option<T>. Explicitly setting gear=[1,0,0,0,0,0],
            // kp=1.0, noise=0.0, or cutoff=0.0 is indistinguishable from "unset" and
            // will be overwritten by defaults. Migrate these to Option<T> to fix.

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

            // dampratio: apply default if not explicitly set
            if result.dampratio.is_none() {
                if let Some(dr) = defaults.dampratio {
                    result.dampratio = Some(dr);
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

            // Activation parameters
            if result.group.is_none() {
                result.group = defaults.group;
            }
            if result.actlimited.is_none() {
                result.actlimited = defaults.actlimited;
            }
            if result.actrange.is_none() {
                result.actrange = defaults.actrange;
            }
            if result.actearly.is_none() {
                result.actearly = defaults.actearly;
            }
            if result.lengthrange.is_none() {
                result.lengthrange = defaults.lengthrange;
            }
        }

        result
    }

    /// Apply defaults to a site, returning a new site with defaults applied.
    #[must_use]
    pub fn apply_to_site(&self, site: &MjcfSite) -> MjcfSite {
        let mut result = site.clone();

        if let Some(defaults) = self.site_defaults(site.class.as_deref()) {
            // All Option<T> fields: element value wins if set, otherwise inherit default.
            if result.site_type.is_none() {
                result.site_type.clone_from(&defaults.site_type);
            }
            if result.size.is_none() {
                result.size.clone_from(&defaults.size);
            }
            if result.rgba.is_none() {
                result.rgba = defaults.rgba;
            }
            if result.group.is_none() {
                result.group = defaults.group;
            }
            if result.pos.is_none() {
                result.pos = defaults.pos;
            }
            if result.quat.is_none() {
                result.quat = defaults.quat;
            }
            if result.euler.is_none() {
                result.euler = defaults.euler;
            }
            if result.axisangle.is_none() {
                result.axisangle = defaults.axisangle;
            }
            if result.xyaxes.is_none() {
                result.xyaxes = defaults.xyaxes;
            }
            if result.zaxis.is_none() {
                result.zaxis = defaults.zaxis;
            }

            // Rendering
            if result.material.is_none() {
                result.material.clone_from(&defaults.material);
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

            // All Option<T> fields: element value wins if set, otherwise inherit default.
            if result.stiffness.is_none() {
                result.stiffness = defaults.stiffness;
            }
            if result.damping.is_none() {
                result.damping = defaults.damping;
            }
            if result.frictionloss.is_none() {
                result.frictionloss = defaults.frictionloss;
            }
            if result.springlength.is_none() {
                result.springlength = defaults.springlength;
            }
            if result.width.is_none() {
                result.width = defaults.width;
            }
            if result.rgba.is_none() {
                result.rgba = defaults.rgba;
            }
            if result.group.is_none() {
                result.group = defaults.group;
            }

            // Solver parameters
            if result.solref.is_none() {
                result.solref = defaults.solref;
            }
            if result.solimp.is_none() {
                result.solimp = defaults.solimp;
            }
            if result.solreffriction.is_none() {
                result.solreffriction = defaults.solreffriction;
            }
            if result.solimpfriction.is_none() {
                result.solimpfriction = defaults.solimpfriction;
            }
            if result.margin.is_none() {
                result.margin = defaults.margin;
            }

            // Rendering
            if result.material.is_none() {
                result.material.clone_from(&defaults.material);
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

    /// Apply defaults to a mesh, returning a new mesh with defaults applied.
    ///
    /// Meshes use the root default class (meshes have no `class` attribute in MuJoCo).
    #[must_use]
    pub fn apply_to_mesh(&self, mesh: &MjcfMesh) -> MjcfMesh {
        let mut result = mesh.clone();

        if let Some(defaults) = self.mesh_defaults(None) {
            if result.scale.is_none() {
                result.scale = defaults.scale;
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
                pos: c.pos.or(p.pos),
                limited: c.limited.or(p.limited),
                axis: c.axis.or(p.axis),
                ref_pos: c.ref_pos.or(p.ref_pos),
                spring_ref: c.spring_ref.or(p.spring_ref),
                damping: c.damping.or(p.damping),
                stiffness: c.stiffness.or(p.stiffness),
                armature: c.armature.or(p.armature),
                frictionloss: c.frictionloss.or(p.frictionloss),
                group: c.group.or(p.group),
                range: c.range.or(p.range),
                solref_limit: c.solref_limit.or(p.solref_limit),
                solimp_limit: c.solimp_limit.or(p.solimp_limit),
                solreffriction: c.solreffriction.or(p.solreffriction),
                solimpfriction: c.solimpfriction.or(p.solimpfriction),
                actuatorgravcomp: c.actuatorgravcomp.or(p.actuatorgravcomp),
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
                condim: c.condim.or(p.condim),
                priority: c.priority.or(p.priority),
                solmix: c.solmix.or(p.solmix),
                margin: c.margin.or(p.margin),
                gap: c.gap.or(p.gap),
                solref: c.solref.or(p.solref),
                solimp: c.solimp.or(p.solimp),
                group: c.group.or(p.group),
                pos: c.pos.or(p.pos),
                quat: c.quat.or(p.quat),
                euler: c.euler.or(p.euler),
                axisangle: c.axisangle.or(p.axisangle),
                xyaxes: c.xyaxes.or(p.xyaxes),
                zaxis: c.zaxis.or(p.zaxis),
                fromto: c.fromto.or(p.fromto),
                mesh: c.mesh.clone().or_else(|| p.mesh.clone()),
                hfield: c.hfield.clone().or_else(|| p.hfield.clone()),
                material: c.material.clone().or_else(|| p.material.clone()),
                fluidshape: c.fluidshape.or(p.fluidshape),
                fluidcoef: c.fluidcoef.or(p.fluidcoef),
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
                dampratio: c.dampratio.or(p.dampratio),
                ctrllimited: c.ctrllimited.or(p.ctrllimited),
                forcelimited: c.forcelimited.or(p.forcelimited),
                gaintype: c.gaintype.clone().or_else(|| p.gaintype.clone()),
                biastype: c.biastype.clone().or_else(|| p.biastype.clone()),
                dyntype: c.dyntype.clone().or_else(|| p.dyntype.clone()),
                gainprm: c.gainprm.clone().or_else(|| p.gainprm.clone()),
                biasprm: c.biasprm.clone().or_else(|| p.biasprm.clone()),
                dynprm: c.dynprm.clone().or_else(|| p.dynprm.clone()),
                group: c.group.or(p.group),
                actlimited: c.actlimited.or(p.actlimited),
                actrange: c.actrange.or(p.actrange),
                actearly: c.actearly.or(p.actearly),
                lengthrange: c.lengthrange.or(p.lengthrange),
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
                springlength: c.springlength.or(p.springlength),
                width: c.width.or(p.width),
                rgba: c.rgba.or(p.rgba),
                group: c.group.or(p.group),
                solref: c.solref.or(p.solref),
                solimp: c.solimp.or(p.solimp),
                solreffriction: c.solreffriction.or(p.solreffriction),
                solimpfriction: c.solimpfriction.or(p.solimpfriction),
                margin: c.margin.or(p.margin),
                material: c.material.clone().or_else(|| p.material.clone()),
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
                group: c.group.or(p.group),
                pos: c.pos.or(p.pos),
                quat: c.quat.or(p.quat),
                euler: c.euler.or(p.euler),
                axisangle: c.axisangle.or(p.axisangle),
                xyaxes: c.xyaxes.or(p.xyaxes),
                zaxis: c.zaxis.or(p.zaxis),
                material: c.material.clone().or_else(|| p.material.clone()),
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
        assert_relative_eq!(resolved.damping.unwrap(), 2.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.stiffness.unwrap(), 100.0, epsilon = 1e-10);
        assert_eq!(resolved.joint_type, Some(MjcfJointType::Slide));

        // Joint with explicit damping should keep it
        let joint_with_damping = MjcfJoint {
            class: Some("test".to_string()),
            damping: Some(0.5), // Explicitly set
            ..Default::default()
        };

        let resolved = resolver.apply_to_joint(&joint_with_damping);
        assert_relative_eq!(resolved.damping.unwrap(), 0.5, epsilon = 1e-10); // Kept explicit value
        assert_relative_eq!(resolved.stiffness.unwrap(), 100.0, epsilon = 1e-10); // Got default
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
        assert_relative_eq!(resolved.rgba.unwrap().x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.rgba.unwrap().y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.density.unwrap(), 500.0, epsilon = 1e-10);
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
        assert_relative_eq!(resolved.stiffness.unwrap(), 1000.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.damping.unwrap(), 10.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.width.unwrap(), 0.01, epsilon = 1e-10);
        assert_relative_eq!(resolved.rgba.unwrap().x, 1.0, epsilon = 1e-10);

        // Tendon with explicit stiffness should keep it
        let tendon_with_stiffness = MjcfTendon {
            class: Some("cable".to_string()),
            stiffness: Some(500.0),
            ..Default::default()
        };

        let resolved = resolver.apply_to_tendon(&tendon_with_stiffness);
        assert_relative_eq!(resolved.stiffness.unwrap(), 500.0, epsilon = 1e-10); // Kept explicit
        assert_relative_eq!(resolved.damping.unwrap(), 10.0, epsilon = 1e-10); // Got default
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
        assert_relative_eq!(resolved.stiffness.unwrap(), 1000.0, epsilon = 1e-10); // From "strong"
        assert_relative_eq!(resolved.damping.unwrap(), 1.0, epsilon = 1e-10); // Inherited from root
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
