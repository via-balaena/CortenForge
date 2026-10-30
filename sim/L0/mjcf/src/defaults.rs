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
//!
//! As MuJoCo 3.5.0's reader makes classes (`mjXReader::Default`): every
//! top-level `<default>` is the one main class, which `class` and
//! `childclass` may name `main`; and a nested class copies its parent as the
//! parent stands when the class is made, so a later top-level block does not
//! reach a class made before it.

use std::collections::HashMap;

use nalgebra::{Vector3, Vector4};

use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{
    MjcfActuator, MjcfActuatorDefaults, MjcfContactPair, MjcfDefault, MjcfEqualityDefaults,
    MjcfGeom, MjcfGeomDefaults, MjcfJoint, MjcfJointDefaults, MjcfMesh, MjcfMeshDefaults,
    MjcfModel, MjcfPairDefaults, MjcfSensor, MjcfSensorDefaults, MjcfSite, MjcfSiteDefaults,
    MjcfTendon, MjcfTendonDefaults, Prefix,
};

/// The name `class` and `childclass` may give the main class, which a
/// top-level `<default>` may also be written with (`mjs_findDefault`; the
/// main class is stored under `""`).
const MAIN: &str = "main";

/// Resolves and applies default classes to MJCF elements.
///
/// Holds every class a model's `<default>`s make, each as it stood when it was
/// made, and applies a class to an element: the element's own values over its
/// class's.
#[derive(Debug, Clone)]
pub struct DefaultResolver {
    /// Each class by name, the main class under `""`.
    classes: HashMap<String, MjcfDefault>,
}

impl DefaultResolver {
    /// The classes `defaults` makes, in order, as MuJoCo 3.5.0's reader makes
    /// them (`mjXReader::Default`): an entry without a parent is read onto the
    /// one main class; a nested entry copies its parent as the parent stands
    /// then (`mjs_addDefault`), with its own values over the copy. The parser
    /// lists each block's own elements before its nested classes, as MuJoCo
    /// reads them. No entry is visited twice, so no list makes a cycle.
    ///
    /// # Errors
    ///
    /// [`MjcfError::RenamedRootClass`] for an entry without a parent named
    /// other than `""` or `main`; [`MjcfError::EmptyClassName`] for a nested
    /// entry without a name; [`MjcfError::DuplicateName`] for a nested entry
    /// named `main` or as an earlier class; [`MjcfError::UnknownClass`] for a
    /// parent that no earlier entry made (a list built in code).
    pub fn new(defaults: &[MjcfDefault]) -> Result<Self> {
        let mut resolver = Self::default();
        for entry in defaults {
            let Some(parent) = entry.parent_class.as_deref() else {
                if !entry.class.is_empty() && entry.class != MAIN {
                    return Err(MjcfError::renamed_root_class(&entry.class));
                }
                resolver.read_onto_main(entry);
                continue;
            };
            if entry.class.is_empty() {
                let nested_in = if parent.is_empty() { MAIN } else { parent };
                return Err(MjcfError::empty_class_name()
                    .at(Location::named(ElementKind::DefaultClass, nested_in)));
            }
            if entry.class == MAIN || resolver.classes.contains_key(&entry.class) {
                return Err(MjcfError::duplicate_name(
                    ElementKind::DefaultClass,
                    &entry.class,
                ));
            }
            let parent_name = if parent == MAIN { "" } else { parent };
            let Some(parent) = resolver.classes.get(parent_name) else {
                return Err(MjcfError::unknown_class(parent_name));
            };
            let class = Self::merge_defaults(parent, entry);
            resolver.classes.insert(entry.class.clone(), class);
        }
        Ok(resolver)
    }

    /// Create a resolver from an MJCF model.
    ///
    /// # Errors
    ///
    /// As [`Self::new`].
    pub fn from_model(model: &MjcfModel) -> Result<Self> {
        Self::new(&model.defaults)
    }

    /// The class `name` names: none or `main` the main class.
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for any other name no class has, `""`
    /// included (`mjXReader::GetClass`).
    pub fn class(&self, name: Option<&str>) -> Result<&MjcfDefault> {
        let key = match name {
            None | Some(MAIN) => "",
            Some("") => return Err(MjcfError::unknown_class("")),
            Some(name) => name,
        };
        self.classes
            .get(key)
            .ok_or_else(|| MjcfError::unknown_class(key))
    }

    /// A top-level entry read onto the main class, its values over the main
    /// class's. An actuator default is the exception: a later block's
    /// replaces an earlier one, as a later actuator element does within one
    /// block. MuJoCo keeps one actuator default per class, each element
    /// writing only its own fields, so a `<motor gear="5"/>` block's gear
    /// survives a later `<position kp="10"/>` block (measured); what a class
    /// two kinds write means is book L17's, with A3-O1.
    fn read_onto_main(&mut self, entry: &MjcfDefault) {
        let main = self.classes.entry(String::new()).or_default();
        let mut merged = Self::merge_defaults(main, entry);
        if entry.actuator.is_some() {
            merged.actuator.clone_from(&entry.actuator);
        }
        *main = merged;
    }

    /// Apply equality defaults to mutable field references.
    /// Generic across all 5 equality constraint types — they share the same 3
    /// defaultable fields (active, solref, solimp).
    /// MuJoCo ref: `OneEquality()` in `xml_native_reader.cc` (defaults context).
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for a class no `<default>` makes.
    pub fn apply_equality_defaults(
        &self,
        class: Option<&str>,
        active: &mut Option<bool>,
        solref: &mut Option<Prefix<2>>,
        solimp: &mut Option<Prefix<5>>,
    ) -> Result<()> {
        if let Some(defaults) = &self.class(class)?.equality {
            if active.is_none() {
                *active = defaults.active;
            }
            *solref = over(*solref, defaults.solref);
            *solimp = over(*solimp, defaults.solimp);
        }
        Ok(())
    }

    /// Apply defaults to a joint, returning a new joint with defaults applied.
    ///
    /// Values explicitly set on the joint take precedence over defaults.
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for a class no `<default>` makes.
    pub fn apply_to_joint(&self, joint: &MjcfJoint) -> Result<MjcfJoint> {
        let mut result = joint.clone();

        if let Some(defaults) = &self.class(joint.class.as_deref())?.joint {
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
            if result.limited.is_none()
                && let Some(limited) = defaults.limited
            {
                result.limited = Some(limited);
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
            result.solref_limit = over(result.solref_limit, defaults.solref_limit);
            result.solimp_limit = over(result.solimp_limit, defaults.solimp_limit);
            result.solreffriction = over(result.solreffriction, defaults.solreffriction);
            result.solimpfriction = over(result.solimpfriction, defaults.solimpfriction);
            if result.actuatorgravcomp.is_none() {
                result.actuatorgravcomp = defaults.actuatorgravcomp;
            }
            if result.margin.is_none() {
                result.margin = defaults.margin;
            }
            if result.user.is_empty()
                && let Some(ref user) = defaults.user
            {
                result.user.clone_from(user);
            }
        }

        Ok(result)
    }

    /// Apply defaults to a geom, returning a new geom with defaults applied.
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for a class no `<default>` makes.
    pub fn apply_to_geom(&self, geom: &MjcfGeom) -> Result<MjcfGeom> {
        let mut result = geom.clone();

        if let Some(defaults) = &self.class(geom.class.as_deref())?.geom {
            // All Option<T> fields: element value wins if set, otherwise inherit default.
            if result.geom_type.is_none() {
                result.geom_type = defaults.geom_type;
            }
            result.friction = over(result.friction, defaults.friction);
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
            result.solref = over(result.solref, defaults.solref);
            result.solimp = over(result.solimp, defaults.solimp);
            if result.group.is_none() {
                result.group = defaults.group;
            }

            if result.pos.is_none() {
                result.pos = defaults.pos;
            }
            if result.quat.is_none() {
                result.quat = defaults.quat;
            }
            (result.euler, result.axisangle, result.xyaxes, result.zaxis) = alternative_over(
                (result.euler, result.axisangle, result.xyaxes, result.zaxis),
                (
                    defaults.euler,
                    defaults.axisangle,
                    defaults.xyaxes,
                    defaults.zaxis,
                ),
            );

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
            result.fluidcoef = over(result.fluidcoef, defaults.fluidcoef);
            if result.user.is_empty()
                && let Some(ref user) = defaults.user
            {
                result.user.clone_from(user);
            }
            if result.shellinertia.is_none() {
                result.shellinertia = defaults.shellinertia;
            }
        }

        Ok(result)
    }

    /// Apply defaults to an actuator, returning a new actuator with defaults applied.
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for a class no `<default>` makes.
    pub fn apply_to_actuator(&self, actuator: &MjcfActuator) -> Result<MjcfActuator> {
        let mut result = actuator.clone();

        if let Some(defaults) = &self.class(actuator.class.as_deref())?.actuator {
            result.gear = over(result.gear, defaults.gear);

            // Control range: apply default if None
            if result.ctrlrange.is_none() {
                result.ctrlrange = defaults.ctrlrange;
            }

            // Force range: apply default if None
            if result.forcerange.is_none() {
                result.forcerange = defaults.forcerange;
            }

            // Control limited: apply default only if not explicitly set on element
            if result.ctrllimited.is_none()
                && let Some(ctrllimited) = defaults.ctrllimited
            {
                result.ctrllimited = Some(ctrllimited);
            }

            // Force limited: apply default only if not explicitly set on element
            if result.forcelimited.is_none()
                && let Some(forcelimited) = defaults.forcelimited
            {
                result.forcelimited = Some(forcelimited);
            }

            result.kp = result.kp.or(defaults.kp);

            // kv: apply default if not explicitly set
            if result.kv.is_none()
                && let Some(kv) = defaults.kv
            {
                result.kv = Some(kv);
            }

            // dampratio: apply default if not explicitly set
            if result.dampratio.is_none()
                && let Some(dr) = defaults.dampratio
            {
                result.dampratio = Some(dr);
            }
            if result.inheritrange.is_none() {
                result.inheritrange = defaults.inheritrange;
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
            result.gainprm = over(result.gainprm, defaults.gainprm);
            result.biasprm = over(result.biasprm, defaults.biasprm);
            result.dynprm = over(result.dynprm, defaults.dynprm);

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
            if result.nsample.is_none() {
                result.nsample = defaults.nsample;
            }
            if result.interp.is_none() {
                result.interp.clone_from(&defaults.interp);
            }
            if result.delay.is_none() {
                result.delay = defaults.delay;
            }

            result.area = result.area.or(defaults.area);
            if result.diameter.is_none() {
                result.diameter = defaults.diameter;
            }
            if result.timeconst.is_none() {
                result.timeconst = defaults.timeconst;
            }
            result.bias = result.bias.or(defaults.bias);

            result.muscle_timeconst = result.muscle_timeconst.or(defaults.muscle_timeconst);
            result.range = result.range.or(defaults.range);
            // MuJoCo reads a negative force as unwritten (`mjs_setToMuscle`,
            // `user_api.cc:1009-1050`): the class's, if it writes one.
            if defaults.force.is_some() && result.force.is_none_or(|f| f < 0.0) {
                result.force = defaults.force;
            }
            result.scale = result.scale.or(defaults.scale);
            result.lmin = result.lmin.or(defaults.lmin);
            result.lmax = result.lmax.or(defaults.lmax);
            result.vmax = result.vmax.or(defaults.vmax);
            result.fpmax = result.fpmax.or(defaults.fpmax);
            result.fvmax = result.fvmax.or(defaults.fvmax);

            result.gain = result.gain.or(defaults.gain);
            if result.user.is_empty()
                && let Some(ref user) = defaults.user
            {
                result.user.clone_from(user);
            }
        }

        Ok(result)
    }

    /// Apply defaults to a site, returning a new site with defaults applied.
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for a class no `<default>` makes.
    pub fn apply_to_site(&self, site: &MjcfSite) -> Result<MjcfSite> {
        let mut result = site.clone();

        if let Some(defaults) = &self.class(site.class.as_deref())?.site {
            // An Option<T> field: the element's value wins if set, otherwise
            // the default's; the size is read over the default's (`over`).
            if result.site_type.is_none() {
                result.site_type.clone_from(&defaults.site_type);
            }
            result.size = over(site.size, defaults.size);
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
            (result.euler, result.axisangle, result.xyaxes, result.zaxis) = alternative_over(
                (result.euler, result.axisangle, result.xyaxes, result.zaxis),
                (
                    defaults.euler,
                    defaults.axisangle,
                    defaults.xyaxes,
                    defaults.zaxis,
                ),
            );
            if result.fromto.is_none() {
                result.fromto = defaults.fromto;
            }

            // Rendering
            if result.material.is_none() {
                result.material.clone_from(&defaults.material);
            }
            if result.user.is_empty()
                && let Some(ref user) = defaults.user
            {
                result.user.clone_from(user);
            }
        }

        Ok(result)
    }

    /// Apply defaults to a tendon, returning a new tendon with defaults applied.
    ///
    /// Values explicitly set on the tendon take precedence over defaults.
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for a class no `<default>` makes.
    pub fn apply_to_tendon(&self, tendon: &MjcfTendon) -> Result<MjcfTendon> {
        let mut result = tendon.clone();

        if let Some(defaults) = &self.class(tendon.class.as_deref())?.tendon {
            // Range: apply default if None
            if result.range.is_none() {
                result.range = defaults.range;
            }

            // Limited: apply default only if not explicitly set on element
            if result.limited.is_none()
                && let Some(limited) = defaults.limited
            {
                result.limited = Some(limited);
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
            result.solref_limit = over(result.solref_limit, defaults.solref_limit);
            result.solimp_limit = over(result.solimp_limit, defaults.solimp_limit);
            result.solreffriction = over(result.solreffriction, defaults.solreffriction);
            result.solimpfriction = over(result.solimpfriction, defaults.solimpfriction);
            if result.margin.is_none() {
                result.margin = defaults.margin;
            }

            // Rendering
            if result.material.is_none() {
                result.material.clone_from(&defaults.material);
            }
            if result.user.is_empty()
                && let Some(ref user) = defaults.user
            {
                result.user.clone_from(user);
            }
        }

        Ok(result)
    }

    /// Apply defaults to a sensor, returning a new sensor with defaults applied.
    ///
    /// Values explicitly set on the sensor take precedence over defaults.
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for a class no `<default>` makes.
    pub fn apply_to_sensor(&self, sensor: &MjcfSensor) -> Result<MjcfSensor> {
        let mut result = sensor.clone();

        if let Some(defaults) = &self.class(sensor.class.as_deref())?.sensor {
            result.noise = result.noise.or(defaults.noise);
            result.cutoff = result.cutoff.or(defaults.cutoff);

            // User: apply default if empty
            if result.user.is_empty()
                && let Some(ref user) = defaults.user
            {
                result.user.clone_from(user);
            }
        }

        Ok(result)
    }

    /// Apply defaults to a contact pair, returning a new pair with defaults applied.
    ///
    /// Values explicitly set on the pair take precedence over defaults.
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for a class no `<default>` makes.
    pub fn apply_to_pair(&self, pair: &MjcfContactPair) -> Result<MjcfContactPair> {
        let mut result = pair.clone();
        if let Some(defaults) = &self.class(pair.class.as_deref())?.pair {
            if result.condim.is_none() {
                result.condim = defaults.condim;
            }
            result.friction = over(result.friction, defaults.friction);
            result.solref = over(result.solref, defaults.solref);
            result.solreffriction = over(result.solreffriction, defaults.solreffriction);
            result.solimp = over(result.solimp, defaults.solimp);
            if result.margin.is_none() {
                result.margin = defaults.margin;
            }
            if result.gap.is_none() {
                result.gap = defaults.gap;
            }
        }
        Ok(result)
    }

    /// Apply defaults to a mesh, returning a new mesh with defaults applied:
    /// its class's, else the main class's (MuJoCo's `mjXReader::Asset`
    /// gives a mesh its `class`).
    ///
    /// # Errors
    ///
    /// [`MjcfError::UnknownClass`] for a class no `<default>` makes.
    pub fn apply_to_mesh(&self, mesh: &MjcfMesh) -> Result<MjcfMesh> {
        let mut result = mesh.clone();

        if let Some(defaults) = &self.class(mesh.class.as_deref())?.mesh {
            if result.scale.is_none() {
                result.scale = defaults.scale;
            }
            if result.maxhullvert.is_none() {
                result.maxhullvert = defaults.maxhullvert;
            }
            if result.inertia.is_none() {
                result.inertia = defaults.inertia;
            }
        }

        Ok(result)
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
            equality: Self::merge_equality_defaults(
                parent.equality.as_ref(),
                child.equality.as_ref(),
            ),
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
                solref_limit: over(c.solref_limit, p.solref_limit),
                solimp_limit: over(c.solimp_limit, p.solimp_limit),
                solreffriction: over(c.solreffriction, p.solreffriction),
                solimpfriction: over(c.solimpfriction, p.solimpfriction),
                actuatorgravcomp: c.actuatorgravcomp.or(p.actuatorgravcomp),
                margin: c.margin.or(p.margin),
                user: c.user.clone().or_else(|| p.user.clone()),
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
            (Some(p), Some(c)) => {
                let (euler, axisangle, xyaxes, zaxis) = alternative_over(
                    (c.euler, c.axisangle, c.xyaxes, c.zaxis),
                    (p.euler, p.axisangle, p.xyaxes, p.zaxis),
                );
                Some(MjcfGeomDefaults {
                    geom_type: c.geom_type.or(p.geom_type),
                    friction: over(c.friction, p.friction),
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
                    solref: over(c.solref, p.solref),
                    solimp: over(c.solimp, p.solimp),
                    group: c.group.or(p.group),
                    pos: c.pos.or(p.pos),
                    quat: c.quat.or(p.quat),
                    euler,
                    axisangle,
                    xyaxes,
                    zaxis,
                    fromto: c.fromto.or(p.fromto),
                    mesh: c.mesh.clone().or_else(|| p.mesh.clone()),
                    hfield: c.hfield.clone().or_else(|| p.hfield.clone()),
                    material: c.material.clone().or_else(|| p.material.clone()),
                    fluidshape: c.fluidshape.or(p.fluidshape),
                    fluidcoef: over(c.fluidcoef, p.fluidcoef),
                    user: c.user.clone().or_else(|| p.user.clone()),
                    shellinertia: c.shellinertia.or(p.shellinertia),
                })
            }
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
                gear: over(c.gear, p.gear),
                kp: c.kp.or(p.kp),
                kv: c.kv.or(p.kv),
                dampratio: c.dampratio.or(p.dampratio),
                inheritrange: c.inheritrange.or(p.inheritrange),
                ctrllimited: c.ctrllimited.or(p.ctrllimited),
                forcelimited: c.forcelimited.or(p.forcelimited),
                gaintype: c.gaintype.clone().or_else(|| p.gaintype.clone()),
                biastype: c.biastype.clone().or_else(|| p.biastype.clone()),
                dyntype: c.dyntype.clone().or_else(|| p.dyntype.clone()),
                gainprm: over(c.gainprm, p.gainprm),
                biasprm: over(c.biasprm, p.biasprm),
                dynprm: over(c.dynprm, p.dynprm),
                group: c.group.or(p.group),
                actlimited: c.actlimited.or(p.actlimited),
                actrange: c.actrange.or(p.actrange),
                actearly: c.actearly.or(p.actearly),
                lengthrange: c.lengthrange.or(p.lengthrange),
                nsample: c.nsample.or(p.nsample),
                interp: c.interp.clone().or_else(|| p.interp.clone()),
                delay: c.delay.or(p.delay),
                // Cylinder-specific
                area: c.area.or(p.area),
                diameter: c.diameter.or(p.diameter),
                timeconst: c.timeconst.or(p.timeconst),
                bias: c.bias.or(p.bias),
                // Muscle-specific
                muscle_timeconst: c.muscle_timeconst.or(p.muscle_timeconst),
                range: c.range.or(p.range),
                force: c.force.or(p.force),
                scale: c.scale.or(p.scale),
                lmin: c.lmin.or(p.lmin),
                lmax: c.lmax.or(p.lmax),
                vmax: c.vmax.or(p.vmax),
                fpmax: c.fpmax.or(p.fpmax),
                fvmax: c.fvmax.or(p.fvmax),
                // Adhesion-specific
                gain: c.gain.or(p.gain),
                user: c.user.clone().or_else(|| p.user.clone()),
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
                solref_limit: over(c.solref_limit, p.solref_limit),
                solimp_limit: over(c.solimp_limit, p.solimp_limit),
                solreffriction: over(c.solreffriction, p.solreffriction),
                solimpfriction: over(c.solimpfriction, p.solimpfriction),
                margin: c.margin.or(p.margin),
                material: c.material.clone().or_else(|| p.material.clone()),
                user: c.user.clone().or_else(|| p.user.clone()),
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
                maxhullvert: c.maxhullvert.or(p.maxhullvert),
                inertia: c.inertia.or(p.inertia),
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
            (Some(p), Some(c)) => {
                let (euler, axisangle, xyaxes, zaxis) = alternative_over(
                    (c.euler, c.axisangle, c.xyaxes, c.zaxis),
                    (p.euler, p.axisangle, p.xyaxes, p.zaxis),
                );
                Some(MjcfSiteDefaults {
                    site_type: c.site_type.clone().or_else(|| p.site_type.clone()),
                    size: over(c.size, p.size),
                    rgba: c.rgba.or(p.rgba),
                    group: c.group.or(p.group),
                    pos: c.pos.or(p.pos),
                    quat: c.quat.or(p.quat),
                    euler,
                    axisangle,
                    xyaxes,
                    zaxis,
                    fromto: c.fromto.or(p.fromto),
                    material: c.material.clone().or_else(|| p.material.clone()),
                    user: c.user.clone().or_else(|| p.user.clone()),
                })
            }
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
                friction: over(c.friction, p.friction),
                solref: over(c.solref, p.solref),
                solreffriction: over(c.solreffriction, p.solreffriction),
                solimp: over(c.solimp, p.solimp),
                margin: c.margin.or(p.margin),
                gap: c.gap.or(p.gap),
            }),
        }
    }

    fn merge_equality_defaults(
        parent: Option<&MjcfEqualityDefaults>,
        child: Option<&MjcfEqualityDefaults>,
    ) -> Option<MjcfEqualityDefaults> {
        match (parent, child) {
            (None, None) => None,
            (Some(p), None) => Some(p.clone()),
            (None, Some(c)) => Some(c.clone()),
            (Some(p), Some(c)) => Some(MjcfEqualityDefaults {
                active: c.active.or(p.active),
                solref: over(c.solref, p.solref),
                solimp: over(c.solimp, p.solimp),
            }),
        }
    }
}

/// A layer's four alternative orientations: euler, axisangle, xyaxes, zaxis.
pub type Alternatives = (
    Option<Vector3<f64>>,
    Option<Vector4<f64>>,
    Option<[f64; 6]>,
    Option<Vector3<f64>>,
);

/// A layer's alternative orientation over the one it inherits, as MuJoCo
/// 3.5.0's reader keeps it: one per element, the last read
/// (`ReadAlternative`, `xml_base.cc:54-77`, sets the element's `alt.type` at
/// each one it reads, and an element starts from its class's values). So a
/// layer — an element over its class, a class over its parent — that writes
/// any of the four takes none it inherits; its `quat` does not count
/// (`ReadQuat` leaves `alt.type` alone, so an inherited alternative replaces
/// a written `quat`).
pub fn alternative_over(written: Alternatives, inherited: Alternatives) -> Alternatives {
    if written.0.is_some() || written.1.is_some() || written.2.is_some() || written.3.is_some() {
        written
    } else {
        inherited
    }
}

/// An array attribute written over the one it inherits, as MuJoCo's
/// `ReadAttr` reads one with `exact = false` (`xml_util.cc:798-818`): the `n`
/// numbers written take its first `n` places and the rest keep the inherited
/// ones. An element's values are read so over its class's, and a class's over
/// its parent's (for a site's size, `xml_native_reader.cc:1928`).
pub fn over<const N: usize>(
    written: Option<Prefix<N>>,
    inherited: Option<Prefix<N>>,
) -> Option<Prefix<N>> {
    match (written, inherited) {
        (Some(written), Some(inherited)) => Some(written.over_prefix(&inherited)),
        (written, inherited) => written.or(inherited),
    }
}

impl Default for DefaultResolver {
    /// The main class alone, empty.
    fn default() -> Self {
        Self {
            classes: HashMap::from([(String::new(), MjcfDefault::default())]),
        }
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
    fn apply_equality_defaults_writes_the_element_over_its_class() {
        let resolver = DefaultResolver::new(&[MjcfDefault {
            class: String::new(),
            equality: Some(MjcfEqualityDefaults {
                solref: Prefix::new(&[0.03, 0.5]),
                solimp: Prefix::new(&[0.7, 0.8, 0.002, 0.4, 3.0]),
                ..Default::default()
            }),
            ..Default::default()
        }])
        .unwrap();
        let mut active = None;
        let mut solref = Prefix::new(&[0.05]);
        let mut solimp = Prefix::new(&[0.6]);
        resolver
            .apply_equality_defaults(None, &mut active, &mut solref, &mut solimp)
            .unwrap();
        assert_eq!(solref.unwrap().given(), &[0.05, 0.5]);
        assert_eq!(solimp.unwrap().given(), &[0.6, 0.8, 0.002, 0.4, 3.0]);
    }

    #[test]
    fn test_empty_resolver() {
        let resolver = DefaultResolver::new(&[]).unwrap();
        assert!(resolver.class(None).unwrap().joint.as_ref().is_none());
        assert!(resolver.class(None).unwrap().geom.as_ref().is_none());
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

        let resolver = DefaultResolver::new(&defaults).unwrap();

        let joint_defaults = resolver
            .class(None)
            .unwrap()
            .joint
            .as_ref()
            .expect("should have joint defaults");
        assert_relative_eq!(joint_defaults.damping.unwrap(), 0.5, epsilon = 1e-10);

        let geom_defaults = resolver
            .class(None)
            .unwrap()
            .geom
            .as_ref()
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

        let resolver = DefaultResolver::new(&defaults).unwrap();

        // Root class should have damping 0.5
        let root_defaults = resolver
            .class(None)
            .unwrap()
            .joint
            .as_ref()
            .expect("root defaults");
        assert_relative_eq!(root_defaults.damping.unwrap(), 0.5, epsilon = 1e-10);

        // Arm class should have damping 1.0 (overridden) and armature 0.1
        let arm_defaults = resolver
            .class(Some("arm"))
            .unwrap()
            .joint
            .as_ref()
            .expect("arm defaults");
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

        let resolver = DefaultResolver::new(&defaults).unwrap();

        let arm_defaults = resolver
            .class(Some("arm"))
            .unwrap()
            .joint
            .as_ref()
            .expect("arm defaults");
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
            parent_class: Some(String::new()),
            joint: Some(MjcfJointDefaults {
                damping: Some(2.0),
                stiffness: Some(100.0),
                joint_type: Some(MjcfJointType::Slide),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults).unwrap();

        // Joint with no explicit values
        let joint = MjcfJoint {
            class: Some("test".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_joint(&joint).unwrap();
        assert_relative_eq!(resolved.damping.unwrap(), 2.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.stiffness.unwrap(), 100.0, epsilon = 1e-10);
        assert_eq!(resolved.joint_type, Some(MjcfJointType::Slide));

        // Joint with explicit damping should keep it
        let joint_with_damping = MjcfJoint {
            class: Some("test".to_string()),
            damping: Some(0.5), // Explicitly set
            ..Default::default()
        };

        let resolved = resolver.apply_to_joint(&joint_with_damping).unwrap();
        assert_relative_eq!(resolved.damping.unwrap(), 0.5, epsilon = 1e-10); // Kept explicit value
        assert_relative_eq!(resolved.stiffness.unwrap(), 100.0, epsilon = 1e-10); // Got default
    }

    #[test]
    fn test_apply_to_geom() {
        let defaults = vec![MjcfDefault {
            class: "visual".to_string(),
            parent_class: Some(String::new()),
            geom: Some(MjcfGeomDefaults {
                rgba: Some(Vector4::new(1.0, 0.0, 0.0, 1.0)),
                density: Some(500.0),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults).unwrap();

        let geom = MjcfGeom {
            class: Some("visual".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_geom(&geom).unwrap();
        assert_relative_eq!(resolved.rgba.unwrap().x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.rgba.unwrap().y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(resolved.density.unwrap(), 500.0, epsilon = 1e-10);
    }

    #[test]
    fn test_apply_to_actuator() {
        let defaults = vec![MjcfDefault {
            class: "motor".to_string(),
            parent_class: Some(String::new()),
            actuator: Some(MjcfActuatorDefaults {
                gear: Some(Prefix::full([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])),
                ctrlrange: Some((-1.0, 1.0)),
                kp: Some(50.0),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults).unwrap();

        let actuator = MjcfActuator {
            class: Some("motor".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_actuator(&actuator).unwrap();
        assert_relative_eq!(resolved.gear.unwrap().given()[0], 100.0, epsilon = 1e-10);
        assert_eq!(resolved.ctrlrange, Some((-1.0, 1.0)));
        assert_eq!(resolved.kp, Some(50.0));
    }

    #[test]
    fn test_tendon_defaults() {
        let defaults = vec![MjcfDefault {
            class: "cable".to_string(),
            parent_class: Some(String::new()),
            tendon: Some(MjcfTendonDefaults {
                stiffness: Some(1000.0),
                damping: Some(10.0),
                width: Some(0.01),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults).unwrap();

        let tendon_defaults = resolver
            .class(Some("cable"))
            .unwrap()
            .tendon
            .as_ref()
            .expect("tendon defaults");
        assert_relative_eq!(tendon_defaults.stiffness.unwrap(), 1000.0, epsilon = 1e-10);
        assert_relative_eq!(tendon_defaults.damping.unwrap(), 10.0, epsilon = 1e-10);
        assert_relative_eq!(tendon_defaults.width.unwrap(), 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_sensor_defaults() {
        let defaults = vec![MjcfDefault {
            class: "noisy".to_string(),
            parent_class: Some(String::new()),
            sensor: Some(MjcfSensorDefaults {
                noise: Some(0.01),
                cutoff: Some(100.0),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults).unwrap();

        let sensor_defaults = resolver
            .class(Some("noisy"))
            .unwrap()
            .sensor
            .as_ref()
            .expect("sensor defaults");
        assert_relative_eq!(sensor_defaults.noise.unwrap(), 0.01, epsilon = 1e-10);
        assert_relative_eq!(sensor_defaults.cutoff.unwrap(), 100.0, epsilon = 1e-10);
    }

    #[test]
    fn test_nonexistent_class() {
        let defaults = vec![MjcfDefault {
            class: "existing".to_string(),
            parent_class: Some(String::new()),
            joint: Some(MjcfJointDefaults {
                damping: Some(1.0),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults).unwrap();

        // A class no `<default>` makes is refused (MuJoCo's `GetClass`).
        assert!(matches!(
            resolver.class(Some("nonexistent")),
            Err(MjcfError::UnknownClass { name, .. }) if name == "nonexistent"
        ));
    }

    #[test]
    fn test_apply_to_tendon() {
        use crate::types::MjcfTendon;

        let defaults = vec![MjcfDefault {
            class: "cable".to_string(),
            parent_class: Some(String::new()),
            tendon: Some(MjcfTendonDefaults {
                stiffness: Some(1000.0),
                damping: Some(10.0),
                width: Some(0.01),
                rgba: Some(Vector4::new(1.0, 0.0, 0.0, 1.0)),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults).unwrap();

        // Tendon with no explicit values
        let tendon = MjcfTendon {
            class: Some("cable".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_tendon(&tendon).unwrap();
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

        let resolved = resolver.apply_to_tendon(&tendon_with_stiffness).unwrap();
        assert_relative_eq!(resolved.stiffness.unwrap(), 500.0, epsilon = 1e-10); // Kept explicit
        assert_relative_eq!(resolved.damping.unwrap(), 10.0, epsilon = 1e-10); // Got default
    }

    #[test]
    fn test_apply_to_sensor() {
        use crate::types::MjcfSensor;

        let defaults = vec![MjcfDefault {
            class: "noisy".to_string(),
            parent_class: Some(String::new()),
            sensor: Some(MjcfSensorDefaults {
                noise: Some(0.01),
                cutoff: Some(100.0),
                user: Some(vec![1.0, 2.0, 3.0]),
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults).unwrap();

        // Sensor with no explicit values
        let sensor = MjcfSensor {
            class: Some("noisy".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_sensor(&sensor).unwrap();
        assert_eq!(resolved.noise, Some(0.01));
        assert_eq!(resolved.cutoff, Some(100.0));
        assert_eq!(resolved.user, vec![1.0, 2.0, 3.0]);

        // Sensor with explicit noise should keep it
        let sensor_with_noise = MjcfSensor {
            class: Some("noisy".to_string()),
            noise: Some(0.05),
            ..Default::default()
        };

        let resolved = resolver.apply_to_sensor(&sensor_with_noise).unwrap();
        assert_eq!(resolved.noise, Some(0.05)); // Kept explicit
        assert_eq!(resolved.cutoff, Some(100.0)); // Got default
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

        let resolver = DefaultResolver::new(&defaults).unwrap();

        let tendon = MjcfTendon {
            class: Some("strong".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_tendon(&tendon).unwrap();
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

        let resolver = DefaultResolver::new(&defaults).unwrap();

        let sensor = MjcfSensor {
            class: Some("filtered".to_string()),
            ..Default::default()
        };

        let resolved = resolver.apply_to_sensor(&sensor).unwrap();
        assert_eq!(resolved.noise, Some(0.001)); // Inherited from root
        assert_eq!(resolved.cutoff, Some(10.0)); // From "filtered"
    }

    // ---- T14 (Spec B): Margin parsed from MJCF defaults ----
    #[test]
    fn t14_margin_parsed_from_defaults() {
        let defaults = vec![MjcfDefault {
            class: String::new(),
            parent_class: None,
            joint: Some(MjcfJointDefaults {
                margin: Some(0.05),
                ..Default::default()
            }),
            ..Default::default()
        }];

        let resolver = DefaultResolver::new(&defaults).unwrap();
        let joint = MjcfJoint::default(); // no explicit margin
        let resolved = resolver.apply_to_joint(&joint).unwrap();
        assert_eq!(
            resolved.margin,
            Some(0.05),
            "margin should inherit from defaults"
        );

        // Explicit override
        let joint_override = MjcfJoint {
            margin: Some(0.1),
            ..Default::default()
        };
        let resolved_override = resolver.apply_to_joint(&joint_override).unwrap();
        assert_eq!(
            resolved_override.margin,
            Some(0.1),
            "explicit margin should override default"
        );
    }

    // T14b: End-to-end MJCF parsing + builder → Model.jnt_margin
    #[test]
    fn t14b_margin_end_to_end_builder() {
        use crate::builder::model_from_mjcf;
        use crate::parse_mjcf_str;

        let mjcf_str = r#"
        <mujoco>
          <default>
            <joint margin="0.05"/>
          </default>
          <worldbody>
            <body>
              <joint type="hinge" limited="true" range="-1 1"/>
              <geom type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
        "#;

        let mjcf = parse_mjcf_str(mjcf_str).expect("parse failed");
        let model = model_from_mjcf(&mjcf, None).expect("build failed");

        assert_eq!(model.jnt_margin.len(), 1, "should have 1 joint");
        assert!(
            (model.jnt_margin[0] - 0.05).abs() < 1e-15,
            "jnt_margin = {}, expected 0.05",
            model.jnt_margin[0]
        );

        // Test explicit override
        let mjcf_str2 = r#"
        <mujoco>
          <default>
            <joint margin="0.05"/>
          </default>
          <worldbody>
            <body>
              <joint type="hinge" margin="0.1" limited="true" range="-1 1"/>
              <geom type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
        "#;

        let mjcf2 = parse_mjcf_str(mjcf_str2).expect("parse failed");
        let model2 = model_from_mjcf(&mjcf2, None).expect("build failed");
        assert!(
            (model2.jnt_margin[0] - 0.1).abs() < 1e-15,
            "explicit margin = {}, expected 0.1",
            model2.jnt_margin[0]
        );
    }
}
