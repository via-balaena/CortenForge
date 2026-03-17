//! Mechanism builder and validated mechanism type.
//!
//! A [`Mechanism`] is a validated multi-part assembly: parts connected by joints,
//! routed with tendons, driven by actuators. It is the atomic unit of design in
//! cf-design.
//!
//! Build a mechanism using [`MechanismBuilder`]:
//!
//! ```
//! use cf_design::{
//!     Mechanism, Part, Material, Solid, JointDef, JointKind,
//!     TendonDef, TendonWaypoint, ActuatorDef, ActuatorKind, PrintProfile,
//! };
//! use nalgebra::{Point3, Vector3};
//!
//! let pla = Material::new("PLA", 1250.0);
//!
//! let mechanism = Mechanism::builder("two_finger_gripper")
//!     .part(Part::new("palm", Solid::cuboid(Vector3::new(18.0, 22.0, 4.0)), pla.clone()))
//!     .part(Part::new("finger_1", Solid::capsule(3.0, 12.0), pla.clone()))
//!     .joint(JointDef::new(
//!         "knuckle_1", "palm", "finger_1",
//!         JointKind::Revolute,
//!         Point3::new(10.0, 20.0, 0.0),
//!         Vector3::x(),
//!     ))
//!     .build();
//!
//! assert_eq!(mechanism.name(), "two_finger_gripper");
//! assert_eq!(mechanism.parts().len(), 2);
//! ```
//!
//! Validation runs automatically in [`MechanismBuilder::build`], or explicitly
//! via [`MechanismBuilder::validate`].

use std::collections::HashSet;
use std::fmt;

use super::actuator::ActuatorDef;
use super::joint::JointDef;
use super::part::Part;
use super::print::PrintProfile;
use super::tendon::TendonDef;

// ── MechanismError ──────────────────────────────────────────────────────

/// Structural validation error in a mechanism definition.
///
/// These represent programming errors — invalid cross-references between
/// parts, joints, tendons, and actuators. Distinct from [`super::DesignWarning`],
/// which represents manufacturing constraint violations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MechanismError {
    /// Two parts share the same name.
    DuplicatePart(String),
    /// Two joints share the same name.
    DuplicateJoint(String),
    /// Two tendons share the same name.
    DuplicateTendon(String),
    /// Two actuators share the same name.
    DuplicateActuator(String),
    /// A joint references a part that does not exist.
    JointRefersToUnknownPart {
        /// Joint name.
        joint: String,
        /// The unknown part name (parent or child).
        part: String,
    },
    /// A tendon waypoint references a part that does not exist.
    TendonRefersToUnknownPart {
        /// Tendon name.
        tendon: String,
        /// The unknown part name.
        part: String,
    },
    /// An actuator references a tendon that does not exist.
    ActuatorRefersToUnknownTendon {
        /// Actuator name.
        actuator: String,
        /// The unknown tendon name.
        tendon: String,
    },
    /// A part is not referenced by any joint (neither as parent nor child).
    OrphanPart(String),
}

impl fmt::Display for MechanismError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicatePart(name) => write!(f, "duplicate part name: \"{name}\""),
            Self::DuplicateJoint(name) => write!(f, "duplicate joint name: \"{name}\""),
            Self::DuplicateTendon(name) => write!(f, "duplicate tendon name: \"{name}\""),
            Self::DuplicateActuator(name) => write!(f, "duplicate actuator name: \"{name}\""),
            Self::JointRefersToUnknownPart { joint, part } => {
                write!(f, "joint \"{joint}\" references unknown part \"{part}\"")
            }
            Self::TendonRefersToUnknownPart { tendon, part } => {
                write!(f, "tendon \"{tendon}\" references unknown part \"{part}\"")
            }
            Self::ActuatorRefersToUnknownTendon { actuator, tendon } => {
                write!(
                    f,
                    "actuator \"{actuator}\" references unknown tendon \"{tendon}\""
                )
            }
            Self::OrphanPart(name) => {
                write!(f, "orphan part: \"{name}\" not connected by any joint")
            }
        }
    }
}

// ── MechanismBuilder ────────────────────────────────────────────────────

/// Builder for constructing a validated [`Mechanism`].
///
/// Collects parts, joints, tendons, actuators, and an optional print profile.
/// Call [`build`](Self::build) to validate cross-references and apply tendon
/// channel subtraction.
#[derive(Debug)]
pub struct MechanismBuilder {
    name: String,
    parts: Vec<Part>,
    joints: Vec<JointDef>,
    tendons: Vec<TendonDef>,
    actuators: Vec<ActuatorDef>,
    print_profile: Option<PrintProfile>,
}

impl MechanismBuilder {
    /// Add a part to the mechanism.
    #[must_use]
    pub fn part(mut self, part: Part) -> Self {
        self.parts.push(part);
        self
    }

    /// Add a joint connecting two parts.
    #[must_use]
    pub fn joint(mut self, joint: JointDef) -> Self {
        self.joints.push(joint);
        self
    }

    /// Add a tendon routed through parts.
    #[must_use]
    pub fn tendon(mut self, tendon: TendonDef) -> Self {
        self.tendons.push(tendon);
        self
    }

    /// Add an actuator driving a tendon.
    #[must_use]
    pub fn actuator(mut self, actuator: ActuatorDef) -> Self {
        self.actuators.push(actuator);
        self
    }

    /// Set the manufacturing print profile.
    #[must_use]
    pub const fn print_profile(mut self, profile: PrintProfile) -> Self {
        self.print_profile = Some(profile);
        self
    }

    /// Validate the mechanism definition without building.
    ///
    /// Returns all structural errors found. An empty vector means the
    /// mechanism is valid.
    #[must_use]
    pub fn validate(&self) -> Vec<MechanismError> {
        let mut errors = Vec::new();

        // ── Duplicate name detection ────────────────────────────────
        check_duplicates(self.parts.iter().map(Part::name), |name| {
            errors.push(MechanismError::DuplicatePart(name));
        });
        check_duplicates(self.joints.iter().map(JointDef::name), |name| {
            errors.push(MechanismError::DuplicateJoint(name));
        });
        check_duplicates(self.tendons.iter().map(TendonDef::name), |name| {
            errors.push(MechanismError::DuplicateTendon(name));
        });
        check_duplicates(self.actuators.iter().map(ActuatorDef::name), |name| {
            errors.push(MechanismError::DuplicateActuator(name));
        });

        // ── Cross-reference validation ──────────────────────────────
        let part_names: HashSet<&str> = self.parts.iter().map(Part::name).collect();
        let tendon_names: HashSet<&str> = self.tendons.iter().map(TendonDef::name).collect();

        // Joint parent/child must reference existing parts.
        for joint in &self.joints {
            if !part_names.contains(joint.parent()) {
                errors.push(MechanismError::JointRefersToUnknownPart {
                    joint: joint.name().to_owned(),
                    part: joint.parent().to_owned(),
                });
            }
            if !part_names.contains(joint.child()) {
                errors.push(MechanismError::JointRefersToUnknownPart {
                    joint: joint.name().to_owned(),
                    part: joint.child().to_owned(),
                });
            }
        }

        // Tendon waypoints must reference existing parts.
        for tendon in &self.tendons {
            let mut seen_parts = HashSet::new();
            for wp in tendon.waypoints() {
                // Only report each unknown part once per tendon.
                if !part_names.contains(wp.part()) && seen_parts.insert(wp.part().to_owned()) {
                    errors.push(MechanismError::TendonRefersToUnknownPart {
                        tendon: tendon.name().to_owned(),
                        part: wp.part().to_owned(),
                    });
                }
            }
        }

        // Actuator tendon must reference existing tendons.
        for actuator in &self.actuators {
            if !tendon_names.contains(actuator.tendon()) {
                errors.push(MechanismError::ActuatorRefersToUnknownTendon {
                    actuator: actuator.name().to_owned(),
                    tendon: actuator.tendon().to_owned(),
                });
            }
        }

        // ── Orphan parts ────────────────────────────────────────────
        // A part is orphaned if it is not referenced by any joint as
        // parent or child. Exception: if there are 0 or 1 parts total,
        // no joints are needed (degenerate mechanism or single-body).
        if self.parts.len() > 1 {
            let mut connected: HashSet<&str> = HashSet::new();
            for joint in &self.joints {
                connected.insert(joint.parent());
                connected.insert(joint.child());
            }
            for part in &self.parts {
                if !connected.contains(part.name()) {
                    errors.push(MechanismError::OrphanPart(part.name().to_owned()));
                }
            }
        }

        errors
    }

    /// Validate and build the mechanism.
    ///
    /// Runs [`validate`](Self::validate) and asserts no errors. Then applies
    /// tendon channel subtraction to each part.
    ///
    /// # Panics
    ///
    /// Panics if validation finds any [`MechanismError`]s. Call
    /// [`validate`](Self::validate) first to inspect errors without panicking.
    #[must_use]
    pub fn build(self) -> Mechanism {
        let errors = self.validate();
        assert!(
            errors.is_empty(),
            "mechanism \"{}\" has {} validation error(s):\n{}",
            self.name,
            errors.len(),
            format_errors(&errors),
        );

        // Apply tendon channel subtraction to each part.
        let parts: Vec<Part> = self
            .parts
            .into_iter()
            .map(|part| part.with_tendon_channels(&self.tendons))
            .collect();

        Mechanism {
            name: self.name,
            parts,
            joints: self.joints,
            tendons: self.tendons,
            actuators: self.actuators,
            print_profile: self.print_profile,
        }
    }
}

/// Format a list of errors for a panic message.
fn format_errors(errors: &[MechanismError]) -> String {
    errors
        .iter()
        .enumerate()
        .map(|(i, e)| format!("  {}. {e}", i + 1))
        .collect::<Vec<_>>()
        .join("\n")
}

/// Report duplicate names via a callback.
fn check_duplicates<'a>(names: impl Iterator<Item = &'a str>, mut report: impl FnMut(String)) {
    let mut seen = HashSet::new();
    for name in names {
        if !seen.insert(name) {
            report(name.to_owned());
        }
    }
}

// ── Mechanism ───────────────────────────────────────────────────────────

/// A validated multi-part mechanism assembly.
///
/// Created via [`MechanismBuilder`]. All cross-references are verified
/// and tendon channels are subtracted from parts at build time.
///
/// # Invariants
///
/// - All joint parent/child names reference parts that exist.
/// - All tendon waypoint part names reference parts that exist.
/// - All actuator tendon names reference tendons that exist.
/// - No orphan parts (every part reachable via joints, or single-part mechanism).
/// - Part solids have tendon channels already subtracted.
#[derive(Debug)]
pub struct Mechanism {
    name: String,
    parts: Vec<Part>,
    joints: Vec<JointDef>,
    tendons: Vec<TendonDef>,
    actuators: Vec<ActuatorDef>,
    print_profile: Option<PrintProfile>,
}

impl Mechanism {
    /// Start building a new mechanism with the given name.
    ///
    /// # Panics
    ///
    /// Panics if `name` is empty.
    #[must_use]
    pub fn builder(name: impl Into<String>) -> MechanismBuilder {
        let name = name.into();
        assert!(!name.is_empty(), "mechanism name must not be empty");
        MechanismBuilder {
            name,
            parts: Vec::new(),
            joints: Vec::new(),
            tendons: Vec::new(),
            actuators: Vec::new(),
            print_profile: None,
        }
    }

    /// Mechanism name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Parts (with tendon channels already subtracted).
    #[must_use]
    pub fn parts(&self) -> &[Part] {
        &self.parts
    }

    /// Joint definitions.
    #[must_use]
    pub fn joints(&self) -> &[JointDef] {
        &self.joints
    }

    /// Tendon definitions.
    #[must_use]
    pub fn tendons(&self) -> &[TendonDef] {
        &self.tendons
    }

    /// Actuator definitions.
    #[must_use]
    pub fn actuators(&self) -> &[ActuatorDef] {
        &self.actuators
    }

    /// Manufacturing print profile, if set.
    #[must_use]
    pub const fn print_profile(&self) -> Option<&PrintProfile> {
        self.print_profile.as_ref()
    }

    /// Validate the mechanism against manufacturing constraints and geometric checks.
    ///
    /// Manufacturing checks (wall thickness, hole diameter, feature resolution)
    /// require a [`PrintProfile`](super::PrintProfile) and are skipped without one.
    /// Geometric checks (joint anchor bounds) always run.
    ///
    /// Returns all warnings found. An empty vector means the mechanism passes
    /// all checks.
    #[must_use]
    pub fn validate(&self) -> Vec<super::DesignWarning> {
        super::validate::validate_mechanism(
            &self.parts,
            &self.joints,
            &self.tendons,
            self.print_profile.as_ref(),
        )
    }

    /// Generate MJCF XML for simulation.
    ///
    /// Each part is meshed at the given `resolution` (passed to
    /// [`Solid::mesh`](crate::Solid::mesh)). The generated XML embeds
    /// vertex/face data inline in `<mesh>` asset elements.
    ///
    /// # Panics
    ///
    /// Panics if `resolution` is not positive and finite.
    #[must_use]
    pub fn to_mjcf(&self, resolution: f64) -> String {
        super::mjcf::generate(self, resolution)
    }

    /// Generate collision shapes for cf-geometry.
    ///
    /// Returns `(part_name, Shape)` pairs in part declaration order.
    ///
    /// - [`ShapeMode::Sdf`](super::ShapeMode::Sdf): evaluates the implicit
    ///   field on a grid → `Shape::Sdf`. Fast, best for design iteration.
    /// - [`ShapeMode::Mesh`](super::ShapeMode::Mesh): meshes the field →
    ///   `Shape::TriangleMesh` with BVH. Accurate, best for final simulation.
    ///
    /// # Panics
    ///
    /// Panics if the resolution/tolerance in `mode` is not positive and finite.
    #[must_use]
    pub fn to_shapes(&self, mode: &super::ShapeMode) -> Vec<(String, cf_geometry::Shape)> {
        super::shapes::generate(self, mode)
    }

    /// Generate per-part meshes for 3D printing.
    ///
    /// Returns `(part_name, IndexedMesh)` pairs in part declaration order.
    /// If a [`PrintProfile`](super::PrintProfile) is set, each part is shrunk
    /// by half the clearance so that mating surfaces have the required gap.
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn to_stl_kit(&self, tolerance: f64) -> Vec<(String, cf_geometry::IndexedMesh)> {
        super::stl::generate(self, tolerance)
    }
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::{ActuatorKind, JointKind, Material, Solid, TendonWaypoint};

    // ── Helpers ─────────────────────────────────────────────────────

    fn pla() -> Material {
        Material::new("PLA", 1250.0)
    }

    fn palm_solid() -> Solid {
        Solid::cuboid(Vector3::new(18.0, 22.0, 4.0))
    }

    fn finger_solid() -> Solid {
        Solid::capsule(3.0, 12.0)
    }

    fn simple_two_part_builder() -> MechanismBuilder {
        Mechanism::builder("gripper")
            .part(Part::new("palm", palm_solid(), pla()))
            .part(Part::new("finger_1", finger_solid(), pla()))
            .joint(JointDef::new(
                "knuckle_1",
                "palm",
                "finger_1",
                JointKind::Revolute,
                Point3::new(10.0, 20.0, 0.0),
                Vector3::x(),
            ))
    }

    // ── 1. Bio-gripper builds successfully ──────────────────────────

    #[test]
    fn bio_gripper_builds() {
        let palm = palm_solid();
        let finger = finger_solid();
        let mat = pla();

        let mechanism = Mechanism::builder("bio_gripper")
            .part(Part::new("palm", palm, mat.clone()))
            .part(Part::new("finger_1", finger.clone(), mat.clone()))
            .part(Part::new("finger_2", finger, mat))
            .joint(
                JointDef::new(
                    "knuckle_1",
                    "palm",
                    "finger_1",
                    JointKind::Revolute,
                    Point3::new(10.0, 20.0, 0.0),
                    Vector3::x(),
                )
                .with_range(-0.1, 1.8),
            )
            .joint(
                JointDef::new(
                    "knuckle_2",
                    "palm",
                    "finger_2",
                    JointKind::Revolute,
                    Point3::new(-10.0, 20.0, 0.0),
                    Vector3::x(),
                )
                .with_range(-0.1, 1.8),
            )
            .tendon(
                TendonDef::new(
                    "flexor_1",
                    vec![
                        TendonWaypoint::new("palm", Point3::new(8.0, -10.0, 0.0)),
                        TendonWaypoint::new("palm", Point3::new(8.0, 18.0, 0.0)),
                        TendonWaypoint::new("finger_1", Point3::new(0.0, 5.0, 0.0)),
                        TendonWaypoint::new("finger_1", Point3::new(0.0, 20.0, 0.0)),
                    ],
                    1.5,
                )
                .with_stiffness(100.0)
                .with_damping(5.0),
            )
            .actuator(
                ActuatorDef::new("motor_1", "flexor_1", ActuatorKind::Motor, (-50.0, 50.0))
                    .with_ctrl_range(-1.0, 1.0),
            )
            .print_profile(PrintProfile::new(0.3, 1.0, 2.0))
            .build();

        assert_eq!(mechanism.name(), "bio_gripper");
        assert_eq!(mechanism.parts().len(), 3);
        assert_eq!(mechanism.joints().len(), 2);
        assert_eq!(mechanism.tendons().len(), 1);
        assert_eq!(mechanism.actuators().len(), 1);
        assert!(mechanism.print_profile().is_some());
    }

    // ── 2. Getters return expected values ───────────────────────────

    #[test]
    fn getters_return_correct_values() {
        let mechanism = simple_two_part_builder().build();

        assert_eq!(mechanism.name(), "gripper");
        assert_eq!(mechanism.parts()[0].name(), "palm");
        assert_eq!(mechanism.parts()[1].name(), "finger_1");
        assert_eq!(mechanism.joints()[0].name(), "knuckle_1");
        assert!(mechanism.tendons().is_empty());
        assert!(mechanism.actuators().is_empty());
        assert!(mechanism.print_profile().is_none());
    }

    // ── 3. Channel auto-application ─────────────────────────────────

    #[test]
    fn channels_subtracted_on_build() {
        let mechanism = Mechanism::builder("channeled")
            .part(Part::new(
                "palm",
                Solid::cuboid(Vector3::new(20.0, 30.0, 10.0)),
                pla(),
            ))
            .part(Part::new("finger", finger_solid(), pla()))
            .joint(JointDef::new(
                "j",
                "palm",
                "finger",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .tendon(TendonDef::new(
                "flexor",
                vec![
                    TendonWaypoint::new("palm", Point3::new(8.0, -10.0, 0.0)),
                    TendonWaypoint::new("palm", Point3::new(8.0, 18.0, 0.0)),
                    TendonWaypoint::new("finger", Point3::new(0.0, 5.0, 0.0)),
                    TendonWaypoint::new("finger", Point3::new(0.0, 20.0, 0.0)),
                ],
                1.5,
            ))
            .build();

        // The channel centerline in the palm should now be void (positive).
        let palm = &mechanism.parts()[0];
        assert_eq!(palm.name(), "palm");
        for y in [-5.0, 0.0, 5.0, 10.0, 15.0] {
            let p = Point3::new(8.0, y, 0.0);
            let val = palm.solid().evaluate(&p);
            assert!(
                val > 0.0,
                "channel centerline at y={y} should be void, got {val}"
            );
        }

        // A point far from the channel should still be solid.
        let val = palm.solid().evaluate(&Point3::new(-5.0, 0.0, 0.0));
        assert!(
            val < 0.0,
            "point away from channel should be solid, got {val}"
        );
    }

    // ── 4. Single-part mechanism ────────────────────────────────────

    #[test]
    fn single_part_no_joints_is_valid() {
        let mechanism = Mechanism::builder("solo")
            .part(Part::new("body", Solid::sphere(5.0), pla()))
            .build();

        assert_eq!(mechanism.parts().len(), 1);
        assert!(mechanism.joints().is_empty());
    }

    // ── 5–8. Duplicate name errors ──────────────────────────────────

    #[test]
    fn duplicate_part_name() {
        let errors = Mechanism::builder("dup")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .part(Part::new("a", Solid::sphere(2.0), pla()))
            .validate();

        assert!(errors.contains(&MechanismError::DuplicatePart("a".into())));
    }

    #[test]
    fn duplicate_joint_name() {
        let errors = Mechanism::builder("dup")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .part(Part::new("b", Solid::sphere(1.0), pla()))
            .joint(JointDef::new(
                "j",
                "a",
                "b",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .joint(JointDef::new(
                "j",
                "a",
                "b",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .validate();

        assert!(errors.contains(&MechanismError::DuplicateJoint("j".into())));
    }

    #[test]
    fn duplicate_tendon_name() {
        let wps = vec![
            TendonWaypoint::new("a", Point3::origin()),
            TendonWaypoint::new("a", Point3::new(0.0, 10.0, 0.0)),
        ];
        let errors = Mechanism::builder("dup")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .tendon(TendonDef::new("t", wps.clone(), 1.0))
            .tendon(TendonDef::new("t", wps, 1.0))
            .validate();

        assert!(errors.contains(&MechanismError::DuplicateTendon("t".into())));
    }

    #[test]
    fn duplicate_actuator_name() {
        let wps = vec![
            TendonWaypoint::new("a", Point3::origin()),
            TendonWaypoint::new("a", Point3::new(0.0, 10.0, 0.0)),
        ];
        let errors = Mechanism::builder("dup")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .tendon(TendonDef::new("t", wps, 1.0))
            .actuator(ActuatorDef::new(
                "act",
                "t",
                ActuatorKind::Motor,
                (-1.0, 1.0),
            ))
            .actuator(ActuatorDef::new(
                "act",
                "t",
                ActuatorKind::Motor,
                (-1.0, 1.0),
            ))
            .validate();

        assert!(errors.contains(&MechanismError::DuplicateActuator("act".into())));
    }

    // ── 9. Joint unknown parent ─────────────────────────────────────

    #[test]
    fn joint_unknown_parent() {
        let errors = Mechanism::builder("bad")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .joint(JointDef::new(
                "j",
                "nonexistent",
                "a",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .validate();

        assert!(errors.contains(&MechanismError::JointRefersToUnknownPart {
            joint: "j".into(),
            part: "nonexistent".into(),
        }));
    }

    // ── 10. Joint unknown child ─────────────────────────────────────

    #[test]
    fn joint_unknown_child() {
        let errors = Mechanism::builder("bad")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .joint(JointDef::new(
                "j",
                "a",
                "ghost",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .validate();

        assert!(errors.contains(&MechanismError::JointRefersToUnknownPart {
            joint: "j".into(),
            part: "ghost".into(),
        }));
    }

    // ── 11. Tendon unknown part ─────────────────────────────────────

    #[test]
    fn tendon_unknown_part() {
        let errors = Mechanism::builder("bad")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .tendon(TendonDef::new(
                "t",
                vec![
                    TendonWaypoint::new("a", Point3::origin()),
                    TendonWaypoint::new("missing", Point3::new(0.0, 10.0, 0.0)),
                ],
                1.0,
            ))
            .validate();

        assert!(errors.contains(&MechanismError::TendonRefersToUnknownPart {
            tendon: "t".into(),
            part: "missing".into(),
        }));
    }

    // ── 12. Actuator unknown tendon ─────────────────────────────────

    #[test]
    fn actuator_unknown_tendon() {
        let errors = Mechanism::builder("bad")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .actuator(ActuatorDef::new(
                "act",
                "nope",
                ActuatorKind::Motor,
                (-1.0, 1.0),
            ))
            .validate();

        assert!(
            errors.contains(&MechanismError::ActuatorRefersToUnknownTendon {
                actuator: "act".into(),
                tendon: "nope".into(),
            })
        );
    }

    // ── 13. Orphan part ─────────────────────────────────────────────

    #[test]
    fn orphan_part_detected() {
        let errors = Mechanism::builder("orphan")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .part(Part::new("b", Solid::sphere(1.0), pla()))
            .part(Part::new("c", Solid::sphere(1.0), pla()))
            .joint(JointDef::new(
                "j",
                "a",
                "b",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .validate();

        // "c" is orphaned — not referenced by any joint.
        assert!(errors.contains(&MechanismError::OrphanPart("c".into())));
        // "a" and "b" should NOT be orphaned.
        assert!(!errors.contains(&MechanismError::OrphanPart("a".into())));
        assert!(!errors.contains(&MechanismError::OrphanPart("b".into())));
    }

    // ── 14. Multiple errors collected ───────────────────────────────

    #[test]
    fn multiple_errors_collected() {
        let errors = Mechanism::builder("multi_err")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .part(Part::new("a", Solid::sphere(1.0), pla())) // duplicate
            .part(Part::new("b", Solid::sphere(1.0), pla())) // orphan
            .joint(JointDef::new(
                "j",
                "a",
                "ghost",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            )) // unknown child
            .actuator(ActuatorDef::new("act", "nope", ActuatorKind::Motor, (-1.0, 1.0))) // unknown tendon
            .validate();

        // Should have at least 3 distinct errors.
        assert!(
            errors.len() >= 3,
            "expected multiple errors, got {}: {:?}",
            errors.len(),
            errors
        );
    }

    // ── 15. Print profile is optional ───────────────────────────────

    #[test]
    fn builds_without_print_profile() {
        let mechanism = simple_two_part_builder().build();
        assert!(mechanism.print_profile().is_none());
    }

    // ── Panic tests ─────────────────────────────────────────────────

    #[test]
    #[should_panic(expected = "mechanism name must not be empty")]
    fn empty_name_panics() {
        let _b = Mechanism::builder("");
    }

    #[test]
    #[should_panic(expected = "validation error")]
    fn build_panics_on_invalid() {
        // Joint references nonexistent parts.
        let _m = Mechanism::builder("bad")
            .joint(JointDef::new(
                "j",
                "a",
                "b",
                JointKind::Revolute,
                Point3::origin(),
                Vector3::z(),
            ))
            .build();
    }
}
