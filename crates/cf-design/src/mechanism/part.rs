//! Part, flex zone, and plane types.
//!
//! A [`Part`] is a named solid body with a material and optional flex zones.
//! It maps to a single MJCF `<body>` + `<geom>` during generation (Session 10).
//!
//! [`FlexZone`]s mark thin cross-sections of a monolithic part that act as
//! flexure joints. Auto-splitting logic (Session 17) converts these into
//! separate MJCF bodies with spring-damper joints.

use nalgebra::Vector3;

use super::joint::JointKind;
use super::material::Material;
use crate::Solid;

/// A geometric plane defined by a unit normal and signed offset.
///
/// The plane is the set of points **p** where `dot(normal, p) = offset`.
/// Used to define [`FlexZone`] cross-sections.
#[derive(Debug, Clone, PartialEq)]
pub struct Plane {
    normal: Vector3<f64>,
    offset: f64,
}

impl Plane {
    /// Create a plane from a normal direction and offset.
    ///
    /// The normal is normalized internally. The offset is the signed distance
    /// from the origin to the plane along the normal direction.
    ///
    /// # Panics
    ///
    /// Panics if `normal` is zero or non-finite, or `offset` is non-finite.
    #[must_use]
    pub fn new(normal: Vector3<f64>, offset: f64) -> Self {
        assert!(
            normal.iter().all(|c| c.is_finite()),
            "plane normal must have finite components"
        );
        let norm = normal.norm();
        assert!(norm > 1e-10, "plane normal must be non-zero");
        assert!(offset.is_finite(), "plane offset must be finite");
        Self {
            normal: normal / norm,
            offset,
        }
    }

    /// Unit normal of the plane.
    #[must_use]
    pub const fn normal(&self) -> &Vector3<f64> {
        &self.normal
    }

    /// Signed offset from the origin along the normal.
    #[must_use]
    pub const fn offset(&self) -> f64 {
        self.offset
    }
}

/// A thin cross-section of a monolithic part that acts as a flexure joint.
///
/// For simulation, the part is auto-split at flex zones into separate MJCF
/// bodies connected by joints with spring-damper properties derived from the
/// material and cross-section geometry (Session 17).
#[derive(Debug, Clone)]
pub struct FlexZone {
    name: String,
    plane: Plane,
    width: f64,
    kind: JointKind,
    axis: Vector3<f64>,
}

impl FlexZone {
    /// Create a flex zone.
    ///
    /// - `plane` — cross-section plane defining where the flex zone is.
    /// - `width` — thickness of the thin section (along the plane normal).
    /// - `kind` — joint type to generate at this flex zone.
    /// - `axis` — axis of rotation/translation (in part-local frame).
    ///
    /// # Panics
    ///
    /// Panics if `width` is not positive/finite, `name` is empty, or `axis`
    /// is zero/non-finite.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        plane: Plane,
        width: f64,
        kind: JointKind,
        axis: Vector3<f64>,
    ) -> Self {
        let name = name.into();
        assert!(!name.is_empty(), "flex zone name must not be empty");
        assert!(
            width > 0.0 && width.is_finite(),
            "flex zone width must be positive and finite, got {width}"
        );
        assert!(
            axis.iter().all(|c| c.is_finite()) && axis.norm() > 1e-10,
            "flex zone axis must be non-zero and finite"
        );

        Self {
            name,
            plane,
            width,
            kind,
            axis: axis.normalize(),
        }
    }

    /// Flex zone name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Cross-section plane.
    #[must_use]
    pub const fn plane(&self) -> &Plane {
        &self.plane
    }

    /// Width of the thin section.
    #[must_use]
    pub const fn width(&self) -> f64 {
        self.width
    }

    /// Joint kind generated at this flex zone.
    #[must_use]
    pub const fn kind(&self) -> JointKind {
        self.kind
    }

    /// Axis of rotation/translation (unit vector, normalized at construction).
    #[must_use]
    pub const fn axis(&self) -> &Vector3<f64> {
        &self.axis
    }
}

/// A named solid body with material properties and optional flex zones.
///
/// A `Part` wraps a [`Solid`] (implicit surface geometry), a [`Material`],
/// and zero or more [`FlexZone`]s. It maps to a single MJCF `<body>` +
/// `<geom>` during mechanism-to-MJCF generation.
///
/// # Example
///
/// ```
/// use cf_design::{Part, Material, Solid};
///
/// let finger = Part::new(
///     "finger",
///     Solid::capsule(3.0, 12.0),
///     Material::new("PLA", 1250.0),
/// );
/// ```
#[derive(Debug, Clone)]
pub struct Part {
    name: String,
    solid: Solid,
    material: Material,
    flex_zones: Vec<FlexZone>,
}

impl Part {
    /// Create a part with the given name, solid geometry, and material.
    ///
    /// Flex zones can be added with [`with_flex_zone`](Self::with_flex_zone).
    ///
    /// # Panics
    ///
    /// Panics if `name` is empty.
    #[must_use]
    pub fn new(name: impl Into<String>, solid: Solid, material: Material) -> Self {
        let name = name.into();
        assert!(!name.is_empty(), "part name must not be empty");
        Self {
            name,
            solid,
            material,
            flex_zones: Vec::new(),
        }
    }

    /// Add a flex zone to this part.
    #[must_use]
    pub fn with_flex_zone(mut self, zone: FlexZone) -> Self {
        self.flex_zones.push(zone);
        self
    }

    /// Part name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Solid geometry (implicit surface field).
    #[must_use]
    pub const fn solid(&self) -> &Solid {
        &self.solid
    }

    /// Physical material.
    #[must_use]
    pub const fn material(&self) -> &Material {
        &self.material
    }

    /// Flex zones (empty if this part has no compliant sections).
    #[must_use]
    pub fn flex_zones(&self) -> &[FlexZone] {
        &self.flex_zones
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn part_new_valid() {
        let p = Part::new("finger", Solid::sphere(5.0), Material::new("PLA", 1250.0));
        assert_eq!(p.name(), "finger");
        assert_eq!(p.material().name, "PLA");
        assert!(p.flex_zones().is_empty());
    }

    #[test]
    fn part_solid_is_accessible() {
        let p = Part::new("ball", Solid::sphere(3.0), Material::new("ABS", 1040.0));
        // Verify the solid evaluates correctly (inside the sphere).
        let val = p.solid().evaluate(&nalgebra::Point3::origin());
        assert!(val < 0.0, "origin should be inside a sphere");
    }

    #[test]
    fn part_with_flex_zone() {
        let zone = FlexZone::new(
            "bend_1",
            Plane::new(Vector3::z(), 5.0),
            0.5,
            JointKind::Revolute,
            Vector3::x(),
        );
        let p = Part::new(
            "link",
            Solid::cuboid(nalgebra::Vector3::new(2.0, 2.0, 10.0)),
            Material::new("PLA", 1250.0),
        )
        .with_flex_zone(zone);
        assert_eq!(p.flex_zones().len(), 1);
        assert_eq!(p.flex_zones()[0].name(), "bend_1");
    }

    #[test]
    #[should_panic(expected = "part name must not be empty")]
    fn part_rejects_empty_name() {
        let _p = Part::new("", Solid::sphere(1.0), Material::new("PLA", 1250.0));
    }

    #[test]
    fn plane_normalizes_normal() {
        let p = Plane::new(Vector3::new(0.0, 0.0, 5.0), 1.0);
        let norm = p.normal().norm();
        assert!(
            (norm - 1.0).abs() < 1e-10,
            "normal should be unit length, got {norm}"
        );
        assert!((p.normal().z - 1.0).abs() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "plane normal must be non-zero")]
    fn plane_rejects_zero_normal() {
        let _p = Plane::new(Vector3::zeros(), 0.0);
    }

    #[test]
    #[should_panic(expected = "plane offset must be finite")]
    fn plane_rejects_nan_offset() {
        let _p = Plane::new(Vector3::z(), f64::NAN);
    }

    #[test]
    fn flex_zone_valid() {
        let fz = FlexZone::new(
            "hinge",
            Plane::new(Vector3::y(), 0.0),
            0.8,
            JointKind::Revolute,
            Vector3::x(),
        );
        assert_eq!(fz.name(), "hinge");
        assert!((fz.width() - 0.8).abs() < f64::EPSILON);
        assert_eq!(fz.kind(), JointKind::Revolute);
    }

    #[test]
    fn flex_zone_normalizes_axis() {
        let fz = FlexZone::new(
            "z",
            Plane::new(Vector3::y(), 0.0),
            1.0,
            JointKind::Prismatic,
            Vector3::new(3.0, 0.0, 0.0),
        );
        let norm = fz.axis().norm();
        assert!((norm - 1.0).abs() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "flex zone width must be positive")]
    fn flex_zone_rejects_zero_width() {
        let _fz = FlexZone::new(
            "bad",
            Plane::new(Vector3::z(), 0.0),
            0.0,
            JointKind::Revolute,
            Vector3::x(),
        );
    }

    #[test]
    #[should_panic(expected = "flex zone name must not be empty")]
    fn flex_zone_rejects_empty_name() {
        let _fz = FlexZone::new(
            "",
            Plane::new(Vector3::z(), 0.0),
            1.0,
            JointKind::Revolute,
            Vector3::x(),
        );
    }

    #[test]
    #[should_panic(expected = "flex zone axis must be non-zero")]
    fn flex_zone_rejects_zero_axis() {
        let _fz = FlexZone::new(
            "bad",
            Plane::new(Vector3::z(), 0.0),
            1.0,
            JointKind::Revolute,
            Vector3::zeros(),
        );
    }
}
