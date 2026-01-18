//! Beam lattice data for 3MF export.
//!
//! This module provides types for representing lattice structures as
//! beam networks, compatible with the 3MF Beam Lattice Extension.

use nalgebra::Point3;

/// Cap style for beam endpoints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum BeamCap {
    /// Spherical cap (smooth blend at intersections).
    #[default]
    Sphere,
    /// Flat cap (perpendicular to beam axis).
    Flat,
    /// No cap (butt end, open).
    Butt,
}

/// A single beam (strut) connecting two vertices.
#[derive(Debug, Clone)]
pub struct Beam {
    /// Index of the first vertex.
    pub v1: u32,
    /// Index of the second vertex.
    pub v2: u32,
    /// Radius at the first vertex (mm).
    pub r1: f64,
    /// Radius at the second vertex (mm).
    pub r2: f64,
    /// Cap style at the first vertex.
    pub cap1: BeamCap,
    /// Cap style at the second vertex.
    pub cap2: BeamCap,
}

impl Beam {
    /// Creates a new beam with uniform radius.
    ///
    /// # Arguments
    ///
    /// * `v1` - First vertex index
    /// * `v2` - Second vertex index
    /// * `radius` - Beam radius in mm
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::Beam;
    ///
    /// let beam = Beam::new(0, 1, 0.5);
    /// assert_eq!(beam.v1, 0);
    /// assert_eq!(beam.v2, 1);
    /// ```
    #[must_use]
    pub fn new(v1: u32, v2: u32, radius: f64) -> Self {
        Self {
            v1,
            v2,
            r1: radius,
            r2: radius,
            cap1: BeamCap::Sphere,
            cap2: BeamCap::Sphere,
        }
    }

    /// Creates a tapered beam with different radii at each end.
    ///
    /// # Arguments
    ///
    /// * `v1` - First vertex index
    /// * `v2` - Second vertex index
    /// * `r1` - Radius at first vertex (mm)
    /// * `r2` - Radius at second vertex (mm)
    #[must_use]
    pub fn tapered(v1: u32, v2: u32, r1: f64, r2: f64) -> Self {
        Self {
            v1,
            v2,
            r1,
            r2,
            cap1: BeamCap::Sphere,
            cap2: BeamCap::Sphere,
        }
    }

    /// Sets the cap style at both ends.
    #[must_use]
    pub fn with_caps(mut self, cap: BeamCap) -> Self {
        self.cap1 = cap;
        self.cap2 = cap;
        self
    }

    /// Returns the length of this beam given vertex positions.
    #[must_use]
    pub fn length(&self, vertices: &[Point3<f64>]) -> Option<f64> {
        let v1 = vertices.get(self.v1 as usize)?;
        let v2 = vertices.get(self.v2 as usize)?;
        Some((v2 - v1).norm())
    }

    /// Returns the average radius of this beam.
    #[must_use]
    pub fn average_radius(&self) -> f64 {
        (self.r1 + self.r2) / 2.0
    }
}

/// A set of beams with shared properties.
#[derive(Debug, Clone, Default)]
pub struct BeamSet {
    /// Name of this beam set.
    pub name: String,
    /// Indices of beams in this set.
    pub beam_indices: Vec<u32>,
}

impl BeamSet {
    /// Creates a new empty beam set.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::BeamSet;
    ///
    /// let set = BeamSet::new("primary_struts");
    /// assert_eq!(set.name, "primary_struts");
    /// ```
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            beam_indices: Vec::new(),
        }
    }

    /// Adds a beam index to this set.
    pub fn add_beam(&mut self, index: u32) {
        self.beam_indices.push(index);
    }

    /// Returns the number of beams in this set.
    #[must_use]
    pub fn len(&self) -> usize {
        self.beam_indices.len()
    }

    /// Returns true if this set is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.beam_indices.is_empty()
    }
}

/// Complete beam lattice data for 3MF export.
///
/// This structure contains all the information needed to export a lattice
/// using the 3MF Beam Lattice Extension, which is more efficient than
/// triangulated geometry for representing strut-based structures.
#[derive(Debug, Clone, Default)]
pub struct BeamLatticeData {
    /// Node (vertex) positions.
    pub vertices: Vec<Point3<f64>>,
    /// Beam (strut) definitions.
    pub beams: Vec<Beam>,
    /// Optional beam groupings.
    pub beam_sets: Vec<BeamSet>,
    /// Default beam radius in mm.
    pub default_radius: f64,
    /// Minimum beam length in mm.
    pub min_length: f64,
    /// Default cap style.
    pub default_cap: BeamCap,
}

impl BeamLatticeData {
    /// Creates a new empty beam lattice.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::BeamLatticeData;
    ///
    /// let data = BeamLatticeData::new(0.5);
    /// assert_eq!(data.default_radius, 0.5);
    /// ```
    #[must_use]
    pub fn new(default_radius: f64) -> Self {
        Self {
            vertices: Vec::new(),
            beams: Vec::new(),
            beam_sets: Vec::new(),
            default_radius,
            min_length: 0.1,
            default_cap: BeamCap::Sphere,
        }
    }

    /// Adds a vertex and returns its index.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::BeamLatticeData;
    /// use nalgebra::Point3;
    ///
    /// let mut data = BeamLatticeData::new(0.5);
    /// let idx = data.add_vertex(Point3::new(0.0, 0.0, 0.0));
    /// assert_eq!(idx, 0);
    /// ```
    pub fn add_vertex(&mut self, position: Point3<f64>) -> u32 {
        let idx = self.vertices.len() as u32;
        self.vertices.push(position);
        idx
    }

    /// Adds a beam with the default radius.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::BeamLatticeData;
    /// use nalgebra::Point3;
    ///
    /// let mut data = BeamLatticeData::new(0.5);
    /// let v1 = data.add_vertex(Point3::new(0.0, 0.0, 0.0));
    /// let v2 = data.add_vertex(Point3::new(1.0, 0.0, 0.0));
    /// data.add_beam(v1, v2);
    /// assert_eq!(data.beams.len(), 1);
    /// ```
    pub fn add_beam(&mut self, v1: u32, v2: u32) {
        self.beams.push(Beam::new(v1, v2, self.default_radius));
    }

    /// Adds a beam with custom radius.
    pub fn add_beam_with_radius(&mut self, v1: u32, v2: u32, radius: f64) {
        self.beams.push(Beam::new(v1, v2, radius));
    }

    /// Adds a tapered beam.
    pub fn add_tapered_beam(&mut self, v1: u32, v2: u32, r1: f64, r2: f64) {
        self.beams.push(Beam::tapered(v1, v2, r1, r2));
    }

    /// Returns the total length of all beams.
    #[must_use]
    pub fn total_length(&self) -> f64 {
        self.beams
            .iter()
            .filter_map(|b| b.length(&self.vertices))
            .sum()
    }

    /// Returns the number of vertices.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Returns the number of beams.
    #[must_use]
    pub fn beam_count(&self) -> usize {
        self.beams.len()
    }

    /// Estimates the volume of the beam lattice.
    ///
    /// This is an approximation that treats each beam as a cylinder
    /// (or truncated cone for tapered beams).
    #[must_use]
    pub fn estimate_volume(&self) -> f64 {
        use std::f64::consts::PI;

        self.beams
            .iter()
            .filter_map(|beam| {
                let length = beam.length(&self.vertices)?;
                // Volume of truncated cone: (π/3) * h * (r1² + r1*r2 + r2²)
                let volume =
                    (PI / 3.0) * length * (beam.r1.powi(2) + beam.r1 * beam.r2 + beam.r2.powi(2));
                Some(volume)
            })
            .sum()
    }

    /// Removes beams shorter than `min_length`.
    pub fn remove_short_beams(&mut self, min_length: f64) {
        self.beams.retain(|beam| {
            beam.length(&self.vertices)
                .map_or(false, |len| len >= min_length)
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_beam_new() {
        let beam = Beam::new(0, 1, 0.5);
        assert_eq!(beam.v1, 0);
        assert_eq!(beam.v2, 1);
        assert!((beam.r1 - 0.5).abs() < f64::EPSILON);
        assert!((beam.r2 - 0.5).abs() < f64::EPSILON);
        assert_eq!(beam.cap1, BeamCap::Sphere);
        assert_eq!(beam.cap2, BeamCap::Sphere);
    }

    #[test]
    fn test_beam_tapered() {
        let beam = Beam::tapered(0, 1, 0.3, 0.6);
        assert!((beam.r1 - 0.3).abs() < f64::EPSILON);
        assert!((beam.r2 - 0.6).abs() < f64::EPSILON);
    }

    #[test]
    fn test_beam_length() {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let beam = Beam::new(0, 1, 0.5);
        assert!((beam.length(&vertices).unwrap_or(0.0) - 1.0).abs() < f64::EPSILON);

        let beam = Beam::new(0, 2, 0.5);
        assert!((beam.length(&vertices).unwrap_or(0.0) - 1.0).abs() < f64::EPSILON);

        // Invalid index
        let beam = Beam::new(0, 10, 0.5);
        assert!(beam.length(&vertices).is_none());
    }

    #[test]
    fn test_beam_average_radius() {
        let beam = Beam::tapered(0, 1, 0.4, 0.6);
        assert!((beam.average_radius() - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn test_beam_set() {
        let mut set = BeamSet::new("test_set");
        assert_eq!(set.name, "test_set");
        assert!(set.is_empty());

        set.add_beam(0);
        set.add_beam(1);
        assert_eq!(set.len(), 2);
        assert!(!set.is_empty());
    }

    #[test]
    fn test_beam_lattice_data_new() {
        let data = BeamLatticeData::new(0.5);
        assert!(data.vertices.is_empty());
        assert!(data.beams.is_empty());
        assert!((data.default_radius - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn test_beam_lattice_add_vertex() {
        let mut data = BeamLatticeData::new(0.5);
        let v1 = data.add_vertex(Point3::new(0.0, 0.0, 0.0));
        let v2 = data.add_vertex(Point3::new(1.0, 0.0, 0.0));
        assert_eq!(v1, 0);
        assert_eq!(v2, 1);
        assert_eq!(data.vertex_count(), 2);
    }

    #[test]
    fn test_beam_lattice_add_beam() {
        let mut data = BeamLatticeData::new(0.5);
        let v1 = data.add_vertex(Point3::new(0.0, 0.0, 0.0));
        let v2 = data.add_vertex(Point3::new(1.0, 0.0, 0.0));
        data.add_beam(v1, v2);
        assert_eq!(data.beam_count(), 1);
        assert!((data.beams[0].r1 - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn test_beam_lattice_total_length() {
        let mut data = BeamLatticeData::new(0.5);
        let v1 = data.add_vertex(Point3::new(0.0, 0.0, 0.0));
        let v2 = data.add_vertex(Point3::new(1.0, 0.0, 0.0));
        let v3 = data.add_vertex(Point3::new(1.0, 1.0, 0.0));
        data.add_beam(v1, v2);
        data.add_beam(v2, v3);
        assert!((data.total_length() - 2.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_beam_lattice_estimate_volume() {
        use std::f64::consts::PI;

        let mut data = BeamLatticeData::new(0.5);
        let v1 = data.add_vertex(Point3::new(0.0, 0.0, 0.0));
        let v2 = data.add_vertex(Point3::new(1.0, 0.0, 0.0));
        data.add_beam(v1, v2);

        // Cylinder volume = π * r² * h = π * 0.25 * 1 = 0.785...
        let expected = PI * 0.25 * 1.0;
        assert!((data.estimate_volume() - expected).abs() < 0.001);
    }

    #[test]
    fn test_remove_short_beams() {
        let mut data = BeamLatticeData::new(0.5);
        let v1 = data.add_vertex(Point3::new(0.0, 0.0, 0.0));
        let v2 = data.add_vertex(Point3::new(0.05, 0.0, 0.0)); // Short
        let v3 = data.add_vertex(Point3::new(1.0, 0.0, 0.0)); // Long
        data.add_beam(v1, v2);
        data.add_beam(v1, v3);
        assert_eq!(data.beam_count(), 2);

        data.remove_short_beams(0.1);
        assert_eq!(data.beam_count(), 1);
    }

    #[test]
    fn test_beam_cap_default() {
        assert_eq!(BeamCap::default(), BeamCap::Sphere);
    }
}
