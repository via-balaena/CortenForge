//! Distance field representation.
//!
//! Stores per-vertex distance values computed by geodesic algorithms.

/// Per-vertex distance field.
///
/// Stores the computed geodesic distances from source vertices to all
/// other vertices in the mesh.
#[derive(Debug, Clone)]
pub struct DistanceField {
    /// Distance values for each vertex.
    /// `f64::INFINITY` indicates an unreachable vertex.
    distances: Vec<f64>,
}

impl DistanceField {
    /// Create a new distance field with all distances set to infinity.
    ///
    /// # Arguments
    ///
    /// * `vertex_count` - Number of vertices in the mesh
    #[must_use]
    pub fn new(vertex_count: usize) -> Self {
        Self {
            distances: vec![f64::INFINITY; vertex_count],
        }
    }

    /// Create a distance field from a vector of distances.
    #[must_use]
    pub const fn from_distances(distances: Vec<f64>) -> Self {
        Self { distances }
    }

    /// Get the distance to a vertex.
    ///
    /// Returns `f64::INFINITY` if the vertex is unreachable or the index is out of bounds.
    #[inline]
    #[must_use]
    pub fn distance(&self, vertex: usize) -> f64 {
        self.distances.get(vertex).copied().unwrap_or(f64::INFINITY)
    }

    /// Get a mutable reference to the distance of a vertex.
    ///
    /// Returns `None` if the index is out of bounds.
    #[inline]
    pub fn distance_mut(&mut self, vertex: usize) -> Option<&mut f64> {
        self.distances.get_mut(vertex)
    }

    /// Set the distance to a vertex.
    ///
    /// Returns `true` if successful, `false` if the index is out of bounds.
    #[inline]
    pub fn set_distance(&mut self, vertex: usize, distance: f64) -> bool {
        self.distances.get_mut(vertex).is_some_and(|d| {
            *d = distance;
            true
        })
    }

    /// Get the number of vertices.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.distances.len()
    }

    /// Check if the distance field is empty.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.distances.is_empty()
    }

    /// Get all distances as a slice.
    #[inline]
    #[must_use]
    pub fn as_slice(&self) -> &[f64] {
        &self.distances
    }

    /// Consume and return the underlying vector.
    #[must_use]
    pub fn into_vec(self) -> Vec<f64> {
        self.distances
    }

    /// Get the minimum distance (excluding zero/source vertices).
    #[must_use]
    pub fn min_distance(&self) -> f64 {
        self.distances
            .iter()
            .filter(|&&d| d > 0.0 && d.is_finite())
            .copied()
            .fold(f64::INFINITY, f64::min)
    }

    /// Get the maximum finite distance.
    #[must_use]
    pub fn max_distance(&self) -> f64 {
        self.distances
            .iter()
            .filter(|&&d| d.is_finite())
            .copied()
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Count the number of reachable vertices (finite distance).
    #[must_use]
    pub fn reachable_count(&self) -> usize {
        self.distances.iter().filter(|d| d.is_finite()).count()
    }

    /// Count the number of unreachable vertices (infinite distance).
    #[must_use]
    pub fn unreachable_count(&self) -> usize {
        self.distances.iter().filter(|d| !d.is_finite()).count()
    }

    /// Iterate over (vertex index, distance) pairs.
    pub fn iter(&self) -> impl Iterator<Item = (usize, f64)> + '_ {
        self.distances.iter().enumerate().map(|(i, &d)| (i, d))
    }

    /// Iterate over reachable vertices only.
    pub fn iter_reachable(&self) -> impl Iterator<Item = (usize, f64)> + '_ {
        self.iter().filter(|(_, d)| d.is_finite())
    }

    /// Find the vertex with the maximum finite distance.
    ///
    /// Returns `None` if no vertices are reachable.
    #[must_use]
    pub fn farthest_vertex(&self) -> Option<(usize, f64)> {
        self.iter_reachable()
            .max_by(|(_, d1), (_, d2)| d1.partial_cmp(d2).unwrap_or(std::cmp::Ordering::Equal))
    }

    /// Normalize distances to [0, 1] range based on max distance.
    ///
    /// Unreachable vertices remain at infinity.
    #[must_use]
    pub fn normalized(&self) -> Self {
        let max = self.max_distance();
        if max <= 0.0 || !max.is_finite() {
            return self.clone();
        }

        let distances = self
            .distances
            .iter()
            .map(|&d| if d.is_finite() { d / max } else { d })
            .collect();

        Self { distances }
    }
}

impl From<Vec<f64>> for DistanceField {
    fn from(distances: Vec<f64>) -> Self {
        Self::from_distances(distances)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_distance_field() {
        let df = DistanceField::new(5);
        assert_eq!(df.len(), 5);
        assert!(df.distance(0).is_infinite());
        assert!(df.distance(4).is_infinite());
    }

    #[test]
    fn set_and_get_distance() {
        let mut df = DistanceField::new(3);
        assert!(df.set_distance(0, 0.0));
        assert!(df.set_distance(1, 1.5));
        assert!(df.set_distance(2, 3.0));

        assert!((df.distance(0) - 0.0).abs() < f64::EPSILON);
        assert!((df.distance(1) - 1.5).abs() < f64::EPSILON);
        assert!((df.distance(2) - 3.0).abs() < f64::EPSILON);
    }

    #[test]
    fn out_of_bounds() {
        let df = DistanceField::new(3);
        assert!(df.distance(10).is_infinite());

        let mut df_mut = DistanceField::new(3);
        assert!(!df_mut.set_distance(10, 1.0));
    }

    #[test]
    fn min_max_distance() {
        let df = DistanceField::from_distances(vec![0.0, 1.0, 2.0, f64::INFINITY]);

        assert!((df.min_distance() - 1.0).abs() < f64::EPSILON);
        assert!((df.max_distance() - 2.0).abs() < f64::EPSILON);
    }

    #[test]
    fn reachable_count() {
        let df = DistanceField::from_distances(vec![0.0, 1.0, f64::INFINITY, 2.0]);

        assert_eq!(df.reachable_count(), 3);
        assert_eq!(df.unreachable_count(), 1);
    }

    #[test]
    fn farthest_vertex() {
        let df = DistanceField::from_distances(vec![0.0, 1.0, 5.0, 3.0]);

        let farthest = df.farthest_vertex();
        assert!(farthest.is_some());
        let (idx, dist) = farthest.unwrap_or((0, 0.0));
        assert_eq!(idx, 2);
        assert!((dist - 5.0).abs() < f64::EPSILON);
    }

    #[test]
    fn normalized() {
        let df = DistanceField::from_distances(vec![0.0, 5.0, 10.0, f64::INFINITY]);
        let norm = df.normalized();

        assert!((norm.distance(0) - 0.0).abs() < f64::EPSILON);
        assert!((norm.distance(1) - 0.5).abs() < f64::EPSILON);
        assert!((norm.distance(2) - 1.0).abs() < f64::EPSILON);
        assert!(norm.distance(3).is_infinite());
    }

    #[test]
    fn iter_reachable() {
        let df = DistanceField::from_distances(vec![0.0, 1.0, f64::INFINITY, 2.0]);

        let reachable: Vec<_> = df.iter_reachable().collect();
        assert_eq!(reachable.len(), 3);
        assert!(reachable.iter().all(|(_, d)| d.is_finite()));
    }
}
