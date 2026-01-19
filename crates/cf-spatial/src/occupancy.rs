//! Occupancy map for probabilistic spatial representation.
//!
//! An occupancy map stores probability values indicating whether each voxel
//! is occupied. This is commonly used in robotics for SLAM (Simultaneous
//! Localization and Mapping) and sensor fusion.

use std::collections::HashMap;

use nalgebra::Point3;

use crate::error::SpatialError;
use crate::grid::GridBounds;
use crate::voxel::VoxelCoord;

/// Default log-odds value for unknown cells.
const DEFAULT_LOG_ODDS: f32 = 0.0;

/// Default log-odds increment for occupied observations.
const DEFAULT_LOG_ODDS_HIT: f32 = 0.85;

/// Default log-odds decrement for free observations.
const DEFAULT_LOG_ODDS_MISS: f32 = -0.4;

/// Minimum log-odds value (clamped).
const MIN_LOG_ODDS: f32 = -2.0;

/// Maximum log-odds value (clamped).
const MAX_LOG_ODDS: f32 = 3.5;

/// A probabilistic occupancy map using log-odds representation.
///
/// The map stores occupancy probabilities in log-odds form for efficient
/// Bayesian updates. Log-odds representation allows simple addition for
/// sensor fusion and avoids numerical issues with probabilities near 0 or 1.
///
/// # Log-Odds Representation
///
/// Log-odds `l` relates to probability `p` by:
/// - `l = log(p / (1 - p))`
/// - `p = 1 / (1 + exp(-l))`
///
/// | Log-odds | Probability |
/// |----------|-------------|
/// | -2.0     | ~12%        |
/// | -1.0     | ~27%        |
/// | 0.0      | 50%         |
/// | 1.0      | ~73%        |
/// | 2.0      | ~88%        |
/// | 3.5      | ~97%        |
///
/// # Example
///
/// ```
/// use cf_spatial::{OccupancyMap, VoxelCoord};
///
/// let mut map = OccupancyMap::new(0.1);
///
/// // Mark a cell as occupied
/// let coord = VoxelCoord::new(5, 5, 5);
/// map.update_occupied(coord);
///
/// // Check if it's likely occupied
/// assert!(map.is_occupied(coord));
/// assert!(map.probability(coord) > 0.5);
/// ```
#[derive(Debug, Clone)]
pub struct OccupancyMap {
    /// Size of each voxel in world units.
    voxel_size: f64,
    /// Inverse of voxel size for faster coordinate conversion.
    inv_voxel_size: f64,
    /// Origin offset in world space.
    origin: Point3<f64>,
    /// Sparse storage of log-odds values.
    data: HashMap<VoxelCoord, f32>,
    /// Log-odds increment for hit (occupied observation).
    log_odds_hit: f32,
    /// Log-odds decrement for miss (free observation).
    log_odds_miss: f32,
    /// Threshold log-odds for considering a cell occupied.
    occupied_threshold: f32,
    /// Threshold log-odds for considering a cell free.
    free_threshold: f32,
}

impl OccupancyMap {
    /// Creates a new empty occupancy map with the specified voxel size.
    ///
    /// Uses default log-odds parameters suitable for typical sensor fusion.
    ///
    /// # Arguments
    ///
    /// * `voxel_size` - The size of each voxel in world units. Must be positive.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::OccupancyMap;
    ///
    /// let map = OccupancyMap::new(0.1);
    /// assert_eq!(map.voxel_size(), 0.1);
    /// assert!(map.is_empty());
    /// ```
    #[must_use]
    pub fn new(voxel_size: f64) -> Self {
        Self::with_origin(voxel_size, Point3::origin())
    }

    /// Creates a new occupancy map with the specified voxel size and origin.
    ///
    /// # Arguments
    ///
    /// * `voxel_size` - The size of each voxel in world units. Must be positive.
    /// * `origin` - The world-space position of grid coordinate (0, 0, 0).
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::OccupancyMap;
    /// use nalgebra::Point3;
    ///
    /// let origin = Point3::new(10.0, 20.0, 30.0);
    /// let map = OccupancyMap::with_origin(0.1, origin);
    /// assert_eq!(map.origin(), &origin);
    /// ```
    #[must_use]
    pub fn with_origin(voxel_size: f64, origin: Point3<f64>) -> Self {
        let voxel_size = voxel_size.abs().max(f64::EPSILON);
        Self {
            voxel_size,
            inv_voxel_size: 1.0 / voxel_size,
            origin,
            data: HashMap::new(),
            log_odds_hit: DEFAULT_LOG_ODDS_HIT,
            log_odds_miss: DEFAULT_LOG_ODDS_MISS,
            occupied_threshold: 0.0, // 50% probability
            free_threshold: -0.4,    // ~40% probability
        }
    }

    /// Attempts to create a new occupancy map, returning an error if the voxel size is invalid.
    ///
    /// # Errors
    ///
    /// Returns [`SpatialError::InvalidVoxelSize`] if `voxel_size` is not positive or finite.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{OccupancyMap, SpatialError};
    ///
    /// let map = OccupancyMap::try_new(0.1);
    /// assert!(map.is_ok());
    ///
    /// let invalid = OccupancyMap::try_new(-1.0);
    /// assert!(matches!(invalid, Err(SpatialError::InvalidVoxelSize(_))));
    /// ```
    pub fn try_new(voxel_size: f64) -> Result<Self, SpatialError> {
        if voxel_size <= 0.0 || !voxel_size.is_finite() {
            return Err(SpatialError::InvalidVoxelSize(voxel_size));
        }
        Ok(Self::new(voxel_size))
    }

    /// Returns the voxel size.
    #[must_use]
    pub const fn voxel_size(&self) -> f64 {
        self.voxel_size
    }

    /// Returns the grid origin in world space.
    #[must_use]
    pub const fn origin(&self) -> &Point3<f64> {
        &self.origin
    }

    /// Returns the number of cells with recorded observations.
    #[must_use]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Returns `true` if the map has no recorded observations.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Sets the log-odds parameters for sensor updates.
    ///
    /// # Arguments
    ///
    /// * `hit` - Log-odds increment for occupied observations (positive)
    /// * `miss` - Log-odds decrement for free observations (negative)
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::OccupancyMap;
    ///
    /// let mut map = OccupancyMap::new(0.1);
    /// map.set_update_parameters(0.9, -0.3);
    /// ```
    pub const fn set_update_parameters(&mut self, hit: f32, miss: f32) {
        self.log_odds_hit = hit;
        self.log_odds_miss = miss;
    }

    /// Sets the occupancy thresholds.
    ///
    /// # Arguments
    ///
    /// * `occupied` - Log-odds threshold above which a cell is considered occupied
    /// * `free` - Log-odds threshold below which a cell is considered free
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::OccupancyMap;
    ///
    /// let mut map = OccupancyMap::new(0.1);
    /// map.set_thresholds(0.5, -0.5);
    /// ```
    pub const fn set_thresholds(&mut self, occupied: f32, free: f32) {
        self.occupied_threshold = occupied;
        self.free_threshold = free;
    }

    /// Converts a world-space point to a grid coordinate.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn world_to_grid(&self, point: Point3<f64>) -> VoxelCoord {
        let relative = point - self.origin;
        VoxelCoord::new(
            (relative.x * self.inv_voxel_size).floor() as i32,
            (relative.y * self.inv_voxel_size).floor() as i32,
            (relative.z * self.inv_voxel_size).floor() as i32,
        )
    }

    /// Converts a grid coordinate to the world-space center of that voxel.
    #[must_use]
    pub fn grid_to_world_center(&self, coord: VoxelCoord) -> Point3<f64> {
        let half = self.voxel_size * 0.5;
        Point3::new(
            f64::from(coord.x).mul_add(self.voxel_size, self.origin.x) + half,
            f64::from(coord.y).mul_add(self.voxel_size, self.origin.y) + half,
            f64::from(coord.z).mul_add(self.voxel_size, self.origin.z) + half,
        )
    }

    /// Gets the log-odds value at a grid coordinate.
    ///
    /// Returns `DEFAULT_LOG_ODDS` (0.0, 50% probability) for unobserved cells.
    #[must_use]
    pub fn log_odds(&self, coord: VoxelCoord) -> f32 {
        self.data.get(&coord).copied().unwrap_or(DEFAULT_LOG_ODDS)
    }

    /// Gets the occupancy probability at a grid coordinate.
    ///
    /// Returns 0.5 for unobserved cells.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{OccupancyMap, VoxelCoord};
    ///
    /// let mut map = OccupancyMap::new(0.1);
    /// let coord = VoxelCoord::new(0, 0, 0);
    ///
    /// // Unobserved cell has 50% probability
    /// assert!((map.probability(coord) - 0.5).abs() < 0.01);
    ///
    /// // After marking occupied, probability increases
    /// map.update_occupied(coord);
    /// assert!(map.probability(coord) > 0.5);
    /// ```
    #[must_use]
    pub fn probability(&self, coord: VoxelCoord) -> f32 {
        log_odds_to_probability(self.log_odds(coord))
    }

    /// Checks if a cell is considered occupied.
    ///
    /// A cell is occupied if its log-odds exceeds the occupied threshold.
    #[must_use]
    pub fn is_occupied(&self, coord: VoxelCoord) -> bool {
        self.log_odds(coord) > self.occupied_threshold
    }

    /// Checks if a cell is considered free (unoccupied).
    ///
    /// A cell is free if its log-odds is below the free threshold.
    #[must_use]
    pub fn is_free(&self, coord: VoxelCoord) -> bool {
        self.log_odds(coord) < self.free_threshold
    }

    /// Checks if a cell is unknown (between free and occupied thresholds).
    #[must_use]
    pub fn is_unknown(&self, coord: VoxelCoord) -> bool {
        let l = self.log_odds(coord);
        l >= self.free_threshold && l <= self.occupied_threshold
    }

    /// Updates a cell with an occupied observation.
    ///
    /// Increments the log-odds by the hit parameter.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{OccupancyMap, VoxelCoord};
    ///
    /// let mut map = OccupancyMap::new(0.1);
    /// let coord = VoxelCoord::new(5, 5, 5);
    ///
    /// // Multiple observations increase confidence
    /// map.update_occupied(coord);
    /// let p1 = map.probability(coord);
    /// map.update_occupied(coord);
    /// let p2 = map.probability(coord);
    /// assert!(p2 > p1);
    /// ```
    pub fn update_occupied(&mut self, coord: VoxelCoord) {
        let current = self.data.get(&coord).copied().unwrap_or(DEFAULT_LOG_ODDS);
        let new_value = (current + self.log_odds_hit).clamp(MIN_LOG_ODDS, MAX_LOG_ODDS);
        self.data.insert(coord, new_value);
    }

    /// Updates a cell with a free (miss) observation.
    ///
    /// Decrements the log-odds by the miss parameter.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{OccupancyMap, VoxelCoord};
    ///
    /// let mut map = OccupancyMap::new(0.1);
    /// let coord = VoxelCoord::new(5, 5, 5);
    ///
    /// // First mark occupied
    /// map.update_occupied(coord);
    /// assert!(map.is_occupied(coord));
    ///
    /// // Then observe as free multiple times
    /// for _ in 0..5 {
    ///     map.update_free(coord);
    /// }
    /// assert!(map.is_free(coord));
    /// ```
    pub fn update_free(&mut self, coord: VoxelCoord) {
        let current = self.data.get(&coord).copied().unwrap_or(DEFAULT_LOG_ODDS);
        let new_value = (current + self.log_odds_miss).clamp(MIN_LOG_ODDS, MAX_LOG_ODDS);
        self.data.insert(coord, new_value);
    }

    /// Updates a cell at a world-space point with an occupied observation.
    pub fn update_occupied_at_world(&mut self, point: Point3<f64>) {
        self.update_occupied(self.world_to_grid(point));
    }

    /// Updates a cell at a world-space point with a free observation.
    pub fn update_free_at_world(&mut self, point: Point3<f64>) {
        self.update_free(self.world_to_grid(point));
    }

    /// Sets the log-odds value directly for a cell.
    ///
    /// The value is clamped to the valid range.
    pub fn set_log_odds(&mut self, coord: VoxelCoord, log_odds: f32) {
        let clamped = log_odds.clamp(MIN_LOG_ODDS, MAX_LOG_ODDS);
        self.data.insert(coord, clamped);
    }

    /// Sets the probability directly for a cell.
    ///
    /// The probability is converted to log-odds and clamped.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{OccupancyMap, VoxelCoord};
    ///
    /// let mut map = OccupancyMap::new(0.1);
    /// let coord = VoxelCoord::new(0, 0, 0);
    ///
    /// map.set_probability(coord, 0.9);
    /// assert!((map.probability(coord) - 0.9).abs() < 0.01);
    /// ```
    pub fn set_probability(&mut self, coord: VoxelCoord, probability: f32) {
        let log_odds = probability_to_log_odds(probability);
        self.set_log_odds(coord, log_odds);
    }

    /// Removes a cell from the map, resetting it to unknown.
    pub fn reset(&mut self, coord: VoxelCoord) {
        self.data.remove(&coord);
    }

    /// Clears all observations from the map.
    pub fn clear(&mut self) {
        self.data.clear();
    }

    /// Returns an iterator over all observed cells and their log-odds values.
    pub fn iter(&self) -> impl Iterator<Item = (&VoxelCoord, &f32)> {
        self.data.iter()
    }

    /// Returns an iterator over all observed cell coordinates.
    pub fn coords(&self) -> impl Iterator<Item = &VoxelCoord> {
        self.data.keys()
    }

    /// Returns an iterator over all occupied cells.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{OccupancyMap, VoxelCoord};
    ///
    /// let mut map = OccupancyMap::new(0.1);
    /// map.update_occupied(VoxelCoord::new(0, 0, 0));
    /// map.update_occupied(VoxelCoord::new(1, 1, 1));
    /// map.update_free(VoxelCoord::new(2, 2, 2));
    ///
    /// let occupied: Vec<_> = map.occupied_cells().collect();
    /// assert_eq!(occupied.len(), 2);
    /// ```
    pub fn occupied_cells(&self) -> impl Iterator<Item = &VoxelCoord> {
        self.data
            .iter()
            .filter(|&(_, log_odds)| *log_odds > self.occupied_threshold)
            .map(|(coord, _)| coord)
    }

    /// Returns an iterator over all free cells.
    pub fn free_cells(&self) -> impl Iterator<Item = &VoxelCoord> {
        self.data
            .iter()
            .filter(|&(_, log_odds)| *log_odds < self.free_threshold)
            .map(|(coord, _)| coord)
    }

    /// Computes the bounding box of all observed cells.
    ///
    /// Returns `None` if the map is empty.
    #[must_use]
    pub fn bounds(&self) -> Option<GridBounds> {
        let mut iter = self.data.keys();
        let first = *iter.next()?;

        let mut bounds = GridBounds::from_point(first);
        for coord in iter {
            bounds.expand_to_include(*coord);
        }

        Some(bounds)
    }

    /// Computes the bounding box of all occupied cells.
    ///
    /// Returns `None` if there are no occupied cells.
    #[must_use]
    pub fn occupied_bounds(&self) -> Option<GridBounds> {
        let mut iter = self.occupied_cells();
        let first = *iter.next()?;

        let mut bounds = GridBounds::from_point(first);
        for coord in iter {
            bounds.expand_to_include(*coord);
        }

        Some(bounds)
    }
}

impl Default for OccupancyMap {
    fn default() -> Self {
        Self::new(1.0)
    }
}

/// Converts log-odds to probability.
///
/// `p = 1 / (1 + exp(-l))`
#[inline]
#[must_use]
pub fn log_odds_to_probability(log_odds: f32) -> f32 {
    1.0 / (1.0 + (-log_odds).exp())
}

/// Converts probability to log-odds.
///
/// `l = log(p / (1 - p))`
///
/// Clamps probability to avoid infinity.
#[inline]
#[must_use]
pub fn probability_to_log_odds(probability: f32) -> f32 {
    let p = probability.clamp(0.001, 0.999);
    (p / (1.0 - p)).ln()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::needless_collect)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let map = OccupancyMap::new(0.1);
        assert!((map.voxel_size() - 0.1).abs() < f64::EPSILON);
        assert!(map.is_empty());
    }

    #[test]
    fn test_with_origin() {
        let origin = Point3::new(10.0, 20.0, 30.0);
        let map = OccupancyMap::with_origin(0.1, origin);
        assert_eq!(map.origin(), &origin);
    }

    #[test]
    fn test_try_new() {
        assert!(OccupancyMap::try_new(0.1).is_ok());
        assert!(matches!(
            OccupancyMap::try_new(-1.0),
            Err(SpatialError::InvalidVoxelSize(_))
        ));
        assert!(matches!(
            OccupancyMap::try_new(0.0),
            Err(SpatialError::InvalidVoxelSize(_))
        ));
    }

    #[test]
    fn test_default_probability() {
        let map = OccupancyMap::new(0.1);
        let coord = VoxelCoord::new(0, 0, 0);
        assert!((map.probability(coord) - 0.5).abs() < 0.01);
        assert!((map.log_odds(coord) - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_update_occupied() {
        let mut map = OccupancyMap::new(0.1);
        let coord = VoxelCoord::new(5, 5, 5);

        map.update_occupied(coord);
        assert!(map.probability(coord) > 0.5);
        assert!(map.is_occupied(coord));
    }

    #[test]
    fn test_update_free() {
        let mut map = OccupancyMap::new(0.1);
        let coord = VoxelCoord::new(5, 5, 5);

        // First mark occupied
        map.update_occupied(coord);
        map.update_occupied(coord);

        // Then mark free multiple times
        for _ in 0..10 {
            map.update_free(coord);
        }

        assert!(map.is_free(coord));
    }

    #[test]
    fn test_multiple_observations() {
        let mut map = OccupancyMap::new(0.1);
        let coord = VoxelCoord::new(0, 0, 0);

        // Multiple occupied observations should increase probability
        let p0 = map.probability(coord);
        map.update_occupied(coord);
        let p1 = map.probability(coord);
        map.update_occupied(coord);
        let p2 = map.probability(coord);

        assert!(p1 > p0);
        assert!(p2 > p1);
    }

    #[test]
    fn test_probability_clamping() {
        let mut map = OccupancyMap::new(0.1);
        let coord = VoxelCoord::new(0, 0, 0);

        // Many occupied observations
        for _ in 0..100 {
            map.update_occupied(coord);
        }

        // Should be clamped to max
        let log_odds = map.log_odds(coord);
        assert!((log_odds - MAX_LOG_ODDS).abs() < 0.01);

        // Reset and do many free observations
        map.reset(coord);
        for _ in 0..100 {
            map.update_free(coord);
        }

        // Should be clamped to min
        let log_odds = map.log_odds(coord);
        assert!((log_odds - MIN_LOG_ODDS).abs() < 0.01);
    }

    #[test]
    fn test_set_probability() {
        let mut map = OccupancyMap::new(0.1);
        let coord = VoxelCoord::new(0, 0, 0);

        map.set_probability(coord, 0.9);
        assert!((map.probability(coord) - 0.9).abs() < 0.02);

        map.set_probability(coord, 0.1);
        assert!((map.probability(coord) - 0.1).abs() < 0.02);
    }

    #[test]
    fn test_set_log_odds() {
        let mut map = OccupancyMap::new(0.1);
        let coord = VoxelCoord::new(0, 0, 0);

        map.set_log_odds(coord, 1.0);
        assert!((map.log_odds(coord) - 1.0).abs() < 0.01);

        // Test clamping
        map.set_log_odds(coord, 100.0);
        assert!((map.log_odds(coord) - MAX_LOG_ODDS).abs() < 0.01);
    }

    #[test]
    fn test_reset() {
        let mut map = OccupancyMap::new(0.1);
        let coord = VoxelCoord::new(0, 0, 0);

        map.update_occupied(coord);
        assert!(!map.is_empty());

        map.reset(coord);
        assert!(map.is_empty());
        assert!((map.probability(coord) - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_clear() {
        let mut map = OccupancyMap::new(0.1);

        map.update_occupied(VoxelCoord::new(0, 0, 0));
        map.update_occupied(VoxelCoord::new(1, 1, 1));
        assert_eq!(map.len(), 2);

        map.clear();
        assert!(map.is_empty());
    }

    #[test]
    fn test_occupied_cells() {
        let mut map = OccupancyMap::new(0.1);

        map.update_occupied(VoxelCoord::new(0, 0, 0));
        map.update_occupied(VoxelCoord::new(1, 1, 1));
        map.update_free(VoxelCoord::new(2, 2, 2));
        map.update_free(VoxelCoord::new(2, 2, 2));

        let occupied: Vec<_> = map.occupied_cells().collect();
        assert_eq!(occupied.len(), 2);
    }

    #[test]
    fn test_free_cells() {
        let mut map = OccupancyMap::new(0.1);

        map.update_free(VoxelCoord::new(0, 0, 0));
        map.update_free(VoxelCoord::new(0, 0, 0));
        map.update_occupied(VoxelCoord::new(1, 1, 1));

        let free: Vec<_> = map.free_cells().collect();
        assert_eq!(free.len(), 1);
    }

    #[test]
    fn test_bounds() {
        let mut map = OccupancyMap::new(0.1);
        assert!(map.bounds().is_none());

        map.update_occupied(VoxelCoord::new(0, 0, 0));
        map.update_occupied(VoxelCoord::new(10, 20, 30));

        let bounds = map.bounds().unwrap();
        assert_eq!(bounds.min, VoxelCoord::new(0, 0, 0));
        assert_eq!(bounds.max, VoxelCoord::new(10, 20, 30));
    }

    #[test]
    fn test_world_to_grid() {
        let map = OccupancyMap::new(0.1);

        let coord = map.world_to_grid(Point3::new(0.15, 0.25, 0.35));
        assert_eq!(coord, VoxelCoord::new(1, 2, 3));
    }

    #[test]
    fn test_grid_to_world_center() {
        let map = OccupancyMap::new(0.1);

        let center = map.grid_to_world_center(VoxelCoord::new(0, 0, 0));
        assert!((center.x - 0.05).abs() < 1e-10);
        assert!((center.y - 0.05).abs() < 1e-10);
        assert!((center.z - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_log_odds_probability_conversion() {
        // log-odds 0 = probability 0.5
        assert!((log_odds_to_probability(0.0) - 0.5).abs() < 0.001);

        // Roundtrip
        for p in [0.1, 0.25, 0.5, 0.75, 0.9] {
            let log_odds = probability_to_log_odds(p);
            let back = log_odds_to_probability(log_odds);
            assert!((back - p).abs() < 0.01);
        }
    }

    #[test]
    fn test_is_unknown() {
        let mut map = OccupancyMap::new(0.1);
        let coord = VoxelCoord::new(0, 0, 0);

        // Unobserved cell is unknown (log-odds = 0, threshold default is 0 for occupied)
        // With default thresholds: free < -0.4, occupied > 0.0
        // So 0.0 is right at the boundary

        // Multiple free observations should make it below free threshold
        // (single observation brings to -0.4, which is not strictly less than -0.4)
        map.update_free(coord);
        map.update_free(coord);
        assert!(map.is_free(coord));

        // Reset and mark occupied
        map.reset(coord);
        map.update_occupied(coord);
        assert!(map.is_occupied(coord));
    }

    #[test]
    fn test_set_parameters() {
        let mut map = OccupancyMap::new(0.1);
        map.set_update_parameters(1.0, -0.5);
        map.set_thresholds(0.5, -0.5);

        let coord = VoxelCoord::new(0, 0, 0);

        // With higher hit value, should become occupied faster
        map.update_occupied(coord);
        assert!(map.log_odds(coord) > 0.9);
    }

    #[test]
    fn test_iter() {
        let mut map = OccupancyMap::new(0.1);

        map.update_occupied(VoxelCoord::new(0, 0, 0));
        map.update_free(VoxelCoord::new(1, 1, 1));

        let entries: Vec<_> = map.iter().collect();
        assert_eq!(entries.len(), 2);
    }

    #[test]
    fn test_coords() {
        let mut map = OccupancyMap::new(0.1);

        map.update_occupied(VoxelCoord::new(0, 0, 0));
        map.update_free(VoxelCoord::new(1, 1, 1));

        let coords: Vec<_> = map.coords().collect();
        assert_eq!(coords.len(), 2);
    }

    #[test]
    fn test_default() {
        let map = OccupancyMap::default();
        assert!((map.voxel_size() - 1.0).abs() < f64::EPSILON);
        assert!(map.is_empty());
    }
}
