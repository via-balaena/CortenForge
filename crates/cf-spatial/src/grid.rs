//! Voxel grid data structure.

use std::collections::HashMap;

use nalgebra::Point3;

use crate::error::SpatialError;
use crate::voxel::VoxelCoord;

/// Axis-aligned bounds in grid (voxel) space.
///
/// Represents a rectangular region of voxels defined by minimum and maximum coordinates.
/// Both bounds are inclusive.
///
/// # Example
///
/// ```
/// use cf_spatial::{GridBounds, VoxelCoord};
///
/// let bounds = GridBounds::new(
///     VoxelCoord::new(0, 0, 0),
///     VoxelCoord::new(10, 10, 10),
/// );
///
/// assert!(bounds.contains(VoxelCoord::new(5, 5, 5)));
/// assert!(!bounds.contains(VoxelCoord::new(15, 5, 5)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GridBounds {
    /// Minimum corner (inclusive).
    pub min: VoxelCoord,
    /// Maximum corner (inclusive).
    pub max: VoxelCoord,
}

impl GridBounds {
    /// Creates new grid bounds from min and max coordinates.
    ///
    /// The coordinates are automatically ordered so min ≤ max on each axis.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{GridBounds, VoxelCoord};
    ///
    /// // Coordinates are automatically ordered
    /// let bounds = GridBounds::new(
    ///     VoxelCoord::new(10, 10, 10),
    ///     VoxelCoord::new(0, 0, 0),
    /// );
    /// assert_eq!(bounds.min, VoxelCoord::new(0, 0, 0));
    /// assert_eq!(bounds.max, VoxelCoord::new(10, 10, 10));
    /// ```
    #[must_use]
    pub fn new(a: VoxelCoord, b: VoxelCoord) -> Self {
        Self {
            min: VoxelCoord::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)),
            max: VoxelCoord::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)),
        }
    }

    /// Creates bounds containing a single voxel.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{GridBounds, VoxelCoord};
    ///
    /// let bounds = GridBounds::from_point(VoxelCoord::new(5, 5, 5));
    /// assert_eq!(bounds.min, bounds.max);
    /// assert_eq!(bounds.size(), (1, 1, 1));
    /// ```
    #[must_use]
    pub const fn from_point(coord: VoxelCoord) -> Self {
        Self {
            min: coord,
            max: coord,
        }
    }

    /// Returns the size of the bounds as (width, height, depth).
    ///
    /// Each dimension is at least 1 (for a single voxel).
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{GridBounds, VoxelCoord};
    ///
    /// let bounds = GridBounds::new(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(9, 19, 29),
    /// );
    /// assert_eq!(bounds.size(), (10, 20, 30));
    /// ```
    #[must_use]
    pub const fn size(&self) -> (u32, u32, u32) {
        (
            self.max.x.abs_diff(self.min.x).saturating_add(1),
            self.max.y.abs_diff(self.min.y).saturating_add(1),
            self.max.z.abs_diff(self.min.z).saturating_add(1),
        )
    }

    /// Returns the total number of voxels in this bounds.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{GridBounds, VoxelCoord};
    ///
    /// let bounds = GridBounds::new(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(9, 9, 9),
    /// );
    /// assert_eq!(bounds.volume(), 1000); // 10 * 10 * 10
    /// ```
    #[must_use]
    pub fn volume(&self) -> u64 {
        let (w, h, d) = self.size();
        u64::from(w)
            .saturating_mul(u64::from(h))
            .saturating_mul(u64::from(d))
    }

    /// Checks if the bounds contain a coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{GridBounds, VoxelCoord};
    ///
    /// let bounds = GridBounds::new(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(10, 10, 10),
    /// );
    /// assert!(bounds.contains(VoxelCoord::new(5, 5, 5)));
    /// assert!(bounds.contains(VoxelCoord::new(0, 0, 0))); // min is inclusive
    /// assert!(bounds.contains(VoxelCoord::new(10, 10, 10))); // max is inclusive
    /// assert!(!bounds.contains(VoxelCoord::new(11, 5, 5)));
    /// ```
    #[must_use]
    pub const fn contains(&self, coord: VoxelCoord) -> bool {
        coord.x >= self.min.x
            && coord.x <= self.max.x
            && coord.y >= self.min.y
            && coord.y <= self.max.y
            && coord.z >= self.min.z
            && coord.z <= self.max.z
    }

    /// Expands the bounds to include a coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{GridBounds, VoxelCoord};
    ///
    /// let mut bounds = GridBounds::from_point(VoxelCoord::new(5, 5, 5));
    /// bounds.expand_to_include(VoxelCoord::new(10, 10, 10));
    /// bounds.expand_to_include(VoxelCoord::new(0, 0, 0));
    ///
    /// assert_eq!(bounds.min, VoxelCoord::new(0, 0, 0));
    /// assert_eq!(bounds.max, VoxelCoord::new(10, 10, 10));
    /// ```
    pub fn expand_to_include(&mut self, coord: VoxelCoord) {
        self.min = VoxelCoord::new(
            self.min.x.min(coord.x),
            self.min.y.min(coord.y),
            self.min.z.min(coord.z),
        );
        self.max = VoxelCoord::new(
            self.max.x.max(coord.x),
            self.max.y.max(coord.y),
            self.max.z.max(coord.z),
        );
    }

    /// Returns the intersection of two bounds, or `None` if they don't overlap.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{GridBounds, VoxelCoord};
    ///
    /// let a = GridBounds::new(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(10, 10, 10),
    /// );
    /// let b = GridBounds::new(
    ///     VoxelCoord::new(5, 5, 5),
    ///     VoxelCoord::new(15, 15, 15),
    /// );
    ///
    /// let intersection = a.intersection(&b).unwrap();
    /// assert_eq!(intersection.min, VoxelCoord::new(5, 5, 5));
    /// assert_eq!(intersection.max, VoxelCoord::new(10, 10, 10));
    /// ```
    #[must_use]
    pub fn intersection(&self, other: &Self) -> Option<Self> {
        let min = VoxelCoord::new(
            self.min.x.max(other.min.x),
            self.min.y.max(other.min.y),
            self.min.z.max(other.min.z),
        );
        let max = VoxelCoord::new(
            self.max.x.min(other.max.x),
            self.max.y.min(other.max.y),
            self.max.z.min(other.max.z),
        );

        if min.x <= max.x && min.y <= max.y && min.z <= max.z {
            Some(Self { min, max })
        } else {
            None
        }
    }

    /// Returns whether this bounds overlaps with another.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{GridBounds, VoxelCoord};
    ///
    /// let a = GridBounds::new(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(10, 10, 10),
    /// );
    /// let b = GridBounds::new(
    ///     VoxelCoord::new(5, 5, 5),
    ///     VoxelCoord::new(15, 15, 15),
    /// );
    /// let c = GridBounds::new(
    ///     VoxelCoord::new(20, 20, 20),
    ///     VoxelCoord::new(30, 30, 30),
    /// );
    ///
    /// assert!(a.overlaps(&b));
    /// assert!(!a.overlaps(&c));
    /// ```
    #[must_use]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.intersection(other).is_some()
    }

    /// Returns an iterator over all coordinates in this bounds.
    ///
    /// Iterates in Z-Y-X order (X varies fastest).
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{GridBounds, VoxelCoord};
    ///
    /// let bounds = GridBounds::new(
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 1, 1),
    /// );
    ///
    /// let coords: Vec<_> = bounds.iter().collect();
    /// assert_eq!(coords.len(), 8); // 2x2x2
    /// ```
    #[must_use]
    pub const fn iter(&self) -> GridBoundsIter {
        GridBoundsIter {
            bounds: *self,
            current: Some(self.min),
        }
    }
}

impl Default for GridBounds {
    fn default() -> Self {
        Self::from_point(VoxelCoord::origin())
    }
}

impl IntoIterator for GridBounds {
    type Item = VoxelCoord;
    type IntoIter = GridBoundsIter;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl IntoIterator for &GridBounds {
    type Item = VoxelCoord;
    type IntoIter = GridBoundsIter;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

/// Iterator over all coordinates in a [`GridBounds`].
#[derive(Debug, Clone)]
pub struct GridBoundsIter {
    bounds: GridBounds,
    current: Option<VoxelCoord>,
}

impl Iterator for GridBoundsIter {
    type Item = VoxelCoord;

    fn next(&mut self) -> Option<Self::Item> {
        let current = self.current?;

        // Advance to next position
        let mut next = current;
        next.x += 1;
        if next.x > self.bounds.max.x {
            next.x = self.bounds.min.x;
            next.y += 1;
            if next.y > self.bounds.max.y {
                next.y = self.bounds.min.y;
                next.z += 1;
                if next.z > self.bounds.max.z {
                    self.current = None;
                    return Some(current);
                }
            }
        }
        self.current = Some(next);

        Some(current)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.current.map_or(0, |current| {
            // Use abs_diff to safely compute differences
            let remaining_x = u64::from(self.bounds.max.x.abs_diff(current.x)) + 1;
            let remaining_y = u64::from(self.bounds.max.y.abs_diff(current.y));
            let remaining_z = u64::from(self.bounds.max.z.abs_diff(current.z));

            let size_x = u64::from(self.bounds.max.x.abs_diff(self.bounds.min.x)) + 1;
            let size_y = u64::from(self.bounds.max.y.abs_diff(self.bounds.min.y)) + 1;

            remaining_x
                .saturating_add(remaining_y.saturating_mul(size_x))
                .saturating_add(remaining_z.saturating_mul(size_x).saturating_mul(size_y))
        });

        let remaining = usize::try_from(remaining).unwrap_or(usize::MAX);
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for GridBoundsIter {}

/// A sparse 3D voxel grid with configurable voxel size.
///
/// This is a hash-based sparse grid that only stores occupied voxels,
/// making it memory-efficient for large spaces with few occupied cells.
///
/// # Type Parameter
///
/// - `T`: The type of data stored in each voxel. Common choices:
///   - `bool` for simple occupancy
///   - `u8` or `f32` for density/probability
///   - Custom types for additional metadata
///
/// # Coordinate Systems
///
/// The grid bridges two coordinate systems:
/// - **World space**: Continuous `f64` coordinates (e.g., meters)
/// - **Grid space**: Discrete `i32` voxel indices
///
/// Conversion uses the `voxel_size` parameter and optional origin offset.
///
/// # Example
///
/// ```
/// use cf_spatial::{VoxelGrid, VoxelCoord};
/// use nalgebra::Point3;
///
/// // Create a grid with 0.1 unit voxels
/// let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
///
/// // Set voxels by grid coordinate
/// grid.set(VoxelCoord::new(0, 0, 0), true);
/// grid.set(VoxelCoord::new(1, 0, 0), true);
///
/// // Query by grid coordinate
/// assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&true));
/// assert_eq!(grid.get(VoxelCoord::new(5, 5, 5)), None);
///
/// // Convert world coordinates to grid coordinates
/// let world_point = Point3::new(0.15, 0.0, 0.0);
/// let grid_coord = grid.world_to_grid(world_point);
/// assert_eq!(grid_coord, VoxelCoord::new(1, 0, 0));
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VoxelGrid<T> {
    /// Size of each voxel in world units.
    voxel_size: f64,
    /// Inverse of voxel size for faster coordinate conversion.
    inv_voxel_size: f64,
    /// Origin offset in world space.
    origin: Point3<f64>,
    /// Sparse storage of voxel data.
    data: HashMap<VoxelCoord, T>,
}

impl<T> VoxelGrid<T> {
    /// Creates a new empty voxel grid with the specified voxel size.
    ///
    /// # Arguments
    ///
    /// * `voxel_size` - The size of each voxel in world units. Must be positive.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    /// assert_eq!(grid.voxel_size(), 0.1);
    /// assert!(grid.is_empty());
    /// ```
    ///
    /// # Panics
    ///
    /// This function does not panic. Use [`VoxelGrid::try_new`] for fallible construction.
    #[must_use]
    pub fn new(voxel_size: f64) -> Self {
        Self::with_origin(voxel_size, Point3::origin())
    }

    /// Creates a new empty voxel grid with the specified voxel size and origin.
    ///
    /// # Arguments
    ///
    /// * `voxel_size` - The size of each voxel in world units. Must be positive.
    /// * `origin` - The world-space position of grid coordinate (0, 0, 0).
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelGrid;
    /// use nalgebra::Point3;
    ///
    /// let origin = Point3::new(10.0, 20.0, 30.0);
    /// let grid: VoxelGrid<bool> = VoxelGrid::with_origin(0.1, origin);
    /// assert_eq!(grid.origin(), &origin);
    /// ```
    #[must_use]
    pub fn with_origin(voxel_size: f64, origin: Point3<f64>) -> Self {
        let voxel_size = voxel_size.abs().max(f64::EPSILON);
        Self {
            voxel_size,
            inv_voxel_size: 1.0 / voxel_size,
            origin,
            data: HashMap::new(),
        }
    }

    /// Attempts to create a new voxel grid, returning an error if the voxel size is invalid.
    ///
    /// # Errors
    ///
    /// Returns [`SpatialError::InvalidVoxelSize`] if `voxel_size` is not positive or finite.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, SpatialError};
    ///
    /// let grid: Result<VoxelGrid<bool>, _> = VoxelGrid::try_new(0.1);
    /// assert!(grid.is_ok());
    ///
    /// let invalid: Result<VoxelGrid<bool>, _> = VoxelGrid::try_new(-1.0);
    /// assert!(matches!(invalid, Err(SpatialError::InvalidVoxelSize(_))));
    /// ```
    pub fn try_new(voxel_size: f64) -> Result<Self, SpatialError> {
        if voxel_size <= 0.0 || !voxel_size.is_finite() {
            return Err(SpatialError::InvalidVoxelSize(voxel_size));
        }
        Ok(Self::new(voxel_size))
    }

    /// Returns the voxel size.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(0.5);
    /// assert_eq!(grid.voxel_size(), 0.5);
    /// ```
    #[must_use]
    pub const fn voxel_size(&self) -> f64 {
        self.voxel_size
    }

    /// Returns the grid origin in world space.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelGrid;
    /// use nalgebra::Point3;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    /// assert_eq!(grid.origin(), &Point3::origin());
    /// ```
    #[must_use]
    pub const fn origin(&self) -> &Point3<f64> {
        &self.origin
    }

    /// Returns the number of occupied voxels.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    /// assert_eq!(grid.len(), 0);
    ///
    /// grid.set(VoxelCoord::new(0, 0, 0), true);
    /// assert_eq!(grid.len(), 1);
    /// ```
    #[must_use]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Returns `true` if the grid has no occupied voxels.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    /// assert!(grid.is_empty());
    ///
    /// grid.set(VoxelCoord::new(0, 0, 0), true);
    /// assert!(!grid.is_empty());
    /// ```
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Converts a world-space point to a grid coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    /// use nalgebra::Point3;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    ///
    /// // Point at (0.15, 0.25, 0.35) maps to voxel (1, 2, 3)
    /// let coord = grid.world_to_grid(Point3::new(0.15, 0.25, 0.35));
    /// assert_eq!(coord, VoxelCoord::new(1, 2, 3));
    ///
    /// // Negative coordinates work too
    /// let neg_coord = grid.world_to_grid(Point3::new(-0.15, -0.25, -0.35));
    /// assert_eq!(neg_coord, VoxelCoord::new(-2, -3, -4));
    /// ```
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn world_to_grid(&self, point: Point3<f64>) -> VoxelCoord {
        let relative = point - self.origin;
        // Truncation is intentional - we want to convert continuous coords to discrete grid coords
        VoxelCoord::new(
            (relative.x * self.inv_voxel_size).floor() as i32,
            (relative.y * self.inv_voxel_size).floor() as i32,
            (relative.z * self.inv_voxel_size).floor() as i32,
        )
    }

    /// Converts a grid coordinate to the world-space center of that voxel.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    /// use nalgebra::Point3;
    /// use approx::assert_relative_eq;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    ///
    /// let center = grid.grid_to_world_center(VoxelCoord::new(0, 0, 0));
    /// assert_relative_eq!(center.x, 0.05, epsilon = 1e-10);
    /// assert_relative_eq!(center.y, 0.05, epsilon = 1e-10);
    /// assert_relative_eq!(center.z, 0.05, epsilon = 1e-10);
    /// ```
    #[must_use]
    pub fn grid_to_world_center(&self, coord: VoxelCoord) -> Point3<f64> {
        let half = self.voxel_size * 0.5;
        Point3::new(
            f64::from(coord.x).mul_add(self.voxel_size, self.origin.x) + half,
            f64::from(coord.y).mul_add(self.voxel_size, self.origin.y) + half,
            f64::from(coord.z).mul_add(self.voxel_size, self.origin.z) + half,
        )
    }

    /// Converts a grid coordinate to the world-space minimum corner of that voxel.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    /// use nalgebra::Point3;
    /// use approx::assert_relative_eq;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    ///
    /// let min = grid.grid_to_world_min(VoxelCoord::new(1, 2, 3));
    /// assert_relative_eq!(min.x, 0.1, epsilon = 1e-10);
    /// assert_relative_eq!(min.y, 0.2, epsilon = 1e-10);
    /// assert_relative_eq!(min.z, 0.3, epsilon = 1e-10);
    /// ```
    #[must_use]
    pub fn grid_to_world_min(&self, coord: VoxelCoord) -> Point3<f64> {
        Point3::new(
            f64::from(coord.x).mul_add(self.voxel_size, self.origin.x),
            f64::from(coord.y).mul_add(self.voxel_size, self.origin.y),
            f64::from(coord.z).mul_add(self.voxel_size, self.origin.z),
        )
    }

    /// Converts a grid coordinate to the world-space maximum corner of that voxel.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    /// use nalgebra::Point3;
    /// use approx::assert_relative_eq;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    ///
    /// let max = grid.grid_to_world_max(VoxelCoord::new(1, 2, 3));
    /// assert_relative_eq!(max.x, 0.2, epsilon = 1e-10);
    /// assert_relative_eq!(max.y, 0.3, epsilon = 1e-10);
    /// assert_relative_eq!(max.z, 0.4, epsilon = 1e-10);
    /// ```
    #[must_use]
    pub fn grid_to_world_max(&self, coord: VoxelCoord) -> Point3<f64> {
        Point3::new(
            f64::from(coord.x + 1).mul_add(self.voxel_size, self.origin.x),
            f64::from(coord.y + 1).mul_add(self.voxel_size, self.origin.y),
            f64::from(coord.z + 1).mul_add(self.voxel_size, self.origin.z),
        )
    }

    /// Gets a reference to the value at a grid coordinate.
    ///
    /// Returns `None` if the coordinate is not occupied.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), 42);
    ///
    /// assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&42));
    /// assert_eq!(grid.get(VoxelCoord::new(1, 1, 1)), None);
    /// ```
    #[must_use]
    pub fn get(&self, coord: VoxelCoord) -> Option<&T> {
        self.data.get(&coord)
    }

    /// Gets a mutable reference to the value at a grid coordinate.
    ///
    /// Returns `None` if the coordinate is not occupied.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), 42);
    ///
    /// if let Some(value) = grid.get_mut(VoxelCoord::new(0, 0, 0)) {
    ///     *value = 100;
    /// }
    ///
    /// assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&100));
    /// ```
    pub fn get_mut(&mut self, coord: VoxelCoord) -> Option<&mut T> {
        self.data.get_mut(&coord)
    }

    /// Gets the value at a world-space point.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    /// use nalgebra::Point3;
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(1, 2, 3), 42);
    ///
    /// let value = grid.get_at_world(Point3::new(0.15, 0.25, 0.35));
    /// assert_eq!(value, Some(&42));
    /// ```
    #[must_use]
    pub fn get_at_world(&self, point: Point3<f64>) -> Option<&T> {
        self.get(self.world_to_grid(point))
    }

    /// Sets the value at a grid coordinate.
    ///
    /// Returns the previous value if there was one.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    ///
    /// let old = grid.set(VoxelCoord::new(0, 0, 0), 42);
    /// assert_eq!(old, None);
    ///
    /// let old = grid.set(VoxelCoord::new(0, 0, 0), 100);
    /// assert_eq!(old, Some(42));
    /// ```
    pub fn set(&mut self, coord: VoxelCoord, value: T) -> Option<T> {
        self.data.insert(coord, value)
    }

    /// Sets the value at a world-space point.
    ///
    /// Returns the previous value if there was one.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    /// use nalgebra::Point3;
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    /// grid.set_at_world(Point3::new(0.15, 0.25, 0.35), 42);
    ///
    /// assert_eq!(grid.get(VoxelCoord::new(1, 2, 3)), Some(&42));
    /// ```
    pub fn set_at_world(&mut self, point: Point3<f64>, value: T) -> Option<T> {
        self.set(self.world_to_grid(point), value)
    }

    /// Removes the value at a grid coordinate.
    ///
    /// Returns the removed value if there was one.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), 42);
    ///
    /// let removed = grid.remove(VoxelCoord::new(0, 0, 0));
    /// assert_eq!(removed, Some(42));
    /// assert!(grid.is_empty());
    /// ```
    pub fn remove(&mut self, coord: VoxelCoord) -> Option<T> {
        self.data.remove(&coord)
    }

    /// Checks if a grid coordinate is occupied.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), true);
    ///
    /// assert!(grid.contains(VoxelCoord::new(0, 0, 0)));
    /// assert!(!grid.contains(VoxelCoord::new(1, 1, 1)));
    /// ```
    #[must_use]
    pub fn contains(&self, coord: VoxelCoord) -> bool {
        self.data.contains_key(&coord)
    }

    /// Checks if a world-space point is in an occupied voxel.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    /// use nalgebra::Point3;
    ///
    /// let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(1, 2, 3), true);
    ///
    /// assert!(grid.contains_at_world(Point3::new(0.15, 0.25, 0.35)));
    /// assert!(!grid.contains_at_world(Point3::new(0.0, 0.0, 0.0)));
    /// ```
    #[must_use]
    pub fn contains_at_world(&self, point: Point3<f64>) -> bool {
        self.contains(self.world_to_grid(point))
    }

    /// Clears all voxels from the grid.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), true);
    /// grid.set(VoxelCoord::new(1, 1, 1), true);
    ///
    /// grid.clear();
    /// assert!(grid.is_empty());
    /// ```
    pub fn clear(&mut self) {
        self.data.clear();
    }

    /// Returns an iterator over all occupied coordinates and their values.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), 1);
    /// grid.set(VoxelCoord::new(1, 1, 1), 2);
    ///
    /// for (coord, value) in grid.iter() {
    ///     println!("{:?}: {}", coord, value);
    /// }
    /// ```
    pub fn iter(&self) -> impl Iterator<Item = (&VoxelCoord, &T)> {
        self.data.iter()
    }

    /// Returns an iterator over all occupied coordinates and mutable values.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), 1);
    /// grid.set(VoxelCoord::new(1, 1, 1), 2);
    ///
    /// for (_, value) in grid.iter_mut() {
    ///     *value *= 10;
    /// }
    /// ```
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (&VoxelCoord, &mut T)> {
        self.data.iter_mut()
    }

    /// Returns an iterator over all occupied coordinates.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), true);
    /// grid.set(VoxelCoord::new(1, 1, 1), true);
    ///
    /// let coords: Vec<_> = grid.coords().collect();
    /// assert_eq!(coords.len(), 2);
    /// ```
    pub fn coords(&self) -> impl Iterator<Item = &VoxelCoord> {
        self.data.keys()
    }

    /// Returns an iterator over all values.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), 1);
    /// grid.set(VoxelCoord::new(1, 1, 1), 2);
    ///
    /// let sum: i32 = grid.values().sum();
    /// assert_eq!(sum, 3);
    /// ```
    pub fn values(&self) -> impl Iterator<Item = &T> {
        self.data.values()
    }

    /// Computes the axis-aligned bounding box of all occupied voxels.
    ///
    /// Returns `None` if the grid is empty.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
    /// assert!(grid.bounds().is_none());
    ///
    /// grid.set(VoxelCoord::new(0, 0, 0), true);
    /// grid.set(VoxelCoord::new(10, 20, 30), true);
    ///
    /// let bounds = grid.bounds().unwrap();
    /// assert_eq!(bounds.min, VoxelCoord::new(0, 0, 0));
    /// assert_eq!(bounds.max, VoxelCoord::new(10, 20, 30));
    /// ```
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
}

impl<T: Clone> VoxelGrid<T> {
    /// Gets the value at a grid coordinate, or a default value if not occupied.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    /// grid.set(VoxelCoord::new(0, 0, 0), 42);
    ///
    /// assert_eq!(grid.get_or(VoxelCoord::new(0, 0, 0), 0), 42);
    /// assert_eq!(grid.get_or(VoxelCoord::new(1, 1, 1), -1), -1);
    /// ```
    #[must_use]
    pub fn get_or(&self, coord: VoxelCoord, default: T) -> T {
        self.data.get(&coord).cloned().unwrap_or(default)
    }
}

impl<T: Default> VoxelGrid<T> {
    /// Gets a mutable reference to the value at a coordinate, inserting a default if not present.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::{VoxelGrid, VoxelCoord};
    ///
    /// let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
    ///
    /// // First access creates the entry with default value (0)
    /// *grid.get_or_insert_default(VoxelCoord::new(0, 0, 0)) += 5;
    /// assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&5));
    ///
    /// // Second access uses existing value
    /// *grid.get_or_insert_default(VoxelCoord::new(0, 0, 0)) += 3;
    /// assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&8));
    /// ```
    pub fn get_or_insert_default(&mut self, coord: VoxelCoord) -> &mut T {
        self.data.entry(coord).or_default()
    }
}

impl<T> Default for VoxelGrid<T> {
    fn default() -> Self {
        Self::new(1.0)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::needless_collect)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // ==================== GridBounds Tests ====================

    #[test]
    fn test_bounds_new() {
        let bounds = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 10, 10));
        assert_eq!(bounds.min, VoxelCoord::new(0, 0, 0));
        assert_eq!(bounds.max, VoxelCoord::new(10, 10, 10));
    }

    #[test]
    fn test_bounds_new_auto_order() {
        let bounds = GridBounds::new(VoxelCoord::new(10, 10, 10), VoxelCoord::new(0, 0, 0));
        assert_eq!(bounds.min, VoxelCoord::new(0, 0, 0));
        assert_eq!(bounds.max, VoxelCoord::new(10, 10, 10));
    }

    #[test]
    fn test_bounds_from_point() {
        let bounds = GridBounds::from_point(VoxelCoord::new(5, 5, 5));
        assert_eq!(bounds.min, VoxelCoord::new(5, 5, 5));
        assert_eq!(bounds.max, VoxelCoord::new(5, 5, 5));
    }

    #[test]
    fn test_bounds_size() {
        let bounds = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(9, 19, 29));
        assert_eq!(bounds.size(), (10, 20, 30));
    }

    #[test]
    fn test_bounds_size_single() {
        let bounds = GridBounds::from_point(VoxelCoord::new(0, 0, 0));
        assert_eq!(bounds.size(), (1, 1, 1));
    }

    #[test]
    fn test_bounds_volume() {
        let bounds = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(9, 9, 9));
        assert_eq!(bounds.volume(), 1000);
    }

    #[test]
    fn test_bounds_contains() {
        let bounds = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 10, 10));
        assert!(bounds.contains(VoxelCoord::new(5, 5, 5)));
        assert!(bounds.contains(VoxelCoord::new(0, 0, 0)));
        assert!(bounds.contains(VoxelCoord::new(10, 10, 10)));
        assert!(!bounds.contains(VoxelCoord::new(11, 5, 5)));
        assert!(!bounds.contains(VoxelCoord::new(-1, 5, 5)));
    }

    #[test]
    fn test_bounds_expand() {
        let mut bounds = GridBounds::from_point(VoxelCoord::new(5, 5, 5));
        bounds.expand_to_include(VoxelCoord::new(10, 10, 10));
        bounds.expand_to_include(VoxelCoord::new(0, 0, 0));
        assert_eq!(bounds.min, VoxelCoord::new(0, 0, 0));
        assert_eq!(bounds.max, VoxelCoord::new(10, 10, 10));
    }

    #[test]
    fn test_bounds_intersection() {
        let a = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 10, 10));
        let b = GridBounds::new(VoxelCoord::new(5, 5, 5), VoxelCoord::new(15, 15, 15));
        let intersection = a.intersection(&b).unwrap();
        assert_eq!(intersection.min, VoxelCoord::new(5, 5, 5));
        assert_eq!(intersection.max, VoxelCoord::new(10, 10, 10));
    }

    #[test]
    fn test_bounds_intersection_none() {
        let a = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(5, 5, 5));
        let b = GridBounds::new(VoxelCoord::new(10, 10, 10), VoxelCoord::new(15, 15, 15));
        assert!(a.intersection(&b).is_none());
    }

    #[test]
    fn test_bounds_overlaps() {
        let a = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(10, 10, 10));
        let b = GridBounds::new(VoxelCoord::new(5, 5, 5), VoxelCoord::new(15, 15, 15));
        let c = GridBounds::new(VoxelCoord::new(20, 20, 20), VoxelCoord::new(30, 30, 30));
        assert!(a.overlaps(&b));
        assert!(!a.overlaps(&c));
    }

    #[test]
    fn test_bounds_iter() {
        let bounds = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 1, 1));
        let coords: Vec<_> = bounds.iter().collect();
        assert_eq!(coords.len(), 8);
        assert!(coords.contains(&VoxelCoord::new(0, 0, 0)));
        assert!(coords.contains(&VoxelCoord::new(1, 1, 1)));
    }

    #[test]
    fn test_bounds_iter_exact_size() {
        let bounds = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(2, 3, 4));
        let iter = bounds.iter();
        assert_eq!(iter.len(), 60); // 3 * 4 * 5
    }

    #[test]
    fn test_bounds_into_iter() {
        let bounds = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 1, 1));
        let count = bounds.into_iter().count();
        assert_eq!(count, 8);
    }

    #[test]
    fn test_bounds_ref_into_iter() {
        let bounds = GridBounds::new(VoxelCoord::new(0, 0, 0), VoxelCoord::new(1, 1, 1));
        let count = (&bounds).into_iter().count();
        assert_eq!(count, 8);
        // bounds is still usable
        assert_eq!(bounds.volume(), 8);
    }

    // ==================== VoxelGrid Tests ====================

    #[test]
    fn test_grid_new() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        assert_eq!(grid.voxel_size(), 0.1);
        assert!(grid.is_empty());
    }

    #[test]
    fn test_grid_with_origin() {
        let origin = Point3::new(10.0, 20.0, 30.0);
        let grid: VoxelGrid<bool> = VoxelGrid::with_origin(0.1, origin);
        assert_eq!(grid.origin(), &origin);
    }

    #[test]
    fn test_grid_try_new() {
        let grid: Result<VoxelGrid<bool>, _> = VoxelGrid::try_new(0.1);
        assert!(grid.is_ok());
    }

    #[test]
    fn test_grid_try_new_invalid() {
        let grid: Result<VoxelGrid<bool>, _> = VoxelGrid::try_new(-1.0);
        assert!(matches!(grid, Err(SpatialError::InvalidVoxelSize(_))));

        let grid: Result<VoxelGrid<bool>, _> = VoxelGrid::try_new(0.0);
        assert!(matches!(grid, Err(SpatialError::InvalidVoxelSize(_))));

        let grid: Result<VoxelGrid<bool>, _> = VoxelGrid::try_new(f64::NAN);
        assert!(matches!(grid, Err(SpatialError::InvalidVoxelSize(_))));

        let grid: Result<VoxelGrid<bool>, _> = VoxelGrid::try_new(f64::INFINITY);
        assert!(matches!(grid, Err(SpatialError::InvalidVoxelSize(_))));
    }

    #[test]
    fn test_grid_set_get() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), 42);
        assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&42));
        assert_eq!(grid.get(VoxelCoord::new(1, 1, 1)), None);
    }

    #[test]
    fn test_grid_set_overwrite() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        let old = grid.set(VoxelCoord::new(0, 0, 0), 42);
        assert_eq!(old, None);
        let old = grid.set(VoxelCoord::new(0, 0, 0), 100);
        assert_eq!(old, Some(42));
        assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&100));
    }

    #[test]
    fn test_grid_get_mut() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), 42);
        if let Some(value) = grid.get_mut(VoxelCoord::new(0, 0, 0)) {
            *value = 100;
        }
        assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&100));
    }

    #[test]
    fn test_grid_remove() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), 42);
        let removed = grid.remove(VoxelCoord::new(0, 0, 0));
        assert_eq!(removed, Some(42));
        assert!(grid.is_empty());
    }

    #[test]
    fn test_grid_contains() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), true);
        assert!(grid.contains(VoxelCoord::new(0, 0, 0)));
        assert!(!grid.contains(VoxelCoord::new(1, 1, 1)));
    }

    #[test]
    fn test_grid_len() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        assert_eq!(grid.len(), 0);
        grid.set(VoxelCoord::new(0, 0, 0), true);
        assert_eq!(grid.len(), 1);
        grid.set(VoxelCoord::new(1, 1, 1), true);
        assert_eq!(grid.len(), 2);
    }

    #[test]
    fn test_grid_clear() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), true);
        grid.set(VoxelCoord::new(1, 1, 1), true);
        grid.clear();
        assert!(grid.is_empty());
    }

    #[test]
    fn test_grid_world_to_grid() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        assert_eq!(
            grid.world_to_grid(Point3::new(0.0, 0.0, 0.0)),
            VoxelCoord::new(0, 0, 0)
        );
        assert_eq!(
            grid.world_to_grid(Point3::new(0.15, 0.25, 0.35)),
            VoxelCoord::new(1, 2, 3)
        );
        assert_eq!(
            grid.world_to_grid(Point3::new(0.99, 0.99, 0.99)),
            VoxelCoord::new(9, 9, 9)
        );
    }

    #[test]
    fn test_grid_world_to_grid_negative() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        assert_eq!(
            grid.world_to_grid(Point3::new(-0.05, -0.05, -0.05)),
            VoxelCoord::new(-1, -1, -1)
        );
        assert_eq!(
            grid.world_to_grid(Point3::new(-0.15, -0.25, -0.35)),
            VoxelCoord::new(-2, -3, -4)
        );
    }

    #[test]
    fn test_grid_world_to_grid_with_origin() {
        let grid: VoxelGrid<bool> = VoxelGrid::with_origin(0.1, Point3::new(1.0, 2.0, 3.0));
        assert_eq!(
            grid.world_to_grid(Point3::new(1.0, 2.0, 3.0)),
            VoxelCoord::new(0, 0, 0)
        );
        assert_eq!(
            grid.world_to_grid(Point3::new(1.15, 2.25, 3.35)),
            VoxelCoord::new(1, 2, 3)
        );
    }

    #[test]
    fn test_grid_to_world_center() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        let center = grid.grid_to_world_center(VoxelCoord::new(0, 0, 0));
        assert_relative_eq!(center.x, 0.05, epsilon = 1e-10);
        assert_relative_eq!(center.y, 0.05, epsilon = 1e-10);
        assert_relative_eq!(center.z, 0.05, epsilon = 1e-10);
    }

    #[test]
    fn test_grid_to_world_min() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        let min = grid.grid_to_world_min(VoxelCoord::new(1, 2, 3));
        assert_relative_eq!(min.x, 0.1, epsilon = 1e-10);
        assert_relative_eq!(min.y, 0.2, epsilon = 1e-10);
        assert_relative_eq!(min.z, 0.3, epsilon = 1e-10);
    }

    #[test]
    fn test_grid_to_world_max() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        let max = grid.grid_to_world_max(VoxelCoord::new(1, 2, 3));
        assert_relative_eq!(max.x, 0.2, epsilon = 1e-10);
        assert_relative_eq!(max.y, 0.3, epsilon = 1e-10);
        assert_relative_eq!(max.z, 0.4, epsilon = 1e-10);
    }

    #[test]
    fn test_grid_world_roundtrip() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        let coord = VoxelCoord::new(5, 10, 15);
        let center = grid.grid_to_world_center(coord);
        let back = grid.world_to_grid(center);
        assert_eq!(back, coord);
    }

    #[test]
    fn test_grid_set_at_world() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set_at_world(Point3::new(0.15, 0.25, 0.35), 42);
        assert_eq!(grid.get(VoxelCoord::new(1, 2, 3)), Some(&42));
    }

    #[test]
    fn test_grid_get_at_world() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(1, 2, 3), 42);
        assert_eq!(grid.get_at_world(Point3::new(0.15, 0.25, 0.35)), Some(&42));
    }

    #[test]
    fn test_grid_contains_at_world() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(1, 2, 3), true);
        assert!(grid.contains_at_world(Point3::new(0.15, 0.25, 0.35)));
        assert!(!grid.contains_at_world(Point3::new(0.0, 0.0, 0.0)));
    }

    #[test]
    fn test_grid_iter() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), 1);
        grid.set(VoxelCoord::new(1, 1, 1), 2);
        let entries: Vec<_> = grid.iter().collect();
        assert_eq!(entries.len(), 2);
    }

    #[test]
    fn test_grid_iter_mut() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), 1);
        grid.set(VoxelCoord::new(1, 1, 1), 2);
        for (_, value) in grid.iter_mut() {
            *value *= 10;
        }
        assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&10));
        assert_eq!(grid.get(VoxelCoord::new(1, 1, 1)), Some(&20));
    }

    #[test]
    fn test_grid_coords() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), true);
        grid.set(VoxelCoord::new(1, 1, 1), true);
        let coords: Vec<_> = grid.coords().collect();
        assert_eq!(coords.len(), 2);
    }

    #[test]
    fn test_grid_values() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), 1);
        grid.set(VoxelCoord::new(1, 1, 1), 2);
        let sum: i32 = grid.values().sum();
        assert_eq!(sum, 3);
    }

    #[test]
    fn test_grid_bounds() {
        let mut grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        assert!(grid.bounds().is_none());

        grid.set(VoxelCoord::new(0, 0, 0), true);
        grid.set(VoxelCoord::new(10, 20, 30), true);

        let bounds = grid.bounds().unwrap();
        assert_eq!(bounds.min, VoxelCoord::new(0, 0, 0));
        assert_eq!(bounds.max, VoxelCoord::new(10, 20, 30));
    }

    #[test]
    fn test_grid_get_or() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), 42);
        assert_eq!(grid.get_or(VoxelCoord::new(0, 0, 0), 0), 42);
        assert_eq!(grid.get_or(VoxelCoord::new(1, 1, 1), -1), -1);
    }

    #[test]
    fn test_grid_get_or_insert_default() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        *grid.get_or_insert_default(VoxelCoord::new(0, 0, 0)) += 5;
        assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&5));
        *grid.get_or_insert_default(VoxelCoord::new(0, 0, 0)) += 3;
        assert_eq!(grid.get(VoxelCoord::new(0, 0, 0)), Some(&8));
    }

    #[test]
    fn test_grid_default() {
        let grid: VoxelGrid<bool> = VoxelGrid::default();
        assert_eq!(grid.voxel_size(), 1.0);
        assert!(grid.is_empty());
    }

    #[test]
    fn test_grid_clone() {
        let mut grid: VoxelGrid<i32> = VoxelGrid::new(0.1);
        grid.set(VoxelCoord::new(0, 0, 0), 42);
        let cloned = grid.clone();
        assert_eq!(cloned.get(VoxelCoord::new(0, 0, 0)), Some(&42));
    }

    #[test]
    fn test_grid_negative_voxel_size_clamped() {
        // Negative voxel sizes get clamped to positive
        let grid: VoxelGrid<bool> = VoxelGrid::new(-0.1);
        assert!(grid.voxel_size() > 0.0);
    }

    #[test]
    fn test_grid_debug() {
        let grid: VoxelGrid<bool> = VoxelGrid::new(0.1);
        let debug_str = format!("{grid:?}");
        assert!(debug_str.contains("VoxelGrid"));
    }
}
