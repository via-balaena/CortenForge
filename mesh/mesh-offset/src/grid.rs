//! 3D scalar grid for storing SDF values.

use nalgebra::Point3;

/// A 3D grid storing scalar values.
///
/// Used for storing signed distance field values and extracting
/// isosurfaces via marching cubes.
#[derive(Debug, Clone)]
pub struct ScalarGrid {
    /// Grid values stored in row-major order (x varies fastest).
    values: Vec<f64>,
    /// Grid dimensions (nx, ny, nz).
    dimensions: (usize, usize, usize),
    /// Minimum corner of the grid.
    origin: Point3<f64>,
    /// Size of each grid cell.
    cell_size: f64,
}

impl ScalarGrid {
    /// Create a new scalar grid.
    ///
    /// # Arguments
    ///
    /// * `dimensions` - Grid dimensions (nx, ny, nz)
    /// * `origin` - Minimum corner of the grid
    /// * `cell_size` - Size of each grid cell
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_offset::ScalarGrid;
    /// use nalgebra::Point3;
    ///
    /// let grid = ScalarGrid::new(
    ///     (10, 10, 10),
    ///     Point3::new(-5.0, -5.0, -5.0),
    ///     1.0,
    /// );
    /// assert_eq!(grid.dimensions(), (10, 10, 10));
    /// ```
    #[must_use]
    pub fn new(dimensions: (usize, usize, usize), origin: Point3<f64>, cell_size: f64) -> Self {
        let (nx, ny, nz) = dimensions;
        let values = vec![0.0; nx * ny * nz];

        Self {
            values,
            dimensions,
            origin,
            cell_size,
        }
    }

    /// Create a grid from mesh bounds with automatic sizing.
    ///
    /// # Arguments
    ///
    /// * `min` - Minimum corner of bounds
    /// * `max` - Maximum corner of bounds
    /// * `cell_size` - Target cell size
    /// * `padding` - Extra cells to add around bounds
    #[must_use]
    pub fn from_bounds(min: Point3<f64>, max: Point3<f64>, cell_size: f64, padding: usize) -> Self {
        let extent = max - min;
        let padding_f = padding as f64 * cell_size;

        let origin = Point3::new(min.x - padding_f, min.y - padding_f, min.z - padding_f);

        let nx = ((extent.x + 2.0 * padding_f) / cell_size).ceil() as usize + 1;
        let ny = ((extent.y + 2.0 * padding_f) / cell_size).ceil() as usize + 1;
        let nz = ((extent.z + 2.0 * padding_f) / cell_size).ceil() as usize + 1;

        Self::new((nx, ny, nz), origin, cell_size)
    }

    /// Get grid dimensions.
    #[must_use]
    pub fn dimensions(&self) -> (usize, usize, usize) {
        self.dimensions
    }

    /// Get the grid origin (minimum corner).
    #[must_use]
    pub fn origin(&self) -> Point3<f64> {
        self.origin
    }

    /// Get the cell size.
    #[must_use]
    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }

    /// Get the value at grid coordinates.
    ///
    /// Returns 0.0 if coordinates are out of bounds.
    #[must_use]
    pub fn get(&self, ix: usize, iy: usize, iz: usize) -> f64 {
        if ix < self.dimensions.0 && iy < self.dimensions.1 && iz < self.dimensions.2 {
            self.values[self.index(ix, iy, iz)]
        } else {
            0.0
        }
    }

    /// Set the value at grid coordinates.
    ///
    /// Does nothing if coordinates are out of bounds.
    pub fn set(&mut self, ix: usize, iy: usize, iz: usize, value: f64) {
        if ix < self.dimensions.0 && iy < self.dimensions.1 && iz < self.dimensions.2 {
            let idx = self.index(ix, iy, iz);
            self.values[idx] = value;
        }
    }

    /// Get the world-space position of a grid point.
    #[must_use]
    pub fn position(&self, ix: usize, iy: usize, iz: usize) -> Point3<f64> {
        Point3::new(
            self.origin.x + ix as f64 * self.cell_size,
            self.origin.y + iy as f64 * self.cell_size,
            self.origin.z + iz as f64 * self.cell_size,
        )
    }

    /// Get the grid coordinates for a world-space position.
    ///
    /// Returns `None` if the position is outside the grid.
    #[must_use]
    pub fn grid_coords(&self, point: Point3<f64>) -> Option<(usize, usize, usize)> {
        let offset = point - self.origin;

        let ix = (offset.x / self.cell_size).floor() as isize;
        let iy = (offset.y / self.cell_size).floor() as isize;
        let iz = (offset.z / self.cell_size).floor() as isize;

        if ix >= 0
            && iy >= 0
            && iz >= 0
            && (ix as usize) < self.dimensions.0
            && (iy as usize) < self.dimensions.1
            && (iz as usize) < self.dimensions.2
        {
            Some((ix as usize, iy as usize, iz as usize))
        } else {
            None
        }
    }

    /// Iterate over all grid points.
    pub fn iter_points(&self) -> impl Iterator<Item = (usize, usize, usize, Point3<f64>)> + '_ {
        let (nx, ny, nz) = self.dimensions;

        (0..nz).flat_map(move |iz| {
            (0..ny)
                .flat_map(move |iy| (0..nx).map(move |ix| (ix, iy, iz, self.position(ix, iy, iz))))
        })
    }

    /// Get the total number of grid points.
    #[must_use]
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Check if the grid is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// Convert 3D coordinates to linear index.
    fn index(&self, ix: usize, iy: usize, iz: usize) -> usize {
        ix + iy * self.dimensions.0 + iz * self.dimensions.0 * self.dimensions.1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn new_grid() {
        let grid = ScalarGrid::new((10, 10, 10), Point3::origin(), 1.0);

        assert_eq!(grid.dimensions(), (10, 10, 10));
        assert_eq!(grid.len(), 1000);
    }

    #[test]
    fn from_bounds() {
        let grid = ScalarGrid::from_bounds(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 10.0),
            1.0,
            2,
        );

        // Should have padding on all sides
        assert!(grid.origin().x < 0.0);
    }

    #[test]
    fn get_set() {
        let mut grid = ScalarGrid::new((5, 5, 5), Point3::origin(), 1.0);

        grid.set(2, 3, 4, 42.0);
        assert_relative_eq!(grid.get(2, 3, 4), 42.0);
    }

    #[test]
    fn get_out_of_bounds() {
        let grid = ScalarGrid::new((5, 5, 5), Point3::origin(), 1.0);

        assert_relative_eq!(grid.get(100, 100, 100), 0.0);
    }

    #[test]
    fn position() {
        let grid = ScalarGrid::new((10, 10, 10), Point3::new(-5.0, -5.0, -5.0), 1.0);

        let pos = grid.position(5, 5, 5);
        assert_relative_eq!(pos.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(pos.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn grid_coords() {
        let grid = ScalarGrid::new((10, 10, 10), Point3::origin(), 1.0);

        let coords = grid.grid_coords(Point3::new(2.5, 3.5, 4.5));
        assert!(coords.is_some());

        let (ix, iy, iz) = coords.expect("should have coords");
        assert_eq!(ix, 2);
        assert_eq!(iy, 3);
        assert_eq!(iz, 4);
    }

    #[test]
    fn grid_coords_out_of_bounds() {
        let grid = ScalarGrid::new((10, 10, 10), Point3::origin(), 1.0);

        let coords = grid.grid_coords(Point3::new(-1.0, 0.0, 0.0));
        assert!(coords.is_none());
    }

    #[test]
    fn iter_points_count() {
        let grid = ScalarGrid::new((3, 4, 5), Point3::origin(), 1.0);

        let count = grid.iter_points().count();
        assert_eq!(count, 60);
    }

    #[test]
    fn is_empty() {
        let grid = ScalarGrid::new((0, 0, 0), Point3::origin(), 1.0);
        assert!(grid.is_empty());

        let grid2 = ScalarGrid::new((1, 1, 1), Point3::origin(), 1.0);
        assert!(!grid2.is_empty());
    }
}
