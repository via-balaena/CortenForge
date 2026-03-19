//! Voxel coordinate types.

use nalgebra::{Point3, Vector3};

/// A discrete 3D coordinate in voxel/grid space.
///
/// Uses `i32` coordinates to support both positive and negative indices,
/// allowing the grid origin to be placed anywhere in world space.
///
/// # Example
///
/// ```
/// use cf_spatial::VoxelCoord;
///
/// let coord = VoxelCoord::new(1, 2, 3);
/// assert_eq!(coord.x, 1);
/// assert_eq!(coord.y, 2);
/// assert_eq!(coord.z, 3);
///
/// // Supports negative coordinates
/// let neg_coord = VoxelCoord::new(-5, -10, -15);
/// assert_eq!(neg_coord.x, -5);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VoxelCoord {
    /// X coordinate (width axis).
    pub x: i32,
    /// Y coordinate (depth axis).
    pub y: i32,
    /// Z coordinate (height axis).
    pub z: i32,
}

impl VoxelCoord {
    /// Creates a new voxel coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let coord = VoxelCoord::new(10, 20, 30);
    /// assert_eq!(coord.x, 10);
    /// ```
    #[must_use]
    pub const fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    /// Creates a coordinate at the origin (0, 0, 0).
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let origin = VoxelCoord::origin();
    /// assert_eq!(origin, VoxelCoord::new(0, 0, 0));
    /// ```
    #[must_use]
    pub const fn origin() -> Self {
        Self::new(0, 0, 0)
    }

    /// Returns the coordinate as a tuple.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let coord = VoxelCoord::new(1, 2, 3);
    /// assert_eq!(coord.as_tuple(), (1, 2, 3));
    /// ```
    #[must_use]
    pub const fn as_tuple(self) -> (i32, i32, i32) {
        (self.x, self.y, self.z)
    }

    /// Returns the coordinate as an array.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let coord = VoxelCoord::new(1, 2, 3);
    /// assert_eq!(coord.as_array(), [1, 2, 3]);
    /// ```
    #[must_use]
    pub const fn as_array(self) -> [i32; 3] {
        [self.x, self.y, self.z]
    }

    /// Converts to a floating-point point.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    /// use nalgebra::Point3;
    ///
    /// let coord = VoxelCoord::new(1, 2, 3);
    /// let point = coord.to_point();
    /// assert_eq!(point, Point3::new(1.0, 2.0, 3.0));
    /// ```
    #[must_use]
    pub fn to_point(self) -> Point3<f64> {
        Point3::new(f64::from(self.x), f64::from(self.y), f64::from(self.z))
    }

    /// Converts to a floating-point vector.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    /// use nalgebra::Vector3;
    ///
    /// let coord = VoxelCoord::new(1, 2, 3);
    /// let vec = coord.to_vector();
    /// assert_eq!(vec, Vector3::new(1.0, 2.0, 3.0));
    /// ```
    #[must_use]
    pub fn to_vector(self) -> Vector3<f64> {
        Vector3::new(f64::from(self.x), f64::from(self.y), f64::from(self.z))
    }

    /// Returns the 6 face-adjacent neighbors (von Neumann neighborhood).
    ///
    /// These are the neighbors sharing a face with this voxel.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let coord = VoxelCoord::new(0, 0, 0);
    /// let neighbors = coord.face_neighbors();
    /// assert_eq!(neighbors.len(), 6);
    /// assert!(neighbors.contains(&VoxelCoord::new(1, 0, 0)));
    /// assert!(neighbors.contains(&VoxelCoord::new(-1, 0, 0)));
    /// ```
    #[must_use]
    pub const fn face_neighbors(self) -> [Self; 6] {
        [
            Self::new(self.x.wrapping_add(1), self.y, self.z),
            Self::new(self.x.wrapping_sub(1), self.y, self.z),
            Self::new(self.x, self.y.wrapping_add(1), self.z),
            Self::new(self.x, self.y.wrapping_sub(1), self.z),
            Self::new(self.x, self.y, self.z.wrapping_add(1)),
            Self::new(self.x, self.y, self.z.wrapping_sub(1)),
        ]
    }

    /// Returns all 26 neighbors (Moore neighborhood).
    ///
    /// These include face-adjacent (6), edge-adjacent (12), and corner-adjacent (8) neighbors.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let coord = VoxelCoord::new(0, 0, 0);
    /// let neighbors = coord.all_neighbors();
    /// assert_eq!(neighbors.len(), 26);
    /// ```
    #[must_use]
    pub fn all_neighbors(self) -> [Self; 26] {
        let mut result = [Self::origin(); 26];
        let mut idx = 0;

        for dx in -1i32..=1 {
            for dy in -1i32..=1 {
                for dz in -1i32..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    result[idx] = Self::new(
                        self.x.wrapping_add(dx),
                        self.y.wrapping_add(dy),
                        self.z.wrapping_add(dz),
                    );
                    idx += 1;
                }
            }
        }

        result
    }

    /// Computes the Manhattan distance to another coordinate.
    ///
    /// The Manhattan distance is the sum of the absolute differences of coordinates.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let a = VoxelCoord::new(0, 0, 0);
    /// let b = VoxelCoord::new(3, 4, 5);
    /// assert_eq!(a.manhattan_distance(b), 12);
    /// ```
    #[must_use]
    pub const fn manhattan_distance(self, other: Self) -> u32 {
        let dx = self.x.abs_diff(other.x);
        let dy = self.y.abs_diff(other.y);
        let dz = self.z.abs_diff(other.z);
        dx.saturating_add(dy).saturating_add(dz)
    }

    /// Computes the Chebyshev distance to another coordinate.
    ///
    /// The Chebyshev distance is the maximum of the absolute differences of coordinates.
    /// This is the number of "king moves" on a 3D chessboard.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let a = VoxelCoord::new(0, 0, 0);
    /// let b = VoxelCoord::new(3, 4, 5);
    /// assert_eq!(a.chebyshev_distance(b), 5);
    /// ```
    #[must_use]
    pub fn chebyshev_distance(self, other: Self) -> u32 {
        let dx = self.x.abs_diff(other.x);
        let dy = self.y.abs_diff(other.y);
        let dz = self.z.abs_diff(other.z);
        dx.max(dy).max(dz)
    }

    /// Adds an offset to this coordinate, returning `None` on overflow.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let coord = VoxelCoord::new(5, 5, 5);
    /// let offset = coord.checked_add(VoxelCoord::new(1, 2, 3));
    /// assert_eq!(offset, Some(VoxelCoord::new(6, 7, 8)));
    /// ```
    #[must_use]
    pub fn checked_add(self, other: Self) -> Option<Self> {
        Some(Self::new(
            self.x.checked_add(other.x)?,
            self.y.checked_add(other.y)?,
            self.z.checked_add(other.z)?,
        ))
    }

    /// Subtracts an offset from this coordinate, returning `None` on overflow.
    ///
    /// # Example
    ///
    /// ```
    /// use cf_spatial::VoxelCoord;
    ///
    /// let coord = VoxelCoord::new(5, 5, 5);
    /// let offset = coord.checked_sub(VoxelCoord::new(1, 2, 3));
    /// assert_eq!(offset, Some(VoxelCoord::new(4, 3, 2)));
    /// ```
    #[must_use]
    pub fn checked_sub(self, other: Self) -> Option<Self> {
        Some(Self::new(
            self.x.checked_sub(other.x)?,
            self.y.checked_sub(other.y)?,
            self.z.checked_sub(other.z)?,
        ))
    }
}

impl From<(i32, i32, i32)> for VoxelCoord {
    fn from((x, y, z): (i32, i32, i32)) -> Self {
        Self::new(x, y, z)
    }
}

impl From<[i32; 3]> for VoxelCoord {
    fn from([x, y, z]: [i32; 3]) -> Self {
        Self::new(x, y, z)
    }
}

impl From<VoxelCoord> for (i32, i32, i32) {
    fn from(coord: VoxelCoord) -> Self {
        coord.as_tuple()
    }
}

impl From<VoxelCoord> for [i32; 3] {
    fn from(coord: VoxelCoord) -> Self {
        coord.as_array()
    }
}

impl std::ops::Add for VoxelCoord {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::new(
            self.x.wrapping_add(other.x),
            self.y.wrapping_add(other.y),
            self.z.wrapping_add(other.z),
        )
    }
}

impl std::ops::Sub for VoxelCoord {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self::new(
            self.x.wrapping_sub(other.x),
            self.y.wrapping_sub(other.y),
            self.z.wrapping_sub(other.z),
        )
    }
}

impl std::ops::Neg for VoxelCoord {
    type Output = Self;

    fn neg(self) -> Self {
        Self::new(
            self.x.wrapping_neg(),
            self.y.wrapping_neg(),
            self.z.wrapping_neg(),
        )
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let coord = VoxelCoord::new(1, 2, 3);
        assert_eq!(coord.x, 1);
        assert_eq!(coord.y, 2);
        assert_eq!(coord.z, 3);
    }

    #[test]
    fn test_origin() {
        let origin = VoxelCoord::origin();
        assert_eq!(origin.x, 0);
        assert_eq!(origin.y, 0);
        assert_eq!(origin.z, 0);
    }

    #[test]
    fn test_negative_coords() {
        let coord = VoxelCoord::new(-5, -10, -15);
        assert_eq!(coord.x, -5);
        assert_eq!(coord.y, -10);
        assert_eq!(coord.z, -15);
    }

    #[test]
    fn test_as_tuple() {
        let coord = VoxelCoord::new(1, 2, 3);
        assert_eq!(coord.as_tuple(), (1, 2, 3));
    }

    #[test]
    fn test_as_array() {
        let coord = VoxelCoord::new(1, 2, 3);
        assert_eq!(coord.as_array(), [1, 2, 3]);
    }

    #[test]
    fn test_to_point() {
        let coord = VoxelCoord::new(1, 2, 3);
        let point = coord.to_point();
        assert_eq!(point.x, 1.0);
        assert_eq!(point.y, 2.0);
        assert_eq!(point.z, 3.0);
    }

    #[test]
    fn test_to_vector() {
        let coord = VoxelCoord::new(1, 2, 3);
        let vec = coord.to_vector();
        assert_eq!(vec.x, 1.0);
        assert_eq!(vec.y, 2.0);
        assert_eq!(vec.z, 3.0);
    }

    #[test]
    fn test_face_neighbors() {
        let coord = VoxelCoord::new(5, 5, 5);
        let neighbors = coord.face_neighbors();
        assert_eq!(neighbors.len(), 6);
        assert!(neighbors.contains(&VoxelCoord::new(6, 5, 5)));
        assert!(neighbors.contains(&VoxelCoord::new(4, 5, 5)));
        assert!(neighbors.contains(&VoxelCoord::new(5, 6, 5)));
        assert!(neighbors.contains(&VoxelCoord::new(5, 4, 5)));
        assert!(neighbors.contains(&VoxelCoord::new(5, 5, 6)));
        assert!(neighbors.contains(&VoxelCoord::new(5, 5, 4)));
    }

    #[test]
    fn test_all_neighbors() {
        let coord = VoxelCoord::new(5, 5, 5);
        let neighbors = coord.all_neighbors();
        assert_eq!(neighbors.len(), 26);
        // Should not contain self
        assert!(!neighbors.contains(&coord));
        // Should contain face neighbors
        assert!(neighbors.contains(&VoxelCoord::new(6, 5, 5)));
        // Should contain edge neighbors
        assert!(neighbors.contains(&VoxelCoord::new(6, 6, 5)));
        // Should contain corner neighbors
        assert!(neighbors.contains(&VoxelCoord::new(6, 6, 6)));
    }

    #[test]
    fn test_manhattan_distance() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(3, 4, 5);
        assert_eq!(a.manhattan_distance(b), 12);
        assert_eq!(b.manhattan_distance(a), 12);
    }

    #[test]
    fn test_manhattan_distance_negative() {
        let a = VoxelCoord::new(-5, -5, -5);
        let b = VoxelCoord::new(5, 5, 5);
        assert_eq!(a.manhattan_distance(b), 30);
    }

    #[test]
    fn test_chebyshev_distance() {
        let a = VoxelCoord::new(0, 0, 0);
        let b = VoxelCoord::new(3, 4, 5);
        assert_eq!(a.chebyshev_distance(b), 5);
        assert_eq!(b.chebyshev_distance(a), 5);
    }

    #[test]
    fn test_chebyshev_distance_negative() {
        let a = VoxelCoord::new(-5, -5, -5);
        let b = VoxelCoord::new(5, 5, 5);
        assert_eq!(a.chebyshev_distance(b), 10);
    }

    #[test]
    fn test_checked_add() {
        let coord = VoxelCoord::new(5, 5, 5);
        assert_eq!(
            coord.checked_add(VoxelCoord::new(1, 2, 3)),
            Some(VoxelCoord::new(6, 7, 8))
        );
    }

    #[test]
    fn test_checked_add_overflow() {
        let coord = VoxelCoord::new(i32::MAX, 0, 0);
        assert_eq!(coord.checked_add(VoxelCoord::new(1, 0, 0)), None);
    }

    #[test]
    fn test_checked_sub() {
        let coord = VoxelCoord::new(5, 5, 5);
        assert_eq!(
            coord.checked_sub(VoxelCoord::new(1, 2, 3)),
            Some(VoxelCoord::new(4, 3, 2))
        );
    }

    #[test]
    fn test_checked_sub_overflow() {
        let coord = VoxelCoord::new(i32::MIN, 0, 0);
        assert_eq!(coord.checked_sub(VoxelCoord::new(1, 0, 0)), None);
    }

    #[test]
    fn test_add_operator() {
        let a = VoxelCoord::new(1, 2, 3);
        let b = VoxelCoord::new(4, 5, 6);
        assert_eq!(a + b, VoxelCoord::new(5, 7, 9));
    }

    #[test]
    fn test_sub_operator() {
        let a = VoxelCoord::new(5, 7, 9);
        let b = VoxelCoord::new(4, 5, 6);
        assert_eq!(a - b, VoxelCoord::new(1, 2, 3));
    }

    #[test]
    fn test_neg_operator() {
        let a = VoxelCoord::new(1, -2, 3);
        assert_eq!(-a, VoxelCoord::new(-1, 2, -3));
    }

    #[test]
    fn test_from_tuple() {
        let coord: VoxelCoord = (1, 2, 3).into();
        assert_eq!(coord, VoxelCoord::new(1, 2, 3));
    }

    #[test]
    fn test_from_array() {
        let coord: VoxelCoord = [1, 2, 3].into();
        assert_eq!(coord, VoxelCoord::new(1, 2, 3));
    }

    #[test]
    fn test_into_tuple() {
        let coord = VoxelCoord::new(1, 2, 3);
        let tuple: (i32, i32, i32) = coord.into();
        assert_eq!(tuple, (1, 2, 3));
    }

    #[test]
    fn test_into_array() {
        let coord = VoxelCoord::new(1, 2, 3);
        let array: [i32; 3] = coord.into();
        assert_eq!(array, [1, 2, 3]);
    }

    #[test]
    fn test_equality() {
        let a = VoxelCoord::new(1, 2, 3);
        let b = VoxelCoord::new(1, 2, 3);
        let c = VoxelCoord::new(1, 2, 4);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn test_hash() {
        use std::collections::HashSet;
        let mut set = HashSet::new();
        set.insert(VoxelCoord::new(1, 2, 3));
        set.insert(VoxelCoord::new(1, 2, 3)); // Duplicate
        set.insert(VoxelCoord::new(4, 5, 6));
        assert_eq!(set.len(), 2);
    }

    #[test]
    fn test_default() {
        let coord = VoxelCoord::default();
        assert_eq!(coord, VoxelCoord::origin());
    }

    #[test]
    fn test_clone() {
        let coord = VoxelCoord::new(1, 2, 3);
        let cloned = coord;
        assert_eq!(coord, cloned);
    }

    #[test]
    fn test_debug() {
        let coord = VoxelCoord::new(1, 2, 3);
        let debug_str = format!("{coord:?}");
        assert!(debug_str.contains('1'));
        assert!(debug_str.contains('2'));
        assert!(debug_str.contains('3'));
    }
}
