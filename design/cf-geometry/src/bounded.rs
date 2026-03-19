//! The [`Bounded`] trait for types that have a bounding box.

use crate::Aabb;

/// Types that have an axis-aligned bounding box.
///
/// Implemented by [`Aabb`], [`Sphere`](crate::Sphere), and (in later phases)
/// `IndexedMesh`, `Shape`, `ConvexHull`, `HeightFieldData`, `SdfGrid`.
pub trait Bounded {
    /// Returns the axis-aligned bounding box enclosing this geometry.
    fn aabb(&self) -> Aabb;
}
