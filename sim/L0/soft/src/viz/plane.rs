//! Axis-aligned cutting plane shared by the analysis- and design-mesh cuts.

/// Axis-aligned cutting plane for [`slab_cut`](super::slab_cut) and [`design_slab_cut`](super::design_slab_cut).
///
/// `axis` is 0 (x), 1 (y), or 2 (z); `value` is the plane's
/// position along that axis. The plane normal points in the +`axis`
/// direction; "above" the plane means `position[axis] > value`.
///
/// Generalizing to arbitrary planes (point + normal) is a future
/// enhancement; F1.1 ships the axis-aligned form because every
/// existing call site cuts along an axis and the simpler API
/// dodges premature plane-frame plumbing.
#[derive(Clone, Copy, Debug)]
pub struct Plane {
    /// Cutting-plane axis (0 = x, 1 = y, 2 = z).
    pub axis: usize,
    /// Plane position along `axis`.
    pub value: f64,
}
