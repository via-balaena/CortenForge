//! Baking an exact distance function into the obstacle's grid, for the
//! fixtures' rigid bodies: the tube's mandrel and K6's cylinder.

use crate::f64::SdfGridLayout;

/// The distance `distance` baked into a grid of spacing `cell` over the
/// body-frame box `[low, high]`, in [`SdfGridLayout`]'s order.
///
/// # Errors
/// A [`BakeError`] unless `cell` is positive, the box is ordered on every
/// axis, each side is a whole number of cells, and the grid holds at most
/// `u32::MAX` samples.
pub fn bake(
    low: [f64; 3],
    high: [f64; 3],
    cell: f64,
    distance: impl Fn([f64; 3]) -> f64,
) -> Result<(SdfGridLayout, Vec<f64>), BakeError> {
    if !(cell.is_finite() && cell > 0.0) {
        return Err(BakeError::Cell);
    }
    let mut sizes = [0_u32; 3];
    for axis in 0..3 {
        let cells = (high[axis] - low[axis]) / cell;
        if !(cells.is_finite() && cells >= 0.0) {
            return Err(BakeError::Box);
        }
        if (cells - cells.round()).abs() > 1e-9 * cells.max(1.0) {
            return Err(BakeError::NotWholeCells { axis });
        }
        // A whole, non-negative number of cells; the product check below
        // bounds it.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let samples = cells.round().min(f64::from(u32::MAX - 1)) as u32 + 1;
        sizes[axis] = samples;
    }
    let total = sizes.iter().map(|&n| u64::from(n)).product::<u64>();
    if total > u64::from(u32::MAX) {
        return Err(BakeError::TooLarge);
    }
    let grid = SdfGridLayout {
        origin_x: low[0],
        origin_y: low[1],
        origin_z: low[2],
        cell_size: cell,
        size_x: sizes[0],
        size_y: sizes[1],
        size_z: sizes[2],
    };
    let mut values = Vec::with_capacity((grid.size_x * grid.size_y * grid.size_z) as usize);
    for k in 0..grid.size_z {
        for j in 0..grid.size_y {
            for i in 0..grid.size_x {
                values.push(distance([
                    low[0] + f64::from(i) * cell,
                    low[1] + f64::from(j) * cell,
                    low[2] + f64::from(k) * cell,
                ]));
            }
        }
    }
    Ok((grid, values))
}

/// Why a distance function could not be baked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum BakeError {
    /// The cell size is not positive and finite.
    #[error("the cell size must be positive and finite")]
    Cell,
    /// The box is not ordered (`low ≤ high`) or not finite.
    #[error("the box must be finite with low <= high on every axis")]
    Box,
    /// A side is not a whole number of cells, so the far face would miss
    /// `high`.
    #[error("the box's side along axis {axis} is not a whole number of cells")]
    NotWholeCells {
        /// The axis, 0 to 2.
        axis: usize,
    },
    /// The grid would hold more than `u32::MAX` samples.
    #[error("the grid would hold more than u32::MAX samples")]
    TooLarge,
}
