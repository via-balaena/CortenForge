//! One row of MuJoCo's `xfrc_applied`: a force and a torque on one body.

use nalgebra::Vector3;

/// Force and torque applied to one body at its centre of mass (`xipos`),
/// both in the world frame: one row of MuJoCo's `mjData.xfrc_applied`, which
/// stores the force first (`mjdata.h:266`).
///
/// The halves are named fields, so they cannot be swapped by index; a row
/// copied from MuJoCo goes through [`BodyWrench::from_mujoco_row`].
///
/// ```
/// use nalgebra::Vector3;
/// use sim_core::{BodyWrench, Model};
///
/// let mut model = Model::free_body(2.0, Vector3::new(0.1, 0.1, 0.1));
/// model.gravity = Vector3::zeros();
/// let mut data = model.make_data();
/// data.xfrc_applied[1] = BodyWrench::from_mujoco_row([0.0, 0.0, 20.0, 0.0, 5.0, 0.0]);
/// data.forward(&model)?;
/// // F = ma on the free joint's linear DOFs, τ = Iα on its angular ones.
/// assert_eq!(data.qacc.as_slice(), &[0.0, 0.0, 10.0, 0.0, 50.0, 0.0]);
/// # Ok::<(), sim_core::StepError>(())
/// ```
///
/// Through 0.9 the row was a `Vector6` laid out torque first. None of the 0.9
/// forms compiles now, so old code cannot run with its halves swapped:
///
/// ```compile_fail,E0608
/// # let model = sim_core::Model::free_body(2.0, nalgebra::Vector3::new(0.1, 0.1, 0.1));
/// # let mut data = model.make_data();
/// data.xfrc_applied[1][5] = 1.0;
/// ```
///
/// ```compile_fail,E0308
/// # let model = sim_core::Model::free_body(2.0, nalgebra::Vector3::new(0.1, 0.1, 0.1));
/// # let mut data = model.make_data();
/// data.xfrc_applied[1] = nalgebra::Vector6::zeros();
/// ```
///
/// ```compile_fail,E0277
/// # let model = sim_core::Model::free_body(2.0, nalgebra::Vector3::new(0.1, 0.1, 0.1));
/// # let mut data = model.make_data();
/// data.xfrc_applied[1] = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0].into();
/// ```
///
/// ```compile_fail,E0277
/// # let model = sim_core::Model::free_body(2.0, nalgebra::Vector3::new(0.1, 0.1, 0.1));
/// # let mut data = model.make_data();
/// data.xfrc_applied[1] = nalgebra::Vector6::<f64>::zeros().into();
/// ```
// No `Index` or `Deref` (the first form above would compile) and no
// `From<[f64; 6]>` or `From<Vector6<f64>>` (the last two would): each would
// take a 0.9 row with its halves swapped.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BodyWrench {
    /// Force (N), world frame, applied at the body's centre of mass.
    pub force: Vector3<f64>,
    /// Torque (N·m), world frame.
    pub torque: Vector3<f64>,
}

impl BodyWrench {
    /// A wrench from its force and its torque.
    #[must_use]
    pub fn new(force: Vector3<f64>, torque: Vector3<f64>) -> Self {
        Self { force, torque }
    }

    /// MuJoCo's row `[fx, fy, fz, tx, ty, tz]`.
    #[must_use]
    pub fn from_mujoco_row(row: [f64; 6]) -> Self {
        Self {
            force: Vector3::new(row[0], row[1], row[2]),
            torque: Vector3::new(row[3], row[4], row[5]),
        }
    }

    /// This wrench as MuJoCo's row `[fx, fy, fz, tx, ty, tz]`.
    #[must_use]
    pub fn to_mujoco_row(&self) -> [f64; 6] {
        let (f, t) = (&self.force, &self.torque);
        [f.x, f.y, f.z, t.x, t.y, t.z]
    }

    /// Whether every bit of every component is zero, as MuJoCo's
    /// `mju_isZeroByte`: `-0.0` is not zero here.
    #[must_use]
    pub fn is_zero_bytes(&self) -> bool {
        self.components().all(|v| v.to_bits() == 0)
    }

    /// Whether every component equals zero, as MuJoCo's `mju_isZero`: `-0.0`
    /// is zero here.
    pub(crate) fn is_zero(&self) -> bool {
        self.components().all(|v| v == 0.0)
    }

    fn components(&self) -> impl Iterator<Item = f64> + '_ {
        self.force.iter().chain(self.torque.iter()).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mujoco_row_round_trips_force_first() {
        let row = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let w = BodyWrench::from_mujoco_row(row);
        assert_eq!(w.force, Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(w.torque, Vector3::new(4.0, 5.0, 6.0));
        assert_eq!(w.to_mujoco_row().map(f64::to_bits), row.map(f64::to_bits));
        assert_eq!(BodyWrench::new(w.force, w.torque), w);
    }

    #[test]
    fn negative_zero_is_zero_by_value_not_by_bytes() {
        let w = BodyWrench::default();
        assert!(w.is_zero() && w.is_zero_bytes());
        for k in 0..6 {
            let mut row = [0.0; 6];
            row[k] = -0.0;
            let w = BodyWrench::from_mujoco_row(row);
            assert!(w.is_zero(), "component {k}");
            assert!(!w.is_zero_bytes(), "component {k}");
        }
    }
}
