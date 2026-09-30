//! The step log's host side (recon §17b): the rows each step writes on the
//! device, added at f64 when a read brings them back.
//!
//! Each `contact` writes a row of the contact forces' resultant, their moment
//! about the rest positions' centroid `c`, and the normal-force sum; each
//! `boundary_conditions` a row of the contact work and the damping loss. The
//! host keeps, for each contact row, the obstacle's origin `p`, move and turn
//! over that step from the f64 track, so the row's moment is moved to the
//! origin, `M = M_c + (c − p) × F`, and the obstacle's work formed, as the CPU
//! executor forms them each step.

use sim_soft_explicit::f64 as shared;

/// Floats in a contact row: the resultant, the moment about `c`, the normal
/// sum.
pub(super) const CONTACT_ROW: usize = 7;

/// Floats in a boundary row: the contact work, the damping loss.
pub(super) const BOUNDARY_ROW: usize = 2;

/// The obstacle's motion over one step, from its f64 track: its body origin
/// at the start, the origin's move, and the turn's rotation vector.
#[derive(Clone, Copy, Debug)]
pub(super) struct Motion {
    pub origin: [f64; 3],
    pub moved: [f64; 3],
    pub turn: [f64; 3],
}

/// The monitors the rows add to: some since the last monitor read, some
/// cumulative.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Totals {
    /// Since the last read: the resultant, the moment about the obstacle's
    /// origin at each step's start, the normal sum, and the steps.
    pub resultant: [f64; 3],
    pub moment: [f64; 3],
    pub normal: f64,
    pub steps: u64,
    /// Cumulative.
    pub obstacle_work: f64,
    pub contact_work: f64,
    pub damping_loss: f64,
}

impl Totals {
    /// Add contact rows, each with the motion kept at its step, the moment
    /// taken about `centroid`.
    pub(super) fn add_contact_rows(
        &mut self,
        rows: &[f32],
        motions: &[Motion],
        centroid: [f64; 3],
    ) {
        for (row, motion) in rows.chunks_exact(CONTACT_ROW).zip(motions) {
            let widened = |i: usize| [row[i], row[i + 1], row[i + 2]].map(f64::from);
            let force = widened(0);
            let arm = shared::vec3_sub(centroid, motion.origin);
            let moment = shared::vec3_add(widened(3), shared::vec3_cross(arm, force));
            self.obstacle_work +=
                shared::vec3_dot(force, motion.moved) + shared::vec3_dot(moment, motion.turn);
            self.resultant = shared::vec3_add(self.resultant, force);
            self.moment = shared::vec3_add(self.moment, moment);
            self.normal += f64::from(row[6]);
            self.steps += 1;
        }
    }

    /// Add boundary rows.
    pub(super) fn add_boundary_rows(&mut self, rows: &[f32]) {
        for row in rows.chunks_exact(BOUNDARY_ROW) {
            self.contact_work += f64::from(row[0]);
            self.damping_loss += f64::from(row[1]);
        }
    }

    /// Restart the ones since the last read.
    pub(super) const fn restart(&mut self) {
        self.resultant = [0.0; 3];
        self.moment = [0.0; 3];
        self.normal = 0.0;
        self.steps = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ★ A moment taken about the centroid and moved to the origin is the
    /// moment about the origin, and the work is the force on the move plus the
    /// moment on the turn. A lever arm dropped, or taken the wrong way round,
    /// moves the moment by `(c − p) × F`, which is not zero here.
    #[test]
    #[allow(clippy::cast_possible_truncation)]
    fn a_rows_moment_is_moved_to_the_origin_and_does_the_obstacles_work() {
        let force = [1.0, 2.0, -0.5];
        let (x, c, p) = ([0.3, -0.2, 0.9], [0.1, 0.4, -0.3], [2.0, -1.0, 0.5]);
        let about_c = shared::vec3_cross(shared::vec3_sub(x, c), force);
        let about_p = shared::vec3_cross(shared::vec3_sub(x, p), force);
        let row = [
            force[0], force[1], force[2], about_c[0], about_c[1], about_c[2], 3.0,
        ]
        .map(|v: f64| v as f32);
        let motion = Motion {
            origin: p,
            moved: [0.01, 0.0, -0.02],
            turn: [0.0, 0.003, 0.001],
        };
        let mut totals = Totals::default();
        totals.add_contact_rows(&row, &[motion, motion], c);
        let widened = row.map(f64::from);
        let expected = shared::vec3_add(
            [widened[3], widened[4], widened[5]],
            shared::vec3_cross(shared::vec3_sub(c, p), [widened[0], widened[1], widened[2]]),
        );
        for d in 0..3 {
            assert!((totals.moment[d] - expected[d]).abs() < 1e-12);
            assert!((totals.moment[d] - about_p[d]).abs() < 1e-6, "{d}");
        }
        let work = shared::vec3_dot([widened[0], widened[1], widened[2]], motion.moved)
            + shared::vec3_dot(expected, motion.turn);
        assert!((totals.obstacle_work - work).abs() < 1e-15);
        assert_eq!(totals.steps, 1, "one row, one step");
        assert!((totals.normal - 3.0).abs() < 1e-15);
    }
}
