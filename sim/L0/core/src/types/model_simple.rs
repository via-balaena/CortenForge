//! Simple bodies and the dof–dof sparsity patterns, as MuJoCo 3.5.0's
//! compiler derives them: which frames equal their body's frame or inertial
//! frame (`*_sameframe`), which bodies are simple (`body_simple`,
//! `dof_simplenum`), and the patterns of the mass matrix and
//! of the velocity derivative that the implicit integrators factor
//! (`mj_makeDofDofSparse`).

use nalgebra::{UnitQuaternion, Vector3};

use super::enums::{MjJointType, SameFrame};
use super::model::Model;

/// Two frames whose positions and quaternions differ by less than this in
/// every component are one frame to MuJoCo's compiler (`kFrameEps`,
/// `user_model.cc:67`).
const FRAME_EPS: f64 = 1e-6;

/// MuJoCo's `mjEPS`: a joint axis component above it in magnitude counts
/// toward the axis not being aligned with a coordinate axis.
const MJ_EPS: f64 = 1e-14;

/// `IsSameVec` (`user_model.cc:69-76`).
fn same_pos(a: &Vector3<f64>, b: &Vector3<f64>) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < FRAME_EPS)
}

/// `IsSameQuat`, either sign (`user_model.cc:90-104`).
fn same_quat(a: &UnitQuaternion<f64>, b: &UnitQuaternion<f64>) -> bool {
    let (a, b) = (a.coords, b.coords);
    let within = |sign: f64| (0..4).all(|k| (a[k] - sign * b[k]).abs() < FRAME_EPS);
    within(1.0) || within(-1.0)
}

fn null_pos(p: &Vector3<f64>) -> bool {
    same_pos(p, &Vector3::zeros())
}

fn null_quat(q: &UnitQuaternion<f64>) -> bool {
    same_quat(q, &UnitQuaternion::identity())
}

impl Model {
    /// Derive `body_sameframe`, `geom_sameframe`, `site_sameframe`,
    /// `body_simple` and `dof_simplenum` from the body, joint, geom and site
    /// frames, as MuJoCo's compiler does (`user_model.cc:2739-2860`,
    /// `:2899-2912`, `:2935-2948`, `:3942-3951`).
    ///
    /// A body is simple when its inertial frame is its frame, it is a root
    /// (or the child of a jointless child of the world), it has no child, and
    /// its joints are at its origin, each slide and hinge along a coordinate
    /// axis, with at most one hinge or ball and none before another joint.
    /// `body_simple` is 2 for a
    /// simple body whose joints are all slides, 1 for another simple body
    /// with a joint, and also 1 for a simple body with none. MuJoCo also
    /// demotes a body that a tendon with armature passes over; sim-core has
    /// no tendon armature.
    pub fn compute_simple_bodies(&mut self) {
        let frame = |pos: &Vector3<f64>, quat: &UnitQuaternion<f64>, body: usize| {
            if null_pos(pos) && null_quat(quat) {
                SameFrame::Body
            } else if null_quat(quat) {
                SameFrame::BodyRot
            } else if same_pos(pos, &self.body_ipos[body])
                && same_quat(quat, &self.body_iquat[body])
            {
                SameFrame::Inertia
            } else if same_quat(quat, &self.body_iquat[body]) {
                SameFrame::InertiaRot
            } else {
                SameFrame::None
            }
        };
        self.geom_sameframe = (0..self.ngeom)
            .map(|g| frame(&self.geom_pos[g], &self.geom_quat[g], self.geom_body[g]))
            .collect();
        self.site_sameframe = (0..self.nsite)
            .map(|s| frame(&self.site_pos[s], &self.site_quat[s], self.site_body[s]))
            .collect();

        self.body_sameframe = (0..self.nbody)
            .map(|b| {
                if null_pos(&self.body_ipos[b]) && null_quat(&self.body_iquat[b]) {
                    SameFrame::Body
                } else if null_quat(&self.body_iquat[b]) {
                    SameFrame::BodyRot
                } else {
                    SameFrame::None
                }
            })
            .collect();

        self.body_simple = vec![0; self.nbody];
        for b in 0..self.nbody {
            let parent = self.body_parent[b];
            let mut simple = self.body_sameframe[b] == SameFrame::Body
                && (self.body_rootid[b] == b
                    || (self.body_parent[parent] == 0 && self.body_dof_num[parent] == 0));
            // A parent body is never simple (unless the world).
            if parent > 0 {
                self.body_simple[parent] = 0;
            }
            let joints = self.body_jnt_adr[b]..self.body_jnt_adr[b] + self.body_jnt_num[b];
            let mut turned = false;
            for j in joints.clone() {
                let kind = self.jnt_type[j];
                let aligned = self.jnt_axis[j].iter().filter(|c| c.abs() > MJ_EPS).count() == 1;
                if turned
                    || !null_pos(&self.jnt_pos[j])
                    || (matches!(kind, MjJointType::Hinge | MjJointType::Slide) && !aligned)
                {
                    simple = false;
                }
                turned |= matches!(kind, MjJointType::Ball | MjJointType::Hinge);
            }
            self.body_simple[b] = match (simple, self.body_dof_num[b]) {
                (false, _) => 0,
                (true, 0) => 1,
                (true, _)
                    if joints
                        .into_iter()
                        .all(|j| self.jnt_type[j] == MjJointType::Slide) =>
                {
                    2
                }
                (true, _) => 1,
            };
        }

        // Each dof counts the simple dofs from it to the next one that is not
        // simple (`FinalizeSimple`).
        self.dof_simplenum = vec![0; self.nv];
        let mut count = 0;
        for i in (0..self.nv).rev() {
            count = if self.body_simple[self.dof_body[i]] == 0 {
                0
            } else {
                count + 1
            };
            self.dof_simplenum[i] = count;
        }
    }

    /// Lay out MuJoCo's two dof–dof sparsity patterns from `dof_parent` and
    /// `dof_simplenum` (`mj_makeDofDofSparse`, 3.5.0 `engine_io.c:695-781`):
    /// the mass matrix's lower triangle, a simple dof's row holding its
    /// diagonal only (`M_*`, `qH`'s layout), and every pair of dofs on one
    /// branch, both triangles (`D_*`, `qLU`'s layout). Each row's columns
    /// ascend.
    #[allow(non_snake_case)]
    pub fn compute_dof_patterns(&mut self) {
        let ancestors = |i: usize| {
            let mut out = Vec::new();
            let mut p = self.dof_parent[i];
            while let Some(j) = p {
                out.push(j);
                p = self.dof_parent[j];
            }
            out.reverse();
            out
        };

        let mut M_rows: Vec<Vec<usize>> = (0..self.nv)
            .map(|i| {
                let mut row = if self.dof_simplenum[i] == 0 {
                    ancestors(i)
                } else {
                    Vec::new()
                };
                row.push(i);
                row
            })
            .collect();

        let mut D_rows: Vec<Vec<usize>> = (0..self.nv).map(ancestors).collect();
        for i in 0..self.nv {
            for j in ancestors(i) {
                D_rows[j].push(i);
            }
        }
        for (i, row) in D_rows.iter_mut().enumerate() {
            row.push(i);
            row.sort_unstable();
        }
        self.D_diag = D_rows
            .iter()
            .enumerate()
            .map(|(i, row)| row.iter().position(|&c| c == i).unwrap_or(0))
            .collect();

        let flatten = |rows: &mut Vec<Vec<usize>>| {
            let rownnz: Vec<usize> = rows.iter().map(Vec::len).collect();
            let rowadr = rownnz
                .iter()
                .scan(0, |adr, &n| {
                    let start = *adr;
                    *adr += n;
                    Some(start)
                })
                .collect();
            (rownnz, rowadr, rows.drain(..).flatten().collect())
        };
        (self.M_rownnz, self.M_rowadr, self.M_colind) = flatten(&mut M_rows);
        (self.D_rownnz, self.D_rowadr, self.D_colind) = flatten(&mut D_rows);
    }
}
