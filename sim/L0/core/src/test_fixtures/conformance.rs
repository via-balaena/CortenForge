//! Canonical joint × topology fixtures for differential-testing dynamics.
//!
//! This is the single source of truth for the conformance MATRIX that both the
//! CPU analytic-vs-FD transition harness
//! (`crate::derivatives::transition_matrix_harness`) and the GPU-vs-CPU
//! dynamics conformance suite (`sim-gpu`'s `pipeline::conformance_tests`) draw
//! from. Keeping one list means the two oracles cannot silently drift to
//! different model sets — a GPU shader bug masked by a missing topology is a
//! recurring failure mode (see `project-gpu-shader-conformance-gap`).
//!
//! The matrix sweeps joint types `{hinge, slide, ball, free}` across topologies
//! `{single, parallel/spatial chains, multi-joint-per-body, branched}` plus a
//! spatial-tendon-with-length-spring case, with **non-root ball/free** bodies
//! (`hinge → ball`, `hinge → free`) — the subspace/Coriolis gaps only bite once
//! the parent moves and the body's own velocity subspace rotates. Each model
//! ships with a non-degenerate operating point (`qpos`, `qvel`): off-axis COM,
//! distinct principal inertia, tilted quaternions, and nonzero velocities so
//! every Coriolis / gravity term is dynamically observable.

use nalgebra::{UnitQuaternion, Vector3};

use crate::test_fixtures::builders::{
    add_ball_joint, add_body, add_freejoint, add_hinge_joint, add_site, add_slide_joint, finalize,
};
use crate::types::Model;
use crate::types::enums::{TendonType, WrapType};

/// One matrix entry: a model plus an operating point.
///
/// The `(qpos, qvel)` point is where analytic, FD, and GPU dynamics are
/// compared. The CPU transition harness and the GPU conformance suite both
/// consume this shape.
pub struct ConformanceCase {
    /// Stable case label (used in test failure messages).
    pub name: &'static str,
    /// The model under test.
    pub model: Model,
    /// Generalized position operating point (length `model.nq`).
    pub qpos: Vec<f64>,
    /// Generalized velocity operating point (length `model.nv`).
    pub qvel: Vec<f64>,
}

/// Off-axis COM + distinct principal inertia so every rotational DOF is
/// dynamically observable (a symmetric, on-axis bob leaves the axial DOF
/// degenerate and hides ball/free Coriolis terms).
fn link_body(model: &mut Model, parent: usize, name: &str, pos: Vector3<f64>, len: f64) -> usize {
    let mass = 0.4;
    let it = mass * len * len / 12.0;
    add_body(
        model,
        parent,
        name,
        pos,
        mass,
        // Distinct, non-degenerate principal moments.
        Vector3::new(it, 0.7 * it, 0.45 * it),
        // COM off the link axis so orientation about all three axes matters.
        Vector3::new(0.05, 0.03, -len),
    )
}

/// A normalized quaternion (as `[w, x, y, z]`) tilted away from identity so
/// ball/free rotational DOFs sit at a generic, non-degenerate orientation.
fn tilted_quat() -> [f64; 4] {
    let q = UnitQuaternion::from_euler_angles(0.3, -0.25, 0.4);
    let c = q.coords; // (x, y, z, w)
    [c.w, c.x, c.y, c.z]
}

/// Append a spatial tendon (two-site path) with an always-active length
/// spring: the deadband `[0, 0]` keeps `length > upper` so the spring force
/// `k·(0 − length)` is live at every state, exercising the `(∂Jᵀ/∂q)·force`
/// cross-term in the position derivative.
fn add_spatial_tendon(model: &mut Model, site0: usize, site1: usize, stiffness: f64) {
    let adr = model.nwrap;
    model.tendon_adr.push(adr);
    model.tendon_num.push(2);
    model.tendon_type.push(TendonType::Spatial);
    model.tendon_stiffness.push(stiffness);
    model.tendon_damping.push(0.0);
    model.tendon_lengthspring.push([0.0, 0.0]);
    model.tendon_length0.push(0.0);
    model.tendon_limited.push(false);
    model.tendon_range.push((-1e10, 1e10));
    model.tendon_margin.push(0.0);
    model.tendon_frictionloss.push(0.0);
    model.tendon_solref_lim.push([0.02, 1.0]);
    model.tendon_solimp_lim.push([0.9, 0.95, 0.001, 0.5, 2.0]);
    model.tendon_solref_fri.push([0.02, 1.0]);
    model.tendon_solimp_fri.push([0.9, 0.95, 0.001, 0.5, 2.0]);
    model.tendon_group.push(0);
    model.tendon_rgba.push([0.5, 0.5, 0.5, 1.0]);
    model.tendon_treenum.push(0); // 0 → island/sleep code skips this tendon
    model.tendon_tree.push(0);
    model.tendon_tree.push(0);
    model.tendon_invweight0.push(0.0);
    model.tendon_name.push(Some("spring_tendon".to_string()));
    model.tendon_user.push(vec![]);

    for (ty, obj) in [(WrapType::Site, site0), (WrapType::Site, site1)] {
        model.wrap_type.push(ty);
        model.wrap_objid.push(obj);
        model.wrap_prm.push(0.0);
        model.wrap_sidesite.push(usize::MAX);
    }
    model.ntendon += 1;
    model.nwrap += 2;
}

/// Build the full conformance model matrix.
///
/// Topology names follow the completion-map spec; the `qpos`/`qvel` vectors are
/// sized to each model's `nq`/`nv` (quaternion joints get a tilted unit
/// quaternion in `qpos`).
///
/// The last case (`spatial_tendon_spring`) has no GPU shader counterpart — the
/// GPU conformance suite filters it out by name.
#[must_use]
pub fn dynamics_conformance_matrix() -> Vec<ConformanceCase> {
    let mut cases = Vec::new();

    // Distinct, non-parallel axes so every ancestor `âₖ × âⱼ` cross-term is
    // exercised (joint 0 stays Y to match the legacy regression).
    let ax = [
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 0.3, 0.0).normalize(),
        Vector3::new(0.2, 1.0, 0.1).normalize(),
    ];
    let l = 0.4;

    // ---- single-body roots ------------------------------------------------
    {
        // hinge_single (baseline — should always pass)
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_hinge_joint(&mut m, b, "h0", ax[0], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "hinge_single",
            model: m,
            qpos: vec![0.3],
            qvel: vec![0.8],
        });
    }
    {
        // hinge_offset_pivot — a hinge whose joint anchor is OFF the body frame
        // origin (`jnt_pos ≠ 0`). Then `r = xpos[b] − jpos_world ≠ 0`, so the
        // motion subspace has a nonzero lever (`S_lin = â×r`) and the same-body
        // ∂S/∂q term carries the `â×(â×r)` linear coupling. The builders default
        // `jnt_pos = 0` (every other case), so this is the only guard that the
        // `anc=b` term stays exact when that coupling is live.
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        let j = add_hinge_joint(&mut m, b, "h0", ax[1], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        m.jnt_pos[j] = Vector3::new(0.08, -0.05, 0.06);
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "hinge_offset_pivot",
            model: m,
            qpos: vec![0.3],
            qvel: vec![0.8],
        });
    }
    {
        // slide_single
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_slide_joint(&mut m, b, "s0", ax[1], 0.0, 0.0, 0.0);
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "slide_single",
            model: m,
            qpos: vec![0.2],
            qvel: vec![0.7],
        });
    }
    {
        // ball_root (tumbling) — root ball, parent (world) at rest
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_ball_joint(&mut m, b, "ball0");
        finalize(&mut m);
        let q = tilted_quat();
        cases.push(ConformanceCase {
            name: "ball_root",
            model: m,
            qpos: q.to_vec(),
            qvel: vec![0.5, -0.3, 0.7],
        });
    }
    {
        // free_root_tumbling — root free body translating AND rotating; the
        // `[0; ω×v]` Coriolis correction is its own-velocity gyroscopic term.
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_freejoint(&mut m, b, "free0");
        finalize(&mut m);
        let q = tilted_quat();
        cases.push(ConformanceCase {
            name: "free_root_tumbling",
            model: m,
            qpos: vec![0.1, -0.2, 0.3, q[0], q[1], q[2], q[3]],
            qvel: vec![0.6, -0.4, 0.5, 0.5, -0.3, 0.7],
        });
    }

    // ---- serial hinge chains (parallel + spatial) -------------------------
    for (name, spatial, n) in [
        ("hinge2_parallel", false, 2usize),
        ("hinge2_spatial", true, 2),
        ("hinge3_spatial", true, 3),
    ] {
        let mut m = Model::empty();
        let mut parent = 0;
        for i in 0..n {
            let pos = if i == 0 {
                Vector3::zeros()
            } else {
                Vector3::new(0.0, 0.0, -l)
            };
            let b = link_body(&mut m, parent, &format!("l{i}"), pos, l);
            let axis = if spatial { ax[i] } else { ax[0] };
            add_hinge_joint(
                &mut m,
                b,
                &format!("h{i}"),
                axis,
                0.0,
                0.0,
                0.0,
                false,
                (-3.0, 3.0),
            );
            parent = b;
        }
        finalize(&mut m);
        let qpos = vec![0.3, -0.4, 0.25][..n].to_vec();
        let qvel = vec![0.9, -1.2, 0.7][..n].to_vec();
        cases.push(ConformanceCase {
            name,
            model: m,
            qpos,
            qvel,
        });
    }

    // ---- non-root ball / free (parent moving) -----------------------------
    {
        // hinge → ball: the ball's same-body ∂S/∂q rotates with a moving parent
        let mut m = Model::empty();
        let b0 = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_hinge_joint(&mut m, b0, "h0", ax[1], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        let b1 = link_body(&mut m, b0, "l1", Vector3::new(0.0, 0.0, -l), l);
        add_ball_joint(&mut m, b1, "ball1");
        finalize(&mut m);
        let q = tilted_quat();
        cases.push(ConformanceCase {
            name: "hinge_then_ball",
            model: m,
            qpos: vec![0.3, q[0], q[1], q[2], q[3]],
            qvel: vec![0.9, 0.5, -0.3, 0.7],
        });
    }
    {
        // hinge → free: non-root free (both `ω×v` and same-body ∂S/∂q bite)
        let mut m = Model::empty();
        let b0 = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_hinge_joint(&mut m, b0, "h0", ax[1], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        let b1 = link_body(&mut m, b0, "l1", Vector3::new(0.0, 0.0, -l), l);
        add_freejoint(&mut m, b1, "free1");
        finalize(&mut m);
        let q = tilted_quat();
        cases.push(ConformanceCase {
            name: "hinge_then_free",
            model: m,
            qpos: vec![0.3, 0.1, -0.2, 0.3, q[0], q[1], q[2], q[3]],
            qvel: vec![0.9, 0.6, -0.4, 0.5, 0.5, -0.3, 0.7],
        });
    }

    {
        // hinge → hinge → free: the free body's parent ORIGIN moves under both
        // ancestor hinges, so the general free-child transport derivative
        // `∂r/∂q = −axis×(xpos[parent]−xanchor)` (parent origin moving, free body
        // origin fixed) is exercised with a genuinely nonzero value — unlike
        // `hinge_then_free` where the root hinge sits at the anchor (∂r = 0).
        let mut m = Model::empty();
        let b0 = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_hinge_joint(&mut m, b0, "h0", ax[1], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        let b1 = link_body(&mut m, b0, "l1", Vector3::new(0.0, 0.0, -l), l);
        add_hinge_joint(&mut m, b1, "h1", ax[0], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        let b2 = link_body(&mut m, b1, "l2", Vector3::new(0.0, 0.0, -l), l);
        add_freejoint(&mut m, b2, "free2");
        finalize(&mut m);
        let q = tilted_quat();
        cases.push(ConformanceCase {
            name: "double_hinge_then_free",
            model: m,
            qpos: vec![0.3, -0.4, 0.1, -0.2, 0.3, q[0], q[1], q[2], q[3]],
            qvel: vec![0.9, -0.7, 0.6, -0.4, 0.5, 0.5, -0.3, 0.7],
        });
    }

    // ---- multi-joint-per-body --------------------------------------------
    {
        // Two hinges on the SAME body (different axes): joint 1's subspace
        // rotates with joint 0's DOF — a same-body cross ∂S/∂q term.
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_hinge_joint(&mut m, b, "h0", ax[1], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        add_hinge_joint(&mut m, b, "h1", ax[2], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "multi_joint_body",
            model: m,
            qpos: vec![0.3, -0.35],
            qvel: vec![0.8, -0.6],
        });
    }

    // ---- branched all-hinge tree -----------------------------------------
    {
        // world → l0(hinge) → { l1(hinge), l2(hinge) }: two children share a
        // moving parent (exercises ancestor terms on a non-serial tree).
        let mut m = Model::empty();
        let b0 = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_hinge_joint(&mut m, b0, "h0", ax[0], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        let b1 = link_body(&mut m, b0, "l1", Vector3::new(0.0, 0.0, -l), l);
        add_hinge_joint(&mut m, b1, "h1", ax[1], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        let b2 = link_body(&mut m, b0, "l2", Vector3::new(0.1, 0.0, -l), l);
        add_hinge_joint(&mut m, b2, "h2", ax[2], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "branched_hinge",
            model: m,
            qpos: vec![0.3, -0.4, 0.25],
            qvel: vec![0.9, -1.2, 0.7],
        });
    }

    // ---- slide → hinge mixed chain ---------------------------------------
    {
        let mut m = Model::empty();
        let b0 = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_slide_joint(&mut m, b0, "s0", ax[1], 0.0, 0.0, 0.0);
        let b1 = link_body(&mut m, b0, "l1", Vector3::new(0.0, 0.0, -l), l);
        add_hinge_joint(&mut m, b1, "h1", ax[2], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "slide_then_hinge",
            model: m,
            qpos: vec![0.2, -0.35],
            qvel: vec![0.7, -0.6],
        });
    }

    // ---- spatial tendon with active length spring ------------------------
    {
        // hinge body, tendon from a world-fixed site to a body site: the
        // tendon Jacobian varies with the hinge angle, so the dropped
        // `(∂Jᵀ/∂q)·force` term is material. No GPU tendon shader exists, so
        // the GPU conformance suite filters this case out by name.
        let mut m = Model::empty();
        let world_site = add_site(&mut m, 0, "anchor", Vector3::new(0.3, 0.05, 0.1), 0.01);
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_hinge_joint(&mut m, b, "h0", ax[1], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        let body_site = add_site(&mut m, b, "tip", Vector3::new(0.0, 0.0, -l), 0.01);
        add_spatial_tendon(&mut m, world_site, body_site, 50.0);
        finalize(&mut m);
        m.compute_spatial_tendon_length0();
        cases.push(ConformanceCase {
            name: "spatial_tendon_spring",
            model: m,
            qpos: vec![0.4],
            qvel: vec![0.6],
        });
    }

    cases
}

/// Set uniform damping `d` on every DOF of every joint, in place, BEFORE
/// `finalize` expands it into `implicit_damping` (`jnt_damping` drives
/// hinge/slide DOFs, `dof_damping` drives ball/free DOFs — see
/// `Model::compute_implicit_params`).
fn damp_all(m: &mut Model, d: f64) {
    for j in 0..m.njnt {
        m.jnt_damping[j] = d;
        let adr = m.jnt_dof_adr[j];
        for i in 0..m.jnt_type[j].nv() {
            m.dof_damping[adr + i] = d;
        }
    }
}

/// Build the damped-integration conformance matrix.
///
/// The Slice-1 topologies but with nonzero joint/DOF damping, for comparing a
/// multi-step trajectory under the *implicit* Euler damping solve
/// (`(M + h·D)·q̈ = f`; see `integrate::Data::integrate`).
///
/// Damping-only (`stiffness = 0`) so the gap under test is unambiguously the
/// damping treatment, not a missing passive spring force. Every case carries
/// nonzero initial `qvel` so the damper force `−D·q̇` is live from step one.
/// Names carry a `_damped` suffix (distinct from [`dynamics_conformance_matrix`])
/// so the GPU suite can route them through a separate known-divergence entry.
#[must_use]
pub fn damped_conformance_matrix() -> Vec<ConformanceCase> {
    const D: f64 = 2.0;
    let ax = [
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 0.3, 0.0).normalize(),
        Vector3::new(0.2, 1.0, 0.1).normalize(),
    ];
    let l = 0.4;
    let mut cases = Vec::new();

    {
        // hinge_single_damped — 1-DOF, diagonal (M + h·D).
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_hinge_joint(&mut m, b, "h0", ax[0], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        damp_all(&mut m, D);
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "hinge_single_damped",
            model: m,
            qpos: vec![0.3],
            qvel: vec![0.8],
        });
    }
    {
        // slide_single_damped — prismatic damping.
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_slide_joint(&mut m, b, "s0", ax[1], 0.0, 0.0, 0.0);
        damp_all(&mut m, D);
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "slide_single_damped",
            model: m,
            qpos: vec![0.2],
            qvel: vec![0.7],
        });
    }
    {
        // ball_root_damped — 3 angular DOFs damped (dof_damping path).
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_ball_joint(&mut m, b, "ball0");
        damp_all(&mut m, D);
        finalize(&mut m);
        let q = tilted_quat();
        cases.push(ConformanceCase {
            name: "ball_root_damped",
            model: m,
            qpos: q.to_vec(),
            qvel: vec![0.5, -0.3, 0.7],
        });
    }
    {
        // free_tumbling_damped — full coupled 6×6 (M + h·D), translate + rotate.
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_freejoint(&mut m, b, "free0");
        damp_all(&mut m, D);
        finalize(&mut m);
        let q = tilted_quat();
        cases.push(ConformanceCase {
            name: "free_tumbling_damped",
            model: m,
            qpos: vec![0.1, -0.2, 0.3, q[0], q[1], q[2], q[3]],
            qvel: vec![0.6, -0.4, 0.5, 0.5, -0.3, 0.7],
        });
    }
    {
        // hinge3_spatial_damped — non-parallel 3-link chain; off-diagonal
        // (M + h·D) coupling that a per-DOF approximation would miss.
        let mut m = Model::empty();
        let mut parent = 0;
        for (i, &axis) in ax.iter().enumerate() {
            let pos = if i == 0 {
                Vector3::zeros()
            } else {
                Vector3::new(0.0, 0.0, -l)
            };
            let b = link_body(&mut m, parent, &format!("l{i}"), pos, l);
            add_hinge_joint(
                &mut m,
                b,
                &format!("h{i}"),
                axis,
                0.0,
                0.0,
                0.0,
                false,
                (-3.0, 3.0),
            );
            parent = b;
        }
        damp_all(&mut m, D);
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "hinge3_spatial_damped",
            model: m,
            qpos: vec![0.3, -0.4, 0.25],
            qvel: vec![0.9, -1.2, 0.7],
        });
    }
    {
        // multi_joint_body_damped — two hinges on one body; intra-body coupling.
        let mut m = Model::empty();
        let b = link_body(&mut m, 0, "l0", Vector3::zeros(), l);
        add_hinge_joint(&mut m, b, "h0", ax[1], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        add_hinge_joint(&mut m, b, "h1", ax[2], 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        damp_all(&mut m, D);
        finalize(&mut m);
        cases.push(ConformanceCase {
            name: "multi_joint_body_damped",
            model: m,
            qpos: vec![0.3, -0.35],
            qvel: vec![0.8, -0.6],
        });
    }

    cases
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]
    use super::{damped_conformance_matrix, dynamics_conformance_matrix};

    /// Both matrices are well-formed: operating points sized to `nq`/`nv`, and
    /// each model forward-evaluates without error (a CPU-side fixture sanity
    /// independent of the GPU suite that consumes them).
    #[test]
    fn conformance_fixtures_well_formed() {
        for case in dynamics_conformance_matrix()
            .into_iter()
            .chain(damped_conformance_matrix())
        {
            assert_eq!(case.qpos.len(), case.model.nq, "{}: qpos len", case.name);
            assert_eq!(case.qvel.len(), case.model.nv, "{}: qvel len", case.name);

            let mut data = case.model.make_data();
            data.qpos.as_mut_slice().copy_from_slice(&case.qpos);
            data.qvel.as_mut_slice().copy_from_slice(&case.qvel);
            data.forward(&case.model).expect("fixture forward");
        }
    }

    /// Every damped fixture carries nonzero `implicit_damping` — guards against a
    /// silently-undamped fixture making the GPU eulerdamp conformance vacuous.
    #[test]
    fn damped_fixtures_have_damping() {
        for case in damped_conformance_matrix() {
            let m = &case.model;
            assert!(
                (0..m.nv).any(|i| m.implicit_damping[i] > 0.0),
                "{}: expected nonzero implicit_damping",
                case.name
            );
        }
    }
}
