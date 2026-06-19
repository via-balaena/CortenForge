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
    add_ball_joint, add_body, add_freejoint, add_ground_plane, add_hinge_joint, add_site,
    add_slide_joint, add_sphere_geom, finalize,
};
use crate::types::enums::{Integrator, TendonType, WrapType};
use crate::types::{Contact, Model};

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

// ════════════════════════════════════════════════════════════════════════
// Contact / constraint conformance (GPU constraint slice)
// ════════════════════════════════════════════════════════════════════════
//
// The smooth/kinematic matrices above feed the contact-free GPU↔CPU suites.
// The contact path can't be compared by just running both collision pipelines:
// GPU collision represents geoms as SDF grids (many per-cell contacts) while CPU
// uses analytic geoms (one contact), so the two engines never agree on the
// contact SET for the same geometry — a collision-layer divergence that would
// swamp the constraint assembly/solver divergences we actually want to expose
// (pyramidal R-scaling, Newton solver).
//
// So the CPU runs its ANALYTIC collision once (a fully self-consistent
// `forward()` — the production path), and the SAME generated contacts are
// injected into the GPU's `contact_buffer` while the GPU's own collision is
// skipped. [`ContactSpec`] carries each contact across the boundary; the CPU
// oracle is the reference and the GPU consumes its `contacts`, so neither side
// can drift, and the comparison isolates assembly + solve. (Collision
// conformance — SDF dedup, per-cell caps — is a separate later slice.) See
// `project-gpu-shader-conformance-gap` (contact/constraint slice).

/// One contact, expressed independently of either engine, carrying it from the
/// CPU analytic collision to the GPU `contact_buffer`.
///
/// `geom1`/`geom2` index into the case model's geoms so the GPU can resolve the
/// per-geom `condim` (`assemble.wgsl`); the normal points `geom1 → geom2`.
#[derive(Debug, Clone)]
pub struct ContactSpec {
    /// World-space contact position.
    pub pos: [f64; 3],
    /// World-space unit normal (`geom1 → geom2`).
    pub normal: [f64; 3],
    /// Penetration depth (positive = overlap).
    pub depth: f64,
    /// Combined sliding friction coefficient (`mu[0]`). `0.0` for frictionless.
    pub mu: f64,
    /// Combined torsional friction coefficient (`mu[2]`). Drives the condim=4
    /// torsional facet Jacobian on both engines; `0.0` (or unused) below condim=4.
    pub mu_torsion: f64,
    /// Contact dimension: 1 (frictionless, 1 row), 3 (pyramidal sliding, 4 rows),
    /// or 4 (pyramidal sliding + torsional, 6 rows).
    pub condim: usize,
    /// First geom index.
    pub geom1: usize,
    /// Second geom index.
    pub geom2: usize,
}

impl ContactSpec {
    /// Capture a CPU-generated [`Contact`] for GPU injection.
    fn from_contact(c: &Contact) -> Self {
        Self {
            pos: [c.pos.x, c.pos.y, c.pos.z],
            normal: [c.normal.x, c.normal.y, c.normal.z],
            depth: c.depth,
            mu: c.mu[0],
            mu_torsion: c.mu[2],
            condim: c.dim,
            geom1: c.geom1,
            geom2: c.geom2,
        }
    }
}

/// A contact conformance case: a free body resting (penetrating) on a plane.
///
/// The contact SET is produced by the CPU oracle's analytic collision (not
/// hand-authored) and returned in [`ConstraintOracle`] for the GPU to inject.
#[derive(Debug, Clone)]
pub struct ContactConformanceCase {
    /// Stable case label (used in test failure messages).
    pub name: &'static str,
    /// The model under test (free body + ground plane + sphere geom(s)).
    pub model: Model,
    /// Generalized position operating point (length `model.nq`).
    pub qpos: Vec<f64>,
    /// Generalized velocity operating point (length `model.nv`).
    pub qvel: Vec<f64>,
}

/// CPU reference outputs for the constraint stage, plus the generated contacts.
///
/// `efc_d`/`efc_aref` are the per-row assembly outputs (the **assembly**
/// comparison channel — where pyramidal R-scaling / aref divergences live);
/// `qacc`/`qfrc_constraint` are the post-solve nv-vectors (the **solve**
/// channel — where the Newton-solver divergences live). The nv-vectors are
/// row-order-independent; the per-row vectors are compared as a sorted multiset
/// (a single contact's facet rows must align, but the harness does not assume a
/// fixed facet order). `contacts` is the analytic contact set the GPU injects so
/// both engines solve the SAME contacts.
#[derive(Debug, Clone)]
pub struct ConstraintOracle {
    /// Number of assembled constraint rows (`efc_D.len()`).
    pub n_rows: usize,
    /// Per-row constraint diagonal `D = 1/R` (GPU buffer: `efc_d`).
    pub efc_d: Vec<f64>,
    /// Per-row reference acceleration (GPU buffer: `efc_aref`).
    pub efc_aref: Vec<f64>,
    /// Post-solve generalized acceleration (length `nv`).
    pub qacc: Vec<f64>,
    /// Post-solve constraint force in joint space (length `nv`).
    pub qfrc_constraint: Vec<f64>,
    /// The CPU-generated contact set, for identical GPU injection.
    pub contacts: Vec<ContactSpec>,
}

/// Build a contact conformance fixture: a free rigid body with `n_contacts`
/// sphere geoms penetrating a ground plane.
///
/// `impratio` and `mu` are set so the **pyramidal** cases unambiguously expose
/// the GPU R-scaling defect: CPU computes `Rpy = 2·mu_reg²·R[first]` with
/// `mu_reg = mu·√(1/impratio)`, while the GPU does not apply that facet scaling
/// — the gap is well above tolerance for `mu ≠ 1`, `impratio ≠ 1`.
/// `diagapprox_bodyweight` is enabled so the *base* diagonal matches the GPU's
/// bodyweight approximation (a documented MuJoCo option, not a port bug),
/// leaving R-scaling as the sole `efc_d` divergence. `mu_torsion` sets the
/// torsional friction coefficient (`geom_friction.y`); it only matters for the
/// condim=4 torsional facets — below condim=4 it is inert (no torsional rows).
fn contact_fixture(
    name: &'static str,
    condim: i32,
    mu: f64,
    mu_torsion: f64,
    impratio: f64,
    n_contacts: usize,
) -> ContactConformanceCase {
    const RADIUS: f64 = 0.1;
    const ORIGIN_Z: f64 = 0.08; // < RADIUS ⇒ each sphere penetrates the z=0 plane

    let mut m = Model::empty();
    m.gravity = Vector3::new(0.0, 0.0, -9.81);
    m.timestep = 0.002;
    m.integrator = Integrator::Euler; // explicit Euler — matches the GPU integrate stage
    m.diagapprox_bodyweight = true; // match the GPU bodyweight diagonal approximation
    m.impratio = impratio;
    m.cone = 0; // pyramidal friction cone (MuJoCo default; the GPU path assumes it)

    // Ground plane on the world body (geom 0).
    let g_plane = add_ground_plane(&mut m);

    // A free body holding the sphere(s); its origin sits below the sphere radius
    // so each sphere penetrates the z=0 plane (depth = RADIUS − ORIGIN_Z).
    let it = 0.02;
    let b = add_body(
        &mut m,
        0,
        "puck",
        Vector3::new(0.0, 0.0, ORIGIN_Z),
        1.0,
        // Distinct principal moments so every angular DOF is observable.
        Vector3::new(it, 1.3 * it, 0.7 * it),
        Vector3::zeros(),
    );
    add_freejoint(&mut m, b, "free");

    // Spheres: a single one centered, or two offset along x (two contacts couple
    // the DOFs enough to drive the Newton solver off its fixed line-search grid).
    let xs: &[f64] = if n_contacts == 1 {
        &[0.0]
    } else {
        &[-0.15, 0.15]
    };
    let mut geoms = vec![g_plane];
    for &x in xs.iter().take(n_contacts) {
        let g = add_sphere_geom(
            &mut m,
            b,
            Some("puck_geom"),
            Vector3::new(x, 0.0, 0.0),
            RADIUS,
            true,
        );
        geoms.push(g);
    }

    // condim drives the pyramidal row count; equal slide friction on both geoms
    // makes the combined contact friction unambiguously `mu`.
    //
    // INVARIANT for valid GPU↔CPU comparison: set the SAME `condim` on both
    // geoms. CPU combines a pair's condim with `max`, the GPU with `min`; they
    // agree only when equal. A future fixture with differing per-geom condim
    // would diverge on row count from the combination rule alone — a phantom
    // divergence, not a real GPU gap. Likewise these fixtures use an axis-aligned
    // contact normal (sphere-on-plane → (0,0,1)); a tilted normal would expose a
    // CPU/GPU tangent-FRAME convention difference that the sorted-multiset
    // compare cannot absorb (it tolerates facet swap/negate, not a frame
    // rotation), so reconcile the frame convention before adding tilted contacts.
    for &g in &geoms {
        m.geom_condim[g] = condim;
        m.geom_friction[g] = Vector3::new(mu, mu_torsion, 0.0001);
    }
    finalize(&mut m);
    // `builders::finalize` does NOT compute `invweight0` (the smooth/damped
    // fixtures never touch it). The constraint diagonal approximation — on BOTH
    // engines — divides by `body_invweight0`; leaving it zero clamps R to MINVAL,
    // giving a degenerate D≈1/MINVAL that matches CPU↔GPU by accident yet blows
    // the solver up. Compute it so the diagonal is physical.
    m.compute_invweight0();

    // Operating point: body at rest height, with translational (normal +
    // tangential) and rotational velocity so the normal AND friction rows carry
    // live `aref`/`jar` velocity terms.
    let qpos = vec![0.0, 0.0, ORIGIN_Z, 1.0, 0.0, 0.0, 0.0];
    let qvel = vec![0.3, -0.2, -0.5, 0.1, 0.05, -0.08];

    ContactConformanceCase {
        name,
        model: m,
        qpos,
        qvel,
    }
}

/// Build the contact conformance matrix.
///
/// Four fixtures, smallest-discriminating-first:
/// - `contact_normal_only` (condim=1, 1 row): the frictionless baseline — no
///   pyramidal scaling, so `efc_d` should MATCH; isolates aref + the
///   single-row solve.
/// - `contact_pyramidal` (condim=3, 4 rows): the R-scaling discriminator
///   (`mu = 0.8`, `impratio = 2.0`).
/// - `contact_pyramidal_2pt` (two condim=3 contacts, 8 rows): the Newton-solver
///   discriminator — coupled contacts force off-grid line-search steps.
/// - `contact_torsional` (condim=4, 6 rows): adds the torsional facet pair. Its
///   `efc_d` R-scaling exposes whether the GPU uses `mu[0]` (slide) or `mu[2]`
///   (torsion) for the facet `mu_reg` — CPU uses `mu[0]` for ALL facets
///   (`postprocess_pyramidal_r_scaling`), so the torsional R-override must too.
#[must_use]
pub fn contact_conformance_matrix() -> Vec<ContactConformanceCase> {
    vec![
        contact_fixture("contact_normal_only", 1, 0.0, 0.005, 1.0, 1),
        contact_fixture("contact_pyramidal", 3, 0.8, 0.005, 2.0, 1),
        contact_fixture("contact_pyramidal_2pt", 3, 0.8, 0.005, 2.0, 2),
        contact_fixture("contact_torsional", 4, 0.8, 0.3, 2.0, 1),
    ]
}

/// Compute the CPU reference for a contact conformance case.
///
/// One standard `forward()` runs analytic collision → constraint assembly →
/// solve in a single, fully self-consistent pass (the production path, so
/// `qacc = qacc_smooth + M⁻¹·qfrc_constraint` holds exactly). The generated
/// contacts are captured for identical GPU injection.
///
/// # Panics
///
/// Panics if `forward()` fails on the fixture (a malformed fixture is a test
/// bug — fail loudly rather than return a silently-degraded oracle).
#[must_use]
// Test-fixture helper: a fixture that cannot forward is a bug to surface, not a
// recoverable condition, so `expect` is the right contract here.
#[allow(clippy::expect_used)]
pub fn contact_constraint_oracle(case: &ContactConformanceCase) -> ConstraintOracle {
    let model = &case.model;
    let mut data = model.make_data();
    data.qpos.as_mut_slice().copy_from_slice(&case.qpos);
    data.qvel.as_mut_slice().copy_from_slice(&case.qvel);
    data.forward(model).expect("contact oracle: forward");

    ConstraintOracle {
        n_rows: data.efc_D.len(),
        efc_d: data.efc_D.clone(),
        efc_aref: data.efc_aref.iter().copied().collect(),
        qacc: data.qacc.iter().copied().collect(),
        qfrc_constraint: data.qfrc_constraint.iter().copied().collect(),
        contacts: data
            .contacts
            .iter()
            .map(ContactSpec::from_contact)
            .collect(),
    }
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

    /// Contact fixtures are well-formed: operating points sized to `nq`/`nv`, a
    /// ground plane plus sphere geom(s).
    #[test]
    fn contact_fixtures_well_formed() {
        for case in super::contact_conformance_matrix() {
            assert_eq!(case.qpos.len(), case.model.nq, "{}: qpos len", case.name);
            assert_eq!(case.qvel.len(), case.model.nv, "{}: qvel len", case.name);
            assert!(
                case.model.ngeom >= 2,
                "{}: expected plane + sphere(s)",
                case.name
            );
        }
    }

    /// The oracle's analytic collision generates the expected contacts and the
    /// solve produces a live, SELF-CONSISTENT response: condim=1 → 1 row,
    /// condim=3 → 4 rows per contact; the contact holds the body
    /// (`qacc ≈ qacc_smooth + M⁻¹·qfrc_constraint`, here a nonzero normal force
    /// against gravity). Guards against a vacuous GPU comparison and against the
    /// stale-`qacc` trap that an out-of-band constraint re-solve would create.
    #[test]
    fn contact_oracle_rows_and_response() {
        let expected = [
            ("contact_normal_only", 1usize, 1usize),
            ("contact_pyramidal", 1, 4),
            ("contact_pyramidal_2pt", 2, 8),
            ("contact_torsional", 1, 6),
        ];
        for case in super::contact_conformance_matrix() {
            let oracle = super::contact_constraint_oracle(&case);
            let (want_contacts, want_rows) = expected
                .iter()
                .find(|(n, _, _)| *n == case.name)
                .map(|&(_, c, r)| (c, r))
                .expect("known case");
            assert_eq!(oracle.contacts.len(), want_contacts, "{}: ncon", case.name);
            assert_eq!(oracle.n_rows, want_rows, "{}: row count", case.name);
            assert_eq!(oracle.efc_d.len(), want_rows, "{}: efc_d len", case.name);
            assert_eq!(
                oracle.efc_aref.len(),
                want_rows,
                "{}: efc_aref len",
                case.name
            );
            // Penetrating contact under gravity → a nonzero normal constraint
            // force (the vertical DOF is index 2 of the free joint).
            assert!(
                oracle.qfrc_constraint[2].abs() > 1e-6,
                "{}: expected nonzero normal constraint force, got {:?}",
                case.name,
                oracle.qfrc_constraint
            );
            assert!(
                oracle.efc_d.iter().all(|&d| d > 0.0),
                "{}: non-positive efc_d {:?}",
                case.name,
                oracle.efc_d
            );
        }
    }
}
