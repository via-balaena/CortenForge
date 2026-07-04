//! Plain data types produced/consumed by [`crate::StaggeredCoupling`]: the
//! per-step [`CoupledStep`] readout, the [`TrajectoryPeakPressure`] and
//! [`GripRolloutFrame`] rollout readouts, the private design+policy tape, and the
//! [`PolicyState`] a [`crate::DiffPolicy`] observes.

use sim_ml_chassis::{Tape, Var};
use sim_soft::Vec3;

/// Result of one coupled step.
#[derive(Clone, Copy, Debug)]
pub struct CoupledStep {
    /// Total contact force the soft body exerts (the reaction on the rigid body
    /// is its negation). In newtons, world frame.
    pub force_on_soft: Vec3,
    /// Current height of the contacting rigid body's reference point.
    pub rigid_z: f64,
    /// Peak contact *pressure* this step (Pa) — the max per-contact-face stress
    /// over the active pairs ([`peak_contact_pressure`](sim_soft::peak_contact_pressure)). The measured local
    /// concentration that total `force_on_soft` cannot see: a finite sphere
    /// reads a high peak pressure at low total force, a broad slab the reverse.
    /// `0.0` with no contact; `f64::NAN` if every active contact is degenerate
    /// (the deliberate off-nominal sentinel — see [`peak_contact_pressure`](sim_soft::peak_contact_pressure)). A
    /// consumer reading this per-step field directly must filter non-finite
    /// values (the [`TrajectoryPeakPressure`] reduction already does), exactly as
    /// for the per-pair [`sim_soft::ContactPairReadout::pressure`] it reduces.
    pub peak_pressure: f64,
}

/// Trajectory peak-pressure readout from
/// [`StaggeredCoupling::coupled_trajectory_peak_pressure`](crate::StaggeredCoupling::coupled_trajectory_peak_pressure) — the measured
/// contrast between local concentration (pressure) and total load (force)
/// over a coupled impact rollout.
#[derive(Clone, Copy, Debug)]
pub struct TrajectoryPeakPressure {
    /// Max per-face contact pressure (Pa) over the rollout — the worst local
    /// concentration any contact face saw (max of each step's
    /// [`CoupledStep::peak_pressure`], ignoring non-finite/degenerate steps).
    pub peak_pressure: f64,
    /// Max total contact force magnitude on the soft body (N) over the rollout
    /// — the worst *total* load, the quantity a force-only readout reports.
    pub peak_total_force: f64,
    /// Step index at which [`Self::peak_pressure`] occurred, or `None` if no
    /// finite contact pressure was ever recorded (no contact, or every contact
    /// degenerate) — distinct from `Some(0)`, which is a real first-step peak.
    pub peak_step: Option<usize>,
}

/// One captured frame of a closed-loop **friction-grip** rollout — the per-step
/// state a viewer replays (see
/// [`StaggeredCoupling::coupled_trajectory_policy_gripped_capture`](crate::StaggeredCoupling::coupled_trajectory_policy_gripped_capture)).
///
/// The capturing rollout shares its loop body with the scalar forward
/// [`StaggeredCoupling::coupled_trajectory_policy_gripped_x`](crate::StaggeredCoupling::coupled_trajectory_policy_gripped_x) (the FD oracle for the
/// design+policy-friction gradient), so the animated scene is the *same* physics the
/// gradient differentiates — not a re-derivation. One frame is emitted before the
/// first step (the rest state) and one after each step, so a rollout of `n_steps`
/// yields `n_steps + 1` frames.
///
/// **Framing.** A frame pairs the post-step `soft_positions` and contact `fist_center`
/// (both from this step's *contact* configuration — the correct staggered pairing: the
/// soft solve responds to the step-start sphere) with the post-step rigid pose
/// (`arm_pivot`, `arm_tip`). Render the swinging limb as the segment `arm_pivot →
/// arm_tip` (the pivot is the fixed hinge for the grip scene; the tip swings). For the
/// centroid grip the `fist_center` is static over the block (the contact is abstracted
/// to the block centroid — the moving-end-effector follow-on would instead pose it at
/// `arm_tip`), so draw it as the abstracted contact patch, not the limb's end.
#[derive(Clone, Debug)]
pub struct GripRolloutFrame {
    /// Deformed soft-body vertex positions after this step, flat
    /// `[x0, y0, z0, x1, y1, z1, …]` (length `3 · n_vertices`). Pair with
    /// [`StaggeredCoupling::soft_boundary_faces`](crate::StaggeredCoupling::soft_boundary_faces) (the fixed rest topology) to build
    /// the surface mesh.
    pub soft_positions: Vec<f64>,
    /// The gripped rigid body's world **origin** (`data().xpos[body]`) after this step
    /// — the limb's anchor (the fixed hinge pivot for the grip scene). Pair with
    /// `arm_tip` to draw the limb.
    pub arm_pivot: [f64; 3],
    /// The gripped rigid body's world **inertial origin** (`data().xipos[body]`) after
    /// this step — the swinging end of the limb, the same quantity the scalar forward
    /// tracks (`tip_x = xipos[body].x`). Render the limb as `arm_pivot → arm_tip`.
    pub arm_tip: [f64; 3],
    /// The contact sphere's world centre this step — the block centroid (centroid
    /// grip) or the arm-tip geom (moving end-effector). This is what the soft body
    /// actually grips; render it as the fist. Sphere-only: a plane collider has no
    /// fist, and this defaults to the block-centre point on the contact plane.
    pub fist_center: [f64; 3],
    /// The gripped limb's primary joint coordinate `qpos[0]` this step — the hinge angle
    /// the holding objective regulates (and the FD oracle for
    /// [`StaggeredCoupling::coupled_trajectory_design_policy_hold_gradient`](crate::StaggeredCoupling::coupled_trajectory_design_policy_hold_gradient)).
    pub qpos0: f64,
}

/// One built design+policy friction-grip tape, ready to seed with an objective.
///
/// The expensive per-step coupled machinery — the #406 design+policy-friction tape (soft
/// re-equilibration, contact + friction wrench nodes, the closed-loop state→control recurrence,
/// the `μ`/`θ` leaves) — is shared by every objective; only the *seeding* differs. The
/// terminal-outcome objective seeds `tip_x` (via `s_final`/`jx_final`); a trajectory-integrated
/// objective seeds a cost summed over `qpos_steps` (the per-step policy-observed hinge angle). Built
/// by `build_design_policy_tape`; consumed by the public design+policy gradients.
pub(super) struct DesignPolicyTape {
    pub(super) tape: Tape,
    /// The material design leaf (`μ`, with the `λ = 4μ` tie folded into its grip node).
    pub(super) p_var: Var,
    /// The policy parameter leaves (`θ`), shared across steps.
    pub(super) param_vars: Vec<Var>,
    /// The policy-observed hinge angle `qpos[0]` at the start of each step — the leaf a
    /// trajectory-integrated objective (e.g. a holding cost) regulates.
    pub(super) qpos_steps: Vec<Var>,
    /// The terminal carried state (`[qpos; qvel]`), the parent of the terminal-`tip_x` seam node.
    pub(super) s_final: Var,
    /// `∂tip_x/∂state` at the terminal config (the linear COM-Jacobian row), for `PoseSeamVjp`.
    pub(super) jx_final: Vec<f64>,
    /// The terminal tangential drag `tip_x = xipos[body].x` (the terminal-outcome value).
    pub(super) tip_x: f64,
}

/// The state a [`DiffPolicy`](crate::DiffPolicy) observes each step — two loop-carried chassis-tape scalar [`Var`]s.
/// On the free platen these are the platen height `z` and vertical velocity `vz`
/// ([`StaggeredCoupling::coupled_trajectory_policy_gradient`](crate::StaggeredCoupling::coupled_trajectory_policy_gradient)); on the articulated grip they are
/// the hinge's joint angle and rate (`s[0]`, `s[nv]`,
/// [`StaggeredCoupling::coupled_trajectory_policy_friction_gradient`](crate::StaggeredCoupling::coupled_trajectory_policy_friction_gradient)). The fields are generic
/// position/velocity-style observations — the `z`/`vz` names are the platen legacy. Richer
/// observations (contact force, soft-state summaries) are a documented follow-on; they would add
/// fields here.
#[derive(Clone, Copy, Debug)]
pub struct PolicyState {
    /// The position-like observation at the step start (platen height `z`, or joint angle) — a
    /// `[1]`-shaped var.
    pub z: Var,
    /// The velocity-like observation at the step start (platen `vz`, or joint rate) — a
    /// `[1]`-shaped var.
    pub vz: Var,
}
