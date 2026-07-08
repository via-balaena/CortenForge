//! The **coupled** L4–L5 FSU: disc + ligaments + facet contact solved as ONE
//! quasi-static equilibrium.
//!
//! Rung 7 characterised the segment by *analytic superposition* — it imposed a
//! flexion angle, read each structure's restoring moment separately, and summed
//! them. This module instead ASSEMBLES the three structures into a single model and
//! **solves for the equilibrium pose** under an applied moment, so the pieces
//! genuinely feel each other: the facets constrain the motion, the ligaments engage
//! at the solved pose, the disc resists — all coupled.
//!
//! The three ingredients, each a piece rung 7 already validated in isolation:
//! - **disc bending** → a hinge spring ([`BondedDisc`]'s small-strain `k_disc`, a
//!   linearised bushing — the disc FEM only converges sub-degree, so across the ROM
//!   it is exactly the linear extrapolation rung 7 uses);
//! - **ligaments** → two pull-only `<spatial>` tendons (anterior ALL + posterior
//!   interspinous), field-derived attachment sites;
//! - **facets** → an SDF penalty over the two articular `ShapeConcave` grids, with
//!   each repulsive force **oriented along the fixed vertebra's outward SDF gradient**
//!   so it genuinely separates the bodies (rung 7 left the raw-grid normal's sign
//!   unvalidated and kept the facet term out of its headline; orienting it makes the
//!   contact restoring by construction — the fix this module adds).
//!
//! ## Sign convention
//!
//! All angles here are **flexion-positive**: the ML axis is oriented (via the facet
//! engagement asymmetry — facets open in flexion, engage in extension) so that a
//! positive angle is flexion and a negative angle is extension. An applied moment is
//! likewise positive for flexion. Callers never handle a raw handedness.

use std::cell::RefCell;
use std::sync::Arc;

use anyhow::{Context, Result};
use cf_fsu_geometry::{
    BODY_RADIUS, FACET_CELL, FACET_MAX_CONTACTS, SegmentFrame, extreme_vertex, facet_grid, oracle,
    segment_frame,
};
use cf_geometry::IndexedMesh;
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use sim_core::sdf::compute_shape_contact;
use sim_core::{Data, Pose, SdfContact, SdfGrid, ShapeConcave};
use sim_mjcf::load_model;

use crate::{BondedDisc, DiscParams, build_bonded_disc};

/// The flexion-probe angle (rad) at which the disc's small-strain bending stiffness
/// `k_disc` is measured — kept inside the bonded disc's validated sub-degree SPD range.
const K_DISC_PROBE: f64 = 0.86_f64 * std::f64::consts::PI / 180.0;
/// Angular bracket for the equilibrium bisection (rad). Comfortably spans the
/// physiological ROM (flexion ~6°, extension ~4.5°) on both sides of neutral.
const EQUILIBRIUM_BRACKET: f64 = 12.0_f64 * std::f64::consts::PI / 180.0;
/// Base residual tolerance (N·m) for accepting a bisection root — physically negligible,
/// far above the ~1e-5 N·m sub-grid jitter of the discretised facet moment at the built-in
/// `k_facet`. A `k_facet` sensitivity sweep scales the facet moment (and hence its jitter),
/// so the effective tolerance scales with that factor (see `equilibrium_with_facet_scale`).
const RESIDUAL_TOL: f64 = 1.0e-3;

/// The physiologic sagittal moment (N·m) an FSU replay is driven to — the ±bound of the
/// default [`moment_ramp`] (7.5 N·m, the standard in-vitro flexion/extension test load).
pub const PHYSIOLOGIC_MOMENT: f64 = 7.5;
/// Frame count of the default replay [`moment_ramp`].
pub const RAMP_FRAMES: usize = 25;

/// The default applied-moment ramp for a coupled replay.
///
/// [`RAMP_FRAMES`] evenly-spaced moments from −[`PHYSIOLOGIC_MOMENT`] (extension) to
/// +[`PHYSIOLOGIC_MOMENT`] (flexion), N·m — the sweep [`CoupledFsu::capture_ramp`] consumes.
/// Shared by the viewer and the validation test so the two can never sweep different grids.
#[must_use]
pub fn moment_ramp() -> Vec<f64> {
    (0..RAMP_FRAMES)
        .map(|i| {
            #[allow(clippy::cast_precision_loss)] // RAMP_FRAMES is tiny; the ratio is exact.
            let t = i as f64 / (RAMP_FRAMES - 1) as f64;
            -PHYSIOLOGIC_MOMENT + t * (2.0 * PHYSIOLOGIC_MOMENT)
        })
        .collect()
}

/// Tunable stiffnesses for the coupled FSU assembly.
///
/// [`Default`] reproduces the rung-5/6c/7 recipe (`k_lig` = 20 N/mm, `k_facet` = 200
/// N/mm, both uncalibrated; the disc recipe is [`DiscParams::default`]).
#[derive(Debug, Clone, Copy)]
pub struct CoupledParams {
    /// Disc tet-mesh + material recipe (drives the measured `k_disc` bushing).
    pub disc: DiscParams,
    /// Ligament tendon stiffness (N/mm).
    pub k_lig: f64,
    /// Facet penalty stiffness (N/mm) — the uncalibrated contact wall.
    pub k_facet: f64,
}

impl Default for CoupledParams {
    fn default() -> Self {
        Self {
            disc: DiscParams::default(),
            k_lig: 20.0,
            k_facet: 200.0,
        }
    }
}

/// The assembled coupled L4–L5 FSU.
///
/// Build with [`CoupledFsu::build`], then query the restoring moments or solve for
/// equilibrium ([`CoupledFsu::equilibrium`]) under an applied moment. Retains the live
/// bonded disc so a viewer can capture the coupled ROM together with the disc's
/// deformation ([`CoupledFsu::capture_ramp`]).
pub struct CoupledFsu {
    /// Disc bushing (hinge spring) + the two ligament tendons, in native mm.
    model: sim_core::Model,
    /// Reused solver scratch for [`Self::restoring_moment`], so the equilibrium bisection
    /// does not allocate a fresh `Data` on every evaluation. `RefCell` because the moment
    /// queries take `&self` (single-threaded use — the FSU is built and swept on one thread).
    scratch: RefCell<Data>,
    /// The segment's anatomical frame (body centres + SI/posterior/ML axes), retained so a
    /// viewer can build the ligament/co-registration overlays without recomputing it.
    frame: SegmentFrame,
    /// Superior (L4) articular SDF grid — rotates with flexion.
    g4: Arc<SdfGrid>,
    /// Inferior (L5) articular SDF grid — fixed; its outward gradient orients contact.
    g5: Arc<SdfGrid>,
    /// Shared flexion pivot (disc AABB centre, native mm).
    pivot: Point3<f64>,
    /// Flexion-positive ML axis (oriented so +θ = flexion).
    axis: Vector3<f64>,
    /// Facet penalty stiffness (N/mm).
    k_facet: f64,
    /// The disc's small-strain bending stiffness (N·m/rad, negative = restoring).
    k_disc: f64,
    /// The live bonded disc, retained for [`Self::capture_ramp`]'s deformation field.
    disc: BondedDisc,
}

impl CoupledFsu {
    /// Assemble the coupled FSU from the three meshes (native mm): tet-mesh + bond the
    /// disc, measure its `k_disc` bushing, build the ligament + bushing hinge model,
    /// and sample the two articular SDF grids. The flexion sense is derived from the
    /// facet engagement asymmetry (never hardcoded).
    ///
    /// # Errors
    /// Propagates a failure to derive the segment frame, build the bonded disc, or
    /// build the ligament MJCF model, and errors if the facets do not engage on exactly
    /// one rotation sense (a degenerate or mislabelled articular geometry).
    pub fn build(
        l4: &IndexedMesh,
        l5: &IndexedMesh,
        disc_mesh: IndexedMesh,
        params: &CoupledParams,
    ) -> Result<Self> {
        let o4 = oracle(l4).context("L4 oracle")?;
        let o5 = oracle(l5).context("L5 oracle")?;
        let frame = segment_frame(l4, l5, &o4, &o5).context("segment frame")?;

        let mut disc = build_bonded_disc(disc_mesh, &params.disc).context("build bonded disc")?;
        let pivot = disc.center_native();
        let ml = disc.ml_axis();
        // Small-strain disc bending stiffness (linear bushing), measured sub-degree.
        let (m, _) = disc.flexion_moment(K_DISC_PROBE);
        let k_disc = m / K_DISC_PROBE;

        // Orient the ML axis so +θ is flexion: probe the facet engagement asymmetry —
        // the articular surfaces open in flexion and engage in extension.
        let g4 = facet_grid(l4, &o4);
        let g5 = facet_grid(l5, &o5);
        let probe = 6.0_f64.to_radians();
        let n_pos = facet_engaged(&g4, &g5, pivot, ml, probe);
        let n_neg = facet_engaged(&g4, &g5, pivot, ml, -probe);
        anyhow::ensure!(
            (n_pos == 0) ^ (n_neg == 0),
            "facets must engage on exactly one rotation sense (extension): +{n_pos} / −{n_neg}"
        );
        // Flexion = the sense the facets OPEN. Fold that handedness into the axis so the
        // rest of the API is flexion-positive.
        let axis = if n_pos == 0 { ml } else { -ml };

        let model = build_coupled_model(l4, l5, &frame, axis, pivot, params.k_lig, -k_disc * 1e3)
            .context("build coupled ligament/bushing model")?;
        let scratch = RefCell::new(model.make_data());

        Ok(Self {
            model,
            scratch,
            frame,
            g4,
            g5,
            pivot,
            axis,
            k_facet: params.k_facet,
            k_disc,
            disc,
        })
    }

    /// The segment's anatomical frame (body centres + SI/posterior/ML axes).
    #[must_use]
    pub const fn frame(&self) -> &SegmentFrame {
        &self.frame
    }

    /// The shared flexion pivot (disc AABB centre, native mm).
    #[must_use]
    pub const fn pivot(&self) -> Point3<f64> {
        self.pivot
    }

    /// The flexion-positive ML axis (unit; +θ about it is flexion).
    #[must_use]
    pub const fn axis(&self) -> Vector3<f64> {
        self.axis
    }

    /// The disc's measured small-strain bending stiffness (N·m/rad, negative = restoring).
    #[must_use]
    pub const fn k_disc(&self) -> f64 {
        self.k_disc
    }

    /// The spring restoring moment (disc bushing + ligaments) about the flexion axis at
    /// angle `theta` (rad), in N·m. Reads the coupled model's `qfrc_spring` — the sum of
    /// the disc joint spring and both tendon springs (native mm → N·m). Negative opposes
    /// a positive (flexion) `theta`. Excludes facet contact (see [`Self::facet_moment`]).
    ///
    /// # Panics
    /// Panics if the coupled model's forward solve fails — unreachable for this fixed,
    /// valid 1-DOF model (no contacts, no actuators, gravity off).
    #[must_use]
    pub fn restoring_moment(&self, theta: f64) -> f64 {
        let mut data = self.scratch.borrow_mut();
        data.qpos[0] = theta;
        // `forward` is a full recompute from `qpos` (position-dependent spring; `qvel`
        // stays 0 so no damper term), so reusing the scratch `Data` across calls is exact.
        // Infallible for this static 1-DOF model (see the `# Panics` note): a forward solve
        // of two bodies + a hinge + tendons has no failure mode to recover from.
        #[allow(clippy::expect_used)]
        data.forward(&self.model).expect("coupled forward");
        data.qfrc_spring[0] * 1e-3
    }

    /// Facet contact at flexion `theta` (rad): `(engaged contact count, restoring moment
    /// about the flexion axis in N·m)`. Each repulsive force is oriented along L5's
    /// outward SDF gradient, so the moment genuinely opposes penetration (restoring).
    /// Zero in flexion (`theta > 0`), engaging in extension. This is the equilibrium
    /// solver's hot path, so it only COUNTS the contacts — it skips the per-contact point
    /// `Vec` that [`Self::facet_response`] collects (the SDF query itself still allocates a
    /// contact set; making that allocation-free is a `compute_shape_contact` change).
    #[must_use]
    pub fn facet_moment(&self, theta: f64) -> (usize, f64) {
        let mut n = 0;
        let moment = facet_oriented(
            &self.g4,
            &self.g5,
            self.pivot,
            self.axis,
            theta,
            self.k_facet,
            |_| {
                n += 1;
            },
        );
        (n, moment)
    }

    /// Facet contact at flexion `theta`: the penetrating contact **points** (native mm) and
    /// the restoring **moment** about the flexion axis (N·m), from one SDF query. The
    /// engaged count is `points.len()`; a viewer draws the points to show where the bones
    /// actually touch at that pose.
    #[must_use]
    pub fn facet_response(&self, theta: f64) -> (Vec<Point3<f64>>, f64) {
        let mut points = Vec::new();
        let moment = facet_oriented(
            &self.g4,
            &self.g5,
            self.pivot,
            self.axis,
            theta,
            self.k_facet,
            |p| points.push(p),
        );
        (points, moment)
    }

    /// Total coupled restoring moment (disc + ligaments + oriented facet contact) about
    /// the flexion axis at `theta` (rad), N·m. Monotone decreasing in `theta`. This is the
    /// SAME quantity the equilibrium solver balances (both route through the private
    /// `total_moment_scaled`), so a diagnostic reading of it can never drift from the
    /// solved root.
    #[must_use]
    pub fn total_moment(&self, theta: f64) -> f64 {
        self.total_moment_scaled(theta, 1.0)
    }

    /// Total restoring moment with the facet penalty scaled by `facet_scale` (N·m). The
    /// single definition of the coupled restoring the solver roots on and `total_moment`
    /// reports (`facet_scale = 1.0`).
    fn total_moment_scaled(&self, theta: f64, facet_scale: f64) -> f64 {
        self.restoring_moment(theta) + facet_scale * self.facet_moment(theta).1
    }

    /// Solve the coupled quasi-static equilibrium under an `applied` moment (N·m,
    /// positive = flexion): the angle `theta` (rad) at which the total restoring moment
    /// balances it (`total_moment(theta) = −applied`). Flexion-positive; extension is a
    /// negative applied moment and a negative `theta`, capped by facet contact.
    ///
    /// The total restoring moment is monotone in `theta`, so a bracketed bisection over
    /// `±EQUILIBRIUM_BRACKET` converges to the unique root. Returns `None` when no
    /// equilibrium exists within `±EQUILIBRIUM_BRACKET` (the applied moment exceeds what
    /// the segment can balance in range) — the caller must decide, never a silent
    /// bracket-edge angle.
    #[must_use]
    pub fn equilibrium(&self, applied: f64) -> Option<f64> {
        self.equilibrium_with_facet_scale(applied, 1.0)
    }

    /// Like [`Self::equilibrium`], but the facet penalty is scaled by `facet_scale`
    /// (`1.0` = the built-in `k_facet`). Because the oriented facet moment is exactly
    /// linear in `k_facet`, this evaluates a stiffer/softer facet contact **without
    /// rebuilding** — the primitive a `k_facet` sensitivity/convergence sweep needs.
    /// `facet_scale = 0.0` is the contact-free (spring-only) equilibrium. Returns `None`
    /// when no equilibrium lies within `±EQUILIBRIUM_BRACKET`.
    #[must_use]
    pub fn equilibrium_with_facet_scale(&self, applied: f64, facet_scale: f64) -> Option<f64> {
        // Balance: restoring + facet_scale·facet = −applied. Monotone DECREASING in theta,
        // so bisect over ±EQUILIBRIUM_BRACKET for the unique root. Scaling the facet moment
        // scales its jitter, so the residual tolerance scales with it too (never below the
        // base) — otherwise a stiff sweep point could reject a valid root as a false gap.
        let tol = RESIDUAL_TOL * facet_scale.max(1.0);
        solve_decreasing(-applied, EQUILIBRIUM_BRACKET, tol, |t| {
            self.total_moment_scaled(t, facet_scale)
        })
    }

    /// Capture a coupled force-driven trajectory for replay: for each `applied` moment
    /// (N·m, positive = flexion), solve the equilibrium angle and record it together with
    /// the disc's deformation at that angle. Because the disc bends linearly (and its FEM
    /// only converges sub-degree), the deformation at each equilibrium angle is the
    /// sub-degree field linearly scaled to that angle — the same extrapolation rung 7's
    /// `k_disc` relies on. The disc is probed at BOTH `±K_DISC_PROBE` and the correct-sign
    /// field is scaled per frame, so extension shows the disc's real (not mirror-of-flexion)
    /// shape. The superior vertebra rotates about `(pivot, axis, theta)`.
    ///
    /// # Errors
    /// Returns an error if a swept `applied` moment has no equilibrium within
    /// `±EQUILIBRIUM_BRACKET` ([`Self::equilibrium`] returns `None`) — e.g. a hypermobile
    /// segment or a supra-physiologic moment beyond the ~6° flexion / ~4.5° extension ROM.
    /// A physiologic ramp (`|moment| ≤ 7.5 N·m`) is always in range.
    ///
    /// # Panics
    /// Panics if the reference disc probe drives the soft solve past its SPD region — it
    /// does not, at the validated sub-degree probe angle (the disc's fail-close contract).
    pub fn capture_ramp(&mut self, applied_moments: &[f64]) -> Result<CoupledTrajectory> {
        // Reference disc deformation at BOTH sub-degree probes: the disc's response is only
        // near-antisymmetric, so extension frames scale the −probe field and flexion frames
        // the +probe field (rather than mirroring one), preserving the real extension shape.
        let ref_traj = self.disc.capture_flexion(&[-K_DISC_PROBE, K_DISC_PROBE]);
        let rest = ref_traj.rest_nodes_native;
        let disp = |frame: usize| -> Vec<Vector3<f64>> {
            ref_traj.frames[frame]
                .deformed_nodes_native
                .iter()
                .zip(&rest)
                .map(|(d, r)| d - r)
                .collect()
        };
        let disp_minus = disp(0); // field at −K_DISC_PROBE
        let disp_plus = disp(1); // field at +K_DISC_PROBE
        // The disc's own `capture_flexion` rotates about `disc.ml_axis()`; the coupled
        // equilibrium `theta` is about the flexion-oriented `self.axis` (= ±ml). A rotation
        // by `theta` about `self.axis` is one by `sign·theta` about `ml`, so the disc-frame
        // angle `phi` carries that sign; pick the probe field matching its sign.
        let disc_sign = self.axis.dot(&self.disc.ml_axis()).signum();

        let frames = applied_moments
            .iter()
            .map(|&applied| {
                let theta = self.equilibrium(applied).with_context(|| {
                    format!("no equilibrium within the ROM bracket for {applied} N·m")
                })?;
                let phi = disc_sign * theta; // disc-frame extrapolation angle
                // At the physiologic flexion peak (~6.13°) this scales the sub-degree field
                // by ~7×. Deliberately UNCAPPED: the disc surface must track L4's real solved
                // ROM (a deform cap would unglue the disc top from the vertebra). The old
                // viewer's ×6 ceiling guarded the *fragmented tet-boundary* render; this
                // displaces the clean watertight STL surface, which stays coherent at this
                // scale (user-verified clean at the 6.13° extreme). The ROM itself is bounded
                // by EQUILIBRIUM_BRACKET, so `s` cannot run away.
                let (field, s) = if phi >= 0.0 {
                    (&disp_plus, phi / K_DISC_PROBE)
                } else {
                    (&disp_minus, -phi / K_DISC_PROBE)
                };
                let deformed_nodes_native =
                    rest.iter().zip(field).map(|(r, d)| r + d * s).collect();
                // One facet query at the solved angle yields the engaged contact points a
                // viewer draws (count = len); the moment is unused here (the solver used it).
                let (facet_points, _) = self.facet_response(theta);
                Ok(CoupledFrame {
                    applied,
                    theta,
                    deformed_nodes_native,
                    facet_points,
                })
            })
            .collect::<Result<Vec<_>>>()?;

        Ok(CoupledTrajectory {
            pivot: self.pivot,
            axis: self.axis,
            rest_nodes_native: rest,
            boundary_faces: ref_traj.boundary_faces,
            frames,
        })
    }
}

/// One captured coupled equilibrium pose: the applied moment, the solved angle, the
/// disc's deformed surface (native mm), and the engaged facet contact points.
pub struct CoupledFrame {
    /// The applied moment about the flexion axis (N·m, positive = flexion).
    pub applied: f64,
    /// The solved equilibrium flexion angle (rad; negative = extension).
    pub theta: f64,
    /// The disc's deformed tet-vertex positions (native mm) at `theta`.
    pub deformed_nodes_native: Vec<Point3<f64>>,
    /// The penetrating facet contact points (native mm) at `theta` — where the bones
    /// actually touch. Empty in flexion (facets open); grows into extension. The engaged
    /// count a viewer reports is `facet_points.len()`.
    pub facet_points: Vec<Point3<f64>>,
}

/// A replayable capture of the coupled FSU swept over a moment ramp.
///
/// Native millimetres throughout (the vertebra/ligament/facet frame). Mirrors
/// [`crate::FlexionTrajectory`] but the frames are *force-driven equilibria* (a
/// solved angle per applied moment) rather than prescribed angles, and each carries
/// the facet engagement count so a viewer can show the bones stopping on the facets.
pub struct CoupledTrajectory {
    /// The flexion pivot (disc AABB centre, native mm).
    pub pivot: Point3<f64>,
    /// The flexion-positive ML axis.
    pub axis: Vector3<f64>,
    /// The disc's rest (θ = 0) tet-vertex positions, native mm.
    pub rest_nodes_native: Vec<Point3<f64>>,
    /// The disc surface triangulation, indexing every frame's `deformed_nodes_native`.
    pub boundary_faces: Vec<[crate::VertexId; 3]>,
    /// The captured equilibria, one per applied moment.
    pub frames: Vec<CoupledFrame>,
}

// ─────────────────────────── internal assembly helpers ───────────────────────────

/// Build the coupled 1-DOF model: L5 fixed, L4 on a flexion hinge about `axis` through
/// `pivot` carrying the disc bushing (`disc_stiffness_nmm`, N·mm/rad) plus the two
/// pull-only ligament tendons (anterior ALL + posterior interspinous), field-derived.
fn build_coupled_model(
    l4: &IndexedMesh,
    l5: &IndexedMesh,
    frame: &SegmentFrame,
    axis: Vector3<f64>,
    pivot: Point3<f64>,
    k_lig: f64,
    disc_stiffness_nmm: f64,
) -> Result<sim_core::Model> {
    let (b4, b5, post) = (frame.b4, frame.b5, frame.posterior);
    let attach = |m: &IndexedMesh, o: Point3<f64>, d: Vector3<f64>, r: f64| {
        extreme_vertex(m, o, d, axis, r).context("ligament attachment site (no vertex qualifies)")
    };
    let all4 = attach(l4, b4, -post, BODY_RADIUS)?;
    let all5 = attach(l5, b5, -post, BODY_RADIUS)?;
    let isp4 = attach(l4, b4, post, f64::INFINITY)?;
    let isp5 = attach(l5, b5, post, f64::INFINITY)?;
    let slack = |a: Point3<f64>, b: Point3<f64>| (a - b).norm();
    let site = |p: Point3<f64>| format!("{} {} {}", p.x, p.y, p.z);
    let mjcf = format!(
        r#"<mujoco model="fsu_coupled"><option gravity="0 0 0"/><worldbody>
      <body name="L5" pos="0 0 0"><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        <site name="l5a" pos="{a5}"/><site name="l5i" pos="{i5}"/></body>
      <body name="L4" pos="0 0 0">
        <joint name="flex" type="hinge" axis="{mx} {my} {mz}" pos="{pv}" stiffness="{ks}" springref="0"/>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        <site name="l4a" pos="{a4}"/><site name="l4i" pos="{i4}"/></body></worldbody>
      <tendon>
        <spatial name="ALL" stiffness="{k}" springlength="0 {sa}" damping="0" width="0.5"><site site="l5a"/><site site="l4a"/></spatial>
        <spatial name="ISP" stiffness="{k}" springlength="0 {si}" damping="0" width="0.5"><site site="l5i"/><site site="l4i"/></spatial>
      </tendon></mujoco>"#,
        a5 = site(all5),
        i5 = site(isp5),
        a4 = site(all4),
        i4 = site(isp4),
        mx = axis.x,
        my = axis.y,
        mz = axis.z,
        pv = site(pivot),
        ks = disc_stiffness_nmm,
        k = k_lig,
        sa = slack(all4, all5),
        si = slack(isp4, isp5),
    );
    load_model(&mjcf).context("coupled ligament/bushing model")
}

/// Bisect for the root of a **monotone-decreasing** `f` equal to `target` over
/// `[−bracket, +bracket]`.
///
/// Returns `None` in two cases, so a caller never receives a non-root angle:
/// - `target` is outside `[f(+bracket), f(−bracket)]` — no root in range (this stops a
///   bracketed bisection from silently returning the *bracket edge* as a solved root);
/// - the bisection converges but the **residual** `|f(root) − target|` exceeds `tol` —
///   which happens when the target falls in a discontinuity gap (no clean crossing, e.g. a
///   segment that would snap rather than balance). `tol` must sit above the numerical
///   jitter of `f` at a real root but below any physical gap.
fn solve_decreasing(target: f64, bracket: f64, tol: f64, f: impl Fn(f64) -> f64) -> Option<f64> {
    // Monotone decreasing ⇒ f(−bracket) is the max, f(+bracket) the min; a root exists
    // iff f(+bracket) ≤ target ≤ f(−bracket).
    if target > f(-bracket) || target < f(bracket) {
        return None;
    }
    let (mut lo, mut hi) = (-bracket, bracket);
    // Halve until the interval reaches f64 precision (~52 steps for a ~0.4 rad bracket);
    // 60 is a safe cap. Each iteration evaluates `f` (an SDF contact query here), so
    // stopping at convergence avoids ~20+ wasted queries per solve.
    for _ in 0..60 {
        if hi - lo < 1e-12 {
            break;
        }
        let mid = 0.5 * (lo + hi);
        if f(mid) > target {
            lo = mid; // still above target → move toward the decreasing (hi) end
        } else {
            hi = mid;
        }
    }
    let root = 0.5 * (lo + hi);
    // A converged bisection on a continuous `f` drives the residual to ~0. A residual that
    // stays above `tol` means the target fell in a discontinuity gap (no static equilibrium
    // — the segment would snap) → signal no-equilibrium.
    ((f(root) - target).abs() < tol).then_some(root)
}

/// The two articular SDF grids posed at flexion `theta` (superior `g4` rotated by `theta`
/// about `axis` through `pivot`, inferior `g5` fixed), and the resulting contact set.
///
/// The single source of the "pose the articular grids and query their contact" convention
/// (cell size [`FACET_CELL`], cap [`FACET_MAX_CONTACTS`]) shared by the coupled solver and
/// the rung-7 validation harness — so the convention lives in one place.
#[must_use]
pub fn posed_facet_contacts(
    g4: &Arc<SdfGrid>,
    g5: &Arc<SdfGrid>,
    pivot: Point3<f64>,
    axis: Vector3<f64>,
    theta: f64,
) -> Vec<SdfContact> {
    let r = UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), theta);
    let pose_a = Pose {
        position: Point3::from(pivot.coords - r * pivot.coords),
        rotation: r,
    };
    let pose_b = Pose {
        position: Point3::origin(),
        rotation: UnitQuaternion::identity(),
    };
    compute_shape_contact(
        &ShapeConcave::new(Arc::clone(g4)),
        &pose_a,
        &ShapeConcave::new(Arc::clone(g5)),
        &pose_b,
        FACET_CELL,
        FACET_MAX_CONTACTS,
    )
}

/// Whether a facet contact is **engaged** — the bones genuinely interpenetrate
/// (`penetration > 0`).
///
/// The single definition of "engaged" shared by the engagement asymmetry probe, the
/// oriented moment, and the rung-7 harness, so they can never disagree on the threshold.
#[must_use]
pub fn is_engaged(c: &SdfContact) -> bool {
    c.penetration > 0.0
}

/// Number of penetrating facet contacts at flexion `theta` (the engagement asymmetry).
fn facet_engaged(
    g4: &Arc<SdfGrid>,
    g5: &Arc<SdfGrid>,
    pivot: Point3<f64>,
    axis: Vector3<f64>,
    theta: f64,
) -> usize {
    posed_facet_contacts(g4, g5, pivot, axis, theta)
        .iter()
        .filter(|c| is_engaged(c))
        .count()
}

/// Oriented facet penalty **moment** about `axis` (N·m) at flexion `theta`, invoking
/// `on_point` for each penetrating (engaged) contact point (native mm). Each repulsive
/// force is oriented along L5's outward SDF gradient at the contact point, so the moment
/// genuinely separates L4 from L5 (restoring). The callback lets the caller COUNT contacts
/// (allocation-free — the equilibrium solver's hot path) or COLLECT the points (the
/// viewer's capture) from one SDF query, without this function committing to a `Vec`.
fn facet_oriented(
    g4: &Arc<SdfGrid>,
    g5: &Arc<SdfGrid>,
    pivot: Point3<f64>,
    axis: Vector3<f64>,
    theta: f64,
    k_facet: f64,
    mut on_point: impl FnMut(Point3<f64>),
) -> f64 {
    let mut m = Vector3::zeros();
    for c in posed_facet_contacts(g4, g5, pivot, axis, theta) {
        if !is_engaged(&c) {
            continue;
        }
        on_point(c.point);
        let g = g5.gradient_clamped(c.point);
        if g.norm() > 1e-9 {
            let f = g.normalize() * (k_facet * c.penetration);
            m += (c.point - pivot).cross(&f);
        }
    }
    m.dot(&axis) * 1e-3
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // tests may unwrap/panic.

    use super::*;
    use crate::test_support::synthetic_disc;

    #[test]
    fn solve_decreasing_finds_a_linear_spring_root() {
        // A pure restoring spring: total_moment(θ) = −k·θ. Equilibrium under an applied
        // moment M is θ* = M/k. Here target = −applied, and f(θ) = −k·θ.
        let k = 3.0_f64;
        let applied = 0.6_f64;
        let theta = solve_decreasing(-applied, 1.0, RESIDUAL_TOL, |t| -k * t).expect("root");
        assert!(
            (theta - applied / k).abs() < 1e-9,
            "expected θ = M/k = {:.6}, got {theta:.6}",
            applied / k
        );
        // A negative (extension) applied moment gives a negative angle.
        let theta_ext = solve_decreasing(applied, 1.0, RESIDUAL_TOL, |t| -k * t).expect("root");
        assert!((theta_ext + applied / k).abs() < 1e-9, "sign symmetry");
    }

    #[test]
    fn solve_decreasing_signals_no_root_outside_bracket() {
        // The bracket guard: when the true root (M/k = 2.0) lies beyond the ±1.0 bracket,
        // return None rather than silently handing back the bracket edge as a "root".
        let k = 3.0_f64;
        assert_eq!(
            solve_decreasing(-6.0, 1.0, RESIDUAL_TOL, |t| -k * t),
            None,
            "a root at θ=2.0 outside the ±1.0 bracket must yield None, not the edge"
        );
        assert_eq!(
            solve_decreasing(6.0, 1.0, RESIDUAL_TOL, |t| -k * t),
            None,
            "the extension side out-of-bracket case must also yield None"
        );
    }

    /// Two overlapping unit spheres one native-mm cell apart along +x, so their SDF
    /// grids interpenetrate → a facet contact query fires. `g5` is the fixed sphere at
    /// the origin; its outward gradient orients the repulsion.
    fn overlapping_spheres() -> (Arc<SdfGrid>, Arc<SdfGrid>) {
        let g5 = Arc::new(SdfGrid::sphere(Point3::origin(), 10.0, 24, 2.0));
        let g4 = Arc::new(SdfGrid::sphere(Point3::new(15.0, 0.0, 0.0), 10.0, 24, 2.0));
        (g4, g5)
    }

    #[test]
    fn facet_engagement_detects_overlap_only() {
        let (g4, g5) = overlapping_spheres();
        let pivot = Point3::origin();
        let axis = Vector3::z();
        // At θ=0 the spheres (centres 15 mm apart, radii 10) overlap by 5 mm → contacts.
        assert!(
            facet_engaged(&g4, &g5, pivot, axis, 0.0) > 0,
            "overlapping grids must produce facet contacts"
        );
        // Pull g4's grid far away by rotating it 90° about a far pivot → no overlap.
        let far_pivot = Point3::new(500.0, 0.0, 0.0);
        assert_eq!(
            facet_engaged(
                &g4,
                &g5,
                far_pivot,
                Vector3::z(),
                std::f64::consts::FRAC_PI_2
            ),
            0,
            "separated grids must produce no facet contacts"
        );
    }

    /// A synthetic vertebra point cloud: near-midline (x ≈ 0) vertices spread along the
    /// anterior/posterior (y) axis and vertically, so `extreme_vertex` can locate a
    /// body-rim (−y) and a spinous (+y) ligament site. `zc` is the body-centre height.
    fn vertebra_cloud(zc: f64) -> IndexedMesh {
        let vertices = [
            [0.0, -18.0, zc], // anterior body rim (−y)
            [0.0, 22.0, zc],  // posterior spinous tip (+y)
            [3.0, 0.0, zc],   // near-midline filler
            [-3.0, 0.0, zc + 4.0],
            [0.0, 5.0, zc - 4.0],
        ]
        .iter()
        .map(|&[x, y, z]| Point3::new(x, y, z))
        .collect();
        IndexedMesh {
            vertices,
            faces: vec![[0, 1, 2]],
        }
    }

    /// Assemble a synthetic [`CoupledFsu`] directly (the test submodule can name the
    /// private fields), exercising `build_coupled_model` + a real (synthetic) bonded disc
    /// + overlapping facet grids — without the licensed anatomy `build` needs.
    fn synthetic_fsu() -> CoupledFsu {
        let (l4, l5) = (vertebra_cloud(20.0), vertebra_cloud(0.0));
        let frame = SegmentFrame {
            b4: Point3::new(0.0, 0.0, 20.0),
            b5: Point3::new(0.0, 0.0, 0.0),
            superior_axis: Vector3::z(),
            posterior: Vector3::y(),
            ml: Vector3::x(),
        };
        let axis = Vector3::x();
        let pivot = Point3::new(0.0, 0.0, 10.0);
        let k_disc = -0.28;
        let model = build_coupled_model(&l4, &l5, &frame, axis, pivot, 20.0, -k_disc * 1e3)
            .expect("synthetic coupled model");
        // Well-separated grids → facets inactive, so the equilibrium here is the clean
        // spring-only (monotone) root. The engaged/oriented facet path is covered by the
        // `overlapping_spheres` tests.
        let g5 = Arc::new(SdfGrid::sphere(Point3::origin(), 10.0, 24, 2.0));
        let g4 = Arc::new(SdfGrid::sphere(Point3::new(80.0, 0.0, 0.0), 10.0, 24, 2.0));
        let disc = build_bonded_disc(synthetic_disc(), &DiscParams::default()).expect("disc");
        let scratch = RefCell::new(model.make_data());
        CoupledFsu {
            model,
            scratch,
            frame,
            g4,
            g5,
            pivot,
            axis,
            k_facet: 200.0,
            k_disc,
            disc,
        }
    }

    #[test]
    fn coupled_queries_and_capture_on_synthetic_parts() {
        let mut fsu = synthetic_fsu();
        // Accessors reflect what was assembled.
        assert_eq!(fsu.pivot(), Point3::new(0.0, 0.0, 10.0));
        assert_eq!(fsu.axis(), Vector3::x());
        assert!((fsu.k_disc() - (-0.28)).abs() < 1e-12);

        // The spring (disc + ligaments) is force-free at neutral and restoring off it.
        assert!(
            fsu.restoring_moment(0.0).abs() < 1e-6,
            "neutral must be force-free"
        );
        assert!(
            fsu.restoring_moment(3.0_f64.to_radians()) < 0.0,
            "a +θ (flexion) tilt must be opposed by a restoring (−) spring moment"
        );

        // Equilibrium under an applied flexion moment exists and is what total_moment
        // balances; an out-of-range moment (past the bracket) returns None.
        let theta = fsu
            .equilibrium(1.0)
            .expect("a 1 N·m flexion moment has an equilibrium in the bracket");
        assert!(theta.is_finite(), "equilibrium must be a finite angle");
        assert!(
            (fsu.total_moment(theta) + 1.0).abs() < 1e-2,
            "at equilibrium total_moment(θ) ≈ −applied"
        );
        assert_eq!(
            fsu.equilibrium(1.0e6),
            None,
            "a moment far beyond the ROM must return None, not a bracket-edge angle"
        );
        // With well-separated grids the facets never engage, so any facet scale gives the
        // same (spring-only) equilibrium as the built-in path — exercises the sweep method.
        assert_eq!(
            fsu.equilibrium_with_facet_scale(1.0, 5.0),
            fsu.equilibrium(1.0),
            "facet scaling must not change a contact-free equilibrium"
        );

        // Capture a small ramp: one frame per applied moment, sharing the disc surface.
        let traj = fsu
            .capture_ramp(&[-0.5, 0.0, 0.5])
            .expect("all three moments have equilibria in the bracket");
        assert_eq!(traj.frames.len(), 3);
        assert_eq!(traj.pivot, fsu.pivot());
        assert_eq!(traj.axis, fsu.axis());
        assert!(!traj.boundary_faces.is_empty(), "disc must have a surface");
        assert!(
            traj.frames
                .iter()
                .all(|f| f.deformed_nodes_native.len() == traj.rest_nodes_native.len()),
            "every frame's deformed surface matches the rest node count"
        );
    }

    #[test]
    fn oriented_facet_moment_is_linear_in_stiffness() {
        // f = ĝ·(k·penetration), so the moment scales exactly linearly with k_facet —
        // a deterministic property of the oriented penalty, independent of the geometry.
        let (g4, g5) = overlapping_spheres();
        let pivot = Point3::new(0.0, 0.0, -5.0); // offset so the contact has a lever arm
        let axis = Vector3::z();
        let (mut n1, mut n2) = (0usize, 0usize);
        let m1 = facet_oriented(&g4, &g5, pivot, axis, 0.0, 100.0, |_| n1 += 1);
        let m2 = facet_oriented(&g4, &g5, pivot, axis, 0.0, 200.0, |_| n2 += 1);
        assert!(n1 > 0 && n1 == n2, "same geometry → same engaged count");
        assert!(
            (m2 - 2.0 * m1).abs() < 1e-9 * m1.abs().max(1e-12),
            "oriented facet moment must be linear in k_facet: m(200)={m2:.4}, 2·m(100)={:.4}",
            2.0 * m1
        );
    }
}
