//! Rung 6b — the two-rigid **bonded sandwich** (FSU disc keystone, forward half).
//!
//! Drives [`BondedSandwich`]: a primitive Neo-Hookean disc bonded between two
//! free-joint rigid bodies (the L5 / L4 endplates), gravity off. Proves the three
//! properties a two-way soft↔rigid bond must have — and that a unilateral penalty
//! plane ([`StaggeredCoupling`](sim_coupling::StaggeredCoupling)) cannot:
//!
//! 1. **Conservation across the coupled interface** — the disc's reaction on the two
//!    endplates is a self-equilibrated wrench: `Σ force = 0` and `Σ moment = 0` about
//!    a common point, to ~1e-10 (Newton's third law, from the solver's own assembly).
//! 2. **Two-way tension** — under flexion the bonded face carries BOTH compression
//!    (`+z`) AND tension (`−z`) reactions (the anulus stretches on one side).
//! 3. **Correct coupled response** — a compressed disc springs the endplates APART.
//!
//! The crisp mechanism (1 + 2 + a restoring moment) is measured at PRESCRIBED
//! compression / flexion configurations via [`BondedSandwich::probe`]; the coupled
//! ROUND-TRIP (3) is measured over a dynamic rollout through the real rigid engine.
//!
//! Forward only (differentiability = rung 6d). Primitive tet disc (real geometry =
//! rung 6c). The reaction readout's force MAGNITUDE is validated independently
//! against the analytic uniaxial solution in `sim-soft`'s
//! `nodal_reaction_matches_analytic_uniaxial_traction`.

#![allow(
    // A malformed MJCF / body index surfaces as a test panic — the canonical
    // fixture idiom in this workspace's integration tests.
    clippy::expect_used,
    clippy::cast_precision_loss
)]

use nalgebra::UnitQuaternion;
use sim_coupling::BondedSandwich;
use sim_mjcf::load_model;
use sim_soft::Vec3;

const EDGE: f64 = 0.02; // 20 mm primitive disc, rest slab z ∈ [0, EDGE]
const N: usize = 2; // cells per edge (nz even)
const MU: f64 = 1.0e5;
const STATIC_DT: f64 = 1.0e3; // quasi-static disc
const H: f64 = 0.01; // rigid box half-thickness (COM offset from the bonded face)

const LOWER: usize = 1; // body index of L5 (world = 0)
const UPPER: usize = 2; // body index of L4
const C: f64 = EDGE / 2.0; // disc lateral centre (the rest slab spans [0, EDGE]²)
const UPPER_REST_Z: f64 = EDGE + H; // L4 COM at rest (bottom face on the disc top)

/// Two free-joint boxes stacked with the disc's rest slab exactly between their
/// facing surfaces: L5's top at `z = 0`, L4's bottom at `z = EDGE`. The boxes are
/// centred on the disc's lateral centre `(C, C)` so each "vertebra" COM sits over
/// its endplate (a centred axial load, no spurious off-COM moment).
fn fsu_mjcf() -> String {
    format!(
        r#"<mujoco>
  <option gravity="0 0 0" timestep="0.001"/>
  <worldbody>
    <body name="L5" pos="{c} {c} {z5}">
      <freejoint/>
      <geom type="box" size="{hx} {hx} {h}" mass="0.05"/>
    </body>
    <body name="L4" pos="{c} {c} {z4}">
      <freejoint/>
      <geom type="box" size="{hx} {hx} {h}" mass="0.05"/>
    </body>
  </worldbody>
</mujoco>"#,
        c = C,
        z5 = -H,
        z4 = UPPER_REST_Z,
        hx = EDGE / 2.0,
        h = H,
    )
}

fn build() -> BondedSandwich {
    let model = load_model(&fsu_mjcf()).expect("FSU MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    BondedSandwich::new(model, data, LOWER, UPPER, N, EDGE, MU, STATIC_DT)
}

/// Total reaction wrench `(Σ force, Σ moment about `p0`)` over BOTH bonded faces,
/// from the sandwich's last per-DOF reaction + the world targets it was read at.
/// For a self-equilibrated internal-force field this is `(0, 0)` up to the free-node
/// residual — the conservation oracle (SUT == the solver's own assembly).
fn total_reaction_wrench(c: &BondedSandwich, p0: Vec3) -> (Vec3, Vec3) {
    let react = c.last_reaction();
    let targets = c.last_targets();
    let (mut force, mut moment) = (Vec3::zeros(), Vec3::zeros());
    for &v in c.lower_face().iter().chain(c.upper_face()) {
        let i = v as usize;
        let f = Vec3::new(react[3 * i], react[3 * i + 1], react[3 * i + 2]);
        let r = Vec3::new(targets[3 * i], targets[3 * i + 1], targets[3 * i + 2]);
        force += f;
        moment += (r - p0).cross(&f);
    }
    (force, moment)
}

/// `(min, max)` of the per-node `z` reaction over the upper bonded face — the
/// two-way-tension probe (a unilateral penalty gives `min ≥ 0`).
fn upper_face_reaction_z_range(c: &BondedSandwich) -> (f64, f64) {
    let react = c.last_reaction();
    c.upper_face()
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &v| {
            let rz = react[3 * v as usize + 2];
            (lo.min(rz), hi.max(rz))
        })
}

#[test]
fn bonded_disc_conserves_and_carries_two_way_tension_under_flexion() {
    // --- (A) COMPRESSION: press L4 straight down by δ, no rotation. ---
    let mut c = build();
    let delta = 0.05 * EDGE; // 5% axial compression
    c.set_body_pose(
        UPPER,
        Vec3::new(C, C, UPPER_REST_Z - delta),
        UnitQuaternion::identity(),
    );
    let comp = c.probe();

    let (f_tot, m_tot) = total_reaction_wrench(&c, Vec3::zeros());
    let scale = comp.force_lower.norm().max(comp.force_upper.norm());
    eprintln!(
        "[COMPRESSION] F_up.z={:+.4} F_low.z={:+.4} |ΣF|/s={:.2e} |ΣM|/(s·EDGE)={:.2e}",
        comp.force_upper.z,
        comp.force_lower.z,
        f_tot.norm() / scale,
        m_tot.norm() / (scale * EDGE)
    );
    assert!(
        f_tot.norm() / scale < 1e-9,
        "compression: force not conserved"
    );
    assert!(
        m_tot.norm() / (scale * EDGE) < 1e-9,
        "compression: moment not conserved"
    );
    // The disc RESISTS compression: it pushes the upper plate up (+z) and the lower
    // plate down (−z) — it springs the endplates apart.
    assert!(
        comp.force_upper.z > 0.0,
        "compressed disc must push the upper plate up"
    );
    assert!(
        comp.force_lower.z < 0.0,
        "compressed disc must push the lower plate down"
    );
    // Pure compression is one-sided: the whole face is in compression (no tension).
    let (comp_min, _comp_max) = upper_face_reaction_z_range(&c);
    assert!(
        comp_min >= 0.0,
        "pure compression should give no tension, got min {comp_min:.4}"
    );

    // --- (B) FLEXION: rotate L4 about the medio-lateral (x) axis through the disc
    //         top-face centre by θ. This tilts the bonded top face — one side lifts
    //         (tension), the other presses (compression). ---
    let mut c = build();
    let theta = 0.08_f64; // ~4.6°
    let pivot = Vec3::new(C, C, EDGE); // disc top-face centre (the flexion axis passes through it)
    let rot = UnitQuaternion::from_axis_angle(&Vec3::x_axis(), theta);
    let rest_com = Vec3::new(C, C, UPPER_REST_Z);
    let flexed_com = pivot + rot * (rest_com - pivot);
    c.set_body_pose(UPPER, flexed_com, rot);
    let flex = c.probe();

    let (f_tot, m_tot) = total_reaction_wrench(&c, Vec3::zeros());
    let scale = flex.force_lower.norm().max(flex.force_upper.norm());
    let (min_z, max_z) = upper_face_reaction_z_range(&c);
    eprintln!(
        "[FLEXION] face rz∈[{min_z:+.4},{max_z:+.4}] M_up.x={:+.5} |ΣF|/s={:.2e} |ΣM|/(s·EDGE)={:.2e}",
        flex.moment_upper.x,
        f_tot.norm() / scale,
        m_tot.norm() / (scale * EDGE)
    );
    assert!(f_tot.norm() / scale < 1e-9, "flexion: force not conserved");
    assert!(
        m_tot.norm() / (scale * EDGE) < 1e-9,
        "flexion: moment not conserved"
    );
    // TWO-WAY: the bonded face carries BOTH tension and compression, comparable in
    // magnitude (not a marginal numerical sliver).
    assert!(
        min_z < 0.0 && max_z > 0.0,
        "flexed face must carry both tension (min<0) and compression (max>0): [{min_z:.4},{max_z:.4}]"
    );
    assert!(
        min_z.abs() > 0.2 * max_z,
        "tension {min_z:.4} should be comparable to compression {max_z:.4}, not a sliver"
    );
    // RESTORING: a +x tilt is opposed by a −x moment on the upper plate.
    assert!(
        flex.moment_upper.x < 0.0,
        "flexed disc must produce a restoring moment (M_up.x<0), got {:+.5}",
        flex.moment_upper.x
    );
}

#[test]
fn deformed_surface_accessors_track_the_solved_field() {
    // The visualization seam (viz flexion-deformation rung): `soft_positions` +
    // `soft_boundary_faces` must expose the ACTUAL solved deformed surface, not the
    // rest mesh. Build+measure — no proxy: pin the upper endplate to a known
    // compressed pose and read the surface back.
    let mut c = build();
    let rest = c.soft_positions().to_vec(); // constructor seeds `x` at rest
    let faces_rest = c.soft_boundary_faces().to_vec();
    let n = rest.len() / 3;

    // Topology is a valid, non-empty surface indexing into the position buffer.
    assert!(!faces_rest.is_empty(), "disc must have a boundary surface");
    assert!(
        faces_rest.iter().flatten().all(|&v| (v as usize) < n),
        "every boundary-face vertex id must index into soft_positions ({n} verts)"
    );

    // Compress the upper endplate straight down by δ and solve.
    let delta = 0.05 * EDGE;
    c.set_body_pose(
        UPPER,
        Vec3::new(C, C, UPPER_REST_Z - delta),
        UnitQuaternion::identity(),
    );
    c.probe();
    let deformed = c.soft_positions();

    // (1) The buffer is the same shape, and the topology is deformation-invariant
    //     (only vertex positions move) — the exact contract a renderer relies on.
    assert_eq!(
        deformed.len(),
        rest.len(),
        "position buffer length constant"
    );
    assert_eq!(
        c.soft_boundary_faces(),
        faces_rest.as_slice(),
        "boundary topology must not change under deformation"
    );

    // (2) The field ACTUALLY deformed — some node moved off its rest position.
    let max_disp = (0..n)
        .map(|i| {
            let d = |k| deformed[3 * i + k] - rest[3 * i + k];
            (d(0) * d(0) + d(1) * d(1) + d(2) * d(2)).sqrt()
        })
        .fold(0.0_f64, f64::max);
    assert!(
        max_disp > 1e-6,
        "probe must deform the disc; max node displacement {max_disp:.2e} m"
    );

    // (3) The upper bonded face sits at its imposed target: every upper-face node's
    //     z is the rest top (EDGE) displaced by exactly −δ (Dirichlet-pinned to the
    //     box). This ties the exposed surface to the posed rigid body precisely.
    for &v in c.upper_face() {
        let z = deformed[3 * v as usize + 2];
        assert!(
            (z - (EDGE - delta)).abs() < 1e-9,
            "upper-face node {v} must be pinned to the compressed endplate z={:.6}, got {z:.6}",
            EDGE - delta
        );
    }
}

#[test]
fn compressed_disc_springs_endplates_apart_dynamically() {
    // The coupled ROUND-TRIP: drive the endplates toward each other (no gravity) and
    // let the disc's reaction wrench drive the real rigid engine. An initial approach
    // VELOCITY (not a displaced qpos, which would corrupt the bond reference)
    // compresses the disc; the reaction must push the plates apart, conserving each step.
    let model = load_model(&fsu_mjcf()).expect("FSU MJCF loads");
    let mut data = model.make_data();
    data.qvel[6 + 2] = -0.02; // L4 linear vz: descend
    data.qvel[2] = 0.02; // L5 linear vz: ascend
    data.forward(&model).expect("initial forward");
    let mut c = BondedSandwich::new(model, data, LOWER, UPPER, N, EDGE, MU, STATIC_DT);

    let mut max_rel_force = 0.0_f64;
    let mut max_norm_moment = 0.0_f64;
    let mut saw_spring_apart = false;
    for k in 0..24 {
        let s = c.step();
        let (f_tot, m_tot) = total_reaction_wrench(&c, Vec3::zeros());
        let scale = s.force_lower.norm().max(s.force_upper.norm()).max(1e-30);
        max_rel_force = max_rel_force.max(f_tot.norm() / scale);
        max_norm_moment = max_norm_moment.max(m_tot.norm() / (scale * EDGE).max(1e-30));
        if s.force_upper.z > 1e-6 && s.force_lower.z < -1e-6 {
            saw_spring_apart = true;
        }
        if k % 6 == 0 {
            eprintln!(
                "k={k:02}: F_low.z={:+.4} F_up.z={:+.4} |ΣF|/s={:.2e} L4.z={:.5} vz4={:+.4}",
                s.force_lower.z,
                s.force_upper.z,
                f_tot.norm() / scale,
                c.data().xpos[UPPER].z,
                c.data().qvel[6 + 2],
            );
        }
    }
    eprintln!("max rel |ΣF| = {max_rel_force:.2e}, max norm'd |ΣM| = {max_norm_moment:.2e}");
    assert!(
        max_rel_force < 1e-9,
        "force not conserved across the bond: {max_rel_force:.2e}"
    );
    assert!(
        max_norm_moment < 1e-9,
        "moment not conserved across the bond: {max_norm_moment:.2e}"
    );
    assert!(
        saw_spring_apart,
        "compressed disc never pushed the endplates apart (F_up.z>0 ∧ F_low.z<0)"
    );
}
