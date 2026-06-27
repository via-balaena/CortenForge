//! Rung 3b of the de-escalation visualization ladder — **two-way coupling**: the striker arm is now
//! a DYNAMIC articulated body (sim-core) that FEELS the buffer's reaction. In Rungs 2/3 the striker
//! was kinematic (the buffer felt the contact; the striker did not feel anything back). Here the
//! buffer's reaction wrench is routed back to the arm, so the arm **decelerates and rebounds** off
//! the soft agent instead of plowing through — the first genuinely two-way finite-contact
//! articulated coupling (R1 had two-way but only with an infinite plane; R2/R3 had finite contact
//! but one-way).
//!
//! The loop is `StaggeredCoupling::step_articulated`: per step the coupling poses the finite fist
//! sphere at the arm's end-effector geom (`with_contact_geom`) → solves the soft buffer → routes the
//! off-COM reaction wrench `[(rᵢ − COM) × fᵢ ; fᵢ]` (the distributed contact moment) to the arm's
//! `xfrc_applied` → steps the arm. Earlier rungs hand-rolled this loop because the coupling could
//! not yet pose a finite collider at a moving end-effector; now it owns the whole two-way step. A
//! single hinge keeps the articulated body minimal so the two-way coupling is the only new variable.
//! Forward only (no gradients).
//!
//! ## Run
//! Headless capture + a one-line summary always run (CI-safe). The Bevy window is opt-in:
//! ```text
//! CF_VISUAL=1 cargo run -p example-integration-two-way-striker-viewer --release
//! ```
//! Orbit: left-drag rotate · right-drag pan · scroll zoom.

// Bevy systems take `Res`/`Query`/`Commands` by value and reference items rustdoc can't link from
// a binary — the standard allow set for this repo's Bevy examples (mirrors the Rung 1/2/3 siblings).
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::needless_pass_by_value,
    clippy::doc_markdown,
    // run_capture is one coherent staggered two-way loop; splitting it would scatter the per-step
    // soft + rigid state across helpers without improving clarity.
    clippy::too_many_lines
)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;
use sim_soft::material::silicone_table::ECOFLEX_00_30_MEASURED;
use sim_soft::{Vec3, VertexId};

// ── Soft buffer (measured-Ecoflex block, finer mesh like Rung 3) ──
const N_PER_EDGE: usize = 10;
const EDGE: f64 = 0.10; // m — soft cube side, z ∈ [0, 0.10], base pinned
const DT: f64 = 1.0e-3;
const KAPPA: f64 = 3.0e4;
const D_HAT: f64 = 5.0e-3;

// ── Striker: a single-hinge arm rooted at a shoulder, a fist (sphere) at the tip ──
const FIST_R: f64 = 0.04; // 4 cm blunt fist
const ARM_L: f64 = 0.15; // pivot → fist length
const PIVOT_X: f64 = -0.06;
const PIVOT_Z: f64 = 0.19;
const FIST_MASS: f64 = 0.5; // kg — the swinging hammer head (dominates the arm inertia)
const ARM_MASS: f64 = 0.05; // kg — the link
const INIT_QVEL: f64 = 8.0; // rad/s downward swing (+θ about +y rotates the fist DOWN)
const N_STEPS: usize = 220;

// ── Replay / render ──
const RENDER_SCALE: f32 = 10.0;
const REPLAY_DT: f64 = 0.012;

/// One headless capture of the two-way strike: soft buffer frames + the arm's [pivot, fist] world
/// points per step, plus the rest surface mesh; the arm's angular velocity is tracked to show the
/// rebound (the two-way signature).
struct Capture {
    rest_positions: Vec<Vec3>,
    boundary_faces: Vec<[VertexId; 3]>,
    soft_frames: Vec<Vec<f64>>,
    joints: Vec<[[f64; 3]; 2]>, // [pivot, fist] per frame
    qvel: Vec<f64>,
    peak_force: f64,
    max_indent_reached: f64,
}

fn run_capture() -> Capture {
    // Single-hinge arm: a capsule link from the pivot to the fist + a heavy fist sphere at the tip.
    // The fist geom is named so we can read its world centre (the end-effector) each step.
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="{PIVOT_X} 0.05 {PIVOT_Z}">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 {ARM_L} 0 0" size="0.012" mass="{ARM_MASS}"/>
      <geom name="fist" type="sphere" pos="{ARM_L} 0 0" size="{FIST_R}" mass="{FIST_MASS}"/>
    </body>
  </worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).expect("arm MJCF loads");
    let arm_bid = model.body_id("arm").expect("arm body");
    let fist_gid = model.geom_id("fist").expect("fist geom");
    let mut data = model.make_data();
    data.qvel[0] = INIT_QVEL;
    data.forward(&model).expect("initial forward");

    // The keystone coupling owns BOTH halves of the two-way loop: per `step_articulated` it poses
    // the soft contact sphere at the fist geom (`with_contact_geom` → the arm tip), takes one
    // finite-fist soft step, and routes the off-COM reaction WRENCH `[(rᵢ−c)×fᵢ ; fᵢ]` back to the
    // arm — the loop the earlier rungs hand-rolled, now the coupling's own forward stepper.
    let mu = ECOFLEX_00_30_MEASURED.mu;
    let mut c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, arm_bid, 0.0, N_PER_EDGE, EDGE, mu, DT, KAPPA, D_HAT, 0.0,
    )
    .with_sphere_collider(FIST_R)
    .with_contact_geom(fist_gid);

    let to_vec3 = |flat: &[f64]| -> Vec<Vec3> {
        flat.chunks_exact(3)
            .map(|p| Vec3::new(p[0], p[1], p[2]))
            .collect()
    };
    let rest_positions = to_vec3(c.soft_positions());
    let boundary_faces: Vec<[VertexId; 3]> = c.soft_boundary_faces();

    let pt = |p: Vec3| [p.x, p.y, p.z];
    let mut soft_frames: Vec<Vec<f64>> = vec![c.soft_positions().to_vec()];
    let mut joints: Vec<[[f64; 3]; 2]> =
        vec![[pt(c.data().xpos[arm_bid]), pt(c.data().geom_xpos[fist_gid])]];
    let mut qvel: Vec<f64> = vec![c.data().qvel[0]];
    let mut peak_force = 0.0_f64;
    let mut max_indent_reached = 0.0_f64;

    for _ in 0..N_STEPS {
        // One two-way coupled step: pose the fist at the arm tip → soft solve → route the off-COM
        // reaction wrench → step the arm (it decelerates / rebounds off the buffer).
        let step = c.step_articulated();

        peak_force = peak_force.max(step.force_on_soft.z.abs());
        let deepest_push = to_vec3(c.soft_positions())
            .iter()
            .zip(&rest_positions)
            .map(|(p, r)| r.z - p.z)
            .fold(0.0_f64, f64::max);
        max_indent_reached = max_indent_reached.max(deepest_push);

        soft_frames.push(c.soft_positions().to_vec());
        joints.push([pt(c.data().xpos[arm_bid]), pt(c.data().geom_xpos[fist_gid])]);
        qvel.push(c.data().qvel[0]);
    }

    Capture {
        rest_positions,
        boundary_faces,
        soft_frames,
        joints,
        qvel,
        peak_force,
        max_indent_reached,
    }
}

fn main() {
    let cap = run_capture();
    println!(
        "two-way strike captured: {} frames · peak contact force {:.0} N · max indent {:.1}%",
        cap.soft_frames.len(),
        cap.peak_force,
        cap.max_indent_reached / EDGE * 100.0,
    );
    // Diagnostic: the arm's angular velocity over the strike — the TWO-WAY signature is a clear
    // deceleration (and rebound, ω reversing sign) when the fist meets the buffer.
    let fist_z = |i: usize| cap.joints[i][1][2];
    let (mut lo_i, mut lo_z) = (0usize, f64::MAX);
    for i in 0..cap.joints.len() {
        if fist_z(i) < lo_z {
            lo_z = fist_z(i);
            lo_i = i;
        }
    }
    let qmin = cap.qvel.iter().copied().fold(f64::MAX, f64::min);
    let qmax = cap.qvel.iter().copied().fold(f64::MIN, f64::max);
    println!(
        "  arm ω (rad/s): start {:.2} → range [{qmin:.2}, {qmax:.2}] · deepest fist z {lo_z:.3} (frame {lo_i}, ω there {:.2})",
        cap.qvel[0], cap.qvel[lo_i],
    );
    println!(
        "  two-way check: ω {} after the strike (decelerated/rebounded ⇒ the arm felt the buffer)",
        if qmin < cap.qvel[0] - 0.5 {
            "DROPPED"
        } else {
            "did NOT drop — coupling looks inert"
        }
    );

    if std::env::var("CF_VISUAL").is_ok() {
        println!(
            "CF_VISUAL set — opening Bevy replay (loops continuously; close window to exit) ..."
        );
        visual::run(cap);
    } else {
        println!(
            "set CF_VISUAL=1 to watch the arm strike and rebound:\n  \
             CF_VISUAL=1 cargo run -p example-integration-two-way-striker-viewer --release"
        );
    }
}

// ─────────────────────────── Bevy replay (visual mode only) ───────────────────────────
mod visual {
    use super::{Capture, EDGE, FIST_R, RENDER_SCALE, REPLAY_DT};
    use bevy::prelude::*;
    use bevy::time::Real;
    use cf_bevy_common::axis::UpAxis;
    use cf_bevy_common::camera::{OrbitCamera, OrbitCameraPlugin};
    use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};

    const LINK_R: f64 = 0.012;

    #[derive(Resource)]
    struct Scene(Option<Capture>);

    #[derive(Resource)]
    struct Replay {
        soft_frames: Vec<Vec<f64>>,
        joints: Vec<[[f64; 3]; 2]>, // [pivot, fist]
        dt: f64,
        epoch: Option<f64>,
    }

    #[derive(Component)]
    struct SoftBody;
    #[derive(Component)]
    struct Fist;
    #[derive(Component)]
    struct Arm;

    pub fn run(cap: Capture) {
        App::new()
            .add_plugins(DefaultPlugins)
            .add_plugins(OrbitCameraPlugin)
            .insert_resource(UpAxis::PlusZ)
            .insert_resource(Scene(Some(cap)))
            .add_systems(Startup, setup)
            .add_systems(Update, replay)
            .run();
    }

    fn to_bevy(p: [f64; 3]) -> Vec3 {
        Vec3::new(p[0] as f32, p[2] as f32, p[1] as f32) * RENDER_SCALE
    }

    /// Transform for a unit cylinder (radius 1, height 1, axis +Y) spanning Bevy points `a → b`.
    fn segment_xf(a: Vec3, b: Vec3, r: f32) -> Transform {
        let d = b - a;
        let len = d.length().max(1e-6);
        Transform {
            translation: (a + b) * 0.5,
            rotation: Quat::from_rotation_arc(Vec3::Y, d / len),
            scale: Vec3::new(r, len, r),
        }
    }

    fn setup(
        mut commands: Commands,
        mut meshes: ResMut<Assets<Mesh>>,
        mut materials: ResMut<Assets<StandardMaterial>>,
        mut scene: ResMut<Scene>,
        up: Res<UpAxis>,
    ) {
        let Some(cap) = scene.0.take() else {
            return;
        };

        let soft_mesh = meshes.add(build_soft_mesh(
            &cap.rest_positions,
            &cap.boundary_faces,
            *up,
        ));
        let soft_material = materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.55, 0.4),
            perceptual_roughness: 0.5,
            metallic: 0.05,
            ..default()
        });
        commands.spawn((
            Mesh3d(soft_mesh),
            MeshMaterial3d(soft_material),
            Transform::from_scale(Vec3::splat(RENDER_SCALE)),
            SoftBody,
        ));

        let limb_material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.6, 0.62, 0.68),
            perceptual_roughness: 0.6,
            ..default()
        });
        let fist_mesh = meshes.add(Sphere::new(FIST_R as f32));
        commands.spawn((
            Mesh3d(fist_mesh),
            MeshMaterial3d(limb_material.clone()),
            Transform {
                translation: to_bevy(cap.joints[0][1]),
                scale: Vec3::splat(RENDER_SCALE),
                ..default()
            },
            Fist,
        ));
        let arm_mesh = meshes.add(Cylinder::new(1.0, 1.0));
        commands.spawn((
            Mesh3d(arm_mesh),
            MeshMaterial3d(limb_material),
            segment_xf(
                to_bevy(cap.joints[0][0]),
                to_bevy(cap.joints[0][1]),
                LINK_R as f32 * RENDER_SCALE,
            ),
            Arm,
        ));

        let target = Vec3::new(
            0.5 * EDGE as f32 * RENDER_SCALE,
            1.1,
            0.5 * EDGE as f32 * RENDER_SCALE,
        );
        commands.spawn((
            Camera3d::default(),
            Transform::default(),
            OrbitCamera::new()
                .with_target(target)
                .with_distance(4.5 * RENDER_SCALE * EDGE as f32)
                .with_angles(0.4, 0.4),
            AmbientLight {
                color: Color::WHITE,
                brightness: 80.0,
                ..default()
            },
        ));
        commands.spawn((
            DirectionalLight {
                illuminance: 12_000.0,
                shadows_enabled: false,
                ..default()
            },
            Transform::from_xyz(0.6, 1.0, 0.4).looking_at(Vec3::ZERO, Vec3::Y),
        ));

        commands.insert_resource(Replay {
            soft_frames: cap.soft_frames,
            joints: cap.joints,
            dt: REPLAY_DT,
            epoch: None,
        });
    }

    fn replay(
        time: Res<Time<Real>>,
        mut replay: ResMut<Replay>,
        up: Res<UpAxis>,
        mut meshes: ResMut<Assets<Mesh>>,
        q_soft: Query<&Mesh3d, With<SoftBody>>,
        mut q_fist: Query<&mut Transform, (With<Fist>, Without<Arm>)>,
        mut q_arm: Query<&mut Transform, (With<Arm>, Without<Fist>)>,
    ) {
        let now = time.elapsed_secs_f64();
        let epoch = *replay.epoch.get_or_insert(now);
        let n = replay.soft_frames.len();
        let idx = (((now - epoch) / replay.dt) as usize) % n;

        let soft_mesh = q_soft.single().ok().and_then(|h| meshes.get_mut(&h.0));
        if let Some(mesh) = soft_mesh {
            apply_soft_positions(mesh, &replay.soft_frames[idx], *up);
        }
        if let Ok(mut tf) = q_fist.single_mut() {
            tf.translation = to_bevy(replay.joints[idx][1]);
        }
        if let Ok(mut tf) = q_arm.single_mut() {
            *tf = segment_xf(
                to_bevy(replay.joints[idx][0]),
                to_bevy(replay.joints[idx][1]),
                LINK_R as f32 * RENDER_SCALE,
            );
        }
    }
}
