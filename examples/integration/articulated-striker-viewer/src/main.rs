//! Rung 3 of the de-escalation visualization ladder — **the articulated "person"**: a 2-link arm
//! (upper arm + forearm) hammer-strikes a fist into the soft buffer, instead of Rung 2's
//! straight-line sphere. This MERGES Rung 1 (articulated kinematics) and Rung 2 (finite-rigid
//! contact): the fist is the finite contact geometry, posed at the arm's end-effector by the arm's
//! forward kinematics.
//!
//! The arm is a hand-computed planar 2-link linkage (closed-form FK + IK — no rigid solver needed
//! for a KINEMATIC swing), rooted directly above the buffer and driven on a scripted HAMMER FIST: it
//! cocks the fist high then drives it straight DOWN onto the top centre (the fist target is
//! interpolated along a vertical line, the elbow flaring on the wind-up), then recocks (cosine ease
//! → a seamless loop). Only the fist is a contact collider (a `SphereSdf`, the Rung 2 path); the arm
//! segments are visual. The arm does not yet feel the buffer's reaction — two-way coupling of a
//! *dynamic* articulated striker is the next beat. Renders both arm segments + the fist + the
//! deforming buffer, all from world points (so no quaternion-axis-swap needed).
//!
//! ## Run
//! Headless capture + a one-line metric summary always run (CI-safe). The Bevy window is opt-in:
//! ```text
//! CF_VISUAL=1 cargo run -p example-integration-articulated-striker-viewer --release
//! ```
//! Orbit: left-drag rotate · right-drag pan · scroll zoom.

// Bevy systems take `Res`/`Query`/`Commands` by value and reference items rustdoc can't link from
// a binary — the standard allow set for this repo's Bevy examples (mirrors the Rung 1/2 siblings).
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::needless_pass_by_value,
    clippy::doc_markdown
)]

use nalgebra::Point3;
use sim_ml_chassis::Tensor;
use sim_soft::material::silicone_table::ECOFLEX_00_30_MEASURED;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh,
    PenaltyRigidContact, PenaltyRigidContactSolver, Sdf, Solver, SolverConfig, SphereSdf, Tet4,
    Vec3, VertexId,
};

// ── Soft buffer (measured-Ecoflex block) ──
// Finer than Rung 2 (6) so a DEEP strike resolves as a local crater AND stays inside the
// NeoHookean validity domain — smaller tets spread the contact strain, so the fist can sink
// further before any element over-stretches and fail-closes.
const N_PER_EDGE: usize = 10;
const EDGE: f64 = 0.10; // m — soft cube side, z ∈ [0, 0.10], base pinned
const DT: f64 = 1.0e-3;
const KAPPA: f64 = 3.0e4;
const D_HAT: f64 = 5.0e-3;

// ── Fist (the finite rigid contact collider) ──
const FIST_R: f64 = 0.04; // 4 cm blunt fist
const MAX_INDENT: f64 = 0.03; // 3 cm peak press — a convincing strike (the fist sinks into a crater)

// ── Arm: a planar 2-link linkage in the x–z plane at y = ARM_Y, rooted DIRECTLY ABOVE the buffer ──
// A HAMMER FIST: the shoulder sits over the buffer centre and the fist is driven straight DOWN onto
// the top (the fist target is interpolated in task space along the vertical line x = STRIKE_X, so its
// path is a clean vertical — not the sideways arc of an off-centre shoulder). The elbow flares out
// on the wind-up and tucks straight on the strike.
const ARM_Y: f64 = 0.05; // swing plane through the buffer's lateral centre
const SHOULDER_X: f64 = 0.5 * EDGE; // directly above the buffer centre
const SHOULDER_Z: f64 = 0.30;
const L1: f64 = 0.11; // upper arm
const L2: f64 = 0.10; // forearm
const LINK_R: f64 = 0.012; // limb render radius (visual only)
// Fist CENTRE travels the vertical line x = STRIKE_X between RETRACT_Z (cocked high) and STRIKE_Z
// (driven in by MAX_INDENT — fist south pole = EDGE − MAX_INDENT at the buffer top centre).
const STRIKE_X: f64 = 0.5 * EDGE;
const STRIKE_Z: f64 = EDGE - MAX_INDENT + FIST_R;
const RETRACT_Z: f64 = 0.24; // fist cocked high above the buffer (clear of it)
const N_STEPS: usize = 120; // cock → hammer down → recock (cosine ease → seamless loop)

// ── Replay / render ──
const RENDER_SCALE: f32 = 10.0; // 0.10 m block → 1.0 Bevy unit
const REPLAY_DT: f64 = 0.035;

/// A finite [`Sdf`] posed at a world `center` — `SphereSdf` is origin-centered, so to drive it we
/// translate the query point. (A sphere needs no rotation; translation alone poses it.)
struct TranslatedSdf<S> {
    inner: S,
    center: Vec3,
}
impl<S: Sdf> Sdf for TranslatedSdf<S> {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.inner.eval(p - self.center)
    }
    fn grad(&self, p: Point3<f64>) -> Vec3 {
        self.inner.grad(p - self.center)
    }
}

/// Closed-form 2-link planar IK in the x–z plane: joint angles `(θ1, θ2)` (from +x, elbow-down)
/// that place the fist at `(tx, tz)`. `θ1` is the upper-arm angle, `θ2` the elbow angle.
fn ik(tx: f64, tz: f64) -> (f64, f64) {
    let (dx, dz) = (tx - SHOULDER_X, tz - SHOULDER_Z);
    let r = dx.hypot(dz).clamp((L1 - L2).abs() + 1e-4, L1 + L2 - 1e-4);
    let cos2 = ((r * r - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)).clamp(-1.0, 1.0);
    let th2 = -cos2.acos(); // elbow-down branch (the arm bends "above" the reach line)
    let th1 = dz.atan2(dx) - (L2 * th2.sin()).atan2(L1 + L2 * th2.cos());
    (th1, th2)
}

/// Forward kinematics: world points `(elbow, fist)` for joint angles `(θ1, θ2)`.
fn fk(th1: f64, th2: f64) -> (Vec3, Vec3) {
    let ex = SHOULDER_X + L1 * th1.cos();
    let ez = SHOULDER_Z + L1 * th1.sin();
    let fx = ex + L2 * (th1 + th2).cos();
    let fz = ez + L2 * (th1 + th2).sin();
    (Vec3::new(ex, ARM_Y, ez), Vec3::new(fx, ARM_Y, fz))
}

/// The three arm joints `[shoulder, elbow, fist]` (world) at step `k`. The fist target travels the
/// vertical line `x = STRIKE_X` from `RETRACT_Z` (cocked) to `STRIKE_Z` (struck) on a cosine ease
/// (0 → 1 → 0) — TASK-space interpolation, so the fist path is a clean vertical (a hammer fist), with
/// the elbow derived per frame by IK.
fn arm_joints(k: usize) -> [Vec3; 3] {
    let ease = 0.5 * (1.0 - (2.0 * std::f64::consts::PI * k as f64 / N_STEPS as f64).cos());
    let fist_z = RETRACT_Z + (STRIKE_Z - RETRACT_Z) * ease;
    let (th1, th2) = ik(STRIKE_X, fist_z);
    let (elbow, fist) = fk(th1, th2);
    [Vec3::new(SHOULDER_X, ARM_Y, SHOULDER_Z), elbow, fist]
}

/// One headless capture: soft buffer deformed-vertex frames + the three arm joints per step, plus
/// the rest surface mesh for the renderer.
struct Capture {
    rest_positions: Vec<Vec3>,
    boundary_faces: Vec<[VertexId; 3]>,
    soft_frames: Vec<Vec<f64>>,
    joints: Vec<[[f64; 3]; 3]>,
    peak_force: f64,
    max_indent_reached: f64,
}

fn run_capture() -> Capture {
    let field = MaterialField::uniform(ECOFLEX_00_30_MEASURED.mu, 4.0 * ECOFLEX_00_30_MEASURED.mu);
    let mesh = HandBuiltTetMesh::uniform_block(N_PER_EDGE, EDGE, &field);
    let n = mesh.n_vertices();
    let pinned: Vec<VertexId> = sim_soft::pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    assert!(!pinned.is_empty(), "soft block has no z=0 base to pin");

    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    let boundary_faces = mesh.boundary_faces().to_vec();
    let mut x: Vec<f64> = rest_positions
        .iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .collect();
    let mut v = vec![0.0_f64; 3 * n];

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 80;
    let theta = Tensor::zeros(&[0]);

    let joint_pt = |j: Vec3| [j.x, j.y, j.z];

    let mut soft_frames: Vec<Vec<f64>> = vec![x.clone()];
    let mut joints: Vec<[[f64; 3]; 3]> = vec![arm_joints(0).map(joint_pt)];
    let mut peak_force = 0.0_f64;
    let mut max_indent_reached = 0.0_f64;

    for k in 1..=N_STEPS {
        let j = arm_joints(k);
        let fist = j[2];
        // Re-pose the finite fist by rebuilding the contact at its FK centre (the Rung 2 pattern).
        let contact = PenaltyRigidContact::with_params(
            vec![TranslatedSdf {
                inner: SphereSdf { radius: FIST_R },
                center: fist,
            }],
            KAPPA,
            D_HAT,
        );
        let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(
            Tet4,
            mesh.clone(),
            contact,
            cfg,
            BoundaryConditions::new(pinned.clone(), Vec::new()),
        );
        let x_t = Tensor::from_slice(&x, &[3 * n]);
        let v_t = Tensor::from_slice(&v, &[3 * n]);
        let x_final = solver.replay_step(&x_t, &v_t, &theta, DT).x_final;
        for i in 0..3 * n {
            v[i] = (x_final[i] - x[i]) / DT;
        }
        x = x_final;

        let positions: Vec<Vec3> = x
            .chunks_exact(3)
            .map(|c| Vec3::new(c[0], c[1], c[2]))
            .collect();
        let readout = PenaltyRigidContact::with_params(
            vec![TranslatedSdf {
                inner: SphereSdf { radius: FIST_R },
                center: fist,
            }],
            KAPPA,
            D_HAT,
        );
        let fz: f64 = readout
            .per_pair_readout(&mesh, &positions)
            .iter()
            .map(|r| r.force_on_soft.z)
            .sum();
        peak_force = peak_force.max(fz.abs());
        let deepest_push = positions
            .iter()
            .zip(&rest_positions)
            .map(|(p, r)| r.z - p.z)
            .fold(0.0_f64, f64::max);
        max_indent_reached = max_indent_reached.max(deepest_push);

        soft_frames.push(x.clone());
        joints.push(j.map(joint_pt));
    }

    Capture {
        rest_positions,
        boundary_faces,
        soft_frames,
        joints,
        peak_force,
        max_indent_reached,
    }
}

fn main() {
    let cap = run_capture();
    println!(
        "articulated striker captured: {} frames · peak contact force {:.0} N · max indent {:.1}%",
        cap.soft_frames.len(),
        cap.peak_force,
        cap.max_indent_reached / EDGE * 100.0,
    );
    // Diagnostic: the fist's z-arc + the deepest fist position (does the lunge actually strike?).
    let fist_z = |i: usize| cap.joints[i][2][2];
    let (mut strike_idx, mut min_fz) = (0usize, f64::MAX);
    for i in 0..cap.joints.len() {
        if fist_z(i) < min_fz {
            min_fz = fist_z(i);
            strike_idx = i;
        }
    }
    let elbow_z_at_strike = cap.joints[strike_idx][1][2];
    println!(
        "  fist z: start {:.3} → deepest {:.3} (frame {strike_idx}, south pole {:.3}, elbow z {:.3})",
        fist_z(0),
        fist_z(strike_idx),
        fist_z(strike_idx) - FIST_R,
        elbow_z_at_strike,
    );

    if std::env::var("CF_VISUAL").is_ok() {
        println!(
            "CF_VISUAL set — opening Bevy replay (loops continuously; close window to exit) ..."
        );
        visual::run(cap);
    } else {
        println!(
            "set CF_VISUAL=1 to watch the arm lunge into the buffer:\n  \
             CF_VISUAL=1 cargo run -p example-integration-articulated-striker-viewer --release"
        );
    }
}

// ─────────────────────────── Bevy replay (visual mode only) ───────────────────────────
mod visual {
    use super::{Capture, EDGE, FIST_R, LINK_R, RENDER_SCALE, REPLAY_DT};
    use bevy::prelude::*;
    use bevy::time::Real;
    use cf_bevy_common::axis::UpAxis;
    use cf_bevy_common::camera::{OrbitCamera, OrbitCameraPlugin};
    use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};

    #[derive(Resource)]
    struct Scene(Option<Capture>);

    /// Captured frames + the LOOPING replay clock. One shared cursor drives the soft mesh, the fist,
    /// and the two arm segments each tick → exact lockstep, repeating the lunge.
    #[derive(Resource)]
    struct Replay {
        soft_frames: Vec<Vec<f64>>,
        joints: Vec<[[f64; 3]; 3]>, // [shoulder, elbow, fist] per frame
        dt: f64,
        epoch: Option<f64>,
    }

    #[derive(Component)]
    struct SoftBody;
    #[derive(Component)]
    struct Fist;
    /// An arm segment between joints `[i, i+1]` (0 = shoulder→elbow, 1 = elbow→fist).
    #[derive(Component)]
    struct Segment(usize);

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

    /// Physics (x, y, z) +Z-up → Bevy (x, z, y) +Y-up, scaled to render units.
    fn to_bevy(p: [f64; 3]) -> Vec3 {
        Vec3::new(p[0] as f32, p[2] as f32, p[1] as f32) * RENDER_SCALE
    }

    /// Transform for a unit cylinder (radius 1, height 1, axis +Y) so it spans Bevy points `a → b`
    /// with world radius `r`. Orientation comes from the two endpoints — no quaternion-axis-swap.
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

        // Soft buffer — coral PBR; positions rewritten per frame by `replay`.
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

        // Fist — a gray sphere of the real radius, moved each frame to the FK end-effector.
        let fist_mesh = meshes.add(Sphere::new(FIST_R as f32));
        commands.spawn((
            Mesh3d(fist_mesh),
            MeshMaterial3d(limb_material.clone()),
            Transform {
                translation: to_bevy(cap.joints[0][2]),
                scale: Vec3::splat(RENDER_SCALE),
                ..default()
            },
            Fist,
        ));

        // Two arm segments — unit cylinders re-posed each frame to span their joint pair.
        let seg_mesh = meshes.add(Cylinder::new(1.0, 1.0));
        for s in 0..2 {
            let a = to_bevy(cap.joints[0][s]);
            let b = to_bevy(cap.joints[0][s + 1]);
            commands.spawn((
                Mesh3d(seg_mesh.clone()),
                MeshMaterial3d(limb_material.clone()),
                segment_xf(a, b, LINK_R as f32 * RENDER_SCALE),
                Segment(s),
            ));
        }

        // Camera framed on the block COM (physics (0.05, 0.05, 0.05) → Bevy (0.5, 0.5, 0.5)).
        // Target between the buffer top and the arm so the whole linkage frames; a wider distance
        // because the shoulder sits well above the buffer.
        let target = Vec3::new(
            0.5 * EDGE as f32 * RENDER_SCALE,
            1.4,
            0.5 * EDGE as f32 * RENDER_SCALE,
        );
        commands.spawn((
            Camera3d::default(),
            Transform::default(),
            OrbitCamera::new()
                .with_target(target)
                .with_distance(5.5 * RENDER_SCALE * EDGE as f32)
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

    /// One shared looping cursor drives the soft mesh, the fist, and the two arm segments from the
    /// same frame index — exact lockstep, repeating forever.
    fn replay(
        time: Res<Time<Real>>,
        mut replay: ResMut<Replay>,
        up: Res<UpAxis>,
        mut meshes: ResMut<Assets<Mesh>>,
        q_soft: Query<&Mesh3d, With<SoftBody>>,
        mut q_fist: Query<&mut Transform, (With<Fist>, Without<Segment>)>,
        mut q_segs: Query<(&mut Transform, &Segment), Without<Fist>>,
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
            tf.translation = to_bevy(replay.joints[idx][2]);
        }
        for (mut tf, seg) in &mut q_segs {
            let a = to_bevy(replay.joints[idx][seg.0]);
            let b = to_bevy(replay.joints[idx][seg.0 + 1]);
            *tf = segment_xf(a, b, LINK_R as f32 * RENDER_SCALE);
        }
    }
}
