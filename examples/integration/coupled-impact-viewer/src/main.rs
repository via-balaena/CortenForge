//! Rung 1 of the de-escalation visualization ladder — **see the coupled grip we already
//! simulate**: a rigid "limb" strikes a soft buffer, and we replay BOTH the deforming soft body
//! and the moving rigid limb together in one Bevy window.
//!
//! This is the keystone viz harness every richer scene renders through (finite contact geometry →
//! an articulated "person" → the agent's own soft-over-rigid body → the co-designed encounter). It
//! is pure COMPOSITION of merged pieces — no new physics:
//! - **Capture:** the `StaggeredCoupling` impact rollout (the RQ1 `impact_recoverability` scene)
//!   is stepped headless, recording per step the soft buffer's deformed vertices
//!   (`soft_positions`) and the rigid limb's world pose (`data().xpos`).
//! - **Replay:** a single looping cursor drives BOTH the soft mesh (rewriting its vertex positions
//!   via sim-bevy-soft's `apply_soft_positions`) and the rigid limb (moving its transform) from the
//!   same frame index, so they stay in exact lockstep and the strike→absorb→recover cycle repeats
//!   continuously.
//!
//! ## Run
//! Headless capture + a one-line metric summary always run (CI-safe). The Bevy window is opt-in:
//! ```text
//! CF_VISUAL=1 cargo run -p example-integration-coupled-impact-viewer --release
//! ```
//! Orbit: left-drag rotate · right-drag pan · scroll zoom.

// Bevy systems take `Res`/`Query`/`Commands` by value and reference items rustdoc can't link from
// a binary — the standard allow set for this repo's Bevy examples (mirrors compressive-block).
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::needless_pass_by_value,
    clippy::doc_markdown
)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;
use sim_soft::material::silicone_table::ECOFLEX_00_30_MEASURED;

// ── Scene (mirrors the RQ1 `impact_recoverability` headline strike) ──
const MASS: f64 = 1.0; // kg — a ~1 kg limb segment
const N_PER_EDGE: usize = 4;
const EDGE: f64 = 0.10; // m — soft cube side (spans z ∈ [0, 0.10])
const DT: f64 = 1.0e-3;
const N_STEPS: usize = 500;
const V_IMPACT: f64 = 2.0; // m/s downward strike
const CLEARANCE: f64 = 0.005;
const D_HAT: f64 = 1.0e-2;
const KAPPA: f64 = 3.0e4;
// Limb box half-extents (m), matching the MJCF geom below.
const LIMB_HALF: [f64; 3] = [0.06, 0.06, 0.005];

// ── Replay / render ──
const RENDER_SCALE: f32 = 10.0; // 0.10 m block → 1.0 Bevy unit
const REPLAY_DT: f64 = 0.012; // wall-clock s per frame (≈6 s for 500 steps: strike → absorb → recover)

/// One headless capture of the coupled impact: the soft buffer's deformed-vertex frames + the
/// rigid limb's per-step world position, plus the rest surface mesh for the renderer.
struct Capture {
    rest_positions: Vec<sim_soft::Vec3>,
    boundary_faces: Vec<[sim_soft::VertexId; 3]>,
    soft_frames: Vec<Vec<f64>>,
    limb_poses: Vec<[f64; 3]>,
    peak_force: f64,
    max_compression: f64,
}

fn run_capture() -> Capture {
    let block_top = EDGE;
    let start_z = block_top + D_HAT + CLEARANCE; // band top: just touching, force ≈ 0
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="limb" pos="0.05 0.05 {start_z}">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="{MASS}"/>
    </body>
  </worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).expect("limb MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    data.qvel[2] = -V_IMPACT; // free-joint linear z velocity: a downward strike

    // Modest contact damping (a VIZ choice, not the RQ1 metric): with `rigid_damping = 0` the limb
    // rebounds elastically and flies off-frame, so the strike reads as a one-time blip. A little
    // damping lets it settle onto the buffer (press-and-hold), which animates legibly. The RQ1
    // `impact_recoverability` test keeps `0` for the physical-impact force metric; the peak force
    // here (~259 N) is essentially unchanged, so the absorb story is faithful.
    let mut coupling: StaggeredCoupling = StaggeredCoupling::new(
        model,
        data,
        /* body */ 1,
        CLEARANCE,
        N_PER_EDGE,
        EDGE,
        ECOFLEX_00_30_MEASURED.mu,
        DT,
        KAPPA,
        D_HAT,
        /* rigid_damping */ 6.0,
    );

    // Rest surface mesh (constant topology) + frame 0 (rest config + start pose).
    let boundary_faces = coupling.soft_boundary_faces();
    let rest_positions: Vec<sim_soft::Vec3> = coupling
        .soft_positions()
        .chunks_exact(3)
        .map(|c| sim_soft::Vec3::new(c[0], c[1], c[2]))
        .collect();

    let mut soft_frames: Vec<Vec<f64>> = vec![coupling.soft_positions().to_vec()];
    let mut limb_poses: Vec<[f64; 3]> = vec![{
        let p = coupling.data().xpos[1];
        [p.x, p.y, p.z]
    }];
    let mut peak_force = 0.0_f64;
    let mut min_z = start_z;

    for _ in 0..N_STEPS {
        let s = coupling.step();
        peak_force = peak_force.max(s.force_on_soft.z.abs());
        min_z = min_z.min(s.rigid_z);
        soft_frames.push(coupling.soft_positions().to_vec());
        let p = coupling.data().xpos[1];
        limb_poses.push([p.x, p.y, p.z]);
    }

    Capture {
        rest_positions,
        boundary_faces,
        soft_frames,
        limb_poses,
        peak_force,
        max_compression: (block_top - (min_z - LIMB_HALF[2])).max(0.0),
    }
}

fn main() {
    let cap = run_capture();
    println!(
        "coupled impact captured: {} frames · peak contact force {:.0} N · max compression {:.1}%",
        cap.soft_frames.len(),
        cap.peak_force,
        cap.max_compression / EDGE * 100.0,
    );
    if std::env::var("CF_VISUAL").is_ok() {
        println!(
            "CF_VISUAL set — opening Bevy replay (loops continuously; close window to exit) ..."
        );
        visual::run(cap);
    } else {
        println!(
            "set CF_VISUAL=1 to watch the strike-and-absorb:\n  \
             CF_VISUAL=1 cargo run -p example-integration-coupled-impact-viewer --release"
        );
    }
}

// ─────────────────────────── Bevy replay (visual mode only) ───────────────────────────
mod visual {
    use super::{Capture, EDGE, LIMB_HALF, RENDER_SCALE, REPLAY_DT};
    use bevy::prelude::*;
    use bevy::time::Real;
    use cf_bevy_common::axis::UpAxis;
    use cf_bevy_common::camera::{OrbitCamera, OrbitCameraPlugin};
    use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};

    /// The captured scene, handed to the startup system (taken once).
    #[derive(Resource)]
    struct Scene(Option<Capture>);

    /// Captured frames + the LOOPING replay clock. One shared cursor drives BOTH the soft mesh and
    /// the rigid limb each tick → they stay in exact lockstep, and the strike→absorb→recover cycle
    /// repeats (no clamp-and-freeze, so the limb's brief dive is always replaying, not parked).
    #[derive(Resource)]
    struct Replay {
        soft_frames: Vec<Vec<f64>>,
        limb_poses: Vec<[f64; 3]>,
        dt: f64,
        epoch: Option<f64>,
    }

    #[derive(Component)]
    struct SoftBody;
    #[derive(Component)]
    struct Limb;

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

        // Soft buffer — coral PBR; its mesh positions are rewritten per frame by `replay`.
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

        // Rigid limb — a gray slab, sized from the box half-extents (swapped to Bevy axes), moved
        // each frame to its captured pose. Rotation is omitted (the symmetric vertical strike keeps
        // the free body ~unrotated); Rung 2+ can render the rigid geometry/pose faithfully.
        let limb_mesh = meshes.add(Cuboid::new(
            (2.0 * LIMB_HALF[0]) as f32,
            (2.0 * LIMB_HALF[2]) as f32,
            (2.0 * LIMB_HALF[1]) as f32,
        ));
        let limb_material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.6, 0.62, 0.68),
            perceptual_roughness: 0.6,
            ..default()
        });
        commands.spawn((
            Mesh3d(limb_mesh),
            MeshMaterial3d(limb_material),
            Transform {
                translation: to_bevy(cap.limb_poses[0]),
                scale: Vec3::splat(RENDER_SCALE),
                ..default()
            },
            Limb,
        ));

        // Camera framed on the block COM (physics (0.05, 0.05, 0.05) → Bevy (0.5, 0.5, 0.5)).
        let target = to_bevy([0.5 * EDGE, 0.5 * EDGE, 0.5 * EDGE]);
        commands.spawn((
            Camera3d::default(),
            Transform::default(),
            OrbitCamera::new()
                .with_target(target)
                .with_distance(3.0 * RENDER_SCALE * EDGE as f32)
                .with_angles(0.4, 0.5),
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
            limb_poses: cap.limb_poses,
            dt: REPLAY_DT,
            epoch: None,
        });
    }

    /// One shared looping cursor drives the soft mesh (rewrite vertex positions) and the limb
    /// (move its transform) from the same frame index — exact lockstep, repeating forever.
    fn replay(
        time: Res<Time<Real>>,
        mut replay: ResMut<Replay>,
        up: Res<UpAxis>,
        mut meshes: ResMut<Assets<Mesh>>,
        q_soft: Query<&Mesh3d, With<SoftBody>>,
        mut q_limb: Query<&mut Transform, With<Limb>>,
    ) {
        let now = time.elapsed_secs_f64();
        let epoch = *replay.epoch.get_or_insert(now);
        let n = replay.soft_frames.len();
        let idx = (((now - epoch) / replay.dt) as usize) % n; // modulo → loop
        let soft_mesh = q_soft.single().ok().and_then(|h| meshes.get_mut(&h.0));
        if let Some(mesh) = soft_mesh {
            apply_soft_positions(mesh, &replay.soft_frames[idx], *up);
        }
        if let Ok(mut tf) = q_limb.single_mut() {
            tf.translation = to_bevy(replay.limb_poses[idx]);
        }
    }
}
