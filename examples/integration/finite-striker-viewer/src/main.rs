//! Rung 2 of the de-escalation visualization ladder — **finite contact geometry**: a finite rigid
//! "striker" (a sphere — a stand-in for a hand/fist) presses into the soft buffer and indents it
//! LOCALLY, instead of the infinite flat plane of Rung 1. The "person" is now a *thing*, not a
//! half-space, so the contact is a localized dimple where the striker actually is.
//!
//! Builds on Rung 1's viz harness (the same looping lockstep replay) and adds the new physics
//! capability: soft-vs-FINITE-rigid contact. The striker is an analytic sphere posed each step at a
//! scripted centre via `StaggeredCoupling::set_sphere_center`, and the buffer responds through the
//! coupling's one-way `step_kinematic` (no infinite `RigidPlane`). The striker is KINEMATIC here —
//! driven on a prescribed press-in/press-out path (the buffer feels the contact; the striker does
//! not feel the reaction). Coupling that reaction back to a *dynamic* rigid striker is a later rung
//! (Rung 3b, `step_articulated`); this rung isolates the finite-geometry contact.
//!
//! ## Run
//! Headless capture + a one-line metric summary always run (CI-safe). The Bevy window is opt-in:
//! ```text
//! CF_VISUAL=1 cargo run -p example-integration-finite-striker-viewer --release
//! ```
//! Orbit: left-drag rotate · right-drag pan · scroll zoom.

// Bevy systems take `Res`/`Query`/`Commands` by value and reference items rustdoc can't link from
// a binary — the standard allow set for this repo's Bevy examples (mirrors the Rung 1 sibling
// coupled-impact-viewer).
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
use sim_soft::{Vec3, VertexId};

// ── Soft buffer (same measured-Ecoflex block as Rung 1) ──
const N_PER_EDGE: usize = 6; // finer than Rung 1 (4) so the local dimple resolves
const EDGE: f64 = 0.10; // m — soft cube side, z ∈ [0, 0.10], base pinned
const DT: f64 = 1.0e-3;
const KAPPA: f64 = 3.0e4;
const D_HAT: f64 = 5.0e-3;

// ── Striker (the finite rigid "hand/fist") ──
// A BLUNT sphere + modest indent: a sharp/deep poke over-stretches a local tet past the NeoHookean
// validity bound (fail-closed). A blunter striker spreads the deformation over more elements.
const STRIKER_R: f64 = 0.04; // 4 cm sphere (blunt — spreads the contact)
const MAX_INDENT: f64 = 0.015; // 1.5 cm peak press into the top face (~15% of edge)
const N_STEPS: usize = 120; // press-in then press-out (cosine ease → seamless loop)

// ── Replay / render ──
const RENDER_SCALE: f32 = 10.0; // 0.10 m block → 1.0 Bevy unit
const REPLAY_DT: f64 = 0.035; // wall-clock s per frame (≈4 s for 120 steps)

/// Striker centre at step `k`: laterally over the block, pressed down by a cosine-eased depth that
/// goes 0 → `MAX_INDENT` → 0 over the rollout (so the indent forms and releases, and the loop is
/// seamless). At depth 0 the sphere's south pole just touches the rest top face (z = `EDGE`).
fn striker_center(k: usize) -> Vec3 {
    let phase = 2.0 * std::f64::consts::PI * k as f64 / N_STEPS as f64;
    let depth = MAX_INDENT * 0.5 * (1.0 - phase.cos());
    Vec3::new(0.5 * EDGE, 0.5 * EDGE, EDGE + STRIKER_R - depth)
}

/// One headless capture of the finite-striker indent: the soft buffer's deformed-vertex frames +
/// the striker centre per step, plus the rest surface mesh for the renderer.
struct Capture {
    rest_positions: Vec<Vec3>,
    boundary_faces: Vec<[VertexId; 3]>,
    soft_frames: Vec<Vec<f64>>,
    striker_centers: Vec<[f64; 3]>,
    peak_force: f64,
    max_indent_reached: f64,
}

fn run_capture() -> Capture {
    // The striker is KINEMATIC — a scripted vertical path, no dynamic rigid body. A static anchor
    // body just hosts the coupling (which owns the soft buffer); `step_kinematic` poses the fist at
    // `striker_center(k)` via `set_sphere_center` and takes one soft step (no reaction fed back).
    let model = load_model(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="anchor"><geom type="sphere" size="0.01"/></body>
  </worldbody>
</mujoco>"#,
    )
    .expect("anchor MJCF loads");
    let data = model.make_data();
    let mu = ECOFLEX_00_30_MEASURED.mu;
    let mut c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, N_PER_EDGE, EDGE, mu, DT, KAPPA, D_HAT, 0.0,
    )
    .with_sphere_collider(STRIKER_R);

    let to_vec3 = |flat: &[f64]| -> Vec<Vec3> {
        flat.chunks_exact(3)
            .map(|p| Vec3::new(p[0], p[1], p[2]))
            .collect()
    };
    let rest_positions = to_vec3(c.soft_positions());
    let boundary_faces: Vec<[VertexId; 3]> = c.soft_boundary_faces();

    let mut soft_frames: Vec<Vec<f64>> = vec![c.soft_positions().to_vec()];
    let mut striker_centers: Vec<[f64; 3]> = vec![{
        let s = striker_center(0);
        [s.x, s.y, s.z]
    }];
    let mut peak_force = 0.0_f64;
    let mut max_indent_reached = 0.0_f64;

    for k in 1..=N_STEPS {
        let center = striker_center(k);
        c.set_sphere_center(center);
        let step = c.step_kinematic();

        // Instrument: total normal force on the soft body + the deepest indent of the top face
        // (the deepest a vertex was pushed DOWN from rest — the dimple under the striker).
        peak_force = peak_force.max(step.force_on_soft.z.abs());
        let deepest_push = to_vec3(c.soft_positions())
            .iter()
            .zip(&rest_positions)
            .map(|(p, r)| r.z - p.z)
            .fold(0.0_f64, f64::max);
        max_indent_reached = max_indent_reached.max(deepest_push);

        soft_frames.push(c.soft_positions().to_vec());
        striker_centers.push([center.x, center.y, center.z]);
    }

    Capture {
        rest_positions,
        boundary_faces,
        soft_frames,
        striker_centers,
        peak_force,
        max_indent_reached,
    }
}

fn main() {
    let cap = run_capture();
    println!(
        "finite-striker indent captured: {} frames · peak contact force {:.0} N · max indent {:.1}%",
        cap.soft_frames.len(),
        cap.peak_force,
        cap.max_indent_reached / EDGE * 100.0,
    );
    if std::env::var("CF_VISUAL").is_ok() {
        println!(
            "CF_VISUAL set — opening Bevy replay (loops continuously; close window to exit) ..."
        );
        visual::run(cap);
    } else {
        println!(
            "set CF_VISUAL=1 to watch the striker press into the buffer:\n  \
             CF_VISUAL=1 cargo run -p example-integration-finite-striker-viewer --release"
        );
    }
}

// ─────────────────────────── Bevy replay (visual mode only) ───────────────────────────
mod visual {
    use super::{Capture, EDGE, RENDER_SCALE, REPLAY_DT, STRIKER_R};
    use bevy::prelude::*;
    use bevy::time::Real;
    use cf_bevy_common::axis::UpAxis;
    use cf_bevy_common::camera::{OrbitCamera, OrbitCameraPlugin};
    use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};

    /// The captured scene, handed to the startup system (taken once).
    #[derive(Resource)]
    struct Scene(Option<Capture>);

    /// Captured frames + the LOOPING replay clock. One shared cursor drives BOTH the soft mesh and
    /// the striker each tick → exact lockstep, and the press-in/press-out cycle repeats.
    #[derive(Resource)]
    struct Replay {
        soft_frames: Vec<Vec<f64>>,
        striker_centers: Vec<[f64; 3]>,
        dt: f64,
        epoch: Option<f64>,
    }

    #[derive(Component)]
    struct SoftBody;
    #[derive(Component)]
    struct Striker;

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

        // Striker — a gray sphere of the real radius, moved each frame to its captured centre.
        let striker_mesh = meshes.add(Sphere::new(STRIKER_R as f32));
        let striker_material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.6, 0.62, 0.68),
            perceptual_roughness: 0.6,
            ..default()
        });
        commands.spawn((
            Mesh3d(striker_mesh),
            MeshMaterial3d(striker_material),
            Transform {
                translation: to_bevy(cap.striker_centers[0]),
                scale: Vec3::splat(RENDER_SCALE),
                ..default()
            },
            Striker,
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
            striker_centers: cap.striker_centers,
            dt: REPLAY_DT,
            epoch: None,
        });
    }

    /// One shared looping cursor drives the soft mesh (rewrite vertex positions) and the striker
    /// (move its transform) from the same frame index — exact lockstep, repeating forever.
    fn replay(
        time: Res<Time<Real>>,
        mut replay: ResMut<Replay>,
        up: Res<UpAxis>,
        mut meshes: ResMut<Assets<Mesh>>,
        q_soft: Query<&Mesh3d, With<SoftBody>>,
        mut q_striker: Query<&mut Transform, With<Striker>>,
    ) {
        let now = time.elapsed_secs_f64();
        let epoch = *replay.epoch.get_or_insert(now);
        let n = replay.soft_frames.len();
        let idx = (((now - epoch) / replay.dt) as usize) % n; // modulo → loop
        let soft_mesh = q_soft.single().ok().and_then(|h| meshes.get_mut(&h.0));
        if let Some(mesh) = soft_mesh {
            apply_soft_positions(mesh, &replay.soft_frames[idx], *up);
        }
        if let Ok(mut tf) = q_striker.single_mut() {
            tf.translation = to_bevy(replay.striker_centers[idx]);
        }
    }
}
