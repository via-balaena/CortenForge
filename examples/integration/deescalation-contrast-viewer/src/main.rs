//! Rung R5 of the de-escalation visualization ladder — **the measured pressure-vs-force story,
//! side by side**: three coupled impact rollouts replay together, each with a live readout of the
//! quantities the contact-pressure foundation now MEASURES (total force AND peak per-face pressure).
//!
//! The three panels share one striker and one buffer geometry, varying ONE thing each so two
//! contrasts read straight across:
//!
//! ```text
//!   [ sphere · soft ]      [ slab · soft ]      [ slab · hard ]
//!      └──── pressure story ────┘  └──── recoverability story ────┘
//!      (vary contact geometry)        (vary buffer stiffness)
//! ```
//!
//! - **Pressure story** (left ↔ middle, same soft Ecoflex buffer): a CONCENTRATED finite sphere
//!   squeezes the load onto a small patch → HIGH peak pressure at LOW total force; the BROAD slab
//!   spreads it → low pressure, high force. The number that flips is the whole point — and total
//!   force alone cannot see it (the gap PRs #442–#445 closed; this is the picture of that result).
//! - **Recoverability story** (middle ↔ right, same broad slab contact): the soft buffer ABSORBS
//!   the strike (stays under the threshold) while the ~12× stiffer hard shell TRANSMITS it (the RQ1
//!   `impact_recoverability` contrast).
//!
//! Each panel's soft-mesh DENT shape carries the geometry story visually (a sphere dimple vs a flat
//! broad compression); the HUD carries the measured force / pressure. The animation is the SAME
//! `StaggeredCoupling` rollout the gate test (`coupled_peak_pressure.rs`) measures — no re-derivation.
//!
//! ## Run
//! Headless capture + a contrast table always run (CI-safe). The Bevy window is opt-in:
//! ```text
//! CF_VISUAL=1 cargo run -p example-integration-deescalation-contrast-viewer --release
//! ```
//! Orbit: left-drag rotate · right-drag pan · scroll zoom.

// Bevy systems take `Res`/`Query`/`Commands` by value and reference items rustdoc can't link from
// a binary — the standard allow set for this repo's Bevy examples (mirrors coupled-impact-viewer).
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::needless_pass_by_value,
    clippy::doc_markdown,
    // The Bevy `setup` system spawns three panels + camera + lights + HUD in one coherent pass;
    // splitting it would scatter scene construction without improving clarity (mirrors siblings).
    clippy::too_many_lines
)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;
use sim_soft::material::silicone_table::{DRAGON_SKIN_30A, ECOFLEX_00_30_MEASURED};

// ── Scene (mirrors the RQ1 `impact_recoverability` / coupled-impact-viewer strike) ──
const MASS: f64 = 1.0; // kg — a ~1 kg limb segment
const N_PER_EDGE: usize = 4;
const EDGE: f64 = 0.10; // m — soft cube side (spans z ∈ [0, 0.10])
const DT: f64 = 1.0e-3;
const N_STEPS: usize = 220;
const V_IMPACT: f64 = 1.5; // m/s downward strike (stable for the finite sphere on the coarse mesh)
const CLEARANCE: f64 = 0.005;
const D_HAT: f64 = 1.0e-2;
const KAPPA: f64 = 3.0e4;
// Modest contact damping — a VIZ choice (lets the limb settle onto the buffer instead of bouncing
// off-frame), consistent across all panels so the comparison stays fair. The RQ1 metric uses 0.
const RIGID_DAMPING: f64 = 6.0;
const SPHERE_R: f64 = 0.04; // the concentrated collider's radius
const LIMB_HALF: [f64; 3] = [0.06, 0.06, 0.005]; // box half-extents (m), matching the MJCF geom

// Recoverability thresholds (N), cited (`project-deescalation-injury-thresholds`).
const ISO_RECOVERABLE_N: f64 = 270.0; // pain-onset
const FRACTURE_CLIFF_N: f64 = 450.0; // irreversible

// ── Replay / render ──
const RENDER_SCALE: f32 = 10.0; // 0.10 m block → 1.0 Bevy unit
const REPLAY_DT: f64 = 0.05; // wall-clock s per replay frame — slow-mo so the strike + heatmap read
const PANEL_GAP: f64 = 0.16; // m between adjacent panel centres along x

#[derive(Clone, Copy)]
enum Collider {
    Slab,
    Sphere,
}

/// One panel's scene: a label, the contact geometry, and the buffer stiffness.
struct SceneCfg {
    title: &'static str,
    sub: &'static str,
    collider: Collider,
    mu: f64,
}

const fn scenes() -> [SceneCfg; 3] {
    [
        SceneCfg {
            title: "SPHERE - soft",
            sub: "concentrated",
            collider: Collider::Sphere,
            mu: ECOFLEX_00_30_MEASURED.mu,
        },
        SceneCfg {
            title: "SLAB - soft",
            sub: "broad / Ecoflex",
            collider: Collider::Slab,
            mu: ECOFLEX_00_30_MEASURED.mu,
        },
        SceneCfg {
            title: "SLAB - hard",
            sub: "broad / Dragon Skin",
            collider: Collider::Slab,
            mu: DRAGON_SKIN_30A.mu,
        },
    ]
}

/// One headless capture: the soft buffer's deformed-vertex frames + the rigid limb's per-step pose,
/// plus per-frame measured force / peak pressure for the HUD, and the rest mesh for the renderer.
struct Panel {
    title: String,
    sub: String,
    is_sphere: bool,
    rest_positions: Vec<sim_soft::Vec3>,
    boundary_faces: Vec<[sim_soft::VertexId; 3]>,
    soft_frames: Vec<Vec<f64>>,
    // Per-vertex contact pressure field per frame (Pa, indexed by VertexId, 0 off-contact) — the
    // local concentration that paints the deformed-surface heatmap (`contact_vertex_pressures`).
    pressure_fields: Vec<Vec<f32>>,
    striker_poses: Vec<[f64; 3]>,
    forces: Vec<f64>,    // |force_on_soft| per frame (N)
    pressures: Vec<f64>, // peak per-face pressure per frame (Pa); non-finite → 0.0
    peak_force: f64,
    peak_pressure: f64,
}

fn run_capture(cfg: &SceneCfg) -> Panel {
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

    let mut coupling: StaggeredCoupling = StaggeredCoupling::new(
        model,
        data,
        /* body */ 1,
        CLEARANCE,
        N_PER_EDGE,
        EDGE,
        cfg.mu,
        DT,
        KAPPA,
        D_HAT,
        RIGID_DAMPING,
    );
    if matches!(cfg.collider, Collider::Sphere) {
        coupling = coupling.with_sphere_collider(SPHERE_R);
    }

    let boundary_faces = coupling.soft_boundary_faces();
    let rest_positions: Vec<sim_soft::Vec3> = coupling
        .soft_positions()
        .chunks_exact(3)
        .map(|c| sim_soft::Vec3::new(c[0], c[1], c[2]))
        .collect();

    let n_vertices = rest_positions.len();

    // Striker RENDER centre: the slab is the limb box (its body pose), but the finite sphere
    // collider is posed over the block-top centroid at `z = plane_height + radius`
    // (`plane_height = xpos.z − clearance`), 0.035 m above the body origin — render it there so the
    // ball sits ON its dent, not buried past it.
    let striker_pos = |xpos: [f64; 3]| -> [f64; 3] {
        if matches!(cfg.collider, Collider::Sphere) {
            [EDGE / 2.0, EDGE / 2.0, xpos[2] - CLEARANCE + SPHERE_R]
        } else {
            xpos
        }
    };
    let body_xpos = |c: &StaggeredCoupling| {
        let p = c.data().xpos[1];
        [p.x, p.y, p.z]
    };

    let mut soft_frames: Vec<Vec<f64>> = vec![coupling.soft_positions().to_vec()];
    let mut pressure_fields: Vec<Vec<f32>> = vec![vec![0.0; n_vertices]]; // rest: no contact
    let mut striker_poses: Vec<[f64; 3]> = vec![striker_pos(body_xpos(&coupling))];
    let mut forces: Vec<f64> = vec![0.0];
    let mut pressures: Vec<f64> = vec![0.0];

    for _ in 0..N_STEPS {
        // `step_with_pressure_field` evaluates the per-vertex field at the SAME posed contact it
        // solves (no one-timestep lag), so the heatmap matches the just-captured deformation.
        let (s, field) = coupling.step_with_pressure_field();
        forces.push(s.force_on_soft.norm());
        // The peak_pressure / field carry the NaN sentinel for all-degenerate contact; the HUD reads
        // a finite scalar, so fold non-finite to 0.0 (the field keeps NaN for the heatmap to flag).
        pressures.push(if s.peak_pressure.is_finite() {
            s.peak_pressure
        } else {
            0.0
        });
        soft_frames.push(coupling.soft_positions().to_vec());
        pressure_fields.push(field.iter().map(|&v| v as f32).collect());
        striker_poses.push(striker_pos(body_xpos(&coupling)));
    }

    Panel {
        title: cfg.title.to_string(),
        sub: cfg.sub.to_string(),
        is_sphere: matches!(cfg.collider, Collider::Sphere),
        rest_positions,
        boundary_faces,
        soft_frames,
        pressure_fields,
        peak_force: forces.iter().copied().fold(0.0, f64::max),
        peak_pressure: pressures.iter().copied().fold(0.0, f64::max),
        striker_poses,
        forces,
        pressures,
    }
}

fn main() {
    let panels: Vec<Panel> = scenes().iter().map(run_capture).collect();

    println!("\n=== De-escalation contrast — measured force AND peak pressure ===");
    println!(
        "{:<22} {:>14} {:>16}",
        "panel", "peak force (N)", "peak press (kPa)"
    );
    for p in &panels {
        println!(
            "{:<22} {:>14.1} {:>16.1}",
            format!("{} ({})", p.title, p.sub),
            p.peak_force,
            p.peak_pressure / 1e3,
        );
    }

    // The two falsifiable contrasts the viewer renders (panels: 0=sphere·soft, 1=slab·soft, 2=slab·hard).
    let (sphere_soft, slab_soft, slab_hard) = (&panels[0], &panels[1], &panels[2]);
    // Pressure story: the concentrated sphere reads HIGHER peak pressure but LOWER total force.
    assert!(
        sphere_soft.peak_pressure > slab_soft.peak_pressure,
        "sphere should read higher peak pressure than the slab ({:.0} vs {:.0} Pa)",
        sphere_soft.peak_pressure,
        slab_soft.peak_pressure,
    );
    assert!(
        sphere_soft.peak_force < slab_soft.peak_force,
        "sphere should read lower total force than the slab ({:.1} vs {:.1} N)",
        sphere_soft.peak_force,
        slab_soft.peak_force,
    );
    // Recoverability story: the hard shell transmits MORE force than the soft buffer (same contact).
    assert!(
        slab_hard.peak_force > slab_soft.peak_force,
        "hard shell should transmit more force than the soft buffer ({:.1} vs {:.1} N)",
        slab_hard.peak_force,
        slab_soft.peak_force,
    );
    println!(
        "\npressure story: sphere {:.0} kPa / {:.0} N  vs  slab {:.0} kPa / {:.0} N",
        sphere_soft.peak_pressure / 1e3,
        sphere_soft.peak_force,
        slab_soft.peak_pressure / 1e3,
        slab_soft.peak_force,
    );
    println!(
        "recoverability story: soft {:.0} N  vs  hard {:.0} N (cliff {FRACTURE_CLIFF_N:.0} N)",
        slab_soft.peak_force, slab_hard.peak_force,
    );

    if std::env::var("CF_VISUAL").is_ok() {
        println!(
            "\nCF_VISUAL set — opening Bevy replay (loops continuously; close window to exit) ..."
        );
        // Shared pressure colour scale across panels (the global peak) so the heatmap intensity is
        // comparable cell-to-cell — a hot patch means the same Pa in every panel.
        let scale = panels
            .iter()
            .map(|p| p.peak_pressure)
            .fold(0.0_f64, f64::max)
            .max(1.0) as f32;
        visual::run(panels, scale);
    } else {
        println!(
            "\nset CF_VISUAL=1 to watch the three rollouts side by side:\n  \
             CF_VISUAL=1 cargo run -p example-integration-deescalation-contrast-viewer --release"
        );
    }
}

// ─────────────────────────── Bevy replay (visual mode only) ───────────────────────────
mod visual {
    use super::{
        EDGE, FRACTURE_CLIFF_N, ISO_RECOVERABLE_N, LIMB_HALF, PANEL_GAP, Panel, RENDER_SCALE,
        REPLAY_DT,
    };
    use bevy::prelude::*;
    use bevy::time::Real;
    use cf_bevy_common::axis::UpAxis;
    use cf_bevy_common::camera::{OrbitCamera, OrbitCameraPlugin};
    use sim_bevy_soft::prelude::{apply_soft_positions, build_soft_mesh};
    use std::fmt::Write as _;

    #[derive(Resource)]
    struct Scene {
        panels: Option<Vec<Panel>>,
        scale: f32,
    }

    /// One panel's replay data (frames + per-frame HUD readouts + the heatmap field).
    struct PanelReplay {
        title: String,
        sub: String,
        soft_frames: Vec<Vec<f64>>,
        pressure_fields: Vec<Vec<f32>>,
        striker_poses: Vec<[f64; 3]>,
        forces: Vec<f64>,
        pressures: Vec<f64>,
    }

    #[derive(Resource)]
    struct Replay {
        panels: Vec<PanelReplay>,
        scale: f32,
        dt: f64,
        epoch: Option<f64>,
        last_idx: Option<usize>,
    }

    /// Panel index carried on each renderable so the replay system updates the right scene.
    #[derive(Component)]
    struct SoftBody(usize);
    #[derive(Component)]
    struct Limb(usize);
    #[derive(Component)]
    struct Hud;

    pub fn run(panels: Vec<Panel>, scale: f32) {
        App::new()
            .add_plugins(DefaultPlugins)
            .add_plugins(OrbitCameraPlugin)
            .insert_resource(UpAxis::PlusZ)
            .insert_resource(Scene {
                panels: Some(panels),
                scale,
            })
            .add_systems(Startup, setup)
            .add_systems(Update, replay)
            .run();
    }

    /// Map a per-vertex pressure fraction `t ∈ [0, 1]` (`pressure / global_peak`) to an RGBA vertex
    /// colour: the calm peach buffer reddening as it is squeezed, then glowing yellow at peak. Red
    /// against light peach is high-contrast and reads as "stress / heat" — the right semantics for
    /// the injury story. `t^0.6` lifts the low end so moderate pressure is already visible.
    /// Multiplies a WHITE material, so this colour IS the surface colour.
    fn heat(t: f32) -> [f32; 4] {
        let t = t.clamp(0.0, 1.0).powf(0.6);
        let cool = [1.0_f32, 0.72, 0.55]; // peach — rest / unloaded
        let mid = [0.90_f32, 0.13, 0.08]; // vivid red — squeezed
        let hot = [1.0_f32, 0.92, 0.30]; // glowing yellow — peak concentration
        let (a, b, u) = if t < 0.6 {
            (cool, mid, t / 0.6)
        } else {
            (mid, hot, (t - 0.6) / 0.4)
        };
        [
            a[0] + (b[0] - a[0]) * u,
            a[1] + (b[1] - a[1]) * u,
            a[2] + (b[2] - a[2]) * u,
            1.0,
        ]
    }

    /// Paint a soft mesh's per-vertex `ATTRIBUTE_COLOR` from its per-vertex pressure field, so the
    /// deformed surface shows WHERE the contact concentrates (the heatmap). Mesh vertices are
    /// indexed by `VertexId`, so the field drops on one-to-one.
    fn apply_vertex_pressures(mesh: &mut Mesh, pressures: &[f32], scale: f32) {
        // A NaN vertex is an off-nominal degenerate contact (zero tributary area) — flag it MAGENTA
        // rather than letting NaN poison the colour, so it never reads as a safe coral/peach.
        let colors: Vec<[f32; 4]> = pressures
            .iter()
            .map(|&p| {
                if p.is_finite() {
                    heat(p / scale)
                } else {
                    [1.0, 0.0, 1.0, 1.0]
                }
            })
            .collect();
        mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    }

    /// Physics (x, y, z) +Z-up → Bevy (x, z, y) +Y-up, scaled to render units.
    fn to_bevy(p: [f64; 3]) -> Vec3 {
        Vec3::new(p[0] as f32, p[2] as f32, p[1] as f32) * RENDER_SCALE
    }

    /// Bevy-world x-shift that lays panel `i` of `n` out left→right, centred on the origin.
    ///
    /// Each soft block spans physics `[0, EDGE]`, so its centre sits half a block-width off its
    /// origin; subtracting that half-width recentres the whole row on the camera target (x = 0)
    /// rather than biasing it `+x` by one block half-extent.
    fn panel_shift(i: usize, n: usize) -> Vec3 {
        let centred = i as f32 - (n as f32 - 1.0) / 2.0;
        let block_half = 0.5 * EDGE as f32 * RENDER_SCALE;
        Vec3::new(
            centred * PANEL_GAP as f32 * RENDER_SCALE - block_half,
            0.0,
            0.0,
        )
    }

    fn setup(
        mut commands: Commands,
        mut meshes: ResMut<Assets<Mesh>>,
        mut materials: ResMut<Assets<StandardMaterial>>,
        mut scene: ResMut<Scene>,
        up: Res<UpAxis>,
    ) {
        let scale = scene.scale;
        let Some(panels) = scene.panels.take() else {
            return;
        };
        let n = panels.len();

        // WHITE base so the per-vertex heatmap colours (coral at rest → hot where squeezed) define
        // the buffer's appearance outright (PBR multiplies base × vertex colour).
        let soft_material = materials.add(StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 0.5,
            metallic: 0.05,
            ..default()
        });
        // Semi-transparent striker — the opaque ball/slab sits right over the contact patch, hiding
        // the heatmap; blending lets the squeezed (red/yellow) surface show through.
        let limb_material = materials.add(StandardMaterial {
            base_color: Color::srgba(0.62, 0.64, 0.70, 0.30),
            perceptual_roughness: 0.6,
            alpha_mode: AlphaMode::Blend,
            ..default()
        });

        let mut replay_panels = Vec::with_capacity(n);
        for (i, p) in panels.into_iter().enumerate() {
            let shift = panel_shift(i, n);

            // Soft buffer — its mesh positions AND per-vertex heatmap colours are rewritten per
            // frame by `replay`. The panel shift is the entity translation; RENDER_SCALE applies to
            // the physics-metre local mesh. Seed the rest-frame (all-cool) colours up front so the
            // mesh carries ATTRIBUTE_COLOR from the first render.
            let mut mesh = build_soft_mesh(&p.rest_positions, &p.boundary_faces, *up);
            apply_vertex_pressures(&mut mesh, &p.pressure_fields[0], scale);
            let soft_mesh = meshes.add(mesh);
            commands.spawn((
                Mesh3d(soft_mesh),
                MeshMaterial3d(soft_material.clone()),
                Transform {
                    translation: shift,
                    scale: Vec3::splat(RENDER_SCALE),
                    ..default()
                },
                SoftBody(i),
            ));

            // Striker — a concentrated SPHERE for the sphere panel (so the localized contact reads
            // at a glance), a broad gray SLAB for the slab panels. Moved each frame to the captured
            // limb pose + panel shift. Built in physics metres; the entity scale applies RENDER_SCALE.
            let limb_mesh = if p.is_sphere {
                meshes.add(Sphere::new(super::SPHERE_R as f32))
            } else {
                meshes.add(Cuboid::new(
                    (2.0 * LIMB_HALF[0]) as f32,
                    (2.0 * LIMB_HALF[2]) as f32,
                    (2.0 * LIMB_HALF[1]) as f32,
                ))
            };
            commands.spawn((
                Mesh3d(limb_mesh),
                MeshMaterial3d(limb_material.clone()),
                Transform {
                    translation: to_bevy(p.striker_poses[0]) + shift,
                    scale: Vec3::splat(RENDER_SCALE),
                    ..default()
                },
                Limb(i),
            ));

            replay_panels.push(PanelReplay {
                title: p.title,
                sub: p.sub,
                soft_frames: p.soft_frames,
                pressure_fields: p.pressure_fields,
                striker_poses: p.striker_poses,
                forces: p.forces,
                pressures: p.pressures,
            });
        }

        // Camera framed on the whole row. The panels run along Bevy +X, so the camera must sit on
        // the +Z side (azimuth ≈ π/2: `eye = target + dist·(cos az cos el, sin el, sin az cos el)`)
        // to view the row FRONTALLY and centred — azimuth ≈ 0 would look down the row edge-on.
        let span = (n as f32) * PANEL_GAP as f32 * RENDER_SCALE;
        commands.spawn((
            Camera3d::default(),
            Transform::default(),
            OrbitCamera::new()
                .with_target(Vec3::new(0.0, 0.55 * RENDER_SCALE * EDGE as f32, 0.0))
                .with_distance(1.25 * span.max(2.0 * RENDER_SCALE * EDGE as f32))
                .with_angles(std::f32::consts::FRAC_PI_2, 0.32),
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

        // Live HUD (top-left) — per-panel force + peak pressure, rewritten each frame by `replay`.
        commands.spawn((
            Text::new("..."),
            TextFont {
                font_size: 15.0,
                ..default()
            },
            TextColor(Color::srgb(0.95, 0.95, 0.7)),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(12.0),
                left: Val::Px(12.0),
                ..default()
            },
            Hud,
        ));
        commands.spawn((
            Text::new(
                "left vs middle: PRESSURE story (sphere concentrates)    |    middle vs right: \
                 RECOVERABILITY story (soft absorbs)    |    orbit: drag / scroll / right-drag",
            ),
            TextFont {
                font_size: 13.0,
                ..default()
            },
            TextColor(Color::srgb(0.8, 0.85, 0.95)),
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(12.0),
                left: Val::Px(12.0),
                ..default()
            },
        ));

        commands.insert_resource(Replay {
            panels: replay_panels,
            scale,
            dt: REPLAY_DT,
            epoch: None,
            last_idx: None,
        });
    }

    /// One shared looping cursor drives every panel's soft mesh + limb from the same frame index,
    /// and rewrites the HUD with each panel's live measured force / peak pressure.
    fn replay(
        time: Res<Time<Real>>,
        mut replay: ResMut<Replay>,
        up: Res<UpAxis>,
        mut meshes: ResMut<Assets<Mesh>>,
        q_soft: Query<(&Mesh3d, &SoftBody)>,
        mut q_limb: Query<(&mut Transform, &Limb)>,
        mut q_hud: Query<&mut Text, With<Hud>>,
    ) {
        let now = time.elapsed_secs_f64();
        let epoch = *replay.epoch.get_or_insert(now);
        let n_frames = replay.panels[0].soft_frames.len();
        let idx = (((now - epoch) / replay.dt) as usize) % n_frames;
        // The frame index only advances every `dt` (slow-mo), but this system runs every render
        // tick — skip the full mesh + colour reupload of all panels when the frame hasn't changed.
        if replay.last_idx == Some(idx) {
            return;
        }
        replay.last_idx = Some(idx);
        let n = replay.panels.len();
        let scale = replay.scale;

        for (mesh3d, body) in &q_soft {
            if let Some(mesh) = meshes.get_mut(&mesh3d.0) {
                let panel = &replay.panels[body.0];
                apply_soft_positions(mesh, &panel.soft_frames[idx], *up);
                // Repaint the heatmap — apply_soft_positions rewrites positions/normals but leaves
                // ATTRIBUTE_COLOR, so the per-vertex pressure tint must be reapplied each frame.
                apply_vertex_pressures(mesh, &panel.pressure_fields[idx], scale);
            }
        }
        for (mut tf, limb) in &mut q_limb {
            tf.translation =
                to_bevy(replay.panels[limb.0].striker_poses[idx]) + panel_shift(limb.0, n);
        }
        if let Ok(mut text) = q_hud.single_mut() {
            let mut s = String::new();
            for p in &replay.panels {
                let f = p.forces[idx];
                let press = p.pressures[idx];
                let flag = if f >= FRACTURE_CLIFF_N {
                    " [FRACTURE]"
                } else if f >= ISO_RECOVERABLE_N {
                    " [painful]"
                } else {
                    ""
                };
                writeln!(
                    s,
                    "{:<20} {:6.0} N   {:6.0} kPa{}",
                    format!("{} ({})", p.title, p.sub),
                    f,
                    press / 1e3,
                    flag,
                )
                .expect("writeln! to a String is infallible");
            }
            *text = Text::new(s);
        }
    }
}
