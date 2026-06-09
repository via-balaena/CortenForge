//! The muscle-DRIVEN demo: drive the leg twin's muscles at known activations and
//! write a scrubbable cf-viewer PLY sequence with each muscle COLORED by its live
//! Millard force — the validated dynamics (G2) made visible.
//!
//!   cargo run -p cf-msk-fit --example muscle_drive [out_dir]
//!   cargo run -p cf-viewer -- <out_dir>          # play/scrub: Space, ← →
//!
//! Two models, same template, at each knee angle: the kinematic [`Fitter`] gives the
//! 3D geometry (femur, tibia, knee, the four muscle paths), and the emitted engine
//! twin — Millard `<general>` actuators on its spatial tendons — is driven at full
//! activation and `forward()`-ed to read each muscle's force. Each muscle tube is
//! tinted pale→bright-red by its force, and a `force_N` vertex scalar lets cf-viewer
//! colormap it. The forces vary purely with pose here (the force-LENGTH effect of the
//! OpenSim-validated Millard model), since activation is held at 1.0.
//!
//! Free forward dynamics (watch it fall under muscle force) is deferred: the emitted
//! coupled knee is driven kinematically, not via equality constraints, so the slides
//! can't yet move freely — a documented G2 follow-up.

use cf_anthro::detect_landmarks;
use cf_anthro::markers::{cube, tube};
use cf_anthro::synthetic::LegSpec;
use cf_mjcf_emit::emit;
use cf_msk_fit::{Fitter, Placement};
use cf_osim::{joint_id, parse_leg_chain};
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3, VertexColor};
use sim_mjcf::load_model;
use std::collections::HashMap;

const MUSCLES: [&str; 4] = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"];

struct Scene {
    geom: IndexedMesh,
    colors: Vec<VertexColor>,
    force: Vec<f32>,
}

impl Scene {
    fn new() -> Self {
        Self {
            geom: IndexedMesh::new(),
            colors: Vec::new(),
            force: Vec::new(),
        }
    }
    fn add(&mut self, m: &IndexedMesh, c: VertexColor, force_n: f32) {
        let base = self.geom.vertices.len() as u32;
        self.geom.vertices.extend(m.vertices.iter().copied());
        self.geom.faces.extend(
            m.faces
                .iter()
                .map(|f| [f[0] + base, f[1] + base, f[2] + base]),
        );
        self.colors.extend(std::iter::repeat_n(c, m.vertices.len()));
        self.force
            .extend(std::iter::repeat_n(force_n, m.vertices.len()));
    }
    fn into_attributed(self) -> AttributedMesh {
        let mut a = AttributedMesh::new(self.geom);
        a.colors = Some(self.colors);
        a.insert_extra("force_N", self.force)
            .expect("force scalar length");
        a
    }
}

fn polyline_tube(pts: &[Point3<f64>], r: f64) -> IndexedMesh {
    let mut m = IndexedMesh::new();
    for w in pts.windows(2) {
        let seg = tube(w[0], w[1], r, 10);
        let base = m.vertices.len() as u32;
        m.vertices.extend(seg.vertices.iter().copied());
        m.faces.extend(
            seg.faces
                .iter()
                .map(|f| [f[0] + base, f[1] + base, f[2] + base]),
        );
    }
    m
}

/// Pale-yellow (slack) → bright-red (max force) tint by normalized force.
fn force_color(force_n: f64, max_n: f64) -> VertexColor {
    let t = (force_n / max_n).clamp(0.0, 1.0);
    let lerp = |a: f64, b: f64| (a + (b - a) * t).round() as u8;
    VertexColor::new(lerp(238.0, 200.0), lerp(232.0, 30.0), lerp(150.0, 40.0))
}

fn frame_scene(p: &Placement, forces: &HashMap<String, f64>, max_n: f64) -> Scene {
    let bone = VertexColor::new(232, 226, 208);
    let knee = VertexColor::new(245, 215, 70);
    let mut s = Scene::new();
    s.add(
        &tube(p.femur.proximal, p.femur.distal, 0.010, 16),
        bone,
        0.0,
    );
    s.add(
        &tube(p.tibia.proximal, p.tibia.distal, 0.009, 16),
        bone,
        0.0,
    );
    s.add(&cube(p.knee, 0.012), knee, 0.0);
    for m in &p.muscles {
        let f = forces.get(&m.name).copied().unwrap_or(0.0);
        // tube radius grows a little with force, too — a thicker, redder pull.
        let r = 0.0035 + 0.0030 * (f / max_n).clamp(0.0, 1.0);
        s.add(
            &polyline_tube(&m.polyline, r),
            force_color(f, max_n),
            f as f32,
        );
    }
    s
}

fn main() {
    let out = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/tmp/muscle_drive".to_string());
    std::fs::create_dir_all(&out).unwrap();

    let osim = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    let sub = parse_leg_chain(&std::fs::read_to_string(&osim).unwrap());

    // Geometry: the kinematic Fitter placed on a synthetic leg.
    let (leg, _gt) = LegSpec::default_leg().build(220, 96);
    let lm = detect_landmarks(&leg).expect("detect landmarks");
    let fit = Fitter::new(&sub, &lm);

    // Forces: the emitted muscle-driven engine twin (Millard actuators).
    let emitted = emit(&sub);
    let model = load_model(&emitted.mjcf).expect("emitted muscle-driven MJCF must load");
    let adr: HashMap<String, usize> = emitted
        .driven
        .iter()
        .map(|j| {
            (
                j.joint.clone(),
                model.jnt_qpos_adr[joint_id(&model, &j.joint)],
            )
        })
        .collect();
    let aid: HashMap<&str, usize> = MUSCLES
        .iter()
        .map(|&m| {
            (
                m,
                model
                    .actuator_name
                    .iter()
                    .position(|n| n.as_deref() == Some(m))
                    .unwrap(),
            )
        })
        .collect();

    // Drive every muscle at full activation; read the force at each knee angle.
    let drive_forces = |th: f64| -> HashMap<String, f64> {
        let mut data = model.make_data();
        let coords = HashMap::from([("knee_angle_r".to_string(), th)]);
        for (joint, val) in emitted.qpos_targets(&coords) {
            data.qpos[adr[&joint]] = val;
        }
        for &m in &MUSCLES {
            data.act[model.actuator_act_adr[aid[m]]] = 1.0;
        }
        data.forward(&model).expect("forward failed");
        MUSCLES
            .iter()
            .map(|&m| (m.to_string(), data.actuator_force[aid[m]].abs()))
            .collect()
    };

    let n = 21; // 0 → 100° flexion, 5° steps
    let max_n = 1500.0; // ~F0 of the strongest target muscle, for the color scale
    println!("Muscle-driven sweep → {out}/ ({n} frames), activation = 1.0");
    println!(
        "  {:>5}  {:>9} {:>9} {:>9} {:>9}",
        "deg", "rect_fem", "vas_int", "bifemlh", "semimem"
    );
    for i in 0..n {
        let deg = -(5.0 * i as f64);
        let th = deg.to_radians();
        let p = fit.pose(th);
        let forces = drive_forces(th);
        save_ply_attributed(
            &frame_scene(&p, &forces, max_n).into_attributed(),
            format!("{out}/muscle_step_{i:02}.ply"),
            true,
        )
        .unwrap();
        if i % 4 == 0 {
            println!(
                "  {:>5.0}  {:>7.0} N {:>7.0} N {:>7.0} N {:>7.0} N",
                -deg,
                forces["rect_fem_r"],
                forces["vas_int_r"],
                forces["bifemlh_r"],
                forces["semimem_r"],
            );
        }
    }
    println!("\nPlay:  cargo run -p cf-viewer -- {out}    (Space = play, ← → = step)");
}
