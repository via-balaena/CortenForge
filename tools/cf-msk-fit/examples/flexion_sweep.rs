//! The "moveable model" demo: drive the placed knee through flexion and write a
//! scrubbable, colored cf-viewer PLY sequence of the articulating skeleton.
//!
//!   cargo run -p cf-msk-fit --example flexion_sweep [out_dir]
//!   cargo run -p cf-viewer -- <out_dir>          # play/scrub: Space, ← →
//!
//! Each frame is the femur + tibia + knee + four muscle/tendon paths at one
//! flexion angle. The skin is omitted on purpose — the scan is an EXTENDED leg,
//! so a flexed shank would leave the straight skin (envelope-containment through
//! the range needs a pose-matched scan; the in-leg view is the S3 static frame
//! at extension). What this shows: the tibia rotating with the COUPLED knee, the
//! patella point tracking, and the muscle paths re-routing through the motion.

use cf_anthro::detect_landmarks;
use cf_anthro::markers::{cube, tube};
use cf_anthro::synthetic::LegSpec;
use cf_msk_fit::{Fitter, Placement};
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3, VertexColor};

/// Accumulates meshes into one attributed mesh (one PLY frame), tagging each
/// vertex with a `part` scalar (0=femur, 1=tibia, 2=knee, 3=muscle) — the
/// cf-viewer sequence colors via scalar→colormap, auto-selecting this field, so
/// the parts read distinctly (RGB vertex colors are kept too for other paths).
struct Scene {
    geom: IndexedMesh,
    colors: Vec<VertexColor>,
    parts: Vec<f32>,
}

impl Scene {
    fn new() -> Self {
        Self {
            geom: IndexedMesh::new(),
            colors: Vec::new(),
            parts: Vec::new(),
        }
    }
    fn add(&mut self, m: &IndexedMesh, c: VertexColor, part: f32) {
        let base = self.geom.vertices.len() as u32;
        self.geom.vertices.extend(m.vertices.iter().copied());
        self.geom.faces.extend(
            m.faces
                .iter()
                .map(|f| [f[0] + base, f[1] + base, f[2] + base]),
        );
        self.colors.extend(std::iter::repeat_n(c, m.vertices.len()));
        self.parts
            .extend(std::iter::repeat_n(part, m.vertices.len()));
    }
    fn into_attributed(self) -> AttributedMesh {
        let mut a = AttributedMesh::new(self.geom);
        a.colors = Some(self.colors);
        a.insert_extra("part", self.parts)
            .expect("part scalar length");
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

fn frame_scene(p: &Placement) -> Scene {
    let bone = VertexColor::new(232, 226, 208);
    let knee = VertexColor::new(245, 215, 70);
    let muscle = VertexColor::new(222, 78, 92);
    let mut s = Scene::new();
    s.add(
        &tube(p.femur.proximal, p.femur.distal, 0.010, 16),
        bone,
        0.0,
    );
    s.add(
        &tube(p.tibia.proximal, p.tibia.distal, 0.009, 16),
        bone,
        1.0,
    );
    s.add(&cube(p.knee, 0.012), knee, 2.0);
    for m in &p.muscles {
        s.add(&polyline_tube(&m.polyline, 0.004), muscle, 3.0);
    }
    s
}

fn path_len(p: &Placement, name: &str) -> f64 {
    p.muscles
        .iter()
        .find(|m| m.name == name)
        .map(|m| m.polyline.windows(2).map(|w| (w[1] - w[0]).norm()).sum())
        .unwrap_or(0.0)
}

fn main() {
    let out = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/tmp/flex_sweep".to_string());
    std::fs::create_dir_all(&out).unwrap();

    let (leg, _gt) = LegSpec::default_leg().build(220, 96);
    let lm = detect_landmarks(&leg).expect("detect landmarks");
    let osim = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    let sub = cf_osim::osim::parse_knee_subgraph(&std::fs::read_to_string(&osim).unwrap());
    let fit = Fitter::new(&sub, &lm);

    let n = 21; // 0 → 100° flexion, 5° steps
    println!("Flexion sweep → {out}/ ({n} frames)");
    println!(
        "  {:>5}  {:>9} {:>9} {:>9} {:>9}  {:>9}",
        "deg", "rect_fem", "vas_int", "bifemlh", "semimem", "knee-roll"
    );
    for i in 0..n {
        let deg = -(5.0 * i as f64);
        let p = fit.pose(deg.to_radians());
        save_ply_attributed(
            &frame_scene(&p).into_attributed(),
            format!("{out}/skel_step_{i:02}.ply"),
            true,
        )
        .unwrap();
        // knee rollback = how far the coupled joint center moved from extension.
        let roll = (p.tibia.proximal - p.femur.distal).norm() * 1000.0;
        if i % 4 == 0 {
            println!(
                "  {:>5.0}  {:>8.0}mm {:>8.0}mm {:>8.0}mm {:>8.0}mm  {:>7.1}mm",
                -deg,
                path_len(&p, "rect_fem_r") * 1000.0,
                path_len(&p, "vas_int_r") * 1000.0,
                path_len(&p, "bifemlh_r") * 1000.0,
                path_len(&p, "semimem_r") * 1000.0,
                roll,
            );
        }
    }
    println!("\nPlay:  cargo run -p cf-viewer -- {out}    (Space = play, ← → = step)");
}
