//! The scan → bones/tendons payoff: place the validated OpenSim knee inside a
//! detected leg scan and write a cf-viewer assembly.
//!
//!   cargo run -p cf-msk-fit --example bones_in_scan [out_dir]
//!   cargo run -p cf-viewer -- <out_dir> --assembly
//!
//! Pieces: the leg (skin), the femur + tibia bones meeting at the knee, a knee
//! marker, and the four muscle/tendon paths threaded through the limb.

use cf_anthro::detect_landmarks;
use cf_anthro::markers::{cube, tube};
use cf_anthro::synthetic::LegSpec;
use cf_msk_fit::{Bone, place_knee};
use mesh_io::save_stl;
use mesh_types::{IndexedMesh, Point3};

fn merge(into: &mut IndexedMesh, other: &IndexedMesh) {
    let base = into.vertices.len() as u32;
    into.vertices.extend(other.vertices.iter().copied());
    into.faces.extend(
        other
            .faces
            .iter()
            .map(|f| [f[0] + base, f[1] + base, f[2] + base]),
    );
}

/// A muscle path as a connected tube (one mesh per muscle → one assembly color).
fn polyline_tube(pts: &[Point3<f64>], r: f64) -> IndexedMesh {
    let mut m = IndexedMesh::new();
    for w in pts.windows(2) {
        merge(&mut m, &tube(w[0], w[1], r, 10));
    }
    m
}

fn bone_mesh(b: &Bone, r: f64) -> IndexedMesh {
    tube(b.proximal, b.distal, r, 16)
}

fn main() {
    let out = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/tmp/bones_in_scan".to_string());
    std::fs::create_dir_all(&out).unwrap();

    // Scan → landmarks.
    let (leg, _gt) = LegSpec::default_leg().build(220, 96);
    let lm = detect_landmarks(&leg).expect("detect landmarks on synthetic leg");

    // Model → placed onto the scan.
    let osim = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    let sub = cf_osim::parse_leg_chain(&std::fs::read_to_string(&osim).unwrap());
    let placed = place_knee(&sub, &lm);

    let write = |name: &str, m: &IndexedMesh| save_stl(m, format!("{out}/{name}"), true).unwrap();
    write("0_leg.stl", &leg);
    write("1_femur.stl", &bone_mesh(&placed.femur, 0.010));
    write("2_tibia.stl", &bone_mesh(&placed.tibia, 0.009));
    write("3_knee.stl", &cube(placed.knee, 0.012));
    for (i, m) in placed.muscles.iter().enumerate() {
        write(
            &format!("{}_{}.stl", i + 4, m.name),
            &polyline_tube(&m.polyline, 0.004),
        );
    }

    println!("Bones-in-scan assembly written to {out}/");
    println!(
        "  knee at z = {:.0} mm  |  model→scan scale = {:.3}",
        lm.knee_z * 1000.0,
        placed.scale
    );
    println!(
        "  femur {:.0} mm, tibia {:.0} mm",
        lm.thigh_length_m * 1000.0,
        lm.shank_length_m * 1000.0
    );
    for m in &placed.muscles {
        let len: f64 = m.polyline.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
        println!(
            "  {:<11} {} pts, path {:.0} mm",
            m.name,
            m.polyline.len(),
            len * 1000.0
        );
    }
    println!("\nView:  cargo run -p cf-viewer -- {out} --assembly");
}
