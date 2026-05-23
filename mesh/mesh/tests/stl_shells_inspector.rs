//! Throwaway diagnostic: load a STL via `INSPECT_STL` env var, weld
//! at 1 µm, find connected components, print per-component face
//! count and AABB to stderr. Used during recon-2 of the cf-cast
//! registration-pin disconnection bookmark. The implementation
//! session promotes this to `design/cf-cast/tests/iter_connectivity_inspector.rs`
//! and deletes this mesh-tier copy (per recon-2 §R3).
//!
//! Run as:
//!   `INSPECT_STL=<path> cargo test --release -p mesh \`
//!     `--test stl_shells_inspector -- --nocapture`
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::explicit_iter_loop)]

use std::path::PathBuf;

use mesh::types::IndexedMesh;
use mesh_io::load_stl;
use mesh_repair::{components::find_connected_components, weld_vertices};

#[test]
fn inspect_external_stl() {
    let Some(raw) = std::env::var_os("INSPECT_STL") else {
        eprintln!("INSPECT_STL unset; skipping inspector");
        return;
    };
    let path = PathBuf::from(&raw);
    let mut mesh = load_stl(&path).expect("load_stl failed");
    let welded = weld_vertices(&mut mesh, 1e-6);
    let analysis = find_connected_components(&mesh);
    eprintln!(
        "INSPECT_STL={} verts={} tris={} welded={} components={}",
        path.display(),
        mesh.vertices.len(),
        mesh.faces.len(),
        welded,
        analysis.component_count,
    );
    for (i, comp) in analysis.components.iter().enumerate() {
        let bb = component_aabb(&mesh, comp);
        let center = bb.center();
        let extent = bb.extent();
        eprintln!(
            "  comp {i:>2}: {:>5} faces  center=[{:>+8.3},{:>+8.3},{:>+8.3}] mm  extent=[{:>6.3},{:>6.3},{:>6.3}] mm",
            comp.len(),
            center[0] * 1000.0,
            center[1] * 1000.0,
            center[2] * 1000.0,
            extent[0] * 1000.0,
            extent[1] * 1000.0,
            extent[2] * 1000.0,
        );
    }
}

#[derive(Debug, Clone, Copy)]
struct Aabb {
    min: [f64; 3],
    max: [f64; 3],
}

impl Aabb {
    fn center(&self) -> [f64; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }
    fn extent(&self) -> [f64; 3] {
        [
            self.max[0] - self.min[0],
            self.max[1] - self.min[1],
            self.max[2] - self.min[2],
        ]
    }
}

fn component_aabb(mesh: &IndexedMesh, faces: &[u32]) -> Aabb {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for &fi in faces {
        let face = &mesh.faces[fi as usize];
        for &vi in face.iter() {
            let v = &mesh.vertices[vi as usize];
            let xs = [v.x, v.y, v.z];
            for k in 0..3 {
                if xs[k] < min[k] {
                    min[k] = xs[k];
                }
                if xs[k] > max[k] {
                    max[k] = xs[k];
                }
            }
        }
    }
    Aabb { min, max }
}
