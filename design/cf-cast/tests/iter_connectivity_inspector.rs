//! Workshop-fixture connectivity inspector — recon-2 §R3 #3
//! (registration-pin disconnection arc).
//!
//! Promoted from a throwaway `mesh/mesh/tests/stl_shells_inspector.rs`
//! used during recon-2 of the registration-pin disconnection
//! bookmark. Loads an external STL via `INSPECT_STL`, welds at
//! 1 µm via `mesh-repair::weld_vertices`, runs
//! `mesh-repair::components::find_connected_components`, and
//! reports per-component face count + AABB to stderr. Optionally
//! enforces the §R1 connectivity invariant when
//! `INSPECT_STL_R1=1` is set so workshop iter-N runs can
//! falsification-gate a regenerated STL set BEFORE physically
//! printing.
//!
//! # Manual run
//!
//! ```text
//! INSPECT_STL=~/scans/cast_iter1/mold_layer_0_piece_0.stl \
//!     cargo test --release -p cf-cast --test iter_connectivity_inspector \
//!     -- --ignored --nocapture
//! ```
//!
//! Add `INSPECT_STL_R1=1` to fail the test when the §R1 invariant
//! (≤ 2 extra components, ≤ 50 faces each, ≤ 1.5 mm max-extent
//! each) is violated.
//!
//! # Why kept around
//!
//! The recon-2 bookmark + implementation session both relied on
//! this inspector to triage the iter-2 STL set's pre-fix vs
//! post-fix component-count signal. Workshop iter-3 will need the
//! same surface to verify any future mating-feature migration
//! preserves §R1; keeping it as an `#[ignore]`-gated cf-cast
//! integration test (parallel to `iter1_gate.rs`) keeps the
//! diagnostic tier reachable from a single
//! `cargo test --release -p cf-cast -- --ignored` invocation.

// Integration test idiom: surface failures with explicit panic
// messages so the workshop user can triage from one stderr block.
#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used,
    clippy::explicit_iter_loop
)]

use std::path::PathBuf;

use mesh_io::load_stl;
use mesh_repair::{components::find_connected_components, weld_vertices};
use mesh_types::IndexedMesh;

/// §R1 thresholds — copy-of from recon-2 §R1 boxed decision
/// (per-cup-piece-mesh; iter-1 sliver is 24 faces / 1.012 mm
/// max-extent so these have margin).
const MAX_EXTRA_COMPONENTS: usize = 2;
const MAX_SLIVER_FACES: usize = 50;
const MAX_SLIVER_MAX_EXTENT_MM: f64 = 1.5;

#[test]
#[ignore = "inspector: requires INSPECT_STL=<path-to-cup-piece-stl>; run with --ignored"]
fn inspect_external_stl() {
    let Some(raw) = std::env::var_os("INSPECT_STL") else {
        panic!(
            "INSPECT_STL unset — see file-level docs for the manual-run command \
             (example: INSPECT_STL=~/scans/cast_iter1/mold_layer_0_piece_0.stl)"
        );
    };
    let path = PathBuf::from(&raw);
    let mut mesh = load_stl(&path).unwrap_or_else(|e| panic!("load_stl({}): {e}", path.display()));
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
    let extra = analysis.component_count.saturating_sub(1);
    let mut violations: Vec<String> = Vec::new();
    for (i, comp) in analysis.components.iter().enumerate() {
        let bb = component_aabb(&mesh, comp);
        let center = bb.center();
        let extent = bb.extent();
        // `mesh_io::load_stl` returns vertices in the STL file's
        // units (cf-cast writes STLs in millimeters via
        // `mesh_csg::METERS_TO_MM`), so center + extent are already
        // in mm — no scale-conversion needed (an earlier inspector
        // version multiplied by 1000.0 here under the wrong
        // assumption that load_stl returned meters; that masked
        // sub-mm slivers behind 1000×-inflated displayed values).
        eprintln!(
            "  comp {i:>2}: {:>5} faces  center=[{:>+8.3},{:>+8.3},{:>+8.3}] mm  extent=[{:>6.3},{:>6.3},{:>6.3}] mm",
            comp.len(),
            center[0],
            center[1],
            center[2],
            extent[0],
            extent[1],
            extent[2],
        );
        if i == 0 {
            continue;
        }
        let max_extent_mm = extent[0].max(extent[1]).max(extent[2]);
        if comp.len() > MAX_SLIVER_FACES {
            violations.push(format!(
                "comp {i}: {} faces > §R1 cap {MAX_SLIVER_FACES}",
                comp.len()
            ));
        }
        if max_extent_mm > MAX_SLIVER_MAX_EXTENT_MM {
            violations.push(format!(
                "comp {i}: max-extent {max_extent_mm:.3} mm > §R1 cap {MAX_SLIVER_MAX_EXTENT_MM} mm"
            ));
        }
    }
    if extra > MAX_EXTRA_COMPONENTS {
        violations.push(format!(
            "{extra} extra components > §R1 cap {MAX_EXTRA_COMPONENTS}"
        ));
    }

    // Default mode reports only; INSPECT_STL_R1=1 enforces the
    // invariant. Workshop iter-N runs set the env-var to gate
    // pre-print.
    let enforce = std::env::var_os("INSPECT_STL_R1").is_some();
    assert!(
        !enforce || violations.is_empty(),
        "§R1 invariant violated for {}:\n  - {}",
        path.display(),
        violations.join("\n  - ")
    );
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
