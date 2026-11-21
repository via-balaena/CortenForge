// Ported from Qhull (www.qhull.org) at github.com/qhull/qhull 62ccc56a, the
// qhull MuJoCo 3.5.0 builds, and from MuJoCo 3.5.0's mjCMesh::MakeGraph.
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// global_r.c: Copyright (c) 1993-2020 The Geometry Center.
// MuJoCo, Copyright 2021 DeepMind Technologies Limited, Apache-2.0.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! Qhull's 3-d convex hull as MuJoCo 3.5.0 runs it, giving MuJoCo's mesh
//! graph.
//!
//! [`mesh_graph`] is MuJoCo's `mjCMesh::MakeGraph` (`user_mesh.cc:1861-2042`):
//! qhull run as `qhull Qt` (and `TA` under a mesh's `maxhullvert`) over a
//! mesh's vertices, then `qh_triangulate` and `qh_vertexneighbors`, and the
//! graph read from qhull's lists: the floating point operations qhull's
//! decisions rest on made in qhull's order, its sets and lists walked in its
//! order. `tests/mesh_graph_golden.rs` asks it for MuJoCo's own graph, int
//! for int, of each point set in `tests/data/mesh_graph_golden.txt`.
//!
//! # What is ported
//!
//! The path `qhull Qt` takes in 3-d: merging on (`C-0`, qhull's default
//! below 5-d), `KEEPnearinside`, no joggle, no scaling or rotation, no
//! output. Delaunay, Voronoi, halfspaces, joggle, the other dimensions and
//! post-merging are unreachable with these options and are not here. qhull's
//! statistics and tracing are not here either, but for the merge count
//! (`Ztotmerge`), which decides how points are partitioned
//! (`qh_USEfindbestnew`).
//!
//! After each point, the new facets are merged where qhull merges them
//! (`qh_premerge`): into a coplanar horizon facet, a flipped facet into its
//! best neighbour, each non-convex ridge's facets into one; then the vertices
//! the merges left on two facets only are renamed (`qh_reducevertices`).
//! After the build, `qh_triangulate` splits each merged facet into triangles.
//!
//! Where the options allow a routine that no measured input reaches, it
//! returns [`Error::Unported`] naming the routine instead of going on.
//!
//! qhull sorts its merges, and the vertices `qh_find_newvertex` chooses
//! from, with its C library's `qsort`, and neither comparator returns
//! equal, so two elements one cannot order are left where that `qsort`
//! leaves them. Here the sort is macOS's `qsort`, ported (`qsort.rs`);
//! `tests/data/qsort_orders.txt` holds the orders macOS's own `qsort` and
//! `heapsort` left arrays in. What MuJoCo gives with another C library's
//! `qsort` has not been measured. qhull also probes a ridge hash keyed on
//! vertex addresses; here the hash is on vertex ids.
//!
//! # Licence
//!
//! Qhull's licence (`COPYING.txt`, shipped with this crate) for the qhull
//! port; Apache-2.0 for the part that reads MuJoCo's graph from it; the
//! 3-clause BSD licence, in its header, for `qsort.rs`, ported from Apple's
//! Libc. The original qhull is at www.qhull.org.

// Items are crate-visible in private modules: `pub(crate)` says so.
#![allow(clippy::redundant_pub_crate)]
// qhull counts in C ints: point, vertex and facet ids and counts fit both i32
// and u32, as MuJoCo holds them in ints; `3 as f64` and the like are exact.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
// qhull's facet, vertex and ridge flags: one bool per C bitfield.
#![allow(clippy::struct_excessive_bools)]

mod build;
mod error;
mod findbest;
mod geom;
mod graph;
mod merge;
mod qsort;
mod set;
mod state;
mod triangulate;

pub use error::Error;
pub use graph::MeshGraph;

use state::Qhull;

/// MuJoCo 3.5.0's convex hull graph of `points` (`mjCMesh::MakeGraph`).
///
/// qhull's hull of them, `None` for fewer than 4 points (MuJoCo makes no
/// graph). `points` are a mesh's vertices as MuJoCo holds them when it makes
/// the graph: before its `refpos`, `refquat` and `scale` are applied
/// (`mjCMesh::Process`, `user_mesh.cc:1567-1603`), and finite (MuJoCo
/// refuses a vertex that is not finite before). The graph of the same points
/// turned can differ.
///
/// `maxhullvert` is the mesh's (`-1` for none): with it qhull stops once the
/// hull has that many vertices. MuJoCo's reader refuses one from 0 to 3; for
/// those, qhull's parse of the `TA` it is given reads a pointer it never set
/// (`global_r.c:1417-1425`), and this port runs as for none.
///
/// # Errors
///
/// [`Error::Qhull`] where qhull stops (flat or degenerate input among them),
/// which MuJoCo reports as "qhull error"; [`Error::Graph`] where MuJoCo's own
/// check of qhull's result fails; [`Error::Unported`] where the points reach
/// a routine this port does not hold.
pub fn mesh_graph(points: &[[f64; 3]], maxhullvert: i32) -> Result<Option<MeshGraph>, Error> {
    if points.len() < 4 {
        return Ok(None);
    }
    // `qhull Qt TA<maxhullvert-4>`: qhull keeps `TA n` as `n + 1`
    // (`global_r.c:1416-1426`); below 4, as for none (above).
    let stop_add = if maxhullvert >= 4 { maxhullvert - 3 } else { 0 };
    let mut qh = Qhull::new(points, stop_add);
    qh.qhull()?;
    qh.triangulate()?;
    qh.vertexneighbors();
    qh.graph().map(Some)
}
