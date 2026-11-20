// Ported from Qhull (www.qhull.org) at github.com/qhull/qhull 62ccc56a, the
// qhull MuJoCo 3.5.0 builds, and from MuJoCo 3.5.0's mjCMesh::MakeGraph.
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// MuJoCo, Copyright 2021 DeepMind Technologies Limited, Apache-2.0.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! Qhull's 3-d convex hull as MuJoCo 3.5.0 runs it, giving MuJoCo's mesh
//! graph.
//!
//! [`mesh_graph`] is MuJoCo's `mjCMesh::MakeGraph` (`user_mesh.cc:1861-2043`):
//! qhull run as `qhull Qt` (and `TA` under a mesh's `maxhullvert`) over a
//! mesh's vertices, then `qh_triangulate` and `qh_vertexneighbors`, and the
//! graph read from qhull's lists. Every decision qhull makes from a floating
//! point comparison is made here from the same operations in the same
//! order, so the graph is MuJoCo's int for int: the same vertices in the same
//! order, the same neighbours in the same order, the same triangles.
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
//! Where the options allow a routine that no input the port was measured on
//! reaches, it returns [`Error::Unported`] naming the routine instead of
//! going on: a ridge of more than two new facets (qhull's dupridges and the
//! pinched vertices they lead to), a cycle of more than one new facet
//! coplanar with one horizon facet (`qh_mergecycle`), a degenerate merge or
//! an extra vertex while vertices are reduced, a mirrored pair of triangles,
//! a point partitioned below every new facet (`qh_findbestlower`), and a
//! narrow hull's furthest point recomputed (`qh_furthestout`).
//!
//! Two of qhull's choices depend on its platform rather than its
//! arithmetic: the order `qsort` leaves equal merges in, and the order a
//! ridge hash keyed on vertex addresses probes in. Here they are a stable
//! sort and a hash on vertex ids.
//!
//! # Licence
//!
//! Qhull's licence (`COPYING.txt`, shipped with this crate) for the qhull
//! port; Apache-2.0 for the part that reads MuJoCo's graph from it. The
//! original qhull is at www.qhull.org.

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
mod set;
mod state;
mod triangulate;

pub use error::Error;
pub use graph::MeshGraph;

use state::Qhull;

/// MuJoCo 3.5.0's convex hull graph of `points` (`mjCMesh::MakeGraph`).
///
/// qhull's hull of them, `None` for fewer than 4 points (MuJoCo makes no
/// graph). `maxhullvert` is the mesh's (`-1` for none): with it qhull stops
/// once the hull has that many vertices (MuJoCo's reader refuses one below 4,
/// where qhull would ignore the option).
///
/// # Errors
///
/// [`Error::Qhull`] where qhull stops (flat or degenerate input among them),
/// which MuJoCo reports as "qhull error"; [`Error::Unported`] where the points
/// reach a routine this port does not hold.
pub fn mesh_graph(points: &[[f64; 3]], maxhullvert: i32) -> Result<Option<MeshGraph>, Error> {
    if points.len() < 4 {
        return Ok(None);
    }
    // `qhull Qt TA<maxhullvert-4>`: qhull keeps `TA n` as `n + 1`
    // (`global_r.c:1416-1426`), and ignores a count that is not a digit.
    let stop_add = if maxhullvert >= 4 { maxhullvert - 3 } else { 0 };
    let mut qh = Qhull::new(points, stop_add);
    qh.qhull()?;
    qh.triangulate()?;
    qh.vertexneighbors();
    qh.graph().map(Some)
}
