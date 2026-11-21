// Ported from Qhull, libqhull_r (user_r.c's qh_errexit, the message numbers).
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! Why a hull was not made.

use std::fmt;

/// Why [`crate::mesh_graph`] made no graph. MuJoCo 3.5.0 refuses the mesh
/// with "qhull error" for every [`Error::Qhull`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// Qhull stopped (`qh_errexit`): its message number (`QH6154` is 6154)
    /// and what it says.
    Qhull {
        /// Qhull's message number.
        code: u16,
        /// Qhull's words, less the numbers it prints into them.
        message: &'static str,
    },
    /// MuJoCo's own check of qhull's result in `MakeGraph`
    /// (`user_mesh.cc:1914-2027`) failed. "Qhull did not return triangle",
    /// "Wrong size in convex hull graph" and "Vertex id not found in convex
    /// hull" are MuJoCo's `mju_error`; "Could not construct convex hull
    /// graph" is its `mju_warning` (`:2007`), after which MuJoCo goes on
    /// with the graph freed, where this port stops.
    Graph {
        /// MuJoCo's words.
        message: &'static str,
    },
    /// The points reach a routine of qhull's that this port does not hold:
    /// none of the inputs it was measured on reaches one.
    Unported {
        /// The qhull routine, by its name in qhull's source.
        routine: &'static str,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Qhull { code, message } => write!(f, "qhull error QH{code}: {message}"),
            Self::Graph { message } => {
                write!(f, "MuJoCo's check of the convex hull graph: {message}")
            }
            Self::Unported { routine } => {
                write!(
                    f,
                    "the points reach qhull's {routine}, which this port does not hold"
                )
            }
        }
    }
}

impl Error {
    /// qhull's `qh_errexit` with its message number and words.
    pub(crate) const fn qhull(code: u16, message: &'static str) -> Self {
        Self::Qhull { code, message }
    }
}

impl std::error::Error for Error {}
