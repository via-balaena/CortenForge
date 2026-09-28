//! How the wall is held (plan §9 decisions 10–11, §16w): vertices of its outer skin, held whole.
//!
//! The outer skin is read from the wall's boundary faces only, so no vertex inside the wall is taken; the old path's
//! pin took every vertex within half a cell of the outer envelope, 646 of them inside the wall (fit plan, Phase 1).
//! Two holds are built from it: a mount, which holds the skin beyond a plane, and a rigid shell bonded to the whole
//! skin. A case the wall can slide along is not one of them: a node held to a fixed direction moves along its tangent
//! plane and so off a curved case, and on the cased tube that let the tube turn and its case open (plan §16n).

use nalgebra::{Point3, Vector3};

use crate::Yeoh;
use crate::mesh::{Mesh, VertexId};
use crate::sdf_bridge::{Sdf, SdfMeshedTetMesh};

/// A plane: a point on it and its unit normal. A point is beyond it on its normal's side.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    /// A point on it.
    pub point: Point3<f64>,
    /// Its unit normal.
    pub normal: Vector3<f64>,
}

impl Plane {
    /// The plane through `point` normal to `normal`, which is normalised; `None` when either is not finite or the
    /// normal has no length.
    #[must_use]
    pub fn new(point: Point3<f64>, normal: Vector3<f64>) -> Option<Self> {
        let finite = point.iter().chain(normal.iter()).all(|c| c.is_finite());
        let length = normal.norm();
        (finite && length > 0.0).then(|| Self {
            point,
            normal: normal / length,
        })
    }

    /// How far `p` lies beyond it: negative on the other side.
    #[must_use]
    pub fn height(&self, p: Point3<f64>) -> f64 {
        (p - self.point).dot(&self.normal)
    }
}

/// The wall's outer skin: its boundary faces whose centroid lies within half a lattice cell of the outer surface's
/// level.
///
/// Where the outer surface ends at a rim, as it meets the mouth, the skin also takes vertices beyond the rim in the
/// mouth's plane: within a cell of it on the cup `tests/lowering_holds.rs` meshes. Which faces bring them in, faces
/// bevelling the rim or flat ones beside it, is not separated.
///
/// `outer` must be the outer surface's own field. A field that also closes the mouth, as the outer shell's floor does,
/// reads near zero on the faces beside the mouth, the canal's included, and the skin takes them
/// (`the_skin_needs_the_outer_surfaces_own_field`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Skin {
    /// The skin's vertices, ascending.
    vertices: Vec<VertexId>,
}

impl Skin {
    /// The skin of `mesh`, meshed on a lattice of spacing `cell`, whose outer surface is `outer`'s zero level.
    #[must_use]
    pub fn of(mesh: &SdfMeshedTetMesh<Yeoh>, outer: &dyn Sdf, cell: f64) -> Self {
        let positions = mesh.positions();
        let at = |v: VertexId| Point3::from(positions[v as usize]);
        let mut vertices: Vec<VertexId> = mesh
            .boundary_faces()
            .iter()
            .filter(|face| {
                let corners = face.map(at);
                let centroid =
                    Point3::from((corners[0].coords + corners[1].coords + corners[2].coords) / 3.0);
                outer.eval(centroid).abs() <= 0.5 * cell
            })
            .flatten()
            .copied()
            .collect();
        vertices.sort_unstable();
        vertices.dedup();
        Self { vertices }
    }

    /// Its vertices, ascending: a rigid shell bonded to the skin holds them all.
    #[must_use]
    pub fn vertices(&self) -> &[VertexId] {
        &self.vertices
    }

    /// Its vertices beyond `plane`, ascending: what a mount there holds.
    #[must_use]
    pub fn beyond(&self, mesh: &SdfMeshedTetMesh<Yeoh>, plane: &Plane) -> Vec<VertexId> {
        let positions = mesh.positions();
        self.vertices
            .iter()
            .copied()
            .filter(|&v| plane.height(Point3::from(positions[v as usize])) > 0.0)
            .collect()
    }
}
