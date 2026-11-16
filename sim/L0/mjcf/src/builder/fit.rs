//! A sphere, capsule, cylinder, ellipsoid or box that names a mesh, fitted
//! to the mesh as MuJoCo 3.5.0 fits one (`mjCMesh::FitGeom`,
//! `user_mesh.cc:979-1057`, from `mjCGeom::Compile`, `user_objects.cc:3738-3739`).

use super::ModelBuilder;
use super::written_over;
use crate::types::{MjcfGeom, MjcfGeomType, Prefix};

/// A primitive geom's fit to the mesh it names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fit {
    /// The mesh, whose frame the geom takes into its own.
    pub mesh: usize,
    /// The geom's sizes: its own, each place its type fits replaced, then
    /// each times `fitscale`.
    pub size: [f64; 3],
    /// The fitted shape's centre in the mesh's frame: the mesh's box's
    /// midpoint under `fitaabb`, else the origin.
    pub center: [f64; 3],
}

impl Fit {
    /// `geom` with the fitted size, as the body's mass and the geom's fluid
    /// model read it.
    pub(crate) fn applied_to(&self, geom: &MjcfGeom) -> MjcfGeom {
        MjcfGeom {
            size: Some(Prefix::full(self.size)),
            ..geom.clone()
        }
    }
}

/// The types MuJoCo fits: every type but a mesh or SDF (`:3738`), less the
/// plane and height field validation refuses (`FitGeom`'s default arm).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Fitted {
    Sphere,
    Capsule,
    Cylinder,
    Ellipsoid,
    Box,
}

impl Fitted {
    fn of(geom_type: MjcfGeomType) -> Option<Self> {
        match geom_type {
            MjcfGeomType::Sphere => Some(Self::Sphere),
            MjcfGeomType::Capsule => Some(Self::Capsule),
            MjcfGeomType::Cylinder => Some(Self::Cylinder),
            MjcfGeomType::Ellipsoid => Some(Self::Ellipsoid),
            MjcfGeomType::Box => Some(Self::Box),
            MjcfGeomType::Plane | MjcfGeomType::Hfield | MjcfGeomType::Mesh | MjcfGeomType::Sdf => {
                None
            }
        }
    }
}

impl ModelBuilder {
    /// `geom`'s fit, for a sphere (an untyped geom is one), capsule,
    /// cylinder, ellipsoid or box naming a mesh; `None` for any other geom.
    ///
    /// From the mesh's equivalent inertia box ([`super::mesh::MeshProps::boxsz`],
    /// MuJoCo's `GetInertiaBoxPtr`): a sphere's radius the mean of its three
    /// half sizes, a capsule's or cylinder's radius the mean of the first
    /// two and its half length the third (a capsule's less half its radius,
    /// at least 0), an ellipsoid's or box's the three. Under `fitaabb` from
    /// the mesh's box in its own frame instead (MuJoCo's `aamm_`), about the
    /// box's midpoint: the smallest shape whose box holds it, a capsule's
    /// half length less its radius.
    pub(crate) fn fit(&self, geom: &MjcfGeom) -> Option<Fit> {
        let name = geom.mesh.as_deref().filter(|m| !m.is_empty())?;
        let fitted = Fitted::of(geom.geom_type.unwrap_or(MjcfGeomType::Sphere))?;
        let mesh = *self.mesh_name_to_id.get(name)?;
        let mut size = written_over(geom.size, [0.0; 3]);
        let mut center = [0.0; 3];
        if self.compiler.fitaabb {
            let (lo, hi) = self.mesh_data[mesh].aabb();
            // `midpoint` is MuJoCo's `(a + b) / 2` wherever that sum does not overflow.
            center = std::array::from_fn(|i| f64::midpoint(lo[i], hi[i]));
            let half: [f64; 3] = std::array::from_fn(|i| hi[i] - center[i]);
            match fitted {
                Fitted::Sphere => size[0] = std_max(std_max(half[0], half[1]), half[2]),
                Fitted::Capsule | Fitted::Cylinder => {
                    size[0] = std_max(half[0], half[1]);
                    size[1] = half[2];
                    if fitted == Fitted::Capsule {
                        size[1] -= size[0];
                    }
                }
                Fitted::Ellipsoid | Fitted::Box => size = half,
            }
        } else {
            let b = self.mesh_props[mesh].boxsz;
            match fitted {
                Fitted::Sphere => size[0] = (b[0] + b[1] + b[2]) / 3.0,
                Fitted::Capsule => {
                    size[0] = f64::midpoint(b[0], b[1]);
                    size[1] = std_max(0.0, b[2] - size[0] / 2.0);
                }
                Fitted::Cylinder => {
                    size[0] = f64::midpoint(b[0], b[1]);
                    size[1] = b[2];
                }
                Fitted::Ellipsoid | Fitted::Box => size = b,
            }
        }
        let scale = geom.fitscale.unwrap_or(1.0);
        Some(Fit {
            mesh,
            size: size.map(|s| s * scale),
            center,
        })
    }
}

/// `std::max(a, b)`: `b` where `a < b`, else `a`.
fn std_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}
