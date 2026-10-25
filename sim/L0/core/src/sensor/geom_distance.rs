//! The geom-distance sensors' distance between two geoms: MuJoCo's
//! `mj_geomDistance` (`engine_support.c:556-601`).

use crate::collision::geom_distance_ccd;
use crate::collision::hfield::collide_hfield_multi;
use crate::collision::narrow::{collide_geoms, convex_pair, primitive_contacts};
use crate::types::{Data, GeomType, Model};

/// `mj_geomDistance`: the smallest distance between `geom1` and `geom2`, at
/// most `distmax` (negative in penetration), and the segment
/// `[from, to]` from `geom1` to `geom2` it spans, zeros where nothing is
/// found below `distmax`.
///
/// MuJoCo takes the pair in type order, the lower `mjtGeom` first. A plane
/// or height field with a plane or height field has no collision function
/// and reads `distmax`. A pair its collision table gives `mjc_Convex` or
/// `mjc_BoxBox` goes to `mjc_ccd` ([`geom_distance_ccd`]), the geoms in the
/// order given. Every other pair goes to its collision function with
/// `distmax` as the margin: the smallest distance it lists, the segment
/// along that contact's normal, centred on its position. The normal is the
/// one the collision function wrote, as MuJoCo's sensor reads it, not the
/// one `mj_setContact` normalises: a pair of spheres, capsules and boxes, or
/// a sphere and a cylinder, through [`primitive_contacts`]; a plane's pair,
/// the plane's z axis, which every plane function writes. A signed distance
/// field takes sim-core's own functions.
// Paired geom identifiers (geom1/geom2, g1/g2) are intentionally similar.
#[allow(clippy::similar_names)]
pub fn geom_distance(
    model: &Model,
    data: &Data,
    geom1: usize,
    geom2: usize,
    distmax: f64,
) -> (f64, [f64; 6]) {
    let mut fromto = [0.0; 6];
    let flip = model.geom_type[geom1].mjtgeom() > model.geom_type[geom2].mjtgeom();
    let (g1, g2) = if flip { (geom2, geom1) } else { (geom1, geom2) };
    let (type1, type2) = (model.geom_type[g1], model.geom_type[g2]);
    if matches!(type2, GeomType::Plane | GeomType::Hfield) {
        return (distmax, fromto);
    }
    if convex_pair(type1, type2) || (type1 == GeomType::Box && type2 == GeomType::Box) {
        return geom_distance_ccd(model, data, geom1, geom2, distmax);
    }

    let (pos1, mat1) = (data.geom_xpos[g1], data.geom_xmat[g1]);
    let (pos2, mat2) = (data.geom_xpos[g2], data.geom_xmat[g2]);
    // Each contact's distance, position and normal.
    let contacts: Vec<(f64, [f64; 3], [f64; 3])> =
        if let Some(raw) = primitive_contacts(model, g1, g2, &pos1, &mat1, &pos2, &mat2, distmax) {
            raw
        } else if type1 == GeomType::Hfield && type2 != GeomType::Sdf {
            collide_hfield_multi(model, g1, g2, pos1, mat1, pos2, mat2, distmax)
                .iter()
                .map(|c| (-c.depth, c.pos.into(), c.normal.into()))
                .collect()
        } else {
            // Every plane function writes the plane's z axis as the normal
            // (`mjraw_PlaneSphere`, `mjc_PlaneBox`, `mjc_PlaneCylinder`,
            // `mjc_PlaneConvex`).
            let plane_z =
                (type1 == GeomType::Plane).then(|| [mat1[(0, 2)], mat1[(1, 2)], mat1[(2, 2)]]);
            collide_geoms(model, g1, g2, pos1, mat1, pos2, mat2, distmax)
                .iter()
                .map(|c| {
                    (
                        -c.depth,
                        c.pos.into(),
                        plane_z.unwrap_or_else(|| c.normal.into()),
                    )
                })
                .collect()
        };
    let mut dist = distmax;
    let mut smallest = None;
    for contact in &contacts {
        if contact.0 < dist {
            dist = contact.0;
            smallest = Some(contact);
        }
    }
    if let Some((_, pos, normal)) = smallest {
        let sign = if flip { -1.0 } else { 1.0 };
        let (from, to) = (-0.5 * sign * dist, 0.5 * sign * dist);
        for k in 0..3 {
            fromto[k] = pos[k] + from * normal[k];
            fromto[3 + k] = pos[k] + to * normal[k];
        }
    }
    (dist, fromto)
}
