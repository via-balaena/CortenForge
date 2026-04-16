//! Composite body expansion — generates body trees from `<composite>` elements.
//!
//! MuJoCo equivalent: `mjCComposite::Make()`, `MakeCable()`, `AddCableBody()`
//! in `user_composite.cc`, and `mjuu_updateFrame()` in `user_util.cc`.
//!
//! In MuJoCo 3.4.0, only `type="cable"` is non-deprecated. All other composite
//! types return deprecation errors matching MuJoCo's exact messages.

use nalgebra::{Rotation3, UnitQuaternion, Vector3};
use std::f64::consts::PI;

use crate::error::{MjcfError, Result};
use crate::types::{
    CompositeType, MjcfBody, MjcfComposite, MjcfContactExclude, MjcfGeom, MjcfGeomType, MjcfJoint,
    MjcfJointType, MjcfSite,
};

/// Expand all composite elements in the body tree.
/// Returns generated contact exclude pairs, or error for deprecated/invalid types.
pub fn expand_composites(body: &mut MjcfBody) -> Result<Vec<MjcfContactExclude>> {
    let mut excludes = Vec::new();
    expand_composites_recursive(body, &mut excludes)?;
    Ok(excludes)
}

fn expand_composites_recursive(
    body: &mut MjcfBody,
    excludes: &mut Vec<MjcfContactExclude>,
) -> Result<()> {
    // First recurse into existing children
    for child in &mut body.children {
        expand_composites_recursive(child, excludes)?;
    }

    // Then expand composites on this body
    let composites = std::mem::take(&mut body.composites);
    for composite in composites {
        match composite.comp_type {
            CompositeType::Cable => {
                let (bodies, excl) = make_cable(&composite)?;
                body.children.extend(bodies);
                excludes.extend(excl);
            }
            CompositeType::Particle => {
                return Err(MjcfError::Unsupported(
                    "The \"particle\" composite type is deprecated. \
                     Please use \"replicate\" instead."
                        .to_string(),
                ));
            }
            CompositeType::Grid => {
                return Err(MjcfError::Unsupported(
                    "The \"grid\" composite type is deprecated. \
                     Please use \"flex\" instead."
                        .to_string(),
                ));
            }
            CompositeType::Rope => {
                return Err(MjcfError::Unsupported(
                    "The \"rope\" composite type is deprecated. \
                     Please use \"cable\" instead."
                        .to_string(),
                ));
            }
            CompositeType::Loop => {
                return Err(MjcfError::Unsupported(
                    "The \"loop\" composite type is deprecated. \
                     Please use \"flexcomp\" instead."
                        .to_string(),
                ));
            }
            CompositeType::Cloth => {
                return Err(MjcfError::Unsupported(
                    "The \"cloth\" composite type is deprecated. \
                     Please use \"shell\" instead."
                        .to_string(),
                ));
            }
        }
    }
    Ok(())
}

/// Validate cable composite constraints before generation.
/// Matches MuJoCo's `Make()` (user_composite.cc:131-193) and
/// `MakeCable()` (user_composite.cc:245-253) validation.
fn validate_cable(composite: &MjcfComposite) -> Result<()> {
    // --- Make()-level validations (before type dispatch) ---

    // Uservert/count mutual exclusion (Make:147-153).
    // If uservert is non-empty, count[0] must be 1 (the default).
    if !composite.uservert.is_empty() && composite.count[0] != 1 {
        return Err(MjcfError::Unsupported(
            "Cannot specify both vertex and count for composite".to_string(),
        ));
    }

    // Uservert length must be a multiple of 3 (xyz triples)
    if !composite.uservert.is_empty() && composite.uservert.len() % 3 != 0 {
        return Err(MjcfError::Unsupported(
            "Composite vertex data must have length divisible by 3".to_string(),
        ));
    }

    // Effective vertex count: uservert overrides count[0] (Make:152-153)
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let effective_count0 = if composite.uservert.is_empty() {
        composite.count[0]
    } else {
        (composite.uservert.len() / 3) as i32
    };

    // Size validation (Make:142-144) — only when no uservert.
    // MuJoCo checks: mju_dot3(size, size) >= mjMINVAL
    if composite.uservert.is_empty() {
        let size_sq = composite.size[0] * composite.size[0]
            + composite.size[1] * composite.size[1]
            + composite.size[2] * composite.size[2];
        if size_sq < 1e-15 {
            return Err(MjcfError::Unsupported(
                "Composite size is too small".to_string(),
            ));
        }
    }

    // Initial value validation — MuJoCo accepts "ball", "none", "free"
    match composite.initial.as_str() {
        "ball" | "none" | "free" => {}
        other => {
            return Err(MjcfError::Unsupported(format!(
                "Invalid composite initial value: \"{other}\". \
                 Must be \"ball\", \"none\", or \"free\""
            )));
        }
    }

    // --- MakeCable()-level validations ---

    // Cable must be 1D: count[1] and count[2] must be 1 (MakeCable:245)
    if composite.count[1] != 1 || composite.count[2] != 1 {
        return Err(MjcfError::Unsupported(
            "Cable must be one-dimensional".to_string(),
        ));
    }

    // Need at least 2 vertices (1 segment)
    if effective_count0 < 2 {
        return Err(MjcfError::Unsupported(
            "Cable requires count >= 2 or at least 2 user-specified vertices".to_string(),
        ));
    }

    // Validate geom type: must be cylinder, capsule, or box (MakeCable:250-253).
    if let Some(ref geom) = composite.geom {
        match geom.geom_type {
            MjcfGeomType::Cylinder | MjcfGeomType::Capsule | MjcfGeomType::Box => {}
            _ => {
                return Err(MjcfError::Unsupported(
                    "Cable geom type must be cylinder, capsule or box".to_string(),
                ));
            }
        }
    } else {
        return Err(MjcfError::Unsupported(
            "Cable composite requires a <geom> child element".to_string(),
        ));
    }

    Ok(())
}

/// Generate cable vertices from curve shapes or user-specified data.
/// Matches `MakeCable()` lines 262-288 in `user_composite.cc`.
#[allow(clippy::cast_precision_loss)] // Cable counts are small; usize→f64 is lossless in practice
fn generate_vertices(composite: &MjcfComposite) -> Vec<[f64; 3]> {
    if !composite.uservert.is_empty() {
        return composite
            .uservert
            .chunks(3)
            .map(|c| [c[0], c[1], c.get(2).copied().unwrap_or(0.0)])
            .collect();
    }

    // `composite.count[0]` is a small non-negative element count from the
    // parsed `<composite count="...">` attribute (validated >= 0 upstream).
    #[allow(clippy::cast_possible_wrap)]
    let n = composite.count[0] as usize;
    let quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        composite.quat[0],
        composite.quat[1],
        composite.quat[2],
        composite.quat[3],
    ));

    let nf = n as f64;
    (0..n)
        .map(|ix| {
            let ixf = ix as f64;
            let curve_val = |k: usize| -> f64 {
                match composite.curve[k] {
                    crate::types::CompositeShape::Line => ixf * composite.size[0] / (nf - 1.0),
                    crate::types::CompositeShape::Cos => {
                        composite.size[1] * (PI * ixf * composite.size[2] / (nf - 1.0)).cos()
                    }
                    crate::types::CompositeShape::Sin => {
                        composite.size[1] * (PI * ixf * composite.size[2] / (nf - 1.0)).sin()
                    }
                    crate::types::CompositeShape::Zero => 0.0,
                }
            };
            let v = [curve_val(0), curve_val(1), curve_val(2)];
            // Rotate by composite quaternion
            let point = Vector3::new(v[0], v[1], v[2]);
            let rotated = quat * point;
            [rotated.x, rotated.y, rotated.z]
        })
        .collect()
}

/// Discrete Bishop frame: compute quaternion orientation for a cable segment.
/// Matches `mjuu_updateFrame()` in `user_util.cc`.
/// Returns `(quaternion, edge_length)`.
fn update_frame(
    normal: &mut Vector3<f64>,
    edge: &Vector3<f64>,
    tprev: &Vector3<f64>,
    tnext: &Vector3<f64>,
    first: bool,
) -> (UnitQuaternion<f64>, f64) {
    let length = edge.norm();
    let tangent = if length > 1e-10 {
        edge / length
    } else {
        Vector3::x()
    };

    if first {
        // Initialize from tangent and next tangent
        let mut binormal = tangent.cross(tnext);
        let bn = binormal.norm();
        if bn > 1e-10 {
            binormal /= bn;
        } else {
            binormal = Vector3::zeros();
        }
        *normal = binormal.cross(&tangent);
        let nn = normal.norm();
        if nn > 1e-10 {
            *normal /= nn;
        } else {
            *normal = Vector3::zeros();
        }
    } else {
        // Darboux rotation: rotate normal about vertex binormal
        let mut binormal_axis = tprev.cross(&tangent);
        let ba_norm = binormal_axis.norm();
        let angle = ba_norm.atan2(tprev.dot(&tangent));
        if ba_norm > 1e-10 {
            binormal_axis /= ba_norm;
            let rot = UnitQuaternion::from_axis_angle(
                &nalgebra::Unit::new_unchecked(binormal_axis),
                angle,
            );
            *normal = rot * *normal;
        }
        let nn = normal.norm();
        if nn > 1e-10 {
            *normal /= nn;
        }
    }

    let binormal = tangent.cross(normal);
    let bn = binormal.norm();
    let binormal_normalized = if bn > 1e-10 { binormal / bn } else { binormal };

    // Construct rotation matrix from frame vectors [tangent, normal, binormal] and convert to quat
    let rot_matrix = nalgebra::Matrix3::from_columns(&[tangent, *normal, binormal_normalized]);
    let quat = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rot_matrix));

    (quat, length)
}

/// Generate cable body chain from a composite definition.
/// Matches `MakeCable()` + `AddCableBody()` in `user_composite.cc`.
#[allow(clippy::unreachable)] // unwrap_or_else fallback: bodies is non-empty after first push
fn make_cable(composite: &MjcfComposite) -> Result<(Vec<MjcfBody>, Vec<MjcfContactExclude>)> {
    validate_cable(composite)?;

    let verts = generate_vertices(composite);
    let n = verts.len(); // count[0]
    let num_bodies = n - 1; // count[0] - 1
    let prefix = &composite.prefix;

    let mut bodies: Vec<MjcfBody> = Vec::new();
    let mut excludes: Vec<MjcfContactExclude> = Vec::new();
    let mut normal = Vector3::new(0.0, 1.0, 0.0);
    let mut prev_quat = UnitQuaternion::identity();

    for ix in 0..num_bodies {
        let last_idx = num_bodies - 1;
        let is_first = ix == 0;
        let is_last = ix == last_idx;

        // Compute edge vector (current vertex to next vertex)
        let edge = Vector3::new(
            verts[ix + 1][0] - verts[ix][0],
            verts[ix + 1][1] - verts[ix][1],
            verts[ix + 1][2] - verts[ix][2],
        );

        // Previous tangent (for Darboux frame rotation)
        let mut tprev = Vector3::zeros();
        let mut length_prev = 0.0;
        if !is_first {
            tprev = Vector3::new(
                verts[ix][0] - verts[ix - 1][0],
                verts[ix][1] - verts[ix - 1][1],
                verts[ix][2] - verts[ix - 1][2],
            );
            length_prev = tprev.norm();
            if length_prev > 1e-10 {
                tprev /= length_prev;
            }
        }

        // Next tangent (for initial frame computation)
        let mut tnext = Vector3::zeros();
        if !is_last {
            tnext = Vector3::new(
                verts[ix + 2][0] - verts[ix + 1][0],
                verts[ix + 2][1] - verts[ix + 1][1],
                verts[ix + 2][2] - verts[ix + 1][2],
            );
            let tn = tnext.norm();
            if tn > 1e-10 {
                tnext /= tn;
            }
        }

        // Update moving frame
        let (this_quat, length) = update_frame(&mut normal, &edge, &tprev, &tnext, is_first);

        // Body, joint, geom names (matching MuJoCo naming convention)
        let body_name = match (is_first, is_last) {
            (true, _) => format!("{prefix}B_first"),
            (_, true) => format!("{prefix}B_last"),
            _ => format!("{prefix}B_{ix}"),
        };
        let next_body_name = if is_last {
            // Last body has no next — but excludes skip last body anyway
            String::new()
        } else if ix + 1 == last_idx {
            format!("{prefix}B_last")
        } else {
            format!("{prefix}B_{}", ix + 1)
        };
        let joint_name = match (is_first, is_last) {
            (true, _) => format!("{prefix}J_first"),
            (_, true) => format!("{prefix}J_last"),
            _ => format!("{prefix}J_{ix}"),
        };
        let geom_name = format!("{prefix}G{ix}");

        // Body position and orientation (matching AddCableBody:373-389)
        let (body_pos, body_quat) = if is_first {
            let pos = Vector3::new(
                composite.offset[0] + verts[ix][0],
                composite.offset[1] + verts[ix][1],
                composite.offset[2] + verts[ix][2],
            );
            (
                pos,
                nalgebra::Vector4::new(this_quat.w, this_quat.i, this_quat.j, this_quat.k),
            )
        } else {
            let pos = Vector3::new(length_prev, 0.0, 0.0);
            let dquat = prev_quat.inverse() * this_quat;
            (
                pos,
                nalgebra::Vector4::new(dquat.w, dquat.i, dquat.j, dquat.k),
            )
        };

        // Build geom from template
        let geom = build_cable_geom(composite, &geom_name, length);

        // Build joint
        let joint = build_cable_joint(composite, &joint_name, is_first);

        // Build sites — first and last body get boundary sites.
        // MuJoCo uses two separate `if` checks (AddCableBody:436-442),
        // NOT if/else. For count=2 the single body is both first AND last,
        // so it gets BOTH S_first and S_last.
        let mut sites = Vec::new();
        if is_first {
            sites.push(MjcfSite {
                name: format!("{prefix}S_first"),
                pos: Some(Vector3::zeros()),
                ..Default::default()
            });
        }
        if is_last {
            sites.push(MjcfSite {
                name: format!("{prefix}S_last"),
                pos: Some(Vector3::new(length, 0.0, 0.0)),
                ..Default::default()
            });
        }

        // Assemble body
        let cable_body = MjcfBody {
            name: body_name.clone(),
            pos: body_pos,
            quat: body_quat,
            geoms: vec![geom],
            joints: joint.into_iter().collect(),
            sites,
            ..Default::default()
        };

        // Contact exclude for all except last (matching AddCableBody:429-433)
        if !is_last {
            excludes.push(MjcfContactExclude {
                name: None,
                body1: body_name.clone(),
                body2: next_body_name,
            });
        }

        prev_quat = this_quat;

        // Chain: each body is a child of the previous
        if bodies.is_empty() {
            bodies.push(cable_body);
        } else {
            // Append to the deepest leaf of the chain
            append_to_chain(
                bodies
                    .last_mut()
                    .unwrap_or_else(|| unreachable!("bodies is non-empty after first push")),
                cable_body,
            );
        }
    }

    Ok((bodies, excludes))
}

/// Append a body to the deepest leaf of a body chain.
/// Produces linear chain: B_first → B_1 → ... → B_last.
#[allow(clippy::unreachable)] // unwrap_or_else fallback: children is non-empty in else branch
fn append_to_chain(parent: &mut MjcfBody, child: MjcfBody) {
    if parent.children.is_empty() {
        parent.children.push(child);
    } else {
        let last = parent
            .children
            .last_mut()
            .unwrap_or_else(|| unreachable!("children is non-empty in else branch"));
        append_to_chain(last, child);
    }
}

/// Build an `MjcfGeom` from the composite's geom template.
/// Matches `AddCableBody:392-403` in `user_composite.cc`.
#[allow(clippy::unreachable)] // unwrap_or_else fallback: geom presence validated by validate_cable() upstream
fn build_cable_geom(composite: &MjcfComposite, name: &str, length: f64) -> MjcfGeom {
    // geom presence validated by validate_cable() before this point
    let template = composite
        .geom
        .as_ref()
        .unwrap_or_else(|| unreachable!("geom presence validated by validate_cable()"));

    let mut geom = MjcfGeom {
        name: Some(name.to_string()),
        geom_type: Some(template.geom_type),
        size: vec![template.size[0], template.size[1], template.size[2]],
        contype: template.contype,
        conaffinity: template.conaffinity,
        condim: template.condim,
        group: template.group,
        mass: template.mass,
        density: template.density,
        solmix: template.solmix,
        solref: template.solref,
        solimp: template.solimp,
        margin: template.margin,
        gap: template.gap,
        material: template.material.clone(),
        priority: template.priority,
        ..Default::default()
    };

    // Set rgba from template (convert [f64; 4] to Vector4)
    if let Some(rgba) = template.rgba {
        geom.rgba = Some(nalgebra::Vector4::new(rgba[0], rgba[1], rgba[2], rgba[3]));
    }

    // Set friction from template (convert [f64; 3] to Vector3)
    if let Some(friction) = template.friction {
        geom.friction = Some(Vector3::new(friction[0], friction[1], friction[2]));
    }

    // Override placement based on geom type (AddCableBody:392-403)
    match template.geom_type {
        MjcfGeomType::Capsule | MjcfGeomType::Cylinder => {
            // fromto along local x-axis: [0,0,0] → [length,0,0]
            geom.fromto = Some([0.0, 0.0, 0.0, length, 0.0, 0.0]);
        }
        MjcfGeomType::Box => {
            // Centered at midpoint, half-length along x
            geom.pos = Some(Vector3::new(length / 2.0, 0.0, 0.0));
            geom.size = vec![length / 2.0, template.size[1], template.size[2]];
        }
        _ => {} // Validated earlier — unreachable
    }

    geom
}

/// Build an `MjcfJoint` for a cable body.
/// Matches `AddCableBody:417-426` in `user_composite.cc`.
fn build_cable_joint(composite: &MjcfComposite, name: &str, is_first: bool) -> Option<MjcfJoint> {
    // No joint if first body and initial="none"
    if is_first && composite.initial == "none" {
        return None;
    }

    let template = composite.joint.as_ref();
    let is_free = is_first && composite.initial == "free";

    let jnt_type = if is_free {
        MjcfJointType::Free
    } else {
        MjcfJointType::Ball
    };

    // Free joints get zero damping/armature/frictionloss (MuJoCo AddCableBody:420-421)
    let (damping, armature, frictionloss) = if is_free {
        (Some(0.0), Some(0.0), Some(0.0))
    } else {
        (
            template.and_then(|t| t.damping),
            template.and_then(|t| t.armature),
            template.and_then(|t| t.frictionloss),
        )
    };

    Some(MjcfJoint {
        name: name.to_string(),
        joint_type: Some(jnt_type),
        damping,
        armature,
        frictionloss,
        group: template.and_then(|t| t.group),
        stiffness: template.and_then(|t| t.stiffness),
        limited: template.and_then(|t| t.limited),
        range: template.and_then(|t| t.range).map(|r| (r[0], r[1])),
        ..Default::default()
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::types::{CompositeGeom, CompositeShape};

    fn make_simple_cable(count: i32) -> MjcfComposite {
        MjcfComposite {
            prefix: String::new(),
            comp_type: CompositeType::Cable,
            count: [count, 1, 1],
            offset: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0],
            initial: "ball".to_string(),
            curve: [
                CompositeShape::Line,
                CompositeShape::Zero,
                CompositeShape::Zero,
            ],
            size: [1.0, 0.0, 0.0],
            uservert: Vec::new(),
            joint: None,
            geom: Some(CompositeGeom {
                geom_type: MjcfGeomType::Capsule,
                size: [0.005, 0.0, 0.0],
                rgba: None,
                contype: None,
                conaffinity: None,
                condim: None,
                group: None,
                friction: None,
                mass: None,
                density: None,
                solmix: None,
                solref: None,
                solimp: None,
                margin: None,
                gap: None,
                material: None,
                priority: None,
            }),
        }
    }

    #[test]
    fn test_generate_vertices_line() {
        let composite = make_simple_cable(5);
        let verts = generate_vertices(&composite);
        assert_eq!(verts.len(), 5);
        // Line from 0 to 1 in x, zero in y/z
        assert!((verts[0][0] - 0.0).abs() < 1e-10);
        assert!((verts[1][0] - 0.25).abs() < 1e-10);
        assert!((verts[4][0] - 1.0).abs() < 1e-10);
        for v in &verts {
            assert!((v[1]).abs() < 1e-10);
            assert!((v[2]).abs() < 1e-10);
        }
    }

    #[test]
    fn test_make_cable_body_count() {
        let composite = make_simple_cable(5);
        let (bodies, excludes) = make_cable(&composite).unwrap();
        // Should produce one root body with 3 nested children (4 bodies total)
        assert_eq!(bodies.len(), 1); // One root (B_first)
        assert_eq!(excludes.len(), 3); // 4 bodies - 1 = 3 exclude pairs

        // Count total bodies in chain
        fn count_bodies(body: &MjcfBody) -> usize {
            1 + body.children.iter().map(count_bodies).sum::<usize>()
        }
        assert_eq!(count_bodies(&bodies[0]), 4);
    }

    #[test]
    fn test_make_cable_naming() {
        let composite = make_simple_cable(5);
        let (bodies, _) = make_cable(&composite).unwrap();

        // Collect all body names
        fn collect_names(body: &MjcfBody, names: &mut Vec<String>) {
            names.push(body.name.clone());
            for child in &body.children {
                collect_names(child, names);
            }
        }
        let mut names = Vec::new();
        collect_names(&bodies[0], &mut names);

        assert_eq!(names, vec!["B_first", "B_1", "B_2", "B_last"]);
    }

    #[test]
    fn test_make_cable_minimum_count() {
        let composite = make_simple_cable(2);
        let (bodies, excludes) = make_cable(&composite).unwrap();

        // count=2 → 1 body that is both first AND last
        fn count_bodies(body: &MjcfBody) -> usize {
            1 + body.children.iter().map(count_bodies).sum::<usize>()
        }
        assert_eq!(count_bodies(&bodies[0]), 1);
        assert_eq!(excludes.len(), 0);

        // The single body should have BOTH S_first and S_last sites
        assert_eq!(bodies[0].sites.len(), 2);
        assert_eq!(bodies[0].sites[0].name, "S_first");
        assert_eq!(bodies[0].sites[1].name, "S_last");
    }

    #[test]
    fn test_validate_cable_too_few() {
        let composite = make_simple_cable(1);
        assert!(validate_cable(&composite).is_err());
    }

    #[test]
    fn test_validate_cable_multi_dim() {
        let mut composite = make_simple_cable(5);
        composite.count = [5, 5, 1];
        assert!(validate_cable(&composite).is_err());
    }

    #[test]
    fn test_deprecated_types() {
        let mut body = MjcfBody::default();
        body.composites.push(MjcfComposite {
            prefix: String::new(),
            comp_type: CompositeType::Rope,
            count: [5, 1, 1],
            offset: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0],
            initial: "ball".to_string(),
            curve: [CompositeShape::Zero; 3],
            size: [1.0, 0.0, 0.0],
            uservert: Vec::new(),
            joint: None,
            geom: None,
        });
        let result = expand_composites(&mut body);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("rope"));
        assert!(err.contains("deprecated"));
    }
}
