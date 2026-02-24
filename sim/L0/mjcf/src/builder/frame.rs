//! Frame expansion for MJCF body trees.
//!
//! `<frame>` elements are coordinate-system wrappers that compose their SE(3)
//! transform into each child element and then disappear.  This module expands
//! all frames before the builder processes bodies, matching MuJoCo's ordering.
//!
//! Also validates `childclass` references, which must run *before* frame
//! expansion because `std::mem::take` dissolves the frame nodes.

use nalgebra::{UnitQuaternion, Vector3, Vector4};

use super::ModelConversionError;
use super::orientation::{quat_to_wxyz, resolve_orientation};
use crate::defaults::DefaultResolver;
use crate::types::{MjcfBody, MjcfCompiler, MjcfFrame};

/// Core SE(3) composition — direct translation of MuJoCo's `mjuu_frameaccumChild`.
///
/// Composes a parent frame transform into a child's pos/quat:
/// - `child_pos_new = frame_pos + frame_quat * child_pos_old`
/// - `child_quat_new = frame_quat * child_quat_old`
fn frame_accum_child(
    frame_pos: &Vector3<f64>,
    frame_quat: &UnitQuaternion<f64>,
    child_pos: &mut Vector3<f64>,
    child_quat: &mut UnitQuaternion<f64>,
) {
    *child_pos = *frame_pos + frame_quat.transform_vector(child_pos);
    *child_quat = *frame_quat * *child_quat;
}

/// Validate that all `childclass` references in the body tree point to defined
/// default classes. Must run BEFORE `expand_frames()` because frame expansion
/// dissolves frames (via `std::mem::take`), losing `frame.childclass` values.
///
/// MuJoCo rejects undefined childclass references at schema validation (S7).
pub fn validate_childclass_references(
    body: &MjcfBody,
    resolver: &DefaultResolver,
) -> std::result::Result<(), ModelConversionError> {
    if let Some(ref cc) = body.childclass {
        if resolver.get_defaults(Some(cc.as_str())).is_none() {
            return Err(ModelConversionError {
                message: format!(
                    "childclass '{}' on body '{}' references undefined default class",
                    cc, body.name
                ),
            });
        }
    }
    for frame in &body.frames {
        validate_frame_childclass_refs(frame, resolver)?;
    }
    for child in &body.children {
        validate_childclass_references(child, resolver)?;
    }
    Ok(())
}

/// Validate childclass references on a frame and its nested contents.
fn validate_frame_childclass_refs(
    frame: &MjcfFrame,
    resolver: &DefaultResolver,
) -> std::result::Result<(), ModelConversionError> {
    if let Some(ref cc) = frame.childclass {
        if resolver.get_defaults(Some(cc.as_str())).is_none() {
            return Err(ModelConversionError {
                message: format!(
                    "childclass '{}' on frame '{}' references undefined default class",
                    cc,
                    frame.name.as_deref().unwrap_or("<unnamed>")
                ),
            });
        }
    }
    for nested in &frame.frames {
        validate_frame_childclass_refs(nested, resolver)?;
    }
    for child_body in &frame.bodies {
        validate_childclass_references(child_body, resolver)?;
    }
    Ok(())
}

/// Recursively expand all `<frame>` elements in a body tree.
///
/// Frames are coordinate-system wrappers that compose their transform into
/// each child element and then disappear. This runs before `discardvisual`/`fusestatic`
/// (matching MuJoCo's order) and before `ModelBuilder` processes bodies.
pub fn expand_frames(
    body: &mut MjcfBody,
    compiler: &MjcfCompiler,
    parent_childclass: Option<&str>,
) {
    // Resolve effective childclass early and own it to avoid borrow conflicts
    let effective_owned: Option<String> = body
        .childclass
        .clone()
        .or_else(|| parent_childclass.map(|s| s.to_string()));
    let effective = effective_owned.as_deref();

    // Take frames out of the body to avoid borrow issues
    let frames = std::mem::take(&mut body.frames);

    for frame in &frames {
        expand_single_frame(
            body,
            frame,
            &Vector3::zeros(),
            &UnitQuaternion::identity(),
            compiler,
            effective,
        );
    }

    // Recursively expand frames in child bodies
    for child in &mut body.children {
        expand_frames(child, compiler, effective);
    }
}

/// Expand a single frame, composing its transform into children and lifting
/// them onto the parent body.
fn expand_single_frame(
    body: &mut MjcfBody,
    frame: &MjcfFrame,
    accumulated_pos: &Vector3<f64>,
    accumulated_quat: &UnitQuaternion<f64>,
    compiler: &MjcfCompiler,
    parent_childclass: Option<&str>,
) {
    // 1. Resolve the frame's own orientation
    let frame_quat = resolve_orientation(
        frame.quat,
        frame.euler,
        frame.axisangle,
        frame.xyaxes,
        frame.zaxis,
        compiler,
    );

    // 2. Compose with accumulated parent-frame transform
    let mut composed_pos = frame.pos;
    let mut composed_quat = frame_quat;
    frame_accum_child(
        accumulated_pos,
        accumulated_quat,
        &mut composed_pos,
        &mut composed_quat,
    );

    // 3. Determine effective childclass: frame's overrides parent's
    let effective = frame.childclass.as_deref().or(parent_childclass);

    // 4. Process child geoms
    for geom in &frame.geoms {
        let mut g = geom.clone();

        // Apply childclass if geom has no explicit class
        if g.class.is_none() {
            g.class = effective.map(|s| s.to_string());
        }

        if let Some(ref mut fromto) = g.fromto {
            // fromto geoms: transform both endpoints through the composed frame.
            // Leave pos/quat untouched — compute_fromto_pose() will derive them later.
            let from = Vector3::new(fromto[0], fromto[1], fromto[2]);
            let to = Vector3::new(fromto[3], fromto[4], fromto[5]);
            let from_new = composed_pos + composed_quat.transform_vector(&from);
            let to_new = composed_pos + composed_quat.transform_vector(&to);
            *fromto = [
                from_new.x, from_new.y, from_new.z, to_new.x, to_new.y, to_new.z,
            ];
        } else {
            // Non-fromto geoms: resolve orientation, compose with frame
            let geom_quat = resolve_orientation(
                g.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0)),
                g.euler,
                g.axisangle,
                g.xyaxes,
                g.zaxis,
                compiler,
            );
            let mut pos = g.pos.unwrap_or_else(Vector3::zeros);
            let mut quat = geom_quat;
            frame_accum_child(&composed_pos, &composed_quat, &mut pos, &mut quat);
            g.pos = Some(pos);
            g.quat = Some(quat_to_wxyz(&quat));
            // Clear alternative orientations (already resolved)
            g.euler = None;
            g.axisangle = None;
            g.xyaxes = None;
            g.zaxis = None;
        }

        body.geoms.push(g);
    }

    // 5. Process child sites
    for site in &frame.sites {
        let mut s = site.clone();

        if s.class.is_none() {
            s.class = effective.map(|s| s.to_string());
        }

        let site_quat = resolve_orientation(
            s.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0)),
            s.euler,
            s.axisangle,
            s.xyaxes,
            s.zaxis,
            compiler,
        );
        let mut pos = s.pos.unwrap_or_else(Vector3::zeros);
        let mut quat = site_quat;
        frame_accum_child(&composed_pos, &composed_quat, &mut pos, &mut quat);
        s.pos = Some(pos);
        s.quat = Some(quat_to_wxyz(&quat));
        s.euler = None;
        s.axisangle = None;
        s.xyaxes = None;
        s.zaxis = None;

        body.sites.push(s);
    }

    // 6. Process child bodies
    for child_body in &frame.bodies {
        let mut b = child_body.clone();

        // If body has no own childclass, inherit from frame's effective childclass
        if b.childclass.is_none() {
            b.childclass = effective.map(|s| s.to_string());
        }

        let body_quat = resolve_orientation(b.quat, b.euler, b.axisangle, None, None, compiler);
        let mut pos = b.pos;
        let mut quat = body_quat;
        frame_accum_child(&composed_pos, &composed_quat, &mut pos, &mut quat);
        b.pos = pos;
        b.quat = quat_to_wxyz(&quat);
        b.euler = None;
        b.axisangle = None;

        body.children.push(b);
    }

    // 7. Recurse into nested frames
    for nested in &frame.frames {
        expand_single_frame(
            body,
            nested,
            &composed_pos,
            &composed_quat,
            compiler,
            effective,
        );
    }
}
