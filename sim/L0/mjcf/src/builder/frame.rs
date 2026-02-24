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

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;
    use nalgebra::Vector3;

    // =========================================================================
    // Frame expansion + childclass tests (Spec #19, 27 acceptance criteria)
    // =========================================================================

    // AC1: Frame position
    #[test]
    fn test_ac01_frame_position() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                        <frame pos="1 0 0">
                            <geom name="fg" type="sphere" size="0.1" pos="0 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(model.geom_pos.len(), 2);
        let pos = model.geom_pos[1];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-10,
            "frame geom should be at (1,0,0), got {pos:?}"
        );
    }

    // AC2: Frame rotation
    #[test]
    fn test_ac02_frame_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <geom name="fg" type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "90deg Z rotation should map (1,0,0) to (0,1,0), got {pos:?}"
        );
    }

    // AC3: Frame position + rotation
    #[test]
    fn test_ac03_frame_pos_plus_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0" euler="0 0 90">
                            <geom name="fg" type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (1,1,0), got {pos:?}"
        );
    }

    // AC4: Nested frames
    #[test]
    fn test_ac04_nested_frames() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <frame pos="0 1 0">
                                <geom name="fg" type="sphere" size="0.1"/>
                            </frame>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 1.0, 0.0)).norm() < 1e-10,
            "expected (1,1,0), got {pos:?}"
        );
    }

    // AC5: 3-deep nested frames
    #[test]
    fn test_ac05_three_deep_nested_frames() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <frame pos="0 1 0">
                                <frame pos="0 0 1">
                                    <geom name="fg" type="sphere" size="0.1"/>
                                </frame>
                            </frame>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 1.0, 1.0)).norm() < 1e-10,
            "expected (1,1,1), got {pos:?}"
        );
    }

    // AC6: Frame wrapping body
    #[test]
    fn test_ac06_frame_wrapping_body() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="parent">
                        <frame pos="2 0 0">
                            <body name="child" pos="1 0 0">
                                <geom type="sphere" size="0.1"/>
                            </body>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(model.body_pos.len(), 3);
        let child_pos = model.body_pos[2];
        assert!(
            (child_pos - Vector3::new(3.0, 0.0, 0.0)).norm() < 1e-10,
            "body_pos should be (3,0,0), got {child_pos:?}"
        );
    }

    // AC7: Frame with fromto geom
    #[test]
    fn test_ac07_frame_fromto_geom() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <geom type="capsule" fromto="0 0 0 0 0 1" size="0.05"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.5)).norm() < 1e-6,
            "expected midpoint (1,0,0.5), got {pos:?}"
        );
    }

    // AC8: Frame with fromto geom + rotation
    #[test]
    fn test_ac08_frame_fromto_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 90 0">
                            <geom type="capsule" fromto="0 0 0 0 0 1" size="0.05"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.5, 0.0, 0.0)).norm() < 1e-6,
            "expected midpoint (0.5,0,0), got {pos:?}"
        );
    }

    // AC9: Frame with site
    #[test]
    fn test_ac09_frame_with_site() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="0.5 0 0">
                            <site name="s" pos="0 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.site_pos[0];
        assert!(
            (pos - Vector3::new(0.5, 0.0, 0.0)).norm() < 1e-10,
            "site should be at (0.5,0,0), got {pos:?}"
        );
    }

    // AC10: Empty frame
    #[test]
    fn test_ac10_empty_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(model.geom_pos.len(), 1);
        assert!(
            (model.geom_pos[0]).norm() < 1e-10,
            "geom should be at origin (empty frame has no children)"
        );
    }

    // AC11: Frame at worldbody level
    #[test]
    fn test_ac11_frame_at_worldbody() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <frame pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                    </frame>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-10,
            "worldbody frame geom should be at (1,0,0), got {pos:?}"
        );
    }

    // AC12: Frame with only orientation (no pos)
    #[test]
    fn test_ac12_frame_only_orientation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // AC13: Frame with only position (no orientation)
    #[test]
    fn test_ac13_frame_only_position() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="3 0 0">
                            <geom type="sphere" size="0.1" pos="0 2 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(3.0, 2.0, 0.0)).norm() < 1e-10,
            "expected (3,2,0), got {pos:?}"
        );
    }

    // AC14: Frame with xyaxes
    #[test]
    fn test_ac14_frame_xyaxes() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame xyaxes="0 1 0 -1 0 0">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // AC15: Frame with zaxis
    #[test]
    fn test_ac15_frame_zaxis() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame zaxis="1 0 0">
                            <geom type="sphere" size="0.1" pos="0 0 1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-6,
            "expected (1,0,0), got {pos:?}"
        );
    }

    // AC16: Frame with axisangle
    #[test]
    fn test_ac16_frame_axisangle() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame axisangle="0 0 1 90">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // AC17: Joint inside frame = error
    #[test]
    fn test_ac17_joint_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame>
                            <joint name="j" type="hinge"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(result.is_err(), "joint inside frame should error");
        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("not allowed inside <frame>"),
            "error should mention invalid element: {err}"
        );
    }

    // AC18: Freejoint inside frame = error
    #[test]
    fn test_ac18_freejoint_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame>
                            <freejoint/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(result.is_err(), "freejoint inside frame should error");
    }

    // AC19: Inertial inside frame = error
    #[test]
    fn test_ac19_inertial_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame>
                            <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(result.is_err(), "inertial inside frame should error");
    }

    // AC20: Childclass on body
    #[test]
    fn test_ac20_childclass_on_body() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="red">
                        <geom contype="7"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="red">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(
            model.geom_contype[0], 7,
            "geom should inherit contype=7 from childclass 'red', got {}",
            model.geom_contype[0]
        );
    }

    // AC21: Childclass override by explicit class
    #[test]
    fn test_ac21_childclass_override_by_explicit() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="red">
                        <geom contype="7"/>
                    </default>
                    <default class="blue">
                        <geom contype="3"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="red">
                        <geom class="blue" type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(
            model.geom_contype[0], 3,
            "geom should use explicit class 'blue' contype=3, got {}",
            model.geom_contype[0]
        );
    }

    // AC22: Childclass on frame
    #[test]
    fn test_ac22_childclass_on_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="red">
                        <geom contype="7"/>
                    </default>
                    <default class="green">
                        <geom contype="3"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="red">
                        <frame childclass="green">
                            <geom type="sphere" size="0.1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(
            model.geom_contype[0], 3,
            "geom should inherit frame childclass 'green' contype=3, got {}",
            model.geom_contype[0]
        );
    }

    // AC23: Childclass inheritance through body hierarchy
    #[test]
    fn test_ac23_childclass_inheritance_hierarchy() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="robot">
                        <joint damping="5.0"/>
                    </default>
                </default>
                <worldbody>
                    <body name="parent" childclass="robot">
                        <body name="child" pos="0 0 1">
                            <joint name="j1" type="hinge"/>
                            <geom type="sphere" size="0.1"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert!(
            (model.dof_damping[0] - 5.0).abs() < 1e-10,
            "joint should inherit damping=5.0 from childclass 'robot', got {}",
            model.dof_damping[0]
        );
    }

    // AC24: Childclass on child body overrides parent
    #[test]
    fn test_ac24_childclass_child_overrides_parent() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="A">
                        <geom contype="7"/>
                    </default>
                    <default class="B">
                        <geom contype="3"/>
                    </default>
                </default>
                <worldbody>
                    <body name="outer" childclass="A">
                        <body name="inner" childclass="B">
                            <geom type="sphere" size="0.1"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(
            model.geom_contype[0], 3,
            "geom should inherit child body childclass 'B' contype=3, got {}",
            model.geom_contype[0]
        );
    }

    // AC25: Regression - no frames, no childclass
    #[test]
    fn test_ac25_regression_no_frames() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b" pos="1 2 3">
                        <joint name="j" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                        <site name="s" pos="0.5 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(model.nbody, 2);
        assert_eq!(model.ngeom, 1);
        let body_pos = model.body_pos[1];
        assert!((body_pos - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-10);
    }

    // AC26: Geom orientation composition
    #[test]
    fn test_ac26_geom_orientation_composition() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <geom type="sphere" size="0.1" euler="90 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let quat = model.geom_quat[0];
        let is_identity = (quat.w - 1.0).abs() < 1e-6
            && quat.i.abs() < 1e-6
            && quat.j.abs() < 1e-6
            && quat.k.abs() < 1e-6;
        assert!(!is_identity, "composed rotation should not be identity");
    }

    // AC27: Frame + angle="radian"
    #[test]
    fn test_ac27_frame_radian_mode() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 1.5707963">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-4,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // ========================================================================
    // AC28-AC35: Childclass edge cases (item #20)
    // ========================================================================

    // AC28: Childclass referencing a nested default class
    #[test]
    fn test_ac28_childclass_nested_default_hierarchy() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="robot">
                        <geom contype="5"/>
                        <default class="arm">
                            <geom conaffinity="3"/>
                        </default>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="arm">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(
            model.geom_contype[0], 5,
            "geom should inherit contype=5 from parent class 'robot' through 'arm', got {}",
            model.geom_contype[0]
        );
        assert_eq!(
            model.geom_conaffinity[0], 3,
            "geom should get conaffinity=3 from class 'arm', got {}",
            model.geom_conaffinity[0]
        );
    }

    // AC29: Childclass applies to geom, joint, AND site simultaneously
    #[test]
    fn test_ac29_childclass_multi_element() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <default>
                    <default class="R">
                        <geom contype="7"/>
                        <joint damping="5.0"/>
                        <site size="0.05"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="R">
                        <joint name="j" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                        <site name="s"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(
            model.geom_contype[0], 7,
            "geom should inherit contype=7 from childclass 'R', got {}",
            model.geom_contype[0]
        );
        assert!(
            (model.dof_damping[0] - 5.0).abs() < 1e-10,
            "joint should inherit damping=5.0 from childclass 'R', got {}",
            model.dof_damping[0]
        );
        let site_size = model.site_size[0];
        assert!(
            (site_size.x - 0.05).abs() < 1e-10,
            "site should inherit size=0.05 from childclass 'R', got {}",
            site_size.x
        );
    }

    // AC30: 3-level deep propagation without override
    #[test]
    fn test_ac30_childclass_3level_propagation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <default>
                    <default class="X">
                        <joint damping="3.0"/>
                    </default>
                </default>
                <worldbody>
                    <body name="top" childclass="X">
                        <geom type="sphere" size="0.1"/>
                        <body name="mid" pos="0 0 1">
                            <geom type="sphere" size="0.1"/>
                            <body name="bot" pos="0 0 1">
                                <joint name="j" type="hinge"/>
                                <geom type="sphere" size="0.1"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert!(
            (model.dof_damping[0] - 3.0).abs() < 1e-10,
            "joint at 3rd level should inherit damping=3.0 from top's childclass 'X', got {}",
            model.dof_damping[0]
        );
    }

    // AC31: 3-level with mid-hierarchy override
    #[test]
    fn test_ac31_childclass_3level_mid_override() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <default>
                    <default class="A"><joint damping="1.0"/></default>
                    <default class="B"><joint damping="9.0"/></default>
                </default>
                <worldbody>
                    <body name="top" childclass="A">
                        <geom type="sphere" size="0.1"/>
                        <body name="mid" childclass="B" pos="0 0 1">
                            <geom type="sphere" size="0.1"/>
                            <body name="bot" pos="0 0 1">
                                <joint name="j" type="hinge"/>
                                <geom type="sphere" size="0.1"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert!(
            (model.dof_damping[0] - 9.0).abs() < 1e-10,
            "joint should inherit damping=9.0 from mid's childclass 'B' (not top's 'A'), got {}",
            model.dof_damping[0]
        );
    }

    // AC32: Nested frames with childclass inheritance and override
    #[test]
    fn test_ac32_nested_frames_childclass() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="F1"><geom contype="4"/></default>
                    <default class="F2"><geom contype="8"/></default>
                </default>
                <worldbody>
                    <body name="b">
                        <frame childclass="F1">
                            <frame>
                                <geom name="g1" type="sphere" size="0.1"/>
                            </frame>
                            <frame childclass="F2">
                                <geom name="g2" type="sphere" size="0.1"/>
                            </frame>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(
            model.geom_contype[0], 4,
            "g1 should inherit contype=4 from outer frame childclass 'F1', got {}",
            model.geom_contype[0]
        );
        assert_eq!(
            model.geom_contype[1], 8,
            "g2 should get contype=8 from inner frame childclass 'F2', got {}",
            model.geom_contype[1]
        );
    }

    // AC33: childclass="nonexistent" on body produces error
    #[test]
    fn test_ac33_childclass_nonexistent_body_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b" childclass="nonexistent">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(
            result.is_err(),
            "childclass='nonexistent' should produce an error"
        );
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("nonexistent"),
            "error message should mention 'nonexistent', got: {err_msg}"
        );
    }

    // AC34: Body with childclass but no child elements succeeds
    #[test]
    fn test_ac34_childclass_empty_body() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="empty"><geom contype="5"/></default>
                </default>
                <worldbody>
                    <body name="b" childclass="empty" pos="0 0 1">
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(model.nbody, 2, "should have world + b");
        assert_eq!(model.ngeom, 0, "body has no geoms");
    }

    // AC35: childclass="ghost" on frame produces error
    #[test]
    fn test_ac35_childclass_nonexistent_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame childclass="ghost">
                            <geom type="sphere" size="0.1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(
            result.is_err(),
            "childclass='ghost' on frame should produce an error"
        );
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("ghost"),
            "error message should mention 'ghost', got: {err_msg}"
        );
    }

    // fromto geom inside a frame with both position and rotation
    #[test]
    fn test_fromto_geom_in_frame_with_pos_and_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0" euler="0 0 90">
                            <geom type="capsule" size="0.05" fromto="0 0 0 0 0 1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.5)).norm() < 1e-4,
            "expected (1,0,0.5), got {pos:?}"
        );
    }
}
