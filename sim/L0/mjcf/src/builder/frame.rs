//! Frame expansion for MJCF body trees.
//!
//! `<frame>` elements are coordinate-system wrappers that compose their SE(3)
//! transform into each child element and then disappear.  This module expands
//! all frames before the builder processes bodies.
//!
//! Each element a frame holds reaches it with its class applied and its
//! `fromto` resolved (`resolve::apply_defaults`), so a frame composes the
//! element's own pose.

use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};

use super::composite::resolve_made_fromtos;
use super::items::{Contents, Element};
use super::orientation::{
    made_unit, mulquat, mulvecmat, quat_as_written, quat_to_wxyz, quat2mat, resolve_orientation,
    site_quat_as_written,
};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{MjcfBody, MjcfCompiler, MjcfFrame};

/// A frame's transform composed into a child's pose, in MuJoCo 3.5.0's
/// arithmetic (`mjuu_frameaccumChild`, `user_util.cc:483-490`, through
/// `mjuu_frameaccum`, `:469-479`): the child's position turned by the frame's
/// rotation matrix (`quat2mat`, `mulvecmat`) and added to the frame's, and
/// its quaternion the frame's times its own, made unit (`mulquat`).
fn frame_accum_child(
    frame_pos: &Vector3<f64>,
    frame_quat: &UnitQuaternion<f64>,
    child_pos: &mut Vector3<f64>,
    child_quat: &mut UnitQuaternion<f64>,
) {
    let frame = wxyz(frame_quat);
    let turned = mulvecmat(&[child_pos.x, child_pos.y, child_pos.z], &quat2mat(&frame));
    *child_pos = Vector3::new(
        frame_pos.x + turned[0],
        frame_pos.y + turned[1],
        frame_pos.z + turned[2],
    );
    let [w, x, y, z] = mulquat(&frame, &wxyz(child_quat));
    *child_quat = UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z));
}

/// `q`'s components, (w, x, y, z).
fn wxyz(q: &UnitQuaternion<f64>) -> [f64; 4] {
    [q.w, q.i, q.j, q.k]
}

/// Expand every `<frame>` of a body tree.
///
/// Frames are coordinate-system wrappers that compose their transform into
/// each element they hold and then disappear. A frame's contents take its
/// place among its body's elements, in the order the document writes them
/// (`MjcfBody::items`), as MuJoCo 3.5.0 adds each element to its body when it
/// reads it. This runs before `discardvisual`/`fusestatic` and before
/// `ModelBuilder` processes bodies.
///
/// Each geom and site a frame holds reaches it resolved
/// (`resolve::apply_defaults`): its class's values applied and its `fromto`
/// made into its `pos`, `quat` and `size`. Once a body's frames are
/// dissolved, the geoms a composite made in it are resolved
/// ([`resolve_made_fromtos`]), before the bodies below it. Elements are
/// moved, never copied, so expanding takes memory in proportion to the tree.
///
/// # Errors
///
/// An orientation alternative `ResolveOrientation` refuses, on a frame or on
/// an element in one: of one body's, the one MuJoCo 3.5.0 names first
/// ([`Faults`]); and a `fromto` of a geom a composite made.
pub fn expand_frames(body: &mut MjcfBody, compiler: &MjcfCompiler) -> Result<()> {
    expand_body(body, None, compiler)
}

/// [`expand_frames`] for `body`, with `own` the refusal of its own
/// orientation when the frame holding it was dissolved, if one was.
fn expand_body(body: &mut MjcfBody, own: Option<MjcfError>, compiler: &MjcfCompiler) -> Result<()> {
    let mut placed = Contents::default();
    let mut faults = Faults::default();
    for element in Contents::take_from_body(body).into_elements() {
        match element {
            Element::Frame(frame) => {
                dissolve(frame, None, &mut placed, &mut faults, &body.name, compiler);
            }
            other => placed.push(other),
        }
    }
    faults.first(own)?;
    placed.put_into_body(body);
    resolve_made_fromtos(body)?;
    let mut held = faults.held.into_iter().peekable();
    for (index, child) in body.children.iter_mut().enumerate() {
        let own = held.next_if(|(i, _)| *i == index).map(|(_, fault)| fault);
        expand_body(child, own, compiler)?;
    }
    Ok(())
}

/// The orientations one body's frames and the elements in them refuse, kept
/// to name the one MuJoCo 3.5.0 names first. MuJoCo compiles bodies one by
/// one (`user_model.cc:4933`), each its frames, then its own orientation,
/// then its geoms, then its sites (`mjCBody::Compile`,
/// `user_objects.cc:2376-2378`, `:2396-2402`, `:2427-2433`, `:2551-2552`).
/// Of each kind, the first in document order. A body a frame holds is one of
/// the body's children: its own orientation's refusal is named in its own
/// turn.
#[derive(Default)]
struct Faults {
    frame: Option<MjcfError>,
    geom: Option<MjcfError>,
    site: Option<MjcfError>,
    /// Each held body's own refusal, by its index among the body's children.
    held: Vec<(usize, MjcfError)>,
}

impl Faults {
    /// `fault` kept in `slot`, unless an earlier one is.
    fn keep(slot: &mut Option<MjcfError>, fault: MjcfError) {
        slot.get_or_insert(fault);
    }

    /// The fault MuJoCo names first in the body's turn, with `own` its own
    /// orientation's refusal, if any.
    fn first(&mut self, own: Option<MjcfError>) -> Result<()> {
        self.frame
            .take()
            .or(own)
            .or_else(|| self.geom.take())
            .or_else(|| self.site.take())
            .map_or(Ok(()), Err)
    }
}

/// One frame's contents, in its document order, composed with its transform
/// (itself composed with `outer`'s, the pose of the frame holding it, where a
/// frame holds it) and added to `placed`, the elements of `body`; a nested
/// frame's at its place. An orientation refused is kept in `faults`, and its
/// element left out (a frame's with its contents), or, a body's, placed as
/// written.
fn dissolve(
    mut frame: MjcfFrame,
    outer: Option<(&Vector3<f64>, &UnitQuaternion<f64>)>,
    placed: &mut Contents,
    faults: &mut Faults,
    body: &str,
    compiler: &MjcfCompiler,
) {
    let in_body =
        |kind, name: Option<&str>| Location::named(kind, name.unwrap_or("")).in_body(body);
    // The frame's own orientation as written, composed with the frame holding
    // it where one does, then made unit, as `mjCFrame::Compile` makes it
    // (`user_objects.cc:2734-2745`); a frame no frame holds keeps its
    // position as written.
    let written = match quat_as_written(
        frame.quat,
        frame.euler,
        frame.axisangle,
        frame.xyaxes,
        frame.zaxis,
        compiler,
    ) {
        Ok(q) => q,
        Err(e) => {
            let at = in_body(ElementKind::Frame, frame.name.as_deref());
            Faults::keep(&mut faults.frame, e.on("frame").at(at));
            return;
        }
    };
    let mut pos = frame.pos;
    let mut quat = written;
    if let Some((outer_pos, outer_quat)) = outer {
        frame_accum_child(outer_pos, outer_quat, &mut pos, &mut quat);
    }
    let quat = made_unit(&quat);

    for element in Contents::take_from_frame(&mut frame).into_elements() {
        let element = match element {
            // Each geom's orientation (a `fromto` resolved already gives its
            // `quat`), composed with the frame's.
            Element::Geom(mut g) => {
                let geom_quat = match resolve_orientation(
                    g.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0)),
                    g.euler,
                    g.axisangle,
                    g.xyaxes,
                    g.zaxis,
                    compiler,
                ) {
                    Ok(q) => q,
                    Err(e) => {
                        let at = in_body(ElementKind::Geom, g.name.as_deref());
                        Faults::keep(&mut faults.geom, e.on("geom").at(at));
                        continue;
                    }
                };
                let mut gpos = g.pos.unwrap_or_else(Vector3::zeros);
                let mut gquat = geom_quat;
                frame_accum_child(&pos, &quat, &mut gpos, &mut gquat);
                g.pos = Some(gpos);
                g.quat = Some(quat_to_wxyz(&gquat));
                g.euler = None;
                g.axisangle = None;
                g.xyaxes = None;
                g.zaxis = None;
                Element::Geom(g)
            }
            // Each site's quaternion as written, composed with the frame's,
            // as MuJoCo composes a site's (`user_objects.cc:3976-3988`).
            Element::Site(mut s) => {
                let mut squat = match site_quat_as_written(&s, compiler) {
                    Ok(q) => q,
                    Err(e) => {
                        let at = in_body(ElementKind::Site, Some(&s.name));
                        Faults::keep(&mut faults.site, e.on("site").at(at));
                        continue;
                    }
                };
                let mut spos = s.pos.unwrap_or_else(Vector3::zeros);
                frame_accum_child(&pos, &quat, &mut spos, &mut squat);
                s.pos = Some(spos);
                s.quat = Some(quat_to_wxyz(&squat));
                s.euler = None;
                s.axisangle = None;
                s.xyaxes = None;
                s.zaxis = None;
                Element::Site(s)
            }
            Element::Body(mut b) => {
                let body_quat = match resolve_orientation(
                    b.quat,
                    b.euler,
                    b.axisangle,
                    b.xyaxes,
                    b.zaxis,
                    compiler,
                ) {
                    Ok(q) => q,
                    Err(e) => {
                        let at = Location::named(ElementKind::Body, &b.name);
                        faults.held.push((placed.body_count(), e.on("body").at(at)));
                        placed.push(Element::Body(b));
                        continue;
                    }
                };
                b.pose_as_written = Some((b.pos, quat_to_wxyz(&body_quat)));
                let mut bpos = b.pos;
                let mut bquat = body_quat;
                frame_accum_child(&pos, &quat, &mut bpos, &mut bquat);
                b.pos = bpos;
                b.quat = quat_to_wxyz(&bquat);
                b.euler = None;
                b.axisangle = None;
                b.xyaxes = None;
                b.zaxis = None;
                Element::Body(b)
            }
            Element::Frame(nested) => {
                dissolve(nested, Some((&pos, &quat)), placed, faults, body, compiler);
                continue;
            }
            // A frame holds no joint (the reader refuses one), and its
            // composites were made into bodies before frames.
            // A flexcomp keeps its place and is not moved: its bodies are
            // its frame's body's, as MuJoCo gives it the body, not the frame.
            other @ (Element::Joint(_) | Element::Composite(_) | Element::Flexcomp(_)) => other,
        };
        placed.push(element);
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;
    use crate::error::MjcfError;
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

    // AC17: a joint inside a frame, which MuJoCo 3.5.0 loads (registry
    // `D-FRAME-JOINT`; the body's geom gives it mass), refused.
    #[test]
    fn test_ac17_joint_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom size="0.1"/>
                        <frame pos="1 0 0">
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
            matches!(&err, MjcfError::Unsupported { feature, .. } if feature == "<joint> inside <frame>"),
            "{err}"
        );
    }

    // AC18: a free joint inside a frame, which MuJoCo 3.5.0 loads (registry
    // `D-FRAME-JOINT`), refused.
    #[test]
    fn test_ac18_freejoint_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom size="0.1"/>
                        <frame>
                            <freejoint/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        let err = result.unwrap_err();
        assert!(
            matches!(&err, MjcfError::Unsupported { feature, .. } if feature == "<freejoint> inside <frame>"),
            "{err}"
        );
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
