//! MJCF compiler pre-processing passes.
//!
//! `discardvisual` removes visual-only geoms and unreferenced meshes.
//! `fusestatic` merges jointless bodies into their parents, reducing the
//! body count while preserving physics.  Both passes run on the parsed
//! `MjcfModel` tree *before* the builder converts it to a physics `Model`.

use std::collections::HashSet;

use super::orientation::{quat_to_wxyz, resolve_orientation};
use crate::types::{MjcfBody, MjcfCompiler, MjcfModel};

/// Apply `discardvisual` pre-processing: remove visual-only geoms (contype=0, conaffinity=0)
/// and unreferenced mesh assets. Geoms referenced by sensors or actuators are protected.
pub fn apply_discardvisual(mjcf: &mut MjcfModel) {
    // Collect geom names referenced by sensors (site-based sensors reference sites, not geoms,
    // but frame sensors can resolve to geoms) and actuators (no geom refs currently, but future-proof).
    let mut protected_geoms: HashSet<String> = HashSet::new();
    for sensor in &mjcf.sensors {
        // Frame sensors can resolve objname as a geom; protect any objname that might be a geom.
        if let Some(ref name) = sensor.objname {
            protected_geoms.insert(name.clone());
        }
    }

    fn remove_visual_geoms(body: &mut MjcfBody, protected: &HashSet<String>) {
        body.geoms.retain(|g| {
            g.contype.unwrap_or(1) != 0
                || g.conaffinity.unwrap_or(1) != 0
                || g.name.as_ref().is_some_and(|n| protected.contains(n))
        });
        for child in &mut body.children {
            remove_visual_geoms(child, protected);
        }
    }

    remove_visual_geoms(&mut mjcf.worldbody, &protected_geoms);

    // Collect all mesh names still referenced
    fn collect_mesh_refs(body: &MjcfBody, refs: &mut HashSet<String>) {
        for g in &body.geoms {
            if let Some(ref m) = g.mesh {
                refs.insert(m.clone());
            }
        }
        for child in &body.children {
            collect_mesh_refs(child, refs);
        }
    }
    let mut used_meshes = HashSet::new();
    collect_mesh_refs(&mjcf.worldbody, &mut used_meshes);

    // Remove unreferenced meshes
    mjcf.meshes.retain(|m| used_meshes.contains(&m.name));
}

/// Apply `fusestatic` pre-processing: fuse jointless bodies into their parent.
///
/// Bodies without joints are "static" relative to their parent. Fusing them
/// transfers their geoms, sites, and children to the parent, reducing the body
/// count. Bodies referenced by equality constraints, sensors, or actuators are
/// not fused (their names must remain valid).
pub fn apply_fusestatic(mjcf: &mut MjcfModel) {
    // Collect names of bodies referenced by constraints, sensors, actuators, skins
    let mut protected: HashSet<String> = HashSet::new();

    // Equality constraints
    for c in &mjcf.equality.connects {
        protected.insert(c.body1.clone());
        if let Some(ref b2) = c.body2 {
            protected.insert(b2.clone());
        }
    }
    for w in &mjcf.equality.welds {
        protected.insert(w.body1.clone());
        if let Some(ref b2) = w.body2 {
            protected.insert(b2.clone());
        }
    }

    // Sensors that reference bodies: subtree sensors always reference body names,
    // frame sensors can resolve to bodies. Protect objnames for both.
    for sensor in &mjcf.sensors {
        use crate::types::MjcfSensorType;
        match sensor.sensor_type {
            MjcfSensorType::Subtreecom
            | MjcfSensorType::Subtreelinvel
            | MjcfSensorType::Subtreeangmom
            | MjcfSensorType::Framepos
            | MjcfSensorType::Framequat
            | MjcfSensorType::Framexaxis
            | MjcfSensorType::Frameyaxis
            | MjcfSensorType::Framezaxis
            | MjcfSensorType::Framelinvel
            | MjcfSensorType::Frameangvel
            | MjcfSensorType::Framelinacc
            | MjcfSensorType::Frameangacc => {
                if let Some(ref name) = sensor.objname {
                    protected.insert(name.clone());
                }
            }
            _ => {}
        }
    }

    // Actuators with body transmission (adhesion actuators)
    for actuator in &mjcf.actuators {
        if let Some(ref body_name) = actuator.body {
            protected.insert(body_name.clone());
        }
    }

    // Skin bones reference body names
    for skin in &mjcf.skins {
        for bone in &skin.bones {
            protected.insert(bone.body.clone());
        }
    }

    // Recursively fuse static children in the worldbody tree.
    fuse_static_body(&mut mjcf.worldbody, &protected, &mjcf.compiler);
}

/// Recursively fuse jointless, unprotected children into their parent body.
/// Operates on a parent body: scans its children, and for any that are static
/// (no joints, not protected), transfers their geoms, sites, and grandchildren
/// to the parent, adjusting positions and orientations by the fused body's frame.
fn fuse_static_body(parent: &mut MjcfBody, protected: &HashSet<String>, compiler: &MjcfCompiler) {
    // Depth-first: fuse within each child first
    for child in &mut parent.children {
        fuse_static_body(child, protected, compiler);
    }

    // Now fuse static children into this parent
    let mut i = 0;
    while i < parent.children.len() {
        let is_static =
            parent.children[i].joints.is_empty() && !protected.contains(&parent.children[i].name);

        if is_static {
            let mut removed = parent.children.remove(i);
            let body_pos = removed.pos;
            let body_quat = resolve_orientation(
                removed.quat,
                removed.euler,
                removed.axisangle,
                None,
                None,
                compiler,
            );
            let has_rotation = body_quat.angle() > 1e-10;

            // Transform geoms into parent frame
            for g in &mut removed.geoms {
                let gpos = g.pos.unwrap_or_else(nalgebra::Vector3::zeros);
                if has_rotation {
                    g.pos = Some(body_quat * gpos + body_pos);
                    let geom_quat = resolve_orientation(
                        g.quat.unwrap_or(nalgebra::Vector4::new(1.0, 0.0, 0.0, 0.0)),
                        g.euler,
                        g.axisangle,
                        g.xyaxes,
                        g.zaxis,
                        compiler,
                    );
                    let composed = body_quat * geom_quat;
                    g.quat = Some(quat_to_wxyz(&composed));
                    // Normalize to quat-only after composition
                    g.euler = None;
                    g.axisangle = None;
                    g.xyaxes = None;
                    g.zaxis = None;
                } else {
                    g.pos = Some(gpos + body_pos);
                }
            }

            // Transform sites into parent frame
            for s in &mut removed.sites {
                let s_pos = s.pos.unwrap_or_else(nalgebra::Vector3::zeros);
                if has_rotation {
                    s.pos = Some(body_quat * s_pos + body_pos);
                    let site_quat = resolve_orientation(
                        s.quat.unwrap_or(nalgebra::Vector4::new(1.0, 0.0, 0.0, 0.0)),
                        s.euler,
                        s.axisangle,
                        s.xyaxes,
                        s.zaxis,
                        compiler,
                    );
                    let composed = body_quat * site_quat;
                    s.quat = Some(quat_to_wxyz(&composed));
                    // Normalize to quat-only after composition
                    s.euler = None;
                    s.axisangle = None;
                    s.xyaxes = None;
                    s.zaxis = None;
                } else {
                    s.pos = Some(s_pos + body_pos);
                }
            }

            // Transform child body frames into parent frame
            for c in &mut removed.children {
                if has_rotation {
                    c.pos = body_quat * c.pos + body_pos;
                    let child_quat =
                        resolve_orientation(c.quat, c.euler, c.axisangle, None, None, compiler);
                    let composed = body_quat * child_quat;
                    c.quat = quat_to_wxyz(&composed);
                    c.euler = None;
                    c.axisangle = None;
                } else {
                    c.pos += body_pos;
                }
            }

            // Transfer geoms and sites to parent
            parent.geoms.extend(removed.geoms);
            parent.sites.extend(removed.sites);

            // Splice grandchildren at position i
            for (j, gc) in removed.children.into_iter().enumerate() {
                parent.children.insert(i + j, gc);
            }
            // Don't increment i — re-check from the same position
        } else {
            i += 1;
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::apply_fusestatic;
    use crate::builder::load_model;
    use nalgebra::Vector3;

    // -- discardvisual / fusestatic tests --

    #[test]
    fn test_discardvisual_removes_visual_geoms() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" discardvisual="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom name="collision" type="sphere" size="0.1" contype="1" conaffinity="1"/>
                        <geom name="visual" type="sphere" size="0.12" contype="0" conaffinity="0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Only collision geom should remain
        assert_eq!(model.ngeom, 1, "visual geom should be discarded");
    }

    #[test]
    fn test_discardvisual_false_keeps_all_geoms() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" discardvisual="false"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom name="collision" type="sphere" size="0.1" contype="1" conaffinity="1"/>
                        <geom name="visual" type="sphere" size="0.12" contype="0" conaffinity="0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert_eq!(
            model.ngeom, 2,
            "all geoms should remain when discardvisual=false"
        );
    }

    #[test]
    fn test_fusestatic_merges_jointless_body() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="static_child" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // static_child (no joints) should be fused into parent
        // parent + world = 2 bodies (static_child merged)
        assert_eq!(
            model.nbody, 2,
            "static child should be fused: nbody={}",
            model.nbody
        );
        // Geoms: parent sphere + fused box = 2
        assert_eq!(model.ngeom, 2, "both geoms should remain after fusion");
    }

    #[test]
    fn test_fusestatic_preserves_jointed_body() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="true"/>
                <worldbody>
                    <body name="base" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="link" pos="0 0 0.5">
                            <joint name="j2" type="hinge" axis="0 1 0"/>
                            <geom type="sphere" size="0.08"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Both bodies have joints — no fusion
        assert_eq!(
            model.nbody, 3,
            "jointed bodies should not be fused: nbody={}",
            model.nbody
        );
    }

    #[test]
    fn test_fusestatic_false_preserves_all_bodies() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="false"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="static_child" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert_eq!(model.nbody, 3, "fusestatic=false should keep all bodies");
    }

    #[test]
    fn test_fusestatic_orientation_handling() {
        // A static body rotated 90deg around Z should rotate its child geom position and orientation.
        let model = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="parent_geom" type="sphere" size="0.1" mass="1"/>
                        <body name="rotated_static" pos="1 0 0" euler="0 0 90">
                            <geom name="child_geom" type="sphere" size="0.05" pos="0.5 0 0" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // The static body should be fused into parent: 2 bodies (world + parent)
        assert_eq!(model.nbody, 2, "rotated static body should be fused");
        // Parent should have 2 geoms (its own + child's)
        assert_eq!(
            model.body_geom_num[1], 2,
            "parent should have 2 geoms after fusion"
        );

        // Check the fused geom position: should be [1, 0.5, 0] after rotation
        let child_geom_idx = 1; // second geom on this body
        let fused_pos = model.geom_pos[child_geom_idx];
        assert!(
            (fused_pos.x - 1.0).abs() < 1e-10,
            "fused geom x should be 1.0, got {}",
            fused_pos.x
        );
        assert!(
            (fused_pos.y - 0.5).abs() < 1e-10,
            "fused geom y should be 0.5 (rotated), got {}",
            fused_pos.y
        );
        assert!(
            fused_pos.z.abs() < 1e-10,
            "fused geom z should be 0, got {}",
            fused_pos.z
        );
    }

    #[test]
    fn test_fusestatic_no_rotation_preserves_geom_euler() {
        let with_fuse = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="g0" type="sphere" size="0.1" mass="1"/>
                        <body name="static_child" pos="0 0 1">
                            <geom name="g1" type="sphere" size="0.05" pos="0 0 0" euler="0 0 90" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("fuse model should load");

        let without_fuse = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="false"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="g0" type="sphere" size="0.1" mass="1"/>
                        <body name="static_child" pos="0 0 1">
                            <geom name="g1" type="sphere" size="0.05" pos="0 0 0" euler="0 0 90" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("no-fuse model should load");

        assert_eq!(with_fuse.nbody, 2, "fused model should have 2 bodies");
        assert_eq!(without_fuse.nbody, 3, "unfused model should have 3 bodies");

        assert_eq!(with_fuse.ngeom, 2);
        assert_eq!(without_fuse.ngeom, 2);

        // Geom quaternions should match
        let fused_quat = with_fuse.geom_quat[1];
        let unfused_quat = without_fuse.geom_quat[1];
        let diff = (fused_quat.into_inner() - unfused_quat.into_inner()).norm();
        assert!(
            diff < 1e-10,
            "fused geom quat should match unfused: fused={fused_quat:?}, unfused={unfused_quat:?}, diff={diff}"
        );

        // Geom positions should account for body offset
        let fused_pos = with_fuse.geom_pos[1];
        assert!(
            (fused_pos.z - 1.0).abs() < 1e-10,
            "fused geom z should be 1.0 (body offset), got {}",
            fused_pos.z
        );
    }

    #[test]
    fn test_fusestatic_protects_sensor_referenced_body() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1"/>
                        <body name="sensor_target" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
                <sensor>
                    <subtreecom name="com_sensor" body="sensor_target"/>
                </sensor>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert_eq!(
            model.nbody, 3,
            "sensor-referenced body should be protected from fusion"
        );
    }

    #[test]
    fn test_fusestatic_protects_actuator_referenced_body() {
        let mut mjcf = crate::parse_mjcf_str(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="actuator_target" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05"/>
                        </body>
                        <body name="unprotected" pos="0 0 1">
                            <geom type="sphere" size="0.05"/>
                        </body>
                    </body>
                </worldbody>
                <actuator>
                    <adhesion name="stick" body="actuator_target" gain="1"/>
                </actuator>
            </mujoco>
            "#,
        )
        .expect("should parse");

        apply_fusestatic(&mut mjcf);

        // actuator_target should still exist, unprotected should be fused
        let parent = &mjcf.worldbody.children[0];
        let child_names: Vec<&str> = parent.children.iter().map(|c| c.name.as_str()).collect();
        assert!(
            child_names.contains(&"actuator_target"),
            "actuator-referenced body should be protected: {child_names:?}"
        );
        assert!(
            !child_names.contains(&"unprotected"),
            "unprotected body should be fused away: {child_names:?}"
        );
    }

    #[test]
    fn test_discardvisual_protects_sensor_referenced_geom() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" discardvisual="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="collision_geom" type="sphere" size="0.1" mass="1"/>
                        <geom name="visual_ref" type="sphere" size="0.2" contype="0" conaffinity="0"/>
                    </body>
                </worldbody>
                <sensor>
                    <framepos name="vis_sensor" objname="visual_ref"/>
                </sensor>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert_eq!(
            model.ngeom, 2,
            "sensor-referenced visual geom should be protected from discard"
        );
    }

    // -- Additional coverage: fusestatic + orientation fields, discardvisual + frame --

    #[test]
    fn test_fusestatic_geom_with_xyaxes() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent">
                        <joint type="hinge"/>
                        <body name="static_child" pos="1 0 0" euler="0 0 90">
                            <geom type="sphere" size="0.1" xyaxes="0 1 0 -1 0 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-4,
            "expected (1,0,0), got {pos:?}"
        );
    }

    #[test]
    fn test_fusestatic_site_with_axisangle() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent">
                        <joint type="hinge"/>
                        <body name="static_child" pos="0 0 1">
                            <site name="s" pos="0 0 0" axisangle="0 0 1 90"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.site_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-4,
            "expected (0,0,1), got {pos:?}"
        );
    }

    #[test]
    fn test_discardvisual_geom_inside_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler discardvisual="true"/>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <geom type="sphere" size="0.1" contype="0" conaffinity="0"/>
                        </frame>
                        <geom type="sphere" size="0.1" pos="0 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Only the non-visual geom should survive
        assert_eq!(
            model.geom_pos.len(),
            1,
            "discardvisual should remove visual geom from frame"
        );
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 0.0, 0.0)).norm() < 1e-4,
            "surviving geom should be at origin"
        );
    }
}
