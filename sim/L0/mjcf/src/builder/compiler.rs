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
