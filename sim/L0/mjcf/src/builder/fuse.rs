//! `fusestatic`, as MuJoCo 3.5.0's `mjCModel::FuseStatic` makes it
//! (`user_model.cc:4237-4381`): once every body is built and checked, each
//! body with its geoms, sites and inertia (`mjCBody::Compile`, `:4931-4934`),
//! and before the flexes, excludes, equalities, tendons, actuators, sensors
//! and contact pairs are made (`:4947-4962`), so that they name the fused
//! bodies.
//!
//! Body by body in the tree's preorder, a body with no joint that is not a
//! mocap body is fused into its parent, unless resolving the other elements'
//! references keeps it ([`ModelBuilder::fuse_static`]): its mass and inertia
//! added to the parent's, unless the parent is the world or the body's mass
//! is below mjMINVAL (`AccumulateInertia`); its children put in its place
//! among the parent's; its geoms and sites appended to the parent's; each
//! child, geom and site composed with the body's pose (`changeframe`,
//! `:4160-4168`, in `mjuu_frameaccum`'s arithmetic). Then every body, geom
//! and site is numbered again in the tree's preorder (`FuseReindex`,
//! `:4173-4189`).

use std::collections::{HashMap, HashSet};

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use sim_core::{GeomType, MjJointType};

use super::ModelBuilder;
use super::eig3::full_inertia;
use super::frame::frame_accum_child;
use super::mass::{MJ_MINVAL, global_inertia, off_center};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{MjcfBody, MjcfModel, MjcfSensor, MjcfSensorType, SpatialPathElement};

/// The names `FuseStatic` resolves before it fuses a named body
/// (`user_model.cc:4243-4254`): a body whose name one of them needs, or one
/// the resolution throws on, is kept.
pub struct FuseReferences {
    /// Each body a camera, light, skin, exclude, equality, actuator, sensor
    /// or tuple names by its name (`mjOBJ_BODY`). An `xbody` reference keeps
    /// none: MuJoCo makes no map of `xbody` names (`ProcessLists`,
    /// `user_model.cc:4408`), so it finds the body by name in the list,
    /// where the body being tried still is, and fuses it.
    bodies: Vec<String>,
    /// The site of each force and torque sensor: the body holding it is
    /// kept ("cannot fuse a body used by a force/torque sensor",
    /// `:4219-4232`).
    force_torque_sites: Vec<String>,
    /// The geom each spatial tendon wraps: MuJoCo's tendon resolution throws
    /// where the geom its name finds is neither a sphere nor a cylinder
    /// (`mjCWrap::ResolveReferences`, `user_objects.cc:6536-6541`).
    wrap_geoms: Vec<String>,
}

impl FuseReferences {
    /// The references of `mjcf`'s elements that name bodies, its force and
    /// torque sensors' sites and its tendons' wrapped geoms.
    pub fn of(mjcf: &MjcfModel) -> Self {
        let mut bodies = Vec::new();
        for connect in &mjcf.equality.connects {
            bodies.extend(connect.body1.iter().chain(&connect.body2).cloned());
        }
        for weld in &mjcf.equality.welds {
            bodies.extend(weld.body1.iter().chain(&weld.body2).cloned());
        }
        for exclude in &mjcf.contact.excludes {
            bodies.push(exclude.body1.clone());
            bodies.push(exclude.body2.clone());
        }
        bodies.extend(mjcf.actuators.iter().filter_map(|a| a.body.clone()));
        for skin in &mjcf.skins {
            bodies.extend(skin.bones.iter().map(|bone| bone.body.clone()));
        }
        // The cameras' and lights' targets, wherever they are written (a
        // frame's are its body's once frames are expanded), and the tree's
        // body and site names.
        let (mut body_names, mut site_names) = (HashSet::new(), HashSet::new());
        let mut stack = vec![&mjcf.worldbody];
        while let Some(body) = stack.pop() {
            bodies.extend(
                body.camera_targets
                    .iter()
                    .chain(&body.light_targets)
                    .cloned(),
            );
            body_names.insert(body.name.as_str());
            site_names.extend(body.sites.iter().map(|site| site.name.as_str()));
            stack.extend(&body.children);
        }
        for tuple in &mjcf.tuples {
            bodies.extend(
                tuple
                    .elements
                    .iter()
                    .filter(|element| element.objtype == "body")
                    .map(|element| element.objname.clone()),
            );
        }
        let mut force_torque_sites = Vec::new();
        for sensor in &mjcf.sensors {
            if matches!(
                sensor.sensor_type,
                MjcfSensorType::Force | MjcfSensorType::Torque
            ) {
                force_torque_sites.extend(sensor.objname.iter().cloned());
            }
            bodies.extend(sensor_bodies(sensor, |name| {
                body_names.contains(name) && !site_names.contains(name)
            }));
        }
        let wrap_geoms = mjcf
            .tendons
            .iter()
            .flat_map(|tendon| &tendon.path_elements)
            .filter_map(|element| match element {
                SpatialPathElement::Geom { geom, .. } => Some(geom.clone()),
                _ => None,
            })
            .collect();
        Self {
            bodies,
            force_torque_sites,
            wrap_geoms,
        }
    }
}

/// The bodies a sensor names by their name: a subtree sensor's object; a
/// frame or user sensor's object and a frame sensor's reference of type
/// `body`; a geom-distance sensor's bodies. A frame sensor with no
/// `objtype`, which MuJoCo refuses and this crate reads by its name, a site
/// first (D-FRAME-SENSOR-OBJTYPE), names a body where `names_a_body`.
fn sensor_bodies(sensor: &MjcfSensor, names_a_body: impl Fn(&str) -> bool) -> Vec<String> {
    let is_body = |kind: Option<&str>| kind == Some("body");
    let mut names = Vec::new();
    match sensor.sensor_type {
        MjcfSensorType::Subtreecom
        | MjcfSensorType::Subtreelinvel
        | MjcfSensorType::Subtreeangmom => {
            names.extend(sensor.objname.iter().cloned());
        }
        MjcfSensorType::Framepos
        | MjcfSensorType::Framequat
        | MjcfSensorType::Framexaxis
        | MjcfSensorType::Frameyaxis
        | MjcfSensorType::Framezaxis
        | MjcfSensorType::Framelinvel
        | MjcfSensorType::Frameangvel
        | MjcfSensorType::Framelinacc
        | MjcfSensorType::Frameangacc => {
            let objtype = sensor.objtype.as_deref();
            let unwritten =
                objtype.is_none() && sensor.objname.as_deref().is_some_and(&names_a_body);
            if unwritten || is_body(objtype) {
                names.extend(sensor.objname.iter().cloned());
            }
            if is_body(sensor.reftype.as_deref()) {
                names.extend(sensor.refname.iter().cloned());
            }
        }
        MjcfSensorType::User => {
            if is_body(sensor.objtype.as_deref()) {
                names.extend(sensor.objname.iter().cloned());
            }
        }
        MjcfSensorType::Distance | MjcfSensorType::Normal | MjcfSensorType::Fromto => {
            names.extend(sensor.body1.iter().chain(&sensor.body2).cloned());
        }
        _ => {}
    }
    names
}

/// MuJoCo 3.5.0's refusal of a camera's or a light's target that names no
/// body ("unknown target body in camera", "… in light",
/// `user_objects.cc:4073-4082`, `:4240-4257`): the first in the tree's
/// preorder, a body's cameras before its lights. `bodies` is the tree's body
/// name map. MuJoCo makes it in the turn of the body holding it (`:4111`,
/// `:4281`); this crate makes it after every body's own checks, so of it and
/// another fault this crate names the other (ledger L353).
pub fn target_fault(world: &MjcfBody, bodies: &HashMap<String, usize>) -> Option<MjcfError> {
    let mut stack = vec![world];
    while let Some(body) = stack.pop() {
        for (targets, of) in [
            (&body.camera_targets, "camera"),
            (&body.light_targets, "light"),
        ] {
            if targets.iter().any(|t| !bodies.contains_key(t)) {
                return Some(
                    MjcfError::invalid_model(format!("unknown target body in {of}"))
                        .at(Location::named(ElementKind::Body, &body.name)),
                );
            }
        }
        stack.extend(body.children.iter().rev());
    }
    None
}

/// A body as `FuseStatic` moves it: its children, geoms and sites in its
/// lists' order.
#[derive(Clone, Default)]
struct Node {
    parent: usize,
    children: Vec<usize>,
    geoms: Vec<usize>,
    sites: Vec<usize>,
}

impl ModelBuilder {
    /// Fuse the static bodies into their parents, as MuJoCo 3.5.0's
    /// `FuseStatic` does once every body is built and checked.
    ///
    /// Which bodies fuse is MuJoCo's loop over its body list: a named body
    /// is kept where one of `references` names it or where resolving them
    /// throws. MuJoCo does not number its names again until the loop ends,
    /// so after k fusions a later body's name maps to an index k too high,
    /// and a reference to one of the last k bodies throws ("object not
    /// found", `findobject`, `user_model.cc:1531-1552`); its geom map goes
    /// stale the same way as fusion moves geoms, so a tendon's wrapped geom
    /// can find a geom that is neither a sphere nor a cylinder, and throw
    /// (`user_objects.cc:6536-6541`): either keeps the body being tried.
    /// Both are done as MuJoCo does them. A force or torque sensor's site is
    /// looked up by its holder: a fusion moves sites only from bodies MuJoCo
    /// has already tried, so its stale site map finds a site on the body
    /// being tried exactly where the sensor's site is; a site that does not
    /// exist throws, as MuJoCo's sensor resolution does.
    ///
    /// A fused body's inertia is added in its parent's frame as the body's
    /// pose composed with its inertial frame. MuJoCo composes them the
    /// other way and turns the moments by the inertial frame alone
    /// (`AccumulateInertia`, `user_objects.cc:2249`, `:2266`), so that a
    /// fused model's dynamics are not the unfused one's; this crate keeps
    /// them the same (registry row D-FUSESTATIC-INERTIA). Where the fused
    /// body's quaternion and its inertial frame's are both the identity
    /// the two agree.
    ///
    /// # Errors
    ///
    /// A sum of inertias whose principal axes MuJoCo's `mjuu_fullInertia`
    /// refuses ("error '…' in fusing static body inertias",
    /// `user_objects.cc:2313-2316`).
    pub(crate) fn fuse_static(&mut self, references: &FuseReferences) -> Result<()> {
        let nbody = self.body_parent.len();
        let mut nodes = vec![Node::default(); nbody];
        for b in 1..nbody {
            nodes[b].parent = self.body_parent[b];
            let parent = self.body_parent[b];
            nodes[parent].children.push(b);
        }
        for (b, node) in nodes.iter_mut().enumerate() {
            let adr = self.body_geom_adr[b];
            node.geoms = (adr..adr + self.body_geom_num[b]).collect();
        }
        for (s, &b) in self.site_body.iter().enumerate() {
            nodes[b].sites.push(s);
        }
        // MuJoCo's body list and its name maps, each name at its index in
        // its list when the loop starts.
        let mut order: Vec<usize> = (0..nbody).collect();
        let mut body_ids: HashMap<String, usize> = self
            .body_name
            .iter()
            .enumerate()
            .filter_map(|(b, name)| Some((name.clone()?, b)))
            .collect();
        let site_ids = index_of(&self.site_name);
        let geom_ids = index_of(&self.geom_name);
        let mut site_holder = self.site_body.clone();
        // MuJoCo's geom list as each fusion leaves it (`FuseReindex`).
        let mut geom_list: Vec<usize> = (0..self.geom_type.len()).collect();

        let mut i = 1;
        while i < order.len() {
            let b = order[i];
            let name = self.body_name[b].clone();
            if let Some(name) = &name {
                body_ids.remove(name);
                let len = order.len();
                let neither = |g: usize| {
                    !matches!(
                        self.geom_type[geom_list[g]],
                        GeomType::Sphere | GeomType::Cylinder
                    )
                };
                let throws = references
                    .bodies
                    .iter()
                    .any(|r| body_ids.get(r).is_none_or(|&id| id > len - 1))
                    || references
                        .force_torque_sites
                        .iter()
                        .any(|s| site_ids.get(s).is_none_or(|&id| site_holder[id] == b))
                    || references
                        .wrap_geoms
                        .iter()
                        .any(|g| geom_ids.get(g).is_none_or(|&id| neither(id)));
                body_ids.insert(name.clone(), i);
                if throws {
                    i += 1;
                    continue;
                }
            }
            if self.body_jnt_num[b] > 0 || self.body_mocapid[b].is_some() {
                i += 1;
                continue;
            }

            let parent = nodes[b].parent;
            if parent != 0 && self.body_mass[b] >= MJ_MINVAL {
                self.accumulate_inertia(parent, b)?;
            }
            let (pos, quat) = (self.body_pos[b], self.body_quat[b]);
            let children = std::mem::take(&mut nodes[b].children);
            for &c in &children {
                frame_accum_child(&pos, &quat, &mut self.body_pos[c], &mut self.body_quat[c]);
                nodes[c].parent = parent;
            }
            // The children in the body's place among its parent's.
            let siblings = &mut nodes[parent].children;
            let at = siblings.iter().position(|&c| c == b);
            debug_assert!(at.is_some(), "body {b} is not among its parent's children");
            if let Some(at) = at {
                siblings.splice(at..=at, children);
            }
            let geoms = std::mem::take(&mut nodes[b].geoms);
            for &g in &geoms {
                frame_accum_child(&pos, &quat, &mut self.geom_pos[g], &mut self.geom_quat[g]);
            }
            nodes[parent].geoms.extend(geoms);
            let sites = std::mem::take(&mut nodes[b].sites);
            for &s in &sites {
                frame_accum_child(&pos, &quat, &mut self.site_pos[s], &mut self.site_quat[s]);
                site_holder[s] = parent;
            }
            nodes[parent].sites.extend(sites);
            order.remove(i);
            if let Some(name) = &name {
                body_ids.remove(name);
            }
            geom_list = preorder(&nodes)
                .into_iter()
                .flat_map(|b| nodes[b].geoms.clone())
                .collect();
        }

        if order.len() < nbody {
            self.renumber_fused(&nodes, &order);
        }
        Ok(())
    }

    /// `AccumulateInertia` (`user_objects.cc:2239-2318`) of `other` into
    /// `result`, in its arithmetic, with `other`'s inertial frame composed
    /// after its pose and its moments turned by that composition.
    fn accumulate_inertia(&mut self, result: usize, other: usize) -> Result<()> {
        let mut other_ipos = self.body_ipos[other];
        let mut other_iquat = self.body_iquat[other];
        frame_accum_child(
            &self.body_pos[other],
            &self.body_quat[other],
            &mut other_ipos,
            &mut other_iquat,
        );
        let mass = [self.body_mass[result], self.body_mass[other]];
        let inertia = [self.body_inertia[result], self.body_inertia[other]];
        let ipos = [self.body_ipos[result], other_ipos];
        let iquat = [wxyz(&self.body_iquat[result]), wxyz(&other_iquat)];

        let mut total = 0.0;
        let mut com = [0.0; 3];
        for j in 0..2 {
            total += mass[j];
            com[0] += mass[j] * ipos[j].x;
            com[1] += mass[j] * ipos[j].y;
            com[2] += mass[j] * ipos[j].z;
        }
        self.body_mass[result] = total;
        if total < MJ_MINVAL {
            self.body_mass[result] = 0.0;
            self.body_inertia[result] = Vector3::zeros();
            self.body_ipos[result] = Vector3::zeros();
            self.body_iquat[result] = UnitQuaternion::identity();
            return Ok(());
        }
        let com = [com[0] / total, com[1] / total, com[2] / total];
        self.body_ipos[result] = Vector3::from(com);
        let mut toti = [0.0; 6];
        for j in 0..2 {
            let dpos = [ipos[j].x - com[0], ipos[j].y - com[1], ipos[j].z - com[2]];
            let inert_a = global_inertia(&[inertia[j].x, inertia[j].y, inertia[j].z], &iquat[j]);
            let inert_b = off_center(mass[j], &dpos);
            for k in 0..6 {
                toti[k] += inert_a[k] + inert_b[k];
            }
        }
        // `mjuu_fullInertia` writes nothing where the inertia's first entry
        // is not a number (`user_util.cc:786-788`).
        if toti[0].is_nan() {
            return Ok(());
        }
        let (moments, [w, x, y, z]) = full_inertia(&toti).map_err(|reason| {
            MjcfError::invalid_model(format!("error '{reason}' in fusing static body inertias"))
        })?;
        self.body_inertia[result] = Vector3::from(moments);
        self.body_iquat[result] = UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z));
        Ok(())
    }

    /// Number the bodies, geoms and sites of the fused tree again, as
    /// `FuseReindex` does: the bodies in `order` (MuJoCo's body list, the
    /// fused tree's preorder), each body's geoms and sites in its lists'
    /// order; each joint's, dof's, geom's and site's body and each body's
    /// parent renumbered; a free joint's `qpos0` its body's pose, as
    /// MuJoCo copies it once the tree is fused (`user_model.cc:2803-2806`).
    fn renumber_fused(&mut self, nodes: &[Node], order: &[usize]) {
        // MuJoCo numbers the bodies by its list and the geoms and sites by
        // walking the tree (`FuseReindex`); a fused body's children take its
        // place among its parent's, so the two orders are one.
        debug_assert_eq!(
            preorder(nodes),
            order,
            "the fused tree's preorder is the body list"
        );
        let mut body_id = vec![usize::MAX; nodes.len()];
        for (new, &old) in order.iter().enumerate() {
            body_id[old] = new;
        }
        let geom_order: Vec<usize> = order.iter().flat_map(|&b| nodes[b].geoms.clone()).collect();
        let site_order: Vec<usize> = order.iter().flat_map(|&b| nodes[b].sites.clone()).collect();

        let mut adr = 0;
        let geom_ranges: Vec<(usize, usize)> = order
            .iter()
            .map(|&b| {
                let range = (adr, nodes[b].geoms.len());
                adr += nodes[b].geoms.len();
                range
            })
            .collect();
        let parents: Vec<usize> = order.iter().map(|&b| body_id[nodes[b].parent]).collect();
        let geom_body: Vec<usize> = order
            .iter()
            .enumerate()
            .flat_map(|(new, &b)| std::iter::repeat_n(new, nodes[b].geoms.len()))
            .collect();
        let site_body: Vec<usize> = order
            .iter()
            .enumerate()
            .flat_map(|(new, &b)| std::iter::repeat_n(new, nodes[b].sites.len()))
            .collect();

        self.select_bodies(order);
        self.body_parent = parents;
        self.body_parent[0] = 0;
        for (b, &range) in geom_ranges.iter().enumerate() {
            self.body_rootid[b] = if b == 0 || self.body_parent[b] == 0 {
                b
            } else {
                self.body_rootid[self.body_parent[b]]
            };
            (self.body_geom_adr[b], self.body_geom_num[b]) = range;
        }
        for b in &mut self.jnt_body {
            *b = body_id[*b];
        }
        for b in &mut self.dof_body {
            *b = body_id[*b];
        }

        self.select_geoms(&geom_order);
        self.geom_body = geom_body;
        self.select_sites(&site_order);
        self.site_body = site_body;

        for j in 0..self.jnt_type.len() {
            if self.jnt_type[j] == MjJointType::Free {
                let b = self.jnt_body[j];
                let (pos, q) = (self.body_pos[b], self.body_quat[b]);
                let pose = [pos.x, pos.y, pos.z, q.w, q.i, q.j, q.k];
                let adr = self.jnt_qpos_adr[j];
                self.qpos0_values[adr..adr + 7].copy_from_slice(&pose);
                self.qpos_spring_values[adr..adr + 7].copy_from_slice(&pose);
            }
        }
    }
}

/// Each non-empty name of `names` to its index.
fn index_of(names: &[Option<String>]) -> HashMap<String, usize> {
    names
        .iter()
        .enumerate()
        .filter_map(|(i, name)| Some((name.clone().filter(|n| !n.is_empty())?, i)))
        .collect()
}

/// The bodies of the tree `nodes` holds, from the world, in preorder.
fn preorder(nodes: &[Node]) -> Vec<usize> {
    let mut out = Vec::new();
    let mut stack = vec![0];
    while let Some(b) = stack.pop() {
        out.push(b);
        stack.extend(nodes[b].children.iter().rev());
    }
    out
}

/// `q`'s components, (w, x, y, z).
fn wxyz(q: &UnitQuaternion<f64>) -> [f64; 4] {
    [q.w, q.i, q.j, q.k]
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;
    use nalgebra::Vector3;

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
        let model = load_model(
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
                    <adhesion name="stick" body="actuator_target" gain="1" ctrlrange="0 1"/>
                </actuator>
            </mujoco>
            "#,
        )
        .expect("should load");

        // actuator_target should still exist, unprotected should be fused
        let child_names: Vec<&str> = model
            .body_name
            .iter()
            .filter_map(|n| n.as_deref())
            .collect();
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
                        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
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
}
