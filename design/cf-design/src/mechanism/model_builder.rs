//! Direct Model construction from a validated mechanism.
//!
//! Converts a [`super::Mechanism`] into a sim-core [`Model`] without going
//! through the MJCF XML round-trip. This enables:
//!
//! - **SDF collision geoms**: each part's implicit surface is discretized into an
//!   [`SdfGrid`] for O(1) collision queries with exact distance semantics.
//! - **High-res visual meshes**: the same [`Solid`] produces a separate triangle
//!   mesh for Bevy rendering, ensuring visual/physics alignment.
//! - **Standard collision filtering**: parent-child bodies are filtered
//!   (default). SDF collision works for non-parent-child geom pairs. Parent-child
//!   SDF collision (socket/condyle) requires multi-contact `sdf_sdf_contact` —
//!   future work.
//!
//! # Architecture
//!
//! ```text
//! Solid (exact SDF)
//!   ├──→ SdfGrid (collision)  ──→ Model.sdf_data / GeomType::Sdf
//!   └──→ mesh (visual)        ──→ Model.mesh_data / GeomType::Mesh
//! ```
//!
//! Both representations derive from the identical mathematical object.

// Model construction inherently pushes many arrays — allow long functions and panics
// for builder assertions, and casts for grid sizing.
#![allow(
    clippy::too_many_lines,
    clippy::panic,
    clippy::expect_used,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::uninlined_format_args,
    clippy::redundant_closure,
    clippy::unwrap_used
)]

use std::collections::HashMap;
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_core::{
    ActuatorDynamics, ActuatorTransmission, BiasType, GainType, GeomType, MjJointType, Model,
    SdfGrid, TendonType, WrapType,
};

use super::actuator::ActuatorKind;
use super::builder::Mechanism;
use super::joint::{JointDef, JointKind};
use super::part::Part;
use super::tendon::TendonDef;

// ── Public entry point ──────────────────────────────────────────────────

impl Mechanism {
    /// Build a sim-core [`Model`] directly from this mechanism.
    ///
    /// Each part produces TWO geoms:
    /// - **SDF geom** (collision, group 2): `GeomType::Sdf` from `SdfGrid::from_fn`
    /// - **Mesh geom** (visual, group 0): `GeomType::Mesh` from `Solid::mesh`
    ///
    /// # Arguments
    ///
    /// - `sdf_resolution`: SDF grid cell size in mm. Smaller = finer collision.
    ///   Must resolve the thinnest wall (e.g., 0.5 for a 0.6mm socket wall).
    /// - `visual_resolution`: Marching cubes cell size for the visual mesh.
    ///   Smaller = smoother rendering (e.g., 0.3 for high quality).
    ///
    /// # Panics
    ///
    /// Panics if either resolution is not positive and finite, or if a part
    /// has no finite bounds (e.g., an infinite plane).
    #[must_use]
    pub fn to_model(&self, sdf_resolution: f64, visual_resolution: f64) -> Model {
        generate(self, sdf_resolution, visual_resolution)
    }
}

// ── Model construction ──────────────────────────────────────────────────

/// Build a sim-core Model from a validated mechanism.
fn generate(mechanism: &Mechanism, sdf_resolution: f64, visual_resolution: f64) -> Model {
    assert!(
        sdf_resolution > 0.0 && sdf_resolution.is_finite(),
        "SDF resolution must be positive and finite, got {sdf_resolution}"
    );
    assert!(
        visual_resolution > 0.0 && visual_resolution.is_finite(),
        "visual resolution must be positive and finite, got {visual_resolution}"
    );

    let parts = mechanism.parts();
    let joints = mechanism.joints();
    let tendons = mechanism.tendons();
    let actuators = mechanism.actuators();

    // ── Build kinematic tree topology ───────────────────────────────
    let part_by_name: HashMap<&str, (usize, &Part)> = parts
        .iter()
        .enumerate()
        .map(|(i, p)| (p.name(), (i, p)))
        .collect();

    // child_part → (parent_part_name, first_joint)
    let mut child_parent: HashMap<&str, &str> = HashMap::new();
    let mut joints_on: HashMap<&str, Vec<&JointDef>> = HashMap::new();

    for joint in joints {
        child_parent
            .entry(joint.child())
            .or_insert_with(|| joint.parent());
        joints_on.entry(joint.child()).or_default().push(joint);
    }

    // parent → [children] in part declaration order
    let mut children_of: HashMap<&str, Vec<&str>> = HashMap::new();
    for part in parts {
        if let Some(&parent) = child_parent.get(part.name()) {
            children_of.entry(parent).or_default().push(part.name());
        }
    }

    // Roots: parts that are never a child
    let roots: Vec<&str> = parts
        .iter()
        .map(Part::name)
        .filter(|n| !child_parent.contains_key(n))
        .collect();

    // ── Assign body IDs (BFS from roots) ────────────────────────────
    // Body 0 = world. Parts start at body 1.
    let mut part_to_body: HashMap<&str, usize> = HashMap::new();
    let mut body_order: Vec<&str> = Vec::with_capacity(parts.len());
    let mut stack: Vec<&str> = roots.iter().copied().rev().collect();

    while let Some(name) = stack.pop() {
        let body_id = body_order.len() + 1; // +1 for world body
        part_to_body.insert(name, body_id);
        body_order.push(name);
        if let Some(kids) = children_of.get(name) {
            for kid in kids.iter().rev() {
                stack.push(kid);
            }
        }
    }

    let nbody = body_order.len() + 1; // +1 for world body

    // ── Generate SDF grids and visual meshes per part ────────────────
    let mut sdf_data: Vec<Arc<SdfGrid>> = Vec::new();
    let mut mesh_data: Vec<Arc<sim_core::TriangleMeshData>> = Vec::new();

    // part_name → (sdf_id, mesh_id, aabb)
    let mut part_assets: HashMap<&str, (usize, usize, cf_geometry::Aabb)> = HashMap::new();

    for part in parts {
        let solid = part.solid();
        let bounds = solid
            .bounds()
            .unwrap_or_else(|| panic!("part '{}' has no finite bounds", part.name()));

        // SDF grid: expand bounds by one cell to capture the surface
        let expanded = bounds.expanded(sdf_resolution * 2.0);
        let size = expanded.size();

        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let width = ((size.x / sdf_resolution).ceil() as usize).max(2);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let height = ((size.y / sdf_resolution).ceil() as usize).max(2);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let depth = ((size.z / sdf_resolution).ceil() as usize).max(2);

        let origin = expanded.min;
        let sdf = SdfGrid::from_fn(width, height, depth, sdf_resolution, origin, |p| {
            solid.evaluate(&p)
        });

        let sdf_id = sdf_data.len();
        sdf_data.push(Arc::new(sdf));

        // Visual mesh: Solid::mesh → IndexedMesh → TriangleMeshData
        let indexed = solid.mesh(visual_resolution);
        let tri =
            sim_core::TriangleMeshData::from_faces(indexed.vertices.clone(), indexed.faces.clone());
        let mesh_id = mesh_data.len();
        mesh_data.push(Arc::new(tri));

        part_assets.insert(part.name(), (sdf_id, mesh_id, bounds));
    }

    // ── Build Model starting from empty ─────────────────────────────
    let mut model = Model::empty();
    model.name = mechanism.name().to_string();

    // Parent-child collision filtering stays ON (MuJoCo default).
    // SDF-SDF parent-child collision (socket/condyle) needs multi-contact
    // support in sdf_sdf_contact before it can constrain concave geometry.

    // ── Bodies ───────────────────────────────────────────────────────
    // World body (index 0) is already in empty model. Add part bodies.
    for (order_idx, &part_name) in body_order.iter().enumerate() {
        let body_id = order_idx + 1;
        let (_, part) = part_by_name[part_name];

        // Parent body
        let parent_body = if let Some(&parent_name) = child_parent.get(part_name) {
            part_to_body[parent_name]
        } else {
            0 // root part → parent is world
        };

        // Body position relative to parent (from first joint anchor)
        let body_pos = joints_on
            .get(part_name)
            .and_then(|jl| jl.first())
            .map_or_else(Vector3::zeros, |j| j.anchor().coords);

        model.body_parent.push(parent_body);
        model.body_rootid.push(if parent_body == 0 {
            body_id
        } else {
            model.body_rootid[parent_body]
        });
        model.body_pos.push(body_pos);
        model.body_quat.push(UnitQuaternion::identity());
        model.body_name.push(Some(part_name.to_string()));

        // Mass properties (computed from implicit field)
        let mass_props = super::mass::mass_properties(part.solid(), part.material().density, 1.0);
        if let Some(mp) = mass_props {
            // Convert inertia from [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] to diagonal Vector3
            // sim-core stores diagonal inertia only (principal axes assumption)
            model.body_mass.push(mp.mass);
            model
                .body_inertia
                .push(Vector3::new(mp.inertia[0], mp.inertia[1], mp.inertia[2]));
            // Center of mass offset in body frame (mm — same units as geometry)
            model.body_ipos.push(mp.center_of_mass.coords);
        } else {
            model.body_mass.push(0.01); // fallback
            model.body_inertia.push(Vector3::new(1e-6, 1e-6, 1e-6));
            model.body_ipos.push(Vector3::zeros());
        }
        model.body_iquat.push(UnitQuaternion::identity());

        // Placeholders for geom/jnt address tracking (set below)
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(0);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(0);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0);
        model.body_subtreemass.push(0.0);
        model.body_mocapid.push(None);
        model.body_gravcomp.push(0.0);
        model.body_invweight0.push([0.0; 2]);
        model.body_treeid.push(usize::MAX);

        // Plugin placeholders
        model.body_plugin.push(None);

        // User data
        model.body_user.push(vec![]);
    }
    model.nbody = nbody;

    // ── Joints ──────────────────────────────────────────────────────
    let mut nq = 0usize;
    let mut nv = 0usize;

    for &part_name in &body_order {
        let body_id = part_to_body[part_name];
        let body_anchor = joints_on
            .get(part_name)
            .and_then(|jl| jl.first())
            .map_or_else(Point3::origin, |j| *j.anchor());

        model.body_jnt_adr[body_id] = model.jnt_type.len();
        model.body_dof_adr[body_id] = nv;

        let jlist = joints_on.get(part_name);
        let jnt_count = jlist.map_or(0, Vec::len);

        if let Some(jlist) = jlist {
            for joint in jlist {
                let jnt_type = match joint.kind() {
                    JointKind::Revolute => MjJointType::Hinge,
                    JointKind::Prismatic => MjJointType::Slide,
                    JointKind::Ball => MjJointType::Ball,
                    JointKind::Free => MjJointType::Free,
                };

                let jnt_id = model.jnt_type.len();

                // Joint position in body-local frame
                let local_pos = joint.anchor() - body_anchor;

                model.jnt_type.push(jnt_type);
                model.jnt_body.push(body_id);
                model.jnt_qpos_adr.push(nq);
                model.jnt_dof_adr.push(nv);
                model
                    .jnt_pos
                    .push(Vector3::new(local_pos.x, local_pos.y, local_pos.z));
                model.jnt_axis.push(*joint.axis());

                // Range
                if let Some((lo, hi)) = joint.range() {
                    model.jnt_limited.push(true);
                    model.jnt_range.push((lo, hi));
                } else {
                    model.jnt_limited.push(false);
                    model.jnt_range.push((0.0, 0.0));
                }

                // Spring/damper
                model.jnt_stiffness.push(joint.stiffness().unwrap_or(0.0));
                model.jnt_springref.push(0.0);
                model.jnt_damping.push(joint.damping().unwrap_or(0.0));
                model.jnt_armature.push(0.0);
                model.jnt_solref.push([0.02, 1.0]);
                model.jnt_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
                model.jnt_name.push(Some(joint.name().to_string()));
                model.jnt_group.push(0);
                model.jnt_actgravcomp.push(false);
                model.jnt_margin.push(0.0);

                // DOF arrays
                let dof_count = jnt_type.nv();

                for d in 0..dof_count {
                    model.dof_body.push(body_id);
                    model.dof_jnt.push(jnt_id);
                    // dof_parent: previous dof in same body, or parent body's last dof
                    let dof_parent = if d > 0 {
                        Some(nv + d - 1)
                    } else if body_id > 1 {
                        // Find last DOF of parent body
                        let pbody = model.body_parent[body_id];
                        let padr = model.body_dof_adr[pbody];
                        let pnum = model.body_dof_num[pbody];
                        if pnum > 0 {
                            Some(padr + pnum - 1)
                        } else {
                            None
                        }
                    } else {
                        None
                    };
                    model.dof_parent.push(dof_parent);
                    model.dof_armature.push(0.0);
                    model.dof_damping.push(joint.damping().unwrap_or(0.0));
                    model.dof_frictionloss.push(0.0);
                    model.dof_solref.push([0.02, 1.0]);
                    model.dof_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
                    model.dof_invweight0.push(0.0);
                    model.dof_treeid.push(0);
                }

                nq += jnt_type.nq();
                nv += dof_count;
            }
        }

        model.body_jnt_num[body_id] = jnt_count;
        model.body_dof_num[body_id] = jlist.map_or(0, |v| {
            v.iter()
                .map(|j| match j.kind() {
                    JointKind::Revolute | JointKind::Prismatic => 1,
                    JointKind::Ball => 3,
                    JointKind::Free => 6,
                })
                .sum()
        });
    }

    model.njnt = model.jnt_type.len();
    model.nq = nq;
    model.nv = nv;

    // qpos0: initial joint positions
    model.qpos0 = nalgebra::DVector::zeros(nq);
    // Set quaternion components for ball/free joints
    for jnt_id in 0..model.njnt {
        match model.jnt_type[jnt_id] {
            MjJointType::Ball => {
                let adr = model.jnt_qpos_adr[jnt_id];
                model.qpos0[adr] = 1.0; // w=1 for identity quaternion
            }
            MjJointType::Free => {
                let adr = model.jnt_qpos_adr[jnt_id];
                model.qpos0[adr + 3] = 1.0; // w=1 for identity quaternion
            }
            _ => {}
        }
    }
    model.qpos_spring = model.qpos0.as_slice().to_vec();

    // ── Geoms ───────────────────────────────────────────────────────
    // Two geoms per part: SDF (collision) + Mesh (visual)
    for &part_name in &body_order {
        let body_id = part_to_body[part_name];
        let (_, part) = part_by_name[part_name];
        let &(sdf_id, mesh_id, _) = &part_assets[part_name];

        let geom_offset = compute_geom_offset(part, &joints_on);

        // RGBA from material color (or default bone color)
        let rgba = part.material().color.unwrap_or([0.9, 0.85, 0.75, 1.0]);

        model.body_geom_adr[body_id] = model.geom_type.len();

        // --- SDF geom (collision only, group 2) ---
        let sdf_grid = &sdf_data[sdf_id];
        let sdf_aabb = cf_geometry::Bounded::aabb(sdf_grid.as_ref());
        let sdf_half = (sdf_aabb.max - sdf_aabb.min) * 0.5;

        push_geom(
            &mut model,
            GeomType::Sdf,
            body_id,
            geom_offset,
            Vector3::new(sdf_half.x, sdf_half.y, sdf_half.z),
            Some(sdf_id), // geom_sdf
            None,         // geom_mesh
            None,         // geom_hfield
            2,            // group 2 = collision only (not rendered)
            rgba,
            Some(format!("{}_sdf", part_name)),
        );

        // --- Mesh geom (visual only, group 0) ---
        push_geom(
            &mut model,
            GeomType::Mesh,
            body_id,
            geom_offset,
            Vector3::new(1.0, 1.0, 1.0), // scale
            None,                        // geom_sdf
            Some(mesh_id),               // geom_mesh
            None,                        // geom_hfield
            0,                           // group 0 = visual (rendered)
            rgba,
            Some(format!("{}_mesh", part_name)),
        );

        model.body_geom_num[body_id] = 2;
    }
    model.ngeom = model.geom_type.len();

    // ── Mesh/SDF asset storage ──────────────────────────────────────
    model.mesh_data = mesh_data;
    model.nmesh = model.mesh_data.len();
    model.mesh_name = parts.iter().map(|p| format!("{}_mesh", p.name())).collect();
    model.sdf_data = sdf_data;
    model.nsdf = model.sdf_data.len();

    // ── Sites (tendon waypoints) ────────────────────────────────────
    let site_map = build_site_map(tendons);
    for &part_name in &body_order {
        let body_id = part_to_body[part_name];
        if let Some(sites) = site_map.get(part_name) {
            for (name, pos) in sites {
                model.site_body.push(body_id);
                model.site_type.push(GeomType::Sphere); // visual marker
                model.site_pos.push(Vector3::new(pos[0], pos[1], pos[2]));
                model.site_quat.push(UnitQuaternion::identity());
                model.site_size.push(Vector3::new(0.5, 0.5, 0.5));
                model.site_name.push(Some(name.clone()));
                model.site_group.push(0);
                model.site_rgba.push([1.0, 0.0, 0.0, 1.0]);

                model.site_user.push(vec![]);
            }
        }
    }
    model.nsite = model.site_body.len();

    // ── Name lookup maps ────────────────────────────────────────────
    for (i, name) in model.body_name.iter().enumerate() {
        if let Some(n) = name {
            model.body_name_to_id.insert(n.clone(), i);
        }
    }
    for (i, name) in model.jnt_name.iter().enumerate() {
        if let Some(n) = name {
            model.jnt_name_to_id.insert(n.clone(), i);
        }
    }
    for (i, name) in model.geom_name.iter().enumerate() {
        if let Some(n) = name {
            model.geom_name_to_id.insert(n.clone(), i);
        }
    }
    for (i, name) in model.site_name.iter().enumerate() {
        if let Some(n) = name {
            model.site_name_to_id.insert(n.clone(), i);
        }
    }
    for (i, name) in model.mesh_name.iter().enumerate() {
        model.mesh_name_to_id.insert(name.clone(), i);
    }

    // ── Tendons ─────────────────────────────────────────────────────
    let site_name_to_id = &model.site_name_to_id;
    for tendon in tendons {
        model.tendon_type.push(TendonType::Spatial);
        model.tendon_range.push((0.0, 0.0));
        model.tendon_limited.push(false);
        model
            .tendon_stiffness
            .push(tendon.stiffness().unwrap_or(0.0));
        model.tendon_damping.push(tendon.damping().unwrap_or(0.0));
        model.tendon_frictionloss.push(0.0);
        model.tendon_solref_fri.push([0.02, 1.0]);
        model.tendon_solimp_fri.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.tendon_lengthspring.push([0.0, 0.0]);
        model.tendon_length0.push(0.0); // computed post-build
        model.tendon_num.push(tendon.waypoints().len());
        model.tendon_adr.push(model.wrap_type.len());
        model.tendon_name.push(Some(tendon.name().to_string()));
        model.tendon_group.push(0);
        model.tendon_rgba.push([0.5, 0.5, 0.5, 1.0]);
        model.tendon_solref_lim.push([0.02, 1.0]);
        model.tendon_solimp_lim.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.tendon_margin.push(0.0);
        model.tendon_treenum.push(0);
        model.tendon_tree.push(usize::MAX);
        model.tendon_tree.push(usize::MAX);
        model.tendon_invweight0.push(0.0);
        model.tendon_user.push(vec![]);

        // Wrap objects (site references)
        for (i, _wp) in tendon.waypoints().iter().enumerate() {
            let site_name = format!("{}_s{i}", tendon.name());
            let site_id = site_name_to_id
                .get(&site_name)
                .copied()
                .unwrap_or_else(|| panic!("site '{site_name}' not found in model"));
            model.wrap_type.push(WrapType::Site);
            model.wrap_objid.push(site_id);
            model.wrap_prm.push(0.0);
            model.wrap_sidesite.push(usize::MAX); // no sidesite
        }
    }
    model.ntendon = model.tendon_type.len();
    model.nwrap = model.wrap_type.len();

    // ── Actuators ───────────────────────────────────────────────────
    for act in actuators {
        let tendon_id = model
            .tendon_name
            .iter()
            .position(|n| n.as_deref() == Some(act.tendon()))
            .unwrap_or_else(|| {
                panic!(
                    "actuator '{}' references unknown tendon '{}'",
                    act.name(),
                    act.tendon()
                )
            });

        model.actuator_trntype.push(ActuatorTransmission::Tendon);
        model.actuator_dyntype.push(ActuatorDynamics::None);
        model.actuator_trnid.push([tendon_id, 0]);
        model.actuator_name.push(Some(act.name().to_string()));

        let (flo, fhi) = act.force_range();
        model.actuator_forcerange.push((flo, fhi));

        // Gear: scale ctrl → force
        if let Some((clo, chi)) = act.ctrl_range() {
            model.actuator_ctrlrange.push((clo, chi));
            let max_ctrl = chi.abs().max(clo.abs());
            let max_force = fhi.abs().max(flo.abs());
            let gear = if max_ctrl > 1e-10 {
                max_force / max_ctrl
            } else {
                1.0
            };
            model.actuator_gear.push([gear, 0.0, 0.0, 0.0, 0.0, 0.0]);
        } else {
            model.actuator_ctrlrange.push((0.0, 0.0));
            model.actuator_gear.push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        }

        // Gain/bias for general actuator: gain = gear, bias = 0 (direct ctrl mapping)
        match act.kind() {
            ActuatorKind::Motor => {
                model.actuator_gaintype.push(GainType::Fixed);
                model.actuator_biastype.push(BiasType::None);
                model
                    .actuator_gainprm
                    .push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                model
                    .actuator_biasprm
                    .push([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
            }
            ActuatorKind::Muscle => {
                model.actuator_gaintype.push(GainType::Muscle);
                model.actuator_biastype.push(BiasType::Muscle);
                model
                    .actuator_gainprm
                    .push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                model
                    .actuator_biasprm
                    .push([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
            }
        }

        model
            .actuator_dynprm
            .push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        model.actuator_act_adr.push(-1i32 as usize);
        model.actuator_act_num.push(0);
        model.actuator_lengthrange.push((0.0, 0.0));
        model.actuator_acc0.push(0.0);
        model.actuator_actlimited.push(false);
        model.actuator_actrange.push((0.0, 0.0));
        model.actuator_actearly.push(false);
        model.actuator_cranklength.push(0.0);
        model.actuator_nsample.push(0);
        model
            .actuator_interp
            .push(sim_core::InterpolationType::Linear);
        model.actuator_historyadr.push(0);
        model.actuator_delay.push(0.0);
        model.actuator_plugin.push(None);
        model.actuator_user.push(vec![]);
        model.actuator_group.push(0);
    }
    model.nu = model.actuator_trntype.len();
    model.na = 0;

    // ── Post-build computations ─────────────────────────────────────
    model.compute_ancestors();
    model.compute_implicit_params();
    model.compute_qld_csr_metadata();

    // Tendon/actuator derived parameters
    model.compute_spatial_tendon_length0();
    model.compute_actuator_params();
    model.compute_stat_meaninertia();
    model.compute_invweight0();

    // Bounding geometry
    compute_geom_bounding(&mut model);

    // Kinematic tree enumeration
    discover_kinematic_trees(&mut model);
    sim_core::compute_dof_lengths(&mut model);

    model
}

// ── Helpers ─────────────────────────────────────────────────────────────

/// Push a single geom entry to all Model geom arrays.
#[allow(clippy::too_many_arguments)]
fn push_geom(
    model: &mut Model,
    geom_type: GeomType,
    body_id: usize,
    pos: Vector3<f64>,
    size: Vector3<f64>,
    sdf_id: Option<usize>,
    mesh_id: Option<usize>,
    hfield_id: Option<usize>,
    group: i32,
    rgba: [f64; 4],
    name: Option<String>,
) {
    model.geom_type.push(geom_type);
    model.geom_body.push(body_id);
    model.geom_pos.push(pos);
    model.geom_quat.push(UnitQuaternion::identity());
    model.geom_size.push(size);
    model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
    model.geom_condim.push(3);
    model.geom_contype.push(1);
    model.geom_conaffinity.push(1);
    model.geom_margin.push(0.0);
    model.geom_gap.push(0.0);
    model.geom_priority.push(0);
    model.geom_solmix.push(1.0);
    model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
    model.geom_solref.push([0.02, 1.0]);
    model.geom_name.push(name);
    model.geom_rbound.push(0.0); // computed in post-build
    model.geom_aabb.push([0.0; 6]); // computed in post-build
    model.geom_mesh.push(mesh_id);
    model.geom_hfield.push(hfield_id);
    model.geom_sdf.push(sdf_id);
    model.geom_group.push(group);
    model.geom_rgba.push(rgba);
    model.geom_fluid.push([0.0; 12]);

    model.geom_plugin.push(None);
    model.geom_user.push(vec![]);

    // Visual-only geoms (Mesh) should not participate in collision.
    let geom_idx = model.geom_type.len() - 1;
    if geom_type == GeomType::Mesh {
        model.geom_contype[geom_idx] = 0;
        model.geom_conaffinity[geom_idx] = 0;
    }
}

/// Compute geom position offset for child parts (same logic as mjcf.rs).
fn compute_geom_offset(part: &Part, joints_on: &HashMap<&str, Vec<&JointDef>>) -> Vector3<f64> {
    let jlist = match joints_on.get(part.name()) {
        Some(jl) if !jl.is_empty() => jl,
        _ => return Vector3::zeros(), // root body
    };

    // Explicit joint origin
    if let Some(origin) = part.joint_origin() {
        return -origin;
    }

    let anchor = jlist[0].anchor();
    let anchor_vec = anchor.coords;
    if anchor_vec.norm() < 1e-10 {
        return Vector3::zeros();
    }

    // Bbox-based alignment
    let mesh = part.solid().mesh(1.0);
    if mesh.vertices.is_empty() {
        return Vector3::zeros();
    }

    let mut min = mesh.vertices[0].coords;
    let mut max = mesh.vertices[0].coords;
    for v in &mesh.vertices {
        min = min.inf(&v.coords);
        max = max.sup(&v.coords);
    }
    let extents = max - min;
    let center = (max + min) * 0.5;

    let longest_axis = if extents.x >= extents.y && extents.x >= extents.z {
        0
    } else if extents.y >= extents.z {
        1
    } else {
        2
    };

    let near_edge = if anchor_vec[longest_axis] >= 0.0 {
        min[longest_axis]
    } else {
        max[longest_axis]
    };
    let mut offset = center;
    offset[longest_axis] = -near_edge;
    offset
}

/// Build `part_name → Vec<(site_name, [x, y, z])>` from tendon waypoints.
fn build_site_map(tendons: &[TendonDef]) -> HashMap<&str, Vec<(String, [f64; 3])>> {
    let mut map: HashMap<&str, Vec<(String, [f64; 3])>> = HashMap::new();
    for tendon in tendons {
        for (i, wp) in tendon.waypoints().iter().enumerate() {
            let name = format!("{}_s{i}", tendon.name());
            let pos = [wp.position().x, wp.position().y, wp.position().z];
            map.entry(wp.part()).or_default().push((name, pos));
        }
    }
    map
}

/// Compute bounding radii and AABBs for all geoms.
fn compute_geom_bounding(model: &mut Model) {
    for geom_id in 0..model.ngeom {
        let geom_type = model.geom_type[geom_id];
        let size = model.geom_size[geom_id];

        // Bounding radius
        model.geom_rbound[geom_id] = match geom_type {
            GeomType::Sdf => {
                if let Some(sdf_id) = model.geom_sdf[geom_id] {
                    let sdf = &model.sdf_data[sdf_id];
                    let aabb = cf_geometry::Bounded::aabb(sdf.as_ref());
                    let half = (aabb.max - aabb.min) * 0.5;
                    half.norm()
                } else {
                    size.norm()
                }
            }
            GeomType::Mesh => {
                if let Some(mesh_id) = model.geom_mesh[geom_id] {
                    let tri = &model.mesh_data[mesh_id];
                    let indexed = tri.indexed_mesh();
                    if indexed.vertices.is_empty() {
                        1.0
                    } else {
                        indexed
                            .vertices
                            .iter()
                            .map(|v| v.coords.norm())
                            .fold(0.0_f64, f64::max)
                    }
                } else {
                    10.0
                }
            }
            _ => geom_type.bounding_radius(size),
        };

        // AABB: [cx, cy, cz, hx, hy, hz]
        model.geom_aabb[geom_id] = match geom_type {
            GeomType::Sdf => {
                if let Some(sdf_id) = model.geom_sdf[geom_id] {
                    let sdf = &model.sdf_data[sdf_id];
                    let aabb = cf_geometry::Bounded::aabb(sdf.as_ref());
                    let center = (aabb.max.coords + aabb.min.coords) * 0.5;
                    let half = (aabb.max.coords - aabb.min.coords) * 0.5;
                    [center.x, center.y, center.z, half.x, half.y, half.z]
                } else {
                    [0.0; 6]
                }
            }
            GeomType::Mesh => {
                if let Some(mesh_id) = model.geom_mesh[geom_id] {
                    let tri = &model.mesh_data[mesh_id];
                    let indexed = tri.indexed_mesh();
                    if indexed.vertices.is_empty() {
                        [0.0; 6]
                    } else {
                        let mut bmin = indexed.vertices[0].coords;
                        let mut bmax = indexed.vertices[0].coords;
                        for v in &indexed.vertices {
                            bmin = bmin.inf(&v.coords);
                            bmax = bmax.sup(&v.coords);
                        }
                        let center = (bmax + bmin) * 0.5;
                        let half = (bmax - bmin) * 0.5;
                        [center.x, center.y, center.z, half.x, half.y, half.z]
                    }
                } else {
                    [0.0; 6]
                }
            }
            _ => {
                // Primitives: center at origin, half-extents from size
                [0.0, 0.0, 0.0, size.x, size.y, size.z]
            }
        };
    }
}

/// Discover kinematic trees (simplified — one tree per root body).
fn discover_kinematic_trees(model: &mut Model) {
    let mut tree_id = 0usize;
    model.tree_body_adr.clear();
    model.tree_body_num.clear();
    model.tree_dof_adr.clear();
    model.tree_dof_num.clear();
    model.tree_sleep_policy.clear();

    for body_id in 1..model.nbody {
        if model.body_parent[body_id] == 0 {
            // Root of a new tree
            model.tree_body_adr.push(body_id);

            // Count bodies in this tree
            let mut count = 0;
            let mut dof_start = usize::MAX;
            let mut dof_count = 0;

            for b in body_id..model.nbody {
                // Check if b is in this tree (trace parent chain to body_id)
                let mut cur = b;
                loop {
                    if cur == body_id {
                        count += 1;
                        model.body_treeid[b] = tree_id;
                        // Track DOFs
                        let adr = model.body_dof_adr[b];
                        let num = model.body_dof_num[b];
                        if num > 0 {
                            dof_start = dof_start.min(adr);
                            dof_count += num;
                            for d in adr..adr + num {
                                model.dof_treeid[d] = tree_id;
                            }
                        }
                        break;
                    }
                    if cur == 0 || model.body_parent[cur] == cur {
                        break;
                    }
                    cur = model.body_parent[cur];
                }
            }

            model.tree_body_num.push(count);
            model.tree_dof_adr.push(if dof_start == usize::MAX {
                0
            } else {
                dof_start
            });
            model.tree_dof_num.push(dof_count);
            model.tree_sleep_policy.push(sim_core::SleepPolicy::Never);

            tree_id += 1;
        }
    }

    model.ntree = tree_id;
    model.dof_length = vec![1.0; model.nv];
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use crate::{
        ActuatorDef, ActuatorKind, JointDef, JointKind, Material, Mechanism, Part, Solid,
        TendonDef, TendonWaypoint,
    };
    use sim_core::GeomType;

    fn pla() -> Material {
        Material::new("PLA", 1250.0)
    }

    fn cuboid_part(name: &str) -> Part {
        Part::new(name, Solid::cuboid(Vector3::new(5.0, 5.0, 5.0)), pla())
    }

    /// Build a minimal two-part mechanism.
    fn two_part_mechanism() -> Mechanism {
        Mechanism::builder("two_part")
            .part(cuboid_part("palm"))
            .part(cuboid_part("finger"))
            .joint(JointDef::new(
                "knuckle",
                "palm",
                "finger",
                JointKind::Revolute,
                Point3::new(5.0, 0.0, 0.0),
                Vector3::x(),
            ))
            .build()
    }

    // ── 1. Model dimensions ────────────────────────────────────────

    #[test]
    fn model_dimensions() {
        let model = two_part_mechanism().to_model(2.0, 2.0);

        // 2 parts → 3 bodies (world + palm + finger)
        assert_eq!(model.nbody, 3);
        // 1 revolute joint → 1 DOF
        assert_eq!(model.njnt, 1);
        assert_eq!(model.nq, 1);
        assert_eq!(model.nv, 1);
        // 2 geoms per part (SDF + mesh) → 4 geoms total
        assert_eq!(model.ngeom, 4);
    }

    // ── 2. SDF geom present ────────────────────────────────────────

    #[test]
    fn sdf_geoms_present() {
        let model = two_part_mechanism().to_model(2.0, 2.0);

        let sdf_count = model
            .geom_type
            .iter()
            .filter(|&&t| t == GeomType::Sdf)
            .count();
        assert_eq!(sdf_count, 2, "expected 2 SDF geoms (one per part)");

        // SDF geoms should have sdf_id set
        for (i, &gt) in model.geom_type.iter().enumerate() {
            if gt == GeomType::Sdf {
                assert!(
                    model.geom_sdf[i].is_some(),
                    "SDF geom {i} should have geom_sdf set"
                );
            }
        }
    }

    // ── 3. Mesh geom present (visual) ──────────────────────────────

    #[test]
    fn mesh_geoms_present() {
        let model = two_part_mechanism().to_model(2.0, 2.0);

        let mesh_count = model
            .geom_type
            .iter()
            .filter(|&&t| t == GeomType::Mesh)
            .count();
        assert_eq!(mesh_count, 2, "expected 2 mesh geoms (one per part)");

        // Mesh geoms should have no collision (contype=0)
        for (i, &gt) in model.geom_type.iter().enumerate() {
            if gt == GeomType::Mesh {
                assert_eq!(model.geom_contype[i], 0, "mesh geom should not collide");
            }
        }
    }

    // ── 4. SDF data populated ──────────────────────────────────────

    #[test]
    fn sdf_data_populated() {
        let model = two_part_mechanism().to_model(2.0, 2.0);

        assert_eq!(model.sdf_data.len(), 2, "expected 2 SDF grids");
        for (i, sdf) in model.sdf_data.iter().enumerate() {
            assert!(sdf.width() > 1, "SDF {i} width too small");
            assert!(sdf.height() > 1, "SDF {i} height too small");
            assert!(sdf.depth() > 1, "SDF {i} depth too small");
            // Interior points should be negative
            assert!(sdf.min_value() < 0.0, "SDF {i} has no interior");
        }
    }

    // ── 5. Parent-child filtered (default) ──────────────────────────

    #[test]
    fn parent_child_collision_filtered() {
        let model = two_part_mechanism().to_model(2.0, 2.0);
        assert_eq!(
            model.disableflags & sim_core::DISABLE_FILTERPARENT,
            0,
            "DISABLE_FILTERPARENT should NOT be set by default"
        );
    }

    // ── 6. Forward kinematics works ────────────────────────────────

    #[test]
    fn forward_kinematics_runs() {
        let model = two_part_mechanism().to_model(2.0, 2.0);
        let mut data = model.make_data();
        let result = data.forward(&model);
        assert!(
            result.is_ok(),
            "forward kinematics should succeed: {:?}",
            result
        );
    }

    // ── 7. Tendon and actuator ─────────────────────────────────────

    #[test]
    fn tendon_actuator_model() {
        let m = Mechanism::builder("actuated")
            .part(cuboid_part("a"))
            .part(cuboid_part("b"))
            .joint(JointDef::new(
                "j",
                "a",
                "b",
                JointKind::Revolute,
                Point3::new(5.0, 0.0, 0.0),
                Vector3::x(),
            ))
            .tendon(
                TendonDef::new(
                    "cable",
                    vec![
                        TendonWaypoint::new("a", Point3::origin()),
                        TendonWaypoint::new("b", Point3::new(0.0, 5.0, 0.0)),
                    ],
                    0.5,
                )
                .with_stiffness(200.0)
                .with_damping(10.0),
            )
            .actuator(
                ActuatorDef::new("motor", "cable", ActuatorKind::Motor, (-50.0, 50.0))
                    .with_ctrl_range(-1.0, 1.0),
            )
            .build();

        let model = m.to_model(2.0, 2.0);

        assert_eq!(model.ntendon, 1, "expected 1 tendon");
        assert_eq!(model.nu, 1, "expected 1 actuator");
        assert_eq!(model.nsite, 2, "expected 2 sites (tendon waypoints)");
    }

    // ── 8. Single part mechanism ───────────────────────────────────

    #[test]
    fn single_part_model() {
        let m = Mechanism::builder("solo")
            .part(Part::new("body", Solid::sphere(5.0), pla()))
            .build();

        let model = m.to_model(2.0, 2.0);

        assert_eq!(model.nbody, 2); // world + body
        assert_eq!(model.njnt, 0);
        assert_eq!(model.ngeom, 2); // SDF + mesh
    }
}
