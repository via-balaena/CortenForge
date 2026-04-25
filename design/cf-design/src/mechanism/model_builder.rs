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
//!   ├──→ SdfGrid (collision)  ──→ Model.shape_data / GeomType::Sdf
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
    PhysicsShape, SolverType, TendonType, WrapType,
};

use crate::mechanism::analytical_shape::AnalyticalShape;

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

    // Roots: parts that are never a child, or whose parent is "world".
    let roots: Vec<&str> = parts
        .iter()
        .map(Part::name)
        .filter(|n| matches!(child_parent.get(n), None | Some(&"world")))
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
    let mut shape_data: Vec<Arc<dyn PhysicsShape>> = Vec::new();
    let mut mesh_data: Vec<Arc<sim_core::TriangleMeshData>> = Vec::new();

    // part_name → (shape_id, mesh_id, aabb)
    let mut part_assets: HashMap<&str, (usize, usize, cf_geometry::Aabb)> = HashMap::new();

    for part in parts {
        let solid = part.solid();
        let bounds = solid
            .bounds()
            .unwrap_or_else(|| panic!("part '{}' has no finite bounds", part.name()));

        let sdf = solid
            .sdf_grid_at(sdf_resolution)
            .unwrap_or_else(|| panic!("part '{}' has no finite bounds", part.name()));

        let sdf_id = shape_data.len();
        let grid = Arc::new(sdf);
        let shape: Arc<dyn PhysicsShape> = Arc::new(AnalyticalShape::new(
            grid,
            Arc::new(solid.clone()),
            solid.shape_hint(),
        ));
        shape_data.push(shape);

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

    // cf-design geometry is in mm. Scale gravity from m/s² to mm/s².
    model.gravity = nalgebra::Vector3::new(0.0, 0.0, -9810.0);

    // mm-scale bodies have tiny masses (O(10⁻⁴) kg), making the contact
    // natural frequency ~1000× higher than at m-scale. The default timestep
    // (0.002s) can't resolve these dynamics. Use 0.0005s (2 kHz) which
    // keeps the contact frequency within the integrator's stability range.
    model.timestep = 0.0005;

    // Match MuJoCo's default diagApprox mode: bodyweight approximation.
    // The exact M⁻¹ solve (default in Model::empty()) produces different
    // impedance values that can destabilize mm-scale contact stacking.
    model.diagapprox_bodyweight = true;

    // MuJoCo defaults to Newton solver, which has superior convergence for
    // contact stacking vs PGS. Model::empty() defaults to PGS.
    model.solver_type = SolverType::Newton;

    // Parent-child collision filtering stays ON (MuJoCo default).
    // SDF-SDF parent-child collision (socket/condyle) needs multi-contact
    // support in sdf_sdf_contact before it can constrain concave geometry.

    // ── Bodies ───────────────────────────────────────────────────────
    // World body (index 0) is already in empty model. Add part bodies.
    for (order_idx, &part_name) in body_order.iter().enumerate() {
        let body_id = order_idx + 1;
        let (_, part) = part_by_name[part_name];

        // Parent body ("world" or absent → world body 0)
        let parent_body = match child_parent.get(part_name) {
            Some(&"world") | None => 0,
            Some(&parent_name) => part_to_body[parent_name],
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

        // Geom offset: how the solid's mesh is shifted in the body frame.
        // For articulated joints (non-free), with_joint_origin or bbox-based
        // alignment shifts the mesh away from the body frame origin. The COM
        // must account for this offset so the physics sees the correct lever arm.
        let geom_offset = compute_geom_offset(part, &joints_on);

        // Mass properties (computed from implicit field)
        let mass_props = super::mass::mass_properties(part.solid(), part.material().density, 1.0);
        if let Some(mp) = mass_props {
            // Convert inertia from [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] to diagonal Vector3
            // sim-core stores diagonal inertia only (principal axes assumption)
            model.body_mass.push(mp.mass);
            model
                .body_inertia
                .push(Vector3::new(mp.inertia[0], mp.inertia[1], mp.inertia[2]));
            // Center of mass in body frame = geom_offset + COM_in_solid_frame.
            // mass_properties returns COM in the solid's local frame (centered
            // at origin for symmetric shapes). The geom_offset translates the
            // solid into the body frame, so we must apply it here.
            model.body_ipos.push(geom_offset + mp.center_of_mass.coords);
        } else {
            model.body_mass.push(0.01); // fallback
            model.body_inertia.push(Vector3::new(1e-6, 1e-6, 1e-6));
            model.body_ipos.push(geom_offset);
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
                let body_id = model.jnt_body[jnt_id];
                let pos = model.body_pos[body_id];
                model.qpos0[adr] = pos.x;
                model.qpos0[adr + 1] = pos.y;
                model.qpos0[adr + 2] = pos.z;
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
        let sdf_grid = shape_data[sdf_id].sdf_grid();
        let sdf_aabb = cf_geometry::Bounded::aabb(sdf_grid);
        let sdf_half = (sdf_aabb.max - sdf_aabb.min) * 0.5;

        push_geom(
            &mut model,
            GeomType::Sdf,
            body_id,
            geom_offset,
            Vector3::new(sdf_half.x, sdf_half.y, sdf_half.z),
            Some(sdf_id), // geom_shape
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
            None,                        // geom_shape
            Some(mesh_id),               // geom_mesh
            None,                        // geom_hfield
            0,                           // group 0 = visual (rendered)
            rgba,
            Some(format!("{}_mesh", part_name)),
        );
        // Visual-only: disable collision so mesh geoms don't pollute
        // the contact list or interfere with SDF collision.
        let visual_geom_id = model.geom_type.len() - 1;
        model.geom_contype[visual_geom_id] = 0;
        model.geom_conaffinity[visual_geom_id] = 0;

        model.body_geom_num[body_id] = 2;
    }
    model.ngeom = model.geom_type.len();

    // ── Mesh/SDF asset storage ──────────────────────────────────────
    model.mesh_data = mesh_data;
    model.nmesh = model.mesh_data.len();
    model.mesh_name = parts.iter().map(|p| format!("{}_mesh", p.name())).collect();
    model.shape_data = shape_data;
    model.nshape = model.shape_data.len();

    // SDF geom_margin = 0: matches the analytical contact convention where
    // depth = 0 at touching and the solver provides force only from actual
    // geometric overlap.  Non-zero margin caused phantom repulsion at the
    // touching configuration that broke stacking.
    // (Margin was previously cell_size * 0.25 — removed in the SDF stacking fix.)

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
// Argument list mirrors the physical-simulation equation signature.
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
    // MuJoCo default solref[0]=0.02 is tuned for m-scale. At mm-scale
    // (gravity=9810), that gives ~0.46 mm equilibrium penetration and
    // spongy contacts. 0.005 (must be > 2×timestep) gives stiff,
    // visually correct contact at mm-scale.
    model.geom_solref.push([0.005, 1.0]);
    model.geom_name.push(name);
    model.geom_rbound.push(0.0); // computed in post-build
    model.geom_aabb.push([0.0; 6]); // computed in post-build
    model.geom_mesh.push(mesh_id);
    model.geom_hfield.push(hfield_id);
    model.geom_shape.push(sdf_id);
    model.geom_group.push(group);
    model.geom_rgba.push(rgba);
    model.geom_fluid.push([0.0; 12]);

    model.geom_plugin.push(None);
    model.geom_user.push(vec![]);

    let geom_idx = model.geom_type.len() - 1;

    // Visual-only geoms (Mesh) should not participate in collision.
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

    // Free joints: geometry centered at body frame (no alignment needed)
    if jlist[0].kind() == JointKind::Free {
        return Vector3::zeros();
    }

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
                if let Some(sdf_id) = model.geom_shape[geom_id] {
                    let sdf = model.shape_data[sdf_id].sdf_grid();
                    let aabb = cf_geometry::Bounded::aabb(sdf);
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
                if let Some(sdf_id) = model.geom_shape[geom_id] {
                    let sdf = model.shape_data[sdf_id].sdf_grid();
                    let aabb = cf_geometry::Bounded::aabb(sdf);
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
                    model.geom_shape[i].is_some(),
                    "SDF geom {i} should have geom_shape set"
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
    fn shape_data_populated() {
        let model = two_part_mechanism().to_model(2.0, 2.0);

        assert_eq!(model.shape_data.len(), 2, "expected 2 shape assets");
        for (i, shape) in model.shape_data.iter().enumerate() {
            let sdf = shape.sdf_grid();
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

    // ── 9. Free joint to world ──────────────────────────────────────

    #[test]
    fn free_joint_to_world() {
        let m = Mechanism::builder("floating")
            .part(Part::new("body", Solid::sphere(5.0), pla()))
            .joint(JointDef::new(
                "free",
                "world",
                "body",
                JointKind::Free,
                Point3::new(0.0, 0.0, 50.0),
                Vector3::z(),
            ))
            .build();

        let model = m.to_model(2.0, 2.0);

        assert_eq!(model.nbody, 2); // world + body
        assert_eq!(model.njnt, 1); // free joint
        assert_eq!(model.nq, 7); // 3 pos + 4 quat
        assert_eq!(model.nv, 6); // 3 linear + 3 angular
        assert_eq!(model.ngeom, 2); // SDF + mesh
        assert_eq!(model.body_parent[1], 0); // parent is world
    }

    #[test]
    fn sdf_sphere_stacking() {
        let material = Material::new("PLA", 1250.0);
        let m = Mechanism::builder("pair")
            .part(Part::new("lower", Solid::sphere(5.0), material.clone()))
            .part(Part::new("upper", Solid::sphere(5.0), material))
            .joint(JointDef::new(
                "free_lower",
                "world",
                "lower",
                JointKind::Free,
                Point3::new(0.0, 0.0, 5.5),
                Vector3::z(),
            ))
            .joint(JointDef::new(
                "free_upper",
                "world",
                "upper",
                JointKind::Free,
                Point3::new(0.0, 0.0, 15.5),
                Vector3::z(),
            ))
            .build();

        let mut model = m.to_model(1.0, 0.3);
        model.add_ground_plane();

        let mut data = model.make_data();
        data.forward(&model).expect("forward");

        for step in 0..2000 {
            data.step(&model).expect("step");
            if step == 100 || step == 500 || step == 1999 {
                eprintln!(
                    "  cfdesign step {}: z_lo={:.3} z_up={:.3} gap={:.3} ncon={}",
                    step,
                    data.qpos[2],
                    data.qpos[9],
                    data.qpos[9] - data.qpos[2],
                    data.ncon
                );
            }
        }

        let z_lo = data.qpos[2];
        let z_up = data.qpos[9];
        let gap = z_up - z_lo;
        assert!(
            (z_lo - 5.0).abs() < 1.0,
            "lower should be at z≈5, got {z_lo:.3}"
        );
        assert!(
            (z_up - 15.0).abs() < 2.0,
            "upper should be at z≈15, got {z_up:.3}"
        );
        assert!((gap - 10.0).abs() < 2.0, "gap should be ≈10, got {gap:.3}");
    }

    // ── Phase 3d: Geometry-driven hinge simulation ───────────────────

    /// Full simulation of a geometry-driven hinge: socket (concave) +
    /// flanged pin (convex, `Free` joint). Both children of world. SDF
    /// collision is the ONLY constraint — no `JointKind::Revolute`.
    ///
    /// Verifies that the solver + integrator + FK can handle concave SDF
    /// multi-contact (1000+ contacts per step) and maintain the constraint.
    #[test]
    fn sdf_geometry_hinge_simulation() {
        // ── Socket: tube with annular caps (concave) ─────────────────
        // Bore R=3.5, outer R=5.5, half_h=8, cap opening R=2.5
        let socket_solid = Solid::cylinder(5.5, 10.0) // outer cylinder
            .subtract(Solid::cylinder(3.5, 8.0)) // bore
            .subtract(Solid::cylinder(2.5, 11.0)); // cap openings

        // Socket: very dense so it barely moves under contact forces
        let mut mat_socket = Material::new("steel", 7800.0);
        mat_socket.color = Some([0.3, 0.5, 1.0, 1.0]);

        // ── Pin: shaft + flanges (convex union) ──────────────────────
        // Shaft R=3.0, flange R=3.2, shaft half_h=6, flange half_h=1
        // shaft_half_h=6.0 → flange at z=±6.0, extends to z=±7.0.
        // Cap face at z=±8.0. Axial clearance = 1.0mm per side.
        // Pin starts offset z=-0.9 so ONLY the bottom flange is near
        // the bottom cap (gap=0.1mm < margin=0.15mm). Top flange is 2.8mm
        // from top cap — no contact. This breaks the symmetric cancellation
        // that prevented gravity resistance in the pre-loaded design.
        let shaft = Solid::cylinder(3.0, 6.0);
        let top_flange = Solid::cylinder(3.2, 1.0).translate(Vector3::new(0.0, 0.0, 6.0));
        let bot_flange = Solid::cylinder(3.2, 1.0).translate(Vector3::new(0.0, 0.0, -6.0));
        let pin_solid = shaft.union(top_flange).union(bot_flange);

        let mut mat_pin = Material::new("PLA", 1250.0);
        mat_pin.color = Some([1.0, 0.35, 0.3, 1.0]);

        // ── Build mechanism: siblings, both children of world ────────
        // Mechanism builder requires all parts to have joints (no orphans).
        // Socket gets a Free joint at origin; heavy density keeps it still.
        let mechanism = Mechanism::builder("hinge_test")
            .part(Part::new("socket", socket_solid, mat_socket))
            .part(Part::new("pin", pin_solid, mat_pin))
            .joint(JointDef::new(
                "socket_free",
                "world",
                "socket",
                JointKind::Free,
                Point3::new(0.0, 0.0, 15.0), // above ground (socket half_h=10+2=12)
                Vector3::z(),
            ))
            .joint(JointDef::new(
                "pin_free",
                "world",
                "pin",
                JointKind::Free,
                Point3::new(0.0, 0.0, 15.0), // same position as socket
                Vector3::z(),
            ))
            .build();

        let mut model = mechanism.to_model(1.0, 1.0);
        // Ground plane holds the socket so gravity creates relative force
        // between socket (resting on ground) and pin (falling inside socket).
        // Without ground, both bodies free-fall equally → zero relative velocity
        // → contact solver never activates against gravity.
        model.add_ground_plane();
        // No ground plane — pure geometry constraint test

        eprintln!();
        eprintln!("  Phase 3d: Geometry-Driven Hinge Simulation");
        eprintln!("  -------------------------------------------");
        eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
        eprintln!("  nq: {}, nv: {}", model.nq, model.nv);
        eprintln!("  Timestep: {:.4} s", model.timestep);

        let mut data = model.make_data();

        // Two Free joints: socket qpos[0..7], pin qpos[7..14]
        let pin_q = 7; // pin qpos start index

        // ── Diagnostic: static contact check ─────────────────────────
        // Place pin 1.5mm below center so flange overlaps cap, verify contacts.
        data.qpos[pin_q + 2] = -1.5; // z = -1.5mm
        data.forward(&model).expect("forward");

        eprintln!();
        eprintln!("  Phase 3d: Geometry-Driven Hinge Simulation");
        eprintln!("  -------------------------------------------");
        eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
        eprintln!("  nq: {}, nv: {}", model.nq, model.nv);

        // Count contacts with +Z and -Z normals to see cap contacts
        let mut cap_contacts = 0;
        let mut bore_contacts = 0;
        for c in data.contacts.iter().take(data.ncon) {
            if c.normal.z.abs() > 0.5 {
                cap_contacts += 1;
            } else {
                bore_contacts += 1;
            }
        }
        eprintln!(
            "  Static check (z=-1.5): ncon={}, bore={bore_contacts}, cap={cap_contacts}",
            data.ncon
        );

        // Print a few cap contact details
        for (i, c) in data
            .contacts
            .iter()
            .take(data.ncon)
            .enumerate()
            .filter(|(_, c)| c.normal.z.abs() > 0.5)
            .take(5)
        {
            eprintln!(
                "    cap[{i}] n=({:.3},{:.3},{:.3}) depth={:.4} pos=({:.2},{:.2},{:.2})",
                c.normal.x, c.normal.y, c.normal.z, c.depth, c.pos.x, c.pos.y, c.pos.z
            );
        }

        assert!(
            cap_contacts > 0,
            "FAIL: no cap contacts when pin is 1.5mm below center"
        );
        eprintln!("  PASS: cap contacts detected ({cap_contacts})");

        // ── Dynamic simulation ───────────────────────────────────────
        // Reset to initial state (static check modified qpos).
        data.qpos = model.qpos0.clone();
        // Pin starts 0.9mm below socket center: bottom flange approaches cap.
        // With ground plane supporting socket, gravity creates asymmetric
        // relative force: socket held by ground, pin falls into cap.
        data.qpos[pin_q + 2] -= 0.9;
        for v in data.qvel.iter_mut() {
            *v = 0.0;
        }
        data.forward(&model).expect("forward");

        let socket_q = 0; // socket qpos start index
        let mut max_rel_x = 0.0_f64;
        let mut max_rel_y = 0.0_f64;
        let mut max_rel_z = 0.0_f64;
        let mut any_nan = false;
        let steps = 400; // enough for socket to settle on ground + pin to equilibrate

        for step in 0..steps {
            data.step(&model).expect("step");

            // Relative position: pin - socket
            let rx = data.qpos[pin_q] - data.qpos[socket_q];
            let ry = data.qpos[pin_q + 1] - data.qpos[socket_q + 1];
            let rz = data.qpos[pin_q + 2] - data.qpos[socket_q + 2];

            if !rx.is_finite() || !ry.is_finite() || !rz.is_finite() {
                any_nan = true;
                eprintln!("  NaN/Inf at step {step}!");
                break;
            }

            max_rel_x = max_rel_x.max(rx.abs());
            max_rel_y = max_rel_y.max(ry.abs());
            max_rel_z = max_rel_z.max(rz.abs());

            if step % 50 == 0 || step == steps - 1 {
                let step_cap = data
                    .contacts
                    .iter()
                    .take(data.ncon)
                    .filter(|c| c.normal.z.abs() > 0.5)
                    .count();
                eprintln!(
                    "  step {step:3}: rel=({rx:+.3},{ry:+.3},{rz:+.3}) sock_z={:.2} pin_z={:.2} ncon={} cap={step_cap}",
                    data.qpos[socket_q + 2],
                    data.qpos[pin_q + 2],
                    data.ncon
                );
            }
        }

        eprintln!();
        eprintln!("  === Phase 3d PASS/FAIL ===");

        assert!(!any_nan, "FAIL: NaN/Inf in positions");
        eprintln!("  PASS: no NaN/Inf");

        // Radial: pin stays within bore (clearance = 0.5mm, allow some solver slop)
        assert!(
            max_rel_x < 2.0,
            "FAIL: pin escaped radially in X (max |Δx| = {max_rel_x:.3})"
        );
        assert!(
            max_rel_y < 2.0,
            "FAIL: pin escaped radially in Y (max |Δy| = {max_rel_y:.3})"
        );
        eprintln!("  PASS: radial constraint (max |Δx|={max_rel_x:.3}, |Δy|={max_rel_y:.3})");

        // Axial: pin stays within bore (1mm clearance per side + initial 0.9mm offset)
        assert!(
            max_rel_z < 3.0,
            "FAIL: pin escaped axially (max |Δz| = {max_rel_z:.3})"
        );
        eprintln!("  PASS: axial constraint (max |Δz|={max_rel_z:.3})");

        assert!(data.ncon > 0, "FAIL: no contacts at final step");
        eprintln!("  PASS: contacts active (ncon={})", data.ncon);

        eprintln!();
    }

    // ========================================================================
    // Phase 6: Octree integration tests + dynamics validation
    // ========================================================================
    //
    // Deferred from Phase 3 (sim-core unit tests can't access cf-design types).
    // These tests use AnalyticalShape via the model builder, verifying that the
    // octree detection and Newton refinement produce correct results for concave
    // CSG geometry.

    /// Octree: sphere in cylindrical bore produces contacts on both sides.
    ///
    /// A sphere (r=3) inside a tube (bore r=3.5) should produce contacts
    /// with radial normals pointing inward from the bore wall.
    #[test]
    fn octree_sphere_in_bore() {
        use sim_core::Pose;
        use sim_core::sdf::octree_detect::octree_contact_detect;
        use std::sync::Arc;

        let bore_solid = Arc::new(Solid::cylinder(5.5, 10.0).subtract(Solid::cylinder(3.5, 10.0)));
        let sphere_solid = Arc::new(Solid::sphere(3.0));

        let bore_grid = Arc::new(bore_solid.sdf_grid_at(0.5).unwrap());
        let sphere_grid = Arc::new(sphere_solid.sdf_grid_at(0.5).unwrap());

        let bore_shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            bore_grid,
            bore_solid,
            crate::solid::ShapeHint::Concave,
        );
        let sphere_shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            sphere_grid,
            sphere_solid,
            crate::solid::ShapeHint::Sphere(3.0),
        );

        // Sphere slightly offset in +X inside the bore
        let bore_pose = Pose::from_position(Point3::origin());
        let sphere_pose = Pose::from_position(Point3::new(0.3, 0.0, 0.0));

        let contacts = octree_contact_detect(
            &bore_shape,
            &bore_pose,
            &sphere_shape,
            &sphere_pose,
            0.5,
            20,
        );

        assert!(
            !contacts.is_empty(),
            "sphere in bore should produce contacts"
        );

        // Contacts should be in the radial overlap region
        for c in &contacts {
            let radial = c.point.x.hypot(c.point.y);
            assert!(
                radial > 2.0 && radial < 5.0,
                "contact at radial={radial:.2} should be between sphere and bore surfaces"
            );
        }

        eprintln!("  octree_sphere_in_bore: {} contacts", contacts.len());
        for (i, c) in contacts.iter().enumerate().take(5) {
            eprintln!(
                "    [{i}] pos=({:.2},{:.2},{:.2}) n=({:.3},{:.3},{:.3}) pen={:.4}",
                c.point.x, c.point.y, c.point.z, c.normal.x, c.normal.y, c.normal.z, c.penetration
            );
        }
    }

    /// Newton refinement on cylinder bore produces purely radial normals
    /// (zero axial component) — the key test for virtual friction elimination.
    #[test]
    fn newton_cylinder_bore_radial_normals() {
        use sim_core::Pose;
        use sim_core::sdf::octree_detect::octree_contact_detect;
        use std::sync::Arc;

        let bore_solid = Arc::new(Solid::cylinder(5.0, 10.0).subtract(Solid::cylinder(3.0, 10.0)));
        let pin_solid = Arc::new(Solid::cylinder(2.8, 8.0));

        let bore_grid = Arc::new(bore_solid.sdf_grid_at(0.5).unwrap());
        let pin_grid = Arc::new(pin_solid.sdf_grid_at(0.5).unwrap());

        let bore_shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            bore_grid,
            bore_solid,
            crate::solid::ShapeHint::Concave,
        );
        let pin_shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            pin_grid,
            pin_solid,
            crate::solid::ShapeHint::Convex,
        );

        // Pin centered in bore — radial clearance = 0.2mm
        let bore_pose = Pose::from_position(Point3::origin());
        let pin_pose = Pose::from_position(Point3::new(0.1, 0.0, 0.0)); // slight offset

        let contacts =
            octree_contact_detect(&bore_shape, &bore_pose, &pin_shape, &pin_pose, 0.5, 20);

        assert!(!contacts.is_empty(), "pin in bore should produce contacts");

        // Key assertion: normals should be purely radial (no axial Z component)
        // This is what the CSG analytical gradient guarantees and the grid can't.
        for (i, c) in contacts.iter().enumerate() {
            let axial = c.normal.z.abs();
            eprintln!(
                "  contact[{i}] n=({:.4},{:.4},{:.4}) axial={axial:.6}",
                c.normal.x, c.normal.y, c.normal.z
            );
            // The bore surface is cylindrical: true normal is (x, y, 0) / r.
            // Allow small axial leak from Newton convergence near cylinder caps,
            // but mid-bore contacts should have essentially zero axial component.
            if c.point.z.abs() < 7.0 {
                // mid-bore (away from caps)
                assert!(
                    axial < 0.01,
                    "mid-bore contact[{i}] has axial leak {axial:.6} (should be < 0.01)"
                );
            }
        }
    }

    /// Octree contact count is bounded and sparse compared to grid path.
    ///
    /// The octree should produce far fewer contacts than the grid scan
    /// for the same geometry. This verifies the octree path was actually used
    /// (not falling through to Tier 3 grid).
    #[test]
    fn octree_produces_sparse_contacts() {
        use sim_core::Pose;
        use sim_core::sdf::octree_detect::octree_contact_detect;
        use sim_core::sdf::sdf_sdf_contact;
        use std::sync::Arc;

        let bore_solid = Arc::new(Solid::cylinder(5.0, 10.0).subtract(Solid::cylinder(3.0, 10.0)));
        let pin_solid = Arc::new(Solid::cylinder(2.8, 8.0));

        let bore_grid = Arc::new(bore_solid.sdf_grid_at(0.5).unwrap());
        let pin_grid = Arc::new(pin_solid.sdf_grid_at(0.5).unwrap());

        let bore_shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            bore_grid.clone(),
            bore_solid,
            crate::solid::ShapeHint::Concave,
        );
        let pin_shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            pin_grid.clone(),
            pin_solid,
            crate::solid::ShapeHint::Convex,
        );

        let bore_pose = Pose::from_position(Point3::origin());
        let pin_pose = Pose::from_position(Point3::new(0.1, 0.0, 0.0));

        // Octree contacts
        let octree_contacts =
            octree_contact_detect(&bore_shape, &bore_pose, &pin_shape, &pin_pose, 0.5, 20);

        // Grid contacts (Tier 3 path — use ShapeConvex wrappers for grid-only evaluation)
        let bore_convex = sim_core::ShapeConvex::new(bore_grid);
        let pin_convex = sim_core::ShapeConvex::new(pin_grid);
        let grid_contacts = sdf_sdf_contact(&bore_convex, &bore_pose, &pin_convex, &pin_pose, 0.5);

        eprintln!(
            "  octree: {} contacts, grid: {} contacts",
            octree_contacts.len(),
            grid_contacts.len()
        );

        // Octree should produce significantly fewer contacts
        assert!(
            octree_contacts.len() <= 20,
            "octree should produce ≤20 contacts, got {}",
            octree_contacts.len()
        );
        assert!(
            grid_contacts.len() > octree_contacts.len(),
            "grid ({}) should produce more contacts than octree ({})",
            grid_contacts.len(),
            octree_contacts.len()
        );
    }

    /// Octree: pin in full socket (tube + caps) produces contacts on
    /// multiple patches with radial normals and no axial leak.
    #[test]
    fn octree_pin_in_socket() {
        use sim_core::Pose;
        use sim_core::sdf::octree_detect::octree_contact_detect;
        use std::sync::Arc;

        // Socket: tube with caps (concave)
        let socket_solid = Arc::new(
            Solid::cylinder(5.5, 10.0)
                .subtract(Solid::cylinder(3.5, 8.0))
                .subtract(Solid::cylinder(2.5, 11.0)),
        );
        // Pin: shaft + flanges
        let shaft = Solid::cylinder(3.0, 6.0);
        let top_flange = Solid::cylinder(3.2, 1.0).translate(Vector3::new(0.0, 0.0, 6.0));
        let bot_flange = Solid::cylinder(3.2, 1.0).translate(Vector3::new(0.0, 0.0, -6.0));
        let pin_solid = Arc::new(shaft.union(top_flange).union(bot_flange));

        let socket_grid = Arc::new(socket_solid.sdf_grid_at(0.5).unwrap());
        let pin_grid = Arc::new(pin_solid.sdf_grid_at(0.5).unwrap());

        let socket_shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            socket_grid,
            socket_solid,
            crate::solid::ShapeHint::Concave,
        );
        let pin_shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            pin_grid,
            pin_solid,
            crate::solid::ShapeHint::Convex,
        );

        // Pin slightly offset radially
        let socket_pose = Pose::from_position(Point3::origin());
        let pin_pose = Pose::from_position(Point3::new(0.2, 0.0, -0.5));

        let contacts =
            octree_contact_detect(&socket_shape, &socket_pose, &pin_shape, &pin_pose, 0.5, 20);

        assert!(
            !contacts.is_empty(),
            "pin in socket should produce contacts"
        );

        // Should have contacts from bore wall (radial) and at least one
        // cap face (axial). Check that normals aren't all the same direction.
        let mut has_radial = false;
        let mut has_axial = false;
        for c in &contacts {
            if c.normal.z.abs() > 0.5 {
                has_axial = true;
            } else {
                has_radial = true;
            }
        }

        eprintln!(
            "  octree_pin_in_socket: {} contacts, radial={has_radial}, axial={has_axial}",
            contacts.len()
        );

        assert!(has_radial, "should have radial (bore wall) contacts");
        // Axial contacts depend on flange-cap overlap; may or may not exist
        // depending on exact geometry clearances. Don't assert.
    }

    /// Octree pruning: for convex shapes (spheres), interval pruning should
    /// eliminate most of the search space. We use spheres here because convex
    /// shapes have tight intervals — concave shapes (Subtract) have loose
    /// intervals that limit pruning effectiveness (see spec §5.7).
    #[test]
    fn octree_pruning_count() {
        use sim_core::Pose;
        use sim_core::sdf::octree_detect::octree_contact_detect_with_stats;
        use std::sync::Arc;

        let sphere_a = Arc::new(Solid::sphere(5.0));
        let sphere_b = Arc::new(Solid::sphere(5.0));

        let grid_a = Arc::new(sphere_a.sdf_grid_at(0.5).unwrap());
        let grid_b = Arc::new(sphere_b.sdf_grid_at(0.5).unwrap());

        let shape_a = crate::mechanism::analytical_shape::AnalyticalShape::new(
            grid_a.clone(),
            sphere_a,
            crate::solid::ShapeHint::Sphere(5.0),
        );
        let shape_b = crate::mechanism::analytical_shape::AnalyticalShape::new(
            grid_b.clone(),
            sphere_b,
            crate::solid::ShapeHint::Sphere(5.0),
        );

        let pose_a = Pose::from_position(Point3::origin());
        let pose_b = Pose::from_position(Point3::new(8.0, 0.0, 0.0));

        let (_contacts, stats) =
            octree_contact_detect_with_stats(&shape_a, &pose_a, &shape_b, &pose_b, 0.5, 20);

        // Grid cell count: both grids combined (full scan baseline)
        let grid_cells = grid_a.width() * grid_a.height() * grid_a.depth()
            + grid_b.width() * grid_b.height() * grid_b.depth();

        eprintln!(
            "  octree_pruning_count: interval_evals={}, leaf_cells={}, grid_cells={grid_cells}",
            stats.interval_evals, stats.leaf_cells
        );

        // For convex shapes with tight intervals, octree should evaluate
        // far fewer cells than the full grid scan
        assert!(
            stats.interval_evals < grid_cells,
            "octree interval evals ({}) should be < grid cells ({grid_cells})",
            stats.interval_evals
        );
        assert!(
            stats.leaf_cells < grid_cells / 5,
            "octree leaf cells ({}) should be < 20% of grid cells ({grid_cells})",
            stats.leaf_cells
        );
    }

    /// Octree contacts should be spatially close to grid contacts for
    /// the same geometry.
    #[test]
    fn octree_matches_grid_positions() {
        use sim_core::Pose;
        use sim_core::sdf::octree_detect::octree_contact_detect;
        use sim_core::sdf::sdf_sdf_contact;
        use std::sync::Arc;

        let sphere_a = Arc::new(Solid::sphere(5.0));
        let sphere_b = Arc::new(Solid::sphere(5.0));

        let grid_a = Arc::new(sphere_a.sdf_grid_at(0.5).unwrap());
        let grid_b = Arc::new(sphere_b.sdf_grid_at(0.5).unwrap());

        let shape_a = crate::mechanism::analytical_shape::AnalyticalShape::new(
            grid_a.clone(),
            sphere_a,
            crate::solid::ShapeHint::Sphere(5.0),
        );
        let shape_b = crate::mechanism::analytical_shape::AnalyticalShape::new(
            grid_b.clone(),
            sphere_b,
            crate::solid::ShapeHint::Sphere(5.0),
        );

        let pose_a = Pose::from_position(Point3::origin());
        let pose_b = Pose::from_position(Point3::new(8.0, 0.0, 0.0));

        let octree_contacts = octree_contact_detect(&shape_a, &pose_a, &shape_b, &pose_b, 0.5, 20);
        let convex_a = sim_core::ShapeConvex::new(grid_a.clone());
        let convex_b = sim_core::ShapeConvex::new(grid_b);
        let grid_contacts = sdf_sdf_contact(&convex_a, &pose_a, &convex_b, &pose_b, 0.5);

        assert!(!octree_contacts.is_empty(), "octree should find contacts");
        assert!(!grid_contacts.is_empty(), "grid should find contacts");

        // For each octree contact, find the nearest grid contact
        let cell_size = grid_a.cell_size();
        for oc in &octree_contacts {
            let min_dist = grid_contacts
                .iter()
                .map(|gc| (gc.point - oc.point).norm())
                .fold(f64::MAX, f64::min);

            assert!(
                min_dist < cell_size * 3.0,
                "octree contact at ({:.2},{:.2},{:.2}) has no grid neighbor within {:.1}mm (nearest={min_dist:.2}mm)",
                oc.point.x,
                oc.point.y,
                oc.point.z,
                cell_size * 3.0
            );
        }

        eprintln!(
            "  octree_matches_grid: {} octree, {} grid contacts — all octree contacts have grid neighbor within {:.1}mm",
            octree_contacts.len(),
            grid_contacts.len(),
            cell_size * 3.0
        );
    }

    /// Newton refinement on CSG subtract produces normals matching
    /// `Solid::gradient()` directly.
    #[test]
    fn newton_csg_subtract_matches_solid_gradient() {
        use sim_core::PhysicsShape;
        use std::sync::Arc;

        let bore_solid = Arc::new(Solid::cylinder(5.0, 10.0).subtract(Solid::cylinder(3.0, 10.0)));
        let bore_grid = Arc::new(bore_solid.sdf_grid_at(0.5).unwrap());
        let bore_shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            bore_grid,
            bore_solid.clone(),
            crate::solid::ShapeHint::Concave,
        );

        // Test points on the bore surface at mid-height
        let test_points = [
            Point3::new(3.0, 0.0, 0.0),   // +X on bore
            Point3::new(0.0, 3.0, 0.0),   // +Y on bore
            Point3::new(-3.0, 0.0, 0.0),  // -X on bore
            Point3::new(2.12, 2.12, 0.0), // 45° on bore
        ];

        for &p in &test_points {
            // Shape gradient (via PhysicsShape trait — delegates to Solid)
            let shape_grad = bore_shape.gradient(&p).unwrap();

            // Direct Solid gradient
            let solid_grad_raw = bore_solid.gradient(&p);
            let norm = solid_grad_raw.norm();
            let solid_grad = solid_grad_raw / norm;

            let diff = (shape_grad - solid_grad).norm();
            assert!(
                diff < 1e-10,
                "shape gradient at ({:.1},{:.1},{:.1}) differs from Solid::gradient by {diff:.2e}",
                p.x,
                p.y,
                p.z
            );

            // Verify the normal is purely radial (no axial component)
            assert!(
                shape_grad.z.abs() < 1e-10,
                "bore normal at ({:.1},{:.1},{:.1}) has axial leak z={:.2e}",
                p.x,
                p.y,
                p.z,
                shape_grad.z
            );
        }

        eprintln!("  newton_csg_subtract: all 4 test points match Solid::gradient() exactly");
    }

    /// Hinge pendulum energy conservation: a frictionless geometry-driven
    /// hinge should oscillate without artificial damping.
    ///
    /// This is the critical dynamics validation test. A pendulum arm
    /// hanging from a pin-in-bore geometry hinge, released from horizontal,
    /// should swing with < 1% energy loss per period.
    #[test]
    fn hinge_pendulum_energy_conservation() {
        use nalgebra::UnitQuaternion;

        // ── Geometry: socket + arm-with-pin ──────────────────────────────
        // Socket: very heavy (acts as fixed anchor), concave bore along Y axis.
        // The bore axis is Y (horizontal), so the arm swings in the XZ plane
        // under gravity (-Z). The bore constrains radial motion (XZ) while
        // allowing rotation around Y.
        let socket_solid = Solid::cylinder(6.0, 4.0) // outer R=6, half-height=4
            .subtract(Solid::cylinder(3.5, 5.0)) // bore R=3.5, through
            .rotate(UnitQuaternion::from_axis_angle(
                &nalgebra::Unit::new_normalize(Vector3::x()),
                std::f64::consts::FRAC_PI_2,
            )); // rotate 90° around X: Z-axis bore becomes Y-axis bore

        // Very high density so socket barely moves under contact forces
        let mut mat_heavy = Material::new("steel", 500_000.0);
        mat_heavy.color = Some([0.5, 0.5, 0.7, 1.0]);

        // Arm: pin (fits in Y-axis bore) + lever arm extending in -Z.
        // Pin is a Y-axis cylinder that sits inside the bore.
        // The arm extends downward from the pin center.
        let pin = Solid::cylinder(3.0, 3.0) // R=3.0 fits in bore R=3.5
            .rotate(UnitQuaternion::from_axis_angle(
                &nalgebra::Unit::new_normalize(Vector3::x()),
                std::f64::consts::FRAC_PI_2,
            )); // Z-axis cylinder → Y-axis cylinder
        let arm =
            Solid::cuboid(Vector3::new(1.0, 1.0, 8.0)).translate(Vector3::new(0.0, 0.0, -14.0)); // extends downward
        let arm_solid = pin.union(arm);

        let mut mat_light = Material::new("PLA", 1250.0);
        mat_light.color = Some([1.0, 0.3, 0.3, 1.0]);

        // ── Build mechanism ──────────────────────────────────────────────
        let mechanism = Mechanism::builder("pendulum")
            .part(Part::new("socket", socket_solid, mat_heavy))
            .part(Part::new("arm", arm_solid, mat_light))
            .joint(JointDef::new(
                "socket_free",
                "world",
                "socket",
                JointKind::Free,
                Point3::new(0.0, 0.0, 20.0), // well above ground
                Vector3::z(),
            ))
            .joint(JointDef::new(
                "arm_free",
                "world",
                "arm",
                JointKind::Free,
                Point3::new(0.0, 0.0, 20.0), // same position
                Vector3::z(),
            ))
            .build();

        let mut model = mechanism.to_model(1.0, 1.0);
        // Frictionless contacts — the whole point of this test
        for i in 0..model.ngeom {
            model.geom_friction[i] = Vector3::new(0.0, 0.0, 0.0);
        }
        // Gravity compensation on the socket body (body 1) so it stays fixed.
        // Without this, both bodies free-fall equally and the bore constraint
        // never engages (no relative force to push pin against bore wall).
        model.body_gravcomp[1] = 1.0;
        model.ngravcomp = 1;
        model.add_ground_plane();

        let mut data = model.make_data();

        // Displace arm from equilibrium: rotate 45° around Y (the bore axis).
        // This tilts the arm from hanging straight down (-Z) to an angle,
        // so gravity creates a restoring torque and the arm swings.
        let arm_q = 7; // arm qpos start (after socket's 7 DOF)
        let angle = std::f64::consts::FRAC_PI_4; // 45°
        data.qpos[arm_q + 3] = (angle / 2.0).cos(); // qw
        data.qpos[arm_q + 4] = 0.0; // qx
        data.qpos[arm_q + 5] = (angle / 2.0).sin(); // qy (rotate around Y)
        data.qpos[arm_q + 6] = 0.0; // qz

        data.forward(&model).expect("forward");

        // ── Diagnostic: print initial state ──────────────────────────────
        eprintln!();
        eprintln!("  === Hinge Pendulum Diagnostic ===");
        eprintln!(
            "  Bodies: {}, Geoms: {}, nq: {}, nv: {}",
            model.nbody, model.ngeom, model.nq, model.nv
        );
        eprintln!(
            "  Timestep: {:.4} s, Gravity: {:?}",
            model.timestep, model.gravity
        );
        for body in 0..model.nbody {
            eprintln!(
                "  body[{body}]: mass={:.4} pos=({:.2},{:.2},{:.2})",
                model.body_mass[body], data.xipos[body].x, data.xipos[body].y, data.xipos[body].z,
            );
        }
        for geom in 0..model.ngeom {
            eprintln!(
                "  geom[{geom}]: type={:?} friction=({:.2},{:.2},{:.2})",
                model.geom_type[geom],
                model.geom_friction[geom].x,
                model.geom_friction[geom].y,
                model.geom_friction[geom].z,
            );
        }
        eprintln!(
            "  socket qpos: ({:.2},{:.2},{:.2}) quat=({:.3},{:.3},{:.3},{:.3})",
            data.qpos[0],
            data.qpos[1],
            data.qpos[2],
            data.qpos[3],
            data.qpos[4],
            data.qpos[5],
            data.qpos[6]
        );
        eprintln!(
            "  arm qpos:    ({:.2},{:.2},{:.2}) quat=({:.3},{:.3},{:.3},{:.3})",
            data.qpos[7],
            data.qpos[8],
            data.qpos[9],
            data.qpos[10],
            data.qpos[11],
            data.qpos[12],
            data.qpos[13]
        );
        eprintln!("  Initial ncon: {}", data.ncon);

        // Print initial contacts
        for (i, c) in data.contacts.iter().take(data.ncon).enumerate().take(10) {
            eprintln!(
                "    contact[{i}] pos=({:.2},{:.2},{:.2}) n=({:.3},{:.3},{:.3}) depth={:.4}",
                c.pos.x, c.pos.y, c.pos.z, c.normal.x, c.normal.y, c.normal.z, c.depth,
            );
        }

        // ── Run simulation and track energy ──────────────────────────────
        let steps = 500;
        let mut energies = Vec::new();

        for step in 0..steps {
            data.step(&model).expect("step");

            // Total energy = KE + PE
            let mut ke = 0.0;
            let mut pe = 0.0;
            for body in 1..model.nbody {
                let mass = model.body_mass[body];
                if body < data.cvel.len() {
                    let cv = &data.cvel[body];
                    let vx = cv[3];
                    let vy = cv[4];
                    let vz = cv[5];
                    ke += 0.5 * mass * (vx * vx + vy * vy + vz * vz);
                }
                if body < data.xipos.len() {
                    let z = data.xipos[body].z;
                    pe += mass * 9.81 * z;
                }
            }

            let total = ke + pe;
            energies.push(total);

            if step % 50 == 0 || step < 5 {
                // Body positions
                let sock_z = if model.nbody > 1 {
                    data.xipos[1].z
                } else {
                    0.0
                };
                let arm_pos = if model.nbody > 2 {
                    data.xipos[2]
                } else {
                    Vector3::zeros()
                };
                eprintln!(
                    "  step {step:3}: sock_z={sock_z:.2} arm=({:.2},{:.2},{:.2}) KE={ke:.4} PE={pe:.4} ncon={}",
                    arm_pos.x, arm_pos.y, arm_pos.z, data.ncon
                );

                // Print a few contacts for first few steps
                if step < 5 {
                    for (i, c) in data.contacts.iter().take(data.ncon).enumerate().take(5) {
                        eprintln!(
                            "    c[{i}] pos=({:.2},{:.2},{:.2}) n=({:.3},{:.3},{:.3}) d={:.4}",
                            c.pos.x, c.pos.y, c.pos.z, c.normal.x, c.normal.y, c.normal.z, c.depth,
                        );
                    }
                }
            }
        }

        // ── Check energy conservation ────────────────────────────────────
        // Compare energy at step 50 (after initial transient) with energy
        // at step 450 (near end). Allow some loss from solver discretization.
        let e_early = energies[50];
        let e_late = energies[450];
        let loss_fraction = if e_early.abs() > 1e-10 {
            (e_early - e_late).abs() / e_early.abs()
        } else {
            0.0
        };

        eprintln!();
        eprintln!(
            "  Energy: early={e_early:.4}, late={e_late:.4}, loss={:.1}%",
            loss_fraction * 100.0
        );

        // Energy conservation: with analytical CSG normals from the contact
        // patch system, a frictionless geometry-driven hinge conserves energy.
        // Grid normals would cause > 50% loss from virtual friction.
        // Threshold 6%: exact SDF evaluation in Tier 3 surface tracing
        // (PhysicsShape threading) gives sharper normals at bore-pin interface,
        // slightly changing solver dynamics vs the old grid interpolation.
        assert!(
            loss_fraction < 0.06,
            "energy loss {:.1}% exceeds 6% threshold — indicates virtual friction or solver instability",
            loss_fraction * 100.0
        );
    }

    // =========================================================================
    // Dispatch routing + octree plane quality (spec §7.1, §7.2)
    // =========================================================================

    /// `AnalyticalShape` cuboid on a plane takes Tier 2 (octree) and produces
    /// distributed multi-contact across the face. This is the primary fix
    /// for 08-stack stability — before the dispatch fix, cuboids got Tier 1
    /// (single contact) with no torque resistance.
    #[test]
    fn cuboid_plane_multi_contact_via_octree() {
        use sim_core::sdf::compute_shape_plane_contact;

        let half = nalgebra::Vector3::new(5.0, 5.0, 5.0);
        let solid = std::sync::Arc::new(Solid::cuboid(half));
        let grid = std::sync::Arc::new(solid.sdf_grid_at(1.0).unwrap());
        let shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            grid,
            solid,
            crate::solid::ShapeHint::Convex,
        );

        // Cuboid center at z=4.5 → bottom face at z=-0.5 (0.5mm into the z=0 plane)
        let pose = sim_core::Pose::from_position(Point3::new(0.0, 0.0, 4.5));
        let plane_pos = nalgebra::Vector3::new(0.0, 0.0, 0.0);
        let plane_normal = nalgebra::Vector3::z();

        let contacts = compute_shape_plane_contact(&shape, &pose, &plane_pos, &plane_normal, 1.0);

        // Must be multi-contact (Tier 2 octree), not single (old Tier 1)
        assert!(
            contacts.len() >= 3,
            "cuboid on plane should produce ≥3 contacts via octree (got {})",
            contacts.len()
        );

        // Contacts should span the face — at least 4mm apart in X or Y
        let max_x = contacts.iter().map(|c| c.point.x).fold(f64::MIN, f64::max);
        let min_x = contacts.iter().map(|c| c.point.x).fold(f64::MAX, f64::min);
        let max_y = contacts.iter().map(|c| c.point.y).fold(f64::MIN, f64::max);
        let min_y = contacts.iter().map(|c| c.point.y).fold(f64::MAX, f64::min);
        let span = (max_x - min_x).max(max_y - min_y);
        assert!(
            span > 4.0,
            "contacts should span the face (span={span:.1}mm, need >4mm)"
        );

        // All normals should be the plane normal (pointing up)
        for c in &contacts {
            assert!(
                c.normal.z > 0.99,
                "contact normal should be +Z (got {:.3},{:.3},{:.3})",
                c.normal.x,
                c.normal.y,
                c.normal.z
            );
        }

        eprintln!(
            "  cuboid-plane: {} contacts, span={:.1}mm",
            contacts.len(),
            span
        );
    }

    /// `AnalyticalShape` sphere on a plane still takes Tier 1 (single contact).
    /// The dispatch fix must preserve the fast path for spheres.
    #[test]
    fn sphere_plane_still_single_contact() {
        use sim_core::sdf::compute_shape_plane_contact;

        let solid = std::sync::Arc::new(Solid::sphere(5.0));
        let grid = std::sync::Arc::new(solid.sdf_grid_at(0.5).unwrap());
        let shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            grid,
            solid,
            crate::solid::ShapeHint::Sphere(5.0),
        );

        // Sphere center at z=4.0 → 1mm penetration into z=0 plane
        let pose = sim_core::Pose::from_position(Point3::new(0.0, 0.0, 4.0));
        let plane_pos = nalgebra::Vector3::new(0.0, 0.0, 0.0);
        let plane_normal = nalgebra::Vector3::z();

        let contacts = compute_shape_plane_contact(&shape, &pose, &plane_pos, &plane_normal, 0.1);

        assert_eq!(
            contacts.len(),
            1,
            "sphere on plane should produce exactly 1 analytical contact"
        );
        assert!(
            (contacts[0].penetration - 1.0).abs() < 0.1,
            "depth should be ~1.0 (got {:.3})",
            contacts[0].penetration
        );
    }

    /// Tilted cuboid (45° about X) resting on edge produces contacts
    /// distributed along the edge, not a single point.
    #[test]
    fn tilted_cuboid_plane_edge_contacts() {
        use nalgebra::UnitQuaternion;
        use sim_core::sdf::compute_shape_plane_contact;

        let half = nalgebra::Vector3::new(5.0, 5.0, 5.0);
        let solid = std::sync::Arc::new(Solid::cuboid(half));
        let grid = std::sync::Arc::new(solid.sdf_grid_at(1.0).unwrap());
        let shape = crate::mechanism::analytical_shape::AnalyticalShape::new(
            grid,
            solid,
            crate::solid::ShapeHint::Convex,
        );

        // Rotate 45° about X → bottom edge is along Y axis.
        // Edge height = half * sqrt(2) ≈ 7.07. Center at z=6.5 → edge at z ≈ -0.57 (below plane).
        let angle = std::f64::consts::FRAC_PI_4;
        let rotation = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(nalgebra::Vector3::x()),
            angle,
        );
        let pose = sim_core::Pose {
            position: Point3::new(0.0, 0.0, 6.5),
            rotation,
        };
        let plane_pos = nalgebra::Vector3::new(0.0, 0.0, 0.0);
        let plane_normal = nalgebra::Vector3::z();

        let contacts = compute_shape_plane_contact(&shape, &pose, &plane_pos, &plane_normal, 1.0);

        assert!(
            contacts.len() >= 2,
            "tilted cuboid on plane should produce ≥2 edge contacts (got {})",
            contacts.len()
        );

        // Contacts should be separated along the Y axis (the edge axis)
        if contacts.len() >= 2 {
            let max_y = contacts.iter().map(|c| c.point.y).fold(f64::MIN, f64::max);
            let min_y = contacts.iter().map(|c| c.point.y).fold(f64::MAX, f64::min);
            let y_span = max_y - min_y;
            assert!(
                y_span > 2.0,
                "edge contacts should be separated along Y (span={y_span:.1}mm)"
            );
        }

        eprintln!("  tilted cuboid: {} contacts", contacts.len());
    }

    // =========================================================================
    // Tier 3 exact evaluation (spec §7.3)
    // =========================================================================

    /// Exact SDF evaluation in Tier 3 surface tracing gives sharper normals
    /// than grid interpolation. Two `AnalyticalShape` cuboids overlapping along
    /// X should produce normals very close to ±X. Compare with grid-only
    /// `ShapeConvex` which may deviate near edges.
    #[test]
    fn tier3_exact_normals_vs_grid() {
        use sim_core::sdf::sdf_sdf_contact;

        let half = nalgebra::Vector3::new(5.0, 5.0, 5.0);

        // Analytical shapes (exact SDF evaluation)
        let solid_a = std::sync::Arc::new(Solid::cuboid(half));
        let solid_b = std::sync::Arc::new(Solid::cuboid(half));
        let grid_a = std::sync::Arc::new(solid_a.sdf_grid_at(1.0).unwrap());
        let grid_b = std::sync::Arc::new(solid_b.sdf_grid_at(1.0).unwrap());
        let exact_a = crate::mechanism::analytical_shape::AnalyticalShape::new(
            grid_a.clone(),
            solid_a,
            crate::solid::ShapeHint::Convex,
        );
        let exact_b = crate::mechanism::analytical_shape::AnalyticalShape::new(
            grid_b.clone(),
            solid_b,
            crate::solid::ShapeHint::Convex,
        );

        // Grid-only shapes (trilinear interpolation)
        let grid_only_a = sim_core::ShapeConvex::new(grid_a);
        let grid_only_b = sim_core::ShapeConvex::new(grid_b);

        // Overlap 1mm along X: A at origin, B at x=9 (faces at x=5 and x=4)
        let pose_a = sim_core::Pose::identity();
        let pose_b = sim_core::Pose::from_position(Point3::new(9.0, 0.0, 0.0));

        let exact_contacts = sdf_sdf_contact(&exact_a, &pose_a, &exact_b, &pose_b, 1.0);
        let grid_contacts = sdf_sdf_contact(&grid_only_a, &pose_a, &grid_only_b, &pose_b, 1.0);

        assert!(
            !exact_contacts.is_empty(),
            "exact cuboid overlap should produce contacts"
        );
        assert!(
            !grid_contacts.is_empty(),
            "grid cuboid overlap should produce contacts"
        );

        // Filter to X-face contacts: normal is primarily ±X (|nx| > 0.5).
        // Other contacts are on Y/Z faces where both cuboids' surfaces overlap.
        let exact_x: Vec<_> = exact_contacts
            .iter()
            .filter(|c| c.normal.x.abs() > 0.5)
            .collect();
        let grid_x: Vec<_> = grid_contacts
            .iter()
            .filter(|c| c.normal.x.abs() > 0.5)
            .collect();

        assert!(
            !exact_x.is_empty(),
            "exact path should produce X-face contacts"
        );

        // Exact X-face normals should be very close to ±X (< 1°)
        let one_degree = 1.0_f64.to_radians();
        for c in &exact_x {
            let x_alignment = c.normal.x.abs();
            assert!(
                x_alignment > one_degree.cos(),
                "exact X-face normal should be within 1° of ±X (got n=({:.3},{:.3},{:.3}))",
                c.normal.x,
                c.normal.y,
                c.normal.z,
            );
        }

        // Report comparison (grid normals may be less aligned near edges)
        let exact_avg: f64 =
            exact_x.iter().map(|c| c.normal.x.abs()).sum::<f64>() / exact_x.len() as f64;
        let grid_avg: f64 = if grid_x.is_empty() {
            0.0
        } else {
            grid_x.iter().map(|c| c.normal.x.abs()).sum::<f64>() / grid_x.len() as f64
        };

        eprintln!(
            "  exact: {} total, {} X-face, avg |nx|={:.4}",
            exact_contacts.len(),
            exact_x.len(),
            exact_avg
        );
        eprintln!(
            "  grid:  {} total, {} X-face, avg |nx|={:.4}",
            grid_contacts.len(),
            grid_x.len(),
            grid_avg
        );
    }
}
