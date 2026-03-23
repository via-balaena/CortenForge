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
    use sim_core::{GeomType, ShapeSphere};

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

    /// Diagnostic: compare cf-design model vs MJCF model for the same sphere pair.
    ///
    /// Findings so far:
    /// - `diagapprox_bodyweight` was false (fixed in generate) — improved settling
    /// - Mass/inertia differ by 1-3.6% (SDF integration vs analytical) — not sole cause
    /// - SDF grid, geom ordering, mesh geoms — all ruled out individually
    /// - A remaining Model field diff causes late instability (~step 1400)
    /// - Next step: serialization-based exhaustive Model field diff
    #[test]
    #[allow(
        clippy::similar_names,
        clippy::unreadable_literal,
        unused_variables,
        clippy::too_many_lines,
        clippy::cognitive_complexity,
        clippy::redundant_clone,
        clippy::cast_lossless,
        clippy::clone_on_copy,
        clippy::or_fun_call
    )]
    fn diagnose_sdf_sphere_gap() {
        use sim_core::SdfGrid;
        use std::sync::Arc;

        // ── Step 1: Compare SDF grids ──────────────────────────────────
        let conformance_sdf = SdfGrid::sphere(Point3::origin(), 5.0, 20, 2.0);
        let cfdesign_sdf = Solid::sphere(5.0).sdf_grid_at(1.0).unwrap();

        eprintln!("=== Step 1: SDF Grid Comparison ===");
        eprintln!(
            "  conformance: cell_size={:.4} dims={}x{}x{} origin=({:.2},{:.2},{:.2})",
            conformance_sdf.cell_size(),
            conformance_sdf.width(),
            conformance_sdf.height(),
            conformance_sdf.depth(),
            conformance_sdf.origin().x,
            conformance_sdf.origin().y,
            conformance_sdf.origin().z,
        );
        eprintln!(
            "  cf-design:   cell_size={:.4} dims={}x{}x{} origin=({:.2},{:.2},{:.2})",
            cfdesign_sdf.cell_size(),
            cfdesign_sdf.width(),
            cfdesign_sdf.height(),
            cfdesign_sdf.depth(),
            cfdesign_sdf.origin().x,
            cfdesign_sdf.origin().y,
            cfdesign_sdf.origin().z,
        );

        // Ray-march radius along Z (the stacking axis)
        let ray_radius = |sdf: &SdfGrid| -> f64 {
            let dir = nalgebra::Vector3::z();
            let mut t = 0.0_f64;
            for _ in 0..200 {
                let point = Point3::from(dir * t);
                let Some(dist) = sdf.distance(point) else {
                    return t;
                };
                if dist >= 0.0 {
                    return t;
                }
                t += (-dist).max(sdf.cell_size() * 0.01);
            }
            t
        };
        let r_conf = ray_radius(&conformance_sdf);
        let r_cfd = ray_radius(&cfdesign_sdf);
        eprintln!(
            "  ray_radius_z: conformance={:.4} cf-design={:.4} delta={:.4}",
            r_conf,
            r_cfd,
            (r_conf - r_cfd).abs()
        );

        // ── Step 2: Build both models and diff fields ──────────────────
        eprintln!("\n=== Step 2: Model Field Comparison ===");

        // MJCF model (conformance path)
        let mjcf = r#"
            <mujoco model="diag_mjcf">
                <option gravity="0 0 -9810" timestep="0.0005"/>
                <worldbody>
                    <geom name="floor" type="plane" size="40 40 0.1"/>
                    <body name="lower" pos="0 0 5.5">
                        <joint type="free"/>
                        <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                        <geom name="lo_sphere" type="sphere" size="5"/>
                    </body>
                    <body name="upper" pos="0 0 15.5">
                        <joint type="free"/>
                        <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                        <geom name="up_sphere" type="sphere" size="5"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;
        let mut mjcf_model = sim_mjcf::load_model(mjcf).expect("load MJCF");
        // Swap sphere geoms to SDF (same as conformance test)
        let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 20, 2.0));
        mjcf_model
            .shape_data
            .push(Arc::new(ShapeSphere::new(grid.clone(), 5.0)));
        mjcf_model
            .shape_data
            .push(Arc::new(ShapeSphere::new(grid, 5.0)));
        mjcf_model.nshape = 2;
        for geom_id in 1..=2 {
            mjcf_model.geom_type[geom_id] = GeomType::Sdf;
            mjcf_model.geom_shape[geom_id] = Some(geom_id - 1);
        }

        // cf-design model
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
        let mut cfd_model = m.to_model(1.0, 0.3);
        cfd_model.add_ground_plane();

        // Compare key fields for bodies 1 and 2 (the spheres)
        eprintln!("  {:>25} {:>15} {:>15}", "field", "mjcf", "cf-design");
        eprintln!("  {:-<25} {:-<15} {:-<15}", "", "", "");

        eprintln!(
            "  nbody:                  {:>15} {:>15}",
            mjcf_model.nbody, cfd_model.nbody
        );
        eprintln!(
            "  ngeom:                  {:>15} {:>15}",
            mjcf_model.ngeom, cfd_model.ngeom
        );
        eprintln!(
            "  njnt:                   {:>15} {:>15}",
            mjcf_model.njnt, cfd_model.njnt
        );
        eprintln!(
            "  nq:                     {:>15} {:>15}",
            mjcf_model.nq, cfd_model.nq
        );
        eprintln!(
            "  nv:                     {:>15} {:>15}",
            mjcf_model.nv, cfd_model.nv
        );
        eprintln!(
            "  timestep:               {:>15.6} {:>15.6}",
            mjcf_model.timestep, cfd_model.timestep
        );
        eprintln!(
            "  gravity_z:              {:>15.1} {:>15.1}",
            mjcf_model.gravity.z, cfd_model.gravity.z
        );

        for body_id in 1..3 {
            let bname_m = mjcf_model
                .body_name
                .get(body_id)
                .and_then(|n| n.as_deref())
                .unwrap_or("?");
            let bname_c = cfd_model
                .body_name
                .get(body_id)
                .and_then(|n| n.as_deref())
                .unwrap_or("?");
            eprintln!(
                "\n  --- body {} (mjcf='{}' cfd='{}') ---",
                body_id, bname_m, bname_c
            );
            eprintln!(
                "  body_mass:              {:>15.6} {:>15.6}",
                mjcf_model.body_mass[body_id], cfd_model.body_mass[body_id]
            );
            eprintln!(
                "  body_inertia:           {:>15.6} {:>15.6}",
                mjcf_model.body_inertia[body_id].x, cfd_model.body_inertia[body_id].x
            );
            eprintln!(
                "  body_pos:               ({:.3},{:.3},{:.3})  ({:.3},{:.3},{:.3})",
                mjcf_model.body_pos[body_id].x,
                mjcf_model.body_pos[body_id].y,
                mjcf_model.body_pos[body_id].z,
                cfd_model.body_pos[body_id].x,
                cfd_model.body_pos[body_id].y,
                cfd_model.body_pos[body_id].z
            );
            eprintln!(
                "  body_ipos:              ({:.4},{:.4},{:.4})  ({:.4},{:.4},{:.4})",
                mjcf_model.body_ipos[body_id].x,
                mjcf_model.body_ipos[body_id].y,
                mjcf_model.body_ipos[body_id].z,
                cfd_model.body_ipos[body_id].x,
                cfd_model.body_ipos[body_id].y,
                cfd_model.body_ipos[body_id].z
            );
            eprintln!(
                "  body_invweight0:        [{:.6},{:.6}]  [{:.6},{:.6}]",
                mjcf_model.body_invweight0[body_id][0],
                mjcf_model.body_invweight0[body_id][1],
                cfd_model.body_invweight0[body_id][0],
                cfd_model.body_invweight0[body_id][1]
            );
            eprintln!(
                "  body_subtreemass:       {:>15.6} {:>15.6}",
                mjcf_model.body_subtreemass[body_id], cfd_model.body_subtreemass[body_id]
            );
            eprintln!(
                "  body_parent:            {:>15} {:>15}",
                mjcf_model.body_parent[body_id], cfd_model.body_parent[body_id]
            );
        }

        // Compare geom fields (SDF geoms only)
        eprintln!("\n  === Geom comparison (SDF geoms) ===");
        let mjcf_sdf_geoms: Vec<usize> = (0..mjcf_model.ngeom)
            .filter(|&g| mjcf_model.geom_type[g] == GeomType::Sdf)
            .collect();
        let cfd_sdf_geoms: Vec<usize> = (0..cfd_model.ngeom)
            .filter(|&g| cfd_model.geom_type[g] == GeomType::Sdf)
            .collect();
        eprintln!(
            "  SDF geom count: mjcf={} cfd={}",
            mjcf_sdf_geoms.len(),
            cfd_sdf_geoms.len()
        );

        for (i, (&mg, &cg)) in mjcf_sdf_geoms.iter().zip(cfd_sdf_geoms.iter()).enumerate() {
            eprintln!("\n  SDF geom pair {} (mjcf={} cfd={}):", i, mg, cg);
            eprintln!(
                "    geom_body:     {} vs {}",
                mjcf_model.geom_body[mg], cfd_model.geom_body[cg]
            );
            eprintln!(
                "    geom_pos:      ({:.3},{:.3},{:.3}) vs ({:.3},{:.3},{:.3})",
                mjcf_model.geom_pos[mg].x,
                mjcf_model.geom_pos[mg].y,
                mjcf_model.geom_pos[mg].z,
                cfd_model.geom_pos[cg].x,
                cfd_model.geom_pos[cg].y,
                cfd_model.geom_pos[cg].z
            );
            eprintln!(
                "    geom_size:     ({:.3},{:.3},{:.3}) vs ({:.3},{:.3},{:.3})",
                mjcf_model.geom_size[mg].x,
                mjcf_model.geom_size[mg].y,
                mjcf_model.geom_size[mg].z,
                cfd_model.geom_size[cg].x,
                cfd_model.geom_size[cg].y,
                cfd_model.geom_size[cg].z
            );
            eprintln!(
                "    geom_margin:   {:.4} vs {:.4}",
                mjcf_model.geom_margin[mg], cfd_model.geom_margin[cg]
            );
            eprintln!(
                "    geom_solref:   {:?} vs {:?}",
                mjcf_model.geom_solref[mg], cfd_model.geom_solref[cg]
            );
            eprintln!(
                "    geom_solimp:   {:?} vs {:?}",
                mjcf_model.geom_solimp[mg], cfd_model.geom_solimp[cg]
            );
            eprintln!(
                "    geom_friction: ({:.4},{:.4},{:.4}) vs ({:.4},{:.4},{:.4})",
                mjcf_model.geom_friction[mg].x,
                mjcf_model.geom_friction[mg].y,
                mjcf_model.geom_friction[mg].z,
                cfd_model.geom_friction[cg].x,
                cfd_model.geom_friction[cg].y,
                cfd_model.geom_friction[cg].z
            );
            eprintln!(
                "    geom_condim:   {} vs {}",
                mjcf_model.geom_condim[mg], cfd_model.geom_condim[cg]
            );
            eprintln!(
                "    geom_contype:  {} vs {}",
                mjcf_model.geom_contype[mg], cfd_model.geom_contype[cg]
            );
            eprintln!(
                "    geom_rbound:   {:.4} vs {:.4}",
                mjcf_model.geom_rbound[mg], cfd_model.geom_rbound[cg]
            );
        }

        // Compare initial qpos
        eprintln!("\n  === Initial qpos ===");
        let nq = mjcf_model.nq.min(cfd_model.nq);
        for i in 0..nq {
            let mq = mjcf_model.qpos0[i];
            let cq = cfd_model.qpos0[i];
            if (mq - cq).abs() > 1e-10 {
                eprintln!(
                    "  qpos0[{}]: mjcf={:.6} cfd={:.6} DIFF={:.6}",
                    i,
                    mq,
                    cq,
                    mq - cq
                );
            }
        }

        // Step 2b: Clone the MJCF model exactly — rebuild it from scratch
        // using the same approach as the conformance test. This isolates
        // whether the issue is in Model construction or something else.
        let mut cfd_patched = mjcf_model.clone();

        // Only difference: use cf-design's SDF grids instead of conformance
        let cfd_sdf_grid = Arc::new(Solid::sphere(5.0).sdf_grid_at(1.0).unwrap());
        cfd_patched.shape_data = vec![
            Arc::new(ShapeSphere::new(cfd_sdf_grid.clone(), 5.0)),
            Arc::new(ShapeSphere::new(cfd_sdf_grid, 5.0)),
        ];

        // Test: add dummy mesh geoms to MJCF model (same structure as cf-design)
        // If this breaks stability, the mesh geoms interfere despite contype=0.
        let mut mjcf_with_mesh = mjcf_model.clone();
        // Insert mesh geom after each SDF geom (to match cf-design layout)
        // For simplicity, just add 2 mesh geoms at the end with contype=0
        for _ in 0..2 {
            mjcf_with_mesh.geom_type.push(GeomType::Mesh);
            mjcf_with_mesh.geom_body.push(1);
            mjcf_with_mesh.geom_pos.push(Vector3::zeros());
            mjcf_with_mesh
                .geom_quat
                .push(nalgebra::UnitQuaternion::identity());
            mjcf_with_mesh.geom_size.push(Vector3::new(1.0, 1.0, 1.0));
            mjcf_with_mesh
                .geom_friction
                .push(Vector3::new(1.0, 0.005, 0.0001));
            mjcf_with_mesh.geom_condim.push(3);
            mjcf_with_mesh.geom_contype.push(0);
            mjcf_with_mesh.geom_conaffinity.push(0);
            mjcf_with_mesh.geom_margin.push(0.0);
            mjcf_with_mesh.geom_gap.push(0.0);
            mjcf_with_mesh.geom_priority.push(0);
            mjcf_with_mesh.geom_solmix.push(1.0);
            mjcf_with_mesh
                .geom_solimp
                .push([0.9, 0.95, 0.001, 0.5, 2.0]);
            mjcf_with_mesh.geom_solref.push([0.02, 1.0]);
            mjcf_with_mesh.geom_name.push(None);
            mjcf_with_mesh.geom_rbound.push(10.0);
            mjcf_with_mesh.geom_aabb.push([0.0; 6]);
            mjcf_with_mesh.geom_mesh.push(None);
            mjcf_with_mesh.geom_hfield.push(None);
            mjcf_with_mesh.geom_shape.push(None);
            mjcf_with_mesh.geom_group.push(0);
            mjcf_with_mesh.geom_rgba.push([0.9, 0.9, 0.9, 1.0]);
            mjcf_with_mesh.geom_fluid.push([0.0; 12]);
            mjcf_with_mesh.geom_plugin.push(None);
            mjcf_with_mesh.geom_user.push(vec![]);
        }
        mjcf_with_mesh.ngeom += 2;
        mjcf_with_mesh.body_geom_num[1] += 1;
        mjcf_with_mesh.body_geom_num[2] += 1;

        let mut mesh_data = mjcf_with_mesh.make_data();
        mesh_data.forward(&mjcf_with_mesh).unwrap();
        for step in 0..2000 {
            mesh_data.step(&mjcf_with_mesh).unwrap();
        }
        eprintln!(
            "  MJCF+mesh: z_lo={:.3} z_up={:.3} gap={:.3} ncon={}",
            mesh_data.qpos[2],
            mesh_data.qpos[9],
            mesh_data.qpos[9] - mesh_data.qpos[2],
            mesh_data.ncon
        );

        eprintln!("\n  === Patched: MJCF clone + cf-design SDF grid (STABLE) ===");
        eprintln!("  ngeom:        {}", cfd_patched.ngeom);

        // Now build the REAL cf-design model with same SDF + mass as MJCF
        // and diff EVERY field to find what cf-design sets wrong.
        let mut cfd_full = m.to_model(1.0, 0.3);
        cfd_full.add_ground_plane();
        // Patch to match MJCF exactly (mass, inertia, SDF)
        let cfd_sdf2 = Arc::new(Solid::sphere(5.0).sdf_grid_at(1.0).unwrap());
        cfd_full.shape_data = vec![
            Arc::new(ShapeSphere::new(cfd_sdf2.clone(), 5.0)),
            Arc::new(ShapeSphere::new(cfd_sdf2, 5.0)),
        ];
        for body_id in 1..cfd_full.nbody {
            cfd_full.body_mass[body_id] = 0.000655;
            cfd_full.body_inertia[body_id] = Vector3::new(0.00655, 0.00655, 0.00655);
        }
        cfd_full.compute_implicit_params();
        cfd_full.compute_invweight0();

        // Both have same mass, inertia, SDF, invweight0.
        // Diff everything else.
        eprintln!("\n  === Field-by-field diff: MJCF-clone(stable) vs cfd_full(unstable) ===");
        eprintln!("  ngeom: {} vs {}", cfd_patched.ngeom, cfd_full.ngeom);
        eprintln!("  nbody: {} vs {}", cfd_patched.nbody, cfd_full.nbody);

        // Geom-level diffs (all geoms)
        let max_geom = cfd_patched.ngeom.max(cfd_full.ngeom);
        for g in 0..max_geom {
            let in_p = g < cfd_patched.ngeom;
            let in_c = g < cfd_full.ngeom;
            if in_p && in_c {
                let tp = cfd_patched.geom_type[g];
                let tc = cfd_full.geom_type[g];
                if tp != tc {
                    eprintln!("  geom[{}] type: {:?} vs {:?}", g, tp, tc);
                }
                let bp = cfd_patched.geom_body[g];
                let bc = cfd_full.geom_body[g];
                if bp != bc {
                    eprintln!("  geom[{}] body: {} vs {}", g, bp, bc);
                }
                let ct_p = cfd_patched.geom_contype[g];
                let ct_c = cfd_full.geom_contype[g];
                let ca_p = cfd_patched.geom_conaffinity[g];
                let ca_c = cfd_full.geom_conaffinity[g];
                if ct_p != ct_c || ca_p != ca_c {
                    eprintln!(
                        "  geom[{}] contype/aff: {}/{} vs {}/{}",
                        g, ct_p, ca_p, ct_c, ca_c
                    );
                }
            } else {
                let which = if in_p { "patched-only" } else { "cfd-only" };
                let model = if in_p { &cfd_patched } else { &cfd_full };
                eprintln!(
                    "  geom[{}] {}: type={:?} body={} contype={} conaff={}",
                    g,
                    which,
                    model.geom_type[g],
                    model.geom_body[g],
                    model.geom_contype[g],
                    model.geom_conaffinity[g]
                );
            }
        }

        // Body geom address/count
        for b in 0..cfd_patched.nbody {
            let a1 = cfd_patched.body_geom_adr[b];
            let n1 = cfd_patched.body_geom_num[b];
            let a2 = cfd_full.body_geom_adr[b];
            let n2 = cfd_full.body_geom_num[b];
            if a1 != a2 || n1 != n2 {
                eprintln!("  body[{}] geom_adr/num: {}/{} vs {}/{}", b, a1, n1, a2, n2);
            }
        }

        // Option/solver fields
        let fields: Vec<(&str, f64, f64)> = vec![
            ("cone", cfd_patched.cone as f64, cfd_full.cone as f64),
            ("impratio", cfd_patched.impratio, cfd_full.impratio),
            (
                "solver_iter",
                cfd_patched.solver_iterations as f64,
                cfd_full.solver_iterations as f64,
            ),
            (
                "sdf_iter",
                cfd_patched.sdf_iterations as f64,
                cfd_full.sdf_iterations as f64,
            ),
            (
                "sdf_initpoints",
                cfd_patched.sdf_initpoints as f64,
                cfd_full.sdf_initpoints as f64,
            ),
            (
                "disableflags",
                cfd_patched.disableflags as f64,
                cfd_full.disableflags as f64,
            ),
            (
                "enableflags",
                cfd_patched.enableflags as f64,
                cfd_full.enableflags as f64,
            ),
        ];
        for (name, v1, v2) in &fields {
            if (v1 - v2).abs() > 1e-12 {
                eprintln!("  {}: {} vs {}", name, v1, v2);
            }
        }

        // Step 3: run all three models and compare
        eprintln!("\n=== Step 3: 2000-step comparison (mjcf vs cfd vs cfd_patched) ===");
        // DEFINITIVE TEST: Take MJCF model, rearrange to match cf-design layout
        // (SDF geoms first, plane last), keeping everything else from MJCF.
        // If this fails → structure matters. If stable → some field in generate() is wrong.
        let mut mjcf_reordered = mjcf_model.clone();

        // Current layout: [Plane(g0,body0), Sdf(g1,body1), Sdf(g2,body2)]
        // Target layout:  [Sdf(g0,body1), Sdf(g1,body2), Plane(g2,body0)]
        // (No mesh geoms yet — just reorder)
        let swap_geom_fields = |m: &mut sim_core::Model, a: usize, b: usize| {
            m.geom_type.swap(a, b);
            m.geom_body.swap(a, b);
            m.geom_pos.swap(a, b);
            m.geom_quat.swap(a, b);
            m.geom_size.swap(a, b);
            m.geom_friction.swap(a, b);
            m.geom_condim.swap(a, b);
            m.geom_contype.swap(a, b);
            m.geom_conaffinity.swap(a, b);
            m.geom_margin.swap(a, b);
            m.geom_gap.swap(a, b);
            m.geom_priority.swap(a, b);
            m.geom_solmix.swap(a, b);
            m.geom_solimp.swap(a, b);
            m.geom_solref.swap(a, b);
            m.geom_name.swap(a, b);
            m.geom_rbound.swap(a, b);
            m.geom_aabb.swap(a, b);
            m.geom_mesh.swap(a, b);
            m.geom_hfield.swap(a, b);
            m.geom_shape.swap(a, b);
            m.geom_group.swap(a, b);
            m.geom_rgba.swap(a, b);
            m.geom_fluid.swap(a, b);
            m.geom_plugin.swap(a, b);
            m.geom_user.swap(a, b);
        };

        // Move Plane from g0 to g2: swap g0↔g2, then swap g0↔g1
        swap_geom_fields(&mut mjcf_reordered, 0, 2);
        swap_geom_fields(&mut mjcf_reordered, 0, 1);
        // Now: [Sdf(body1), Sdf(body2), Plane(body0)]
        // Update body_geom_adr/num
        mjcf_reordered.body_geom_adr[0] = 2; // world body has plane at g2
        mjcf_reordered.body_geom_adr[1] = 0; // lower body has sdf at g0
        mjcf_reordered.body_geom_adr[2] = 1; // upper body has sdf at g1

        eprintln!("\n  === MJCF reordered geom layout ===");
        for g in 0..mjcf_reordered.ngeom {
            eprintln!(
                "    g{}: {:?} body={}",
                g, mjcf_reordered.geom_type[g], mjcf_reordered.geom_body[g]
            );
        }

        let mut reord_data = mjcf_reordered.make_data();
        reord_data.forward(&mjcf_reordered).unwrap();
        for _ in 0..2000 {
            reord_data.step(&mjcf_reordered).unwrap();
        }
        let gap_reord = reord_data.qpos[9] - reord_data.qpos[2];
        eprintln!(
            "  MJCF-reordered: z_lo={:.3} z_up={:.3} gap={:.3} ncon={} {}",
            reord_data.qpos[2],
            reord_data.qpos[9],
            gap_reord,
            reord_data.ncon,
            if gap_reord > 8.0 { "STABLE" } else { "FAILED" }
        );

        // REVERSE APPROACH: Start from MJCF (stable), apply cf-design changes
        // one at a time to find which breaks stability.
        let test_model = |name: &str, model: &sim_core::Model| {
            let mut d = model.make_data();
            d.forward(model).unwrap();
            for _ in 0..2000 {
                d.step(model).unwrap();
            }
            let gap = d.qpos[9] - d.qpos[2];
            let status = if gap > 8.0 { "STABLE" } else { "FAILED" };
            eprintln!("  {}: gap={:.3} {}", name, gap, status);
        };

        eprintln!("\n  === Reverse: MJCF + progressive cf-design changes ===");
        // Baseline
        test_model("mjcf_baseline", &cfd_patched); // MJCF clone + cf-design SDF (already stable)

        // 1. Set diagapprox = false (cf-design default before fix)
        let mut m1 = cfd_patched.clone();
        m1.diagapprox_bodyweight = false;
        test_model("+ diagapprox=false", &m1);

        // 2. Also set mass/inertia to cf-design values
        let mut m2 = m1.clone();
        for b in 1..m2.nbody {
            m2.body_mass[b] = cfd_model.body_mass[b];
            m2.body_inertia[b] = cfd_model.body_inertia[b];
        }
        m2.compute_implicit_params();
        m2.compute_stat_meaninertia();
        m2.compute_invweight0();
        test_model("+ cfd mass/inertia", &m2);

        // 3. Start over: MJCF + just diagapprox=false (no mass change)
        let mut m3 = cfd_patched.clone();
        m3.diagapprox_bodyweight = false;
        test_model("diagapprox=false only", &m3);

        // 4. MJCF + just mass change (no diagapprox)
        let mut m4 = cfd_patched.clone();
        for b in 1..m4.nbody {
            m4.body_mass[b] = cfd_model.body_mass[b];
            m4.body_inertia[b] = cfd_model.body_inertia[b];
        }
        m4.compute_implicit_params();
        m4.compute_stat_meaninertia();
        m4.compute_invweight0();
        test_model("cfd mass only", &m4);

        // COMPREHENSIVE DIFF: cfd_patched (MJCF+cfSDF, stable) vs cfd_model (unstable)
        // cfd_patched has diagapprox=true, MJCF mass, cf-design SDF.
        // cfd_model has diagapprox=true (now), cf-design mass, cf-design SDF.
        // Apply cf-design mass to cfd_patched so mass matches.
        let mut m1_model = cfd_patched.clone();
        for b in 1..m1_model.nbody {
            m1_model.body_mass[b] = cfd_model.body_mass[b];
            m1_model.body_inertia[b] = cfd_model.body_inertia[b];
        }
        m1_model.compute_implicit_params();
        m1_model.compute_stat_meaninertia();
        m1_model.compute_invweight0();
        let m1 = &m1_model;
        let m2 = &cfd_model;

        eprintln!("\n  === COMPREHENSIVE DIFF: mjcf_reord vs cfd ===");

        // Joint-level
        for j in 0..m1.njnt.min(m2.njnt) {
            let fields: Vec<(&str, f64, f64)> = vec![
                (
                    "jnt_stiffness",
                    m1.jnt_stiffness.get(j).copied().unwrap_or(0.0),
                    m2.jnt_stiffness.get(j).copied().unwrap_or(0.0),
                ),
                (
                    "jnt_armature",
                    m1.jnt_armature.get(j).copied().unwrap_or(0.0),
                    m2.jnt_armature.get(j).copied().unwrap_or(0.0),
                ),
                (
                    "jnt_damping",
                    m1.jnt_damping.get(j).copied().unwrap_or(0.0),
                    m2.jnt_damping.get(j).copied().unwrap_or(0.0),
                ),
                (
                    "jnt_margin",
                    m1.jnt_margin.get(j).copied().unwrap_or(0.0),
                    m2.jnt_margin.get(j).copied().unwrap_or(0.0),
                ),
            ];
            for (name, v1, v2) in fields {
                if (v1 - v2).abs() > 1e-12 {
                    eprintln!("  jnt[{}].{}: {:.8} vs {:.8}", j, name, v1, v2);
                }
            }
        }

        // DOF-level
        for d in 0..m1.nv.min(m2.nv) {
            let fields: Vec<(&str, f64, f64)> = vec![
                (
                    "dof_damping",
                    m1.dof_damping.get(d).copied().unwrap_or(0.0),
                    m2.dof_damping.get(d).copied().unwrap_or(0.0),
                ),
                (
                    "dof_armature",
                    m1.dof_armature.get(d).copied().unwrap_or(0.0),
                    m2.dof_armature.get(d).copied().unwrap_or(0.0),
                ),
                (
                    "dof_invweight0",
                    m1.dof_invweight0.get(d).copied().unwrap_or(0.0),
                    m2.dof_invweight0.get(d).copied().unwrap_or(0.0),
                ),
                (
                    "dof_frictionloss",
                    m1.dof_frictionloss.get(d).copied().unwrap_or(0.0),
                    m2.dof_frictionloss.get(d).copied().unwrap_or(0.0),
                ),
            ];
            for (name, v1, v2) in fields {
                if (v1 - v2).abs() > 1e-12 {
                    eprintln!("  dof[{}].{}: {:.8} vs {:.8}", d, name, v1, v2);
                }
            }
        }

        // Global options
        let globals: Vec<(&str, f64, f64)> = vec![
            ("stat_meaninertia", m1.stat_meaninertia, m2.stat_meaninertia),
            ("density", m1.density, m2.density),
            ("viscosity", m1.viscosity, m2.viscosity),
            ("o_margin", m1.o_margin, m2.o_margin),
        ];
        for (name, v1, v2) in globals {
            if (v1 - v2).abs() > 1e-12 {
                eprintln!("  {}: {:.8} vs {:.8}", name, v1, v2);
            }
        }

        // Body-level (ALL fields)
        for b in 0..m1.nbody.min(m2.nbody) {
            // Quaternion comparison
            let iq1 = m1.body_iquat.get(b).map(|q| q.as_vector().clone());
            let iq2 = m2.body_iquat.get(b).map(|q| q.as_vector().clone());
            if let (Some(q1), Some(q2)) = (&iq1, &iq2) {
                let diff = (q1 - q2).norm();
                if diff > 1e-10 {
                    eprintln!(
                        "  body[{}].iquat: ({:.4},{:.4},{:.4},{:.4}) vs ({:.4},{:.4},{:.4},{:.4})",
                        b, q1[0], q1[1], q1[2], q1[3], q2[0], q2[1], q2[2], q2[3]
                    );
                }
            }
            let bq1 = m1.body_quat.get(b).map(|q| q.as_vector().clone());
            let bq2 = m2.body_quat.get(b).map(|q| q.as_vector().clone());
            if let (Some(q1), Some(q2)) = (&bq1, &bq2) {
                let diff = (q1 - q2).norm();
                if diff > 1e-10 {
                    eprintln!("  body[{}].quat: diff={:.6}", b, diff);
                }
            }
        }

        // Joint-level (ALL fields including pos, axis)
        for j in 0..m1.njnt.min(m2.njnt) {
            let jp1 = m1.jnt_pos.get(j).copied().unwrap_or(Vector3::zeros());
            let jp2 = m2.jnt_pos.get(j).copied().unwrap_or(Vector3::zeros());
            if (jp1 - jp2).norm() > 1e-10 {
                eprintln!(
                    "  jnt[{}].pos: ({:.4},{:.4},{:.4}) vs ({:.4},{:.4},{:.4})",
                    j, jp1.x, jp1.y, jp1.z, jp2.x, jp2.y, jp2.z
                );
            }
            let ja1 = m1.jnt_axis.get(j).copied().unwrap_or(Vector3::z());
            let ja2 = m2.jnt_axis.get(j).copied().unwrap_or(Vector3::z());
            if (ja1 - ja2).norm() > 1e-10 {
                eprintln!(
                    "  jnt[{}].axis: ({:.4},{:.4},{:.4}) vs ({:.4},{:.4},{:.4})",
                    j, ja1.x, ja1.y, ja1.z, ja2.x, ja2.y, ja2.z
                );
            }
            let jb1 = m1.jnt_body.get(j).copied().unwrap_or(0);
            let jb2 = m2.jnt_body.get(j).copied().unwrap_or(0);
            if jb1 != jb2 {
                eprintln!("  jnt[{}].body: {} vs {}", j, jb1, jb2);
            }
        }

        // DOF-level (body mapping)
        for d in 0..m1.nv.min(m2.nv) {
            let db1 = m1.dof_body.get(d).copied().unwrap_or(0);
            let db2 = m2.dof_body.get(d).copied().unwrap_or(0);
            if db1 != db2 {
                eprintln!("  dof[{}].body: {} vs {}", d, db1, db2);
            }
        }

        // ── EXHAUSTIVE SERIALIZATION-BASED DIFF ──────────────────────────
        {
            use std::collections::{BTreeMap, BTreeSet, HashSet};

            eprintln!("\n╔══════════════════════════════════════════════════════════════╗");
            eprintln!("║  EXHAUSTIVE DIFF: cfd_patched (STABLE) vs cfd_model (UNSTABLE)  ║");
            eprintln!("╚══════════════════════════════════════════════════════════════╝");

            let pa = &cfd_patched;
            let pb = &cfd_model;

            let mut fa: BTreeMap<String, Vec<f64>> = BTreeMap::new();
            let mut fb: BTreeMap<String, Vec<f64>> = BTreeMap::new();

            macro_rules! ins {
                ($key:expr, $va:expr, $vb:expr) => {
                    fa.insert($key.to_string(), $va);
                    fb.insert($key.to_string(), $vb);
                };
            }

            // === GLOBAL SCALARS ===
            ins!("timestep", vec![pa.timestep], vec![pb.timestep]);
            ins!(
                "gravity",
                vec![pa.gravity.x, pa.gravity.y, pa.gravity.z],
                vec![pb.gravity.x, pb.gravity.y, pb.gravity.z]
            );
            ins!(
                "wind",
                vec![pa.wind.x, pa.wind.y, pa.wind.z],
                vec![pb.wind.x, pb.wind.y, pb.wind.z]
            );
            ins!(
                "magnetic",
                vec![pa.magnetic.x, pa.magnetic.y, pa.magnetic.z],
                vec![pb.magnetic.x, pb.magnetic.y, pb.magnetic.z]
            );
            ins!("density", vec![pa.density], vec![pb.density]);
            ins!("viscosity", vec![pa.viscosity], vec![pb.viscosity]);
            ins!("impratio", vec![pa.impratio], vec![pb.impratio]);
            ins!("o_margin", vec![pa.o_margin], vec![pb.o_margin]);
            ins!("o_solref", pa.o_solref.to_vec(), pb.o_solref.to_vec());
            ins!("o_solimp", pa.o_solimp.to_vec(), pb.o_solimp.to_vec());
            ins!("o_friction", pa.o_friction.to_vec(), pb.o_friction.to_vec());
            ins!(
                "solver_iterations",
                vec![pa.solver_iterations as f64],
                vec![pb.solver_iterations as f64]
            );
            ins!(
                "solver_tolerance",
                vec![pa.solver_tolerance],
                vec![pb.solver_tolerance]
            );
            ins!(
                "regularization",
                vec![pa.regularization],
                vec![pb.regularization]
            );
            ins!(
                "friction_smoothing",
                vec![pa.friction_smoothing],
                vec![pb.friction_smoothing]
            );
            ins!("cone", vec![pa.cone as f64], vec![pb.cone as f64]);
            ins!(
                "stat_meaninertia",
                vec![pa.stat_meaninertia],
                vec![pb.stat_meaninertia]
            );
            ins!(
                "ls_iterations",
                vec![pa.ls_iterations as f64],
                vec![pb.ls_iterations as f64]
            );
            ins!("ls_tolerance", vec![pa.ls_tolerance], vec![pb.ls_tolerance]);
            ins!(
                "noslip_iterations",
                vec![pa.noslip_iterations as f64],
                vec![pb.noslip_iterations as f64]
            );
            ins!(
                "noslip_tolerance",
                vec![pa.noslip_tolerance],
                vec![pb.noslip_tolerance]
            );
            ins!(
                "diagapprox_bodyweight",
                vec![pa.diagapprox_bodyweight as u8 as f64],
                vec![pb.diagapprox_bodyweight as u8 as f64]
            );
            ins!(
                "disableflags",
                vec![pa.disableflags as f64],
                vec![pb.disableflags as f64]
            );
            ins!(
                "enableflags",
                vec![pa.enableflags as f64],
                vec![pb.enableflags as f64]
            );
            ins!(
                "disableactuator",
                vec![pa.disableactuator as f64],
                vec![pb.disableactuator as f64]
            );
            ins!(
                "ccd_iterations",
                vec![pa.ccd_iterations as f64],
                vec![pb.ccd_iterations as f64]
            );
            ins!(
                "ccd_tolerance",
                vec![pa.ccd_tolerance],
                vec![pb.ccd_tolerance]
            );
            ins!(
                "sdf_iterations",
                vec![pa.sdf_iterations as f64],
                vec![pb.sdf_iterations as f64]
            );
            ins!(
                "sdf_initpoints",
                vec![pa.sdf_initpoints as f64],
                vec![pb.sdf_initpoints as f64]
            );
            ins!(
                "sleep_tolerance",
                vec![pa.sleep_tolerance],
                vec![pb.sleep_tolerance]
            );
            ins!(
                "ngravcomp",
                vec![pa.ngravcomp as f64],
                vec![pb.ngravcomp as f64]
            );

            // Dimensions (informational)
            ins!("nq", vec![pa.nq as f64], vec![pb.nq as f64]);
            ins!("nv", vec![pa.nv as f64], vec![pb.nv as f64]);
            ins!("nbody", vec![pa.nbody as f64], vec![pb.nbody as f64]);
            ins!("njnt", vec![pa.njnt as f64], vec![pb.njnt as f64]);
            ins!("ngeom", vec![pa.ngeom as f64], vec![pb.ngeom as f64]);
            ins!("ntree", vec![pa.ntree as f64], vec![pb.ntree as f64]);

            // Sparse LDL
            ins!("qLD_nnz", vec![pa.qLD_nnz as f64], vec![pb.qLD_nnz as f64]);
            ins!(
                "qLD_rowadr",
                pa.qLD_rowadr.iter().map(|&x| x as f64).collect(),
                pb.qLD_rowadr.iter().map(|&x| x as f64).collect()
            );
            ins!(
                "qLD_rownnz",
                pa.qLD_rownnz.iter().map(|&x| x as f64).collect(),
                pb.qLD_rownnz.iter().map(|&x| x as f64).collect()
            );
            ins!(
                "qLD_colind",
                pa.qLD_colind.iter().map(|&x| x as f64).collect(),
                pb.qLD_colind.iter().map(|&x| x as f64).collect()
            );

            // Vectors
            ins!(
                "qpos0",
                pa.qpos0.as_slice().to_vec(),
                pb.qpos0.as_slice().to_vec()
            );
            ins!(
                "qpos_spring",
                pa.qpos_spring.clone(),
                pb.qpos_spring.clone()
            );
            ins!(
                "implicit_stiffness",
                pa.implicit_stiffness.as_slice().to_vec(),
                pb.implicit_stiffness.as_slice().to_vec()
            );
            ins!(
                "implicit_damping",
                pa.implicit_damping.as_slice().to_vec(),
                pb.implicit_damping.as_slice().to_vec()
            );
            ins!(
                "implicit_springref",
                pa.implicit_springref.as_slice().to_vec(),
                pb.implicit_springref.as_slice().to_vec()
            );

            // === PER-BODY ===
            let nb = pa.nbody.min(pb.nbody);
            for b in 0..nb {
                let p = format!("body[{}]", b);
                ins!(
                    format!("{p}.parent"),
                    vec![pa.body_parent[b] as f64],
                    vec![pb.body_parent[b] as f64]
                );
                ins!(
                    format!("{p}.rootid"),
                    vec![pa.body_rootid[b] as f64],
                    vec![pb.body_rootid[b] as f64]
                );
                ins!(
                    format!("{p}.jnt_adr"),
                    vec![pa.body_jnt_adr[b] as f64],
                    vec![pb.body_jnt_adr[b] as f64]
                );
                ins!(
                    format!("{p}.jnt_num"),
                    vec![pa.body_jnt_num[b] as f64],
                    vec![pb.body_jnt_num[b] as f64]
                );
                ins!(
                    format!("{p}.dof_adr"),
                    vec![pa.body_dof_adr[b] as f64],
                    vec![pb.body_dof_adr[b] as f64]
                );
                ins!(
                    format!("{p}.dof_num"),
                    vec![pa.body_dof_num[b] as f64],
                    vec![pb.body_dof_num[b] as f64]
                );
                ins!(
                    format!("{p}.geom_adr"),
                    vec![pa.body_geom_adr[b] as f64],
                    vec![pb.body_geom_adr[b] as f64]
                );
                ins!(
                    format!("{p}.geom_num"),
                    vec![pa.body_geom_num[b] as f64],
                    vec![pb.body_geom_num[b] as f64]
                );
                ins!(
                    format!("{p}.mass"),
                    vec![pa.body_mass[b]],
                    vec![pb.body_mass[b]]
                );
                ins!(
                    format!("{p}.inertia"),
                    vec![
                        pa.body_inertia[b].x,
                        pa.body_inertia[b].y,
                        pa.body_inertia[b].z
                    ],
                    vec![
                        pb.body_inertia[b].x,
                        pb.body_inertia[b].y,
                        pb.body_inertia[b].z
                    ]
                );
                ins!(
                    format!("{p}.pos"),
                    vec![pa.body_pos[b].x, pa.body_pos[b].y, pa.body_pos[b].z],
                    vec![pb.body_pos[b].x, pb.body_pos[b].y, pb.body_pos[b].z]
                );
                let qa = pa.body_quat[b].as_vector();
                let qb_ = pb.body_quat[b].as_vector();
                ins!(
                    format!("{p}.quat"),
                    vec![qa[0], qa[1], qa[2], qa[3]],
                    vec![qb_[0], qb_[1], qb_[2], qb_[3]]
                );
                ins!(
                    format!("{p}.ipos"),
                    vec![pa.body_ipos[b].x, pa.body_ipos[b].y, pa.body_ipos[b].z],
                    vec![pb.body_ipos[b].x, pb.body_ipos[b].y, pb.body_ipos[b].z]
                );
                let iqa = pa.body_iquat[b].as_vector();
                let iqb = pb.body_iquat[b].as_vector();
                ins!(
                    format!("{p}.iquat"),
                    vec![iqa[0], iqa[1], iqa[2], iqa[3]],
                    vec![iqb[0], iqb[1], iqb[2], iqb[3]]
                );
                ins!(
                    format!("{p}.subtreemass"),
                    vec![pa.body_subtreemass[b]],
                    vec![pb.body_subtreemass[b]]
                );
                ins!(
                    format!("{p}.gravcomp"),
                    vec![pa.body_gravcomp[b]],
                    vec![pb.body_gravcomp[b]]
                );
                ins!(
                    format!("{p}.invweight0"),
                    vec![pa.body_invweight0[b][0], pa.body_invweight0[b][1]],
                    vec![pb.body_invweight0[b][0], pb.body_invweight0[b][1]]
                );
                ins!(
                    format!("{p}.treeid"),
                    vec![pa.body_treeid[b] as f64],
                    vec![pb.body_treeid[b] as f64]
                );
            }

            // === PER-JOINT ===
            let nj = pa.njnt.min(pb.njnt);
            for j in 0..nj {
                let p = format!("jnt[{}]", j);
                ins!(
                    format!("{p}.body"),
                    vec![pa.jnt_body[j] as f64],
                    vec![pb.jnt_body[j] as f64]
                );
                ins!(
                    format!("{p}.qpos_adr"),
                    vec![pa.jnt_qpos_adr[j] as f64],
                    vec![pb.jnt_qpos_adr[j] as f64]
                );
                ins!(
                    format!("{p}.dof_adr"),
                    vec![pa.jnt_dof_adr[j] as f64],
                    vec![pb.jnt_dof_adr[j] as f64]
                );
                ins!(
                    format!("{p}.pos"),
                    vec![pa.jnt_pos[j].x, pa.jnt_pos[j].y, pa.jnt_pos[j].z],
                    vec![pb.jnt_pos[j].x, pb.jnt_pos[j].y, pb.jnt_pos[j].z]
                );
                ins!(
                    format!("{p}.axis"),
                    vec![pa.jnt_axis[j].x, pa.jnt_axis[j].y, pa.jnt_axis[j].z],
                    vec![pb.jnt_axis[j].x, pb.jnt_axis[j].y, pb.jnt_axis[j].z]
                );
                ins!(
                    format!("{p}.limited"),
                    vec![pa.jnt_limited[j] as u8 as f64],
                    vec![pb.jnt_limited[j] as u8 as f64]
                );
                ins!(
                    format!("{p}.range"),
                    vec![pa.jnt_range[j].0, pa.jnt_range[j].1],
                    vec![pb.jnt_range[j].0, pb.jnt_range[j].1]
                );
                ins!(
                    format!("{p}.stiffness"),
                    vec![pa.jnt_stiffness[j]],
                    vec![pb.jnt_stiffness[j]]
                );
                ins!(
                    format!("{p}.springref"),
                    vec![pa.jnt_springref[j]],
                    vec![pb.jnt_springref[j]]
                );
                ins!(
                    format!("{p}.damping"),
                    vec![pa.jnt_damping[j]],
                    vec![pb.jnt_damping[j]]
                );
                ins!(
                    format!("{p}.armature"),
                    vec![pa.jnt_armature[j]],
                    vec![pb.jnt_armature[j]]
                );
                ins!(
                    format!("{p}.solref"),
                    pa.jnt_solref[j].to_vec(),
                    pb.jnt_solref[j].to_vec()
                );
                ins!(
                    format!("{p}.solimp"),
                    pa.jnt_solimp[j].to_vec(),
                    pb.jnt_solimp[j].to_vec()
                );
                ins!(
                    format!("{p}.margin"),
                    vec![pa.jnt_margin[j]],
                    vec![pb.jnt_margin[j]]
                );
                ins!(
                    format!("{p}.group"),
                    vec![pa.jnt_group[j] as f64],
                    vec![pb.jnt_group[j] as f64]
                );
                ins!(
                    format!("{p}.actgravcomp"),
                    vec![pa.jnt_actgravcomp[j] as u8 as f64],
                    vec![pb.jnt_actgravcomp[j] as u8 as f64]
                );
            }

            // === PER-DOF ===
            let ndof = pa.nv.min(pb.nv);
            for d in 0..ndof {
                let p = format!("dof[{}]", d);
                ins!(
                    format!("{p}.body"),
                    vec![pa.dof_body[d] as f64],
                    vec![pb.dof_body[d] as f64]
                );
                ins!(
                    format!("{p}.jnt"),
                    vec![pa.dof_jnt[d] as f64],
                    vec![pb.dof_jnt[d] as f64]
                );
                ins!(
                    format!("{p}.parent"),
                    vec![pa.dof_parent[d].map_or(-1.0, |x| x as f64)],
                    vec![pb.dof_parent[d].map_or(-1.0, |x| x as f64)]
                );
                ins!(
                    format!("{p}.armature"),
                    vec![pa.dof_armature[d]],
                    vec![pb.dof_armature[d]]
                );
                ins!(
                    format!("{p}.damping"),
                    vec![pa.dof_damping[d]],
                    vec![pb.dof_damping[d]]
                );
                ins!(
                    format!("{p}.frictionloss"),
                    vec![pa.dof_frictionloss[d]],
                    vec![pb.dof_frictionloss[d]]
                );
                ins!(
                    format!("{p}.solref"),
                    pa.dof_solref[d].to_vec(),
                    pb.dof_solref[d].to_vec()
                );
                ins!(
                    format!("{p}.solimp"),
                    pa.dof_solimp[d].to_vec(),
                    pb.dof_solimp[d].to_vec()
                );
                ins!(
                    format!("{p}.invweight0"),
                    vec![pa.dof_invweight0[d]],
                    vec![pb.dof_invweight0[d]]
                );
                ins!(
                    format!("{p}.treeid"),
                    vec![pa.dof_treeid[d] as f64],
                    vec![pb.dof_treeid[d] as f64]
                );
                ins!(
                    format!("{p}.length"),
                    vec![pa.dof_length[d]],
                    vec![pb.dof_length[d]]
                );
            }

            // === MATCHED GEOMS (by type + body) ===
            let mut used_b = HashSet::new();
            let mut geom_pairs: Vec<(usize, usize, String)> = Vec::new();
            for ga in 0..pa.ngeom {
                for gb in 0..pb.ngeom {
                    if used_b.contains(&gb) {
                        continue;
                    }
                    if pb.geom_type[gb] == pa.geom_type[ga] && pb.geom_body[gb] == pa.geom_body[ga]
                    {
                        geom_pairs.push((
                            ga,
                            gb,
                            format!("{:?}_body{}", pa.geom_type[ga], pa.geom_body[ga]),
                        ));
                        used_b.insert(gb);
                        break;
                    }
                }
            }

            for (ga, gb, label) in &geom_pairs {
                let p = format!("geom[{}]", label);
                ins!(
                    format!("{p}.pos"),
                    vec![pa.geom_pos[*ga].x, pa.geom_pos[*ga].y, pa.geom_pos[*ga].z],
                    vec![pb.geom_pos[*gb].x, pb.geom_pos[*gb].y, pb.geom_pos[*gb].z]
                );
                let gqa = pa.geom_quat[*ga].as_vector();
                let gqb = pb.geom_quat[*gb].as_vector();
                ins!(
                    format!("{p}.quat"),
                    vec![gqa[0], gqa[1], gqa[2], gqa[3]],
                    vec![gqb[0], gqb[1], gqb[2], gqb[3]]
                );
                ins!(
                    format!("{p}.size"),
                    vec![
                        pa.geom_size[*ga].x,
                        pa.geom_size[*ga].y,
                        pa.geom_size[*ga].z
                    ],
                    vec![
                        pb.geom_size[*gb].x,
                        pb.geom_size[*gb].y,
                        pb.geom_size[*gb].z
                    ]
                );
                ins!(
                    format!("{p}.friction"),
                    vec![
                        pa.geom_friction[*ga].x,
                        pa.geom_friction[*ga].y,
                        pa.geom_friction[*ga].z
                    ],
                    vec![
                        pb.geom_friction[*gb].x,
                        pb.geom_friction[*gb].y,
                        pb.geom_friction[*gb].z
                    ]
                );
                ins!(
                    format!("{p}.condim"),
                    vec![pa.geom_condim[*ga] as f64],
                    vec![pb.geom_condim[*gb] as f64]
                );
                ins!(
                    format!("{p}.contype"),
                    vec![pa.geom_contype[*ga] as f64],
                    vec![pb.geom_contype[*gb] as f64]
                );
                ins!(
                    format!("{p}.conaffinity"),
                    vec![pa.geom_conaffinity[*ga] as f64],
                    vec![pb.geom_conaffinity[*gb] as f64]
                );
                ins!(
                    format!("{p}.margin"),
                    vec![pa.geom_margin[*ga]],
                    vec![pb.geom_margin[*gb]]
                );
                ins!(
                    format!("{p}.gap"),
                    vec![pa.geom_gap[*ga]],
                    vec![pb.geom_gap[*gb]]
                );
                ins!(
                    format!("{p}.priority"),
                    vec![pa.geom_priority[*ga] as f64],
                    vec![pb.geom_priority[*gb] as f64]
                );
                ins!(
                    format!("{p}.solmix"),
                    vec![pa.geom_solmix[*ga]],
                    vec![pb.geom_solmix[*gb]]
                );
                ins!(
                    format!("{p}.solimp"),
                    pa.geom_solimp[*ga].to_vec(),
                    pb.geom_solimp[*gb].to_vec()
                );
                ins!(
                    format!("{p}.solref"),
                    pa.geom_solref[*ga].to_vec(),
                    pb.geom_solref[*gb].to_vec()
                );
                ins!(
                    format!("{p}.rbound"),
                    vec![pa.geom_rbound[*ga]],
                    vec![pb.geom_rbound[*gb]]
                );
                ins!(
                    format!("{p}.aabb"),
                    pa.geom_aabb[*ga].to_vec(),
                    pb.geom_aabb[*gb].to_vec()
                );
                ins!(
                    format!("{p}.group"),
                    vec![pa.geom_group[*ga] as f64],
                    vec![pb.geom_group[*gb] as f64]
                );
                ins!(
                    format!("{p}.rgba"),
                    pa.geom_rgba[*ga].to_vec(),
                    pb.geom_rgba[*gb].to_vec()
                );
                ins!(
                    format!("{p}.fluid"),
                    pa.geom_fluid[*ga].to_vec(),
                    pb.geom_fluid[*gb].to_vec()
                );
            }

            // === TREE ===
            let nt = pa.ntree.min(pb.ntree);
            for t in 0..nt {
                let p = format!("tree[{}]", t);
                ins!(
                    format!("{p}.body_adr"),
                    vec![pa.tree_body_adr[t] as f64],
                    vec![pb.tree_body_adr[t] as f64]
                );
                ins!(
                    format!("{p}.body_num"),
                    vec![pa.tree_body_num[t] as f64],
                    vec![pb.tree_body_num[t] as f64]
                );
                ins!(
                    format!("{p}.dof_adr"),
                    vec![pa.tree_dof_adr[t] as f64],
                    vec![pb.tree_dof_adr[t] as f64]
                );
                ins!(
                    format!("{p}.dof_num"),
                    vec![pa.tree_dof_num[t] as f64],
                    vec![pb.tree_dof_num[t] as f64]
                );
            }

            // === AUTO-DIFF ===
            let all_keys: BTreeSet<String> = fa.keys().chain(fb.keys()).cloned().collect();
            let mut diff_count = 0usize;
            let mut diff_fields: Vec<String> = Vec::new();

            for key in &all_keys {
                match (fa.get(key), fb.get(key)) {
                    (Some(va), Some(vb)) => {
                        if va.len() == vb.len() {
                            for (i, (a, b)) in va.iter().zip(vb.iter()).enumerate() {
                                if (a - b).abs() > 1e-12 {
                                    if va.len() == 1 {
                                        eprintln!(
                                            "  DIFF {}: {:.10} vs {:.10} (Δ={:.2e})",
                                            key,
                                            a,
                                            b,
                                            (a - b).abs()
                                        );
                                    } else {
                                        eprintln!(
                                            "  DIFF {}[{}]: {:.10} vs {:.10} (Δ={:.2e})",
                                            key,
                                            i,
                                            a,
                                            b,
                                            (a - b).abs()
                                        );
                                    }
                                    diff_count += 1;
                                    if !diff_fields.contains(key) {
                                        diff_fields.push(key.clone());
                                    }
                                }
                            }
                        } else {
                            eprintln!("  DIFF {}: len {} vs {}", key, va.len(), vb.len());
                            diff_count += 1;
                            diff_fields.push(key.clone());
                        }
                    }
                    (Some(_), None) => {
                        eprintln!("  DIFF {}: in patched only", key);
                        diff_count += 1;
                        diff_fields.push(key.clone());
                    }
                    (None, Some(_)) => {
                        eprintln!("  DIFF {}: in cfd only", key);
                        diff_count += 1;
                        diff_fields.push(key.clone());
                    }
                    (None, None) => unreachable!(),
                }
            }

            // Enum fields (non-numeric comparison)
            if format!("{:?}", pa.integrator) != format!("{:?}", pb.integrator) {
                eprintln!(
                    "  DIFF integrator: {:?} vs {:?}",
                    pa.integrator, pb.integrator
                );
                diff_count += 1;
                diff_fields.push("integrator".into());
            }
            if format!("{:?}", pa.solver_type) != format!("{:?}", pb.solver_type) {
                eprintln!(
                    "  DIFF solver_type: {:?} vs {:?}",
                    pa.solver_type, pb.solver_type
                );
                diff_count += 1;
                diff_fields.push("solver_type".into());
            }
            for j in 0..nj {
                if format!("{:?}", pa.jnt_type[j]) != format!("{:?}", pb.jnt_type[j]) {
                    eprintln!(
                        "  DIFF jnt[{}].type: {:?} vs {:?}",
                        j, pa.jnt_type[j], pb.jnt_type[j]
                    );
                    diff_count += 1;
                    diff_fields.push(format!("jnt[{}].type", j));
                }
            }

            eprintln!("\n  ══════════════════════════════════════════");
            eprintln!("  Total fields checked: {}", all_keys.len() + 3); // +3 for enum fields
            eprintln!("  Total diff entries: {}", diff_count);
            eprintln!("  Diff fields: {:?}", diff_fields);
            eprintln!("  ══════════════════════════════════════════");
        }
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

        // Grid contacts (Tier 3 path)
        let grid_contacts = sdf_sdf_contact(&bore_grid, &bore_pose, &pin_grid, &pin_pose, 0.5);

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
}
