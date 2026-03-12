//! Flex deformable body processing.
//!
//! Converts MJCF `<flex>` / `<flexcomp>` elements into the flex body arrays.
//! Creates a kinematic body (+ 3 slide joints for unpinned vertices) per flex
//! vertex, extracts edges/hinges, and populates per-flex material properties.
//!
//! MuJoCo architecture: flex vertices ARE bodies in the kinematic tree.
//! `mj_crb`/`mj_rne` have zero flex-specific code — the standard rigid-body
//! pipeline handles everything.

use nalgebra::{UnitQuaternion, Vector3};

use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, ModelConversionError};
use crate::types::MjcfFlex;
use sim_core::{FlexSelfCollide, MjJointType};

/// Number of coefficients per bending element in the discrete thin-shell model.
/// Each edge hinge stores a 4×4 symmetric matrix (16 entries) + 1 rest-angle scalar = 17 values.
const BENDING_COEFFS: usize = 17;

/// Minimum vertex mass (kg) applied when computed mass is near-zero.
/// Prevents singular mass matrices in the deformable body integrator.
const MIN_VERTEX_MASS: f64 = 0.001;

impl ModelBuilder {
    /// Process flex deformable bodies: create a real body (+ 3 slide joints for unpinned
    /// vertices) per flex vertex, extract edges/hinges, populate Model flex_* arrays.
    ///
    /// MuJoCo architecture: flex vertices ARE bodies in the kinematic tree. `mj_crb`/`mj_rne`
    /// have zero flex-specific code — the standard rigid-body pipeline handles everything.
    pub(crate) fn process_flex_bodies(
        &mut self,
        flex_list: &[MjcfFlex],
    ) -> std::result::Result<(), ModelConversionError> {
        use std::collections::HashMap;

        let axes = [Vector3::x(), Vector3::y(), Vector3::z()];

        for flex_orig in flex_list {
            // Resolve `node` attribute: if `node` names are present and no vertices
            // exist, use the named bodies as flex vertices (MuJoCo semantics).
            //
            // Each node body IS the vertex. The new vertex body is parented to the
            // node body with body_pos = zeros (vertex is at node body origin).
            // We store the node body's world position as the vertex position so that
            // edge rest-lengths and mass lumping use correct geometry. Then override
            // body_pos to zeros when creating the vertex body (see `is_node_flex`
            // flag below).
            let resolved;
            let is_node_flex;
            let flex = if !flex_orig.node.is_empty() && flex_orig.vertices.is_empty() {
                resolved = {
                    let mut f = flex_orig.clone();
                    for node_name in &flex_orig.node {
                        let &bid = self.body_name_to_id.get(node_name).ok_or_else(|| {
                            ModelConversionError {
                                message: format!(
                                    "flex '{}' node references undefined body '{}'",
                                    flex_orig.name, node_name
                                ),
                            }
                        })?;
                        // World position for geometry (rest-lengths, mass lumping).
                        // body_world_pos is indexed by (body_id - 1) since worldbody
                        // has no entry. Node bodies are always non-world (bid >= 1).
                        let world_pos = if bid == 0 {
                            Vector3::zeros()
                        } else {
                            self.body_world_pos[bid - 1]
                        };
                        f.vertices.push(world_pos);
                        f.body.push(node_name.clone());
                    }
                    f
                };
                is_node_flex = true;
                &resolved
            } else {
                is_node_flex = false;
                flex_orig
            };
            let flex_id = self.nflex;
            self.nflex += 1;

            let vert_start = self.nflexvert;

            // Compute per-vertex masses (uniform from mass attr, or element-based lumping)
            let vertex_masses = compute_vertex_masses(flex);

            // Create a body (+ joints for unpinned) per vertex
            for (i, pos) in flex.vertices.iter().enumerate() {
                let mass = vertex_masses[i];
                let pinned = flex.pinned.contains(&i);
                let inv_mass = if pinned || mass <= 0.0 {
                    0.0
                } else {
                    1.0 / mass
                };

                self.nflexvert += 1;

                // --- Create body for this vertex ---
                let body_id = self.body_parent.len();

                // Resolve parent body: for bare <flex> use the named body attr;
                // for <flexcomp> (no body attr), parent is worldbody (0).
                let parent_id = if i < flex.body.len() {
                    if let Some(&bid) = self.body_name_to_id.get(&flex.body[i]) {
                        bid
                    } else {
                        eprintln!(
                            "Warning: flex '{}' vertex {} references unknown body '{}', \
                             parenting to worldbody",
                            flex.name, i, flex.body[i]
                        );
                        0
                    }
                } else {
                    0 // <flexcomp> → parent is worldbody
                };

                // Each flex vertex body parented to worldbody is its own kinematic tree root.
                // For bodies parented to a non-world body, inherit the root.
                let root_id = if parent_id == 0 {
                    body_id
                } else {
                    self.body_rootid[parent_id]
                };

                let jnt_adr = self.jnt_type.len();
                let dof_adr = self.nv;
                let geom_adr = self.geom_type.len();

                // Push body arrays (mirrors process_body_with_world_frame pattern)
                self.body_parent.push(parent_id);
                self.body_rootid.push(root_id);
                self.body_jnt_adr.push(jnt_adr);
                self.body_dof_adr.push(dof_adr);
                self.body_geom_adr.push(geom_adr);
                // For node-derived vertices, body_pos = zeros (vertex is at the node
                // body's origin; the slide joints provide displacement from there).
                // For flexcomp/bare-flex, body_pos = vertex world position (parent is
                // worldbody, so local offset == world position).
                let body_pos_val = if is_node_flex { Vector3::zeros() } else { *pos };
                self.body_pos.push(body_pos_val);
                self.body_quat.push(UnitQuaternion::identity());
                self.body_ipos.push(Vector3::zeros()); // point mass: CoM at body origin
                self.body_iquat.push(UnitQuaternion::identity());
                self.body_mass.push(mass);
                self.body_inertia.push(Vector3::zeros()); // point mass: zero inertia
                self.body_mocapid.push(None);
                self.body_name.push(None);
                self.body_sleep_policy.push(None);
                // *pos is the vertex world position from node resolution or flexcomp.
                // For flexcomp (parent=world), local offset == world position.
                // For node-flex, *pos is the node body's world position.
                self.body_world_pos.push(*pos);
                self.body_world_quat.push(UnitQuaternion::identity());

                if pinned {
                    // Pinned vertex: body exists but has no joints/DOFs.
                    // Body is fixed at its initial position (like a welded body).
                    self.body_jnt_num.push(0);
                    self.body_dof_num.push(0);
                    self.body_geom_num.push(0);

                    self.flexvert_qposadr.push(usize::MAX); // sentinel: no qpos
                    self.flexvert_dofadr.push(usize::MAX); // sentinel: no dof
                } else {
                    // Unpinned vertex: 3 slide joints (X, Y, Z)
                    // dof_parent chain: first DOF has no parent (worldbody has no DOFs),
                    // subsequent DOFs chain to previous within this vertex.
                    let mut last_dof: Option<usize> = None;
                    // Determine parent's last DOF for kinematic tree linkage.
                    // Worldbody (parent_id=0) has no DOFs → None.
                    // Non-world parent: use their last DOF.
                    let parent_last_dof = if parent_id == 0 {
                        None
                    } else {
                        let pdof_adr = self.body_dof_adr[parent_id];
                        let pdof_num = self.body_dof_num[parent_id];
                        if pdof_num > 0 {
                            Some(pdof_adr + pdof_num - 1)
                        } else {
                            None
                        }
                    };

                    for (axis_idx, axis) in axes.iter().enumerate() {
                        let jnt_id = self.jnt_type.len();
                        let qpos_adr = self.nq;
                        let dof_idx = self.nv;

                        // Joint arrays
                        self.jnt_type.push(MjJointType::Slide);
                        self.jnt_body.push(body_id);
                        self.jnt_qpos_adr.push(qpos_adr);
                        self.jnt_dof_adr.push(dof_idx);
                        self.jnt_pos.push(Vector3::zeros());
                        self.jnt_axis.push(*axis);
                        self.jnt_limited.push(false);
                        self.jnt_range
                            .push((-std::f64::consts::PI, std::f64::consts::PI));
                        self.jnt_stiffness.push(0.0);
                        self.jnt_springref.push(0.0);
                        self.jnt_damping.push(0.0);
                        self.jnt_armature.push(0.0);

                        // DOF arrays
                        let dof_parent_val = if axis_idx == 0 {
                            parent_last_dof // first DOF links to parent body's last DOF
                        } else {
                            last_dof // subsequent DOFs chain within this vertex
                        };
                        self.dof_parent.push(dof_parent_val);
                        self.dof_body.push(body_id);
                        self.dof_jnt.push(jnt_id);
                        self.dof_armature.push(0.0);
                        self.dof_damping.push(0.0);
                        self.dof_frictionloss.push(0.0);
                        self.dof_solref.push(DEFAULT_SOLREF);
                        self.dof_solimp.push(DEFAULT_SOLIMP);

                        // qpos0 = 0 for each slide DOF (body_pos encodes initial position)
                        self.qpos0_values.push(0.0);
                        // qpos_spring = 0 for flex vertex slide joints (springref = 0)
                        self.qpos_spring_values.push(0.0);

                        self.nq += 1;
                        self.nv += 1;
                        last_dof = Some(dof_idx);
                    }

                    self.body_jnt_num.push(3);
                    self.body_dof_num.push(3);
                    self.body_geom_num.push(0);

                    self.flexvert_qposadr.push(dof_adr); // first of 3 slide joint qpos addresses
                    self.flexvert_dofadr.push(dof_adr); // first of 3 DOF addresses
                }

                // Convenience mirrors for edge/element force code
                self.flexvert_mass.push(mass);
                self.flexvert_invmass.push(inv_mass);
                self.flexvert_radius.push(flex.radius);
                self.flexvert_flexid.push(flex_id);
                self.flexvert_bodyid.push(body_id);
                self.flexvert_initial_pos.push(*pos);
            }

            // Extract edges from element connectivity
            let edge_start = self.nflexedge;
            let mut edge_set: HashMap<(usize, usize), bool> = HashMap::new();
            let mut edge_key_to_global: HashMap<(usize, usize), usize> = HashMap::new();
            for elem in &flex.elements {
                let n = elem.len();
                for i_v in 0..n {
                    for j_v in (i_v + 1)..n {
                        let (a, b) = if elem[i_v] < elem[j_v] {
                            (elem[i_v], elem[j_v])
                        } else {
                            (elem[j_v], elem[i_v])
                        };
                        edge_set.entry((a, b)).or_insert(true);
                    }
                }
            }
            for &(a, b) in edge_set.keys() {
                let rest_len = (flex.vertices[b] - flex.vertices[a]).norm();
                self.flexedge_vert.push([vert_start + a, vert_start + b]);
                self.flexedge_length0.push(rest_len);
                self.flexedge_flexid.push(flex_id);
                // §42A-ii: flexedge_rigid — both endpoints pinned (invmass == 0).
                let va = vert_start + a;
                let vb = vert_start + b;
                self.flexedge_rigid
                    .push(self.flexvert_invmass[va] == 0.0 && self.flexvert_invmass[vb] == 0.0);
                // §42B S1: Initialize flexedge_flap to [-1, -1] (boundary default).
                self.flexedge_flap.push([-1, -1]);
                // §42B S2+S3: Initialize flex_bending to zeros per edge.
                self.flex_bending.extend_from_slice(&[0.0; BENDING_COEFFS]);
                // Track edge key → global index for flap topology.
                edge_key_to_global.insert((a, b), self.nflexedge);
                self.nflexedge += 1;
            }

            // Extract bending hinges from adjacent elements
            let mut edge_elements: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
            for (elem_idx, verts) in flex.elements.iter().enumerate() {
                let n = verts.len();
                for i_v in 0..n {
                    for j_v in (i_v + 1)..n {
                        let (a, b) = if verts[i_v] < verts[j_v] {
                            (verts[i_v], verts[j_v])
                        } else {
                            (verts[j_v], verts[i_v])
                        };
                        edge_elements.entry((a, b)).or_default().push(elem_idx);
                    }
                }
            }
            for ((ve0, ve1), elems) in &edge_elements {
                if elems.len() != 2 {
                    continue; // boundary edge
                }
                let opp_a = flex.elements[elems[0]]
                    .iter()
                    .find(|&&v| v != *ve0 && v != *ve1)
                    .copied();
                let opp_b = flex.elements[elems[1]]
                    .iter()
                    .find(|&&v| v != *ve0 && v != *ve1)
                    .copied();
                if let (Some(va), Some(vb)) = (opp_a, opp_b) {
                    let rest_angle = compute_dihedral_angle(
                        flex.vertices[*ve0],
                        flex.vertices[*ve1],
                        flex.vertices[va],
                        flex.vertices[vb],
                    );
                    self.flexhinge_vert.push([
                        vert_start + *ve0,
                        vert_start + *ve1,
                        vert_start + va,
                        vert_start + vb,
                    ]);
                    self.flexhinge_angle0.push(rest_angle);
                    self.flexhinge_flexid.push(flex_id);
                    self.nflexhinge += 1;

                    // §42B S1: Populate flexedge_flap for this interior edge.
                    if let Some(&global_e) = edge_key_to_global.get(&(*ve0, *ve1)) {
                        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
                        {
                            self.flexedge_flap[global_e] =
                                [(vert_start + va) as i32, (vert_start + vb) as i32];
                        }
                    }
                }
            }

            // §42B S2+S3: Compute cotangent bending coefficients for dim==2
            // flex with valid material (young > 0, thickness > 0).
            if flex.dim == 2 && flex.young > 0.0 && flex.thickness > 0.0 {
                let mu = flex.young / (2.0 * (1.0 + flex.poisson));
                let num_edges = self.nflexedge - edge_start;
                for local_e in 0..num_edges {
                    let global_e = edge_start + local_e;
                    let flap = self.flexedge_flap[global_e];
                    if flap[1] == -1 {
                        continue; // boundary edge: coefficients stay zero
                    }
                    let ev = self.flexedge_vert[global_e];
                    let v = [
                        flex.vertices[ev[0] - vert_start],
                        flex.vertices[ev[1] - vert_start],
                        flex.vertices[(flap[0] as usize) - vert_start],
                        flex.vertices[(flap[1] as usize) - vert_start],
                    ];
                    let coeffs = compute_bending_coefficients(v, mu, flex.thickness);
                    let base = BENDING_COEFFS * global_e;
                    self.flex_bending[base..base + BENDING_COEFFS].copy_from_slice(&coeffs);
                }
            }

            // Process elements
            for elem in &flex.elements {
                let adr = self.flexelem_data.len();
                for &v in elem {
                    self.flexelem_data.push(vert_start + v);
                }
                self.flexelem_dataadr.push(adr);
                self.flexelem_datanum.push(elem.len());
                self.flexelem_flexid.push(flex_id);

                let rest_vol = if elem.len() == 4 {
                    let e1 = flex.vertices[elem[1]] - flex.vertices[elem[0]];
                    let e2 = flex.vertices[elem[2]] - flex.vertices[elem[0]];
                    let e3 = flex.vertices[elem[3]] - flex.vertices[elem[0]];
                    // Use signed volume to match the constraint assembly formula.
                    // Positive for right-handed vertex ordering, negative for inverted tets.
                    e1.dot(&e2.cross(&e3)) / 6.0
                } else {
                    0.0
                };
                self.flexelem_volume0.push(rest_vol);
                self.nflexelem += 1;
            }

            // Compute solref from material properties
            let edge_solref = compute_edge_solref(flex);
            let k_bend = compute_bend_stiffness_from_material(flex);
            let b_bend = compute_bend_damping_from_material(flex, k_bend);

            // Per-flex arrays
            self.flex_dim.push(flex.dim);
            self.flex_vertadr.push(vert_start);
            self.flex_vertnum.push(flex.vertices.len());
            self.flex_damping.push(flex.damping);
            self.flex_friction.push(flex.friction);
            self.flex_condim.push(flex.condim);
            self.flex_margin.push(flex.margin);
            self.flex_gap.push(flex.gap);
            self.flex_priority.push(flex.priority);
            self.flex_solmix.push(flex.solmix);
            self.flex_solref.push(flex.solref);
            self.flex_solimp.push(flex.solimp);
            self.flex_edge_solref.push(edge_solref);
            self.flex_edge_solimp.push(flex.solimp);
            self.flex_bend_stiffness.push(k_bend);
            self.flex_bend_damping.push(b_bend);
            self.flex_young.push(flex.young);
            self.flex_poisson.push(flex.poisson);
            self.flex_thickness.push(flex.thickness);
            self.flex_density.push(flex.density);
            self.flex_group.push(flex.group);
            self.flex_contype.push(flex.contype.unwrap_or(1) as u32);
            self.flex_conaffinity
                .push(flex.conaffinity.unwrap_or(1) as u32);
            // MuJoCo default is "auto" (enabled). Convert string to FlexSelfCollide enum.
            let sc = match flex.selfcollide.as_deref() {
                Some("none") => FlexSelfCollide::None,
                Some("narrow") => FlexSelfCollide::Narrow,
                Some("bvh") => FlexSelfCollide::Bvh,
                Some("sap") => FlexSelfCollide::Sap,
                Some("auto") | None | Some(_) => FlexSelfCollide::Auto,
            };
            self.flex_selfcollide.push(sc);
            // DT-85: flex contact runtime attributes
            self.flex_internal.push(flex.internal);
            self.flex_activelayers.push(flex.activelayers);
            self.flex_vertcollide.push(flex.vertcollide);
            self.flex_passive.push(flex.passive);
            self.flex_edgestiffness.push(flex.edge_stiffness);
            self.flex_edgedamping.push(flex.edge_damping);

            // §42A-ii: Compute flex_rigid — true if ALL vertices have invmass == 0.
            let all_rigid = (vert_start..vert_start + flex.vertices.len())
                .all(|v| self.flexvert_invmass[v] == 0.0);
            self.flex_rigid.push(all_rigid);

            // §42B S4: Push bending model type (default: Cotangent).
            self.flex_bending_type.push(flex.bending_model);
        }

        // §42A-i: Allocate CSR structure for sparse edge Jacobian.
        // Sparsity pattern: union of DOFs from the two endpoint bodies.
        // Runs once after ALL flexes have been processed so all edges are present.
        {
            let nedge = self.nflexedge;
            let mut rownnz = Vec::with_capacity(nedge);
            let mut rowadr = Vec::with_capacity(nedge);
            let mut colind = Vec::new();
            let mut adr: usize = 0;

            for e in 0..nedge {
                let [v0, v1] = self.flexedge_vert[e];
                let b0 = self.flexvert_bodyid[v0];
                let b1 = self.flexvert_bodyid[v1];
                let dn0 = self.body_dof_num[b0];
                let dn1 = self.body_dof_num[b1];
                let nnz = dn0 + dn1;

                rownnz.push(nnz);
                rowadr.push(adr);

                let da0 = self.body_dof_adr[b0];
                for k in 0..dn0 {
                    colind.push(da0 + k);
                }
                let da1 = self.body_dof_adr[b1];
                for k in 0..dn1 {
                    colind.push(da1 + k);
                }

                adr += nnz;
            }

            self.flexedge_J_rownnz = rownnz;
            self.flexedge_J_rowadr = rowadr;
            self.flexedge_J_colind = colind;
        }

        Ok(())
    }
}

// ============================================================================
// Flex helper functions
// ============================================================================

/// Compute per-edge cross-section area based on flex dimensionality.
///
/// - dim=1 (cable): PI * radius^2
/// - dim=2 (shell): thickness * rest_length (dual edge cross-section)
/// - dim=3 (solid): rest_length^2 (L^2 approximation)
pub fn compute_flexedge_crosssection(
    flexedge_flexid: &[usize],
    flexedge_length0: &[f64],
    flex_dim: &[usize],
    flex_thickness: &[f64],
    flexvert_radius: &[f64],
    flex_vertadr: &[usize],
) -> Vec<f64> {
    let nedge = flexedge_flexid.len();
    let mut crosssection = vec![1.0; nedge];
    for i in 0..nedge {
        let fid = flexedge_flexid[i];
        if fid >= flex_dim.len() {
            continue;
        }
        let dim = flex_dim[fid];
        let rest_len = flexedge_length0[i];
        crosssection[i] = match dim {
            1 => {
                // Cable: PI * radius^2
                let radius = if fid < flex_vertadr.len() {
                    let vert0 = flex_vertadr[fid];
                    if vert0 < flexvert_radius.len() {
                        flexvert_radius[vert0]
                    } else {
                        0.005
                    }
                } else {
                    0.005
                };
                std::f64::consts::PI * radius * radius
            }
            2 => {
                // Shell: thickness * rest_length
                let thickness = flex_thickness[fid];
                thickness * rest_len
            }
            3 => {
                // Solid: L^2 approximation
                rest_len * rest_len
            }
            _ => 1.0,
        };
    }
    crosssection
}

/// Compute per-flex address table from a per-item `flexid` array.
///
/// Returns a Vec of length `nflex` where `result[i]` is the index of the first
/// item belonging to flex `i` in the corresponding data array.
pub fn compute_flex_address_table(item_flexid: &[usize], nflex: usize) -> Vec<usize> {
    let mut adr = vec![0usize; nflex];
    let mut offset = 0;
    for (flex_id, a) in adr.iter_mut().enumerate() {
        *a = offset;
        offset += item_flexid.iter().filter(|&&fid| fid == flex_id).count();
    }
    adr
}

/// Compute per-flex count table from a per-item `flexid` array.
///
/// Returns a Vec of length `nflex` where `result[i]` is the number of items
/// belonging to flex `i`.
pub fn compute_flex_count_table(item_flexid: &[usize], nflex: usize) -> Vec<usize> {
    let mut count = vec![0usize; nflex];
    for &fid in item_flexid {
        if fid < nflex {
            count[fid] += 1;
        }
    }
    count
}

/// Compute per-vertex masses.
///
/// Two paths:
/// 1. **Uniform mass** (`flex.mass` is `Some`): distribute `mass / npnt` to every vertex
///    (including pinned). MuJoCo conformant — pinned share is silently discarded downstream.
/// 2. **Element-based lumping** (fallback): density * element_measure / vertices_per_element.
pub fn compute_vertex_masses(flex: &MjcfFlex) -> Vec<f64> {
    let npnt = flex.vertices.len();
    if npnt == 0 {
        return Vec::new();
    }

    // Path 1: uniform mass from <flexcomp mass="...">
    // No min-mass floor: mass=0 → per_vert=0 → invmass=0 (static vertices).
    // This matches MuJoCo semantics where explicit mass=0 pins vertices.
    if let Some(total_mass) = flex.mass {
        #[allow(clippy::cast_precision_loss)] // npnt is vertex count, never near 2^52
        let per_vert = total_mass / npnt as f64;
        return vec![per_vert; npnt];
    }

    // Path 2: element-based mass lumping from density
    let mut masses = vec![0.0f64; npnt];

    match flex.dim {
        1 => {
            let density = flex.density;
            for elem in &flex.elements {
                if elem.len() >= 2 {
                    let len = (flex.vertices[elem[1]] - flex.vertices[elem[0]]).norm();
                    let mass_per_vert = density * len / 2.0;
                    masses[elem[0]] += mass_per_vert;
                    masses[elem[1]] += mass_per_vert;
                }
            }
        }
        2 => {
            let area_density = flex.density * flex.thickness;
            for elem in &flex.elements {
                if elem.len() >= 3 {
                    let e1 = flex.vertices[elem[1]] - flex.vertices[elem[0]];
                    let e2 = flex.vertices[elem[2]] - flex.vertices[elem[0]];
                    let area = e1.cross(&e2).norm() / 2.0;
                    let mass_per_vert = area_density * area / 3.0;
                    masses[elem[0]] += mass_per_vert;
                    masses[elem[1]] += mass_per_vert;
                    masses[elem[2]] += mass_per_vert;
                }
            }
        }
        3 => {
            let density = flex.density;
            for elem in &flex.elements {
                if elem.len() >= 4 {
                    let e1 = flex.vertices[elem[1]] - flex.vertices[elem[0]];
                    let e2 = flex.vertices[elem[2]] - flex.vertices[elem[0]];
                    let e3 = flex.vertices[elem[3]] - flex.vertices[elem[0]];
                    let volume = e1.dot(&e2.cross(&e3)).abs() / 6.0;
                    let mass_per_vert = density * volume / 4.0;
                    masses[elem[0]] += mass_per_vert;
                    masses[elem[1]] += mass_per_vert;
                    masses[elem[2]] += mass_per_vert;
                    masses[elem[3]] += mass_per_vert;
                }
            }
        }
        _ => {}
    }

    // Ensure minimum mass
    for m in &mut masses {
        if *m < 1e-10 {
            *m = MIN_VERTEX_MASS;
        }
    }

    masses
}

/// Compute dihedral angle between two triangles sharing an edge.
fn compute_dihedral_angle(
    pe0: Vector3<f64>,
    pe1: Vector3<f64>,
    pa: Vector3<f64>,
    pb: Vector3<f64>,
) -> f64 {
    let e = pe1 - pe0;
    let e_len_sq = e.norm_squared();
    if e_len_sq < 1e-20 {
        return 0.0;
    }
    let offset_a = pa - pe0;
    let offset_b = pb - pe0;
    let normal_face_a = e.cross(&offset_a);
    let normal_face_b = offset_b.cross(&e);

    let norm_sq_a = normal_face_a.norm_squared();
    let norm_sq_b = normal_face_b.norm_squared();
    if norm_sq_a < 1e-20 || norm_sq_b < 1e-20 {
        return 0.0;
    }

    let e_dir = e / e_len_sq.sqrt();
    let normal_unit_a = normal_face_a / norm_sq_a.sqrt();
    let normal_unit_b = normal_face_b / norm_sq_b.sqrt();
    let cos_theta = normal_unit_a.dot(&normal_unit_b).clamp(-1.0, 1.0);
    let sin_theta = normal_unit_a.cross(&normal_unit_b).dot(&e_dir);
    sin_theta.atan2(cos_theta)
}

/// Compute per-flex edge constraint solref.
///
/// MuJoCo architecture note: Edge constraint solref comes from the equality
/// constraint definition (eq_solref), not from material properties. The
/// `<edge stiffness="..." damping="..."/>` attributes control passive spring-
/// damper forces (applied in `mj_fwd_passive`), not constraint solver params.
///
/// Our current architecture uses flex-level solref for all edge constraints,
/// which matches MuJoCo's behavior when no explicit equality constraint
/// overrides are defined.
fn compute_edge_solref(flex: &MjcfFlex) -> [f64; 2] {
    flex.solref
}

/// Compute bending stiffness from material properties (passive force coefficient).
///
/// For dim=2 (shells): Kirchhoff-Love thin plate bending stiffness
///   D = E * t^3 / (12 * (1 - nu^2))
/// For dim=3 (solids): characteristic stiffness D = E
///   (the Bridson dihedral gradient provides geometric scaling)
/// For dim=1 (cables): 0.0 (no bending)
fn compute_bend_stiffness_from_material(flex: &MjcfFlex) -> f64 {
    if flex.young <= 0.0 {
        return 0.0;
    }
    match flex.dim {
        2 => {
            let nu = flex.poisson.clamp(0.0, 0.499);
            flex.young * flex.thickness.powi(3) / (12.0 * (1.0 - nu * nu))
        }
        3 => flex.young,
        _ => 0.0,
    }
}

/// Compute bending damping from material properties.
///
/// Proportional damping model: b_bend = damping * k_bend.
fn compute_bend_damping_from_material(flex: &MjcfFlex, k_bend: f64) -> f64 {
    if flex.damping <= 0.0 || k_bend <= 0.0 {
        return 0.0;
    }
    flex.damping * k_bend
}

/// Cotangent of angle at v0 in triangle (v0, v1, v2).
///
/// Returns `dot(e1,e2) / |cross(e1,e2)| = cos(alpha) / sin(alpha) = cot(alpha)`.
/// MuJoCo equivalent: `cot()` in `user_mesh.cc:3645–3654`.
fn cot_angle(v0: Vector3<f64>, v1: Vector3<f64>, v2: Vector3<f64>) -> f64 {
    let e1 = v1 - v0;
    let e2 = v2 - v0;
    let cross = e1.cross(&e2);
    let cross_norm = cross.norm();
    if cross_norm < 1e-30 {
        return 0.0;
    }
    e1.dot(&e2) / cross_norm
}

/// Area of triangle (v0, v1, v2) = |cross(v1-v0, v2-v0)| / 2.
///
/// MuJoCo equivalent: `ComputeVolume()` in `user_mesh.cc:3655–3663`.
fn triangle_area(v0: Vector3<f64>, v1: Vector3<f64>, v2: Vector3<f64>) -> f64 {
    let e1 = v1 - v0;
    let e2 = v2 - v0;
    e1.cross(&e2).norm() / 2.0
}

/// Compute bending coefficients for one edge per MuJoCo's `ComputeBending()`.
///
/// `v[0]`, `v[1]`: edge endpoints (rest positions)
/// `v[2]`: opposite vertex in triangle 1
/// `v[3]`: opposite vertex in triangle 2
/// `mu`: shear modulus = young / (2 * (1 + poisson))
/// `thickness`: shell thickness
///
/// Returns `[f64; BENDING_COEFFS]`: `b[0..16]` = 4x4 stiffness matrix, `b[16]` = Garg curved ref.
///
/// MuJoCo equivalent: `ComputeBending()` in `user_mesh.cc:3666–3712`.
fn compute_bending_coefficients(
    v: [Vector3<f64>; 4],
    mu: f64,
    thickness: f64,
) -> [f64; BENDING_COEFFS] {
    let mut b = [0.0f64; BENDING_COEFFS];

    // Step 1: cotangent weights
    let a01 = cot_angle(v[0], v[1], v[2]); // angle at v[0] in tri 1
    let a02 = cot_angle(v[0], v[3], v[1]); // angle at v[0] in tri 2
    let a03 = cot_angle(v[1], v[2], v[0]); // angle at v[1] in tri 1
    let a04 = cot_angle(v[1], v[0], v[3]); // angle at v[1] in tri 2

    // Step 2: weight vector
    let c = [
        a03 + a04,    // sum of cots at v[1]
        a01 + a02,    // sum of cots at v[0]
        -(a01 + a03), // neg sum from tri 1
        -(a02 + a04), // neg sum from tri 2
    ];

    // Step 3: diamond volume (sum of triangle areas)
    let vol1 = triangle_area(v[0], v[1], v[2]);
    let vol2 = triangle_area(v[1], v[0], v[3]);
    let volume = vol1 + vol2;
    if volume < 1e-30 {
        return b; // degenerate diamond: all zero coefficients
    }

    // Step 4: material stiffness
    let stiffness = 3.0 * mu * thickness.powi(3) / (24.0 * volume);

    // Step 5: transport vectors and cos_theta
    let e0 = v[1] - v[0]; // shared edge
    let e1 = v[2] - v[0]; // to opp vertex in tri 1
    let e2 = v[3] - v[0]; // to opp vertex in tri 2
    let e3 = v[2] - v[1];
    let e4 = v[3] - v[1];
    let t0 = -(e1 * a03 + e3 * a01); // transport vector, tri 1
    let t1 = -(e2 * a04 + e4 * a02); // transport vector, tri 2
    let sqr = e0.dot(&e0); // |edge|^2
    let cos_theta = if sqr < 1e-30 {
        1.0 // degenerate edge: treat as flat
    } else {
        -t0.dot(&t1) / sqr
    };

    // Step 6: 4x4 stiffness matrix (outer product)
    for i in 0..4 {
        for j in 0..4 {
            b[4 * i + j] = c[i] * c[j] * cos_theta * stiffness;
        }
    }

    // Step 7: curved reference coefficient (Garg correction)
    let n = e0.cross(&e1); // normal to tri 1 (unnormalized)
    if sqr > 1e-30 {
        b[16] = n.dot(&e2) * (a01 - a03) * (a04 - a02) * stiffness / (sqr * sqr.sqrt());
    }

    b
}

/// Compute element adjacency for self-collision dispatch (Spec C S4).
///
/// Two elements are adjacent if they share at least one vertex. Returns
/// CSR-style flat sorted adjacency list: `(adj_data, adj_adr, adj_num)`.
pub fn compute_element_adjacency(
    flexelem_data: &[usize],
    flexelem_dataadr: &[usize],
    flexelem_datanum: &[usize],
    nflexelem: usize,
    nflexvert: usize,
) -> (Vec<usize>, Vec<usize>, Vec<usize>) {
    if nflexelem == 0 {
        return (vec![], vec![], vec![]);
    }

    // Step 1: vertex → element map
    let mut vert_to_elems: Vec<Vec<usize>> = vec![vec![]; nflexvert];
    for e in 0..nflexelem {
        let adr = flexelem_dataadr[e];
        let num = flexelem_datanum[e];
        for i in 0..num {
            let v = flexelem_data[adr + i];
            if v < nflexvert {
                vert_to_elems[v].push(e);
            }
        }
    }

    // Step 2: per-element adjacency
    let mut adj_data = Vec::new();
    let mut adj_adr = Vec::with_capacity(nflexelem);
    let mut adj_num = Vec::with_capacity(nflexelem);

    for e in 0..nflexelem {
        let adr = flexelem_dataadr[e];
        let num = flexelem_datanum[e];
        let mut neighbors = Vec::new();
        for i in 0..num {
            let v = flexelem_data[adr + i];
            if v < nflexvert {
                for &neighbor in &vert_to_elems[v] {
                    if neighbor != e {
                        neighbors.push(neighbor);
                    }
                }
            }
        }
        neighbors.sort_unstable();
        neighbors.dedup();

        adj_adr.push(adj_data.len());
        adj_num.push(neighbors.len());
        adj_data.extend(neighbors);
    }

    (adj_data, adj_adr, adj_num)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::{MIN_VERTEX_MASS, compute_vertex_masses};
    use crate::types::MjcfFlex;
    use nalgebra::Vector3;

    // -- compute_vertex_masses: mass attribute path --

    /// #27E AC1: mass attribute distributes uniformly across vertices
    #[test]
    fn test_vertex_masses_from_mass_attribute() {
        let mut flex = MjcfFlex::default();
        flex.dim = 2;
        flex.mass = Some(1.0);
        flex.vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
        ];
        flex.elements = vec![vec![0, 1, 2], vec![1, 3, 2]];

        let masses = compute_vertex_masses(&flex);

        assert_eq!(masses.len(), 4);
        // 1.0 / 4 = 0.25 per vertex
        for &m in &masses {
            assert!((m - 0.25).abs() < 1e-15, "expected 0.25, got {m}");
        }
    }

    /// #27E AC2: mass=0 -> per_vert=0 (static vertices, no min-mass floor)
    #[test]
    fn test_vertex_masses_zero_mass_produces_zero() {
        let mut flex = MjcfFlex::default();
        flex.dim = 2;
        flex.mass = Some(0.0);
        flex.vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        flex.elements = vec![vec![0, 1, 2]];

        let masses = compute_vertex_masses(&flex);

        assert_eq!(masses.len(), 3);
        for &m in &masses {
            assert!(
                m == 0.0,
                "mass=0 should produce per_vert=0.0 (static), got {m}"
            );
        }
    }

    /// #27E AC3: mass takes precedence over density
    #[test]
    fn test_vertex_masses_mass_overrides_density() {
        let mut flex = MjcfFlex::default();
        flex.dim = 2;
        flex.mass = Some(2.0);
        flex.density = 99999.0; // Should be ignored when mass is set
        flex.thickness = 0.01;
        flex.vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(0.5, 0.5, 0.0),
        ];
        flex.elements = vec![vec![0, 1, 4], vec![1, 3, 4], vec![3, 2, 4], vec![2, 0, 4]];

        let masses = compute_vertex_masses(&flex);

        assert_eq!(masses.len(), 5);
        // 2.0 / 5 = 0.4 per vertex (density is ignored)
        for &m in &masses {
            assert!((m - 0.4).abs() < 1e-15, "expected 0.4, got {m}");
        }
    }

    /// #27E: density path still applies min-mass floor
    #[test]
    fn test_vertex_masses_density_path_has_floor() {
        let mut flex = MjcfFlex::default();
        flex.dim = 2;
        flex.mass = None; // No mass attribute -> density path
        flex.density = 0.0; // Zero density -> zero mass per vertex before floor
        flex.thickness = 0.01;
        flex.vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        flex.elements = vec![vec![0, 1, 2]];

        let masses = compute_vertex_masses(&flex);

        assert_eq!(masses.len(), 3);
        // Density=0 -> element mass=0 -> min-mass floor kicks in -> MIN_VERTEX_MASS
        for &m in &masses {
            assert!(
                (m - MIN_VERTEX_MASS).abs() < 1e-15,
                "density path should apply min-mass floor {MIN_VERTEX_MASS}, got {m}"
            );
        }
    }
}
