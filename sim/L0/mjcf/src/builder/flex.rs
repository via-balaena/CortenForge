//! Flex deformable body processing.
//!
//! Converts MJCF `<flex>` / `<flexcomp>` elements into the flex body arrays.
//! A `<flex>`'s vertices are the bodies it names (MuJoCo 3.5.0's form); a
//! `<deformable><flexcomp>`'s each get a body of its own (+ 3 slide joints if
//! unpinned). Then the edges and hinges are extracted and the per-flex material
//! properties set.
//!
//! MuJoCo architecture: flex vertices ARE bodies in the kinematic tree.
//! `mj_crb`/`mj_rne` have zero flex-specific code — the standard rigid-body
//! pipeline handles everything.

use nalgebra::{UnitQuaternion, Vector3};

use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::keywords;
use crate::types::{FlexVertexBodies, MjcfFlex};
use sim_core::{EqualityType, FlexSelfCollide, MjJointType};

/// Number of coefficients per bending element in the discrete thin-shell model.
/// Each edge hinge stores a 4×4 symmetric matrix (16 entries) + 1 rest-angle scalar = 17 values.
const BENDING_COEFFS: usize = 17;

/// Minimum vertex mass (kg) applied when computed mass is near-zero.
/// Prevents singular mass matrices in the deformable body integrator.
const MIN_VERTEX_MASS: f64 = 0.001;

/// MuJoCo's `mjMINVAL` (`mjtnum.h`).
const MJ_MINVAL: f64 = 1e-15;

impl ModelBuilder {
    /// Process flex deformable bodies: a `<flex>`'s vertices on the bodies it names, a
    /// flexcomp's on a body each (+ 3 slide joints if unpinned); extract edges/hinges,
    /// populate Model flex_* arrays.
    ///
    /// MuJoCo architecture: flex vertices ARE bodies in the kinematic tree. `mj_crb`/`mj_rne`
    /// have zero flex-specific code — the standard rigid-body pipeline handles everything.
    pub(crate) fn process_flex_bodies(&mut self, flex_list: &[MjcfFlex]) -> Result<()> {
        for (index, flex_orig) in flex_list.iter().enumerate() {
            let at = || Location::nth(ElementKind::Flex, Some(&flex_orig.name), index);
            // A `<flex>`'s vertices are the bodies it names; a flexcomp's get
            // bodies of their own below.
            let resolved;
            let mut vertex_bodies = None;
            let flex = if flex_orig.vertex_bodies == FlexVertexBodies::Named {
                let (named, bodies) = self
                    .resolve_vertex_bodies(flex_orig)
                    .map_err(|e| e.with_location_if_unknown(at()))?;
                resolved = named;
                vertex_bodies = Some(bodies);
                &resolved
            } else {
                // A flexcomp's checks; a `<flex>` has had them above.
                check_flexcomp(flex_orig)
                    .and_then(|()| check_flex_elements(flex_orig))
                    .map_err(|e| e.with_location_if_unknown(at()))?;
                flex_orig
            };
            // Before the masses, edges, hinges and elements read the order.
            let oriented;
            let flex = if flex.dim == 3 {
                let mut elements = flex.elements.clone();
                orient_tetrahedra(&flex.vertices, &mut elements);
                oriented = MjcfFlex {
                    elements,
                    ..flex.clone()
                };
                &oriented
            } else {
                flex
            };
            let flex_id = self.nflex;
            self.nflex += 1;

            let vert_start = self.nflexvert;

            if let Some(bodies) = &vertex_bodies {
                for (&body_id, pos) in bodies.iter().zip(&flex.vertices) {
                    self.push_named_flex_vertex(flex, body_id, pos, flex_id);
                }
            } else {
                // Compute per-vertex masses (uniform from mass attr, or element-based lumping)
                let vertex_masses = compute_vertex_masses(flex);

                // Create a body (+ joints for unpinned) per vertex
                for (i, pos) in flex.vertices.iter().enumerate() {
                    let mass = vertex_masses[i];
                    let pinned = flex.pinned.contains(&i);
                    self.create_flex_vertex_body(flex, pos, mass, pinned, flex_id);
                }
            }

            // A flex whose vertices are named bodies is rigid when they are one
            // body (`mjCFlex::Compile`, `user_mesh.cc:4205-4213`), which
            // `resolve_vertex_bodies` refuses. Where the loader makes the
            // vertex bodies: when no vertex has an inverse mass.
            let rigid = vertex_bodies.is_none()
                && (vert_start..vert_start + flex.vertices.len())
                    .all(|v| self.flexvert_invmass[v] == 0.0);

            // Extract edges from element connectivity
            let (edge_start, edge_key_to_global) =
                self.extract_flex_edges(flex, vert_start, flex_id, rigid);

            // Extract bending hinges from adjacent elements + compute bending coefficients
            self.extract_flex_bending_hinges(
                flex,
                vert_start,
                flex_id,
                edge_start,
                &edge_key_to_global,
            );

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
                    // Unsigned: every tetrahedron is turned the same way above.
                    (e1.dot(&e2.cross(&e3)) / 6.0).abs()
                } else {
                    0.0
                };
                self.flexelem_volume0.push(rest_vol);
                self.nflexelem += 1;
            }

            // The flex's edge equality, with the edge solref and the flex's
            // solimp: every flex holds its edges until the parser reads
            // `<equality><flex>` (Rigid-loading L31).
            let edge_solref = compute_edge_solref(flex);
            self.eq_type.push(EqualityType::Flex);
            self.eq_obj1id.push(flex_id);
            self.eq_obj2id.push(usize::MAX);
            self.eq_data.push([0.0; 11]);
            self.eq_active.push(true);
            self.eq_solref.push(edge_solref);
            self.eq_solimp.push(flex.solimp);
            self.eq_name.push(None);
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
            self.flex_radius.push(flex.radius);
            self.flex_priority.push(flex.priority);
            self.flex_solmix.push(flex.solmix);
            self.flex_solref.push(flex.solref);
            self.flex_solimp.push(flex.solimp);
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
            // MuJoCo's default is `auto`.
            let sc = match flex.selfcollide.as_deref() {
                Some(s) => keywords::flex_selfcollide(s)
                    .ok_or_else(|| MjcfError::invalid_keyword("flex", "selfcollide", s).at(at()))?,
                None => FlexSelfCollide::Auto,
            };
            self.flex_selfcollide.push(sc);
            // DT-85: flex contact runtime attributes
            self.flex_internal.push(flex.internal);
            self.flex_activelayers.push(flex.activelayers);
            self.flex_vertcollide.push(flex.vertcollide);
            self.flex_passive.push(flex.passive);
            self.flex_edgestiffness.push(flex.edge_stiffness);
            self.flex_edgedamping.push(flex.edge_damping);

            self.flex_rigid.push(rigid);

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

    /// The body `body` is welded to: itself if it has joints, else its
    /// parent's, the world's id 0 at the root (MuJoCo's `weldid`,
    /// `user_objects.cc:2393`).
    fn weld_id(&self, mut body: usize) -> usize {
        while body != 0 && self.body_jnt_num[body] == 0 {
            body = self.body_parent[body];
        }
        body
    }

    /// Whether a body joined to `body` moves with it: one with joints at or
    /// above `body`, or, when `mocap_too`, a mocap body there, which moves
    /// without joints.
    fn moves_with(&self, mut body: usize, mocap_too: bool) -> bool {
        while body != 0 {
            if self.body_jnt_num[body] > 0 || (mocap_too && self.body_mocapid[body].is_some()) {
                return true;
            }
            body = self.body_parent[body];
        }
        false
    }

    /// A `<flex>` whose vertices are the bodies it names, given MuJoCo 3.5.0's
    /// checks of its elements, vertices, bodies, nodes, Poisson ratio and a
    /// node flex's element count in its order (`mjCFlex::Compile` and
    /// `ResolveReferences`, `user_mesh.cc:4076-4202`, `:4331-4343`; not
    /// `material` or `texcoord`, which this crate does not read,
    /// Rigid-loading L47's), then held to the
    /// subset sim-core's flex code takes: each vertex on a body of its own (a
    /// flex on one body is MuJoCo's rigid flex), at its body's origin, and
    /// each body either jointless with no joint above it, or with exactly
    /// three slides along x, y and z, in that order, unrotated, with no joint
    /// or mocap body above it. sim-core reads a vertex's three dofs as its
    /// world x, y and z (the edge Jacobian, the passive forces) and a vertex
    /// without dofs as fixed, so anything else is refused, a stated
    /// limitation.
    ///
    /// Returns the flex with each vertex's world position, its body's at
    /// compile time (MuJoCo's centered flex: `vertxpos` is the body's
    /// `xpos0`), and the vertex bodies' ids.
    fn resolve_vertex_bodies(&self, flex: &MjcfFlex) -> Result<(MjcfFlex, Vec<usize>)> {
        let refuse = |attribute: &str, reason: &str| {
            MjcfError::invalid_value("flex", attribute, reason.to_string())
        };
        // MuJoCo's checks first, in its order; its rigid flex (one body for
        // every vertex) passes them.
        check_flex_dim_and_elements(flex)?;
        let interpolated = !flex.node.is_empty();
        if flex.body.is_empty() && !interpolated {
            return Err(refuse("body", "vertbody and nodebody are both empty"));
        }
        check_flex_edge_stiffness(flex)?;
        if interpolated && flex.selfcollide.as_deref() != Some("none") {
            return Err(refuse(
                "node",
                "trilinear interpolation cannot do self-collision",
            ));
        }
        if interpolated && flex.internal {
            return Err(refuse(
                "node",
                "trilinear interpolation cannot do internal collisions",
            ));
        }
        let nvert = if flex.vertices.is_empty() {
            flex.body.len()
        } else if flex.body.len() == 1 || flex.body.len() == flex.vertices.len() {
            flex.vertices.len()
        } else {
            return Err(refuse("body", "vertbody size must be 1 or nvert"));
        };
        check_flex_vertex_count(flex, nvert)?;
        if interpolated {
            // MuJoCo's expression, `pow(nnode, 1.0 / 3) - 1` truncated to an
            // int (`:4150-4157`), whose truncation the check must repeat (not
            // `cbrt`); a node count is far below 2^52.
            let nnode = flex.node.len();
            #[allow(
                clippy::imprecise_flops,
                clippy::cast_precision_loss,
                clippy::cast_possible_truncation
            )]
            let order = ((nnode as f64).powf(1.0 / 3.0) - 1.0) as i64;
            if usize::try_from((order + 1).pow(3)) != Ok(nnode) {
                return Err(refuse(
                    "node",
                    &format!("number of nodes must be {order}^3 but it is {nnode}"),
                ));
            }
        }
        check_flex_element_ids(flex, nvert)?;
        let mut bodies = Vec::with_capacity(flex.body.len());
        for name in flex.body.iter().chain(&flex.node) {
            let &id = self
                .body_name_to_id
                .get(name)
                .ok_or_else(|| MjcfError::undefined(ElementKind::Body, name))?;
            bodies.push(id);
        }
        bodies.truncate(flex.body.len());
        check_flex_repeated_vertices(flex)?;
        check_flex_poisson(flex)?;
        check_flex_node_elements(flex)?;

        // The subset sim-core takes; the error's location names the flex.
        if interpolated {
            return Err(MjcfError::unsupported(
                "a flex's node (MuJoCo's trilinear interpolation)",
            ));
        }
        if bodies.len() == 1 && nvert > 1 {
            return Err(MjcfError::unsupported(
                "one body for every vertex (MuJoCo's rigid flex)",
            ));
        }
        for (k, (name, id)) in flex.body.iter().zip(&bodies).enumerate() {
            if bodies[..k].contains(id) {
                return Err(MjcfError::unsupported(format!(
                    "body '{name}' holds two of its vertices"
                )));
            }
        }
        if flex.vertices.iter().any(|v| *v != Vector3::zeros()) {
            return Err(MjcfError::unsupported(
                "a vertex away from its body's origin (a nonzero vertex offset)",
            ));
        }
        let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
        for (name, &id) in flex.body.iter().zip(&bodies) {
            let (adr, num) = (self.body_jnt_adr[id], self.body_jnt_num[id]);
            let three_slides = num == 3
                && (0..3).all(|k| {
                    self.jnt_type[adr + k] == MjJointType::Slide
                        && self.jnt_axis[adr + k] == axes[k]
                });
            if num != 0 && !three_slides {
                return Err(MjcfError::unsupported(format!(
                    "vertex body '{name}' has joints other than three slides along x, y and z"
                )));
            }
            if id == 0 {
                continue;
            }
            if num != 0 && self.body_world_quat[id - 1].angle() != 0.0 {
                return Err(MjcfError::unsupported(format!(
                    "vertex body '{name}' is rotated"
                )));
            }
            // Slides turn with a mocap body above them; a jointless vertex
            // on one has no dofs to misread.
            if self.moves_with(self.body_parent[id], num != 0) {
                return Err(MjcfError::unsupported(format!(
                    "vertex body '{name}' moves with a body above it"
                )));
            }
        }

        let vertices = bodies
            .iter()
            .map(|&id| {
                if id == 0 {
                    Vector3::zeros()
                } else {
                    self.body_world_pos[id - 1]
                }
            })
            .collect();
        Ok((
            MjcfFlex {
                vertices,
                ..flex.clone()
            },
            bodies,
        ))
    }

    /// One vertex of a flex whose vertices are named bodies: the `flexvert_*`
    /// entries of the vertex at world position `pos` on body `body_id`, its
    /// mass the body's and its dofs the body's three slides, or none.
    fn push_named_flex_vertex(
        &mut self,
        flex: &MjcfFlex,
        body_id: usize,
        pos: &Vector3<f64>,
        flex_id: usize,
    ) {
        let mass = self.body_mass[body_id];
        let moves = self.body_dof_num[body_id] > 0;
        self.nflexvert += 1;
        if moves {
            let first = self.body_jnt_adr[body_id];
            self.flexvert_qposadr.push(self.jnt_qpos_adr[first]);
            self.flexvert_dofadr.push(self.body_dof_adr[body_id]);
        } else {
            self.flexvert_qposadr.push(usize::MAX);
            self.flexvert_dofadr.push(usize::MAX);
        }
        self.flexvert_mass.push(mass);
        self.flexvert_invmass
            .push(if moves && mass > 0.0 { 1.0 / mass } else { 0.0 });
        self.flexvert_radius.push(flex.radius);
        self.flexvert_flexid.push(flex_id);
        self.flexvert_bodyid.push(body_id);
        self.flexvert_initial_pos.push(*pos);
    }

    /// Create a body (+ 3 slide joints for unpinned vertices) for one vertex
    /// of a `<deformable><flexcomp>`, on the world body.
    ///
    /// Pushes body, joint, DOF, qpos, and flexvert arrays for the vertex at `pos`.
    /// Pinned vertices get a body with no joints/DOFs; unpinned get 3 slide joints (X, Y, Z),
    /// each with the flex's damping.
    fn create_flex_vertex_body(
        &mut self,
        flex: &MjcfFlex,
        pos: &Vector3<f64>,
        mass: f64,
        pinned: bool,
        flex_id: usize,
    ) {
        let axes = [Vector3::x(), Vector3::y(), Vector3::z()];

        // `check_flexcomp` has refused a mass below mjMINVAL, and the density
        // path floors each vertex's, so `mass` is positive.
        let inv_mass = if pinned { 0.0 } else { 1.0 / mass };

        self.nflexvert += 1;

        // --- Create body for this vertex ---
        let body_id = self.body_parent.len();

        // A flexcomp's vertex bodies hang off the world body, each the root of
        // its own kinematic tree.
        let jnt_adr = self.jnt_type.len();
        let dof_adr = self.nv;
        let qpos_start = self.nq;
        let geom_adr = self.geom_type.len();

        // Push body arrays (mirrors process_body_with_world_frame pattern)
        self.body_parent.push(0);
        self.body_rootid.push(body_id);
        self.body_jnt_adr.push(jnt_adr);
        self.body_dof_adr.push(dof_adr);
        self.body_geom_adr.push(geom_adr);
        // On the world body, the local offset is the world position.
        self.body_pos.push(*pos);
        self.body_quat.push(UnitQuaternion::identity());
        self.body_ipos.push(Vector3::zeros()); // CoM at the body origin
        self.body_iquat.push(UnitQuaternion::identity());
        self.body_mass.push(mass);
        // MuJoCo's flexcomp body inertia (`user_flexcomp.cc:471`, `:474-477`).
        self.body_inertia.push(Vector3::repeat(
            mass * (2.0 * flex.inertiabox * flex.inertiabox) / 3.0,
        ));
        self.body_mocapid.push(None);
        self.body_name.push(None);
        self.body_sleep_policy.push(None);
        // The defaults of a body written with none of these (`body.rs`).
        self.body_gravcomp.push(0.0);
        self.body_user_raw.push(Vec::new());
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
            // Unpinned vertex: 3 slide joints (X, Y, Z). The first dof has no
            // parent (the world body has no dofs); each next one chains to the
            // one before it.
            let mut last_dof: Option<usize> = None;

            for axis in &axes {
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
                // The flex's damping as each slide's (MuJoCo's flex damping
                // acts only through its elasticity and bending).
                self.jnt_damping.push(flex.damping);
                self.jnt_armature.push(0.0);
                // The defaults of a joint written with none of these (`joint.rs`).
                self.jnt_solref.push(DEFAULT_SOLREF);
                self.jnt_solimp.push(DEFAULT_SOLIMP);
                self.jnt_name.push(None);
                self.jnt_group.push(0);
                self.jnt_actgravcomp.push(false);
                self.jnt_margin.push(0.0);
                self.jnt_user_raw.push(Vec::new());

                // DOF arrays
                self.dof_parent.push(last_dof);
                self.dof_body.push(body_id);
                self.dof_jnt.push(jnt_id);
                self.dof_armature.push(0.0);
                self.dof_damping.push(flex.damping);
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

            self.flexvert_qposadr.push(qpos_start); // first of 3 slide joint qpos addresses
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

    /// Extract edges from element connectivity for one flex.
    ///
    /// Numbers the edges as MuJoCo's compiler does (`mjCFlex::Compile`,
    /// `user_mesh.cc:4297-4321`): each edge, `(min, max)` of its two vertices,
    /// where it first appears, the elements in order and each element's edges in
    /// [`ELEMENT_EDGES`] order. Pushes the `flexedge_*` arrays and returns
    /// `(edge_start, edge_key_to_global)`, the map read only to look an edge up.
    fn extract_flex_edges(
        &mut self,
        flex: &MjcfFlex,
        vert_start: usize,
        flex_id: usize,
        rigid: bool,
    ) -> (usize, std::collections::HashMap<(usize, usize), usize>) {
        use std::collections::HashMap;

        let edge_start = self.nflexedge;
        let mut edge_key_to_global: HashMap<(usize, usize), usize> = HashMap::new();
        let element_edges = ELEMENT_EDGES[flex.dim - 1];
        for elem in &flex.elements {
            for &(i, j) in element_edges {
                let (a, b) = (elem[i].min(elem[j]), elem[i].max(elem[j]));
                if edge_key_to_global.contains_key(&(a, b)) {
                    continue;
                }
                self.push_flex_edge(flex, vert_start, flex_id, a, b, rigid);
                edge_key_to_global.insert((a, b), self.nflexedge);
                self.nflexedge += 1;
            }
        }

        (edge_start, edge_key_to_global)
    }

    /// One edge's `flexedge_*` entries, between the flex's vertices `a < b`,
    /// of a flex that is `rigid` or not.
    fn push_flex_edge(
        &mut self,
        flex: &MjcfFlex,
        vert_start: usize,
        flex_id: usize,
        a: usize,
        b: usize,
        rigid: bool,
    ) {
        let rest_len = (flex.vertices[b] - flex.vertices[a]).norm();
        self.flexedge_vert.push([vert_start + a, vert_start + b]);
        self.flexedge_length0.push(rest_len);
        self.flexedge_flexid.push(flex_id);
        // Every edge of a rigid flex, and an edge whose ends' bodies are
        // welded together, is rigid (MuJoCo's rule, `user_model.cc:3511-3519`).
        // No document makes the first half decide an edge: a flexcomp is
        // rigid only when every vertex is pinned, and the pinned vertices'
        // bodies all weld to the world.
        let weld = |v: usize| self.weld_id(self.flexvert_bodyid[vert_start + v]);
        self.flexedge_rigid.push(rigid || weld(a) == weld(b));
        // §42B S1: Initialize flexedge_flap to [-1, -1] (boundary default).
        self.flexedge_flap.push([-1, -1]);
        // §42B S2+S3: Initialize flex_bending to zeros per edge.
        self.flex_bending.extend_from_slice(&[0.0; BENDING_COEFFS]);
    }

    /// Extract bending hinges from adjacent elements and compute bending coefficients.
    ///
    /// For each interior edge (shared by exactly 2 elements), finds the opposite
    /// vertices, computes the rest dihedral angle, populates `flexhinge_*` arrays,
    /// and fills `flexedge_flap`. Then computes cotangent bending coefficients for
    /// dim==2 shells with valid material properties.
    fn extract_flex_bending_hinges(
        &mut self,
        flex: &MjcfFlex,
        vert_start: usize,
        flex_id: usize,
        edge_start: usize,
        edge_key_to_global: &std::collections::HashMap<(usize, usize), usize>,
    ) {
        // Each edge's elements, in element order, the edges in their order.
        let mut edge_elements: Vec<Vec<usize>> = vec![Vec::new(); self.nflexedge - edge_start];
        for (elem_idx, verts) in flex.elements.iter().enumerate() {
            for &(i, j) in ELEMENT_EDGES[flex.dim - 1] {
                let key = (verts[i].min(verts[j]), verts[i].max(verts[j]));
                if let Some(&global_e) = edge_key_to_global.get(&key) {
                    edge_elements[global_e - edge_start].push(elem_idx);
                }
            }
        }
        for (local_e, elems) in edge_elements.iter().enumerate() {
            if elems.len() != 2 {
                continue; // boundary edge
            }
            let global_e = edge_start + local_e;
            let ve0 = self.flexedge_vert[global_e][0] - vert_start;
            let ve1 = self.flexedge_vert[global_e][1] - vert_start;
            let opp_a = flex.elements[elems[0]]
                .iter()
                .find(|&&v| v != ve0 && v != ve1)
                .copied();
            let opp_b = flex.elements[elems[1]]
                .iter()
                .find(|&&v| v != ve0 && v != ve1)
                .copied();
            if let (Some(va), Some(vb)) = (opp_a, opp_b) {
                let rest_angle = compute_dihedral_angle(
                    flex.vertices[ve0],
                    flex.vertices[ve1],
                    flex.vertices[va],
                    flex.vertices[vb],
                );
                self.flexhinge_vert.push([
                    vert_start + ve0,
                    vert_start + ve1,
                    vert_start + va,
                    vert_start + vb,
                ]);
                self.flexhinge_angle0.push(rest_angle);
                self.flexhinge_flexid.push(flex_id);
                self.nflexhinge += 1;

                // §42B S1: Populate flexedge_flap for this interior edge.
                #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
                {
                    self.flexedge_flap[global_e] =
                        [(vert_start + va) as i32, (vert_start + vb) as i32];
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
    }
}

// ============================================================================
// Flex helper functions
// ============================================================================

/// The edges of an element of each dimension, as pairs of its vertex slots, in
/// MuJoCo's order (`eledge`, `user_mesh.cc:3447-3452`): a cable `(0,1)`, a
/// triangle `(1,2) (2,0) (0,1)`, a tetrahedron `(0,1) (1,2) (2,0) (2,3) (0,3)
/// (1,3)`. Indexed by `dim - 1`.
const ELEMENT_EDGES: [&[(usize, usize)]; 3] = [
    &[(0, 1)],
    &[(1, 2), (2, 0), (0, 1)],
    &[(0, 1), (1, 2), (2, 0), (2, 3), (0, 3), (1, 3)],
];

/// A flexcomp's own checks, before its flex's: no `body` or `node`, which
/// MuJoCo 3.5.0's `<flexcomp>` does not have (its schema,
/// `xml_native_reader.cc:312-315`; the parser refuses `body` on one, and an
/// [`MjcfFlex`] built in code can carry either), then MuJoCo's check of its
/// mass and `inertiabox` (`mjCFlexcomp::Make`, `user_flexcomp.cc:143-145`).
fn check_flexcomp(flex: &MjcfFlex) -> Result<()> {
    for (attribute, names) in [("body", &flex.body), ("node", &flex.node)] {
        if !names.is_empty() {
            return Err(MjcfError::unknown_attribute("flexcomp", attribute));
        }
    }
    let low_mass = flex.mass.is_some_and(|mass| mass < MJ_MINVAL);
    if low_mass || flex.inertiabox < MJ_MINVAL {
        return Err(MjcfError::invalid_value(
            "flexcomp",
            if low_mass { "mass" } else { "inertiabox" },
            "Mass and inertiabox must be larger than mjMINVAL",
        ));
    }
    Ok(())
}

/// MuJoCo's checks of a flexcomp's elements, edge stiffness and Poisson ratio before anything
/// is built from them (`mjCFlex::Compile`, `user_mesh.cc`), in its order, as
/// the functions below make them. A `<flex>` takes them one by one in
/// `resolve_vertex_bodies`, among the checks of its bodies, and not here.
fn check_flex_elements(flex: &MjcfFlex) -> Result<()> {
    check_flex_dim_and_elements(flex)?;
    check_flex_edge_stiffness(flex)?;
    check_flex_vertex_count(flex, flex.vertices.len())?;
    check_flex_element_ids(flex, flex.vertices.len())?;
    check_flex_repeated_vertices(flex)?;
    check_flex_poisson(flex)
}

/// With a Young's modulus, a Poisson ratio in [0, 0.5), as MuJoCo's
/// compiler requires (`user_mesh.cc:4331-4334`).
fn check_flex_poisson(flex: &MjcfFlex) -> Result<()> {
    if flex.young > 0.0 && !(0.0..0.5).contains(&flex.poisson) {
        return Err(MjcfError::invalid_value(
            "elasticity",
            "poisson",
            "Poisson ratio must be in [0, 0.5)",
        ));
    }
    Ok(())
}

/// With a Young's modulus, a node flex's elements, at least `(3 n)² / 21`
/// for its `n` nodes (integer division), as MuJoCo's compiler requires
/// (`user_mesh.cc:4339-4343`). MuJoCo's message prints a number that is not
/// the count (its format call passes a string where the count goes;
/// measured); this one prints the count.
fn check_flex_node_elements(flex: &MjcfFlex) -> Result<()> {
    let coordinates = 3 * flex.node.len();
    let least = coordinates * coordinates / 21;
    if flex.young > 0.0 && !flex.node.is_empty() && flex.elements.len() < least {
        return Err(refuse_flex(
            "element",
            &format!("Trilinear dofs are require at least {least} elements"),
        ));
    }
    Ok(())
}

fn refuse_flex(attribute: &str, reason: &str) -> MjcfError {
    MjcfError::invalid_value("flex", attribute, reason)
}

/// MuJoCo's first checks of a flex (`mjCFlex::Compile`, `user_mesh.cc:
/// 4108-4116`): its dim, and its elements, some, each of `dim + 1` vertices.
fn check_flex_dim_and_elements(flex: &MjcfFlex) -> Result<()> {
    if !(1..=3).contains(&flex.dim) {
        return Err(refuse_flex("dim", "dim must be 1, 2 or 3"));
    }
    if flex.elements.is_empty() {
        return Err(refuse_flex("element", "elem is empty"));
    }
    if flex.elements.iter().any(|e| e.len() != flex.dim + 1) {
        return Err(refuse_flex(
            "element",
            "elem size must be multiple of (dim+1)",
        ));
    }
    Ok(())
}

/// A positive edge stiffness only on a flex of dim 1 (`user_mesh.cc:
/// 4123-4125`). MuJoCo's passive edge spring has no dim check
/// (`engine_passive.c:413-443`), nor has sim-core's, so a model built in
/// code still reaches it.
fn check_flex_edge_stiffness(flex: &MjcfFlex) -> Result<()> {
    if flex.edge_stiffness > 0.0 && flex.dim > 1 {
        return Err(MjcfError::invalid_value(
            "edge",
            "stiffness",
            "edge stiffness only available for dim=1, please use elasticity plugins",
        ));
    }
    Ok(())
}

/// Its vertex count against its dim (`user_mesh.cc:4146-4148`).
fn check_flex_vertex_count(flex: &MjcfFlex, nvert: usize) -> Result<()> {
    if nvert < flex.dim + 1 {
        return Err(refuse_flex("vertex", "not enough vertices"));
    }
    Ok(())
}

/// Each element's vertex ids against its vertex count (`user_mesh.cc:
/// 4160-4165`).
fn check_flex_element_ids(flex: &MjcfFlex, nvert: usize) -> Result<()> {
    if flex.elements.iter().flatten().any(|&v| v >= nvert) {
        return Err(refuse_flex("element", "elem vertex id out of range"));
    }
    Ok(())
}

/// A vertex twice in one element, checked after its bodies are named
/// (`user_mesh.cc:4189-4202`).
fn check_flex_repeated_vertices(flex: &MjcfFlex) -> Result<()> {
    for element in &flex.elements {
        let mut sorted = element.clone();
        sorted.sort_unstable();
        if sorted.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(refuse_flex("element", "repeated vertex in element"));
        }
    }
    Ok(())
}

/// Each tetrahedron of `elements` turned as MuJoCo's compiler turns it, so
/// its faces (0,1,2) (0,2,3) (0,3,1) (1,3,2) are right-handed outward
/// (`mjCFlex::Compile`, `user_mesh.cc:4272-4293`): where
/// `(v1 − v0) × (v2 − v0) · (v3 − v0)` is positive at the vertices' positions
/// `world_pos`, vertices 1 and 2 swap.
fn orient_tetrahedra(world_pos: &[Vector3<f64>], elements: &mut [Vec<usize>]) {
    for element in elements {
        if let [i0, i1, i2, i3] = element[..] {
            let (v0, v1, v2, v3) = (world_pos[i0], world_pos[i1], world_pos[i2], world_pos[i3]);
            let (a, b, c) = (v1 - v0, v2 - v0, v3 - v0);
            let nrm = [
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x,
            ];
            if nrm[0] * c.x + nrm[1] * c.y + nrm[2] * c.z > 0.0 {
                element.swap(1, 2);
            }
        }
    }
}

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

    // Path 1: uniform mass from <flexcomp mass="...">, at least mjMINVAL
    // (`check_flexcomp`), so no floor.
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
