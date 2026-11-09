//! Flex deformable body processing.
//!
//! Converts MJCF `<flex>` elements, and the flexes body-level `<flexcomp>`s
//! make, into the flex body arrays. A `<flex>`'s vertices are the bodies it
//! names (MuJoCo 3.5.0's form); a flexcomp's are the bodies it made, a pinned
//! point on a jointless body made here. Then the edges and hinges are extracted
//! and the per-flex material properties set.
//!
//! MuJoCo architecture: flex vertices ARE bodies in the kinematic tree.
//! `mj_crb`/`mj_rne` have zero flex-specific code — the standard rigid-body
//! pipeline handles everything.

use nalgebra::{UnitQuaternion, Vector3};

use super::ModelBuilder;
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::keywords;
use crate::types::MjcfFlex;
use sim_core::{FlexSelfCollide, MjJointType};

/// A flex whose vertices are named bodies, as the builder takes it: each
/// vertex's world position, what each vertex is on, and whether the flex is
/// rigid.
struct NamedVertices {
    flex: MjcfFlex,
    on: Vec<VertexOn>,
    rigid: bool,
}

/// MuJoCo's rule for a flex's `rigid` (`user_mesh.cc:4204-4213`): every
/// vertex on one body, and never an interpolated flex.
fn mujoco_rigid(flex: &MjcfFlex, bodies: &[usize]) -> bool {
    flex.node.is_empty() && bodies.windows(2).all(|w| w[0] == w[1])
}

/// What a vertex of a flex with named vertex bodies is on.
#[derive(Debug, Clone, Copy)]
enum VertexOn {
    /// The body of this id.
    Body(usize),
    /// A jointless body the builder makes for it (a pinned flexcomp point).
    StandIn,
}

/// Number of coefficients per bending element in the discrete thin-shell model.
/// Each edge hinge stores a 4×4 symmetric matrix (16 entries) + 1 rest-angle scalar = 17 values.
const BENDING_COEFFS: usize = 17;

impl ModelBuilder {
    /// Process flex deformable bodies: a `<flex>`'s vertices on the bodies it
    /// names, a body-level flexcomp's on the bodies it made (a pinned point on
    /// a jointless body made here); extract edges/hinges, populate Model
    /// flex_* arrays.
    ///
    /// MuJoCo architecture: flex vertices ARE bodies in the kinematic tree. `mj_crb`/`mj_rne`
    /// have zero flex-specific code — the standard rigid-body pipeline handles everything.
    pub(crate) fn process_flex_bodies(&mut self, flex_list: &[MjcfFlex]) -> Result<()> {
        for (index, flex_orig) in flex_list.iter().enumerate() {
            let at = || Location::nth(ElementKind::Flex, Some(&flex_orig.name), index);
            // A `<flex>`'s vertices are the bodies it names, a flexcomp's the
            // bodies it made.
            let named = match self.resolve_vertex_bodies(flex_orig) {
                Ok(named) => named,
                // A flexcomp's flex sim-core does not take: built with every
                // vertex fixed where it is, refused after the keyframes.
                Err(e @ MjcfError::Unsupported { .. }) if flex_orig.flexcomp_holder.is_some() => {
                    self.flexcomp_fault
                        .get_or_insert_with(|| (index, e.with_location_if_unknown(at())));
                    self.fixed_flexcomp_vertices(flex_orig)
                }
                Err(e) => return Err(e.with_location_if_unknown(at())),
            };
            let flex = &named.flex;
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

            for (vertex, pos) in named.on.iter().zip(&flex.vertices) {
                match *vertex {
                    VertexOn::Body(body_id) => {
                        self.push_named_flex_vertex(flex, body_id, pos, flex_id);
                    }
                    // A pinned flexcomp point: a jointless body of its own at
                    // its world position (D-FLEX-PINNED-BODY), massless, as
                    // MuJoCo makes no body for it.
                    VertexOn::StandIn => {
                        // The mass pipeline's bounds skip it: MuJoCo has no such body.
                        self.flex_stand_ins.push(self.body_parent.len());
                        self.create_flex_stand_in(flex, pos, flex_id);
                    }
                }
            }

            // Rigid when every vertex is on one body (`mjCFlex::Compile`,
            // `user_mesh.cc:4205-4213`), which `resolve_vertex_bodies` refuses
            // of a `<flex>` and takes of a flexcomp's (every point on its
            // holding body).
            let rigid = named.rigid;

            // Extract edges from element connectivity
            let (edge_start, edge_key_to_global) =
                self.extract_flex_edges(flex, vert_start, flex_id, rigid);

            // A dim-2 flex's flaps, hinges and bending coefficients
            self.extract_flex_flaps(flex, vert_start, flex_id, edge_start, &edge_key_to_global);

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
    /// checks of its elements, edge stiffness, vertices, bodies, nodes,
    /// Poisson ratio and a node flex's element count in its order
    /// (`mjCFlex::Compile` and `ResolveReferences`, `user_mesh.cc:4076-4202`,
    /// `:4331-4343`; not `material` or `texcoord`, which this crate does not
    /// read, Rigid-loading L47's), then held to the subset sim-core's flex
    /// code takes: each vertex on a body of its own (a flex on one body is
    /// MuJoCo's rigid flex), at its body's origin, and
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
    fn resolve_vertex_bodies(&self, flex: &MjcfFlex) -> Result<NamedVertices> {
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
        if let Some(holder) = flex.flexcomp_holder.as_deref() {
            return self.resolve_flexcomp_vertices(flex, holder, &bodies);
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
        for (name, &id) in flex.body.iter().zip(&bodies) {
            self.check_vertex_body(name, id)?;
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
        Ok(NamedVertices {
            flex: MjcfFlex {
                vertices,
                ..flex.clone()
            },
            on: bodies.into_iter().map(VertexOn::Body).collect(),
            rigid: false,
        })
    }

    /// A body-level flexcomp's flex, past MuJoCo's checks of a flex: each
    /// vertex on the holding body (a pinned point, or every point of a rigid
    /// flexcomp) gets a jointless body of its own at its world position
    /// (`D-FLEX-PINNED-BODY`), which needs the holding body fixed to the
    /// world; each other vertex is on the body the flexcomp made it, held to
    /// the subset sim-core takes as a `<flex>`'s vertex bodies are. Rigid by
    /// MuJoCo's rule: every vertex on one body (`user_mesh.cc:4204-4213`).
    fn resolve_flexcomp_vertices(
        &self,
        flex: &MjcfFlex,
        holder: &str,
        bodies: &[usize],
    ) -> Result<NamedVertices> {
        // Gone under fusestatic where no vertex names it, as in MuJoCo, whose
        // flex names the holding body only for a point on it.
        let holder_id = self.body_name_to_id.get(holder).copied();
        let nvert = if flex.vertices.is_empty() {
            bodies.len()
        } else {
            flex.vertices.len()
        };
        let body_of = |k: usize| {
            if bodies.len() == 1 {
                bodies[0]
            } else {
                bodies[k]
            }
        };
        let mut on = Vec::with_capacity(nvert);
        for k in 0..nvert {
            let id = body_of(k);
            if Some(id) == holder_id {
                if self.moves_with(id, true) {
                    return Err(MjcfError::unsupported(format!(
                        "a pinned point of flexcomp '{}' on body '{holder}', which moves",
                        flex.name
                    )));
                }
                on.push(VertexOn::StandIn);
            } else {
                self.check_vertex_body(&flex.body[k], id)?;
                on.push(VertexOn::Body(id));
            }
        }
        Ok(NamedVertices {
            flex: MjcfFlex {
                vertices: self.flexcomp_world_vertices(flex, bodies, nvert),
                ..flex.clone()
            },
            on,
            rigid: mujoco_rigid(flex, bodies),
        })
    }

    /// A flexcomp flex sim-core does not take, built with every vertex fixed
    /// at its world position, to be refused after the keyframes.
    fn fixed_flexcomp_vertices(&self, flex: &MjcfFlex) -> NamedVertices {
        let bodies: Vec<usize> = flex
            .body
            .iter()
            .map(|name| self.body_name_to_id.get(name).copied().unwrap_or(0))
            .collect();
        let nvert = if flex.vertices.is_empty() {
            bodies.len()
        } else {
            flex.vertices.len()
        };
        NamedVertices {
            flex: MjcfFlex {
                vertices: self.flexcomp_world_vertices(flex, &bodies, nvert),
                ..flex.clone()
            },
            on: vec![VertexOn::StandIn; nvert],
            rigid: mujoco_rigid(flex, &bodies),
        }
    }

    /// Each vertex's world position: its body's pose applied to its offset.
    fn flexcomp_world_vertices(
        &self,
        flex: &MjcfFlex,
        bodies: &[usize],
        nvert: usize,
    ) -> Vec<Vector3<f64>> {
        (0..nvert)
            .map(|k| {
                let id = if bodies.len() == 1 {
                    bodies[0]
                } else {
                    bodies[k]
                };
                let offset = flex.vertices.get(k).copied().unwrap_or_else(Vector3::zeros);
                let (pos, quat) = self.body_world_pose(id);
                pos + quat * offset
            })
            .collect()
    }

    /// A vertex body held to the subset sim-core takes: jointless with no
    /// joint above it, or three slides along x, y and z, unrotated, with no
    /// joint or mocap body above it.
    fn check_vertex_body(&self, name: &str, id: usize) -> Result<()> {
        let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
        let (adr, num) = (self.body_jnt_adr[id], self.body_jnt_num[id]);
        let three_slides = num == 3
            && (0..3).all(|k| {
                self.jnt_type[adr + k] == MjJointType::Slide && self.jnt_axis[adr + k] == axes[k]
            });
        if num != 0 && !three_slides {
            return Err(MjcfError::unsupported(format!(
                "vertex body '{name}' has joints other than three slides along x, y and z"
            )));
        }
        if id == 0 {
            return Ok(());
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
        Ok(())
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

    /// The jointless, massless body of a body-level flexcomp's pinned point
    /// (`D-FLEX-PINNED-BODY`), on the world body at the point's world position
    /// `pos`, and its flex vertex's entries: no dofs, no sphere (MuJoCo makes
    /// no body for the point).
    fn create_flex_stand_in(&mut self, flex: &MjcfFlex, pos: &Vector3<f64>, flex_id: usize) {
        self.nflexvert += 1;
        let body_id = self.body_parent.len();
        // Push body arrays (mirrors process_body_with_world_frame pattern)
        self.body_parent.push(0);
        self.body_rootid.push(body_id);
        self.body_jnt_adr.push(self.jnt_type.len());
        self.body_dof_adr.push(self.nv);
        self.body_geom_adr.push(self.geom_type.len());
        // On the world body, the local offset is the world position.
        self.body_pos.push(*pos);
        self.body_quat.push(UnitQuaternion::identity());
        self.body_ipos.push(Vector3::zeros());
        self.body_iquat.push(UnitQuaternion::identity());
        self.body_mass.push(0.0);
        self.body_inertia.push(Vector3::zeros());
        self.body_mocapid.push(None);
        self.body_name.push(None);
        self.body_sleep_policy.push(None);
        // The defaults of a body written with none of these (`body.rs`).
        self.body_gravcomp.push(0.0);
        self.body_faults.push(super::body::BodyFaults::default());
        self.body_user_raw.push(Vec::new());
        self.body_world_pos.push(*pos);
        self.body_world_quat.push(UnitQuaternion::identity());
        self.body_jnt_num.push(0);
        self.body_dof_num.push(0);
        self.body_geom_num.push(0);

        self.flexvert_qposadr.push(usize::MAX); // sentinel: no qpos
        self.flexvert_dofadr.push(usize::MAX); // sentinel: no dof
        self.flexvert_mass.push(0.0);
        self.flexvert_invmass.push(0.0);
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
        // welded together, is rigid (MuJoCo's rule, `user_model.cc:3512-3520`).
        // No model sim-mjcf builds makes the first half decide an edge: a
        // flexcomp's vertex masses are finite and positive, so it is rigid
        // only when every vertex is pinned, and the pinned vertices' bodies
        // all weld to the world.
        let weld = |v: usize| self.weld_id(self.flexvert_bodyid[vert_start + v]);
        self.flexedge_rigid.push(rigid || weld(a) == weld(b));
        // §42B S1: no flap until `extract_flex_flaps` fills a dim-2 flex's.
        self.flexedge_flap.push([-1, -1]);
        // §42B S2+S3: Initialize flex_bending to zeros per edge.
        self.flex_bending.extend_from_slice(&[0.0; BENDING_COEFFS]);
    }

    /// The flaps, hinges and cotangent bending coefficients of a dim-2 flex
    /// that bends (`elastic2d` bend or both: MuJoCo stores `flex_edgeflap`
    /// then only, `user_model.cc:3504-3510`); any other flex has none.
    ///
    /// The flaps are MuJoCo's (`CreateFlapStencil`, `user_mesh.cc:3667-3717`):
    /// each triangle in order, its edges `(1,2) (2,0) (0,1)`; the first
    /// triangle holding an edge stores the vertex opposite it in slot 0 and
    /// leaves slot 1 at -1, and a later one stores its opposite vertex in slot
    /// 1, so an edge of three triangles would keep the first and the last
    /// ([`check_flex_elasticity`] refuses one). The
    /// hinges (this crate's Bridson bending) are the edges whose flap has both
    /// slots, in edge order; the coefficients are computed for those edges
    /// when the material has `young > 0` and `thickness > 0`.
    fn extract_flex_flaps(
        &mut self,
        flex: &MjcfFlex,
        vert_start: usize,
        flex_id: usize,
        edge_start: usize,
        edge_key_to_global: &std::collections::HashMap<(usize, usize), usize>,
    ) {
        if flex.dim != 2 || !flex.elastic2d.bends() {
            return;
        }
        for elem in &flex.elements {
            for &(i, j) in ELEMENT_EDGES[1] {
                let key = (elem[i].min(elem[j]), elem[i].max(elem[j]));
                let Some(&global_e) = edge_key_to_global.get(&key) else {
                    continue;
                };
                // MuJoCo's opposite vertex: the one after the edge's second.
                #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
                let opposite = (vert_start + elem[(j + 1) % 3]) as i32;
                let flap = &mut self.flexedge_flap[global_e];
                if flap[0] == -1 {
                    flap[0] = opposite;
                } else {
                    flap[1] = opposite;
                }
            }
        }
        for global_e in edge_start..self.nflexedge {
            let [va, vb] = self.flexedge_flap[global_e];
            if vb == -1 {
                continue; // a boundary edge
            }
            let [ve0, ve1] = self.flexedge_vert[global_e];
            // Both slots hold vertex indices here: a slot is -1 only while no
            // triangle has filled it, and slot 0 is filled before slot 1.
            #[allow(clippy::cast_sign_loss)]
            let (va, vb) = (va as usize, vb as usize);
            let rest_angle = compute_dihedral_angle(
                flex.vertices[ve0 - vert_start],
                flex.vertices[ve1 - vert_start],
                flex.vertices[va - vert_start],
                flex.vertices[vb - vert_start],
            );
            self.flexhinge_vert.push([ve0, ve1, va, vb]);
            self.flexhinge_angle0.push(rest_angle);
            self.flexhinge_flexid.push(flex_id);
            self.nflexhinge += 1;
        }

        // §42B S2+S3: Compute cotangent bending coefficients for a flex with
        // valid material (young > 0, thickness > 0).
        if flex.young > 0.0 && flex.thickness > 0.0 {
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

    /// Each flex vertex's mass and inverse mass (`flexvert_mass`,
    /// `flexvert_invmass`) as its body's mass stands once the bounds and
    /// `settotalmass` have changed it, after the flexes were built. A vertex
    /// with no degree of freedom keeps an inverse mass of 0, as it was made.
    pub(crate) fn sync_flex_vertex_masses(&mut self) {
        for v in 0..self.nflexvert {
            let mass = self.body_mass[self.flexvert_bodyid[v]];
            let moves = self.flexvert_dofadr[v] != usize::MAX;
            self.flexvert_mass[v] = mass;
            self.flexvert_invmass[v] = if moves && mass > 0.0 { 1.0 / mass } else { 0.0 };
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

/// A flex's condim outside 1 to 6, refused (registry `D-FLEX-CONDIM-RANGE`):
/// MuJoCo 3.5.0 checks a geom's and a pair's condim but not a flex's, loads
/// it and stops at the first contact that takes it ('Invalid condim value',
/// `mj_contactParam`, `engine_collision_driver.c:1394-1397`). The caller runs
/// it after the keyframes, as a refusal MuJoCo does not make.
pub fn check_flex_condims(flexes: &[MjcfFlex]) -> Result<()> {
    for (i, flex) in flexes.iter().enumerate() {
        if !(1..=6).contains(&flex.condim) {
            return Err(MjcfError::invalid_value(
                "flex",
                "condim",
                format!("{} is outside 1 to 6", flex.condim),
            )
            .at(Location::nth(ElementKind::Flex, Some(&flex.name), i)));
        }
    }
    Ok(())
}

/// MuJoCo's check of a dim-2 flex that bends, with a Young's modulus
/// (`mjCFlex::Compile`, `user_mesh.cc:4363-4367`): a thickness that is not
/// negative ("thickness must be positive for bending stiffness"; 0 loads,
/// with no bending).
pub fn check_flex_bending_thickness(flexes: &[MjcfFlex]) -> Result<()> {
    for (i, flex) in flexes.iter().enumerate() {
        if flex.dim == 2 && flex.elastic2d.bends() && flex.young > 0.0 && flex.thickness < 0.0 {
            return Err(MjcfError::invalid_value(
                "elasticity",
                "thickness",
                "thickness must be positive for bending stiffness",
            )
            .at(Location::nth(ElementKind::Flex, Some(&flex.name), i)));
        }
    }
    Ok(())
}

/// The flex elasticity this crate refuses where MuJoCo computes it or takes
/// it as given: a dim-2 flex's stretching where MuJoCo makes its membrane,
/// with a Young's modulus and a positive thickness (`user_mesh.cc:4351-4355`;
/// sim-core has none, `D-FLEX-MEMBRANE`); a dim-3 flex's Young's modulus,
/// MuJoCo's tetrahedral elasticity (`:4356-4360`; sim-core has none,
/// `D-FLEX-TET-ELASTICITY`); and an edge of three or more triangles in a flex
/// that bends, where MuJoCo pairs the first and the last (`CreateFlapStencil`,
/// `:3705`) and leaves out the others (`D-FLEX-BEND-NONMANIFOLD`).
pub fn check_flex_elasticity(flexes: &[MjcfFlex]) -> Result<()> {
    for (i, flex) in flexes.iter().enumerate() {
        let at = || Location::nth(ElementKind::Flex, Some(&flex.name), i);
        if flex.dim == 2 && flex.elastic2d.stretches() && flex.young > 0.0 && flex.thickness > 0.0 {
            return Err(MjcfError::unsupported(
                "a flex's stretching (elastic2d stretch or both, MuJoCo's membrane elasticity)",
            )
            .at(at()));
        }
        if flex.dim == 3 && flex.young > 0.0 {
            return Err(MjcfError::unsupported(
                "a dim-3 flex's Young's modulus (MuJoCo's tetrahedral elasticity)",
            )
            .at(at()));
        }
        if flex.dim == 2 && flex.elastic2d.bends() && has_an_edge_of_three_triangles(flex) {
            return Err(MjcfError::invalid_model(
                "an edge of three or more triangles in a flex that bends",
            )
            .at(at()));
        }
    }
    Ok(())
}

/// Whether three of a dim-2 flex's triangles hold one edge.
fn has_an_edge_of_three_triangles(flex: &MjcfFlex) -> bool {
    let mut held = std::collections::HashMap::new();
    for elem in &flex.elements {
        for &(i, j) in ELEMENT_EDGES[1] {
            let count = held
                .entry((elem[i].min(elem[j]), elem[i].max(elem[j])))
                .or_insert(0_u32);
            *count += 1;
            if *count == 3 {
                return true;
            }
        }
    }
    false
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

/// The bending stiffness the Bridson bending reads (`flex_bend_stiffness`)
/// of a dim-2 flex that bends: the Kirchhoff-Love thin-plate stiffness
/// `D = E t³ / (12 (1 − ν²))`. 0 for any other flex, or without a Young's
/// modulus.
fn compute_bend_stiffness_from_material(flex: &MjcfFlex) -> f64 {
    if flex.dim != 2 || !flex.elastic2d.bends() || flex.young <= 0.0 {
        return 0.0;
    }
    let nu = flex.poisson.clamp(0.0, 0.499);
    flex.young * flex.thickness.powi(3) / (12.0 * (1.0 - nu * nu))
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
