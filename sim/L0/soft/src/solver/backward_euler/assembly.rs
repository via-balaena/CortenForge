//! Force + Hessian assembly for
//! [`CpuNewtonSolver`](super::CpuNewtonSolver).

use std::collections::BTreeMap;

use faer::sparse::Triplet;
use sim_ml_chassis::Tensor;

use crate::contact::{ActivePairsFor, ContactModel};
use crate::element::Element;
use crate::material::Material;
use crate::mesh::{Mesh, TetId};
use crate::readout::LoadAxis;

use super::CpuNewtonSolver;
use super::helpers::{deformation_gradient, extract_element_dof_values, slice_to_vec3s};

impl<E, Msh, C, M, const N: usize, const G: usize> CpuNewtonSolver<E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    /// Read θ via the load map and emit the global external-force
    /// vector (Decision L). All-`AxisZ` BC (Stage 1) treats `theta` as
    /// a length-1 magnitude broadcast to every loaded vertex's `+ẑ`
    /// DOF; all-`FullVector` BC (Stage 2) consumes `theta` as
    /// `[3 * n_loaded]` per-vertex traction triples. Mixed-axis
    /// scenes are out of Phase 2 scope per `LoadAxis` doc.
    ///
    /// `config.gravity_z` adds a body-force `m_v · gravity_z` to every
    /// vertex's `+ẑ` DOF after the θ scatter. Default `gravity_z = 0`
    /// short-circuits the loop — bit-equal regression on the
    /// passthrough / compressive-block / Hertzian fixtures (which
    /// leave `gravity_z = 0`).
    //
    // Shape mismatches (BC contradicting θ length) panic — programmer
    // bug at scene wiring time, not runtime input.
    #[allow(clippy::panic)]
    pub(super) fn assemble_external_force(&self, theta: &Tensor<f64>, f_ext: &mut [f64]) {
        debug_assert!(
            f_ext.len() == self.n_dof,
            "f_ext output buffer length {} ≠ n_dof {}",
            f_ext.len(),
            self.n_dof,
        );
        f_ext.fill(0.0);
        let theta_slice = theta.as_slice();
        let loaded = &self.boundary_conditions.loaded_vertices;

        if loaded.is_empty() {
            assert!(
                theta_slice.is_empty(),
                "θ has length {} but BC has no loaded vertices",
                theta_slice.len(),
            );
        } else {
            let all_axis_z = loaded.iter().all(|(_, ax)| matches!(ax, LoadAxis::AxisZ));
            let all_full_vec = loaded
                .iter()
                .all(|(_, ax)| matches!(ax, LoadAxis::FullVector));
            assert!(
                all_axis_z || all_full_vec,
                "Mixed-axis loaded_vertices are out of Phase 2 scope; got {loaded:?}"
            );

            if all_axis_z {
                // Stage-1 broadcast: one θ scalar drives `+ẑ` on every loaded vertex.
                assert!(
                    theta_slice.len() == 1,
                    "AxisZ-loaded θ must have length 1 (broadcast magnitude), got {}",
                    theta_slice.len(),
                );
                let mag = theta_slice[0];
                for &(vertex_id, _) in loaded {
                    f_ext[3 * vertex_id as usize + 2] = mag;
                }
            } else {
                // Stage-2 per-vertex: θ supplies xyz for each loaded vertex in order.
                let expected = 3 * loaded.len();
                assert!(
                    theta_slice.len() == expected,
                    "FullVector-loaded θ must have length {expected} (3 × {}), got {}",
                    loaded.len(),
                    theta_slice.len(),
                );
                for (i, &(vertex_id, _)) in loaded.iter().enumerate() {
                    let v = vertex_id as usize;
                    f_ext[3 * v] = theta_slice[3 * i];
                    f_ext[3 * v + 1] = theta_slice[3 * i + 1];
                    f_ext[3 * v + 2] = theta_slice[3 * i + 2];
                }
            }
        }

        // Gravity body force `f_z += m_v · g_z` per vertex. The
        // exact-zero short-circuit preserves bit-equality on the
        // regression net (the passthrough / compressive-block /
        // Hertzian fixtures leave `gravity_z = 0`). For loaded AxisZ
        // vertices the gravity term adds onto the θ-traction; orphan
        // vertices are auto-pinned at `new()`'s `effective_pinned`
        // step so their zero `mass_per_dof` never reaches a free DOF.
        if self.config.gravity_z != 0.0 {
            let n_vertices = self.n_dof / 3;
            for v in 0..n_vertices {
                f_ext[3 * v + 2] += self.mass_per_dof[3 * v + 2] * self.config.gravity_z;
            }
        }
    }

    /// Walk every element, scatter per-vertex internal-force
    /// contributions into the global `f_int` buffer.
    ///
    /// `f_int` is zeroed inside; caller need not pre-clear. Reads
    /// `x_curr` (length `n_dof`), cached `element_geometries`, the
    /// per-tet `M` from `self.mesh.materials()` (Phase 4 commit 5 —
    /// Newton hot path reads from the per-tet material cache per
    /// Part 7 §02 §00), and `self.contact` for active-pair gradient
    /// contributions (Phase 5 commit 5; scope memo Decision H —
    /// per-iter active-pair recompute).
    //
    // Lint allows: see assemble_free_hessian_triplets justification.
    #[allow(clippy::cast_possible_truncation, clippy::needless_range_loop)]
    pub(super) fn assemble_global_int_force(
        &self,
        x_curr: &[f64],
        x_prev: &[f64],
        dt: f64,
        f_int: &mut [f64],
    ) {
        debug_assert!(x_curr.len() == self.n_dof);
        debug_assert!(f_int.len() == self.n_dof);
        f_int.fill(0.0);
        let materials = self.mesh.materials();
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let piola = materials[tet_id].first_piola(&f);
            // Per-vertex internal-force contribution `V · P · grad_X N_a`.
            for a in 0..4 {
                let v = verts[a] as usize;
                for i in 0..3 {
                    let mut sum = 0.0;
                    for j in 0..3 {
                        sum += piola[(i, j)] * geom.grad_x_n[(a, j)];
                    }
                    f_int[3 * v + i] += geom.volume * sum;
                }
            }
        }

        // Contact gradient contributions. Sign convention per scope
        // memo §6 R-5 lens (v): the gradient `∂E_pen/∂x` is scattered
        // as `+force` into f_int, mirroring the elastic case where
        // `∂Ψ_elastic/∂x` is the +`f_int` contribution above.
        // `NullContact` returns an empty contributions Vec → empty
        // for-loops → bit-equal to the pre-Phase-5 elastic-only path.
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        for pair in &pairs {
            let g = self.contact.gradient(pair, &positions);
            for &(vid, force) in &g.contributions {
                let v = vid as usize;
                f_int[3 * v] += force.x;
                f_int[3 * v + 1] += force.y;
                f_int[3 * v + 2] += force.z;
            }
        }

        // Smoothed-Coulomb friction gradient `∇D` (the dissipative tangential force),
        // scattered like the contact force. Empty when frictionless ⇒ bit-equal.
        for (v, grad, _) in self.friction_blocks(x_curr, x_prev, dt) {
            f_int[3 * v] += grad.x;
            f_int[3 * v + 1] += grad.y;
            f_int[3 * v + 2] += grad.z;
        }
    }

    /// Per-active-pair smoothed-Coulomb friction `(vertex, ∇D, ∇²D)` at the current
    /// configuration, with the lagged `(normal, λⁿ)` read from the contact's own gradient
    /// (the contact-energy gradient `force = ∇E_contact` has magnitude `λⁿ` and lies along
    /// `±n̂`; the tangent plane is sign-agnostic). `xᵗ = x_prev` (the step start), `w =
    /// dt·ε_v`. Empty when `friction_mu == 0` (the frictionless short-circuit) or no pairs.
    /// Shared by the forward residual and Hessian assemblies so the two stay in lockstep.
    pub(super) fn friction_blocks(
        &self,
        x_curr: &[f64],
        x_prev: &[f64],
        dt: f64,
    ) -> Vec<(usize, crate::Vec3, nalgebra::Matrix3<f64>)> {
        let mu = self.config.friction_mu;
        if mu == 0.0 {
            return Vec::new();
        }
        let w = dt * self.config.friction_eps_v;
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        let mut out = Vec::new();
        for pair in &pairs {
            for (vid, force) in self.contact.gradient(pair, &positions).contributions {
                let lambda = force.norm(); // |∇E_contact| = the normal-force magnitude λⁿ
                if lambda == 0.0 {
                    continue;
                }
                let v = vid as usize;
                let x_v = positions[v];
                // Effective step-start `xᵗ_eff = xᵗ + Δ_surf`: shifting the friction
                // reference by the rigid surface's within-step tangential drift makes
                // `u_T = Tⁿᵀ(x_v − xᵗ − Δ_surf)` the slip RELATIVE to the moving
                // collider (a stationary vertex under a sliding collider gets dragged).
                // Δ_surf = 0 (the default) recovers PR1's static-collider friction.
                let x_start = crate::Vec3::new(x_prev[3 * v], x_prev[3 * v + 1], x_prev[3 * v + 2])
                    + self.friction_surface_drift;
                let (grad, hess) =
                    crate::contact::friction::grad_hess(x_v, x_start, force, lambda, mu, w);
                out.push((v, grad, hess));
            }
        }
        out
    }

    /// Nodal **reaction forces** the soft body exerts on its Dirichlet
    /// constraints at the configuration `x_curr` — the public readout the
    /// soft↔rigid *bond* consumes (rung 6).
    ///
    /// ⚠ **Quasi-static regime only.** This returns `−f_int`, which equals the
    /// constraint reaction ONLY when the inertial term is negligible (a large-`dt`
    /// static solve) and the constrained nodes carry no external load. It takes no
    /// velocity and does not subtract `M/dt²·(x − x_prev − dt·v)`, so calling it at a
    /// physical `dt` where inertia matters returns a **silently wrong** reaction. A
    /// dynamic consumer must form the full residual instead (`dt`/`x_prev` are taken
    /// here only to feed the friction term of `assemble_global_int_force`).
    ///
    /// Returns a length-`n_dof` vector holding `−f_int` per DOF, where
    /// `f_int` is the assembled internal force (`assemble_global_int_force`).
    /// For a **converged static** solve (`dt` large so the inertial term
    /// `M/dt²` is negligible, no external load on the constrained nodes) the
    /// force a Dirichlet-pinned node's constraint must supply to hold it is the
    /// residual `r = f_int` there, so by Newton's third law the force the soft
    /// body exerts *on that constraint* (e.g. a bonded rigid endplate) is
    /// `−f_int`. Summing the returned entries over a bonded vertex set gives the
    /// net force that face applies to its rigid attachment; pairing each with its
    /// moment arm about the body centre gives the wrench.
    ///
    /// The internal force includes any active contact / friction contributions
    /// (empty for the `NullContact`, frictionless bond), so this is the total
    /// nodal reaction, not the elastic part alone. It does **not** subtract
    /// inertia or external load — a static, load-free constrained solve (the bond
    /// regime) is where `−f_int` *is* the reaction; a dynamic or externally-loaded
    /// constrained node would need the full residual instead.
    #[must_use]
    pub fn nodal_reaction_forces(&self, x_curr: &[f64], x_prev: &[f64], dt: f64) -> Vec<f64> {
        let mut f_int = vec![0.0; self.n_dof];
        self.assemble_global_int_force(x_curr, x_prev, dt, &mut f_int);
        for f in &mut f_int {
            *f = -*f;
        }
        f_int
    }

    /// Assemble the lower-triangle triplets of the free-DOF Hessian
    /// `A_free = M_free / Δt² + K_free(x_curr) + K_contact(x_curr)`
    /// per Decision J + Phase 5 commit 5.
    ///
    /// For each element + each (a, b) vertex pair, contributes the
    /// 3×3 block from `B_a^T 𝕔 B_b` (using BF-5's flattening
    /// convention) into the global free-DOF sparse matrix at
    /// `(free_idx_a, free_idx_b)` whenever both DOFs are free (looked
    /// up via `full_to_free_idx`). Contact Hessian contributions from
    /// `self.contact.hessian` (rank-1 symmetric `κ·n⊗n` blocks for
    /// `PenaltyRigidContact`; symmetry assumption matches the existing
    /// lower-triangle filter) scatter through the same gate between
    /// the elastic and mass-diagonal passes. Diagonal mass added at
    /// the end. `BTreeMap` accumulates with deterministic (col, row)
    /// iteration order per Decision M.
    //
    // Lint allows: `as TetId` is the Mesh-trait-API tax (n_tets
    // returns usize, tet_vertices takes u32) — same as commit 2's
    // HandBuiltTetMesh. `for a/b in 0..4` iterates over Tet4's 4 nodes
    // by index (used for both `verts[a]` AND `geom.grad_x_n[(a, l)]`
    // — the lint flags the verts use only). `va`/`vb` similar-name
    // pair mirrors the (a, b) symmetry of the per-element block math.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::needless_range_loop,
        clippy::similar_names
    )]
    pub(super) fn assemble_free_hessian_triplets(
        &self,
        x_curr: &[f64],
        x_prev: Option<&[f64]>,
        dt: f64,
    ) -> Vec<Triplet<usize, usize, f64>> {
        debug_assert!(x_curr.len() == self.n_dof);
        // `dt * dt` then `mass / dt2` matches the pre-commit-4 code's
        // mass-diagonal expression order (`mass_per_dof / (dt * dt)`).
        // Distinct from `mass * (1.0 / (dt * dt))` at the last bit — the
        // FP-equal form preserves bit-equality with the pre-Phase-2
        // 1-tet path, simplifying any future bisect.
        let dt2 = dt * dt;
        // (col, row) → accumulated value. BTreeMap for sorted iteration
        // (Decision M D-3); no HashMap on numeric paths.
        let mut acc: BTreeMap<(usize, usize), f64> = BTreeMap::new();

        let materials = self.mesh.materials();
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let tangent_9x9 = materials[tet_id].tangent(&f);

            for a in 0..4 {
                let va = verts[a] as usize;
                for b in 0..4 {
                    let vb = verts[b] as usize;
                    for i in 0..3 {
                        for j in 0..3 {
                            let row_full = 3 * va + i;
                            let col_full = 3 * vb + j;
                            if let (Some(row_free), Some(col_free)) = (
                                self.full_to_free_idx[row_full],
                                self.full_to_free_idx[col_full],
                            ) && row_free >= col_free
                            {
                                // (B_a^T 𝕔 B_b)[i,j] = Σ_{l,l'} (grad_X N_a)_l ·
                                //   𝕔[(i+3l), (j+3l')] · (grad_X N_b)_{l'}
                                let mut block = 0.0;
                                for l in 0..3 {
                                    for lp in 0..3 {
                                        block += geom.grad_x_n[(a, l)]
                                            * tangent_9x9[(i + 3 * l, j + 3 * lp)]
                                            * geom.grad_x_n[(b, lp)];
                                    }
                                }
                                *acc.entry((col_free, row_free)).or_insert(0.0) +=
                                    geom.volume * block;
                            }
                        }
                    }
                }
            }
        }

        // Contact Hessian contributions. Scatter through the same
        // free-DOF + lower-triangle filter as the elastic block above
        // (scope memo §6 R-5 lens (v) — sign-consistent with the
        // f_int gradient scatter in `assemble_global_int_force`).
        // `NullContact` returns an empty contributions Vec → empty
        // for-loops → acc unchanged → bit-equal Hessian.
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        for pair in &pairs {
            let h = self.contact.hessian(pair, &positions);
            for &(row_vid, col_vid, block) in &h.contributions {
                for i in 0..3 {
                    for j in 0..3 {
                        let row_full = 3 * (row_vid as usize) + i;
                        let col_full = 3 * (col_vid as usize) + j;
                        if let (Some(row_free), Some(col_free)) = (
                            self.full_to_free_idx[row_full],
                            self.full_to_free_idx[col_full],
                        ) && row_free >= col_free
                        {
                            *acc.entry((col_free, row_free)).or_insert(0.0) += block[(i, j)];
                        }
                    }
                }
            }
        }

        // Smoothed-Coulomb friction Hessian `∇²D` (PSD per-pair 3×3 block at the contacted
        // vertex), scattered through the same free-DOF + lower-triangle filter. Threaded
        // ONLY when `x_prev` is supplied — the FORWARD Newton solve passes `Some(x_prev)`
        // (so the solve converges to the friction equilibrium); the differentiable tangent
        // (`factor_at_position`) passes `None` (friction enters the adjoint in the
        // differentiability leaf). Empty / `None` ⇒ acc unchanged ⇒ bit-equal.
        if let Some(x_prev) = x_prev {
            for (v, _, block) in self.friction_blocks(x_curr, x_prev, dt) {
                for i in 0..3 {
                    for j in 0..3 {
                        let (row_full, col_full) = (3 * v + i, 3 * v + j);
                        if let (Some(rf), Some(cf)) = (
                            self.full_to_free_idx[row_full],
                            self.full_to_free_idx[col_full],
                        ) && rf >= cf
                        {
                            *acc.entry((cf, rf)).or_insert(0.0) += block[(i, j)];
                        }
                    }
                }
            }
        }

        // Mass diagonal: M_free / Δt² · I on (k, k).
        for k in 0..self.n_free {
            let mass_dof = self.mass_per_dof[self.free_dof_indices[k]];
            *acc.entry((k, k)).or_insert(0.0) += mass_dof / dt2;
        }

        acc.into_iter()
            .map(|((c, r), v)| Triplet::new(r, c, v))
            .collect()
    }
}
