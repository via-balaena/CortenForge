//! Nodal-averaged F-bar volumetric-locking cure (assembly-layer transform).
//!
//! Per [`Part 2 Ch 05 02-f-bar.md`][fbar]: the near-incompressible
//! (`ν → 0.5`) Tet4 solver locks because the single-Gauss-point
//! per-element volumetric constraint `J ≈ 1` over-constrains the 12-DOF
//! element. F-bar relaxes this to a *patch-averaged* `J̄ ≈ 1` constraint by
//! feeding the constitutive law a modified deformation gradient
//!
//! ```text
//!     F* = (J̄ / J)^{1/3} · F ,     J̄ = patch-averaged volume ratio
//! ```
//!
//! whose isochoric part `F̄ = J^{-1/3} F` is unchanged (`det F* = J̄`, so
//! `F̄(F*) = F̄(F)`) — only the volumetric scalar the law sees is coarsened.
//! This module is the **assembly-layer transform** that owns `J̄`: the
//! `Material` trait keeps its clean per-element `&F` surface and never sees a
//! neighbor (a `Material` structurally cannot — `J̄` is a mesh-topology
//! quantity), so the modified kinematic is built *here* and passed into the
//! unchanged `first_piola` call.
//!
//! ## Nodal-averaged patch (the "F-bar-patch" of de Souza Neto)
//!
//! The patch is the *nodal* volume average. With per-element rest volume `V_e`
//! and current volume ratio `J_e = det F_e`:
//!
//! ```text
//!     W_v   = Σ_{e ∋ v} V_e                     (nodal rest volume)
//!     Ĵ_v   = (1/W_v) Σ_{e ∋ v} V_e J_e         (volume-weighted nodal J)
//!     J̄_e   = (1/4) Σ_{a ∈ e} Ĵ_{v_a}           (element patch J, 4 nodes)
//! ```
//!
//! `J̄_e` depends on every element sharing a node with `e` — the neighbor
//! coupling the internal force (and, in the differentiable leaf, the tangent)
//! carries.
//!
//! ## Consistent internal force
//!
//! Energy `E = Σ_e V_e W(F*_e)`. With `θ_e = (J̄_e/J_e)^{1/3}` (so
//! `F*_e = θ_e F_e`), the internal force is the exact gradient `f = ∂E/∂x`.
//! Using the local log-volume gradient `g_e(a) = F_e^{-T} ∇N_a`
//! (`∂ln J_e/∂x_a = g_e(a)`, nonzero only for `a ∈ e`) and the scalar
//! volumetric power `π_e = P*_e : F_e`, it splits into a **local** part (on
//! `e`'s own nodes) and a **patch** part (across the node star):
//!
//! ```text
//!     f_local[a∈e]  += θ_e V_e ( P*_e ∇N_a − (π_e/3) g_e(a) )
//!     f_patch       += Σ_v c_v · ∂Ĵ_v/∂x ,   c_v = Σ_{e∋v} V_e θ_e π_e / (12 J̄_e)
//!     ∂Ĵ_v/∂x       = (1/W_v) Σ_{e'∋v} V_{e'} J_{e'} g_{e'}(a')  → node a' of e'
//! ```
//!
//! The `f_patch` term is accumulated in reverse: a per-node adjoint `c_v`
//! (the `+∂ln J̄` sensitivity, `1/12 = (1/3)(1/4)`), then scattered back
//! through node incidence. At rest (`F = I`) every `P*_e = 0`, so all forces
//! vanish.
//!
//! ## Accuracy: a QUALITATIVE cure, not a quantitative one (measured)
//!
//! This F-bar **removes the locking pathology** — it lets `ν → 0.49` (Ecoflex
//! 00-30) *converge* where plain Tet4 locks/stalls, and it recovers the right
//! deformation *shape* (the ν-collapse of tip deflection is gone; ~4.2×
//! recovery on the cantilever gate). That is real, useful stability.
//!
//! It is **not quantitatively accurate.** Measured against the analytic Lamé
//! thick-shell oracle (`tests/concentric_lame_shells.rs` machinery), the
//! cavity-wall displacement **over-softens** — mesh-converged ~5 % at ν=0.4,
//! growing to ~21 % at ν=0.49 (vs plain Tet4, which converges to the analytic
//! at ν=0.4 and *under*-predicts ~23 % — locked — at ν=0.49). The cause is the
//! nodal-star patch average *over-relaxing* the volumetric constraint: it is a
//! property of this patch **scheme**, not the material (a dev/vol-split
//! Neo-Hookean was spiked and gives identical results at this strain) and not a
//! bug (the residual/tangent are FD-proven consistent with the `J̄` energy).
//! Per the spec, this is F-bar's designated role — the "fast *approximate*"
//! cure; mixed-u-p / higher-order (Tet10) are the *quantitatively accurate*
//! near-incompressible paths. Use F-bar for ν=0.49 *stability* and qualitative
//! / relative work; do not read "cures locking" as "accurate at ν=0.49."
//! Roadmap to the accurate element: `docs/SIM_SOFT_TET10_PLAN.md`.
//!
//! [fbar]: ../../../../../../docs/studies/soft_body_architecture/src/20-materials/05-incompressibility/02-f-bar.md

// Per-element assembly idioms, shared verbatim with `assembly.rs` / `construct.rs`
// (which carry the same justified allows): the `0..4` node loops index several
// parallel arrays at once (`verts` / `g` / `grad_x_n` / `out`), so an index is
// clearer than `enumerate`; `as TetId` is the `Mesh`-trait API tax (`n_tets`
// returns `usize`, `tet_vertices` takes `u32`, meshes stay far below `u32::MAX`);
// and `va`/`vb`, `e`/`e2` mirror the (node, node) / (element, element) symmetry of
// the block math.
#![allow(
    clippy::needless_range_loop,
    clippy::cast_possible_truncation,
    clippy::similar_names
)]

use nalgebra::{Matrix3, SMatrix, Vector3};

use crate::material::Material;
use crate::mesh::{Mesh, TetId};

use super::ElementGeometry;
use super::helpers::{deformation_gradient, extract_element_dof_values};

/// Precomputed nodal-patch topology for the nodal-averaged F-bar transform.
///
/// Built once at solver construction (F-bar mode only) from rest-mesh
/// element-vertex incidence and per-element rest volumes. Holds the nodal rest
/// volumes `W_v` and, per vertex, the `(tet, local-node-index)` pairs that
/// reference it — the incidence the `J̄` average and its scatter walk.
#[derive(Clone, Debug)]
pub(super) struct FbarCache {
    /// Nodal rest volume `W_v = Σ_{e ∋ v} V_e`, indexed by vertex. Vertices
    /// referenced by no tet carry `0.0` (they never enter a patch).
    nodal_volume: Vec<f64>,
    /// Per-vertex incidence: `incident[v]` lists `(tet_id, a)` where tet
    /// `tet_id`'s local node `a ∈ 0..4` equals `v`.
    incident: Vec<Vec<(TetId, u8)>>,
}

/// Per-element forward F-bar scalars, cached across the internal-force and
/// (differentiable-leaf) tangent passes.
struct ElementFbar {
    /// Deformation gradient `F_e`.
    f: Matrix3<f64>,
    /// `F_e^{-T}` (the log-volume gradient operator's left factor).
    f_inv_t: Matrix3<f64>,
    /// Per-element `J_e = det F_e`.
    j: f64,
    /// Patch ratio `J̄_e`.
    j_bar: f64,
    /// Volumetric scale `θ_e = (J̄_e/J_e)^{1/3}`.
    theta: f64,
}

impl FbarCache {
    /// Build the nodal-patch cache from mesh topology and cached per-element
    /// rest volumes (`element_geometries[e].volume`, in `tet_id` order).
    /// Single-threaded incidence walk for determinism.
    pub(super) fn build<M, Msh>(mesh: &Msh, geometries: &[ElementGeometry]) -> Self
    where
        M: Material,
        Msh: Mesh<M>,
    {
        let n_vertices = mesh.n_vertices();
        let n_tets = mesh.n_tets();
        debug_assert_eq!(geometries.len(), n_tets);
        let mut nodal_volume = vec![0.0; n_vertices];
        let mut incident: Vec<Vec<(TetId, u8)>> = vec![Vec::new(); n_vertices];
        for tet_id in 0..n_tets as TetId {
            let verts = mesh.tet_vertices(tet_id);
            let v_e = geometries[tet_id as usize].volume;
            for (a, &v) in verts.iter().enumerate() {
                nodal_volume[v as usize] += v_e;
                incident[v as usize].push((tet_id, a as u8));
            }
        }
        Self {
            nodal_volume,
            incident,
        }
    }

    /// Per-element patch ratio `J̄_e`, indexed by `tet_id`. `j_elem[e] = det F_e`.
    fn element_j_bar<M, Msh>(
        &self,
        mesh: &Msh,
        j_elem: &[f64],
        geometries: &[ElementGeometry],
    ) -> Vec<f64>
    where
        M: Material,
        Msh: Mesh<M>,
    {
        // Ĵ_v (nodal), then average over each element's 4 nodes.
        let mut nodal = vec![1.0; self.nodal_volume.len()];
        for (v, incidence) in self.incident.iter().enumerate() {
            let w_v = self.nodal_volume[v];
            if w_v == 0.0 {
                continue;
            }
            let mut acc = 0.0;
            for &(e, _) in incidence {
                acc += geometries[e as usize].volume * j_elem[e as usize];
            }
            nodal[v] = acc / w_v;
        }
        (0..mesh.n_tets() as TetId)
            .map(|tet_id| {
                let verts = mesh.tet_vertices(tet_id);
                0.25 * verts.iter().map(|&v| nodal[v as usize]).sum::<f64>()
            })
            .collect()
    }

    /// Forward F-bar scalars per element at configuration `x_curr`.
    fn forward<M, Msh>(
        &self,
        mesh: &Msh,
        x_curr: &[f64],
        geometries: &[ElementGeometry],
    ) -> Vec<ElementFbar>
    where
        M: Material,
        Msh: Mesh<M>,
    {
        // Pass 1: per-element F_e and J_e.
        let n_tets = mesh.n_tets();
        let mut f_e = Vec::with_capacity(n_tets);
        let mut j_elem = Vec::with_capacity(n_tets);
        for (tet_id, geom) in geometries.iter().enumerate() {
            let verts = mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            j_elem.push(f.determinant());
            f_e.push(f);
        }
        // Pass 2: patch ratios, then assemble the per-element scalar bundle.
        let j_bar = self.element_j_bar(mesh, &j_elem, geometries);
        f_e.into_iter()
            .zip(j_elem)
            .zip(j_bar)
            .map(|((f, j), j_bar)| ElementFbar {
                f_inv_t: invert_transpose(&f),
                f,
                j,
                j_bar,
                theta: (j_bar / j).cbrt(),
            })
            .collect()
    }

    /// Scatter the consistent F-bar elastic internal force into `f_int`
    /// (length `n_dof`, caller-zeroed). Replaces the plain per-element elastic
    /// scatter of [`assemble_global_int_force`](super::CpuNewtonSolver::assemble_global_int_force);
    /// contact / friction contributions are added by the caller unchanged.
    pub(super) fn scatter_internal_force<M, Msh>(
        &self,
        mesh: &Msh,
        materials: &[M],
        x_curr: &[f64],
        geometries: &[ElementGeometry],
        f_int: &mut [f64],
    ) where
        M: Material,
        Msh: Mesh<M>,
    {
        let elems = self.forward(mesh, x_curr, geometries);
        // Per-node adjoint c_v for the +∂ln J̄ patch path (1/12 = (1/3)(1/4)).
        let mut c_v = vec![0.0; self.nodal_volume.len()];

        for (tet_id, ef) in elems.iter().enumerate() {
            let geom = &geometries[tet_id];
            let verts = mesh.tet_vertices(tet_id as TetId);
            let p_star = materials[tet_id].first_piola(&(ef.theta * ef.f));
            let pi = frobenius(&p_star, &ef.f); // π_e = P*_e : F_e
            let g = local_log_volume_grad(&ef.f_inv_t, &geom.grad_x_n);

            // Local force on e's own nodes: θ_e V_e ( P* ∇N_a − (π/3) g_a ).
            let scale = ef.theta * geom.volume;
            for a in 0..4 {
                let grad_n = grad_row(&geom.grad_x_n, a);
                let force = scale * (p_star * grad_n - (pi / 3.0) * g[a]);
                let v = verts[a] as usize;
                f_int[3 * v] += force.x;
                f_int[3 * v + 1] += force.y;
                f_int[3 * v + 2] += force.z;
            }

            // Nodal adjoint deposit for the patch path.
            let deposit = geom.volume * ef.theta * pi / (12.0 * ef.j_bar);
            for &v in &verts {
                c_v[v as usize] += deposit;
            }
        }

        // Patch scatter: f += Σ_v c_v ∂Ĵ_v/∂x, with
        // ∂Ĵ_v/∂x = (1/W_v) Σ_{e'∋v} V_{e'} J_{e'} g_{e'}(a') → node a' of e'.
        for (v, incidence) in self.incident.iter().enumerate() {
            let w_v = self.nodal_volume[v];
            if c_v[v] == 0.0 || w_v == 0.0 {
                continue;
            }
            let cw = c_v[v] / w_v;
            for &(e, _) in incidence {
                let ef = &elems[e as usize];
                let geom = &geometries[e as usize];
                let verts = mesh.tet_vertices(e);
                let g = local_log_volume_grad(&ef.f_inv_t, &geom.grad_x_n);
                let coeff = cw * geom.volume * ef.j;
                for a in 0..4 {
                    let force = coeff * g[a];
                    let vp = verts[a] as usize;
                    f_int[3 * vp] += force.x;
                    f_int[3 * vp + 1] += force.y;
                    f_int[3 * vp + 2] += force.z;
                }
            }
        }
    }

    /// Total F-bar elastic energy `Σ_e V_e W(F*_e)` — the FD reference the
    /// internal-force consistency test differences.
    pub(super) fn energy<M, Msh>(
        &self,
        mesh: &Msh,
        materials: &[M],
        x_curr: &[f64],
        geometries: &[ElementGeometry],
    ) -> f64
    where
        M: Material,
        Msh: Mesh<M>,
    {
        let elems = self.forward(mesh, x_curr, geometries);
        elems
            .iter()
            .enumerate()
            .map(|(tet_id, ef)| {
                geometries[tet_id].volume * materials[tet_id].energy(&(ef.theta * ef.f))
            })
            .sum()
    }

    /// Consistent F-bar tangent action `K·δ = ∂f/∂x · δ` (the directional
    /// derivative of [`Self::scatter_internal_force`]'s output in direction
    /// `delta`), returned as a length-`n_dof` vector.
    ///
    /// Derived by differentiating the residual assembly *once* via the product
    /// rule: each per-element quantity `q(x)` carries a directional derivative
    /// `q̇ = ∂q/∂x · δ`, and the residual's own reverse-accumulation structure
    /// propagates the `J̄`-neighbor coupling into `K·δ` automatically — no
    /// hand-derived second-derivative tensors. This is the exact Hessian action
    /// of the F-bar elastic energy (verified against the residual by finite
    /// difference); the triplet assembly the Newton factor consumes is the same
    /// bilinear form, blocked by node pair.
    //
    // Long by design: the forward + dotted per-element bundle, the nodal `J̇`
    // reduction, and the local + patch derivative scatter are one coherent
    // product-rule sweep — splitting it would scatter the shared dotted state.
    #[allow(clippy::too_many_lines)]
    pub(super) fn tangent_matvec<M, Msh>(
        &self,
        mesh: &Msh,
        materials: &[M],
        x_curr: &[f64],
        geometries: &[ElementGeometry],
        delta: &[f64],
    ) -> Vec<f64>
    where
        M: Material,
        Msh: Mesh<M>,
    {
        let n_tets = mesh.n_tets();
        let elems = self.forward(mesh, x_curr, geometries);

        // Per-element forward + dotted (directional-derivative) bundle.
        let mut p_star = Vec::with_capacity(n_tets); // P*_e
        let mut c_star = Vec::with_capacity(n_tets); // C*_e = ∂P/∂F(F*_e)
        let mut g_rows = Vec::with_capacity(n_tets); // g_e(a) = F_e^{-T} ∇N_a
        let mut pi_e = Vec::with_capacity(n_tets); // π_e = P*_e : F_e
        let mut fdot = Vec::with_capacity(n_tets); // Ḟ_e
        let mut dln_j = Vec::with_capacity(n_tets); // δln J_e
        let mut jdot = Vec::with_capacity(n_tets); // J̇_e
        let mut gdot_rows = Vec::with_capacity(n_tets); // ġ_e(a)

        for (tet_id, ef) in elems.iter().enumerate() {
            let geom = &geometries[tet_id];
            let verts = mesh.tet_vertices(tet_id as TetId);
            let fs = ef.theta * ef.f;
            let p = materials[tet_id].first_piola(&fs);
            let c = materials[tet_id].tangent(&fs);
            let g = local_log_volume_grad(&ef.f_inv_t, &geom.grad_x_n);

            // Ḟ_e = Σ_{b∈e} δx_b ⊗ ∇N_b, and δln J_e = Σ_b g(b)·δx_b.
            let mut f_dot = Matrix3::zeros();
            let mut dlnj = 0.0;
            for a in 0..4 {
                let dx = node_delta(delta, verts[a]);
                f_dot += dx * grad_row(&geom.grad_x_n, a).transpose();
                dlnj += g[a].dot(&dx);
            }
            // ġ_e(a) = −F^{-T} Ḟ_eᵀ g(a).
            let mut g_dot = [Vector3::zeros(); 4];
            for a in 0..4 {
                g_dot[a] = -(ef.f_inv_t * f_dot.transpose() * g[a]);
            }

            pi_e.push(frobenius(&p, &ef.f));
            p_star.push(p);
            c_star.push(c);
            g_rows.push(g);
            jdot.push(ef.j * dlnj);
            dln_j.push(dlnj);
            fdot.push(f_dot);
            gdot_rows.push(g_dot);
        }

        // Nodal Ĵ̇_v = (1/W_v) Σ_{e∋v} V_e J̇_e.
        let mut nodal_jdot = vec![0.0; self.nodal_volume.len()];
        for (v, incidence) in self.incident.iter().enumerate() {
            let w_v = self.nodal_volume[v];
            if w_v == 0.0 {
                continue;
            }
            let mut acc = 0.0;
            for &(e, _) in incidence {
                acc += geometries[e as usize].volume * jdot[e as usize];
            }
            nodal_jdot[v] = acc / w_v;
        }

        let mut out = vec![0.0; delta.len()];
        // Per-node adjoints for the patch path: c_v and its derivative ċ_v.
        let mut c_v = vec![0.0; self.nodal_volume.len()];
        let mut cdot_v = vec![0.0; self.nodal_volume.len()];

        for (tet_id, ef) in elems.iter().enumerate() {
            let geom = &geometries[tet_id];
            let verts = mesh.tet_vertices(tet_id as TetId);
            let vol = geom.volume;
            let g = &g_rows[tet_id];
            let p = &p_star[tet_id];
            let pi = pi_e[tet_id];

            // J̄̇_e, θ̇_e, Ḟ*_e, Ṗ*_e, π̇_e.
            let jbar_dot = 0.25 * verts.iter().map(|&v| nodal_jdot[v as usize]).sum::<f64>();
            let theta_dot = (ef.theta / 3.0) * (jbar_dot / ef.j_bar - dln_j[tet_id]);
            let fs_dot = theta_dot * ef.f + ef.theta * fdot[tet_id];
            let p_dot = contract_tangent(&c_star[tet_id], &fs_dot);
            let pi_dot = frobenius(&p_dot, &ef.f) + frobenius(p, &fdot[tet_id]);

            // Local force derivative L̇_e[b] on e's own nodes.
            for a in 0..4 {
                let grad_n = grad_row(&geom.grad_x_n, a);
                let term = theta_dot * (p * grad_n - (pi / 3.0) * g[a])
                    + ef.theta
                        * (p_dot * grad_n
                            - (pi_dot / 3.0) * g[a]
                            - (pi / 3.0) * gdot_rows[tet_id][a]);
                add_node(&mut out, verts[a], &(vol * term));
            }

            // Patch adjoint deposits: c_v = Σ V θ π/(12 J̄) and its derivative.
            let deposit = vol * ef.theta * pi / (12.0 * ef.j_bar);
            let deposit_dot = vol / 12.0
                * (theta_dot * pi / ef.j_bar + ef.theta * pi_dot / ef.j_bar
                    - ef.theta * pi * jbar_dot / (ef.j_bar * ef.j_bar));
            for &v in &verts {
                c_v[v as usize] += deposit;
                cdot_v[v as usize] += deposit_dot;
            }
        }

        // Patch scatter derivative:
        // ḟ_patch[b] = Σ_v (ċ_v/W_v) Σ_{e∋v} V_e J_e g_e(b)
        //            + Σ_v (c_v/W_v) Σ_{e∋v} V_e (J̇_e g_e(b) + J_e ġ_e(b)).
        for (v, incidence) in self.incident.iter().enumerate() {
            let w_v = self.nodal_volume[v];
            if w_v == 0.0 {
                continue;
            }
            let cw = c_v[v] / w_v;
            let cwdot = cdot_v[v] / w_v;
            for &(e, _) in incidence {
                let ef = &elems[e as usize];
                let verts = mesh.tet_vertices(e);
                let vol = geometries[e as usize].volume;
                let g = &g_rows[e as usize];
                let gd = &gdot_rows[e as usize];
                for a in 0..4 {
                    let force = cwdot * vol * ef.j * g[a]
                        + cw * vol * (jdot[e as usize] * g[a] + ef.j * gd[a]);
                    add_node(&mut out, verts[a], &force);
                }
            }
        }

        out
    }
}

/// Per-element static (configuration-fixed) F-bar quantities, precomputed once
/// per tangent assembly and shared across every seed of the block kernel.
struct ElementStatic {
    /// Deformation gradient `F_e`.
    f: Matrix3<f64>,
    /// `F_e^{-T}`.
    f_inv_t: Matrix3<f64>,
    /// `J_e = det F_e`.
    j: f64,
    /// Patch ratio `J̄_e`.
    j_bar: f64,
    /// `θ_e = (J̄_e/J_e)^{1/3}`.
    theta: f64,
    /// First Piola at the modified kinematic, `P*_e = P(θ_e F_e)`.
    p_star: Matrix3<f64>,
    /// Material tangent at the modified kinematic, `C*_e = ∂P/∂F(θ_e F_e)`.
    c_star: SMatrix<f64, 9, 9>,
    /// Log-volume gradient rows `g_e(a) = F_e^{-T} ∇N_a`.
    g: [Vector3<f64>; 4],
    /// Volumetric power `π_e = P*_e : F_e`.
    pi: f64,
}

impl FbarCache {
    /// Per-element static F-bar quantities at `x_curr` (configuration-fixed;
    /// the block kernel's seeds vary only the perturbation direction).
    fn statics<M, Msh>(
        &self,
        mesh: &Msh,
        materials: &[M],
        x_curr: &[f64],
        geometries: &[ElementGeometry],
    ) -> Vec<ElementStatic>
    where
        M: Material,
        Msh: Mesh<M>,
    {
        self.forward(mesh, x_curr, geometries)
            .into_iter()
            .enumerate()
            .map(|(tet_id, ef)| {
                let geom = &geometries[tet_id];
                let fs = ef.theta * ef.f;
                let p_star = materials[tet_id].first_piola(&fs);
                ElementStatic {
                    c_star: materials[tet_id].tangent(&fs),
                    g: local_log_volume_grad(&ef.f_inv_t, &geom.grad_x_n),
                    pi: frobenius(&p_star, &ef.f),
                    p_star,
                    f: ef.f,
                    f_inv_t: ef.f_inv_t,
                    j: ef.j,
                    j_bar: ef.j_bar,
                    theta: ef.theta,
                }
            })
            .collect()
    }

    /// The patch node set for element `e`: every node of every element sharing
    /// a node with `e` (its own nodes included). Both the rows `e`'s residual
    /// derivative touches and the DOFs it depends on lie in this set, so it is
    /// the block's row/col span AND the sparsity-pattern coupling of `e`.
    fn patch_nodes<Msh, M>(&self, mesh: &Msh, e: usize) -> Vec<crate::mesh::VertexId>
    where
        M: Material,
        Msh: Mesh<M>,
    {
        let mut nodes = std::collections::BTreeSet::new();
        for &v in &mesh.tet_vertices(e as TetId) {
            for &(e2, _) in &self.incident[v as usize] {
                for &v2 in &mesh.tet_vertices(e2) {
                    nodes.insert(v2);
                }
            }
        }
        nodes.into_iter().collect()
    }

    /// The patch DOF list for element `e` — [`Self::patch_nodes`] expanded to
    /// the three DOFs of each node. The seed set for `e`'s block kernel.
    fn element_patch_dofs<Msh, M>(&self, mesh: &Msh, e: usize) -> Vec<usize>
    where
        M: Material,
        Msh: Mesh<M>,
    {
        self.patch_nodes(mesh, e)
            .into_iter()
            .flat_map(|v| (0..3).map(move |k| 3 * v as usize + k))
            .collect()
    }

    /// Insert the F-bar tangent's free-DOF lower-triangle sparsity pattern into
    /// `triplet_set` (keyed `(col_free, row_free)`). The F-bar tangent couples
    /// every pair of nodes within each element's patch — the node-star 2-ring,
    /// wider than Tet4's element-incidence 1-ring because `J̄_e` reaches the
    /// star. Dedups against pairs already present. Consumed by `construct.rs`'s
    /// symbolic-factor build so the F-bar coupling is pre-allocated.
    pub(super) fn insert_free_coupling_pattern<Msh, M>(
        &self,
        mesh: &Msh,
        full_to_free: &[Option<usize>],
        triplet_set: &mut std::collections::BTreeSet<(usize, usize)>,
    ) where
        M: Material,
        Msh: Mesh<M>,
    {
        for e in 0..mesh.n_tets() {
            let patch = self.patch_nodes(mesh, e);
            for &va in &patch {
                for &vb in &patch {
                    for i in 0..3 {
                        for j in 0..3 {
                            let (row_full, col_full) = (3 * va as usize + i, 3 * vb as usize + j);
                            if let (Some(row_free), Some(col_free)) =
                                (full_to_free[row_full], full_to_free[col_full])
                                && row_free >= col_free
                            {
                                triplet_set.insert((col_free, row_free));
                            }
                        }
                    }
                }
            }
        }
    }

    /// Accumulate the consistent F-bar tangent's **free-DOF lower-triangle**
    /// entries into `acc` (keyed `(col_free, row_free)`), matching the
    /// `assemble_free_hessian_triplets` convention. Each element's block is
    /// built by seeding [`Self::element_force_deriv`] over its patch DOFs
    /// (`O(patch²)` per element); `Σ_e` is the exact Hessian, verified against
    /// [`Self::tangent_matvec`].
    pub(super) fn accumulate_free_tangent<M, Msh>(
        &self,
        mesh: &Msh,
        materials: &[M],
        x_curr: &[f64],
        geometries: &[ElementGeometry],
        full_to_free: &[Option<usize>],
        acc: &mut std::collections::BTreeMap<(usize, usize), f64>,
    ) where
        M: Material,
        Msh: Mesh<M>,
    {
        let statics = self.statics(mesh, materials, x_curr, geometries);
        let n_dof = full_to_free.len();
        let mut seed = vec![0.0; n_dof];
        for e in 0..mesh.n_tets() {
            for &col_full in &self.element_patch_dofs(mesh, e) {
                let Some(col_free) = full_to_free[col_full] else {
                    continue;
                };
                seed[col_full] = 1.0;
                self.element_force_deriv(mesh, geometries, &statics, e, &seed, |v, force| {
                    for i in 0..3 {
                        let row_full = 3 * v as usize + i;
                        if let Some(row_free) = full_to_free[row_full]
                            && row_free >= col_free
                        {
                            *acc.entry((col_free, row_free)).or_insert(0.0) += force[i];
                        }
                    }
                });
                seed[col_full] = 0.0;
            }
        }
    }

    /// Derivative of ONE element's residual contribution in direction `delta` —
    /// the isolated per-element term of [`Self::tangent_matvec`]. `visit(v,
    /// force)` receives each `(node, Ṙ_e-contribution)`; nodes span `e`'s patch
    /// (its own nodes + the nodes of every element sharing a node with `e`).
    ///
    /// `Σ_e element_force_deriv(e, δ) = tangent_matvec(δ)` — the assembly below
    /// seeds this per element to build the exact Hessian blocks (`O(patch²)` per
    /// element), verified against the whole-mesh matvec.
    fn element_force_deriv<Msh, M>(
        &self,
        mesh: &Msh,
        geometries: &[ElementGeometry],
        statics: &[ElementStatic],
        e: usize,
        delta: &[f64],
        mut visit: impl FnMut(crate::mesh::VertexId, Vector3<f64>),
    ) where
        M: Material,
        Msh: Mesh<M>,
    {
        let se = &statics[e];
        let geom = &geometries[e];
        let verts = mesh.tet_vertices(e as TetId);

        // Ḟ_e, δln J_e, ġ_e over e's own nodes.
        let mut f_dot = Matrix3::zeros();
        let mut dlnj = 0.0;
        for a in 0..4 {
            let dx = node_delta(delta, verts[a]);
            f_dot += dx * grad_row(&geom.grad_x_n, a).transpose();
            dlnj += se.g[a].dot(&dx);
        }
        let mut g_dot = [Vector3::zeros(); 4];
        for a in 0..4 {
            g_dot[a] = -(se.f_inv_t * f_dot.transpose() * se.g[a]);
        }

        // J̄̇_e = (1/4) Σ_{a∈e} (1/W_{v_a}) Σ_{e'∋v_a} V_{e'} J̇_{e'}, with
        // J̇_{e'} = J_{e'} Σ_{b∈e'} g_{e'}(b)·δx_b.
        let jdot = |e2: usize| -> f64 {
            let s2 = &statics[e2];
            let v2 = mesh.tet_vertices(e2 as TetId);
            let mut d = 0.0;
            for a in 0..4 {
                d += s2.g[a].dot(&node_delta(delta, v2[a]));
            }
            s2.j * d
        };
        let mut jbar_dot = 0.0;
        for &v in &verts {
            let w_v = self.nodal_volume[v as usize];
            let mut acc = 0.0;
            for &(e2, _) in &self.incident[v as usize] {
                acc += geometries[e2 as usize].volume * jdot(e2 as usize);
            }
            jbar_dot += acc / w_v;
        }
        jbar_dot *= 0.25;

        let theta_dot = (se.theta / 3.0) * (jbar_dot / se.j_bar - dlnj);
        let fs_dot = theta_dot * se.f + se.theta * f_dot;
        let p_dot = contract_tangent(&se.c_star, &fs_dot);
        let pi_dot = frobenius(&p_dot, &se.f) + frobenius(&se.p_star, &f_dot);

        // Local force derivative on e's own nodes.
        for a in 0..4 {
            let grad_n = grad_row(&geom.grad_x_n, a);
            let term = theta_dot * (se.p_star * grad_n - (se.pi / 3.0) * se.g[a])
                + se.theta * (p_dot * grad_n - (pi_dot / 3.0) * se.g[a] - (se.pi / 3.0) * g_dot[a]);
            visit(verts[a], geom.volume * term);
        }

        // Patch scatter of e's own adjoint deposit and its derivative.
        let deposit = geom.volume * se.theta * se.pi / (12.0 * se.j_bar);
        let deposit_dot = geom.volume / 12.0
            * (theta_dot * se.pi / se.j_bar + se.theta * pi_dot / se.j_bar
                - se.theta * se.pi * jbar_dot / (se.j_bar * se.j_bar));
        for &v in &verts {
            let w_v = self.nodal_volume[v as usize];
            let cw = deposit / w_v;
            let cwdot = deposit_dot / w_v;
            for &(e2, _) in &self.incident[v as usize] {
                let s2 = &statics[e2 as usize];
                let v2 = mesh.tet_vertices(e2);
                let vol2 = geometries[e2 as usize].volume;
                let jdot2 = jdot(e2 as usize);
                // ġ_{e'}(b) for the neighbor element.
                let mut fdot2 = Matrix3::zeros();
                for a in 0..4 {
                    fdot2 += node_delta(delta, v2[a])
                        * grad_row(&geometries[e2 as usize].grad_x_n, a).transpose();
                }
                for a in 0..4 {
                    let gd2 = -(s2.f_inv_t * fdot2.transpose() * s2.g[a]);
                    let force =
                        cwdot * vol2 * s2.j * s2.g[a] + cw * vol2 * (jdot2 * s2.g[a] + s2.j * gd2);
                    visit(v2[a], force);
                }
            }
        }
    }
}

/// One node's 3-vector slice from a flat DOF buffer.
fn node_delta(delta: &[f64], v: crate::mesh::VertexId) -> Vector3<f64> {
    let v = v as usize;
    Vector3::new(delta[3 * v], delta[3 * v + 1], delta[3 * v + 2])
}

/// Scatter a 3-vector force onto node `v` in a flat DOF buffer.
fn add_node(out: &mut [f64], v: crate::mesh::VertexId, force: &Vector3<f64>) {
    let v = v as usize;
    out[3 * v] += force.x;
    out[3 * v + 1] += force.y;
    out[3 * v + 2] += force.z;
}

/// Contract the 9×9 material tangent with a matrix perturbation, BF-5
/// flattening (`tangent[(i+3j, k+3l)] = ∂P_ij/∂F_kl`): `(C:A)_ij = Σ_kl
/// C[(i+3j),(k+3l)] A_kl`.
fn contract_tangent(c: &SMatrix<f64, 9, 9>, a: &Matrix3<f64>) -> Matrix3<f64> {
    let mut out = Matrix3::zeros();
    for i in 0..3 {
        for j in 0..3 {
            let mut s = 0.0;
            for k in 0..3 {
                for l in 0..3 {
                    s += c[(i + 3 * j, k + 3 * l)] * a[(k, l)];
                }
            }
            out[(i, j)] = s;
        }
    }
    out
}

/// Local log-volume gradient rows `g_e(a) = F_e^{-T} ∇N_a`: row `a` (0..4) is
/// the 3-vector `∂ln J_e/∂x_a`.
fn local_log_volume_grad(
    f_inv_t: &Matrix3<f64>,
    grad_x_n: &SMatrix<f64, 4, 3>,
) -> [Vector3<f64>; 4] {
    let mut out = [Vector3::zeros(); 4];
    for a in 0..4 {
        out[a] = f_inv_t * grad_row(grad_x_n, a);
    }
    out
}

/// Rest shape gradient `∇N_a` (row `a` of `grad_x_n`) as a column 3-vector.
fn grad_row(grad_x_n: &SMatrix<f64, 4, 3>, a: usize) -> Vector3<f64> {
    Vector3::new(grad_x_n[(a, 0)], grad_x_n[(a, 1)], grad_x_n[(a, 2)])
}

/// Frobenius inner product `A : B = Σ_ij A_ij B_ij`.
fn frobenius(a: &Matrix3<f64>, b: &Matrix3<f64>) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

/// `F^{-T}`.
//
// F-bar shares `NeoHookean`'s `RequireOrientation` contract: a non-invertible
// `F` is an upstream IPC-barrier failure, not a branch here — so the expect is
// a programmer-error tripwire, matching `neo_hookean::invert_transpose`.
#[allow(clippy::expect_used)]
fn invert_transpose(f: &Matrix3<f64>) -> Matrix3<f64> {
    f.try_inverse()
        .expect("non-invertible F in F-bar transform; IPC barrier should prevent this")
        .transpose()
}

#[cfg(test)]
// Test-only: the deterministic LCG that seeds random perturbation directions
// casts its `u64` state to `f64` (a full-width cast is fine for a test RNG), the
// standalone geometry rebuild casts node counts, and `expect` on a non-singular
// rest Jacobian is a fixture invariant.
#[allow(clippy::cast_precision_loss, clippy::expect_used)]
mod tests {
    use nalgebra::Matrix3;

    use super::*;
    use crate::material::{MaterialField, NeoHookean};
    use crate::mesh::HandBuiltTetMesh;
    use crate::solver::backward_euler::ElementGeometry;

    /// ν = 0.49 Lamé pair (near-incompressible, the F-bar target regime),
    /// built via `from_lame` to bypass the standalone ν<0.45 gate.
    fn nu049_field() -> MaterialField {
        let mu = 1.0e5;
        let nu = 0.49;
        let lambda = 2.0 * mu * nu / (1.0 - 2.0 * nu);
        MaterialField::uniform(mu, lambda)
    }

    /// Rebuild the per-element rest geometry the solver caches, standalone —
    /// the F-bar module is unit-tested without a full `CpuNewtonSolver`.
    fn geometries(mesh: &HandBuiltTetMesh) -> (Vec<ElementGeometry>, Vec<f64>) {
        use crate::element::{Element, Tet4};
        let x_rest = mesh.positions();
        let grad_xi = Tet4.shape_gradients(crate::Vec3::new(0.25, 0.25, 0.25));
        let mut geoms = Vec::new();
        let mut vols = Vec::new();
        for tet_id in 0..mesh.n_tets() as TetId {
            let verts = mesh.tet_vertices(tet_id);
            let v0 = x_rest[verts[0] as usize];
            let v1 = x_rest[verts[1] as usize];
            let v2 = x_rest[verts[2] as usize];
            let v3 = x_rest[verts[3] as usize];
            let j0 = Matrix3::from_columns(&[v1 - v0, v2 - v0, v3 - v0]);
            let volume = j0.determinant().abs() / 6.0;
            let grad_x_n = grad_xi * j0.try_inverse().expect("non-singular rest Jacobian");
            geoms.push(ElementGeometry { grad_x_n, volume });
            vols.push(volume);
        }
        (geoms, vols)
    }

    /// The load-bearing consistency gate: the assembled F-bar internal force
    /// equals the finite-difference gradient of the F-bar energy, on a
    /// two-tet mesh with a genuine shared face (so the patch coupling is
    /// exercised) at a non-trivial deformed configuration and ν = 0.49.
    #[test]
    fn internal_force_matches_energy_finite_difference() {
        let field = nu049_field();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let (geoms, _vols) = geometries(&mesh);
        let materials: Vec<NeoHookean> = mesh.materials().to_vec();
        let cache = FbarCache::build(&mesh, &geoms);

        // Deformed config: rest + a smooth non-uniform, non-isochoric warp so
        // J_e ≠ J̄_e and every force term is non-zero.
        let rest = mesh.positions();
        let n_dof = 3 * rest.len();
        let mut x = vec![0.0; n_dof];
        for (v, p) in rest.iter().enumerate() {
            x[3 * v] = p.x * 1.07 + 0.003 * p.y;
            x[3 * v + 1] = p.y * 0.96 - 0.002 * p.z;
            x[3 * v + 2] = p.z * 1.02 + 0.004 * p.x;
        }

        let mut f_int = vec![0.0; n_dof];
        cache.scatter_internal_force(&mesh, &materials, &x, &geoms, &mut f_int);

        let e = |xv: &[f64]| cache.energy(&mesh, &materials, xv, &geoms);
        let h = 1e-7;
        let mut max_rel = 0.0_f64;
        for i in 0..n_dof {
            let mut xp = x.clone();
            let mut xm = x.clone();
            xp[i] += h;
            xm[i] -= h;
            let fd = (e(&xp) - e(&xm)) / (2.0 * h);
            let denom = f_int[i].abs().max(fd.abs()).max(1e-6);
            let rel = (f_int[i] - fd).abs() / denom;
            max_rel = max_rel.max(rel);
        }
        assert!(
            max_rel < 1e-5,
            "F-bar internal force disagrees with energy FD (max rel {max_rel:e})"
        );
    }

    /// The tangent gate: `K·δ` (the analytic directional derivative of the
    /// internal force) equals the finite-difference of the *proven* residual in
    /// direction `δ`, on the shared-face mesh at ν = 0.49 for several random
    /// directions. Validates the consistent F-bar tangent — including the
    /// `J̄`-neighbor coupling — against the residual that already matched energy
    /// FD, so the two gates chain into a full second-derivative check.
    #[test]
    fn tangent_matvec_matches_residual_finite_difference() {
        let field = nu049_field();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let (geoms, _vols) = geometries(&mesh);
        let materials: Vec<NeoHookean> = mesh.materials().to_vec();
        let cache = FbarCache::build(&mesh, &geoms);

        let rest = mesh.positions();
        let n_dof = 3 * rest.len();
        let mut x = vec![0.0; n_dof];
        for (v, p) in rest.iter().enumerate() {
            x[3 * v] = p.x * 1.07 + 0.003 * p.y;
            x[3 * v + 1] = p.y * 0.96 - 0.002 * p.z;
            x[3 * v + 2] = p.z * 1.02 + 0.004 * p.x;
        }

        let residual = |xv: &[f64]| {
            let mut f = vec![0.0; n_dof];
            cache.scatter_internal_force(&mesh, &materials, xv, &geoms, &mut f);
            f
        };

        // A few deterministic directions (a simple LCG, since Math::random is
        // unavailable): each exercises the full coupled block.
        let mut seed = 0x1234_5678_u64;
        let mut next = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            ((seed >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        for _trial in 0..4 {
            let delta: Vec<f64> = (0..n_dof).map(|_| next()).collect();
            let kd = cache.tangent_matvec(&mesh, &materials, &x, &geoms, &delta);

            let h = 1e-6;
            let xp: Vec<f64> = x.iter().zip(&delta).map(|(a, d)| a + h * d).collect();
            let xm: Vec<f64> = x.iter().zip(&delta).map(|(a, d)| a - h * d).collect();
            let fp = residual(&xp);
            let fm = residual(&xm);

            let mut max_rel = 0.0_f64;
            for i in 0..n_dof {
                let fd = (fp[i] - fm[i]) / (2.0 * h);
                let denom = kd[i].abs().max(fd.abs()).max(1e-3);
                max_rel = max_rel.max((kd[i] - fd).abs() / denom);
            }
            assert!(
                max_rel < 1e-5,
                "F-bar tangent K·δ disagrees with residual FD (max rel {max_rel:e})"
            );
        }
    }

    /// The block-assembly gate: the sparse triplet assembly
    /// ([`FbarCache::accumulate_free_tangent`]) reconstructs the *proven*
    /// [`FbarCache::tangent_matvec`] exactly. Assembles the full-DOF lower
    /// triangle (identity free-map), rebuilds `K·δ` from it exploiting symmetry,
    /// and compares to the matvec across random directions — so the O(patch²)
    /// block kernel is verified against the whole-mesh reverse-mode form. Also
    /// confirms `K` is symmetric (the reconstruction assumes it).
    #[test]
    fn block_assembly_matches_tangent_matvec() {
        let field = nu049_field();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let (geoms, _vols) = geometries(&mesh);
        let materials: Vec<NeoHookean> = mesh.materials().to_vec();
        let cache = FbarCache::build(&mesh, &geoms);

        let rest = mesh.positions();
        let n_dof = 3 * rest.len();
        let mut x = vec![0.0; n_dof];
        for (v, p) in rest.iter().enumerate() {
            x[3 * v] = p.x * 1.07 + 0.003 * p.y;
            x[3 * v + 1] = p.y * 0.96 - 0.002 * p.z;
            x[3 * v + 2] = p.z * 1.02 + 0.004 * p.x;
        }

        // Full-DOF lower triangle (every DOF free).
        let identity: Vec<Option<usize>> = (0..n_dof).map(Some).collect();
        let mut acc = std::collections::BTreeMap::new();
        cache.accumulate_free_tangent(&mesh, &materials, &x, &geoms, &identity, &mut acc);

        let mut seed = 0x9E37_79B9_u64;
        let mut next = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            ((seed >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        for _trial in 0..4 {
            let delta: Vec<f64> = (0..n_dof).map(|_| next()).collect();
            // Reconstruct K·δ from the lower-triangle, using symmetry.
            let mut kd = vec![0.0; n_dof];
            for (&(col, row), &val) in &acc {
                kd[row] += val * delta[col];
                if row != col {
                    kd[col] += val * delta[row];
                }
            }
            let reference = cache.tangent_matvec(&mesh, &materials, &x, &geoms, &delta);

            let mut max_rel = 0.0_f64;
            for i in 0..n_dof {
                let denom = kd[i].abs().max(reference[i].abs()).max(1e-3);
                max_rel = max_rel.max((kd[i] - reference[i]).abs() / denom);
            }
            assert!(
                max_rel < 1e-9,
                "block-assembled tangent disagrees with matvec (max rel {max_rel:e})"
            );
        }
    }

    /// High-valence hardening: re-run the residual (vs energy-FD) and tangent
    /// (vs residual-FD) consistency checks on a `uniform_block(2)` mesh, whose
    /// interior nodes sit in many tets (valence ≫ 2). The `two_tet_shared_face`
    /// gates above exercise only valence-2 stars; this confirms the `J̄`
    /// nodal-average and its linearization stay exact when a node's patch spans
    /// a large element star — so "exact Hessian" does not rest on minimal
    /// topology.
    #[test]
    fn fd_consistency_on_high_valence_block() {
        let field = nu049_field();
        let mesh = HandBuiltTetMesh::uniform_block(2, 0.1, &field);
        let (geoms, _vols) = geometries(&mesh);
        let materials: Vec<NeoHookean> = mesh.materials().to_vec();
        let cache = FbarCache::build(&mesh, &geoms);

        // Non-uniform, non-isochoric warp so J_e ≠ J̄_e across the star.
        let rest = mesh.positions();
        let n_dof = 3 * rest.len();
        let mut x = vec![0.0; n_dof];
        for (v, p) in rest.iter().enumerate() {
            x[3 * v] = p.x * 1.05 + 0.02 * p.y - 0.01 * p.z;
            x[3 * v + 1] = p.y * 0.97 + 0.015 * p.z;
            x[3 * v + 2] = p.z * 1.03 - 0.012 * p.x;
        }

        // Residual == ∂E/∂x (energy central-difference). Vector-∞-norm-relative
        // metric (`max|Δ| / max|f|`): a symmetric block's center node carries a
        // ~0 net force, so a per-DOF ratio would divide energy-FD roundoff
        // (~1e-7) by that ~0 and false-positive — the global force scale is the
        // honest denominator. (The two-tet gates have no near-zero-force node,
        // so their per-DOF metric there is fine.)
        let mut f_int = vec![0.0; n_dof];
        cache.scatter_internal_force(&mesh, &materials, &x, &geoms, &mut f_int);
        let energy = |xv: &[f64]| cache.energy(&mesh, &materials, xv, &geoms);
        let h = 1e-7;
        let mut max_abs = 0.0_f64;
        let mut force_scale = 0.0_f64;
        for i in 0..n_dof {
            let mut xp = x.clone();
            let mut xm = x.clone();
            xp[i] += h;
            xm[i] -= h;
            let fd = (energy(&xp) - energy(&xm)) / (2.0 * h);
            max_abs = max_abs.max((f_int[i] - fd).abs());
            force_scale = force_scale.max(f_int[i].abs());
        }
        assert!(
            max_abs / force_scale < 1e-5,
            "high-valence F-bar residual disagrees with energy FD \
             (max |Δ| {max_abs:e} / force scale {force_scale:e})"
        );

        // Tangent K·δ == residual central-difference in direction δ.
        let residual = |xv: &[f64]| {
            let mut f = vec![0.0; n_dof];
            cache.scatter_internal_force(&mesh, &materials, xv, &geoms, &mut f);
            f
        };
        let mut seed = 0xC0FF_EE42_u64;
        let mut next = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            ((seed >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        for _trial in 0..3 {
            let delta: Vec<f64> = (0..n_dof).map(|_| next()).collect();
            let kd = cache.tangent_matvec(&mesh, &materials, &x, &geoms, &delta);
            let hd = 1e-6;
            let xp: Vec<f64> = x.iter().zip(&delta).map(|(a, d)| a + hd * d).collect();
            let xm: Vec<f64> = x.iter().zip(&delta).map(|(a, d)| a - hd * d).collect();
            let fp = residual(&xp);
            let fm = residual(&xm);
            let mut max_abs = 0.0_f64;
            let mut kd_scale = 0.0_f64;
            for i in 0..n_dof {
                let fd = (fp[i] - fm[i]) / (2.0 * hd);
                max_abs = max_abs.max((kd[i] - fd).abs());
                kd_scale = kd_scale.max(kd[i].abs());
            }
            assert!(
                max_abs / kd_scale < 1e-5,
                "high-valence F-bar tangent K·δ disagrees with residual FD \
                 (max |Δ| {max_abs:e} / scale {kd_scale:e})"
            );
        }
    }
}
