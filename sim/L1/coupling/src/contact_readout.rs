//! Contact readout and pressure observables — the per-pair contact geometry, force, and
//! peak/vertex pressure fields sampled at a plane height (no gradients).

use sim_soft::{
    ContactPair, ContactPairReadout, HandBuiltTetMesh, RigidPlane, Vec3, peak_contact_pressure,
};

use crate::StaggeredCoupling;

use crate::contact::{Collider, PlaneContact, posed_sphere};

impl<C: PlaneContact> StaggeredCoupling<C> {
    /// Assert no contact end-effector geom is set ([`Self::with_contact_geom`]) — the scope
    /// guard for every sphere-capable trajectory method that does NOT thread the
    /// moving-end-effector centre carry.
    ///
    /// Every sphere-capable gradient is curvature-correct on a CENTROID sphere (the contact nodes
    /// carry the #415–#429 curvature; the actuator `g_act` channel is contact-independent). The
    /// *moving* end-effector (the 3-vector centre channel, posing at + differentiating through
    /// `geom_xpos(q)`) is threaded by the ARTICULATED normal/friction gradients (#428/#429) AND the
    /// ACTUATOR/POLICY gradients (and all their forward oracles —
    /// [`Self::coupled_trajectory_articulated_z`], [`Self::coupled_trajectory_gripped_articulated`],
    /// the actuated/policy `*_z`/`*_gripped_x` oracles). The remaining guarded set — which would
    /// otherwise silently pose the sphere at the block centroid (ignoring the tip), a silent
    /// contract violation on a public API (ship-blocking even if currently unused) — is the
    /// FREE-BODY gradients (`material`/`peak_force`/`control`/`policy`/`joint`/`tangential_*`) and
    /// the free-body grip forward ([`Self::coupled_trajectory_grip`]). A no-op when no geom is set
    /// (the centroid-posed gates + the plane).
    ///
    /// **The free-body guard is a DELIBERATE boundary, not an unfinished follow-on** (verified by a
    /// spike, 2026-06-27). A moving end-effector on the free platen is DEGENERATE — there is no real
    /// centre channel to thread:
    /// - The free-body coupling threads scalar lanes (vertical `z`/`vz`, plus lateral `xx`/`vx` for
    ///   friction) with NO off-COM moment, so the contact's absolute lateral *position* feeds no
    ///   gradient channel (unlike the articulated arm, whose tip arcs and whose off-COM moment makes
    ///   `tip_z` pose-sensitive).
    /// - Over the FLAT block the contact is laterally translation-invariant (a sphere staying over
    ///   the block feels the same force at any x), and the collider's relative sweep is ALREADY the
    ///   drift channel (`Δ_surf = v_collider·dt`). So a `geom_xpos` centre channel adds nothing the
    ///   drift + height don't (consistent with #429's pose-insensitive friction `tip_x`).
    /// - Naive geom-posing isn't even the same model: the geom centre is `xpos.z`, vs the keystone's
    ///   `height + r` south-pole reference — a vertical-convention mismatch, not a tip-tracking gain.
    ///
    /// So this guard stays as the correct LOUD rejection; "completing" the free body would mean a
    /// full 6-DOF rigid model (a different effort), not a moving-EE carry.
    pub(super) fn require_no_moving_ee(&self) {
        assert!(
            self.contact_geom.is_none(),
            "a moving end-effector (with_contact_geom) is not supported on the free-body path — it \
             is DEGENERATE there (flat-block translation-invariance + no off-COM moment; the \
             collider sweep is already the drift channel). Use the ARTICULATED or ACTUATOR/POLICY \
             gradients for a tip-posed sphere, or the block-centroid default here."
        );
    }

    pub(super) fn build_contact(&self, height: f64) -> C {
        if let Collider::Sphere { radius } = self.collider {
            let center = self.sphere_center(radius, height);
            return C::from_primitive(posed_sphere(radius, center), self.kappa, self.d_hat);
        }
        let plane = if self.rotating_normal {
            // Rotating normal: the plane tracks the body ORIENTATION (`n̂ = R·(0,0,−1)`)
            // while still honoring the scalar `height` like the flat branch — the plane
            // passes through the reference point at the body's lateral position and world
            // z = `height + clearance`, so `offset = p_ref·n̂ + clearance`. This keeps
            // `height` a live parameter (the S3 single-step probes translate the tilted
            // plane vertically by varying it) AND reduces to the flat `((0,0,−1),
            // −height)` at `R = I`. In the coupled path `height = xipos.z − clearance`, so
            // `p_ref = xipos` exactly (byte-identical to reading the live COM pose).
            let n = self.data.xquat[self.body] * Vec3::new(0.0, 0.0, -1.0);
            let com = self.data.xipos[self.body];
            let p_ref = Vec3::new(com.x, com.y, height + self.contact_clearance);
            let offset = p_ref.dot(&n) + self.contact_clearance;
            RigidPlane::new(n, offset)
        } else {
            // Downward ceiling at `height`: normal −z, offset −height ⇒ a soft
            // vertex below the plane has positive signed distance `height − z`.
            RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height)
        };
        C::from_primitive(plane, self.kappa, self.d_hat)
    }

    /// A fresh rest-topology soft mesh (the solver consumes one per step, and
    /// `per_pair_readout` needs one too; topology + rest geometry are fixed).
    pub(super) fn fresh_mesh(&self) -> HandBuiltTetMesh {
        HandBuiltTetMesh::uniform_block(self.n_per_edge, self.edge, &self.field)
    }

    /// Current soft vertex positions as points (vertex-major xyz → `Vec3`s).
    pub(super) fn positions(&self) -> Vec<Vec3> {
        self.x
            .chunks_exact(3)
            .map(|c| Vec3::new(c[0], c[1], c[2]))
            .collect()
    }

    /// Per-active-pair readouts at the current soft configuration with the
    /// collider posed at `height` — does not re-solve or mutate. The shared
    /// building block for the total-force readout, its pose-sensitivity, and
    /// the peak-pressure readout (so all three see the same posed contact).
    pub(super) fn pair_readouts_at_height(&self, height: f64) -> Vec<ContactPairReadout> {
        self.build_contact(height)
            .pair_readout(&self.fresh_mesh(), &self.positions())
    }

    /// `(total force_on_soft, active-pair count)` at the current soft
    /// configuration with the contact plane placed at `height` — does not
    /// re-solve or mutate. The shared building block for the contact-force
    /// readout and its analytic pose-sensitivity.
    fn contact_readout(&self, height: f64) -> (Vec3, usize) {
        let readout = self.pair_readouts_at_height(height);
        (readout.iter().map(|r| r.force_on_soft).sum(), readout.len())
    }

    /// Peak contact *pressure* (Pa) — the max per-contact-face stress over the
    /// active pairs ([`peak_contact_pressure`]) — at the current soft
    /// configuration with the collider posed at `height`. Companion to
    /// [`Self::contact_force_at_height`]; does not re-solve or mutate. The
    /// measured local concentration total force cannot see (a finite sphere:
    /// high peak pressure at low total force; a broad slab: the reverse).
    #[must_use]
    pub fn contact_peak_pressure_at_height(&self, height: f64) -> f64 {
        peak_contact_pressure(&self.pair_readouts_at_height(height))
    }

    /// Per-vertex contact pressure field (Pa) at the current soft configuration with the collider
    /// posed at `height` — the per-vertex form of [`Self::contact_peak_pressure_at_height`], for a
    /// pressure *field* (e.g. a deformed-surface heatmap). Length `n_vertices`, indexed by
    /// `VertexId`: each entry is the max per-face stress over that vertex's active pairs, `0.0`
    /// where the vertex is not in contact, or `f64::NAN` where it IS in contact but every pair is
    /// degenerate (zero tributary area) — the same NaN-degenerate sentinel
    /// [`peak_contact_pressure`] uses, so a degenerate contact reads off-nominal rather than as a
    /// danger-masking zero, and the field's finite max equals the scalar peak. Does not re-solve or
    /// mutate. Indices line up with `soft_positions` / `soft_boundary_faces`, so a renderer drops
    /// the values straight onto the surface mesh's vertices.
    ///
    /// ⚠ For a forward-replay heatmap that must match the just-solved deformation, use
    /// [`Self::step_with_pressure_field`] instead of calling this after [`Self::step`] — `step`
    /// advances the rigid body, so the live `plane_height` is one timestep ahead of the contact the
    /// field should describe.
    #[must_use]
    pub fn contact_vertex_pressures_at_height(&self, height: f64) -> Vec<f64> {
        self.vertex_pressures_from(&self.pair_readouts_at_height(height))
    }

    /// Scatter a set of active-pair readouts into a per-vertex pressure field (the shared core of
    /// [`Self::contact_vertex_pressures_at_height`] and [`Self::step_with_pressure_field`], so both
    /// see the same max-of-faces + NaN-degenerate reduction as [`peak_contact_pressure`]).
    pub(super) fn vertex_pressures_from(&self, readouts: &[ContactPairReadout]) -> Vec<f64> {
        let mut field = vec![0.0_f64; self.n_vertices];
        let mut any_pair = vec![false; self.n_vertices];
        let mut any_finite = vec![false; self.n_vertices];
        for r in readouts {
            let ContactPair::Vertex { vertex_id, .. } = r.pair;
            let v = vertex_id as usize;
            any_pair[v] = true;
            if r.pressure.is_finite() {
                any_finite[v] = true;
                field[v] = field[v].max(r.pressure);
            }
        }
        // A vertex in contact whose every pair was degenerate (no finite pressure) is off-nominal —
        // the NaN sentinel, not a reassuring 0.0 (matches `peak_contact_pressure`'s altitude).
        for v in 0..self.n_vertices {
            if any_pair[v] && !any_finite[v] {
                field[v] = f64::NAN;
            }
        }
        field
    }

    /// Total contact force the soft body exerts, evaluated at the current soft
    /// configuration with the contact plane at `height` (no re-solve, no
    /// mutation). The forward building block for finite-difference and analytic
    /// contact-force sensitivities w.r.t. the rigid pose.
    #[must_use]
    pub fn contact_force_at_height(&self, height: f64) -> Vec3 {
        self.contact_readout(height).0
    }

    /// Per active contact pair at `positions` with the collider posed at `height`:
    /// `(vertex_id, ∂fz/∂x_v, ∂fz/∂height)` — the PRECOMPUTED z-force gradient factors
    /// the free-body contact crossing scatters ([`ContactForceVjp`], [`ContactForceTrajVjp`]),
    /// curvature-correct for any collider (the L1b carry, dual to the FD-validated
    /// single-step [`Self::contact_force_height_total_jacobian`]).
    ///
    /// Differentiating the total normal force `fz = Σ gᵢ·ẑ = Σ f_mag·n̂_z` (per-pair force
    /// `gᵢ = f_mag·n̂`, `f_mag = gᵢ·n̂`) splits into a MAGNITUDE change (`cᵥ = d²E/dsd²`, the
    /// flat term) and a NORMAL-ROTATION change (`f_mag·∂n̂`, #415's geometric stiffness
    /// `H = ∇²sd` = [`Self::collider_hessian`]). Taking the z-row of the per-pair force
    /// Jacobian `∂force/∂x_v = −cᵥ·n̂⊗n̂ + f_mag·H` and the explicit height partial
    /// `∂force/∂h = cᵥ·n̂_z·n̂ + f_mag·(−H·ẑ)` (raising the height translates the primitive
    /// `+ẑ`, `∂sd/∂h = −n̂_z`, `∂n̂/∂h = −H·ẑ`):
    /// - `∂fz/∂x_v = −cᵥ·n̂_z·n̂ + f_mag·(H·ẑ)`
    /// - `∂fz/∂height = cᵥ·n̂_z² − f_mag·(H·ẑ)_z`
    ///
    /// `H = 0` for the plane, where additionally `n̂ = −ẑ` exactly ⇒ `∂fz/∂x_v = (0,0,−cᵥ)`
    /// and `∂fz/∂height = cᵥ` — BYTE-IDENTICAL to the old flat `−cᵥ·n̂⊗n̂` / `Σ cᵥ` factors
    /// (#402–#406 untouched). For the sphere the `H·ẑ` terms are what make the free-body
    /// trajectory gradients match the curved re-solve FD. Read from
    /// [`Self::active_pair_wrench_data`] + [`Self::collider_hessian`]; does not re-solve/mutate.
    ///
    /// NOT a guard: the NORMAL contact crossing is curvature-correct for the sphere, as are the
    /// free-body + articulated FRICTION wrench paths (curved `DN·C` carried by #419 / the
    /// articulated-friction rung) AND the actuator/policy gradients. The MOVING end-effector centre
    /// channel is threaded by the articulated normal/friction + actuator/policy gradients; only the
    /// FREE-BODY gradients still guard it ([`Self::require_no_moving_ee`]).
    pub(super) fn active_pair_force_factors(
        &self,
        height: f64,
        positions: &[Vec3],
    ) -> Vec<(usize, Vec3, f64)> {
        let zhat = Vec3::new(0.0, 0.0, 1.0);
        self.active_pair_wrench_data(height, positions)
            .into_iter()
            .map(|(v, g, n, curv, _r)| {
                let f_mag = g.dot(&n);
                // H·ẑ = the z-row of the symmetric contact Hessian H = ∇²sd (#415);
                // 0 for the plane, the sphere's `(I − n̂n̂ᵀ)/‖p − c‖ · ẑ` for the finite collider.
                let h_zhat = self.collider_hessian(height, positions[v]) * zhat;
                let dfz_dxv = -curv * n.z * n + f_mag * h_zhat;
                let dfz_dz = curv * n.z * n.z - f_mag * h_zhat.z;
                (v, dfz_dxv, dfz_dz)
            })
            .collect()
    }
}
