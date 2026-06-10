//! Keystone S4 (PR2) — the soft-tape `VjpOp` crossing: ONE `tape.backward`
//! across both engines.
//!
//! `StaggeredCoupling::coupled_step_load_gradient` builds one differentiable
//! coupled step on a single chassis tape —
//! `theta →(NewtonStepVjp) x* →(ContactForceVjp) fz →(neg) xfrc →(RigidStepVjp) vz'`
//! — and `tape.backward(vz')` flows the cotangent `vz' → xfrc → force → x* →
//! theta`, crossing the soft↔rigid interface. The rigid + contact-force VjpOps
//! compose with the soft Newton load adjoint; this is the gradient substrate
//! the co-design optimizer consumes.
//!
//! The gate is an INDEPENDENT numeric check: the oracle
//! (`coupled_step_load_vz`) re-runs the full nonlinear coupled step (re-solve
//! the soft Newton problem → contact force → rigid step) at `theta ± ε` and
//! central-differences `vz'`, touching none of the tape / VjpOp machinery.
//!
//! Scope: load handle `theta` (the soft material-param VJP is the next leaf and
//! rides the same crossing); single staggered step (plane fixed during the soft
//! solve); contact-engaged, stable-active-set, hard-penalty regime (penalty
//! active-set non-smooth — IPC deferred). See `docs/keystone/s4_vjp_crossing_recon.md`.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;
use sim_soft::{HandBuiltTetMesh, MaterialField, VertexId, pick_vertices_by_predicate};

const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

// Block params mirror the StaggeredCoupling fixture (n_per_edge=4, edge=0.1,
// μ=3e4 → field uniform(μ, 4μ)); used only to recover the top-face vertex ids.
const N_PER_EDGE: usize = 4;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;

fn coupling() -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("MJCF");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, N_PER_EDGE, EDGE, MU, 1.0e-3, 3.0e4, 1.0e-2, 12.0,
    )
}

fn top_face_loaded() -> Vec<VertexId> {
    let mesh =
        HandBuiltTetMesh::uniform_block(N_PER_EDGE, EDGE, &MaterialField::uniform(MU, 4.0 * MU));
    pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE).abs() < 1e-9)
}

#[test]
fn cross_engine_backward_matches_full_coupled_fd() {
    let c = coupling();
    let loaded = top_face_loaded();
    assert!(!loaded.is_empty(), "top face must have loadable vertices");

    // The one-tape ∂vz'/∂theta vs the independent full-coupled-step central FD
    // at one operating point `(height, theta)`. Both `height`s penetrate the
    // rest top face (z=0.1), so the active set is the 25 top vertices.
    let check = |h: f64, theta0: f64| {
        let (vz, grad) = c.coupled_step_load_gradient(h, &loaded, theta0);
        let eps = 1.0e-4;
        let fd = (c.coupled_step_load_vz(h, &loaded, theta0 + eps)
            - c.coupled_step_load_vz(h, &loaded, theta0 - eps))
            / (2.0 * eps);
        let rel = (grad - fd).abs() / fd.abs();
        eprintln!(
            "S4 crossing @ (h={h}, θ={theta0}): vz'={vz:.6e}  tape ∂vz'/∂θ={grad:.6e}  \
             full-coupled FD={fd:.6e}  rel={rel:.3e}"
        );
        assert!(vz.is_finite(), "vz' not finite: {vz}");
        // Non-degenerate: the load genuinely moves the rigid outcome via contact.
        assert!(
            fd.abs() > 1e-4,
            "degenerate gate: ∂vz'/∂θ ≈ 0 ({fd}) — load not coupling to the rigid body?"
        );
        // HEADLINE: the one-tape cross-engine gradient matches the independent
        // nonlinear re-solve FD. Bound is FD/Newton-tol-limited (oracle re-runs
        // Newton to tol 1e-10, central diff divides by 2ε≈2e-4); observed ~5e-10.
        assert!(
            rel < 1e-6,
            "tape grad disagrees with full-coupled FD at (h={h}, θ={theta0}) \
             (rel {rel:.3e}): {grad} vs {fd}"
        );
    };

    // Two operating points — a lightly-loaded shallow-engaged config and a
    // more-loaded deeper-engaged one — so the crossing is exercised away from a
    // single corner (the soft θ→x*→fz map is the nonlinear leg; the rigid leg
    // is exactly affine).
    check(0.099, 5.0);
    check(0.097, 9.0);
}
