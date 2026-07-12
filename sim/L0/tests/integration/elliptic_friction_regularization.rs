//! Regression guard — elliptic contact friction rows must inherit the normal
//! row's regularization (R/D).
//!
//! MuJoCo computes ONE impedance per contact (from the normal/penetration) and
//! applies the same imp/diagApprox/R/D to every condim row. A prior bug derived
//! each elliptic friction row's `R` from its own (tangential) `diagApprox`, whose
//! exact `J·M⁻¹·Jᵀ` carries a lever-arm term (≈ r²/I) that MuJoCo omits. That left
//! elliptic friction rows ~10³× too soft, starving tangential grip: a fast oblique
//! tennis-ball bounce (Cross 2002, Emery-45) reached only ~27% of rolling instead
//! of matching MuJoCo (and reality) at ~89%.
//!
//! These tests pin both the structural invariant (rows share R/D) and the
//! MuJoCo-conformant numbers for a frozen contact state.

use sim_core::ConstraintType;
use sim_mjcf::load_model;

/// Sphere-on-plane with an explicit elliptic contact pair (condim 3, μ=0.67).
const ELLIPTIC_PAIR_MJCF: &str = r#"
<mujoco model="elliptic_contact">
  <option timestep="1e-5" cone="elliptic"/>
  <compiler angle="radian"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" pos="0 0 0"/>
    <body name="ball" pos="0 0 0.033">
      <freejoint/>
      <inertial pos="0 0 0" mass="0.057" diaginertia="4.0e-5 4.0e-5 4.0e-5"/>
      <geom name="ball" type="sphere" size="0.033"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="floor" geom2="ball" condim="3"
          friction="0.67 0.67 0.005 0.0001 0.0001" solref="0.001 0.08"/>
  </contact>
</mujoco>
"#;

/// Freeze a penetrating, tangentially-sliding, spinning state and assemble.
///
/// `bodyweight` selects the DT-39 invweight diagApprox approximation vs the exact
/// `M⁻¹` solve. The fix must be conformant in both: MuJoCo's own default is the
/// invweight approximation, and it reports friction R = normal R in both regimes.
fn frozen_contact_data(bodyweight: bool) -> sim_core::Data {
    let r = 0.033;
    let mut model = load_model(ELLIPTIC_PAIR_MJCF).expect("load elliptic pair model");
    model.diagapprox_bodyweight = bodyweight;
    let mut data = model.make_data();
    data.qpos[2] = r - 0.0005; // 0.5 mm penetration
    data.qvel[0] = 1.5; // tangential slide
    data.qvel[2] = -1.0; // approaching
    data.qvel[4] = 10.0; // spin about +y
    data.forward(&model).expect("forward");
    data
}

/// Indices of the elliptic contact's rows: (normal, [friction...]).
fn elliptic_rows(data: &sim_core::Data) -> (usize, Vec<usize>) {
    let normal = data
        .efc_type
        .iter()
        .position(|t| *t == ConstraintType::ContactElliptic)
        .expect("expected an elliptic contact row");
    let dim = data.efc_dim[normal];
    let friction = (normal + 1..normal + dim).collect();
    (normal, friction)
}

/// The invariant that regressed: every condim row shares the normal row's R and D.
/// Must hold in both diagApprox modes (exact `M⁻¹` and DT-39 invweight).
#[test]
fn elliptic_friction_rows_share_normal_regularization() {
    for bodyweight in [false, true] {
        let data = frozen_contact_data(bodyweight);
        let (normal, friction) = elliptic_rows(&data);
        assert!(!friction.is_empty(), "expected friction rows for condim 3");

        let r_n = data.efc_R[normal];
        let d_n = data.efc_D[normal];
        for &fr in &friction {
            assert!(
                (data.efc_R[fr] - r_n).abs() <= 1e-12 * r_n.max(1.0),
                "bodyweight={bodyweight}: friction row {fr} R = {} must equal normal \
                 row R = {r_n} (pre-fix bug: friction R was ~10³× larger from \
                 tangential diagApprox)",
                data.efc_R[fr],
            );
            assert!(
                (data.efc_D[fr] - d_n).abs() <= 1e-12 * d_n.max(1.0),
                "bodyweight={bodyweight}: friction row {fr} D = {} must equal normal \
                 row D = {d_n}",
                data.efc_D[fr],
            );
        }
    }
}

/// Pin the MuJoCo-conformant numbers for this frozen state. Independently
/// verified against stock MuJoCo (`mj_forward`): R = 1.4225 on all rows and the
/// solved tangential friction force = 53.9187 N. Pre-fix this force was ~0.87 N.
/// MuJoCo's default diagApprox is the invweight approximation, so both of our
/// modes must reproduce these figures.
#[test]
fn elliptic_friction_force_matches_mujoco() {
    for bodyweight in [false, true] {
        let data = frozen_contact_data(bodyweight);
        let (normal, friction) = elliptic_rows(&data);

        // Normal-row regularization (MuJoCo reference value for this geometry).
        assert!(
            (data.efc_R[normal] - 1.4225).abs() < 1e-3,
            "bodyweight={bodyweight}: normal R = {}, expected MuJoCo 1.4225",
            data.efc_R[normal],
        );

        // The sliding direction carries the grip; its force must match MuJoCo, not
        // the ~62× weaker starved value the bug produced.
        let max_friction_force = friction
            .iter()
            .map(|&fr| data.efc_force[fr].abs())
            .fold(0.0_f64, f64::max);
        assert!(
            (max_friction_force - 53.9187).abs() < 0.05,
            "bodyweight={bodyweight}: tangential friction force = {max_friction_force}, \
             expected MuJoCo 53.9187 (pre-fix bug produced ~0.87 — starved grip)",
        );
    }
}
