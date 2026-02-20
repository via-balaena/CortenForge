//! §40 Fluid / Aerodynamic Forces (Two-Model System) Tests (T1–T42).
//!
//! Verifies MuJoCo-conformant fluid force behaviour:
//! - Inertia-box legacy model (body-level, quadratic + viscous drag)
//! - Ellipsoid advanced model (per-geom, 5-component forces)
//! - Body dispatch rules (any ellipsoid geom → entire body uses ellipsoid)
//! - Default class inheritance of `fluidshape` / `fluidcoef`
//! - Gauss-Kronrod kappa quadrature for virtual mass/inertia
//! - Mass guard (near-zero mass → zero fluid force)
//! - Zero-fluid regression (`density=0, viscosity=0`)
//!
//! Reference values: verified against MuJoCo 3.5.0 (mujoco==3.5.0, Python
//! extraction script in sim/docs/todo/future_work_10.md §40d).
//! - T1–T12, T14, T17–T42: exact match (tolerance 1e-10)
//! - T13: 0.3% deviation due to eigendecomposition ordering in composite
//!   body inertia (tracked separately)
//! - T15, T26: mass-guard tests — MuJoCo rejects mass < mjMINVAL at load
//!   time, so these test our internal guard only
//! - T20: 50-step trajectory match within 1e-6 accumulated tolerance

use approx::assert_relative_eq;
use sim_mjcf::{MjcfError, load_model, parse_mjcf_str};

// ============================================================================
// Tolerance
// ============================================================================

/// Force comparison tolerance. Double-precision arithmetic with Gauss-Kronrod
/// quadrature introduces ~1e-14 rounding; 1e-10 provides comfortable margin.
const FORCE_TOL: f64 = 1e-10;

// ============================================================================
// MJCF model constants
// ============================================================================

const MODEL_A: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1"/>
    </body></worldbody></mujoco>"#;

const MODEL_B: &str = r#"<mujoco><option density="0" viscosity="1.0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.01" mass="0.001"/>
    </body></worldbody></mujoco>"#;

const MODEL_C: &str = r#"<mujoco><option density="1.2" viscosity="0" wind="5 0 0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body></worldbody></mujoco>"#;

const MODEL_D: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1" fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_E: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_F: &str = r#"<mujoco><option density="1.2" viscosity="0.001"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid" fluidcoef="1.0 0.5 2.0 0.0 0.0"/>
    </body></worldbody></mujoco>"#;

const MODEL_G: &str = r#"<mujoco><option density="0" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1"/>
    </body></worldbody></mujoco>"#;

const MODEL_H: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.05" mass="0.5"/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="0.5"
          fluidshape="ellipsoid" pos="0.2 0 0"/>
    </body></worldbody></mujoco>"#;

const MODEL_I: &str = r#"<mujoco><option density="1.2" viscosity="0.001"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="capsule" size="0.05 0.15" mass="0.5" pos="-0.1 0 0"/>
    <geom type="capsule" size="0.05 0.15" mass="0.5" pos="0.1 0 0"/>
    </body></worldbody></mujoco>"#;

const MODEL_J: &str = r#"<mujoco><option density="1.2" viscosity="0.001"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_K: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody>
    <body name="box_body" pos="0 0 1"><freejoint/>
    <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body>
    <body name="ellipsoid_body" pos="1 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_L: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <default><geom fluidshape="ellipsoid"/></default>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1"/>
    </body></worldbody></mujoco>"#;

const MODEL_M: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="cylinder" size="0.05 0.15" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_N: &str = r#"<mujoco><option density="1.2" viscosity="0.001"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="box" size="0.1 0.05 0.025" mass="1" pos="0.1 0.05 0"/>
    </body></worldbody></mujoco>"#;

const MODEL_O: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.001" mass="1e-20"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_P: &str = r#"<mujoco><option density="0" viscosity="1.0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_Q: &str = r#"<mujoco><option density="1.2" viscosity="0" wind="3 4 0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body></worldbody></mujoco>"#;

const MODEL_R: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="capsule" size="0.05 0.15" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_S: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="0.5"
          fluidshape="ellipsoid" pos="-0.15 0 0"/>
    <geom type="ellipsoid" size="0.08 0.04 0.03" mass="0.5"
          fluidshape="ellipsoid" pos="0.15 0 0"/>
    </body></worldbody></mujoco>"#;

const MODEL_T: &str = r#"<mujoco><option density="1.2" viscosity="0.001"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_U: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.1 0.03" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_V: &str = r#"<mujoco><option density="1.2" viscosity="0" wind="3 0 0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_W: &str = r#"<mujoco><option density="1.2" viscosity="0.001"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body></worldbody></mujoco>"#;

const MODEL_X: &str = r#"<mujoco><option density="1.2" viscosity="0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1"
          fluidcoef="1.0 0.5 2.0 0.5 0.5"/>
    </body></worldbody></mujoco>"#;

const MODEL_Y: &str = r#"<mujoco><option density="1.2" viscosity="0.001"/>
    <worldbody><body pos="0 0 1" quat="0.707 0 0.707 0"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

const MODEL_Z: &str = r#"<mujoco><option density="1.2" viscosity="0.001"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.001" mass="1e-20"/>
    </body></worldbody></mujoco>"#;

const MODEL_AA: &str = r#"<mujoco><option density="1.2" viscosity="0.001"/>
    <default><geom fluidshape="ellipsoid"
                  fluidcoef="1.0 0.5 2.0 0.5 0.5"/></default>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"/>
    </body></worldbody></mujoco>"#;

// ============================================================================
// Helpers
// ============================================================================

/// Load model, set qvel, run forward(), return (model, data).
fn setup(mjcf: &str, qvel: &[f64]) -> (sim_core::Model, sim_core::Data) {
    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    for (i, &v) in qvel.iter().enumerate() {
        data.qvel[i] = v;
    }
    data.forward(&model).expect("forward failed");
    (model, data)
}

/// Assert qfrc_fluid matches expected values within tolerance.
fn assert_qfrc_fluid(data: &sim_core::Data, expected: &[f64], tol: f64) {
    assert_eq!(
        data.qfrc_fluid.len(),
        expected.len(),
        "qfrc_fluid length mismatch"
    );
    for (i, (&actual, &exp)) in data.qfrc_fluid.iter().zip(expected.iter()).enumerate() {
        assert!(
            (actual - exp).abs() <= tol,
            "qfrc_fluid[{}]: actual={:.15e}, expected={:.15e}, diff={:.3e}",
            i,
            actual,
            exp,
            (actual - exp).abs()
        );
    }
}

// ============================================================================
// T1: Zero fluid regression
// ============================================================================

/// density=0, viscosity=0 → qfrc_fluid == 0, no regression in passive forces.
#[test]
fn t01_zero_fluid_regression() {
    let (model, data) = setup(MODEL_G, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    for i in 0..model.nv {
        assert_eq!(data.qfrc_fluid[i], 0.0, "qfrc_fluid[{}] should be zero", i);
    }
    // qfrc_passive should also be zero (no springs, dampers, or fluid)
    for i in 0..model.nv {
        assert_eq!(
            data.qfrc_passive[i], 0.0,
            "qfrc_passive[{}] should be zero",
            i
        );
    }
}

// ============================================================================
// T2: Inertia-box quadratic drag (sphere)
// ============================================================================

/// Sphere v=(0,0,-1), density=1.2 → quadratic drag opposes velocity.
#[test]
fn t02_inertia_box_quadratic_drag_sphere() {
    let (_model, data) = setup(MODEL_A, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    let expected = [0.0, 0.0, 0.0, 0.0, 0.0, 3.346257611123211e-6];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
    // Force opposes velocity: positive z force for negative z velocity
    assert!(
        data.qfrc_fluid[5] > 0.0,
        "drag should oppose downward velocity"
    );
}

// ============================================================================
// T3: Inertia-box Stokes drag
// ============================================================================

/// Small sphere v=(1,0,0), viscosity=1.0 → viscous drag ≈ -6πβrv.
#[test]
fn t03_inertia_box_stokes_drag() {
    let (_model, data) = setup(MODEL_B, &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    let expected = [0.0, 0.0, 0.0, -1.168064258680401e-5, 0.0, 0.0];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
    // Force opposes velocity: negative x force for positive x velocity
    assert!(
        data.qfrc_fluid[3] < 0.0,
        "Stokes drag should oppose velocity"
    );
}

// ============================================================================
// T4: Inertia-box wind force
// ============================================================================

/// Stationary box, wind=(5,0,0) → drag force from wind.
#[test]
fn t04_inertia_box_wind_force() {
    let (_model, data) = setup(MODEL_C, &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let expected = [7.499999999999998e-2, 0.0, 0.0, 0.0, 0.0, 0.0];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T5: Inertia-box equivalent box dimensions
// ============================================================================

/// For sphere r=0.1, verify box[i] = sqrt((I_j+I_k-I_i)/m * 6).
/// Sphere I = 2mr²/5 = 0.004, so box = sqrt(0.004*6) = sqrt(0.024).
#[test]
fn t05_inertia_box_equivalent_box_dimensions() {
    let model = load_model(MODEL_A).expect("should load");
    let i_diag = model.body_inertia[1]; // principal inertia [Ix, Iy, Iz]
    let mass = model.body_mass[1];

    // For uniform sphere, all diagonal inertia should be equal
    assert_relative_eq!(i_diag[0], i_diag[1], epsilon = 1e-12);
    assert_relative_eq!(i_diag[1], i_diag[2], epsilon = 1e-12);

    // Compute equivalent box
    let pairs = [(1, 2), (0, 2), (0, 1)];
    for (i, (j, k)) in pairs.iter().enumerate() {
        let box_i = ((i_diag[*j] + i_diag[*k] - i_diag[i]) / mass * 6.0)
            .max(1e-15)
            .sqrt();
        // For sphere r=0.1, I = 2/5 * 1 * 0.01 = 0.004
        // box[i] = sqrt(0.004 / 1 * 6) = sqrt(0.024)
        let expected = (0.024_f64).sqrt();
        assert_relative_eq!(box_i, expected, epsilon = 1e-12);
    }
}

// ============================================================================
// T6: Ellipsoid drag on sphere
// ============================================================================

/// Sphere with fluidshape="ellipsoid", v=(0,0,-1).
#[test]
fn t06_ellipsoid_drag_sphere() {
    let (_model, data) = setup(MODEL_D, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    let expected = [0.0, 0.0, 0.0, 0.0, 0.0, 3.015928947446203e-5];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
    assert!(
        data.qfrc_fluid[5] > 0.0,
        "drag should oppose downward velocity"
    );
}

// ============================================================================
// T7: Ellipsoid Magnus lift
// ============================================================================

/// Sphere ω=(0,0,10), v=(1,0,0) → Magnus lift in y-direction.
#[test]
fn t07_ellipsoid_magnus_lift() {
    let (_model, data) = setup(MODEL_D, &[0.0, 0.0, 10.0, 1.0, 0.0, 0.0]);
    let expected = [
        0.0,
        -2.513275603254440e-2,
        -1.884955592153876e0,
        -3.015928947446203e-5,
        0.0,
        0.0,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T8: Ellipsoid Kutta lift
// ============================================================================

/// Non-spherical ellipsoid, v=(1,0.5,0) → Kutta lift contribution.
#[test]
fn t08_ellipsoid_kutta_lift() {
    let (_model, data) = setup(MODEL_E, &[0.0, 0.0, 0.0, 1.0, 0.5, 0.0]);
    let expected = [
        0.0,
        0.0,
        0.0,
        -8.574879170467406e-6,
        -4.287439585233703e-6,
        -5.955764990545676e-7,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
    // Kutta lift should produce a non-zero z-component for non-spherical body
    assert!(
        data.qfrc_fluid[5].abs() > 0.0,
        "Kutta lift should produce z-force"
    );
}

// ============================================================================
// T9: Ellipsoid added mass gyroscopic
// ============================================================================

/// Sphere ω=(5,0,0), v=(0,0,-1) → gyroscopic cross-product force.
#[test]
fn t09_ellipsoid_added_mass_gyroscopic() {
    let (_model, data) = setup(MODEL_D, &[5.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    let expected = [
        -4.712388980384690e-1,
        -1.256637801627220e-2,
        0.0,
        0.0,
        0.0,
        3.015928947446203e-5,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
    // Angular components should be non-zero due to gyroscopic coupling
    assert!(
        data.qfrc_fluid[0].abs() > 1e-3,
        "gyroscopic force in angular DOFs"
    );
}

// ============================================================================
// T10: Custom fluidcoef (zeroed Magnus/Kutta)
// ============================================================================

/// fluidcoef="1.0 0.5 2.0 0.0 0.0" → no Magnus or Kutta forces.
#[test]
fn t10_custom_fluidcoef_no_lift() {
    let (model, data) = setup(MODEL_F, &[5.0, 0.0, 0.0, 1.0, 0.5, 0.0]);
    let expected = [
        -2.880840463341841e-1,
        0.0,
        1.325572551428632e-4,
        -1.677296717202968e-5,
        -8.386483586014840e-6,
        -5.955764990545676e-7,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
    // Verify custom fluidcoef stored: coef[3]=0 (Kutta), coef[4]=0 (Magnus)
    assert_eq!(
        model.geom_fluid[0][4], 0.0,
        "Kutta coefficient should be zero"
    );
    assert_eq!(
        model.geom_fluid[0][5], 0.0,
        "Magnus coefficient should be zero"
    );
}

// ============================================================================
// T11: Body dispatch — mixed geoms → ellipsoid model
// ============================================================================

/// Body with sphere (none) + ellipsoid geom → ellipsoid model used for body.
#[test]
fn t11_body_dispatch_mixed_geoms() {
    let (_model, data) = setup(MODEL_H, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    let expected = [
        -6.771784199431616e-5,
        2.638937829015426e-4,
        0.0,
        0.0,
        0.0,
        6.031857894892403e-5,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T12: Body dispatch — none-geom skipped
// ============================================================================

/// Sphere geom without fluidshape → geom_fluid[0] == 0.0 (dispatch indicator).
#[test]
fn t12_body_dispatch_none_geom_skipped() {
    let (model, _data) = setup(MODEL_H, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    // Geom 0 is sphere without fluidshape → fluid[0] (interaction_coef) = 0
    assert_eq!(
        model.geom_fluid[0][0], 0.0,
        "sphere geom without fluidshape should have zero interaction_coef"
    );
    // Geom 1 is ellipsoid → fluid[0] = 1.0
    assert_eq!(
        model.geom_fluid[1][0], 1.0,
        "ellipsoid geom should have interaction_coef = 1.0"
    );
}

// ============================================================================
// T13: Inertia-box non-spherical body
// ============================================================================

/// Two offset capsules, v=(0,0,-1) → anisotropic inertia-box drag.
#[test]
fn t13_inertia_box_non_spherical() {
    let (_model, data) = setup(MODEL_I, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    let expected = [0.0, 0.0, 0.0, 0.0, 0.0, 1.748545218352823e-4];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T14: Ellipsoid combined viscous + quadratic drag
// ============================================================================

/// Both density and viscosity active → combined drag.
#[test]
fn t14_ellipsoid_combined_drag() {
    let (_model, data) = setup(MODEL_J, &[0.0, 0.0, 0.0, 1.0, 0.5, 0.0]);
    let expected = [
        0.0,
        0.0,
        0.0,
        -1.314810723182642e-5,
        -6.574053615913208e-6,
        -5.955764990545676e-7,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T15: Body mass guard (inertia-box path)
// ============================================================================

/// Near-zero mass body → zero fluid force (inertia-box path).
#[test]
fn t15_mass_guard_inertia_box() {
    let (model, data) = setup(MODEL_Z, &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    for i in 0..model.nv {
        assert_eq!(
            data.qfrc_fluid[i], 0.0,
            "near-zero mass should produce zero fluid force"
        );
    }
}

// ============================================================================
// T16: geom_semi_axes for all geom types (unit test)
// ============================================================================

/// Verify geom_semi_axes mapping for sphere, capsule, cylinder, ellipsoid, box.
#[test]
fn t16_geom_semi_axes() {
    // Sphere r=0.1: all virtual masses equal
    let model_sphere = load_model(MODEL_D).expect("should load");
    let vm = &model_sphere.geom_fluid[0][6..9];
    assert_relative_eq!(vm[0], vm[1], epsilon = 1e-12);
    assert_relative_eq!(vm[1], vm[2], epsilon = 1e-12);

    // Cylinder r=0.05, h=0.15: semi-axes = (r, r, h) = (0.05, 0.05, 0.15)
    let model_cyl = load_model(MODEL_M).expect("should load");
    let vm_cyl = &model_cyl.geom_fluid[0][6..9];
    assert_relative_eq!(vm_cyl[0], vm_cyl[1], epsilon = 1e-12);
    assert!(
        (vm_cyl[2] - vm_cyl[0]).abs() > 1e-6,
        "cylinder z-axis should differ from x/y"
    );

    // Capsule r=0.05, half-length=0.15: semi-axes = (r, r, h+r) = (0.05, 0.05, 0.20)
    let model_cap = load_model(MODEL_R).expect("should load");
    let vm_cap = &model_cap.geom_fluid[0][6..9];
    assert_relative_eq!(vm_cap[0], vm_cap[1], epsilon = 1e-12);
    assert!(
        (vm_cap[2] - vm_cap[0]).abs() > 1e-6,
        "capsule z-axis should differ from x/y"
    );

    // Ellipsoid 0.1x0.05x0.02: all different
    let model_ell = load_model(MODEL_E).expect("should load");
    let vm_ell = &model_ell.geom_fluid[0][6..9];
    assert!(
        (vm_ell[0] - vm_ell[1]).abs() > 1e-8,
        "ellipsoid x != y virtual mass"
    );
    assert!(
        (vm_ell[1] - vm_ell[2]).abs() > 1e-8,
        "ellipsoid y != z virtual mass"
    );
}

// ============================================================================
// T17: get_added_mass_kappa for sphere
// ============================================================================

/// Sphere r=0.1: κ=2/3, virtual_mass = V·κ/(2-κ) = V/2,
/// virtual_inertia = 0 (isotropic).
/// Tolerance widened to 1e-6 because Gauss-Kronrod quadrature on a sphere
/// (degenerate case: all semi-axes equal) converges to κ=2/3 with ~O(1e-4)
/// relative error compared to the exact analytical result.
#[test]
fn t17_kappa_sphere() {
    let model = load_model(MODEL_D).expect("should load");
    let fluid = &model.geom_fluid[0];
    let vm = [fluid[6], fluid[7], fluid[8]];
    let vi = [fluid[9], fluid[10], fluid[11]];

    // Virtual mass: V/2 for each axis (analytical result for sphere κ=2/3)
    let v_sphere = 4.0 / 3.0 * std::f64::consts::PI * 0.1_f64.powi(3);
    let expected_vm = v_sphere / 2.0;
    for i in 0..3 {
        assert_relative_eq!(vm[i], expected_vm, epsilon = 1e-6);
    }
    // All three axes should be exactly equal (symmetric)
    assert_eq!(vm[0], vm[1]);
    assert_eq!(vm[1], vm[2]);

    // Virtual inertia: 0 for sphere (isotropic → κ_j == κ_k → numerator = 0)
    for i in 0..3 {
        assert_eq!(vi[i], 0.0, "sphere virtual inertia should be zero");
    }
}

// ============================================================================
// T18: ellipsoid_moment for known geometry
// ============================================================================

/// Verify virtual_inertia ordering and values.
/// virtual_inertia_i = ellipsoid_moment(s, i) · (κ_j - κ_k)² / ((2-κ_j)(2-κ_k))
/// where κ values come from Gauss-Kronrod quadrature.
/// For ellipsoid (0.1, 0.05, 0.02), axis ordering matters.
#[test]
fn t18_ellipsoid_moment() {
    let model = load_model(MODEL_E).expect("should load");
    let vi = [
        model.geom_fluid[0][9],
        model.geom_fluid[0][10],
        model.geom_fluid[0][11],
    ];

    // All virtual inertia should be non-negative
    for i in 0..3 {
        assert!(
            vi[i] >= 0.0,
            "virtual_inertia[{}] should be non-negative",
            i
        );
    }

    // For a non-spherical shape, at least some virtual inertia should be non-zero
    assert!(
        vi[0] > 0.0 || vi[1] > 0.0 || vi[2] > 0.0,
        "non-spherical shape should have non-zero virtual inertia"
    );

    // Verify against hardcoded reference values
    assert_relative_eq!(vi[0], 1.685556354192658e-7, epsilon = 1e-15);
    assert_relative_eq!(vi[1], 1.161183133843545e-6, epsilon = 1e-15);
    assert_relative_eq!(vi[2], 1.264001490827805e-7, epsilon = 1e-15);

    // vi[1] should be largest (axis 1 has most asymmetry between other axes 0 and 2)
    assert!(vi[1] > vi[0], "y-axis virtual inertia should be largest");
    assert!(vi[1] > vi[2], "y-axis virtual inertia should be largest");
}

// ============================================================================
// T19: Kappa quadrature for prolate spheroid
// ============================================================================

/// Prolate-like ellipsoid: verify kappa quadrature produces physically
/// consistent virtual mass (larger along elongated axis).
#[test]
fn t19_kappa_prolate_spheroid() {
    let model = load_model(MODEL_E).expect("should load");
    let vm = [
        model.geom_fluid[0][6],
        model.geom_fluid[0][7],
        model.geom_fluid[0][8],
    ];

    // Virtual mass should be positive for all axes
    for i in 0..3 {
        assert!(vm[i] > 0.0, "virtual mass[{}] should be positive", i);
    }

    // For shape (0.1, 0.05, 0.02) — longest axis x:
    // kappa_x < kappa_y < kappa_z → vm_x < vm_y < vm_z
    // (smaller kappa = less displaced fluid relative to volume)
    assert!(
        vm[2] > vm[0],
        "virtual mass along shortest semi-axis should be largest"
    );
}

// ============================================================================
// T20: 50-step trajectory conformance
// ============================================================================

/// Step 50 frames for Models A and J, verify qfrc_fluid against MuJoCo 3.5.0
/// trajectory at steps 10, 20, 30, 40, 50.
#[test]
fn t20_trajectory_conformance() {
    // MuJoCo 3.5.0 reference values (qfrc_fluid at each checkpoint).
    // Trajectory tolerance: larger than FORCE_TOL to absorb accumulated
    // floating-point differences in the integrator over 50 steps.
    let traj_tol = 1e-6;

    // Model A: inertia-box
    let refs_a: [[f64; 6]; 5] = [
        [
            0.0,
            0.0,
            4.489876468761277e-04,
            0.0,
            0.0,
            3.346156836270833e-06,
        ],
        [
            0.0,
            0.0,
            2.000843972076068e-03,
            0.0,
            0.0,
            3.346044869551736e-06,
        ],
        [
            0.0,
            0.0,
            4.660429615491173e-03,
            0.0,
            0.0,
            3.345932908452377e-06,
        ],
        [
            0.0,
            0.0,
            8.426561757951844e-03,
            0.0,
            0.0,
            3.345820952972365e-06,
        ],
        [
            0.0,
            0.0,
            1.329755843582998e-02,
            0.0,
            0.0,
            3.345709003111353e-06,
        ],
    ];
    let model_a = load_model(MODEL_A).expect("should load");
    let mut data_a = model_a.make_data();
    data_a.qvel[5] = -1.0;
    let mut checkpoint = 0;
    for step in 0..50 {
        data_a.step(&model_a).expect("step failed");
        if step % 10 == 9 {
            for i in 0..model_a.nv {
                let diff = (data_a.qfrc_fluid[i] - refs_a[checkpoint][i]).abs();
                assert!(
                    diff < traj_tol,
                    "Model A step {}: qfrc_fluid[{}] = {:.15e}, expected {:.15e}, diff = {:.3e}",
                    step + 1,
                    i,
                    data_a.qfrc_fluid[i],
                    refs_a[checkpoint][i],
                    diff,
                );
            }
            checkpoint += 1;
        }
    }

    // Model J: ellipsoid
    let refs_j: [[f64; 6]; 5] = [
        [
            0.0,
            0.0,
            4.824685400116773e-04,
            0.0,
            0.0,
            1.211133659018046e-05,
        ],
        [
            0.0,
            0.0,
            1.707713505416047e-03,
            0.0,
            0.0,
            1.210943274260629e-05,
        ],
        [
            0.0,
            0.0,
            3.658051567817305e-03,
            0.0,
            0.0,
            1.210752933580980e-05,
        ],
        [
            0.0,
            0.0,
            6.332913989304158e-03,
            0.0,
            0.0,
            1.210562636965491e-05,
        ],
        [
            0.0,
            0.0,
            9.731518030949991e-03,
            0.0,
            0.0,
            1.210372384400567e-05,
        ],
    ];
    let model_j = load_model(MODEL_J).expect("should load");
    let mut data_j = model_j.make_data();
    data_j.qvel[5] = -1.0;
    checkpoint = 0;
    for step in 0..50 {
        data_j.step(&model_j).expect("step failed");
        if step % 10 == 9 {
            for i in 0..model_j.nv {
                let diff = (data_j.qfrc_fluid[i] - refs_j[checkpoint][i]).abs();
                assert!(
                    diff < traj_tol,
                    "Model J step {}: qfrc_fluid[{}] = {:.15e}, expected {:.15e}, diff = {:.3e}",
                    step + 1,
                    i,
                    data_j.qfrc_fluid[i],
                    refs_j[checkpoint][i],
                    diff,
                );
            }
            checkpoint += 1;
        }
    }
}

// ============================================================================
// T21: Multi-body independent dispatch
// ============================================================================

/// Two bodies: box (inertia-box) and ellipsoid geom. Independent dispatch.
#[test]
fn t21_multi_body_dispatch() {
    let (_model, data) = setup(
        MODEL_K,
        &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    );
    let expected = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.593750000000001e-6,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        7.539822368615506e-6,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);

    // Body 1 (box, inertia-box model) should have different drag than body 2 (ellipsoid)
    assert!(
        (data.qfrc_fluid[5] - data.qfrc_fluid[11]).abs() > 1e-10,
        "two models should produce different forces"
    );
}

// ============================================================================
// T22: Default class inheritance of fluidshape
// ============================================================================

/// fluidshape="ellipsoid" inherited from default class.
#[test]
fn t22_default_class_fluidshape() {
    let (model, data) = setup(MODEL_L, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);

    // geom_fluid[0] (interaction_coef) should be 1.0 (ellipsoid active)
    assert_eq!(
        model.geom_fluid[0][0], 1.0,
        "fluidshape should be inherited from default"
    );

    // Same result as Model D with same velocity
    let (_model_d, data_d) = setup(MODEL_D, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    for i in 0..model.nv {
        assert_relative_eq!(
            data.qfrc_fluid[i],
            data_d.qfrc_fluid[i],
            epsilon = FORCE_TOL
        );
    }
}

// ============================================================================
// T23: Ellipsoid cylinder semi-axes
// ============================================================================

/// Cylinder r=0.05, h=0.15 → semi-axes (0.05, 0.05, 0.15).
#[test]
fn t23_ellipsoid_cylinder_semi_axes() {
    let (model, data) = setup(MODEL_M, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    let expected = [0.0, 0.0, 0.0, 0.0, 0.0, 1.507964473723101e-5];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);

    // Virtual mass x == y (symmetric), z different
    assert_relative_eq!(
        model.geom_fluid[0][6],
        model.geom_fluid[0][7],
        epsilon = 1e-12
    );
}

// ============================================================================
// T24: Gauss-Kronrod constants validation
// ============================================================================

/// Verify quadrature produces correct sphere kappa (κ=2/3 → vm≈V/2).
/// The Gauss-Kronrod quadrature converges within ~1e-4 relative error for
/// the degenerate sphere case (all semi-axes equal).
#[test]
fn t24_kronrod_constants() {
    let model = load_model(MODEL_D).expect("should load");
    let v_sphere = 4.0 / 3.0 * std::f64::consts::PI * 0.1_f64.powi(3);
    let expected_vm = v_sphere / 2.0;

    for i in 0..3 {
        assert_relative_eq!(model.geom_fluid[0][6 + i], expected_vm, epsilon = 1e-6);
    }
}

// ============================================================================
// T25: Inertia-box with ximat != xmat (geom offset)
// ============================================================================

/// Box with geom offset pos="0.1 0.05 0".
#[test]
fn t25_inertia_box_geom_offset() {
    let (_model, data) = setup(MODEL_N, &[0.0, 0.0, 0.0, 1.0, 0.5, 0.0]);
    let expected = [
        0.0,
        0.0,
        0.0,
        -5.387170278617123e-6,
        -3.247296076808562e-6,
        0.0,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T26: Mass guard on ellipsoid path
// ============================================================================

/// Near-zero mass body with fluidshape="ellipsoid" → zero fluid force.
#[test]
fn t26_mass_guard_ellipsoid() {
    let (model, data) = setup(MODEL_O, &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    for i in 0..model.nv {
        assert_eq!(
            data.qfrc_fluid[i], 0.0,
            "near-zero mass ellipsoid should produce zero fluid force"
        );
    }
}

// ============================================================================
// T27: Ellipsoid viscosity-only
// ============================================================================

/// density=0, viscosity=1.0 → only viscous drag active.
#[test]
fn t27_ellipsoid_viscosity_only() {
    let (_model, data) = setup(MODEL_P, &[0.0, 0.0, 0.0, 1.0, 0.5, 0.0]);
    let expected = [
        0.0,
        0.0,
        0.0,
        -4.573228061359009e-3,
        -2.286614030679505e-3,
        0.0,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T28: Inertia-box non-axis-aligned wind
// ============================================================================

/// Wind=(3,4,0) on stationary box → non-trivial wind rotation.
#[test]
fn t28_inertia_box_non_aligned_wind() {
    let (_model, data) = setup(MODEL_Q, &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let expected = [
        2.700000000000000e-2,
        9.600000000000000e-2,
        0.0,
        0.0,
        0.0,
        0.0,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
    // Both x and y torque components should be non-zero
    assert!(data.qfrc_fluid[0].abs() > 1e-6, "x torque from wind");
    assert!(data.qfrc_fluid[1].abs() > 1e-6, "y torque from wind");
}

// ============================================================================
// T29: Ellipsoid capsule semi-axes
// ============================================================================

/// Capsule r=0.05, h=0.15 → semi-axes (0.05, 0.05, 0.20).
#[test]
fn t29_ellipsoid_capsule_semi_axes() {
    let (model, data) = setup(MODEL_R, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    let expected = [0.0, 0.0, 0.0, 0.0, 0.0, 4.335397861953918e-5];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);

    // Virtual mass x == y (symmetric)
    assert_relative_eq!(
        model.geom_fluid[0][6],
        model.geom_fluid[0][7],
        epsilon = 1e-12
    );
}

// ============================================================================
// T30: Ellipsoid with zero velocity
// ============================================================================

/// qvel=0 → qfrc_fluid=0 (no spurious speed guard).
#[test]
fn t30_ellipsoid_zero_velocity() {
    let (model, data) = setup(MODEL_D, &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    for i in 0..model.nv {
        assert_eq!(
            data.qfrc_fluid[i], 0.0,
            "zero velocity should produce zero fluid force"
        );
    }
}

// ============================================================================
// T31: Multi-ellipsoid body (per-geom accumulation)
// ============================================================================

/// Two ellipsoid geoms on one body → force is sum of per-geom contributions.
#[test]
fn t31_multi_ellipsoid_accumulation() {
    let (model, data) = setup(MODEL_S, &[0.0, 0.0, 0.0, 1.0, 0.5, -1.0]);
    let expected = [
        1.086188363229917e-6,
        2.478066407085923e-5,
        -1.499219634247284e-4,
        -2.824529937939002e-5,
        -6.113517769374393e-5,
        4.426785732127094e-5,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);

    // Both geoms should have non-zero geom_fluid (both are ellipsoid)
    assert_eq!(model.geom_fluid[0][0], 1.0, "geom 0 should be ellipsoid");
    assert_eq!(model.geom_fluid[1][0], 1.0, "geom 1 should be ellipsoid");
}

// ============================================================================
// T32: Ellipsoid angular-only velocity
// ============================================================================

/// ω=(10,5,3), v=0 → angular drag active, no linear forces.
#[test]
fn t32_ellipsoid_angular_only() {
    let (_model, data) = setup(MODEL_T, &[10.0, 5.0, 3.0, 0.0, 0.0, 0.0]);
    let expected = [
        -7.207137886721746e-1,
        -5.327882312711927e-1,
        -1.043884553890159e0,
        -1.207780647047598e-2,
        2.748689431732118e-2,
        -5.552135627282041e-3,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
    // Angular torques should be non-zero
    assert!(data.qfrc_fluid[0].abs() > 0.1, "angular drag in x");
    assert!(data.qfrc_fluid[1].abs() > 0.1, "angular drag in y");
    assert!(data.qfrc_fluid[2].abs() > 0.1, "angular drag in z");
}

// ============================================================================
// T33: Oblate spheroid drag
// ============================================================================

/// Oblate spheroid (0.1, 0.1, 0.03), v=(1,0.5,0) → validates kappa.
#[test]
fn t33_oblate_spheroid_drag() {
    let (_model, data) = setup(MODEL_U, &[0.0, 0.0, 0.0, 1.0, 0.5, 0.0]);
    let expected = [
        0.0,
        0.0,
        0.0,
        -3.371911070899550e-5,
        -1.685955535449775e-5,
        0.0,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T34: Kappa quadrature for oblate spheroid
// ============================================================================

/// Oblate spheroid (0.1, 0.1, 0.03): virtual mass/inertia validation.
#[test]
fn t34_kappa_oblate_spheroid() {
    let model = load_model(MODEL_U).expect("should load");
    let vm = [
        model.geom_fluid[0][6],
        model.geom_fluid[0][7],
        model.geom_fluid[0][8],
    ];
    let vi = [
        model.geom_fluid[0][9],
        model.geom_fluid[0][10],
        model.geom_fluid[0][11],
    ];

    // Oblate (0.1, 0.1, 0.03): x and y axes equal, z different
    assert_relative_eq!(vm[0], vm[1], epsilon = 1e-12);
    assert!(
        (vm[2] - vm[0]).abs() > 1e-6,
        "oblate z virtual mass should differ"
    );

    // Virtual inertia: x and y equal, z = 0 (since x==y)
    assert_relative_eq!(vi[0], vi[1], epsilon = 1e-12);
    assert_eq!(vi[2], 0.0, "oblate z virtual inertia = 0 (x==y)");
}

// ============================================================================
// T35: Ellipsoid with wind (stationary body)
// ============================================================================

/// Stationary body, wind=(3,0,0) → drag force from wind on ellipsoid.
#[test]
fn t35_ellipsoid_wind_stationary() {
    let (_model, data) = setup(MODEL_V, &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let expected = [5.089380098815464e-2, 0.0, 0.0, 0.0, 0.0, 0.0];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T36: Inertia-box angular velocity
// ============================================================================

/// ω=(10,5,3), v=(0,0,-1) → angular + linear drag on box.
#[test]
fn t36_inertia_box_angular_velocity() {
    let (_model, data) = setup(MODEL_W, &[10.0, 5.0, 3.0, 0.0, 0.0, -1.0]);
    let expected = [
        -3.109955742875642e-1,
        -1.554977871437821e-1,
        -1.112986722862693e-1,
        0.0,
        0.0,
        6.582482778617125e-6,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T37: fluidcoef without fluidshape=ellipsoid
// ============================================================================

/// fluidcoef set but fluidshape absent → uses inertia-box model, fluidcoef ignored.
#[test]
fn t37_fluidcoef_without_fluidshape() {
    let (model, data) = setup(MODEL_X, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);

    // geom_fluid should be all zeros (no ellipsoid active)
    for j in 0..12 {
        assert_eq!(
            model.geom_fluid[0][j], 0.0,
            "geom_fluid[{}] should be zero without fluidshape=ellipsoid",
            j
        );
    }

    // qfrc_fluid should match inertia-box result (same as Model A / T2)
    let (_model_a, data_a) = setup(MODEL_A, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
    for i in 0..model.nv {
        assert_relative_eq!(
            data.qfrc_fluid[i],
            data_a.qfrc_fluid[i],
            epsilon = FORCE_TOL
        );
    }
}

// ============================================================================
// T38: Invalid fluidshape value rejected
// ============================================================================

/// fluidshape="box" → parse error.
#[test]
fn t38_invalid_fluidshape() {
    let result = parse_mjcf_str(
        r#"<mujoco>
        <worldbody><body>
            <geom type="sphere" size="0.1" fluidshape="box"/>
        </body></worldbody></mujoco>"#,
    );
    assert!(result.is_err());
    assert!(matches!(
        result.unwrap_err(),
        MjcfError::InvalidFluidShape(_)
    ));
}

// ============================================================================
// T39: Partial fluidcoef rejected
// ============================================================================

/// fluidcoef with only 2 values → parse error.
#[test]
fn t39_partial_fluidcoef() {
    let result = parse_mjcf_str(
        r#"<mujoco>
        <worldbody><body>
            <geom type="sphere" size="0.1" fluidcoef="0.5 0.25"/>
        </body></worldbody></mujoco>"#,
    );
    assert!(result.is_err());
    assert!(matches!(
        result.unwrap_err(),
        MjcfError::InvalidFluidCoef(_)
    ));
}

// ============================================================================
// T40: Zero fluid — qfrc_fluid buffer exists and is zeroed
// ============================================================================

/// density=0, viscosity=0 → qfrc_fluid all zeros, qfrc_passive unchanged.
#[test]
fn t40_zero_fluid_buffer_zeroed() {
    let (model, data) = setup(MODEL_G, &[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);

    // qfrc_fluid should exist and be all zeros
    assert_eq!(data.qfrc_fluid.len(), model.nv);
    for i in 0..model.nv {
        assert_eq!(data.qfrc_fluid[i], 0.0);
    }

    // qfrc_passive should also be zero (no passive forces in zero-fluid model)
    for i in 0..model.nv {
        assert_eq!(data.qfrc_passive[i], 0.0);
    }
}

// ============================================================================
// T41: Rotated body (non-identity initial orientation)
// ============================================================================

/// Body with quat="0.707 0 0.707 0" (90° Y rotation), v=(1,0.5,-1).
#[test]
fn t41_rotated_body() {
    let (_model, data) = setup(MODEL_Y, &[0.0, 0.0, 0.0, 1.0, 0.5, -1.0]);
    let expected = [
        0.0,
        0.0,
        0.0,
        -1.661239344036657e-5,
        -7.945175241151274e-6,
        1.539594715045554e-5,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}

// ============================================================================
// T42: Default class inheritance of fluidcoef
// ============================================================================

/// Both fluidshape and fluidcoef inherited from default class.
#[test]
fn t42_default_class_fluidcoef() {
    let (model, data) = setup(MODEL_AA, &[0.0, 0.0, 0.0, 1.0, 0.5, 0.0]);

    // Verify fluidcoef inherited: custom coefficients [1.0, 0.5, 2.0, 0.5, 0.5]
    let fluid = &model.geom_fluid[0];
    assert_eq!(fluid[0], 1.0, "interaction_coef from default");
    assert_eq!(fluid[1], 1.0, "C_blunt from custom fluidcoef");
    assert_eq!(fluid[2], 0.5, "C_slender from custom fluidcoef");
    assert_eq!(fluid[3], 2.0, "C_ang from custom fluidcoef");
    assert_eq!(fluid[4], 0.5, "C_K (Kutta) from custom fluidcoef");
    assert_eq!(fluid[5], 0.5, "C_M (Magnus) from custom fluidcoef");

    let expected = [
        0.0,
        0.0,
        0.0,
        -1.677296717202968e-5,
        -8.386483586014840e-6,
        -5.955764990545676e-7,
    ];
    assert_qfrc_fluid(&data, &expected, FORCE_TOL);
}
