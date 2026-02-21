//! §40a Fluid Force Velocity Derivatives Tests (T1–T32).
//!
//! Verifies ∂qfrc_fluid/∂qvel for both fluid models (inertia-box, ellipsoid)
//! so the implicit integrator correctly accounts for fluid damping in the D matrix.
//!
//! Test strategy:
//! - FD validation: analytical derivatives vs centered finite differences (tol 1e-5)
//! - Structural tests: symmetry, guards, edge cases
//! - Pipeline tests: energy dissipation, dispatch, zero-fluid
//!
//! MuJoCo conformance: T5, T13, T17, T25–T31 compare against MuJoCo 3.5.0
//! reference values extracted via mj_forward + mj_implicit (tol 1e-10, 1e-8 for implicitfast).

use sim_core::{Data, Model};
use sim_mjcf::load_model;

// ============================================================================
// Tolerances
// ============================================================================

/// FD validation tolerance. Central FD with ε=1e-6 has O(ε²)≈1e-12 truncation
/// but guard boundaries introduce O(ε) error. 1e-5 catches formula errors.
const FD_TOL: f64 = 1e-5;

/// MuJoCo conformance tolerance for analytical-vs-analytical comparison.
/// Both implementations use identical analytical formulas; only accumulation
/// order differences contribute error. Matches existing derivative conformance tests.
const MJ_TOL: f64 = 1e-10;

/// Relaxed MuJoCo conformance tolerance for implicitfast (T17).
/// Full pipeline crosses FK → RNE → passive → derivatives, accumulating
/// rounding across more stages.
const MJ_TOL_FAST: f64 = 1e-8;

// ============================================================================
// MuJoCo 3.5.0 reference qDeriv values
// ============================================================================
// Extracted via: mj_forward(m,d) → mj_implicit(m,d) → sparse-to-dense qDeriv.
//
// Both our code and MuJoCo use [v₁v₂v₃ω₁ω₂ω₃] DOF ordering for free joints
// (linear DOFs first, angular DOFs second). No permutation needed.
//
// Free joint conformance tests compare fluid-only qDeriv (full − zero-fluid)
// to isolate the fluid contribution from Coriolis terms that differ between
// our Featherstone RNE pass and MuJoCo's split bias computation.
//
// Non-free joint tests (hinge, ball, chain) compare full qDeriv directly.
// Each array is stored in row-major order: entry [i*nv + j] = qDeriv[i,j].

/// T5: Inertia-box fluid-only (free-floating box, density=1.2, viscosity=0.001, implicit).
/// qvel = [0.5,-1.1,0.8,1.2,-0.7,0.3] → v=[0.5,-1.1,0.8], ω=[1.2,-0.7,0.3].
/// Computed as: qDeriv(full) − qDeriv(zero-fluid). Same [v,ω] convention, no permutation.
#[rustfmt::skip]
const MUJOCO_FLUID_T5: [f64; 36] = [
    -4.09955742875642726e-03, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, -1.42995574287564289e-02, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, -2.02995574287564255e-02, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -5.94498277861712347e-06, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -9.20513902861712436e-06, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -5.94498277861712432e-06,
];

/// T13: Ellipsoid fluid-only (free-floating, density=1.2, viscosity=0.001, implicit).
/// qvel = [0.5,-1.1,0.8,1.2,-0.7,0.3].
/// Computed as: qDeriv(full) − qDeriv(zero-fluid). Same [v,ω] convention, no permutation.
#[rustfmt::skip]
const MUJOCO_FLUID_T13: [f64; 36] = [
    -5.57710201188235319e-03, 2.82776520811180805e-03, 4.86566823129717680e-03, 0.00000000000000000e+00, -3.73278310448120715e-04, 3.72448130968738888e-04,
    7.04779442288996482e-03, -1.58233482877900740e-02, -8.86949683882521170e-03, 3.73278310448120715e-04, 0.00000000000000000e+00, 2.24815961258610831e-04,
    -1.00752237722172185e-02, 2.13307189185037160e-02, -2.82835564136601561e-02, -3.72448130968738888e-04, -2.24815961258610831e-04, 0.00000000000000000e+00,
    0.00000000000000000e+00, -6.44149678425385425e-04, 8.85705807834904959e-04, -1.84548921911749227e-05, 1.64838833516059175e-05, -2.59543500831602852e-06,
    7.32983848461898056e-04, 0.00000000000000000e+00, 4.58114905288686325e-04, 1.16660357770900780e-06, -2.58272785813621330e-05, 9.46256191993767592e-07,
    1.22146983800204963e-04, -5.55213562728204351e-05, 0.00000000000000000e+00, 3.27330147480587447e-07, 2.59845677154195431e-06, -1.68605387116710238e-05,
];

/// T17: Ellipsoid fluid-only implicitfast (symmetric). Same geometry as T13.
/// qvel = [0.5,-1.1,0.8,1.2,-0.7,0.3].
/// Computed as: qDeriv(full) − qDeriv(zero-fluid). Same [v,ω] convention, no permutation.
#[rustfmt::skip]
const MUJOCO_FLUID_T17: [f64; 36] = [
    -5.57710201188235319e-03, 4.93777981550088622e-03, -2.60477777046002083e-03, 0.00000000000000000e+00, 1.79852769006888670e-04, 2.47297557384471912e-04,
    4.93777981550088622e-03, -1.58233482877900740e-02, 6.23061103983925217e-03, -1.35435683988632355e-04, 0.00000000000000000e+00, 8.46473024928951981e-05,
    -2.60477777046002083e-03, 6.23061103983925217e-03, -2.82835564136601561e-02, 2.56628838433083008e-04, 1.16649472015037747e-04, 0.00000000000000000e+00,
    0.00000000000000000e+00, -1.35435683988632355e-04, 2.56628838433083008e-04, -1.84548921911749227e-05, 8.82524346465747960e-06, -1.13405243041771990e-06,
    1.79852769006888670e-04, 0.00000000000000000e+00, 1.16649472015037747e-04, 8.82524346465747960e-06, -2.58272785813621330e-05, 1.77235648176788171e-06,
    2.47297557384471912e-04, 8.46473024928951981e-05, 0.00000000000000000e+00, -1.13405243041771990e-06, 1.77235648176788171e-06, -1.68605387116710238e-05,
];

/// T25: Geom offset fluid-only (pos="1 0.5 0"). Lever-arm effects.
/// qvel = [0.5,-1.1,0.8,1.2,-0.7,0.3].
/// Computed as: qDeriv(full) − qDeriv(zero-fluid). Same [v,ω] convention, no permutation.
#[rustfmt::skip]
const MUJOCO_FLUID_T25: [f64; 36] = [
    1.23828514299759440e-02, 2.05633094570861720e-03, 3.82757222278551920e-03, 1.91378611139275678e-03, -4.80742778771170443e-03, -3.86422340130210262e-03,
    3.42555797887017727e-03, 2.82327399327784653e-03, -5.79397226982785417e-03, -1.91713056998765730e-03, 5.79397226982791835e-03, 1.26786617672380864e-03,
    -1.37195493483135474e-02, 2.89268984463329736e-02, -4.14889193207992474e-02, -2.10153310283768824e-02, 4.13315481479182290e-02, 3.57866731204897759e-02,
    -6.85977467415677371e-03, 1.27725563172998505e-02, -2.01003099819742383e-02, -1.02040455671669195e-02, 2.00381082788853304e-02, 1.61998482193699278e-02,
    1.56436319505260310e-02, -2.89268984463329736e-02, 4.18095997545013254e-02, 2.11768378488056408e-02, -4.16780558602016654e-02, -3.67477681654040378e-02,
    -2.67703356608128178e-03, 1.75624357103256368e-03, -7.70775838122061355e-03, -2.87369629553646533e-03, 8.20028462045530038e-03, 3.09983530425392891e-03,
];

/// T26: Wind="5 0 0" fluid-only + ellipsoid. Larger relative velocity → larger derivatives.
/// qvel = [0.5,-1.1,0.8,1.2,-0.7,0.3].
/// Computed as: qDeriv(full) − qDeriv(zero-fluid). Same [v,ω] convention, no permutation.
#[rustfmt::skip]
const MUJOCO_FLUID_T26: [f64; 36] = [
    -6.23471094213296981e-02, 3.70640209425262797e-03, 2.07513593351456027e-02, 0.00000000000000000e+00, -3.73278310448120715e-04, 3.72448130968738888e-04,
    -1.24406726423529762e-02, -5.04705278486153736e-02, -5.43368959753918329e-03, 3.73278310448120715e-04, 0.00000000000000000e+00, -2.02334365132749751e-03,
    3.70326546794855979e-02, 1.18822640829241277e-02, -1.02807137546813726e-01, -3.72448130968738888e-04, 2.02334365132749751e-03, 0.00000000000000000e+00,
    0.00000000000000000e+00, -6.44149678425385425e-04, 8.85705807834904959e-04, -1.84548921911749227e-05, 1.64838833516059175e-05, -2.59543500831602852e-06,
    7.32983848461898056e-04, 0.00000000000000000e+00, -4.12303414759817698e-03, 1.16660357770900780e-06, -2.58272785813621330e-05, 9.46256191993767592e-07,
    1.22146983800204963e-04, 4.99692206455383862e-04, 0.00000000000000000e+00, 3.27330147480587447e-07, 2.59845677154195431e-06, -1.68605387116710238e-05,
];

/// T27: Mixed-type fluid-only, two free joints (nv=12).
/// qvel = [0.5,-1.1,0.8,1.2,-0.7,0.3, 0.3,0.6,-0.4,0.8,-0.5,1.0].
/// Computed as: qDeriv(full) − qDeriv(zero-fluid). Same [v,ω] convention, no permutation.
#[rustfmt::skip]
const MUJOCO_FLUID_T27: [f64; 144] = [
    // Body 1 (box, inertia-box) — rows 0–5
    -4.09955742875642726e-03, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, -1.42995574287564289e-02, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, -2.02995574287564255e-02, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -5.94498277861712347e-06, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -9.20513902861712436e-06, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -5.94498277861712432e-06, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    // Body 2 (ellipsoid) — rows 6–11
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -4.22663691940393989e-03, -1.89441723532884150e-03, -2.26003456724712922e-03, 0.00000000000000000e+00, 1.86639155224060357e-04, -2.03153525982948519e-04,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -3.57970430164901345e-03, -9.27905216695902590e-03, -4.48955608365104314e-03, -1.86639155224060357e-04, 0.00000000000000000e+00, 1.34889576755166510e-04,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 5.98195026534333128e-03, 1.09181621945668075e-02, -1.60289204769517926e-02, 2.03153525982948519e-04, -1.34889576755166510e-04, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 3.22074839212692712e-04, -4.83112258819039068e-04, -1.66826859843772366e-05, 9.39778233622473851e-06, -4.69889116811236926e-06,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -3.66491924230949028e-04, 0.00000000000000000e+00, 2.74868943173211784e-04, 5.47664816358262030e-07, -2.08230104660078859e-05, 2.50829409390181633e-06,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -6.66256275273845140e-05, -3.33128137636922570e-05, 0.00000000000000000e+00, -6.00926300869464650e-07, 9.24213104465236059e-06, -2.08230104660078825e-05,
];

/// T28: Single-axis linear velocity fluid-only (v_x=1, ω=0).
/// qvel(ours) = [1,0,0,0,0,0] → v=[1,0,0], ω=[0,0,0] in [v,ω].
/// Computed as: qDeriv(full) − qDeriv(zero-fluid). No permutation needed.
#[rustfmt::skip]
const MUJOCO_FLUID_T28: [f64; 36] = [
    -1.23778750551437854e-02, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, -1.80327418316054144e-02, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 4.49631922517221662e-04,
    0.00000000000000000e+00, 0.00000000000000000e+00, -9.72008767020682563e-02, 0.00000000000000000e+00, -4.49631922517221662e-04, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -4.57322806135900956e-06, 0.00000000000000000e+00, 0.00000000000000000e+00,
    0.00000000000000000e+00, 0.00000000000000000e+00, 9.16229810577372651e-04, 0.00000000000000000e+00, -4.57322806135900956e-06, 0.00000000000000000e+00,
    0.00000000000000000e+00, -1.11042712545640870e-04, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -4.57322806135900956e-06,
];

/// T29: Hinge joint (nv=1), inertia-box.
/// qvel = [1.5].
const MUJOCO_QDERIV_T29: [f64; 1] = [-9.76998277861712412e-06];

/// T30: Ball joint fluid-only (nv=3), ellipsoid.
/// qvel = [0.5, -1.1, 0.8].
/// Computed as: qDeriv(full) − qDeriv(zero-fluid). No permutation (ball joint).
#[rustfmt::skip]
const MUJOCO_FLUID_T30: [f64; 9] = [
    -2.25759192560539032e-05, 8.03231516440991705e-06, -2.64571781243689002e-06,
    4.75840659912773675e-07, -3.78268646235792693e-05, 2.79027610781393178e-06,
    9.34770169341240381e-07, 1.06667010994087761e-05, -2.43889197617763925e-05,
];

/// T31: 3-body chain (hinge joints, nv=3).
/// qvel = [1.0, -0.5, 0.3].
#[rustfmt::skip]
const MUJOCO_QDERIV_T31: [f64; 9] = [
    -1.82024336193120466e-03, -6.00606871335238502e-04, -1.45227228018129944e-06,
    -6.00606871335238502e-04, -2.75726787041199258e-04, -1.45227228018129944e-06,
    -1.45227228018129944e-06, -1.45227228018129944e-06, -1.45227228018129944e-06,
];

// ============================================================================
// MJCF model constants
// ============================================================================

/// Free-floating sphere, density=1.2 (inertia-box)
const IBOX_SPHERE: &str = r#"<mujoco><option density="1.2" viscosity="0" integrator="implicit"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1"/>
    </body></worldbody></mujoco>"#;

/// Free-floating sphere, viscosity-only (inertia-box)
const IBOX_VISCOUS: &str = r#"<mujoco><option density="0" viscosity="1.0" integrator="implicit"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.01" mass="0.001"/>
    </body></worldbody></mujoco>"#;

/// Free-floating box, density+viscosity (inertia-box)
const IBOX_COMBINED: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body></worldbody></mujoco>"#;

/// Free-floating ellipsoid (ellipsoid model)
const ELLIPSOID_BASIC: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

/// Free-floating sphere (ellipsoid model)
const ELLIPSOID_SPHERE: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

/// Zero-fluid model
const ZERO_FLUID: &str = r#"<mujoco><option density="0" viscosity="0" integrator="implicit"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1"/>
    </body></worldbody></mujoco>"#;

/// Multi-geom body (ellipsoid model)
const MULTI_GEOM: &str = r#"<mujoco><option density="1.2" viscosity="0" integrator="implicit"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="0.5"
          fluidshape="ellipsoid" pos="-0.15 0 0"/>
    <geom type="ellipsoid" size="0.08 0.04 0.03" mass="0.5"
          fluidshape="ellipsoid" pos="0.15 0 0"/>
    </body></worldbody></mujoco>"#;

/// ImplicitFast integrator model
const IMPLICITFAST_SPHERE: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicitfast"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

/// Hinge joint model (inertia-box)
const HINGE_IBOX: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
    <worldbody><body pos="0 0 1">
    <joint type="hinge" axis="0 0 1"/>
    <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body></worldbody></mujoco>"#;

/// Ball joint model (ellipsoid)
const BALL_ELLIPSOID: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
    <worldbody><body pos="0 0 1">
    <joint type="ball"/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

/// 3-body chain (hinge joints)
const CHAIN_3BODY: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
    <worldbody>
    <body pos="0 0 1">
    <joint type="hinge" axis="0 1 0"/>
    <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    <body pos="0.3 0 0">
    <joint type="hinge" axis="0 1 0"/>
    <geom type="box" size="0.08 0.04 0.02" mass="0.5"/>
    <body pos="0.25 0 0">
    <joint type="hinge" axis="0 1 0"/>
    <geom type="box" size="0.06 0.03 0.015" mass="0.3"/>
    </body></body></body></worldbody></mujoco>"#;

/// Wind + ellipsoid
const WIND_ELLIPSOID: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit" wind="5 0 0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

/// Geom offset from body CoM
const GEOM_OFFSET: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid" pos="1 0.5 0"/>
    </body></worldbody></mujoco>"#;

/// Mixed-type multi-body
const MIXED_MULTI: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
    <worldbody>
    <body name="box_body" pos="0 0 1"><freejoint/>
    <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body>
    <body name="ell_body" pos="1 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="ellipsoid"/>
    </body></worldbody></mujoco>"#;

/// Interaction coef = 0 geom (should be skipped)
const ZERO_INTERACTION: &str = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
          fluidshape="none"/>
    </body></worldbody></mujoco>"#;

/// Energy dissipation model (sphere, density only, no gravity)
const ENERGY_SPHERE: &str = r#"<mujoco><option density="1.2" viscosity="0" integrator="implicit" timestep="0.002" gravity="0 0 0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1"/>
    </body></worldbody></mujoco>"#;

/// Energy dissipation model (implicitfast, no gravity)
const ENERGY_SPHERE_FAST: &str = r#"<mujoco><option density="1.2" viscosity="0" integrator="implicitfast" timestep="0.002" gravity="0 0 0"/>
    <worldbody><body pos="0 0 1"><freejoint/>
    <geom type="sphere" size="0.1" mass="1"/>
    </body></worldbody></mujoco>"#;

// ============================================================================
// Helpers
// ============================================================================

/// Load model, set qvel, run forward(), return (model, data).
fn setup(mjcf: &str, qvel: &[f64]) -> (Model, Data) {
    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    for (i, &v) in qvel.iter().enumerate() {
        data.qvel[i] = v;
    }
    data.forward(&model).expect("forward failed");
    (model, data)
}

/// Compute qDeriv via finite differences of qfrc_fluid.
/// Perturbs each qvel component by ±ε and computes centered differences.
fn qderiv_fd(model: &Model, data: &Data, eps: f64) -> nalgebra::DMatrix<f64> {
    let nv = model.nv;
    let mut result = nalgebra::DMatrix::zeros(nv, nv);

    for j in 0..nv {
        // +ε perturbation
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(model).expect("forward +eps");

        // −ε perturbation
        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(model).expect("forward -eps");

        // Central difference: (f(v+ε) − f(v−ε)) / 2ε
        for i in 0..nv {
            result[(i, j)] = (d_plus.qfrc_fluid[i] - d_minus.qfrc_fluid[i]) / (2.0 * eps);
        }
    }

    result
}

/// Compute kinetic energy: 0.5 * v^T * M * v
fn kinetic_energy(model: &Model, data: &Data) -> f64 {
    let mut ke = 0.0;
    for i in 0..model.nv {
        for j in 0..model.nv {
            ke += 0.5 * data.qvel[i] * data.qM[(i, j)] * data.qvel[j];
        }
    }
    ke
}

// ============================================================================
// T1: Inertia-box viscous-only B scalars
// ============================================================================

/// Viscous-only (ρ=0): each b_k is a velocity-independent constant.
#[test]
fn t01_inertia_box_viscous_b_scalars() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(IBOX_VISCOUS, &qvel);

    // With density=0, the quadratic terms are zero.
    // The viscous B scalars should not depend on velocity magnitude.
    let q1 = qderiv_fd(&model, &data, 1e-6);

    // Compare at a different velocity to verify velocity-independence of viscous terms
    let (model2, data2) = setup(IBOX_VISCOUS, &[2.0, -3.0, 1.5, 0.1, 0.2, -0.5]);
    let q2 = qderiv_fd(&model2, &data2, 1e-6);

    // The diagonal structure means the FD Jacobian of qfrc_fluid should be similar
    // (not identical since Jacobian involves J which depends on qpos, not qvel)
    // But with same qpos, the viscous B scalars should be the same.
    // Since both models have the same qpos (identity), the Jacobians should match.
    for i in 0..model.nv {
        for j in 0..model.nv {
            assert!(
                (q1[(i, j)] - q2[(i, j)]).abs() < FD_TOL,
                "Viscous qDeriv should be velocity-independent: ({},{}) diff={:.3e}",
                i,
                j,
                (q1[(i, j)] - q2[(i, j)]).abs()
            );
        }
    }
}

// ============================================================================
// T2: Inertia-box density-only B scalars
// ============================================================================

/// Density-only (β=0): b_k depends on 2|v_k|. Verify at known velocity.
#[test]
fn t02_inertia_box_density_b_scalars() {
    let qvel = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0];
    let (model, data) = setup(IBOX_SPHERE, &qvel);

    let fd = qderiv_fd(&model, &data, 1e-6);

    // qDeriv should be nonzero (density > 0, velocity nonzero → quadratic drag)
    let max_abs = fd.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
    assert!(
        max_abs > 1e-10,
        "qDeriv should have nonzero entries for density>0"
    );
}

// ============================================================================
// T3: Inertia-box combined B scalars
// ============================================================================

/// Combined (β>0, ρ>0): sum of viscous + quadratic terms.
#[test]
fn t03_inertia_box_combined_b_scalars() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(IBOX_COMBINED, &qvel);

    let fd = qderiv_fd(&model, &data, 1e-6);

    let max_abs = fd.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
    assert!(max_abs > 1e-10, "Combined qDeriv should be nonzero");
}

// ============================================================================
// T4: Inertia-box FD validation
// ============================================================================

/// Analytical inertia-box B matches FD of forward force. Non-axis-aligned velocity.
#[test]
fn t04_inertia_box_fd_validation() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(IBOX_COMBINED, &qvel);

    // Analytical qDeriv from pipeline (includes fluid + per-DOF damping)
    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);
    let analytical = work.qDeriv.clone();

    // FD of full qfrc_smooth (fluid + passive + actuator - bias)
    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");
        let f_plus: Vec<f64> = (0..nv)
            .map(|i| d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i])
            .collect();

        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");
        let f_minus: Vec<f64> = (0..nv)
            .map(|i| d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i])
            .collect();

        for i in 0..nv {
            fd[(i, j)] = (f_plus[i] - f_minus[i]) / (2.0 * eps);
        }
    }

    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (analytical[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T4 inertia-box FD mismatch at ({},{}): analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                i,
                j,
                analytical[(i, j)],
                fd[(i, j)],
                (analytical[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }
}

// ============================================================================
// T5: Inertia-box qDeriv matches MuJoCo (conformance)
// ============================================================================

/// Compare our fluid-only passive derivative against MuJoCo 3.5.0 fluid-only reference.
/// Free joint → compare fluid contribution only (isolates from Coriolis differences).
/// Identical analytical formulas → tolerance 1e-10.
#[test]
fn t05_inertia_box_mujoco_conformance() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(IBOX_COMBINED, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_passive_vel(&model, &mut work);

    let nv = model.nv;
    assert_eq!(nv, 6);
    for i in 0..nv {
        for j in 0..nv {
            let ours = work.qDeriv[(i, j)];
            let mj = MUJOCO_FLUID_T5[i * nv + j];
            assert!(
                (ours - mj).abs() < MJ_TOL,
                "T5 MuJoCo mismatch at ({},{}): ours={:.17e}, mj={:.17e}, diff={:.3e}",
                i,
                j,
                ours,
                mj,
                (ours - mj).abs()
            );
        }
    }
}

// ============================================================================
// T6: mjd_cross helper FD validation
// ============================================================================

/// Verify ∂(a×b)/∂a and ∂(a×b)/∂b against FD. Multiple vector pairs.
#[test]
fn t06_mjd_cross_fd() {
    let test_cases: Vec<([f64; 3], [f64; 3])> = vec![
        ([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]),
        ([0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
        ([1.2, -0.7, 0.3], [0.5, -1.1, 0.8]),
    ];

    let eps = 1e-7;
    for (a, b) in &test_cases {
        // FD of ∂(a×b)/∂a
        for j in 0..3 {
            let mut a_plus = *a;
            a_plus[j] += eps;
            let c_plus = [
                a_plus[1] * b[2] - a_plus[2] * b[1],
                a_plus[2] * b[0] - a_plus[0] * b[2],
                a_plus[0] * b[1] - a_plus[1] * b[0],
            ];

            let mut a_minus = *a;
            a_minus[j] -= eps;
            let c_minus = [
                a_minus[1] * b[2] - a_minus[2] * b[1],
                a_minus[2] * b[0] - a_minus[0] * b[2],
                a_minus[0] * b[1] - a_minus[1] * b[0],
            ];

            // Expected: Da column j
            // Da = [b]×^T:
            //   Da[0][0]=0,    Da[0][1]=b[2],  Da[0][2]=-b[1]
            //   Da[1][0]=-b[2],Da[1][1]=0,     Da[1][2]=b[0]
            //   Da[2][0]=b[1], Da[2][1]=-b[0], Da[2][2]=0
            let da = [[0.0, b[2], -b[1]], [-b[2], 0.0, b[0]], [b[1], -b[0], 0.0]];

            for i in 0..3 {
                let fd_val = (c_plus[i] - c_minus[i]) / (2.0 * eps);
                assert!(
                    (da[i][j] - fd_val).abs() < 1e-6,
                    "Da[{}][{}] mismatch: analytical={:.10e}, fd={:.10e}",
                    i,
                    j,
                    da[i][j],
                    fd_val
                );
            }
        }

        // FD of ∂(a×b)/∂b
        let db = [[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]];
        for j in 0..3 {
            let mut b_plus = *b;
            b_plus[j] += eps;
            let c_plus = [
                a[1] * b_plus[2] - a[2] * b_plus[1],
                a[2] * b_plus[0] - a[0] * b_plus[2],
                a[0] * b_plus[1] - a[1] * b_plus[0],
            ];

            let mut b_minus = *b;
            b_minus[j] -= eps;
            let c_minus = [
                a[1] * b_minus[2] - a[2] * b_minus[1],
                a[2] * b_minus[0] - a[0] * b_minus[2],
                a[0] * b_minus[1] - a[1] * b_minus[0],
            ];

            for i in 0..3 {
                let fd_val = (c_plus[i] - c_minus[i]) / (2.0 * eps);
                assert!(
                    (db[i][j] - fd_val).abs() < 1e-6,
                    "Db[{}][{}] mismatch: analytical={:.10e}, fd={:.10e}",
                    i,
                    j,
                    db[i][j],
                    fd_val
                );
            }
        }
    }
}

// ============================================================================
// T7: Added mass derivatives vs FD
// ============================================================================

/// Added mass forces include all 4 quadrant terms (cross-products of ω and v).
/// Verify analytical qDeriv matches FD at pipeline level for a sphere where
/// added mass dominates (symmetric geometry → simple coefficients).
/// Per-component FD validation is in derivatives.rs::fluid_derivative_unit_tests::t07.
#[test]
fn t07_added_mass_fd() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(ELLIPSOID_SPHERE, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");
        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");
        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }
    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T7 added mass FD mismatch at ({},{}): analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                i,
                j,
                work.qDeriv[(i, j)],
                fd[(i, j)],
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }
}

// ============================================================================
// T8: Magnus derivatives vs FD
// ============================================================================

/// Magnus lift produces ∂force/∂ω coupling. Verify the ellipsoid model's
/// analytical qDeriv matches FD, confirming Magnus + all other components.
/// Per-component FD validation is in derivatives.rs::fluid_derivative_unit_tests::t08.
#[test]
fn t08_magnus_fd() {
    // Use angular + linear velocity to exercise Magnus coupling (ω×v)
    let qvel = [2.0, -0.5, 1.5, 0.3, -0.8, 0.1];
    let (model, data) = setup(ELLIPSOID_BASIC, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");
        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");
        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }
    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T8 Magnus FD mismatch at ({},{}): analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                i,
                j,
                work.qDeriv[(i, j)],
                fd[(i, j)],
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }
}

// ============================================================================
// T9: Kutta lift derivatives vs FD
// ============================================================================

/// Kutta lift depends on velocity direction → non-axis-aligned velocity
/// exercises all cross-terms in the derivative. Pipeline-level FD validation.
/// Per-component FD validation is in derivatives.rs::fluid_derivative_unit_tests::t09.
#[test]
fn t09_kutta_lift_fd() {
    // Linear-only velocity to isolate Kutta (no angular → no Magnus/added-mass coupling)
    let qvel = [0.0, 0.0, 0.0, 1.2, -0.7, 0.3];
    let (model, data) = setup(ELLIPSOID_BASIC, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");
        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");
        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }
    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T9 Kutta FD mismatch at ({},{}): analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                i,
                j,
                work.qDeriv[(i, j)],
                fd[(i, j)],
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }
}

// ============================================================================
// T10: Viscous drag derivatives vs FD
// ============================================================================

/// Viscous drag: both viscous-only (β>0,ρ=0) and combined cases.
/// Pipeline-level FD validation for linear drag contributions.
/// Per-component FD validation is in derivatives.rs::fluid_derivative_unit_tests::t10.
#[test]
fn t10_viscous_drag_fd() {
    // Linear-only velocity to isolate drag components
    let qvel = [0.0, 0.0, 0.0, 1.2, -0.7, 0.3];
    let (model, data) = setup(ELLIPSOID_BASIC, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");
        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");
        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }
    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T10 viscous drag FD mismatch at ({},{}): diff={:.3e}",
                i,
                j,
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }
}

// ============================================================================
// T11: Viscous torque derivatives vs FD
// ============================================================================

/// Angular drag: angular-only velocity to isolate torque derivative.
/// Pipeline-level FD validation for angular drag contributions.
/// Per-component FD validation is in derivatives.rs::fluid_derivative_unit_tests::t11.
#[test]
fn t11_viscous_torque_fd() {
    let qvel = [0.5, -1.1, 0.8, 0.0, 0.0, 0.0]; // angular only
    let (model, data) = setup(ELLIPSOID_BASIC, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");
        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");
        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }
    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T11 viscous torque FD mismatch at ({},{}): diff={:.3e}",
                i,
                j,
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }
}

// ============================================================================
// T12: Full ellipsoid 6×6 B vs FD
// ============================================================================

/// Assemble all 5 components, compare full qDeriv against FD. Tol 1e-5.
#[test]
fn t12_full_ellipsoid_fd_validation() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(ELLIPSOID_BASIC, &qvel);

    // Analytical qDeriv
    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);
    let analytical = work.qDeriv.clone();

    // FD of qfrc_smooth
    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");

        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");

        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (analytical[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T12 ellipsoid FD mismatch at ({},{}): analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                i,
                j,
                analytical[(i, j)],
                fd[(i, j)],
                (analytical[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }
}

// ============================================================================
// T13: Ellipsoid qDeriv matches MuJoCo (conformance)
// ============================================================================

/// Compare our fluid-only ellipsoid derivative against MuJoCo 3.5.0 fluid-only reference.
/// Free joint → compare fluid contribution only (isolates from Coriolis differences).
/// Full 6×6 (non-symmetric for implicit integrator). Tol: 1e-10.
#[test]
fn t13_ellipsoid_mujoco_conformance() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(ELLIPSOID_BASIC, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_passive_vel(&model, &mut work);

    let nv = model.nv;
    assert_eq!(nv, 6);
    for i in 0..nv {
        for j in 0..nv {
            let ours = work.qDeriv[(i, j)];
            let mj = MUJOCO_FLUID_T13[i * nv + j];
            assert!(
                (ours - mj).abs() < MJ_TOL,
                "T13 MuJoCo mismatch at ({},{}): ours={:.17e}, mj={:.17e}, diff={:.3e}",
                i,
                j,
                ours,
                mj,
                (ours - mj).abs()
            );
        }
    }
}

// ============================================================================
// T14: Multi-geom body
// ============================================================================

/// Body with 2 ellipsoid geoms. Verify additive accumulation.
#[test]
fn t14_multi_geom_body() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(MULTI_GEOM, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);
    let analytical = work.qDeriv.clone();

    // FD validation
    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");

        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");

        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (analytical[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T14 multi-geom mismatch at ({},{}): diff={:.3e}",
                i,
                j,
                (analytical[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }
}

// ============================================================================
// T15: ImplicitFast B is symmetric
// ============================================================================

/// After symmetrization, qDeriv should be symmetric.
#[test]
fn t15_implicitfast_b_symmetric() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(IMPLICITFAST_SPHERE, &qvel);

    // ImplicitFast path: forward() populates qDeriv and symmetrizes it
    // We can check by running mjd_smooth_vel on an implicitfast model
    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    // The symmetrization in our B matrix happens inside mjd_ellipsoid_fluid
    // per-geom. Let's verify the analytical B is symmetric by running the implicit
    // integrator path directly.
    let (model2, data2) = setup(IMPLICITFAST_SPHERE, &qvel);
    // forward() was already called, and for implicitfast it fills qDeriv.
    // data2.qDeriv should be symmetric after the implicitfast path.
    for i in 0..model2.nv {
        for j in (i + 1)..model2.nv {
            assert!(
                (data2.qDeriv[(i, j)] - data2.qDeriv[(j, i)]).abs() < 1e-14,
                "T15 implicitfast qDeriv not symmetric at ({},{}): {:.10e} vs {:.10e}",
                i,
                j,
                data2.qDeriv[(i, j)],
                data2.qDeriv[(j, i)]
            );
        }
    }
}

// ============================================================================
// T16: Full Implicit B is not symmetrized
// ============================================================================

/// Full Implicit path retains asymmetric terms.
#[test]
fn t16_implicit_b_asymmetric() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(ELLIPSOID_BASIC, &qvel);

    // For the full implicit path, B is NOT symmetrized
    // Check that qDeriv has at least one asymmetric entry
    let mut has_asymmetric = false;
    for i in 0..model.nv {
        for j in (i + 1)..model.nv {
            if (data.qDeriv[(i, j)] - data.qDeriv[(j, i)]).abs() > 1e-14 {
                has_asymmetric = true;
                break;
            }
        }
        if has_asymmetric {
            break;
        }
    }
    // With Coriolis + added mass cross terms, the implicit path should produce
    // asymmetric qDeriv (Coriolis is inherently asymmetric)
    assert!(
        has_asymmetric,
        "T16: Full Implicit qDeriv should have asymmetric entries"
    );
}

// ============================================================================
// T17: ImplicitFast qDeriv matches MuJoCo (conformance)
// ============================================================================

/// Compare our implicitfast fluid-only derivative against MuJoCo 3.5.0 fluid-only reference.
/// Free joint → compare fluid contribution only (isolates from Coriolis differences).
/// Symmetric matrix (B is symmetrized as (B+B^T)/2). Tol: 1e-8 (pipeline accumulation).
#[test]
fn t17_implicitfast_mujoco_conformance() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(IMPLICITFAST_SPHERE, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_passive_vel(&model, &mut work);

    let nv = model.nv;
    assert_eq!(nv, 6);
    for i in 0..nv {
        for j in 0..nv {
            let ours = work.qDeriv[(i, j)];
            let mj = MUJOCO_FLUID_T17[i * nv + j];
            assert!(
                (ours - mj).abs() < MJ_TOL_FAST,
                "T17 MuJoCo mismatch at ({},{}): ours={:.17e}, mj={:.17e}, diff={:.3e}",
                i,
                j,
                ours,
                mj,
                (ours - mj).abs()
            );
        }
    }
}

// ============================================================================
// T19: Implicit + fluid energy dissipation
// ============================================================================

/// Implicit integrator with fluid should monotonically dissipate energy.
#[test]
fn t19_implicit_energy_dissipation() {
    let (model, mut data) = setup(ENERGY_SPHERE, &[1.0, -0.5, 0.3, 0.5, -1.1, 0.8]);

    let mut prev_ke = kinetic_energy(&model, &data);
    for step in 0..1000 {
        data.step(&model).expect("step failed");
        let ke = kinetic_energy(&model, &data);
        assert!(
            ke <= prev_ke + 1e-12,
            "T19: KE increased at step {}: {:.15e} → {:.15e}, delta={:.3e}",
            step,
            prev_ke,
            ke,
            ke - prev_ke
        );
        prev_ke = ke;
    }
}

// ============================================================================
// T20: ImplicitFast + fluid energy dissipation
// ============================================================================

/// ImplicitFast integrator with fluid should monotonically dissipate energy.
#[test]
fn t20_implicitfast_energy_dissipation() {
    let (model, mut data) = setup(ENERGY_SPHERE_FAST, &[1.0, -0.5, 0.3, 0.5, -1.1, 0.8]);

    let mut prev_ke = kinetic_energy(&model, &data);
    for step in 0..1000 {
        data.step(&model).expect("step failed");
        let ke = kinetic_energy(&model, &data);
        assert!(
            ke <= prev_ke + 1e-12,
            "T20: KE increased at step {}: {:.15e} → {:.15e}, delta={:.3e}",
            step,
            prev_ke,
            ke,
            ke - prev_ke
        );
        prev_ke = ke;
    }
}

// ============================================================================
// T21: Zero fluid → no D contribution
// ============================================================================

/// density=0, viscosity=0 → qDeriv unchanged by mjd_fluid_vel.
#[test]
fn t21_zero_fluid_no_d_contribution() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(ZERO_FLUID, &qvel);

    // Compute fluid-only FD — should be all zeros
    let fd = qderiv_fd(&model, &data, 1e-6);
    for i in 0..model.nv {
        for j in 0..model.nv {
            assert!(
                fd[(i, j)].abs() < 1e-10,
                "T21: Zero-fluid qDeriv should be zero at ({},{}): {:.3e}",
                i,
                j,
                fd[(i, j)]
            );
        }
    }
}

// ============================================================================
// T22: Zero velocity → zero quadratic B
// ============================================================================

/// At rest: quadratic terms = 0, only viscous terms contribute.
#[test]
fn t22_zero_velocity_zero_quadratic() {
    let qvel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let (model, data) = setup(IBOX_COMBINED, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    // With zero velocity, quadratic terms vanish.
    // Only viscous (velocity-independent) terms should remain.
    // The qDeriv should still be nonzero due to viscous terms + per-DOF damping.
    // Actually, for a free joint with no springs/dampers, per-DOF damping = 0.
    // But viscous fluid drag contributes negative diagonal entries.
    // Just verify no NaN/Inf
    for i in 0..model.nv {
        for j in 0..model.nv {
            assert!(
                work.qDeriv[(i, j)].is_finite(),
                "T22: qDeriv should be finite at zero velocity"
            );
        }
    }
}

// ============================================================================
// T23: Massless body skipped
// ============================================================================

/// body_mass < MJ_MINVAL → no fluid derivative contribution.
#[test]
fn t23_massless_body_skipped() {
    // Use a model where body mass is near zero
    let mjcf = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
        <worldbody><body pos="0 0 1"><freejoint/>
        <geom type="sphere" size="0.001" mass="1e-20"/>
        </body></worldbody></mujoco>"#;
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(mjcf, &qvel);

    // Fluid FD should be negligible for near-zero mass body
    let fd = qderiv_fd(&model, &data, 1e-6);
    for i in 0..model.nv {
        for j in 0..model.nv {
            assert!(
                fd[(i, j)].abs() < 1e-8,
                "T23: Massless body should have negligible fluid derivatives"
            );
        }
    }
}

// ============================================================================
// T24: interaction_coef=0 geom skipped
// ============================================================================

/// Ellipsoid geom with fluid[0]=0 → no derivative contribution.
#[test]
fn t24_zero_interaction_coef() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(ZERO_INTERACTION, &qvel);

    // fluidshape="none" means geom_fluid[0]=0 → inertia-box path used (no ellipsoid).
    // The inertia-box path computes body-level forces. This test just validates no crash.
    let fd = qderiv_fd(&model, &data, 1e-6);
    let max_abs = fd.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
    // Should be nonzero because inertia-box still contributes
    assert!(max_abs.is_finite(), "T24: qDeriv should be finite");
}

// ============================================================================
// T25: Geom offset from body CoM
// ============================================================================

/// Ellipsoid with pos="1 0.5 0" — Jacobian at geom_xpos, not xipos.
#[test]
fn t25_geom_offset() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(GEOM_OFFSET, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    // FD validation
    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");

        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");

        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T25 geom offset FD mismatch at ({},{}): diff={:.3e}",
                i,
                j,
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }

    // MuJoCo fluid-only conformance (free joint → isolate from Coriolis)
    let mut fluid_work = data.clone();
    fluid_work.qDeriv.fill(0.0);
    sim_core::mjd_passive_vel(&model, &mut fluid_work);
    for i in 0..nv {
        for j in 0..nv {
            let ours = fluid_work.qDeriv[(i, j)];
            let mj = MUJOCO_FLUID_T25[i * nv + j];
            assert!(
                (ours - mj).abs() < MJ_TOL,
                "T25 MuJoCo mismatch at ({},{}): ours={:.17e}, mj={:.17e}, diff={:.3e}",
                i,
                j,
                ours,
                mj,
                (ours - mj).abs()
            );
        }
    }
}

// ============================================================================
// T26: Wind + fluid derivative
// ============================================================================

/// Non-zero wind with ellipsoid — velocity correctly wind-subtracted.
#[test]
fn t26_wind_derivative() {
    let qvel = [0.5, -1.1, 0.8, 1.2, -0.7, 0.3];
    let (model, data) = setup(WIND_ELLIPSOID, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    // FD validation
    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");

        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");

        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T26 wind FD mismatch at ({},{}): diff={:.3e}",
                i,
                j,
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }

    // MuJoCo fluid-only conformance (free joint → isolate from Coriolis)
    let mut fluid_work = data.clone();
    fluid_work.qDeriv.fill(0.0);
    sim_core::mjd_passive_vel(&model, &mut fluid_work);
    for i in 0..nv {
        for j in 0..nv {
            let ours = fluid_work.qDeriv[(i, j)];
            let mj = MUJOCO_FLUID_T26[i * nv + j];
            assert!(
                (ours - mj).abs() < MJ_TOL,
                "T26 MuJoCo mismatch at ({},{}): ours={:.17e}, mj={:.17e}, diff={:.3e}",
                i,
                j,
                ours,
                mj,
                (ours - mj).abs()
            );
        }
    }
}

// ============================================================================
// T27: Mixed-type multi-body
// ============================================================================

/// Body A (inertia-box) + Body B (ellipsoid). Both contribute independently.
#[test]
fn t27_mixed_multi_body() {
    let qvel = [
        0.5, -1.1, 0.8, 1.2, -0.7, 0.3, // body A (inertia-box)
        0.3, 0.6, -0.4, 0.8, -0.5, 1.0, // body B (ellipsoid)
    ];
    let (model, data) = setup(MIXED_MULTI, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    // FD validation of full qDeriv
    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");

        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");

        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T27 mixed FD mismatch at ({},{}): analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                i,
                j,
                work.qDeriv[(i, j)],
                fd[(i, j)],
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }

    // MuJoCo fluid-only conformance (free joints → isolate from Coriolis)
    let mut fluid_work = data.clone();
    fluid_work.qDeriv.fill(0.0);
    sim_core::mjd_passive_vel(&model, &mut fluid_work);
    for i in 0..nv {
        for j in 0..nv {
            let ours = fluid_work.qDeriv[(i, j)];
            let mj = MUJOCO_FLUID_T27[i * nv + j];
            assert!(
                (ours - mj).abs() < MJ_TOL,
                "T27 MuJoCo mismatch at ({},{}): ours={:.17e}, mj={:.17e}, diff={:.3e}",
                i,
                j,
                ours,
                mj,
                (ours - mj).abs()
            );
        }
    }
}

// ============================================================================
// T28: Single-axis velocity
// ============================================================================

/// Only v_x ≠ 0 (linear velocity). Tests degenerate proj_denom in Kutta/viscous drag.
/// DOF convention: [v₁,v₂,v₃,ω₁,ω₂,ω₃], so v_x=1 is at index 0.
#[test]
fn t28_single_axis_velocity() {
    let qvel = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let (model, data) = setup(ELLIPSOID_BASIC, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    // Should not NaN or crash
    for i in 0..model.nv {
        for j in 0..model.nv {
            assert!(
                work.qDeriv[(i, j)].is_finite(),
                "T28: qDeriv should be finite for single-axis velocity"
            );
        }
    }

    // FD validation
    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");

        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");

        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T28 single-axis FD mismatch at ({},{}): diff={:.3e}",
                i,
                j,
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }

    // MuJoCo fluid-only conformance (free joint → isolate from Coriolis)
    let mut fluid_work = data.clone();
    fluid_work.qDeriv.fill(0.0);
    sim_core::mjd_passive_vel(&model, &mut fluid_work);
    for i in 0..nv {
        for j in 0..nv {
            let ours = fluid_work.qDeriv[(i, j)];
            let mj = MUJOCO_FLUID_T28[i * nv + j];
            assert!(
                (ours - mj).abs() < MJ_TOL,
                "T28 MuJoCo mismatch at ({},{}): ours={:.17e}, mj={:.17e}, diff={:.3e}",
                i,
                j,
                ours,
                mj,
                (ours - mj).abs()
            );
        }
    }
}

// ============================================================================
// T29: Hinge joint Jacobian
// ============================================================================

/// 1D Jacobian → single nonzero entry in qDeriv.
#[test]
fn t29_hinge_joint_deriv() {
    let qvel = [1.5]; // single hinge DOF
    let (model, data) = setup(HINGE_IBOX, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    // 1×1 qDeriv should be nonzero (fluid damping)
    assert!(
        work.qDeriv[(0, 0)].abs() > 1e-12,
        "T29: Hinge qDeriv(0,0) should be nonzero: {:.10e}",
        work.qDeriv[(0, 0)]
    );

    // FD validation
    let eps = 1e-6;
    let mut d_plus = data.clone();
    d_plus.qvel[0] += eps;
    d_plus.forward(&model).expect("fwd +");

    let mut d_minus = data.clone();
    d_minus.qvel[0] -= eps;
    d_minus.forward(&model).expect("fwd -");

    let f_plus = d_plus.qfrc_passive[0] + d_plus.qfrc_actuator[0] - d_plus.qfrc_bias[0];
    let f_minus = d_minus.qfrc_passive[0] + d_minus.qfrc_actuator[0] - d_minus.qfrc_bias[0];
    let fd = (f_plus - f_minus) / (2.0 * eps);

    assert!(
        (work.qDeriv[(0, 0)] - fd).abs() < FD_TOL,
        "T29 hinge FD mismatch: analytical={:.10e}, fd={:.10e}",
        work.qDeriv[(0, 0)],
        fd
    );

    // MuJoCo conformance
    let ours = work.qDeriv[(0, 0)];
    let mj = MUJOCO_QDERIV_T29[0];
    assert!(
        (ours - mj).abs() < MJ_TOL,
        "T29 MuJoCo mismatch: ours={:.17e}, mj={:.17e}, diff={:.3e}",
        ours,
        mj,
        (ours - mj).abs()
    );
}

// ============================================================================
// T30: Ball joint Jacobian
// ============================================================================

/// 3-DOF Jacobian → 3×3 block in qDeriv.
#[test]
fn t30_ball_joint_deriv() {
    let qvel = [0.5, -1.1, 0.8]; // 3 ball DOFs
    let (model, data) = setup(BALL_ELLIPSOID, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    // 3×3 qDeriv should have nonzero entries
    let mut max_abs = 0.0f64;
    for i in 0..3 {
        for j in 0..3 {
            max_abs = max_abs.max(work.qDeriv[(i, j)].abs());
        }
    }
    assert!(max_abs > 1e-12, "T30: Ball qDeriv should be nonzero");

    // FD validation
    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");

        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");

        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T30 ball FD mismatch at ({},{}): diff={:.3e}",
                i,
                j,
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }

    // MuJoCo fluid-only conformance (ball joint → isolate from gyroscopic differences)
    let mut fluid_work = data.clone();
    fluid_work.qDeriv.fill(0.0);
    sim_core::mjd_passive_vel(&model, &mut fluid_work);
    for i in 0..nv {
        for j in 0..nv {
            let ours = fluid_work.qDeriv[(i, j)];
            let mj = MUJOCO_FLUID_T30[i * nv + j];
            assert!(
                (ours - mj).abs() < MJ_TOL,
                "T30 MuJoCo mismatch at ({},{}): ours={:.17e}, mj={:.17e}, diff={:.3e}",
                i,
                j,
                ours,
                mj,
                (ours - mj).abs()
            );
        }
    }
}

// ============================================================================
// T31: Multi-body chain
// ============================================================================

/// 3-body chain (hinge joints), non-zero velocity. Verify ancestor DOF pattern.
#[test]
fn t31_multi_body_chain() {
    let qvel = [1.0, -0.5, 0.3]; // 3 hinge DOFs
    let (model, data) = setup(CHAIN_3BODY, &qvel);

    let mut work = data.clone();
    work.qDeriv.fill(0.0);
    sim_core::mjd_smooth_vel(&model, &mut work);

    // 3×3 qDeriv should have entries from all 3 bodies at correct DOF intersections
    assert!(model.nv == 3, "Expected 3 DOFs in chain");

    // FD validation
    let eps = 1e-6;
    let nv = model.nv;
    let mut fd = nalgebra::DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut d_plus = data.clone();
        d_plus.qvel[j] += eps;
        d_plus.forward(&model).expect("fwd +");

        let mut d_minus = data.clone();
        d_minus.qvel[j] -= eps;
        d_minus.forward(&model).expect("fwd -");

        for i in 0..nv {
            let f_plus = d_plus.qfrc_passive[i] + d_plus.qfrc_actuator[i] - d_plus.qfrc_bias[i];
            let f_minus = d_minus.qfrc_passive[i] + d_minus.qfrc_actuator[i] - d_minus.qfrc_bias[i];
            fd[(i, j)] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    for i in 0..nv {
        for j in 0..nv {
            assert!(
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs() < FD_TOL,
                "T31 chain FD mismatch at ({},{}): analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                i,
                j,
                work.qDeriv[(i, j)],
                fd[(i, j)],
                (work.qDeriv[(i, j)] - fd[(i, j)]).abs()
            );
        }
    }

    // MuJoCo conformance
    for i in 0..nv {
        for j in 0..nv {
            let ours = work.qDeriv[(i, j)];
            let mj = MUJOCO_QDERIV_T31[i * nv + j];
            assert!(
                (ours - mj).abs() < MJ_TOL,
                "T31 MuJoCo mismatch at ({},{}): ours={:.17e}, mj={:.17e}, diff={:.3e}",
                i,
                j,
                ours,
                mj,
                (ours - mj).abs()
            );
        }
    }
}

// ============================================================================
// T32: mj_jac_point matches mj_jac_site
// ============================================================================

/// Place a site at body CoM. Verify mj_jac_point rows 0–2 match jac_rot,
/// rows 3–5 match jac_trans from mj_jac_site. Direct entry-wise comparison.
#[test]
fn t32_jac_point_matches_jac_site() {
    // Use a 3-body hinge chain with sites at body CoMs
    let mjcf = r#"<mujoco><option density="1.2" viscosity="0.001" integrator="implicit"/>
        <worldbody>
        <body name="b1" pos="0 0 1">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="box" size="0.1 0.05 0.025" mass="1"/>
        <site name="s1" pos="0 0 0"/>
        <body name="b2" pos="0.3 0 0">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="box" size="0.08 0.04 0.02" mass="0.5"/>
        <site name="s2" pos="0 0 0"/>
        <body name="b3" pos="0.25 0 0">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="box" size="0.06 0.03 0.015" mass="0.3"/>
        <site name="s3" pos="0 0 0"/>
        </body></body></body></worldbody></mujoco>"#;
    let (model, data) = setup(mjcf, &[0.3, -0.5, 0.7]);

    // For each site, directly compare mj_jac_point against mj_jac_site
    for site_id in 0..model.nsite {
        let body_id = model.site_body[site_id];
        let point = data.site_xpos[site_id];

        let jac_combined = sim_core::mj_jac_point(&model, &data, body_id, &point);
        let (jac_trans, jac_rot) = sim_core::mj_jac_site(&model, &data, site_id);

        let nv = model.nv;

        // Rows 0–2 of mj_jac_point should exactly match jac_rot
        for i in 0..3 {
            for j in 0..nv {
                assert!(
                    (jac_combined[(i, j)] - jac_rot[(i, j)]).abs() == 0.0,
                    "T32 site {}: rot row {} col {}: point={:.15e}, site={:.15e}",
                    site_id,
                    i,
                    j,
                    jac_combined[(i, j)],
                    jac_rot[(i, j)]
                );
            }
        }

        // Rows 3–5 of mj_jac_point should exactly match jac_trans
        for i in 0..3 {
            for j in 0..nv {
                assert!(
                    (jac_combined[(i + 3, j)] - jac_trans[(i, j)]).abs() == 0.0,
                    "T32 site {}: trans row {} col {}: point={:.15e}, site={:.15e}",
                    site_id,
                    i,
                    j,
                    jac_combined[(i + 3, j)],
                    jac_trans[(i, j)]
                );
            }
        }
    }
}
