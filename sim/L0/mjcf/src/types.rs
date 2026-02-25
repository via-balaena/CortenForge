//! Intermediate representation types for MJCF data.
//!
//! These types represent the parsed MJCF structure before conversion to sim types.
//! They closely mirror the MJCF XML schema but use Rust-native types.

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3, Vector4};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

// ============================================================================
// Global Options
// ============================================================================

/// Integration method for simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MjcfIntegrator {
    /// Explicit Euler integration (first-order).
    #[default]
    Euler,
    /// 4th-order Runge-Kutta (high accuracy, expensive).
    RK4,
    /// Full implicit integration (asymmetric D, LU factorization).
    Implicit,
    /// Implicit-fast (symmetric D, Cholesky; skips Coriolis terms).
    ImplicitFast,
    /// Legacy diagonal-only implicit for per-DOF spring/damper forces.
    ImplicitSpringDamper,
}

impl MjcfIntegrator {
    /// Parse integrator from MJCF string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "euler" => Some(Self::Euler),
            "rk4" => Some(Self::RK4),
            "implicit" => Some(Self::Implicit),
            "implicitfast" => Some(Self::ImplicitFast),
            "implicitspringdamper" => Some(Self::ImplicitSpringDamper),
            _ => None,
        }
    }

    /// Get the MJCF string representation.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Euler => "Euler",
            Self::RK4 => "RK4",
            Self::Implicit => "implicit",
            Self::ImplicitFast => "implicitfast",
            Self::ImplicitSpringDamper => "implicitspringdamper",
        }
    }
}

/// Friction cone type for contact constraints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MjcfConeType {
    /// Pyramidal approximation to friction cone (faster).
    #[default]
    Pyramidal,
    /// Elliptic friction cone (more accurate).
    Elliptic,
}

impl MjcfConeType {
    /// Parse cone type from MJCF string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "pyramidal" => Some(Self::Pyramidal),
            "elliptic" => Some(Self::Elliptic),
            _ => None,
        }
    }

    /// Get the MJCF string representation.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Pyramidal => "pyramidal",
            Self::Elliptic => "elliptic",
        }
    }
}

/// Constraint solver algorithm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MjcfSolverType {
    /// Projected Gauss-Seidel (PGS) solver.
    PGS,
    /// Conjugate Gradient solver.
    CG,
    /// Newton solver (default, most accurate).
    #[default]
    Newton,
}

impl MjcfSolverType {
    /// Parse solver type from MJCF string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_uppercase().as_str() {
            "PGS" => Some(Self::PGS),
            "CG" => Some(Self::CG),
            "NEWTON" => Some(Self::Newton),
            _ => None,
        }
    }

    /// Get the MJCF string representation.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::PGS => "PGS",
            Self::CG => "CG",
            Self::Newton => "Newton",
        }
    }
}

/// Jacobian computation type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MjcfJacobianType {
    /// Dense Jacobian matrix.
    #[default]
    Dense,
    /// Sparse Jacobian matrix (better for large systems).
    Sparse,
    /// Automatic selection based on sparsity.
    Auto,
}

impl MjcfJacobianType {
    /// Parse jacobian type from MJCF string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "dense" => Some(Self::Dense),
            "sparse" => Some(Self::Sparse),
            "auto" => Some(Self::Auto),
            _ => None,
        }
    }

    /// Get the MJCF string representation.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Dense => "dense",
            Self::Sparse => "sparse",
            Self::Auto => "auto",
        }
    }
}

/// Simulation flags from `<flag>` element.
///
/// These control which features are enabled during simulation.
/// Note: This struct intentionally has many boolean fields to match MuJoCo's flag element.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[allow(clippy::struct_excessive_bools)]
pub struct MjcfFlag {
    /// Enable constraint force computation.
    pub constraint: bool,
    /// Enable equality constraints.
    pub equality: bool,
    /// Enable joint/tendon friction loss constraints.
    pub frictionloss: bool,
    /// Enable joint/tendon limit constraints.
    pub limit: bool,
    /// Enable contact force computation.
    pub contact: bool,
    /// Enable passive spring forces.
    pub spring: bool,
    /// Enable passive damping forces.
    pub damper: bool,
    /// Enable gravity force.
    pub gravity: bool,
    /// Clamping ctrl values to ctrlrange.
    pub clampctrl: bool,
    /// Enable warm-starting of constraint solver.
    pub warmstart: bool,
    /// Parent-child body collision filtering.
    pub filterparent: bool,
    /// Enable actuation.
    pub actuation: bool,
    /// Solref time constant safety floor (solref[0] >= 2*timestep).
    pub refsafe: bool,
    /// Enable sensor computation.
    pub sensor: bool,
    /// Enable mid-phase collision detection (BVH).
    pub midphase: bool,
    /// Native CCD (vs libccd fallback for convex collision).
    pub nativeccd: bool,
    /// Implicit damping in Euler integrator.
    pub eulerdamp: bool,
    /// Enable auto-reset on NaN/divergence.
    pub autoreset: bool,
    /// Enable contact parameter override.
    pub override_contacts: bool,
    /// Enable energy computation.
    pub energy: bool,
    /// Enable forward/inverse comparison stats.
    pub fwdinv: bool,
    /// Discrete-time inverse dynamics.
    pub invdiscrete: bool,
    /// Island discovery for parallel constraint solving.
    pub island: bool,
    /// Multi-point CCD for flat surfaces.
    pub multiccd: bool,
    /// Enable sleep/deactivation (MuJoCo sleep flag).
    pub sleep: bool,
}

impl Default for MjcfFlag {
    fn default() -> Self {
        Self {
            // Disable flags: true = feature enabled (bit NOT set in disableflags).
            constraint: true,
            equality: true,
            frictionloss: true,
            limit: true,
            contact: true,
            spring: true,
            damper: true,
            gravity: true,
            clampctrl: true,
            warmstart: true,
            filterparent: true,
            actuation: true,
            refsafe: true,
            sensor: true,
            midphase: true,
            nativeccd: true,
            eulerdamp: true,
            autoreset: true,
            island: true, // Fixed: was false, but MuJoCo defaults to disableflags=0 (S2e)
            // Enable flags: false = feature disabled (bit NOT set in enableflags).
            override_contacts: false,
            energy: false, // MuJoCo default: enableflags=0 (energy off unless explicit)
            fwdinv: false,
            invdiscrete: false,
            multiccd: false,
            sleep: false,
        }
    }
}

// ============================================================================
// Compiler Settings
// ============================================================================

/// Angular unit for MJCF compiler.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum AngleUnit {
    /// Angles specified in degrees (MuJoCo default).
    #[default]
    Degree,
    /// Angles specified in radians.
    Radian,
}

/// Inertia computation control from `<compiler inertiafromgeom="..."/>`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum InertiaFromGeom {
    /// Never compute inertia from geoms.
    False,
    /// Always compute inertia from geoms, overriding explicit `<inertial>`.
    True,
    /// Compute from geoms only when body lacks explicit `<inertial>` child.
    #[default]
    Auto,
}

/// Compiler settings from `<compiler>` element.
///
/// Controls how the MJCF model is compiled into simulation data.
/// All attributes use MuJoCo defaults.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[allow(clippy::struct_excessive_bools)]
pub struct MjcfCompiler {
    /// Angular unit for all angle-valued attributes.
    pub angle: AngleUnit,
    /// Euler angle rotation sequence (3 chars from `{x,y,z,X,Y,Z}`).
    pub eulerseq: String,
    /// Directory for mesh and hfield files.
    pub meshdir: Option<String>,
    /// Directory for texture files.
    pub texturedir: Option<String>,
    /// Fallback directory for all asset files.
    pub assetdir: Option<String>,
    /// Automatically infer `limited` from `range` presence.
    pub autolimits: bool,
    /// Controls inertia computation from geoms.
    pub inertiafromgeom: InertiaFromGeom,
    /// Minimum mass for non-world bodies (0 = disabled).
    pub boundmass: f64,
    /// Minimum diagonal inertia for non-world bodies (0 = disabled).
    pub boundinertia: f64,
    /// Correct inertia matrix when triangle inequality is violated.
    pub balanceinertia: bool,
    /// Rescale all masses so total equals this value (-1 = disabled).
    pub settotalmass: f64,
    /// Strip directory components from asset file references.
    pub strippath: bool,
    /// Discard visual-only geoms during compilation.
    pub discardvisual: bool,
    /// Fuse static (jointless) bodies into their parent.
    pub fusestatic: bool,
    // Deferred attributes (stored, no behavior yet):
    /// Fit AABBs to geom groups.
    pub fitaabb: bool,
    /// Enable multi-threaded compilation.
    pub usethread: bool,
    /// Align free joints to body frame.
    pub alignfree: bool,
    /// Use exact mesh inertia computation (signed tetrahedron decomposition).
    /// When false, same exact algorithm is used (CortenForge always uses exact).
    /// Parsed for MJCF round-trip fidelity.
    pub exactmeshinertia: bool,
}

impl Default for MjcfCompiler {
    fn default() -> Self {
        Self {
            angle: AngleUnit::Degree,
            eulerseq: "xyz".to_string(),
            meshdir: None,
            texturedir: None,
            assetdir: None,
            autolimits: true,
            inertiafromgeom: InertiaFromGeom::Auto,
            boundmass: 0.0,
            boundinertia: 0.0,
            balanceinertia: false,
            settotalmass: -1.0,
            strippath: false,
            discardvisual: false,
            fusestatic: false,
            fitaabb: false,
            usethread: true,
            alignfree: false,
            exactmeshinertia: false,
        }
    }
}

// ============================================================================
// Simulation Options
// ============================================================================

/// Global simulation options from `<option>` element.
///
/// These settings control the core simulation parameters including
/// timestep, solver configuration, integration method, and physics options.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfOption {
    // ========== Core Simulation ==========
    /// Timestep in seconds (default: 0.002).
    pub timestep: f64,

    /// Integration method (default: Euler).
    pub integrator: MjcfIntegrator,

    // ========== Solver Configuration ==========
    /// Constraint solver algorithm (default: Newton).
    pub solver: MjcfSolverType,

    /// Maximum number of main solver iterations (default: 100).
    pub iterations: usize,

    /// Solver early termination tolerance (default: 1e-8).
    pub tolerance: f64,

    /// Maximum iterations for line search in CG/Newton solvers (default: 50).
    pub ls_iterations: usize,

    /// Line search gradient tolerance for Newton solver (default: 0.01).
    pub ls_tolerance: f64,

    /// Iterations for no-slip solver (default: 0 = disabled).
    pub noslip_iterations: usize,

    /// Tolerance for no-slip solver (default: 1e-6).
    pub noslip_tolerance: f64,

    /// Iterations for convex collision detection (default: 50).
    pub ccd_iterations: usize,

    // ========== Contact Configuration ==========
    /// Friction cone type (default: Pyramidal).
    pub cone: MjcfConeType,

    /// Jacobian computation type (default: Dense).
    pub jacobian: MjcfJacobianType,

    /// Ratio of frictional-to-normal constraint impedance (default: 1.0).
    ///
    /// Higher values make friction constraints stiffer relative to normal forces.
    pub impratio: f64,

    // ========== Constraint Solver Tuning ==========
    /// Base regularization for PGS constraint matrix diagonal (default: 1e-6).
    /// CFM scales as `regularization + (1 - impedance) * regularization * 100`.
    pub regularization: f64,

    /// Friction smoothing factor — sharpness of tanh transition (default: 1000.0).
    pub friction_smoothing: f64,

    // ========== Physics Environment ==========
    /// Gravity vector (default: 0 0 -9.81).
    pub gravity: Vector3<f64>,

    /// Wind/medium velocity vector (default: 0 0 0).
    pub wind: Vector3<f64>,

    /// Global magnetic flux direction (default: 0 -0.5 0).
    pub magnetic: Vector3<f64>,

    /// Medium density for lift/drag forces in kg/m³ (default: 0 = disabled).
    pub density: f64,

    /// Medium viscosity (default: 0 = disabled).
    pub viscosity: f64,

    // ========== Constraint Limits ==========
    /// Maximum number of contacts (0 = unlimited, default: 0).
    pub nconmax: usize,

    /// Maximum number of constraint pairs (0 = unlimited, default: 0).
    pub njmax: usize,

    // ========== Overrides ==========
    /// Override contact margin (negative = use geom-specific, default: -1).
    pub o_margin: f64,

    /// Override contact solimp parameters.
    pub o_solimp: Option<[f64; 5]>,

    /// Override contact solref parameters.
    pub o_solref: Option<[f64; 2]>,

    /// Override contact friction coefficients.
    pub o_friction: Option<[f64; 5]>,

    // ========== Sleep ==========
    /// Velocity threshold for body sleeping (default: 1e-4).
    pub sleep_tolerance: f64,

    // ========== Per-group actuator disabling ==========
    /// Bitmask parsed from `actuatorgroupdisable` attribute.
    /// Bit `i` set = group `i` disabled. Parsed from space-separated group IDs (0–30).
    pub actuatorgroupdisable: u32,

    // ========== Flags (child element) ==========
    /// Simulation flags controlling feature enable/disable.
    pub flag: MjcfFlag,
}

impl Default for MjcfOption {
    fn default() -> Self {
        Self {
            // Core simulation
            timestep: 0.002, // MuJoCo default
            integrator: MjcfIntegrator::default(),

            // Solver configuration
            solver: MjcfSolverType::default(),
            iterations: 100,
            tolerance: 1e-8,
            ls_iterations: 50,
            ls_tolerance: 0.01,
            noslip_iterations: 0,
            noslip_tolerance: 1e-6,
            ccd_iterations: 50,

            // Contact configuration
            cone: MjcfConeType::default(),
            jacobian: MjcfJacobianType::default(),
            impratio: 1.0,

            // Constraint solver tuning
            regularization: 1e-6,
            friction_smoothing: 1000.0,

            // Physics environment
            gravity: Vector3::new(0.0, 0.0, -9.81), // Z-up convention
            wind: Vector3::zeros(),
            magnetic: Vector3::new(0.0, -0.5, 0.0),
            density: 0.0,
            viscosity: 0.0,

            // Constraint limits
            nconmax: 0,
            njmax: 0,

            // Overrides (negative = use defaults)
            o_margin: -1.0,
            o_solimp: None,
            o_solref: None,
            o_friction: None,

            // Sleep
            sleep_tolerance: 1e-4,

            // Per-group actuator disabling
            actuatorgroupdisable: 0,

            // Flags
            flag: MjcfFlag::default(),
        }
    }
}

impl MjcfOption {
    /// Check if gravity is enabled (flag and non-zero gravity vector).
    #[must_use]
    pub fn gravity_enabled(&self) -> bool {
        self.flag.gravity && self.gravity.norm() > 1e-10
    }

    /// Check if contacts are enabled.
    #[must_use]
    pub fn contacts_enabled(&self) -> bool {
        self.flag.contact
    }

    /// Get effective contact margin (uses override if positive).
    #[must_use]
    pub fn effective_margin(&self) -> Option<f64> {
        if self.o_margin >= 0.0 {
            Some(self.o_margin)
        } else {
            None
        }
    }
}

// ============================================================================
// Default Classes
// ============================================================================

/// Default parameter class from `<default>` element.
///
/// MJCF supports inheritance hierarchies of default classes.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfDefault {
    /// Class name (empty string for root default).
    pub class: String,
    /// Parent class name (for inheritance).
    pub parent_class: Option<String>,
    /// Default joint parameters.
    pub joint: Option<MjcfJointDefaults>,
    /// Default geom parameters.
    pub geom: Option<MjcfGeomDefaults>,
    /// Default actuator parameters.
    pub actuator: Option<MjcfActuatorDefaults>,
    /// Default tendon parameters.
    pub tendon: Option<MjcfTendonDefaults>,
    /// Default sensor parameters.
    pub sensor: Option<MjcfSensorDefaults>,
    /// Default mesh parameters.
    pub mesh: Option<MjcfMeshDefaults>,
    /// Default site parameters.
    pub site: Option<MjcfSiteDefaults>,
    /// Default pair parameters.
    pub pair: Option<MjcfPairDefaults>,
}

/// Default joint parameters.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfJointDefaults {
    /// Joint type.
    pub joint_type: Option<MjcfJointType>,
    /// Joint position relative to body frame.
    pub pos: Option<Vector3<f64>>,
    /// Position limits enabled.
    pub limited: Option<bool>,
    /// Joint axis.
    pub axis: Option<Vector3<f64>>,
    /// Joint reference position.
    pub ref_pos: Option<f64>,
    /// Spring equilibrium position.
    pub spring_ref: Option<f64>,
    /// Damping coefficient.
    pub damping: Option<f64>,
    /// Spring stiffness.
    pub stiffness: Option<f64>,
    /// Armature (rotor inertia).
    pub armature: Option<f64>,
    /// Friction loss.
    pub frictionloss: Option<f64>,
    /// Visualization group (0–5).
    pub group: Option<i32>,
    /// Position limit range [lower, upper].
    pub range: Option<(f64, f64)>,
    /// Solver reference parameters for joint limits [timeconst, dampratio].
    pub solref_limit: Option<[f64; 2]>,
    /// Solver impedance parameters for joint limits [d0, d_width, width, midpoint, power].
    pub solimp_limit: Option<[f64; 5]>,
    /// Solver reference parameters for friction loss [timeconst, dampratio].
    pub solreffriction: Option<[f64; 2]>,
    /// Solver impedance parameters for friction loss [d0, d_width, width, midpoint, power].
    pub solimpfriction: Option<[f64; 5]>,
    /// Gravity compensation routing via actuator.
    pub actuatorgravcomp: Option<bool>,
}

/// Default geom parameters.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfGeomDefaults {
    /// Geom type.
    pub geom_type: Option<MjcfGeomType>,
    /// Friction coefficients.
    pub friction: Option<Vector3<f64>>,
    /// Density for mass computation.
    pub density: Option<f64>,
    /// Explicit mass (overrides density).
    pub mass: Option<f64>,
    /// RGBA color.
    pub rgba: Option<Vector4<f64>>,
    /// Collision enabled.
    pub contype: Option<i32>,
    /// Collision affinity.
    pub conaffinity: Option<i32>,
    /// Contact dimensionality.
    pub condim: Option<i32>,
    /// Contact priority.
    pub priority: Option<i32>,
    /// Solver mixing weight.
    pub solmix: Option<f64>,
    /// Contact margin.
    pub margin: Option<f64>,
    /// Contact gap.
    pub gap: Option<f64>,
    /// Solver reference parameters for contacts [timeconst, dampratio].
    pub solref: Option<[f64; 2]>,
    /// Solver impedance parameters for contacts [d0, d_width, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
    /// Visualization group (0–5).
    pub group: Option<i32>,
    /// Default position.
    pub pos: Option<Vector3<f64>>,
    /// Default quaternion orientation (w, x, y, z).
    pub quat: Option<Vector4<f64>>,
    /// Default euler angles (degrees).
    pub euler: Option<Vector3<f64>>,
    /// Default axis-angle (x, y, z, angle).
    pub axisangle: Option<Vector4<f64>>,
    /// Default two-axis frame (x-axis 3, y-axis 3).
    pub xyaxes: Option<[f64; 6]>,
    /// Default z-axis (minimal rotation from Z).
    pub zaxis: Option<Vector3<f64>>,
    /// Default fromto specification [x1, y1, z1, x2, y2, z2].
    pub fromto: Option<[f64; 6]>,
    /// Default mesh asset name.
    pub mesh: Option<String>,
    /// Default height field asset name.
    pub hfield: Option<String>,
    /// Default material asset name.
    pub material: Option<String>,
    /// Fluid shape for ellipsoid fluid model: `"none"` or `"ellipsoid"`.
    pub fluidshape: Option<FluidShape>,
    /// Fluid coefficients `[C_blunt, C_slender, C_ang, C_K, C_M]`.
    pub fluidcoef: Option<[f64; 5]>,
}

/// Default actuator parameters.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfActuatorDefaults {
    /// Control range.
    pub ctrlrange: Option<(f64, f64)>,
    /// Force range.
    pub forcerange: Option<(f64, f64)>,
    /// Gear ratio (6D: [tx ty tz rx ry rz]).
    pub gear: Option<[f64; 6]>,
    /// Position gain (kp) for position actuators.
    pub kp: Option<f64>,
    /// Velocity gain (kv) for velocity actuators.
    pub kv: Option<f64>,
    /// Control limited.
    pub ctrllimited: Option<bool>,
    /// Force limited.
    pub forcelimited: Option<bool>,
    /// Gain type default for `<general>` actuators.
    pub gaintype: Option<String>,
    /// Bias type default for `<general>` actuators.
    pub biastype: Option<String>,
    /// Dynamics type default for `<general>` actuators.
    pub dyntype: Option<String>,
    /// Gain parameters default for `<general>` actuators.
    pub gainprm: Option<Vec<f64>>,
    /// Bias parameters default for `<general>` actuators.
    pub biasprm: Option<Vec<f64>>,
    /// Dynamics parameters default for `<general>` actuators.
    pub dynprm: Option<Vec<f64>>,
    /// Visualization group (0–5).
    pub group: Option<i32>,
    /// Whether activation is clamped to actrange.
    pub actlimited: Option<bool>,
    /// Activation clamping range [lower, upper].
    pub actrange: Option<(f64, f64)>,
    /// If true, activation dynamics are applied early (before constraint).
    pub actearly: Option<bool>,
    /// Length range for the actuator (requires tendon length range computation).
    pub lengthrange: Option<(f64, f64)>,
    // #todo: MuJoCo supports actuator-type-specific defaults (cylinder area/timeconst/bias,
    // muscle force/scale/lmin/lmax/vmax/fpmax/fvmax/timeconst, adhesion gain).
    // These fields exist on MjcfActuator but are not yet defaultable.
}

/// Default tendon parameters.
///
/// MuJoCo tendons can have these defaults specified in the `<default>` element.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfTendonDefaults {
    /// Range limits [lower, upper] for tendon length.
    pub range: Option<(f64, f64)>,
    /// Whether range limits are enabled.
    pub limited: Option<bool>,
    /// Stiffness coefficient.
    pub stiffness: Option<f64>,
    /// Damping coefficient.
    pub damping: Option<f64>,
    /// Friction loss.
    pub frictionloss: Option<f64>,
    /// Spring rest length pair (low, high) for deadband spring.
    pub springlength: Option<(f64, f64)>,
    /// Tendon width for visualization.
    pub width: Option<f64>,
    /// RGBA color for visualization.
    pub rgba: Option<Vector4<f64>>,
    /// Visualization group (0–5).
    pub group: Option<i32>,
    /// Solver reference parameters for tendon limits [timeconst, dampratio].
    pub solref: Option<[f64; 2]>,
    /// Solver impedance parameters for tendon limits [d0, d_width, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
    /// Solver reference parameters for friction loss [timeconst, dampratio].
    pub solreffriction: Option<[f64; 2]>,
    /// Solver impedance parameters for friction loss [d0, d_width, width, midpoint, power].
    pub solimpfriction: Option<[f64; 5]>,
    /// Contact margin for tendon limits.
    pub margin: Option<f64>,
    /// Default material asset name.
    pub material: Option<String>,
}

/// Default sensor parameters.
///
/// MuJoCo sensors can have these defaults specified in the `<default>` element.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfSensorDefaults {
    /// Noise standard deviation.
    pub noise: Option<f64>,
    /// Cutoff frequency for low-pass filter (0 = no filter).
    pub cutoff: Option<f64>,
    /// User-defined data fields.
    pub user: Option<Vec<f64>>,
}

/// Default mesh parameters.
///
/// MuJoCo mesh loading can have these defaults specified in the `<default>` element.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfMeshDefaults {
    /// Mesh scale factors [x, y, z].
    pub scale: Option<Vector3<f64>>,
}

// ============================================================================
// Assets
// ============================================================================

/// A mesh asset from `<mesh>` element in the `<asset>` section.
///
/// MuJoCo meshes can reference external files or contain embedded vertices.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfMesh {
    /// Asset name (used to reference the mesh from geoms).
    pub name: String,
    /// File path for external mesh (STL, OBJ, etc.).
    pub file: Option<String>,
    /// Scale factor for the mesh vertices.
    pub scale: Option<Vector3<f64>>,
    /// Embedded vertex data (if not loading from file).
    /// Format: flat array of xyz coordinates.
    pub vertex: Option<Vec<f64>>,
    /// Embedded face data (if not loading from file).
    /// Format: flat array of vertex indices (triangles).
    pub face: Option<Vec<u32>>,
}

impl Default for MjcfMesh {
    fn default() -> Self {
        Self {
            name: String::new(),
            file: None,
            scale: None,
            vertex: None,
            face: None,
        }
    }
}

/// Height field asset parsed from `<hfield>` element in MJCF.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfHfield {
    /// Asset name (referenced by `<geom hfield="...">`).
    pub name: String,
    /// Size: `[x, y, z_top, z_bottom]`.
    /// x/y = half-extents; z_top = elevation scale; z_bottom = base depth.
    pub size: [f64; 4],
    /// Number of rows (Y samples).
    pub nrow: usize,
    /// Number of columns (X samples).
    pub ncol: usize,
    /// Normalized elevation data, row-major (row 0 = min Y, X varies fastest).
    /// Length: `nrow × ncol`.
    pub elevation: Vec<f64>,
}

impl MjcfMesh {
    /// Create a new mesh asset with a name.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            ..Default::default()
        }
    }

    /// Create a mesh asset from a file path.
    #[must_use]
    pub fn from_file(name: impl Into<String>, file: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            file: Some(file.into()),
            ..Default::default()
        }
    }

    /// Set the scale factor.
    #[must_use]
    pub fn with_scale(mut self, scale: Vector3<f64>) -> Self {
        self.scale = Some(scale);
        self
    }

    /// Check if this mesh has embedded data.
    #[must_use]
    pub fn has_embedded_data(&self) -> bool {
        self.vertex.is_some()
    }

    /// Check if this mesh references an external file.
    #[must_use]
    pub fn has_file(&self) -> bool {
        self.file.is_some()
    }

    /// Get the number of embedded vertices (if any).
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.vertex.as_ref().map_or(0, |v| v.len() / 3)
    }

    /// Get embedded vertices as Point3 array.
    #[must_use]
    pub fn vertices_as_points(&self) -> Vec<Point3<f64>> {
        let scale = self.scale.unwrap_or(Vector3::new(1.0, 1.0, 1.0));
        self.vertex.as_ref().map_or_else(Vec::new, |v| {
            v.chunks_exact(3)
                .map(|c| Point3::new(c[0] * scale.x, c[1] * scale.y, c[2] * scale.z))
                .collect()
        })
    }
}

/// Default site parameters.
///
/// MuJoCo sites can have these defaults specified in the `<default>` element.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfSiteDefaults {
    /// Site type (sphere, capsule, ellipsoid, cylinder, box).
    pub site_type: Option<String>,
    /// Site size.
    pub size: Option<Vec<f64>>,
    /// RGBA color.
    pub rgba: Option<Vector4<f64>>,
    /// Visualization group (0–5).
    pub group: Option<i32>,
    /// Default position.
    pub pos: Option<Vector3<f64>>,
    /// Default quaternion orientation (w, x, y, z).
    pub quat: Option<Vector4<f64>>,
    /// Default euler angles (degrees).
    pub euler: Option<Vector3<f64>>,
    /// Default axis-angle (x, y, z, angle).
    pub axisangle: Option<Vector4<f64>>,
    /// Default two-axis frame (x-axis 3, y-axis 3).
    pub xyaxes: Option<[f64; 6]>,
    /// Default z-axis (minimal rotation from Z).
    pub zaxis: Option<Vector3<f64>>,
    /// Default material asset name.
    pub material: Option<String>,
}

/// Default pair parameters (from `<default><pair .../>`).
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfPairDefaults {
    /// Contact dimensionality (1, 3, 4, 6).
    pub condim: Option<i32>,
    /// 5-element friction: [tan1, tan2, torsional, roll1, roll2].
    pub friction: Option<[f64; 5]>,
    /// Solver reference (normal direction).
    pub solref: Option<[f64; 2]>,
    /// Solver reference (friction directions).
    pub solreffriction: Option<[f64; 2]>,
    /// Solver impedance.
    pub solimp: Option<[f64; 5]>,
    /// Distance threshold for contact activation.
    pub margin: Option<f64>,
    /// Contact included if distance < margin - gap.
    pub gap: Option<f64>,
}

// ============================================================================
// Inertial Properties
// ============================================================================

/// Inertial properties from `<inertial>` element.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfInertial {
    /// Position of center of mass relative to body frame.
    pub pos: Vector3<f64>,
    /// Orientation of principal axes (quaternion: w x y z).
    pub quat: Vector4<f64>,
    /// Mass in kg.
    pub mass: f64,
    /// Diagonal inertia (Ixx, Iyy, Izz) - if specified.
    pub diaginertia: Option<Vector3<f64>>,
    /// Full inertia tensor (Ixx, Iyy, Izz, Ixy, Ixz, Iyz) - if specified.
    pub fullinertia: Option<[f64; 6]>,
}

impl Default for MjcfInertial {
    fn default() -> Self {
        Self {
            pos: Vector3::zeros(),
            quat: Vector4::new(1.0, 0.0, 0.0, 0.0), // Identity quaternion (w, x, y, z)
            mass: 1.0,
            diaginertia: None,
            fullinertia: None,
        }
    }
}

impl MjcfInertial {
    /// Create inertial with mass only (unit sphere inertia).
    #[must_use]
    pub fn with_mass(mass: f64) -> Self {
        Self {
            mass,
            ..Default::default()
        }
    }

    /// Convert to 3x3 inertia matrix.
    #[must_use]
    pub fn inertia_matrix(&self) -> Matrix3<f64> {
        if let Some(diag) = self.diaginertia {
            // Diagonal inertia
            Matrix3::from_diagonal(&diag)
        } else if let Some(full) = self.fullinertia {
            // Full inertia tensor: [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
            Matrix3::new(
                full[0], full[3], full[4], // Row 0: Ixx, Ixy, Ixz
                full[3], full[1], full[5], // Row 1: Ixy, Iyy, Iyz
                full[4], full[5], full[2], // Row 2: Ixz, Iyz, Izz
            )
        } else {
            // Default: unit sphere inertia (2/5 * m * r^2 with m=mass, r=1)
            let i = 0.4 * self.mass;
            Matrix3::from_diagonal(&Vector3::new(i, i, i))
        }
    }

    /// Get quaternion as UnitQuaternion.
    #[must_use]
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        // MJCF uses (w, x, y, z) order
        let q = nalgebra::Quaternion::new(self.quat[0], self.quat[1], self.quat[2], self.quat[3]);
        UnitQuaternion::from_quaternion(q)
    }
}

// ============================================================================
// Geometry Types
// ============================================================================

/// Fluid shape attribute for per-geom fluid model dispatch.
///
/// Controls whether a geom participates in the ellipsoid fluid model.
/// When `Ellipsoid`, `geom_fluid[0]` is set to 1.0 (master switch on).
/// When `None`, `geom_fluid` remains all zeros (inertia-box model used for body).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum FluidShape {
    /// Inertia-box model (body-level, legacy). Default.
    #[default]
    None,
    /// Ellipsoid model (per-geom, advanced).
    Ellipsoid,
}

/// Geometry type from MJCF.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MjcfGeomType {
    /// Plane (infinite).
    Plane,
    /// Sphere with radius.
    Sphere,
    /// Capsule (cylinder with hemispherical caps).
    Capsule,
    /// Box with half-extents.
    Box,
    /// Cylinder (Z-aligned).
    Cylinder,
    /// Ellipsoid with semi-axes.
    Ellipsoid,
    /// Mesh from file.
    Mesh,
    /// Height field.
    Hfield,
    /// Signed distance field.
    Sdf,
    /// Non-convex triangle mesh.
    /// Unlike `Mesh` which is treated as a convex hull, this preserves
    /// the original triangle structure for accurate non-convex collision.
    TriangleMesh,
}

impl MjcfGeomType {
    /// Parse geom type from string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s {
            "plane" => Some(Self::Plane),
            "sphere" => Some(Self::Sphere),
            "capsule" => Some(Self::Capsule),
            "box" => Some(Self::Box),
            "cylinder" => Some(Self::Cylinder),
            "ellipsoid" => Some(Self::Ellipsoid),
            "mesh" => Some(Self::Mesh),
            "hfield" => Some(Self::Hfield),
            "sdf" => Some(Self::Sdf),
            "trimesh" | "triangle_mesh" | "nonconvex" => Some(Self::TriangleMesh),
            _ => None,
        }
    }

    /// Get the name of this geom type.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Plane => "plane",
            Self::Sphere => "sphere",
            Self::Capsule => "capsule",
            Self::Box => "box",
            Self::Cylinder => "cylinder",
            Self::Ellipsoid => "ellipsoid",
            Self::Mesh => "mesh",
            Self::Hfield => "hfield",
            Self::Sdf => "sdf",
            Self::TriangleMesh => "trimesh",
        }
    }
}

/// Geometry from `<geom>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfGeom {
    /// Optional name.
    pub name: Option<String>,
    /// Default class.
    pub class: Option<String>,
    /// Geom type (None = not explicitly set, inherits from defaults or falls back to Sphere).
    pub geom_type: Option<MjcfGeomType>,
    /// Position relative to body frame.
    pub pos: Option<Vector3<f64>>,
    /// Orientation (quaternion: w x y z).
    pub quat: Option<Vector4<f64>>,
    /// Alternative orientation as euler angles (degrees, XYZ order).
    pub euler: Option<Vector3<f64>>,
    /// Alternative orientation as axis-angle (x, y, z, angle).
    pub axisangle: Option<Vector4<f64>>,
    /// Alternative orientation as two axes (x-axis 3 floats, y-axis 3 floats).
    pub xyaxes: Option<[f64; 6]>,
    /// Alternative orientation as z-axis (minimal rotation from Z).
    pub zaxis: Option<Vector3<f64>>,
    /// Size parameters (interpretation depends on type).
    /// - Sphere: `[radius]`
    /// - Capsule: `[radius, half-length]`
    /// - Box: `[x, y, z]` half-extents
    /// - Cylinder: `[radius, half-length]`
    /// - Plane: `[x, y, spacing]` (spacing for grid visualization)
    pub size: Vec<f64>,
    /// Alternative specification using fromto (start and end points).
    pub fromto: Option<[f64; 6]>,
    /// Friction coefficients [sliding, torsional, rolling].
    pub friction: Option<Vector3<f64>>,
    /// Density for mass computation (kg/m³).
    pub density: Option<f64>,
    /// Explicit mass (overrides density).
    pub mass: Option<f64>,
    /// RGBA color.
    pub rgba: Option<Vector4<f64>>,
    /// Collision type bitmask.
    pub contype: Option<i32>,
    /// Collision affinity bitmask.
    pub conaffinity: Option<i32>,
    /// Contact dimensionality (condim).
    pub condim: Option<i32>,
    /// Mesh asset name (for type="mesh").
    pub mesh: Option<String>,
    /// Height field asset name (for type="hfield").
    pub hfield: Option<String>,
    /// Solver reference parameters for contacts [timeconst, dampratio].
    /// Controls contact softness/stiffness.
    pub solref: Option<[f64; 2]>,
    /// Solver impedance parameters for contacts [d0, d_width, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
    /// Contact priority. When priorities differ, higher-priority geom's params win.
    pub priority: Option<i32>,
    /// Solver mixing weight for contact parameter combination (default 1.0).
    pub solmix: Option<f64>,
    /// Contact margin — expands collision detection distance.
    pub margin: Option<f64>,
    /// Contact gap — creates buffer zone within margin where no force is applied.
    pub gap: Option<f64>,
    /// Visualization group (0–5). Default 0.
    pub group: Option<i32>,
    /// Material asset name (for rendering).
    pub material: Option<String>,
    /// Fluid shape for ellipsoid fluid model: `"none"` or `"ellipsoid"`.
    pub fluidshape: Option<FluidShape>,
    /// Fluid coefficients `[C_blunt, C_slender, C_ang, C_K, C_M]`.
    pub fluidcoef: Option<[f64; 5]>,
}

impl Default for MjcfGeom {
    fn default() -> Self {
        Self {
            name: None,
            class: None,
            geom_type: None,
            pos: None,
            quat: None,
            euler: None,
            axisangle: None,
            xyaxes: None,
            zaxis: None,
            size: vec![0.1], // Default sphere radius
            fromto: None,
            friction: None,
            density: None,
            mass: None,
            rgba: None,
            contype: None,
            conaffinity: None,
            condim: None,
            mesh: None,
            hfield: None,
            solref: None,
            solimp: None,
            priority: None,
            solmix: None,
            margin: None,
            gap: None,
            group: None,
            material: None,
            fluidshape: None,
            fluidcoef: None,
        }
    }
}

impl MjcfGeom {
    /// Create a sphere geom.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        Self {
            geom_type: Some(MjcfGeomType::Sphere),
            size: vec![radius],
            ..Default::default()
        }
    }

    /// Create a box geom.
    #[must_use]
    pub fn box_shape(half_extents: Vector3<f64>) -> Self {
        Self {
            geom_type: Some(MjcfGeomType::Box),
            size: vec![half_extents.x, half_extents.y, half_extents.z],
            ..Default::default()
        }
    }

    /// Create a capsule geom.
    #[must_use]
    pub fn capsule(radius: f64, half_length: f64) -> Self {
        Self {
            geom_type: Some(MjcfGeomType::Capsule),
            size: vec![radius, half_length],
            ..Default::default()
        }
    }

    /// Get quaternion as `UnitQuaternion`.
    #[must_use]
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        let qv = self.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0));
        let q = nalgebra::Quaternion::new(qv[0], qv[1], qv[2], qv[3]);
        UnitQuaternion::from_quaternion(q)
    }

    /// Compute mass from density and volume.
    #[must_use]
    pub fn computed_mass(&self) -> f64 {
        if let Some(mass) = self.mass {
            return mass;
        }

        let volume = match self.geom_type.unwrap_or(MjcfGeomType::Sphere) {
            MjcfGeomType::Sphere => {
                let r = self.size.first().copied().unwrap_or(0.1);
                (4.0 / 3.0) * std::f64::consts::PI * r.powi(3)
            }
            MjcfGeomType::Box => {
                let x = self.size.first().copied().unwrap_or(0.1);
                let y = self.size.get(1).copied().unwrap_or(x);
                let z = self.size.get(2).copied().unwrap_or(y);
                8.0 * x * y * z // Full size = 2 * half-extents
            }
            MjcfGeomType::Capsule | MjcfGeomType::Cylinder => {
                let r = self.size.first().copied().unwrap_or(0.1);
                let h = self.size.get(1).copied().unwrap_or(0.1);
                // Cylinder + 2 hemispheres for capsule
                std::f64::consts::PI * r.powi(2) * (2.0 * h)
                    + (4.0 / 3.0) * std::f64::consts::PI * r.powi(3)
            }
            MjcfGeomType::Plane => 0.0, // Planes have no volume
            _ => 0.001,                 // Default small volume for unsupported types
        };

        self.density.unwrap_or(1000.0) * volume
    }
}

// ============================================================================
// Joint Types
// ============================================================================

/// Joint type from MJCF.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MjcfJointType {
    /// Hinge/revolute joint (1 DOF rotation).
    Hinge,
    /// Slide/prismatic joint (1 DOF translation).
    Slide,
    /// Ball/spherical joint (3 DOF rotation).
    Ball,
    /// Free joint (6 DOF).
    Free,
    /// Cylindrical joint (2 DOF: rotation + translation along same axis).
    /// Note: This is an extension to standard MJCF.
    Cylindrical,
    /// Planar joint (3 DOF: x, y translation + rotation about normal).
    /// Note: This is an extension to standard MJCF.
    Planar,
}

impl MjcfJointType {
    /// Parse joint type from string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s {
            "hinge" => Some(Self::Hinge),
            "slide" => Some(Self::Slide),
            "ball" => Some(Self::Ball),
            "free" => Some(Self::Free),
            "cylindrical" => Some(Self::Cylindrical),
            "planar" => Some(Self::Planar),
            _ => None,
        }
    }

    /// Get degrees of freedom.
    #[must_use]
    pub fn dof(&self) -> usize {
        match self {
            Self::Hinge | Self::Slide => 1,
            Self::Cylindrical => 2,
            Self::Ball | Self::Planar => 3,
            Self::Free => 6,
        }
    }
}

impl Default for MjcfJointType {
    fn default() -> Self {
        Self::Hinge
    }
}

/// Joint from `<joint>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfJoint {
    /// Joint name (auto-generated if not specified).
    pub name: String,
    /// Default class.
    pub class: Option<String>,
    /// Joint type.
    pub joint_type: Option<MjcfJointType>,
    /// Joint position relative to body frame.
    pub pos: Option<Vector3<f64>>,
    /// Joint axis (for hinge and slide).
    pub axis: Option<Vector3<f64>>,
    /// Whether position limits are enabled. `None` means not explicitly set
    /// (subject to autolimits inference).
    pub limited: Option<bool>,
    /// Position limit range [lower, upper].
    pub range: Option<(f64, f64)>,
    /// Joint reference position.
    pub ref_pos: Option<f64>,
    /// Spring equilibrium position.
    pub spring_ref: Option<f64>,
    /// Damping coefficient.
    pub damping: Option<f64>,
    /// Spring stiffness.
    pub stiffness: Option<f64>,
    /// Armature (rotor inertia) - added to diagonal of inertia matrix.
    pub armature: Option<f64>,
    /// Friction loss.
    pub frictionloss: Option<f64>,
    /// Visualization group (0–5). Default 0.
    pub group: Option<i32>,
    /// Solver reference parameters for joint limits [timeconst, dampratio].
    /// Controls how stiffly/softly limits are enforced.
    pub solref_limit: Option<[f64; 2]>,
    /// Solver impedance parameters for joint limits [d0, d_width, width, midpoint, power].
    pub solimp_limit: Option<[f64; 5]>,
    /// Solver reference parameters for friction loss [timeconst, dampratio].
    pub solreffriction: Option<[f64; 2]>,
    /// Solver impedance parameters for friction loss [d0, d_width, width, midpoint, power].
    pub solimpfriction: Option<[f64; 5]>,
    /// If true, gravcomp routes through `qfrc_actuator` instead of `qfrc_passive`.
    pub actuatorgravcomp: Option<bool>,
    /// Body this joint belongs to (set during parsing).
    pub body: Option<String>,
}

impl Default for MjcfJoint {
    fn default() -> Self {
        Self {
            name: String::new(),
            class: None,
            joint_type: None,
            pos: None,
            axis: None,
            limited: None,
            range: None,
            ref_pos: None,
            spring_ref: None,
            damping: None,
            stiffness: None,
            armature: None,
            frictionloss: None,
            group: None,
            solref_limit: None,
            solimp_limit: None,
            solreffriction: None,
            solimpfriction: None,
            actuatorgravcomp: None,
            body: None,
        }
    }
}

impl MjcfJoint {
    /// Create a new hinge joint.
    #[must_use]
    pub fn hinge(name: impl Into<String>, axis: Vector3<f64>) -> Self {
        Self {
            name: name.into(),
            joint_type: Some(MjcfJointType::Hinge),
            axis: Some(axis),
            ..Default::default()
        }
    }

    /// Create a new slide joint.
    #[must_use]
    pub fn slide(name: impl Into<String>, axis: Vector3<f64>) -> Self {
        Self {
            name: name.into(),
            joint_type: Some(MjcfJointType::Slide),
            axis: Some(axis),
            ..Default::default()
        }
    }

    /// Set position limits.
    #[must_use]
    pub fn with_limits(mut self, lower: f64, upper: f64) -> Self {
        self.limited = Some(true);
        self.range = Some((lower, upper));
        self
    }

    /// Set damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = Some(damping);
        self
    }
}

// ============================================================================
// Site (Marker)
// ============================================================================

/// Site from `<site>` element.
///
/// Sites are markers/attachment points that don't participate in physics.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfSite {
    /// Site name.
    pub name: String,
    /// Default class.
    pub class: Option<String>,
    /// Site type (sphere, capsule, ellipsoid, cylinder, box).
    pub site_type: Option<String>,
    /// Position relative to body frame.
    pub pos: Option<Vector3<f64>>,
    /// Orientation (quaternion: w x y z).
    pub quat: Option<Vector4<f64>>,
    /// Alternative orientation as euler angles.
    pub euler: Option<Vector3<f64>>,
    /// Alternative orientation as axis-angle (x, y, z, angle).
    pub axisangle: Option<Vector4<f64>>,
    /// Alternative orientation as two axes (x-axis 3 floats, y-axis 3 floats).
    pub xyaxes: Option<[f64; 6]>,
    /// Alternative orientation as z-axis (minimal rotation from Z).
    pub zaxis: Option<Vector3<f64>>,
    /// Size (for visualization).
    pub size: Option<Vec<f64>>,
    /// RGBA color.
    pub rgba: Option<Vector4<f64>>,
    /// Visualization group (0–5). Default 0.
    pub group: Option<i32>,
    /// Material asset name (for rendering).
    pub material: Option<String>,
}

impl Default for MjcfSite {
    fn default() -> Self {
        Self {
            name: String::new(),
            class: None,
            site_type: None,
            pos: None,
            quat: None,
            euler: None,
            axisangle: None,
            xyaxes: None,
            zaxis: None,
            size: None,
            rgba: None,
            group: None,
            material: None,
        }
    }
}

// ============================================================================
// Equality Constraints
// ============================================================================

/// Connect (ball) equality constraint from `<connect>` element.
///
/// This constraint enforces that two attachment points (one on each body)
/// coincide in 3D space, acting like a ball-and-socket joint without any
/// rotational constraints.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfConnect {
    /// Optional constraint name.
    pub name: Option<String>,
    /// Default class for inheriting parameters.
    pub class: Option<String>,
    /// Name of the first body.
    pub body1: String,
    /// Name of the second body (optional, defaults to world).
    pub body2: Option<String>,
    /// Anchor point in body1's local frame.
    pub anchor: Vector3<f64>,
    /// Solver impedance parameters [dmin, dmax, width, midpoint, power].
    /// Controls constraint softness/stiffness.
    pub solimp: Option<[f64; 5]>,
    /// Solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    /// Controls constraint dynamics.
    pub solref: Option<[f64; 2]>,
    /// Whether this constraint is active.
    pub active: bool,
}

impl Default for MjcfConnect {
    fn default() -> Self {
        Self {
            name: None,
            class: None,
            body1: String::new(),
            body2: None,
            anchor: Vector3::zeros(),
            solimp: None,
            solref: None,
            active: true,
        }
    }
}

impl MjcfConnect {
    /// Create a new connect constraint between two bodies.
    #[must_use]
    pub fn new(body1: impl Into<String>, body2: impl Into<String>) -> Self {
        Self {
            body1: body1.into(),
            body2: Some(body2.into()),
            ..Default::default()
        }
    }

    /// Create a connect constraint to the world frame.
    #[must_use]
    pub fn to_world(body1: impl Into<String>) -> Self {
        Self {
            body1: body1.into(),
            body2: None,
            ..Default::default()
        }
    }

    /// Set the anchor point in body1's frame.
    #[must_use]
    pub fn with_anchor(mut self, anchor: Vector3<f64>) -> Self {
        self.anchor = anchor;
        self
    }

    /// Set the constraint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set solver reference parameters.
    #[must_use]
    pub fn with_solref(mut self, solref: [f64; 2]) -> Self {
        self.solref = Some(solref);
        self
    }

    /// Set solver impedance parameters.
    #[must_use]
    pub fn with_solimp(mut self, solimp: [f64; 5]) -> Self {
        self.solimp = Some(solimp);
        self
    }
}

/// A weld constraint from `<weld>` element.
///
/// Welds two bodies together with a fixed relative pose (position and orientation).
/// This creates a 6 DOF constraint locking both translation and rotation.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfWeld {
    /// Optional constraint name.
    pub name: Option<String>,
    /// Default class for inheriting parameters.
    pub class: Option<String>,
    /// Name of the first body.
    pub body1: String,
    /// Name of the second body (optional, defaults to world).
    pub body2: Option<String>,
    /// Anchor point in body1's local frame.
    pub anchor: Vector3<f64>,
    /// Relative position offset between the two anchor frames.
    pub relpose: Option<[f64; 7]>,
    /// Solver impedance parameters [dmin, dmax, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
    /// Solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    pub solref: Option<[f64; 2]>,
    /// Whether this constraint is active.
    pub active: bool,
}

impl Default for MjcfWeld {
    fn default() -> Self {
        Self {
            name: None,
            class: None,
            body1: String::new(),
            body2: None,
            anchor: Vector3::zeros(),
            relpose: None,
            solimp: None,
            solref: None,
            active: true,
        }
    }
}

impl MjcfWeld {
    /// Create a new weld constraint between two bodies.
    #[must_use]
    pub fn new(body1: impl Into<String>, body2: impl Into<String>) -> Self {
        Self {
            body1: body1.into(),
            body2: Some(body2.into()),
            ..Default::default()
        }
    }

    /// Create a weld constraint to the world frame.
    #[must_use]
    pub fn to_world(body1: impl Into<String>) -> Self {
        Self {
            body1: body1.into(),
            body2: None,
            ..Default::default()
        }
    }

    /// Set the anchor point in body1's frame.
    #[must_use]
    pub fn with_anchor(mut self, anchor: Vector3<f64>) -> Self {
        self.anchor = anchor;
        self
    }

    /// Set the relative pose [x, y, z, qw, qx, qy, qz].
    #[must_use]
    pub fn with_relpose(mut self, relpose: [f64; 7]) -> Self {
        self.relpose = Some(relpose);
        self
    }

    /// Set the constraint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set solver reference parameters.
    #[must_use]
    pub fn with_solref(mut self, solref: [f64; 2]) -> Self {
        self.solref = Some(solref);
        self
    }

    /// Set solver impedance parameters.
    #[must_use]
    pub fn with_solimp(mut self, solimp: [f64; 5]) -> Self {
        self.solimp = Some(solimp);
        self
    }
}

/// A joint equality constraint from `<joint>` element within `<equality>`.
///
/// Constrains a joint to maintain a specific position, or couples two joints
/// with a linear relationship: `q2 = polycoef\[0\] + polycoef\[1\]*q1 + ...`
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfJointEquality {
    /// Optional constraint name.
    pub name: Option<String>,
    /// Default class for inheriting parameters.
    pub class: Option<String>,
    /// Name of the first (primary) joint.
    pub joint1: String,
    /// Name of the second joint (optional, for coupling).
    pub joint2: Option<String>,
    /// Polynomial coefficients for coupling: `q2 = sum(polycoef\[i\] * q1^i)`.
    /// Default is `[0, 1]` meaning `q2 = q1` (mimic).
    pub polycoef: Vec<f64>,
    /// Solver impedance parameters [dmin, dmax, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
    /// Solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    pub solref: Option<[f64; 2]>,
    /// Whether this constraint is active.
    pub active: bool,
}

impl Default for MjcfJointEquality {
    fn default() -> Self {
        Self {
            name: None,
            class: None,
            joint1: String::new(),
            joint2: None,
            polycoef: vec![0.0, 1.0], // Default: q2 = q1
            solimp: None,
            solref: None,
            active: true,
        }
    }
}

impl MjcfJointEquality {
    /// Create a joint position lock constraint.
    #[must_use]
    pub fn lock(joint: impl Into<String>) -> Self {
        Self {
            joint1: joint.into(),
            joint2: None,
            polycoef: vec![0.0], // Lock at position 0
            ..Default::default()
        }
    }

    /// Create a joint position lock at a specific position.
    #[must_use]
    pub fn lock_at(joint: impl Into<String>, position: f64) -> Self {
        Self {
            joint1: joint.into(),
            joint2: None,
            polycoef: vec![position],
            ..Default::default()
        }
    }

    /// Create a mimic constraint (joint2 = joint1).
    #[must_use]
    pub fn mimic(joint1: impl Into<String>, joint2: impl Into<String>) -> Self {
        Self {
            joint1: joint1.into(),
            joint2: Some(joint2.into()),
            polycoef: vec![0.0, 1.0],
            ..Default::default()
        }
    }

    /// Create a gear constraint (joint2 = ratio * joint1).
    #[must_use]
    pub fn gear(joint1: impl Into<String>, joint2: impl Into<String>, ratio: f64) -> Self {
        Self {
            joint1: joint1.into(),
            joint2: Some(joint2.into()),
            polycoef: vec![0.0, ratio],
            ..Default::default()
        }
    }

    /// Set the constraint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set polynomial coefficients.
    #[must_use]
    pub fn with_polycoef(mut self, polycoef: Vec<f64>) -> Self {
        self.polycoef = polycoef;
        self
    }

    /// Set solver reference parameters.
    #[must_use]
    pub fn with_solref(mut self, solref: [f64; 2]) -> Self {
        self.solref = Some(solref);
        self
    }

    /// Set solver impedance parameters.
    #[must_use]
    pub fn with_solimp(mut self, solimp: [f64; 5]) -> Self {
        self.solimp = Some(solimp);
        self
    }
}

/// A distance constraint from `<distance>` element.
///
/// Maintains a fixed distance between two anchor points on different bodies.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfDistance {
    /// Optional constraint name.
    pub name: Option<String>,
    /// Default class for inheriting parameters.
    pub class: Option<String>,
    /// Name of the first geom (anchor derived from geom position).
    pub geom1: String,
    /// Name of the second geom (optional, defaults to world origin).
    pub geom2: Option<String>,
    /// Target distance between geom centers.
    pub distance: Option<f64>,
    /// Solver impedance parameters [dmin, dmax, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
    /// Solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    pub solref: Option<[f64; 2]>,
    /// Whether this constraint is active.
    pub active: bool,
}

impl Default for MjcfDistance {
    fn default() -> Self {
        Self {
            name: None,
            class: None,
            geom1: String::new(),
            geom2: None,
            distance: None, // Computed from initial configuration
            solimp: None,
            solref: None,
            active: true,
        }
    }
}

impl MjcfDistance {
    /// Create a distance constraint between two geoms.
    #[must_use]
    pub fn new(geom1: impl Into<String>, geom2: impl Into<String>) -> Self {
        Self {
            geom1: geom1.into(),
            geom2: Some(geom2.into()),
            ..Default::default()
        }
    }

    /// Create a distance constraint from a geom to the world origin.
    #[must_use]
    pub fn to_world(geom1: impl Into<String>) -> Self {
        Self {
            geom1: geom1.into(),
            geom2: None,
            ..Default::default()
        }
    }

    /// Set the target distance.
    #[must_use]
    pub fn with_distance(mut self, distance: f64) -> Self {
        self.distance = Some(distance);
        self
    }

    /// Set the constraint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set solver reference parameters.
    #[must_use]
    pub fn with_solref(mut self, solref: [f64; 2]) -> Self {
        self.solref = Some(solref);
        self
    }

    /// Set solver impedance parameters.
    #[must_use]
    pub fn with_solimp(mut self, solimp: [f64; 5]) -> Self {
        self.solimp = Some(solimp);
        self
    }
}

/// A tendon equality constraint from `<tendon>` element within `<equality>`.
///
/// Constrains a tendon to its reference length, or couples two tendons
/// with a polynomial relationship: `(L1-L1_0) = data[0] + P(L2-L2_0)`.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfTendonEquality {
    /// Optional constraint name.
    pub name: Option<String>,
    /// Default class for inheriting parameters.
    pub class: Option<String>,
    /// Name of the first (primary) tendon.
    pub tendon1: String,
    /// Name of the second tendon (optional, for coupling).
    pub tendon2: Option<String>,
    /// Polynomial coefficients: `[offset, c1, c2, c3, c4]`.
    /// Default is `[0, 1]` meaning equal deviation from reference (trailing zeros implicit).
    pub polycoef: Vec<f64>,
    /// Solver impedance parameters [dmin, dmax, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
    /// Solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    pub solref: Option<[f64; 2]>,
    /// Whether this constraint is active.
    pub active: bool,
}

impl Default for MjcfTendonEquality {
    fn default() -> Self {
        Self {
            name: None,
            class: None,
            tendon1: String::new(),
            tendon2: None,
            polycoef: vec![0.0, 1.0], // Default: equal deviation from reference
            solimp: None,
            solref: None,
            active: true,
        }
    }
}

impl MjcfTendonEquality {
    /// Create a tendon coupling constraint (both deviate equally from reference).
    #[must_use]
    pub fn couple(tendon1: impl Into<String>, tendon2: impl Into<String>) -> Self {
        Self {
            tendon1: tendon1.into(),
            tendon2: Some(tendon2.into()),
            polycoef: vec![0.0, 1.0],
            ..Default::default()
        }
    }

    /// Create a single-tendon constraint (lock to reference length + offset).
    #[must_use]
    pub fn lock(tendon: impl Into<String>, offset: f64) -> Self {
        Self {
            tendon1: tendon.into(),
            tendon2: None,
            polycoef: vec![offset],
            ..Default::default()
        }
    }

    /// Set the constraint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set polynomial coefficients.
    #[must_use]
    pub fn with_polycoef(mut self, polycoef: Vec<f64>) -> Self {
        self.polycoef = polycoef;
        self
    }

    /// Set solver reference parameters.
    #[must_use]
    pub fn with_solref(mut self, solref: [f64; 2]) -> Self {
        self.solref = Some(solref);
        self
    }

    /// Set solver impedance parameters.
    #[must_use]
    pub fn with_solimp(mut self, solimp: [f64; 5]) -> Self {
        self.solimp = Some(solimp);
        self
    }
}

/// Container for equality constraints from `<equality>` element.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfEquality {
    /// Connect (ball-and-socket) constraints.
    pub connects: Vec<MjcfConnect>,
    /// Weld (6 DOF lock) constraints.
    pub welds: Vec<MjcfWeld>,
    /// Joint equality constraints (position lock or coupling).
    pub joints: Vec<MjcfJointEquality>,
    /// Distance constraints between geoms.
    pub distances: Vec<MjcfDistance>,
    /// Tendon equality constraints.
    pub tendons: Vec<MjcfTendonEquality>,
}

impl MjcfEquality {
    /// Create a new empty equality constraint container.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a connect constraint.
    #[must_use]
    pub fn with_connect(mut self, connect: MjcfConnect) -> Self {
        self.connects.push(connect);
        self
    }

    /// Add a weld constraint.
    #[must_use]
    pub fn with_weld(mut self, weld: MjcfWeld) -> Self {
        self.welds.push(weld);
        self
    }

    /// Add a joint equality constraint.
    #[must_use]
    pub fn with_joint(mut self, joint: MjcfJointEquality) -> Self {
        self.joints.push(joint);
        self
    }

    /// Add a distance constraint.
    #[must_use]
    pub fn with_distance(mut self, distance: MjcfDistance) -> Self {
        self.distances.push(distance);
        self
    }

    /// Add a tendon equality constraint.
    #[must_use]
    pub fn with_tendon(mut self, tendon: MjcfTendonEquality) -> Self {
        self.tendons.push(tendon);
        self
    }

    /// Check if there are any equality constraints.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.connects.is_empty()
            && self.welds.is_empty()
            && self.joints.is_empty()
            && self.distances.is_empty()
            && self.tendons.is_empty()
    }

    /// Get the total number of equality constraints.
    #[must_use]
    pub fn len(&self) -> usize {
        self.connects.len()
            + self.welds.len()
            + self.joints.len()
            + self.distances.len()
            + self.tendons.len()
    }
}

// ============================================================================
// Frame
// ============================================================================

/// A coordinate frame transformation from `<frame>` element.
///
/// Frames provide local coordinate system offsets for child elements.
/// All children have their pos/quat interpreted relative to the frame,
/// not the parent body. Frames disappear during model building — their
/// transforms are composed into their children's positions/orientations.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfFrame {
    /// Optional name (not preserved in compiled model).
    pub name: Option<String>,
    /// Position offset relative to parent body/frame.
    pub pos: Vector3<f64>,
    /// Orientation quaternion (w, x, y, z).
    pub quat: Vector4<f64>,
    /// Alternative orientation as axis-angle (x, y, z, angle).
    pub axisangle: Option<Vector4<f64>>,
    /// Alternative orientation as euler angles.
    pub euler: Option<Vector3<f64>>,
    /// Alternative orientation as two axes (x-axis 3 floats, y-axis 3 floats).
    pub xyaxes: Option<[f64; 6]>,
    /// Alternative orientation as z-axis (minimal rotation from Z).
    pub zaxis: Option<Vector3<f64>>,
    /// Default class for child elements.
    pub childclass: Option<String>,
    /// Child bodies within this frame.
    pub bodies: Vec<MjcfBody>,
    /// Child geoms within this frame.
    pub geoms: Vec<MjcfGeom>,
    /// Child sites within this frame.
    pub sites: Vec<MjcfSite>,
    /// Nested child frames.
    pub frames: Vec<MjcfFrame>,
}

impl Default for MjcfFrame {
    fn default() -> Self {
        Self {
            name: None,
            pos: Vector3::zeros(),
            quat: Vector4::new(1.0, 0.0, 0.0, 0.0),
            axisangle: None,
            euler: None,
            xyaxes: None,
            zaxis: None,
            childclass: None,
            bodies: Vec::new(),
            geoms: Vec::new(),
            sites: Vec::new(),
            frames: Vec::new(),
        }
    }
}

// ============================================================================
// Body
// ============================================================================

/// A body from `<body>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfBody {
    /// Body name.
    pub name: String,
    /// Position relative to parent body frame.
    pub pos: Vector3<f64>,
    /// Orientation (quaternion: w x y z).
    pub quat: Vector4<f64>,
    /// Alternative orientation as axis-angle.
    pub axisangle: Option<Vector4<f64>>,
    /// Alternative orientation as euler angles (xyz).
    pub euler: Option<Vector3<f64>>,
    /// Explicit inertial properties.
    pub inertial: Option<MjcfInertial>,
    /// Joints attached to this body.
    pub joints: Vec<MjcfJoint>,
    /// Geoms attached to this body.
    pub geoms: Vec<MjcfGeom>,
    /// Sites attached to this body.
    pub sites: Vec<MjcfSite>,
    /// Frames within this body (expanded during model building).
    pub frames: Vec<MjcfFrame>,
    /// Child bodies.
    pub children: Vec<MjcfBody>,
    /// Parent body name (set during flattening).
    pub parent: Option<String>,
    /// Default class for child elements (geoms, joints, sites).
    pub childclass: Option<String>,
    /// Whether this body is a mocap (kinematic input) body.
    #[cfg_attr(feature = "serde", serde(default))]
    pub mocap: bool,
    /// Sleep policy override for this body's kinematic tree ("auto"|"allowed"|"never"|"init").
    pub sleep: Option<String>,
    /// Gravity compensation factor (0=none, 1=full, >1=over-compensate, <0=amplify gravity).
    pub gravcomp: Option<f64>,
}

impl Default for MjcfBody {
    fn default() -> Self {
        Self {
            name: String::new(),
            pos: Vector3::zeros(),
            quat: Vector4::new(1.0, 0.0, 0.0, 0.0),
            axisangle: None,
            euler: None,
            inertial: None,
            joints: Vec::new(),
            geoms: Vec::new(),
            sites: Vec::new(),
            frames: Vec::new(),
            children: Vec::new(),
            parent: None,
            childclass: None,
            mocap: false,
            sleep: None,
            gravcomp: None,
        }
    }
}

impl MjcfBody {
    /// Create a new body with a name.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            ..Default::default()
        }
    }

    /// Set position.
    #[must_use]
    pub fn with_pos(mut self, x: f64, y: f64, z: f64) -> Self {
        self.pos = Vector3::new(x, y, z);
        self
    }

    /// Add a geom.
    #[must_use]
    pub fn with_geom(mut self, geom: MjcfGeom) -> Self {
        self.geoms.push(geom);
        self
    }

    /// Add a joint.
    #[must_use]
    pub fn with_joint(mut self, joint: MjcfJoint) -> Self {
        self.joints.push(joint);
        self
    }

    /// Add a child body.
    #[must_use]
    pub fn with_child(mut self, child: MjcfBody) -> Self {
        self.children.push(child);
        self
    }

    /// Get rotation as `UnitQuaternion`.
    #[must_use]
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        if let Some(euler) = self.euler {
            // XYZ Euler angles
            UnitQuaternion::from_euler_angles(euler.x, euler.y, euler.z)
        } else if let Some(aa) = self.axisangle {
            // Axis-angle: [x, y, z, angle]
            let axis = nalgebra::Unit::new_normalize(Vector3::new(aa.x, aa.y, aa.z));
            UnitQuaternion::from_axis_angle(&axis, aa.w)
        } else {
            // Quaternion (w, x, y, z)
            let q =
                nalgebra::Quaternion::new(self.quat[0], self.quat[1], self.quat[2], self.quat[3]);
            UnitQuaternion::from_quaternion(q)
        }
    }

    /// Get position as Point3.
    #[must_use]
    pub fn position(&self) -> Point3<f64> {
        Point3::from(self.pos)
    }
}

// ============================================================================
// Actuator Types
// ============================================================================

/// Actuator type from MJCF.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MjcfActuatorType {
    /// Direct force/torque motor.
    Motor,
    /// Position servo (PD control).
    Position,
    /// Velocity servo.
    Velocity,
    /// General actuator with custom dynamics.
    General,
    /// Muscle actuator (Hill-type muscle model).
    Muscle,
    /// Cylinder (pneumatic/hydraulic actuator).
    Cylinder,
    /// Damper (passive damping).
    Damper,
    /// Adhesion actuator (controllable adhesion force).
    Adhesion,
}

impl MjcfActuatorType {
    /// Parse actuator type from element name.
    pub fn from_str(s: &str) -> Option<Self> {
        match s {
            "motor" => Some(Self::Motor),
            "position" => Some(Self::Position),
            "velocity" => Some(Self::Velocity),
            "general" => Some(Self::General),
            "muscle" => Some(Self::Muscle),
            "cylinder" => Some(Self::Cylinder),
            "damper" => Some(Self::Damper),
            "adhesion" => Some(Self::Adhesion),
            _ => None,
        }
    }

    /// Get the MJCF string representation.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Motor => "motor",
            Self::Position => "position",
            Self::Velocity => "velocity",
            Self::General => "general",
            Self::Muscle => "muscle",
            Self::Cylinder => "cylinder",
            Self::Damper => "damper",
            Self::Adhesion => "adhesion",
        }
    }
}

/// Actuator from `<actuator>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfActuator {
    /// Actuator name.
    pub name: String,
    /// Default class.
    pub class: Option<String>,
    /// Actuator type.
    pub actuator_type: MjcfActuatorType,
    /// Target joint name.
    pub joint: Option<String>,
    /// Target site name (for site actuators).
    pub site: Option<String>,
    /// Target tendon name.
    pub tendon: Option<String>,
    /// Target body name (for adhesion actuators).
    pub body: Option<String>,
    /// Reference site name (for site transmissions with Cartesian control).
    pub refsite: Option<String>,
    /// Gear ratio (6D: [tx ty tz rx ry rz]).
    pub gear: [f64; 6],
    /// Control range [lower, upper].
    pub ctrlrange: Option<(f64, f64)>,
    /// Force range [lower, upper].
    pub forcerange: Option<(f64, f64)>,
    /// Whether control is clamped to range. `None` means not explicitly set
    /// (subject to autolimits inference).
    pub ctrllimited: Option<bool>,
    /// Whether force is clamped to range. `None` means not explicitly set
    /// (subject to autolimits inference).
    pub forcelimited: Option<bool>,
    /// Position gain (for position actuators).
    pub kp: f64,
    /// Velocity gain. `None` means use actuator-type default.
    pub kv: Option<f64>,
    /// Visualization group (0–5).
    pub group: Option<i32>,
    /// Whether activation is clamped to actrange.
    pub actlimited: Option<bool>,
    /// Activation clamping range [lower, upper].
    pub actrange: Option<(f64, f64)>,
    /// If true, activation dynamics are applied early (before constraint).
    pub actearly: Option<bool>,
    /// Length range for the actuator.
    pub lengthrange: Option<(f64, f64)>,

    // ========================================================================
    // Cylinder-specific attributes
    // ========================================================================
    /// Cylinder cross-sectional area (m²). Used as gain for cylinder actuators.
    pub area: f64,
    /// Cylinder diameter (m). Alternative to area; takes precedence if set.
    pub diameter: Option<f64>,
    /// Activation dynamics time constant (s). `None` means use actuator-type default.
    pub timeconst: Option<f64>,
    /// Bias parameters [prm0, prm1, prm2] for cylinder.
    pub bias: [f64; 3],

    // ========================================================================
    // Muscle-specific attributes
    // ========================================================================
    /// Activation/deactivation time constants [act, deact] for muscle.
    pub muscle_timeconst: (f64, f64),
    /// Operating length range [lower, upper] in L0 units for muscle.
    pub range: (f64, f64),
    /// Peak active force (N). Negative triggers automatic computation.
    pub force: f64,
    /// Force scaling factor for muscle.
    pub scale: f64,
    /// Lower FLV curve position (L0 units).
    pub lmin: f64,
    /// Upper FLV curve position (L0 units).
    pub lmax: f64,
    /// Shortening velocity limit (L0/second).
    pub vmax: f64,
    /// Passive force at lmax (relative to peak force).
    pub fpmax: f64,
    /// Active force at lengthening (relative to peak force).
    pub fvmax: f64,

    // ========================================================================
    // Adhesion-specific attributes
    // ========================================================================
    /// Gain in force units for adhesion (total force = control × gain).
    pub gain: f64,

    // ========================================================================
    // <general>-specific attributes
    // ========================================================================
    // These are only meaningful for MjcfActuatorType::General.
    // For shortcut types (Motor, Position, etc.), these are ignored —
    // the model builder expands shortcuts using their own dedicated logic.
    /// Gain type: "fixed", "affine", "muscle".
    /// None means use default (Fixed). Only parsed for `<general>`.
    pub gaintype: Option<String>,
    /// Bias type: "none", "affine", "muscle".
    /// None means use default (None/no bias). Only parsed for `<general>`.
    pub biastype: Option<String>,
    /// Dynamics type: "none", "integrator", "filter", "filterexact", "muscle".
    /// None means use default (None/direct). Only parsed for `<general>`.
    pub dyntype: Option<String>,
    /// Gain parameters (up to 9 elements, zero-padded).
    /// None means use default [1, 0, ..., 0]. Only parsed for `<general>`.
    pub gainprm: Option<Vec<f64>>,
    /// Bias parameters (up to 9 elements, zero-padded).
    /// None means use default [0, ..., 0]. Only parsed for `<general>`.
    pub biasprm: Option<Vec<f64>>,
    /// Dynamics parameters (up to 3 elements, zero-padded).
    /// None means use default [1, 0, 0]. Only parsed for `<general>`.
    pub dynprm: Option<Vec<f64>>,
}

impl Default for MjcfActuator {
    fn default() -> Self {
        Self {
            name: String::new(),
            class: None,
            actuator_type: MjcfActuatorType::Motor,
            joint: None,
            site: None,
            tendon: None,
            body: None,
            refsite: None,
            gear: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ctrlrange: None,
            forcerange: None,
            ctrllimited: None,
            forcelimited: None,
            kp: 1.0,
            kv: None,
            group: None,
            actlimited: None,
            actrange: None,
            actearly: None,
            lengthrange: None,
            // Cylinder defaults (MuJoCo defaults)
            area: 1.0,
            diameter: None,
            timeconst: None,
            bias: [0.0, 0.0, 0.0],
            // Muscle defaults (MuJoCo defaults)
            muscle_timeconst: (0.01, 0.04),
            range: (0.75, 1.05),
            force: -1.0, // Negative triggers automatic computation
            scale: 200.0,
            lmin: 0.5,
            lmax: 1.6,
            vmax: 1.5,
            fpmax: 1.3,
            fvmax: 1.2,
            // Adhesion defaults
            gain: 1.0,
            // <general>-specific defaults (all None = use MuJoCo defaults in model builder)
            gaintype: None,
            biastype: None,
            dyntype: None,
            gainprm: None,
            biasprm: None,
            dynprm: None,
        }
    }
}

impl MjcfActuator {
    /// Create a motor actuator for a joint.
    #[must_use]
    pub fn motor(name: impl Into<String>, joint: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            actuator_type: MjcfActuatorType::Motor,
            joint: Some(joint.into()),
            ..Default::default()
        }
    }

    /// Create a position servo for a joint.
    #[must_use]
    pub fn position(name: impl Into<String>, joint: impl Into<String>, kp: f64) -> Self {
        Self {
            name: name.into(),
            actuator_type: MjcfActuatorType::Position,
            joint: Some(joint.into()),
            kp,
            ..Default::default()
        }
    }

    /// Create a velocity servo for a joint.
    #[must_use]
    pub fn velocity(name: impl Into<String>, joint: impl Into<String>, kv: f64) -> Self {
        Self {
            name: name.into(),
            actuator_type: MjcfActuatorType::Velocity,
            joint: Some(joint.into()),
            kv: Some(kv),
            ..Default::default()
        }
    }

    /// Create a cylinder (pneumatic/hydraulic) actuator for a joint.
    #[must_use]
    pub fn cylinder(name: impl Into<String>, joint: impl Into<String>, area: f64) -> Self {
        Self {
            name: name.into(),
            actuator_type: MjcfActuatorType::Cylinder,
            joint: Some(joint.into()),
            area,
            timeconst: Some(1.0),
            ..Default::default()
        }
    }

    /// Create a damper actuator for a joint.
    #[must_use]
    pub fn damper(name: impl Into<String>, joint: impl Into<String>, kv: f64) -> Self {
        Self {
            name: name.into(),
            actuator_type: MjcfActuatorType::Damper,
            joint: Some(joint.into()),
            kv: Some(kv),
            ..Default::default()
        }
    }

    /// Create a muscle actuator for a joint.
    #[must_use]
    pub fn muscle(name: impl Into<String>, joint: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            actuator_type: MjcfActuatorType::Muscle,
            joint: Some(joint.into()),
            ..Default::default()
        }
    }

    /// Create an adhesion actuator for a body.
    #[must_use]
    pub fn adhesion(name: impl Into<String>, body: impl Into<String>, gain: f64) -> Self {
        Self {
            name: name.into(),
            actuator_type: MjcfActuatorType::Adhesion,
            body: Some(body.into()),
            gain,
            ..Default::default()
        }
    }
}

// ============================================================================
// Tendon Types
// ============================================================================

/// Tendon type from MJCF.
///
/// MuJoCo supports both spatial (point-to-point) and fixed-path tendons.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MjcfTendonType {
    /// Spatial tendon (point-to-point through sites).
    #[default]
    Spatial,
    /// Fixed tendon (linear combination of joint positions).
    Fixed,
}

impl MjcfTendonType {
    /// Parse tendon type from element name.
    pub fn from_str(s: &str) -> Option<Self> {
        match s {
            "spatial" => Some(Self::Spatial),
            "fixed" => Some(Self::Fixed),
            _ => None,
        }
    }

    /// Get the MJCF string representation.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Spatial => "spatial",
            Self::Fixed => "fixed",
        }
    }
}

/// A tendon from `<tendon>` element.
///
/// A single element in a spatial tendon path, preserving MJCF ordering.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SpatialPathElement {
    /// Via-point: tendon passes through this site.
    Site {
        /// Site name reference.
        site: String,
    },
    /// Wrapping surface: tendon wraps around this geom (sphere or cylinder only).
    /// `sidesite` determines which side of the geometry the tendon wraps around,
    /// preventing discontinuous jumps when multiple wrapping solutions exist.
    Geom {
        /// Wrapping geom name reference.
        geom: String,
        /// Optional sidesite for wrapping side disambiguation.
        sidesite: Option<String>,
    },
    /// Pulley: scales subsequent path length and Jacobian contributions by
    /// 1/divisor until the next pulley element or end of tendon.
    Pulley {
        /// Pulley divisor (must be positive).
        divisor: f64,
    },
}

/// MuJoCo tendon specification.
///
/// Tendons connect multiple joints or sites and apply forces based on their
/// length. Used for modeling cables, tendons, or force transmission mechanisms.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfTendon {
    /// Tendon name.
    pub name: String,
    /// Default class.
    pub class: Option<String>,
    /// Tendon type.
    pub tendon_type: MjcfTendonType,
    /// Range limits [lower, upper] for tendon length.
    pub range: Option<(f64, f64)>,
    /// Whether range limits are enabled. `None` means not explicitly set
    /// (subject to autolimits inference).
    pub limited: Option<bool>,
    /// Stiffness coefficient.
    pub stiffness: Option<f64>,
    /// Damping coefficient.
    pub damping: Option<f64>,
    /// Friction loss.
    pub frictionloss: Option<f64>,
    /// Tendon width for visualization.
    pub width: Option<f64>,
    /// RGBA color for visualization.
    pub rgba: Option<Vector4<f64>>,
    /// Visualization group (0–5). Default 0.
    pub group: Option<i32>,
    /// Solver reference parameters for tendon limits [timeconst, dampratio].
    pub solref: Option<[f64; 2]>,
    /// Solver impedance parameters for tendon limits [d0, d_width, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
    /// Solver reference parameters for friction loss [timeconst, dampratio].
    pub solreffriction: Option<[f64; 2]>,
    /// Solver impedance parameters for friction loss [d0, d_width, width, midpoint, power].
    pub solimpfriction: Option<[f64; 5]>,
    /// Contact margin for tendon limits.
    pub margin: Option<f64>,
    /// Material asset name (for rendering).
    pub material: Option<String>,
    /// Spring rest length pair (low, high) for deadband spring.
    /// `None` = auto-compute from tendon length at qpos0 (see S3 divergence note).
    /// Single value in MJCF → both elements equal (no deadband).
    /// When low < high, force is zero within [low, high].
    pub springlength: Option<(f64, f64)>,
    /// Ordered path elements for spatial tendons (sites, wrapping geoms, pulleys).
    pub path_elements: Vec<SpatialPathElement>,
    /// Joint coefficients for fixed tendons: (joint_name, coefficient).
    pub joints: Vec<(String, f64)>,
}

impl Default for MjcfTendon {
    fn default() -> Self {
        Self {
            name: String::new(),
            class: None,
            tendon_type: MjcfTendonType::default(),
            range: None,
            limited: None,
            stiffness: None,
            damping: None,
            frictionloss: None,
            width: None,
            rgba: None,
            group: None,
            solref: None,
            solimp: None,
            solreffriction: None,
            solimpfriction: None,
            margin: None,
            material: None,
            springlength: None,
            path_elements: Vec::new(),
            joints: Vec::new(),
        }
    }
}

impl MjcfTendon {
    /// Create a new spatial tendon.
    #[must_use]
    pub fn spatial(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            tendon_type: MjcfTendonType::Spatial,
            ..Default::default()
        }
    }

    /// Create a new fixed tendon.
    #[must_use]
    pub fn fixed(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            tendon_type: MjcfTendonType::Fixed,
            ..Default::default()
        }
    }

    /// Add a site path element to a spatial tendon.
    #[must_use]
    pub fn with_site(mut self, site: impl Into<String>) -> Self {
        self.path_elements
            .push(SpatialPathElement::Site { site: site.into() });
        self
    }

    /// Add a joint with coefficient to a fixed tendon.
    #[must_use]
    pub fn with_joint(mut self, joint: impl Into<String>, coef: f64) -> Self {
        self.joints.push((joint.into(), coef));
        self
    }

    /// Set tendon stiffness.
    #[must_use]
    pub fn with_stiffness(mut self, stiffness: f64) -> Self {
        self.stiffness = Some(stiffness);
        self
    }

    /// Set tendon damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = Some(damping);
        self
    }

    /// Set range limits.
    #[must_use]
    pub fn with_limits(mut self, lower: f64, upper: f64) -> Self {
        self.limited = Some(true);
        self.range = Some((lower, upper));
        self
    }

    /// Add a wrapping geom path element to a spatial tendon.
    #[must_use]
    pub fn with_wrapping_geom(mut self, geom: impl Into<String>) -> Self {
        self.path_elements.push(SpatialPathElement::Geom {
            geom: geom.into(),
            sidesite: None,
        });
        self
    }

    /// Add a wrapping geom path element with sidesite to a spatial tendon.
    #[must_use]
    pub fn with_wrapping_geom_sidesite(
        mut self,
        geom: impl Into<String>,
        sidesite: impl Into<String>,
    ) -> Self {
        self.path_elements.push(SpatialPathElement::Geom {
            geom: geom.into(),
            sidesite: Some(sidesite.into()),
        });
        self
    }

    /// Add a pulley path element to a spatial tendon.
    #[must_use]
    pub fn with_pulley(mut self, divisor: f64) -> Self {
        self.path_elements
            .push(SpatialPathElement::Pulley { divisor });
        self
    }
}

// ============================================================================
// Sensor Types
// ============================================================================

/// Sensor type from MJCF.
///
/// MuJoCo supports many sensor types that measure different quantities.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MjcfSensorType {
    // Position sensors
    /// Joint position sensor.
    #[default]
    Jointpos,
    /// Tendon length sensor.
    Tendonpos,
    /// Actuator position sensor.
    Actuatorpos,
    /// Ball joint quaternion sensor.
    Ballquat,
    /// Frame position sensor.
    Framepos,
    /// Frame orientation quaternion sensor.
    Framequat,
    /// Frame x-axis sensor.
    Framexaxis,
    /// Frame y-axis sensor.
    Frameyaxis,
    /// Frame z-axis sensor.
    Framezaxis,

    // Velocity sensors
    /// Joint velocity sensor.
    Jointvel,
    /// Tendon velocity sensor.
    Tendonvel,
    /// Actuator velocity sensor.
    Actuatorvel,
    /// Ball joint angular velocity sensor.
    Ballangvel,
    /// Frame linear velocity sensor.
    Framelinvel,
    /// Frame angular velocity sensor.
    Frameangvel,

    // Force sensors
    /// Actuator force sensor.
    Actuatorfrc,
    /// Joint limit force sensor.
    Jointlimitfrc,
    /// Tendon limit force sensor.
    Tendonlimitfrc,
    /// Contact force sensor.
    Touch,
    /// Force sensor (measures force on a site).
    Force,
    /// Torque sensor (measures torque on a site).
    Torque,

    // Acceleration sensors
    /// Accelerometer.
    Accelerometer,
    /// Gyroscope.
    Gyro,
    /// Velocimeter (linear velocity, 3D).
    Velocimeter,
    /// Magnetometer (magnetic field, 3D).
    Magnetometer,
    /// Rangefinder (distance to nearest surface, 1D).
    Rangefinder,
    /// Subtree center of mass (3D).
    Subtreecom,
    /// Subtree linear velocity/momentum (3D).
    Subtreelinvel,
    /// Subtree angular momentum (3D).
    Subtreeangmom,
    /// Frame linear acceleration (3D).
    Framelinacc,
    /// Frame angular acceleration (3D).
    Frameangacc,

    // User-defined sensors
    /// User-defined sensor.
    User,
}

impl MjcfSensorType {
    /// Parse sensor type from element name.
    pub fn from_str(s: &str) -> Option<Self> {
        match s {
            "jointpos" => Some(Self::Jointpos),
            "tendonpos" => Some(Self::Tendonpos),
            "actuatorpos" => Some(Self::Actuatorpos),
            "ballquat" => Some(Self::Ballquat),
            "framepos" => Some(Self::Framepos),
            "framequat" => Some(Self::Framequat),
            "framexaxis" => Some(Self::Framexaxis),
            "frameyaxis" => Some(Self::Frameyaxis),
            "framezaxis" => Some(Self::Framezaxis),
            "jointvel" => Some(Self::Jointvel),
            "tendonvel" => Some(Self::Tendonvel),
            "actuatorvel" => Some(Self::Actuatorvel),
            "ballangvel" => Some(Self::Ballangvel),
            "framelinvel" => Some(Self::Framelinvel),
            "frameangvel" => Some(Self::Frameangvel),
            "actuatorfrc" => Some(Self::Actuatorfrc),
            "jointlimitfrc" => Some(Self::Jointlimitfrc),
            "tendonlimitfrc" => Some(Self::Tendonlimitfrc),
            "touch" => Some(Self::Touch),
            "force" => Some(Self::Force),
            "torque" => Some(Self::Torque),
            "accelerometer" => Some(Self::Accelerometer),
            "gyro" => Some(Self::Gyro),
            "velocimeter" => Some(Self::Velocimeter),
            "magnetometer" => Some(Self::Magnetometer),
            "rangefinder" => Some(Self::Rangefinder),
            "subtreecom" => Some(Self::Subtreecom),
            "subtreelinvel" => Some(Self::Subtreelinvel),
            "subtreeangmom" => Some(Self::Subtreeangmom),
            "framelinacc" => Some(Self::Framelinacc),
            "frameangacc" => Some(Self::Frameangacc),
            "user" => Some(Self::User),
            _ => None,
        }
    }

    /// Get the MJCF string representation.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Jointpos => "jointpos",
            Self::Tendonpos => "tendonpos",
            Self::Actuatorpos => "actuatorpos",
            Self::Ballquat => "ballquat",
            Self::Framepos => "framepos",
            Self::Framequat => "framequat",
            Self::Framexaxis => "framexaxis",
            Self::Frameyaxis => "frameyaxis",
            Self::Framezaxis => "framezaxis",
            Self::Jointvel => "jointvel",
            Self::Tendonvel => "tendonvel",
            Self::Actuatorvel => "actuatorvel",
            Self::Ballangvel => "ballangvel",
            Self::Framelinvel => "framelinvel",
            Self::Frameangvel => "frameangvel",
            Self::Actuatorfrc => "actuatorfrc",
            Self::Jointlimitfrc => "jointlimitfrc",
            Self::Tendonlimitfrc => "tendonlimitfrc",
            Self::Touch => "touch",
            Self::Force => "force",
            Self::Torque => "torque",
            Self::Accelerometer => "accelerometer",
            Self::Gyro => "gyro",
            Self::Velocimeter => "velocimeter",
            Self::Magnetometer => "magnetometer",
            Self::Rangefinder => "rangefinder",
            Self::Subtreecom => "subtreecom",
            Self::Subtreelinvel => "subtreelinvel",
            Self::Subtreeangmom => "subtreeangmom",
            Self::Framelinacc => "framelinacc",
            Self::Frameangacc => "frameangacc",
            Self::User => "user",
        }
    }

    /// Get the dimensionality of this sensor's output.
    #[must_use]
    #[allow(clippy::match_same_arms)] // Keep arms separate for documentation clarity
    pub fn dim(self) -> usize {
        match self {
            Self::Jointpos
            | Self::Tendonpos
            | Self::Actuatorpos
            | Self::Jointvel
            | Self::Tendonvel
            | Self::Actuatorvel
            | Self::Actuatorfrc
            | Self::Jointlimitfrc
            | Self::Tendonlimitfrc
            | Self::Touch
            | Self::Rangefinder => 1,

            Self::Framepos
            | Self::Framexaxis
            | Self::Frameyaxis
            | Self::Framezaxis
            | Self::Framelinvel
            | Self::Frameangvel
            | Self::Framelinacc
            | Self::Frameangacc
            | Self::Ballangvel
            | Self::Force
            | Self::Torque
            | Self::Accelerometer
            | Self::Gyro
            | Self::Velocimeter
            | Self::Magnetometer
            | Self::Subtreecom
            | Self::Subtreelinvel
            | Self::Subtreeangmom => 3,

            Self::Ballquat | Self::Framequat => 4,

            // User sensors have configurable dimension, default to 1
            Self::User => 1,
        }
    }
}

/// A sensor from `<sensor>` element.
///
/// MuJoCo sensors measure various quantities during simulation and
/// can add noise to simulate real sensor behavior.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfSensor {
    /// Sensor name.
    pub name: String,
    /// Default class.
    pub class: Option<String>,
    /// Sensor type.
    pub sensor_type: MjcfSensorType,
    /// Target object name (joint, site, body, etc. depending on type).
    pub objname: Option<String>,
    /// Reference object for frame sensors.
    pub refname: Option<String>,
    /// Noise standard deviation.
    pub noise: f64,
    /// Cutoff frequency for low-pass filter (0 = no filter).
    pub cutoff: f64,
    /// User-defined data fields.
    pub user: Vec<f64>,
}

impl Default for MjcfSensor {
    fn default() -> Self {
        Self {
            name: String::new(),
            class: None,
            sensor_type: MjcfSensorType::default(),
            objname: None,
            refname: None,
            noise: 0.0,
            cutoff: 0.0,
            user: Vec::new(),
        }
    }
}

impl MjcfSensor {
    /// Create a joint position sensor.
    #[must_use]
    pub fn jointpos(name: impl Into<String>, joint: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            sensor_type: MjcfSensorType::Jointpos,
            objname: Some(joint.into()),
            ..Default::default()
        }
    }

    /// Create a joint velocity sensor.
    #[must_use]
    pub fn jointvel(name: impl Into<String>, joint: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            sensor_type: MjcfSensorType::Jointvel,
            objname: Some(joint.into()),
            ..Default::default()
        }
    }

    /// Create an accelerometer sensor.
    #[must_use]
    pub fn accelerometer(name: impl Into<String>, site: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            sensor_type: MjcfSensorType::Accelerometer,
            objname: Some(site.into()),
            ..Default::default()
        }
    }

    /// Create a gyroscope sensor.
    #[must_use]
    pub fn gyro(name: impl Into<String>, site: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            sensor_type: MjcfSensorType::Gyro,
            objname: Some(site.into()),
            ..Default::default()
        }
    }

    /// Set noise standard deviation.
    #[must_use]
    pub fn with_noise(mut self, noise: f64) -> Self {
        self.noise = noise;
        self
    }

    /// Set cutoff frequency for filtering.
    #[must_use]
    pub fn with_cutoff(mut self, cutoff: f64) -> Self {
        self.cutoff = cutoff;
        self
    }

    /// Set user data.
    #[must_use]
    pub fn with_user(mut self, user: Vec<f64>) -> Self {
        self.user = user;
        self
    }

    /// Get the output dimensionality of this sensor.
    #[must_use]
    pub fn dim(&self) -> usize {
        self.sensor_type.dim()
    }
}

// ============================================================================
// Skinned Mesh
// ============================================================================

/// A bone reference within a skin element.
///
/// MuJoCo `<skin>` elements contain `<bone>` children that reference bodies
/// in the kinematic tree. Each bone has a bind pose that defines the
/// initial transform at which vertex weights were assigned.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfSkinBone {
    /// Name of the body this bone references.
    pub body: String,
    /// Bind position in the body's local frame.
    pub bindpos: Vector3<f64>,
    /// Bind orientation as quaternion (w, x, y, z).
    pub bindquat: Vector4<f64>,
}

impl Default for MjcfSkinBone {
    fn default() -> Self {
        Self {
            body: String::new(),
            bindpos: Vector3::zeros(),
            bindquat: Vector4::new(1.0, 0.0, 0.0, 0.0), // Identity quaternion
        }
    }
}

impl MjcfSkinBone {
    /// Create a new skin bone referencing a body.
    #[must_use]
    pub fn new(body: impl Into<String>) -> Self {
        Self {
            body: body.into(),
            ..Default::default()
        }
    }

    /// Set the bind position.
    #[must_use]
    pub fn with_bindpos(mut self, pos: Vector3<f64>) -> Self {
        self.bindpos = pos;
        self
    }

    /// Set the bind orientation.
    #[must_use]
    pub fn with_bindquat(mut self, quat: Vector4<f64>) -> Self {
        self.bindquat = quat;
        self
    }

    /// Get bind orientation as `UnitQuaternion`.
    #[must_use]
    pub fn bind_rotation(&self) -> UnitQuaternion<f64> {
        let q = nalgebra::Quaternion::new(
            self.bindquat[0],
            self.bindquat[1],
            self.bindquat[2],
            self.bindquat[3],
        );
        UnitQuaternion::from_quaternion(q)
    }
}

/// A vertex weight within a skin element.
///
/// Each `<vertex>` in a MuJoCo `<skin>` element specifies which bone(s)
/// influence a particular vertex and with what weight.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfSkinVertex {
    /// Vertex index in the mesh.
    pub id: usize,
    /// Index of the bone within the skin's bone list.
    pub bone: usize,
    /// Weight of this bone's influence on the vertex (0.0 to 1.0).
    pub weight: f64,
}

impl MjcfSkinVertex {
    /// Create a new skin vertex weight.
    #[must_use]
    pub const fn new(id: usize, bone: usize, weight: f64) -> Self {
        Self { id, bone, weight }
    }
}

/// A skinned mesh from `<skin>` element.
///
/// MuJoCo `<skin>` elements define visual meshes that are deformed based on
/// body poses. The skin contains bones (body references) and vertex weights
/// that define how each vertex is influenced by bone transforms.
///
/// # Example MJCF
///
/// ```xml
/// <skin name="body_skin" material="skin_material">
///     <bone body="torso" bindpos="0 0 0" bindquat="1 0 0 0"/>
///     <bone body="upper_arm" bindpos="0.2 0 0" bindquat="1 0 0 0"/>
///     <vertex id="0" bone="0" weight="1.0"/>
///     <vertex id="1" bone="0" weight="0.5"/>
///     <vertex id="1" bone="1" weight="0.5"/>
/// </skin>
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfSkin {
    /// Skin name.
    pub name: String,
    /// Associated mesh asset name (optional).
    pub mesh: Option<String>,
    /// Material name for rendering (optional).
    pub material: Option<String>,
    /// Bones (body references) that influence this skin.
    pub bones: Vec<MjcfSkinBone>,
    /// Vertex weights defining bone influences.
    pub vertices: Vec<MjcfSkinVertex>,
    /// Embedded vertex positions (if not using mesh asset).
    /// Format: flat array of xyz coordinates.
    pub vertex_positions: Option<Vec<f64>>,
    /// Embedded face data (if not using mesh asset).
    /// Format: flat array of vertex indices (triangles).
    pub faces: Option<Vec<u32>>,
    /// RGBA color for visualization.
    pub rgba: Vector4<f64>,
    /// Whether skinning is enabled.
    pub inflate: f64,
}

impl Default for MjcfSkin {
    fn default() -> Self {
        Self {
            name: String::new(),
            mesh: None,
            material: None,
            bones: Vec::new(),
            vertices: Vec::new(),
            vertex_positions: None,
            faces: None,
            rgba: Vector4::new(0.5, 0.5, 0.5, 1.0),
            inflate: 0.0,
        }
    }
}

impl MjcfSkin {
    /// Create a new skin with a name.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            ..Default::default()
        }
    }

    /// Set the mesh asset reference.
    #[must_use]
    pub fn with_mesh(mut self, mesh: impl Into<String>) -> Self {
        self.mesh = Some(mesh.into());
        self
    }

    /// Set the material reference.
    #[must_use]
    pub fn with_material(mut self, material: impl Into<String>) -> Self {
        self.material = Some(material.into());
        self
    }

    /// Add a bone to the skin.
    #[must_use]
    pub fn with_bone(mut self, bone: MjcfSkinBone) -> Self {
        self.bones.push(bone);
        self
    }

    /// Add a vertex weight.
    #[must_use]
    pub fn with_vertex(mut self, id: usize, bone: usize, weight: f64) -> Self {
        self.vertices.push(MjcfSkinVertex::new(id, bone, weight));
        self
    }

    /// Check if this skin has embedded geometry.
    #[must_use]
    pub fn has_embedded_geometry(&self) -> bool {
        self.vertex_positions.is_some()
    }

    /// Get the number of bones.
    #[must_use]
    pub fn num_bones(&self) -> usize {
        self.bones.len()
    }

    /// Get the number of vertex weights.
    #[must_use]
    pub fn num_vertex_weights(&self) -> usize {
        self.vertices.len()
    }

    /// Get embedded vertices as Point3 array.
    #[must_use]
    pub fn vertices_as_points(&self) -> Vec<Point3<f64>> {
        self.vertex_positions.as_ref().map_or_else(Vec::new, |v| {
            v.chunks_exact(3)
                .map(|c| Point3::new(c[0], c[1], c[2]))
                .collect()
        })
    }

    /// Collect all vertex weights grouped by vertex ID.
    ///
    /// Returns a map from vertex index to list of (bone_index, weight) pairs.
    #[must_use]
    pub fn weights_by_vertex(&self) -> std::collections::HashMap<usize, Vec<(usize, f64)>> {
        let mut map: std::collections::HashMap<usize, Vec<(usize, f64)>> =
            std::collections::HashMap::new();
        for v in &self.vertices {
            map.entry(v.id).or_default().push((v.bone, v.weight));
        }
        map
    }
}

// ============================================================================
// Contact
// ============================================================================

/// Parsed `<contact><pair>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfContactPair {
    /// Identifier for this pair.
    pub name: Option<String>,
    /// Defaults class (inherits `<default><pair .../>`).
    pub class: Option<String>,
    /// First geom (by name).
    pub geom1: String,
    /// Second geom (by name).
    pub geom2: String,
    /// Contact dimensionality (1, 3, 4, 6).
    pub condim: Option<i32>,
    /// 5-element friction: [tan1, tan2, torsional, roll1, roll2].
    pub friction: Option<[f64; 5]>,
    /// Solver reference (normal direction).
    pub solref: Option<[f64; 2]>,
    /// Solver reference (friction directions).
    pub solreffriction: Option<[f64; 2]>,
    /// Solver impedance.
    pub solimp: Option<[f64; 5]>,
    /// Distance threshold for contact activation.
    pub margin: Option<f64>,
    /// Contact included if distance < margin - gap.
    pub gap: Option<f64>,
}

/// Parsed `<contact><exclude>` element.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfContactExclude {
    /// Identifier for this exclusion.
    pub name: Option<String>,
    /// First body (by name).
    pub body1: String,
    /// Second body (by name).
    pub body2: String,
}

/// Parsed `<contact>` element (grouping container).
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfContact {
    /// Explicit contact pairs.
    pub pairs: Vec<MjcfContactPair>,
    /// Body-pair exclusions.
    pub excludes: Vec<MjcfContactExclude>,
}

// ============================================================================
// Keyframe
// ============================================================================

/// A single MJCF `<key>` element within `<keyframe>`.
///
/// All state fields are optional — `None` means "use model default" (qpos0
/// for qpos, zeros for qvel/act/ctrl, body_pos/body_quat for mocap). The
/// `name` field uses an empty string for unnamed keyframes.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfKeyframe {
    /// Keyframe name from MJCF `name` attribute. Empty string if unnamed.
    pub name: String,
    /// None means "use model default (qpos0)".
    pub qpos: Option<Vec<f64>>,
    /// None means "use model default (zeros)".
    pub qvel: Option<Vec<f64>>,
    /// None means "use model default (zeros)".
    pub act: Option<Vec<f64>>,
    /// None means "use model default (zeros)".
    pub ctrl: Option<Vec<f64>>,
    /// None means "use model default (body_pos for each mocap body)".
    pub mpos: Option<Vec<f64>>,
    /// None means "use model default (body_quat for each mocap body)".
    pub mquat: Option<Vec<f64>>,
    /// Simulation time for this keyframe. Default: 0.0.
    pub time: f64,
}

impl Default for MjcfKeyframe {
    fn default() -> Self {
        Self {
            name: String::new(),
            qpos: None,
            qvel: None,
            act: None,
            ctrl: None,
            mpos: None,
            mquat: None,
            time: 0.0,
        }
    }
}

// ============================================================================
// Model
// ============================================================================

/// Parsed `<flex>` element from `<deformable>`.
///
/// Represents a deformable flex body with vertices, elements, and material/collision
/// properties. Maps to MuJoCo's `<flex>` element within `<deformable>`.
///
/// Contact parameters come from `<flex><contact>`, elasticity from `<flex><elasticity>`,
/// and edge spring-damper from `<flex><edge>`. Direct `<flex>` attributes are only
/// `name`, `dim`, `radius`, `body`, `node`, `group` (plus structural data arrays).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfFlex {
    // --- Direct <flex> attributes ---
    /// Flex name.
    pub name: String,
    /// Dimensionality: 1=cable, 2=shell, 3=solid.
    pub dim: usize,
    /// Collision vertex radius [m].
    pub radius: f64,
    /// Body names for vertex attachment (one per vertex). Required on `<flex>`.
    /// For `<flexcomp>`, auto-generated during expansion (one body per vertex).
    pub body: Vec<String>,
    /// Node body names (alternative to `<vertex>` positions).
    pub node: Vec<String>,
    /// Visualization group (0-5).
    pub group: i32,

    // --- <flex><contact> child element attributes ---
    /// Contact priority (default 0). Higher priority geom's params win.
    pub priority: i32,
    /// Solver parameter mixing weight (default 1.0).
    pub solmix: f64,
    /// Contact gap — buffer zone within margin (default 0.0).
    pub gap: f64,
    /// Collision friction coefficients: tangential, torsional, rolling.
    pub friction: Vector3<f64>,
    /// Collision contact dimensionality (1, 3, or 4).
    pub condim: i32,
    /// Collision margin [m].
    pub margin: f64,
    /// Solver reference parameters.
    pub solref: [f64; 2],
    /// Solver impedance parameters.
    pub solimp: [f64; 5],
    /// Collision type bitmask (default 1). Used for flex-rigid and flex-flex
    /// bitmask filtering: collision proceeds when
    /// `(flex_contype & geom_conaffinity) != 0 || (geom_contype & flex_conaffinity) != 0`.
    pub contype: Option<i32>,
    /// Collision affinity bitmask (default 1). See `contype`.
    pub conaffinity: Option<i32>,
    /// Self-collision broadphase mode. MuJoCo keyword: [none, narrow, bvh, sap, auto].
    /// None = absent (default "auto"); Some("none") = disabled; other = enabled.
    pub selfcollide: Option<String>,

    // --- <flex><elasticity> child element attributes ---
    /// Young's modulus [Pa].
    pub young: f64,
    /// Poisson's ratio.
    pub poisson: f64,
    /// Damping coefficient.
    pub damping: f64,
    /// Shell thickness [m] (dim=2 only). -1 = "not set".
    pub thickness: f64,

    // --- <flex><edge> child element attributes ---
    /// Passive edge spring stiffness. Default 0.0 = disabled.
    /// Used for direct spring-damper forces in the passive force path.
    pub edge_stiffness: f64,
    /// Passive edge damping coefficient. Default 0.0 = disabled.
    pub edge_damping: f64,

    // --- Internal / derived ---
    /// Total mass [kg] for uniform distribution across all vertices (including pinned).
    /// MuJoCo semantics: `mass / npnt` per vertex, pinned share silently discarded.
    /// When `Some`, overrides element-based mass lumping from `density`.
    pub mass: Option<f64>,
    /// Volumetric density [kg/m³] (dim=2,3) or linear density [kg/m] (dim=1).
    /// Fallback for element-based mass lumping when `mass` is `None`.
    pub density: f64,

    // --- Structural data arrays ---
    /// Vertex positions.
    pub vertices: Vec<Vector3<f64>>,
    /// Element connectivity (each element is a list of vertex indices).
    pub elements: Vec<Vec<usize>>,
    /// Pinned vertex indices (infinite mass, fixed in place).
    pub pinned: Vec<usize>,
}

impl Default for MjcfFlex {
    fn default() -> Self {
        Self {
            // Direct <flex> attributes
            name: String::new(),
            dim: 2,
            radius: 0.005,
            body: Vec::new(),
            node: Vec::new(),
            group: 0,
            // <contact> child element (MuJoCo defaults)
            priority: 0,
            solmix: 1.0,
            gap: 0.0,
            friction: Vector3::new(1.0, 0.005, 0.0001),
            condim: 3,
            margin: 0.0,
            solref: [0.02, 1.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
            contype: None,     // MuJoCo default: 1
            conaffinity: None, // MuJoCo default: 1
            selfcollide: None, // MuJoCo default is "auto" (enabled)
            // <elasticity> child element (MuJoCo defaults)
            young: 0.0, // MuJoCo default; was 1e6
            poisson: 0.0,
            damping: 0.0,
            thickness: -1.0, // MuJoCo default sentinel "not set"; was 0.001
            // <edge> child element
            edge_stiffness: 0.0,
            edge_damping: 0.0,
            // Internal
            mass: None,
            density: 1000.0,
            // Structural data
            vertices: Vec::new(),
            elements: Vec::new(),
            pinned: Vec::new(),
        }
    }
}

/// A complete MJCF model.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfModel {
    /// Model name.
    pub name: String,
    /// Global simulation options.
    pub option: MjcfOption,
    /// Compiler settings.
    pub compiler: MjcfCompiler,
    /// Default parameter classes.
    pub defaults: Vec<MjcfDefault>,
    /// Mesh assets.
    pub meshes: Vec<MjcfMesh>,
    /// Height field assets.
    pub hfields: Vec<MjcfHfield>,
    /// Root worldbody containing the body tree.
    pub worldbody: MjcfBody,
    /// Actuators.
    pub actuators: Vec<MjcfActuator>,
    /// Tendons.
    pub tendons: Vec<MjcfTendon>,
    /// Sensors.
    pub sensors: Vec<MjcfSensor>,
    /// Equality constraints.
    pub equality: MjcfEquality,
    /// Contact pairs and exclusions.
    pub contact: MjcfContact,
    /// Skinned meshes for visual deformation.
    pub skins: Vec<MjcfSkin>,
    /// Flex deformable bodies.
    pub flex: Vec<MjcfFlex>,
    /// Keyframes for quick state reset.
    #[cfg_attr(feature = "serde", serde(default))]
    pub keyframes: Vec<MjcfKeyframe>,
}

impl Default for MjcfModel {
    fn default() -> Self {
        Self {
            name: "unnamed".to_string(),
            option: MjcfOption::default(),
            compiler: MjcfCompiler::default(),
            defaults: Vec::new(),
            meshes: Vec::new(),
            hfields: Vec::new(),
            worldbody: MjcfBody::new("world"),
            actuators: Vec::new(),
            tendons: Vec::new(),
            sensors: Vec::new(),
            equality: MjcfEquality::default(),
            contact: MjcfContact::default(),
            skins: Vec::new(),
            flex: Vec::new(),
            keyframes: Vec::new(),
        }
    }
}

impl MjcfModel {
    /// Create a new model with a name.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            ..Default::default()
        }
    }

    /// Add a default class.
    #[must_use]
    pub fn with_default(mut self, default: MjcfDefault) -> Self {
        self.defaults.push(default);
        self
    }

    /// Add a body to the worldbody.
    #[must_use]
    pub fn with_body(mut self, body: MjcfBody) -> Self {
        self.worldbody.children.push(body);
        self
    }

    /// Add an actuator.
    #[must_use]
    pub fn with_actuator(mut self, actuator: MjcfActuator) -> Self {
        self.actuators.push(actuator);
        self
    }

    /// Get a flattened list of all bodies (breadth-first).
    #[must_use]
    pub fn all_bodies(&self) -> Vec<&MjcfBody> {
        let mut bodies = Vec::new();
        let mut queue = vec![&self.worldbody];

        while let Some(body) = queue.pop() {
            // Skip worldbody itself but include its children
            for child in &body.children {
                bodies.push(child);
                queue.extend(child.children.iter());
            }
        }

        bodies
    }

    /// Get a body by name.
    #[must_use]
    pub fn body(&self, name: &str) -> Option<&MjcfBody> {
        fn find_body<'a>(body: &'a MjcfBody, name: &str) -> Option<&'a MjcfBody> {
            if body.name == name {
                return Some(body);
            }
            for child in &body.children {
                if let Some(found) = find_body(child, name) {
                    return Some(found);
                }
            }
            None
        }

        for child in &self.worldbody.children {
            if let Some(found) = find_body(child, name) {
                return Some(found);
            }
        }
        None
    }

    /// Get a joint by name.
    #[must_use]
    pub fn joint(&self, name: &str) -> Option<&MjcfJoint> {
        fn find_joint<'a>(body: &'a MjcfBody, name: &str) -> Option<&'a MjcfJoint> {
            for joint in &body.joints {
                if joint.name == name {
                    return Some(joint);
                }
            }
            for child in &body.children {
                if let Some(found) = find_joint(child, name) {
                    return Some(found);
                }
            }
            None
        }

        for child in &self.worldbody.children {
            if let Some(found) = find_joint(child, name) {
                return Some(found);
            }
        }
        None
    }

    /// Get a mesh asset by name.
    #[must_use]
    pub fn mesh(&self, name: &str) -> Option<&MjcfMesh> {
        self.meshes.iter().find(|m| m.name == name)
    }

    /// Add a mesh asset to the model.
    #[must_use]
    pub fn with_mesh(mut self, mesh: MjcfMesh) -> Self {
        self.meshes.push(mesh);
        self
    }

    /// Add a skin to the model.
    #[must_use]
    pub fn with_skin(mut self, skin: MjcfSkin) -> Self {
        self.skins.push(skin);
        self
    }

    /// Get a skin by name.
    #[must_use]
    pub fn skin(&self, name: &str) -> Option<&MjcfSkin> {
        self.skins.iter().find(|s| s.name == name)
    }

    /// Add a tendon to the model.
    #[must_use]
    pub fn with_tendon(mut self, tendon: MjcfTendon) -> Self {
        self.tendons.push(tendon);
        self
    }

    /// Get a tendon by name.
    #[must_use]
    pub fn tendon(&self, name: &str) -> Option<&MjcfTendon> {
        self.tendons.iter().find(|t| t.name == name)
    }

    /// Add a sensor to the model.
    #[must_use]
    pub fn with_sensor(mut self, sensor: MjcfSensor) -> Self {
        self.sensors.push(sensor);
        self
    }

    /// Get a sensor by name.
    #[must_use]
    pub fn sensor(&self, name: &str) -> Option<&MjcfSensor> {
        self.sensors.iter().find(|s| s.name == name)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_option_default() {
        let option = MjcfOption::default();
        assert_relative_eq!(option.timestep, 0.002, epsilon = 1e-10);
        assert_relative_eq!(option.gravity.z, -9.81, epsilon = 1e-10);
    }

    #[test]
    fn test_inertial_matrix() {
        let inertial = MjcfInertial {
            mass: 2.0,
            diaginertia: Some(Vector3::new(1.0, 2.0, 3.0)),
            ..Default::default()
        };

        let m = inertial.inertia_matrix();
        assert_relative_eq!(m[(0, 0)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(m[(1, 1)], 2.0, epsilon = 1e-10);
        assert_relative_eq!(m[(2, 2)], 3.0, epsilon = 1e-10);
        assert_relative_eq!(m[(0, 1)], 0.0, epsilon = 1e-10); // Off-diagonal
    }

    #[test]
    fn test_geom_type_parse() {
        assert_eq!(MjcfGeomType::from_str("sphere"), Some(MjcfGeomType::Sphere));
        assert_eq!(MjcfGeomType::from_str("box"), Some(MjcfGeomType::Box));
        assert_eq!(
            MjcfGeomType::from_str("capsule"),
            Some(MjcfGeomType::Capsule)
        );
        assert_eq!(MjcfGeomType::from_str("invalid"), None);
    }

    #[test]
    fn test_joint_type_dof() {
        assert_eq!(MjcfJointType::Hinge.dof(), 1);
        assert_eq!(MjcfJointType::Slide.dof(), 1);
        assert_eq!(MjcfJointType::Ball.dof(), 3);
        assert_eq!(MjcfJointType::Free.dof(), 6);
    }

    #[test]
    fn test_geom_computed_mass() {
        let sphere = MjcfGeom {
            geom_type: Some(MjcfGeomType::Sphere),
            size: vec![0.1],
            density: Some(1000.0),
            mass: None,
            ..Default::default()
        };

        // Volume of sphere = 4/3 * pi * r^3
        let expected_volume = (4.0 / 3.0) * std::f64::consts::PI * 0.1_f64.powi(3);
        let expected_mass = 1000.0 * expected_volume;
        assert_relative_eq!(sphere.computed_mass(), expected_mass, epsilon = 1e-10);
    }

    #[test]
    fn test_geom_explicit_mass() {
        let sphere = MjcfGeom {
            geom_type: Some(MjcfGeomType::Sphere),
            size: vec![0.1],
            density: Some(1000.0),
            mass: Some(5.0), // Explicit mass overrides density
            ..Default::default()
        };

        assert_relative_eq!(sphere.computed_mass(), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_body_rotation() {
        let body = MjcfBody {
            euler: Some(Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_2)),
            ..Default::default()
        };

        let rot = body.rotation();
        let rotated = rot * Vector3::x();
        assert_relative_eq!(rotated.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(rotated.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_model_builder() {
        let model = MjcfModel::new("test_model")
            .with_body(
                MjcfBody::new("body1")
                    .with_pos(1.0, 0.0, 0.0)
                    .with_geom(MjcfGeom::sphere(0.1))
                    .with_joint(MjcfJoint::hinge("joint1", Vector3::z())),
            )
            .with_actuator(MjcfActuator::motor("motor1", "joint1"));

        assert_eq!(model.name, "test_model");
        assert_eq!(model.worldbody.children.len(), 1);
        assert_eq!(model.actuators.len(), 1);

        assert!(model.body("body1").is_some());
        assert!(model.joint("joint1").is_some());
    }
}
