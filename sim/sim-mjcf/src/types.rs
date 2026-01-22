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
    /// Implicit-in-velocity integration (stable for stiff systems).
    Implicit,
    /// Implicit-fast (skips Coriolis terms for performance).
    ImplicitFast,
}

impl MjcfIntegrator {
    /// Parse integrator from MJCF string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "euler" => Some(Self::Euler),
            "rk4" => Some(Self::RK4),
            "implicit" => Some(Self::Implicit),
            "implicitfast" => Some(Self::ImplicitFast),
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
    /// Enable friction limit constraints.
    pub frictionloss: bool,
    /// Enable joint/tendon limit constraints.
    pub limit: bool,
    /// Enable contact force computation.
    pub contact: bool,
    /// Enable passive spring forces.
    pub passive: bool,
    /// Enable gravity force.
    pub gravity: bool,
    /// Enable Coriolis/centrifugal forces.
    pub clampctrl: bool,
    /// Enable warm-starting of constraint solver.
    pub warmstart: bool,
    /// Enable filtering of contact pairs.
    pub filterparent: bool,
    /// Enable actuation.
    pub actuation: bool,
    /// Enable reference configuration in computation.
    pub refsafe: bool,
    /// Enable sensor computation.
    pub sensor: bool,
    /// Enable mid-phase collision detection.
    pub midphase: bool,
    /// Enable native CCD (continuous collision detection).
    pub nativeccd: bool,
    /// Enable Euler angle damping.
    pub eulerdamp: bool,
    /// Override contacts (use constraint-based contacts).
    pub override_contacts: bool,
    /// Enable energy computation.
    pub energy: bool,
    /// Enable body sleeping/deactivation.
    pub island: bool,
    /// Enable multi-CCD (multiple CCD iterations).
    pub multiccd: bool,
}

impl Default for MjcfFlag {
    fn default() -> Self {
        Self {
            constraint: true,
            equality: true,
            frictionloss: true,
            limit: true,
            contact: true,
            passive: true,
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
            override_contacts: false,
            energy: false,
            island: false,
            multiccd: false,
        }
    }
}

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

    /// Iterations for no-slip solver (default: 0 = disabled).
    pub noslip_iterations: usize,

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
            noslip_iterations: 0,
            ccd_iterations: 50,

            // Contact configuration
            cone: MjcfConeType::default(),
            jacobian: MjcfJacobianType::default(),
            impratio: 1.0,

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
}

/// Default joint parameters.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfJointDefaults {
    /// Joint type.
    pub joint_type: Option<MjcfJointType>,
    /// Position limits enabled.
    pub limited: Option<bool>,
    /// Joint axis.
    pub axis: Option<Vector3<f64>>,
    /// Damping coefficient.
    pub damping: Option<f64>,
    /// Spring stiffness.
    pub stiffness: Option<f64>,
    /// Armature (rotor inertia).
    pub armature: Option<f64>,
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
}

/// Default actuator parameters.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfActuatorDefaults {
    /// Control range.
    pub ctrlrange: Option<(f64, f64)>,
    /// Force range.
    pub forcerange: Option<(f64, f64)>,
    /// Gear ratio.
    pub gear: Option<f64>,
    /// Position gain (kp) for position actuators.
    pub kp: Option<f64>,
    /// Velocity gain (kv) for velocity actuators.
    pub kv: Option<f64>,
    /// Control limited.
    pub ctrllimited: Option<bool>,
    /// Force limited.
    pub forcelimited: Option<bool>,
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
    /// Tendon width for visualization.
    pub width: Option<f64>,
    /// RGBA color for visualization.
    pub rgba: Option<Vector4<f64>>,
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
    pub scale: Vector3<f64>,
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
            scale: Vector3::new(1.0, 1.0, 1.0),
            vertex: None,
            face: None,
        }
    }
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
        self.scale = scale;
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
        self.vertex.as_ref().map_or_else(Vec::new, |v| {
            v.chunks_exact(3)
                .map(|c| {
                    Point3::new(
                        c[0] * self.scale.x,
                        c[1] * self.scale.y,
                        c[2] * self.scale.z,
                    )
                })
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
    /// Geom type.
    pub geom_type: MjcfGeomType,
    /// Position relative to body frame.
    pub pos: Vector3<f64>,
    /// Orientation (quaternion: w x y z).
    pub quat: Vector4<f64>,
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
    pub friction: Vector3<f64>,
    /// Density for mass computation (kg/m³).
    pub density: f64,
    /// Explicit mass (overrides density).
    pub mass: Option<f64>,
    /// RGBA color.
    pub rgba: Vector4<f64>,
    /// Collision type bitmask.
    pub contype: i32,
    /// Collision affinity bitmask.
    pub conaffinity: i32,
    /// Contact dimensionality (condim).
    pub condim: i32,
    /// Mesh asset name (for type="mesh").
    pub mesh: Option<String>,
}

impl Default for MjcfGeom {
    fn default() -> Self {
        Self {
            name: None,
            class: None,
            geom_type: MjcfGeomType::Sphere,
            pos: Vector3::zeros(),
            quat: Vector4::new(1.0, 0.0, 0.0, 0.0),
            size: vec![0.1], // Default sphere radius
            fromto: None,
            friction: Vector3::new(1.0, 0.005, 0.0001), // MuJoCo defaults
            density: 1000.0,                            // Water density
            mass: None,
            rgba: Vector4::new(0.5, 0.5, 0.5, 1.0),
            contype: 1,
            conaffinity: 1,
            condim: 3,
            mesh: None,
        }
    }
}

impl MjcfGeom {
    /// Create a sphere geom.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        Self {
            geom_type: MjcfGeomType::Sphere,
            size: vec![radius],
            ..Default::default()
        }
    }

    /// Create a box geom.
    #[must_use]
    pub fn box_shape(half_extents: Vector3<f64>) -> Self {
        Self {
            geom_type: MjcfGeomType::Box,
            size: vec![half_extents.x, half_extents.y, half_extents.z],
            ..Default::default()
        }
    }

    /// Create a capsule geom.
    #[must_use]
    pub fn capsule(radius: f64, half_length: f64) -> Self {
        Self {
            geom_type: MjcfGeomType::Capsule,
            size: vec![radius, half_length],
            ..Default::default()
        }
    }

    /// Get quaternion as `UnitQuaternion`.
    #[must_use]
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        let q = nalgebra::Quaternion::new(self.quat[0], self.quat[1], self.quat[2], self.quat[3]);
        UnitQuaternion::from_quaternion(q)
    }

    /// Compute mass from density and volume.
    #[must_use]
    pub fn computed_mass(&self) -> f64 {
        if let Some(mass) = self.mass {
            return mass;
        }

        let volume = match self.geom_type {
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

        self.density * volume
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
    pub joint_type: MjcfJointType,
    /// Joint position relative to body frame.
    pub pos: Vector3<f64>,
    /// Joint axis (for hinge and slide).
    pub axis: Vector3<f64>,
    /// Whether position limits are enabled.
    pub limited: bool,
    /// Position limit range [lower, upper].
    pub range: Option<(f64, f64)>,
    /// Joint reference position.
    pub ref_pos: f64,
    /// Spring equilibrium position.
    pub spring_ref: f64,
    /// Damping coefficient.
    pub damping: f64,
    /// Spring stiffness.
    pub stiffness: f64,
    /// Armature (rotor inertia) - added to diagonal of inertia matrix.
    pub armature: f64,
    /// Friction loss.
    pub frictionloss: f64,
    /// Body this joint belongs to (set during parsing).
    pub body: Option<String>,
}

impl Default for MjcfJoint {
    fn default() -> Self {
        Self {
            name: String::new(),
            class: None,
            joint_type: MjcfJointType::Hinge,
            pos: Vector3::zeros(),
            axis: Vector3::z(), // Default Z-axis
            limited: false,
            range: None,
            ref_pos: 0.0,
            spring_ref: 0.0,
            damping: 0.0,
            stiffness: 0.0,
            armature: 0.0,
            frictionloss: 0.0,
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
            joint_type: MjcfJointType::Hinge,
            axis,
            ..Default::default()
        }
    }

    /// Create a new slide joint.
    #[must_use]
    pub fn slide(name: impl Into<String>, axis: Vector3<f64>) -> Self {
        Self {
            name: name.into(),
            joint_type: MjcfJointType::Slide,
            axis,
            ..Default::default()
        }
    }

    /// Set position limits.
    #[must_use]
    pub fn with_limits(mut self, lower: f64, upper: f64) -> Self {
        self.limited = true;
        self.range = Some((lower, upper));
        self
    }

    /// Set damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping;
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
    /// Position relative to body frame.
    pub pos: Vector3<f64>,
    /// Orientation (quaternion: w x y z).
    pub quat: Vector4<f64>,
    /// Size (for visualization).
    pub size: Vec<f64>,
    /// RGBA color.
    pub rgba: Vector4<f64>,
}

impl Default for MjcfSite {
    fn default() -> Self {
        Self {
            name: String::new(),
            pos: Vector3::zeros(),
            quat: Vector4::new(1.0, 0.0, 0.0, 0.0),
            size: vec![0.01],
            rgba: Vector4::new(1.0, 0.0, 0.0, 1.0),
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

/// Container for equality constraints from `<equality>` element.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfEquality {
    /// Connect (ball) constraints.
    pub connects: Vec<MjcfConnect>,
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

    /// Check if there are any equality constraints.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.connects.is_empty()
    }

    /// Get the total number of equality constraints.
    #[must_use]
    pub fn len(&self) -> usize {
        self.connects.len()
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
    /// Child bodies.
    pub children: Vec<MjcfBody>,
    /// Parent body name (set during flattening).
    pub parent: Option<String>,
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
            children: Vec::new(),
            parent: None,
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
    /// Muscle actuator.
    Muscle,
    /// Cylinder (pneumatic/hydraulic).
    Cylinder,
    /// Damper (passive).
    Damper,
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
            _ => None,
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
    /// Gear ratio (torque/force scaling).
    pub gear: f64,
    /// Control range [lower, upper].
    pub ctrlrange: Option<(f64, f64)>,
    /// Force range [lower, upper].
    pub forcerange: Option<(f64, f64)>,
    /// Whether control is clamped to range.
    pub ctrllimited: bool,
    /// Whether force is clamped to range.
    pub forcelimited: bool,
    /// Position gain (for position actuators).
    pub kp: f64,
    /// Velocity gain (for position/velocity actuators).
    pub kv: f64,
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
            gear: 1.0,
            ctrlrange: None,
            forcerange: None,
            ctrllimited: false,
            forcelimited: false,
            kp: 1.0,
            kv: 0.0,
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
            kv,
            ..Default::default()
        }
    }
}

// ============================================================================
// Model
// ============================================================================

/// A complete MJCF model.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfModel {
    /// Model name.
    pub name: String,
    /// Global simulation options.
    pub option: MjcfOption,
    /// Default parameter classes.
    pub defaults: Vec<MjcfDefault>,
    /// Mesh assets.
    pub meshes: Vec<MjcfMesh>,
    /// Root worldbody containing the body tree.
    pub worldbody: MjcfBody,
    /// Actuators.
    pub actuators: Vec<MjcfActuator>,
    /// Equality constraints.
    pub equality: MjcfEquality,
}

impl Default for MjcfModel {
    fn default() -> Self {
        Self {
            name: "unnamed".to_string(),
            option: MjcfOption::default(),
            defaults: Vec::new(),
            meshes: Vec::new(),
            worldbody: MjcfBody::new("world"),
            actuators: Vec::new(),
            equality: MjcfEquality::default(),
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
            geom_type: MjcfGeomType::Sphere,
            size: vec![0.1],
            density: 1000.0,
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
            geom_type: MjcfGeomType::Sphere,
            size: vec![0.1],
            density: 1000.0,
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
