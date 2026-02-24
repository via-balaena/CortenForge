//! Contact-related types for constraint generation.
//!
//! Contains [`ContactPair`] (explicit pair overrides from MJCF `<pair>`),
//! [`Contact`] (detected contact point for constraint generation),
//! and the [`compute_tangent_frame`] helper.

use nalgebra::Vector3;

use crate::constraint::impedance::{DEFAULT_SOLIMP, DEFAULT_SOLREF};

/// Explicit contact pair: geom indices + per-pair overrides.
/// All fields are fully resolved at build time (no Options).
#[derive(Debug, Clone)]
pub struct ContactPair {
    /// First geometry index.
    pub geom1: usize,
    /// Second geometry index.
    pub geom2: usize,
    /// Contact dimensionality (1, 3, 4, 6).
    pub condim: i32,
    /// 5-element friction: [tan1, tan2, torsional, roll1, roll2].
    pub friction: [f64; 5],
    /// Solver reference (normal direction).
    pub solref: [f64; 2],
    /// Solver reference (friction directions).
    pub solreffriction: [f64; 2],
    /// Solver impedance.
    pub solimp: [f64; 5],
    /// Distance threshold for contact activation.
    pub margin: f64,
    /// Contact included if distance < margin - gap.
    pub gap: f64,
}

/// Contact point for constraint generation.
///
/// Matches MuJoCo's mjContact structure with all relevant fields
/// for constraint-based contact resolution.
#[derive(Debug, Clone)]
pub struct Contact {
    /// Contact position in world frame.
    pub pos: Vector3<f64>,
    /// Contact normal (from geom1 toward geom2, unit vector).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = penetrating).
    pub depth: f64,
    /// First geometry ID.
    pub geom1: usize,
    /// Second geometry ID.
    pub geom2: usize,
    /// Friction coefficient (combined from both geoms).
    pub friction: f64,
    /// Contact dimension: 1 (frictionless), 3 (friction), 4 (elliptic), 6 (torsional).
    pub dim: usize,
    /// Distance threshold for constraint force onset.
    /// `includemargin = effective_margin - effective_gap`
    /// Constraint is excluded when dist >= includemargin (MuJoCo convention).
    /// In our depth convention (positive = overlap): excluded when depth <= -includemargin.
    pub includemargin: f64,
    /// Friction parameters for MuJoCo-style 5-element friction.
    /// `[sliding1, sliding2, torsional, rolling1, rolling2]`
    /// - sliding1/2: tangent friction coefficients
    /// - torsional: spin friction coefficient
    /// - rolling1/2: rolling friction coefficients
    pub mu: [f64; 5],
    /// Solver reference parameters (from geom pair).
    pub solref: [f64; 2],
    /// Solver reference for friction directions (§31).
    ///
    /// Only meaningful for elliptic contacts from explicit `<pair>` definitions.
    /// `[0.0, 0.0]` is the sentinel meaning "use `solref`" (MuJoCo convention).
    /// Auto-generated contacts always have `[0.0, 0.0]`.
    ///
    /// When nonzero **and** the constraint is `ContactElliptic` with `row > 0`
    /// (friction row), the KBIP computation uses `solreffriction` instead of
    /// `solref` for the B (damping) term. K is always 0 for friction rows.
    pub solreffriction: [f64; 2],
    /// Solver impedance parameters (from geom pair).
    pub solimp: [f64; 5],
    /// Contact frame tangent vectors (orthogonal to normal).
    /// `frame[0..3]` = t1, `frame[3..6]` = t2 (for friction cone).
    pub frame: [Vector3<f64>; 2],
    /// If `Some(vertex_idx)`, this is a flex-rigid contact.
    /// `geom1` and `geom2` both reference the rigid geom index.
    /// The flex vertex index is stored here. If `None`, standard rigid-rigid contact.
    pub flex_vertex: Option<usize>,
}

impl Contact {
    /// Create a new contact with basic parameters.
    ///
    /// The contact frame (tangent vectors) is computed automatically from the normal.
    /// Advanced solver parameters use MuJoCo defaults.
    ///
    /// # Numerical Safety
    /// - Negative or NaN friction is clamped to 0.0
    /// - NaN depth is set to 0.0
    /// - NaN position/normal components are handled gracefully
    #[must_use]
    #[inline]
    pub fn new(
        pos: Vector3<f64>,
        normal: Vector3<f64>,
        depth: f64,
        geom1: usize,
        geom2: usize,
        friction: f64,
    ) -> Self {
        Self::with_solver_params(
            pos,
            normal,
            depth,
            geom1,
            geom2,
            friction,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        )
    }

    /// Create a new contact with explicit solver parameters.
    ///
    /// This constructor should be used when creating contacts from collision
    /// detection, where the solref/solimp values come from the colliding geoms.
    ///
    /// MuJoCo combines geom solver parameters using element-wise minimum for
    /// solref (stiffer wins) and element-wise maximum for solimp (harder wins).
    ///
    /// # Numerical Safety
    /// - Negative or NaN friction is clamped to 0.0
    /// - NaN depth is set to 0.0
    /// - NaN position/normal components are handled gracefully
    #[must_use]
    #[inline]
    #[allow(clippy::too_many_arguments)] // Matches MuJoCo's contact fields; grouping into a struct would add indirection for no benefit
    pub fn with_solver_params(
        pos: Vector3<f64>,
        normal: Vector3<f64>,
        depth: f64,
        geom1: usize,
        geom2: usize,
        friction: f64,
        solref: [f64; 2],
        solimp: [f64; 5],
    ) -> Self {
        // Safety: clamp friction to non-negative finite value
        let friction = if friction.is_finite() && friction > 0.0 {
            friction
        } else {
            0.0
        };

        // Safety: ensure depth is finite
        let depth = if depth.is_finite() { depth } else { 0.0 };

        // Compute tangent frame from normal (handles NaN/zero normals internally)
        let (t1, t2) = compute_tangent_frame(&normal);

        Self {
            pos,
            normal,
            depth,
            geom1,
            geom2,
            friction,
            dim: if friction > 0.0 { 3 } else { 1 }, // 3D friction cone or frictionless
            includemargin: 0.0,
            // MuJoCo 5-element friction: [sliding1, sliding2, torsional, rolling1, rolling2]
            mu: [
                friction,
                friction,
                friction * 0.005,
                friction * 0.001,
                friction * 0.001,
            ],
            solref,
            solreffriction: [0.0, 0.0],
            solimp,
            frame: (t1, t2).into(),
            flex_vertex: None,
        }
    }

    /// Create a contact with explicit condim and per-type friction coefficients.
    ///
    /// This constructor is used when creating contacts from collision detection
    /// where condim and friction values come from combining both geom properties.
    ///
    /// # Arguments
    /// * `sliding` - Sliding friction coefficient (mu\[0\], mu\[1\])
    /// * `torsional` - Torsional/spin friction coefficient (mu\[2\])
    /// * `rolling` - Rolling friction coefficient (mu\[3\], mu\[4\])
    /// * `condim` - Contact dimension (1, 3, 4, or 6)
    #[must_use]
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub fn with_condim(
        pos: Vector3<f64>,
        normal: Vector3<f64>,
        depth: f64,
        geom1: usize,
        geom2: usize,
        sliding: f64,
        torsional: f64,
        rolling: f64,
        condim: i32,
        solref: [f64; 2],
        solimp: [f64; 5],
    ) -> Self {
        // Safety: clamp friction values to non-negative finite
        let sliding = if sliding.is_finite() && sliding > 0.0 {
            sliding
        } else {
            0.0
        };
        let torsional = if torsional.is_finite() && torsional > 0.0 {
            torsional
        } else {
            0.0
        };
        let rolling = if rolling.is_finite() && rolling > 0.0 {
            rolling
        } else {
            0.0
        };

        // Safety: ensure depth is finite
        let depth = if depth.is_finite() { depth } else { 0.0 };

        // Compute tangent frame from normal
        let (t1, t2) = compute_tangent_frame(&normal);

        // Contact dimension is determined directly by condim
        // The friction coefficients determine the cone shape, not the structure
        // condim=1: frictionless (normal only)
        // condim=3: normal + 2 sliding friction
        // condim=4: normal + 2 sliding + 1 torsional
        // condim=6: normal + 2 sliding + 1 torsional + 2 rolling
        //
        // Note: condim is validated and clamped in the MJCF loader (builder/mod.rs)
        // to {1, 3, 4, 6}. This fallback handles test code that constructs contacts
        // directly without going through the loader.
        //
        // Explicit match arms document valid values; clippy::match_same_arms allowed
        // because the structure mirrors the MJCF loader's validation pattern.
        // cast_sign_loss is safe because all arms return positive values.
        #[allow(clippy::match_same_arms, clippy::cast_sign_loss)]
        let dim = match condim {
            1 => 1,
            3 => 3,
            4 => 4,
            6 => 6,
            // Round up invalid values (matches MJCF loader behavior)
            0 | 2 => 3,
            5 => 6,
            _ => 6, // >6 clamps to 6
        } as usize;

        Self {
            pos,
            normal,
            depth,
            geom1,
            geom2,
            friction: sliding, // Keep legacy field for compatibility
            dim,
            includemargin: 0.0,
            // MuJoCo 5-element friction: [sliding1, sliding2, torsional, rolling1, rolling2]
            mu: [sliding, sliding, torsional, rolling, rolling],
            solref,
            solreffriction: [0.0, 0.0],
            solimp,
            frame: (t1, t2).into(),
            flex_vertex: None,
        }
    }
}

/// Compute orthonormal tangent frame from contact normal.
///
/// Returns (t1, t2) where t1, t2, normal form a right-handed orthonormal basis.
/// Handles degenerate cases (zero/NaN normal) by returning a default frame.
#[inline]
#[must_use]
pub fn compute_tangent_frame(normal: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    // Safety check: handle zero/NaN normals
    let normal_len = normal.norm();
    if !normal_len.is_finite() || normal_len < 1e-10 {
        // Degenerate case: return default frame
        return (Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
    }

    // Normalize the normal (in case it wasn't already)
    let n = normal / normal_len;

    // Choose a reference vector not parallel to normal
    let reference = if n.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };

    // Gram-Schmidt orthogonalization
    let t1 = reference - n * n.dot(&reference);
    let t1_norm = t1.norm();
    let t1 = if t1_norm > 1e-10 {
        t1 / t1_norm
    } else {
        // This shouldn't happen if reference was chosen correctly, but be safe
        Vector3::new(1.0, 0.0, 0.0)
    };

    let t2 = n.cross(&t1);
    // t2 should already be unit length since n and t1 are orthonormal
    (t1, t2)
}
