//! Contact-related types for constraint generation.
//!
//! Contains [`ContactPair`] (explicit pair overrides from MJCF `<pair>`),
//! [`Contact`] (detected contact point for constraint generation),
//! and the [`make_frame`] helper.

use nalgebra::Vector3;

use super::model::Model;
use crate::constraint::impedance::{DEFAULT_SOLIMP, DEFAULT_SOLREF};
use crate::quat::normalize3;

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
    /// Contact normal: from geom1 toward geom2, unit as `make_frame`
    /// normalises it; one shorter than 0.5 is kept as written and refuses
    /// the step
    /// ([`StepError::UndefinedContactFrame`](crate::StepError::UndefinedContactFrame)).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = penetrating).
    pub depth: f64,
    /// First geometry ID. A contact between two geoms lists them in
    /// MuJoCo's order (`mj_collideGeoms`): the lower MuJoCo geom type first
    /// (plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh, sdf),
    /// for equal types the lower body's geom first (an explicit pair whose
    /// geoms share a body: as declared).
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
    /// `[0.0, 0.0]` is the sentinel meaning "use `solref`" (MuJoCo convention).
    /// Auto-generated contacts have `[0.0, 0.0]`, but under `ENABLE_OVERRIDE`
    /// `o_solref`.
    ///
    /// When nonzero **and** the constraint is `ContactElliptic` with `row > 0`
    /// (friction row), the KBIP computation uses `solreffriction` instead of
    /// `solref` for the B (damping) term. K is always 0 for friction rows. A
    /// mixed one (one value positive, the other not) is taken as `[0.0, 0.0]`
    /// when the rows are made, as MuJoCo's `getsolparam`.
    pub solreffriction: [f64; 2],
    /// Solver impedance parameters (from geom pair).
    pub solimp: [f64; 5],
    /// Friction of the regularized cone, `mu[0]·√(R[1]/R[0])` over the
    /// contact's first two rows, set when its rows are made; 0 until then,
    /// and for a contact without friction rows (MuJoCo's `mjContact::mu`, 0
    /// from collision, `engine_collision_driver.c:1422`, set by
    /// `mj_makeImpedance`, `engine_core_constraint.c:1540`).
    pub regularized_mu: f64,
    /// Contact frame tangent vectors (orthogonal to normal).
    /// `frame[0..3]` = t1, `frame[3..6]` = t2 (for friction cone).
    pub frame: [Vector3<f64>; 2],
    /// If `Some(vertex_idx)`, this is a flex-rigid contact.
    /// `geom1` and `geom2` both reference the rigid geom index.
    /// The flex vertex index is stored here. If `None`, standard rigid-rigid contact.
    pub flex_vertex: Option<usize>,
    /// If `Some(vertex_idx)`, this is a flex self-collision contact.
    /// `flex_vertex` = penetrating vertex (side 1).
    /// `flex_vertex2` = nearest opposing vertex (side 2).
    /// `geom1 = geom2 = usize::MAX` (no rigid geom).
    /// If `None`, standard flex-rigid or rigid-rigid contact.
    pub flex_vertex2: Option<usize>,
}

impl Contact {
    /// Derive the two body indices for this contact.
    ///
    /// For rigid-rigid: `(geom_body[geom1], geom_body[geom2])`.
    /// For flex-rigid: `(flexvert_bodyid[vertex], geom_body[geom2])`.
    /// For flex-self/flex-flex: `(flexvert_bodyid[v1], flexvert_bodyid[v2])`.
    ///
    /// Returns world body (0) for out-of-bounds geom indices (defensive
    /// against synthetic test contacts with sentinel geom values).
    #[inline]
    #[must_use]
    pub fn bodies(&self, model: &Model) -> (usize, usize) {
        match (self.flex_vertex, self.flex_vertex2) {
            (Some(vi1), Some(vi2)) => {
                // Flex self-collision or flex-flex
                (model.flexvert_bodyid[vi1], model.flexvert_bodyid[vi2])
            }
            (Some(vi), None) => {
                // Flex-rigid: vertex body + rigid geom body
                (model.flexvert_bodyid[vi], model.geom_body[self.geom2])
            }
            _ => {
                // Rigid-rigid (with bounds guard for synthetic contacts)
                let b1 = if self.geom1 < model.geom_body.len() {
                    model.geom_body[self.geom1]
                } else {
                    0
                };
                let b2 = if self.geom2 < model.geom_body.len() {
                    model.geom_body[self.geom2]
                } else {
                    0
                };
                (b1, b2)
            }
        }
    }

    /// Whether the contact lies in the gap: its distance at or past
    /// `includemargin` (margin − gap), MuJoCo's `exclude` (`mj_setContact`,
    /// `engine_collision_driver.c:1414-1415`). An excluded contact is listed
    /// (it counts in `ncon`, wakes the trees it joins and counts for an
    /// adhesion actuator) but makes no constraint rows.
    #[must_use]
    pub fn is_excluded(&self) -> bool {
        -self.depth >= self.includemargin
    }

    /// Create a new contact with basic parameters.
    ///
    /// The contact frame (tangent vectors) is computed automatically from the normal.
    /// Advanced solver parameters use MuJoCo defaults.
    ///
    /// # Numerical Safety
    /// - Negative or NaN friction is clamped to 0.0
    /// - NaN depth is set to 0.0
    /// - A normal that is not finite gives a frame that is not (`make_frame`)
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
    /// - A normal that is not finite gives a frame that is not (`make_frame`)
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

        // The normal normalised and the frame completed, as MuJoCo's
        // mj_setContact (one shorter than 0.5 is kept, for the step to refuse).
        let (normal, frame) = make_frame(&normal, &Vector3::zeros());

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
            frame,
            flex_vertex: None,
            regularized_mu: 0.0,
            flex_vertex2: None,
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
    // Contact builder takes the full contact context (geom indices, poses, normal, depth, friction).
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

        // The normal normalised and the frame completed, as MuJoCo's
        // mj_setContact.
        let (normal, frame) = make_frame(&normal, &Vector3::zeros());

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
            frame,
            flex_vertex: None,
            regularized_mu: 0.0,
            flex_vertex2: None,
        }
    }
}

/// The contact frame as MuJoCo's `mju_makeFrame` completes it
/// (`engine_util_spatial.c:508-534`), which `mj_setContact` calls for every
/// contact (`engine_collision_driver.c:1418`): the normal normalised as
/// `mju_normalize3`; the first tangent `hint` if its squared length is at
/// least 0.25, else `(0, 1, 0)` when the normal's `y` lies in (−0.5, 0.5)
/// and `(0, 0, 1)` otherwise, made orthogonal to the normal and normalised;
/// the second the normal × the first. Returns the normal and the tangents.
///
/// MuJoCo stops on a normal shorter than 0.5 ("xaxis of contact frame
/// undefined", `engine_util_spatial.c:511-513`). Such a normal is kept as
/// written, with the tangents `(1, 0, 0)` and `(0, 1, 0)`, and the position
/// stage refuses it
/// ([`StepError::UndefinedContactFrame`](crate::StepError::UndefinedContactFrame)).
/// A normal that is not finite passes that test, as MuJoCo's does, and its
/// frame is not finite either.
#[must_use]
pub fn make_frame(normal: &Vector3<f64>, hint: &Vector3<f64>) -> (Vector3<f64>, [Vector3<f64>; 2]) {
    let dot = |a: &[f64; 3], b: &[f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let mut n = [normal.x, normal.y, normal.z];
    let length = dot(&n, &n).sqrt();
    if length < 0.5 {
        return (*normal, [Vector3::x(), Vector3::y()]);
    }
    normalize3(&mut n);
    let h = [hint.x, hint.y, hint.z];
    let mut t1 = if dot(&h, &h) < 0.25 {
        if n[1] < 0.5 && n[1] > -0.5 {
            [0.0, 1.0, 0.0]
        } else {
            [0.0, 0.0, 1.0]
        }
    } else {
        h
    };
    // The seed less its part along the normal, MuJoCo's operations in order.
    let along = dot(&n, &t1);
    t1 = [
        t1[0] - n[0] * along,
        t1[1] - n[1] * along,
        t1[2] - n[2] * along,
    ];
    normalize3(&mut t1);
    let t2 = [
        n[1] * t1[2] - n[2] * t1[1],
        n[2] * t1[0] - n[0] * t1[2],
        n[0] * t1[1] - n[1] * t1[0],
    ];
    (Vector3::from(n), [Vector3::from(t1), Vector3::from(t2)])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `mju_makeFrame` seeds the first tangent with (0, 1, 0) only while the
    /// normal's `y` lies strictly inside (−0.5, 0.5): (1, ±1, √2), whose
    /// unit normal's `y` is ±0.5 exactly, is seeded with (0, 0, 1), so its
    /// frame is the one (0, 0, 1) given as the hint builds, bit for bit.
    #[test]
    fn a_normal_at_y_half_seeds_the_z_axis() {
        for y in [1.0, -1.0] {
            let normal = Vector3::new(1.0, y, 2.0_f64.sqrt());
            let (unit, frame) = make_frame(&normal, &Vector3::zeros());
            assert_eq!(unit.y.to_bits(), (0.5 * y).to_bits(), "the unit normal's y");
            let (_, seeded) = make_frame(&normal, &Vector3::z());
            let bits = |v: &Vector3<f64>| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
            for k in 0..2 {
                assert_eq!(bits(&frame[k]), bits(&seeded[k]), "y {y}: tangent {k}");
            }
            let (_, other) = make_frame(&normal, &Vector3::y());
            assert_ne!(
                frame[0], other[0],
                "y {y}: the (0, 1, 0) seed gives the same"
            );
        }
    }

    /// MuJoCo's `mj_setContact` normalises the normal a collision function
    /// wrote (`mju_makeFrame`): (0, 1, 1) is stored as
    /// (0, 0.7071067811865475, 0.7071067811865475).
    #[test]
    fn contact_normal_is_normalised_as_mj_setcontact() {
        let c = Contact::with_condim(
            Vector3::zeros(),
            Vector3::new(0.0, 1.0, 1.0),
            0.01,
            0,
            1,
            1.0,
            0.005,
            0.0001,
            3,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        assert_eq!(
            c.normal,
            Vector3::new(0.0, 0.707_106_781_186_547_5, 0.707_106_781_186_547_5)
        );
        assert_eq!(
            c.frame,
            make_frame(&Vector3::new(0.0, 1.0, 1.0), &Vector3::zeros()).1
        );
    }

    /// Where MuJoCo's `mju_makeFrame` stops, a normal shorter than 0.5 is kept
    /// as written, for the position stage to refuse; from 0.5 on it is
    /// normalised.
    #[test]
    fn a_normal_shorter_than_half_is_kept_as_written() {
        for normal in [
            Vector3::zeros(),
            Vector3::new(0.0, 0.3, 0.0),
            Vector3::new(0.0, 0.0, 0.499_999),
        ] {
            let (n, frame) = make_frame(&normal, &Vector3::zeros());
            assert_eq!(n, normal);
            assert_eq!(frame, [Vector3::x(), Vector3::y()]);
        }
        assert_eq!(
            make_frame(&Vector3::new(0.0, 0.0, 0.5), &Vector3::zeros()).0,
            Vector3::z()
        );
    }

    /// A normal that is not finite passes MuJoCo's test (`NaN < 0.5` is
    /// false), and `mju_normalize3` and `mju_makeFrame`'s arithmetic give a
    /// frame that is not finite.
    #[test]
    fn a_normal_that_is_not_finite_gives_a_frame_that_is_not() {
        for normal in [
            Vector3::new(f64::NAN, 0.0, 1.0),
            Vector3::new(f64::INFINITY, 0.0, 0.0),
        ] {
            let (n, frame) = make_frame(&normal, &Vector3::zeros());
            assert!(n.x.is_nan(), "{n:?}");
            for t in frame {
                assert!(t.iter().any(|x| x.is_nan()), "{t:?}");
            }
        }
    }
}
