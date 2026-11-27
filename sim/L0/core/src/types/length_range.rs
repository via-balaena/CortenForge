//! An actuator's length range as MuJoCo 3.5.0's compiler computes it: the
//! options (`mjLROpt`) and the errors that refuse a model. The computation is
//! [`Model::set_length_range`](crate::Model::set_length_range).

use std::fmt;

use super::enums::StepError;

/// The options of the length-range computation, MuJoCo 3.5.0's `mjLROpt`
/// (`mjmodel.h:477-491`), with its defaults (`engine_init.c:234-246`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LengthRangeOpt {
    /// Which actuators get a computed range.
    pub mode: LengthRangeMode,
    /// Keep an actuator's range where its lower bound is below its upper.
    pub useexisting: bool,
    /// Copy a limited joint's or tendon's range, as written (no gear).
    pub uselimit: bool,
    /// The acceleration the applied force gives, in the run.
    pub accel: f64,
    /// A cap on the applied force's norm; 0 for none.
    pub maxforce: f64,
    /// The time constant of the velocity's damping, in seconds.
    pub timeconst: f64,
    /// The run's timestep; the model's where not positive.
    pub timestep: f64,
    /// The time each side runs, in seconds.
    pub inttotal: f64,
    /// The last stretch of each side whose lengths are measured, in seconds.
    pub interval: f64,
    /// How far a side's length may move over `interval`, as a fraction of
    /// the range.
    pub tolrange: f64,
}

impl Default for LengthRangeOpt {
    fn default() -> Self {
        Self {
            mode: LengthRangeMode::Muscle,
            useexisting: true,
            uselimit: false,
            accel: 20.0,
            maxforce: 0.0,
            timeconst: 1.0,
            timestep: 0.01,
            inttotal: 10.0,
            interval: 2.0,
            tolrange: 0.05,
        }
    }
}

/// Which actuators get a computed range, MuJoCo's `mjtLRMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LengthRangeMode {
    /// None.
    None,
    /// An actuator whose gain or bias type is `Muscle` (the default).
    #[default]
    Muscle,
    /// An actuator whose gain or bias type is `Muscle` or `User`.
    MuscleUser,
    /// Every actuator.
    All,
}

/// Why an actuator's length range refuses its model.
///
/// `mj_setLengthRange`'s three errors (`engine_setconst.c:1215-1266`), each
/// displayed in its words with its numbers as C's `%g` prints them, and a step
/// this crate refuses.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum LengthRangeError {
    /// A step left the time at 0: "Unstable lengthrange simulation in
    /// actuator N". MuJoCo's comes from a zero timestep. This crate's also
    /// from a step that resets the state, where MuJoCo restarts the side from
    /// `qpos0` (registry `D-LENGTHRANGE-RESET`).
    Unstable {
        /// The actuator.
        actuator: usize,
    },
    /// The range the run found is empty or backwards: "Invalid lengthrange
    /// (lo, hi) in actuator N".
    InvalidRange {
        /// The actuator.
        actuator: usize,
        /// The range found.
        range: (f64, f64),
    },
    /// A side's length moved more than `tolrange` of the range over the last
    /// `interval`: "Lengthrange computation did not converge in actuator N:"
    /// with the side's lengths (`eval`) and the range.
    NotConverged {
        /// The actuator.
        actuator: usize,
        /// The side that moved: 0 pushes the length down, 1 up.
        side: usize,
        /// The side's least and greatest length over the last `interval`.
        eval: (f64, f64),
        /// The range found.
        range: (f64, f64),
    },
    /// `inttotal` is infinite where an actuator needs a run, which then never
    /// ends, MuJoCo's too (registry `D-NONFINITE`). This crate's words;
    /// MuJoCo has none.
    InfiniteInttotal {
        /// The actuator.
        actuator: usize,
    },
    /// A step of the run refused the model (MuJoCo's own step stops on some
    /// of these: a discrete inverse dynamics under RK4).
    Step {
        /// The actuator.
        actuator: usize,
        /// The step's refusal.
        error: StepError,
    },
}

impl fmt::Display for LengthRangeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::Unstable { actuator } => {
                write!(f, "Unstable lengthrange simulation in actuator {actuator}")
            }
            Self::InvalidRange { actuator, range } => write!(
                f,
                "Invalid lengthrange ({}, {}) in actuator {actuator}",
                c_g(range.0),
                c_g(range.1)
            ),
            // Side 1's range line starts with one space, side 0's with two,
            // as MuJoCo's two messages do (`engine_setconst.c:1251`, `:1262`).
            Self::NotConverged {
                actuator,
                side,
                eval,
                range,
            } => write!(
                f,
                "Lengthrange computation did not converge in actuator {actuator}:\n  eval ({}, {})\n{}range ({}, {})",
                c_g(eval.0),
                c_g(eval.1),
                if side == 0 { "  " } else { " " },
                c_g(range.0),
                c_g(range.1)
            ),
            Self::InfiniteInttotal { actuator } => write!(
                f,
                "Lengthrange simulation in actuator {actuator} never ends: inttotal is infinite"
            ),
            Self::Step { actuator, error } => {
                write!(f, "Lengthrange simulation in actuator {actuator}: {error}")
            }
        }
    }
}

impl std::error::Error for LengthRangeError {}

/// `x` as C's `printf("%g")` prints it: six significant digits, fixed where
/// the decimal exponent is from -4 to 5 and `e±dd` otherwise, trailing zeros
/// and a bare point dropped.
fn c_g(x: f64) -> String {
    if x.is_nan() {
        return if x.is_sign_negative() { "-nan" } else { "nan" }.to_owned();
    }
    if x.is_infinite() {
        return if x < 0.0 { "-inf" } else { "inf" }.to_owned();
    }
    if x == 0.0 {
        return if x.is_sign_negative() { "-0" } else { "0" }.to_owned();
    }
    // The exponent `%e` prints at five decimals (its rounding can carry it).
    let e = format!("{x:.5e}");
    let (mantissa, exp) = e.split_once('e').unwrap_or((&e, "0"));
    let exp: i32 = exp.parse().unwrap_or(0);
    let trim = |s: &str| -> String {
        if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_owned()
        } else {
            s.to_owned()
        }
    };
    if (-4..6).contains(&exp) {
        // The precision is 5 - exp, from 0 to 9.
        #[allow(clippy::cast_sign_loss)]
        let precision = (5 - exp) as usize;
        trim(&format!("{x:.precision$}"))
    } else {
        let sign = if exp < 0 { '-' } else { '+' };
        format!("{}e{sign}{:02}", trim(mantissa), exp.unsigned_abs())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// C's `%g`, each value printed by Python's `"%g" % x` (C's rules).
    #[test]
    fn c_g_prints_as_c_does() {
        for (x, want) in [
            (0.0, "0"),
            (-0.0, "-0"),
            (-181.002_741_320_750_7, "-181.003"),
            (-140.808_209_193_592_23, "-140.808"),
            (0.3, "0.3"),
            (1e-5, "1e-05"),
            (0.0001, "0.0001"),
            (1_234_567.0, "1.23457e+06"),
            (100_000.0, "100000"),
            (999_999.5, "1e+06"),
            (0.000_099_999_95, "0.0001"),
            (-0.556_633_2, "-0.556633"),
            (1.0e300, "1e+300"),
            (f64::INFINITY, "inf"),
            (f64::NEG_INFINITY, "-inf"),
            (f64::NAN, "nan"),
        ] {
            assert_eq!(c_g(x), want, "{x:e}");
        }
    }

    /// MuJoCo's three messages, as its Python bindings print them (after
    /// "Error: "), on its unlimited-hinge, coincident-site and in-line-rim
    /// documents (RL-11a's start set: m16, t06, x06).
    #[test]
    fn the_errors_read_as_mujocos() {
        let not_converged = LengthRangeError::NotConverged {
            actuator: 0,
            side: 0,
            eval: (-181.002_741_320_750_7, -140.808_209_193_592_23),
            range: (-181.002_741_320_750_7, 181.002_741_320_750_7),
        };
        assert_eq!(
            not_converged.to_string(),
            "Lengthrange computation did not converge in actuator 0:\n  eval (-181.003, -140.808)\n  range (-181.003, 181.003)"
        );
        let side1 = LengthRangeError::NotConverged {
            actuator: 0,
            side: 1,
            eval: (140.808_209_193_592_23, 181.002_741_320_750_7),
            range: (-181.002_741_320_750_7, 181.002_741_320_750_7),
        };
        assert!(side1.to_string().ends_with("\n range (-181.003, 181.003)"));
        assert_eq!(
            LengthRangeError::InvalidRange {
                actuator: 0,
                range: (0.3, 0.3)
            }
            .to_string(),
            "Invalid lengthrange (0.3, 0.3) in actuator 0"
        );
        assert_eq!(
            LengthRangeError::Unstable { actuator: 2 }.to_string(),
            "Unstable lengthrange simulation in actuator 2"
        );
    }
}
