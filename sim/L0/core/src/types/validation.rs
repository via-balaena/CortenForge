//! Numeric validation utilities for simulation state.
//!
//! Contains MuJoCo-aligned detection primitives for NaN, infinity,
//! and divergence (values exceeding `MAX_VAL`).

/// Minimum meaningful value — values below this are treated as zero.
/// Matches MuJoCo's `mjMINVAL = 1e-15`. Used for gravity-norm checks
/// and other near-zero guards.
pub const MIN_VAL: f64 = 1e-15;

/// Maximum allowed value in qpos, qvel, qacc (matches MuJoCo's `mjMAXVAL`).
pub const MAX_VAL: f64 = 1e10;

/// Returns true if value is NaN, +inf, -inf, or exceeds `MAX_VAL`.
/// Matches MuJoCo's `mju_isBad()`.
#[inline]
#[must_use]
pub fn is_bad(x: f64) -> bool {
    // Note: deliberately NOT using RangeInclusive::contains() — NaN fails
    // range checks silently (NaN is not <=, >=, or contained), so the
    // explicit is_nan() check is required for correctness.
    #[allow(clippy::manual_range_contains)]
    {
        x.is_nan() || x > MAX_VAL || x < -MAX_VAL
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_bad() {
        assert!(is_bad(f64::NAN));
        assert!(is_bad(f64::INFINITY));
        assert!(is_bad(f64::NEG_INFINITY));
        assert!(is_bad(1.1e10));
        assert!(is_bad(-1.1e10));
        assert!(!is_bad(0.0));
        assert!(!is_bad(1.0));
        assert!(!is_bad(-1.0));
        assert!(!is_bad(1e10)); // exactly MAX_VAL is not bad
        assert!(!is_bad(-1e10));
    }
}
