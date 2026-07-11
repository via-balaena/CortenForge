//! 1D Catmull-Rom radius interpolation for loft profiles.
//!
//! Shared by the loft SDF evaluator ([`eval_loft`](crate::evaluate)) and its
//! gradient ([`grad_loft`](crate::gradient)) so both read the profile through
//! one definition. `stations` is `[[z, radius], ...]` sorted by z, with at
//! least two entries.

/// Catmull-Rom cubic interpolation of radius along a loft profile.
///
/// `stations` is `[[z, radius], ...]` sorted by z, with at least 2 entries.
/// Returns the interpolated radius at the given z (clamped to station range).
pub fn loft_radius_at(stations: &[[f64; 2]], z: f64) -> f64 {
    let n = stations.len();
    debug_assert!(n >= 2);

    let z_clamped = z.clamp(stations[0][0], stations[n - 1][0]);

    // Find span: last station where z_clamped >= station z
    let mut span = 0;
    for (i, station) in stations[..n - 1].iter().enumerate() {
        if z_clamped >= station[0] {
            span = i;
        }
    }
    span = span.min(n - 2);

    let z0 = stations[span][0];
    let z1 = stations[span + 1][0];
    let dz = z1 - z0;
    if dz < 1e-15 {
        return stations[span][1];
    }
    let t = ((z_clamped - z0) / dz).clamp(0.0, 1.0);

    // Catmull-Rom 4-point stencil (endpoint repetition for boundaries)
    let r_m1 = stations[span.saturating_sub(1)][1];
    let r0 = stations[span][1];
    let r1 = stations[span + 1][1];
    let r2 = stations[(span + 2).min(n - 1)][1];

    catmull_rom_1d(r_m1, r0, r1, r2, t)
}

/// Derivative dR/dz of the loft radius interpolation at a given z.
///
/// `stations` layout matches [`loft_radius_at`]. Zero outside the station
/// range and across a degenerate (zero-width) span.
pub fn loft_radius_deriv(stations: &[[f64; 2]], z: f64) -> f64 {
    let n = stations.len();
    debug_assert!(n >= 2);

    let z_clamped = z.clamp(stations[0][0], stations[n - 1][0]);

    let mut span = 0;
    for (i, station) in stations[..n - 1].iter().enumerate() {
        if z_clamped >= station[0] {
            span = i;
        }
    }
    span = span.min(n - 2);

    let z0 = stations[span][0];
    let z1 = stations[span + 1][0];
    let dz = z1 - z0;
    if dz < 1e-15 {
        return 0.0;
    }
    let t = ((z_clamped - z0) / dz).clamp(0.0, 1.0);

    let r_m1 = stations[span.saturating_sub(1)][1];
    let r0 = stations[span][1];
    let r1 = stations[span + 1][1];
    let r2 = stations[(span + 2).min(n - 1)][1];

    catmull_rom_1d_deriv(r_m1, r0, r1, r2, t) / dz
}

/// 1D Catmull-Rom interpolation (uniform α=0.5 basis).
// Short names mirror textbook / paper notation.
#[allow(clippy::many_single_char_names)]
fn catmull_rom_1d(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let t2 = t * t;
    let t3 = t2 * t;
    // C(t) = 0.5 * (2P1 + (-P0+P2)t + (2P0-5P1+4P2-P3)t² + (-P0+3P1-3P2+P3)t³)
    let linear = 2.0f64.mul_add(p1, (-p0 + p2) * t);
    let quad = 2.0f64.mul_add(p0, 4.0f64.mul_add(p2, -(5.0 * p1) - p3)) * t2;
    let cubic = 3.0f64.mul_add(p1, 3.0f64.mul_add(-p2, -p0 + p3)) * t3;
    0.5 * (linear + quad + cubic)
}

/// 1D Catmull-Rom first derivative.
// Short names mirror textbook / paper notation.
#[allow(clippy::many_single_char_names)]
fn catmull_rom_1d_deriv(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let t2 = t * t;
    let linear = -p0 + p2;
    let quad = 2.0 * 2.0f64.mul_add(p0, 4.0f64.mul_add(p2, -(5.0 * p1) - p3)) * t;
    let cubic = 3.0 * 3.0f64.mul_add(p1, 3.0f64.mul_add(-p2, -p0 + p3)) * t2;
    0.5 * (linear + quad + cubic)
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    const EPS: f64 = 1e-7;

    #[test]
    fn catmull_rom_1d_endpoints() {
        // At t=0, result should be p1; at t=1, result should be p2
        assert_abs_diff_eq!(catmull_rom_1d(0.0, 1.0, 3.0, 4.0, 0.0), 1.0, epsilon = EPS);
        assert_abs_diff_eq!(catmull_rom_1d(0.0, 1.0, 3.0, 4.0, 1.0), 3.0, epsilon = EPS);
    }

    #[test]
    fn catmull_rom_1d_linear_for_uniform_spacing() {
        // For equally spaced values (0, 1, 2, 3), midpoint should be 1.5
        assert_abs_diff_eq!(catmull_rom_1d(0.0, 1.0, 2.0, 3.0, 0.5), 1.5, epsilon = EPS);
    }
}
