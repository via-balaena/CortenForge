//! Quadric error metric implementation.
//!
//! The quadric error metric (QEM) measures the distance from a point to a set of planes.
//! It's used to determine the optimal position for merged vertices during edge collapse.

/// Quadric error matrix (4x4 symmetric matrix stored as 10 values).
///
/// Represents the error metric for a vertex based on the planes of its adjacent faces.
#[derive(Debug, Clone, Copy)]
pub struct Quadric {
    // Symmetric 4x4 matrix stored as upper triangle:
    // [a b c d]
    // [  e f g]
    // [    h i]
    // [      j]
    a: f64,
    b: f64,
    c: f64,
    d: f64,
    e: f64,
    f: f64,
    g: f64,
    h: f64,
    i: f64,
    j: f64,
}

impl Default for Quadric {
    fn default() -> Self {
        Self {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            d: 0.0,
            e: 0.0,
            f: 0.0,
            g: 0.0,
            h: 0.0,
            i: 0.0,
            j: 0.0,
        }
    }
}

impl Quadric {
    /// Create a quadric from a plane equation (ax + by + cz + d = 0).
    ///
    /// The plane should have a normalized normal vector (a, b, c).
    #[must_use]
    pub fn from_plane(a: f64, b: f64, c: f64, d: f64) -> Self {
        Self {
            a: a * a,
            b: a * b,
            c: a * c,
            d: a * d,
            e: b * b,
            f: b * c,
            g: b * d,
            h: c * c,
            i: c * d,
            j: d * d,
        }
    }

    /// Add another quadric to this one.
    pub fn add(&mut self, other: &Self) {
        self.a += other.a;
        self.b += other.b;
        self.c += other.c;
        self.d += other.d;
        self.e += other.e;
        self.f += other.f;
        self.g += other.g;
        self.h += other.h;
        self.i += other.i;
        self.j += other.j;
    }

    /// Evaluate the quadric error for a point.
    ///
    /// Returns the sum of squared distances from the point to all planes
    /// that contributed to this quadric.
    #[must_use]
    pub fn evaluate(&self, x: f64, y: f64, z: f64) -> f64 {
        // v^T * Q * v where v = [x, y, z, 1]
        x.mul_add(
            x.mul_add(self.a, 2.0 * y.mul_add(self.b, z.mul_add(self.c, self.d))),
            y.mul_add(
                y.mul_add(self.e, 2.0 * z.mul_add(self.f, self.g)),
                z.mul_add(z.mul_add(self.h, 2.0 * self.i), self.j),
            ),
        )
    }

    /// Find the optimal point that minimizes error, or return None if matrix is singular.
    ///
    /// Solves the linear system to find the point with minimum quadric error.
    #[must_use]
    pub fn optimal_point(&self) -> Option<[f64; 3]> {
        // Solve the linear system:
        // [a b c] [x]   [-d]
        // [b e f] [y] = [-g]
        // [c f h] [z]   [-i]

        let det = self.a.mul_add(
            self.f.mul_add(-self.f, self.e * self.h),
            self.b.mul_add(
                self.c.mul_add(self.f, -self.b * self.h),
                self.c * self.e.mul_add(-self.c, self.b * self.f),
            ),
        );

        if det.abs() < 1e-10 {
            return None;
        }

        let inv_det = 1.0 / det;

        // Compute inverse matrix elements
        let m00 = self.f.mul_add(-self.f, self.e * self.h) * inv_det;
        let m01 = self.c.mul_add(self.f, -self.b * self.h) * inv_det;
        let m02 = self.c.mul_add(-self.e, self.b * self.f) * inv_det;
        let m11 = self.c.mul_add(-self.c, self.a * self.h) * inv_det;
        let m12 = self.b.mul_add(self.c, -self.a * self.f) * inv_det;
        let m22 = self.b.mul_add(-self.b, self.a * self.e) * inv_det;

        let x = m00.mul_add(-self.d, m01.mul_add(-self.g, m02 * -self.i));
        let y = m01.mul_add(-self.d, m11.mul_add(-self.g, m12 * -self.i));
        let z = m02.mul_add(-self.d, m12.mul_add(-self.g, m22 * -self.i));

        Some([x, y, z])
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn test_quadric_default() {
        let q = Quadric::default();
        // Zero quadric should evaluate to 0 for any point
        assert!((q.evaluate(1.0, 2.0, 3.0)).abs() < f64::EPSILON);
    }

    #[test]
    fn test_quadric_from_plane() {
        // Plane z = 0 (normal = [0, 0, 1], d = 0)
        let q = Quadric::from_plane(0.0, 0.0, 1.0, 0.0);

        // Points on the plane should have zero error
        assert!((q.evaluate(1.0, 2.0, 0.0)).abs() < 1e-10);

        // Point at z=1 should have error = 1 (distance^2 = 1)
        assert!((q.evaluate(0.0, 0.0, 1.0) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_quadric_add() {
        let q1 = Quadric::from_plane(0.0, 0.0, 1.0, 0.0);
        let q2 = Quadric::from_plane(0.0, 1.0, 0.0, 0.0);

        let mut combined = q1;
        combined.add(&q2);

        // Point at origin should have zero error (on both planes)
        assert!((combined.evaluate(0.0, 0.0, 0.0)).abs() < 1e-10);
    }

    #[test]
    fn test_optimal_point() {
        // Create quadric from three planes meeting at origin
        let mut q = Quadric::from_plane(1.0, 0.0, 0.0, 0.0);
        q.add(&Quadric::from_plane(0.0, 1.0, 0.0, 0.0));
        q.add(&Quadric::from_plane(0.0, 0.0, 1.0, 0.0));

        let optimal = q.optimal_point();
        assert!(optimal.is_some());

        let [x, y, z] = optimal.unwrap();
        assert!((x).abs() < 1e-10);
        assert!((y).abs() < 1e-10);
        assert!((z).abs() < 1e-10);
    }
}
