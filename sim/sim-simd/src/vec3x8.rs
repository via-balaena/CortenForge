//! SIMD-optimized 8-wide `Vector3` batch type.
//!
//! [`Vec3x8`] stores 8 `Vector3<f64>` values in a structure-of-arrays (`SoA`) layout
//! optimized for SIMD operations on 512-bit registers (`AVX-512`).

use nalgebra::Vector3;

/// A batch of 8 `Vector3<f64>` values stored in `SoA` (Structure of Arrays) layout.
///
/// This layout enables efficient SIMD operations by grouping x, y, and z
/// components together, allowing 8 operations to execute simultaneously
/// on AVX-512 capable processors.
///
/// # Memory Layout
///
/// ```text
/// xs: [x0, x1, x2, x3, x4, x5, x6, x7]  <- 8 x components (512 bits / 64 bytes)
/// ys: [y0, y1, y2, y3, y4, y5, y6, y7]  <- 8 y components (512 bits / 64 bytes)
/// zs: [z0, z1, z2, z3, z4, z5, z6, z7]  <- 8 z components (512 bits / 64 bytes)
/// ```
///
/// # Example
///
/// ```
/// use sim_simd::Vec3x8;
/// use nalgebra::Vector3;
///
/// let batch = Vec3x8::splat(Vector3::new(1.0, 2.0, 3.0));
///
/// // All 8 vectors are the same
/// let norms_sq = batch.norm_squared();
/// for norm_sq in norms_sq {
///     assert_eq!(norm_sq, 14.0);  // 1² + 2² + 3²
/// }
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C, align(64))]
pub struct Vec3x8 {
    /// X components of all 8 vectors.
    pub xs: [f64; 8],
    /// Y components of all 8 vectors.
    pub ys: [f64; 8],
    /// Z components of all 8 vectors.
    pub zs: [f64; 8],
}

impl Default for Vec3x8 {
    fn default() -> Self {
        Self::zeros()
    }
}

impl Vec3x8 {
    /// Create a batch of 8 zero vectors.
    #[must_use]
    #[inline]
    pub const fn zeros() -> Self {
        Self {
            xs: [0.0; 8],
            ys: [0.0; 8],
            zs: [0.0; 8],
        }
    }

    /// Create from 8 individual vectors.
    #[must_use]
    #[inline]
    pub fn from_vectors(vectors: [Vector3<f64>; 8]) -> Self {
        Self {
            xs: [
                vectors[0].x,
                vectors[1].x,
                vectors[2].x,
                vectors[3].x,
                vectors[4].x,
                vectors[5].x,
                vectors[6].x,
                vectors[7].x,
            ],
            ys: [
                vectors[0].y,
                vectors[1].y,
                vectors[2].y,
                vectors[3].y,
                vectors[4].y,
                vectors[5].y,
                vectors[6].y,
                vectors[7].y,
            ],
            zs: [
                vectors[0].z,
                vectors[1].z,
                vectors[2].z,
                vectors[3].z,
                vectors[4].z,
                vectors[5].z,
                vectors[6].z,
                vectors[7].z,
            ],
        }
    }

    /// Create from a slice of vectors.
    ///
    /// # Panics
    ///
    /// Panics if the slice has fewer than 8 elements.
    #[must_use]
    #[inline]
    pub fn from_slice(vectors: &[Vector3<f64>]) -> Self {
        debug_assert!(vectors.len() >= 8, "Need at least 8 vectors");
        Self {
            xs: [
                vectors[0].x,
                vectors[1].x,
                vectors[2].x,
                vectors[3].x,
                vectors[4].x,
                vectors[5].x,
                vectors[6].x,
                vectors[7].x,
            ],
            ys: [
                vectors[0].y,
                vectors[1].y,
                vectors[2].y,
                vectors[3].y,
                vectors[4].y,
                vectors[5].y,
                vectors[6].y,
                vectors[7].y,
            ],
            zs: [
                vectors[0].z,
                vectors[1].z,
                vectors[2].z,
                vectors[3].z,
                vectors[4].z,
                vectors[5].z,
                vectors[6].z,
                vectors[7].z,
            ],
        }
    }

    /// Create from a slice, padding with zeros if fewer than 8 vectors.
    #[must_use]
    #[inline]
    pub fn from_slice_padded(vectors: &[Vector3<f64>]) -> Self {
        let mut result = Self::zeros();
        for (i, v) in vectors.iter().take(8).enumerate() {
            result.xs[i] = v.x;
            result.ys[i] = v.y;
            result.zs[i] = v.z;
        }
        result
    }

    /// Create by broadcasting a single vector to all 8 lanes.
    #[must_use]
    #[inline]
    pub fn splat(v: Vector3<f64>) -> Self {
        Self {
            xs: [v.x; 8],
            ys: [v.y; 8],
            zs: [v.z; 8],
        }
    }

    /// Extract the vector at a given index.
    #[must_use]
    #[inline]
    pub fn get(&self, index: usize) -> Vector3<f64> {
        debug_assert!(index < 8);
        Vector3::new(self.xs[index], self.ys[index], self.zs[index])
    }

    /// Convert to an array of 8 vectors.
    #[must_use]
    #[inline]
    pub fn to_vectors(&self) -> [Vector3<f64>; 8] {
        [
            self.get(0),
            self.get(1),
            self.get(2),
            self.get(3),
            self.get(4),
            self.get(5),
            self.get(6),
            self.get(7),
        ]
    }

    /// Compute dot product of each vector with a single direction vector.
    ///
    /// Returns 8 dot products simultaneously.
    #[must_use]
    #[inline]
    pub fn dot(&self, direction: &Vector3<f64>) -> [f64; 8] {
        let mut result = [0.0; 8];

        // SIMD-friendly: separate loops for each component
        for i in 0..8 {
            result[i] = self.xs[i] * direction.x;
        }
        for i in 0..8 {
            result[i] += self.ys[i] * direction.y;
        }
        for i in 0..8 {
            result[i] += self.zs[i] * direction.z;
        }

        result
    }

    /// Compute pairwise dot products between this batch and another.
    #[must_use]
    #[inline]
    pub fn dot_pairwise(&self, other: &Self) -> [f64; 8] {
        let mut result = [0.0; 8];

        for i in 0..8 {
            result[i] = self.xs[i] * other.xs[i];
        }
        for i in 0..8 {
            result[i] += self.ys[i] * other.ys[i];
        }
        for i in 0..8 {
            result[i] += self.zs[i] * other.zs[i];
        }

        result
    }

    /// Compute squared norm of each vector.
    #[must_use]
    #[inline]
    pub fn norm_squared(&self) -> [f64; 8] {
        self.dot_pairwise(self)
    }

    /// Compute norm of each vector.
    #[must_use]
    #[inline]
    pub fn norm(&self) -> [f64; 8] {
        let sq = self.norm_squared();
        [
            sq[0].sqrt(),
            sq[1].sqrt(),
            sq[2].sqrt(),
            sq[3].sqrt(),
            sq[4].sqrt(),
            sq[5].sqrt(),
            sq[6].sqrt(),
            sq[7].sqrt(),
        ]
    }

    /// Add two batches element-wise.
    #[must_use]
    #[inline]
    pub fn add(&self, other: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..8 {
            result.xs[i] = self.xs[i] + other.xs[i];
        }
        for i in 0..8 {
            result.ys[i] = self.ys[i] + other.ys[i];
        }
        for i in 0..8 {
            result.zs[i] = self.zs[i] + other.zs[i];
        }
        result
    }

    /// Subtract two batches element-wise.
    #[must_use]
    #[inline]
    pub fn sub(&self, other: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..8 {
            result.xs[i] = self.xs[i] - other.xs[i];
        }
        for i in 0..8 {
            result.ys[i] = self.ys[i] - other.ys[i];
        }
        for i in 0..8 {
            result.zs[i] = self.zs[i] - other.zs[i];
        }
        result
    }

    /// Multiply each vector by a scalar.
    #[must_use]
    #[inline]
    pub fn scale(&self, scalar: f64) -> Self {
        let mut result = Self::zeros();
        for i in 0..8 {
            result.xs[i] = self.xs[i] * scalar;
        }
        for i in 0..8 {
            result.ys[i] = self.ys[i] * scalar;
        }
        for i in 0..8 {
            result.zs[i] = self.zs[i] * scalar;
        }
        result
    }

    /// Multiply each vector by its corresponding scalar.
    #[must_use]
    #[inline]
    pub fn scale_each(&self, scalars: [f64; 8]) -> Self {
        let mut result = Self::zeros();
        for i in 0..8 {
            result.xs[i] = self.xs[i] * scalars[i];
        }
        for i in 0..8 {
            result.ys[i] = self.ys[i] * scalars[i];
        }
        for i in 0..8 {
            result.zs[i] = self.zs[i] * scalars[i];
        }
        result
    }

    /// Negate all vectors.
    #[must_use]
    #[inline]
    pub fn neg(&self) -> Self {
        let mut result = Self::zeros();
        for i in 0..8 {
            result.xs[i] = -self.xs[i];
        }
        for i in 0..8 {
            result.ys[i] = -self.ys[i];
        }
        for i in 0..8 {
            result.zs[i] = -self.zs[i];
        }
        result
    }

    /// Compute cross product of each vector with a single direction.
    #[must_use]
    #[inline]
    pub fn cross(&self, direction: &Vector3<f64>) -> Self {
        let mut result = Self::zeros();
        for i in 0..8 {
            result.xs[i] = self.ys[i] * direction.z - self.zs[i] * direction.y;
        }
        for i in 0..8 {
            result.ys[i] = self.zs[i] * direction.x - self.xs[i] * direction.z;
        }
        for i in 0..8 {
            result.zs[i] = self.xs[i] * direction.y - self.ys[i] * direction.x;
        }
        result
    }

    /// Compute pairwise cross products.
    #[must_use]
    #[inline]
    pub fn cross_pairwise(&self, other: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..8 {
            result.xs[i] = self.ys[i] * other.zs[i] - self.zs[i] * other.ys[i];
        }
        for i in 0..8 {
            result.ys[i] = self.zs[i] * other.xs[i] - self.xs[i] * other.zs[i];
        }
        for i in 0..8 {
            result.zs[i] = self.xs[i] * other.ys[i] - self.ys[i] * other.xs[i];
        }
        result
    }

    /// Normalize each vector.
    #[must_use]
    #[inline]
    pub fn normalize(&self) -> Self {
        let norms = self.norm();
        let mut inv_norms = [1.0; 8];
        for i in 0..8 {
            if norms[i] > 1e-10 {
                inv_norms[i] = 1.0 / norms[i];
            }
        }
        self.scale_each(inv_norms)
    }

    /// Find the index of the vector with the maximum dot product with direction.
    #[must_use]
    #[inline]
    pub fn argmax_dot(&self, direction: &Vector3<f64>) -> (usize, f64) {
        let dots = self.dot(direction);
        let mut max_idx = 0;
        let mut max_val = dots[0];
        for i in 1..8 {
            if dots[i] > max_val {
                max_val = dots[i];
                max_idx = i;
            }
        }
        (max_idx, max_val)
    }

    /// Find the index of the vector with the minimum dot product with direction.
    #[must_use]
    #[inline]
    pub fn argmin_dot(&self, direction: &Vector3<f64>) -> (usize, f64) {
        let dots = self.dot(direction);
        let mut min_idx = 0;
        let mut min_val = dots[0];
        for i in 1..8 {
            if dots[i] < min_val {
                min_val = dots[i];
                min_idx = i;
            }
        }
        (min_idx, min_val)
    }

    /// Fused multiply-add: self * a + b.
    #[must_use]
    #[inline]
    pub fn mul_add(&self, a: f64, b: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..8 {
            result.xs[i] = self.xs[i].mul_add(a, b.xs[i]);
        }
        for i in 0..8 {
            result.ys[i] = self.ys[i].mul_add(a, b.ys[i]);
        }
        for i in 0..8 {
            result.zs[i] = self.zs[i].mul_add(a, b.zs[i]);
        }
        result
    }
}

impl std::ops::Add for Vec3x8 {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::add(&self, &rhs)
    }
}

impl std::ops::Sub for Vec3x8 {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::sub(&self, &rhs)
    }
}

impl std::ops::Neg for Vec3x8 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self::neg(&self)
    }
}

impl std::ops::Mul<f64> for Vec3x8 {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self {
        self.scale(rhs)
    }
}
