//! SIMD-optimized 4-wide `Vector3` batch type.
//!
//! [`Vec3x4`] stores 4 `Vector3<f64>` values in a structure-of-arrays (`SoA`) layout
//! optimized for SIMD operations on 256-bit registers (`AVX`/`AVX2`).

use nalgebra::Vector3;

/// A batch of 4 `Vector3<f64>` values stored in `SoA` (Structure of Arrays) layout.
///
/// This layout enables efficient SIMD operations by grouping x, y, and z
/// components together, allowing 4 operations to execute simultaneously.
///
/// # Memory Layout
///
/// ```text
/// xs: [x0, x1, x2, x3]  <- 4 x components (256 bits / 32 bytes)
/// ys: [y0, y1, y2, y3]  <- 4 y components (256 bits / 32 bytes)
/// zs: [z0, z1, z2, z3]  <- 4 z components (256 bits / 32 bytes)
/// ```
///
/// # Example
///
/// ```
/// use sim_simd::Vec3x4;
/// use nalgebra::Vector3;
///
/// let batch = Vec3x4::from_vectors([
///     Vector3::new(1.0, 2.0, 3.0),
///     Vector3::new(4.0, 5.0, 6.0),
///     Vector3::new(7.0, 8.0, 9.0),
///     Vector3::new(10.0, 11.0, 12.0),
/// ]);
///
/// // Compute all 4 squared norms at once
/// let norms_sq = batch.norm_squared();
/// assert_eq!(norms_sq[0], 14.0);  // 1² + 2² + 3²
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C, align(32))]
pub struct Vec3x4 {
    /// X components of all 4 vectors.
    pub xs: [f64; 4],
    /// Y components of all 4 vectors.
    pub ys: [f64; 4],
    /// Z components of all 4 vectors.
    pub zs: [f64; 4],
}

impl Default for Vec3x4 {
    fn default() -> Self {
        Self::zeros()
    }
}

impl Vec3x4 {
    /// Create a batch of 4 zero vectors.
    #[must_use]
    #[inline]
    pub const fn zeros() -> Self {
        Self {
            xs: [0.0; 4],
            ys: [0.0; 4],
            zs: [0.0; 4],
        }
    }

    /// Create from 4 individual vectors.
    #[must_use]
    #[inline]
    pub fn from_vectors(vectors: [Vector3<f64>; 4]) -> Self {
        Self {
            xs: [vectors[0].x, vectors[1].x, vectors[2].x, vectors[3].x],
            ys: [vectors[0].y, vectors[1].y, vectors[2].y, vectors[3].y],
            zs: [vectors[0].z, vectors[1].z, vectors[2].z, vectors[3].z],
        }
    }

    /// Create from a slice of vectors.
    ///
    /// # Panics
    ///
    /// Panics if the slice has fewer than 4 elements.
    #[must_use]
    #[inline]
    pub fn from_slice(vectors: &[Vector3<f64>]) -> Self {
        debug_assert!(vectors.len() >= 4, "Need at least 4 vectors");
        Self {
            xs: [vectors[0].x, vectors[1].x, vectors[2].x, vectors[3].x],
            ys: [vectors[0].y, vectors[1].y, vectors[2].y, vectors[3].y],
            zs: [vectors[0].z, vectors[1].z, vectors[2].z, vectors[3].z],
        }
    }

    /// Create from a slice, padding with zeros if fewer than 4 vectors.
    #[must_use]
    #[inline]
    pub fn from_slice_padded(vectors: &[Vector3<f64>]) -> Self {
        let mut result = Self::zeros();
        for (i, v) in vectors.iter().take(4).enumerate() {
            result.xs[i] = v.x;
            result.ys[i] = v.y;
            result.zs[i] = v.z;
        }
        result
    }

    /// Create by broadcasting a single vector to all 4 lanes.
    #[must_use]
    #[inline]
    pub fn splat(v: Vector3<f64>) -> Self {
        Self {
            xs: [v.x; 4],
            ys: [v.y; 4],
            zs: [v.z; 4],
        }
    }

    /// Extract the vector at a given index.
    #[must_use]
    #[inline]
    pub fn get(&self, index: usize) -> Vector3<f64> {
        debug_assert!(index < 4);
        Vector3::new(self.xs[index], self.ys[index], self.zs[index])
    }

    /// Convert to an array of 4 vectors.
    #[must_use]
    #[inline]
    pub fn to_vectors(&self) -> [Vector3<f64>; 4] {
        [self.get(0), self.get(1), self.get(2), self.get(3)]
    }

    /// Compute dot product of each vector with a single direction vector.
    ///
    /// Returns 4 dot products simultaneously.
    #[must_use]
    #[inline]
    pub fn dot(&self, direction: &Vector3<f64>) -> [f64; 4] {
        // SIMD-friendly: each component multiply-accumulate is independent
        [
            self.xs[0] * direction.x + self.ys[0] * direction.y + self.zs[0] * direction.z,
            self.xs[1] * direction.x + self.ys[1] * direction.y + self.zs[1] * direction.z,
            self.xs[2] * direction.x + self.ys[2] * direction.y + self.zs[2] * direction.z,
            self.xs[3] * direction.x + self.ys[3] * direction.y + self.zs[3] * direction.z,
        ]
    }

    /// Compute dot product of each vector with a single direction (SIMD-optimized).
    ///
    /// This version is explicitly written for auto-vectorization.
    #[must_use]
    #[inline]
    pub fn dot_simd(&self, direction: &Vector3<f64>) -> [f64; 4] {
        let mut result = [0.0; 4];

        // X contribution
        for i in 0..4 {
            result[i] = self.xs[i] * direction.x;
        }

        // Y contribution (fused multiply-add pattern)
        for i in 0..4 {
            result[i] += self.ys[i] * direction.y;
        }

        // Z contribution (fused multiply-add pattern)
        for i in 0..4 {
            result[i] += self.zs[i] * direction.z;
        }

        result
    }

    /// Compute pairwise dot products between this batch and another.
    ///
    /// Returns `[self[0]·other[0], self[1]·other[1], self[2]·other[2], self[3]·other[3]]`
    #[must_use]
    #[inline]
    pub fn dot_pairwise(&self, other: &Self) -> [f64; 4] {
        let mut result = [0.0; 4];

        for i in 0..4 {
            result[i] = self.xs[i] * other.xs[i];
        }
        for i in 0..4 {
            result[i] += self.ys[i] * other.ys[i];
        }
        for i in 0..4 {
            result[i] += self.zs[i] * other.zs[i];
        }

        result
    }

    /// Compute squared norm of each vector.
    #[must_use]
    #[inline]
    pub fn norm_squared(&self) -> [f64; 4] {
        self.dot_pairwise(self)
    }

    /// Compute norm of each vector.
    #[must_use]
    #[inline]
    pub fn norm(&self) -> [f64; 4] {
        let sq = self.norm_squared();
        [sq[0].sqrt(), sq[1].sqrt(), sq[2].sqrt(), sq[3].sqrt()]
    }

    /// Add two batches element-wise.
    #[must_use]
    #[inline]
    pub fn add(&self, other: &Self) -> Self {
        Self {
            xs: [
                self.xs[0] + other.xs[0],
                self.xs[1] + other.xs[1],
                self.xs[2] + other.xs[2],
                self.xs[3] + other.xs[3],
            ],
            ys: [
                self.ys[0] + other.ys[0],
                self.ys[1] + other.ys[1],
                self.ys[2] + other.ys[2],
                self.ys[3] + other.ys[3],
            ],
            zs: [
                self.zs[0] + other.zs[0],
                self.zs[1] + other.zs[1],
                self.zs[2] + other.zs[2],
                self.zs[3] + other.zs[3],
            ],
        }
    }

    /// Subtract two batches element-wise.
    #[must_use]
    #[inline]
    pub fn sub(&self, other: &Self) -> Self {
        Self {
            xs: [
                self.xs[0] - other.xs[0],
                self.xs[1] - other.xs[1],
                self.xs[2] - other.xs[2],
                self.xs[3] - other.xs[3],
            ],
            ys: [
                self.ys[0] - other.ys[0],
                self.ys[1] - other.ys[1],
                self.ys[2] - other.ys[2],
                self.ys[3] - other.ys[3],
            ],
            zs: [
                self.zs[0] - other.zs[0],
                self.zs[1] - other.zs[1],
                self.zs[2] - other.zs[2],
                self.zs[3] - other.zs[3],
            ],
        }
    }

    /// Multiply each vector by a scalar.
    #[must_use]
    #[inline]
    pub fn scale(&self, scalar: f64) -> Self {
        Self {
            xs: [
                self.xs[0] * scalar,
                self.xs[1] * scalar,
                self.xs[2] * scalar,
                self.xs[3] * scalar,
            ],
            ys: [
                self.ys[0] * scalar,
                self.ys[1] * scalar,
                self.ys[2] * scalar,
                self.ys[3] * scalar,
            ],
            zs: [
                self.zs[0] * scalar,
                self.zs[1] * scalar,
                self.zs[2] * scalar,
                self.zs[3] * scalar,
            ],
        }
    }

    /// Multiply each vector by its corresponding scalar.
    #[must_use]
    #[inline]
    pub fn scale_each(&self, scalars: [f64; 4]) -> Self {
        Self {
            xs: [
                self.xs[0] * scalars[0],
                self.xs[1] * scalars[1],
                self.xs[2] * scalars[2],
                self.xs[3] * scalars[3],
            ],
            ys: [
                self.ys[0] * scalars[0],
                self.ys[1] * scalars[1],
                self.ys[2] * scalars[2],
                self.ys[3] * scalars[3],
            ],
            zs: [
                self.zs[0] * scalars[0],
                self.zs[1] * scalars[1],
                self.zs[2] * scalars[2],
                self.zs[3] * scalars[3],
            ],
        }
    }

    /// Negate all vectors.
    #[must_use]
    #[inline]
    pub fn neg(&self) -> Self {
        Self {
            xs: [-self.xs[0], -self.xs[1], -self.xs[2], -self.xs[3]],
            ys: [-self.ys[0], -self.ys[1], -self.ys[2], -self.ys[3]],
            zs: [-self.zs[0], -self.zs[1], -self.zs[2], -self.zs[3]],
        }
    }

    /// Compute cross product of each vector with a single direction.
    ///
    /// Returns `[self[0]×dir, self[1]×dir, self[2]×dir, self[3]×dir]`
    #[must_use]
    #[inline]
    pub fn cross(&self, direction: &Vector3<f64>) -> Self {
        // cross(a, b) = (a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)
        Self {
            xs: [
                self.ys[0] * direction.z - self.zs[0] * direction.y,
                self.ys[1] * direction.z - self.zs[1] * direction.y,
                self.ys[2] * direction.z - self.zs[2] * direction.y,
                self.ys[3] * direction.z - self.zs[3] * direction.y,
            ],
            ys: [
                self.zs[0] * direction.x - self.xs[0] * direction.z,
                self.zs[1] * direction.x - self.xs[1] * direction.z,
                self.zs[2] * direction.x - self.xs[2] * direction.z,
                self.zs[3] * direction.x - self.xs[3] * direction.z,
            ],
            zs: [
                self.xs[0] * direction.y - self.ys[0] * direction.x,
                self.xs[1] * direction.y - self.ys[1] * direction.x,
                self.xs[2] * direction.y - self.ys[2] * direction.x,
                self.xs[3] * direction.y - self.ys[3] * direction.x,
            ],
        }
    }

    /// Compute pairwise cross products.
    #[must_use]
    #[inline]
    pub fn cross_pairwise(&self, other: &Self) -> Self {
        Self {
            xs: [
                self.ys[0] * other.zs[0] - self.zs[0] * other.ys[0],
                self.ys[1] * other.zs[1] - self.zs[1] * other.ys[1],
                self.ys[2] * other.zs[2] - self.zs[2] * other.ys[2],
                self.ys[3] * other.zs[3] - self.zs[3] * other.ys[3],
            ],
            ys: [
                self.zs[0] * other.xs[0] - self.xs[0] * other.zs[0],
                self.zs[1] * other.xs[1] - self.xs[1] * other.zs[1],
                self.zs[2] * other.xs[2] - self.xs[2] * other.zs[2],
                self.zs[3] * other.xs[3] - self.xs[3] * other.zs[3],
            ],
            zs: [
                self.xs[0] * other.ys[0] - self.ys[0] * other.xs[0],
                self.xs[1] * other.ys[1] - self.ys[1] * other.xs[1],
                self.xs[2] * other.ys[2] - self.ys[2] * other.xs[2],
                self.xs[3] * other.ys[3] - self.ys[3] * other.xs[3],
            ],
        }
    }

    /// Normalize each vector.
    ///
    /// Vectors with near-zero norm are left unchanged to avoid `NaN`.
    #[must_use]
    #[inline]
    pub fn normalize(&self) -> Self {
        let norms = self.norm();
        let inv_norms = [
            if norms[0] > 1e-10 {
                1.0 / norms[0]
            } else {
                1.0
            },
            if norms[1] > 1e-10 {
                1.0 / norms[1]
            } else {
                1.0
            },
            if norms[2] > 1e-10 {
                1.0 / norms[2]
            } else {
                1.0
            },
            if norms[3] > 1e-10 {
                1.0 / norms[3]
            } else {
                1.0
            },
        ];
        self.scale_each(inv_norms)
    }

    /// Clamp each component of each vector to the given range.
    #[must_use]
    #[inline]
    pub fn clamp(&self, min: f64, max: f64) -> Self {
        Self {
            xs: [
                self.xs[0].clamp(min, max),
                self.xs[1].clamp(min, max),
                self.xs[2].clamp(min, max),
                self.xs[3].clamp(min, max),
            ],
            ys: [
                self.ys[0].clamp(min, max),
                self.ys[1].clamp(min, max),
                self.ys[2].clamp(min, max),
                self.ys[3].clamp(min, max),
            ],
            zs: [
                self.zs[0].clamp(min, max),
                self.zs[1].clamp(min, max),
                self.zs[2].clamp(min, max),
                self.zs[3].clamp(min, max),
            ],
        }
    }

    /// Component-wise clamp to a box defined by half-extents.
    ///
    /// Clamps each vector's components to [-half.x, half.x], [-half.y, half.y], [-half.z, half.z].
    #[must_use]
    #[inline]
    pub fn clamp_to_box(&self, half_extents: &Vector3<f64>) -> Self {
        Self {
            xs: [
                self.xs[0].clamp(-half_extents.x, half_extents.x),
                self.xs[1].clamp(-half_extents.x, half_extents.x),
                self.xs[2].clamp(-half_extents.x, half_extents.x),
                self.xs[3].clamp(-half_extents.x, half_extents.x),
            ],
            ys: [
                self.ys[0].clamp(-half_extents.y, half_extents.y),
                self.ys[1].clamp(-half_extents.y, half_extents.y),
                self.ys[2].clamp(-half_extents.y, half_extents.y),
                self.ys[3].clamp(-half_extents.y, half_extents.y),
            ],
            zs: [
                self.zs[0].clamp(-half_extents.z, half_extents.z),
                self.zs[1].clamp(-half_extents.z, half_extents.z),
                self.zs[2].clamp(-half_extents.z, half_extents.z),
                self.zs[3].clamp(-half_extents.z, half_extents.z),
            ],
        }
    }

    /// Find the index of the vector with the maximum dot product with direction.
    ///
    /// Returns `(index, dot_value)`.
    #[must_use]
    #[inline]
    pub fn argmax_dot(&self, direction: &Vector3<f64>) -> (usize, f64) {
        let dots = self.dot(direction);
        let mut max_idx = 0;
        let mut max_val = dots[0];
        for i in 1..4 {
            if dots[i] > max_val {
                max_val = dots[i];
                max_idx = i;
            }
        }
        (max_idx, max_val)
    }

    /// Find the index of the vector with the minimum dot product with direction.
    ///
    /// Returns `(index, dot_value)`.
    #[must_use]
    #[inline]
    pub fn argmin_dot(&self, direction: &Vector3<f64>) -> (usize, f64) {
        let dots = self.dot(direction);
        let mut min_idx = 0;
        let mut min_val = dots[0];
        for i in 1..4 {
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
        Self {
            xs: [
                self.xs[0].mul_add(a, b.xs[0]),
                self.xs[1].mul_add(a, b.xs[1]),
                self.xs[2].mul_add(a, b.xs[2]),
                self.xs[3].mul_add(a, b.xs[3]),
            ],
            ys: [
                self.ys[0].mul_add(a, b.ys[0]),
                self.ys[1].mul_add(a, b.ys[1]),
                self.ys[2].mul_add(a, b.ys[2]),
                self.ys[3].mul_add(a, b.ys[3]),
            ],
            zs: [
                self.zs[0].mul_add(a, b.zs[0]),
                self.zs[1].mul_add(a, b.zs[1]),
                self.zs[2].mul_add(a, b.zs[2]),
                self.zs[3].mul_add(a, b.zs[3]),
            ],
        }
    }
}

impl std::ops::Add for Vec3x4 {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::add(&self, &rhs)
    }
}

impl std::ops::Sub for Vec3x4 {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::sub(&self, &rhs)
    }
}

impl std::ops::Neg for Vec3x4 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self::neg(&self)
    }
}

impl std::ops::Mul<f64> for Vec3x4 {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self {
        self.scale(rhs)
    }
}
