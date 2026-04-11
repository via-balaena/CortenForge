//! Single-site systematic-scan Gibbs sampler for pairwise Ising models.
//!
//! Operates on discrete spins `σ ∈ {−1, +1}^N` with the same Hamiltonian
//! as [`exact_distribution`](crate::ising::exact_distribution):
//!
//! ```text
//! H(σ) = −Σ J_{ij} σ_i σ_j − Σ h_i σ_i
//! ```
//!
//! Each call to [`GibbsSampler::sweep`] updates all N sites in index order
//! using the exact conditional `P(σ_i | σ_{-i}) = sigmoid(2·local_field_i / kT)`.
//!
//! Phase 6 of the thermodynamic computing initiative uses this sampler
//! as a CPU reference for three-way distribution comparison (Gibbs vs
//! exact vs Langevin).

use rand::Rng;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// Single-site systematic-scan Gibbs sampler for pairwise Ising models.
///
/// Construct via [`GibbsSampler::new`], then call [`sample`](Self::sample)
/// to collect a configuration histogram in the same `Vec<(u32, f64)>`
/// format as [`exact_distribution`](crate::ising::exact_distribution).
pub struct GibbsSampler {
    n: usize,
    edges: Vec<(usize, usize)>,
    coupling_j: Vec<f64>,
    field_h: Vec<f64>,
    k_b_t: f64,
    /// Current spin configuration: each element is +1.0 or −1.0.
    spins: Vec<f64>,
    rng: ChaCha8Rng,
    /// Precomputed adjacency: `neighbors[i]` contains `(j, J_{ij})` for
    /// every edge involving site `i`. Makes `local_field_i` O(degree)
    /// instead of O(|edges|).
    neighbors: Vec<Vec<(usize, f64)>>,
}

impl GibbsSampler {
    /// Create a new Gibbs sampler with all spins initialized to +1.
    ///
    /// # Panics
    /// - If `n > 20` (inherited safety limit from `exact_distribution`).
    /// - If `coupling_j.len() != edges.len()`.
    /// - If `field_h.len() != n`.
    /// - If `k_b_t <= 0`.
    #[must_use]
    pub fn new(
        n: usize,
        edges: &[(usize, usize)],
        coupling_j: &[f64],
        field_h: &[f64],
        k_b_t: f64,
        seed: u64,
    ) -> Self {
        assert!(n <= 20, "n={n} exceeds safety limit of 20");
        assert!(
            coupling_j.len() == edges.len(),
            "coupling_j length ({}) must match edges length ({})",
            coupling_j.len(),
            edges.len(),
        );
        assert!(
            field_h.len() == n,
            "field_h length ({}) must match n ({n})",
            field_h.len(),
        );
        assert!(k_b_t > 0.0, "k_b_t must be positive, got {k_b_t}");

        // Build adjacency list: for each site i, store (j, J_ij) for all
        // edges involving i. Edges are undirected, so (i,j) adds j to i's
        // list and i to j's list.
        let mut neighbors = vec![Vec::new(); n];
        for (k, &(i, j)) in edges.iter().enumerate() {
            neighbors[i].push((j, coupling_j[k]));
            neighbors[j].push((i, coupling_j[k]));
        }

        Self {
            n,
            edges: edges.to_vec(),
            coupling_j: coupling_j.to_vec(),
            field_h: field_h.to_vec(),
            k_b_t,
            spins: vec![1.0; n],
            rng: ChaCha8Rng::seed_from_u64(seed),
            neighbors,
        }
    }

    /// Edge list (read-only).
    #[must_use]
    pub fn edges(&self) -> &[(usize, usize)] {
        &self.edges
    }

    /// Per-edge coupling constants (read-only).
    #[must_use]
    pub fn coupling_j(&self) -> &[f64] {
        &self.coupling_j
    }

    /// Per-site external fields (read-only).
    #[must_use]
    pub fn field_h(&self) -> &[f64] {
        &self.field_h
    }

    /// Perform one full sweep: update each site once in index order.
    ///
    /// For each site `i`:
    /// 1. Compute `local_field_i = Σ_{j ∈ neighbors(i)} J_{ij} · σ_j + h_i`.
    /// 2. Compute `P(σ_i = +1) = 1 / (1 + exp(−2 · local_field_i / kT))`.
    /// 3. Draw `u ~ Uniform(0, 1)` and set `σ_i = +1` if `u < P`, else `−1`.
    pub fn sweep(&mut self) {
        for i in 0..self.n {
            // Local field from neighbors + external field
            let local_field: f64 = self.neighbors[i]
                .iter()
                .map(|&(j, j_ij)| j_ij * self.spins[j])
                .sum::<f64>()
                + self.field_h[i];

            // P(σ_i = +1 | σ_{-i}) = sigmoid(2 · local_field / kT)
            let p_plus = 1.0 / (1.0 + (-2.0 * local_field / self.k_b_t).exp());

            let u: f64 = self.rng.random();
            self.spins[i] = if u < p_plus { 1.0 } else { -1.0 };
        }
    }

    /// Current spin configuration as a bitmask.
    ///
    /// Matches [`exact_distribution`](crate::ising::exact_distribution)'s
    /// convention: bit `i` set means `σ_i = +1`.
    #[must_use]
    pub fn config_bitmask(&self) -> u32 {
        let mut mask: u32 = 0;
        for (i, &s) in self.spins.iter().enumerate() {
            if s > 0.0 {
                mask |= 1 << i;
            }
        }
        mask
    }

    /// Run `n_burn_in` sweeps (discarded) then `n_samples` sweeps,
    /// collecting a configuration histogram.
    ///
    /// Returns the empirical distribution in the same `Vec<(u32, f64)>`
    /// format as [`exact_distribution`](crate::ising::exact_distribution):
    /// all `2^N` configs present, sorted by config index, probabilities
    /// summing to 1.
    #[must_use]
    pub fn sample(&mut self, n_burn_in: usize, n_samples: usize) -> Vec<(u32, f64)> {
        // Burn-in
        for _ in 0..n_burn_in {
            self.sweep();
        }

        // Collect histogram
        let n_configs = 1u32 << self.n;
        let mut counts = vec![0_u64; n_configs as usize];
        for _ in 0..n_samples {
            self.sweep();
            counts[self.config_bitmask() as usize] += 1;
        }

        // Normalize to probabilities
        // u64/usize → f64: counts are ≤ n_samples which is at most ~10^18;
        // precision loss in the mantissa is negligible for histogram work.
        #[allow(clippy::cast_precision_loss)]
        let probs: Vec<(u32, f64)> = {
            let total = n_samples as f64;
            (0..n_configs)
                .map(|c| (c, counts[c as usize] as f64 / total))
                .collect()
        };
        probs
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn uniform_distribution_at_zero_coupling_and_field() {
        // J=0, h=0: all configurations are equiprobable.
        let n = 4;
        let mut sampler = GibbsSampler::new(n, &[], &[], &[0.0; 4], 1.0, 42);
        let dist = sampler.sample(1_000, 100_000);

        assert_eq!(dist.len(), 16);
        let expected_p = 1.0 / 16.0;
        for &(c, p) in &dist {
            let err = (p - expected_p).abs();
            assert!(
                err < 0.01,
                "config {c}: p={p:.4}, expected {expected_p:.4}, error {err:.4}"
            );
        }
    }

    #[test]
    fn reproducibility_same_seed() {
        let n = 4;
        let edges = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
        let j = [0.8, -0.3, 0.1, 0.5, -0.2, 0.6];
        let h = [0.3, -0.2, 0.0, 0.15];

        let mut s1 = GibbsSampler::new(n, &edges, &j, &h, 1.0, 999);
        let mut s2 = GibbsSampler::new(n, &edges, &j, &h, 1.0, 999);

        let d1 = s1.sample(100, 10_000);
        let d2 = s2.sample(100, 10_000);

        for (a, b) in d1.iter().zip(&d2) {
            assert_eq!(a.0, b.0, "config mismatch");
            assert_eq!(a.1, b.1, "probability not bit-exact for config {}", a.0);
        }
    }

    #[test]
    fn config_bitmask_matches_spin_state() {
        let n = 4;
        let mut sampler = GibbsSampler::new(n, &[], &[], &[0.0; 4], 1.0, 123);

        // All spins start at +1 → bitmask should be 0b1111 = 15
        assert_eq!(sampler.config_bitmask(), 0b1111);

        // Manually set spins: [+1, -1, +1, -1] → bits 0 and 2 set → 0b0101 = 5
        sampler.spins = vec![1.0, -1.0, 1.0, -1.0];
        assert_eq!(sampler.config_bitmask(), 0b0101);

        // All -1 → 0
        sampler.spins = vec![-1.0, -1.0, -1.0, -1.0];
        assert_eq!(sampler.config_bitmask(), 0);
    }
}
