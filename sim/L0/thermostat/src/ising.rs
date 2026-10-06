//! Exact Ising model computations for thermodynamic computing validation.
//!
//! Provides exact enumeration over `2^N` spin configurations for arbitrary
//! coupling topology, per-edge coupling constants, and per-site external
//! fields. Used by [`IsingLearner`](crate::IsingLearner) for KL divergence
//! monitoring and by Phase 6+ for Gibbs sampler comparison.

/// Summary statistics from an Ising distribution.
#[derive(Clone, Debug)]
pub struct IsingStats {
    /// Per-site magnetizations `⟨σ_i⟩`.
    pub magnetizations: Vec<f64>,
    /// Per-edge correlations `⟨σ_i σ_j⟩` (same order as edge list).
    pub correlations: Vec<f64>,
}

/// Spin value for site `i` in configuration `c`: `+1` if bit `i` is set,
/// `−1` otherwise.
const fn spin(c: u32, i: usize) -> f64 {
    if c & (1 << i) != 0 { 1.0 } else { -1.0 }
}

/// Exact Ising distribution by enumeration over `2^N` configurations.
///
/// The Ising Hamiltonian:
/// ```text
/// H(σ) = −Σ_{k} J_k · σ_{i_k} · σ_{j_k} − Σ_i h_i · σ_i
/// ```
///
/// The Boltzmann distribution: `P(σ) = exp(−H(σ) / kT) / Z`, computed with
/// every energy taken relative to the minimum, so low temperatures do not
/// overflow.
///
/// Returns a vector of `(config_bitmask, probability)` pairs sorted by
/// config index. Bit `i` of the bitmask represents spin `i`: set = `+1`,
/// clear = `−1`.
///
/// # Panics
/// - If `n > 20` (safety limit: `2^20` = 1M configurations).
/// - If `coupling_j.len() != edges.len()`.
/// - If `field_h.len() != n`.
/// - If `k_b_t <= 0`.
#[must_use]
pub fn exact_distribution(
    n: usize,
    edges: &[(usize, usize)],
    coupling_j: &[f64],
    field_h: &[f64],
    k_b_t: f64,
) -> Vec<(u32, f64)> {
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

    let n_configs = 1u32 << n;
    let mut dist = Vec::with_capacity(n_configs as usize);

    for c in 0..n_configs {
        // Coupling energy: −Σ J_k σ_i σ_j
        let coupling_energy: f64 = edges
            .iter()
            .zip(coupling_j)
            .map(|(&(i, j), &j_k)| -j_k * spin(c, i) * spin(c, j))
            .sum();

        // Field energy: −Σ h_i σ_i
        let field_energy: f64 = field_h
            .iter()
            .enumerate()
            .map(|(i, &h)| -h * spin(c, i))
            .sum();

        dist.push((c, coupling_energy + field_energy));
    }

    // Boltzmann factors relative to the minimum energy.
    let e_min = dist.iter().map(|&(_, e)| e).fold(f64::INFINITY, f64::min);
    let mut z = 0.0_f64;
    for (_, w) in &mut dist {
        *w = (-(*w - e_min) / k_b_t).exp();
        z += *w;
    }

    // Normalize to probabilities
    for (_, p) in &mut dist {
        *p /= z;
    }

    dist
}

/// Extract magnetizations `⟨σ_i⟩` and pairwise correlations `⟨σ_i σ_j⟩`
/// from an exact distribution.
#[must_use]
pub fn ising_statistics(dist: &[(u32, f64)], n: usize, edges: &[(usize, usize)]) -> IsingStats {
    let mut magnetizations = vec![0.0; n];
    let mut correlations = vec![0.0; edges.len()];

    for &(c, p) in dist {
        for (i, mag) in magnetizations.iter_mut().enumerate() {
            *mag += p * spin(c, i);
        }
        for (k, &(i, j)) in edges.iter().enumerate() {
            correlations[k] += p * spin(c, i) * spin(c, j);
        }
    }

    IsingStats {
        magnetizations,
        correlations,
    }
}

/// Total variation distance between two distributions over the same
/// configuration space.
///
/// `TV(P, Q) = ½ Σ_c |P(c) − Q(c)|`
///
/// Returns a value in `[0, 1]`. Unlike KL divergence, TV distance is
/// symmetric and bounded, making it more suitable for validation gates.
///
/// Both distributions must have the same length and be sorted by config
/// index (as returned by [`exact_distribution`]).
///
/// # Panics
/// Panics if `p.len() != q.len()`.
#[must_use]
pub fn tv_distance(p: &[(u32, f64)], q: &[(u32, f64)]) -> f64 {
    assert!(
        p.len() == q.len(),
        "distributions must have the same length: {} vs {}",
        p.len(),
        q.len(),
    );

    0.5 * p
        .iter()
        .zip(q)
        .map(|(&(_, p_val), &(_, q_val))| (p_val - q_val).abs())
        .sum::<f64>()
}

/// KL divergence `KL(P ‖ Q)` between two distributions over the same
/// configuration space.
///
/// Both distributions must have the same length and be sorted by config
/// index (as returned by [`exact_distribution`]).
///
/// Returns `f64::INFINITY` if any configuration has `P > 0` and `Q = 0`.
///
/// # Panics
/// Panics if `p.len() != q.len()`.
#[must_use]
pub fn kl_divergence(p: &[(u32, f64)], q: &[(u32, f64)]) -> f64 {
    assert!(
        p.len() == q.len(),
        "distributions must have the same length: {} vs {}",
        p.len(),
        q.len(),
    );

    p.iter()
        .zip(q)
        .map(|(&(_, p_val), &(_, q_val))| {
            if p_val < 1e-300 {
                0.0 // 0 · log(0/q) = 0 by convention
            } else if q_val < 1e-300 {
                f64::INFINITY
            } else {
                p_val * (p_val / q_val).ln()
            }
        })
        .sum()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn uniform_distribution_at_zero_coupling_and_field() {
        let n = 4;
        let edges: Vec<(usize, usize)> = vec![];
        let coupling_j: Vec<f64> = vec![];
        let field_h = vec![0.0; n];

        let dist = exact_distribution(n, &edges, &coupling_j, &field_h, 1.0);
        assert_eq!(dist.len(), 16);

        let expected_p = 1.0 / 16.0;
        for &(_, p) in &dist {
            assert!(
                (p - expected_p).abs() < 1e-14,
                "expected {expected_p}, got {p}"
            );
        }
    }

    #[test]
    fn magnetization_with_field_only() {
        // No coupling, field on site 0 only: h_0 = 1.0, others 0.
        // ⟨σ_0⟩ = tanh(h/kT) = tanh(1.0) for independent spins with field.
        let n = 2;
        let dist = exact_distribution(n, &[], &[], &[1.0, 0.0], 1.0);
        let stats = ising_statistics(&dist, n, &[]);

        let expected_mag = 1.0_f64.tanh();
        assert!(
            (stats.magnetizations[0] - expected_mag).abs() < 1e-12,
            "⟨σ_0⟩ = {}, expected tanh(1) = {expected_mag}",
            stats.magnetizations[0],
        );
        assert!(
            stats.magnetizations[1].abs() < 1e-12,
            "⟨σ_1⟩ should be 0, got {}",
            stats.magnetizations[1],
        );
    }

    #[test]
    fn nn_chain_matches_phase4() {
        // 4-element chain with uniform J=0.5, no fields, kT=1.0.
        // ⟨σ_i σ_{i+1}⟩ = tanh(βJ) = tanh(0.5).
        let n = 4;
        let edges: Vec<(usize, usize)> = vec![(0, 1), (1, 2), (2, 3)];
        let coupling_j = vec![0.5; 3];
        let field_h = vec![0.0; n];

        let dist = exact_distribution(n, &edges, &coupling_j, &field_h, 1.0);
        let stats = ising_statistics(&dist, n, &edges);

        let expected_nn = 0.5_f64.tanh();
        for (k, &corr) in stats.correlations.iter().enumerate() {
            assert!(
                (corr - expected_nn).abs() < 1e-12,
                "edge {k}: ⟨σσ⟩ = {corr}, expected tanh(0.5) = {expected_nn}",
            );
        }

        // All magnetizations should be zero (no field, symmetric)
        for (i, &mag) in stats.magnetizations.iter().enumerate() {
            assert!(mag.abs() < 1e-12, "⟨σ_{i}⟩ should be 0, got {mag}");
        }
    }

    #[test]
    fn kl_divergence_is_zero_for_identical_distributions() {
        let n = 3;
        let dist = exact_distribution(n, &[(0, 1)], &[0.5], &[0.0; 3], 1.0);
        let kl = kl_divergence(&dist, &dist);
        assert!(kl.abs() < 1e-14, "KL(P||P) should be 0, got {kl}");
    }

    #[test]
    fn kl_divergence_is_positive_for_different_distributions() {
        let n = 3;
        let p = exact_distribution(n, &[(0, 1)], &[0.5], &[0.0; 3], 1.0);
        let q = exact_distribution(n, &[(0, 1)], &[1.0], &[0.0; 3], 1.0);
        let kl = kl_divergence(&p, &q);
        assert!(kl > 0.0, "KL(P||Q) should be positive, got {kl}");
    }

    #[test]
    fn fully_connected_partition_function() {
        // Verify Z sums to 1 after normalization (probabilities sum to 1).
        let n = 4;
        let edges: Vec<(usize, usize)> = vec![(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
        let coupling_j = vec![0.3, -0.2, 0.1, 0.5, -0.1, 0.4];
        let field_h = vec![0.2, -0.1, 0.0, 0.15];

        let dist = exact_distribution(n, &edges, &coupling_j, &field_h, 1.0);
        let total: f64 = dist.iter().map(|&(_, p)| p).sum();
        assert!(
            (total - 1.0).abs() < 1e-14,
            "probabilities should sum to 1, got {total}"
        );
        assert_eq!(dist.len(), 16);
    }

    #[test]
    fn kl_divergence_asymmetric() {
        // KL(P||Q) != KL(Q||P) in general.
        let n = 2;
        let p = exact_distribution(n, &[(0, 1)], &[0.5], &[0.3, 0.0], 1.0);
        let q = exact_distribution(n, &[(0, 1)], &[1.0], &[0.0, 0.0], 1.0);
        let kl_pq = kl_divergence(&p, &q);
        let kl_qp = kl_divergence(&q, &p);
        assert!(
            (kl_pq - kl_qp).abs() > 1e-6,
            "KL should be asymmetric: KL(P||Q)={kl_pq}, KL(Q||P)={kl_qp}"
        );
    }

    // ─── tv_distance tests ───────────────────────────────────────────

    #[test]
    fn tv_distance_identity_is_zero() {
        let dist = exact_distribution(3, &[(0, 1)], &[0.5], &[0.0; 3], 1.0);
        let tv = tv_distance(&dist, &dist);
        assert!(tv.abs() < 1e-14, "TV(P, P) should be 0, got {tv}");
    }

    #[test]
    fn tv_distance_is_symmetric() {
        let p = exact_distribution(3, &[(0, 1)], &[0.5], &[0.0; 3], 1.0);
        let q = exact_distribution(3, &[(0, 1)], &[1.0], &[0.0; 3], 1.0);
        let tv_pq = tv_distance(&p, &q);
        let tv_qp = tv_distance(&q, &p);
        assert!(
            (tv_pq - tv_qp).abs() < 1e-14,
            "TV should be symmetric: TV(P,Q)={tv_pq}, TV(Q,P)={tv_qp}"
        );
    }

    #[test]
    fn tv_distance_is_positive_for_different_distributions() {
        let p = exact_distribution(3, &[(0, 1)], &[0.5], &[0.0; 3], 1.0);
        let q = exact_distribution(3, &[(0, 1)], &[1.0], &[0.0; 3], 1.0);
        let tv = tv_distance(&p, &q);
        assert!(tv > 0.0, "TV(P,Q) should be positive, got {tv}");
        assert!(tv <= 1.0, "TV(P,Q) should be at most 1, got {tv}");
    }

    #[test]
    fn tv_distance_uniform_vs_delta() {
        // Uniform over 2^N=8 configs vs delta at config 0.
        // TV = ½ Σ |1/8 - δ(c,0)| = ½ (|1/8 - 1| + 7·|1/8 - 0|)
        //    = ½ (7/8 + 7/8) = 7/8 = 1 - 1/2^N
        let n = 3;
        let uniform = exact_distribution(n, &[], &[], &[0.0; 3], 1.0);
        let mut delta: Vec<(u32, f64)> = (0..8).map(|c| (c, 0.0)).collect();
        delta[0].1 = 1.0;

        let tv = tv_distance(&uniform, &delta);
        let expected = 1.0 - 1.0 / 8.0;
        assert!(
            (tv - expected).abs() < 1e-14,
            "TV(uniform, delta) should be {expected}, got {tv}"
        );
    }

    #[test]
    fn tv_distance_bounded_zero_to_one() {
        // TV is always in [0, 1]. Test with various distributions.
        let n = 4;
        let edges: Vec<(usize, usize)> = vec![(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
        let p = exact_distribution(n, &edges, &[0.3; 6], &[0.0; 4], 1.0);
        let q = exact_distribution(n, &edges, &[-0.5; 6], &[0.5, -0.5, 0.0, 0.1], 2.0);
        let tv = tv_distance(&p, &q);
        assert!(tv >= 0.0, "TV should be >= 0, got {tv}");
        assert!(tv <= 1.0, "TV should be <= 1, got {tv}");
    }

    #[test]
    fn low_temperature_keeps_the_ground_state() {
        // One spin in a field h = ±1 at kT = 1/720: the ground state's
        // Boltzmann factor exp(720) exceeds f64::MAX. Both signs, so the
        // ground state is config 1 once and config 0 once.
        for h in [1.0, -1.0] {
            let dist = exact_distribution(1, &[], &[], &[h], 1.0 / 720.0);
            let ground = usize::from(h > 0.0);
            let p_ground = dist[ground].1;
            let p_other = dist[1 - ground].1;
            assert!(
                (p_ground - 1.0).abs() < 1e-12,
                "h = {h}: p(ground) = {p_ground}"
            );
            assert!(
                (0.0..1e-300).contains(&p_other),
                "h = {h}: p(other) = {p_other}"
            );
        }
    }

    #[test]
    fn low_temperature_resolves_a_near_tie() {
        // Two spins, h = (1, 0.01), at kT = 1/720: the two lowest states both
        // have Boltzmann factors above f64::MAX, and they differ by
        // ΔE/kT = 0.02 · 720 = 14.4.
        let dist = exact_distribution(2, &[], &[], &[1.0, 0.01], 1.0 / 720.0);
        let r = (-14.4_f64).exp();
        let expected = [0.0, r / (1.0 + r), 0.0, 1.0 / (1.0 + r)];
        for (config, p) in dist {
            let want = expected[config as usize];
            assert!(
                (p - want).abs() < 1e-12,
                "config {config:#04b}: p = {p}, expected {want}"
            );
        }
    }

    #[test]
    fn tied_ground_states_split_evenly_near_overflow() {
        // A frustrated antiferromagnetic triangle (J = -1, no field) at
        // kT = 1/709.5: the six configurations with one spin against the
        // other two tie at E = -1, and each has a finite factor exp(709.5),
        // but their sum exceeds f64::MAX. The two aligned ones have E = 3.
        let edges = [(0, 1), (1, 2), (0, 2)];
        let dist = exact_distribution(3, &edges, &[-1.0; 3], &[0.0; 3], 1.0 / 709.5);
        for (config, p) in dist {
            let expected = if config == 0 || config == 7 {
                0.0
            } else {
                1.0 / 6.0
            };
            assert!(
                (p - expected).abs() < 1e-12,
                "config {config:#05b}: p = {p}, expected {expected}"
            );
        }
    }
}
