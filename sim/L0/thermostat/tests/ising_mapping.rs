//! What Ising model a coupled bistable array realises, and where its wells vanish.
//!
//! Each site is a [`DoubleWellPotential`] with barrier `ΔV` and minima at `±x₀`; Ising
//! couplings `J` and fields `h` (energy units) enter as [`PairwiseCoupling`] with `J/x₀²` and
//! [`ExternalField`] with `h/x₀`, and spin `i` is the sign of `x_i`. The continuous Boltzmann
//! distribution coarse-grained to spins, `P_cont(s)`, is integrated by tensor Gauss–Legendre
//! quadrature, so nothing here is sampled. Two readouts:
//!
//! - **drop**: a configuration counts only while every `|x_i| ≥ x₀/2`, the `WellState`
//!   threshold the Langevin gates and `IsingLearner` use;
//! - **sign**: every configuration counts.
//!
//! The effective Ising couplings are the order-2 Walsh–Hadamard coefficients of
//! `ln P_cont(s)`. To second order in the couplings they are
//! `J_eff/J = μ² + β·μ²·σ²·Σ_l J_il·J_lj / J_ij`, where `μ` and `σ²` are the mean and
//! variance of `x/x₀` in one well under the readout (the second term is the coupling that
//! sites `i` and `j` induce through each shared neighbour `l`).
//!
//! The crate's other mapping checks run Langevin at n = 4 (`gibbs_sampler.rs` gates B and C,
//! `coupled_bistable_array.rs`), with sampling noise.
#![allow(
    clippy::unwrap_used,
    clippy::many_single_char_names,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::similar_names,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::float_cmp
)]

use sim_core::test_fixtures::bistable_chain;
use sim_thermostat::{
    DoubleWellPotential, ExternalField, IsingProblem, PairwiseCoupling, PassiveStack,
};

/// The tilt at which one quartic well loses its second minimum, in units of `ΔV/x₀`:
/// `8/(3√3) ≈ 1.5396`.
fn well_removal_tilt() -> f64 {
    8.0 / (3.0 * 3.0_f64.sqrt())
}

/// An Ising problem in energy units.
struct Problem {
    n: usize,
    edges: Vec<(usize, usize)>,
    coupling_j: Vec<f64>,
    field_h: Vec<f64>,
}

impl Problem {
    /// The complete graph on `n` spins, every coupling `j`, no field.
    fn complete(n: usize, j: f64) -> Self {
        let edges: Vec<_> = (0..n)
            .flat_map(|a| (a + 1..n).map(move |b| (a, b)))
            .collect();
        Self {
            n,
            coupling_j: vec![j; edges.len()],
            edges,
            field_h: vec![0.0; n],
        }
    }

    /// The uniform coupling that puts every site of a `degree`-regular graph at tilt ratio
    /// `r` (clamped-neighbour tilt over the well-removal tilt), with no field.
    fn coupling_at(r: f64, degree: usize, delta_v: f64) -> f64 {
        r * well_removal_tilt() * delta_v / degree as f64
    }

    /// `Σ_l J_il·J_lj` over the neighbours `l` that spins `i` and `j` share.
    fn shared_neighbour_sum(&self, i: usize, j: usize) -> f64 {
        let coupling = |a: usize, b: usize| {
            self.edges
                .iter()
                .position(|&(p, q)| (p, q) == (a, b) || (p, q) == (b, a))
                .map(|k| self.coupling_j[k])
        };
        (0..self.n)
            .filter(|&l| l != i && l != j)
            .filter_map(|l| Some(coupling(i, l)? * coupling(l, j)?))
            .sum()
    }
}

/// The physical parameters: barrier, well position, temperature.
#[derive(Clone, Copy)]
struct Physics {
    delta_v: f64,
    x_0: f64,
    k_b_t: f64,
}

impl Physics {
    /// One well's potential, `ΔV((x/x₀)² − 1)²`.
    fn well(self, x: f64) -> f64 {
        let u = (x / self.x_0).powi(2) - 1.0;
        self.delta_v * u * u
    }

    /// The array's energy at positions `x`, as the components define it.
    fn energy(self, p: &Problem, x: &[f64]) -> f64 {
        let x_0 = self.x_0;
        let wells: f64 = x.iter().map(|&xi| self.well(xi)).sum();
        let couplings: f64 = p
            .edges
            .iter()
            .zip(&p.coupling_j)
            .map(|(&(i, j), &jk)| -jk / (x_0 * x_0) * x[i] * x[j])
            .sum();
        let fields: f64 = x
            .iter()
            .zip(&p.field_h)
            .map(|(&xi, &h)| -h / x_0 * xi)
            .sum();
        wells + couplings + fields
    }

    /// `∂energy/∂x_i`.
    fn gradient(self, p: &Problem, x: &[f64]) -> Vec<f64> {
        let x_0 = self.x_0;
        let mut g: Vec<f64> = x
            .iter()
            .zip(&p.field_h)
            .map(|(&xi, &h)| {
                4.0 * self.delta_v * xi / x_0.powi(2) * ((xi / x_0).powi(2) - 1.0) - h / x_0
            })
            .collect();
        for (&(i, j), &jk) in p.edges.iter().zip(&p.coupling_j) {
            g[i] -= jk / (x_0 * x_0) * x[j];
            g[j] -= jk / (x_0 * x_0) * x[i];
        }
        g
    }
}

// ─── The test's energy is the components' ────────────────────────────────

/// `Physics::energy` is the energy of the component types (built here with `J/x₀²` and
/// `h/x₀`), and the force of the stack `IsingProblem::add_components` installs is its
/// negative gradient: at `x₀ = 1.5`, so a `J/x₀` for `J/x₀²` mix-up shows. (The quadrature
/// below assembles the same terms site by site in `visit`.)
#[test]
fn the_test_energy_is_the_components_and_the_installed_force_its_gradient() {
    let physics = Physics {
        delta_v: 3.0,
        x_0: 1.5,
        k_b_t: 1.0,
    };
    let problem = Problem {
        n: 4,
        edges: vec![(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)],
        coupling_j: vec![0.8, -0.3, 0.1, 0.5, -0.2, 0.6],
        field_h: vec![0.3, -0.2, 0.0, 0.15],
    };
    let x_0 = physics.x_0;
    let well = |i| DoubleWellPotential::new(physics.delta_v, x_0, i);
    let coupling = || {
        PairwiseCoupling::new(
            problem.coupling_j.iter().map(|j| j / (x_0 * x_0)).collect(),
            problem.edges.clone(),
        )
    };
    let field = || ExternalField::new(problem.field_h.iter().map(|h| h / x_0).collect());
    // The installed stack is the shipped helper's, so the forces below pin it: its wells'
    // barrier and DOFs, and its J/x₀² and h/x₀, at positions off the wells' bottoms.
    let mut model = bistable_chain(problem.n);
    IsingProblem::new(
        problem.n,
        problem.edges.clone(),
        problem.coupling_j.clone(),
        problem.field_h.clone(),
    )
    .add_components(PassiveStack::builder(), physics.delta_v, x_0)
    .build()
    .try_install(&mut model)
    .unwrap();
    let mut data = model.make_data();

    for x in [
        [1.4, -1.6, 0.2, -2.9],
        [-0.7, 0.0, 3.1, 1.5],
        [0.05, -0.4, -1.1, 2.2],
    ] {
        data.qpos.as_mut_slice().copy_from_slice(&x);
        data.forward(&model).unwrap();
        let components = (0..problem.n).map(|i| well(i).potential(x[i])).sum::<f64>()
            + coupling().coupling_energy(&model, &data).unwrap()
            + field().field_energy(&model, &data).unwrap();
        let energy = physics.energy(&problem, &x);
        assert!(
            (components - energy).abs() < 1e-12 * energy.abs().max(1.0),
            "at {x:?}: components {components}, test {energy}"
        );
        for (i, g) in physics.gradient(&problem, &x).into_iter().enumerate() {
            let force = data.qfrc_passive[i];
            assert!(
                (force + g).abs() < 1e-12 * g.abs().max(1.0),
                "at {x:?}, DOF {i}: force {force}, -gradient {}",
                -g
            );
        }
    }
}

// ─── Quadrature ──────────────────────────────────────────────────────────

/// Gauss–Legendre nodes and weights on `[a, b]`.
fn gauss_legendre(q: usize, a: f64, b: f64) -> Vec<(f64, f64)> {
    let (mid, half) = (0.5 * (a + b), 0.5 * (b - a));
    (0..q)
        .map(|k| {
            // Newton on the Legendre polynomial P_q from the usual first guess.
            let mut t = (std::f64::consts::PI * (k as f64 + 0.75) / (q as f64 + 0.5)).cos();
            let mut dp = 0.0;
            for _ in 0..100 {
                let (mut p0, mut p1) = (1.0, t);
                for m in 2..=q {
                    let m = m as f64;
                    (p0, p1) = (p1, ((2.0 * m - 1.0) * t * p1 - (m - 1.0) * p0) / m);
                }
                dp = q as f64 * (t * p1 - p0) / (t * t - 1.0);
                let step = p1 / dp;
                t -= step;
                if step.abs() < 1e-15 {
                    break;
                }
            }
            (mid + half * t, half * 2.0 / ((1.0 - t * t) * dp * dp))
        })
        .collect()
}

/// Which samples a configuration's probability counts.
#[derive(Clone, Copy, Debug)]
enum Readout {
    /// Only while every `|x_i| ≥ x₀/2`.
    Drop,
    /// Every sample.
    Sign,
}

/// Nodes per segment, and the cut-off in units of `x₀`: one well there is `ΔV·(2.6² − 1)²
/// ≈ 33·ΔV` above its minimum.
const NODES: usize = 24;
const CUT_OFF: f64 = 2.6;

/// `P_cont(s)` for both readouts, indexed by configuration bitmask (bit `i` set ⇔ `x_i > 0`).
fn spin_distributions(physics: Physics, p: &Problem) -> (Vec<f64>, Vec<f64>) {
    let x_0 = physics.x_0;
    // |x| nodes: the barrier strip [0, x₀/2] (counted by Sign only) and the well [x₀/2, cut].
    let nodes: Vec<(f64, f64, bool)> = gauss_legendre(NODES, 0.0, 0.5 * x_0)
        .into_iter()
        .map(|(x, w)| (x, w, false))
        .chain(
            gauss_legendre(NODES, 0.5 * x_0, CUT_OFF * x_0)
                .into_iter()
                .map(|(x, w)| (x, w, true)),
        )
        .collect();
    let configurations = 1 << p.n;
    let (mut drop, mut sign) = (vec![0.0; configurations], vec![0.0; configurations]);
    let mut x = vec![0.0; p.n];
    visit(
        physics, p, &nodes, 0, &mut x, 0, true, 1.0, &mut drop, &mut sign,
    );
    for z in [&mut drop, &mut sign] {
        let total: f64 = z.iter().sum();
        for v in z.iter_mut() {
            *v /= total;
        }
    }
    (drop, sign)
}

/// Sum `exp(−energy/kT)` over every node tuple, site `k` onwards, given sites `0..k` at
/// `x[..k]` with weight `weight`.
#[allow(clippy::too_many_arguments)]
fn visit(
    physics: Physics,
    p: &Problem,
    nodes: &[(f64, f64, bool)],
    k: usize,
    x: &mut [f64],
    config: usize,
    in_wells: bool,
    weight: f64,
    drop: &mut [f64],
    sign: &mut [f64],
) {
    if k == p.n {
        sign[config] += weight;
        if in_wells {
            drop[config] += weight;
        }
        return;
    }
    let x_0 = physics.x_0;
    // The coupling of site k to the sites already placed, per unit x_k.
    let pull: f64 = p.field_h[k] / x_0
        + p.edges
            .iter()
            .zip(&p.coupling_j)
            .filter_map(|(&(i, j), &jk)| match (i, j) {
                (i, j) if j == k && i < k => Some(jk * x[i]),
                (i, j) if i == k && j < k => Some(jk * x[j]),
                _ => None,
            })
            .sum::<f64>()
            / (x_0 * x_0);
    for (bit, s) in [(0, -1.0), (1, 1.0)] {
        for &(magnitude, w, in_well) in nodes {
            let xk = s * magnitude;
            x[k] = xk;
            let site = (-(physics.well(xk) - pull * xk) / physics.k_b_t).exp();
            visit(
                physics,
                p,
                nodes,
                k + 1,
                x,
                config | (bit << k),
                in_wells && in_well,
                weight * w * site,
                drop,
                sign,
            );
        }
    }
}

/// The mean and variance of `x/x₀` in the right-hand well under `readout`.
fn well_moments(physics: Physics, readout: Readout) -> (f64, f64) {
    let lo = match readout {
        Readout::Drop => 0.5,
        Readout::Sign => 0.0,
    };
    let unit = Physics {
        x_0: 1.0,
        ..physics
    };
    let (mut z, mut m1, mut m2) = (0.0, 0.0, 0.0);
    for (y, w) in gauss_legendre(200, lo, CUT_OFF) {
        let b = w * (-unit.well(y) / unit.k_b_t).exp();
        z += b;
        m1 += b * y;
        m2 += b * y * y;
    }
    let mean = m1 / z;
    (mean, m2 / z - mean.powi(2))
}

/// The effective coupling of each edge: `kT` times the order-2 Walsh–Hadamard coefficient
/// of `ln P(s)` on that edge.
fn effective_couplings(physics: Physics, p: &Problem, distribution: &[f64]) -> Vec<f64> {
    let spin = |c: usize, i: usize| if c >> i & 1 == 1 { 1.0 } else { -1.0 };
    p.edges
        .iter()
        .map(|&(i, j)| {
            let coefficient: f64 = distribution
                .iter()
                .enumerate()
                .map(|(c, &prob)| prob.ln() * spin(c, i) * spin(c, j))
                .sum::<f64>()
                / distribution.len() as f64;
            physics.k_b_t * coefficient
        })
        .collect()
}

/// The effective field on each spin: `kT` times the order-1 Walsh–Hadamard coefficient of
/// `ln P(s)` on that spin.
fn effective_fields(physics: Physics, p: &Problem, distribution: &[f64]) -> Vec<f64> {
    let spin = |c: usize, i: usize| if c >> i & 1 == 1 { 1.0 } else { -1.0 };
    (0..p.n)
        .map(|i| {
            let coefficient: f64 = distribution
                .iter()
                .enumerate()
                .map(|(c, &prob)| prob.ln() * spin(c, i))
                .sum::<f64>()
                / distribution.len() as f64;
            physics.k_b_t * coefficient
        })
        .collect()
}

// ─── Effective couplings ─────────────────────────────────────────────────

/// On the complete graph K4, ferro- and antiferromagnetic, at tilt ratio 0.3 and `ΔV/kT = 3`,
/// in both readouts: every edge's effective coupling is `J·μ²` plus the induced term, within
/// 0.005. The first-order factor `μ²` is 0.914 with the drop and 0.822 sign-only, each more
/// than 0.05 from 1, so the band tells `μ²·J` from `J`.
///
/// The bands were set after cold reviewers' quadrature of the same integrals (residual
/// ≤ 0.002), not before.
#[test]
fn k4_realises_the_first_order_factor_and_the_induced_couplings() {
    let physics = Physics {
        delta_v: 3.0,
        x_0: 1.5,
        k_b_t: 1.0,
    };
    for sign in [1.0, -1.0] {
        let j = sign * Problem::coupling_at(0.3, 3, physics.delta_v);
        let problem = Problem::complete(4, j);
        let (drop, sign_only) = spin_distributions(physics, &problem);
        for (readout, distribution) in [(Readout::Drop, drop), (Readout::Sign, sign_only)] {
            let (mean, variance) = well_moments(physics, readout);
            let first_order = mean * mean;
            let documented = match readout {
                Readout::Drop => 0.914,
                Readout::Sign => 0.822,
            };
            assert!(
                (first_order - documented).abs() < 0.001,
                "{readout:?}: μ² is {first_order}, the docs say {documented}"
            );
            for (k, (&(i, l), j_eff)) in problem
                .edges
                .iter()
                .zip(effective_couplings(physics, &problem, &distribution))
                .enumerate()
            {
                let induced = first_order * variance * problem.shared_neighbour_sum(i, l)
                    / (physics.k_b_t * problem.coupling_j[k]);
                let ratio = j_eff / problem.coupling_j[k];
                eprintln!(
                    "J {j:+.4} {readout:?} edge ({i},{l}): J_eff/J {ratio:.5}, \
                     μ² {first_order:.5} + induced {induced:+.5}"
                );
                assert!(
                    (ratio - (first_order + induced)).abs() < 0.005,
                    "J {j}, {readout:?}, edge ({i},{l}): J_eff/J {ratio}, predicted {}",
                    first_order + induced
                );
            }
        }
    }
}

/// Fields enter at first order as `μ·h` (one factor of `μ`, where couplings get two), plus
/// `β·μ·σ²·Σ_l h_l·J_lj` at second order. Checked on `gibbs_sampler.rs` gate B's mixed problem
/// (couplings `[0.8, −0.3, 0.1, 0.5, −0.2, 0.6]`, fields `[0.3, −0.2, 0.0, 0.15]`, tilt
/// ratios 0.23–0.37 at `ΔV = 3`), both readouts, `ΔV/kT = 3`. The 0.005 band was set before
/// this test first ran; `μ²·h` instead of `μ·h` misses spin 0 by 0.013.
#[test]
fn fields_enter_with_one_factor_of_mu() {
    let physics = Physics {
        delta_v: 3.0,
        x_0: 1.5,
        k_b_t: 1.0,
    };
    let problem = Problem {
        n: 4,
        edges: vec![(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)],
        coupling_j: vec![0.8, -0.3, 0.1, 0.5, -0.2, 0.6],
        field_h: vec![0.3, -0.2, 0.0, 0.15],
    };
    let coupling = |a: usize, b: usize| {
        problem
            .edges
            .iter()
            .position(|&(p, q)| (p, q) == (a, b) || (p, q) == (b, a))
            .map_or(0.0, |k| problem.coupling_j[k])
    };
    let (drop, sign_only) = spin_distributions(physics, &problem);
    for (readout, distribution) in [(Readout::Drop, drop), (Readout::Sign, sign_only)] {
        let (mean, variance) = well_moments(physics, readout);
        let fields = effective_fields(physics, &problem, &distribution);
        for (j, h_eff) in fields.into_iter().enumerate() {
            let induced = mean * variance / physics.k_b_t
                * (0..problem.n)
                    .map(|l| problem.field_h[l] * coupling(l, j))
                    .sum::<f64>();
            let predicted = mean * problem.field_h[j] + induced;
            eprintln!(
                "{readout:?} spin {j}: h {:+.3}, h_eff {h_eff:+.5}, μ·h {:+.5} + induced {induced:+.5}",
                problem.field_h[j],
                mean * problem.field_h[j]
            );
            assert!(
                (h_eff - predicted).abs() < 0.005,
                "{readout:?}, spin {j}: h_eff {h_eff}, predicted {predicted}"
            );
        }
        for (k, (&(i, l), j_eff)) in problem
            .edges
            .iter()
            .zip(effective_couplings(physics, &problem, &distribution))
            .enumerate()
        {
            let predicted = mean * mean * problem.coupling_j[k]
                + mean * mean * variance / physics.k_b_t * problem.shared_neighbour_sum(i, l);
            eprintln!(
                "{readout:?} edge ({i},{l}): J {:+.3}, J_eff {j_eff:+.5}, predicted {predicted:+.5}",
                problem.coupling_j[k]
            );
            assert!(
                (j_eff - predicted).abs() < 0.005,
                "{readout:?}, edge ({i},{l}): J_eff {j_eff}, predicted {predicted}"
            );
        }
    }
}

// ─── Where a well vanishes ───────────────────────────────────────────────

/// Whether configuration `s` (bit `i` set ⇔ `x_i > 0`) has a local minimum of the energy
/// with its signs: descend from `s·x₀` and from small perturbations of it, and accept an
/// end point only if it keeps the signs and its Hessian is positive definite.
fn has_minimum(physics: Physics, p: &Problem, s: usize) -> bool {
    let spin = |i: usize| if s >> i & 1 == 1 { 1.0 } else { -1.0 };
    let starts = [
        [0.0; 4],
        [0.01, -0.01, 0.02, -0.02],
        [-0.02, 0.01, -0.01, 0.02],
    ];
    assert!(p.n <= 4, "the perturbed starts cover 4 spins");
    starts.iter().any(|delta| {
        let mut x: Vec<f64> = (0..p.n).map(|i| spin(i) * physics.x_0 + delta[i]).collect();
        let step = 1e-3 * physics.x_0 * physics.x_0 / physics.delta_v;
        let converged = (0..2_000_000).any(|_| {
            let g = physics.gradient(p, &x);
            if g.iter().map(|v| v * v).sum::<f64>().sqrt() < 1e-10 {
                return true;
            }
            x.iter_mut().zip(&g).for_each(|(xi, gi)| *xi -= step * gi);
            false
        });
        assert!(converged, "descent from configuration {s} did not converge");
        let signs_kept = (0..p.n).all(|i| x[i] * spin(i) > 0.0);
        signs_kept && hessian_is_positive_definite(physics, p, &x)
    })
}

/// Cholesky on the energy's Hessian at `x`.
fn hessian_is_positive_definite(physics: Physics, p: &Problem, x: &[f64]) -> bool {
    let (n, x_0) = (p.n, physics.x_0);
    let mut h = vec![vec![0.0; n]; n];
    for i in 0..n {
        h[i][i] = 4.0 * physics.delta_v / x_0.powi(2) * (3.0 * (x[i] / x_0).powi(2) - 1.0);
    }
    for (&(i, j), &jk) in p.edges.iter().zip(&p.coupling_j) {
        h[i][j] -= jk / (x_0 * x_0);
        h[j][i] -= jk / (x_0 * x_0);
    }
    for k in 0..n {
        let pivot = h[k][k] - (0..k).map(|m| h[k][m] * h[k][m]).sum::<f64>();
        if pivot <= 0.0 {
            return false;
        }
        h[k][k] = pivot.sqrt();
        for i in k + 1..n {
            h[i][k] = (h[i][k] - (0..k).map(|m| h[i][m] * h[k][m]).sum::<f64>()) / h[k][k];
        }
    }
    true
}

/// Whether every configuration keeps a minimum with its signs.
fn every_configuration_has_a_minimum(physics: Physics, p: &Problem) -> bool {
    (0..1 << p.n).all(|s| has_minimum(physics, p, s))
}

/// The tilt ratio `r` at which a well first vanishes depends on the graph and the sign of
/// the couplings, so no one value of `r` bounds the mapping. Brackets, at `T = 0` and no
/// field: the pair (analytic: the anti-aligned minimum becomes a saddle at `J = 2ΔV`, so
/// `r = 2/1.5396 = 1.299`), and the complete graph K4, ferro- and antiferromagnetic.
#[test]
fn the_tilt_ratio_where_a_well_first_vanishes_depends_on_the_graph() {
    let physics = Physics {
        delta_v: 3.0,
        x_0: 1.5,
        k_b_t: 1.0,
    };
    for (label, n, degree, sign, below, above) in [
        ("pair", 2, 1, 1.0, 1.28, 1.32),
        ("K4 ferro", 4, 3, 1.0, 0.90, 0.95),
        ("K4 antiferro", 4, 3, -1.0, 1.53, 1.58),
    ] {
        let at = |r: f64| {
            let problem =
                Problem::complete(n, sign * Problem::coupling_at(r, degree, physics.delta_v));
            // The tilt ratio the brackets are in is the public one.
            let reported = IsingProblem::new(
                n,
                problem.edges.clone(),
                problem.coupling_j.clone(),
                problem.field_h.clone(),
            )
            .tilt_ratios(physics.delta_v);
            assert!(
                reported.iter().all(|&ri| (ri - r).abs() < 1e-12),
                "{label}: tilt_ratios {reported:?}, r {r}"
            );
            every_configuration_has_a_minimum(physics, &problem)
        };
        assert!(at(below), "{label}: a well has vanished at r = {below}");
        assert!(!at(above), "{label}: every well survives at r = {above}");
    }
}
