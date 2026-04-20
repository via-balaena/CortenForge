# High-dimensional BayesOpt

Classical GP-BayesOpt on the Matérn-5/2 ARD kernel from [§00](00-gp.md) with the acquisitions from [§01](01-acquisitions.md) is known to degrade at high dimension: kernel length-scales $\ell_a$ become hard to identify when $d$ is large and $n$ is modest, the posterior variance becomes near-uniformly large, and acquisition functions flatten out. The parent spine's Section 4 names three approaches `sim-soft` commits to for scaling past the classical break-down; this leaf names what each one buys, when each is used, and which ones are in scope for Phase A-I versus scheduled as research.

## The classical break-down and the scaling path

The canonical problem's design space sits at $d \in [10, 50]$ per [Ch 00](../00-forward.md), with high-dimensional configurations reaching $\sim 200$. The scaling path, in rising-dimension order, is:

| Regime | Approach | Phase |
|---|---|---|
| Low-dimensional | Classical GP-BayesOpt — Matérn-5/2 ARD, EI / UCB, multi-start L-BFGS on the acquisition | Phase D |
| Group-decomposable | Additive kernel decomposition on top of the GP | Phase G |
| High-dimensional, non-decomposable | Trust-region BayesOpt (TuRBO) | Phase H |
| Beyond TuRBO — research frontier | Latent-space BayesOpt via a learned low-dimensional representation | Research ([Part 12 Ch 07](../../120-roadmap/07-open-questions.md)) |

The regime boundaries are approximate: the break-down of classical BayesOpt depends on the target function's *effective* dimensionality (how many axes $R(\theta)$ actually varies meaningfully along) more than on the input dimension. Classical GP-BayesOpt applies well beyond $d = 20$ on problems whose effective dimension is small; additive and trust-region structure earn their keep on problems where $R$ depends on most of its inputs. The parent spine's Section 4 gives the canonical-problem-specific cutoffs in dimension ($\sim 20$ for additive, $\sim 50$ for TuRBO); this leaf treats them as guidance for which path to reach for rather than as hard classifications.

## Additive kernel decomposition

If $R(\theta) = \sum_g R_g(\theta_g)$ decomposes into a sum of group-wise functions, each group acting on a subset $\theta_g$ of at most $\sim 10$ dimensions, then the GP posterior also decomposes group-wise:

$$
k(\theta, \theta') = \sum_g k_g(\theta_g, \theta'_g),
$$

and the full posterior mean factors as $\mu(\theta) = \sum_g \mu_g(\theta_g)$ with posterior variance $\sigma^2(\theta) = \sum_g \sigma^2_g(\theta_g)$ in the independent-groups case. The acquisition function is nonlinear in $\mu, \sigma$ and so does not itself decompose into a sum of per-group acquisitions, but the factored fit lets the inner-loop optimizer exploit the structure via block-coordinate ascent: run EI / UCB maximization per group, each inner loop operating only over its $\leq 10$-dimensional subspace, sweeping through the groups to convergence. The per-step cost is dominated by small-subspace inner optimizations rather than by a full-$d$ acquisition optimization.

This works when the design space actually decomposes at the group level. On the canonical problem, the SDF parameters from [Ch 00](../00-forward.md) (a primitive's geometry) typically interact weakly with material-field parameters for a *different* region — the reward is sensitive to both, but the dominant coupling is between a primitive and *its own* material field, not between geometry in one region and material in another. When the groups are chosen along those natural boundaries, the additive decomposition captures most of $R$'s structure. When the groups are chosen wrong, the decomposition misses cross-group interactions and the fit underperforms vanilla GP-BayesOpt.

Group-discovery is not automatic in the default `sim-soft` path — the groups are specified by the designer, informed by the SDF / material-field structure of the composition tree. An automatic-discovery variant (for example, mutual-information-based group detection on an initial Sobol sample) is a Phase-H add-on, not a Phase-G commitment.

## Trust-region BayesOpt (TuRBO)

TuRBO, from [Eriksson, Pearce, Gardner, Turner & Poloczek 2019 (NeurIPS)](https://arxiv.org/abs/1910.01739), maintains a trust region around the current best-seen point, runs classical GP-BayesOpt inside that region, and expands or contracts the region based on the success rate of recent candidate evaluations:

1. Initialize a hyperrectangle centered at the incumbent, with side lengths proportional to the kernel's ARD length-scales $\ell_a$.
2. Run classical BayesOpt (GP fit, acquisition optimization) restricted to the trust region. Candidates outside the region are excluded.
3. After a batch of evaluations, if a sufficient fraction improve on the incumbent, expand the region; if too few improve, contract. When the region contracts below a threshold, re-initialize from a fresh incumbent.

The scaling advantage is that a *local* GP fit can remain well-behaved at input dimensions where a *global* fit breaks down: inside a small region, the function can be approximated by a low-complexity kernel even when the global function's effective dimension is high, because the local variation is dominated by a smaller set of axes than the global variation is. Eriksson et al. empirically demonstrate this on benchmarks drawn from robotics, reinforcement learning, and scientific applications, showing that local-plus-region-adaptation reaches input dimensions where classical GP-BayesOpt no longer converges.

TuRBO also parallelizes naturally: a batch of candidates drawn from independent trust regions around diverse incumbents gives diverse candidates without the fantasy-update machinery batch-UCB needs. The parent spine's Section 4 commits TuRBO to the Phase H scope — after the additive-decomposition path in Phase G has addressed the problems where decomposition is available, TuRBO picks up the ones where it is not.

## Latent-space BayesOpt (research, not Phase A-I)

The third path is to learn a low-dimensional latent representation $z \in \mathbb R^{d_z}$ with $d_z \ll d$ from the first $\sim 200$ evaluations — a variational autoencoder on $(\theta, R)$ pairs is the common construction — and then run classical GP-BayesOpt in the latent space, decoding candidates back to $\theta$ before evaluation.

The appeal is that if the effective dimensionality of $R(\theta)$ is truly small, a learned latent can uncover it automatically, and BayesOpt in the low-dimensional latent recovers its favorable low-$d$ performance. The risks are load-bearing:

- **Representation quality is sample-coverage-bound.** The VAE's latent is only as good as the first $\sim 200$ evaluations' coverage of the design space. When that coverage is poor, the latent's coordinates are poorly calibrated, and BayesOpt in that latent chases artifacts of the representation rather than features of $R$.
- **Decoder error behaves as effective noise.** The decoder's reconstruction error contributes to effective noise in the latent-space GP's observations. The `GradientEstimate::Noisy` contract from [Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md) would need extending to cover latent-encoding noise as a separate source, with its own variance accounting in the acquisition functions of [§01](01-acquisitions.md).
- **Cache invalidation on latent re-training.** Any learned latent is specific to the evaluation set it was trained on. Re-training the latent mid-loop breaks the cached Cholesky factor from [§00](00-gp.md) and forces a full re-factor on the new latent coordinates — a cost the outer loop was not budgeted to pay.

For these reasons the spine schedules latent-space BayesOpt as an open question in [Part 12 Ch 07](../../120-roadmap/07-open-questions.md) rather than as a Phase A-I deliverable. Additive decomposition (Phase G) and TuRBO (Phase H) together cover the canonical design space's dimensional range without requiring the learned-representation apparatus.

## What this sub-leaf commits the book to

- **Classical GP-BayesOpt is the Phase-D default at low dimension.** The break-down regime boundary is effective-dimension-driven, not strict input-dimension-driven; the spine's $d \sim 20$ is guidance for the canonical problem.
- **Additive kernel decomposition (Phase G) handles group-decomposable problems.** Groups are designer-specified at Phase G, informed by the SDF / material-field structure of the `cf-design` composition tree; automatic group-discovery is a Phase-H add-on.
- **Trust-region BayesOpt (Eriksson et al. 2019, Phase H) handles high-dimensional non-decomposable problems** by running classical BayesOpt inside adaptively-sized local regions. Eriksson et al. demonstrate the technique on benchmarks drawn from robotics, reinforcement learning, and scientific applications.
- **Latent-space BayesOpt is research-scheduled** ([Part 12 Ch 07](../../120-roadmap/07-open-questions.md)), not Phase A-I. The VAE-calibration, decoder-noise, and cache-invalidation risks are real and would need separate treatment before that path is viable.
- **Regime boundaries are approximate and effective-dimension-dependent.** The regime table is a guide for which technique to reach for, not a hard classification.
