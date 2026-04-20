# Optimization-loop papers

The Bayesian-optimization, active-learning, preference-learning, and sim-to-real cluster cited inline across [Part 10 (optimization loop)](../../100-optimization/00-forward.md). Five clusters: BayesOpt foundations (kriging-era EI through the modern survey); gradient-enhanced GPs and high-dimensional BayesOpt (derivative kernels, TuRBO); information-theoretic active learning (MES, PES, BALD, and cost-aware extensions); preference learning over pairwise ratings (probit-GP, Bradley–Terry, dueling bandits); and surrogate uncertainty plus sim-to-real calibration (deep ensembles, calibration diagnostics, Sobolev training, Kennedy–O'Hagan discrepancy, domain randomization, residual physics, and the GPML textbook). Each anchor below is traced to the specific Part 10 sub-leaf and claim the paper grounds.

## BayesOpt foundations

## Jones, Schonlau & Welch 1998 {#jones-schonlau-welch-1998}

*Efficient Global Optimization of Expensive Black-Box Functions.* Journal of Global Optimization 13(4), pp. 455–492, 1998. Authors: Donald R. Jones (General Motors R&D), Matthias Schonlau (National Institute of Statistical Sciences at the time), William J. Welch (University of Waterloo). DOI [10.1023/A:1008306431147](https://doi.org/10.1023/A:1008306431147).

The Expected Improvement (EI) acquisition's canonical reference. Introduces the EGO (Efficient Global Optimization) framework on a kriging (GP) prior, derives EI's closed form under the Gaussian posterior, and establishes the exploration-exploitation-balance argument that downstream BayesOpt inherits. Cited inline from [Part 10 Ch 02 §01 — Acquisitions](../../100-optimization/02-bayesopt/01-acquisitions.md) as the EI reference and default-acquisition motivation; EI is the default `Acquisition` variant `sim-soft` ships.

## Srinivas, Krause, Kakade & Seeger 2010 {#srinivas-2010}

*Gaussian Process Optimization in the Bandit Setting: No Regret and Experimental Design.* Proceedings of the 27th International Conference on Machine Learning (ICML 2010). arXiv:[0912.3995](https://arxiv.org/abs/0912.3995). Authors: Niranjan Srinivas (Caltech at press time), Andreas Krause (Caltech), Sham M. Kakade, Matthias W. Seeger.

The paper the BayesOpt community standardised on as "GP-UCB," though the paper itself frames the algorithm in information-gain terms rather than under that name. Proves sublinear cumulative regret for a broad class of GP kernels with a scheduled exploration coefficient $\beta_t$; the regret rate is controlled by the kernel's maximum information gain rather than input dimensionality directly. Cited inline from [Part 10 Ch 02 §01](../../100-optimization/02-bayesopt/01-acquisitions.md) as the UCB analysis reference; `sim-soft`'s `UpperConfidenceBound { beta }` variant follows the paper's form with the parent-spine convention $\mu + \beta_t\, \sigma$.

## Shahriari, Swersky, Wang, Adams & de Freitas 2016 {#shahriari-2016}

*Taking the Human Out of the Loop: A Review of Bayesian Optimization.* Proceedings of the IEEE 104(1), pp. 148–175, January 2016. DOI [10.1109/JPROC.2015.2494218](https://doi.org/10.1109/JPROC.2015.2494218); IEEE Xplore article ID [7352306](https://ieeexplore.ieee.org/document/7352306). Authors: Bobak Shahriari (University of British Columbia), Kevin Swersky (University of Toronto), Ziyu Wang (UBC), Ryan P. Adams (Harvard), Nando de Freitas (UBC / DeepMind at press time).

The modern survey of BayesOpt — covers kernel families, acquisition functions, constraint-handling, parallelism, and multi-fidelity variants in one treatment. Cited inline from [Part 10 Ch 02 (parent spine)](../../100-optimization/02-bayesopt.md) as the textbook reference for the method's breadth. The IEEE Xplore `7352306` is the article ID (not the DOI); both URL forms resolve.

## Frazier 2018 {#frazier-2018}

*A Tutorial on Bayesian Optimization.* arXiv:[1807.02811](https://arxiv.org/abs/1807.02811), July 2018. Preprint — no subsequent conference or journal publication. Author: Peter I. Frazier (Cornell University, ORIE).

The tutorial BayesOpt practitioners reach for first. Frazier covers the GP surrogate, EI / UCB / Thompson / Knowledge-Gradient acquisitions, acquisition-optimization inner loops, and the extension to batched and gradient-aware settings — all at the level of detail that makes the decisions `sim-soft` commits to in [Part 10 Ch 02 (parent spine)](../../100-optimization/02-bayesopt.md) and [Ch 02 §01](../../100-optimization/02-bayesopt/01-acquisitions.md) directly traceable. Cite as arXiv preprint; the paper has not been republished in a conference or journal venue.

## Gradient-enhanced GPs and high-dimensional BayesOpt

## Solak, Murray-Smith, Leithead, Leith & Rasmussen 2003 {#solak-2003}

*Derivative Observations in Gaussian Process Models of Dynamic Systems.* Advances in Neural Information Processing Systems 15 (NIPS 2002 proceedings, published 2003 by MIT Press). [NeurIPS-hosted abstract](https://proceedings.neurips.cc/paper/2002/hash/5b8e4fd39d9786228649a8a8bec4e008-Abstract.html). Authors: Ercan Solak, Roderick Murray-Smith (Hamilton Institute, NUI Maynooth at press time), William E. Leithead, Douglas Leith, Carl Edward Rasmussen.

Develops the value-derivative and derivative-derivative cross-covariance construction that turns a standard GP into a gradient-enhanced GP in the dynamic-systems context: differentiate the kernel analytically to extend the joint-Gaussian distribution over observations to include derivative observations, then condition on the stacked value-and-derivative vector. Cited inline from [Part 10 Ch 02 §00 — Gaussian processes](../../100-optimization/02-bayesopt/00-gp.md) as the kernel-augmentation construction reference. `sim-soft`'s `GradientEnhancedGP` ingests $(\theta_i, R_i, \nabla_\theta R_i)$ triples under this construction; the Matérn-5/2 kernel is twice-differentiable everywhere, so the mixed partials are well-defined closed-form expressions.

## Wu, Poloczek, Wilson & Frazier 2017 {#wu-2017}

*Bayesian Optimization with Gradients.* Advances in Neural Information Processing Systems 30 (NeurIPS 2017). arXiv:[1703.04389](https://arxiv.org/abs/1703.04389). Authors: Jian Wu (Cornell), Matthias Poloczek (University of Arizona at press time), Andrew Gordon Wilson (Cornell), Peter I. Frazier (Cornell).

Introduces the derivative-enabled Knowledge Gradient (dKG) acquisition built on top of a gradient-enhanced GP of the [Solak 2003](#solak-2003) style, and reports state-of-the-art performance against both gradient-free and gradient-using baselines on synthetic and real-world benchmarks. Cited inline from [Part 10 Ch 02 §00](../../100-optimization/02-bayesopt/00-gp.md) as the BayesOpt-side paper that uses the gradient-enhanced construction and from [Part 10 Ch 01 §01 — Training on sim evaluations](../../100-optimization/01-surrogate/01-training.md) as the GP analogue of neural Sobolev training. The paper does not commit to a specific sample-efficiency ratio; the sample-efficiency argument in `sim-soft`'s spine is framed as intuition rather than a cited headline result.

## Eriksson, Pearce, Gardner, Turner & Poloczek 2019 {#eriksson-2019}

*Scalable Global Optimization via Local Bayesian Optimization.* Advances in Neural Information Processing Systems 32 (NeurIPS 2019), pp. 5497–5508. arXiv:[1910.01739](https://arxiv.org/abs/1910.01739). Authors: David Eriksson (Uber AI at press time), Michael Pearce (University of Warwick), Jacob R. Gardner (Uber AI), Ryan Turner (Uber AI), Matthias Poloczek (Uber AI).

TuRBO — trust-region Bayesian optimization. Maintains a hyperrectangular trust region around the current best-seen point, runs classical GP-BayesOpt inside the region, and expands or contracts based on recent-candidate success rate. The scaling advantage is that a local GP fit can remain well-behaved at input dimensions where a global fit breaks down. Cited inline from [Part 10 Ch 02 §02 — High-dimensional BayesOpt](../../100-optimization/02-bayesopt/02-high-dim.md) as the Phase-H scaling path for non-decomposable high-dimensional problems. The parent spine softened its original "$d \sim 100$" claim to "reaches dimensions where classical GP-BayesOpt no longer converges" during the Ch 02 source-verification pass; the paper's experiments span several benchmark families but do not commit to a single maximum-$d$ number.

## Information-theoretic active learning

## Wang & Jegelka 2017 {#wang-jegelka-2017}

*Max-value Entropy Search for Efficient Bayesian Optimization.* Proceedings of the 34th International Conference on Machine Learning (ICML 2017), PMLR 70. arXiv:[1703.01968](https://arxiv.org/abs/1703.01968). Authors: Zi Wang, Stefanie Jegelka (both MIT CSAIL).

Introduces MES — max-value entropy search — as a mutual-information acquisition whose latent is the global-optimum *value* $y^\ast$ rather than the maximizer location $x^\ast$ or the function realization $f$. The conditional entropy under $y \le y^\ast$ truncation is closed-form (truncated-normal entropy), and the outer expectation over $y^\ast$ is Monte-Carlo'd via a Gumbel fit to the posterior maximum — both steps cheap compared to the physical-print wait the acquisition sits inside. Cited inline from [Part 10 Ch 04 §00 — Information-theoretic criteria](../../100-optimization/04-active-learning/00-info-criteria.md) as the default physical-print-layer acquisition; `sim-soft`'s `MaxValueEntropySearch { n_ystar_samples }` variant implements the Wang–Jegelka construction.

## Hernández-Lobato, Hoffman & Ghahramani 2014 {#hernandez-lobato-2014}

*Predictive Entropy Search for Efficient Global Optimization of Black-box Functions.* Advances in Neural Information Processing Systems 27 (NeurIPS 2014). arXiv:[1406.2541](https://arxiv.org/abs/1406.2541). Authors: José Miguel Hernández-Lobato, Matthew W. Hoffman, Zoubin Ghahramani (all University of Cambridge at press time).

Introduces PES — predictive entropy search — the mutual-information-acquisition counterpart to [MES](#wang-jegelka-2017) with the latent $z = x^\ast$ (the maximizer location) rather than $y^\ast$ (the maximum value). Asks "where is the optimum?" where MES asks "what is the optimum?" Cited inline from [Part 10 Ch 04 §00](../../100-optimization/04-active-learning/00-info-criteria.md) as the latent-completeness reference; `sim-soft` does not ship PES because sampling $x^\ast$ requires an inner optimization per draw (same order of cost as the outer BayesOpt inner loop), and the physical-print use case routes both latents through the same downstream decision.

## Houlsby, Huszár, Ghahramani & Lengyel 2011 {#houlsby-2011}

*Bayesian Active Learning for Classification and Preference Learning.* arXiv:[1112.5745](https://arxiv.org/abs/1112.5745), December 2011. Preprint — no formal conference publication. Authors: Neil Houlsby, Ferenc Huszár, Zoubin Ghahramani, Máté Lengyel (all University of Cambridge / Gatsby Unit at press time).

Introduces BALD — Bayesian Active Learning by Disagreement — with the latent $z = f$, the underlying function realization itself, derived for GP *classification* with Bernoulli likelihoods. The acquisition is expected KL divergence between the full posterior predictive and the predictive conditioned on a single function draw. Cited inline from [Part 10 Ch 04 §00](../../100-optimization/04-active-learning/00-info-criteria.md) for the classification-BALD mutual-information identity; the regression adaptation (log signal-to-noise ratio of the GP posterior) is algebraic from that identity and is well-known in the GP-active-learning literature but is not derived in the original paper. The sub-leaf names the translation explicitly rather than silently extending the paper's framing.

## Snoek, Larochelle & Adams 2012 {#snoek-2012}

*Practical Bayesian Optimization of Machine Learning Algorithms.* Advances in Neural Information Processing Systems 25 (NIPS 2012). arXiv:[1206.2944](https://arxiv.org/abs/1206.2944). Authors: Jasper Snoek (University of Toronto at press time), Hugo Larochelle (Université de Sherbrooke at press time), Ryan P. Adams (Harvard).

The canonical practical-BayesOpt paper for machine-learning hyperparameter tuning. Covers GP choice, acquisition-function design, and — most load-bearing for this anchor — the cost-weighted expected-improvement-per-second formulation, dividing the acquisition by the expected evaluation cost for settings where training-time budget varies by orders of magnitude across the hyperparameter space. Cited inline from [Part 10 Ch 04 §01 — Cost-aware active learning](../../100-optimization/04-active-learning/01-cost-aware.md) as the canonical reference for the division-by-cost acquisition pattern in the hyperparameter-tuning setting; `sim-soft`'s sim-versus-print cost asymmetry is a direct analogue of the training-time budget asymmetry Snoek et al. address.

## Kandasamy, Dasarathy, Schneider & Póczos 2017 {#kandasamy-2017}

*Multi-fidelity Bayesian Optimisation with Continuous Approximations.* Proceedings of the 34th International Conference on Machine Learning (ICML 2017), PMLR 70, pp. 1799–1808. arXiv:[1703.06240](https://arxiv.org/abs/1703.06240). Authors: Kirthevasan Kandasamy, Gautam Dasarathy, Jeff Schneider, Barnabás Póczos (all Carnegie Mellon University at press time).

Generalises cost-aware BayesOpt to continuous-fidelity settings — the evaluation outcome depends on a $(\theta, \text{fidelity})$ joint input, with the fidelity axis continuous and controlling evaluation cost and noise level jointly. The acquisition is defined over the joint space rather than normalised per-$\theta$. Cited inline from [Part 10 Ch 04 §01](../../100-optimization/04-active-learning/01-cost-aware.md) as the generalisation reference `sim-soft` explicitly does *not* adopt: the physical-print layer has discrete fidelity levels (sim vs print), so the quotient form $\mathrm{IG}(\theta)/c(\theta)$ from [Snoek 2012](#snoek-2012) is the simpler and sufficient model.

## Preference learning

## Bradley & Terry 1952 {#bradley-terry-1952}

*Rank Analysis of Incomplete Block Designs: I. The Method of Paired Comparisons.* Biometrika 39(3–4), pp. 324–345, 1952. DOI [10.1093/biomet/39.3-4.324](https://doi.org/10.1093/biomet/39.3-4.324). Authors: Ralph Allan Bradley (Virginia Polytechnic Institute at press time), Milton E. Terry (Florida State University at press time).

The original parametric model for paired-comparison data: for items $\{1, \ldots, N\}$ with positive strength parameters $\pi_i$, the model is $P(i \succ j) = \pi_i / (\pi_i + \pi_j)$. Cited inline from [Part 10 Ch 03 §01 — Bradley–Terry model](../../100-optimization/03-preference/01-bradley-terry.md) as the likelihood form's source; the exponential reparameterization $\pi_i = \exp(s_i)$ that turns the model into a sigmoid of the latent-strength difference, and the GP-prior extension over a continuous weight space, are modern augmentations rather than the 1952 paper's own content.

## Chu & Ghahramani 2005 {#chu-ghahramani-2005}

*Preference Learning with Gaussian Processes.* Proceedings of the 22nd International Conference on Machine Learning (ICML 2005), pp. 137–144. ACM DOI [10.1145/1102351.1102369](https://doi.org/10.1145/1102351.1102369). [ICML proceedings PDF](https://icml.cc/Conferences/2005/proceedings/papers/018_Preference_ChuGhahramani.pdf). Authors: Wei Chu, Zoubin Ghahramani.

The canonical paper on preference learning with GP priors. The likelihood for a pair $(w_A, w_B)$ with observed preference $w_A \succ w_B$ is the probit (standard-normal CDF) form $\Phi((f(w_A) - f(w_B))/\sqrt{2}\sigma_n)$, arising naturally from modelling designer-judgment noise as additive Gaussian on the latent utility. The primary inference path is a Laplace approximation at the mode of the log-posterior. Cited inline from [Part 10 Ch 03 §00 — GP over pairwise ratings](../../100-optimization/03-preference/00-gp-pairs.md) as the probit-likelihood source and the Laplace-approximation reference; `sim-soft`'s `PreferenceGP` implements this construction.

## Yue, Broder, Kleinberg & Joachims — COLT 2009 + JCSS 2012 {#yue-broder-kleinberg-joachims-2012}

*The K-armed Dueling Bandits Problem.* Two closely-related papers, typically cited together:

- **COLT 2009 original:** Proceedings of the 22nd Annual Conference on Learning Theory (COLT 2009), [conference PDF](https://www.cs.cornell.edu/people/tj/publications/yue_etal_09a.pdf).
- **JCSS 2012 extension:** Journal of Computer and System Sciences 78(5), pp. 1538–1556, 2012. DOI [10.1016/j.jcss.2011.12.028](https://doi.org/10.1016/j.jcss.2011.12.028).

Authors: Yisong Yue (Cornell at press time), Josef Broder (Cornell), Robert Kleinberg (Cornell), Thorsten Joachims (Cornell). The JCSS 2012 paper extends the COLT 2009 original with refined analysis and proofs; both share the title above.

Introduces the dueling-bandit framework — the learner selects a pair of arms at each round and observes only a noisy preference outcome between them. The core algorithmic contribution is the Interleaved Filter algorithm family (IF1 and IF2 in the published analysis), progressively narrowing the candidate-arm pool by eliminating arms that lose enough head-to-head duels with statistically high confidence. The regret bounds are cumulative-regret-under-stochastic-transitivity guarantees, not a universal-setting result. Cited inline from [Part 10 Ch 03 §02 — Dueling bandits](../../100-optimization/03-preference/02-dueling.md) as the framework source and the Interleaved-Filter regret-bounded fallback; `sim-soft`'s default is the KL-based pair-selection rule with Interleaved Filter exposed as a knob.

## Surrogate uncertainty and sim-to-real calibration

## Rasmussen & Williams 2006 {#rasmussen-williams-2006}

*Gaussian Processes for Machine Learning.* MIT Press, 2006. ISBN 978-0-262-18253-9. Authors: Carl Edward Rasmussen (Max Planck Institute for Biological Cybernetics / University of Cambridge), Christopher K. I. Williams (University of Edinburgh). [Author-hosted full-text PDF](https://gaussianprocess.org/gpml/).

The canonical textbook for Gaussian processes in machine learning. Chapter 2 (Regression) develops the Cholesky-based GP inference pattern including the predictive-mean + predictive-variance forms and the $\alpha = K^{-1}y$ cache; Chapter 5 (Model Selection and Adaptation of Hyperparameters) covers marginal-likelihood maximisation and its gradient with respect to kernel hyperparameters. Cited inline from [Part 10 Ch 05 §00 — Calibration from measured prints](../../100-optimization/05-sim-to-real/00-calibration.md) (Chapter 5 for the anchor-print marginal-likelihood fit) and [§01 — Online updating](../../100-optimization/05-sim-to-real/01-online.md) (Chapter 2 for the bordered-Cholesky incremental-update pattern). The book's treatment of the Cholesky-factor-caching pattern is the reference that makes the $O(n^2)$ per-print update cost tractable for `sim-soft`'s residual GP.

## Kennedy & O'Hagan 2001 {#kennedy-ohagan-2001}

*Bayesian Calibration of Computer Models.* Journal of the Royal Statistical Society, Series B (Statistical Methodology) 63(3), pp. 425–464, 2001. DOI [10.1111/1467-9868.00294](https://doi.org/10.1111/1467-9868.00294). Authors: Marc C. Kennedy (University of Sheffield at press time), Anthony O'Hagan (University of Sheffield).

The canonical computer-model calibration framework. Models the real-world observation as $y(x) = \eta(x, \theta) + \delta(x) + \varepsilon$, where $\eta$ is the simulator's output at calibrated parameter $\theta$, $\delta(x)$ is a GP-modelled *model-discrepancy* function on the input space, and $\varepsilon$ is observation noise. The paper handles calibration of $\theta$ and model-inadequacy estimation jointly. Cited inline from [Part 10 Ch 05 §00](../../100-optimization/05-sim-to-real/00-calibration.md) as the foundational reference for `sim-soft`'s discrepancy model $R_\text{real}(\theta) = R_\text{sim}(\theta) + r(\theta) + \varepsilon$; `sim-soft` inherits the discrepancy-GP half of the construction and treats the sim's own physical parameters as already calibrated upstream through [Part 1 Ch 04's material-data pipeline](../../10-physical/04-material-data.md).

## Tobin, Fong, Ray, Schneider, Zaremba & Abbeel 2017 {#tobin-2017}

*Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World.* IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2017). arXiv:[1703.06907](https://arxiv.org/abs/1703.06907). Authors: Josh Tobin (OpenAI / UC Berkeley at press time), Rachel Fong (OpenAI), Alex Ray (OpenAI), Jonas Schneider (OpenAI), Wojciech Zaremba (OpenAI), Pieter Abbeel (UC Berkeley / OpenAI).

The origin of visual-domain-randomization for sim-to-real transfer. Trains object-detection policies on simulated images with randomized textures, lighting, and camera placements wide enough that the real world appears as another sample from the training distribution. Cited inline from [Part 10 Ch 05 §00](../../100-optimization/05-sim-to-real/00-calibration.md) as one of the two alternative sim-to-real strategies `sim-soft` *does not* ship — the visual-rendering-focused formulation — contrasted against the residual-GP discrepancy approach.

## Peng, Andrychowicz, Zaremba & Abbeel 2018 {#peng-2018}

*Sim-to-Real Transfer of Robotic Control with Dynamics Randomization.* IEEE International Conference on Robotics and Automation (ICRA 2018). arXiv:[1710.06537](https://arxiv.org/abs/1710.06537). Authors: Xue Bin Peng (UC Berkeley at press time), Marcin Andrychowicz (OpenAI), Wojciech Zaremba (OpenAI), Pieter Abbeel (UC Berkeley / OpenAI).

Extends domain randomization to dynamics parameters — friction, mass, joint damping, actuator delays — for robotic-control sim-to-real transfer. Paired with [Tobin 2017](#tobin-2017) in [Part 10 Ch 05 §00](../../100-optimization/05-sim-to-real/00-calibration.md)'s domain-randomization contrast because the soft-body case is dynamics-heavy (silicone stiffness, Prony terms, friction), closer to Peng's formulation than Tobin's visual-first one. The dual citation was added during Ch 05 Pass 3 after a user-prompted re-audit of the visual-vs-dynamics DR distinction.

## Chebotar, Handa, Makoviychuk, Macklin, Issac, Ratliff & Fox 2019 {#chebotar-2019}

*Closing the Sim-to-Real Loop: Adapting Simulation Randomization with Real World Experience.* IEEE International Conference on Robotics and Automation (ICRA 2019). arXiv:[1810.05687](https://arxiv.org/abs/1810.05687). Authors: Yevgen Chebotar, Ankur Handa, Viktor Makoviychuk, Miles Macklin, Jan Issac, Nathan Ratliff, Dieter Fox (all NVIDIA / University of Washington at press time).

Adaptive domain randomization — updates the sim-parameter distribution based on real-world rollout feedback so the simulator converges toward real behavior. Modifies the *simulator's* parameters, whereas residual-GP calibration modifies the *gap* between a fixed simulator and reality. Cited inline from [Part 10 Ch 05 §00](../../100-optimization/05-sim-to-real/00-calibration.md) as the second alternative strategy `sim-soft` does not ship, and from [Part 10 Ch 05 §01 — Online updating](../../100-optimization/05-sim-to-real/01-online.md) as the contrast case that clarifies why residual-GP updating preserves sim-side determinism: a given $\theta$ produces the same $R_\text{sim}(\theta)$ across the life of the run, keeping the BayesOpt inner-loop GP's cached training data valid, whereas adapting sim parameters invalidates the cache.

## Gao, Michelis, Spielberg & Katzschmann 2024 {#gao-2024}

*Sim-to-Real of Soft Robots with Learned Residual Physics.* arXiv:[2402.01086](https://arxiv.org/abs/2402.01086), submitted February 2024. Authors: Junpeng Gao, Mike Yan Michelis, Andrew Spielberg, Robert K. Katzschmann (Soft Robotics Lab, ETH Zürich at press time).

A learned neural residual-physics correction on top of an FEM simulator for soft robots, trained on real-world rollouts to tighten sim-to-real agreement. Cited inline from [Part 10 Ch 05 §00](../../100-optimization/05-sim-to-real/00-calibration.md) as the closest published analogue for the residual-physics construction `sim-soft` commits to. The `sim-soft` construction is a GP over per-component reward residuals rather than a neural residual force field, but the lineage — a learned additive correction on top of a first-principles simulator, trained on measured real-world outcomes — is direct. The sub-leaf's wording was softened from the pre-verification "pneumatic soft-body robots" to the paper's actual "soft robots" framing during Ch 05 Pass 3.

## Lakshminarayanan, Pritzel & Blundell 2017 {#lakshminarayanan-2017}

*Simple and Scalable Predictive Uncertainty Estimation using Deep Ensembles.* Advances in Neural Information Processing Systems 30 (NeurIPS 2017). arXiv:[1612.01474](https://arxiv.org/abs/1612.01474). Authors: Balaji Lakshminarayanan, Alexander Pritzel, Charles Blundell (all DeepMind).

The deep-ensembles reference for predictive uncertainty. Trains $M$ identically-architected networks from independent random initializations on the same training set, reports the ensemble mean as point prediction and the empirical variance across members as predictive variance, and demonstrates empirically that deep ensembles produce better-calibrated uncertainty than MC-dropout on a range of regression and classification benchmarks — including sharper uncertainty response on out-of-distribution inputs, which is the regime BayesOpt acquisitions depend on for exploration. Cited inline from [Part 10 Ch 01 §00 — Neural surrogate architecture](../../100-optimization/01-surrogate/00-architecture.md) as the ensemble-construction reference and from [Part 10 Ch 01 §02 — Uncertainty quantification](../../100-optimization/01-surrogate/02-uncertainty.md) as the ensemble-vs-MC-dropout comparison source.

## Guo, Pleiss, Sun & Weinberger 2017 {#guo-2017}

*On Calibration of Modern Neural Networks.* Proceedings of the 34th International Conference on Machine Learning (ICML 2017), PMLR 70. arXiv:[1706.04599](https://arxiv.org/abs/1706.04599). Authors: Chuan Guo, Geoff Pleiss, Yu Sun, Kilian Q. Weinberger (all Cornell University at press time).

Establishes reliability-diagram calibration diagnostics and temperature scaling as a post-hoc correction for neural-network classification. Cited inline from [Part 10 Ch 01 §02](../../100-optimization/01-surrogate/02-uncertainty.md) as the framework reference whose classification-calibration diagnostic (predicted-vs-empirical coverage plots) `sim-soft` adapts to the regression setting. The post-hoc single-scalar $\tau$ rescaling of $\sigma$ that the sub-leaf commits to is the regression analogue of Guo et al.'s temperature scaling of softmax logits; the adaptation is explicit rather than silent.

## Czarnecki, Osindero, Jaderberg, Świrszcz & Pascanu 2017 {#czarnecki-2017}

*Sobolev Training for Neural Networks.* Advances in Neural Information Processing Systems 30 (NeurIPS 2017). arXiv:[1706.04859](https://arxiv.org/abs/1706.04859). Authors: Wojciech Marian Czarnecki, Simon Osindero, Max Jaderberg, Grzegorz Świrszcz, Razvan Pascanu (all DeepMind).

The Sobolev-training pattern for neural networks — adds first-derivative supervision (and higher-order where available) alongside value supervision in the training loss, so the model matches both $R$ and $\nabla_\theta R$. Cited inline from [Part 10 Ch 01 §01 — Training on sim evaluations](../../100-optimization/01-surrogate/01-training.md) as the loss-structure reference `sim-soft`'s neural surrogate inherits, paired with [Wu 2017](#wu-2017) as the GP analogue on the BayesOpt surrogate path. The gradient-supervision advantage at moderate input dimension turns each training sample's information rate into a multiple of the value-only rate.

## Skipped-and-why

Four citations were considered during Part 10's authoring and deliberately not anchored; the rationales are recorded here rather than silently dropped, per the Pass 3 "skipped-and-why" discipline established at Ch 05's Tobin-vs-Peng re-audit. A reader who expects to find these works and does not should be able to see *why* they are absent and decide whether that rationale is load-bearing for their use case.

- **Gal & Ghahramani 2016** — *Dropout as a Bayesian Approximation* (arXiv:1506.02142, ICML 2016). The MC-dropout-as-approximate-Bayesian-inference reference. Cited *by name* (no anchor) in [Part 10 Ch 01 §00](../../100-optimization/01-surrogate/00-architecture.md) and [§02](../../100-optimization/01-surrogate/02-uncertainty.md) as the technique the MC-dropout fallback implements. Not anchored because the sub-leaves describe MC-dropout as a *fallback for budget-constrained settings*; the load-bearing references for the chapter are [Lakshminarayanan 2017](#lakshminarayanan-2017) (the default) and [Guo 2017](#guo-2017) (the diagnostic). Pass 3 may promote the soft cite to an anchor if downstream chapters reach for MC-dropout as a primary.
- **Andrychowicz et al. 2020 ADR** — *Solving Rubik's Cube with a Robot Hand* (arXiv:1910.07113). Introduces Automatic Domain Randomization, the self-adapting-distribution extension of [Tobin 2017](#tobin-2017). Not anchored because `sim-soft` does not ship ADR and the sub-leaf's DR discussion is already covered by Tobin 2017 (visual-first) + [Peng 2018](#peng-2018) (dynamics-specific) + [Chebotar 2019](#chebotar-2019) (adaptive); ADR would be a fourth citation in a cluster the sub-leaf explicitly contrasts against the residual-GP path the book commits to.
- **Koos, Mouret & Doncieux 2010** — *Crossing the Reality Gap in Evolutionary Robotics by Promoting Transferable Controllers* (GECCO 2010). The historical evolutionary-robotics reality-gap reference. Not anchored because the sub-leaves' sim-to-real lineage flows through Kennedy–O'Hagan 2001 (calibration framework) and Gao 2024 (modern residual-physics analogue for soft robots); Koos et al.'s GA-focused transferability treatment is further from `sim-soft`'s load-bearing construction than the distance the anchor list targets.
- **Mockus 1974** — *On Bayesian Methods for Seeking the Extremum* (Technical Conference on Optimization Techniques, IFIP 1974). The original Expected-Improvement antecedent that [Jones 1998](#jones-schonlau-welch-1998) re-framed in the kriging-response-surface context. Not anchored because the sub-leaves cite [Jones 1998](#jones-schonlau-welch-1998) as the EI reference the BayesOpt literature standardised on; Mockus 1974 is the pre-history, not the load-bearing construction. A history-of-EI discussion would anchor it; the operational commitments in [Ch 02 §01](../../100-optimization/02-bayesopt/01-acquisitions.md) do not.
