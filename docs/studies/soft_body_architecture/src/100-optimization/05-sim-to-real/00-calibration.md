# Calibration from measured prints

The parent spine commits to a residual GP that absorbs the systematic sim-to-real discrepancy as a learnable function of the design vector. This leaf names the decomposition the GP actually models (what is bias, what is noise), the per-component structure that turns the reward-breakdown into a bundle of independent residual GPs, the variance-convention block that keeps those GPs composable with [Ch 02 §00](../02-bayesopt/00-gp.md), and the initial-calibration protocol that fits the hyperparameters from the first batch of measured prints.

## The residual-GP discrepancy model

Writing the real-world reward as the sim reward plus an additive, learnable discrepancy term is the **Kennedy-O'Hagan calibration framework** from [Kennedy & O'Hagan 2001](https://rss.onlinelibrary.wiley.com/doi/abs/10.1111/1467-9868.00294):

$$
R_\text{real}(\theta) = R_\text{sim}(\theta) + r(\theta) + \varepsilon, \qquad r(\theta) \sim \mathrm{GP}(0, k_r(\theta, \theta')),
$$

where $r(\theta)$ is a GP-modeled *model-discrepancy* function on the design space and $\varepsilon$ absorbs the unmodeled per-observation fluctuations discussed below. Kennedy-O'Hagan's original framing handles computer-model calibration and model inadequacy jointly; `sim-soft` inherits the discrepancy-GP half of the construction and treats the sim's own physical parameters (silicone Prony coefficients, friction coefficients, etc.) as already calibrated upstream in [Part 1 Ch 04's material-data pipeline](../../10-physical/04-material-data.md). The residual GP carries only the gap that remains after the sim is best-possible on its own terms.

Two alternative sim-to-real strategies exist and are not what `sim-soft` ships:

- **Domain randomization** ([Tobin, Fong, Ray, Schneider, Zaremba & Abbeel 2017](https://arxiv.org/abs/1703.06907); dynamics-specific extension in [Peng, Andrychowicz, Zaremba & Abbeel 2018](https://arxiv.org/abs/1710.06537)) trains policies under a distribution over sim parameters wide enough that the real world appears as another randomized instance. Tobin's formulation targets visual-rendering parameters; Peng's targets dynamics parameters, which is the case closer to soft-body material randomization (stiffness, Prony terms, friction). The approach is common in RL-for-robotics but is ill-matched to soft-body *design* workflows: the design-space analogue would seek designs robust across a distribution of simulated worlds, whereas the designer is optimizing for one specific printer's behavior and wants the peak-performance design there rather than cross-printer robustness.
- **Adaptive domain randomization** ([Chebotar, Handa, Makoviychuk, Macklin, Issac, Ratliff & Fox 2019](https://arxiv.org/abs/1810.05687)) updates the sim-parameter distribution using real-world feedback so the sim converges toward real behavior. This modifies the *simulator's* parameters; residual-GP calibration modifies the *gap* between the simulator and reality. Both are online-learning approaches against real data; the difference is what state is being updated. Residual GPs preserve sim-side determinism — a given $\theta$ always produces the same $R_\text{sim}(\theta)$ — which keeps the BayesOpt inner loop's cached training data valid across correction updates, as [§01 online updating](01-online.md) discusses.

Soft-body-specific residual-physics correction has been applied in recent work: [Gao et al. 2024](https://arxiv.org/abs/2402.01086) learns a neural residual physics correction on top of an FEM simulator, using real-world rollouts to tighten sim-to-real agreement on soft robots. The construction here is a GP rather than a neural net and targets per-component reward residuals rather than a residual force field, but the lineage is direct: a learned additive correction on top of a first-principles simulator, trained on measured real-world outcomes.

## The three-part error decomposition

The discrepancy $R_\text{real}(\theta) - R_\text{sim}(\theta)$ observed in any finite experiment decomposes into three components, only one of which is learnable:

- **Systematic model bias $r(\theta)$.** A function of $\theta$ — the Prony coefficients were fit at one strain-rate range and the actual print lives at another, or the smoothed-Coulomb friction model misses the surface-roughness contribution that varies with patch geometry. The residual GP models this term and only this term.
- **Per-print manufacturing variance $\eta_\text{print}$.** Random per-print at a fixed $\theta$ — voxel-scale porosity, cure-gradient microstructure, voids near seams. Repeated prints of the same $\theta$ give a sample of this distribution, with variance $\sigma^2_\text{print}$. White across prints.
- **Per-measurement noise $\eta_\text{meas}$.** Random per-measurement at a fixed print — force sensor at $\pm 1\%$ accuracy, pressure sensor at $\pm 3\%$ patch-center accuracy degrading at patch edges. Independent across measurements on the same print, with variance $\sigma^2_\text{meas}$.

The GP's observation-noise variance is the sum of the last two,

$$
\sigma^2_\text{noise} = \sigma^2_\text{print} + \sigma^2_\text{meas},
$$

estimated jointly with the kernel hyperparameters during calibration and held constant during acquisition. The architectural split — one learnable structured term, two white unlearnable terms — is what lets the GP concentrate its length-scales on the systematic part rather than spreading its capacity across everything that looks stochastic.

## Per-component residuals and the aggregate reward GP

The reward is multi-modal per [Part 1 Ch 01](../../10-physical/01-reward.md), so the residual is too. Per the parent spine, `sim-soft` fits *independent* GPs per reward component: a scalar-output residual over the force curve, a vector-output residual over the contact-pressure field, a per-channel residual over the diffusion profile, and a scalar residual over effective stiffness. Independence at the component level reflects the physical story — the bias mechanism that shows up in force is different from the bias mechanism that shows up in diffusion profile — and keeps each GP's kernel at a manageable output dimensionality.

The correction is applied at the reward-composition layer:

$$
\hat R_\text{real}(\theta; w) = R_\text{sim}(\theta; w) + \sum_{c} w_c\, \mu_{r,c}(\theta),
$$

where $w$ is the preference-weighted reward-composition vector from [Ch 03](../03-preference.md)'s preference model and $\mu_{r,c}(\theta)$ is the posterior mean of the residual GP for component $c$. The pressure-field component is aggregated into a scalar contribution via the same per-patch norm [Part 1 Ch 01](../../10-physical/01-reward.md) uses for the reward definition; details are pushed through `RewardBreakdown::apply_residuals` rather than recomputed here.

**Satisfying the [Ch 04 §01](../04-active-learning/01-cost-aware.md) forward-ref.** Ch 04 §01 treats "the residual GP's posterior" as a single posterior its acquisitions operate against. At a fixed preference-weight state $w$, the aggregate reward-residual

$$
R_\text{real}(\theta; w) - R_\text{sim}(\theta; w) = \sum_c w_c\, r_c(\theta)
$$

is a linear combination of independent GPs and is therefore itself a GP — call it $r_\text{agg}(\theta; w)$. Its posterior mean is $\sum_c w_c\, \mu_{r,c}(\theta)$ and its predictive variance is $\sum_c w_c^2\, \sigma^2_{r,c}(\theta)$. The aggregate GP is the object Ch 04's information-gain acquisitions bind to, and it is well-defined because the per-component GPs are assumed independent at the model-specification level. When the preference weights update between outer-loop iterations, the aggregate posterior re-composes from the fixed per-component GPs — the weight update does not require re-fitting any residual GP.

## Variance convention

`sim-soft`'s calibration stack mixes variances from four different sources. The conventions for this chapter and downstream:

| Symbol | Meaning |
|---|---|
| $\sigma^2_\text{sim}(\theta)$ | Inner-loop BayesOpt GP predictive variance at $\theta$ (per [Ch 02 §00](../02-bayesopt/00-gp.md)). Includes $\sigma^2_\text{noise,sim}$ if the sim is noisy; typically not the case under deterministic [Ch 00 forward map](../00-forward.md) evaluations. |
| $\sigma^2_{r,c,\text{latent}}(\theta)$ | Per-component residual GP's latent variance at $\theta$ — uncertainty over the true $r_c(\theta)$ exclusive of observation noise. |
| $\sigma^2_{r,c}(\theta)$ | Per-component residual GP's predictive variance, $\sigma^2_{r,c,\text{latent}}(\theta) + \sigma^2_\text{noise,c}$. Follows the [Ch 02 §00](../02-bayesopt/00-gp.md) predictive-with-noise convention; this is what [Ch 04 §00](../04-active-learning/00-info-criteria.md)'s BALD-regression form consumes. |
| $\sigma^2_\text{noise,c}$ | Observation-noise variance for component $c$; $\sigma^2_\text{noise,c} = \sigma^2_{\text{print},c} + \sigma^2_{\text{meas},c}$ per the three-part decomposition above. |
| $\sigma^2_{r_\text{agg}}(\theta; w)$ | Aggregate reward-residual GP predictive variance at a weight state $w$: $\sum_c w_c^2\, \sigma^2_{r,c}(\theta)$ (scalar components; pressure-field component contributes its per-patch-norm variance). |

Every closed-form expression in [§01](01-online.md) and in downstream Ch 04 §00 / §01 consumption threads these symbols in their predictive-with-noise form. Dropping the subscript $c$ is only allowed when the discussion is explicitly about a single component or about the aggregate $r_\text{agg}$ and both ambiguities are resolved by context.

## Measurement protocol

Each calibration print runs a protocol selected from a set of per-component measurement capabilities:

- **Force curve.** Universal testing machine runs a prescribed squeeze-vs-displacement trajectory against the printed part; force is logged at sampled displacement values. Training data for the force GP is the residual $F_\text{real}(\theta, d) - F_\text{sim}(\theta, d)$ at each displacement sample; the GP's input is the stacked $(\theta, d)$ vector.
- **Contact-pressure map.** Tekscan pressure film or capacitive pressure array records per-patch pressure under a prescribed boundary condition; training data is the 2D per-patch pressure discrepancy after ICP-style registration of the real contact patch to the sim mesh patch.
- **Visual diffusion profile.** Per [Part 9 Ch 00](../../90-visual/00-sss.md)'s material-DB protocol, a flat coupon printed from the same batch is measured via integrating-sphere spectrophotometry, producing a per-channel scattering-vs-distance curve. Training data is the per-channel difference between the measured curve and the [`DiffusionProfile`](../../90-visual/00-sss.md) the MaterialField carried during simulation.
- **Effective stiffness.** Slope of force-vs-displacement at a reference indentation depth — a single scalar per print, derived from the force-curve measurement when available and direct-probed otherwise.

A given calibration print does not have to run all four — a `MeasurementProtocol` field tracks which signals were collected, and each residual GP trains only on the prints that contributed its signal. The [Ch 06 budget table](../06-full-loop.md#budget-scales) allots $\sim 15$ min per print on the instrumented bench; a full four-protocol pass fits inside that budget on the canonical hardware described there, but a reduced protocol (force curve only, for example) runs faster and is appropriate during rapid-iteration periods where the other residual GPs have already converged.

## The struct

```rust
use sim_ml_chassis::{Surrogate, Tensor};
use sim_soft::RewardBreakdown;

pub struct SimToRealCorrection {
    pub force_gp:     ResidualGp,           // scalar over (theta, depth)
    pub pressure_gp:  ResidualFieldGp,      // vector output over theta
    pub diffusion_gp: Vec<ResidualGp>,      // one per color channel
    pub stiffness_gp: ResidualGp,           // scalar over theta
    pub protocol:     MeasurementProtocol,  // which signals were collected per print
}

impl SimToRealCorrection {
    /// Apply the residual correction to a sim-side reward breakdown.
    pub fn correct(&self, theta: &Tensor<f64>, r_sim: &RewardBreakdown) -> RewardBreakdown {
        let f_residual  = self.force_gp.posterior_mean(theta);
        let p_residual  = self.pressure_gp.posterior_mean(theta);
        let d_residuals = self.diffusion_gp.iter().map(|gp| gp.posterior_mean(theta)).collect();
        let s_residual  = self.stiffness_gp.posterior_mean(theta);
        r_sim.apply_residuals(f_residual, p_residual, d_residuals, s_residual)
    }

    /// Aggregate predictive variance at theta under the preference-weighted composition.
    pub fn aggregate_variance(&self, theta: &Tensor<f64>, weights: &RewardWeights) -> f64 {
        /* sum_c w_c^2 * sigma_{r,c}^2(theta) across scalar components plus the pressure-field norm */
    }
}
```

`correct` is the per-sim-evaluation consumer that [Ch 06's full loop](../06-full-loop.md) calls once per inner-loop step. `aggregate_variance` is the consumer [Ch 04 §00](../04-active-learning/00-info-criteria.md)'s information-gain acquisitions call against the aggregate reward-residual GP when selecting the next print batch. Both read only the cached GP state; no re-training happens in the correction path.

## Initial hyperparameter fit from 10 to 30 anchor prints

The parent spine names the initial calibration budget at $\sim 10$–$30$ prints. The protocol:

1. **Design the anchor set.** Select $n_0 \in [10, 30]$ design points via a space-filling Latin-hypercube or maximin sampling over the reward-relevant subset of the design space, prioritizing coverage over the axes the sim-side BayesOpt has flagged as highest-variance under its inner-loop posterior. The anchor set is the calibration-time investment; its design matters because $n_0$ is small enough that every print has to carry weight.
2. **Print and measure.** Each anchor print runs the measurement protocol selected for that batch and produces a `MeasurementReport` with the components collected. Partial protocols are allowed; the per-component GPs ingest only the signals they received.
3. **Fit hyperparameters per component.** Each residual GP maximizes its marginal likelihood over its own training data:

   $$
   \log p(\mathbf r_c \mid \Theta_c, \sigma_{f,c}^2, \{\ell_{a,c}\}, \sigma^2_\text{noise,c}) = -\tfrac{1}{2}\mathbf r_c^\top K_c^{-1} \mathbf r_c - \tfrac{1}{2}\log|K_c| - \tfrac{n_c}{2}\log(2\pi),
   $$

   with length-scale priors informed by the sim-side kernel's post-fit length-scales — the residual is expected to vary on the same scales as the sim reward itself, and an uninformative prior at $n_c \in [10, 30]$ tends to overfit. [Rasmussen & Williams 2006 ch. 5](https://gaussianprocess.org/gpml/) is the canonical source for the marginal-likelihood optimization path and its gradient wrt hyperparameters.
4. **Hold out a calibration check.** With anchor sets this small, cross-validation is marginal; the [Ch 01 §02](../01-surrogate/02-uncertainty.md) reliability-diagram discipline carries over — the standardized residuals on held-out prints should be approximately standard-normal if the GP's uncertainty is calibrated. A failed check at $n_0$ triggers additional anchor prints rather than a kernel-class change; the initial fit is conservative by design.

The anchor-print investment is the one-time cost of entering the residual-GP regime. Once calibrated, the GP consumes one measured print per outer-loop iteration and contributes $\lt 1$ ms per correction call ([Ch 06's budget](../06-full-loop.md)), so its ongoing cost is negligible against the print-time and measurement-time investments the outer loop already pays.

## What this sub-leaf commits the book to

- **The residual GP is a Kennedy-O'Hagan discrepancy model** of the form $R_\text{real}(\theta) = R_\text{sim}(\theta) + r(\theta) + \varepsilon$ with $r \sim \mathrm{GP}$. Cites [Kennedy & O'Hagan 2001](https://rss.onlinelibrary.wiley.com/doi/abs/10.1111/1467-9868.00294) as the foundational reference; contrasts against domain-randomization (visual: [Tobin et al. 2017](https://arxiv.org/abs/1703.06907); dynamics: [Peng et al. 2018](https://arxiv.org/abs/1710.06537)) and adaptive domain-randomization ([Chebotar et al. 2019](https://arxiv.org/abs/1810.05687)) as the shipped strategy's alternatives; soft-body residual-physics lineage noted via [Gao et al. 2024](https://arxiv.org/abs/2402.01086).
- **The discrepancy decomposes into systematic bias (learnable, $\theta$-dependent) plus per-print variance (white, $\sigma^2_\text{print}$) plus per-measurement noise (white, $\sigma^2_\text{meas}$).** The GP models the bias and absorbs the other two into $\sigma^2_\text{noise} = \sigma^2_\text{print} + \sigma^2_\text{meas}$.
- **Per-component residual GPs are independent** at the model-specification level (force, pressure, diffusion, stiffness). Composed correction sums per-component residuals at the reward-composition layer; the aggregate reward-residual GP $r_\text{agg}(\theta; w)$ is the single GP [Ch 04 §01](../04-active-learning/01-cost-aware.md)'s acquisitions bind to, with posterior variance $\sum_c w_c^2 \sigma^2_{r,c}(\theta)$ at preference-weight state $w$.
- **Variance convention** follows [Ch 02 §00](../02-bayesopt/00-gp.md)'s predictive-with-noise rule: $\sigma^2_{r,c}(\theta) = \sigma^2_{r,c,\text{latent}}(\theta) + \sigma^2_\text{noise,c}$. Table in the body enumerates the symbols used in this chapter and their downstream consumption.
- **`SimToRealCorrection` wraps four GPs with a `MeasurementProtocol`** tracking which signals each print contributed. `correct` applies the composed residual to a sim-side `RewardBreakdown`; `aggregate_variance` exposes the aggregate reward-residual GP's predictive variance to [Ch 04](../04-active-learning.md) acquisitions.
- **Initial hyperparameter fit runs over $n_0 \in [10, 30]$ anchor prints** via marginal-likelihood maximization per component ([Rasmussen & Williams 2006 ch. 5](https://gaussianprocess.org/gpml/)), with length-scale priors informed by the sim-side BayesOpt kernel. Calibration-check reliability diagram reuses [Ch 01 §02](../01-surrogate/02-uncertainty.md)'s diagnostic machinery.
