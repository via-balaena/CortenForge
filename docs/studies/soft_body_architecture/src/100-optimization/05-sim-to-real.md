# Sim-to-real bias correction

This is the chapter where the [Part 1 Ch 03 thesis](../10-physical/03-thesis.md)'s claim to "serve real engineering workflows" is cashed out or defaulted on. Every simulator — `sim-soft` included — is systematically wrong against the physical system it models. Silicone exhibits strain-rate dependence the Prony-series approximation misses by a few percent; 3D printing produces voxel-scale porosity the MaterialField does not parameterize; measured contact friction has surface-roughness terms the smoothed-Coulomb model smears. Each of these biases is small in isolation and large in aggregate, and the aggregate is *systematic* — it does not wash out across designs. Without a correction loop, the optimal-in-sim design is not the optimal-in-reality design, and the platform's claim to design real parts is a lie.

This chapter commits to the sim-to-real bias correction as a first-class subsystem of `sim-soft`'s optimization stack, structurally equivalent to [Ch 02's BayesOpt](02-bayesopt.md) and distinct enough to justify its own elevated treatment. The two sub-chapters walk calibration (one-shot fit from measured-print data) and online updating (incremental update as prints arrive).

| Section | What it covers |
|---|---|
| [Calibration from measured prints](05-sim-to-real/00-calibration.md) | The residual GP $r_\text{GP}(\theta) = R_\text{real}(\theta) - R_\text{sim}(\theta)$; measurement protocol (force curve, contact-patch pressure map, measured diffusion profile); initial hyperparameter fit from ≈10–30 prints |
| [Online updating](05-sim-to-real/01-online.md) | Incremental posterior update per new measured print; change-detection on hyperparameters (drift in the printer's state over months); batch re-fit when drift is detected |

Four claims Ch 05 commits to.

## 1. The bias is structural, not noise

A naive view of sim-to-real discrepancy treats it as measurement noise — fit the sim to average out the error. This is wrong. The discrepancy decomposes into three categories, only one of which is noise:

- **Systematic model bias** — the Prony-series coefficients in the MaterialField were fit to DMA data at one temperature and one strain-rate range; the physical sample lives at different conditions. Bias is a *function of $\theta$*, not a constant. Silicone at 40°C is different from silicone at 20°C, and the bias shows up as a temperature-dependent over- or under-prediction of effective stiffness. Fitting this out requires a bias *model*, not a constant offset.
- **Manufacturing variance** — the specific print has voids, seams, and cure-gradient microstructure the MaterialField cannot parameterize. Random per-print, not per-design. This part *is* noise; repeated prints of the same $\theta$ give a sample of this distribution.
- **Measurement noise** — the force sensor has ±1% accuracy, the contact-pressure sensor has ±3% accuracy at the patch center and higher at the edges. Random per-measurement, not per-print.

The residual GP in `sim-soft` models *only* the first term — the systematic model bias as a function of $\theta$. The per-print manufacturing variance enters as the GP's *observation noise*, which is per-sample. Measurement noise enters the same way. The architectural split is that the systematic bias has a learnable structure (it is a smooth function of $\theta$) while the other two do not (they are white).

## 2. Calibration is multi-modal, not a single scalar

Real-world validation of a simulated design is not a single-scalar comparison. The canonical problem's reward has four or five components per [Part 1 Ch 01](../10-physical/01-reward.md), and the bias in each component is potentially different. `sim-soft` commits to *per-component* calibration against per-component measurements:

- **Force curve.** Measured force-vs-squeeze on a universal testing machine. Sim prediction: same quantity, from the IPC contact force integrated over the contact patch under prescribed boundary displacement. Residual: $r_F(\theta, d) = F_\text{real}(\theta, d) - F_\text{sim}(\theta, d)$ as a function of squeeze depth $d$.
- **Contact-pressure map.** Measured with a Tekscan pressure film or a capacitive pressure array. Sim prediction: per-vertex pressure from `readout`. Residual: 2D per-vertex or per-patch pressure discrepancy after registration.
- **Visual diffusion profile.** Measured per [Part 9 Ch 00's material-DB protocol](../90-visual/00-sss.md) via integrating-sphere spectrophotometry on a flat coupon from the same print batch. Sim prediction: the `DiffusionProfile` field the MaterialField declared. Residual: curve-fit discrepancy.
- **Effective stiffness at a probe.** Measured as slope of force-vs-displacement at a reference indentation depth. Sim prediction: same. Residual: scalar, a single number per print.

Per-component residuals are modeled by independent GPs — the bias in force is not structurally the same as the bias in diffusion profile — and the composed correction is the sum of the per-component corrections at the reward-composition layer. This is more machinery than a single-scalar correction but it is the correct amount of machinery for the problem; hiding the multi-modality inside a scalar correction would blur the signal and slow the convergence of the loop.

```rust
use sim_ml_chassis::{Tensor, Surrogate};

pub struct SimToRealCorrection {
    pub force_gp:     ResidualGp,           // scalar-output GP over (theta, depth)
    pub pressure_gp:  ResidualFieldGp,      // vector-output GP over theta, producing a pressure field
    pub diffusion_gp: Vec<ResidualGp>,      // one per color channel
    pub stiffness_gp: ResidualGp,           // scalar GP over theta
    pub protocol:     MeasurementProtocol,  // which measurements were collected per print
}

impl SimToRealCorrection {
    /// Apply the residual correction to a sim-side reward.
    pub fn correct(&self, theta: &Tensor<f64>, r_sim: &RewardBreakdown) -> RewardBreakdown {
        let f_residual  = self.force_gp.posterior_mean(theta);
        let p_residual  = self.pressure_gp.posterior_mean(theta);
        let d_residuals = self.diffusion_gp.iter().map(|gp| gp.posterior_mean(theta)).collect();
        let s_residual  = self.stiffness_gp.posterior_mean(theta);
        r_sim.apply_residuals(f_residual, p_residual, d_residuals, s_residual)
    }
}
```

The `MeasurementProtocol` tracks which of the four signals was collected per print — a given batch might have force curves on all 8 designs but pressure maps on only 3 — and the residual GPs train only on the signals they received.

## 3. Online updating is how the printer's drift stays in the model

Real printers drift. Resin batches change supplier or age; the build chamber's temperature regulation wanders by a few degrees over months; the post-cure UV lamps lose intensity. The residual GP's mean function is not stationary — it slowly moves with the printer's state. `sim-soft`'s online-update policy tracks this:

- **Per-print incremental update.** Each new measured print appends one row to the residual GP's training data and triggers a posterior update. At $n_\text{real} \lesssim 100$ prints total, the update is a rank-1 Cholesky update (cheap). At larger $n_\text{real}$, the GP matrix becomes ill-conditioned and the update is re-computed from scratch every ~20 prints.
- **Change-detection on hyperparameters.** Periodically (every ~20 prints), re-fit the kernel hyperparameters via marginal-likelihood maximization and compare to the prior fit. A significant shift in a length-scale or the signal variance indicates the bias structure has changed, and older training data should be *down-weighted* (not deleted — the drift is gradual, old data is still partially informative).
- **Batch re-fit on explicit drift.** When the designer changes printer or resin batch, drop old training data and re-calibrate from a fresh sequence of ≈10 anchor prints. The anchor-print schedule is built into [Ch 06's full loop](06-full-loop.md).

The online-updating machinery lives in `sim-opt` (not `sim-soft`), consuming the per-print measurements through a well-defined `MeasurementReport` type. The boundary keeps `sim-soft`'s simulation-side logic free of printer-state concerns; only the optimization layer touches it.

## 4. Diffusion-profile correction closes the visual half of the thesis

The [Part 9 Ch 00 SSS chapter](../90-visual/00-sss.md) committed to a measured-diffusion-profile-per-material approach: the `MaterialField` ships a `DiffusionProfile` per material, fit to integrating-sphere data. This chapter closes that loop. After a print, the diffusion profile of the *actual printed coupon* is re-measured, compared to the MaterialField's profile used to render it, and the residual feeds the `diffusion_gp`.

Practically, this means: the designer renders a part, prints it, measures the real part's scattering on a coupon from the same print, and the next render of the same material is *calibrated to the specific print batch*. Two silicone parts from different print batches will render slightly differently even under the same `MaterialField`, because the calibration loop has separated the material's nominal scattering profile from its per-batch realization.

This is the feature that makes a designer stop treating the rendered preview as "what the simulator thinks it'll look like" and start treating it as "what the next print will actually look like." The distinction closes the visual half of the thesis's physically-correct-and-visually-great commitment. Without this correction, the visual preview is decorative; with it, it is load-bearing.

## What this commits downstream

- **[Ch 06 full loop](06-full-loop.md)** closes around the residual GP: every physical print is an observation, every observation updates the GP, every subsequent sim evaluation is corrected by the GP. The loop is not optional, and it is explicitly post-Phase-I — outside the Phase A–I roadmap because it requires a specific printer, sensors, and measurement protocol; [Part 12 Ch 07](../120-roadmap/07-open-questions.md) names it as the roadmap's highest-priority post-Phase-I deliverable.
- **[Part 9 Ch 00 SSS](../90-visual/00-sss.md)**'s measured-diffusion-profile commitment is cashed out by the `diffusion_gp`. The rendered preview's calibration level is the designer's `MeasurementProtocol` setting: no correction, batch-level correction, or per-print-coupon correction.
- **[Ch 04 active learning](04-active-learning.md)** chooses prints to maximally reduce the residual GP's posterior uncertainty, which is how the sample-efficient side of this loop gets paid for.
- **[Part 11 Ch 02 ml-chassis coupling](../110-crate/02-coupling/03-ml-chassis.md)** extends the `Surrogate` trait to include residual-style surrogates; the trait surface grows to accommodate per-component output types.
- **Appendix material database** grows to include per-material reference diffusion profiles measured from reference prints. The appendix is the calibration anchor that a designer with a new printer compares against.

## What this does NOT commit

- **Automatic sensor integration.** The measurement protocol assumes the designer runs the measurements and enters them (via a `MeasurementReport` JSON or equivalent). `sim-soft` does not ship drivers for force sensors, pressure films, or integrating spheres. Automating that is outside the platform's scope.
- **Cross-printer transfer learning.** The residual GP is tied to a specific printer configuration; transferring the learned bias from one printer to a structurally different one is research-frontier work. Not committed.
- **Adversarial / worst-case bias bounds.** The GP gives probabilistic bias estimates; it does not give worst-case guarantees. Applications requiring bias certification (medical, aerospace) would need additional machinery not in scope here.
- **Correction of non-smooth bias sources.** A print failure (hole in the wall, unsintered voxel pocket) is not a smooth bias — the residual GP absorbs it as noise, and the design flagged as failing stays flagged. Detection of structural-print-failures is upstream of this subsystem.
