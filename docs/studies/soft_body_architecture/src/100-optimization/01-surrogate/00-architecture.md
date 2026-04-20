# Neural surrogate architecture

The surrogate family `sim-soft` commits to for the $d \gtrsim 100$, $n \gtrsim 10^4$ regime is an MLP with small residual blocks, wrapped in a deep ensemble for uncertainty. [The parent spine](../01-surrogate.md) names this family; this leaf names the input featurization, the hidden-block shape, the output head, and the ensemble wrapper — four choices that together make the surrogate a drop-in for the acquisition-function and active-learning inner loops the parent justifies.

## Input: the design vector, not a spatial field

The surrogate ingests $\theta \in \mathbb{R}^d$ directly — the flattened design-parameter vector enumerated in [Ch 00's §What θ contains](../00-forward.md). This is deliberate:

- The upstream representations (`cf-design`'s SDF composition tree, the `MaterialField` spatial distribution) are *already compressed* by `cf-design` into the finite-dimensional $\theta$ before the forward map runs. Re-ingesting the `SdfField` or `MaterialField` trait objects would expand a representation `cf-design` deliberately finitized, and would break the boundary Ch 00's forward-map contract draws between `cf-design` and `sim-soft`.
- Typical $d \in [10, 50]$ per [Ch 00](../00-forward.md), with high-dimensional configurations reaching $\sim 200$. An MLP over a vector of that size is compact to train and query at moderate ensemble sizes; a mesh GNN or SDF-grid CNN would be larger, slower, and would have to contend with the mesh-topology transitions the design space induces — exactly the discontinuity [Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md) absorbs by routing topology-crossing edits through an FD wrapper.
- The surrogate's job is **amortize evaluations of a specific scalar function over a specific parameter space**, not generalize across geometries. A mesh-level representation would be sized for the generalization task; MLP-on-$\theta$ is appropriately sized for the amortization task.

Featurization: raw $\theta$ components are normalized to unit variance on the training set. Components corresponding to bounded parameters (radii with physical upper bounds, weights $w_i \in [0, 1]$) are clipped to the known bound before the normalizing transform. No further engineering — the network is expected to learn bilinear and higher-order interactions internally through its hidden layers.

## Hidden: narrow MLP with residual blocks

Body: a small stack of residual blocks, each block being two fully-connected layers with a smooth ($C^2$) activation and a residual identity shortcut. The smoothness requirement is not decorative — [§01's gradient-matching loss](01-training.md) contains $\nabla_\theta \widehat R$ as one of its terms, and training updates $\phi$ by backpropagating through that gradient, which requires the activation's second derivative $\sigma''$ to exist. ReLU-family activations fail this at the piecewise boundary; $C^2$ smoothness on the activation is the structural requirement, not a stylistic preference.

Block count and hidden width are hyperparameters tuned per canonical problem. The expected scale is small because the function being learned is smooth within a topology per [Ch 00's reward-surface properties](../00-forward.md), and because training-sample counts are dominated by the real-evaluation budget rather than by compute. A large surrogate fit to a small sample set overfits without adding capacity that matters to the downstream acquisition functions.

Residual identity shortcuts (rather than a plain MLP) also ground the gradient-supervision signal: at initialization the gradient of the network with respect to its input is not vanishing, because the shortcut contributes the identity. This keeps the gradient term in the [§01](01-training.md) loss numerically comparable to the value term from the first training step, rather than needing many steps of warm-up before the gradient supervision becomes informative.

## Output: scalar reward with optional decomposition

The default output head is a single scalar — the predicted reward $\widehat R(\theta)$. An optional multi-head variant predicts the four [Part 1 Ch 01 reward terms](../../10-physical/01-reward.md) (pressure uniformity, contact coverage, peak-pressure bound, effective-stiffness bound) separately, and composes them using the same weights $w_i$ the forward map uses.

The multi-head variant is preferred when the weights $w_i$ are themselves part of $\theta$:

- Training a single-scalar head while the weights are in $\theta$ forces the network to learn the composition internally. The composition is a known linear form; spending capacity on rediscovering it is wasted.
- Training a multi-head head and composing externally factors the known linearity out of what the network has to learn; only the four upstream terms — which are nonlinear functions of the rest of $\theta$ — go through the network.

[Ch 03 preference learning](../03-preference.md) writes over the weights $w_i$ from user rating data. A multi-head surrogate lets preference learning re-predict reward under new weights without retraining — the weights multiply the already-trained multi-head output. A scalar-head surrogate would have to retrain whenever the weights move.

## The ensemble wrapper

The wrapper is an ensemble of $M$ identically-architected networks trained from independent random initializations on the same training set, per [Lakshminarayanan, Pritzel & Blundell 2017](https://arxiv.org/abs/1612.01474). The ensemble's point prediction is the mean over the $M$ members; its predictive variance is the empirical variance across members.

Typical $M$ in the literature is small (single-digit), consistent with the experimental setup in that paper. There is no shared feature extractor, no cross-member communication, no distillation — $M$ independent networks trained and queried through the same code path.

```rust
use sim_ml_chassis::Tensor;

pub struct NeuralSurrogate {
    members: Vec<MlpResidual>,      // M networks, identical architecture, different init seeds
    input_norm: InputNormalizer,    // fit once at train time, shared across members
}

pub struct SurrogatePrediction {
    pub mean: Tensor<f64>,      // scalar (default head) or 4-vector (multi-head variant)
    pub variance: Tensor<f64>,  // empirical across members, same shape as mean
}

impl NeuralSurrogate {
    /// Prediction: mean + empirical variance across M members.
    pub fn predict(&self, theta: &Tensor<f64>) -> SurrogatePrediction {
        let normed = self.input_norm.apply(theta);
        let preds: Vec<Tensor<f64>> = self.members.iter()
            .map(|m| m.forward(&normed))
            .collect();
        /* mean + empirical variance assembled from preds */
    }
}
```

Deep ensembles rather than MC-Dropout (Gal & Ghahramani 2016) for the reasons [§02](02-uncertainty.md) develops in detail; the architectural consequence is that the wrapper is a thin container around $M$ independent networks, each trained and queried by the same code path as any single-network variant.

## What this sub-leaf commits the book to

- **Input is $\theta$ directly.** The surrogate does not re-ingest `SdfField` or `MaterialField`; the parameter-space compression `cf-design` already performs is sufficient.
- **Body is an MLP with small residual blocks and a $C^2$ activation.** The second-derivative requirement is load-bearing for [§01](01-training.md)'s gradient-matching loss, not stylistic; block count and hidden width are Phase-B hyperparameter choices.
- **Output is scalar reward, with an optional multi-head variant** over the four Part 1 Ch 01 reward terms composed externally using the weights $w_i \in \theta$.
- **Ensemble of $M$ independent networks** (Lakshminarayanan et al. 2017), typical $M$ single-digit, no weight-sharing between members.
- **`NeuralSurrogate::predict` returns mean + empirical variance across members.** [§02](02-uncertainty.md) interprets the variance; [Ch 02 BayesOpt](../02-bayesopt.md) and [Ch 04 active learning](../04-active-learning.md) consume it as the acquisition functions' posterior-variance input.
