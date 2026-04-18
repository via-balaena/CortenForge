# When to prefer over neo-Hookean

The [two-parameter sibling](00-two-param.md) put the closed-form Mooney-Rivlin law in place; the [calibration sibling](01-calibration.md) stated what data it takes to fit. This leaf answers the decision question the [Mooney-Rivlin branch parent](../01-mooney-rivlin.md) opened — when to pay the extra parameter's cost over [neo-Hookean](../00-neo-hookean.md), and when to skip straight to [Ogden](../02-ogden.md) instead of picking Mooney-Rivlin at all.

## Three axes, one decision

The choice between the three hyperelastic laws is made on three axes — strain regime, calibration-data availability, and evaluation-cost budget — and the answer often is simply "neo-Hookean" even when the test case has some features that look like they would argue for Mooney-Rivlin.

**Strain regime.** The canonical problem's contact zones span a wide stretch distribution: the indentation zone sees compression down to $\lambda \approx 0.7$, the sleeve rim sees tension up to $\lambda \approx 2$ on Ecoflex 00-30 under a rounded probe, and the bulk sits near $\lambda \approx 1$. The laws have overlapping but distinct regimes of fidelity:

- Below ≈50% stretch deviation from the reference, neo-Hookean and Mooney-Rivlin are indistinguishable within typical measurement noise. $I_1$ and $I_2$ are close enough to collinear in this regime that the second parameter carries no extra information on most silicone data.
- Between ≈50% and ≈150% stretch, the neo-Hookean fit begins to drift. The $I_1$-only form saturates in the moderate-stretch regime where published silicone curves have a slight inflection that $I_2$ sensitivity captures; Mooney-Rivlin tracks the curve, neo-Hookean produces a few percent aggregate error on stress and visible drift on rim-conformity rewards.
- Above ≈150% stretch, even Mooney-Rivlin saturates against the strain-hardening-then-failure shape silicone shows near the elongation limit. The non-integer-exponent freedom of Ogden (per the [why-Ogden sibling](../02-ogden/01-why-ogden.md)) is what captures that regime; a $N = 2$ Ogden fit tracks Ecoflex through the hardening inflection, Mooney-Rivlin does not.

**Calibration-data availability.** Mooney-Rivlin is only better than neo-Hookean when it is *calibrated*, and as the [calibration sibling](01-calibration.md) stated, uniaxial-only data is insufficient for a well-conditioned $(C_{10}, C_{01})$ fit. Published silicone data from manufacturer sheets is overwhelmingly uniaxial-only. If no biaxial or planar-shear data is available for the target material, the right call is to fit neo-Hookean rather than ship an ill-conditioned Mooney-Rivlin instance — the one-parameter fit's covariance is narrower than the ill-posed two-parameter fit's, even though the nominal law is less expressive.

**Evaluation-cost budget.** Per Gauss point, Mooney-Rivlin costs one extra Frobenius-norm-squared plus one $3\times 3$ triple product over neo-Hookean (per the [two-parameter sibling's](00-two-param.md) evaluation-path statement). Ogden costs a per-Gauss-point eigen-decomposition plus a non-integer `pow` per principal stretch per term. Sparse-assembly shape is identical across the three — $12 \times 12$ (Tet4) or $30 \times 30$ (Tet10) blocks — so the cost difference lives inside the per-element arithmetic loop, not in the solver. For the [Phase E real-time budget](../../../110-crate/03-build-order.md), a scene where Mooney-Rivlin earns the reward it costs is worth it; one where it does not is pure overhead.

## When Mooney-Rivlin is the right call

The narrow positive case.

- The target material is a silicone whose characterization includes both uniaxial *and* biaxial or planar-shear data. This is a property of the calibration dataset that's available, not of the silicone itself — a material with published dual-protocol data qualifies, one with uniaxial-only data does not, regardless of grade.
- The canonical problem or a specific scene in it sits predominantly in the 50–150% stretch range. A sleeve that's thick enough to sit near its reference configuration through the whole probe insertion sits in the regime where neo-Hookean is already sufficient; a thin-walled sleeve that stretches over a wider-than-reference probe sits in the regime where Mooney-Rivlin's $I_2$ sensitivity moves the rim-conformity reward.
- The scene's total simulation cost budget has headroom for per-Gauss-point arithmetic that doesn't buy Ogden's extra regime coverage. Spending a modest fraction more on per-Gauss-point flops to gain the moderate-stretch fidelity of Mooney-Rivlin is a real tradeoff; spending that fraction to gain nothing (because the scene sits below 50% stretch) is not.

When any of the three fails, the answer is either neo-Hookean (below 50%, or uniaxial-only data, or cost-constrained scene) or Ogden (above 150%, or strain-hardening-sensitive reward, or published data that includes the inflection region).

## When to skip to Ogden instead

Two situations argue for stepping past Mooney-Rivlin rather than picking it.

- **The operating regime crosses 150% stretch.** Both Mooney-Rivlin and neo-Hookean saturate above this threshold for typical silicones; the error past it grows with stretch in a way $I_1, I_2$ polynomials cannot capture at any $(C_{10}, C_{01})$ choice. Going from neo-Hookean to Mooney-Rivlin costs calibration data and runtime arithmetic and still does not cross the regime. Going to Ogden with $N = 2$ or $N = 3$ does, and the published Ecoflex data that extends into the strain-hardening regime is the specific case the [why-Ogden sibling](../02-ogden/01-why-ogden.md) treats.
- **The stress–strain response is asymmetric about the reference configuration.** Ecoflex under contact loading sees tension at the sleeve rim and compression at the indentation zone with measurably different stiffnesses. Mooney-Rivlin is a polynomial in $(I_1, I_2)$ and therefore symmetric in the principal-stretch exchange; it cannot represent compression-vs-tension stiffness asymmetry at any coefficient choice. Ogden's freedom to choose both positive and negative $\alpha_i$ lets a single model capture the asymmetric stiffening on both sides of $\lambda = 1$.

## Per-region vs per-mesh decision

The decision is per-region in a [spatial material field](../../09-spatial-fields.md), not per-mesh. A scene with a stiff Dragon Skin core (moderate stretch, 50–100% regime) and a soft Ecoflex skin (high stretch, 150%+ regime) takes a Mooney-Rivlin region for the core and an Ogden region for the skin — the [trait-surface](../../00-trait-hierarchy/00-trait-surface.md)'s uniform `Material` interface is what lets the solver assemble both without branching. The three laws are not competing defaults for the whole simulation; they are choices per `MaterialField` region.

## What this sub-leaf commits the book to

- **Neo-Hookean remains the default.** A test case sitting below 50% stretch, or with uniaxial-only calibration data, takes neo-Hookean. Mooney-Rivlin is not a drop-in upgrade; switching the law without the data to justify it adds runtime cost and regression risk with no reward movement.
- **Mooney-Rivlin is a narrow fit.** It occupies the moderate-stretch regime with dual-protocol calibration data and a scene where the $I_2$ sensitivity is what closes the reward gap. Outside that fit, either neo-Hookean or Ogden is the better call.
- **Ogden is the answer for strain-hardening and stiffness asymmetry.** Stepping past Mooney-Rivlin rather than stacking on top of it is the right call when the operating regime crosses 150% stretch or when the material shows tension-vs-compression asymmetry. The [Ogden branch](../02-ogden.md) carries its own decision tree for $N$-term selection.
- **The decision is per-`MaterialField` region, not per-mesh.** Multi-material meshes pick the law per region; the solver consumes the trait-generic `Material` interface and assembles heterogeneous hyperelastic regions without branching. This is what makes "the right law per region" tractable at the scene level rather than a global configuration choice.
