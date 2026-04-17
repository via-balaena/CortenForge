# Viscoelastic spectrum

Dragon Skin's viscoelastic response is the close cousin of [Ecoflex's](../00-ecoflex/02-viscoelastic.md) — same chemistry family, same storage-and-loss-modulus structure, same Prony-series working form — but with one important difference: the loss factor is modestly higher, and the harder grades dissipate more per cycle than the softer ones. This is why Dragon Skin is the working material for high-cycle drive regimes and why the wobble-failure closure has a different margin here than in the Ecoflex family.

## Measured spectrum

The measured DMA response for Dragon Skin across the three grades, over the same $10^{-2}$ Hz to $10^2$ Hz window the [Ecoflex viscoelastic sibling leaf](../00-ecoflex/02-viscoelastic.md) covers, has the same overall shape as Ecoflex's — nearly flat storage modulus, loss modulus roughly an order of magnitude below storage, Prony weights spread across multiple decades of relaxation time — with two differences worth naming.

- **The loss factor is modestly higher.** $\tan \delta = G''/G'$ sits in a band slightly above Ecoflex's — the harder platinum-cure chemistry dissipates somewhat more per cycle than the softer formulations — though it remains well below natural-rubber levels. Under cyclic loading this is a feature, not a cost: the extra damping closes the [wobble failure](../../02-what-goes-wrong/01-wobble.md) with tighter margin than Ecoflex does at matched integrator damping.
- **Per-grade loss-factor drift matters.** Unlike Ecoflex where the relaxation-spectrum shape transferred within measurement noise across the grade ladder, Dragon Skin shows a small but systematic trend: harder grades have modestly higher loss factors at the same frequency. The trend is small enough that a single Prony fit on 20A transfers to 10A and 30A within working-fit noise, but large enough that per-grade DMA refinement at Pass 3 will change the $g_i$ weights by more than the measurement noise.

## The Prony fit

The `Material<Viscoelastic<NeoHookean>>` and `Material<Viscoelastic<MooneyRivlin>>` wrappers the book ships for Dragon Skin use an $N = 3$ Prony series on 20A as the canonical working fit, chosen over 10A or 30A because 20A is the canonical-problem default grade and because 20A sits at the midpoint of the loss-factor drift across the ladder.

The three relaxation times cover the same band the [Ecoflex Prony fit](../00-ecoflex/02-viscoelastic.md) covers — $\tau_1$ in the 0.1 s neighbourhood, $\tau_2$ in the 1 s neighbourhood, $\tau_3$ in the 10 s neighbourhood — with weights sized to match Dragon Skin's slightly larger total relaxation fraction. The specific coefficient set lands in the [material database](../../../appendices/02-material-db.md) at Pass 3 once the Dragon Skin DMA measurement pass is anchored.

The same two structural statements from the [Ecoflex Prony-fit section](../00-ecoflex/02-viscoelastic.md) apply verbatim to Dragon Skin: the equilibrium modulus $G_{\infty}$ is fixed at the grade's small-strain $\mu$ (not re-fit), making the viscoelastic wrapper a decorator on the elastic core in the [Part 2 Ch 07](../../../20-materials/07-viscoelastic.md) trait-composition pattern; and $N = 3$ is the floor-not-ceiling, with $N = 4$ reachable as a Pass-3 upgrade if a specific application's loading spectrum demands it.

## Per-grade transfer

The Dragon Skin per-grade transfer story differs from Ecoflex's in one respect: the loss-factor trend across the ladder introduces a systematic bias a single-grade fit does not capture. Shipping a 20A-calibrated Prony fit for all three grades is the working approximation; the expected refinement at Pass-3 per-grade DMA is a shift in the total relaxation fraction (up for 30A, down for 10A) of roughly the same magnitude as the 20A-fit's measurement noise.

This is different from Ecoflex's "same relaxation-spectrum shape, different equilibrium modulus" transfer pattern: Dragon Skin's spectrum shape itself drifts across the ladder. In practice the [Part 10 Ch 05 sim-to-real loop](../../../100-optimization/05-sim-to-real.md) absorbs both drifts (modulus and relaxation-fraction) and converges to the per-print calibration; the working-fit-on-20A approximation is what the book ships at Pass 1 before the full measurement deliverable lands.

## How this closes the wobble failure

The wobble-failure mechanism is the same as [Ecoflex's](../00-ecoflex/02-viscoelastic.md): each Prony branch's history-strain contribution damps mesh oscillations at a timescale set by the branch's relaxation time, and $N = 3$ branches spanning 0.1 to 10 s cover the natural-frequency band of the canonical-problem mesh.

What changes for Dragon Skin is the margin. The modestly higher loss factor means the amplitude decay under Dragon Skin is faster per cycle than under Ecoflex at matched modulus, which makes Dragon Skin a more forgiving material for cycling-regime regression tests. The [wobble-failure regression test](../../../110-crate/04-testing/01-regression.md) uses separate pass envelopes for Ecoflex and Dragon Skin models — same shape, different decay-rate thresholds — which the regression-test harness picks up from the `Material`'s validity-metadata dissipation field.

## Autograd and cycle-count-to-failure

The Prony evolution is smooth in the history state, so the gradient through a Dragon Skin viscoelastic step is a straight application of the [Part 6 Ch 01 custom-VJP surface](../../../60-differentiability/01-custom-vjps.md). The [stochastic-adjoint deferral to Phase H](../../../60-differentiability/03-time-adjoint/02-stochastic.md) applies to topology-change and adaptive-timestep-shrink events, not Prony evolution — the viscoelastic wrapper adds no FD-wrapper entry.

One additional consideration for Dragon Skin specifically: the [mechanical-data sibling leaf](00-mechanical.md) names cycle-count-to-failure as a Pass-3 measurement column distinct from the single-cycle elongation-at-break. The viscoelastic dissipation per cycle is what drives the cycle-count-to-failure curve — at matched elastic-strain amplitude, higher dissipation accumulates more stored damage per cycle — which means the Prony coefficients the viscoelastic leaf ships are also inputs to the cycle-count-to-failure prediction in the `Material`'s validity-metadata cycle-life field. The data pipeline is the same as the viscoelastic-decorator's pipeline; the cycle-life annotation is one additional field on the per-grade `Material` row.

## Alternatives considered

**Ship a single Prony fit across the whole Smooth-On platinum-cure family.** Use the Ecoflex 00-30 fit for both Ecoflex and Dragon Skin. Rejected because the loss-factor drift across families is larger than the drift within either family — Dragon Skin's systematically higher dissipation is a family-level difference that the Ecoflex fit does not capture. The book ships a separate Dragon Skin Prony fit (calibrated on 20A) precisely because the family-level shift is the largest drift in the viscoelastic data.

**Defer viscoelasticity entirely until per-grade DMA is measured in-house.** Rejected because the [Part 1 Ch 02 wobble-failure](../../02-what-goes-wrong/01-wobble.md) regression test requires a working Prony fit at the default canonical-problem scale, and shipping an elastic-only Dragon Skin baseline would fail that test at Phase B. The single-20A-fit working approximation with explicit transfer caveats is more honest than elastic-only with an "eventually" comment.

**Ship per-grade independent Prony fits from day one.** Rejected at Pass 1 for the same budget reason the [Ecoflex viscoelastic leaf](../00-ecoflex/02-viscoelastic.md) states: a Pass-1 full-per-grade Dragon Skin DMA dataset does not exist in the literature; the Pass-3 measurement is the right slot for it.
