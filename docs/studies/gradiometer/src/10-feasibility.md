# The physics gates: can this work, and for what?

Before any architecture, one question has to be answered with numbers: **can a
room-temperature MEMS gravity gradiometer detect anything useful, and if so, what?**
If the physics says no, the rest of this book is moot. So this chapter leads with what
could kill the idea.

Every figure below comes from a **feasibility spike** — a short, throwaway calculation
run before this book was written, using only first-principles Newtonian gravity, the
standard thermomechanical noise model, and textbook constants. The formulas and constants
are collected in the [appendix](appendices.md); the intent is that any reader can reproduce
the numbers.

Gravity gradients are quoted in **Eötvös**: $1\ \mathrm{E} = 10^{-9}\ \mathrm{s^{-2}}$.

## Gate 1 — Is there a signal?

A compact mass anomaly at distance $r$ produces a gravity-gradient signal whose dominant
component scales as

$$\Gamma \approx \frac{2 G m}{r^{3}},$$

where $G = 6.674\times10^{-11}\ \mathrm{m^3\,kg^{-1}\,s^{-2}}$ and $m$ is the anomalous
mass (for a void, the *deficit* $m = \rho_\text{rock}\,\tfrac{4}{3}\pi R^3$ with
$\rho_\text{rock}\approx 2500\ \mathrm{kg/m^3}$). The $1/r^3$ falloff is the crux: gradiometry
is a **near-field, large-mass** instrument.

| Anomaly | mass (kg) | @ 1 m | @ 3 m | @ 5 m | @ 10 m |
|---|---:|---:|---:|---:|---:|
| 1 m-radius void (tunnel) | 10 472 | 1400 | 51.8 | 11.2 | 1.4 |
| 2 m-radius void (chamber) | 83 776 | 11 200 | 414 | 89.5 | 11.2 |
| 500 kg dense cache | 500 | 66.7 | 2.47 | 0.53 | 0.07 |
| 100 kg cache | 100 | 13.3 | 0.49 | 0.11 | 0.01 |
| 70 kg person *(clutter)* | 70 | 9.3 | 0.35 | 0.07 | 0.01 |
| 1 kg pistol *(person-borne)* | 1 | 0.13 | 0.005 | 0.001 | 0.0001 |

*(values in Eötvös)*

Two facts fall straight out of this table, and they define the entire product:

- **The honest niche is large, buried anomalies at a few metres' standoff** — tunnels,
  voids, hidden rooms, sizable caches — which produce **1–100 E**.
- **Person-borne threat detection is not physical.** A 1 kg pistol is well below
  0.01 E at any useful range, and a human body is itself **~10 E of clutter** — the target
  is smaller than the people standing next to it. Concealed weapons belong to millimetre-wave,
  terahertz, and trace-chemical sensors, not gravity. This instrument finds the *hidden room*,
  not the gun in a pocket.

## Gate 2 — Is the signal above the noise?

A MEMS proof mass is fundamentally limited by **thermomechanical (Brownian) noise** — the
fluctuation-dissipation floor of a damped resonator. Its acceleration noise density is

$$a_n = \sqrt{\frac{4 k_B T\,\omega_0}{m\,Q}}\quad\left[\mathrm{m\,s^{-2}/\sqrt{Hz}}\right],
\qquad \omega_0 = 2\pi f_0,$$

with Boltzmann constant $k_B$, temperature $T = 293\ \mathrm{K}$, proof mass $m$, resonant
frequency $f_0$, and quality factor $Q$. Two independent proof masses on a baseline $L$,
averaged for a dwell time $\tau$, give a gradient noise of

$$\sigma_\Gamma = \frac{a_n}{L\,\sqrt{\tau}}.$$

Proof mass $m$, resonance $f_0$, and baseline $L$ are design levers the geometry sets; $Q$ — the
mechanical dissipation — is physics that must be *modelled* and cannot be hand-waved, which makes
it the least-controllable driver of the floor. This table brackets a pessimistic, nominal, and
optimistic device (all at a proof-mass resonance $f_0 \approx 1\ \mathrm{kHz}$; the floor scales
as $\sqrt{f_0}$):

| Device | $a_n$ (nano-$g$/√Hz) | @ 1 s | @ 100 s | @ 1000 s |
|---|---:|---:|---:|---:|
| pessimistic — 10 mg, $Q{=}10^3$, $L{=}0.1$ m | 10.3 | 1008 | 101 | 32 |
| **nominal — 100 mg, $Q{=}10^4$, $L{=}0.5$ m** | 1.0 | 20 | **2.0** | **0.64** |
| optimistic — 1 g, $Q{=}10^5$, $L{=}1.0$ m | 0.1 | 1.0 | 0.10 | 0.03 |

*(gradient floor in Eötvös, at the given dwell time; $f_0 \approx 1\ \mathrm{kHz}$)*

**The nominal device reaches ~2 E at 100 s and ~0.6 E at 1000 s of dwell** — comfortably
below the 1–100 E signals of the tunnel/chamber niche. The pessimistic device is marginal;
the optimistic one reaches into the sub-0.1 E regime. The verdict is a **conditional GO**:
the physics supports a useful room-temperature MEMS gradiometer for the large-anomaly niche,
operated *still* with dwell time — walk, plant, and integrate, exactly as terrestrial
gravimetry has always worked.

Of the four parameters, $Q$ is the one that is modelled physics rather than a design choice, so
it is where the risk concentrates: dissipation — squeeze-film gas damping, thermoelastic loss,
anchor loss, material loss angle — is what pushes a design toward nominal or pessimistic, and it
cannot be hand-waved. That is flagged as the hard Tier-2 gate in
[Chapter 4](40-program.md#gate-2--the-q--dissipation-floor---the-genuinely-hard-part).

One honesty caveat on the numbers above: this is a **thermomechanical-only** floor. The real
instrument adds **readout noise** (capacitive pickoff + amplifier) in quadrature, and for room-temp
capacitive MEMS that term is frequently *dominant* — so the quoted ~0.6–2 E is optimistic until the
Rung 4 budget carries the readout term. Readout noise is in-budget by design (a differentiable
referred-input model, not circuit design); see [Chapter 3](30-primitives.md#where-the-scope-line-actually-falls).

## Gate 3 — What actually makes it hard (and where CortenForge earns its keep)

Passing Gates 1 and 2 says a *bare, static, isolated* proof-mass pair could work. The reason
real gradiometers are expensive is everything between that and a usable instrument — and it is
precisely mechanical.

### Matching, because the platform moves

On any moving platform the proof masses see enormous **common-mode** acceleration — vehicle
vibration, tilt, translation — that must cancel in the difference. It cancels only to the
degree the two channels are **mechanically matched**. For a signal $\Gamma_\text{sig}$ across
baseline $L$, the differential signal is $\Gamma_\text{sig} L$; to bury a common-mode
acceleration $a_\text{cm}$ beneath it, the channel scale-factor match $\varepsilon$ must satisfy
$\varepsilon < \Gamma_\text{sig} L / a_\text{cm}$.

For a 10 E signal on a 0.5 m baseline:

| Platform common-mode | required match $\varepsilon$ | required CMRR |
|---|---:|---:|
| $10^{-1}\,g$ (rough vehicle) | $5\times10^{-9}$ | $2\times10^{8}$ |
| $10^{-2}\,g$ (quiet vehicle) | $5\times10^{-8}$ | $2\times10^{7}$ |
| $10^{-3}\,g$ (well-isolated) | $5\times10^{-7}$ | $2\times10^{6}$ |

Hand-matched proof-mass pairs reach $\varepsilon \sim 10^{-3}$–$10^{-4}$; painstaking lab
trimming reaches $10^{-5}$–$10^{-6}$. **Sub-ppm matching is required to operate on a moving
platform at all** — and matching is a *geometry* problem: two structures whose mechanical
response is identical by construction. This is where matched-by-construction co-design is the
differentiator, and it is why the target is the gradiometer, not the single-proof-mass
gravimeter.

### Self-gravity, because the instrument has mass

The instrument's own structure gravitates. A modest housing mass near a proof mass produces:

| Housing mass @ standoff | self-gravity gradient at the proof mass |
|---|---:|
| 200 g @ 3 cm | ~989 E |
| 50 g @ 2 cm | ~834 E |
| 500 g @ 5 cm | ~534 E |

That is **~100× the signal**, as a DC offset that *drifts* as the housing thermally expands by
micrometres. It cannot be shielded — gravity passes through everything — so it must be
**modeled and subtracted**. Computing it requires the instrument's full mass distribution, which
is exactly what a CAD model plus a mass-property integrator provides, and nulling its
temperature sensitivity is a differentiable-geometry optimization. This is the capability no
budget vendor has, and it is native to CortenForge.

## The verdict, and the one number still unproven

- **Gate 1 (signal):** PASS — 1–100 E for the large-anomaly / void niche; person-borne is out.
- **Gate 2 (noise):** CONDITIONAL PASS — a nominal room-temperature MEMS device reaches
  ~0.6–2 E with dwell; contingent on $Q$/dissipation, the hard part.
- **Gate 3 (what's hard):** the cost lives in **matching** (sub-ppm, a geometry problem) and
  **self-gravity** (~1000 E, a CAD mass-model problem) — both squarely on CortenForge's
  differentiable-co-design strength.

One thing this feasibility spike **does not** prove, and cannot: that a *co-designed* match
actually beats a *hand-matched* pair by enough to matter under real FEM behavior and real
manufacturing tolerances. A toy optimization confirms matching is a smooth, differentiable
objective — a necessary condition — but the load-bearing claim needs the FEM matching primitive
and a tolerance model. That is the program's **first real gate**, carried into
[Chapter 4](40-program.md), not something to assert here.

With feasibility established and honestly bounded, [Chapter 2](20-thesis.md) turns to why this
particular instrument is the one the SDK was, in effect, already built to design.
