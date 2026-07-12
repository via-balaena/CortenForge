# Constants, formulas, and references

## Constants

| Symbol | Value | Meaning |
|---|---|---|
| $G$ | $6.674\times10^{-11}\ \mathrm{m^3\,kg^{-1}\,s^{-2}}$ | gravitational constant |
| $k_B$ | $1.381\times10^{-23}\ \mathrm{J/K}$ | Boltzmann constant |
| $T$ | $293\ \mathrm{K}$ | room temperature |
| $g$ | $9.81\ \mathrm{m/s^2}$ | standard gravity |
| $1\ \mathrm{E}$ | $10^{-9}\ \mathrm{s^{-2}}$ | one Eötvös |
| $\rho_\text{rock}$ | $2500\ \mathrm{kg/m^3}$ | typical rock density (for void deficits) |
| $f_0$ | $\approx 1\ \mathrm{kHz}$ | assumed proof-mass resonance in the noise table (spike) |

## Formulas used in the feasibility spike

**Gravity gradient of a compact mass** (dominant component):
$$\Gamma \approx \frac{2 G m}{r^{3}}$$

**Void anomaly mass** (deficit of an air-filled sphere of radius $R$ in rock):
$$m = \rho_\text{rock}\,\tfrac{4}{3}\pi R^3$$

**Thermomechanical acceleration noise** of a damped MEMS proof mass:
$$a_n = \sqrt{\frac{4 k_B T\,\omega_0}{m\,Q}}, \qquad \omega_0 = 2\pi f_0$$

**Gradient noise floor** for two independent proof masses on baseline $L$, dwell time $\tau$:
$$\sigma_\Gamma = \frac{a_n}{L\,\sqrt{\tau}}$$

**Required channel match** to reject a common-mode acceleration $a_\text{cm}$ beneath a
gradient signal $\Gamma_\text{sig}$ on baseline $L$:
$$\varepsilon < \frac{\Gamma_\text{sig}\,L}{a_\text{cm}}, \qquad \text{CMRR} = 1/\varepsilon$$

## On the feasibility spike

The Chapter 1 figures were produced by a standalone, throwaway calculation (first-principles
Newtonian gravity + the thermomechanical noise model above), run before this study was drafted
so that the architecture would rest on measured facts rather than optimism. It is a *feasibility*
estimate: point-mass gravity, an idealized single-mode resonator, and literature-typical MEMS
parameters. It deliberately does **not** model the real FEM behavior, dissipation ($Q$)
mechanisms, or manufacturing tolerances — those are the subject of the build program
([Chapter 4](40-program.md)), not the feasibility gate. It is also **mechanical-only**: it omits
readout noise, which adds in quadrature and can dominate at room temperature — so the quoted floor
is optimistic, and the Rung 4 budget must close that gap before any floor is asserted as real.

## SDK symbols quick-reference (for the implementer)

The primitives the ladder stands on or extends, with where to find them:

| Symbol | Location | What it gives the gradiometer |
|---|---|---|
| `Sdf`, `Solid` | `cf-geometry`, `cf-design` | parametric proof-mass + housing geometry |
| `mass_properties(solid, density, cell_size)` | `cf-design` `mechanism/mass.rs` | mass, COM, inertia by grid-integrating an implicit field — the ∇g model reuses this voxel walk |
| SDF gradient / params | `cf-design` `gradient.rs`, `param.rs` | geometry parameters carried differentiably |
| `CoDesignProblem`, `optimize(problem, x0, cfg)`, `OptConfig`, `Normalized` | `cf-codesign` `src/lib.rs` | the co-design loop the objective plugs into |
| forward dynamics + accel sensors | `sim-core` | the differential / platform-motion framing |
| structural FEM (stiffness/mass assembly) | `sim-soft` | substrate for a *future* modal analysis ($f_0$) — the eigenproblem itself is not built yet |

**Missing (to build):** the ∇g forward model (Rung 1); modal/eigenfrequency analysis with an
anisotropic-Si material path, and the $Q$/dissipation + readout-noise budget (Rung 4, the hard
part). **Out of scope:** the *engines* behind the inputs — etch-process physics, circuit synthesis,
DAQ firmware — not the inputs themselves. Anisotropic Si, fab-geometry tolerances, and readout noise
all enter as differentiable terms; see [Chapter 3](30-primitives.md#where-the-scope-line-actually-falls).

## Glossary

- **Eötvös (E)** — unit of gravity gradient, $10^{-9}\ \mathrm{s^{-2}}$.
- **Common-mode rejection (CMRR)** — the factor by which a matched pair cancels acceleration
  common to both proof masses (platform vibration, tilt, the DC of $g$); $= 1/\varepsilon$.
- **$\varepsilon$ (channel match)** — fractional scale-factor mismatch between the two channels;
  the objective co-design drives toward zero.
- **Self-gravity** — the gradient the instrument's *own* mass produces at its proof masses
  (~1000 E); a DC offset + thermal drift that must be modeled from CAD and subtracted.
- **$Q$ (quality factor)** — inverse mechanical loss; sets the thermomechanical noise floor via
  fluctuation-dissipation. The genuinely hard modelling.
- **The niche** — large buried anomalies (tunnels, voids, chambers, sizable caches) at a few
  metres' standoff, surveyed still-and-dwell. *Not* person-borne threat detection.

## Key numbers to remember

- Signal: tunnel/chamber/big-cache = **1–100 E**; person-borne = sub-0.01 E (out); a human body =
  ~10 E of clutter.
- Noise: nominal room-temp MEMS reaches **~0.6–2 E** with 100–1000 s dwell (contingent on $Q$).
- Matching: a moving platform needs **sub-ppm** ($\varepsilon \lesssim 10^{-6}$); hand-matched is
  $10^{-3}$–$10^{-4}$.
- Self-gravity: **~1000 E** from the instrument's own housing.

## References and prior CortenForge notes

- The dialed-in target (room-temperature MEMS / electromechanical tier), the tier-cost argument
  for why CortenForge's leverage lands only on matching + self-gravity, and the Tier-1/2/3 gap
  map are recorded in the project memory *gravity-gradiometer-direction*.
- Sibling architecture studies: the mesh, soft-body, casting, ML-chassis, and biological-
  navigation books under `docs/studies/`.
- CortenForge mission: `MISSION.md` — "close the loop between a real human body and the device
  that helps it, and keep that loop differentiable end to end." Gradiometry is a versatility
  expansion of the same differentiable-mechanical-co-design core.
