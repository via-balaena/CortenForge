# Muscles — Activation Dynamics and Force Generation

Five examples covering the muscle actuator system: activation dynamics,
force-length curves, force-velocity curves, and multi-muscle coordination.

Two muscle models exist in the engine:

- **MuJoCo `<muscle>`** — piecewise-quadratic FL/FV/FP curves, conformant
  with MuJoCo 3.5.0. Handles joint-transmission length normalization via
  `lengthrange`. Used by all visual examples.
- **CortenForge `HillMuscle`** — Gaussian FL, Hill hyperbolic FV,
  exponential FP, with pennation angle. Extension beyond MuJoCo. Validated
  in the stress test.

Both share identical activation dynamics (Millard et al. 2013) with
activation-dependent time constants and asymmetric rise/fall behavior.

## Examples

| Example | What it demonstrates |
|---------|---------------------|
| [stress-test](stress-test/) | 51 headless checks: curve pinning, pipeline force, dynamics |
| [forearm-flexion](forearm-flexion/) | Single muscle lifts forearm against gravity, flex/release cycle |
| [activation](activation/) | Three time constants — arms contract together, release at different rates |
| [cocontraction](cocontraction/) | Agonist-antagonist pair — joint stiffening vs single-muscle movement |
| [force-length](force-length/) | Wide vs narrow FL range — same motion, different force output |

## Visual conventions

- **Tendon color** — blue at rest, red under tension (driven by actual
  force magnitude, not activation level)
- **3D tendon mesh** — cylinder stretched between body-derived attachment
  points, updated each frame from FK transforms
- **HUD** — activation, force, angle, velocity displayed in real time

## Key physics

**Activation asymmetry:** Muscle activation (ctrl → force) is fast, but
deactivation is slow. This is the Millard et al. (2013) model:
`tau_deact_eff = tau_deact / (0.5 + 1.5 * act)`. The effective deactivation
time constant grows 4x as activation drops, creating a long tail. See the
[activation](activation/) example.

**Force-length relationship:** Muscles produce peak force at their optimal
length and weaken at shorter or longer lengths. The width of this curve
(controlled by `lmin`/`lmax`) determines the operating range. See the
[force-length](force-length/) example.

**Cocontraction:** Muscles can only pull. To control both position and
stiffness, opposing muscles activate simultaneously. Net torque is near
zero but the joint becomes stiff. See the [cocontraction](cocontraction/)
example.
