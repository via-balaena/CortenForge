# Contact Tuning

Contact parameter examples — friction coefficients, contact dimensionality,
solver reference (bounce/stiffness), and explicit pair overrides.

## Examples

| Example | What it shows | Run |
|---------|---------------|-----|
| `friction-slide` | Coulomb friction on tilted plane (mu=0.1, 0.5, 1.0) | `cargo run -p example-contact-friction-slide --release` |
| `condim-compare` | condim=1 vs 3 vs 6 (frictionless, sliding, rolling) | `cargo run -p example-contact-condim-compare --release` |
| `solref-bounce` | Contact stiffness via solref (bouncy vs absorbing) | `cargo run -p example-contact-solref-bounce --release` |
| `pair-override` | Explicit `<pair>` override vs auto-combination | `cargo run -p example-contact-pair-override --release` |
| `solimp-depth` | Impedance curve: soft start vs stiff (penetration depth) | `cargo run -p example-contact-solimp-depth --release` |
| `margin-gap` | Contact activation distance (floating balls) | `cargo run -p example-contact-margin-gap --release` |
| `stress-test` | Headless validation of all contact parameters (26 checks) | `cargo run -p example-contact-stress-test --release` |

## Key Concepts

### Friction

MuJoCo uses a 5-element friction model: `[sliding1, sliding2, torsional, rolling1, rolling2]`.
In MJCF, you specify 3 values: `friction="sliding torsional rolling"`.

When two geoms collide, friction is combined element-wise MAX:
`combined = [max(g1.sliding, g2.sliding), ...]`

The Coulomb friction cone limits tangential force to `mu * normal_force`.

### Contact Dimensionality (condim)

| condim | Rows | What it models |
|--------|------|---------------|
| 1 | 1 | Frictionless (normal only) |
| 3 | 3 | Sliding friction (normal + 2 tangent) |
| 4 | 4 | + Torsional (spin about normal) |
| 6 | 6 | + Rolling (2 rolling directions) |

### Solver Reference (solref)

Controls contact stiffness and damping. Two modes:

**Standard mode** (`solref[0] > 0`): `[timeconst, dampratio]`
- `dampratio = 1.0` = critically damped (no bounce)
- `dampratio < 1.0` = underdamped (bounces)
- Larger `timeconst` = softer contact

**Direct mode** (`solref[0] <= 0`): `[-stiffness, -damping]`
- Same K, different B isolates the damping effect on bounce
- There is no restitution coefficient — bounce comes entirely from impedance.

### Solver Impedance (solimp)

Controls how contact stiffness varies with penetration depth:
`[d0, d_width, width, midpoint, power]`

- `d0` = impedance at zero penetration (low = soft start like foam)
- `d_width` = impedance at full width (endpoint)
- `width` = transition zone in meters

### Margin / Gap

`margin` creates a buffer zone — contact activates before physical
penetration. `gap` subtracts from margin: `includemargin = margin - gap`.

### Pair Overrides

The `<contact><pair>` element overrides auto-combined parameters for a
specific geom pair. Parameter hierarchy: `<pair>` > combination > defaults.
