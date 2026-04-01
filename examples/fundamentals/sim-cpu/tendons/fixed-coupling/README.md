# Fixed Coupling — Linear Joint Coupling via Tendon

A fixed tendon couples two hinge joints with coefficients `[1.0, −0.5]`.
The tendon length is a linear combination of joint positions:
`L = 1.0 × q_A + (−0.5) × q_B`. A stiff spring on the tendon creates a
restoring force that mechanically links the two joints.

## Concept

A **fixed tendon** does not follow a 3D path — it computes a scalar length
from a weighted sum of joint positions. With nonzero stiffness, the tendon
applies forces proportional to its stretch, creating a coupling force between
joints. The coupling ratio is set by the coefficients: when joint A moves
+1 rad, the tendon pulls joint B by −0.5 rad (opposite sign, half magnitude).

## What you see

- **Blue pendulum (A):** driven by a sinusoidal motor (0.5 Hz, ±2 N·m).
- **Red pendulum (B):** no motor — moves only through the tendon spring force.
- **Tendon line** connects the two tips, colored by force:
  green (relaxed) → yellow (moderate) → red (high stress).
- B follows A at roughly half the amplitude and in the opposite direction.
- The HUD confirms `TendonPos` matches the manual calculation
  `1.0 × q_A − 0.5 × q_B` to machine precision every frame.

## Run

```
cargo run -p example-tendon-fixed-coupling --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
