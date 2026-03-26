# Cone-Limit Orbit — Sliding Along the Constraint Surface

A pendulum on a ball joint with a 45° cone limit, given initial angular
velocity so the tip mass **orbits along the inside of the cone boundary**.
A fading trail shows the circular path on the constraint surface — proving
the cone limit is a true 3D surface, not just a 1D angular limit.

See also: [Cone Limit](../cone-limit/) — planar constraint enforcement,
[Conical Pendulum](../conical-pendulum/) — free orbit without constraints.

## What you see

- **Orange tip sphere** — the 1 kg mass sliding along the cone boundary
- **Fading orange trail** — 5 seconds of trajectory history, showing the
  circular orbit that slowly spirals inward as damping dissipates energy
- **Steel rod** — rigid link from pivot to mass
- **Dark socket** — the ball joint at the pivot
- **Support frame** — horizontal beam with vertical posts

## Physics

The mass starts tilted exactly to the 45° cone limit with the angular
velocity needed for a conical orbit at that angle. The constraint solver
keeps the mass on (or just inside) the cone surface while allowing
tangential motion — friction-free constraint sliding.

With very light damping (0.002 Ns/m), the orbit persists for 20+ seconds
but slowly spirals inward as energy is dissipated.

```
ω = √(g / (L·cos(θ))) = √(9.81 / (0.6·cos(45°))) = 4.809 rad/s
```

| Parameter | Value |
|-----------|-------|
| Mass | 1.0 kg |
| Rod length | 0.6 m |
| Cone limit | 45° |
| Damping | 0.002 Ns/m |
| Integrator | RK4 |
| Initial tilt | 45° (at cone boundary) |
| Initial ω | 4.809 rad/s about vertical |

## Validation

The example runs three automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Quat norm** | Unit quaternion preserved | < 1e-10 deviation |
| **Energy** | Monotonically decreasing | < 1e-3 J increase |
| **Cone limit** | Angle within 45° + solver margin | < 0.02 rad violation |

## Run

```
cargo run -p example-ball-joint-cone-orbit --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
