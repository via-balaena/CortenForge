# Double Pendulum — Chaotic Hinge Chain

Two rigid links chained by hinge joints, released from a dramatic initial
angle. The system exhibits **deterministic chaos** — extreme sensitivity to
initial conditions makes the motion unpredictable, yet the physics engine
must conserve energy perfectly throughout.

See also: [Simple Pendulum](../simple-pendulum/) — single-link baseline with
period validation.

## What you see

- **Upper link** — 1.0 kg rod with polished steel finish, thicker capsule
- **Lower link** — 0.5 kg rod with brushed metal finish, thinner capsule
- **Blue tip sphere** — the lighter mass at the end of the chain
- **Dark joint balls** — cast iron at both pivot points
- **Metallic bracket** — visual mount at the top

## Physics

Two hinge joints in a kinematic chain — **2 DOF**, each joint a scalar angle.
The system is a textbook example of deterministic chaos: tiny changes in
initial conditions lead to exponentially diverging trajectories.

The equations of motion are coupled nonlinear ODEs with no closed-form
solution. There is **no repeating period** — the motion never exactly repeats.

With 120° + 90° initial angles, the system has 17.9 J of energy — enough for
link 2 to flip over the top (needs only 3.4 J) but not enough for link 1 to
flip (needs 19.6 J). This creates dramatically chaotic motion while keeping
the upper link bounded.

| Parameter | Value |
|-----------|-------|
| Link 1 mass | 1.0 kg |
| Link 1 length | 1.0 m |
| Link 2 mass | 0.5 kg |
| Link 2 length | 0.7 m |
| Damping | 0 (both joints) |
| Integrator | RK4 |
| Initial angles | 120°, 90° |

## Validation

The example runs one automated check at t=30s (longer window to stress the
integrator through more chaotic state space):

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Energy** | Conserved (undamped, no contacts) | < 0.5% drift |

No period check — chaotic systems have no repeating period. Energy conservation
over 30 seconds of chaotic motion is a strong integrator test.

## Run

```
cargo run -p example-hinge-joint-double-pendulum --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
