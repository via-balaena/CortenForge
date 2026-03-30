# Free Flight — Conservation in Isolation

A rigid box floats in empty space with no gravity, no contacts, and no
forces. It was given an initial push (linear velocity) and spin (angular
velocity), then left alone.

## What you should see

The box drifts steadily in one direction while tumbling. The tumbling
pattern looks complex — the angular velocity changes direction in space
because the box has asymmetric inertia (it's longer in one axis than the
others). This is called **torque-free precession**, the same effect that
makes a thrown book wobble mid-air.

Despite the wobbling, three quantities remain exactly constant:

- **Kinetic energy** — the total energy of motion never changes
- **Linear momentum** — the box drifts at constant speed in a straight line
- **Angular momentum** — the total rotational "oomph" is conserved, even
  though the spin axis wanders

These are the three fundamental conservation laws of mechanics. In a
perfect simulation with no external forces, they hold to machine precision
(~1e-17 drift over 10 seconds).

## What to watch for in the HUD

| Field | Expected |
|-------|----------|
| KE | Constant at 0.3428 J |
| KE_drift | ~0 (machine epsilon) |
| \|p\| drift | ~0 |
| \|L\| | Constant at 0.0132 |
| \|L\| drift | ~0 |
| omega_x/y/z | Changing (precession) |

The angular velocity components (omega) change over time — that's the
precession. But |L| stays perfectly constant. This is the key insight:
angular momentum is conserved, not angular velocity.

## Run

```
cargo run -p example-energy-free-flight --release
```
