# Inertia Tensor Handling

Two free-floating boxes in zero gravity, side by side:
- **Left (blue):** diagonal inertia, `diaginertia` in this scene's MJCF
- **Right (orange):** off-diagonal inertia, `fullinertia` in this scene's MJCF

Both start with the same angular velocity but precess differently.

## What you see

Two colored boxes spinning. The blue box (diagonal/symmetric) precesses
smoothly. The orange box (off-diagonal/asymmetric) tumbles differently
due to cross-coupling terms in Euler's equations.

## What it tests

URDF always specifies a full symmetric inertia tensor (6 values). In an
unturned inertial origin the converter writes it as `fullinertia` in MJCF,
a diagonal tensor too (six zero moments, a tensor not given, as
`diaginertia="0 0 0"`), as MuJoCo 3.5.0's URDF importer does. The two bodies here are MJCF written
by hand, one with `diaginertia` and one with `fullinertia`. Both paths
must produce correct dynamics.

Note: this example uses a combined MJCF model (not URDF) for rendering,
since URDF can't express two independent free-floating bodies in one file.
The URDF→fullinertia conversion is verified separately.

## Validation

| Check | Source |
|-------|--------|
| Off-diagonal → fullinertia in MJCF | `print_report` (checks URDF converter output) |
| Diagonal body spins stably | `print_report` |
| Full inertia body spins stably | `print_report` |
| Different precession (diverged) | `print_report` (omega_diff > 0.01) |

## Run

```
cargo run -p example-urdf-inertia --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
