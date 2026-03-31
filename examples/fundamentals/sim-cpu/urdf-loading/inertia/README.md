# Inertia Tensor Handling

Two free-floating bodies in zero gravity with initial angular velocity:

1. **Diagonal inertia** (I_xy = I_xz = I_yz = 0) → `diaginertia` in MJCF
2. **Off-diagonal inertia** (cross-coupling terms) → `fullinertia` in MJCF

Both are given identical angular velocity. The diagonal body precesses
symmetrically; the off-diagonal body precesses differently due to
cross-coupling terms in Euler's equations.

## What it tests

URDF always specifies a full 3x3 symmetric inertia tensor (6 values:
ixx, ixy, ixz, iyy, iyz, izz). The converter detects whether
off-diagonal terms are present and emits either `diaginertia="Ixx Iyy Izz"`
or `fullinertia="Ixx Iyy Izz Ixy Ixz Iyz"` in the MJCF output.

Both paths must produce correct dynamics. The test verifies that:
- Diagonal inertia values propagate correctly
- Off-diagonal inertia triggers the `fullinertia` path
- Both bodies spin stably without NaN
- Different inertias produce measurably different precession patterns

## Checks

| # | Check | Tolerance |
|---|-------|-----------|
| 1 | Diagonal inertia values (0.1, 0.2, 0.3) | 0.001 |
| 2 | Off-diagonal → `fullinertia` in MJCF | exact |
| 3 | Full inertia model loads (mass = 2.0) | 0.001 |
| 4 | Diagonal body spins stably after 5000 steps | qualitative |
| 5 | Different inertias → different precession | diff > 0.01 |

## Run

```
cargo run -p example-urdf-inertia --release
```
