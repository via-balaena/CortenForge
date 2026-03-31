# Mimic Joint Coupling

Two revolute joints on a common base. The follower joint mimics the leader
with multiplier=2 and offset=0.1. The URDF `<mimic>` element converts to
an MJCF `<equality><joint>` with `polycoef`, creating a polynomial
coupling constraint enforced by the solver.

## What it tests

The URDF mimic semantics `position = multiplier * leader + offset` map to
the MuJoCo equality constraint:

```xml
<equality>
    <joint joint1="follower" joint2="leader" polycoef="0.1 2.0 0 0 0"/>
</equality>
```

The constraint solver enforces `follower = 2*leader + 0.1` at each
timestep. The example verifies this relationship holds from two different
initial conditions.

## Checks

| # | Check | Tolerance |
|---|-------|-----------|
| 1 | MJCF output contains `<equality>` with `polycoef` | exact |
| 2 | Model has equality constraints (neq > 0) | exact |
| 3 | Two joints in model | exact |
| 4 | Follower tracks 2*leader + 0.1 from q=0.5 | 0.05 rad |
| 5 | Constraint holds from q=-0.3 | 0.05 rad |

## Run

```
cargo run -p example-urdf-mimic --release
```
