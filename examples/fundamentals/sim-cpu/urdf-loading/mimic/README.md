# Mimic Joint Coupling

Two pendulum arms side by side. The follower mimics the leader with
multiplier=2 and offset=0.1. The URDF `<mimic>` element converts to an
MJCF `<equality><joint>` with `polycoef`.

## What you see

Two arms swinging — the right arm (follower) swings at roughly double
the angle of the left arm (leader). The HUD shows both angles, the
expected follower angle, and the mimic tracking error.

## What it tests

URDF mimic semantics: `position = multiplier * leader + offset`

MuJoCo equivalent:
```xml
<equality>
    <joint joint1="follower" joint2="leader" polycoef="0.1 2.0 0 0 0"/>
</equality>
```

## Validation

| Check | Source |
|-------|--------|
| Has equality constraint (neq > 0) | `print_report` |
| Two joints in model | `print_report` |
| Mimic tracks (max err < 0.05 rad) | `print_report` (measured: ~0.0001 rad) |

## Run

```
cargo run -p example-urdf-mimic --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
