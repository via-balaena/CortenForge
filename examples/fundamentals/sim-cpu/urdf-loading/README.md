# URDF Loading — URDF-to-Model Pipeline

Ten examples covering the full `sim-urdf` pipeline: parsing URDF XML,
converting to MJCF, and compiling to a `sim-core::Model`. Each example
isolates a single URDF feature.

The pipeline is: URDF XML → `urdf_to_mjcf()` → MJCF XML → `load_model()` → Model/Data.

## Examples

Eight visual examples (Bevy window with HUD and orbit camera) plus two
headless validators:

| Example | Type | What it demonstrates |
|---------|------|---------------------|
| [revolute](revolute/) | Visual | Revolute pendulum, period + energy tracking |
| [prismatic](prismatic/) | Visual | Slide joint with spring, period tracking |
| [continuous](continuous/) | Visual | Unlimited revolute wheel, torque → velocity ramp |
| [fixed](fixed/) | Visual | Fixed joint fuses into parent, body count reduces |
| [mimic](mimic/) | Visual | `<mimic>` → equality constraint, 2:1 leader-follower |
| [geometry](geometry/) | Visual | Box + cylinder + sphere size conversion |
| [inertia](inertia/) | Visual | Diagonal vs fullinertia, different precession |
| [damping-friction](damping-friction/) | Visual | Three pendulums: no-loss, damped, friction |
| [error-handling](error-handling/) | Headless | Invalid URDFs → correct error variants (7 checks) |
| [stress-test](stress-test/) | Headless | Full pipeline regression gate (31 checks) |

## Key conversions

| URDF | MJCF | Notes |
|------|------|-------|
| `revolute` | `hinge` + `limited="true"` | Range from `<limit>` |
| `continuous` | `hinge` + `limited="false"` | No limits |
| `prismatic` | `slide` + `limited="true"` | Range from `<limit>` |
| `fixed` | eliminated by `fusestatic="true"` | Body merges into parent |
| `floating` | `free` | 6 DOF (3 translate + 3 rotate) |
| `planar` | 2 slides + 1 hinge | 3 DOF in plane |
| `box size="x y z"` | `box size="x/2 y/2 z/2"` | Full → half extents |
| `cylinder r l` | `cylinder size="r l/2"` | Full → half length |
| `sphere r` | `sphere size="r"` | Unchanged |
| `<dynamics damping>` | `damping="..."` | Velocity-dependent |
| `<dynamics friction>` | `frictionloss="..."` | Velocity-independent |
| `<mimic>` | `<equality><joint polycoef>` | Polynomial coupling |

## Running

Visual examples open a Bevy window:

```
cargo run -p example-urdf-revolute --release
```

Headless examples print to console and exit:

```
cargo run -p example-urdf-stress-test --release
cargo run -p example-urdf-error-handling --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
