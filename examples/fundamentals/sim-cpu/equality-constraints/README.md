# Equality Constraints

Equality constraints remove degrees of freedom between bodies or joints using
the **penalty method** — a stiff spring that pulls violations back toward zero.
Unlike rigid joints (hinge, slide, ball), equality constraints can be applied
between any two bodies and tuned via `solref` (stiffness/damping) and `solimp`
(impedance). Small but non-zero constraint violation is expected — this is
inherent to the penalty approach.

## Constraint Types

| Type | DOFs removed | What it does | Analogue |
|------|-------------|--------------|----------|
| **Connect** | 3 (translation) | Locks a point on body1 to body2 (or world). Rotation free. | Ball-and-socket joint |
| **Weld** | 6 (all) | Locks full pose of body1 relative to body2 (or world). | Rigid glue / fixed joint |
| **Distance** | 1 (scalar) | Maintains fixed distance between two geom centers. | Rigid rod |
| **Joint** | 1 per constraint | Couples joint positions: `q1 = poly(q2)`. | Gear train, mimic joint |

## Examples

| Example | Concept | Run |
|---------|---------|-----|
| [connect-to-world/](connect-to-world/) | Single body pinned to world origin | `cargo run -p example-equality-connect-to-world --release` |
| [connect-body-to-body/](connect-body-to-body/) | Two bodies chained (double pendulum) | `cargo run -p example-equality-connect-body-to-body --release` |
| [weld-to-world/](weld-to-world/) | Body frozen in space despite gravity | `cargo run -p example-equality-weld-to-world --release` |
| [weld-body-to-body/](weld-body-to-body/) | Two bodies glued, fall as one unit | `cargo run -p example-equality-weld-body-to-body --release` |
| [distance/](distance/) | Two spheres held 0.5m apart | `cargo run -p example-equality-distance --release` |
| [joint-mimic/](joint-mimic/) | Two hinges coupled 1:1 (mimic) | `cargo run -p example-equality-joint-mimic --release` |
| [joint-gear/](joint-gear/) | Two hinges coupled 2:1 with motor | `cargo run -p example-equality-joint-gear --release` |

Each example isolates one concept. Read the README inside each directory for
the physics explanation, parameter tables, and validation criteria.

## Key Concepts

**Connect vs Weld:** Connect allows rotation at the attachment point (like a
ball-and-socket). Weld locks everything — position and orientation. If you want
a body to pivot freely, use connect. If you want two bodies rigidly glued, use
weld.

**Distance vs Connect:** Distance constrains only the scalar separation between
two geom centers — the bodies can orbit freely in any direction. Connect locks
a specific point, constraining the full 3D position vector.

**Joint constraints:** Unlike the spatial constraints above, joint equality
constraints operate in **joint space** — they couple scalar joint positions via
a polynomial `q1 = c0 + c1*q2 + c2*q2² + ...`. The simplest cases are mimic
(1:1) and gear ratios (1:N).

**Penalty method:** All equality constraints use spring-damper penalties
controlled by `solref=[timeconst, dampratio]`. Stiffer constraints (smaller
timeconst) reduce violation but can cause instability. The examples use
`solref="0.005 1.0"` for spatial constraints and `solref="0.05 1.0"` for
joint-space constraints.

**Free joints required:** Connect, weld, and distance operate on body poses —
the bodies need `<freejoint/>` to have DOFs for the constraint to act on.
Joint constraints operate on existing hinge/slide joints instead.
