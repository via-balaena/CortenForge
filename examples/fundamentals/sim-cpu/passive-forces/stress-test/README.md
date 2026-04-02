# Stress Test — Passive Force Invariants

Headless validation of all passive force subsystems: fluid drag (inertia-box
and ellipsoid models), wind interaction, spring mechanics, damper mechanics,
ImplicitSpringDamper mode, and disable flags.

## Checks (18)

| # | Check | What it validates |
|---|-------|-------------------|
| 1 | Terminal velocity magnitude | Inertia-box drag balances gravity at analytical v_t |
| 2 | Drag proportional to v² | Quadratic scaling: F(2v)/F(v) = 4.0 |
| 3 | Zero medium = zero drag | density=0, viscosity=0 → qfrc_fluid == 0 |
| 4 | Viscous drag linear in velocity | Pure viscosity: F(2v)/F(v) = 2.0 |
| 5 | Ellipsoid activates on fluidshape | `fluidshape="ellipsoid"` → nonzero qfrc_fluid |
| 6 | Shape dependence | Flat cylinder has higher drag than sphere at same speed |
| 7 | Interaction coefficient scaling | geom_fluid[0]=0.5 → half the force of 1.0 |
| 8 | Wind on stationary body | Body at rest + wind → nonzero fluid force |
| 9 | Wind cancellation | Body velocity = wind → zero effective velocity, zero drag |
| 10 | Wind direction | Y-wind produces Y-force, not X-force |
| 11 | Spring restoring force | F = -k(q - springref) to machine precision |
| 12 | Springref != qpos0 | Spring pulls toward springref, not initial position |
| 13 | Spring + gravity equilibrium | Vertical slide settles at q = -mg/k |
| 14 | Damper dissipation | Total energy monotonically decreases |
| 15 | Damper force proportional to velocity | F = -b*qvel to machine precision |
| 16 | ImplicitSpringDamper suppression | qfrc_spring=0, qfrc_damper=0, fluid still computed |
| 17 | DISABLE_SPRING | Spring zeroed, damper still active |
| 18 | DISABLE_DAMPER | Damper zeroed, spring still active |

## Run

```
cargo run -p example-passive-stress-test --release
```
