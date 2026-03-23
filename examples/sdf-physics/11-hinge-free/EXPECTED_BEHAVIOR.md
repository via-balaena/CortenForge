# Example 09 — Geometry-Driven Hinge: Expected Behavior

## What this demonstrates

A flanged pin inside a capped socket where **SDF collision is the only
constraint**. No `JointKind::Revolute`. The geometry could be 3D printed
in place and would perform similarly.

This is the first CortenForge example where physics emerges entirely
from the shape, not from abstract mathematical joints.

## Setup

- **Socket**: Concave SDF (cylinder − bore − cap holes). Steel density (7800 kg/m³).
  Bore R=3.5mm, outer R=5.5mm, cap opening R=2.5mm.
- **Pin**: Convex SDF (shaft R=3.0mm + flanges R=3.2mm). PLA density (1250 kg/m³).
- Both are Free joint children of world (siblings). Ground plane supports socket.
- Pin starts 0.9mm below socket center, with initial ωz = 5 rad/s (spinning).

## What constrains what

| DOF | Constrained by | Mechanism |
|-----|---------------|-----------|
| X translation | Bore wall | Pin shaft (R=3.0) hits bore (R=3.5) |
| Y translation | Bore wall | Same, orthogonal direction |
| Z translation | Flange + cap | Pin flange (R=3.2) hits cap annulus (opening R=2.5) |
| X rotation | Bore wall | Radial contacts resist tipping |
| Y rotation | Bore wall | Same |
| **Z rotation** | **FREE** | **Pin spins freely inside bore** |

## What you should see

1. The dark steel socket sits on the ground, stationary
2. The red pin spins inside the socket (visible as rotating mesh facets)
3. The pin stays centered in the bore (doesn't drift laterally)
4. The pin stays at its axial position (doesn't fall through the cap)
5. Spin gradually slows from contact friction

## Pass criteria (at t ≥ 3s)

| Check | Threshold |
|-------|-----------|
| All values finite | No NaN/Inf |
| Radial constraint | \|Δxy\| < 1mm |
| Axial constraint | \|Δz\| < 2mm |
| Still spinning | \|ωz\| > 0.1 rad/s |
| Contacts active | ncon > 0 |
| Socket on ground | socket z > 5mm |

## Key dimensions

```
Clearance: bore R (3.5) − shaft R (3.0) = 0.5mm radial
Trap:      flange R (3.2) − cap opening R (2.5) = 0.7mm annulus
Axial:     bore half_h (8) − flange center (6) − flange half_h (1) = 1.0mm per side
Grid:      1.0mm SDF cell size (collision), 0.5mm visual mesh
```

## Architecture note

Both bodies are Free joint children of world — the same sibling topology
as examples 01–08 (stacking). The socket needs ground plane support;
without it, both bodies free-fall equally and the contact solver sees
zero relative velocity (no constraint activation).
