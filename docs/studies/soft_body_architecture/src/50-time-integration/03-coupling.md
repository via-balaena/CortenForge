# Coupling with rigid-body timestep

`sim-soft` simulates deformable bodies; `sim-mjcf` simulates rigid bodies via MuJoCo. The [canonical problem](../10-physical/00-canonical.md) has both: a compliant cavity ([sim-soft](../110-crate/00-module-layout.md)) contacted by a rigid probe ([sim-mjcf](../110-crate/02-coupling/00-mjcf.md)). The two simulators have different timesteps, different solver families, and different conventions for representing contact. This chapter names the co-simulation scheme that bridges them and explains why a monolithic "one timestep solves both" approach was rejected.

| Section | What it covers |
|---|---|
| [Co-simulation with MuJoCo](03-coupling/00-mujoco.md) | The boundary: `sim-soft` receives rigid-body poses and velocities from `sim-mjcf` each substep; returns contact forces and (optionally) constrained-node reactions back into MuJoCo's external-force channel |
| [Fixed-point iteration across substeps](03-coupling/01-fixed-point.md) | The scheme: both simulators advance one substep, exchange boundary data, iterate 2–3 times to consistency; [Phase F's](../110-crate/03-build-order.md#the-committed-order) deliverable |

Two claims this chapter rests on.

**Fixed-point co-simulation, not monolithic assembly.** A monolithic scheme would assemble one Hessian containing both rigid-body and deformable DOFs, factor it once per Newton step, and solve the whole system jointly. This is standard in academic multi-physics FEM (FEBio, PolyFEM) but requires `sim-soft` to either absorb MuJoCo's constraint solver (rewriting MuJoCo internally, which is off-platform) or reimplement rigid-body dynamics (redundant with sim-mjcf, which [`sim-mjcf` is foundation-of-record for](../110-crate/02-coupling/00-mjcf.md)). The fixed-point alternative treats each simulator as a black box: advance `sim-soft` one substep with rigid-body boundary conditions held fixed, advance `sim-mjcf` one substep with contact-force boundary conditions held fixed, exchange the new boundary data, and iterate until the boundary disagrees by less than a tolerance (default 2–3 iterations in practice). Each simulator retains its own native solver; the coupling is an outer-loop handshake.

**The substep frequencies can differ.** `sim-mjcf` typically runs a rigid-body timestep of 0.5–2 ms for contact stability; `sim-soft` runs a soft-body timestep of 16 ms in experience mode or 1 ms in design mode. The fixed-point coupling resolves the mismatch by treating the coarser simulator's substep as the handshake period: within one `sim-soft` step (16 ms), `sim-mjcf` advances 16–32 internal substeps, and the coupling exchange happens at the `sim-soft` rate. The [Phase F deliverable](../110-crate/03-build-order.md#the-committed-order) is this handshake working end-to-end on the canonical problem; details of the substep-count mismatch and the iteration-convergence criterion are in the [fixed-point sub-chapter](03-coupling/01-fixed-point.md).

## Boundary data crossing the interface

Four fields cross the `sim-soft` ↔ `sim-mjcf` boundary at each handshake.

- **Rigid pose + velocity** (`sim-mjcf` → `sim-soft`): the probe's current $SE(3)$ pose and spatial velocity, used by `sim-soft`'s contact detection to find contact pairs between deformable-body surface tets and rigid-body geometry.
- **Contact force + contact-point locations** (`sim-soft` → `sim-mjcf`): the contact pressure field integrated over contact area, applied as an external wrench on the rigid body via MuJoCo's `qfrc_applied` channel.
- **Pin constraint reactions** (optional, `sim-soft` → `sim-mjcf`): if the deformable body is pinned to a rigid body (e.g., a silicone sleeve bonded to a rigid mount), the constraint reaction force on the pin is fed back.
- **Temperature and heat flux** (optional, via [`sim-thermostat`](../110-crate/02-coupling/01-thermostat.md)): if thermal coupling is enabled, the deformable-body temperature field at the rigid-contact interface.

Each field has a canonical representation documented in `sim-soft::coupling::mjcf` (see [Part 11 Ch 02 sub-chapter 00](../110-crate/02-coupling/00-mjcf.md)); the [Part 5 Ch 03 sub-chapter 00](03-coupling/00-mujoco.md) covers the co-simulation API and serialization details.

## What this does NOT cover

- **Coupling with `sim-thermostat`** — the temperature-field handshake is architecturally similar but lives in [Part 2 Ch 08](../20-materials/08-thermal-coupling.md) and [Part 11 Ch 02 sub-chapter 01](../110-crate/02-coupling/01-thermostat.md), since the physical coupling (dissipative heating, temperature-dependent modulus) is material-layer concern rather than time-integration concern.
- **Coupling with `cf-design`** — design-parameter edits are handled at the [SDF bridge](../70-sdf-pipeline/00-sdf-primitive.md) layer, not the time-integration layer. An SDF edit triggers re-meshing and warm-started re-solve; it does not cross through the `sim-mjcf` handshake.
- **Multi-soft-body coupling** — multiple `sim-soft` instances coupled through contact. Not a Phase F concern and not on the [Phase A–I roadmap](../110-crate/03-build-order.md); when it lands, it will use the same fixed-point scheme between instances.
