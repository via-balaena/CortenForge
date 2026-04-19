# Co-simulation with MuJoCo

`sim-soft`'s co-simulation partner is `sim-core` — the MuJoCo-pipeline-aligned rigid-body runtime. `sim-mjcf` sits upstream of the handshake as a loader; it parses the MJCF XML file at setup, populates `sim-core`'s `Model` (which is immutable after load) and initial `Data` (which `sim-core` advances through time), and is not involved in the per-step exchange. This sub-chapter specifies what crosses the `sim-soft` ↔ `sim-core` interface at each handshake, in what representation, and what MuJoCo-side channel carries each field.

## The four boundary fields

At each handshake tick (period set by the coarser of the two native timesteps — see [§01 fixed-point](01-fixed-point.md)), four fields cross the interface. Two are required for the canonical problem; two are optional and activated by scene configuration.

**Rigid pose + spatial velocity** (`sim-core` → `sim-soft`, required). For each rigid body participating in contact with the deformable body, `sim-core` exports the body's current $SE(3)$ pose $(R, t) \in SO(3) \times \mathbb{R}^3$ (body origin translation and orientation) and its spatial velocity $(\omega, v) \in \mathbb{R}^3 \times \mathbb{R}^3$ (angular plus COM-centered linear velocity, expressed in a world-aligned frame translated to the body's COM). These are read directly out of `sim-core`'s `Data.xpos`, `Data.xquat`, `Data.cvel` arrays (MuJoCo-convention field names; `cvel` is the com-based spatial velocity). `sim-soft`'s contact detection — [Part 4 Ch 03 §00 bvh](../../40-contact/03-self-contact/00-bvh.md) for broadphase, [Part 4 Ch 03 §01 proximity](../../40-contact/03-self-contact/01-proximity.md) for pair generation — uses the pose to find contact pairs between the deformable body's surface tets and the rigid body's collision geometry. The velocity is used in two places: continuous-collision detection against the rigid body's swept motion across the handshake interval (see [Part 4 Ch 01 §02 ccd](../../40-contact/01-ipc-internals/02-ccd.md)) and computation of the relative tangential velocity at contact points for friction's lagged reference (see [Part 4 Ch 02 §02 ipc-friction](../../40-contact/02-friction/02-ipc-friction.md)).

**Contact force + contact-point locations** (`sim-soft` → `sim-core`, required). After `sim-soft` solves its Newton step against the rigid-boundary data held fixed for that iterate, it integrates the IPC barrier's contact force field over each contact region and returns a per-rigid-body net wrench $(f, \tau) \in \mathbb{R}^3 \times \mathbb{R}^3$ expressed in the world frame, plus the contact-region centroid as a diagnostic. The wrench is applied to the rigid body via the MuJoCo-convention `xfrc_applied` array (`sim-core`'s `Data.xfrc_applied`, a per-body 6-component Cartesian force/torque entry). MuJoCo's pipeline projects `xfrc_applied` through the body Jacobian (`qfrc_smooth += J^T * xfrc_applied`) during the forward-acceleration stage and advances the rigid body accordingly at its next native step. `sim-soft` does NOT write into `sim-core`'s constraint solver or its contact state; the soft-body reaction enters as an external wrench only, and `sim-core`'s constraint solver handles the rigid body's own constraints (joints, limits, rigid-rigid contacts) without seeing the soft-rigid pair. This is a one-way architecture at each simulator's internal solver (neither solver's state is touched by the other); the bidirectional exchange happens only at the handshake — consistent with the [Part 4 Ch 00 §01 impulse](../../40-contact/00-why-ipc/01-impulse.md) framing that the rigid-body side uses whatever contact law MuJoCo provides and the soft-body side uses IPC, without a shared constraint-level contact representation.

**Pin constraint reactions** (`sim-soft` → `sim-core`, optional). If the deformable body is pinned to a rigid body — for example, a silicone sleeve bonded to a rigid mounting flange — the constraint reaction at each pinned node is summed into an additional wrench on the host rigid body. This channel reuses the same `xfrc_applied` mechanism as the contact wrench (sum of both contributions before export) and is only populated when the scene configuration declares pinned nodes.

**Temperature and heat flux** (via `sim-thermostat`, optional). If thermal coupling is enabled, the deformable-body temperature field at the rigid-contact interface crosses as boundary data for `sim-thermostat`'s diffusion solve. The thermostat handshake is documented in [Part 11 Ch 02 §01 thermostat](../../110-crate/02-coupling/01-thermostat.md); it is independent of the `sim-core` handshake and not covered further here.

## Why `xfrc_applied` and not MuJoCo's contact system

MuJoCo's native contact system represents contacts as constraint-space entries that its convex solver resolves jointly. Injecting `sim-soft`'s contact forces there would require expressing the IPC barrier force in MuJoCo's contact-constraint representation (normal/tangential axes, friction cones, impulse limits), which the IPC barrier's continuous-force form does not map onto cleanly. Using `xfrc_applied` sidesteps the representational mismatch — the barrier force is a smooth external wrench, not a constraint impulse, and `xfrc_applied` accepts arbitrary per-body Cartesian wrenches with no additional structure. The architectural consequence is that `sim-core`'s constraint solver never sees the soft-rigid contact pair; it sees only the rigid body's other constraints (joints, limits, rigid-rigid contacts) plus the external wrench representing what the soft body is pushing back with.

## Representation and units

All boundary fields are expressed in SI units and in `sim-core`'s world frame, matching MuJoCo conventions:

- Position in metres, orientation as a unit quaternion $(w, x, y, z)$ (MuJoCo's convention; $w$ scalar-first).
- Linear velocity in m/s, angular velocity in rad/s.
- Force in newtons, torque in newton-metres.
- Wrenches in `xfrc_applied` are a (force, torque) pair in the world frame acting on the body at its centre of mass (MuJoCo's inertial reference point for applied wrenches); `sim-soft` composes the distributed contact-pressure field into an equivalent net force plus net torque about the body's COM before export. The contact-region centroid is serialized separately for diagnostic use only and does not affect the force application.

The canonical API sketch in `sim-soft::coupling::mjcf` reflects these conventions:

```rust
pub struct RigidBoundary {
    pub body_id: BodyId,         // sim-core body index (from sim-mjcf load)
    pub pose: Pose,              // SE(3): rotation + translation
    pub spatial_vel: SpatialVec, // MuJoCo cvel: world-aligned axes, linear at COM
}

pub struct ContactReaction {
    pub body_id: BodyId,
    pub wrench: Wrench,          // (force, torque) on the rigid body
    pub contact_centroid: Vec3,  // diagnostic; not used by sim-core
}

pub trait MjcfHandshake {
    /// Called by the outer coupling loop before sim-soft advances.
    fn import_rigid_state(&mut self, boundaries: &[RigidBoundary]);
    /// Called after sim-soft's Newton step converges; returns
    /// reactions to be fed to sim-core's xfrc_applied channel.
    fn export_contact_reactions(&self) -> Vec<ContactReaction>;
}
```

The full trait specification lives in [Part 11 Ch 02 §00](../../110-crate/02-coupling/00-mjcf.md); this sub-leaf fixes the boundary-data semantics that the trait implements.

## What this commits

- `sim-soft` treats `sim-core` as a black-box rigid-body runtime exposing `Model`/`Data` in MuJoCo conventions — pose (`xpos`, `xquat`) and velocity (`cvel`) read out, wrenches applied via `xfrc_applied`.
- `sim-mjcf` is a setup-time loader only; it is not involved in the per-step handshake.
- The four boundary fields above are the only state that crosses the interface; no intermediate Newton residuals, Hessian blocks, or constraint Jacobians are shared.
- The iteration that resolves mutual force-and-motion consistency across the interface is the fixed-point scheme in [§01](01-fixed-point.md).
