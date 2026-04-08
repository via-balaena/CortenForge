# Refactors

Tracked cleanup work that isn't urgent but should be done.

---

## nalgebra re-export adoption

**Added**: 2026-04-08
**Context**: sim-core now re-exports 6 nalgebra types that appear in its
public API (`Vector3`, `UnitQuaternion`, `DVector`, `DMatrix`, `Matrix3`,
`Matrix6`). Downstream crates still import these directly from `nalgebra`.

**Why not now**: Every downstream crate (sim-mjcf, sim-bevy, sim-urdf,
sim-gpu, sim-simd) also uses nalgebra types that are NOT re-exported
(`Point3`, `Quaternion`, `Vector4`, `Rotation3`, `Isometry3`,
`UnitVector3`, `Vector2`). They can't drop their direct nalgebra dep
regardless, so switching `use nalgebra::Vector3` to `use sim_core::Vector3`
while keeping `use nalgebra::Point3` on the same line is cosmetic churn.

**When to do it**: If sim-core ever re-exports all nalgebra types used
across the workspace (or if a crate is refactored to only use re-exported
types), update that crate's imports and drop its direct nalgebra dep.

**Scope**: ~120 `use nalgebra::` statements across sim/ (excluding
sim-core internal, which correctly imports from nalgebra directly).
