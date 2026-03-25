# Ball Joint — Bugs Found

## BUG-1: Z-up→Y-up rotation conversion inconsistent with position conversion

**Status:** FIXED
**Severity:** Visual correctness (affects ALL rotated body-attached geoms)
**Found:** 2026-03-25
**Fixed:** 2026-03-25 — `sim/L1/bevy/src/convert.rs`

`quat_from_unit_quaternion` used quaternion conjugation by a -90° X rotation to
convert physics rotations (Z-up) to Bevy rotations (Y-up). But the Y/Z swap is
a *reflection* (det=-1), not a rotation (det=+1). The conjugation maps
directions as `(x,y,z) → (x,z,-y)` while the position conversion
`vec3_from_vector` maps `(x,y,z) → (x,z,y)` — the Y component gets negated in
the rotation but not the position.

**Symptom:** For bodies with non-trivial rotations (ball joints, tilted hinges),
capsule/cylinder geom meshes pointed in a different direction than their
endpoint sphere positions. Visually: rod disconnected from tip sphere.

**Why it wasn't caught before:** All prior examples used either worldbody geoms
(no body rotation), slide joints (translation only), or hinge joints rotating
purely around X (X axis is unchanged by Y/Z swap, so the sign error didn't
manifest).

**Fix:** Replace conjugation with matrix swap `R_bevy = S·R_physics·S` where S
swaps rows/columns 1↔2. This is exactly consistent with the position swap.
The swap preserves `det(R)=+1` (it's still a proper rotation) because
`det(S)² = (-1)² = 1`.

**Verification:** New test `rotation_and_position_are_consistent` verifies that
a rotated capsule axis matches the position-converted endpoint. 33/33 sim-bevy
tests pass.
