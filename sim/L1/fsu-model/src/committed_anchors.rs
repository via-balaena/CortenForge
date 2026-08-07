//! The committed small-strain bending stiffness of the shipped raw and conformed disc, in
//! N·m/rad — **one definition, shared by the two gates that pin it.**
//!
//! `rung5_step1_mesh_realization_noise_floor_fom` lives in this crate's inline `#[cfg(test)]`
//! module (it needs private pipeline items, so it cannot move to `tests/`), while
//! `conform_delta_by_element` is a separate integration-test crate. The feature gate lets both
//! see the same constants without them entering the shipped library; `sim-core` uses the same
//! idiom. Rung β re-anchored two symmetric gates one at a time and left a retired rationale on
//! the second, so the point of one definition is that the next re-anchor of these four cannot
//! half-apply.
//!
//! **Scope.** Only these four. The coupled anchors (`RUNG7_K_DISC`, `BASELINE_K_DISC`,
//! `QUADRATIC_K_DISC`), the envelope peaks, the ramp displacements, and every prose
//! restatement of these numbers are still hand-set. Asserts cannot diverge; prose still can.
//!
//! `cfg(feature = ...)` is not `cfg(test)`, so `xtask grade` counts these as production
//! lines. `const` items emit no coverage segments, so the effect is nil — but that would not
//! hold for anything with a body.
//!
//! # What these numbers are
//!
//! Exact reproduction of a fixed configuration — the three BodyParts3D meshes at
//! `DiscParams::default` (cell 0.003), stepped ±0.5° sweep. **Not** a physics constant:
//! `rung5_step1` measures the absolutes moving 21.13 % peak-to-peak across a ±3.4 % cell
//! window, so each is one draw from a wide distribution. The *ratios* built from them carry
//! the physics claim.
//!
//! # Re-anchored at rung β.4
//!
//! α.1 stopped the mesher retaining phantom material, so these gates now measure a different
//! (correct) domain and every absolute fell 48–58 %.
//!
//! ⚠ **Rung 4's re-anchor rationale does not transfer.** That one was an element swap at fixed
//! geometry — "the old value was not wrong; it was the linear *element's* answer". Here the
//! element is unchanged and the geometry moved, so the old value is not a different-but-valid
//! reading: it is a reading of a mesh containing material the anatomy does not have.
//!
//! The element ratio moved 0.666 → 0.827 with a mechanism: Tet4 over-stiffness in bending grows
//! with element distortion, so phantom slivers inflated the Tet4↔Tet10 gap and removing them
//! must cost Tet4 more. Measured −58 % vs −48 %, in the predicted order. The conform ratio was
//! 0.982 / 0.985 and now rounds to 0.979 on both elements — consistent with the two axes being
//! separable, though one realization inside 1.81 % measured jitter cannot establish that.
//!
//! Measured at `808d499b`, verified at `6071b6e7`; meshes SHA-256-verified against the pin in
//! `design/cf-fsu-geometry/BODYPARTS3D.md`. `rung5_step1` and `conform_delta_by_element` agree
//! on the raw column to four decimals through two different probe drivers — but they share the
//! mesher, geometry pipeline, material and solver, so that cross-checks the **probe**, not the
//! mesh.

/// Raw (un-conformed) disc, linear Tet4 element: `(flexion, extension)` in N·m/rad.
pub const COMMITTED_TET4_RAW: (f64, f64) = (-0.1170, -0.1156);
/// Raw (un-conformed) disc, quadratic Tet10 element: `(flexion, extension)` in N·m/rad.
pub const COMMITTED_TET10_RAW: (f64, f64) = (-0.0968, -0.0958);
/// Endplate-conformed disc, linear Tet4 element: `(flexion, extension)` in N·m/rad.
pub const COMMITTED_TET4_CONFORMED: (f64, f64) = (-0.1146, -0.1132);
/// Endplate-conformed disc, quadratic Tet10 element: `(flexion, extension)` in N·m/rad.
pub const COMMITTED_TET10_CONFORMED: (f64, f64) = (-0.0948, -0.0938);
