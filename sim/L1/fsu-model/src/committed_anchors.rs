//! The committed small-strain bending stiffness of the shipped raw and conformed disc,
//! in N·m/rad — **one definition, shared by every gate that pins it.**
//!
//! Gated on `any(test, feature = "test-fixtures")` because the two consumers cannot otherwise
//! see each other: [`crate`]'s inline `#[cfg(test)]` module hosts
//! `rung5_step1_mesh_realization_noise_floor_fom` (which needs private pipeline items and so
//! cannot move to `tests/`), while `conform_delta_by_element` is a separate integration-test
//! crate. The feature is this workspace's existing idiom for exactly that — `sim-core`
//! declares one and nine crates consume it — and the crate is `publish = false`, so no
//! semver surface is created.
//!
//! ⚠ Two costs of the feature gate, stated rather than discovered later. `cfg(feature = ...)`
//! is not `cfg(test)`, so `xtask grade`'s coverage criterion counts these lines as production;
//! they are `const` items with no executable regions, so the effect is nil, but the pattern
//! would not be free for anything with a body. And the module is compiled into the library
//! when the feature is on, so it must stay free of anything a consumer could misuse.
//!
//! **Why one definition matters here specifically.** Rung β re-anchored two symmetric gates one
//! at a time and left a retired rationale standing on the second — the cold read caught it, but
//! the failure was structural, not careless. These values are pinned in both files and must
//! agree; a hand-copy makes divergence silent, and the gate that drifts is the one nobody
//! re-ran. With a single producer the next re-anchor of **these four** is one edit.
//!
//! ⚠ Scope, stated so the guarantee is not over-trusted: the coupled anchors (`RUNG7_K_DISC`,
//! `BASELINE_K_DISC`, `QUADRATIC_K_DISC` — the last two pinning the *same* measured quantity in
//! two crates), the envelope peaks, the ramp displacements, and every prose restatement of
//! these numbers are still hand-set. Asserts can no longer diverge; prose still can.
//!
//! # What these numbers are, and are not
//!
//! They pin **exact reproduction of a fixed configuration** — the three BodyParts3D meshes at
//! `DiscParams::default` (cell 0.003), stepped ±0.5° sweep — and nothing more. They are *not* a
//! physics constant. `rung5_step1`'s own realization sweep measures the absolutes moving
//! **21.13 %** peak-to-peak across a ±3.4 % cell window, so every absolute here is one draw from
//! a wide distribution; it is the *ratios* built from them that carry the physics claim.
//! Re-anchoring them when the geometry changes is bookkeeping, and must be recorded as such.
//!
//! # Re-anchored at rung β.4 — measured, and what caused it
//!
//! α.1 stopped the mesher retaining phantom material, so the disc these gates measure is a
//! different (correct) domain. Every absolute below fell 48–58 %; the previous values were
//! measured on a mesh carrying slivers that are not anatomy.
//!
//! ⚠ **This is a GEOMETRY change, and rung 4's re-anchor rationale does not transfer.** That one
//! justified itself as "the old value was not wrong; it was the linear *element's* answer" — an
//! element swap at fixed geometry. Here the element is unchanged and the geometry moved, so the
//! old value is not a different-but-valid reading: it is a reading of a mesh that contained
//! material the anatomy does not have.
//!
//! ★ **The ratios moved far less than the absolutes.** The conform cost was 0.982 (Tet4) / 0.985
//! (Tet10) and now rounds to **0.979 on both elements**, consistent with the two axes being
//! separable. ⚠ That is consistency, not proof of element-independence: this is ONE mesh
//! realization, and `rung5_step1` measures 1.81 % peak-to-peak jitter on the element ratio
//! across its cell window (jitter on the *conform* ratio has never been measured), so a 0.015 %
//! agreement sits well inside the noise.
//!
//! The element ratio moved 0.666 → **0.827** for a stated mechanism: Tet4 over-stiffness in
//! bending grows with element distortion, so phantom slivers inflated the Tet4↔Tet10 gap, and
//! removing them must cost Tet4 more than Tet10. Measured −58 % vs −48 %, in the predicted
//! direction and order.
//!
//! Measured at `808d499b`, verified at `6071b6e7`, meshes SHA-256-verified against the pin in
//! `design/cf-fsu-geometry/BODYPARTS3D.md`. Two gates agree on the raw column to four decimals
//! (`rung5_step1`'s shipped sweep row and `conform_delta_by_element`) through two different
//! probe drivers — a two-probe `flexion_moment` and a stepped `capture_flexion`. ⚠ They share
//! the mesher, geometry pipeline, material and solver, i.e. everything α.1 touched, so this
//! cross-checks the **probe**, not the mesh.

/// Raw (un-conformed) disc, linear Tet4 element: `(flexion, extension)` in N·m/rad.
pub const COMMITTED_TET4_RAW: (f64, f64) = (-0.1170, -0.1156);
/// Raw (un-conformed) disc, quadratic Tet10 element: `(flexion, extension)` in N·m/rad.
pub const COMMITTED_TET10_RAW: (f64, f64) = (-0.0968, -0.0958);
/// Endplate-conformed disc, linear Tet4 element: `(flexion, extension)` in N·m/rad.
pub const COMMITTED_TET4_CONFORMED: (f64, f64) = (-0.1146, -0.1132);
/// Endplate-conformed disc, quadratic Tet10 element: `(flexion, extension)` in N·m/rad.
pub const COMMITTED_TET10_CONFORMED: (f64, f64) = (-0.0948, -0.0938);
