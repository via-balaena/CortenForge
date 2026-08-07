// The committed small-strain bending stiffness of the shipped raw and conformed disc,
// in N·m/rad — **one definition, included by every gate that pins it.**
//
// `include!`d rather than `mod`-declared, because the two consumers cannot see each other:
// `rung5_step1_mesh_realization_noise_floor_fom` lives in the crate's inline
// `#[cfg(test)]` module, and `conform_delta_by_element` is a separate integration-test
// crate. Sharing through the library would mean making test anchors permanent public API of
// an L1 crate, which is the same objection that ruled out moving these gates to `tests/`.
//
// **Why one definition matters here specifically.** Rung β re-anchored two symmetric gates
// one at a time and left a retired rationale standing on the second — the cold read caught
// it, but the failure was structural, not careless. These values are pinned in both files
// and must agree; a hand-copy makes divergence silent, and the gate that drifts is the one
// nobody re-ran. With a single producer the next re-anchor is one edit and cannot half-apply.
//
// ## What these numbers are, and are not
//
// They pin **exact reproduction of a fixed configuration** — the three BodyParts3D meshes at
// `DiscParams::default` (cell 0.003), stepped ±0.5° sweep — and nothing more. They are *not*
// a physics constant. `rung5_step1`'s own realization sweep measures the absolutes moving
// ~21 % peak-to-peak across a ±3.4 % cell window, so every absolute here is one draw from a
// wide distribution; it is the *ratios* built from them that carry the physics claim.
// Re-anchoring them when the geometry changes is bookkeeping, and must be recorded as such.

// `#[allow(dead_code)]` per constant, justified: the table is deliberately whole while its
// consumers are partial — `rung5_step1` cross-checks only the raw pair, `conform_delta_by_element`
// pins all four. Splitting it per consumer would reintroduce the hand-copy this file exists to
// remove, and a whole table is also what makes a missing column visible. The attribute sits on
// each item rather than as a `#![...]` header because an included file is spliced mid-module,
// where inner attributes are not valid. Same reason this header is `//` and not `//!`.

// ## RE-ANCHORED at rung β.4 — measured, and what caused it
//
// α.1 stopped the mesher retaining phantom material, so the disc these gates measure is a
// different (correct) domain. Every absolute below fell 48–58 %; the previous values were
// measured on a mesh carrying slivers that are not anatomy.
//
// ⚠ **This is a GEOMETRY change, and rung 4's re-anchor rationale does not transfer.** That
// one justified itself as "the old value was not wrong; it was the linear *element's* answer"
// — an element swap at fixed geometry. Here the element is unchanged and the geometry moved,
// so the old value is not a different-but-valid reading: it is a reading of a mesh that
// contained material the anatomy does not have.
//
// ★ The RATIOS, which carry the physics, moved far less than the absolutes — and one of them
// got *cleaner*: the conform cost was 0.982 (Tet4) / 0.985 (Tet10) and is now **0.979 on both
// elements to three decimals**, so the conform's cost is element-independent, which is a
// stronger form of the orthogonality this crate already claimed. The element ratio moved
// 0.666 → **0.827** for a stated mechanism: Tet4 over-stiffness in bending grows with element
// distortion, so phantom slivers inflated the Tet4↔Tet10 gap, and removing them must cost
// Tet4 more than Tet10. Measured −58 % vs −48 %, in the predicted direction and order.
//
// Measured at `e10005a4`, meshes SHA-256-verified against the pin in
// `design/cf-fsu-geometry/BODYPARTS3D.md`. Two independent gates agree on the raw column to
// four decimals (`rung5_step1`'s shipped sweep row and `conform_delta_by_element`), by
// different code paths.

/// Raw (un-conformed) disc, linear Tet4 element: (flexion, extension).
#[allow(dead_code)]
const COMMITTED_TET4_RAW: (f64, f64) = (-0.1170, -0.1156);
/// Raw (un-conformed) disc, quadratic Tet10 element: (flexion, extension).
#[allow(dead_code)]
const COMMITTED_TET10_RAW: (f64, f64) = (-0.0968, -0.0958);
/// Endplate-conformed disc, linear Tet4 element: (flexion, extension).
#[allow(dead_code)]
const COMMITTED_TET4_CONFORMED: (f64, f64) = (-0.1146, -0.1132);
/// Endplate-conformed disc, quadratic Tet10 element: (flexion, extension).
#[allow(dead_code)]
const COMMITTED_TET10_CONFORMED: (f64, f64) = (-0.0948, -0.0938);
