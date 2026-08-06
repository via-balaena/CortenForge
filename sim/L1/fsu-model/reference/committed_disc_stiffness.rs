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

/// Raw (un-conformed) disc, linear Tet4 element: (flexion, extension).
#[allow(dead_code)]
const COMMITTED_TET4_RAW: (f64, f64) = (-0.2811, -0.2788);
/// Raw (un-conformed) disc, quadratic Tet10 element: (flexion, extension).
#[allow(dead_code)]
const COMMITTED_TET10_RAW: (f64, f64) = (-0.1873, -0.1849);
/// Endplate-conformed disc, linear Tet4 element: (flexion, extension).
#[allow(dead_code)]
const COMMITTED_TET4_CONFORMED: (f64, f64) = (-0.2760, -0.2738);
/// Endplate-conformed disc, quadratic Tet10 element: (flexion, extension).
#[allow(dead_code)]
const COMMITTED_TET10_CONFORMED: (f64, f64) = (-0.1845, -0.1821);
