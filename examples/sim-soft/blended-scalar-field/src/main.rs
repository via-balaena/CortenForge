//! blended-scalar-field — `BlendedScalarField` smooth cubic-Hermite-smoothstep
//! transition between two uniform Lamé regions (the "stiff skin over soft
//! core" canonical idiom from `material_field.rs:130`), with the same SDF
//! used both inside the field's blend AND attached via
//! `MaterialField::with_interface_sdf` to populate `Mesh::interface_flags`.
//!
//! Per inventory Tier 3 row 9 — the canonical Phase 4 §02 §01 composition
//! example. A solid `SphereSdf{ radius: R_OUTER }` body (reused from row 8)
//! is meshed; an interior `SphereSdf{ radius: R_INTERFACE }` (NEW)
//! drives two `BlendedScalarField`s (one per Lamé parameter) blending
//! `(MU_INNER, LAMBDA_INNER)` (inside the interface sphere) into
//! `(MU_OUTER, LAMBDA_OUTER)` (outside it) over a band of half-width
//! `BAND_HALF_WIDTH = 0.015 m` straddling the interface zero set. The
//! same interface SDF is also threaded through `with_interface_sdf` so
//! `Mesh::interface_flags` populates per the book's `|φ(x_c)| < L_e` rule
//! (Part 7 §02 §01) — the headline new feature vs row 8's all-false
//! interface flags.
//!
//! The headline cf-view artifact is a per-tet centroid point cloud,
//! filtered to a thin `|z| < cell_size/2` z-slab (row 8 banked pattern)
//! so the projected disk reads cleanly as concentric color regions on
//! the z=0 plane. The PLY carries three per-vertex scalars: the
//! categorical `interface_flag` (tab10 binary highlight of the IV-6
//! flagged band), the continuous `material_mu` (viridis sequential —
//! the physical readout), and the continuous `smoothstep_weight`
//! (viridis sequential — the cubic Hermite kernel `s²(3 − 2s)`
//! mechanism). cf-view's dropdown switches between them; alphabetical
//! first selects `interface_flag` on launch.
//!
//! Geometry constants reuse `LAYERED_SPHERE_R_OUTER` /
//! `_BBOX_HALF_EXTENT` from sim-soft's re-exports — same body sphere
//! and bbox as row 8 for cross-row visual continuity. `R_INTERFACE`
//! and `BAND_HALF_WIDTH` are NEW constants, semantically distinct from
//! row 8's partition-boundary constants (`R_INNER_OUTER` /
//! `R_OUTER_INNER`) — row 9's interface is the BlendedScalarField's
//! zero-set "stiff skin over soft core" boundary, NOT a partition
//! boundary. Lamé pairs reuse row 8's MU_INNER / LAMBDA_INNER /
//! MU_OUTER / LAMBDA_OUTER (skipping the middle shell): row 9 reads
//! pedagogically as "row 8 with the middle shell dissolved into a
//! smoothstep transition between the inner and outer values".
//!
//! `cell_size = 0.02` matches III-1 / IV-4 h/2 canonical (same as row
//! 8) → 6768 tets per the III-1 determinism contract.
//!
//! Anchor groups (all assertions exit-0 on success):
//!
//! - **Band-half-width validity** — `BAND_HALF_WIDTH > 0` and finite +
//!   band fully interior (`R_INTERFACE − BAND_HALF_WIDTH > 0`,
//!   `R_INTERFACE + BAND_HALF_WIDTH < R_OUTER`); `const { assert!(...) }`
//!   compile-time enforcement on the geometry constants.
//! - **Determinism** — second `from_sdf` call produces an
//!   `equals_structurally` mesh + materials cache energy(F_probe)
//!   bit-equal + `interface_flags()` cache bit-equal element-by-element.
//! - **Counts** — `n_tets`, `n_vertices`, `referenced_vertices.len()`
//!   exact-pinned per III-1 contract (same scene + resolution as
//!   row 3 / row 8).
//! - **Per-tet blended assignment matches smoothstep predicate** — for
//!   every tet, `mesh.materials()[t]` (probed via `energy(F_probe)` +
//!   `first_piola(F_probe)` bit-equal at `F_probe = diag(1.2, 1, 1)`)
//!   matches `expected = NH::from_lame(blend(MU_INNER, MU_OUTER, w),
//!   blend(LAMBDA_INNER, LAMBDA_OUTER, w))` where `w =
//!   smoothstep_weight(phi(centroid))` is re-implemented test-side
//!   bit-exact-matching `BlendedScalarField::sample`'s arithmetic
//!   (clamp + cubic Hermite + FMA-blend). HEADLINE 1 — cross-impl gate
//!   between mesher centroid-sampling pass and test-side smoothstep
//!   re-derivation; drift in either fires loud.
//! - **Outside-band snaps bit-equal** — for tets with
//!   `phi(c) <= -BAND_HALF_WIDTH`, `mesh.materials()[t]` bit-equal
//!   `NH::from_lame(MU_INNER, LAMBDA_INNER)`; for `phi(c) >=
//!   +BAND_HALF_WIDTH`, bit-equal `(MU_OUTER, LAMBDA_OUTER)`. Verifies
//!   the BlendedScalarField "snaps cleanly to 0/1 outside the band"
//!   contract (`field/layered.rs:148-153`).
//! - **Inside-band monotonic μ gradient** — for tets in the
//!   smoothstep band (`|phi(c)| < BAND_HALF_WIDTH`), sort by
//!   `phi(centroid)` ascending; assert the test-side blended μ values
//!   are monotone non-decreasing across the sorted order. Inventory
//!   row 9's named gate — directly verifies the smooth-gradient
//!   property the row's headline ships.
//! - **Interface flags match book rule per tet** — for every tet,
//!   `mesh.interface_flags()[t]` bit-equal `(|phi(centroid)| <
//!   L_e(t))` where `L_e(t)` is the test-side re-derivation of the
//!   six-edge-mean rule (`mesh/mod.rs:180-187`). HEADLINE 2 — first
//!   user-facing coverage of the `BlendedScalarField +
//!   with_interface_sdf` composition path uncovered by IV-6
//!   (`tests/interface_band_flagging.rs:53-60` explicitly notes IV-6
//!   uses the LayeredScalarField shell-pattern, not BlendedScalarField).
//! - **Band populations all non-empty** — inside-snap, band, and
//!   outside-snap each have ≥ 1 tet, AND `interface_flag = true` count
//!   is ≥ 1. Sanity guard against a degenerate scene + the
//!   monotonicity / per-tet-flag anchors having something to verify.
//! - **Band populations exact** — captured per-bucket smoothstep tet
//!   counts (inside-snap / band / outside-snap) AND the IV-6
//!   flagged-tet count under the III-1 determinism contract; pinned
//!   separately because the smoothstep band (`|phi| < BAND_HALF_WIDTH`)
//!   and the IV-6 interface band (`|phi| < L_e`) are DIFFERENT bands
//!   with overlapping but distinct populations.
//! - **Z-slab visual populations exact** — captured per-bucket
//!   smoothstep tet counts in the `|z| < cell_size/2` slab cut that
//!   drives the cf-view PLY. Visual-pedagogy guard: each bucket must
//!   have ≥ 1 z-slab centroid for the smooth gradient to read in
//!   cf-view's projected disk.

// PLY field-data is single-precision on disk; converting f64 quantities
// (interface_flag 0.0/1.0, material_mu, smoothstep_weight) to f32 for
// the AttributedMesh extras is intrinsic to the PLY format. Same
// precedent as rows 1+2+3+8.
#![allow(clippy::cast_possible_truncation)]
// `from_sdf(...).expect(...)` — the canonical interior-interface scene
// either succeeds by construction or surfaces a regression worth
// investigating. Same precedent as row 8.
#![allow(clippy::expect_used)]
// `as TetId` cast on `mesh.n_tets()` (≤ 6768 here, ≪ u32::MAX) — the
// standard Mesh-trait API tax mirrored across the workspace.
#![allow(clippy::cast_possible_wrap)]
// `doc_markdown` flags `VIEWER_DESIGN.md` (a literal filename, not a
// code identifier) inside the module docstring. Same allowance as
// rows 5 + 6 + 8 for Unicode math notation.
#![allow(clippy::doc_markdown)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use nalgebra::Matrix3;
use sim_soft::{
    Aabb3, BlendedScalarField, ConstantField, Field, LAYERED_SPHERE_BBOX_HALF_EXTENT,
    LAYERED_SPHERE_R_OUTER, Material, MaterialField, Mesh, MeshingHints, NeoHookean,
    SdfMeshedTetMesh, SphereSdf, TetId, Vec3, VertexId, referenced_vertices,
};

// =============================================================================
// Constants
// =============================================================================

/// Outer radius (m) — body's exterior surface. Reused from
/// `LAYERED_SPHERE_R_OUTER` for cross-row alignment with row 8 +
/// (forthcoming) row 11.
const R_OUTER: f64 = LAYERED_SPHERE_R_OUTER;

/// Interface radius (m) — interior sphere driving the BlendedScalarField
/// blend AND attached as the IV-6 interface SDF. NEW constant
/// semantically distinct from row 8's partition-boundary constants
/// (`R_INNER_OUTER` / `R_OUTER_INNER`): row 9's interface is the
/// BlendedScalarField zero set ("stiff skin over soft core"), NOT a
/// partition boundary. Visual midpoint between row 8's
/// `R_INNER_OUTER = 0.06` and `R_OUTER_INNER = 0.08`.
const R_INTERFACE: f64 = 0.07;

/// Smoothstep band half-width (m). `0.015 ≈ 0.75 × CELL_SIZE`, so the
/// full band (`2 × BAND_HALF_WIDTH = 0.03`) is roughly 1.5 BCC cells
/// wide — well-resolved by the mesh (band populated by 2736 of 6768
/// tets at this resolution per `N_BAND_EXACT` capture), and not so
/// wide that the inside-snap region collapses (1056 inside-snap tets
/// per `N_INSIDE_SNAP_EXACT`). Strictly interior to the body:
/// `R_INTERFACE − BAND_HALF_WIDTH = 0.055 > 0` and `R_INTERFACE +
/// BAND_HALF_WIDTH = 0.085 < R_OUTER = 0.10`. Note: distinct from the
/// IV-6 per-tet `L_e` band — `L_e(t)` varies per tet with the BCC +
/// stuffing edge distribution and is empirically larger than this on
/// most tets at this resolution (flagged count 3480 > band count 2736
/// per the captured constants below).
const BAND_HALF_WIDTH: f64 = 0.015;

/// Bounding-box half-extent (m). Matches the III-1 / IV-4 canonical
/// `bbox_half_extent / cell_size = 6.0` ratio at `cell_size = h/2 = 0.02`.
const BBOX_HALF_EXTENT: f64 = LAYERED_SPHERE_BBOX_HALF_EXTENT;

/// BCC lattice spacing (m) — III-1 / IV-4 h/2 canonical (same as row 8).
const CELL_SIZE: f64 = 0.02;

// ── Lamé pairs per region ────────────────────────────────────────────────
//
// Reuse row 8's `MU_INNER` / `LAMBDA_INNER` (innermost shell) and
// `MU_OUTER` / `LAMBDA_OUTER` (outermost shell), skipping the middle
// shell. Row 9 reads pedagogically as "row 8 with the middle shell
// dissolved into a smoothstep transition between the inner and outer
// values." All in `λ = 4μ` ⇒ `ν = 0.4` compressible regime; `0.5×` and
// `2×` IV-1 baseline `(1e5, 4e5)`.

const MU_INNER: f64 = 5.0e4;
const LAMBDA_INNER: f64 = 2.0e5;
const MU_OUTER: f64 = 2.0e5;
const LAMBDA_OUTER: f64 = 8.0e5;

/// Probe stretch for the Material-trait value-correctness gate.
/// `F_probe = diag(PROBE_LAMBDA, 1, 1)` with `PROBE_LAMBDA = 1.20` ties
/// to row 6's `LAMBDA_STRETCH` and row 8's `PROBE_LAMBDA` for
/// cross-row continuity.
const PROBE_LAMBDA: f64 = 1.20;

/// Bit-equal tolerance for the Material-trait probe. Both the test-side
/// blended `expected` and the mesher-side `mesh.materials()[t]` run
/// identical `NeoHookean::first_piola` / `energy` arithmetic on identical
/// blended `(μ, λ)` pairs (test-side blend bit-exact-matches
/// `BlendedScalarField::sample`'s mul_add chain) on identical `F_probe`
/// — outputs are bit-equal by construction on a fixed toolchain.
/// `epsilon = 0.0` satisfies clippy's `float_cmp` lint without actually
/// relaxing the bound (mirrors row 2's `EXACT_TOL = 0.0`).
const EXACT_TOL: f64 = 0.0;

// ── Exact-pinned counts (III-1 determinism contract) ─────────────────────
//
// Same scene topology + cell_size as row 3 / row 8 (`SphereSdf{ R_OUTER
// = 0.10 }`, bbox `[-0.12, 0.12]³`, cell_size 0.02). Material-field
// carry doesn't affect mesh topology (BCC + stuffing runs before
// `materials_from_field` populates the cache) so n_tets / n_vertices /
// referenced inherit row 3 / row 8 exactly.

/// Total emitted tet count. Identical to row 3 / row 8 — material-field
/// composition does not shift mesh topology.
const N_TETS_EXACT: usize = 6768;

/// Total mesh vertex count, including orphan BCC lattice corners.
/// Identical to row 3 / row 8.
const N_VERTICES_EXACT: usize = 4634;

/// Vertices referenced by at least one tet. Identical to row 3 / row 8.
const N_REFERENCED_EXACT: usize = 1483;

/// Per-bucket smoothstep tet counts captured on the canonical scene.
/// `INSIDE_SNAP + BAND + OUTSIDE_SNAP == N_TETS_EXACT` exact (the
/// partition is total over the body's tets — every tet bucket-assigns
/// into exactly one of `phi ≤ -BAND_HALF_WIDTH`, `|phi| <
/// BAND_HALF_WIDTH`, or `phi ≥ +BAND_HALF_WIDTH`). Captured 2026-05-05
/// at sim-soft `dev` (post-row-8 tip `ce6485f4`, post-N+3 row-8 docs),
/// rustc 1.95.0 (`59807616e` 2026-04-14) on macOS arm64 — matching the
/// toolchain and platform of IV-1's reference capture per
/// `invariant_iv_1_uniform_passthrough.rs:138-151`. III-1 determinism
/// contract makes these run-to-run + same-toolchain bit-stable;
/// cross-platform sparse-mesh stability (~3k tets) is empirically
/// untested, so a future CI matrix expansion may surface drift —
/// relax via the IV-1 sparse-tier protocol (do NOT re-bake without
/// diagnosing toolchain vs real regression first).
const N_INSIDE_SNAP_EXACT: usize = 1056;
const N_BAND_EXACT: usize = 2736;
const N_OUTSIDE_SNAP_EXACT: usize = 2976;

/// IV-6 interface-flagged tet count under the `|phi(c)| < L_e` rule
/// (Part 7 §02 §01), where `L_e(t)` is the per-tet six-edge mean
/// (`mesh/mod.rs:180-186`). DIFFERENT from the smoothstep band counts
/// above — `L_e(t)` varies per tet with the BCC + Isosurface Stuffing
/// edge-length distribution and is empirically larger on most tets
/// than `BAND_HALF_WIDTH`, so the IV-6 flagged count exceeds the
/// smoothstep band count. Capture provenance inherits the block above.
const N_FLAGGED_EXACT: usize = 3480;

/// Per-bucket smoothstep tet counts in the visual-PLY z-slab cut
/// (`|centroid.z| < CELL_SIZE / 2`). Capture provenance inherits the
/// block above. Z-slab is symmetric about z=0, so counts are
/// approximately `(slab_thickness / body_diameter) × N_TETS = (cell_size
/// / (2 R_OUTER)) × N_TETS = 0.10 × 6768 ≈ 677` total (exact-pinned
/// per-bucket). The z-slab PLY is the cf-view artifact — projecting
/// onto a thin disk reveals the smooth inside-snap → band → outside-snap
/// gradient unmistakably from any camera angle, mirroring row 8's
/// banked z-slab pattern (see
/// `project_cf_viewer_dense_point_cloud_gap.md` for the engine-side
/// followup).
const N_INSIDE_SNAP_ZSLAB_EXACT: usize = 200;
const N_BAND_ZSLAB_EXACT: usize = 248;
const N_OUTSIDE_SNAP_ZSLAB_EXACT: usize = 200;

/// Half-thickness of the z-slab visual cut. `cell_size / 2 = 0.01 m` —
/// row 8's banked z-slab visual-cut pattern reused verbatim for cf-view
/// rendering against dense per-tet centroid clouds (see
/// `project_cf_viewer_dense_point_cloud_gap.md`).
const ZSLAB_HALF_THICKNESS: f64 = CELL_SIZE / 2.0;

// =============================================================================
// Smoothstep predicate (test-side bit-exact mirror of BlendedScalarField)
// =============================================================================

/// Cubic Hermite smoothstep `outside_weight = s² (3 − 2s)` with
/// `s = clamp((phi + band) / (2 · band), 0, 1)` — bit-exact mirror of
/// `BlendedScalarField::sample`'s arithmetic at `field/layered.rs:213-219`.
/// At the three reference points: `phi <= -BAND_HALF_WIDTH ⇒ w = 0`
/// exactly; `phi == 0 ⇒ w = 0.5` exactly; `phi >= +BAND_HALF_WIDTH ⇒
/// w = 1` exactly (the cubic kernel is C¹ at both band edges and
/// bit-exact at s ∈ {0, 0.5, 1}).
fn smoothstep_weight(phi: f64) -> f64 {
    let s = ((phi + BAND_HALF_WIDTH) / (2.0 * BAND_HALF_WIDTH)).clamp(0.0, 1.0);
    s * s * 2.0_f64.mul_add(-s, 3.0)
}

/// FMA-fused affine blend `(1 − w) · inside + w · outside` — bit-exact
/// mirror of `BlendedScalarField::sample`'s final blend at
/// `field/layered.rs:222-224`. Same operation order: compute
/// `w * outside` first (one rounding), then `fma(1 - w, inside, _)`
/// (one fma rounding); test-side and mesher-side outputs bit-equal by
/// construction on a fixed toolchain.
fn fma_blend(weight: f64, inside: f64, outside: f64) -> f64 {
    let outside_term = weight * outside;
    (1.0 - weight).mul_add(inside, outside_term)
}

/// Smoothstep bucket label: 0 = inside snap (`phi ≤ -BAND_HALF_WIDTH`),
/// 1 = band (`-BAND_HALF_WIDTH < phi < +BAND_HALF_WIDTH`), 2 = outside
/// snap (`phi ≥ +BAND_HALF_WIDTH`). Boundary convention follows the
/// `clamp(0, 1)` in `BlendedScalarField::sample`: at exactly
/// `phi = -BAND_HALF_WIDTH`, `s = 0`, `outside_weight = 0` ⇒ inside
/// snap; at exactly `phi = +BAND_HALF_WIDTH`, `s = 1`,
/// `outside_weight = 1` ⇒ outside snap. Strict `<` for the band
/// interior accordingly.
fn smoothstep_bucket(phi: f64) -> usize {
    if phi <= -BAND_HALF_WIDTH {
        0
    } else if phi >= BAND_HALF_WIDTH {
        2
    } else {
        1
    }
}

/// Per-tet mean edge length `L_e = (||e_01|| + ||e_02|| + ||e_03|| +
/// ||e_12|| + ||e_13|| + ||e_23||) / 6` — bit-exact mirror of
/// `interface_flags_from_field` at `mesh/mod.rs:180-186`. Six norms in
/// the exact same order, then divide once. Used in
/// [`verify_interface_flags_match_book_rule_per_tet`] to re-derive the
/// expected interface flag.
fn mean_edge_length(v0: Vec3, v1: Vec3, v2: Vec3, v3: Vec3) -> f64 {
    ((v1 - v0).norm()
        + (v2 - v0).norm()
        + (v3 - v0).norm()
        + (v2 - v1).norm()
        + (v3 - v1).norm()
        + (v3 - v2).norm())
        / 6.0
}

// =============================================================================
// Constructors
// =============================================================================

/// Build the canonical 2-region `MaterialField` with interface SDF
/// attached for IV-6 flag population.
///
/// Two `BlendedScalarField`s — one for `μ`, one for `λ` — keyed on the
/// interior `SphereSdf{ radius: R_INTERFACE }`, blending two
/// `ConstantField`s via the cubic Hermite smoothstep `s²(3 − 2s)` over a
/// band of half-width `BAND_HALF_WIDTH` straddling the interface. The
/// SAME interior sphere is then attached via
/// `MaterialField::with_interface_sdf` so `Mesh::interface_flags`
/// populates per the `|φ(x_c)| < L_e` rule (Part 7 §02 §01) — the
/// canonical "stiff skin over soft core" pattern named in
/// `material_field.rs:130`.
///
/// Box-allocated trait objects per call (rather than `clone()`) because
/// `BlendedScalarField::new` takes ownership of the SDF and field
/// boxes; same precedent as row 8's `body = || Box::new(...)` closure.
fn build_blended_field() -> MaterialField {
    let interface_sdf = || {
        Box::new(SphereSdf {
            radius: R_INTERFACE,
        })
    };
    let mu_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
        interface_sdf(),
        Box::new(ConstantField::new(MU_INNER)),
        Box::new(ConstantField::new(MU_OUTER)),
        BAND_HALF_WIDTH,
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
        interface_sdf(),
        Box::new(ConstantField::new(LAMBDA_INNER)),
        Box::new(ConstantField::new(LAMBDA_OUTER)),
        BAND_HALF_WIDTH,
    ));
    MaterialField::from_fields(mu_field, lambda_field).with_interface_sdf(interface_sdf())
}

/// Build the canonical blended mesh — solid `SphereSdf{ radius: R_OUTER }`
/// meshed at `cell_size = 0.02`, with the 2-region BlendedScalarField
/// `MaterialField` carrying per-tet `(μ, λ)` via centroid sampling AND
/// the per-tet `interface_flags` populated via the IV-6 band rule.
fn build_blended_mesh() -> SdfMeshedTetMesh {
    let half = BBOX_HALF_EXTENT;
    let hints = MeshingHints {
        bbox: Aabb3::new(Vec3::new(-half, -half, -half), Vec3::new(half, half, half)),
        cell_size: CELL_SIZE,
        material_field: Some(build_blended_field()),
    };
    SdfMeshedTetMesh::from_sdf(&SphereSdf { radius: R_OUTER }, &hints)
        .expect("canonical interior-interface scene meshes successfully — see III-1 + IV-6")
}

// =============================================================================
// Per-tet record
// =============================================================================

#[derive(Debug, Clone)]
struct TetRecord {
    tet_id: u32,
    centroid: Vec3,
    /// `phi = sdf.eval(centroid) = ||centroid|| - R_INTERFACE` (bit-exact
    /// mirror of `SphereSdf::eval`).
    phi: f64,
    /// Cubic Hermite smoothstep weight `s²(3 − 2s)` from
    /// [`smoothstep_weight`].
    weight: f64,
    /// Smoothstep bucket label (0 / 1 / 2) from [`smoothstep_bucket`].
    bucket: usize,
    /// Mean edge length from [`mean_edge_length`] — drives the
    /// re-derivation of the IV-6 interface flag.
    l_e: f64,
    /// Test-side re-derivation of `Mesh::interface_flags()[t]`:
    /// `|phi| < L_e` (strict `<`, mirroring `mesh/mod.rs:187`).
    expected_flag: bool,
    /// Test-side blended μ via [`fma_blend`] — the value
    /// `mesh.materials()[t]` should carry, used in the per-tet
    /// value-correctness gate.
    blended_mu: f64,
    /// Test-side blended λ via [`fma_blend`] — sister of `blended_mu`.
    blended_lambda: f64,
}

/// Walk every tet, compute its centroid + phi + smoothstep weight +
/// bucket + L_e + expected interface flag + blended (μ, λ).
/// The centroid IS the canonical evaluation point per Part 7 §02 §00 —
/// same point `materials_from_field` and `interface_flags_from_field`
/// sample at. Each computation here bit-exact mirrors the corresponding
/// engine-side helper, so downstream `assert_eq!` / `assert_relative_eq!`
/// against `mesh.materials()` / `mesh.interface_flags()` all hold at
/// `EXACT_TOL = 0.0` on a fixed toolchain.
fn extract_centroid_records(mesh: &SdfMeshedTetMesh) -> Vec<TetRecord> {
    let positions = mesh.positions();
    let n_tets_id = mesh.n_tets() as TetId;
    let mut records = Vec::with_capacity(mesh.n_tets());
    for tet_id in 0..n_tets_id {
        let [v0_id, v1_id, v2_id, v3_id] = mesh.tet_vertices(tet_id);
        let v0 = positions[v0_id as usize];
        let v1 = positions[v1_id as usize];
        let v2 = positions[v2_id as usize];
        let v3 = positions[v3_id as usize];
        let centroid = (v0 + v1 + v2 + v3) * 0.25;
        let phi = centroid.norm() - R_INTERFACE;
        let weight = smoothstep_weight(phi);
        let bucket = smoothstep_bucket(phi);
        let l_e = mean_edge_length(v0, v1, v2, v3);
        let expected_flag = phi.abs() < l_e;
        let blended_mu = fma_blend(weight, MU_INNER, MU_OUTER);
        let blended_lambda = fma_blend(weight, LAMBDA_INNER, LAMBDA_OUTER);
        records.push(TetRecord {
            tet_id,
            centroid,
            phi,
            weight,
            bucket,
            l_e,
            expected_flag,
            blended_mu,
            blended_lambda,
        });
    }
    records
}

// =============================================================================
// verify_band_half_width_validity
// =============================================================================

const fn verify_band_half_width_validity() {
    // Re-assert the BlendedScalarField constructor's invariant
    // (`band_half_width` strictly positive and finite) at the user-facing
    // example layer. Geometry constants are compile-time; the asserts
    // evaluate at compile time via `const { ... }` blocks — reaching
    // runtime here is impossible, but the const-block panic surfaces at
    // compile time on a regression and the source location anchors the
    // dependency on the band width and band-fully-interior contract
    // loud and user-facing. Mirrors the BlendedScalarField runtime
    // invariant (`field/layered.rs:196-201`) at compile time, plus the
    // example-specific "band fully interior to body" contract that
    // `outside_band_snaps_bit_equal` and `band_populations_all_non_empty`
    // depend on.
    const { assert!(BAND_HALF_WIDTH > 0.0) };
    const { assert!(BAND_HALF_WIDTH.is_finite()) };
    const { assert!(R_INTERFACE - BAND_HALF_WIDTH > 0.0) };
    const { assert!(R_INTERFACE + BAND_HALF_WIDTH < R_OUTER) };
}

// =============================================================================
// verify_determinism
// =============================================================================

fn verify_determinism(mesh: &SdfMeshedTetMesh) {
    let other = build_blended_mesh();
    assert!(
        mesh.equals_structurally(&other),
        "second from_sdf call drifted — III-1 determinism contract violated",
    );
    // Material cache determinism — second build's `materials()` must be
    // bit-equal under the Material-trait probe at every tet. Mirrors
    // row 8's gate.
    assert_eq!(
        mesh.materials().len(),
        other.materials().len(),
        "materials cache length drift",
    );
    let f_probe = probe_f();
    for (i, (a, b)) in mesh
        .materials()
        .iter()
        .zip(other.materials().iter())
        .enumerate()
    {
        assert_eq!(
            a.energy(&f_probe).to_bits(),
            b.energy(&f_probe).to_bits(),
            "materials cache drift at tet {i}: energy(F_probe) bit-unequal across runs",
        );
    }
    // Interface flags cache determinism — sister of IV-6's
    // `iv_6_flag_vector_is_run_to_run_deterministic` gate, surfaced at
    // the user-facing example layer for the BlendedScalarField path.
    let flags_a = mesh.interface_flags();
    let flags_b = other.interface_flags();
    assert_eq!(
        flags_a.len(),
        flags_b.len(),
        "interface_flags cache length drift",
    );
    for (i, (&a, &b)) in flags_a.iter().zip(flags_b.iter()).enumerate() {
        assert_eq!(a, b, "interface_flags drift at tet {i}: {a} vs {b}");
    }
}

// =============================================================================
// verify_counts
// =============================================================================

fn verify_counts(mesh: &SdfMeshedTetMesh, referenced: &[VertexId]) {
    assert_eq!(mesh.n_tets(), N_TETS_EXACT, "n_tets drift");
    assert_eq!(mesh.n_vertices(), N_VERTICES_EXACT, "n_vertices drift");
    assert_eq!(
        referenced.len(),
        N_REFERENCED_EXACT,
        "referenced vertex count drift",
    );
    assert!(
        referenced.len() < mesh.n_vertices(),
        "orphan-rejection invariant vacuous: referenced ({}) == n_vertices ({})",
        referenced.len(),
        mesh.n_vertices(),
    );
}

// =============================================================================
// verify_per_tet_blended_assignment_matches_smoothstep_predicate
// =============================================================================

/// HEADLINE 1 — for every tet, the mesher's centroid-sampled
/// `mesh.materials()[t]` must agree with the test-side blended
/// `expected = NH::from_lame(rec.blended_mu, rec.blended_lambda)` under
/// the Material-trait probe.
///
/// Cross-implementation gate: the test-side smoothstep arithmetic
/// (clamp + cubic Hermite + FMA-blend in [`smoothstep_weight`] +
/// [`fma_blend`]) is a bit-exact mirror of `BlendedScalarField::sample`
/// (`field/layered.rs:212-225`). Drift between the mesher's
/// centroid-sampling pass and the test-side re-derivation fires loud at
/// every tet. Same Matrix3 over-determination logic as row 8 (energy
/// alone underdetermines the `(μ, λ)` pair; first_piola fixes it via
/// `P_22 = λ ln J` then `P_11`).
fn verify_per_tet_blended_assignment_matches_smoothstep_predicate(
    mesh: &SdfMeshedTetMesh,
    records: &[TetRecord],
) {
    let f_probe = probe_f();
    let materials = mesh.materials();
    assert_eq!(
        materials.len(),
        records.len(),
        "materials() length ({}) does not match per-tet record length ({})",
        materials.len(),
        records.len(),
    );
    for rec in records {
        let observed = &materials[rec.tet_id as usize];
        let expected_nh = NeoHookean::from_lame(rec.blended_mu, rec.blended_lambda);
        // Probe via `energy` — scalar comparison surfaces the easiest
        // diagnostic when a regression hits.
        assert_relative_eq!(
            observed.energy(&f_probe),
            expected_nh.energy(&f_probe),
            epsilon = EXACT_TOL,
        );
        // Probe via `first_piola` — Matrix3 entries over-determine the
        // (μ, λ) pair (energy alone underdetermines).
        let observed_p = observed.first_piola(&f_probe);
        let expected_p = expected_nh.first_piola(&f_probe);
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(observed_p[(i, j)], expected_p[(i, j)], epsilon = EXACT_TOL,);
            }
        }
    }
}

// =============================================================================
// verify_outside_band_snaps_bit_equal
// =============================================================================

/// For tets with `phi(c) <= -BAND_HALF_WIDTH`, `mesh.materials()[t]`
/// must bit-equal `NH::from_lame(MU_INNER, LAMBDA_INNER)` (smoothstep
/// `s = 0 ⇒ w = 0` exactly ⇒ `fma(1, MU_INNER, 0 * MU_OUTER) = MU_INNER`
/// exactly via IEEE 754); for `phi(c) >= +BAND_HALF_WIDTH`, bit-equal
/// `(MU_OUTER, LAMBDA_OUTER)` (`s = 1 ⇒ w = 1 ⇒ fma(0, MU_INNER, 1 *
/// MU_OUTER) = MU_OUTER`). Verifies the BlendedScalarField "snaps
/// cleanly to 0 or 1 outside the band" contract at
/// `field/layered.rs:148-153`.
fn verify_outside_band_snaps_bit_equal(mesh: &SdfMeshedTetMesh, records: &[TetRecord]) {
    let f_probe = probe_f();
    let materials = mesh.materials();
    let inside_snap_nh = NeoHookean::from_lame(MU_INNER, LAMBDA_INNER);
    let outside_snap_nh = NeoHookean::from_lame(MU_OUTER, LAMBDA_OUTER);
    let inside_snap_e = inside_snap_nh.energy(&f_probe);
    let outside_snap_e = outside_snap_nh.energy(&f_probe);
    for rec in records {
        let observed = &materials[rec.tet_id as usize];
        match rec.bucket {
            0 => {
                assert_eq!(
                    observed.energy(&f_probe).to_bits(),
                    inside_snap_e.to_bits(),
                    "tet {} (phi = {}, bucket = inside-snap) materials energy not bit-equal to \
                     NH(MU_INNER, LAMBDA_INNER) — smoothstep s=0 bit-exact snap violated",
                    rec.tet_id,
                    rec.phi,
                );
            }
            2 => {
                assert_eq!(
                    observed.energy(&f_probe).to_bits(),
                    outside_snap_e.to_bits(),
                    "tet {} (phi = {}, bucket = outside-snap) materials energy not bit-equal to \
                     NH(MU_OUTER, LAMBDA_OUTER) — smoothstep s=1 bit-exact snap violated",
                    rec.tet_id,
                    rec.phi,
                );
            }
            _ => { /* band tets covered by anchors 4 + 6 */ }
        }
    }
}

// =============================================================================
// verify_inside_band_monotonic_mu_gradient
// =============================================================================

/// For tets with `|phi(c)| < BAND_HALF_WIDTH` (smoothstep band), sort
/// by `phi(centroid)` ascending; assert the test-side `blended_mu`
/// values are monotone non-decreasing across the sorted order.
///
/// Inventory row 9's named gate. The smoothstep `s²(3 − 2s)` is monotone
/// non-decreasing in `s ∈ [0, 1]`, `s` is monotone non-decreasing in
/// `phi`, and the FMA-blend `MU_INNER + w · (MU_OUTER − MU_INNER)` is
/// monotone non-decreasing in `w` since `MU_OUTER > MU_INNER`. Composing
/// these three monotonicities, `blended_mu` must be monotone
/// non-decreasing in `phi` — the smooth-gradient property the row's
/// headline ships. Direct verification at every band tet.
fn verify_inside_band_monotonic_mu_gradient(records: &[TetRecord]) {
    let mut band: Vec<&TetRecord> = records.iter().filter(|r| r.bucket == 1).collect();
    band.sort_by(|a, b| {
        a.phi
            .partial_cmp(&b.phi)
            .expect("phi finite by construction (centroid + R_INTERFACE both finite)")
    });
    for pair in band.windows(2) {
        let prev = pair[0];
        let next = pair[1];
        assert!(
            next.blended_mu >= prev.blended_mu,
            "blended_mu monotonicity violated between band tets {} (phi = {}, mu = {}) and {} \
             (phi = {}, mu = {}): blended_mu must be non-decreasing in phi",
            prev.tet_id,
            prev.phi,
            prev.blended_mu,
            next.tet_id,
            next.phi,
            next.blended_mu,
        );
    }
}

// =============================================================================
// verify_interface_flags_match_book_rule_per_tet
// =============================================================================

/// HEADLINE 2 — for every tet, `mesh.interface_flags()[t]` must
/// bit-equal the test-side re-derivation `(|phi(centroid)| < L_e(t))`,
/// where `L_e(t)` is the six-edge mean from [`mean_edge_length`].
///
/// First user-facing coverage of the `BlendedScalarField +
/// with_interface_sdf` composition path uncovered by IV-6
/// (`tests/interface_band_flagging.rs:53-60` explicitly notes IV-6
/// uses `LayeredScalarField`, NOT `BlendedScalarField`). Test-side
/// `mean_edge_length` is a bit-exact mirror of
/// `interface_flags_from_field` at `mesh/mod.rs:180-186` — same six
/// norms in the same order, divided by 6.0 once. `expected_flag` is
/// bit-equal to the engine's flag computation by construction on a
/// fixed toolchain.
fn verify_interface_flags_match_book_rule_per_tet(mesh: &SdfMeshedTetMesh, records: &[TetRecord]) {
    let flags = mesh.interface_flags();
    assert_eq!(
        flags.len(),
        mesh.n_tets(),
        "interface_flags length ({}) != n_tets ({})",
        flags.len(),
        mesh.n_tets(),
    );
    for rec in records {
        let observed = flags[rec.tet_id as usize];
        assert_eq!(
            observed,
            rec.expected_flag,
            "interface_flag mismatch at tet {} (phi = {}, |phi| = {}, L_e = {}): observed {} \
             vs expected {} (rule: |phi(c)| < L_e per Part 7 §02 §01)",
            rec.tet_id,
            rec.phi,
            rec.phi.abs(),
            rec.l_e,
            observed,
            rec.expected_flag,
        );
    }
}

// =============================================================================
// verify_band_populations_all_non_empty
// =============================================================================

fn verify_band_populations_all_non_empty(records: &[TetRecord]) -> ([usize; 3], usize) {
    let counts = bucket_population_counts(records);
    let flagged_count = records.iter().filter(|r| r.expected_flag).count();
    assert!(
        counts[0] > 0,
        "inside-snap bucket empty — band positioning + width yields no \
         `phi <= -BAND_HALF_WIDTH` tets; outside-band snap gate cannot fire",
    );
    assert!(
        counts[1] > 0,
        "smoothstep band bucket empty — band positioning + width yields no \
         `|phi| < BAND_HALF_WIDTH` tets; monotonicity gate has nothing to verify",
    );
    assert!(
        counts[2] > 0,
        "outside-snap bucket empty — band positioning + width yields no \
         `phi >= +BAND_HALF_WIDTH` tets; outside-band snap gate cannot fire",
    );
    assert!(
        flagged_count > 0,
        "interface_flag = true count is zero — IV-6 BlendedScalarField + \
         with_interface_sdf path produced no flagged tets, headline new feature \
         vs row 8 not exercised",
    );
    (counts, flagged_count)
}

// =============================================================================
// verify_band_populations_exact
// =============================================================================

fn verify_band_populations_exact(counts: [usize; 3], flagged_count: usize) {
    assert_eq!(
        counts[0], N_INSIDE_SNAP_EXACT,
        "inside-snap bucket count drift",
    );
    assert_eq!(
        counts[1], N_BAND_EXACT,
        "smoothstep band bucket count drift"
    );
    assert_eq!(
        counts[2], N_OUTSIDE_SNAP_EXACT,
        "outside-snap bucket count drift",
    );
    // Total partitions over the body — the 3 smoothstep buckets sum to
    // `N_TETS_EXACT` (every tet bucket-assigns into exactly one bucket).
    assert_eq!(
        counts[0] + counts[1] + counts[2],
        N_TETS_EXACT,
        "smoothstep bucket counts do not partition: sum != N_TETS_EXACT",
    );
    // IV-6 flagged count is a SEPARATE gate: the interface band
    // (`|phi| < L_e`, where `L_e(t)` is the per-tet six-edge mean and
    // varies with the BCC + stuffing edge distribution) is a different
    // rule from the smoothstep band (`|phi| < BAND_HALF_WIDTH = 0.015`,
    // a constant). Empirically `L_e(t) > BAND_HALF_WIDTH` on most tets
    // at this resolution, so flagged_count (3480) > band count (2736)
    // — overlapping but distinct populations.
    assert_eq!(
        flagged_count, N_FLAGGED_EXACT,
        "IV-6 interface-flagged tet count drift",
    );
}

// =============================================================================
// verify_zslab_visual_populations_exact
// =============================================================================

fn verify_zslab_visual_populations_exact(records: &[TetRecord]) -> [usize; 3] {
    let mut counts = [0usize; 3];
    for rec in records.iter().filter(|r| keep_for_zslab_visual(r)) {
        counts[rec.bucket] += 1;
    }
    assert!(
        counts[0] > 0,
        "z-slab inside-snap bucket empty — cf-view visual would lose the inner core color",
    );
    assert!(
        counts[1] > 0,
        "z-slab band bucket empty — cf-view visual would lose the smooth-gradient ring",
    );
    assert!(
        counts[2] > 0,
        "z-slab outside-snap bucket empty — cf-view visual would lose the outer skin color",
    );
    assert_eq!(
        counts[0], N_INSIDE_SNAP_ZSLAB_EXACT,
        "z-slab inside-snap bucket count drift",
    );
    assert_eq!(
        counts[1], N_BAND_ZSLAB_EXACT,
        "z-slab band bucket count drift",
    );
    assert_eq!(
        counts[2], N_OUTSIDE_SNAP_ZSLAB_EXACT,
        "z-slab outside-snap bucket count drift",
    );
    counts
}

// =============================================================================
// Helpers
// =============================================================================

fn bucket_population_counts(records: &[TetRecord]) -> [usize; 3] {
    let mut counts = [0usize; 3];
    for rec in records {
        counts[rec.bucket] += 1;
    }
    counts
}

fn probe_f() -> Matrix3<f64> {
    Matrix3::from_diagonal_element(1.0)
        .map_with_location(|i, j, x| if i == 0 && j == 0 { PROBE_LAMBDA } else { x })
}

/// Keep centroids in the `|z| < ZSLAB_HALF_THICKNESS` slab for cf-view
/// rendering — projects to a thin disk in 3D, reading as concentric
/// color regions (inside-snap solid disk, band gradient ring,
/// outside-snap outer ring) on the z=0 plane.
///
/// The correctness anchors (1-9) run over all 6768 body tets; only the
/// visual PLY emit + anchor 10 (`verify_zslab_visual_populations_exact`)
/// operate on the filtered slab subset. Mirrors hollow-shell-sdf row 2's
/// z=0 slice precedent and row 8's z-slab cut for cross-section cf-view
/// visualization (rationale: a thin slab in 3D projects unambiguously
/// to a 2D disk pattern at any orbit angle, where a 3D wedge requires
/// the user to face the cut planes).
fn keep_for_zslab_visual(rec: &TetRecord) -> bool {
    rec.centroid.z.abs() < ZSLAB_HALF_THICKNESS
}

// =============================================================================
// PLY emit — per-tet centroid point cloud + 3 per-vertex scalars
// =============================================================================

/// Vertices = per-tet centroids in the `|z| < ZSLAB_HALF_THICKNESS`
/// slab cut (see [`keep_for_zslab_visual`]); faces empty (point cloud);
/// three per-vertex scalars cast as f32:
///
/// - `interface_flag` (categorical 0.0 / 1.0; cf-view auto-detects
///   integer-valued + 2 unique → tab10 binary highlight of the IV-6
///   flagged band)
/// - `material_mu` (continuous; cf-view auto-detects all-positive +
///   continuous → viridis sequential — the physical readout)
/// - `smoothstep_weight` (continuous ∈ [0, 1]; cf-view auto-detects
///   continuous → viridis sequential — the cubic Hermite kernel
///   mechanism)
///
/// cf-view's Q5 colormap heuristic detects each scalar's distribution
/// independently. Default-pick is alphabetical-first → `interface_flag`
/// shows on launch (binary highlight makes the IV-6 flagged band
/// loud); user dropdowns to `material_mu` (smooth gradient ring) or
/// `smoothstep_weight` (the underlying cubic kernel directly).
fn save_blended_ply(records: &[TetRecord], path: &Path) -> Result<()> {
    let kept: Vec<&TetRecord> = records
        .iter()
        .filter(|r| keep_for_zslab_visual(r))
        .collect();
    let vertices: Vec<Point3<f64>> = kept
        .iter()
        .map(|r| Point3::new(r.centroid.x, r.centroid.y, r.centroid.z))
        .collect();
    let faces: Vec<[u32; 3]> = Vec::new();
    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut attributed = AttributedMesh::new(geometry);
    let interface_flag: Vec<f32> = kept
        .iter()
        .map(|r| if r.expected_flag { 1.0 } else { 0.0 })
        .collect();
    let material_mu: Vec<f32> = kept.iter().map(|r| r.blended_mu as f32).collect();
    let smoothstep_weight_attr: Vec<f32> = kept.iter().map(|r| r.weight as f32).collect();
    attributed.insert_extra("interface_flag", interface_flag)?;
    attributed.insert_extra("material_mu", material_mu)?;
    attributed.insert_extra("smoothstep_weight", smoothstep_weight_attr)?;
    save_ply_attributed(&attributed, path, true)?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

// Single museum-plaque summary block printed at the end of `main()`;
// breaking it into multiple helpers per anchor / per region would add
// indirection without information per the README's "stdout summary
// covers the same numbers in human-readable form" framing.
#[allow(clippy::too_many_lines)]
fn print_summary(
    mesh: &SdfMeshedTetMesh,
    referenced: &[VertexId],
    counts: [usize; 3],
    flagged_count: usize,
    zslab_counts: [usize; 3],
    path: &Path,
) {
    let n_orphans = mesh.n_vertices() - referenced.len();
    println!("==== blended-scalar-field ====");
    println!();
    println!("input  : SphereSdf {{ radius: {R_OUTER} }}  (solid body)");
    println!("         MeshingHints {{");
    println!("           bbox:           [-{BBOX_HALF_EXTENT}, {BBOX_HALF_EXTENT}]³,");
    println!("           cell_size:      {CELL_SIZE},");
    println!(
        "           material_field: MaterialField::from_fields(μ_field, λ_field).with_interface_sdf(...)"
    );
    println!("         }}");
    println!();
    println!(
        "MaterialField  : 2-region BlendedScalarField × 2 (μ, λ) keyed on interior \
         SphereSdf {{ R_INTERFACE = {R_INTERFACE} }};"
    );
    println!(
        "                 cubic Hermite smoothstep s²(3 − 2s) over band of half-width \
         {BAND_HALF_WIDTH} m around phi = ‖p‖ − R_INTERFACE = 0;"
    );
    println!(
        "                 SAME interior sphere attached via with_interface_sdf for IV-6 \
         |phi(c)| < L_e flagging."
    );
    println!(
        "                 inside snap   ‖p‖ ≤ {snap_in:.3}                  (μ = {MU_INNER:e}, λ = {LAMBDA_INNER:e})",
        snap_in = R_INTERFACE - BAND_HALF_WIDTH,
    );
    println!(
        "                 band          {snap_in:.3} < ‖p‖ < {snap_out:.3}      (smoothstep blends μ, λ across band)",
        snap_in = R_INTERFACE - BAND_HALF_WIDTH,
        snap_out = R_INTERFACE + BAND_HALF_WIDTH,
    );
    println!(
        "                 outside snap  ‖p‖ ≥ {snap_out:.3}                  (μ = {MU_OUTER:e}, λ = {LAMBDA_OUTER:e})",
        snap_out = R_INTERFACE + BAND_HALF_WIDTH,
    );
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!(
        "  band_half_width_validity              : BAND_HALF_WIDTH > 0 + finite + band fully interior to body"
    );
    println!(
        "  determinism                           : 2nd from_sdf call equals_structurally + materials + interface_flags bit-equal"
    );
    println!(
        "  counts                                : n_tets / n_vertices / referenced exact-pin (III-1 contract)"
    );
    println!(
        "  per_tet_blended_assignment            : mesh.materials()[t] vs expected smoothstep-blended NH bit-equal (HEADLINE 1)"
    );
    println!(
        "  outside_band_snaps_bit_equal          : phi ≤ -band ⇒ NH(MU_INNER, LAMBDA_INNER); phi ≥ +band ⇒ NH(MU_OUTER, LAMBDA_OUTER)"
    );
    println!(
        "  inside_band_monotonic_mu_gradient     : sorted by phi, blended_mu monotone non-decreasing across band"
    );
    println!(
        "  interface_flags_match_book_rule       : per-tet interface_flag bit-equal to (|phi| < L_e) re-derived test-side (HEADLINE 2)"
    );
    println!(
        "  band_populations_all_non_empty        : inside-snap / band / outside-snap / interface-flagged all > 0"
    );
    println!(
        "  band_populations_exact                : per-bucket smoothstep counts + IV-6 flagged count exact-pinned"
    );
    println!(
        "  zslab_visual_populations_exact        : per-bucket z-slab counts exact-pinned (cf-view artifact)"
    );
    println!();
    println!("Mesh:");
    println!("  n_tets                     : {:>7}", mesh.n_tets());
    println!("  n_vertices                 : {:>7}", mesh.n_vertices());
    println!("  referenced (≤ n_vertices)  : {:>7}", referenced.len());
    println!("  orphans (n_vertices − ref) : {n_orphans:>7}");
    println!();
    println!(
        "Per-bucket smoothstep partition (over all {} body tets):",
        mesh.n_tets()
    );
    println!("  inside-snap   (phi ≤ -band)  : {:>7}", counts[0]);
    println!("  band          (|phi| < band) : {:>7}", counts[1]);
    println!("  outside-snap  (phi ≥ +band)  : {:>7}", counts[2]);
    println!(
        "  total                        : {:>7}  (== n_tets)",
        counts[0] + counts[1] + counts[2]
    );
    println!();
    println!("IV-6 interface flag (|phi(c)| < L_e, L_e = mean of 6 edge lengths):");
    println!("  flagged (true)            : {flagged_count:>7}");
    println!(
        "  unflagged (false)         : {:>7}",
        mesh.n_tets() - flagged_count
    );
    println!();
    let zslab_total = zslab_counts[0] + zslab_counts[1] + zslab_counts[2];
    println!("Z-slab cut PLY (cf-view artifact, |z| < {ZSLAB_HALF_THICKNESS} m):");
    println!("  inside-snap                : {:>7}", zslab_counts[0]);
    println!("  band                       : {:>7}", zslab_counts[1]);
    println!("  outside-snap               : {:>7}", zslab_counts[2]);
    println!(
        "  total                      : {zslab_total:>7}  (≈ N_TETS · slab_thickness / body_diameter)"
    );
    println!();
    println!("PLY    : {}", path.display());
    println!(
        "         vertices-only point cloud ({zslab_total} centroids — z-slab of the {} \
         body tets) + 3 per-vertex scalars:",
        mesh.n_tets(),
    );
    println!(
        "           extras[\"interface_flag\"]   — categorical 0.0/1.0  (IV-6 |phi(c)| < L_e flag)"
    );
    println!("           extras[\"material_mu\"]      — continuous Pa        (physical μ readout)");
    println!(
        "           extras[\"smoothstep_weight\"] — continuous [0, 1]    (cubic Hermite kernel)"
    );
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!(
        "         alphabetical-first scalar pick lands on `interface_flag` (categorical → tab10);"
    );
    println!("         scalar dropdown switches to `material_mu` (continuous → viridis) for the");
    println!("         physical gradient or `smoothstep_weight` (continuous → viridis) for the");
    println!("         underlying cubic Hermite kernel directly. The thin z-slab projects to a");
    println!("         2D disk in cf-view, reading as concentric color regions on the z=0 plane.");
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let mesh = build_blended_mesh();

    verify_band_half_width_validity();
    verify_determinism(&mesh);

    let referenced = referenced_vertices(&mesh);
    verify_counts(&mesh, &referenced);

    let records = extract_centroid_records(&mesh);

    verify_per_tet_blended_assignment_matches_smoothstep_predicate(&mesh, &records);
    verify_outside_band_snaps_bit_equal(&mesh, &records);
    verify_inside_band_monotonic_mu_gradient(&records);
    verify_interface_flags_match_book_rule_per_tet(&mesh, &records);
    let (counts, flagged_count) = verify_band_populations_all_non_empty(&records);
    verify_band_populations_exact(counts, flagged_count);
    let zslab_counts = verify_zslab_visual_populations_exact(&records);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("material_blend.ply");
    save_blended_ply(&records, &out_path)?;

    print_summary(
        &mesh,
        &referenced,
        counts,
        flagged_count,
        zslab_counts,
        &out_path,
    );

    Ok(())
}
