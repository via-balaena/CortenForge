//! blended — `BlendedScalarField` smooth cubic-Hermite-smoothstep
//! transition between two uniform Lamé regions (the "stiff skin over soft
//! core" canonical idiom from `material_field.rs:130`), with the same SDF
//! used both inside the field's blend AND attached via
//! `MaterialField::with_interface_sdf` to populate `Mesh::interface_flags`.
//!
//! Per inventory Tier 3 row 9 — the canonical Phase 4 composition example
//! per Part 7 §02 §01. A solid `SphereSdf{ radius: R_OUTER }` body (reused from row 8)
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
//! Per-tet CORRECTNESS of the Blended composition — that `mesh.materials()`
//! carries the smoothstep-blended `(μ, λ)` and `mesh.interface_flags()`
//! obeys the `|φ(x_c)| < L_e` book rule — is validated in the LIBRARY
//! (`sim/L0/soft/tests/blended_material_composition.rs`, mirror-free vs the
//! smoothstep spec / an independent `L_e`). This example is the pipeline
//! DEMONSTRATION: it builds the composition, spot-checks it against
//! independent oracles reading the real library outputs, and emits the
//! cf-view artifact. Anchor groups (all assertions exit-0 on success):
//!
//! - **Band-half-width validity** — `BAND_HALF_WIDTH > 0` and finite +
//!   band fully interior (`R_INTERFACE − BAND_HALF_WIDTH > 0`,
//!   `R_INTERFACE + BAND_HALF_WIDTH < R_OUTER`); `const { assert!(...) }`
//!   compile-time enforcement on the geometry constants.
//! - **Determinism** — second `from_sdf` call produces an
//!   `equals_structurally` mesh + materials `(μ, λ)` bit-equal (via the
//!   public accessors) + `interface_flags()` cache bit-equal.
//! - **Counts** — `n_tets`, `n_vertices`, `referenced_vertices.len()`
//!   exact-pinned per III-1 contract (same scene + resolution as
//!   row 3 / row 8); the deterministic mesh the pipeline demonstrates.
//!   Finer per-bucket / z-slab counts are deliberately NOT pinned.
//! - **Snap and grade materials** — reading `mesh.materials()` via the
//!   public `NeoHookean::mu()` / `lambda()`: every inside-snap tet
//!   (`phi ≤ -BAND_HALF_WIDTH`) is `(MU_INNER, LAMBDA_INNER)` bit-exact and
//!   every outside-snap tet (`phi ≥ +BAND_HALF_WIDTH`) is `(MU_OUTER,
//!   LAMBDA_OUTER)` (oracle = endpoint constants), AND at least one band
//!   tet carries a μ strictly *between* the endpoints (oracle = strict
//!   interval — the graded-not-stepped distinction from
//!   `LayeredScalarField`). No smoothstep re-derivation.
//! - **Band populations non-empty** — inside-snap / band / outside-snap
//!   each ≥ 1 tet, AND the mesher's `interface_flags()` are mixed (≥ 1
//!   true so the `with_interface_sdf` feature fires, ≥ 1 false so the rule
//!   discriminates). Structural (softened from the former exact pins).
//! - **Z-slab visual populations non-empty** — each smoothstep bucket
//!   contributes ≥ 1 centroid to the `|z| < cell_size/2` slab so the
//!   cf-view projected disk reads as concentric color regions. Exact
//!   per-bucket slab counts are NOT pinned (mesher-version artifacts).

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
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use sim_soft::{
    Aabb3, BlendedScalarField, ConstantField, Field, LAYERED_SPHERE_BBOX_HALF_EXTENT,
    LAYERED_SPHERE_R_OUTER, MaterialField, Mesh, MeshingHints, SdfMeshedTetMesh, SphereSdf, TetId,
    Vec3, VertexId, referenced_vertices,
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
/// wide — well-resolved by the mesh (a few thousand of the 6768 body
/// tets land in the band at this resolution), and not so wide that the
/// inside-snap region collapses. Strictly interior to the body:
/// `R_INTERFACE − BAND_HALF_WIDTH = 0.055 > 0` and `R_INTERFACE +
/// BAND_HALF_WIDTH = 0.085 < R_OUTER = 0.10`. Note: distinct from the
/// interface-flag `L_e` band — `L_e(t)` varies per tet with the BCC +
/// stuffing edge distribution and is empirically larger than this on
/// most tets at this resolution, so the interface-flagged population
/// exceeds the smoothstep band population.
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

// ── Exact-pinned mesh counts (III-1 determinism contract) ────────────────
//
// Same scene topology + cell_size as row 3 / row 8 (`SphereSdf{ R_OUTER
// = 0.10 }`, bbox `[-0.12, 0.12]³`, cell_size 0.02). Material-field
// carry doesn't affect mesh topology (BCC + stuffing runs before
// `materials_from_field` populates the cache) so n_tets / n_vertices /
// referenced inherit row 3 / row 8 exactly. These top-line counts pin
// the deterministic mesh the pipeline validator demonstrates; the
// finer per-bucket / z-slab tet counts are deliberately NOT pinned —
// they are mesher-version artifacts (softened to structural non-empty +
// total-partition guards), and per-tet material/flag correctness lives
// in the lib (`tests/blended_material_composition.rs`).

/// Total emitted tet count. Identical to row 3 / row 8 — material-field
/// composition does not shift mesh topology.
const N_TETS_EXACT: usize = 6768;

/// Total mesh vertex count, including orphan BCC lattice corners.
/// Identical to row 3 / row 8.
const N_VERTICES_EXACT: usize = 4634;

/// Vertices referenced by at least one tet. Identical to row 3 / row 8.
const N_REFERENCED_EXACT: usize = 1483;

/// Half-thickness of the z-slab visual cut. `cell_size / 2 = 0.01 m` —
/// row 8's banked z-slab visual-cut pattern reused verbatim for cf-view
/// rendering against dense per-tet centroid clouds (see
/// `project_cf_viewer_dense_point_cloud_gap.md`).
const ZSLAB_HALF_THICKNESS: f64 = CELL_SIZE / 2.0;

// =============================================================================
// Smoothstep predicate (visualization + bucket classification only)
// =============================================================================
//
// `smoothstep_weight` drives the PLY `smoothstep_weight` scalar (the cubic
// Hermite kernel, for cf-view); `smoothstep_bucket` classifies each tet
// into inside-snap / band / outside-snap for the population guards and the
// snap-vs-grade material check. Per-tet material CORRECTNESS is validated
// in the lib (`tests/blended_material_composition.rs`), not by re-deriving
// `BlendedScalarField::sample`'s arithmetic here.

/// Cubic Hermite smoothstep `outside_weight = s² (3 − 2s)` with
/// `s = clamp((phi + band) / (2 · band), 0, 1)`. Emitted as the PLY
/// `smoothstep_weight` scalar so cf-view renders the kernel directly. At
/// the three reference points `phi ≤ -BAND_HALF_WIDTH ⇒ w = 0`,
/// `phi == 0 ⇒ w = 0.5`, `phi ≥ +BAND_HALF_WIDTH ⇒ w = 1` (the cubic
/// kernel is C¹ at both band edges).
fn smoothstep_weight(phi: f64) -> f64 {
    let s = ((phi + BAND_HALF_WIDTH) / (2.0 * BAND_HALF_WIDTH)).clamp(0.0, 1.0);
    s * s * 2.0_f64.mul_add(-s, 3.0)
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
    /// `phi = ||centroid|| - R_INTERFACE` (the same signed distance the
    /// mesher's interface SDF evaluates; drives the bucket + summary).
    phi: f64,
    /// Cubic Hermite smoothstep weight `s²(3 − 2s)` from
    /// [`smoothstep_weight`] — the PLY `smoothstep_weight` scalar only.
    weight: f64,
    /// Smoothstep bucket label (0 / 1 / 2) from [`smoothstep_bucket`].
    bucket: usize,
    /// Mesher-assigned `μ` read straight off `mesh.materials()[t].mu()`
    /// — the real library output (snap/grade check + PLY `material_mu`).
    mu: f64,
    /// Mesher-assigned `λ` read straight off `mesh.materials()[t].lambda()`.
    lambda: f64,
    /// Mesher-populated interface flag read straight off
    /// `mesh.interface_flags()[t]` — the real library output (flagged
    /// population guard + PLY `interface_flag`). The per-tet `|phi| < L_e`
    /// book-rule correctness is validated in the lib
    /// (`tests/blended_material_composition.rs`), not re-derived here.
    flag: bool,
}

/// Walk every tet, pairing its centroid + phi + smoothstep weight/bucket
/// with the mesher's own per-tet outputs (`materials()`,
/// `interface_flags()`). The centroid IS the canonical evaluation point
/// per Part 7 §02 §00 — the same point `materials_from_field` and
/// `interface_flags_from_field` sample at — so `phi`/`bucket` line up with
/// the mesher's material assignment for the snap/grade check without
/// re-implementing the blend arithmetic.
fn extract_centroid_records(mesh: &SdfMeshedTetMesh) -> Vec<TetRecord> {
    let positions = mesh.positions();
    let materials = mesh.materials();
    let flags = mesh.interface_flags();
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
        let material = &materials[tet_id as usize];
        records.push(TetRecord {
            tet_id,
            centroid,
            phi,
            weight: smoothstep_weight(phi),
            bucket: smoothstep_bucket(phi),
            mu: material.mu(),
            lambda: material.lambda(),
            flag: flags[tet_id as usize],
        });
    }
    records
}

// =============================================================================
// verify_band_half_width_validity
// =============================================================================

const fn verify_band_half_width_validity() {
    // Compile-time scene preconditions. `BAND_HALF_WIDTH > 0` + finite
    // guards the `BlendedScalarField::new` invariant at the example layer;
    // the band-fully-interior pair (`R_INTERFACE ∓ BAND_HALF_WIDTH` inside
    // the body) is the example-specific contract that
    // `verify_snap_and_grade_materials` and
    // `verify_band_populations_non_empty` depend on — if the band spilled
    // past the body surface the inside-snap / outside-snap buckets could
    // empty. The `const { ... }` blocks panic at compile time on a
    // regression, anchoring the dependency loud and user-facing.
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
    // bit-equal (Lamé pairs read via the public accessors) at every tet.
    assert_eq!(
        mesh.materials().len(),
        other.materials().len(),
        "materials cache length drift",
    );
    for (i, (a, b)) in mesh
        .materials()
        .iter()
        .zip(other.materials().iter())
        .enumerate()
    {
        assert_eq!(
            (a.mu().to_bits(), a.lambda().to_bits()),
            (b.mu().to_bits(), b.lambda().to_bits()),
            "materials cache drift at tet {i}: (μ, λ) bit-unequal across runs",
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
// verify_snap_and_grade_materials
// =============================================================================

/// Reads the mesher's own `mesh.materials()` (via the public
/// `NeoHookean::mu()` / `lambda()` accessors carried on each record) and
/// checks two spec properties of the blend against INDEPENDENT oracles —
/// never a re-derivation of `BlendedScalarField::sample`'s FMA arithmetic.
/// The full per-tet `materials()` correctness for the Blended composition
/// is owned by the lib (`tests/blended_material_composition.rs`); this is
/// the example-layer demonstration that the pipeline carries the blend:
///
/// - **Snap** — every inside-snap tet (`phi ≤ -BAND_HALF_WIDTH`, so the
///   smoothstep weight is `0` exactly) carries `(MU_INNER, LAMBDA_INNER)`
///   bit-exactly; every outside-snap tet (`phi ≥ +BAND_HALF_WIDTH`, weight
///   `1`) carries `(MU_OUTER, LAMBDA_OUTER)`. Oracle = the endpoint
///   constants.
/// - **Grade** — at least one band tet carries a μ strictly *between* the
///   endpoints, i.e. the blend is a smooth gradient, not a sharp step (the
///   behavioural distinction from `LayeredScalarField`). Oracle = the
///   strict interval `(MU_INNER, MU_OUTER)`.
fn verify_snap_and_grade_materials(records: &[TetRecord]) {
    let mut graded = false;
    for rec in records {
        match rec.bucket {
            0 => assert_eq!(
                (rec.mu.to_bits(), rec.lambda.to_bits()),
                (MU_INNER.to_bits(), LAMBDA_INNER.to_bits()),
                "tet {} (phi = {}, inside-snap) materials not bit-equal to (MU_INNER, \
                 LAMBDA_INNER) — smoothstep s=0 snap violated",
                rec.tet_id,
                rec.phi,
            ),
            2 => assert_eq!(
                (rec.mu.to_bits(), rec.lambda.to_bits()),
                (MU_OUTER.to_bits(), LAMBDA_OUTER.to_bits()),
                "tet {} (phi = {}, outside-snap) materials not bit-equal to (MU_OUTER, \
                 LAMBDA_OUTER) — smoothstep s=1 snap violated",
                rec.tet_id,
                rec.phi,
            ),
            _ => {
                if rec.mu > MU_INNER && rec.mu < MU_OUTER {
                    graded = true;
                }
            }
        }
    }
    assert!(
        graded,
        "no band tet carries a strictly-intermediate μ ∈ (MU_INNER, MU_OUTER); the mesher's \
         BlendedScalarField materials collapsed to a step instead of grading",
    );
}

// =============================================================================
// verify_band_populations_non_empty
// =============================================================================

/// Structural population guards (softened from the former exact per-bucket
/// pins, which were mesher-version artifacts). Each smoothstep bucket must
/// be non-empty — so [`verify_snap_and_grade_materials`] has inside-snap,
/// band, and outside-snap tets to see — and the mesher-populated interface
/// flags must be genuinely *mixed*: at least one `true` (the
/// `BlendedScalarField + with_interface_sdf` feature actually fires) and at
/// least one `false` (the `|phi| < L_e` rule discriminates, isn't trivially
/// all-flagged). `flagged_count` is read straight off the real
/// `mesh.interface_flags()` carried on each record — not re-derived.
/// Returns the bucket counts + flagged count for the summary.
fn verify_band_populations_non_empty(records: &[TetRecord]) -> ([usize; 3], usize) {
    let counts = bucket_population_counts(records);
    let flagged_count = records.iter().filter(|r| r.flag).count();
    assert!(
        counts[0] > 0,
        "inside-snap bucket empty — no `phi <= -BAND_HALF_WIDTH` tets",
    );
    assert!(
        counts[1] > 0,
        "smoothstep band bucket empty — no `|phi| < BAND_HALF_WIDTH` tets",
    );
    assert!(
        counts[2] > 0,
        "outside-snap bucket empty — no `phi >= +BAND_HALF_WIDTH` tets",
    );
    assert!(
        flagged_count > 0,
        "no interface_flag = true tets — BlendedScalarField + with_interface_sdf produced no \
         flagged band",
    );
    assert!(
        flagged_count < records.len(),
        "every tet interface-flagged — the |phi| < L_e rule is not discriminating",
    );
    (counts, flagged_count)
}

// =============================================================================
// verify_zslab_visual_populations_non_empty
// =============================================================================

/// Visual-pedagogy guard for the cf-view z-slab PLY: each smoothstep
/// bucket must contribute at least one centroid to the `|z| < cell_size/2`
/// slab so the projected disk shows the inside-snap core, the gradient
/// ring, and the outside-snap skin. Exact per-bucket slab counts are NOT
/// pinned — they are mesher-version artifacts.
fn verify_zslab_visual_populations_non_empty(records: &[TetRecord]) -> [usize; 3] {
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

/// Keep centroids in the `|z| < ZSLAB_HALF_THICKNESS` slab for cf-view
/// rendering — projects to a thin disk in 3D, reading as concentric
/// color regions (inside-snap solid disk, band gradient ring,
/// outside-snap outer ring) on the z=0 plane.
///
/// The correctness anchors run over all 6768 body tets; only the visual
/// PLY emit + [`verify_zslab_visual_populations_non_empty`] operate on the
/// filtered slab subset. Mirrors `hollow_shell` row 2's
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
        .map(|r| if r.flag { 1.0 } else { 0.0 })
        .collect();
    let material_mu: Vec<f32> = kept.iter().map(|r| r.mu as f32).collect();
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
    println!("==== blended ====");
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
        "  band_half_width_validity          : BAND_HALF_WIDTH > 0 + finite + band fully interior to body"
    );
    println!(
        "  determinism                       : 2nd from_sdf call equals_structurally + materials (μ, λ) + interface_flags bit-equal"
    );
    println!(
        "  counts                            : n_tets / n_vertices / referenced exact-pin (III-1 contract)"
    );
    println!(
        "  snap_and_grade_materials          : mesh.materials() snap NH(MU_INNER)/NH(MU_OUTER) outside band + ∃ strictly-graded band tet"
    );
    println!(
        "  band_populations_non_empty        : inside-snap / band / outside-snap non-empty + interface flags mixed (some true, some false)"
    );
    println!(
        "  zslab_visual_populations_non_empty: each z-slab bucket non-empty (cf-view artifact)"
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

pub fn run() -> Result<()> {
    let mesh = build_blended_mesh();

    verify_band_half_width_validity();
    verify_determinism(&mesh);

    let referenced = referenced_vertices(&mesh);
    verify_counts(&mesh, &referenced);

    let records = extract_centroid_records(&mesh);

    verify_snap_and_grade_materials(&records);
    let (counts, flagged_count) = verify_band_populations_non_empty(&records);
    let zslab_counts = verify_zslab_visual_populations_non_empty(&records);

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
