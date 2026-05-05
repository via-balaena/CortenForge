//! layered-scalar-field — `LayeredScalarField` over a 3-shell concentric
//! `SphereSdf` partition.
//!
//! Per inventory Tier 3 row 8 — the canonical multi-material spatial-field
//! example. A solid `SphereSdf{ radius: R_OUTER }` body partitions into 3
//! concentric shells via two `LayeredScalarField`s (one per Lamé parameter)
//! threaded through `MaterialField::from_fields`; `SdfMeshedTetMesh::from_sdf`
//! samples each per-tet `(μ, λ)` at the centroid and caches it in
//! `Mesh::materials()`. The headline cf-view artifact is a per-tet centroid
//! point cloud carrying integer `material_layer_id ∈ {0, 1, 2}` — cf-view's
//! categorical-colormap heuristic (integer-valued + < 16 unique → tab10)
//! renders three sharp colors, visualizing the sharp-boundary spatial
//! composition `LayeredScalarField` ships.
//!
//! Geometry constants reuse `LAYERED_SPHERE_R_OUTER` / `_OUTER_INNER` /
//! `_INNER_OUTER` / `BBOX_HALF_EXTENT` from sim-soft's re-exports — the
//! IV-5 silicone-device geometry minus the cavity (row 11 will reuse the
//! same constants plus `R_CAVITY` for its hollow form). Lamé pairs match
//! `sdf_material_tagging.rs`'s IV-4 deviation (`0.5× / 1× / 2×` IV-1
//! baseline; all in `λ = 4μ` ⇒ `ν = 0.4` compressible regime), giving
//! three distinct shells so a shell-swap bug surfaces in the value-
//! correctness gate. `cell_size = 0.02` matches III-1 / IV-4 h/2 canonical.
//!
//! Anchor groups (all assertions exit-0 on success):
//!
//! - **Determinism** — second `from_sdf` call produces an
//!   `equals_structurally` mesh.
//! - **Counts** — `n_tets`, `n_vertices`, `referenced_vertices.len()`
//!   exact-pinned per III-1 contract (same scene + resolution as row 3).
//! - **Layered-field thresholds strictly monotone** — the
//!   `LayeredScalarField` constructor's invariant re-asserted (loud,
//!   user-facing). `R_INNER_OUTER < R_OUTER_INNER < R_OUTER`; phi
//!   thresholds `[-0.04, -0.02]` strict-ascending.
//! - **Per-tet layer assignment matches SDF predicate** — for every tet,
//!   `mesh.materials()[t]` (probed via `energy(F_probe)` +
//!   `first_piola(F_probe)` bit-equal at `F_probe = diag(1.2, 1, 1)`)
//!   matches `expected[shell_at(centroid)]`. The headline cross-impl
//!   gate: the test-side `shell_at` reimplements `LayeredScalarField`'s
//!   `partition_point(|&t| t <= phi)` rule, so centroid-sampled
//!   `materials()` and re-derived `expected[shell_at(...)]` MUST agree
//!   on every tet by construction. Drift between mesher partition pass
//!   and test re-derivation fires loud.
//! - **Shell populations non-empty** — each layer (0, 1, 2) has ≥ 1 tet
//!   at this resolution. Sanity guard against a degenerate scene
//!   (e.g., a middle annulus too thin to admit any centroid).
//! - **Shell populations exact-pinned** — captured per-shell tet counts
//!   under the III-1 determinism contract; mirrors `N_*_EXACT` shape.
//! - **Layer-ID categorical count** — exactly 3 unique IDs in the
//!   per-tet vector. Gates cf-view's `< 16 unique` categorical-colormap
//!   detection per VIEWER_DESIGN.md Q5; if a regression introduced a
//!   fourth bucket, cf-view would still pick categorical (4 < 16) but
//!   the example's pedagogy ("3-shell sharp-boundary composition")
//!   would silently slip.
//! - **Interface flags all-false** — `mesh.interface_flags()` is
//!   `[false; n_tets]` because `MaterialField::with_interface_sdf` was
//!   NOT called. Pedagogically clarifies the interface flag is the
//!   `BlendedScalarField` smooth-transition band rule (`|φ(x_c)| < L_e`
//!   per Part 7 §02 §01), not relevant to `LayeredScalarField`'s sharp
//!   boundaries.
//! - **Centroid radii within body** — every per-tet centroid satisfies
//!   `‖centroid‖ < R_OUTER`. Solid-body sanity check; the mesher only
//!   emits tets inside the SDF, so centroids must be strictly interior.
//! - **Z-slab visual populations exact-pinned** — captured per-shell tet
//!   counts in the `|z| < cell_size/2` slab cut that drives the cf-view
//!   PLY. Visual-pedagogy guard: each layer must have ≥ 1 z-slab
//!   centroid for the three concentric-ring cf-view rendering to work.

// PLY field-data is single-precision on disk; converting f64 layer IDs
// (0.0 / 1.0 / 2.0) to f32 for `extras["material_layer_id"]` is intrinsic
// to the PLY format. Same precedent as rows 1+2+3.
#![allow(clippy::cast_possible_truncation)]
// usize as f64 / usize as i64 for layer-id encoding + count summaries.
// Per-shell counts at h/2 stay in the few-thousands range, well below
// f64 mantissa exact-representation (52 bits).
#![allow(clippy::cast_precision_loss)]
// `from_sdf(...).expect(...)` — the canonical layered-sphere scene either
// succeeds by construction or surfaces a regression worth investigating.
// Same precedent as row 3 / `sdf_material_tagging.rs`.
#![allow(clippy::expect_used)]
// `as TetId` cast on `mesh.n_tets()` (≤ 6768 here, ≪ u32::MAX) — the
// standard Mesh-trait API tax mirrored across the workspace.
#![allow(clippy::cast_possible_wrap)]
// `doc_markdown` flags `VIEWER_DESIGN.md` (a literal filename, not a
// code identifier) inside the module docstring. Same allowance as
// rows 5 + 6 for Unicode math notation.
#![allow(clippy::doc_markdown)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use nalgebra::Matrix3;
use sim_soft::{
    Aabb3, Field, LAYERED_SPHERE_BBOX_HALF_EXTENT, LAYERED_SPHERE_R_INNER_OUTER,
    LAYERED_SPHERE_R_OUTER, LAYERED_SPHERE_R_OUTER_INNER, LayeredScalarField, Material,
    MaterialField, Mesh, MeshingHints, NeoHookean, SdfMeshedTetMesh, SphereSdf, TetId, Vec3,
    VertexId, referenced_vertices,
};

// =============================================================================
// Constants
// =============================================================================

/// Outer radius (m) — body's exterior surface. Reused from
/// `LAYERED_SPHERE_R_OUTER` for cross-row alignment with row 11.
const R_OUTER: f64 = LAYERED_SPHERE_R_OUTER;

/// Boundary radius between middle and outer shells (m).
/// `R_OUTER_INNER < R_OUTER` ⇒ shell `[R_OUTER_INNER, R_OUTER]` = outer.
const R_OUTER_INNER: f64 = LAYERED_SPHERE_R_OUTER_INNER;

/// Boundary radius between inner and middle shells (m).
/// `R_INNER_OUTER < R_OUTER_INNER` ⇒ shell `[R_INNER_OUTER, R_OUTER_INNER)`
/// = middle (closed-left, open-right per the partition rule —
/// `‖p‖ = R_OUTER_INNER` lands in outer, not middle). Inner shell is
/// `‖p‖ < R_INNER_OUTER` (down to origin — row 8 is solid; the cavity is
/// row 11's territory).
const R_INNER_OUTER: f64 = LAYERED_SPHERE_R_INNER_OUTER;

/// Bounding-box half-extent (m). Matches the III-1 / IV-4 canonical
/// `bbox_half_extent / cell_size = 6.0` ratio at `cell_size = h/2 = 0.02`.
const BBOX_HALF_EXTENT: f64 = LAYERED_SPHERE_BBOX_HALF_EXTENT;

/// BCC lattice spacing (m) — III-1 / IV-4 h/2 canonical.
const CELL_SIZE: f64 = 0.02;

/// Inner-shell threshold on `phi = ‖p‖ − R_OUTER`. `phi < -0.04` ⇒
/// `‖p‖ < R_INNER_OUTER` ⇒ inner shell (strict less than per the
/// `partition_point(|&t| t <= phi)` rule — at `phi == -0.04`, the point
/// lands in the OUTER shell relative to this threshold = middle).
const PHI_INNER_THRESHOLD: f64 = R_INNER_OUTER - R_OUTER;

/// Middle-shell threshold on `phi`. `-0.04 ≤ phi < -0.02` ⇒
/// `R_INNER_OUTER ≤ ‖p‖ < R_OUTER_INNER` ⇒ middle shell.
/// `phi ≥ -0.02` ⇒ outer shell.
const PHI_MIDDLE_THRESHOLD: f64 = R_OUTER_INNER - R_OUTER;

// ── Lamé pairs per shell ─────────────────────────────────────────────────
//
// Three distinct values mirroring `sdf_material_tagging.rs` IV-4 deviation
// (Decision J's outer = inner symmetry would mask shell-swap bugs). All
// in `λ = 4μ` ⇒ `ν = 0.4` compressible regime; `0.5× / 1× / 2×` IV-1's
// `(μ, λ) = (1e5, 4e5)` baseline.

const MU_INNER: f64 = 5.0e4;
const LAMBDA_INNER: f64 = 2.0e5;
const MU_MIDDLE: f64 = 1.0e5;
const LAMBDA_MIDDLE: f64 = 4.0e5;
const MU_OUTER: f64 = 2.0e5;
const LAMBDA_OUTER: f64 = 8.0e5;

/// Probe stretch for the Material-trait value-correctness gate.
/// `F_probe = diag(PROBE_LAMBDA, 1, 1)` with `PROBE_LAMBDA = 1.20` ties
/// to row 6's `LAMBDA_STRETCH` for cross-row continuity.
const PROBE_LAMBDA: f64 = 1.20;

/// Bit-equal tolerance for the Material-trait probe. Both the test-side
/// `expected[shell_at(centroid)]` and the mesher-side
/// `mesh.materials()[t]` run identical `NeoHookean::first_piola` /
/// `energy` arithmetic on identical `(μ, λ)` pairs and identical
/// `F_probe`; outputs are bit-equal by construction. `epsilon = 0.0`
/// satisfies clippy's `float_cmp` lint without actually relaxing the
/// bound (mirrors row 2's `EXACT_TOL = 0.0`).
const EXACT_TOL: f64 = 0.0;

// ── Exact-pinned counts (III-1 determinism contract) ─────────────────────
//
// Same scene topology + cell_size as row 3 (`sdf-to-tet-sphere`); the
// material-field carry doesn't affect mesh topology (BCC + stuffing runs
// before `materials_from_field` populates the cache). Mesh counts are
// therefore identical to row 3's `N_*_EXACT`. Per-shell tet counts are
// captured fresh during this row's first run on the capture platform
// (rustc 1.95.0, macOS arm64).

/// Total emitted tet count. Identical to row 3's `N_TETS_EXACT` —
/// material-field carry doesn't shift mesh topology.
const N_TETS_EXACT: usize = 6768;

/// Total mesh vertex count, including orphan BCC lattice corners.
/// Identical to row 3.
const N_VERTICES_EXACT: usize = 4634;

/// Vertices referenced by at least one tet. Identical to row 3.
const N_REFERENCED_EXACT: usize = 1483;

/// Per-shell tet counts captured on the III-1 / IV-4 h/2 canonical scene.
/// `INNER + MIDDLE + OUTER == N_TETS_EXACT` exact (the partition is
/// total over the body's tets — no tet sits outside all three buckets).
/// Captured 2026-05-05 at sim-soft `dev` (post-row-6 tip), rustc 1.95.0
/// (`59807616e` 2026-04-14) on macOS arm64 — same toolchain + platform
/// as IV-1's reference capture per
/// `invariant_iv_1_uniform_passthrough.rs:138-151`. III-1 determinism
/// contract makes these run-to-run + same-toolchain bit-stable; cross-
/// platform sparse-mesh stability (~3k tets) is empirically untested,
/// so a future CI matrix expansion may surface drift — relax via the
/// IV-1 sparse-tier protocol (do NOT re-bake without diagnosing
/// toolchain vs real regression first).
const N_INNER_TETS_EXACT: usize = 1344;
const N_MIDDLE_TETS_EXACT: usize = 1800;
const N_OUTER_TETS_EXACT: usize = 3624;

/// Per-shell tet counts in the visual-PLY z-slab cut
/// (`|centroid.z| < CELL_SIZE / 2`). Captured under the same provenance
/// as the full-body counts above; the BCC + stuffing pipeline is
/// symmetric across the body so the z-slab counts are approximately
/// `(slab_thickness / body_diameter) × N_TETS = (cell_size / (2 R_OUTER))
/// × N_TETS = 0.10 × 6768 ≈ 677` total (exact-pinned per-shell). The
/// z-slab PLY is the cf-view artifact — projecting onto a thin disk
/// reveals the three concentric color rings unmistakably from any
/// camera angle, mirroring hollow-shell-sdf row 2's z=0 slice
/// precedent. The un-filtered 6768-centroid cloud renders the outer
/// shell as a near-solid surface under cf-view's commit-3 sphere-radius
/// factor, occluding the inner layers; the octant-cut variant solves
/// most of the occlusion but the convex outer-shell surface still
/// dominates from non-cut-face camera angles. See
/// `project_cf_viewer_dense_point_cloud_gap.md` for the engine-side
/// followup.
const N_INNER_TETS_ZSLAB_EXACT: usize = 216;
const N_MIDDLE_TETS_ZSLAB_EXACT: usize = 176;
const N_OUTER_TETS_ZSLAB_EXACT: usize = 256;

// =============================================================================
// Shell partition (mirrors `sdf_material_tagging.rs::shell_at`)
// =============================================================================

/// Layer-ID for a single point under the partition rule
/// `LayeredScalarField` uses internally
/// (`partition_point(|&t| t <= phi)` ⇒ at exactly `phi == threshold[i]`
/// the point lands in the *outer* shell, `values[i + 1]`). Returns
/// `0` for inner, `1` for middle, `2` for outer — matching the `values`
/// slot indexing of the `LayeredScalarField` constructed in
/// [`build_three_shell_field`]. Test-side reimplementation of the
/// partition rule provides the cross-implementation gate at
/// [`verify_per_tet_layer_assignment_matches_sdf_predicate`]: any drift
/// between this if-else and the mesher's `partition_point` walk fires
/// loud at runtime.
fn shell_at(p: Vec3) -> usize {
    let phi = p.norm() - R_OUTER;
    if phi < PHI_INNER_THRESHOLD {
        0
    } else if phi < PHI_MIDDLE_THRESHOLD {
        1
    } else {
        2
    }
}

// =============================================================================
// Constructors
// =============================================================================

/// Build the canonical 3-shell `MaterialField`.
///
/// Two `LayeredScalarField`s — one for `μ`, one for `λ` — keyed on the
/// same body `SphereSdf{ radius: R_OUTER }`, with thresholds
/// `[PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD]` and per-shell values.
/// `MaterialField::from_fields` aggregates the two into the per-mesh
/// material-cache source.
///
/// No `with_interface_sdf` call: `LayeredScalarField`'s sharp boundaries
/// are not the `BlendedScalarField` smooth-transition band rule, so
/// `mesh.interface_flags()` will populate as `[false; n_tets]`. See
/// [`verify_interface_flags_all_false`] for the pedagogical contract.
fn build_three_shell_field() -> MaterialField {
    let body = || Box::new(SphereSdf { radius: R_OUTER });
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],
        vec![MU_INNER, MU_MIDDLE, MU_OUTER],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],
        vec![LAMBDA_INNER, LAMBDA_MIDDLE, LAMBDA_OUTER],
    ));
    MaterialField::from_fields(mu_field, lambda_field)
}

/// Build the canonical layered mesh — solid `SphereSdf{ radius: R_OUTER }`
/// meshed at `cell_size = 0.02`, with the 3-shell `MaterialField` carrying
/// per-tet `(μ, λ)` via centroid sampling.
fn build_layered_mesh() -> SdfMeshedTetMesh {
    let half = BBOX_HALF_EXTENT;
    let hints = MeshingHints {
        bbox: Aabb3::new(Vec3::new(-half, -half, -half), Vec3::new(half, half, half)),
        cell_size: CELL_SIZE,
        material_field: Some(build_three_shell_field()),
    };
    SdfMeshedTetMesh::from_sdf(&SphereSdf { radius: R_OUTER }, &hints)
        .expect("canonical layered-sphere scene meshes successfully — see III-1 + IV-4")
}

// =============================================================================
// Per-tet record (drives the value-correctness gate + PLY emit + summary)
// =============================================================================

#[derive(Debug, Clone)]
struct TetRecord {
    tet_id: u32,
    centroid: Vec3,
    layer_id: usize,
}

/// Walk every tet, compute its centroid, classify into a shell via
/// [`shell_at`]. The centroid IS the canonical evaluation point per Part
/// 7 §02 §00 — same point `materials_from_field` samples at when
/// populating `mesh.materials()`. Caller chains
/// [`verify_per_tet_layer_assignment_matches_sdf_predicate`] to gate
/// `mesh.materials()[t]` against `expected[shell_at(centroid)]` for
/// every tet via the Material-trait probe.
fn extract_centroid_records(mesh: &SdfMeshedTetMesh) -> Vec<TetRecord> {
    let positions = mesh.positions();
    let n_tets_id = mesh.n_tets() as TetId;
    let mut records = Vec::with_capacity(mesh.n_tets());
    for tet_id in 0..n_tets_id {
        let [v0, v1, v2, v3] = mesh.tet_vertices(tet_id);
        let p0 = positions[v0 as usize];
        let p1 = positions[v1 as usize];
        let p2 = positions[v2 as usize];
        let p3 = positions[v3 as usize];
        let centroid = (p0 + p1 + p2 + p3) * 0.25;
        records.push(TetRecord {
            tet_id,
            centroid,
            layer_id: shell_at(centroid),
        });
    }
    records
}

// =============================================================================
// verify_determinism
// =============================================================================

fn verify_determinism(mesh: &SdfMeshedTetMesh) {
    let other = build_layered_mesh();
    assert!(
        mesh.equals_structurally(&other),
        "second from_sdf call drifted — III-1 determinism contract violated",
    );
    // Material cache determinism — second build's `materials()` must
    // be bit-equal under the Material-trait probe at every tet.
    // (Sister of III-1's `materials_from_field` determinism: same
    // centroid sampling on bit-equal mesh topology.)
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
// verify_layered_field_thresholds_strictly_monotone
// =============================================================================

const fn verify_layered_field_thresholds_strictly_monotone() {
    // Re-assert the LayeredScalarField constructor's invariant
    // (`thresholds` strictly monotone-increasing) at the user-facing
    // example layer. The geometry constants are compile-time constants
    // and the asserts evaluate at compile time via `const { ... }`
    // blocks — reaching runtime here is impossible, but the const-block
    // panic surfaces at compile time on a regression and the source
    // location anchors the dependency on threshold ordering loud and
    // user-facing. Mirrors the LayeredScalarField runtime invariant
    // (sim-soft `field/layered.rs:101-110`) at compile time.
    const { assert!(R_INNER_OUTER < R_OUTER_INNER) };
    const { assert!(R_OUTER_INNER < R_OUTER) };
    const { assert!(PHI_INNER_THRESHOLD < PHI_MIDDLE_THRESHOLD) };
}

// =============================================================================
// verify_per_tet_layer_assignment_matches_sdf_predicate
// =============================================================================

/// Headline cross-implementation gate — for every tet, the mesher's
/// centroid-sampled `mesh.materials()[t]` must agree with
/// `expected[shell_at(centroid)]` under the Material-trait probe.
///
/// `NeoHookean::{mu, lambda}` are private (no public accessors), so the
/// comparison runs through the trait's `energy` + `first_piola` at a
/// reference `F_probe = diag(PROBE_LAMBDA, 1, 1)`. The full probe is
/// energy (1 scalar) + first_piola (Matrix3 = 9 scalars); together they
/// over-determine the `(μ, λ)` pair — `P_22 = λ ln(J)` directly fixes
/// `λ`, then `P_11 = μ(F − F⁻ᵀ)_11 + λ ln(J) F⁻ᵀ_11` fixes `μ`. Energy
/// alone is one linear equation in `(μ, λ)` so wouldn't suffice; the
/// Matrix3 entries are the load-bearing identification path. `EXACT_TOL
/// = 0.0` because both sides run identical NH arithmetic on identical
/// Lamé pairs + identical `F` — bit-equal by construction on a fixed
/// toolchain.
fn verify_per_tet_layer_assignment_matches_sdf_predicate(
    mesh: &SdfMeshedTetMesh,
    records: &[TetRecord],
) {
    let expected = expected_neo_hookeans();
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
        let expected_nh = &expected[rec.layer_id];
        // Probe via `energy` — scalar comparison surfaces the easiest
        // diagnostic when a regression hits.
        assert_relative_eq!(
            observed.energy(&f_probe),
            expected_nh.energy(&f_probe),
            epsilon = EXACT_TOL,
        );
        // Probe via `first_piola` — Matrix3 comparison adds the per-
        // entry coverage. A regression that nudges `μ` and `λ` such that
        // `energy` happens to coincide while `first_piola` differs
        // surfaces here.
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
// verify_shell_populations_non_empty
// =============================================================================

fn verify_shell_populations_non_empty(records: &[TetRecord]) -> [usize; 3] {
    let counts = shell_population_counts(records);
    assert!(
        counts[0] > 0,
        "inner shell empty at cell_size = {CELL_SIZE} — degenerate scene",
    );
    assert!(
        counts[1] > 0,
        "middle shell empty at cell_size = {CELL_SIZE} — annulus too thin?",
    );
    assert!(
        counts[2] > 0,
        "outer shell empty at cell_size = {CELL_SIZE} — degenerate scene",
    );
    counts
}

// =============================================================================
// verify_shell_populations_exact
// =============================================================================

fn verify_shell_populations_exact(counts: [usize; 3]) {
    assert_eq!(counts[0], N_INNER_TETS_EXACT, "inner-shell tet count drift");
    assert_eq!(
        counts[1], N_MIDDLE_TETS_EXACT,
        "middle-shell tet count drift"
    );
    assert_eq!(counts[2], N_OUTER_TETS_EXACT, "outer-shell tet count drift");
    // Total partitions over the body — the 3 shell counts sum to
    // `N_TETS_EXACT` (every tet bucket-assigns into exactly one shell).
    assert_eq!(
        counts[0] + counts[1] + counts[2],
        N_TETS_EXACT,
        "shell counts do not partition: sum != N_TETS_EXACT",
    );
}

// =============================================================================
// verify_layer_id_categorical_count
// =============================================================================

fn verify_layer_id_categorical_count(records: &[TetRecord]) {
    let mut seen = [false; 3];
    for rec in records {
        assert!(
            rec.layer_id < 3,
            "tet {} layer_id {} out of range [0, 3)",
            rec.tet_id,
            rec.layer_id,
        );
        seen[rec.layer_id] = true;
    }
    let unique_count = seen.iter().filter(|&&s| s).count();
    assert_eq!(
        unique_count, 3,
        "layer-id unique count {unique_count} != 3 — cf-view's categorical \
         heuristic ({{0, 1, 2}} → 3 sharp colors) requires all three buckets populated",
    );
}

// =============================================================================
// verify_interface_flags_all_false
// =============================================================================

fn verify_interface_flags_all_false(mesh: &SdfMeshedTetMesh) {
    let flags = mesh.interface_flags();
    assert_eq!(
        flags.len(),
        mesh.n_tets(),
        "interface_flags length ({}) != n_tets ({})",
        flags.len(),
        mesh.n_tets(),
    );
    let any_true_count = flags.iter().filter(|&&b| b).count();
    assert_eq!(
        any_true_count, 0,
        "interface_flags has {any_true_count} `true` flags — `LayeredScalarField`'s \
         sharp boundaries do not populate the `BlendedScalarField` interface band rule \
         `|φ(x_c)| < L_e` (Part 7 §02 §01); a non-zero count means \
         `MaterialField::with_interface_sdf` was called somewhere upstream",
    );
}

// =============================================================================
// verify_centroid_radii_within_body
// =============================================================================

fn verify_centroid_radii_within_body(records: &[TetRecord]) -> f64 {
    let mut max_radius = 0.0_f64;
    for rec in records {
        let r = rec.centroid.norm();
        if r > max_radius {
            max_radius = r;
        }
        assert!(
            r < R_OUTER,
            "tet {} centroid at radius {r:e} >= R_OUTER ({R_OUTER}) — mesher \
             retained a tet whose centroid landed outside the body. Either the \
             SDF sampling sign flipped or stuffing dispatch regressed.",
            rec.tet_id,
        );
    }
    max_radius
}

// =============================================================================
// Helpers
// =============================================================================

fn shell_population_counts(records: &[TetRecord]) -> [usize; 3] {
    let mut counts = [0usize; 3];
    for rec in records {
        counts[rec.layer_id] += 1;
    }
    counts
}

const fn expected_neo_hookeans() -> [NeoHookean; 3] {
    [
        NeoHookean::from_lame(MU_INNER, LAMBDA_INNER),
        NeoHookean::from_lame(MU_MIDDLE, LAMBDA_MIDDLE),
        NeoHookean::from_lame(MU_OUTER, LAMBDA_OUTER),
    ]
}

fn probe_f() -> Matrix3<f64> {
    Matrix3::from_diagonal_element(1.0)
        .map_with_location(|i, j, x| if i == 0 && j == 0 { PROBE_LAMBDA } else { x })
}

// =============================================================================
// Z-slab visual filter
// =============================================================================

/// Half-thickness of the z-slab visual cut. `cell_size / 2 = 0.01 m` is
/// thin enough that BCC + stuffing produces approximately one cell-cube
/// layer of tets within the slab, sparse enough for cf-view's commit-3
/// sphere-radius factor to render discrete dots without overlap. The
/// projected disk in cf-view shows three concentric color rings cleanly
/// from any camera angle (no orbit needed — strictly cleaner pedagogy
/// than the octant-cut variant's wedge, which exposes the cut faces but
/// the convex outer-shell surface still dominates from non-cut-face
/// camera angles).
const ZSLAB_HALF_THICKNESS: f64 = CELL_SIZE / 2.0;

/// Keep centroids in the `|z| < ZSLAB_HALF_THICKNESS` slab for cf-view
/// rendering — projects to a thin disk in 3D, reading as three
/// concentric color rings on the z=0 plane.
///
/// The asserts (anchors 1-9) run over all 6768 tets — only the visual
/// PLY is filtered. Per-shell counts in the slab are exact-pinned via
/// the `*_ZSLAB_EXACT` constants for III-1 determinism contract
/// coverage. Mirrors hollow-shell-sdf row 2's z=0 slice precedent for
/// cross-section cf-view visualization (rationale: a thin slab in 3D
/// projects unambiguously to a 2D disk pattern at any orbit angle,
/// where a 3D wedge requires the user to face the cut planes).
fn keep_for_zslab_visual(rec: &TetRecord) -> bool {
    rec.centroid.z.abs() < ZSLAB_HALF_THICKNESS
}

// =============================================================================
// verify_zslab_visual_populations_exact
// =============================================================================

fn verify_zslab_visual_populations_exact(records: &[TetRecord]) -> [usize; 3] {
    let mut counts = [0usize; 3];
    for rec in records.iter().filter(|r| keep_for_zslab_visual(r)) {
        counts[rec.layer_id] += 1;
    }
    assert!(
        counts[0] > 0,
        "z-slab inner-shell empty — cf-view visual would lose the inner-layer ring",
    );
    assert!(
        counts[1] > 0,
        "z-slab middle-shell empty — cf-view visual would lose the middle-layer ring",
    );
    assert!(
        counts[2] > 0,
        "z-slab outer-shell empty — cf-view visual would lose the outer-layer ring",
    );
    assert_eq!(
        counts[0], N_INNER_TETS_ZSLAB_EXACT,
        "z-slab inner-shell count drift",
    );
    assert_eq!(
        counts[1], N_MIDDLE_TETS_ZSLAB_EXACT,
        "z-slab middle-shell count drift",
    );
    assert_eq!(
        counts[2], N_OUTER_TETS_ZSLAB_EXACT,
        "z-slab outer-shell count drift",
    );
    counts
}

// =============================================================================
// PLY emit — per-tet centroid point cloud + categorical layer-id scalar
// =============================================================================

/// Vertices = per-tet centroids in the `|z| < ZSLAB_HALF_THICKNESS`
/// slab cut (see [`keep_for_zslab_visual`]); faces empty (point cloud);
/// one per-vertex scalar `material_layer_id` carrying the integer 0/1/2
/// cast as f32. cf-view's Q5 categorical-colormap heuristic detects
/// "integer-valued + < 16 unique values" → tab10, rendering 3 sharp
/// colors as concentric rings on the projected disk. Mirrors row 1's
/// vertices-only PLY pattern + row 2's z=0 cross-section precedent.
fn save_layer_id_ply(records: &[TetRecord], path: &Path) -> Result<()> {
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
    let layer_id: Vec<f32> = kept.iter().map(|r| r.layer_id as f32).collect();
    attributed.insert_extra("material_layer_id", layer_id)?;
    save_ply_attributed(&attributed, path, true)?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

fn print_summary(
    mesh: &SdfMeshedTetMesh,
    referenced: &[VertexId],
    counts: [usize; 3],
    zslab_counts: [usize; 3],
    centroid_radius_max: f64,
    path: &Path,
) {
    let n_orphans = mesh.n_vertices() - referenced.len();
    println!("==== layered-scalar-field ====");
    println!();
    println!("input  : SphereSdf {{ radius: {R_OUTER} }}  (solid body — no cavity)");
    println!("         MeshingHints {{");
    println!("           bbox:           [-{BBOX_HALF_EXTENT}, {BBOX_HALF_EXTENT}]³,");
    println!("           cell_size:      {CELL_SIZE},");
    println!("           material_field: MaterialField::from_fields(μ_field, λ_field)");
    println!("         }}");
    println!();
    println!("MaterialField  : 3-shell concentric LayeredScalarField × 2 (μ, λ)");
    println!(
        "                 keyed on body SphereSdf, thresholds = [{PHI_INNER_THRESHOLD:.2}, \
         {PHI_MIDDLE_THRESHOLD:.2}] on phi = ‖p‖ − {R_OUTER}"
    );
    // Boundary convention follows `LayeredScalarField`'s
    // `partition_point(|&t| t <= phi)` rule: at `phi == threshold[i]`
    // the point lands in the OUTER shell (values[i+1]). So the
    // shell intervals are closed-left / open-right (with `R_OUTER`
    // closed at the body's exterior surface — inclusive only because
    // tet centroids are interior, see `verify_centroid_radii_within_body`).
    println!(
        "                 inner   ‖p‖ ∈ [0,    {R_INNER_OUTER})   (μ = {MU_INNER:e}, λ = \
         {LAMBDA_INNER:e})"
    );
    println!(
        "                 middle  ‖p‖ ∈ [{R_INNER_OUTER}, {R_OUTER_INNER})   (μ = {MU_MIDDLE:e}, λ = \
         {LAMBDA_MIDDLE:e})"
    );
    println!(
        "                 outer   ‖p‖ ∈ [{R_OUTER_INNER}, {R_OUTER}]   (μ = {MU_OUTER:e}, λ = \
         {LAMBDA_OUTER:e})"
    );
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!(
        "  determinism                       : 2nd from_sdf call equals_structurally + materials cache bit-equal"
    );
    println!(
        "  counts                            : n_tets / n_vertices / referenced exact-pin (III-1 contract)"
    );
    println!(
        "  layered_field_thresholds_monotone : R_INNER_OUTER < R_OUTER_INNER < R_OUTER (constructor invariant)"
    );
    println!(
        "  per_tet_layer_assignment          : mesh.materials()[t] vs expected[shell_at(centroid)] bit-equal"
    );
    println!(
        "  shell_populations_non_empty       : all three shells > 0 tets at cell_size = {CELL_SIZE}"
    );
    println!("  shell_populations_exact           : per-shell tet counts exact-pinned");
    println!("  layer_id_categorical_count        : exactly 3 unique IDs ∈ {{0, 1, 2}}");
    println!(
        "  interface_flags_all_false         : LayeredScalarField does not populate interface band rule"
    );
    println!("  centroid_radii_within_body        : every per-tet centroid ‖c‖ < R_OUTER");
    println!(
        "  zslab_visual_populations_exact    : per-shell z-slab counts exact-pinned (cf-view artifact)"
    );
    println!();
    println!("Mesh:");
    println!("  n_tets                     : {:>7}", mesh.n_tets());
    println!("  n_vertices                 : {:>7}", mesh.n_vertices());
    println!("  referenced (≤ n_vertices)  : {:>7}", referenced.len());
    println!("  orphans (n_vertices − ref) : {n_orphans:>7}");
    println!();
    println!("Per-shell tet partition:");
    println!("  inner   (layer 0)          : {:>7}", counts[0]);
    println!("  middle  (layer 1)          : {:>7}", counts[1]);
    println!("  outer   (layer 2)          : {:>7}", counts[2]);
    println!(
        "  total                      : {:>7}  (== n_tets)",
        counts[0] + counts[1] + counts[2]
    );
    println!();
    println!("Centroid radii:");
    println!(
        "  max ‖c‖                    : {centroid_radius_max:>13.6e} m  (body radius {R_OUTER} m)"
    );
    println!();
    let zslab_total = zslab_counts[0] + zslab_counts[1] + zslab_counts[2];
    println!("Z-slab cut PLY (cf-view artifact, |z| < {ZSLAB_HALF_THICKNESS} m):");
    println!("  inner   (layer 0)          : {:>7}", zslab_counts[0]);
    println!("  middle  (layer 1)          : {:>7}", zslab_counts[1]);
    println!("  outer   (layer 2)          : {:>7}", zslab_counts[2]);
    println!(
        "  total                      : {zslab_total:>7}  (≈ N_TETS · slab_thickness / body_diameter)"
    );
    println!();
    println!("PLY    : {}", path.display());
    println!(
        "         vertices-only point cloud ({zslab_total} centroids — z-slab of the {} \
         body tets) + 1 per-vertex scalar:",
        mesh.n_tets(),
    );
    println!("           extras[\"material_layer_id\"] — integer 0 / 1 / 2 (categorical)");
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!("         default-picks material_layer_id (only scalar present); cf-view's");
    println!("         categorical heuristic detects integer-valued + 3 unique ⇒ tab10");
    println!("         palette. The thin z-slab projects to a 2D disk in cf-view, reading");
    println!("         as three concentric color rings unmistakably from any camera angle.");
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let mesh = build_layered_mesh();

    verify_layered_field_thresholds_strictly_monotone();
    verify_determinism(&mesh);

    let referenced = referenced_vertices(&mesh);
    verify_counts(&mesh, &referenced);

    let records = extract_centroid_records(&mesh);

    verify_per_tet_layer_assignment_matches_sdf_predicate(&mesh, &records);
    let counts = verify_shell_populations_non_empty(&records);
    verify_shell_populations_exact(counts);
    verify_layer_id_categorical_count(&records);
    verify_interface_flags_all_false(&mesh);
    let centroid_radius_max = verify_centroid_radii_within_body(&records);

    let zslab_counts = verify_zslab_visual_populations_exact(&records);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("material_layer_assignment.ply");
    save_layer_id_ply(&records, &out_path)?;

    print_summary(
        &mesh,
        &referenced,
        counts,
        zslab_counts,
        centroid_radius_max,
        &out_path,
    );

    Ok(())
}
