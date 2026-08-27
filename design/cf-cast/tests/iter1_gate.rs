//! S3 plumbing-refactor falsification gate against the iter-1
//! reference STL set (`docs/CF_CAST_MATING_FEATURES_PLAN.md` §G2,
//! recon §11 item 1).
//!
//! Verifies that each reference STL in `$CF_CAST_ITER1_DIR` is
//! manifold-clean (accepted by manifold3d's `from_mesh_f64` after a
//! 1 µm input weld) and that the round-trip through manifold3d
//! preserves the vertex set and face count. This is the core
//! invariant S4/S5/S6 rely on when they begin emitting concrete
//! [`cf_cast::MatingTransform`] variants.
//!
//! # Why we do NOT assert triangle-set equivalence
//!
//! The cf-cast funnel STL surfaces a real manifold3d behavior:
//! complex shapes (the funnel's truncated-cone region in iter-1)
//! get **re-tessellated** under round-trip — same point cloud, same
//! face count, different triangle set (planar-quad diagonal swap).
//! The other 13 iter-1 reference STLs (cup pieces, plugs, platform)
//! preserve the triangle set exactly. ⚠ That 1-plus-13 split describes
//! the 14-piece generation it was measured on; the current spec emits
//! **15** (`dowel` and `funnel` both present). Whether the two added
//! pieces preserve their triangle set is NOT re-measured — this gate
//! checks vertex and face COUNT, never the triangle set, so it cannot
//! answer that either way. `geometric_equivalence` is
//! the right tool for S4-S6 before-vs-after feature-migration tests
//! (where re-tessellation isn't expected because both sides
//! manifold3d-round-trip the same composition); for "raw STL →
//! manifold round-trip" the looser vertex-set + face-count check
//! is what manifold3d actually guarantees.
//!
//! The live S3 pipeline pure-pass-through (empty `Vec<MatingTransform>`)
//! short-circuits the round-trip entirely (see
//! `cf_cast::apply_mating_transforms`'s empty-Vec early return), so
//! S3's live output is bit-equal to the pre-refactor reference and
//! does NOT need this gate to pass on every commit. This gate
//! confirms iter-1 reference STLs are themselves manifold-clean and
//! geometry-preserving under round-trip — a stronger sentinel than
//! the synthetic unit-cube round-trip in `mesh_csg::tests`.
//!
//! # Why ignored by default
//!
//! NOT for cost — the test depends on an absolute path outside the
//! repo (`~/scans/cast_iter1_design/`). Gating behind `#[ignore]` + an
//! env-var lookup keeps the standard `cargo test -p cf-cast` workflow
//! hermetic.
//!
//! ⚠ **THE COST FIGURES THAT USED TO JUSTIFY THIS WERE BOTH WRONG, AND
//! WRONG IN THE SAME DIRECTION.** It read: "Iter-1 cf-cast-cli runs take
//! ~6 min wall time (`project_cf_cast_f4_split_asymmetry`); loading +
//! welding all 14 reference STLs is ~10 s". Re-measured 2026-08-27 on an
//! M4 laptop:
//!
//! - `cf-cast-cli` on `cast.iter1-design.toml`: **96 s**, not ~6 min.
//! - this gate over **15** STLs, `--release`: **0.07 s** (3 runs, stable),
//!   not ~10 s — off by more than two orders of magnitude.
//!
//! ⇒ The fixture path is the whole reason for `#[ignore]`, and stating a
//! cost that was never re-checked made a hermeticity constraint look like
//! a performance one. The synthetic tests at the bottom of this file are
//! the reason a stale figure here can no longer hide the gate rotting:
//! they run on every `cargo test -p cf-cast`, in microseconds, with no
//! fixture at all.
//!
//! # Manual run
//!
//! ```text
//! CF_CAST_ITER1_DIR=$HOME/scans/cast_iter1_design \
//!   cargo test --release -p cf-cast --test iter1_gate -- --ignored --nocapture
//! ```
//!
//! ⚠ **The cast folder OR its `stls/` subfolder both work** — see
//! [`resolve_stl_dir`]. cf-cast-cli used to write its STLs flat into the
//! output directory and now writes them to `output_dir/stls/` (the generated
//! `procedure.md` says so, and promises `cf-viewer` accepts either form).
//! This gate read the directory NON-recursively, so the command above — its
//! own documented one — found zero `.stl` and failed with "wrong fixture
//! directory?". Measured 2026-08-27, the first time the gate had been run
//! since the layout changed; the pipeline itself was fine.

// Integration test idiom: surface failures with explicit panic
// messages so the workshop user can triage from one stderr block.
#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::path::PathBuf;

use cf_cast::{WELD_TOLERANCE_M, weld_in_place};
use manifold3d::Manifold;

/// The directory actually holding the STLs, given a cast-output path.
///
/// Accepts both layouts cf-cast-cli has emitted: STLs flat in the output
/// directory (pre-2026-05) and STLs under `output_dir/stls/` (current). Returns
/// the `stls/` subdirectory only when it exists AND the given directory holds
/// no `.stl` of its own, so a flat fixture is never silently redirected.
///
/// ★ Resolution lives here, not inline in the gate, because the gate needs an
/// out-of-repo fixture and so is `#[ignore]`d — logic inside it is unreachable
/// from CI. The two tests below DO run in `cargo test -p cf-cast`, so the rule
/// that decides where to look is gated even though the round-trip is not.
fn resolve_stl_dir(dir: &std::path::Path) -> PathBuf {
    let holds_stl = |d: &std::path::Path| {
        std::fs::read_dir(d).is_ok_and(|mut entries| {
            entries.any(|e| e.is_ok_and(|e| e.path().extension().is_some_and(|ext| ext == "stl")))
        })
    };
    let nested = dir.join("stls");
    if !holds_stl(dir) && nested.is_dir() {
        return nested;
    }
    dir.to_path_buf()
}

/// Round-trip every `.stl` under `$CF_CAST_ITER1_DIR` through
/// manifold3d and confirm geometric equivalence is preserved.
///
/// Marked `#[ignore]` because the test depends on an out-of-repo
/// fixture path (see file-level docs); opt in via `--ignored` when
/// the env var is set.
#[test]
#[ignore = "iter-1 gate: requires CF_CAST_ITER1_DIR=<path-to-cast_iter1_design>; run with --ignored"]
fn iter1_reference_stls_round_trip_through_manifold3d() {
    let Ok(dir_env) = std::env::var("CF_CAST_ITER1_DIR") else {
        panic!("CF_CAST_ITER1_DIR not set — see file-level docs for the manual-run command");
    };
    let dir = PathBuf::from(&dir_env);
    assert!(
        dir.is_dir(),
        "CF_CAST_ITER1_DIR={dir_env} is not a directory"
    );
    check_reference_stls(&dir);
}

/// The gate's whole body, taking the directory **as the operator gave it** —
/// resolution included.
///
/// ★★ SPLIT OUT SO THE CALL TO [`resolve_stl_dir`] IS ITSELF GATED. Before
/// this, `resolve_stl_dir` had two tests and its single call site had none:
/// deleting `let dir = resolve_stl_dir(&dir)` from the gate left the whole
/// suite green, because the tests kept the function alive and the gate that
/// used it is `#[ignore]`d. The rule was gated; the WIRING was not — the same
/// shape this arc keeps finding, and the reason the tests below hand this
/// function an unresolved cast folder rather than calling `resolve_stl_dir`
/// directly.
///
/// Panics with the same messages the gate always did, so the `#[ignore]`d
/// caller's workshop-triage output is unchanged.
fn check_reference_stls(given: &std::path::Path) {
    let dir = resolve_stl_dir(given);

    let mut stls: Vec<PathBuf> = std::fs::read_dir(&dir)
        .expect("read CF_CAST_ITER1_DIR")
        .filter_map(|entry| {
            let path = entry.ok()?.path();
            if path.extension().is_some_and(|ext| ext == "stl") {
                Some(path)
            } else {
                None
            }
        })
        .collect();
    stls.sort_unstable();
    assert!(
        !stls.is_empty(),
        "no .stl files in {} — wrong fixture directory?",
        dir.display()
    );

    eprintln!(
        "[iter1-gate] checking {} STLs from {}",
        stls.len(),
        dir.display()
    );

    let mut failures: Vec<(PathBuf, String)> = Vec::new();
    for stl in &stls {
        let mut mesh = mesh_io::load_stl(stl).unwrap_or_else(|e| {
            panic!("load_stl({}): {e}", stl.display());
        });
        // STL is non-indexed — weld at 1 µm so manifold3d accepts
        // the shared-index form (recon §11 item 3 + S1 ADR finding 3).
        weld_in_place(&mut mesh, WELD_TOLERANCE_M);

        let manifold = match Manifold::from_vertices_and_faces(&mesh.vertices, &mesh.faces) {
            Ok(m) => m,
            Err(e) => {
                failures.push((
                    stl.clone(),
                    format!("from_vertices_and_faces: {e} (welded mesh not manifold-clean)"),
                ));
                continue;
            }
        };

        let (round_verts, round_faces) = manifold.to_vertices_and_faces();

        // Vertex count + face count match — geometry is preserved
        // under round-trip even if manifold3d re-tessellates planar
        // regions (see file-level docs on the funnel observation).
        if round_verts.len() != mesh.vertices.len() {
            failures.push((
                stl.clone(),
                format!(
                    "vertex-count drift: input {} vs round-trip {}",
                    mesh.vertices.len(),
                    round_verts.len()
                ),
            ));
            continue;
        }
        if round_faces.len() != mesh.faces.len() {
            failures.push((
                stl.clone(),
                format!(
                    "face-count drift: input {} vs round-trip {}",
                    mesh.faces.len(),
                    round_faces.len()
                ),
            ));
            continue;
        }

        eprintln!(
            "[iter1-gate] ok: {} ({} verts, {} faces — manifold-clean, preserved under round-trip)",
            stl.file_name().and_then(|s| s.to_str()).unwrap_or("?"),
            mesh.vertices.len(),
            mesh.faces.len(),
        );
    }

    assert!(
        failures.is_empty(),
        "iter-1 reference STLs failed the manifold3d round-trip gate ({} of {}):\n{}",
        failures.len(),
        stls.len(),
        failures
            .iter()
            .map(|(p, m)| format!("  {} — {m}", p.display()))
            .collect::<Vec<_>>()
            .join("\n"),
    );
}

/// The current `output_dir/stls/` layout is found from the cast folder.
///
/// ★ This is the case that was broken: the gate's own documented command names
/// the cast folder, cf-cast-cli writes to `stls/`, and a non-recursive
/// `read_dir` found nothing. Runs in plain `cargo test -p cf-cast` — the gate
/// it protects cannot, so without this the rule would be untested.
#[test]
fn the_nested_stls_layout_is_found_from_the_cast_folder() {
    let root = fixture("nested");
    std::fs::create_dir_all(root.join("stls")).expect("mkdir stls");
    std::fs::write(root.join("procedure.md"), "# procedure\n").expect("write");
    std::fs::write(root.join("stls/piece.stl"), b"solid x\nendsolid x\n").expect("write");

    assert_eq!(
        resolve_stl_dir(&root),
        root.join("stls"),
        "a cast folder whose STLs live in stls/ must resolve into it"
    );
}

/// A flat fixture is never redirected, even when `stls/` exists beside it.
///
/// ⚠ THE POSITIVE CONTROL. Without it, `resolve_stl_dir` could return
/// `dir.join("stls")` unconditionally and the test above would still pass —
/// silently sending every pre-2026-05 fixture (and every `.OLD*` generation,
/// which are all flat) at an empty or unrelated directory.
#[test]
fn a_flat_layout_is_never_redirected() {
    let root = fixture("flat");
    std::fs::create_dir_all(root.join("stls")).expect("mkdir stls");
    std::fs::write(root.join("piece.stl"), b"solid x\nendsolid x\n").expect("write");
    std::fs::write(root.join("stls/other.stl"), b"solid y\nendsolid y\n").expect("write");

    assert_eq!(
        resolve_stl_dir(&root),
        root,
        "a directory holding its own STLs must be used as given"
    );

    // No `stls/` at all: hand back the path as given, so the gate's
    // "no .stl files in {dir} — wrong fixture directory?" names what the
    // operator actually typed rather than a subdirectory they never mentioned.
    let empty = fixture("empty");
    assert_eq!(resolve_stl_dir(&empty), empty);

    // ⚠ THE EXCEPTION, asserted rather than left implied. An EMPTY `stls/`
    // still redirects — `holds_stl(dir)` is false and `nested.is_dir()` is
    // true — so in that one case the error names `<dir>/stls`, NOT what was
    // typed. That is the more useful message (it says where we looked), but
    // the commit that added this claimed the path is "returned as given"
    // whenever no layout is present, and that claim was wrong here.
    let hollow = fixture("hollow");
    std::fs::create_dir_all(hollow.join("stls")).expect("mkdir stls");
    assert_eq!(
        resolve_stl_dir(&hollow),
        hollow.join("stls"),
        "an empty stls/ is still a redirect — the error will name it, not the given path"
    );
}

/// A throwaway directory for the two resolution tests.
fn fixture(tag: &str) -> PathBuf {
    let root = std::env::temp_dir().join(format!(
        "cf-iter1-gate-{tag}-{}-{:?}",
        std::process::id(),
        std::thread::current().id()
    ));
    // Absent on a first run, present on a re-run. Anything ELSE is a fixture
    // we failed to clear — and a stale directory would decide the resolution
    // these tests are asserting, so it must not pass silently.
    if let Err(e) = std::fs::remove_dir_all(&root) {
        assert_eq!(
            e.kind(),
            std::io::ErrorKind::NotFound,
            "could not clear fixture {}: {e}",
            root.display()
        );
    }
    std::fs::create_dir_all(&root).expect("mkdir root");
    root
}

/// A closed unit tetrahedron in ASCII STL, wound outward.
///
/// Four triangles, four vertices, every edge shared by exactly two faces —
/// the smallest thing manifold3d will accept, so a failure here is about the
/// GATE and not about the mesh being marginal.
fn tetrahedron_stl() -> String {
    use std::fmt::Write as _;

    // Outward normals verified by right-hand rule against (0,0,0)-(1,0,0)-
    // (0,1,0)-(0,0,1): -z, -y, -x, and (1,1,1)/√3 for the slanted face.
    let faces = [
        [(0., 0., 0.), (0., 1., 0.), (1., 0., 0.)],
        [(0., 0., 0.), (1., 0., 0.), (0., 0., 1.)],
        [(0., 0., 0.), (0., 0., 1.), (0., 1., 0.)],
        [(1., 0., 0.), (0., 1., 0.), (0., 0., 1.)],
    ];
    let mut s = String::from("solid tet\n");
    for f in faces {
        s.push_str("facet normal 0 0 0\n  outer loop\n");
        for (x, y, z) in f {
            // Infallible into a String; `expect` rather than a discarded
            // Result, which this crate's lints reject.
            writeln!(s, "    vertex {x} {y} {z}").expect("write to String");
        }
        s.push_str("  endloop\nendfacet\n");
    }
    s.push_str("endsolid tet\n");
    s
}

/// The gate, driven end-to-end from an UNRESOLVED cast folder, on a sound mesh.
///
/// ★★ Two things at once, both of which the suite lacked:
///
/// 1. **The wiring.** This hands [`check_reference_stls`] the cast folder, not
///    `stls/`. Delete the `resolve_stl_dir` call inside it and this test goes
///    red — which it did NOT before, when only the rule was tested.
/// 2. **A positive control.** Every other assertion here is about paths or
///    about failure; nothing proved the round-trip can PASS. A gate verified
///    only where it fires is indistinguishable from one that always fires.
#[test]
fn the_gate_resolves_into_stls_and_a_sound_mesh_passes() {
    let root = fixture("gate-ok");
    std::fs::create_dir_all(root.join("stls")).expect("mkdir stls");
    std::fs::write(root.join("stls/tet.stl"), tetrahedron_stl()).expect("write");

    // Panics on any failure, exactly as it does for the workshop fixture.
    check_reference_stls(&root);
}

/// The gate must be able to FAIL — a lone triangle is not manifold-clean.
///
/// ⚠ THE NEGATIVE CONTROL, and the reason it is worth its lines: the test
/// above passes just as happily against a `check_reference_stls` whose
/// failure branch is unreachable. One open triangle has three boundary edges,
/// so `from_vertices_and_faces` rejects it and the gate's summary assertion
/// fires. Pinned on the summary's own wording so a silent downgrade to
/// "collect failures and return" cannot pass.
#[test]
#[should_panic(expected = "failed the manifold3d round-trip gate")]
fn the_gate_rejects_a_mesh_that_is_not_manifold_clean() {
    let root = fixture("gate-bad");
    std::fs::create_dir_all(root.join("stls")).expect("mkdir stls");
    let open = "solid open\nfacet normal 0 0 0\n  outer loop\n    \
                vertex 0 0 0\n    vertex 1 0 0\n    vertex 0 1 0\n  \
                endloop\nendfacet\nendsolid open\n";
    std::fs::write(root.join("stls/open.stl"), open).expect("write");

    check_reference_stls(&root);
}
