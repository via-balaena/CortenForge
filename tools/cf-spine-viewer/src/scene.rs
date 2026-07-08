//! Headless FSU scene builder (Bevy-free).
//!
//! Assembles the static L4–L5 Functional Spinal Unit as plain data for
//! rendering — everything in native BodyParts3D millimetres (a static view runs
//! no solver, so no recenter/scale is needed; the three meshes already stack in
//! their shared anatomical frame):
//!
//!   * the two vertebra surfaces (welded), plus the disc STL shell;
//!   * the field-derived ligament attachment sites (ALL + interspinous);
//!   * the facet near-contact points, from the `ShapeConcave` SDF collision at
//!     the neutral pose.
//!
//! The shared field recipes (`load` / `oracle` / `segment_frame` /
//! `extreme_vertex` / `facet_grid`) live in the graded `cf-fsu-geometry` crate,
//! consumed here; the rung 4b/5/7 tests migrate to it in a follow-up. This
//! module keeps the viewer-specific assembly: the disc principal-ML choice,
//! ligament building, the co-registration warnings, neutral-pose facet contacts,
//! and the scene `build` orchestrator.

use std::path::Path;
use std::sync::Arc;

use anyhow::Result;
use cf_fsu_geometry::{
    BODY_RADIUS, FACET_CELL, FACET_MAX_CONTACTS, SegmentFrame, extreme_vertex, facet_grid, load,
    oracle, segment_frame,
};
use cf_fsu_model::{CoupledFsu, CoupledParams, CoupledTrajectory, VertexId};
use mesh_types::{Aabb, IndexedMesh};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_core::sdf::compute_shape_contact;
use sim_core::{Pose, SdfContact, SdfGrid, ShapeConcave};

/// Peak applied moment of the captured ramp (N·m) — the physiologic pure-moment probe
/// (rung 7). Swept from `−MAX` (extension) to `+MAX` (flexion); the coupled solver
/// returns the equilibrium angle at each, so the motion is force-driven and ROM-limited
/// (bones stop on the facets in extension, ligaments/disc limit flexion).
const MAX_MOMENT: f64 = 7.5;
/// Frames in the captured ramp, evenly spaced in applied moment across `[−MAX, +MAX]`.
/// Each solves one coupled equilibrium (cheap) + reuses one sub-degree disc solve, so the
/// count can be generous; the viewer interpolates between frames for smooth playback.
const N_RAMP_FRAMES: usize = 25;

/// A ligament rendered as a straight line between two field-derived sites.
pub struct Ligament {
    pub name: &'static str,
    pub inferior: Point3<f64>, // attachment on L5
    pub superior: Point3<f64>, // attachment on L4
}

/// The assembled scene, as plain geometry in native millimetres.
pub struct FsuScene {
    pub l4: IndexedMesh,
    pub l5: IndexedMesh,
    /// The clean STL disc surface (native mm) — a smooth watertight lens, the render
    /// geometry. The tet mesh is too fragmented to render as a coherent disc, so the
    /// viewer draws THIS surface and displaces it by the FEM field (see `disc_node_map`).
    pub disc_surface: IndexedMesh,
    /// For each `disc_surface` vertex, the index (into [`CoupledTrajectory::rest_nodes_native`])
    /// of the nearest tet node on the largest component's boundary. Per frame the vertex is
    /// displaced by that node's FEM displacement — a smooth, complete disc that deforms by
    /// the real physics without depending on the ragged tet boundary.
    pub disc_node_map: Vec<usize>,
    /// The coupled FSU's captured force-driven ramp: per-frame equilibrium angle +
    /// deformed disc tet-node positions (native mm) + facet engagement + pivot/axis. The
    /// disc FEM displacement field is sampled onto `disc_surface` via `disc_node_map`.
    pub flexion: CoupledTrajectory,
    pub ligaments: Vec<Ligament>,
    /// Facet near-contact points (within the 1 mm detection margin at the
    /// neutral pose) — the articular-process contact region.
    pub facet_contacts: Vec<Point3<f64>>,
    /// Combined bounding box across all three meshes, for camera framing.
    pub aabb: Aabb,
    /// Co-registration / degeneracy warnings surfaced during assembly (empty
    /// for a well-formed, co-registered trio). Shown in the panel + on stderr
    /// so a misaligned or mismatched input is never presented as a valid FSU.
    pub warnings: Vec<String>,
}

/// The disc's principal medio-lateral axis = its widest AABB extent, snapped to
/// the coordinate axis (rung-7's `ml_scaled` derivation). This is the CANONICAL
/// flexion/extension axis rung-7 adopted over the vertebral-frame ML: the latter
/// is tilted ~19° by the lordotic wedge + coarse body-centre localisation, so
/// the ligament midline filter uses THIS axis to match the validated FSU.
fn disc_principal_ml(disc: &IndexedMesh) -> Vector3<f64> {
    let bbox = Aabb::from_points(disc.vertices.iter());
    let span = bbox.max - bbox.min;
    let extents = [span.x, span.y, span.z];
    // Widest extent = ML. On exactly-equal extents `max_by` keeps the later
    // index (deterministic); a real disc has a distinct widest axis.
    let widest = (0..3)
        .max_by(|&a, &b| extents[a].total_cmp(&extents[b]))
        .unwrap_or(0);
    let mut ml = Vector3::zeros();
    ml[widest] = 1.0;
    ml
}

/// The two modelled ligaments: anterior longitudinal (ALL, on the body rim,
/// `-posterior`) and interspinous (ISP, at the spinous tips, `+posterior`) —
/// endpoints field-derived. `ml` is the disc principal axis (rung-7's canonical
/// flexion axis), used as the mid-sagittal-plane normal for the near-midline
/// filter so the sites match the validated FSU. A ligament whose sites can't be
/// located on a degenerate mesh is skipped with a warning rather than drawn wrong.
fn build_ligaments(
    l4: &IndexedMesh,
    l5: &IndexedMesh,
    frame: &SegmentFrame,
    ml: Vector3<f64>,
    warnings: &mut Vec<String>,
) -> Vec<Ligament> {
    let (b4, b5, post) = (frame.b4, frame.b5, frame.posterior);
    let mut ligaments = Vec::new();

    match (
        extreme_vertex(l5, b5, -post, ml, BODY_RADIUS),
        extreme_vertex(l4, b4, -post, ml, BODY_RADIUS),
    ) {
        (Some(inferior), Some(superior)) => {
            ligaments.push(Ligament {
                name: "ALL",
                inferior,
                superior,
            });
        }
        _ => warnings.push(
            "ALL ligament: no near-midline body-rim vertex found (degenerate mesh?)".to_string(),
        ),
    }

    match (
        extreme_vertex(l5, b5, post, ml, f64::INFINITY),
        extreme_vertex(l4, b4, post, ml, f64::INFINITY),
    ) {
        (Some(inferior), Some(superior)) => {
            ligaments.push(Ligament {
                name: "ISP",
                inferior,
                superior,
            });
        }
        _ => warnings.push(
            "ISP ligament: no near-midline spinous vertex found (degenerate mesh?)".to_string(),
        ),
    }

    ligaments
}

/// Co-registration sanity checks, cheaply reproduced from AABBs (no SDF needed):
/// the disc's principal ML must agree with the vertebral frame ML (rung-7's own
/// obliquity cross-check), AND the disc must be seated between the two body
/// centres along the superior axis (a viewer-added guard rung-7 does not assert).
/// A grossly mismatched or misaligned trio (wrong specimen, wrong disc level,
/// rotated frame) trips these — so it is never shown as a literature-validated FSU.
///
/// The ML agreement mirrors rung-7's own obliquity assert (`disc_ml · frame.ml >
/// 0.9`): `disc_ml` is a coordinate axis, `frame.ml` continuous, so the 0.9 band
/// assumes the anatomy sits roughly axis-aligned — true for the BodyParts3D
/// frame this tool targets (rung 7 measured ~0.95), not for arbitrary rotations.
/// Note it does NOT detect an L4↔L5 swap: that renders valid geometry at true
/// positions with only the tissue labels flipped (the guards stay in-range).
fn coregistration_warnings(
    disc: &IndexedMesh,
    frame: &SegmentFrame,
    disc_ml: Vector3<f64>,
) -> Vec<String> {
    let mut warnings = Vec::new();

    let agreement = disc_ml.dot(&frame.ml).abs();
    if agreement < 0.9 {
        warnings.push(format!(
            "disc ML vs vertebral ML agreement {agreement:.2} < 0.90 — meshes may be misaligned or from different specimens"
        ));
    }

    // Disc centre must sit between the L4/L5 body centres along the SI axis.
    let bbox = Aabb::from_points(disc.vertices.iter());
    let disc_center = Point3::from(bbox.min.coords + (bbox.max - bbox.min) * 0.5);
    let along = (disc_center - frame.b5).dot(&frame.superior_axis);
    let separation = (frame.b4 - frame.b5).dot(&frame.superior_axis);
    if along < 0.0 || along > separation {
        warnings.push(format!(
            "disc centre is not between the L4/L5 bodies ({along:.1} mm of {separation:.1} mm along SI) — wrong disc level?"
        ));
    }

    warnings
}

/// Facet near-contact points at the NEUTRAL pose (both vertebrae at identity).
/// `compute_shape_contact` returns every pair within the detection margin
/// (`FACET_CELL` = 1 mm); at neutral the articular surfaces nearly touch (rung
/// 4b measured a small positive gap, no interpenetration) so these are the
/// near-contact facet region, not interpenetrations — we render them all rather
/// than filtering `penetration > 0` (which is the force-relevant subset, empty
/// at neutral).
fn facet_contact_points(g4: &Arc<SdfGrid>, g5: &Arc<SdfGrid>) -> Vec<Point3<f64>> {
    let identity = Pose {
        position: Point3::origin(),
        rotation: UnitQuaternion::identity(),
    };
    let contacts: Vec<SdfContact> = compute_shape_contact(
        &ShapeConcave::new(Arc::clone(g4)),
        &identity,
        &ShapeConcave::new(Arc::clone(g5)),
        &identity,
        FACET_CELL,
        FACET_MAX_CONTACTS,
    );
    contacts.iter().map(|c| c.point).collect()
}

fn combined_aabb(meshes: &[&IndexedMesh]) -> Aabb {
    Aabb::from_points(meshes.iter().flat_map(|m| m.vertices.iter()))
}

/// Evenly-spaced applied moments across `[-MAX_MOMENT, +MAX_MOMENT]` (N·m), ascending
/// (extension → flexion) so the replay sweeps the segment open smoothly.
fn moment_ramp() -> Vec<f64> {
    (0..N_RAMP_FRAMES)
        .map(|i| {
            #[allow(clippy::cast_precision_loss)] // N_RAMP_FRAMES is tiny; the ratio is exact.
            let t = i as f64 / (N_RAMP_FRAMES - 1) as f64; // 0..=1
            -MAX_MOMENT + t * (2.0 * MAX_MOMENT)
        })
        .collect()
}

/// Assemble the coupled FSU (disc bushing + ligament tendons + oriented facet contact)
/// and capture its force-driven equilibrium over a pure-moment ramp as a replayable
/// trajectory (native mm). Consumes its own copy of the disc mesh (the disc build
/// recentres + scales it destructively).
///
/// The disc's deformation at each equilibrium angle is the sub-degree FEM field linearly
/// extrapolated to that angle (the same `k_disc` linearity rung 7 relies on) — the disc
/// only converges sub-degree, so this is the honest larger-angle disc shape. The bones
/// stop on the facets in extension because the solved angle is ROM-limited, not exaggerated.
fn capture_coupled(
    l4: &IndexedMesh,
    l5: &IndexedMesh,
    disc: IndexedMesh,
) -> Result<CoupledTrajectory> {
    let mut fsu = CoupledFsu::build(l4, l5, disc, &CoupledParams::default())?;
    let traj = fsu.capture_ramp(&moment_ramp());
    // Guard against a disc that tet-meshed to nothing (all components dropped): with no
    // boundary faces, `nearest_tet_nodes` would silently map every surface vertex to node
    // 0 and render a collapsed spike. Fail loud instead of showing a wrong disc.
    anyhow::ensure!(
        !traj.boundary_faces.is_empty() && !traj.rest_nodes_native.is_empty(),
        "disc tet-mesh produced no surface (degenerate/near-flat disc mesh?) — cannot render"
    );
    let (ext, flex) = (
        traj.frames.first().map_or(0.0, |f| f.theta.to_degrees()),
        traj.frames.last().map_or(0.0, |f| f.theta.to_degrees()),
    );
    println!(
        "coupled ramp: {} frames, ±{MAX_MOMENT} N·m → extension {ext:.2}° … flexion {flex:.2}°; \
         disc surface {} nodes, {} boundary faces",
        traj.frames.len(),
        traj.rest_nodes_native.len(),
        traj.boundary_faces.len()
    );
    Ok(traj)
}

/// For each `surface` vertex, the index of the nearest tet node **on the disc's rendered
/// boundary** (`boundary_faces` — the largest connected component's surface nodes).
///
/// Restricting candidates to the boundary nodes matters for correctness, not just speed:
/// the dropped rim-island nodes stay at rest (zero displacement), so mapping a surface
/// vertex onto one would tear the disc under exaggeration; the boundary nodes all carry a
/// real, smooth, connected FEM displacement. It also shrinks the brute-force pass ~5× (a
/// one-time startup cost, dwarfed by the sweep's FEM solves). The matched node's
/// displacement is applied to that surface vertex each frame.
fn nearest_tet_nodes(
    surface: &IndexedMesh,
    rest_nodes: &[Point3<f64>],
    boundary_faces: &[[VertexId; 3]],
) -> Vec<usize> {
    let mut candidates: Vec<usize> = boundary_faces
        .iter()
        .flatten()
        .map(|&v| v as usize)
        .collect();
    candidates.sort_unstable();
    candidates.dedup();
    surface
        .vertices
        .iter()
        .map(|v| {
            candidates
                .iter()
                .copied()
                .min_by(|&a, &b| {
                    (rest_nodes[a] - v)
                        .norm_squared()
                        .total_cmp(&(rest_nodes[b] - v).norm_squared())
                })
                .unwrap_or(0)
        })
        .collect()
}

/// Assemble the static FSU scene from the three STL paths (native mm).
pub fn build(l4_path: &Path, l5_path: &Path, disc_path: &Path) -> Result<FsuScene> {
    let l4 = load(l4_path)?;
    let l5 = load(l5_path)?;
    let disc = load(disc_path)?;
    // Per-mesh load summary. The crate's `load` is a pure (silent) library call,
    // so the viewer reports the post-repair size here as the build-blind sanity
    // signal — a broken/degenerate mesh shows up as an implausible vertex count.
    // (The raw pre-weld count the old inline loader printed is a repair internal:
    // always large for STL triangle soup, so it doesn't distinguish good meshes
    // from bad; the post-repair count does.)
    for (name, mesh) in [("L4", &l4), ("L5", &l5), ("disc", &disc)] {
        println!(
            "loaded {name}: {} verts / {} faces",
            mesh.vertices.len(),
            mesh.faces.len()
        );
    }

    let o4 = oracle(&l4)?;
    let o5 = oracle(&l5)?;
    let frame = segment_frame(&l4, &l5, &o4, &o5)?;

    // The disc's own principal axis is the canonical ML (rung-7): use it for
    // both the ligament midline filter and the co-registration cross-check.
    let disc_ml = disc_principal_ml(&disc);
    let mut warnings = coregistration_warnings(&disc, &frame, disc_ml);
    let ligaments = build_ligaments(&l4, &l5, &frame, disc_ml, &mut warnings);

    let g4 = facet_grid(&l4, &o4);
    let g5 = facet_grid(&l5, &o5);
    let facet_contacts = facet_contact_points(&g4, &g5);
    for lig in &ligaments {
        println!(
            "ligament {}: L5 site {:.1?} → L4 site {:.1?}  ({:.1} mm)",
            lig.name,
            lig.inferior.coords.as_slice(),
            lig.superior.coords.as_slice(),
            (lig.superior - lig.inferior).norm()
        );
    }
    println!(
        "assembled FSU: {} facet near-contact points",
        facet_contacts.len()
    );
    for w in &warnings {
        eprintln!("WARNING: {w}");
    }

    // Frame the camera on all three tissues, then capture the coupled force-driven ramp.
    // The disc STL is kept as the (clean) render surface; `capture_coupled` consumes a
    // clone for the disc FEM tet-mesh + the coupled solve.
    let aabb = combined_aabb(&[&l4, &l5, &disc]);
    println!("capturing coupled FSU moment ramp ({N_RAMP_FRAMES} frames, ±{MAX_MOMENT} N·m)…");
    let flexion = capture_coupled(&l4, &l5, disc.clone())?;
    println!("captured {} coupled frames", flexion.frames.len());

    // Map each clean-STL vertex to its nearest connected-component boundary tet node, so
    // the smooth surface can be displaced by the FEM field per frame (the tet boundary is
    // too fragmented to draw directly).
    let disc_node_map =
        nearest_tet_nodes(&disc, &flexion.rest_nodes_native, &flexion.boundary_faces);

    Ok(FsuScene {
        l4,
        l5,
        disc_surface: disc,
        disc_node_map,
        flexion,
        ligaments,
        facet_contacts,
        aabb,
        warnings,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A vertices-only mesh (the coordinate-free helpers read only `vertices`).
    /// A single dummy face is added only when there are ≥3 vertices, so the
    /// helper can't hand back an out-of-bounds face index to a future test.
    fn mesh(verts: &[[f64; 3]]) -> IndexedMesh {
        let faces = if verts.len() >= 3 {
            vec![[0, 1, 2]]
        } else {
            vec![]
        };
        IndexedMesh {
            vertices: verts
                .iter()
                .map(|&[x, y, z]| Point3::new(x, y, z))
                .collect(),
            faces,
        }
    }

    /// Build a `SegmentFrame` directly (bypassing the SDF oracles) so the pure
    /// co-registration math can be tested without the licensed meshes.
    fn frame(b5: [f64; 3], b4: [f64; 3], ml: Vector3<f64>) -> SegmentFrame {
        let b5 = Point3::new(b5[0], b5[1], b5[2]);
        let b4 = Point3::new(b4[0], b4[1], b4[2]);
        SegmentFrame {
            b4,
            b5,
            superior_axis: (b4 - b5).normalize(),
            posterior: Vector3::y(), // unused by coregistration_warnings
            ml,
        }
    }

    #[test]
    fn combined_aabb_spans_every_mesh() {
        let a = mesh(&[[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [0.5, 0.5, 0.5]]);
        let b = mesh(&[[-2.0, 0.0, 0.0], [3.0, 4.0, 5.0], [1.0, 1.0, 1.0]]);
        let bb = combined_aabb(&[&a, &b]);
        assert_eq!(bb.min, Point3::new(-2.0, 0.0, 0.0));
        assert_eq!(bb.max, Point3::new(3.0, 4.0, 5.0));
    }

    #[test]
    fn disc_principal_ml_is_widest_extent_axis() {
        // Extents x=2, y=12, z=1 → widest is Y → ML snaps to the +Y axis.
        let m = mesh(&[[0.0, -2.0, 0.0], [2.0, 10.0, 1.0]]);
        assert_eq!(disc_principal_ml(&m), Vector3::y());
    }

    #[test]
    fn disc_principal_ml_uses_aabb_extent_not_pca() {
        // The tool deliberately uses the widest AABB EXTENT (not a PCA principal
        // axis): this cloud's longest point-to-point direction is the x-y diagonal,
        // but its widest axis-aligned extent is x (span 10 vs y-span 3) → +X.
        let m = mesh(&[[0.0, 0.0, 0.0], [10.0, 3.0, 1.0]]);
        assert_eq!(disc_principal_ml(&m), Vector3::x());
    }

    #[test]
    fn coregistration_clean_when_aligned_and_seated() {
        // L5→L4 along +z; disc widest along x, centred midway between the bodies.
        let f = frame([0.0, 0.0, 0.0], [0.0, 0.0, 20.0], Vector3::x());
        let disc = mesh(&[[-10.0, -5.0, 9.0], [10.0, 5.0, 11.0]]); // ML x; centre (0,0,10)
        let disc_ml = disc_principal_ml(&disc);
        assert!(
            coregistration_warnings(&disc, &f, disc_ml).is_empty(),
            "an aligned, seated trio must produce no warnings"
        );
    }

    #[test]
    fn coregistration_warns_on_ml_disagreement() {
        // Vertebral ML is +y but the disc's widest extent (→ ML) is +x: dot 0 < 0.9.
        let f = frame([0.0, 0.0, 0.0], [0.0, 0.0, 20.0], Vector3::y());
        let disc = mesh(&[[-10.0, -1.0, 9.0], [10.0, 1.0, 11.0]]); // ML x; centre (0,0,10) seated
        let w = coregistration_warnings(&disc, &f, disc_principal_ml(&disc));
        // Exactly the ML warning — the disc is seated, so the between-check stays silent.
        assert_eq!(
            w.len(),
            1,
            "expected only the ML-agreement warning, got {w:?}"
        );
        assert!(w[0].contains("agreement"), "wrong warning: {w:?}");
    }

    #[test]
    fn coregistration_warns_when_disc_below_l5() {
        // Disc centred BELOW L5 along the SI axis → `along` negative → "not between".
        let f = frame([0.0, 0.0, 0.0], [0.0, 0.0, 20.0], Vector3::x());
        let disc = mesh(&[[-10.0, -5.0, -15.0], [10.0, 5.0, -13.0]]); // centre z=-14
        let w = coregistration_warnings(&disc, &f, disc_principal_ml(&disc));
        assert_eq!(
            w.len(),
            1,
            "expected only the not-between warning, got {w:?}"
        );
        assert!(w[0].contains("not between"), "wrong warning: {w:?}");
    }

    #[test]
    fn coregistration_warns_when_disc_above_l4() {
        // The OTHER half of the seating guard: disc centred ABOVE L4 (along >
        // separation) must also warn — guards against dropping `|| along > sep`.
        let f = frame([0.0, 0.0, 0.0], [0.0, 0.0, 20.0], Vector3::x());
        let disc = mesh(&[[-10.0, -5.0, 29.0], [10.0, 5.0, 31.0]]); // centre z=30 > sep 20
        let w = coregistration_warnings(&disc, &f, disc_principal_ml(&disc));
        assert_eq!(
            w.len(),
            1,
            "expected only the not-between warning, got {w:?}"
        );
        assert!(w[0].contains("not between"), "wrong warning: {w:?}");
    }

    #[test]
    fn coregistration_clean_at_seating_boundary() {
        // The seating test is strict (`< 0` / `> separation`): a disc centred
        // exactly on L5 (along == 0) is seated, so NO warning — a `<=` typo bites.
        let f = frame([0.0, 0.0, 0.0], [0.0, 0.0, 20.0], Vector3::x());
        let disc = mesh(&[[-10.0, -5.0, -1.0], [10.0, 5.0, 1.0]]); // centre z=0 == b5
        let w = coregistration_warnings(&disc, &f, disc_principal_ml(&disc));
        assert!(w.is_empty(), "on-boundary disc must not warn, got {w:?}");
    }
}
