//! Headless FSU scene builder (Bevy-free).
//!
//! Assembles the L4–L5 Functional Spinal Unit as plain data for rendering, everything in
//! native BodyParts3D millimetres. The heavy lifting is [`cf_fsu_model::CoupledFsu`], built
//! ONCE here: it owns the anatomical frame, the two articular SDF grids, the flexion axis,
//! and the bonded disc, and it captures the force-driven moment-ramp
//! ([`CoupledFsu::capture_ramp`]) whose frames carry the disc deformation + the real engaged
//! facet contact points. The scene reuses the FSU's frame + flexion axis for its overlays,
//! so nothing (oracle / segment-frame / facet-grid / ML axis) is computed twice.
//!
//! This module keeps the viewer-specific assembly on top of the FSU: the two ligament lines
//! (field-derived attachment sites), the co-registration warnings, and the `build`
//! orchestrator that stitches the FSU + overlays + render surfaces into an [`FsuScene`].

use std::path::Path;

use anyhow::{Result, ensure};
use cf_fsu_geometry::{BODY_RADIUS, SegmentFrame, extreme_vertex, load};
use cf_fsu_model::{CoupledFsu, CoupledParams, CoupledTrajectory, VertexId};
use mesh_types::{Aabb, IndexedMesh};
use nalgebra::{Point3, Vector3};

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
    /// Combined bounding box across all three meshes, for camera framing.
    pub aabb: Aabb,
    /// Co-registration / degeneracy warnings surfaced during assembly (empty
    /// for a well-formed, co-registered trio). Shown in the panel + on stderr
    /// so a misaligned or mismatched input is never presented as a valid FSU.
    pub warnings: Vec<String>,
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

    // Assemble the coupled FSU ONCE. It builds and owns the anatomical frame, the two
    // articular SDF grids, the flexion-oriented ML axis, and the bonded disc — the viewer's
    // overlays reuse those (no second oracle / segment-frame / facet-grid / ML-axis
    // computation), and the disc's own `ml_axis` is the single source of the flexion axis.
    println!(
        "assembling coupled FSU + capturing moment ramp ({N_RAMP_FRAMES} frames, ±{MAX_MOMENT} N·m)…"
    );
    let mut fsu = CoupledFsu::build(&l4, &l5, disc.clone(), &CoupledParams::default())?;

    // Overlays derived from the coupled FSU's shared frame + flexion axis.
    let mut warnings = coregistration_warnings(&disc, fsu.frame(), fsu.axis());
    let ligaments = build_ligaments(&l4, &l5, fsu.frame(), fsu.axis(), &mut warnings);
    for lig in &ligaments {
        println!(
            "ligament {}: L5 site {:.1?} → L4 site {:.1?}  ({:.1} mm)",
            lig.name,
            lig.inferior.coords.as_slice(),
            lig.superior.coords.as_slice(),
            (lig.superior - lig.inferior).norm()
        );
    }
    for w in &warnings {
        eprintln!("WARNING: {w}");
    }

    // Capture the coupled force-driven ramp. The disc STL stays as the clean render surface;
    // the FSU already consumed a clone for the FEM tet-mesh + coupled solve.
    let flexion = fsu.capture_ramp(&moment_ramp())?;
    // Guard against a disc that tet-meshed to nothing (all components dropped): with no
    // boundary faces, `nearest_tet_nodes` would silently collapse the disc onto node 0.
    ensure!(
        !flexion.boundary_faces.is_empty() && !flexion.rest_nodes_native.is_empty(),
        "disc tet-mesh produced no surface (degenerate/near-flat disc mesh?) — cannot render"
    );
    let (ext, flex) = (
        flexion.frames.first().map_or(0.0, |f| f.theta.to_degrees()),
        flexion.frames.last().map_or(0.0, |f| f.theta.to_degrees()),
    );
    println!(
        "coupled ramp: {} frames → extension {ext:.2}° … flexion {flex:.2}°; \
         disc surface {} nodes, {} boundary faces",
        flexion.frames.len(),
        flexion.rest_nodes_native.len(),
        flexion.boundary_faces.len()
    );

    // Map each clean-STL vertex to its nearest connected-component boundary tet node, so
    // the smooth surface can be displaced by the FEM field per frame (the tet boundary is
    // too fragmented to draw directly).
    let aabb = combined_aabb(&[&l4, &l5, &disc]);
    let disc_node_map =
        nearest_tet_nodes(&disc, &flexion.rest_nodes_native, &flexion.boundary_faces);

    Ok(FsuScene {
        l4,
        l5,
        disc_surface: disc,
        disc_node_map,
        flexion,
        ligaments,
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
    fn coregistration_clean_when_aligned_and_seated() {
        // L5→L4 along +z; disc ML x (widest extent — the flexion axis the coupled FSU
        // supplies from `BondedDisc::ml_axis`), centred midway between the bodies.
        let f = frame([0.0, 0.0, 0.0], [0.0, 0.0, 20.0], Vector3::x());
        let disc = mesh(&[[-10.0, -5.0, 9.0], [10.0, 5.0, 11.0]]); // centre (0,0,10)
        assert!(
            coregistration_warnings(&disc, &f, Vector3::x()).is_empty(),
            "an aligned, seated trio must produce no warnings"
        );
    }

    #[test]
    fn coregistration_warns_on_ml_disagreement() {
        // Vertebral ML is +y but the disc ML is +x: dot 0 < 0.9.
        let f = frame([0.0, 0.0, 0.0], [0.0, 0.0, 20.0], Vector3::y());
        let disc = mesh(&[[-10.0, -1.0, 9.0], [10.0, 1.0, 11.0]]); // centre (0,0,10) seated
        let w = coregistration_warnings(&disc, &f, Vector3::x());
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
        let w = coregistration_warnings(&disc, &f, Vector3::x());
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
        let w = coregistration_warnings(&disc, &f, Vector3::x());
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
        let w = coregistration_warnings(&disc, &f, Vector3::x());
        assert!(w.is_empty(), "on-boundary disc must not warn, got {w:?}");
    }
}
