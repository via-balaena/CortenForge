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
//! (field-derived attachment sites), the co-registration warnings, and the
//! `build_from_meshes` orchestrator that stitches the FSU + overlays + render surfaces
//! into an [`FsuScene`].

use anyhow::{Result, ensure};
use cf_fsu_geometry::{BODY_RADIUS, MeshOracle, SegmentFrame, extreme_vertex, oracle};
use cf_fsu_model::{
    CoupledFsu, CoupledParams, CoupledTrajectory, PHYSIOLOGIC_MOMENT, RAMP_FRAMES, VertexId,
    moment_ramp,
};
use mesh_types::{Aabb, IndexedMesh};
use nalgebra::{Point3, Vector3};

/// A ligament rendered as a straight line between two field-derived sites.
pub struct Ligament {
    pub name: &'static str,
    pub inferior: Point3<f64>, // attachment on L5
    pub superior: Point3<f64>, // attachment on L4
}

/// The assembled scene, as plain geometry in native millimetres.
pub struct FsuScene {
    /// The disc render surface (native mm) — the clean STL lens with its endplate bands
    /// **conformed onto the real L4/L5 surfaces** ([`CoupledFsu::conformed_disc_surface`]),
    /// so the drawn disc coincides with the bone at the endplates (rendered === contacts).
    /// The tet mesh is too fragmented to render as a coherent disc, so the viewer draws THIS
    /// surface and displaces it by the FEM field (see `disc_node_weights`).
    pub disc_surface: IndexedMesh,
    /// For each `disc_surface` vertex, the `K` nearest boundary tet nodes (indices into
    /// [`CoupledTrajectory::rest_nodes_native`]) with inverse-distance-squared weights summing to 1.
    /// Per frame the vertex is displaced by the WEIGHTED BLEND of those nodes' FEM
    /// displacements — smooth C⁰ skinning, so the fine surface does not facet/tear over the
    /// coarse (few-mm) tet field the way a single nearest-node lookup does.
    pub disc_node_weights: Vec<Vec<(usize, f64)>>,
    /// The L4/L5 signed-distance oracles — the viewer projects any deformed disc vertex that
    /// lands INSIDE a bone back onto its surface (the disc FEM bonds to boxes, not the bones, so
    /// its annulus bulges freely and would otherwise pierce the vertebra on the compression
    /// side; a real disc can't — the bone stops it). Built here so the fallible construction
    /// stays in the `Result`-returning assembly, not the infallible Bevy driver.
    pub o4: MeshOracle,
    pub o5: MeshOracle,
    /// The coupled FSU's captured force-driven ramp: per-frame equilibrium angle +
    /// deformed disc tet-node positions (native mm) + facet engagement + pivot/axis. The
    /// disc FEM displacement field is sampled onto `disc_surface` via `disc_node_weights`.
    pub flexion: CoupledTrajectory,
    pub ligaments: Vec<Ligament>,
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

/// Number of tet nodes each surface vertex blends over. Enough to interpolate smoothly across
/// the coarse (few-mm) tet field without reaching past the local neighbourhood.
const SKIN_NEIGHBOURS: usize = 6;

/// For each `surface` vertex, the [`SKIN_NEIGHBOURS`] nearest tet nodes **on the disc's
/// rendered boundary** (`boundary_faces` — the largest connected component's surface nodes),
/// with inverse-distance-squared weights normalised to sum to 1.
///
/// Restricting candidates to the boundary nodes matters for correctness, not just speed:
/// the dropped rim-island nodes stay at rest (zero displacement), so blending a surface
/// vertex onto one would tear the disc; the boundary nodes all carry a real, smooth,
/// connected FEM displacement. Blending several (vs the single nearest) skins the fine
/// surface **smoothly** over the coarse tet field — a single nearest-node lookup gives each
/// vertex one node's displacement verbatim, so the surface facets/tears at the bone interface
/// where the displacement gradient is steepest.
fn weighted_tet_nodes(
    surface: &IndexedMesh,
    rest_nodes: &[Point3<f64>],
    boundary_faces: &[[VertexId; 3]],
) -> Vec<Vec<(usize, f64)>> {
    let mut candidates: Vec<usize> = boundary_faces
        .iter()
        .flatten()
        .map(|&v| v as usize)
        .collect();
    candidates.sort_unstable();
    candidates.dedup();
    // Distance below which a surface vertex is taken as coincident with a node (avoids a
    // divide-by-zero and lets an exactly-matched vertex track that node rigidly).
    const EPS2: f64 = 1e-12;
    surface
        .vertices
        .iter()
        .map(|v| {
            // K nearest candidates by rest distance.
            let mut near: Vec<(usize, f64)> = candidates
                .iter()
                .map(|&i| (i, (rest_nodes[i] - v).norm_squared()))
                .collect();
            let k = SKIN_NEIGHBOURS.min(near.len());
            near.select_nth_unstable_by(k - 1, |a, b| a.1.total_cmp(&b.1));
            near.truncate(k);
            // Inverse-distance-squared weights, normalised to sum to 1.
            let raw: Vec<(usize, f64)> =
                near.iter().map(|&(i, d2)| (i, 1.0 / (d2 + EPS2))).collect();
            let total: f64 = raw.iter().map(|&(_, w)| w).sum();
            raw.into_iter().map(|(i, w)| (i, w / total)).collect()
        })
        .collect()
}

/// Assemble the static FSU scene from three **in-memory** meshes (native mm) —
/// the app loads L4/L5 + the disc and runs this on demand (the disc becomes a
/// painted + lofted mesh once the paint front-end lands). Heavy (~90 s in
/// release): tet-mesh + SDF grids + the moment-ramp capture, so the app dispatches
/// it to a background thread.
pub fn build_from_meshes(l4: IndexedMesh, l5: IndexedMesh, disc: IndexedMesh) -> Result<FsuScene> {
    // Per-mesh size summary as the build-blind sanity signal — a broken or
    // degenerate mesh shows up as an implausible vertex count. For loaded meshes
    // this is the post-repair size (`load` weld-repairs silently); for a painted
    // disc it is the loft output. (The raw pre-weld triangle-soup count is a
    // repair internal — always large — so it doesn't distinguish good from bad.)
    for (name, mesh) in [("L4", &l4), ("L5", &l5), ("disc", &disc)] {
        println!(
            "mesh {name}: {} verts / {} faces",
            mesh.vertices.len(),
            mesh.faces.len()
        );
    }

    // Assemble the coupled FSU ONCE. It builds and owns the anatomical frame, the two
    // articular SDF grids, the flexion-oriented ML axis, and the bonded disc — the viewer's
    // overlays reuse those (no second oracle / segment-frame / facet-grid / ML-axis
    // computation), and the disc's own `ml_axis` is the single source of the flexion axis.
    println!(
        "assembling coupled FSU + capturing moment ramp ({RAMP_FRAMES} frames, ±{PHYSIOLOGIC_MOMENT} N·m)…"
    );
    let mut fsu = CoupledFsu::build(&l4, &l5, &disc, &CoupledParams::default())?;
    // The disc render surface = the FSU's disc conformed onto the real endplates (native
    // mm). Drawing THIS (not the raw STL) is what makes rendered === contacts: the disc's
    // endplate bands now sit on the bone, closing the disc-lift seam. Cloned once here so
    // the later `capture_ramp` can borrow `fsu` mutably.
    let disc_surface = fsu.conformed_disc_surface().clone();

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

    // Capture the coupled force-driven ramp. `disc_surface` (the conformed render surface)
    // is already captured above; the FSU tet-meshes + solves on the same conformed disc.
    let flexion = fsu.capture_ramp(&moment_ramp())?;
    // Guard against a disc that tet-meshed to nothing (all components dropped): with no
    // boundary faces, `weighted_tet_nodes` would silently collapse the disc onto node 0.
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

    // Skin each conformed-surface vertex to its nearest boundary tet nodes (inverse-distance-squared
    // weighted), so the smooth surface deforms by a blended FEM field per frame (the tet
    // boundary is too fragmented to draw directly). Blending several nodes keeps the fine
    // surface from faceting over the coarse tet field at the bone interface.
    let disc_node_weights = weighted_tet_nodes(
        &disc_surface,
        &flexion.rest_nodes_native,
        &flexion.boundary_faces,
    );
    // Bone oracles for the viewer's per-frame disc-vs-bone non-penetration projection.
    let o4 = oracle(&l4)?;
    let o5 = oracle(&l5)?;

    Ok(FsuScene {
        disc_surface,
        disc_node_weights,
        o4,
        o5,
        flexion,
        ligaments,
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
