//! Post-MC mesh-CSG stage for cf-cast's mating-features architectural fix.
//!
//! `docs/CF_CAST_MATING_FEATURES_PLAN.md` (S3) introduces a transform
//! stage between [`crate::mesher::solid_to_mm_mesh`] and the F4
//! printability gate. The stage applies exact-CAD primitives — true
//! cylinders + true half-spaces — as mesh-CSG operations against the
//! MC-output mold pieces, replacing the SDF/MC encoding that iter-1
//! print falsified for mating surfaces (cup-piece seams not flat
//! enough to seal; pin protrusion ≠ socket depth; T-bar misfit).
//!
//! S3 ships the empty-Vec pass-through: every composer emits
//! `(Solid, Vec<MatingTransform>)` with the Vec empty. S4 (seam
//! plane), S5 (registration pins), and S6 (T-bar + plug-pin) populate
//! the Vec with concrete transforms. See decision §1 + §11 of
//! `docs/CF_CAST_MATING_FEATURES_RECON.md` for the full surface.
//!
//! # Unit boundary
//!
//! Inputs (`IndexedMesh`) are in **millimeter world coordinates** —
//! the convention `solid_to_mm_mesh` emits. [`CylinderParent`] and
//! [`MatingTransform`] carry their geometry in **meters** (cf-design's
//! compose-time frame); the builders convert m → mm at primitive build
//! time so the manifold ops compose with the input mesh directly
//! (recon §3 Approach B).
//!
//! # Determinism contract
//!
//! Two cylinder primitives built from the same [`CylinderParent`],
//! same `segments`, and same `radius_m` are bit-equal across calls
//! and across pieces (recon §2 + §10 S2-C). Same parent + same
//! `segments` with a *different* `radius_m` produces meshes whose
//! vertex layout matches face-for-face modulo the radial scale —
//! exactly what the cross-piece pin/socket fit invariant requires.

use std::collections::HashMap;

use manifold3d::Manifold;
use mesh_types::IndexedMesh;
use nalgebra::{Isometry3, Point3, Rotation3, Translation3, UnitVector3, Vector3};

use crate::error::{CastError, CastTarget};
use crate::mesher::METERS_TO_MM;

/// Cross-piece shared geometry triple.
///
/// Pin and socket of a registration pair, or T-bar/T-slot of a
/// plug+cup pair, both derive their cylinder primitive from the
/// same `CylinderParent`; only the per-side
/// [`CylinderParams::radius_m`] differs (recon §2).
///
/// All coordinates are meters in cf-design's compose-time frame.
/// The builders convert m → mm at primitive-build time.
#[derive(Debug, Clone, PartialEq)]
pub struct CylinderParent {
    /// Cylinder centre (meters, world coords).
    pub center_m: Point3<f64>,
    /// Cylinder long-axis direction. Unit-normalized; bit-equality
    /// across pieces requires bit-equal axis components.
    pub axis: UnitVector3<f64>,
    /// Half the cylinder's length along [`Self::axis`] (meters).
    pub half_length_m: f64,
}

/// Per-operation cylinder payload.
///
/// A shared [`CylinderParent`] plus the per-side radius and facet
/// count. The `segments` field is part of the determinism contract:
/// bit-equal `segments` is required for bit-equal output across
/// pieces.
#[derive(Debug, Clone, PartialEq)]
pub struct CylinderParams {
    /// Shared geometry triple — identical between pin and socket of
    /// a registration pair, plug and cup of a T-bar pair, etc.
    pub parent: CylinderParent,
    /// Per-side radius (meters). Pin and socket of a registration
    /// pair consume *different* values here: pin uses the pin radius
    /// (e.g., [`crate::PinSpec::pin_radius_m`]), socket uses
    /// pin radius + `diametral_clearance_m / 2`. Geometric match
    /// across pieces comes from the shared [`Self::parent`] pose +
    /// length, not from radius equality.
    pub radius_m: f64,
    /// Polygonal facet count around the circumference. Determinism
    /// contract: same parent + same `segments` → bit-equal output.
    pub segments: u32,
}

/// One mating-feature mesh-CSG operation.
///
/// Applied between [`crate::mesher::solid_to_mm_mesh`] and the F4
/// printability gate. Transforms are applied in declared order;
/// S3 ships the empty-Vec pass-through, S4/S5/S6 populate concrete
/// variants.
#[derive(Debug, Clone, PartialEq)]
pub enum MatingTransform {
    /// Trim the mesh against an exact plane defined as
    /// `dot(normal, p) = offset_m` (meters world coords). The
    /// `normal` direction points toward the half-space to **keep**;
    /// matter on the opposite side is removed. S4 (seam plane)
    /// emits one of these per cup piece.
    SeamTrim {
        /// Outward-pointing unit normal of the kept half-space.
        normal: UnitVector3<f64>,
        /// Plane offset along `normal` (meters). The plane equation
        /// is `dot(normal, p) = offset_m`.
        offset_m: f64,
    },
    /// Mesh-union of an exact axis-aligned cylinder primitive.
    /// S5 emits one per Negative-side registration pin; S6 emits
    /// one for the plug-side T-bar + plug-pin shaft.
    UnionCylinder {
        /// Cylinder geometry payload.
        params: CylinderParams,
    },
    /// Mesh-subtract of an exact axis-aligned cylinder primitive.
    /// S5 emits one per Positive-side registration socket; S6 emits
    /// the cup-side T-slot + plug-pin socket carves.
    SubtractCylinder {
        /// Cylinder geometry payload.
        params: CylinderParams,
    },
}

/// Apply mating-feature mesh-CSG operations between marching-cubes
/// output and the F4 printability gate.
///
/// Empty `transforms` short-circuits to `Ok(mesh)` unchanged — the
/// pure pass-through path S3 ships before S4/S5/S6 begin emitting
/// concrete transforms.
///
/// Inputs and outputs are in **millimeter world coordinates**;
/// transform geometry is in meters (converted at primitive-build time).
///
/// # Errors
///
/// Returns [`CastError::MeshCsg`] if the input mesh fails manifold3d's
/// manifoldness validation on entry. The transform-apply ops (union,
/// difference, trim) are infallible at manifold3d's API surface and
/// produce a (possibly empty) `Manifold`; downstream F4 will flag any
/// pathology in the geometric result.
pub fn apply_mating_transforms(
    mesh: IndexedMesh,
    transforms: &[MatingTransform],
    target: CastTarget,
) -> Result<IndexedMesh, CastError> {
    if transforms.is_empty() {
        return Ok(mesh);
    }
    let mut manifold = indexed_mesh_to_manifold(&mesh).map_err(|source| CastError::MeshCsg {
        target,
        context: "mesh→manifold conversion".to_string(),
        source,
    })?;
    for transform in transforms {
        manifold = apply_one(&manifold, transform);
    }
    Ok(manifold_to_indexed_mesh(&manifold))
}

/// Apply one transform to a manifold, returning the result. All S3
/// variants are infallible at manifold3d's API surface; a fallible
/// S4+ variant would change this signature to `Result`.
fn apply_one(m: &Manifold, t: &MatingTransform) -> Manifold {
    match t {
        MatingTransform::SeamTrim { normal, offset_m } => {
            // Mesh is in mm world coords; offset_m is in meters.
            // Convention match: manifold3d's `trim_by_plane` keeps
            // the half-space `n · p > offset`, which is also our
            // "keep" direction.
            let offset_mm = offset_m * METERS_TO_MM;
            m.trim_by_plane_nalgebra(&normal.into_inner(), offset_mm)
        }
        MatingTransform::UnionCylinder { params } => {
            let cyl = build_cylinder_along_axis(&params.parent, params.radius_m, params.segments);
            m.union(&cyl)
        }
        MatingTransform::SubtractCylinder { params } => {
            let cyl = build_cylinder_along_axis(&params.parent, params.radius_m, params.segments);
            m.difference(&cyl)
        }
    }
}

/// Build a [`Manifold`] cylinder primitive aligned to `parent.axis`,
/// centred at `parent.center_m`, with `radius_m` and length
/// `2 × parent.half_length_m`.
///
/// `segments` is the polygonal facet count around the circumference
/// and is part of the determinism contract — same parent + radius +
/// segments → bit-equal output across calls and across pieces
/// (recon §2). m → mm conversion happens here so the returned
/// manifold composes directly with mm-scale `IndexedMesh` input.
#[must_use]
pub fn build_cylinder_along_axis(
    parent: &CylinderParent,
    radius_m: f64,
    segments: u32,
) -> Manifold {
    let length_mm = 2.0 * parent.half_length_m * METERS_TO_MM;
    let radius_mm = radius_m * METERS_TO_MM;
    // Production `segments` values are < 256 (32-segment cylinders
    // are the workshop default); the `i32` ceiling is reachable only
    // by degenerate input. Clip rather than panic — the resulting
    // Manifold would be unusable either way.
    let segments_i32 = i32::try_from(segments).unwrap_or(i32::MAX);

    // manifold3d's `cylinder` primitive is Z-axis-aligned centred at
    // origin when `center=true`.
    let z_aligned = Manifold::cylinder(length_mm, radius_mm, radius_mm, segments_i32, true);

    let center_mm = Point3::new(
        parent.center_m.x * METERS_TO_MM,
        parent.center_m.y * METERS_TO_MM,
        parent.center_m.z * METERS_TO_MM,
    );
    let iso = pose_from_z_axis(parent.axis, center_mm);
    z_aligned.transform(&affine_12_from_isometry(&iso))
}

/// Build a slab `Manifold` for half-space intersection.
///
/// The slab is oriented so its `+normal`-facing face passes through
/// `plane_point` and its body extends `2 × slab_half_thickness_m`
/// into the `-normal` half-space.
///
/// Useful when S4's seam-trim mechanism needs an explicit Boolean
/// `difference(slab)` instead of [`Manifold::trim_by_plane`] (e.g.,
/// to work around upstream issue #1516's coincident-face reassembly
/// caveat in scenarios that compose against the slab directly).
/// `slab_half_thickness_m` must exceed the target mesh's extent on
/// the `-normal` side of the plane.
///
/// Currently exercised only by S3's unit tests; the live S3 pipeline
/// uses [`MatingTransform::SeamTrim`] which routes through manifold3d's
/// direct `trim_by_plane` op. S4 may switch to slab-intersect if
/// downstream issues surface.
#[must_use]
pub fn build_half_space_slab(
    plane_point: Point3<f64>,
    normal: UnitVector3<f64>,
    slab_half_thickness_m: f64,
) -> Manifold {
    // Side dimension big enough to fully contain any iter-1 mesh.
    // Iter-1 cup pieces are < 200 mm on any axis; 1 m is comfortably
    // larger across all current and near-future cast geometries.
    const SLAB_SIDE_MM: f64 = 1000.0;

    let depth_mm = 2.0 * slab_half_thickness_m * METERS_TO_MM;
    let slab_z = Manifold::cube(SLAB_SIDE_MM, SLAB_SIDE_MM, depth_mm, /*center=*/ true);

    // Slab centre sits half-thickness back along `-normal` from
    // `plane_point`, so the +normal face of the slab lies on the
    // plane.
    let n = normal.into_inner();
    let offset_mm = slab_half_thickness_m * METERS_TO_MM;
    let center_mm = Point3::new(
        plane_point.x.mul_add(METERS_TO_MM, -(n.x * offset_mm)),
        plane_point.y.mul_add(METERS_TO_MM, -(n.y * offset_mm)),
        plane_point.z.mul_add(METERS_TO_MM, -(n.z * offset_mm)),
    );
    let iso = pose_from_z_axis(normal, center_mm);
    slab_z.transform(&affine_12_from_isometry(&iso))
}

/// Weld coincident vertices in `mesh` to within `tolerance_m`.
///
/// `tolerance_m` is in meters in cf-design coords, equivalent to
/// `tolerance_m × 1000` mm in mesh coords. Vertices whose quantized
/// keys collide are collapsed; faces are remapped and any face that
/// degenerates after collapse is dropped.
///
/// Live pipeline does NOT need this — [`crate::mesher::solid_to_mm_mesh`]
/// emits shared-index meshes by construction (recon §10 S2-C). This
/// helper exists for the S3 [`geometric_equivalence`] test helper
/// and any future STL-round-trip fixtures where a non-indexed STL
/// loader produces duplicated vertices that manifold3d rejects with
/// [`manifold3d::CsgError::ManifoldStatus`] (recon §11 item 3).
pub fn weld_in_place(mesh: &mut IndexedMesh, tolerance_m: f64) {
    // Mesh vertices are in mm; tolerance arrives in meters.
    let tolerance_mm = tolerance_m * METERS_TO_MM;
    debug_assert!(tolerance_mm > 0.0, "weld tolerance must be positive");
    let inv = 1.0 / tolerance_mm;

    let mut canonical: Vec<Point3<f64>> = Vec::with_capacity(mesh.vertices.len());
    let mut remap: Vec<u32> = Vec::with_capacity(mesh.vertices.len());
    let mut index_of: HashMap<(i64, i64, i64), u32> = HashMap::with_capacity(mesh.vertices.len());

    for v in &mesh.vertices {
        // Round-to-nearest quantization picks the closest grid cell;
        // ties go away from zero per `f64::round`. Symmetric around
        // zero.
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (v.x * inv).round() as i64,
            (v.y * inv).round() as i64,
            (v.z * inv).round() as i64,
        );
        let new_idx = *index_of.entry(key).or_insert_with(|| {
            // Welded vertex counts > u32::MAX would indicate a
            // multi-billion-vertex input, well outside cast-pipeline
            // scope. Clip rather than panic; downstream face
            // remapping will produce nonsense but the lib stays
            // panic-free.
            let idx = u32::try_from(canonical.len()).unwrap_or(u32::MAX);
            canonical.push(*v);
            idx
        });
        remap.push(new_idx);
    }

    let new_faces: Vec<[u32; 3]> = mesh
        .faces
        .iter()
        .map(|&[a, b, c]| [remap[a as usize], remap[b as usize], remap[c as usize]])
        .filter(|&[a, b, c]| a != b && b != c && c != a)
        .collect();

    mesh.vertices = canonical;
    mesh.faces = new_faces;
}

/// Construct an [`Isometry3`] whose rotation maps `+Z` to `axis` and
/// whose translation is `center_mm`.
///
/// Handles the antipodal case (`axis ≈ -Z`, where
/// [`Rotation3::rotation_between`] returns `None`) by picking a
/// deterministic perpendicular axis and applying a 180° rotation about
/// it. The branch is structural in `axis`, so two builders consuming
/// the same parent triple produce bit-equal rotations.
fn pose_from_z_axis(axis: UnitVector3<f64>, center_mm: Point3<f64>) -> Isometry3<f64> {
    let z = Vector3::z_axis();
    let rotation =
        Rotation3::rotation_between(&z.into_inner(), &axis.into_inner()).unwrap_or_else(|| {
            // Antipodal: axis is collinear with -Z. Pick X or Y as
            // the 180° pivot — deterministically biased so the
            // structural branch only depends on `axis.x`.
            let a = axis.into_inner();
            let perp = if a.x.abs() < 0.9 {
                Vector3::x_axis()
            } else {
                Vector3::y_axis()
            };
            Rotation3::from_axis_angle(&perp, std::f64::consts::PI)
        });
    Isometry3::from_parts(Translation3::from(center_mm.coords), rotation.into())
}

/// Pack a [`nalgebra::Isometry3<f64>`] into manifold3d's column-major
/// 4×3 affine `[f64; 12]` (recon §10 S2-B).
///
/// Layout per `manifold-csg-0.1.8/src/manifold.rs:561`:
///
/// ```text
/// | m[0] m[3] m[6] m[9]  |   col1 = X basis
/// | m[1] m[4] m[7] m[10] |   col2 = Y basis
/// | m[2] m[5] m[8] m[11] |   col3 = Z basis
///                              col4 = translation
/// ```
#[must_use]
pub(crate) fn affine_12_from_isometry(iso: &Isometry3<f64>) -> [f64; 12] {
    let r = iso.rotation.to_rotation_matrix();
    let t = iso.translation.vector;
    [
        r[(0, 0)],
        r[(1, 0)],
        r[(2, 0)], // X basis column
        r[(0, 1)],
        r[(1, 1)],
        r[(2, 1)], // Y basis column
        r[(0, 2)],
        r[(1, 2)],
        r[(2, 2)], // Z basis column
        t.x,
        t.y,
        t.z, // translation
    ]
}

/// Convert an [`IndexedMesh`] (mm world coords) into a [`Manifold`].
///
/// Widens `Vec<[u32; 3]>` faces into the flat `Vec<u64>` shape
/// manifold3d's `from_mesh_f64` expects. Returns
/// [`manifold3d::CsgError`] if the input is not manifold-clean
/// (recon §11 item 3; live pipeline is safe because
/// `solid_to_mm_mesh` emits shared-index meshes by construction).
fn indexed_mesh_to_manifold(mesh: &IndexedMesh) -> Result<Manifold, manifold3d::CsgError> {
    let vert_props: Vec<f64> = mesh.vertices.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
    let tri_indices: Vec<u64> = mesh
        .faces
        .iter()
        .flat_map(|&[i0, i1, i2]| [u64::from(i0), u64::from(i1), u64::from(i2)])
        .collect();
    Manifold::from_mesh_f64(&vert_props, 3, &tri_indices)
}

/// Convert a [`Manifold`] back into an [`IndexedMesh`].
///
/// manifold3d's `to_mesh_f64` returns shared-index data verbatim from
/// its internal `MeshGL64` form (recon §10 S2-C); no re-weld is
/// needed. Narrows `Vec<u64>` indices back to `Vec<[u32; 3]>` with a
/// `debug_assert!` on the u32 ceiling — iter-1 cup pieces are
/// ~50k verts, comfortably under `u32::MAX`.
fn manifold_to_indexed_mesh(manifold: &Manifold) -> IndexedMesh {
    let (vp, n_props, tri) = manifold.to_mesh_f64();
    debug_assert_eq!(
        n_props, 3,
        "we feed manifold3d 3 props per vertex; expected 3 back"
    );
    let vertices: Vec<Point3<f64>> = vp
        .chunks_exact(n_props)
        .map(|c| Point3::new(c[0], c[1], c[2]))
        .collect();
    debug_assert!(
        vertices.len() < u32::MAX as usize,
        "vertex count exceeds u32; iter-1 cup is ~50k, this is a runaway CSG"
    );
    // The debug_assert above guards the realistic ceiling; the
    // `unwrap_or(u32::MAX)` keeps the lib panic-free under any
    // hypothetical runaway and lets the resulting (broken) mesh
    // fail downstream F4 visibly instead of via a process crash.
    let faces: Vec<[u32; 3]> = tri
        .chunks_exact(3)
        .map(|c| {
            [
                u32::try_from(c[0]).unwrap_or(u32::MAX),
                u32::try_from(c[1]).unwrap_or(u32::MAX),
                u32::try_from(c[2]).unwrap_or(u32::MAX),
            ]
        })
        .collect();
    IndexedMesh::from_parts(vertices, faces)
}

/// Canonical 1 µm weld tolerance.
///
/// Used by STL round-trip fixtures (recon §11 item 3 + S1 ADR
/// finding 3) as the default quantization step for
/// [`geometric_equivalence`] and the canonical input to
/// [`weld_in_place`].
pub const WELD_TOLERANCE_M: f64 = 1.0e-6;

/// Test two meshes for geometric equivalence.
///
/// Two meshes are equivalent if they have the same vertex set
/// (quantized at [`WELD_TOLERANCE_M`]) and the same face set (each
/// face represented as its sorted vertex-rank triple, so winding
/// direction is ignored), regardless of insertion order. Reused by
/// the S3 plumbing-refactor falsification gate (plan §G2) and
/// S4-S6 feature-migration regressions: adding a pass-through
/// stage may shuffle internal vertex/face ordering even with
/// semantically identical output, so byte-identical STL comparison
/// is too strict.
///
/// **Winding agnostic.** A face `[a, b, c]` matches `[a, c, b]` —
/// manifold3d's `MeshGL64` output may renumber triangle cycles
/// (observed on iter-1 `funnel.stl` round-trip), and STL writers
/// can flip individual triangle windings depending on outward-
/// normal computation. The helper compares triangle *sets*; tests
/// that care about orientation must add a separate normal-consistency
/// assertion.
///
/// Returns `Ok(())` on equivalence, `Err(message)` on mismatch.
/// The error message names the first observable difference (vert
/// count, face count, or first mismatched canonical key) for fast
/// triage.
///
/// # Errors
///
/// Returns `Err(String)` with a short diagnostic describing the
/// first observable difference between the two meshes.
pub fn geometric_equivalence(lhs: &IndexedMesh, rhs: &IndexedMesh) -> Result<(), String> {
    if lhs.vertices.len() != rhs.vertices.len() {
        return Err(format!(
            "vertex count: {} vs {}",
            lhs.vertices.len(),
            rhs.vertices.len()
        ));
    }
    if lhs.faces.len() != rhs.faces.len() {
        return Err(format!(
            "face count: {} vs {}",
            lhs.faces.len(),
            rhs.faces.len()
        ));
    }
    let inv = 1.0 / (WELD_TOLERANCE_M * METERS_TO_MM);
    #[allow(clippy::cast_possible_truncation)]
    let quantize = |p: &Point3<f64>| -> (i64, i64, i64) {
        (
            (p.x * inv).round() as i64,
            (p.y * inv).round() as i64,
            (p.z * inv).round() as i64,
        )
    };

    let mut lhs_v: Vec<(i64, i64, i64)> = lhs.vertices.iter().map(quantize).collect();
    let mut rhs_v: Vec<(i64, i64, i64)> = rhs.vertices.iter().map(quantize).collect();
    lhs_v.sort_unstable();
    rhs_v.sort_unstable();
    if lhs_v != rhs_v {
        let first_diff = lhs_v
            .iter()
            .zip(&rhs_v)
            .position(|(a, b)| a != b)
            .unwrap_or(0);
        return Err(format!(
            "vertex set differs at sorted-position {first_diff}: {:?} vs {:?}",
            lhs_v.get(first_diff),
            rhs_v.get(first_diff),
        ));
    }

    // Winding-agnostic canonical form: sort each face's three
    // vertex ranks. A triangle is identified by its set of three
    // vertices; both possible windings ([a,b,c] and [a,c,b])
    // collapse to the same `[min, mid, max]`. Both sides re-key
    // against the shared sorted_keys list so vertex-array
    // permutations between lhs and rhs don't matter.
    let mut sorted_keys = lhs_v.clone();
    sorted_keys.dedup();
    let lhs_keys: Vec<(i64, i64, i64)> = lhs.vertices.iter().map(quantize).collect();
    let rhs_keys: Vec<(i64, i64, i64)> = rhs.vertices.iter().map(quantize).collect();
    let sorted_triple = |keys: &[(i64, i64, i64)], face: [u32; 3]| -> [usize; 3] {
        let mut t = [
            sorted_keys
                .binary_search(&keys[face[0] as usize])
                .unwrap_or(usize::MAX),
            sorted_keys
                .binary_search(&keys[face[1] as usize])
                .unwrap_or(usize::MAX),
            sorted_keys
                .binary_search(&keys[face[2] as usize])
                .unwrap_or(usize::MAX),
        ];
        t.sort_unstable();
        t
    };
    let mut lhs_f: Vec<[usize; 3]> = lhs
        .faces
        .iter()
        .map(|&face| sorted_triple(&lhs_keys, face))
        .collect();
    let mut rhs_f: Vec<[usize; 3]> = rhs
        .faces
        .iter()
        .map(|&face| sorted_triple(&rhs_keys, face))
        .collect();
    lhs_f.sort_unstable();
    rhs_f.sort_unstable();
    if lhs_f != rhs_f {
        let first_diff = lhs_f.iter().zip(&rhs_f).position(|(a, b)| a != b);
        return Err(format!(
            "face set differs (first sorted-position diff: {first_diff:?})"
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    // Workspace lint policy denies `unwrap()` in lib code but warns
    // in tests. Localized allow makes assertion failures read clearly
    // without panic-handling boilerplate at every probe.
    #![allow(clippy::unwrap_used)]

    use super::*;

    fn unit_cube_mesh() -> IndexedMesh {
        // Axis-aligned cube spanning [-1, 1] mm. 8 verts, 12 tris,
        // shared-index (so manifold3d accepts it without welding).
        let v = |x, y, z| Point3::new(x, y, z);
        let vertices = vec![
            v(-1.0, -1.0, -1.0), // 0
            v(1.0, -1.0, -1.0),  // 1
            v(1.0, 1.0, -1.0),   // 2
            v(-1.0, 1.0, -1.0),  // 3
            v(-1.0, -1.0, 1.0),  // 4
            v(1.0, -1.0, 1.0),   // 5
            v(1.0, 1.0, 1.0),    // 6
            v(-1.0, 1.0, 1.0),   // 7
        ];
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2], // -Z bottom
            [4, 5, 6],
            [4, 6, 7], // +Z top
            [0, 1, 5],
            [0, 5, 4], // -Y front
            [2, 3, 7],
            [2, 7, 6], // +Y back
            [1, 2, 6],
            [1, 6, 5], // +X right
            [0, 4, 7],
            [0, 7, 3], // -X left
        ];
        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn empty_transforms_pass_through_unchanged() {
        let mesh = unit_cube_mesh();
        let out = apply_mating_transforms(
            mesh.clone(),
            &[],
            CastTarget::MoldPiece {
                layer_index: 0,
                piece_side: crate::ribbon::PieceSide::Negative,
            },
        )
        .unwrap();
        // Empty Vec short-circuits to the input mesh verbatim — no
        // manifold round-trip, no vertex reordering.
        assert_eq!(out.vertices.len(), mesh.vertices.len());
        assert_eq!(out.faces.len(), mesh.faces.len());
        for (a, b) in out.vertices.iter().zip(&mesh.vertices) {
            assert_eq!(a, b);
        }
        for (a, b) in out.faces.iter().zip(&mesh.faces) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn empty_transforms_geometric_equivalence_round_trips() {
        // Sanity check on the test helper itself: a mesh equals itself.
        let mesh = unit_cube_mesh();
        geometric_equivalence(&mesh, &mesh).unwrap();
    }

    #[test]
    fn geometric_equivalence_detects_vertex_count_mismatch() {
        let lhs = unit_cube_mesh();
        let mut rhs = lhs.clone();
        rhs.vertices.push(Point3::new(2.0, 2.0, 2.0));
        let err = geometric_equivalence(&lhs, &rhs).unwrap_err();
        assert!(err.contains("vertex count"));
    }

    #[test]
    fn geometric_equivalence_ignores_vertex_array_permutation() {
        let lhs = unit_cube_mesh();
        // Reverse the vertex array AND remap faces — same geometry,
        // different ordering in memory.
        let mut rhs_vertices = lhs.vertices.clone();
        rhs_vertices.reverse();
        let n = u32::try_from(lhs.vertices.len()).unwrap() - 1;
        let rhs_faces: Vec<[u32; 3]> = lhs
            .faces
            .iter()
            .map(|&[a, b, c]| [n - a, n - b, n - c])
            .collect();
        let rhs = IndexedMesh::from_parts(rhs_vertices, rhs_faces);
        geometric_equivalence(&lhs, &rhs).unwrap();
    }

    #[test]
    fn cylinder_along_z_is_axis_aligned() {
        let parent = CylinderParent {
            center_m: Point3::origin(),
            axis: Vector3::z_axis(),
            half_length_m: 0.005, // 5 mm half-length
        };
        let m = build_cylinder_along_axis(&parent, 0.0015, 32);
        let bb = m.bounding_box_nalgebra().unwrap();
        // Cylinder of r=1.5mm, length=10mm at origin: bb roughly
        // [-1.5, -1.5, -5] → [1.5, 1.5, 5] mm.
        assert!((bb.0.x - -1.5).abs() < 0.1);
        assert!((bb.0.y - -1.5).abs() < 0.1);
        assert!((bb.0.z - -5.0).abs() < 0.05);
        assert!((bb.1.x - 1.5).abs() < 0.1);
        assert!((bb.1.y - 1.5).abs() < 0.1);
        assert!((bb.1.z - 5.0).abs() < 0.05);
    }

    #[test]
    fn cylinder_along_y_axis_pose_matches_axis() {
        // Same half-length and radius as above, but pointing along +Y.
        let parent = CylinderParent {
            center_m: Point3::new(0.030, 0.010, 0.0),
            axis: Vector3::y_axis(),
            half_length_m: 0.005,
        };
        let m = build_cylinder_along_axis(&parent, 0.0015, 32);
        let bb = m.bounding_box_nalgebra().unwrap();
        // Centred at (30, 10, 0) mm; radius 1.5 mm in XZ; half-length
        // 5 mm in Y. Expect:
        //  x ∈ [28.5, 31.5], y ∈ [5, 15], z ∈ [-1.5, 1.5].
        assert!((bb.0.x - 28.5).abs() < 0.1, "min.x: {}", bb.0.x);
        assert!((bb.0.y - 5.0).abs() < 0.05, "min.y: {}", bb.0.y);
        assert!((bb.0.z - -1.5).abs() < 0.1, "min.z: {}", bb.0.z);
        assert!((bb.1.x - 31.5).abs() < 0.1, "max.x: {}", bb.1.x);
        assert!((bb.1.y - 15.0).abs() < 0.05, "max.y: {}", bb.1.y);
        assert!((bb.1.z - 1.5).abs() < 0.1, "max.z: {}", bb.1.z);
    }

    #[test]
    fn cylinder_along_negative_z_axis_handles_antipodal_branch() {
        // axis = -Z is the rotation_between(z, axis) -> None case;
        // pose_from_z_axis falls through to the 180°-about-X branch.
        let parent = CylinderParent {
            center_m: Point3::origin(),
            axis: -Vector3::z_axis(),
            half_length_m: 0.005,
        };
        let m = build_cylinder_along_axis(&parent, 0.0015, 32);
        let bb = m.bounding_box_nalgebra().unwrap();
        // Same expected bb as +Z (cylinder is centred and symmetric).
        assert!((bb.0.x - -1.5).abs() < 0.1);
        assert!((bb.0.y - -1.5).abs() < 0.1);
        assert!((bb.0.z - -5.0).abs() < 0.05);
        assert!((bb.1.x - 1.5).abs() < 0.1);
        assert!((bb.1.y - 1.5).abs() < 0.1);
        assert!((bb.1.z - 5.0).abs() < 0.05);
    }

    #[test]
    fn affine_12_identity_iso_round_trips() {
        let iso = Isometry3::<f64>::identity();
        let a = affine_12_from_isometry(&iso);
        // X basis column = (1, 0, 0); Y = (0, 1, 0); Z = (0, 0, 1);
        // translation = (0, 0, 0). Identity rotation is exact in
        // f64 so bit-equality holds, but compare elementwise to
        // silence the float_cmp lint.
        let expected = [
            1.0_f64, 0.0, 0.0, // X basis
            0.0, 1.0, 0.0, // Y basis
            0.0, 0.0, 1.0, // Z basis
            0.0, 0.0, 0.0, // translation
        ];
        for (i, (got, want)) in a.iter().zip(&expected).enumerate() {
            assert!(
                (got - want).abs() < f64::EPSILON,
                "slot {i}: {got} vs {want}"
            );
        }
    }

    #[test]
    fn weld_in_place_collapses_within_tolerance() {
        let mut mesh = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                // Comfortably inside tolerance (0.3 µm = 0.0003 mm):
                // quantizes to grid cell 0 alongside vert 0.
                Point3::new(0.0003, 0.0, 0.0),
                // Beyond tolerance (2 µm = 0.002 mm): distinct cell.
                Point3::new(0.002, 0.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        weld_in_place(&mut mesh, WELD_TOLERANCE_M); // 1 µm = 0.001 mm
        assert_eq!(mesh.vertices.len(), 2);
        assert_eq!(mesh.faces.len(), 0); // [0,0,2] degenerates → dropped.
    }

    #[test]
    fn weld_preserves_unique_vertices() {
        let mut mesh = unit_cube_mesh();
        let n_before = mesh.vertices.len();
        let f_before = mesh.faces.len();
        weld_in_place(&mut mesh, WELD_TOLERANCE_M);
        assert_eq!(mesh.vertices.len(), n_before);
        assert_eq!(mesh.faces.len(), f_before);
    }

    /// S4: a `SeamTrim` with non-zero offset against a mesh that
    /// straddles the cut plane removes geometry from the opposite
    /// half and caps the cut with a flat face at the offset.
    ///
    /// Fixture: unit cube spanning [-1, +1] mm. Trim normal +Z,
    /// offset 0.5 mm → kept half is `z > 0.5`. Output should:
    /// - Have no vertices with z < 0.5 - tolerance.
    /// - Include vertices exactly at z = 0.5 (the cut cap).
    #[test]
    fn seam_trim_caps_at_offset_and_removes_opposite_half() {
        let cube = unit_cube_mesh();
        let target = CastTarget::MoldPiece {
            layer_index: 0,
            piece_side: crate::ribbon::PieceSide::Negative,
        };
        let trimmed = apply_mating_transforms(
            cube,
            &[MatingTransform::SeamTrim {
                normal: Vector3::z_axis(),
                offset_m: 0.0005, // 0.5 mm in mesh coords
            }],
            target,
        )
        .unwrap();

        assert!(
            !trimmed.vertices.is_empty(),
            "trimmed mesh must retain the +Z half"
        );
        // Geometric proxies for "trim worked" (vertex/face counts
        // can coincide with the input cube's 8/12 by manifold3d's
        // particular triangulation choice, so structural assertions
        // on the cut plane are the load-bearing checks here).
        // No vertices below the cut plane (within 1 µm of f64 noise).
        let min_z = trimmed
            .vertices
            .iter()
            .map(|v| v.z)
            .fold(f64::MAX, f64::min);
        assert!(
            min_z >= 0.5 - 1.0e-6,
            "no vertex should sit below the cut plane at z=0.5; got min.z = {min_z}",
        );
        // At least one vertex sits exactly on the cap plane.
        let any_at_cap = trimmed.vertices.iter().any(|v| (v.z - 0.5).abs() < 1.0e-6);
        assert!(
            any_at_cap,
            "trim must cap the cut with new vertices at z = 0.5",
        );
    }

    /// `SeamTrim` with an offset large enough to remove the entire
    /// input mesh produces an empty output (manifold3d's trim
    /// returns an empty manifold; downstream F4 surfaces the empty
    /// geometry).
    #[test]
    fn seam_trim_offset_past_mesh_extent_produces_empty_output() {
        let cube = unit_cube_mesh();
        let target = CastTarget::MoldPiece {
            layer_index: 0,
            piece_side: crate::ribbon::PieceSide::Negative,
        };
        let trimmed = apply_mating_transforms(
            cube,
            &[MatingTransform::SeamTrim {
                normal: Vector3::z_axis(),
                offset_m: 0.010, // 10 mm — cube max.z is 1 mm
            }],
            target,
        )
        .unwrap();
        assert!(
            trimmed.vertices.is_empty(),
            "trim past mesh extent should leave no geometry; got {} verts",
            trimmed.vertices.len(),
        );
    }

    /// S5 cross-piece determinism contract (recon §2): two
    /// independent calls to [`build_cylinder_along_axis`] with the
    /// same [`CylinderParent`] + same radius + same `segments` MUST
    /// produce bit-equal `to_mesh_f64` output (both vertex array
    /// and triangle index array). This is the load-bearing invariant
    /// the architectural fix rests on — pin and socket of a
    /// registration pair share the same parent triple, so their
    /// meshes are matched by construction across pieces rather than
    /// by tolerance.
    #[test]
    fn pin_cylinder_mesh_is_bit_equal_across_pieces() {
        // Synthetic parent triple — same shape as the M2 path in
        // production. The Y-axis pin matches the
        // `cylinder_along_y_axis_pose_matches_axis` test above; we
        // pick a non-trivial pose so the affine-12 packing is
        // exercised, not the Z-aligned trivial case.
        let parent = CylinderParent {
            center_m: Point3::new(0.030, 0.010, 0.000),
            axis: Vector3::y_axis(),
            half_length_m: 0.005,
        };
        let r_pin = 0.0015; // PinSpec::iter1 pin radius
        let segments = 32_u32;

        // Two independent builds → bit-equal output.
        let m_a = build_cylinder_along_axis(&parent, r_pin, segments);
        let m_b = build_cylinder_along_axis(&parent, r_pin, segments);
        let (verts_a, props_a, tris_a) = m_a.to_mesh_f64();
        let (verts_b, props_b, tris_b) = m_b.to_mesh_f64();

        assert_eq!(props_a, props_b);
        assert_eq!(
            verts_a, verts_b,
            "two builds of same parent + radius + segments must produce bit-equal vertex array"
        );
        assert_eq!(tris_a, tris_b, "…and bit-equal triangle index array");
        // Sanity check: not empty.
        assert!(!verts_a.is_empty());
        assert!(!tris_a.is_empty());
    }

    /// S5 pin/socket fit invariant — math-verified (recon §9 + plan §S5).
    ///
    /// Pin and socket parents share `center_m` and `axis`; only
    /// `half_length` and `radius_m` differ. With the same `segments`:
    /// - Socket radius = pin radius + `diametral_clearance_m / 2`
    ///   → socket diameter exceeds pin diameter by exactly
    ///   `diametral_clearance_m` (manifold3d cylinder primitive's
    ///   radial AABB extent scales linearly with the radius input;
    ///   the per-side radial gap equals `diametral_clearance / 2`).
    /// - Socket parent `half_length` = pin parent `half_length` +
    ///   `axial_clearance_m / 2` → the socket *cylinder primitive*
    ///   extends `axial_clearance_m / 2` past the pin cylinder on
    ///   each axial face (symmetric `/2` convention matching the
    ///   diametral budget). The per-piece `SeamTrim` downstream clips
    ///   each side's near-seam half independently, so the AWAY-from-
    ///   seam extension is the workshop-meaningful component
    ///   (modest bottom-of-pocket relief beyond a pin-half-length-
    ///   only socket); the near-seam extension is removed by trim.
    ///
    /// Locks the clearance arithmetic at engine precision rather
    /// than via cf-view smoke per
    /// `feedback_math_verify_geometric_contracts`: manifold3d's
    /// cylinder bounding-box is bit-precise on f64, so the fit
    /// invariant should hold to ≤ 1 µm.
    #[test]
    fn pin_and_socket_fit_invariant() {
        let parent_pin = CylinderParent {
            center_m: Point3::origin(),
            axis: Vector3::z_axis(),
            half_length_m: 0.005, // 5 mm
        };
        let r_pin_m = 0.0015; // 1.5 mm
        let diametral_clearance_m = 0.00020; // 0.20 mm
        let axial_clearance_m = 0.00050; // 0.50 mm
        let segments = 32_u32;

        // Socket parent extends half-length by axial_clearance / 2;
        // socket radius extends pin radius by diametral_clearance / 2.
        let parent_socket = CylinderParent {
            center_m: parent_pin.center_m,
            axis: parent_pin.axis,
            half_length_m: parent_pin.half_length_m + axial_clearance_m / 2.0,
        };
        let r_socket_m = r_pin_m + diametral_clearance_m / 2.0;

        let pin = build_cylinder_along_axis(&parent_pin, r_pin_m, segments);
        let socket = build_cylinder_along_axis(&parent_socket, r_socket_m, segments);

        let (pin_min, pin_max) = pin.bounding_box_nalgebra().unwrap();
        let (sock_min, sock_max) = socket.bounding_box_nalgebra().unwrap();

        // Radial extent (X, Y): socket bigger than pin by exactly
        // diametral_clearance / 2 on each side (= diametral_clearance
        // across the diameter).
        let half_diametral_mm = (diametral_clearance_m / 2.0) * METERS_TO_MM;
        let tol_mm = 1.0e-3; // 1 µm — manifold3d f64 precision
        let radial_gaps = [
            ("+X", sock_max.x - pin_max.x),
            ("-X", pin_min.x - sock_min.x),
            ("+Y", sock_max.y - pin_max.y),
            ("-Y", pin_min.y - sock_min.y),
        ];
        for (name, gap) in radial_gaps {
            assert!(
                (gap - half_diametral_mm).abs() < tol_mm,
                "socket-vs-pin radial gap on {name} should equal diametral/2 = \
                 {half_diametral_mm:.6} mm; got {gap:.6} mm",
            );
        }

        // Axial extent (Z): socket cylinder bigger than pin cylinder
        // by exactly axial_clearance / 2 on EACH end (symmetric since
        // both cylinders are centred at the same parent.center; per-
        // piece SeamTrim later clips the near-seam half of each so
        // only the away-from-seam extension survives in the per-piece
        // STL — see `pin_transform_for_side` doc for the workshop
        // semantics).
        let half_axial_mm = (axial_clearance_m / 2.0) * METERS_TO_MM;
        let axial_gap_z_max = sock_max.z - pin_max.z;
        let axial_gap_z_min = pin_min.z - sock_min.z;
        assert!(
            (axial_gap_z_max - half_axial_mm).abs() < tol_mm,
            "socket-vs-pin axial gap on +Z should equal axial/2 = \
             {half_axial_mm:.6} mm; got {axial_gap_z_max:.6} mm",
        );
        assert!(
            (axial_gap_z_min - half_axial_mm).abs() < tol_mm,
            "socket-vs-pin axial gap on -Z should equal axial/2 = \
             {half_axial_mm:.6} mm; got {axial_gap_z_min:.6} mm",
        );
    }

    #[test]
    fn half_space_slab_has_face_at_plane_point() {
        // Slab oriented with +normal = +Z, plane_point at origin.
        // The slab's +Z face should lie on z = 0; its body extends
        // 2 mm into -Z.
        let slab = build_half_space_slab(
            Point3::origin(),
            Vector3::z_axis(),
            0.001, // 1 mm half-thickness
        );
        let bb = slab.bounding_box_nalgebra().unwrap();
        assert!(
            (bb.1.z - 0.0).abs() < 0.01,
            "max.z should be ~0: {}",
            bb.1.z
        );
        assert!(
            (bb.0.z - -2.0).abs() < 0.01,
            "min.z should be ~-2 mm: {}",
            bb.0.z
        );
    }
}
